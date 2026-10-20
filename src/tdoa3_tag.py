import struct 
import scipy.optimize 
import numpy as np
import time 
import csv
import math
from datetime import datetime
from collections import deque, defaultdict

from dw1000 import DW1000
from config import (
    ANCHORS,
    LOCALIZATION_FREQUENCY,
    SAVE_TO_CSV,
    CSV_SAVE_INTERVAL_SECONDS,
)

RECORDING_DURATION = 1/LOCALIZATION_FREQUENCY # How long we listen before generating an update 
SPEED_OF_LIGHT = 299_702_547 # m/s 
HISTORY = 8        # How many messages we record per anchor before overwritting them. Testing showed lag to be up to 4 messages.
SEQ_MASK = 0x7F    # TDoA3 seq is 7 bits
# NOTE: TODO empirical tuning of these parameters in the future
MAX_ALPHA_ERROR_PPM = 150           # Guard against large alpha coefficients 
MAX_ALPHA_MEDIAN_DEVIATION_PPM = 10 # Guard against large alpha changes through time 
ALPHA_HISTORY = 8                   # How many alphas we keep in the history for the guard comparison

# ANTENNA DELAY FOR ANCHORS FROM BC FIRMWARE - USED FOR TOF VALIDATION
ANTENNA_OFFSET_M = 154.6
RADIO_TICK_FREQUENCY = 499.2e6 * 128
ANTENNA_DELAY_TICKS = ANTENNA_OFFSET_M * RADIO_TICK_FREQUENCY / 299_792_458.0

def wrap_diff(a, b, bits:int, signed:bool=True):
    """
    Computes (a - b) with correct wraparound for a fixed-width hardware counter. Anchor clocks are capped at 32bit from BC firmware. Tag clock is 40bit based from DW1000 class.

    ARGS:
    - bits: width of the counter this value came from.
        - 32 for anything read from an anchor
        - 40 for anything read from the tag own's clock 
    - signed: whether the true result can meaningfully be negative.
        - False: use when a and b come from the SAME clock
          taken in known chronological order (e.g. two
          consecutive tx_ts values from one anchor's own broadcasts, or
          two consecutive rx values the tag recorded for the same
          anchor). The true delta is always >= 0 by construction, so we
          just need it correctly wrapped -- reinterpreting as signed
          could flip a large-but-legitimate positive delta into a
          bogus negative one.
        - True: use when a and b come from DIFFERENT clocks,
          or from the same clock but with no guarantee which is larger
          (e.g. an anchor's tx_ts vs. a remote anchor's rx-derived value;
          or the tag's rx timestamps for two different anchors). These
          deltas represent clock offsets / propagation differences and
          can legitimately go either way, so the wrapped result is
          reinterpreted as signed (values in the upper half of the
          range become negative).

    Signed results are only unambiguous when the true delta's magnitude
    is less than half the counter's range. The caller must validate that
    assumption when timestamps can be separated by longer intervals.
    """
    mask = (1 << bits) - 1
    d = ((a & mask) - (b & mask)) & mask
    if signed and d >= (1 << (bits - 1)):
        d -= 1 << bits
    return d

def unwrap_delta(raw_delta: int, reference_delta: float, bits: int) -> int:
    """Choose the modular alias closest to an independent delta estimate.

    A wrapped counter difference may have multiple valid values separated by
    its modulus. Unlike a signed ``wrap_diff``, this uses the reference estimate
    to select the right alias, including differences beyond half the counter
    range.
    """
    modulus = 1 << bits
    wraps = round((reference_delta - raw_delta) / modulus)
    return raw_delta + wraps * modulus

class TDOA3_Tag: 
    """
    A DW1000 based UWB Tag that works with Bitcraze anchors following their TDoA3 protocol. 
    Only use with a context manager for safety. 

    ARGS: 
    - id: The tag's ID 
    - bus: SPI bus for the DW1000 connection
    - cs:  SPI chip select for the DW1000 connection
    """
    def __init__(self, id, bus=0, cs=0): 
        self._dw = DW1000(bus, cs, channel=2, PRF=64, bitrate=6, preamble_length=128, preamble_code=9, smart_tx_power=True, tx_power_settings=None)
        self.id = id 

        self.history = defaultdict(lambda: deque(maxlen=HISTORY))   # anchor_id -> msgs, oldest first
        self.fresh_anchors = set()                                  # anchors that got a new msg in the last batch
        self.alpha_history = defaultdict(lambda: deque(maxlen=ALPHA_HISTORY)) 

        # Anchor positions can be pre-defined in config.py to overwrite firmware values. 
        # If a position is not pre-defined, it will be populated with the firmware-defined values published by the anchor in it's messages.
        self.anchors = dict(ANCHORS) # Copy of the config dict. Format {anchor_id: (x,y,z)}
        self.TOF_ref_anchors = {}    # Holds theoretical TOF between anchors. Used for outlier detection. {anchor_id: {other_anchor_id:TOF}}
        self.update_ref_TOFs()       # Computing TOF for all config.py anchors 
        self._csv_rows = []
        self._last_csv_save = time.monotonic()
        # Guessing an initial position. Will serve as starting point for subsequent optimization in .run() 
        if len(self.anchors)>=1: 
            anchor_pos = np.array( [pos_tuple for pos_tuple in self.anchors.values()] )
            initial_pos = np.mean(anchor_pos, axis=0) # Geometric centroid of the anchors 
            self.initial = initial_pos # TEMP TODO NOTE testing - safe anchoring of solver  
            #self.position = (initial_pos[0], initial_pos[1], initial_pos[2])
        else: 
            pass 
            #self.position = (0,0,0) # If we don't have any info, try (0,0,0). NOTE todo in future: evaluate how this actually performs. If needed, shift this to on first listen when can use firmware positions of anchors to help.     
    
    def __enter__(self):
        return self 
    def __exit__(self, exc_type, exc_val, exc_tb):
        try:
            self._save_csv_rows()
        finally:
            self._dw.close()

    def update_ref_TOFs(self): 
        """
        Updates the data of TOF_ref_anchors dict in case new anchors were added. 
        This is punctually computationaly intensive and memory-innefficient, but overall is readable and 
        avoids calculating distances over and over while running the algorithm. 
        """
        # TOF_ref_anchors has shape: {anchor_id: {other_anchor_id: TOF}} where TOF is in DW1000 clock ticks 
        for anchor, pos in self.anchors.items(): 
            # If the anchor has no data on others, creating an empty dict 
            self.TOF_ref_anchors[anchor] = self.TOF_ref_anchors.get(anchor, {})
            # Compute TOF for all other anchors for which we do not have data on 
            for remote_anchor, remote_pos in self.anchors.items(): 
                if remote_anchor==anchor or remote_anchor in self.TOF_ref_anchors[anchor]: 
                    continue 
                else: 
                    # NOTE TODO currently expecting meters as anchor position, define this properly 
                    tof = int((np.linalg.norm(np.array(remote_pos)-np.array(pos))/SPEED_OF_LIGHT)/DW1000.TIME_UNIT) 
                    self.TOF_ref_anchors[anchor][remote_anchor] = tof 

    def validate_measured_TOF(self, a1:int, a2:int, z_tof:int)->bool: 
        """
        Compares the reported TOF, including the firmware antenna delay, with the geometric anchor-to-anchor TOF.
        """
        tolerance_m = 0.3
        tolerance_ticks = tolerance_m / (SPEED_OF_LIGHT * DW1000.TIME_UNIT)
        expected_tof = self.TOF_ref_anchors[a1][a2] + ANTENNA_DELAY_TICKS
        return abs(z_tof - expected_tof) <= tolerance_ticks
    
    def run(self, enable_print:bool): 
        """
        Starts an infinite loop that runs continuous localization of the tag. 

        High level loop structure: 
        1. Gather raw packets from anchors for the duration specified by `LOCALIZATION_FREQUENCY` in `config.py` 
        2. Aggregating raw data into per anchor dict: `{anchor_id: [msg, ..., msg]}` where `msg = {seq, rx, tx, remote_data}`
        3. Computing the most TDOA pairs possible from the aggregated data 
        4. Attempting multilateration with the TDOA pairs 
        5. Printing/Saving result  

        Localization results are buffered and periodically appended to CSV when
        SAVE_TO_CSV is enabled. Pending rows are flushed when this loop exits.
        """
        try:
            while True:
                # Gathering anchor data
                raw_data = []
                t1 = time.perf_counter()
                while (time.perf_counter()-t1)<RECORDING_DURATION:
                    pkt, rx = self._dw.listen(ranging=True)
                    if pkt: # rx is None if and only if pkt is None per DW1000 class methods.
                        # We don't process the pkts yet to ensure we get as much info as possible
                        raw_data.append((pkt, rx))
                # Parsing & organizing raw data into an organized format
                # Using a dict {anchor_id: deque_of_msgs[{rx, tx, remote_data}]}
                aggregated_data = self.aggregate_raw_pkts(raw_data)
                # Processing the data into TDOA-anchor pairs
                tdoa_anchor_data = self.process_anchor_data(aggregated_data)
                # Calculating position
                print(f"Sending a processed list of size {len(tdoa_anchor_data)}")
                position = self.multilaterate(tdoa_anchor_data)
                if enable_print:
                    print(f"[INFO] -----> New position estimated: {position}")
                if SAVE_TO_CSV:
                    coordinates = position if position is not None else ("", "", "")
                    self._csv_rows.append((
                        datetime.now().astimezone().isoformat(timespec="milliseconds"),
                        len(self.anchors),
                        len(tdoa_anchor_data),
                        *coordinates,
                    ))
                    if (len(self._csv_rows) >= max(1,math.ceil(LOCALIZATION_FREQUENCY * CSV_SAVE_INTERVAL_SECONDS),)
                        or time.monotonic() - self._last_csv_save >= CSV_SAVE_INTERVAL_SECONDS
                    ):
                        self._save_csv_rows()
        finally:
            self._save_csv_rows()

    def _save_csv_rows(self):
        if not SAVE_TO_CSV or not self._csv_rows:
            return

        filename = f"tdoa3_tag_{self.id}.csv"
        with open(filename, "a", newline="", encoding="utf-8") as csv_file:
            writer = csv.writer(csv_file)
            csv_file.seek(0, 2)
            if csv_file.tell() == 0:
                writer.writerow((
                    "timestamp",
                    "known_anchors",
                    "processed_data_pairs",
                    "position_x",
                    "position_y",
                    "position_z",
                ))
            writer.writerows(self._csv_rows)
        self._csv_rows.clear()
        self._last_csv_save = time.monotonic()

    def save_runs(self, runs_to_save:int): 
        """
        For synthetic testing: saves raw RX data recorded for runs_to_save intervals of RECORDING_DURATION seconds. 
        """
        saver = [] # list of lists of raw data to save
        while True: 
            # Gathering anchor data 
            raw_data = [] 
            t1 = time.perf_counter() 
            while (time.perf_counter()-t1)<RECORDING_DURATION: 
                pkt, rx = self._dw.listen(ranging=True) 
                if pkt: # rx is None if and only if pkt is None per DW1000 class methods.
                    # We don't process the pkts yet to ensure we get as much info as possible 
                    raw_data.append((pkt, rx))
            saver.append(raw_data)
            if len(saver)==runs_to_save: 
                import pickle 
                with open(f"{runs_to_save}_1s_raw_datas.pkl", "wb") as file:
                    pickle.dump(saver, file)
                print(f"SAVED {len(saver)} ITEMS IN SAVER LIST. {type(saver[0])=}")
                return 
    
    def aggregate_raw_pkts(self, data:list[tuple[int, int]])->dict[int, list[dict]]: 
        """
        Takes a list of received raw packets [(raw_packet, local_rx_time)] and appends them to the
        per-anchor history (last HISTORY messages, oldest first, gaps allowed).

        RETURNS:
        - {anchor_id: [msg, ..., msg]} where msg = {seq, rx, tx, remote_data}
        - self.fresh_anchors is set to the anchors that received a new message in this batch
        """
        self.fresh_anchors = set()
        for pkt, rx in data:
            interpreted_msg = self.interpret_anchor_msg(pkt)
            if interpreted_msg is None:
                continue
            anchor_id, seq, tx_ts, remote_anchors, anchor_pos = interpreted_msg
            # Adding anchor_pos to our references if it doesn't exist
            if anchor_id not in self.anchors:
                if anchor_pos is None:
                    print(f"[WARNING] ANCHOR {anchor_id} DOES NOT HAVE A POSITION CONFIGURED IN THE FIRMWARE OR config.py.")
                    print("IT WILL BE IGNORED FOR ALL COMPUTATIONS. Add it's position through firmware or config.py to enable it.")
                    continue
                else: 
                    print(f"[INFO] Anchor {anchor_id} did not have a position configured in config.py, it's firmware value of {anchor_pos} will be used.")
                self.anchors[anchor_id] = (anchor_pos[0], anchor_pos[1], anchor_pos[2])
                self.update_ref_TOFs()
            self.history[anchor_id].append( {'seq': seq, 'rx': rx, 'tx': tx_ts, 'remote_data': remote_anchors} )
            self.fresh_anchors.add(anchor_id)
        return {a: list(h) for a, h in self.history.items()}

    def process_anchor_data(self, aggregated_data: dict[int, list[dict]])->list[tuple]:
        """
        Computes TDOA pairs w.r.t. a single reference anchor. Every anchor with a fresh message is tried
        as reference; the one that yields the most valid pairs is used.

        RETURNS: List of shape [(TDOA, ref_pos, remote_pos), ...]
        """
        best = []
        for ref_anchor in sorted(self.fresh_anchors):
            results = self._tdoa_from_ref(aggregated_data, ref_anchor)
            if len(results) > len(best):
                best = results
        return best

    def _tdoa_from_ref(self, aggregated_data, ref_anchor):
        results = []
        ref_data = aggregated_data[ref_anchor]
        # Need 2 fresh consecutive messages from the ref_anchor to establish alpha 
        if len(ref_data) < 2: 
            return results
        prev, cur = ref_data[-2], ref_data[-1]
        if (cur['seq'] - prev['seq']) & SEQ_MASK != 1:
            return results      # missed a packet in between -> clock ratio span too long to trust
        # Computing alpha (clock correction coefficient from ref_anchor -> tag ticks) 
        delta_tx_prime = wrap_diff(cur['tx'], prev['tx'], bits=32, signed=False)
        delta_rx_prime = wrap_diff(cur['rx'], prev['rx'], bits=40, signed=False)
        try:
            alpha = delta_rx_prime / delta_tx_prime  
        except ZeroDivisionError:
            return results
        # Reject if alpha is too big 
        if abs(alpha - 1) > MAX_ALPHA_ERROR_PPM * 1e-6:
            return results
        recent_alphas = self.alpha_history[ref_anchor]
        # Reject if alpha changed too much recently 
        if (
            len(recent_alphas) >= 3
            and abs(alpha - np.median(recent_alphas))
            > MAX_ALPHA_MEDIAN_DEVIATION_PPM * 1e-6
        ):
            return results
        recent_alphas.append(alpha)
        # Computing TDOA with respect to each remote anchor in the reference anchor's received messages 
        for remote_anchor, (r_seq, r_rx, r_tof) in cur['remote_data'].items():
            if remote_anchor not in aggregated_data:
                continue
            if r_tof is None:
                continue
            # The tag must have independently heard the exact packet the ref reported (same SEQ)
            remote_msg = next((m for m in aggregated_data[remote_anchor] if m['seq'] == r_seq), None)
            if remote_msg is None:
                continue
            if not self.validate_measured_TOF(ref_anchor, remote_anchor, r_tof):
                continue
            remote_tx = wrap_diff(r_rx, r_tof, bits=32, signed=False)
            delta_tx_unsigned = wrap_diff(cur["tx"], remote_tx, bits=32, signed=False)
            delta_rx = wrap_diff(cur['rx'], remote_msg['rx'], bits=40, signed=True)
            delta_tx = unwrap_delta(
                delta_tx_unsigned,
                delta_rx / alpha,
                bits=32,
            )
            # Distance between anchors can never be > than distance passing through tag
            tdoa_ticks = delta_rx - alpha * delta_tx
            if abs(tdoa_ticks) > self.TOF_ref_anchors[ref_anchor][remote_anchor]:
                continue
            TDoA = tdoa_ticks * DW1000.TIME_UNIT
            results.append((TDoA, self.anchors[ref_anchor], self.anchors[remote_anchor]))
        return results

    @staticmethod
    def interpret_anchor_msg(msg:list[int])->tuple[int, int, int, dict, tuple|None]:
        """
        Takes a raw intercepted message between anchors and extracts the info we need from it. 

        RETURNS: 
        - anchor_id: ID of the anchor that sent the message
        - seq:       SEQ of the transaction 
        - tx_ts:     Transmit timestamp of the message in 32-bit based DW1000 ticks
        - remote_anchors: dict of shape {remoteAnchorIds: (seq, rx_ts, r_tof)} rx (32-bit based) and tof (16bit) in ticks 
        - anchor_pos: (x,y,z) position of the anchor or None 

        INFO: 
        Anchors communicate between themselves following the Bitcraze TDOA3 protocol.
        Messages have the following format: BC_HEADER + TDOA3 Header + TDOA3 Info + Anchor Position 
        NOTE: I am writing the default bytes in MSB order for readibility, but they are transmitted LSB order 
        BC_HEADER = 0xDC41 + 0x00 + 0x0000 + 0xBCCF0000000000FF + 0xBCCF{6 bytes source ID} 
        TDOA3_HEADER = 0x30 + {SEQ(1byte)} + {4 bytes tx timestamp} + {0x0 to 0x8 remoteCount} 
        TDOA3_INFO = remoteCount * [{1 byte remoteAnchorID} + {1bit hasDistance+7bits seq} + {4 bytes rx timestamp} + {0||2 bytes distance}]
        ANCHOR_POS = If available, last 14 bytes (0xF0 for LPP short + 0x01 for anchor pos + 4bytes x 3 floats for the actual position data) 
        """
        BC_TDOA3_DEST_HEADER = [0x41, 0xdc, 0x0,  0x0, 0x0, 0xff, 0x0, 0x0, 0x0, 0x0, 0x0, 0xcf, 0xbc] 
        ALTERNATIVE_TDOA3_DEST_HEADER = [0x0, 0x0] + BC_TDOA3_DEST_HEADER[2:] # TODO TEMP ONLY HERE UNTIL ALL ANCHORS GO ON THE SAME FIRMWARE
        # Checking if the message has the expected header (BC format and general broadcast to 0xFF + TDOA3 header 0x30) 
        if (msg[:13] != BC_TDOA3_DEST_HEADER and msg[:13] != ALTERNATIVE_TDOA3_DEST_HEADER) or msg[21] != 0x30: 
            return 
        ## Extracting important info from message 
        # Source anchor ID 
        anchor_id = int.from_bytes(bytes(msg[13:19]), byteorder='little') 
        # SEQ 
        seq = msg[22] 
        # TX Timestamp
        tx_ts = int.from_bytes(bytes(msg[23:27]), 'little') 
        # remoteCount 
        n_other_anchors = msg[27]
        # Processing remote anchors data if they are present
        remote_anchors = {} # {id: (seq, rx_ts, r_tof)}
        if n_other_anchors!=0: 
            start_idx = 28 
            for _ in range(n_other_anchors): 
                id_idx = start_idx 
                r_id = msg[id_idx]
                has_dist = msg[id_idx+1]>>7
                r_seq    = msg[id_idx+1]&SEQ_MASK
                rx_ts    = int.from_bytes(bytes(msg[id_idx+2:id_idx+6]), 'little') 
                r_tof    = int.from_bytes(bytes(msg[id_idx+6:id_idx+8]), 'little') if has_dist else None # NOTE tof is in radio ticks

                remote_anchors[r_id] = (r_seq, rx_ts, r_tof)
                start_idx += 6 + (2 if has_dist else 0) # next anchor index will depend on if this info had a distance
        # Source anchor position if available. Since it's position depends on remoteAnchors, we index from the end 
        anchor_pos_packet = msg[-14:] 
        anchor_pos = None 
        if anchor_pos_packet[0:2] == [0xF0, 0x01]: # Expected header for a LPP short packet with anchor position 
            anchor_pos = struct.unpack('<fff', bytes(anchor_pos_packet[2:]))

        return anchor_id, seq, tx_ts, remote_anchors, anchor_pos  

    def multilaterate(self, tdoa_data:list[tuple])->tuple[int,int,int]:
        """
        Use TDoA multilateration to estimate the tag's position. Automatically updates the "position" attribute.\n
        Returns the estimated position as (x,y,z) coordinates\n 
        ARGS: 
        - tdoa_data: A list of tuples representing independent measurements (minimum 3) 
            - tuple format: (TDOA_measure_seconds, anchor_pos_tuple_1, anchor_pos_tuple_2)

        INFO:\n
        Position is estimated through non-linear least squares.
        The residual function is defined as (TDOA corresponding to the current estimated position - measured TDOA by the tag) \n
        The TDOA values given should always be with respect to (tag->a1 - tag->a2) 
        """
        if len(tdoa_data)<3:
            #print("[INFO] multilaterate() was called with less than 3 measurements, cannot converge.")
            return None 
        def tdoa_residuals(current_pos_estimate:np.ndarray, tdoa_data):
            # Residual function: (TDOA corresponding to an estimate - measured TDOA) 
            # IMPORTANT: TDOAs are assumed to be with respect to anchor 1 - anchor 2. 
            residuals = [] 
            for tdoa, a1_pos, a2_pos in tdoa_data: 
                estimated_delta = np.linalg.norm(current_pos_estimate - a1_pos) - np.linalg.norm(current_pos_estimate - a2_pos)
                measured_delta  = tdoa*SPEED_OF_LIGHT # result in meters 
                residuals.append(estimated_delta-measured_delta)
            return np.array(residuals) 
        # Non-linear least squares, using last known position as initial guess 
        # Using huber to cap the influence of unexpected outliers
        # Using f_scale of 0.7m for now - to be tuned more in the future 
        result = scipy.optimize.least_squares(tdoa_residuals, np.array(self.initial), args=(tdoa_data,),
                                               method='trf', loss='huber', f_scale=0.7)  
        if result.success: 
            self.position = tuple(result.x) 
            return self.position 
        else: 
            print(f"[ERROR] Least squares failed to converge {result.message}")
            return None 