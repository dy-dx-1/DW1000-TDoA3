import struct 
import scipy.optimize 
import numpy as np
import time 

from collections import deque, defaultdict
HISTORY = 8        # observed lag is up to 4 packets from testing 
SEQ_MASK = 0x7F    # TDoA3 seq is 7 bits

from config import ANCHORS 

SPEED_OF_LIGHT = 299_702_547 # m/s 

# Mock DW1000 to simulate time unit without needing spidev 
class DW1000: 
    TIME_UNIT = 1.5650040064102565e-11


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

    Only valid if the true (unwrapped) magnitude of a - b is less than
    half the counter's range: 2**31 ticks (~33.55 ms) for bits=32,
    2**39 ticks (~8.6 s) for bits=40. Both hold comfortably for the
    quantities we compute here.
    """
    mask = (1 << bits) - 1
    d = (a - b) & mask
    if signed and d >= (1 << (bits - 1)):
        d -= (1 << bits)
    return d

class TDOA3_Tag: 
    """
    Synthetic tag, doesn't connect to DW1000 as uses recorded data 
    """
    def __init__(self, id, bus=0, cs=0): 
        #self._dw = DW1000(bus, cs, channel=2, PRF=64, bitrate=6, preamble_length=128, preamble_code=9, smart_tx_power=True, tx_power_settings=None)
        self.id = id 

        self.history = defaultdict(lambda: deque(maxlen=HISTORY))   # anchor_id -> msgs, oldest first
        self.fresh_anchors = set()                                  # anchors that got a new msg in the last batch

        # Anchor positions can be pre-defined in config.py to overwrite firmware values. 
        # If a position is not pre-defined, it will be populated with the firmware-defined values published by the anchor in it's messages.
        self.anchors = dict(ANCHORS) # Copy of the config dict. Format {anchor_id: (x,y,z)}
        self.TOF_ref_anchors = {}    # Holds theoretical TOF between anchors. Used for outlier detection. {anchor_id: {other_anchor_id:TOF}}
        self.update_ref_TOFs() 
        # Guessing an initial position. Will serve as starting point for subsequent optimization in .run() 
        if len(self.anchors)>=1: 
            anchor_pos = np.array( [pos_tuple for pos_tuple in self.anchors.values()] )
            initial_pos = np.mean(anchor_pos, axis=0) # Geometric centroid of the anchors 
            self.position = (initial_pos[0], initial_pos[1], initial_pos[2])
        else: 
            self.position = (0,0,0) # If we don't have any info, try (0,0,0). NOTE todo in future: evaluate how this actually performs. If needed, shift this to on first listen when can use firmware positions of anchors to help. 
        
    def __enter__(self):
        return self 
    def __exit__(self, exc_type, exc_val, exc_tb):
        #self._dw.close() 
        pass 

    def update_ref_TOFs(self): 
        """
        Updates the data of TOF_ref_anchors dict in case new anchors were added. 
        This is punctually computationaly intensive and memory-innefficient, but overall is readable and 
        allows us to avoid calculating distances over and over while running the algorithm. 
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
        Compares the measured TOF between anchors against the geometric expected value to determine 
        if the exchange is good enough to be used to compute TDOA. This serves as a first line of defense against
        NLOS/multipath readings. 
        """
        return True # TODO REMOVE, ONLY HERE FOR ROUGH ROOM TESTING 
        # NOTE TODO formalize and calibrate this properly, currently using 50cm 
        tolerance = 0.5/SPEED_OF_LIGHT/DW1000.TIME_UNIT 
        expected_tof = self.TOF_ref_anchors[a1][a2] 
        if abs(z_tof-expected_tof)<=tolerance: 
            return True 
        else: 
            return False

    def run_synthetic(self, enable_print:bool): 
        """
        Copy of run() but uses the synthetic data recorded at the lab to debug
        Parts of the code that are different ONLY for synthetic running are clearly 
        marked by ############# above and below. 
        Correct approx position by TWR: [cm] (243, 263, 10) (Z is truly more like 40!)
        """
        ################################## 
        import numpy as np 
        avg = [] 
        # The data from the file is 10 items of 1s recordings 
        import pickle 
        with open("C:/Users/Nicolas/Documents/Github/DW1000-TDoA3/src/10_1s_raw_datas.pkl", "rb") as file:
            saved_runs = pickle.load(file) 
        for raw_data in saved_runs: 
        ##################################
            # Now that out of gathering info loop, parsing & organizing the data 
            # Using a dict {anchor_id: [{rx_t-1, tx_t-1, remote_data_t-1}, {rx, tx, remote_data}]}
            # Only keep the latest 2 messages for any anchor (need fresh data for calculations)
            aggregated_data = self.aggregate_raw_pkts(raw_data) 
            # Processing the data into TDOA-anchor pairs 
            tdoa_anchor_data = self.process_anchor_data(aggregated_data)     
            print(f"PURE AGGREGATED: {len(aggregated_data)}  |  PROCESSED: {len(tdoa_anchor_data)}")           
            # Calculating position 
            # use some kind of array manipulation to compute all the tdoas efficiently? 
            ###########################################################################
            #print(f"Passing {len(tdoa_anchor_data)} TDOA measures to multilaterate")
            position = self.multilaterate(tdoa_anchor_data)
            if enable_print: 
                if position: 
                    position = tuple(round(i*100) for i in position) 
                    avg.append(position) 
                #print(f"[INFO] -----> New position estimated: {position}")
        avg = np.array(avg) 
        print(f"Average: {np.mean(avg, axis=0)}")
        print(f"Median: {np.median(avg, axis=0)}")
        print(f"STD dev: {np.std(avg, axis=0)}")
    
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
                    print(f"[ERROR] ANCHOR {anchor_id} DOES NOT HAVE A POSITION CONFIGURED IN THE FIRMWARE OR config.py.")
                    print("IT WILL BE IGNORED FOR ALL COMPUTATIONS. Add it's position through firmware or config.py to enable it.")
                    continue
                self.anchors[anchor_id] = (anchor_pos[0], anchor_pos[1], anchor_pos[2])
                self.update_ref_TOFs()
            self.history[anchor_id].append(
                {'seq': seq, 'rx': rx, 'tx': tx_ts, 'remote_data': remote_anchors})
            self.fresh_anchors.add(anchor_id)
        return {a: list(h) for a, h in self.history.items()}

    def process_anchor_data(self, aggregated_data: dict[int, list[dict]])->list[tuple]:
        """
        Computes TDOA pairs w.r.t. a single reference anchor. Every anchor with a fresh message is tried
        as reference; the one that yields the most valid pairs is used.

        RETURNS: List of shape [(TDOA, ref_pos, remote_pos), ...]
        """
        best = []
        for ref_anchor in self.fresh_anchors:
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
        # Computing TDOA with respect to each remote anchor in the reference anchor's received messages 
        for remote_anchor, (r_seq, r_rx, r_tof) in cur['remote_data'].items():
            if remote_anchor not in aggregated_data or r_tof is None:
                continue
            # The tag must have independently heard the exact packet the ref reported (same SEQ)
            remote_msg = next((m for m in aggregated_data[remote_anchor] if m['seq'] == r_seq), None)
            if remote_msg is None:
                continue
            if not self.validate_measured_TOF(ref_anchor, remote_anchor, r_tof):
                continue
            delta_tx = wrap_diff(cur['tx'], wrap_diff(r_rx, r_tof, bits=32, signed=False),
                                bits=32, signed=True)
            delta_rx = wrap_diff(cur['rx'], remote_msg['rx'], bits=40, signed=True)
            TDoA = (delta_rx - alpha * delta_tx) * DW1000.TIME_UNIT
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
        result = scipy.optimize.least_squares(tdoa_residuals, np.array(self.position), args=(tdoa_data,),
                                               method='trf', loss='huber', f_scale=0.7)  
        if result.success: 
            self.position = tuple(result.x) 
            return self.position 
        else: 
            print(f"[ERROR] Least squares failed to converge {result.message}")
            return None 