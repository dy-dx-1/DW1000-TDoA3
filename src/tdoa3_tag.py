import struct 
from dw1000 import DW1000
import scipy.optimize 
import numpy as np
import time 

from config import ANCHORS 

SPEED_OF_LIGHT = 299_702_547 # m/s 

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
        self._dw.close() 

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
        # NOTE TODO formalize and calibrate this properly, currently using 50cm 
        tolerance = 0.5/SPEED_OF_LIGHT/DW1000.TIME_UNIT 
        expected_tof = self.TOF_ref_anchors[a1][a2] 
        if abs(z_tof-expected_tof)<=tolerance: 
            return True 
        else: 
            return False

    def run(self, enable_print:bool): 
        """
        Starts an infinite loop that runs continuous localization of the tag. 
        TODO Add CSV buffering+saving? Future ROS publishing? 
        """
        while True: 
            # Gathering anchor data 
            raw_data = [] 
            t1 = time.perf_counter() 
            while (time.perf_counter()-t1)<1: # TODO formalize update frequency in config 
                pkt, rx = self._dw.listen(ranging=True) 
                if pkt: # rx is None if and only if pkt is None per DW1000 class methods.
                    # We don't process the pkts yet to ensure we get as much info as possible 
                    raw_data.append((pkt, rx))
            # Now that out of gathering info loop, parsing & organizing the data 
            # Using a dict {anchor_id: [{rx_t-1, tx_t-1, remote_data_t-1}, {rx, tx, remote_data}]}
            # Only keep the latest 2 messages for any anchor (need fresh data for calculations)
            aggregated_data = self.aggregate_raw_pkts(raw_data) 
            # Processing the data into TDOA-anchor pairs 
            tdoa_anchor_data = self.process_anchor_data(aggregated_data)                
            # Calculating position 
            # use some kind of array manipulation to compute all the tdoas efficiently? 
            position = self.multilaterate(tdoa_anchor_data)
            if enable_print: 
                print(f"[INFO] New position estimated: {position}")
    
    def aggregate_raw_pkts(self, data:list[tuple[int, int]])->dict[int, list[dict]]: 
        """
        Takes a list of received raw packets and compiles the data-per anchor in a dict. 
        
        ARGS:
        - data: List of tuples [(raw_packet, local_rx_time)] 
        
        RETURNS:
        - Dict of shape {anchor_id: [{rx_t-1, tx_t-1, remote_data_t-1}, {rx, tx, remote_data}]}
            - For each anchor, only the last 2 messages are kept to compute TDOA. If both are present, the SEQs are subsequent. 
        """
        parsed_data = {} 
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
            # Adding to dict
            msg_data = {'seq':seq, 'rx': rx, 'tx': tx_ts, 'remote_data':remote_anchors}
            number_of_msgs = len(parsed_data.get(anchor_id, [])) 
            if number_of_msgs==0: 
                # If we haven't added any messages -> initialize the list with one
                parsed_data[anchor_id] = [msg_data] 
            elif number_of_msgs==1: 
                # If we have 1 msg, add this one if it's the subsequent SEQ. Else, flush and keep the freshest 
                if seq == (parsed_data[anchor_id][0]['seq'] + 1)&0xFF: # &0xFF to wrap the counter if needed
                    parsed_data[anchor_id].append(msg_data)
                else: 
                    parsed_data[anchor_id] = [msg_data] 
            elif number_of_msgs==2: 
                # If we have 2 msgs, clear the 1st and add this one if it's subsequent SEQ. Else, flush and keep freshest. 
                if seq == (parsed_data[anchor_id][1]['seq'] + 1)&0xFF: 
                    parsed_data[anchor_id].pop(0) 
                    parsed_data[anchor_id].append(msg_data)
                else:
                    parsed_data[anchor_id] = [msg_data] 
        return parsed_data

    def process_anchor_data(self, aggregated_data:dict[int, list[dict, dict]]):
        """
        Takes an aggregated dict of per-anchor data and computes TDOA with anchor pairs with respect to a reference anchor. 
        The reference anchor is chosen as the one with most info about the others. 

        ARGS: 
        - Dict of aggregated data: {anchor_id: [{rx, tx, remote_data}, {rx, tx, remote_data}]}
            - At least one of the anchors must have data for 2 transactions to be able to compute TDOA with respect to it
            - If multiple anchors have 2 transaction data, the one with the most remote_data is used as reference 
            - Item 0 of the list is the oldest data 

        RETURNS:
        - List of shape [(TDOA, a1_pos, a2_pos), ...] 
        """
        results = [] 
        # Selecting reference anchor as the one with the largest remote_data that also has 2 subsequent transactions
        ref_anchor = None 
        best_remote_size = 0 
        for anchor_id, data_pair in aggregated_data.items(): 
            if len(data_pair)<2: 
                continue 
            current_remote_size = min(len(data_pair[0]['remote_data']), len(data_pair[1]['remote_data']))
            if  current_remote_size > best_remote_size:
                best_remote_size = current_remote_size
                ref_anchor = anchor_id 
        if ref_anchor is None: 
            return results 
        # Computing alpha for this reference anchor
        delta_tx_prime = wrap_diff(aggregated_data[ref_anchor][1]['tx'], aggregated_data[ref_anchor][0]['tx'], bits=32, signed=False) 
        delta_rx_prime = wrap_diff(aggregated_data[ref_anchor][1]['rx'], aggregated_data[ref_anchor][0]['rx'], bits=40, signed=False)
        try: 
            alpha = delta_rx_prime/delta_tx_prime # Conversion factor from ref_anchor clock ticks -> tag clock ticks 
        except ZeroDivisionError:
            return results # This shouldn't happen but just in case 
        # Going over all possible ref_anchor -> other anchor pairings and computing TDOA 
        for remote_anchor, (r_seq, r_rx, r_tof) in aggregated_data[ref_anchor][1]['remote_data'].items(): 
            if (remote_anchor not in aggregated_data) or (r_tof is None): 
                # Ignore the data for this anchor if we didn't hear from it or we don't have TOF info to it 
                continue 
            # Checking if one of the msgs we caught from the remote anchor matches the SEQ from reference-remote anchor transaction
            # These MUST match for delta RX to make physical sense. Else, ignore. 
            remote_msgs = aggregated_data[remote_anchor]
            if len(remote_msgs) == 2 and remote_msgs[1]['seq'] == r_seq:
                tag_data_from_remote_anchor = remote_msgs[1]
            elif remote_msgs[0]['seq'] == r_seq:  # safe whether len is 1 or 2
                tag_data_from_remote_anchor = remote_msgs[0]
            else:
                continue
            # Outlier check by comparing measured TOF with geometric TOF since we know all anchor positions 
            if not self.validate_measured_TOF(ref_anchor, remote_anchor, r_tof): 
                continue 
            # Computing delta TX in the reference anchor's clock: ref TX info - (ref RX info of the other tag - TOF both tags)
            delta_tx = wrap_diff(aggregated_data[ref_anchor][1]['tx'], wrap_diff(r_rx, r_tof, bits=32, signed=False),
                                  bits=32, signed=True)
            # Computing delta RX in the tag's clock 
            delta_rx = wrap_diff(aggregated_data[ref_anchor][1]['rx'], tag_data_from_remote_anchor['rx'], bits=40, signed=True)
            # Computing TDOA and storing 
            TDoA = (delta_rx - (alpha*delta_tx))*DW1000.TIME_UNIT # ticks->seconds 
            results.append( (TDoA, self.anchors[ref_anchor], self.anchors[remote_anchor]) )
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
        # Checking if the message has the expected header (BC format and general broadcast to 0xFF + TDOA3 header 0x30) 
        if msg[:13] != BC_TDOA3_DEST_HEADER or msg[21] != 0x30: 
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
                r_seq    = msg[id_idx+1]&0x7F
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
            print("[INFO] multilaterate() was called with less than 3 measurements, cannot converge.")
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