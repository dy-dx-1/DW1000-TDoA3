import struct 
from .dw1000 import DW1000
import scipy
import numpy as np

from .config import ANCHORS 

SPEED_OF_LIGHT = 299_702_547 # m/s 

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
        self.id = id 
        self._dw = DW1000(bus, cs, channel=2, PRF=64, bitrate=6, preamble_length=128, preamble_code=9, smart_tx_power=True, tx_power_settings=None)
        self.position = (0,0,0) # TODO better initialization of position? Centroid of known anchors?  

        # Anchor positions can be pre-defined in config.py to overwrite firmware values. 
        # If a position is not pre-defined, it will be populated with the firmware-defined values published by the anchor in it's messages.
        self.anchors = ANCHORS # {anchor_id: (x,y,z)}
        
    def __enter__(self):
        return self 
    def __exit__(self, exc_type, exc_val, exc_tb):
        self._dw.close() 

    def run(self, enable_print:bool): 
        """
        Starts an infinite loop that runs continuous localization of the tag. 
        TODO Add CSV buffering+saving? Future ROS publishing? 
        """
        # Computing TDOA parameters requires 2 subsequent measurements 
        # Therefore we iterate 2 different behaviors: initial listen -> 'analysis' (trying to construct parameters) 
        # Only if we catch enough subsequent listens to build sufficient TDOA pairs can we localize 
        # The high frequency of transmissions makes it so this strict policy still enables localization 
        iter_type = 0 
        while True: 
            pkt, rx = self._dw.listen(ranging=True, timeout=5) # We should always be near anchors so the timeout should never be reached unless we move out of range. 
            if not pkt: # rx is None if and only if pkt is None per DW1000 class methods. 
                continue   
            anchor_id, seq, tx_ts, remote_anchors, anchor_pos = self.interpret_anchor_msg(pkt) 
            # Adding anchor_pos to our references if it doesn't exist 
            if anchor_id not in self.anchors: 
                self.anchors[anchor_id] = (anchor_pos[0], anchor_pos[1], anchor_pos[2])
            # If this is an 'initial listen' iter, store what we heard 
            if not iter_type: 
                pass
            # Else, we are at an 'analysis' iter, try to build TDOA info  
            else: 
                # TODO extract delta TX from tx_ts and remote_anchors data 
                # TODO extract alpha with seq and remote_anchors data 
                # on every 2nd transmission, compute tdoa for each pair 
                # compute it in array manner for efficiency ? 
                position = self.multilaterate([])
                if enable_print: 
                    print(f"[INFO] New position estimated: {position}")
            # Switch iter type for next round 
            iter_type = 1 - iter_type 

    @staticmethod
    def interpret_anchor_msg(msg:list[int])->tuple[int, int, int, dict, tuple|None]:
        """
        Takes a raw intercepted message between anchors and extracts the info we need from it. 

        RETURNS: 
        - anchor_id: ID of the anchor that sent the message
        - seq:       SEQ of the transaction 
        - tx_ts:     Transmit timestamp of the message in 32-bit based DW1000 ticks
        - remote_anchors: dict of shape {remoteAnchorIds: (seq, rx_ts, r_dist)} rx (32-bit based) and distances (16bit) in ticks 
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
        if msg[:13] != BC_TDOA3_DEST_HEADER and msg[21] != 0x30: 
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
        remote_anchors = {} # {id: (seq, rx_ts, r_dist)}
        if n_other_anchors!=0: 
            start_idx = 28 
            for _ in range(n_other_anchors): 
                id_idx = start_idx 
                r_id = msg[id_idx]
                has_dist = msg[id_idx+1]>>7
                r_seq    = msg[id_idx+1]&0x7F
                rx_ts    = int.from_bytes(bytes(msg[id_idx+2:id_idx+6]), 'little') 
                r_dist   = int.from_bytes(bytes(msg[id_idx+6:id_idx+8]), 'little') if has_dist else None # NOTE dist is in radio ticks

                remote_anchors[r_id] = (r_seq, rx_ts, r_dist)
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
        - tdoa_data: A list of tuples representing independent measurements (minimum 4) 
            - tuple format: (TDOA_measure_seconds, anchor_pos_tuple_1, anchor_pos_tuple_2)

        INFO:\n
        Position is estimated through non-linear least squares.
        The residual function is defined as (TDOA corresponding to the current estimated position - measured TDOA by the tag) \n
        The TDOA values given should always be with respect to (tag->a1 - tag->a2) 
        """
        if len(tdoa_data)<4:
            print("[ERROR] multilaterate() was called with less than 4 measurements, cannot converge.")
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
        result = scipy.optimize.least_squares(tdoa_residuals, np.array(self.position), args=(tdoa_data,), method='lm') 
        if result.success: 
            self.position = tuple(result.x[0], result.x[1], result.x[2]) 
            return self.position 
        else: 
            print(f"[ERROR] Least squares failed to converge {result.message}")
            return None 