import struct 
from .dw1000 import DW1000

class TDOA3_Tag: 
    """
    A DW1000 based UWB Tag that works with the Bitcraze anchors following their TDoA3 protocol. 
    Only use with a context manager for safety. 

    ARGS: 
    - id: The tag's ID 
    - bus: SPI bus for the DW1000 connection
    - cs:  SPI chip select for the DW1000 connection
    """
    def __init__(self, id, bus=0, cs=0): 
        self.id = id 
        self._dw = DW1000(bus, cs, channel=2, PRF=64, bitrate=6, preamble_length=128, preamble_code=9, smart_tx_power=True, tx_power_settings=None)

    def __enter__(self):
        return self 
    def __exit__(self, exc_type, exc_val, exc_tb):
        self._dw.close() 

    @staticmethod
    def interpret_anchor_msg(msg:list[int])->tuple[int, int, int, dict, tuple|None]:
        """
        Takes a raw intercepted message between anchors and extracts the info we need from it. 

        RETURNS: 
        - anchor_id: ID of the anchor that sent the message
        - seq:       SEQ of the transaction 
        - tx_ts:     Transmit timestamp of the message  # TODO fix units 
        - remote_anchors: dict of shape {remoteAnchorIds: (seq, rx_ts, r_dist)} distances in ticks 
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
        print([hex(i) for i in msg[21:]])
        if msg[:13] != BC_TDOA3_DEST_HEADER and msg[21] != 0x30: 
            return 
        ## Extracting important info from message 
        # Source anchor ID 
        anchor_id = int.from_bytes(bytes(msg[13:19]), byteorder='little') 
        # SEQ 
        seq = msg[22] 
        # TX Timestamp
        tx_ts = msg[23:27]
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
                rx_ts    = msg[id_idx+2:id_idx+6]
                r_dist   = int.from_bytes(bytes(msg[id_idx+6:id_idx+8]), 'little') if has_dist else None # NOTE dist is in radio ticks

                remote_anchors[r_id] = (r_seq, rx_ts, r_dist)
                start_idx += 6 + (2 if has_dist else 0) # next anchor index will depend on if this info had a distance
        # Source anchor position if available. Since it's position depends on remoteAnchors, we index from the end 
        anchor_pos_packet = msg[-14:] 
        anchor_pos = None 
        if anchor_pos_packet[0:2] == [0xF0, 0x01]: # Expected header for a LPP short packet with anchor position 
            anchor_pos = struct.unpack('fff', bytes(anchor_pos_packet[2:]))

        return anchor_id, seq, tx_ts, remote_anchors, anchor_pos  