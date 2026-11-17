"""
Controller for the dynamic_MIST.py display from /utils.
Allows for dynamic display of the position on a different machine.
"""
import socket
import struct

from outputs.base import OutputController
from pose import CycleUpdate
from units import position_in_units

UDP_HOST = "127.0.0.1"
UDP_PORT = 5005


class DynamicDisplayController(OutputController):
    """
    Sends the latest position update in configured UNITS to a UDP socket for use with utils/dynamic_MIST.py.
    Coordinates are not rounded to DECIMALS.
    """
    def __init__(self, tag_id):
        self.tag_id = tag_id
        self.destination = (UDP_HOST, UDP_PORT)
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def publish(self, update: CycleUpdate) -> None:
        if update.pose is None:
            return

        packet = struct.pack("<ifff", self.tag_id, *position_in_units(update.pose.position))
        self._socket.sendto(packet, self.destination)

    def close(self) -> None:
        self._socket.close()
