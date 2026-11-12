from config import UNITS
from pose import CycleUpdate
from units import position_from_meters
from outputs.base import OutputController


class ConsoleOutputController(OutputController):
    """Prints every cycle's result to stdout. Coordinates are in UNITS, rounded to DECIMALS."""
    def __init__(self, tag_id):
        self.tag_id = tag_id

    def publish(self, update: CycleUpdate) -> None:
        print(f"known anchors:{update.n_known_anchors} | aggregated:{update.n_aggregated} | processed:{update.n_pairs}")
        if update.pose is None:
            print("[INFO] -----> New position estimated: None")
        else:
            print(f"[INFO] -----> New position estimated: {position_from_meters(update.pose.position)} {UNITS}")
