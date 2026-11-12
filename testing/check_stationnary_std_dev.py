"""Collect stationary tag positions and report their per-axis standard deviation."""
from pathlib import Path
import sys

import numpy as np

SRC_DIR = Path(__file__).resolve().parents[1] / "src"
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from config import TAG_ID, SPI_BUS, SPI_CS, ANCHORS
from tdoa3_tag import TDOA3_Tag
from units import anchors_to_meters


def main():
    positions = []

    class PositionTrackingTag(TDOA3_Tag):
        def multilaterate(self, tdoa_data, ref_anchor=None):
            pose = super().multilaterate(tdoa_data, ref_anchor=ref_anchor)
            if pose is not None:
                positions.append(pose.position)
            return pose

    try:
        with PositionTrackingTag(id=TAG_ID, bus=SPI_BUS, cs=SPI_CS,
                                 anchors=anchors_to_meters(ANCHORS)) as tag:
            tag.run()
    except KeyboardInterrupt:
        if not positions:
            print("[INFO] No valid positions were collected.")
            return

        std_dev = np.std(positions, axis=0)
        print(f"[INFO] Position standard deviation (x, y, z): "
              f"{std_dev[0]:.6f}, {std_dev[1]:.6f}, {std_dev[2]:.6f} m "
              f"({len(positions)} positions).")


if __name__ == "__main__":
    main()
