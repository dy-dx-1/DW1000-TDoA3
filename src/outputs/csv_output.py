import csv
import math
import time
from datetime import datetime
from config import LOCALIZATION_FREQUENCY, CSV_SAVE_INTERVAL_SECONDS
from pose import CycleUpdate
from units import position_from_meters
from outputs.base import OutputController

CSV_HEADER = (
    "timestamp",
    "known_anchors",
    "aggregated_data_size",
    "processed_data_pairs",
    "position_x",
    "position_y",
    "position_z",
)


class CsvOutputController(OutputController):
    """
    Buffers one row per cycle and appends them to `tdoa3_tag_<id>.csv` in the working directory.
    Rows are flushed when the buffer holds CSV_SAVE_INTERVAL_SECONDS worth of cycles, when
    CSV_SAVE_INTERVAL_SECONDS have passed since the last flush, and on close().
    Coordinates are in UNITS, rounded to DECIMALS, and left blank for cycles without a pose.
    """
    def __init__(self, tag_id):
        self.filename = f"tdoa3_tag_{tag_id}.csv"
        self._rows = []
        self._last_save = time.monotonic()
        self._max_rows = max(1, math.ceil(LOCALIZATION_FREQUENCY * CSV_SAVE_INTERVAL_SECONDS))

    def publish(self, update: CycleUpdate) -> None:
        coordinates = position_from_meters(update.pose.position) if update.pose is not None else ("", "", "")
        self._rows.append((
            datetime.fromtimestamp(update.stamp_ns / 1e9).astimezone().isoformat(timespec="milliseconds"),
            update.n_known_anchors,
            update.n_aggregated,
            update.n_pairs,
            *coordinates,
        ))
        if (len(self._rows) >= self._max_rows
            or time.monotonic() - self._last_save >= CSV_SAVE_INTERVAL_SECONDS
        ):
            self._save_rows()

    def close(self) -> None:
        self._save_rows()

    def _save_rows(self):
        if not self._rows:
            return

        with open(self.filename, "a", newline="", encoding="utf-8") as csv_file:
            writer = csv.writer(csv_file)
            csv_file.seek(0, 2)
            if csv_file.tell() == 0:
                writer.writerow(CSV_HEADER)
            writer.writerows(self._rows)
        self._rows.clear()
        self._last_save = time.monotonic()
