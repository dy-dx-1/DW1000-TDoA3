import csv
import math
import os
import random
import time
from datetime import datetime
from config import LOCALIZATION_FREQUENCY, CSV_SAVE_INTERVAL_SECONDS
from pose import CycleUpdate
from units import position_from_meters, covariance_from_meters
from outputs.base import OutputController

CSV_HEADER = (
    "timestamp",
    "known_anchors",
    "aggregated_data_size",
    "processed_data_pairs",
    "position_x",
    "position_y",
    "position_z",
    "ref_anchor",
    "cov_xx",
    "cov_yy",
    "cov_xy",
)
MAX_FILE_SUFFIX = 100  # Existing files are never appended to: a new one gets a random _<0..MAX_FILE_SUFFIX> suffix


def new_csv_filename(base:str)->str:
    """
    Returns `<base>.csv` if it doesn't exist yet, otherwise `<base>_<n>.csv` with a random unused n in [0, MAX_FILE_SUFFIX].
    Falls back to a timestamp suffix if all of them are taken.
    """
    if not os.path.exists(f"{base}.csv"):
        return f"{base}.csv"
    free = [n for n in range(MAX_FILE_SUFFIX + 1) if not os.path.exists(f"{base}_{n}.csv")]
    filename = f"{base}_{random.choice(free)}.csv" if free else f"{base}_{datetime.now():%Y%m%d-%H%M%S}.csv"
    print(f"[INFO] {base}.csv already exists, localization results will be saved to {filename}")
    return filename


class CsvOutputController(OutputController):
    """
    Buffers one row per cycle and appends them to a new CSV file in the working directory: `tag_<id>.csv`,
    or `tag_<id>_<n>.csv` if that one already exists, so previous runs are never appended to.
    Rows are flushed when the buffer holds CSV_SAVE_INTERVAL_SECONDS worth of cycles, when
    CSV_SAVE_INTERVAL_SECONDS have passed since the last flush, and on close().
    Coordinates are in UNITS, rounded to DECIMALS. The covariance entries are in UNITS², not rounded.
    Pose columns (coordinates, ref_anchor, covariance) are left blank for cycles without a pose.
    """
    def __init__(self, tag_id):
        self.filename = new_csv_filename(f"tag_{tag_id}")
        self._rows = []
        self._last_save = time.monotonic()
        self._max_rows = max(1, math.ceil(LOCALIZATION_FREQUENCY * CSV_SAVE_INTERVAL_SECONDS))

    def publish(self, update: CycleUpdate) -> None:
        pose = update.pose
        if pose is not None:
            cov = pose.covariance
            pose_columns = (
                *position_from_meters(pose.position),
                pose.ref_anchor if pose.ref_anchor is not None else "",
                *(f"{covariance_from_meters(v):.4g}" for v in (cov[0, 0], cov[1, 1], cov[0, 1])),
            )
        else:
            pose_columns = ("",) * 7
        self._rows.append((
            datetime.fromtimestamp(update.stamp_ns / 1e9).astimezone().isoformat(timespec="milliseconds"),
            update.n_known_anchors,
            update.n_aggregated,
            update.n_pairs,
            *pose_columns,
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
