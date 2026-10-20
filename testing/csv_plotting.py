
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np 

# For 3D plotting
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401


# ============================================================
# Configuration
# ============================================================

ANCHORS = {1: (4.736, 2.194, 1.213), 
           2: (1.373, 5.520, 2.190), 
           3: (1.372, 0.150, 0.128), 
           4: (4.736, 2.194, 0.302), 
           5: (0.793, 2.971, 2.190), 
           6: (4.582, 6.040, 1.460), 
           7: (0.223, 0.458, 2.190), 
           8: (5.045, 4.194, 1.070)} 

CSV_PATH = "tdoa3_tag_1.csv"

PLOT_3D = False

# ============================================================
# Helper functions
# ============================================================

def format_duration(seconds):
    """Format a duration in seconds as hours, minutes, and seconds."""
    seconds = max(0, int(round(seconds)))

    hours, remainder = divmod(seconds, 3600)
    minutes, secs = divmod(remainder, 60)

    if hours:
        return f"{hours} h {minutes} min {secs} s"
    elif minutes:
        return f"{minutes} min {secs} s"
    else:
        return f"{secs} s"


def format_interval(seconds):
    """Format a sampling interval using an appropriate time unit."""
    if seconds < 0:
        return f"{seconds:.3g} s"
    elif seconds < 1:
        return f"{seconds * 1000:.3g} ms"
    elif seconds < 60:
        return f"{seconds:.3g} s"
    elif seconds < 3600:
        return f"{seconds / 60:.3g} min"
    else:
        return f"{seconds / 3600:.3g} h"


# ============================================================
# Load CSV
# ============================================================

df = pd.read_csv(CSV_PATH)

required_columns = [
    "timestamp",
    "position_x",
    "position_y",
    "position_z",
]

missing_columns = set(required_columns) - set(df.columns)

if missing_columns:
    raise ValueError(
        f"Missing required CSV columns: {sorted(missing_columns)}"
    )

if df.empty:
    raise ValueError("The CSV file contains no data.")


# ============================================================
# Parse timestamps
# ============================================================

# Timestamps are ISO 8601 strings, for example:
# 2026-10-20T11:37:12.345-04:00
#
# Convert to UTC so elapsed times remain correct even if
# timestamps have different timezone offsets.

timestamps = pd.to_datetime(
    df["timestamp"].str.strip(),
    format="ISO8601",
    utc=True,
    errors="coerce",
)

if timestamps.isna().any():
    invalid_count = int(timestamps.isna().sum())
    raise ValueError(
        f"{invalid_count} timestamp(s) could not be parsed. "
        "Expected ISO 8601 datetime strings with timezone offsets."
    )

# Total duration from earliest to latest timestamp.
duration_seconds = (
    timestamps.max() - timestamps.min()
).total_seconds()

# Calculate intervals between chronological timestamps.
sorted_timestamps = timestamps.sort_values()

intervals = (
    sorted_timestamps.diff()
    .dropna()
    .dt.total_seconds()
)

# Median interval represents the typical recording interval.
recording_interval = (
    float(intervals.median())
    if not intervals.empty
    else float("nan")
)


# ============================================================
# Process position columns
# ============================================================

position_columns = [
    "position_x",
    "position_y",
    "position_z",
]

# Convert empty strings and whitespace-only values to NaN.
for column in position_columns:
    df[column] = pd.to_numeric(
        df[column].replace(r"^\s*$", np.nan, regex=True),
        errors="coerce",
    )

# A position is valid only if all three coordinates are present.
valid_positions = df[position_columns].notna().all(axis=1)

# Keep the original CSV order to preserve trajectory direction.
trajectory = df.loc[valid_positions].copy()

if trajectory.empty:
    raise ValueError(
        "No valid positions found. Check the position_x, "
        "position_y, and position_z columns."
    )


# ============================================================
# Calculate total trajectory length
# ============================================================

positions = trajectory[position_columns].to_numpy(dtype=float)

if len(positions) >= 2:
    # Euclidean distance between consecutive valid positions.
    segment_lengths = np.linalg.norm(
        np.diff(positions, axis=0),
        axis=1,
    )

    path_length = float(segment_lengths.sum())
else:
    path_length = 0.0


# ============================================================
# Prepare plot title
# ============================================================

if np.isnan(recording_interval):
    interval_text = "N/A"
else:
    interval_text = format_interval(recording_interval)

title = (
    f"Trajectory | Duration: {format_duration(duration_seconds)}"
    f" | Median interval: {interval_text}"
)


# ============================================================
# Plot trajectory
# ============================================================

if PLOT_3D:
    fig = plt.figure(figsize=(11, 8))
    ax = fig.add_subplot(111, projection="3d")

    ax.plot(
        trajectory["position_x"],
        trajectory["position_y"],
        trajectory["position_z"],
        color="royalblue",
        linewidth=1.5,
        label="Trajectory",
    )

    # Plot anchors as stars.
    for anchor_id, position in ANCHORS.items():
        x, y, z = position

        ax.scatter(
            x,
            y,
            z,
            marker="*",
            s=220,
            edgecolors="black",
            label=f"Anchor {anchor_id}",
        )

    ax.set_xlabel("X position")
    ax.set_ylabel("Y position")
    ax.set_zlabel("Z position")

    ax.grid(True)

else:
    fig, ax = plt.subplots(figsize=(10, 8))

    ax.plot(
        trajectory["position_x"],
        trajectory["position_y"],
        color="royalblue",
        linewidth=1.5,
        label="Trajectory",
    )

    # Plot the XY projection of each anchor.
    for anchor_id, position in ANCHORS.items():
        x, y, _z = position

        ax.scatter(
            x,
            y,
            marker="*",
            s=220,
            edgecolors="black",
            label=f"Anchor {anchor_id}",
        )

    ax.set_xlabel("X position")
    ax.set_ylabel("Y position")
    ax.set_aspect("equal", adjustable="datalim")
    ax.grid(True)


# ============================================================
# Finalize and display
# ============================================================

ax.set_title(title)
ax.legend(loc="best")

fig.tight_layout()

print(f"CSV file:              {CSV_PATH}")
print(f"Valid trajectory points: {len(trajectory)}")
print(f"Total recording time:  {format_duration(duration_seconds)}")
print(f"Median recording interval: {interval_text}")
print(f"Total path length:     {path_length:.3f}")

plt.show()
