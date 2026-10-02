"""
Continuously positions a DW1000 tag and dynamically plots the position in matplotlib using WebAgg.

TO USE THE SCRIPT:
1. Open SSH Tunnel from main machine and forward port 8988
ssh -L 8988:localhost:8988 pi@raspberrypi_ip
2. Run the script on the pi 
3. Open a webbrowser on the main machine and go to http://localhost:8988/
4. Use ctrl-c to stop
"""
import sys
from pathlib import Path
import time 
import numpy as np 

# --- CRITICAL: Must be set before importing pyplot ---
import matplotlib
matplotlib.use("WebAgg")
import matplotlib.pyplot as plt
# ----------------------------------------------------

from scipy.optimize import least_squares

# Add parent directory to sys.path
parent_dir = str(Path(__file__).resolve().parent.parent)
if parent_dir not in sys.path:
    sys.path.insert(0, parent_dir)

from src.clamour.interfaces.bitcraze_tag import BitcrazeTag
from src.clamour.interfaces.anchors import Anchors 

### CONFIGURATION PARAMETERS
# NOTE: anchors correspond to config.py 
REFRESH_RATE = 1  # Hz (Change this if you want to push toward higher frequencies)


def trilaterate(anchor_positions, distances, initial_position=None):
    """Estimate a 2D tag position from at least three ranges, in centimeters."""
    anchor_positions = np.asarray(anchor_positions, dtype=float)
    distances = np.asarray(distances, dtype=float)

    if len(anchor_positions) < 3:
        return None

    if initial_position is None:
        initial_position = np.mean(anchor_positions, axis=0)

    def residuals(position):
        estimated_distances = np.linalg.norm(anchor_positions - position, axis=1)
        return estimated_distances - distances

    result = least_squares(residuals, np.asarray(initial_position, dtype=float))
    if not result.success:
        return None
    return result.x


anchors = Anchors().anchors_dict
anchor_positions = np.array(
    [anchor[:2] for anchor in anchors.values()], dtype=float
)

plt.ion()
figure, axis = plt.subplots(figsize=(6, 6))
axis.set_title("Dynamic DW1000 tag position (WebAgg)")
axis.set_xlabel("X (cm)")
axis.set_ylabel("Y (cm)")
axis.plot(anchor_positions[:, 0], anchor_positions[:, 1], "b*", markersize=10, label="Anchors")
axis.set_aspect("equal", adjustable="datalim")
trajectory_line, = axis.plot([], [], "r-", linewidth=1.5, label="Trajectory")
position_points, = axis.plot([], [], "ro", markersize=4)
axis.legend()
trajectory = []

with BitcrazeTag(tag_id=11, dw1000_bus=0, dw1000_cs=0, channel=2, PRF=64, bitrate=6.8,
                preamble_length=128, preamble_code=9,
                smart_tx_power=True, tx_power_settings=None) as tag:
    try:
        print("WebAgg server starting... Access via your local browser through the SSH tunnel.")
        while plt.fignum_exists(figure.number):
            # Collect all successful ranges before attempting this cycle's update.
            measured_positions = []
            measured_distances = []
            for anchor_id, anchor_pos in anchors.items():
                distance, _ = tag.compute_range(anchor_id)
                if distance is not None:
                    measured_positions.append(anchor_pos.coordinates[:2])
                    measured_distances.append(distance)

            if len(measured_positions) >= 3:
                position = trilaterate(measured_positions, measured_distances)
                if position is not None:
                    trajectory.append(position)
                    trajectory_array = np.asarray(trajectory)
                    trajectory_line.set_data(trajectory_array[:, 0], trajectory_array[:, 1])
                    position_points.set_data(trajectory_array[:, 0], trajectory_array[:, 1])
                    axis.relim()
                    axis.autoscale_view()

            figure.canvas.draw_idle()
            figure.canvas.flush_events()
            plt.pause(1 / REFRESH_RATE)
            
    except KeyboardInterrupt:
        print("\nStopping live plot...")
    finally:
        plt.close(figure)
