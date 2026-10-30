"""
TDoA multilateration solver. Pure module (numpy/scipy only), shared by the real and synthetic tags.
"""
import time
import numpy as np
import scipy.optimize
from pose import TDOA3_Pose

SPEED_OF_LIGHT = 299_702_547 # m/s
MIN_PAIRS = 3


def multilaterate(tdoa_data:list[tuple], initial)->TDOA3_Pose|None:
    """
    Use TDoA multilateration to estimate the tag's position.\n
    Returns the estimated TDOA3_Pose, or None if there are too few measurements or the solver failed.\n
    ARGS:
    - tdoa_data: A list of tuples representing independent measurements (minimum 3)
        - tuple format: (TDOA_measure_seconds, anchor_pos_tuple_1, anchor_pos_tuple_2)
    - initial: (x,y,z) starting point of the optimization

    INFO:\n
    Position is estimated through non-linear least squares.
    The residual function is defined as (TDOA corresponding to the current estimated position - measured TDOA by the tag) \n
    The TDOA values given should always be with respect to (tag->a1 - tag->a2)
    """
    if len(tdoa_data)<MIN_PAIRS:
        #print("[INFO] multilaterate() was called with less than 3 measurements, cannot converge.")
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
    # Using huber to cap the influence of unexpected outliers
    # Using f_scale of 0.7m for now - to be tuned more in the future
    result = scipy.optimize.least_squares(tdoa_residuals, np.array(initial), args=(tdoa_data,),
                                           method='trf', loss='huber', f_scale=0.7)
    if not result.success:
        print(f"[ERROR] Least squares failed to converge {result.message}")
        return None
    x, y, z = (float(v) for v in result.x)
    return TDOA3_Pose(
        stamp_ns=time.time_ns(),
        x=x, y=y, z=z,
        n_pairs=len(tdoa_data),
        mode_2d=False,
    )
