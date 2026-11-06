"""
TDoA multilateration solver. Pure module (numpy/scipy only), shared by the real and synthetic tags.
"""
import numpy as np
import scipy.optimize
from pose import TDOA3_Pose

SPEED_OF_LIGHT = 299_702_547 # m/s
MIN_PAIRS = 3           # Minimum TDOA pairs to attempt a solve (enough for both 2D and 3D)
MIN_PAIRS_RMS_GATE = 4  # Minimum TDOA pairs when the max_residual_rms_m gate is enabled, so there are always redundant pairs
HUBER_F_SCALE_M = 0.7   # Residual size [m] above which huber starts capping a pair's influence. To be tuned more in the future

# Noise model used for the covariance. Not user-configurable: the user gets a covariance without having to model uncertainty.
TDOA_SIGMA_M = 0.10      # PLACEHOLDER until calibrated. Std dev of one TDOA pair's range difference [m]
PAIR_CORRELATION = 0.5   # Every pair reuses the reference anchor's packet, so any two pairs share half their noise
                         # when the tag's rx timestamp noise dominates


def multilaterate(tdoa_data:list[tuple], initial, ref_anchor:int|None=None, mode_2d:bool=False,
                  antenna_height_m:float=0.0, max_residual_rms_m:float|None=None)->TDOA3_Pose|None:
    """
    Use TDoA multilateration to estimate the tag's position and its covariance.\n
    Returns the estimated TDOA3_Pose, or None if there are too few measurements or the solve was rejected.\n
    ARGS:
    - tdoa_data: A list of tuples representing independent measurements (minimum MIN_PAIRS, MIN_PAIRS_RMS_GATE with the RMS gate)
        - tuple format: (TDOA_measure_seconds, anchor_pos_tuple_1, anchor_pos_tuple_2)
    - initial: (x,y,z) starting point of the optimization. Only x,y are used in 2D mode
    - ref_anchor: ID of the reference anchor of the TDOA pairs, copied to the pose
    - mode_2d: Solve x,y only, with z fixed at antenna_height_m
    - antenna_height_m: z of the tag in 2D mode, in meters
    - max_residual_rms_m: Reject the pose if the pairs' RMS disagreement at the solution is above this [m]. None = disabled

    INFO:\n
    Position is estimated through non-linear least squares.
    The residual function is defined as (TDOA corresponding to the current estimated position - measured TDOA by the tag) \n
    The TDOA values given should always be with respect to (tag->a1 - tag->a2)
    """
    min_pairs = MIN_PAIRS if max_residual_rms_m is None else MIN_PAIRS_RMS_GATE
    if len(tdoa_data)<min_pairs:
        return None
    a1_pos = np.array([a1 for _, a1, _ in tdoa_data], dtype=float)
    a2_pos = np.array([a2 for _, _, a2 in tdoa_data], dtype=float)
    measured_delta = np.array([tdoa for tdoa, _, _ in tdoa_data]) * SPEED_OF_LIGHT # result in meters

    def to_position(params:np.ndarray)->np.ndarray:
        # In 2D mode only x,y are optimized, z stays at the antenna height
        return np.append(params, antenna_height_m) if mode_2d else params

    def tdoa_residuals(params:np.ndarray)->np.ndarray:
        # Residual function: (TDOA corresponding to an estimate - measured TDOA)
        # IMPORTANT: TDOAs are assumed to be with respect to anchor 1 - anchor 2.
        p = to_position(params)
        estimated_delta = np.linalg.norm(p - a1_pos, axis=1) - np.linalg.norm(p - a2_pos, axis=1)
        return estimated_delta - measured_delta

    # Non-linear least squares, using huber to cap the influence of unexpected outliers
    start = np.asarray(initial, dtype=float)[:2 if mode_2d else 3]
    result = scipy.optimize.least_squares(tdoa_residuals, start, method='trf', loss='huber', f_scale=HUBER_F_SCALE_M)
    if not result.success:
        print(f"[ERROR] Least squares failed to converge {result.message}")
        return None
    position = to_position(result.x)
    if not np.all(np.isfinite(position)):
        return None

    residuals = tdoa_residuals(result.x) # plain residuals, not huber-weighted
    n_pairs, n_unknowns = len(residuals), len(result.x)
    if max_residual_rms_m is not None:
        rms = np.sqrt(residuals @ residuals / (n_pairs - n_unknowns))
        if rms > max_residual_rms_m:
            return None

    covariance = position_covariance(position, a1_pos, a2_pos, residuals, n_unknowns)
    if covariance is None:
        return None
    x, y, z = (float(v) for v in position)
    return TDOA3_Pose(
        x=x, y=y, z=z,
        covariance=covariance,
        ref_anchor=ref_anchor,
        mode_2d=mode_2d,
    )


def position_covariance(position:np.ndarray, a1_pos:np.ndarray, a2_pos:np.ndarray, residuals:np.ndarray,
                        n_unknowns:int)->np.ndarray|None:
    """
    Estimates the 3x3 position covariance [m²] at the solved position. Returns None if the geometry can't be inverted.
    In 2D mode (n_unknowns = 2) only the x,y block is filled, the z row/col is 0.

    1. Geometry: plain Jacobian of the residuals, row i = (p-a1)/|p-a1| - (p-a2)/|p-a2|
       (scipy's result.jac is huber-scaled, so it is not used)
    2. Pair noise: R = σ²·C, C = (1-ρ)·I + ρ·11ᵀ, as all pairs share the reference anchor's packet
    3. Position noise: the solve is unweighted least squares, so the matching sandwich formula
       cov = (JᵀJ)⁻¹·JᵀRJ·(JᵀJ)⁻¹
    4. Bad-cycle inflation: if the pairs disagree more than σ this cycle, cov is scaled up by s²/σ² (never down)
    """
    to_a1, to_a2 = position - a1_pos, position - a2_pos
    jacobian = (to_a1 / np.linalg.norm(to_a1, axis=1, keepdims=True)
                - to_a2 / np.linalg.norm(to_a2, axis=1, keepdims=True))[:, :n_unknowns]
    n_pairs = len(residuals)
    correlation = (1 - PAIR_CORRELATION) * np.eye(n_pairs) + PAIR_CORRELATION * np.ones((n_pairs, n_pairs))
    try:
        jtj_inv = np.linalg.inv(jacobian.T @ jacobian)
    except np.linalg.LinAlgError:
        return None
    cov = TDOA_SIGMA_M**2 * jtj_inv @ jacobian.T @ correlation @ jacobian @ jtj_inv
    if n_pairs > n_unknowns:
        s2 = residuals @ np.linalg.solve(correlation, residuals) / (n_pairs - n_unknowns)
        cov *= max(1.0, s2 / TDOA_SIGMA_M**2)
    cov = (cov + cov.T) / 2
    if not np.all(np.isfinite(cov)):
        return None
    covariance = np.zeros((3, 3))
    covariance[:n_unknowns, :n_unknowns] = cov
    return covariance
