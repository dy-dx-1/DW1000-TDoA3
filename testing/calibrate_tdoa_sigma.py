"""
Calibration of TDOA_SIGMA_M (src/solver.py): the std dev of one TDOA pair's range difference, in meters.

Records stationary data with the tag (or loads a previous recording) and reports what TDOA_SIGMA_M should be.
Keep the tag perfectly still during the recording. Uses ANCHORS, UNITS and SOLVER from config.py.

USAGE (from the repo root):
- Record for 60 s on the tag's computer, then analyse:  python testing/calibrate_tdoa_sigma.py
- Re-analyse a previous recording (works anywhere):     python testing/calibrate_tdoa_sigma.py --pkl calibration_<date>.pkl
- If the tag sits on a surveyed point, give it (in meters) so biases are measured against the truth:
                                                        python testing/calibrate_tdoa_sigma.py --true-position 3.5 3.0 0.8

METHOD:
1. Every cycle's TDOA pairs are compared with the range differences expected at the tag's position
   (the surveyed --true-position, or else the median of the solved positions).
2. Errors are grouped per (reference, remote) anchor pair. Their mean is a fixed bias (anchor position or antenna
   delay errors, NLOS), their spread around it is the noise. TDOA_SIGMA_M is the pooled noise std over all pairs.
3. The poses are then solved again with the recommended value to check the reported covariance against the spread.
"""
import argparse
import contextlib
import io
import pickle
import sys
import time
from collections import defaultdict
from datetime import datetime
from pathlib import Path

import numpy as np

SRC_DIR = Path(__file__).resolve().parents[1] / "src"
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

import solver
from config import SOLVER
from solver import SPEED_OF_LIGHT

MIN_SAMPLES_PER_PAIR = 10  # Anchor pairs seen fewer times are left out of the pooled estimate
OUTLIER_ROBUST_STDS = 5    # Pair errors further than this many robust std devs from the pair's median are dropped
OUTLIER_POSE_DISTANCE_M = 1.0  # Poses further than this from the reference position are left out of the consistency check
EKF_GATE_SIGMAS = 4.0      # pose0_pose_rejection_threshold of the EKF


def record(duration_s:float)->list[list[tuple]]:
    """Records raw (pkt, rx) batches with the DW1000 for duration_s seconds, like TDOA3_Tag.save_runs(). Needs the radio."""
    from config import TAG_ID, SPI_BUS, SPI_CS
    import tdoa3_tag
    batches = []
    with tdoa3_tag.TDOA3_Tag(TAG_ID, SPI_BUS, SPI_CS, outputs=[]) as tag:
        print(f"[INFO] Recording for {duration_s:.0f} s, keep the tag still...")
        end = time.perf_counter() + duration_s
        while time.perf_counter() < end:
            batches.append(tag.listen())
            print(f"\r[INFO] {len(batches)} batches, {sum(len(b) for b in batches)} packets", end="")
    print()
    return batches


def extract_cycles(path:str):
    """Runs a recording through the tag's aggregation and processing. Returns (anchors, [pairs per cycle])."""
    # The synthetic tag runs the real tag's processing on recorded data, without needing the radio
    from tdoa3_tag_synthetic import SyntheticTag
    cycles = []
    with contextlib.redirect_stdout(io.StringIO()):
        with SyntheticTag(path, id=1, outputs=[]) as tag:
            while (raw_data := tag.listen()) is not None:
                pairs, _ = tag.process_anchor_data(tag.aggregate_raw_pkts(raw_data))
                cycles.append(pairs)
    return tag.anchors, cycles


def solve(cycles:list[list[tuple]], anchors:dict)->list:
    """Solves every cycle from the anchor centroid with the SOLVER mode from config.py. None for failed cycles."""
    initial = np.mean(np.array(list(anchors.values())), axis=0)
    with contextlib.redirect_stdout(io.StringIO()):
        return [solver.multilaterate(pairs, initial, mode_2d=SOLVER["mode_2d"],
                                     antenna_height_m=SOLVER["antenna_height_m"]) for pairs in cycles]


def pair_errors(pairs:list[tuple], center:np.ndarray)->np.ndarray:
    """Measured range differences minus the ones expected at center, in meters."""
    a1 = np.array([a for _, a, _ in pairs], dtype=float)
    a2 = np.array([a for _, _, a in pairs], dtype=float)
    expected = np.linalg.norm(center - a1, axis=1) - np.linalg.norm(center - a2, axis=1)
    return np.array([tdoa for tdoa, _, _ in pairs]) * SPEED_OF_LIGHT - expected


def residual_s2(pairs:list[tuple], pose, n_unknowns:int)->float:
    """The solver's bad-cycle statistic s² = rᵀC⁻¹r/(m-n): the covariance is inflated when it exceeds TDOA_SIGMA_M²."""
    r = -pair_errors(pairs, np.array(pose.position))
    m = len(r)
    correlation = (1 - solver.PAIR_CORRELATION) * np.eye(m) + solver.PAIR_CORRELATION * np.ones((m, m))
    return r @ np.linalg.solve(correlation, r) / (m - n_unknowns)


def analyse(path:str, true_position:list[float]|None):
    anchors, cycles = extract_cycles(path)
    anchor_ids = {tuple(pos): anchor_id for anchor_id, pos in anchors.items()}
    n_unknowns = 2 if SOLVER["mode_2d"] else 3
    print(f"\n[INFO] {len(cycles)} cycles, {sum(len(c) for c in cycles)} TDOA pairs, "
          f"{'2D' if SOLVER['mode_2d'] else '3D'} solve (SOLVER in config.py)")
    if len(cycles) < 30:
        print("[WARNING] Fewer than 30 cycles, the estimate will be rough. Record for longer.")

    # 1. Reference position
    poses = solve(cycles, anchors)
    solved = [p for p in poses if p is not None]
    if true_position is not None:
        center = np.array(true_position, dtype=float)
        print(f"[INFO] Using the surveyed position {center} m. Biases are measured against the truth.")
    elif solved:
        center = np.median(np.array([p.position for p in solved]), axis=0)
        print(f"[INFO] Using the median solved position {np.round(center, 3)} m (no --true-position given). "
              "Biases are only relative: part of them is absorbed into this position.")
    else:
        print("[ERROR] No cycle could be solved, nothing to calibrate.")
        return

    # 2. Per anchor pair errors: bias = mean, noise = spread around it
    errors = defaultdict(list) # (ref_id, remote_id) -> [(cycle index, error)]
    for i, pairs in enumerate(cycles):
        if not pairs:
            continue
        for (_, a1, a2), e in zip(pairs, pair_errors(pairs, center)):
            errors[(anchor_ids[tuple(a1)], anchor_ids[tuple(a2)])].append((i, e))
    kept, biases, dropped = {}, {}, 0
    print(f"\n=== Per anchor pair error (pairs with >= {MIN_SAMPLES_PER_PAIR} samples)")
    print("   ref - remote      bias [m]   noise std [m]   samples")
    for key in sorted(errors, key=lambda k: -len(errors[k])):
        e = np.array([err for _, err in errors[key]])
        median = np.median(e)
        robust_std = max(1.4826 * np.median(np.abs(e - median)), 0.01)
        inliers = np.abs(e - median) <= OUTLIER_ROBUST_STDS * robust_std
        dropped += int(np.sum(~inliers))
        if inliers.sum() < MIN_SAMPLES_PER_PAIR:
            continue
        kept[key] = [s for s, ok in zip(errors[key], inliers) if ok]
        biases[key] = e[inliers].mean()
        print(f"   {key[0]:>3} - {key[1]:<6}    {biases[key]:+8.3f}   {e[inliers].std(ddof=1):13.3f}   {inliers.sum():7d}")
    if not kept:
        print("[ERROR] No anchor pair has enough samples. Record for longer.")
        return
    dof = sum(len(s) - 1 for s in kept.values())
    sigma = np.sqrt(sum((len(s) - 1) * np.var([e for _, e in s], ddof=1) for s in kept.values()) / dof)
    bias_rms = np.sqrt(np.mean([b**2 for b in biases.values()]))
    print(f"   {dropped} outlier samples dropped")

    # Correlation between pairs of the same cycle, after removing each pair's bias (the solver assumes PAIR_CORRELATION)
    per_cycle = defaultdict(list)
    for key, samples in kept.items():
        for i, e in samples:
            per_cycle[i].append(e - biases[key])
    squares = [e**2 for es in per_cycle.values() for e in es]
    products = [es[a]*es[b] for es in per_cycle.values() for a in range(len(es)) for b in range(a+1, len(es))]
    rho = np.mean(products) / np.mean(squares) if products else float("nan")

    # 3. Consistency check with the recommended value
    solver.TDOA_SIGMA_M = float(sigma)
    solved = [(pairs, p) for pairs, p in zip(cycles, solve(cycles, anchors)) if p is not None]
    wild = [p for _, p in solved if np.linalg.norm(np.array(p.position) - center) > OUTLIER_POSE_DISTANCE_M]
    solved = [(pairs, p) for pairs, p in solved if p not in wild]
    poses = [p for _, p in solved]
    positions = np.array([p.position for p in poses])[:, :n_unknowns]
    observed_std = positions.std(axis=0, ddof=1)
    reported_std = np.sqrt(np.mean([np.diag(p.covariance)[:n_unknowns] for p in poses], axis=0))
    nees_xy = np.array([d @ np.linalg.solve(p.covariance[:2, :2], d)
                        for p, d in zip(poses, positions[:, :2] - center[:2])])
    inflated = np.mean([residual_s2(pairs, p, n_unknowns) > sigma**2 for pairs, p in solved if len(pairs) > n_unknowns])

    print("\n=== Results")
    print(f"   Pair noise std (pooled, bias removed): {sigma:.3f} m")
    print(f"   Pair bias RMS:                         {bias_rms:.3f} m")
    print(f"   Pair correlation within a cycle:       {rho:.2f} (solver assumes {solver.PAIR_CORRELATION})")
    axes = "xyz"[:n_unknowns]
    print(f"   With TDOA_SIGMA_M = {sigma:.3f}: {len(poses)}/{len(cycles)} poses "
          f"({len(wild)} wild poses further than {OUTLIER_POSE_DISTANCE_M} m left out of this check)")
    print(f"     observed std  " + "  ".join(f"{a}={s:.3f}" for a, s in zip(axes, observed_std)))
    print(f"     reported std  " + "  ".join(f"{a}={s:.3f}" for a, s in zip(axes, reported_std)) +
          "  (rms of the covariance diagonal)")
    print(f"     xy NEES vs {'truth' if true_position is not None else 'median'}: {nees_xy.mean():.2f} (2 = consistent, "
          f"lower = covariance too large, higher = too small)")
    print(f"     poses the EKF would reject ({EKF_GATE_SIGMAS:.0f} sigma gate): {np.mean(nees_xy > EKF_GATE_SIGMAS**2)*100:.1f}%")
    print(f"     cycles where the bad-cycle inflation grows the covariance: about {inflated*100:.0f}%")

    print(f"\n=== Recommendation: set TDOA_SIGMA_M = {sigma:.2f} in src/solver.py")
    if bias_rms > sigma:
        print(f"[WARNING] The pair biases ({bias_rms:.3f} m RMS) are larger than the noise. They make the pairs disagree on "
              "most cycles, so the covariance is set by the bad-cycle inflation rather than by TDOA_SIGMA_M.\n"
              "          Check the anchor positions and antenna delays before relying on this calibration.")
    if abs(rho - solver.PAIR_CORRELATION) > 0.2:
        print(f"[WARNING] The measured pair correlation ({rho:.2f}) differs from PAIR_CORRELATION ({solver.PAIR_CORRELATION}).")


def main():
    parser = argparse.ArgumentParser(description="Record stationary data and report what TDOA_SIGMA_M should be.")
    parser.add_argument("--pkl", help="Analyse this recording instead of recording a new one")
    parser.add_argument("--duration", type=float, default=60, help="Recording duration in seconds (default 60)")
    parser.add_argument("--true-position", type=float, nargs=3, metavar=("X", "Y", "Z"),
                        help="Surveyed tag position in meters, in the anchor frame")
    args = parser.parse_args()

    path = args.pkl
    if path is None:
        batches = record(args.duration)
        path = f"calibration_{datetime.now():%Y%m%d-%H%M%S}.pkl"
        with open(path, "wb") as file:
            pickle.dump(batches, file)
        print(f"[INFO] Recording saved to {path} (re-analyse it with --pkl)")
    analyse(path, args.true_position)


if __name__ == "__main__":
    main()
