# Plan: Add ROS 2 output (robot_localization EKF) to DW1000-TDoA3 — rev. 2

## Context
- Repo: DW1000-TDoA3. A Python TDoA3 tag on a DW1000, listening to Bitcraze LPS anchors.
  - `src/tdoa3_tag.py`: `TDOA3_Tag.run()` loops: listen for 1/LOCALIZATION_FREQUENCY s → `aggregate_raw_pkts` → `process_anchor_data` → `multilaterate` → print/CSV (inline in run()).
    - Imports `dw1000_spi` (needs spidev) at module level, so it cannot be imported on a laptop/Windows. This is why the tests import the synthetic copy instead.
  - `src/config.py`: plain constants (has a TODO for output control).
  - `src/main.py`: entry point. Ignores TAG_ID/SPI_BUS/SPI_CS from config (hard-codes id=1).
  - `src/tdoa3_tag_synthetic.py`: near-copy of the class that replays `.pkl` recordings (`run_synthetic`). Pickles hold `[(pkt, rx), ...]` per batch, no host times.
  - `testing/`: unittest-based tests (`test_tdoa3_tag_synthetic.py`), `stationary.csv` (older format, no `aggregated_data_size` column, ~1 Hz), `straight_line.csv` (~7.6 Hz), `csv_plotting.py` (reads `position_x/y/z` only).
- Target: ROS 2 **Jazzy** on **Ubuntu 24.04** (system Python 3.12). The driver runs on the robot computer.
- Consumer: the team's standard `robot_localization` `ekf_node` (`/mobile_manip/ekf_filter_node`):
  - `pose0: /mobile_manip/sensors/uwb/pose`. Fuses **x, y only**.
  - `pose0_pose_rejection_threshold: 4.0` (Mahalanobis gate in σ units, so the covariance must be honest)
  - `pose0_differential: false`, `pose0_relative: false` (absolute map coordinates)
  - `two_d_mode: true`, `world_frame: map`, `frequency: 50`, `sensor_timeout: 0.2`, `publish_tf: false`
  - `smooth_lagged_data` not enabled: late measurements are applied to the current state without rewinding. Keep latency low (see the Step 3 timestamp decision).
- Assumptions (for now): anchor coordinate frame == `map`; the UWB antenna is at `base_link` (no lever arm, no static TF).
- Architecture: **loose coupling**. Publish the solved pose with covariance (robot_localization has no TDOA measurement model).

## Decisions
- ROS is optional: `rclpy`/`geometry_msgs` are imported **only** inside `src/outputs/ros2_output.py`, and only if "ros2" is in `OUTPUTS`. rclpy is not in requirements.txt (it comes from apt via `source /opt/ros/jazzy/setup.bash`).
- The solver lives in a **pure module** (`src/solver.py`, numpy/scipy only) used by both the real and synthetic tag, so it is testable anywhere and does not drift between the two copies.
- Output controllers receive a per-cycle `CycleUpdate` (counts + optional `TDOA3_Pose`), not a bare `TDOA3_Pose`, so CSV can keep logging cycles with no solution.
- 2D solve option: z fixed at a configurable antenna height, solve x,y only.
- TDOA noise: configurable, default σ = 0.10 m (range-difference std dev), with the shared-reference correlation modelled.
- Message: `geometry_msgs/msg/PoseWithCovarianceStamped` on `/mobile_manip/sensors/uwb/pose`, frame_id `map`.

## Order of work
0 (spike) → 1 (types + solver extraction) → 2 (config + output controllers refactor) → 3 (pose timestamps) → 4 (solver features) → 5 (CSV columns) → 6 (ROS 2 output controller) → 7 (synthetic replay) → 8 (integration) → 9 (docs).
Each step is its own commit; steps 1 and 2 must not change behaviour.

## Step 0: Environment spike (on the robot computer, throwaway code)
- `source /opt/ros/jazzy/setup.bash`, `python3 -m venv --system-site-packages .venv`, activate, `pip install -r requirements.txt`.
- Verify `import rclpy`, `from geometry_msgs.msg import PoseWithCovarianceStamped`, and that constructing and publishing a message works (assigning the 36-float covariance exercises the NumPy-backed typesupport).
- **Expect** the NumPy clash: requirements.txt pins `numpy==2.5.4`, but Jazzy's generated message typesupport is built against Ubuntu's NumPy 1.26 ("module compiled using NumPy 1.x cannot be run in NumPy 2.x"). If it happens, create `requirements-ros.txt` with `numpy<2` **and** a scipy that still supports NumPy 1.26 on Python 3.12 (scipy 1.18 likely won't; try the newest 1.15/1.16 that resolves). Record the working pins.
- Verify `rclpy.init(signal_handler_options=SignalHandlerOptions.NO)` is accepted, and that Ctrl-C then raises a normal `KeyboardInterrupt`.
- Verify SPI access without root (user in the `spidev` group).
- Publish a fake constant pose and confirm the EKF is subscribed: `ros2 topic info -v /mobile_manip/sensors/uwb/pose`.

## ✅ Step 1: Result types + solver extraction (no behaviour change)
- New `src/pose.py`:
  - `TDOA3_Pose` dataclass:
    - `x, y, z: float`
    - `covariance: np.ndarray` (3x3, m², order x,y,z; in 2D mode the z row/col is 0 and is replaced at output). Until Step 4 fills it: NaN-filled.
    - `ref_anchor: int | None`, `mode_2d: bool`
    - No timestamp or pair count of its own: those of the carrying `CycleUpdate` apply (see Step 3).
  - `CycleUpdate` dataclass (what output controllers get, one per loop iteration):
    - `stamp_ns: int` (cycle host wall time, taken right after the solve; used by the CSV timestamp column and as the pose's timestamp)
    - `n_known_anchors: int` (`len(self.anchors)`), `n_aggregated: int` (`len(aggregated_data)`), `n_pairs: int`
    - `pose: TDOA3_Pose | None`
- New `src/solver.py`: `multilaterate(tdoa_data, initial) -> TDOA3_Pose | None` (Step 4 adds the solver config), a straight move of today's logic (3D, huber, f_scale 0.7, ≥3 pairs). No `dw1000_spi` import.
- `TDOA3_Tag.multilaterate` and the synthetic copy become thin wrappers calling `solver.multilaterate`. Keep `self.position` updated for now.
- Test: move the existing solver behaviour under a unit test in `testing/test_solver.py` (synthetic geometry, known position recovered).

## ✅ Step 2: Config + output controller layer (pure refactor)
- `src/config.py`:
```python
OUTPUTS = ["console"]          # any of: "console", "csv", "ros2"  (replaces SAVE_TO_CSV)
CSV_SAVE_INTERVAL_SECONDS = 3600
```
  (`SOLVER` and `ROS2` dicts are added in Steps 4 and 6.)
- Layout:
```
src/outputs/
  __init__.py     # make_output_controllers(outputs, tag_id) -> list[OutputController]; lazy import per name; NO ROS imports at module level
  base.py         # class OutputController: publish(update: CycleUpdate) -> None; close() -> None
  console.py      # ConsoleOutputController: current print behaviour (incl. the "Sending a processed list of size N" line)
  csv_output.py   # CsvOutputController: current CSV buffering/flush logic moved out of TDOA3_Tag, identical columns
  ros2_output.py  # Step 6
```
- `make_output_controllers` maps names → modules with `importlib.import_module` inside the function. Unknown name → `ValueError`. The "ros2" entry is registered in Step 6, and there an `ImportError` is re-raised as: "ROS 2 output selected but rclpy not importable. Did you `source /opt/ros/jazzy/setup.bash` and create the venv with --system-site-packages?"
- `CsvOutputController` keeps today's semantics: one row per cycle, blank coordinates when `pose is None`, file `tdoa3_tag_<id>.csv`, flush on row limit (`ceil(LOCALIZATION_FREQUENCY * CSV_SAVE_INTERVAL_SECONDS)`) or interval, flush on close.
- `TDOA3_Tag.__init__(id, bus, cs, outputs=OUTPUTS)` builds the output controllers. `run()` (no more `enable_print` argument) builds a `CycleUpdate` and calls `controller.publish(update)` for each output controller. `__exit__` closes every output controller (each in its own try/except, errors logged), then the radio in a `finally`.
- Remove `_csv_rows` / `_save_csv_rows` / `SAVE_TO_CSV` from TDOA3_Tag.
- `main.py`: `TDOA3_Tag(id=TAG_ID, bus=SPI_BUS, cs=SPI_CS)`, `tag.run()`. Note: CSV file becomes `tdoa3_tag_10.csv` with the current `TAG_ID`.
- Verify: same CSV header and row format as before on a hardware run (or with a fake radio).

## ✅ Step 3: Pose timestamps
- **Decision: poses are stamped with the host wall time at solve time**, i.e. `CycleUpdate.stamp_ns` (`time.time_ns()` taken in `run()` right after the solve), not with the host time at which the packets were received. `TDOA3_Pose` has no stamp of its own, and `solver.multilaterate` does not read the clock.
  - The publishing delay is the same either way: a pose is always published at the end of its listening window. Only the stamp differs. With a solve-time stamp, the pose describes where the tag was up to one listening window (`1/LOCALIZATION_FREQUENCY` s) plus processing time before its stamp.
  - With `smooth_lagged_data` disabled, robot_localization applies late measurements to the current state without rewinding, so an exact (older) receive-time stamp would be fused almost exactly like a solve-time stamp. Receive-time stamping was implemented and then removed, because the extra code bought almost nothing in this setup.
  - Resulting error: at most `speed × (1/LOCALIZATION_FREQUENCY + processing time)`, about half that on average, typically well below the expected UWB noise.
  - **Revisit** (stamp with the reference anchor's packet receive time instead) if `smooth_lagged_data` / `history_length` is enabled in the EKF, or if accurate UWB timing relative to the other sensors is needed in bags. That would bring back a `stamp_ns` on `TDOA3_Pose` (the measurement time, distinct from the cycle time).
- `process_anchor_data` returns `(pairs, ref_anchor)`: the winning reference anchor id, or None when there are no pairs. `run()` passes it to `multilaterate`, which copies it to `TDOA3_Pose.ref_anchor` (for the Step 5 CSV column and debugging). The synthetic copy is handled in Step 7.
- `run()` prints at startup how far behind its stamp a pose can be, computed from `LOCALIZATION_FREQUENCY`.
- Known limitation (document, don't fix): remote messages can be slightly older than the reference message, giving a small motion error per pair.

## Step 4: Solver features (`src/solver.py`)
- Config:
```python
SOLVER = {
    "mode_2d": False,
    "antenna_height_m": 0.0,      # z used when mode_2d is True
    "warm_start": False,          # start each solve from the last pose instead of the anchor centroid
    "max_residual_rms_m": None,   # reject poses whose pairs disagree by more than this (m), None = disabled
}
```
- **2D mode**: optimise [x, y] only; the residual function builds p = (x, y, antenna_height_m). Unknowns n = 2 (3D: n = 3).
- **Gates** (return None):
  - `len(tdoa_data) < MIN_PAIRS` (hard-coded in `solver.py`, not configurable): 3 normally, raised to `MIN_PAIRS_RMS_GATE = 4` when `max_residual_rms_m` is set (see below).
  - Solver failure, any NaN/inf in x or covariance, or `JᵀJ` not invertible (`LinAlgError`). There is no condition-number setting.
  - `max_residual_rms_m` when set (see below).
- **`max_residual_rms_m`** (optional, the only quality setting exposed to the user):
  - Quantity: `rms = sqrt(Σ rᵢ² / (m − n))`, computed from the plain residuals (not huber-weighted) at the solution, in meters. It is the per-pair disagreement this cycle, directly comparable to `TDOA_SIGMA_M`.
  - Reject the pose when `rms > max_residual_rms_m`.
  - Needs redundant pairs: with m = n the solve fits every pair exactly and the RMS is meaningless (division by zero). Hence the higher `MIN_PAIRS_RMS_GATE = 4` when the gate is on, so 3D always has ≥ 1 and 2D ≥ 2 redundant pairs. Cycles with only 3 pairs are dropped while it is enabled.
  - README explains what it is and how to set it (Step 9).
- **Covariance**: computed automatically every cycle at the solution `p̂` with m pairs. The noise model is **hard-coded in `solver.py`, not configurable**: the user gets a pose and a covariance without having to model uncertainty.
  - Module constants (not in `config.py`):
    - `TDOA_SIGMA_M`: std dev of one pair's range difference, in meters. Calibrated once by us (see below), starting value 0.10.
    - `PAIR_CORRELATION = 0.5`: every pair reuses the reference anchor's packet (its rx timestamp is in every pair), so any two pairs share half their noise when the tag's timestamp noise dominates.
  - 1. **Geometry**: recompute the plain (not huber-scaled) Jacobian analytically: row i = (p-a_ref)/|p-a_ref| − (p-a_i)/|p-a_i| (only the x,y columns in 2D). `result.jac` is loss-scaled, so don't use it.
  - 2. **Pair noise**: `R = σ² · C`, with `C = (1 − ρ)·I + ρ·11ᵀ` (diagonal 1, off-diagonal ρ = ½), σ = `TDOA_SIGMA_M`.
  - 3. **Position noise**: the solve is unweighted least squares, so use the matching sandwich formula `cov = (JᵀJ)⁻¹ · Jᵀ R J · (JᵀJ)⁻¹`. Don't whiten the solve to use the simpler `(JᵀR⁻¹J)⁻¹`: whitening mixes pairs, so one bad pair leaks into all of them and huber can no longer isolate it. If `JᵀJ` can't be inverted, reject the pose. Symmetrise `(cov + covᵀ)/2`.
  - 4. **Bad-cycle inflation**: when m > n, `s² = rᵀ C⁻¹ r / (m − n)` (how much the pairs actually disagreed this cycle); multiply `cov` by `max(1, s² / σ²)`. It only ever grows the covariance, never shrinks it below the calibrated level.
  - Result: 3x3 (2x2 block + z = 0 in 2D) in `TDOA3_Pose.covariance`. Flooring and the unused-axis variances are done at the ROS output (Step 6).
  - **Calibration of `TDOA_SIGMA_M`** (once, by us): replay the stationary `.pkl` recordings and pick the σ for which the computed covariance matches the observed spread at that spot (`stationary.csv`: σx ≈ 0.07 m, σy ≈ 0.13 m, 3D solve). Then rerun in 2D mode and check it still matches. Record the value and the data it came from in a comment next to the constant.
- **Warm start** (`SOLVER["warm_start"]`, off by default): the solver gets `initial` from the tag. Off: always the anchor centroid (z = antenna height in 2D). On: the last successful pose's position (centroid until there is one), with no age limit. Only successful solves update it.
  - Known risk: a bad solve that still passes the gates seeds the next one and can keep it in a wrong minimum. Accepted for now since it is off by default.
  - Later (out of scope): seed from the EKF's `/mobile_manip/odometry/filtered` instead of the last UWB pose.

## Step 5: CSV extra columns
- Append `ref_anchor, cov_xx, cov_yy, cov_xy` to the CSV row (blank when no pose). Existing columns unchanged and first, so `csv_plotting.py` keeps working.
  - Covariance entries are converted to UNITS² (`units.covariance_from_meters`) to match the coordinates, and written with 4 significant digits rather than rounded to DECIMALS (variances are often below it).
- Existing files are never appended to: if `tdoa3_tag_<id>.csv` exists, the run writes to `tdoa3_tag_<id>_<n>.csv` with a random unused n in [0, 100] (timestamp suffix if all are taken) and prints a notice. The name is chosen once per run, when the output controller is created. This makes a header check unnecessary.

## Step 6: `src/outputs/ros2_output.py`
- Config:
```python
ROS2 = {
    "node_name": "uwb_tdoa3_tag",
    "topic": "/mobile_manip/sensors/uwb/pose",   # absolute name
    "frame_id": "map",
    "min_variance_m2": 0.05**2,  # floor on x/y(/z) variance
    "unused_variance": 1e3,      # z (in 2D), roll, pitch, yaw diagonal
    "queue_depth": 10,
    "skip_log_every": 50,        # log skipped fixes every N
}
```
- `__init__`: if not `rclpy.ok()`, `rclpy.init(signal_handler_options=SignalHandlerOptions.NO)` so Ctrl-C stays a plain `KeyboardInterrupt` handled by the tag's try/finally. `create_node(node_name)`, `create_publisher(PoseWithCovarianceStamped, topic, queue_depth)` (default reliable QoS). Publish-only, no spin.
- `publish(update)`: skip if `update.pose` is None or non-finite; count skips and log every `skip_log_every`.
  - `header.stamp` from `update.stamp_ns`: `sec = stamp_ns // 1_000_000_000`, `nanosec = stamp_ns % 1_000_000_000` (integer math). use_sim_time is false, so system time matches the other sensors.
  - `header.frame_id = frame_id`
  - `pose.position = (x, y, z)` in meters straight from the solver, ignoring `UNITS`/`DECIMALS` (ROS uses meters, REP 103; those settings only apply to the console and CSV outputs) (z = antenna height in 2D); `pose.orientation = (0, 0, 0, 1)` (MUST be a valid quaternion).
  - `covariance = build_covariance_array(pose, cfg)`.
- `build_covariance_array(pose, cfg) -> list[float]` is a **pure function with no ROS imports**, kept in a ROS-free module (e.g. `src/outputs/covariance.py`) so tests can import it without rclpy. 36 floats, row-major, order x,y,z,rot_x,rot_y,rot_z:
  - [0] = max(cov_xx, min_var), [7] = max(cov_yy, min_var), [1] = [6] = cov_xy
  - 3D: [14] = max(cov_zz, min_var), [2]/[12] = cov_xz, [8]/[13] = cov_yz
  - 2D: [14] = unused_variance, z cross terms 0
  - [21], [28], [35] = unused_variance
  - After flooring, keep the matrix PD: clamp |cov_xy| ≤ 0.99·sqrt([0]·[7]) (and the same for xz/yz in 3D).
- `close()`: `node.destroy_node()`, then `rclpy.shutdown()` only if `rclpy.ok()`. Both wrapped so a failure doesn't block closing the radio.
- Tests: `build_covariance_array` indices/floors/PD (no ROS); `make_output_controllers(["console","csv"])` leaves `"rclpy" not in sys.modules`; ROS-only tests behind `unittest.skipUnless(importlib.util.find_spec("rclpy"))`.

## Step 7: Synthetic replay through the same output controllers
- `run_synthetic(path, outputs=...)` builds `CycleUpdate`s (its `process_anchor_data` returns `(pairs, ref_anchor)` like the real tag) and uses `make_output_controllers`, so recorded `.pkl` data can be published to the EKF from a laptop (stamped with `CycleUpdate.stamp_ns`, as in Step 3). Optional `realtime=True` sleeps `RECORDING_DURATION` per batch.
- The mean/median/std summary stays in `run_synthetic` as an analysis printout (computed from the results), not as an output controller.
- Synthetic and real tag share `solver.py`. Turning the synthetic tag into a subclass of `TDOA3_Tag` (with a lazily created radio) is still optional.

## Step 8: Integration (manual, on the robot)
- `ros2 topic hz /mobile_manip/sensors/uwb/pose` (expect ≈ LOCALIZATION_FREQUENCY minus skipped cycles), `ros2 topic echo ...` (check stamp, frame, covariance).
- `ros2 topic info -v` shows `ekf_filter_node` subscribed.
- Run the EKF with the supervisor's YAML + the L-shape bags (T265 + wheel odom) + UWB live or replayed. Inspect `/mobile_manip/odometry/filtered` in RViz2 and `/diagnostics` for pose0 rejections. A high rejection rate means the covariance is too small: recalibrate `TDOA_SIGMA_M` in `solver.py`, and check for per-anchor biases (NLOS, antenna delay), which no covariance can fix.

## Step 9: Docs
- README "ROS 2 output" section: Jazzy/Ubuntu 24.04, `source /opt/ros/jazzy/setup.bash`, venv with `--system-site-packages`, the NumPy/scipy pins from Step 0, enabling via `OUTPUTS`, topic/message/frame/covariance details, the 2D option, the Step 3 timestamp decision (solve-time stamps, the resulting error as a function of `LOCALIZATION_FREQUENCY`, and when to switch to receive-time stamps), and the assumptions (anchor frame == map, antenna at base_link).
- README "Position uncertainty" section (under Code Implementation) explaining the covariance model in plain terms, for users who never need to touch it:
  - What the covariance means (expected position error, m²) and that it is computed automatically every cycle, no settings.
  - The four steps: pair geometry (Jacobian), pair noise with the shared-reference correlation (σ, ρ = ½, and why the pairs are correlated), the sandwich formula, and the bad-cycle inflation.
  - Why it is not a fixed number: it grows near the edges of the anchor setup, with fewer pairs, and on noisy cycles, which is when the EKF most needs to trust UWB less.
  - How `TDOA_SIGMA_M` was calibrated (which recording, observed spread) and when to recalibrate (new anchor layout or environment, frequent EKF rejections).
  - Limitation: biases (NLOS, anchor antenna-delay errors) are not noise and are not represented in the covariance.
- README `max_residual_rms_m` explanation (with the solver settings):
  - What it is: the RMS disagreement between the TDOA pairs at the solved position, in meters. High values mean no single position explains all pairs (outlier pair, NLOS, wrong solution).
  - How to set it: replay or record a run, look at the typical RMS (logged/printed), and set the threshold a few times above it (roughly 3–5× `TDOA_SIGMA_M`). Too low drops good poses; leave it `None` when publishing to the EKF only, since bad cycles already get an inflated covariance there.
  - That enabling it requires at least 4 pairs per cycle, so fewer poses come out in sparse anchor coverage.
- Update the README's existing CSV section (`OUTPUTS` replaces `SAVE_TO_CSV`, new columns, header-mismatch behaviour) and the "Main localization loop" TODO step 4.
