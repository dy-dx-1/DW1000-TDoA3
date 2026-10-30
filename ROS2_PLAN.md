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
  - `smooth_lagged_data` not enabled: late measurements are applied to the current state without rewinding. Keep latency low and stamps accurate.
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
0 (spike) → 1 (types + solver extraction) → 2 (config + output controllers refactor) → 3 (host timestamps) → 4 (solver features) → 5 (CSV columns) → 6 (ROS 2 output controller) → 7 (synthetic replay) → 8 (integration) → 9 (docs).
Each step is its own commit; steps 1 and 2 must not change behaviour.

## Step 0: Environment spike (on the robot computer, throwaway code)
- `source /opt/ros/jazzy/setup.bash`, `python3 -m venv --system-site-packages .venv`, activate, `pip install -r requirements.txt`.
- Verify `import rclpy`, `from geometry_msgs.msg import PoseWithCovarianceStamped`, and that constructing and publishing a message works (assigning the 36-float covariance exercises the NumPy-backed typesupport).
- **Expect** the NumPy clash: requirements.txt pins `numpy==2.5.4`, but Jazzy's generated message typesupport is built against Ubuntu's NumPy 1.26 ("module compiled using NumPy 1.x cannot be run in NumPy 2.x"). If it happens, create `requirements-ros.txt` with `numpy<2` **and** a scipy that still supports NumPy 1.26 on Python 3.12 (scipy 1.18 likely won't; try the newest 1.15/1.16 that resolves). Record the working pins.
- Verify `rclpy.init(signal_handler_options=SignalHandlerOptions.NO)` is accepted, and that Ctrl-C then raises a normal `KeyboardInterrupt`.
- Verify SPI access without root (user in the `spidev` group).
- Publish a fake constant pose and confirm the EKF is subscribed: `ros2 topic info -v /mobile_manip/sensors/uwb/pose`.

## Step 1: Result types + solver extraction (no behaviour change)
- New `src/pose.py`:
  - `TDOA3_Pose` dataclass:
    - `stamp_ns: int` (host time, `time.time_ns()` domain)
    - `x, y, z: float`
    - `covariance: np.ndarray` (3x3, m², order x,y,z; in 2D mode the z row/col is 0 and is replaced at output). Until Step 4 fills it: NaN-filled.
    - `n_pairs: int`, `ref_anchor: int | None`, `mode_2d: bool`
  - `CycleUpdate` dataclass (what output controllers get, one per loop iteration):
    - `stamp_ns: int` (cycle wall time, used by the CSV timestamp column)
    - `n_known_anchors: int` (`len(self.anchors)`), `n_aggregated: int` (`len(aggregated_data)`), `n_pairs: int`
    - `pose: TDOA3_Pose | None`
- New `src/solver.py`: `multilaterate(tdoa_data, initial, cfg) -> TDOA3_Pose | None`, a straight move of today's logic (3D, huber, f_scale 0.7, ≥3 pairs). No `dw1000_spi` import.
- `TDOA3_Tag.multilaterate` and the synthetic copy become thin wrappers calling `solver.multilaterate`. Keep `self.position` updated for now.
- Test: move the existing solver behaviour under a unit test in `testing/test_solver.py` (synthetic geometry, known position recovered).

## Step 2: Config + output controller layer (pure refactor)
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

## Step 3: Host timestamps
- In `run()`, record `host_ns = time.time_ns()` right after each `self._dw.listen()` returns a packet; collect `(pkt, rx, host_ns)`.
- `aggregate_raw_pkts` accepts both `(pkt, rx)` and `(pkt, rx, host_ns)` tuples (synthetic pickles have no host time → `host_ns=None`). Store `'host_ns'` in each history dict `{seq, rx, tx, remote_data, host_ns}`.
- `_tdoa_from_ref` / `process_anchor_data` also return the winning reference anchor id and its `cur['host_ns']`. Since only fresh anchors can be the reference, `cur` is always from the current batch.
- TDOA3_Pose stamp = reference `cur['host_ns']`, or `time.time_ns()` at solve time if it is None.
- Known limitations (document, don't fix): the pose can be up to ~1 batch (100 ms) old when published, about 5 cm of error at 0.5 m/s with no lag smoothing. Remote messages can be slightly older than `cur`, giving a small motion error per pair.

## Step 4: Solver features (`src/solver.py`)
- Config:
```python
SOLVER = {
    "mode_2d": False,
    "antenna_height_m": 0.0,      # z used when mode_2d is True
    "tdoa_sigma_m": 0.10,         # range-difference std dev per pair
    "correlated_pairs": True,     # model the shared-reference correlation
    "min_pairs": 3,
    "warm_start_timeout_s": 1.0,
    "max_cost": None,             # optional gate, None = disabled
    "max_condition": 1e8,         # reject ill-conditioned geometry
}
```
- **2D mode**: optimise [x, y] only; the residual function builds p = (x, y, antenna_height_m). Unknowns n = 2 (3D: n = 3).
- **Gates** (return None): `len(tdoa_data) < max(min_pairs, n)`, solver failure, any NaN/inf in x or covariance, `cost > max_cost` when set, condition number above `max_condition`.
- **Covariance** at the solution `p̂` with m pairs:
  - Recompute the plain (not huber-scaled) Jacobian analytically: row i = (p-a_ref)/|p-a_ref| − (p-a_i)/|p-a_i| (only the x,y columns in 2D). `result.jac` is loss-scaled, so don't use it.
  - Measurement correlation: every pair shares the reference anchor's rx timestamp. With `correlated_pairs`, use the normalised correlation matrix `C = (I + 11ᵀ)/2` (diagonal 1, off-diagonal ½), with `C⁻¹ = 2(I − 11ᵀ/(m+1))`; otherwise `C = I`. This is an approximation (TDoA3 also adds noise from the remote tx estimate), hence the flag.
  - Scale: `s² = rᵀ C⁻¹ r / (m − n)` when m > n, else 0; `σ² = max(tdoa_sigma_m², s²)`.
  - `cov = σ² · (Jᵀ C⁻¹ J)⁻¹`. Check `cond(Jᵀ C⁻¹ J)` before inverting. Symmetrise `(cov + covᵀ)/2`.
- **Warm start**: the solver gets `initial` from the tag; the tag keeps `(last_good_xyz, last_good_monotonic)` and passes it if younger than `warm_start_timeout_s`, otherwise the anchor centroid (z = antenna height in 2D). Only successful solves update it.
- Unit tests (`testing/test_solver.py`, no hardware or ROS): 2D vs 3D recovery on synthetic geometry with noise; covariance shape/symmetry/PD; covariance grows outside the anchor hull; correlated vs uncorrelated model; gates (too few pairs, NaN, ill-conditioned collinear anchors); warm-start timeout logic.
- Sanity check against data: `stationary.csv` shows σx ≈ 0.07 m, σy ≈ 0.13 m, σz ≈ 0.27 m (42 fixes, 3D solve). Replay the `.pkl` recordings and check the reported σ is the same order. Then rerun in 2D mode and compare.

## Step 5: CSV extra columns
- Append `stamp_ns, ref_anchor, cov_xx, cov_yy, cov_xy` to the CSV row (blank when no pose). Existing columns unchanged and first, so `csv_plotting.py` keeps working.
- Header check before appending: if the existing file's header differs from the current one, don't append. Start `tdoa3_tag_<id>_<YYYYmmdd-HHMMSS>.csv` and print a notice.

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
  - `header.stamp`: `sec = stamp_ns // 1_000_000_000`, `nanosec = stamp_ns % 1_000_000_000` (integer math). use_sim_time is false, so system time matches the other sensors.
  - `header.frame_id = frame_id`
  - `pose.position = (x, y, z)` (z = antenna height in 2D); `pose.orientation = (0, 0, 0, 1)` (MUST be a valid quaternion).
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
- `run_synthetic(path, outputs=...)` builds `CycleUpdate`s and uses `make_output_controllers`, so recorded `.pkl` data can be published to the EKF from a laptop (stamp = `time.time_ns()` at solve). Optional `realtime=True` sleeps `RECORDING_DURATION` per batch.
- The mean/median/std summary stays in `run_synthetic` as an analysis printout (computed from the results), not as an output controller.
- Synthetic and real tag share `solver.py`. Turning the synthetic tag into a subclass of `TDOA3_Tag` (with a lazily created radio) is still optional.

## Step 8: Integration (manual, on the robot)
- `ros2 topic hz /mobile_manip/sensors/uwb/pose` (expect ≈ LOCALIZATION_FREQUENCY minus skipped cycles), `ros2 topic echo ...` (check stamp, frame, covariance).
- `ros2 topic info -v` shows `ekf_filter_node` subscribed.
- Run the EKF with the supervisor's YAML + the L-shape bags (T265 + wheel odom) + UWB live or replayed. Inspect `/mobile_manip/odometry/filtered` in RViz2 and `/diagnostics` for pose0 rejections. A high rejection rate means the covariance is too small (raise `tdoa_sigma_m`).

## Step 9: Docs
- README "ROS 2 output" section: Jazzy/Ubuntu 24.04, `source /opt/ros/jazzy/setup.bash`, venv with `--system-site-packages`, the NumPy/scipy pins from Step 0, enabling via `OUTPUTS`, topic/message/frame/covariance details, the 2D option, the latency limitation, and the assumptions (anchor frame == map, antenna at base_link).
- Update the README's existing CSV section (`OUTPUTS` replaces `SAVE_TO_CSV`, new columns, header-mismatch behaviour) and the "Main localization loop" TODO step 4.
