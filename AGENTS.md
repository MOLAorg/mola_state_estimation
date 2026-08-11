# AGENTS.md - mola_state_estimation

## Overview

This repository provides **state estimation and sensor fusion** for the
[MOLA](https://docs.mola-slam.org/latest/) (Modular Optimization framework for
Localization and mApping) framework. It fuses odometry, IMU, GNSS and pose
measurements to estimate a vehicle's pose, velocity and orientation over time.
It also provides offline georeferencing of keyframe-based maps.

Rules: keep this AGENTS.md in sync with architecture/code changes. Use American
spelling. Don't use en/em dashes. Don't sign commits as an AI agent. Keep 
explanations short and general, not "this fixes the problem we had with XXX".
Use clang-format-14 before commiting.

License: GNU GPL v3 (commercial options available upon request).

## Repository layout

```
mola_state_estimation/              <- ROS 2 metapackage (no code, just dependency grouping)
mola_state_estimation_simple/       <- Lightweight constant-velocity kinematic estimator
mola_state_estimation_smoother/     <- Advanced iSAM2 factor-graph smoother
mola_gtsam_factors/                 <- Reusable custom GTSAM factor library
mola_georeferencing/                <- Georeferencing SimpleMaps with GNSS/IMU
docs/                               <- Sphinx/RST documentation sources
scripts/                            <- clang-format helpers
params/                             <- (inside smoother) default YAML configuration
```

## Sub-packages

### 1. `mola_state_estimation_simple`

A fast, minimal state estimator using a constant-velocity kinematic model with
incremental velocity extrapolation. Good enough for LiDAR-only odometry (LO/LIO)
on most automotive datasets.

| Item | Path |
|------|------|
| Main class | `mola_state_estimation_simple/include/.../StateEstimationSimple.h` |
| Parameters | `mola_state_estimation_simple/include/.../Parameters.h` |
| Implementation | `mola_state_estimation_simple/src/StateEstimationSimple.cpp` |
| Test | `mola_state_estimation_simple/tests/test-state-estimation-simple.cpp` |

Key traits:
- Inherits `mola::NavStateFilter` (abstract interface defined in `mola_kernel`).
- Fuses pose, odometry, IMU and twist; ignores GNSS.
- Not frame-aware (ignores `frame_id`).
- Supports optional planar-motion enforcement (`enforce_planar_motion`).
- `fuse_imu()` only **buffers** the reading (`state_.pending_imu`). It is fused
  by `fuse_pending_imu_up_to()`, called at the start of every method that
  carries a time of interest (`estimated_navstate()`, `fuse_pose()`,
  `fuse_twist()`, `fuse_odometry*()`), which fuses exactly the readings not
  newer than that time, in timestamp order. `get_last_twist()`, having no time
  of interest, fuses everything buffered. So the estimate for a given time is a
  function of the measurement timestamps, not of the delivery order: a pipeline
  feeding this estimator from concurrent sensor threads stays reproducible.

### 2. `mola_state_estimation_smoother`

An advanced sliding-window factor-graph smoother using GTSAM's iSAM2 for
incremental optimization. The primary estimator for multi-sensor fusion.

| Item | Path |
|------|------|
| Main class | `mola_state_estimation_smoother/include/.../StateEstimationSmoother.h` |
| Parameters | `mola_state_estimation_smoother/include/.../Parameters.h` |
| Implementation | `mola_state_estimation_smoother/src/StateEstimationSmoother.cpp` |
| Default config | `mola_state_estimation_smoother/params/state-estimation-smoother.yaml` |
| ROS 2 launch | `mola_state_estimation_smoother/ros2-launchs/ros2-state-estimator.launch.py` |
| MOLA-CLI launch | `mola_state_estimation_smoother/mola-cli-launchs/state_estimator_ros2.yaml` |
| CLI app | `mola_state_estimation_smoother/apps/mola-navstate-cli.cpp` |
| Tests (11) | `mola_state_estimation_smoother/tests/test-*.cpp` |
| Integration tests | `mola_state_estimation_smoother/test/integration/test_*.py` |

Key traits:
- Inherits `mola::NavStateFilter`, `mola::LocalizationSourceBase`,
  `mola::MapSourceBase`.
- Uses Pimpl pattern (`GtsamImpl`) to hide GTSAM details.
- Sliding time window of keyframes (default 6.0 s, `sliding_window_length`).
- Two kinematic models: `ConstantVelocity` and `Tricycle` (Ackermann).
- Multi-frame-aware: tracks multiple odometry sources by `frame_id`.
- `estimated_navstate(t, {odom_i})` is served **frame-local**: it anchors on the
  source's own last raw pose in `{odom_i}` (`State::last_raw_pose_by_source`,
  stored by `fuse_pose_locked`) and extrapolates by the body-twist increment
  (`body_twist_delta()`, kinematic-model-aware), NOT by reconstructing
  `X(kf) (-) T_map_to_odom_i`. This keeps a front end's short-term prediction
  continuous in its own frame, immune to `{map}` corrections (geo-ref / loop
  closure / per-solve jitter). The sliding window already kept that leak small
  (the two formulations agree to <1 mm in `test-navstate-odom-gnss-fusion`), so
  this is defensive; `mola_mapper_3d`'s NON-windowed central map is where the
  global reconstruction actually breaks (it injected meter/degree jumps into
  LIO's ICP guess after geo-ref). Falls back to the global conversion before a
  source's first `fuse_pose()`.
- Optional ENU-to-map georeferencing from GNSS.
- Thread-safe (`std::recursive_mutex`).
- Configuration via YAML with `${ENV_VAR|default}` substitution.
- Optional high-rate same-sensor decimation (both default `0` = off):
  `odometry_min_sample_period` and `imu_min_sample_period` cap how often a
  high-rate stream spawns keyframes/factors (solve cost grows with keyframe
  count). Wheel-odom drops are *merged* (the pose anchor is held across the
  dropped span, so the next kept reading fuses the accumulated increment +
  covariance -- no motion lost); IMU drops are skipped (attitude/gravity are
  absolute). Distinct from `min_time_difference_to_create_new_frame`, which only
  merges near-simultaneous readings from different sensors.
- Optional async backend (`async_backend: true`, default `false`): the iSAM2
  window solve runs in a dedicated thread and `estimated_navstate()` is served
  by a lock-free `FastPredictor` (`src/FastPredictor.{h,cpp}`) that extrapolates
  the latest solved Snapshot (`src/Snapshot.h`) with the configured kinematic
  model (shared with the backend via `src/extrapolation.h`). A query then runs
  no solve and never takes `stateMutex_`, so its latency stays sub-millisecond;
  the frame-local `{odom_i}` hardening is preserved (a query anchors on that
  source's own last raw pose in the Snapshot). The default synchronous path is
  byte-for-byte unchanged (deterministic offline/unit-test runs).
- Predict-twist low-pass (`predict_twist_filter_enabled: true`,
  `predict_twist_filter_time_const: 0.3` s): the velocity extrapolated by both
  the sync path and the `FastPredictor` is a dt-aware EMA of the newest
  keyframe's optimized twist, NOT the raw value. The newest keyframe is the
  boundary node of the sliding window (constrained on one side only), so its raw
  velocity is the noisiest state in the graph; extrapolating it un-damped feeds a
  jittery motion prior to a LiDAR front end, which starts ICP from a bad guess,
  slows/degrades registration, and (under real-time load) drops scans -- a
  feedback loop that produces meter-scale trajectory spikes on wheeled robots
  where the lightweight estimator (which low-passes velocity) stays smooth. The
  filter is a plain EMA, hence deterministic. Its cost is a short lag behind
  genuine acceleration; the paired sigma defaults below are moderate enough that
  the two together match the smoothest sigma-only tuning while generalizing
  better than very tight sigmas.
- Short-term extrapolation is covariance-aware: both the sync path and the
  `FastPredictor` share `extrapolate_pose_pdf()` (`src/extrapolation.h`), which
  returns the pose PDF of `anchor (+) Exp(twist*dt)` with first-order uncertainty
  propagation -- the anchor pose covariance transported through the composition
  (MRPT's `CPose3DPDFGaussian::operator+` Adjoint) plus the body-increment
  covariance (current-velocity uncertainty over the interval + acceleration
  process noise). The increment's tangent-to-pose Jacobian is approximated as
  identity (exact in translation, first order in rotation) in the sub-second
  extrapolation regime. This replaces the earlier mean-only extrapolation that
  left the pose covariance ungrown.

Sensor inputs:
- `fuse_pose()` - localization / LiDAR odometry poses
- `fuse_odometry()` - wheel odometry with uncertainty
- `fuse_imu()` - gravity alignment, angular velocity, attitude
- `fuse_gnss()` - GPS in ENU coordinates
- `fuse_twist()` - direct velocity measurements

### 3. `mola_gtsam_factors`

Reusable GTSAM factor classes for state estimation and georeferencing.

| Factor class | Purpose |
|--------------|---------|
| `FactorAngularVelocityIntegration` | Gyroscope-based rotation integration |
| `FactorConstLocalVelocity` | Constant-velocity prior in body frame |
| `FactorGnssEnu` | GNSS position measurement in ENU |
| `FactorGnssMapEnu` | GNSS with explicit ENU-to-map transform |
| `FactorTrapezoidalIntegrator` | Trapezoidal velocity integration |
| `FactorTricycleKinematic` | Ackermann / tricycle steering kinematics |
| `MeasuredGravityFactor` | Gravity-vector leveling from accelerometer |
| `Pose3RotationFactor` | Rotation-only constraint (decoupled from translation) |

Headers are in `mola_gtsam_factors/include/mola_gtsam_factors/`.
Implementations in `mola_gtsam_factors/src/`.
Most factors derive from `gtsam::ExpressionFactorN`; `Pose3RotationFactor`
derives from `gtsam::NonlinearFactor`.

### 4. `mola_georeferencing`

Offline georeferencing of MOLA SimpleMaps using GNSS and IMU observations.

| Item | Path |
|------|------|
| Library API | `mola_georeferencing/include/.../simplemap_georeference.h` |
| Implementation | `mola_georeferencing/src/simplemap_georeference.cpp` |
| CLI: georeference a simplemap | `mola_georeferencing/apps/mola-sm-georeferencing-cli.cpp` |
| CLI: georeference a trajectory | `mola_georeferencing/apps/mola-trajectory-georef-cli.cpp` |
| CLI: add geodetic info to maps | `mola_georeferencing/apps/mola-mm-add-geodetic-cli.cpp` |
| Test | `mola_georeferencing/tests/test_imu_attitude.cpp` |

Main function: `simplemap_georeference()` -- takes a `CSimpleMap` with GNSS
observations and returns an optimal ENU-to-map transformation + RMSE.

## Class hierarchy

```
mola::NavStateFilter  (from mola_kernel, abstract)
  |-- StateEstimationSimple
  |-- StateEstimationSmoother  (also: LocalizationSourceBase, MapSourceBase)

gtsam::ExpressionFactorN<...>
  |-- FactorAngularVelocityIntegration
  |-- FactorConstLocalVelocity
  |-- FactorGnssEnu / FactorGnssMapEnu
  |-- FactorTrapezoidalIntegrator
  |-- FactorTricycleKinematic
  |-- MeasuredGravityFactor

gtsam::NonlinearFactor
  |-- Pose3RotationFactor
```

## Key dependencies

| Dependency | Role |
|------------|------|
| **GTSAM** (>= 4.0) | Factor-graph optimization (iSAM2) |
| **MRPT** (poses, obs, maps) | Pose representations, sensor observations, maps |
| **mola_kernel** | `NavStateFilter` interface, MOLA module lifecycle |
| **mola_imu_preintegration** | IMU measurement handling |
| **mp2p_icp** | Point-cloud ICP (used by georeferencing) |

## Build

Standard colcon build from the ROS 2 workspace:

```bash
cd ~/ros2_ws
colcon build --packages-select \
  mola_gtsam_factors \
  mola_state_estimation_simple \
  mola_state_estimation_smoother \
  mola_georeferencing \
  mola_state_estimation
```

All packages use `ament_cmake` and require C++17.

## Tests

```bash
cd ~/ros2_ws
colcon test --packages-select \
  mola_state_estimation_simple \
  mola_state_estimation_smoother \
  mola_georeferencing
colcon test-result --verbose
```

## ROS 2 integration

The smoother integrates with ROS 2 via the `mola_launcher` node and a ROS 2
bridge. By default it advertises the fused `map -> base_link` pose (the bridge
publishes it to `/tf`, either directly or, in its REP-105 mode, composed into
`map -> odom`).

Optional additional TF outputs (both default off, additive, distinct
`method` suffixes so the bridge's TF/odometry source filters route them
independently):

- `publish_map_to_odom_tf`: advertise `map -> odom` (method `/map_odom`) taken
  directly from the estimator's own `T_map_to_odom` graph variable. It is smooth
  and time-consistent, so the bridge forwards it to `/tf` verbatim (its
  `child_frame != base_link` path), avoiding the stale
  `(map->base_link)*(odom->base_link)^-1` composition that otherwise injects
  motion-correlated jitter. Point the bridge's TF source filter at the
  `/map_odom` method. `map_to_odom_frame_name` selects which odometry *source*
  to read (auto if exactly one is known); `map_to_odom_child_frame` sets the
  published `/tf` child (empty = the source frame_id). These differ in practice:
  a source's frame_id is its sensor label (e.g. `odom_wheels`), which is usually
  NOT the REP-105 odom `/tf` frame the external driver publishes
  `odom -> base_link` for; they must match for `map -> odom -> base_link` to
  connect, so set `map_to_odom_child_frame` (or rename the source label). A
  `map_to_odom_frame_name` naming no known source logs the known-frame list and
  publishes nothing, rather than silently emitting a disconnected TF.
- `publish_fused_vehicle_tf`: advertise the fused vehicle pose in a distinct
  child frame `fused_vehicle_frame_name` (default `base_link_fused`, method
  `/fused`) at the module rate. A distinct child never collides with an
  `odom -> base_link` chain, giving consumers a high-rate fused pose outside the
  canonical robot tree.

ROS 2 launch file:
`mola_state_estimation_smoother/ros2-launchs/ros2-state-estimator.launch.py`

Default subscribed topics (configurable): `/gps` (NavSatFix), `/imu` (Imu).

## Configuration reference

The smoother's default YAML is at
`mola_state_estimation_smoother/params/state-estimation-smoother.yaml`.
Major parameter groups:

- **Reference frames**: `vehicle_frame_name`, `reference_frame_name`, `enu_frame_name`
- **Kinematic model**: `ConstantVelocity` or `Tricycle`
- **Sliding window**: `sliding_window_length` (seconds)
- **Noise models**: acceleration, integrator, twist uncertainties
- **IMU**: attitude sigma, azimuth offset, gravity alignment sigma
- **Georeferencing**: `estimate_geo_reference`, convergence thresholds
- **Sensor filtering**: regex patterns to match/reject sensor labels
- **Real-time path**: `async_backend`, `odometry_min_sample_period`,
  `imu_min_sample_period`, `publish_map_to_odom_tf` + `map_to_odom_frame_name` +
  `map_to_odom_child_frame`, `publish_fused_vehicle_tf` + `fused_vehicle_frame_name`

### Real-time setup (shipped defaults) and REP-105 `map -> odom`

The shipped `params/state-estimation-smoother.yaml` targets real-time use, so
these are **on by default** (each also settable via the shown env var):

Note on run paths: the `mola-cli` launches (e.g. `mola-lo-gui-rosbag2`, which
runs `lidar_odometry_from_rosbag2.yaml`) replay the bag in real time through a
`Rosbag2Dataset` module that emits observations at wall-clock rate. This is a
live-deployment surrogate, NOT the offline `mola-lidar-odometry-cli` (which
consumes the dataset as fast as possible). So under `mola-cli` the per-sensor
timing matches a real robot and `async_backend: true` is the appropriate,
deployed setting; only the true offline CLIs (`mola-lidar-odometry-cli`,
`mola-navstate-cli`) need it off for reproducibility.

| feature | param / env var | default | notes |
|---|---|---|---|
| async backend | `async_backend` / `MOLA_ASYNC_BACKEND` | **true** | sub-ms queries; solve off the caller's thread. **Set false for offline/reproducible batch runs** (mola-navstate-cli) - async serving is non-deterministic. The C++ `Parameters` default stays `false`, so unit tests (inline YAML) remain deterministic. |
| odom keyframe merge | `odometry_min_sample_period` / `MOLA_ODOMETRY_MIN_SAMPLE_PERIOD` | **0.1** | ~10 Hz keyframes; merges (no motion lost). Deterministic. |
| odom motion-model noise | `odom_motion_model_a1..a4` / `MOLA_ODOM_MOTION_MODEL_A1..A4`, `_min_std_xy` / `_min_std_phi_deg` | MRPT `CActionRobotMovement2D` Gaussian-model defaults, `min_std` floors tighter (1e-3 m / 0.1 deg) | Thrun-style proportional noise over each `fuse_odometry()` increment (grows with distance/angle, not a fixed per-step sigma). Lower `a1..a4` for accurate encoders with no slip; raise for loose terrain. |
| IMU keyframe skip | `imu_min_sample_period` / `MOLA_IMU_MIN_SAMPLE_PERIOD` | **0.1** | ~10 Hz attitude/gravity factors. |
| predict-twist low-pass | `predict_twist_filter_enabled` / `MOLA_NAVSTATE_PREDICT_TWIST_FILTER` (+ `predict_twist_filter_time_const` / `MOLA_NAVSTATE_PREDICT_TWIST_TAU`, 0.3 s) | **true** | EMA-smooths the extrapolation velocity so the front end's ICP prior stays smooth. Deterministic; C++ default also true. |
| accel random-walk sigmas | `sigma_random_walk_acceleration_linear` / `MOLA_NAVSTATE_SIGMA_RANDOM_WALK_LINACC`, `..._angular` / `MOLA_NAVSTATE_SIGMA_RANDOM_WALK_ANGACC` | **0.5** / **10.0** | Angular is loose on purpose: a tight value lets the constant-velocity factor override the per-keyframe gyro prior and average genuine fast rotations away (measured in-window: at 1.0 the optimized `W` kept only ~70% of the true rate at fast turns; at 10.0, ~98%). The boundary-node yaw variability this allows is damped by the predict-twist low-pass. |
| gyro observations | `imu_angular_velocity_sigma` / `MOLA_IMU_ANGULAR_VELOCITY_SIGMA` | **0.10** | Direct (Huber-robust) prior on each keyframe's `W`, so a fast rotation reaches the graph immediately instead of only through the constant-velocity factor. 0 disables. Deliberately loose, see below. |

**Gyro priors vs. the constant-angular-velocity factor.** These two fight each
other, and getting it wrong blows up the whole window. That factor's sigma is
`sigma_random_walk_acceleration_angular * dt`, which vanishes as `dt` does,
while the graph routinely produces keyframe pairs only ~10 ms apart (a pose
observation landing just past `min_time_difference_to_create_new_frame` from an
existing IMU keyframe). Those two keyframes then hold two *independent* noisy
gyro measurements that the model insists must be nearly identical. The residual
is `R_i*w_i - R_j*w_j`, so the optimizer can only relieve the disagreement by
rotating the poses: linear velocity diverged to ~1e6 m/s and iSAM2 threw
`IndeterminantLinearSystemException` on an unrelated `t`/`v` (or on `f0`,
`T_enu_to_map`, when the gravity factors sharing it get dragged along), roughly
once a second through an entire run. Three unconditional guards, all in
`StateEstimationSmoother.cpp`:

- The **default `imu_angular_velocity_sigma` is loose (0.10 rad/s)**. This is the
  dominant lever: a single raw, instantaneous sample is being used as the average
  `W` over a whole keyframe interval on a vibrating platform, so pinning it hard is
  physically wrong. On Oxford Spires the solver failed below ~0.08 rad/s and was
  clean at and above it (0.10-0.20 all clean); 0.10 is the honest value with
  margin. Loosening only the const-vel factor instead (see below) did *not*
  suffice on its own -- the prior tightness is what over-constrains.
- `angular_const_vel_sigma()` adds `sqrt(2)*imu_angular_velocity_sigma` in
  quadrature to the random-walk term, so the factor is never tighter than the
  sensor noise it competes with. Reduces to the old model when no gyro is fused.
- The gyro prior itself uses a Huber kernel, bounding how far any single raw
  sample can drag the window.

Regression test: `tests/test-imu-gyro-solver-stability.cpp`. Two approaches were
tried and rejected: a startup-only warmup gate (wrong shape -- the failures span
the whole run, not just the cold start), and loosening only the const-vel factor
(insufficient -- see the first bullet).

REP-105 `map -> odom` published from the graph variable (default **off**,
enable per deployment because it needs the routing + frame wiring below):

- `publish_map_to_odom_tf: true` (`MOLA_PUBLISH_MAP_TO_ODOM_TF`) advertises
  `map -> odom` from `T_map_to_odom` under method `<label>/map_odom`.
- `map_to_odom_frame_name` (`MOLA_MAP_TO_ODOM_FRAME`) selects which fused
  odometry SOURCE to read; its frame_id is that source's sensor label (from the
  bridge subscription's `output_sensor_label`, e.g. `odom_wheels`), NOT the ROS
  odom /tf frame.
- `map_to_odom_child_frame` (`MOLA_MAP_TO_ODOM_CHILD_FRAME`) is the published
  `/tf` child; it MUST equal the REP-105 odom frame the external driver
  publishes `odom -> base_link` for (usually `odom`). Empty = the source
  frame_id, which only works when the label already is that frame. Mismatch =>
  `map -> <label>` does not connect to `odom -> base_link` (tf2 "two
  unconnected trees").
- Route the bridge's TF source to the `<label>/map_odom` method via
  mola_lidar_odometry's `localization_publish_tf_source` launch argument, so the
  bridge forwards it verbatim instead of composing (and jittering) `map -> odom`.
  See `mola_lidar_odometry/agents.md`.

Example (single wheel-odom source labeled `odom_wheels`, ROS odom frame `odom`):
`MOLA_PUBLISH_MAP_TO_ODOM_TF=true MOLA_MAP_TO_ODOM_FRAME=odom_wheels
MOLA_MAP_TO_ODOM_CHILD_FRAME=odom`, plus
`localization_publish_tf_source:=state_estimation/map_odom`.

## Open issue: large, persistent `MeasuredGravityFactor` errors

Not yet diagnosed. On Oxford Spires (`2024-03-14-blenheim-palace-01`), dumping
the graph with `NAVSTATE_PRINT_FG_ERRORS=true` shows `MeasuredGravityFactor`
residuals of a few hundred within the first second, growing to ~1e4 and staying
there for the rest of the run. With `imu_normalized_gravity_alignment_sigma`
0.4 deg, an error of 280 is roughly a 10 deg mismatch between the measured
up-vector and the one implied by `T_enu_to_map` and the keyframe pose.

It is *not* caused by the gyro priors: the same magnitudes appear with
`MOLA_IMU_ANGULAR_VELOCITY_SIGMA=0`. Early on the residual also flip-flops
between two values on consecutive updates, suggesting `T_enu_to_map` and the
keyframe rotations are trading the error back and forth rather than converging.
Candidates to check: whether the sigma is simply far too tight for an
accelerometer under real linear acceleration (no `imu_accel_looks_like_gravity`
rejection beyond the magnitude test), and whether `T_enu_to_map` is
observable enough when `estimate_geo_reference=false` and no GNSS is fused.

## Relocalize mode (`estimate_geo_reference=false`, GNSS+IMU init against a known map)

Found and fixed via MOLA + MVSIM end-to-end testing (loading a geo-referenced
map, then relying purely on GNSS position + IMU attitude to recover the
initial pose, no `FixedPose` seed):

- **`set_geo_reference()` was unimplemented** (`StateEstimationSmoother` never
  overrode `NavStateFilter`'s default no-op). Front ends (e.g.
  `mola_lidar_odometry`) call this once a geo-referenced map is loaded, to
  anchor `T_enu_to_map` to a known, fixed value instead of leaving it a free
  variable. Without an override, `symbol_T_enu_to_map` kept its
  construction-time `ENU2MAP_WEAK_SIGMA` (1e4, i.e. effectively
  unconstrained) prior forever: every `Pose3RotationFactor`/
  `MeasuredGravityFactor` only measures rotation *relative to*
  `T_enu_to_map`, so with `T_enu_to_map` itself unpinned, the whole system's
  absolute rotation was a genuine gauge freedom (a null space GTSAM can't
  solve) -- reproduced as a `gtsam::IndeterminantLinearSystemException`
  ("underconstrained variables") a few dozen keyframes into any run. Now
  implemented: stores the geo-reference in `params_.fixed_geo_reference` and
  calls `reset_locked()` (not `reinitialize_gtsam_locked()` directly --
  that alone leaves stale pending `newValues`/`newFactors` around from the
  load-params-time initialization, which throws "key already exists" on the
  next `smoother.update()`).
- **`estimated_navstate()` didn't catch exceptions** from
  `get_latest_state_and_covariance()`'s `marginalCovariance()` calls, despite
  its own `[[nodiscard]] std::optional<NavState>` signature promising a
  graceful "not ready yet" for exactly this kind of case (an under-constrained
  factor graph, expected transiently at the start of any fusion). The
  exception propagated all the way up through the calling module's thread and
  crashed it entirely (`MolaLauncherApp`: "thread ... ended due to an
  exception"), killing GNSS/IMU fusion for the rest of the run. Now caught
  and treated like the function's other early `return {};` paths.
- **`has_converged_localization()` used the wrong criterion for relocalize
  mode**: it returned `params_.estimate_geo_reference &&
  state_.geo_reference.has_value()`, which is unconditionally false whenever
  `estimate_geo_reference=false` (true by definition in relocalize mode,
  where the geo-reference is fixed, not estimated) -- so relocalization could
  never be reported as converged, no matter how good the GNSS+IMU fix
  actually was. Now mode-aware: for `estimate_geo_reference=true`, unchanged
  (sticky on `state_.geo_reference.has_value()`, since that's only ever set
  once T_enu_to_map's own sigmas already passed the same thresholds); for
  `estimate_geo_reference=false`, checks the vehicle's own latest pose
  sigma (position + orientation) against `convergence_max_position_sigma`/
  `convergence_max_orientation_sigma_deg` instead.
- **`imu_attitude_azimuth_offset_deg` gotcha, not a bug**: `fuse_imu()`
  unconditionally applies a `+90 deg` rotation to the IMU's reported
  orientation, assuming raw IMU yaw is North-referenced ("yaw=0 => North",
  common for a real magnetometer/compass) and needs that correction to
  become ENU-referenced ("yaw=0 => East"). Simulators that report IMU
  orientation directly in the world/ENU frame (confirmed with MVSIM) need
  `imu_attitude_azimuth_offset_deg: -90` to cancel it out; forgetting this
  produces a stable, self-consistent, but exactly 90 deg wrong localization
  (high ICP goodness, high confidence -- easy to mistake for a real result).
- **Steady-state position sigma is bounded by raw GNSS noise, not by
  window duration**: with a real-time sliding-window filter (not a batch
  averager), position sigma settles close to the raw single-fix GNSS noise
  (e.g. ~1.2-2.0m steady-state for 1.5m raw noise), not indefinitely lower.
  `convergence_max_position_sigma` and `mola_lidar_odometry`'s separate,
  stricter `from_state_estimator_max_position_sigma` re-check both default
  to sub-meter values tuned for better-than-1.5m GNSS; with noisier GNSS
  they must be raised to match, or convergence will simply never be
  reported (not wrong, just perpetually "not yet").

## Code style

The project uses `clang-format` (config in `.clang-format` at repo root).
CI enforces formatting via `.github/workflows/check-clang-format.yml`.
Don't use long hyphens. Use American spelling. Use braced statements instead of
one-liners.
