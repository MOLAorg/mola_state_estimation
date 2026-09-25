# AGENTS.md - mola_state_estimation

> **Maintaining this file.** Describe WHAT exists: components, behavior,
> parameters, conventions. Do not add why/history: no changelog entries, bug
> stories, "was fixed" notes, dataset anecdotes or measurements. Those belong
> in commit messages and code comments. Edit the relevant entry in place, keep
> each entry to a few lines, and keep the whole file under ~300 lines. If
> something needs a paragraph, document it in the code and point to it here.

## Overview

State estimation and sensor fusion for [MOLA](https://docs.mola-slam.org/latest/):
fuses pose, wheel odometry, IMU, GNSS and twist measurements into a vehicle's
pose, velocity and orientation over time. Also provides offline
georeferencing of keyframe-based maps.

Rules: keep this file in sync with code changes. American spelling. No en/em
dashes. Don't sign commits as an AI agent. Run clang-format-14 before
committing. License: GNU GPL v3 (commercial options available upon request).

## Repository layout

```
mola_state_estimation/            <- ROS 2 metapackage (dependency grouping only)
mola_state_estimation_simple/     <- Lightweight constant-velocity estimator
mola_state_estimation_smoother/   <- iSAM2 sliding-window factor-graph smoother
mola_gtsam_factors/               <- Reusable custom GTSAM factors
mola_georeferencing/              <- Offline georeferencing of SimpleMaps
docs/                             <- Sphinx/RST documentation
scripts/                          <- clang-format helpers, status table generator
```

## 1. `mola_state_estimation_simple`

Constant-velocity kinematic estimator with velocity extrapolation.

| Item | Path |
|------|------|
| Main class | `include/mola_state_estimation_simple/StateEstimationSimple.h` |
| Parameters | `include/mola_state_estimation_simple/Parameters.h` |
| Implementation | `src/StateEstimationSimple.cpp` |
| Test | `tests/test-state-estimation-simple.cpp` |

- Inherits `mola::NavStateFilter`. Ignores `frame_id`.
- Fuses pose, wheel odometry, IMU and twist. GNSS is optional
  (`gnss_enabled`, default false): low-sigma fixes nudge the pose anchor once
  a geo-reference is set via `set_geo_reference()`.
- IMU and odometry readings are buffered and fused in timestamp order up to
  the time of interest of each call (`fuse_pending_imu_up_to()`,
  `fuse_pending_odometry_up_to()`), so results depend on measurement
  timestamps, not on delivery order.
- Velocity is low-pass filtered (`velocity_filter_enabled`, default true).
- `initial_twist` (+ `initial_twist_sigma_lin`/`_ang`) seeds the twist on
  `initialize()`/`reset()`; a source's first pose does not clear it.
  `loadFrom()` rejects non-positive sigmas when `initial_twist` is non-zero.
- Prior covariance returned by `estimated_navstate()`: see the class docs.
- Optional planar motion (`enforce_planar_motion`).

## 2. `mola_state_estimation_smoother`

Sliding-window factor-graph smoother on GTSAM's iSAM2
(`IncrementalFixedLagSmoother`). The primary multi-sensor estimator.

| Item | Path |
|------|------|
| Main class | `include/mola_state_estimation_smoother/StateEstimationSmoother.h` |
| Parameters | `include/mola_state_estimation_smoother/Parameters.h` |
| Implementation | `src/StateEstimationSmoother.cpp` |
| Async serving | `src/FastPredictor.{h,cpp}`, `src/Snapshot.h`, `src/extrapolation.h` |
| Default config | `params/state-estimation-smoother.yaml` |
| ROS 2 launches | `ros2-launchs/ros2-state-estimator.launch.py`, `ros2-fuse-two-odometries.launch.py` |
| MOLA-CLI launches | `mola-cli-launchs/state_estimator_ros2.yaml`, `demo_lidar_odom_plus_wheel_odom_fusion.yaml` |
| CLI app | `apps/mola-navstate-cli.cpp` |
| Unit tests (25) | `tests/test-*.cpp` |
| Integration tests | `test/integration/test_*.py` (real time; run serially) |

Structure:
- Inherits `mola::NavStateFilter`, `mola::LocalizationSourceBase`,
  `mola::MapSourceBase`. GTSAM details are hidden in the `GtsamImpl` pimpl.
- State is guarded by `stateMutex_` (`std::mutex`); `*_locked()` methods
  assume it is held.
- Per keyframe: pose `T(k)`, body linear velocity `V(k)`, body angular velocity
  `W(k)`. Per odometry frame `i`: `T_map_to_odom_i`. Plus `T_enu_to_map`.
- Keyframes are created per timestamp, reusing an existing one closer than
  `min_time_difference_to_create_new_frame`; consecutive keyframes are linked
  by kinematic factors (`ConstantVelocity` or `Tricycle` model).
- Window length `sliding_window_length` [s].
- YAML supports `${ENV_VAR|default}` substitution.

Sensor inputs:
- `fuse_pose()`: poses in `{map}` (prior factor) or in a source frame
  `{odom_i}` (absolute factor against `T_map_to_odom_i`). Sources matching
  `relative_factors_frame_ids_re` are fused as relative increments instead
  (see below).
- `fuse_odometry()`: wheel odometry, always fused as relative increments with
  a motion-model covariance (`odom_motion_model_a1..a4`, `_min_std_*`, plus a
  `1e-4` variance floor).
- `fuse_imu()`: attitude, gravity (`MeasuredGravityFactor`) and a Huber-robust
  angular-velocity prior on `W(k)` (`imu_angular_velocity_sigma`, 0 disables).
- `fuse_gnss()`: ENU position, Huber-robust (`gnss_huber_threshold`).
- `fuse_twist()`: velocity priors.

Relative formulation (wheel odometry and `relative_factors_frame_ids_re`):
- One absolute factor on the source's first reading resolves
  `T_map_to_odom_i`; after that, one `BetweenFactor` per keyframe change.
- Readings landing on an existing keyframe hold the chain anchor at the first
  of them, so their motion goes into the next increment.
- A factor whose previous keyframe was already marginalized is skipped.
- For `fuse_pose()` sources the supplied covariance is read as the uncertainty
  of one increment. `relative_pose_increment_sigma_lin`/`_ang` replace it
  (the replaced block's cross terms are cleared);
  `relative_pose_increment_sigma_per_sqrt_meter`/`_per_sqrt_rad` add a
  variance growing linearly with the increment size (random walk; requires
  the flat sigma > 0).

Other behavior:
- `pose_robust_huber_threshold` (0 = off): Huber kernel on every per-reading
  `fuse_pose()` factor, not on the one-time anchor.
- Decimation (0 = off): `odometry_min_sample_period` and
  `pose_min_sample_period` merge dropped readings into the next kept one (no
  motion lost); `imu_min_sample_period` averages readings (`mola::imu::ImuAverager`),
  one fused reading per period, so vibration does not alias into the factors.
  `pose_min_sample_period` applies to every `fuse_pose()` source; a dropped
  reading still refreshes that source's own-frame anchor.
- `estimated_navstate(t, {odom_i})` is frame-local: it extrapolates from the
  source's own last raw pose (`State::last_raw_pose_by_source`) by the
  body-twist increment, falling back to the `{map}` conversion before the
  source's first reading.
- Estimated `T_map_to_odom_i` of a relative source is computed as
  `X(chain tail kf) (+) pose_in_odom(tail)^-1`; for other sources it is the
  graph variable.
- Extrapolation (`extrapolate_pose_pdf()`) propagates covariance: anchor
  covariance through the composition plus velocity and acceleration noise.
- Predict-twist low-pass (`predict_twist_filter_enabled`,
  `predict_twist_filter_time_const`): the extrapolation velocity is a dt-aware
  EMA of the newest keyframe's twist.
- The angular constant-velocity factor sigma is
  `sigma_random_walk_acceleration_angular * dt` combined in quadrature with
  `sqrt(2) * imu_angular_velocity_sigma` (`angular_const_vel_sigma()`).
- `async_backend` (C++ default false): solve in a backend thread;
  `estimated_navstate()` served lock-free by `FastPredictor` from the latest
  `Snapshot`. The synchronous path is deterministic.
- `estimated_navstate()` returns empty, instead of throwing, while the graph
  cannot be solved yet.
- Observations labeled `ground_truth` are ignored by both estimators unless
  `fuse_ground_truth_label: true`.
- Optional planar motion (`enforce_planar_motion`).

Georeferencing and relocalization:
- `estimate_geo_reference: true`: `T_enu_to_map` estimated from GNSS/IMU,
  published once its sigmas pass `convergence_max_position_sigma`/
  `convergence_max_orientation_sigma_deg`.
- `set_geo_reference()` / `fixed_geo_reference`: fixes `T_enu_to_map` and
  resets the estimator.
- `has_converged_localization()`: with `estimate_geo_reference=true`, true once
  the geo-reference is known; otherwise, when the vehicle's own pose sigmas
  pass the same thresholds. Steady-state position sigma is bounded by the raw
  GNSS noise, so the thresholds must match the receiver.
- `fuse_imu()` applies a fixed +90 deg yaw so North-referenced IMU yaw becomes
  ENU; sources already reporting ENU yaw (e.g. simulators) need
  `imu_attitude_azimuth_offset_deg: -90`.

Known open issue: `MeasuredGravityFactor` residuals can stay large for a whole
run on some datasets; not yet diagnosed.

### Shipped YAML defaults that differ from the C++ `Parameters` defaults

The shipped YAML targets real-time deployment; unit tests use inline YAML and
the C++ defaults.

| Param (env var) | YAML | C++ |
|---|---|---|
| `async_backend` (`MOLA_ASYNC_BACKEND`) | true | false |
| `odometry_min_sample_period` (`MOLA_ODOMETRY_MIN_SAMPLE_PERIOD`) | 0.1 | 0 |
| `imu_min_sample_period` (`MOLA_IMU_MIN_SAMPLE_PERIOD`) | 0.1 | 0 |
| `sliding_window_length` (`MOLA_NAVSTATE_SLIDING_WINDOW_SEC`) | 6.0 | 5.0 |
| `sigma_integrator_orientation` (`MOLA_NAVSTATE_SIGMA_INTEGRATOR_ANG`) | 1.0 | 0.1 |

Set `async_backend: false` for offline, reproducible runs
(`mola-navstate-cli`, `mola-lidar-odometry-cli`).

### Diagnostics (env vars, off by default)

- `MOLA_NAVSTATE_DUMP=<file>` (both estimators): one CSV row per
  `estimated_navstate()` result (pose, twist, covariance, information, `dt`).
- `NAVSTATE_PRINT_FG`, `NAVSTATE_PRINT_FG_ERRORS` (+
  `NAVSTATE_PRINT_FG_ERRORS_THRESHOLD`): print the smoother graph / its
  largest factor errors.
- `MOLA_VEL_FILTER_DUMP=<file>`: velocity-filter trace (simple estimator).

## 3. `mola_gtsam_factors`

Headers in `include/mola_gtsam_factors/`, sources in `src/`.

| Class | Base | Purpose |
|---|---|---|
| `FactorAngularVelocityIntegration`, `...Pose` | `ExpressionFactorN` | Rotation integration from angular velocity |
| `FactorConstLocalVelocity`, `...Pose` | `ExpressionFactorN` | Constant body-frame velocity |
| `FactorTrapezoidalIntegrator` | `ExpressionFactorN` | Trapezoidal velocity integration |
| `FactorTricycleKinematic` | `NoiseModelFactor4` | Ackermann / tricycle kinematics |
| `FactorGnssEnu` | `ExpressionFactorN` | GNSS position in ENU |
| `FactorGnssMapEnu` | `ExpressionFactorN` | GNSS with explicit `T_enu_to_map` |
| `MeasuredGravityFactor` | `ExpressionFactorN` | Up-vector leveling relative to `T_enu_to_map` |
| `MapGravityFactor` | `ExpressionFactorN` | Up-vector leveling in a z-up map frame (no geo-reference) |
| `Pose3RotationFactor` | `ExpressionFactorN` | Rotation-only constraint |

`imu_helpers.h`: accelerometer/quaternion sanity checks and the ENU azimuth
correction shared by the estimators and georeferencing.

## 4. `mola_georeferencing`

| Item | Path |
|------|------|
| Library API | `include/mola_georeferencing/simplemap_georeference.h` |
| Implementation | `src/simplemap_georeference.cpp` |
| CLI: georeference a simplemap | `apps/mola-sm-georeferencing-cli.cpp` |
| CLI: georeference a trajectory | `apps/mola-trajectory-georef-cli.cpp` |
| CLI: add geodetic info to maps | `apps/mola-mm-add-geodetic-cli.cpp` |
| Tests | `tests/test_*.cpp` |

- `simplemap_georeference()`: optimal `T_enu_to_map` (+ RMSE) from the GNSS
  and IMU observations stored in a `CSimpleMap`.
- `recenter_georeference()`: re-datums a geo-reference to a desired
  `T_enu_to_map` translation, geometrically equivalent.
- Graph convention: one `T(0)` (`T_enu_to_map`) and one `P(i)` per keyframe,
  with `P(i)` the vehicle pose in `{map}` (anchored by `PriorFactor`). Every
  measurement factor composes `T(0)` itself: `FactorGnssMapEnu(T(0), P(i))`,
  `MeasuredGravityFactor`/`Pose3RotationFactor` on `(T(0), P(i))`.
  `test_gnss_and_imu_attitude` covers GNSS and IMU together.
- Azimuth degeneracy check (`extract_gnss_frames_from_sm()`) is horizontal
  only. Without IMU attitude in the keyframes, yaw rests on GNSS alone (and a
  warning is printed); MOLA-LO stores IMU only with
  `simplemap.save_imu_max_age`.
- Diagnostics (env vars): `MOLA_SM_GEOREF_PRINT_FACTOR_GRAPH`,
  `MOLA_SM_GEOREF_PRINT_FG_ERRORS`, `MOLA_SM_GEOREF_PRINT_GNSS_FRAMES`,
  `MOLA_SM_GEOREF_PRINT_LARGE_FACTOR_ERRORS` (+ `..._THRESHOLD`,
  `..._MAX_PRINT`); `MOLA_SM_GEOREF_DUMP_GNSS=<file>` and
  `MOLA_SM_GEOREF_DUMP_IMU_ATTITUDE=<file>` write per-observation measurement,
  prediction, residual and timestamp offset from the keyframe.

## Dependencies

GTSAM (iSAM2), MRPT (poses, obs, maps), `mola_kernel` (`NavStateFilter`,
module lifecycle), `mola_imu_preintegration`, `mola_yaml`, `mp2p_icp`
(georeferencing). All packages use `ament_cmake` and C++17.

## Build and test

```bash
cd ~/ros2_ws
colcon build --packages-select mola_gtsam_factors mola_state_estimation_simple \
  mola_state_estimation_smoother mola_georeferencing mola_state_estimation
colcon test --packages-select mola_state_estimation_simple \
  mola_state_estimation_smoother mola_georeferencing
colcon test-result --verbose
```

Test binaries need `/opt/ros/<distro>/setup.bash` sourced as well as the
workspace overlay.

## ROS 2 integration

Runs inside `mola_launcher`, with sensors bridged from ROS 2.
`ros2-state-estimator.launch.py` takes `imu_topic_name`, `gnss_topic_name` and
`odom{1,2,3}_topic` (+ `_label`), all empty (disabled) by default. The fused
`map -> base_link` pose is advertised; the bridge publishes it to `/tf`.

Optional extra outputs (default off, distinct `method` suffixes):
- `publish_map_to_odom_tf`: `map -> odom` under method `<label>/map_odom`,
  from the estimated `T_map_to_odom_i` of the source named by
  `map_to_odom_frame_name` (auto if only one is known; a name matching no
  source publishes nothing). `map_to_odom_child_frame` sets the `/tf` child
  (empty = source label) and must equal the odom frame of the external
  `odom -> base_link`. Route the bridge's TF source to `<label>/map_odom` with
  `mola_lidar_odometry`'s `localization_publish_tf_source`.
- `publish_fused_vehicle_tf`: fused pose in child frame
  `fused_vehicle_frame_name` (default `base_link_fused`), method `/fused`.

## Code style

`.clang-format` at the repo root (clang-format-14), enforced by
`.github/workflows/check-clang-format.yml`. Braced statements, no one-liners.
