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
- `fuse_pose_deferred()`: the reading is fused by the next call that reads or
  changes the estimate (`apply_deferred_pose()` at the top of every public
  method except `fuse_imu()`), which waits for the covariance outside
  `state_mtx_` and fuses only a reading whose covariance is ready. Readings are
  fused in call order: a second `fuse_pose_deferred()` waits for an earlier
  pending covariance. Internal paths that already hold the lock use the
  `*_locked()` bodies (e.g. GNSS in `onNewObservation()`).
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
| GTSAM 4.2 smoother ordering | `src/NewestLastFixedLagSmoother.{h,cpp}` |
| Default config | `params/state-estimation-smoother.yaml` |
| ROS 2 launches | `ros2-launchs/ros2-state-estimator.launch.py`, `ros2-fuse-two-odometries.launch.py`, `ros2-demo-simulated-sensors.launch.py` (+ `rviz2/state_estimation_demo.rviz`) |
| MOLA-CLI launches | `mola-cli-launchs/state_estimator_ros2.yaml`, `demo_lidar_odom_plus_wheel_odom_fusion.yaml` |
| CLI app | `apps/mola-navstate-cli.cpp` |
| Unit tests (47) | `tests/test-*.cpp` |
| Integration tests | `test/integration/test_*.py` (real time; run serially) |

Structure:
- Inherits `mola::NavStateFilter`, `mola::LocalizationSourceBase`,
  `mola::MapSourceBase`, and `mola::ImuBiasSource` when both mola_kernel and
  mola_imu_preintegration have it (`MOLA_SMOOTHER_HAS_IMU_BIAS_SOURCE`). GTSAM details are hidden in the `GtsamImpl` pimpl.
- State is guarded by `stateMutex_` (`std::mutex`); `*_locked()` methods
  assume it is held.
- Per keyframe: pose `T(k)`, body linear velocity `V(k)`, body angular velocity
  `W(k)`. Per odometry frame `i`: `T_map_to_odom_i`. Plus `T_enu_to_map`.
  With `imu_preintegration.enabled`, also IMU biases `Z(k)`
  (`imuBias::ConstantBias`), chained by a random walk on every link.
- Keyframes are created per timestamp, reusing an existing one closer than
  `min_time_difference_to_create_new_frame` (the first neighbor found; GNSS asks
  for the closest one); consecutive keyframes are linked
  by kinematic factors (`ConstantVelocity` or `Tricycle` model).
- Window length `sliding_window_length` [s].
- YAML supports `${ENV_VAR|default}` substitution.

Sensor inputs:
- `fuse_pose()`: poses in `{map}` (prior factor), or in a source frame
  `{odom_i}`, always fused as relative increments (see below).
- `fuse_odometry()`: wheel odometry, always fused as relative increments with
  a motion-model covariance (`odom_motion_model_a1..a4`, `_min_std_*`, plus a
  `1e-4` variance floor). Its frame is named after its label, or
  `<label>_wheels` if a `fuse_pose()` source uses that name, in either arrival
  order: two sources never share a frame (`test-wheel-odometry-frame-collision`).
  Out of plane (z, pitch, roll) it asserts planar motion in the vehicle frame
  unless `odom_out_of_plane_std_z/_roll_pitch` add uncertainty there (default
  0; raise on uneven terrain or for legged/handheld carriers:
  `test-planar-odometry-out-of-plane`).
- `fuse_imu()`: attitude, gravity (`MeasuredGravityFactor`) and a Huber-robust
  angular-velocity prior on `W(k)` (`imu_angular_velocity_sigma`, 0 disables).
  Before averaging, the accelerometer is corrected by the centripetal term
  gyro x newest estimated velocity (`imu_gravity_compensate_centripetal`).
  Attitude at most once per `imu_attitude_min_sample_period` per sensor label.
  With `imu_attitude_yaw_bias_sigma_deg > 0`, attitude readings share an azimuth
  bias state `B(k)` (double, one per keyframe with attitude,
  `Pose3RotationYawBiasFactor`), a first-order Gauss-Markov chain
  (`imu_attitude_yaw_bias_time_constant`; prior on the chain's first variable,
  restarted if its tail was marginalized). While only the IMU observes the
  heading, its sigma stays near (~0.7x, fixed-lag linearization) that sigma.
  `estimated_imu_yaw_bias()`; `publish_imu_yaw_bias` publishes it as module
  diagnostics (`imu_yaw_bias_deg`, `imu_yaw_bias_sigma_deg`).
- IMU preintegration (`imu_preintegration`, default on): raw readings of one IMU
  are buffered before decimation; each kinematic link gets a
  `FactorImuPreintegrationBodyVelEnu` (gtsam::ImuFactor with body-frame
  velocities, gravity along -z of `{enu}` through `T_enu_to_map`, so `{map}` may
  be tilted; before its first solve, `level_enu_to_map_locked()` sets the roll and
  pitch of `T_enu_to_map` from the accelerometer at the pinned first pose, the
  first pose reading, or the first reading of a wheel odometry that defines `{map}`)
  once the readings cover it (`attach_imu_factors_locked()`), with a
  covariance floor (`min_sigma_*`); while an estimated geo-reference still has its
  azimuth pinned by the gauge anchor, links get the plain factor with gravity
  fixed in `{map}` at the leveled tilt instead. The constant-velocity linear
  factors stay, weaker (`linear_kinematics_sigma_scale`) once an IMU has been seen,
  so without any IMU the estimates are those with preintegration off.
  Accelerometer units: `accel_scale` (0 = automatic: units of g if the mean norm of
  the first 0.5 s of readings is below 3, kept across resets like `recentAccel_`);
  links wait in `imuLinksAwaiting` until it is known.
  The per-keyframe gravity and angular-velocity factors stay on with it.
- `relative_pose_increment_from_source_covariance` (default off): odometry-frame
  increments take each reading's own covariance instead of
  `relative_pose_increment_sigma_*`.
- `fuse_pose_deferred()` (needs the mola_kernel with it): a pose whose
  covariance is a future; with `async_backend` the backend waits for it before
  taking `stateMutex_` (`deferredCovs_`), otherwise it is awaited at once.
- `fuse_gnss()`: ENU position, robust kernel `gnss_robust_kernel` (Huber by default;
  Cauchy, Geman-McClure, DCS) scaled by `gnss_huber_threshold`, sigmas floored
  at `gnss_min_sigma_floor_xy` / `_z` (also for the azimuth baselines). Before that,
  `gnss_gate_locked()`: admission tests (fix qualities, finite positive variances,
  `gnss_max_horizontal_sigma`, `gnss_max_sigma_ratio` against the lowest sigma in the last
  `gnss_sigma_ratio_window` s (off by default: it only helps if the odometry beats the degraded
  fixes), `gnss_max_height_deviation`); the datum only after
  `gnss_datum_min_consistent_fixes` consistent fixes; fix-to-fix consistency while
  there is no prediction (bounded by the odometry's antenna motion when there is any;
  a short gap in it rejects; else `gnss_max_plausible_speed`); predictions only from solved keyframes; once converged (or relocalized), a chi2 innovation gate
  against the predicted antenna, whose sigma grows with the path since the last
  accepted fix; a run of `gnss_gate_reacquire_after` rejected fixes that move like
  the prediction over `gnss_gate_reacquire_min_path` is reacquired. Counters in
  `gnss_gate_stats()` (kept across resets), throttled warnings. The fix goes
  to the closest keyframe within
  `gnss_nearby_keyframe_stamp_tolerance`; a nonzero time offset uses
  `FactorGnssMapEnuConstVel`, which moves the antenna to the fix time with that
  keyframe's `V`/`W`.
- `fuse_twist()`: velocity priors.
- IMU biases (`imu_bias_tracking`, default off): one `mola::imu::ImuBiasTracker` for all
  IMUs matching `imu_bias_labels_re` (raw readings from `onNewObservation()`, even if not
  fused), under its own leaf `imuBiasMutex_`. Motion hints from the increments (>=
  `imu_bias_hint_min_period`, hinted along the whole increment) of every pose input by frame
  name (including the reference frame), wheel odometry and twists; thresholds above LIO jitter
  at rest. Updates are advertised from `spinOnce()` only (never from callers' threads). Gyro
  bias subtracted from fused angular velocities. Kept across `reset()`.
  `publish_imu_biases`: diagnostics `imu_bias_<label>_gyro_*` and hint counts.

Relative formulation (wheel odometry and every non-`{map}` `fuse_pose()` source):
- One absolute factor on the source's first reading resolves
  `T_map_to_odom_i`; after that, one `BetweenFactor` per keyframe change.
- Readings landing on an existing keyframe hold the chain anchor at the first
  of them, so their motion goes into the next increment.
- A factor whose previous keyframe was already marginalized is skipped.
- For `fuse_pose()` sources, the increment covariance is diagonal, from
  `relative_pose_increment_sigma_lin`/`_ang` (defaults tuned for LiDAR odometry:
  `_ang` 1e-5 rad; `_lin` 0 = automatic, resolved by `loadFrom()` to 0.1 mm with
  preintegration on and available, 2 cm otherwise); the supplied covariance is
  only used by the anchor factor. `relative_pose_increment_sigma_per_sqrt_meter`/`_per_sqrt_rad` add
  a variance growing linearly with the increment size (random walk).

Other behavior:
- `pose_robust_huber_threshold` (0 = off): Huber kernel on every per-reading
  `fuse_pose()` factor, not on the one-time anchor.
- `pose_max_lead_time` (0.5 s, 0 = off): a `fuse_pose()` reading stamped more than
  this after both its source's previous reading and the newest reading of the other
  inputs that delivered readings since then is dropped (`accept_pose_stamp()`, in
  the caller's thread, before the async queue; stamps per input in `inputStamps_`,
  kept across resets). After 10 in a row, that source is accepted again.
  `pose_lead_drops()`; `test-pose-lead-guard`.
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
- Estimated `T_map_to_odom_i` of an odometry source is computed as
  `X(chain tail kf) (+) pose_in_odom(tail)^-1`, not read from the graph
  variable (which only the anchor factor constrains). Its covariance (the
  graph variable's marginal) is computed on first use after each solve
  (`complete_odom_frame_covariance_locked()`); the async `Snapshot` carries
  it only for sources without a raw pose yet.
- Extrapolation (`extrapolate_pose_pdf()`) propagates covariance: anchor
  covariance through the composition plus velocity and acceleration noise.
- Predict-twist low-pass (`predict_twist_filter_enabled`,
  `predict_twist_filter_time_const`, 0.1 s): the extrapolation velocity is a
  dt-aware EMA of the newest keyframe's twist. Longer constants lag at turn
  exits and the front end follows the prior into a heading error.
- Prediction covariance in an odometry source's frame
  (`predict_covariance_from_residuals`, default on): twist covariance from the
  running mean (`predict_residual_time_const`) of that source's squared
  one-step prediction errors (`State::prediction_residuals`, copied into the
  `Snapshot`), no random-walk term on top; the graph's twist marginal until 10
  errors are in, whose per-axis median seeds the mean (a start-up transient
  must not set it: `test-prediction-residual-seed`).
- Linear kinematics (`sigma_random_walk_acceleration_linear` 5 m/s²,
  `sigma_integrator_position` 1 m, YAML and C++): a prior on accelerations,
  centripetal included. An accurate odometry-frame pose source must come out
  unchanged (`test-odometry-pose-source-fidelity`).
- iSAM2 relinearization: per variable type, `isam2_relinearize_threshold`
  (0.01) for poses `T`/`F`, a fixed 0.1 for `V`/`W`/`Z`/`B`
  (`test-imu-gyro-solver-stability`).
- iSAM2 ordering (`isam2_newest_keys_last`, default on, GTSAM 4.2 only):
  `NewestLastFixedLagSmoother` re-implements 4.2's fixed-lag `update()` so the
  keys of new factors and those at the newest stamp are eliminated last (root
  of the Bayes tree), and the cliques of marginalized keys are re-eliminated
  (4.2's `marginalizeLeaves()` needs those keys first in their cliques). A third
  constraint group only with >= 3 new-factor keys: 4.2's CCOLAMD overflows if a
  group number reaches the reordered size.
- The angular constant-velocity factor sigma is
  `sigma_random_walk_acceleration_angular * dt` combined in quadrature with
  `sqrt(2) * imu_angular_velocity_sigma` (`angular_const_vel_sigma()`).
- `async_backend` (C++ default false): solve in a backend thread;
  `estimated_navstate()` served lock-free by `FastPredictor` from the latest
  `Snapshot`. The synchronous path is deterministic.
- `estimated_trajectory()` (`keep_finalized_trajectory`): keyframes' final values,
  only those holding a pose or odometry observation (`State::pose_keyframes`);
  IMU-, twist- or GNSS-only keyframes are left out, their attitude is poorly observed.
- `estimated_navstate()` returns empty, instead of throwing, while the graph
  cannot be solved yet. Any exception in the solve (`solve_locked()`: update,
  estimate or marginals) is logged and resets the smoother; it never reaches callers.
- Two resets: `reset()` (also after a failed solve) starts from scratch: the next
  poses define {map} again, as after a re-localization. `reset_motion()` keeps
  {map}: odometry frames come back anchored at their live T_map_to_odom_i (a frame
  not kept waits a window length for a kept one before anchoring {map}), the
  first keyframe is not linked to the origin, and with IMU preintegration
  T_enu_to_map keeps the leveling that preintegration used (leveling again while
  moving reads acceleration as gravity; the solved tilt is not used, attitude
  factors may bend it) with its last solved azimuth. A second `reset_motion()` with no successful solve in
  between resets from scratch.
- Gauge anchors (strong priors on never-marginalized variables, withdrawn once
  observed; weak priors there break GTSAM>=4.3 Cholesky): `{map}` is pinned to
  the first odometry frame until a `{map}` pose arrives (or, with a fixed
  geo-reference, GNSS/IMU attitude); the azimuth of `T_enu_to_map` until IMU
  attitude or a GNSS baseline long enough for
  `convergence_max_orientation_sigma_deg`. Estimated geo-references do not
  converge while pinned. Until something relates `{map}` to the vehicle,
  solving is deferred (up to one window). When `{map}` is observed only in
  position (GNSS with a fixed geo-reference, no attitude), the orientation of
  that odometry frame stays pinned: roll/pitch until IMU gravity or attitude,
  yaw until IMU attitude or a GNSS baseline good for ~5 deg; both by a `{map}`
  pose. `has_converged_localization()` then reports the pinned angles as
  unknown (yaw sigma pi, roll/pitch pi/2), so a robot without IMU converges on
  position only if `convergence_max_orientation_sigma_deg` allows it. Feature
  macro `MOLA_STATE_ESTIMATION_SMOOTHER_HAS_ORIENTATION_GAUGE` (`Parameters.h`).
- A new `T_map_to_odom_i` is seeded from its first reading. A new key is never
  marginalized in the update that inserts it.
- Observations labeled `ground_truth` are ignored by both estimators unless
  `fuse_ground_truth_label: true`.
- Optional planar motion (`enforce_planar_motion`).

Georeferencing and relocalization:
- `estimate_geo_reference: true`: `T_enu_to_map` estimated from GNSS/IMU,
  published (and reported by `current_georeferencing()`) only once its sigmas pass
  `convergence_max_position_sigma`/`convergence_max_orientation_sigma_deg` (roll/pitch
  only if observed). Without an attitude source, while its azimuth is pinned by the
  gauge anchor, neither GNSS nor gravity factors are added: once marginalized at the
  pinned azimuth (standing still longer than the window), they biased it for good.
  The fix completing the baseline replaces the anchor with a prior at the azimuth
  measured by the GNSS vs. {map} antenna displacement.
- `get_geo_reference()`: the fixed one, or the datum (first fix) plus
  `T_enu_to_map` as of the last solve (`lastSolvedGeoref`) once its azimuth is
  unpinned, converged or not. It also serves the `enu` frame in
  `estimated_navstate()` (T of that solve, causal) and `estimated_trajectory()`
  (each keyframe through the T it was finalized with once the georef converged,
  `finalizedTrajectoryEnu_`, cleared on reset; latest T for the rest). {map} moves with respect to ENU
  while T is refined, so a {map} trajectory is not rigidly related to ENU.
  `save_final_georef_to_file` (`MOLA_SE_SAVE_GEOREF_FILE`) writes it on
  destruction, as mp2p_icp YAML.
- `set_geo_reference()` / `fixed_geo_reference`: fixes `T_enu_to_map` and
  resets the estimator. Zero variances are floored to 1e-6 (singular otherwise).
- `has_converged_localization()`: with `estimate_geo_reference=true`, true once
  the geo-reference is known; otherwise, when `{map}` is observed in the last
  solve (not just pinned by its gauge anchor, which makes the pose look certain
  at the odometry origin) and the vehicle's own pose sigmas pass the same thresholds.
  Steady-state position sigma is bounded by the raw GNSS noise, so the
  thresholds must match the receiver.
- `fuse_imu()` applies a fixed +90 deg yaw so North-referenced IMU yaw becomes
  ENU; sources already reporting ENU yaw (e.g. simulators) need
  `imu_attitude_azimuth_offset_deg: -90`.

Large `MeasuredGravityFactor` residuals for a whole run: one known cause is an
IMU driver publishing a constant placeholder orientation (with covariance >= 0)
that fights the accelerometer. `fuse_imu()` warns when attitude and
accelerometer disagree by > 30 deg; `imu_attitude_sigma_deg: 0` ignores attitude.

### Shipped YAML defaults that differ from the C++ `Parameters` defaults

The shipped YAML targets real-time deployment; unit tests use inline YAML and
the C++ defaults.

| Param (env var) | YAML | C++ |
|---|---|---|
| `async_backend` (`MOLA_ASYNC_BACKEND`) | true | false |
| `odometry_min_sample_period` (`MOLA_ODOMETRY_MIN_SAMPLE_PERIOD`) | 0.1 | 0 |
| `imu_min_sample_period` (`MOLA_IMU_MIN_SAMPLE_PERIOD`) | 0.1 | 0 |
| `sliding_window_length` (`MOLA_NAVSTATE_SLIDING_WINDOW_SEC`) | 6.0 | 5.0 |
| `imu_attitude_min_sample_period` (`MOLA_IMU_ATTITUDE_MIN_SAMPLE_PERIOD`) | 0.5 | 0 |
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
| `FactorGnssMapEnuConstVel` | `ExpressionFactorN` | Same, fix time offset from its keyframe (uses `V`, `W`) |
| `MeasuredGravityFactor` | `ExpressionFactorN` | Up-vector leveling relative to `T_enu_to_map` |
| `MapGravityFactor` | `ExpressionFactorN` | Up-vector leveling in a z-up map frame (no geo-reference) |
| `Pose3RotationFactor` | `ExpressionFactorN` | Rotation-only constraint |
| `Pose3RotationYawBiasFactor` | `ExpressionFactorN` | Rotation-only, with a scalar azimuth bias variable |

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
`odom{1,2,3}_topic` (+ `_label`), all empty (disabled) by default; its
`navstate_*` args are empty by default, meaning "use the params YAML value".
`ros2-fuse-two-odometries.launch.py` uses the same topic argument names. The fused
`map -> base_link` pose is advertised; the bridge publishes it to `/tf`.

`ros2-demo-simulated-sensors.launch.py` (`mode:=wheels_imu|wheels_imu_gnss|two_odometries|imu_gnss`)
runs `mola_demos`' `fake_sensor_publisher.py` (circle scenario), includes
`ros2-state-estimator.launch.py`, and opens RViz; its static identity TFs rely
on the simulated odometries and ground truth starting at the `{enu}` origin.

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
