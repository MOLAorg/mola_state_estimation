# AGENTS.md - mola_state_estimation

## Overview

This repository provides **state estimation and sensor fusion** for the
[MOLA](https://docs.mola-slam.org/latest/) (Modular Optimization framework for
Localization and mApping) framework. It fuses odometry, IMU, GNSS and pose
measurements to estimate a vehicle's pose, velocity and orientation over time.
It also provides offline georeferencing of keyframe-based maps.

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
| Tests (6) | `mola_state_estimation_smoother/tests/test-*.cpp` |
| Integration tests | `mola_state_estimation_smoother/test/integration/test_*.py` |

Key traits:
- Inherits `mola::NavStateFilter`, `mola::LocalizationSourceBase`,
  `mola::MapSourceBase`.
- Uses Pimpl pattern (`GtsamImpl`) to hide GTSAM details.
- Sliding time window of keyframes (default 2.5 s).
- Two kinematic models: `ConstantVelocity` and `Tricycle` (Ackermann).
- Multi-frame-aware: tracks multiple odometry sources by `frame_id`.
- Optional ENU-to-map georeferencing from GNSS.
- Thread-safe (`std::recursive_mutex`).
- Configuration via YAML with `${ENV_VAR|default}` substitution.

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
bridge. It publishes `map -> base_link` on `/tf` (does **not** follow REP 105's
`map -> odom -> base_link` chain by default).

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

## Code style

The project uses `clang-format` (config in `.clang-format` at repo root).
CI enforces formatting via `.github/workflows/check-clang-format.yml`.
Don't use long hyphens. Use American spelling. Use braced statements instead of
one-liners.
