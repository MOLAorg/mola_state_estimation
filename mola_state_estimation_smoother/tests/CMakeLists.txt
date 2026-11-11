# Unit tests:
mola_add_test(
  TARGET  test-navstate-basic
  SOURCES test-navstate-basic.cpp
  LINK_LIBRARIES
    mola::mola_state_estimation_smoother
)

mola_add_test(
  TARGET  test-navstate-odom-gnss-fusion
  SOURCES test-navstate-odom-gnss-fusion.cpp
  LINK_LIBRARIES
    mola::mola_state_estimation_smoother
)

mola_add_test(
  TARGET  test-odometry-long-drift
  SOURCES test-odometry-long-drift.cpp
  LINK_LIBRARIES
    mola::mola_state_estimation_smoother
)

mola_add_test(
  TARGET  test-odometry-pose-source-fidelity
  SOURCES test-odometry-pose-source-fidelity.cpp
  LINK_LIBRARIES
    mola::mola_state_estimation_smoother
)

mola_add_test(
  TARGET  test-static-gnss-imu-orientation
  SOURCES test-static-gnss-imu-orientation.cpp
  LINK_LIBRARIES
    mola::mola_state_estimation_smoother
)

mola_add_test(
  TARGET  test-relocalize-set-geo-reference
  SOURCES test-relocalize-set-geo-reference.cpp
  LINK_LIBRARIES
    mola::mola_state_estimation_smoother
)

mola_add_test(
  TARGET  test-relocalize-without-attitude
  SOURCES test-relocalize-without-attitude.cpp
  LINK_LIBRARIES
    mola::mola_state_estimation_smoother
)

mola_add_test(
  TARGET  test-relocalize-not-converged-on-odometry-only
  SOURCES test-relocalize-not-converged-on-odometry-only.cpp
  LINK_LIBRARIES
    mola::mola_state_estimation_smoother
)

mola_add_test(
  TARGET  test-odometry-relative-same-keyframe
  SOURCES test-odometry-relative-same-keyframe.cpp
  LINK_LIBRARIES
    mola::mola_state_estimation_smoother
)

mola_add_test(
  TARGET  test-pose-keyframe-decimation
  SOURCES test-pose-keyframe-decimation.cpp
  LINK_LIBRARIES
    mola::mola_state_estimation_smoother
)

mola_add_test(
  TARGET  test-pose-robust-huber
  SOURCES test-pose-robust-huber.cpp
  LINK_LIBRARIES
    mola::mola_state_estimation_smoother
)

mola_add_test(
  TARGET  test-relative-pose-increment-growth
  SOURCES test-relative-pose-increment-growth.cpp
  LINK_LIBRARIES
    mola::mola_state_estimation_smoother
)

mola_add_test(
  TARGET  test-relative-pose-increment-sigma
  SOURCES test-relative-pose-increment-sigma.cpp
  LINK_LIBRARIES
    mola::mola_state_estimation_smoother
)

mola_add_test(
  TARGET  test-relative-source-map-to-odom
  SOURCES test-relative-source-map-to-odom.cpp
  LINK_LIBRARIES
    mola::mola_state_estimation_smoother
)

mola_add_test(
  TARGET  test-relative-pose-factors
  SOURCES test-relative-pose-factors.cpp
  LINK_LIBRARIES
    mola::mola_state_estimation_smoother
)

mola_add_test(
  TARGET  test-two-odometries
  SOURCES test-two-odometries.cpp
  LINK_LIBRARIES
    mola::mola_state_estimation_smoother
)

mola_add_test(
  TARGET  test-lidar-imu-leveling
  SOURCES test-lidar-imu-leveling.cpp
  LINK_LIBRARIES
    mola::mola_state_estimation_smoother
)

mola_add_test(
  TARGET  test-twist-fusion
  SOURCES test-twist-fusion.cpp
  LINK_LIBRARIES
    mola::mola_state_estimation_smoother
)

mola_add_test(
  TARGET  test-kinematic-splice-no-triangle
  SOURCES test-kinematic-splice-no-triangle.cpp
  LINK_LIBRARIES
    mola::mola_state_estimation_smoother
)

mola_add_test(
  TARGET  test-navstate-stamp-monotonic
  SOURCES test-navstate-stamp-monotonic.cpp
  LINK_LIBRARIES
    mola::mola_state_estimation_smoother
)

mola_add_test(
  TARGET  test-keyframe-decimation
  SOURCES test-keyframe-decimation.cpp
  LINK_LIBRARIES
    mola::mola_state_estimation_smoother
)

mola_add_test(
  TARGET  test-async-navstate
  SOURCES test-async-navstate.cpp
  LINK_LIBRARIES
    mola::mola_state_estimation_smoother
)

mola_add_test(
  TARGET  test-deferred-pose-covariance
  SOURCES test-deferred-pose-covariance.cpp
  LINK_LIBRARIES
    mola::mola_state_estimation_smoother
)

mola_add_test(
  TARGET  test-publish-map-to-odom
  SOURCES test-publish-map-to-odom.cpp
  LINK_LIBRARIES
    mola::mola_state_estimation_smoother
)

mola_add_test(
  TARGET  test-publish-needs-map-observation
  SOURCES test-publish-needs-map-observation.cpp
  LINK_LIBRARIES
    mola::mola_state_estimation_smoother
)

mola_add_test(
  TARGET  test-predict-twist-filter
  SOURCES test-predict-twist-filter.cpp
  LINK_LIBRARIES
    mola::mola_state_estimation_smoother
)

mola_add_test(
  TARGET  test-prediction-residual-seed
  SOURCES test-prediction-residual-seed.cpp
  LINK_LIBRARIES
    mola::mola_state_estimation_smoother
)

mola_add_test(
  TARGET  test-wheel-odometry-frame-collision
  SOURCES test-wheel-odometry-frame-collision.cpp
  LINK_LIBRARIES
    mola::mola_state_estimation_smoother
)

mola_add_test(
  TARGET  test-planar-odometry-out-of-plane
  SOURCES test-planar-odometry-out-of-plane.cpp
  LINK_LIBRARIES
    mola::mola_state_estimation_smoother
)

mola_add_test(
  TARGET  test-imu-gyro-abrupt-rotation
  SOURCES test-imu-gyro-abrupt-rotation.cpp
  LINK_LIBRARIES
    mola::mola_state_estimation_smoother
)

mola_add_test(
  TARGET  test-imu-gyro-solver-stability
  SOURCES test-imu-gyro-solver-stability.cpp
  LINK_LIBRARIES
    mola::mola_state_estimation_smoother
)


mola_add_test(
  TARGET  test-relative-pose-sigma-floor
  SOURCES test-relative-pose-sigma-floor.cpp
  LINK_LIBRARIES
    mola::mola_state_estimation_smoother
)

mola_add_test(
  TARGET  test-finalized-trajectory
  SOURCES test-finalized-trajectory.cpp
  LINK_LIBRARIES
    mola::mola_state_estimation_smoother
)

mola_add_test(
  TARGET  test-imu-preintegration-leveling
  SOURCES test-imu-preintegration-leveling.cpp
  LINK_LIBRARIES
    mola::mola_state_estimation_smoother
)

mola_add_test(
  TARGET  test-imu-preintegration-live-georef
  SOURCES test-imu-preintegration-live-georef.cpp
  LINK_LIBRARIES
    mola::mola_state_estimation_smoother
)

mola_add_test(
  TARGET  test-imu-preintegration-without-imu
  SOURCES test-imu-preintegration-without-imu.cpp
  LINK_LIBRARIES
    mola::mola_state_estimation_smoother
)

mola_add_test(
  TARGET  test-imu-preintegration-accel-units
  SOURCES test-imu-preintegration-accel-units.cpp
  LINK_LIBRARIES
    mola::mola_state_estimation_smoother
)

mola_add_test(
  TARGET  test-gauge-anchors
  SOURCES test-gauge-anchors.cpp
  LINK_LIBRARIES
    mola::mola_state_estimation_smoother
)

mola_add_test(
  TARGET  test-live-georef-without-attitude
  SOURCES test-live-georef-without-attitude.cpp
  LINK_LIBRARIES
    mola::mola_state_estimation_smoother
)

mola_add_test(
  TARGET  test-georef-enu-outputs
  SOURCES test-georef-enu-outputs.cpp
  LINK_LIBRARIES
    mola::mola_state_estimation_smoother
)

mola_add_test(
  TARGET  test-imu-placeholder-attitude
  SOURCES test-imu-placeholder-attitude.cpp
  LINK_LIBRARIES
    mola::mola_state_estimation_smoother
)

mola_add_test(
  TARGET  test-imu-attitude-yaw-bias
  SOURCES test-imu-attitude-yaw-bias.cpp
  LINK_LIBRARIES
    mola::mola_state_estimation_smoother
)

mola_add_test(
  TARGET  test-imu-gravity-vibration
  SOURCES test-imu-gravity-vibration.cpp
  LINK_LIBRARIES
    mola::mola_state_estimation_smoother
)

mola_add_test(
  TARGET  test-imu-bias-provider
  SOURCES test-imu-bias-provider.cpp
  LINK_LIBRARIES
    mola::mola_state_estimation_smoother
)

mola_add_test(
  TARGET  test-gnss-innovation-gate
  SOURCES test-gnss-innovation-gate.cpp
  LINK_LIBRARIES
    mola::mola_state_estimation_smoother
)

mola_add_test(
  TARGET  test-pose-lead-guard
  SOURCES test-pose-lead-guard.cpp
  LINK_LIBRARIES
    mola::mola_state_estimation_smoother
)

mola_add_test(
  TARGET  test-reset-motion-drifted-odometry
  SOURCES test-reset-motion-drifted-odometry.cpp
  LINK_LIBRARIES
    mola::mola_state_estimation_smoother
)

mola_add_test(
  TARGET  test-newest-last-fixed-lag-smoother
  SOURCES test-newest-last-fixed-lag-smoother.cpp
          ../src/NewestLastFixedLagSmoother.cpp
  LINK_LIBRARIES
    mola::mola_gtsam_factors
    gtsam
    gtsam_unstable
)
