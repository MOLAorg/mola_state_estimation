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
  TARGET  test-publish-map-to-odom
  SOURCES test-publish-map-to-odom.cpp
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
