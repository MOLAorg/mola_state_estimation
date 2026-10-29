# C++ unit tests:
mola_add_test(
  TARGET  test_imu_helpers
  SOURCES test_imu_helpers.cpp
  LINK_LIBRARIES
    mola::mola_gtsam_factors
)

mola_add_test(
  TARGET  test_pose3_rotation_yaw_bias_factor
  SOURCES test_pose3_rotation_yaw_bias_factor.cpp
  LINK_LIBRARIES
    mola::mola_gtsam_factors
)
