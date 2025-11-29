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
