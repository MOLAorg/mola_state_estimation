# Unit tests:
mola_add_test(
  TARGET  test-navstate-basic
  SOURCES test-navstate-basic.cpp
  LINK_LIBRARIES
    mola::mola_state_estimation_smoother
)
