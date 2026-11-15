# Unit tests:
mola_add_test(
  TARGET  test-state-estimation-simple
  SOURCES test-state-estimation-simple.cpp
  LINK_LIBRARIES
    mola::mola_state_estimation_simple
)

# Fail fast if a query hangs:
set_tests_properties(run_test-state-estimation-simple PROPERTIES TIMEOUT 60)
