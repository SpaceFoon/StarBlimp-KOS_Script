// run_tests.ks - Entry point for AOS unit tests
// Execute from the kOS terminal: RUNPATH("0:/Ships/Script/aos/tests/run_tests.ks").

PRINT("==============================================").
PRINT("AOS Unit Test Runner").
PRINT("==============================================").

// Load supporting libraries (paths relative to CPU volume root 0:/)
RUNPATH("0:/aos/tests/test_framework").
RUNPATH("0:/aos/ag_legend").
RUNPATH("0:/aos/ag_constants").
RUNPATH("0:/aos/ag_lib").

// Reset counters for fresh run
TEST_RESET().

// Execute individual test suites
RUNPATH("0:/aos/tests/test_ag_lib").

// Print summary results
TEST_SUMMARY().
