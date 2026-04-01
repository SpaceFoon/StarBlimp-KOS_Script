# AOS Unit Tests

This directory contains a lightweight unit testing harness for the Airship Operating System (AOS) kOS scripts. The goal is to make it easy to exercise logic that does not require an active vessel or part modules, such as the action-group helpers in `aos/ag_lib.ks`.

## Running the tests

1. Load your kOS CPU and ensure the `Ships/Script/aos` folder is available on the `0:/` volume (standard for local disk deployments).
2. In the kOS terminal, execute:

   ```
   RUNPATH("0:/aos/tests/run_tests").
   ```

3. The runner outputs a per-test status and a final summary with totals.

If you want to re-run the tests, simply invoke the same command again. The framework resets its counters at the beginning of each run.

## Adding new tests

- Create a new file alongside this one (for example `test_orbital.ks`).
- Start the file by loading dependencies with `RUNONCEPATH` calls if it relies on other libraries.
- For each scenario, wrap your assertions in `TEST_BEGIN("name")` / `TEST_END()`.
- Use the assertion helpers:
  - `ASSERT_TRUE(condition, message)`
  - `ASSERT_FALSE(condition, message)`
  - `ASSERT_EQUAL(expected, actual, message)`
  - `ASSERT_NOT_EQUAL(not_expected, actual, message)`
  - `ASSERT_LIST_EQUALS(expected_list, actual_list, message)`
- Register the new suite in `run_tests.ks` by adding another `RUNPATH("0:/aos/tests/your_test_file")`.

### Tips for testable code

- Prefer extracting pure helper functions that accept parameters instead of reading part state directly.
- Consider adding optional "test hooks" (for example, injectable data sources) when working with code that touches game state.
- Keep test files focused on a single module to make failures easier to diagnose.
