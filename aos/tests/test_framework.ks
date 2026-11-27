// test_framework.ks - Minimal testing helpers for kOS scripts
// Usage:
//   RUNPATH("0:/Ships/Script/aos/tests/test_framework.ks").
//   TEST_RESET().
//   TEST_BEGIN("example").
//   ASSERT_TRUE(TRUE, "should never fail").
//   TEST_END().
//   TEST_SUMMARY().

DECLARE GLOBAL TEST_TOTAL TO 0.
DECLARE GLOBAL TEST_PASSED TO 0.
DECLARE GLOBAL TEST_FAILED TO 0.
DECLARE GLOBAL TEST_LOG TO LIST().
DECLARE GLOBAL TEST_CURRENT_NAME TO "".
DECLARE GLOBAL TEST_CURRENT_ERRORS TO LIST().
DECLARE GLOBAL TEST_CURRENT_ASSERTS TO 0.

FUNCTION TEST_RESET {
  SET TEST_TOTAL TO 0.
  SET TEST_PASSED TO 0.
  SET TEST_FAILED TO 0.
  SET TEST_LOG TO LIST().
}.

FUNCTION TEST_BEGIN {
  PARAMETER name.
  SET TEST_CURRENT_NAME TO name.
  SET TEST_CURRENT_ERRORS TO LIST().
  SET TEST_CURRENT_ASSERTS TO 0.
  PRINT("[TEST] " + name).
}.

FUNCTION __TEST_REGISTER_FAILURE {
  PARAMETER message.
  TEST_CURRENT_ERRORS:ADD(message).
}.

FUNCTION ASSERT_TRUE {
  PARAMETER condition, message IS "".
  SET TEST_CURRENT_ASSERTS TO TEST_CURRENT_ASSERTS + 1.
  IF NOT condition {
    IF message = "" {
      SET message TO "Expected condition to be TRUE but it was FALSE.".
    }.
    __TEST_REGISTER_FAILURE(message).
  }.
}.

FUNCTION ASSERT_FALSE {
  PARAMETER condition, message IS "".
  IF message = "" {
    SET message TO "Expected condition to be FALSE but it was TRUE.".
  }.
  ASSERT_TRUE(NOT condition, message).
}.

FUNCTION ASSERT_EQUAL {
  PARAMETER expected, actual, message IS "".
  IF message = "" {
    SET message TO "Expected " + expected + " but got " + actual + ".".
  }.
  ASSERT_TRUE(expected = actual, message).
}.

FUNCTION ASSERT_NOT_EQUAL {
  PARAMETER not_expected, actual, message IS "".
  IF message = "" {
    SET message TO "Did not expect " + actual + ".".
  }.
  ASSERT_TRUE(not_expected <> actual, message).
}.

FUNCTION ASSERT_LIST_EQUALS {
  PARAMETER expected, actual, message IS "".
  SET TEST_CURRENT_ASSERTS TO TEST_CURRENT_ASSERTS + 1.

  IF expected:LENGTH <> actual:LENGTH {
    LOCAL len_message IS message.
    IF len_message = "" {
      SET len_message TO "Expected list length " + expected:LENGTH + " but got " + actual:LENGTH + ".".
    }.
    __TEST_REGISTER_FAILURE(len_message).
    RETURN.
  }.

  LOCAL idx IS 0.
  UNTIL idx = expected:LENGTH {
    IF expected[idx] <> actual[idx] {
      LOCAL diff_message IS message.
      IF diff_message = "" {
        SET diff_message TO "Lists differ at index " + idx + ": expected " + expected[idx] + " but got " + actual[idx] + ".".
      }.
      __TEST_REGISTER_FAILURE(diff_message).
      RETURN.
    }.
    SET idx TO idx + 1.
  }.
}.

FUNCTION TEST_END {
  LOCAL test_status IS "PASS".

  IF TEST_CURRENT_ERRORS:LENGTH > 0 {
    SET test_status TO "FAIL".
    SET TEST_FAILED TO TEST_FAILED + 1.
    PRINT("[FAIL] " + TEST_CURRENT_NAME).
    FOR message IN TEST_CURRENT_ERRORS {
      PRINT("  -> " + message).
    }.
  } ELSE {
    SET TEST_PASSED TO TEST_PASSED + 1.
    PRINT("[PASS] " + TEST_CURRENT_NAME + " (" + TEST_CURRENT_ASSERTS + " assertions)").
  }.

  SET TEST_TOTAL TO TEST_TOTAL + 1.

  LOCAL entry IS LEXICON().
  SET entry["name"] TO TEST_CURRENT_NAME.
  SET entry["status"] TO test_status.
  SET entry["asserts"] TO TEST_CURRENT_ASSERTS.
  SET entry["errors"] TO TEST_CURRENT_ERRORS.
  TEST_LOG:ADD(entry).
}.

FUNCTION TEST_SUMMARY {
  PRINT("----------------------------------------------").
  PRINT("Test run complete").
  PRINT("  Total: " + TEST_TOTAL).
  PRINT("  Passed: " + TEST_PASSED).
  PRINT("  Failed: " + TEST_FAILED).
  IF TEST_FAILED > 0 {
    PRINT("  Review failure details above.").
  }.
  PRINT("----------------------------------------------").
}.
