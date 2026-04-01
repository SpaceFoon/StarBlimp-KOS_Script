// test_ag_lib.ks - Unit tests for aos/ag_lib.ks helpers

// Ensure dependencies are available when this file is run standalone.
RUNONCEPATH("0:/aos/tests/test_framework.ks").
RUNONCEPATH("0:/aos/ag_legend.ks").
RUNONCEPATH("0:/aos/ag_constants.ks").
RUNONCEPATH("0:/aos/ag_lib.ks").

PRINT("Running AG library tests...").

// Ensure legend definitions are loaded before executing tests.
IF NOT AGX_LEGEND:HASKEY("jets") {
  PRINT("[WARN] AGX_LEGEND is missing expected keys. Did ag_legend.ks run?").
}.

// Test: scalar ID input returns the same ID
TEST_BEGIN("AG_RESOLVE_IDS returns scalar ID").
LOCAL result_scalar IS AG_RESOLVE_IDS(5).
LOCAL expected_scalar IS LIST().
expected_scalar:ADD(5).
ASSERT_LIST_EQUALS(expected_scalar, result_scalar, "Scalar ID should round-trip.").
TEST_END().

// Test: string label resolves to mapped ID
TEST_BEGIN("AG_RESOLVE_IDS resolves label").
LOCAL result_label IS AG_RESOLVE_IDS("jets").
LOCAL expected_label IS LIST().
expected_label:ADD(AGX_LEGEND["jets"]).
ASSERT_LIST_EQUALS(expected_label, result_label, "Should resolve label to ID from legend.").
TEST_END().

// Test: nested lists flatten with deduplicated IDs
TEST_BEGIN("AG_RESOLVE_IDS flattens list input").
LOCAL nested_input IS LIST().
nested_input:ADD("jets").
LOCAL inner IS LIST().
inner:ADD(AGX_LEGEND["jets"]).
inner:ADD(AGX_LEGEND["fan_system"]).
nested_input:ADD(inner).
nested_input:ADD(AGX_LEGEND["fan_system"]).

LOCAL result_nested IS AG_RESOLVE_IDS(nested_input).
LOCAL expected_nested IS LIST().
expected_nested:ADD(AGX_LEGEND["jets"]).
expected_nested:ADD(AGX_LEGEND["fan_system"]).
ASSERT_LIST_EQUALS(expected_nested, result_nested, "Nested input should flatten and deduplicate IDs.").
TEST_END().

// Test: unknown label returns empty list
TEST_BEGIN("AG_RESOLVE_IDS handles unknown label").
LOCAL result_unknown IS AG_RESOLVE_IDS("this_label_should_not_exist").
LOCAL expected_empty IS LIST().
ASSERT_LIST_EQUALS(expected_empty, result_unknown, "Unknown label should produce empty list.").
TEST_END().
