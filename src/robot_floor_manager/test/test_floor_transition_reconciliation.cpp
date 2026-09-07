#include <gtest/gtest.h>

#include "robot_floor_manager/floor_transition_reconciliation.hpp"

namespace robot_floor_manager
{
namespace
{

TEST(FloorTransitionReconciliation, ExactLateEvidenceOverridesUnknownRpcOutcome)
{
  const auto result = reconcile_explicit_localization(
    false,
    true,
    false,
    "timed out waiting for global localization response");

  EXPECT_TRUE(result.success);
  EXPECT_EQ(result.code, "OK_RECONCILED");
  EXPECT_NE(result.detail.find("exact target evidence"), std::string::npos);
}

TEST(FloorTransitionReconciliation, AcceptedRpcWithoutEvidenceRemainsUnproven)
{
  const auto result = reconcile_explicit_localization(true, false, false, "");

  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.code, "EXPLICIT_LOCALIZATION_UNPROVEN");
}

TEST(FloorTransitionReconciliation, RejectedRpcWithoutEvidenceReportsOriginalFailure)
{
  const auto result = reconcile_explicit_localization(
    false,
    false,
    false,
    "global localization trigger rejected: not ready");

  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.code, "EXPLICIT_LOCALIZATION_REJECTED");
  EXPECT_NE(result.detail.find("not ready"), std::string::npos);
}

TEST(FloorTransitionReconciliation, CancellationWinsWhenNoEvidenceWasProven)
{
  const auto result = reconcile_explicit_localization(false, false, true, "timeout");

  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.code, "CANCELLED");
}

TEST(FloorTransitionReconciliation, PostDispatchFailuresAreNotRetryable)
{
  for (const auto * code : {
        "FRESH_LOCALIZATION_RETRY_REQUIRED",
        "ISAAC_DISPATCH_FAILED",
        "ISAAC_DISPATCH_TIMEOUT",
        "LOCALIZATION_RESULT_TIMEOUT",
        "BRIDGE_ACCEPT_TIMEOUT",
        "MAP_TO_ODOM_TIMEOUT"})
  {
    const auto result = classify_explicit_localization_failure(
      std::string("global localization trigger rejected: failure_code=") + code +
      " dispatch_state=dispatched");
    EXPECT_FALSE(result.retryable) << code;
    EXPECT_EQ(result.failure_code, code);
  }
}

TEST(FloorTransitionReconciliation, ProvenPreDispatchFailuresAreRetryable)
{
  for (const auto * code : {
        "LOCALIZER_POST_RELOAD_NOT_READY",
        "LOCALIZER_OPERATION_BUSY",
        "LOCALIZER_INPUT_NOT_FRESH",
        "ISAAC_SERVICE_UNAVAILABLE",
        "BRIDGE_FORCE_ACCEPT_UNAVAILABLE",
        "BRIDGE_FORCE_ACCEPT_TIMEOUT"})
  {
    const auto result = classify_explicit_localization_failure(
      std::string("global localization trigger rejected: failure_code=") + code +
      " dispatch_state=not_dispatched detail");
    EXPECT_TRUE(result.retryable) << code;
    EXPECT_EQ(result.failure_code, code);
  }
}

TEST(FloorTransitionReconciliation, BridgeOutcomesNeverAuthorizeAnotherIsaacDispatch)
{
  EXPECT_FALSE(
    classify_explicit_localization_failure(
      "failure_code=MAP_TO_ODOM_TIMEOUT has_map_to_odom=true "
      "owner=robot_localization_bridge").retryable);
  EXPECT_FALSE(
    classify_explicit_localization_failure(
      "failure_code=BRIDGE_ACCEPT_TIMEOUT "
      "last_reject_reason=isaac_triggered_pose_stale_ms=5434 gate_mode=triggered").retryable);
  EXPECT_FALSE(
    classify_explicit_localization_failure(
      "failure_code=BRIDGE_REJECTED_RESULT last_reject_reason=AMCL_POSE_STALE").retryable);

  EXPECT_FALSE(
    classify_explicit_localization_failure(
      "failure_code=MAP_TO_ODOM_TIMEOUT has_map_to_odom=false").retryable);
  EXPECT_FALSE(
    classify_explicit_localization_failure(
      "failure_code=BRIDGE_ACCEPT_TIMEOUT bridge did not accept localization_result").retryable);
  EXPECT_FALSE(
    classify_explicit_localization_failure(
      "failure_code=BRIDGE_REJECTED_RESULT last_reject_reason=INNOVATION_TOO_LARGE").retryable);
}

TEST(FloorTransitionReconciliation, StructuralAndMalformedFailuresRemainTerminal)
{
  for (const auto & error : {
        std::string("failure_code=MAP_TO_ODOM_WRONG_OWNER owner=amcl"),
        std::string("failure_code=TF_HISTORY_MISSING odom history unavailable"),
        std::string("not ready; FRESH_LOCALIZATION_RETRY_REQUIRED without a failure code"),
        std::string("failure_code=FRESH_LOCALIZATION_RETRY_REQUIRED_SUFFIX")})
  {
    EXPECT_FALSE(classify_explicit_localization_failure(error).retryable) << error;
  }
}

}  // namespace
}  // namespace robot_floor_manager
