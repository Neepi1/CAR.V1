#include <gtest/gtest.h>

#include "robot_api_server/features/floor_switch/floor_switch_http_transaction.hpp"

namespace
{

robot_api_server::FloorSwitchHttpTarget target()
{
  return robot_api_server::FloorSwitchHttpTarget{
    "B11", "F2", "map-f2", 42U,
    "sha256:aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa"};
}

robot_api_server::FloorSwitchHttpRuntimeContext ready_runtime()
{
  const auto expected = target();
  return {
    true,
    true,
    "ready",
    expected.building_id,
    expected.floor_id,
    expected.map_id,
    expected.asset_epoch,
    expected.asset_digest,
    7U};
}

TEST(FloorSwitchHttpTransaction, BlocksSubmissionUntilSourceRuntimeIsReady)
{
  auto runtime = ready_runtime();
  runtime.confirmed = false;
  runtime.state = "starting";

  const auto decision =
    robot_api_server::evaluate_floor_switch_runtime_admission(target(), runtime);

  EXPECT_FALSE(decision.permitted);
  EXPECT_FALSE(decision.already_active);
  EXPECT_EQ(decision.code, "FLOOR_SWITCH_RUNTIME_NOT_READY");
}

TEST(FloorSwitchHttpTransaction, RecognizesExactActiveTargetAsIdempotent)
{
  const auto decision =
    robot_api_server::evaluate_floor_switch_runtime_admission(
    target(), ready_runtime());

  EXPECT_TRUE(decision.permitted);
  EXPECT_TRUE(decision.already_active);
  EXPECT_EQ(decision.code, "FLOOR_SWITCH_ALREADY_ACTIVE");
}

TEST(FloorSwitchHttpTransaction, RejectsSameMapIdWithDifferentAssetIdentity)
{
  auto runtime = ready_runtime();
  ++runtime.asset_epoch;

  const auto decision =
    robot_api_server::evaluate_floor_switch_runtime_admission(target(), runtime);

  EXPECT_FALSE(decision.permitted);
  EXPECT_EQ(decision.code, "FLOOR_SWITCH_RUNTIME_IDENTITY_MISMATCH");
}

TEST(FloorSwitchHttpTransaction, AllowsDifferentTargetFromReadySourceRuntime)
{
  auto runtime = ready_runtime();
  runtime.floor_id = "F1";
  runtime.map_id = "map-f1";
  runtime.asset_epoch = 9U;

  const auto decision =
    robot_api_server::evaluate_floor_switch_runtime_admission(target(), runtime);

  EXPECT_TRUE(decision.permitted);
  EXPECT_FALSE(decision.already_active);
  EXPECT_EQ(decision.code, "OK");
}

TEST(FloorSwitchHttpTransaction, RequiresExactCommittedIdentityForSuccess)
{
  robot_api_server::FloorSwitchHttpTransaction transaction;
  const auto started = transaction.start("manual-floor-switch-1", target());
  ASSERT_TRUE(started.accepted);
  EXPECT_EQ(started.snapshot.state, "SUBMITTING");
  EXPECT_TRUE(transaction.observe_goal_accepted("manual-floor-switch-1"));

  robot_api_server::FloorSwitchHttpOutcome wrong;
  wrong.success = true;
  wrong.active_building_id = "B11";
  wrong.active_floor_id = "F2";
  wrong.active_map_id = "different-map";
  wrong.asset_epoch = 42U;
  wrong.asset_digest = target().asset_digest;
  wrong.runtime_context_valid = true;
  EXPECT_TRUE(transaction.finish("manual-floor-switch-1", wrong));

  const auto finished = transaction.snapshot("manual-floor-switch-1");
  ASSERT_TRUE(finished.has_value());
  EXPECT_TRUE(finished->terminal());
  EXPECT_FALSE(finished->success);
  EXPECT_EQ(finished->state, "FAILED");
}

TEST(FloorSwitchHttpTransaction, CancelWaitsForExplicitActionTerminal)
{
  robot_api_server::FloorSwitchHttpTransaction transaction;
  ASSERT_TRUE(transaction.start("manual-floor-switch-2", target()).accepted);
  ASSERT_TRUE(transaction.observe_goal_accepted("manual-floor-switch-2"));

  const auto cancel = transaction.request_cancel("manual-floor-switch-2");
  ASSERT_TRUE(cancel.accepted);
  EXPECT_EQ(cancel.snapshot.state, "CANCELING");
  EXPECT_FALSE(cancel.snapshot.terminal());

  robot_api_server::FloorSwitchHttpOutcome cancelled;
  cancelled.cancelled = true;
  cancelled.detail = "exact action goal reached CANCELED";
  ASSERT_TRUE(transaction.finish("manual-floor-switch-2", cancelled));
  const auto finished = transaction.snapshot("manual-floor-switch-2");
  ASSERT_TRUE(finished.has_value());
  EXPECT_EQ(finished->state, "CANCELLED");
  EXPECT_TRUE(finished->terminal());
}

TEST(FloorSwitchHttpTransaction, RejectsOverlappingTransaction)
{
  robot_api_server::FloorSwitchHttpTransaction transaction;
  ASSERT_TRUE(transaction.start("manual-floor-switch-3", target()).accepted);
  const auto overlapping =
    transaction.start("manual-floor-switch-4", target());
  EXPECT_FALSE(overlapping.accepted);
  EXPECT_EQ(overlapping.code, "FLOOR_SWITCH_TRANSACTION_BUSY");
  EXPECT_EQ(overlapping.snapshot.transaction_id, "manual-floor-switch-3");
}

TEST(FloorSwitchHttpTransaction, UnprovenTerminalRetainsRecoveryFence)
{
  robot_api_server::FloorSwitchHttpTransaction transaction;
  ASSERT_TRUE(transaction.start("manual-floor-switch-5", target()).accepted);

  robot_api_server::FloorSwitchHttpOutcome unknown;
  unknown.terminal_proven = false;
  unknown.detail = "action terminal remained unknown";
  ASSERT_TRUE(transaction.finish("manual-floor-switch-5", unknown));

  const auto retained = transaction.snapshot("manual-floor-switch-5");
  ASSERT_TRUE(retained.has_value());
  EXPECT_EQ(retained->state, "UNKNOWN");
  EXPECT_TRUE(retained->recovery_required);

  const auto replacement =
    transaction.start("manual-floor-switch-6", target());
  EXPECT_FALSE(replacement.accepted);
  EXPECT_EQ(replacement.code, "FLOOR_SWITCH_RECOVERY_REQUIRED");
  EXPECT_EQ(replacement.snapshot.transaction_id, "manual-floor-switch-5");
}

}  // namespace
