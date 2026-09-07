#include <gtest/gtest.h>

#include "robot_api_server/features/floor_switch/floor_runtime_interlock.hpp"

namespace
{

using robot_api_server::FloorRuntimeInterlock;

TEST(FloorRuntimeInterlock, MissingTypedEvidencePreservesLegacyRuntime)
{
  FloorRuntimeInterlock interlock;

  const auto decision = interlock.decision();

  EXPECT_FALSE(decision.blocked);
  EXPECT_EQ(decision.code, "FLOOR_RUNTIME_LEGACY_UNSCOPED");
}

TEST(FloorRuntimeInterlock, NonMutatingPreflightFailureDoesNotLatch)
{
  FloorRuntimeInterlock interlock;

  interlock.observe_floor_switch_status(
    "tx-preflight", "PREFLIGHT", "VALIDATE_GOAL", 0U, "validating");
  EXPECT_FALSE(interlock.decision().blocked);

  interlock.observe_floor_switch_status(
    "tx-preflight", "BLOCKED", "LIVE_FLOOR_SWITCH_DISABLED", 41U,
    "preflight-only adapter changed no runtime asset");
  EXPECT_FALSE(interlock.decision().blocked);
}

TEST(FloorRuntimeInterlock, MutationStateBlocksUntilSafeTerminalStatus)
{
  FloorRuntimeInterlock interlock;

  interlock.observe_floor_switch_status(
    "tx-live", "LOADING_NAV_MAP", "LOAD_NAV_MAP", 0U, "loading");
  auto decision = interlock.decision();
  EXPECT_TRUE(decision.blocked);
  EXPECT_EQ(decision.code, "FLOOR_TRANSITION_ACTIVE");
  EXPECT_EQ(decision.transaction_id, "tx-live");

  interlock.observe_floor_switch_status(
    "tx-live", "COMPLETE", "COMPLETE", 0U, "committed");
  EXPECT_FALSE(interlock.decision().blocked);
}

TEST(FloorRuntimeInterlock, TypedHealthInvalidityBlocksAndRecoveryClears)
{
  FloorRuntimeInterlock interlock;

  interlock.observe_localization_health(true, false, "transition active");
  EXPECT_TRUE(interlock.decision().blocked);
  EXPECT_EQ(interlock.decision().code, "FLOOR_TRANSITION_ACTIVE");

  interlock.observe_localization_health(false, true, "target runtime committed");
  EXPECT_FALSE(interlock.decision().blocked);
}

TEST(FloorRuntimeInterlock, FailedLockCannotBeHiddenByHealthyBridgeSample)
{
  FloorRuntimeInterlock interlock;

  interlock.observe_floor_switch_status(
    "tx-failed", "FAILED_LOCKED", "HOLD_AND_LOCK", 99U, "manual recovery required");
  interlock.observe_localization_health(false, true, "inconsistent healthy sample");

  const auto decision = interlock.decision();
  EXPECT_TRUE(decision.blocked);
  EXPECT_EQ(decision.code, "FLOOR_TRANSITION_FAILED_LOCKED");
  EXPECT_EQ(decision.transaction_id, "tx-failed");
}

TEST(FloorRuntimeInterlock, FailedHealthDetailIsFailClosed)
{
  FloorRuntimeInterlock interlock;

  interlock.observe_localization_health(false, true, "FAILED_LOCKED: recovery required");

  const auto decision = interlock.decision();
  EXPECT_TRUE(decision.blocked);
  EXPECT_EQ(decision.code, "FLOOR_TRANSITION_FAILED_LOCKED");
}

TEST(FloorRuntimeInterlock, FailedFloorLockSurvivesLaterPreflightStatus)
{
  FloorRuntimeInterlock interlock;

  interlock.observe_floor_switch_status(
    "tx-failed", "FAILED_LOCKED", "HOLD_AND_LOCK", 99U, "manual recovery required");
  interlock.observe_floor_switch_status(
    "tx-new", "BLOCKED", "LIVE_FLOOR_SWITCH_DISABLED", 41U,
    "a later preflight changed no runtime asset");

  const auto decision = interlock.decision();
  EXPECT_TRUE(decision.blocked);
  EXPECT_EQ(decision.code, "FLOOR_TRANSITION_FAILED_LOCKED");
  EXPECT_EQ(decision.transaction_id, "tx-failed");
}

TEST(FloorRuntimeInterlock, FailedHealthLockSurvivesHealthyHeartbeat)
{
  FloorRuntimeInterlock interlock;

  interlock.observe_localization_health(false, false, "FAILED_LOCKED: recovery required");
  interlock.observe_localization_health(false, true, "inconsistent healthy heartbeat");

  const auto decision = interlock.decision();
  EXPECT_TRUE(decision.blocked);
  EXPECT_EQ(decision.code, "FLOOR_TRANSITION_FAILED_LOCKED");
}

TEST(FloorRuntimeInterlock, UnknownStatusDoesNotInventAReadinessFailure)
{
  FloorRuntimeInterlock interlock;

  interlock.observe_floor_switch_status(
    "tx-future", "FUTURE_DIAGNOSTIC_ONLY", "REPORT", 0U, "new non-mutating state");

  EXPECT_FALSE(interlock.decision().blocked);
}

}  // namespace
