#include <gtest/gtest.h>

#include "robot_api_server/features/floor_switch/floor_runtime_interlock.hpp"

namespace
{

using robot_api_server::FloorRuntimeInterlock;

constexpr const char * kOfflineAssetOperations[] = {
  "elevator_config_save_draft", "elevator_config_save_draft_commit",
  "elevator_config_publish", "elevator_config_publish_commit",
  "elevator_config_rollback", "elevator_config_rollback_commit",
  "pose_save", "pose_save_commit", "pose_delete", "pose_delete_commit",
  "pose_batch_replace", "pose_batch_replace_commit",
};

constexpr const char * kSourceIndependentOperations[] = {
  "mapping_start", "mapping_start_worker", "mapping_start_context_clear",
  "mapping_process_launch", "mapping_save", "mapping_save_commit",
  "navigation_start", "navigation_runtime_resume", "navigation_runtime_resume_commit",
  "navigation_runtime_launch", "manual_localization", "localization_trigger",
  "live_floor_switch_start", "floor_switch", "floor_switch_noop_commit",
  "floor_switch_submit", "floor_selection_commit",
};

TEST(FloorRuntimeInterlock, InvalidSourceDoesNotCreateAnotherGlobalRecoveryLock)
{
  FloorRuntimeInterlock interlock;
  interlock.observe_floor_switch_status("old", "FAILED", "ABORTED", 99U, "target failed");
  interlock.observe_localization_health(false, false, "ABORTED_CONTEXT_INVALID");
  for (const auto * operation : kSourceIndependentOperations) {
    SCOPED_TRACE(operation);
    EXPECT_FALSE(interlock.decision_for_operation(operation).blocked);
  }
  EXPECT_TRUE(interlock.decision().blocked);
  EXPECT_EQ(interlock.decision().code, "FLOOR_RUNTIME_CONTEXT_INVALID");
}

TEST(FloorRuntimeInterlock, EverySourceIndependentStepStillRejectsCurrentActivity)
{
  for (const bool active_in_health : {false, true}) {
    FloorRuntimeInterlock interlock;
    interlock.observe_localization_health(active_in_health, false, "current evidence");
    if (!active_in_health) {
      interlock.observe_floor_switch_status("new", "RUNNING", "LOADING_FILTERS", 0U, "loading");
    }
    for (const auto * operation : kSourceIndependentOperations) {
      SCOPED_TRACE(operation);
      EXPECT_TRUE(interlock.decision_for_operation(operation).blocked);
      EXPECT_EQ(interlock.decision_for_operation(operation).code, "FLOOR_TRANSITION_ACTIVE");
    }
  }
}

TEST(FloorRuntimeInterlock, TerminalStatusDoesNotKeepItsLastRunningStageActive)
{
  FloorRuntimeInterlock interlock;
  interlock.observe_floor_switch_status("tx", "RUNNING", "FAILURE_CLEANUP", 99U, "cleanup");
  EXPECT_TRUE(interlock.decision().blocked);
  interlock.observe_floor_switch_status("tx", "FAILED", "FAILURE_CLEANUP", 99U, "ended");
  EXPECT_FALSE(interlock.decision_for_operation("live_floor_switch_start").blocked);
}

TEST(FloorRuntimeInterlock, ActualFloorManagerRunningEffectStatusBlocksRecoveryOperations)
{
  FloorRuntimeInterlock interlock;
  interlock.observe_localization_health(false, false, "source unlocalized before BEGIN");
  interlock.observe_floor_switch_status("tx", "RUNNING", "LOAD_NAV_MAP", 0U, "loading");
  for (const auto * operation : kSourceIndependentOperations) {
    SCOPED_TRACE(operation);
    EXPECT_TRUE(interlock.decision_for_operation(operation).blocked);
    EXPECT_EQ(interlock.decision_for_operation(operation).code, "FLOOR_TRANSITION_ACTIVE");
  }
}

TEST(FloorRuntimeInterlock, RejectedOtherTransactionDoesNotEndTheCurrentSwitch)
{
  FloorRuntimeInterlock interlock;
  interlock.observe_floor_switch_status("active", "LOADING_NAV_MAP", "LOAD_NAV_MAP", 0U, "loading");
  interlock.observe_floor_switch_status("rejected", "BLOCKED", "TRANSACTION_CONFLICT", 1U, "busy");
  EXPECT_TRUE(interlock.decision_for_operation("live_floor_switch_start").blocked);
  EXPECT_TRUE(interlock.decision_for_operation("elevator_config_publish").blocked);
  EXPECT_EQ(interlock.decision().transaction_id, "active");
  interlock.observe_floor_switch_status("active", "FAILED", "ABORTED", 99U, "cleanup finished");
  EXPECT_FALSE(interlock.decision_for_operation("live_floor_switch_start").blocked);
}

TEST(FloorRuntimeInterlock, TerminalFailureIsHistoryNotAnIndependentRuntimeLock)
{
  FloorRuntimeInterlock interlock;
  interlock.observe_floor_switch_status(
    "failed", "FAILED_LOCKED", "HOLD_AND_LOCK", 99U, "historical failure");
  interlock.observe_localization_health(false, true, "current runtime ready");
  EXPECT_FALSE(interlock.decision_for_operation("navigation_goal").blocked);
  EXPECT_FALSE(interlock.decision_for_operation("live_floor_switch_start").blocked);
}

TEST(FloorRuntimeInterlock, FailedUnlocalizedRuntimeAllowsExplicitRetryButNotMotion)
{
  FloorRuntimeInterlock interlock;
  interlock.observe_floor_switch_status(
    "failed", "FAILED_LOCKED", "HOLD_AND_LOCK", 99U, "target map unlocalized");
  interlock.observe_localization_health(false, false, "FAILED_LOCKED: target unlocalized");
  EXPECT_FALSE(interlock.decision_for_operation("live_floor_switch_start").blocked);
  EXPECT_TRUE(interlock.decision_for_operation("navigation_goal").blocked);
  EXPECT_EQ(interlock.decision_for_operation("navigation_goal").code,
    "FLOOR_RUNTIME_CONTEXT_INVALID");
}

TEST(FloorRuntimeInterlock, FailedHealthHistoryDoesNotOverrideCurrentTypedEvidence)
{
  FloorRuntimeInterlock interlock;
  interlock.observe_localization_health(false, false, "FAILED_LOCKED: failure");
  interlock.observe_localization_health(false, true, "current runtime ready");
  EXPECT_FALSE(interlock.decision_for_operation("navigation_goal").blocked);
}

TEST(FloorRuntimeInterlock, CurrentActivityWinsOverFailureHistoryAndRetry)
{
  for (const bool active_in_health : {false, true}) {
    FloorRuntimeInterlock interlock;
    interlock.observe_floor_switch_status(
      "failed", "FAILED_LOCKED", "HOLD_AND_LOCK", 99U, "failure");
    interlock.observe_localization_health(false, true, "current runtime ready");
    if (active_in_health) {
      interlock.observe_localization_health(true, false, "FAILED_LOCKED: old detail");
    } else {
      interlock.observe_floor_switch_status(
        "new", "LOADING_NAV_MAP", "LOAD_NAV_MAP", 0U, "loading");
    }
    for (const auto * operation : {"navigation_goal", "live_floor_switch_start",
        "elevator_config_publish"}) {
      SCOPED_TRACE(operation);
      EXPECT_TRUE(interlock.decision_for_operation(operation).blocked);
      EXPECT_EQ(interlock.decision_for_operation(operation).code, "FLOOR_TRANSITION_ACTIVE");
    }
  }
}

TEST(FloorRuntimeInterlock, FailedFloorDoesNotLockOfflineAssetEditing)
{
  FloorRuntimeInterlock interlock;
  interlock.observe_floor_switch_status(
    "elevator-test-failed", "FAILED_LOCKED", "RECOVERY_LOCKED", 99U,
    "live filter-mask reload requires active keepout and speed mask servers");
  interlock.observe_localization_health(false, false, "target runtime unlocalized");
  for (const auto * operation : kOfflineAssetOperations) {
    SCOPED_TRACE(operation);
    const auto decision = interlock.decision_for_operation(operation);
    EXPECT_FALSE(decision.blocked);
    EXPECT_EQ(decision.code, "FLOOR_RUNTIME_OFFLINE_ASSET_EDIT_ALLOWED");
  }
  EXPECT_TRUE(interlock.decision().blocked);
  EXPECT_EQ(interlock.decision().code, "FLOOR_RUNTIME_CONTEXT_INVALID");
}

TEST(FloorRuntimeInterlock, FailedBridgeDoesNotLockOfflineAssetEditing)
{
  FloorRuntimeInterlock interlock;
  interlock.observe_localization_health(false, false, "FAILED_LOCKED: target unknown");
  for (const auto * operation : kOfflineAssetOperations) {
    SCOPED_TRACE(operation);
    EXPECT_FALSE(interlock.decision_for_operation(operation).blocked);
  }
  EXPECT_TRUE(interlock.decision().blocked);
}

TEST(FloorRuntimeInterlock, SourceIndependentAdmissionDoesNotAllowMotionOrWildcardNames)
{
  FloorRuntimeInterlock interlock;
  interlock.observe_floor_switch_status("tx", "FAILED_LOCKED", "HOLD", 99U, "failed");
  interlock.observe_localization_health(false, false, "ABORTED_CONTEXT_INVALID");
  for (const auto * operation : {
      "navigation_goal", "navigation_goal_worker", "navigation_goal_submit", "navigation_pre_send",
      "safety_resume", "docking_start", "docking_start_commit",
      "docking_undock", "docking_undock_commit", "docking_undock_submit",
      "keepout_update", "keepout_update_commit", "map_delete", "map_delete_commit",
      "current_pose_save", "current_pose_save_commit", "elevator_test_start",
      "elevator_config_publish_and_start", "pose_save_current", "future_save", ""})
  {
    SCOPED_TRACE(operation);
    EXPECT_TRUE(interlock.decision_for_operation(operation).blocked);
  }
}

TEST(FloorRuntimeInterlock, ActiveTransitionStillSerializesOfflineAssetMutations)
{
  FloorRuntimeInterlock interlock;
  interlock.observe_floor_switch_status("tx", "LOADING_FILTERS", "LOAD_FILTERS", 0U, "loading");
  for (const auto * operation : kOfflineAssetOperations) {
    SCOPED_TRACE(operation);
    EXPECT_TRUE(interlock.decision_for_operation(operation).blocked);
  }
}

TEST(FloorRuntimeInterlock, OperationPolicyPreservesSourceIndependentMapSwitch)
{
  FloorRuntimeInterlock interlock;
  interlock.observe_localization_health(false, false, "source unlocalized");
  EXPECT_FALSE(interlock.decision_for_operation("live_floor_switch_start").blocked);
  EXPECT_TRUE(interlock.decision_for_operation("navigation_goal").blocked);
}

TEST(FloorRuntimeInterlock, HistoricalFailureDoesNotHideNewActiveEvidence)
{
  for (const bool failed_in_health : {false, true}) {
    for (const bool active_in_health : {false, true}) {
      SCOPED_TRACE(failed_in_health);
      SCOPED_TRACE(active_in_health);
      FloorRuntimeInterlock interlock;
      if (failed_in_health) {
        interlock.observe_localization_health(false, false, "FAILED_LOCKED: old failure");
      } else {
        interlock.observe_floor_switch_status("old", "FAILED_LOCKED", "HOLD", 99U, "failed");
      }
      if (active_in_health) {
        interlock.observe_localization_health(true, false, "new transition active");
      } else {
        interlock.observe_floor_switch_status("new", "LOADING_FILTERS", "LOAD_FILTERS", 0U, "loading");
      }
      for (const auto * operation : kOfflineAssetOperations) {
        SCOPED_TRACE(operation);
        EXPECT_TRUE(interlock.decision_for_operation(operation).blocked);
      }
      if (active_in_health) {
        interlock.observe_localization_health(false, true, "new transition ended");
      } else {
        interlock.observe_floor_switch_status("new", "COMPLETE", "COMPLETE", 0U, "complete");
      }
      EXPECT_FALSE(interlock.decision_for_operation("elevator_config_publish").blocked);
      EXPECT_EQ(interlock.decision_for_operation("navigation_goal").blocked,
        failed_in_health && !active_in_health);
    }
  }
}

TEST(FloorRuntimeInterlock, InvalidSourceDoesNotBlockMapSwitchOrHideConcurrentTransition)
{
  FloorRuntimeInterlock interlock;
  interlock.observe_localization_health(false, false, "initial localization failed");
  EXPECT_TRUE(interlock.decision().blocked);
  EXPECT_FALSE(interlock.decision_for_map_switch().blocked);
  interlock.observe_floor_switch_status("tx-active", "LOADING_NAV_MAP", "LOAD_NAV_MAP", 0U, "loading");
  EXPECT_TRUE(interlock.decision_for_map_switch().blocked);
  EXPECT_EQ(interlock.decision_for_map_switch().code, "FLOOR_TRANSITION_ACTIVE");
}

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

TEST(FloorRuntimeInterlock, CurrentHealthyBridgeIsNotOverriddenByTerminalFailure)
{
  FloorRuntimeInterlock interlock;

  interlock.observe_floor_switch_status(
    "tx-failed", "FAILED_LOCKED", "HOLD_AND_LOCK", 99U, "manual recovery required");
  interlock.observe_localization_health(false, true, "current healthy sample");

  const auto decision = interlock.decision();
  EXPECT_FALSE(decision.blocked);
  EXPECT_EQ(decision.code, "FLOOR_RUNTIME_NO_NEGATIVE_EVIDENCE");
}

TEST(FloorRuntimeInterlock, FailureTextDoesNotOverrideCurrentTypedHealth)
{
  FloorRuntimeInterlock interlock;

  interlock.observe_localization_health(false, true, "FAILED_LOCKED: recovery required");

  const auto decision = interlock.decision();
  EXPECT_FALSE(decision.blocked);
}

TEST(FloorRuntimeInterlock, LaterPreflightDoesNotInventValidLocalization)
{
  FloorRuntimeInterlock interlock;

  interlock.observe_floor_switch_status(
    "tx-failed", "FAILED_LOCKED", "HOLD_AND_LOCK", 99U, "manual recovery required");
  interlock.observe_localization_health(false, false, "current runtime unlocalized");
  interlock.observe_floor_switch_status(
    "tx-new", "BLOCKED", "LIVE_FLOOR_SWITCH_DISABLED", 41U,
    "a later preflight changed no runtime asset");

  const auto decision = interlock.decision();
  EXPECT_TRUE(decision.blocked);
  EXPECT_EQ(decision.code, "FLOOR_RUNTIME_CONTEXT_INVALID");
}

TEST(FloorRuntimeInterlock, RecoveredHealthCanBecomeInvalidAgain)
{
  FloorRuntimeInterlock interlock;

  interlock.observe_localization_health(false, false, "FAILED_LOCKED: recovery required");
  interlock.observe_localization_health(false, true, "current healthy heartbeat");
  EXPECT_FALSE(interlock.decision().blocked);
  interlock.observe_localization_health(false, false, "new localization loss");

  const auto decision = interlock.decision();
  EXPECT_TRUE(decision.blocked);
  EXPECT_EQ(decision.code, "FLOOR_RUNTIME_CONTEXT_INVALID");
}

TEST(FloorRuntimeInterlock, UnknownStatusDoesNotInventAReadinessFailure)
{
  FloorRuntimeInterlock interlock;

  interlock.observe_floor_switch_status(
    "tx-future", "FUTURE_DIAGNOSTIC_ONLY", "REPORT", 0U, "new non-mutating state");

  EXPECT_FALSE(interlock.decision().blocked);
}

}  // namespace
