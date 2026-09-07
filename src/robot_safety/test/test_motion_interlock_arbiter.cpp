#include <algorithm>

#include <gtest/gtest.h>

#include "robot_safety/motion_interlock_arbiter.hpp"

namespace robot_safety
{

ExecutionCommand execution_command(
  const std::string & lease_id = "lease-1",
  const std::string & owner = "robot_elevator_manager",
  const bool recovery = false)
{
  ExecutionCommand command;
  command.owner = owner;
  command.mission_id = "mission-1";
  command.transaction_id = "elevator-transaction-1";
  command.lease_id = lease_id;
  command.lease_duration_sec = 1.0;
  command.recovery = recovery;
  return command;
}

TEST(MotionInterlockArbiter, StartupDoesNotChangeExistingSafetyPolicy)
{
  MotionInterlockArbiter arbiter;
  const auto state = arbiter.snapshot(10.0);

  EXPECT_FALSE(state.motion_blocked);
  EXPECT_TRUE(arbiter.motion_permitted(10.0));
  EXPECT_TRUE(state.hold_keys.empty());
  EXPECT_FALSE(state.execution_session_engaged);
  EXPECT_EQ(state.generation, 0U);
}

TEST(MotionInterlockArbiter, HoldsAreOwnerScopedAndCompose)
{
  MotionInterlockArbiter arbiter;
  HoldCommand elevator_hold{HoldOperation::kAcquire, "elevator", "tx-1", "settle"};
  HoldCommand floor_hold{HoldOperation::kAcquire, "floor", "tx-1", "switch"};

  EXPECT_TRUE(arbiter.apply_hold(elevator_hold, 10.0).accepted);
  const auto second = arbiter.apply_hold(floor_hold, 10.1);
  EXPECT_TRUE(second.accepted);
  EXPECT_EQ(second.state.hold_keys.size(), 2U);
  EXPECT_TRUE(second.state.motion_blocked);

  auto wrong_release = elevator_hold;
  wrong_release.operation = HoldOperation::kRelease;
  wrong_release.transaction_id = "tx-other";
  const auto rejected = arbiter.apply_hold(wrong_release, 10.2);
  EXPECT_FALSE(rejected.accepted);
  EXPECT_EQ(rejected.code, InterlockDecisionCode::kNotOwner);
  EXPECT_EQ(rejected.state.hold_keys.size(), 2U);

  elevator_hold.operation = HoldOperation::kRelease;
  EXPECT_TRUE(arbiter.apply_hold(elevator_hold, 10.3).accepted);
  EXPECT_TRUE(arbiter.snapshot(10.3).motion_blocked);
  floor_hold.operation = HoldOperation::kRelease;
  EXPECT_TRUE(arbiter.apply_hold(floor_hold, 10.4).accepted);
  EXPECT_TRUE(arbiter.motion_permitted(10.4));
}

TEST(MotionInterlockArbiter, ReleaseBeforeDelayedAcquireFencesSequencedHold)
{
  MotionInterlockArbiter arbiter;
  HoldCommand release{
    HoldOperation::kRelease, "robot_floor_manager", "tx-late", "cleanup"};
  release.command_sequence = 12U;
  const auto fenced = arbiter.apply_hold(release, 10.0);
  ASSERT_TRUE(fenced.accepted);
  EXPECT_EQ(fenced.applied_sequence, 12U);
  EXPECT_TRUE(fenced.state.hold_keys.empty());

  auto delayed_acquire = release;
  delayed_acquire.operation = HoldOperation::kAcquire;
  delayed_acquire.reason = "late acquire";
  delayed_acquire.command_sequence = 11U;
  const auto stale = arbiter.apply_hold(delayed_acquire, 10.1);
  EXPECT_FALSE(stale.accepted);
  EXPECT_EQ(stale.code, InterlockDecisionCode::kStaleCommand);
  EXPECT_EQ(stale.applied_sequence, 12U);
  EXPECT_TRUE(stale.state.hold_keys.empty());
}

TEST(MotionInterlockArbiter, LateSequencedReleaseCannotOverrideNewerAcquire)
{
  MotionInterlockArbiter arbiter;
  HoldCommand acquire{
    HoldOperation::kAcquire, "robot_elevator_manager", "tx-sequenced", "preflight"};
  acquire.command_sequence = 11U;
  const auto acquired = arbiter.apply_hold(acquire, 10.0);
  ASSERT_TRUE(acquired.accepted);
  EXPECT_EQ(acquired.applied_sequence, 11U);
  EXPECT_TRUE(acquired.state.motion_blocked);

  auto late_release = acquire;
  late_release.operation = HoldOperation::kRelease;
  late_release.command_sequence = 10U;
  const auto stale = arbiter.apply_hold(late_release, 10.1);

  EXPECT_FALSE(stale.accepted);
  EXPECT_EQ(stale.code, InterlockDecisionCode::kStaleCommand);
  EXPECT_EQ(stale.applied_sequence, 11U);
  EXPECT_TRUE(stale.state.motion_blocked);
  ASSERT_EQ(stale.state.hold_keys.size(), 1U);

  auto current_release = late_release;
  current_release.command_sequence = 12U;
  const auto released = arbiter.apply_hold(current_release, 10.2);
  EXPECT_TRUE(released.accepted);
  EXPECT_EQ(released.applied_sequence, 12U);
  EXPECT_FALSE(released.state.motion_blocked);
}

TEST(MotionInterlockArbiter, SequencedKeyRejectsLegacyCommandsWithoutMutatingHold)
{
  MotionInterlockArbiter arbiter;
  HoldCommand acquire{
    HoldOperation::kAcquire, "robot_elevator_manager", "tx-sequenced", "preflight"};
  acquire.command_sequence = 3U;
  ASSERT_TRUE(arbiter.apply_hold(acquire, 10.0).accepted);

  auto legacy_release = acquire;
  legacy_release.operation = HoldOperation::kRelease;
  legacy_release.command_sequence = 0U;
  const auto rejected = arbiter.apply_hold(legacy_release, 10.1);

  EXPECT_FALSE(rejected.accepted);
  EXPECT_EQ(rejected.code, InterlockDecisionCode::kStaleCommand);
  EXPECT_EQ(rejected.applied_sequence, 3U);
  EXPECT_TRUE(rejected.state.motion_blocked);
}

TEST(MotionInterlockArbiter, LegacyKeyRemainsCompatibleUntilSequencingIsObserved)
{
  MotionInterlockArbiter arbiter;
  HoldCommand legacy{
    HoldOperation::kAcquire, "legacy_owner", "legacy-tx", "legacy integration"};
  const auto acquired = arbiter.apply_hold(legacy, 10.0);
  ASSERT_TRUE(acquired.accepted);
  EXPECT_EQ(acquired.applied_sequence, 0U);

  legacy.operation = HoldOperation::kRelease;
  const auto released = arbiter.apply_hold(legacy, 10.1);
  EXPECT_TRUE(released.accepted);
  EXPECT_EQ(released.applied_sequence, 0U);
  EXPECT_FALSE(released.state.motion_blocked);
}

TEST(MotionInterlockArbiter, ObservingInvalidSequencedCommandDisablesLegacyForKey)
{
  MotionInterlockArbiter arbiter;
  HoldCommand invalid{
    HoldOperation::kAcquire, "sequenced_owner", "sequenced-tx", ""};
  invalid.command_sequence = 7U;
  const auto rejected_invalid = arbiter.apply_hold(invalid, 10.0);
  ASSERT_FALSE(rejected_invalid.accepted);
  EXPECT_EQ(rejected_invalid.applied_sequence, 0U);

  HoldCommand legacy{
    HoldOperation::kAcquire, "sequenced_owner", "sequenced-tx", "legacy retry"};
  const auto rejected_legacy = arbiter.apply_hold(legacy, 10.1);
  EXPECT_FALSE(rejected_legacy.accepted);
  EXPECT_EQ(rejected_legacy.code, InterlockDecisionCode::kStaleCommand);
  EXPECT_EQ(rejected_legacy.applied_sequence, 0U);

  invalid.reason = "sequenced retry";
  invalid.command_sequence = 8U;
  const auto accepted = arbiter.apply_hold(invalid, 10.2);
  EXPECT_TRUE(accepted.accepted);
  EXPECT_EQ(accepted.applied_sequence, 8U);
}

TEST(MotionInterlockArbiter, HoldDominatesAHealthyExecutionLease)
{
  MotionInterlockArbiter arbiter;
  ASSERT_TRUE(arbiter.apply_execution(execution_command(), 10.0).accepted);
  EXPECT_TRUE(arbiter.motion_permitted(10.5));

  const auto held = arbiter.apply_hold(
    HoldCommand{HoldOperation::kAcquire, "elevator", "tx-1", "goal boundary"}, 10.5);
  EXPECT_TRUE(held.state.execution_lease_active);
  EXPECT_TRUE(held.state.motion_blocked);
  EXPECT_FALSE(arbiter.motion_permitted(10.5));
}

TEST(MotionInterlockArbiter, ExecutionExpiryStaysFailureLocked)
{
  MotionInterlockArbiter arbiter;
  ASSERT_TRUE(arbiter.apply_execution(execution_command(), 10.0).accepted);

  EXPECT_TRUE(arbiter.motion_permitted(10.999));
  EXPECT_FALSE(arbiter.motion_permitted(11.0));
  const auto expired = arbiter.expire(11.0);
  ASSERT_TRUE(expired.has_value());
  EXPECT_TRUE(expired->execution_session_engaged);
  EXPECT_FALSE(expired->execution_lease_active);
  EXPECT_TRUE(expired->motion_blocked);
  EXPECT_EQ(expired->transition_reason, "execution_lease_expired");
  EXPECT_FALSE(arbiter.expire(12.0).has_value());

  const auto stale = arbiter.apply_execution(execution_command(), 12.0);
  EXPECT_FALSE(stale.accepted);
  EXPECT_EQ(stale.code, InterlockDecisionCode::kStaleLease);
  EXPECT_TRUE(stale.state.motion_blocked);
}

TEST(MotionInterlockArbiter, ExactLeaseRenewsAndReleasesSession)
{
  MotionInterlockArbiter arbiter;
  ASSERT_TRUE(arbiter.apply_execution(execution_command(), 10.0).accepted);

  const auto renewed = arbiter.apply_execution(execution_command(), 10.5);
  EXPECT_TRUE(renewed.accepted);
  EXPECT_FALSE(renewed.changed);
  EXPECT_DOUBLE_EQ(renewed.state.execution_lease_remaining_sec, 1.0);

  auto release = execution_command();
  release.operation = ExecutionOperation::kRelease;
  const auto released = arbiter.apply_execution(release, 10.6);
  EXPECT_TRUE(released.accepted);
  EXPECT_TRUE(released.changed);
  EXPECT_FALSE(released.state.execution_session_engaged);
  EXPECT_FALSE(released.state.motion_blocked);

  const auto stale = arbiter.apply_execution(execution_command(), 10.7);
  EXPECT_FALSE(stale.accepted);
  EXPECT_EQ(stale.code, InterlockDecisionCode::kStaleLease);
}

TEST(MotionInterlockArbiter, ReleaseBeforeDelayedSetFencesUniqueLease)
{
  MotionInterlockArbiter arbiter;
  auto release = execution_command("late-lease");
  release.operation = ExecutionOperation::kRelease;

  const auto fenced = arbiter.apply_execution(release, 10.0);
  ASSERT_TRUE(fenced.accepted);
  EXPECT_FALSE(fenced.state.execution_session_engaged);

  const auto delayed_set =
    arbiter.apply_execution(execution_command("late-lease"), 10.1);
  EXPECT_FALSE(delayed_set.accepted);
  EXPECT_EQ(delayed_set.code, InterlockDecisionCode::kStaleLease);
  EXPECT_FALSE(delayed_set.state.execution_session_engaged);
}

TEST(MotionInterlockArbiter, ExactReleaseClosesExpiredFailureLockedSession)
{
  MotionInterlockArbiter arbiter;
  ASSERT_TRUE(arbiter.apply_execution(execution_command(), 10.0).accepted);
  ASSERT_TRUE(arbiter.expire(11.0).has_value());

  auto release = execution_command();
  release.operation = ExecutionOperation::kRelease;
  const auto closed = arbiter.apply_execution(release, 11.1);
  EXPECT_TRUE(closed.accepted);
  EXPECT_FALSE(closed.state.execution_session_engaged);
  EXPECT_FALSE(closed.state.execution_lease_active);
  EXPECT_FALSE(closed.state.motion_blocked);
}

TEST(MotionInterlockArbiter, DifferentExecutionLeaseCannotTakeOver)
{
  MotionInterlockArbiter arbiter;
  ASSERT_TRUE(arbiter.apply_execution(execution_command(), 10.0).accepted);

  const auto conflict = arbiter.apply_execution(execution_command("lease-2"), 10.1);
  EXPECT_FALSE(conflict.accepted);
  EXPECT_EQ(conflict.code, InterlockDecisionCode::kConflict);
  EXPECT_EQ(conflict.state.execution_lease_id, "lease-1");
}

TEST(MotionInterlockArbiter, RecoveryOwnerCanRecoverAnExpiredSession)
{
  MotionInterlockArbiter arbiter;
  ASSERT_TRUE(arbiter.apply_execution(execution_command(), 10.0).accepted);
  ASSERT_TRUE(arbiter.expire(11.0).has_value());

  const auto ordinary = arbiter.apply_execution(execution_command("lease-2"), 11.1);
  EXPECT_FALSE(ordinary.accepted);
  EXPECT_EQ(ordinary.code, InterlockDecisionCode::kConflict);

  auto recovery = execution_command("recovery-lease", "robot_mission_manager", true);
  const auto recovered = arbiter.apply_execution(recovery, 11.2);
  EXPECT_TRUE(recovered.accepted);
  EXPECT_TRUE(recovered.state.execution_lease_active);
  EXPECT_TRUE(arbiter.motion_permitted(11.2));

  recovery.operation = ExecutionOperation::kRelease;
  const auto closed = arbiter.apply_execution(recovery, 11.3);
  EXPECT_TRUE(closed.accepted);
  EXPECT_FALSE(closed.state.execution_session_engaged);
}

TEST(MotionInterlockArbiter, InvalidCommandsNeverMutateGeneration)
{
  MotionInterlockArbiter arbiter;
  auto invalid_execution = execution_command();
  invalid_execution.lease_duration_sec = 10.0;
  const auto rejected_execution = arbiter.apply_execution(invalid_execution, 10.0);
  EXPECT_FALSE(rejected_execution.accepted);
  EXPECT_EQ(rejected_execution.code, InterlockDecisionCode::kInvalidRequest);
  EXPECT_EQ(rejected_execution.state.generation, 0U);

  const auto rejected_hold = arbiter.apply_hold(
    HoldCommand{HoldOperation::kAcquire, "", "tx", "reason"}, 10.0);
  EXPECT_FALSE(rejected_hold.accepted);
  EXPECT_EQ(rejected_hold.state.generation, 0U);
}

TEST(MotionInterlockArbiter, RetiredExecutionLeaseNeverBecomesReusable)
{
  MotionInterlockArbiter arbiter;
  for (int index = 0; index < 80; ++index) {
    auto command = execution_command("lease-" + std::to_string(index));
    ASSERT_TRUE(arbiter.apply_execution(command, 10.0 + index).accepted);
    command.operation = ExecutionOperation::kRelease;
    ASSERT_TRUE(arbiter.apply_execution(command, 10.1 + index).accepted);
  }

  const auto replay = arbiter.apply_execution(execution_command("lease-0"), 100.0);

  EXPECT_FALSE(replay.accepted);
  EXPECT_EQ(replay.code, InterlockDecisionCode::kStaleLease);
}

TEST(MotionInterlockArbiter, ConditionalRecoveryReleaseIsAtomic)
{
  MotionInterlockArbiter arbiter;
  HoldCommand acquire{
    HoldOperation::kAcquire,
    "robot_elevator_manager",
    "elevator-test-1",
    "restart_locked_recovery"};
  acquire.command_sequence = 10U;
  const auto acquired = arbiter.apply_hold(acquire, 10.0);
  ASSERT_TRUE(acquired.accepted);
  ASSERT_EQ(acquired.state.generation, 1U);

  ConditionalHoldReleaseCommand release;
  release.owner = acquire.owner;
  release.transaction_id = acquire.transaction_id;
  release.reason = "explicit_preflight_orphan_recovery";
  release.command_sequence = 11U;
  release.expected_generation = acquired.state.generation;
  const auto released =
    arbiter.release_hold_if_execution_idle(release, 10.1);

  EXPECT_TRUE(released.accepted);
  EXPECT_TRUE(released.changed);
  EXPECT_EQ(released.code, InterlockDecisionCode::kOk);
  EXPECT_EQ(released.applied_sequence, 11U);
  EXPECT_EQ(released.state.generation, 2U);
  EXPECT_FALSE(released.state.execution_session_engaged);
  EXPECT_FALSE(released.state.execution_lease_active);
  EXPECT_TRUE(released.state.hold_keys.empty());
}

TEST(MotionInterlockArbiter, ConditionalRecoveryReleaseRejectsGenerationDrift)
{
  MotionInterlockArbiter arbiter;
  HoldCommand acquire{
    HoldOperation::kAcquire,
    "robot_elevator_manager",
    "elevator-test-1",
    "restart_locked_recovery"};
  acquire.command_sequence = 10U;
  const auto acquired = arbiter.apply_hold(acquire, 10.0);
  ASSERT_TRUE(acquired.accepted);
  ASSERT_TRUE(
    arbiter.apply_hold(
      HoldCommand{
        HoldOperation::kAcquire, "another_owner", "other-tx", "other hold"},
      10.1).accepted);

  ConditionalHoldReleaseCommand release;
  release.owner = acquire.owner;
  release.transaction_id = acquire.transaction_id;
  release.reason = "explicit_preflight_orphan_recovery";
  release.command_sequence = 11U;
  release.expected_generation = acquired.state.generation;
  const auto rejected =
    arbiter.release_hold_if_execution_idle(release, 10.2);

  EXPECT_FALSE(rejected.accepted);
  EXPECT_EQ(rejected.code, InterlockDecisionCode::kGenerationMismatch);
  EXPECT_EQ(rejected.state.generation, 2U);
  EXPECT_NE(
    std::find(
      rejected.state.hold_keys.cbegin(),
      rejected.state.hold_keys.cend(),
      "robot_elevator_manager:elevator-test-1"),
    rejected.state.hold_keys.cend());
}

TEST(MotionInterlockArbiter, ConditionalRecoveryReleaseRejectsExecutionSessionOrLease)
{
  MotionInterlockArbiter arbiter;
  HoldCommand acquire{
    HoldOperation::kAcquire,
    "robot_elevator_manager",
    "elevator-test-1",
    "restart_locked_recovery"};
  acquire.command_sequence = 10U;
  ASSERT_TRUE(arbiter.apply_hold(acquire, 10.0).accepted);
  const auto execution =
    arbiter.apply_execution(execution_command(), 10.1);
  ASSERT_TRUE(execution.accepted);
  ASSERT_TRUE(execution.state.execution_session_engaged);
  ASSERT_TRUE(execution.state.execution_lease_active);

  ConditionalHoldReleaseCommand release;
  release.owner = acquire.owner;
  release.transaction_id = acquire.transaction_id;
  release.reason = "explicit_preflight_orphan_recovery";
  release.command_sequence = 11U;
  release.expected_generation = execution.state.generation;
  const auto live_lease =
    arbiter.release_hold_if_execution_idle(release, 10.2);
  EXPECT_FALSE(live_lease.accepted);
  EXPECT_EQ(live_lease.code, InterlockDecisionCode::kExecutionActive);
  EXPECT_TRUE(live_lease.state.execution_session_engaged);
  EXPECT_TRUE(live_lease.state.execution_lease_active);

  const auto expired = arbiter.expire(11.2);
  ASSERT_TRUE(expired.has_value());
  ASSERT_TRUE(expired->execution_session_engaged);
  ASSERT_FALSE(expired->execution_lease_active);
  release.command_sequence = 12U;
  release.expected_generation = expired->generation;
  const auto failure_latched_session =
    arbiter.release_hold_if_execution_idle(release, 11.3);
  EXPECT_FALSE(failure_latched_session.accepted);
  EXPECT_EQ(
    failure_latched_session.code,
    InterlockDecisionCode::kExecutionActive);
  EXPECT_TRUE(failure_latched_session.state.execution_session_engaged);
  EXPECT_FALSE(failure_latched_session.state.execution_lease_active);
}

TEST(MotionInterlockArbiter, ConditionalRecoveryReleaseRejectsWrongOwnerAndStaleSequence)
{
  MotionInterlockArbiter arbiter;
  HoldCommand acquire{
    HoldOperation::kAcquire,
    "robot_elevator_manager",
    "elevator-test-1",
    "restart_locked_recovery"};
  acquire.command_sequence = 20U;
  const auto acquired = arbiter.apply_hold(acquire, 10.0);
  ASSERT_TRUE(acquired.accepted);

  ConditionalHoldReleaseCommand wrong_owner;
  wrong_owner.owner = "other_owner";
  wrong_owner.transaction_id = acquire.transaction_id;
  wrong_owner.reason = "invalid recovery owner";
  wrong_owner.command_sequence = 21U;
  wrong_owner.expected_generation = acquired.state.generation;
  const auto owner_rejected =
    arbiter.release_hold_if_execution_idle(wrong_owner, 10.1);
  EXPECT_FALSE(owner_rejected.accepted);
  EXPECT_EQ(owner_rejected.code, InterlockDecisionCode::kNotOwner);
  EXPECT_EQ(owner_rejected.state.generation, acquired.state.generation);

  ConditionalHoldReleaseCommand stale;
  stale.owner = acquire.owner;
  stale.transaction_id = acquire.transaction_id;
  stale.reason = "stale recovery";
  stale.command_sequence = 20U;
  stale.expected_generation = acquired.state.generation;
  const auto stale_rejected =
    arbiter.release_hold_if_execution_idle(stale, 10.2);
  EXPECT_FALSE(stale_rejected.accepted);
  EXPECT_EQ(stale_rejected.code, InterlockDecisionCode::kStaleCommand);
  EXPECT_EQ(stale_rejected.applied_sequence, 20U);
  EXPECT_EQ(stale_rejected.state.generation, acquired.state.generation);
}

}  // namespace robot_safety
