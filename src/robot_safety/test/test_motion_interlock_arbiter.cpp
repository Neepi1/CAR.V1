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
  HoldCommand floor_hold{HoldOperation::kAcquire, "floor", "tx-2", "switch"};

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

}  // namespace robot_safety
