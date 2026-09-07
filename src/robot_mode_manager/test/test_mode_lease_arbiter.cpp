#include <gtest/gtest.h>

#include "robot_mode_manager/mode_lease_arbiter.hpp"

namespace robot_mode_manager
{

ModeCommand make_command(
  const std::string & mode = "ELEVATOR_WAIT",
  const std::string & owner = "robot_elevator_manager",
  const std::string & mission_id = "mission-42",
  const std::string & lease_id = "lease-42",
  const double duration_sec = 5.0)
{
  ModeCommand command;
  command.mode = mode;
  command.owner = owner;
  command.mission_id = mission_id;
  command.lease_id = lease_id;
  command.lease_duration_sec = duration_sec;
  return command;
}

TEST(ModeLeaseArbiter, StartsInUnownedNormalMode)
{
  ModeLeaseArbiter arbiter;

  const auto state = arbiter.snapshot(10.0);

  EXPECT_EQ(state.mode, OperatingMode::kNormal);
  EXPECT_FALSE(state.lease_active);
  EXPECT_TRUE(state.owner.empty());
  EXPECT_TRUE(state.mission_id.empty());
  EXPECT_TRUE(state.lease_id.empty());
  EXPECT_DOUBLE_EQ(state.lease_remaining_sec, 0.0);
  EXPECT_EQ(state.generation, 0U);
}

TEST(ModeLeaseArbiter, AcquiresBoundedElevatorWaitLease)
{
  ModeLeaseArbiter arbiter;
  const auto command = make_command();

  const auto decision = arbiter.apply(command, 10.0);

  ASSERT_TRUE(decision.accepted);
  EXPECT_TRUE(decision.changed);
  EXPECT_EQ(decision.code, ModeDecisionCode::kOk);
  EXPECT_EQ(decision.state.mode, OperatingMode::kElevatorWait);
  EXPECT_EQ(decision.state.owner, command.owner);
  EXPECT_EQ(decision.state.mission_id, command.mission_id);
  EXPECT_EQ(decision.state.lease_id, command.lease_id);
  EXPECT_TRUE(decision.state.lease_active);
  EXPECT_DOUBLE_EQ(decision.state.lease_remaining_sec, 5.0);
  EXPECT_EQ(decision.state.generation, 1U);
}

TEST(ModeLeaseArbiter, RejectsInvalidRequestWithoutMutation)
{
  ModeLeaseArbiter arbiter;
  auto command = make_command();
  command.owner.clear();

  const auto decision = arbiter.apply(command, 10.0);

  EXPECT_FALSE(decision.accepted);
  EXPECT_EQ(decision.code, ModeDecisionCode::kInvalidRequest);
  EXPECT_EQ(decision.state.mode, OperatingMode::kNormal);
  EXPECT_EQ(decision.state.generation, 0U);

  command = make_command("elevator_wait");
  const auto unsupported = arbiter.apply(command, 11.0);
  EXPECT_FALSE(unsupported.accepted);
  EXPECT_EQ(unsupported.code, ModeDecisionCode::kUnsupportedMode);
  EXPECT_EQ(unsupported.state.generation, 0U);

  command = make_command("DOORWAY", "robot_elevator_manager", "mission-42", "lease-42", 30.1);
  const auto oversized = arbiter.apply(command, 12.0);
  EXPECT_FALSE(oversized.accepted);
  EXPECT_EQ(oversized.code, ModeDecisionCode::kInvalidRequest);
  EXPECT_EQ(oversized.state.generation, 0U);
}

TEST(ModeLeaseArbiter, RejectsConflictingOwnerAndPreservesLease)
{
  ModeLeaseArbiter arbiter;
  ASSERT_TRUE(arbiter.apply(make_command(), 10.0).accepted);

  const auto decision = arbiter.apply(
    make_command("DOORWAY", "another_owner", "mission-99", "lease-99"), 11.0);

  EXPECT_FALSE(decision.accepted);
  EXPECT_EQ(decision.code, ModeDecisionCode::kLeaseConflict);
  EXPECT_EQ(decision.state.mode, OperatingMode::kElevatorWait);
  EXPECT_EQ(decision.state.owner, "robot_elevator_manager");
  EXPECT_EQ(decision.state.generation, 1U);
}

TEST(ModeLeaseArbiter, SameLeaseCanRenewAndChangeMode)
{
  ModeLeaseArbiter arbiter;
  ASSERT_TRUE(arbiter.apply(make_command(), 10.0).accepted);

  const auto renewed = arbiter.apply(make_command(), 11.0);
  EXPECT_TRUE(renewed.accepted);
  EXPECT_FALSE(renewed.changed);
  EXPECT_EQ(renewed.state.generation, 1U);
  EXPECT_DOUBLE_EQ(renewed.state.lease_remaining_sec, 5.0);

  const auto changed = arbiter.apply(make_command("DOORWAY"), 12.0);
  EXPECT_TRUE(changed.accepted);
  EXPECT_TRUE(changed.changed);
  EXPECT_EQ(changed.state.mode, OperatingMode::kDoorway);
  EXPECT_EQ(changed.state.generation, 2U);
}

TEST(ModeLeaseArbiter, OnlyExactLeaseCanRelease)
{
  ModeLeaseArbiter arbiter;
  ASSERT_TRUE(arbiter.apply(make_command(), 10.0).accepted);

  auto release = make_command();
  release.operation = ModeOperation::kRelease;
  release.lease_id = "wrong-lease";
  const auto rejected = arbiter.apply(release, 11.0);
  EXPECT_FALSE(rejected.accepted);
  EXPECT_EQ(rejected.code, ModeDecisionCode::kNotOwner);
  EXPECT_EQ(rejected.state.mode, OperatingMode::kElevatorWait);

  release.lease_id = "lease-42";
  const auto accepted = arbiter.apply(release, 12.0);
  EXPECT_TRUE(accepted.accepted);
  EXPECT_TRUE(accepted.changed);
  EXPECT_EQ(accepted.state.mode, OperatingMode::kNormal);
  EXPECT_FALSE(accepted.state.lease_active);
  EXPECT_TRUE(accepted.state.owner.empty());
  EXPECT_EQ(accepted.state.generation, 2U);
  EXPECT_EQ(accepted.state.transition_reason, "lease_released");
}

TEST(ModeLeaseArbiter, ExpiryTransitionsOnceAndRetiresLease)
{
  ModeLeaseArbiter arbiter;
  ASSERT_TRUE(arbiter.apply(make_command(), 10.0).accepted);

  EXPECT_FALSE(arbiter.expire(14.999).has_value());
  const auto expired = arbiter.expire(15.0);
  ASSERT_TRUE(expired.has_value());
  EXPECT_EQ(expired->mode, OperatingMode::kNormal);
  EXPECT_EQ(expired->generation, 2U);
  EXPECT_EQ(expired->transition_reason, "lease_expired");
  EXPECT_FALSE(arbiter.expire(16.0).has_value());

  const auto stale = arbiter.apply(make_command(), 17.0);
  EXPECT_FALSE(stale.accepted);
  EXPECT_EQ(stale.code, ModeDecisionCode::kStaleLease);
  EXPECT_EQ(stale.state.generation, 2U);
}

TEST(ModeLeaseArbiter, ReleaseBeforeDelayedSetFencesUniqueLease)
{
  ModeLeaseArbiter arbiter;
  auto release = make_command(
    "ELEVATOR_WAIT", "robot_elevator_manager", "mission-late",
    "lease-late");
  release.operation = ModeOperation::kRelease;

  const auto fenced = arbiter.apply(release, 10.0);
  ASSERT_TRUE(fenced.accepted);
  EXPECT_FALSE(fenced.state.lease_active);
  EXPECT_EQ(fenced.state.mode, OperatingMode::kNormal);

  const auto delayed = arbiter.apply(
    make_command(
      "ELEVATOR_WAIT", "robot_elevator_manager", "mission-late",
      "lease-late"),
    10.1);
  EXPECT_FALSE(delayed.accepted);
  EXPECT_EQ(delayed.code, ModeDecisionCode::kStaleLease);
  EXPECT_EQ(delayed.state.mode, OperatingMode::kNormal);
}

TEST(ModeLeaseArbiter, ConfiguredRecoveryOwnerCanPreempt)
{
  ModeLeaseArbiter arbiter;
  ASSERT_TRUE(arbiter.apply(make_command(), 10.0).accepted);

  const auto ordinary = arbiter.apply(
    make_command("RECOVERY", "ordinary_owner", "mission-r", "lease-r"), 11.0);
  EXPECT_FALSE(ordinary.accepted);
  EXPECT_EQ(ordinary.code, ModeDecisionCode::kLeaseConflict);

  const auto recovery = arbiter.apply(
    make_command("RECOVERY", "robot_mission_manager", "mission-r", "lease-r"), 12.0);
  EXPECT_TRUE(recovery.accepted);
  EXPECT_TRUE(recovery.changed);
  EXPECT_EQ(recovery.state.mode, OperatingMode::kRecovery);
  EXPECT_EQ(recovery.state.owner, "robot_mission_manager");
  EXPECT_EQ(recovery.state.generation, 2U);
  EXPECT_EQ(recovery.state.transition_reason, "recovery_preempted");

  const auto old_lease = arbiter.apply(make_command("DOORWAY"), 13.0);
  EXPECT_FALSE(old_lease.accepted);
  EXPECT_EQ(old_lease.code, ModeDecisionCode::kStaleLease);
}

TEST(ModeLeaseArbiter, RetiredLeaseNeverBecomesReusable)
{
  ModeLeaseArbiter arbiter;
  for (int index = 0; index < 80; ++index) {
    auto command = make_command(
      "ELEVATOR_WAIT",
      "robot_elevator_manager",
      "mission-" + std::to_string(index),
      "lease-" + std::to_string(index));
    ASSERT_TRUE(arbiter.apply(command, 10.0 + index).accepted);
    command.operation = ModeOperation::kRelease;
    ASSERT_TRUE(arbiter.apply(command, 10.1 + index).accepted);
  }

  const auto replay = arbiter.apply(
    make_command(
      "DOORWAY", "robot_elevator_manager", "mission-0", "lease-0"),
    100.0);

  EXPECT_FALSE(replay.accepted);
  EXPECT_EQ(replay.code, ModeDecisionCode::kStaleLease);
}

}  // namespace robot_mode_manager
