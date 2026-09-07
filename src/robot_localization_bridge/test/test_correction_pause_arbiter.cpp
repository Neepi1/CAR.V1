#include <gtest/gtest.h>

#include "robot_localization_bridge/correction_pause_arbiter.hpp"

namespace robot_localization_bridge
{

TEST(CorrectionPauseArbiter, StartsUnpaused)
{
  CorrectionPauseArbiter arbiter;
  const auto state = arbiter.snapshot();
  EXPECT_FALSE(state.paused);
  EXPECT_TRUE(state.lease_keys.empty());
  EXPECT_EQ(state.generation, 0U);
}

TEST(CorrectionPauseArbiter, MultipleOwnersCompose)
{
  CorrectionPauseArbiter arbiter;
  PauseCommand docking{
    PauseOperation::kAcquire, "robot_api_server", "shared-1", "docking_fine", 1U};
  PauseCommand elevator{
    PauseOperation::kAcquire, "robot_elevator_manager", "shared-1", "ride", 1U};

  EXPECT_TRUE(arbiter.apply(docking).accepted);
  const auto second = arbiter.apply(elevator);
  EXPECT_TRUE(second.accepted);
  EXPECT_EQ(second.state.lease_keys.size(), 2U);
  EXPECT_TRUE(second.state.paused);

  docking.operation = PauseOperation::kRelease;
  docking.command_sequence = 2U;
  const auto first_released = arbiter.apply(docking);
  EXPECT_TRUE(first_released.accepted);
  EXPECT_TRUE(first_released.state.paused);
  EXPECT_EQ(first_released.state.lease_keys.size(), 1U);

  elevator.operation = PauseOperation::kRelease;
  elevator.command_sequence = 2U;
  const auto all_released = arbiter.apply(elevator);
  EXPECT_TRUE(all_released.accepted);
  EXPECT_FALSE(all_released.state.paused);
}

TEST(CorrectionPauseArbiter, ExactAbsentReleaseCannotReleaseAnotherOwner)
{
  CorrectionPauseArbiter arbiter;
  ASSERT_TRUE(
    arbiter.apply(
      PauseCommand{
        PauseOperation::kAcquire, "robot_elevator_manager", "elevator-1",
        "ride", 1U})
    .accepted);

  const auto idempotent = arbiter.apply(
    PauseCommand{
      PauseOperation::kRelease, "robot_mission_manager", "elevator-1", "",
      1U});
  EXPECT_TRUE(idempotent.accepted);
  EXPECT_TRUE(idempotent.changed);
  EXPECT_TRUE(idempotent.state.paused);
  EXPECT_EQ(idempotent.state.generation, 2U);
  ASSERT_EQ(idempotent.state.lease_keys.size(), 1U);
  EXPECT_EQ(
    idempotent.state.lease_keys.front(),
    "robot_elevator_manager:elevator-1");
}

TEST(CorrectionPauseArbiter, DuplicateAcquireIsIdempotent)
{
  CorrectionPauseArbiter arbiter;
  PauseCommand acquire{
    PauseOperation::kAcquire, "robot_elevator_manager", "elevator-1", "ride",
    1U};
  ASSERT_TRUE(arbiter.apply(acquire).accepted);
  acquire.command_sequence = 2U;
  const auto duplicate = arbiter.apply(acquire);
  EXPECT_TRUE(duplicate.accepted);
  EXPECT_FALSE(duplicate.changed);
  EXPECT_EQ(duplicate.state.generation, 1U);
}

TEST(CorrectionPauseArbiter, RejectsUnsafeFieldsAndStaleCommands)
{
  CorrectionPauseArbiter arbiter;
  const auto invalid = arbiter.apply(
    PauseCommand{PauseOperation::kAcquire, "", "elevator-1", "ride", 1U});
  EXPECT_FALSE(invalid.accepted);
  EXPECT_EQ(invalid.code, PauseDecisionCode::kInvalidRequest);

  const auto zero_sequence = arbiter.apply(
    PauseCommand{
      PauseOperation::kAcquire, "owner-a", "shared-tx", "reason", 0U});
  EXPECT_FALSE(zero_sequence.accepted);
  EXPECT_EQ(zero_sequence.code, PauseDecisionCode::kInvalidRequest);

  ASSERT_TRUE(
    arbiter.apply(
      PauseCommand{
        PauseOperation::kAcquire, "owner-a", "shared-tx", "reason", 10U})
    .accepted);
  ASSERT_TRUE(
    arbiter.apply(
      PauseCommand{
        PauseOperation::kRelease, "owner-a", "shared-tx", "", 12U})
    .accepted);
  const auto stale = arbiter.apply(
    PauseCommand{
      PauseOperation::kAcquire, "owner-a", "shared-tx", "late", 11U});
  EXPECT_FALSE(stale.accepted);
  EXPECT_EQ(stale.code, PauseDecisionCode::kStaleCommand);
  EXPECT_EQ(stale.applied_sequence, 12U);
  EXPECT_FALSE(stale.state.paused);
}

}  // namespace robot_localization_bridge
