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
    PauseOperation::kAcquire, "robot_api_server", "dock-1", "docking_fine"};
  PauseCommand elevator{
    PauseOperation::kAcquire, "robot_elevator_manager", "elevator-1", "ride"};

  EXPECT_TRUE(arbiter.apply(docking).accepted);
  const auto second = arbiter.apply(elevator);
  EXPECT_TRUE(second.accepted);
  EXPECT_EQ(second.state.lease_keys.size(), 2U);
  EXPECT_TRUE(second.state.paused);

  docking.operation = PauseOperation::kRelease;
  const auto first_released = arbiter.apply(docking);
  EXPECT_TRUE(first_released.accepted);
  EXPECT_TRUE(first_released.state.paused);
  EXPECT_EQ(first_released.state.lease_keys.size(), 1U);

  elevator.operation = PauseOperation::kRelease;
  const auto all_released = arbiter.apply(elevator);
  EXPECT_TRUE(all_released.accepted);
  EXPECT_FALSE(all_released.state.paused);
}

TEST(CorrectionPauseArbiter, WrongOwnerCannotRelease)
{
  CorrectionPauseArbiter arbiter;
  ASSERT_TRUE(
    arbiter.apply(
      PauseCommand{
        PauseOperation::kAcquire, "robot_elevator_manager", "elevator-1", "ride"})
    .accepted);

  const auto rejected = arbiter.apply(
    PauseCommand{
      PauseOperation::kRelease, "robot_mission_manager", "elevator-1", ""});
  EXPECT_FALSE(rejected.accepted);
  EXPECT_EQ(rejected.code, PauseDecisionCode::kNotOwner);
  EXPECT_TRUE(rejected.state.paused);
  EXPECT_EQ(rejected.state.generation, 1U);
}

TEST(CorrectionPauseArbiter, DuplicateAcquireIsIdempotent)
{
  CorrectionPauseArbiter arbiter;
  const PauseCommand acquire{
    PauseOperation::kAcquire, "robot_elevator_manager", "elevator-1", "ride"};
  ASSERT_TRUE(arbiter.apply(acquire).accepted);
  const auto duplicate = arbiter.apply(acquire);
  EXPECT_TRUE(duplicate.accepted);
  EXPECT_FALSE(duplicate.changed);
  EXPECT_EQ(duplicate.state.generation, 1U);
}

TEST(CorrectionPauseArbiter, RejectsUnsafeEmptyFieldsAndTransactionCollision)
{
  CorrectionPauseArbiter arbiter;
  const auto invalid = arbiter.apply(
    PauseCommand{PauseOperation::kAcquire, "", "elevator-1", "ride"});
  EXPECT_FALSE(invalid.accepted);
  EXPECT_EQ(invalid.code, PauseDecisionCode::kInvalidRequest);

  ASSERT_TRUE(
    arbiter.apply(
      PauseCommand{PauseOperation::kAcquire, "owner-a", "shared-tx", "reason"})
    .accepted);
  const auto conflict = arbiter.apply(
    PauseCommand{PauseOperation::kAcquire, "owner-b", "shared-tx", "reason"});
  EXPECT_FALSE(conflict.accepted);
  EXPECT_EQ(conflict.code, PauseDecisionCode::kConflict);
  EXPECT_EQ(conflict.state.generation, 1U);
}

}  // namespace robot_localization_bridge
