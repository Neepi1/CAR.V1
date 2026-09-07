#include <gtest/gtest.h>

#include "robot_api_server/features/floor_switch/floor_switch_handoff_tracker.hpp"

namespace robot_api_server
{
namespace
{

TEST(FloorSwitchHandoffTracker, ReadyRemainsLatchedAcrossFollowingStage)
{
  FloorSwitchHandoffTracker tracker;
  tracker.reset("elevator-test-1", 7U);

  ASSERT_TRUE(tracker.observe_feedback(
      "elevator-test-1", 7U, 4U, true));
  ASSERT_TRUE(tracker.observe_feedback(
      "elevator-test-1", 7U, 5U, false));

  const auto state = tracker.snapshot();
  EXPECT_TRUE(state.ready);
  EXPECT_EQ(state.accepted_stage_sequence, 5U);
}

TEST(FloorSwitchHandoffTracker, RejectsFeedbackFromAnotherSubmission)
{
  FloorSwitchHandoffTracker tracker;
  tracker.reset("elevator-test-2", 8U);

  EXPECT_FALSE(tracker.observe_feedback(
      "elevator-test-2", 7U, 9U, true));
  EXPECT_FALSE(tracker.observe_feedback(
      "another-transaction", 8U, 9U, true));
  EXPECT_FALSE(tracker.snapshot().ready);
}

TEST(FloorSwitchHandoffTracker, RejectsOutOfOrderButAcceptsDuplicateReady)
{
  FloorSwitchHandoffTracker tracker;
  tracker.reset("elevator-test-3", 9U);

  ASSERT_TRUE(tracker.observe_feedback(
      "elevator-test-3", 9U, 5U, false));
  EXPECT_FALSE(tracker.observe_feedback(
      "elevator-test-3", 9U, 4U, true));
  EXPECT_FALSE(tracker.snapshot().ready);
  EXPECT_TRUE(tracker.observe_feedback(
      "elevator-test-3", 9U, 5U, true));
  EXPECT_TRUE(tracker.snapshot().ready);
}

TEST(FloorSwitchHandoffTracker, TerminalBeforeReadyCannotBeRescuedByLateFeedback)
{
  FloorSwitchHandoffTracker tracker;
  tracker.reset("elevator-test-4", 10U);

  ASSERT_TRUE(tracker.observe_terminal("elevator-test-4", 10U));
  EXPECT_FALSE(tracker.observe_feedback(
      "elevator-test-4", 10U, 6U, true));

  const auto state = tracker.snapshot();
  EXPECT_TRUE(state.terminal);
  EXPECT_TRUE(state.terminal_before_ready);
  EXPECT_FALSE(state.ready);
}

TEST(FloorSwitchHandoffTracker, TerminalAfterReadyPreservesProvenHandoff)
{
  FloorSwitchHandoffTracker tracker;
  tracker.reset("elevator-test-5", 11U);

  ASSERT_TRUE(tracker.observe_feedback(
      "elevator-test-5", 11U, 6U, true));
  ASSERT_TRUE(tracker.observe_terminal("elevator-test-5", 11U));

  const auto state = tracker.snapshot();
  EXPECT_TRUE(state.terminal);
  EXPECT_FALSE(state.terminal_before_ready);
  EXPECT_TRUE(state.ready);
}

}  // namespace
}  // namespace robot_api_server
