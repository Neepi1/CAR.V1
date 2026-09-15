#include <gtest/gtest.h>
#include "robot_nav_config/navigation_recovery/recovery_state.hpp"

using robot_nav_config::navigation_recovery::RecoveryState;
namespace
{
nav_msgs::msg::Path path(int sec, double x = 2.0)
{
  nav_msgs::msg::Path p;
  p.header.frame_id = "map";
  p.header.stamp.sec = sec;
  p.poses.resize(2);
  for (auto & pose : p.poses) {
    pose.header = p.header;
    pose.pose.orientation.w = 1.0;
  }
  p.poses.back().pose.position.x = x;
  return p;
}
void failed(RecoveryState & state)
{
  state.set_active(true);
  state.observe_plan(path(1));
  state.reset_progress();
  state.observe_control();
  state.observe_progress(false, 3000000000LL);
}
}

TEST(OrdinaryRecoveryState, RequiresActualProgressFailure)
{
  RecoveryState state;
  state.set_active(true);
  state.observe_plan(path(1));
  state.observe_control();
  std::string reason;
  EXPECT_FALSE(state.prepare(path(4), 1000000000LL, reason));
  EXPECT_EQ(reason, "no_matching_progress_failure");
}

TEST(OrdinaryRecoveryState, ExactPreparedPathRearmsOnceWithoutIdleDelay)
{
  RecoveryState state;
  failed(state);
  std::string reason;
  auto next = path(4);
  ASSERT_TRUE(state.prepare(next, 1000000000LL, reason));
  // Either reset-before-plan or plan-before-reset must work on Humble.
  state.reset_progress();
  EXPECT_TRUE(state.observe_plan(next));
  EXPECT_FALSE(state.observe_plan(next));
}

TEST(OrdinaryRecoveryState, NewPathDiscardsCancelledPreparation)
{
  RecoveryState state;
  failed(state);
  std::string reason;
  ASSERT_TRUE(state.prepare(path(4), 1000000000LL, reason));
  EXPECT_FALSE(state.observe_plan(path(5)));
  EXPECT_FALSE(state.observe_plan(path(4)));
}

TEST(OrdinaryRecoveryState, RejectsOldFailureChangedGoalAndStalePlan)
{
  RecoveryState state;
  failed(state);
  std::string reason;
  EXPECT_FALSE(state.prepare(path(4), 4000000000LL, reason));
  EXPECT_FALSE(state.prepare(path(4, 3.0), 1000000000LL, reason));
  EXPECT_FALSE(state.prepare(path(2), 1000000000LL, reason));
  auto changed_yaw = path(4);
  changed_yaw.poses.back().pose.orientation.z = 1.0;
  changed_yaw.poses.back().pose.orientation.w = 0.0;
  EXPECT_FALSE(state.prepare(changed_yaw, 1000000000LL, reason));
}

TEST(OrdinaryRecoveryState, OtherControllersDoNotInheritSelectedController)
{
  RecoveryState state;
  failed(state);
  state.reset_progress();
  state.observe_progress(false, 5000000000LL);
  std::string reason;
  EXPECT_FALSE(state.prepare(path(6), 4000000000LL, reason));
}

TEST(OrdinaryRecoveryState, LifecycleAndResumedProgressRetireFailure)
{
  RecoveryState state;
  failed(state);
  state.observe_progress(true, 4000000000LL);
  std::string reason;
  EXPECT_FALSE(state.prepare(path(5), 1000000000LL, reason));
  failed(state);
  ASSERT_TRUE(state.prepare(path(4), 1000000000LL, reason));
  state.set_active(false);
  EXPECT_FALSE(state.observe_plan(path(4)));
  EXPECT_FALSE(state.prepare(path(4), 1000000000LL, reason));
}

TEST(OrdinaryRecoveryState, SameEndpointButDifferentPathCannotConsumePreparation)
{
  RecoveryState state;
  failed(state);
  std::string reason;
  ASSERT_TRUE(state.prepare(path(4), 1000000000LL, reason));
  auto other = path(4);
  other.poses.front().pose.position.y = 0.1;
  EXPECT_FALSE(state.observe_plan(other));
}

TEST(OrdinaryRecoveryState, WaitingIsExplicitFreshAndScopedToCurrentAttempt)
{
  RecoveryState state;
  failed(state);
  state.observe_pose(0.0, 0.0, true, 4000000000LL);
  EXPECT_TRUE(state.inspect(path(1), 1000000000LL, 4500000000LL).waiting);
  EXPECT_FALSE(state.inspect(path(1), 5000000000LL, 5000000000LL).waiting);
  EXPECT_FALSE(state.inspect(path(1), 1000000000LL, 5100000000LL).waiting);
  EXPECT_FALSE(state.inspect(path(1, 3.0), 1000000000LL, 4500000000LL).matched);
  state.reset_progress();
  EXPECT_FALSE(state.inspect(path(1), 1000000000LL, 4500000000LL).matched);
}

TEST(OrdinaryRecoveryState, StationaryWaitAndJitterDoNotRenewStartupRecovery)
{
  RecoveryState state;
  failed(state);
  std::string reason;
  ASSERT_TRUE(state.prepare(path(4), 1000000000LL, reason));
  EXPECT_TRUE(state.observe_plan(path(4)));
  state.reset_progress();
  state.observe_control();
  for (int i = 0; i < 100; ++i) {
    state.observe_progress(true, 5000000000LL + i * 100000000LL);
    state.observe_pose((i % 2) * 0.01, 0.0, false, 5000000000LL + i * 100000000LL);
  }
  EXPECT_EQ(state.inspect(path(4), 4000000000LL, 15000000000LL).progress_epoch, 0U);
  state.observe_progress(false, 16000000000LL);
  ASSERT_TRUE(state.prepare(path(17), 4000000000LL, reason));
  bool preserve = false;
  EXPECT_FALSE(state.observe_plan(path(17), &preserve));
  EXPECT_TRUE(preserve);
}

TEST(OrdinaryRecoveryState, SustainedTranslationAllowsNewBlockageEpisode)
{
  RecoveryState state;
  failed(state);
  std::string reason;
  ASSERT_TRUE(state.prepare(path(4), 1000000000LL, reason));
  ASSERT_TRUE(state.observe_plan(path(4)));
  state.reset_progress();
  state.observe_control();
  for (int i = 0; i <= 25; ++i) {
    state.observe_pose(i * 0.01, 0.0, false, 5000000000LL + i * 100000000LL);
  }
  EXPECT_EQ(state.inspect(path(4), 4000000000LL, 7500000000LL).progress_epoch, 1U);
  state.observe_progress(false, 8000000000LL);
  ASSERT_TRUE(state.prepare(path(9), 4000000000LL, reason));
  EXPECT_TRUE(state.observe_plan(path(9)));
}

TEST(OrdinaryRecoveryState, NewOuterTaskAtSameEndpointGetsItsOwnRecovery)
{
  RecoveryState state;
  failed(state);
  std::string reason;
  ASSERT_TRUE(state.prepare(path(4), 1000000000LL, reason, 100));
  ASSERT_TRUE(state.observe_plan(path(4)));
  state.reset_progress();
  state.observe_control();
  state.observe_progress(false, 6000000000LL);
  ASSERT_TRUE(state.prepare(path(7), 4000000000LL, reason, 200));
  EXPECT_TRUE(state.observe_plan(path(7)));
}

TEST(OrdinaryRecoveryState, ProgressUsesRobotFrameNotGlobalGoalCoordinates)
{
  RecoveryState state;
  failed(state); // path ends at map x=2; odom origin is unrelated
  for (int i = 0; i <= 25; ++i) {
    state.observe_pose(200.0 + i * 0.01, 50.0, false,
      5000000000LL + i * 100000000LL, "odom");
  }
  EXPECT_EQ(state.inspect(path(1), 1000000000LL, 7500000000LL).progress_epoch, 1U);
}

TEST(OrdinaryRecoveryState, FrameChangeAndSinglePoseJumpDoNotCountAsProgress)
{
  RecoveryState state;
  failed(state);
  for (int i = 0; i <= 25; ++i) {
    state.observe_pose(i < 10 ? 0.0 : 1.0, 0.0, false,
      5000000000LL + i * 100000000LL, i < 10 ? "odom" : "other");
  }
  EXPECT_EQ(state.inspect(path(1), 1000000000LL, 7500000000LL).progress_epoch, 0U);
}
