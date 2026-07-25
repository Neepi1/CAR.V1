#include <gtest/gtest.h>

#include "robot_nav_config/goal_scope_tracker.hpp"

namespace robot_nav_config
{

TEST(GoalScopeTracker, SuppressesReplansAfterStartupAlignment)
{
  GoalScopeTracker tracker;
  tracker.set_thresholds(0.01, 0.01);
  const GoalSignature goal{"map", 1.0, 2.0, 0.5};

  EXPECT_TRUE(tracker.observe_goal(goal));
  EXPECT_FALSE(tracker.startup_alignment_consumed());
  tracker.mark_startup_alignment_consumed();

  EXPECT_FALSE(tracker.observe_goal(goal));
  EXPECT_TRUE(tracker.startup_alignment_consumed());
}

TEST(GoalScopeTracker, RearmsWhenGoalPositionChanges)
{
  GoalScopeTracker tracker;
  tracker.set_thresholds(0.01, 0.01);
  EXPECT_TRUE(tracker.observe_goal({"map", 1.0, 2.0, 0.5}));
  tracker.mark_startup_alignment_consumed();

  EXPECT_TRUE(tracker.observe_goal({"map", 1.02, 2.0, 0.5}));
  EXPECT_FALSE(tracker.startup_alignment_consumed());
}

TEST(GoalScopeTracker, RearmsWhenGoalYawChanges)
{
  GoalScopeTracker tracker;
  tracker.set_thresholds(0.01, 0.01);
  EXPECT_TRUE(tracker.observe_goal({"map", 1.0, 2.0, 0.5}));
  tracker.mark_startup_alignment_consumed();

  EXPECT_TRUE(tracker.observe_goal({"map", 1.0, 2.0, 0.52}));
  EXPECT_FALSE(tracker.startup_alignment_consumed());
}

TEST(GoalScopeTracker, IgnoresSubThresholdPlannerNoise)
{
  GoalScopeTracker tracker;
  tracker.set_thresholds(0.01, 0.01);
  EXPECT_TRUE(tracker.observe_goal({"map", 1.0, 2.0, 3.14}));
  tracker.mark_startup_alignment_consumed();

  EXPECT_FALSE(tracker.observe_goal({"map", 1.005, 2.004, -3.139}));
  EXPECT_TRUE(tracker.startup_alignment_consumed());
}

}  // namespace robot_nav_config
