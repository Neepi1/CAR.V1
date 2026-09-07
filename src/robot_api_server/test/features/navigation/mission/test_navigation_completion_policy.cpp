#include <string>

#include "gtest/gtest.h"

#include "robot_api_server/features/navigation/mission/navigation_completion_policy.hpp"

namespace navigation = robot_api_server::features::navigation;

namespace
{

navigation::NavigationCompletionPolicyConfig production_like_config()
{
  navigation::NavigationCompletionPolicyConfig config;
  config.map_frame = "map";
  config.robot_pose_freshness_sec = 0.5;
  config.position_tolerance_m = 0.06;
  config.yaw_tolerance_rad = 0.05;
  config.yaw_align_trigger_rad = 0.08;
  config.final_verify_enabled = true;
  config.retry_uses_same_nav2_goal = true;
  config.final_verify_max_retry_count = 3;
  config.acceptance_slack_m = 0.02;
  config.xy_retry_min_error_m = 0.06;
  config.yaw_retry_if_failed = true;
  config.terminal_recovery_max_distance_m = 0.40;
  config.nav2_failed_near_goal_retry_enabled = true;
  config.nav2_failed_near_goal_retry_max_count = 1;
  config.nav2_failed_near_goal_retry_requires_yaw_error = false;
  return config;
}

robot_api_server::RobotPoseSnapshot map_pose(
  const double x,
  const double y,
  const double yaw,
  const double age_sec = 0.0)
{
  robot_api_server::RobotPoseSnapshot pose;
  pose.available = true;
  pose.frame_id = "map";
  pose.x = x;
  pose.y = y;
  pose.yaw = yaw;
  pose.age_sec = age_sec;
  return pose;
}

robot_api_server::StoredPose target_pose()
{
  robot_api_server::StoredPose target;
  target.x = 1.0;
  target.y = 2.0;
  target.yaw = 0.0;
  return target;
}

}  // namespace

TEST(NavigationCompletionPolicy, EvaluatesExactCommercialPoseGate)
{
  const navigation::NavigationCompletionPolicy policy(production_like_config());

  const auto inside = policy.evaluate_final_pose(
    map_pose(1.059, 2.0, 0.049), target_pose(), true, "");
  EXPECT_TRUE(inside.pose_available);
  EXPECT_TRUE(inside.position_reached);
  EXPECT_NEAR(inside.distance_m, 0.059, 1e-9);
  EXPECT_NEAR(inside.yaw_error_rad, 0.049, 1e-9);
  EXPECT_NE(inside.reason.find("position_tolerance=0.060"), std::string::npos);

  const auto outside = policy.evaluate_final_pose(
    map_pose(1.061, 2.0, 0.051), target_pose(), true, "");
  EXPECT_FALSE(outside.position_reached);
  EXPECT_GT(outside.distance_m, 0.06);
}

TEST(NavigationCompletionPolicy, RejectsUnavailableWrongFrameAndStalePose)
{
  const navigation::NavigationCompletionPolicy policy(production_like_config());
  robot_api_server::RobotPoseSnapshot unavailable;
  EXPECT_EQ(
    policy.evaluate_final_pose(unavailable, target_pose(), true, "pose timeout").reason,
    "pose timeout");

  auto wrong_frame = map_pose(1.0, 2.0, 0.0);
  wrong_frame.frame_id = "odom";
  EXPECT_EQ(
    policy.evaluate_final_pose(wrong_frame, target_pose(), true, "").reason,
    "no fresh map-frame robot pose");

  const auto stale = policy.evaluate_final_pose(
    map_pose(1.0, 2.0, 0.0, 0.51), target_pose(), true, "");
  EXPECT_FALSE(stale.pose_available);
  EXPECT_FALSE(stale.pose.available);
  EXPECT_EQ(stale.reason, "map-frame robot pose is stale");
}

TEST(NavigationCompletionPolicy, ClassifiesBoundedFinalVerifyRetries)
{
  const navigation::NavigationCompletionPolicy policy(production_like_config());
  navigation::FinalPoseCheck check;
  check.pose_available = true;
  check.position_reached = false;
  check.distance_m = 0.08;
  check.yaw_error_rad = 0.01;

  auto decision = policy.final_verify_retry(check, "pose_required", 0);
  ASSERT_TRUE(decision.allowed);
  EXPECT_EQ(decision.reason, "xy_light_overrun");
  EXPECT_EQ(decision.phase, "retry_nav2_after_final_verify_light_xy_overrun");

  check.distance_m = 0.20;
  decision = policy.final_verify_retry(check, "pose_required", 0);
  ASSERT_TRUE(decision.allowed);
  EXPECT_EQ(decision.reason, "xy_recovery_overrun");

  check.position_reached = true;
  check.distance_m = 0.03;
  check.yaw_error_rad = 0.10;
  decision = policy.final_verify_retry(check, "pose_required", 0);
  ASSERT_TRUE(decision.allowed);
  EXPECT_EQ(decision.reason, "yaw_light_overrun");

  EXPECT_FALSE(policy.final_verify_retry(check, "pose_required", 3).allowed);
}

TEST(NavigationCompletionPolicy, AllowsSlackOnlyAfterRetryBudgetIsExhausted)
{
  const navigation::NavigationCompletionPolicy policy(production_like_config());
  navigation::FinalPoseCheck check;
  check.pose_available = true;
  check.position_reached = false;
  check.distance_m = 0.075;
  check.reason = "final distance=0.075";

  EXPECT_FALSE(policy.final_verify_acceptance_slack(check, 2).allowed);
  const auto exhausted = policy.final_verify_acceptance_slack(check, 3);
  EXPECT_TRUE(exhausted.allowed);
  EXPECT_NE(exhausted.detail.find("effective_position_tolerance=0.080"), std::string::npos);
  EXPECT_NE(exhausted.detail.find("retry_count=3/3"), std::string::npos);
}

TEST(NavigationCompletionPolicy, BoundsNav2FailedNearGoalRetry)
{
  const navigation::NavigationCompletionPolicy policy(production_like_config());
  navigation::FinalPoseCheck check;
  check.pose_available = true;
  check.distance_m = 0.20;
  check.yaw_error_rad = 0.10;

  const auto allowed = policy.nav2_failed_near_goal_retry(
    check, "pose_required", 0.08, 0);
  EXPECT_TRUE(allowed.allowed);
  EXPECT_EQ(allowed.phase, "retry_nav2_after_nav2_failed_near_goal");
  EXPECT_NE(allowed.reason.find("distance=0.200"), std::string::npos);

  EXPECT_FALSE(policy.nav2_failed_near_goal_retry(
    check, "position_only", 0.08, 0).allowed);
  EXPECT_FALSE(policy.nav2_failed_near_goal_retry(
    check, "pose_required", 0.08, 1).allowed);
}
