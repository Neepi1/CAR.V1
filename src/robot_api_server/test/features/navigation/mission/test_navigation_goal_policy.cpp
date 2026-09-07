#include <cmath>
#include <string>

#include "gtest/gtest.h"

#include "robot_api_server/features/navigation/mission/navigation_goal_policy.hpp"

namespace navigation = robot_api_server::features::navigation;

namespace
{

navigation::NavigationGoalPolicyConfig config()
{
  navigation::NavigationGoalPolicyConfig value;
  value.default_completion_policy = "pose_required";
  value.delivery_point_completion_policy = "position_only";
  value.position_only_nav2_yaw_mode = "approach_heading";
  value.position_only_approach_heading_min_distance_m = 0.20;
  return value;
}

robot_api_server::StoredPose target(const std::string & type = "delivery")
{
  robot_api_server::StoredPose value;
  value.type = type;
  value.x = 2.0;
  value.y = 2.0;
  value.yaw = 1.25;
  return value;
}

robot_api_server::RobotPoseSnapshot current_pose(
  const double x,
  const double y,
  const double yaw)
{
  robot_api_server::RobotPoseSnapshot value;
  value.available = true;
  value.x = x;
  value.y = y;
  value.yaw = yaw;
  return value;
}

}  // namespace

TEST(NavigationGoalPolicy, NormalizesAndValidatesPublicModes)
{
  EXPECT_EQ(navigation::normalize_navigation_goal_completion_policy("Pose-Required"),
    "pose_required");
  EXPECT_TRUE(navigation::is_valid_navigation_goal_completion_policy("position_only"));
  EXPECT_TRUE(navigation::is_valid_navigation_goal_completion_policy("dock_staging"));
  EXPECT_FALSE(navigation::is_valid_navigation_goal_completion_policy("loose"));

  EXPECT_EQ(navigation::normalize_position_only_nav2_yaw_mode("Current-Yaw"), "current_yaw");
  EXPECT_TRUE(navigation::is_valid_position_only_nav2_yaw_mode("approach_heading"));
  EXPECT_FALSE(navigation::is_valid_position_only_nav2_yaw_mode("spin"));
}

TEST(NavigationGoalPolicy, ResolvesExplicitAndStoredPoseCompletionPolicy)
{
  const navigation::NavigationGoalPolicy policy(config());

  auto result = policy.resolve_completion_policy(
    R"({"goal_completion_policy":"POSITION-ONLY"})", target());
  EXPECT_EQ(result.policy, "position_only");
  EXPECT_TRUE(result.error.empty());

  result = policy.resolve_completion_policy(R"({"require_yaw":true})", target());
  EXPECT_EQ(result.policy, "pose_required");

  result = policy.resolve_completion_policy("{}", target("precise_pose"));
  EXPECT_EQ(result.policy, "pose_required");

  result = policy.resolve_completion_policy("{}", target("delivery"));
  EXPECT_EQ(result.policy, "position_only");

  result = policy.resolve_completion_policy(
    R"({"goal_completion_policy":"unsupported"})", target());
  EXPECT_EQ(result.policy, "pose_required");
  EXPECT_EQ(result.error,
    "goal_completion_policy must be position_only, pose_required, or dock_staging");
}

TEST(NavigationGoalPolicy, KeepsStoredYawForPoseRequired)
{
  const navigation::NavigationGoalPolicy policy(config());
  const auto result = policy.resolve_nav2_goal_yaw(
    "pose_required", target(), current_pose(0.0, 0.0, -0.5), "");
  EXPECT_NEAR(result.yaw, 1.25, 1e-12);
  EXPECT_EQ(result.source, "stored_yaw");
}

TEST(NavigationGoalPolicy, UsesApproachHeadingAndCurrentYawNearTarget)
{
  const navigation::NavigationGoalPolicy policy(config());
  auto result = policy.resolve_nav2_goal_yaw(
    "position_only", target(), current_pose(1.0, 2.0, -0.5), "");
  EXPECT_NEAR(result.yaw, 0.0, 1e-12);
  EXPECT_EQ(result.source, "approach_heading");

  result = policy.resolve_nav2_goal_yaw(
    "position_only", target(), current_pose(1.9, 2.0, -0.5), "");
  EXPECT_NEAR(result.yaw, -0.5, 1e-12);
  EXPECT_EQ(result.source, "current_yaw_near_target");
}

TEST(NavigationGoalPolicy, FallsBackToStoredYawWithoutFreshPose)
{
  const navigation::NavigationGoalPolicy policy(config());
  robot_api_server::RobotPoseSnapshot unavailable;
  const auto result = policy.resolve_nav2_goal_yaw(
    "position_only", target(), unavailable, "pose timeout");
  EXPECT_NEAR(result.yaw, 1.25, 1e-12);
  EXPECT_EQ(result.source, "stored_yaw_fallback_no_fresh_pose:pose timeout");
}
