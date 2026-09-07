#include <cmath>
#include <cstddef>

#include "gtest/gtest.h"
#include "robot_nav_config/ranger_exact_goal_endpoint.hpp"

namespace robot_nav_config
{
namespace
{

geometry_msgs::msg::PoseStamped exact_goal_pose(
  const double x, const double y, const double yaw)
{
  geometry_msgs::msg::PoseStamped result;
  result.header.frame_id = "map";
  result.pose.position.x = x;
  result.pose.position.y = y;
  result.pose.orientation.z = std::sin(yaw * 0.5);
  result.pose.orientation.w = std::cos(yaw * 0.5);
  return result;
}

nav_msgs::msg::Path snapped_path(
  const geometry_msgs::msg::PoseStamped & endpoint)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  path.poses.push_back(exact_goal_pose(-1.0, -1.0, 0.0));
  path.poses.push_back(endpoint);
  return path;
}

TEST(RangerExactGoalEndpoint, AppendsExactCommissionedPoseWithoutChangingSearchPath)
{
  const auto snapped = exact_goal_pose(-0.717628, -0.160219, 0.0);
  const auto requested = exact_goal_pose(-0.742627, -0.135219, 0.073631);
  const auto original = snapped_path(snapped);
  std::size_t checked_pose_count = 0U;

  const auto result = append_exact_goal_endpoint(
    original, requested, ExactGoalEndpointParameters{},
    [&checked_pose_count](const double, const double, const double) {
      ++checked_pose_count;
      return true;
    });

  ASSERT_TRUE(result.has_value());
  ASSERT_EQ(result->poses.size(), original.poses.size() + 1U);
  EXPECT_EQ(result->poses.front(), original.poses.front());
  EXPECT_EQ(result->poses[1], original.poses[1]);
  EXPECT_DOUBLE_EQ(result->poses.back().pose.position.x, requested.pose.position.x);
  EXPECT_DOUBLE_EQ(result->poses.back().pose.position.y, requested.pose.position.y);
  EXPECT_DOUBLE_EQ(result->poses.back().pose.orientation.z, requested.pose.orientation.z);
  EXPECT_DOUBLE_EQ(result->poses.back().pose.orientation.w, requested.pose.orientation.w);
  EXPECT_GT(checked_pose_count, 1U);
}

TEST(RangerExactGoalEndpoint, RejectsBlockedCentimeterEndpointBridge)
{
  const auto snapped = exact_goal_pose(0.0, 0.0, 0.0);
  const auto requested = exact_goal_pose(0.04, 0.0, 0.04);

  const auto result = append_exact_goal_endpoint(
    snapped_path(snapped), requested, ExactGoalEndpointParameters{},
    [](const double x, const double, const double) {return x < 0.03;});

  EXPECT_FALSE(result.has_value());
}

TEST(RangerExactGoalEndpoint, RejectsCorrectionOutsideLatticeQuantizationEnvelope)
{
  const auto snapped = exact_goal_pose(0.0, 0.0, 0.0);
  const auto requested = exact_goal_pose(0.20, 0.0, 0.0);

  const auto result = append_exact_goal_endpoint(
    snapped_path(snapped), requested, ExactGoalEndpointParameters{},
    [](const double, const double, const double) {return true;});

  EXPECT_FALSE(result.has_value());
}

TEST(RangerExactGoalEndpoint, RejectsFrameMismatch)
{
  const auto snapped = exact_goal_pose(0.0, 0.0, 0.0);
  auto requested = exact_goal_pose(0.02, 0.0, 0.0);
  requested.header.frame_id = "odom";

  const auto result = append_exact_goal_endpoint(
    snapped_path(snapped), requested, ExactGoalEndpointParameters{},
    [](const double, const double, const double) {return true;});

  EXPECT_FALSE(result.has_value());
}

}  // namespace
}  // namespace robot_nav_config
