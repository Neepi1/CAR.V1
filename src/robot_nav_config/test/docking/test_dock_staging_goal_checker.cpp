#include <cmath>
#include <memory>

#include <gtest/gtest.h>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "robot_nav_config/docking/dock_staging_goal_checker.hpp"

namespace
{

geometry_msgs::msg::Pose pose(
  const double x, const double y, const double yaw)
{
  geometry_msgs::msg::Pose value;
  value.position.x = x;
  value.position.y = y;
  value.orientation.z = std::sin(yaw * 0.5);
  value.orientation.w = std::cos(yaw * 0.5);
  return value;
}

TEST(DockStagingGoalChecker, ReadsScopedLimitsAndDoesNotRequireZeroVelocity)
{
  rclcpp::init(0, nullptr);
  rclcpp::NodeOptions options;
  options.parameter_overrides({
    rclcpp::Parameter("dock_staging.forward_capture_min_m", -0.20),
    rclcpp::Parameter("dock_staging.forward_capture_max_m", 0.30),
    rclcpp::Parameter("dock_staging.lateral_capture_max_m", 0.10),
    rclcpp::Parameter("dock_staging.yaw_capture_max_rad", 0.50),
  });
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
    "dock_staging_goal_checker_test", "", options);
  robot_nav_config::DockStagingGoalChecker checker;
  checker.initialize(node, "dock_staging", nullptr);

  geometry_msgs::msg::Twist moving;
  moving.linear.x = 0.12;
  EXPECT_TRUE(checker.isGoalReached(
      pose(0.25, 0.05, 0.40), pose(0.0, 0.0, 0.0), moving));
  EXPECT_FALSE(checker.isGoalReached(
      pose(0.31, 0.05, 0.40), pose(0.0, 0.0, 0.0), moving));

  geometry_msgs::msg::Pose pose_tolerance;
  geometry_msgs::msg::Twist velocity_tolerance;
  EXPECT_TRUE(checker.getTolerances(pose_tolerance, velocity_tolerance));
  // RotationShim interprets this scalar as a circular XY gate.  It must be
  // wholly contained by the asymmetric docking capture rectangle; reporting
  // the rectangle's outer radius lets RotationShim enter terminal-yaw mode
  // while the real goal checker is still false, which permanently suppresses
  // translational progress.
  EXPECT_DOUBLE_EQ(pose_tolerance.position.x, 0.10);
  EXPECT_DOUBLE_EQ(pose_tolerance.position.y, 0.10);
  EXPECT_DOUBLE_EQ(pose_tolerance.orientation.z, 0.50);

  rclcpp::shutdown();
}

}  // namespace
