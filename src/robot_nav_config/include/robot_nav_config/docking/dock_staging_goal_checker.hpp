#pragma once

#include <memory>
#include <optional>
#include <string>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_core/goal_checker.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "robot_nav_config/docking/dock_staging_goal_policy.hpp"

namespace robot_nav_config
{

class DockStagingGoalChecker : public nav2_core::GoalChecker
{
public:
  DockStagingGoalChecker() = default;
  ~DockStagingGoalChecker() override = default;

  void initialize(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    const std::string & plugin_name,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void reset() override;

  bool isGoalReached(
    const geometry_msgs::msg::Pose & query_pose,
    const geometry_msgs::msg::Pose & goal_pose,
    const geometry_msgs::msg::Twist & velocity) override;

  bool getTolerances(
    geometry_msgs::msg::Pose & pose_tolerance,
    geometry_msgs::msg::Twist & vel_tolerance) override;

private:
  rclcpp::Logger logger_{rclcpp::get_logger("DockStagingGoalChecker")};
  std::string plugin_name_;
  std::unique_ptr<DockStagingGoalPolicy> policy_;
  std::optional<bool> last_reached_;
};

}  // namespace robot_nav_config
