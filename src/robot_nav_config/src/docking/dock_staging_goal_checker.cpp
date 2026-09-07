#include "robot_nav_config/docking/dock_staging_goal_checker.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.h"

namespace robot_nav_config
{
namespace
{

bool valid_quaternion(const geometry_msgs::msg::Quaternion & quaternion)
{
  const bool finite =
    std::isfinite(quaternion.x) && std::isfinite(quaternion.y) &&
    std::isfinite(quaternion.z) && std::isfinite(quaternion.w);
  const double norm_squared =
    quaternion.x * quaternion.x + quaternion.y * quaternion.y +
    quaternion.z * quaternion.z + quaternion.w * quaternion.w;
  return finite && norm_squared > 1e-12;
}

}  // namespace

void DockStagingGoalChecker::initialize(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  const std::string & plugin_name,
  const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  (void)costmap_ros;
  auto node = parent.lock();
  if (!node) {
    throw std::runtime_error("DockStagingGoalChecker parent node expired");
  }
  logger_ = node->get_logger();
  plugin_name_ = plugin_name;

  const auto declare_double = [&node, this](
      const char * suffix, const double default_value) {
      nav2_util::declare_parameter_if_not_declared(
        node, plugin_name_ + suffix, rclcpp::ParameterValue(default_value));
    };
  declare_double(".forward_capture_min_m", -0.40);
  declare_double(".forward_capture_max_m", 0.55);
  declare_double(".lateral_capture_max_m", 0.25);
  declare_double(".yaw_capture_max_rad", 3.14159265358979323846);

  DockStagingGoalLimits limits;
  node->get_parameter(
    plugin_name_ + ".forward_capture_min_m", limits.forward_capture_min_m);
  node->get_parameter(
    plugin_name_ + ".forward_capture_max_m", limits.forward_capture_max_m);
  node->get_parameter(
    plugin_name_ + ".lateral_capture_max_m", limits.lateral_capture_max_m);
  node->get_parameter(
    plugin_name_ + ".yaw_capture_max_rad", limits.yaw_capture_max_rad);
  policy_ = std::make_unique<DockStagingGoalPolicy>(limits);

  RCLCPP_INFO(
    logger_,
    "Dock staging goal checker configured: forward=[%.3f, %.3f]m "
    "lateral=%.3fm yaw=%.3frad",
    limits.forward_capture_min_m, limits.forward_capture_max_m,
    limits.lateral_capture_max_m, limits.yaw_capture_max_rad);
}

void DockStagingGoalChecker::reset()
{
  last_reached_.reset();
}

bool DockStagingGoalChecker::isGoalReached(
  const geometry_msgs::msg::Pose & query_pose,
  const geometry_msgs::msg::Pose & goal_pose,
  const geometry_msgs::msg::Twist & velocity)
{
  (void)velocity;
  if (!policy_ || !valid_quaternion(query_pose.orientation) ||
    !valid_quaternion(goal_pose.orientation))
  {
    return false;
  }

  const auto assessment = policy_->assess({
    query_pose.position.x,
    query_pose.position.y,
    tf2::getYaw(query_pose.orientation),
    goal_pose.position.x,
    goal_pose.position.y,
    tf2::getYaw(goal_pose.orientation)});

  if (assessment.valid && assessment.reached &&
    (!last_reached_.has_value() || !*last_reached_))
  {
    RCLCPP_INFO(
      logger_,
      "Predock entered docking-manager capture envelope: "
      "forward=%.3fm lateral=%.3fm yaw=%.3frad",
      assessment.forward_error_m, assessment.lateral_error_m,
      assessment.yaw_error_rad);
  }
  last_reached_ = assessment.reached;
  return assessment.valid && assessment.reached;
}

bool DockStagingGoalChecker::getTolerances(
  geometry_msgs::msg::Pose & pose_tolerance,
  geometry_msgs::msg::Twist & vel_tolerance)
{
  if (!policy_) {
    return false;
  }
  const auto & limits = policy_->limits();
  // Humble RotationShim consumes position.x as a circular XY tolerance before
  // it asks the real goal checker whether the asymmetric capture rectangle is
  // reached.  Report the largest circle wholly contained by that rectangle.
  // An outer/circumscribed radius can make RotationShim hold terminal-yaw
  // control forever at a pose which this checker still rejects, suppressing
  // the translation needed to enter the capture envelope.
  const double translation_bound = std::min({
      std::fabs(limits.forward_capture_min_m),
      std::fabs(limits.forward_capture_max_m),
      limits.lateral_capture_max_m});

  pose_tolerance = geometry_msgs::msg::Pose{};
  pose_tolerance.position.x = translation_bound;
  pose_tolerance.position.y = translation_bound;
  pose_tolerance.orientation.z = limits.yaw_capture_max_rad;
  vel_tolerance = geometry_msgs::msg::Twist{};
  vel_tolerance.linear.x = std::numeric_limits<double>::lowest();
  vel_tolerance.linear.y = std::numeric_limits<double>::lowest();
  vel_tolerance.angular.z = std::numeric_limits<double>::lowest();
  return true;
}

}  // namespace robot_nav_config

PLUGINLIB_EXPORT_CLASS(
  robot_nav_config::DockStagingGoalChecker,
  nav2_core::GoalChecker)
