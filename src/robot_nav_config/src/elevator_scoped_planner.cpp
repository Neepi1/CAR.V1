#include "robot_nav_config/elevator_scoped_planner.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <utility>

#include "nav2_core/exceptions.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "robot_nav_config/elevator_scoped_clearance.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.h"

namespace robot_nav_config
{

void ElevatorScopedPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer>,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  auto node = parent.lock();
  if (!node) {
    throw std::runtime_error("ElevatorScopedPlanner parent node expired");
  }
  node_ = parent;
  logger_ = node->get_logger();
  plugin_name_ = std::move(name);
  costmap_ros_ = std::move(costmap_ros);
  if (!costmap_ros_ || !costmap_ros_->getCostmap()) {
    throw std::runtime_error("ElevatorScopedPlanner requires a global costmap");
  }

  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_distance_m", rclcpp::ParameterValue(2.5));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".translation_step_m", rclcpp::ParameterValue(0.025));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".rotation_step_rad", rclcpp::ParameterValue(0.05));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".search_grid_step_m", rclcpp::ParameterValue(0.05));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".search_radius_m", rclcpp::ParameterValue(2.5));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".goal_connect_distance_m", rclcpp::ParameterValue(0.10));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".maximum_expansions", rclcpp::ParameterValue(60000));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".forward_cost", rclcpp::ParameterValue(1.0));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".reverse_cost", rclcpp::ParameterValue(1.35));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".lateral_cost", rclcpp::ParameterValue(1.15));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".mode_switch_cost", rclcpp::ParameterValue(0.20));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".soft_cost_weight", rclcpp::ParameterValue(0.35));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".clearance_preference_cost_ratio",
    rclcpp::ParameterValue(1.10));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".maximum_search_time_sec",
    rclcpp::ParameterValue(1.0));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".prefer_clear_startup_spin",
    rclcpp::ParameterValue(false));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".reverse_entry_staging_sequence",
    rclcpp::ParameterValue(false));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".reverse_docking_sequence",
    rclcpp::ParameterValue(false));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".unchecked_direct_path",
    rclcpp::ParameterValue(false));

  node->get_parameter(
    plugin_name_ + ".max_distance_m", parameters_.sampling.max_distance_m);
  node->get_parameter(
    plugin_name_ + ".translation_step_m", parameters_.sampling.translation_step_m);
  node->get_parameter(
    plugin_name_ + ".rotation_step_rad", parameters_.sampling.rotation_step_rad);
  node->get_parameter(
    plugin_name_ + ".search_grid_step_m", parameters_.search_grid_step_m);
  node->get_parameter(
    plugin_name_ + ".search_radius_m", parameters_.search_radius_m);
  node->get_parameter(
    plugin_name_ + ".goal_connect_distance_m",
    parameters_.goal_connect_distance_m);
  int maximum_expansions = 60000;
  node->get_parameter(
    plugin_name_ + ".maximum_expansions", maximum_expansions);
  node->get_parameter(plugin_name_ + ".forward_cost", parameters_.forward_cost);
  node->get_parameter(plugin_name_ + ".reverse_cost", parameters_.reverse_cost);
  node->get_parameter(plugin_name_ + ".lateral_cost", parameters_.lateral_cost);
  node->get_parameter(
    plugin_name_ + ".mode_switch_cost", parameters_.mode_switch_cost);
  node->get_parameter(
    plugin_name_ + ".soft_cost_weight", parameters_.soft_cost_weight);
  node->get_parameter(
    plugin_name_ + ".clearance_preference_cost_ratio",
    parameters_.clearance_preference_cost_ratio);
  node->get_parameter(
    plugin_name_ + ".maximum_search_time_sec",
    parameters_.maximum_search_time_sec);
  bool prefer_clear_startup_spin = false;
  node->get_parameter(
    plugin_name_ + ".prefer_clear_startup_spin",
    prefer_clear_startup_spin);
  bool reverse_entry_staging_sequence = false;
  node->get_parameter(
    plugin_name_ + ".reverse_entry_staging_sequence",
    reverse_entry_staging_sequence);
  bool reverse_docking_sequence = false;
  node->get_parameter(
    plugin_name_ + ".reverse_docking_sequence",
    reverse_docking_sequence);
  node->get_parameter(
    plugin_name_ + ".unchecked_direct_path", unchecked_direct_path_);
  const int selected_route_policies =
    static_cast<int>(prefer_clear_startup_spin) +
    static_cast<int>(reverse_entry_staging_sequence) +
    static_cast<int>(reverse_docking_sequence);
  if (selected_route_policies > 1) {
    throw std::runtime_error(
            "ElevatorScopedPlanner route policies are mutually exclusive");
  }
  parameters_.route_policy = reverse_docking_sequence ?
    ElevatorScopedRoutePolicy::kReverseDockingSequence :
    (reverse_entry_staging_sequence ?
    ElevatorScopedRoutePolicy::kReverseEntryStagingSequence :
    (prefer_clear_startup_spin ?
    ElevatorScopedRoutePolicy::kPreferClearStartupSpin :
    ElevatorScopedRoutePolicy::kPreserveIngressHeading));

  parameters_.sampling.max_distance_m = std::clamp(
    parameters_.sampling.max_distance_m, 0.06, 2.5);
  parameters_.sampling.translation_step_m = std::clamp(
    parameters_.sampling.translation_step_m, 0.005, 0.05);
  parameters_.sampling.rotation_step_rad = std::clamp(
    parameters_.sampling.rotation_step_rad, 0.01, 0.10);
  parameters_.search_grid_step_m = std::clamp(
    parameters_.search_grid_step_m, 0.025, 0.10);
  parameters_.search_radius_m = std::clamp(
    parameters_.search_radius_m,
    parameters_.sampling.max_distance_m, 3.0);
  parameters_.goal_connect_distance_m = std::clamp(
    parameters_.goal_connect_distance_m, 0.05, 0.25);
  parameters_.maximum_expansions = static_cast<std::size_t>(
    std::clamp(maximum_expansions, 1000, 200000));
  parameters_.forward_cost = std::clamp(parameters_.forward_cost, 0.5, 5.0);
  parameters_.reverse_cost = std::clamp(
    parameters_.reverse_cost, parameters_.forward_cost, 8.0);
  parameters_.lateral_cost = std::clamp(parameters_.lateral_cost, 0.5, 8.0);
  parameters_.mode_switch_cost = std::clamp(
    parameters_.mode_switch_cost, 0.0, 5.0);
  parameters_.soft_cost_weight = std::clamp(
    parameters_.soft_cost_weight, 0.0, 5.0);
  parameters_.clearance_preference_cost_ratio = std::clamp(
    parameters_.clearance_preference_cost_ratio, 1.0, 2.0);
  parameters_.maximum_search_time_sec = std::clamp(
    parameters_.maximum_search_time_sec, 0.01, 1.0);

  RCLCPP_INFO(
    logger_,
    "Elevator scoped planner configured: max_distance=%.2fm "
    "sample_step=%.3fm search_step=%.3fm radius=%.2fm expansions=%zu "
    "budget=%.0fms clearance_ratio=%.2f route_policy=%s "
    "motion=16_heading_forward_reverse_lateral_spin "
    "clearance=filled_footprint_lethal_unknown",
    parameters_.sampling.max_distance_m,
    parameters_.sampling.translation_step_m,
    parameters_.search_grid_step_m,
    parameters_.search_radius_m,
    parameters_.maximum_expansions,
    parameters_.maximum_search_time_sec * 1000.0,
    parameters_.clearance_preference_cost_ratio,
    reverse_docking_sequence ? "reverse_docking_sequence" :
    (reverse_entry_staging_sequence ? "reverse_entry_staging_sequence" :
    (prefer_clear_startup_spin ?
    "clear_startup_spin_then_escape" : "preserve_ingress_heading")));
}

void ElevatorScopedPlanner::cleanup()
{
  costmap_ros_.reset();
  node_.reset();
}

void ElevatorScopedPlanner::activate()
{
  RCLCPP_INFO(logger_, "Elevator scoped planner activated");
}

void ElevatorScopedPlanner::deactivate()
{
  RCLCPP_INFO(logger_, "Elevator scoped planner deactivated");
}

nav_msgs::msg::Path ElevatorScopedPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  const std::string frame_id =
    !start.header.frame_id.empty() ? start.header.frame_id :
    goal.header.frame_id;
  if (frame_id.empty() ||
    (!start.header.frame_id.empty() && !goal.header.frame_id.empty() &&
    start.header.frame_id != goal.header.frame_id) ||
    !costmap_ros_ ||
    frame_id != costmap_ros_->getGlobalFrameID())
  {
    throw nav2_core::PlannerException(
            "elevator scoped start/goal must share the global costmap frame");
  }

  const ElevatorScopedPose scoped_start{
    start.pose.position.x,
    start.pose.position.y,
    tf2::getYaw(start.pose.orientation),
  };
  const ElevatorScopedPose scoped_goal{
    goal.pose.position.x,
    goal.pose.position.y,
    tf2::getYaw(goal.pose.orientation),
  };
  if (unchecked_direct_path_) {
    const double distance = std::hypot(
      scoped_goal.x - scoped_start.x, scoped_goal.y - scoped_start.y);
    if (!std::isfinite(distance) || distance > parameters_.sampling.max_distance_m) {
      throw nav2_core::PlannerException(
              "unchecked elevator entry path is outside its bounded envelope");
    }
    nav_msgs::msg::Path path;
    path.header.frame_id = frame_id;
    path.header.stamp = start.header.stamp;
    path.poses = {start, goal};
    path.poses.front().header = path.header;
    path.poses.back().header = path.header;
    RCLCPP_WARN(
      logger_,
      "Elevator entry direct Nav2 path selected: distance=%.3fm clearance=transaction-bypassed",
      distance);
    return path;
  }
  auto * costmap = costmap_ros_->getCostmap();
  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(
    *costmap->getMutex());
  const auto footprint = costmap_ros_->getRobotFootprint();
  const auto search = search_elevator_scoped_path(
    *costmap, footprint, scoped_start, scoped_goal, parameters_);
  if (!search.succeeded()) {
    throw nav2_core::PlannerException(
            "elevator scoped search failed status=" +
            std::string(elevator_scoped_search_status_name(search.status)) +
            " expanded=" + std::to_string(search.expanded_nodes) +
            " direct_block_reason=" +
            elevator_scoped_clearance_status_name(
              search.direct_path_block.status) +
            " cell=(" + std::to_string(search.direct_path_block.cell_x) + "," +
            std::to_string(search.direct_path_block.cell_y) + ")" +
            " cost=" + std::to_string(
              static_cast<unsigned int>(search.direct_path_block.cost)));
  }
  const auto & scoped_path = search.path;

  nav_msgs::msg::Path path;
  path.header.frame_id = frame_id;
  path.header.stamp = start.header.stamp;
  path.poses.reserve(scoped_path.samples.size());
  for (const auto & sample : scoped_path.samples) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = sample.pose.x;
    pose.pose.position.y = sample.pose.y;
    pose.pose.position.z = start.pose.position.z;
    pose.pose.orientation.z = std::sin(sample.pose.yaw * 0.5);
    pose.pose.orientation.w = std::cos(sample.pose.yaw * 0.5);
    path.poses.push_back(std::move(pose));
  }
  path.poses.back().pose = goal.pose;
  path.poses.back().header.frame_id = frame_id;

  RCLCPP_INFO(
    logger_,
    "Elevator scoped path accepted: distance=%.3fm path_length=%.3fm "
    "detour=%s startup_rotation_escape=%s expanded=%zu "
    "mode_switches=%zu samples=%zu",
    std::hypot(
      scoped_goal.x - scoped_start.x,
      scoped_goal.y - scoped_start.y),
    search.path_length_m,
    search.used_detour ? "true" : "false",
    search.used_startup_rotation_escape ? "true" : "false",
    search.expanded_nodes,
    search.mode_switches,
    path.poses.size());
  return path;
}

const char * ElevatorScopedPlanner::phase_name(
  const ElevatorScopedMotionPhase phase)
{
  switch (phase) {
    case ElevatorScopedMotionPhase::kYaw:
      return "yaw";
    case ElevatorScopedMotionPhase::kLateral:
      return "lateral";
    case ElevatorScopedMotionPhase::kForward:
      return "forward";
    case ElevatorScopedMotionPhase::kReverse:
      return "reverse";
  }
  return "unknown";
}

}  // namespace robot_nav_config

PLUGINLIB_EXPORT_CLASS(
  robot_nav_config::ElevatorScopedPlanner,
  nav2_core::GlobalPlanner)
