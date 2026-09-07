#include "robot_nav_config/ranger_mini3_lattice_planner.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <utility>

#include "nav2_core/exceptions.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/footprint_collision_checker.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace robot_nav_config
{

void RangerMini3LatticePlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  nav2_smac_planner::SmacPlannerLattice::configure(parent, name, tf, costmap_ros);

  auto node = parent.lock();
  if (!node) {
    throw std::runtime_error("RangerMini3LatticePlanner parent node expired");
  }

  nav2_util::declare_parameter_if_not_declared(
    node, name + ".direct_corridor_enabled", rclcpp::ParameterValue(true));
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".direct_corridor_minimum_length_m", rclcpp::ParameterValue(1.0));
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".direct_corridor_goal_yaw_tolerance_rad", rclcpp::ParameterValue(0.20));
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".direct_corridor_sample_step_m", rclcpp::ParameterValue(0.025));
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".direct_corridor_max_cost", rclcpp::ParameterValue(0));

  node->get_parameter(name + ".direct_corridor_enabled", direct_corridor_enabled_);
  node->get_parameter(
    name + ".direct_corridor_minimum_length_m",
    direct_corridor_parameters_.minimum_length_m);
  node->get_parameter(
    name + ".direct_corridor_goal_yaw_tolerance_rad",
    direct_corridor_parameters_.goal_yaw_tolerance_rad);
  node->get_parameter(
    name + ".direct_corridor_sample_step_m",
    direct_corridor_parameters_.sample_step_m);
  node->get_parameter(name + ".direct_corridor_max_cost", direct_corridor_max_cost_);

  direct_corridor_parameters_.minimum_length_m = std::max(
    0.0, direct_corridor_parameters_.minimum_length_m);
  direct_corridor_parameters_.goal_yaw_tolerance_rad = std::clamp(
    direct_corridor_parameters_.goal_yaw_tolerance_rad, 0.0, M_PI);
  direct_corridor_parameters_.sample_step_m = std::max(
    0.005, direct_corridor_parameters_.sample_step_m);
  direct_corridor_max_cost_ = std::clamp(direct_corridor_max_cost_, 0, 252);

  RCLCPP_INFO(
    _logger,
    "Ranger direct-corridor policy: enabled=%s min_length=%.2fm "
    "goal_yaw_tolerance=%.3frad sample_step=%.3fm max_cost=%d",
    direct_corridor_enabled_ ? "true" : "false",
    direct_corridor_parameters_.minimum_length_m,
    direct_corridor_parameters_.goal_yaw_tolerance_rad,
    direct_corridor_parameters_.sample_step_m,
    direct_corridor_max_cost_);
}

nav_msgs::msg::Path RangerMini3LatticePlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  auto lattice_path = nav2_smac_planner::SmacPlannerLattice::createPlan(start, goal);
  if (lattice_path.poses.empty()) {
    return lattice_path;
  }

  std::lock_guard<std::mutex> lock(_mutex);
  if (direct_corridor_enabled_) {
    const auto direct_path = make_direct_corridor_path(
      start, goal, direct_corridor_parameters_,
      [this](const double x, const double y, const double yaw) {
        return corridor_pose_is_clear(x, y, yaw);
      });
    if (direct_path.has_value()) {
      RCLCPP_INFO(
        _logger,
        "Ranger direct corridor selected: direct_length=%.3fm lattice_length=%.3fm poses=%zu",
        path_length(*direct_path), path_length(lattice_path), direct_path->poses.size());
      return *direct_path;
    }
  }

  const auto exact_goal_path = append_exact_goal_endpoint(
    lattice_path, goal, ExactGoalEndpointParameters{},
    [this](const double x, const double y, const double yaw) {
      return endpoint_pose_is_clear(x, y, yaw);
    });
  if (!exact_goal_path.has_value()) {
    throw nav2_core::PlannerException(
            "Ranger lattice endpoint cannot safely preserve the exact requested goal pose");
  }
  if (exact_goal_path->poses.size() != lattice_path.poses.size()) {
    RCLCPP_DEBUG(
      _logger,
      "Ranger lattice exact goal endpoint appended: lattice_poses=%zu final_poses=%zu",
      lattice_path.poses.size(), exact_goal_path->poses.size());
  }
  return *exact_goal_path;
}

bool RangerMini3LatticePlanner::corridor_pose_is_clear(
  const double x, const double y, const double yaw)
{
  if (_costmap == nullptr || !std::isfinite(x) || !std::isfinite(y) || !std::isfinite(yaw)) {
    return false;
  }

  const double resolution = _costmap->getResolution();
  const double map_x = (x - _costmap->getOriginX()) / resolution;
  const double map_y = (y - _costmap->getOriginY()) / resolution;
  if (map_x < 0.0 || map_y < 0.0 ||
    map_x >= static_cast<double>(_costmap->getSizeInCellsX()) ||
    map_y >= static_cast<double>(_costmap->getSizeInCellsY()))
  {
    return false;
  }

  const auto cell_x = static_cast<unsigned int>(std::floor(map_x));
  const auto cell_y = static_cast<unsigned int>(std::floor(map_y));
  if (static_cast<int>(_costmap->getCost(cell_x, cell_y)) > direct_corridor_max_cost_) {
    return false;
  }

  if (_costmap_ros == nullptr) {
    return false;
  }
  nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *> checker(
    _costmap);
  const double footprint_cost = checker.footprintCostAtPose(
    x, y, yaw, _costmap_ros->getRobotFootprint());
  return footprint_cost < static_cast<double>(nav2_costmap_2d::LETHAL_OBSTACLE);
}

bool RangerMini3LatticePlanner::endpoint_pose_is_clear(
  const double x, const double y, const double yaw)
{
  if (_costmap == nullptr || _costmap_ros == nullptr ||
    !std::isfinite(x) || !std::isfinite(y) || !std::isfinite(yaw))
  {
    return false;
  }

  const double resolution = _costmap->getResolution();
  const double map_x = (x - _costmap->getOriginX()) / resolution;
  const double map_y = (y - _costmap->getOriginY()) / resolution;
  if (map_x < 0.0 || map_y < 0.0 ||
    map_x >= static_cast<double>(_costmap->getSizeInCellsX()) ||
    map_y >= static_cast<double>(_costmap->getSizeInCellsY()))
  {
    return false;
  }

  nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *> checker(
    _costmap);
  const double footprint_cost = checker.footprintCostAtPose(
    x, y, yaw, _costmap_ros->getRobotFootprint());
  return footprint_cost >= 0.0 &&
         footprint_cost < static_cast<double>(nav2_costmap_2d::LETHAL_OBSTACLE);
}

double RangerMini3LatticePlanner::path_length(const nav_msgs::msg::Path & path) const
{
  double length_m = 0.0;
  for (std::size_t index = 1U; index < path.poses.size(); ++index) {
    const auto & previous = path.poses[index - 1U].pose.position;
    const auto & current = path.poses[index].pose.position;
    length_m += std::hypot(current.x - previous.x, current.y - previous.y);
  }
  return length_m;
}

}  // namespace robot_nav_config

PLUGINLIB_EXPORT_CLASS(
  robot_nav_config::RangerMini3LatticePlanner,
  nav2_core::GlobalPlanner)
