#pragma once

#include <memory>
#include <string>

#include "nav2_smac_planner/smac_planner_lattice.hpp"
#include "robot_nav_config/ranger_direct_corridor.hpp"
#include "robot_nav_config/ranger_exact_goal_endpoint.hpp"

namespace robot_nav_config
{

class RangerMini3LatticePlanner : public nav2_smac_planner::SmacPlannerLattice
{
public:
  RangerMini3LatticePlanner() = default;
  ~RangerMini3LatticePlanner() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override;

private:
  bool corridor_pose_is_clear(double x, double y, double yaw);
  bool endpoint_pose_is_clear(double x, double y, double yaw);
  double path_length(const nav_msgs::msg::Path & path) const;

  bool direct_corridor_enabled_{true};
  DirectCorridorParameters direct_corridor_parameters_;
  int direct_corridor_max_cost_{0};
};

}  // namespace robot_nav_config
