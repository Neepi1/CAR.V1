#pragma once

#include <memory>
#include <string>

#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_nav_config/elevator_scoped_search.hpp"

namespace robot_nav_config
{

class ElevatorScopedPlanner : public nav2_core::GlobalPlanner
{
public:
  ElevatorScopedPlanner() = default;
  ~ElevatorScopedPlanner() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
  void cleanup() override;
  void activate() override;
  void deactivate() override;

  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override;

private:
  static const char * phase_name(ElevatorScopedMotionPhase phase);

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  rclcpp::Logger logger_{rclcpp::get_logger("ElevatorScopedPlanner")};
  std::string plugin_name_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  ElevatorScopedSearchParameters parameters_;
  bool unchecked_direct_path_{false};
};

}  // namespace robot_nav_config
