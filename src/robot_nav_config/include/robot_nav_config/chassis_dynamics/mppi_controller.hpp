#pragma once

#include <memory>
#include "nav2_core/controller.hpp"

namespace robot_nav_config
{
// Humble MPPI adapter: same optimizer, path handling, critics and action contract;
// measured MotionModel plus post-filter control constraints. No command publisher lives here.
class RangerMPPIController : public nav2_core::Controller
{
public:
  RangerMPPIController();
  ~RangerMPPIController() override;
  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap) override;
  void cleanup() override;
  void activate() override;
  void deactivate() override;
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist & speed,
    nav2_core::GoalChecker * goal_checker) override;
  void setPlan(const nav_msgs::msg::Path & path) override;
  void setSpeedLimit(const double & limit, const bool & percentage) override;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}
