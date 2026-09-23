#pragma once

#include <chrono>
#include <cstddef>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_rotation_shim_controller/nav2_rotation_shim_controller.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "robot_nav_config/goal_scope_tracker.hpp"
#include "robot_nav_config/navlite_log_gate.hpp"
#include "robot_nav_config/ordinary_local_path_repair_runtime.hpp"
#include "robot_nav_config/navigation_recovery/recovery_state.hpp"
#include "robot_nav_config/srv/prepare_ordinary_navigation_recovery.hpp"
#include "robot_nav_config/startup_alignment_guard.hpp"
#include "robot_nav_config/terminal_pose_handoff.hpp"
#include "std_msgs/msg/bool.hpp"

namespace robot_nav_config
{

class GoalScopedRotationShimController
  : public nav2_rotation_shim_controller::RotationShimController
{
public:
  GoalScopedRotationShimController() = default;
  ~GoalScopedRotationShimController() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void activate() override;
  void deactivate() override;
  void cleanup() override;
  void setPlan(const nav_msgs::msg::Path & path) override;

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;

private:
  void log_navlite_output(const char * branch,
    const geometry_msgs::msg::TwistStamped & command,
    const geometry_msgs::msg::PoseStamped & pose);
  NavliteLogGate navlite_output_log_;

  struct StartupHeadingMeasurement
  {
    std::optional<double> error;
    bool path_too_short{false};
  };

  GoalSignature goal_signature(const nav_msgs::msg::Path & path) const;
  StartupHeadingMeasurement startup_path_heading_error(
    const geometry_msgs::msg::PoseStamped & pose);
  std::optional<double> terminal_goal_yaw_error(
    const geometry_msgs::msg::PoseStamped & pose,
    nav2_core::GoalChecker * goal_checker);
  std::optional<TerminalPoseError> terminal_pose_error(
    const geometry_msgs::msg::PoseStamped & pose);
  std::optional<TerminalPathMetrics> terminal_path_metrics(
    const geometry_msgs::msg::PoseStamped & pose);
  bool terminal_command_is_clear(
    const geometry_msgs::msg::PoseStamped & pose,
    const TerminalVelocityCommand & command);
  geometry_msgs::msg::TwistStamped terminal_handoff_command(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    const TerminalPoseError & error);
  void publish_terminal_permits(bool lateral, bool reverse);
  static const char * terminal_phase_name(TerminalControlPhase phase);

  GoalScopeTracker goal_scope_;
  StartupAlignmentGuard startup_alignment_guard_;
  TerminalHandoffParameters terminal_handoff_parameters_;
  TerminalPoseHandoffController terminal_handoff_controller_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> terminal_costmap_ros_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Bool>::SharedPtr
    terminal_lateral_permit_pub_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Bool>::SharedPtr
    terminal_reverse_permit_pub_;
  std::unique_ptr<OrdinaryLocalPathRepairRuntime> ordinary_local_repair_runtime_;
  std::shared_ptr<navigation_recovery::RecoveryState> ordinary_recovery_state_;
  rclcpp::Service<robot_nav_config::srv::PrepareOrdinaryNavigationRecovery>::SharedPtr
    ordinary_recovery_service_;
  bool recovery_alignment_active_{false};
  bool recovery_alignment_logged_{false};
  bool rotate_to_heading_once_{true};
  double goal_change_xy_threshold_{0.01};
  double goal_change_yaw_threshold_{0.01};
  double same_goal_rearm_after_idle_sec_{2.0};
  bool terminal_rotation_braking_enabled_{true};
  double terminal_costmap_lookahead_m_{0.15};
  std::string terminal_lateral_permit_topic_;
  std::string terminal_reverse_permit_topic_;
  bool have_last_compute_time_{false};
  std::chrono::steady_clock::time_point last_compute_time_;
  std::size_t suppressed_same_goal_replans_{0U};
};

}  // namespace robot_nav_config
