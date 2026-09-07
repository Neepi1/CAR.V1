#pragma once

#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

#include "robot_api_server/features/navigation/terminal_control/navigation_terminal_control.hpp"

namespace robot_api_server::features::navigation
{

struct ModeControllerStatusSnapshot
{
  bool available{false};
  bool actual_available{false};
  bool actual_fresh{false};
  int actual_motion_mode_code{255};
  bool mode_aligned{false};
  double age_sec{-1.0};
  std::string raw;
};

struct NavigationTerminalRuntimeConfig
{
  std::string command_topic{"/cmd_vel_api"};
  int zero_command_count{3};

  bool speed_limit_enabled{true};
  std::string speed_limit_topic{"/speed_limit"};
  double speed_limit_far_mps{1.20};

  std::string reverse_enable_topic{"/ranger_mini3/allow_reverse"};
  bool post_nav2_reverse_permit_enabled{true};
  bool navigation_reverse_permit_enabled{true};
  double reverse_permit_enter_distance_m{0.30};
  double reverse_permit_exit_distance_m{0.35};
  double reverse_permit_refresh_period_sec{0.20};

  std::string mode_controller_status_topic{"/ranger_base/status"};
  std::string actual_stop_odom_topic{"/wheel/odom"};
  std::string local_costmap_topic{"/local_costmap/costmap"};
  std::string map_frame{"map"};
  std::string base_frame{"base_link"};
  double robot_pose_freshness_sec{0.5};

  bool yaw_actual_stop_check_enabled{true};
  double yaw_actual_wz_threshold_radps{0.02};
  int yaw_actual_wz_stable_samples{5};
  int yaw_actual_stop_timeout_ms{800};
  double yaw_actual_wz_max_age_sec{0.20};

  bool terminal_settle_enabled{true};
  double terminal_settle_linear_speed_threshold_mps{0.01};
  double terminal_settle_angular_speed_threshold_radps{0.02};
  double terminal_settle_stable_duration_sec{0.30};
  double terminal_settle_timeout_sec{2.50};
  double terminal_settle_odom_max_age_sec{0.20};
  bool terminal_settle_require_dual_ackermann_mode{true};
  double terminal_settle_mode_status_max_age_sec{0.50};
};

struct TerminalFramePoseSnapshot
{
  bool available{false};
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
  std::chrono::steady_clock::time_point received_at{};
};

struct NavigationTerminalRuntimePorts
{
  std::function<bool()> running;
  std::function<bool(std::uint64_t, std::string &)> cancel_requested;
  std::function<bool(std::string &)> safety_hard_blocked;
  std::function<FinalPoseCheck(const StoredPose &, bool)> verify_final_pose;
  std::function<void(std::uint64_t, const FinalPoseCheck &, const std::string &)>
  update_final_pose;
  std::function<void(std::uint64_t, const std::string &, const std::string &)>
  set_job_phase;
  std::function<void(const std::string &)> publish_motion_mode;
  std::function<RobotPoseSnapshot()> current_robot_pose;
  std::function<TerminalFramePoseSnapshot(const std::string &)> pose_in_frame;
  std::function<bool(std::string &)> dock_contact_blocked;
};

// Owns the complete ROS-facing runtime used by terminal navigation control:
// command/permit/limit publishers, chassis and odometry acknowledgements,
// local-costmap evidence, rosout drop evidence, and the policy callback port.
class NavigationTerminalRuntimeModule : public NavigationTerminalRuntimePort
{
public:
  NavigationTerminalRuntimeModule(
    rclcpp::Node & node,
    NavigationTerminalControl & terminal_control,
    NavigationTerminalRuntimeConfig config,
    NavigationTerminalRuntimePorts ports);
  ~NavigationTerminalRuntimeModule() override;

  NavigationTerminalRuntimeModule(const NavigationTerminalRuntimeModule &) = delete;
  NavigationTerminalRuntimeModule & operator=(const NavigationTerminalRuntimeModule &) = delete;

  void publish_command(const geometry_msgs::msg::Twist & command);
  void publish_zero_burst();
  void publish_speed_limit_for_goal(const StoredPose & target);
  void clear_speed_limit();
  void update_reverse_permit_for_goal(
    const StoredPose & target,
    bool & permit_active,
    std::chrono::steady_clock::time_point & next_refresh_at,
    const std::string & context);
  void clear_reverse_permit(bool & permit_active, const std::string & context);

  ModeControllerStatusSnapshot mode_controller_status_snapshot() const;
  void reset_yaw_actual_stop_stability();
  bool wait_for_yaw_actual_stop(const std::string & context, std::string & detail) const;
  void reset_actual_stop_stability();
  bool wait_for_actual_stop(
    const std::string & context,
    std::string & detail,
    bool require_dual_ackermann_mode) const;
  bool wait_for_actual_stop(const std::string & context, std::string & detail) const;

  std::uint64_t local_costmap_update_count() const;
  std::uint64_t local_costmap_message_filter_drop_count() const;
  std::string last_local_costmap_message_filter_drop_text() const;

  TerminalTimePoint terminal_now() const override;
  void terminal_sleep_for(std::chrono::milliseconds duration) override;
  bool terminal_cancel_requested(std::uint64_t job_id, std::string & detail) override;
  bool terminal_safety_hard_blocked(std::string & detail) override;
  FinalPoseCheck terminal_verify_final_pose(
    const StoredPose & target,
    bool require_fresh_pose) override;
  void terminal_update_final_pose(
    std::uint64_t job_id,
    const FinalPoseCheck & check,
    const std::string & reason) override;
  void terminal_set_job_phase(
    std::uint64_t job_id,
    const std::string & phase,
    const std::string & detail) override;
  void terminal_publish_command(const geometry_msgs::msg::Twist & command) override;
  void terminal_publish_motion_mode(const std::string & mode) override;
  bool terminal_reverse_permit_available() const override;
  void terminal_publish_reverse_permit(bool enabled) override;
  void terminal_reset_actual_stop_stability() override;
  bool terminal_wait_actual_stop(
    const std::string & context,
    std::string & detail) override;
  TerminalCostmapContext terminal_costmap_context(TerminalTimePoint now) override;
  RobotPoseSnapshot terminal_current_robot_pose() override;
  bool terminal_dock_contact_blocked(std::string & detail) override;
  void terminal_reset_yaw_actual_stop_stability() override;
  bool terminal_wait_yaw_actual_stop(
    const std::string & context,
    std::string & detail) override;
  void terminal_warn(const std::string & warning) override;

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace robot_api_server::features::navigation
