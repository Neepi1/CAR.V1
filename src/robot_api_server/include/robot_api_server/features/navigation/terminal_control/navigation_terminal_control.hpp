#pragma once

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

#include "robot_api_server/features/navigation/mission/navigation_completion_policy.hpp"

namespace robot_api_server::features::navigation
{

struct TerminalLateralCorrectionResult
{
  bool attempted{false};
  bool succeeded{false};
  bool canceled{false};
  bool blocked{false};
  bool settle_confirmed{false};
  bool direction_reversed{false};
  std::string detail;
  std::string settle_detail;
  double duration_sec{-1.0};
  double settle_duration_sec{-1.0};
  double initial_forward_m{0.0};
  double initial_lateral_m{0.0};
  double final_forward_m{0.0};
  double final_lateral_m{0.0};
  double final_distance_m{-1.0};
  double settled_forward_m{0.0};
  double settled_lateral_m{0.0};
  double settled_distance_m{-1.0};
  int settle_recheck_count{0};
};

struct FinalYawAlignResult
{
  bool attempted{false};
  bool succeeded{false};
  bool blocked{false};
  bool canceled{false};
  std::string phase{"position_reached_yaw_aligning"};
  std::string detail;
  std::string blocked_reason;
  double duration_sec{-1.0};
  double initial_yaw_error_rad{-1.0};
  double final_yaw_error_rad{-1.0};
  double observed_xy_drift_m{0.0};
};

struct TerminalControlConfig
{
  bool final_verify_enabled{true};
  bool api_velocity_correction_enabled{true};
  bool lateral_correction_enabled{true};
  bool reverse_permit_enabled{true};
  bool costmap_guard_enabled{true};

  double goal_position_success_tolerance_m{0.06};
  double terminal_recovery_max_distance_m{0.35};
  double final_yaw_success_tolerance_rad{0.045};

  double lateral_target_m{0.03};
  double lateral_trigger_m{0.04};
  double lateral_max_forward_m{0.15};
  double lateral_speed_mps{0.04};
  double lateral_kp{0.8};
  double lateral_timeout_sec{20.0};
  double lateral_command_sign{1.0};
  int terminal_settle_max_recheck_count{1};

  double lateral_divergence_epsilon_m{0.01};
  int lateral_divergence_count{3};
  std::string lateral_forced_mode{"side_slip"};
  std::string lateral_release_mode{"auto"};

  double final_yaw_kp{1.8};
  double final_yaw_min_speed_radps{0.10};
  double final_yaw_max_speed_radps{0.50};
  double final_yaw_slow_max_speed_radps{0.22};
  double final_yaw_slowdown_start_rad{0.16};
  double final_yaw_timeout_sec{8.0};
  double final_yaw_max_xy_drift_m{0.08};
  bool final_yaw_require_fresh_pose{true};
  double robot_pose_freshness_sec{0.5};
  std::string map_frame{"map"};
  bool yaw_stop_lead_enabled{true};
  double yaw_stop_lead_time_sec{0.12};
  double yaw_stop_lead_min_rad{0.0};
  double yaw_stop_lead_max_rad{0.08};
  int zero_command_count{3};

  double costmap_max_age_sec{0.50};
  int costmap_occupied_threshold{50};
  double costmap_lookahead_m{0.15};

  double speed_limit_far_distance_m{2.0};
  double speed_limit_mid_distance_m{1.2};
  double speed_limit_near_distance_m{0.6};
  double speed_limit_crawl_distance_m{0.15};
  double speed_limit_far_mps{1.20};
  double speed_limit_mid_mps{0.70};
  double speed_limit_near_mps{0.40};
  double speed_limit_crawl_mps{0.25};
  double speed_limit_final_mps{0.10};
};

struct LateralCorrectionGate
{
  bool allowed{false};
  double forward_m{0.0};
  double lateral_m{0.0};
  std::string reason;
};

enum class TerminalCorrectionAxis
{
  kNone,
  kYaw,
  kLateral,
  kForward,
};

struct TerminalCorrectionStep
{
  TerminalCorrectionAxis axis{TerminalCorrectionAxis::kNone};
  geometry_msgs::msg::Twist command;
};

struct TerminalCostmapContext
{
  nav_msgs::msg::OccupancyGrid::SharedPtr grid;
  std::chrono::steady_clock::time_point grid_received_at{};
  std::string grid_frame;
  bool robot_pose_available{false};
  double robot_x{0.0};
  double robot_y{0.0};
  double robot_yaw{0.0};
  std::chrono::steady_clock::time_point robot_pose_received_at{};
};

struct TerminalCostmapCheck
{
  bool clear{false};
  std::string detail;
};

using TerminalTimePoint = std::chrono::steady_clock::time_point;

class NavigationTerminalRuntimePort
{
public:
  virtual ~NavigationTerminalRuntimePort() = default;

  virtual TerminalTimePoint terminal_now() const = 0;
  virtual void terminal_sleep_for(std::chrono::milliseconds duration) = 0;
  virtual bool terminal_cancel_requested(std::uint64_t job_id, std::string & detail) = 0;
  virtual bool terminal_safety_hard_blocked(std::string & detail) = 0;
  virtual FinalPoseCheck terminal_verify_final_pose(
    const StoredPose & target,
    bool require_fresh_pose) = 0;
  virtual void terminal_update_final_pose(
    std::uint64_t job_id,
    const FinalPoseCheck & check,
    const std::string & reason) = 0;
  virtual void terminal_set_job_phase(
    std::uint64_t job_id,
    const std::string & phase,
    const std::string & detail) = 0;
  virtual void terminal_publish_command(const geometry_msgs::msg::Twist & command) = 0;
  virtual void terminal_publish_motion_mode(const std::string & mode) = 0;
  virtual bool terminal_reverse_permit_available() const = 0;
  virtual void terminal_publish_reverse_permit(bool enabled) = 0;
  virtual void terminal_reset_actual_stop_stability() = 0;
  virtual bool terminal_wait_actual_stop(
    const std::string & context,
    std::string & detail) = 0;
  virtual TerminalCostmapContext terminal_costmap_context(TerminalTimePoint now) = 0;
  virtual RobotPoseSnapshot terminal_current_robot_pose() = 0;
  virtual bool terminal_dock_contact_blocked(std::string & detail) = 0;
  virtual void terminal_reset_yaw_actual_stop_stability() = 0;
  virtual bool terminal_wait_yaw_actual_stop(
    const std::string & context,
    std::string & detail) = 0;
  virtual void terminal_warn(const std::string & warning) = 0;
};

class NavigationTerminalControl
{
public:
  explicit NavigationTerminalControl(TerminalControlConfig config);

  const TerminalControlConfig & config() const noexcept;
  double speed_limit_for_distance(double distance_m) const;
  double final_yaw_command_speed(double signed_yaw_error_rad) const;
  double yaw_stop_threshold(
    double command_speed_radps,
    double success_tolerance_rad) const;

  static void goal_error_in_base_frame(
    const StoredPose & target,
    const RobotPoseSnapshot & pose,
    double & forward_m,
    double & lateral_m);

  LateralCorrectionGate lateral_correction_gate(
    const StoredPose & target,
    const FinalPoseCheck & check,
    const std::string & goal_completion_policy) const;

  TerminalCorrectionStep correction_step(
    double forward_error_m,
    double lateral_error_m,
    double signed_yaw_error_rad,
    double lateral_direction_multiplier) const;

  TerminalCostmapCheck costmap_path_clear(
    const TerminalCostmapContext & context,
    double forward_probe_m,
    double lateral_probe_m,
    std::chrono::steady_clock::time_point now) const;

  TerminalLateralCorrectionResult run_lateral_correction(
    std::uint64_t job_id,
    const StoredPose & target,
    const FinalPoseCheck & initial_check,
    const std::string & goal_completion_policy,
    NavigationTerminalRuntimePort & runtime) const;

  FinalYawAlignResult run_final_yaw_motion(
    std::uint64_t job_id,
    const StoredPose & target,
    const FinalPoseCheck & initial_check,
    NavigationTerminalRuntimePort & runtime) const;

  static std::string lateral_correction_diagnostics(
    const TerminalLateralCorrectionResult & result);

private:
  void publish_zero_burst(NavigationTerminalRuntimePort & runtime) const;

  TerminalControlConfig config_;
};

}  // namespace robot_api_server::features::navigation
