#pragma once

#include <cstdint>
#include <string>

namespace robot_api_server::features::navigation
{

struct NavigationGoalJob
{
  std::uint64_t id{0U};
  std::string state{"idle"};
  std::string phase{"idle"};
  std::string pose_id;
  std::string building_id;
  std::string floor_id;
  std::string detail;
  std::string started_at;
  std::string completed_at;
  std::string goal_completion_policy{"pose_required"};
  bool native_nav2_goal_completion{true};
  bool api_final_yaw_align_enabled{false};
  bool post_nav2_final_verify_enabled{false};
  bool post_nav2_final_verify_wait_bridge_smoothing{false};
  double post_nav2_final_verify_bridge_wait_elapsed_ms{0.0};
  bool post_nav2_final_verify_bridge_wait_timeout{false};
  int final_verify_retry_count{0};
  std::string final_verify_retry_reason;
  int final_verify_retry_max_count{0};
  bool final_verify_retry_goal_sent{false};
  double final_verify_xy_error_m{-1.0};
  double final_verify_yaw_error_rad{-1.0};
  bool final_verify_failure_is_terminal{false};
  bool post_nav2_final_verify_api_velocity_correction_enabled{false};
  bool nav2_rotation_shim_enabled{true};
  double target_x{0.0};
  double target_y{0.0};
  double target_yaw{0.0};
  double nav2_goal_yaw{0.0};
  std::string nav2_goal_yaw_source{"stored_yaw"};
  double final_distance_m{-1.0};
  double final_yaw_error_rad{-1.0};
  int nav2_result_code{0};
  bool position_reached{false};
  bool nav2_succeeded{false};
  bool yaw_align_required{false};
  bool yaw_align_active{false};
  bool yaw_align_failed{false};
  bool final_yaw_align_requested{false};
  bool final_yaw_align_attempted{false};
  bool final_yaw_align_succeeded{false};
  bool final_yaw_align_blocked{false};
  std::string final_yaw_align_blocked_reason;
  double final_yaw_align_duration_sec{-1.0};
  double final_yaw_align_timeout_sec{-1.0};
  double final_yaw_align_target_yaw_rad{0.0};
  double final_yaw_align_initial_yaw_error_rad{-1.0};
  double final_yaw_align_final_yaw_error_rad{-1.0};
  double final_yaw_align_max_xy_drift_m{-1.0};
  double final_yaw_align_observed_xy_drift_m{-1.0};
  std::string final_yaw_align_cmd_topic;
  bool final_yaw_align_bypass_collision_monitor{true};
  bool final_pose_verified{false};
  bool task_complete{false};
  bool pre_navigation_undock{false};
  std::string pre_navigation_undock_detail;
  bool pre_navigation_relocalization_requested{false};
  bool pre_navigation_relocalization_succeeded{false};
  std::string pre_navigation_relocalization_detail;
  int final_yaw_align_retry_count{0};
  int reposition_after_yaw_drift_retry_count{0};
  bool ordinary_final_yaw_align_active{false};
  bool predock_yaw_align_active{false};
  bool cmd_owner_conflict_detected{false};
  bool final_yaw_align_blocked_by_docking{false};
  bool docking_blocked_by_final_yaw_align{false};
  std::string final_pose_verify_reason;
  bool cancel_requested{false};
  std::string cancel_reason;
};

struct NavigationGoalJobStartSpec
{
  std::uint64_t id{0U};
  std::string pose_id;
  std::string building_id;
  std::string floor_id;
  std::string goal_completion_policy{"pose_required"};
  bool native_nav2_goal_completion{true};
  bool api_final_yaw_align_enabled{false};
  bool post_nav2_final_verify_enabled{true};
  bool post_nav2_final_verify_wait_bridge_smoothing{true};
  int final_verify_retry_max_count{0};
  bool post_nav2_final_verify_api_velocity_correction_enabled{false};
  bool nav2_rotation_shim_enabled{true};
  double target_x{0.0};
  double target_y{0.0};
  double target_yaw{0.0};
  double nav2_goal_yaw{0.0};
  std::string nav2_goal_yaw_source{"stored_yaw"};
  double final_yaw_align_timeout_sec{-1.0};
  double final_yaw_align_max_xy_drift_m{-1.0};
  std::string final_yaw_align_cmd_topic;
  bool final_yaw_align_bypass_collision_monitor{true};
  bool pre_navigation_undock{false};
  std::string pre_navigation_undock_detail;
  std::string started_at;
};

struct NavigationGoalJobFinishSpec
{
  bool succeeded{false};
  std::string phase;
  std::string detail;
  std::string completed_at;
  double final_distance_m{-1.0};
  double final_yaw_error_rad{-1.0};
  int nav2_result_code{0};
  bool nav2_succeeded{false};
  bool position_reached{false};
  bool final_yaw_align_requested{false};
  bool final_yaw_align_succeeded{false};
  bool final_yaw_align_blocked{false};
};

struct NavigationGoalFinalYawUpdate
{
  bool attempted{false};
  bool succeeded{false};
  bool blocked{false};
  std::string blocked_reason;
  double duration_sec{-1.0};
  double initial_yaw_error_rad{-1.0};
  double final_yaw_error_rad{-1.0};
  double observed_xy_drift_m{-1.0};
};

std::string navigation_goal_job_json(const NavigationGoalJob & job);
NavigationGoalJob make_navigation_goal_job(const NavigationGoalJobStartSpec & start);
void apply_navigation_goal_job_finish(
  NavigationGoalJob & job,
  const NavigationGoalJobFinishSpec & finish);
void apply_navigation_goal_final_pose(
  NavigationGoalJob & job,
  double distance_m,
  double yaw_error_rad,
  bool position_reached,
  const std::string & reason);
void apply_navigation_goal_bridge_wait(
  NavigationGoalJob & job,
  double elapsed_ms,
  bool timeout,
  const std::string & detail);
void apply_navigation_goal_final_yaw(
  NavigationGoalJob & job,
  const NavigationGoalFinalYawUpdate & update);

}  // namespace robot_api_server::features::navigation
