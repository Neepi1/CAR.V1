#include "robot_api_server/features/navigation/mission/navigation_goal_job.hpp"

#include <iomanip>
#include <sstream>

#include "robot_api_server/infrastructure/http/http_common.hpp"

namespace robot_api_server::features::navigation
{

NavigationGoalJob make_navigation_goal_job(const NavigationGoalJobStartSpec & start)
{
  NavigationGoalJob job;
  job.id = start.id;
  job.state = "running";
  job.phase = "accepted";
  job.pose_id = start.pose_id;
  job.building_id = start.building_id;
  job.floor_id = start.floor_id;
  job.goal_completion_policy = start.goal_completion_policy;
  job.native_nav2_goal_completion = start.native_nav2_goal_completion;
  job.api_final_yaw_align_enabled = start.api_final_yaw_align_enabled;
  job.post_nav2_final_verify_enabled = start.post_nav2_final_verify_enabled;
  job.post_nav2_final_verify_wait_bridge_smoothing =
    start.post_nav2_final_verify_wait_bridge_smoothing;
  job.final_verify_retry_max_count = start.final_verify_retry_max_count;
  job.post_nav2_final_verify_api_velocity_correction_enabled =
    start.post_nav2_final_verify_api_velocity_correction_enabled;
  job.nav2_rotation_shim_enabled = start.nav2_rotation_shim_enabled;
  job.target_x = start.target_x;
  job.target_y = start.target_y;
  job.target_yaw = start.target_yaw;
  job.nav2_goal_yaw = start.nav2_goal_yaw;
  job.nav2_goal_yaw_source = start.nav2_goal_yaw_source;
  job.yaw_align_required = start.goal_completion_policy == "pose_required";
  job.final_yaw_align_timeout_sec = start.final_yaw_align_timeout_sec;
  job.final_yaw_align_target_yaw_rad = start.target_yaw;
  job.final_yaw_align_max_xy_drift_m = start.final_yaw_align_max_xy_drift_m;
  job.final_yaw_align_cmd_topic = start.final_yaw_align_cmd_topic;
  job.final_yaw_align_bypass_collision_monitor =
    start.final_yaw_align_bypass_collision_monitor;
  job.pre_navigation_undock = start.pre_navigation_undock;
  job.pre_navigation_undock_detail = start.pre_navigation_undock_detail;
  job.pre_navigation_relocalization_requested = false;
  job.pre_navigation_relocalization_succeeded = false;
  job.pre_navigation_relocalization_detail =
    "normal path relocalization disabled; goal-start readiness will be checked in navigation background job";
  job.detail = start.pre_navigation_undock ?
    "navigation goal accepted; controlled undock queued before Nav2 goal send" :
    "navigation goal accepted; goal-start readiness queued before Nav2 goal send";
  job.started_at = start.started_at;
  return job;
}

void apply_navigation_goal_job_finish(
  NavigationGoalJob & job,
  const NavigationGoalJobFinishSpec & finish)
{
  const bool canceled = finish.phase == "canceled";
  const bool degraded =
    !finish.succeeded && !canceled && finish.phase.rfind("degraded", 0) == 0;
  job.state = canceled ?
    "canceled" : (finish.succeeded ? "succeeded" : (degraded ? "degraded" : "failed"));
  job.phase = finish.phase;
  job.detail = finish.detail;
  job.completed_at = finish.completed_at;
  job.final_distance_m = finish.final_distance_m;
  job.final_yaw_error_rad = finish.final_yaw_error_rad;
  job.nav2_result_code = finish.nav2_result_code;
  job.nav2_succeeded = finish.nav2_succeeded;
  job.position_reached = finish.position_reached;
  job.final_yaw_align_requested = finish.final_yaw_align_requested;
  job.final_yaw_align_succeeded = finish.final_yaw_align_succeeded;
  job.final_yaw_align_blocked = finish.final_yaw_align_blocked;
  job.yaw_align_active = false;
  job.yaw_align_failed =
    finish.final_yaw_align_requested && !finish.final_yaw_align_succeeded && !canceled;
  job.task_complete = finish.succeeded && finish.phase == "final_pose_verified";
  job.final_verify_failure_is_terminal =
    !finish.succeeded && !canceled && finish.phase == "failed_final_pose_verify";
  job.ordinary_final_yaw_align_active = false;
}

void apply_navigation_goal_final_pose(
  NavigationGoalJob & job,
  const double distance_m,
  const double yaw_error_rad,
  const bool position_reached,
  const std::string & reason)
{
  job.final_distance_m = distance_m;
  job.final_yaw_error_rad = yaw_error_rad;
  job.final_verify_xy_error_m = distance_m;
  job.final_verify_yaw_error_rad = yaw_error_rad;
  job.position_reached = position_reached;
  job.final_pose_verify_reason = reason;
}

void apply_navigation_goal_bridge_wait(
  NavigationGoalJob & job,
  const double elapsed_ms,
  const bool timeout,
  const std::string & detail)
{
  job.post_nav2_final_verify_bridge_wait_elapsed_ms = elapsed_ms;
  job.post_nav2_final_verify_bridge_wait_timeout = timeout;
  if (!detail.empty()) {
    job.detail = detail;
  }
}

void apply_navigation_goal_final_yaw(
  NavigationGoalJob & job,
  const NavigationGoalFinalYawUpdate & update)
{
  job.final_yaw_align_attempted = update.attempted;
  job.final_yaw_align_succeeded = update.succeeded;
  job.final_yaw_align_blocked = update.blocked;
  job.final_yaw_align_blocked_reason = update.blocked_reason;
  job.yaw_align_failed = update.blocked && !update.succeeded;
  job.final_yaw_align_duration_sec = update.duration_sec;
  job.final_yaw_align_initial_yaw_error_rad = update.initial_yaw_error_rad;
  job.final_yaw_align_final_yaw_error_rad = update.final_yaw_error_rad;
  job.final_yaw_align_observed_xy_drift_m = update.observed_xy_drift_m;
}

std::string navigation_goal_job_json(const NavigationGoalJob & job)
{
  std::ostringstream out;
  out << std::fixed << std::setprecision(6)
      << "{\"id\":" << job.id
      << ",\"state\":" << json_string(job.state)
      << ",\"phase\":" << json_string(job.phase)
      << ",\"pose_id\":" << json_string(job.pose_id)
      << ",\"building_id\":" << json_string(job.building_id)
      << ",\"floor_id\":" << json_string(job.floor_id)
      << ",\"detail\":" << json_string(job.detail)
      << ",\"started_at\":" << json_string(job.started_at)
      << ",\"completed_at\":" << json_string(job.completed_at)
      << ",\"goal_completion_policy\":" << json_string(job.goal_completion_policy)
      << ",\"native_nav2_goal_completion\":"
      << (job.native_nav2_goal_completion ? "true" : "false")
      << ",\"api_final_yaw_align_enabled\":"
      << (job.api_final_yaw_align_enabled ? "true" : "false")
      << ",\"post_nav2_final_verify_enabled\":"
      << (job.post_nav2_final_verify_enabled ? "true" : "false")
      << ",\"post_nav2_final_verify_wait_bridge_smoothing\":"
      << (job.post_nav2_final_verify_wait_bridge_smoothing ? "true" : "false")
      << ",\"post_nav2_final_verify_bridge_wait_elapsed_ms\":"
      << job.post_nav2_final_verify_bridge_wait_elapsed_ms
      << ",\"post_nav2_final_verify_bridge_wait_timeout\":"
      << (job.post_nav2_final_verify_bridge_wait_timeout ? "true" : "false")
      << ",\"final_verify_retry_count\":" << job.final_verify_retry_count
      << ",\"final_verify_retry_reason\":" << json_string(job.final_verify_retry_reason)
      << ",\"final_verify_retry_max_count\":" << job.final_verify_retry_max_count
      << ",\"final_verify_retry_goal_sent\":"
      << (job.final_verify_retry_goal_sent ? "true" : "false")
      << ",\"final_verify_xy_error_m\":" << job.final_verify_xy_error_m
      << ",\"final_verify_yaw_error_rad\":" << job.final_verify_yaw_error_rad
      << ",\"final_verify_failure_is_terminal\":"
      << (job.final_verify_failure_is_terminal ? "true" : "false")
      << ",\"api_velocity_correction_enabled\":"
      << (job.post_nav2_final_verify_api_velocity_correction_enabled ? "true" : "false")
      << ",\"nav2_rotation_shim_enabled\":"
      << (job.nav2_rotation_shim_enabled ? "true" : "false")
      << ",\"target\":{\"x\":" << job.target_x
      << ",\"y\":" << job.target_y
      << ",\"yaw\":" << job.target_yaw << "}"
      << ",\"nav2_goal_yaw_rad\":" << job.nav2_goal_yaw
      << ",\"nav2_goal_yaw_source\":" << json_string(job.nav2_goal_yaw_source)
      << ",\"final_distance_m\":" << job.final_distance_m
      << ",\"final_yaw_error_rad\":" << job.final_yaw_error_rad
      << ",\"nav2_result_code\":" << job.nav2_result_code
      << ",\"nav2_succeeded\":" << (job.nav2_succeeded ? "true" : "false")
      << ",\"position_reached\":" << (job.position_reached ? "true" : "false")
      << ",\"yaw_align_required\":" << (job.yaw_align_required ? "true" : "false")
      << ",\"yaw_align_active\":" << (job.yaw_align_active ? "true" : "false")
      << ",\"yaw_align_succeeded\":" << (job.final_yaw_align_succeeded ? "true" : "false")
      << ",\"yaw_align_failed\":" << (job.yaw_align_failed ? "true" : "false")
      << ",\"final_yaw_align_requested\":" << (job.final_yaw_align_requested ? "true" : "false")
      << ",\"final_yaw_align_attempted\":" << (job.final_yaw_align_attempted ? "true" : "false")
      << ",\"final_yaw_align_succeeded\":" << (job.final_yaw_align_succeeded ? "true" : "false")
      << ",\"final_yaw_align_blocked\":" << (job.final_yaw_align_blocked ? "true" : "false")
      << ",\"final_yaw_align_blocked_reason\":" << json_string(job.final_yaw_align_blocked_reason)
      << ",\"final_yaw_align_duration_sec\":" << job.final_yaw_align_duration_sec
      << ",\"final_yaw_align_timeout_sec\":" << job.final_yaw_align_timeout_sec
      << ",\"final_yaw_align_target_yaw_rad\":" << job.final_yaw_align_target_yaw_rad
      << ",\"final_yaw_align_initial_yaw_error_rad\":" << job.final_yaw_align_initial_yaw_error_rad
      << ",\"final_yaw_align_final_yaw_error_rad\":" << job.final_yaw_align_final_yaw_error_rad
      << ",\"final_yaw_align_max_xy_drift_m\":" << job.final_yaw_align_max_xy_drift_m
      << ",\"final_yaw_align_observed_xy_drift_m\":" << job.final_yaw_align_observed_xy_drift_m
      << ",\"final_yaw_align_cmd_topic\":" << json_string(job.final_yaw_align_cmd_topic)
      << ",\"final_yaw_align_bypass_collision_monitor\":"
      << (job.final_yaw_align_bypass_collision_monitor ? "true" : "false")
      << ",\"final_pose_verified\":" << (job.final_pose_verified ? "true" : "false")
      << ",\"task_complete\":" << (job.task_complete ? "true" : "false")
      << ",\"pre_navigation_undock\":" << (job.pre_navigation_undock ? "true" : "false")
      << ",\"pre_navigation_undock_detail\":"
      << json_string(job.pre_navigation_undock_detail)
      << ",\"pre_navigation_relocalization_requested\":"
      << (job.pre_navigation_relocalization_requested ? "true" : "false")
      << ",\"pre_navigation_relocalization_succeeded\":"
      << (job.pre_navigation_relocalization_succeeded ? "true" : "false")
      << ",\"pre_navigation_relocalization_detail\":"
      << json_string(job.pre_navigation_relocalization_detail)
      << ",\"final_yaw_align_retry_count\":" << job.final_yaw_align_retry_count
      << ",\"reposition_after_yaw_drift_retry_count\":"
      << job.reposition_after_yaw_drift_retry_count
      << ",\"ordinary_final_yaw_align_active\":"
      << (job.ordinary_final_yaw_align_active ? "true" : "false")
      << ",\"predock_yaw_align_active\":" << (job.predock_yaw_align_active ? "true" : "false")
      << ",\"cmd_owner_conflict_detected\":"
      << (job.cmd_owner_conflict_detected ? "true" : "false")
      << ",\"final_yaw_align_blocked_by_docking\":"
      << (job.final_yaw_align_blocked_by_docking ? "true" : "false")
      << ",\"docking_blocked_by_final_yaw_align\":"
      << (job.docking_blocked_by_final_yaw_align ? "true" : "false")
      << ",\"final_pose_verify_reason\":" << json_string(job.final_pose_verify_reason)
      << "}";
  return out.str();
}

}  // namespace robot_api_server::features::navigation
