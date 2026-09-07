#pragma once

#include <chrono>
#include <cstdint>
#include <optional>
#include <string>

#include "robot_api_server/features/navigation/mission/navigation_completion_policy.hpp"
#include "robot_api_server/features/navigation/mission/navigation_mission_runtime.hpp"
#include "robot_api_server/features/navigation/runtime/navigation_action_runtime.hpp"
#include "robot_api_server/features/navigation/runtime/navigation_bridge_wait.hpp"
#include "robot_api_server/features/navigation/terminal_control/navigation_terminal_control.hpp"

namespace robot_api_server::features::navigation
{

struct NavigationPreGoalDockSnapshot;

struct NavigationRepositionResult
{
  bool attempted{false};
  bool succeeded{false};
  bool canceled{false};
  bool near_goal_stalled_handoff{false};
  bool nav2_succeeded{false};
  int nav2_result_code{0};
  std::string detail;
};

struct NavigationGoalExecutorConfig
{
  bool api_final_yaw_align_fallback_enabled{true};
  bool navigation_final_yaw_align_enabled{true};
  bool navigation_nav2_failed_near_goal_retry_enabled{true};
  double navigation_goal_result_timeout_sec{300.0};
  double navigation_goal_position_success_tolerance_m{0.06};
  double navigation_final_yaw_align_max_xy_drift_m{0.08};
  double navigation_final_yaw_align_trigger_rad{0.08};
  double navigation_final_yaw_align_timeout_sec{8.0};
  double navigation_final_yaw_tolerance_rad{0.10};
  double navigation_final_yaw_align_success_tolerance_rad{0.045};
  double navigation_terminal_recovery_max_distance_m{0.35};
  double post_nav2_final_verify_terminal_lateral_target_m{0.03};
  int post_nav2_final_verify_max_retry_count{3};
  std::chrono::nanoseconds service_timeout{std::chrono::seconds(8)};
};

class NavigationGoalExecutionPort
{
public:
  using GoalHandle = NavigationActionRuntime::GoalHandle;

  virtual ~NavigationGoalExecutionPort() = default;

  virtual void clear_navigation_terminal_speed_limit() = 0;
  virtual void publish_navigation_terminal_speed_limit_for_goal(
    const StoredPose & target) = 0;
  virtual void clear_navigation_terminal_reverse_permit(
    bool & permit_active,
    const std::string & context) = 0;
  virtual void update_navigation_terminal_reverse_permit_for_goal(
    const StoredPose & target,
    bool & permit_active,
    std::chrono::steady_clock::time_point & next_refresh_at,
    const std::string & context) = 0;
  virtual bool navigation_goal_cancel_requested(
    std::uint64_t job_id,
    std::string & detail) = 0;
  virtual bool floor_runtime_operation_blocked(
    const std::string & operation,
    std::string & detail,
    std::string * reason_code = nullptr) const = 0;
  virtual bool undock_before_navigation_if_needed(
    const NavigationPreGoalDockSnapshot & dock_check,
    std::string & detail,
    bool & undock_performed) = 0;
  virtual bool bridge_safe_for_goal_start(
    const std::string & context,
    std::string & detail) const = 0;
  virtual bool send_initial_navigation_goal_to_nav2(
    std::uint64_t job_id,
    const nav2_msgs::action::NavigateToPose::Goal & goal,
    const std::string & pose_id,
    const std::string & building_id,
    const std::string & floor_id,
    GoalHandle::SharedPtr & goal_handle) = 0;
  virtual bool cancel_active_navigation_goal(std::string & detail) = 0;
  virtual void publish_final_yaw_align_zero_burst() = 0;
  virtual void finish_navigation_goal_job(
    std::uint64_t job_id,
    bool succeeded,
    const std::string & phase,
    const std::string & detail,
    double final_distance,
    double final_yaw_error,
    int nav2_result_code,
    bool nav2_succeeded,
    bool position_reached,
    bool final_yaw_align_requested,
    bool final_yaw_align_succeeded,
    bool final_yaw_align_blocked) = 0;
  virtual bool maybe_navigation_near_goal_stalled_handoff(
    std::uint64_t job_id,
    const StoredPose & target,
    const GoalHandle::SharedPtr & goal_handle,
    std::chrono::steady_clock::time_point wait_started_at,
    std::chrono::steady_clock::time_point & next_check_at,
    std::optional<double> & best_distance_m,
    std::chrono::steady_clock::time_point & last_progress_at,
    const std::string & observing_phase,
    std::string & detail) = 0;
  virtual NavigationBridgeWaitResult wait_for_bridge_smoothing_before_final_verify(
    std::uint64_t job_id) = 0;
  virtual FinalPoseCheck verify_navigation_final_pose(
    const StoredPose & target,
    bool require_fresh_pose) = 0;
  virtual void update_navigation_goal_final_pose_fields(
    std::uint64_t job_id,
    const FinalPoseCheck & check,
    const std::string & reason) = 0;
  virtual bool post_nav2_terminal_lateral_correction_allowed(
    const StoredPose & target,
    const FinalPoseCheck & check,
    const std::string & goal_completion_policy,
    double & forward_m,
    double & lateral_m,
    std::string & reason) const = 0;
  virtual bool nav2_failed_near_goal_retry_allowed(
    std::uint64_t job_id,
    const FinalPoseCheck & check,
    const std::string & goal_completion_policy,
    double yaw_align_xy_gate,
    std::string & retry_reason,
    std::string & retry_phase) = 0;
  virtual NavigationRepositionResult run_post_nav2_final_verify_retry(
    std::uint64_t job_id,
    const StoredPose & target,
    const std::string & retry_reason,
    const std::string & retry_phase) = 0;
  virtual void update_navigation_goal_final_yaw_fields(
    std::uint64_t job_id,
    const FinalYawAlignResult & result) = 0;
  virtual FinalYawAlignResult run_final_yaw_align(
    std::uint64_t job_id,
    const StoredPose & target,
    const FinalPoseCheck & initial_check) = 0;
  virtual bool post_nav2_final_verify_acceptance_slack_allowed(
    std::uint64_t job_id,
    const FinalPoseCheck & check,
    std::string & detail) = 0;
  virtual TerminalLateralCorrectionResult run_post_nav2_terminal_lateral_correction(
    std::uint64_t job_id,
    const StoredPose & target,
    const FinalPoseCheck & initial_check,
    const std::string & goal_completion_policy) = 0;
  virtual bool post_nav2_final_verify_retry_allowed(
    std::uint64_t job_id,
    const FinalPoseCheck & check,
    const std::string & goal_completion_policy,
    std::string & retry_reason,
    std::string & retry_phase) = 0;
};

class NavigationGoalExecutor
{
public:
  NavigationGoalExecutor(
    NavigationGoalExecutorConfig config,
    NavigationActionRuntime & action_runtime,
    NavigationMissionRuntime & mission_runtime,
    NavigationGoalExecutionPort & port);

  void run(
    std::uint64_t job_id,
    const NavigationActionRuntime::GoalHandle::SharedPtr & goal_handle,
    const StoredPose & target);

  void run_guarded(
    std::uint64_t job_id,
    const nav2_msgs::action::NavigateToPose::Goal & goal,
    const StoredPose & target,
    const std::string & pose_id,
    const std::string & building_id,
    const std::string & floor_id,
    const NavigationPreGoalDockSnapshot & dock_snapshot);

private:
  void finish_send_failure(
    std::uint64_t job_id,
    const std::string & phase,
    const std::string & detail);
  bool wait_for_goal_start_readiness(std::uint64_t job_id, std::string & detail);
  bool run_pre_send_sequence(
    std::uint64_t job_id,
    const NavigationPreGoalDockSnapshot & dock_snapshot);

  NavigationGoalExecutorConfig config_;
  NavigationActionRuntime & action_runtime_;
  NavigationMissionRuntime & mission_runtime_;
  NavigationGoalExecutionPort & port_;
};

}  // namespace robot_api_server::features::navigation
