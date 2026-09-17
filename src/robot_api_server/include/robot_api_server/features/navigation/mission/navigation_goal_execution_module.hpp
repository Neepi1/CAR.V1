#pragma once

#include <chrono>
#include <cstdint>
#include <functional>
#include <mutex>
#include <optional>
#include <string>

#include "builtin_interfaces/msg/time.hpp"

#include "robot_api_server/features/navigation/mission/navigation_goal_executor.hpp"
#include "robot_api_server/features/navigation/terminal_control/navigation_terminal_runtime_module.hpp"

namespace robot_api_server::features::navigation
{

struct NavigationGoalExecutionConfig
{
  std::string action_name{"/navigate_to_pose"};
  std::chrono::nanoseconds service_timeout{std::chrono::seconds(8)};
  double goal_result_timeout_sec{600.0};
  bool near_goal_stalled_handoff_enabled{false};
  bool final_verify_enabled{true};
  bool final_verify_wait_bridge_smoothing{true};
  bool final_yaw_wait_bridge_smoothing{true};
  bool final_verify_request_amcl_nomotion_update{true};
  double near_goal_stalled_handoff_min_wait_sec{3.0};
  double near_goal_stalled_handoff_stall_sec{1.5};
  double near_goal_stalled_handoff_improvement_epsilon_m{0.02};
  double terminal_recovery_max_distance_m{0.40};
  double terminal_lateral_max_forward_m{0.15};
  bool pause_global_correction_during_final_yaw{true};
  double reposition_after_yaw_drift_timeout_sec{30.0};
};

struct NavigationGoalExecutionPorts
{
  std::function<bool(
      const std::string &,
      std::string &,
      std::string *)>
  floor_runtime_operation_blocked;
  std::function<bool(
      const NavigationPreGoalDockSnapshot &,
      std::string &,
      bool &)>
  undock_before_navigation;
  std::function<bool(const std::string &, std::string &)> bridge_safe_for_goal_start;
  std::function<BridgeReadinessSnapshot()> bridge_readiness_snapshot;
  std::function<bool(
      const std::string &,
      std::string &,
      std::chrono::nanoseconds)>
  request_amcl_nomotion_update;
  std::function<bool(
      bool,
      std::string &,
      std::chrono::nanoseconds,
      bool)>
  request_bridge_correction_pause;
  std::function<RobotPoseSnapshot(bool, std::string &)> robot_pose_snapshot;
  std::function<bool(std::string &)> safety_hard_blocked;
  std::function<bool()> docking_job_running;
  std::function<bool(std::string &)> dock_contact_blocked;
  std::function<bool(std::string &)> cancel_active_goal;
  std::function<bool(
      const NavigationActionRuntime::GoalHandle::SharedPtr &,
      std::string &)>
  cancel_goal_for_handoff;
  std::function<bool(const std::string &)> request_goal_cancel;
  std::function<bool()> navigation_runtime_active;
  std::function<void(
      bool,
      const std::string &,
      const std::string &,
      bool)>
  set_navigation_runtime_state;
  std::function<std::string()> timestamp_now;
  std::function<builtin_interfaces::msg::Time()> goal_stamp;
  std::function<void()> delayed_side_effect_started;
  std::function<void()> delayed_side_effect_resolved;
  std::function<void(const std::string &)> warn;
};

// Owns the complete ordinary-navigation goal execution edge after HTTP
// admission: Nav2 submission/result evidence, commercial final verification,
// bridge settle, bounded retries, deterministic terminal correction, and
// mutual exclusion with predock command ownership.
class NavigationGoalExecutionModule final
  : public NavigationGoalExecutionPort,
  public NavigationBridgeWaitRuntimePort
{
public:
  NavigationGoalExecutionModule(
    NavigationGoalExecutionConfig config,
    NavigationActionRuntime & action_runtime,
    NavigationMissionRuntime & mission_runtime,
    NavigationCompletionPolicy & completion_policy,
    NavigationBridgeWait & bridge_wait,
    NavigationTerminalControl & terminal_control,
    NavigationTerminalRuntimeModule & terminal_runtime,
    NavigationGoalExecutionPorts ports);

  bool try_acquire_predock_motion_owner(bool lateral);
  void release_predock_motion_owner();
  bool ordinary_final_yaw_align_active() const;
  void record_docking_cmd_owner_conflict();
  bool request_goal_cancel(const std::string & reason);

  void clear_navigation_terminal_speed_limit() override;
  void publish_navigation_terminal_speed_limit_for_goal(
    const StoredPose & target) override;
  void clear_navigation_terminal_reverse_permit(
    bool & permit_active,
    const std::string & context) override;
  void update_navigation_terminal_reverse_permit_for_goal(
    const StoredPose & target,
    bool & permit_active,
    std::chrono::steady_clock::time_point & next_refresh_at,
    const std::string & context) override;
  bool navigation_goal_cancel_requested(
    std::uint64_t job_id,
    std::string & detail) override;
  bool floor_runtime_operation_blocked(
    const std::string & operation,
    std::string & detail,
    std::string * reason_code = nullptr) const override;
  bool undock_before_navigation_if_needed(
    const NavigationPreGoalDockSnapshot & dock_check,
    std::string & detail,
    bool & undock_performed) override;
  bool bridge_safe_for_goal_start(
    const std::string & context,
    std::string & detail) const override;
  bool send_initial_navigation_goal_to_nav2(
    std::uint64_t job_id,
    const nav2_msgs::action::NavigateToPose::Goal & goal,
    const std::string & pose_id,
    const std::string & building_id,
    const std::string & floor_id,
    GoalHandle::SharedPtr & goal_handle) override;
  bool cancel_active_navigation_goal(std::string & detail) override;
  void publish_final_yaw_align_zero_burst() override;
  void finish_navigation_goal_job(
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
    bool final_yaw_align_blocked) override;
  bool maybe_navigation_near_goal_stalled_handoff(
    std::uint64_t job_id,
    const StoredPose & target,
    const GoalHandle::SharedPtr & goal_handle,
    std::chrono::steady_clock::time_point wait_started_at,
    std::chrono::steady_clock::time_point & next_check_at,
    std::optional<double> & best_distance_m,
    std::chrono::steady_clock::time_point & last_progress_at,
    const std::string & observing_phase,
    std::string & detail) override;
  NavigationBridgeWaitResult wait_for_bridge_smoothing_before_final_verify(
    std::uint64_t job_id) override;
  FinalPoseCheck verify_navigation_final_pose(
    const StoredPose & target,
    bool require_fresh_pose) override;
  void update_navigation_goal_final_pose_fields(
    std::uint64_t job_id,
    const FinalPoseCheck & check,
    const std::string & reason) override;
  bool post_nav2_terminal_lateral_correction_allowed(
    const StoredPose & target,
    const FinalPoseCheck & check,
    const std::string & goal_completion_policy,
    double & forward_m,
    double & lateral_m,
    std::string & reason) const override;
  bool nav2_failed_near_goal_retry_allowed(
    std::uint64_t job_id,
    const FinalPoseCheck & check,
    const std::string & goal_completion_policy,
    double yaw_align_xy_gate,
    std::string & retry_reason,
    std::string & retry_phase) override;
  NavigationRepositionResult run_post_nav2_final_verify_retry(
    std::uint64_t job_id,
    const StoredPose & target,
    const std::string & retry_reason,
    const std::string & retry_phase) override;
  void update_navigation_goal_final_yaw_fields(
    std::uint64_t job_id,
    const FinalYawAlignResult & result) override;
  FinalYawAlignResult run_final_yaw_align(
    std::uint64_t job_id,
    const StoredPose & target,
    const FinalPoseCheck & initial_check) override;
  bool post_nav2_final_verify_acceptance_slack_allowed(
    std::uint64_t job_id,
    const FinalPoseCheck & check,
    std::string & detail) override;
  bool post_nav2_terminal_actual_stop_confirmed(std::string & detail) const override;
  TerminalLateralCorrectionResult run_post_nav2_terminal_lateral_correction(
    std::uint64_t job_id,
    const StoredPose & target,
    const FinalPoseCheck & initial_check,
    const std::string & goal_completion_policy) override;
  bool post_nav2_final_verify_retry_allowed(
    std::uint64_t job_id,
    const FinalPoseCheck & check,
    const std::string & goal_completion_policy,
    std::string & retry_reason,
    std::string & retry_phase) override;

  NavigationRepositionResult run_reposition_after_yaw_drift(
    std::uint64_t job_id,
    const StoredPose & target);

  TimePoint bridge_wait_now() const override;
  void bridge_wait_sleep_for(std::chrono::milliseconds duration) override;
  bool bridge_wait_cancel_requested(
    std::uint64_t job_id,
    std::string & detail) override;
  void publish_bridge_wait_zero() override;
  BridgeReadinessSnapshot bridge_wait_snapshot() override;
  void request_bridge_wait_nomotion_update(
    const BridgeReadinessSnapshot & bridge,
    std::string & detail) override;

private:
  bool request_amcl_nomotion_update_for_final_verify(
    const BridgeReadinessSnapshot & bridge,
    std::string & detail);
  void update_post_nav2_bridge_wait_fields(
    std::uint64_t job_id,
    const NavigationBridgeWaitResult & result);
  NavigationBridgeWaitResult wait_for_bridge_smoothing_before_final_yaw_align(
    std::uint64_t job_id);
  void finish_navigation_goal_send_failure(
    std::uint64_t job_id,
    const std::string & phase,
    const std::string & detail);

  NavigationGoalExecutionConfig config_;
  NavigationActionRuntime & action_runtime_;
  NavigationMissionRuntime & mission_runtime_;
  NavigationCompletionPolicy & completion_policy_;
  NavigationBridgeWait & bridge_wait_;
  NavigationTerminalControl & terminal_control_;
  NavigationTerminalRuntimeModule & terminal_runtime_;
  NavigationGoalExecutionPorts ports_;

  mutable std::mutex motion_owner_mutex_;
  bool ordinary_final_yaw_align_active_{false};
  bool predock_yaw_align_active_{false};
  bool predock_lateral_align_active_{false};
  bool cmd_owner_conflict_detected_{false};
};

}  // namespace robot_api_server::features::navigation
