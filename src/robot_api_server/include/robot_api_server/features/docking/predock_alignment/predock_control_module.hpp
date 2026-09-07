#pragma once

#include <cstdint>
#include <functional>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/logger.hpp"

#include "robot_api_server/features/docking/lifecycle/docking_job_store.hpp"
#include "robot_api_server/features/docking/predock_alignment/predock_alignment_policy.hpp"
#include "robot_api_server/features/localization/localization_module.hpp"

namespace robot_api_server::features::docking::predock_alignment
{

struct PredockControlConfig
{
  bool validate_pose_after_relocalization{true};
  bool delegate_staging_motion_to_manager{true};
  bool yaw_align_enabled{true};
  bool yaw_align_fallback_enabled{true};
  double yaw_align_tolerance_rad{0.0698};
  double yaw_align_hard_fail_rad{0.80};
  double yaw_align_timeout_sec{10.0};
  double yaw_align_min_speed_radps{0.05};
  double yaw_align_max_speed_radps{0.20};
  double yaw_align_kp{1.2};
  int yaw_align_success_hold_count{3};
  int yaw_align_period_ms{67};
  int yaw_align_zero_cmd_count{5};
  bool yaw_align_require_actual_spin{true};
  double yaw_align_mode_switch_timeout_sec{2.0};
  double yaw_align_no_yaw_motion_timeout_sec{2.0};
  double yaw_align_motion_epsilon_rad{0.01};

  bool lateral_align_enabled{true};
  double lateral_align_target_m{0.03};
  double lateral_align_max_correction_m{0.25};
  double forward_capture_min_m{-0.40};
  double forward_capture_max_m{0.55};
  int staging_capture_max_cycles{6};
  double lateral_align_timeout_sec{8.0};
  double lateral_align_speed_mps{0.025};
  double lateral_align_kp{0.7};
  int lateral_align_period_ms{67};
  int lateral_align_zero_cmd_count{5};
  double lateral_align_command_sign{-1.0};
  double lateral_align_no_motion_timeout_sec{2.0};
  double lateral_align_motion_epsilon_m{0.005};
  double lateral_align_divergence_epsilon_m{0.015};
  int lateral_align_divergence_count{2};
  bool lateral_align_auto_reverse_on_divergence{true};
  std::string lateral_align_forced_mode{"side_slip"};
  std::string lateral_align_release_mode{"auto"};

  bool fine_entry_require_observation_fresh{true};
  bool fine_entry_require_predock_yaw_aligned{true};
  double fine_entry_max_distance_m{0.35};
  double fine_entry_max_yaw_rad{0.0349};
  double fine_entry_max_lateral_m{0.05};
  bool fine_retry_on_yaw_reject{true};
  bool fine_wait_for_bridge_smoothing_enabled{true};
  int fine_bridge_smoothing_wait_timeout_ms{60000};
  int fine_bridge_smoothing_sample_period_ms{100};
  int fine_bridge_smoothing_stable_samples{3};
  bool fine_bridge_smoothing_zero_cmd_during_wait{true};
  bool pause_global_correction_during_fine{true};

  std::string observation_backend{"target_observation"};
  std::string map_frame{"map"};
  double robot_pose_freshness_sec{0.5};
};

struct PredockMotionModeSnapshot
{
  bool available{false};
  bool actual_available{false};
  bool actual_fresh{false};
  int actual_motion_mode_code{255};
  double age_sec{-1.0};
};

struct PredockObservationSnapshot
{
  double age_sec{-1.0};
  bool usable{false};
  std::string detail;
};

struct PredockAssessment
{
  PredockPoseVerification check;
  bool recovery_allowed{false};
  bool yaw_aligned{false};
};

struct PredockControlPorts
{
  std::function<RobotPoseSnapshot(bool, std::string &)> wait_for_current_pose;
  std::function<RobotPoseSnapshot()> current_pose;
  std::function<bool(std::string &)> safety_motion_hard_blocked;

  // The production adapter arbitrates the shared ordinary-final-yaw versus
  // predock command owner atomically. The bool requests lateral ownership too.
  std::function<bool(bool)> try_acquire_motion_owner;
  std::function<void()> release_motion_owner;
  std::function<void(const std::string &)> request_navigation_goal_cancel;
  std::function<void()> publish_final_yaw_zero_burst;
  std::function<void(const geometry_msgs::msg::Twist &)> publish_command;
  std::function<void(const std::string &)> publish_forced_mode;
  std::function<void()> clear_teleop_command;
  std::function<void()> publish_teleop_zero;

  std::function<PredockMotionModeSnapshot()> motion_mode_snapshot;
  std::function<void()> reset_actual_stop_stability;
  std::function<bool(const std::string &, std::string &)> wait_for_actual_stop;
  std::function<double(double, double)> yaw_stop_threshold_rad;

  std::function<localization::BridgeStatusSnapshot()> bridge_status_snapshot;
  std::function<bool(
      const localization::BridgeStatusSnapshot &,
      const std::string &,
      std::string &)>
  bridge_safe_for_goal_start;
  std::function<bool(
      std::uint64_t,
      bool,
      const std::string &,
      std::string &)>
  set_global_correction_paused;
  std::function<void(bool, const std::string &, const std::string &, bool)>
  set_docking_runtime_state;

  std::function<PredockObservationSnapshot()> observation_snapshot;
  std::function<bool(std::string &)> ensure_docking_manager_running;
  std::function<bool(const std::string &, std::string &)> floor_runtime_operation_blocked;
  std::function<bool(std::string &)> start_fine_docking;
};

// Deep predock module. The executor knows only assessment, post-relocalization
// validation, and the complete fine-handoff transaction. All closed-loop
// control, bridge freeze, retry, evidence projection, and failure
// terminalization stay behind this interface.
class PredockControlModule
{
public:
  PredockControlModule(
    rclcpp::Logger logger,
    PredockControlConfig config,
    PredockAlignmentPolicy & policy,
    DockingJobStore & job_store,
    PredockControlPorts ports);

  PredockAssessment probe_handoff(std::uint64_t job_id, const DockingJob & job);
  PredockAssessment verify_staging_pose(std::uint64_t job_id, const DockingJob & job);
  bool validate_current_pose_near_approach(const DockingJob & job, std::string & detail);

  // Returns true only after /docking/start has been accepted and the shared
  // job has entered FINE_ALIGN. Every false return is already terminalized.
  bool start_fine_docking_handoff(std::uint64_t job_id, const DockingJob & job);

  // Existing hard-stop adapter used by the root safety latch. It only sends
  // zero on /cmd_vel_docking and releases the forced Ranger mode.
  void stop_and_release_motion();

private:
  PredockControlConfig config_;
  rclcpp::Logger logger_;
  PredockAlignmentPolicy & policy_;
  DockingJobStore & job_store_;
  PredockControlPorts ports_;

  PredockPoseVerification evaluate_pose(const DockingJob & job);
  void record_pose(std::uint64_t job_id, const PredockPoseVerification & check);
  void record_yaw(std::uint64_t job_id, const PredockYawAlignResult & result);
  void record_lateral(std::uint64_t job_id, const PredockLateralAlignResult & result);
  void record_fine_entry(
    std::uint64_t job_id,
    const PredockPoseVerification & check,
    bool ok,
    const std::string & failure_code,
    const std::string & detail);

  bool yaw_fallback_allowed() const;
  void publish_yaw_zero_burst();
  void publish_lateral_zero_burst();
  bool acquire_motion_owner(std::uint64_t job_id, bool lateral, std::string & failure_detail);
  void release_motion_owner(std::uint64_t job_id);
  PredockYawAlignResult run_yaw_align(
    std::uint64_t job_id,
    double expected_yaw,
    const PredockPoseVerification & initial_check,
    double success_tolerance_rad = -1.0);
  PredockLateralAlignResult run_lateral_align(
    std::uint64_t job_id,
    const DockingJob & job,
    const PredockPoseVerification & initial_check);
  bool ensure_lateral_alignment(
    std::uint64_t job_id,
    const DockingJob & job,
    PredockPoseVerification & check,
    bool & yaw_aligned,
    const std::string & align_phase,
    const std::string & verify_phase,
    const std::string & align_detail);

  bool bridge_safe_for_fine_entry(
    const localization::BridgeStatusSnapshot & bridge,
    std::string & detail) const;
  bool wait_for_bridge_smoothing(
    std::uint64_t job_id,
    std::string & failure_code,
    std::string & detail);
  void update_bridge_settle_state(
    std::uint64_t job_id,
    bool started,
    bool complete,
    const std::string & failure_code,
    const std::string & detail,
    double duration_sec,
    const localization::BridgeStatusSnapshot & bridge);
  bool evaluate_fine_entry(
    const DockingJob & job,
    bool yaw_aligned,
    PredockPoseVerification & check,
    std::string & failure_code,
    std::string & detail);
};

}  // namespace robot_api_server::features::docking::predock_alignment
