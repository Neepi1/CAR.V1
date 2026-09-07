#pragma once

#include <chrono>
#include <cstdint>
#include <functional>
#include <mutex>
#include <string>

#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/time.hpp"

#include "robot_api_server/features/docking/lifecycle/docking_job_model.hpp"
#include "robot_api_server/features/docking/predock_alignment/predock_control_module.hpp"
#include "robot_api_server/features/localization/localization_module.hpp"
#include "robot_api_server/features/maps/poses/poses_io.hpp"
#include "robot_api_server/features/navigation/runtime/navigation_action_runtime.hpp"
#include "robot_api_server/features/power/bms_contact.hpp"

namespace robot_api_server::features::docking
{

struct DockingJobExecutorConfig
{
  bool cancel_active_goal_before_predock{true};
  bool relocalize_before_predock{false};
  bool relocalize_after_predock{false};
  bool relocalize_after_predock_required{false};
  bool validate_predock_pose_after_relocalization{true};
  bool predock_early_handoff_enabled{false};
  double predock_nav_timeout_sec{120.0};
  std::string navigate_to_pose_action{"/navigate_to_pose"};
  std::string predock_behavior_tree;
};

struct DockingRelocalizationSettleResult
{
  bool ok{false};
  std::string failure_code;
  std::string detail;
};

// ROS and neighboring-domain boundary for the complete return-to-dock
// execution state machine. Implementations may observe or invoke existing
// services, but the executor itself never publishes directly to the chassis.
class DockingJobExecutionPort
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandle = navigation::NavigationActionRuntime::GoalHandle;
  virtual ~DockingJobExecutionPort() = default;

  virtual std::mutex & docking_job_mutex() = 0;
  virtual DockingJob & docking_job_unsafe() = 0;
  virtual bool floor_runtime_operation_blocked(
    const std::string & operation,
    std::string & detail,
    std::string * reason_code = nullptr) const = 0;
  virtual bool resume_navigation_runtime_for_docking(
    const DockingJob & job,
    std::string & detail) = 0;
  virtual std::chrono::nanoseconds docking_navigation_start_timeout() const = 0;
  virtual rclcpp::Time docking_goal_stamp() const = 0;
  virtual bool send_predock_navigation_goal(
    const NavigateToPose::Goal & goal,
    GoalHandle::SharedPtr & goal_handle,
    std::string & detail) = 0;

  virtual bool docking_cancel_requested(std::uint64_t job_id) = 0;
  virtual void set_docking_job_phase(std::uint64_t job_id, const std::string & phase) = 0;
  virtual void finish_docking_job(
    std::uint64_t job_id,
    bool ok,
    const std::string & final_state,
    const std::string & detail) = 0;
  virtual void finish_docking_job_with_code(
    std::uint64_t job_id,
    const std::string & code,
    const std::string & detail) = 0;
  virtual void mark_docking_nav_goal_sent(
    std::uint64_t job_id,
    const GoalHandle::SharedPtr & goal_handle,
    const std::string & building_id,
    const std::string & floor_id) = 0;

  virtual bool cancel_active_navigation_goal(std::string & detail) = 0;
  virtual bool request_navigation_goal_cancel(const std::string & reason) = 0;
  virtual bool ordinary_final_yaw_align_active() = 0;
  virtual void record_docking_cmd_owner_conflict() = 0;
  virtual void publish_final_yaw_align_zero_burst() = 0;
  virtual void clear_teleop_command() = 0;
  virtual void publish_teleop_zero_burst() = 0;

  virtual void set_navigation_runtime_state(
    bool active,
    const std::string & state,
    const std::string & message = "",
    bool healthy = true) = 0;
  virtual void set_docking_runtime_state(
    bool active,
    const std::string & state,
    const std::string & message = "",
    bool healthy = true) = 0;
  virtual bool bridge_safe_for_goal_start(
    const std::string & context,
    std::string & detail) const = 0;
  virtual bool trigger_localization_and_wait_for_result(
    const std::string & reason,
    std::string & detail,
    double wait_timeout_sec = -1.0,
    std::uint64_t * accepted_sequence = nullptr) = 0;
  virtual DockingRelocalizationSettleResult wait_for_docking_relocalization_settle_barrier(
    std::uint64_t expected_sequence,
    const std::string & reason,
    const std::string & target_next_stage,
    const std::function<bool(std::string &)> & cancel_requested = {}) = 0;

  virtual void clear_navigation_terminal_speed_limit() = 0;
  virtual void publish_navigation_terminal_speed_limit_for_goal(const StoredPose & target) = 0;
  virtual void clear_navigation_terminal_reverse_permit(
    bool & permit_active,
    const std::string & context) = 0;
  virtual void update_navigation_terminal_reverse_permit_for_goal(
    const StoredPose & target,
    bool & permit_active,
    std::chrono::steady_clock::time_point & next_refresh_at,
    const std::string & context) = 0;
  virtual BmsChargingContactSnapshot bms_charging_contact_snapshot() = 0;
  virtual void reset_terminal_actual_stop_stability() = 0;
  virtual bool wait_for_terminal_actual_stop(
    const std::string & context,
    std::string & detail,
    bool require_dual_ackermann_mode) const = 0;

};

class DockingJobExecutor
{
public:
  DockingJobExecutor(
    DockingJobExecutorConfig config,
    navigation::NavigationActionRuntime & navigation_action_runtime,
    predock_alignment::PredockControlModule & predock_control,
    DockingJobExecutionPort & port);

  void run(std::uint64_t job_id);
  void run_guarded(std::uint64_t job_id);

private:
  DockingJobExecutorConfig config_;
  navigation::NavigationActionRuntime & navigation_action_runtime_;
  predock_alignment::PredockControlModule & predock_control_;
  DockingJobExecutionPort & port_;
};

}  // namespace robot_api_server::features::docking
