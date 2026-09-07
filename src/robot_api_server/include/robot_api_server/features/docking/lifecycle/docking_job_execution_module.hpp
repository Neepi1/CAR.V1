#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <mutex>
#include <string>

#include "robot_api_server/features/docking/lifecycle/docking_job_executor.hpp"
#include "robot_api_server/features/docking/lifecycle/docking_job_store.hpp"

namespace robot_api_server::features::docking
{

struct DockingJobExecutionModuleConfig
{
  double service_timeout_sec{8.0};
  double navigation_start_wait_sec{45.0};
};

struct DockingJobExecutionModulePorts
{
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandle = navigation::NavigationActionRuntime::GoalHandle;
  using ActionClient = navigation::NavigationActionRuntime::Client;

  std::function<bool(
      const std::string &,
      std::string &,
      std::string *)>
  floor_runtime_operation_blocked;
  std::function<bool(const DockingJob &, std::string &)> resume_navigation_runtime;
  std::function<rclcpp::Time()> now;
  std::function<std::mutex & ()> navigation_action_mutex;
  std::function<ActionClient::SharedPtr()> navigation_action_client;
  std::function<void(
      GoalHandle::SharedPtr,
      std::string,
      std::string,
      std::string)>
  track_navigation_goal;

  std::function<bool(std::string &)> cancel_active_navigation_goal;
  std::function<bool(const std::string &)> request_navigation_goal_cancel;
  std::function<bool()> ordinary_final_yaw_align_active;
  std::function<void()> record_docking_cmd_owner_conflict;
  std::function<void()> publish_final_yaw_align_zero_burst;
  std::function<void()> clear_teleop_command;
  std::function<void()> publish_teleop_zero_burst;

  std::function<void(bool, const std::string &, const std::string &, bool)>
  set_navigation_runtime_state;
  std::function<void(bool, const std::string &, const std::string &, bool)>
  set_docking_runtime_state;
  std::function<bool(const std::string &, std::string &)> bridge_safe_for_goal_start;
  std::function<bool(
      const std::string &,
      std::string &,
      double,
      std::uint64_t *)>
  trigger_localization_and_wait_for_result;
  std::function<DockingRelocalizationSettleResult(
      std::uint64_t,
      const std::string &,
      const std::string &,
      const std::function<bool(std::string &)> &)>
  wait_for_relocalization_settle;

  std::function<void()> clear_navigation_terminal_speed_limit;
  std::function<void(const StoredPose &)> publish_navigation_terminal_speed_limit_for_goal;
  std::function<void(bool &, const std::string &)> clear_navigation_terminal_reverse_permit;
  std::function<void(
      const StoredPose &,
      bool &,
      std::chrono::steady_clock::time_point &,
      const std::string &)>
  update_navigation_terminal_reverse_permit_for_goal;
  std::function<BmsChargingContactSnapshot()> bms_charging_contact_snapshot;
  std::function<void()> reset_terminal_actual_stop_stability;
  std::function<bool(const std::string &, std::string &, bool)>
  wait_for_terminal_actual_stop;
};

// Owns the complete concrete adapter between DockingJobExecutor and the
// neighboring runtime modules. DockingJobExecutor retains orchestration; this
// module retains the single job store, Nav2 submission evidence and every
// cross-domain effect needed by that orchestration.
class DockingJobExecutionModule : public DockingJobExecutionPort
{
public:
  DockingJobExecutionModule(
    DockingJobExecutionModuleConfig config,
    DockingJobStore & job_store,
    std::atomic<std::uint64_t> & delayed_side_effect_unknown_count,
    DockingJobExecutionModulePorts ports);

  std::mutex & docking_job_mutex() override;
  DockingJob & docking_job_unsafe() override;
  bool floor_runtime_operation_blocked(
    const std::string & operation,
    std::string & detail,
    std::string * reason_code = nullptr) const override;
  bool resume_navigation_runtime_for_docking(
    const DockingJob & job,
    std::string & detail) override;
  std::chrono::nanoseconds docking_navigation_start_timeout() const override;
  rclcpp::Time docking_goal_stamp() const override;
  bool send_predock_navigation_goal(
    const NavigateToPose::Goal & goal,
    GoalHandle::SharedPtr & goal_handle,
    std::string & detail) override;

  bool docking_cancel_requested(std::uint64_t job_id) override;
  void set_docking_job_phase(std::uint64_t job_id, const std::string & phase) override;
  void finish_docking_job(
    std::uint64_t job_id,
    bool ok,
    const std::string & final_state,
    const std::string & detail) override;
  void finish_docking_job_with_code(
    std::uint64_t job_id,
    const std::string & code,
    const std::string & detail) override;
  void mark_docking_nav_goal_sent(
    std::uint64_t job_id,
    const GoalHandle::SharedPtr & goal_handle,
    const std::string & building_id,
    const std::string & floor_id) override;

  bool cancel_active_navigation_goal(std::string & detail) override;
  bool request_navigation_goal_cancel(const std::string & reason) override;
  bool ordinary_final_yaw_align_active() override;
  void record_docking_cmd_owner_conflict() override;
  void publish_final_yaw_align_zero_burst() override;
  void clear_teleop_command() override;
  void publish_teleop_zero_burst() override;

  void set_navigation_runtime_state(
    bool active,
    const std::string & state,
    const std::string & message = "",
    bool healthy = true) override;
  void set_docking_runtime_state(
    bool active,
    const std::string & state,
    const std::string & message = "",
    bool healthy = true) override;
  bool bridge_safe_for_goal_start(
    const std::string & context,
    std::string & detail) const override;
  bool trigger_localization_and_wait_for_result(
    const std::string & reason,
    std::string & detail,
    double wait_timeout_sec = -1.0,
    std::uint64_t * accepted_sequence = nullptr) override;
  DockingRelocalizationSettleResult wait_for_docking_relocalization_settle_barrier(
    std::uint64_t expected_sequence,
    const std::string & reason,
    const std::string & target_next_stage,
    const std::function<bool(std::string &)> & cancel_requested = {}) override;

  void clear_navigation_terminal_speed_limit() override;
  void publish_navigation_terminal_speed_limit_for_goal(const StoredPose & target) override;
  void clear_navigation_terminal_reverse_permit(
    bool & permit_active,
    const std::string & context) override;
  void update_navigation_terminal_reverse_permit_for_goal(
    const StoredPose & target,
    bool & permit_active,
    std::chrono::steady_clock::time_point & next_refresh_at,
    const std::string & context) override;
  BmsChargingContactSnapshot bms_charging_contact_snapshot() override;
  void reset_terminal_actual_stop_stability() override;
  bool wait_for_terminal_actual_stop(
    const std::string & context,
    std::string & detail,
    bool require_dual_ackermann_mode) const override;

private:
  std::chrono::nanoseconds service_timeout() const;

  DockingJobExecutionModuleConfig config_;
  DockingJobStore & job_store_;
  std::atomic<std::uint64_t> & delayed_side_effect_unknown_count_;
  DockingJobExecutionModulePorts ports_;
};

}  // namespace robot_api_server::features::docking
