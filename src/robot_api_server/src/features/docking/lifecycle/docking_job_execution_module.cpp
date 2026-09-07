#include "robot_api_server/features/docking/lifecycle/docking_job_execution_module.hpp"

#include <future>
#include <stdexcept>
#include <utility>

namespace robot_api_server::features::docking
{

namespace
{

void require_ports(const DockingJobExecutionModulePorts & ports)
{
  if (!ports.floor_runtime_operation_blocked ||
    !ports.resume_navigation_runtime ||
    !ports.now ||
    !ports.navigation_action_mutex ||
    !ports.navigation_action_client ||
    !ports.track_navigation_goal ||
    !ports.cancel_active_navigation_goal ||
    !ports.request_navigation_goal_cancel ||
    !ports.ordinary_final_yaw_align_active ||
    !ports.record_docking_cmd_owner_conflict ||
    !ports.publish_final_yaw_align_zero_burst ||
    !ports.clear_teleop_command ||
    !ports.publish_teleop_zero_burst ||
    !ports.set_navigation_runtime_state ||
    !ports.set_docking_runtime_state ||
    !ports.bridge_safe_for_goal_start ||
    !ports.trigger_localization_and_wait_for_result ||
    !ports.wait_for_relocalization_settle ||
    !ports.clear_navigation_terminal_speed_limit ||
    !ports.publish_navigation_terminal_speed_limit_for_goal ||
    !ports.clear_navigation_terminal_reverse_permit ||
    !ports.update_navigation_terminal_reverse_permit_for_goal ||
    !ports.bms_charging_contact_snapshot ||
    !ports.reset_terminal_actual_stop_stability ||
    !ports.wait_for_terminal_actual_stop)
  {
    throw std::invalid_argument(
            "docking job execution module requires every runtime port");
  }
}

// A timed-out ROS action submission may still reach its server. Destruction
// without resolve intentionally leaves the process-wide unknown count set.
class PendingSideEffectEvidence
{
public:
  explicit PendingSideEffectEvidence(std::atomic<std::uint64_t> & count)
  : count_(count)
  {
    count_.fetch_add(1U, std::memory_order_acq_rel);
  }

  PendingSideEffectEvidence(const PendingSideEffectEvidence &) = delete;
  PendingSideEffectEvidence & operator=(const PendingSideEffectEvidence &) = delete;

  void resolve() noexcept
  {
    if (!resolved_) {
      count_.fetch_sub(1U, std::memory_order_acq_rel);
      resolved_ = true;
    }
  }

private:
  std::atomic<std::uint64_t> & count_;
  bool resolved_{false};
};

}  // namespace

DockingJobExecutionModule::DockingJobExecutionModule(
  DockingJobExecutionModuleConfig config,
  DockingJobStore & job_store,
  std::atomic<std::uint64_t> & delayed_side_effect_unknown_count,
  DockingJobExecutionModulePorts ports)
: config_(std::move(config)),
  job_store_(job_store),
  delayed_side_effect_unknown_count_(delayed_side_effect_unknown_count),
  ports_(std::move(ports))
{
  require_ports(ports_);
}

std::mutex & DockingJobExecutionModule::docking_job_mutex()
{
  return job_store_.mutex();
}

DockingJob & DockingJobExecutionModule::docking_job_unsafe()
{
  return job_store_.job_unsafe();
}

bool DockingJobExecutionModule::floor_runtime_operation_blocked(
  const std::string & operation,
  std::string & detail,
  std::string * reason_code) const
{
  return ports_.floor_runtime_operation_blocked(operation, detail, reason_code);
}

bool DockingJobExecutionModule::resume_navigation_runtime_for_docking(
  const DockingJob & job,
  std::string & detail)
{
  return ports_.resume_navigation_runtime(job, detail);
}

std::chrono::nanoseconds DockingJobExecutionModule::docking_navigation_start_timeout() const
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(config_.navigation_start_wait_sec));
}

rclcpp::Time DockingJobExecutionModule::docking_goal_stamp() const
{
  return ports_.now();
}

bool DockingJobExecutionModule::send_predock_navigation_goal(
  const NavigateToPose::Goal & goal,
  GoalHandle::SharedPtr & goal_handle,
  std::string & detail)
{
  try {
    std::lock_guard<std::mutex> action_lock(ports_.navigation_action_mutex());
    if (floor_runtime_operation_blocked("docking_predock_goal_submit", detail)) {
      return false;
    }
    PendingSideEffectEvidence pending_side_effect(delayed_side_effect_unknown_count_);
    auto future = ports_.navigation_action_client()->async_send_goal(goal);
    if (future.wait_for(service_timeout()) != std::future_status::ready) {
      detail = "timed out sending predock navigation goal";
      return false;
    }
    goal_handle = future.get();
    pending_side_effect.resolve();
    return true;
  } catch (const std::exception & exc) {
    detail = std::string("exception sending predock goal: ") + exc.what();
    return false;
  } catch (...) {
    detail = "unknown exception sending predock goal";
    return false;
  }
}

bool DockingJobExecutionModule::docking_cancel_requested(const std::uint64_t job_id)
{
  return job_store_.cancel_requested(job_id);
}

void DockingJobExecutionModule::set_docking_job_phase(
  const std::uint64_t job_id,
  const std::string & phase)
{
  job_store_.set_phase(job_id, phase);
}

void DockingJobExecutionModule::finish_docking_job(
  const std::uint64_t job_id,
  const bool ok,
  const std::string & final_state,
  const std::string & detail)
{
  job_store_.finish(job_id, ok, final_state, detail);
}

void DockingJobExecutionModule::finish_docking_job_with_code(
  const std::uint64_t job_id,
  const std::string & code,
  const std::string & detail)
{
  job_store_.finish_with_code(job_id, code, detail);
}

void DockingJobExecutionModule::mark_docking_nav_goal_sent(
  const std::uint64_t job_id,
  const GoalHandle::SharedPtr & goal_handle,
  const std::string & building_id,
  const std::string & floor_id)
{
  ports_.track_navigation_goal(goal_handle, "", building_id, floor_id);
  job_store_.mark_navigation_goal_sent(job_id);
}

bool DockingJobExecutionModule::cancel_active_navigation_goal(std::string & detail)
{
  return ports_.cancel_active_navigation_goal(detail);
}

bool DockingJobExecutionModule::request_navigation_goal_cancel(const std::string & reason)
{
  return ports_.request_navigation_goal_cancel(reason);
}

bool DockingJobExecutionModule::ordinary_final_yaw_align_active()
{
  return ports_.ordinary_final_yaw_align_active();
}

void DockingJobExecutionModule::record_docking_cmd_owner_conflict()
{
  ports_.record_docking_cmd_owner_conflict();
}

void DockingJobExecutionModule::publish_final_yaw_align_zero_burst()
{
  ports_.publish_final_yaw_align_zero_burst();
}

void DockingJobExecutionModule::clear_teleop_command()
{
  ports_.clear_teleop_command();
}

void DockingJobExecutionModule::publish_teleop_zero_burst()
{
  ports_.publish_teleop_zero_burst();
}

void DockingJobExecutionModule::set_navigation_runtime_state(
  const bool active,
  const std::string & state,
  const std::string & message,
  const bool healthy)
{
  ports_.set_navigation_runtime_state(active, state, message, healthy);
}

void DockingJobExecutionModule::set_docking_runtime_state(
  const bool active,
  const std::string & state,
  const std::string & message,
  const bool healthy)
{
  ports_.set_docking_runtime_state(active, state, message, healthy);
}

bool DockingJobExecutionModule::bridge_safe_for_goal_start(
  const std::string & context,
  std::string & detail) const
{
  return ports_.bridge_safe_for_goal_start(context, detail);
}

bool DockingJobExecutionModule::trigger_localization_and_wait_for_result(
  const std::string & reason,
  std::string & detail,
  const double wait_timeout_sec,
  std::uint64_t * accepted_sequence)
{
  return ports_.trigger_localization_and_wait_for_result(
    reason, detail, wait_timeout_sec, accepted_sequence);
}

DockingRelocalizationSettleResult
DockingJobExecutionModule::wait_for_docking_relocalization_settle_barrier(
  const std::uint64_t expected_sequence,
  const std::string & reason,
  const std::string & target_next_stage,
  const std::function<bool(std::string &)> & cancel_requested)
{
  return ports_.wait_for_relocalization_settle(
    expected_sequence, reason, target_next_stage, cancel_requested);
}

void DockingJobExecutionModule::clear_navigation_terminal_speed_limit()
{
  ports_.clear_navigation_terminal_speed_limit();
}

void DockingJobExecutionModule::publish_navigation_terminal_speed_limit_for_goal(
  const StoredPose & target)
{
  ports_.publish_navigation_terminal_speed_limit_for_goal(target);
}

void DockingJobExecutionModule::clear_navigation_terminal_reverse_permit(
  bool & permit_active,
  const std::string & context)
{
  ports_.clear_navigation_terminal_reverse_permit(permit_active, context);
}

void DockingJobExecutionModule::update_navigation_terminal_reverse_permit_for_goal(
  const StoredPose & target,
  bool & permit_active,
  std::chrono::steady_clock::time_point & next_refresh_at,
  const std::string & context)
{
  ports_.update_navigation_terminal_reverse_permit_for_goal(
    target, permit_active, next_refresh_at, context);
}

BmsChargingContactSnapshot DockingJobExecutionModule::bms_charging_contact_snapshot()
{
  return ports_.bms_charging_contact_snapshot();
}

void DockingJobExecutionModule::reset_terminal_actual_stop_stability()
{
  ports_.reset_terminal_actual_stop_stability();
}

bool DockingJobExecutionModule::wait_for_terminal_actual_stop(
  const std::string & context,
  std::string & detail,
  const bool require_dual_ackermann_mode) const
{
  return ports_.wait_for_terminal_actual_stop(
    context, detail, require_dual_ackermann_mode);
}

std::chrono::nanoseconds DockingJobExecutionModule::service_timeout() const
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(config_.service_timeout_sec));
}

}  // namespace robot_api_server::features::docking
