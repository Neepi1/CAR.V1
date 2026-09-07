#include "robot_api_server/features/docking/lifecycle/pre_navigation_undock_module.hpp"

#include <stdexcept>
#include <utility>

#include "robot_api_server/features/docking/lifecycle/docking_status_utils.hpp"

namespace robot_api_server::features::docking
{

PreNavigationUndockModule::PreNavigationUndockModule(
  std::mutex & docking_start_mutex,
  DockingJobStore & docking_job_store,
  PreNavigationUndockConfig config,
  PreNavigationUndockPorts ports)
: docking_start_mutex_(docking_start_mutex),
  docking_job_store_(docking_job_store),
  config_(std::move(config)),
  ports_(std::move(ports))
{
  if (!ports_.clear_teleop_command || !ports_.publish_zero_motion ||
    !ports_.prepare_controlled_undock ||
    !ports_.release_stale_fine_pause || !ports_.join_docking_worker ||
    !ports_.runtime_snapshot || !ports_.ensure_manager_running ||
    !ports_.call_undock_with_charging_retry || !ports_.observe_undock_status ||
    !ports_.set_docking_runtime_state || !ports_.set_docking_identity ||
    !ports_.timestamp_now || !ports_.monotonic_now || !ports_.sleep_for)
  {
    throw std::invalid_argument("PreNavigationUndockModule requires all ports");
  }
}

bool PreNavigationUndockModule::run_if_needed(
  const PreNavigationUndockRequest & request,
  std::string & detail,
  bool & undock_performed)
{
  undock_performed = false;
  if (!request.auto_undock_required) {
    if (request.docking_active_not_docked_block) {
      detail = "docking is active but not docked: " + request.runtime_docking_state;
      return false;
    }
    detail = request.auto_undock_reason;
    return true;
  }

  undock_performed = true;
  if (!request.runtime_state_undocking &&
    !request.docking_status_indicates_undocking &&
    !start(detail, request.charging_contact_at_gate))
  {
    return false;
  }
  return wait_for_completion(detail);
}

bool PreNavigationUndockModule::start(
  std::string & detail,
  const bool charging_contact_at_gate)
{
  ports_.clear_teleop_command();
  ports_.publish_zero_motion();

  std::lock_guard<std::mutex> start_lock(docking_start_mutex_);
  bool takeover_required = false;
  {
    std::lock_guard<std::mutex> lock(docking_job_store_.mutex());
    if (docking_job_store_.job_unsafe().state == "running") {
      if (docking_job_store_.job_unsafe().phase == "undocking") {
        detail = "undocking already active";
        return true;
      }
      takeover_required = true;
    }
  }
  if (takeover_required && !ports_.prepare_controlled_undock(detail)) {
    return false;
  }

  std::string stale_pause_detail;
  if (!ports_.release_stale_fine_pause(
      "pre_navigation_undock_start", stale_pause_detail))
  {
    detail = stale_pause_detail;
    return false;
  }

  ports_.join_docking_worker();

  const auto runtime = ports_.runtime_snapshot();
  const std::string dock_id = runtime.docking_dock_id;
  std::string ensure_detail;
  if (!ports_.ensure_manager_running(ensure_detail)) {
    detail = ensure_detail;
    return false;
  }

  DockingJob next_job;
  next_job.state = "running";
  next_job.phase = "undocking";
  next_job.dock_id = dock_id;
  next_job.detail = "auto_undock_before_navigation";
  next_job.last_status = "undocking before navigation accepted";
  next_job.started_at = ports_.timestamp_now();
  next_job.resume_navigation = true;
  next_job.pending_goal_held_for_post_undock_settle = true;
  next_job.pending_goal_released_after_post_undock_settle = false;
  next_job.api_accepted = true;
  next_job.already_running = false;
  next_job.docking_status_at_request = runtime.docking_status;

  std::uint64_t job_id = 0U;
  {
    std::lock_guard<std::mutex> lock(docking_job_store_.mutex());
    job_id = ++docking_job_store_.sequence_unsafe();
    next_job.id = job_id;
    docking_job_store_.job_unsafe() = next_job;
  }
  ports_.set_docking_runtime_state(
    true, "undocking", "undocking before navigation accepted");
  ports_.set_docking_identity(dock_id);

  std::string service_detail;
  PreNavigationUndockServiceObservation service_observation;
  if (!ports_.call_undock_with_charging_retry(
      service_detail, charging_contact_at_gate, &service_observation))
  {
    const auto after_status = ports_.runtime_snapshot().docking_status;
    {
      std::lock_guard<std::mutex> lock(docking_job_store_.mutex());
      if (docking_job_store_.job_unsafe().id == job_id) {
        docking_job_store_.job_unsafe().api_accepted = false;
        docking_job_store_.job_unsafe().docking_service_called =
          service_observation.service_called;
        docking_job_store_.job_unsafe().docking_service_success =
          service_observation.service_success;
        docking_job_store_.job_unsafe().docking_service_message =
          service_observation.message;
        ports_.observe_undock_status(
          docking_job_store_.job_unsafe(), after_status);
      }
    }
    docking_job_store_.finish(job_id, false, "failed", service_detail);
    detail = service_detail;
    return false;
  }

  const auto after_status = ports_.runtime_snapshot().docking_status;
  {
    std::lock_guard<std::mutex> lock(docking_job_store_.mutex());
    if (docking_job_store_.job_unsafe().id == job_id &&
      docking_job_store_.job_unsafe().state == "running")
    {
      docking_job_store_.job_unsafe().docking_service_called =
        service_observation.service_called;
      docking_job_store_.job_unsafe().docking_service_success =
        service_observation.service_success;
      docking_job_store_.job_unsafe().docking_service_message =
        service_observation.message;
      docking_job_store_.job_unsafe().detail = service_detail;
      docking_job_store_.job_unsafe().last_status = service_detail;
      ports_.observe_undock_status(
        docking_job_store_.job_unsafe(), after_status);
      if (docking_job_store_.job_unsafe().docking_service_success &&
        !docking_job_store_.job_unsafe().undock_started_observed)
      {
        docking_job_store_.job_unsafe().docking_service_warning =
          "service_success_without_undocking_status_observed_yet";
      }
    }
  }
  ports_.set_docking_runtime_state(true, "undocking", service_detail);
  detail = service_detail;
  return true;
}

bool PreNavigationUndockModule::wait_for_completion(std::string & detail)
{
  const double post_undock_wait_sec =
    config_.relocalize_after_success ? config_.relocalize_wait_sec : 0.0;
  const auto deadline = ports_.monotonic_now() +
    std::chrono::duration_cast<std::chrono::steady_clock::duration>(
    std::chrono::duration<double>(
      config_.auto_undock_timeout_sec + post_undock_wait_sec));
  while (ports_.monotonic_now() < deadline) {
    {
      std::lock_guard<std::mutex> lock(docking_job_store_.mutex());
      auto & job = docking_job_store_.job_unsafe();
      if (job.state == "undocked") {
        if (!config_.relocalize_after_success) {
          detail = job.detail.empty() ? "undocked before navigation" : job.detail;
          return true;
        }
        if (job.post_undock_relocalization_requested &&
          job.post_undock_relocalization_succeeded)
        {
          detail = job.detail.empty() ?
            "undocked and relocalized before navigation" : job.detail;
          job.pending_goal_released_after_post_undock_settle = true;
          return true;
        }
        const std::string readiness_detail =
          !job.post_undock_navigation_readiness_detail.empty() ?
          job.post_undock_navigation_readiness_detail :
          job.post_undock_relocalization_detail;
        detail =
          "undocked before navigation; post-undock navigation readiness failed, Nav2 goal not sent: " +
          readiness_detail;
        return false;
      }
      if (job.state == "failed") {
        detail = job.detail.empty() ?
          "undock failed before navigation" : job.detail;
        return false;
      }
      if (job.state == "running" && job.phase == "relocalize_after_undock") {
        detail = job.post_undock_relocalization_detail.empty() ?
          "waiting for post-undock relocalization" :
          job.post_undock_relocalization_detail;
      }
    }

    const auto runtime = ports_.runtime_snapshot();
    if ((runtime.docking_state == "undocked" ||
      docking_status_is_undocked(runtime.docking_status)) &&
      !config_.relocalize_after_success)
    {
      detail = runtime.docking_status.empty() ?
        "undocked before navigation" : runtime.docking_status;
      return true;
    }
    if (runtime.docking_state == "failed" ||
      docking_status_is_undock_failed(runtime.docking_status) ||
      runtime.docking_status.find("undock_rejected") != std::string::npos)
    {
      detail = runtime.docking_status.empty() ?
        "undock failed before navigation" : runtime.docking_status;
      return false;
    }
    ports_.sleep_for(config_.poll_period);
  }
  detail = "timed out waiting for undock before navigation";
  return false;
}

}  // namespace robot_api_server::features::docking
