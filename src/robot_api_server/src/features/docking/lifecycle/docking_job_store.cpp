#include "robot_api_server/features/docking/lifecycle/docking_job_store.hpp"

#include <sstream>
#include <utility>

#include "robot_api_server/infrastructure/http/http_common.hpp"

namespace robot_api_server::features::docking
{

DockingJobStore::DockingJobStore(DockingJobStorePorts ports)
: ports_(std::move(ports))
{
}

std::mutex & DockingJobStore::mutex()
{
  return mutex_;
}

DockingJob & DockingJobStore::job_unsafe()
{
  return job_;
}

const DockingJob & DockingJobStore::job_unsafe() const
{
  return job_;
}

std::uint64_t & DockingJobStore::sequence_unsafe()
{
  return sequence_;
}

DockingJob DockingJobStore::snapshot() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return job_;
}

std::string DockingJobStore::job_json_locked() const
{
  return docking_job_json(job_);
}

std::string DockingJobStore::post_undock_settle_json_locked() const
{
  std::ostringstream out;
  out << "{\"post_undock_relocalization_started\":"
      << (job_.post_undock_relocalization_started ? "true" : "false")
      << ",\"post_undock_relocalization_accepted\":"
      << (job_.post_undock_relocalization_accepted ? "true" : "false")
      << ",\"post_undock_settle_started\":"
      << (job_.post_undock_settle_started ? "true" : "false")
      << ",\"post_undock_settle_complete\":"
      << (job_.post_undock_settle_complete ? "true" : "false")
      << ",\"post_undock_settle_failure_reason\":"
      << json_string(job_.post_undock_settle_failure_reason)
      << ",\"post_undock_navigation_readiness_failed\":"
      << (job_.post_undock_navigation_readiness_failed ? "true" : "false")
      << ",\"post_undock_navigation_readiness_failure_code\":"
      << json_string(job_.post_undock_navigation_readiness_failure_code)
      << ",\"post_undock_navigation_readiness_detail\":"
      << json_string(job_.post_undock_navigation_readiness_detail)
      << ",\"pending_goal_held_for_post_undock_settle\":"
      << (job_.pending_goal_held_for_post_undock_settle ? "true" : "false")
      << ",\"pending_goal_released_after_post_undock_settle\":"
      << (job_.pending_goal_released_after_post_undock_settle ? "true" : "false")
      << ",\"using_triggered_baseline_only\":"
      << (job_.using_triggered_baseline_only ? "true" : "false")
      << ",\"amcl_ready\":" << (job_.amcl_ready ? "true" : "false")
      << ",\"localization_degraded\":"
      << (job_.localization_degraded ? "true" : "false")
      << ",\"failure_code\":"
      << json_string(job_.post_undock_navigation_readiness_failure_code.empty() ?
        job_.failure_code : job_.post_undock_navigation_readiness_failure_code)
      << ",\"detail\":" << json_string(job_.detail)
      << "}";
  return out.str();
}

void DockingJobStore::set_phase(
  const std::uint64_t job_id,
  const std::string & phase)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (job_.id == job_id && job_.state == "running") {
    job_.phase = phase;
  }
}

bool DockingJobStore::cancel_requested(const std::uint64_t job_id) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return job_.id == job_id && job_.cancel_requested;
}

void DockingJobStore::mark_navigation_goal_sent(const std::uint64_t job_id)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (job_.id == job_id && job_.state == "running") {
    job_.nav_goal_sent = true;
  }
}

bool DockingJobStore::finish_locked(
  const bool ok,
  const std::string & final_state,
  const std::string & detail)
{
  const bool release_pause_after_finish = job_.global_correction_paused;
  if (ok && (final_state == "docked" || final_state == "charging")) {
    ports_.update_dock_contact_latch(true, "docking_job", detail, job_.dock_id);
  } else if (ok && final_state == "undocked") {
    ports_.update_dock_contact_latch(false, "docking_job", detail, job_.dock_id);
  }
  job_.state = final_state;
  job_.phase = "finished";
  job_.ok = ok;
  job_.detail = detail;
  job_.finished_at = ports_.timestamp_now();
  ports_.finish_runtime(final_state, detail);
  return release_pause_after_finish;
}

void DockingJobStore::finish(
  const std::uint64_t job_id,
  const bool ok,
  const std::string & final_state,
  const std::string & detail)
{
  bool release_pause_after_finish = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (job_.id != job_id) {
      return;
    }
    release_pause_after_finish = finish_locked(ok, final_state, detail);
  }
  if (release_pause_after_finish) {
    std::string release_detail;
    (void)ports_.set_global_correction_paused(
      job_id,
      false,
      "docking_job_finished_" + final_state,
      release_detail);
  }
}

void DockingJobStore::finish_with_code(
  const std::uint64_t job_id,
  const std::string & code,
  const std::string & detail)
{
  bool release_pause_after_finish = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (job_.id != job_id) {
      return;
    }
    job_.failure_code = code;
    job_.last_error_code = code;
    job_.last_error_detail = detail;
    release_pause_after_finish = finish_locked(false, "failed", code + ": " + detail);
  }
  if (release_pause_after_finish) {
    std::string release_detail;
    (void)ports_.set_global_correction_paused(
      job_id,
      false,
      "docking_job_finished_" + code,
      release_detail);
  }
}

}  // namespace robot_api_server::features::docking
