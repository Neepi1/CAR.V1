#include "robot_api_server/features/mapping/runtime/mapping_start_job.hpp"

#include <sstream>

#include "robot_api_server/infrastructure/http/http_common.hpp"

namespace robot_api_server::features::mapping::runtime
{

std::string mapping_start_job_json(const MappingStartJobSnapshot & job)
{
  std::ostringstream out;
  out << "{\"id\":" << job.id
      << ",\"state\":" << json_string(job.state)
      << ",\"phase\":" << json_string(job.phase)
      << ",\"detail\":" << json_string(job.detail)
      << ",\"navigation_was_active\":" << (job.navigation_was_active ? "true" : "false")
      << ",\"navigation_cancel_ok\":" << (job.navigation_cancel_ok ? "true" : "false")
      << ",\"navigation_stop_ok\":" << (job.navigation_stop_ok ? "true" : "false")
      << ",\"cancel_requested\":" << (job.cancel_requested ? "true" : "false")
      << ",\"mapping_pid\":" << job.mapping_pid
      << ",\"started_at\":" << json_string(job.started_at)
      << ",\"finished_at\":" << json_string(job.finished_at) << "}";
  return out.str();
}

MappingStartJobSnapshot MappingStartJobTracker::snapshot() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return job_;
}

std::string MappingStartJobTracker::json() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return mapping_start_job_json(job_);
}

bool MappingStartJobTracker::running() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return job_.state == "running";
}

std::optional<std::uint64_t> MappingStartJobTracker::begin(
  const bool navigation_was_active,
  const std::string & started_at)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (job_.state == "running") {
    return std::nullopt;
  }

  job_ = MappingStartJobSnapshot{};
  job_.id = ++sequence_;
  job_.state = "running";
  job_.phase = "accepted";
  job_.detail = navigation_was_active ?
    "2D mapping transition accepted; navigation shutdown queued" :
    "2D mapping start accepted";
  job_.navigation_was_active = navigation_was_active;
  job_.started_at = started_at;
  return job_.id;
}

bool MappingStartJobTracker::request_cancel(const std::string & detail)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (job_.state != "running") {
    return false;
  }
  job_.cancel_requested = true;
  job_.detail = detail;
  return true;
}

bool MappingStartJobTracker::cancel_requested(const std::uint64_t job_id) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return job_.id == job_id && job_.cancel_requested;
}

bool MappingStartJobTracker::set_phase(
  const std::uint64_t job_id,
  const std::string & phase,
  const std::string & detail)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (job_.id != job_id || job_.state != "running") {
    return false;
  }
  job_.phase = phase;
  if (!detail.empty()) {
    job_.detail = detail;
  }
  return true;
}

bool MappingStartJobTracker::set_navigation_result(
  const std::uint64_t job_id,
  const bool cancel_ok,
  const bool stop_ok)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (job_.id != job_id || job_.state != "running") {
    return false;
  }
  job_.navigation_cancel_ok = cancel_ok;
  job_.navigation_stop_ok = stop_ok;
  return true;
}

bool MappingStartJobTracker::finish(
  const std::uint64_t job_id,
  const std::string & state,
  const std::string & detail,
  const pid_t mapping_pid,
  const std::string & finished_at)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (job_.id != job_id) {
    return false;
  }
  job_.state = state;
  job_.phase = "finished";
  job_.detail = detail;
  job_.mapping_pid = mapping_pid;
  job_.finished_at = finished_at;
  return true;
}

}  // namespace robot_api_server::features::mapping::runtime
