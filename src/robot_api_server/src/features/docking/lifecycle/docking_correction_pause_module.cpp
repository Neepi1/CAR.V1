#include "robot_api_server/features/docking/lifecycle/docking_correction_pause_module.hpp"

#include <mutex>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace robot_api_server::features::docking
{

DockingCorrectionPauseModule::DockingCorrectionPauseModule(
  DockingCorrectionPauseConfig config,
  DockingJobStore & job_store,
  DockingCorrectionPausePorts ports)
: config_(std::move(config)),
  job_store_(job_store),
  ports_(std::move(ports))
{
  if (!ports_.request_correction_pause || !ports_.bridge_status_snapshot ||
    !ports_.current_robot_pose || !ports_.warn || !ports_.error)
  {
    throw std::invalid_argument("DockingCorrectionPauseModule requires all ports");
  }
}

bool DockingCorrectionPauseModule::phase_owns_fine_pause(
  const std::string & phase)
{
  return phase == "FINE_DOCKING_ENTRY_CHECK" ||
         phase == "FINE_ALIGN" ||
         phase == "fine_docking" ||
         phase == "relocalize_after_fine_docking";
}

bool DockingCorrectionPauseModule::bridge_has_docking_fine_pause(
  const features::localization::BridgeStatusSnapshot & bridge)
{
  return bridge.available &&
         (bridge.map_odom_correction_paused || bridge.map_odom_frozen_due_to_pause) &&
         bridge.correction_pause_reason == "docking_fine";
}

void DockingCorrectionPauseModule::update_job_state(
  const std::uint64_t job_id,
  const bool paused,
  const std::string & reason,
  const std::string & detail)
{
  std::lock_guard<std::mutex> lock(job_store_.mutex());
  auto & job = job_store_.job_unsafe();
  if (job.id != job_id) {
    return;
  }
  job.global_correction_paused = paused;
  job.pause_reason = paused ? reason : "";
  if (!detail.empty()) {
    job.detail = detail;
  }
  const auto pose = ports_.current_robot_pose();
  if (pose.available) {
    job.display_pose_source = paused ? "frozen_map_odom_plus_odom_base" : "map_base";
    job.display_pose_x = pose.x;
    job.display_pose_y = pose.y;
    job.display_pose_yaw = pose.yaw;
    job.display_pose_age_sec = pose.age_sec;
  }
}

bool DockingCorrectionPauseModule::set_paused(
  const std::uint64_t job_id,
  const bool paused,
  const std::string & reason,
  std::string & detail)
{
  const bool ok = ports_.request_correction_pause(
    paused,
    detail,
    config_.service_timeout,
    config_.enabled);
  std::ostringstream out;
  out << (paused ? "pause" : "resume")
      << " global correction reason=" << reason
      << " ok=" << (ok ? "true" : "false")
      << " detail=" << detail;
  update_job_state(job_id, paused && ok, reason, out.str());
  return ok;
}

bool DockingCorrectionPauseModule::release_stale_if_needed(
  const std::string & context,
  std::string & detail)
{
  const auto bridge = ports_.bridge_status_snapshot();
  const bool bridge_has_fine_pause = bridge_has_docking_fine_pause(bridge);
  std::uint64_t job_id = 0U;
  bool job_has_pause = false;
  bool active_fine_owner = false;
  std::string state;
  std::string phase;
  {
    std::lock_guard<std::mutex> lock(job_store_.mutex());
    const auto & job = job_store_.job_unsafe();
    job_id = job.id;
    job_has_pause = job.global_correction_paused;
    state = job.state;
    phase = job.phase;
    active_fine_owner = job.state == "running" &&
      job.global_correction_paused && phase_owns_fine_pause(job.phase);
  }

  if (active_fine_owner) {
    detail = "active docking fine job still owns global correction pause before " + context +
      " state=" + state + " phase=" + phase;
    return false;
  }
  if (!job_has_pause && !bridge_has_fine_pause) {
    detail = "no stale docking_fine correction pause before " + context;
    return true;
  }

  std::string release_detail;
  const bool ok = set_paused(
    job_id,
    false,
    "stale_docking_fine_pause_" + context,
    release_detail);
  std::ostringstream out;
  out << "stale docking_fine correction pause cleanup before " << context
      << " ok=" << (ok ? "true" : "false")
      << " job_had_pause=" << (job_has_pause ? "true" : "false")
      << " bridge_had_pause=" << (bridge_has_fine_pause ? "true" : "false")
      << " state=" << state
      << " phase=" << phase
      << " detail=" << release_detail;
  detail = out.str();
  if (ok) {
    ports_.warn(detail);
  } else {
    ports_.error(detail);
  }
  return ok;
}

}  // namespace robot_api_server::features::docking
