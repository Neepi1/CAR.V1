#include "robot_api_server/features/docking/predock_alignment/predock_alignment_policy.hpp"

#include <cmath>
#include <iomanip>
#include <sstream>
#include <utility>

namespace robot_api_server::features::docking::predock_alignment
{

PredockAlignmentPolicy::PredockAlignmentPolicy(PredockAlignmentConfig config)
: config_(std::move(config))
{
}

double PredockAlignmentPolicy::normalize_yaw_error(const double yaw_error) const
{
  return std::atan2(std::sin(yaw_error), std::cos(yaw_error));
}

double PredockAlignmentPolicy::expected_staging_yaw(const DockingJob & job) const
{
  return normalize_yaw_error(job.approach_yaw);
}

double PredockAlignmentPolicy::predock_yaw_error(
  const double current_base_yaw,
  const double expected_base_yaw) const
{
  return normalize_yaw_error(expected_base_yaw - current_base_yaw);
}

double PredockAlignmentPolicy::contact_yaw_error(
  const double current_contact_yaw,
  const double expected_contact_yaw) const
{
  return normalize_yaw_error(expected_contact_yaw - current_contact_yaw);
}

PredockPoseVerification PredockAlignmentPolicy::evaluate_pose(
  const DockingJob & job,
  const RobotPoseSnapshot & pose,
  const std::string & pose_error) const
{
  PredockPoseVerification check;
  check.expected_base_yaw = expected_staging_yaw(job);
  check.expected_contact_yaw = check.expected_base_yaw;
  check.pose = pose;
  if (!check.pose.available) {
    check.detail = "no fresh map-frame pose for predock approach check: " + pose_error;
    return check;
  }
  check.pose_available = true;
  check.current_base_yaw = normalize_yaw_error(check.pose.yaw);
  check.contact_frame_available = false;
  check.current_contact_yaw = check.current_base_yaw;

  const double dx = check.pose.x - job.approach_x;
  const double dy = check.pose.y - job.approach_y;
  check.distance_m = std::hypot(dx, dy);
  const double approach_yaw = check.expected_base_yaw;
  check.forward_m = std::cos(approach_yaw) * dx + std::sin(approach_yaw) * dy;
  check.lateral_m = -std::sin(approach_yaw) * dx + std::cos(approach_yaw) * dy;
  check.lateral_abs_m = std::fabs(check.lateral_m);
  check.base_yaw_error_rad =
    std::fabs(predock_yaw_error(check.current_base_yaw, check.expected_base_yaw));
  check.contact_yaw_error_rad =
    std::fabs(contact_yaw_error(check.current_contact_yaw, check.expected_contact_yaw));
  check.xy_ok = check.distance_m <= config_.pose_max_distance_m;
  check.base_yaw_ok = check.base_yaw_error_rad <= config_.yaw_tolerance_rad;
  check.contact_yaw_ok = check.contact_yaw_error_rad <= config_.yaw_tolerance_rad;

  std::ostringstream out;
  out << std::fixed << std::setprecision(3)
      << "predock pose check distance=" << check.distance_m
      << " base_yaw_error=" << check.base_yaw_error_rad
      << " contact_yaw_error=" << check.contact_yaw_error_rad
      << " forward=" << check.forward_m
      << " lateral=" << check.lateral_m
      << " max_distance=" << config_.pose_max_distance_m
      << " forward_capture_min=" << config_.forward_capture_min_m
      << " forward_capture_max=" << config_.forward_capture_max_m
      << " handoff_yaw_max=" << config_.handoff_max_yaw_rad
      << " yaw_tolerance=" << config_.yaw_tolerance_rad
      << " lateral_target=" << config_.lateral_target_m
      << " pose=(" << check.pose.x << "," << check.pose.y << "," << check.pose.yaw << ")"
      << " approach=(" << job.approach_x << "," << job.approach_y << "," << job.approach_yaw << ")"
      << " dock_insertion_yaw_map=" << job.dock_yaw
      << " expected_base_yaw_at_predock=" << check.expected_base_yaw
      << " expected_contact_yaw_at_predock=" << check.expected_contact_yaw
      << " current_base_yaw_map=" << check.current_base_yaw
      << " current_contact_yaw_map=" << check.current_contact_yaw
      << " reverse_yaw_offset_applied=" << (job.reverse_yaw_offset_applied ? "true" : "false")
      << " contact_frame_available=" << (check.contact_frame_available ? "true" : "false")
      << " contact_yaw_source=base_link_aligned_" << job.contact_frame;
  check.detail = out.str();
  return check;
}

bool PredockAlignmentPolicy::pose_inside_handoff_window(
  const PredockPoseVerification & check) const
{
  return check.pose_available &&
    check.xy_ok &&
    check.base_yaw_error_rad <= config_.handoff_max_yaw_rad &&
    check.contact_yaw_error_rad <= config_.handoff_max_yaw_rad;
}

bool PredockAlignmentPolicy::pose_inside_xy_handoff_window(
  const PredockPoseVerification & check) const
{
  return lateral_capture_allowed(check);
}

bool PredockAlignmentPolicy::pose_allows_staging_recovery(
  const PredockPoseVerification & check) const
{
  return pose_inside_xy_handoff_window(check) &&
    check.base_yaw_error_rad <= config_.yaw_hard_fail_rad &&
    check.contact_yaw_error_rad <= config_.yaw_hard_fail_rad;
}

bool PredockAlignmentPolicy::yaw_target_met(const PredockPoseVerification & check) const
{
  return check.pose_available && check.xy_ok && check.base_yaw_ok && check.contact_yaw_ok;
}

bool PredockAlignmentPolicy::yaw_angles_met(const PredockPoseVerification & check) const
{
  return check.pose_available && check.base_yaw_ok && check.contact_yaw_ok;
}

bool PredockAlignmentPolicy::lateral_target_met(const PredockPoseVerification & check) const
{
  return check.pose_available && check.lateral_abs_m <= config_.lateral_target_m;
}

bool PredockAlignmentPolicy::lateral_error_target_met(
  const PredockPoseVerification & check) const
{
  return check.pose_available && check.lateral_abs_m <= config_.lateral_target_m;
}

bool PredockAlignmentPolicy::staging_target_met(const PredockPoseVerification & check) const
{
  return check.pose_available &&
    forward_capture_window_met(check) &&
    yaw_angles_met(check) &&
    lateral_error_target_met(check);
}

bool PredockAlignmentPolicy::forward_capture_window_met(
  const PredockPoseVerification & check) const
{
  return check.pose_available &&
    check.forward_m >= config_.forward_capture_min_m &&
    check.forward_m <= config_.forward_capture_max_m;
}

bool PredockAlignmentPolicy::lateral_capture_allowed(
  const PredockPoseVerification & check) const
{
  return check.pose_available &&
    forward_capture_window_met(check) &&
    check.lateral_abs_m <= config_.lateral_max_correction_m;
}

double PredockAlignmentPolicy::lateral_align_yaw_gate_rad() const
{
  return config_.yaw_tolerance_rad + config_.lateral_yaw_slack_rad;
}

bool PredockAlignmentPolicy::lateral_align_yaw_gate_met(
  const PredockPoseVerification & check) const
{
  const double yaw_gate = lateral_align_yaw_gate_rad();
  return check.pose_available &&
    check.base_yaw_error_rad <= yaw_gate &&
    check.contact_yaw_error_rad <= yaw_gate;
}

void PredockAlignmentPolicy::apply_pose_verification(
  DockingJob & job,
  const PredockPoseVerification & check) const
{
  const bool verified = yaw_target_met(check);
  job.predock_pose_verified = verified;
  job.predock_xy_ok = check.xy_ok;
  job.predock_base_yaw_ok = check.base_yaw_ok;
  job.predock_contact_yaw_ok = check.contact_yaw_ok;
  job.predock_yaw_verified_by_nav2 =
    verified && job.nav_goal_succeeded && !job.predock_yaw_align_attempted;
  job.contact_frame_available = check.contact_frame_available;
  job.predock_distance_m = check.distance_m;
  job.predock_forward_m = check.forward_m;
  job.predock_lateral_m = check.lateral_m;
  job.predock_lateral_abs_m = check.lateral_abs_m;
  job.predock_expected_base_yaw = check.expected_base_yaw;
  job.predock_expected_contact_yaw = check.expected_contact_yaw;
  job.predock_current_base_yaw = check.current_base_yaw;
  job.predock_current_contact_yaw = check.current_contact_yaw;
  job.predock_base_yaw_error_rad = check.base_yaw_error_rad;
  job.predock_contact_yaw_error_rad = check.contact_yaw_error_rad;
  job.detail = check.detail;
}

void PredockAlignmentPolicy::apply_yaw_align_result(
  DockingJob & job,
  const PredockYawAlignResult & result) const
{
  job.predock_yaw_align_attempted = result.attempted;
  job.predock_yaw_align_succeeded = result.attempted && result.succeeded;
  job.predock_yaw_aligned = result.succeeded;
  job.predock_yaw_align_required_actual_spin = result.actual_spin_required;
  job.predock_yaw_align_initial_error_rad = result.initial_error_rad;
  job.predock_yaw_align_final_error_rad = result.final_error_rad;
  job.predock_yaw_align_duration_sec = result.duration_sec;
  job.predock_yaw_align_observed_yaw_motion_rad = result.observed_yaw_motion_rad;
  job.predock_yaw_align_detail = result.detail;
  job.predock_yaw_align_failure_code = result.succeeded ? "" : result.failure_code;
  if (result.attempted) {
    job.predock_yaw_verified_by_nav2 = false;
  }
  if (!result.succeeded) {
    job.last_error_code = result.failure_code;
    job.last_error_detail = result.detail;
  }
  job.detail = result.detail;
}

void PredockAlignmentPolicy::apply_lateral_align_result(
  DockingJob & job,
  const PredockLateralAlignResult & result) const
{
  job.predock_lateral_align_attempted = job.predock_lateral_align_attempted || result.attempted;
  job.predock_lateral_align_succeeded = result.succeeded;
  job.predock_lateral_aligned = result.succeeded;
  job.predock_lateral_align_initial_error_m = result.initial_error_m;
  job.predock_lateral_align_final_error_m = result.final_error_m;
  job.predock_lateral_align_duration_sec = result.duration_sec;
  job.predock_lateral_align_observed_motion_m = result.observed_lateral_motion_m;
  job.predock_lateral_align_detail = result.detail;
  job.predock_lateral_align_failure_code = result.succeeded ? "" : result.failure_code;
  if (!result.succeeded) {
    job.last_error_code = result.failure_code;
    job.last_error_detail = result.detail;
  }
  job.detail = result.detail;
}

void PredockAlignmentPolicy::apply_fine_entry_check(
  DockingJob & job,
  const PredockPoseVerification & check,
  const bool ok,
  const std::string & failure_code,
  const std::string & detail) const
{
  job.fine_entry_checked = true;
  job.fine_entry_ok = ok;
  job.fine_entry_failure_code = failure_code;
  job.fine_entry_detail = detail;
  job.fine_entry_distance_m = check.distance_m;
  job.fine_entry_lateral_m = check.lateral_abs_m;
  job.fine_entry_base_yaw_error_rad = check.base_yaw_error_rad;
  job.fine_entry_contact_yaw_error_rad = check.contact_yaw_error_rad;
  if (!ok) {
    job.last_error_code = failure_code;
    job.last_error_detail = detail;
  }
  job.detail = detail;
}

}  // namespace robot_api_server::features::docking::predock_alignment
