#include "robot_docking_manager/near_field_docking_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace robot_docking_manager
{
namespace
{

double clamp(const double value, const double low, const double high)
{
  return std::min(std::max(value, low), high);
}

double apply_deadband(const double value, const double deadband)
{
  if (std::abs(value) <= deadband) {
    return 0.0;
  }
  return std::copysign(std::abs(value) - deadband, value);
}

double bounded_nonzero(
  const double requested,
  const double minimum,
  const double maximum)
{
  if (std::abs(requested) <= std::numeric_limits<double>::epsilon() || maximum <= 0.0) {
    return 0.0;
  }
  return std::copysign(clamp(std::abs(requested), minimum, maximum), requested);
}

}  // namespace

NearFieldDockingController::NearFieldDockingController(NearFieldControlConfig config)
: config_(std::move(config))
{
  config_.distance_tolerance_m = std::max(0.0, config_.distance_tolerance_m);
  config_.lateral_tolerance_m = std::max(0.0, config_.lateral_tolerance_m);
  config_.yaw_exit_tolerance_rad = std::max(0.0, config_.yaw_exit_tolerance_rad);
  config_.yaw_reentry_threshold_rad = std::max(
    config_.yaw_exit_tolerance_rad, config_.yaw_reentry_threshold_rad);
  config_.yaw_stable_samples = std::max(1, config_.yaw_stable_samples);
  config_.max_yaw_realignments = std::max(0, config_.max_yaw_realignments);
  config_.min_forward_speed_mps = std::max(0.0, config_.min_forward_speed_mps);
  config_.max_forward_speed_mps = std::max(
    config_.min_forward_speed_mps, config_.max_forward_speed_mps);
  config_.min_lateral_speed_mps = std::max(0.0, config_.min_lateral_speed_mps);
  config_.max_lateral_speed_mps = std::max(
    config_.min_lateral_speed_mps, config_.max_lateral_speed_mps);
  config_.min_yaw_speed_radps = std::max(0.0, config_.min_yaw_speed_radps);
  config_.max_yaw_speed_radps = std::max(
    config_.min_yaw_speed_radps, config_.max_yaw_speed_radps);
  config_.max_parallel_speed_mps = std::max(0.0, config_.max_parallel_speed_mps);
  config_.final_approach_window_m = std::max(
    config_.distance_tolerance_m, config_.final_approach_window_m);
  config_.final_lateral_lock_distance_m = clamp(
    config_.final_lateral_lock_distance_m, 0.0, config_.final_approach_window_m);
  config_.final_forward_speed_mps = clamp(
    config_.final_forward_speed_mps, 0.0, config_.max_forward_speed_mps);
  config_.contact_cruise_speed_mps = std::max(1.0e-3, config_.contact_cruise_speed_mps);
  config_.contact_final_zone_m = std::max(0.0, config_.contact_final_zone_m);
  config_.contact_final_speed_mps = std::max(1.0e-3, config_.contact_final_speed_mps);
  config_.contact_timeout_safety_factor = std::max(1.0, config_.contact_timeout_safety_factor);
  config_.contact_timeout_margin_sec = std::max(0.0, config_.contact_timeout_margin_sec);
  config_.contact_timeout_min_sec = std::max(0.0, config_.contact_timeout_min_sec);
  config_.retry_backoff_min_distance_m = std::max(0.0, config_.retry_backoff_min_distance_m);
  config_.retry_backoff_max_distance_m = std::max(
    config_.retry_backoff_min_distance_m, config_.retry_backoff_max_distance_m);
  config_.retry_backoff_clearance_margin_m = std::max(
    0.0, config_.retry_backoff_clearance_margin_m);
  reset();
}

void NearFieldDockingController::reset()
{
  phase_ = NearFieldPhase::YawCapture;
  resume_phase_ = NearFieldPhase::VectorApproach;
  yaw_stable_samples_ = 0;
  yaw_realignments_ = 0;
  last_yaw_sequence_ = 0U;
  have_last_yaw_sequence_ = false;
}

NearFieldDecision NearFieldDockingController::step(const NearFieldObservation & observation)
{
  NearFieldDecision decision;
  decision.phase = phase_;
  decision.yaw_realignments = yaw_realignments_;
  decision.yaw_stable_samples = yaw_stable_samples_;

  if (!observation.valid || !std::isfinite(observation.distance_m) ||
    !std::isfinite(observation.lateral_error_m) ||
    !std::isfinite(observation.yaw_error_rad))
  {
    decision.reason = "invalid_observation";
    return decision;
  }

  const double absolute_yaw = std::abs(observation.yaw_error_rad);
  if ((phase_ == NearFieldPhase::VectorApproach ||
    phase_ == NearFieldPhase::FinalApproach) &&
    absolute_yaw >= config_.yaw_reentry_threshold_rad)
  {
    if (yaw_realignments_ >= config_.max_yaw_realignments) {
      phase_ = NearFieldPhase::AlignmentBlocked;
    } else {
      resume_phase_ = phase_;
      phase_ = NearFieldPhase::YawCapture;
      ++yaw_realignments_;
      yaw_stable_samples_ = 0;
      have_last_yaw_sequence_ = false;
    }
  }

  if (phase_ == NearFieldPhase::AlignmentBlocked) {
    decision.phase = phase_;
    decision.mode = NearFieldMotionMode::Spinning;
    decision.alignment_blocked = true;
    decision.yaw_realignments = yaw_realignments_;
    decision.reason = "yaw_realign_budget_exhausted";
    return decision;
  }

  if (phase_ == NearFieldPhase::YawCapture) {
    if (!have_last_yaw_sequence_ || observation.sequence != last_yaw_sequence_) {
      last_yaw_sequence_ = observation.sequence;
      have_last_yaw_sequence_ = true;
      if (absolute_yaw <= config_.yaw_exit_tolerance_rad) {
        yaw_stable_samples_ = std::min(yaw_stable_samples_ + 1, config_.yaw_stable_samples);
      } else {
        yaw_stable_samples_ = 0;
      }
    }

    if (yaw_stable_samples_ >= config_.yaw_stable_samples) {
      phase_ = resume_phase_;
    } else {
      const double controlled_yaw = apply_deadband(
        observation.yaw_error_rad, config_.yaw_deadband_rad);
      decision.phase = phase_;
      decision.mode = NearFieldMotionMode::Spinning;
      decision.angular_z_radps = bounded_nonzero(
        config_.k_yaw * controlled_yaw,
        config_.min_yaw_speed_radps,
        config_.max_yaw_speed_radps);
      decision.yaw_realignments = yaw_realignments_;
      decision.yaw_stable_samples = yaw_stable_samples_;
      decision.reason = decision.angular_z_radps == 0.0 ? "yaw_settle" : "yaw_capture";
      return decision;
    }
  }

  const double remaining = std::max(0.0, observation.distance_m - config_.target_distance_m);
  if (phase_ == NearFieldPhase::VectorApproach &&
    remaining <= config_.final_approach_window_m)
  {
    phase_ = NearFieldPhase::FinalApproach;
  }

  const bool distance_ok = remaining <= config_.distance_tolerance_m;
  const bool lateral_ok = std::abs(observation.lateral_error_m) <= config_.lateral_tolerance_m;
  // Reaching a translation phase means yaw capture has already succeeded.
  // Keep that result latched until the re-entry threshold above explicitly
  // returns the controller to YawCapture. Rechecking the narrower exit
  // tolerance here creates an actionless gap between the exit and re-entry
  // thresholds.
  const bool yaw_ok =
    phase_ == NearFieldPhase::VectorApproach ||
    phase_ == NearFieldPhase::FinalApproach;
  if (distance_ok && lateral_ok && yaw_ok) {
    decision.phase = phase_;
    decision.mode = NearFieldMotionMode::Parallel;
    decision.enter_contact_verify = true;
    decision.yaw_realignments = yaw_realignments_;
    decision.yaw_stable_samples = yaw_stable_samples_;
    decision.reason = "visual_alignment_complete";
    return decision;
  }

  double forward = 0.0;
  if (!distance_ok && remaining > 0.0) {
    const double maximum = phase_ == NearFieldPhase::FinalApproach ?
      std::min(config_.max_forward_speed_mps, config_.final_forward_speed_mps) :
      config_.max_forward_speed_mps;
    forward = bounded_nonzero(
      config_.k_forward * remaining,
      std::min(config_.min_forward_speed_mps, maximum),
      maximum);
  }

  double lateral = 0.0;
  const bool final_lateral_locked =
    phase_ == NearFieldPhase::FinalApproach &&
    remaining <= config_.final_lateral_lock_distance_m && lateral_ok;
  if (!final_lateral_locked) {
    const double controlled_lateral = apply_deadband(
      observation.lateral_error_m, config_.lateral_deadband_m);
    lateral = bounded_nonzero(
      config_.lateral_command_sign * config_.k_lateral * controlled_lateral,
      config_.min_lateral_speed_mps,
      config_.max_lateral_speed_mps);
  }

  const double vector_speed = std::hypot(forward, lateral);
  if (vector_speed > config_.max_parallel_speed_mps && vector_speed > 0.0) {
    const double scale = config_.max_parallel_speed_mps / vector_speed;
    forward *= scale;
    lateral *= scale;
  }

  if (std::abs(forward) <= std::numeric_limits<double>::epsilon() &&
    std::abs(lateral) <= std::numeric_limits<double>::epsilon())
  {
    phase_ = NearFieldPhase::AlignmentBlocked;
    decision.phase = phase_;
    decision.mode = NearFieldMotionMode::Parallel;
    decision.alignment_blocked = true;
    decision.yaw_realignments = yaw_realignments_;
    decision.yaw_stable_samples = yaw_stable_samples_;
    decision.reason = "no_action_available";
    return decision;
  }

  decision.phase = phase_;
  decision.mode = NearFieldMotionMode::Parallel;
  decision.linear_x_mps = forward;
  decision.linear_y_mps = lateral;
  decision.angular_z_radps = 0.0;
  decision.yaw_realignments = yaw_realignments_;
  decision.yaw_stable_samples = yaw_stable_samples_;
  decision.reason = final_lateral_locked ?
    "final_lateral_lock" :
    (phase_ == NearFieldPhase::FinalApproach ? "final_vector_approach" : "vector_approach");
  return decision;
}

double NearFieldDockingController::contact_timeout_sec(const double contact_distance_m) const
{
  const double distance = std::max(0.0, contact_distance_m);
  const double final_distance = std::min(distance, config_.contact_final_zone_m);
  const double cruise_distance = std::max(0.0, distance - final_distance);
  const double expected_motion_sec =
    cruise_distance / config_.contact_cruise_speed_mps +
    final_distance / config_.contact_final_speed_mps;
  return std::max(
    config_.contact_timeout_min_sec,
    expected_motion_sec * config_.contact_timeout_safety_factor +
    config_.contact_timeout_margin_sec);
}

double NearFieldDockingController::retry_backoff_distance_m(
  const double attempted_contact_distance_m) const
{
  return clamp(
    std::max(0.0, attempted_contact_distance_m) +
    config_.retry_backoff_clearance_margin_m,
    config_.retry_backoff_min_distance_m,
    config_.retry_backoff_max_distance_m);
}

}  // namespace robot_docking_manager
