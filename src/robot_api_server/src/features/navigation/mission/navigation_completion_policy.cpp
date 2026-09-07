#include "robot_api_server/features/navigation/mission/navigation_completion_policy.hpp"

#include <cmath>
#include <iomanip>
#include <sstream>
#include <utility>

namespace robot_api_server::features::navigation
{
namespace
{

double normalize_angle(double value)
{
  constexpr double kPi = 3.14159265358979323846;
  while (value > kPi) {
    value -= 2.0 * kPi;
  }
  while (value < -kPi) {
    value += 2.0 * kPi;
  }
  return value;
}

}  // namespace

NavigationCompletionPolicy::NavigationCompletionPolicy(NavigationCompletionPolicyConfig config)
: config_(std::move(config))
{
}

FinalPoseCheck NavigationCompletionPolicy::evaluate_final_pose(
  const RobotPoseSnapshot & pose,
  const StoredPose & target,
  const bool require_fresh_pose,
  const std::string & pose_error) const
{
  FinalPoseCheck check;
  check.pose = pose;
  if (!check.pose.available || check.pose.frame_id != config_.map_frame) {
    check.reason = pose_error.empty() ? "no fresh map-frame robot pose" : pose_error;
    return check;
  }
  if (require_fresh_pose && check.pose.age_sec > config_.robot_pose_freshness_sec) {
    check.pose.available = false;
    check.reason = "map-frame robot pose is stale";
    return check;
  }

  check.pose_available = true;
  check.distance_m = std::hypot(check.pose.x - target.x, check.pose.y - target.y);
  check.yaw_error_rad = std::fabs(normalize_angle(target.yaw - check.pose.yaw));
  check.position_reached = check.distance_m <= config_.position_tolerance_m;

  std::ostringstream reason;
  reason << std::fixed << std::setprecision(3)
         << "final distance=" << check.distance_m
         << " position_tolerance=" << config_.position_tolerance_m
         << " yaw_error=" << check.yaw_error_rad
         << " yaw_tolerance=" << config_.yaw_tolerance_rad
         << " yaw_trigger=" << config_.yaw_align_trigger_rad;
  check.reason = reason.str();
  return check;
}

NavigationRetryDecision NavigationCompletionPolicy::final_verify_retry(
  const FinalPoseCheck & check,
  const std::string & goal_completion_policy,
  const int retry_count) const
{
  NavigationRetryDecision decision;
  if (!config_.final_verify_enabled ||
    !config_.retry_uses_same_nav2_goal ||
    !check.pose_available ||
    retry_count >= config_.final_verify_max_retry_count)
  {
    return decision;
  }

  constexpr double kLightOverrunMaxM = 0.12;
  constexpr double kYawLightOverrunMaxRad = 0.15;
  constexpr double kYawRecoveryMaxRad = 0.35;

  if (!check.position_reached &&
    check.distance_m > config_.position_tolerance_m &&
    check.distance_m >= config_.xy_retry_min_error_m &&
    check.distance_m <= config_.terminal_recovery_max_distance_m)
  {
    decision.allowed = true;
    if (check.distance_m <= kLightOverrunMaxM) {
      decision.reason = "xy_light_overrun";
      decision.phase = "retry_nav2_after_final_verify_light_xy_overrun";
    } else {
      decision.reason = "xy_recovery_overrun";
      decision.phase = "recovery_retry_nav2_after_final_verify_xy_error";
    }
    return decision;
  }

  if (goal_completion_policy == "pose_required" &&
    config_.yaw_retry_if_failed &&
    check.position_reached &&
    check.yaw_error_rad > config_.yaw_tolerance_rad &&
    check.yaw_error_rad <= kYawRecoveryMaxRad)
  {
    decision.allowed = true;
    if (check.yaw_error_rad <= kYawLightOverrunMaxRad) {
      decision.reason = "yaw_light_overrun";
      decision.phase = "retry_nav2_after_final_verify_light_yaw_overrun";
    } else {
      decision.reason = "yaw_recovery_overrun";
      decision.phase = "recovery_retry_nav2_after_final_verify_yaw_error";
    }
  }
  return decision;
}

NavigationAcceptanceDecision NavigationCompletionPolicy::final_verify_acceptance_slack(
  const FinalPoseCheck & check,
  const int retry_count) const
{
  NavigationAcceptanceDecision decision;
  if (!config_.final_verify_enabled || !check.pose_available || config_.acceptance_slack_m <= 0.0) {
    return decision;
  }
  if (check.position_reached || check.distance_m <= config_.position_tolerance_m) {
    return decision;
  }

  const double effective_tolerance = config_.position_tolerance_m + config_.acceptance_slack_m;
  if (check.distance_m > effective_tolerance) {
    return decision;
  }
  if (config_.retry_uses_same_nav2_goal && retry_count < config_.final_verify_max_retry_count) {
    return decision;
  }

  std::ostringstream out;
  out << std::fixed << std::setprecision(3)
      << check.reason
      << "; accepted after Nav2 retry within post-Nav2 final verify slack"
      << " effective_position_tolerance=" << effective_tolerance
      << " base_position_tolerance=" << config_.position_tolerance_m
      << " slack=" << config_.acceptance_slack_m
      << " retry_count=" << retry_count << "/" << config_.final_verify_max_retry_count;
  decision.allowed = true;
  decision.detail = out.str();
  return decision;
}

NavigationRetryDecision NavigationCompletionPolicy::nav2_failed_near_goal_retry(
  const FinalPoseCheck & check,
  const std::string & goal_completion_policy,
  const double yaw_align_xy_gate,
  const int retry_count) const
{
  NavigationRetryDecision decision;
  if (!config_.nav2_failed_near_goal_retry_enabled ||
    goal_completion_policy != "pose_required" ||
    !check.pose_available ||
    retry_count >= config_.nav2_failed_near_goal_retry_max_count ||
    check.distance_m <= yaw_align_xy_gate ||
    check.distance_m > config_.terminal_recovery_max_distance_m)
  {
    return decision;
  }
  if (config_.nav2_failed_near_goal_retry_requires_yaw_error &&
    check.yaw_error_rad <= config_.yaw_align_trigger_rad)
  {
    return decision;
  }

  std::ostringstream reason;
  reason << std::fixed << std::setprecision(3)
         << "nav2_failed_near_goal"
         << " distance=" << check.distance_m
         << " yaw_error=" << check.yaw_error_rad
         << " yaw_align_xy_gate=" << yaw_align_xy_gate
         << " max_distance=" << config_.terminal_recovery_max_distance_m
         << " retry_count=" << retry_count
         << "/" << config_.nav2_failed_near_goal_retry_max_count;
  decision.allowed = true;
  decision.reason = reason.str();
  decision.phase = "retry_nav2_after_nav2_failed_near_goal";
  return decision;
}

}  // namespace robot_api_server::features::navigation
