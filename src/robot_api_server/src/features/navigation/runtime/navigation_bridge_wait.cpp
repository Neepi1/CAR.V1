#include "robot_api_server/features/navigation/runtime/navigation_bridge_wait.hpp"

#include <iomanip>
#include <sstream>
#include <utility>

namespace robot_api_server::features::navigation
{
namespace
{

bool localization_degraded_blocks_goal_start(const BridgeReadinessSnapshot & bridge)
{
  if (!bridge.localization_degraded) {
    return false;
  }
  const bool static_standby_ok =
    bridge.has_map_to_odom &&
    bridge.safe_for_goal_start &&
    bridge.amcl_seeded &&
    bridge.amcl_static_standby &&
    bridge.amcl_not_moving_no_update_ok &&
    bridge.amcl_tracking_ready &&
    !bridge.amcl_correction_pending;
  return !static_standby_ok;
}

double elapsed_ms(
  const NavigationBridgeWaitRuntimePort::TimePoint started,
  NavigationBridgeWaitRuntimePort & runtime)
{
  return std::chrono::duration<double, std::milli>(
    runtime.bridge_wait_now() - started).count();
}

}  // namespace

BridgeReadinessDecision evaluate_bridge_readiness(
  const BridgeReadinessSnapshot & bridge,
  const BridgeReadinessPurpose purpose,
  const std::string & context)
{
  BridgeReadinessDecision decision;
  if (!bridge.available) {
    decision.detail = "LOCALIZATION_DEGRADED: bridge_status unavailable before " + context;
    return decision;
  }
  if (!bridge.has_map_to_odom ||
    bridge.map_to_odom_publisher_owner != "robot_localization_bridge")
  {
    decision.detail = "LOCALIZATION_DEGRADED: bridge map->odom not ready before " + context;
    return decision;
  }
  if (localization_degraded_blocks_goal_start(bridge)) {
    decision.detail = "LOCALIZATION_DEGRADED: " +
      (bridge.amcl_degraded_reason.empty() ?
      std::string("bridge localization degraded") : bridge.amcl_degraded_reason);
    return decision;
  }
  if (bridge.map_odom_correction_paused || bridge.map_odom_frozen_due_to_pause) {
    std::ostringstream out;
    if (purpose == BridgeReadinessPurpose::kFinalPoseVerify) {
      out << std::fixed << std::setprecision(3);
    }
    out << "LOCALIZATION_TRANSITION_ACTIVE: bridge correction paused before "
        << context
        << " pause_reason=" << bridge.correction_pause_reason
        << " frozen_due_to_pause="
        << (bridge.map_odom_frozen_due_to_pause ? "true" : "false");
    decision.detail = out.str();
    return decision;
  }

  const bool amcl_static_pending_is_standby =
    bridge.amcl_static_standby && bridge.amcl_not_moving_no_update_ok;
  if (bridge.amcl_input_enabled && bridge.amcl_correction_pending &&
    !amcl_static_pending_is_standby)
  {
    std::ostringstream out;
    out << std::fixed << std::setprecision(3)
        << "LOCALIZATION_TRANSITION_ACTIVE: AMCL correction pending before "
        << context
        << " correction_ready=" << (bridge.amcl_correction_ready ? "true" : "false")
        << " static_standby=" << (bridge.amcl_static_standby ? "true" : "false")
        << " tracking_ready=" << (bridge.amcl_tracking_ready ? "true" : "false");
    if (purpose == BridgeReadinessPurpose::kFinalPoseVerify) {
      out << " nomotion_pose_received="
          << (bridge.amcl_nomotion_pose_received ? "true" : "false");
    }
    out << " status_source=" << bridge.amcl_status_source
        << " status_age_ms=" << bridge.amcl_status_age_ms;
    decision.detail = out.str();
    return decision;
  }
  if (bridge.amcl_input_enabled && bridge.amcl_tracking_ready &&
    !bridge.amcl_correction_ready && !bridge.amcl_static_standby)
  {
    std::ostringstream out;
    out << std::fixed << std::setprecision(3)
        << "LOCALIZATION_DEGRADED: AMCL correction not ready before "
        << context
        << " static_standby=false"
        << " status_source=" << bridge.amcl_status_source
        << " status_age_ms=" << bridge.amcl_status_age_ms;
    decision.detail = out.str();
    return decision;
  }

  if (purpose == BridgeReadinessPurpose::kFinalPoseVerify) {
    if (!bridge.safe_for_goal_start || bridge.correction_active) {
      std::ostringstream out;
      out << std::fixed << std::setprecision(3)
          << "LOCALIZATION_TRANSITION_ACTIVE: bridge correction smoothing active before "
          << context
          << " safe_for_goal_start=" << (bridge.safe_for_goal_start ? "true" : "false")
          << " correction_active=" << (bridge.correction_active ? "true" : "false")
          << " remaining_translation=" << bridge.remaining_translation_error_m
          << " remaining_yaw=" << bridge.remaining_yaw_error_rad
          << " current_sequence=" << bridge.current_sequence
          << " target_sequence=" << bridge.target_sequence;
      decision.detail = out.str();
      return decision;
    }
    decision.safe = true;
    decision.detail = "bridge final pose verify safe before " + context;
    if (bridge.amcl_input_enabled && bridge.amcl_correction_pending &&
      amcl_static_pending_is_standby)
    {
      decision.detail += "; AMCL static standby pending tolerated for final pose verify";
    }
    return decision;
  }

  if (!bridge.safe_for_goal_start) {
    std::ostringstream out;
    out << std::fixed << std::setprecision(3)
        << "LOCALIZATION_TRANSITION_ACTIVE: bridge correction smoothing active before "
        << context
        << " remaining_translation=" << bridge.remaining_translation_error_m
        << " remaining_yaw=" << bridge.remaining_yaw_error_rad
        << " current_sequence=" << bridge.current_sequence
        << " target_sequence=" << bridge.target_sequence;
    decision.detail = out.str();
    return decision;
  }
  decision.safe = true;
  decision.detail = "bridge safe_for_goal_start=true before " + context;
  return decision;
}

NavigationBridgeWait::NavigationBridgeWait(NavigationBridgeWaitConfig config)
: config_(std::move(config))
{
}

NavigationBridgeWaitResult NavigationBridgeWait::wait_before_final_verify(
  const std::uint64_t job_id,
  NavigationBridgeWaitRuntimePort & runtime) const
{
  NavigationBridgeWaitResult result;
  if (!config_.final_verify_enabled || !config_.final_verify_wait_enabled) {
    result.detail = "post-Nav2 final verification bridge smoothing wait disabled";
    return result;
  }

  result.waited = true;
  const auto started = runtime.bridge_wait_now();
  const auto deadline = started + config_.final_verify_timeout;
  std::string last_detail;
  std::string nomotion_detail;
  runtime.publish_bridge_wait_zero();
  const auto initial_bridge = runtime.bridge_wait_snapshot();
  runtime.request_bridge_wait_nomotion_update(initial_bridge, nomotion_detail);

  while (runtime.bridge_wait_now() <= deadline) {
    std::string cancel_detail;
    if (runtime.bridge_wait_cancel_requested(job_id, cancel_detail)) {
      result.canceled = true;
      result.detail =
        "navigation goal canceled during post-Nav2 bridge smoothing wait: " + cancel_detail;
      result.elapsed_ms = elapsed_ms(started, runtime);
      return result;
    }

    runtime.publish_bridge_wait_zero();
    const auto bridge = runtime.bridge_wait_snapshot();
    const auto readiness = evaluate_bridge_readiness(
      bridge, BridgeReadinessPurpose::kFinalPoseVerify,
      "post-Nav2 final verification");
    if (readiness.safe) {
      result.elapsed_ms = elapsed_ms(started, runtime);
      std::ostringstream out;
      out << std::fixed << std::setprecision(3)
          << "bridge smoothing settled before post-Nav2 final pose verify"
          << " elapsed_ms=" << result.elapsed_ms
          << "; " << readiness.detail;
      if (!nomotion_detail.empty()) {
        out << "; " << nomotion_detail;
      }
      result.detail = out.str();
      return result;
    }
    last_detail = readiness.detail;
    runtime.bridge_wait_sleep_for(config_.final_verify_sample_period);
  }

  result.timeout = true;
  result.elapsed_ms = elapsed_ms(started, runtime);
  std::ostringstream out;
  out << std::fixed << std::setprecision(3)
      << "bridge smoothing wait timed out before post-Nav2 final pose verify"
      << " timeout_ms=" << config_.final_verify_timeout.count()
      << " elapsed_ms=" << result.elapsed_ms
      << " last_detail=" << last_detail;
  if (!nomotion_detail.empty()) {
    out << " nomotion_detail=" << nomotion_detail;
  }
  result.detail = out.str();
  return result;
}

NavigationBridgeWaitResult NavigationBridgeWait::wait_before_final_yaw_align(
  const std::uint64_t job_id,
  NavigationBridgeWaitRuntimePort & runtime) const
{
  NavigationBridgeWaitResult result;
  if (!config_.final_yaw_wait_enabled) {
    result.detail = "final yaw alignment bridge smoothing wait disabled";
    return result;
  }

  result.waited = true;
  const auto started = runtime.bridge_wait_now();
  const auto deadline = started + config_.final_yaw_timeout;
  std::string last_detail;
  while (runtime.bridge_wait_now() <= deadline) {
    std::string cancel_detail;
    if (runtime.bridge_wait_cancel_requested(job_id, cancel_detail)) {
      result.canceled = true;
      result.detail =
        "navigation goal canceled during final yaw bridge smoothing wait: " + cancel_detail;
      result.elapsed_ms = elapsed_ms(started, runtime);
      return result;
    }

    runtime.publish_bridge_wait_zero();
    const auto bridge = runtime.bridge_wait_snapshot();
    const auto readiness = evaluate_bridge_readiness(
      bridge, BridgeReadinessPurpose::kGoalStart, "final yaw alignment");
    if (readiness.safe) {
      result.elapsed_ms = elapsed_ms(started, runtime);
      std::ostringstream out;
      out << std::fixed << std::setprecision(3)
          << "bridge smoothing settled before final yaw alignment"
          << " elapsed_ms=" << result.elapsed_ms
          << "; " << readiness.detail;
      result.detail = out.str();
      return result;
    }
    last_detail = readiness.detail;
    runtime.bridge_wait_sleep_for(config_.final_yaw_sample_period);
  }

  result.timeout = true;
  result.elapsed_ms = elapsed_ms(started, runtime);
  std::ostringstream out;
  out << std::fixed << std::setprecision(3)
      << "bridge smoothing wait timed out before final yaw alignment"
      << " timeout_ms=" << config_.final_yaw_timeout.count()
      << " elapsed_ms=" << result.elapsed_ms
      << " last_detail=" << last_detail;
  result.detail = out.str();
  return result;
}

}  // namespace robot_api_server::features::navigation
