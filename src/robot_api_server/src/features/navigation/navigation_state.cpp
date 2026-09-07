#include "robot_api_server/features/navigation/navigation_state.hpp"

#include <iomanip>
#include <sstream>

#include "robot_api_server/features/navigation/runtime/navigation_bridge_wait.hpp"

namespace robot_api_server::features::navigation
{
namespace
{

BridgeReadinessSnapshot bridge_readiness_snapshot(
  const localization::BridgeStatusSnapshot & bridge)
{
  BridgeReadinessSnapshot snapshot;
  snapshot.available = bridge.available;
  snapshot.has_map_to_odom = bridge.has_map_to_odom;
  snapshot.map_to_odom_publisher_owner = bridge.map_to_odom_publisher_owner;
  snapshot.map_odom_correction_paused = bridge.map_odom_correction_paused;
  snapshot.correction_pause_reason = bridge.correction_pause_reason;
  snapshot.map_odom_frozen_due_to_pause = bridge.map_odom_frozen_due_to_pause;
  snapshot.correction_active = bridge.correction_active;
  snapshot.safe_for_goal_start = bridge.safe_for_goal_start;
  snapshot.current_sequence = bridge.current_sequence;
  snapshot.target_sequence = bridge.target_sequence;
  snapshot.remaining_translation_error_m = bridge.remaining_translation_error_m;
  snapshot.remaining_yaw_error_rad = bridge.remaining_yaw_error_rad;
  snapshot.amcl_input_enabled = bridge.amcl_input_enabled;
  snapshot.amcl_degraded_reason = bridge.amcl_degraded_reason;
  snapshot.amcl_status_source = bridge.amcl_status_source;
  snapshot.amcl_status_age_ms = bridge.amcl_status_age_ms;
  snapshot.amcl_process_ready = bridge.amcl_process_ready;
  snapshot.amcl_seeded = bridge.amcl_seeded;
  snapshot.amcl_nomotion_pose_received = bridge.amcl_nomotion_pose_received;
  snapshot.amcl_static_standby = bridge.amcl_static_standby;
  snapshot.amcl_tracking_ready = bridge.amcl_tracking_ready;
  snapshot.amcl_correction_ready = bridge.amcl_correction_ready;
  snapshot.amcl_correction_pending = bridge.amcl_correction_pending;
  snapshot.amcl_not_moving_no_update_ok = bridge.amcl_not_moving_no_update_ok;
  snapshot.localization_degraded = bridge.localization_degraded;
  return snapshot;
}

bool localization_transition_active(const std::string & detail)
{
  return detail.rfind("LOCALIZATION_TRANSITION_ACTIVE:", 0) == 0;
}

std::string json_nullable_number(const bool valid, const double value)
{
  if (!valid) {
    return "null";
  }
  std::ostringstream output;
  output << std::fixed << std::setprecision(6) << value;
  return output.str();
}

std::string json_string_array(const std::vector<std::string> & values)
{
  std::ostringstream output;
  output << "[";
  for (std::size_t index = 0; index < values.size(); ++index) {
    if (index != 0U) {
      output << ",";
    }
    output << json_string(values[index]);
  }
  output << "]";
  return output.str();
}

}  // namespace

HttpResponse navigation_state_response(
  const NavigationStateConfig & config,
  const NavigationStateSnapshot & snapshot)
{
  const auto & runtime = snapshot.runtime;
  const auto & runtime_context = snapshot.runtime_map_context;
  const auto & dock = snapshot.dock;
  const auto & amcl_status = snapshot.amcl;
  const auto & bridge_status = snapshot.bridge;
  const auto & safety_state = snapshot.safety;

  const bool amcl_file_authoritative = amcl_status.available && !amcl_status.stale;
  const bool bridge_amcl_available = bridge_status.available && bridge_status.amcl_input_enabled;
  const bool effective_amcl_ready =
    bridge_amcl_available ? bridge_status.amcl_ready :
    (amcl_file_authoritative && amcl_status.ready);
  const bool effective_amcl_correction_ready =
    bridge_amcl_available ? bridge_status.amcl_correction_ready :
    (amcl_file_authoritative && amcl_status.correction_ready);
  const bool effective_amcl_correction_pending =
    bridge_amcl_available ? bridge_status.amcl_correction_pending :
    ((amcl_status.available && amcl_status.mode != "disabled") &&
    effective_amcl_ready && !effective_amcl_correction_ready);
  const bool effective_amcl_degraded =
    bridge_amcl_available ? bridge_status.localization_degraded :
    (amcl_file_authoritative && (amcl_status.degraded || !amcl_status.ready));
  const std::string effective_amcl_degraded_reason =
    bridge_amcl_available ?
    (bridge_status.amcl_degraded_reason.empty() ?
      (bridge_status.localization_degraded ? std::string("AMCL_NOT_READY") : std::string()) :
      bridge_status.amcl_degraded_reason) :
    (effective_amcl_degraded ?
      (amcl_status.degraded_reason.empty() ?
        std::string("AMCL_NOT_READY") : amcl_status.degraded_reason) :
      std::string());
  const bool localization_degraded = effective_amcl_degraded;
  const bool using_triggered_baseline_only =
    (bridge_amcl_available || (amcl_status.available && amcl_status.mode != "disabled")) &&
    !effective_amcl_ready;
  const std::string localization_degraded_reason =
    localization_degraded ? effective_amcl_degraded_reason : std::string();
  const auto active_runtime_mode = application::runtime_mode::active_runtime_profile(runtime);
  const auto bridge_decision = evaluate_bridge_readiness(
    bridge_readiness_snapshot(bridge_status),
    BridgeReadinessPurpose::kGoalStart,
    "navigation state");
  const bool localization_recovery_required =
    !bridge_decision.safe && !localization_transition_active(bridge_decision.detail);

  std::ostringstream response;
  response << "{\"ok\":true,"
           << "\"mode\":" << json_string(runtime.mode) << ","
           << "\"active_runtime_mode\":" << json_string(active_runtime_mode) << ","
           << "\"state\":" << json_string(runtime.navigation_state) << ","
           << "\"navigation_active\":" << (runtime.navigation_active ? "true" : "false") << ","
           << "\"healthy\":" << (runtime.healthy ? "true" : "false") << ","
           << "\"message\":" << json_string(runtime.message) << ","
           << "\"runtime_map_context\":";
  if (runtime_context) {
    response << "{"
             << "\"state\":" << json_string(runtime_context->state) << ","
             << "\"confirmed\":" << (runtime_context->confirmed ? "true" : "false") << ","
             << "\"startup_stage\":" << json_string(runtime_context->startup_stage) << ","
             << "\"message\":" << json_string(runtime_context->message) << ","
             << "\"map_id\":" << json_string(runtime_context->map_id) << ","
             << "\"building_id\":" << json_string(runtime_context->building_id) << ","
             << "\"floor_id\":" << json_string(runtime_context->floor_id) << ","
             << "\"updated_at\":" << runtime_context->updated_at_sec
             << "}";
  } else {
    response << "null";
  }
  response << ","
           << "\"localization_degraded\":" << (localization_degraded ? "true" : "false") << ","
           << "\"localization_degraded_reason\":"
           << json_string(localization_degraded_reason) << ","
           << "\"using_triggered_baseline_only\":"
           << (using_triggered_baseline_only ? "true" : "false") << ","
           << "\"safe_for_goal_start\":" << (bridge_decision.safe ? "true" : "false") << ","
           << "\"goal_start_detail\":" << json_string(bridge_decision.detail) << ","
           << "\"correction_active\":"
           << (bridge_status.available && bridge_status.correction_active ? "true" : "false") << ","
           << "\"navigation_normal_path_relocalization_enabled\":false,"
           << "\"docking_normal_path_relocalization_enabled\":false,"
           << "\"force_accept_allowed_in_normal_path\":false,"
           << "\"ordinary_navigation_triggered_relocalization\":false,"
           << "\"docking_predock_triggered_relocalization\":false,"
           << "\"native_nav2_goal_completion\":"
           << (config.nav2_native_goal_completion_enabled ? "true" : "false") << ","
           << "\"api_final_yaw_align_enabled\":"
           << (config.api_final_yaw_align_enabled ? "true" : "false") << ","
           << "\"nav2_rotation_shim_enabled\":"
           << (config.nav2_rotation_shim_enabled ? "true" : "false") << ","
           << "\"localization_recovery_available\":true,"
           << "\"removed_redundant_gates\":["
              "\"normal_pre_goal_force_accept\","
              "\"normal_pre_goal_global_localization_trigger\","
              "\"normal_pre_goal_post_relocalization_settle\","
              "\"docking_default_predock_relocalization\"],"
           << "\"localization_recovery_required\":"
           << (localization_recovery_required ? "true" : "false") << ","
           << "\"amcl_mode\":" << json_string(amcl_status.mode) << ","
           << "\"amcl_state\":" << json_string(amcl_status.state) << ","
           << "\"amcl_start_result\":" << json_string(amcl_status.start_result) << ","
           << "\"amcl_status_file_stale\":" << (amcl_status.stale ? "true" : "false") << ","
           << "\"amcl_status_age_ms\":" << amcl_status.age_ms << ","
           << "\"amcl_status_source\":" << json_string(
    bridge_amcl_available ? bridge_status.amcl_status_source :
    (amcl_file_authoritative ? std::string("file") : std::string("stale_file_ignored"))) << ","
           << "\"amcl_ready\":" << (effective_amcl_ready ? "true" : "false") << ","
           << "\"amcl_degraded\":" << (effective_amcl_degraded ? "true" : "false") << ","
           << "\"amcl_degraded_reason\":" << json_string(effective_amcl_degraded_reason) << ","
           << "\"amcl_process_alive\":" << (amcl_status.process_alive ? "true" : "false") << ","
           << "\"amcl_process_ready\":" << ((
    bridge_amcl_available ? bridge_status.amcl_process_ready : amcl_status.process_ready
  ) ? "true" : "false") << ","
           << "\"amcl_seeded\":" << ((
    bridge_amcl_available ? bridge_status.amcl_seeded : amcl_status.seeded
  ) ? "true" : "false") << ","
           << "\"amcl_seed_response_ok\":" << ((
    bridge_amcl_available ? bridge_status.amcl_seed_response_ok : amcl_status.seed_response_ok
  ) ? "true" : "false") << ","
           << "\"amcl_nomotion_pose_received\":" << ((
    bridge_amcl_available ?
    bridge_status.amcl_nomotion_pose_received : amcl_status.nomotion_pose_received
  ) ? "true" : "false") << ","
           << "\"amcl_static_standby\":" << ((
    bridge_amcl_available ? bridge_status.amcl_static_standby : amcl_status.static_standby
  ) ? "true" : "false") << ","
           << "\"amcl_tracking_ready\":" << ((
    bridge_amcl_available ? bridge_status.amcl_tracking_ready : amcl_status.tracking_ready
  ) ? "true" : "false") << ","
           << "\"amcl_correction_ready\":" << ((
    bridge_amcl_available ? bridge_status.amcl_correction_ready : amcl_status.correction_ready
  ) ? "true" : "false") << ","
           << "\"amcl_correction_pending\":"
           << (effective_amcl_correction_pending ? "true" : "false") << ","
           << "\"amcl_not_moving_no_update_ok\":" << ((
    bridge_amcl_available ?
    bridge_status.amcl_not_moving_no_update_ok : amcl_status.not_moving_no_update_ok
  ) ? "true" : "false") << ","
           << "\"amcl_scan_admission_alive\":"
           << (amcl_status.scan_admission_alive ? "true" : "false") << ","
           << "\"amcl_pose_publisher_count\":" << amcl_status.pose_publisher_count << ","
           << "\"amcl_scan_admission_status_publisher_count\":"
           << amcl_status.scan_admission_status_publisher_count << ","
           << "\"dock_occupancy_state\":" << json_string(dock.occupancy_state) << ","
           << "\"dock_occupancy_evidence\":" << json_string_array(dock.occupancy_evidence) << ","
           << "\"dock_occupancy_reason\":" << json_string(dock.occupancy_reason) << ","
           << "\"charging_session_latched\":"
           << (dock.charging_session_latched ? "true" : "false") << ","
           << "\"charging_session_age_sec\":" << json_nullable_number(
    dock.charging_session_age_sec >= 0.0, dock.charging_session_age_sec) << ","
           << "\"charging_session_last_confirmed_at\":"
           << json_string(dock.charging_session_last_confirmed_at) << ","
           << "\"full_charge_idle_on_dock\":"
           << (dock.full_charge_idle_on_dock ? "true" : "false") << ","
           << "\"pre_navigation_dock_check\":" << dock.pre_navigation_check_json << ","
           << "\"blocked_by_docked_contact\":"
           << (dock.auto_undock_required ? "true" : "false") << ","
           << "\"normal_motion_blocked_reason\":"
           << json_string(safety::normal_motion_blocked_reason(safety_state)) << ","
           << "\"safety\":" << safety::safety_state_json(safety_state) << ","
           << "\"post_relocalization_settle\":"
           << snapshot.post_relocalization_settle_json << ","
           << "\"post_undock_settle\":" << snapshot.post_undock_settle_json << ","
           << "\"navigation_goal\":" << snapshot.navigation_goal_json << ","
           << "\"navigation_cancel\":" << snapshot.navigation_cancel_json << "}";
  return {200, "application/json", response.str()};
}

}  // namespace robot_api_server::features::navigation
