#include "robot_api_server/features/system_status/system_status_module.hpp"

#include <cmath>
#include <mutex>
#include <sstream>
#include <utility>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

namespace robot_api_server::features::system_status
{
namespace
{

constexpr const char * kUnavailableMappingStatus =
  "{\"active\":false,\"state\":\"unavailable\","
  "\"map_topic\":\"/map\",\"map_endpoint\":\"/api/v1/mapping/2d/map\","
  "\"live_map_available\":false,\"live_map_age_sec\":null,"
  "\"live_map_width\":0,\"live_map_height\":0,"
  "\"start_job\":{\"id\":0,\"state\":\"idle\"}}";

bool localization_transition_active(const std::string & detail)
{
  return detail.rfind("LOCALIZATION_TRANSITION_ACTIVE:", 0) == 0;
}

}  // namespace

struct SystemStatusModule::Impl
{
  Impl(
    rclcpp::Node & module_node,
    SystemStatusModuleConfig module_config,
    SystemStatusModulePorts module_ports)
  : node(module_node), config(std::move(module_config)), ports(std::move(module_ports))
  {
    ensure_floor_status_subscription_active();
  }

  void ensure_floor_status_subscription_active()
  {
    std::lock_guard<std::mutex> lock(floor_subscription_mutex);
    if (floor_status_subscription) {
      return;
    }
    floor_status_subscription = node.create_subscription<std_msgs::msg::String>(
      config.floor_status_topic,
      rclcpp::QoS(10),
      [this](const std_msgs::msg::String::SharedPtr message) {
        std::lock_guard<std::mutex> state_lock(floor_status_mutex);
        floor_status = message->data;
      });
  }

  std::string floor_status_snapshot() const
  {
    std::lock_guard<std::mutex> lock(floor_status_mutex);
    return floor_status;
  }

  std::optional<HttpResponse> handle_http(const HttpRequest & request)
  {
    if (request.method == "GET" && request.path == "/api/v1/status") {
      return handle_status();
    }
    if (request.method == "GET" && request.path == "/api/v1/robot/pose") {
      return handle_robot_pose();
    }
    return std::nullopt;
  }

  HttpResponse handle_status()
  {
    const auto snapshot = ports.status_snapshot ?
      ports.status_snapshot() : SystemStatusSnapshot{};
    const auto & runtime = snapshot.runtime;
    const auto & safety_state = snapshot.safety;
    const auto & bms = snapshot.bms;
    const auto & localization = snapshot.localization;
    const auto & amcl_status = snapshot.amcl;
    const auto & bridge_status = snapshot.bridge;
    const auto active_runtime_mode =
      application::runtime_mode::active_runtime_profile(runtime);
    const bool bms_valid = bms.have_state && bms.have_soc && bms.fresh;
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
      effective_amcl_ready &&
      !effective_amcl_correction_ready);
    const bool effective_amcl_degraded =
      bridge_amcl_available ? bridge_status.localization_degraded :
      (amcl_file_authoritative && (amcl_status.degraded || !amcl_status.ready));
    const std::string effective_amcl_degraded_reason =
      bridge_amcl_available ?
      (bridge_status.amcl_degraded_reason.empty() ?
        (bridge_status.localization_degraded ? std::string("AMCL_NOT_READY") : std::string()) :
        bridge_status.amcl_degraded_reason) :
      (effective_amcl_degraded ?
        (amcl_status.degraded_reason.empty() ? std::string("AMCL_NOT_READY") :
        amcl_status.degraded_reason) :
        std::string());
    const bool localization_degraded = effective_amcl_degraded;
    const bool using_triggered_baseline_only =
      (bridge_amcl_available || (amcl_status.available && amcl_status.mode != "disabled")) &&
      !effective_amcl_ready;
    const std::string localization_degraded_reason =
      localization_degraded ? effective_amcl_degraded_reason : std::string();
    const bool localization_recovery_required =
      !snapshot.bridge_safe_for_goal_start &&
      !localization_transition_active(snapshot.bridge_goal_start_detail);
    const auto mapping_status_json = snapshot.mapping_status_json.empty() ?
      std::string(kUnavailableMappingStatus) : snapshot.mapping_status_json;
    const auto subscriptions_json = snapshot.subscriptions_json.empty() ?
      std::string("{\"resources\":{}}") : snapshot.subscriptions_json;
    const auto floor_status_value = floor_status_snapshot();

    std::ostringstream body;
    body << "{";
    body << "\"ok\":true,";
    body << "\"api_version\":\"v1\",";
    body << "\"node\":\"robot_api_server\",";
    body << "\"mode\":" << json_string(runtime.mode) << ",";
    body << "\"active_runtime_mode\":" << json_string(active_runtime_mode) << ",";
    body << "\"state\":" << json_string(runtime.state) << ",";
    body << "\"mapping_active\":" << (runtime.mapping_active ? "true" : "false") << ",";
    body << "\"navigation_active\":" << (runtime.navigation_active ? "true" : "false") << ",";
    body << "\"healthy\":" << (runtime.healthy ? "true" : "false") << ",";
    body << "\"keepout_integrity_degraded\":"
         << (snapshot.keepout_integrity_degraded ? "true" : "false") << ",";
    body << "\"delayed_side_effect_unknown_count\":"
         << snapshot.delayed_side_effect_unknown_count << ",";
    body << "\"delayed_side_effect_recovery_blocked\":"
         << (snapshot.delayed_side_effect_unknown_count != 0U ? "true" : "false") << ",";
    body << "\"message\":" << json_string(runtime.message) << ",";
    body << "\"mode_transition\":{";
    body << "\"active\":" << (!snapshot.mode_transition_owner.empty() ? "true" : "false") << ",";
    body << "\"owner\":" << json_string(snapshot.mode_transition_owner) << "},";
    body << "\"floor_runtime_interlock\":{";
    body << "\"enabled\":" << (snapshot.floor_runtime_interlock_enabled ? "true" : "false") << ",";
    body << "\"negative_only\":true,";
    body << "\"blocked\":" << (snapshot.floor_runtime_interlock.blocked ? "true" : "false") << ",";
    body << "\"reason_code\":" << json_string(snapshot.floor_runtime_interlock.code) << ",";
    body << "\"transaction_id\":"
         << json_string(snapshot.floor_runtime_interlock.transaction_id) << ",";
    body << "\"detail\":" << json_string(snapshot.floor_runtime_interlock.detail) << "},";
    body << "\"localization_degraded\":" << (localization_degraded ? "true" : "false") << ",";
    body << "\"localization_degraded_reason\":"
         << json_string(localization_degraded_reason) << ",";
    body << "\"using_triggered_baseline_only\":"
         << (using_triggered_baseline_only ? "true" : "false") << ",";
    body << "\"mapping\":" << mapping_status_json << ",";
    body << "\"navigation\":{";
    body << "\"active\":" << (runtime.navigation_active ? "true" : "false") << ",";
    body << "\"state\":" << json_string(runtime.navigation_state) << ",";
    body << "\"action\":" << json_string(config.navigate_to_pose_action) << ",";
    body << "\"navigation_normal_path_relocalization_enabled\":false,";
    body << "\"force_accept_allowed_in_normal_path\":false,";
    body << "\"ordinary_navigation_triggered_relocalization\":false,";
    body << "\"localization_recovery_available\":true,";
    body << "\"removed_redundant_gates\":["
            "\"normal_pre_goal_force_accept\","
            "\"normal_pre_goal_global_localization_trigger\","
            "\"normal_pre_goal_post_relocalization_settle\","
            "\"docking_default_predock_relocalization\"],";
    body << "\"localization_recovery_required\":"
         << (localization_recovery_required ? "true" : "false") << ",";
    body << "\"pre_goal_check_endpoint\":\"/api/v1/navigation/pre_goal_check\",";
    body << "\"blocked_by_docked_contact\":"
         << (snapshot.dock.auto_undock_required ? "true" : "false") << ",";
    body << "\"normal_motion_blocked_reason\":"
         << json_string(safety::normal_motion_blocked_reason(safety_state)) << ",";
    body << "\"pre_navigation_dock_check\":" << snapshot.dock.pre_navigation_check_json << ",";
    body << "\"post_relocalization_settle\":" << snapshot.post_relocalization_settle_json << ",";
    body << "\"post_undock_settle\":" << snapshot.post_undock_settle_json << ",";
    body << "\"goal\":" << snapshot.navigation_goal_json;
    body << "},";
    body << "\"docking_active\":" << (runtime.docking_active ? "true" : "false") << ",";
    body << "\"docking\":{";
    body << "\"active\":" << (runtime.docking_active ? "true" : "false") << ",";
    body << "\"state\":" << json_string(runtime.docking_state) << ",";
    body << "\"docking_normal_path_relocalization_enabled\":false,";
    body << "\"docking_predock_triggered_relocalization\":false,";
    body << "\"charging_contact\":" << (bms.contact ? "true" : "false") << ",";
    body << "\"inferred_docked\":" << (snapshot.dock.inferred_docked ? "true" : "false") << ",";
    body << "\"dock_id\":" << json_string(runtime.docking_dock_id) << ",";
    body << "\"status_topic\":" << json_string(config.docking_status_topic) << ",";
    body << "\"last_status\":" << json_string(runtime.docking_status) << ",";
    body << "\"can_auto_undock\":" << (snapshot.dock.can_auto_undock ? "true" : "false") << ",";
    body << "\"auto_undock_reason\":" << json_string(snapshot.dock.auto_undock_reason) << ",";
    body << "\"pre_navigation_dock_check\":" << snapshot.dock.pre_navigation_check_json;
    body << "},";
    body << "\"safety_status\":" << json_string(safety_state.status) << ",";
    body << "\"motion_allowed\":" << (safety_state.motion_allowed ? "true" : "false") << ",";
    body << "\"motion_allowed_valid\":"
         << (safety_state.motion_allowed_valid ? "true" : "false") << ",";
    body << "\"safety\":" << safety::safety_state_json(safety_state) << ",";
    body << "\"floor_status\":" << json_string(floor_status_value) << ",";
    body << "\"localization\":{";
    body << "\"active_runtime_mode\":" << json_string(active_runtime_mode) << ",";
    body << "\"trigger_service\":" << json_string(config.localization_trigger_service) << ",";
    body << "\"result_topic\":" << json_string(config.localization_result_topic) << ",";
    body << "\"amcl_mode\":" << json_string(amcl_status.mode) << ",";
    body << "\"amcl_state\":" << json_string(amcl_status.state) << ",";
    body << "\"amcl_start_result\":" << json_string(amcl_status.start_result) << ",";
    body << "\"amcl_status_file_stale\":" << (amcl_status.stale ? "true" : "false") << ",";
    body << "\"amcl_status_age_ms\":" << amcl_status.age_ms << ",";
    body << "\"amcl_status_source\":" << json_string(
      bridge_amcl_available ? bridge_status.amcl_status_source :
      (amcl_file_authoritative ? std::string("file") : std::string("stale_file_ignored"))) << ",";
    body << "\"amcl_ready\":" << (effective_amcl_ready ? "true" : "false") << ",";
    body << "\"amcl_degraded\":" << (effective_amcl_degraded ? "true" : "false") << ",";
    body << "\"amcl_degraded_reason\":" << json_string(effective_amcl_degraded_reason) << ",";
    body << "\"amcl_process_alive\":" << (amcl_status.process_alive ? "true" : "false") << ",";
    body << "\"amcl_process_ready\":" << ((
      bridge_amcl_available ? bridge_status.amcl_process_ready : amcl_status.process_ready
    ) ? "true" : "false") << ",";
    body << "\"amcl_seeded\":" << ((
      bridge_amcl_available ? bridge_status.amcl_seeded : amcl_status.seeded
    ) ? "true" : "false") << ",";
    body << "\"amcl_seed_response_ok\":" << ((
      bridge_amcl_available ? bridge_status.amcl_seed_response_ok : amcl_status.seed_response_ok
    ) ? "true" : "false") << ",";
    body << "\"amcl_nomotion_pose_received\":" << ((
      bridge_amcl_available ? bridge_status.amcl_nomotion_pose_received : amcl_status.nomotion_pose_received
    ) ? "true" : "false") << ",";
    body << "\"amcl_static_standby\":" << ((
      bridge_amcl_available ? bridge_status.amcl_static_standby : amcl_status.static_standby
    ) ? "true" : "false") << ",";
    body << "\"amcl_tracking_ready\":" << ((
      bridge_amcl_available ? bridge_status.amcl_tracking_ready : amcl_status.tracking_ready
    ) ? "true" : "false") << ",";
    body << "\"amcl_correction_ready\":" << ((
      bridge_amcl_available ? bridge_status.amcl_correction_ready : amcl_status.correction_ready
    ) ? "true" : "false") << ",";
    body << "\"amcl_correction_pending\":"
         << (effective_amcl_correction_pending ? "true" : "false") << ",";
    body << "\"amcl_not_moving_no_update_ok\":" << ((
      bridge_amcl_available ?
        bridge_status.amcl_not_moving_no_update_ok :
        amcl_status.not_moving_no_update_ok
    ) ? "true" : "false") << ",";
    body << "\"amcl_scan_admission_alive\":"
         << (amcl_status.scan_admission_alive ? "true" : "false") << ",";
    body << "\"amcl_pose_publisher_count\":" << amcl_status.pose_publisher_count << ",";
    body << "\"amcl_scan_admission_status_publisher_count\":"
         << amcl_status.scan_admission_status_publisher_count << ",";
    body << "\"localization_degraded\":" << (localization_degraded ? "true" : "false") << ",";
    body << "\"localization_degraded_reason\":"
         << json_string(localization_degraded_reason) << ",";
    body << "\"using_triggered_baseline_only\":"
         << (using_triggered_baseline_only ? "true" : "false") << ",";
    body << "\"safe_for_goal_start\":"
         << (snapshot.bridge_safe_for_goal_start ? "true" : "false") << ",";
    body << "\"goal_start_detail\":" << json_string(snapshot.bridge_goal_start_detail) << ",";
    body << "\"correction_active\":"
         << (bridge_status.available && bridge_status.correction_active ? "true" : "false") << ",";
    body << "\"smoothing_enabled\":"
         << (bridge_status.available && bridge_status.smoothing_enabled ? "true" : "false") << ",";
    body << "\"remaining_translation_error_m\":"
         << (bridge_status.available ? bridge_status.remaining_translation_error_m : -1.0) << ",";
    body << "\"remaining_yaw_error_rad\":"
         << (bridge_status.available ? bridge_status.remaining_yaw_error_rad : -1.0) << ",";
    body << "\"localization_recovery_available\":true,";
    body << "\"localization_recovery_required\":"
         << (localization_recovery_required ? "true" : "false") << ",";
    body << "\"last_result_available\":" << (localization.available ? "true" : "false") << ",";
    body << "\"last_result_frame\":" << json_string(localization.frame_id) << ",";
    body << "\"last_result_age_sec\":";
    if (localization.available) {
      body << localization.age_sec;
    } else {
      body << "null";
    }
    body << ",\"last_result_seq\":" << localization.seq << ",";
    body << "\"post_relocalization_settle\":" << snapshot.post_relocalization_settle_json;
    body << "},";
    body << "\"bms\":{";
    body << "\"soc\":";
    if (bms.have_soc) {
      body << bms.soc;
    } else {
      body << "null";
    }
    body << ",\"soc_valid\":" << (bms_valid ? "true" : "false") << ",";
    body << "\"source_topic\":" << json_string(config.bms_state_topic) << ",";
    body << "\"age_sec\":" << bms.age_sec << ",";
    body << "\"power_supply_status\":" << bms.power_supply_status << ",";
    body << "\"power_supply_health\":" << bms.power_supply_health << ",";
    body << "\"power_supply_technology\":" << bms.power_supply_technology << ",";
    body << "\"present\":" << (bms.present ? "true" : "false") << ",";
    body << "\"charging_contact\":" << (bms.contact ? "true" : "false") << ",";
    body << "\"charging_contact_reason\":" << json_string(bms.reason) << ",";
    body << "\"contact_snapshot\":{" << snapshot.dock.bms_contact_snapshot_json << "},";
    body << "\"voltage\":";
    if (bms.have_state && std::isfinite(bms.voltage)) {
      body << bms.voltage;
    } else {
      body << "null";
    }
    body << ",\"current\":";
    if (bms.have_state && std::isfinite(bms.current)) {
      body << bms.current;
    } else {
      body << "null";
    }
    body << ",\"temperature\":";
    if (bms.have_state && std::isfinite(bms.temperature)) {
      body << bms.temperature;
    } else {
      body << "null";
    }
    body << "},";
    body << "\"subscriptions\":" << subscriptions_json << ",";
    body << "\"http\":{";
    body << "\"active_connections\":" << snapshot.http_active_connections << ",";
    body << "\"max_connections\":" << config.max_http_connections;
    body << "},";
    body << "\"maps_root\":" << json_string(config.maps_root) << ",";
    body << "\"runtime_maps_dir\":" << json_string(config.runtime_maps_dir);
    body << "}";
    return {200, "application/json", body.str()};
  }

  HttpResponse handle_robot_pose()
  {
    const auto runtime_context = ports.robot_pose_runtime_context ?
      ports.robot_pose_runtime_context() : RobotPoseRuntimeContextSnapshot{};
    if (runtime_context.available && runtime_context.blocked) {
      std::string detail = "runtime map context is not ready: state=" + runtime_context.state;
      if (!runtime_context.startup_stage.empty()) {
        detail += " startup_stage=" + runtime_context.startup_stage;
      }
      if (!runtime_context.message.empty()) {
        detail += " message=" + runtime_context.message;
      }
      return {503, "application/json", floor_context_not_ready_robot_pose_json(
          config.map_frame, config.base_frame, runtime_context.state, detail)};
    }

    std::string error;
    const auto pose = ports.wait_for_robot_pose ?
      ports.wait_for_robot_pose(error) : RobotPoseSnapshot{};
    if (!pose.available) {
      return {503, "application/json", no_fresh_map_robot_pose_json(
          config.map_frame, config.base_frame)};
    }

    const auto identity = ports.robot_pose_identity ?
      ports.robot_pose_identity() : RobotPoseIdentitySnapshot{};
    if (identity.blocked_by_pending_context) {
      return {503, "application/json", no_fresh_map_robot_pose_json(
          config.map_frame, config.base_frame, identity.error)};
    }
    return {200, "application/json", robot_pose_json(pose, identity.identity)};
  }

  rclcpp::Node & node;
  SystemStatusModuleConfig config;
  SystemStatusModulePorts ports;
  mutable std::mutex floor_status_mutex;
  std::mutex floor_subscription_mutex;
  std::string floor_status{"UNKNOWN"};
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr floor_status_subscription;
};

SystemStatusModule::SystemStatusModule(
  rclcpp::Node & node,
  SystemStatusModuleConfig config,
  SystemStatusModulePorts ports)
: impl_(std::make_unique<Impl>(node, std::move(config), std::move(ports)))
{
}

SystemStatusModule::~SystemStatusModule() = default;

std::optional<HttpResponse> SystemStatusModule::handle_http(const HttpRequest & request)
{
  return impl_->handle_http(request);
}

void SystemStatusModule::ensure_floor_status_subscription_active()
{
  impl_->ensure_floor_status_subscription_active();
}

}  // namespace robot_api_server::features::system_status
