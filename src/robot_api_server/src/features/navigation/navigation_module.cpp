#include "robot_api_server/features/navigation/navigation_module.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <future>
#include <iomanip>
#include <limits>
#include <map>
#include <mutex>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <utility>
#include <vector>

#include "lifecycle_msgs/msg/state.hpp"

#include "robot_api_server/features/elevator/configuration/elevator_configuration_module.hpp"
#include "robot_api_server/features/floor_switch/runtime_map_context_io.hpp"
#include "robot_api_server/features/localization/tf_pose_utils.hpp"
#include "robot_api_server/features/maps/catalog_activation/api_time_utils.hpp"
#include "robot_api_server/features/maps/catalog_activation/floor_asset_resolver.hpp"
#include "robot_api_server/features/maps/catalog_activation/map_asset_filesystem.hpp"
#include "robot_api_server/features/maps/catalog_activation/map_asset_identity_binding.hpp"
#include "robot_api_server/features/maps/catalog_activation/map_manifest_io.hpp"
#include "robot_api_server/features/maps/catalog_activation/runtime_map_lookup.hpp"
#include "robot_api_server/features/maps/poses/poses_io.hpp"
#include "robot_api_server/features/navigation/mission/navigation_goal_http.hpp"
#include "robot_api_server/features/navigation/runtime/navigation_cancel_policy.hpp"

namespace robot_api_server::features::navigation
{

class NavigationModule::Impl
{
public:
  Impl(
    rclcpp::Node & node,
    rclcpp::CallbackGroup::SharedPtr callback_group,
    rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr
    global_costmap_lifecycle_client,
    features::maps::MapsModule & maps_module,
    features::localization::LocalizationModule & localization_module,
    features::elevator::ElevatorModule & elevator_module,
    NavigationModuleConfig config,
    NavigationModulePorts ports)
  : node_(node),
    callback_group_(std::move(callback_group)),
    maps_module_(maps_module),
    localization_module_(localization_module),
    elevator_module_(elevator_module),
    ports_(std::move(ports)),
    maps_root_(config.maps_root),
    runtime_map_context_file_(config.runtime_map_context_file),
    resume_starting_context_ttl_sec_(config.resume_starting_context_ttl_sec),
    nav2_native_goal_completion_enabled_(config.nav2_native_goal_completion_enabled),
    api_final_yaw_align_fallback_enabled_(config.api_final_yaw_align_fallback_enabled),
    navigation_final_yaw_align_enabled_(config.navigation_final_yaw_align_enabled),
    post_nav2_final_verify_enabled_(config.post_nav2_final_verify_enabled),
    post_nav2_final_verify_wait_bridge_smoothing_(
      config.post_nav2_final_verify_wait_bridge_smoothing),
    post_nav2_final_verify_max_retry_count_(config.post_nav2_final_verify_max_retry_count),
    navigation_nav2_failed_near_goal_retry_max_count_(
      config.navigation_nav2_failed_near_goal_retry_max_count),
    post_nav2_final_verify_api_velocity_correction_enabled_(
      config.post_nav2_final_verify_api_velocity_correction_enabled),
    nav2_rotation_shim_enabled_(config.nav2_rotation_shim_enabled),
    navigation_final_yaw_align_timeout_sec_(config.navigation_final_yaw_align_timeout_sec),
    navigation_final_yaw_align_max_xy_drift_m_(
      config.navigation_final_yaw_align_max_xy_drift_m),
    navigation_final_yaw_align_cmd_topic_(
      std::move(config.navigation_final_yaw_align_cmd_topic)),
    navigation_final_yaw_align_bypass_collision_monitor_(
      config.navigation_final_yaw_align_bypass_collision_monitor),
    position_only_nav2_yaw_mode_(std::move(config.position_only_nav2_yaw_mode)),
    cancel_action_wait_(config.cancel_action_wait),
    action_name_(config.action.action_name),
    lifecycle_check_timeout_sec_(config.lifecycle_check_timeout_sec),
    action_runtime_(node, std::move(config.action), std::move(ports_.action)),
    mission_runtime_(),
    cancel_runtime_(),
    process_runtime_(std::move(config.process)),
    goal_policy_(std::move(config.goal_policy)),
    completion_policy_(std::move(config.completion_policy)),
    bridge_wait_(std::move(config.bridge_wait)),
    terminal_control_(std::move(config.terminal_control))
  {
    if (!ports_.pre_goal_dock_snapshot ||
      !ports_.floor_runtime_interlock_response ||
      !ports_.runtime_neighbor_snapshot ||
      !ports_.set_navigation_runtime_state ||
      !ports_.read_runtime_map_context ||
      !ports_.write_runtime_map_context ||
      !ports_.write_last_navigation_map_selection ||
      !ports_.clear_terminal_speed_limit ||
      !ports_.publish_zero_motion ||
      !ports_.safety_snapshot ||
      !ports_.post_relocalization_settle_json ||
      !ports_.post_undock_settle_json ||
      !ports_.run_goal_job)
    {
      throw std::invalid_argument("navigation module requires every runtime integration port");
    }
    for (const auto & node_name : lifecycle_node_names()) {
      if (node_name == "/global_costmap/global_costmap") {
        lifecycle_clients_[node_name] = global_costmap_lifecycle_client;
      } else {
        lifecycle_clients_[node_name] =
          node_.create_client<lifecycle_msgs::srv::GetState>(
          node_name + "/get_state",
          rmw_qos_profile_services_default,
          callback_group_);
      }
    }
  }

  NavigationLifecycleSnapshot lifecycle_snapshot()
  {
    NavigationLifecycleSnapshot snapshot;
    std::vector<std::string> inactive;
    const auto timeout = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(lifecycle_check_timeout_sec_));
    for (const auto & node_name : lifecycle_node_names()) {
      const auto client_it = lifecycle_clients_.find(node_name);
      if (client_it == lifecycle_clients_.end() || !client_it->second) {
        inactive.push_back(node_name + ":client_missing");
        continue;
      }
      auto & client = client_it->second;
      if (!client->wait_for_service(timeout)) {
        inactive.push_back(node_name + ":service_unavailable");
        continue;
      }
      auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();
      auto future = client->async_send_request(request);
      if (future.wait_for(timeout) != std::future_status::ready) {
        inactive.push_back(node_name + ":state_timeout");
        continue;
      }
      const auto response = future.get();
      if (response->current_state.id != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
        inactive.push_back(node_name + ":" + response->current_state.label);
      }
    }
    if (inactive.empty()) {
      snapshot.active = true;
      snapshot.detail = "navigation lifecycle active";
      return snapshot;
    }
    std::ostringstream detail;
    detail << "navigation lifecycle inactive";
    for (const auto & item : inactive) {
      detail << "; " << item;
    }
    snapshot.detail = detail.str();
    return snapshot;
  }

  NavigationLifecycleSnapshot goal_admission_snapshot()
  {
    const bool action_ready = action_runtime_.action_server_ready();
    NavigationLifecycleSnapshot snapshot;
    snapshot.active = action_ready;
    snapshot.detail = action_ready ?
      "navigation action server ready; lifecycle service probes are diagnostics only" :
      "navigation action server unavailable: " + action_name_;
    return snapshot;
  }

  HttpResponse handle_state()
  {
    refresh_runtime_state(false);

    NavigationStateSnapshot snapshot;
    snapshot.runtime = ports_.runtime_neighbor_snapshot(false).runtime;
    snapshot.runtime_map_context = ports_.read_runtime_map_context();

    NavigationPreGoalDockContext dock_context;
    dock_context.operation = "navigation_state";
    dock_context.frame_id = "map";
    const auto dock = ports_.pre_goal_dock_snapshot(dock_context);
    snapshot.dock.occupancy_state = dock.dock_occupancy_state;
    snapshot.dock.occupancy_evidence = dock.dock_occupancy_evidence;
    snapshot.dock.occupancy_reason = dock.dock_occupancy_reason;
    snapshot.dock.charging_session_latched = dock.charging_session_latched;
    snapshot.dock.charging_session_age_sec = dock.charging_session_age_sec;
    snapshot.dock.charging_session_last_confirmed_at =
      dock.charging_session_last_confirmed_at;
    snapshot.dock.full_charge_idle_on_dock = dock.full_charge_idle_on_dock;
    snapshot.dock.auto_undock_required = dock.auto_undock_required;
    snapshot.dock.pre_navigation_check_json = dock.json;
    snapshot.amcl = localization_module_.read_amcl_runtime_status(wall_time_seconds());
    snapshot.bridge = localization_module_.bridge_status_snapshot();
    snapshot.safety = ports_.safety_snapshot();
    snapshot.post_relocalization_settle_json = ports_.post_relocalization_settle_json();
    snapshot.post_undock_settle_json = ports_.post_undock_settle_json();
    snapshot.navigation_goal_json = mission_runtime_.json();
    snapshot.navigation_cancel_json = cancel_runtime_.json();

    NavigationStateConfig state_config;
    state_config.nav2_native_goal_completion_enabled =
      nav2_native_goal_completion_enabled_;
    state_config.api_final_yaw_align_enabled =
      api_final_yaw_align_fallback_enabled_ && navigation_final_yaw_align_enabled_;
    state_config.nav2_rotation_shim_enabled = nav2_rotation_shim_enabled_;
    return navigation_state_response(state_config, snapshot);
  }

  std::optional<HttpResponse> handle_http(
    const HttpRequest & request,
    const ElevatorMotionAdmissionFence::Epoch motion_admission_epoch)
  {
    if (request.method == "POST" && request.path == "/api/v1/navigation/start") {
      return handle_start(request.body, motion_admission_epoch);
    }
    if (request.method == "GET" && request.path == "/api/v1/navigation/state") {
      return handle_state();
    }
    if (request.method == "GET" && request.path == "/api/v1/navigation/pre_goal_check") {
      return handle_pre_goal_check(request);
    }
    if (request.method == "POST" && request.path == "/api/v1/navigation/goal") {
      return handle_goal(request.body, motion_admission_epoch);
    }
    if (request.method == "POST" && request.path == "/api/v1/navigation/cancel") {
      return handle_cancel(request.body, false);
    }
    if (request.method == "POST" &&
      (request.path == "/api/v1/navigation/stop" ||
      request.path == "/api/v1/navigation/stop_runtime"))
    {
      return handle_cancel(request.body, true);
    }
    return std::nullopt;
  }

  HttpResponse handle_pre_goal_check(const HttpRequest & request)
  {
    const auto query_string_value = [&request](
      const std::string & key, const std::string & default_value = "") {
        const auto it = request.query.find(key);
        return it == request.query.end() ? default_value : it->second;
      };
    const auto query_number_value = [&request](const std::string & key) -> std::optional<double> {
        const auto it = request.query.find(key);
        if (it == request.query.end() || it->second.empty()) {
          return std::nullopt;
        }
        try {
          std::size_t parsed = 0U;
          const double value = std::stod(it->second, &parsed);
          return parsed == it->second.size() ? std::optional<double>(value) : std::nullopt;
        } catch (...) {
          return std::nullopt;
        }
      };

    const auto pose_id = query_string_value(
      "pose_id", query_string_value("id", ""));
    auto building_id = query_string_value("building_id", "");
    auto floor_id = query_string_value("floor_id", "");
    const auto map_id = query_string_value("map_id", "");
    const auto frame_id = normalized_frame_id(query_string_value("frame_id", "map"));
    const bool by_pose_id = !pose_id.empty();
    const bool has_direct_pose_query =
      request.query.find("x") != request.query.end() ||
      request.query.find("y") != request.query.end() ||
      request.query.find("yaw") != request.query.end() ||
      request.query.find("theta") != request.query.end();

    bool pose_requested = by_pose_id;
    bool pose_ok = false;
    std::string pose_status = by_pose_id ? "unresolved" : "not_requested";
    std::string pose_detail;
    std::string target_source;
    StoredPose target;

    if (by_pose_id) {
      if (!safe_pose_id(pose_id)) {
        pose_status = "invalid_pose_id";
        pose_detail = "valid pose_id is required";
      } else {
        try {
          std::optional<MapManifest> manifest;
          if (!map_id.empty()) {
            manifest = maps_module_.catalog().find_map_by_id(map_id);
            if (!manifest) {
              pose_status = "map_not_found";
              pose_detail = "map_id not found: " + map_id;
            } else {
              building_id = manifest->building_id;
              floor_id = manifest->floor_id;
            }
          } else if (!safe_map_name(building_id) || !safe_map_name(floor_id)) {
            pose_status = "invalid_floor";
            pose_detail = "valid building_id and floor_id are required for pose_id navigation";
          } else {
            manifest = maps_module_.catalog().active_floor_map(building_id, floor_id);
            if (!manifest) {
              pose_status = "active_map_not_found";
              pose_detail = "active map not found for floor: " + building_id + "/" + floor_id;
            }
          }

          if (manifest) {
            const auto pose = find_floor_catalog_pose(
              maps_module_.catalog(), building_id, floor_id, pose_id);
            if (!pose) {
              pose_status = "pose_not_found";
              pose_detail = "pose_id not found in poses.yaml: " + pose_id;
            } else if (is_elevator_internal_pose_type(pose->type) ||
              is_reserved_elevator_pose_id(pose->id))
            {
              pose_status = "elevator_internal_pose_requires_mission";
              pose_detail = "elevator internal poses can only be used by an elevator mission";
              target_source = "elevator_configuration";
            } else {
              target = *pose;
              pose_ok = true;
              pose_status = "resolved";
              pose_detail = "pose_id resolved from poses.yaml";
              target_source = "poses_yaml";
            }
          }
        } catch (const std::exception & exception) {
          pose_status = "resolve_exception";
          pose_detail = exception.what();
        }
      }
    } else {
      const auto x = query_number_value("x");
      const auto y = query_number_value("y");
      auto yaw = query_number_value("yaw");
      if (!yaw) {
        yaw = query_number_value("theta");
      }
      if (x || y || yaw) {
        pose_requested = true;
        if (frame_id != "map") {
          pose_status = "invalid_frame";
          pose_detail = "navigation goals must be in map frame";
        } else if (!x || !y || !yaw || !std::isfinite(*x) ||
          !std::isfinite(*y) || !std::isfinite(*yaw))
        {
          pose_status = "invalid_direct_pose";
          pose_detail = "finite x, y, and yaw query parameters are required for direct pose precheck";
        } else {
          target.id = "direct";
          target.name = "direct";
          target.type = "direct_goal";
          target.x = *x;
          target.y = *y;
          target.yaw = normalize_angle(*yaw);
          pose_ok = true;
          pose_status = "resolved";
          pose_detail = "direct map-frame pose is valid";
          target_source = "direct_pose";
        }
      }
    }

    NavigationPreGoalDockContext dock_context;
    dock_context.pose_id = pose_id;
    dock_context.building_id = building_id;
    dock_context.floor_id = floor_id;
    dock_context.frame_id = frame_id;
    dock_context.has_direct_pose_query = has_direct_pose_query;
    const auto dock = ports_.pre_goal_dock_snapshot(dock_context);
    const auto lifecycle = goal_admission_snapshot();

    std::ostringstream response;
    response << std::fixed << std::setprecision(6)
             << "{\"ok\":true,"
             << "\"read_only\":true,"
             << "\"endpoint\":\"GET /api/v1/navigation/pre_goal_check\","
             << "\"would_auto_undock\":"
             << (dock.auto_undock_required ? "true" : "false") << ","
             << "\"auto_undock_required\":"
             << (dock.auto_undock_required ? "true" : "false") << ","
             << "\"can_auto_undock\":" << (dock.can_auto_undock ? "true" : "false") << ","
             << "\"pre_navigation_dock_check\":" << dock.json << ","
             << "\"navigation_lifecycle\":{"
             << "\"active\":" << (lifecycle.active ? "true" : "false") << ","
             << "\"detail\":" << json_string(lifecycle.detail) << "},"
             << "\"pose_resolution\":{"
             << "\"requested\":" << (pose_requested ? "true" : "false") << ","
             << "\"ok\":" << (pose_ok ? "true" : "false") << ","
             << "\"status\":" << json_string(pose_status) << ","
             << "\"detail\":" << json_string(pose_detail) << ","
             << "\"source\":" << json_string(target_source) << ","
             << "\"pose_id\":" << json_string(pose_id) << ","
             << "\"building_id\":" << json_string(building_id) << ","
             << "\"floor_id\":" << json_string(floor_id) << ","
             << "\"map_id\":" << json_string(map_id) << ","
             << "\"frame_id\":" << json_string(frame_id);
    if (pose_ok) {
      response << ",\"goal\":{\"x\":" << target.x << ",\"y\":" << target.y
               << ",\"yaw\":" << target.yaw << "}";
    }
    response << "}}";
    return {200, "application/json", response.str()};
  }

  HttpResponse handle_goal(
    const std::string & body,
    const ElevatorMotionAdmissionFence::Epoch motion_admission_epoch)
  {
    std::lock_guard<std::mutex> keepout_guard(maps_module_.mutation_mutex());
    if (const auto blocked = ports_.floor_runtime_interlock_response("navigation_goal")) {
      return *blocked;
    }
    const auto pose_id = json_string_value(body, "pose_id").value_or(
      json_string_value(body, "id").value_or(""));
    const auto building_id = json_string_value(body, "building_id");
    const auto floor_id = json_string_value(body, "floor_id");
    const bool by_pose_id = !pose_id.empty();

    StoredPose target;
    std::string target_source = "direct_pose";
    std::string frame_id = normalized_frame_id(
      json_string_value(body, "frame_id").value_or("map"));
    const bool force_pre_navigation_relocalization =
      json_bool_value(body, "force_relocalize", false) ||
      json_bool_value(body, "force_relocalization", false);

    NavigationPreGoalDockContext dock_context;
    dock_context.operation = "navigation_goal";
    dock_context.pose_id = pose_id;
    dock_context.building_id = building_id.value_or("");
    dock_context.floor_id = floor_id.value_or("");
    dock_context.frame_id = frame_id;
    dock_context.has_direct_pose_query = !by_pose_id;
    const auto dock = ports_.pre_goal_dock_snapshot(dock_context);
    RCLCPP_INFO(
      node_.get_logger(),
      "pre_navigation dock gate source=%s pose_id=%s auto_undock_required=%s can_auto_undock=%s reason=%s "
      "dock_occupancy_state=%s dock_occupancy_reason=%s "
      "bms_contact=%s bms_stable=%s latch_docked=%s latch_source=%s latch_age=%.3f latch_stale=%s "
      "latch_contradicted=%s latch_valid=%s strong_live_docked=%s docking_state=%s docking_status=%s",
      by_pose_id ? "pose_id" : "direct_pose",
      pose_id.c_str(),
      dock.auto_undock_required ? "true" : "false",
      dock.can_auto_undock ? "true" : "false",
      dock.auto_undock_reason.c_str(),
      dock.dock_occupancy_state.c_str(),
      dock.dock_occupancy_reason.c_str(),
      dock.bms_contact ? "true" : "false",
      dock.live_bms_charging_contact_stable ? "true" : "false",
      dock.dock_latch_indicates_docked ? "true" : "false",
      dock.dock_contact_latch_source.c_str(),
      dock.dock_contact_latch_age_sec,
      dock.dock_contact_latch_stale ? "true" : "false",
      dock.dock_contact_latch_contradicted_by_live_state ? "true" : "false",
      dock.latch_valid_for_auto_undock ? "true" : "false",
      dock.strong_live_docked ? "true" : "false",
      dock.runtime_docking_state.c_str(),
      dock.runtime_docking_status.c_str());
    const auto dock_payload = [&]() {
        NavigationPreGoalDockContext render_context;
        render_context.operation = "navigation_goal";
        render_context.pose_id = pose_id;
        render_context.building_id = building_id.value_or("");
        render_context.floor_id = floor_id.value_or("");
        render_context.frame_id = frame_id;
        render_context.has_direct_pose_query = !by_pose_id;
        return dock.render_json ? dock.render_json(render_context) : dock.json;
      };
    const auto goal_error = [&] (
        const int status,
        const std::string & error,
        const bool pre_navigation_undock = false,
        const std::string & pre_navigation_undock_detail = std::string()) {
        return navigation_goal_error_response(
          status,
          error,
          pre_navigation_undock,
          pre_navigation_undock_detail,
          dock_payload());
      };

    if (maps_module_.integrity_degraded()) {
      return goal_error(
        503,
        "keepout integrity is degraded; repair and re-prove the active keepout layer before navigation");
    }

    const auto neighbors = ports_.runtime_neighbor_snapshot(false);
    if (neighbors.runtime.mapping_active || neighbors.mapping_start_job_running ||
      neighbors.mode_transition_owner == "mapping_start")
    {
      return goal_error(
        409,
        "navigation goals are unavailable while 2D mapping is active or starting");
    }

    if (by_pose_id) {
      if (!building_id || !safe_map_name(*building_id)) {
        return goal_error(400, "valid building_id is required for pose_id navigation");
      }
      if (!floor_id || !safe_map_name(*floor_id)) {
        return goal_error(400, "valid floor_id is required for pose_id navigation");
      }
      if (!safe_pose_id(pose_id)) {
        return goal_error(400, "valid pose_id is required");
      }
      std::optional<StoredPose> pose;
      try {
        pose = find_floor_catalog_pose(
          maps_module_.catalog(), *building_id, *floor_id, pose_id);
      } catch (const std::exception & exception) {
        return goal_error(500, exception.what());
      }
      if (!pose) {
        return goal_error(404, "pose_id not found in poses.yaml: " + pose_id);
      }
      if (is_elevator_internal_pose_type(pose->type) ||
        is_reserved_elevator_pose_id(pose->id))
      {
        return navigation_goal_error_response(
          409,
          "elevator internal poses can only be used by an elevator mission",
          false,
          "",
          dock_payload(),
          "ELEVATOR_INTERNAL_POSE_REQUIRES_MISSION");
      }
      target = *pose;
      target_source = "poses_yaml";
      frame_id = "map";
    } else {
      const auto x = json_number_value(body, "x").value_or(
        json_nested_number_value(body, "pose", "x").value_or(
          std::numeric_limits<double>::quiet_NaN()));
      const auto y = json_number_value(body, "y").value_or(
        json_nested_number_value(body, "pose", "y").value_or(
          std::numeric_limits<double>::quiet_NaN()));
      const auto yaw = json_number_value(body, "yaw").value_or(
        json_number_value(body, "theta").value_or(
          json_nested_number_value(body, "pose", "yaw").value_or(
            std::numeric_limits<double>::quiet_NaN())));
      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(yaw)) {
        return goal_error(400, "pose_id or finite x, y, and yaw are required");
      }
      target.id = "direct";
      target.name = "direct";
      target.type = "direct_goal";
      target.x = x;
      target.y = y;
      target.yaw = normalize_angle(yaw);
    }

    if (frame_id != "map") {
      return goal_error(400, "navigation goals must be in map frame");
    }

    const auto completion_resolution = goal_policy_.resolve_completion_policy(body, target);
    if (!completion_resolution.error.empty()) {
      return goal_error(400, completion_resolution.error);
    }
    const auto & goal_completion_policy = completion_resolution.policy;
    if (goal_completion_policy == "dock_staging") {
      return goal_error(
        400,
        "goal_completion_policy=dock_staging is reserved for /api/v1/docking/start");
    }

    if (by_pose_id) {
      std::string context_error;
      bool blocked_by_pending_context = false;
      const auto active_map = maps_module_.confirmed_runtime_manifest(
        context_error, blocked_by_pending_context);
      if (blocked_by_pending_context) {
        return goal_error(503, context_error);
      }
      if (active_map &&
        (active_map->building_id != *building_id || active_map->floor_id != *floor_id))
      {
        return goal_error(
          409,
          "requested pose target does not match confirmed runtime map context: target=" +
          *building_id + "/" + *floor_id + " current=" +
          active_map->building_id + "/" + active_map->floor_id + "/" + active_map->map_id);
      }
    }

    std::lock_guard<std::mutex> goal_start_lock(goal_start_mutex_);
    if (mission_runtime_.running()) {
      return goal_error(
        409,
        "navigation goal is already running; cancel it before starting a new goal");
    }
    mission_runtime_.join();

    if (force_pre_navigation_relocalization) {
      const auto detail =
        "LOCALIZATION_RECOVERY_REQUIRED: force_relocalize is no longer executed inside normal navigation goals; "
        "call the explicit localization recovery endpoint before retrying the goal";
      ports_.set_navigation_runtime_state(
        true, "localization_recovery_required", detail, false);
      return goal_error(
        409, detail, dock.auto_undock_required, dock.auto_undock_reason);
    }

    RobotPoseSnapshot current_pose;
    std::string pose_error;
    if (goal_completion_policy == "position_only" &&
      position_only_nav2_yaw_mode_ != "stored_yaw")
    {
      current_pose = localization_module_.wait_for_current_robot_pose(true, pose_error);
    }
    const auto yaw_resolution = goal_policy_.resolve_nav2_goal_yaw(
      goal_completion_policy, target, current_pose, pose_error);

    nav2_msgs::action::NavigateToPose::Goal goal;
    goal.pose.header.frame_id = frame_id;
    goal.pose.header.stamp = node_.now();
    goal.pose.pose.position.x = target.x;
    goal.pose.pose.position.y = target.y;
    goal.pose.pose.position.z = 0.0;
    goal.pose.pose.orientation.z = std::sin(yaw_resolution.yaw * 0.5);
    goal.pose.pose.orientation.w = std::cos(yaw_resolution.yaw * 0.5);

    NavigationGoalJobStartSpec start;
    start.pose_id = by_pose_id ? pose_id : "";
    start.building_id = building_id.value_or("");
    start.floor_id = floor_id.value_or("");
    start.goal_completion_policy = goal_completion_policy;
    start.native_nav2_goal_completion =
      goal_completion_policy == "pose_required" && nav2_native_goal_completion_enabled_;
    start.api_final_yaw_align_enabled =
      goal_completion_policy == "pose_required" &&
      api_final_yaw_align_fallback_enabled_ &&
      navigation_final_yaw_align_enabled_;
    start.post_nav2_final_verify_enabled = post_nav2_final_verify_enabled_;
    start.post_nav2_final_verify_wait_bridge_smoothing =
      post_nav2_final_verify_wait_bridge_smoothing_;
    start.final_verify_retry_max_count = std::max(
      post_nav2_final_verify_max_retry_count_,
      navigation_nav2_failed_near_goal_retry_max_count_);
    start.post_nav2_final_verify_api_velocity_correction_enabled =
      post_nav2_final_verify_api_velocity_correction_enabled_;
    start.nav2_rotation_shim_enabled = nav2_rotation_shim_enabled_;
    start.target_x = target.x;
    start.target_y = target.y;
    start.target_yaw = target.yaw;
    start.nav2_goal_yaw = yaw_resolution.yaw;
    start.nav2_goal_yaw_source = yaw_resolution.source;
    start.final_yaw_align_timeout_sec = navigation_final_yaw_align_timeout_sec_;
    start.final_yaw_align_max_xy_drift_m = navigation_final_yaw_align_max_xy_drift_m_;
    start.final_yaw_align_cmd_topic = navigation_final_yaw_align_cmd_topic_;
    start.final_yaw_align_bypass_collision_monitor =
      navigation_final_yaw_align_bypass_collision_monitor_;
    start.pre_navigation_undock = dock.auto_undock_required;
    start.pre_navigation_undock_detail = dock.auto_undock_required ?
      "queued controlled undock in navigation background job" :
      dock.auto_undock_reason;
    start.started_at = utc_timestamp_iso8601();

    ports_.set_navigation_runtime_state(
      true,
      "navigating",
      dock.auto_undock_required ?
      "navigation goal accepted; background job will undock before Nav2 action send" :
      "navigation goal accepted; background job will check readiness before Nav2 action send",
      true);

    const auto active_pose_id = by_pose_id ? pose_id : std::string();
    const auto active_building_id = building_id.value_or("");
    const auto active_floor_id = floor_id.value_or("");
    std::uint64_t navigation_job_id = 0U;
    try {
      navigation_job_id = mission_runtime_.start(
        std::move(start),
        [this,
        goal,
        target,
        active_pose_id,
        active_building_id,
        active_floor_id,
        dock](const std::uint64_t job_id) {
          ports_.run_goal_job(
            job_id,
            goal,
            target,
            active_pose_id,
            active_building_id,
            active_floor_id,
            dock);
        });
    } catch (const std::exception & exception) {
      ports_.set_navigation_runtime_state(
        true,
        "failed",
        std::string("failed to start navigation goal worker: ") + exception.what(),
        false);
      return goal_error(
        500,
        std::string("failed to start navigation goal worker: ") + exception.what(),
        dock.auto_undock_required,
        dock.auto_undock_reason);
    }

    NavigationGoalAcceptedPayload accepted;
    accepted.navigation_goal_id = navigation_job_id;
    accepted.action = action_name_;
    accepted.source = target_source;
    accepted.goal_completion_policy = goal_completion_policy;
    accepted.native_nav2_goal_completion =
      goal_completion_policy == "pose_required" && nav2_native_goal_completion_enabled_;
    accepted.api_final_yaw_align_enabled =
      goal_completion_policy == "pose_required" &&
      api_final_yaw_align_fallback_enabled_ && navigation_final_yaw_align_enabled_;
    accepted.nav2_rotation_shim_enabled = nav2_rotation_shim_enabled_;
    accepted.frame_id = frame_id;
    accepted.pose_id = by_pose_id ? pose_id : "";
    accepted.building_id = building_id;
    accepted.floor_id = floor_id;
    accepted.pre_navigation_undock = dock.auto_undock_required;
    accepted.pre_navigation_undock_detail = accepted.pre_navigation_undock ?
      "queued controlled undock in navigation background job" :
      dock.auto_undock_reason;
    accepted.pre_navigation_dock_check_json = dock_payload();
    accepted.pre_navigation_relocalization_requested = false;
    accepted.pre_navigation_relocalization_succeeded = false;
    accepted.pre_navigation_relocalization_detail =
      "normal path relocalization disabled; goal-start readiness will be checked in navigation background job";
    accepted.goal = target;
    return navigation_goal_accepted_response(accepted);
  }

  bool request_goal_cancel(const std::string & reason)
  {
    ports_.clear_terminal_speed_limit();
    return mission_runtime_.request_cancel(reason);
  }

  bool cancel_active_goal(std::string & detail)
  {
    request_goal_cancel("navigation action cancel requested");
    return action_runtime_.cancel_active_goal(
      "cached navigation goal cancellation", detail);
  }

  bool cancel_goal_for_api_handoff(
    const NavigationActionRuntime::GoalHandle::SharedPtr & goal_handle,
    std::string & detail)
  {
    return action_runtime_.cancel_goal_and_prove_terminal(
      goal_handle,
      "Nav2 goal cancellation for API near-goal handoff",
      detail);
  }

  bool cancel_task_for_mode_switch(const std::string & reason, std::string & detail)
  {
    std::lock_guard<std::mutex> start_lock(cancel_start_mutex_);
    cancel_runtime_.join();
    request_goal_cancel(reason);
    bool action_available = false;
    std::string action_availability_detail;
    try {
      std::lock_guard<std::mutex> action_lock(action_runtime_.mutex());
      action_available = action_runtime_.wait_for_action_server(cancel_action_wait_);
    } catch (const std::exception & exception) {
      action_availability_detail =
        std::string("exception waiting for navigation action server: ") + exception.what();
    } catch (...) {
      action_availability_detail = "unknown exception waiting for navigation action server";
    }

    std::string active_goal_detail = "not requested";
    bool active_goal_cancel_requested = false;
    bool cancel_all_requested = false;
    bool cancel_all_ok = true;
    std::string cancel_all_detail = "not requested";
    if (action_available) {
      active_goal_cancel_requested = cancel_active_goal(active_goal_detail);
      cancel_all_requested = true;
      cancel_all_ok = action_runtime_.cancel_all_goals_with_evidence(
        "canceling navigation goals for mode switch", cancel_all_detail);
    } else {
      cancel_all_ok = !mission_runtime_.running();
      active_goal_detail = action_availability_detail.empty() ?
        "action server unavailable: " + action_name_ : action_availability_detail;
      cancel_all_detail = cancel_all_ok ?
        "no running navigation goal; action server unavailable" :
        "navigation goal is still running but action server is unavailable";
    }

    ports_.publish_zero_motion();
    std::ostringstream output;
    output << reason
           << ": action_available=" << (action_available ? "true" : "false")
           << ", active_goal_cancel_requested="
           << (active_goal_cancel_requested ? "true" : "false")
           << ", active_goal_detail=" << active_goal_detail
           << ", cancel_all_requested=" << (cancel_all_requested ? "true" : "false")
           << ", cancel_all_detail=" << cancel_all_detail;
    detail = output.str();
    return cancel_all_ok;
  }

  void set_cancel_job_phase(const std::uint64_t job_id, const std::string & phase)
  {
    (void)cancel_runtime_.update_running(
      job_id, [&phase](NavigationCancelJob & job) {job.phase = phase;});
  }

  void finish_cancel_job(
    const std::uint64_t job_id,
    const bool ok,
    const bool action_available,
    const bool active_goal_cancel_requested,
    const bool cancel_all_requested,
    const bool cancel_all_ok,
    const bool stop_stack_ok,
    const std::string & detail,
    const std::string & cancel_all_detail,
    const std::string & stop_stack_detail)
  {
    const auto current = cancel_runtime_.snapshot();
    if (current.id != job_id) {
      return;
    }
    NavigationCancelFinishSpec finish;
    finish.ok = ok;
    finish.action_available = action_available;
    finish.active_goal_cancel_requested = active_goal_cancel_requested;
    finish.cancel_all_requested = cancel_all_requested;
    finish.cancel_all_ok = cancel_all_ok;
    finish.stop_stack_ok = stop_stack_ok;
    finish.detail = detail;
    finish.cancel_all_detail = cancel_all_detail;
    finish.stop_stack_detail = stop_stack_detail;
    finish.finished_at = utc_timestamp_iso8601();
    if (!cancel_runtime_.finish(job_id, finish)) {
      return;
    }
    if (ok && current.stop_stack) {
      ports_.set_navigation_runtime_state(
        false, "stopped", "navigation runtime stopped", true);
    } else if (ok) {
      ports_.set_navigation_runtime_state(
        true, "ready", "navigation goal canceled; navigation stack remains active", true);
    } else {
      const auto message = stop_stack_detail.empty() ? detail : stop_stack_detail;
      ports_.set_navigation_runtime_state(true, "error", message, false);
    }
  }

  void run_cancel_job(const std::uint64_t job_id, const bool stop_stack)
  {
    bool action_available = false;
    std::string action_availability_detail;
    set_cancel_job_phase(job_id, "wait_for_nav2_action");
    try {
      std::lock_guard<std::mutex> action_lock(action_runtime_.mutex());
      action_available = action_runtime_.wait_for_action_server(cancel_action_wait_);
    } catch (const std::exception & exception) {
      action_availability_detail =
        std::string("exception waiting for navigation action server: ") + exception.what();
    } catch (...) {
      action_availability_detail = "unknown exception waiting for navigation action server";
    }

    std::string active_goal_detail;
    bool active_goal_cancel_requested = false;
    bool cancel_all_requested = false;
    bool cancel_all_ok = true;
    std::string cancel_all_detail = "not requested";
    if (action_available) {
      set_cancel_job_phase(job_id, "cancel_cached_goal");
      active_goal_cancel_requested = cancel_active_goal(active_goal_detail);
      cancel_all_requested = true;
      set_cancel_job_phase(job_id, "cancel_all_goals");
      cancel_all_ok = action_runtime_.cancel_all_goals_with_evidence(
        "canceling all navigation goals", cancel_all_detail);
    } else {
      active_goal_detail = action_availability_detail.empty() ?
        "action server unavailable: " + action_name_ : action_availability_detail;
      cancel_all_ok = false;
      cancel_all_detail = active_goal_detail;
    }

    set_cancel_job_phase(job_id, "publish_zero_velocity");
    ports_.publish_zero_motion();

    std::string stop_stack_detail = "not requested";
    bool stop_stack_ok = true;
    if (stop_stack) {
      if (navigation_stack_stop_allowed(cancel_all_ok)) {
        set_cancel_job_phase(job_id, "stop_navigation_stack");
        stop_stack_ok = stop_runtime_stack(stop_stack_detail);
        ports_.publish_zero_motion();
      } else {
        stop_stack_ok = false;
        stop_stack_detail =
          "navigation runtime stop was not attempted because Nav2 goal "
          "terminal state is unproven; " + cancel_all_detail;
      }
    }

    const bool ok = navigation_cancel_job_succeeded(
      cancel_all_ok, stop_stack, stop_stack_ok);
    finish_cancel_job(
      job_id,
      ok,
      action_available,
      active_goal_cancel_requested,
      cancel_all_requested,
      cancel_all_ok,
      stop_stack_ok,
      active_goal_detail,
      cancel_all_detail,
      stop_stack_detail);
  }

  void run_cancel_job_guarded(const std::uint64_t job_id, const bool stop_stack)
  {
    try {
      run_cancel_job(job_id, stop_stack);
    } catch (const std::exception & exception) {
      const std::string detail =
        std::string("navigation cancel worker exception: ") + exception.what();
      finish_cancel_job(
        job_id, false, false, false, false, false, false, detail, detail, "not completed");
    } catch (...) {
      const std::string detail = "navigation cancel worker unknown exception";
      finish_cancel_job(
        job_id, false, false, false, false, false, false, detail, detail, "not completed");
    }
  }

  HttpResponse handle_cancel(const std::string & body, const bool force_stop_stack)
  {
    const auto reason = json_string_value(body, "reason").value_or("");
    const bool stop_stack = force_stop_stack || json_bool_value(body, "stop_stack", false);
    const auto neighbors = ports_.runtime_neighbor_snapshot(false);
    if (neighbors.runtime.mapping_active || neighbors.mapping_start_job_running ||
      neighbors.mode_transition_owner == "mapping_start")
    {
      ports_.publish_zero_motion();
      return {
        409,
        "application/json",
        error_json("navigation cancel/stop is unavailable while 2D mapping is active or starting")};
    }

    ports_.publish_zero_motion();
    request_goal_cancel(reason.empty() ? "app_navigation_cancel" : reason);
    ports_.set_navigation_runtime_state(
      true,
      stop_stack ? "stopping" : "canceling",
      stop_stack ? "navigation runtime stop accepted" : "navigation cancel accepted",
      true);

    std::lock_guard<std::mutex> start_lock(cancel_start_mutex_);
    if (cancel_runtime_.running()) {
      std::ostringstream response;
      response << "{\"ok\":true,"
               << "\"accepted\":true,"
               << "\"already_running\":true,"
               << "\"navigation_cancel\":" << cancel_runtime_.json() << "}";
      return {202, "application/json", response.str()};
    }
    cancel_runtime_.join();

    NavigationCancelStartSpec start;
    start.reason = reason;
    start.stop_stack = stop_stack;
    start.started_at = utc_timestamp_iso8601();
    start.zero_velocity_published = true;
    try {
      (void)cancel_runtime_.start(
        start,
        [this](const std::uint64_t id, const bool worker_stop_stack) {
          run_cancel_job_guarded(id, worker_stop_stack);
        });
    } catch (const std::exception & exception) {
      ports_.set_navigation_runtime_state(
        true,
        "error",
        std::string("failed to start navigation cancel worker: ") + exception.what(),
        false);
      return {
        500,
        "application/json",
        error_json(
          std::string("failed to start navigation cancel worker: ") + exception.what())};
    }

    std::ostringstream response;
    response << "{\"ok\":true,"
             << "\"accepted\":true,"
             << "\"cancel_requested\":true,"
             << "\"stop_stack\":" << (stop_stack ? "true" : "false") << ","
             << "\"action\":" << json_string(action_name_) << ","
             << "\"navigation_cancel\":" << cancel_runtime_.json() << "}";
    return {202, "application/json", response.str()};
  }

  void refresh_runtime_state(const bool probe_lifecycle)
  {
    const auto neighbor = ports_.runtime_neighbor_snapshot(false);
    if (neighbor.runtime.mapping_active ||
      neighbor.mode_transition_owner == "mapping_start")
    {
      return;
    }

    const bool had_process = process_runtime_.pid() > 0;
    const bool process_is_running = process_runtime_.running();
    if (had_process && !process_is_running) {
      ports_.set_navigation_runtime_state(
        false,
        "failed",
        "navigation runtime process exited before ready; check " +
        process_runtime_.resume_log_file(),
        false);
      return;
    }
    const auto context = ports_.read_runtime_map_context();
    if (context && !context->confirmed && context->state == "failed") {
      ports_.set_navigation_runtime_state(
        false,
        "failed",
        context->message.empty() ?
        "navigation runtime failed before ready" : context->message,
        false);
      return;
    }
    const bool context_ready = context && context->confirmed && context->state == "ready";
    const bool navigate_action_ready = action_runtime_.action_server_ready();
    if (context_ready && (process_is_running || navigate_action_ready)) {
      if (probe_lifecycle) {
        const auto lifecycle = lifecycle_snapshot();
        if (!lifecycle.active) {
          ports_.set_navigation_runtime_state(true, "degraded", lifecycle.detail, false);
          return;
        }
      }
      ports_.set_navigation_runtime_state(
        true,
        "running",
        context->message.empty() ? "navigation runtime ready" : context->message,
        true);
    }
  }

  bool process_running()
  {
    return process_runtime_.running();
  }

  bool stop_runtime_stack(std::string & detail)
  {
    std::lock_guard<std::mutex> lock(runtime_commit_mutex_);
    return process_runtime_.stop_stack(detail);
  }

  HttpResponse handle_start(
    const std::string & body,
    const ElevatorMotionAdmissionFence::Epoch motion_admission_epoch)
  {
    std::lock_guard<std::mutex> keepout_guard(maps_module_.mutation_mutex());
    if (const auto blocked = ports_.floor_runtime_interlock_response("navigation_start")) {
      return *blocked;
    }
    const auto reject = [](const int status, const std::string & code, const std::string & detail) {
        std::ostringstream response;
        response << "{\"ok\":false,"
                 << "\"code\":" << json_string(code) << ","
                 << "\"detail\":" << json_string(detail) << "}";
        return HttpResponse{status, "application/json", response.str()};
      };
    if (maps_module_.integrity_degraded()) {
      return reject(
        503,
        "NAVIGATION_MAP_INTEGRITY_DEGRADED",
        "keepout integrity is degraded; navigation startup is blocked until repaired");
    }

    const auto building_id = json_string_value(body, "building_id");
    const auto floor_id = json_string_value(body, "floor_id");
    const auto map_id = json_string_value(body, "map_id");
    if (!building_id || building_id->empty() ||
      !floor_id || floor_id->empty() ||
      !map_id || map_id->empty())
    {
      return reject(
        400,
        "INVALID_NAVIGATION_START_REQUEST",
        "building_id, floor_id, and map_id are required");
    }
    if (!safe_asset_id(*building_id) ||
      !safe_asset_id(*floor_id) ||
      !safe_asset_id(*map_id))
    {
      return reject(
        400,
        "INVALID_NAVIGATION_START_REQUEST",
        "building_id, floor_id, and map_id must be safe asset ids");
    }

    std::optional<MapManifest> selected_map;
    try {
      selected_map = maps_module_.catalog().find_map_by_id(*map_id);
    } catch (const std::exception & exception) {
      return reject(500, "NAVIGATION_MAP_LOOKUP_FAILED", exception.what());
    }
    if (!selected_map) {
      return reject(404, "NAVIGATION_MAP_NOT_FOUND", "map_id not found: " + *map_id);
    }
    if (selected_map->building_id != *building_id ||
      selected_map->floor_id != *floor_id)
    {
      return reject(
        400,
        "NAVIGATION_MAP_IDENTITY_MISMATCH",
        "map_id does not belong to the requested building_id/floor_id");
    }

    std::string asset_error;
    std::unique_ptr<MapAssetCommitTransaction> asset_transaction;
    try {
      asset_transaction = std::make_unique<MapAssetCommitTransaction>(maps_root_);
      if (!maps_module_.validate_manifest_assets(*selected_map, asset_error)) {
        return reject(409, "NAVIGATION_MAP_ASSETS_INVALID", asset_error);
      }
      selected_map = verify_map_asset_identity_snapshot(
        *selected_map, maps_root_, *asset_transaction).manifest;
    } catch (const std::exception & exception) {
      return reject(
        409,
        "NAVIGATION_MAP_ASSETS_INVALID",
        std::string("map identity verification failed: ") + exception.what());
    }

    std::size_t active_manifest_count = 0U;
    bool requested_map_is_active = false;
    try {
      for (const auto & manifest : maps_module_.catalog().read_floor_map_manifests(
          *building_id, *floor_id, false))
      {
        if (!manifest.active) {
          continue;
        }
        ++active_manifest_count;
        requested_map_is_active = requested_map_is_active || manifest.map_id == *map_id;
      }
    } catch (const std::exception & exception) {
      return reject(500, "NAVIGATION_MAP_LOOKUP_FAILED", exception.what());
    }
    if (!selected_map->active || active_manifest_count != 1U || !requested_map_is_active) {
      return reject(
        409,
        "NAVIGATION_MAP_NOT_SELECTED",
        "requested map is not the single active offline-selected map for its floor");
    }

    const auto current_root =
      maps_module_.catalog().floor_current_root_path(*building_id, *floor_id);
    const auto current_manifest_path = current_root / "manifest.json";
    std::optional<MapManifest> current_manifest;
    try {
      current_manifest = read_map_manifest(current_manifest_path);
    } catch (const std::exception & exception) {
      return reject(409, "NAVIGATION_MAP_NOT_SELECTED", exception.what());
    }
    if (!current_manifest ||
      current_manifest->building_id != *building_id ||
      current_manifest->floor_id != *floor_id ||
      current_manifest->map_id != *map_id ||
      current_manifest->asset_epoch != selected_map->asset_epoch ||
      current_manifest->asset_digest != selected_map->asset_digest)
    {
      return reject(
        409,
        "NAVIGATION_MAP_NOT_SELECTED",
        "backend current/ mirror does not match the requested active map");
    }
    if (!maps_module_.validate_current_projection_assets(
        *selected_map, current_root, asset_error))
    {
      return reject(
        409,
        "NAVIGATION_MAP_ASSETS_INVALID",
        "backend current/ mirror is incomplete: " + asset_error);
    }

    (void)ports_.runtime_neighbor_snapshot(true);
    auto neighbor = ports_.runtime_neighbor_snapshot(false);
    if (neighbor.mode_transition_owner != "mapping_start") {
      refresh_runtime_state(false);
      neighbor = ports_.runtime_neighbor_snapshot(false);
    }
    const auto & runtime = neighbor.runtime;
    if (runtime.mapping_active || runtime.docking_active ||
      neighbor.mapping_start_job_running || !neighbor.mode_transition_owner.empty())
    {
      return reject(
        409,
        "NAVIGATION_RUNTIME_BUSY",
        "navigation startup requires mapping, docking, and mode transitions to be idle");
    }
    if (neighbor.docking_job_running) {
      return reject(
        409,
        "NAVIGATION_RUNTIME_BUSY",
        "navigation startup is blocked while a docking job is running");
    }
    if (mission_runtime_.running()) {
      return reject(
        409,
        "NAVIGATION_RUNTIME_BUSY",
        "navigation startup is blocked while a navigation goal job is running");
    }

    const bool resume_process_running = process_runtime_.running();
    if (runtime.navigation_active || resume_process_running) {
      const auto context = ports_.read_runtime_map_context();
      if (!context || !runtime_context_same_resume_request(
          *context, *building_id, *floor_id, *selected_map))
      {
        return reject(
          409,
          "NAVIGATION_RUNTIME_BUSY",
          "an existing navigation runtime does not match the requested map");
      }
    }

    return handle_resume_floor_navigation(
      *building_id,
      *floor_id,
      std::optional<MapManifest>(*selected_map),
      motion_admission_epoch);
  }

  HttpResponse handle_resume_floor_navigation(
    const std::string & building_id,
    const std::string & floor_id,
    const std::optional<MapManifest> & selected_map,
    const ElevatorMotionAdmissionFence::Epoch motion_admission_epoch)
  {
    if (const auto blocked =
      ports_.floor_runtime_interlock_response("navigation_runtime_resume"))
    {
      return *blocked;
    }
    FloorAssetPaths assets;
    std::string error;
    if (!resolve_floor_asset_paths(
        maps_module_.catalog(), building_id, floor_id, assets, error))
    {
      return {404, "application/json", error_json(error)};
    }
    if (!process_runtime_.resume_command_available()) {
      return {
        503,
        "application/json",
        error_json(
          "navigation resume command is not available: " +
          process_runtime_.resume_command())};
    }

    std::lock_guard<std::mutex> process_lock(runtime_commit_mutex_);
    if (const auto blocked =
      ports_.floor_runtime_interlock_response("navigation_runtime_resume_commit"))
    {
      return *blocked;
    }
    const bool existing_resume_process_running = process_runtime_.running();
    const auto existing_context = selected_map ?
      ports_.read_runtime_map_context() : std::nullopt;
    const bool navigate_action_ready = action_runtime_.action_server_ready();
    const auto lifecycle = goal_admission_snapshot();
    if (selected_map && existing_context && runtime_context_same_resume_request(
        *existing_context, building_id, floor_id, *selected_map))
    {
      const bool context_ready = runtime_context_matches_resume_request(
        *existing_context, building_id, floor_id, *selected_map);
      const bool context_starting_fresh = runtime_context_starting_is_fresh(*existing_context);
      const bool runtime_owner_evident =
        existing_resume_process_running || navigate_action_ready || lifecycle.active;
      if (context_ready && runtime_owner_evident) {
        try {
          ports_.write_last_navigation_map_selection(
            *selected_map, "navigation_runtime_reused");
        } catch (const std::exception & exception) {
          return {500, "application/json", error_json(exception.what())};
        }
        ports_.set_navigation_runtime_state(
          true,
          lifecycle.active ? "running" : "degraded",
          lifecycle.active ? existing_context->message : lifecycle.detail,
          lifecycle.active);

        std::ostringstream response;
        response << "{\"ok\":true,"
                 << "\"state\":\"navigation_runtime_reused\","
                 << "\"pid\":" << process_runtime_.pid() << ","
                 << "\"building_id\":" << json_string(building_id) << ","
                 << "\"floor_id\":" << json_string(floor_id) << ","
                 << "\"map_id\":" << json_string(selected_map->map_id) << ","
                 << "\"display_name\":" << json_string(selected_map->display_name) << ","
                 << "\"resume_navigation\":true,"
                 << "\"reused\":true,"
                 << "\"nav_map_yaml\":" << json_string(assets.nav_map_yaml.string()) << ","
                 << "\"localizer_map_png\":" << json_string(assets.localizer_map_png.string()) << ","
                 << "\"localizer_params_yaml\":"
                 << json_string(assets.localizer_params_yaml.string()) << ","
                 << "\"log_file\":" << json_string(process_runtime_.resume_log_file()) << "}";
        return {200, "application/json", response.str()};
      }
      if (context_starting_fresh && runtime_owner_evident) {
        try {
          ports_.write_last_navigation_map_selection(
            *selected_map, "navigation_runtime_starting_reused");
        } catch (const std::exception & exception) {
          return {500, "application/json", error_json(exception.what())};
        }
        const std::string detail = existing_context->message.empty() ?
          "resident navigation runtime already starting for selected floor" :
          existing_context->message;
        ports_.set_navigation_runtime_state(true, "starting", detail, false);

        std::ostringstream response;
        response << "{\"ok\":true,"
                 << "\"state\":\"navigation_runtime_starting_reused\","
                 << "\"pid\":" << process_runtime_.pid() << ","
                 << "\"building_id\":" << json_string(building_id) << ","
                 << "\"floor_id\":" << json_string(floor_id) << ","
                 << "\"map_id\":" << json_string(selected_map->map_id) << ","
                 << "\"display_name\":" << json_string(selected_map->display_name) << ","
                 << "\"resume_navigation\":true,"
                 << "\"reused\":true,"
                 << "\"external_runtime\":"
                 << (existing_resume_process_running ? "false" : "true") << ","
                 << "\"nav_map_yaml\":" << json_string(assets.nav_map_yaml.string()) << ","
                 << "\"localizer_map_png\":" << json_string(assets.localizer_map_png.string()) << ","
                 << "\"localizer_params_yaml\":"
                 << json_string(assets.localizer_params_yaml.string()) << ","
                 << "\"log_file\":" << json_string(process_runtime_.resume_log_file()) << "}";
        return {202, "application/json", response.str()};
      }
    }

    if (const auto blocked =
      ports_.floor_runtime_interlock_response("navigation_runtime_launch"))
    {
      return *blocked;
    }
    process_runtime_.terminate_managed_process();
    if (selected_map) {
      try {
        ports_.write_last_navigation_map_selection(*selected_map, "navigation_resume_start");
      } catch (const std::exception & exception) {
        return {500, "application/json", error_json(exception.what())};
      }
      ports_.write_runtime_map_context(
        *selected_map, "starting", false, "navigation runtime start accepted");
    }

    NavigationProcessLaunchSpec launch_spec;
    launch_spec.building_id = building_id;
    launch_spec.floor_id = floor_id;
    launch_spec.environment = {
      {"NJRH_AMCL_RUNTIME_STATUS_FILE", localization_module_.amcl_runtime_status_file().string()},
      {"NJRH_NAVIGATION_RESUME_LOG_FILE", process_runtime_.resume_log_file()},
      {"NJRH_NAVIGATION_START_SOURCE", "api_resume"}};
    if (selected_map) {
      launch_spec.environment.emplace_back(
        "NJRH_RUNTIME_MAP_CONTEXT_FILE", runtime_map_context_file_.string());
      launch_spec.environment.emplace_back("NJRH_MAP_ID", selected_map->map_id);
      launch_spec.environment.emplace_back("NJRH_MAP_DISPLAY_NAME", selected_map->display_name);
      launch_spec.environment.emplace_back(
        "NJRH_MAP_CONTEXT_BUILDING_ID", selected_map->building_id);
      launch_spec.environment.emplace_back(
        "NJRH_MAP_CONTEXT_FLOOR_ID", selected_map->floor_id);
    }
    pid_t pid = -1;
    std::string launch_detail;
    if (!process_runtime_.launch_replacing(launch_spec, pid, launch_detail)) {
      return {500, "application/json", error_json(launch_detail)};
    }
    ports_.set_navigation_runtime_state(
      true, "starting", "navigation runtime start accepted", true);

    std::ostringstream response;
    response << "{\"ok\":true,"
             << "\"state\":\"navigation_resume_starting\","
             << "\"pid\":" << pid << ","
             << "\"building_id\":" << json_string(building_id) << ","
             << "\"floor_id\":" << json_string(floor_id) << ","
             << "\"map_id\":" << json_string(selected_map ? selected_map->map_id : "") << ","
             << "\"display_name\":"
             << json_string(selected_map ? selected_map->display_name : "") << ","
             << "\"resume_navigation\":true,"
             << "\"nav_map_yaml\":" << json_string(assets.nav_map_yaml.string()) << ","
             << "\"localizer_map_png\":" << json_string(assets.localizer_map_png.string()) << ","
             << "\"localizer_params_yaml\":"
             << json_string(assets.localizer_params_yaml.string()) << ","
             << "\"log_file\":" << json_string(process_runtime_.resume_log_file()) << "}";
    return {202, "application/json", response.str()};
  }

  bool runtime_context_matches_resume_request(
    const RuntimeMapContext & context,
    const std::string & building_id,
    const std::string & floor_id,
    const MapManifest & selected_map) const
  {
    return context.confirmed && context.state == "ready" &&
           context.map_id == selected_map.map_id &&
           context.building_id == building_id &&
           context.floor_id == floor_id;
  }

  bool runtime_context_same_resume_request(
    const RuntimeMapContext & context,
    const std::string & building_id,
    const std::string & floor_id,
    const MapManifest & selected_map) const
  {
    return context.map_id == selected_map.map_id &&
           context.building_id == building_id &&
           context.floor_id == floor_id;
  }

  bool runtime_context_starting_is_fresh(const RuntimeMapContext & context) const
  {
    if (context.state != "starting" || context.updated_at_sec <= 0.0) {
      return false;
    }
    const double age_sec = wall_time_seconds() - context.updated_at_sec;
    return age_sec >= 0.0 && age_sec <= resume_starting_context_ttl_sec_;
  }

private:
  static std::vector<std::string> lifecycle_node_names()
  {
    return {
      "/controller_server",
      "/planner_server",
      "/bt_navigator",
      "/behavior_server",
      "/local_costmap/local_costmap",
      "/global_costmap/global_costmap",
      "/velocity_smoother",
      "/collision_monitor"};
  }

public:
  rclcpp::Node & node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  features::maps::MapsModule & maps_module_;
  features::localization::LocalizationModule & localization_module_;
  features::elevator::ElevatorModule & elevator_module_;
  NavigationModulePorts ports_;
  std::filesystem::path maps_root_;
  std::filesystem::path runtime_map_context_file_;
  double resume_starting_context_ttl_sec_{300.0};
  std::mutex runtime_commit_mutex_;
  std::mutex goal_start_mutex_;
  std::mutex cancel_start_mutex_;
  bool nav2_native_goal_completion_enabled_{true};
  bool api_final_yaw_align_fallback_enabled_{true};
  bool navigation_final_yaw_align_enabled_{true};
  bool post_nav2_final_verify_enabled_{true};
  bool post_nav2_final_verify_wait_bridge_smoothing_{true};
  int post_nav2_final_verify_max_retry_count_{3};
  int navigation_nav2_failed_near_goal_retry_max_count_{1};
  bool post_nav2_final_verify_api_velocity_correction_enabled_{true};
  bool nav2_rotation_shim_enabled_{true};
  double navigation_final_yaw_align_timeout_sec_{8.0};
  double navigation_final_yaw_align_max_xy_drift_m_{0.08};
  std::string navigation_final_yaw_align_cmd_topic_{"/cmd_vel_api"};
  bool navigation_final_yaw_align_bypass_collision_monitor_{true};
  std::string position_only_nav2_yaw_mode_{"approach_heading"};
  std::chrono::nanoseconds cancel_action_wait_{std::chrono::milliseconds(750)};
  std::string action_name_;
  double lifecycle_check_timeout_sec_{1.5};
  std::map<std::string, rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr>
    lifecycle_clients_;

  NavigationActionRuntime action_runtime_;
  NavigationMissionRuntime mission_runtime_;
  NavigationCancelRuntime cancel_runtime_;
  NavigationProcessRuntime process_runtime_;
  NavigationGoalPolicy goal_policy_;
  NavigationCompletionPolicy completion_policy_;
  NavigationBridgeWait bridge_wait_;
  NavigationTerminalControl terminal_control_;
};

NavigationModule::NavigationModule(
  rclcpp::Node & node,
  rclcpp::CallbackGroup::SharedPtr callback_group,
  rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr
  global_costmap_lifecycle_client,
  features::maps::MapsModule & maps_module,
  features::localization::LocalizationModule & localization_module,
  features::elevator::ElevatorModule & elevator_module,
  NavigationModuleConfig config,
  NavigationModulePorts ports)
: impl_(std::make_unique<Impl>(
    node,
    std::move(callback_group),
    std::move(global_costmap_lifecycle_client),
    maps_module,
    localization_module,
    elevator_module,
    std::move(config),
    std::move(ports)))
{
}

NavigationModule::~NavigationModule() = default;

NavigationActionRuntime & NavigationModule::action_runtime() {return impl_->action_runtime_;}
NavigationMissionRuntime & NavigationModule::mission_runtime() {return impl_->mission_runtime_;}
NavigationCancelRuntime & NavigationModule::cancel_runtime() {return impl_->cancel_runtime_;}
NavigationProcessRuntime & NavigationModule::process_runtime() {return impl_->process_runtime_;}
NavigationGoalPolicy & NavigationModule::goal_policy() {return impl_->goal_policy_;}
NavigationCompletionPolicy & NavigationModule::completion_policy()
{
  return impl_->completion_policy_;
}
NavigationBridgeWait & NavigationModule::bridge_wait() {return impl_->bridge_wait_;}
NavigationTerminalControl & NavigationModule::terminal_control() {return impl_->terminal_control_;}
NavigationLifecycleSnapshot NavigationModule::lifecycle_snapshot()
{
  return impl_->lifecycle_snapshot();
}

NavigationLifecycleSnapshot NavigationModule::goal_admission_snapshot()
{
  return impl_->goal_admission_snapshot();
}

std::optional<HttpResponse> NavigationModule::handle_http(
  const HttpRequest & request,
  const ElevatorMotionAdmissionFence::Epoch motion_admission_epoch)
{
  return impl_->handle_http(request, motion_admission_epoch);
}

bool NavigationModule::cancel_active_goal(std::string & detail)
{
  return impl_->cancel_active_goal(detail);
}

bool NavigationModule::cancel_goal_for_api_handoff(
  const NavigationActionRuntime::GoalHandle::SharedPtr & goal_handle,
  std::string & detail)
{
  return impl_->cancel_goal_for_api_handoff(goal_handle, detail);
}

bool NavigationModule::cancel_task_for_mode_switch(
  const std::string & reason, std::string & detail)
{
  return impl_->cancel_task_for_mode_switch(reason, detail);
}

bool NavigationModule::request_goal_cancel(const std::string & reason)
{
  return impl_->request_goal_cancel(reason);
}

void NavigationModule::join_cancel_worker()
{
  impl_->cancel_runtime_.join();
}

std::string NavigationModule::cancel_job_json() const
{
  return impl_->cancel_runtime_.json();
}

HttpResponse NavigationModule::resume_floor_navigation(
  const std::string & building_id,
  const std::string & floor_id,
  const std::optional<MapManifest> & selected_map,
  const ElevatorMotionAdmissionFence::Epoch motion_admission_epoch)
{
  return impl_->handle_resume_floor_navigation(
    building_id, floor_id, selected_map, motion_admission_epoch);
}

void NavigationModule::refresh_runtime_state(const bool probe_lifecycle)
{
  impl_->refresh_runtime_state(probe_lifecycle);
}

bool NavigationModule::process_running()
{
  return impl_->process_running();
}

bool NavigationModule::stop_runtime_stack(std::string & detail)
{
  return impl_->stop_runtime_stack(detail);
}

}  // namespace robot_api_server::features::navigation
