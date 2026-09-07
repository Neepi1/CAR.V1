#include "robot_api_server/features/docking/lifecycle/docking_http_module.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <utility>

#include "robot_api_server/features/elevator/configuration/elevator_configuration_module.hpp"
#include "robot_api_server/features/docking/lifecycle/docking_status_utils.hpp"
#include "robot_api_server/features/localization/tf_pose_utils.hpp"
#include "robot_api_server/features/maps/catalog_activation/api_time_utils.hpp"
#include "robot_api_server/features/maps/catalog_activation/floor_asset_resolver.hpp"

namespace robot_api_server::features::docking
{
namespace
{

std::string json_nullable_number(const bool valid, const double value)
{
  if (!valid || !std::isfinite(value)) {
    return "null";
  }
  std::ostringstream out;
  out << std::fixed << std::setprecision(6) << value;
  return out.str();
}

std::string json_string_array_fragment(const std::vector<std::string> & values)
{
  std::ostringstream out;
  out << "[";
  for (std::size_t index = 0; index < values.size(); ++index) {
    if (index != 0U) {
      out << ",";
    }
    out << json_string(values[index]);
  }
  out << "]";
  return out.str();
}

HttpResponse elevator_internal_pose_requires_mission_response(const std::string & detail)
{
  std::ostringstream response;
  response << "{\"ok\":false,"
           << "\"code\":\"ELEVATOR_INTERNAL_POSE_REQUIRES_MISSION\","
           << "\"detail\":" << json_string(detail) << "}";
  return {409, "application/json", response.str()};
}

HttpResponse floor_switch_required_response(const int status, const std::string & detail)
{
  std::ostringstream response;
  response << "{\"ok\":false,\"code\":\"FLOOR_SWITCH_REQUIRED\","
           << "\"detail\":" << json_string(detail) << "}";
  return {status, "application/json", response.str()};
}

}  // namespace

class DockingHttpModule::Impl
{
public:
  Impl(
    features::maps::MapsModule & maps_module,
    features::elevator::ElevatorModule & elevator_module,
    application::runtime_mode::RuntimeModeCoordinator & runtime_mode,
    std::mutex & docking_start_mutex,
    DockingJobStore & job_store,
    DockingHttpConfig config,
    DockingHttpPorts ports)
  : maps_module_(maps_module),
    elevator_module_(elevator_module),
    runtime_mode_(runtime_mode),
    docking_start_mutex_(docking_start_mutex),
    docking_job_mutex_(job_store.mutex()),
    docking_job_(job_store.job_unsafe()),
    docking_job_sequence_(job_store.sequence_unsafe()),
    config_(std::move(config)),
    ports_(std::move(ports))
  {
  }

  std::optional<HttpResponse> handle_http(
    const HttpRequest & request,
    const ElevatorMotionAdmissionFence::Epoch motion_admission_epoch)
  {
    if (request.method == "GET" && request.path == "/api/v1/docking/state") {
      return handle_state();
    }
    if (request.method == "POST" && request.path == "/api/v1/docking/start") {
      return handle_start(request.body, motion_admission_epoch);
    }
    if (request.method == "POST" && request.path == "/api/v1/docking/undock") {
      return handle_undock(request.body, motion_admission_epoch);
    }
    if (request.method == "POST" && request.path == "/api/v1/docking/confirm_docked") {
      return handle_confirm_docked(request.body);
    }
    if (request.method == "POST" && request.path == "/api/v1/docking/clear_docked_latch") {
      return handle_clear_latch(request.body);
    }
    if (request.method == "POST" &&
      (request.path == "/api/v1/docking/cancel" || request.path == "/api/v1/docking/stop"))
    {
      return handle_cancel(request.body);
    }
    return std::nullopt;
  }

private:
  std::string docking_job_json_locked() const
  {
    return docking_job_json(docking_job_);
  }

  HttpResponse handle_start(
    const std::string & body,
    const ElevatorMotionAdmissionFence::Epoch motion_admission_epoch)
  {
    std::lock_guard<std::mutex> keepout_guard(maps_module_.mutation_mutex());
    if (const auto blocked = ports_.floor_runtime_interlock_response("docking_start")) {
      return *blocked;
    }
    if (maps_module_.integrity_degraded()) {
      return {
        503,
        "application/json",
        error_json("keepout integrity is degraded; docking admission is blocked until repaired")};
    }
    auto building_id = json_string_value(body, "building_id").value_or("building_1");
    auto floor_id = json_string_value(body, "floor_id");
    const auto map_id = json_string_value(body, "map_id");
    const auto map_name = json_string_value(body, "map_name");
    const auto dock_id = json_string_value(body, "dock_id").value_or(
      json_string_value(body, "pose_id").value_or(json_string_value(body, "id").value_or("")));
    const auto requested_predock_pose_id = json_string_value(body, "predock_pose_id").value_or(
      json_string_value(body, "approach_pose_id").value_or(""));
    const bool resume_navigation = json_bool_value(body, "resume_navigation", true);
    const double approach_distance = std::clamp(
      json_number_value(body, "approach_distance_m").value_or(config_.pre_dock_distance_m),
      0.10,
      2.00);

    const auto docking_start_runtime = runtime_mode_.snapshot();
    if (docking_start_runtime.mapping_active || ports_.mapping_start_job_running() ||
      runtime_mode_.transition_owner() == "mapping_start")
    {
      return {
        409,
        "application/json",
        error_json("docking is unavailable while 2D mapping is active or starting")};
    }

    if (!floor_id || floor_id->empty()) {
      return {400, "application/json", error_json("floor_id is required")};
    }
    if (!safe_asset_id(building_id) || !safe_asset_id(*floor_id)) {
      return {400, "application/json", error_json("building_id/floor_id must be safe asset ids")};
    }
    if (!safe_pose_id(dock_id)) {
      return {400, "application/json", error_json("valid dock_id is required")};
    }

    std::optional<MapManifest> selected_map;
    try {
      if (map_id && !map_id->empty()) {
        selected_map = maps_module_.catalog().find_map_by_id(*map_id);
        if (!selected_map) {
          return {404, "application/json", error_json("map_id not found: " + *map_id)};
        }
        building_id = selected_map->building_id;
        floor_id = selected_map->floor_id;
      } else if (map_name && !map_name->empty()) {
        std::string error;
        selected_map = maps_module_.catalog().find_floor_map_by_name(
          building_id, *floor_id, *map_name, error);
        if (!error.empty()) {
          return {409, "application/json", error_json(error)};
        }
        if (!selected_map) {
          return {
            404,
            "application/json",
            error_json("map_name not found on requested floor: " + *map_name)};
        }
      } else {
        selected_map = maps_module_.catalog().active_floor_map(building_id, *floor_id);
      }
    } catch (const std::exception & exc) {
      return {500, "application/json", error_json(exc.what())};
    }
    if (!selected_map) {
      return floor_switch_required_response(
        503,
        "docking requires a selected map and a confirmed ready runtime map context");
    }
    const auto docking_runtime_context = ports_.read_runtime_map_context();
    if (!docking_runtime_context || !docking_runtime_context->confirmed ||
      docking_runtime_context->state != "ready")
    {
      return floor_switch_required_response(
        503,
        "docking requires a confirmed ready runtime map context; complete the floor "
        "switch before retrying");
    }
    std::string docking_context_error;
    if (!selected_map->active) {
      return floor_switch_required_response(
        409,
        "requested docking map is not the active manifest for its floor");
    }
    if (!maps_module_.runtime_context_matches(*selected_map, docking_context_error)) {
      return floor_switch_required_response(
        409,
        "requested docking map is not the confirmed active runtime map: " +
        docking_context_error);
    }

    const auto pose = find_floor_catalog_pose(
      maps_module_.catalog(), building_id, *floor_id, dock_id);
    if (!pose) {
      return {404, "application/json", error_json("dock_id not found in poses.yaml: " + dock_id)};
    }
    if (is_elevator_internal_pose_type(pose->type) || is_reserved_elevator_pose_id(pose->id)) {
      return elevator_internal_pose_requires_mission_response(
        "elevator internal poses cannot be used as docking targets");
    }

    std::string predock_source;
    std::string predock_error;
    int predock_error_status = 0;
    const auto predock_pose = ports_.resolve_predock_pose(
      building_id,
      *floor_id,
      dock_id,
      *pose,
      requested_predock_pose_id,
      predock_source,
      predock_error,
      predock_error_status);
    if (!predock_error.empty()) {
      return {
        predock_error_status == 0 ? 409 : predock_error_status,
        "application/json",
        error_json(predock_error)};
    }
    if (predock_pose &&
      (is_elevator_internal_pose_type(predock_pose->type) ||
      is_reserved_elevator_pose_id(predock_pose->id)))
    {
      return elevator_internal_pose_requires_mission_response(
        "elevator internal poses cannot be used as docking pre-approach targets");
    }
    if (predock_pose && !ports_.validate_predock_pose(*pose, *predock_pose, predock_error)) {
      return {409, "application/json", error_json(predock_error)};
    }

    DockingJob next_job;
    next_job.id = 0U;
    next_job.state = "running";
    next_job.phase = "accepted";
    next_job.building_id = building_id;
    next_job.floor_id = *floor_id;
    next_job.map_id = selected_map->map_id;
    next_job.dock_id = dock_id;
    next_job.dock_name = pose->name;
    next_job.dock_type = pose->type;
    next_job.dock_profile_id = config_.default_dock_profile_id;
    next_job.dock_profile_type = config_.default_dock_profile_type;
    next_job.approach_direction = config_.default_approach_direction;
    next_job.contact_frame = config_.default_contact_frame;
    next_job.sensor_frame = config_.default_sensor_frame;
    next_job.goal_completion_policy = "dock_staging";
    next_job.max_retries = config_.max_retries;
    next_job.started_at = utc_timestamp_iso8601();
    next_job.resume_navigation = resume_navigation;
    next_job.dock_x = pose->x;
    next_job.dock_y = pose->y;
    next_job.dock_yaw = normalize_angle(pose->yaw);
    if (predock_pose) {
      next_job.predock_pose_id = predock_pose->id;
      next_job.approach_source = predock_source;
      next_job.approach_x = predock_pose->x;
      next_job.approach_y = predock_pose->y;
      next_job.approach_yaw = normalize_angle(predock_pose->yaw);
      next_job.approach_distance_m =
        std::hypot(next_job.dock_x - next_job.approach_x, next_job.dock_y - next_job.approach_y);
    } else {
      next_job.approach_source = "computed_from_dock_pose";
      next_job.approach_distance_m = approach_distance;
      next_job.approach_x = next_job.dock_x - std::cos(next_job.dock_yaw) * approach_distance;
      next_job.approach_y = next_job.dock_y - std::sin(next_job.dock_yaw) * approach_distance;
      next_job.approach_yaw = next_job.dock_yaw;
    }

    std::lock_guard<std::mutex> start_lock(docking_start_mutex_);
    {
      std::lock_guard<std::mutex> lock(docking_job_mutex_);
      if (docking_job_.state == "running") {
        std::ostringstream response;
        response << "{\"ok\":true,\"accepted\":true,\"already_running\":true,"
                 << "\"docking\":" << docking_job_json_locked() << "}";
        return {202, "application/json", response.str()};
      }
    }
    ports_.join_docking_worker();
    auto motion_admission = elevator_module_.acquire_motion_admission(motion_admission_epoch);
    if (!motion_admission.admitted()) {
      return elevator_module_.motion_admission_failure_response(
        "docking_start", motion_admission);
    }
    if (const auto blocked = ports_.floor_runtime_interlock_response("docking_start_commit")) {
      return *blocked;
    }
    const auto commit_map = maps_module_.catalog().find_map_by_id(selected_map->map_id);
    if (!commit_map) {
      return floor_switch_required_response(
        409,
        "requested docking map disappeared while admission was pending");
    }
    if (!commit_map->active) {
      return floor_switch_required_response(
        409,
        "requested docking map became inactive while admission was pending");
    }
    std::string commit_context_error;
    if (!maps_module_.runtime_context_matches(*commit_map, commit_context_error)) {
      return floor_switch_required_response(
        409,
        "runtime map context changed while docking admission was pending: " +
        commit_context_error);
    }
    std::uint64_t job_id = 0U;
    {
      std::lock_guard<std::mutex> lock(docking_job_mutex_);
      job_id = ++docking_job_sequence_;
      next_job.id = job_id;
      docking_job_ = next_job;
    }
    runtime_mode_.accept_docking(dock_id, "docking accepted");
    motion_admission.unlock();

    std::string launch_error;
    if (!ports_.launch_docking_worker(job_id, launch_error)) {
      ports_.finish_docking_job(job_id, false, "failed", launch_error);
      return {500, "application/json", error_json(launch_error)};
    }

    std::lock_guard<std::mutex> lock(docking_job_mutex_);
    std::ostringstream response;
    response << "{\"ok\":true,\"accepted\":true,\"docking\":"
             << docking_job_json_locked() << "}";
    return {202, "application/json", response.str()};
  }

  HttpResponse handle_cancel(const std::string & body);
  HttpResponse handle_confirm_docked(const std::string & body);
  HttpResponse handle_clear_latch(const std::string & body);
  HttpResponse handle_undock(
    const std::string & body,
    ElevatorMotionAdmissionFence::Epoch motion_admission_epoch);
  HttpResponse handle_state();

  features::maps::MapsModule & maps_module_;
  features::elevator::ElevatorModule & elevator_module_;
  application::runtime_mode::RuntimeModeCoordinator & runtime_mode_;
  std::mutex & docking_start_mutex_;
  std::mutex & docking_job_mutex_;
  DockingJob & docking_job_;
  std::uint64_t & docking_job_sequence_;
  DockingHttpConfig config_;
  DockingHttpPorts ports_;
};

HttpResponse DockingHttpModule::Impl::handle_cancel(const std::string & body)
{
  const auto reason = json_string_value(body, "reason").value_or("app_docking_cancel");
  ports_.clear_teleop_command();
  ports_.publish_teleop_zero_burst();
  std::uint64_t job_id = 0U;
  {
    std::lock_guard<std::mutex> lock(docking_job_mutex_);
    docking_job_.cancel_requested = true;
    docking_job_.detail = reason;
    job_id = docking_job_.id;
  }
  std::string nav_detail;
  ports_.cancel_active_navigation_goal(nav_detail);
  std::string stop_detail;
  (void)ports_.stop_docking_if_available(stop_detail);
  std::string correction_resume_detail;
  (void)ports_.set_global_correction_paused(
    job_id,
    false,
    "docking_canceled",
    correction_resume_detail);
  ports_.publish_teleop_zero_burst();
  if (job_id != 0U) {
    ports_.finish_docking_job(
      job_id,
      true,
      "canceled",
      reason + "; " + nav_detail + "; " + stop_detail + "; " + correction_resume_detail);
  } else {
    runtime_mode_.set_docking(false, "canceled", reason + "; " + stop_detail);
  }
  std::lock_guard<std::mutex> lock(docking_job_mutex_);
  std::ostringstream response;
  response << "{\"ok\":true,\"accepted\":true,\"navigation_cancel_detail\":"
           << json_string(nav_detail) << ",\"docking_stop_detail\":" << json_string(stop_detail)
           << ",\"docking\":" << docking_job_json_locked() << "}";
  return {202, "application/json", response.str()};
}

HttpResponse DockingHttpModule::Impl::handle_confirm_docked(const std::string & body)
{
  auto building_id = json_string_value(body, "building_id").value_or("");
  auto floor_id = json_string_value(body, "floor_id").value_or("");
  auto map_id = json_string_value(body, "map_id").value_or("");
  const auto dock_id = json_string_value(body, "dock_id").value_or("");
  const auto note = json_string_value(body, "note").value_or("");
  const auto reason = json_string_value(body, "reason").value_or("manual_confirm");
  if (!building_id.empty() && !safe_asset_id(building_id)) {
    return {400, "application/json", error_json("valid building_id is required")};
  }
  if (!floor_id.empty() && !safe_asset_id(floor_id)) {
    return {400, "application/json", error_json("valid floor_id is required")};
  }
  if (!map_id.empty() && !safe_asset_id(map_id)) {
    return {400, "application/json", error_json("valid map_id is required")};
  }
  if (!dock_id.empty() && !safe_pose_id(dock_id)) {
    return {400, "application/json", error_json("dock_id must be a safe pose id when provided")};
  }
  if (auto context = ports_.read_runtime_map_context()) {
    if (building_id.empty()) {
      building_id = context->building_id;
    }
    if (floor_id.empty()) {
      floor_id = context->floor_id;
    }
    if (map_id.empty()) {
      map_id = context->map_id;
    }
  }
  ports_.update_dock_contact_latch(
    true,
    "manual_confirm",
    reason,
    dock_id,
    building_id,
    floor_id,
    map_id,
    note);
  std::ostringstream response;
  response << "{\"ok\":true,"
           << "\"maintenance_only\":true,"
           << "\"sent_velocity\":false,"
           << "\"latch\":" << ports_.dock_contact_latch_json() << "}";
  return {200, "application/json", response.str()};
}

HttpResponse DockingHttpModule::Impl::handle_clear_latch(const std::string & body)
{
  const auto reason = json_string_value(body, "reason").value_or("manual_clear");
  const auto note = json_string_value(body, "note").value_or("");
  ports_.update_dock_contact_latch(false, "manual_clear", reason, "", "", "", "", note);
  std::ostringstream response;
  response << "{\"ok\":true,"
           << "\"maintenance_only\":true,"
           << "\"sent_velocity\":false,"
           << "\"latch\":" << ports_.dock_contact_latch_json() << "}";
  return {200, "application/json", response.str()};
}

HttpResponse DockingHttpModule::Impl::handle_undock(
  const std::string & body,
  const ElevatorMotionAdmissionFence::Epoch motion_admission_epoch)
{
  if (const auto blocked = ports_.floor_runtime_interlock_response("docking_undock")) {
    return *blocked;
  }
  const auto reason = json_string_value(body, "reason").value_or("app_manual_undock");
  auto dock_id = json_string_value(body, "dock_id").value_or("");
  if (!dock_id.empty() && !safe_pose_id(dock_id)) {
    return {400, "application/json", error_json("dock_id must be a safe pose id when provided")};
  }

  ports_.clear_teleop_command();
  ports_.publish_teleop_zero_burst();

  std::lock_guard<std::mutex> start_lock(docking_start_mutex_);
  bool takeover_required = false;
  {
    std::lock_guard<std::mutex> lock(docking_job_mutex_);
    if (docking_job_.state == "running") {
      if (docking_job_.phase == "undocking") {
        std::ostringstream response;
        response << "{\"ok\":true,\"accepted\":true,\"already_running\":true,"
                 << "\"api_accepted\":true,"
                 << "\"docking_service_called\":false,"
                 << "\"docking_service_success\":false,"
                 << "\"docking_service_message\":"
                 << json_string("already running; no new /docking/undock service call") << ","
                 << "\"docking_status_at_request\":" << json_string(docking_job_.last_status) << ","
                 << "\"docking_status_after_request\":" << json_string(docking_job_.last_status) << ","
                 << "\"undock_started_observed\":"
                 << (docking_job_.undock_started_observed ? "true" : "false") << ","
                 << "\"undock_cmd_count_observed\":" << docking_job_.undock_cmd_count_observed << ","
                 << "\"undock_failure_reason\":" << json_string(docking_job_.undock_failure_reason) << ","
                 << "\"docking\":" << docking_job_json_locked() << "}";
        return {202, "application/json", response.str()};
      }
      takeover_required = true;
    }
    if (dock_id.empty()) {
      dock_id = docking_job_.dock_id;
    }
  }

  if (takeover_required) {
    std::string takeover_detail;
    if (!ports_.prepare_controlled_undock ||
      !ports_.prepare_controlled_undock(takeover_detail))
    {
      return {
        503,
        "application/json",
        error_json(
          takeover_detail.empty() ?
          "failed to safely retire active docking owner before undock" :
          takeover_detail)};
    }
  }

  ports_.join_docking_worker();

  const auto runtime = runtime_mode_.snapshot();
  if (dock_id.empty()) {
    dock_id = runtime.docking_dock_id;
  }
  const auto charging_contact_snapshot = ports_.bms_charging_contact_snapshot();
  const bool charging_contact = charging_contact_snapshot.contact;
  const auto dock_check = ports_.docking_occupancy_snapshot();
  const bool docked_state = dock_check.final_is_docked_or_charging ||
    runtime.docking_state == "docked" ||
    docking_status_is_success(runtime.docking_status) ||
    dock_check.dock_latch_indicates_docked;
  if (!docked_state && !charging_contact) {
    std::ostringstream response;
    response << "{\"ok\":true,\"accepted\":true,"
             << "\"already_undocked\":true,"
             << "\"sent_velocity\":false,"
             << "\"detail\":\"no retained dock evidence after owner cleanup\","
             << "\"dock_occupancy_state\":"
             << json_string(dock_check.dock_occupancy_state) << "}";
    return {200, "application/json", response.str()};
  }

  if (const auto blocked = ports_.floor_runtime_interlock_response("docking_undock_commit")) {
    return *blocked;
  }
  auto motion_admission = elevator_module_.acquire_motion_admission(motion_admission_epoch);
  if (!motion_admission.admitted()) {
    return elevator_module_.motion_admission_failure_response(
      "docking_undock", motion_admission);
  }
  std::string ensure_detail;
  if (!ports_.ensure_docking_manager_running(ensure_detail)) {
    return {500, "application/json", error_json(ensure_detail)};
  }

  DockingJob next_job;
  next_job.state = "running";
  next_job.phase = "undocking";
  next_job.dock_id = dock_id;
  next_job.detail = reason;
  next_job.last_status = "undocking accepted";
  next_job.started_at = utc_timestamp_iso8601();
  next_job.resume_navigation = false;
  next_job.api_accepted = true;
  next_job.already_running = false;
  next_job.docking_status_at_request = runtime.docking_status;

  std::uint64_t job_id = 0U;
  {
    std::lock_guard<std::mutex> lock(docking_job_mutex_);
    job_id = ++docking_job_sequence_;
    next_job.id = job_id;
    docking_job_ = next_job;
  }
  runtime_mode_.set_docking(true, "undocking", "undocking accepted");
  runtime_mode_.set_docking_identity(dock_id, "undocking accepted");
  motion_admission.unlock();

  std::string service_detail;
  DockingUndockServiceObservation service_observation;
  if (const auto blocked = ports_.floor_runtime_interlock_response("docking_undock_submit")) {
    ports_.finish_docking_job(job_id, false, "failed", blocked->body);
    return *blocked;
  }
  if (!ports_.call_undock_service_with_charging_retry(
      service_detail, charging_contact, &service_observation))
  {
    const auto after_status = runtime_mode_.snapshot().docking_status;
    {
      std::lock_guard<std::mutex> lock(docking_job_mutex_);
      if (docking_job_.id == job_id) {
        docking_job_.api_accepted = false;
        docking_job_.docking_service_called = service_observation.service_called;
        docking_job_.docking_service_success = service_observation.service_success;
        docking_job_.docking_service_message = service_observation.message;
        ports_.record_undock_status_observation(docking_job_, after_status);
      }
    }
    ports_.finish_docking_job(job_id, false, "failed", service_detail);
    std::string docking_json;
    {
      std::lock_guard<std::mutex> lock(docking_job_mutex_);
      docking_json = docking_job_json_locked();
    }
    std::ostringstream response;
    response << "{\"ok\":false,\"accepted\":false,\"api_accepted\":false,"
             << "\"already_running\":false,"
             << "\"docking_service_called\":"
             << (service_observation.service_called ? "true" : "false") << ","
             << "\"docking_service_success\":"
             << (service_observation.service_success ? "true" : "false") << ","
             << "\"docking_service_message\":" << json_string(service_observation.message) << ","
             << "\"docking_status_at_request\":" << json_string(runtime.docking_status) << ","
             << "\"docking_status_after_request\":" << json_string(after_status) << ","
             << "\"error\":" << json_string(service_detail) << ","
             << "\"docking\":" << docking_json << "}";
    return {409, "application/json", response.str()};
  }
  const auto after_status = runtime_mode_.snapshot().docking_status;
  {
    std::lock_guard<std::mutex> lock(docking_job_mutex_);
    if (docking_job_.id == job_id && docking_job_.state == "running") {
      docking_job_.docking_service_called = service_observation.service_called;
      docking_job_.docking_service_success = service_observation.service_success;
      docking_job_.docking_service_message = service_observation.message;
      docking_job_.detail = service_detail;
      docking_job_.last_status = service_detail;
      ports_.record_undock_status_observation(docking_job_, after_status);
      if (docking_job_.docking_service_success && !docking_job_.undock_started_observed) {
        docking_job_.docking_service_warning =
          "service_success_without_undocking_status_observed_yet";
      }
    }
  }
  runtime_mode_.set_docking(true, "undocking", service_detail);

  std::lock_guard<std::mutex> lock(docking_job_mutex_);
  std::ostringstream response;
  response << "{\"ok\":true,\"accepted\":true,"
           << "\"api_accepted\":true,"
           << "\"already_running\":false,"
           << "\"docking_service_called\":"
           << (docking_job_.docking_service_called ? "true" : "false") << ","
           << "\"docking_service_success\":"
           << (docking_job_.docking_service_success ? "true" : "false") << ","
           << "\"docking_service_message\":" << json_string(docking_job_.docking_service_message) << ","
           << "\"docking_status_at_request\":" << json_string(docking_job_.docking_status_at_request) << ","
           << "\"docking_status_after_request\":" << json_string(docking_job_.docking_status_after_request) << ","
           << "\"undock_started_observed\":"
           << (docking_job_.undock_started_observed ? "true" : "false") << ","
           << "\"undock_cmd_count_observed\":" << docking_job_.undock_cmd_count_observed << ","
           << "\"undock_failure_reason\":" << json_string(docking_job_.undock_failure_reason) << ","
           << "\"docking_service_warning\":" << json_string(docking_job_.docking_service_warning) << ","
           << "\"docking\":" << docking_job_json_locked() << "}";
  return {202, "application/json", response.str()};
}

HttpResponse DockingHttpModule::Impl::handle_state()
{
  const auto runtime = runtime_mode_.snapshot();
  const auto charging_contact = ports_.bms_charging_contact_snapshot();
  const auto dock_check = ports_.docking_occupancy_snapshot();
  std::lock_guard<std::mutex> lock(docking_job_mutex_);
  std::ostringstream response;
  response << "{\"ok\":true,"
           << "\"mode\":" << json_string(runtime.mode) << ","
           << "\"state\":" << json_string(runtime.docking_state) << ","
           << "\"docking_active\":" << (runtime.docking_active ? "true" : "false") << ","
           << "\"docking_normal_path_relocalization_enabled\":false,"
           << "\"docking_predock_triggered_relocalization\":false,"
           << "\"charging_contact\":" << (charging_contact.contact ? "true" : "false") << ","
           << "\"charging_contact_reason\":" << json_string(charging_contact.reason) << ","
           << "\"inferred_docked\":" << (dock_check.inferred_docked ? "true" : "false") << ","
           << "\"dock_occupancy_state\":" << json_string(dock_check.dock_occupancy_state) << ","
           << "\"dock_occupancy_evidence\":"
           << json_string_array_fragment(dock_check.dock_occupancy_evidence) << ","
           << "\"dock_occupancy_reason\":" << json_string(dock_check.dock_occupancy_reason) << ","
           << "\"charging_session_latched\":"
           << (dock_check.charging_session_latched ? "true" : "false") << ","
           << "\"charging_session_age_sec\":"
           << json_nullable_number(
             dock_check.charging_session_age_sec >= 0.0,
             dock_check.charging_session_age_sec) << ","
           << "\"charging_session_last_confirmed_at\":"
           << json_string(dock_check.charging_session_last_confirmed_at) << ","
           << "\"full_charge_idle_on_dock\":"
           << (dock_check.full_charge_idle_on_dock ? "true" : "false") << ","
           << "\"can_undock\":"
           << (dock_check.final_is_docked_or_charging ? "true" : "false") << ","
           << "\"can_auto_undock\":" << (dock_check.can_auto_undock ? "true" : "false") << ","
           << "\"auto_undock_reason\":" << json_string(dock_check.auto_undock_reason) << ","
           << "\"last_status\":" << json_string(runtime.docking_status) << ","
           << "\"api_accepted\":" << (docking_job_.api_accepted ? "true" : "false") << ","
           << "\"already_running\":" << (docking_job_.already_running ? "true" : "false") << ","
           << "\"docking_service_called\":"
           << (docking_job_.docking_service_called ? "true" : "false") << ","
           << "\"docking_service_success\":"
           << (docking_job_.docking_service_success ? "true" : "false") << ","
           << "\"docking_service_message\":" << json_string(docking_job_.docking_service_message) << ","
           << "\"docking_status_at_request\":" << json_string(docking_job_.docking_status_at_request) << ","
           << "\"docking_status_after_request\":" << json_string(docking_job_.docking_status_after_request) << ","
           << "\"undock_started_observed\":"
           << (docking_job_.undock_started_observed ? "true" : "false") << ","
           << "\"undock_cmd_count_observed\":" << docking_job_.undock_cmd_count_observed << ","
           << "\"undock_failure_reason\":" << json_string(docking_job_.undock_failure_reason) << ","
           << "\"docking_service_warning\":" << json_string(docking_job_.docking_service_warning) << ","
           << "\"pre_navigation_dock_check\":" << dock_check.json << ","
           << "\"docking\":" << docking_job_json_locked() << "}";
  return {200, "application/json", response.str()};
}

DockingHttpModule::DockingHttpModule(
  features::maps::MapsModule & maps_module,
  features::elevator::ElevatorModule & elevator_module,
  application::runtime_mode::RuntimeModeCoordinator & runtime_mode,
  std::mutex & docking_start_mutex,
  DockingJobStore & job_store,
  DockingHttpConfig config,
  DockingHttpPorts ports)
: impl_(std::make_unique<Impl>(
      maps_module,
      elevator_module,
      runtime_mode,
      docking_start_mutex,
      job_store,
      std::move(config),
      std::move(ports)))
{
}

DockingHttpModule::~DockingHttpModule() = default;

std::optional<HttpResponse> DockingHttpModule::handle_http(
  const HttpRequest & request,
  const ElevatorMotionAdmissionFence::Epoch motion_admission_epoch)
{
  return impl_->handle_http(request, motion_admission_epoch);
}

}  // namespace robot_api_server::features::docking
