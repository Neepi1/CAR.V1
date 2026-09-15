#include "robot_api_server/features/mapping/mapping_module.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <future>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <utility>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "std_srvs/srv/set_bool.hpp"

#include "robot_api_server/features/mapping/map_asset_writer.hpp"
#include "robot_api_server/features/mapping/runtime/mapping_start_job.hpp"
#include "robot_api_server/features/mapping/runtime/mapping_save_job.hpp"
#include "robot_api_server/features/maps/catalog_activation/api_time_utils.hpp"
#include "robot_api_server/features/maps/catalog_activation/file_utils.hpp"
#include "robot_api_server/features/maps/catalog_activation/map_asset_filesystem.hpp"
#include "robot_api_server/features/maps/catalog_activation/map_asset_identity_binding.hpp"
#include "robot_api_server/features/maps/catalog_activation/map_asset_io.hpp"
#include "robot_api_server/features/maps/catalog_activation/runtime_map_lookup.hpp"
#include "robot_api_server/features/maps/catalog_activation/storage_models.hpp"

namespace robot_api_server::features::mapping
{

namespace fs = std::filesystem;
using namespace std::chrono_literals;
using application::runtime_mode::RuntimeModeCoordinator;
using runtime::MappingProcessRuntime;
using runtime::MappingRuntimeLogLevel;
using runtime::MappingStartJobTracker;

namespace
{

std::optional<std::string> query_value(
  const HttpRequest & request,
  const std::string & key)
{
  const auto it = request.query.find(key);
  if (it == request.query.end() || it->second.empty()) {
    return std::nullopt;
  }
  return it->second;
}

std::string motion_admission_failure_detail(
  const std::string & operation,
  const ElevatorMotionAdmissionFence::AdmissionGuard & admission)
{
  if (admission.stale()) {
    return operation + " request predates the completed elevator admission barrier";
  }
  if (admission.interlock().delayed_side_effect_unknown()) {
    return operation + " is blocked because " +
           std::to_string(admission.interlock().delayed_side_effect_unknown_count) +
           " timed-out motion/runtime submission(s) have no proven outcome";
  }
  if (admission.interlock().recovery_required()) {
    return operation +
           " is blocked by retained elevator recovery state; transaction_id=" +
           admission.interlock().transaction_id;
  }
  return operation + " is blocked during elevator execution; transaction_id=" +
         admission.interlock().transaction_id;
}

HttpResponse motion_admission_failure_response(
  const std::string & operation,
  const ElevatorMotionAdmissionFence::AdmissionGuard & admission)
{
  const std::string code = admission.stale() ?
    "ELEVATOR_MOTION_ADMISSION_STALE" :
    (admission.interlock().delayed_side_effect_unknown() ?
    "DELAYED_SIDE_EFFECT_UNKNOWN" :
    (admission.interlock().recovery_required() ?
    "ELEVATOR_EXECUTION_RECOVERY_REQUIRED" :
    "ELEVATOR_EXECUTION_ACTIVE"));
  std::ostringstream body;
  body << "{\"ok\":false,\"code\":" << json_string(code)
       << ",\"transaction_id\":"
       << json_string(admission.interlock().transaction_id)
       << ",\"delayed_side_effect_unknown_count\":"
       << admission.interlock().delayed_side_effect_unknown_count
       << ",\"detail\":"
       << json_string(motion_admission_failure_detail(operation, admission))
       << "}";
  return {409, "application/json", body.str()};
}

void require_ports(const MappingModulePorts & ports)
{
  if (!ports.floor_runtime_operation_blocked ||
    !ports.floor_runtime_interlock_response ||
    !ports.map_asset_integrity_degraded ||
    !ports.acquire_motion_admission ||
    !ports.navigation_goal_running ||
    !ports.cancel_navigation ||
    !ports.stop_navigation_runtime ||
    !ports.clear_runtime_map_context)
  {
    throw std::invalid_argument("mapping module requires every cross-domain port");
  }
}

}  // namespace

class MappingModule::Impl
{
public:
  Impl(
    rclcpp::Node & node,
    rclcpp::CallbackGroup::SharedPtr callback_group,
    RuntimeModeCoordinator & runtime_mode,
    MapCatalog & map_catalog,
    std::mutex & map_asset_mutation_mutex,
    MappingModuleConfig config,
    MappingModulePorts ports)
  : node_(node),
    callback_group_(std::move(callback_group)),
    runtime_mode_(runtime_mode),
    map_catalog_(map_catalog),
    map_asset_mutation_mutex_(map_asset_mutation_mutex),
    config_(std::move(config)),
    ports_(std::move(ports)),
    save_job_(config_.runtime_maps_dir / "save_jobs"),
    process_runtime_(
      config_.process,
      [this](
        const bool active,
        const std::string & state,
        const std::string & detail,
        const bool healthy)
      {
        set_mapping_runtime_state(active, state, detail, healthy);
      },
      [this](const MappingRuntimeLogLevel level, const std::string & message) {
        if (level == MappingRuntimeLogLevel::Warning) {
          RCLCPP_WARN(node_.get_logger(), "%s", message.c_str());
        } else {
          RCLCPP_INFO(node_.get_logger(), "%s", message.c_str());
        }
      })
  {
    require_ports(ports_);
    if (!callback_group_) {
      throw std::invalid_argument("mapping module requires a ROS callback group");
    }
    scan_control_client_ = node_.create_client<std_srvs::srv::SetBool>(
      config_.resident_scan_control_service,
      rmw_qos_profile_services_default,
      callback_group_);
  }

  ~Impl()
  {
    shutdown();
  }

  std::optional<HttpResponse> handle_http(
    const HttpRequest & request,
    const ElevatorMotionAdmissionFence::Epoch motion_admission_epoch)
  {
    if (request.method == "GET" && request.path == "/api/v1/mapping/2d/save/status") {
      auto result = save_job_.find(query_value(request, "request_id").value_or(""));
      if (json_string_value(result.body, "state") == "recovery_required" &&
        json_bool_value(result.body, "map_saved", false))
      {
        const auto saved = json_object_value(result.body, "result").value_or("{}");
        try {
          const auto manifest = map_catalog_.find_map_by_id(
            json_string_value(saved, "map_id").value_or(""));
          if (!manifest || json_string_value(saved, "asset_digest") != manifest->asset_digest) {
            throw std::runtime_error("saved map identity no longer matches the receipt");
          }
          verify_map_asset_identity(*manifest, config_.maps_root);
        } catch (const std::exception & error) {
          const std::string marker = "\"map_saved\":true";
          result.body.replace(result.body.find(marker), marker.size(), "\"map_saved\":false");
          result.body.pop_back();
          result.body += ",\"asset_verification_error\":" + json_string(error.what()) + "}";
        }
      }
      return result;
    }
    if (request.method == "GET" && request.path == "/api/v1/mapping/2d/map") {
      return handle_mapping_2d_map_png(request);
    }
    if (request.method == "POST" && request.path == "/api/v1/mapping/2d/start") {
      return handle_start_mapping_2d(motion_admission_epoch);
    }
    if (request.method == "POST" &&
      (request.path == "/api/v1/mapping/2d/stop" ||
      request.path == "/api/v1/mapping/stop"))
    {
      return handle_stop_mapping_2d();
    }
    if (request.method == "POST" &&
      (request.path == "/api/v1/mapping/2d/save" ||
      request.path == "/api/v1/mapping/save"))
    {
      if (json_bool_value(request.body, "async", false)) {
        return submit_mapping_save(request.body);
      }
      std::lock_guard<std::mutex> lock(start_mutex_);
      if (save_job_.running() || start_job_.running()) {
        return HttpResponse{409, "application/json", error_json("mapping start/save is still running")};
      }
      return handle_save_mapping_2d(request.body, motion_admission_epoch);
    }
    return std::nullopt;
  }

  MappingModuleSnapshot snapshot(const bool refresh_runtime)
  {
    if (refresh_runtime) {
      refresh_mapping_2d_runtime_state();
    }

    const auto process = refresh_runtime ?
      process_runtime_.recover_snapshot() : process_runtime_.tracked_snapshot();
    MappingModuleSnapshot result;
    result.process_active = process.active;
    result.process_running = process.running;
    result.start_job_running = start_job_.running();

    std::lock_guard<std::mutex> map_lock(live_map_mutex_);
    if (!have_live_map_) {
      return result;
    }
    result.live_map_available = true;
    result.live_map_age_sec = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - latest_live_map_received_at_).count();
    result.live_map_width = latest_live_map_.info.width;
    result.live_map_height = latest_live_map_.info.height;
    result.live_map_resolution = latest_live_map_.info.resolution;
    const auto known_cells = std::count_if(
      latest_live_map_.data.begin(), latest_live_map_.data.end(),
      [](const int8_t value) {return value >= 0;});
    result.known_area_m2 = static_cast<double>(known_cells) *
      result.live_map_resolution * result.live_map_resolution;
    return result;
  }

  std::string status_json()
  {
    refresh_mapping_2d_runtime_state();
    const auto runtime = runtime_mode_.snapshot();
    const auto state = snapshot(false);
    std::ostringstream body;
    body << "{\"active\":" << (runtime.mapping_active ? "true" : "false") << ",";
    body << "\"state\":" << json_string(runtime.mapping_state) << ",";
    body << "\"map_topic\":" << json_string(config_.live_map_topic) << ",";
    body << "\"map_endpoint\":\"/api/v1/mapping/2d/map\",";
    body << "\"live_map_available\":"
         << (state.live_map_available ? "true" : "false") << ",";
    if (state.live_map_available) {
      body << "\"live_map_age_sec\":" << state.live_map_age_sec << ",";
      body << "\"live_map_width\":" << state.live_map_width << ",";
      body << "\"live_map_height\":" << state.live_map_height;
    } else {
      body << "\"live_map_age_sec\":null,";
      body << "\"live_map_width\":0,";
      body << "\"live_map_height\":0";
    }
    body << ",\"start_job\":" << start_job_.json() << "}";
    return body.str();
  }

  void set_live_map_page_active(const bool active)
  {
    bool should_clear_cache = false;
    {
      std::lock_guard<std::mutex> lock(live_subscription_mutex_);
      live_map_page_subscription_active_ = active;
      reconcile_live_map_subscription_locked();
      should_clear_cache = !live_map_sub_;
    }
    if (should_clear_cache) {
      clear_live_map_cache();
    }
  }

  void shutdown()
  {
    if (shutdown_requested_.exchange(true, std::memory_order_acq_rel)) {
      return;
    }
    (void)start_job_.request_cancel("2D mapping start cancellation requested");
    join_mapping_start_worker();
    save_job_.join();
    {
      std::lock_guard<std::mutex> lock(live_subscription_mutex_);
      live_map_page_subscription_active_ = false;
      live_map_mapping_cache_active_ = false;
      live_map_sub_.reset();
    }
    clear_live_map_cache();
  }

private:
  struct ManifestLookupResult
  {
    bool ok{false};
    MapManifest manifest;
    HttpResponse error{400, "application/json", "{}"};
  };

  void set_mapping_runtime_state(
    const bool active,
    const std::string & state,
    const std::string & detail = "",
    const bool healthy = true)
  {
    runtime_mode_.set_mapping(active, state, detail, healthy);
  }

  void clear_live_map_cache()
  {
    std::lock_guard<std::mutex> map_lock(live_map_mutex_);
    latest_live_map_ = nav_msgs::msg::OccupancyGrid{};
    have_live_map_ = false;
    latest_live_map_received_at_ = {};
  }

  void reconcile_live_map_subscription_locked()
  {
    const bool required =
      live_map_page_subscription_active_ || live_map_mapping_cache_active_;
    if (required) {
      if (!live_map_sub_) {
        live_map_sub_ = node_.create_subscription<nav_msgs::msg::OccupancyGrid>(
          config_.live_map_topic,
          rclcpp::QoS(1).reliable(),
          [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
            std::lock_guard<std::mutex> map_lock(live_map_mutex_);
            latest_live_map_ = *msg;
            latest_live_map_received_at_ = std::chrono::steady_clock::now();
            have_live_map_ = true;
          });
      }
      return;
    }
    live_map_sub_.reset();
  }

  void set_mapping_live_map_cache_active(const bool active)
  {
    bool should_clear_cache = false;
    {
      std::lock_guard<std::mutex> lock(live_subscription_mutex_);
      live_map_mapping_cache_active_ = active;
      reconcile_live_map_subscription_locked();
      should_clear_cache = !live_map_sub_;
    }
    if (should_clear_cache) {
      clear_live_map_cache();
    }
  }

  ManifestLookupResult resolve_map_manifest_from_query(const HttpRequest & request)
  {
    const auto requested_building_id = query_value(request, "building_id");
    const auto requested_floor_id = query_value(request, "floor_id");
    const auto requested_map_id = query_value(request, "map_id");
    const auto requested_map_name = query_value(request, "map_name").value_or(
      query_value(request, "display_name").value_or(""));

    ManifestLookupResult result;
    if (requested_building_id && !safe_asset_id(*requested_building_id)) {
      result.error = {400, "application/json", error_json("valid building_id is required")};
      return result;
    }
    if (requested_floor_id && !safe_asset_id(*requested_floor_id)) {
      result.error = {400, "application/json", error_json("valid floor_id is required")};
      return result;
    }

    std::optional<MapManifest> manifest;
    if (requested_map_id) {
      if (!safe_asset_id(*requested_map_id)) {
        result.error = {400, "application/json", error_json("valid map_id is required")};
        return result;
      }
      manifest = map_catalog_.find_map_by_id(*requested_map_id);
      if (!manifest) {
        result.error = {
          404, "application/json", error_json("map_id not found: " + *requested_map_id)};
        return result;
      }
      if (requested_building_id && *requested_building_id != manifest->building_id) {
        result.error = {
          400, "application/json",
          error_json("map_id does not belong to requested building")};
        return result;
      }
      if (requested_floor_id && *requested_floor_id != manifest->floor_id) {
        result.error = {
          400, "application/json",
          error_json("map_id does not belong to requested floor")};
        return result;
      }
    } else {
      if (!requested_building_id) {
        result.error = {400, "application/json", error_json("valid building_id is required")};
        return result;
      }
      if (!requested_floor_id) {
        result.error = {400, "application/json", error_json("valid floor_id is required")};
        return result;
      }
      if (!requested_map_name.empty()) {
        if (!valid_display_map_name(requested_map_name)) {
          result.error = {400, "application/json", error_json("valid map_name is required")};
          return result;
        }
        std::string error;
        manifest = map_catalog_.find_floor_map_by_name(
          *requested_building_id,
          *requested_floor_id,
          requested_map_name,
          error);
        if (!manifest) {
          result.error = {
            error.empty() ? 404 : 400,
            "application/json",
            error_json(
              error.empty() ? "map_name not found: " + requested_map_name : error)};
          return result;
        }
      } else {
        manifest = map_catalog_.active_floor_map(
          *requested_building_id, *requested_floor_id);
        if (!manifest) {
          result.error = {
            404,
            "application/json",
            error_json(
              "active map not found for floor: " + *requested_building_id + "/" +
              *requested_floor_id)};
          return result;
        }
      }
    }

    result.ok = true;
    result.manifest = *manifest;
    return result;
  }

  HttpResponse handle_saved_mapping_2d_map_png(const HttpRequest & request)
  {
    if (request.query.find("map_id") != request.query.end()) {
      const auto requested_map_id = query_value(request, "map_id");
      if (!requested_map_id) {
        return {400, "application/json", error_json("valid map_id is required")};
      }

      const auto lookup = resolve_map_manifest_from_query(request);
      if (!lookup.ok) {
        return lookup.error;
      }
      const auto expected_root = map_catalog_.map_root_path(
        lookup.manifest.building_id,
        lookup.manifest.floor_id,
        lookup.manifest.map_id);
      if (!::robot_api_server::features::maps::same_normalized_path(
          lookup.manifest.root, expected_root) ||
        !::robot_api_server::features::maps::safe_bundle_regular_file(
          lookup.manifest.localizer_map_png, lookup.manifest.root))
      {
        return {
          404,
          "application/json",
          error_json(
            "saved 2D PNG map is unavailable for map_id: " +
            lookup.manifest.map_id)};
      }

      try {
        return {200, "image/png", read_binary_file(lookup.manifest.localizer_map_png)};
      } catch (const std::exception &) {
        return {
          404,
          "application/json",
          error_json(
            "failed to open exact saved 2D PNG map: " +
            lookup.manifest.localizer_map_png.string())};
      }
    }

    const auto png_path = resolve_mapping_2d_png(
      request, config_.runtime_maps_dir, config_.maps_root);
    if (!png_path) {
      return {
        404,
        "application/json",
        error_json("no saved 2D PNG map is available; save a slam_toolbox 2D map first")};
    }
    try {
      return {200, "image/png", read_binary_file(*png_path)};
    } catch (const std::exception &) {
      return {
        404,
        "application/json",
        error_json("failed to open 2D PNG map: " + png_path->string())};
    }
  }

  nav_msgs::msg::OccupancyGrid latest_mapping_map_for_save(double & age_sec)
  {
    std::lock_guard<std::mutex> map_lock(live_map_mutex_);
    if (!have_live_map_) {
      throw std::runtime_error(
              "no live slam_toolbox /map has been received; start 2D mapping before saving");
    }
    age_sec = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - latest_live_map_received_at_).count();
    return latest_live_map_;
  }

  HttpResponse submit_mapping_save(const std::string & body)
  {
    std::lock_guard<std::mutex> lock(start_mutex_);
    const auto id = json_string_value(body, "request_id").value_or("");
    if (!runtime::MappingSaveJob::valid_id(id)) {
      return {400, "application/json", error_json("valid request_id is required")};
    }
    const auto building = json_string_value(body, "building_id").value_or("");
    const auto floor = json_string_value(body, "floor_id").value_or("");
    const auto name = json_string_value(body, "map_name").value_or("");
    if (!safe_asset_id(building) || !safe_asset_id(floor) || !valid_display_map_name(name)) {
      return {400, "application/json", error_json("valid building_id, floor_id and map_name are required")};
    }
    const auto key = json_string(building) + ":" + json_string(floor) + ":" + json_string(name);
    if (save_job_.find(id).status == 200) {
      return save_job_.submit(id, key, {}); // Same identity returns the original result.
    }
    if (shutdown_requested_.load() || start_job_.running() || save_job_.running()) {
      return {409, "application/json", error_json("mapping start/save is still running")};
    }
    double age = 0.0;
    std::shared_ptr<const nav_msgs::msg::OccupancyGrid> frozen;
    try {
      frozen = std::make_shared<nav_msgs::msg::OccupancyGrid>(latest_mapping_map_for_save(age));
    } catch (const std::exception & error) {
      return {404, "application/json", error_json(error.what())};
    }
    return save_job_.submit(id, key, [this, body, frozen, age](const auto & saved) {
        return handle_save_mapping_2d(body, 0U, frozen, age, saved);
      });
  }

  HttpResponse handle_save_mapping_2d(
    const std::string & body,
    const ElevatorMotionAdmissionFence::Epoch /* legacy_epoch */,
    std::shared_ptr<const nav_msgs::msg::OccupancyGrid> frozen = {},
    const double frozen_age = 0.0,
    const runtime::MappingSaveJob::Saved & saved = {})
  {
    const auto begin = std::chrono::steady_clock::now();
    std::unique_lock<std::mutex> asset_guard(map_asset_mutation_mutex_);
    if (const auto blocked = ports_.floor_runtime_interlock_response("mapping_save")) {
      return *blocked;
    }
    if (ports_.map_asset_integrity_degraded()) {
      return {
        503,
        "application/json",
        error_json("keepout integrity is degraded; repair the keepout layer before saving maps")};
    }
    const auto map_name = json_string_value(body, "map_name");
    const auto building_id = json_string_value(body, "building_id");
    const auto floor_id = json_string_value(body, "floor_id");
    if (!map_name || !valid_display_map_name(*map_name)) {
      return {400, "application/json", error_json("valid map_name is required")};
    }
    if (!building_id || !safe_asset_id(*building_id)) {
      return {400, "application/json", error_json("valid building_id is required")};
    }
    if (!floor_id || !safe_asset_id(*floor_id)) {
      return {400, "application/json", error_json("valid floor_id is required")};
    }

    double map_age_sec = 0.0;
    nav_msgs::msg::OccupancyGrid map;
    const bool mapping_was_active = process_runtime_.tracked_snapshot().active;
    try {
      map = frozen ? *frozen : latest_mapping_map_for_save(map_age_sec);
      if (frozen) {map_age_sec = frozen_age;}
    } catch (const std::exception & exception) {
      return {404, "application/json", error_json(exception.what())};
    }
    const std::uint32_t width = map.info.width;
    const std::uint32_t height = map.info.height;
    if (width == 0U || height == 0U ||
      map.data.size() != static_cast<std::size_t>(width) * height)
    {
      return {
        503, "application/json",
        error_json("live slam_toolbox /map has invalid dimensions")};
    }
    if (const auto blocked = ports_.floor_runtime_interlock_response(
        "mapping_save_commit"))
    {
      return *blocked;
    }

    set_mapping_runtime_state(true, "saving", "saving live 2D mapping assets");
    ports_.clear_runtime_map_context();
    const auto pixels = occupancy_grid_to_image_pixels(map);
    const auto png = encode_grayscale_png(width, height, pixels);
    if (png.empty()) {
      set_mapping_runtime_state(
        true, "running", "map save failed: PNG encode failed", false);
      return {
        500, "application/json",
        error_json("failed to encode live slam_toolbox map as PNG")};
    }

    auto manifest = map_catalog_.make_new_manifest(*building_id, *floor_id, *map_name);
    manifest.active = false;
    const fs::path runtime_base = config_.runtime_maps_dir / manifest.safe_map_name;
    const fs::path runtime_yaml = runtime_base.string() + ".yaml";
    const fs::path runtime_pgm = runtime_base.string() + ".pgm";
    const fs::path runtime_png = runtime_base.string() + ".png";
    const fs::path runtime_localizer_yaml =
      config_.runtime_maps_dir / (manifest.safe_map_name + ".localizer.yaml");
    const fs::path runtime_localizer_png =
      config_.runtime_maps_dir / (manifest.safe_map_name + ".localizer.png");

    try {
      write_pgm_file(runtime_pgm, width, height, pixels);
      write_binary_file(runtime_png, png);
      write_text_file(runtime_yaml, map_yaml_text(runtime_pgm.filename().string(), map));
      write_binary_file(runtime_localizer_png, png);
      write_text_file(
        runtime_localizer_yaml,
        map_yaml_text(runtime_localizer_png.filename().string(), map));
      write_pgm_file(manifest.nav_map_pgm, width, height, pixels);
      write_text_file(
        manifest.nav_map_yaml,
        map_yaml_text(manifest.nav_map_pgm.filename().string(), map));
      write_binary_file(manifest.localizer_map_png, png);
      write_text_file(
        manifest.localizer_params_yaml,
        map_yaml_text(manifest.localizer_map_png.filename().string(), map));
      write_neutral_filter_assets(manifest.root / "filters", map);
      if (!fs::exists(manifest.poses_yaml)) {
        write_text_file(manifest.poses_yaml, "poses: []\n");
      }
      write_asset_report(manifest, map);
      stamp_map_asset_identity(manifest, config_.maps_root);
    } catch (const std::exception & exception) {
      set_mapping_runtime_state(
        true,
        "running",
        std::string("map save failed: ") + exception.what(),
        false);
      return {500, "application/json", error_json(exception.what())};
    }

    std::ostringstream response;
    response << std::fixed << std::setprecision(3);
    response << "{\"ok\":true,"
             << "\"map_saved\":true,"
             << "\"mapping_was_active\":"
             << (mapping_was_active ? "true" : "false") << ","
             << "\"map_age_sec\":" << map_age_sec << ","
             << "\"map_id\":" << json_string(manifest.map_id) << ","
             << "\"display_name\":" << json_string(manifest.display_name) << ","
             << "\"map_name\":" << json_string(manifest.display_name) << ","
             << "\"safe_map_name\":" << json_string(manifest.safe_map_name) << ","
             << "\"building_id\":" << json_string(*building_id) << ","
             << "\"floor_id\":" << json_string(*floor_id) << ","
             << "\"asset_epoch\":" << manifest.asset_epoch << ","
             << "\"asset_digest\":" << json_string(manifest.asset_digest) << ","
             << "\"active\":false,"
             << "\"selected_for_navigation\":false,"
             << "\"requires_manual_navigation_selection\":true,"
             << "\"runtime_map\":{"
             << "\"yaml\":" << json_string(runtime_yaml.string()) << ","
             << "\"pgm\":" << json_string(runtime_pgm.string()) << ","
             << "\"png\":" << json_string(runtime_png.string()) << ","
             << "\"localizer_yaml\":" << json_string(runtime_localizer_yaml.string()) << ","
             << "\"localizer_png\":" << json_string(runtime_localizer_png.string()) << "},"
             << "\"floor_assets\":{"
             << "\"root\":" << json_string(manifest.root.string()) << ","
             << "\"current_root\":"
             << json_string(
      map_catalog_.floor_current_root_path(*building_id, *floor_id).string()) << ","
             << "\"manifest_json\":" << json_string(manifest.manifest_json.string()) << ","
             << "\"nav_map_yaml\":" << json_string(manifest.nav_map_yaml.string()) << ","
             << "\"nav_map_pgm\":" << json_string(manifest.nav_map_pgm.string()) << ","
             << "\"localizer_map_png\":"
             << json_string(manifest.localizer_map_png.string()) << ","
             << "\"localizer_params_yaml\":"
             << json_string(manifest.localizer_params_yaml.string()) << ","
             << "\"asset_report_json\":"
             << json_string(manifest.asset_report_json.string()) << "}}";
    asset_guard.unlock();
    const auto assets_done = std::chrono::steady_clock::now();
    const std::string asset_result = response.str();
    if (saved) {saved(asset_result);}
    RCLCPP_INFO(node_.get_logger(), "mapping_save map_id=%s phase=assets_committed elapsed_ms=%lld",
      manifest.map_id.c_str(), static_cast<long long>(
        std::chrono::duration_cast<std::chrono::milliseconds>(assets_done - begin).count()));
    std::size_t stopped_groups = 0U;
    {
      std::lock_guard<std::mutex> launch_lock(transition_launch_mutex_);
      stopped_groups = process_runtime_.stop();
    }
    const bool mapping_stopped = !process_runtime_.recover_snapshot().running;
    if (mapping_stopped) {
      set_mapping_live_map_cache_active(false);
      clear_live_map_cache();
    }
    set_mapping_runtime_state(!mapping_stopped, mapping_stopped ? "stopped" : "failed",
      mapping_stopped ? "2D map saved and mapping chain stopped" :
      "2D map saved, but mapping shutdown is not confirmed", mapping_stopped);
    RCLCPP_INFO(node_.get_logger(), "mapping_save map_id=%s phase=shutdown_finished elapsed_ms=%lld stopped=%d",
      manifest.map_id.c_str(), static_cast<long long>(
        std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::steady_clock::now() - assets_done).count()), mapping_stopped);
    auto result = asset_result;
    result.pop_back();
    result += ",\"mapping_active\":" + std::string(mapping_stopped ? "false" : "true") +
      ",\"mapping_stopped\":" + (mapping_stopped ? "true" : "false") +
      ",\"stopped\":" + (mapping_stopped ? "true" : "false") +
      ",\"stopped_groups\":" + std::to_string(stopped_groups) + "}";
    return {200, "application/json", result};
  }

  bool mapping_scan_has_exact_navigation_owner() const
  {
    const auto publishers = node_.get_publishers_info_by_topic(config_.scan_owner_topic);
    if (publishers.size() != 1U) {
      return false;
    }
    const auto & publisher = publishers.front();
    return publisher.node_name() == config_.navigation_scan_owner_node &&
           (publisher.node_namespace().empty() || publisher.node_namespace() == "/");
  }

  bool wait_for_mapping_scan_owner_restored(const double timeout_sec) const
  {
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(timeout_sec);
    while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
      if (mapping_scan_has_exact_navigation_owner()) {
        return true;
      }
      std::this_thread::sleep_for(100ms);
    }
    return mapping_scan_has_exact_navigation_owner();
  }

  bool ensure_mapping_scan_owner_restored(std::string & detail)
  {
    if (wait_for_mapping_scan_owner_restored(1.0)) {
      detail = "canonical navigation /scan ownership already restored";
      return true;
    }
    const auto service_timeout = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(config_.scan_owner_restore_timeout_sec));
    if (!scan_control_client_->wait_for_service(service_timeout)) {
      detail = "mapping stop cannot restore /scan: service unavailable: " +
        config_.resident_scan_control_service;
      return false;
    }
    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = true;
    auto future = scan_control_client_->async_send_request(request);
    if (future.wait_for(service_timeout) != std::future_status::ready) {
      if (wait_for_mapping_scan_owner_restored(config_.scan_owner_restore_timeout_sec)) {
        detail = "canonical navigation /scan ownership restored after delayed service response";
        return true;
      }
      detail = "mapping stop timed out restoring canonical navigation /scan ownership";
      return false;
    }
    const auto response = future.get();
    if (!response->success) {
      detail = "mapping stop resident /scan restore rejected: " + response->message;
      return false;
    }
    if (!wait_for_mapping_scan_owner_restored(config_.scan_owner_restore_timeout_sec)) {
      detail = "mapping stop service succeeded but exact navigation /scan owner was not observed";
      return false;
    }
    detail = "canonical navigation /scan ownership restored";
    return true;
  }

  void finish_mapping_start_job(
    const std::uint64_t job_id,
    const std::string & state,
    const std::string & detail,
    const pid_t mapping_pid = -1)
  {
    if (!start_job_.finish(
        job_id, state, detail, mapping_pid, utc_timestamp_iso8601()))
    {
      return;
    }
    runtime_mode_.finish_transition("mapping_start");
  }

  void join_mapping_start_worker()
  {
    if (start_worker_.joinable()) {
      start_worker_.join();
    }
  }

  void run_mapping_start_job(
    const std::uint64_t job_id,
    const bool navigation_was_active)
  {
    bool navigation_cancel_ok = true;
    bool navigation_stop_ok = true;
    std::string floor_interlock_detail;
    if (ports_.floor_runtime_operation_blocked(
        "mapping_start_worker", floor_interlock_detail))
    {
      set_mapping_runtime_state(false, "blocked", floor_interlock_detail, false);
      finish_mapping_start_job(job_id, "failed", floor_interlock_detail);
      return;
    }

    if (navigation_was_active) {
      (void)start_job_.set_phase(
        job_id,
        "cancel_navigation",
        "canceling active navigation task before 2D mapping");
      const auto cancel_result = ports_.cancel_navigation();
      navigation_cancel_ok = cancel_result.ok;
      (void)start_job_.set_phase(job_id, "cancel_navigation", cancel_result.detail);
      (void)start_job_.set_navigation_result(job_id, navigation_cancel_ok, false);

      if (start_job_.cancel_requested(job_id)) {
        runtime_mode_.set_navigation(
          true, "ready", "2D mapping start canceled; navigation remains active");
        finish_mapping_start_job(
          job_id, "canceled", "2D mapping start canceled before navigation stop");
        return;
      }

      (void)start_job_.set_phase(
        job_id,
        "stop_navigation_runtime",
        "stopping navigation mode services before 2D mapping");
      const auto stop_result = ports_.stop_navigation_runtime();
      navigation_stop_ok = stop_result.ok;
      (void)start_job_.set_navigation_result(
        job_id, navigation_cancel_ok, navigation_stop_ok);
      (void)start_job_.set_phase(
        job_id, "stop_navigation_runtime", stop_result.detail);
      if (!navigation_stop_ok) {
        runtime_mode_.set_navigation(true, "error", stop_result.detail, false);
        finish_mapping_start_job(job_id, "failed", stop_result.detail);
        return;
      }
      runtime_mode_.set_navigation(
        false, "stopped", "navigation runtime stopped for 2D mapping");
    }
    (void)start_job_.set_navigation_result(
      job_id, navigation_cancel_ok, navigation_stop_ok);

    if (start_job_.cancel_requested(job_id)) {
      set_mapping_runtime_state(false, "stopped", "2D mapping start canceled");
      finish_mapping_start_job(
        job_id, "canceled", "2D mapping start canceled before process launch");
      return;
    }
    if (ports_.floor_runtime_operation_blocked(
        "mapping_start_context_clear", floor_interlock_detail))
    {
      set_mapping_runtime_state(false, "blocked", floor_interlock_detail, false);
      finish_mapping_start_job(job_id, "failed", floor_interlock_detail);
      return;
    }
    (void)start_job_.set_phase(
      job_id, "clear_navigation_context", "clearing navigation map context");
    ports_.clear_runtime_map_context();

    pid_t pid = -1;
    std::string start_detail;
    {
      std::lock_guard<std::mutex> launch_lock(transition_launch_mutex_);
      if (start_job_.cancel_requested(job_id)) {
        set_mapping_runtime_state(false, "stopped", "2D mapping start canceled");
        finish_mapping_start_job(
          job_id, "canceled", "2D mapping start canceled before process launch");
        return;
      }
      if (ports_.floor_runtime_operation_blocked(
          "mapping_process_launch", floor_interlock_detail))
      {
        set_mapping_runtime_state(false, "blocked", floor_interlock_detail, false);
        finish_mapping_start_job(job_id, "failed", floor_interlock_detail);
        return;
      }
      (void)start_job_.set_phase(
        job_id, "start_mapping_process", "starting 2D mapping process");
      const auto start_result = process_runtime_.start([this]() {
          clear_live_map_cache();
        });
      pid = start_result.pid;
      start_detail = start_result.detail;
      if (!start_result.ok) {
        set_mapping_live_map_cache_active(false);
        set_mapping_runtime_state(false, "failed", start_detail, false);
        finish_mapping_start_job(job_id, "failed", start_detail);
        return;
      }
    }

    set_mapping_live_map_cache_active(true);
    set_mapping_runtime_state(true, "starting", "2D mapping chain start accepted");
    finish_mapping_start_job(job_id, "succeeded", start_detail, pid);
  }

  void run_mapping_start_job_guarded(
    const std::uint64_t job_id,
    const bool navigation_was_active)
  {
    try {
      run_mapping_start_job(job_id, navigation_was_active);
    } catch (const std::exception & exception) {
      const std::string detail =
        std::string("2D mapping start worker exception: ") + exception.what();
      set_mapping_runtime_state(false, "failed", detail, false);
      finish_mapping_start_job(job_id, "failed", detail);
    } catch (...) {
      const std::string detail = "2D mapping start worker unknown exception";
      set_mapping_runtime_state(false, "failed", detail, false);
      finish_mapping_start_job(job_id, "failed", detail);
    }
  }

  HttpResponse handle_start_mapping_2d(
    const ElevatorMotionAdmissionFence::Epoch /* legacy_epoch */)
  {
    std::lock_guard<std::mutex> start_lock(start_mutex_);
    if (save_job_.running()) {
      return {409, "application/json", error_json("mapping save is still running")};
    }
    std::lock_guard<std::mutex> asset_guard(map_asset_mutation_mutex_);
    if (const auto blocked = ports_.floor_runtime_interlock_response("mapping_start")) {
      return *blocked;
    }
    if (ports_.map_asset_integrity_degraded()) {
      return {
        503,
        "application/json",
        error_json(
          "keepout integrity is degraded; navigation and mapping admission are blocked")};
    }
    if (!process_runtime_.start_command_available()) {
      return {
        503,
        "application/json",
        error_json(
          "2D slam_toolbox start command is not available: " +
          process_runtime_.start_command())};
    }

    auto runtime = runtime_mode_.snapshot();
    if (runtime.docking_active) {
      return {
        409, "application/json",
        error_json("cannot start 2D mapping while docking is active")};
    }
    if (process_runtime_.tracked_snapshot().running) {
      set_mapping_live_map_cache_active(true);
      set_mapping_runtime_state(true, "running", "2D mapping chain is already running");
      return {
        202,
        "application/json",
        "{\"ok\":true,\"state\":\"already_running\",\"map_topic\":" +
        json_string(config_.live_map_topic) +
        ",\"map_endpoint\":\"/api/v1/mapping/2d/map\"}"};
    }

    if (start_job_.running()) {
      return {
        202,
        "application/json",
        "{\"ok\":true,\"accepted\":true,\"already_running\":true,\"mapping_start\":" +
        start_job_.json() + "}"};
    }
    join_mapping_start_worker();

    std::string conflict_owner;
    if (!runtime_mode_.try_begin_transition("mapping_start", conflict_owner)) {
      return {
        409,
        "application/json",
        error_json("runtime mode transition is already active: " + conflict_owner)};
    }

    runtime = runtime_mode_.snapshot();
    if (runtime.docking_active) {
      runtime_mode_.finish_transition("mapping_start");
      return {
        409, "application/json",
        error_json("cannot start 2D mapping while docking is active")};
    }
    const bool navigation_was_active =
      runtime.navigation_active || ports_.navigation_goal_running();
    const auto accepted_job = start_job_.begin(
      navigation_was_active, utc_timestamp_iso8601());
    if (!accepted_job) {
      runtime_mode_.finish_transition("mapping_start");
      return {
        409, "application/json",
        error_json("2D mapping start transition is already active")};
    }
    const std::uint64_t job_id = *accepted_job;

    if (navigation_was_active) {
      runtime_mode_.set_navigation(
        true,
        "stopping_for_mapping",
        "2D mapping transition accepted; stopping navigation runtime");
    }
    try {
      start_worker_ = std::thread([this, job_id, navigation_was_active]() {
          run_mapping_start_job_guarded(job_id, navigation_was_active);
        });
    } catch (const std::exception & exception) {
      const std::string detail =
        std::string("failed to start 2D mapping transition worker: ") +
        exception.what();
      if (navigation_was_active) {
        runtime_mode_.set_navigation(
          true, "ready", "2D mapping transition worker did not start");
      }
      finish_mapping_start_job(job_id, "failed", detail);
      return {500, "application/json", error_json(detail)};
    }

    std::ostringstream response;
    response << "{\"ok\":true,\"accepted\":true,\"state\":\"starting\","
             << "\"map_topic\":" << json_string(config_.live_map_topic) << ","
             << "\"map_endpoint\":\"/api/v1/mapping/2d/map\","
             << "\"mapping_start\":" << start_job_.json() << "}";
    return {202, "application/json", response.str()};
  }

  HttpResponse handle_stop_mapping_2d()
  {
    std::lock_guard<std::mutex> start_lock(start_mutex_);
    if (save_job_.running()) {
      return {202, "application/json",
        "{\"ok\":true,\"accepted\":true,\"state\":\"save_in_progress\"}"};
    }
    std::size_t requested_groups = 0U;
    bool start_transition_cancel_requested = false;
    {
      std::lock_guard<std::mutex> launch_lock(transition_launch_mutex_);
      start_transition_cancel_requested = start_job_.request_cancel(
        "2D mapping stop requested during startup transition");
      requested_groups = process_runtime_.stop();
    }
    std::string scan_restore_detail;
    if (!ensure_mapping_scan_owner_restored(scan_restore_detail)) {
      set_mapping_live_map_cache_active(false);
      clear_live_map_cache();
      const std::string detail =
        "mapping stop failed to restore canonical navigation /scan ownership: " +
        scan_restore_detail;
      set_mapping_runtime_state(false, "failed", detail, false);
      return {503, "application/json", error_json(detail)};
    }
    set_mapping_live_map_cache_active(false);
    clear_live_map_cache();
    set_mapping_runtime_state(false, "stopped", "2D mapping chain stopped");

    std::ostringstream body;
    body << "{\"ok\":true,\"mapping_active\":false,"
         << "\"start_transition_cancel_requested\":"
         << (start_transition_cancel_requested ? "true" : "false")
         << ",\"stopped\":" << (requested_groups > 0U ? "true" : "false")
         << ",\"stopped_groups\":" << requested_groups << "}";
    return {
      start_transition_cancel_requested ? 202 : 200,
      "application/json",
      body.str()};
  }

  void refresh_mapping_2d_runtime_state()
  {
    auto runtime = runtime_mode_.snapshot();
    if (!runtime.mapping_active) {
      if (!process_runtime_.recover_snapshot().running) {
        return;
      }
      set_mapping_runtime_state(true, "starting", "2D mapping process discovered");
      runtime = runtime_mode_.snapshot();
    }
    if (runtime.mapping_state == "saving" || runtime.mapping_state == "stopping") {
      return;
    }

    const auto process = process_runtime_.recover_snapshot();
    if (!process.running) {
      set_mapping_live_map_cache_active(false);
      return;
    }
    set_mapping_live_map_cache_active(true);

    bool live_map_ready = false;
    {
      std::lock_guard<std::mutex> map_lock(live_map_mutex_);
      if (have_live_map_ && latest_live_map_received_at_ >= process.started_at) {
        const auto age = std::chrono::duration<double>(
          std::chrono::steady_clock::now() - latest_live_map_received_at_).count();
        const auto expected_size =
          static_cast<std::size_t>(latest_live_map_.info.width) *
          latest_live_map_.info.height;
        live_map_ready =
          age <= config_.live_map_max_age_sec &&
          latest_live_map_.info.width > 0U &&
          latest_live_map_.info.height > 0U &&
          latest_live_map_.data.size() == expected_size;
      }
    }
    if (live_map_ready && runtime.mapping_state != "running") {
      set_mapping_runtime_state(true, "running", "live 2D mapping map ready");
    }
  }

  HttpResponse handle_live_mapping_2d_map_png()
  {
    {
      std::lock_guard<std::mutex> lock(live_subscription_mutex_);
      if (!live_map_page_subscription_active_) {
        return {
          409,
          "application/json",
          error_json(
            "live_map resource is not acquired; call POST /api/v1/subscriptions/acquire first")};
      }
    }
    const auto process = process_runtime_.recover_snapshot();
    if (!process.active) {
      return {
        409,
        "application/json",
        error_json(
          "2D slam_toolbox mapping is not active; call POST /api/v1/mapping/2d/start first")};
    }

    nav_msgs::msg::OccupancyGrid map;
    std::chrono::steady_clock::time_point received_at;
    {
      std::lock_guard<std::mutex> map_lock(live_map_mutex_);
      if (!have_live_map_ || latest_live_map_received_at_ < process.started_at) {
        return {
          404,
          "application/json",
          error_json("waiting for live slam_toolbox /map data")};
      }
      map = latest_live_map_;
      received_at = latest_live_map_received_at_;
    }
    const auto age = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - received_at).count();
    if (age > config_.live_map_max_age_sec) {
      return {
        503, "application/json", error_json("live slam_toolbox /map is stale")};
    }
    const std::uint32_t width = map.info.width;
    const std::uint32_t height = map.info.height;
    if (width == 0U || height == 0U ||
      map.data.size() != static_cast<std::size_t>(width) * height)
    {
      return {
        503, "application/json",
        error_json("live slam_toolbox /map has invalid dimensions")};
    }
    const auto png = encode_grayscale_png(
      width, height, occupancy_grid_to_image_pixels(map));
    if (png.empty()) {
      return {
        500, "application/json",
        error_json("failed to encode live slam_toolbox map as PNG")};
    }
    set_mapping_runtime_state(true, "running", "live 2D mapping map ready");
    return {200, "image/png", png};
  }

  HttpResponse handle_mapping_2d_map_png(const HttpRequest & request)
  {
    const auto source_it = request.query.find("source");
    const bool explicit_saved =
      request.query.find("name") != request.query.end() ||
      (source_it != request.query.end() && source_it->second == "saved");
    if (explicit_saved) {
      return handle_saved_mapping_2d_map_png(request);
    }
    return handle_live_mapping_2d_map_png();
  }

  rclcpp::Node & node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  RuntimeModeCoordinator & runtime_mode_;
  MapCatalog & map_catalog_;
  std::mutex & map_asset_mutation_mutex_;
  MappingModuleConfig config_;
  MappingModulePorts ports_;
  runtime::MappingSaveJob save_job_;
  MappingProcessRuntime process_runtime_;
  MappingStartJobTracker start_job_;
  std::mutex start_mutex_;
  std::mutex transition_launch_mutex_;
  std::thread start_worker_;
  std::atomic<bool> shutdown_requested_{false};

  std::mutex live_subscription_mutex_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr live_map_sub_;
  bool live_map_page_subscription_active_{false};
  bool live_map_mapping_cache_active_{false};
  std::mutex live_map_mutex_;
  nav_msgs::msg::OccupancyGrid latest_live_map_;
  bool have_live_map_{false};
  std::chrono::steady_clock::time_point latest_live_map_received_at_{};
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr scan_control_client_;
};

MappingModule::MappingModule(
  rclcpp::Node & node,
  rclcpp::CallbackGroup::SharedPtr callback_group,
  RuntimeModeCoordinator & runtime_mode,
  MapCatalog & map_catalog,
  std::mutex & map_asset_mutation_mutex,
  MappingModuleConfig config,
  MappingModulePorts ports)
: impl_(std::make_unique<Impl>(
      node,
      std::move(callback_group),
      runtime_mode,
      map_catalog,
      map_asset_mutation_mutex,
      std::move(config),
      std::move(ports)))
{
}

MappingModule::~MappingModule() = default;

std::optional<HttpResponse> MappingModule::handle_http(
  const HttpRequest & request,
  const ElevatorMotionAdmissionFence::Epoch motion_admission_epoch)
{
  return impl_->handle_http(request, motion_admission_epoch);
}

std::string MappingModule::status_json()
{
  return impl_->status_json();
}

MappingModuleSnapshot MappingModule::snapshot(const bool refresh_runtime)
{
  return impl_->snapshot(refresh_runtime);
}

void MappingModule::set_live_map_page_active(const bool active)
{
  impl_->set_live_map_page_active(active);
}

void MappingModule::shutdown()
{
  impl_->shutdown();
}

}  // namespace robot_api_server::features::mapping
