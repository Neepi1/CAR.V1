#include "robot_api_server/features/maps/maps_module.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <future>
#include <iomanip>
#include <limits>
#include <map>
#include <set>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <utility>
#include <vector>

#include <unistd.h>

#include "lifecycle_msgs/msg/state.hpp"
#include "nav2_msgs/msg/costmap_filter_info.hpp"
#include "nav2_msgs/srv/clear_entire_costmap.hpp"
#include "nav2_msgs/srv/load_map.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rcl_interfaces/msg/parameter_type.hpp"
#include "rcl_interfaces/srv/get_parameters.hpp"

#include "robot_api_server/features/floor_switch/runtime_map_context_io.hpp"
#include "robot_api_server/features/elevator/configuration/elevator_configuration_module.hpp"
#include "robot_api_server/features/localization/tf_pose_utils.hpp"
#include "robot_api_server/features/maps/catalog_activation/api_time_utils.hpp"
#include "robot_api_server/features/maps/catalog_activation/file_utils.hpp"
#include "robot_api_server/features/maps/catalog_activation/floor_asset_resolver.hpp"
#include "robot_api_server/features/maps/catalog_activation/map_asset_digest.hpp"
#include "robot_api_server/features/maps/catalog_activation/map_asset_filesystem.hpp"
#include "robot_api_server/features/maps/catalog_activation/map_asset_io.hpp"
#include "robot_api_server/features/maps/catalog_activation/map_manifest_io.hpp"
#include "robot_api_server/features/maps/catalog_activation/storage_models.hpp"
#include "robot_api_server/features/maps/keepout/keepout_layer.hpp"
#include "robot_api_server/features/maps/keepout/semantic_layer_io.hpp"
#include "robot_api_server/features/maps/poses/poses_io.hpp"

namespace robot_api_server::features::maps
{

namespace fs = std::filesystem;
using namespace std::chrono_literals;

namespace
{

bool starts_with(const std::string & value, const std::string & prefix)
{
  return value.rfind(prefix, 0U) == 0U;
}

void require_ports(const MapsModulePorts & ports)
{
  if (!ports.runtime_snapshot ||
    !ports.floor_runtime_interlock_response ||
    !ports.wait_for_current_robot_pose ||
    !ports.wait_for_terminal_actual_stop ||
    !ports.query_elevator_reference ||
    !ports.acquire_motion_admission ||
    !ports.mark_delayed_side_effect_unknown ||
    !ports.resolve_delayed_side_effect_unknown)
  {
    throw std::invalid_argument("maps module requires every cross-domain port");
  }
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
       << ",\"transaction_id\":" << json_string(admission.interlock().transaction_id)
       << ",\"delayed_side_effect_unknown_count\":"
       << admission.interlock().delayed_side_effect_unknown_count
       << ",\"detail\":" << json_string(
    motion_admission_failure_detail(operation, admission)) << "}";
  return {409, "application/json", body.str()};
}

class DelayedSideEffectEvidence
{
public:
  explicit DelayedSideEffectEvidence(const MapsModulePorts & ports)
  : ports_(ports)
  {
    ports_.mark_delayed_side_effect_unknown();
  }

  DelayedSideEffectEvidence(const DelayedSideEffectEvidence &) = delete;
  DelayedSideEffectEvidence & operator=(const DelayedSideEffectEvidence &) = delete;

  void resolve() noexcept
  {
    if (!resolved_) {
      ports_.resolve_delayed_side_effect_unknown();
      resolved_ = true;
    }
  }

private:
  const MapsModulePorts & ports_;
  bool resolved_{false};
};

}  // namespace

class MapsModule::Impl
{
public:
  Impl(
    rclcpp::Node & node,
    rclcpp::CallbackGroup::SharedPtr callback_group,
    std::mutex & cross_asset_commit_mutex,
    MapsModuleConfig config,
    MapsModulePorts ports)
  : node_(node),
    callback_group_(std::move(callback_group)),
    cross_asset_commit_mutex_(cross_asset_commit_mutex),
    config_(std::move(config)),
    ports_(std::move(ports)),
    map_catalog_(
      config_.maps_root,
      [this](const std::string & building_id, const std::string & floor_id) {
        ensure_legacy_floor_map_manifest(building_id, floor_id);
      })
  {
    require_ports(ports_);
    if (!callback_group_) {
      throw std::invalid_argument("maps module requires a ROS callback group");
    }
    if (config_.maps_root.empty()) {
      throw std::invalid_argument("maps module requires maps_root");
    }

    bool activation_recovery_ok = true;
    try {
      recover_pending_map_activation();
    } catch (const std::exception & error) {
      activation_recovery_ok = false;
      persist_map_asset_integrity_degraded(
        "", std::string("activation recovery failed: ") + error.what());
      RCLCPP_ERROR(
        node_.get_logger(), "map activation recovery failed closed: %s", error.what());
    }
    const bool startup_integrity_marker_present =
      persisted_map_asset_integrity_marker().has_value();
    const bool migration_allowed =
      activation_recovery_ok && !startup_integrity_marker_present;
    const auto startup_manifests =
      map_catalog_.read_all_map_manifests(migration_allowed);
    for (auto manifest : startup_manifests) {
      if (!migration_allowed) {
        break;
      }
      if (manifest.asset_epoch != 0U && !manifest.asset_digest.empty()) {
        continue;
      }
      std::string migration_error;
      if (!validate_map_manifest_assets(manifest, migration_error)) {
        RCLCPP_WARN(
          node_.get_logger(),
          "legacy map identity migration skipped for %s: %s",
          manifest.map_id.c_str(), migration_error.c_str());
        continue;
      }
      try {
        stamp_map_asset_identity(manifest, config_.maps_root);
      } catch (const std::exception & error) {
        RCLCPP_ERROR(
          node_.get_logger(),
          "legacy map identity migration failed for %s: %s",
          manifest.map_id.c_str(), error.what());
      }
    }
    restore_map_asset_integrity_state();

    keepout_mask_sub_ = node_.create_subscription<nav_msgs::msg::OccupancyGrid>(
      config_.keepout_mask_topic,
      rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(),
      [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        handle_keepout_mask(msg);
      });
    keepout_filter_info_sub_ =
      node_.create_subscription<nav2_msgs::msg::CostmapFilterInfo>(
      config_.keepout_filter_info_topic,
      rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(),
      [this](const nav2_msgs::msg::CostmapFilterInfo::SharedPtr msg) {
        handle_keepout_filter_info(msg);
      });

    keepout_mask_load_client_ = node_.create_client<nav2_msgs::srv::LoadMap>(
      config_.keepout_mask_load_service,
      rmw_qos_profile_services_default,
      callback_group_);
    keepout_mask_state_client_ = node_.create_client<lifecycle_msgs::srv::GetState>(
      config_.keepout_mask_state_service,
      rmw_qos_profile_services_default,
      callback_group_);
    keepout_filter_info_state_client_ =
      node_.create_client<lifecycle_msgs::srv::GetState>(
      config_.keepout_filter_info_state_service,
      rmw_qos_profile_services_default,
      callback_group_);
    keepout_mask_get_parameters_client_ =
      node_.create_client<rcl_interfaces::srv::GetParameters>(
      config_.keepout_mask_get_parameters_service,
      rmw_qos_profile_services_default,
      callback_group_);
    global_costmap_get_parameters_client_ =
      node_.create_client<rcl_interfaces::srv::GetParameters>(
      config_.global_costmap_get_parameters_service,
      rmw_qos_profile_services_default,
      callback_group_);
    global_costmap_lifecycle_client_ =
      node_.create_client<lifecycle_msgs::srv::GetState>(
      config_.global_costmap_state_service,
      rmw_qos_profile_services_default,
      callback_group_);
    global_costmap_clear_client_ =
      node_.create_client<nav2_msgs::srv::ClearEntireCostmap>(
      config_.global_costmap_clear_service,
      rmw_qos_profile_services_default,
      callback_group_);
  }

  std::optional<HttpResponse> handle_http(
    const HttpRequest & request,
    const ElevatorMotionAdmissionFence::Epoch motion_admission_epoch)
  {
    if (request.method == "GET" && request.path == "/api/v1/maps") {
      return handle_maps();
    }
    if (request.method == "GET" && request.path == "/api/v1/maps/semantic_layer") {
      return handle_get_semantic_layer(request);
    }
    if (request.method == "GET" && request.path == "/api/v1/maps/poses") {
      return handle_get_poses(request);
    }
    if (request.method == "GET" && request.path == "/api/v1/maps/filters/keepout") {
      return handle_get_keepout_filter(request);
    }
    if (request.method == "POST" && request.path == "/api/v1/maps/delete") {
      return handle_delete_map(request.body);
    }
    if (request.method == "POST" &&
      (request.path == "/api/v1/maps/poses" ||
      request.path == "/api/v1/maps/poses/save"))
    {
      return handle_save_pose(request.body);
    }
    if (request.method == "POST" &&
      request.path == "/api/v1/maps/poses/save_current")
    {
      return handle_save_current_pose(request.body);
    }
    if (request.method == "PUT" && request.path == "/api/v1/maps/poses/batch") {
      return handle_replace_poses_batch(request.body);
    }
    const std::string pose_item_prefix = "/api/v1/maps/poses/";
    if ((request.method == "PUT" || request.method == "DELETE") &&
      starts_with(request.path, pose_item_prefix))
    {
      const auto pose_id = request.path.substr(pose_item_prefix.size());
      if (!safe_pose_id(pose_id)) {
        return HttpResponse{
          400, "application/json",
          error_json("valid pose_id path segment is required")};
      }
      if (request.method == "PUT") {
        return handle_save_pose(
          request.body, std::optional<std::string>(pose_id));
      }
      return handle_delete_pose(request, pose_id);
    }
    if (request.method == "POST" &&
      request.path == "/api/v1/maps/filters/keepout/save")
    {
      return handle_save_keepout_filter(
        request.body, motion_admission_epoch);
    }
    return std::nullopt;
  }

  MapCatalog & catalog() noexcept
  {
    return map_catalog_;
  }

  std::mutex & mutation_mutex() noexcept
  {
    return asset_mutation_mutex_;
  }

  rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr
  global_costmap_lifecycle_client() const
  {
    return global_costmap_lifecycle_client_;
  }

  std::optional<RuntimeMapContext> read_runtime_map_context() const
  {
    if (config_.runtime_map_context_file.empty()) {
      return std::nullopt;
    }
    return read_runtime_map_context_file(config_.runtime_map_context_file);
  }

private:
  friend class MapsModule;

  struct KeepoutMaskObservation
  {
    std::uint64_t sequence{0U};
    std::uint32_t width{0U};
    std::uint32_t height{0U};
    double resolution{0.0};
    double origin_x{0.0};
    double origin_y{0.0};
    double origin_yaw{0.0};
    std::size_t active_cells{0U};
    std::string occupancy_digest;
    std::chrono::steady_clock::time_point received_at{};
  };

  struct KeepoutFilterInfoObservation
  {
    bool valid{false};
    std::uint8_t type{255U};
    std::string mask_topic;
    double base{0.0};
    double multiplier{0.0};
    std::chrono::steady_clock::time_point received_at{};
  };

  struct KeepoutCostmapProbeState
  {
    std::mutex mutex;
    std::condition_variable condition;
    std::uint64_t sequence{0U};
    nav_msgs::msg::OccupancyGrid::SharedPtr latest;
    std::chrono::steady_clock::time_point received_at{};
  };










  void handle_keepout_mask(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    KeepoutMaskObservation observation;
    observation.width = msg->info.width;
    observation.height = msg->info.height;
    observation.resolution = msg->info.resolution;
    observation.origin_x = msg->info.origin.position.x;
    observation.origin_y = msg->info.origin.position.y;
    const auto & orientation = msg->info.origin.orientation;
    observation.origin_yaw = quaternion_yaw(
      orientation.x, orientation.y, orientation.z, orientation.w);
    observation.received_at = std::chrono::steady_clock::now();
    std::string normalized;
    normalized.resize(msg->data.size());
    for (std::size_t index = 0U; index < msg->data.size(); ++index) {
      const auto value = msg->data[index];
      if (value < 0) {
        normalized[index] = '\2';
      } else if (value > 0) {
        normalized[index] = '\1';
        ++observation.active_cells;
      } else {
        normalized[index] = '\0';
      }
    }
    observation.occupancy_digest =
      "occupancy-fnv64-" + fixed_hex(fnv1a64(normalized), 16);

    {
      std::lock_guard<std::mutex> lock(keepout_observation_mutex_);
      observation.sequence = latest_keepout_mask_.sequence + 1U;
      latest_keepout_mask_ = std::move(observation);
    }
    keepout_observation_cv_.notify_all();
  }

  void handle_keepout_filter_info(const nav2_msgs::msg::CostmapFilterInfo::SharedPtr msg)
  {
    KeepoutFilterInfoObservation observation;
    observation.type = msg->type;
    observation.mask_topic = msg->filter_mask_topic;
    observation.base = msg->base;
    observation.multiplier = msg->multiplier;
    observation.received_at = std::chrono::steady_clock::now();
    observation.valid =
      msg->type == 0U &&
      msg->filter_mask_topic == config_.keepout_mask_topic &&
      std::isfinite(msg->base) &&
      std::isfinite(msg->multiplier);
    std::lock_guard<std::mutex> lock(keepout_observation_mutex_);
    latest_keepout_filter_info_ = std::move(observation);
  }
  std::optional<MapManifest> confirmed_runtime_map_manifest(
    std::string & unavailable_reason,
    bool & blocked_by_pending_context)
  {
    blocked_by_pending_context = false;
    const auto runtime = ports_.runtime_snapshot(false);
    const auto context = read_runtime_map_context();
    if (context) {
      if (!context->confirmed) {
        if (runtime.navigation_active || runtime.docking_active) {
          blocked_by_pending_context = true;
          unavailable_reason = "runtime map context is not confirmed yet: " +
            context->building_id + "/" + context->floor_id + "/" + context->map_id +
            " state=" + context->state;
        }
        return std::nullopt;
      }
      const auto manifest = map_catalog_.find_map_by_id(context->map_id);
      if (!manifest || manifest->building_id != context->building_id ||
        manifest->floor_id != context->floor_id)
      {
        if (runtime.navigation_active || runtime.docking_active) {
          blocked_by_pending_context = true;
          unavailable_reason = "confirmed runtime map context does not match a valid manifest: " +
            context->building_id + "/" + context->floor_id + "/" + context->map_id;
        }
        return std::nullopt;
      }
      return manifest;
    }

    if (runtime.navigation_active || runtime.docking_active) {
      blocked_by_pending_context = true;
      unavailable_reason = "navigation or docking is active but no runtime map context is recorded";
      return std::nullopt;
    }

    if (runtime.mapping_active) {
      return std::nullopt;
    }

    return map_catalog_.unique_active_map_manifest();
  }

  bool validate_map_manifest_assets(const MapManifest & manifest, std::string & error) const
  {
    const std::vector<fs::path> required = {
      manifest.nav_map_yaml,
      manifest.nav_map_pgm,
      manifest.localizer_map_png,
      manifest.localizer_params_yaml,
      manifest.keepout_mask_yaml,
      manifest.keepout_mask_pgm,
      manifest.speed_mask_yaml,
      manifest.speed_mask_pgm,
      manifest.binary_mask_yaml,
      manifest.binary_mask_pgm,
      manifest.asset_report_json,
      manifest.poses_yaml
    };
    for (const auto & path : required) {
      if (!fs::exists(path)) {
        error = "map asset is incomplete, missing: " + path.string();
        return false;
      }
    }
    return true;
  }

  bool validate_current_map_projection_assets(
    const MapManifest & source_manifest,
    const fs::path & current_root,
    std::string & error) const
  {
    const std::vector<fs::path> required = {
      current_root / "manifest.json",
      current_root / "nav" / "nav_map.yaml",
      current_root / "nav" / "nav_map.pgm",
      current_root / "localizer" / "localizer_map.png",
      current_root / "localizer" / "localizer_params.yaml",
      current_root / "filters" / "keepout_mask.yaml",
      current_root / "filters" / "keepout_mask.pgm",
      current_root / "filters" / "speed_mask.yaml",
      current_root / "filters" / "speed_mask.pgm",
      current_root / "filters" / "binary_mask.yaml",
      current_root / "filters" / "binary_mask.pgm",
      current_root / "reports" / "asset_report.json",
      current_root / "poses.yaml"
    };
    for (const auto & path : required) {
      if (!safe_bundle_regular_file(path, current_root)) {
        error = "current map projection is incomplete or unsafe: " + path.string();
        return false;
      }
    }

    try {
      if (read_regular_text_file_checked(
          current_root / "manifest.json", 1024U * 1024U) !=
        map_manifest_json(source_manifest))
      {
        error = "current manifest content differs from the verified source manifest";
        return false;
      }

      struct YamlProjection
      {
        fs::path source;
        fs::path target;
        std::string image_file;
      };
      const std::vector<YamlProjection> yaml_projections = {
        {
          source_manifest.nav_map_yaml,
          current_root / "nav" / "nav_map.yaml",
          "nav_map.pgm"
        },
        {
          source_manifest.localizer_params_yaml,
          current_root / "localizer" / "localizer_params.yaml",
          "localizer_map.png"
        },
        {
          source_manifest.keepout_mask_yaml,
          current_root / "filters" / "keepout_mask.yaml",
          "keepout_mask.pgm"
        },
        {
          source_manifest.speed_mask_yaml,
          current_root / "filters" / "speed_mask.yaml",
          "speed_mask.pgm"
        },
        {
          source_manifest.binary_mask_yaml,
          current_root / "filters" / "binary_mask.yaml",
          "binary_mask.pgm"
        }
      };
      for (const auto & projection : yaml_projections) {
        const auto expected = yaml_with_image_file(
          read_regular_text_file_checked(projection.source),
          projection.image_file);
        const auto actual = read_regular_text_file_checked(projection.target);
        if (actual != expected) {
          error =
            "current YAML projection differs from verified source: " +
            projection.target.string();
          return false;
        }
      }

      constexpr std::uintmax_t kMaximumComparableAssetBytes =
        512ULL * 1024ULL * 1024ULL;
      const std::vector<std::pair<fs::path, fs::path>> exact_projections = {
        {
          source_manifest.nav_map_pgm,
          current_root / "nav" / "nav_map.pgm"
        },
        {
          source_manifest.localizer_map_png,
          current_root / "localizer" / "localizer_map.png"
        },
        {
          source_manifest.keepout_mask_pgm,
          current_root / "filters" / "keepout_mask.pgm"
        },
        {
          source_manifest.speed_mask_pgm,
          current_root / "filters" / "speed_mask.pgm"
        },
        {
          source_manifest.binary_mask_pgm,
          current_root / "filters" / "binary_mask.pgm"
        },
        {
          source_manifest.asset_report_json,
          current_root / "reports" / "asset_report.json"
        },
        {
          source_manifest.poses_yaml,
          current_root / "poses.yaml"
        }
      };
      for (const auto & projection : exact_projections) {
        if (!regular_file_contents_equal_checked(
            projection.first,
            projection.second,
            kMaximumComparableAssetBytes,
            error))
        {
          return false;
        }
      }
    } catch (const std::exception & exc) {
      error = std::string("current projection comparison failed: ") + exc.what();
      return false;
    }
    return true;
  }

  void sync_manifest_to_fixed_entry(
    const MapManifest & manifest,
    const fs::path & fixed_root,
    const bool include_manifest) const
  {
    const auto durable_copy = [](const fs::path & source, const fs::path & target) {
        std::error_code source_error;
        const auto source_status = fs::symlink_status(source, source_error);
        if (source_error ||
          source_status.type() != fs::file_type::regular)
        {
          throw std::runtime_error(
                  "required projection source is not a regular file: " +
                  source.string());
        }
        copy_file_if_exists(source, target);
        durable_sync_regular_file(target);
      };
    const auto durable_yaml_copy =
      [](const fs::path & source, const fs::path & target, const std::string & image_file) {
        std::error_code source_error;
        const auto source_status = fs::symlink_status(source, source_error);
        if (source_error ||
          source_status.type() != fs::file_type::regular)
        {
          throw std::runtime_error(
                  "required YAML projection source is not a regular file: " +
                  source.string());
        }
        fs::create_directories(target.parent_path());
        durable_write_text_file_atomic(
          target,
          yaml_with_image_file(
            read_regular_text_file_checked(source), image_file),
          0644);
      };
    const auto durable_remove_optional =
      [](const fs::path & path) {
        std::error_code remove_error;
        const bool removed = fs::remove(path, remove_error);
        if (remove_error) {
          throw std::runtime_error(
                  "failed to remove stale projection file: " +
                  path.string() + " error=" + remove_error.message());
        }
        if (removed) {
          durable_sync_directory(path.parent_path());
        }
      };
    const auto durable_optional_copy =
      [&](const fs::path & source, const fs::path & target) {
        std::error_code source_error;
        const auto source_status = fs::symlink_status(source, source_error);
        if (!source_error &&
          source_status.type() == fs::file_type::regular)
        {
          durable_copy(source, target);
        } else if (!source_error &&
          source_status.type() == fs::file_type::not_found)
        {
          durable_remove_optional(target);
        } else if (source_error == std::errc::no_such_file_or_directory) {
          durable_remove_optional(target);
        } else {
          throw std::runtime_error(
                  "optional projection source is unsafe or unreadable: " +
                  source.string() +
                  (source_error ? " error=" + source_error.message() : ""));
        }
      };

    durable_copy(manifest.nav_map_pgm, fixed_root / "nav" / "nav_map.pgm");
    durable_yaml_copy(
      manifest.nav_map_yaml, fixed_root / "nav" / "nav_map.yaml", "nav_map.pgm");
    durable_copy(
      manifest.localizer_map_png, fixed_root / "localizer" / "localizer_map.png");
    durable_yaml_copy(
      manifest.localizer_params_yaml,
      fixed_root / "localizer" / "localizer_params.yaml",
      "localizer_map.png");
    durable_copy(
      manifest.keepout_mask_pgm, fixed_root / "filters" / "keepout_mask.pgm");
    durable_yaml_copy(
      manifest.keepout_mask_yaml,
      fixed_root / "filters" / "keepout_mask.yaml",
      "keepout_mask.pgm");
    durable_optional_copy(
      keepout_semantic_json_path(manifest),
      fixed_root / "filters" / "keepout_semantic_layer.json");
    const auto keepout_commit =
      manifest.keepout_mask_yaml.parent_path() / "keepout_commit.json";
    const auto fixed_keepout_commit =
      fixed_root / "filters" / "keepout_commit.json";
    durable_optional_copy(keepout_commit, fixed_keepout_commit);
    durable_copy(
      manifest.speed_mask_pgm, fixed_root / "filters" / "speed_mask.pgm");
    durable_yaml_copy(
      manifest.speed_mask_yaml,
      fixed_root / "filters" / "speed_mask.yaml",
      "speed_mask.pgm");
    durable_copy(
      manifest.binary_mask_pgm, fixed_root / "filters" / "binary_mask.pgm");
    durable_yaml_copy(
      manifest.binary_mask_yaml,
      fixed_root / "filters" / "binary_mask.yaml",
      "binary_mask.pgm");
    durable_copy(
      manifest.asset_report_json, fixed_root / "reports" / "asset_report.json");
    durable_copy(manifest.poses_yaml, fixed_root / "poses.yaml");
    if (include_manifest) {
      const auto fixed_manifest = fixed_root / "manifest.json";
      durable_write_text_file_atomic(
        fixed_manifest, map_manifest_json(manifest), 0644);
    }

    // The activation journal may be removed only after every compatibility
    // projection and every directory entry needed to reach it is durable.
    for (const auto & child :
      std::array<fs::path, 4>{
        fixed_root / "nav",
        fixed_root / "localizer",
        fixed_root / "filters",
        fixed_root / "reports"})
    {
      durable_sync_directory(child);
    }
    durable_sync_directory(fixed_root);
    durable_sync_directory(fixed_root.parent_path());
  }

  void remove_current_map_entry(const std::string & building_id, const std::string & floor_id) const
  {
    const auto floor_root = map_catalog_.floor_root_path(building_id, floor_id);
    const auto current_root = map_catalog_.floor_current_root_path(building_id, floor_id);
    if (current_root.filename() != "current" || current_root.parent_path() != floor_root) {
      throw std::runtime_error("refusing unsafe current map reset path: " + current_root.string());
    }

    std::error_code ec;
    fs::remove_all(current_root, ec);
    if (!ec) {
      return;
    }

    const auto remove_error = ec.message();
    std::error_code status_ec;
    const auto status = fs::symlink_status(current_root, status_ec);
    if (status_ec) {
      throw std::runtime_error(
              "failed to reset current map entry: " + current_root.string() +
              " remove_all=" + remove_error + " status=" + status_ec.message());
    }
    if (!fs::exists(status)) {
      return;
    }

    // Old dashboard/test runs may leave current/ as a non-empty root-owned directory.
    // Renaming only needs write permission on the floor directory, then the new
    // current/ can be created by the API runtime user.
    fs::path stale_root;
    for (int index = 0; index < 100; ++index) {
      stale_root = floor_root /
        (".stale_current_" + utc_timestamp_compact() + "_" + std::to_string(::getpid()) + "_" +
        std::to_string(index));
      if (!fs::exists(stale_root)) {
        break;
      }
    }

    std::error_code rename_ec;
    fs::rename(current_root, stale_root, rename_ec);
    if (rename_ec) {
      throw std::runtime_error(
              "failed to reset current map entry: " + current_root.string() +
              " remove_all=" + remove_error + " rename=" + rename_ec.message());
    }

    std::error_code cleanup_ec;
    fs::remove_all(stale_root, cleanup_ec);
    if (cleanup_ec) {
      RCLCPP_WARN(
        node_.get_logger(),
        "quarantined stale current map entry at %s after reset; cleanup skipped: %s",
        stale_root.string().c_str(), cleanup_ec.message().c_str());
    }
  }

  fs::path map_activation_journal_path() const
  {
    return fs::path(config_.maps_root) / ".map_activation_transaction.v1";
  }

  fs::path map_asset_integrity_marker_path() const
  {
    return fs::path(config_.maps_root) / ".map_asset_integrity_degraded.v1";
  }

  struct PersistedMapAssetIntegrityMarker
  {
    std::string map_id;
    std::string non_keepout_asset_digest;

    bool repairable_for(const std::string & requested_map_id) const
    {
      return map_id == requested_map_id &&
             robot_api_server::is_canonical_sha256_digest(
        non_keepout_asset_digest);
    }
  };

  void persist_map_asset_integrity_degraded(
    const std::string & map_id,
    const std::string & reason,
    const std::string & non_keepout_asset_digest = "unknown")
  {
    const auto safe_map_id =
      safe_asset_id(map_id) ? map_id : std::string("unknown");
    const auto safe_non_keepout_digest =
      robot_api_server::is_canonical_sha256_digest(
      non_keepout_asset_digest) ?
      non_keepout_asset_digest : std::string("unknown");
    std::ostringstream marker;
    marker << "schema=njrh.map_asset_integrity_degraded.v1\n"
           << "map_id=" << safe_map_id << "\n"
           << "non_keepout_asset_digest=" << safe_non_keepout_digest << "\n"
           << "reason_digest=" << fixed_hex(fnv1a64(reason), 16) << "\n";
    durable_write_text_file_atomic(
      map_asset_integrity_marker_path(), marker.str());
    keepout_integrity_degraded_.store(true);
  }

  void clear_map_asset_integrity_degraded()
  {
    if (map_activation_journal_blocks_mutation()) {
      throw std::runtime_error(
              "cannot clear map integrity latch while an activation journal exists");
    }
    durable_remove_file(map_asset_integrity_marker_path());
    keepout_integrity_degraded_.store(false);
  }

  std::optional<PersistedMapAssetIntegrityMarker>
  persisted_map_asset_integrity_marker() const
  {
    const auto path = map_asset_integrity_marker_path();
    std::error_code error;
    const auto status = fs::symlink_status(path, error);
    if (status.type() == fs::file_type::not_found &&
      (!error || error == std::errc::no_such_file_or_directory))
    {
      return std::nullopt;
    }
    if (error || status.type() != fs::file_type::regular ||
      fs::hard_link_count(path, error) != 1U || error ||
      fs::file_size(path, error) > 4096U || error)
    {
      return PersistedMapAssetIntegrityMarker{"unknown", "unknown"};
    }
    try {
      std::istringstream input(read_text_file(path));
      std::map<std::string, std::string> fields;
      std::string line;
      while (std::getline(input, line)) {
        if (line.empty()) {
          continue;
        }
        const auto separator = line.find('=');
        if (separator == std::string::npos ||
          line.find('=', separator + 1U) != std::string::npos ||
          !fields.emplace(
            line.substr(0U, separator),
            line.substr(separator + 1U)).second)
        {
          return PersistedMapAssetIntegrityMarker{"unknown", "unknown"};
        }
      }
      if (fields.size() != 4U ||
        fields["schema"] != "njrh.map_asset_integrity_degraded.v1" ||
        !safe_asset_id(fields["map_id"]) ||
        (fields["non_keepout_asset_digest"] != "unknown" &&
        !robot_api_server::is_canonical_sha256_digest(
          fields["non_keepout_asset_digest"])) ||
        fields["reason_digest"].size() != 16U)
      {
        return PersistedMapAssetIntegrityMarker{"unknown", "unknown"};
      }
      return PersistedMapAssetIntegrityMarker{
        fields["map_id"], fields["non_keepout_asset_digest"]};
    } catch (const std::exception &) {
      return PersistedMapAssetIntegrityMarker{"unknown", "unknown"};
    }
  }

  bool map_asset_integrity_degraded()
  {
    if (keepout_integrity_degraded_.load()) {
      return true;
    }
    if (map_activation_journal_blocks_mutation()) {
      return true;
    }
    if (const auto persisted = persisted_map_asset_integrity_marker()) {
      keepout_integrity_degraded_.store(true);
      return true;
    }
    return false;
  }

  void restore_map_asset_integrity_state()
  {
    const auto persisted = persisted_map_asset_integrity_marker();
    if (persisted) {
      keepout_integrity_degraded_.store(true);
    }
    const bool marker_repairable =
      persisted && persisted->repairable_for(persisted->map_id);
    std::unique_ptr<MapAssetCommitTransaction> repair_transaction;
    bool unexplained_failure = false;
    bool marker_map_seen = false;
    if (marker_repairable) {
      try {
        repair_transaction =
          std::make_unique<MapAssetCommitTransaction>(fs::path(config_.maps_root));
      } catch (const std::exception & error) {
        unexplained_failure = true;
        RCLCPP_ERROR(
          node_.get_logger(),
          "failed to open the startup keepout repair transaction: %s",
          error.what());
      }
    }
    for (const auto & manifest :
      map_catalog_.read_all_map_manifests(false))
    {
      if (manifest.schema != "njrh.map_manifest.v2" ||
        manifest.asset_epoch == 0U || manifest.asset_digest.empty())
      {
        continue;
      }
      if (persisted && manifest.map_id == persisted->map_id) {
        marker_map_seen = true;
      }
      try {
        if (repair_transaction) {
          (void)verify_map_asset_identity_snapshot(
            manifest, fs::path(config_.maps_root), *repair_transaction);
        } else {
          verify_map_asset_identity(manifest, fs::path(config_.maps_root));
        }
      } catch (const std::exception & error) {
        bool explained_keepout_crash = false;
        if (repair_transaction && persisted &&
          manifest.map_id == persisted->map_id)
        {
          try {
            const auto inspected =
              inspect_map_asset_identity_for_keepout_repair(
              manifest, fs::path(config_.maps_root), *repair_transaction);
            explained_keepout_crash =
              inspected.non_keepout_asset_digest ==
              persisted->non_keepout_asset_digest;
          } catch (const std::exception & inspection_error) {
            RCLCPP_ERROR(
              node_.get_logger(),
              "startup keepout repair proof failed for %s: %s",
              manifest.map_id.c_str(), inspection_error.what());
          }
        }
        if (!explained_keepout_crash) {
          unexplained_failure = true;
        }
        RCLCPP_ERROR(
          node_.get_logger(),
          "map identity verification failed closed for %s%s: %s",
          manifest.map_id.c_str(),
          explained_keepout_crash ?
          " (preserving exact keepout crash-repair proof)" :
          " (unexplained integrity failure)",
          error.what());
      }
    }
    if (marker_repairable && !marker_map_seen) {
      unexplained_failure = true;
      RCLCPP_ERROR(
        node_.get_logger(),
        "persistent keepout repair marker refers to a missing map: %s",
        persisted->map_id.c_str());
    }
    if (!unexplained_failure) {
      return;
    }
    try {
      // A single persistent marker may remain repairable only when it fully
      // explains the sole aggregate mismatch. Any unrelated failure must
      // collapse the global latch to an unrepairable state so repairing one
      // map cannot accidentally clear another map's fault.
      persist_map_asset_integrity_degraded(
        "unknown", "startup map identity verification found an unexplained failure");
    } catch (const std::exception & marker_error) {
      keepout_integrity_degraded_.store(true);
      RCLCPP_ERROR(
        node_.get_logger(),
        "failed to persist the unrepairable startup integrity latch: %s",
        marker_error.what());
    }
  }

  void write_map_activation_journal(const MapManifest & manifest) const
  {
    const auto existing = read_map_activation_journal();
    if (existing &&
      ((*existing)[0] != manifest.building_id ||
      (*existing)[1] != manifest.floor_id ||
      (*existing)[2] != manifest.map_id))
    {
      throw std::runtime_error(
              "refusing to overwrite a different pending map activation");
    }
    std::ostringstream journal;
    journal << "schema=njrh.map_activation_transaction.v1\n"
            << "building_id=" << manifest.building_id << "\n"
            << "floor_id=" << manifest.floor_id << "\n"
            << "map_id=" << manifest.map_id << "\n";
    durable_write_text_file_atomic(
      map_activation_journal_path(), journal.str());
  }

  std::optional<std::array<std::string, 3>> read_map_activation_journal() const
  {
    const auto path = map_activation_journal_path();
    std::error_code error;
    const auto status = fs::symlink_status(path, error);
    if (status.type() == fs::file_type::not_found &&
      (!error || error == std::errc::no_such_file_or_directory))
    {
      return std::nullopt;
    }
    if (error || status.type() != fs::file_type::regular ||
      fs::hard_link_count(path, error) != 1U || error ||
      fs::file_size(path, error) > 4096U || error)
    {
      throw std::runtime_error(
              "map activation journal is not a bounded single-link regular file");
    }
    std::istringstream input(read_text_file(path));
    std::map<std::string, std::string> fields;
    std::string line;
    while (std::getline(input, line)) {
      if (line.empty()) {
        continue;
      }
      const auto separator = line.find('=');
      if (separator == std::string::npos ||
        line.find('=', separator + 1U) != std::string::npos)
      {
        throw std::runtime_error("map activation journal is malformed");
      }
      const auto key = line.substr(0U, separator);
      const auto value = line.substr(separator + 1U);
      if (!fields.emplace(key, value).second) {
        throw std::runtime_error(
                "map activation journal has duplicate fields");
      }
    }
    if (fields.size() != 4U ||
      fields["schema"] != "njrh.map_activation_transaction.v1" ||
      !safe_asset_id(fields["building_id"]) ||
      !safe_asset_id(fields["floor_id"]) ||
      !safe_asset_id(fields["map_id"]))
    {
      throw std::runtime_error("map activation journal has invalid fields");
    }
    return std::array<std::string, 3>{
      fields["building_id"], fields["floor_id"], fields["map_id"]};
  }

  bool map_activation_journal_blocks_mutation() const
  {
    try {
      return read_map_activation_journal().has_value();
    } catch (const std::exception &) {
      // Invalid, inaccessible, or link-based journal state is never
      // equivalent to an absent journal.
      return true;
    }
  }

  void activate_map_manifest_locked(
    MapManifest requested_manifest,
    MapAssetCommitTransaction & transaction,
    const bool require_exact_source)
  {
    const auto refreshed =
      map_catalog_.find_map_by_id(requested_manifest.map_id);
    if (!refreshed ||
      refreshed->building_id != requested_manifest.building_id ||
      refreshed->floor_id != requested_manifest.floor_id ||
      refreshed->map_id != requested_manifest.map_id)
    {
      if (require_exact_source) {
        throw ExactMapAssetSourceDrift(
                "map source identity changed before activation transaction");
      }
      throw std::runtime_error(
              "map identity changed before activation transaction");
    }

    MapManifest manifest = *refreshed;
    if (require_exact_source) {
      if (!same_exact_map_asset_source(manifest, requested_manifest)) {
        throw ExactMapAssetSourceDrift(
                "map source identity changed before activation transaction");
      }
      try {
        manifest = verify_map_asset_identity_snapshot(
          manifest, fs::path(config_.maps_root), transaction).manifest;
      } catch (const std::exception & exc) {
        throw ExactMapAssetSourceDrift(
                std::string("map source verification drifted before activation: ") +
                exc.what());
      }
      if (!same_exact_map_asset_source(manifest, requested_manifest)) {
        throw ExactMapAssetSourceDrift(
                "verified map source identity drifted before activation transaction");
      }
    } else if (manifest.schema == "njrh.map_manifest.v2") {
      // Pending-journal recovery may stamp a genuinely legacy manifest once,
      // but an already authoritative v2 identity is immutable evidence. Never
      // rebind changed v2 content to a new epoch while recovering a crash.
      manifest = verify_map_asset_identity_snapshot(
        manifest, fs::path(config_.maps_root), transaction).manifest;
    }

    std::string error;
    if (!validate_map_manifest_assets(manifest, error)) {
      throw std::runtime_error(error);
    }
    write_map_activation_journal(manifest);
    try {
      for (auto other : map_catalog_.read_floor_map_manifests(
          manifest.building_id, manifest.floor_id, false))
      {
        if (other.map_id == manifest.map_id) {
          continue;
        }
        if (other.active) {
          stamp_map_asset_identity(
            other, fs::path(config_.maps_root), transaction, false);
        }
      }

      stamp_map_asset_identity(
        manifest, fs::path(config_.maps_root), transaction, true);
      if (require_exact_source &&
        !same_exact_map_asset_source(manifest, requested_manifest))
      {
        throw ExactMapAssetSourceDrift(
                "map source identity changed while activating the exact source");
      }

      const auto current_root = map_catalog_.floor_current_root_path(
        manifest.building_id, manifest.floor_id);
      remove_current_map_entry(manifest.building_id, manifest.floor_id);
      sync_manifest_to_fixed_entry(manifest, current_root, true);

      // Keep the historical fixed role files in the floor root as a
      // compatibility shim for older tools.
      sync_manifest_to_fixed_entry(
        manifest,
        map_catalog_.floor_root_path(
          manifest.building_id, manifest.floor_id),
        false);
      durable_remove_file(map_activation_journal_path());
    } catch (...) {
      try {
        persist_map_asset_integrity_degraded(
          manifest.map_id, "map activation transaction failed");
      } catch (const std::exception & marker_error) {
        keepout_integrity_degraded_.store(true);
        RCLCPP_ERROR(
          node_.get_logger(),
          "failed to persist activation integrity latch for %s: %s",
          manifest.map_id.c_str(), marker_error.what());
      }
      throw;
    }
  }

  void activate_map_manifest(
    MapManifest requested_manifest,
    MapAssetCommitTransaction & transaction)
  {
    activate_map_manifest_locked(
      std::move(requested_manifest), transaction, true);
  }

  void activate_map_manifest(MapManifest requested_manifest)
  {
    MapAssetCommitTransaction transaction{fs::path(config_.maps_root)};
    activate_map_manifest_locked(
      std::move(requested_manifest), transaction, false);
  }

  void recover_pending_map_activation()
  {
    const auto pending = read_map_activation_journal();
    if (!pending) {
      return;
    }
    const auto manifest = map_catalog_.find_map_by_id((*pending)[2]);
    if (!manifest ||
      manifest->building_id != (*pending)[0] ||
      manifest->floor_id != (*pending)[1])
    {
      throw std::runtime_error(
              "pending map activation target is no longer present");
    }
    activate_map_manifest(*manifest);
  }

  void clear_fixed_floor_entries(const std::string & building_id, const std::string & floor_id) const
  {
    const auto floor_root = map_catalog_.floor_root_path(building_id, floor_id);
    std::error_code ec;
    remove_current_map_entry(building_id, floor_id);
    fs::remove_all(floor_root / "nav", ec);
    fs::remove_all(floor_root / "localizer", ec);
    fs::remove_all(floor_root / "filters", ec);
    fs::remove_all(floor_root / "reports", ec);
    fs::remove(floor_root / "poses.yaml", ec);
  }

  void ensure_legacy_floor_map_manifest(
    const std::string & building_id,
    const std::string & floor_id)
  {
    if (!safe_asset_id(building_id) || !safe_asset_id(floor_id)) {
      return;
    }
    const auto maps_root = map_catalog_.floor_maps_root_path(building_id, floor_id);
    if (fs::exists(maps_root) && fs::is_directory(maps_root)) {
      for (const auto & entry : fs::directory_iterator(maps_root)) {
        if (entry.is_directory() && fs::exists(entry.path() / "manifest.json")) {
          return;
        }
      }
    }

    const auto floor_root = map_catalog_.floor_root_path(building_id, floor_id);
    const auto legacy_nav_yaml = floor_root / "nav" / "nav_map.yaml";
    const auto legacy_nav_pgm = floor_root / "nav" / "nav_map.pgm";
    const auto legacy_localizer_png = floor_root / "localizer" / "localizer_map.png";
    const auto legacy_localizer_params = floor_root / "localizer" / "localizer_params.yaml";
    const auto legacy_report = floor_root / "reports" / "asset_report.json";
    const auto legacy_poses = floor_root / "poses.yaml";
    if (!fs::exists(legacy_nav_yaml) || !fs::exists(legacy_nav_pgm) ||
      !fs::exists(legacy_localizer_png) || !fs::exists(legacy_localizer_params))
    {
      return;
    }

    MapManifest manifest;
    manifest.map_id = "legacy_" + fixed_hex(fnv1a64(floor_root.string()), 12);
    manifest.display_name = "legacy_" + floor_id;
    manifest.safe_map_name = safe_file_stem_from_display_name(manifest.display_name);
    manifest.building_id = building_id;
    manifest.floor_id = floor_id;
    manifest.created_at = utc_timestamp_iso8601();
    manifest.active = true;
    manifest.root = map_catalog_.map_root_path(building_id, floor_id, manifest.map_id);
    fill_manifest_paths(manifest);

    copy_file_if_exists(legacy_nav_pgm, manifest.nav_map_pgm);
    copy_yaml_with_image_if_exists(
      legacy_nav_yaml, manifest.nav_map_yaml, manifest.nav_map_pgm.filename().string());
    copy_file_if_exists(legacy_localizer_png, manifest.localizer_map_png);
    copy_yaml_with_image_if_exists(
      legacy_localizer_params,
      manifest.localizer_params_yaml,
      manifest.localizer_map_png.filename().string());
    copy_file_if_exists(floor_root / "filters" / "keepout_mask.pgm", manifest.keepout_mask_pgm);
    copy_yaml_with_image_if_exists(
      floor_root / "filters" / "keepout_mask.yaml",
      manifest.keepout_mask_yaml,
      "keepout_mask.pgm");
    copy_file_if_exists(floor_root / "filters" / "speed_mask.pgm", manifest.speed_mask_pgm);
    copy_yaml_with_image_if_exists(
      floor_root / "filters" / "speed_mask.yaml",
      manifest.speed_mask_yaml,
      "speed_mask.pgm");
    copy_file_if_exists(floor_root / "filters" / "binary_mask.pgm", manifest.binary_mask_pgm);
    copy_yaml_with_image_if_exists(
      floor_root / "filters" / "binary_mask.yaml",
      manifest.binary_mask_yaml,
      "binary_mask.pgm");
    copy_file_if_exists(legacy_report, manifest.asset_report_json);
    if (!fs::exists(manifest.asset_report_json)) {
      write_text_file(manifest.asset_report_json, "{}\n");
    }
    copy_file_if_exists(legacy_poses, manifest.poses_yaml);
    if (!fs::exists(manifest.poses_yaml)) {
      write_text_file(manifest.poses_yaml, "poses: []\n");
    }
    std::string error;
    if (validate_map_manifest_assets(manifest, error)) {
      stamp_map_asset_identity(manifest, fs::path(config_.maps_root));
      activate_map_manifest(manifest);
    } else {
      RCLCPP_WARN(node_.get_logger(), "legacy map manifest created but not activated: %s", error.c_str());
    }
  }

  HttpResponse handle_maps()
  {
    std::ostringstream body;
    body << "{\"ok\":true,\"runtime_maps\":[";
    bool first = true;
    if (fs::exists(config_.runtime_maps_dir)) {
      for (const auto & entry : fs::directory_iterator(config_.runtime_maps_dir)) {
        if (!entry.is_regular_file() || entry.path().extension() != ".yaml") {
          continue;
        }
        const auto stem = entry.path().stem().string();
        if (stem.size() >= 10 && stem.substr(stem.size() - 10) == ".localizer") {
          continue;
        }
        const auto pgm = entry.path().parent_path() / (stem + ".pgm");
        if (!fs::exists(pgm)) {
          continue;
        }
        if (!first) {
          body << ",";
        }
        first = false;
        const auto map_info = read_nav_map_info(entry.path());
        body << "{\"name\":" << json_string(stem)
             << ",\"yaml\":" << json_string(entry.path().string())
             << ",\"pgm\":" << json_string(pgm.string())
             << ",\"map_info\":" << map_info_json(map_info) << "}";
      }
    }

    const auto manifests = map_catalog_.read_all_map_manifests(false);
    body << "],\"floor_maps\":[";
    first = true;
    for (const auto & manifest : manifests) {
      if (!first) {
        body << ",";
      }
      first = false;
      body << "{\"map_id\":" << json_string(manifest.map_id)
           << ",\"display_name\":" << json_string(manifest.display_name)
           << ",\"map_name\":" << json_string(manifest.display_name)
           << ",\"building_id\":" << json_string(manifest.building_id)
           << ",\"floor_id\":" << json_string(manifest.floor_id)
           << ",\"asset_epoch\":" << manifest.asset_epoch
           << ",\"asset_digest\":" << json_string(manifest.asset_digest)
           << ",\"active\":" << (manifest.active ? "true" : "false")
           << ",\"nav_map_yaml\":" << json_string(manifest.nav_map_yaml.string())
           << ",\"localizer_map_png\":" << json_string(manifest.localizer_map_png.string())
           << ",\"map_info\":" << map_info_json(read_nav_map_info(manifest.nav_map_yaml))
           << ",\"manifest_json\":" << json_string(manifest.manifest_json.string()) << "}";
    }

    body << "],\"floors\":[";
    first = true;
    if (fs::exists(config_.maps_root)) {
      for (const auto & building : fs::directory_iterator(config_.maps_root)) {
        if (!building.is_directory()) {
          continue;
        }
        for (const auto & floor : fs::directory_iterator(building.path())) {
          if (!floor.is_directory()) {
            continue;
          }
          const auto building_id = building.path().filename().string();
          const auto floor_id = floor.path().filename().string();
          const auto active = map_catalog_.active_floor_map(building_id, floor_id);
          const auto current_root = floor.path() / "current";
          const auto root = fs::exists(current_root / "nav" / "nav_map.yaml") ? current_root : floor.path();
          const auto nav_yaml = root / "nav" / "nav_map.yaml";
          const auto nav_pgm = root / "nav" / "nav_map.pgm";
          const auto localizer_png = root / "localizer" / "localizer_map.png";
          const auto localizer_params = root / "localizer" / "localizer_params.yaml";
          if (!fs::exists(nav_yaml) || !fs::exists(nav_pgm) || !fs::exists(localizer_png) ||
            !fs::exists(localizer_params))
          {
            continue;
          }
          if (!first) {
            body << ",";
          }
          first = false;
          body << "{\"building_id\":" << json_string(building_id)
               << ",\"floor_id\":" << json_string(floor_id)
               << ",\"active_map_id\":" << json_string(active ? active->map_id : "")
               << ",\"active_display_name\":" << json_string(active ? active->display_name : "")
               << ",\"active_asset_epoch\":" << (active ? active->asset_epoch : 0U)
               << ",\"active_asset_digest\":"
               << json_string(active ? active->asset_digest : "")
               << ",\"nav_map_yaml\":" << json_string(nav_yaml.string())
               << ",\"nav_map_pgm\":" << json_string(nav_pgm.string())
               << ",\"localizer_map_png\":" << json_string(localizer_png.string())
               << ",\"localizer_params_yaml\":" << json_string(localizer_params.string())
               << ",\"map_info\":" << map_info_json(read_nav_map_info(nav_yaml)) << "}";
        }
      }
    }
    body << "]}";
    return {200, "application/json", body.str()};
  }

  HttpResponse handle_delete_map(const std::string & body)
  {
    std::lock_guard<std::mutex> keepout_guard(asset_mutation_mutex_);
    std::lock_guard<std::mutex> elevator_guard(cross_asset_commit_mutex_);
    if (const auto blocked = ports_.floor_runtime_interlock_response("map_delete")) {
      return *blocked;
    }
    if (map_asset_integrity_degraded()) {
      return {
        503,
        "application/json",
        error_json("keepout integrity is degraded; repair the keepout layer before deleting maps")};
    }
    const auto map_id = json_string_value(body, "map_id");
    const auto building_id = json_string_value(body, "building_id");
    const auto floor_id = json_string_value(body, "floor_id");

    if (!map_id) {
      if (building_id || floor_id) {
        return {
          400,
          "application/json",
          error_json("delete by map_id only; refusing to delete non-empty building/floor assets")
        };
      }
      return {400, "application/json", error_json("map_id is required")};
    }
    if (!safe_asset_id(*map_id)) {
      return {400, "application/json", error_json("valid map_id is required")};
    }

    const auto manifest = map_catalog_.find_map_by_id(*map_id);
    if (!manifest) {
      return {404, "application/json", error_json("map_id not found: " + *map_id)};
    }
    const auto active_map_delete_response = [](const std::string & detail) {
        std::ostringstream response;
        response << "{\"ok\":false,\"code\":\"ACTIVE_MAP_DELETE_DISABLED\","
                 << "\"detail\":" << json_string(detail) << "}";
        return HttpResponse{409, "application/json", response.str()};
      };
    const auto unsafe_map_asset_response = [](const std::string & detail) {
        std::ostringstream response;
        response << "{\"ok\":false,\"code\":\"UNSAFE_MAP_ASSET_PATH\","
                 << "\"detail\":" << json_string(detail) << "}";
        return HttpResponse{409, "application/json", response.str()};
      };
    const auto map_is_bound_to_runtime = [this](const MapManifest & candidate) {
        const auto context = read_runtime_map_context();
        return context &&
               context->building_id == candidate.building_id &&
               context->floor_id == candidate.floor_id &&
               context->map_id == candidate.map_id;
      };
    if (manifest->active || map_is_bound_to_runtime(*manifest)) {
      return active_map_delete_response(
        "refusing to delete an active or runtime-bound map; stop the runtime and "
        "select another map explicitly before deletion");
    }
    const auto reference = ports_.query_elevator_reference(*manifest);
    if (reference.status >= 400) {
      return reference;
    }
    if (json_bool_value(reference.body, "referenced", false)) {
      const auto referenced_release_id =
        json_string_value(reference.body, "release_id");
      std::ostringstream response;
      response << "{\"ok\":false,"
               << "\"code\":\"ELEVATOR_CONFIG_MAP_IN_USE\","
               << "\"detail\":\"refusing to delete a map referenced by the current "
                  "elevator configuration release\","
               << "\"building_id\":" << json_string(manifest->building_id) << ","
               << "\"floor_id\":" << json_string(manifest->floor_id) << ","
               << "\"map_id\":" << json_string(manifest->map_id) << ","
               << "\"release_id\":"
               << (referenced_release_id ?
        json_string(*referenced_release_id) :
        std::string("null"))
               << "}";
      return {409, "application/json", response.str()};
    }

    std::uintmax_t entries_deleted = 0;
    if (const auto blocked = ports_.floor_runtime_interlock_response("map_delete_commit")) {
      return *blocked;
    }
    try {
      MapAssetCommitTransaction asset_transaction{fs::path(config_.maps_root)};
      (void)asset_transaction;
      if (persisted_map_asset_integrity_marker()) {
        return {
          503,
          "application/json",
          error_json(
            "map asset integrity is persistently degraded; deletion is blocked")};
      }
      const auto commit_manifest = map_catalog_.find_map_by_id(*map_id);
      if (!commit_manifest) {
        return {404, "application/json", error_json("map_id disappeared before deletion: " + *map_id)};
      }
      if (commit_manifest->active || map_is_bound_to_runtime(*commit_manifest)) {
        return active_map_delete_response(
          "map became active or runtime-bound while deletion was pending; retry only "
          "after an explicit safe map selection");
      }
      if (commit_manifest->building_id != manifest->building_id ||
        commit_manifest->floor_id != manifest->floor_id ||
        commit_manifest->map_id != manifest->map_id)
      {
        return unsafe_map_asset_response(
          "map identity changed while deletion was pending");
      }
      const auto expected_root = map_catalog_.map_root_path(
        commit_manifest->building_id,
        commit_manifest->floor_id,
        commit_manifest->map_id);
      if (!same_normalized_path(commit_manifest->root, expected_root) ||
        !safe_bundle_directory(expected_root, fs::path(config_.maps_root)))
      {
        return unsafe_map_asset_response(
          "map root is not the exact real directory beneath maps_root");
      }

      const auto tombstone = map_delete_tombstone_path(expected_root);
      std::error_code ec;
      if (fs::symlink_status(tombstone, ec).type() !=
        fs::file_type::not_found)
      {
        return {
          500,
          "application/json",
          error_json("refusing to replace an existing map deletion tombstone")};
      }
      ec.clear();
      fs::rename(expected_root, tombstone, ec);
      if (ec) {
        return {
          500,
          "application/json",
          error_json("failed to quarantine map asset before deletion: " +
            expected_root.string())};
      }
      // The rename is the logical deletion commit. Persist it before
      // recursively cleaning the now-unreachable tombstone.
      durable_sync_directory(expected_root.parent_path());

      entries_deleted = fs::remove_all(tombstone, ec);
      if (ec) {
        return {
          500,
          "application/json",
          error_json(
            "map was quarantined but tombstone cleanup failed: " +
            tombstone.string())};
      }
      durable_sync_directory(expected_root.parent_path());
    } catch (const std::exception & exc) {
      return {500, "application/json", error_json(exc.what())};
    }

    std::ostringstream response;
    response << "{\"ok\":true,"
             << "\"deleted\":" << (entries_deleted > 0U ? "true" : "false") << ","
             << "\"map_id\":" << json_string(manifest->map_id) << ","
             << "\"display_name\":" << json_string(manifest->display_name) << ","
             << "\"building_id\":" << json_string(manifest->building_id) << ","
             << "\"floor_id\":" << json_string(manifest->floor_id) << ","
             << "\"active_deleted\":" << (manifest->active ? "true" : "false") << ","
             << "\"entries_deleted\":" << entries_deleted << "}";
    return {200, "application/json", response.str()};
  }

  struct ManifestLookupResult
  {
    bool ok{false};
    MapManifest manifest;
    HttpResponse error{400, "application/json", "{}"};
  };

  std::optional<std::string> query_value(const HttpRequest & request, const std::string & key) const
  {
    const auto it = request.query.find(key);
    if (it == request.query.end() || it->second.empty()) {
      return std::nullopt;
    }
    return it->second;
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
        result.error = {404, "application/json", error_json("map_id not found: " + *requested_map_id)};
        return result;
      }
      if (requested_building_id && *requested_building_id != manifest->building_id) {
        result.error = {400, "application/json", error_json("map_id does not belong to requested building")};
        return result;
      }
      if (requested_floor_id && *requested_floor_id != manifest->floor_id) {
        result.error = {400, "application/json", error_json("map_id does not belong to requested floor")};
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
        manifest = map_catalog_.find_floor_map_by_name(*requested_building_id, *requested_floor_id, requested_map_name, error);
        if (!manifest) {
          result.error = {
            error.empty() ? 404 : 400,
            "application/json",
            error_json(error.empty() ? "map_name not found: " + requested_map_name : error)
          };
          return result;
        }
      } else {
        manifest = map_catalog_.active_floor_map(*requested_building_id, *requested_floor_id);
        if (!manifest) {
          result.error = {
            404,
            "application/json",
            error_json("active map not found for floor: " + *requested_building_id + "/" + *requested_floor_id)
          };
          return result;
        }
      }
    }

    result.ok = true;
    result.manifest = *manifest;
    return result;
  }

  HttpResponse handle_get_keepout_filter(const HttpRequest & request)
  {
    const auto lookup = resolve_map_manifest_from_query(request);
    if (!lookup.ok) {
      return lookup.error;
    }
    try {
      KeepoutLayerModule keepout_module;
      const auto summary = keepout_module.inspect(lookup.manifest);
      std::ostringstream response;
      response << std::fixed << std::setprecision(9)
               << "{\"ok\":true,"
               << "\"building_id\":" << json_string(lookup.manifest.building_id) << ","
               << "\"floor_id\":" << json_string(lookup.manifest.floor_id) << ","
               << "\"map_id\":" << json_string(lookup.manifest.map_id) << ","
               << "\"display_name\":" << json_string(lookup.manifest.display_name) << ","
               << "\"map_name\":" << json_string(lookup.manifest.display_name) << ","
               << "\"active\":" << (lookup.manifest.active ? "true" : "false") << ","
               << "\"revision\":" << json_string(summary.revision) << ","
               << "\"keepout_revision\":" << json_string(summary.revision) << ","
               << "\"mask\":{"
               << "\"width\":" << summary.width << ","
               << "\"height\":" << summary.height << ","
               << "\"resolution\":" << summary.resolution << ","
               << "\"origin\":[" << summary.origin_x << "," << summary.origin_y << ","
               << summary.origin_yaw << "],"
               << "\"active_cells\":" << summary.active_cells << ","
               << "\"occupancy_digest\":" << json_string(summary.occupancy_digest) << "},"
               << "\"filter\":\"keepout\","
               << "\"keepout\":" << keepout_filter_json(lookup.manifest) << ","
               << "\"filters\":{\"keepout\":" << keepout_filter_json(lookup.manifest) << "}"
               << "}";
      return {200, "application/json", response.str()};
    } catch (const std::exception & exc) {
      return {500, "application/json", error_json(exc.what())};
    }
  }

  HttpResponse handle_get_semantic_layer(const HttpRequest & request)
  {
    const auto lookup = resolve_map_manifest_from_query(request);
    if (!lookup.ok) {
      return lookup.error;
    }

    try {
      auto poses = read_floor_poses(lookup.manifest.poses_yaml);
      poses.erase(
        std::remove_if(
          poses.begin(), poses.end(), [](const StoredPose & pose) {
            return is_elevator_internal_pose_type(pose.type) ||
                   is_reserved_elevator_pose_id(pose.id);
          }),
        poses.end());
      KeepoutLayerModule keepout_module;
      const auto keepout_summary = keepout_module.inspect(lookup.manifest);
      const auto keepout_filter = keepout_filter_json(lookup.manifest);
      const auto keepout_payload = keepout_semantic_payload_json(
        read_optional_text_file(keepout_semantic_json_path(lookup.manifest)));
      std::ostringstream response;
      response << "{\"ok\":true,"
               << "\"schema\":\"njrh.semantic_layer.v1\","
               << "\"building_id\":" << json_string(lookup.manifest.building_id) << ","
               << "\"floor_id\":" << json_string(lookup.manifest.floor_id) << ","
               << "\"map_id\":" << json_string(lookup.manifest.map_id) << ","
               << "\"display_name\":" << json_string(lookup.manifest.display_name) << ","
               << "\"map_name\":" << json_string(lookup.manifest.display_name) << ","
               << "\"active\":" << (lookup.manifest.active ? "true" : "false") << ","
               << "\"keepout_revision\":" << json_string(keepout_summary.revision) << ","
               << "\"poses_yaml\":" << json_string(lookup.manifest.poses_yaml.string()) << ","
               << "\"poses\":" << poses_json_array(poses) << ","
               << "\"filters\":{\"keepout\":" << keepout_filter << "},"
               << "\"keepout_filter\":" << keepout_filter << ","
               << "\"keepout\":" << keepout_payload
               << "}";
      return {200, "application/json", response.str()};
    } catch (const std::exception & exc) {
      return {500, "application/json", error_json(exc.what())};
    }
  }

  HttpResponse handle_get_poses(const HttpRequest & request)
  {
    const auto lookup = resolve_map_manifest_from_query(request);
    if (!lookup.ok) {
      return lookup.error;
    }

    std::vector<StoredPose> poses;
    try {
      poses = read_floor_poses(lookup.manifest.poses_yaml);
      poses.erase(
        std::remove_if(
          poses.begin(), poses.end(), [](const StoredPose & pose) {
            return is_elevator_internal_pose_type(pose.type) ||
                   is_reserved_elevator_pose_id(pose.id);
          }),
        poses.end());
    } catch (const std::exception & exc) {
      return {500, "application/json", error_json(exc.what())};
    }

    std::ostringstream response;
    response << std::fixed << std::setprecision(6);
    response << "{\"ok\":true,"
             << "\"building_id\":" << json_string(lookup.manifest.building_id) << ","
             << "\"floor_id\":" << json_string(lookup.manifest.floor_id) << ","
             << "\"map_id\":" << json_string(lookup.manifest.map_id) << ","
             << "\"display_name\":" << json_string(lookup.manifest.display_name) << ","
             << "\"map_name\":" << json_string(lookup.manifest.display_name) << ","
             << "\"active\":" << (lookup.manifest.active ? "true" : "false") << ","
             << "\"poses_yaml\":" << json_string(lookup.manifest.poses_yaml.string()) << ","
             << "\"poses\":" << poses_json_array(poses) << "}";
    return {200, "application/json", response.str()};
  }

  HttpResponse handle_save_pose(
    const std::string & body,
    const std::optional<std::string> & forced_pose_id = std::nullopt)
  {
    if (const auto blocked = ports_.floor_runtime_interlock_response("pose_save")) {
      return *blocked;
    }
    const auto requested_building_id = json_string_value(body, "building_id");
    const auto requested_floor_id = json_string_value(body, "floor_id");
    const auto map_id = json_string_value(body, "map_id");
    const auto body_pose_id = json_string_value(body, "pose_id").value_or(json_string_value(body, "id").value_or(""));
    const auto pose_id = forced_pose_id.value_or(body_pose_id);
    if (requested_building_id && !safe_asset_id(*requested_building_id)) {
      return {400, "application/json", error_json("valid building_id is required")};
    }
    if (requested_floor_id && !safe_asset_id(*requested_floor_id)) {
      return {400, "application/json", error_json("valid floor_id is required")};
    }
    if (map_id && !map_id->empty() && !safe_asset_id(*map_id)) {
      return {400, "application/json", error_json("valid map_id is required")};
    }
    if (!safe_pose_id(pose_id)) {
      return {400, "application/json", error_json("valid pose_id is required")};
    }
    const auto requested_pose_type =
      json_string_value(body, "type").value_or("delivery_point");
    if (is_elevator_internal_pose_type(requested_pose_type) ||
      is_reserved_elevator_pose_id(pose_id))
    {
      return {
        400, "application/json",
        error_json(
          "elevator internal poses are reserved for the elevator configuration module")};
    }
    if (forced_pose_id && !body_pose_id.empty() && body_pose_id != *forced_pose_id) {
      return {400, "application/json", error_json("pose_id in body does not match path pose_id")};
    }

    const auto x = json_number_value(body, "x").value_or(
      json_nested_number_value(body, "pose", "x").value_or(std::numeric_limits<double>::quiet_NaN()));
    const auto y = json_number_value(body, "y").value_or(
      json_nested_number_value(body, "pose", "y").value_or(std::numeric_limits<double>::quiet_NaN()));
    const auto yaw = json_number_value(body, "yaw").value_or(
      json_number_value(body, "theta").value_or(
        json_nested_number_value(body, "pose", "yaw").value_or(std::numeric_limits<double>::quiet_NaN())));
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(yaw)) {
      return {400, "application/json", error_json("finite x, y, and yaw are required")};
    }

    std::optional<MapManifest> manifest;
    std::string resolved_building_id;
    std::string resolved_floor_id;
    if (map_id && !map_id->empty()) {
      manifest = map_catalog_.find_map_by_id(*map_id);
      if (!manifest) {
        return {404, "application/json", error_json("map_id not found: " + *map_id)};
      }
      if (requested_building_id && manifest->building_id != *requested_building_id) {
        return {400, "application/json", error_json("map_id does not belong to requested building")};
      }
      if (requested_floor_id && manifest->floor_id != *requested_floor_id) {
        return {400, "application/json", error_json("map_id does not belong to requested floor")};
      }
      resolved_building_id = manifest->building_id;
      resolved_floor_id = manifest->floor_id;
    } else {
      if (!requested_building_id) {
        return {400, "application/json", error_json("valid building_id is required")};
      }
      if (!requested_floor_id) {
        return {400, "application/json", error_json("valid floor_id is required")};
      }
      resolved_building_id = *requested_building_id;
      resolved_floor_id = *requested_floor_id;
      manifest = map_catalog_.active_floor_map(resolved_building_id, resolved_floor_id);
    }

    const auto floor_root = map_catalog_.floor_root_path(resolved_building_id, resolved_floor_id);
    if (!fs::exists(floor_root) || !fs::is_directory(floor_root)) {
      return {404, "application/json", error_json("floor asset does not exist: " + floor_root.string())};
    }

    StoredPose pose;
    pose.id = pose_id;
    pose.name = json_string_value(body, "name").value_or(pose_id);
    pose.type = requested_pose_type;
    pose.x = x;
    pose.y = y;
    pose.yaw = normalize_angle(yaw);

    const auto path = manifest ? manifest->poses_yaml :
      poses_yaml_path(map_catalog_, resolved_building_id, resolved_floor_id);
    try {
      auto poses = read_floor_poses(path);
      bool updated = false;
      for (auto & existing : poses) {
        if (existing.id == pose.id) {
          if (is_elevator_internal_pose_type(existing.type) ||
            is_reserved_elevator_pose_id(existing.id))
          {
            return {
              400, "application/json",
              error_json(
                "elevator internal poses are managed only by the elevator configuration module")};
          }
          existing = pose;
          updated = true;
          break;
        }
      }
      if (!updated) {
        poses.push_back(pose);
      }
      if (const auto blocked = ports_.floor_runtime_interlock_response("pose_save_commit")) {
        return *blocked;
      }
      write_floor_poses(path, poses);
      if (manifest && manifest->active) {
        copy_file_if_exists(path, map_catalog_.floor_current_root_path(resolved_building_id, resolved_floor_id) / "poses.yaml");
        copy_file_if_exists(path, map_catalog_.floor_root_path(resolved_building_id, resolved_floor_id) / "poses.yaml");
      }

      std::ostringstream response;
      response << std::fixed << std::setprecision(6)
               << "{\"ok\":true,"
               << "\"building_id\":" << json_string(resolved_building_id) << ","
               << "\"floor_id\":" << json_string(resolved_floor_id) << ","
               << "\"map_id\":" << json_string(manifest ? manifest->map_id : "") << ","
               << "\"pose_id\":" << json_string(pose.id) << ","
               << "\"updated\":" << (updated ? "true" : "false") << ","
               << "\"poses_yaml\":" << json_string(path.string()) << ","
               << "\"pose\":{\"x\":" << pose.x << ",\"y\":" << pose.y << ",\"yaw\":" << pose.yaw << "}}";
      return {200, "application/json", response.str()};
    } catch (const std::exception & ex) {
      return {500, "application/json", error_json(ex.what())};
    }
  }

  HttpResponse handle_save_current_pose(const std::string & body)
  {
    if (const auto blocked = ports_.floor_runtime_interlock_response("current_pose_save")) {
      return *blocked;
    }
    const auto stripped_body = trim(body);
    if (stripped_body.empty() || stripped_body.front() != '{') {
      return {400, "application/json", error_json("JSON request body is required")};
    }

    std::optional<MapManifest> manifest;
    std::string resolved_building_id;
    std::string resolved_floor_id;
    HttpResponse error;
    if (!resolve_pose_target_manifest(
        json_string_value(body, "building_id"),
        json_string_value(body, "floor_id"),
        json_string_value(body, "map_id"),
        manifest,
        resolved_building_id,
        resolved_floor_id,
        error))
    {
      return error;
    }
    if (!manifest) {
      return {
        404,
        "application/json",
        error_json("active map not found for floor: " + resolved_building_id + "/" + resolved_floor_id)
      };
    }

    const auto floor_root = map_catalog_.floor_root_path(resolved_building_id, resolved_floor_id);
    if (!fs::exists(floor_root) || !fs::is_directory(floor_root)) {
      return {404, "application/json", error_json("floor asset does not exist: " + floor_root.string())};
    }

    const auto body_pose_id = json_string_value(body, "pose_id").value_or(
      json_string_value(body, "id").value_or(""));
    const std::string pose_type = json_string_value(body, "type").value_or("delivery_point");
    const std::string pose_name = json_string_value(body, "name").value_or(
      body_pose_id.empty() ? pose_type : body_pose_id);
    const std::string pose_id =
      body_pose_id.empty() ? generate_current_pose_id(pose_type, pose_name) : body_pose_id;
    if (!safe_pose_id(pose_id)) {
      return {400, "application/json", error_json("valid pose_id is required")};
    }
    if (is_elevator_internal_pose_type(pose_type) ||
      is_reserved_elevator_pose_id(pose_id))
    {
      return {
        400, "application/json",
        error_json(
          "elevator internal poses are reserved for the elevator configuration module")};
    }

    std::string pose_error;
    const auto current_pose = ports_.wait_for_current_robot_pose();
    if (!current_pose.available) {
      (void)pose_error;
      return {503, "application/json", no_fresh_map_robot_pose_json(config_.map_frame, config_.base_frame)};
    }
    std::string context_error;
    bool blocked_by_pending_context = false;
    const auto current_context = confirmed_runtime_map_manifest(context_error, blocked_by_pending_context);
    if (blocked_by_pending_context) {
      return {503, "application/json", no_fresh_map_robot_pose_json(config_.map_frame, config_.base_frame, context_error)};
    }
    if (current_context &&
      (current_context->map_id != manifest->map_id ||
      current_context->building_id != resolved_building_id ||
      current_context->floor_id != resolved_floor_id))
    {
      return {
        503,
        "application/json",
        no_fresh_map_robot_pose_json(
          config_.map_frame,
          config_.base_frame,
          "requested pose target does not match confirmed runtime map context: " +
          current_context->building_id + "/" + current_context->floor_id + "/" +
          current_context->map_id)
      };
    }

    StoredPose pose;
    pose.id = pose_id;
    pose.name = pose_name.empty() ? pose_id : pose_name;
    pose.type = pose_type.empty() ? "delivery_point" : pose_type;
    pose.x = current_pose.x;
    pose.y = current_pose.y;
    pose.yaw = current_pose.yaw;

    const auto path = pose_target_path(manifest, resolved_building_id, resolved_floor_id);
    try {
      auto poses = read_floor_poses(path);
      bool updated = false;
      for (auto & existing : poses) {
        if (existing.id == pose.id) {
          if (is_elevator_internal_pose_type(existing.type) ||
            is_reserved_elevator_pose_id(existing.id))
          {
            return {
              400, "application/json",
              error_json(
                "elevator internal poses are managed only by the elevator configuration module")};
          }
          existing = pose;
          updated = true;
          break;
        }
      }
      if (!updated) {
        poses.push_back(pose);
      }
      if (const auto blocked = ports_.floor_runtime_interlock_response("current_pose_save_commit")) {
        return *blocked;
      }
      write_floor_poses(path, poses);
      sync_active_poses_if_needed(manifest, resolved_building_id, resolved_floor_id, path);

      std::ostringstream response;
      response << std::fixed << std::setprecision(6)
               << "{\"ok\":true,"
               << "\"source\":\"current_robot_pose\","
               << "\"building_id\":" << json_string(resolved_building_id) << ","
               << "\"floor_id\":" << json_string(resolved_floor_id) << ","
               << "\"map_id\":" << json_string(manifest ? manifest->map_id : "") << ","
               << "\"pose_id\":" << json_string(pose.id) << ","
               << "\"updated\":" << (updated ? "true" : "false") << ","
               << "\"poses_yaml\":" << json_string(path.string()) << ","
               << "\"source_pose\":{"
               << "\"frame_id\":" << json_string(current_pose.frame_id) << ","
               << "\"child_frame_id\":" << json_string(current_pose.child_frame_id) << ","
               << "\"x\":" << current_pose.x << ","
               << "\"y\":" << current_pose.y << ","
               << "\"yaw\":" << current_pose.yaw << ","
               << "\"stamp\":" << current_pose.stamp_sec << ","
               << "\"age_sec\":" << current_pose.age_sec << "},"
               << "\"pose\":{"
               << "\"id\":" << json_string(pose.id) << ","
               << "\"name\":" << json_string(pose.name) << ","
               << "\"type\":" << json_string(pose.type) << ","
               << "\"x\":" << pose.x << ","
               << "\"y\":" << pose.y << ","
               << "\"yaw\":" << pose.yaw << "}}";
      return {200, "application/json", response.str()};
    } catch (const std::exception & exc) {
      return {500, "application/json", error_json(exc.what())};
    }
  }

  std::optional<std::string> request_value(const HttpRequest & request, const std::string & key) const
  {
    const auto query = query_value(request, key);
    if (query) {
      return query;
    }
    return json_string_value(request.body, key);
  }

  bool resolve_pose_target_manifest(
    const std::optional<std::string> & requested_building_id,
    const std::optional<std::string> & requested_floor_id,
    const std::optional<std::string> & map_id,
    std::optional<MapManifest> & manifest,
    std::string & resolved_building_id,
    std::string & resolved_floor_id,
    HttpResponse & error)
  {
    if (requested_building_id && !safe_asset_id(*requested_building_id)) {
      error = {400, "application/json", error_json("valid building_id is required")};
      return false;
    }
    if (requested_floor_id && !safe_asset_id(*requested_floor_id)) {
      error = {400, "application/json", error_json("valid floor_id is required")};
      return false;
    }
    if (map_id && !map_id->empty() && !safe_asset_id(*map_id)) {
      error = {400, "application/json", error_json("valid map_id is required")};
      return false;
    }

    if (map_id && !map_id->empty()) {
      manifest = map_catalog_.find_map_by_id(*map_id);
      if (!manifest) {
        error = {404, "application/json", error_json("map_id not found: " + *map_id)};
        return false;
      }
      if (requested_building_id && manifest->building_id != *requested_building_id) {
        error = {400, "application/json", error_json("map_id does not belong to requested building")};
        return false;
      }
      if (requested_floor_id && manifest->floor_id != *requested_floor_id) {
        error = {400, "application/json", error_json("map_id does not belong to requested floor")};
        return false;
      }
      resolved_building_id = manifest->building_id;
      resolved_floor_id = manifest->floor_id;
      return true;
    }

    if (!requested_building_id) {
      error = {400, "application/json", error_json("valid building_id is required")};
      return false;
    }
    if (!requested_floor_id) {
      error = {400, "application/json", error_json("valid floor_id is required")};
      return false;
    }
    resolved_building_id = *requested_building_id;
    resolved_floor_id = *requested_floor_id;
    manifest = map_catalog_.active_floor_map(resolved_building_id, resolved_floor_id);
    return true;
  }

  fs::path pose_target_path(
    const std::optional<MapManifest> & manifest,
    const std::string & building_id,
    const std::string & floor_id) const
  {
    return manifest ? manifest->poses_yaml : poses_yaml_path(map_catalog_, building_id, floor_id);
  }

  void sync_active_poses_if_needed(
    const std::optional<MapManifest> & manifest,
    const std::string & building_id,
    const std::string & floor_id,
    const fs::path & path) const
  {
    if (!manifest || !manifest->active) {
      return;
    }
    copy_file_if_exists(path, map_catalog_.floor_current_root_path(building_id, floor_id) / "poses.yaml");
    copy_file_if_exists(path, map_catalog_.floor_root_path(building_id, floor_id) / "poses.yaml");
  }

  std::optional<StoredPose> parse_pose_payload(
    const std::string & payload,
    const std::optional<std::string> & forced_pose_id,
    std::string & error) const
  {
    const auto body_pose_id = json_string_value(payload, "pose_id").value_or(
      json_string_value(payload, "id").value_or(""));
    const auto pose_id = forced_pose_id.value_or(body_pose_id);
    if (!safe_pose_id(pose_id)) {
      error = "valid pose_id is required";
      return std::nullopt;
    }
    if (forced_pose_id && !body_pose_id.empty() && body_pose_id != *forced_pose_id) {
      error = "pose_id in body does not match path pose_id";
      return std::nullopt;
    }

    const auto x = json_number_value(payload, "x").value_or(
      json_nested_number_value(payload, "pose", "x").value_or(std::numeric_limits<double>::quiet_NaN()));
    const auto y = json_number_value(payload, "y").value_or(
      json_nested_number_value(payload, "pose", "y").value_or(std::numeric_limits<double>::quiet_NaN()));
    const auto yaw = json_number_value(payload, "yaw").value_or(
      json_number_value(payload, "theta").value_or(
        json_nested_number_value(payload, "pose", "yaw").value_or(std::numeric_limits<double>::quiet_NaN())));
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(yaw)) {
      error = "finite x, y, and yaw are required";
      return std::nullopt;
    }

    StoredPose pose;
    pose.id = pose_id;
    pose.name = json_string_value(payload, "name").value_or(pose_id);
    pose.type = json_string_value(payload, "type").value_or("delivery_point");
    if (is_elevator_internal_pose_type(pose.type) ||
      is_reserved_elevator_pose_id(pose.id))
    {
      error =
        "elevator internal poses are reserved for the elevator configuration module";
      return std::nullopt;
    }
    pose.x = x;
    pose.y = y;
    pose.yaw = normalize_angle(yaw);
    return pose;
  }

  HttpResponse handle_delete_pose(const HttpRequest & request, const std::string & pose_id)
  {
    if (const auto blocked = ports_.floor_runtime_interlock_response("pose_delete")) {
      return *blocked;
    }
    if (!safe_pose_id(pose_id)) {
      return {400, "application/json", error_json("valid pose_id is required")};
    }

    std::optional<MapManifest> manifest;
    std::string resolved_building_id;
    std::string resolved_floor_id;
    HttpResponse error;
    if (!resolve_pose_target_manifest(
        request_value(request, "building_id"),
        request_value(request, "floor_id"),
        request_value(request, "map_id"),
        manifest,
        resolved_building_id,
        resolved_floor_id,
        error))
    {
      return error;
    }

    const auto floor_root = map_catalog_.floor_root_path(resolved_building_id, resolved_floor_id);
    if (!fs::exists(floor_root) || !fs::is_directory(floor_root)) {
      return {404, "application/json", error_json("floor asset does not exist: " + floor_root.string())};
    }

    const auto path = pose_target_path(manifest, resolved_building_id, resolved_floor_id);
    try {
      auto poses = read_floor_poses(path);
      const auto requested = std::find_if(
        poses.begin(), poses.end(), [&pose_id](const StoredPose & pose) {
          return pose.id == pose_id;
        });
      if (requested != poses.end() &&
        (is_elevator_internal_pose_type(requested->type) ||
        is_reserved_elevator_pose_id(requested->id)))
      {
        return {
          400, "application/json",
          error_json(
            "elevator internal poses are managed only by the elevator configuration module")};
      }
      const auto before = poses.size();
      poses.erase(
        std::remove_if(poses.begin(), poses.end(), [&pose_id](const StoredPose & pose) {
          return pose.id == pose_id;
        }),
        poses.end());
      if (poses.size() == before) {
        return {404, "application/json", error_json("pose_id not found in poses.yaml: " + pose_id)};
      }
      if (const auto blocked = ports_.floor_runtime_interlock_response("pose_delete_commit")) {
        return *blocked;
      }
      write_floor_poses(path, poses);
      sync_active_poses_if_needed(manifest, resolved_building_id, resolved_floor_id, path);

      std::ostringstream response;
      response << "{\"ok\":true,"
               << "\"deleted\":true,"
               << "\"building_id\":" << json_string(resolved_building_id) << ","
               << "\"floor_id\":" << json_string(resolved_floor_id) << ","
               << "\"map_id\":" << json_string(manifest ? manifest->map_id : "") << ","
               << "\"pose_id\":" << json_string(pose_id) << ","
               << "\"remaining\":" << poses.size() << ","
               << "\"poses_yaml\":" << json_string(path.string()) << "}";
      return {200, "application/json", response.str()};
    } catch (const std::exception & exc) {
      return {500, "application/json", error_json(exc.what())};
    }
  }

  HttpResponse handle_replace_poses_batch(const std::string & body)
  {
    if (const auto blocked = ports_.floor_runtime_interlock_response("pose_batch_replace")) {
      return *blocked;
    }
    if (body.find("\"poses\"") == std::string::npos) {
      return {400, "application/json", error_json("poses array is required")};
    }

    std::optional<MapManifest> manifest;
    std::string resolved_building_id;
    std::string resolved_floor_id;
    HttpResponse error;
    if (!resolve_pose_target_manifest(
        json_string_value(body, "building_id"),
        json_string_value(body, "floor_id"),
        json_string_value(body, "map_id"),
        manifest,
        resolved_building_id,
        resolved_floor_id,
        error))
    {
      return error;
    }

    const auto floor_root = map_catalog_.floor_root_path(resolved_building_id, resolved_floor_id);
    if (!fs::exists(floor_root) || !fs::is_directory(floor_root)) {
      return {404, "application/json", error_json("floor asset does not exist: " + floor_root.string())};
    }

    std::vector<StoredPose> poses;
    std::set<std::string> pose_ids;
    for (const auto & object : json_object_array_value(body, "poses")) {
      std::string parse_error;
      auto pose = parse_pose_payload(object, std::nullopt, parse_error);
      if (!pose) {
        return {400, "application/json", error_json(parse_error)};
      }
      if (!pose_ids.insert(pose->id).second) {
        return {400, "application/json", error_json("duplicate pose_id in batch: " + pose->id)};
      }
      poses.push_back(*pose);
    }
    const auto requested_pose_count = poses.size();

    const auto path = pose_target_path(manifest, resolved_building_id, resolved_floor_id);
    try {
      const auto existing = read_floor_poses(path);
      for (const auto & preserved : existing) {
        if (!is_elevator_internal_pose_type(preserved.type) &&
          !is_reserved_elevator_pose_id(preserved.id))
        {
          continue;
        }
        if (pose_ids.count(preserved.id) > 0U) {
          return {
            400, "application/json",
            error_json(
              "batch replacement cannot overwrite an elevator internal pose")};
        }
        poses.push_back(preserved);
      }
      if (const auto blocked = ports_.floor_runtime_interlock_response("pose_batch_replace_commit")) {
        return *blocked;
      }
      write_floor_poses(path, poses);
      sync_active_poses_if_needed(manifest, resolved_building_id, resolved_floor_id, path);

      std::ostringstream response;
      response << "{\"ok\":true,"
               << "\"replaced\":true,"
               << "\"building_id\":" << json_string(resolved_building_id) << ","
               << "\"floor_id\":" << json_string(resolved_floor_id) << ","
               << "\"map_id\":" << json_string(manifest ? manifest->map_id : "") << ","
               << "\"count\":" << requested_pose_count << ","
               << "\"poses_yaml\":" << json_string(path.string()) << "}";
      return {200, "application/json", response.str()};
    } catch (const std::exception & exc) {
      return {500, "application/json", error_json(exc.what())};
    }
  }

  bool runtime_context_matches_map(const MapManifest & map, std::string & error) const
  {
    const auto context = read_runtime_map_context();
    if (!context) {
      error = "no runtime map context";
      return false;
    }
    if (!context->confirmed || context->state != "ready") {
      error =
        "runtime map context is not ready: " + context->building_id + "/" +
        context->floor_id + "/" + context->map_id + " state=" + context->state;
      return false;
    }
    if (context->building_id != map.building_id ||
      context->floor_id != map.floor_id ||
      context->map_id != map.map_id)
    {
      error =
        "runtime map changed: expected " + map.building_id + "/" + map.floor_id + "/" +
        map.map_id + " but found " + context->building_id + "/" + context->floor_id + "/" +
        context->map_id;
      return false;
    }
    return true;
  }

  bool lifecycle_service_is_active(
    const rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr & client,
    const std::string & service,
    std::string & error)
  {
    const auto timeout = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(config_.keepout_runtime_apply_timeout_sec));
    if (!client || !client->wait_for_service(timeout)) {
      error = "lifecycle service unavailable: " + service;
      return false;
    }
    auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();
    auto future = client->async_send_request(request);
    if (future.wait_for(timeout) != std::future_status::ready) {
      error = "timed out querying lifecycle service: " + service;
      return false;
    }
    const auto response = future.get();
    if (response->current_state.id != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
      error =
        service + " is not active: " + response->current_state.label +
        " [" + std::to_string(response->current_state.id) + "]";
      return false;
    }
    return true;
  }

  bool keepout_plugin_is_enabled(std::string & error)
  {
    const auto timeout = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(config_.keepout_runtime_apply_timeout_sec));
    if (!global_costmap_get_parameters_client_ ||
      !global_costmap_get_parameters_client_->wait_for_service(timeout))
    {
      error = "global costmap parameter service unavailable: " +
        config_.global_costmap_get_parameters_service;
      return false;
    }
    auto request = std::make_shared<rcl_interfaces::srv::GetParameters::Request>();
    request->names = {"filters", "keepout_filter.enabled"};
    auto future = global_costmap_get_parameters_client_->async_send_request(request);
    if (future.wait_for(timeout) != std::future_status::ready) {
      error = "timed out reading global costmap keepout parameters";
      return false;
    }
    const auto response = future.get();
    if (response->values.size() != 2U) {
      error = "global costmap returned incomplete keepout parameters";
      return false;
    }
    const auto & filters = response->values[0];
    const auto & enabled = response->values[1];
    if (filters.type != rcl_interfaces::msg::ParameterType::PARAMETER_STRING_ARRAY ||
      enabled.type != rcl_interfaces::msg::ParameterType::PARAMETER_BOOL)
    {
      error = "global costmap keepout parameters have unexpected types";
      return false;
    }
    const bool listed = std::find(
      filters.string_array_value.begin(),
      filters.string_array_value.end(),
      "keepout_filter") != filters.string_array_value.end();
    if (!listed || !enabled.bool_value) {
      error = "global costmap KeepoutFilter is not enabled";
      return false;
    }

    std::lock_guard<std::mutex> lock(keepout_observation_mutex_);
    if (!latest_keepout_filter_info_.valid) {
      error =
        "keepout filter info is missing or invalid; expected type=0 and mask_topic=" +
        config_.keepout_mask_topic;
      return false;
    }
    return true;
  }

  std::optional<KeepoutAssetPaths> resolve_keepout_runtime_projection(std::string & error)
  {
    const auto timeout = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(config_.keepout_runtime_apply_timeout_sec));
    if (!keepout_mask_get_parameters_client_ ||
      !keepout_mask_get_parameters_client_->wait_for_service(timeout))
    {
      error =
        "keepout mask parameter service unavailable: " + config_.keepout_mask_get_parameters_service;
      return std::nullopt;
    }

    auto request = std::make_shared<rcl_interfaces::srv::GetParameters::Request>();
    request->names = {"yaml_filename"};
    auto future = keepout_mask_get_parameters_client_->async_send_request(request);
    if (future.wait_for(timeout) != std::future_status::ready) {
      error = "timed out reading keepout mask yaml_filename";
      return std::nullopt;
    }
    const auto response = future.get();
    if (response->values.size() != 1U ||
      response->values.front().type !=
      rcl_interfaces::msg::ParameterType::PARAMETER_STRING ||
      response->values.front().string_value.empty())
    {
      error = "keepout mask server returned an invalid yaml_filename";
      return std::nullopt;
    }

    std::error_code root_error;
    std::error_code yaml_error;
    const auto stage_root = fs::weakly_canonical(
      fs::path(config_.keepout_runtime_stage_root), root_error);
    const auto stage_yaml = fs::weakly_canonical(
      fs::path(response->values.front().string_value), yaml_error);
    if (root_error || yaml_error || !fs::exists(stage_yaml) || !fs::is_regular_file(stage_yaml)) {
      error =
        "keepout runtime yaml_filename cannot be resolved: " +
        response->values.front().string_value;
      return std::nullopt;
    }
    const auto relative = stage_yaml.lexically_relative(stage_root);
    const auto relative_text = relative.generic_string();
    if (relative.empty() || relative.is_absolute() ||
      relative_text == ".." || relative_text.rfind("../", 0U) == 0U)
    {
      error =
        "keepout runtime yaml_filename is outside the managed staging root: " +
        stage_yaml.string();
      return std::nullopt;
    }

    return KeepoutAssetPaths{
      stage_yaml.parent_path() / "keepout_semantic_layer.json",
      stage_yaml,
      stage_yaml.parent_path() / "keepout_mask.pgm"};
  }

  bool keepout_mask_observation_matches(
    const KeepoutMaskObservation & observation,
    const KeepoutMaskSummary & expected) const
  {
    constexpr double geometry_tolerance = 1e-6;
    return observation.width == expected.width &&
           observation.height == expected.height &&
           std::fabs(observation.resolution - expected.resolution) <= geometry_tolerance &&
           std::fabs(observation.origin_x - expected.origin_x) <= geometry_tolerance &&
           std::fabs(observation.origin_y - expected.origin_y) <= geometry_tolerance &&
           std::fabs(normalize_angle(observation.origin_yaw - expected.origin_yaw)) <=
           geometry_tolerance &&
           observation.active_cells == expected.active_cells &&
           observation.occupancy_digest == expected.occupancy_digest;
  }

  bool global_costmap_matches_keepout_samples(
    const nav_msgs::msg::OccupancyGrid & costmap,
    const KeepoutMaskSummary & expected,
    std::size_t & checked,
    std::size_t & blocked,
    std::size_t & cleared,
    std::string & error) const
  {
    checked = 0U;
    blocked = 0U;
    cleared = 0U;
    if (normalized_frame_id(costmap.header.frame_id) != "map") {
      error =
        "global costmap frame is not map: " + normalized_frame_id(costmap.header.frame_id);
      return false;
    }
    if (costmap.info.width == 0U || costmap.info.height == 0U ||
      !std::isfinite(costmap.info.resolution) || costmap.info.resolution <= 0.0 ||
      costmap.data.size() !=
      static_cast<std::size_t>(costmap.info.width) *
      static_cast<std::size_t>(costmap.info.height))
    {
      error = "global costmap has invalid geometry or data length";
      return false;
    }

    const double origin_yaw = quaternion_yaw(
      costmap.info.origin.orientation.x,
      costmap.info.origin.orientation.y,
      costmap.info.origin.orientation.z,
      costmap.info.origin.orientation.w);
    const double cosine = std::cos(origin_yaw);
    const double sine = std::sin(origin_yaw);
    const auto cost_at = [&](const robot_api_server::KeepoutCostmapSample & sample)
      -> std::optional<int>
      {
        const double dx = sample.map_x - costmap.info.origin.position.x;
        const double dy = sample.map_y - costmap.info.origin.position.y;
        const double local_x = cosine * dx + sine * dy;
        const double local_y = -sine * dx + cosine * dy;
        const auto column = static_cast<long long>(
          std::floor(local_x / static_cast<double>(costmap.info.resolution)));
        const auto row = static_cast<long long>(
          std::floor(local_y / static_cast<double>(costmap.info.resolution)));
        if (column < 0 || row < 0 ||
          column >= static_cast<long long>(costmap.info.width) ||
          row >= static_cast<long long>(costmap.info.height))
        {
          std::ostringstream detail;
          detail << "keepout verification sample is outside the global costmap: map=("
                 << sample.map_x << "," << sample.map_y << ")";
          error = detail.str();
          return std::nullopt;
        }
        const auto index =
          static_cast<std::size_t>(row) * static_cast<std::size_t>(costmap.info.width) +
          static_cast<std::size_t>(column);
        return static_cast<int>(costmap.data[index]);
      };

    for (const auto & sample : expected.blocked_costmap_samples) {
      const auto value = cost_at(sample);
      if (!value) {
        return false;
      }
      ++checked;
      // OccupancyGrid value 100 is lethal. Value 99 can be the static
      // inflation/inscribed band and must not be accepted as keepout proof.
      if (*value == 100) {
        ++blocked;
        continue;
      }
      std::ostringstream detail;
      detail << "global costmap does not mark keepout sample lethal: map=("
             << sample.map_x << "," << sample.map_y << ") value=" << *value
             << " checked=" << checked << " blocked=" << blocked;
      error = detail.str();
      return false;
    }
    for (const auto & sample : expected.cleared_costmap_samples) {
      const auto value = cost_at(sample);
      if (!value) {
        return false;
      }
      ++checked;
      if (*value >= 0 && *value < 100) {
        ++cleared;
        continue;
      }
      std::ostringstream detail;
      detail << "global costmap still marks a removed keepout sample lethal or unknown: map=("
             << sample.map_x << "," << sample.map_y << ") value=" << *value
             << " checked=" << checked << " cleared=" << cleared;
      error = detail.str();
      return false;
    }
    return true;
  }

  bool apply_keepout_runtime(
    const MapManifest & map,
    const fs::path & mask_yaml,
    const KeepoutMaskSummary & expected,
    KeepoutRuntimeProof & proof,
    std::string & error)
  {
    if (!runtime_context_matches_map(map, error)) {
      return false;
    }
    if (!lifecycle_service_is_active(
        keepout_mask_state_client_, config_.keepout_mask_state_service, error))
    {
      return false;
    }
    proof.mask_server_active = true;
    if (!lifecycle_service_is_active(
        keepout_filter_info_state_client_, config_.keepout_filter_info_state_service, error))
    {
      return false;
    }

    if (!lifecycle_service_is_active(
        global_costmap_lifecycle_client_,
        config_.global_costmap_state_service,
        error))
    {
      return false;
    }
    if (!keepout_plugin_is_enabled(error)) {
      return false;
    }
    proof.filter_plugin_enabled = true;

    std::uint64_t baseline_sequence = 0U;
    {
      std::lock_guard<std::mutex> lock(keepout_observation_mutex_);
      baseline_sequence = latest_keepout_mask_.sequence;
    }

    const auto timeout = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(config_.keepout_runtime_apply_timeout_sec));
    if (!keepout_mask_load_client_ || !keepout_mask_load_client_->wait_for_service(timeout)) {
      error = "keepout mask load service unavailable: " + config_.keepout_mask_load_service;
      return false;
    }
    auto load_request = std::make_shared<nav2_msgs::srv::LoadMap::Request>();
    load_request->map_url = mask_yaml.string();
    proof.mutation_state = KeepoutRuntimeMutationState::MAY_HAVE_CHANGED;
    DelayedSideEffectEvidence load_pending_side_effect(ports_);
    auto load_future = keepout_mask_load_client_->async_send_request(load_request);
    if (load_future.wait_for(timeout) != std::future_status::ready) {
      error = "timed out loading keepout mask: " + mask_yaml.string();
      return false;
    }
    const auto load_response = load_future.get();
    load_pending_side_effect.resolve();
    if (load_response->result != nav2_msgs::srv::LoadMap::Response::RESULT_SUCCESS) {
      error =
        config_.keepout_mask_load_service + " rejected keepout mask with result code " +
        std::to_string(load_response->result);
      return false;
    }
    proof.load_map_succeeded = true;

    {
      std::unique_lock<std::mutex> lock(keepout_observation_mutex_);
      const auto deadline = std::chrono::steady_clock::now() +
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(config_.keepout_runtime_apply_timeout_sec));
      const bool matched = keepout_observation_cv_.wait_until(
        lock,
        deadline,
        [&]() {
          return latest_keepout_mask_.sequence > baseline_sequence &&
                 keepout_mask_observation_matches(latest_keepout_mask_, expected);
        });
      if (!matched) {
        std::ostringstream detail;
        detail << "keepout mask topic did not publish the requested artifact"
               << " expected=" << expected.width << "x" << expected.height
               << " active=" << expected.active_cells
               << " digest=" << expected.occupancy_digest
               << " observed_seq=" << latest_keepout_mask_.sequence
               << " baseline_seq=" << baseline_sequence
               << " observed=" << latest_keepout_mask_.width << "x"
               << latest_keepout_mask_.height
               << " active=" << latest_keepout_mask_.active_cells
               << " digest=" << latest_keepout_mask_.occupancy_digest;
        error = detail.str();
        return false;
      }
    }
    proof.mask_topic_matches = true;

    auto costmap_probe = std::make_shared<KeepoutCostmapProbeState>();
    auto costmap_probe_subscription = node_.create_subscription<nav_msgs::msg::OccupancyGrid>(
      config_.global_costmap_topic,
      rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(),
      [costmap_probe](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        {
          std::lock_guard<std::mutex> lock(costmap_probe->mutex);
          ++costmap_probe->sequence;
          costmap_probe->latest = msg;
          costmap_probe->received_at = std::chrono::steady_clock::now();
        }
        costmap_probe->condition.notify_all();
      });
    {
      std::unique_lock<std::mutex> lock(costmap_probe->mutex);
      const bool observed = costmap_probe->condition.wait_for(
        lock,
        timeout,
        [&]() {return costmap_probe->sequence > 0U;});
      if (!observed) {
        error = "global costmap topic unavailable before keepout clear: " + config_.global_costmap_topic;
        return false;
      }
    }

    if (!global_costmap_clear_client_ ||
      !global_costmap_clear_client_->wait_for_service(timeout))
    {
      error = "global costmap clear service unavailable: " + config_.global_costmap_clear_service;
      return false;
    }
    auto clear_request = std::make_shared<nav2_msgs::srv::ClearEntireCostmap::Request>();
    DelayedSideEffectEvidence clear_pending_side_effect(ports_);
    auto clear_future = global_costmap_clear_client_->async_send_request(clear_request);
    if (clear_future.wait_for(timeout) != std::future_status::ready) {
      error = "timed out clearing global costmap after keepout reload";
      return false;
    }
    (void)clear_future.get();
    clear_pending_side_effect.resolve();
    const auto clear_completed_steady = std::chrono::steady_clock::now();
    const auto clear_completed_ros_ns = node_.now().nanoseconds();
    std::uint64_t costmap_baseline = 0U;
    {
      std::lock_guard<std::mutex> lock(costmap_probe->mutex);
      costmap_baseline = costmap_probe->sequence;
    }
    proof.global_costmap_cleared = true;

    nav_msgs::msg::OccupancyGrid::SharedPtr verified_costmap;
    {
      std::unique_lock<std::mutex> lock(costmap_probe->mutex);
      const bool updated = costmap_probe->condition.wait_for(
        lock,
        timeout,
        [&]() {
          if (costmap_probe->sequence <= costmap_baseline ||
            !costmap_probe->latest ||
            costmap_probe->received_at <= clear_completed_steady)
          {
            return false;
          }
          return rclcpp::Time(costmap_probe->latest->header.stamp).nanoseconds() >
                 clear_completed_ros_ns;
        });
      if (!updated) {
        error =
          "global costmap did not publish a fresh full map after keepout reload and clear: " +
          config_.global_costmap_topic;
        return false;
      }
      verified_costmap = costmap_probe->latest;
    }
    proof.global_costmap_updated = true;
    costmap_probe_subscription.reset();

    if (!verified_costmap ||
      !global_costmap_matches_keepout_samples(
        *verified_costmap,
        expected,
        proof.costmap_samples_checked,
        proof.costmap_samples_blocked,
        proof.costmap_samples_cleared,
        error))
    {
      return false;
    }
    proof.global_costmap_content_matches = true;

    if (!runtime_context_matches_map(map, error)) {
      return false;
    }
    std::ostringstream detail;
    detail << "mask_server=active; filter_info_server=active; plugin=enabled"
           << "; load_map=success; mask_topic=matched"
           << "; global_costmap=cleared_updated_and_sampled"
           << "; samples=" << proof.costmap_samples_blocked << "/"
           << expected.blocked_costmap_samples.size()
           << " blocked, " << proof.costmap_samples_cleared << "/"
           << expected.cleared_costmap_samples.size() << " cleared"
           << ", " << proof.costmap_samples_checked << " total"
           << "; active_cells=" << expected.active_cells
           << "; digest=" << expected.occupancy_digest;
    proof.detail = detail.str();
    return true;
  }

  class RosKeepoutRuntimeAdapter final : public KeepoutRuntimePort
  {
  public:
    RosKeepoutRuntimeAdapter(Impl & owner, MapManifest map)
    : owner_(owner), map_(std::move(map))
    {
    }

    bool apply(
      const fs::path & mask_yaml,
      const KeepoutMaskSummary & expected,
      KeepoutRuntimeProof & proof,
      std::string & error) override
    {
      return owner_.apply_keepout_runtime(map_, mask_yaml, expected, proof, error);
    }

    bool restore(
      const fs::path & mask_yaml,
      const KeepoutMaskSummary & expected,
      KeepoutRuntimeProof & proof,
      std::string & error) override
    {
      return owner_.apply_keepout_runtime(map_, mask_yaml, expected, proof, error);
    }

  private:
    Impl & owner_;
    MapManifest map_;
  };

  HttpResponse handle_save_keepout_filter(
    const std::string & body,
    const ElevatorMotionAdmissionFence::Epoch motion_admission_epoch)
  {
    if (const auto blocked = ports_.floor_runtime_interlock_response("keepout_update")) {
      return *blocked;
    }
    const auto stripped_body = trim(body);
    if (stripped_body.empty() || stripped_body.front() != '{') {
      return {400, "application/json", error_json("JSON request body is required")};
    }

    const auto requested_building_id = json_string_value(body, "building_id");
    const auto requested_floor_id = json_string_value(body, "floor_id");
    const auto map_id = json_string_value(body, "map_id");
    if (requested_building_id && !safe_asset_id(*requested_building_id)) {
      return {400, "application/json", error_json("valid building_id is required")};
    }
    if (requested_floor_id && !safe_asset_id(*requested_floor_id)) {
      return {400, "application/json", error_json("valid floor_id is required")};
    }
    if (map_id && !map_id->empty() && !safe_asset_id(*map_id)) {
      return {400, "application/json", error_json("valid map_id is required")};
    }

    std::lock_guard<std::mutex> update_lock(asset_mutation_mutex_);
    std::lock_guard<std::mutex> elevator_lock(cross_asset_commit_mutex_);
    std::optional<MapManifest> manifest;
    std::string resolved_building_id;
    std::string resolved_floor_id;
    if (map_id && !map_id->empty()) {
      manifest = map_catalog_.find_map_by_id(*map_id);
      if (!manifest) {
        return {404, "application/json", error_json("map_id not found: " + *map_id)};
      }
      if (requested_building_id && manifest->building_id != *requested_building_id) {
        return {400, "application/json", error_json("map_id does not belong to requested building")};
      }
      if (requested_floor_id && manifest->floor_id != *requested_floor_id) {
        return {400, "application/json", error_json("map_id does not belong to requested floor")};
      }
      resolved_building_id = manifest->building_id;
      resolved_floor_id = manifest->floor_id;
    } else {
      if (!requested_building_id) {
        return {400, "application/json", error_json("valid building_id is required")};
      }
      if (!requested_floor_id) {
        return {400, "application/json", error_json("valid floor_id is required")};
      }
      resolved_building_id = *requested_building_id;
      resolved_floor_id = *requested_floor_id;
      manifest = map_catalog_.active_floor_map(resolved_building_id, resolved_floor_id);
      if (!manifest) {
        return {
          404,
          "application/json",
          error_json("active map not found for floor: " + resolved_building_id + "/" + resolved_floor_id)
        };
      }
    }

    const bool reload_filter = json_bool_value(body, "reload_filter", true);
    const auto busy_response = [](const std::string & code, const std::string & message) {
        std::ostringstream response;
        response << "{\"ok\":false,\"persisted\":false,\"effective\":false,"
                 << "\"error\":{\"code\":" << json_string(code)
                 << ",\"stage\":\"precondition\",\"retryable\":true,\"message\":"
                 << json_string(message) << "}}";
        return HttpResponse{409, "application/json", response.str()};
    };

    const auto elevator_reference = ports_.query_elevator_reference(*manifest);
    if (elevator_reference.status >= 400) {
      return elevator_reference;
    }
    if (json_bool_value(elevator_reference.body, "referenced", false)) {
      const auto release_id =
        json_string_value(elevator_reference.body, "release_id");
      std::ostringstream response;
      response
        << "{\"ok\":false,\"persisted\":false,\"effective\":false,"
        << "\"error\":{\"code\":\"ELEVATOR_CONFIG_MAP_IN_USE\","
        << "\"stage\":\"precondition\",\"retryable\":false,"
        << "\"message\":\"published elevator configuration binds this immutable "
           "map bundle; publish a new map/configuration release before editing "
           "keepout assets\"},"
        << "\"building_id\":" << json_string(manifest->building_id) << ","
        << "\"floor_id\":" << json_string(manifest->floor_id) << ","
        << "\"map_id\":" << json_string(manifest->map_id) << ","
        << "\"release_id\":"
        << (release_id ? json_string(*release_id) : std::string("null"))
        << "}";
      return {409, "application/json", response.str()};
    }
    const auto runtime = ports_.runtime_snapshot(true);
    if (runtime.mapping_active || runtime.mapping_start_job_running) {
      return busy_response(
        "RUNTIME_BUSY", "keepout editing is blocked while mapping is active or starting");
    }
    if (runtime.docking_active) {
      return busy_response("RUNTIME_BUSY", "keepout editing is blocked while docking is active");
    }
    if (runtime.navigation_goal_running) {
      return busy_response(
        "NAVIGATION_BUSY", "keepout editing is blocked while a navigation goal is running");
    }
    if (runtime.nav2_goal_active) {
      return busy_response(
        "NAVIGATION_BUSY",
        "keepout editing is blocked while Nav2 reports an active action goal");
    }

    bool runtime_selected = false;
    const auto runtime_context = read_runtime_map_context();
    if (runtime.navigation_active) {
      if (!runtime_context || !runtime_context->confirmed || runtime_context->state != "ready") {
        return busy_response(
          "ACTIVE_MAP_UNCONFIRMED",
          "navigation is active but its exact runtime map context is not confirmed");
      }
      runtime_selected =
        runtime_context->building_id == manifest->building_id &&
        runtime_context->floor_id == manifest->floor_id &&
        runtime_context->map_id == manifest->map_id;
    }
    if (runtime_selected && !manifest->active) {
      return busy_response(
        "ACTIVE_MAP_CHANGED",
        "the runtime map is no longer the active manifest for its floor");
    }
    if (runtime_selected && !reload_filter) {
      return busy_response(
        "RUNTIME_APPLY_REQUIRED",
        "the selected map is loaded by navigation; reload_filter must remain enabled");
    }
    if (runtime_selected) {
      std::string stop_detail;
      if (!ports_.wait_for_terminal_actual_stop(
          "keepout update requires stable wheel odometry", stop_detail))
      {
        return busy_response(
          "ROBOT_NOT_STATIONARY",
          "keepout editing is blocked until the robot is stably stopped: " + stop_detail);
      }
      if (ports_.runtime_snapshot(false).nav2_goal_active) {
        return busy_response(
          "NAVIGATION_BUSY",
          "Nav2 accepted a goal while keepout preconditions were being checked");
      }
    }

    MapAssetCommitTransaction asset_transaction{fs::path(config_.maps_root)};
    if (map_activation_journal_blocks_mutation()) {
      return {
        503,
        "application/json",
        error_json(
          "a map activation transaction is pending; keepout repair is blocked")};
    }
    const auto persisted_integrity_marker =
      persisted_map_asset_integrity_marker();
    if (keepout_integrity_degraded_.load() &&
      !persisted_integrity_marker)
    {
      return {
        503,
        "application/json",
        error_json(
          "map asset integrity is latched without repairable persistent "
          "evidence; keepout self-repair is refused")};
    }
    const auto commit_manifest =
      map_catalog_.find_map_by_id(manifest->map_id);
    if (!commit_manifest ||
      commit_manifest->building_id != manifest->building_id ||
      commit_manifest->floor_id != manifest->floor_id ||
      commit_manifest->map_id != manifest->map_id)
    {
      return busy_response(
        "MAP_ASSET_CHANGED",
        "map identity changed before the keepout commit transaction");
    }
    manifest = *commit_manifest;
    resolved_building_id = manifest->building_id;
    resolved_floor_id = manifest->floor_id;
    if (runtime_selected && !manifest->active) {
      return busy_response(
        "ACTIVE_MAP_CHANGED",
        "the map became inactive before the keepout commit transaction");
    }

    std::string expected_non_keepout_digest;
    try {
      if (persisted_integrity_marker) {
        const auto & persisted = *persisted_integrity_marker;
        if (!persisted.repairable_for(manifest->map_id)) {
          return {
            503,
            "application/json",
            error_json(
              "a different or unprovable map asset integrity latch is active; "
              "this keepout update cannot clear it")};
        }
        const auto inspected =
          inspect_map_asset_identity_for_keepout_repair(
          *manifest, fs::path(config_.maps_root), asset_transaction);
        if (inspected.non_keepout_asset_digest !=
          persisted.non_keepout_asset_digest)
        {
          return {
            503,
            "application/json",
            error_json(
              "non-keepout map assets changed while the keepout transaction "
              "was degraded; automatic repair is refused")};
        }
        expected_non_keepout_digest =
          persisted.non_keepout_asset_digest;
      } else {
        const auto verified = verify_map_asset_identity_snapshot(
          *manifest, fs::path(config_.maps_root), asset_transaction);
        expected_non_keepout_digest =
          verified.non_keepout_asset_digest;
      }
    } catch (const std::exception & error) {
      return {
        503,
        "application/json",
        error_json(
          std::string("map identity preflight failed before keepout update: ") +
          error.what())};
    }

    ReplaceKeepoutCommand command;
    command.request_json = stripped_body;
    command.expected_revision = json_string_value(body, "expected_revision");
    if (command.expected_revision && command.expected_revision->empty()) {
      return {
        400,
        "application/json",
        error_json("expected_revision must be a non-empty revision when provided")};
    }
    command.map = *manifest;
    command.runtime_selected = runtime_selected;
    command.runtime_apply_requested = runtime_selected && reload_filter;
    command.projections.push_back(
      KeepoutAssetPaths{
        keepout_semantic_json_path(*manifest),
        manifest->keepout_mask_yaml,
        manifest->keepout_mask_pgm});
    if (manifest->active) {
      const auto current_filters =
        map_catalog_.floor_current_root_path(resolved_building_id, resolved_floor_id) / "filters";
      command.projections.push_back(
        KeepoutAssetPaths{
          current_filters / "keepout_semantic_layer.json",
          current_filters / "keepout_mask.yaml",
          current_filters / "keepout_mask.pgm"});
      const auto floor_filters =
        map_catalog_.floor_root_path(resolved_building_id, resolved_floor_id) / "filters";
      command.projections.push_back(
        KeepoutAssetPaths{
          floor_filters / "keepout_semantic_layer.json",
          floor_filters / "keepout_mask.yaml",
          floor_filters / "keepout_mask.pgm"});
    }
    if (runtime_selected) {
      std::string staging_error;
      const auto runtime_projection = resolve_keepout_runtime_projection(staging_error);
      if (!runtime_projection) {
        return {
          503,
          "application/json",
          error_json("failed to resolve active keepout staging artifact: " + staging_error)};
      }
      const bool already_projected = std::any_of(
        command.projections.begin(),
        command.projections.end(),
        [&](const KeepoutAssetPaths & projection) {
          return projection.mask_yaml == runtime_projection->mask_yaml;
        });
      if (!already_projected) {
        command.projections.push_back(*runtime_projection);
      }
    }

    KeepoutLayerModule keepout_module;
    RosKeepoutRuntimeAdapter runtime_adapter(*this, *manifest);
    if (const auto blocked = ports_.floor_runtime_interlock_response("keepout_update_commit")) {
      return *blocked;
    }
    try {
      // Persist the fail-closed latch before the first payload write. A crash
      // anywhere after this point remains blocked across process restart until
      // the exact bundle identity and active projection are re-proved.
      persist_map_asset_integrity_degraded(
        manifest->map_id,
        "keepout update transaction in progress",
        expected_non_keepout_digest);
      const auto result = keepout_module.replace(
        command,
        command.runtime_apply_requested ? &runtime_adapter : nullptr);
      const auto changed_snapshot =
        inspect_map_asset_identity_for_keepout_repair(
        *manifest, fs::path(config_.maps_root), asset_transaction);
      if (changed_snapshot.non_keepout_asset_digest !=
        expected_non_keepout_digest)
      {
        throw std::runtime_error(
                "non-keepout map assets changed during keepout commit");
      }
      stamp_map_asset_identity(
        *manifest,
        fs::path(config_.maps_root),
        asset_transaction);
      const auto committed_snapshot = verify_map_asset_identity_snapshot(
        *manifest, fs::path(config_.maps_root), asset_transaction);
      if (committed_snapshot.non_keepout_asset_digest !=
        expected_non_keepout_digest)
      {
        throw std::runtime_error(
                "non-keepout map asset proof changed after keepout commit");
      }
      if (manifest->active) {
        const auto current_manifest =
          map_catalog_.floor_current_root_path(
          manifest->building_id, manifest->floor_id) / "manifest.json";
        durable_write_text_file_atomic(
          current_manifest, map_manifest_json(*manifest));
      }
      clear_map_asset_integrity_degraded();
      std::ostringstream response;
      response << std::fixed << std::setprecision(9)
               << "{\"ok\":true,"
               << "\"persisted\":" << (result.persisted ? "true" : "false") << ","
               << "\"changed\":" << (result.changed ? "true" : "false") << ","
               << "\"mask_updated\":" << (result.changed ? "true" : "false") << ","
               << "\"effective\":" << (result.runtime_effective ? "true" : "false") << ","
               << "\"runtime_effective\":"
               << (result.runtime_effective ? "true" : "false") << ","
               << "\"integrity_degraded\":"
               << (keepout_integrity_degraded_.load() ? "true" : "false") << ","
               << "\"runtime_selected\":" << (result.runtime_selected ? "true" : "false") << ","
               << "\"effective_on_next_activation\":"
               << (result.effective_on_next_activation ? "true" : "false") << ","
               << "\"outcome\":" << json_string(result.outcome) << ","
               << "\"revision\":" << json_string(result.revision) << ","
               << "\"previous_revision\":" << json_string(result.previous_revision) << ","
               << "\"building_id\":" << json_string(resolved_building_id) << ","
               << "\"floor_id\":" << json_string(resolved_floor_id) << ","
               << "\"map_id\":" << json_string(manifest->map_id) << ","
               << "\"asset_epoch\":" << manifest->asset_epoch << ","
               << "\"asset_digest\":" << json_string(manifest->asset_digest) << ","
               << "\"non_keepout_asset_digest\":"
               << json_string(expected_non_keepout_digest) << ","
               << "\"display_name\":" << json_string(manifest->display_name) << ","
               << "\"map_name\":" << json_string(manifest->display_name) << ","
               << "\"active\":" << (manifest->active ? "true" : "false") << ","
               << "\"semantic_json_path\":"
               << json_string(keepout_semantic_json_path(*manifest).string()) << ","
               << "\"keepout_mask_yaml\":" << json_string(manifest->keepout_mask_yaml.string())
               << ",\"keepout_mask_pgm\":" << json_string(manifest->keepout_mask_pgm.string())
               << ",\"mask\":{"
               << "\"width\":" << result.mask.width << ","
               << "\"height\":" << result.mask.height << ","
               << "\"resolution\":" << result.mask.resolution << ","
               << "\"origin\":[" << result.mask.origin_x << "," << result.mask.origin_y << ","
               << result.mask.origin_yaw << "],"
               << "\"active_cells\":" << result.mask.active_cells << ","
               << "\"occupancy_digest\":" << json_string(result.mask.occupancy_digest)
               << "},\"runtime\":{"
               << "\"mask_server_active\":"
               << (result.runtime_proof.mask_server_active ? "true" : "false") << ","
               << "\"filter_plugin_enabled\":"
               << (result.runtime_proof.filter_plugin_enabled ? "true" : "false") << ","
               << "\"load_map_succeeded\":"
               << (result.runtime_proof.load_map_succeeded ? "true" : "false") << ","
               << "\"mask_topic_matches\":"
               << (result.runtime_proof.mask_topic_matches ? "true" : "false") << ","
               << "\"global_costmap_cleared\":"
               << (result.runtime_proof.global_costmap_cleared ? "true" : "false") << ","
               << "\"global_costmap_updated\":"
               << (result.runtime_proof.global_costmap_updated ? "true" : "false") << ","
               << "\"global_costmap_content_matches\":"
               << (result.runtime_proof.global_costmap_content_matches ? "true" : "false") << ","
               << "\"costmap_samples_checked\":"
               << result.runtime_proof.costmap_samples_checked << ","
               << "\"costmap_samples_blocked\":"
               << result.runtime_proof.costmap_samples_blocked << ","
               << "\"costmap_samples_cleared\":"
               << result.runtime_proof.costmap_samples_cleared << ","
               << "\"detail\":" << json_string(result.runtime_proof.detail) << "}}";
      return {200, "application/json", response.str()};
    } catch (const KeepoutLayerError & error) {
      bool integrity_unknown =
        error.code() == "ARTIFACT_ROLLBACK_FAILED" ||
        error.code() == "RUNTIME_ROLLBACK_FAILED";
      if (!integrity_unknown) {
        try {
          // The keepout module reports that rollback completed. Re-bind the
          // exact rolled-back bytes before removing the persistent latch.
          const auto rolled_back_snapshot =
            inspect_map_asset_identity_for_keepout_repair(
            *manifest, fs::path(config_.maps_root), asset_transaction);
          if (rolled_back_snapshot.non_keepout_asset_digest !=
            expected_non_keepout_digest)
          {
            throw std::runtime_error(
                    "non-keepout map assets changed during keepout rollback");
          }
          stamp_map_asset_identity(
            *manifest,
            fs::path(config_.maps_root),
            asset_transaction);
          const auto committed_snapshot =
            verify_map_asset_identity_snapshot(
            *manifest, fs::path(config_.maps_root), asset_transaction);
          if (committed_snapshot.non_keepout_asset_digest !=
            expected_non_keepout_digest)
          {
            throw std::runtime_error(
                    "non-keepout map asset proof changed after rollback");
          }
          if (manifest->active) {
            const auto current_manifest =
              map_catalog_.floor_current_root_path(
              manifest->building_id, manifest->floor_id) / "manifest.json";
            durable_write_text_file_atomic(
              current_manifest, map_manifest_json(*manifest));
          }
          clear_map_asset_integrity_degraded();
        } catch (const std::exception &) {
          integrity_unknown = true;
        }
      }
      std::ostringstream response;
      response << "{\"ok\":false,\"persisted\":"
               << (integrity_unknown ? "null" : "false")
               << ",\"effective\":" << (integrity_unknown ? "null" : "false")
               << ",\"integrity_degraded\":"
               << (keepout_integrity_degraded_.load() ? "true" : "false") << ","
               << "\"error\":{\"code\":" << json_string(error.code())
               << ",\"stage\":\"keepout_replace\",\"retryable\":"
               << (error.http_status() >= 500 ? "true" : "false")
               << ",\"message\":" << json_string(error.what()) << "}}";
      return {error.http_status(), "application/json", response.str()};
    } catch (const std::exception & error) {
      return {500, "application/json", error_json(error.what())};
    }
  }


  rclcpp::Node & node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  std::mutex & cross_asset_commit_mutex_;
  MapsModuleConfig config_;
  MapsModulePorts ports_;
  MapCatalog map_catalog_;
  std::mutex asset_mutation_mutex_;
  std::atomic<bool> keepout_integrity_degraded_{false};
  std::mutex keepout_observation_mutex_;
  std::condition_variable keepout_observation_cv_;
  KeepoutMaskObservation latest_keepout_mask_;
  KeepoutFilterInfoObservation latest_keepout_filter_info_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr keepout_mask_sub_;
  rclcpp::Subscription<nav2_msgs::msg::CostmapFilterInfo>::SharedPtr
    keepout_filter_info_sub_;
  rclcpp::Client<nav2_msgs::srv::LoadMap>::SharedPtr keepout_mask_load_client_;
  rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr keepout_mask_state_client_;
  rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr
    keepout_filter_info_state_client_;
  rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedPtr
    keepout_mask_get_parameters_client_;
  rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedPtr
    global_costmap_get_parameters_client_;
  rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr
    global_costmap_lifecycle_client_;
  rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedPtr
    global_costmap_clear_client_;
};

MapsModule::MapsModule(
  rclcpp::Node & node,
  rclcpp::CallbackGroup::SharedPtr callback_group,
  std::mutex & cross_asset_commit_mutex,
  MapsModuleConfig config,
  MapsModulePorts ports)
: impl_(std::make_unique<Impl>(
    node,
    std::move(callback_group),
    cross_asset_commit_mutex,
    std::move(config),
    std::move(ports)))
{
}

MapsModule::~MapsModule() = default;

std::optional<HttpResponse> MapsModule::handle_http(
  const HttpRequest & request,
  const ElevatorMotionAdmissionFence::Epoch motion_admission_epoch)
{
  return impl_->handle_http(request, motion_admission_epoch);
}

MapCatalog & MapsModule::catalog()
{
  return impl_->catalog();
}

std::mutex & MapsModule::mutation_mutex()
{
  return impl_->mutation_mutex();
}

rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr
MapsModule::global_costmap_lifecycle_client()
{
  return impl_->global_costmap_lifecycle_client();
}

bool MapsModule::integrity_degraded()
{
  return impl_->map_asset_integrity_degraded();
}

bool MapsModule::validate_manifest_assets(
  const MapManifest & manifest,
  std::string & error) const
{
  return impl_->validate_map_manifest_assets(manifest, error);
}

bool MapsModule::validate_current_projection_assets(
  const MapManifest & source_manifest,
  const std::filesystem::path & current_root,
  std::string & error) const
{
  return impl_->validate_current_map_projection_assets(
    source_manifest, current_root, error);
}

bool MapsModule::runtime_context_matches(
  const MapManifest & manifest,
  std::string & error) const
{
  return impl_->runtime_context_matches_map(manifest, error);
}

void MapsModule::activate_manifest(
  MapManifest manifest,
  MapAssetCommitTransaction & transaction)
{
  impl_->activate_map_manifest(std::move(manifest), transaction);
}

std::optional<MapManifest> MapsModule::confirmed_runtime_manifest(
  std::string & unavailable_reason,
  bool & blocked_by_pending_context)
{
  return impl_->confirmed_runtime_map_manifest(
    unavailable_reason, blocked_by_pending_context);
}

}  // namespace robot_api_server::features::maps
