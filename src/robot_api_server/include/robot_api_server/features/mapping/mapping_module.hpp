#pragma once

#include <cstdint>
#include <filesystem>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "robot_api_server/application/runtime_mode/runtime_mode_coordinator.hpp"
#include "robot_api_server/features/elevator/execution/elevator_runtime_policy.hpp"
#include "robot_api_server/features/mapping/runtime/mapping_process_runtime.hpp"
#include "robot_api_server/features/maps/catalog_activation/map_catalog.hpp"
#include "robot_api_server/infrastructure/http/http_common.hpp"

namespace robot_api_server::features::mapping
{

struct MappingModuleConfig
{
  runtime::MappingProcessRuntimeConfig process;
  std::filesystem::path maps_root;
  std::filesystem::path runtime_maps_dir;
  std::string live_map_topic{"/map"};
  double live_map_max_age_sec{3.0};
  std::string scan_owner_topic{"/scan"};
  std::string navigation_scan_owner_node{"pointcloud_accel_axis_node"};
  std::string resident_scan_control_service{
    "/pointcloud_accel_axis_node/set_scan_output_enabled"};
  double scan_owner_restore_timeout_sec{30.0};
};

struct MappingNavigationActionResult
{
  bool ok{false};
  std::string detail;
};

struct MappingModulePorts
{
  std::function<bool(const std::string & operation, std::string & detail)>
  floor_runtime_operation_blocked;
  std::function<std::optional<HttpResponse>(const std::string & operation)>
  floor_runtime_interlock_response;
  std::function<bool()> map_asset_integrity_degraded;
  std::function<ElevatorMotionAdmissionFence::AdmissionGuard(
      ElevatorMotionAdmissionFence::Epoch expected_epoch)>
  acquire_motion_admission;
  std::function<bool()> navigation_goal_running;
  std::function<MappingNavigationActionResult()> cancel_navigation;
  std::function<MappingNavigationActionResult()> stop_navigation_runtime;
  std::function<void()> clear_runtime_map_context;
};

struct MappingModuleSnapshot
{
  bool process_active{false};
  bool process_running{false};
  bool start_job_running{false};
  bool live_map_available{false};
  double live_map_age_sec{-1.0};
  std::uint32_t live_map_width{0U};
  std::uint32_t live_map_height{0U};
  double live_map_resolution{0.0};
  double known_area_m2{0.0};
};

// Owns the complete 2D-mapping vertical slice. The composition root supplies
// only cross-domain ports; HTTP behavior, ROS live-map/cache ownership,
// transition jobs, process lifetime, asset persistence, and status formatting
// remain private implementation details behind this interface.
class MappingModule
{
public:
  MappingModule(
    rclcpp::Node & node,
    rclcpp::CallbackGroup::SharedPtr callback_group,
    application::runtime_mode::RuntimeModeCoordinator & runtime_mode,
    MapCatalog & map_catalog,
    std::mutex & map_asset_mutation_mutex,
    MappingModuleConfig config,
    MappingModulePorts ports);
  ~MappingModule();

  MappingModule(const MappingModule &) = delete;
  MappingModule & operator=(const MappingModule &) = delete;

  std::optional<HttpResponse> handle_http(
    const HttpRequest & request,
    ElevatorMotionAdmissionFence::Epoch motion_admission_epoch);
  std::string status_json();
  MappingModuleSnapshot snapshot(bool refresh_runtime = true);
  void set_live_map_page_active(bool active);
  void shutdown();

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace robot_api_server::features::mapping
