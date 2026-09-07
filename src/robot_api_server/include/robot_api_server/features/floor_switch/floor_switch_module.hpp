#pragma once

#include <chrono>
#include <filesystem>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "robot_api_server/features/elevator/execution/elevator_runtime_policy.hpp"
#include "robot_api_server/features/floor_switch/floor_runtime_interlock.hpp"
#include "robot_api_server/features/maps/catalog_activation/map_asset_identity_binding.hpp"
#include "robot_api_server/features/maps/catalog_activation/map_catalog.hpp"
#include "robot_api_server/infrastructure/http/http_common.hpp"

namespace robot_api_server::features::floor_switch
{

struct FloorSwitchModuleConfig
{
  std::filesystem::path maps_root;
  std::filesystem::path runtime_map_context_file;
  std::string legacy_service{"/floor_manager/switch_floor"};
  std::string live_action{"/floor_manager/floor_switch"};
  std::string transition_status_topic{"/floor_manager/transition_status"};
  std::string localization_health_topic{"/localization/floor_health"};
  bool negative_interlock_enabled{true};
  double live_timeout_sec{120.0};
  std::chrono::nanoseconds service_timeout{std::chrono::seconds(8)};
};

// Read-only observations needed to prove that the legacy offline selector is
// not racing a motion runtime. FloorSwitchModule owns the decision and response
// policy; the composition root only supplies facts from neighboring modules.
struct FloorSwitchRuntimeSnapshot
{
  bool navigation_active{false};
  bool mapping_active{false};
  bool docking_active{false};
  std::string mode_transition_owner;
  bool navigation_goal_running{false};
  bool mapping_start_job_running{false};
  bool docking_job_running{false};
  bool nav2_goal_active{false};
  bool navigation_process_running{false};
};

struct FloorSwitchModulePorts
{
  std::function<bool()> map_asset_integrity_degraded;
  std::function<FloorSwitchRuntimeSnapshot()> runtime_snapshot;
  std::function<bool(const MapManifest & manifest, std::string & error)>
  validate_map_manifest_assets;
  std::function<void(
      MapManifest manifest,
      MapAssetCommitTransaction & transaction)>
  activate_map_manifest;
  std::function<ElevatorMotionAdmissionFence::AdmissionGuard(
      ElevatorMotionAdmissionFence::Epoch expected_epoch)>
  acquire_motion_admission;
  std::function<void()> mark_delayed_side_effect_unknown;
  std::function<void()> resolve_delayed_side_effect_unknown;
};

// Owns the complete floor-switch vertical slice: four HTTP routes, strict live
// action transactions and cancellation, the legacy offline selection service,
// exact-map verification, floor-runtime health subscriptions/interlock, and all
// worker/client shutdown state. Cross-domain map activation and runtime facts
// enter only through narrow ports.
class FloorSwitchModule
{
public:
  FloorSwitchModule(
    rclcpp::Node & node,
    rclcpp::CallbackGroup::SharedPtr callback_group,
    MapCatalog & map_catalog,
    std::mutex & map_asset_mutation_mutex,
    std::mutex & activation_commit_mutex,
    FloorSwitchModuleConfig config,
    FloorSwitchModulePorts ports);
  ~FloorSwitchModule();

  FloorSwitchModule(const FloorSwitchModule &) = delete;
  FloorSwitchModule & operator=(const FloorSwitchModule &) = delete;

  std::optional<HttpResponse> handle_http(
    const HttpRequest & request,
    ElevatorMotionAdmissionFence::Epoch motion_admission_epoch);

  bool interlock_enabled() const noexcept;
  FloorRuntimeInterlockDecision interlock_decision() const;
  bool operation_blocked(
    const std::string & operation,
    std::string & detail,
    std::string * reason_code = nullptr) const;
  std::optional<HttpResponse> interlock_response(
    const std::string & operation) const;
  void shutdown();

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace robot_api_server::features::floor_switch
