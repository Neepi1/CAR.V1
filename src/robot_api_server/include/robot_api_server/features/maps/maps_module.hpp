#pragma once

#include <chrono>
#include <filesystem>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include "lifecycle_msgs/srv/get_state.hpp"
#include "rclcpp/rclcpp.hpp"

#include "robot_api_server/features/elevator/execution/elevator_runtime_policy.hpp"
#include "robot_api_server/features/maps/catalog_activation/map_asset_identity_binding.hpp"
#include "robot_api_server/features/maps/catalog_activation/map_catalog.hpp"
#include "robot_api_server/features/system_status/robot_pose_model.hpp"
#include "robot_api_server/infrastructure/http/http_common.hpp"

namespace robot_api_server::features::maps
{

struct MapsModuleConfig
{
  std::filesystem::path maps_root;
  std::filesystem::path runtime_maps_dir;
  std::filesystem::path runtime_map_context_file;
  std::string map_frame{"map"};
  std::string base_frame{"base_link"};
  std::string keepout_mask_load_service{"/keepout_filter_mask_server/load_map"};
  std::string keepout_mask_state_service{"/keepout_filter_mask_server/get_state"};
  std::string keepout_filter_info_state_service{
    "/keepout_costmap_filter_info_server/get_state"};
  std::string keepout_mask_get_parameters_service{
    "/keepout_filter_mask_server/get_parameters"};
  std::filesystem::path keepout_runtime_stage_root;
  std::string keepout_mask_topic{"/keepout_filter_mask"};
  std::string keepout_filter_info_topic{"/costmap_filter_info/keepout"};
  std::string global_costmap_get_parameters_service{
    "/global_costmap/global_costmap/get_parameters"};
  std::string global_costmap_state_service{
    "/global_costmap/global_costmap/get_state"};
  std::string global_costmap_clear_service{
    "/global_costmap/clear_entirely_global_costmap"};
  std::string global_costmap_topic{"/global_costmap/costmap"};
  double keepout_runtime_apply_timeout_sec{5.0};
};

struct MapsRuntimeSnapshot
{
  bool navigation_active{false};
  bool mapping_active{false};
  bool docking_active{false};
  bool mapping_start_job_running{false};
  bool navigation_goal_running{false};
  bool nav2_goal_active{false};
};

struct MapsModulePorts
{
  std::function<MapsRuntimeSnapshot(bool refresh_runtime)> runtime_snapshot;
  std::function<std::optional<HttpResponse>(const std::string & operation)>
  floor_runtime_interlock_response;
  std::function<RobotPoseSnapshot()> wait_for_current_robot_pose;
  std::function<bool(const std::string & context, std::string & detail)>
  wait_for_terminal_actual_stop;
  std::function<HttpResponse(const MapManifest & map)> query_elevator_reference;
  std::function<ElevatorMotionAdmissionFence::AdmissionGuard(
      ElevatorMotionAdmissionFence::Epoch expected_epoch)>
  acquire_motion_admission;
  std::function<void()> mark_delayed_side_effect_unknown;
  std::function<void()> resolve_delayed_side_effect_unknown;
};

// Owns the complete maps vertical slice: catalog startup/recovery, immutable
// map activation and deletion, semantic/pose assets, keepout persistence and
// its ROS runtime proof. The node remains a composition root and supplies only
// neighboring runtime facts through narrow ports.
class MapsModule
{
public:
  MapsModule(
    rclcpp::Node & node,
    rclcpp::CallbackGroup::SharedPtr callback_group,
    std::mutex & cross_asset_commit_mutex,
    MapsModuleConfig config,
    MapsModulePorts ports);
  ~MapsModule();

  MapsModule(const MapsModule &) = delete;
  MapsModule & operator=(const MapsModule &) = delete;

  std::optional<HttpResponse> handle_http(
    const HttpRequest & request,
    ElevatorMotionAdmissionFence::Epoch motion_admission_epoch);

  MapCatalog & catalog();
  std::mutex & mutation_mutex();
  rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr
  global_costmap_lifecycle_client();
  bool integrity_degraded();
  bool validate_manifest_assets(
    const MapManifest & manifest,
    std::string & error) const;
  bool validate_current_projection_assets(
    const MapManifest & source_manifest,
    const std::filesystem::path & current_root,
    std::string & error) const;
  bool runtime_context_matches(
    const MapManifest & manifest,
    std::string & error) const;
  void activate_manifest(
    MapManifest manifest,
    MapAssetCommitTransaction & transaction);
  std::optional<MapManifest> confirmed_runtime_manifest(
    std::string & unavailable_reason,
    bool & blocked_by_pending_context);

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace robot_api_server::features::maps
