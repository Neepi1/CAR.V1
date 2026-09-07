#pragma once

#include <array>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "robot_api_server/features/elevator/execution/elevator_runtime_policy.hpp"
#include "robot_api_server/features/maps/catalog_activation/map_catalog.hpp"
#include "robot_api_server/infrastructure/http/http_common.hpp"

namespace robot_api_server::features::elevator
{

struct ElevatorModuleConfig
{
  std::filesystem::path maps_root;
  std::filesystem::path runtime_map_context_file;
  bool runtime_adapter_enabled{false};
  bool arm_button_control_enabled{false};
  std::string arm_service_host{"127.0.0.1"};
  int arm_service_port{8083};
  double arm_request_timeout_sec{12.0};
  double arm_task_timeout_sec{180.0};
  double arm_poll_interval_sec{0.20};
  std::string navigate_to_pose_action{"/navigate_to_pose"};
  std::string floor_switch_action{"/floor_manager/floor_switch"};
  double floor_switch_timeout_sec{90.0};
  std::string elevator_scoped_behavior_tree;
  std::string elevator_hall_call_scoped_behavior_tree;
  std::string elevator_reverse_entry_staging_behavior_tree;
  std::string elevator_reverse_docking_behavior_tree;
  std::string elevator_cabin_entry_direct_behavior_tree;
  std::string elevator_entry_collision_bypass_permit_topic{
    "/ranger_mini3/elevator_entry_collision_bypass"};
  double elevator_entry_collision_bypass_refresh_sec{0.20};
  std::string motion_allowed_topic{"/safety/motion_allowed"};
  std::string safety_status_topic{"/safety/status"};
  std::string navigation_status_topic{"/navigate_to_pose/_action/status"};
  std::string recovery_hold_release_service{
    "/safety/release_motion_hold_if_execution_idle"};
};

struct ElevatorModulePorts
{
  std::function<std::optional<HttpResponse>(const std::string & operation)>
  floor_runtime_interlock_response;
  std::function<std::array<bool, 3>()> runtime_idle_probe;
  std::function<std::uint64_t()> delayed_side_effect_unknown_probe;
  std::function<std::optional<ElevatorMapPose>()> current_map_pose_probe;
};

// Owns the complete API-facing elevator vertical slice: immutable
// configuration releases, commissioning-test HTTP transactions, the ROS
// execution adapter, recovery interlock, and motion-admission fence. The arm
// service remains an external black box and is only called through its client.
class ElevatorModule
{
public:
  ElevatorModule(
    rclcpp::Node & node,
    MapCatalog & map_catalog,
    std::mutex & map_mutation_mutex,
    std::mutex & cross_asset_commit_mutex,
    ElevatorModuleConfig config,
    ElevatorModulePorts ports);
  ~ElevatorModule();

  ElevatorModule(const ElevatorModule &) = delete;
  ElevatorModule & operator=(const ElevatorModule &) = delete;

  std::optional<HttpResponse> handle_http(
    const HttpRequest & request,
    ElevatorMotionAdmissionFence::Epoch motion_admission_epoch,
    bool api_token_configured,
    bool maintenance_peer_is_loopback);

  std::optional<HttpResponse> interlock_response(
    const HttpRequest & request) const;
  ElevatorExecutionInterlock execution_interlock() const;
  ElevatorMotionAdmissionFence::Epoch capture_motion_admission_epoch() const;
  ElevatorMotionAdmissionFence::AdmissionGuard acquire_motion_admission(
    ElevatorMotionAdmissionFence::Epoch expected_epoch) const;
  HttpResponse motion_admission_failure_response(
    const std::string & operation,
    const ElevatorMotionAdmissionFence::AdmissionGuard & admission) const;
  HttpResponse query_map_reference(const MapManifest & manifest) const;

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace robot_api_server::features::elevator
