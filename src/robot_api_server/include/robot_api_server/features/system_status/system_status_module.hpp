#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <string>

#include "robot_api_server/application/runtime_mode/runtime_mode_coordinator.hpp"
#include "robot_api_server/features/floor_switch/floor_runtime_interlock.hpp"
#include "robot_api_server/features/localization/amcl_runtime_status.hpp"
#include "robot_api_server/features/localization/localization_module.hpp"
#include "robot_api_server/features/localization/localization_result_model.hpp"
#include "robot_api_server/features/power/bms_contact.hpp"
#include "robot_api_server/features/safety/safety_state.hpp"
#include "robot_api_server/features/system_status/robot_pose_model.hpp"
#include "robot_api_server/infrastructure/http/http_common.hpp"

namespace rclcpp
{
class Node;
}

namespace robot_api_server::features::system_status
{

struct SystemStatusDockSnapshot
{
  bool inferred_docked{false};
  bool auto_undock_required{false};
  bool can_auto_undock{false};
  std::string auto_undock_reason{"not_docked"};
  std::string pre_navigation_check_json{"{}"};
  std::string bms_contact_snapshot_json;
};

struct SystemStatusSnapshot
{
  std::string mapping_status_json;
  application::runtime_mode::RuntimeModeSnapshot runtime;
  safety::SafetyStateSnapshot safety;
  BmsChargingContactSnapshot bms;
  SystemStatusDockSnapshot dock;
  LocalizationResultSnapshot localization;
  localization::AmclRuntimeStatus amcl;
  localization::BridgeStatusSnapshot bridge;
  FloorRuntimeInterlockDecision floor_runtime_interlock;
  bool floor_runtime_interlock_enabled{false};
  bool keepout_integrity_degraded{false};
  bool bridge_safe_for_goal_start{true};
  std::string bridge_goal_start_detail;
  std::string navigation_goal_json{"{}"};
  std::string mode_transition_owner;
  std::string post_relocalization_settle_json{"{}"};
  std::string post_undock_settle_json{"{}"};
  std::string subscriptions_json{"{\"resources\":{}}"};
  std::uint64_t delayed_side_effect_unknown_count{0U};
  std::size_t http_active_connections{0U};
};

struct RobotPoseRuntimeContextSnapshot
{
  bool available{false};
  bool blocked{false};
  std::string state;
  std::string startup_stage;
  std::string message;
};

struct RobotPoseIdentitySnapshot
{
  bool blocked_by_pending_context{false};
  std::string error;
  RobotPoseMapIdentity identity;
};

struct SystemStatusModuleConfig
{
  std::string floor_status_topic{"/floor_manager/status"};
  std::string map_frame{"map"};
  std::string base_frame{"base_link"};
  std::string navigate_to_pose_action{"/navigate_to_pose"};
  std::string docking_status_topic{"/docking/status"};
  std::string localization_trigger_service;
  std::string localization_result_topic;
  std::string bms_state_topic{"/battery_state"};
  std::string maps_root;
  std::string runtime_maps_dir;
  std::size_t max_http_connections{0U};
};

struct SystemStatusModulePorts
{
  std::function<SystemStatusSnapshot()> status_snapshot;
  std::function<RobotPoseRuntimeContextSnapshot()> robot_pose_runtime_context;
  std::function<RobotPoseSnapshot(std::string &)> wait_for_robot_pose;
  std::function<RobotPoseIdentitySnapshot()> robot_pose_identity;
};

// Owns the complete read-only system-status HTTP slice and its one resident
// floor-status input. Neighboring domains supply immutable snapshots only.
class SystemStatusModule
{
public:
  SystemStatusModule(
    rclcpp::Node & node,
    SystemStatusModuleConfig config,
    SystemStatusModulePorts ports);
  ~SystemStatusModule();

  SystemStatusModule(const SystemStatusModule &) = delete;
  SystemStatusModule & operator=(const SystemStatusModule &) = delete;

  std::optional<HttpResponse> handle_http(const HttpRequest & request);
  void ensure_floor_status_subscription_active();

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace robot_api_server::features::system_status
