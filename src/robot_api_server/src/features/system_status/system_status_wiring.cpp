#include "robot_api_server/features/system_status/system_status_wiring.hpp"

#include <mutex>
#include <stdexcept>
#include <string>

#include "robot_api_server/application/runtime_mode/runtime_mode_coordinator.hpp"
#include "robot_api_server/application/subscriptions/subscription_module.hpp"
#include "robot_api_server/features/docking/lifecycle/dock_contact_interlock_module.hpp"
#include "robot_api_server/features/docking/lifecycle/docking_job_store.hpp"
#include "robot_api_server/features/floor_switch/floor_switch_module.hpp"
#include "robot_api_server/features/localization/localization_module.hpp"
#include "robot_api_server/features/localization/post_relocalization_settle_module.hpp"
#include "robot_api_server/features/mapping/mapping_module.hpp"
#include "robot_api_server/features/maps/catalog_activation/api_time_utils.hpp"
#include "robot_api_server/features/maps/catalog_activation/map_runtime_state_store.hpp"
#include "robot_api_server/features/maps/maps_module.hpp"
#include "robot_api_server/features/navigation/mission/navigation_mission_runtime.hpp"
#include "robot_api_server/features/navigation/navigation_module.hpp"
#include "robot_api_server/features/power/power_module.hpp"
#include "robot_api_server/features/safety/safety_module.hpp"
#include "robot_api_server/infrastructure/http/api_gateway_module.hpp"

namespace robot_api_server::features::system_status
{
namespace
{

void require_dependency(const bool available, const char * name)
{
  if (!available) {
    throw std::invalid_argument(
            std::string("system status wiring requires dependency: ") + name);
  }
}

}  // namespace

SystemStatusModulePorts make_system_status_module_ports(
  const SystemStatusWiringDependencies dependencies)
{
  require_dependency(dependencies.mapping != nullptr, "mapping");
  require_dependency(dependencies.navigation != nullptr, "navigation");
  require_dependency(dependencies.safety != nullptr, "safety");
  require_dependency(dependencies.power != nullptr, "power");
  require_dependency(dependencies.runtime_mode != nullptr, "runtime_mode");
  require_dependency(dependencies.dock_contact_interlock != nullptr, "dock_contact_interlock");
  require_dependency(dependencies.localization != nullptr, "localization");
  require_dependency(dependencies.navigation_mission != nullptr, "navigation_mission");
  require_dependency(dependencies.docking_job_store != nullptr, "docking_job_store");
  require_dependency(
    dependencies.post_relocalization_settle != nullptr, "post_relocalization_settle");
  require_dependency(dependencies.floor_switch != nullptr, "floor_switch");
  require_dependency(dependencies.maps != nullptr, "maps");
  require_dependency(dependencies.subscriptions != nullptr, "subscriptions");
  require_dependency(dependencies.api_gateway != nullptr, "api_gateway");
  require_dependency(dependencies.map_runtime_state_store != nullptr, "map_runtime_state_store");
  require_dependency(
    dependencies.delayed_side_effect_unknown_count != nullptr,
    "delayed_side_effect_unknown_count");
  require_dependency(
    static_cast<bool>(dependencies.bridge_goal_start_evaluator),
    "bridge_goal_start_evaluator");

  SystemStatusModulePorts ports;
  ports.status_snapshot = [dependencies]() {
      SystemStatusSnapshot snapshot;
      snapshot.mapping_status_json = dependencies.mapping->status_json();
      if (dependencies.runtime_mode->transition_owner() != "mapping_start") {
        dependencies.navigation->refresh_runtime_state(false);
      }
      snapshot.safety = dependencies.safety->snapshot();
      snapshot.bms = dependencies.power->snapshot();
      snapshot.runtime = dependencies.runtime_mode->snapshot();
      const auto dock_check = dependencies.dock_contact_interlock->snapshot();
      snapshot.dock = SystemStatusDockSnapshot{
        dock_check.inferred_docked,
        dock_check.final_auto_undock_required,
        dock_check.can_auto_undock,
        dock_check.auto_undock_reason,
        dependencies.dock_contact_interlock->pre_navigation_check_json(
          dock_check, docking::PreNavigationDockCheckContext{
          "status", "", "", "", "map", false}),
        dependencies.dock_contact_interlock->bms_snapshot_json(dock_check.bms)};
      snapshot.localization = dependencies.localization->localization_result_snapshot();
      snapshot.amcl = dependencies.localization->read_amcl_runtime_status(wall_time_seconds());
      snapshot.bridge = dependencies.localization->bridge_status_snapshot();
      snapshot.bridge_safe_for_goal_start = dependencies.bridge_goal_start_evaluator(
        snapshot.bridge, "status", snapshot.bridge_goal_start_detail);
      snapshot.navigation_goal_json = dependencies.navigation_mission->json();
      snapshot.mode_transition_owner = dependencies.runtime_mode->transition_owner();
      snapshot.post_relocalization_settle_json =
        dependencies.post_relocalization_settle->state_json();
      {
        std::lock_guard<std::mutex> lock(dependencies.docking_job_store->mutex());
        snapshot.post_undock_settle_json =
          dependencies.docking_job_store->post_undock_settle_json_locked();
      }
      snapshot.floor_runtime_interlock = dependencies.floor_switch->interlock_decision();
      snapshot.floor_runtime_interlock_enabled = dependencies.floor_switch->interlock_enabled();
      snapshot.keepout_integrity_degraded = dependencies.maps->integrity_degraded();
      snapshot.delayed_side_effect_unknown_count =
        dependencies.delayed_side_effect_unknown_count->load(std::memory_order_acquire);
      snapshot.subscriptions_json = dependencies.subscriptions->snapshot_json();
      snapshot.http_active_connections = dependencies.api_gateway->active_connections();
      return snapshot;
    };
  ports.robot_pose_runtime_context = [dependencies]() {
      RobotPoseRuntimeContextSnapshot snapshot;
      const auto runtime_context =
        dependencies.map_runtime_state_store->read_runtime_map_context();
      if (!runtime_context) {
        return snapshot;
      }
      snapshot.available = true;
      snapshot.state = runtime_context->state;
      snapshot.startup_stage = runtime_context->startup_stage;
      snapshot.message = runtime_context->message;
      snapshot.blocked =
        !runtime_context->confirmed ||
        runtime_context->state == "starting" ||
        runtime_context->state == "failed";
      return snapshot;
    };
  ports.wait_for_robot_pose = [dependencies](std::string & error) {
      return dependencies.localization->wait_for_current_robot_pose(true, error);
    };
  ports.robot_pose_identity = [dependencies]() {
      RobotPoseIdentitySnapshot snapshot;
      const auto active_map = dependencies.maps->confirmed_runtime_manifest(
        snapshot.error, snapshot.blocked_by_pending_context);
      if (active_map) {
        snapshot.identity.map_id = active_map->map_id;
        snapshot.identity.floor_id = active_map->floor_id;
        snapshot.identity.building_id = active_map->building_id;
      }
      return snapshot;
    };
  return ports;
}

}  // namespace robot_api_server::features::system_status
