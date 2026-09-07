#pragma once

#include <optional>
#include <string>
#include <vector>

#include "robot_api_server/application/runtime_mode/runtime_mode_coordinator.hpp"
#include "robot_api_server/features/localization/amcl_runtime_status.hpp"
#include "robot_api_server/features/localization/localization_module.hpp"
#include "robot_api_server/features/maps/catalog_activation/storage_models.hpp"
#include "robot_api_server/features/safety/safety_state.hpp"
#include "robot_api_server/infrastructure/http/http_common.hpp"

namespace robot_api_server::features::navigation
{

struct NavigationStateConfig
{
  bool nav2_native_goal_completion_enabled{true};
  bool api_final_yaw_align_enabled{true};
  bool nav2_rotation_shim_enabled{true};
};

struct NavigationStateDockSnapshot
{
  std::string occupancy_state{"UNKNOWN"};
  std::vector<std::string> occupancy_evidence;
  std::string occupancy_reason{"no_dock_evidence"};
  bool charging_session_latched{false};
  double charging_session_age_sec{-1.0};
  std::string charging_session_last_confirmed_at;
  bool full_charge_idle_on_dock{false};
  bool auto_undock_required{false};
  std::string pre_navigation_check_json{"{}"};
};

struct NavigationStateSnapshot
{
  application::runtime_mode::RuntimeModeSnapshot runtime;
  std::optional<RuntimeMapContext> runtime_map_context;
  NavigationStateDockSnapshot dock;
  localization::AmclRuntimeStatus amcl;
  localization::BridgeStatusSnapshot bridge;
  safety::SafetyStateSnapshot safety;
  std::string post_relocalization_settle_json{"{}"};
  std::string post_undock_settle_json{"{}"};
  std::string navigation_goal_json{"{}"};
  std::string navigation_cancel_json{"{}"};
};

HttpResponse navigation_state_response(
  const NavigationStateConfig & config,
  const NavigationStateSnapshot & snapshot);

}  // namespace robot_api_server::features::navigation
