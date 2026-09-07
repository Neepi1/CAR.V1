#pragma once

#include <atomic>
#include <cstdint>
#include <functional>
#include <string>

#include "robot_api_server/features/system_status/system_status_module.hpp"

namespace robot_api_server::application::runtime_mode
{
class RuntimeModeCoordinator;
}

namespace robot_api_server::application::subscriptions
{
class SubscriptionModule;
}

namespace robot_api_server::features::docking
{
class DockContactInterlockModule;
class DockingJobStore;
}

namespace robot_api_server::features::floor_switch
{
class FloorSwitchModule;
}

namespace robot_api_server::features::localization
{
class LocalizationModule;
class PostRelocalizationSettleModule;
}

namespace robot_api_server::features::mapping
{
class MappingModule;
}

namespace robot_api_server::features::maps
{
class MapRuntimeStateStore;
class MapsModule;
}

namespace robot_api_server::features::navigation
{
class NavigationMissionRuntime;
class NavigationModule;
}

namespace robot_api_server::features::power
{
class PowerModule;
}

namespace robot_api_server::features::safety
{
class SafetyModule;
}

namespace robot_api_server::infrastructure::http
{
class ApiGatewayModule;
}

namespace robot_api_server::features::system_status
{

using BridgeGoalStartEvaluator = std::function<bool (
      const localization::BridgeStatusSnapshot &,
      const std::string &,
      std::string &)>;

struct SystemStatusWiringDependencies
{
  mapping::MappingModule * mapping{nullptr};
  navigation::NavigationModule * navigation{nullptr};
  safety::SafetyModule * safety{nullptr};
  power::PowerModule * power{nullptr};
  application::runtime_mode::RuntimeModeCoordinator * runtime_mode{nullptr};
  docking::DockContactInterlockModule * dock_contact_interlock{nullptr};
  localization::LocalizationModule * localization{nullptr};
  navigation::NavigationMissionRuntime * navigation_mission{nullptr};
  docking::DockingJobStore * docking_job_store{nullptr};
  localization::PostRelocalizationSettleModule * post_relocalization_settle{nullptr};
  floor_switch::FloorSwitchModule * floor_switch{nullptr};
  maps::MapsModule * maps{nullptr};
  application::subscriptions::SubscriptionModule * subscriptions{nullptr};
  infrastructure::http::ApiGatewayModule * api_gateway{nullptr};
  maps::MapRuntimeStateStore * map_runtime_state_store{nullptr};
  std::atomic<std::uint64_t> * delayed_side_effect_unknown_count{nullptr};
  BridgeGoalStartEvaluator bridge_goal_start_evaluator;
};

SystemStatusModulePorts make_system_status_module_ports(
  SystemStatusWiringDependencies dependencies);

}  // namespace robot_api_server::features::system_status
