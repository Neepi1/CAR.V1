#pragma once

#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

namespace robot_api_server::application::runtime_mode
{
class RuntimeModeCoordinator;
}

namespace robot_api_server::features::docking
{
class DockContactInterlockModule;
class DockingJobStore;
class DockingRuntimeModule;
class PreNavigationUndockModule;
}

namespace robot_api_server::features::elevator
{
class ElevatorModule;
}

namespace robot_api_server::features::floor_switch
{
class FloorSwitchModule;
}

namespace robot_api_server::features::localization
{
struct BridgeStatusSnapshot;
class LocalizationModule;
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

namespace robot_api_server::features::safety
{
class SafetyModule;
}

namespace robot_api_server::features::teleop
{
class TeleopModule;
}

namespace robot_api_server::features::navigation
{

namespace configuration
{
struct NavigationConfiguration;
}

struct BridgeReadinessSnapshot;
class NavigationActionRuntime;
class NavigationCompletionPolicy;
class NavigationGoalExecutionModule;
class NavigationMissionRuntime;
class NavigationModule;
class NavigationTerminalControl;
class NavigationTerminalRuntimeModule;

struct NavigationFeatureModuleDependencies
{
  features::maps::MapsModule * maps{nullptr};
  features::maps::MapRuntimeStateStore * map_runtime_state_store{nullptr};
  features::mapping::MappingModule * mapping{nullptr};
  features::localization::LocalizationModule * localization{nullptr};
  features::floor_switch::FloorSwitchModule * floor_switch{nullptr};
  features::elevator::ElevatorModule * elevator{nullptr};
  features::safety::SafetyModule * safety{nullptr};
  features::teleop::TeleopModule * teleop{nullptr};
  features::docking::DockContactInterlockModule * dock_contact_interlock{nullptr};
  features::docking::DockingJobStore * docking_job_store{nullptr};
  features::docking::DockingRuntimeModule * docking_runtime{nullptr};
  features::docking::PreNavigationUndockModule * pre_navigation_undock{nullptr};
  application::runtime_mode::RuntimeModeCoordinator * runtime_mode{nullptr};
  std::atomic<std::uint64_t> * delayed_side_effect_unknown_count{nullptr};

  // These owners are intentionally initialized after navigation. The
  // callbacks are evaluated only after the complete API composition starts.
  std::function<bool()> api_runtime_running;
  std::function<std::string()> post_relocalization_settle_json;
};

// Owns the complete ordinary-navigation runtime family and all of its
// cross-feature adapters. The process entry point keeps one object instead of
// assembling NavigationModule, terminal control, goal execution, and their
// delayed callback cycle independently.
class NavigationFeatureModule
{
public:
  NavigationFeatureModule(
    rclcpp::Node & node,
    rclcpp::CallbackGroup::SharedPtr callback_group,
    configuration::NavigationConfiguration configuration,
    NavigationFeatureModuleDependencies dependencies);
  ~NavigationFeatureModule();

  NavigationFeatureModule(const NavigationFeatureModule &) = delete;
  NavigationFeatureModule & operator=(const NavigationFeatureModule &) = delete;

  NavigationModule & module();
  NavigationActionRuntime & action_runtime();
  NavigationMissionRuntime & mission_runtime();
  NavigationCompletionPolicy & completion_policy();
  NavigationTerminalControl & terminal_control();
  NavigationTerminalRuntimeModule & terminal_runtime();
  NavigationGoalExecutionModule & goal_execution();

  BridgeReadinessSnapshot bridge_readiness_snapshot(
    const features::localization::BridgeStatusSnapshot & bridge) const;
  bool bridge_status_safe_for_goal_start(
    const features::localization::BridgeStatusSnapshot & bridge,
    const std::string & context,
    std::string & detail) const;
  bool bridge_safe_for_goal_start(
    const std::string & context,
    std::string & detail) const;

  void shutdown();

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace robot_api_server::features::navigation
