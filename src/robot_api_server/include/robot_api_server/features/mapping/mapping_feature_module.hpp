#pragma once

#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"

namespace robot_api_server::application::runtime_mode
{
class RuntimeModeCoordinator;
}

namespace robot_api_server::features::elevator
{
class ElevatorModule;
}

namespace robot_api_server::features::floor_switch
{
class FloorSwitchModule;
}

namespace robot_api_server::features::maps
{
class MapRuntimeStateStore;
class MapsModule;
}

namespace robot_api_server::features::navigation
{
class NavigationFeatureModule;
}

namespace robot_api_server::features::teleop
{
class TeleopModule;
}

namespace robot_api_server::features::mapping
{

struct MappingModuleConfig;
class MappingModule;

struct MappingFeatureModuleDependencies
{
  features::maps::MapsModule * maps{nullptr};
  features::maps::MapRuntimeStateStore * map_runtime_state_store{nullptr};
  features::floor_switch::FloorSwitchModule * floor_switch{nullptr};
  application::runtime_mode::RuntimeModeCoordinator * runtime_mode{nullptr};
  std::function<features::navigation::NavigationFeatureModule * ()> navigation;
  std::function<features::elevator::ElevatorModule * ()> elevator;
  std::function<features::teleop::TeleopModule * ()> teleop;
};

// Owns the complete mapping aggregate and all of its neighboring-domain port
// wiring. FAST-LIO2 remains an external process controlled through the
// existing MappingModule runtime; this aggregate does not change algorithms.
class MappingFeatureModule
{
public:
  MappingFeatureModule(
    rclcpp::Node & node,
    rclcpp::CallbackGroup::SharedPtr callback_group,
    MappingModuleConfig configuration,
    MappingFeatureModuleDependencies dependencies);
  ~MappingFeatureModule();

  MappingFeatureModule(const MappingFeatureModule &) = delete;
  MappingFeatureModule & operator=(const MappingFeatureModule &) = delete;

  MappingModule & module();
  void shutdown();

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace robot_api_server::features::mapping
