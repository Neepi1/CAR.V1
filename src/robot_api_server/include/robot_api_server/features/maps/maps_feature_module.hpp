#pragma once

#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>

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

namespace robot_api_server::features::localization
{
class LocalizationFeatureModule;
}

namespace robot_api_server::features::mapping
{
class MappingModule;
}

namespace robot_api_server::features::navigation
{
class NavigationFeatureModule;
}

namespace robot_api_server::features::maps
{

struct MapsConfiguration;
struct MapsRuntimePaths;
class MapRuntimeStateStore;
class MapsModule;

struct MapsFeatureModuleDependencies
{
  application::runtime_mode::RuntimeModeCoordinator * runtime_mode{nullptr};
  std::atomic<std::uint64_t> * delayed_side_effect_unknown_count{nullptr};
  std::function<features::mapping::MappingModule * ()> mapping;
  std::function<features::floor_switch::FloorSwitchModule * ()> floor_switch;
  std::function<features::navigation::NavigationFeatureModule * ()> navigation;
  std::function<features::elevator::ElevatorModule * ()> elevator;
  std::function<features::localization::LocalizationFeatureModule * ()> localization;
};

// Owns the complete maps aggregate, including the runtime map-context store,
// the cross-asset commit mutex, the catalog and every maps/poses/keepout API
// endpoint. Neighboring runtime facts remain late-bound read-only ports.
class MapsFeatureModule
{
public:
  MapsFeatureModule(
    rclcpp::Node & node,
    rclcpp::CallbackGroup::SharedPtr callback_group,
    MapsConfiguration configuration,
    MapsFeatureModuleDependencies dependencies);
  ~MapsFeatureModule();

  MapsFeatureModule(const MapsFeatureModule &) = delete;
  MapsFeatureModule & operator=(const MapsFeatureModule &) = delete;

  MapsModule & module();
  MapRuntimeStateStore & runtime_state_store();
  std::mutex & cross_asset_commit_mutex();
  const MapsRuntimePaths & runtime_paths() const;

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace robot_api_server::features::maps
