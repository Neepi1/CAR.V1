#pragma once

#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>

#include "rclcpp/rclcpp.hpp"

namespace robot_api_server::application::runtime_mode
{
class RuntimeModeCoordinator;
}

namespace robot_api_server::features::docking
{
class DockingFeatureModule;
}

namespace robot_api_server::features::elevator
{
class ElevatorModule;
}

namespace robot_api_server::features::mapping
{
class MappingModule;
}

namespace robot_api_server::features::maps
{
class MapsModule;
}

namespace robot_api_server::features::navigation
{
class NavigationFeatureModule;
}

namespace robot_api_server::features::floor_switch
{

struct FloorSwitchConfiguration;
class FloorSwitchModule;

struct FloorSwitchFeatureModuleDependencies
{
  features::maps::MapsModule * maps{nullptr};
  application::runtime_mode::RuntimeModeCoordinator * runtime_mode{nullptr};
  std::atomic<std::uint64_t> * delayed_side_effect_unknown_count{nullptr};
  std::function<features::mapping::MappingModule * ()> mapping;
  std::function<features::navigation::NavigationFeatureModule * ()> navigation;
  std::function<features::docking::DockingFeatureModule * ()> docking;
  std::function<features::elevator::ElevatorModule * ()> elevator;
};

// Owns the complete floor-switch aggregate and its configuration projection.
// Cross-domain activity is observed through late-bound providers so the
// established construction order and atomic switch transaction stay intact.
class FloorSwitchFeatureModule
{
public:
  FloorSwitchFeatureModule(
    rclcpp::Node & node,
    rclcpp::CallbackGroup::SharedPtr callback_group,
    FloorSwitchConfiguration configuration,
    std::mutex & cross_asset_commit_mutex,
    FloorSwitchFeatureModuleDependencies dependencies);
  ~FloorSwitchFeatureModule();

  FloorSwitchFeatureModule(const FloorSwitchFeatureModule &) = delete;
  FloorSwitchFeatureModule & operator=(const FloorSwitchFeatureModule &) = delete;

  FloorSwitchModule & module();
  const std::string & floor_status_topic() const;
  const std::string & live_action() const;
  double live_timeout_sec() const;
  void shutdown();

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace robot_api_server::features::floor_switch
