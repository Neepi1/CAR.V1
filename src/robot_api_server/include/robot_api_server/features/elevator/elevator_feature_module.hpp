#pragma once

#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>

namespace rclcpp {
class Node;
}

namespace robot_api_server::application::runtime_mode {
class RuntimeModeCoordinator;
}

namespace robot_api_server::features::docking {
class DockingFeatureModule;
}

namespace robot_api_server::features::floor_switch {
class FloorSwitchModule;
}

namespace robot_api_server::features::localization {
class LocalizationFeatureModule;
}

namespace robot_api_server::features::mapping {
class MappingModule;
}

namespace robot_api_server::features::maps {
class MapsFeatureModule;
}

namespace robot_api_server::features::navigation {
class NavigationFeatureModule;
}

namespace robot_api_server::features::teleop {
class TeleopModule;
}

namespace robot_api_server::features::elevator {

struct ElevatorModuleConfig;
class ElevatorModule;

struct ElevatorFeatureModuleDependencies {
  features::maps::MapsFeatureModule *maps{nullptr};
  features::mapping::MappingModule *mapping{nullptr};
  features::floor_switch::FloorSwitchModule *floor_switch{nullptr};
  application::runtime_mode::RuntimeModeCoordinator *runtime_mode{nullptr};
  std::atomic<std::uint64_t> *delayed_side_effect_unknown_count{nullptr};
  std::function<features::localization::LocalizationFeatureModule *()>
      localization;
  std::function<features::navigation::NavigationFeatureModule *()> navigation;
  std::function<features::docking::DockingFeatureModule *()> docking;
  std::function<features::teleop::TeleopModule *()> teleop;
  std::string map_frame;
  double robot_pose_freshness_sec{0.0};
};

// Owns the complete elevator module and the live runtime-observation graph
// used by its execution adapter. The mechanical-arm endpoint remains external.
class ElevatorFeatureModule {
public:
  ElevatorFeatureModule(rclcpp::Node &node, ElevatorModuleConfig configuration,
                        ElevatorFeatureModuleDependencies dependencies);
  ~ElevatorFeatureModule();

  ElevatorFeatureModule(const ElevatorFeatureModule &) = delete;
  ElevatorFeatureModule &operator=(const ElevatorFeatureModule &) = delete;

  ElevatorModule &module();

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

} // namespace robot_api_server::features::elevator
