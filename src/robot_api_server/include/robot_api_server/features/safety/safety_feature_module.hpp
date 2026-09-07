#pragma once

#include <functional>
#include <memory>
#include <string>

namespace rclcpp {
class Node;
}

namespace robot_api_server::features::docking {
class DockingFeatureModule;
}

namespace robot_api_server::features::elevator {
class ElevatorModule;
}

namespace robot_api_server::features::floor_switch {
class FloorSwitchModule;
}

namespace robot_api_server::features::navigation {
class NavigationFeatureModule;
}

namespace robot_api_server::features::teleop {
class TeleopModule;
}

namespace robot_api_server::features::safety {

struct SafetyModuleConfig;
class SafetyModule;

struct SafetyFeatureModuleDependencies {
  std::function<features::floor_switch::FloorSwitchModule *()> floor_switch;
  std::function<features::elevator::ElevatorModule *()> elevator;
  std::function<features::teleop::TeleopModule *()> teleop;
  std::function<features::navigation::NavigationFeatureModule *()> navigation;
  std::function<features::docking::DockingFeatureModule *()> docking;
};

// Owns the App-facing safety module and its cross-domain stop/admission port
// graph. Final velocity arbitration remains exclusively in robot_safety.
class SafetyFeatureModule {
public:
  SafetyFeatureModule(rclcpp::Node &node, SafetyModuleConfig configuration,
                      SafetyFeatureModuleDependencies dependencies);
  ~SafetyFeatureModule();

  SafetyFeatureModule(const SafetyFeatureModule &) = delete;
  SafetyFeatureModule &operator=(const SafetyFeatureModule &) = delete;

  SafetyModule &module();
  std::string status_topic() const;
  std::string motion_allowed_topic() const;

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

} // namespace robot_api_server::features::safety
