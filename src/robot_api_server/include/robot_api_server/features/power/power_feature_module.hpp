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

namespace robot_api_server::features::teleop {
class TeleopModule;
}

namespace robot_api_server::features::power {

struct PowerModuleConfig;
class PowerModule;

struct PowerFeatureModuleDependencies {
  std::function<features::docking::DockingFeatureModule *()> docking;
  std::function<features::teleop::TeleopModule *()> teleop;
};

// Owns the process-resident BMS edge and forwards its immutable contact
// evidence to docking and teleop without exposing subscription state.
class PowerFeatureModule {
public:
  PowerFeatureModule(rclcpp::Node &node, PowerModuleConfig configuration,
                     PowerFeatureModuleDependencies dependencies);
  ~PowerFeatureModule();

  PowerFeatureModule(const PowerFeatureModule &) = delete;
  PowerFeatureModule &operator=(const PowerFeatureModule &) = delete;

  PowerModule &module();
  std::string state_topic() const;

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

} // namespace robot_api_server::features::power
