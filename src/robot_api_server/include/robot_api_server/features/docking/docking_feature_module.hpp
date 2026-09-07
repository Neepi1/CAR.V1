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
class NavigationFeatureModule;
}

namespace robot_api_server::features::power
{
class PowerModule;
}

namespace robot_api_server::features::safety
{
class SafetyModule;
}

namespace robot_api_server::features::teleop
{
class TeleopModule;
}

namespace robot_api_server::features::docking
{

namespace configuration
{
struct DockingConfiguration;
}

class DockContactInterlockModule;
class DockingHttpModule;
class DockingJobStore;
class DockingRuntimeModule;
class PreNavigationUndockModule;

// Stable neighboring services used by the complete docking feature.  The
// navigation callback is intentionally late-bound because ordinary navigation
// consumes the core docking interlock before the rest of docking is composed.
struct DockingFeatureModuleDependencies
{
  features::maps::MapsModule * maps{nullptr};
  features::maps::MapRuntimeStateStore * map_runtime_state_store{nullptr};
  features::mapping::MappingModule * mapping{nullptr};
  features::localization::LocalizationModule * localization{nullptr};
  features::floor_switch::FloorSwitchModule * floor_switch{nullptr};
  features::safety::SafetyModule * safety{nullptr};
  features::teleop::TeleopModule * teleop{nullptr};
  features::power::PowerModule * power{nullptr};
  application::runtime_mode::RuntimeModeCoordinator * runtime_mode{nullptr};
  std::atomic<std::uint64_t> * delayed_side_effect_unknown_count{nullptr};
  std::function<bool()> navigation_goal_running;
};

// These owners depend on the core docking interlock, so they are attached in
// one explicit second phase before the API gateway starts accepting requests.
struct DockingFeatureLateDependencies
{
  features::elevator::ElevatorModule * elevator{nullptr};
  features::navigation::NavigationFeatureModule * navigation{nullptr};
  features::localization::PostRelocalizationSettleModule *
    post_relocalization_settle{nullptr};
};

// Owns the entire docking aggregate: contact evidence, job state, correction
// pause, manager runtime, automatic undock, predock alignment, job execution,
// HTTP handling, and status reconciliation.
class DockingFeatureModule
{
public:
  DockingFeatureModule(
    rclcpp::Node & node,
    rclcpp::CallbackGroup::SharedPtr callback_group,
    configuration::DockingConfiguration configuration,
    DockingFeatureModuleDependencies dependencies);
  ~DockingFeatureModule();

  DockingFeatureModule(const DockingFeatureModule &) = delete;
  DockingFeatureModule & operator=(const DockingFeatureModule &) = delete;

  void complete(DockingFeatureLateDependencies dependencies);
  bool complete() const;

  DockContactInterlockModule & contact_interlock();
  DockingJobStore & job_store();
  DockingRuntimeModule & runtime();
  PreNavigationUndockModule & pre_navigation_undock();
  DockingHttpModule & http();

  bool job_running() const;
  void stop_predock_motion();

  // Preserve the original two-stage shutdown ordering: stop queued docking
  // work before neighboring modules, then stop ROS/runtime workers.
  void prepare_shutdown();
  void shutdown();

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace robot_api_server::features::docking
