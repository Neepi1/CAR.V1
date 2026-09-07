#include "robot_api_server/features/mapping/mapping_feature_module.hpp"

#include <stdexcept>
#include <string>
#include <utility>

#include "robot_api_server/application/runtime_mode/runtime_mode_coordinator.hpp"
#include "robot_api_server/features/elevator/elevator_module.hpp"
#include "robot_api_server/features/floor_switch/floor_switch_module.hpp"
#include "robot_api_server/features/mapping/mapping_module.hpp"
#include "robot_api_server/features/maps/catalog_activation/map_runtime_state_store.hpp"
#include "robot_api_server/features/maps/maps_module.hpp"
#include "robot_api_server/features/navigation/navigation_feature_module.hpp"
#include "robot_api_server/features/navigation/navigation_module.hpp"
#include "robot_api_server/features/navigation/mission/navigation_mission_runtime.hpp"
#include \
  "robot_api_server/features/navigation/terminal_control/navigation_terminal_runtime_module.hpp"
#include "robot_api_server/features/teleop/teleop_module.hpp"

namespace robot_api_server::features::mapping
{

namespace
{

void validate_dependencies(
  const MappingFeatureModuleDependencies & dependencies)
{
  if (dependencies.maps == nullptr ||
    dependencies.map_runtime_state_store == nullptr ||
    dependencies.floor_switch == nullptr ||
    dependencies.runtime_mode == nullptr ||
    !dependencies.navigation || !dependencies.elevator || !dependencies.teleop)
  {
    throw std::invalid_argument(
            "MappingFeatureModule requires every runtime dependency provider");
  }
}

}  // namespace

class MappingFeatureModule::Impl
{
public:
  Impl(
    rclcpp::Node & node,
    rclcpp::CallbackGroup::SharedPtr callback_group,
    MappingModuleConfig configuration,
    MappingFeatureModuleDependencies dependencies)
  : dependencies_(std::move(dependencies))
  {
    validate_dependencies(dependencies_);

    MappingModulePorts ports;
    ports.floor_runtime_operation_blocked = [this](
      const std::string & operation, std::string & detail) {
        return dependencies_.floor_switch->operation_blocked(operation, detail);
      };
    ports.floor_runtime_interlock_response = [this](const std::string & operation) {
        return dependencies_.floor_switch->interlock_response(operation);
      };
    ports.map_asset_integrity_degraded = [this]() {
        return dependencies_.maps->integrity_degraded();
      };
    ports.acquire_motion_admission = [this](
      const ElevatorMotionAdmissionFence::Epoch expected_epoch) {
        auto * elevator = dependencies_.elevator();
        if (elevator == nullptr) {
          throw std::logic_error("elevator module is not initialized");
        }
        return elevator->acquire_motion_admission(expected_epoch);
      };
    ports.navigation_goal_running = [this]() {
        auto * navigation = dependencies_.navigation();
        return navigation != nullptr && navigation->mission_runtime().running();
      };
    ports.cancel_navigation = [this]() {
        auto * navigation = dependencies_.navigation();
        if (navigation == nullptr) {
          return MappingNavigationActionResult{
          false, "navigation feature module is not initialized"};
        }
        std::string detail;
        const bool ok = navigation->module().cancel_task_for_mode_switch(
          "2D mapping mode transition", detail);
        auto * teleop = dependencies_.teleop();
        if (teleop != nullptr) {
          teleop->publish_zero_burst();
        }
        navigation->terminal_runtime().publish_zero_burst();
        return MappingNavigationActionResult{ok, detail};
      };
    ports.stop_navigation_runtime = [this]() {
        auto * navigation = dependencies_.navigation();
        if (navigation == nullptr) {
          return MappingNavigationActionResult{
          false, "navigation feature module is not initialized"};
        }
        std::string detail;
        const bool ok = navigation->module().stop_runtime_stack(detail);
        auto * teleop = dependencies_.teleop();
        if (teleop != nullptr) {
          teleop->publish_zero_burst();
        }
        navigation->terminal_runtime().publish_zero_burst();
        if (ok) {
          navigation->mission_runtime().join();
        }
        return MappingNavigationActionResult{ok, detail};
      };
    ports.clear_runtime_map_context = [this]() {
        dependencies_.map_runtime_state_store->clear_runtime_map_context();
      };
    module_ = std::make_unique<MappingModule>(
      node,
      std::move(callback_group),
      *dependencies_.runtime_mode,
      dependencies_.maps->catalog(),
      dependencies_.maps->mutation_mutex(),
      std::move(configuration),
      std::move(ports));
  }

  MappingFeatureModuleDependencies dependencies_;
  std::unique_ptr<MappingModule> module_;
};

MappingFeatureModule::MappingFeatureModule(
  rclcpp::Node & node,
  rclcpp::CallbackGroup::SharedPtr callback_group,
  MappingModuleConfig configuration,
  MappingFeatureModuleDependencies dependencies)
: impl_(std::make_unique<Impl>(
      node,
      std::move(callback_group),
      std::move(configuration),
      std::move(dependencies)))
{
}

MappingFeatureModule::~MappingFeatureModule() = default;

MappingModule & MappingFeatureModule::module() {return *impl_->module_;}

void MappingFeatureModule::shutdown()
{
  impl_->module_->shutdown();
}

}  // namespace robot_api_server::features::mapping
