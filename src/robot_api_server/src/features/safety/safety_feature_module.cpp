#include "robot_api_server/features/safety/safety_feature_module.hpp"

#include <stdexcept>
#include <utility>

#include "robot_api_server/features/docking/docking_feature_module.hpp"
#include "robot_api_server/features/elevator/elevator_module.hpp"
#include "robot_api_server/features/floor_switch/floor_switch_module.hpp"
#include "robot_api_server/features/navigation/navigation_feature_module.hpp"
#include "robot_api_server/features/navigation/terminal_control/navigation_terminal_runtime_module.hpp"
#include "robot_api_server/features/safety/safety_module.hpp"
#include "robot_api_server/features/teleop/teleop_module.hpp"
#include "robot_api_server/infrastructure/http/http_common.hpp"

namespace robot_api_server::features::safety {

namespace {

void validate_dependencies(
    const SafetyFeatureModuleDependencies &dependencies) {
  if (!dependencies.floor_switch || !dependencies.elevator ||
      !dependencies.teleop || !dependencies.navigation ||
      !dependencies.docking) {
    throw std::invalid_argument(
        "SafetyFeatureModule requires every runtime dependency provider");
  }
}

} // namespace

class SafetyFeatureModule::Impl {
public:
  Impl(rclcpp::Node &node, SafetyModuleConfig configuration,
       SafetyFeatureModuleDependencies dependencies)
      : dependencies_(std::move(dependencies)) {
    validate_dependencies(dependencies_);

    SafetyModulePorts ports;
    ports.floor_runtime_interlock_response =
        [this](const std::string &operation) {
          auto *floor_switch = dependencies_.floor_switch();
          return floor_switch != nullptr
                     ? floor_switch->interlock_response(operation)
                     : std::nullopt;
        };
    ports.run_admitted_operation =
        [this](const std::uint64_t motion_admission_epoch,
               const std::string &operation,
               const AdmittedSafetyOperation &work) {
          return work();
        };
    ports.clear_teleop_command = [this]() {
      auto *teleop = dependencies_.teleop();
      if (teleop != nullptr) {
        teleop->clear_command();
      }
    };
    ports.publish_teleop_zero_burst = [this]() {
      auto *teleop = dependencies_.teleop();
      if (teleop != nullptr) {
        teleop->publish_zero_burst();
      }
    };
    ports.publish_terminal_zero_burst = [this]() {
      auto *navigation = dependencies_.navigation();
      if (navigation != nullptr) {
        navigation->terminal_runtime().publish_zero_burst();
      }
    };
    ports.stop_predock_motion = [this]() {
      auto *docking = dependencies_.docking();
      if (docking != nullptr) {
        docking->stop_predock_motion();
      }
    };
    module_ = std::make_unique<SafetyModule>(node, std::move(configuration),
                                             std::move(ports));
  }

  SafetyFeatureModuleDependencies dependencies_;
  std::unique_ptr<SafetyModule> module_;
};

SafetyFeatureModule::SafetyFeatureModule(
    rclcpp::Node &node, SafetyModuleConfig configuration,
    SafetyFeatureModuleDependencies dependencies)
    : impl_(std::make_unique<Impl>(node, std::move(configuration),
                                   std::move(dependencies))) {}

SafetyFeatureModule::~SafetyFeatureModule() = default;

SafetyModule &SafetyFeatureModule::module() { return *impl_->module_; }

std::string SafetyFeatureModule::status_topic() const {
  return impl_->module_->status_topic();
}

std::string SafetyFeatureModule::motion_allowed_topic() const {
  return impl_->module_->motion_allowed_topic();
}

} // namespace robot_api_server::features::safety
