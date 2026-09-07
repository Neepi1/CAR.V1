#include "robot_api_server/features/power/power_feature_module.hpp"

#include <stdexcept>
#include <utility>

#include "robot_api_server/features/docking/docking_feature_module.hpp"
#include "robot_api_server/features/docking/lifecycle/dock_contact_interlock_module.hpp"
#include "robot_api_server/features/power/power_module.hpp"
#include "robot_api_server/features/teleop/teleop_module.hpp"

namespace robot_api_server::features::power {

namespace {

void validate_dependencies(const PowerFeatureModuleDependencies &dependencies) {
  if (!dependencies.docking || !dependencies.teleop) {
    throw std::invalid_argument(
        "PowerFeatureModule requires every runtime dependency provider");
  }
}

} // namespace

class PowerFeatureModule::Impl {
public:
  Impl(rclcpp::Node &node, PowerModuleConfig configuration,
       PowerFeatureModuleDependencies dependencies)
      : dependencies_(std::move(dependencies)) {
    validate_dependencies(dependencies_);

    PowerModulePorts ports;
    ports.on_contact_evidence = [this](const BatteryContactEvaluation &contact,
                                       const bool stable,
                                       const double stable_duration_sec) {
      auto *docking = dependencies_.docking();
      if (docking != nullptr) {
        docking->contact_interlock().on_bms_contact_evidence(
            contact, stable, stable_duration_sec);
      }
    };
    ports.on_charging_contact = [this](const bool contact) {
      auto *teleop = dependencies_.teleop();
      if (teleop != nullptr) {
        teleop->on_charging_contact(contact);
      }
    };
    module_ = std::make_unique<PowerModule>(node, std::move(configuration),
                                            std::move(ports));
  }

  PowerFeatureModuleDependencies dependencies_;
  std::unique_ptr<PowerModule> module_;
};

PowerFeatureModule::PowerFeatureModule(
    rclcpp::Node &node, PowerModuleConfig configuration,
    PowerFeatureModuleDependencies dependencies)
    : impl_(std::make_unique<Impl>(node, std::move(configuration),
                                   std::move(dependencies))) {}

PowerFeatureModule::~PowerFeatureModule() = default;

PowerModule &PowerFeatureModule::module() { return *impl_->module_; }

std::string PowerFeatureModule::state_topic() const {
  return impl_->module_->state_topic();
}

} // namespace robot_api_server::features::power
