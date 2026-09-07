#include "robot_api_server/application/subscriptions/subscription_feature_module.hpp"

#include <stdexcept>
#include <utility>

#include "robot_api_server/application/subscriptions/subscription_module.hpp"
#include "robot_api_server/features/localization/localization_feature_module.hpp"
#include "robot_api_server/features/localization/localization_module.hpp"
#include "robot_api_server/features/mapping/mapping_module.hpp"
#include "robot_api_server/features/system_status/system_status_module.hpp"
#include "robot_api_server/features/teleop/teleop_module.hpp"

namespace robot_api_server::application::subscriptions {

namespace {

void validate_dependencies(
    const SubscriptionFeatureModuleDependencies &dependencies) {
  if (!dependencies.system_status || !dependencies.mapping ||
      !dependencies.localization || !dependencies.teleop) {
    throw std::invalid_argument(
        "SubscriptionFeatureModule requires every runtime dependency provider");
  }
}

} // namespace

class SubscriptionFeatureModule::Impl {
public:
  Impl(rclcpp::Node &node, SubscriptionModuleConfig configuration,
       SubscriptionFeatureModuleDependencies dependencies)
      : dependencies_(std::move(dependencies)) {
    validate_dependencies(dependencies_);

    SubscriptionModulePorts ports;
    ports.ensure_status_resident = [this]() {
      auto *status = dependencies_.system_status();
      if (status != nullptr) {
        status->ensure_floor_status_subscription_active();
      }
    };
    ports.set_live_map_page_active = [this](const bool active) {
      auto *mapping = dependencies_.mapping();
      if (mapping != nullptr) {
        mapping->set_live_map_page_active(active);
      }
    };
    ports.ensure_tf_resident = [this]() {
      auto *localization = dependencies_.localization();
      if (localization != nullptr) {
        localization->module().ensure_tf_subscription_active();
      }
    };
    ports.clear_teleop_command = [this]() {
      auto *teleop = dependencies_.teleop();
      if (teleop != nullptr) {
        teleop->clear_command();
      }
    };
    module_ = std::make_unique<SubscriptionModule>(
        node, std::move(configuration), std::move(ports));
  }

  SubscriptionFeatureModuleDependencies dependencies_;
  std::unique_ptr<SubscriptionModule> module_;
};

SubscriptionFeatureModule::SubscriptionFeatureModule(
    rclcpp::Node &node, SubscriptionModuleConfig configuration,
    SubscriptionFeatureModuleDependencies dependencies)
    : impl_(std::make_unique<Impl>(node, std::move(configuration),
                                   std::move(dependencies))) {}

SubscriptionFeatureModule::~SubscriptionFeatureModule() = default;

SubscriptionModule &SubscriptionFeatureModule::module() {
  return *impl_->module_;
}

int SubscriptionFeatureModule::max_ttl_ms() const {
  return impl_->module_->max_ttl_ms();
}

} // namespace robot_api_server::application::subscriptions
