#pragma once

#include <functional>
#include <memory>

namespace rclcpp {
class Node;
}

namespace robot_api_server::features::localization {
class LocalizationFeatureModule;
}

namespace robot_api_server::features::mapping {
class MappingModule;
}

namespace robot_api_server::features::system_status {
class SystemStatusModule;
}

namespace robot_api_server::features::teleop {
class TeleopModule;
}

namespace robot_api_server::application::subscriptions {

struct SubscriptionModuleConfig;
class SubscriptionModule;

struct SubscriptionFeatureModuleDependencies {
  std::function<features::system_status::SystemStatusModule *()> system_status;
  std::function<features::mapping::MappingModule *()> mapping;
  std::function<features::localization::LocalizationFeatureModule *()>
      localization;
  std::function<features::teleop::TeleopModule *()> teleop;
};

// Owns page-subscription leases and the complete resource-transition port
// graph. It does not own the underlying robot feature lifetimes.
class SubscriptionFeatureModule {
public:
  SubscriptionFeatureModule(rclcpp::Node &node,
                            SubscriptionModuleConfig configuration,
                            SubscriptionFeatureModuleDependencies dependencies);
  ~SubscriptionFeatureModule();

  SubscriptionFeatureModule(const SubscriptionFeatureModule &) = delete;
  SubscriptionFeatureModule &
  operator=(const SubscriptionFeatureModule &) = delete;

  SubscriptionModule &module();
  int max_ttl_ms() const;

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

} // namespace robot_api_server::application::subscriptions
