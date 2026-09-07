#pragma once

#include <functional>
#include <memory>

namespace rclcpp {
class Node;
}

namespace robot_api_server {
struct HttpRequest;
}

namespace robot_api_server::application::subscriptions {
class SubscriptionModule;
}

namespace robot_api_server::features::elevator {
class ElevatorModule;
}

namespace robot_api_server::features::localization {
class LocalizationFeatureModule;
}

namespace robot_api_server::features::mapping {
class MappingModule;
}

namespace robot_api_server::features::power {
class PowerModule;
}

namespace robot_api_server::infrastructure::http {
class ApiGatewayModule;
}

namespace robot_api_server::features::teleop {

struct TeleopModuleConfig;
class TeleopModule;

struct TeleopFeatureModuleDependencies {
  std::function<infrastructure::http::ApiGatewayModule *()> api_gateway;
  std::function<features::elevator::ElevatorModule *()> elevator;
  features::power::PowerModule *power{nullptr};
  features::mapping::MappingModule *mapping{nullptr};
  std::function<features::localization::LocalizationFeatureModule *()>
      localization;
  application::subscriptions::SubscriptionModule *subscriptions{nullptr};
  double tf_pose_max_age_sec{0.0};
};

// Owns the complete mapping-only teleop module and every cross-domain port it
// consumes. Generic HTTP transport and final chassis arbitration stay outside.
class TeleopFeatureModule {
public:
  TeleopFeatureModule(rclcpp::Node &node, TeleopModuleConfig configuration,
                      TeleopFeatureModuleDependencies dependencies);
  ~TeleopFeatureModule();

  TeleopFeatureModule(const TeleopFeatureModule &) = delete;
  TeleopFeatureModule &operator=(const TeleopFeatureModule &) = delete;

  TeleopModule &module();
  bool handle_socket(int client_fd, const HttpRequest &request);
  void shutdown();

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

} // namespace robot_api_server::features::teleop
