#pragma once

#include <cstdint>
#include <functional>
#include <optional>

#include "robot_api_server/infrastructure/http/http_common.hpp"

namespace robot_api_server::application::subscriptions
{
class SubscriptionModule;
}

namespace robot_api_server::features::docking
{
class DockingHttpModule;
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
}

namespace robot_api_server::features::mapping
{
class MappingModule;
}

namespace robot_api_server::features::maps
{
class MapsModule;
}

namespace robot_api_server::features::navigation
{
class NavigationModule;
}

namespace robot_api_server::features::safety
{
class SafetyModule;
}

namespace robot_api_server::features::system_status
{
class SystemStatusModule;
}

namespace robot_api_server::infrastructure::http
{
class ApiGatewayModule;
}

namespace robot_api_server::application::routing
{

using MotionAdmissionEpoch = std::uint64_t;
using PlainHttpHandler =
  std::function<std::optional<HttpResponse>(const HttpRequest &)>;
using AdmittedHttpHandler =
  std::function<std::optional<HttpResponse>(const HttpRequest &, MotionAdmissionEpoch)>;

struct ApplicationRouterModulePorts
{
  std::function<MotionAdmissionEpoch()> capture_motion_admission_epoch;
  std::function<std::optional<HttpResponse>(const HttpRequest &)> elevator_interlock;
  PlainHttpHandler system_status;
  AdmittedHttpHandler maps;
  std::function<std::optional<HttpResponse>(
      const HttpRequest &, MotionAdmissionEpoch, bool, bool)> elevator;
  AdmittedHttpHandler mapping;
  PlainHttpHandler gateway_metadata;
  PlainHttpHandler subscriptions;
  AdmittedHttpHandler safety;
  AdmittedHttpHandler floor_switch;
  AdmittedHttpHandler localization;
  AdmittedHttpHandler navigation;
  AdmittedHttpHandler docking;
  std::function<bool()> gateway_token_configured;
};

struct ApplicationRouterDependencies
{
  features::elevator::ElevatorModule * elevator{nullptr};
  features::system_status::SystemStatusModule * system_status{nullptr};
  features::maps::MapsModule * maps{nullptr};
  features::mapping::MappingModule * mapping{nullptr};
  application::subscriptions::SubscriptionModule * subscriptions{nullptr};
  features::safety::SafetyModule * safety{nullptr};
  features::floor_switch::FloorSwitchModule * floor_switch{nullptr};
  features::localization::LocalizationModule * localization{nullptr};
  features::navigation::NavigationModule * navigation{nullptr};
  features::docking::DockingHttpModule * docking{nullptr};
  infrastructure::http::ApiGatewayModule * gateway{nullptr};
};

ApplicationRouterModulePorts make_application_router_ports(
  ApplicationRouterDependencies dependencies);

// Owns authenticated application-route precedence and fallback policy. HTTP
// transport, authentication, and individual feature endpoint semantics remain
// in their respective gateway and feature modules.
class ApplicationRouterModule
{
public:
  explicit ApplicationRouterModule(ApplicationRouterModulePorts ports);

  HttpResponse route(
    const HttpRequest & request,
    bool maintenance_peer_is_loopback) const;

private:
  ApplicationRouterModulePorts ports_;
};

}  // namespace robot_api_server::application::routing
