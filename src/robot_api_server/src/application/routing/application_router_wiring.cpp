#include "robot_api_server/application/routing/application_router_module.hpp"

#include <stdexcept>
#include <string>

#include "robot_api_server/application/subscriptions/subscription_module.hpp"
#include "robot_api_server/features/docking/lifecycle/docking_http_module.hpp"
#include "robot_api_server/features/elevator/elevator_module.hpp"
#include "robot_api_server/features/floor_switch/floor_switch_module.hpp"
#include "robot_api_server/features/localization/localization_module.hpp"
#include "robot_api_server/features/mapping/mapping_module.hpp"
#include "robot_api_server/features/maps/maps_module.hpp"
#include "robot_api_server/features/navigation/navigation_module.hpp"
#include "robot_api_server/features/safety/safety_module.hpp"
#include "robot_api_server/features/system_status/system_status_module.hpp"
#include "robot_api_server/infrastructure/http/api_gateway_module.hpp"

namespace robot_api_server::application::routing
{
namespace
{

void require_dependency(const bool available, const char * name)
{
  if (!available) {
    throw std::invalid_argument(
            std::string("ApplicationRouterModule requires dependency: ") + name);
  }
}

}  // namespace

ApplicationRouterModulePorts make_application_router_ports(
  const ApplicationRouterDependencies dependencies)
{
  require_dependency(dependencies.elevator != nullptr, "elevator");
  require_dependency(dependencies.system_status != nullptr, "system_status");
  require_dependency(dependencies.maps != nullptr, "maps");
  require_dependency(dependencies.mapping != nullptr, "mapping");
  require_dependency(dependencies.subscriptions != nullptr, "subscriptions");
  require_dependency(dependencies.safety != nullptr, "safety");
  require_dependency(dependencies.floor_switch != nullptr, "floor_switch");
  require_dependency(dependencies.localization != nullptr, "localization");
  require_dependency(dependencies.navigation != nullptr, "navigation");
  require_dependency(dependencies.docking != nullptr, "docking");
  require_dependency(dependencies.gateway != nullptr, "gateway");

  ApplicationRouterModulePorts ports;
  ports.capture_motion_admission_epoch = [elevator = dependencies.elevator]() {
      return elevator->capture_motion_admission_epoch();
    };
  ports.elevator_interlock = [elevator = dependencies.elevator](const HttpRequest & request) {
      return elevator->interlock_response(request);
    };
  ports.system_status = [system_status = dependencies.system_status](const HttpRequest & request) {
      return system_status->handle_http(request);
    };
  ports.maps = [maps = dependencies.maps](
    const HttpRequest & request, const std::uint64_t motion_admission_epoch) {
      return maps->handle_http(request, motion_admission_epoch);
    };
  ports.elevator = [elevator = dependencies.elevator](
    const HttpRequest & request,
    const std::uint64_t motion_admission_epoch,
    const bool token_configured,
    const bool maintenance_peer_is_loopback) {
      return elevator->handle_http(
        request,
        motion_admission_epoch,
        token_configured,
        maintenance_peer_is_loopback);
    };
  ports.mapping = [mapping = dependencies.mapping](
    const HttpRequest & request, const std::uint64_t motion_admission_epoch) {
      return mapping->handle_http(request, motion_admission_epoch);
    };
  ports.gateway_metadata = [gateway = dependencies.gateway](const HttpRequest & request) {
      return gateway->handle_metadata(request);
    };
  ports.subscriptions = [subscriptions = dependencies.subscriptions](const HttpRequest & request) {
      return subscriptions->handle_http(request);
    };
  ports.safety = [safety = dependencies.safety](
    const HttpRequest & request, const std::uint64_t motion_admission_epoch) {
      return safety->handle_http(request, motion_admission_epoch);
    };
  ports.floor_switch = [floor_switch = dependencies.floor_switch](
    const HttpRequest & request, const std::uint64_t motion_admission_epoch) {
      return floor_switch->handle_http(request, motion_admission_epoch);
    };
  ports.localization = [localization = dependencies.localization](
    const HttpRequest & request, const std::uint64_t motion_admission_epoch) {
      return localization->handle_http(request, motion_admission_epoch);
    };
  ports.navigation = [navigation = dependencies.navigation](
    const HttpRequest & request, const std::uint64_t motion_admission_epoch) {
      return navigation->handle_http(request, motion_admission_epoch);
    };
  ports.docking = [docking = dependencies.docking](
    const HttpRequest & request, const std::uint64_t motion_admission_epoch) {
      return docking->handle_http(request, motion_admission_epoch);
    };
  ports.gateway_token_configured = [gateway = dependencies.gateway]() {
      return gateway->token_configured();
    };
  return ports;
}

}  // namespace robot_api_server::application::routing
