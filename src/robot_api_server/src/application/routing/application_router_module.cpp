#include "robot_api_server/application/routing/application_router_module.hpp"

#include <stdexcept>
#include <string>
#include <utility>

namespace robot_api_server::application::routing
{
namespace
{

bool starts_with(const std::string & value, const std::string & prefix)
{
  return value.rfind(prefix, 0) == 0;
}

HttpResponse not_wired(const std::string & endpoint)
{
  return {
    501,
    "application/json",
    "{\"ok\":false,\"error\":\"endpoint is reserved but not wired to a ROS-native service/action yet\","  // NOLINT
    "\"endpoint\":" + json_string(endpoint) + "}"
  };
}

void require_port(const bool available, const char * name)
{
  if (!available) {
    throw std::invalid_argument(
            std::string("ApplicationRouterModule requires port: ") + name);
  }
}

}  // namespace

ApplicationRouterModule::ApplicationRouterModule(ApplicationRouterModulePorts ports)
: ports_(std::move(ports))
{
  require_port(
    static_cast<bool>(ports_.capture_motion_admission_epoch),
    "capture_motion_admission_epoch");
  require_port(static_cast<bool>(ports_.elevator_interlock), "elevator_interlock");
  require_port(static_cast<bool>(ports_.system_status), "system_status");
  require_port(static_cast<bool>(ports_.maps), "maps");
  require_port(static_cast<bool>(ports_.elevator), "elevator");
  require_port(static_cast<bool>(ports_.mapping), "mapping");
  require_port(static_cast<bool>(ports_.gateway_metadata), "gateway_metadata");
  require_port(static_cast<bool>(ports_.subscriptions), "subscriptions");
  require_port(static_cast<bool>(ports_.safety), "safety");
  require_port(static_cast<bool>(ports_.floor_switch), "floor_switch");
  require_port(static_cast<bool>(ports_.localization), "localization");
  require_port(static_cast<bool>(ports_.navigation), "navigation");
  require_port(static_cast<bool>(ports_.docking), "docking");
  require_port(
    static_cast<bool>(ports_.gateway_token_configured),
    "gateway_token_configured");
}

HttpResponse ApplicationRouterModule::route(
  const HttpRequest & request,
  const bool maintenance_peer_is_loopback) const
{
  const auto motion_admission_epoch = ports_.capture_motion_admission_epoch();
  if (const auto blocked = ports_.elevator_interlock(request)) {
    return *blocked;
  }

  if (const auto response = ports_.system_status(request)) {
    return *response;
  }
  if (const auto response = ports_.maps(request, motion_admission_epoch)) {
    return *response;
  }
  if (const auto response = ports_.elevator(
      request,
      motion_admission_epoch,
      ports_.gateway_token_configured(),
      maintenance_peer_is_loopback))
  {
    return *response;
  }
  if (const auto response = ports_.mapping(request, motion_admission_epoch)) {
    return *response;
  }
  if (const auto response = ports_.gateway_metadata(request)) {
    return *response;
  }
  if (const auto response = ports_.subscriptions(request)) {
    return *response;
  }
  if (const auto response = ports_.safety(request, motion_admission_epoch)) {
    return *response;
  }
  if (const auto response = ports_.floor_switch(request, motion_admission_epoch)) {
    return *response;
  }
  if (const auto response = ports_.localization(request, motion_admission_epoch)) {
    return *response;
  }
  if (const auto response = ports_.navigation(request, motion_admission_epoch)) {
    return *response;
  }
  if (const auto response = ports_.docking(request, motion_admission_epoch)) {
    return *response;
  }
  if (
    starts_with(request.path, "/api/v1/mapping/") ||
    starts_with(request.path, "/api/v1/navigation/"))
  {
    return not_wired(request.path);
  }
  return {404, "application/json", error_json("endpoint not found: " + request.path)};
}

}  // namespace robot_api_server::application::routing
