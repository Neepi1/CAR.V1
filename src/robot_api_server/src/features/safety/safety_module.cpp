#include "robot_api_server/features/safety/safety_module.hpp"

#include <stdexcept>
#include <utility>

#include "rclcpp/rclcpp.hpp"

namespace robot_api_server::features::safety
{
namespace
{

void require_ports(const SafetyModulePorts & ports)
{
  if (!ports.floor_runtime_interlock_response ||
    !ports.run_admitted_operation ||
    !ports.clear_teleop_command ||
    !ports.publish_teleop_zero_burst ||
    !ports.publish_terminal_zero_burst ||
    !ports.stop_predock_motion)
  {
    throw std::invalid_argument("safety module requires every cross-domain port");
  }
}

}  // namespace

struct SafetyModule::Impl
{
  Impl(
    rclcpp::Node & module_node,
    SafetyModuleConfig module_config,
    SafetyModulePorts module_ports)
  : node(module_node),
    config(std::move(module_config)),
    ports(std::move(module_ports)),
    ros(node, config.ros)
  {
    require_ports(ports);
  }

  std::optional<HttpResponse> handle_http(
    const HttpRequest & request,
    const std::uint64_t motion_admission_epoch)
  {
    if (request.method == "POST" && request.path == "/api/v1/safety/stop") {
      return publish_estop(true);
    }
    if (request.method != "POST" || request.path != "/api/v1/safety/resume") {
      return std::nullopt;
    }

    if (const auto blocked = ports.floor_runtime_interlock_response("safety_resume")) {
      return *blocked;
    }
    return ports.run_admitted_operation(
      motion_admission_epoch,
      "safety_resume",
      [this]() {return publish_estop(false);});
  }

  HttpResponse publish_estop(const bool active)
  {
    ros.publish_estop(active);
    return {202, "application/json", safety_estop_response_json(active)};
  }

  void latch_stop_for_unproven_navigation_terminal()
  {
    ros.publish_estop(true);
    ports.clear_teleop_command();
    ports.publish_teleop_zero_burst();
    ports.publish_terminal_zero_burst();
    ports.stop_predock_motion();
    RCLCPP_ERROR(
      node.get_logger(),
      "Safety estop latched because an accepted Nav2 goal has no proven "
      "terminal result; explicit safety resume is required after inspection");
  }

  rclcpp::Node & node;
  SafetyModuleConfig config;
  SafetyModulePorts ports;
  SafetyRosAdapter ros;
};

SafetyModule::SafetyModule(
  rclcpp::Node & node,
  SafetyModuleConfig config,
  SafetyModulePorts ports)
: impl_(std::make_unique<Impl>(node, std::move(config), std::move(ports)))
{
}

SafetyModule::~SafetyModule() = default;

std::optional<HttpResponse> SafetyModule::handle_http(
  const HttpRequest & request,
  const std::uint64_t motion_admission_epoch)
{
  return impl_->handle_http(request, motion_admission_epoch);
}

SafetyStateSnapshot SafetyModule::snapshot() const
{
  return impl_->ros.snapshot();
}

bool SafetyModule::hard_blocked(std::string & detail) const
{
  const auto decision = impl_->ros.hard_block_decision();
  detail = decision.detail;
  return decision.blocked;
}

void SafetyModule::latch_stop_for_unproven_navigation_terminal()
{
  impl_->latch_stop_for_unproven_navigation_terminal();
}

std::string SafetyModule::estop_topic() const
{
  return impl_->config.ros.estop_topic;
}

std::string SafetyModule::status_topic() const
{
  return impl_->config.ros.status_topic;
}

std::string SafetyModule::motion_allowed_topic() const
{
  return impl_->config.ros.motion_allowed_topic;
}

}  // namespace robot_api_server::features::safety
