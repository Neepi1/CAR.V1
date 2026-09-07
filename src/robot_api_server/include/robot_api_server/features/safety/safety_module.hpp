#pragma once

#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <string>

#include "robot_api_server/features/safety/safety_ros_adapter.hpp"
#include "robot_api_server/infrastructure/http/http_common.hpp"

namespace rclcpp
{
class Node;
}

namespace robot_api_server::features::safety
{

struct SafetyModuleConfig
{
  SafetyRosAdapterOptions ros;
};

using AdmittedSafetyOperation = std::function<HttpResponse()>;

struct SafetyModulePorts
{
  std::function<std::optional<HttpResponse>(const std::string & operation)>
  floor_runtime_interlock_response;
  std::function<HttpResponse(
      std::uint64_t motion_admission_epoch,
      const std::string & operation,
      const AdmittedSafetyOperation & work)>
  run_admitted_operation;
  std::function<void()> clear_teleop_command;
  std::function<void()> publish_teleop_zero_burst;
  std::function<void()> publish_terminal_zero_burst;
  std::function<void()> stop_predock_motion;
};

// Owns the complete App-facing safety vertical slice while robot_safety keeps
// final velocity arbitration. Cross-domain admission and stop effects are
// explicit ports so their ordering remains testable and atomic.
class SafetyModule
{
public:
  SafetyModule(
    rclcpp::Node & node,
    SafetyModuleConfig config,
    SafetyModulePorts ports);
  ~SafetyModule();

  SafetyModule(const SafetyModule &) = delete;
  SafetyModule & operator=(const SafetyModule &) = delete;

  std::optional<HttpResponse> handle_http(
    const HttpRequest & request,
    std::uint64_t motion_admission_epoch);
  SafetyStateSnapshot snapshot() const;
  bool hard_blocked(std::string & detail) const;
  void latch_stop_for_unproven_navigation_terminal();

  std::string estop_topic() const;
  std::string status_topic() const;
  std::string motion_allowed_topic() const;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace robot_api_server::features::safety
