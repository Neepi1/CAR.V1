#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"

#include "robot_api_server/features/safety/safety_module.hpp"

namespace robot_api_server::features::safety
{
namespace
{

class SafetyModuleTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok()) {
      int argc = 0;
      rclcpp::init(argc, nullptr);
    }
  }

  static void TearDownTestSuite()
  {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }
};

struct PortObservations
{
  bool floor_blocked{false};
  int admitted_operation_count{0};
  std::uint64_t admitted_epoch{0U};
  std::string admitted_name;
  std::vector<std::string> stop_effects;
};

SafetyModulePorts make_ports(PortObservations & observations)
{
  SafetyModulePorts ports;
  ports.floor_runtime_interlock_response =
    [&observations](const std::string &) -> std::optional<HttpResponse> {
      if (observations.floor_blocked) {
        return HttpResponse{409, "application/json", "{\"blocked\":true}"};
      }
      return std::nullopt;
    };
  ports.run_admitted_operation =
    [&observations](
    const std::uint64_t epoch,
    const std::string & operation,
    const AdmittedSafetyOperation & work) {
      ++observations.admitted_operation_count;
      observations.admitted_epoch = epoch;
      observations.admitted_name = operation;
      return work();
    };
  ports.clear_teleop_command = [&observations]() {
      observations.stop_effects.emplace_back("clear_teleop");
    };
  ports.publish_teleop_zero_burst = [&observations]() {
      observations.stop_effects.emplace_back("teleop_zero");
    };
  ports.publish_terminal_zero_burst = [&observations]() {
      observations.stop_effects.emplace_back("terminal_zero");
    };
  ports.stop_predock_motion = [&observations]() {
      observations.stop_effects.emplace_back("stop_predock");
    };
  return ports;
}

HttpRequest make_request(const std::string & method, const std::string & path)
{
  HttpRequest request;
  request.method = method;
  request.path = path;
  return request;
}

TEST_F(SafetyModuleTest, OwnsStopAndAtomicallyAdmittedResumeRoutes)
{
  auto node = std::make_shared<rclcpp::Node>("safety_module_http_test");
  SafetyModuleConfig config;
  config.ros.estop_topic = "/safety_module_http_test/estop";
  config.ros.status_topic = "/safety_module_http_test/status";
  config.ros.motion_allowed_topic = "/safety_module_http_test/motion_allowed";
  PortObservations observations;
  SafetyModule module(*node, config, make_ports(observations));

  EXPECT_FALSE(module.handle_http(make_request("GET", "/not-safety"), 1U));

  const auto stop = module.handle_http(
    make_request("POST", "/api/v1/safety/stop"), 2U);
  ASSERT_TRUE(stop);
  EXPECT_EQ(stop->status, 202);
  EXPECT_EQ(stop->body, "{\"ok\":true,\"estop\":true}");
  EXPECT_EQ(observations.admitted_operation_count, 0);

  observations.floor_blocked = true;
  const auto blocked_resume = module.handle_http(
    make_request("POST", "/api/v1/safety/resume"), 3U);
  ASSERT_TRUE(blocked_resume);
  EXPECT_EQ(blocked_resume->status, 409);
  EXPECT_EQ(observations.admitted_operation_count, 0);

  observations.floor_blocked = false;
  const auto resume = module.handle_http(
    make_request("POST", "/api/v1/safety/resume"), 42U);
  ASSERT_TRUE(resume);
  EXPECT_EQ(resume->status, 202);
  EXPECT_EQ(resume->body, "{\"ok\":true,\"estop\":false}");
  EXPECT_EQ(observations.admitted_operation_count, 1);
  EXPECT_EQ(observations.admitted_epoch, 42U);
  EXPECT_EQ(observations.admitted_name, "safety_resume");
}

TEST_F(SafetyModuleTest, OwnsUnprovenTerminalStopEffectOrderingAndTopicProjection)
{
  auto node = std::make_shared<rclcpp::Node>("safety_module_latch_test");
  SafetyModuleConfig config;
  config.ros.estop_topic = "/test/estop";
  config.ros.status_topic = "/test/status";
  config.ros.motion_allowed_topic = "/test/motion_allowed";
  PortObservations observations;
  SafetyModule module(*node, config, make_ports(observations));

  module.latch_stop_for_unproven_navigation_terminal();

  EXPECT_EQ(
    observations.stop_effects,
    (std::vector<std::string>{
        "clear_teleop", "teleop_zero", "terminal_zero", "stop_predock"}));
  EXPECT_EQ(module.estop_topic(), "/test/estop");
  EXPECT_EQ(module.status_topic(), "/test/status");
  EXPECT_EQ(module.motion_allowed_topic(), "/test/motion_allowed");
}

}  // namespace
}  // namespace robot_api_server::features::safety
