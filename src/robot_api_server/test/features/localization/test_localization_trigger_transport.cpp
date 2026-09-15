#include <atomic>
#include <chrono>
#include <cstdlib>
#include <thread>

#include "gtest/gtest.h"
#include "robot_api_server/features/localization/localization_module.hpp"
#include "robot_interfaces/msg/localization_trigger_status.hpp"
#include "std_msgs/msg/string.hpp"
#include "robot_interfaces/srv/trigger_localization_tracked.hpp"

using namespace std::chrono_literals;
namespace localization = robot_api_server::features::localization;
using Tracked = robot_interfaces::srv::TriggerLocalizationTracked;
using Status = robot_interfaces::msg::LocalizationTriggerStatus;

class TriggerTransport : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    ASSERT_STREQ(std::getenv("ROS_DOMAIN_ID"), "217");
    ASSERT_STREQ(std::getenv("ROS_LOCALHOST_ONLY"), "1");
    rclcpp::init(0, nullptr);
  }
  static void TearDownTestSuite() {if (rclcpp::ok()) {rclcpp::shutdown();}}
  void SetUp() override
  {
    node = std::make_shared<rclcpp::Node>("trigger_transport_fixture");
    group = node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    publisher = node->create_publisher<Status>(
      "/global_localization/trigger_status", rclcpp::QoS(128).reliable().transient_local());
    service = node->create_service<Tracked>("/global_localization/trigger/tracked",
      [this](const Tracked::Request::SharedPtr request, Tracked::Response::SharedPtr response) {
        ++dispatches;
        last_request = request->request_id;
        if (delay_response) {std::this_thread::sleep_for(900ms);}
        response->status.request_id = request->request_id;
        response->status.state = return_unknown ? Status::UNKNOWN : Status::SUCCEEDED;
        response->status.code = return_unknown ? "LOCALIZATION_OUTCOME_UNKNOWN" : "LOCALIZATION_APPLIED";
        response->status.outcome_known = !return_unknown;
        response->status.baseline_explicit_sequence = 3;
        response->status.accepted_explicit_sequence = return_unknown ? 0 : 4;
      }, rmw_qos_profile_services_default, group);
    localization::LocalizationModulePorts ports;
    ports.operation_blocked = [this](const auto &, auto &) {return pending.load() != 0;};
    ports.side_effect_started = [this] {++pending;};
    ports.side_effect_resolved = [this] {--pending;};
    ports.run_admitted_operation = [](auto, const auto &, const auto & operation) {return operation();};
    ports.wait_for_settle = [this](auto, const auto &, const auto &) {
      ++settle_calls;
      return localization::RelocalizationSettleResult{
        navigation_settle_ok, navigation_settle_ok ? "" : "NAVIGATION_NOT_READY", "settle result"};
    };
    ports.settle_state_json = [] {return "{}";};
    localization::LocalizationModuleConfig config;
    config.service_timeout_sec = 0.15;
    config.trigger_service_timeout_sec = 0.3;
    config.default_relocalization_wait_sec = 0.15;
    config.manual_amcl_refine_enabled = false;
    config.manual_amcl_refine_required = false;
    config.manual_amcl_refine_timeout_sec = 0.8;
    module = std::make_unique<localization::LocalizationModule>(*node, group, config, ports);
    executor.add_node(node);
    spin = std::thread([this] {executor.spin();});
    std::this_thread::sleep_for(200ms);
  }
  void TearDown() override
  {
    executor.cancel();
    if (spin.joinable()) {spin.join();}
    executor.remove_node(node);
    module.reset();
  }
  bool wait_resolved()
  {
    const auto deadline = std::chrono::steady_clock::now() + 3s;
    while (pending != 0 && std::chrono::steady_clock::now() < deadline) {
      std::this_thread::sleep_for(20ms);
    }
    return pending == 0;
  }
  rclcpp::Node::SharedPtr node;
  rclcpp::CallbackGroup::SharedPtr group;
  rclcpp::Publisher<Status>::SharedPtr publisher;
  rclcpp::Service<Tracked>::SharedPtr service;
  rclcpp::executors::MultiThreadedExecutor executor{rclcpp::ExecutorOptions(), 2};
  std::unique_ptr<localization::LocalizationModule> module;
  std::thread spin;
  std::atomic<int> pending{0}, dispatches{0};
  bool delay_response{false}, return_unknown{false};
  bool navigation_settle_ok{true};
  int settle_calls{0};
  std::string last_request;
};

TEST_F(TriggerTransport, LateServiceResponseClearsEvidenceAfterHttpWaitExpires)
{
  delay_response = true;
  std::string detail;
  EXPECT_FALSE(module->trigger_localization_and_wait_for_result("late", detail));
  EXPECT_EQ(pending, 1);
  EXPECT_TRUE(wait_resolved());
  EXPECT_EQ(dispatches, 1);
}

TEST_F(TriggerTransport, UnknownReplyKeepsEvidenceUntilMatchingTerminalStatus)
{
  return_unknown = true;
  std::string detail;
  EXPECT_FALSE(module->trigger_localization_and_wait_for_result("unknown", detail));
  EXPECT_EQ(pending, 1);
  Status status;
  status.request_id = "foreign-id";
  status.state = Status::SUCCEEDED;
  status.outcome_known = true;
  status.baseline_explicit_sequence = 3;
  status.accepted_explicit_sequence = 4;
  publisher->publish(status);
  std::this_thread::sleep_for(100ms);
  EXPECT_EQ(pending, 1);
  status.request_id = last_request;
  publisher->publish(status);
  publisher->publish(status);
  EXPECT_TRUE(wait_resolved());
  std::this_thread::sleep_for(100ms);
  EXPECT_EQ(pending, 0);
  EXPECT_EQ(dispatches, 1);
}

TEST_F(TriggerTransport, AppliedRefinementDoesNotRequireNavigationPermission)
{
  auto bridge = node->create_publisher<std_msgs::msg::String>("/localization/bridge_status", 10);
  auto timer = node->create_wall_timer(20ms, [&] {
    std_msgs::msg::String msg;
    msg.data = R"({"amcl_post_isaac_refined_sequence":4,"safe_for_goal_start":false,"correction_active":false,"current_sequence":8,"target_sequence":8,"last_published_sequence":8})";
    bridge->publish(msg);
  });
  std::this_thread::sleep_for(150ms);
  std::string detail;
  EXPECT_TRUE(module->wait_for_manual_relocalization_amcl_refine(4, true, true, detail)) << detail;
  EXPECT_FALSE(module->bridge_status_snapshot().safe_for_goal_start);
}

TEST_F(TriggerTransport, LegacyNavigationPostcheckDoesNotEraseLocalizationResult)
{
  navigation_settle_ok = false;
  robot_api_server::HttpRequest request;
  request.method = "POST";
  request.path = "/api/v1/localization/trigger";
  request.body = R"({"wait_for_settle":true,"amcl_refine":false})";
  const auto response = module->handle_http(request, 0);
  ASSERT_TRUE(response.has_value());
  EXPECT_EQ(response->status, 503);  // Preserve legacy combined-operation HTTP semantics.
  EXPECT_NE(response->body.find("\"localization_complete\":true"), std::string::npos);
  EXPECT_NE(response->body.find("\"post_relocalization_settle_ok\":false"), std::string::npos);
  EXPECT_EQ(settle_calls, 1);
}

TEST_F(TriggerTransport, PlainLocalizationDoesNotInvokeNavigationPostcheck)
{
  navigation_settle_ok = false;
  robot_api_server::HttpRequest request;
  request.method = "POST";
  request.path = "/api/v1/localization/trigger";
  request.body = R"({"amcl_refine":false})";
  const auto response = module->handle_http(request, 0);
  ASSERT_TRUE(response.has_value());
  EXPECT_EQ(response->status, 200);
  EXPECT_EQ(settle_calls, 0);
}
