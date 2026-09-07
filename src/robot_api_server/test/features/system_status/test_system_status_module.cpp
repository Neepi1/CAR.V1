#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <yaml-cpp/yaml.h>

#include "robot_api_server/features/system_status/system_status_module.hpp"

namespace system_status = robot_api_server::features::system_status;
using namespace std::chrono_literals;

TEST(SystemStatusModule, OwnsReadOnlyRoutesAndResidentFloorStatus)
{
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp::Node>("robot_api_system_status_module_test");

  system_status::SystemStatusModuleConfig config;
  config.floor_status_topic = "/robot_api_system_status_module_test/floor_status";
  config.map_frame = "map";
  config.base_frame = "base_link";
  config.navigate_to_pose_action = "/navigate_to_pose";
  config.docking_status_topic = "/docking/status";
  config.localization_trigger_service = "/global_localization/trigger";
  config.localization_result_topic = "/global_localization/result";
  config.bms_state_topic = "/battery_state";
  config.maps_root = "/maps";
  config.runtime_maps_dir = "/runtime/maps";
  config.max_http_connections = 8U;

  bool context_blocked = true;
  bool wait_called = false;
  system_status::SystemStatusModulePorts ports;
  ports.status_snapshot = []() {
      system_status::SystemStatusSnapshot snapshot;
      snapshot.mapping_status_json = "{\"active\":false}";
      snapshot.runtime.mode = "NAVIGATION";
      snapshot.runtime.state = "ready";
      snapshot.runtime.navigation_state = "idle";
      snapshot.runtime.healthy = true;
      snapshot.safety.status = "COMMAND_STALE";
      snapshot.safety.motion_allowed_valid = true;
      snapshot.bms.have_state = true;
      snapshot.bms.have_soc = true;
      snapshot.bms.fresh = true;
      snapshot.bms.soc = 55.0;
      snapshot.bms.age_sec = 0.1;
      snapshot.bms.reason = "no_contact";
      snapshot.dock.pre_navigation_check_json = "{\"final_auto_undock_required\":false}";
      snapshot.dock.bms_contact_snapshot_json = "\"contact\":false";
      snapshot.navigation_goal_json = "{\"state\":\"idle\"}";
      snapshot.post_relocalization_settle_json = "{\"complete\":true}";
      snapshot.post_undock_settle_json = "{\"complete\":true}";
      snapshot.subscriptions_json = "{\"resources\":{}}";
      snapshot.bridge_safe_for_goal_start = true;
      snapshot.http_active_connections = 1U;
      return snapshot;
    };
  ports.robot_pose_runtime_context = [&]() {
      system_status::RobotPoseRuntimeContextSnapshot snapshot;
      snapshot.available = true;
      snapshot.blocked = context_blocked;
      snapshot.state = context_blocked ? "starting" : "ready";
      snapshot.startup_stage = context_blocked ? "localization" : "complete";
      snapshot.message = context_blocked ? "waiting" : "ready";
      return snapshot;
    };
  ports.wait_for_robot_pose = [&](std::string &) {
      wait_called = true;
      robot_api_server::RobotPoseSnapshot pose;
      pose.available = true;
      pose.frame_id = "map";
      pose.child_frame_id = "base_link";
      pose.x = 1.0;
      pose.y = 2.0;
      pose.yaw = 0.3;
      pose.stamp_sec = 42.0;
      pose.age_sec = 0.01;
      return pose;
    };
  ports.robot_pose_identity = []() {
      system_status::RobotPoseIdentitySnapshot snapshot;
      snapshot.identity.map_id = "map-1";
      snapshot.identity.floor_id = "F1";
      snapshot.identity.building_id = "B10";
      return snapshot;
    };

  auto module = std::make_unique<system_status::SystemStatusModule>(
    *node, config, std::move(ports));
  module->ensure_floor_status_subscription_active();

  auto publisher = node->create_publisher<std_msgs::msg::String>(
    config.floor_status_topic, rclcpp::QoS(10));
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  const auto discovery_deadline = std::chrono::steady_clock::now() + 2s;
  while (
    std::chrono::steady_clock::now() < discovery_deadline &&
    publisher->get_subscription_count() == 0U)
  {
    executor.spin_some();
    std::this_thread::sleep_for(10ms);
  }
  ASSERT_EQ(publisher->get_subscription_count(), 1U);
  std_msgs::msg::String floor_message;
  floor_message.data = "READY:B10/F1";
  publisher->publish(floor_message);
  const auto receive_deadline = std::chrono::steady_clock::now() + 2s;
  std::optional<robot_api_server::HttpResponse> status;
  while (std::chrono::steady_clock::now() < receive_deadline) {
    executor.spin_some();
    status = module->handle_http({"GET", "/api/v1/status", {}, {}, ""});
    if (status && status->body.find("READY:B10/F1") != std::string::npos) {
      break;
    }
    std::this_thread::sleep_for(10ms);
  }
  ASSERT_TRUE(status.has_value());
  EXPECT_EQ(status->status, 200);
  EXPECT_NE(status->body.find("\"floor_status\":\"READY:B10/F1\""), std::string::npos);
  EXPECT_NE(status->body.find("\"source_topic\":\"/battery_state\""), std::string::npos);
  EXPECT_NE(status->body.find("\"max_connections\":8"), std::string::npos);
  const auto status_document = YAML::Load(status->body);
  ASSERT_TRUE(status_document["ok"].as<bool>());
  EXPECT_EQ(status_document["api_version"].as<std::string>(), "v1");
  EXPECT_EQ(status_document["mode"].as<std::string>(), "NAVIGATION");
  ASSERT_TRUE(status_document["mapping"]);
  ASSERT_TRUE(status_document["navigation"]);
  ASSERT_TRUE(status_document["docking"]);
  ASSERT_TRUE(status_document["safety"]);
  ASSERT_TRUE(status_document["localization"]);
  ASSERT_TRUE(status_document["bms"]);
  ASSERT_TRUE(status_document["subscriptions"]);
  ASSERT_TRUE(status_document["http"]);
  EXPECT_DOUBLE_EQ(status_document["bms"]["soc"].as<double>(), 55.0);
  EXPECT_EQ(status_document["http"]["max_connections"].as<std::size_t>(), 8U);
  EXPECT_FALSE(module->handle_http({"GET", "/api/v1/unknown", {}, {}, ""}).has_value());

  const auto blocked_pose = module->handle_http({"GET", "/api/v1/robot/pose", {}, {}, ""});
  ASSERT_TRUE(blocked_pose.has_value());
  EXPECT_EQ(blocked_pose->status, 503);
  EXPECT_NE(blocked_pose->body.find("runtime map context is not ready"), std::string::npos);
  EXPECT_FALSE(wait_called);

  context_blocked = false;
  const auto ready_pose = module->handle_http({"GET", "/api/v1/robot/pose", {}, {}, ""});
  ASSERT_TRUE(ready_pose.has_value());
  EXPECT_EQ(ready_pose->status, 200);
  EXPECT_TRUE(wait_called);
  EXPECT_NE(ready_pose->body.find("\"map_id\":\"map-1\""), std::string::npos);
  EXPECT_NE(ready_pose->body.find("\"building_id\":\"B10\""), std::string::npos);
  const auto pose_document = YAML::Load(ready_pose->body);
  EXPECT_TRUE(pose_document["ok"].as<bool>());
  EXPECT_EQ(pose_document["frame_id"].as<std::string>(), "map");
  EXPECT_EQ(pose_document["map_id"].as<std::string>(), "map-1");

  executor.remove_node(node);
  publisher.reset();
  module.reset();
  node.reset();
  rclcpp::shutdown();
}
