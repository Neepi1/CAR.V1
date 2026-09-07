#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <yaml-cpp/yaml.h>

#include "robot_api_server/application/subscriptions/subscription_module.hpp"

namespace subscriptions = robot_api_server::application::subscriptions;
using namespace std::chrono_literals;

namespace
{

class SubscriptionModuleTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestSuite()
  {
    rclcpp::shutdown();
  }

  void SetUp() override
  {
    node_ = std::make_shared<rclcpp::Node>(
      "subscription_module_test_" + std::to_string(++node_sequence_));
  }

  void TearDown() override
  {
    module_.reset();
    node_.reset();
  }

  static inline int node_sequence_{0};
  std::shared_ptr<rclcpp::Node> node_;
  std::unique_ptr<subscriptions::SubscriptionModule> module_;
};

TEST_F(SubscriptionModuleTest, OwnsCompatibilityHttpLifecycleAndResourceTransitions)
{
  int status_resident_count = 0;
  int tf_resident_count = 0;
  int teleop_clear_count = 0;
  std::vector<bool> live_map_transitions;

  subscriptions::SubscriptionModuleConfig config;
  config.default_ttl_ms = 10000;
  config.max_ttl_ms = 60000;
  config.scan_topic = "/subscription_module_test/scan";
  subscriptions::SubscriptionModulePorts ports;
  ports.ensure_status_resident = [&]() {++status_resident_count;};
  ports.set_live_map_page_active = [&](const bool active) {
      live_map_transitions.push_back(active);
    };
  ports.ensure_tf_resident = [&]() {++tf_resident_count;};
  ports.clear_teleop_command = [&]() {++teleop_clear_count;};
  module_ = std::make_unique<subscriptions::SubscriptionModule>(
    *node_, config, std::move(ports));

  EXPECT_FALSE(module_->handle_http({"GET", "/api/v1/unknown", {}, {}, ""}).has_value());
  EXPECT_EQ(module_->max_ttl_ms(), 60000);

  const auto acquire = module_->handle_http(
    {
      "POST",
      "/api/v1/subscriptions/acquire",
      {},
      {},
      R"({"clientId":"app_device_001","resources":["tf","scan","live_map","status","teleop","scan"],"ttl_ms":90000})"});
  ASSERT_TRUE(acquire.has_value());
  ASSERT_EQ(acquire->status, 200);
  const auto acquire_json = YAML::Load(acquire->body);
  EXPECT_TRUE(acquire_json["ok"].as<bool>());
  EXPECT_EQ(acquire_json["action"].as<std::string>(), "acquire");
  EXPECT_EQ(acquire_json["client_id"].as<std::string>(), "app_device_001");
  EXPECT_EQ(acquire_json["lease_id"].as<std::string>(), "app_device_001");
  EXPECT_EQ(acquire_json["client_id_source"].as<std::string>(), "clientId");
  EXPECT_FALSE(acquire_json["refreshed"].as<bool>());
  EXPECT_EQ(acquire_json["ttl_ms"].as<int>(), 60000);
  EXPECT_EQ(
    acquire_json["resources"].as<std::vector<std::string>>(),
    (std::vector<std::string>{"live_map", "scan", "status", "teleop", "tf"}));
  EXPECT_EQ(status_resident_count, 1);
  EXPECT_EQ(tf_resident_count, 1);
  EXPECT_EQ(teleop_clear_count, 0);
  EXPECT_EQ(live_map_transitions, (std::vector<bool>{true}));
  EXPECT_TRUE(module_->scan_snapshot().active);

  const auto heartbeat = module_->handle_http(
    {
      "POST",
      "/api/v1/subscriptions/heartbeat",
      {},
      {},
      R"({"client_id":"app_device_001"})"});
  ASSERT_TRUE(heartbeat.has_value());
  ASSERT_EQ(heartbeat->status, 200);
  const auto heartbeat_json = YAML::Load(heartbeat->body);
  EXPECT_TRUE(heartbeat_json["refreshed"].as<bool>());
  EXPECT_EQ(heartbeat_json["ttl_ms"].as<int>(), 10000);
  EXPECT_EQ(
    heartbeat_json["resources"].as<std::vector<std::string>>(),
    (std::vector<std::string>{"live_map", "scan", "status", "teleop", "tf"}));
  EXPECT_EQ(status_resident_count, 1);
  EXPECT_EQ(tf_resident_count, 1);
  EXPECT_EQ(live_map_transitions, (std::vector<bool>{true}));

  const auto release = module_->handle_http(
    {
      "POST",
      "/api/v1/subscriptions/release",
      {},
      {},
      R"({"client_id":"app_device_001"})"});
  ASSERT_TRUE(release.has_value());
  ASSERT_EQ(release->status, 200);
  const auto release_json = YAML::Load(release->body);
  EXPECT_EQ(release_json["ttl_ms"].as<int>(), 0);
  EXPECT_EQ(release_json["resources"].size(), 0U);
  EXPECT_EQ(status_resident_count, 2);
  EXPECT_EQ(tf_resident_count, 2);
  EXPECT_EQ(teleop_clear_count, 1);
  EXPECT_EQ(live_map_transitions, (std::vector<bool>{true, false}));
  const auto released_scan = module_->scan_snapshot();
  EXPECT_FALSE(released_scan.active);
  EXPECT_FALSE(released_scan.available);

  const auto empty_heartbeat = module_->handle_http(
    {
      "POST",
      "/api/v1/subscriptions/heartbeat",
      {},
      {},
      R"({"lease_id":"unknown_client"})"});
  ASSERT_TRUE(empty_heartbeat.has_value());
  ASSERT_EQ(empty_heartbeat->status, 200);
  const auto empty_heartbeat_json = YAML::Load(empty_heartbeat->body);
  EXPECT_FALSE(empty_heartbeat_json["refreshed"].as<bool>());
  EXPECT_EQ(empty_heartbeat_json["ttl_ms"].as<int>(), 0);
  EXPECT_EQ(empty_heartbeat_json["resources"].size(), 0U);

  const auto missing_resources = module_->handle_http(
    {
      "POST", "/api/v1/subscriptions/acquire", {}, {}, R"({"client_id":"empty"})"});
  ASSERT_TRUE(missing_resources.has_value());
  EXPECT_EQ(missing_resources->status, 400);
  const auto unsupported = module_->handle_http(
    {
      "POST",
      "/api/v1/subscriptions/acquire",
      {},
      {},
      R"({"client_id":"bad","resources":["pointcloud"]})"});
  ASSERT_TRUE(unsupported.has_value());
  EXPECT_EQ(unsupported->status, 400);
}

TEST_F(SubscriptionModuleTest, CreatesAndClearsThePageScopedScanCache)
{
  subscriptions::SubscriptionModuleConfig config;
  config.scan_topic = "/subscription_module_test/scan_cache";
  module_ = std::make_unique<subscriptions::SubscriptionModule>(
    *node_, config, subscriptions::SubscriptionModulePorts{});
  auto publisher = node_->create_publisher<sensor_msgs::msg::LaserScan>(
    config.scan_topic, rclcpp::QoS(10));
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node_);

  module_->acquire("scan_test", {"scan"}, 10s);
  const auto discovery_deadline = std::chrono::steady_clock::now() + 2s;
  while (
    publisher->get_subscription_count() == 0U &&
    std::chrono::steady_clock::now() < discovery_deadline)
  {
    executor.spin_some();
    std::this_thread::sleep_for(10ms);
  }
  ASSERT_EQ(publisher->get_subscription_count(), 1U);

  sensor_msgs::msg::LaserScan message;
  message.header.frame_id = "laser";
  message.angle_min = -1.0F;
  message.angle_max = 1.0F;
  message.ranges = {1.0F, 2.0F, 3.0F};
  publisher->publish(message);
  const auto receive_deadline = std::chrono::steady_clock::now() + 2s;
  auto scan = module_->scan_snapshot();
  while (!scan.available && std::chrono::steady_clock::now() < receive_deadline) {
    executor.spin_some();
    scan = module_->scan_snapshot();
    std::this_thread::sleep_for(10ms);
  }
  ASSERT_TRUE(scan.active);
  ASSERT_TRUE(scan.available);
  EXPECT_TRUE(scan.fresh);
  EXPECT_EQ(scan.frame_id, "laser");
  EXPECT_EQ(scan.range_count, 3U);
  EXPECT_DOUBLE_EQ(scan.angle_min, -1.0);
  EXPECT_DOUBLE_EQ(scan.angle_max, 1.0);

  module_->release("scan_test", {"scan"});
  scan = module_->scan_snapshot();
  EXPECT_FALSE(scan.active);
  EXPECT_FALSE(scan.available);
  EXPECT_EQ(scan.range_count, 0U);
  EXPECT_TRUE(scan.frame_id.empty());

  executor.remove_node(node_);
  publisher.reset();
}

TEST_F(SubscriptionModuleTest, PreservesFallbackIdentityAndMinimumHttpTtl)
{
  module_ = std::make_unique<subscriptions::SubscriptionModule>(
    *node_, subscriptions::SubscriptionModuleConfig{},
    subscriptions::SubscriptionModulePorts{});

  const auto acquire = module_->handle_http(
    {
      "POST",
      "/api/v1/subscriptions/acquire",
      {},
      {},
      R"({"resource":"status","ttl_ms":1})"});
  ASSERT_TRUE(acquire.has_value());
  ASSERT_EQ(acquire->status, 200);
  const auto response = YAML::Load(acquire->body);
  EXPECT_EQ(response["client_id"].as<std::string>(), "http:compat-default");
  EXPECT_EQ(response["lease_id"].as<std::string>(), "http:compat-default");
  EXPECT_EQ(response["client_id_source"].as<std::string>(), "fallback");
  EXPECT_EQ(response["ttl_ms"].as<int>(), 1000);
  EXPECT_EQ(
    response["resources"].as<std::vector<std::string>>(),
    (std::vector<std::string>{"status"}));

  const auto release = module_->handle_http(
    {
      "POST", "/api/v1/subscriptions/release", {}, {}, "{}"});
  ASSERT_TRUE(release.has_value());
  EXPECT_EQ(release->status, 200);
  EXPECT_NE(
    module_->snapshot_json().find(
      "\"status\":{\"active\":false,\"ref_count\":0}"),
    std::string::npos);
}

TEST_F(SubscriptionModuleTest, NormalizesConfiguredTtlBeforeServingOrProjectingIt)
{
  subscriptions::SubscriptionModuleConfig config;
  config.default_ttl_ms = 500;
  config.max_ttl_ms = 100;
  module_ = std::make_unique<subscriptions::SubscriptionModule>(
    *node_, config, subscriptions::SubscriptionModulePorts{});

  EXPECT_EQ(module_->max_ttl_ms(), 1000);
  const auto acquire = module_->handle_http(
    {
      "POST",
      "/api/v1/subscriptions/acquire",
      {},
      {},
      R"({"client_id":"bounds","resource":"status","ttl_ms":5000})"});
  ASSERT_TRUE(acquire.has_value());
  ASSERT_EQ(acquire->status, 200);
  EXPECT_EQ(YAML::Load(acquire->body)["ttl_ms"].as<int>(), 1000);
}

TEST_F(SubscriptionModuleTest, ExpiresLostClientAndRunsFinalResourceCleanup)
{
  int teleop_clear_count = 0;
  std::vector<bool> live_map_transitions;
  subscriptions::SubscriptionModuleConfig config;
  config.expiry_period = 10ms;
  subscriptions::SubscriptionModulePorts ports;
  ports.set_live_map_page_active = [&](const bool active) {
      live_map_transitions.push_back(active);
    };
  ports.clear_teleop_command = [&]() {++teleop_clear_count;};
  module_ = std::make_unique<subscriptions::SubscriptionModule>(
    *node_, config, std::move(ports));
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node_);

  module_->acquire("lost_app", {"live_map", "teleop"}, 40ms);
  ASSERT_EQ(live_map_transitions, (std::vector<bool>{true}));
  ASSERT_EQ(teleop_clear_count, 0);

  const auto expiry_deadline = std::chrono::steady_clock::now() + 1s;
  while (
    module_->snapshot_json().find(
      "\"live_map\":{\"active\":false,\"ref_count\":0}") == std::string::npos &&
    std::chrono::steady_clock::now() < expiry_deadline)
  {
    executor.spin_some();
    std::this_thread::sleep_for(5ms);
  }
  // Both resources have returned to zero, so the page-owned map is released
  // and the last teleop lease performs its mandatory command cleanup.
  const auto snapshot = YAML::Load(module_->snapshot_json());
  EXPECT_EQ(snapshot["resources"]["live_map"]["ref_count"].as<int>(), 0);
  EXPECT_EQ(snapshot["resources"]["teleop"]["ref_count"].as<int>(), 0);
  EXPECT_EQ(live_map_transitions, (std::vector<bool>{true, false}));
  EXPECT_EQ(teleop_clear_count, 1);

  executor.remove_node(node_);
}

}  // namespace
