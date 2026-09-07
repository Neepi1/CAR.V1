#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

#include "robot_api_server/features/safety/safety_ros_adapter.hpp"

namespace safety = robot_api_server::features::safety;
using namespace std::chrono_literals;

TEST(SafetyRosAdapter, KeepsStateResidentAndPublishesEstopIntent)
{
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp::Node>("robot_api_safety_adapter_test");

  safety::SafetyRosAdapterOptions options;
  options.estop_topic = "/robot_api_safety_adapter_test/estop";
  options.status_topic = "/robot_api_safety_adapter_test/status";
  options.motion_allowed_topic =
    "/robot_api_safety_adapter_test/motion_allowed";
  auto adapter = std::make_unique<safety::SafetyRosAdapter>(*node, options);

  const auto state_qos =
    rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
  auto status_publisher = node->create_publisher<std_msgs::msg::String>(
    options.status_topic, state_qos);
  auto motion_publisher = node->create_publisher<std_msgs::msg::Bool>(
    options.motion_allowed_topic, state_qos);
  std::atomic<bool> estop_received{false};
  auto estop_subscription = node->create_subscription<std_msgs::msg::Bool>(
    options.estop_topic, state_qos,
    [&estop_received](const std_msgs::msg::Bool::SharedPtr message) {
      estop_received.store(message->data, std::memory_order_release);
    });

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  std_msgs::msg::String status;
  status.data = "LOCALIZATION_INVALID";
  std_msgs::msg::Bool motion_allowed;
  motion_allowed.data = false;
  status_publisher->publish(status);
  motion_publisher->publish(motion_allowed);

  const auto state_deadline = std::chrono::steady_clock::now() + 2s;
  auto snapshot = adapter->snapshot();
  while (
    std::chrono::steady_clock::now() < state_deadline &&
    (!snapshot.motion_allowed_valid || snapshot.status != status.data))
  {
    executor.spin_some();
    std::this_thread::sleep_for(10ms);
    snapshot = adapter->snapshot();
  }
  EXPECT_EQ(snapshot.status, "LOCALIZATION_INVALID");
  EXPECT_TRUE(snapshot.motion_allowed_valid);
  EXPECT_FALSE(snapshot.motion_allowed);
  EXPECT_TRUE(adapter->hard_block_decision().blocked);

  adapter->publish_estop(true);
  const auto estop_deadline = std::chrono::steady_clock::now() + 2s;
  while (
    std::chrono::steady_clock::now() < estop_deadline &&
    !estop_received.load(std::memory_order_acquire))
  {
    executor.spin_some();
    std::this_thread::sleep_for(10ms);
  }
  EXPECT_TRUE(estop_received.load(std::memory_order_acquire));

  executor.remove_node(node);
  estop_subscription.reset();
  motion_publisher.reset();
  status_publisher.reset();
  adapter.reset();
  node.reset();
  rclcpp::shutdown();
}
