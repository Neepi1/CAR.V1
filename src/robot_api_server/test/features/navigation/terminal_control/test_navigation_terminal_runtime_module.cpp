#include <gtest/gtest.h>

#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "nav2_msgs/msg/speed_limit.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rcl_interfaces/msg/log.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"

#include \
  "robot_api_server/features/navigation/terminal_control/navigation_terminal_runtime_module.hpp"

namespace navigation = robot_api_server::features::navigation;
using namespace std::chrono_literals;

namespace
{

navigation::NavigationTerminalRuntimePorts complete_ports(
  robot_api_server::RobotPoseSnapshot * current_pose = nullptr)
{
  navigation::NavigationTerminalRuntimePorts ports;
  ports.running = []() {return true;};
  ports.cancel_requested = [](std::uint64_t, std::string &) {return false;};
  ports.safety_hard_blocked = [](std::string &) {return false;};
  ports.verify_final_pose = [](
    const robot_api_server::StoredPose &, bool) {
      return navigation::FinalPoseCheck{};
    };
  ports.update_final_pose = [](
    std::uint64_t, const navigation::FinalPoseCheck &, const std::string &) {};
  ports.set_job_phase = [](std::uint64_t, const std::string &, const std::string &) {};
  ports.publish_motion_mode = [](const std::string &) {};
  ports.current_robot_pose = [current_pose]() {
      return current_pose ? *current_pose : robot_api_server::RobotPoseSnapshot{};
    };
  ports.pose_in_frame = [](const std::string &) {
      return navigation::TerminalFramePoseSnapshot{};
    };
  ports.dock_contact_blocked = [](std::string &) {return false;};
  return ports;
}

navigation::NavigationTerminalRuntimeConfig test_config(const int sequence)
{
  const std::string suffix = std::to_string(sequence);
  navigation::NavigationTerminalRuntimeConfig config;
  config.command_topic = "/test/terminal_command_" + suffix;
  config.speed_limit_topic = "/test/terminal_speed_limit_" + suffix;
  config.reverse_enable_topic = "/test/terminal_reverse_" + suffix;
  config.mode_controller_status_topic = "/test/terminal_mode_status_" + suffix;
  config.actual_stop_odom_topic = "/test/terminal_odom_" + suffix;
  config.local_costmap_topic = "/test/terminal_costmap_" + suffix;
  config.zero_command_count = 1;
  config.yaw_actual_wz_stable_samples = 1;
  config.yaw_actual_stop_timeout_ms = 100;
  config.terminal_settle_stable_duration_sec = 0.0;
  config.terminal_settle_timeout_sec = 0.1;
  return config;
}

class NavigationTerminalRuntimeModuleTest : public ::testing::Test
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
      "navigation_terminal_runtime_test_" + std::to_string(++sequence_));
  }

  void TearDown() override
  {
    module_.reset();
    node_.reset();
  }

  void spin_until(const std::function<bool()> & predicate)
  {
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node_);
    const auto deadline = std::chrono::steady_clock::now() + 2s;
    while (!predicate() && std::chrono::steady_clock::now() < deadline) {
      executor.spin_some();
      std::this_thread::sleep_for(5ms);
    }
    executor.remove_node(node_);
  }

  static inline int sequence_{0};
  std::shared_ptr<rclcpp::Node> node_;
  navigation::NavigationTerminalControl terminal_control_{
    navigation::TerminalControlConfig{}};
  std::unique_ptr<navigation::NavigationTerminalRuntimeModule> module_;
};

TEST_F(NavigationTerminalRuntimeModuleTest, RejectsIncompleteIntegrationPorts)
{
  EXPECT_THROW(
    module_ = std::make_unique<navigation::NavigationTerminalRuntimeModule>(
      *node_,
      terminal_control_,
      test_config(sequence_),
      navigation::NavigationTerminalRuntimePorts{}),
    std::invalid_argument);
}

TEST_F(NavigationTerminalRuntimeModuleTest, OwnsStatusStopAndCostmapEvidence)
{
  const auto config = test_config(sequence_);
  module_ = std::make_unique<navigation::NavigationTerminalRuntimeModule>(
    *node_, terminal_control_, config, complete_ports());

  auto mode_pub = node_->create_publisher<std_msgs::msg::String>(
    config.mode_controller_status_topic, rclcpp::QoS(10));
  auto odom_pub = node_->create_publisher<nav_msgs::msg::Odometry>(
    config.actual_stop_odom_topic, rclcpp::QoS(20));
  auto costmap_pub = node_->create_publisher<nav_msgs::msg::OccupancyGrid>(
    config.local_costmap_topic,
    rclcpp::QoS(rclcpp::KeepLast(5)).reliable().transient_local());
  auto rosout_pub = node_->create_publisher<rcl_interfaces::msg::Log>(
    "/rosout", rclcpp::QoS(100));
  spin_until(
    [&]() {
      return mode_pub->get_subscription_count() >= 1U &&
      odom_pub->get_subscription_count() >= 1U &&
      costmap_pub->get_subscription_count() >= 1U &&
      rosout_pub->get_subscription_count() >= 1U;
    });

  std_msgs::msg::String mode;
  mode.data =
    "{\"actual_motion_mode\":{\"available\":true,\"fresh\":true,\"code\":0},"
    "\"mode_aligned\":true}";
  mode_pub->publish(mode);
  nav_msgs::msg::Odometry odom;
  odom.twist.twist.linear.x = 0.0;
  odom.twist.twist.linear.y = 0.0;
  odom.twist.twist.angular.z = 0.0;
  odom_pub->publish(odom);
  nav_msgs::msg::OccupancyGrid costmap;
  costmap.header.frame_id = "base_link";
  costmap_pub->publish(costmap);
  rcl_interfaces::msg::Log log;
  log.name = "controller_server";
  log.msg = "Message Filter dropping local_costmap observation";
  rosout_pub->publish(log);
  spin_until(
    [&]() {
      return module_->mode_controller_status_snapshot().available &&
      module_->local_costmap_update_count() == 1U &&
      module_->local_costmap_message_filter_drop_count() == 1U;
    });

  const auto status = module_->mode_controller_status_snapshot();
  EXPECT_TRUE(status.actual_available);
  EXPECT_TRUE(status.actual_fresh);
  EXPECT_EQ(status.actual_motion_mode_code, 0);
  EXPECT_TRUE(status.mode_aligned);
  EXPECT_EQ(module_->local_costmap_update_count(), 1U);
  EXPECT_EQ(module_->local_costmap_message_filter_drop_count(), 1U);
  EXPECT_NE(
    module_->last_local_costmap_message_filter_drop_text().find("local_costmap"),
    std::string::npos);

  std::string yaw_detail;
  EXPECT_TRUE(module_->wait_for_yaw_actual_stop("test", yaw_detail)) << yaw_detail;
  std::string settle_detail;
  EXPECT_TRUE(module_->wait_for_actual_stop("test", settle_detail)) << settle_detail;
  const auto context = module_->terminal_costmap_context(std::chrono::steady_clock::now());
  ASSERT_TRUE(context.grid);
  EXPECT_EQ(context.grid_frame, "base_link");
  EXPECT_TRUE(context.robot_pose_available);
}

TEST_F(NavigationTerminalRuntimeModuleTest, PublishesCommandsLimitsAndReverseHysteresis)
{
  const auto config = test_config(sequence_);
  robot_api_server::RobotPoseSnapshot pose;
  pose.available = true;
  pose.frame_id = "map";
  pose.age_sec = 0.0;
  pose.x = 0.0;
  pose.y = 0.0;

  std::mutex received_mutex;
  std::vector<geometry_msgs::msg::Twist> commands;
  std::vector<nav2_msgs::msg::SpeedLimit> limits;
  std::vector<bool> permits;
  auto command_sub = node_->create_subscription<geometry_msgs::msg::Twist>(
    config.command_topic,
    rclcpp::QoS(rclcpp::KeepLast(1)).reliable(),
    [&](const geometry_msgs::msg::Twist::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(received_mutex);
      commands.push_back(*msg);
    });
  auto limit_sub = node_->create_subscription<nav2_msgs::msg::SpeedLimit>(
    config.speed_limit_topic,
    rclcpp::QoS(1).reliable().transient_local(),
    [&](const nav2_msgs::msg::SpeedLimit::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(received_mutex);
      limits.push_back(*msg);
    });
  auto permit_sub = node_->create_subscription<std_msgs::msg::Bool>(
    config.reverse_enable_topic,
    rclcpp::QoS(1),
    [&](const std_msgs::msg::Bool::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(received_mutex);
      permits.push_back(msg->data);
    });

  module_ = std::make_unique<navigation::NavigationTerminalRuntimeModule>(
    *node_, terminal_control_, config, complete_ports(&pose));
  spin_until(
    [&]() {
      return command_sub->get_publisher_count() == 1U &&
      limit_sub->get_publisher_count() == 1U &&
      permit_sub->get_publisher_count() == 1U;
    });

  geometry_msgs::msg::Twist command;
  command.angular.z = 0.17;
  module_->publish_command(command);
  robot_api_server::StoredPose target;
  target.x = 0.20;
  target.y = 0.0;
  module_->publish_speed_limit_for_goal(target);
  bool permit_active = false;
  std::chrono::steady_clock::time_point next_refresh_at{};
  module_->update_reverse_permit_for_goal(
    target, permit_active, next_refresh_at, "test_enter");
  EXPECT_TRUE(permit_active);
  spin_until(
    [&]() {
      std::lock_guard<std::mutex> lock(received_mutex);
      return !commands.empty() && !limits.empty() && !permits.empty();
    });

  pose.x = -0.50;
  next_refresh_at = {};
  module_->update_reverse_permit_for_goal(
    target, permit_active, next_refresh_at, "test_exit");
  EXPECT_FALSE(permit_active);
  spin_until(
    [&]() {
      std::lock_guard<std::mutex> lock(received_mutex);
      return permits.size() >= 2U;
    });

  std::lock_guard<std::mutex> lock(received_mutex);
  ASSERT_FALSE(commands.empty());
  EXPECT_DOUBLE_EQ(commands.back().angular.z, 0.17);
  ASSERT_FALSE(limits.empty());
  EXPECT_DOUBLE_EQ(
    limits.back().speed_limit,
    terminal_control_.speed_limit_for_distance(0.20));
  ASSERT_GE(permits.size(), 2U);
  EXPECT_TRUE(permits.front());
  EXPECT_FALSE(permits.back());
}

}  // namespace
