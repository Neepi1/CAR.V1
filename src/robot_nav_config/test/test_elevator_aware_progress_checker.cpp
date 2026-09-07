#include <chrono>
#include <cmath>
#include <memory>
#include <thread>

#include <gtest/gtest.h>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "robot_nav_config/elevator_aware_progress_checker.hpp"
#include "robot_nav_config/elevator_scoped_progress_state.hpp"
#include "std_msgs/msg/u_int8.hpp"

namespace {

using namespace std::chrono_literals;

geometry_msgs::msg::PoseStamped pose_with_yaw(const double yaw) {
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "odom";
  pose.pose.orientation.z = std::sin(yaw * 0.5);
  pose.pose.orientation.w = std::cos(yaw * 0.5);
  return pose;
}

TEST(ElevatorAwareProgressChecker,
     ProgressStateSelectsAndPausesElevatorProgress) {
  rclcpp::init(0, nullptr);
  rclcpp::NodeOptions options;
  options.parameter_overrides({
      rclcpp::Parameter("progress_checker.required_movement_radius", 0.03),
      rclcpp::Parameter("progress_checker.required_movement_angle", 0.05),
      rclcpp::Parameter("progress_checker.movement_time_allowance", 0.05),
      rclcpp::Parameter("progress_checker.elevator_required_movement_radius",
                        0.015),
      rclcpp::Parameter("progress_checker.elevator_required_movement_angle",
                        0.015),
      rclcpp::Parameter("progress_checker.elevator_movement_time_allowance",
                        0.20),
      rclcpp::Parameter("progress_checker.elevator_progress_state_timeout",
                        0.50),
      rclcpp::Parameter("progress_checker.elevator_progress_state_topic",
                        "/test/elevator_scoped_progress_state"),
  });
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
      "elevator_aware_progress_checker_test", "", options);
  robot_nav_config::ElevatorAwareProgressChecker checker;
  checker.initialize(node, "progress_checker");

  auto origin = pose_with_yaw(0.0);
  auto field_correction = pose_with_yaw(0.0407);
  checker.reset();
  EXPECT_TRUE(checker.check(origin));
  std::this_thread::sleep_for(70ms);
  EXPECT_FALSE(checker.check(field_correction));

  auto progress_state = node->create_publisher<std_msgs::msg::UInt8>(
      "/test/elevator_scoped_progress_state", rclcpp::QoS(1).reliable());
  progress_state->on_activate();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());

  // Controller server resets progress at the new action boundary before the
  // selected elevator controller begins publishing its state.
  checker.reset();
  std_msgs::msg::UInt8 state;
  state.data = static_cast<std::uint8_t>(
      robot_nav_config::ElevatorScopedProgressState::kTracking);
  progress_state->publish(state);
  executor.spin_some();

  EXPECT_TRUE(checker.check(origin));
  std::this_thread::sleep_for(70ms);
  EXPECT_TRUE(checker.check(field_correction));

  state.data = static_cast<std::uint8_t>(
      robot_nav_config::ElevatorScopedProgressState::kWaitClear);
  progress_state->publish(state);
  executor.spin_some();
  std::this_thread::sleep_for(250ms);
  EXPECT_TRUE(checker.check(field_correction));

  state.data = static_cast<std::uint8_t>(
      robot_nav_config::ElevatorScopedProgressState::kTracking);
  progress_state->publish(state);
  executor.spin_some();
  EXPECT_TRUE(checker.check(field_correction));

  // reset() is the action boundary. A state from the preceding elevator
  // action must not weaken the next ordinary action for the remaining TTL.
  checker.reset();
  EXPECT_TRUE(checker.check(origin));
  std::this_thread::sleep_for(70ms);
  EXPECT_FALSE(checker.check(field_correction));

  executor.remove_node(node->get_node_base_interface());
  rclcpp::shutdown();
}

} // namespace
