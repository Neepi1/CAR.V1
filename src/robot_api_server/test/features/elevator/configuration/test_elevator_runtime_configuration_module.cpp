#include <memory>
#include <string>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"

#include \
  "robot_api_server/features/elevator/configuration/elevator_runtime_configuration_module.hpp"

namespace robot_api_server::features::elevator::configuration
{
namespace
{

class ElevatorRuntimeConfigurationModuleTest : public ::testing::Test
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

ElevatorRuntimeConfigurationInputs test_inputs()
{
  ElevatorRuntimeConfigurationInputs inputs;
  inputs.maps_root = "/test/maps";
  inputs.runtime_map_context_file = "/test/runtime_map_context.json";
  inputs.navigate_to_pose_action = "/test_navigate";
  inputs.floor_switch_action = "/test_floor_switch";
  inputs.floor_switch_timeout_sec = 123.0;
  inputs.motion_allowed_topic = "/test/motion_allowed";
  inputs.safety_status_topic = "/test/safety_status";
  inputs.navigation_status_topic = "/test/navigation_status";
  return inputs;
}

TEST_F(ElevatorRuntimeConfigurationModuleTest, PreservesDefaultsAndAllNeighborProjections)
{
  auto node = std::make_shared<rclcpp::Node>("elevator_runtime_configuration_defaults_test");
  const auto config = ElevatorRuntimeConfigurationModule::declare_parameters(
    *node, test_inputs());

  EXPECT_EQ(config.maps_root, "/test/maps");
  EXPECT_EQ(config.runtime_map_context_file, "/test/runtime_map_context.json");
  EXPECT_EQ(config.navigate_to_pose_action, "/test_navigate");
  EXPECT_EQ(config.floor_switch_action, "/test_floor_switch");
  EXPECT_DOUBLE_EQ(config.floor_switch_timeout_sec, 123.0);
  EXPECT_EQ(config.motion_allowed_topic, "/test/motion_allowed");
  EXPECT_EQ(config.safety_status_topic, "/test/safety_status");
  EXPECT_EQ(config.navigation_status_topic, "/test/navigation_status");
  EXPECT_FALSE(config.runtime_adapter_enabled);
  EXPECT_FALSE(config.arm_button_control_enabled);
  EXPECT_EQ(config.arm_service_host, "127.0.0.1");
  EXPECT_EQ(config.arm_service_port, 8083);
  EXPECT_DOUBLE_EQ(config.arm_request_timeout_sec, 12.0);
  EXPECT_DOUBLE_EQ(config.arm_task_timeout_sec, 180.0);
  EXPECT_DOUBLE_EQ(config.arm_poll_interval_sec, 0.20);
  EXPECT_NE(
    config.elevator_scoped_behavior_tree.find("navigate_elevator_scoped_motion.xml"),
    std::string::npos);
  EXPECT_NE(
    config.elevator_hall_call_scoped_behavior_tree.find(
      "navigate_elevator_hall_call_scoped_motion.xml"),
    std::string::npos);
  EXPECT_NE(
    config.elevator_reverse_entry_staging_behavior_tree.find(
      "navigate_elevator_reverse_entry_staging.xml"),
    std::string::npos);
  EXPECT_NE(
    config.elevator_reverse_docking_behavior_tree.find(
      "navigate_elevator_reverse_docking.xml"),
    std::string::npos);
  EXPECT_NE(
    config.elevator_cabin_entry_direct_behavior_tree.find(
      "navigate_elevator_cabin_entry_direct.xml"),
    std::string::npos);
  EXPECT_EQ(
    config.elevator_entry_collision_bypass_permit_topic,
    "/ranger_mini3/elevator_entry_collision_bypass");
  EXPECT_DOUBLE_EQ(config.elevator_entry_collision_bypass_refresh_sec, 0.20);
  EXPECT_EQ(
    config.recovery_hold_release_service,
    "/safety/release_motion_hold_if_execution_idle");
}

TEST_F(ElevatorRuntimeConfigurationModuleTest, AppliesEstablishedPortAndTimeoutClamps)
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("elevator_arm_service_port", 70000);
  options.append_parameter_override("elevator_arm_request_timeout_sec", 0.01);
  options.append_parameter_override("elevator_arm_task_timeout_sec", 900.0);
  options.append_parameter_override("elevator_arm_poll_interval_sec", 0.001);
  options.append_parameter_override("elevator_entry_collision_bypass_refresh_sec", 0.001);
  auto node = std::make_shared<rclcpp::Node>(
    "elevator_runtime_configuration_clamps_test", options);

  const auto config = ElevatorRuntimeConfigurationModule::declare_parameters(
    *node, test_inputs());

  EXPECT_EQ(config.arm_service_port, 65535);
  EXPECT_DOUBLE_EQ(config.arm_request_timeout_sec, 0.1);
  EXPECT_DOUBLE_EQ(config.arm_task_timeout_sec, 600.0);
  EXPECT_DOUBLE_EQ(config.arm_poll_interval_sec, 0.02);
  EXPECT_DOUBLE_EQ(config.elevator_entry_collision_bypass_refresh_sec, 0.05);
}

}  // namespace
}  // namespace robot_api_server::features::elevator::configuration
