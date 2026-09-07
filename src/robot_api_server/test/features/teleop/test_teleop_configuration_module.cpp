#include <memory>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"

#include "robot_api_server/features/teleop/teleop_configuration_module.hpp"

namespace robot_api_server::features::teleop
{
namespace
{

class TeleopConfigurationModuleTest : public ::testing::Test
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

TEST_F(TeleopConfigurationModuleTest, PreservesProductionDefaultsAndTtlProjection)
{
  auto node = std::make_shared<rclcpp::Node>("teleop_configuration_defaults_test");
  TeleopConfigurationInputs inputs;
  inputs.subscription_max_ttl_ms = 42000;

  const auto config = TeleopConfigurationModule::declare_parameters(*node, inputs);

  EXPECT_TRUE(config.module.stop_on_charging);
  EXPECT_EQ(config.module.cmd_topic, "/cmd_vel_api");
  EXPECT_EQ(config.module.reverse_enable_topic, "/ranger_mini3/teleop_allow_reverse");
  EXPECT_EQ(config.module.pose_topic, "/local_state/odometry");
  EXPECT_DOUBLE_EQ(config.module.max_linear_x_mps, 1.00);
  EXPECT_DOUBLE_EQ(config.module.max_angular_z_radps, 0.55);
  EXPECT_FALSE(config.module.allow_reverse);
  EXPECT_TRUE(config.module.require_mapping_active);
  EXPECT_DOUBLE_EQ(config.module.watchdog_timeout_sec, 0.5);
  EXPECT_DOUBLE_EQ(config.module.socket_idle_timeout_sec, 5.0);
  EXPECT_DOUBLE_EQ(config.module.repeat_rate_hz, 20.0);
  EXPECT_EQ(config.module.subscription_max_ttl_ms, 42000);
}

TEST_F(TeleopConfigurationModuleTest, AppliesEstablishedBoundsAndDependencies)
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("teleop_stop_on_charging", false);
  options.append_parameter_override("teleop_cmd_topic", "/test/cmd_vel");
  options.append_parameter_override("teleop_reverse_enable_topic", "/test/reverse");
  options.append_parameter_override("teleop_pose_topic", "/test/pose");
  options.append_parameter_override("teleop_max_linear_x_mps", -1.0);
  options.append_parameter_override("teleop_max_angular_z_radps", -2.0);
  options.append_parameter_override("teleop_allow_reverse", true);
  options.append_parameter_override("teleop_require_mapping_active", false);
  options.append_parameter_override("teleop_watchdog_timeout_sec", 0.01);
  options.append_parameter_override("teleop_socket_idle_timeout_sec", 0.05);
  options.append_parameter_override("teleop_repeat_rate_hz", 0.0);
  auto node = std::make_shared<rclcpp::Node>(
    "teleop_configuration_bounds_test", options);
  TeleopConfigurationInputs inputs;
  inputs.subscription_max_ttl_ms = 12345;

  const auto config = TeleopConfigurationModule::declare_parameters(*node, inputs);

  EXPECT_FALSE(config.module.stop_on_charging);
  EXPECT_EQ(config.module.cmd_topic, "/test/cmd_vel");
  EXPECT_EQ(config.module.reverse_enable_topic, "/test/reverse");
  EXPECT_EQ(config.module.pose_topic, "/test/pose");
  EXPECT_DOUBLE_EQ(config.module.max_linear_x_mps, 0.0);
  EXPECT_DOUBLE_EQ(config.module.max_angular_z_radps, 0.0);
  EXPECT_TRUE(config.module.allow_reverse);
  EXPECT_FALSE(config.module.require_mapping_active);
  EXPECT_DOUBLE_EQ(config.module.watchdog_timeout_sec, 0.1);
  EXPECT_DOUBLE_EQ(config.module.socket_idle_timeout_sec, 0.1);
  EXPECT_DOUBLE_EQ(config.module.repeat_rate_hz, 1.0);
  EXPECT_EQ(config.module.subscription_max_ttl_ms, 12345);
}

}  // namespace
}  // namespace robot_api_server::features::teleop
