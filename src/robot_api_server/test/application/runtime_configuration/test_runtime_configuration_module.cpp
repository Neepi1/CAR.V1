#include <chrono>
#include <memory>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"

#include \
  "robot_api_server/application/runtime_configuration/runtime_configuration_module.hpp"

namespace robot_api_server::application::runtime_configuration
{
namespace
{

class RuntimeConfigurationModuleTest : public ::testing::Test
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

TEST_F(RuntimeConfigurationModuleTest, PreservesProductionDefaults)
{
  auto node = std::make_shared<rclcpp::Node>("runtime_configuration_defaults_test");

  const auto config = RuntimeConfigurationModule::declare_parameters(*node);

  EXPECT_EQ(config.navigate_to_pose_action, "/navigate_to_pose");
  EXPECT_EQ(config.navigate_to_pose_status_topic, "/navigate_to_pose/_action/status");
  EXPECT_DOUBLE_EQ(config.service_timeout_sec, 8.0);
  EXPECT_EQ(config.service_timeout(), std::chrono::seconds(8));
}

TEST_F(RuntimeConfigurationModuleTest, ProjectsOverridesWithoutChangingSemantics)
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("navigate_to_pose_action", "/test/navigate");
  options.append_parameter_override(
    "navigate_to_pose_status_topic", "/test/navigate/_action/status");
  options.append_parameter_override("service_timeout_sec", 2.5);
  auto node = std::make_shared<rclcpp::Node>(
    "runtime_configuration_overrides_test", options);

  const auto config = RuntimeConfigurationModule::declare_parameters(*node);

  EXPECT_EQ(config.navigate_to_pose_action, "/test/navigate");
  EXPECT_EQ(config.navigate_to_pose_status_topic, "/test/navigate/_action/status");
  EXPECT_DOUBLE_EQ(config.service_timeout_sec, 2.5);
  EXPECT_EQ(config.service_timeout(), std::chrono::milliseconds(2500));
}

}  // namespace
}  // namespace robot_api_server::application::runtime_configuration
