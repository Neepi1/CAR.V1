#include <memory>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"

#include "robot_api_server/features/safety/safety_configuration_module.hpp"

namespace robot_api_server::features::safety
{
namespace
{

class SafetyConfigurationModuleTest : public ::testing::Test
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

TEST_F(SafetyConfigurationModuleTest, PreservesProductionDefaults)
{
  auto node = std::make_shared<rclcpp::Node>("safety_configuration_defaults_test");

  const auto config = SafetyConfigurationModule::declare_parameters(*node);

  EXPECT_EQ(config.module.ros.estop_topic, "/safety/estop");
  EXPECT_EQ(config.module.ros.status_topic, "/safety/status");
  EXPECT_EQ(config.module.ros.motion_allowed_topic, "/safety/motion_allowed");
}

TEST_F(SafetyConfigurationModuleTest, ProjectsOverridesWithoutInterpretation)
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("safety_estop_topic", "/test/estop");
  options.append_parameter_override("safety_status_topic", "/test/status");
  options.append_parameter_override("safety_motion_allowed_topic", "/test/motion_allowed");
  auto node = std::make_shared<rclcpp::Node>("safety_configuration_overrides_test", options);

  const auto config = SafetyConfigurationModule::declare_parameters(*node);

  EXPECT_EQ(config.module.ros.estop_topic, "/test/estop");
  EXPECT_EQ(config.module.ros.status_topic, "/test/status");
  EXPECT_EQ(config.module.ros.motion_allowed_topic, "/test/motion_allowed");
}

}  // namespace
}  // namespace robot_api_server::features::safety
