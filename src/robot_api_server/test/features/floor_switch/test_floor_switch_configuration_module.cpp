#include <chrono>
#include <memory>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"

#include \
  "robot_api_server/features/floor_switch/floor_switch_configuration_module.hpp"

namespace robot_api_server::features::floor_switch
{
namespace
{

class FloorSwitchConfigurationModuleTest : public ::testing::Test
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

FloorSwitchConfigurationInputs test_inputs()
{
  FloorSwitchConfigurationInputs inputs;
  inputs.maps_root = "/test/maps";
  inputs.runtime_map_context_file = "/test/runtime_map_context.json";
  inputs.localization_health_topic = "/test/localization_health";
  inputs.service_timeout = std::chrono::seconds(11);
  return inputs;
}

TEST_F(FloorSwitchConfigurationModuleTest, PreservesDefaultsAndNeighborProjections)
{
  auto node = std::make_shared<rclcpp::Node>("floor_switch_configuration_defaults_test");
  const auto config = FloorSwitchConfigurationModule::declare_parameters(
    *node, test_inputs());

  EXPECT_EQ(config.module.maps_root, "/test/maps");
  EXPECT_EQ(config.module.runtime_map_context_file, "/test/runtime_map_context.json");
  EXPECT_EQ(config.module.localization_health_topic, "/test/localization_health");
  EXPECT_EQ(config.module.service_timeout, std::chrono::seconds(11));
  EXPECT_EQ(config.floor_status_topic, "/floor_manager/status");
  EXPECT_EQ(config.module.transition_status_topic, "/floor_manager/transition_status");
  EXPECT_TRUE(config.module.negative_interlock_enabled);
  EXPECT_EQ(config.module.legacy_service, "/floor_manager/switch_floor");
  EXPECT_EQ(config.module.live_action, "/floor_manager/floor_switch");
  EXPECT_DOUBLE_EQ(config.module.live_timeout_sec, 120.0);
}

TEST_F(FloorSwitchConfigurationModuleTest, AppliesEstablishedLiveTimeoutBounds)
{
  rclcpp::NodeOptions low_options;
  low_options.append_parameter_override("live_floor_switch_timeout_sec", 1.0);
  auto low_node = std::make_shared<rclcpp::Node>(
    "floor_switch_configuration_low_timeout_test", low_options);
  const auto low = FloorSwitchConfigurationModule::declare_parameters(
    *low_node, test_inputs());
  EXPECT_DOUBLE_EQ(low.module.live_timeout_sec, 15.0);

  rclcpp::NodeOptions high_options;
  high_options.append_parameter_override("live_floor_switch_timeout_sec", 1000.0);
  auto high_node = std::make_shared<rclcpp::Node>(
    "floor_switch_configuration_high_timeout_test", high_options);
  const auto high = FloorSwitchConfigurationModule::declare_parameters(
    *high_node, test_inputs());
  EXPECT_DOUBLE_EQ(high.module.live_timeout_sec, 300.0);
}

}  // namespace
}  // namespace robot_api_server::features::floor_switch
