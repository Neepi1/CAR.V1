#include <memory>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"

#include "robot_api_server/features/power/power_configuration_module.hpp"

namespace robot_api_server::features::power
{
namespace
{

class PowerConfigurationModuleTest : public ::testing::Test
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

TEST_F(PowerConfigurationModuleTest, PreservesProductionDefaults)
{
  auto node = std::make_shared<rclcpp::Node>("power_configuration_defaults_test");

  const auto config = PowerConfigurationModule::declare_parameters(*node);

  EXPECT_EQ(config.module.state_topic, "/battery_state");
  EXPECT_DOUBLE_EQ(config.module.state_max_age_sec, 3.0);
  EXPECT_DOUBLE_EQ(config.module.charging_current_min_a, 0.10);
  EXPECT_DOUBLE_EQ(config.module.charging_contact_voltage_min_v, 40.0);
  EXPECT_DOUBLE_EQ(config.module.charging_contact_voltage_max_v, 1000.0);
  EXPECT_TRUE(config.module.full_soc_voltage_contact_enable);
  EXPECT_DOUBLE_EQ(config.module.full_soc_threshold_pct, 99.0);
  EXPECT_DOUBLE_EQ(config.module.contact_stable_required_sec, 2.0);
}

TEST_F(PowerConfigurationModuleTest, AppliesExistingBoundsAndDependencies)
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("bms_state_max_age_sec", -1.0);
  options.append_parameter_override("teleop_charging_current_min_a", -1.0);
  options.append_parameter_override("bms_charging_contact_voltage_min_v", 50.0);
  options.append_parameter_override("bms_charging_contact_voltage_max_v", 40.0);
  options.append_parameter_override("bms_full_soc_threshold_pct", 120.0);
  options.append_parameter_override("bms_full_soc_voltage_contact_enable", false);
  options.append_parameter_override("dock_contact_latch_bms_require_contact_sec", -2.0);
  auto node = std::make_shared<rclcpp::Node>("power_configuration_bounds_test", options);

  const auto config = PowerConfigurationModule::declare_parameters(*node);

  EXPECT_DOUBLE_EQ(config.module.state_max_age_sec, 0.1);
  EXPECT_DOUBLE_EQ(config.module.charging_current_min_a, 0.0);
  EXPECT_DOUBLE_EQ(config.module.charging_contact_voltage_min_v, 50.0);
  EXPECT_DOUBLE_EQ(config.module.charging_contact_voltage_max_v, 50.0);
  EXPECT_FALSE(config.module.full_soc_voltage_contact_enable);
  EXPECT_DOUBLE_EQ(config.module.full_soc_threshold_pct, 100.0);
  EXPECT_DOUBLE_EQ(config.module.contact_stable_required_sec, 0.0);
}

}  // namespace
}  // namespace robot_api_server::features::power
