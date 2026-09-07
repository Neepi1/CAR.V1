#include <chrono>
#include <memory>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"

#include "robot_api_server/application/subscriptions/subscription_configuration_module.hpp"

namespace robot_api_server::application::subscriptions
{
namespace
{

class SubscriptionConfigurationModuleTest : public ::testing::Test
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

TEST_F(SubscriptionConfigurationModuleTest, PreservesProductionDefaults)
{
  auto node = std::make_shared<rclcpp::Node>("subscription_configuration_defaults_test");

  const auto config = SubscriptionConfigurationModule::declare_parameters(*node);

  EXPECT_EQ(config.scan_topic, "/scan");
  EXPECT_DOUBLE_EQ(config.scan_max_age_sec, 2.0);
  EXPECT_EQ(config.default_ttl_ms, 10000);
  EXPECT_EQ(config.max_ttl_ms, 60000);
  EXPECT_EQ(config.expiry_period, std::chrono::milliseconds(1000));
}

TEST_F(SubscriptionConfigurationModuleTest, ProjectsOverridesForRuntimeNormalization)
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("scan_topic", "/test/scan");
  options.append_parameter_override("scan_max_age_sec", -1.0);
  options.append_parameter_override("subscription_default_ttl_ms", 500);
  options.append_parameter_override("subscription_max_ttl_ms", 100);
  auto node = std::make_shared<rclcpp::Node>(
    "subscription_configuration_overrides_test", options);

  const auto config = SubscriptionConfigurationModule::declare_parameters(*node);

  EXPECT_EQ(config.scan_topic, "/test/scan");
  EXPECT_DOUBLE_EQ(config.scan_max_age_sec, -1.0);
  EXPECT_EQ(config.default_ttl_ms, 500);
  EXPECT_EQ(config.max_ttl_ms, 100);
}

}  // namespace
}  // namespace robot_api_server::application::subscriptions
