#include <memory>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"

#include "robot_api_server/infrastructure/http/api_gateway_configuration_module.hpp"

namespace robot_api_server::infrastructure::http
{
namespace
{

class ApiGatewayConfigurationModuleTest : public ::testing::Test
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

TEST_F(ApiGatewayConfigurationModuleTest, PreservesProductionDefaults)
{
  auto node = std::make_shared<rclcpp::Node>("api_gateway_configuration_defaults_test");

  const auto config = ApiGatewayConfigurationModule::declare_parameters(*node);

  EXPECT_EQ(config.host, "0.0.0.0");
  EXPECT_EQ(config.port, 8080);
  EXPECT_TRUE(config.api_token.empty());
  EXPECT_EQ(config.max_connections, 16);
  EXPECT_EQ(config.socket_timeout_sec, 15);
  EXPECT_EQ(config.max_request_bytes, 1024U * 1024U);
}

TEST_F(ApiGatewayConfigurationModuleTest, ProjectsOverridesWithoutChangingRuntimePolicy)
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("host", "127.0.0.1");
  options.append_parameter_override("port", 18080);
  options.append_parameter_override("api_token", "test-secret");
  options.append_parameter_override("max_http_connections", 100);
  auto node = std::make_shared<rclcpp::Node>(
    "api_gateway_configuration_overrides_test", options);

  const auto config = ApiGatewayConfigurationModule::declare_parameters(*node);

  EXPECT_EQ(config.host, "127.0.0.1");
  EXPECT_EQ(config.port, 18080);
  EXPECT_EQ(config.api_token, "test-secret");
  EXPECT_EQ(config.max_connections, 100);
}

}  // namespace
}  // namespace robot_api_server::infrastructure::http
