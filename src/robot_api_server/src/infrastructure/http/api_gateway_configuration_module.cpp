#include "robot_api_server/infrastructure/http/api_gateway_configuration_module.hpp"

#include <string>

namespace robot_api_server::infrastructure::http
{

ApiGatewayModuleConfig ApiGatewayConfigurationModule::declare_parameters(
  rclcpp::Node & node)
{
  ApiGatewayModuleConfig config;
  config.host = node.declare_parameter<std::string>("host", "0.0.0.0");
  config.port = node.declare_parameter<int>("port", 8080);
  config.api_token = node.declare_parameter<std::string>("api_token", "");
  config.max_connections = node.declare_parameter<int>("max_http_connections", 16);
  return config;
}

}  // namespace robot_api_server::infrastructure::http
