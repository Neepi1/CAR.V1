#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <cerrno>
#include <cstdlib>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <rclcpp/logger.hpp>
#include <yaml-cpp/yaml.h>

#include "robot_api_server/infrastructure/http/api_gateway_module.hpp"

namespace http = robot_api_server::infrastructure::http;

namespace
{

int wait_for_bound_port(const http::ApiGatewayModule & gateway)
{
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (std::chrono::steady_clock::now() < deadline) {
    const int port = gateway.bound_port();
    if (port > 0) {
      return port;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  return 0;
}

std::string exchange_http(const int port, const std::string & request)
{
  const int client_fd = ::socket(AF_INET, SOCK_STREAM, 0);
  EXPECT_GE(client_fd, 0);
  if (client_fd < 0) {
    return {};
  }

  sockaddr_in address{};
  address.sin_family = AF_INET;
  address.sin_port = htons(static_cast<std::uint16_t>(port));
  EXPECT_EQ(::inet_pton(AF_INET, "127.0.0.1", &address.sin_addr), 1);
  if (
    ::connect(client_fd, reinterpret_cast<sockaddr *>(&address), sizeof(address)) != 0)
  {
    ADD_FAILURE() << "failed to connect to API gateway";
    (void)::close(client_fd);
    return {};
  }

  std::size_t sent = 0U;
  while (sent < request.size()) {
    const ssize_t count = ::send(
      client_fd, request.data() + sent, request.size() - sent, MSG_NOSIGNAL);
    if (count <= 0) {
      ADD_FAILURE() << "failed to send API gateway request";
      (void)::close(client_fd);
      return {};
    }
    sent += static_cast<std::size_t>(count);
  }
  (void)::shutdown(client_fd, SHUT_WR);

  std::string response;
  char buffer[2048];
  while (true) {
    const ssize_t count = ::recv(client_fd, buffer, sizeof(buffer), 0);
    if (count < 0 && errno == EINTR) {
      continue;
    }
    if (count <= 0) {
      break;
    }
    response.append(buffer, static_cast<std::size_t>(count));
  }
  (void)::close(client_fd);
  return response;
}

TEST(ApiGatewayModuleTest, OwnsTransportPreflightAndLifecycle)
{
  std::atomic<int> business_route_calls{0};
  std::atomic<bool> saw_loopback{false};
  std::atomic<int> socket_shutdown_calls{0};

  http::ApiGatewayModuleConfig config;
  config.host = "127.0.0.1";
  config.port = 0;
  config.api_token = "commercial-secret";
  config.max_connections = 100;
  config.socket_timeout_sec = 2;

  http::ApiGatewayModulePorts ports;
  ports.authenticated_route = [&](const auto & request, const bool loopback) {
      business_route_calls.fetch_add(1, std::memory_order_acq_rel);
      saw_loopback.store(loopback, std::memory_order_release);
      return robot_api_server::HttpResponse{
        202, "application/json", "{\"accepted\":\"" + request.path + "\"}"};
    };
  ports.socket_route = [](const int, const auto &) {return false;};
  ports.shutdown_socket_sessions = [&]() {
      socket_shutdown_calls.fetch_add(1, std::memory_order_acq_rel);
    };

  http::ApiGatewayModule gateway(
    rclcpp::get_logger("api_gateway_module_test"), config, std::move(ports));
  EXPECT_EQ(gateway.max_connections(), 64);
  EXPECT_TRUE(gateway.token_configured());
  EXPECT_FALSE(gateway.running());

  gateway.start();
  EXPECT_TRUE(gateway.running());
  const int port = wait_for_bound_port(gateway);
  ASSERT_GT(port, 0);

  const auto options = exchange_http(
    port,
    "OPTIONS /api/v1/navigation/start HTTP/1.1\r\nHost: 127.0.0.1\r\n\r\n");
  EXPECT_NE(options.find("HTTP/1.1 200 OK"), std::string::npos);
  EXPECT_EQ(business_route_calls.load(std::memory_order_acquire), 0);

  const auto unauthorized = exchange_http(
    port,
    "GET /api/v1/status HTTP/1.1\r\nHost: 127.0.0.1\r\n\r\n");
  EXPECT_NE(unauthorized.find("HTTP/1.1 401 Unauthorized"), std::string::npos);
  EXPECT_NE(unauthorized.find("missing or invalid X-Robot-Token"), std::string::npos);
  EXPECT_EQ(business_route_calls.load(std::memory_order_acquire), 0);

  const auto accepted = exchange_http(
    port,
    "GET /api/v1/status HTTP/1.1\r\nHost: 127.0.0.1\r\n"
    "X-Robot-Token: commercial-secret\r\n\r\n");
  EXPECT_NE(accepted.find("HTTP/1.1 202 Accepted"), std::string::npos);
  EXPECT_NE(accepted.find("{\"accepted\":\"/api/v1/status\"}"), std::string::npos);
  EXPECT_EQ(business_route_calls.load(std::memory_order_acquire), 1);
  EXPECT_TRUE(saw_loopback.load(std::memory_order_acquire));

  gateway.stop();
  EXPECT_FALSE(gateway.running());
  EXPECT_EQ(gateway.active_connections(), 0);
  EXPECT_EQ(gateway.bound_port(), 0);
  EXPECT_EQ(socket_shutdown_calls.load(std::memory_order_acquire), 1);
  gateway.stop();
  EXPECT_EQ(socket_shutdown_calls.load(std::memory_order_acquire), 1);
}

TEST(ApiGatewayModuleTest, PreservesExactOpenApiCatalog)
{
  http::ApiGatewayModuleConfig config;
  http::ApiGatewayModulePorts ports;
  ports.authenticated_route = [](const auto &, const bool) {
      return robot_api_server::HttpResponse{404, "application/json", "{}"};
    };
  http::ApiGatewayModule gateway(
    rclcpp::get_logger("api_gateway_openapi_test"), config, std::move(ports));

  EXPECT_FALSE(gateway.handle_metadata({"POST", "/api/v1/openapi", {}, {}, ""}));
  EXPECT_FALSE(gateway.handle_metadata({"GET", "/api/v1/unknown", {}, {}, ""}));
  const auto response = gateway.handle_metadata({"GET", "/api/v1/openapi", {}, {}, ""});
  ASSERT_TRUE(response.has_value());
  EXPECT_EQ(response->status, 200);
  const auto document = YAML::Load(response->body);
  EXPECT_TRUE(document["ok"].as<bool>());
  EXPECT_EQ(
    document["endpoints"].as<std::vector<std::string>>(),
    (std::vector<std::string>{
      "GET /api/v1/status",
      "GET /api/v1/robot/pose",
      "GET /api/v1/maps",
      "GET /api/v1/maps/semantic_layer",
      "GET /api/v1/maps/poses",
      "GET /api/v1/elevator-config",
      "PUT /api/v1/elevator-config/draft",
      "POST /api/v1/elevator-config/publish",
      "POST /api/v1/elevator-config/rollback",
      "POST /api/v1/elevator-test/start",
      "GET /api/v1/elevator-test/state",
      "POST /api/v1/elevator-test/confirm",
      "POST /api/v1/elevator-test/cancel",
      "POST /api/v1/elevator-test/recover",
      "GET /api/v1/maps/filters/keepout",
      "GET /api/v1/mapping/2d/map",
      "GET /api/v1/openapi",
      "POST /api/v1/maps/poses",
      "PUT /api/v1/maps/poses/{pose_id}",
      "DELETE /api/v1/maps/poses/{pose_id}",
      "PUT /api/v1/maps/poses/batch",
      "POST /api/v1/subscriptions/acquire",
      "POST /api/v1/subscriptions/release",
      "POST /api/v1/subscriptions/heartbeat",
      "POST /api/v1/mapping/2d/start",
      "POST /api/v1/mapping/2d/stop",
      "POST /api/v1/mapping/2d/save",
      "POST /api/v1/mapping/stop",
      "POST /api/v1/mapping/save",
      "POST /api/v1/maps/delete",
      "POST /api/v1/maps/poses/save",
      "POST /api/v1/maps/poses/save_current",
      "POST /api/v1/maps/filters/keepout/save",
      "POST /api/v1/safety/stop",
      "POST /api/v1/safety/resume",
      "POST /api/v1/floor-switch/start",
      "GET /api/v1/floor-switch/state",
      "POST /api/v1/floor-switch/cancel",
      "POST /api/v1/floors/switch",
      "POST /api/v1/localization/trigger",
      "POST /api/v1/navigation/start",
      "GET /api/v1/navigation/state",
      "GET /api/v1/navigation/pre_goal_check",
      "POST /api/v1/navigation/goal",
      "POST /api/v1/navigation/cancel",
      "POST /api/v1/navigation/stop",
      "POST /api/v1/navigation/stop_runtime",
      "GET /api/v1/docking/state",
      "POST /api/v1/docking/start",
      "POST /api/v1/docking/undock",
      "POST /api/v1/docking/confirm_docked",
      "POST /api/v1/docking/clear_docked_latch",
      "POST /api/v1/docking/cancel",
      "POST /api/v1/docking/stop",
      "WS /ws/v1/teleop",
    }));
  EXPECT_EQ(
    document["not_wired"].as<std::vector<std::string>>(),
    (std::vector<std::string>{"POST /api/v1/mapping/3d/start"}));
}

TEST(ApiGatewayModuleTest, FallsBackToEnvironmentToken)
{
  ASSERT_EQ(::setenv("ROBOT_API_TOKEN", "environment-secret", 1), 0);
  http::ApiGatewayModuleConfig config;
  config.api_token.clear();
  http::ApiGatewayModulePorts ports;
  ports.authenticated_route = [](const auto &, const bool) {
      return robot_api_server::HttpResponse{};
    };
  {
    http::ApiGatewayModule gateway(
      rclcpp::get_logger("api_gateway_environment_test"), config, std::move(ports));
    EXPECT_TRUE(gateway.token_configured());
    EXPECT_TRUE(gateway.token_allowed({
        "GET", "/api/v1/status", {}, {{"x-robot-token", "environment-secret"}}, ""}));
    EXPECT_FALSE(gateway.token_allowed({"GET", "/api/v1/status", {}, {}, ""}));
  }
  ASSERT_EQ(::unsetenv("ROBOT_API_TOKEN"), 0);
}

}  // namespace
