#pragma once

#include <atomic>
#include <cstddef>
#include <functional>
#include <memory>
#include <optional>
#include <string>

#include <rclcpp/logger.hpp>

#include "robot_api_server/infrastructure/http/http_common.hpp"

namespace robot_api_server::infrastructure::http
{

class HttpServer;

struct ApiGatewayModuleConfig
{
  std::string host{"0.0.0.0"};
  int port{8080};
  std::string api_token;
  int max_connections{16};
  int socket_timeout_sec{15};
  std::size_t max_request_bytes{1024U * 1024U};
};

struct ApiGatewayModulePorts
{
  std::function<HttpResponse(const HttpRequest &, bool)> authenticated_route;
  std::function<bool(int, const HttpRequest &)> socket_route;
  std::function<void()> shutdown_socket_sessions;
};

// Owns the complete public HTTP gateway policy: listener lifecycle, transport
// callbacks, preflight/authentication, access logging, connection observations,
// and API catalog metadata. Domain routing remains an injected composition port.
class ApiGatewayModule
{
public:
  ApiGatewayModule(
    rclcpp::Logger logger,
    ApiGatewayModuleConfig config,
    ApiGatewayModulePorts ports);
  ~ApiGatewayModule();

  ApiGatewayModule(const ApiGatewayModule &) = delete;
  ApiGatewayModule & operator=(const ApiGatewayModule &) = delete;
  ApiGatewayModule(ApiGatewayModule &&) = delete;
  ApiGatewayModule & operator=(ApiGatewayModule &&) = delete;

  void start();
  void stop();

  bool running() const noexcept;
  int active_connections() const noexcept;
  int max_connections() const noexcept;
  int bound_port() const noexcept;
  bool token_configured() const noexcept;
  bool token_allowed(const HttpRequest & request) const;

  std::optional<HttpResponse> handle_metadata(const HttpRequest & request) const;

private:
  HttpResponse dispatch_authenticated(
    const HttpRequest & request,
    bool maintenance_peer_is_loopback) const;
  void log_http_request(const HttpRequest & request, int status, long latency_ms) const;

  rclcpp::Logger logger_;
  ApiGatewayModuleConfig config_;
  ApiGatewayModulePorts ports_;
  std::atomic<bool> running_{false};
  std::unique_ptr<HttpServer> server_;
};

}  // namespace robot_api_server::infrastructure::http
