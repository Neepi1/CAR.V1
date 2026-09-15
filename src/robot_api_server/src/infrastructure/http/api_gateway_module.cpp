#include "robot_api_server/infrastructure/http/api_gateway_module.hpp"

#include <algorithm>
#include <cstdlib>
#include <stdexcept>
#include <utility>

#include <rclcpp/logging.hpp>

#include "robot_api_server/infrastructure/http/http_server.hpp"

namespace robot_api_server::infrastructure::http
{

ApiGatewayModule::ApiGatewayModule(
  rclcpp::Logger logger,
  ApiGatewayModuleConfig config,
  ApiGatewayModulePorts ports)
: logger_(std::move(logger)), config_(std::move(config)), ports_(std::move(ports))
{
  if (!ports_.authenticated_route) {
    throw std::invalid_argument("ApiGatewayModule requires an authenticated route port");
  }
  config_.max_connections = std::clamp(config_.max_connections, 4, 64);
  config_.socket_timeout_sec = std::max(1, config_.socket_timeout_sec);
  config_.max_request_bytes = std::max<std::size_t>(4096U, config_.max_request_bytes);
  if (config_.api_token.empty()) {
    const char * environment_api_token = std::getenv("ROBOT_API_TOKEN");
    if (environment_api_token != nullptr) {
      config_.api_token = environment_api_token;
    }
  }

  HttpServerCallbacks callbacks;
  callbacks.route = [this](const HttpRequest & request, const bool peer_loopback) {
      return dispatch_authenticated(request, peer_loopback);
    };
  callbacks.socket_route = [this](const int client_fd, const HttpRequest & request) {
      return ports_.socket_route && ports_.socket_route(client_fd, request);
    };
  callbacks.access_log = [this](
    const HttpRequest & request, const int status, const long latency_ms) {
      log_http_request(request, status, latency_ms);
    };
  callbacks.event_log = [this](
    const HttpServerLogLevel level, const std::string & message) {
      switch (level) {
        case HttpServerLogLevel::kInfo:
          RCLCPP_INFO(logger_, "%s", message.c_str());
          break;
        case HttpServerLogLevel::kWarning:
          RCLCPP_WARN(logger_, "%s", message.c_str());
          break;
        case HttpServerLogLevel::kError:
          RCLCPP_ERROR(logger_, "%s", message.c_str());
          break;
      }
    };
  server_ = std::make_unique<HttpServer>(
    HttpServerOptions{
      config_.host,
      config_.port,
      config_.max_connections,
      config_.socket_timeout_sec,
      config_.max_request_bytes},
    std::move(callbacks));
}

ApiGatewayModule::~ApiGatewayModule()
{
  stop();
}

void ApiGatewayModule::start()
{
  bool expected = false;
  if (!running_.compare_exchange_strong(expected, true, std::memory_order_acq_rel)) {
    return;
  }
  server_->start();
}

void ApiGatewayModule::stop()
{
  bool expected = true;
  if (!running_.compare_exchange_strong(expected, false, std::memory_order_acq_rel)) {
    return;
  }
  if (ports_.shutdown_socket_sessions) {
    ports_.shutdown_socket_sessions();
  }
  server_->stop();
}

bool ApiGatewayModule::running() const noexcept
{
  return running_.load(std::memory_order_acquire);
}

int ApiGatewayModule::active_connections() const noexcept
{
  return server_ ? server_->active_connections() : 0;
}

int ApiGatewayModule::max_connections() const noexcept
{
  return config_.max_connections;
}

int ApiGatewayModule::bound_port() const noexcept
{
  return server_ ? server_->bound_port() : 0;
}

bool ApiGatewayModule::token_configured() const noexcept
{
  return !config_.api_token.empty();
}

bool ApiGatewayModule::token_allowed(const HttpRequest & request) const
{
  if (config_.api_token.empty()) {
    return true;
  }
  const auto it = request.headers.find("x-robot-token");
  return it != request.headers.end() && it->second == config_.api_token;
}

std::optional<HttpResponse> ApiGatewayModule::handle_metadata(
  const HttpRequest & request) const
{
  if (request.method != "GET" || request.path != "/api/v1/openapi") {
    return std::nullopt;
  }
  const std::string body =
    "{"
    "\"ok\":true,"
    "\"endpoints\":["
    "\"GET /api/v1/status\","
    "\"GET /api/v1/robot/pose\","
    "\"GET /api/v1/maps\","
    "\"GET /api/v1/maps/semantic_layer\","
    "\"GET /api/v1/maps/poses\","
    "\"GET /api/v1/elevator-config\","
    "\"PUT /api/v1/elevator-config/draft\","
    "\"POST /api/v1/elevator-config/publish\","
    "\"POST /api/v1/elevator-config/rollback\","
    "\"POST /api/v1/elevator-test/start\","
    "\"GET /api/v1/elevator-test/state\","
    "\"POST /api/v1/elevator-test/confirm\","
    "\"POST /api/v1/elevator-test/cancel\","
    "\"POST /api/v1/elevator-test/recover\","
    "\"GET /api/v1/maps/filters/keepout\","
    "\"GET /api/v1/mapping/2d/map\","
    "\"GET /api/v1/openapi\","
    "\"POST /api/v1/maps/poses\","
    "\"PUT /api/v1/maps/poses/{pose_id}\","
    "\"DELETE /api/v1/maps/poses/{pose_id}\","
    "\"PUT /api/v1/maps/poses/batch\","
    "\"POST /api/v1/subscriptions/acquire\","
    "\"POST /api/v1/subscriptions/release\","
    "\"POST /api/v1/subscriptions/heartbeat\","
    "\"POST /api/v1/mapping/2d/start\","
    "\"POST /api/v1/mapping/2d/stop\","
    "\"POST /api/v1/mapping/2d/save\","
    "\"GET /api/v1/mapping/2d/save/status\","
    "\"POST /api/v1/mapping/stop\","
    "\"POST /api/v1/mapping/save\","
    "\"POST /api/v1/maps/delete\","
    "\"POST /api/v1/maps/poses/save\","
    "\"POST /api/v1/maps/poses/save_current\","
    "\"POST /api/v1/maps/filters/keepout/save\","
    "\"POST /api/v1/safety/stop\","
    "\"POST /api/v1/safety/resume\","
    "\"POST /api/v1/floor-switch/start\","
    "\"GET /api/v1/floor-switch/state\","
    "\"POST /api/v1/floor-switch/cancel\","
    "\"POST /api/v1/floors/switch\","
    "\"POST /api/v1/localization/trigger\","
    "\"POST /api/v1/navigation/start\","
    "\"GET /api/v1/navigation/state\","
    "\"GET /api/v1/navigation/pre_goal_check\","
    "\"POST /api/v1/navigation/goal\","
    "\"POST /api/v1/navigation/cancel\","
    "\"POST /api/v1/navigation/stop\","
    "\"POST /api/v1/navigation/stop_runtime\","
    "\"GET /api/v1/docking/state\","
    "\"POST /api/v1/docking/start\","
    "\"POST /api/v1/docking/undock\","
    "\"POST /api/v1/docking/confirm_docked\","
    "\"POST /api/v1/docking/clear_docked_latch\","
    "\"POST /api/v1/docking/cancel\","
    "\"POST /api/v1/docking/stop\","
    "\"WS /ws/v1/teleop\""
    "],"
    "\"not_wired\":["
    "\"POST /api/v1/mapping/3d/start\""
    "]"
    "}";
  return HttpResponse{200, "application/json", body};
}

HttpResponse ApiGatewayModule::dispatch_authenticated(
  const HttpRequest & request,
  const bool maintenance_peer_is_loopback) const
{
  if (request.method == "OPTIONS") {
    return {200, "application/json", "{\"ok\":true}"};
  }
  if (!token_allowed(request)) {
    return {401, "application/json", error_json("missing or invalid X-Robot-Token")};
  }
  return ports_.authenticated_route(request, maintenance_peer_is_loopback);
}

void ApiGatewayModule::log_http_request(
  const HttpRequest & request,
  const int status,
  const long latency_ms) const
{
  if (status >= 500) {
    RCLCPP_ERROR(
      logger_, "HTTP %s %s -> %d in %ld ms", request.method.c_str(),
      request.path.c_str(), status, latency_ms);
    return;
  }
  if (status >= 400) {
    RCLCPP_WARN(
      logger_, "HTTP %s %s -> %d in %ld ms", request.method.c_str(),
      request.path.c_str(), status, latency_ms);
    return;
  }
  if (latency_ms > 2000) {
    RCLCPP_WARN(
      logger_, "slow HTTP %s %s -> %d in %ld ms", request.method.c_str(),
      request.path.c_str(), status, latency_ms);
  }
}

}  // namespace robot_api_server::infrastructure::http
