#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <cerrno>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include "robot_api_server/infrastructure/http/http_server.hpp"

namespace
{

using robot_api_server::HttpResponse;
using robot_api_server::infrastructure::http::HttpServer;
using robot_api_server::infrastructure::http::HttpServerCallbacks;
using robot_api_server::infrastructure::http::HttpServerLogLevel;
using robot_api_server::infrastructure::http::HttpServerOptions;

int wait_for_bound_port(const HttpServer & server)
{
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (std::chrono::steady_clock::now() < deadline) {
    const int port = server.bound_port();
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
  if (::inet_pton(AF_INET, "127.0.0.1", &address.sin_addr) != 1) {
    ADD_FAILURE() << "failed to encode loopback address";
    (void)::close(client_fd);
    return {};
  }
  if (
    ::connect(client_fd, reinterpret_cast<sockaddr *>(&address), sizeof(address)) != 0)
  {
    ADD_FAILURE() << "failed to connect to HTTP test server";
    (void)::close(client_fd);
    return {};
  }

  std::size_t sent = 0U;
  while (sent < request.size()) {
    const ssize_t count = ::send(
      client_fd, request.data() + sent, request.size() - sent, MSG_NOSIGNAL);
    if (count <= 0) {
      ADD_FAILURE() << "failed to send HTTP test request";
      (void)::close(client_fd);
      return {};
    }
    sent += static_cast<std::size_t>(count);
  }
  (void)::shutdown(client_fd, SHUT_WR);

  std::string response;
  char buffer[1024];
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

TEST(HttpServerTest, DelegatesBusinessRoutingAndPreservesTransportContract)
{
  std::atomic<bool> saw_loopback{false};
  std::atomic<int> access_status{0};
  std::mutex log_mutex;
  std::vector<std::string> event_logs;

  HttpServerCallbacks callbacks;
  callbacks.route = [&saw_loopback](const auto & request, const bool loopback) {
      saw_loopback.store(loopback, std::memory_order_release);
      return HttpResponse{
        200,
        "application/json",
        "{\"path\":\"" + request.path + "\",\"query\":\"" +
        request.query.at("probe") + "\"}"};
    };
  callbacks.socket_route = [](const int, const auto &) {return false;};
  callbacks.access_log = [&access_status](const auto &, const int status, const long) {
      access_status.store(status, std::memory_order_release);
    };
  callbacks.event_log = [&log_mutex, &event_logs](
    const HttpServerLogLevel, const std::string & message) {
      std::lock_guard<std::mutex> lock(log_mutex);
      event_logs.push_back(message);
    };

  HttpServer server(
    HttpServerOptions{"127.0.0.1", 0, 4, 2, 1024U * 1024U},
    std::move(callbacks));
  server.start();
  const int port = wait_for_bound_port(server);
  ASSERT_GT(port, 0);

  const std::string response = exchange_http(
    port,
    "GET /api/v1/status?probe=ok HTTP/1.1\r\nHost: 127.0.0.1\r\n\r\n");
  EXPECT_NE(response.find("HTTP/1.1 200 OK"), std::string::npos);
  EXPECT_NE(response.find("Access-Control-Allow-Origin: *"), std::string::npos);
  EXPECT_NE(
    response.find("{\"path\":\"/api/v1/status\",\"query\":\"ok\"}"),
    std::string::npos);
  EXPECT_TRUE(saw_loopback.load(std::memory_order_acquire));
  EXPECT_EQ(access_status.load(std::memory_order_acquire), 200);
  EXPECT_EQ(server.max_connections(), 4);

  server.stop();
  EXPECT_FALSE(server.running());
  EXPECT_EQ(server.active_connections(), 0);
  EXPECT_EQ(server.bound_port(), 0);
  server.stop();

  std::lock_guard<std::mutex> lock(log_mutex);
  ASSERT_FALSE(event_logs.empty());
  EXPECT_NE(event_logs.front().find("listening on 127.0.0.1:"), std::string::npos);
}

TEST(HttpServerTest, GivesSocketRouteExclusiveSynchronousOwnership)
{
  std::atomic<int> normal_route_calls{0};
  std::atomic<int> socket_route_calls{0};
  HttpServerCallbacks callbacks;
  callbacks.route = [&normal_route_calls](const auto &, const bool) {
      normal_route_calls.fetch_add(1, std::memory_order_acq_rel);
      return HttpResponse{500, "application/json", "{}"};
    };
  callbacks.socket_route = [&socket_route_calls](const int fd, const auto & request) {
      if (request.path != "/ws/v1/teleop") {
        return false;
      }
      socket_route_calls.fetch_add(1, std::memory_order_acq_rel);
      EXPECT_TRUE(HttpServer::write_response(
        fd, {409, "application/json", "{\"handled\":true}"}));
      return true;
    };

  HttpServer server(
    HttpServerOptions{"127.0.0.1", 0, 4, 2, 1024U * 1024U},
    std::move(callbacks));
  server.start();
  const int port = wait_for_bound_port(server);
  ASSERT_GT(port, 0);

  const std::string response = exchange_http(
    port,
    "GET /ws/v1/teleop HTTP/1.1\r\nHost: 127.0.0.1\r\n\r\n");
  EXPECT_NE(response.find("HTTP/1.1 409 Conflict"), std::string::npos);
  EXPECT_NE(response.find("{\"handled\":true}"), std::string::npos);
  EXPECT_EQ(socket_route_calls.load(std::memory_order_acquire), 1);
  EXPECT_EQ(normal_route_calls.load(std::memory_order_acquire), 0);

  server.stop();
  EXPECT_EQ(server.active_connections(), 0);
}

}  // namespace
