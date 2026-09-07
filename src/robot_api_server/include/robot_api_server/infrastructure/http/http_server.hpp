#pragma once

#include <cstddef>
#include <functional>
#include <memory>
#include <string>

#include "robot_api_server/infrastructure/http/http_common.hpp"

namespace robot_api_server::infrastructure::http
{

enum class HttpServerLogLevel
{
  kInfo,
  kWarning,
  kError,
};

struct HttpServerOptions
{
  std::string host{"0.0.0.0"};
  int port{8080};
  int max_connections{16};
  int socket_timeout_sec{15};
  std::size_t max_request_bytes{1024U * 1024U};
};

struct HttpServerCallbacks
{
  using Route = std::function<HttpResponse(const HttpRequest &, bool)>;
  using SocketRoute = std::function<bool(int, const HttpRequest &)>;
  using AccessLog = std::function<void(const HttpRequest &, int, long)>;
  using EventLog = std::function<void(HttpServerLogLevel, const std::string &)>;

  Route route;
  SocketRoute socket_route;
  AccessLog access_log;
  EventLog event_log;
};

// Owns only HTTP transport concerns: listening socket, bounded worker pool,
// request framing, connection accounting, and response framing. Business
// routing and WebSocket session behavior are injected through callbacks.
class HttpServer
{
public:
  HttpServer(HttpServerOptions options, HttpServerCallbacks callbacks);
  ~HttpServer();

  HttpServer(const HttpServer &) = delete;
  HttpServer & operator=(const HttpServer &) = delete;
  HttpServer(HttpServer &&) = delete;
  HttpServer & operator=(HttpServer &&) = delete;

  void start();
  void stop();

  bool running() const noexcept;
  int active_connections() const noexcept;
  int max_connections() const noexcept;
  int bound_port() const noexcept;

  // Used by an injected socket route when it must reject an upgrade request
  // with a normal HTTP response before taking over the connection.
  static bool write_response(int client_fd, const HttpResponse & response);

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace robot_api_server::infrastructure::http
