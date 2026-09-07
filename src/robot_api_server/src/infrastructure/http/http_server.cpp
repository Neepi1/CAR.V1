#include "robot_api_server/infrastructure/http/http_server.hpp"

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstring>
#include <deque>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <utility>
#include <vector>

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

namespace robot_api_server::infrastructure::http
{
namespace
{

bool send_all(const int client_fd, const std::string & text)
{
  std::size_t sent = 0U;
  while (sent < text.size()) {
    const ssize_t count = ::send(
      client_fd, text.data() + sent, text.size() - sent, MSG_NOSIGNAL);
    if (count < 0 && errno == EINTR) {
      continue;
    }
    if (count <= 0) {
      return false;
    }
    sent += static_cast<std::size_t>(count);
  }
  return true;
}

void set_close_on_exec(const int fd)
{
  const int flags = ::fcntl(fd, F_GETFD);
  if (flags >= 0) {
    (void)::fcntl(fd, F_SETFD, flags | FD_CLOEXEC);
  }
}

bool peer_is_loopback(const int client_fd)
{
  sockaddr_in peer{};
  socklen_t peer_length = sizeof(peer);
  if (
    ::getpeername(
      client_fd, reinterpret_cast<sockaddr *>(&peer), &peer_length) != 0 ||
    peer.sin_family != AF_INET)
  {
    return false;
  }
  return (ntohl(peer.sin_addr.s_addr) & 0xff000000U) == 0x7f000000U;
}

class ScopedClientFd
{
public:
  explicit ScopedClientFd(const int fd)
  : fd_(fd)
  {
  }

  ~ScopedClientFd()
  {
    if (fd_ >= 0) {
      (void)::close(fd_);
    }
  }

  ScopedClientFd(const ScopedClientFd &) = delete;
  ScopedClientFd & operator=(const ScopedClientFd &) = delete;

private:
  int fd_{-1};
};

}  // namespace

class HttpServer::Impl
{
public:
  Impl(HttpServerOptions options, HttpServerCallbacks callbacks)
  : options_(std::move(options)), callbacks_(std::move(callbacks))
  {
    if (!callbacks_.route) {
      throw std::invalid_argument("HttpServer requires a route callback");
    }
    options_.max_connections = std::max(1, options_.max_connections);
    options_.socket_timeout_sec = std::max(1, options_.socket_timeout_sec);
    options_.max_request_bytes = std::max<std::size_t>(4096U, options_.max_request_bytes);
  }

  ~Impl()
  {
    stop();
  }

  void start()
  {
    bool expected = false;
    if (!running_.compare_exchange_strong(expected, true, std::memory_order_acq_rel)) {
      return;
    }
    bound_port_.store(0, std::memory_order_release);
    start_workers();
    server_thread_ = std::thread([this]() { serve(); });
  }

  void stop()
  {
    running_.store(false, std::memory_order_release);
    const int server_fd = server_fd_.exchange(-1, std::memory_order_acq_rel);
    if (server_fd >= 0) {
      (void)::shutdown(server_fd, SHUT_RDWR);
      (void)::close(server_fd);
    }
    if (server_thread_.joinable()) {
      server_thread_.join();
    }
    stop_workers();
    bound_port_.store(0, std::memory_order_release);
  }

  bool running() const noexcept
  {
    return running_.load(std::memory_order_acquire);
  }

  int active_connections() const noexcept
  {
    return active_connections_.load(std::memory_order_acquire);
  }

  int max_connections() const noexcept
  {
    return options_.max_connections;
  }

  int bound_port() const noexcept
  {
    return bound_port_.load(std::memory_order_acquire);
  }

private:
  void log(const HttpServerLogLevel level, const std::string & message) const noexcept
  {
    if (!callbacks_.event_log) {
      return;
    }
    try {
      callbacks_.event_log(level, message);
    } catch (...) {
      // Logging must never terminate the transport worker.
    }
  }

  void start_workers()
  {
    workers_.clear();
    workers_.reserve(static_cast<std::size_t>(options_.max_connections));
    for (int index = 0; index < options_.max_connections; ++index) {
      workers_.emplace_back([this]() { worker_loop(); });
    }
  }

  void stop_workers()
  {
    {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      while (!client_queue_.empty()) {
        (void)::close(client_queue_.front());
        client_queue_.pop_front();
        active_connections_.fetch_sub(1, std::memory_order_acq_rel);
      }
    }
    queue_cv_.notify_all();
    for (auto & worker : workers_) {
      if (worker.joinable()) {
        worker.join();
      }
    }
    workers_.clear();
  }

  void enqueue_client(const int client_fd)
  {
    {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      if (!running_.load(std::memory_order_acquire)) {
        (void)::close(client_fd);
        active_connections_.fetch_sub(1, std::memory_order_acq_rel);
        return;
      }
      client_queue_.push_back(client_fd);
    }
    queue_cv_.notify_one();
  }

  void worker_loop() noexcept
  {
    while (true) {
      int client_fd = -1;
      {
        std::unique_lock<std::mutex> lock(queue_mutex_);
        queue_cv_.wait(lock, [this]() {
          return !running_.load(std::memory_order_acquire) || !client_queue_.empty();
        });
        if (client_queue_.empty()) {
          if (!running_.load(std::memory_order_acquire)) {
            return;
          }
          continue;
        }
        client_fd = client_queue_.front();
        client_queue_.pop_front();
      }

      struct ActiveConnectionGuard
      {
        std::atomic<int> & counter;
        ~ActiveConnectionGuard()
        {
          counter.fetch_sub(1, std::memory_order_acq_rel);
        }
      } active_guard{active_connections_};

      if (!running_.load(std::memory_order_acquire)) {
        (void)::close(client_fd);
        continue;
      }
      try {
        handle_client(client_fd);
      } catch (const std::exception & error) {
        log(HttpServerLogLevel::kError, std::string("HTTP worker failed: ") + error.what());
      } catch (...) {
        log(HttpServerLogLevel::kError, "HTTP worker failed with an unknown exception");
      }
    }
  }

  void close_server_fd(const int local_fd)
  {
    int expected = local_fd;
    if (server_fd_.compare_exchange_strong(expected, -1, std::memory_order_acq_rel)) {
      (void)::close(local_fd);
    }
  }

  void serve()
  {
    const int local_server_fd = ::socket(AF_INET, SOCK_STREAM, 0);
    if (local_server_fd < 0) {
      log(
        HttpServerLogLevel::kError,
        std::string("failed to create API socket: ") + std::strerror(errno));
      return;
    }
    server_fd_.store(local_server_fd, std::memory_order_release);
    set_close_on_exec(local_server_fd);

    int reuse = 1;
    (void)::setsockopt(local_server_fd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_port = htons(static_cast<uint16_t>(options_.port));
    if (options_.host == "0.0.0.0" || options_.host.empty()) {
      address.sin_addr.s_addr = INADDR_ANY;
    } else if (::inet_pton(AF_INET, options_.host.c_str(), &address.sin_addr) != 1) {
      log(HttpServerLogLevel::kError, "invalid API host: " + options_.host);
      close_server_fd(local_server_fd);
      return;
    }

    if (
      ::bind(local_server_fd, reinterpret_cast<sockaddr *>(&address), sizeof(address)) < 0)
    {
      std::ostringstream message;
      message << "failed to bind API server on " << options_.host << ":" << options_.port
              << ": " << std::strerror(errno);
      log(HttpServerLogLevel::kError, message.str());
      close_server_fd(local_server_fd);
      return;
    }
    if (::listen(local_server_fd, 64) < 0) {
      log(
        HttpServerLogLevel::kError,
        std::string("failed to listen on API socket: ") + std::strerror(errno));
      close_server_fd(local_server_fd);
      return;
    }

    sockaddr_in bound_address{};
    socklen_t bound_length = sizeof(bound_address);
    if (
      ::getsockname(
        local_server_fd, reinterpret_cast<sockaddr *>(&bound_address), &bound_length) == 0)
    {
      bound_port_.store(ntohs(bound_address.sin_port), std::memory_order_release);
    } else {
      bound_port_.store(options_.port, std::memory_order_release);
    }

    {
      std::ostringstream message;
      message << "robot_api_server listening on " << options_.host << ":"
              << bound_port_.load(std::memory_order_acquire);
      log(HttpServerLogLevel::kInfo, message.str());
    }

    while (running_.load(std::memory_order_acquire)) {
      sockaddr_in client_address{};
      socklen_t client_length = sizeof(client_address);
      const int client_fd = ::accept(
        local_server_fd, reinterpret_cast<sockaddr *>(&client_address), &client_length);
      if (client_fd < 0) {
        if (running_.load(std::memory_order_acquire)) {
          log(
            HttpServerLogLevel::kWarning,
            std::string("API accept failed: ") + std::strerror(errno));
        }
        continue;
      }

      set_close_on_exec(client_fd);
      timeval timeout{};
      timeout.tv_sec = options_.socket_timeout_sec;
      timeout.tv_usec = 0;
      (void)::setsockopt(client_fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
      (void)::setsockopt(client_fd, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));

      if (
        active_connections_.load(std::memory_order_relaxed) >=
        options_.max_connections)
      {
        const auto now = std::chrono::steady_clock::now();
        bool should_log = false;
        {
          std::lock_guard<std::mutex> lock(busy_log_mutex_);
          if (
            last_busy_log_at_ == std::chrono::steady_clock::time_point{} ||
            now - last_busy_log_at_ >= std::chrono::seconds(5))
          {
            last_busy_log_at_ = now;
            should_log = true;
          }
        }
        if (should_log) {
          log(
            HttpServerLogLevel::kWarning,
            "rejecting HTTP client: active connections reached limit " +
            std::to_string(options_.max_connections));
        }
        (void)HttpServer::write_response(
          client_fd, {503, "application/json", error_json("server busy")});
        (void)::close(client_fd);
        continue;
      }

      active_connections_.fetch_add(1, std::memory_order_acq_rel);
      enqueue_client(client_fd);
    }
  }

  void handle_client(const int client_fd)
  {
    ScopedClientFd client_guard(client_fd);
    std::string raw;
    char buffer[4096];
    std::size_t expected_body = 0U;

    while (running_.load(std::memory_order_acquire)) {
      const ssize_t count = ::recv(client_fd, buffer, sizeof(buffer), 0);
      if (count <= 0) {
        break;
      }
      raw.append(buffer, static_cast<std::size_t>(count));
      const auto header_end = raw.find("\r\n\r\n");
      if (header_end != std::string::npos) {
        expected_body = content_length_from_headers(raw.substr(0, header_end));
        const auto current_body = raw.size() - header_end - 4U;
        if (current_body >= expected_body) {
          break;
        }
      }
      if (raw.size() > options_.max_request_bytes) {
        (void)HttpServer::write_response(
          client_fd, {400, "application/json", error_json("request too large")});
        return;
      }
    }

    const auto request = parse_http_request(raw);
    if (!request) {
      (void)HttpServer::write_response(
        client_fd, {400, "application/json", error_json("invalid HTTP request")});
      return;
    }

    if (callbacks_.socket_route && callbacks_.socket_route(client_fd, *request)) {
      return;
    }

    const auto started_at = std::chrono::steady_clock::now();
    HttpResponse response;
    try {
      response = callbacks_.route(*request, peer_is_loopback(client_fd));
    } catch (const std::exception & error) {
      response = {
        500,
        "application/json",
        error_json(std::string("unhandled API exception: ") + error.what())};
    } catch (...) {
      response = {500, "application/json", error_json("unknown unhandled API exception")};
    }
    const auto latency_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now() - started_at).count();
    if (callbacks_.access_log) {
      callbacks_.access_log(*request, response.status, latency_ms);
    }
    if (!HttpServer::write_response(client_fd, response)) {
      std::ostringstream message;
      message << "failed to send full HTTP response status=" << response.status
              << " bytes=" << response.body.size();
      log(HttpServerLogLevel::kWarning, message.str());
    }
  }

  HttpServerOptions options_;
  HttpServerCallbacks callbacks_;
  std::atomic<bool> running_{false};
  std::atomic<int> active_connections_{0};
  std::atomic<int> server_fd_{-1};
  std::atomic<int> bound_port_{0};
  std::thread server_thread_;
  std::vector<std::thread> workers_;
  std::mutex queue_mutex_;
  std::condition_variable queue_cv_;
  std::deque<int> client_queue_;
  std::mutex busy_log_mutex_;
  std::chrono::steady_clock::time_point last_busy_log_at_{};
};

HttpServer::HttpServer(HttpServerOptions options, HttpServerCallbacks callbacks)
: impl_(std::make_unique<Impl>(std::move(options), std::move(callbacks)))
{
}

HttpServer::~HttpServer() = default;

void HttpServer::start()
{
  impl_->start();
}

void HttpServer::stop()
{
  impl_->stop();
}

bool HttpServer::running() const noexcept
{
  return impl_->running();
}

int HttpServer::active_connections() const noexcept
{
  return impl_->active_connections();
}

int HttpServer::max_connections() const noexcept
{
  return impl_->max_connections();
}

int HttpServer::bound_port() const noexcept
{
  return impl_->bound_port();
}

bool HttpServer::write_response(const int client_fd, const HttpResponse & response)
{
  std::ostringstream out;
  out << "HTTP/1.1 " << response.status << " " << reason_phrase(response.status) << "\r\n";
  out << "Content-Type: " << response.content_type << "\r\n";
  out << "Content-Length: " << response.body.size() << "\r\n";
  out << "Connection: close\r\n";
  out << "Access-Control-Allow-Origin: *\r\n";
  out << "Access-Control-Allow-Methods: GET, POST, PUT, DELETE, OPTIONS\r\n";
  out << "Access-Control-Allow-Headers: Content-Type, X-Robot-Token\r\n";
  out << "\r\n";
  out << response.body;
  return send_all(client_fd, out.str());
}

}  // namespace robot_api_server::infrastructure::http
