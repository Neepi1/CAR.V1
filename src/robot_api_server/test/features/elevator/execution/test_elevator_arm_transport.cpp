#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <functional>
#include <iostream>
#include <stdexcept>
#include <thread>
#include <vector>

#include <arpa/inet.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include "robot_api_server/features/elevator/execution/elevator_arm_client.hpp"

namespace robot_api_server
{
namespace
{
using namespace std::chrono_literals;

class LocalServer
{
public:
  using Handler = std::function<void(int, const std::string &, const std::atomic<bool> &)>;

  explicit LocalServer(Handler handler)
  {
    listener_ = ::socket(AF_INET, SOCK_STREAM, 0);
    if (listener_ < 0) {throw std::runtime_error("test socket failed");}
    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    address.sin_port = 0;  // Kernel-assigned test port; never contact the arm.
    socklen_t length = sizeof(address);
    if (::bind(listener_, reinterpret_cast<sockaddr *>(&address), length) != 0 ||
      ::getsockname(listener_, reinterpret_cast<sockaddr *>(&address), &length) != 0 ||
      ::listen(listener_, 1) != 0)
    {
      ::close(listener_);
      throw std::runtime_error("test listen failed");
    }
    port_ = ntohs(address.sin_port);
    worker_ = std::thread([this, handler]() {
        while (!stopping_.load()) {
          pollfd descriptor{listener_, POLLIN, 0};
          if (::poll(&descriptor, 1, 20) <= 0) {continue;}
          const int client = ::accept(listener_, nullptr, nullptr);
          if (client >= 0) {
            try {
              const auto request = read_request(client);
              requests_.push_back(request);
              handler(client, request, stopping_);
            } catch (...) {
              // A mock-server error must not escape a std::thread boundary.
              failed_.store(true);
            }
            ::close(client);
          }
        }
      });
  }

  ~LocalServer()
  {
    stop();
    ::close(listener_);
  }

  std::uint16_t port() const {return port_;}
  void stop()
  {
    stopping_.store(true);
    if (worker_.joinable()) {worker_.join();}
  }
  const std::vector<std::string> & requests() const {return requests_;}
  bool failed() const {return failed_.load();}

private:
  std::string read_request(const int client)
  {
    std::string result;
    const auto deadline = std::chrono::steady_clock::now() + 2s;
    while (!stopping_.load() && std::chrono::steady_clock::now() < deadline) {
      pollfd pending{client, POLLIN, 0};
      if (::poll(&pending, 1, 20) <= 0) {continue;}
      char buffer[4096];
      const auto count = ::recv(client, buffer, sizeof(buffer), 0);
      if (count <= 0) {break;}
      result.append(buffer, static_cast<std::size_t>(count));
      const auto header_end = result.find("\r\n\r\n");
      if (header_end == std::string::npos) {continue;}
      const auto length_field = result.find("Content-Length: ");
      const auto length = length_field == std::string::npos ? 0U :
        std::stoul(result.substr(length_field + 16U));
      if (result.size() >= header_end + 4U + length) {return result;}
    }
    throw std::runtime_error("mock request incomplete");
  }

  int listener_{-1};
  std::uint16_t port_{0};
  std::atomic<bool> stopping_{false};
  std::atomic<bool> failed_{false};
  std::thread worker_;
  std::vector<std::string> requests_;
};

void reply_json(const int client, const int status, const std::string & body)
{
  const std::string response = "HTTP/1.0 " + std::to_string(status) +
    " Test\r\nContent-Length: " + std::to_string(body.size()) + "\r\n\r\n" + body;
  (void)::send(client, response.data(), response.size(), MSG_NOSIGNAL);
}

void successful_task_reply(const int client, const std::string & request)
{
  if (request.find("GET /api/v1/tasks/") == 0U) {
    reply_json(client, 200, R"({"state":"succeeded"})");
  } else if (request.find("POST /api/v1/arm/ready ") == 0U) {
    reply_json(client, 202, R"({"task_id":"ready"})");
  } else if (request.find("POST /api/v1/elevator/press-floor ") == 0U) {
    reply_json(client, 202, R"({"task_id":"press"})");
  } else if (request.find("POST /api/v1/arm/release ") == 0U) {
    reply_json(client, 202, R"({"task_id":"release"})");
  } else {
    reply_json(client, 400, R"({"error_code":"unexpected_mock_request"})");
  }
}

TEST(ElevatorArmTransport, DrippingResponseSharesOneRequestDeadline)
{
  LocalServer server([](int client, const std::string & request,
      const std::atomic<bool> & stopping) {
      if (request.find("POST /api/v1/arm/release ") == 0U) {
        reply_json(client, 202, R"({"task_id":"release"})");
        return;
      }
      if (request.find("GET /api/v1/tasks/release ") == 0U) {
        reply_json(client, 200, R"({"task_id":"release","state":"succeeded"})");
        return;
      }
      const std::string header = "HTTP/1.0 200 OK\r\n\r\n";
      (void)::send(client, header.data(), header.size(), MSG_NOSIGNAL);
      for (int i = 0; i < 20 && !stopping.load(); ++i) {
        if (::send(client, "x", 1, MSG_NOSIGNAL) <= 0) {break;}
        std::this_thread::sleep_for(50ms);
      }
    });
  ElevatorArmClientOptions options;
  options.port = server.port();
  options.request_timeout = 200ms;
  options.task_timeout = 500ms;
  options.poll_interval = 0ms;
  ASSERT_NE(options.port, 8083U);
  ElevatorArmClient arm(options);
  const auto started = std::chrono::steady_clock::now();
  const auto result = arm.press_floor("transport-test", 1U, "F2", []() {return false;});
  const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::steady_clock::now() - started);
  std::cout << "dripping request elapsed_ms=" << elapsed.count() << '\n';
  server.stop();
  EXPECT_FALSE(server.failed());
  EXPECT_LT(elapsed, 600ms);
  EXPECT_FALSE(result.succeeded());
  EXPECT_NE(result.detail.find("request deadline exceeded"), std::string::npos);
}

TEST(ElevatorArmTransport, NormalReadyPressReleaseKeepsGetAndPostPayloads)
{
  LocalServer server([](int client, const std::string & request,
      const std::atomic<bool> &) {successful_task_reply(client, request);});
  ElevatorArmClientOptions options;
  options.port = server.port();
  options.request_timeout = 500ms;
  options.poll_interval = 0ms;
  ASSERT_NE(options.port, 8083U);
  ElevatorArmClient arm(options);
  const auto result = arm.press_floor("transport-normal", 7U, "F2", []() {return false;});
  server.stop();
  ASSERT_TRUE(result.succeeded()) << result.code << ": " << result.detail;
  EXPECT_FALSE(server.failed());
  const auto & requests = server.requests();
  ASSERT_EQ(requests.size(), 6U);
  EXPECT_EQ(requests[0].find("POST /api/v1/arm/ready HTTP/1.0"), 0U);
  EXPECT_EQ(requests[1].find("GET /api/v1/tasks/ready HTTP/1.0"), 0U);
  EXPECT_NE(requests[2].find(R"("floor":"2")"), std::string::npos);
  EXPECT_EQ(requests[3].find("GET /api/v1/tasks/press HTTP/1.0"), 0U);
  EXPECT_EQ(requests[4].find("POST /api/v1/arm/release HTTP/1.0"), 0U);
  EXPECT_EQ(requests[5].find("GET /api/v1/tasks/release HTTP/1.0"), 0U);
}

TEST(ElevatorArmTransport, ResponseByteLimitRemainsEnforced)
{
  LocalServer server([](int client, const std::string & request,
      const std::atomic<bool> &) {
      if (request.find("POST /api/v1/arm/ready ") == 0U) {
        reply_json(client, 202, "{\"padding\":\"" + std::string(2048, 'x') + "\"}");
      } else {
        successful_task_reply(client, request);
      }
    });
  ElevatorArmClientOptions options;
  options.port = server.port();
  options.request_timeout = 500ms;
  options.maximum_response_bytes = 512;
  options.poll_interval = 0ms;
  ASSERT_NE(options.port, 8083U);
  ElevatorArmClient arm(options);
  const auto result = arm.press_floor("transport-limit", 1U, "F2", []() {return false;});
  server.stop();
  EXPECT_FALSE(server.failed());
  EXPECT_FALSE(result.succeeded());
  EXPECT_NE(result.detail.find("exceeds configured size limit"), std::string::npos);
}

TEST(ElevatorArmTransport, NonLoopbackHostIsRejectedBeforeAnyRequest)
{
  ElevatorArmClientOptions options;
  options.host = "192.0.2.1";
  EXPECT_THROW(ElevatorArmClient{options}, std::invalid_argument);
}

}  // namespace
}  // namespace robot_api_server
