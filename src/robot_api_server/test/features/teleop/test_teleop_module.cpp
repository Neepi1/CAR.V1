#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <sys/socket.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"

#include "robot_api_server/features/teleop/teleop_module.hpp"

namespace robot_api_server::features::teleop
{
namespace
{

class TeleopModuleTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestSuite()
  {
    rclcpp::shutdown();
  }

  void SetUp() override
  {
    node_ = std::make_shared<rclcpp::Node>(
      "teleop_module_test_" + std::to_string(++node_sequence_));
    fence_ = std::make_shared<ElevatorMotionAdmissionFence>();
  }

  void TearDown() override
  {
    module_.reset();
    fence_.reset();
    node_.reset();
  }

  TeleopModulePorts make_ports(
    const bool token_is_allowed,
    const bool mapping_active,
    int & admission_count)
  {
    TeleopModulePorts ports;
    ports.token_allowed = [token_is_allowed](const HttpRequest &) {
        return token_is_allowed;
      };
    ports.elevator_interlock = []() {return ElevatorExecutionInterlock{};};
    ports.capture_motion_admission_epoch = [fence = fence_]() {
        return fence->capture_epoch();
      };
    ports.acquire_motion_admission = [fence = fence_, &admission_count](
        const ElevatorMotionAdmissionFence::Epoch epoch)
      {
        ++admission_count;
        return fence->acquire_for_submission(
          epoch, []() {return ElevatorExecutionInterlock{};});
      };
    ports.charging_contact_active = []() {return false;};
    ports.mapping_snapshot = [mapping_active](const bool) {
        TeleopMappingSnapshot snapshot;
        snapshot.active = mapping_active;
        return snapshot;
      };
    ports.pose_snapshot = []() {return TeleopPoseSnapshot{};};
    ports.acquire_subscriptions = [](
      const std::string &,
      const std::vector<std::string> &,
      const std::chrono::milliseconds) {};
    ports.release_subscriptions = [](
      const std::string &,
      const std::vector<std::string> &) {};
    ports.runtime_running = []() {return false;};
    return ports;
  }

  std::string invoke_and_read(const HttpRequest & request)
  {
    int sockets[2]{-1, -1};
    EXPECT_EQ(::socketpair(AF_UNIX, SOCK_STREAM, 0, sockets), 0);
    if (sockets[0] < 0 || sockets[1] < 0) {
      return {};
    }
    EXPECT_TRUE(module_->handle_socket(sockets[0], request));
    char buffer[2048]{};
    const auto count = ::recv(sockets[1], buffer, sizeof(buffer), 0);
    ::close(sockets[0]);
    ::close(sockets[1]);
    if (count <= 0) {
      return {};
    }
    return std::string(buffer, static_cast<std::size_t>(count));
  }

  static inline int node_sequence_{0};
  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<ElevatorMotionAdmissionFence> fence_;
  std::unique_ptr<TeleopModule> module_;
};

TEST_F(TeleopModuleTest, DeclinesSocketsOutsideItsSingleRoute)
{
  int admission_count = 0;
  module_ = std::make_unique<TeleopModule>(
    *node_, TeleopModuleConfig{}, make_ports(true, true, admission_count));
  HttpRequest request;
  request.method = "GET";
  request.path = "/api/v1/status";

  EXPECT_FALSE(module_->handle_socket(-1, request));
  EXPECT_EQ(admission_count, 0);
  EXPECT_FALSE(module_->active());
  EXPECT_TRUE(module_->idle());
}

TEST_F(TeleopModuleTest, RejectsAuthenticationBeforeMotionAdmission)
{
  int admission_count = 0;
  module_ = std::make_unique<TeleopModule>(
    *node_, TeleopModuleConfig{}, make_ports(false, true, admission_count));
  HttpRequest request;
  request.method = "GET";
  request.path = "/ws/v1/teleop";

  const auto response = invoke_and_read(request);

  EXPECT_NE(response.find("HTTP/1.1 401"), std::string::npos);
  EXPECT_NE(response.find("missing or invalid X-Robot-Token"), std::string::npos);
  EXPECT_EQ(admission_count, 0);
}

TEST_F(TeleopModuleTest, RejectsInactiveMappingBeforeMotionAdmission)
{
  int admission_count = 0;
  module_ = std::make_unique<TeleopModule>(
    *node_, TeleopModuleConfig{}, make_ports(true, false, admission_count));
  HttpRequest request;
  request.method = "GET";
  request.path = "/ws/v1/teleop";
  request.headers["upgrade"] = "websocket";
  request.headers["connection"] = "Upgrade";
  request.headers["sec-websocket-key"] = "dGhlIHNhbXBsZSBub25jZQ==";

  const auto response = invoke_and_read(request);

  EXPECT_NE(response.find("HTTP/1.1 409"), std::string::npos);
  EXPECT_NE(
    response.find("WebSocket teleop is only allowed while 2D mapping is active"),
    std::string::npos);
  EXPECT_EQ(admission_count, 0);
}

TEST_F(TeleopModuleTest, OwnsAcceptedSessionLeaseAndFinalDisconnectCleanup)
{
  int admission_count = 0;
  int lease_acquire_count = 0;
  int lease_release_count = 0;
  auto ports = make_ports(true, true, admission_count);
  ports.acquire_subscriptions = [&lease_acquire_count](
      const std::string &,
      const std::vector<std::string> & resources,
      const std::chrono::milliseconds ttl)
    {
      ++lease_acquire_count;
      EXPECT_EQ(resources, (std::vector<std::string>{"teleop", "tf"}));
      EXPECT_EQ(ttl, std::chrono::milliseconds(6000));
    };
  ports.release_subscriptions = [&lease_release_count](
      const std::string &,
      const std::vector<std::string> & resources)
    {
      ++lease_release_count;
      EXPECT_EQ(resources, (std::vector<std::string>{"teleop", "tf"}));
    };
  module_ = std::make_unique<TeleopModule>(
    *node_, TeleopModuleConfig{}, std::move(ports));
  HttpRequest request;
  request.method = "GET";
  request.path = "/ws/v1/teleop";
  request.headers["upgrade"] = "websocket";
  request.headers["connection"] = "Upgrade";
  request.headers["sec-websocket-key"] = "dGhlIHNhbXBsZSBub25jZQ==";

  const auto response = invoke_and_read(request);

  EXPECT_NE(response.find("HTTP/1.1 101 Switching Protocols"), std::string::npos);
  EXPECT_EQ(admission_count, 1);
  EXPECT_EQ(lease_acquire_count, 1);
  EXPECT_EQ(lease_release_count, 1);
  EXPECT_FALSE(module_->active());
  EXPECT_TRUE(module_->idle());
}

}  // namespace
}  // namespace robot_api_server::features::teleop
