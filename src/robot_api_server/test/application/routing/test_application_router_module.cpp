#include <gtest/gtest.h>

#include <cstdint>
#include <chrono>
#include <future>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "robot_api_server/application/routing/application_router_module.hpp"

namespace routing = robot_api_server::application::routing;

namespace
{

using robot_api_server::HttpRequest;
using robot_api_server::HttpResponse;

HttpRequest request_for(const std::string & method, const std::string & path)
{
  HttpRequest request;
  request.method = method;
  request.path = path;
  return request;
}

routing::ApplicationRouterModulePorts make_ports(std::vector<std::string> & calls)
{
  routing::ApplicationRouterModulePorts ports;
  ports.capture_motion_admission_epoch = [&calls]() {
      calls.emplace_back("epoch");
      return std::uint64_t{42U};
    };
  ports.elevator_interlock = [&calls](const auto &) {
      calls.emplace_back("interlock");
      return std::optional<HttpResponse>{};
    };
  ports.system_status = [&calls](const auto &) {
      calls.emplace_back("system_status");
      return std::optional<HttpResponse>{};
    };
  ports.maps = [&calls](const auto &, const auto epoch) {
      EXPECT_EQ(epoch, 0U);
      calls.emplace_back("maps");
      return std::optional<HttpResponse>{};
    };
  ports.elevator = [&calls](const auto &, const auto epoch, const bool, const bool) {
      EXPECT_EQ(epoch, 0U);
      calls.emplace_back("elevator");
      return std::optional<HttpResponse>{};
    };
  ports.mapping = [&calls](const auto &, const auto epoch) {
      EXPECT_EQ(epoch, 0U);
      calls.emplace_back("mapping");
      return std::optional<HttpResponse>{};
    };
  ports.gateway_metadata = [&calls](const auto &) {
      calls.emplace_back("metadata");
      return std::optional<HttpResponse>{};
    };
  ports.subscriptions = [&calls](const auto &) {
      calls.emplace_back("subscriptions");
      return std::optional<HttpResponse>{};
    };
  ports.safety = [&calls](const auto &, const auto epoch) {
      EXPECT_EQ(epoch, 0U);
      calls.emplace_back("safety");
      return std::optional<HttpResponse>{};
    };
  ports.floor_switch = [&calls](const auto &, const auto epoch) {
      EXPECT_EQ(epoch, 0U);
      calls.emplace_back("floor_switch");
      return std::optional<HttpResponse>{};
    };
  ports.localization = [&calls](const auto &, const auto epoch) {
      EXPECT_EQ(epoch, 0U);
      calls.emplace_back("localization");
      return std::optional<HttpResponse>{};
    };
  ports.navigation = [&calls](const auto &, const auto epoch) {
      EXPECT_EQ(epoch, 0U);
      calls.emplace_back("navigation");
      return std::optional<HttpResponse>{};
    };
  ports.docking = [&calls](const auto &, const auto epoch) {
      EXPECT_EQ(epoch, 0U);
      calls.emplace_back("docking");
      return std::optional<HttpResponse>{};
    };
  ports.gateway_token_configured = []() {return true;};
  return ports;
}

TEST(ApplicationRouterModuleTest, ElevatorInterlockAppliesOnlyToElevatorTest)
{
  std::vector<std::string> calls;
  auto ports = make_ports(calls);
  ports.elevator_interlock = [&calls](const auto &) {
      calls.emplace_back("interlock");
      return std::optional<HttpResponse>{HttpResponse{423, "application/json", "blocked"}};
    };
  routing::ApplicationRouterModule router(std::move(ports));

  const auto response = router.route(
    request_for("POST", "/api/v1/elevator-test/start"), false);

  EXPECT_EQ(response.status, 423);
  EXPECT_EQ(response.body, "blocked");
  EXPECT_EQ(calls, (std::vector<std::string>{"epoch", "interlock"}));
}

TEST(ApplicationRouterModuleTest, StatusDoesNotWaitForAnElevatorTestSubmission)
{
  using namespace std::chrono_literals;
  std::vector<std::string> calls;
  auto ports = make_ports(calls);
  std::mutex elevator_submission;
  std::unique_lock<std::mutex> active_test(elevator_submission);
  ports.capture_motion_admission_epoch = [&]() {
      std::lock_guard<std::mutex> lock(elevator_submission);
      return std::uint64_t{42U};
    };
  ports.system_status = [](const auto &) {
      return std::optional<HttpResponse>{HttpResponse{200, "application/json", "status"}};
    };
  routing::ApplicationRouterModule router(std::move(ports));
  auto response = std::async(std::launch::async, [&]() {
      return router.route(request_for("GET", "/api/v1/status"), false);
    });
  const auto before_release = response.wait_for(100ms);
  // Always release the fixture before assertions; a failing test cannot hang.
  active_test.unlock();
  EXPECT_EQ(before_release, std::future_status::ready);
  EXPECT_EQ(response.get().status, 200);
}

TEST(ApplicationRouterModuleTest, PreservesCompleteHandlerOrderWithoutGlobalEpoch)
{
  std::vector<std::string> calls;
  auto ports = make_ports(calls);
  ports.elevator = [&calls](
    const auto &, const auto epoch, const bool token, const bool loopback) {
      EXPECT_EQ(epoch, 0U);
      EXPECT_TRUE(token);
      EXPECT_TRUE(loopback);
      calls.emplace_back("elevator");
      return std::optional<HttpResponse>{};
    };
  ports.docking = [&calls](const auto &, const auto epoch) {
      EXPECT_EQ(epoch, 0U);
      calls.emplace_back("docking");
      return std::optional<HttpResponse>{HttpResponse{202, "application/json", "dock"}};
    };
  routing::ApplicationRouterModule router(std::move(ports));

  const auto response = router.route(request_for("POST", "/api/v1/docking/start"), true);

  EXPECT_EQ(response.status, 202);
  EXPECT_EQ(response.body, "dock");
  EXPECT_EQ(
    calls,
    (std::vector<std::string>{
      "system_status", "maps", "elevator", "mapping",
      "metadata", "subscriptions", "safety", "floor_switch", "localization",
      "navigation", "docking"}));
}

TEST(ApplicationRouterModuleTest, StopsAtFirstMatchingFeature)
{
  std::vector<std::string> calls;
  auto ports = make_ports(calls);
  ports.system_status = [&calls](const auto &) {
      calls.emplace_back("system_status");
      return std::optional<HttpResponse>{HttpResponse{200, "application/json", "status"}};
    };
  routing::ApplicationRouterModule router(std::move(ports));

  const auto response = router.route(request_for("GET", "/api/v1/status"), false);

  EXPECT_EQ(response.body, "status");
  EXPECT_EQ(calls, (std::vector<std::string>{"system_status"}));
}

TEST(ApplicationRouterModuleTest, PreservesReservedAndUnknownFallbacks)
{
  std::vector<std::string> calls;
  routing::ApplicationRouterModule router(make_ports(calls));

  const auto reserved = router.route(
    request_for("POST", "/api/v1/mapping/3d/start"), false);
  EXPECT_EQ(reserved.status, 501);
  EXPECT_NE(reserved.body.find("endpoint is reserved"), std::string::npos);
  EXPECT_NE(reserved.body.find("/api/v1/mapping/3d/start"), std::string::npos);

  calls.clear();
  const auto missing = router.route(request_for("GET", "/api/v1/unknown"), false);
  EXPECT_EQ(missing.status, 404);
  EXPECT_NE(missing.body.find("endpoint not found: /api/v1/unknown"), std::string::npos);
}

TEST(ApplicationRouterModuleTest, RejectsIncompleteRoutingGraph)
{
  routing::ApplicationRouterModulePorts ports;
  EXPECT_THROW(routing::ApplicationRouterModule router(std::move(ports)), std::invalid_argument);
}

}  // namespace
