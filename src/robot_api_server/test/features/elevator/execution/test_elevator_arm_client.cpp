#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <deque>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "robot_api_server/features/elevator/execution/elevator_arm_client.hpp"

namespace robot_api_server
{
namespace
{

class ScriptedTransport final : public ElevatorArmHttpTransport
{
public:
  struct Step
  {
    std::string method;
    std::string path;
    ElevatorArmHttpResponse response;
  };

  void push(
    std::string method,
    std::string path,
    const int status,
    std::string body)
  {
    steps_.push_back(
      Step{
          std::move(method), std::move(path),
          ElevatorArmHttpResponse{status, std::move(body), {}}});
  }

  ElevatorArmHttpResponse request(
    const std::string & method,
    const std::string & path,
    const std::string & body,
    const std::chrono::milliseconds) override
  {
    requests_.push_back({method, path, body});
    if (steps_.empty()) {
      return {599, R"({"ok":false,"error":"unexpected request"})", {}};
    }
    auto step = std::move(steps_.front());
    steps_.pop_front();
    EXPECT_EQ(method, step.method);
    EXPECT_EQ(path, step.path);
    return step.response;
  }

  struct Request
  {
    std::string method;
    std::string path;
    std::string body;
  };

  const std::vector<Request> & requests() const
  {
    return requests_;
  }

  bool empty() const
  {
    return steps_.empty();
  }

private:
  std::deque<Step> steps_;
  std::vector<Request> requests_;
};

ElevatorArmClientOptions fast_options()
{
  ElevatorArmClientOptions options;
  options.request_timeout = std::chrono::milliseconds(50);
  options.task_timeout = std::chrono::milliseconds(100);
  options.poll_interval = std::chrono::milliseconds(0);
  return options;
}

void queue_succeeded_task(
  const std::shared_ptr<ScriptedTransport> & transport,
  const std::string & command_path,
  const std::string & task_id)
{
  transport->push(
    "POST", command_path, 202,
    "{\"task_id\":\"" + task_id +
    "\",\"state\":\"queued\",\"terminal\":false}");
  transport->push(
    "GET", "/api/v1/tasks/" + task_id, 200,
    "{\"task_id\":\"" + task_id +
    "\",\"state\":\"succeeded\"}");
}

TEST(ElevatorArmClient, ParsesFloorsAndDerivesHallDirectionFromTheRoute)
{
  EXPECT_EQ(ElevatorArmClient::floor_button_label("F1"), "1");
  EXPECT_EQ(ElevatorArmClient::floor_button_label("F20"), "20");
  EXPECT_EQ(ElevatorArmClient::floor_button_label("-1"), "-1");
  EXPECT_EQ(ElevatorArmClient::floor_button_label("-2"), "-2");
  EXPECT_FALSE(ElevatorArmClient::floor_button_label("B1"));
  EXPECT_FALSE(ElevatorArmClient::floor_button_label("B2"));
  EXPECT_FALSE(ElevatorArmClient::floor_button_label("b1"));
  EXPECT_FALSE(ElevatorArmClient::floor_button_label("F0"));
  EXPECT_FALSE(ElevatorArmClient::floor_button_label("floor_one"));
  EXPECT_FALSE(ElevatorArmClient::floor_button_label("F21"));

  EXPECT_EQ(ElevatorArmClient::hall_call_direction("F1", "F2"), "up");
  EXPECT_EQ(ElevatorArmClient::hall_call_direction("F2", "F1"), "down");
  EXPECT_EQ(ElevatorArmClient::hall_call_direction("-2", "-1"), "up");
  EXPECT_EQ(ElevatorArmClient::hall_call_direction("-1", "-2"), "down");
  EXPECT_EQ(ElevatorArmClient::hall_call_direction("-1", "F1"), "up");
  EXPECT_EQ(ElevatorArmClient::hall_call_direction("F1", "-1"), "down");
  EXPECT_EQ(ElevatorArmClient::hall_call_direction("-2", "1"), "up");
  EXPECT_FALSE(ElevatorArmClient::hall_call_direction("B1", "F2"));
  EXPECT_FALSE(ElevatorArmClient::hall_call_direction("F1", "B2"));
  EXPECT_FALSE(ElevatorArmClient::hall_call_direction("F2", "F2"));
  EXPECT_FALSE(ElevatorArmClient::hall_call_direction("floor_one", "F2"));
  EXPECT_FALSE(ElevatorArmClient::hall_call_direction("F1", "F21"));
}

TEST(ElevatorArmClient, FloorPressRunsReadyPressReleaseWithoutStatusGates)
{
  auto transport = std::make_shared<ScriptedTransport>();
  queue_succeeded_task(transport, "/api/v1/arm/ready", "a1");
  queue_succeeded_task(
    transport, "/api/v1/elevator/press-floor", "a2");
  queue_succeeded_task(transport, "/api/v1/arm/release", "a3");
  ElevatorArmClient client(fast_options(), transport);

  const auto result = client.press_floor(
    "elevator-test-123", 9U, "F10", []() {return false;});

  EXPECT_TRUE(result.succeeded()) << result.code << ": " << result.detail;
  EXPECT_TRUE(transport->empty());
  ASSERT_EQ(transport->requests().size(), 6U);
  EXPECT_NE(
    transport->requests()[0].body.find(
      "elevator-test-123:9:ready"),
    std::string::npos);
  EXPECT_NE(transport->requests()[2].body.find("\"floor\":\"10\""), std::string::npos);
  EXPECT_NE(
    transport->requests()[4].body.find(
      "elevator-test-123:9:release"),
    std::string::npos);
}

TEST(ElevatorArmClient, UnavailableHallCallBecomesManualAfterReleaseTask)
{
  auto transport = std::make_shared<ScriptedTransport>();
  queue_succeeded_task(transport, "/api/v1/arm/ready", "b1");
  transport->push(
    "POST", "/api/v1/elevator/call", 501,
    R"({"ok":false,"error_code":"capability_unavailable","error":"outside panel unavailable"})");
  queue_succeeded_task(transport, "/api/v1/arm/release", "b2");
  ElevatorArmClient client(fast_options(), transport);

  const auto result = client.press_hall_call(
    "elevator-test-456", 4U, "F1", "F2", []() {return false;});

  EXPECT_EQ(result.kind, ElevatorArmOutcomeKind::kManualConfirmationRequired);
  EXPECT_EQ(result.code, "ELEVATOR_CALL_MANUAL_CONFIRMATION_REQUIRED");
  EXPECT_TRUE(transport->empty());
  EXPECT_NE(transport->requests()[2].body.find("\"direction\":\"up\""), std::string::npos);
}

TEST(ElevatorArmClient, PhysicalHallCallUsesTheSingleAcceptedSubmission)
{
  auto transport = std::make_shared<ScriptedTransport>();
  queue_succeeded_task(transport, "/api/v1/arm/ready", "d1");
  queue_succeeded_task(
    transport, "/api/v1/elevator/call", "d2");
  queue_succeeded_task(transport, "/api/v1/arm/release", "d3");
  ElevatorArmClient client(fast_options(), transport);

  const auto result = client.press_hall_call(
    "elevator-test-physical", 5U, "F2", "F1", []() {return false;});

  EXPECT_TRUE(result.succeeded()) << result.code << ": " << result.detail;
  EXPECT_TRUE(transport->empty());
  EXPECT_EQ(
    std::count_if(
      transport->requests().begin(), transport->requests().end(),
      [](const ScriptedTransport::Request & request) {
        return request.method == "POST" &&
        request.path == "/api/v1/elevator/call";
      }),
    1);
  EXPECT_NE(
    transport->requests()[2].body.find("\"direction\":\"down\""),
    std::string::npos);
}

TEST(ElevatorArmClient, FailedReleaseIsReportedAsFunctionalFailure)
{
  auto transport = std::make_shared<ScriptedTransport>();
  queue_succeeded_task(transport, "/api/v1/arm/ready", "c1");
  transport->push(
    "POST", "/api/v1/elevator/call", 501,
    R"({"ok":false,"error_code":"capability_unavailable"})");
  transport->push(
    "POST", "/api/v1/arm/release", 202,
    R"({"task_id":"c2","state":"queued","terminal":false})");
  transport->push(
    "GET", "/api/v1/tasks/c2", 200,
    R"({"task_id":"c2","state":"failed","terminal":true,"ok":false,"error_code":"execution_failed","message":"release failed"})");
  ElevatorArmClient client(fast_options(), transport);

  const auto result = client.press_hall_call(
    "elevator-test-789", 4U, "F2", "F1", []() {return false;});

  EXPECT_EQ(result.kind, ElevatorArmOutcomeKind::kFailed);
  EXPECT_NE(result.code.find("ARM_RELEASE"), std::string::npos);
  EXPECT_TRUE(transport->empty());
}

TEST(ElevatorArmClient, UnknownNonterminalTaskStateDoesNotGateTheSequence)
{
  auto transport = std::make_shared<ScriptedTransport>();
  transport->push(
    "POST", "/api/v1/arm/ready", 202,
    R"({"task_id":"feature-validation-ready","state":"queued"})");
  transport->push(
    "GET", "/api/v1/tasks/feature-validation-ready", 200,
    R"({"task_id":"feature-validation-ready","state":"backend_specific_progress"})");
  transport->push(
    "GET", "/api/v1/tasks/feature-validation-ready", 200,
    R"({"task_id":"feature-validation-ready","state":"succeeded"})");
  queue_succeeded_task(
    transport, "/api/v1/elevator/press-floor", "feature-validation-press");
  queue_succeeded_task(
    transport, "/api/v1/arm/release", "feature-validation-release");
  ElevatorArmClient client(fast_options(), transport);

  const auto result = client.press_floor(
    "feature-validation", 1U, "F2", []() {return false;});

  EXPECT_TRUE(result.succeeded()) << result.code << ": " << result.detail;
  EXPECT_TRUE(transport->empty());
}

}  // namespace
}  // namespace robot_api_server
