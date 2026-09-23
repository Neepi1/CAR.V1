#include <gtest/gtest.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <deque>
#include <memory>
#include <functional>
#include <thread>
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
    std::function<void()> before_reply;
  };

  void push(
    std::string method,
    std::string path,
    const int status,
    std::string body,
    std::function<void()> before_reply = {},
    std::string transport_error = {})
  {
    steps_.push_back(
      Step{
          std::move(method), std::move(path),
          ElevatorArmHttpResponse{status, std::move(body), std::move(transport_error)},
          std::move(before_reply)});
  }

  ElevatorArmHttpResponse request(
    const std::string & method,
    const std::string & path,
    const std::string & body,
    const std::chrono::milliseconds timeout) override
  {
    timeouts_.push_back(timeout);
    requests_.push_back({method, path, body});
    if (steps_.empty()) {
      return {599, R"({"ok":false,"error":"unexpected request"})", {}};
    }
    auto step = std::move(steps_.front());
    steps_.pop_front();
    EXPECT_EQ(method, step.method);
    EXPECT_EQ(path, step.path);
    if (step.before_reply) {
      step.before_reply();
    }
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

  const std::vector<std::chrono::milliseconds> & timeouts() const {return timeouts_;}

private:
  std::deque<Step> steps_;
  std::vector<Request> requests_;
  std::vector<std::chrono::milliseconds> timeouts_;
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

void queue_failed_task(
  const std::shared_ptr<ScriptedTransport> & transport,
  const std::string & command_path,
  const std::string & task_id)
{
  transport->push("POST", command_path, 202, "{\"task_id\":\"" + task_id + "\"}");
  transport->push("GET", "/api/v1/tasks/" + task_id, 200,
    R"({"state":"failed","terminal":true,"error_code":"execution_failed","message":"test failure"})");
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

TEST(ElevatorArmClient, CancelDuringFailedReleaseDoesNotRetryForever)
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
    "elevator-test-789", 4U, "F2", "F1", [&] {
      return transport->requests().size() >= 5U;
    });

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

TEST(ElevatorArmClient, TransientQueryFailureKeepsTheAcceptedTask)
{
  auto transport = std::make_shared<ScriptedTransport>();
  queue_succeeded_task(transport, "/api/v1/arm/ready", "ready");
  transport->push("POST", "/api/v1/elevator/press-floor", 202,
    R"({"task_id":"press"})");
  transport->push("GET", "/api/v1/tasks/press", 503, R"({"error":"busy"})");
  transport->push("GET", "/api/v1/tasks/press", 200, R"({"state":"succeeded"})");
  queue_succeeded_task(transport, "/api/v1/arm/release", "release");
  ElevatorArmClient client(fast_options(), transport);

  const auto result = client.press_floor("query-retry", 1U, "F2", [] {return false;});

  EXPECT_TRUE(result.succeeded()) << result.detail;
  EXPECT_TRUE(transport->empty());
  EXPECT_EQ(std::count_if(transport->requests().begin(), transport->requests().end(),
    [](const ScriptedTransport::Request & r) {
      return r.method == "POST" && r.path == "/api/v1/elevator/press-floor";
    }), 1);
}

TEST(ElevatorArmClient, ReadyKeepsRetryingPastThreeFailuresInTheSameEffect)
{
  auto transport = std::make_shared<ScriptedTransport>();
  for (int i = 0; i < 4; ++i) {
    queue_failed_task(transport, "/api/v1/arm/ready", "ready" + std::to_string(i));
  }
  queue_succeeded_task(transport, "/api/v1/arm/ready", "ready-ok");
  queue_succeeded_task(transport, "/api/v1/elevator/press-floor", "press");
  queue_succeeded_task(transport, "/api/v1/arm/release", "release");
  ElevatorArmClient client(fast_options(), transport);

  const auto result = client.press_floor("retry", 9U, "F2", [] {return false;});

  EXPECT_TRUE(result.succeeded()) << result.detail;
  EXPECT_TRUE(transport->empty());
  ASSERT_EQ(transport->requests().size(), 14U);
  for (std::size_t i = 2U; i <= 8U; i += 2U) {
    EXPECT_NE(transport->requests()[i].body, transport->requests()[i - 2U].body);
    EXPECT_NE(transport->requests()[i].body.find("retry:9:ready"), std::string::npos);
  }
}

TEST(ElevatorArmClient, SuccessfulButtonIsNotRepeatedWhenReleaseRetries)
{
  auto transport = std::make_shared<ScriptedTransport>();
  queue_succeeded_task(transport, "/api/v1/arm/ready", "ready");
  queue_succeeded_task(transport, "/api/v1/elevator/press-floor", "press");
  queue_failed_task(transport, "/api/v1/arm/release", "release-failed");
  queue_succeeded_task(transport, "/api/v1/arm/release", "release-ok");
  ElevatorArmClient client(fast_options(), transport);

  const auto result = client.press_floor("release-retry", 2U, "F2", [] {return false;});

  EXPECT_TRUE(result.succeeded()) << result.detail;
  EXPECT_EQ(result.task_id, "press");
  EXPECT_TRUE(transport->empty());
  ASSERT_EQ(transport->requests().size(), 8U);
  EXPECT_NE(transport->requests()[4].body, transport->requests()[6].body);
  EXPECT_EQ(std::count_if(transport->requests().begin(), transport->requests().end(),
    [](const ScriptedTransport::Request & r) {
      return r.method == "POST" && r.path == "/api/v1/elevator/press-floor";
    }), 1);
}

TEST(ElevatorArmClient, FailedButtonReleasesThenPreparesAndRetriesOnlyThisEffect)
{
  for (const bool hall : {false, true}) {
    SCOPED_TRACE(hall);
    const std::string path = hall ? "/api/v1/elevator/call" : "/api/v1/elevator/press-floor";
    auto transport = std::make_shared<ScriptedTransport>();
    queue_succeeded_task(transport, "/api/v1/arm/ready", "ready1");
    queue_failed_task(transport, path, "press1");
    queue_succeeded_task(transport, "/api/v1/arm/release", "release1");
    queue_succeeded_task(transport, "/api/v1/arm/ready", "ready2");
    queue_succeeded_task(transport, path, "press2");
    queue_succeeded_task(transport, "/api/v1/arm/release", "release2");
    ElevatorArmClient client(fast_options(), transport);

    const auto result = hall ? client.press_hall_call("button-retry", 3U, "F2", "F1",
      [] {return false;}) : client.press_floor("button-retry", 3U, "F1", [] {return false;});

    EXPECT_TRUE(result.succeeded()) << result.detail;
    EXPECT_EQ(result.task_id, "press2");
    EXPECT_TRUE(transport->empty());
    ASSERT_EQ(transport->requests().size(), 12U);
    for (std::size_t i : {0U, 2U, 4U}) {
      EXPECT_NE(transport->requests()[i].body, transport->requests()[i + 6U].body);
    }
  }
}

TEST(ElevatorArmClient, LateSuccessfulQueryDoesNotExtendTheTaskBudget)
{
  auto transport = std::make_shared<ScriptedTransport>();
  transport->push("POST", "/api/v1/arm/ready", 202, R"({"task_id":"late"})");
  transport->push("GET", "/api/v1/tasks/late", 200, R"({"state":"succeeded"})", [] {
    std::this_thread::sleep_for(std::chrono::milliseconds(60));
  });
  queue_succeeded_task(transport, "/api/v1/arm/release", "release");
  auto options = fast_options();
  options.task_timeout = std::chrono::milliseconds(30);
  ElevatorArmClient client(options, transport);

  const auto result = client.press_floor("budget", 1U, "F2", [] {return false;});

  EXPECT_EQ(result.code, "ELEVATOR_ARM_TASK_TIMEOUT");
  EXPECT_TRUE(transport->empty());
  EXPECT_LE(transport->timeouts()[1], options.task_timeout);
  EXPECT_EQ(transport->requests().size(), 4U);
}

TEST(ElevatorArmClient, AlreadyCancelledSubmitsNothing)
{
  auto transport = std::make_shared<ScriptedTransport>();
  ElevatorArmClient client(fast_options(), transport);
  EXPECT_EQ(client.press_floor("cancel", 1U, "F2", [] {return true;}).code,
    "ELEVATOR_ARM_TASK_CANCELLED");
  EXPECT_TRUE(transport->requests().empty());
}

TEST(ElevatorArmClient, CancelAtReadyCompletionReleasesWithoutSubmittingEitherButton)
{
  for (const bool hall : {false, true}) {
    SCOPED_TRACE(hall);
    bool cancel = false;
    auto transport = std::make_shared<ScriptedTransport>();
    transport->push("POST", "/api/v1/arm/ready", 202, R"({"task_id":"ready"})");
    transport->push("GET", "/api/v1/tasks/ready", 200, R"({"state":"succeeded"})",
      [&] {cancel = true;});
    queue_succeeded_task(transport, "/api/v1/arm/release", "release");
    ElevatorArmClient client(fast_options(), transport);
    const auto result = hall ? client.press_hall_call("cancel", 1U, "F1", "F2",
      [&] {return cancel;}) : client.press_floor("cancel", 1U, "F2", [&] {return cancel;});
    EXPECT_EQ(result.code, "ELEVATOR_ARM_TASK_CANCELLED");
    EXPECT_EQ(transport->requests().size(), 4U);
    EXPECT_TRUE(transport->empty());
  }
}

TEST(ElevatorArmClient, CancelBetweenButtonAndReleaseStillSubmitsOneCleanup)
{
  auto transport = std::make_shared<ScriptedTransport>();
  queue_succeeded_task(transport, "/api/v1/arm/ready", "ready");
  queue_succeeded_task(transport, "/api/v1/elevator/press-floor", "press");
  queue_succeeded_task(transport, "/api/v1/arm/release", "release");
  ElevatorArmClient client(fast_options(), transport);
  bool cancel = false;
  int after_press_checks = 0;
  const auto result = client.press_floor("cancel", 1U, "F2", [&] {
    // Return false for the post-GET check and release decision; cancellation
    // arrives at the next observation, before a cancellable release could POST.
    if (transport->requests().size() == 4U && ++after_press_checks == 2) {
      cancel = true;
      return false;
    }
    return cancel;
  });
  EXPECT_EQ(transport->requests().size(), 6U);
  EXPECT_TRUE(transport->empty());
  EXPECT_TRUE(cancel);
  EXPECT_EQ(result.code, "ELEVATOR_ARM_TASK_CANCELLED");
}

TEST(ElevatorArmClient, CancellationInterruptsRetryDelayWithoutAnotherReady)
{
  auto transport = std::make_shared<ScriptedTransport>();
  std::atomic<bool> cancel{false};
  std::thread canceller;
  transport->push("POST", "/api/v1/arm/ready", 202, R"({"task_id":"ready"})");
  transport->push("GET", "/api/v1/tasks/ready", 200, R"({"state":"failed"})", [&] {
    canceller = std::thread([&] {
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
      cancel.store(true);
    });
  });
  queue_succeeded_task(transport, "/api/v1/arm/release", "release");
  ElevatorArmClient client(fast_options(), transport);
  const auto start = std::chrono::steady_clock::now();
  const auto result = client.press_floor("cancel", 1U, "F2", [&] {return cancel.load();});
  if (canceller.joinable()) {canceller.join();}
  EXPECT_LT(std::chrono::steady_clock::now() - start, std::chrono::milliseconds(500));
  EXPECT_EQ(result.code, "ELEVATOR_ARM_TASK_CANCELLED");
  EXPECT_EQ(transport->requests().size(), 4U);
  EXPECT_TRUE(transport->empty());
}

TEST(ElevatorArmClient, CancelWhileReleaseIsPendingDoesNotStartAnotherRelease)
{
  bool cancel = false;
  auto transport = std::make_shared<ScriptedTransport>();
  queue_succeeded_task(transport, "/api/v1/arm/ready", "ready");
  queue_succeeded_task(transport, "/api/v1/elevator/press-floor", "press");
  transport->push("POST", "/api/v1/arm/release", 202, R"({"task_id":"release"})");
  transport->push("GET", "/api/v1/tasks/release", 200, R"({"state":"running"})",
    [&] {cancel = true;});
  transport->push("GET", "/api/v1/tasks/release", 200,
    R"({"state":"failed","message":"release failed"})");
  ElevatorArmClient client(fast_options(), transport);
  const auto result = client.press_floor("cancel", 1U, "F2", [&] {return cancel;});
  EXPECT_FALSE(result.succeeded());
  EXPECT_TRUE(transport->empty());
  EXPECT_EQ(transport->requests().size(), 7U);
}

TEST(ElevatorArmClient, AmbiguousSubmissionDoesNotResubmitTheButton)
{
  const std::vector<ElevatorArmHttpResponse> responses{
    {599, {}, "receive failed: connection reset"},
    {202, R"({"ok":true})", {}},
    {500, R"({"error":"unknown acceptance"})", {}}};
  for (const auto & response : responses) {
    SCOPED_TRACE(response.status);
    auto transport = std::make_shared<ScriptedTransport>();
    queue_succeeded_task(transport, "/api/v1/arm/ready", "ready");
    transport->push("POST", "/api/v1/elevator/press-floor", response.status,
      response.body, {}, response.transport_error);
    queue_succeeded_task(transport, "/api/v1/arm/release", "release");
    ElevatorArmClient client(fast_options(), transport);
    const auto result = client.press_floor("unknown", 1U, "F2", [] {return false;});
    EXPECT_FALSE(result.succeeded());
    EXPECT_EQ(transport->requests().size(), 5U);
    EXPECT_TRUE(transport->empty());
  }
}

TEST(ElevatorArmClient, QueryProtocolAndPermanentErrorsDoNotStartANewAction)
{
  const std::vector<ElevatorArmHttpResponse> responses{
    {401, R"({"error":"unauthorized"})", {}},
    {404, R"({"error":"task missing"})", {}},
    {200, "[not a task object]", {}},
    {599, {}, "malformed arm HTTP response"},
    {200, R"({"state":"cancelled","terminal":true})", {}}};
  for (const auto & response : responses) {
    SCOPED_TRACE(response.body);
    auto transport = std::make_shared<ScriptedTransport>();
    queue_succeeded_task(transport, "/api/v1/arm/ready", "ready");
    transport->push("POST", "/api/v1/elevator/press-floor", 202, R"({"task_id":"press"})");
    transport->push("GET", "/api/v1/tasks/press", response.status, response.body,
      {}, response.transport_error);
    queue_succeeded_task(transport, "/api/v1/arm/release", "release");
    ElevatorArmClient client(fast_options(), transport);
    EXPECT_FALSE(client.press_floor("bad-query", 1U, "F2", [] {return false;}).succeeded());
    EXPECT_EQ(transport->requests().size(), 6U);
    EXPECT_TRUE(transport->empty());
  }
}

TEST(ElevatorArmClient, NetworkQueryFailureAndServiceOverloadKeepTheSameTask)
{
  const std::vector<ElevatorArmHttpResponse> responses{
    {599, {}, "receive failed: connection reset"},
    {599, {}, "request deadline exceeded"},
    {408, {}, {}}, {429, {}, {}}, {500, {}, {}}, {502, {}, {}}, {504, {}, {}}};
  for (const auto & response : responses) {
    SCOPED_TRACE(response.status);
    auto transport = std::make_shared<ScriptedTransport>();
    queue_succeeded_task(transport, "/api/v1/arm/ready", "ready");
    transport->push("POST", "/api/v1/elevator/press-floor", 202, R"({"task_id":"press"})");
    transport->push("GET", "/api/v1/tasks/press", response.status, response.body,
      {}, response.transport_error);
    transport->push("GET", "/api/v1/tasks/press", 200, R"({"state":"succeeded"})");
    queue_succeeded_task(transport, "/api/v1/arm/release", "release");
    ElevatorArmClient client(fast_options(), transport);
    EXPECT_TRUE(client.press_floor("query", 1U, "F2", [] {return false;}).succeeded());
    EXPECT_EQ(transport->requests().size(), 7U);
    EXPECT_TRUE(transport->empty());
  }
}

TEST(ElevatorArmClient, PendingButtonTimeoutDoesNotResubmit)
{
  auto transport = std::make_shared<ScriptedTransport>();
  queue_succeeded_task(transport, "/api/v1/arm/ready", "ready");
  transport->push("POST", "/api/v1/elevator/press-floor", 202, R"({"task_id":"pending"})");
  transport->push("GET", "/api/v1/tasks/pending", 200, R"({"state":"running"})", [] {
    std::this_thread::sleep_for(std::chrono::milliseconds(60));
  });
  queue_succeeded_task(transport, "/api/v1/arm/release", "release");
  auto options = fast_options();
  options.task_timeout = std::chrono::milliseconds(30);
  ElevatorArmClient client(options, transport);
  const auto result = client.press_floor("pending", 1U, "F2", [] {return false;});
  EXPECT_EQ(result.code, "ELEVATOR_ARM_TASK_TIMEOUT");
  EXPECT_EQ(result.task_id, "pending");
  EXPECT_TRUE(transport->empty());
  EXPECT_EQ(transport->requests().size(), 6U);
}

TEST(ElevatorArmClient, CancellationDuringTransientQueryDoesNotRetryOrPressAgain)
{
  bool cancel = false;
  auto transport = std::make_shared<ScriptedTransport>();
  queue_succeeded_task(transport, "/api/v1/arm/ready", "ready");
  transport->push("POST", "/api/v1/elevator/press-floor", 202, R"({"task_id":"press"})");
  transport->push("GET", "/api/v1/tasks/press", 503, R"({"error":"busy"})",
    [&] {cancel = true;});
  queue_failed_task(transport, "/api/v1/arm/release", "release");
  ElevatorArmClient client(fast_options(), transport);
  const auto result = client.press_floor("cancel", 1U, "F2", [&] {return cancel;});
  EXPECT_FALSE(result.succeeded());
  EXPECT_EQ(result.code, "ELEVATOR_ARM_RELEASE_FAILED");
  EXPECT_NE(result.detail.find("ELEVATOR_ARM_TASK_CANCELLED"), std::string::npos);
  EXPECT_EQ(transport->requests().size(), 6U);
  EXPECT_TRUE(transport->empty());
}

TEST(ElevatorArmClient, RuntimeFaultDuringRepeatedReadyFailureReturnsOriginalFault)
{
  class FailingReadyTransport final : public ElevatorArmHttpTransport
  {
  public:
    int ready_posts{0}, button_posts{0}, release_posts{0};
    bool runtime_failed{false};
    ElevatorArmHttpResponse request(const std::string & method,
      const std::string & path, const std::string &, std::chrono::milliseconds) override
    {
      if (method == "POST") {
        if (path == "/api/v1/arm/ready") {
          ++ready_posts;
          return {202, R"({"task_id":"ready"})", {}};
        }
        if (path == "/api/v1/arm/release") {
          ++release_posts;
          return {202, R"({"task_id":"release"})", {}};
        }
        ++button_posts;
      }
      if (path == "/api/v1/tasks/ready") {
        runtime_failed = true;
        return {200, R"({"state":"failed","error_code":"execution_failed"})", {}};
      }
      return {200, R"({"state":"succeeded"})", {}};
    }
  };
  auto transport = std::make_shared<FailingReadyTransport>();
  ElevatorArmClient client(fast_options(), transport);
  const auto result = client.press_floor("runtime-fault", 1U, "F2",
    // Bounded escape for the pre-fix implementation; not the expected outcome.
    [&] {return transport->ready_posts >= 2;}, [&]() -> std::optional<ElevatorArmOutcome> {
      if (!transport->runtime_failed) {return std::nullopt;}
      return ElevatorArmOutcome{ElevatorArmOutcomeKind::kFailed,
        "ELEVATOR_RUNTIME_EXECUTOR_UNHEALTHY", "original executor failure", {}};
    });
  EXPECT_EQ(result.code, "ELEVATOR_RUNTIME_EXECUTOR_UNHEALTHY");
  EXPECT_NE(result.detail.find("original executor failure"), std::string::npos);
  EXPECT_FALSE(result.succeeded());
  EXPECT_EQ(transport->ready_posts, 1);
  EXPECT_EQ(transport->button_posts, 0);
  EXPECT_EQ(transport->release_posts, 1);
}

TEST(ElevatorArmClient, ExistingRuntimeFaultSubmitsNothingAndIsNotCancellation)
{
  auto transport = std::make_shared<ScriptedTransport>();
  ElevatorArmClient client(fast_options(), transport);
  const auto result = client.press_hall_call("fault-before-start", 1U, "F1", "F2",
    [] {return false;}, []() -> std::optional<ElevatorArmOutcome> {
      return ElevatorArmOutcome{ElevatorArmOutcomeKind::kFailed,
        "ELEVATOR_SAFETY_HOLD_LOST", "existing hold evidence lost", {}};
    });
  EXPECT_EQ(result.code, "ELEVATOR_SAFETY_HOLD_LOST");
  EXPECT_TRUE(transport->requests().empty());
}

TEST(ElevatorArmClient, RuntimeFaultDuringButtonQueryKeepsFaultWhenCleanupAlsoFails)
{
  for (const bool hall : {false, true}) {
    SCOPED_TRACE(hall);
    bool runtime_failed = false;
    auto transport = std::make_shared<ScriptedTransport>();
    const std::string path = hall ? "/api/v1/elevator/call" : "/api/v1/elevator/press-floor";
    queue_succeeded_task(transport, "/api/v1/arm/ready", "ready");
    transport->push("POST", path, 202, R"({"task_id":"press"})");
    transport->push("GET", "/api/v1/tasks/press", 200, R"({"state":"running"})",
      [&] {runtime_failed = true;});
    queue_failed_task(transport, "/api/v1/arm/release", "release");
    ElevatorArmClient client(fast_options(), transport);
    const ElevatorArmRuntimeFailureProbe probe = [&]() -> std::optional<ElevatorArmOutcome> {
        if (!runtime_failed) {return std::nullopt;}
        return ElevatorArmOutcome{ElevatorArmOutcomeKind::kFailed,
          "ELEVATOR_CORRECTION_PAUSE_LOST", "original pause failure", {}};
      };
    const auto result = hall ? client.press_hall_call("fault", 1U, "F1", "F2",
      [] {return false;}, probe) : client.press_floor("fault", 1U, "F2",
      [] {return false;}, probe);
    EXPECT_EQ(result.code, "ELEVATOR_CORRECTION_PAUSE_LOST");
    EXPECT_NE(result.detail.find("original pause failure"), std::string::npos);
    EXPECT_NE(result.detail.find("execution_failed"), std::string::npos);
    EXPECT_EQ(result.task_id, "press");
    EXPECT_EQ(transport->requests().size(), 6U);
    EXPECT_TRUE(transport->empty());
  }
}

TEST(ElevatorArmClient, RuntimeFaultInterruptsRetryDelayWithoutAnotherReady)
{
  std::atomic<bool> runtime_failed{false};
  std::thread fault_thread;
  auto transport = std::make_shared<ScriptedTransport>();
  transport->push("POST", "/api/v1/arm/ready", 202, R"({"task_id":"ready"})");
  transport->push("GET", "/api/v1/tasks/ready", 200, R"({"state":"failed"})", [&] {
    fault_thread = std::thread([&] {
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
      runtime_failed.store(true);
    });
  });
  queue_succeeded_task(transport, "/api/v1/arm/release", "release");
  ElevatorArmClient client(fast_options(), transport);
  const auto start = std::chrono::steady_clock::now();
  const auto result = client.press_floor("fault-backoff", 1U, "F2", [] {return false;},
    [&]() -> std::optional<ElevatorArmOutcome> {
      if (!runtime_failed.load()) {return std::nullopt;}
      return ElevatorArmOutcome{ElevatorArmOutcomeKind::kFailed,
        "ELEVATOR_RUNTIME_EXECUTOR_UNHEALTHY", "executor stopped during backoff", {}};
    });
  if (fault_thread.joinable()) {fault_thread.join();}
  EXPECT_LT(std::chrono::steady_clock::now() - start, std::chrono::milliseconds(500));
  EXPECT_EQ(result.code, "ELEVATOR_RUNTIME_EXECUTOR_UNHEALTHY");
  EXPECT_EQ(transport->requests().size(), 4U);
  EXPECT_TRUE(transport->empty());
}

TEST(ElevatorArmClient, RuntimeFaultDuringReleaseDoesNotRepeatReleaseOrButton)
{
  for (const bool release_succeeds : {false, true}) {
    SCOPED_TRACE(release_succeeds);
    bool runtime_failed = false;
    auto transport = std::make_shared<ScriptedTransport>();
    queue_succeeded_task(transport, "/api/v1/arm/ready", "ready");
    queue_succeeded_task(transport, "/api/v1/elevator/press-floor", "press");
    transport->push("POST", "/api/v1/arm/release", 202, R"({"task_id":"release"})");
    transport->push("GET", "/api/v1/tasks/release", 200, R"({"state":"running"})",
      [&] {runtime_failed = true;});
    transport->push("GET", "/api/v1/tasks/release", 200,
      release_succeeds ? R"({"state":"succeeded"})" : R"({"state":"failed"})");
    ElevatorArmClient client(fast_options(), transport);
    const auto result = client.press_floor("fault-release", 1U, "F2", [] {return false;},
      [&]() -> std::optional<ElevatorArmOutcome> {
        if (!runtime_failed) {return std::nullopt;}
        return ElevatorArmOutcome{ElevatorArmOutcomeKind::kFailed,
          "ELEVATOR_RUNTIME_EXECUTOR_UNHEALTHY", "executor failed during release", {}};
      });
    EXPECT_EQ(result.code, "ELEVATOR_RUNTIME_EXECUTOR_UNHEALTHY");
    EXPECT_EQ(transport->requests().size(), 7U);
    EXPECT_TRUE(transport->empty());
  }
}

TEST(ElevatorArmClient, ThrowingRuntimeProbeStillRunsOneBoundedCleanup)
{
  for (const bool standard_exception : {false, true}) {
    SCOPED_TRACE(standard_exception);
    bool throw_now = false;
    auto transport = std::make_shared<ScriptedTransport>();
    transport->push("POST", "/api/v1/arm/ready", 202, R"({"task_id":"ready"})");
    transport->push("GET", "/api/v1/tasks/ready", 200, R"({"state":"running"})",
      [&] {throw_now = true;});
    queue_failed_task(transport, "/api/v1/arm/release", "release");
    ElevatorArmClient client(fast_options(), transport);
    const auto result = client.press_floor("probe-throw", 1U, "F2", [] {return false;},
      [&]() -> std::optional<ElevatorArmOutcome> {
        if (!throw_now) {return std::nullopt;}
        if (standard_exception) {throw std::runtime_error("original probe failure");}
        throw 7;
      });
    EXPECT_EQ(result.code, "ELEVATOR_RUNTIME_HEALTH_EXCEPTION");
    if (standard_exception) {
      EXPECT_NE(result.detail.find("original probe failure"), std::string::npos);
    }
    EXPECT_EQ(transport->requests().size(), 4U);
    EXPECT_TRUE(transport->empty());
  }
}

TEST(ElevatorArmClient, RuntimeFaultCleanupKeepsExistingTaskDeadline)
{
  bool runtime_failed = false;
  auto transport = std::make_shared<ScriptedTransport>();
  transport->push("POST", "/api/v1/arm/ready", 202, R"({"task_id":"ready"})");
  transport->push("GET", "/api/v1/tasks/ready", 200, R"({"state":"failed"})",
    [&] {runtime_failed = true;});
  transport->push("POST", "/api/v1/arm/release", 202, R"({"task_id":"release"})");
  transport->push("GET", "/api/v1/tasks/release", 200, R"({"state":"running"})", [] {
    std::this_thread::sleep_for(std::chrono::milliseconds(60));
  });
  auto options = fast_options();
  options.task_timeout = std::chrono::milliseconds(30);
  ElevatorArmClient client(options, transport);
  const auto result = client.press_floor("fault-timeout", 1U, "F2", [] {return false;},
    [&]() -> std::optional<ElevatorArmOutcome> {
      if (!runtime_failed) {return std::nullopt;}
      return ElevatorArmOutcome{ElevatorArmOutcomeKind::kFailed,
        "ELEVATOR_RUNTIME_EXECUTOR_UNHEALTHY", "original fault", {}};
    });
  EXPECT_EQ(result.code, "ELEVATOR_RUNTIME_EXECUTOR_UNHEALTHY");
  EXPECT_NE(result.detail.find("ELEVATOR_ARM_TASK_TIMEOUT"), std::string::npos);
  EXPECT_EQ(transport->requests().size(), 4U);
  EXPECT_TRUE(transport->empty());
}

TEST(ElevatorArmClient, SimultaneousCancelAndRuntimeFaultKeepsFaultAndBoundedCleanup)
{
  bool interrupted = false;
  auto transport = std::make_shared<ScriptedTransport>();
  queue_succeeded_task(transport, "/api/v1/arm/ready", "ready");
  transport->push("POST", "/api/v1/elevator/press-floor", 202, R"({"task_id":"press"})");
  transport->push("GET", "/api/v1/tasks/press", 200, R"({"state":"running"})",
    [&] {interrupted = true;});
  queue_failed_task(transport, "/api/v1/arm/release", "release");
  ElevatorArmClient client(fast_options(), transport);
  const auto result = client.press_floor("cancel-and-fault", 1U, "F2",
    [&] {return interrupted;}, [&]() -> std::optional<ElevatorArmOutcome> {
      if (!interrupted) {return std::nullopt;}
      return ElevatorArmOutcome{ElevatorArmOutcomeKind::kFailed,
        "ELEVATOR_RUNTIME_EXECUTOR_UNHEALTHY", "original runtime fault", {}};
    });
  EXPECT_EQ(result.code, "ELEVATOR_RUNTIME_EXECUTOR_UNHEALTHY");
  EXPECT_NE(result.detail.find("original runtime fault"), std::string::npos);
  EXPECT_NE(result.detail.find("execution_failed"), std::string::npos);
  EXPECT_EQ(transport->requests().size(), 6U);
  EXPECT_TRUE(transport->empty());
}

}  // namespace
}  // namespace robot_api_server
