#include <gtest/gtest.h>

#include <optional>
#include <string>
#include <vector>

#include "robot_api_server/features/elevator/execution/elevator_navigation_retry.hpp"
#include "robot_nav_config/elevator_scoped_execution_session.hpp"

namespace robot_api_server
{
namespace
{
using Result = robot_elevator_manager::ElevatorRuntimeResult;

TEST(ElevatorNavigationRetry, ConfirmedAbortRetriesSameEffectUntilSuccess)
{
  const std::string frozen_goal = "source:F1:hall:1.2:3.4:0.5:hall.xml";
  std::vector<std::string> goals;
  std::vector<std::string> failures;
  const auto result = run_elevator_navigation_retries(
    [&](const std::uint64_t attempt) {
      goals.push_back(frozen_goal);
      return ElevatorNavigationAttemptResult{
        attempt < 4U ? Result{false, "ELEVATOR_NAV2_GOAL_FAILED", "original ABORTED"} :
        Result{true, "OK", "original success"},
        attempt < 4U ? ElevatorNavigationAttemptEnd::kAborted :
        ElevatorNavigationAttemptEnd::kOtherTerminal};
    },
    [&](std::uint64_t, const Result & failure) -> std::optional<Result> {
      failures.push_back(failure.code + ":" + failure.detail);
      return std::nullopt;
    });
  EXPECT_TRUE(result.success);
  EXPECT_EQ(result.detail, "original success");
  ASSERT_EQ(goals.size(), 5U);
  for (const auto & goal : goals) {EXPECT_EQ(goal, frozen_goal);}
  ASSERT_EQ(failures.size(), 4U);
  for (const auto & failure : failures) {
    EXPECT_EQ(failure, "ELEVATOR_NAV2_GOAL_FAILED:original ABORTED");
  }
}

TEST(ElevatorNavigationRetry, RejectionAndConfirmedTimeoutTerminalAllowAnotherAttempt)
{
  for (const auto end : {ElevatorNavigationAttemptEnd::kRejected,
      ElevatorNavigationAttemptEnd::kTimeoutTerminal})
  {
    int attempts = 0;
    const auto result = run_elevator_navigation_retries(
      [&](std::uint64_t) {
        ++attempts;
        return ElevatorNavigationAttemptResult{
          attempts == 1 ? Result{false, "original", "original failure"} :
          Result{true, "OK", "success"}, end};
      }, [](std::uint64_t, const Result &) -> std::optional<Result> {return std::nullopt;});
    EXPECT_TRUE(result.success);
    EXPECT_EQ(attempts, 2);
  }
}

TEST(ElevatorNavigationRetry, RetryResetsControllerEvenWhenPreviousEndWasLost)
{
  robot_nav_config::ElevatorScopedExecutionSession controller;
  const auto first = elevator_navigation_attempt_session("transaction:7", 0U);
  const auto next = elevator_navigation_attempt_session("transaction:7", 1U);
  EXPECT_EQ(first, "transaction:7");
  ASSERT_TRUE(controller.begin(first).reset_required);
  EXPECT_TRUE(controller.begin(next).reset_required);
  EXPECT_FALSE(controller.end(first).accepted);
  EXPECT_EQ(controller.active_session_id(), next);
}

TEST(ElevatorNavigationRetry, UnknownAndOtherTerminalNeverResubmitDespiteFailureCode)
{
  for (const auto end : {ElevatorNavigationAttemptEnd::kUnproven,
      ElevatorNavigationAttemptEnd::kOtherTerminal})
  {
    int attempts = 0;
    int pauses = 0;
    const auto result = run_elevator_navigation_retries(
      [&](std::uint64_t) {
        ++attempts;
        return ElevatorNavigationAttemptResult{
          Result{false, "ELEVATOR_NAV2_GOAL_FAILED", "original UNKNOWN or CANCELED"}, end};
      }, [&](std::uint64_t, const Result &) -> std::optional<Result> {
        ++pauses;
        return Result{false, "unexpected retry", ""};
      });
    EXPECT_FALSE(result.success);
    EXPECT_EQ(result.detail, "original UNKNOWN or CANCELED");
    EXPECT_EQ(attempts, 1);
    EXPECT_EQ(pauses, 0);
  }
}

TEST(ElevatorNavigationRetry, CancelOrExistingRuntimeFaultInterruptsWithoutAnotherAttempt)
{
  for (const auto * code : {"ELEVATOR_CANCEL_FENCE_ACTIVE", "ELEVATOR_RUNTIME_EXECUTOR_UNHEALTHY"}) {
    int attempts = 0;
    const auto result = run_elevator_navigation_retries(
      [&](std::uint64_t) {
        ++attempts;
        return ElevatorNavigationAttemptResult{
          Result{false, "ELEVATOR_NAV2_GOAL_FAILED", "original abort"},
          ElevatorNavigationAttemptEnd::kAborted};
      }, [&](std::uint64_t, const Result &) -> std::optional<Result> {
        return Result{false, code, "original interruption"};
      });
    EXPECT_EQ(attempts, 1);
    EXPECT_EQ(result.code, code);
    EXPECT_EQ(result.detail, "original interruption");
  }
}

TEST(ElevatorNavigationRetry, SuccessDoesNotWaitOrChangeResultEvidence)
{
  int pauses = 0;
  const auto result = run_elevator_navigation_retries(
    [](std::uint64_t) {
      return ElevatorNavigationAttemptResult{Result{true, "OK", "settled", true, true},
        ElevatorNavigationAttemptEnd::kOtherTerminal};
    }, [&](std::uint64_t, const Result &) -> std::optional<Result> {
      ++pauses;
      return std::nullopt;
    });
  EXPECT_TRUE(result.success);
  EXPECT_TRUE(result.safety_hold_proven);
  EXPECT_TRUE(result.dual_odom_stop_proven);
  EXPECT_EQ(result.detail, "settled");
  EXPECT_EQ(pauses, 0);
}
}  // namespace
}  // namespace robot_api_server
