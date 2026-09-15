#include <gtest/gtest.h>
#include "robot_api_server/features/navigation/runtime/navigation_recovery_wait.hpp"

using namespace robot_api_server::features::navigation;
namespace {
std::string status(std::int64_t goal, std::int64_t stamp, std::string phase = "waiting")
{
  return "{\"version\":1,\"goal_stamp_ns\":" + std::to_string(goal) +
    ",\"stamp_ns\":" + std::to_string(stamp) + ",\"phase\":\"" + phase + "\"}";
}
TEST(NavigationRecoveryWait, RequiresMatchingFreshGoalAndMonotonicPublisherStamp)
{
  NavigationRecoveryWait wait;
  wait.bind(100);
  EXPECT_FALSE(wait.receive(status(99, 200), 200, 1.0));
  EXPECT_TRUE(wait.receive(status(100, 200), 200, 1.0));
  EXPECT_EQ(wait.phase(200, 1.0), "waiting");
  EXPECT_FALSE(wait.receive(status(100, 199, "tracking"), 200, 1.0));
  EXPECT_FALSE(wait.receive(status(100, 201), 200, 1.0));
  EXPECT_FALSE(wait.receive(status(100, 300), 2000000000LL, 1.0));
  EXPECT_EQ(wait.phase(2000000000LL, 1.0), "");
  EXPECT_EQ(wait.phase(200, 2.6), "");
  EXPECT_EQ(wait.phase(199, 1.1), "");
}
TEST(NavigationRecoveryWait, ClearRebindAndInvalidMessagesCannotLendWaitToNextTask)
{
  NavigationRecoveryWait wait;
  wait.bind(100);
  ASSERT_TRUE(wait.receive(status(100, 200), 200, 1.0));
  wait.bind(201);
  EXPECT_EQ(wait.phase(202, 1.1), "");
  EXPECT_FALSE(wait.receive(status(100, 202), 202, 1.1));
  EXPECT_FALSE(wait.receive("invalid", 202, 1.1));
  EXPECT_FALSE(wait.receive(std::string(513, 'a'), 202, 1.1));
  EXPECT_FALSE(wait.receive(status(201, 202, "invented"), 202, 1.1));
  wait.bind(0);
  EXPECT_FALSE(wait.receive(status(0, 202), 202, 1.1));
}
TEST(NavigationRecoveryWait, TerminalAndFaultPhasesNeverSuspendBudget)
{
  for (const auto & phase : {"tracking", "checking_failure", "idle", "failed", "succeeded", ""}) {
    EXPECT_FALSE(NavigationRecoveryWait::pauses_execution(phase));
  }
  EXPECT_TRUE(NavigationRecoveryWait::pauses_execution("waiting"));
  EXPECT_TRUE(NavigationRecoveryWait::pauses_execution("recovering"));
}
TEST(NavigationExecutionBudget, ProlongedObservedWaitSurvivesOriginal600Seconds)
{
  const auto start = NavigationExecutionBudget::Clock::time_point{};
  NavigationExecutionBudget budget(600.0, start);
  for (int i = 1; i <= 12000; ++i) {
    ASSERT_FALSE(budget.expired(start + std::chrono::milliseconds(i * 100), true));
  }
  EXPECT_FALSE(budget.expired(start + std::chrono::seconds(1700), false));
  EXPECT_TRUE(budget.expired(start + std::chrono::seconds(1801), false));
}
TEST(NavigationExecutionBudget, AbsentStatusAndLongUnknownGapsStillExpire)
{
  const auto start = NavigationExecutionBudget::Clock::time_point{};
  NavigationExecutionBudget normal(600.0, start);
  EXPECT_TRUE(normal.expired(start + std::chrono::seconds(600), false));
  NavigationExecutionBudget stalled(600.0, start);
  EXPECT_FALSE(stalled.expired(start + std::chrono::milliseconds(100), true));
  EXPECT_TRUE(stalled.expired(start + std::chrono::seconds(700), true));
}
}  // namespace
