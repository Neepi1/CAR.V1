#include <atomic>
#include <chrono>
#include <thread>

#include <gtest/gtest.h>

#include "robot_api_server/features/elevator/execution/elevator_lease_keepalive.hpp"
#include "robot_safety/motion_interlock_arbiter.hpp"

namespace
{

using namespace std::chrono_literals;

robot_safety::ExecutionCommand execution_command()
{
  robot_safety::ExecutionCommand command;
  command.operation = robot_safety::ExecutionOperation::kSet;
  command.owner = "robot_elevator_manager";
  command.mission_id = "elevator_transaction";
  command.transaction_id = "transaction";
  command.lease_id = "execution_transaction";
  command.lease_duration_sec = 0.09;
  return command;
}

TEST(ElevatorLeaseKeepalive, KeepsExactLeaseAliveAcrossConsecutiveEffectGap)
{
  robot_safety::MotionInterlockArbiter arbiter(0.01, 1.0);
  const auto started_at = std::chrono::steady_clock::now();
  const auto now_sec = [&started_at]() {
      return std::chrono::duration<double>(
        std::chrono::steady_clock::now() - started_at).count();
    };
  const auto command = execution_command();
  ASSERT_TRUE(arbiter.apply_execution(command, now_sec()).accepted);

  std::atomic<int> renewals{0};
  robot_api_server::ElevatorLeaseKeepalive keepalive(
    20ms,
    [&]() {
      const auto decision = arbiter.apply_execution(command, now_sec());
      if (!decision.accepted) {
        return robot_api_server::ElevatorLeaseKeepaliveStatus{
          false, "ELEVATOR_EXECUTION_LEASE_REJECTED", decision.message};
      }
      ++renewals;
      return robot_api_server::ElevatorLeaseKeepaliveStatus{
        true, {}, decision.message};
    });
  ASSERT_TRUE(keepalive.start().success);

  // Models terminal hold acquisition + dual-odom settle + admission of the
  // immediately following goal. The gap is deliberately longer than the TTL.
  std::this_thread::sleep_for(260ms);
  (void)arbiter.expire(now_sec());

  const auto snapshot = arbiter.snapshot(now_sec());
  EXPECT_TRUE(snapshot.execution_session_engaged);
  EXPECT_TRUE(snapshot.execution_lease_active);
  EXPECT_GE(renewals.load(), 4);
  EXPECT_TRUE(arbiter.apply_execution(command, now_sec()).accepted);
  keepalive.stop();
}

TEST(ElevatorLeaseKeepaliveGroup, SlowModeRenewalCannotStarveExecutionHeartbeat)
{
  robot_safety::MotionInterlockArbiter arbiter(0.01, 1.0);
  const auto started_at = std::chrono::steady_clock::now();
  const auto now_sec = [&started_at]() {
      return std::chrono::duration<double>(
        std::chrono::steady_clock::now() - started_at).count();
    };
  auto command = execution_command();
  command.lease_duration_sec = 0.15;
  ASSERT_TRUE(arbiter.apply_execution(command, now_sec()).accepted);

  std::atomic<int> execution_renewals{0};
  std::atomic<bool> slow_mode_renewal_started{false};
  robot_api_server::ElevatorLeaseKeepaliveGroup keepalives(
    20ms,
    [&]() {
      const auto decision = arbiter.apply_execution(command, now_sec());
      if (!decision.accepted) {
        return robot_api_server::ElevatorLeaseKeepaliveStatus{
          false, "ELEVATOR_EXECUTION_LEASE_REJECTED", decision.message};
      }
      ++execution_renewals;
      return robot_api_server::ElevatorLeaseKeepaliveStatus{
        true, {}, decision.message};
    },
    10ms,
    [&]() {
      slow_mode_renewal_started.store(true);
      std::this_thread::sleep_for(350ms);
      return robot_api_server::ElevatorLeaseKeepaliveStatus{
        true, {}, "mode lease renewed"};
    });
  ASSERT_TRUE(keepalives.start().success);

  const auto mode_deadline = std::chrono::steady_clock::now() + 250ms;
  while (
    !slow_mode_renewal_started.load() &&
    std::chrono::steady_clock::now() < mode_deadline)
  {
    std::this_thread::sleep_for(2ms);
  }
  ASSERT_TRUE(slow_mode_renewal_started.load());

  // The mode service remains blocked well beyond the execution TTL. The
  // execution heartbeat must continue on its own worker instead of waiting
  // behind that unrelated service call.
  std::this_thread::sleep_for(220ms);
  (void)arbiter.expire(now_sec());
  const auto snapshot = arbiter.snapshot(now_sec());
  EXPECT_TRUE(snapshot.execution_session_engaged);
  EXPECT_TRUE(snapshot.execution_lease_active);
  EXPECT_GE(execution_renewals.load(), 5);

  keepalives.stop();
  const auto renewals_after_stop = execution_renewals.load();
  std::this_thread::sleep_for(50ms);
  EXPECT_EQ(execution_renewals.load(), renewals_after_stop);
}

TEST(ElevatorLeaseKeepaliveGroup, FirstChannelFailureStopsBothWorkersOnce)
{
  std::atomic<int> execution_renewals{0};
  std::atomic<int> mode_renewals{0};
  std::atomic<int> failure_callbacks{0};
  robot_api_server::ElevatorLeaseKeepaliveGroup keepalives(
    10ms,
    [&]() {
      ++execution_renewals;
      return robot_api_server::ElevatorLeaseKeepaliveStatus{
        true, {}, "execution renewed"};
    },
    20ms,
    [&]() {
      ++mode_renewals;
      return robot_api_server::ElevatorLeaseKeepaliveStatus{
        false, "ELEVATOR_OPERATING_MODE_REJECTED", "mode lease rejected"};
    },
    [&](const robot_api_server::ElevatorLeaseKeepaliveStatus & status) {
      EXPECT_EQ(status.code, "ELEVATOR_OPERATING_MODE_REJECTED");
      ++failure_callbacks;
    });
  ASSERT_TRUE(keepalives.start().success);

  const auto failure_deadline = std::chrono::steady_clock::now() + 250ms;
  while (!keepalives.last_failure() &&
    std::chrono::steady_clock::now() < failure_deadline)
  {
    std::this_thread::sleep_for(2ms);
  }
  ASSERT_TRUE(keepalives.last_failure().has_value());
  EXPECT_EQ(
    keepalives.last_failure()->code, "ELEVATOR_OPERATING_MODE_REJECTED");
  EXPECT_FALSE(keepalives.running());
  EXPECT_EQ(failure_callbacks.load(), 1);
  EXPECT_EQ(mode_renewals.load(), 1);

  keepalives.stop();
  const auto execution_after_failure = execution_renewals.load();
  const auto mode_after_failure = mode_renewals.load();
  std::this_thread::sleep_for(50ms);
  EXPECT_EQ(execution_renewals.load(), execution_after_failure);
  EXPECT_EQ(mode_renewals.load(), mode_after_failure);
  EXPECT_FALSE(keepalives.renew_now().success);
  EXPECT_EQ(failure_callbacks.load(), 1);
}

TEST(ElevatorLeaseKeepalive, LatchesFirstFailureAndStopsRenewing)
{
  std::atomic<int> attempts{0};
  std::atomic<int> failure_callbacks{0};
  robot_api_server::ElevatorLeaseKeepalive keepalive(
    10ms,
    [&]() {
      ++attempts;
      return robot_api_server::ElevatorLeaseKeepaliveStatus{
        false, "ELEVATOR_EXECUTION_LEASE_REJECTED", "lease retired"};
    },
    [&](const robot_api_server::ElevatorLeaseKeepaliveStatus & status) {
      EXPECT_EQ(status.code, "ELEVATOR_EXECUTION_LEASE_REJECTED");
      ++failure_callbacks;
    });

  ASSERT_TRUE(keepalive.start().success);
  const auto deadline = std::chrono::steady_clock::now() + 250ms;
  while (!keepalive.last_failure() && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(2ms);
  }
  ASSERT_TRUE(keepalive.last_failure().has_value());
  EXPECT_EQ(keepalive.last_failure()->code, "ELEVATOR_EXECUTION_LEASE_REJECTED");
  EXPECT_FALSE(keepalive.running());
  EXPECT_EQ(failure_callbacks.load(), 1);
  const auto attempts_after_failure = attempts.load();
  std::this_thread::sleep_for(40ms);
  EXPECT_EQ(attempts.load(), attempts_after_failure);
  EXPECT_FALSE(keepalive.renew_now().success);
  EXPECT_EQ(failure_callbacks.load(), 1);
  keepalive.stop();
}

TEST(ElevatorLeaseKeepalive, StopPreventsAnyLaterRenewal)
{
  std::atomic<int> renewals{0};
  robot_api_server::ElevatorLeaseKeepalive keepalive(
    10ms,
    [&]() {
      ++renewals;
      return robot_api_server::ElevatorLeaseKeepaliveStatus{
        true, {}, "renewed"};
    });
  ASSERT_TRUE(keepalive.start().success);
  const auto deadline = std::chrono::steady_clock::now() + 250ms;
  while (renewals.load() < 2 && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(2ms);
  }
  ASSERT_GE(renewals.load(), 2);
  keepalive.stop();
  const auto stopped_at = renewals.load();
  std::this_thread::sleep_for(40ms);
  EXPECT_EQ(renewals.load(), stopped_at);
  EXPECT_FALSE(keepalive.running());
}

}  // namespace
