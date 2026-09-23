#include <gtest/gtest.h>

#include "robot_api_server/features/elevator/execution/elevator_cached_health.hpp"

namespace robot_api_server
{
namespace
{

TEST(ElevatorCachedHealth, ArmWaitReportsUnavailableExecutorWithoutRemoteTerminal)
{
  ElevatorCachedHealth evidence;
  evidence.executor_unavailable = true;
  const auto failure = check_elevator_cached_health(evidence, true);
  ASSERT_TRUE(failure);
  EXPECT_EQ(failure->code, "ELEVATOR_RUNTIME_EXECUTOR_UNHEALTHY");
  EXPECT_EQ(failure->detail, "ROS runtime executor is unavailable");
}

TEST(ElevatorCachedHealth, ArmWaitDoesNotTurnModeRenewalUnknownIntoFailureButDetectsLostHold)
{
  ElevatorCachedHealth evidence;
  evidence.expect_hold = true;
  evidence.hold_current = true;
  evidence.mode_ownership_unknown = true;
  evidence.expect_mode = true;
  evidence.mode_current = false;
  EXPECT_FALSE(check_elevator_cached_health(evidence, true));

  evidence.hold_current = false;
  const auto failure = check_elevator_cached_health(evidence, true);
  ASSERT_TRUE(failure);
  EXPECT_EQ(failure->code, "ELEVATOR_SAFETY_HOLD_LOST");
  EXPECT_EQ(failure->detail, "owner-scoped safety hold is missing or stale");
}

TEST(ElevatorCachedHealth, MonitorKeepsOriginalModeAndCancellationFailures)
{
  ElevatorCachedHealth evidence;
  EXPECT_FALSE(check_elevator_cached_health(evidence, false));
  evidence.mode_ownership_unknown = true;
  auto failure = check_elevator_cached_health(evidence, false);
  ASSERT_TRUE(failure);
  EXPECT_EQ(failure->code, "ELEVATOR_RUNTIME_RESOURCE_STATE_UNPROVEN");

  evidence.mode_ownership_unknown = false;
  evidence.expect_mode = true;
  failure = check_elevator_cached_health(evidence, false);
  ASSERT_TRUE(failure);
  EXPECT_EQ(failure->code, "ELEVATOR_OPERATING_MODE_LEASE_LOST");
  evidence.expect_hold = true;
  evidence.hold_current = true;
  EXPECT_FALSE(check_elevator_cached_health(evidence, false));

  evidence.cancel_requested = true;
  failure = check_elevator_cached_health(evidence, false);
  ASSERT_TRUE(failure);
  EXPECT_EQ(failure->code, "ELEVATOR_RUNTIME_CANCEL_REQUESTED");
  // The arm client has a distinct cancellation probe; don't mislabel it a fault.
  EXPECT_FALSE(check_elevator_cached_health(evidence, true));
}

TEST(ElevatorCachedHealth, BothPathsPreserveCorrectionPauseFailuresAndPriority)
{
  for (const bool arm_wait : {false, true}) {
    ElevatorCachedHealth evidence;
    evidence.expect_hold = true;
    evidence.hold_current = true;
    evidence.expect_correction_pause = true;
    evidence.correction_pause_current = true;
    EXPECT_FALSE(check_elevator_cached_health(evidence, arm_wait));
    evidence.correction_pause_current = false;
    auto failure = check_elevator_cached_health(evidence, arm_wait);
    ASSERT_TRUE(failure);
    EXPECT_EQ(failure->code, "ELEVATOR_CORRECTION_PAUSE_LOST");
    evidence.correction_ownership_unknown = true;
    evidence.hold_current = false;
    failure = check_elevator_cached_health(evidence, arm_wait);
    ASSERT_TRUE(failure);
    EXPECT_EQ(failure->code, "ELEVATOR_RUNTIME_RESOURCE_STATE_UNPROVEN");
    evidence.executor_unavailable = true;
    failure = check_elevator_cached_health(evidence, arm_wait);
    ASSERT_TRUE(failure);
    EXPECT_EQ(failure->code, "ELEVATOR_RUNTIME_EXECUTOR_UNHEALTHY");
  }
}

TEST(ElevatorCachedHealth, MonitorPreservesOriginalFailurePriorityForEveryBooleanCombination)
{
  // Lock the pre-extraction poll_health decision order, not ROS transport.
  for (unsigned mask = 0; mask < 1024; ++mask) {
    const auto bit = [mask](unsigned index) {return (mask & (1U << index)) != 0;};
    const ElevatorCachedHealth evidence{
      bit(0), bit(1), bit(2), bit(3), bit(4),
      bit(5), bit(6), bit(7), bit(8), bit(9)};
    const std::string expected = evidence.executor_unavailable ?
      "ELEVATOR_RUNTIME_EXECUTOR_UNHEALTHY" : evidence.cancel_requested ?
      "ELEVATOR_RUNTIME_CANCEL_REQUESTED" :
      (evidence.mode_ownership_unknown || evidence.correction_ownership_unknown) ?
      "ELEVATOR_RUNTIME_RESOURCE_STATE_UNPROVEN" :
      (evidence.expect_hold && !evidence.hold_current) ?
      "ELEVATOR_SAFETY_HOLD_LOST" :
      (evidence.expect_mode && !evidence.expect_hold && !evidence.mode_current) ?
      "ELEVATOR_OPERATING_MODE_LEASE_LOST" :
      (evidence.expect_correction_pause && !evidence.correction_pause_current) ?
      "ELEVATOR_CORRECTION_PAUSE_LOST" : "";
    const auto failure = check_elevator_cached_health(evidence, false);
    EXPECT_EQ(failure ? failure->code : "", expected) << "mask=" << mask;
  }
}

}  // namespace
}  // namespace robot_api_server
