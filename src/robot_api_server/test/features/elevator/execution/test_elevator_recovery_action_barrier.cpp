#include <gtest/gtest.h>

#include <cstdint>
#include <vector>

#include "robot_api_server/features/elevator/execution/elevator_recovery_action_barrier.hpp"

namespace
{

using robot_api_server::ElevatorRecoveryActionBarrier;
using robot_api_server::ElevatorRecoveryActionBarrierState;
using robot_api_server::ElevatorRecoveryGoalId;
using robot_api_server::ElevatorRecoveryGoalState;
using robot_api_server::ElevatorRecoveryGoalStatus;

ElevatorRecoveryGoalId goal(const std::uint8_t value)
{
  ElevatorRecoveryGoalId id{};
  id.back() = value;
  return id;
}

ElevatorRecoveryGoalStatus status(
  const std::uint8_t id,
  const ElevatorRecoveryGoalState state)
{
  return {goal(id), state};
}

TEST(ElevatorRecoveryActionBarrier, EmptyCancelNeedsNoStatusMessage)
{
  ElevatorRecoveryActionBarrier barrier;
  barrier.begin_round();
  barrier.record_cancel_response({});
  EXPECT_EQ(
    barrier.state(),
    ElevatorRecoveryActionBarrierState::kIdleProven);
}

TEST(ElevatorRecoveryActionBarrier, CancelResponseBelongsToRoundUntilReset)
{
  ElevatorRecoveryActionBarrier barrier;
  barrier.begin_round();
  barrier.record_cancel_response({});
  for (int observation = 0; observation < 100; ++observation) {
    EXPECT_EQ(
      barrier.state(),
      ElevatorRecoveryActionBarrierState::kIdleProven);
  }
  barrier.begin_round();
  EXPECT_EQ(barrier.state(), ElevatorRecoveryActionBarrierState::kWaiting);
}

TEST(ElevatorRecoveryActionBarrier, PreResponseStatusCannotProveCanceledGoal)
{
  ElevatorRecoveryActionBarrier barrier;
  barrier.begin_round();
  barrier.observe_post_response_status(
    {status(1U, ElevatorRecoveryGoalState::kTerminal)});
  barrier.record_cancel_response({goal(1U)});
  EXPECT_EQ(barrier.state(), ElevatorRecoveryActionBarrierState::kWaiting);
}

TEST(ElevatorRecoveryActionBarrier, PreResponseActiveIsIgnoredForEmptyCancel)
{
  ElevatorRecoveryActionBarrier barrier;
  barrier.begin_round();
  barrier.observe_post_response_status(
    {status(1U, ElevatorRecoveryGoalState::kActive)});
  barrier.record_cancel_response({});
  EXPECT_EQ(
    barrier.state(),
    ElevatorRecoveryActionBarrierState::kIdleProven);
}

TEST(ElevatorRecoveryActionBarrier, PostResponseActiveRejectsEmptyCancel)
{
  ElevatorRecoveryActionBarrier barrier;
  barrier.begin_round();
  barrier.record_cancel_response({});
  barrier.observe_post_response_status(
    {status(1U, ElevatorRecoveryGoalState::kActive)});
  EXPECT_EQ(barrier.state(), ElevatorRecoveryActionBarrierState::kUnsafe);
}

TEST(ElevatorRecoveryActionBarrier, EveryNamedGoalNeedsExactTerminalEvidence)
{
  ElevatorRecoveryActionBarrier barrier;
  barrier.begin_round();
  barrier.record_cancel_response({goal(1U), goal(2U)});
  barrier.observe_post_response_status(
    {
      status(1U, ElevatorRecoveryGoalState::kTerminal),
      status(2U, ElevatorRecoveryGoalState::kActive),
    });
  EXPECT_EQ(barrier.state(), ElevatorRecoveryActionBarrierState::kWaiting);
  barrier.observe_post_response_status(
    {status(2U, ElevatorRecoveryGoalState::kTerminal)});
  EXPECT_EQ(
    barrier.state(),
    ElevatorRecoveryActionBarrierState::kIdleProven);
}

TEST(ElevatorRecoveryActionBarrier, OtherTerminalGoalDoesNotProveNamedGoal)
{
  ElevatorRecoveryActionBarrier barrier;
  barrier.begin_round();
  barrier.record_cancel_response({goal(1U)});
  barrier.observe_post_response_status(
    {status(2U, ElevatorRecoveryGoalState::kTerminal)});
  EXPECT_EQ(barrier.state(), ElevatorRecoveryActionBarrierState::kWaiting);
}

TEST(ElevatorRecoveryActionBarrier, UnexpectedPostResponseActiveGoalIsUnsafe)
{
  ElevatorRecoveryActionBarrier barrier;
  barrier.begin_round();
  barrier.record_cancel_response({goal(1U)});
  barrier.observe_post_response_status(
    {status(2U, ElevatorRecoveryGoalState::kActive)});
  EXPECT_EQ(barrier.state(), ElevatorRecoveryActionBarrierState::kUnsafe);
}

TEST(ElevatorRecoveryActionBarrier, UnknownPostResponseStateFailsClosed)
{
  ElevatorRecoveryActionBarrier barrier;
  barrier.begin_round();
  barrier.record_cancel_response({});
  barrier.observe_post_response_status(
    {status(1U, ElevatorRecoveryGoalState::kUnknown)});
  EXPECT_EQ(barrier.state(), ElevatorRecoveryActionBarrierState::kUnsafe);
}

TEST(ElevatorRecoveryActionBarrier, TerminalToActiveRegressionIsUnsafe)
{
  ElevatorRecoveryActionBarrier barrier;
  barrier.begin_round();
  barrier.record_cancel_response({goal(1U)});
  barrier.observe_post_response_status(
    {status(1U, ElevatorRecoveryGoalState::kTerminal)});
  ASSERT_EQ(
    barrier.state(),
    ElevatorRecoveryActionBarrierState::kIdleProven);
  barrier.observe_post_response_status(
    {status(1U, ElevatorRecoveryGoalState::kActive)});
  EXPECT_EQ(barrier.state(), ElevatorRecoveryActionBarrierState::kUnsafe);
}

}  // namespace
