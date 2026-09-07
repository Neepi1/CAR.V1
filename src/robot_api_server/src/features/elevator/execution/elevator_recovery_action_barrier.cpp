#include "robot_api_server/features/elevator/execution/elevator_recovery_action_barrier.hpp"

#include <algorithm>

namespace robot_api_server
{
namespace
{

bool contains_goal(
  const std::vector<ElevatorRecoveryGoalId> & goals,
  const ElevatorRecoveryGoalId & goal)
{
  return std::find(goals.cbegin(), goals.cend(), goal) != goals.cend();
}

}  // namespace

void ElevatorRecoveryActionBarrier::begin_round()
{
  response_recorded_ = false;
  unsafe_ = false;
  expected_goal_ids_.clear();
  terminal_goal_ids_.clear();
  detail_ = "cancel-all response has not been recorded";
}

void ElevatorRecoveryActionBarrier::record_cancel_response(
  const std::vector<ElevatorRecoveryGoalId> & goals_canceling)
{
  response_recorded_ = true;
  unsafe_ = false;
  expected_goal_ids_.clear();
  terminal_goal_ids_.clear();
  for (const auto & goal : goals_canceling) {
    if (!contains_goal(expected_goal_ids_, goal)) {
      expected_goal_ids_.push_back(goal);
    }
  }
  detail_ = expected_goal_ids_.empty() ?
    "cancel-all response reported no matching goals" :
    "waiting for post-response terminal status for every canceled goal";
}

void ElevatorRecoveryActionBarrier::observe_post_response_status(
  const std::vector<ElevatorRecoveryGoalStatus> & statuses)
{
  if (!response_recorded_ || unsafe_) {
    return;
  }

  for (const auto & status : statuses) {
    if (status.state == ElevatorRecoveryGoalState::kUnknown) {
      unsafe_ = true;
      detail_ = "post-response action status contained an unknown goal state";
      return;
    }

    const bool expected =
      contains_goal(expected_goal_ids_, status.goal_id);
    if (status.state == ElevatorRecoveryGoalState::kActive) {
      if (!expected) {
        unsafe_ = true;
        detail_ =
          "a goal not named by cancel-all became active after its response";
        return;
      }
      if (contains_goal(terminal_goal_ids_, status.goal_id)) {
        unsafe_ = true;
        detail_ =
          "a canceled goal regressed from terminal to active";
        return;
      }
      continue;
    }

    if (
      expected &&
      !contains_goal(terminal_goal_ids_, status.goal_id))
    {
      terminal_goal_ids_.push_back(status.goal_id);
    }
  }

  if (
    !expected_goal_ids_.empty() &&
    terminal_goal_ids_.size() == expected_goal_ids_.size())
  {
    detail_ =
      "every goal named by cancel-all has post-response terminal evidence";
  }
}

ElevatorRecoveryActionBarrierState
ElevatorRecoveryActionBarrier::state() const noexcept
{
  if (!response_recorded_) {
    return ElevatorRecoveryActionBarrierState::kWaiting;
  }
  if (unsafe_) {
    return ElevatorRecoveryActionBarrierState::kUnsafe;
  }
  if (
    expected_goal_ids_.empty() ||
    terminal_goal_ids_.size() == expected_goal_ids_.size())
  {
    return ElevatorRecoveryActionBarrierState::kIdleProven;
  }
  return ElevatorRecoveryActionBarrierState::kWaiting;
}

const std::string & ElevatorRecoveryActionBarrier::detail() const noexcept
{
  return detail_;
}

}  // namespace robot_api_server
