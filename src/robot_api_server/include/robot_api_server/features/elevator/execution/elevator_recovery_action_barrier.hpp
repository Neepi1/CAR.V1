#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace robot_api_server
{

using ElevatorRecoveryGoalId = std::array<std::uint8_t, 16>;

enum class ElevatorRecoveryGoalState
{
  kUnknown,
  kActive,
  kTerminal,
};

struct ElevatorRecoveryGoalStatus
{
  ElevatorRecoveryGoalId goal_id{};
  ElevatorRecoveryGoalState state{ElevatorRecoveryGoalState::kUnknown};
};

enum class ElevatorRecoveryActionBarrierState
{
  kWaiting,
  kIdleProven,
  kUnsafe,
};

// Models one cancel-all round. Status samples observed before the response are
// deliberately ignored: they cannot prove what remained after cancellation.
// An empty successful response is itself an idle proof, while every goal named
// by a non-empty response must subsequently be observed in a terminal state.
class ElevatorRecoveryActionBarrier
{
public:
  void begin_round();
  void record_cancel_response(
    const std::vector<ElevatorRecoveryGoalId> & goals_canceling);
  void observe_post_response_status(
    const std::vector<ElevatorRecoveryGoalStatus> & statuses);

  ElevatorRecoveryActionBarrierState state() const noexcept;
  const std::string & detail() const noexcept;

private:
  bool response_recorded_{false};
  bool unsafe_{false};
  std::vector<ElevatorRecoveryGoalId> expected_goal_ids_;
  std::vector<ElevatorRecoveryGoalId> terminal_goal_ids_;
  std::string detail_{"cancel-all response has not been recorded"};
};

}  // namespace robot_api_server
