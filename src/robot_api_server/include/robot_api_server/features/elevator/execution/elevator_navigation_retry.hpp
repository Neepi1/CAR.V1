#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <utility>

#include "robot_elevator_manager/elevator_execution_module.hpp"

namespace robot_api_server
{

// Facts from this exact Action attempt, never inferred from an error string.
enum class ElevatorNavigationAttemptEnd
{
  kUnproven,
  kAborted,
  kRejected,
  kTimeoutTerminal,
  kOtherTerminal,
};

struct ElevatorNavigationAttemptResult
{
  ElevatorNavigationAttemptResult(
    robot_elevator_manager::ElevatorRuntimeResult value,
    ElevatorNavigationAttemptEnd ending = ElevatorNavigationAttemptEnd::kUnproven)
  : result(std::move(value)), end(ending) {}

  robot_elevator_manager::ElevatorRuntimeResult result;
  ElevatorNavigationAttemptEnd end{ElevatorNavigationAttemptEnd::kUnproven};
};

inline std::string elevator_navigation_attempt_session(
  const std::string & effect_session, const std::uint64_t attempt)
{
  return attempt == 0U ? effect_session : effect_session + ":retry:" + std::to_string(attempt);
}

// Internal synchronous effect seam. The adapter retains all Action ownership,
// stop/hold handling, frozen goal, and interruptible waiting responsibilities.
template<typename Attempt, typename Pause>
robot_elevator_manager::ElevatorRuntimeResult run_elevator_navigation_retries(
  Attempt attempt, Pause pause)
{
  for (std::uint64_t index = 0U; ; ++index) {
    auto outcome = attempt(index);
    const bool retryable = outcome.end == ElevatorNavigationAttemptEnd::kAborted ||
      outcome.end == ElevatorNavigationAttemptEnd::kRejected ||
      outcome.end == ElevatorNavigationAttemptEnd::kTimeoutTerminal;
    if (outcome.result.success || !retryable) {
      return outcome.result;
    }
    if (auto interrupted = pause(index, outcome.result)) {
      return *interrupted;
    }
  }
}

}  // namespace robot_api_server
