#pragma once

#include <optional>
#include <string>

namespace robot_api_server
{

// Existing adapter evidence only: no black-box health/status query or ROS call.
struct ElevatorCachedHealth
{
  bool executor_unavailable{false};
  bool cancel_requested{false};
  bool mode_ownership_unknown{false};
  bool correction_ownership_unknown{false};
  bool expect_hold{false};
  bool hold_current{false};
  bool expect_mode{false};
  bool mode_current{false};
  bool expect_correction_pause{false};
  bool correction_pause_current{false};
};

struct ElevatorCachedHealthFailure
{
  std::string code;
  std::string detail;
};

inline std::optional<ElevatorCachedHealthFailure> check_elevator_cached_health(
  const ElevatorCachedHealth & evidence, const bool arm_wait)
{
  if (evidence.executor_unavailable) {
    return ElevatorCachedHealthFailure{
      "ELEVATOR_RUNTIME_EXECUTOR_UNHEALTHY",
      "ROS runtime executor is unavailable"};
  }
  if (!arm_wait && evidence.cancel_requested) {
    return ElevatorCachedHealthFailure{
      "ELEVATOR_RUNTIME_CANCEL_REQUESTED",
      "elevator transaction cancellation is pending cleanup"};
  }
  // poll_health performs the existing renewal/recovery RPC before this check.
  // An arm wait must not turn its cached mode failure into a new rejection:
  // the existing worker/motion boundary remains responsible for renewal.
  if ((!arm_wait && evidence.mode_ownership_unknown) ||
    evidence.correction_ownership_unknown)
  {
    return ElevatorCachedHealthFailure{
      "ELEVATOR_RUNTIME_RESOURCE_STATE_UNPROVEN",
      "mode or correction-pause outcome is unknown"};
  }
  if (evidence.expect_hold && !evidence.hold_current) {
    return ElevatorCachedHealthFailure{
      "ELEVATOR_SAFETY_HOLD_LOST",
      "owner-scoped safety hold is missing or stale"};
  }
  if (!arm_wait && evidence.expect_mode && !evidence.expect_hold &&
    !evidence.mode_current)
  {
    return ElevatorCachedHealthFailure{
      "ELEVATOR_OPERATING_MODE_LEASE_LOST",
      "owner-scoped operating mode lease is missing, stale, or mismatched"};
  }
  if (evidence.expect_correction_pause && !evidence.correction_pause_current) {
    return ElevatorCachedHealthFailure{
      "ELEVATOR_CORRECTION_PAUSE_LOST",
      "owner-scoped localization correction pause is missing or stale"};
  }
  return std::nullopt;
}

}  // namespace robot_api_server
