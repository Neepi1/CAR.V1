#pragma once

#include <cstdint>

namespace robot_nav_config {

enum class ElevatorScopedProgressState : std::uint8_t {
  kOrdinary = 0U,
  kTracking = 1U,
  kReplanning = 2U,
  kWaitClear = 3U,
  kLocalizationSettling = 4U,
  kOrdinaryLocalReplanning = 5U,
  kOrdinaryLocalWaitClear = 6U,
};

inline bool elevator_scoped_progress_is_active(
    const ElevatorScopedProgressState state) noexcept {
  return state == ElevatorScopedProgressState::kTracking ||
         state == ElevatorScopedProgressState::kReplanning ||
         state == ElevatorScopedProgressState::kWaitClear ||
         state == ElevatorScopedProgressState::kLocalizationSettling;
}

inline bool elevator_scoped_progress_is_paused(
    const ElevatorScopedProgressState state) noexcept {
  return state == ElevatorScopedProgressState::kReplanning ||
         state == ElevatorScopedProgressState::kWaitClear ||
         state == ElevatorScopedProgressState::kLocalizationSettling ||
         state == ElevatorScopedProgressState::kOrdinaryLocalReplanning ||
         state == ElevatorScopedProgressState::kOrdinaryLocalWaitClear;
}

inline bool
elevator_scoped_progress_state_is_valid(const std::uint8_t value) noexcept {
  return value <= static_cast<std::uint8_t>(
                      ElevatorScopedProgressState::kOrdinaryLocalWaitClear);
}

} // namespace robot_nav_config
