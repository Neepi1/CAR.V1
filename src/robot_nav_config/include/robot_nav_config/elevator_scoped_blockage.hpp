#pragma once

#include <chrono>
#include <cmath>
#include <optional>

namespace robot_nav_config
{

class ElevatorScopedBlockage
{
public:
  using Clock = std::chrono::steady_clock;
  using TimePoint = Clock::time_point;

  void reset() noexcept {blocked_since_.reset();}

  void observe_blocked(const TimePoint now) noexcept
  {
    if (!blocked_since_) {
      blocked_since_ = now;
    }
  }

  // Producing another zero command is not evidence that physical progress
  // resumed, so it does not extend the blocked episode deadline.
  void observe_zero_command() noexcept {}

  void observe_verified_nonzero_command() noexcept
  {
    // Replan admission is tracked independently by
    // ElevatorScopedReplanProgress. This class owns only blockage timing.
    blocked_since_.reset();
  }

  bool active() const noexcept {return blocked_since_.has_value();}

  bool expired(const TimePoint now, const double timeout_sec) const noexcept
  {
    if (!blocked_since_ || !std::isfinite(timeout_sec) || timeout_sec < 0.0) {
      return false;
    }
    return std::chrono::duration<double>(now - *blocked_since_).count() >=
           timeout_sec;
  }

private:
  std::optional<TimePoint> blocked_since_;
};

} // namespace robot_nav_config
