#pragma once

#include <algorithm>
#include <chrono>
#include <optional>

namespace robot_fastlio_mapping
{

enum class ScanTfGateDecision
{
  wait,
  release,
  drop_timeout,
};

class MappingScanTfGatePolicy
{
public:
  using Clock = std::chrono::steady_clock;
  using TimePoint = Clock::time_point;
  using Duration = Clock::duration;

  struct Timing
  {
    TimePoint arrival;
    std::optional<TimePoint> tf_ready_since;
  };

  MappingScanTfGatePolicy(
    const std::chrono::duration<double> max_wait,
    const std::chrono::duration<double> post_tf_settle)
  : max_wait_(to_nonnegative_duration(max_wait)),
    post_tf_settle_(to_nonnegative_duration(post_tf_settle))
  {
  }

  void mark_tf_ready_before_enqueue(Timing & timing) const
  {
    timing.tf_ready_since = timing.arrival - post_tf_settle_;
  }

  ScanTfGateDecision evaluate(
    Timing & timing,
    const TimePoint now,
    const bool tf_available) const
  {
    if (max_wait_ > Duration::zero() && now - timing.arrival >= max_wait_) {
      return ScanTfGateDecision::drop_timeout;
    }

    if (!tf_available) {
      timing.tf_ready_since.reset();
      return ScanTfGateDecision::wait;
    }

    if (!timing.tf_ready_since) {
      timing.tf_ready_since = now;
    }
    if (now - *timing.tf_ready_since >= post_tf_settle_) {
      return ScanTfGateDecision::release;
    }
    return ScanTfGateDecision::wait;
  }

  Duration max_wait() const
  {
    return max_wait_;
  }

  Duration post_tf_settle() const
  {
    return post_tf_settle_;
  }

private:
  static Duration to_nonnegative_duration(const std::chrono::duration<double> value)
  {
    const auto nonnegative = std::max(value, std::chrono::duration<double>::zero());
    return std::chrono::duration_cast<Duration>(nonnegative);
  }

  Duration max_wait_;
  Duration post_tf_settle_;
};

}  // namespace robot_fastlio_mapping
