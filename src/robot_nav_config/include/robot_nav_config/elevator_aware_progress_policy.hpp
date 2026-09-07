#pragma once

#include <cmath>
#include <optional>

#include "robot_nav_config/elevator_scoped_progress_state.hpp"

namespace robot_nav_config {

struct PoseProgressLimits {
  double required_translation_m{0.03};
  double required_yaw_rad{0.05};
  double time_allowance_sec{12.0};
};

struct PoseProgressSample {
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

class ElevatorAwareProgressPolicy {
public:
  ElevatorAwareProgressPolicy(const PoseProgressLimits &normal,
                              const PoseProgressLimits &elevator)
      : normal_(normal), elevator_(elevator) {}

  bool check(const PoseProgressSample &sample, const double now_sec,
             const bool elevator_scoped) {
    if (!sample_is_finite(sample) || !std::isfinite(now_sec) ||
        !limits_are_valid(normal_) || !limits_are_valid(elevator_)) {
      return false;
    }

    if (!baseline_ || !baseline_time_sec_ || !baseline_elevator_scoped_ ||
        now_sec < *baseline_time_sec_ ||
        elevator_scoped != *baseline_elevator_scoped_) {
      reset_baseline(sample, now_sec, elevator_scoped);
      return true;
    }

    const auto &limits = elevator_scoped ? elevator_ : normal_;
    const double translation =
        std::hypot(sample.x - baseline_->x, sample.y - baseline_->y);
    const double yaw = std::abs(
        std::remainder(sample.yaw - baseline_->yaw, 2.0 * std::acos(-1.0)));
    if (translation >= limits.required_translation_m ||
        yaw >= limits.required_yaw_rad) {
      reset_baseline(sample, now_sec, elevator_scoped);
      return true;
    }

    return now_sec - *baseline_time_sec_ <= limits.time_allowance_sec;
  }

  bool check(const PoseProgressSample &sample, const double now_sec,
             const ElevatorScopedProgressState state) {
    if (elevator_scoped_progress_is_paused(state)) {
      if (!sample_is_finite(sample) || !std::isfinite(now_sec)) {
        return false;
      }
      // Deliberate zero-output states are not failed motion.  Resetting here
      // makes the first subsequent TRACKING sample establish a new baseline.
      reset();
      return true;
    }
    return check(sample, now_sec, elevator_scoped_progress_is_active(state));
  }

  void reset() {
    baseline_.reset();
    baseline_time_sec_.reset();
    baseline_elevator_scoped_.reset();
  }

private:
  static bool limits_are_valid(const PoseProgressLimits &limits) {
    return std::isfinite(limits.required_translation_m) &&
           std::isfinite(limits.required_yaw_rad) &&
           std::isfinite(limits.time_allowance_sec) &&
           limits.required_translation_m > 0.0 &&
           limits.required_yaw_rad > 0.0 && limits.time_allowance_sec > 0.0;
  }

  static bool sample_is_finite(const PoseProgressSample &sample) {
    return std::isfinite(sample.x) && std::isfinite(sample.y) &&
           std::isfinite(sample.yaw);
  }

  void reset_baseline(const PoseProgressSample &sample, const double now_sec,
                      const bool elevator_scoped) {
    baseline_ = sample;
    baseline_time_sec_ = now_sec;
    baseline_elevator_scoped_ = elevator_scoped;
  }

  PoseProgressLimits normal_;
  PoseProgressLimits elevator_;
  std::optional<PoseProgressSample> baseline_;
  std::optional<double> baseline_time_sec_;
  std::optional<bool> baseline_elevator_scoped_;
};

} // namespace robot_nav_config
