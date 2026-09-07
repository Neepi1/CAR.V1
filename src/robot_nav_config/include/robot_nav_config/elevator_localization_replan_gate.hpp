#pragma once

#include <cmath>
#include <optional>

namespace robot_nav_config {

struct ElevatorLocalizationReplanObservation {
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

struct ElevatorLocalizationReplanParameters {
  bool enabled{true};
  double translation_trigger_m{0.08};
  double yaw_trigger_rad{0.08};
  // Retained for deployed YAML compatibility.  These former settling-gate
  // values no longer influence motion or route replacement.
  double stable_translation_m{0.015};
  double stable_yaw_rad{0.015};
  double stable_duration_sec{0.60};
};

enum class ElevatorLocalizationReplanAction {
  kTrack,
  kCorrectionDetected,
  kHold,
  kRequestReplan,
};

class ElevatorLocalizationReplanGate {
public:
  explicit ElevatorLocalizationReplanGate(
      ElevatorLocalizationReplanParameters parameters = {})
      : parameters_(parameters) {}

  void set_parameters(const ElevatorLocalizationReplanParameters &parameters) {
    parameters_ = parameters;
    reset();
  }

  void reset() noexcept {
    baseline_.reset();
    stable_anchor_.reset();
    stable_since_sec_.reset();
    correction_pending_ = false;
    correction_episode_active_ = false;
  }

  ElevatorLocalizationReplanAction
  observe(const ElevatorLocalizationReplanObservation &observation,
          const double now_sec) {
    if (!observation_is_finite(observation) || !std::isfinite(now_sec)) {
      return ElevatorLocalizationReplanAction::kHold;
    }
    if (!parameters_.enabled || !parameters_are_valid()) {
      baseline_ = observation;
      stable_anchor_.reset();
      stable_since_sec_.reset();
      correction_episode_active_ = false;
      correction_pending_ = false;
      return ElevatorLocalizationReplanAction::kTrack;
    }
    if (!baseline_) {
      baseline_ = observation;
      stable_anchor_ = observation;
      stable_since_sec_ = now_sec;
      return ElevatorLocalizationReplanAction::kTrack;
    }

    if (correction_episode_active_) {
      if (!stable_anchor_ || !stable_since_sec_ ||
          now_sec < *stable_since_sec_ ||
          translation_between(*stable_anchor_, observation) >=
              parameters_.stable_translation_m ||
          yaw_between(*stable_anchor_, observation) >=
              parameters_.stable_yaw_rad) {
        stable_anchor_ = observation;
        stable_since_sec_ = now_sec;
      } else if (now_sec - *stable_since_sec_ >=
                 parameters_.stable_duration_sec) {
        // Settling ends only the diagnostic episode.  It never changes motion
        // state and exists solely to coalesce a burst into one audit event.
        correction_episode_active_ = false;
        baseline_ = observation;
      }
      return ElevatorLocalizationReplanAction::kTrack;
    }

    const bool threshold_crossed =
        translation_between(*baseline_, observation) >=
            parameters_.translation_trigger_m ||
        yaw_between(*baseline_, observation) >= parameters_.yaw_trigger_rad;
    if (threshold_crossed) {
      // map->odom is allowed to move while a FollowPath is active.  The
      // controller transforms the active map-frame target on every cycle, so
      // a material correction is diagnostic evidence, not authority to pause
      // motion or request a replacement path.
      correction_pending_ = true;
      correction_episode_active_ = true;
      baseline_ = observation;
      stable_anchor_ = observation;
      stable_since_sec_ = now_sec;
    }
    return ElevatorLocalizationReplanAction::kTrack;
  }

  bool consume_correction_event() noexcept {
    const bool detected = correction_pending_;
    correction_pending_ = false;
    return detected;
  }

  bool correction_pending() const noexcept { return correction_pending_; }

private:
  static bool observation_is_finite(
      const ElevatorLocalizationReplanObservation &observation) noexcept {
    return std::isfinite(observation.x) && std::isfinite(observation.y) &&
           std::isfinite(observation.yaw);
  }

  bool parameters_are_valid() const noexcept {
    return std::isfinite(parameters_.translation_trigger_m) &&
           std::isfinite(parameters_.yaw_trigger_rad) &&
           std::isfinite(parameters_.stable_translation_m) &&
           std::isfinite(parameters_.stable_yaw_rad) &&
           std::isfinite(parameters_.stable_duration_sec) &&
           parameters_.translation_trigger_m > 0.0 &&
           parameters_.yaw_trigger_rad > 0.0 &&
           parameters_.stable_translation_m > 0.0 &&
           parameters_.stable_yaw_rad > 0.0 &&
           parameters_.stable_duration_sec >= 0.0;
  }

  static double translation_between(
      const ElevatorLocalizationReplanObservation &lhs,
      const ElevatorLocalizationReplanObservation &rhs) noexcept {
    return std::hypot(rhs.x - lhs.x, rhs.y - lhs.y);
  }

  static double
  yaw_between(const ElevatorLocalizationReplanObservation &lhs,
              const ElevatorLocalizationReplanObservation &rhs) noexcept {
    return std::abs(std::remainder(rhs.yaw - lhs.yaw, 2.0 * std::acos(-1.0)));
  }

  ElevatorLocalizationReplanParameters parameters_;
  std::optional<ElevatorLocalizationReplanObservation> baseline_;
  std::optional<ElevatorLocalizationReplanObservation> stable_anchor_;
  std::optional<double> stable_since_sec_;
  bool correction_pending_{false};
  bool correction_episode_active_{false};
};

} // namespace robot_nav_config
