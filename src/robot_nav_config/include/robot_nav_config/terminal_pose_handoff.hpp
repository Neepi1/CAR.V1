#pragma once

#include <algorithm>
#include <cmath>
#include <optional>

namespace robot_nav_config
{

struct TerminalPoseError
{
  double distance_m{0.0};
  double forward_m{0.0};
  double lateral_m{0.0};
  double yaw_rad{0.0};
};

struct TerminalPathMetrics
{
  double chord_m{0.0};
  double length_m{0.0};
  double max_cross_track_m{0.0};
};

struct TerminalHandoffParameters
{
  bool enabled{true};
  double max_distance_m{0.40};
  double max_abs_forward_m{0.15};
  double minimum_abs_lateral_m{0.08};
  double lateral_dominance_ratio{1.20};
  double path_length_ratio_threshold{1.80};
  double path_cross_track_threshold_m{0.15};

  double goal_xy_tolerance_m{0.06};
  double goal_yaw_tolerance_rad{0.05};
  double yaw_exit_tolerance_rad{0.035};
  double lateral_entry_tolerance_m{0.04};
  double lateral_exit_tolerance_m{0.025};
  double forward_entry_tolerance_m{0.04};
  double forward_exit_tolerance_m{0.025};

  double yaw_kp{1.20};
  double yaw_min_speed_radps{0.08};
  double yaw_max_speed_radps{0.30};
  double lateral_kp{0.80};
  double lateral_max_speed_mps{0.05};
  double forward_kp{0.80};
  double forward_max_speed_mps{0.10};
  double reverse_max_speed_mps{0.08};

  double settle_linear_speed_threshold_mps{0.01};
  double settle_angular_speed_threshold_radps{0.02};
  double settle_stable_duration_sec{0.30};
  double total_timeout_sec{20.0};
};

inline bool terminal_values_are_finite(
  const TerminalPoseError & error,
  const TerminalPathMetrics & path)
{
  return std::isfinite(error.distance_m) &&
         std::isfinite(error.forward_m) &&
         std::isfinite(error.lateral_m) &&
         std::isfinite(error.yaw_rad) &&
         std::isfinite(path.chord_m) &&
         std::isfinite(path.length_m) &&
         std::isfinite(path.max_cross_track_m);
}

inline bool should_start_terminal_handoff(
  const TerminalPoseError & error,
  const TerminalPathMetrics & path,
  const TerminalHandoffParameters & parameters)
{
  constexpr double kMinimumChordM = 1.0e-3;
  if (!parameters.enabled || !terminal_values_are_finite(error, path) ||
    error.distance_m > parameters.max_distance_m ||
    std::abs(error.forward_m) > parameters.max_abs_forward_m ||
    std::abs(error.lateral_m) < parameters.minimum_abs_lateral_m)
  {
    return false;
  }

  const bool lateral_residual_is_kinematically_unsuitable =
    std::abs(error.lateral_m) >=
    parameters.lateral_dominance_ratio * std::abs(error.forward_m);
  const bool path_is_geometrically_unsuitable =
    path.chord_m >= kMinimumChordM &&
    (path.length_m / path.chord_m > parameters.path_length_ratio_threshold ||
    path.max_cross_track_m > parameters.path_cross_track_threshold_m);
  return lateral_residual_is_kinematically_unsuitable || path_is_geometrically_unsuitable;
}

enum class TerminalControlPhase
{
  kInactive,
  kSettling,
  kYaw,
  kLateral,
  kForward,
  kComplete,
  kFailed,
};

enum class TerminalControlFailureReason
{
  kNone,
  kInvalidInput,
  kClockRegression,
  kTimeout,
};

inline const char * terminal_control_failure_reason_name(
  const TerminalControlFailureReason reason) noexcept
{
  switch (reason) {
    case TerminalControlFailureReason::kNone:
      return "none";
    case TerminalControlFailureReason::kInvalidInput:
      return "invalid_input";
    case TerminalControlFailureReason::kClockRegression:
      return "clock_regression";
    case TerminalControlFailureReason::kTimeout:
      return "timeout";
  }
  return "invalid_input";
}

struct TerminalVelocityCommand
{
  double linear_x{0.0};
  double linear_y{0.0};
  double angular_z{0.0};
};

struct TerminalControlInput
{
  TerminalPoseError error;
  double actual_linear_x_mps{0.0};
  double actual_linear_y_mps{0.0};
  double actual_angular_z_radps{0.0};
  double now_sec{0.0};
};

struct TerminalControlOutput
{
  TerminalControlPhase phase{TerminalControlPhase::kInactive};
  TerminalVelocityCommand command;
  bool active{false};
  bool complete{false};
  bool failed{false};
  bool lateral_permit{false};
  bool reverse_permit{false};
  TerminalControlFailureReason failure_reason{
    TerminalControlFailureReason::kNone};
};

class TerminalPoseHandoffController
{
public:
  explicit TerminalPoseHandoffController(
    const TerminalHandoffParameters & parameters = TerminalHandoffParameters{})
  : parameters_(parameters)
  {
  }

  void set_parameters(const TerminalHandoffParameters & parameters)
  {
    parameters_ = parameters;
    reset();
  }

  void reset()
  {
    phase_ = TerminalControlPhase::kInactive;
    failure_reason_ = TerminalControlFailureReason::kNone;
    started_at_sec_ = 0.0;
    settle_stable_since_sec_.reset();
  }

  void begin(const double now_sec)
  {
    phase_ = TerminalControlPhase::kSettling;
    failure_reason_ = TerminalControlFailureReason::kNone;
    started_at_sec_ = now_sec;
    settle_stable_since_sec_.reset();
  }

  bool active() const
  {
    return phase_ != TerminalControlPhase::kInactive &&
           phase_ != TerminalControlPhase::kComplete &&
           phase_ != TerminalControlPhase::kFailed;
  }

  TerminalControlPhase phase() const
  {
    return phase_;
  }

  TerminalControlOutput update(const TerminalControlInput & input)
  {
    if (phase_ == TerminalControlPhase::kInactive) {
      return make_output();
    }

    if (!input_is_finite(input)) {
      phase_ = TerminalControlPhase::kFailed;
      failure_reason_ = TerminalControlFailureReason::kInvalidInput;
      return make_output();
    }
    if (input.now_sec < started_at_sec_) {
      phase_ = TerminalControlPhase::kFailed;
      failure_reason_ = TerminalControlFailureReason::kClockRegression;
      return make_output();
    }
    if (parameters_.total_timeout_sec > 0.0 &&
      input.now_sec - started_at_sec_ > parameters_.total_timeout_sec)
    {
      phase_ = TerminalControlPhase::kFailed;
      failure_reason_ = TerminalControlFailureReason::kTimeout;
      return make_output();
    }

    if (phase_ == TerminalControlPhase::kComplete && !goal_is_accepted(input.error)) {
      enter_settling(input);
    }

    if (phase_ == TerminalControlPhase::kSettling) {
      if (!actual_motion_is_stable(input)) {
        settle_stable_since_sec_.reset();
        return make_output();
      }
      if (!settle_stable_since_sec_.has_value()) {
        settle_stable_since_sec_ = input.now_sec;
      }
      if (input.now_sec - *settle_stable_since_sec_ <
        parameters_.settle_stable_duration_sec)
      {
        return make_output();
      }
      phase_ = select_next_phase(input.error);
      settle_stable_since_sec_.reset();
    }

    if (phase_ == TerminalControlPhase::kYaw) {
      if (std::abs(input.error.yaw_rad) <= parameters_.yaw_exit_tolerance_rad) {
        enter_settling(input);
        return make_output();
      }
      auto output = make_output();
      output.command.angular_z = bounded_signed_command(
        input.error.yaw_rad,
        parameters_.yaw_kp,
        parameters_.yaw_min_speed_radps,
        parameters_.yaw_max_speed_radps);
      return output;
    }

    if (phase_ == TerminalControlPhase::kLateral) {
      if (std::abs(input.error.lateral_m) <= parameters_.lateral_exit_tolerance_m) {
        enter_settling(input);
        return make_output();
      }
      auto output = make_output();
      output.command.linear_y = std::clamp(
        parameters_.lateral_kp * input.error.lateral_m,
        -parameters_.lateral_max_speed_mps,
        parameters_.lateral_max_speed_mps);
      output.lateral_permit = true;
      return output;
    }

    if (phase_ == TerminalControlPhase::kForward) {
      if (std::abs(input.error.forward_m) <= parameters_.forward_exit_tolerance_m) {
        enter_settling(input);
        return make_output();
      }
      auto output = make_output();
      const double lower_bound = -parameters_.reverse_max_speed_mps;
      output.command.linear_x = std::clamp(
        parameters_.forward_kp * input.error.forward_m,
        lower_bound,
        parameters_.forward_max_speed_mps);
      output.reverse_permit = output.command.linear_x < 0.0;
      return output;
    }

    return make_output();
  }

private:
  static bool input_is_finite(const TerminalControlInput & input)
  {
    return std::isfinite(input.error.distance_m) &&
           std::isfinite(input.error.forward_m) &&
           std::isfinite(input.error.lateral_m) &&
           std::isfinite(input.error.yaw_rad) &&
           std::isfinite(input.actual_linear_x_mps) &&
           std::isfinite(input.actual_linear_y_mps) &&
           std::isfinite(input.actual_angular_z_radps) &&
           std::isfinite(input.now_sec);
  }

  bool actual_motion_is_stable(const TerminalControlInput & input) const
  {
    return std::hypot(
      input.actual_linear_x_mps,
      input.actual_linear_y_mps) <= parameters_.settle_linear_speed_threshold_mps &&
           std::abs(input.actual_angular_z_radps) <=
           parameters_.settle_angular_speed_threshold_radps;
  }

  bool goal_is_accepted(const TerminalPoseError & error) const
  {
    return error.distance_m <= parameters_.goal_xy_tolerance_m &&
           std::abs(error.yaw_rad) <= parameters_.goal_yaw_tolerance_rad;
  }

  TerminalControlPhase select_next_phase(const TerminalPoseError & error) const
  {
    if (std::abs(error.yaw_rad) > parameters_.goal_yaw_tolerance_rad) {
      return TerminalControlPhase::kYaw;
    }
    if (std::abs(error.lateral_m) > parameters_.lateral_entry_tolerance_m) {
      return TerminalControlPhase::kLateral;
    }
    if (std::abs(error.forward_m) > parameters_.forward_entry_tolerance_m ||
      error.distance_m > parameters_.goal_xy_tolerance_m)
    {
      return TerminalControlPhase::kForward;
    }
    return TerminalControlPhase::kComplete;
  }

  void enter_settling(const TerminalControlInput & input)
  {
    phase_ = TerminalControlPhase::kSettling;
    settle_stable_since_sec_ = actual_motion_is_stable(input) ?
      std::optional<double>(input.now_sec) : std::nullopt;
  }

  TerminalControlOutput make_output() const
  {
    TerminalControlOutput output;
    output.phase = phase_;
    output.active = active();
    output.complete = phase_ == TerminalControlPhase::kComplete;
    output.failed = phase_ == TerminalControlPhase::kFailed;
    output.failure_reason = failure_reason_;
    return output;
  }

  static double bounded_signed_command(
    const double error,
    const double proportional_gain,
    const double minimum_magnitude,
    const double maximum_magnitude)
  {
    const double magnitude = std::clamp(
      std::abs(proportional_gain * error), minimum_magnitude, maximum_magnitude);
    return std::copysign(magnitude, error);
  }

  TerminalHandoffParameters parameters_;
  TerminalControlPhase phase_{TerminalControlPhase::kInactive};
  double started_at_sec_{0.0};
  std::optional<double> settle_stable_since_sec_;
  TerminalControlFailureReason failure_reason_{
    TerminalControlFailureReason::kNone};
};

}  // namespace robot_nav_config
