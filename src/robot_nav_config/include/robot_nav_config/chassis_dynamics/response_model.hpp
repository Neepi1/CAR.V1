#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>

namespace robot_nav_config::chassis_dynamics
{
// Fitted from the 2026-06-30 CAN captures, NOT the velocity-smoother limits.
struct Parameters
{
  double linear_delay{0.11};
  double linear_tau{0.01823};
  double acceleration{0.61888};
  double deceleration{1.80150};
  double steering_delay{0.11};
  double steering_tau{0.05402};
  double wheelbase{0.494};
  double track{0.364};

  void validate() const
  {
    for (double v : {linear_tau, acceleration, deceleration, steering_tau, wheelbase, track}) {
      if (!std::isfinite(v) || v <= 0.0) {throw std::invalid_argument("Invalid Ranger response coefficient");}
    }
    for (double v : {linear_delay, steering_delay}) {
      if (!std::isfinite(v) || v < 0.0 || v > 0.5) {
        throw std::invalid_argument("Ranger response delay outside [0, 0.5] seconds");
      }
    }
  }
};

struct Command {double time; double linear; double angular;};
struct State {double linear; double steering; double angular;};

// Pure plant model. Its input is AFTER velocity smoothing. It never changes a
// real command, mode, footprint or safety decision, and never publishes ROS.
class ResponseModel
{
public:
  explicit ResponseModel(Parameters parameters = {}) : p_(parameters) {p_.validate();}

  double steering_for(double linear, double angular) const
  {
    if (std::abs(linear) < 1e-6 || std::abs(angular) < 1e-6) {return 0.0;}
    const double central = std::asin(std::min(std::abs(angular / linear) * p_.wheelbase / 2, 1.0));
    const double inner = std::atan2(p_.wheelbase * std::sin(central),
      p_.wheelbase * std::cos(central) - p_.track * std::sin(central));
    return std::copysign(std::min(inner, 40.0 * M_PI / 180.0), angular * linear);
  }

  double yaw_rate(double linear, double inner) const
  {
    const double magnitude = std::abs(inner);
    const double central = std::atan2(p_.wheelbase * std::sin(magnitude),
      p_.wheelbase * std::cos(magnitude) + p_.track * std::sin(magnitude));
    return 2 * linear * std::sin(std::copysign(central, inner)) / p_.wheelbase;
  }

  void reset(double linear, double steering, double last_linear, double last_angular)
  {
    state_ = {linear, steering, yaw_rate(linear, steering)};
    time_ = 0.0;
    count_ = 0;
    next_ = 0;
    remember({-1.0, last_linear, last_angular});
  }

  // Times are relative to this prediction's start; only already-issued commands.
  void remember(Command command)
  {
    history_[next_] = command;
    next_ = (next_ + 1) % history_.size();
    count_ = std::min(count_ + 1, history_.size());
  }

  State advance(double linear_request, double angular_request, double dt)
  {
    if (!std::isfinite(dt) || dt <= 0 || dt > 0.1 ||
      !std::isfinite(linear_request) || !std::isfinite(angular_request))
    {
      throw std::invalid_argument("Invalid Ranger prediction input/time step");
    }
    remember({time_, linear_request, angular_request});
    const auto linear_command = delayed(time_ - p_.linear_delay);
    const auto steering_command = delayed(time_ - p_.steering_delay);
    const double target = steering_for(steering_command.linear, steering_command.angular);
    const double dv = (linear_command.linear - state_.linear) * (-std::expm1(-dt / p_.linear_tau));
    state_.linear += std::clamp(dv, -p_.deceleration * dt, p_.acceleration * dt);
    state_.steering += (target - state_.steering) * (-std::expm1(-dt / p_.steering_tau));
    state_.angular = yaw_rate(state_.linear, state_.steering);
    time_ += dt;
    return state_;
  }

private:
  Command delayed(double time) const
  {
    // At 15 Hz only ~3 entries are inspected. Fixed storage avoids per-rollout allocation.
    for (std::size_t i = 0; i < count_; ++i) {
      const auto index = (next_ + history_.size() - 1 - i) % history_.size();
      if (history_[index].time <= time + 1e-9 || i + 1 == count_) {return history_[index];}
    }
    return {time, 0.0, 0.0};
  }
  Parameters p_;
  State state_{};
  double time_{0};
  std::array<Command, 128> history_{};
  std::size_t next_{0};
  std::size_t count_{0};
};
}
