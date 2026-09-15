#pragma once

#include <vector>
#include "nav2_mppi_controller/motion_models.hpp"
#include "robot_nav_config/chassis_dynamics/response_model.hpp"

namespace robot_nav_config::chassis_dynamics
{
struct PredictionInput
{
  std::vector<Command> issued;
  double smoother_linear{0};
  double smoother_angular{0};
  bool smoother_valid{false};
};

class RangerMotionModel : public mppi::AckermannMotionModel
{
public:
  RangerMotionModel(mppi::ParametersHandler * handler, const std::string & name,
    Parameters parameters, double model_dt,
    double linear_accel, double linear_decel, double angular_accel, double angular_decel)
  : mppi::AckermannMotionModel(handler, name), parameters_(parameters), dt_(model_dt),
    linear_accel_(linear_accel), linear_decel_(linear_decel),
    angular_accel_(angular_accel), angular_decel_(angular_decel)
  {
    parameters_.validate();
  }

  void set_input(PredictionInput input, double dt, double vx_min, double vx_max, double wz_max)
  {
    if (!std::isfinite(dt) || dt <= 0 || dt > 0.1) {
      throw std::invalid_argument("Ranger dynamics model_dt must be in (0, 0.1]");
    }
    input_ = std::move(input);
    dt_ = dt;
    vx_min_ = vx_min;
    vx_max_ = vx_max;
    wz_max_ = wz_max;
  }

  void predict(mppi::models::State & state) override
  {
    const auto batch = state.cvx.shape()[0];
    const auto steps = state.cvx.shape()[1];
    for (std::size_t b = 0; b < batch; ++b) {
      ResponseModel model(parameters_);
      const double v0 = state.vx(b, 0);
      const double w0 = state.wz(b, 0);
      // Odometry observes body curvature while moving. At rest, wheel angle
      // is unobservable here: neutral is an explicit initialization assumption.
      const double initial_steering = model.steering_for(v0, w0);
      model.reset(v0, initial_steering, v0, w0);
      for (const auto & command : input_.issued) {model.remember(command);}
      double sv = input_.smoother_valid ? input_.smoother_linear : v0;
      double sw = input_.smoother_valid ? input_.smoother_angular : w0;
      for (std::size_t t = 1; t < steps; ++t) {
        const double target_v = std::clamp(static_cast<double>(state.cvx(b, t - 1)), vx_min_, vx_max_);
        const double target_w = std::clamp(static_cast<double>(state.cwz(b, t - 1)),
          -std::min(wz_max_, std::abs(target_v) / getMinTurningRadius()),
          std::min(wz_max_, std::abs(target_v) / getMinTurningRadius()));
        sv = smooth(sv, target_v, linear_accel_, linear_decel_, dt_);
        sw = smooth(sw, target_w, angular_accel_, angular_decel_, dt_);
        const auto prediction = model.advance(sv, sw, dt_);
        state.vx(b, t) = prediction.linear;
        state.wz(b, t) = prediction.angular;
      }
    }
  }

  static double smooth(double value, double target, double accel, double decel, double dt)
  {
    // Same signed per-axis acceleration/deceleration distinction as Humble's
    // OPEN_LOOP smoother. This predicts its effect; it does not smooth output twice.
    const bool accelerating = std::abs(target) >= std::abs(value) && value * target >= 0;
    const double increment = (accelerating ? accel : decel) * dt;
    return value + std::clamp(target - value, -increment, increment);
  }

private:
  Parameters parameters_;
  PredictionInput input_;
  double dt_, linear_accel_, linear_decel_, angular_accel_, angular_decel_;
  double vx_min_{0}, vx_max_{1.2}, wz_max_{0.7};
};
}
