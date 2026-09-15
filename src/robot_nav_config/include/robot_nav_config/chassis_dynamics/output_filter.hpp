#pragma once

#include "nav2_mppi_controller/motion_models.hpp"
#include "nav2_mppi_controller/tools/utils.hpp"

namespace robot_nav_config::chassis_dynamics
{
// Finalize the optimizer's sequence BEFORE selecting or shifting its command.
inline void filter_control_sequence(
  mppi::models::ControlSequence & sequence,
  std::array<mppi::models::Control, 4> & history,
  const mppi::models::OptimizerSettings & settings,
  mppi::MotionModel & model)
{
  const auto previous_history = history;
  mppi::utils::savitskyGolayFilter(sequence, history, settings);

  // Signed SG weights can overshoot even a previously constrained sequence.
  // Restore CURRENT speed limits, then the native model's coupled constraints.
  // This is not another acceleration smoother or a low-speed deadband.
  const auto & limits = settings.constraints;
  sequence.vx = xt::clip(sequence.vx, limits.vx_min, limits.vx_max);
  sequence.wz = xt::clip(sequence.wz, -limits.wz, limits.wz);
  if (model.isHolonomic()) {
    sequence.vy = xt::clip(sequence.vy, -limits.vy, limits.vy);
  } else {
    sequence.vy.fill(0.0F);
  }
  model.applyConstraints(sequence);

  // Commit once, using the same offset as Humble's command selector. Preserve
  // the actual older commands even when a newly lowered speed limit excludes
  // them. Also works when a short horizon makes the native filter skip history.
  const unsigned int offset = settings.shift_control_sequence ? 1 : 0;
  history = {previous_history[1], previous_history[2], previous_history[3],
    {sequence.vx(offset), sequence.vy(offset), sequence.wz(offset)}};
}
}
