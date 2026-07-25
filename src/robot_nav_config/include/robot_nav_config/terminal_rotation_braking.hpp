#pragma once

#include <algorithm>
#include <cmath>

namespace robot_nav_config
{

inline double limit_terminal_rotation_speed(
  const double requested_speed_radps,
  const double remaining_yaw_rad,
  const double max_angular_decel_radps2)
{
  if (!std::isfinite(requested_speed_radps) ||
    !std::isfinite(remaining_yaw_rad) ||
    !std::isfinite(max_angular_decel_radps2) ||
    max_angular_decel_radps2 <= 0.0)
  {
    return requested_speed_radps;
  }

  const double stopping_speed = std::sqrt(
    2.0 * max_angular_decel_radps2 * std::abs(remaining_yaw_rad));
  return std::copysign(
    std::min(std::abs(requested_speed_radps), stopping_speed),
    requested_speed_radps);
}

}  // namespace robot_nav_config
