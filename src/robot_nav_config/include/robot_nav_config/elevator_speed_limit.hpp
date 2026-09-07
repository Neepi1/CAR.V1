#pragma once

#include <algorithm>
#include <cmath>

namespace robot_nav_config
{

inline double elevator_speed_limit_scale(
  const double speed_limit,
  const bool percentage,
  const double configured_max_speed)
{
  // nav2_msgs::msg::SpeedLimit uses 0.0 as NO_SPEED_LIMIT. It is not a stop
  // command. Safety stops remain owned by collision_monitor and robot_safety.
  if (!std::isfinite(speed_limit) || speed_limit <= 0.0) {
    return 1.0;
  }
  if (percentage) {
    return std::clamp(speed_limit / 100.0, 0.0, 1.0);
  }
  if (!std::isfinite(configured_max_speed) || configured_max_speed <= 0.0) {
    return 0.0;
  }
  return std::clamp(speed_limit / configured_max_speed, 0.0, 1.0);
}

} // namespace robot_nav_config
