#pragma once

#include <algorithm>

namespace robot_safety
{

struct NavigationSpeedEnvelope
{
  double reverse_max_mps{0.0};
  double lateral_max_mps{0.0};
};

inline NavigationSpeedEnvelope select_navigation_speed_envelope(
  const bool elevator_execution_session_engaged,
  const double normal_reverse_max_mps,
  const double normal_lateral_max_mps,
  const double elevator_reverse_max_mps,
  const double elevator_lateral_max_mps)
{
  return {
    std::max(
      0.0,
      elevator_execution_session_engaged ?
      elevator_reverse_max_mps : normal_reverse_max_mps),
    std::max(
      0.0,
      elevator_execution_session_engaged ?
      elevator_lateral_max_mps : normal_lateral_max_mps)};
}

}  // namespace robot_safety
