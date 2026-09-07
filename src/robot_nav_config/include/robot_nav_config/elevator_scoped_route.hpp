#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

#include "robot_nav_config/elevator_scoped_path.hpp"
#include "robot_nav_config/terminal_pose_handoff.hpp"

namespace robot_nav_config
{

enum class ElevatorScopedRoutePhase
{
  kYaw,
  kLateral,
  kForward,
  kReverse,
  kPose,
};

struct ElevatorScopedRouteSegment
{
  ElevatorScopedRoutePhase phase{ElevatorScopedRoutePhase::kForward};
  std::size_t end_index{0U};
  int direction{1};
};

struct ElevatorScopedCommandProjection
{
  double translation_m{0.0};
  double rotation_rad{0.0};
};

inline TerminalPoseError
make_elevator_scoped_segment_error(
  const TerminalPoseError & error,
  const ElevatorScopedRouteSegment & segment)
{
  TerminalPoseError constrained = error;
  switch (segment.phase) {
    case ElevatorScopedRoutePhase::kYaw:
      constrained.distance_m = 0.0;
      constrained.forward_m = 0.0;
      constrained.lateral_m = 0.0;
      break;
    case ElevatorScopedRoutePhase::kLateral:
      constrained.forward_m = 0.0;
      constrained.lateral_m = segment.direction < 0 ?
        std::min(error.lateral_m, 0.0) :
        std::max(error.lateral_m, 0.0);
      constrained.distance_m = std::abs(constrained.lateral_m);
      break;
    case ElevatorScopedRoutePhase::kForward:
      constrained.forward_m = std::max(error.forward_m, 0.0);
      constrained.lateral_m = 0.0;
      constrained.distance_m = std::abs(constrained.forward_m);
      break;
    case ElevatorScopedRoutePhase::kReverse:
      constrained.forward_m = std::min(error.forward_m, 0.0);
      constrained.lateral_m = 0.0;
      constrained.distance_m = std::abs(constrained.forward_m);
      break;
    case ElevatorScopedRoutePhase::kPose:
      break;
  }
  return constrained;
}

inline ElevatorScopedCommandProjection
make_elevator_scoped_command_projection(
  const TerminalPoseError & active_error,
  const TerminalVelocityCommand & command,
  const double configured_translation_m,
  const double configured_rotation_rad)
{
  ElevatorScopedCommandProjection projection;
  if (!std::isfinite(configured_translation_m) ||
    !std::isfinite(configured_rotation_rad) ||
    configured_translation_m < 0.0 || configured_rotation_rad < 0.0)
  {
    return projection;
  }

  const double linear_norm = std::hypot(command.linear_x, command.linear_y);
  if (std::isfinite(linear_norm) && linear_norm > 1.0e-9) {
    const double remaining =
      std::abs(command.linear_y) > std::abs(command.linear_x) ?
      std::abs(active_error.lateral_m) :
      std::abs(active_error.forward_m);
    if (std::isfinite(remaining)) {
      projection.translation_m =
        std::min(configured_translation_m, remaining);
    }
  }

  if (std::isfinite(command.angular_z) &&
    std::abs(command.angular_z) > 1.0e-9 &&
    std::isfinite(active_error.yaw_rad))
  {
    projection.rotation_rad = std::min(
      configured_rotation_rad, std::abs(active_error.yaw_rad));
  }
  return projection;
}

inline std::vector<ElevatorScopedRouteSegment>
make_elevator_scoped_route_segments(
  const std::vector<ElevatorScopedPose> & poses,
  const double translation_epsilon_m = 1.0e-6,
  const double yaw_epsilon_rad = 1.0e-6)
{
  std::vector<ElevatorScopedRouteSegment> segments;
  if (poses.empty() || !std::isfinite(translation_epsilon_m) ||
    !std::isfinite(yaw_epsilon_rad) || translation_epsilon_m < 0.0 ||
    yaw_epsilon_rad < 0.0)
  {
    return segments;
  }

  const auto append = [&segments](const ElevatorScopedRoutePhase phase,
      const std::size_t end_index,
      const int direction) {
      if (!segments.empty() && segments.back().phase == phase &&
        segments.back().direction == direction)
      {
        segments.back().end_index = end_index;
        return;
      }
      segments.push_back({phase, end_index, direction});
    };

  for (std::size_t index = 1U; index < poses.size(); ++index) {
    const auto & from = poses[index - 1U];
    const auto & to = poses[index];
    if (!elevator_scoped_pose_is_finite(from) ||
      !elevator_scoped_pose_is_finite(to))
    {
      segments.clear();
      return segments;
    }

    const double dx = to.x - from.x;
    const double dy = to.y - from.y;
    const double distance = std::hypot(dx, dy);
    const double yaw_delta = normalize_elevator_scoped_angle(to.yaw - from.yaw);
    if (distance <= translation_epsilon_m) {
      if (std::abs(yaw_delta) > yaw_epsilon_rad) {
        append(ElevatorScopedRoutePhase::kYaw, index, yaw_delta < 0.0 ? -1 : 1);
      }
      continue;
    }

    const double forward = std::cos(to.yaw) * dx + std::sin(to.yaw) * dy;
    const double lateral = -std::sin(to.yaw) * dx + std::cos(to.yaw) * dy;
    if (std::abs(lateral) > std::abs(forward)) {
      append(ElevatorScopedRoutePhase::kLateral, index, lateral < 0.0 ? -1 : 1);
    } else {
      append(
        forward < 0.0 ? ElevatorScopedRoutePhase::kReverse :
        ElevatorScopedRoutePhase::kForward,
        index, forward < 0.0 ? -1 : 1);
    }
  }

  if (segments.empty()) {
    segments.push_back({ElevatorScopedRoutePhase::kPose, poses.size() - 1U, 0});
  } else if (segments.back().end_index + 1U < poses.size()) {
    segments.back().end_index = poses.size() - 1U;
  }
  if (segments.back().phase != ElevatorScopedRoutePhase::kPose) {
    segments.push_back({ElevatorScopedRoutePhase::kPose, poses.size() - 1U, 0});
  }
  return segments;
}

inline bool ensure_elevator_scoped_route_boundary(
  std::vector<ElevatorScopedRouteSegment> & segments,
  const std::size_t boundary_index)
{
  if (segments.empty()) {
    return false;
  }
  if (boundary_index == 0U) {
    return true;
  }

  std::size_t previous_end_index = 0U;
  for (std::size_t index = 0U; index < segments.size(); ++index) {
    if (segments[index].end_index == boundary_index) {
      return true;
    }
    if (previous_end_index < boundary_index &&
      boundary_index < segments[index].end_index)
    {
      auto suffix = segments[index];
      segments[index].end_index = boundary_index;
      segments.insert(
        segments.begin() + static_cast<std::ptrdiff_t>(index + 1U), suffix);
      return true;
    }
    previous_end_index = segments[index].end_index;
  }
  return false;
}

} // namespace robot_nav_config
