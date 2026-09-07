#pragma once

#include <cmath>
#include <cstddef>
#include <limits>
#include <optional>
#include <vector>

namespace robot_nav_config
{

enum class ElevatorScopedMotionPhase
{
  kYaw,
  kLateral,
  kForward,
  kReverse,
};

struct ElevatorScopedPose
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

struct ElevatorScopedPathSample
{
  ElevatorScopedMotionPhase phase{ElevatorScopedMotionPhase::kYaw};
  ElevatorScopedPose pose;
};

struct ElevatorScopedPath
{
  std::vector<ElevatorScopedPathSample> samples;
  double lateral_m{0.0};
  double forward_m{0.0};
};

struct ElevatorScopedPathParameters
{
  double max_distance_m{2.5};
  double translation_step_m{0.025};
  double rotation_step_rad{0.05};
  std::size_t maximum_samples{20000U};
};

inline double normalize_elevator_scoped_angle(const double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

inline bool elevator_scoped_pose_is_finite(const ElevatorScopedPose & pose)
{
  return std::isfinite(pose.x) &&
         std::isfinite(pose.y) &&
         std::isfinite(pose.yaw);
}

inline std::optional<ElevatorScopedPath> make_elevator_scoped_path(
  const ElevatorScopedPose & start,
  const ElevatorScopedPose & goal,
  const ElevatorScopedPathParameters & parameters =
  ElevatorScopedPathParameters{})
{
  if (!elevator_scoped_pose_is_finite(start) ||
    !elevator_scoped_pose_is_finite(goal) ||
    !std::isfinite(parameters.max_distance_m) ||
    !std::isfinite(parameters.translation_step_m) ||
    !std::isfinite(parameters.rotation_step_rad) ||
    parameters.max_distance_m <= 0.0 ||
    parameters.translation_step_m <= 0.0 ||
    parameters.rotation_step_rad <= 0.0 ||
    parameters.maximum_samples == 0U)
  {
    return std::nullopt;
  }

  const double dx = goal.x - start.x;
  const double dy = goal.y - start.y;
  const double distance_m = std::hypot(dx, dy);
  if (!std::isfinite(distance_m) || distance_m > parameters.max_distance_m) {
    return std::nullopt;
  }

  const double goal_cos = std::cos(goal.yaw);
  const double goal_sin = std::sin(goal.yaw);
  const double forward_m = goal_cos * dx + goal_sin * dy;
  const double lateral_m = -goal_sin * dx + goal_cos * dy;
  const double yaw_delta =
    normalize_elevator_scoped_angle(goal.yaw - start.yaw);
  if (!std::isfinite(forward_m) ||
    !std::isfinite(lateral_m) ||
    !std::isfinite(yaw_delta))
  {
    return std::nullopt;
  }

  const auto segment_count =
    [](const double magnitude, const double step) -> std::size_t {
      if (magnitude <= std::numeric_limits<double>::epsilon()) {
        return 0U;
      }
      return static_cast<std::size_t>(std::ceil(magnitude / step));
    };
  const std::size_t yaw_segments =
    segment_count(std::abs(yaw_delta), parameters.rotation_step_rad);
  const std::size_t lateral_segments =
    segment_count(std::abs(lateral_m), parameters.translation_step_m);
  const std::size_t forward_segments =
    segment_count(std::abs(forward_m), parameters.translation_step_m);
  if (yaw_segments > parameters.maximum_samples ||
    lateral_segments > parameters.maximum_samples ||
    forward_segments > parameters.maximum_samples ||
    yaw_segments + lateral_segments + forward_segments + 1U >
    parameters.maximum_samples)
  {
    return std::nullopt;
  }

  ElevatorScopedPath path;
  path.lateral_m = lateral_m;
  path.forward_m = forward_m;
  path.samples.reserve(
    yaw_segments + lateral_segments + forward_segments + 1U);

  // Include the exact start pose so the planner collision-checks the current
  // footprint before authorizing any scoped motion.
  path.samples.push_back({
    ElevatorScopedMotionPhase::kYaw,
    ElevatorScopedPose{start.x, start.y, start.yaw},
  });
  for (std::size_t index = 1U; index <= yaw_segments; ++index) {
    const double ratio =
      static_cast<double>(index) / static_cast<double>(yaw_segments);
    path.samples.push_back({
      ElevatorScopedMotionPhase::kYaw,
      ElevatorScopedPose{
        start.x,
        start.y,
        normalize_elevator_scoped_angle(start.yaw + ratio * yaw_delta),
      },
    });
  }

  const double lateral_x = -goal_sin * lateral_m;
  const double lateral_y = goal_cos * lateral_m;
  for (std::size_t index = 1U; index <= lateral_segments; ++index) {
    const double ratio =
      static_cast<double>(index) / static_cast<double>(lateral_segments);
    path.samples.push_back({
      ElevatorScopedMotionPhase::kLateral,
      ElevatorScopedPose{
        start.x + ratio * lateral_x,
        start.y + ratio * lateral_y,
        goal.yaw,
      },
    });
  }

  const double intermediate_x = start.x + lateral_x;
  const double intermediate_y = start.y + lateral_y;
  for (std::size_t index = 1U; index <= forward_segments; ++index) {
    const double ratio =
      static_cast<double>(index) / static_cast<double>(forward_segments);
    path.samples.push_back({
      forward_m < 0.0 ?
      ElevatorScopedMotionPhase::kReverse :
      ElevatorScopedMotionPhase::kForward,
      ElevatorScopedPose{
        intermediate_x + ratio * goal_cos * forward_m,
        intermediate_y + ratio * goal_sin * forward_m,
        goal.yaw,
      },
    });
  }

  path.samples.back().pose = goal;
  return path;
}

}  // namespace robot_nav_config
