#pragma once

#include <cmath>
#include <cstddef>
#include <optional>
#include <string>
#include <vector>

#include "robot_nav_config/elevator_scoped_route.hpp"

namespace robot_nav_config {

struct ElevatorScopedGoalSignature {
  std::string frame_id;
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

enum class ElevatorScopedPlanUpdateAction {
  kRejectInvalidPlan,
  kResetForNewGoal,
  kHotSwapPreserveMotion,
  kKeepActivePlan,
};

struct ElevatorScopedPlanUpdateDecision {
  ElevatorScopedPlanUpdateAction action{
      ElevatorScopedPlanUpdateAction::kRejectInvalidPlan};
  std::size_t new_segment_index{0U};
};

inline bool elevator_scoped_goal_is_finite(
    const ElevatorScopedGoalSignature &goal) noexcept {
  return !goal.frame_id.empty() && std::isfinite(goal.x) &&
         std::isfinite(goal.y) && std::isfinite(goal.yaw);
}

inline bool
elevator_scoped_goals_match(const ElevatorScopedGoalSignature &lhs,
                            const ElevatorScopedGoalSignature &rhs,
                            const double xy_tolerance_m = 0.01,
                            const double yaw_tolerance_rad = 0.01) noexcept {
  if (!elevator_scoped_goal_is_finite(lhs) ||
      !elevator_scoped_goal_is_finite(rhs) || lhs.frame_id != rhs.frame_id ||
      !std::isfinite(xy_tolerance_m) || !std::isfinite(yaw_tolerance_rad) ||
      xy_tolerance_m < 0.0 || yaw_tolerance_rad < 0.0) {
    return false;
  }
  const double yaw_delta =
      std::atan2(std::sin(rhs.yaw - lhs.yaw), std::cos(rhs.yaw - lhs.yaw));
  return std::hypot(rhs.x - lhs.x, rhs.y - lhs.y) <= xy_tolerance_m &&
         std::abs(yaw_delta) <= yaw_tolerance_rad;
}

inline bool elevator_scoped_segments_are_motion_compatible(
    const ElevatorScopedRouteSegment &active,
    const ElevatorScopedRouteSegment &replacement) noexcept {
  if (active.phase != replacement.phase) {
    return false;
  }
  return active.phase == ElevatorScopedRoutePhase::kPose ||
         active.direction == replacement.direction;
}

inline ElevatorScopedPlanUpdateDecision decide_elevator_scoped_plan_update(
    const std::optional<ElevatorScopedGoalSignature> &active_goal,
    const std::vector<ElevatorScopedRouteSegment> &active_segments,
    const std::size_t active_segment_index, const bool active_execution,
    const ElevatorScopedGoalSignature &replacement_goal,
    const std::vector<ElevatorScopedRouteSegment> &replacement_segments) {
  if (!elevator_scoped_goal_is_finite(replacement_goal) ||
      replacement_segments.empty()) {
    return {ElevatorScopedPlanUpdateAction::kRejectInvalidPlan, 0U};
  }
  if (!active_execution || !active_goal ||
      !elevator_scoped_goals_match(*active_goal, replacement_goal)) {
    return {ElevatorScopedPlanUpdateAction::kResetForNewGoal, 0U};
  }
  if (active_segments.empty() ||
      active_segment_index >= active_segments.size()) {
    return {ElevatorScopedPlanUpdateAction::kKeepActivePlan,
            active_segment_index};
  }
  if (elevator_scoped_segments_are_motion_compatible(
          active_segments[active_segment_index],
          replacement_segments.front())) {
    // A planner refresh starts at the robot's current pose.  Segment zero is
    // therefore the remaining suffix of the active semantic motion, not a
    // replay of already completed historical segments.
    return {ElevatorScopedPlanUpdateAction::kHotSwapPreserveMotion, 0U};
  }
  return {ElevatorScopedPlanUpdateAction::kKeepActivePlan,
          active_segment_index};
}

} // namespace robot_nav_config
