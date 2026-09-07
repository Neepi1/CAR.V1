#include <gtest/gtest.h>

#include "robot_nav_config/elevator_scoped_plan_update.hpp"

namespace robot_nav_config {

namespace {

ElevatorScopedGoalSignature goal(const double x = 1.0, const double y = 2.0,
                                 const double yaw = 0.5) {
  return {"map", x, y, yaw};
}

std::vector<ElevatorScopedRouteSegment>
route(const ElevatorScopedRoutePhase phase, const int direction = 1) {
  return {{phase, 1U, direction}, {ElevatorScopedRoutePhase::kPose, 2U, 0}};
}

} // namespace

TEST(ElevatorScopedPlanUpdate, InitialPlanResetsForNewGoal) {
  const auto decision =
      decide_elevator_scoped_plan_update(std::nullopt, {}, 0U, false, goal(),
                                         route(ElevatorScopedRoutePhase::kYaw));

  EXPECT_EQ(decision.action, ElevatorScopedPlanUpdateAction::kResetForNewGoal);
}

TEST(ElevatorScopedPlanUpdate, ChangedGoalResetsControllerState) {
  const auto decision = decide_elevator_scoped_plan_update(
      goal(), route(ElevatorScopedRoutePhase::kYaw), 0U, true,
      goal(1.02, 2.0, 0.5), route(ElevatorScopedRoutePhase::kYaw));

  EXPECT_EQ(decision.action, ElevatorScopedPlanUpdateAction::kResetForNewGoal);
}

TEST(ElevatorScopedPlanUpdate, CompatibleSameGoalPlanHotSwapsWithoutReset) {
  const auto decision = decide_elevator_scoped_plan_update(
      goal(), route(ElevatorScopedRoutePhase::kYaw), 0U, true, goal(),
      route(ElevatorScopedRoutePhase::kYaw));

  EXPECT_EQ(decision.action,
            ElevatorScopedPlanUpdateAction::kHotSwapPreserveMotion);
  EXPECT_EQ(decision.new_segment_index, 0U);
}

TEST(ElevatorScopedPlanUpdate, IncompatibleSameGoalRefreshKeepsActivePlan) {
  const auto decision = decide_elevator_scoped_plan_update(
      goal(), route(ElevatorScopedRoutePhase::kYaw), 0U, true, goal(),
      route(ElevatorScopedRoutePhase::kLateral));

  EXPECT_EQ(decision.action, ElevatorScopedPlanUpdateAction::kKeepActivePlan);
}

TEST(ElevatorScopedPlanUpdate, GoalYawWrapDoesNotCreateANewGoal) {
  const ElevatorScopedGoalSignature old_goal{"map", 1.0, 2.0, 3.14};
  const ElevatorScopedGoalSignature new_goal{"map", 1.0, 2.0, -3.139};
  const auto decision = decide_elevator_scoped_plan_update(
      old_goal, route(ElevatorScopedRoutePhase::kYaw), 0U, true, new_goal,
      route(ElevatorScopedRoutePhase::kYaw));

  EXPECT_EQ(decision.action,
            ElevatorScopedPlanUpdateAction::kHotSwapPreserveMotion);
}

TEST(ElevatorScopedPlanUpdate, CompletedSameGoalStartsAFreshAction) {
  const auto decision = decide_elevator_scoped_plan_update(
      goal(), route(ElevatorScopedRoutePhase::kPose), 1U, false, goal(),
      route(ElevatorScopedRoutePhase::kYaw));

  EXPECT_EQ(decision.action, ElevatorScopedPlanUpdateAction::kResetForNewGoal);
}

} // namespace robot_nav_config
