#pragma once

#include <cstddef>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "robot_nav_config/elevator_scoped_clearance.hpp"
#include "robot_nav_config/elevator_scoped_path.hpp"

namespace robot_nav_config
{

enum class ElevatorScopedSearchStatus
{
  kSuccess,
  kInvalidRequest,
  kStartBlocked,
  kGoalBlocked,
  kNoPath,
  kExpansionLimit,
  kTimeLimit,
};

enum class ElevatorScopedRoutePolicy
{
  kPreserveIngressHeading,
  kPreferClearStartupSpin,
  // Source-floor hall-call -> landing staging: first adopt the commissioned
  // landing yaw, then translate laterally, then close the remaining
  // longitudinal error.  This policy is intentionally separate from the
  // reverse-docking sequence used after the door opens.
  kReverseEntryStagingSequence,
  // Commissioned four-point elevator sequence: first adopt the goal yaw,
  // then translate longitudinally, then translate laterally.  A blocked
  // maneuver is reported instead of being replaced by an arbitrary detour.
  kReverseDockingSequence,
};

struct ElevatorScopedSearchParameters
{
  ElevatorScopedPathParameters sampling;
  ElevatorScopedRoutePolicy route_policy{
    ElevatorScopedRoutePolicy::kPreserveIngressHeading};
  double search_grid_step_m{0.05};
  double search_radius_m{2.5};
  double goal_connect_distance_m{0.12};
  std::size_t maximum_expansions{60000U};
  double forward_cost{1.0};
  double reverse_cost{1.35};
  double lateral_cost{1.15};
  double mode_switch_cost{0.20};
  double soft_cost_weight{0.35};
  double clearance_preference_cost_ratio{1.10};
  double maximum_search_time_sec{0.25};
};

struct ElevatorScopedSearchResult
{
  ElevatorScopedSearchStatus status{
    ElevatorScopedSearchStatus::kInvalidRequest};
  ElevatorScopedPath path;
  bool used_detour{false};
  bool used_startup_rotation_escape{false};
  std::size_t expanded_nodes{0U};
  std::size_t mode_switches{0U};
  double path_length_m{0.0};
  double direct_path_objective{0.0};
  double searched_path_objective{0.0};
  double selected_path_objective{0.0};
  double direct_soft_cost_exposure{0.0};
  double searched_soft_cost_exposure{0.0};
  ElevatorScopedClearanceResult direct_path_block;

  bool succeeded() const noexcept
  {
    return status == ElevatorScopedSearchStatus::kSuccess;
  }
};

ElevatorScopedSearchResult search_elevator_scoped_path(
  nav2_costmap_2d::Costmap2D & costmap,
  const std::vector<geometry_msgs::msg::Point> & footprint,
  const ElevatorScopedPose & start,
  const ElevatorScopedPose & goal,
  const ElevatorScopedSearchParameters & parameters =
  ElevatorScopedSearchParameters{});

const char * elevator_scoped_search_status_name(
  ElevatorScopedSearchStatus status) noexcept;

}  // namespace robot_nav_config
