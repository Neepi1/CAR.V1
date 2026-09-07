#pragma once

#include <cstddef>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "robot_nav_config/elevator_scoped_search.hpp"

namespace robot_nav_config
{

struct ElevatorScopedIndexedPose
{
  std::size_t path_index{0U};
  ElevatorScopedPose pose;
};

struct ElevatorScopedLocalRepairParameters
{
  double minimum_rejoin_distance_m{0.35};
  double maximum_rejoin_distance_m{1.20};
  double rejoin_spacing_m{0.20};
  std::size_t maximum_rejoin_candidates{6U};
};

enum class ElevatorScopedLocalRepairStatus
{
  kSuccess,
  kInvalidRequest,
  kNoRejoinCandidate,
  kSearchFailed,
  kTimeLimit,
};

struct ElevatorScopedLocalRepairResult
{
  ElevatorScopedLocalRepairStatus status{
    ElevatorScopedLocalRepairStatus::kInvalidRequest};
  ElevatorScopedSearchResult search;
  std::size_t rejoin_path_index{0U};
  std::size_t nearest_path_index{0U};
  std::size_t candidates_considered{0U};

  bool succeeded() const noexcept
  {
    return status == ElevatorScopedLocalRepairStatus::kSuccess &&
           search.succeeded();
  }
};

ElevatorScopedLocalRepairResult search_elevator_scoped_local_repair(
  nav2_costmap_2d::Costmap2D & costmap,
  const std::vector<geometry_msgs::msg::Point> & footprint,
  const ElevatorScopedPose & start,
  const std::vector<ElevatorScopedIndexedPose> & remaining_path,
  const ElevatorScopedSearchParameters & search_parameters,
  const ElevatorScopedLocalRepairParameters & repair_parameters =
  ElevatorScopedLocalRepairParameters{});

const char * elevator_scoped_local_repair_status_name(
  ElevatorScopedLocalRepairStatus status) noexcept;

}  // namespace robot_nav_config
