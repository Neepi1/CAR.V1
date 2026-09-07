#include <cmath>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "geometry_msgs/msg/point.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "robot_nav_config/elevator_scoped_clearance.hpp"
#include "robot_nav_config/elevator_scoped_local_repair.hpp"

namespace
{

using robot_nav_config::ElevatorScopedIndexedPose;
using robot_nav_config::ElevatorScopedLocalRepairParameters;
using robot_nav_config::ElevatorScopedLocalRepairStatus;
using robot_nav_config::ElevatorScopedMotionPhase;
using robot_nav_config::ElevatorScopedPose;
using robot_nav_config::ElevatorScopedSearchParameters;
using robot_nav_config::evaluate_elevator_scoped_clearance;
using robot_nav_config::search_elevator_scoped_local_repair;

std::vector<geometry_msgs::msg::Point> padded_ranger_footprint()
{
  std::vector<geometry_msgs::msg::Point> footprint;
  for (const auto & coordinate : {
      std::pair<double, double>{0.39, 0.28},
      std::pair<double, double>{0.39, -0.28},
      std::pair<double, double>{-0.39, -0.28},
      std::pair<double, double>{-0.39, 0.28}})
  {
    geometry_msgs::msg::Point point;
    point.x = coordinate.first;
    point.y = coordinate.second;
    footprint.push_back(point);
  }
  return footprint;
}

std::vector<ElevatorScopedIndexedPose> straight_remaining_path()
{
  std::vector<ElevatorScopedIndexedPose> path;
  for (std::size_t index = 0U; index <= 32U; ++index) {
    path.push_back(
      {
        index,
        ElevatorScopedPose{0.05 * static_cast<double>(index), 0.0, 0.0},
      });
  }
  return path;
}

ElevatorScopedSearchParameters local_search_parameters()
{
  ElevatorScopedSearchParameters parameters;
  parameters.sampling.max_distance_m = 2.5;
  parameters.sampling.translation_step_m = 0.025;
  parameters.sampling.rotation_step_rad = 0.05;
  parameters.search_grid_step_m = 0.05;
  parameters.search_radius_m = 1.5;
  parameters.goal_connect_distance_m = 0.10;
  parameters.maximum_expansions = 20000U;
  parameters.maximum_search_time_sec = 0.10;
  return parameters;
}

ElevatorScopedLocalRepairParameters repair_parameters()
{
  ElevatorScopedLocalRepairParameters parameters;
  parameters.minimum_rejoin_distance_m = 0.55;
  parameters.maximum_rejoin_distance_m = 1.10;
  parameters.rejoin_spacing_m = 0.15;
  parameters.maximum_rejoin_candidates = 5U;
  return parameters;
}

void add_front_corner_obstacle(
  nav2_costmap_2d::Costmap2D & costmap, const double lateral_y)
{
  unsigned int cell_x = 0U;
  unsigned int cell_y = 0U;
  ASSERT_TRUE(costmap.worldToMap(0.47, lateral_y, cell_x, cell_y));
  costmap.setCost(cell_x, cell_y, nav2_costmap_2d::LETHAL_OBSTACLE);
}

double first_translation_lateral_delta(
  const robot_nav_config::ElevatorScopedPath & path)
{
  if (path.samples.empty()) {
    return 0.0;
  }
  const auto start = path.samples.front().pose;
  for (const auto & sample : path.samples) {
    if (sample.phase != ElevatorScopedMotionPhase::kYaw &&
      std::hypot(sample.pose.x - start.x, sample.pose.y - start.y) > 1.0e-6)
    {
      return sample.pose.y - start.y;
    }
  }
  return 0.0;
}

std::size_t lateral_direction_changes(
  const robot_nav_config::ElevatorScopedPath & path)
{
  int previous_sign = 0;
  std::size_t changes = 0U;
  for (std::size_t index = 1U; index < path.samples.size(); ++index) {
    if (path.samples[index].phase != ElevatorScopedMotionPhase::kLateral) {
      continue;
    }
    const double delta_y =
      path.samples[index].pose.y - path.samples[index - 1U].pose.y;
    const int sign = delta_y > 1.0e-6 ? 1 : (delta_y < -1.0e-6 ? -1 : 0);
    if (sign == 0) {
      continue;
    }
    if (previous_sign != 0 && sign != previous_sign) {
      ++changes;
    }
    previous_sign = sign;
  }
  return changes;
}

void expect_clear_repair(
  nav2_costmap_2d::Costmap2D & costmap,
  const double obstacle_y,
  const double expected_lateral_sign)
{
  const auto footprint = padded_ranger_footprint();
  const ElevatorScopedPose start{0.0, 0.0, 0.0};
  add_front_corner_obstacle(costmap, obstacle_y);

  ASSERT_TRUE(
    evaluate_elevator_scoped_clearance(
      costmap, footprint, start.x, start.y, start.yaw).is_clear());
  ASSERT_FALSE(
    evaluate_elevator_scoped_clearance(
      costmap, footprint, 0.15, 0.0, 0.0).is_clear());

  const auto result = search_elevator_scoped_local_repair(
    costmap, footprint, start, straight_remaining_path(),
    local_search_parameters(), repair_parameters());

  ASSERT_TRUE(result.succeeded())
    << "status=" << robot_nav_config::elevator_scoped_local_repair_status_name(
    result.status)
    << " search=" << robot_nav_config::elevator_scoped_search_status_name(
    result.search.status)
    << " candidates=" << result.candidates_considered;
  EXPECT_GE(result.rejoin_path_index, 11U);
  EXPECT_LE(result.rejoin_path_index, 22U);
  ASSERT_FALSE(result.search.path.samples.empty());
  EXPECT_EQ(
    std::copysign(1.0, first_translation_lateral_delta(result.search.path)),
    expected_lateral_sign);
  EXPECT_LE(lateral_direction_changes(result.search.path), 1U);
  for (const auto & sample : result.search.path.samples) {
    EXPECT_TRUE(
      evaluate_elevator_scoped_clearance(
        costmap, footprint,
        sample.pose.x, sample.pose.y, sample.pose.yaw).is_clear());
  }
}

TEST(ElevatorScopedLocalRepair, FrontLeftBlockUsesRightSideSlipAndRejoins)
{
  nav2_costmap_2d::Costmap2D costmap(
    120U, 120U, 0.05, -2.0, -3.0, nav2_costmap_2d::FREE_SPACE);
  expect_clear_repair(costmap, 0.22, -1.0);
}

TEST(ElevatorScopedLocalRepair, FrontRightBlockUsesLeftSideSlipAndRejoins)
{
  nav2_costmap_2d::Costmap2D costmap(
    120U, 120U, 0.05, -2.0, -3.0, nav2_costmap_2d::FREE_SPACE);
  expect_clear_repair(costmap, -0.22, 1.0);
}

TEST(ElevatorScopedLocalRepair, SealedCorridorFailsClosed)
{
  nav2_costmap_2d::Costmap2D costmap(
    120U, 120U, 0.05, -2.0, -3.0, nav2_costmap_2d::FREE_SPACE);
  unsigned int wall_x = 0U;
  unsigned int unused_y = 0U;
  ASSERT_TRUE(costmap.worldToMap(0.47, 0.0, wall_x, unused_y));
  for (unsigned int cell_y = 0U; cell_y < costmap.getSizeInCellsY(); ++cell_y) {
    costmap.setCost(wall_x, cell_y, nav2_costmap_2d::LETHAL_OBSTACLE);
  }

  const auto footprint = padded_ranger_footprint();
  const ElevatorScopedPose start{0.0, 0.0, 0.0};
  ASSERT_TRUE(
    evaluate_elevator_scoped_clearance(
      costmap, footprint, start.x, start.y, start.yaw).is_clear());

  const auto result = search_elevator_scoped_local_repair(
    costmap, footprint, start, straight_remaining_path(),
    local_search_parameters(), repair_parameters());

  EXPECT_FALSE(result.succeeded());
  EXPECT_NE(result.status, ElevatorScopedLocalRepairStatus::kSuccess);
  EXPECT_GE(result.candidates_considered, 1U);
}

TEST(ElevatorScopedLocalRepair, ShortTerminalRemainderDoesNotCreateMicroRejoin)
{
  nav2_costmap_2d::Costmap2D costmap(
    120U, 120U, 0.05, -2.0, -3.0, nav2_costmap_2d::FREE_SPACE);
  const auto footprint = padded_ranger_footprint();
  const ElevatorScopedPose start{0.0, 0.0, 0.0};
  const std::vector<ElevatorScopedIndexedPose> short_remaining_path{
    {40U, ElevatorScopedPose{0.0, 0.0, 0.0}},
    {41U, ElevatorScopedPose{0.025, 0.0, 0.0}},
  };

  const auto result = search_elevator_scoped_local_repair(
    costmap, footprint, start, short_remaining_path,
    local_search_parameters(), repair_parameters());

  EXPECT_FALSE(result.succeeded());
  EXPECT_EQ(result.status, ElevatorScopedLocalRepairStatus::kNoRejoinCandidate);
  EXPECT_EQ(result.candidates_considered, 0U);
}

}  // namespace
