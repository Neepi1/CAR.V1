#include <chrono>
#include <cstdint>
#include <future>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "geometry_msgs/msg/point.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "robot_nav_config/elevator_scoped_clearance.hpp"

namespace {

using robot_nav_config::ElevatorScopedClearanceStatus;
using robot_nav_config::evaluate_elevator_scoped_clearance;
using robot_nav_config::evaluate_elevator_scoped_path_clearance;
using robot_nav_config::evaluate_elevator_scoped_path_clearance_locked;

constexpr double kPi = 3.14159265358979323846;

std::vector<geometry_msgs::msg::Point> padded_ranger_footprint() {
  std::vector<geometry_msgs::msg::Point> footprint;
  for (const auto &coordinate : {std::pair<double, double>{0.39, 0.28},
                                 std::pair<double, double>{0.39, -0.28},
                                 std::pair<double, double>{-0.39, -0.28},
                                 std::pair<double, double>{-0.39, 0.28}}) {
    geometry_msgs::msg::Point point;
    point.x = coordinate.first;
    point.y = coordinate.second;
    footprint.push_back(point);
  }
  return footprint;
}

void set_world_cost(nav2_costmap_2d::Costmap2D &costmap, const double x,
                    const double y, const std::uint8_t cost) {
  unsigned int cell_x = 0U;
  unsigned int cell_y = 0U;
  ASSERT_TRUE(costmap.worldToMap(x, y, cell_x, cell_y));
  costmap.setCost(cell_x, cell_y, cost);
}

TEST(ElevatorScopedClearance,
     SoftAndInscribedInflationDoNotBlockAFreeFootprint) {
  nav2_costmap_2d::Costmap2D costmap(80U, 80U, 0.05, -2.0, -2.0,
                                     nav2_costmap_2d::FREE_SPACE);
  const auto footprint = padded_ranger_footprint();

  set_world_cost(costmap, 0.0, 0.0,
                 nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
  set_world_cost(costmap, 0.35, 0.20, 200U);

  const auto result =
      evaluate_elevator_scoped_clearance(costmap, footprint, 0.0, 0.0, 0.0);

  EXPECT_TRUE(result.is_clear());
  EXPECT_EQ(result.status, ElevatorScopedClearanceStatus::kClear);
}

TEST(ElevatorScopedClearance,
     LethalCellInsideFilledFootprintBlocksTheCorridor) {
  nav2_costmap_2d::Costmap2D costmap(80U, 80U, 0.05, -2.0, -2.0,
                                     nav2_costmap_2d::FREE_SPACE);
  const auto footprint = padded_ranger_footprint();

  // This cell is inside the body but away from the rectangle perimeter. A
  // perimeter-only footprint checker would not inspect it.
  set_world_cost(costmap, 0.15, 0.10, nav2_costmap_2d::LETHAL_OBSTACLE);

  const auto result =
      evaluate_elevator_scoped_clearance(costmap, footprint, 0.0, 0.0, 0.0);

  EXPECT_FALSE(result.is_clear());
  EXPECT_EQ(result.status, ElevatorScopedClearanceStatus::kLethalObstacle);
  EXPECT_EQ(result.cost, nav2_costmap_2d::LETHAL_OBSTACLE);
}

TEST(ElevatorScopedClearance, UnknownCellInsideFilledFootprintFailsClosed) {
  nav2_costmap_2d::Costmap2D costmap(80U, 80U, 0.05, -2.0, -2.0,
                                     nav2_costmap_2d::FREE_SPACE);
  const auto footprint = padded_ranger_footprint();

  set_world_cost(costmap, -0.10, 0.05, nav2_costmap_2d::NO_INFORMATION);

  const auto result =
      evaluate_elevator_scoped_clearance(costmap, footprint, 0.0, 0.0, 0.0);

  EXPECT_FALSE(result.is_clear());
  EXPECT_EQ(result.status, ElevatorScopedClearanceStatus::kUnknownSpace);
  EXPECT_EQ(result.cost, nav2_costmap_2d::NO_INFORMATION);
}

TEST(ElevatorScopedClearance, LethalCellOutsideFootprintDoesNotBlock) {
  nav2_costmap_2d::Costmap2D costmap(80U, 80U, 0.05, -2.0, -2.0,
                                     nav2_costmap_2d::FREE_SPACE);
  const auto footprint = padded_ranger_footprint();

  // The cell is inside the one-cell-expanded search box, but its square is
  // still separated from the footprint's +X edge at 0.39 m.
  set_world_cost(costmap, 0.42, 0.0, nav2_costmap_2d::LETHAL_OBSTACLE);

  const auto result =
      evaluate_elevator_scoped_clearance(costmap, footprint, 0.0, 0.0, 0.0);

  EXPECT_TRUE(result.is_clear());
  EXPECT_EQ(result.status, ElevatorScopedClearanceStatus::kClear);
}

TEST(ElevatorScopedClearance,
     RotatedFilledFootprintStillDetectsInteriorObstacle) {
  nav2_costmap_2d::Costmap2D costmap(80U, 80U, 0.05, -2.0, -2.0,
                                     nav2_costmap_2d::FREE_SPACE);
  const auto footprint = padded_ranger_footprint();

  set_world_cost(costmap, 0.0, 0.20, nav2_costmap_2d::LETHAL_OBSTACLE);

  const auto result = evaluate_elevator_scoped_clearance(costmap, footprint,
                                                         0.0, 0.0, kPi / 4.0);

  EXPECT_FALSE(result.is_clear());
  EXPECT_EQ(result.status, ElevatorScopedClearanceStatus::kLethalObstacle);
}

TEST(ElevatorScopedClearance, FootprintLeavingCostmapFailsClosed) {
  nav2_costmap_2d::Costmap2D costmap(80U, 80U, 0.05, -2.0, -2.0,
                                     nav2_costmap_2d::FREE_SPACE);
  const auto footprint = padded_ranger_footprint();

  const auto result =
      evaluate_elevator_scoped_clearance(costmap, footprint, 1.70, 0.0, 0.0);

  EXPECT_FALSE(result.is_clear());
  EXPECT_EQ(result.status, ElevatorScopedClearanceStatus::kOutOfMap);
}

TEST(ElevatorScopedClearance, SweptRouteReportsTheFirstBlockingSample) {
  nav2_costmap_2d::Costmap2D costmap(80U, 80U, 0.05, -2.0, -2.0,
                                     nav2_costmap_2d::FREE_SPACE);
  const auto footprint = padded_ranger_footprint();
  set_world_cost(costmap, 0.50, 0.0, nav2_costmap_2d::LETHAL_OBSTACLE);

  std::vector<robot_nav_config::ElevatorScopedPathSample> samples;
  for (const double x : {0.0, 0.05, 0.10, 0.15}) {
    samples.push_back(
        {robot_nav_config::ElevatorScopedMotionPhase::kForward, {x, 0.0, 0.0}});
  }
  const auto result =
      evaluate_elevator_scoped_path_clearance(costmap, footprint, samples);

  EXPECT_FALSE(result.is_clear());
  EXPECT_EQ(result.clearance.status,
            ElevatorScopedClearanceStatus::kLethalObstacle);
  // At x=0.15 m the padded front edge first intersects the 0.50 m cell.
  EXPECT_EQ(result.sample_index, 3U);
  EXPECT_EQ(result.clearance.cost, nav2_costmap_2d::LETHAL_OBSTACLE);
}

TEST(ElevatorScopedClearance, SweptRouteUsesTheSameSoftCostPolicy) {
  nav2_costmap_2d::Costmap2D costmap(80U, 80U, 0.05, -2.0, -2.0,
                                     nav2_costmap_2d::FREE_SPACE);
  const auto footprint = padded_ranger_footprint();
  set_world_cost(costmap, 0.50, 0.0,
                 nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE);

  const std::vector<robot_nav_config::ElevatorScopedPathSample> samples{
      {robot_nav_config::ElevatorScopedMotionPhase::kForward, {0.0, 0.0, 0.0}},
      {robot_nav_config::ElevatorScopedMotionPhase::kForward, {0.10, 0.0, 0.0}},
  };
  const auto result =
      evaluate_elevator_scoped_path_clearance(costmap, footprint, samples);

  EXPECT_TRUE(result.is_clear());
  EXPECT_EQ(result.clearance.status, ElevatorScopedClearanceStatus::kClear);
}

TEST(ElevatorScopedClearance,
     CommandSnapshotWaitsForAConcurrentCostmapUpdateInsteadOfReportingBusy) {
  nav2_costmap_2d::Costmap2D costmap(80U, 80U, 0.05, -2.0, -2.0,
                                     nav2_costmap_2d::FREE_SPACE);
  const auto footprint = padded_ranger_footprint();
  const std::vector<robot_nav_config::ElevatorScopedPathSample> samples{
      {robot_nav_config::ElevatorScopedMotionPhase::kForward, {0.0, 0.0, 0.0}},
      {robot_nav_config::ElevatorScopedMotionPhase::kForward, {0.05, 0.0, 0.0}},
  };

  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> update_lock(
      *costmap.getMutex());
  auto command_check = std::async(std::launch::async, [&]() {
    return evaluate_elevator_scoped_path_clearance_locked(
        costmap, footprint, samples);
  });

  EXPECT_EQ(command_check.wait_for(std::chrono::milliseconds(20)),
            std::future_status::timeout);
  update_lock.unlock();
  const auto result = command_check.get();
  EXPECT_TRUE(result.is_clear());
  EXPECT_EQ(result.clearance.status, ElevatorScopedClearanceStatus::kClear);
}

} // namespace
