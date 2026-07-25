#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <vector>

#include "gtest/gtest.h"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_smac_planner/a_star.hpp"
#include "nav2_smac_planner/collision_checker.hpp"
#include "nav2_smac_planner/node_lattice.hpp"
#include "nav2_smac_planner/types.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "rclcpp/rclcpp.hpp"

#ifndef RANGER_LATTICE_TEST_PATH
#error "RANGER_LATTICE_TEST_PATH must point to the generated lattice artifact"
#endif

namespace
{

constexpr float kResolutionM = 0.05F;
constexpr unsigned int kHeadingCount = 16U;

class RclcppFixture
{
public:
  RclcppFixture()
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  ~RclcppFixture()
  {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }
};

RclcppFixture g_rclcpp_fixture;

struct PlanMetrics
{
  bool created{false};
  int iterations{0};
  double length_m{0.0};
  double max_cross_track_m{0.0};
  double startup_spin_abs_rad{0.0};
  double first_translation_heading_error_rad{std::numeric_limits<double>::infinity()};
  std::size_t midpath_spin_segment_count{0U};
  std::size_t reverse_segment_count{0U};
  double first_reverse_distance_from_start_m{std::numeric_limits<double>::infinity()};
  double last_reverse_distance_from_goal_m{std::numeric_limits<double>::infinity()};
  double max_reverse_euclidean_goal_distance_m{0.0};
};

struct GridPose
{
  unsigned int x;
  unsigned int y;
  unsigned int heading;
};

double normalizeAngle(const double value)
{
  return std::atan2(std::sin(value), std::cos(value));
}

void resetLatticeStatics()
{
  nav2_smac_planner::NodeLattice::motion_table =
    nav2_smac_planner::LatticeMotionTable{};
  nav2_smac_planner::NodeLattice::dist_heuristic_lookup_table.clear();
  nav2_smac_planner::NodeLattice::size_lookup = 25.0F;
}

PlanMetrics planScenario(
  const bool allow_reverse_expansion,
  const GridPose & start,
  const GridPose & goal,
  const float reverse_penalty = 2.10F,
  const float rotation_penalty = 5.0F)
{
  resetLatticeStatics();

  nav2_smac_planner::SearchInfo search_info{};
  search_info.minimum_turning_radius = 0.81F / kResolutionM;
  search_info.non_straight_penalty = 1.05F;
  search_info.change_penalty = 0.20F;
  search_info.reverse_penalty = reverse_penalty;
  search_info.cost_penalty = 2.0F;
  search_info.retrospective_penalty = 0.015F;
  search_info.rotation_penalty = rotation_penalty;
  search_info.analytic_expansion_ratio = 3.5F;
  search_info.analytic_expansion_max_length = 4.05F / kResolutionM;
  search_info.lattice_filepath = RANGER_LATTICE_TEST_PATH;
  search_info.cache_obstacle_heuristic = false;
  search_info.allow_reverse_expansion = allow_reverse_expansion;

  nav2_smac_planner::AStarAlgorithm<nav2_smac_planner::NodeLattice> planner(
    nav2_smac_planner::MotionModel::STATE_LATTICE, search_info);
  int max_iterations = 1000000;
  constexpr int kMaxOnApproachIterations = 1000;
  constexpr double kMaxPlanningTimeSec = 10.0;
  constexpr float kLookupTableSizeCells = 401.0F;
  planner.initialize(
    false, max_iterations, kMaxOnApproachIterations, kMaxPlanningTimeSec,
    kLookupTableSizeCells, kHeadingCount);

  auto node = std::make_shared<nav2_util::LifecycleNode>(
    allow_reverse_expansion ? "ranger_lattice_reverse_test" :
    "ranger_lattice_forward_test");
  auto costmap = std::make_unique<nav2_costmap_2d::Costmap2D>(
    240U, 240U, kResolutionM, 0.0, 0.0, 0U);
  auto collision_checker = std::make_unique<nav2_smac_planner::GridCollisionChecker>(
    costmap.get(), kHeadingCount, node);
  collision_checker->setFootprint(nav2_costmap_2d::Footprint(), true, 0.0);
  planner.setCollisionChecker(collision_checker.get());

  planner.setStart(start.x, start.y, start.heading);
  planner.setGoal(goal.x, goal.y, goal.heading);

  nav2_smac_planner::NodeLattice::CoordinateVector reverse_ordered_path;
  PlanMetrics metrics;
  constexpr float kToleranceCells = 1.0F;
  metrics.created = planner.createPath(
    reverse_ordered_path, metrics.iterations, kToleranceCells);
  if (!metrics.created || reverse_ordered_path.size() < 2U) {
    return metrics;
  }

  std::vector<nav2_smac_planner::NodeLattice::Coordinates> path(
    reverse_ordered_path.rbegin(), reverse_ordered_path.rend());
  std::vector<double> segment_lengths;
  segment_lengths.reserve(path.size() - 1U);
  for (std::size_t index = 0; index + 1U < path.size(); ++index) {
    const double dx = path[index + 1U].x - path[index].x;
    const double dy = path[index + 1U].y - path[index].y;
    const double segment_length_m = std::hypot(dx, dy) * kResolutionM;
    segment_lengths.push_back(segment_length_m);
    metrics.length_m += segment_length_m;
  }

  const double route_dx = static_cast<double>(goal.x) - static_cast<double>(start.x);
  const double route_dy = static_cast<double>(goal.y) - static_cast<double>(start.y);
  const double route_length_cells = std::hypot(route_dx, route_dy);
  const double route_heading = std::atan2(route_dy, route_dx);
  if (route_length_cells > 1.0e-9) {
    for (const auto & coordinate : path) {
      const double point_dx = coordinate.x - static_cast<double>(start.x);
      const double point_dy = coordinate.y - static_cast<double>(start.y);
      const double cross_track_cells =
        std::abs(route_dx * point_dy - route_dy * point_dx) / route_length_cells;
      metrics.max_cross_track_m = std::max(
        metrics.max_cross_track_m, cross_track_cells * kResolutionM);
    }
  }

  double cumulative_m = 0.0;
  bool translation_started = false;
  for (std::size_t index = 0; index < segment_lengths.size(); ++index) {
    const double dx = path[index + 1U].x - path[index].x;
    const double dy = path[index + 1U].y - path[index].y;
    const double yaw = path[index].theta;
    const double delta_yaw = normalizeAngle(path[index + 1U].theta - path[index].theta);
    const double forward_projection = dx * std::cos(yaw) + dy * std::sin(yaw);
    if (!translation_started && segment_lengths[index] <= 0.002) {
      metrics.startup_spin_abs_rad += std::abs(delta_yaw);
    } else if (!translation_started) {
      translation_started = true;
      metrics.first_translation_heading_error_rad =
        std::abs(normalizeAngle(yaw - route_heading));
    }
    if (translation_started && segment_lengths[index] <= 0.002 &&
      std::abs(delta_yaw) > 1.0e-4 &&
      cumulative_m > 0.05 && metrics.length_m - cumulative_m > 0.05)
    {
      ++metrics.midpath_spin_segment_count;
    }
    if (segment_lengths[index] > 0.002 && forward_projection < -1.0e-4) {
      ++metrics.reverse_segment_count;
      metrics.first_reverse_distance_from_start_m = std::min(
        metrics.first_reverse_distance_from_start_m, cumulative_m);
      metrics.last_reverse_distance_from_goal_m = std::min(
        metrics.last_reverse_distance_from_goal_m,
        metrics.length_m - cumulative_m);
      metrics.max_reverse_euclidean_goal_distance_m = std::max(
        metrics.max_reverse_euclidean_goal_distance_m,
        std::hypot(
          path[index].x - static_cast<double>(goal.x),
          path[index].y - static_cast<double>(goal.y)) * kResolutionM);
    }
    cumulative_m += segment_lengths[index];
  }

  return metrics;
}

}  // namespace

TEST(RangerLatticeTerminalReachability, ReverseExpansionShortensLateralResidual)
{
  // Leg 8 ended facing south with the goal about 8 cm to robot-right and
  // 3 cm forward. At 5 cm resolution this is the corresponding grid case.
  constexpr GridPose kStart{120U, 120U, 12U};
  constexpr GridPose kGoal{118U, 119U, 12U};
  const auto forward_only = planScenario(false, kStart, kGoal, 2.10F, 5.0F);
  const auto reverse_enabled = planScenario(true, kStart, kGoal, 2.10F, 3.0F);

  ASSERT_TRUE(forward_only.created);
  ASSERT_TRUE(reverse_enabled.created);
  std::cout << "forward_only_length_m=" << forward_only.length_m
            << " iterations=" << forward_only.iterations << '\n'
            << "reverse_enabled_length_m=" << reverse_enabled.length_m
            << " reverse_segments=" << reverse_enabled.reverse_segment_count
            << " max_reverse_goal_distance_m="
            << reverse_enabled.max_reverse_euclidean_goal_distance_m
            << " iterations=" << reverse_enabled.iterations << std::endl;

  EXPECT_GT(forward_only.length_m, 2.0);
  EXPECT_LT(reverse_enabled.length_m, 1.5);
  EXPECT_LT(reverse_enabled.length_m, forward_only.length_m * 0.5);
  EXPECT_GT(reverse_enabled.reverse_segment_count, 0U);
  EXPECT_LE(reverse_enabled.max_reverse_euclidean_goal_distance_m, 0.30);
}

TEST(RangerLatticeTerminalReachability, ReverseExpansionDoesNotReverseLongRoutes)
{
  constexpr float kSelectedRotationPenalty = 3.0F;
  const auto straight = planScenario(
    true, GridPose{40U, 120U, 0U}, GridPose{200U, 120U, 0U},
    2.10F, kSelectedRotationPenalty);
  const auto turn_around_then_drive = planScenario(
    true, GridPose{200U, 120U, 0U}, GridPose{40U, 120U, 8U},
    2.10F, kSelectedRotationPenalty);

  ASSERT_TRUE(straight.created);
  ASSERT_TRUE(turn_around_then_drive.created);
  std::cout << "straight_length_m=" << straight.length_m
             << " reverse_segments=" << straight.reverse_segment_count << '\n'
             << "turn_around_length_m=" << turn_around_then_drive.length_m
             << " reverse_segments=" << turn_around_then_drive.reverse_segment_count
             << " max_cross_track_m=" << turn_around_then_drive.max_cross_track_m
             << " startup_spin_deg="
             << turn_around_then_drive.startup_spin_abs_rad * 180.0 / M_PI
             << std::endl;

  EXPECT_EQ(straight.reverse_segment_count, 0U);
  EXPECT_EQ(turn_around_then_drive.reverse_segment_count, 0U);
  EXPECT_NEAR(straight.length_m, 8.0, 0.30);
  EXPECT_NEAR(turn_around_then_drive.length_m, 8.0, 0.30);
}
