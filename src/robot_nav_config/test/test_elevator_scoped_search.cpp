#include <cmath>
#include <cctype>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "geometry_msgs/msg/point.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "robot_nav_config/elevator_scoped_clearance.hpp"
#include "robot_nav_config/elevator_scoped_route.hpp"
#include "robot_nav_config/elevator_scoped_search.hpp"

namespace
{

using robot_nav_config::ElevatorScopedMotionPhase;
using robot_nav_config::ElevatorScopedPose;
using robot_nav_config::ElevatorScopedRoutePolicy;
using robot_nav_config::ElevatorScopedSearchParameters;
using robot_nav_config::ElevatorScopedSearchStatus;
using robot_nav_config::evaluate_elevator_scoped_clearance;
using robot_nav_config::make_elevator_scoped_route_segments;
using robot_nav_config::search_elevator_scoped_path;

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

ElevatorScopedSearchParameters field_parameters()
{
  ElevatorScopedSearchParameters parameters;
  parameters.sampling.max_distance_m = 2.5;
  parameters.sampling.translation_step_m = 0.025;
  parameters.sampling.rotation_step_rad = 0.05;
  parameters.search_grid_step_m = 0.05;
  parameters.search_radius_m = 2.5;
  parameters.goal_connect_distance_m = 0.10;
  parameters.maximum_expansions = 60000U;
  return parameters;
}

void carve_free_rectangle(
  nav2_costmap_2d::Costmap2D & costmap,
  const double minimum_x,
  const double minimum_y,
  const double maximum_x,
  const double maximum_y)
{
  for (unsigned int cell_y = 0U; cell_y < costmap.getSizeInCellsY(); ++cell_y) {
    for (unsigned int cell_x = 0U; cell_x < costmap.getSizeInCellsX(); ++cell_x) {
      double world_x = 0.0;
      double world_y = 0.0;
      costmap.mapToWorld(cell_x, cell_y, world_x, world_y);
      if (
        world_x >= minimum_x && world_x <= maximum_x &&
        world_y >= minimum_y && world_y <= maximum_y)
      {
        costmap.setCost(cell_x, cell_y, nav2_costmap_2d::FREE_SPACE);
      }
    }
  }
}

std::string read_pgm_token(std::istream & stream)
{
  std::string token;
  char character = '\0';
  while (stream.get(character)) {
    if (character == '#') {
      std::string ignored;
      std::getline(stream, ignored);
      continue;
    }
    if (!std::isspace(static_cast<unsigned char>(character))) {
      token.push_back(character);
      break;
    }
  }
  while (stream.get(character)) {
    if (std::isspace(static_cast<unsigned char>(character))) {
      break;
    }
    token.push_back(character);
  }
  return token;
}

nav2_costmap_2d::Costmap2D load_f2_trinary_costmap(const std::string & path)
{
  std::ifstream stream(path, std::ios::binary);
  if (!stream) {
    throw std::runtime_error("cannot open F2 PGM fixture: " + path);
  }
  if (read_pgm_token(stream) != "P5") {
    throw std::runtime_error("F2 fixture is not a binary PGM");
  }
  const auto width = static_cast<unsigned int>(
    std::stoul(read_pgm_token(stream)));
  const auto height = static_cast<unsigned int>(
    std::stoul(read_pgm_token(stream)));
  const auto maximum = std::stoul(read_pgm_token(stream));
  if (width != 204U || height != 198U || maximum != 255U) {
    throw std::runtime_error("unexpected F2 PGM geometry");
  }

  std::vector<unsigned char> pixels(width * height);
  stream.read(
    reinterpret_cast<char *>(pixels.data()),
    static_cast<std::streamsize>(pixels.size()));
  if (stream.gcount() != static_cast<std::streamsize>(pixels.size())) {
    throw std::runtime_error("truncated F2 PGM fixture");
  }

  nav2_costmap_2d::Costmap2D costmap(
    width, height, 0.05, -5.466016, -4.197015,
    nav2_costmap_2d::NO_INFORMATION);
  for (unsigned int image_y = 0U; image_y < height; ++image_y) {
    const unsigned int map_y = height - image_y - 1U;
    for (unsigned int x = 0U; x < width; ++x) {
      const auto pixel = pixels[image_y * width + x];
      const double occupancy =
        (255.0 - static_cast<double>(pixel)) / 255.0;
      const std::uint8_t cost = occupancy > 0.65 ?
        nav2_costmap_2d::LETHAL_OBSTACLE :
        (occupancy < 0.196 ? nav2_costmap_2d::FREE_SPACE :
        nav2_costmap_2d::NO_INFORMATION);
      costmap.setCost(x, map_y, cost);
    }
  }
  return costmap;
}

TEST(ElevatorScopedSearch, PreservesTheExactF2LethalCellAndReturnsAClearRoute)
{
  constexpr double kResolution = 0.05;
  constexpr double kOriginX = -5.466016;
  constexpr double kOriginY = -4.197015;
  nav2_costmap_2d::Costmap2D costmap(
    204U, 198U, kResolution, kOriginX, kOriginY,
    nav2_costmap_2d::FREE_SPACE);
  costmap.setCost(71U, 76U, nav2_costmap_2d::LETHAL_OBSTACLE);
  const auto footprint = padded_ranger_footprint();

  const ElevatorScopedPose landing{-2.434692, -0.157901, 0.557354};
  const ElevatorScopedPose cabin{-0.266016, 0.102985, 0.1197220556383663};
  const auto result = search_elevator_scoped_path(
    costmap, footprint, landing, cabin, field_parameters());

  ASSERT_TRUE(result.succeeded());
  EXPECT_EQ(costmap.getCost(71U, 76U), nav2_costmap_2d::LETHAL_OBSTACLE);
  EXPECT_GT(
    result.path_length_m, std::hypot(
      cabin.x - landing.x, cabin.y - landing.y));
  ASSERT_FALSE(result.path.samples.empty());
  EXPECT_NEAR(result.path.samples.back().pose.x, cabin.x, 1.0e-9);
  EXPECT_NEAR(result.path.samples.back().pose.y, cabin.y, 1.0e-9);

  bool saw_lateral = false;
  for (const auto & sample : result.path.samples) {
    saw_lateral = saw_lateral ||
      sample.phase == ElevatorScopedMotionPhase::kLateral;
    EXPECT_TRUE(
      evaluate_elevator_scoped_clearance(
        costmap, footprint,
        sample.pose.x, sample.pose.y, sample.pose.yaw).is_clear());
  }
  EXPECT_TRUE(saw_lateral);

  std::vector<ElevatorScopedPose> route_poses;
  route_poses.reserve(result.path.samples.size());
  for (const auto & sample : result.path.samples) {
    route_poses.push_back(sample.pose);
  }
  const auto segments = make_elevator_scoped_route_segments(route_poses);
  ASSERT_FALSE(segments.empty());
  EXPECT_EQ(segments.back().end_index, result.path.samples.size() - 1U);
}

TEST(ElevatorScopedSearch, FindsAPathOnTheCompleteDeployedF2OccupancyMap)
{
  const char * fixture = std::getenv("NJRH_ELEVATOR_F2_MAP_PGM");
  if (fixture == nullptr || std::string(fixture).empty()) {
    GTEST_SKIP() << "set NJRH_ELEVATOR_F2_MAP_PGM for deployed-map validation";
  }
  auto costmap = load_f2_trinary_costmap(fixture);
  const auto result = search_elevator_scoped_path(
    costmap, padded_ranger_footprint(),
    ElevatorScopedPose{-2.434692, -0.157901, 0.557354},
    ElevatorScopedPose{-0.266016, 0.102985, 0.1197220556383663},
    field_parameters());

  ASSERT_TRUE(result.succeeded())
    << "status=" << robot_nav_config::elevator_scoped_search_status_name(
    result.status)
    << " expanded=" << result.expanded_nodes
    << " direct_block_cell=(" << result.direct_path_block.cell_x << ","
    << result.direct_path_block.cell_y << ")";
  EXPECT_EQ(
    costmap.getCost(71U, 76U), nav2_costmap_2d::LETHAL_OBSTACLE);
}

TEST(
  ElevatorScopedSearch,
  RejectsTheCapturedF2HallCallWhenHardMapTopologyHasNoConnectedCorridor)
{
  const char * fixture = std::getenv("NJRH_ELEVATOR_F2_MAP_PGM");
  if (fixture == nullptr || std::string(fixture).empty()) {
    GTEST_SKIP() << "set NJRH_ELEVATOR_F2_MAP_PGM for deployed-map validation";
  }
  auto costmap = load_f2_trinary_costmap(fixture);
  auto parameters = field_parameters();
  parameters.route_policy =
    ElevatorScopedRoutePolicy::kPreferClearStartupSpin;
  parameters.maximum_search_time_sec = 1.0;

  // Captured from the two NAVIGATING_HALL_CALL failures at
  // 2026-08-05T13:51Z. The robot never moved; both attempts exhausted the
  // production search budget at the same static lethal cell (39, 67).
  const ElevatorScopedPose start{-4.054318, -0.965755, 2.246880};
  const ElevatorScopedPose goal{-2.654357, 0.350301, 0.581367};
  const auto result = search_elevator_scoped_path(
    costmap, padded_ranger_footprint(), start, goal, parameters);

  EXPECT_FALSE(result.succeeded());
  EXPECT_EQ(result.status, ElevatorScopedSearchStatus::kNoPath);
  EXPECT_EQ(result.expanded_nodes, 0U);
  EXPECT_EQ(result.direct_path_block.cell_x, 39U);
  EXPECT_EQ(result.direct_path_block.cell_y, 67U);
  EXPECT_EQ(
    result.direct_path_block.cost, nav2_costmap_2d::LETHAL_OBSTACLE);
}

TEST(
  ElevatorScopedSearch,
  KeepsIngressHeadingUntilClearThenTurnsInsideAnLShapedCabinApproach)
{
  nav2_costmap_2d::Costmap2D costmap(
    120U, 120U, 0.05, -3.0, -3.0,
    nav2_costmap_2d::LETHAL_OBSTACLE);
  carve_free_rectangle(costmap, -1.5, -0.40, 0.80, 0.40);
  carve_free_rectangle(costmap, 0.10, -0.60, 1.40, 0.60);
  carve_free_rectangle(costmap, 0.44, -1.40, 1.16, -0.20);
  carve_free_rectangle(costmap, 0.10, -1.80, 1.40, -0.70);

  auto parameters = field_parameters();
  parameters.maximum_search_time_sec = 1.0;
  const ElevatorScopedPose start{-1.0, 0.0, 0.0};
  const ElevatorScopedPose goal{0.8, -1.2, 3.14159265358979323846};
  const auto result = search_elevator_scoped_path(
    costmap, padded_ranger_footprint(), start, goal, parameters);

  ASSERT_TRUE(result.succeeded())
    << "status=" << robot_nav_config::elevator_scoped_search_status_name(
    result.status)
    << " expanded=" << result.expanded_nodes;
  ASSERT_FALSE(result.path.samples.empty());

  bool translated = false;
  bool turned_after_progress = false;
  bool moved_forward_after_turn = false;
  for (const auto & sample : result.path.samples) {
    const double progress = std::hypot(
      sample.pose.x - start.x, sample.pose.y - start.y);
    const double yaw_delta = std::atan2(
      std::sin(sample.pose.yaw - start.yaw),
      std::cos(sample.pose.yaw - start.yaw));
    if (!translated && sample.phase != ElevatorScopedMotionPhase::kYaw) {
      EXPECT_EQ(sample.phase, ElevatorScopedMotionPhase::kForward);
      translated = true;
    }
    if (translated && progress > 0.50 && std::abs(yaw_delta) > 1.0) {
      turned_after_progress = true;
    }
    if (
      turned_after_progress &&
      sample.phase == ElevatorScopedMotionPhase::kForward)
    {
      moved_forward_after_turn = true;
    }
  }
  EXPECT_TRUE(translated);
  EXPECT_TRUE(turned_after_progress);
  EXPECT_TRUE(moved_forward_after_turn);
  EXPECT_NEAR(result.path.samples.back().pose.x, goal.x, 1.0e-9);
  EXPECT_NEAR(result.path.samples.back().pose.y, goal.y, 1.0e-9);
  EXPECT_NEAR(result.path.samples.back().pose.yaw, goal.yaw, 1.0e-9);
}

TEST(
  ElevatorScopedSearch,
  ReverseDockingSequenceTurnsThenMovesLongitudinallyThenLaterally)
{
  nav2_costmap_2d::Costmap2D costmap(
    200U, 200U, 0.05, -5.0, -5.0,
    nav2_costmap_2d::FREE_SPACE);
  auto parameters = field_parameters();
  parameters.route_policy =
    ElevatorScopedRoutePolicy::kReverseDockingSequence;
  parameters.maximum_search_time_sec = 1.0;
  const ElevatorScopedPose start{0.0, 0.0, 0.0};
  const ElevatorScopedPose goal{1.0, 0.5, 3.14159265358979323846};

  const auto result = search_elevator_scoped_path(
    costmap, padded_ranger_footprint(), start, goal, parameters);

  ASSERT_TRUE(result.succeeded())
    << elevator_scoped_search_status_name(result.status);
  EXPECT_FALSE(result.used_detour);
  EXPECT_EQ(result.expanded_nodes, 0U);
  std::vector<ElevatorScopedMotionPhase> episodes;
  auto previous = result.path.samples.front().pose;
  for (const auto & sample : result.path.samples) {
    const bool moved = std::hypot(
      sample.pose.x - previous.x, sample.pose.y - previous.y) > 1.0e-6;
    const bool turned = std::abs(std::atan2(
        std::sin(sample.pose.yaw - previous.yaw),
        std::cos(sample.pose.yaw - previous.yaw))) > 1.0e-6;
    if ((moved || turned) &&
      (episodes.empty() || episodes.back() != sample.phase))
    {
      episodes.push_back(sample.phase);
    }
    if (moved) {
      EXPECT_NEAR(sample.pose.yaw, goal.yaw, 1.0e-9);
    }
    previous = sample.pose;
  }
  ASSERT_EQ(episodes.size(), 3U);
  EXPECT_EQ(episodes[0], ElevatorScopedMotionPhase::kYaw);
  EXPECT_EQ(episodes[1], ElevatorScopedMotionPhase::kReverse);
  EXPECT_EQ(episodes[2], ElevatorScopedMotionPhase::kLateral);
  EXPECT_NEAR(result.path.samples.back().pose.x, goal.x, 1.0e-9);
  EXPECT_NEAR(result.path.samples.back().pose.y, goal.y, 1.0e-9);
  EXPECT_NEAR(result.path.samples.back().pose.yaw, goal.yaw, 1.0e-9);
}

TEST(
  ElevatorScopedSearch,
  ReverseEntryStagingTurnsThenMovesLaterallyThenLongitudinally)
{
  nav2_costmap_2d::Costmap2D costmap(
    200U, 200U, 0.05, -5.0, -5.0,
    nav2_costmap_2d::FREE_SPACE);
  auto parameters = field_parameters();
  parameters.route_policy =
    ElevatorScopedRoutePolicy::kReverseEntryStagingSequence;
  parameters.maximum_search_time_sec = 1.0;

  // Captured from B11/F2 elevator-test-1786108908308-1531468606675040-20.
  // In the commissioned landing yaw the target is 43.01 cm to the left and
  // 11.23 cm behind the live hall-call pose.  The staging contract must not
  // start by reversing toward the nearby door-frame cost.
  const ElevatorScopedPose start{-2.560861, 0.322567, -2.602372};
  const ElevatorScopedPose goal{-2.235149, 0.020067, -2.574592};

  const auto result = search_elevator_scoped_path(
    costmap, padded_ranger_footprint(), start, goal, parameters);

  ASSERT_TRUE(result.succeeded())
    << elevator_scoped_search_status_name(result.status);
  EXPECT_FALSE(result.used_detour);
  EXPECT_EQ(result.expanded_nodes, 0U);
  EXPECT_NEAR(result.path.forward_m, -0.112269, 1.0e-5);
  EXPECT_NEAR(result.path.lateral_m, 0.430105, 1.0e-5);
  EXPECT_NEAR(result.path_length_m, 0.542374, 1.0e-5);

  std::vector<ElevatorScopedMotionPhase> episodes;
  auto previous = result.path.samples.front().pose;
  for (const auto & sample : result.path.samples) {
    const bool moved = std::hypot(
      sample.pose.x - previous.x, sample.pose.y - previous.y) > 1.0e-6;
    const bool turned = std::abs(std::atan2(
        std::sin(sample.pose.yaw - previous.yaw),
        std::cos(sample.pose.yaw - previous.yaw))) > 1.0e-6;
    if ((moved || turned) &&
      (episodes.empty() || episodes.back() != sample.phase))
    {
      episodes.push_back(sample.phase);
    }
    if (moved) {
      EXPECT_NEAR(sample.pose.yaw, goal.yaw, 1.0e-9);
    }
    previous = sample.pose;
  }

  ASSERT_EQ(episodes.size(), 3U);
  EXPECT_EQ(episodes[0], ElevatorScopedMotionPhase::kYaw);
  EXPECT_EQ(episodes[1], ElevatorScopedMotionPhase::kLateral);
  EXPECT_EQ(episodes[2], ElevatorScopedMotionPhase::kReverse);
  EXPECT_NEAR(result.path.samples.back().pose.x, goal.x, 1.0e-9);
  EXPECT_NEAR(result.path.samples.back().pose.y, goal.y, 1.0e-9);
  EXPECT_NEAR(result.path.samples.back().pose.yaw, goal.yaw, 1.0e-9);
}

TEST(
  ElevatorScopedSearch,
  ReverseDockingSequenceDoesNotReplaceCommissionedManeuverWithADetour)
{
  nav2_costmap_2d::Costmap2D costmap(
    200U, 200U, 0.05, -5.0, -5.0,
    nav2_costmap_2d::FREE_SPACE);
  unsigned int obstacle_x = 0U;
  unsigned int obstacle_y = 0U;
  ASSERT_TRUE(costmap.worldToMap(0.55, 0.0, obstacle_x, obstacle_y));
  costmap.setCost(
    obstacle_x, obstacle_y, nav2_costmap_2d::LETHAL_OBSTACLE);
  auto parameters = field_parameters();
  parameters.route_policy =
    ElevatorScopedRoutePolicy::kReverseDockingSequence;
  parameters.maximum_search_time_sec = 1.0;

  const auto result = search_elevator_scoped_path(
    costmap, padded_ranger_footprint(),
    ElevatorScopedPose{0.0, 0.0, 0.0},
    ElevatorScopedPose{1.2, 0.0, 3.14159265358979323846},
    parameters);

  EXPECT_FALSE(result.succeeded());
  EXPECT_EQ(result.status, ElevatorScopedSearchStatus::kNoPath);
  EXPECT_EQ(result.expanded_nodes, 0U);
  EXPECT_FALSE(result.used_detour);
}

TEST(
  ElevatorScopedSearch,
  ClearNearbyHallCallSpinsTowardTravelHeadingBeforeTranslation)
{
  nav2_costmap_2d::Costmap2D costmap(
    200U, 200U, 0.05, -5.0, -5.0,
    nav2_costmap_2d::FREE_SPACE);
  auto parameters = field_parameters();
  parameters.maximum_search_time_sec = 1.0;
  parameters.route_policy =
    ElevatorScopedRoutePolicy::kPreferClearStartupSpin;

  // Captured from elevator-test-1785871336509-1293896807666816-2.
  const ElevatorScopedPose start{-1.610729, 0.395740, 0.563186};
  const ElevatorScopedPose goal{-2.654357, 0.350301, 0.581367};
  const auto result = search_elevator_scoped_path(
    costmap, padded_ranger_footprint(), start, goal, parameters);

  ASSERT_TRUE(result.succeeded())
    << "status=" << robot_nav_config::elevator_scoped_search_status_name(
    result.status)
    << " expanded=" << result.expanded_nodes;
  ASSERT_FALSE(result.path.samples.empty());

  bool saw_forward = false;
  double maximum_yaw_change = 0.0;
  std::optional<ElevatorScopedMotionPhase> first_motion;
  auto previous = result.path.samples.front().pose;
  for (const auto & sample : result.path.samples) {
    const bool moved = std::hypot(
      sample.pose.x - previous.x, sample.pose.y - previous.y) > 1.0e-6;
    const bool turned = std::abs(std::atan2(
        std::sin(sample.pose.yaw - previous.yaw),
        std::cos(sample.pose.yaw - previous.yaw))) > 1.0e-6;
    if (!first_motion && (moved || turned)) {
      first_motion = sample.phase;
    }
    saw_forward = saw_forward ||
      sample.phase == ElevatorScopedMotionPhase::kForward;
    maximum_yaw_change = std::max(
      maximum_yaw_change,
      std::abs(std::atan2(
        std::sin(sample.pose.yaw - start.yaw),
        std::cos(sample.pose.yaw - start.yaw))));
    previous = sample.pose;
  }

  ASSERT_TRUE(first_motion.has_value());
  EXPECT_EQ(*first_motion, ElevatorScopedMotionPhase::kYaw);
  EXPECT_TRUE(saw_forward);
  EXPECT_GT(maximum_yaw_change, 2.0);
  EXPECT_FALSE(result.used_startup_rotation_escape);
  EXPECT_NEAR(result.path.samples.back().pose.x, goal.x, 1.0e-9);
  EXPECT_NEAR(result.path.samples.back().pose.y, goal.y, 1.0e-9);
  EXPECT_NEAR(result.path.samples.back().pose.yaw, goal.yaw, 1.0e-9);
}

TEST(
  ElevatorScopedSearch,
  BlockedStartupSpinUsesCheckedTranslationWithoutRetryingTheUnsafeTurn)
{
  nav2_costmap_2d::Costmap2D costmap(
    160U, 160U, 0.05, -4.0, -4.0,
    nav2_costmap_2d::FREE_SPACE);
  unsigned int obstacle_x = 0U;
  unsigned int obstacle_y = 0U;
  ASSERT_TRUE(costmap.worldToMap(0.0, 0.40, obstacle_x, obstacle_y));
  costmap.setCost(
    obstacle_x, obstacle_y, nav2_costmap_2d::LETHAL_OBSTACLE);

  auto parameters = field_parameters();
  parameters.maximum_search_time_sec = 1.0;
  parameters.route_policy =
    ElevatorScopedRoutePolicy::kPreferClearStartupSpin;
  const ElevatorScopedPose start{0.0, 0.0, 0.0};
  const ElevatorScopedPose goal{-1.0, 0.0, 0.0};
  ASSERT_TRUE(
    evaluate_elevator_scoped_clearance(
      costmap, padded_ranger_footprint(), start.x, start.y, start.yaw).is_clear());

  const auto result = search_elevator_scoped_path(
    costmap, padded_ranger_footprint(), start, goal, parameters);

  ASSERT_TRUE(result.succeeded())
    << "status=" << robot_nav_config::elevator_scoped_search_status_name(
    result.status)
    << " expanded=" << result.expanded_nodes;
  ASSERT_FALSE(result.path.samples.empty());

  std::optional<ElevatorScopedMotionPhase> first_motion;
  bool translated_before_turn = false;
  auto previous = result.path.samples.front().pose;
  for (const auto & sample : result.path.samples) {
    const bool moved = std::hypot(
      sample.pose.x - previous.x, sample.pose.y - previous.y) > 1.0e-6;
    const bool turned = std::abs(std::atan2(
        std::sin(sample.pose.yaw - previous.yaw),
        std::cos(sample.pose.yaw - previous.yaw))) > 1.0e-6;
    if (!first_motion && (moved || turned)) {
      first_motion = sample.phase;
    }
    if (moved) {
      translated_before_turn = true;
    }
    EXPECT_TRUE(
      evaluate_elevator_scoped_clearance(
        costmap, padded_ranger_footprint(),
        sample.pose.x, sample.pose.y, sample.pose.yaw).is_clear());
    previous = sample.pose;
  }

  ASSERT_TRUE(first_motion.has_value());
  EXPECT_EQ(*first_motion, ElevatorScopedMotionPhase::kReverse);
  EXPECT_TRUE(translated_before_turn);
  EXPECT_TRUE(result.used_startup_rotation_escape);
  EXPECT_NEAR(result.path.samples.back().pose.x, goal.x, 1.0e-9);
  EXPECT_NEAR(result.path.samples.back().pose.y, goal.y, 1.0e-9);
  EXPECT_NEAR(result.path.samples.back().pose.yaw, goal.yaw, 1.0e-9);
}

TEST(
  ElevatorScopedSearch,
  BlockedStartupSpinCanUsePureLateralMotionWithoutChangingYaw)
{
  nav2_costmap_2d::Costmap2D costmap(
    160U, 160U, 0.05, -4.0, -4.0,
    nav2_costmap_2d::FREE_SPACE);
  unsigned int obstacle_x = 0U;
  unsigned int obstacle_y = 0U;
  ASSERT_TRUE(costmap.worldToMap(0.45, 0.0, obstacle_x, obstacle_y));
  costmap.setCost(
    obstacle_x, obstacle_y, nav2_costmap_2d::LETHAL_OBSTACLE);

  auto parameters = field_parameters();
  parameters.route_policy =
    ElevatorScopedRoutePolicy::kPreferClearStartupSpin;
  parameters.maximum_search_time_sec = 1.0;
  const ElevatorScopedPose start{0.0, 0.0, 0.0};
  const ElevatorScopedPose goal{0.0, 1.0, 0.0};
  ASSERT_TRUE(
    evaluate_elevator_scoped_clearance(
      costmap, padded_ranger_footprint(), start.x, start.y, start.yaw).is_clear());

  const auto result = search_elevator_scoped_path(
    costmap, padded_ranger_footprint(), start, goal, parameters);

  ASSERT_TRUE(result.succeeded())
    << "status=" << robot_nav_config::elevator_scoped_search_status_name(
    result.status)
    << " expanded=" << result.expanded_nodes;
  ASSERT_FALSE(result.path.samples.empty());
  std::optional<ElevatorScopedMotionPhase> first_motion;
  auto previous = result.path.samples.front().pose;
  for (const auto & sample : result.path.samples) {
    const bool moved = std::hypot(
      sample.pose.x - previous.x, sample.pose.y - previous.y) > 1.0e-6;
    const bool turned = std::abs(std::atan2(
        std::sin(sample.pose.yaw - previous.yaw),
        std::cos(sample.pose.yaw - previous.yaw))) > 1.0e-6;
    if (!first_motion && (moved || turned)) {
      first_motion = sample.phase;
    }
    EXPECT_TRUE(
      evaluate_elevator_scoped_clearance(
        costmap, padded_ranger_footprint(),
        sample.pose.x, sample.pose.y, sample.pose.yaw).is_clear());
    previous = sample.pose;
  }
  ASSERT_TRUE(first_motion.has_value());
  EXPECT_EQ(*first_motion, ElevatorScopedMotionPhase::kLateral);
  EXPECT_NEAR(result.path.samples.back().pose.yaw, start.yaw, 1.0e-9);
}

TEST(
  ElevatorScopedSearch,
  HallPolicyComposesTranslationsAndMultipleIntermediateTurnsInANarrowRoute)
{
  nav2_costmap_2d::Costmap2D costmap(
    160U, 160U, 0.05, -4.0, -4.0,
    nav2_costmap_2d::LETHAL_OBSTACLE);

  // Two 0.70 m corridors joined by rotation pockets. The padded Ranger is
  // 0.78 x 0.56 m: it must enter the first pocket facing east, rotate north,
  // traverse the vertical leg, rotate east again, then leave the second
  // pocket. A translate-on-one-heading plus one final straight chord cannot
  // satisfy this geometry.
  carve_free_rectangle(costmap, -1.30, -0.35, 0.10, 0.35);
  carve_free_rectangle(costmap, -0.55, -0.55, 0.55, 0.55);
  carve_free_rectangle(costmap, -0.35, 0.00, 0.35, 1.60);
  carve_free_rectangle(costmap, -0.55, 1.05, 0.55, 2.15);
  carve_free_rectangle(costmap, 0.00, 1.25, 1.30, 1.95);

  auto parameters = field_parameters();
  parameters.route_policy =
    ElevatorScopedRoutePolicy::kPreferClearStartupSpin;
  parameters.maximum_search_time_sec = 1.0;
  const ElevatorScopedPose start{-0.80, 0.0, 0.0};
  const ElevatorScopedPose goal{0.80, 1.60, 0.0};
  const auto result = search_elevator_scoped_path(
    costmap, padded_ranger_footprint(), start, goal, parameters);

  ASSERT_TRUE(result.succeeded())
    << "status=" << robot_nav_config::elevator_scoped_search_status_name(
    result.status)
    << " expanded=" << result.expanded_nodes;
  ASSERT_FALSE(result.path.samples.empty());

  std::vector<ElevatorScopedMotionPhase> motion_episodes;
  auto previous_pose = result.path.samples.front().pose;
  for (const auto & sample : result.path.samples) {
    const bool moved = std::hypot(
      sample.pose.x - previous_pose.x,
      sample.pose.y - previous_pose.y) > 1.0e-6;
    const bool turned = std::abs(std::atan2(
        std::sin(sample.pose.yaw - previous_pose.yaw),
        std::cos(sample.pose.yaw - previous_pose.yaw))) > 1.0e-6;
    if ((moved || turned) &&
      (motion_episodes.empty() || motion_episodes.back() != sample.phase))
    {
      motion_episodes.push_back(sample.phase);
    }
    EXPECT_TRUE(
      evaluate_elevator_scoped_clearance(
        costmap, padded_ranger_footprint(),
        sample.pose.x, sample.pose.y, sample.pose.yaw).is_clear());
    previous_pose = sample.pose;
  }

  bool translated = false;
  bool first_intermediate_turn = false;
  bool translated_after_first_turn = false;
  bool second_intermediate_turn = false;
  bool translated_after_second_turn = false;
  for (const auto phase : motion_episodes) {
    if (phase != ElevatorScopedMotionPhase::kYaw) {
      if (second_intermediate_turn) {
        translated_after_second_turn = true;
      } else if (first_intermediate_turn) {
        translated_after_first_turn = true;
      } else {
        translated = true;
      }
      continue;
    }
    if (translated_after_first_turn) {
      second_intermediate_turn = true;
    } else if (translated) {
      first_intermediate_turn = true;
    }
  }
  EXPECT_TRUE(translated);
  EXPECT_TRUE(first_intermediate_turn);
  EXPECT_TRUE(translated_after_first_turn);
  EXPECT_TRUE(second_intermediate_turn);
  EXPECT_TRUE(translated_after_second_turn);
  EXPECT_GE(result.mode_switches, 4U);
  EXPECT_NEAR(result.path.samples.back().pose.x, goal.x, 1.0e-9);
  EXPECT_NEAR(result.path.samples.back().pose.y, goal.y, 1.0e-9);
  EXPECT_NEAR(result.path.samples.back().pose.yaw, goal.yaw, 1.0e-9);
}

TEST(ElevatorScopedSearch, TreatsInflationAsCostRatherThanAWall)
{
  nav2_costmap_2d::Costmap2D costmap(
    100U, 100U, 0.05, -2.5, -2.5,
    nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
  const auto footprint = padded_ranger_footprint();
  const ElevatorScopedPose start{-1.0, 0.0, 0.0};
  const ElevatorScopedPose goal{1.0, 0.0, 0.0};

  const auto result = search_elevator_scoped_path(
    costmap, footprint, start, goal, field_parameters());

  ASSERT_TRUE(result.succeeded());
  EXPECT_FALSE(result.used_detour);
}

TEST(ElevatorScopedSearch, ChoosesLowerCostDetourWhenDirectRouteCrossesSoftInflation)
{
  nav2_costmap_2d::Costmap2D costmap(
    120U, 120U, 0.05, -3.0, -3.0, nav2_costmap_2d::FREE_SPACE);
  const ElevatorScopedPose start{-1.0, 0.0, 0.0};
  const ElevatorScopedPose goal{1.0, 0.0, 0.0};

  unsigned int start_x = 0U;
  unsigned int center_y = 0U;
  unsigned int goal_x = 0U;
  unsigned int ignored_y = 0U;
  ASSERT_TRUE(costmap.worldToMap(start.x, start.y, start_x, center_y));
  ASSERT_TRUE(costmap.worldToMap(goal.x, goal.y, goal_x, ignored_y));
  for (unsigned int x = start_x; x <= goal_x; ++x) {
    costmap.setCost(
      x, center_y, nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
  }

  const auto result = search_elevator_scoped_path(
    costmap, padded_ranger_footprint(), start, goal, field_parameters());

  ASSERT_TRUE(result.succeeded());
  EXPECT_TRUE(result.used_detour)
    << "direct_objective=" << result.direct_path_objective
    << " searched_objective=" << result.searched_path_objective
    << " selected_objective=" << result.selected_path_objective
    << " expanded=" << result.expanded_nodes
    << " path_length=" << result.path_length_m;
  bool left_soft_centerline = false;
  for (const auto & sample : result.path.samples) {
    left_soft_centerline = left_soft_centerline ||
      std::abs(sample.pose.y) >= 0.049;
  }
  EXPECT_TRUE(left_soft_centerline);
}

TEST(ElevatorScopedSearch, FailsClosedWhenNoFootprintRouteExists)
{
  nav2_costmap_2d::Costmap2D costmap(
    120U, 120U, 0.05, -3.0, -3.0, nav2_costmap_2d::FREE_SPACE);
  for (unsigned int y = 0U; y < costmap.getSizeInCellsY(); ++y) {
    costmap.setCost(60U, y, nav2_costmap_2d::LETHAL_OBSTACLE);
  }
  const auto footprint = padded_ranger_footprint();
  const ElevatorScopedPose start{-1.0, 0.0, 0.0};
  const ElevatorScopedPose goal{1.0, 0.0, 0.0};

  const auto result = search_elevator_scoped_path(
    costmap, footprint, start, goal, field_parameters());

  EXPECT_FALSE(result.succeeded());
  EXPECT_TRUE(
    result.status == ElevatorScopedSearchStatus::kNoPath ||
    result.status == ElevatorScopedSearchStatus::kExpansionLimit ||
    result.status == ElevatorScopedSearchStatus::kTimeLimit);
}

TEST(ElevatorScopedSearch, StopsAnUnsolvableSearchAtItsTimeBudget)
{
  nav2_costmap_2d::Costmap2D costmap(
    240U, 240U, 0.025, -3.0, -3.0, nav2_costmap_2d::FREE_SPACE);
  for (unsigned int y = 0U; y < costmap.getSizeInCellsY(); ++y) {
    costmap.setCost(120U, y, nav2_costmap_2d::LETHAL_OBSTACLE);
  }
  auto parameters = field_parameters();
  parameters.search_grid_step_m = 0.025;
  parameters.maximum_expansions = 1000000U;
  parameters.maximum_search_time_sec = 0.001;

  const auto result = search_elevator_scoped_path(
    costmap, padded_ranger_footprint(),
    ElevatorScopedPose{-1.0, 0.0, 0.0},
    ElevatorScopedPose{1.0, 0.0, 0.0}, parameters);

  EXPECT_FALSE(result.succeeded());
  EXPECT_EQ(result.status, ElevatorScopedSearchStatus::kTimeLimit);
}

}  // namespace
