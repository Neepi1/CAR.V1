#include <cmath>
#include <cstddef>

#include "gtest/gtest.h"
#include "robot_nav_config/ranger_direct_corridor.hpp"

namespace robot_nav_config
{
namespace
{

geometry_msgs::msg::PoseStamped pose(
  const double x, const double y, const double yaw)
{
  geometry_msgs::msg::PoseStamped result;
  result.header.frame_id = "map";
  result.pose.position.x = x;
  result.pose.position.y = y;
  result.pose.orientation = yaw_quaternion(yaw);
  return result;
}

TEST(RangerDirectCorridor, OppositeHeadingClearRouteBecomesStraight)
{
  const auto start = pose(0.0, 0.0, M_PI);
  const auto goal = pose(8.0, 0.0, 0.0);
  DirectCorridorParameters parameters;
  std::size_t checked_pose_count = 0U;

  const auto path = make_direct_corridor_path(
    start, goal, parameters,
    [&checked_pose_count](const double, const double, const double) {
      ++checked_pose_count;
      return true;
    });

  ASSERT_TRUE(path.has_value());
  ASSERT_GT(path->poses.size(), 2U);
  EXPECT_EQ(checked_pose_count, path->poses.size());
  EXPECT_DOUBLE_EQ(path->poses.front().pose.position.x, 0.0);
  EXPECT_DOUBLE_EQ(path->poses.back().pose.position.x, 8.0);
  for (const auto & path_pose : path->poses) {
    EXPECT_NEAR(path_pose.pose.position.y, 0.0, 1.0e-9);
    EXPECT_NEAR(quaternion_yaw(path_pose.pose.orientation), 0.0, 1.0e-9);
  }
}

TEST(RangerDirectCorridor, OccupiedSamplePreservesLatticeFallback)
{
  const auto start = pose(0.0, 0.0, M_PI);
  const auto goal = pose(8.0, 0.0, 0.0);
  DirectCorridorParameters parameters;

  const auto path = make_direct_corridor_path(
    start, goal, parameters,
    [](const double x, const double, const double) {
      return x < 3.9 || x > 4.1;
    });

  EXPECT_FALSE(path.has_value());
}

TEST(RangerDirectCorridor, GoalHeadingMustAgreeWithTravelDirection)
{
  const auto start = pose(0.0, 0.0, M_PI);
  const auto goal = pose(8.0, 0.0, M_PI_2);
  DirectCorridorParameters parameters;

  const auto path = make_direct_corridor_path(
    start, goal, parameters,
    [](const double, const double, const double) {return true;});

  EXPECT_FALSE(path.has_value());
}

TEST(RangerDirectCorridor, ShortTerminalManeuverStaysWithLattice)
{
  const auto start = pose(0.0, 0.0, M_PI);
  const auto goal = pose(0.5, 0.0, 0.0);
  DirectCorridorParameters parameters;

  const auto path = make_direct_corridor_path(
    start, goal, parameters,
    [](const double, const double, const double) {return true;});

  EXPECT_FALSE(path.has_value());
}

}  // namespace
}  // namespace robot_nav_config
