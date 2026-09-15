#include <cmath>
#include <cstddef>
#include <memory>
#include <vector>

#include "gtest/gtest.h"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_nav_config/ordinary_local_path_repair.hpp"

namespace
{

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

geometry_msgs::msg::Quaternion quaternionFromYaw(const double yaw)
{
  geometry_msgs::msg::Quaternion orientation;
  orientation.z = std::sin(yaw * 0.5);
  orientation.w = std::cos(yaw * 0.5);
  return orientation;
}

nav_msgs::msg::Path straightReferencePath()
{
  nav_msgs::msg::Path path;
  path.header.frame_id = "odom";
  for (double x = -2.0; x <= 3.0001; x += 0.05) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = path.header.frame_id;
    pose.pose.position.x = x;
    pose.pose.orientation = quaternionFromYaw(0.0);
    path.poses.push_back(pose);
  }
  return path;
}

std::vector<geometry_msgs::msg::Point> rangerFootprintWithMppiMargin()
{
  std::vector<geometry_msgs::msg::Point> footprint(4U);
  footprint[0].x = 0.47;
  footprint[0].y = 0.36;
  footprint[1].x = 0.47;
  footprint[1].y = -0.36;
  footprint[2].x = -0.47;
  footprint[2].y = -0.36;
  footprint[3].x = -0.47;
  footprint[3].y = 0.36;
  return footprint;
}

void addPersonSizedBarrier(nav2_costmap_2d::Costmap2D & costmap)
{
  for (double x = 0.35; x <= 0.75; x += costmap.getResolution()) {
    for (double y = -0.55; y <= 0.55; y += costmap.getResolution()) {
      unsigned int mx = 0U;
      unsigned int my = 0U;
      ASSERT_TRUE(costmap.worldToMap(x, y, mx, my));
      costmap.setCost(mx, my, nav2_costmap_2d::LETHAL_OBSTACLE);
    }
  }
}

robot_nav_config::OrdinaryLocalPathRepairRequest makeRequest(
  const std::shared_ptr<nav2_costmap_2d::Costmap2D> & costmap)
{
  robot_nav_config::OrdinaryLocalPathRepairRequest request;
  request.costmap = costmap;
  request.costmap_frame = "odom";
  request.footprint = rangerFootprintWithMppiMargin();
  request.start.x = -2.0;
  request.start.y = 0.0;
  request.start.yaw = 0.0;
  request.reference_path = straightReferencePath();
  request.parameters.minimum_turning_radius_m = 0.81;
  request.parameters.lookahead_distance_m = 4.0;
  request.parameters.rejoin_distance_after_blockage_m = 0.80;
  request.parameters.max_planning_time_sec = 0.40;
  request.parameters.max_iterations = 60000;
  request.parameters.angle_quantization_bins = 72U;
  return request;
}

}  // namespace

TEST(OrdinaryLocalPathRepair, RejoinsPastPersistentLocalObstacleWithAckermannPath)
{
  auto node = std::make_shared<nav2_util::LifecycleNode>("ordinary_local_repair_test");
  auto costmap = std::make_shared<nav2_costmap_2d::Costmap2D>(
    160U, 120U, 0.05, -4.0, -3.0, nav2_costmap_2d::FREE_SPACE);
  addPersonSizedBarrier(*costmap);

  const auto result = robot_nav_config::repair_ordinary_local_path(makeRequest(costmap), node);

  ASSERT_EQ(result.status, robot_nav_config::OrdinaryLocalPathRepairStatus::kSuccess);
  ASSERT_GT(result.path.poses.size(), 2U);
  EXPECT_GT(result.rejoin_index, result.first_blocked_index);
  EXPECT_GT(result.maximum_lateral_deviation_m, 0.70);
  EXPECT_NEAR(result.path.poses.back().pose.position.x, 3.0, 1.0e-6);
  EXPECT_NEAR(result.path.poses.back().pose.position.y, 0.0, 1.0e-6);

  // Every lethal-cell center must remain outside the expanded rectangular
  // footprint at every returned pose. This catches point-robot detours that
  // would still collide with the physical Ranger envelope plus MPPI margin.
  for (const auto & pose : result.path.poses) {
    const double yaw = 2.0 * std::atan2(
      pose.pose.orientation.z, pose.pose.orientation.w);
    for (unsigned int my = 0U; my < costmap->getSizeInCellsY(); ++my) {
      for (unsigned int mx = 0U; mx < costmap->getSizeInCellsX(); ++mx) {
        if (costmap->getCost(mx, my) != nav2_costmap_2d::LETHAL_OBSTACLE) {
          continue;
        }
        double obstacle_x = 0.0;
        double obstacle_y = 0.0;
        costmap->mapToWorld(mx, my, obstacle_x, obstacle_y);
        const double dx = obstacle_x - pose.pose.position.x;
        const double dy = obstacle_y - pose.pose.position.y;
        const double body_x = std::cos(yaw) * dx + std::sin(yaw) * dy;
        const double body_y = -std::sin(yaw) * dx + std::cos(yaw) * dy;
        EXPECT_TRUE(std::abs(body_x) > 0.47 || std::abs(body_y) > 0.36)
          << "collision at path pose (" << pose.pose.position.x << ", "
          << pose.pose.position.y << ") with obstacle cell (" << obstacle_x
          << ", " << obstacle_y << ")";
      }
    }
  }
}

TEST(OrdinaryLocalPathRepair, LeavesClearReferencePathUntouched)
{
  auto node = std::make_shared<nav2_util::LifecycleNode>("ordinary_local_clear_test");
  auto costmap = std::make_shared<nav2_costmap_2d::Costmap2D>(
    160U, 120U, 0.05, -4.0, -3.0, nav2_costmap_2d::FREE_SPACE);

  const auto result = robot_nav_config::repair_ordinary_local_path(makeRequest(costmap), node);

  EXPECT_EQ(result.status, robot_nav_config::OrdinaryLocalPathRepairStatus::kPathClear);
  EXPECT_TRUE(result.path.poses.empty());
}

TEST(OrdinaryLocalPathRepair, PreferenceDetectsPathOutsideLegacyHardEnvelope)
{
  auto node = std::make_shared<nav2_util::LifecycleNode>("preferred_inspection_test");
  auto map = std::make_shared<nav2_costmap_2d::Costmap2D>(
    200U, 200U, 0.05, -5.0, -5.0, nav2_costmap_2d::FREE_SPACE);
  auto request = makeRequest(map);
  // Cell occupies y=[0.40, 0.45]: outside hard y=0.36, inside preference y=0.41.
  for (unsigned int x = 94; x <= 104; ++x) {
    map->setCost(x, 108, nav2_costmap_2d::LETHAL_OBSTACLE);
  }
  EXPECT_TRUE(robot_nav_config::inspect_ordinary_local_path(request, node).path_clear);
  request.preferred_footprint = request.footprint;
  nav2_costmap_2d::padFootprint(request.preferred_footprint, 0.05);
  EXPECT_FALSE(robot_nav_config::inspect_ordinary_local_path(request, node).path_clear);
  const auto result = robot_nav_config::repair_ordinary_local_path(request, node);
  ASSERT_EQ(result.status, robot_nav_config::OrdinaryLocalPathRepairStatus::kSuccess);
  EXPECT_TRUE(result.used_preferred_clearance);
  EXPECT_EQ(result.path.poses.back(), request.reference_path.poses.back());
  auto check = request;
  check.reference_path = result.path;
  EXPECT_TRUE(robot_nav_config::inspect_ordinary_local_path(check, node).path_clear);
}

TEST(OrdinaryLocalPathRepair, StartInsidePreferenceKeepsLegacyClearPathAndAttachment)
{
  auto node = std::make_shared<nav2_util::LifecycleNode>("preferred_escape_test");
  auto map = std::make_shared<nav2_costmap_2d::Costmap2D>(
    200U, 200U, 0.05, -5.0, -5.0, nav2_costmap_2d::FREE_SPACE);
  auto request = makeRequest(map);
  for (unsigned int x = 52; x <= 64; ++x) {
    map->setCost(x, 108, nav2_costmap_2d::LETHAL_OBSTACLE);
  }
  request.preferred_footprint = request.footprint;
  nav2_costmap_2d::padFootprint(request.preferred_footprint, 0.05);
  const auto result = robot_nav_config::repair_ordinary_local_path(request, node);
  EXPECT_EQ(result.status, robot_nav_config::OrdinaryLocalPathRepairStatus::kPathClear);
  EXPECT_FALSE(result.used_preferred_clearance);
  EXPECT_TRUE(robot_nav_config::attach_ordinary_local_path(request, node).has_value());
}

TEST(OrdinaryLocalPathRepair, RejoinsBeyondInspectionWindowWhenLocalMapCoversIt)
{
  auto node = std::make_shared<nav2_util::LifecycleNode>("rejoin_window_test");
  auto map = std::make_shared<nav2_costmap_2d::Costmap2D>(
    200U, 200U, 0.05, -5.0, -5.0, nav2_costmap_2d::FREE_SPACE);
  auto request = makeRequest(map);
  request.parameters.lookahead_distance_m = 4.0;
  for (int i = 1; i <= 20; ++i) {
    auto p = request.reference_path.poses.back();
    p.pose.position.x = 3.0 + 0.05 * i;
    request.reference_path.poses.push_back(p);
  }
  for (double x = 2.25; x < 2.46; x += 0.05) {
    for (double y = -0.30; y < 0.31; y += 0.05) {
      unsigned int mx, my;
      ASSERT_TRUE(map->worldToMap(x, y, mx, my));
      map->setCost(mx, my, nav2_costmap_2d::LETHAL_OBSTACLE);
    }
  }
  const auto result = robot_nav_config::repair_ordinary_local_path(request, node);
  ASSERT_EQ(result.status, robot_nav_config::OrdinaryLocalPathRepairStatus::kSuccess);
  EXPECT_GT(request.reference_path.poses[result.rejoin_index].pose.position.x -
    request.start.x, 4.0);
  EXPECT_EQ(result.path.poses.back(), request.reference_path.poses.back());
}

TEST(OrdinaryLocalPathRepair, TriesIndependentExitAfterUnreachableFarCandidate)
{
  auto node = std::make_shared<nav2_util::LifecycleNode>("rejoin_retry_test");
  auto map = std::make_shared<nav2_costmap_2d::Costmap2D>(
    200U, 200U, 0.05, -5.0, -5.0, nav2_costmap_2d::FREE_SPACE);
  for (unsigned int mx = 95; mx <= 103; ++mx) {
    for (unsigned int my = 96; my <= 104; ++my) {
      map->setCost(mx, my, nav2_costmap_2d::LETHAL_OBSTACLE);
    }
  }
  // Far exit is individually clear but enclosed; another exit before the
  // enclosure is reachable. Three adjacent far poses are not three exits.
  for (unsigned int mx = 148; mx <= 178; ++mx) {
    for (unsigned int my = 84; my <= 116; ++my) {
      if (mx == 148 || mx == 178 || my == 84 || my == 116) {
        map->setCost(mx, my, nav2_costmap_2d::LETHAL_OBSTACLE);
      }
    }
  }
  const auto request = makeRequest(map);
  const auto result = robot_nav_config::repair_ordinary_local_path(request, node);
  ASSERT_EQ(result.status, robot_nav_config::OrdinaryLocalPathRepairStatus::kSuccess)
    << "attempt expansions: " << ::testing::PrintToString(result.attempt_iterations);
  EXPECT_LT(request.reference_path.poses[result.rejoin_index].pose.position.x, 2.0);
  ASSERT_GE(result.attempt_iterations.size(), 2U);
  EXPECT_GT(result.attempt_iterations[0], 0);
  EXPECT_GT(result.attempt_iterations[1], 0);
  EXPECT_EQ(result.path.poses.back(), request.reference_path.poses.back());
}

TEST(OrdinaryLocalPathRepair, MovingRobotReattachesFromLatestPoseWithOriginalGoal)
{
  auto node = std::make_shared<nav2_util::LifecycleNode>("attachment_test");
  auto map = std::make_shared<nav2_costmap_2d::Costmap2D>(
    200U, 200U, 0.05, -5.0, -5.0, nav2_costmap_2d::FREE_SPACE);
  auto request = makeRequest(map);
  request.start = {-1.7, 0.08, 0.04};
  const auto attached = robot_nav_config::attach_ordinary_local_path(request, node);
  ASSERT_TRUE(attached.has_value());
  EXPECT_NEAR(attached->poses.front().pose.position.x, request.start.x, 1e-9);
  EXPECT_NEAR(attached->poses.front().pose.position.y, request.start.y, 1e-9);
  EXPECT_EQ(attached->poses.back(), request.reference_path.poses.back());
}

TEST(OrdinaryLocalPathRepair, RejectsCurrentFootprintCollisionSkippedByNearestPathInspection)
{
  auto node = std::make_shared<nav2_util::LifecycleNode>("attachment_blocked_test");
  auto map = std::make_shared<nav2_costmap_2d::Costmap2D>(
    200U, 200U, 0.05, -5.0, -5.0, nav2_costmap_2d::FREE_SPACE);
  auto request = makeRequest(map);
  request.start = {-1.7, 0.6, 0.0};
  // Lethal strip touches the actual footprint, but not the y=0 old reference.
  for (unsigned int mx = 62; mx <= 80; ++mx) {
    map->setCost(mx, 112, nav2_costmap_2d::LETHAL_OBSTACLE);
  }
  EXPECT_FALSE(robot_nav_config::attach_ordinary_local_path(request, node).has_value());
}
