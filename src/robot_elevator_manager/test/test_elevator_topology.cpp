#include <gtest/gtest.h>

#include <algorithm>
#include <string>

#include "robot_elevator_manager/elevator_topology.hpp"

namespace robot_elevator_manager
{
namespace
{

FloorElevatorTopology make_floor(const std::string & floor_id, const std::string & prefix)
{
  return FloorElevatorTopology{
    floor_id,
    floor_id + "_map",
    {
      {PoseRole::kHallCall, prefix + "_hall_call"},
      {PoseRole::kHallWait, prefix + "_hall_wait"},
      {PoseRole::kDoorway, prefix + "_doorway"},
      {PoseRole::kCabin, prefix + "_cabin"},
      {PoseRole::kExit, prefix + "_exit"},
    },
    {{0.0, -0.6}, {0.0, 0.6}, {1.0, 0.0}, 0.05},
  };
}

ElevatorTopology make_topology()
{
  return ElevatorTopology{
    "elevator_west",
    "building_1",
    {
      make_floor("F1", "f1_west"),
      make_floor("F2", "f2_west"),
    },
  };
}

bool contains_issue(const TopologyValidation & result, const TopologyIssueCode code)
{
  return std::any_of(result.issues.begin(), result.issues.end(), [code](const TopologyIssue & issue) {
    return issue.code == code;
  });
}

TEST(ElevatorTopology, AcceptsSafeCompleteTwoFloorTopology)
{
  const auto result = validate_topology(make_topology());

  EXPECT_TRUE(result.ok());
  EXPECT_TRUE(result.issues.empty());
}

TEST(ElevatorTopology, RejectsTraversalAndWhitespaceIdentifiers)
{
  auto topology = make_topology();
  topology.elevator_id = "../west";
  topology.building_id = "building one";
  topology.floors.front().floor_id = "F1/../../etc";
  topology.floors.front().map_id = "../map";
  topology.floors.front().poses.front().pose_id = "pose\\escape";

  const auto result = validate_topology(topology);

  EXPECT_FALSE(result.ok());
  EXPECT_TRUE(contains_issue(result, TopologyIssueCode::kUnsafeElevatorId));
  EXPECT_TRUE(contains_issue(result, TopologyIssueCode::kUnsafeBuildingId));
  EXPECT_TRUE(contains_issue(result, TopologyIssueCode::kUnsafeFloorId));
  EXPECT_TRUE(contains_issue(result, TopologyIssueCode::kUnsafeMapId));
  EXPECT_TRUE(contains_issue(result, TopologyIssueCode::kUnsafePoseId));
}

TEST(ElevatorTopology, RejectsDotDirectoryAlias)
{
  auto topology = make_topology();
  topology.floors.front().map_id = ".";

  const auto result = validate_topology(topology);

  EXPECT_FALSE(result.ok());
  EXPECT_TRUE(contains_issue(result, TopologyIssueCode::kUnsafeMapId));
}

TEST(ElevatorTopology, RejectsMissingAndDuplicatePoseRoles)
{
  auto topology = make_topology();
  topology.floors.front().poses.erase(topology.floors.front().poses.begin());
  topology.floors.back().poses.push_back(
    {PoseRole::kHallWait, "f2_west_second_hall_wait"});

  const auto result = validate_topology(topology);

  EXPECT_FALSE(result.ok());
  EXPECT_TRUE(contains_issue(result, TopologyIssueCode::kMissingPoseRole));
  EXPECT_TRUE(contains_issue(result, TopologyIssueCode::kDuplicatePoseRole));
}

TEST(ElevatorTopology, RejectsPoseReuseWithinOneFloor)
{
  auto topology = make_topology();
  topology.floors.front().poses[1].pose_id = topology.floors.front().poses[0].pose_id;

  const auto result = validate_topology(topology);

  EXPECT_FALSE(result.ok());
  EXPECT_TRUE(contains_issue(result, TopologyIssueCode::kDuplicatePoseId));
}

TEST(ElevatorTopology, RejectsUnknownPoseRoleValues)
{
  auto topology = make_topology();
  topology.floors.front().poses.front().role = static_cast<PoseRole>(99);

  const auto result = validate_topology(topology);

  EXPECT_FALSE(result.ok());
  EXPECT_TRUE(contains_issue(result, TopologyIssueCode::kUnknownPoseRole));
  EXPECT_TRUE(contains_issue(result, TopologyIssueCode::kMissingPoseRole));
}

TEST(ElevatorTopology, RejectsDuplicateFloorsAndInvalidThresholdGeometry)
{
  auto topology = make_topology();
  topology.floors.back().floor_id = topology.floors.front().floor_id;
  topology.floors.front().threshold.right = topology.floors.front().threshold.left;
  topology.floors.back().threshold.cabin_reference = {0.0, 0.0};

  const auto result = validate_topology(topology);

  EXPECT_FALSE(result.ok());
  EXPECT_TRUE(contains_issue(result, TopologyIssueCode::kDuplicateFloor));
  EXPECT_TRUE(contains_issue(result, TopologyIssueCode::kInvalidThreshold));
}

TEST(ElevatorTopology, ResolvesOnlyDistinctKnownFloors)
{
  const auto topology = make_topology();

  const auto route = resolve_route(topology, "F1", "F1_map", "F2", "F2_map");
  ASSERT_TRUE(route.ok());
  ASSERT_TRUE(route.route.has_value());
  EXPECT_EQ(route.route->source.floor_id, "F1");
  EXPECT_EQ(route.route->source.map_id, "F1_map");
  EXPECT_EQ(route.route->target.floor_id, "F2");
  EXPECT_EQ(route.route->target.map_id, "F2_map");
  EXPECT_EQ(
    find_pose_id(route.route->source, PoseRole::kHallCall).value_or(""),
    "f1_west_hall_call");
  EXPECT_EQ(
    find_pose_id(route.route->target, PoseRole::kExit).value_or(""),
    "f2_west_exit");

  const auto same_floor = resolve_route(topology, "F1", "F1_map", "F1", "F1_map");
  EXPECT_FALSE(same_floor.ok());
  EXPECT_EQ(same_floor.error, RouteError::kSameFloor);

  const auto missing_floor =
    resolve_route(topology, "F1", "F1_map", "F9", "F9_map");
  EXPECT_FALSE(missing_floor.ok());
  EXPECT_EQ(missing_floor.error, RouteError::kUnknownFloor);

  const auto wrong_map =
    resolve_route(topology, "F1", "stale_map", "F2", "F2_map");
  EXPECT_FALSE(wrong_map.ok());
  EXPECT_EQ(wrong_map.error, RouteError::kUnknownFloor);
}

}  // namespace
}  // namespace robot_elevator_manager
