#include <gtest/gtest.h>

#include <algorithm>
#include <optional>
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
      {PoseRole::kLanding, prefix + "_landing"},
      {PoseRole::kCabin, prefix + "_cabin"},
    },
    std::nullopt,
  };
}

FloorElevatorTopology make_legacy_floor(
  const std::string & floor_id,
  const std::string & prefix)
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
    DoorThreshold{{0.0, -0.6}, {0.0, 0.6}, {1.0, 0.0}, 0.05, 0.05},
  };
}

FloorElevatorTopology make_reverse_entry_floor(
  const std::string & floor_id,
  const std::string & prefix,
  const PanelSide hall_call_panel_side = PanelSide::kLeft,
  const PanelSide cabin_panel_side = PanelSide::kRight)
{
  return FloorElevatorTopology{
    floor_id,
    floor_id + "_map",
    {
      {PoseRole::kHallCall, prefix + "_hall_call"},
      {PoseRole::kLanding, prefix + "_landing"},
      {PoseRole::kCabin, prefix + "_cabin"},
      {PoseRole::kCabinPanel, prefix + "_cabin_panel"},
    },
    std::nullopt,
    hall_call_panel_side,
    cabin_panel_side,
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
    2U,
  };
}

ElevatorTopology make_legacy_topology()
{
  return ElevatorTopology{
    "elevator_west",
    "building_1",
    {
      make_legacy_floor("F1", "f1_west"),
      make_legacy_floor("F2", "f2_west"),
    },
    1U,
  };
}

ElevatorTopology make_reverse_entry_topology()
{
  return ElevatorTopology{
    "elevator_west",
    "building_1",
    {
      make_reverse_entry_floor("F1", "f1_west"),
      make_reverse_entry_floor("F2", "f2_west"),
    },
    3U,
  };
}

bool contains_issue(const TopologyValidation & result, const TopologyIssueCode code)
{
  return std::any_of(result.issues.begin(), result.issues.end(), [code](const TopologyIssue & issue) {
    return issue.code == code;
  });
}

TEST(ElevatorTopology, AcceptsSafeCompleteV2ThreePoseTopology)
{
  const auto result = validate_topology(make_topology());

  EXPECT_TRUE(result.ok());
  EXPECT_TRUE(result.issues.empty());
  EXPECT_EQ(required_pose_roles(2U).size(), 3U);
}

TEST(ElevatorTopology, AcceptsCompleteLegacyTopologyForHistoricalValidation)
{
  const auto result = validate_topology(make_legacy_topology());

  EXPECT_TRUE(result.ok());
  EXPECT_EQ(required_pose_roles(1U).size(), 5U);
}

TEST(ElevatorTopology, AcceptsV3ReverseEntryFourPoseTopologyWithPanelSides)
{
  const auto result = validate_topology(make_reverse_entry_topology());

  EXPECT_TRUE(result.ok());
  EXPECT_TRUE(result.issues.empty());
  EXPECT_EQ(required_pose_roles(3U).size(), 4U);
  EXPECT_EQ(to_string(PanelSide::kLeft), "LEFT");
  EXPECT_EQ(to_string(PanelSide::kRight), "RIGHT");

  const auto route = resolve_route(
    make_reverse_entry_topology(), "F1", "F1_map", "F2", "F2_map");
  ASSERT_TRUE(route.ok());
  ASSERT_TRUE(route.route.has_value());
  EXPECT_EQ(route.route->schema_version, 3U);
  EXPECT_EQ(route.route->source.hall_call_panel_side, PanelSide::kLeft);
  EXPECT_EQ(route.route->source.cabin_panel_side, PanelSide::kRight);
  EXPECT_EQ(
    find_pose_id(route.route->source, PoseRole::kCabinPanel).value_or(""),
    "f1_west_cabin_panel");
}

TEST(ElevatorTopology, RejectsV3MissingPanelSideAndCabinPanelPose)
{
  auto topology = make_reverse_entry_topology();
  topology.floors.front().hall_call_panel_side = PanelSide::kUnknown;
  topology.floors.front().poses.pop_back();

  const auto result = validate_topology(topology);

  EXPECT_FALSE(result.ok());
  EXPECT_TRUE(contains_issue(result, TopologyIssueCode::kMissingPanelSide));
  EXPECT_TRUE(contains_issue(result, TopologyIssueCode::kMissingPoseRole));
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
    {PoseRole::kLanding, "f2_west_second_landing"});

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
  auto topology = make_legacy_topology();
  topology.floors.back().floor_id = topology.floors.front().floor_id;
  topology.floors.front().threshold->right = topology.floors.front().threshold->left;
  topology.floors.back().threshold->cabin_reference = {0.0, 0.0};

  const auto result = validate_topology(topology);

  EXPECT_FALSE(result.ok());
  EXPECT_TRUE(contains_issue(result, TopologyIssueCode::kDuplicateFloor));
  EXPECT_TRUE(contains_issue(result, TopologyIssueCode::kInvalidThreshold));
}

TEST(ElevatorTopology, RejectsLegacyThresholdInV2)
{
  auto topology = make_topology();
  topology.floors.front().threshold =
    DoorThreshold{{0.0, -0.6}, {0.0, 0.6}, {1.0, 0.0}, 0.05, 0.05};

  const auto result = validate_topology(topology);

  EXPECT_FALSE(result.ok());
  EXPECT_TRUE(contains_issue(result, TopologyIssueCode::kInvalidThreshold));
}

TEST(ElevatorTopology, ResolvesOnlyDistinctKnownFloors)
{
  const auto topology = make_topology();

  const auto route = resolve_route(topology, "F1", "F1_map", "F2", "F2_map");
  ASSERT_TRUE(route.ok());
  ASSERT_TRUE(route.route.has_value());
  EXPECT_EQ(route.route->schema_version, 2U);
  EXPECT_EQ(route.route->source.floor_id, "F1");
  EXPECT_EQ(route.route->source.map_id, "F1_map");
  EXPECT_EQ(route.route->target.floor_id, "F2");
  EXPECT_EQ(route.route->target.map_id, "F2_map");
  EXPECT_EQ(
    find_pose_id(route.route->source, PoseRole::kHallCall).value_or(""),
    "f1_west_hall_call");
  EXPECT_EQ(
    find_pose_id(route.route->target, PoseRole::kLanding).value_or(""),
    "f2_west_landing");

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

TEST(ElevatorTopology, PreservesLegacySchemaOnResolvedReadOnlyRoute)
{
  const auto route =
    resolve_route(make_legacy_topology(), "F1", "F1_map", "F2", "F2_map");

  ASSERT_TRUE(route.ok());
  ASSERT_TRUE(route.route.has_value());
  EXPECT_EQ(route.route->schema_version, 1U);
}

}  // namespace
}  // namespace robot_elevator_manager
