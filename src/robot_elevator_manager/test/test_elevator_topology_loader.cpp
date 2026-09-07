#include <gtest/gtest.h>

#include <string>

#include "robot_elevator_manager/elevator_topology_loader.hpp"

namespace robot_elevator_manager
{
namespace
{

std::string valid_yaml()
{
  return R"(
schema_version: 2
mock_ports_enabled: true
building_id: building_1
elevators:
  - elevator_id: elevator_west
    floors:
      - floor_id: F1
        map_id: map_f1
        poses:
          hall_call: f1_hall_call
          landing: f1_landing
          cabin: f1_cabin
      - floor_id: F2
        map_id: map_f2
        poses:
          hall_call: f2_hall_call
          landing: f2_landing
          cabin: f2_cabin
)";
}

std::string valid_legacy_yaml()
{
  return R"(
schema_version: 1
mock_ports_enabled: false
building_id: building_1
elevators:
  - elevator_id: elevator_west
    floors:
      - floor_id: F1
        map_id: map_f1
        poses:
          hall_call: f1_hall_call
          hall_wait: f1_hall_wait
          doorway: f1_doorway
          cabin: f1_cabin
          exit: f1_exit
        threshold:
          left: [0.0, -0.6]
          right: [0.0, 0.6]
          cabin_reference: [1.0, 0.0]
          clearance_m: 0.05
          jamb_clearance_m: 0.05
      - floor_id: F2
        map_id: map_f2
        poses:
          hall_call: f2_hall_call
          hall_wait: f2_hall_wait
          doorway: f2_doorway
          cabin: f2_cabin
          exit: f2_exit
        threshold:
          left: [0.0, -0.6]
          right: [0.0, 0.6]
          cabin_reference: [1.0, 0.0]
          clearance_m: 0.05
          jamb_clearance_m: 0.05
)";
}

std::string valid_reverse_entry_yaml()
{
  return R"(
schema_version: 3
mock_ports_enabled: false
building_id: building_1
elevators:
  - elevator_id: elevator_west
    floors:
      - floor_id: F1
        map_id: map_f1
        hall_call_panel_side: LEFT
        cabin_panel_side: RIGHT
        poses:
          hall_call: f1_hall_call
          landing: f1_landing
          cabin: f1_cabin
          cabin_panel: f1_cabin_panel
      - floor_id: F2
        map_id: map_f2
        hall_call_panel_side: RIGHT
        cabin_panel_side: LEFT
        poses:
          hall_call: f2_hall_call
          landing: f2_landing
          cabin: f2_cabin
          cabin_panel: f2_cabin_panel
)";
}

TEST(ElevatorTopologyLoader, ParsesValidatedCatalog)
{
  const auto result = parse_topology_yaml(valid_yaml());

  ASSERT_TRUE(result.ok());
  ASSERT_TRUE(result.catalog.has_value());
  EXPECT_EQ(result.catalog->building_id, "building_1");
  EXPECT_TRUE(result.catalog->mock_ports_enabled);
  ASSERT_EQ(result.catalog->elevators.size(), 1U);
  EXPECT_EQ(result.catalog->elevators.front().floors[1].map_id, "map_f2");
  EXPECT_EQ(result.catalog->elevators.front().schema_version, 2U);
}

TEST(ElevatorTopologyLoader, ParsesLegacyCatalogForHistoricalValidation)
{
  const auto result = parse_topology_yaml(valid_legacy_yaml());

  ASSERT_TRUE(result.ok());
  ASSERT_TRUE(result.catalog.has_value());
  EXPECT_EQ(result.catalog->elevators.front().schema_version, 1U);
}

TEST(ElevatorTopologyLoader, ParsesV3FourPosesAndBothPanelSides)
{
  const auto result = parse_topology_yaml(valid_reverse_entry_yaml());

  ASSERT_TRUE(result.ok());
  ASSERT_TRUE(result.catalog.has_value());
  const auto & topology = result.catalog->elevators.front();
  ASSERT_EQ(topology.schema_version, 3U);
  ASSERT_EQ(topology.floors.front().poses.size(), 4U);
  EXPECT_EQ(topology.floors.front().hall_call_panel_side, PanelSide::kLeft);
  EXPECT_EQ(topology.floors.front().cabin_panel_side, PanelSide::kRight);
  EXPECT_EQ(
    find_pose_id(topology.floors.front(), PoseRole::kCabinPanel).value_or(""),
    "f1_cabin_panel");
}

TEST(ElevatorTopologyLoader, AppliesHistoricalDefaultsToOmittedLegacyClearances)
{
  auto text = valid_legacy_yaml();
  const auto erase_all = [&text](const std::string & needle) {
      std::size_t offset = 0U;
      while ((offset = text.find(needle, offset)) != std::string::npos) {
        text.erase(offset, needle.size());
      }
    };
  erase_all("          clearance_m: 0.05\n");
  erase_all("          jamb_clearance_m: 0.05\n");

  const auto result = parse_topology_yaml(text);

  ASSERT_TRUE(result.ok());
  ASSERT_TRUE(result.catalog.has_value());
  const auto & threshold =
    result.catalog->elevators.front().floors.front().threshold;
  ASSERT_TRUE(threshold.has_value());
  EXPECT_DOUBLE_EQ(threshold->clearance_m, 0.05);
  EXPECT_DOUBLE_EQ(threshold->jamb_clearance_m, 0.05);
}

TEST(ElevatorTopologyLoader, RequiresExplicitMockPortPolicy)
{
  auto text = valid_yaml();
  const auto marker = text.find("mock_ports_enabled: true\n");
  ASSERT_NE(marker, std::string::npos);
  text.erase(marker, std::string("mock_ports_enabled: true\n").size());

  const auto result = parse_topology_yaml(text);

  EXPECT_FALSE(result.ok());
  EXPECT_FALSE(result.errors.empty());
}

TEST(ElevatorTopologyLoader, PreservesFailClosedMockPortPolicy)
{
  auto text = valid_yaml();
  text.replace(
    text.find("mock_ports_enabled: true"),
    std::string("mock_ports_enabled: true").size(),
    "mock_ports_enabled: false");

  const auto result = parse_topology_yaml(text);

  ASSERT_TRUE(result.ok());
  ASSERT_TRUE(result.catalog.has_value());
  EXPECT_FALSE(result.catalog->mock_ports_enabled);
}

TEST(ElevatorTopologyLoader, RejectsMissingMapId)
{
  auto text = valid_yaml();
  const auto marker = text.find("        map_id: map_f1\n");
  ASSERT_NE(marker, std::string::npos);
  text.erase(marker, std::string("        map_id: map_f1\n").size());

  const auto result = parse_topology_yaml(text);

  EXPECT_FALSE(result.ok());
  EXPECT_FALSE(result.errors.empty());
}

TEST(ElevatorTopologyLoader, RejectsUnknownPoseRole)
{
  auto text = valid_yaml();
  const auto marker = text.find("          hall_call: f1_hall_call");
  ASSERT_NE(marker, std::string::npos);
  text.replace(
    marker,
    std::string("          hall_call").size(),
    "          call_magic");

  const auto result = parse_topology_yaml(text);

  EXPECT_FALSE(result.ok());
  EXPECT_FALSE(result.errors.empty());
}

TEST(ElevatorTopologyLoader, RejectsDuplicateElevatorIds)
{
  auto text = valid_yaml();
  const auto floors = text.substr(text.find("    floors:"));
  text += "\n  - elevator_id: elevator_west\n" + floors;

  const auto result = parse_topology_yaml(text);

  EXPECT_FALSE(result.ok());
  EXPECT_FALSE(result.errors.empty());
}

TEST(ElevatorTopologyLoader, RejectsUnsupportedSchema)
{
  auto text = valid_yaml();
  text.replace(text.find("schema_version: 2"), 17U, "schema_version: 4");

  const auto result = parse_topology_yaml(text);

  EXPECT_FALSE(result.ok());
  EXPECT_FALSE(result.errors.empty());
}

TEST(ElevatorTopologyLoader, RejectsLegacyThresholdInV2)
{
  auto text = valid_yaml();
  const auto marker = text.find("          cabin: f1_cabin\n");
  ASSERT_NE(marker, std::string::npos);
  text.insert(
    marker + std::string("          cabin: f1_cabin\n").size(),
    "        threshold:\n"
    "          left: [0.0, -0.6]\n"
    "          right: [0.0, 0.6]\n"
    "          cabin_reference: [1.0, 0.0]\n"
    "          clearance_m: 0.05\n"
    "          jamb_clearance_m: 0.05\n");

  const auto result = parse_topology_yaml(text);

  EXPECT_FALSE(result.ok());
  EXPECT_FALSE(result.errors.empty());
}

}  // namespace
}  // namespace robot_elevator_manager
