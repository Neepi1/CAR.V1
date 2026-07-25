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
schema_version: 1
mock_ports_enabled: true
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

TEST(ElevatorTopologyLoader, ParsesValidatedCatalog)
{
  const auto result = parse_topology_yaml(valid_yaml());

  ASSERT_TRUE(result.ok());
  ASSERT_TRUE(result.catalog.has_value());
  EXPECT_EQ(result.catalog->building_id, "building_1");
  EXPECT_TRUE(result.catalog->mock_ports_enabled);
  ASSERT_EQ(result.catalog->elevators.size(), 1U);
  EXPECT_EQ(result.catalog->elevators.front().floors[1].map_id, "map_f2");
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
  text.replace(text.find("schema_version: 1"), 17U, "schema_version: 2");

  const auto result = parse_topology_yaml(text);

  EXPECT_FALSE(result.ok());
  EXPECT_FALSE(result.errors.empty());
}

}  // namespace
}  // namespace robot_elevator_manager
