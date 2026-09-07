#include <algorithm>
#include <cerrno>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "gtest/gtest.h"
#include <yaml-cpp/yaml.h>

#include "robot_api_server/features/elevator/configuration/elevator_configuration_module.hpp"
#include "robot_api_server/features/maps/catalog_activation/file_utils.hpp"
#include "robot_api_server/infrastructure/http/http_common.hpp"
#include "robot_elevator_manager/elevator_release_loader.hpp"
#include "robot_elevator_manager/elevator_topology_loader.hpp"

namespace robot_api_server
{
namespace
{

namespace fs = std::filesystem;

std::string canonical_test_digest(const char digit)
{
  return "sha256:" + std::string(64U, digit);
}

std::string canonical_scalar_json(const std::string & value)
{
  if (value == "null" || value == "Null" || value == "NULL" || value == "~") {
    return "null";
  }
  if (value == "true" || value == "True" || value == "TRUE") {
    return "true";
  }
  if (value == "false" || value == "False" || value == "FALSE") {
    return "false";
  }
  const bool unsigned_integer =
    !value.empty() &&
    (value == "0" ||
    (value.front() >= '1' && value.front() <= '9' &&
    std::all_of(
      value.begin() + 1, value.end(),
      [](const unsigned char character) {
        return std::isdigit(character) != 0;
      })));
  const bool negative_integer =
    value.size() > 1U && value.front() == '-' &&
    value[1] >= '1' && value[1] <= '9' &&
    std::all_of(
    value.begin() + 2, value.end(),
    [](const unsigned char character) {
      return std::isdigit(character) != 0;
    });
  if (unsigned_integer || negative_integer) {
    return value;
  }
  char * end = nullptr;
  errno = 0;
  const double number = std::strtod(value.c_str(), &end);
  if (
    errno == 0 && end == value.c_str() + value.size() &&
    std::isfinite(number))
  {
    std::ostringstream normalized;
    normalized << std::setprecision(std::numeric_limits<double>::max_digits10)
               << number;
    return normalized.str();
  }
  return json_string(value);
}

std::string canonical_node_json(const YAML::Node & node)
{
  if (!node || node.IsNull()) {
    return "null";
  }
  if (node.IsScalar()) {
    const auto tag = node.Tag();
    return (tag == "!" || tag == "tag:yaml.org,2002:str") ?
      json_string(node.Scalar()) : canonical_scalar_json(node.Scalar());
  }
  if (node.IsSequence()) {
    std::ostringstream output;
    output << "[";
    for (std::size_t index = 0U; index < node.size(); ++index) {
      if (index > 0U) {
        output << ",";
      }
      output << canonical_node_json(node[index]);
    }
    output << "]";
    return output.str();
  }
  if (node.IsMap()) {
    std::vector<std::pair<std::string, std::string>> entries;
    for (const auto & entry : node) {
      entries.emplace_back(
        entry.first.as<std::string>(), canonical_node_json(entry.second));
    }
    std::sort(entries.begin(), entries.end());
    std::ostringstream output;
    output << "{";
    for (std::size_t index = 0U; index < entries.size(); ++index) {
      if (index > 0U) {
        output << ",";
      }
      output << json_string(entries[index].first) << ":" <<
        entries[index].second;
    }
    output << "}";
    return output.str();
  }
  return "null";
}

std::uint64_t test_fnv1a64(const std::string & value)
{
  std::uint64_t hash = 1469598103934665603ULL;
  for (const auto character : value) {
    hash ^= static_cast<unsigned char>(character);
    hash *= 1099511628211ULL;
  }
  return hash;
}

std::string fixed_test_hex(const std::uint64_t value)
{
  std::ostringstream output;
  output << std::hex << std::nouppercase << std::setw(16) <<
    std::setfill('0') << value;
  return output.str();
}

class TemporaryDirectory
{
public:
  TemporaryDirectory()
  {
    root_ = fs::temp_directory_path() /
      ("njrh_elevator_config_test_" + std::to_string(
        std::chrono::steady_clock::now().time_since_epoch().count()));
    fs::create_directories(root_);
  }

  ~TemporaryDirectory()
  {
    std::error_code error;
    fs::remove_all(root_, error);
  }

  const fs::path & path() const
  {
    return root_;
  }

private:
  fs::path root_;
};

std::string valid_document(const double f4_landing_x = -0.6)
{
  return R"json({
    "schema_version": 2,
    "building_id": "B3",
    "elevators": [{
      "elevator_id": "elevator_1",
      "display_name": "1号电梯",
      "floors": [{
        "floor_id": "F3",
        "map_id": "map_f3",
        "poses": {
          "hall_call": {"x": -1.0, "y": 0.0, "yaw": 0.0},
          "landing": {"x": -0.6, "y": 0.0, "yaw": 0.0},
          "cabin": {"x": 0.8, "y": 0.0, "yaw": 3.1415926}
        }
      }, {
        "floor_id": "F4",
        "map_id": "map_f4",
        "poses": {
          "hall_call": {"x": -1.0, "y": 0.0, "yaw": 0.0},
          "landing": {"x": )json" + std::to_string(f4_landing_x) + R"json(, "y": 0.0, "yaw": 0.0},
          "cabin": {"x": 0.8, "y": 0.0, "yaw": 3.1415926}
        }
      }]
    }]
  })json";
}

std::string valid_reverse_entry_document()
{
  return R"json({
    "schema_version": 3,
    "building_id": "B3",
    "elevators": [{
      "elevator_id": "elevator_1",
      "display_name": "1号电梯",
      "floors": [{
        "floor_id": "F3",
        "map_id": "map_f3",
        "hall_call_panel_side": "LEFT",
        "cabin_panel_side": "LEFT",
        "poses": {
          "hall_call": {"x": -1.0, "y": 0.0, "yaw": 0.0},
          "landing": {"x": -0.6, "y": 0.0, "yaw": 3.141592653589793},
          "cabin": {"x": 0.8, "y": 0.0, "yaw": 3.141592653589793},
          "cabin_panel": {"x": 0.8, "y": -0.5, "yaw": 3.141592653589793}
        }
      }, {
        "floor_id": "F4",
        "map_id": "map_f4",
        "hall_call_panel_side": "RIGHT",
        "cabin_panel_side": "RIGHT",
        "poses": {
          "hall_call": {"x": -1.0, "y": 0.0, "yaw": 0.0},
          "landing": {"x": -0.6, "y": 0.0, "yaw": 3.141592653589793},
          "cabin": {"x": 0.8, "y": 0.0, "yaw": 3.141592653589793},
          "cabin_panel": {"x": 0.8, "y": 0.5, "yaw": 3.141592653589793}
        }
      }]
    }]
  })json";
}

std::string legacy_v1_document()
{
  return R"json({
    "schema_version": 1,
    "building_id": "B3",
    "elevators": [{
      "elevator_id": "elevator_1",
      "floors": [{
        "floor_id": "F3",
        "map_id": "map_f3",
        "poses": {
          "hall_call": {"x": -1.0, "y": 0.0, "yaw": 0.1},
          "hall_wait": {"x": -0.8, "y": 0.1, "yaw": 0.2},
          "doorway": {"x": -0.4, "y": 0.2, "yaw": 0.3},
          "cabin": {"x": 0.8, "y": 0.0, "yaw": 3.0},
          "exit": {"x": -0.2, "y": 0.3, "yaw": 0.4}
        },
        "threshold": {
          "left": [0.0, -0.6],
          "right": [0.0, 0.6],
          "cabin_reference": [1.0, 0.0],
          "clearance_m": 0.05,
          "jamb_clearance_m": 0.05
        }
      }, {
        "floor_id": "F4",
        "map_id": "map_f4",
        "poses": {
          "hall_call": {"x": -1.0, "y": 0.0, "yaw": 0.1},
          "hall_wait": {"x": -0.8, "y": 0.1, "yaw": 0.2},
          "doorway": {"x": -0.4, "y": 0.2, "yaw": 0.3},
          "cabin": {"x": 0.8, "y": 0.0, "yaw": 3.0},
          "exit": {"x": -0.2, "y": 0.3, "yaw": 0.4}
        },
        "threshold": {
          "left": [0.0, -0.6],
          "right": [0.0, 0.6],
          "cabin_reference": [1.0, 0.0],
          "clearance_m": 0.05,
          "jamb_clearance_m": 0.05
        }
      }]
    }]
  })json";
}

std::string install_legacy_v1_draft(const fs::path & maps_root)
{
  const auto configuration =
    canonical_node_json(YAML::Load(legacy_v1_document())) + "\n";
  const std::string validation =
    "{\"valid_for_publish\":true,\"issues\":[]}\n";
  const auto revision = "draft-v1-" + fixed_test_hex(
    test_fnv1a64("B3\n" + configuration + "\n" + validation));
  const auto config_root = maps_root / "B3" / ".elevator_config";
  const auto draft_root = config_root / "drafts" / revision;
  fs::create_directories(draft_root);
  {
    std::ofstream output(draft_root / "configuration.yaml", std::ios::binary);
    output << configuration;
  }
  {
    std::ofstream output(draft_root / "validation.json", std::ios::binary);
    output << validation;
  }
  {
    std::ofstream output(config_root / "draft.json", std::ios::binary);
    output << "{\"schema_version\":1,\"draft_revision\":" <<
      json_string(revision) << "}\n";
  }
  return revision;
}

std::string install_mismatched_v2_revision_with_v1_content(
  const fs::path & maps_root)
{
  const auto configuration =
    canonical_node_json(YAML::Load(legacy_v1_document())) + "\n";
  const std::string validation =
    "{\"valid_for_publish\":true,\"issues\":[]}\n";
  const auto revision = "draft-v2-" + fixed_test_hex(
    test_fnv1a64("B3\n" + configuration + "\n" + validation));
  const auto config_root = maps_root / "B3" / ".elevator_config";
  const auto draft_root = config_root / "drafts" / revision;
  fs::create_directories(draft_root);
  {
    std::ofstream output(draft_root / "configuration.yaml", std::ios::binary);
    output << configuration;
  }
  {
    std::ofstream output(draft_root / "validation.json", std::ios::binary);
    output << validation;
  }
  {
    std::ofstream output(config_root / "draft.json", std::ios::binary);
    output << "{\"schema_version\":1,\"draft_revision\":"
           << json_string(revision) << "}\n";
  }
  return revision;
}

std::string install_legacy_v1_release(
  const fs::path & maps_root,
  const bool select_as_current)
{
  auto configuration_root = YAML::Load(legacy_v1_document());
  for (std::size_t floor_index = 0U; floor_index < 2U; ++floor_index) {
    auto floor = configuration_root["elevators"][0]["floors"][floor_index];
    floor["map_asset_epoch"] = floor_index == 0U ? 33U : 44U;
    floor["map_asset_digest"] =
      canonical_test_digest(floor_index == 0U ? '3' : '4');
  }
  const auto configuration =
    canonical_node_json(configuration_root) + "\n";
  const std::string topology =
    "schema_version: 1\n"
    "mock_ports_enabled: false\n"
    "building_id: B3\n"
    "elevators:\n"
    "  - elevator_id: elevator_1\n"
    "    floors:\n"
    "      - floor_id: F3\n"
    "        map_id: map_f3\n"
    "        poses:\n"
    "          hall_call: legacy_f3_hall_call\n"
    "          hall_wait: legacy_f3_hall_wait\n"
    "          doorway: legacy_f3_doorway\n"
    "          cabin: legacy_f3_cabin\n"
    "          exit: legacy_f3_exit\n"
    "        threshold:\n"
    "          left: [0.0, -0.6]\n"
    "          right: [0.0, 0.6]\n"
    "          cabin_reference: [1.0, 0.0]\n"
    "          clearance_m: 0.05\n"
    "          jamb_clearance_m: 0.05\n"
    "      - floor_id: F4\n"
    "        map_id: map_f4\n"
    "        poses:\n"
    "          hall_call: legacy_f4_hall_call\n"
    "          hall_wait: legacy_f4_hall_wait\n"
    "          doorway: legacy_f4_doorway\n"
    "          cabin: legacy_f4_cabin\n"
    "          exit: legacy_f4_exit\n"
    "        threshold:\n"
    "          left: [0.0, -0.6]\n"
    "          right: [0.0, 0.6]\n"
    "          cabin_reference: [1.0, 0.0]\n"
    "          clearance_m: 0.05\n"
    "          jamb_clearance_m: 0.05\n";
  struct LegacyPoseFixture
  {
    const char * role;
    double x;
    double y;
    double yaw;
  };
  const std::vector<LegacyPoseFixture> legacy_poses{
    {"hall_call", -1.0, 0.0, 0.1},
    {"hall_wait", -0.8, 0.1, 0.2},
    {"doorway", -0.4, 0.2, 0.3},
    {"cabin", 0.8, 0.0, 3.0},
    {"exit", -0.2, 0.3, 0.4},
  };
  std::ostringstream internal_poses_stream;
  internal_poses_stream
    << "schema_version: 1\n"
    << "building_id: B3\n"
    << "poses:\n";
  for (const auto & floor :
    std::vector<std::pair<std::string, std::string>>{
      {"F3", "f3"}, {"F4", "f4"}})
  {
    for (const auto & pose : legacy_poses) {
      internal_poses_stream
        << "  - pose_id: legacy_" << floor.second << "_" << pose.role << "\n"
        << "    type: elevator_internal\n"
        << "    elevator_id: elevator_1\n"
        << "    floor_id: " << floor.first << "\n"
        << "    map_id: map_" << floor.second << "\n"
        << "    role: " << pose.role << "\n"
        << "    x: " << pose.x << "\n"
        << "    y: " << pose.y << "\n"
        << "    yaw: " << pose.yaw << "\n";
    }
  }
  const auto internal_poses = internal_poses_stream.str();
  const std::string validation =
    "{\"valid_for_publish\":true,\"issues\":[]}\n";
  const auto content_digest = fixed_test_hex(
    test_fnv1a64(configuration + topology + internal_poses));
  const auto release_suffix = fixed_test_hex(
    test_fnv1a64(
      "1\n" + configuration + topology + internal_poses)).substr(0U, 12U);
  const auto release_id = "elevator-config-000001-" + release_suffix;
  const std::string actor_id = "legacy_commissioner";
  const std::string timestamp = "2026-01-01T00:00:00Z";
  std::ostringstream manifest;
  manifest
    << "{\"schema_version\":1,"
    << "\"release_id\":" << json_string(release_id) << ","
    << "\"parent_release_id\":null,"
    << "\"source_draft_revision\":\"draft-v1-0123456789abcdef\","
    << "\"generation\":1,"
    << "\"created_at\":" << json_string(timestamp) << ","
    << "\"actor_id\":" << json_string(actor_id) << ","
    << "\"rollback_of\":null,"
    << "\"configuration_digest_algorithm\":\"fnv1a64\","
    << "\"configuration_digest\":" << json_string(content_digest) << ","
    << "\"map_bindings\":["
    << "{\"floor_id\":\"F3\",\"map_id\":\"map_f3\",\"asset_epoch\":33,"
    << "\"asset_digest_contract\":\"njrh-map-asset-bundle-v1\","
    << "\"asset_digest_algorithm\":\"sha256\",\"asset_digest\":"
    << json_string(canonical_test_digest('3')) << "},"
    << "{\"floor_id\":\"F4\",\"map_id\":\"map_f4\",\"asset_epoch\":44,"
    << "\"asset_digest_contract\":\"njrh-map-asset-bundle-v1\","
    << "\"asset_digest_algorithm\":\"sha256\",\"asset_digest\":"
    << json_string(canonical_test_digest('4')) << "}],"
    << "\"asset_published\":true,\"runtime_applied\":false}\n";
  std::ostringstream current;
  current
    << "{\"schema_version\":1,"
    << "\"release_id\":" << json_string(release_id) << ","
    << "\"parent_release_id\":null,\"generation\":1,"
    << "\"published_at\":" << json_string(timestamp) << ","
    << "\"actor_id\":" << json_string(actor_id) << ","
    << "\"rollback_of\":null}\n";

  const auto config_root = maps_root / "B3" / ".elevator_config";
  const auto release_root = config_root / "releases" / release_id;
  fs::create_directories(release_root);
  const std::vector<std::pair<std::string, std::string>> files{
    {"configuration.yaml", configuration},
    {"elevators.yaml", topology},
    {"elevator_internal_poses.yaml", internal_poses},
    {"validation.json", validation},
    {"manifest.json", manifest.str()},
    {"current.json", current.str()},
  };
  for (const auto & file : files) {
    std::ofstream output(release_root / file.first, std::ios::binary);
    output << file.second;
  }
  if (select_as_current) {
#ifndef _WIN32
    fs::create_directory_symlink(
      fs::path("releases") / release_id, config_root / "current");
#else
    std::ofstream output(config_root / "current.json", std::ios::binary);
    output << current.str();
#endif
  }
  return release_id;
}

ElevatorFloorAssetResolver test_resolver()
{
  return [](
           const std::string & building_id,
           const std::string & floor_id,
           const std::string & map_id)
         -> std::optional<ElevatorFloorAssetSnapshot>
         {
           if (building_id != "B3" ||
             !((floor_id == "F3" && map_id == "map_f3") ||
             (floor_id == "F4" && map_id == "map_f4")))
           {
             return std::nullopt;
           }
           ElevatorFloorAssetSnapshot snapshot;
           snapshot.manifest.building_id = building_id;
           snapshot.manifest.floor_id = floor_id;
           snapshot.manifest.map_id = map_id;
           snapshot.manifest.active = true;
           snapshot.map_info = MapYamlInfo{
             200U, 200U, 0.05, {-5.0, -5.0, 0.0}};
           snapshot.asset_epoch =
             map_id == "map_f3" ? 33U : 44U;
           snapshot.asset_digest =
             canonical_test_digest(map_id == "map_f3" ? '3' : '4');
           snapshot.required_assets_complete = true;
           return snapshot;
         };
}

TEST(ElevatorConfigurationModule, SavesAndPublishesValidatedBuildingRelease)
{
  TemporaryDirectory temporary;
  ElevatorConfigurationModule module(temporary.path(), test_resolver());

  ElevatorConfigurationCommand save;
  save.type = ElevatorConfigurationCommandType::kSaveDraft;
  save.building_id = "B3";
  save.document = valid_document();
  save.actor_id = "commissioning_app";
  const auto saved = module.execute(save);

  ASSERT_EQ(saved.status, 200) << saved.body;
  EXPECT_EQ(saved.code, "DRAFT_SAVED");
  EXPECT_TRUE(json_bool_value(saved.body, "valid_for_publish", false));
  const auto draft_revision = json_string_value(saved.body, "draft_revision");
  ASSERT_TRUE(draft_revision.has_value());
  EXPECT_EQ(draft_revision->rfind("draft-v2-", 0U), 0U);
  EXPECT_FALSE(json_bool_value(saved.body, "migration_required", true));
  EXPECT_EQ(
    json_number_value(saved.body, "source_schema_version").value_or(0.0),
    2.0);
  EXPECT_NE(
    saved.body.find("\"configuration\":"),
    std::string::npos);
  EXPECT_NE(
    saved.body.find("\"map_asset_epoch\":33"),
    std::string::npos);
  EXPECT_NE(
    saved.body.find(
      "\"map_asset_digest\":\"" + canonical_test_digest('3') + "\""),
    std::string::npos);
  const auto saved_draft = module.query(
    ElevatorConfigurationQuery{"B3", "", "", ""});
  ASSERT_EQ(saved_draft.status, 200) << saved_draft.body;
  EXPECT_NE(
    saved_draft.body.find("\"map_asset_epoch\":33"),
    std::string::npos);
  EXPECT_NE(
    saved_draft.body.find(
      "\"map_asset_digest\":\"" + canonical_test_digest('3') + "\""),
    std::string::npos);

  ElevatorConfigurationCommand publish;
  publish.type = ElevatorConfigurationCommandType::kPublish;
  publish.building_id = "B3";
  publish.expected_draft_revision = *draft_revision;
  publish.actor_id = "commissioning_app";
  const auto published = module.execute(publish);

  ASSERT_EQ(published.status, 201) << published.body;
  EXPECT_EQ(published.code, "CONFIGURATION_PUBLISHED");
  EXPECT_TRUE(json_bool_value(published.body, "asset_published", false));
  EXPECT_FALSE(json_bool_value(published.body, "runtime_applied", true));
  EXPECT_EQ(
    json_number_value(
      published.body, "configuration_schema_version").value_or(0.0),
    2.0);
  const auto release_id = json_string_value(published.body, "release_id");
  ASSERT_TRUE(release_id.has_value());

  const auto building_root = temporary.path() / "B3";
  const auto release_root =
    building_root / ".elevator_config" / "releases" / *release_id;
  EXPECT_TRUE(fs::is_regular_file(release_root / "configuration.yaml"));
  EXPECT_TRUE(fs::is_regular_file(release_root / "elevators.yaml"));
  EXPECT_TRUE(fs::is_regular_file(release_root / "elevator_internal_poses.yaml"));
  EXPECT_TRUE(fs::is_regular_file(release_root / "validation.json"));
  EXPECT_TRUE(fs::is_regular_file(release_root / "manifest.json"));
  EXPECT_NE(
    read_text_file(release_root / "manifest.json").find(
      "\"asset_digest_algorithm\":\"sha256\""),
    std::string::npos);
  EXPECT_TRUE(fs::is_regular_file(building_root / "elevators.yaml"));
  EXPECT_TRUE(fs::is_regular_file(building_root / "elevator_internal_poses.yaml"));
#ifndef _WIN32
  const auto config_root = building_root / ".elevator_config";
  EXPECT_TRUE(fs::is_symlink(config_root / "current"));
  EXPECT_EQ(
    fs::read_symlink(config_root / "current"),
    fs::path("releases") / *release_id);
  EXPECT_TRUE(fs::is_symlink(config_root / "current.json"));
  EXPECT_TRUE(fs::is_symlink(building_root / "elevators.yaml"));
  EXPECT_TRUE(fs::is_symlink(building_root / "elevator_internal_poses.yaml"));
#endif

  const auto topology = robot_elevator_manager::parse_topology_yaml(
    read_text_file(release_root / "elevators.yaml"));
  ASSERT_TRUE(topology.ok());
  ASSERT_TRUE(topology.catalog.has_value());
  EXPECT_FALSE(topology.catalog->mock_ports_enabled);
  ASSERT_EQ(topology.catalog->elevators.size(), 1U);
  EXPECT_EQ(topology.catalog->elevators.front().floors.size(), 2U);
  EXPECT_EQ(topology.catalog->elevators.front().schema_version, 2U);
  for (const auto & floor : topology.catalog->elevators.front().floors) {
    EXPECT_EQ(floor.poses.size(), 3U);
    EXPECT_FALSE(floor.threshold.has_value());
  }

  EXPECT_FALSE(fs::exists(building_root / "F3" / "poses.yaml"));
  EXPECT_FALSE(fs::exists(building_root / "F4" / "poses.yaml"));

  robot_elevator_manager::ElevatorReleaseLoadRequest runtime_request;
  runtime_request.config_root =
    (building_root / ".elevator_config").string();
  runtime_request.building_id = "B3";
  runtime_request.source_floor_id = "F3";
  runtime_request.source_map_id = "map_f3";
  runtime_request.target_floor_id = "F4";
  runtime_request.target_map_id = "map_f4";
  runtime_request.preferred_elevator_id = "elevator_1";
  runtime_request.expected_release_id = *release_id;
  const auto runtime_release =
    robot_elevator_manager::load_elevator_release(runtime_request);
  ASSERT_TRUE(runtime_release.ok())
    << robot_elevator_manager::to_string(runtime_release.error)
    << ": " << runtime_release.message;
  ASSERT_TRUE(runtime_release.release.has_value());
  EXPECT_EQ(runtime_release.release->release_id, *release_id);
  EXPECT_EQ(runtime_release.release->generation, 1U);
  EXPECT_EQ(runtime_release.release->elevator_id, "elevator_1");
  EXPECT_EQ(
    runtime_release.release->source.map_asset_digest,
    canonical_test_digest('3'));
  EXPECT_EQ(runtime_release.release->source.map_asset_epoch, 33U);
  EXPECT_EQ(
    runtime_release.release->target.map_asset_digest,
    canonical_test_digest('4'));
  EXPECT_EQ(runtime_release.release->target.map_asset_epoch, 44U);
  EXPECT_EQ(
    runtime_release.release->source.poses.front().role,
    robot_elevator_manager::PoseRole::kHallCall);
  EXPECT_EQ(
    runtime_release.release->source.poses.front().pose_id.rfind("eip_", 0U),
    0U);

  const auto released_configuration = YAML::Load(
    read_text_file(release_root / "configuration.yaml"));
  EXPECT_EQ(released_configuration["schema_version"].as<int>(), 2);
  EXPECT_FALSE(
    released_configuration["elevators"][0]["floors"][0]["threshold"].IsDefined());
  EXPECT_FALSE(
    released_configuration["elevators"][0]["floors"][0]["poses"]["hall_wait"]
    .IsDefined());
  EXPECT_TRUE(
    released_configuration["elevators"][0]["floors"][0]["poses"]["landing"]
    .IsDefined());
  EXPECT_EQ(
    released_configuration["elevators"][0]["floors"][0]["map_asset_epoch"]
    .as<std::uint64_t>(),
    33U);
  EXPECT_EQ(
    released_configuration["elevators"][0]["floors"][1]["map_asset_epoch"]
    .as<std::uint64_t>(),
    44U);
  const auto release_manifest =
    read_text_file(release_root / "manifest.json");
  EXPECT_NE(
    release_manifest.find("\"asset_epoch\":33"),
    std::string::npos);
  EXPECT_NE(
    release_manifest.find("\"asset_epoch\":44"),
    std::string::npos);
  EXPECT_NE(
    release_manifest.find(
      "\"asset_digest_contract\":\"njrh-map-asset-bundle-v1\""),
    std::string::npos);

  const auto inspected = module.query(
    ElevatorConfigurationQuery{"B3", "", "", ""});
  ASSERT_EQ(inspected.status, 200) << inspected.body;
  EXPECT_EQ(json_string_value(inspected.body, "current_release_id"), release_id);

  const auto referenced = module.query(
    ElevatorConfigurationQuery{"B3", "", "F3", "map_f3"});
  ASSERT_EQ(referenced.status, 200) << referenced.body;
  EXPECT_TRUE(json_bool_value(referenced.body, "referenced", false));
  EXPECT_EQ(json_string_value(referenced.body, "release_id"), release_id);

  const auto unreferenced = module.query(
    ElevatorConfigurationQuery{"B3", "", "F3", "map_other"});
  ASSERT_EQ(unreferenced.status, 200) << unreferenced.body;
  EXPECT_FALSE(json_bool_value(unreferenced.body, "referenced", true));
}

TEST(ElevatorConfigurationModule, PublishesV3ReverseEntryFourPointRelease)
{
  TemporaryDirectory temporary;
  ElevatorConfigurationModule module(temporary.path(), test_resolver());

  ElevatorConfigurationCommand save;
  save.type = ElevatorConfigurationCommandType::kSaveDraft;
  save.building_id = "B3";
  save.document = valid_reverse_entry_document();
  save.actor_id = "commissioning_app";
  const auto saved = module.execute(save);

  ASSERT_EQ(saved.status, 200) << saved.body;
  ASSERT_TRUE(json_bool_value(saved.body, "valid_for_publish", false))
    << saved.body;
  const auto revision = json_string_value(saved.body, "draft_revision");
  ASSERT_TRUE(revision.has_value());
  EXPECT_EQ(revision->rfind("draft-v3-", 0U), 0U);
  EXPECT_EQ(
    json_number_value(
      saved.body, "configuration_schema_version").value_or(0.0),
    3.0);

  ElevatorConfigurationCommand publish;
  publish.type = ElevatorConfigurationCommandType::kPublish;
  publish.building_id = "B3";
  publish.expected_draft_revision = *revision;
  publish.actor_id = "commissioning_app";
  const auto published = module.execute(publish);
  ASSERT_EQ(published.status, 201) << published.body;
  const auto release_id = json_string_value(published.body, "release_id");
  ASSERT_TRUE(release_id.has_value());

  robot_elevator_manager::ElevatorReleaseLoadRequest request;
  request.config_root =
    (temporary.path() / "B3" / ".elevator_config").string();
  request.building_id = "B3";
  request.source_floor_id = "F3";
  request.source_map_id = "map_f3";
  request.target_floor_id = "F4";
  request.target_map_id = "map_f4";
  request.preferred_elevator_id = "elevator_1";
  request.expected_release_id = *release_id;
  const auto frozen = robot_elevator_manager::load_elevator_release(request);
  ASSERT_TRUE(frozen.ok()) << frozen.message;
  ASSERT_TRUE(frozen.release.has_value());
  EXPECT_EQ(frozen.release->schema_version, 3U);
  EXPECT_EQ(frozen.release->source.poses.size(), 4U);
  EXPECT_EQ(
    frozen.release->source.cabin_panel_side,
    robot_elevator_manager::PanelSide::kLeft);
  EXPECT_EQ(
    frozen.release->target.cabin_panel_side,
    robot_elevator_manager::PanelSide::kRight);
  EXPECT_EQ(
    frozen.release->source.poses.back().role,
    robot_elevator_manager::PoseRole::kCabinPanel);
}

TEST(ElevatorConfigurationModule, AllowsCabinPanelPoseWithLongitudinalOffset)
{
  TemporaryDirectory temporary;
  ElevatorConfigurationModule module(temporary.path(), test_resolver());
  auto document = valid_reverse_entry_document();
  const std::string aligned_panel =
    R"json("cabin_panel": {"x": 0.8, "y": 0.5, "yaw": 3.141592653589793})json";
  const std::string offset_panel =
    R"json("cabin_panel": {"x": 1.061, "y": 0.5, "yaw": 3.141592653589793})json";
  const auto panel = document.find(aligned_panel);
  ASSERT_NE(panel, std::string::npos);
  document.replace(panel, aligned_panel.size(), offset_panel);

  ElevatorConfigurationCommand save;
  save.type = ElevatorConfigurationCommandType::kSaveDraft;
  save.building_id = "B3";
  save.document = document;
  save.actor_id = "commissioning_app";
  const auto saved = module.execute(save);

  ASSERT_EQ(saved.status, 200) << saved.body;
  EXPECT_TRUE(json_bool_value(saved.body, "valid_for_publish", false))
    << saved.body;
  EXPECT_EQ(saved.body.find("CABIN_PANEL_AXIS_INVALID"), std::string::npos)
    << saved.body;
}

TEST(ElevatorConfigurationModule, MigratesLegacyDraftToThreePoseSchemaV2)
{
  const auto erase_role = [](std::string & document, const std::string & role) {
      const auto marker = "          \"" + role + "\":";
      std::size_t position = 0U;
      while ((position = document.find(marker, position)) != std::string::npos) {
        const auto end = document.find('\n', position);
        ASSERT_NE(end, std::string::npos);
        document.erase(position, end - position + 1U);
      }
    };
  struct MigrationCase
  {
    bool remove_hall_wait;
    bool remove_doorway;
    double expected_landing_x;
  };
  const std::vector<MigrationCase> cases{
    {false, false, -0.8},
    {true, false, -0.4},
    {true, true, -0.2},
  };

  for (const auto & migration_case : cases) {
    SCOPED_TRACE(migration_case.expected_landing_x);
    TemporaryDirectory temporary;
    ElevatorConfigurationModule module(temporary.path(), test_resolver());
    auto document = legacy_v1_document();
    if (migration_case.remove_hall_wait) {
      erase_role(document, "hall_wait");
    }
    if (migration_case.remove_doorway) {
      erase_role(document, "doorway");
    }

    ElevatorConfigurationCommand save;
    save.type = ElevatorConfigurationCommandType::kSaveDraft;
    save.building_id = "B3";
    save.document = document;
    const auto saved = module.execute(save);
    ASSERT_EQ(saved.status, 200) << saved.body;
    EXPECT_EQ(
      json_number_value(saved.body, "source_schema_version").value_or(0.0),
      1.0);
    EXPECT_EQ(
      json_number_value(
        saved.body, "configuration_schema_version").value_or(0.0),
      2.0);
    EXPECT_FALSE(json_bool_value(saved.body, "migration_required", true));
    const auto revision =
      json_string_value(saved.body, "draft_revision").value_or("");
    EXPECT_EQ(revision.rfind("draft-v2-", 0U), 0U);

    const auto inspected = module.query(
      ElevatorConfigurationQuery{"B3", "", "", ""});
    ASSERT_EQ(inspected.status, 200) << inspected.body;
    const auto configuration_json =
      json_object_value(inspected.body, "configuration");
    ASSERT_TRUE(configuration_json.has_value());
    const auto configuration = YAML::Load(*configuration_json);
    const auto floor = configuration["elevators"][0]["floors"][0];
    EXPECT_EQ(floor["poses"]["landing"]["x"].as<double>(),
      migration_case.expected_landing_x);
    EXPECT_TRUE(floor["poses"]["hall_call"].IsDefined());
    EXPECT_TRUE(floor["poses"]["cabin"].IsDefined());
    EXPECT_FALSE(floor["poses"]["hall_wait"].IsDefined());
    EXPECT_FALSE(floor["poses"]["doorway"].IsDefined());
    EXPECT_FALSE(floor["poses"]["exit"].IsDefined());
    EXPECT_FALSE(floor["threshold"].IsDefined());
  }
}

TEST(ElevatorConfigurationModule, DoesNotFallbackFromMalformedLegacyHallWait)
{
  TemporaryDirectory temporary;
  ElevatorConfigurationModule module(temporary.path(), test_resolver());
  auto document = legacy_v1_document();
  const std::string valid =
    "\"hall_wait\": {\"x\": -0.8, \"y\": 0.1, \"yaw\": 0.2}";
  const std::string malformed =
    "\"hall_wait\": {\"x\": \"invalid\", \"y\": 0.1, \"yaw\": 0.2}";
  std::size_t position = 0U;
  while ((position = document.find(valid, position)) != std::string::npos) {
    document.replace(position, valid.size(), malformed);
    position += malformed.size();
  }

  ElevatorConfigurationCommand save;
  save.type = ElevatorConfigurationCommandType::kSaveDraft;
  save.building_id = "B3";
  save.document = document;
  const auto saved = module.execute(save);

  ASSERT_EQ(saved.status, 200) << saved.body;
  EXPECT_FALSE(json_bool_value(saved.body, "valid_for_publish", true));
  EXPECT_NE(saved.body.find("INVALID_POSE"), std::string::npos);
  const auto inspected = module.query(
    ElevatorConfigurationQuery{"B3", "", "", ""});
  const auto configuration_json =
    json_object_value(inspected.body, "configuration");
  ASSERT_TRUE(configuration_json.has_value());
  const auto configuration = YAML::Load(*configuration_json);
  EXPECT_EQ(
    configuration["elevators"][0]["floors"][0]["poses"]["landing"]["x"]
    .as<std::string>(),
    "invalid");
}

TEST(ElevatorConfigurationModule, PreviewsLegacyDraftMigrationWithoutWriting)
{
  TemporaryDirectory temporary;
  const auto revision = install_legacy_v1_draft(temporary.path());
  const auto config_root =
    temporary.path() / "B3" / ".elevator_config";
  const auto configuration_path =
    config_root / "drafts" / revision / "configuration.yaml";
  const auto validation_path =
    config_root / "drafts" / revision / "validation.json";
  const auto pointer_path = config_root / "draft.json";
  const auto configuration_before = read_text_file(configuration_path);
  const auto validation_before = read_text_file(validation_path);
  const auto pointer_before = read_text_file(pointer_path);
  ElevatorConfigurationModule module(temporary.path(), test_resolver());

  const auto preview = module.query(
    ElevatorConfigurationQuery{"B3", "", "", ""});

  ASSERT_EQ(preview.status, 200) << preview.body;
  EXPECT_EQ(json_string_value(preview.body, "draft_revision"), revision);
  EXPECT_TRUE(json_bool_value(preview.body, "migration_required", false));
  EXPECT_FALSE(json_bool_value(preview.body, "legacy_read_only", true));
  EXPECT_EQ(
    json_number_value(preview.body, "source_schema_version").value_or(0.0),
    1.0);
  EXPECT_EQ(
    json_number_value(
      preview.body, "configuration_schema_version").value_or(0.0),
    2.0);
  const auto preview_document =
    json_object_value(preview.body, "configuration");
  ASSERT_TRUE(preview_document.has_value());
  const auto preview_root = YAML::Load(*preview_document);
  EXPECT_EQ(preview_root["schema_version"].as<int>(), 2);
  EXPECT_EQ(
    preview_root["elevators"][0]["floors"][0]["poses"]["landing"]["x"]
    .as<double>(),
    -0.8);
  EXPECT_FALSE(
    preview_root["elevators"][0]["floors"][0]["threshold"].IsDefined());
  EXPECT_EQ(read_text_file(configuration_path), configuration_before);
  EXPECT_EQ(read_text_file(validation_path), validation_before);
  EXPECT_EQ(read_text_file(pointer_path), pointer_before);

  ElevatorConfigurationCommand publish;
  publish.type = ElevatorConfigurationCommandType::kPublish;
  publish.building_id = "B3";
  publish.expected_draft_revision = revision;
  const auto rejected_publish = module.execute(publish);
  EXPECT_EQ(rejected_publish.status, 422) << rejected_publish.body;
  EXPECT_EQ(
    rejected_publish.code, "LEGACY_DRAFT_MIGRATION_REQUIRED");

  ElevatorConfigurationCommand save;
  save.type = ElevatorConfigurationCommandType::kSaveDraft;
  save.building_id = "B3";
  save.document = *preview_document;
  save.expected_draft_revision = revision;
  const auto migrated = module.execute(save);
  ASSERT_EQ(migrated.status, 200) << migrated.body;
  const auto migrated_revision =
    json_string_value(migrated.body, "draft_revision").value_or("");
  EXPECT_EQ(migrated_revision.rfind("draft-v2-", 0U), 0U);
  EXPECT_NE(migrated_revision, revision);
}

TEST(ElevatorConfigurationModule, RejectsDraftRevisionSchemaContentMismatch)
{
  TemporaryDirectory temporary;
  install_mismatched_v2_revision_with_v1_content(temporary.path());
  ElevatorConfigurationModule module(temporary.path(), test_resolver());

  const auto result = module.query(
    ElevatorConfigurationQuery{"B3", "", "", ""});

  EXPECT_EQ(result.status, 500) << result.body;
  EXPECT_EQ(result.code, "ELEVATOR_CONFIG_IO_ERROR");
  EXPECT_NE(
    result.body.find("immutable revision"),
    std::string::npos);
}

TEST(ElevatorConfigurationModule, KeepsLegacyReleaseReadableButReadOnly)
{
  TemporaryDirectory temporary;
  const auto release_id =
    install_legacy_v1_release(temporary.path(), true);
  const auto release_root =
    temporary.path() / "B3" / ".elevator_config" / "releases" / release_id;
  const auto configuration_before =
    read_text_file(release_root / "configuration.yaml");
  const auto manifest_before =
    read_text_file(release_root / "manifest.json");
  ElevatorConfigurationModule module(temporary.path(), test_resolver());

  const auto explicit_release = module.query(
    ElevatorConfigurationQuery{"B3", release_id, "", ""});

  ASSERT_EQ(explicit_release.status, 200) << explicit_release.body;
  EXPECT_EQ(explicit_release.code, "RELEASE_FOUND");
  EXPECT_TRUE(
    json_bool_value(explicit_release.body, "legacy_read_only", false));
  EXPECT_FALSE(
    json_bool_value(explicit_release.body, "migration_required", true));
  EXPECT_EQ(
    json_number_value(
      explicit_release.body, "source_schema_version").value_or(0.0),
    1.0);
  EXPECT_NE(
    explicit_release.body.find("\"schema_version\":1"),
    std::string::npos);
  EXPECT_EQ(
    read_text_file(release_root / "configuration.yaml"),
    configuration_before);
  EXPECT_EQ(read_text_file(release_root / "manifest.json"), manifest_before);

  const auto current = module.query(
    ElevatorConfigurationQuery{"B3", "", "", ""});
  ASSERT_EQ(current.status, 200) << current.body;
  EXPECT_EQ(
    json_string_value(current.body, "current_release_id"), release_id);
  EXPECT_TRUE(json_bool_value(current.body, "legacy_read_only", false));
  EXPECT_NE(
    current.body.find("\"releases\":[\"" + release_id + "\"]"),
    std::string::npos) << current.body;

  const auto reference = module.query(
    ElevatorConfigurationQuery{"B3", "", "F3", "map_f3"});
  ASSERT_EQ(reference.status, 200) << reference.body;
  EXPECT_TRUE(json_bool_value(reference.body, "referenced", false));
  EXPECT_EQ(json_string_value(reference.body, "release_id"), release_id);

  robot_elevator_manager::ElevatorReleaseLoadRequest runtime_request;
  runtime_request.config_root =
    (temporary.path() / "B3" / ".elevator_config").string();
  runtime_request.building_id = "B3";
  runtime_request.source_floor_id = "F3";
  runtime_request.source_map_id = "map_f3";
  runtime_request.target_floor_id = "F4";
  runtime_request.target_map_id = "map_f4";
  runtime_request.preferred_elevator_id = "elevator_1";
  runtime_request.expected_release_id = release_id;
  const auto runtime_release =
    robot_elevator_manager::load_elevator_release(runtime_request);
  EXPECT_EQ(
    runtime_release.error,
    robot_elevator_manager::ElevatorReleaseLoadError::kLegacyReadOnly);
  EXPECT_FALSE(runtime_release.release.has_value());

  ElevatorConfigurationCommand rollback;
  rollback.type = ElevatorConfigurationCommandType::kRollback;
  rollback.building_id = "B3";
  rollback.release_id = release_id;
  rollback.expected_release_id = release_id;
  const auto rejected = module.execute(rollback);
  EXPECT_EQ(rejected.status, 409) << rejected.body;
  EXPECT_EQ(rejected.code, "LEGACY_RELEASE_READ_ONLY");
  EXPECT_EQ(
    read_text_file(release_root / "configuration.yaml"),
    configuration_before);
}

TEST(ElevatorConfigurationModule, RejectsReleaseSourceDraftSchemaMismatch)
{
  TemporaryDirectory temporary;
  const auto release_id =
    install_legacy_v1_release(temporary.path(), false);
  const auto manifest_path =
    temporary.path() / "B3" / ".elevator_config" / "releases" /
    release_id / "manifest.json";
  auto manifest = read_text_file(manifest_path);
  const auto prefix = manifest.find("draft-v1-");
  ASSERT_NE(prefix, std::string::npos);
  manifest.replace(prefix, std::string("draft-v1-").size(), "draft-v2-");
  {
    std::ofstream output(
      manifest_path, std::ios::binary | std::ios::trunc);
    output << manifest;
  }
  ElevatorConfigurationModule module(temporary.path(), test_resolver());

  const auto result = module.query(
    ElevatorConfigurationQuery{"B3", release_id, "", ""});

  EXPECT_EQ(result.status, 500) << result.body;
  EXPECT_EQ(result.code, "ELEVATOR_CONFIG_IO_ERROR");
  EXPECT_NE(
    result.body.find("audit metadata"),
    std::string::npos);
}

TEST(ElevatorConfigurationModule, KeepsIncompleteDraftButRejectsPublish)
{
  TemporaryDirectory temporary;
  ElevatorConfigurationModule module(temporary.path(), test_resolver());

  ElevatorConfigurationCommand save;
  save.type = ElevatorConfigurationCommandType::kSaveDraft;
  save.building_id = "B3";
  save.document =
    R"json({"schema_version":2,"building_id":"B3","elevators":[]})json";
  const auto saved = module.execute(save);

  ASSERT_EQ(saved.status, 200) << saved.body;
  EXPECT_FALSE(json_bool_value(saved.body, "valid_for_publish", true));
  EXPECT_NE(saved.body.find("MISSING_ELEVATORS"), std::string::npos);
  const auto draft_revision = json_string_value(saved.body, "draft_revision");
  ASSERT_TRUE(draft_revision.has_value());

  ElevatorConfigurationCommand publish;
  publish.type = ElevatorConfigurationCommandType::kPublish;
  publish.building_id = "B3";
  publish.expected_draft_revision = *draft_revision;
  const auto rejected = module.execute(publish);

  EXPECT_EQ(rejected.status, 422) << rejected.body;
  EXPECT_EQ(rejected.code, "DRAFT_REVIEW_REQUIRED");
  EXPECT_TRUE(json_bool_value(rejected.body, "review_required", false));
  EXPECT_FALSE(fs::exists(
      temporary.path() / "B3" / ".elevator_config" / "current.json"));
  EXPECT_FALSE(fs::exists(temporary.path() / "B3" / "elevators.yaml"));
}

TEST(ElevatorConfigurationModule, BoundsTopologyBeforeAssetResolution)
{
  TemporaryDirectory temporary;
  std::size_t resolver_calls = 0U;
  ElevatorConfigurationModule module(
    temporary.path(),
    [&resolver_calls](
      const std::string &,
      const std::string &,
      const std::string &)
    -> std::optional<ElevatorFloorAssetSnapshot>
    {
      ++resolver_calls;
      return std::nullopt;
    });

  std::string document =
    R"json({"schema_version":2,"building_id":"B3","elevators":[)json";
  for (std::size_t index = 0U; index < 17U; ++index) {
    if (index > 0U) {
      document += ",";
    }
    document +=
      "{\"elevator_id\":\"elevator_" + std::to_string(index) +
      "\",\"floors\":[]}";
  }
  document += "]}";

  ElevatorConfigurationCommand save;
  save.type = ElevatorConfigurationCommandType::kSaveDraft;
  save.building_id = "B3";
  save.document = document;
  const auto rejected = module.execute(save);

  ASSERT_EQ(rejected.status, 200) << rejected.body;
  EXPECT_FALSE(json_bool_value(rejected.body, "valid_for_publish", true));
  EXPECT_NE(
    rejected.body.find("CONFIGURATION_LIMIT_EXCEEDED"),
    std::string::npos) << rejected.body;
  EXPECT_EQ(resolver_calls, 0U);
}

TEST(ElevatorConfigurationModule, CachesRepeatedMapAssetResolutionPerValidation)
{
  TemporaryDirectory temporary;
  std::size_t resolver_calls = 0U;
  const auto delegate = test_resolver();
  ElevatorConfigurationModule module(
    temporary.path(),
    [&resolver_calls, &delegate](
      const std::string & building_id,
      const std::string & floor_id,
      const std::string & map_id)
    {
      ++resolver_calls;
      return delegate(building_id, floor_id, map_id);
    });

  auto document = valid_document();
  const auto elevators = document.find("\"elevators\"");
  const auto first_elevator = document.find('{', elevators);
  const auto elevator_array_end = document.rfind(']');
  const auto first_elevator_end =
    document.rfind('}', elevator_array_end);
  ASSERT_NE(elevators, std::string::npos);
  ASSERT_NE(first_elevator, std::string::npos);
  ASSERT_NE(elevator_array_end, std::string::npos);
  ASSERT_NE(first_elevator_end, std::string::npos);
  auto duplicate = document.substr(
    first_elevator, first_elevator_end - first_elevator + 1U);
  const auto duplicate_id =
    duplicate.find("\"elevator_id\": \"elevator_1\"");
  ASSERT_NE(duplicate_id, std::string::npos);
  duplicate.replace(
    duplicate_id,
    std::string("\"elevator_id\": \"elevator_1\"").size(),
    "\"elevator_id\": \"elevator_2\"");
  document.insert(first_elevator_end + 1U, "," + duplicate);

  ElevatorConfigurationCommand save;
  save.type = ElevatorConfigurationCommandType::kSaveDraft;
  save.building_id = "B3";
  save.document = document;
  const auto saved = module.execute(save);

  ASSERT_EQ(saved.status, 200) << saved.body;
  EXPECT_TRUE(json_bool_value(saved.body, "valid_for_publish", false));
  EXPECT_EQ(resolver_calls, 2U);
}

TEST(ElevatorConfigurationModule, RejectsStaleDraftAndReleaseRevisions)
{
  TemporaryDirectory temporary;
  ElevatorConfigurationModule module(temporary.path(), test_resolver());

  ElevatorConfigurationCommand first;
  first.type = ElevatorConfigurationCommandType::kSaveDraft;
  first.building_id = "B3";
  first.document = valid_document();
  const auto first_saved = module.execute(first);
  ASSERT_EQ(first_saved.status, 200) << first_saved.body;
  const auto first_revision =
    json_string_value(first_saved.body, "draft_revision");
  ASSERT_TRUE(first_revision.has_value());

  ElevatorConfigurationCommand stale_save = first;
  stale_save.document = valid_document(-0.7);
  stale_save.expected_draft_revision = "draft-v2-stale";
  const auto stale_save_result = module.execute(stale_save);
  EXPECT_EQ(stale_save_result.status, 409) << stale_save_result.body;
  EXPECT_EQ(stale_save_result.code, "DRAFT_REVISION_CONFLICT");

  ElevatorConfigurationCommand publish;
  publish.type = ElevatorConfigurationCommandType::kPublish;
  publish.building_id = "B3";
  publish.expected_draft_revision = *first_revision;
  const auto first_published = module.execute(publish);
  ASSERT_EQ(first_published.status, 201) << first_published.body;
  const auto first_release =
    json_string_value(first_published.body, "release_id");
  ASSERT_TRUE(first_release.has_value());

  const auto stale_publish = module.execute(publish);
  EXPECT_EQ(stale_publish.status, 409) << stale_publish.body;
  EXPECT_EQ(stale_publish.code, "RELEASE_REVISION_CONFLICT");
  const auto inspected = module.query(
    ElevatorConfigurationQuery{"B3", "", "", ""});
  EXPECT_EQ(
    json_string_value(inspected.body, "current_release_id"),
    first_release);
}

TEST(ElevatorConfigurationModule, RollbackCreatesNewAuditableRelease)
{
  TemporaryDirectory temporary;
  ElevatorConfigurationModule module(temporary.path(), test_resolver());

  ElevatorConfigurationCommand save_first;
  save_first.type = ElevatorConfigurationCommandType::kSaveDraft;
  save_first.building_id = "B3";
  save_first.document = valid_document(-0.6);
  const auto saved_first = module.execute(save_first);
  ASSERT_EQ(saved_first.status, 200) << saved_first.body;
  const auto draft_first =
    json_string_value(saved_first.body, "draft_revision");
  ASSERT_TRUE(draft_first.has_value());

  ElevatorConfigurationCommand publish_first;
  publish_first.type = ElevatorConfigurationCommandType::kPublish;
  publish_first.building_id = "B3";
  publish_first.expected_draft_revision = *draft_first;
  const auto published_first = module.execute(publish_first);
  ASSERT_EQ(published_first.status, 201) << published_first.body;
  const auto release_first =
    json_string_value(published_first.body, "release_id");
  ASSERT_TRUE(release_first.has_value());

  ElevatorConfigurationCommand save_second = save_first;
  save_second.document = valid_document(-0.7);
  save_second.expected_draft_revision = *draft_first;
  const auto saved_second = module.execute(save_second);
  ASSERT_EQ(saved_second.status, 200) << saved_second.body;
  const auto draft_second =
    json_string_value(saved_second.body, "draft_revision");
  ASSERT_TRUE(draft_second.has_value());
  EXPECT_NE(*draft_second, *draft_first);

  ElevatorConfigurationCommand publish_second = publish_first;
  publish_second.expected_draft_revision = *draft_second;
  publish_second.expected_release_id = *release_first;
  const auto published_second = module.execute(publish_second);
  ASSERT_EQ(published_second.status, 201) << published_second.body;
  const auto release_second =
    json_string_value(published_second.body, "release_id");
  ASSERT_TRUE(release_second.has_value());

  ElevatorConfigurationCommand rollback;
  rollback.type = ElevatorConfigurationCommandType::kRollback;
  rollback.building_id = "B3";
  rollback.release_id = *release_first;
  rollback.expected_release_id = *release_second;
  rollback.actor_id = "commissioning_app";
  const auto rolled_back = module.execute(rollback);

  ASSERT_EQ(rolled_back.status, 201) << rolled_back.body;
  EXPECT_EQ(rolled_back.code, "CONFIGURATION_ROLLED_BACK");
  const auto release_third =
    json_string_value(rolled_back.body, "release_id");
  ASSERT_TRUE(release_third.has_value());
  EXPECT_NE(*release_third, *release_first);
  EXPECT_NE(*release_third, *release_second);
  EXPECT_EQ(
    json_string_value(rolled_back.body, "rollback_of"),
    release_first);

  const auto releases_root =
    temporary.path() / "B3" / ".elevator_config" / "releases";
  EXPECT_TRUE(fs::is_directory(releases_root / *release_first));
  EXPECT_TRUE(fs::is_directory(releases_root / *release_second));
  EXPECT_TRUE(fs::is_directory(releases_root / *release_third));
  const auto current = module.query(
    ElevatorConfigurationQuery{"B3", "", "", ""});
  EXPECT_EQ(
    json_string_value(current.body, "current_release_id"),
    release_third);
  EXPECT_NE(current.body.find("\"generation\":3"), std::string::npos);
}

TEST(ElevatorConfigurationModule, RevalidatesMapBoundsAndDigest)
{
  TemporaryDirectory temporary;
  auto digest = std::make_shared<std::string>(canonical_test_digest('1'));
  ElevatorFloorAssetResolver resolver =
    [digest](
    const std::string & building_id,
    const std::string & floor_id,
    const std::string & map_id)
    -> std::optional<ElevatorFloorAssetSnapshot>
    {
      const auto base = test_resolver()(building_id, floor_id, map_id);
      if (!base) {
        return std::nullopt;
      }
      auto snapshot = *base;
      snapshot.asset_digest = *digest;
      return snapshot;
    };
  ElevatorConfigurationModule module(temporary.path(), resolver);

  ElevatorConfigurationCommand out_of_bounds;
  out_of_bounds.type = ElevatorConfigurationCommandType::kSaveDraft;
  out_of_bounds.building_id = "B3";
  out_of_bounds.document = valid_document(20.0);
  const auto rejected_bounds = module.execute(out_of_bounds);
  ASSERT_EQ(rejected_bounds.status, 200) << rejected_bounds.body;
  EXPECT_FALSE(json_bool_value(
      rejected_bounds.body, "valid_for_publish", true));
  EXPECT_NE(rejected_bounds.body.find("POSE_OUT_OF_MAP"), std::string::npos);

  ElevatorConfigurationCommand valid = out_of_bounds;
  valid.document = valid_document();
  valid.expected_draft_revision =
    json_string_value(rejected_bounds.body, "draft_revision").value_or("");
  const auto saved = module.execute(valid);
  ASSERT_EQ(saved.status, 200) << saved.body;
  const auto draft_revision =
    json_string_value(saved.body, "draft_revision");
  ASSERT_TRUE(draft_revision.has_value());

  ElevatorConfigurationCommand publish;
  publish.type = ElevatorConfigurationCommandType::kPublish;
  publish.building_id = "B3";
  publish.expected_draft_revision = *draft_revision;
  const auto published = module.execute(publish);
  ASSERT_EQ(published.status, 201) << published.body;
  const auto release_id = json_string_value(published.body, "release_id");
  ASSERT_TRUE(release_id.has_value());

  *digest = canonical_test_digest('2');
  ElevatorConfigurationCommand rollback;
  rollback.type = ElevatorConfigurationCommandType::kRollback;
  rollback.building_id = "B3";
  rollback.release_id = *release_id;
  rollback.expected_release_id = *release_id;
  const auto rejected_digest = module.execute(rollback);

  EXPECT_EQ(rejected_digest.status, 422) << rejected_digest.body;
  EXPECT_EQ(rejected_digest.code, "ROLLBACK_TARGET_INVALID");
  EXPECT_NE(
    rejected_digest.body.find("MAP_ASSET_DIGEST_CHANGED"),
    std::string::npos);
}

TEST(ElevatorConfigurationModule, RejectsPublishWhenMapChangesAfterDraftReview)
{
  TemporaryDirectory temporary;
  auto digest = std::make_shared<std::string>(canonical_test_digest('a'));
  ElevatorFloorAssetResolver resolver =
    [digest](
    const std::string & building_id,
    const std::string & floor_id,
    const std::string & map_id)
    -> std::optional<ElevatorFloorAssetSnapshot>
    {
      const auto base = test_resolver()(building_id, floor_id, map_id);
      if (!base) {
        return std::nullopt;
      }
      auto snapshot = *base;
      snapshot.asset_digest = *digest;
      return snapshot;
    };
  ElevatorConfigurationModule module(temporary.path(), resolver);

  ElevatorConfigurationCommand save;
  save.type = ElevatorConfigurationCommandType::kSaveDraft;
  save.building_id = "B3";
  save.document = valid_document();
  const auto saved = module.execute(save);
  ASSERT_EQ(saved.status, 200) << saved.body;
  ASSERT_TRUE(json_bool_value(saved.body, "valid_for_publish", false));

  *digest = canonical_test_digest('b');
  ElevatorConfigurationCommand publish;
  publish.type = ElevatorConfigurationCommandType::kPublish;
  publish.building_id = "B3";
  publish.expected_draft_revision =
    json_string_value(saved.body, "draft_revision").value_or("");
  const auto rejected = module.execute(publish);

  EXPECT_EQ(rejected.status, 422) << rejected.body;
  EXPECT_EQ(rejected.code, "CONFIGURATION_INVALID");
  EXPECT_NE(
    rejected.body.find("MAP_ASSET_DIGEST_CHANGED"),
    std::string::npos);
  EXPECT_FALSE(fs::exists(
      temporary.path() / "B3" / ".elevator_config" / "current"));
}

TEST(ElevatorConfigurationModule, RejectsPublishWhenMapEpochChangesAfterDraftReview)
{
  TemporaryDirectory temporary;
  auto asset_epoch = std::make_shared<std::uint64_t>(71U);
  ElevatorFloorAssetResolver resolver =
    [asset_epoch](
    const std::string & building_id,
    const std::string & floor_id,
    const std::string & map_id)
    -> std::optional<ElevatorFloorAssetSnapshot>
    {
      const auto base = test_resolver()(building_id, floor_id, map_id);
      if (!base) {
        return std::nullopt;
      }
      auto snapshot = *base;
      snapshot.asset_epoch = *asset_epoch;
      return snapshot;
    };
  ElevatorConfigurationModule module(temporary.path(), resolver);

  ElevatorConfigurationCommand save;
  save.type = ElevatorConfigurationCommandType::kSaveDraft;
  save.building_id = "B3";
  save.document = valid_document();
  const auto saved = module.execute(save);
  ASSERT_EQ(saved.status, 200) << saved.body;
  ASSERT_TRUE(json_bool_value(saved.body, "valid_for_publish", false));

  *asset_epoch = 72U;
  ElevatorConfigurationCommand publish;
  publish.type = ElevatorConfigurationCommandType::kPublish;
  publish.building_id = "B3";
  publish.expected_draft_revision =
    json_string_value(saved.body, "draft_revision").value_or("");
  const auto rejected = module.execute(publish);

  EXPECT_EQ(rejected.status, 422) << rejected.body;
  EXPECT_EQ(rejected.code, "CONFIGURATION_INVALID");
  EXPECT_NE(
    rejected.body.find("MAP_ASSET_EPOCH_CHANGED"),
    std::string::npos);
  EXPECT_FALSE(fs::exists(
      temporary.path() / "B3" / ".elevator_config" / "current"));
}

TEST(ElevatorConfigurationModule, RequiresResaveAfterInvalidDraftEnvironmentIsFixed)
{
  TemporaryDirectory temporary;
  bool maps_available = false;
  const auto delegate = test_resolver();
  ElevatorConfigurationModule module(
    temporary.path(),
    [&maps_available, &delegate](
      const std::string & building_id,
      const std::string & floor_id,
      const std::string & map_id)
    -> std::optional<ElevatorFloorAssetSnapshot>
    {
      if (!maps_available) {
        return std::nullopt;
      }
      return delegate(building_id, floor_id, map_id);
    });

  ElevatorConfigurationCommand save;
  save.type = ElevatorConfigurationCommandType::kSaveDraft;
  save.building_id = "B3";
  save.document = valid_document();
  const auto invalid = module.execute(save);
  ASSERT_EQ(invalid.status, 200) << invalid.body;
  EXPECT_FALSE(json_bool_value(invalid.body, "valid_for_publish", true));
  const auto invalid_revision =
    json_string_value(invalid.body, "draft_revision");
  ASSERT_TRUE(invalid_revision.has_value());

  maps_available = true;
  ElevatorConfigurationCommand publish;
  publish.type = ElevatorConfigurationCommandType::kPublish;
  publish.building_id = "B3";
  publish.expected_draft_revision = *invalid_revision;
  const auto rejected = module.execute(publish);
  EXPECT_EQ(rejected.status, 422) << rejected.body;
  EXPECT_EQ(rejected.code, "DRAFT_REVIEW_REQUIRED");
  EXPECT_TRUE(json_bool_value(rejected.body, "review_required", false));

  save.expected_draft_revision = *invalid_revision;
  const auto reviewed = module.execute(save);
  ASSERT_EQ(reviewed.status, 200) << reviewed.body;
  EXPECT_TRUE(json_bool_value(reviewed.body, "valid_for_publish", false));
  const auto reviewed_revision =
    json_string_value(reviewed.body, "draft_revision");
  ASSERT_TRUE(reviewed_revision.has_value());
  EXPECT_NE(*reviewed_revision, *invalid_revision);

  publish.expected_draft_revision = *reviewed_revision;
  const auto published = module.execute(publish);
  EXPECT_EQ(published.status, 201) << published.body;
}

TEST(ElevatorConfigurationModule, RejectsLegacyThresholdInSchemaV2)
{
  TemporaryDirectory temporary;
  ElevatorConfigurationModule module(temporary.path(), test_resolver());
  auto document = valid_document();
  const std::string marker = "\"map_id\": \"map_f3\",";
  const auto map_binding = document.find(marker);
  ASSERT_NE(map_binding, std::string::npos);
  document.insert(map_binding + marker.size(), " \"threshold\": {},");

  ElevatorConfigurationCommand save;
  save.type = ElevatorConfigurationCommandType::kSaveDraft;
  save.building_id = "B3";
  save.document = document;
  const auto rejected = module.execute(save);

  ASSERT_EQ(rejected.status, 200) << rejected.body;
  EXPECT_FALSE(json_bool_value(rejected.body, "valid_for_publish", true));
  EXPECT_NE(
    rejected.body.find("LEGACY_THRESHOLD_FORBIDDEN"),
    std::string::npos);
}

TEST(ElevatorConfigurationModule, RejectsLegacyPoseRolesInSchemaV2)
{
  TemporaryDirectory temporary;
  ElevatorConfigurationModule module(temporary.path(), test_resolver());
  auto document = valid_document();
  const std::string marker = "\"landing\": {";
  const auto landing = document.find(marker);
  ASSERT_NE(landing, std::string::npos);
  document.replace(
    landing, std::string("\"landing\"").size(), "\"hall_wait\"");

  ElevatorConfigurationCommand save;
  save.type = ElevatorConfigurationCommandType::kSaveDraft;
  save.building_id = "B3";
  save.document = document;
  const auto rejected = module.execute(save);

  ASSERT_EQ(rejected.status, 200) << rejected.body;
  EXPECT_FALSE(json_bool_value(rejected.body, "valid_for_publish", true));
  EXPECT_NE(
    rejected.body.find("LEGACY_POSE_ROLE_FORBIDDEN"),
    std::string::npos);
  const auto response = YAML::Load(rejected.body);
  ASSERT_TRUE(response["issues"].IsSequence());
  std::size_t missing_role_count = 0U;
  for (const auto & issue : response["issues"]) {
    if (issue["code"].as<std::string>() == "MISSING_POSE_ROLE") {
      ++missing_role_count;
    }
  }
  EXPECT_EQ(missing_role_count, 1U);
}

TEST(ElevatorConfigurationModule, RejectsUnsupportedConfigurationSchema)
{
  TemporaryDirectory temporary;
  ElevatorConfigurationModule module(temporary.path(), test_resolver());
  auto document = valid_document();
  const std::string supported = "\"schema_version\": 2";
  const auto schema = document.find(supported);
  ASSERT_NE(schema, std::string::npos);
  document.replace(schema, supported.size(), "\"schema_version\": 4");

  ElevatorConfigurationCommand save;
  save.type = ElevatorConfigurationCommandType::kSaveDraft;
  save.building_id = "B3";
  save.document = document;
  const auto rejected = module.execute(save);

  EXPECT_EQ(rejected.status, 400) << rejected.body;
  EXPECT_EQ(rejected.code, "UNSUPPORTED_SCHEMA_VERSION");
}

TEST(ElevatorConfigurationModule, RejectsNonScalarMapDigestInsteadOfRebinding)
{
  TemporaryDirectory temporary;
  ElevatorConfigurationModule module(temporary.path(), test_resolver());
  auto document = valid_document();
  const auto map_binding = document.find("\"map_id\": \"map_f3\",");
  ASSERT_NE(map_binding, std::string::npos);
  document.insert(
    map_binding + std::string("\"map_id\": \"map_f3\",").size(),
    " \"map_asset_digest\": {},");

  ElevatorConfigurationCommand save;
  save.type = ElevatorConfigurationCommandType::kSaveDraft;
  save.building_id = "B3";
  save.document = document;
  const auto rejected = module.execute(save);

  ASSERT_EQ(rejected.status, 200) << rejected.body;
  EXPECT_FALSE(json_bool_value(rejected.body, "valid_for_publish", true));
  EXPECT_NE(
    rejected.body.find("INVALID_MAP_ASSET_DIGEST"),
    std::string::npos);
}

TEST(ElevatorConfigurationModule, RejectsNonCanonicalUserSuppliedMapEpoch)
{
  const std::vector<std::string> invalid_epochs{
    "0",
    "1.0",
    "\"1\"",
    "18446744073709551616",
  };
  for (const auto & invalid_epoch : invalid_epochs) {
    SCOPED_TRACE(invalid_epoch);
    TemporaryDirectory temporary;
    ElevatorConfigurationModule module(temporary.path(), test_resolver());
    auto document = valid_document();
    const auto map_binding = document.find("\"map_id\": \"map_f3\",");
    ASSERT_NE(map_binding, std::string::npos);
    document.insert(
      map_binding + std::string("\"map_id\": \"map_f3\",").size(),
      " \"map_asset_epoch\": " + invalid_epoch + ",");

    ElevatorConfigurationCommand save;
    save.type = ElevatorConfigurationCommandType::kSaveDraft;
    save.building_id = "B3";
    save.document = document;
    const auto rejected = module.execute(save);

    ASSERT_EQ(rejected.status, 200) << rejected.body;
    EXPECT_FALSE(json_bool_value(rejected.body, "valid_for_publish", true));
    EXPECT_NE(
      rejected.body.find("INVALID_MAP_ASSET_EPOCH"),
      std::string::npos) << rejected.body;
  }
}

TEST(ElevatorConfigurationModule, RequiresAuthoritativePositiveMapEpoch)
{
  TemporaryDirectory temporary;
  const auto delegate = test_resolver();
  ElevatorConfigurationModule module(
    temporary.path(),
    [&delegate](
      const std::string & building_id,
      const std::string & floor_id,
      const std::string & map_id)
    {
      auto snapshot = delegate(building_id, floor_id, map_id);
      if (snapshot) {
        snapshot->asset_epoch = 0U;
      }
      return snapshot;
    });

  ElevatorConfigurationCommand save;
  save.type = ElevatorConfigurationCommandType::kSaveDraft;
  save.building_id = "B3";
  save.document = valid_document();
  const auto rejected = module.execute(save);

  ASSERT_EQ(rejected.status, 200) << rejected.body;
  EXPECT_FALSE(json_bool_value(rejected.body, "valid_for_publish", true));
  EXPECT_NE(
    rejected.body.find("MAP_ASSET_EPOCH_UNAVAILABLE"),
    std::string::npos) << rejected.body;
}

TEST(ElevatorConfigurationModule, PreservesMaximumUint64MapEpochExactly)
{
  TemporaryDirectory temporary;
  const auto delegate = test_resolver();
  ElevatorConfigurationModule module(
    temporary.path(),
    [&delegate](
      const std::string & building_id,
      const std::string & floor_id,
      const std::string & map_id)
    {
      auto snapshot = delegate(building_id, floor_id, map_id);
      if (snapshot) {
        snapshot->asset_epoch =
          std::numeric_limits<std::uint64_t>::max();
      }
      return snapshot;
    });

  ElevatorConfigurationCommand save;
  save.type = ElevatorConfigurationCommandType::kSaveDraft;
  save.building_id = "B3";
  save.document = valid_document();
  const auto saved = module.execute(save);

  ASSERT_EQ(saved.status, 200) << saved.body;
  ASSERT_TRUE(json_bool_value(saved.body, "valid_for_publish", false));
  const auto inspected = module.query(
    ElevatorConfigurationQuery{"B3", "", "", ""});
  ASSERT_EQ(inspected.status, 200) << inspected.body;
  EXPECT_NE(
    inspected.body.find("\"map_asset_epoch\":18446744073709551615"),
    std::string::npos) << inspected.body;
}

TEST(ElevatorConfigurationModule, PreservesQuotedScalarTypesInJsonQueries)
{
  TemporaryDirectory temporary;
  ElevatorConfigurationModule module(temporary.path(), test_resolver());
  auto document = valid_document();
  const auto object_start = document.find('{');
  ASSERT_NE(object_start, std::string::npos);
  document.insert(
    object_start + 1U,
    R"json("commissioning_code":"01","truth_label":"true","signed_label":"+1",)json");

  ElevatorConfigurationCommand save;
  save.type = ElevatorConfigurationCommandType::kSaveDraft;
  save.building_id = "B3";
  save.document = document;
  const auto saved = module.execute(save);
  ASSERT_EQ(saved.status, 200) << saved.body;

  const auto inspected = module.query(
    ElevatorConfigurationQuery{"B3", "", "", ""});
  ASSERT_EQ(inspected.status, 200) << inspected.body;
  EXPECT_NE(
    inspected.body.find("\"commissioning_code\":\"01\""),
    std::string::npos) << inspected.body;
  EXPECT_NE(
    inspected.body.find("\"truth_label\":\"true\""),
    std::string::npos) << inspected.body;
  EXPECT_NE(
    inspected.body.find("\"signed_label\":\"+1\""),
    std::string::npos) << inspected.body;

  ElevatorConfigurationCommand publish;
  publish.type = ElevatorConfigurationCommandType::kPublish;
  publish.building_id = "B3";
  publish.expected_draft_revision =
    json_string_value(saved.body, "draft_revision").value_or("");
  const auto published = module.execute(publish);
  ASSERT_EQ(published.status, 201) << published.body;
  const auto release_id =
    json_string_value(published.body, "release_id");
  ASSERT_TRUE(release_id.has_value());

  const auto released = module.query(
    ElevatorConfigurationQuery{"B3", *release_id, "", ""});
  ASSERT_EQ(released.status, 200) << released.body;
  EXPECT_NE(
    released.body.find("\"commissioning_code\":\"01\""),
    std::string::npos) << released.body;
  EXPECT_NE(
    released.body.find("\"truth_label\":\"true\""),
    std::string::npos) << released.body;
  EXPECT_NE(
    released.body.find("\"signed_label\":\"+1\""),
    std::string::npos) << released.body;
}

TEST(ElevatorConfigurationModule, RejectsYamlOnlyNumberSyntax)
{
  TemporaryDirectory temporary;
  ElevatorConfigurationModule module(temporary.path(), test_resolver());
  auto document = valid_document();
  const auto first_x = document.find("\"x\": -1.0");
  ASSERT_NE(first_x, std::string::npos);
  document.replace(
    first_x, std::string("\"x\": -1.0").size(),
    "\"x\": +1");

  ElevatorConfigurationCommand save;
  save.type = ElevatorConfigurationCommandType::kSaveDraft;
  save.building_id = "B3";
  save.document = document;
  const auto rejected = module.execute(save);

  EXPECT_EQ(rejected.status, 400) << rejected.body;
  EXPECT_EQ(rejected.code, "INVALID_DOCUMENT");
  EXPECT_NE(rejected.body.find("strict JSON"), std::string::npos);
}

TEST(ElevatorConfigurationModule, RejectsDuplicateKeysAtEveryConfigurationLevel)
{
  struct DuplicateCase
  {
    std::string marker;
    std::string duplicate_member;
  };
  const std::vector<DuplicateCase> cases{
    {"\"building_id\": \"B3\",", "\"elevators\":[],"},
    {"\"elevator_id\": \"elevator_1\",", "\"floors\":[],"},
    {
      "\"map_id\": \"map_f3\",",
      "\"map_asset_epoch\":1,\"map_asset_epoch\":2,"
    },
    {
      "\"map_id\": \"map_f3\",",
      "\"map_asset_digest\":\"" + canonical_test_digest('1') +
      "\",\"map_asset_digest\":\"" + canonical_test_digest('2') + "\","
    },
    {"\"hall_call\": {", "\"x\":0,"},
  };

  for (const auto & duplicate : cases) {
    SCOPED_TRACE(duplicate.duplicate_member);
    TemporaryDirectory temporary;
    ElevatorConfigurationModule module(temporary.path(), test_resolver());
    auto document = valid_document();
    const auto marker = document.find(duplicate.marker);
    ASSERT_NE(marker, std::string::npos);
    document.insert(marker + duplicate.marker.size(), duplicate.duplicate_member);

    ElevatorConfigurationCommand save;
    save.type = ElevatorConfigurationCommandType::kSaveDraft;
    save.building_id = "B3";
    save.document = document;
    const auto rejected = module.execute(save);

    EXPECT_EQ(rejected.status, 400) << rejected.body;
    EXPECT_EQ(rejected.code, "INVALID_DOCUMENT");
    EXPECT_NE(
      rejected.body.find("duplicate decoded object key"),
      std::string::npos) << rejected.body;
    EXPECT_FALSE(fs::exists(
        temporary.path() / "B3" / ".elevator_config" / "draft.json"));
  }
}

TEST(ElevatorConfigurationModule, RejectsEscapedEquivalentObjectKeys)
{
  const std::vector<std::pair<std::string, std::string>> cases{
    {
      "\"building_id\": \"B3\",",
      R"json("\u0065levators":[],)json"
    },
    {
      "\"map_id\": \"map_f3\",",
      R"json("map_asset_epoch":1,"\u006dap_asset_epoch":2,)json"
    },
  };

  for (const auto & duplicate : cases) {
    SCOPED_TRACE(duplicate.second);
    TemporaryDirectory temporary;
    ElevatorConfigurationModule module(temporary.path(), test_resolver());
    auto document = valid_document();
    const auto marker = document.find(duplicate.first);
    ASSERT_NE(marker, std::string::npos);
    document.insert(marker + duplicate.first.size(), duplicate.second);

    ElevatorConfigurationCommand save;
    save.type = ElevatorConfigurationCommandType::kSaveDraft;
    save.building_id = "B3";
    save.document = document;
    const auto rejected = module.execute(save);

    EXPECT_EQ(rejected.status, 400) << rejected.body;
    EXPECT_NE(
      rejected.body.find("duplicate decoded object key"),
      std::string::npos) << rejected.body;
  }
}

TEST(ElevatorConfigurationModule, EnforcesStrictJsonDepthBoundary)
{
  const auto add_nested_extension =
    [](std::string document, const std::size_t array_depth)
    {
      const auto root = document.find('{');
      EXPECT_NE(root, std::string::npos);
      std::string extension = "\"depth_probe\":";
      extension.append(array_depth, '[');
      extension += "0";
      extension.append(array_depth, ']');
      extension += ",";
      document.insert(root + 1U, extension);
      return document;
    };

  {
    TemporaryDirectory temporary;
    ElevatorConfigurationModule module(temporary.path(), test_resolver());
    ElevatorConfigurationCommand save;
    save.type = ElevatorConfigurationCommandType::kSaveDraft;
    save.building_id = "B3";
    // The root object consumes one level; 63 nested arrays reach exactly 64.
    save.document = add_nested_extension(valid_document(), 63U);
    const auto accepted = module.execute(save);
    EXPECT_EQ(accepted.status, 200) << accepted.body;
  }
  {
    TemporaryDirectory temporary;
    ElevatorConfigurationModule module(temporary.path(), test_resolver());
    ElevatorConfigurationCommand save;
    save.type = ElevatorConfigurationCommandType::kSaveDraft;
    save.building_id = "B3";
    save.document = add_nested_extension(valid_document(), 64U);
    const auto rejected = module.execute(save);
    EXPECT_EQ(rejected.status, 400) << rejected.body;
    EXPECT_NE(
      rejected.body.find("maximum depth of 64"),
      std::string::npos) << rejected.body;
  }
}

TEST(ElevatorConfigurationModule, EnforcesStrictJsonSizeBoundary)
{
  constexpr std::size_t maximum_document_bytes =
    2U * 1024U * 1024U;
  {
    TemporaryDirectory temporary;
    ElevatorConfigurationModule module(temporary.path(), test_resolver());
    auto document = valid_document();
    ASSERT_LT(document.size(), maximum_document_bytes);
    document.append(maximum_document_bytes - document.size(), ' ');
    ElevatorConfigurationCommand save;
    save.type = ElevatorConfigurationCommandType::kSaveDraft;
    save.building_id = "B3";
    save.document = std::move(document);
    const auto accepted = module.execute(save);
    EXPECT_EQ(accepted.status, 200) << accepted.body;
  }
  {
    TemporaryDirectory temporary;
    ElevatorConfigurationModule module(temporary.path(), test_resolver());
    ElevatorConfigurationCommand save;
    save.type = ElevatorConfigurationCommandType::kSaveDraft;
    save.building_id = "B3";
    save.document.assign(maximum_document_bytes + 1U, ' ');
    const auto rejected = module.execute(save);
    EXPECT_EQ(rejected.status, 400) << rejected.body;
    EXPECT_EQ(rejected.code, "INVALID_DOCUMENT");
  }
}

TEST(ElevatorConfigurationModule, RejectsDraftContentThatNoLongerMatchesRevision)
{
  TemporaryDirectory temporary;
  ElevatorConfigurationModule module(temporary.path(), test_resolver());

  ElevatorConfigurationCommand save;
  save.type = ElevatorConfigurationCommandType::kSaveDraft;
  save.building_id = "B3";
  save.document = valid_document();
  const auto saved = module.execute(save);
  ASSERT_EQ(saved.status, 200) << saved.body;
  const auto revision =
    json_string_value(saved.body, "draft_revision");
  ASSERT_TRUE(revision.has_value());

  const auto configuration =
    temporary.path() / "B3" / ".elevator_config" / "drafts" /
    *revision / "configuration.yaml";
  {
    std::ofstream output(configuration, std::ios::trunc);
    output << "{}\n";
  }

  ElevatorConfigurationCommand publish;
  publish.type = ElevatorConfigurationCommandType::kPublish;
  publish.building_id = "B3";
  publish.expected_draft_revision = *revision;
  const auto rejected = module.execute(publish);

  EXPECT_EQ(rejected.status, 500) << rejected.body;
  EXPECT_EQ(rejected.code, "ELEVATOR_CONFIG_IO_ERROR");
  EXPECT_NE(rejected.body.find("immutable revision"), std::string::npos);
}

TEST(ElevatorConfigurationModule, RejectsDuplicateKeysInPersistedDraftDocuments)
{
  TemporaryDirectory temporary;
  ElevatorConfigurationModule module(temporary.path(), test_resolver());

  ElevatorConfigurationCommand save;
  save.type = ElevatorConfigurationCommandType::kSaveDraft;
  save.building_id = "B3";
  save.document = valid_document();
  const auto saved = module.execute(save);
  ASSERT_EQ(saved.status, 200) << saved.body;
  const auto revision =
    json_string_value(saved.body, "draft_revision");
  ASSERT_TRUE(revision.has_value());

  const auto configuration =
    temporary.path() / "B3" / ".elevator_config" / "drafts" /
    *revision / "configuration.yaml";
  auto text = read_text_file(configuration);
  const auto epoch = text.find("\"map_asset_epoch\":33");
  ASSERT_NE(epoch, std::string::npos);
  text.insert(epoch, "\"map_asset_epoch\":33,");
  {
    std::ofstream output(configuration, std::ios::trunc);
    output << text;
  }

  ElevatorConfigurationCommand publish;
  publish.type = ElevatorConfigurationCommandType::kPublish;
  publish.building_id = "B3";
  publish.expected_draft_revision = *revision;
  const auto rejected = module.execute(publish);

  EXPECT_EQ(rejected.status, 500) << rejected.body;
  EXPECT_EQ(rejected.code, "ELEVATOR_CONFIG_IO_ERROR");
  EXPECT_NE(
      rejected.body.find("duplicate decoded object key"),
      std::string::npos) << rejected.body;
}

TEST(ElevatorConfigurationModule, RejectsDuplicateKeysInPersistedReleaseConfiguration)
{
  TemporaryDirectory temporary;
  ElevatorConfigurationModule module(temporary.path(), test_resolver());

  ElevatorConfigurationCommand save;
  save.type = ElevatorConfigurationCommandType::kSaveDraft;
  save.building_id = "B3";
  save.document = valid_document();
  const auto saved = module.execute(save);
  ASSERT_EQ(saved.status, 200) << saved.body;
  ElevatorConfigurationCommand publish;
  publish.type = ElevatorConfigurationCommandType::kPublish;
  publish.building_id = "B3";
  publish.expected_draft_revision =
    json_string_value(saved.body, "draft_revision").value_or("");
  const auto published = module.execute(publish);
  ASSERT_EQ(published.status, 201) << published.body;
  const auto release_id =
    json_string_value(published.body, "release_id");
  ASSERT_TRUE(release_id.has_value());

  const auto configuration =
    temporary.path() / "B3" / ".elevator_config" / "releases" /
    *release_id / "configuration.yaml";
  auto text = read_text_file(configuration);
  const auto digest = text.find("\"map_asset_digest\":");
  ASSERT_NE(digest, std::string::npos);
  text.insert(
    digest,
    "\"map_asset_digest\":\"" + canonical_test_digest('3') + "\",");
  {
    std::ofstream output(configuration, std::ios::trunc);
    output << text;
  }

  const auto rejected = module.query(
    ElevatorConfigurationQuery{"B3", *release_id, "", ""});
  EXPECT_EQ(rejected.status, 500) << rejected.body;
  EXPECT_EQ(rejected.code, "ELEVATOR_CONFIG_IO_ERROR");
  EXPECT_NE(
    rejected.body.find("duplicate decoded object key"),
    std::string::npos) << rejected.body;
}

TEST(ElevatorConfigurationModule, RejectsDuplicateKeysInReleaseManifest)
{
  TemporaryDirectory temporary;
  ElevatorConfigurationModule module(temporary.path(), test_resolver());

  ElevatorConfigurationCommand save;
  save.type = ElevatorConfigurationCommandType::kSaveDraft;
  save.building_id = "B3";
  save.document = valid_document();
  const auto saved = module.execute(save);
  ASSERT_EQ(saved.status, 200) << saved.body;
  ElevatorConfigurationCommand publish;
  publish.type = ElevatorConfigurationCommandType::kPublish;
  publish.building_id = "B3";
  publish.expected_draft_revision =
    json_string_value(saved.body, "draft_revision").value_or("");
  const auto published = module.execute(publish);
  ASSERT_EQ(published.status, 201) << published.body;
  const auto release_id =
    json_string_value(published.body, "release_id");
  ASSERT_TRUE(release_id.has_value());

  const auto manifest =
    temporary.path() / "B3" / ".elevator_config" / "releases" /
    *release_id / "manifest.json";
  auto text = read_text_file(manifest);
  const auto root = text.find('{');
  ASSERT_NE(root, std::string::npos);
  text.insert(root + 1U, R"json("\u0073chema_version":1,)json");
  {
    std::ofstream output(manifest, std::ios::trunc);
    output << text;
  }

  const auto rejected = module.query(
    ElevatorConfigurationQuery{"B3", *release_id, "", ""});
  EXPECT_EQ(rejected.status, 500) << rejected.body;
  EXPECT_EQ(rejected.code, "ELEVATOR_CONFIG_IO_ERROR");
  EXPECT_NE(
    rejected.body.find("duplicate decoded object key"),
    std::string::npos) << rejected.body;
}

TEST(ElevatorConfigurationModule, RejectsDuplicateKeysInCurrentMetadata)
{
  TemporaryDirectory temporary;
  ElevatorConfigurationModule module(temporary.path(), test_resolver());

  ElevatorConfigurationCommand save;
  save.type = ElevatorConfigurationCommandType::kSaveDraft;
  save.building_id = "B3";
  save.document = valid_document();
  const auto saved = module.execute(save);
  ASSERT_EQ(saved.status, 200) << saved.body;
  ElevatorConfigurationCommand publish;
  publish.type = ElevatorConfigurationCommandType::kPublish;
  publish.building_id = "B3";
  publish.expected_draft_revision =
    json_string_value(saved.body, "draft_revision").value_or("");
  const auto published = module.execute(publish);
  ASSERT_EQ(published.status, 201) << published.body;
  const auto release_id =
    json_string_value(published.body, "release_id");
  ASSERT_TRUE(release_id.has_value());

  const auto metadata =
    temporary.path() / "B3" / ".elevator_config" / "releases" /
    *release_id / "current.json";
  auto text = read_text_file(metadata);
  const auto generation = text.find("\"generation\":1");
  ASSERT_NE(generation, std::string::npos);
  text.insert(generation, "\"generation\":1,");
  {
    std::ofstream output(metadata, std::ios::trunc);
    output << text;
  }

  const auto rejected = module.query(
    ElevatorConfigurationQuery{"B3", "", "", ""});
  EXPECT_EQ(rejected.status, 500) << rejected.body;
  EXPECT_EQ(rejected.code, "ELEVATOR_CONFIG_IO_ERROR");
  EXPECT_NE(
    rejected.body.find("duplicate decoded object key"),
    std::string::npos) << rejected.body;
}

TEST(ElevatorConfigurationModule, ReservesInternalPoseNamespace)
{
  EXPECT_TRUE(is_elevator_internal_pose_type("elevator_internal"));
  EXPECT_TRUE(is_elevator_internal_pose_type("ELEVATOR_INTERNAL"));
  EXPECT_FALSE(is_elevator_internal_pose_type("delivery_point"));
  EXPECT_TRUE(is_reserved_elevator_pose_id("eip_0123456789abcdef_cabin"));
  EXPECT_FALSE(is_reserved_elevator_pose_id("delivery_1"));
}

TEST(ElevatorConfigurationModule, ReportsTamperedReleaseManifestBindings)
{
  TemporaryDirectory temporary;
  ElevatorConfigurationModule module(temporary.path(), test_resolver());

  ElevatorConfigurationCommand save;
  save.type = ElevatorConfigurationCommandType::kSaveDraft;
  save.building_id = "B3";
  save.document = valid_document();
  const auto saved = module.execute(save);
  ASSERT_EQ(saved.status, 200) << saved.body;

  ElevatorConfigurationCommand publish;
  publish.type = ElevatorConfigurationCommandType::kPublish;
  publish.building_id = "B3";
  publish.expected_draft_revision =
    json_string_value(saved.body, "draft_revision").value_or("");
  const auto published = module.execute(publish);
  ASSERT_EQ(published.status, 201) << published.body;
  const auto release_id =
    json_string_value(published.body, "release_id");
  ASSERT_TRUE(release_id.has_value());

  const auto manifest =
    temporary.path() / "B3" / ".elevator_config" / "releases" /
    *release_id / "manifest.json";
  auto text = read_text_file(manifest);
  const auto recorded =
    "\"asset_digest\":\"" + canonical_test_digest('3') + "\"";
  const auto tampered =
    "\"asset_digest\":\"" + canonical_test_digest('f') + "\"";
  const auto digest = text.find(recorded);
  ASSERT_NE(digest, std::string::npos);
  text.replace(digest, recorded.size(), tampered);
  {
    std::ofstream output(manifest, std::ios::trunc);
    output << text;
  }

  const auto explicit_query = module.query(
    ElevatorConfigurationQuery{"B3", *release_id, "", ""});
  EXPECT_EQ(explicit_query.status, 500) << explicit_query.body;
  EXPECT_EQ(explicit_query.code, "ELEVATOR_CONFIG_IO_ERROR");

  const auto inventory = module.query(
    ElevatorConfigurationQuery{"B3", "", "", ""});
  ASSERT_EQ(inventory.status, 200) << inventory.body;
  EXPECT_NE(
    inventory.body.find(
      "\"release_integrity_errors\":[\"" + *release_id + "\"]"),
      std::string::npos) << inventory.body;
}

TEST(ElevatorConfigurationModule, ReportsTamperedReleaseDigestContract)
{
  TemporaryDirectory temporary;
  ElevatorConfigurationModule module(temporary.path(), test_resolver());

  ElevatorConfigurationCommand save;
  save.type = ElevatorConfigurationCommandType::kSaveDraft;
  save.building_id = "B3";
  save.document = valid_document();
  const auto saved = module.execute(save);
  ASSERT_EQ(saved.status, 200) << saved.body;

  ElevatorConfigurationCommand publish;
  publish.type = ElevatorConfigurationCommandType::kPublish;
  publish.building_id = "B3";
  publish.expected_draft_revision =
    json_string_value(saved.body, "draft_revision").value_or("");
  const auto published = module.execute(publish);
  ASSERT_EQ(published.status, 201) << published.body;
  const auto release_id =
    json_string_value(published.body, "release_id");
  ASSERT_TRUE(release_id.has_value());

  const auto manifest =
    temporary.path() / "B3" / ".elevator_config" / "releases" /
    *release_id / "manifest.json";
  auto text = read_text_file(manifest);
  const std::string recorded =
    "\"asset_digest_contract\":\"njrh-map-asset-bundle-v1\"";
  const std::string tampered =
    "\"asset_digest_contract\":\"other-contract\"";
  const auto contract = text.find(recorded);
  ASSERT_NE(contract, std::string::npos);
  text.replace(contract, recorded.size(), tampered);
  {
    std::ofstream output(manifest, std::ios::trunc);
    output << text;
  }

  const auto explicit_query = module.query(
    ElevatorConfigurationQuery{"B3", *release_id, "", ""});
  EXPECT_EQ(explicit_query.status, 500) << explicit_query.body;
  EXPECT_EQ(explicit_query.code, "ELEVATOR_CONFIG_IO_ERROR");

  const auto inventory = module.query(
    ElevatorConfigurationQuery{"B3", "", "", ""});
  ASSERT_EQ(inventory.status, 200) << inventory.body;
  EXPECT_NE(
    inventory.body.find(
      "\"release_integrity_errors\":[\"" + *release_id + "\"]"),
    std::string::npos) << inventory.body;
}

TEST(ElevatorConfigurationModule, RejectsNonCanonicalMapAssetDigest)
{
  TemporaryDirectory temporary;
  ElevatorConfigurationModule module(
    temporary.path(),
    [](
      const std::string & building_id,
      const std::string & floor_id,
      const std::string & map_id)
    -> std::optional<ElevatorFloorAssetSnapshot>
    {
      auto snapshot = test_resolver()(building_id, floor_id, map_id);
      if (snapshot) {
        snapshot->asset_digest = "fnv1a64-0123456789abcdef";
      }
      return snapshot;
    });

  ElevatorConfigurationCommand save;
  save.type = ElevatorConfigurationCommandType::kSaveDraft;
  save.building_id = "B3";
  save.document = valid_document();
  const auto result = module.execute(save);

  ASSERT_EQ(result.status, 200) << result.body;
  EXPECT_FALSE(json_bool_value(result.body, "valid_for_publish", true));
  EXPECT_NE(
    result.body.find("NON_CANONICAL_MAP_ASSET_DIGEST"),
    std::string::npos);
}

TEST(ElevatorConfigurationModule, RejectsManifestThatClaimsRuntimeApplication)
{
  TemporaryDirectory temporary;
  ElevatorConfigurationModule module(temporary.path(), test_resolver());

  ElevatorConfigurationCommand save;
  save.type = ElevatorConfigurationCommandType::kSaveDraft;
  save.building_id = "B3";
  save.document = valid_document();
  const auto saved = module.execute(save);
  ASSERT_EQ(saved.status, 200) << saved.body;

  ElevatorConfigurationCommand publish;
  publish.type = ElevatorConfigurationCommandType::kPublish;
  publish.building_id = "B3";
  publish.expected_draft_revision =
    json_string_value(saved.body, "draft_revision").value_or("");
  const auto published = module.execute(publish);
  ASSERT_EQ(published.status, 201) << published.body;
  const auto release_id =
    json_string_value(published.body, "release_id");
  ASSERT_TRUE(release_id.has_value());

  const auto manifest =
    temporary.path() / "B3" / ".elevator_config" / "releases" /
    *release_id / "manifest.json";
  auto text = read_text_file(manifest);
  const auto runtime_applied =
    text.find("\"runtime_applied\":false");
  ASSERT_NE(runtime_applied, std::string::npos);
  text.replace(
    runtime_applied,
    std::string("\"runtime_applied\":false").size(),
    "\"runtime_applied\":true");
  {
    std::ofstream output(manifest, std::ios::trunc);
    output << text;
  }

  const auto explicit_query = module.query(
    ElevatorConfigurationQuery{"B3", *release_id, "", ""});
  EXPECT_EQ(explicit_query.status, 500) << explicit_query.body;
  EXPECT_EQ(explicit_query.code, "ELEVATOR_CONFIG_IO_ERROR");

  const auto inventory = module.query(
    ElevatorConfigurationQuery{"B3", "", "", ""});
  ASSERT_EQ(inventory.status, 200) << inventory.body;
  EXPECT_NE(
    inventory.body.find(
      "\"release_integrity_errors\":[\"" + *release_id + "\"]"),
    std::string::npos) << inventory.body;
}

#ifndef _WIN32
TEST(ElevatorConfigurationModule, ExactRetryCompletesInterruptedSelectorSwitch)
{
  TemporaryDirectory temporary;
  ElevatorConfigurationModule module(temporary.path(), test_resolver());

  ElevatorConfigurationCommand save;
  save.type = ElevatorConfigurationCommandType::kSaveDraft;
  save.building_id = "B3";
  save.document = valid_document();
  save.actor_id = "retrying_commissioner";
  const auto saved = module.execute(save);
  ASSERT_EQ(saved.status, 200) << saved.body;

  ElevatorConfigurationCommand publish;
  publish.type = ElevatorConfigurationCommandType::kPublish;
  publish.building_id = "B3";
  publish.expected_draft_revision =
    json_string_value(saved.body, "draft_revision").value_or("");
  publish.actor_id = "retrying_commissioner";
  const auto first = module.execute(publish);
  ASSERT_EQ(first.status, 201) << first.body;
  const auto release_id = json_string_value(first.body, "release_id");
  ASSERT_TRUE(release_id.has_value());

  const auto config_root =
    temporary.path() / "B3" / ".elevator_config";
  ASSERT_TRUE(fs::remove(config_root / "current"));
  std::this_thread::sleep_for(std::chrono::milliseconds(1100));

  const auto retried = module.execute(publish);
  ASSERT_EQ(retried.status, 201) << retried.body;
  EXPECT_EQ(json_string_value(retried.body, "release_id"), release_id);
  EXPECT_EQ(
    fs::read_symlink(config_root / "current"),
    fs::path("releases") / *release_id);
}

TEST(ElevatorConfigurationModule, RejectsSymlinkedDraftStorageEscape)
{
  TemporaryDirectory temporary;
  const auto maps_root = temporary.path() / "maps";
  const auto config_root = maps_root / "B3" / ".elevator_config";
  const auto outside = temporary.path() / "outside";
  fs::create_directories(config_root);
  fs::create_directories(outside);
  fs::create_directory_symlink(outside, config_root / "drafts");
  ElevatorConfigurationModule module(maps_root, test_resolver());

  ElevatorConfigurationCommand save;
  save.type = ElevatorConfigurationCommandType::kSaveDraft;
  save.building_id = "B3";
  save.document = valid_document();
  const auto rejected = module.execute(save);

  EXPECT_EQ(rejected.status, 500) << rejected.body;
  EXPECT_EQ(rejected.code, "ELEVATOR_CONFIG_IO_ERROR");
  EXPECT_TRUE(fs::is_empty(outside));
}

TEST(ElevatorConfigurationModule, RejectsInTreeAliasForDraftStore)
{
  TemporaryDirectory temporary;
  const auto maps_root = temporary.path() / "maps";
  const auto config_root = maps_root / "B3" / ".elevator_config";
  const auto aliased_store = config_root / "drafts-real";
  fs::create_directories(aliased_store);
  fs::create_directory_symlink("drafts-real", config_root / "drafts");
  ElevatorConfigurationModule module(maps_root, test_resolver());

  ElevatorConfigurationCommand save;
  save.type = ElevatorConfigurationCommandType::kSaveDraft;
  save.building_id = "B3";
  save.document = valid_document();
  const auto rejected = module.execute(save);

  EXPECT_EQ(rejected.status, 500) << rejected.body;
  EXPECT_EQ(rejected.code, "ELEVATOR_CONFIG_IO_ERROR");
  EXPECT_TRUE(fs::is_empty(aliased_store));
}

TEST(ElevatorConfigurationModule, RejectsInTreeAliasForReleaseStore)
{
  TemporaryDirectory temporary;
  ElevatorConfigurationModule module(temporary.path(), test_resolver());

  ElevatorConfigurationCommand save;
  save.type = ElevatorConfigurationCommandType::kSaveDraft;
  save.building_id = "B3";
  save.document = valid_document();
  const auto saved = module.execute(save);
  ASSERT_EQ(saved.status, 200) << saved.body;

  const auto config_root =
    temporary.path() / "B3" / ".elevator_config";
  fs::create_directories(config_root / "releases-real");
  fs::create_directory_symlink(
    "releases-real", config_root / "releases");

  ElevatorConfigurationCommand publish;
  publish.type = ElevatorConfigurationCommandType::kPublish;
  publish.building_id = "B3";
  publish.expected_draft_revision =
    json_string_value(saved.body, "draft_revision").value_or("");
  const auto rejected = module.execute(publish);

  EXPECT_EQ(rejected.status, 500) << rejected.body;
  EXPECT_EQ(rejected.code, "ELEVATOR_CONFIG_IO_ERROR");
  EXPECT_TRUE(fs::is_empty(config_root / "releases-real"));
  EXPECT_FALSE(fs::exists(config_root / "current"));
}

TEST(ElevatorConfigurationModule, RejectsSymlinkedReleaseFiles)
{
  TemporaryDirectory temporary;
  ElevatorConfigurationModule module(temporary.path(), test_resolver());

  ElevatorConfigurationCommand save;
  save.type = ElevatorConfigurationCommandType::kSaveDraft;
  save.building_id = "B3";
  save.document = valid_document();
  const auto saved = module.execute(save);
  ASSERT_EQ(saved.status, 200) << saved.body;

  ElevatorConfigurationCommand publish;
  publish.type = ElevatorConfigurationCommandType::kPublish;
  publish.building_id = "B3";
  publish.expected_draft_revision =
    json_string_value(saved.body, "draft_revision").value_or("");
  const auto published = module.execute(publish);
  ASSERT_EQ(published.status, 201) << published.body;
  const auto release_id =
    json_string_value(published.body, "release_id");
  ASSERT_TRUE(release_id.has_value());

  const auto outside = temporary.path() / "outside_configuration.yaml";
  {
    std::ofstream output(outside);
    output << valid_document();
  }
  const auto configuration =
    temporary.path() / "B3" / ".elevator_config" / "releases" /
    *release_id / "configuration.yaml";
  ASSERT_TRUE(fs::remove(configuration));
  fs::create_symlink(outside, configuration);

  const auto rejected = module.query(
    ElevatorConfigurationQuery{"B3", *release_id, "", ""});
  EXPECT_EQ(rejected.status, 500) << rejected.body;
  EXPECT_EQ(rejected.code, "ELEVATOR_CONFIG_IO_ERROR");
}
#endif

}  // namespace
}  // namespace robot_api_server
