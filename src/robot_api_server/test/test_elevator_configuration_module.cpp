#include <chrono>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "gtest/gtest.h"
#include <yaml-cpp/yaml.h>

#include "robot_api_server/elevator_configuration_module.hpp"
#include "robot_api_server/file_utils.hpp"
#include "robot_api_server/http_common.hpp"
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

std::string valid_document(const double f4_exit_x = -0.6)
{
  return R"json({
    "schema_version": 1,
    "building_id": "B3",
    "elevators": [{
      "elevator_id": "elevator_1",
      "display_name": "1号电梯",
      "floors": [{
        "floor_id": "F3",
        "map_id": "map_f3",
        "poses": {
          "hall_call": {"x": -1.0, "y": 0.0, "yaw": 0.0},
          "hall_wait": {"x": -0.8, "y": 0.0, "yaw": 0.0},
          "doorway": {"x": 0.0, "y": 0.0, "yaw": 0.0},
          "cabin": {"x": 0.8, "y": 0.0, "yaw": 3.1415926},
          "exit": {"x": -0.6, "y": 0.0, "yaw": 0.0}
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
          "hall_call": {"x": -1.0, "y": 0.0, "yaw": 0.0},
          "hall_wait": {"x": -0.8, "y": 0.0, "yaw": 0.0},
          "doorway": {"x": 0.0, "y": 0.0, "yaw": 0.0},
          "cabin": {"x": 0.8, "y": 0.0, "yaw": 3.1415926},
          "exit": {"x": )json" + std::to_string(f4_exit_x) + R"json(, "y": 0.0, "yaw": 0.0}
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

TEST(ElevatorConfigurationModule, KeepsIncompleteDraftButRejectsPublish)
{
  TemporaryDirectory temporary;
  ElevatorConfigurationModule module(temporary.path(), test_resolver());

  ElevatorConfigurationCommand save;
  save.type = ElevatorConfigurationCommandType::kSaveDraft;
  save.building_id = "B3";
  save.document =
    R"json({"schema_version":1,"building_id":"B3","elevators":[]})json";
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
    R"json({"schema_version":1,"building_id":"B3","elevators":[)json";
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
  stale_save.expected_draft_revision = "draft-v1-stale";
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

TEST(ElevatorConfigurationModule, DoesNotSilentlyDefaultMalformedClearance)
{
  TemporaryDirectory temporary;
  ElevatorConfigurationModule module(temporary.path(), test_resolver());
  auto document = valid_document();
  const auto clearance = document.find("\"clearance_m\": 0.05");
  ASSERT_NE(clearance, std::string::npos);
  document.replace(
    clearance, std::string("\"clearance_m\": 0.05").size(),
    "\"clearance_m\": \"invalid\"");

  ElevatorConfigurationCommand save;
  save.type = ElevatorConfigurationCommandType::kSaveDraft;
  save.building_id = "B3";
  save.document = document;
  const auto rejected = module.execute(save);

  ASSERT_EQ(rejected.status, 200) << rejected.body;
  EXPECT_FALSE(json_bool_value(rejected.body, "valid_for_publish", true));
  EXPECT_NE(
    rejected.body.find("INVALID_THRESHOLD_CLEARANCE"),
    std::string::npos);
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
