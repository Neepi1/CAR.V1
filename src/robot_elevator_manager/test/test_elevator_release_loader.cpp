#include <gtest/gtest.h>

#include <array>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

#include "robot_elevator_manager/elevator_release_loader.hpp"

namespace robot_elevator_manager
{
namespace
{

namespace fs = std::filesystem;

constexpr const char * kSourceDigest =
  "sha256:0000000000000000000000000000000000000000000000000000000000000001";
constexpr const char * kTargetDigest =
  "sha256:0000000000000000000000000000000000000000000000000000000000000002";
constexpr std::uint64_t kSourceEpoch = 101U;
constexpr std::uint64_t kTargetEpoch = 202U;

std::uint64_t fnv1a64(const std::string & value)
{
  // Must match robot_api_server/storage_models.cpp's legacy release identity.
  std::uint64_t hash = 1469598103934665603ULL;
  for (const unsigned char character : value) {
    hash ^= static_cast<std::uint64_t>(character);
    hash *= 1099511628211ULL;
  }
  return hash;
}

std::string fixed_hex(const std::uint64_t value, const std::size_t width)
{
  std::ostringstream out;
  out << std::hex << std::nouppercase << std::setw(static_cast<int>(width))
      << std::setfill('0') << value;
  return out.str();
}

struct ReleaseTexts
{
  std::string configuration;
  std::string topology;
  std::string internal_poses;
};

ReleaseTexts valid_release_texts()
{
  ReleaseTexts texts;
  texts.configuration =
    "schema_version: 1\n"
    "building_id: building_1\n"
    "elevators:\n"
    "  - elevator_id: elevator_west\n"
    "    floors:\n"
    "      - floor_id: F1\n"
    "        map_id: map_f1\n"
    "        map_asset_epoch: 101\n"
    "        map_asset_digest: " + std::string(kSourceDigest) + "\n"
    "        poses:\n"
    "          hall_call: {x: 1.0, y: 1.1, yaw: 0.1}\n"
    "          hall_wait: {x: 1.2, y: 1.3, yaw: 0.2}\n"
    "          doorway: {x: 1.4, y: 1.5, yaw: 0.3}\n"
    "          cabin: {x: 1.6, y: 1.7, yaw: 0.4}\n"
    "          exit: {x: 1.8, y: 1.9, yaw: 0.5}\n"
    "        threshold:\n"
    "          left: [0.0, -0.6]\n"
    "          right: [0.0, 0.6]\n"
    "          cabin_reference: [1.0, 0.0]\n"
    "          clearance_m: 0.05\n"
    "          jamb_clearance_m: 0.05\n"
    "      - floor_id: F2\n"
    "        map_id: map_f2\n"
    "        map_asset_epoch: 202\n"
    "        map_asset_digest: " + std::string(kTargetDigest) + "\n"
    "        poses:\n"
    "          hall_call: {x: 2.0, y: 2.1, yaw: 0.1}\n"
    "          hall_wait: {x: 2.2, y: 2.3, yaw: 0.2}\n"
    "          doorway: {x: 2.4, y: 2.5, yaw: 0.3}\n"
    "          cabin: {x: 2.6, y: 2.7, yaw: 0.4}\n"
    "          exit: {x: 2.8, y: 2.9, yaw: 0.5}\n"
    "        threshold:\n"
    "          left: [0.0, -0.7]\n"
    "          right: [0.0, 0.7]\n"
    "          cabin_reference: [1.0, 0.0]\n"
    "          clearance_m: 0.06\n"
    "          jamb_clearance_m: 0.06\n";

  texts.topology =
    "schema_version: 1\n"
    "mock_ports_enabled: false\n"
    "building_id: building_1\n"
    "elevators:\n"
    "  - elevator_id: elevator_west\n"
    "    floors:\n"
    "      - floor_id: F1\n"
    "        map_id: map_f1\n"
    "        poses:\n"
    "          hall_call: f1_west_hall_call\n"
    "          hall_wait: f1_west_hall_wait\n"
    "          doorway: f1_west_doorway\n"
    "          cabin: f1_west_cabin\n"
    "          exit: f1_west_exit\n"
    "        threshold:\n"
    "          left: [0.0, -0.6]\n"
    "          right: [0.0, 0.6]\n"
    "          cabin_reference: [1.0, 0.0]\n"
    "          clearance_m: 0.05\n"
    "          jamb_clearance_m: 0.05\n"
    "      - floor_id: F2\n"
    "        map_id: map_f2\n"
    "        poses:\n"
    "          hall_call: f2_west_hall_call\n"
    "          hall_wait: f2_west_hall_wait\n"
    "          doorway: f2_west_doorway\n"
    "          cabin: f2_west_cabin\n"
    "          exit: f2_west_exit\n"
    "        threshold:\n"
    "          left: [0.0, -0.7]\n"
    "          right: [0.0, 0.7]\n"
    "          cabin_reference: [1.0, 0.0]\n"
    "          clearance_m: 0.06\n"
    "          jamb_clearance_m: 0.06\n";

  const std::array<std::string, 5> roles{
    "hall_call", "hall_wait", "doorway", "cabin", "exit"};
  std::ostringstream internal;
  internal << "schema_version: 1\n"
           << "building_id: building_1\n"
           << "poses:\n";
  for (const auto & floor : {std::string("F1"), std::string("F2")}) {
    const std::string lower = floor == "F1" ? "f1" : "f2";
    const double base = floor == "F1" ? 1.0 : 2.0;
    for (std::size_t index = 0; index < roles.size(); ++index) {
      internal << "  - pose_id: " << lower << "_west_" << roles[index] << "\n"
               << "    type: elevator_internal\n"
               << "    elevator_id: elevator_west\n"
               << "    floor_id: " << floor << "\n"
               << "    map_id: " << (floor == "F1" ? "map_f1" : "map_f2") << "\n"
               << "    role: " << roles[index] << "\n"
               << "    x: " << base + 0.2 * static_cast<double>(index) << "\n"
               << "    y: " << base + 0.1 + 0.2 * static_cast<double>(index) << "\n"
               << "    yaw: " << 0.1 + 0.1 * static_cast<double>(index) << "\n";
    }
  }
  texts.internal_poses = internal.str();
  return texts;
}

void replace_all(
  std::string & value,
  const std::string & from,
  const std::string & to)
{
  std::size_t position = 0U;
  while ((position = value.find(from, position)) != std::string::npos) {
    value.replace(position, from.size(), to);
    position += to.size();
  }
}

ReleaseTexts two_elevator_release_texts()
{
  auto texts = valid_release_texts();
  for (auto * catalog : {&texts.configuration, &texts.topology}) {
    const auto start = catalog->find("  - elevator_id: elevator_west\n");
    EXPECT_NE(start, std::string::npos);
    auto alpha = catalog->substr(start);
    replace_all(alpha, "elevator_west", "elevator_alpha");
    replace_all(alpha, "_west_", "_alpha_");
    catalog->append(alpha);
  }
  const auto internal_start = texts.internal_poses.find("  - pose_id:");
  EXPECT_NE(internal_start, std::string::npos);
  auto alpha_internal = texts.internal_poses.substr(internal_start);
  replace_all(alpha_internal, "elevator_west", "elevator_alpha");
  replace_all(alpha_internal, "_west_", "_alpha_");
  texts.internal_poses.append(alpha_internal);
  return texts;
}

class ReleaseStore
{
public:
  ReleaseStore()
  {
    const auto nonce = std::chrono::steady_clock::now().time_since_epoch().count();
    root_ = fs::temp_directory_path() /
      ("robot_elevator_release_loader_" + std::to_string(nonce));
    config_root_ = root_ / ".elevator_config";
    fs::create_directories(config_root_ / "releases");
  }

  ~ReleaseStore()
  {
    std::error_code ignored;
    fs::remove_all(root_, ignored);
  }

  std::string add_release(
    const ReleaseTexts & texts,
    const std::uint64_t generation = 1U,
    const bool select = true,
    const std::string & parent_release_id = "",
    const std::string & rollback_of = "",
    const std::string & actor_id = "commissioner")
  {
    const std::string content_digest = fixed_hex(
      fnv1a64(texts.configuration + texts.topology + texts.internal_poses), 16U);
    const std::string release_digest = fixed_hex(
      fnv1a64(
        std::to_string(generation) + "\n" + texts.configuration +
        texts.topology + texts.internal_poses),
      16U);
    std::ostringstream release_id;
    release_id << "elevator-config-" << std::setw(6) << std::setfill('0')
               << generation << "-" << release_digest.substr(0U, 12U);

    const fs::path release_root = config_root_ / "releases" / release_id.str();
    fs::create_directories(release_root);
    write(release_root / "configuration.yaml", texts.configuration);
    write(release_root / "elevators.yaml", texts.topology);
    write(release_root / "elevator_internal_poses.yaml", texts.internal_poses);
    write(
      release_root / "validation.json",
      "{\"valid_for_publish\":true,\"issues\":[]}\n");

    std::ostringstream manifest;
    manifest
      << "{\"schema_version\":1,"
      << "\"release_id\":\"" << release_id.str() << "\","
      << "\"parent_release_id\":"
      << (parent_release_id.empty() ?
      "null" : "\"" + parent_release_id + "\"") << ","
      << "\"source_draft_revision\":"
      << (rollback_of.empty() ?
      "\"draft-v1-0123456789abcdef\"" : "null") << ","
      << "\"generation\":" << generation << ","
      << "\"created_at\":\"2026-07-24T00:00:00Z\","
      << "\"actor_id\":\"" << actor_id << "\","
      << "\"rollback_of\":"
      << (rollback_of.empty() ? "null" : "\"" + rollback_of + "\"") << ","
      << "\"configuration_digest_algorithm\":\"fnv1a64\","
      << "\"configuration_digest\":\"" << content_digest << "\","
      << "\"map_bindings\":["
      << "{\"floor_id\":\"F1\",\"map_id\":\"map_f1\","
      << "\"asset_epoch\":101,"
      << "\"asset_digest_contract\":\"njrh-map-asset-bundle-v1\","
      << "\"asset_digest_algorithm\":\"sha256\","
      << "\"asset_digest\":\"" << kSourceDigest << "\"},"
      << "{\"floor_id\":\"F2\",\"map_id\":\"map_f2\","
      << "\"asset_epoch\":202,"
      << "\"asset_digest_contract\":\"njrh-map-asset-bundle-v1\","
      << "\"asset_digest_algorithm\":\"sha256\","
      << "\"asset_digest\":\"" << kTargetDigest << "\"}],"
      << "\"asset_published\":true,\"runtime_applied\":false}\n";
    write(release_root / "manifest.json", manifest.str());

    std::ostringstream current;
    current
      << "{\"schema_version\":1,"
      << "\"release_id\":\"" << release_id.str() << "\","
      << "\"parent_release_id\":"
      << (parent_release_id.empty() ?
      "null" : "\"" + parent_release_id + "\"") << ","
      << "\"generation\":" << generation << ","
      << "\"published_at\":\"2026-07-24T00:00:00Z\","
      << "\"actor_id\":\"" << actor_id << "\",\"rollback_of\":"
      << (rollback_of.empty() ? "null" : "\"" + rollback_of + "\"") << "}\n";
    write(release_root / "current.json", current.str());

    if (select) {
      std::error_code ignored;
      fs::remove(config_root_ / "current", ignored);
      fs::create_directory_symlink(
        fs::path("releases") / release_id.str(), config_root_ / "current");
    }
    return release_id.str();
  }

  const fs::path & config_root() const noexcept
  {
    return config_root_;
  }

  fs::path release_root(const std::string & release_id) const
  {
    return config_root_ / "releases" / release_id;
  }

  void replace_release_file(
    const std::string & release_id,
    const std::string & filename,
    const std::string & value)
  {
    write(release_root(release_id) / filename, value);
  }

  void replace_in_release_file(
    const std::string & release_id,
    const std::string & filename,
    const std::string & from,
    const std::string & to)
  {
    const auto path = release_root(release_id) / filename;
    std::ifstream stream(path, std::ios::binary);
    std::ostringstream contents;
    contents << stream.rdbuf();
    auto value = contents.str();
    const auto position = value.find(from);
    ASSERT_NE(position, std::string::npos) << path;
    value.replace(position, from.size(), to);
    write(path, value);
  }

private:
  static void write(const fs::path & path, const std::string & value)
  {
    std::ofstream stream(path, std::ios::binary);
    ASSERT_TRUE(stream.good()) << path;
    stream.write(value.data(), static_cast<std::streamsize>(value.size()));
    ASSERT_TRUE(stream.good()) << path;
  }

  fs::path root_;
  fs::path config_root_;
};

ElevatorReleaseLoadRequest valid_request(const fs::path & config_root)
{
  ElevatorReleaseLoadRequest request;
  request.config_root = config_root.string();
  request.building_id = "building_1";
  request.source_floor_id = "F1";
  request.source_map_id = "map_f1";
  request.target_floor_id = "F2";
  request.target_map_id = "map_f2";
  return request;
}

TEST(ElevatorReleaseLoader, FreezesThePublisherLegacyFnvIdentityVector)
{
  EXPECT_EQ(fixed_hex(fnv1a64(""), 16U), "14650fb0739d0383");
  EXPECT_EQ(fixed_hex(fnv1a64("a"), 16U), "44bd8ad473cd9906");
  EXPECT_EQ(
    fixed_hex(fnv1a64("robot_api_server/elevator-release-v1"), 16U),
    "de8d3e0625a6104b");
}

TEST(ElevatorReleaseLoader, LoadsOnePinnedValidatedRelease)
{
  ReleaseStore store;
  const auto texts = valid_release_texts();
  const auto release_id = store.add_release(texts);
  const auto expected_configuration_digest = fixed_hex(
    fnv1a64(texts.configuration + texts.topology + texts.internal_poses), 16U);

  const auto result = load_elevator_release(valid_request(store.config_root()));

  ASSERT_TRUE(result.ok()) << result.message;
  ASSERT_TRUE(result.release.has_value());
  EXPECT_EQ(result.release->release_id, release_id);
  EXPECT_EQ(result.release->generation, 1U);
  EXPECT_EQ(
    result.release->configuration_digest,
    expected_configuration_digest);
  EXPECT_EQ(result.release->elevator_id, "elevator_west");
  EXPECT_EQ(result.release->source.map_asset_epoch, kSourceEpoch);
  EXPECT_EQ(result.release->source.map_asset_digest, kSourceDigest);
  EXPECT_EQ(result.release->target.map_asset_epoch, kTargetEpoch);
  EXPECT_EQ(result.release->target.map_asset_digest, kTargetDigest);
  ASSERT_EQ(result.release->source.poses.size(), 5U);
  EXPECT_EQ(result.release->source.poses[0].role, PoseRole::kHallCall);
  EXPECT_EQ(result.release->source.poses[0].pose_id, "f1_west_hall_call");
  EXPECT_DOUBLE_EQ(result.release->source.poses[0].x, 1.0);
  EXPECT_EQ(result.release->source.poses[4].role, PoseRole::kExit);
  EXPECT_EQ(result.release->target.poses[0].pose_id, "f2_west_hall_call");
  EXPECT_DOUBLE_EQ(result.release->target.poses[4].yaw, 0.5);
}

TEST(ElevatorReleaseLoader, AcceptsPublisherNormalizedEquivalentYaw)
{
  ReleaseStore store;
  auto texts = valid_release_texts();
  replace_all(
    texts.configuration,
    "hall_call: {x: 1.0, y: 1.1, yaw: 0.1}",
    "hall_call: {x: 1.0, y: 1.1, yaw: 6.383185307179586}");
  store.add_release(texts);

  const auto result = load_elevator_release(valid_request(store.config_root()));

  ASSERT_TRUE(result.ok()) << result.message;
  EXPECT_NEAR(result.release->source.poses[0].yaw, 0.1, 1.0e-12);
}

TEST(ElevatorReleaseLoader, RejectsAConfigPathTraversingAnAncestorSymlink)
{
  ReleaseStore store;
  store.add_release(valid_release_texts());
  const auto real_parent = store.config_root().parent_path();
  const auto alias =
    real_parent.parent_path() / (real_parent.filename().string() + "_alias");
  fs::create_directory_symlink(real_parent, alias);

  const auto result =
    load_elevator_release(valid_request(alias / ".elevator_config"));

  std::error_code ignored;
  fs::remove(alias, ignored);
  EXPECT_FALSE(result.ok());
  EXPECT_EQ(result.error, ElevatorReleaseLoadError::kUnsafePath);
}

TEST(ElevatorReleaseLoader, RejectsANestedCurrentTargetAsUnsafe)
{
  ReleaseStore store;
  const auto release_id = store.add_release(valid_release_texts());
  fs::remove(store.config_root() / "current");
  fs::create_directory_symlink(
    fs::path("releases") / release_id / "nested",
    store.config_root() / "current");

  const auto result = load_elevator_release(valid_request(store.config_root()));

  EXPECT_FALSE(result.ok());
  EXPECT_EQ(result.error, ElevatorReleaseLoadError::kUnsafePath);
}

TEST(ElevatorReleaseLoader, RejectsANonSymlinkCurrentSelectorAsUnsafe)
{
  ReleaseStore store;
  store.add_release(valid_release_texts());
  fs::remove(store.config_root() / "current");
  {
    std::ofstream selector(store.config_root() / "current", std::ios::binary);
    selector << "releases/not-a-selector";
  }

  const auto result = load_elevator_release(valid_request(store.config_root()));

  EXPECT_FALSE(result.ok());
  EXPECT_EQ(result.error, ElevatorReleaseLoadError::kUnsafePath);
}

TEST(ElevatorReleaseLoader, ExplicitExpectedReleaseDoesNotRereadCurrentSelector)
{
  ReleaseStore store;
  const auto selected_release = store.add_release(valid_release_texts());
  auto historical_texts = valid_release_texts();
  historical_texts.configuration += "\n";
  const auto expected_release =
    store.add_release(historical_texts, 1U, false);
  ASSERT_NE(selected_release, expected_release);

  auto request = valid_request(store.config_root());
  request.expected_release_id = expected_release;
  const auto result = load_elevator_release(request);

  ASSERT_TRUE(result.ok()) << result.message;
  ASSERT_TRUE(result.release.has_value());
  EXPECT_EQ(result.release->release_id, expected_release);
}

TEST(ElevatorReleaseLoader, ReturnedReleaseStaysFrozenAfterSelectorSwitch)
{
  ReleaseStore store;
  const auto first =
    store.add_release(valid_release_texts(), 1U, true);
  const auto before =
    load_elevator_release(valid_request(store.config_root()));
  ASSERT_TRUE(before.ok()) << before.message;

  const auto second =
    store.add_release(valid_release_texts(), 2U, true, first);
  const auto after =
    load_elevator_release(valid_request(store.config_root()));

  ASSERT_TRUE(after.ok()) << after.message;
  EXPECT_EQ(before.release->release_id, first);
  EXPECT_EQ(before.release->generation, 1U);
  EXPECT_EQ(after.release->release_id, second);
  EXPECT_EQ(after.release->generation, 2U);
}

TEST(ElevatorReleaseLoader, ReportsAMissingExplicitReleaseWithoutFollowingFallbacks)
{
  ReleaseStore store;
  auto request = valid_request(store.config_root());
  request.expected_release_id =
    "elevator-config-000001-0123456789ab";

  const auto result = load_elevator_release(request);

  EXPECT_FALSE(result.ok());
  EXPECT_EQ(result.error, ElevatorReleaseLoadError::kReleaseNotFound);
}

TEST(ElevatorReleaseLoader, LoadsAConsistentSecondGenerationRollbackRelease)
{
  ReleaseStore store;
  const auto parent_release = store.add_release(valid_release_texts());
  const auto rollback_release = store.add_release(
    valid_release_texts(), 2U, true, parent_release, parent_release);

  const auto result = load_elevator_release(valid_request(store.config_root()));

  ASSERT_TRUE(result.ok()) << result.message;
  ASSERT_TRUE(result.release.has_value());
  EXPECT_EQ(result.release->release_id, rollback_release);
  EXPECT_EQ(result.release->generation, 2U);
}

TEST(ElevatorReleaseLoader, RejectsAReleaseWhoseParentIsItself)
{
  ReleaseStore store;
  const auto parent_release = store.add_release(valid_release_texts());
  const auto release_id = store.add_release(
    valid_release_texts(), 2U, true, parent_release);
  store.replace_in_release_file(
    release_id, "manifest.json", parent_release, release_id);
  store.replace_in_release_file(
    release_id, "current.json", parent_release, release_id);

  const auto result = load_elevator_release(valid_request(store.config_root()));

  EXPECT_FALSE(result.ok());
  EXPECT_EQ(result.error, ElevatorReleaseLoadError::kInvalidMetadata);
}

TEST(ElevatorReleaseLoader, RejectsAParentFromAnyGenerationOtherThanThePreviousOne)
{
  ReleaseStore store;
  store.add_release(
    valid_release_texts(), 2U, true,
    "elevator-config-000009-0123456789ab");

  const auto result = load_elevator_release(valid_request(store.config_root()));

  EXPECT_FALSE(result.ok());
  EXPECT_EQ(result.error, ElevatorReleaseLoadError::kInvalidMetadata);
}

TEST(ElevatorReleaseLoader, EmptyPreferredElevatorSelectsTheLowestStableId)
{
  ReleaseStore store;
  store.add_release(two_elevator_release_texts());

  const auto result = load_elevator_release(valid_request(store.config_root()));

  ASSERT_TRUE(result.ok()) << result.message;
  ASSERT_TRUE(result.release.has_value());
  EXPECT_EQ(result.release->elevator_id, "elevator_alpha");
  EXPECT_EQ(result.release->source.poses[0].pose_id, "f1_alpha_hall_call");
}

TEST(ElevatorReleaseLoader, ExactPreferredElevatorOverridesStableSelection)
{
  ReleaseStore store;
  store.add_release(two_elevator_release_texts());
  auto request = valid_request(store.config_root());
  request.preferred_elevator_id = "elevator_west";

  const auto result = load_elevator_release(request);

  ASSERT_TRUE(result.ok()) << result.message;
  ASSERT_TRUE(result.release.has_value());
  EXPECT_EQ(result.release->elevator_id, "elevator_west");
  EXPECT_EQ(result.release->source.poses[0].pose_id, "f1_west_hall_call");
}

TEST(ElevatorReleaseLoader, RejectsAnExactRouteWithTheWrongMapId)
{
  ReleaseStore store;
  store.add_release(valid_release_texts());
  auto request = valid_request(store.config_root());
  request.target_map_id = "map_f2_other";

  const auto result = load_elevator_release(request);

  EXPECT_FALSE(result.ok());
  EXPECT_EQ(result.error, ElevatorReleaseLoadError::kNoRoute);
}

TEST(ElevatorReleaseLoader, RejectsAPreferredElevatorOutsideTheCandidateSet)
{
  ReleaseStore store;
  store.add_release(two_elevator_release_texts());
  auto request = valid_request(store.config_root());
  request.preferred_elevator_id = "elevator_missing";

  const auto result = load_elevator_release(request);

  EXPECT_FALSE(result.ok());
  EXPECT_EQ(result.error, ElevatorReleaseLoadError::kNoRoute);
}

TEST(ElevatorReleaseLoader, RejectsASymlinkedRequiredReleaseFile)
{
  ReleaseStore store;
  const auto release_id = store.add_release(valid_release_texts());
  const auto release_root = store.release_root(release_id);
  const auto outside = store.config_root().parent_path() / "outside.yaml";
  {
    std::ofstream stream(outside, std::ios::binary);
    stream << valid_release_texts().configuration;
  }
  fs::remove(release_root / "configuration.yaml");
  fs::create_symlink(outside, release_root / "configuration.yaml");

  const auto result = load_elevator_release(valid_request(store.config_root()));

  EXPECT_FALSE(result.ok());
  EXPECT_EQ(result.error, ElevatorReleaseLoadError::kUnsafePath);
}

TEST(ElevatorReleaseLoader, RejectsARequiredFileHardLinkedOutsideTheRelease)
{
  ReleaseStore store;
  const auto release_id = store.add_release(valid_release_texts());
  const auto release_file =
    store.release_root(release_id) / "configuration.yaml";
  const auto outside =
    store.config_root().parent_path() / "outside-configuration.yaml";
  fs::rename(release_file, outside);
  fs::create_hard_link(outside, release_file);

  const auto result = load_elevator_release(valid_request(store.config_root()));

  EXPECT_FALSE(result.ok());
  EXPECT_EQ(result.error, ElevatorReleaseLoadError::kUnsafePath);
}

TEST(ElevatorReleaseLoader, RejectsANonRegularRequiredReleaseEntry)
{
  ReleaseStore store;
  const auto release_id = store.add_release(valid_release_texts());
  const auto current_metadata = store.release_root(release_id) / "current.json";
  fs::remove(current_metadata);
  fs::create_directory(current_metadata);

  const auto result = load_elevator_release(valid_request(store.config_root()));

  EXPECT_FALSE(result.ok());
  EXPECT_EQ(result.error, ElevatorReleaseLoadError::kUnsafePath);
}

TEST(ElevatorReleaseLoader, RejectsAReleaseFileLargerThanTwoMiB)
{
  ReleaseStore store;
  const auto release_id = store.add_release(valid_release_texts());
  store.replace_release_file(
    release_id, "configuration.yaml",
    std::string(2U * 1024U * 1024U + 1U, 'x'));

  const auto result = load_elevator_release(valid_request(store.config_root()));

  EXPECT_FALSE(result.ok());
  EXPECT_EQ(result.error, ElevatorReleaseLoadError::kUnsafePath);
}

TEST(ElevatorReleaseLoader, RejectsAReleaseMarkedRuntimeApplied)
{
  ReleaseStore store;
  const auto release_id = store.add_release(valid_release_texts());
  store.replace_in_release_file(
    release_id, "manifest.json",
    "\"runtime_applied\":false", "\"runtime_applied\":true");

  const auto result = load_elevator_release(valid_request(store.config_root()));

  EXPECT_FALSE(result.ok());
  EXPECT_EQ(result.error, ElevatorReleaseLoadError::kInvalidMetadata);
}

TEST(ElevatorReleaseLoader, RejectsAnUnpublishedRelease)
{
  ReleaseStore store;
  const auto release_id = store.add_release(valid_release_texts());
  store.replace_in_release_file(
    release_id, "manifest.json",
    "\"asset_published\":true", "\"asset_published\":false");

  const auto result = load_elevator_release(valid_request(store.config_root()));

  EXPECT_FALSE(result.ok());
  EXPECT_EQ(result.error, ElevatorReleaseLoadError::kInvalidMetadata);
}

TEST(ElevatorReleaseLoader, RejectsYamlOnlyManifestSyntax)
{
  ReleaseStore store;
  const auto release_id = store.add_release(valid_release_texts());
  store.replace_in_release_file(
    release_id, "manifest.json",
    "\"schema_version\":1", "schema_version: 1");

  const auto result = load_elevator_release(valid_request(store.config_root()));

  EXPECT_FALSE(result.ok());
  EXPECT_EQ(result.error, ElevatorReleaseLoadError::kInvalidMetadata);
}

TEST(ElevatorReleaseLoader, RejectsADuplicateManifestKey)
{
  ReleaseStore store;
  const auto release_id = store.add_release(valid_release_texts());
  store.replace_in_release_file(
    release_id, "manifest.json",
    "\"runtime_applied\":false",
    "\"runtime_applied\":false,\"runtime_applied\":false");

  const auto result = load_elevator_release(valid_request(store.config_root()));

  EXPECT_FALSE(result.ok());
  EXPECT_EQ(result.error, ElevatorReleaseLoadError::kInvalidMetadata);
}

TEST(ElevatorReleaseLoader, RejectsAnActorBeyondThePublisherLimit)
{
  ReleaseStore store;
  store.add_release(
    valid_release_texts(), 1U, true, "", "", std::string(257U, 'a'));

  const auto result = load_elevator_release(valid_request(store.config_root()));

  EXPECT_FALSE(result.ok());
  EXPECT_EQ(result.error, ElevatorReleaseLoadError::kInvalidMetadata);
}

TEST(ElevatorReleaseLoader, RejectsManifestAndCurrentActorDisagreement)
{
  ReleaseStore store;
  const auto release_id = store.add_release(valid_release_texts());
  store.replace_in_release_file(
    release_id, "current.json",
    "\"actor_id\":\"commissioner\"", "\"actor_id\":\"other_actor\"");

  const auto result = load_elevator_release(valid_request(store.config_root()));

  EXPECT_FALSE(result.ok());
  EXPECT_EQ(result.error, ElevatorReleaseLoadError::kInvalidMetadata);
}

TEST(ElevatorReleaseLoader, RejectsAZeroGenerationRollbackIdentity)
{
  ReleaseStore store;
  const auto parent =
    store.add_release(valid_release_texts(), 1U, false);
  store.add_release(
    valid_release_texts(), 2U, true, parent,
    "elevator-config-000000-000000000000");

  const auto result = load_elevator_release(valid_request(store.config_root()));

  EXPECT_FALSE(result.ok());
  EXPECT_EQ(result.error, ElevatorReleaseLoadError::kInvalidMetadata);
}

TEST(ElevatorReleaseLoader, RejectsManifestAndCurrentGenerationDisagreement)
{
  ReleaseStore store;
  const auto release_id = store.add_release(valid_release_texts());
  store.replace_in_release_file(
    release_id, "current.json",
    "\"generation\":1", "\"generation\":2");

  const auto result = load_elevator_release(valid_request(store.config_root()));

  EXPECT_FALSE(result.ok());
  EXPECT_EQ(result.error, ElevatorReleaseLoadError::kInvalidMetadata);
}

TEST(ElevatorReleaseLoader, RejectsConflictingDraftAndRollbackLineage)
{
  ReleaseStore store;
  const auto parent_release = store.add_release(valid_release_texts());
  const auto release_id = store.add_release(
    valid_release_texts(), 2U, true, parent_release, parent_release);
  store.replace_in_release_file(
    release_id, "manifest.json",
    "\"source_draft_revision\":null",
    "\"source_draft_revision\":\"draft-v1-0123456789abcdef\"");

  const auto result = load_elevator_release(valid_request(store.config_root()));

  EXPECT_FALSE(result.ok());
  EXPECT_EQ(result.error, ElevatorReleaseLoadError::kInvalidMetadata);
}

TEST(ElevatorReleaseLoader, RejectsAnySchemaOtherThanOne)
{
  ReleaseStore store;
  const auto release_id = store.add_release(valid_release_texts());
  store.replace_in_release_file(
    release_id, "manifest.json",
    "\"schema_version\":1", "\"schema_version\":2");

  const auto result = load_elevator_release(valid_request(store.config_root()));

  EXPECT_FALSE(result.ok());
  EXPECT_EQ(result.error, ElevatorReleaseLoadError::kInvalidMetadata);
}

TEST(ElevatorReleaseLoader, RejectsContentChangedAfterFNVPublication)
{
  ReleaseStore store;
  const auto release_id = store.add_release(valid_release_texts());
  auto changed = valid_release_texts().configuration;
  changed += "\n";
  store.replace_release_file(release_id, "configuration.yaml", changed);

  const auto result = load_elevator_release(valid_request(store.config_root()));

  EXPECT_FALSE(result.ok());
  EXPECT_EQ(result.error, ElevatorReleaseLoadError::kInvalidContent);
}

TEST(ElevatorReleaseLoader, RejectsAnyValidationRecordOtherThanFixedValid)
{
  ReleaseStore store;
  const auto release_id = store.add_release(valid_release_texts());
  store.replace_release_file(
    release_id, "validation.json",
    "{\"valid_for_publish\":false,\"issues\":[]}\n");

  const auto result = load_elevator_release(valid_request(store.config_root()));

  EXPECT_FALSE(result.ok());
  EXPECT_EQ(result.error, ElevatorReleaseLoadError::kInvalidMetadata);
}

TEST(ElevatorReleaseLoader, RejectsNonSha256ManifestMapBindings)
{
  ReleaseStore store;
  const auto release_id = store.add_release(valid_release_texts());
  store.replace_in_release_file(
    release_id, "manifest.json",
    "\"asset_digest_algorithm\":\"sha256\"",
    "\"asset_digest_algorithm\":\"fnv1a64\"");

  const auto result = load_elevator_release(valid_request(store.config_root()));

  EXPECT_FALSE(result.ok());
  EXPECT_EQ(result.error, ElevatorReleaseLoadError::kInvalidMetadata);
}

TEST(ElevatorReleaseLoader, RejectsMissingOrWrongManifestDigestContract)
{
  const std::vector<std::string> replacements{
    "",
    "\"asset_digest_contract\":\"other-contract\",",
  };
  for (const auto & replacement : replacements) {
    SCOPED_TRACE(replacement);
    ReleaseStore store;
    const auto release_id = store.add_release(valid_release_texts());
    store.replace_in_release_file(
      release_id, "manifest.json",
      "\"asset_digest_contract\":\"njrh-map-asset-bundle-v1\",",
      replacement);

    const auto result =
      load_elevator_release(valid_request(store.config_root()));

    EXPECT_FALSE(result.ok());
    EXPECT_EQ(
      result.error,
      ElevatorReleaseLoadError::kInvalidMetadata) << result.message;
  }
}

TEST(ElevatorReleaseLoader, RejectsNonCanonicalConfigurationMapDigest)
{
  auto texts = valid_release_texts();
  replace_all(
    texts.configuration, kSourceDigest,
    "sha256:not-a-canonical-digest");
  ReleaseStore store;
  store.add_release(texts);

  const auto result = load_elevator_release(valid_request(store.config_root()));

  EXPECT_FALSE(result.ok());
  EXPECT_EQ(result.error, ElevatorReleaseLoadError::kInvalidConfiguration);
}

TEST(ElevatorReleaseLoader, RejectsMissingOrNonCanonicalConfigurationMapEpoch)
{
  const std::vector<std::string> replacements{
    "",
    "        map_asset_epoch: 0\n",
    "        map_asset_epoch: 101.0\n",
    "        map_asset_epoch: \"101\"\n",
    "        map_asset_epoch: 18446744073709551616\n",
  };
  for (const auto & replacement : replacements) {
    SCOPED_TRACE(replacement);
    auto texts = valid_release_texts();
    replace_all(
      texts.configuration,
      "        map_asset_epoch: 101\n",
      replacement);
    ReleaseStore store;
    store.add_release(texts);

    const auto result =
      load_elevator_release(valid_request(store.config_root()));

    EXPECT_FALSE(result.ok());
    EXPECT_EQ(
      result.error,
      ElevatorReleaseLoadError::kInvalidConfiguration) << result.message;
  }
}

TEST(ElevatorReleaseLoader, RejectsMissingOrNonCanonicalManifestMapEpoch)
{
  const std::vector<std::string> replacements{
    "",
    "\"asset_epoch\":0,",
    "\"asset_epoch\":101.0,",
    "\"asset_epoch\":\"101\",",
    "\"asset_epoch\":18446744073709551616,",
  };
  for (const auto & replacement : replacements) {
    SCOPED_TRACE(replacement);
    ReleaseStore store;
    const auto release_id = store.add_release(valid_release_texts());
    store.replace_in_release_file(
      release_id, "manifest.json",
      "\"asset_epoch\":101,", replacement);

    const auto result =
      load_elevator_release(valid_request(store.config_root()));

    EXPECT_FALSE(result.ok());
    EXPECT_EQ(
      result.error,
      ElevatorReleaseLoadError::kInvalidMetadata) << result.message;
  }
}

TEST(ElevatorReleaseLoader, RejectsConflictingConfigurationMapEpochs)
{
  auto texts = two_elevator_release_texts();
  const std::string marker = "        map_asset_epoch: 101\n";
  const auto first = texts.configuration.find(marker);
  ASSERT_NE(first, std::string::npos);
  const auto second = texts.configuration.find(marker, first + marker.size());
  ASSERT_NE(second, std::string::npos);
  texts.configuration.replace(
    second, marker.size(), "        map_asset_epoch: 102\n");
  ReleaseStore store;
  store.add_release(texts);

  const auto result =
    load_elevator_release(valid_request(store.config_root()));

  EXPECT_FALSE(result.ok());
  EXPECT_EQ(
    result.error,
    ElevatorReleaseLoadError::kInvalidConfiguration) << result.message;
}

TEST(ElevatorReleaseLoader, RejectsManifestAndConfigurationEpochMismatch)
{
  ReleaseStore store;
  const auto release_id = store.add_release(valid_release_texts());
  store.replace_in_release_file(
    release_id, "manifest.json",
    "\"asset_epoch\":101,", "\"asset_epoch\":102,");

  const auto result =
    load_elevator_release(valid_request(store.config_root()));

  EXPECT_FALSE(result.ok());
  EXPECT_EQ(result.error, ElevatorReleaseLoadError::kInvalidContent);
}

TEST(ElevatorReleaseLoader, RejectsManifestAndConfigurationBindingMismatch)
{
  ReleaseStore store;
  const auto release_id = store.add_release(valid_release_texts());
  auto changed_digest = std::string(kSourceDigest);
  changed_digest.back() = '3';
  store.replace_in_release_file(
    release_id, "manifest.json", kSourceDigest, changed_digest);

  const auto result = load_elevator_release(valid_request(store.config_root()));

  EXPECT_FALSE(result.ok());
  EXPECT_EQ(result.error, ElevatorReleaseLoadError::kInvalidContent);
}

TEST(ElevatorReleaseLoader, RejectsTopologyAndInternalPoseIdMismatch)
{
  auto texts = valid_release_texts();
  replace_all(
    texts.internal_poses,
    "f1_west_hall_call",
    "f1_west_hall_call_changed");
  ReleaseStore store;
  store.add_release(texts);

  const auto result = load_elevator_release(valid_request(store.config_root()));

  EXPECT_FALSE(result.ok());
  EXPECT_EQ(result.error, ElevatorReleaseLoadError::kInvalidInternalPoses);
}

TEST(ElevatorReleaseLoader, RejectsConfigurationAndInternalCoordinatesMismatch)
{
  auto texts = valid_release_texts();
  const auto first_x = texts.internal_poses.find("    x: 1\n");
  ASSERT_NE(first_x, std::string::npos);
  texts.internal_poses.replace(first_x, std::string("    x: 1\n").size(), "    x: 9\n");
  ReleaseStore store;
  store.add_release(texts);

  const auto result = load_elevator_release(valid_request(store.config_root()));

  EXPECT_FALSE(result.ok());
  EXPECT_EQ(result.error, ElevatorReleaseLoadError::kInvalidInternalPoses);
}

TEST(ElevatorReleaseLoader, RejectsDuplicateInternalPoseIds)
{
  auto texts = valid_release_texts();
  replace_all(
    texts.internal_poses,
    "f1_west_hall_wait",
    "f1_west_hall_call");
  ReleaseStore store;
  store.add_release(texts);

  const auto result = load_elevator_release(valid_request(store.config_root()));

  EXPECT_FALSE(result.ok());
  EXPECT_EQ(result.error, ElevatorReleaseLoadError::kInvalidInternalPoses);
}

TEST(ElevatorReleaseLoader, RejectsNonFiniteInternalPoseCoordinates)
{
  auto texts = valid_release_texts();
  const auto first_yaw = texts.internal_poses.find("    yaw: 0.1\n");
  ASSERT_NE(first_yaw, std::string::npos);
  texts.internal_poses.replace(
    first_yaw, std::string("    yaw: 0.1\n").size(), "    yaw: .nan\n");
  ReleaseStore store;
  store.add_release(texts);

  const auto result = load_elevator_release(valid_request(store.config_root()));

  EXPECT_FALSE(result.ok());
  EXPECT_EQ(result.error, ElevatorReleaseLoadError::kInvalidInternalPoses);
}

}  // namespace
}  // namespace robot_elevator_manager
