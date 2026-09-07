#include <chrono>
#include <filesystem>
#include <fstream>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "robot_api_server/features/maps/catalog_activation/map_asset_identity_binding.hpp"
#include "robot_api_server/features/maps/catalog_activation/map_asset_io.hpp"
#include "robot_api_server/features/maps/catalog_activation/map_manifest_io.hpp"

namespace robot_api_server
{
namespace
{
namespace fs = std::filesystem;

std::string read_file(const fs::path & path)
{
  std::ifstream input(path, std::ios::binary);
  return {
    std::istreambuf_iterator<char>(input),
    std::istreambuf_iterator<char>()};
}

class TemporaryMapBundle
{
public:
  TemporaryMapBundle()
  : root_(
      fs::temp_directory_path() /
      ("robot_api_asset_identity_" + std::to_string(
        std::chrono::steady_clock::now().time_since_epoch().count())))
  {
    fs::create_directories(root_);
  }

  ~TemporaryMapBundle()
  {
    std::error_code error;
    fs::remove_all(root_, error);
  }

  const fs::path & root() const
  {
    return root_;
  }

  MapManifest make_map(
    const std::string & map_id,
    const std::string & safe_name = "test_map") const
  {
    MapManifest manifest;
    manifest.map_id = map_id;
    manifest.display_name = "Test map";
    manifest.safe_map_name = safe_name;
    manifest.building_id = "B10";
    manifest.floor_id = "F2";
    manifest.created_at = "2026-07-24T00:00:00Z";
    manifest.root = root_ / "B10" / "F2" / "maps" / map_id;
    fill_manifest_paths(manifest);

    const std::vector<std::pair<fs::path, std::string>> files{
      {
        manifest.nav_map_yaml,
        "image: " + safe_name + ".pgm\n"
        "resolution: 0.05\n"
        "origin: [1.0, 2.0, 0.0]\n"},
      {manifest.nav_map_pgm, "P5\n1 1\n255\nx"},
      {manifest.localizer_map_png, "png"},
      {manifest.localizer_params_yaml, "image: test_map.png\n"},
      {manifest.keepout_mask_yaml, "image: keepout_mask.pgm\n"},
      {manifest.keepout_mask_pgm, "keepout"},
      {manifest.speed_mask_yaml, "image: speed_mask.pgm\n"},
      {manifest.speed_mask_pgm, "speed"},
      {manifest.binary_mask_yaml, "image: binary_mask.pgm\n"},
      {manifest.binary_mask_pgm, "binary"},
      {manifest.asset_report_json, "{}\n"},
    };
    for (const auto & [path, content] : files) {
      fs::create_directories(path.parent_path());
      std::ofstream output(path, std::ios::binary);
      output << content;
    }
    {
      std::ofstream poses(manifest.poses_yaml, std::ios::binary);
      poses << "poses: []\n";
    }
    return manifest;
  }

private:
  fs::path root_;
};

TEST(MapAssetIdentityBinding, PersistsAndReusesExactIdentity)
{
  TemporaryMapBundle temporary;
  auto manifest = temporary.make_map("map_1");

  stamp_map_asset_identity(manifest, temporary.root());

  EXPECT_EQ(manifest.schema, "njrh.map_manifest.v2");
  EXPECT_EQ(manifest.asset_epoch, 1U);
  EXPECT_EQ(manifest.asset_digest_algorithm, "sha256");
  EXPECT_EQ(manifest.asset_digest_contract, "njrh-map-asset-bundle-v1");
  ASSERT_FALSE(manifest.asset_digest.empty());
  const auto first_digest = manifest.asset_digest;

  auto persisted = read_map_manifest(manifest.manifest_json);
  ASSERT_TRUE(persisted.has_value());
  EXPECT_EQ(persisted->asset_epoch, 1U);
  EXPECT_EQ(persisted->asset_digest, first_digest);

  stamp_map_asset_identity(manifest, temporary.root());
  EXPECT_EQ(manifest.asset_epoch, 1U);
  EXPECT_EQ(manifest.asset_digest, first_digest);
}

TEST(MapAssetIdentityBinding, ReadOnlyVerificationNeverRepairsOrAllocatesIdentity)
{
  TemporaryMapBundle temporary;
  auto manifest = temporary.make_map("map_read_only");
  stamp_map_asset_identity(manifest, temporary.root());

  const auto manifest_before = read_file(manifest.manifest_json);
  const auto state_path =
    temporary.root() / ".map_asset_registry" / "epoch_state.json";
  const auto bindings_path =
    temporary.root() / ".map_asset_registry" / "bindings.json";
  const auto state_before = read_file(state_path);
  const auto bindings_before = read_file(bindings_path);
  const auto manifest_time_before = fs::last_write_time(manifest.manifest_json);
  const auto state_time_before = fs::last_write_time(state_path);
  const auto bindings_time_before = fs::last_write_time(bindings_path);

  EXPECT_NO_THROW(verify_map_asset_identity(manifest, temporary.root()));
  EXPECT_EQ(read_file(manifest.manifest_json), manifest_before);
  EXPECT_EQ(read_file(state_path), state_before);
  EXPECT_EQ(read_file(bindings_path), bindings_before);
  EXPECT_EQ(fs::last_write_time(manifest.manifest_json), manifest_time_before);
  EXPECT_EQ(fs::last_write_time(state_path), state_time_before);
  EXPECT_EQ(fs::last_write_time(bindings_path), bindings_time_before);

  {
    std::ofstream changed(
      manifest.keepout_mask_pgm,
      std::ios::binary | std::ios::app);
    changed << "-drift";
  }
  EXPECT_THROW(
    verify_map_asset_identity(manifest, temporary.root()),
    std::runtime_error);
  EXPECT_EQ(read_file(manifest.manifest_json), manifest_before);
  EXPECT_EQ(read_file(state_path), state_before);
  EXPECT_EQ(read_file(bindings_path), bindings_before);
}

TEST(MapAssetIdentityBinding, SnapshotPinsPersistedManifestAndNavigationBytes)
{
  TemporaryMapBundle temporary;
  auto manifest = temporary.make_map("map_snapshot");
  stamp_map_asset_identity(manifest, temporary.root());

  const auto snapshot =
    verify_map_asset_identity_snapshot(manifest, temporary.root());
  EXPECT_EQ(snapshot.manifest.asset_epoch, manifest.asset_epoch);
  EXPECT_EQ(snapshot.manifest.asset_digest, manifest.asset_digest);
  EXPECT_FALSE(snapshot.non_keepout_asset_digest.empty());
  EXPECT_NE(snapshot.nav_map_yaml.find("resolution: 0.05"), std::string::npos);
  EXPECT_EQ(snapshot.nav_map_pgm_header.substr(0U, 2U), "P5");
  const auto map_info = read_nav_map_info_exact_content(
    snapshot.nav_map_yaml,
    snapshot.nav_map_pgm_header,
    manifest.nav_map_pgm.filename().string());
  ASSERT_TRUE(map_info.has_value());
  EXPECT_EQ(map_info->width, 1U);
  EXPECT_EQ(map_info->height, 1U);
  EXPECT_DOUBLE_EQ(map_info->resolution, 0.05);

  auto stale = manifest;
  stale.active = !manifest.active;
  EXPECT_THROW(
    verify_map_asset_identity_snapshot(stale, temporary.root()),
    std::runtime_error);
}

TEST(MapAssetIdentityBinding, KeepoutRepairInspectionFreezesEveryOtherRole)
{
  TemporaryMapBundle temporary;
  auto manifest = temporary.make_map("map_keepout_repair");
  stamp_map_asset_identity(manifest, temporary.root());

  std::string expected_non_keepout;
  {
    MapAssetCommitTransaction transaction(temporary.root());
    expected_non_keepout =
      verify_map_asset_identity_snapshot(
      manifest, temporary.root(), transaction).non_keepout_asset_digest;
  }
  {
    std::ofstream changed(
      manifest.keepout_mask_pgm,
      std::ios::binary | std::ios::app);
    changed << "-keepout-change";
  }
  {
    MapAssetCommitTransaction transaction(temporary.root());
    const auto repair = inspect_map_asset_identity_for_keepout_repair(
      manifest, temporary.root(), transaction);
    EXPECT_EQ(repair.non_keepout_asset_digest, expected_non_keepout);
    EXPECT_THROW(
      verify_map_asset_identity_snapshot(
        manifest, temporary.root(), transaction),
      std::runtime_error);
  }
  {
    std::ofstream changed(
      manifest.asset_report_json,
      std::ios::binary | std::ios::app);
    changed << "-non-keepout-change";
  }
  {
    MapAssetCommitTransaction transaction(temporary.root());
    const auto repair = inspect_map_asset_identity_for_keepout_repair(
      manifest, temporary.root(), transaction);
    EXPECT_NE(repair.non_keepout_asset_digest, expected_non_keepout);
  }
}

TEST(MapAssetIdentityBinding, StampPreservesCommittedActiveBitUnlessTransactionOverridesIt)
{
  TemporaryMapBundle temporary;
  auto manifest = temporary.make_map("map_active");
  stamp_map_asset_identity(manifest, temporary.root());

  auto committed = manifest;
  committed.active = true;
  write_map_manifest(committed);

  manifest.active = false;
  stamp_map_asset_identity(manifest, temporary.root());
  EXPECT_TRUE(manifest.active);

  {
    MapAssetCommitTransaction transaction(temporary.root());
    stamp_map_asset_identity(
      manifest, temporary.root(), transaction, false);
  }
  EXPECT_FALSE(manifest.active);
  const auto persisted = read_map_manifest(manifest.manifest_json);
  ASSERT_TRUE(persisted.has_value());
  EXPECT_FALSE(persisted->active);
}

TEST(MapAssetIdentityBinding, ReadOnlyVerifyDoesNotRecreateMissingCommitLock)
{
  TemporaryMapBundle temporary;
  auto manifest = temporary.make_map("map_missing_commit_lock");
  stamp_map_asset_identity(manifest, temporary.root());
  const auto lock_path =
    temporary.root() / ".map_asset_identity_commit.lock";
  ASSERT_TRUE(fs::remove(lock_path));

  EXPECT_THROW(
    verify_map_asset_identity(manifest, temporary.root()),
    std::runtime_error);
  EXPECT_FALSE(fs::exists(lock_path));
}

TEST(MapAssetIdentityBinding, ChangedBundleAndNewMapReceiveLargerEpochs)
{
  TemporaryMapBundle temporary;
  auto first = temporary.make_map("map_1");
  stamp_map_asset_identity(first, temporary.root());

  {
    std::ofstream changed(first.keepout_mask_pgm, std::ios::binary | std::ios::app);
    changed << "-changed";
  }
  stamp_map_asset_identity(first, temporary.root());
  EXPECT_EQ(first.asset_epoch, 2U);

  auto second = temporary.make_map("map_2", "test_map_2");
  {
    std::ofstream unique(second.asset_report_json, std::ios::binary | std::ios::app);
    unique << "{\"map\":\"map_2\"}\n";
  }
  stamp_map_asset_identity(second, temporary.root());
  EXPECT_EQ(second.asset_epoch, 3U);
  EXPECT_NE(second.asset_digest, first.asset_digest);
}

TEST(MapAssetIdentityBinding, RejectsPathEscapeSymlinkAndHardlink)
{
  TemporaryMapBundle temporary;

  auto missing_poses = temporary.make_map("map_missing_poses");
  fs::remove(missing_poses.poses_yaml);
  EXPECT_THROW(
    stamp_map_asset_identity(missing_poses, temporary.root()),
    std::runtime_error);

  auto escaped = temporary.make_map("map_escape");
  escaped.nav_map_pgm = temporary.root().parent_path() / "outside-map.pgm";
  EXPECT_THROW(
    stamp_map_asset_identity(escaped, temporary.root()),
    std::runtime_error);

  auto linked = temporary.make_map("map_linked");
  const auto original = linked.keepout_mask_pgm;
  const auto outside = temporary.root() / "outside.pgm";
  fs::rename(original, outside);
  std::error_code link_error;
  fs::create_symlink(outside, original, link_error);
  if (!link_error) {
    EXPECT_THROW(
      stamp_map_asset_identity(linked, temporary.root()),
      std::runtime_error);
  }

#ifndef _WIN32
  auto hardlinked = temporary.make_map("map_hardlinked");
  const auto hardlink_source = temporary.root() / "hardlink-source.pgm";
  fs::rename(hardlinked.speed_mask_pgm, hardlink_source);
  fs::create_hard_link(hardlink_source, hardlinked.speed_mask_pgm);
  EXPECT_THROW(
    stamp_map_asset_identity(hardlinked, temporary.root()),
    std::runtime_error);

  auto ancestor_linked = temporary.make_map("map_ancestor_linked");
  const auto building = temporary.root() / "B10";
  const auto real_building = temporary.root() / "B10-real";
  fs::rename(building, real_building);
  fs::create_directory_symlink(real_building, building);
  EXPECT_THROW(
    stamp_map_asset_identity(ancestor_linked, temporary.root()),
    std::runtime_error);
#endif
}

TEST(MapAssetIdentityBinding, RejectsOversizedAssetsBeforeReadingThem)
{
  TemporaryMapBundle temporary;
  auto manifest = temporary.make_map("map_oversized");
  fs::resize_file(
    manifest.nav_map_pgm,
    512ULL * 1024ULL * 1024ULL + 1ULL);
  EXPECT_THROW(
    stamp_map_asset_identity(manifest, temporary.root()),
    std::runtime_error);
}

#ifndef _WIN32
TEST(MapAssetIdentityBinding, RejectsAggregateBundleSizeBeforeHashing)
{
  TemporaryMapBundle temporary;
  auto manifest = temporary.make_map("map_aggregate_oversized");
  constexpr std::uintmax_t kSparseFileBytes =
    400ULL * 1024ULL * 1024ULL;
  fs::resize_file(manifest.nav_map_pgm, kSparseFileBytes);
  fs::resize_file(manifest.localizer_map_png, kSparseFileBytes);
  fs::resize_file(manifest.keepout_mask_pgm, kSparseFileBytes);
  EXPECT_THROW(
    stamp_map_asset_identity(manifest, temporary.root()),
    std::runtime_error);
}
#endif

}  // namespace
}  // namespace robot_api_server
