#include <chrono>
#include <filesystem>
#include <fstream>
#include <string>
#include <system_error>

#include "gtest/gtest.h"

#include "robot_api_server/features/maps/catalog_activation/map_asset_filesystem.hpp"

namespace robot_api_server::features::maps
{
namespace
{

namespace fs = std::filesystem;

class TemporaryDirectory
{
public:
  TemporaryDirectory()
  {
    root_ = fs::temp_directory_path() /
      ("njrh_map_asset_filesystem_test_" + std::to_string(
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

void write_text(const fs::path & path, const std::string & content)
{
  std::ofstream output(path, std::ios::binary | std::ios::trunc);
  ASSERT_TRUE(output.is_open());
  output << content;
  ASSERT_TRUE(output.good());
}

MapManifest make_manifest(const fs::path & root)
{
  MapManifest manifest;
  manifest.schema = "njrh.map_manifest.v2";
  manifest.asset_epoch = 42U;
  manifest.asset_digest_algorithm = "sha256";
  manifest.asset_digest_contract = "njrh-map-asset-bundle-v1";
  manifest.asset_digest = "sha256:asset-digest";
  manifest.map_id = "map_a";
  manifest.display_name = "Map A";
  manifest.safe_map_name = "map_a";
  manifest.building_id = "B1";
  manifest.floor_id = "F1";
  manifest.created_at = "2026-08-31T00:00:00Z";
  manifest.active = false;
  manifest.root = root;
  manifest.manifest_json = root / "manifest.json";
  manifest.nav_map_yaml = root / "nav" / "nav_map.yaml";
  manifest.nav_map_pgm = root / "nav" / "nav_map.pgm";
  manifest.localizer_map_png = root / "localizer" / "localizer_map.png";
  manifest.localizer_params_yaml =
    root / "localizer" / "localizer_params.yaml";
  manifest.keepout_mask_yaml = root / "filters" / "keepout_mask.yaml";
  manifest.keepout_mask_pgm = root / "filters" / "keepout_mask.pgm";
  manifest.speed_mask_yaml = root / "filters" / "speed_mask.yaml";
  manifest.speed_mask_pgm = root / "filters" / "speed_mask.pgm";
  manifest.binary_mask_yaml = root / "filters" / "binary_mask.yaml";
  manifest.binary_mask_pgm = root / "filters" / "binary_mask.pgm";
  manifest.asset_report_json = root / "reports" / "asset_report.json";
  manifest.poses_yaml = root / "poses.yaml";
  return manifest;
}

TEST(MapAssetFilesystem, ExactSourceComparisonIgnoresOnlyActiveSelectionState)
{
  TemporaryDirectory temporary;
  const auto left = make_manifest(temporary.path() / "map_a");
  auto right = left;
  right.active = true;

  EXPECT_TRUE(same_exact_map_asset_source(left, right));

  right.asset_digest = "sha256:different";
  EXPECT_FALSE(same_exact_map_asset_source(left, right));

  right = left;
  right.nav_map_yaml = left.root / "nav" / "other.yaml";
  EXPECT_FALSE(same_exact_map_asset_source(left, right));
}

TEST(MapAssetFilesystem, BundleChecksRejectEscapesAndSymlinks)
{
  TemporaryDirectory temporary;
  const auto managed_root = temporary.path() / "maps";
  const auto bundle_root = managed_root / "B1" / "F1" / "map_a";
  const auto nested = bundle_root / "nav";
  const auto regular_file = nested / "nav_map.yaml";
  const auto outside = temporary.path() / "outside";
  fs::create_directories(nested);
  fs::create_directories(outside);
  write_text(regular_file, "image: nav_map.pgm\n");
  write_text(outside / "outside.yaml", "outside\n");

  EXPECT_TRUE(path_lexically_within(bundle_root, managed_root));
  EXPECT_FALSE(path_lexically_within(outside, managed_root));
  EXPECT_TRUE(safe_bundle_directory(bundle_root, managed_root));
  EXPECT_FALSE(safe_bundle_directory(outside, managed_root));
  EXPECT_TRUE(safe_bundle_regular_file(regular_file, bundle_root));
  EXPECT_FALSE(
    safe_bundle_regular_file(outside / "outside.yaml", bundle_root));

  std::error_code error;
  const auto file_symlink = nested / "symlink.yaml";
  fs::create_symlink(regular_file, file_symlink, error);
  ASSERT_FALSE(error) << error.message();
  EXPECT_FALSE(safe_bundle_regular_file(file_symlink, bundle_root));

  const auto directory_symlink = bundle_root / "linked_nav";
  fs::create_directory_symlink(nested, directory_symlink, error);
  ASSERT_FALSE(error) << error.message();
  EXPECT_FALSE(safe_bundle_directory(directory_symlink, managed_root));
  EXPECT_FALSE(
    safe_bundle_regular_file(directory_symlink / "nav_map.yaml", bundle_root));
}

TEST(MapAssetFilesystem, DurableAtomicReadCompareAndRemoveRoundTrip)
{
  TemporaryDirectory temporary;
  const auto left = temporary.path() / "left.txt";
  const auto right = temporary.path() / "right.txt";

  ASSERT_NO_THROW(durable_write_text_file_atomic(left, "map-content"));
  ASSERT_NO_THROW(durable_write_text_file_atomic(right, "map-content"));
  EXPECT_EQ(read_regular_text_file_checked(left), "map-content");

  std::string error;
  EXPECT_TRUE(
    regular_file_contents_equal_checked(left, right, 1024U, error));
  EXPECT_TRUE(error.empty());
  EXPECT_NO_THROW(durable_sync_regular_file(left));
  EXPECT_NO_THROW(durable_sync_directory(temporary.path()));

  ASSERT_NO_THROW(durable_write_text_file_atomic(right, "map-ContenT"));
  EXPECT_FALSE(
    regular_file_contents_equal_checked(left, right, 1024U, error));
  EXPECT_NE(error.find("content differs"), std::string::npos);

  for (const auto & entry : fs::directory_iterator(temporary.path())) {
    EXPECT_EQ(entry.path().filename().string().find(".tmp."), std::string::npos);
  }

  EXPECT_NO_THROW(durable_remove_file(left));
  EXPECT_FALSE(fs::exists(left));
  EXPECT_NO_THROW(durable_remove_file(left));
}

TEST(MapAssetFilesystem, CheckedReadRejectsSymlinksAndOversizedFiles)
{
  TemporaryDirectory temporary;
  const auto source = temporary.path() / "source.txt";
  const auto symlink = temporary.path() / "source_link.txt";
  write_text(source, "payload");

  std::error_code error;
  fs::create_symlink(source, symlink, error);
  ASSERT_FALSE(error) << error.message();

  EXPECT_THROW(read_regular_text_file_checked(symlink), std::runtime_error);
  EXPECT_THROW(read_regular_text_file_checked(source, 3U), std::runtime_error);
  EXPECT_EQ(read_regular_text_file_checked(source, 7U), "payload");
}

TEST(MapAssetFilesystem, ComparisonRejectsMultiLinkFiles)
{
  TemporaryDirectory temporary;
  const auto source = temporary.path() / "source.txt";
  const auto source_alias = temporary.path() / "source_alias.txt";
  const auto current = temporary.path() / "current.txt";
  write_text(source, "same");
  write_text(current, "same");

  std::error_code error_code;
  fs::create_hard_link(source, source_alias, error_code);
  ASSERT_FALSE(error_code) << error_code.message();

  std::string error;
  EXPECT_FALSE(
    regular_file_contents_equal_checked(source, current, 1024U, error));
  EXPECT_EQ(
    error,
    "map projection comparison requires single-link regular files");
}

TEST(MapAssetFilesystem, DeleteTombstonesAreUniqueSiblingPaths)
{
  TemporaryDirectory temporary;
  const auto map_root = temporary.path() / "map_a";
  const auto first = map_delete_tombstone_path(map_root);
  const auto second = map_delete_tombstone_path(map_root);

  EXPECT_EQ(first.parent_path(), map_root.parent_path());
  EXPECT_EQ(second.parent_path(), map_root.parent_path());
  EXPECT_NE(first, second);
  EXPECT_EQ(first.filename().string().find(".map_a.delete-"), 0U);
  EXPECT_TRUE(path_lexically_within(first, map_root.parent_path()));
}

}  // namespace
}  // namespace robot_api_server::features::maps
