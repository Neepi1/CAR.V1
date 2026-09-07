#include <chrono>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <string>
#include <system_error>

#include "gtest/gtest.h"

#include "robot_api_server/features/maps/catalog_activation/map_catalog.hpp"
#include "robot_api_server/features/maps/catalog_activation/map_manifest_io.hpp"

namespace robot_api_server
{
namespace
{

namespace fs = std::filesystem;

constexpr char kDigest[] =
  "sha256:aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa"
  "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa";

class TemporaryDirectory
{
public:
  TemporaryDirectory()
  {
    root_ = fs::temp_directory_path() /
      ("njrh_map_catalog_test_" + std::to_string(
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

MapManifest make_manifest(
  const fs::path & maps_root,
  const std::string & map_id = "map_a")
{
  MapManifest manifest;
  manifest.schema = "njrh.map_manifest.v2";
  manifest.asset_epoch = 42U;
  manifest.asset_digest_algorithm = "sha256";
  manifest.asset_digest_contract = "njrh-map-asset-bundle-v1";
  manifest.asset_digest = kDigest;
  manifest.map_id = map_id;
  manifest.display_name = "Lobby Map";
  manifest.safe_map_name = "lobby_map";
  manifest.building_id = "building_a";
  manifest.floor_id = "floor_1";
  manifest.created_at = "2026-07-24T00:00:00Z";
  manifest.active = true;
  manifest.root =
    maps_root / manifest.building_id / manifest.floor_id / "maps" /
    manifest.map_id;
  fill_manifest_paths(manifest);
  return manifest;
}

std::string read_text(const fs::path & path)
{
  std::ifstream input(path, std::ios::binary);
  EXPECT_TRUE(input.is_open());
  return {
    std::istreambuf_iterator<char>(input),
    std::istreambuf_iterator<char>()};
}

TEST(MapCatalog, GetLikeLookupsAndDefaultReadsArePure)
{
  TemporaryDirectory temporary;
  const auto manifest = make_manifest(temporary.path());
  fs::create_directories(manifest.root);
  write_map_manifest(manifest);

  const auto stable_mtime =
    fs::file_time_type::clock::now() - std::chrono::hours(24);
  fs::last_write_time(manifest.manifest_json, stable_mtime);
  const auto content_before = read_text(manifest.manifest_json);
  const auto mtime_before = fs::last_write_time(manifest.manifest_json);

  std::uint32_t migration_calls = 0U;
  MapCatalog catalog(
    temporary.path(),
    [&](const std::string &, const std::string &) {
      ++migration_calls;
      std::ofstream output(
        manifest.manifest_json,
        std::ios::binary | std::ios::app);
      output << ' ';
      output.close();
      fs::last_write_time(
        manifest.manifest_json,
        stable_mtime + std::chrono::hours(48));
    });

  const auto floor_maps =
    catalog.read_floor_map_manifests("building_a", "floor_1");
  ASSERT_EQ(floor_maps.size(), 1U);
  EXPECT_EQ(floor_maps.front().map_id, "map_a");

  const auto all_maps = catalog.read_all_map_manifests();
  ASSERT_EQ(all_maps.size(), 1U);
  EXPECT_EQ(all_maps.front().map_id, "map_a");

  const auto by_id = catalog.find_map_by_id("map_a");
  ASSERT_TRUE(by_id.has_value());
  EXPECT_EQ(by_id->map_id, "map_a");

  std::string lookup_error;
  const auto by_name = catalog.find_floor_map_by_name(
    "building_a", "floor_1", "Lobby Map", lookup_error);
  ASSERT_TRUE(by_name.has_value());
  EXPECT_TRUE(lookup_error.empty());
  EXPECT_EQ(by_name->map_id, "map_a");

  const auto floor_active =
    catalog.active_floor_map("building_a", "floor_1");
  ASSERT_TRUE(floor_active.has_value());
  EXPECT_EQ(floor_active->map_id, "map_a");

  const auto unique_active = catalog.unique_active_map_manifest();
  ASSERT_TRUE(unique_active.has_value());
  EXPECT_EQ(unique_active->map_id, "map_a");

  EXPECT_EQ(migration_calls, 0U);
  EXPECT_EQ(read_text(manifest.manifest_json), content_before);
  EXPECT_EQ(fs::last_write_time(manifest.manifest_json), mtime_before);
}

TEST(MapCatalog, ExplicitMigrationRemainsOptInForStartupCallers)
{
  TemporaryDirectory temporary;
  const auto manifest = make_manifest(temporary.path());
  fs::create_directories(manifest.root);
  write_map_manifest(manifest);

  std::uint32_t migration_calls = 0U;
  MapCatalog catalog(
    temporary.path(),
    [&](const std::string & building_id, const std::string & floor_id) {
      EXPECT_EQ(building_id, "building_a");
      EXPECT_EQ(floor_id, "floor_1");
      ++migration_calls;
    });

  EXPECT_EQ(
    catalog.read_floor_map_manifests(
      "building_a", "floor_1", true).size(),
    1U);
  EXPECT_EQ(migration_calls, 1U);

  EXPECT_EQ(catalog.read_all_map_manifests(true).size(), 1U);
  EXPECT_EQ(migration_calls, 2U);

  EXPECT_EQ(
    catalog.read_floor_map_manifests(
      "building_a", "floor_1").size(),
    1U);
  EXPECT_EQ(catalog.read_all_map_manifests().size(), 1U);
  EXPECT_EQ(migration_calls, 2U);
}

}  // namespace
}  // namespace robot_api_server
