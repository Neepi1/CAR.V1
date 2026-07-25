#include "robot_floor_manager/floor_asset_snapshot_loader.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#ifndef _WIN32
#include <unistd.h>
#endif

#include "robot_map_asset_identity/map_asset_identity.hpp"

namespace
{
namespace fs = std::filesystem;
using robot_floor_manager::FloorAssetSnapshotError;
using robot_floor_manager::FloorAssetSnapshotLoader;
using robot_floor_manager::FloorAssetSnapshotRequest;
using robot_map_asset_identity::DigestEntry;

class TemporaryDirectory
{
public:
  TemporaryDirectory()
  {
    const auto seed =
      std::chrono::steady_clock::now().time_since_epoch().count();
    root_ = fs::temp_directory_path() /
      ("floor_asset_snapshot_test_" + std::to_string(seed));
    fs::create_directories(root_);
  }

  ~TemporaryDirectory()
  {
    std::error_code error;
    fs::remove_all(root_, error);
  }

  const fs::path & path() const noexcept
  {
    return root_;
  }

private:
  fs::path root_;
};

void write_file(const fs::path & path, const std::string & content)
{
  fs::create_directories(path.parent_path());
  std::ofstream output(path, std::ios::binary);
  if (!output) {
    throw std::runtime_error("cannot write fixture " + path.string());
  }
  output.write(content.data(), static_cast<std::streamsize>(content.size()));
}

struct MapFixture
{
  TemporaryDirectory temporary;
  fs::path maps_root{temporary.path() / "maps_release"};
  std::string building_id{"building-a"};
  std::string floor_id{"floor-1"};
  std::string map_id{"map-a"};
  std::string safe_map_name{"delivery-map"};
  fs::path root{
    maps_root / building_id / floor_id / "maps" / map_id};
  std::uint64_t epoch{0U};
  std::string digest;

  std::vector<std::pair<std::string, fs::path>> roles() const
  {
    return {
      {"nav_map_yaml", root / "nav" / (safe_map_name + ".yaml")},
      {"nav_map_pgm", root / "nav" / (safe_map_name + ".pgm")},
      {"localizer_map_png", root / "localizer" / (safe_map_name + ".png")},
      {"localizer_params_yaml", root / "localizer" / (safe_map_name + ".yaml")},
      {"keepout_mask_yaml", root / "filters" / "keepout_mask.yaml"},
      {"keepout_mask_pgm", root / "filters" / "keepout_mask.pgm"},
      {"speed_mask_yaml", root / "filters" / "speed_mask.yaml"},
      {"speed_mask_pgm", root / "filters" / "speed_mask.pgm"},
      {"binary_mask_yaml", root / "filters" / "binary_mask.yaml"},
      {"binary_mask_pgm", root / "filters" / "binary_mask.pgm"},
      {"asset_report_json", root / "reports" / "asset_report.json"},
    };
  }

  MapFixture()
  {
    fs::create_directories(maps_root);
    std::vector<DigestEntry> entries;
    const auto role_files = roles();
    for (std::size_t index = 0U; index < role_files.size(); ++index) {
      const auto content =
        role_files[index].first + "-content-" + std::to_string(index);
      write_file(role_files[index].second, content);
      entries.push_back({role_files[index].first, content});
    }
    write_file(root / "poses.yaml", "poses: []\n");
    digest =
      robot_map_asset_identity::canonical_map_asset_digest(entries);
    robot_map_asset_identity::PersistentAssetEpochRegistry registry(maps_root);
    const auto identity = registry.bind(
      {building_id, floor_id, map_id}, digest);
    epoch = identity.asset_epoch;
    write_manifest();
  }

  void rebind_current_role_contents()
  {
    std::vector<DigestEntry> entries;
    for (const auto & [logical_name, path] : roles()) {
      std::ifstream input(path, std::ios::binary);
      if (!input) {
        throw std::runtime_error("cannot read fixture " + path.string());
      }
      std::ostringstream content;
      content << input.rdbuf();
      entries.push_back({logical_name, content.str()});
    }
    digest =
      robot_map_asset_identity::canonical_map_asset_digest(entries);
    robot_map_asset_identity::PersistentAssetEpochRegistry registry(maps_root);
    const auto identity = registry.bind(
      {building_id, floor_id, map_id}, digest);
    epoch = identity.asset_epoch;
    write_manifest();
  }

  FloorAssetSnapshotRequest request() const
  {
    return {
      maps_root,
      building_id,
      floor_id,
      map_id,
      epoch,
      digest,
    };
  }

  void write_manifest(const std::string & extra_root_field = "")
  {
    std::ostringstream json;
    json << "{\n"
         << "  \"schema\": \"njrh.map_manifest.v2\",\n"
         << "  \"asset_epoch\": " << epoch << ",\n"
         << "  \"asset_digest_algorithm\": \"sha256\",\n"
         << "  \"asset_digest_contract\": \"njrh-map-asset-bundle-v1\",\n"
         << "  \"asset_digest\": \"" << digest << "\",\n"
         << "  \"building_id\": \"" << building_id << "\",\n"
         << "  \"floor_id\": \"" << floor_id << "\",\n"
         << "  \"map_id\": \"" << map_id << "\",\n"
         << "  \"safe_map_name\": \"" << safe_map_name << "\"";
    if (!extra_root_field.empty()) {
      json << ",\n" << extra_root_field;
    }
    json << "\n}\n";
    write_file(root / "manifest.json", json.str());
  }
};

std::map<std::string, std::string> snapshot_file_contents(
  const fs::path & root)
{
  std::map<std::string, std::string> contents;
  for (const auto & entry : fs::recursive_directory_iterator(root)) {
    std::error_code error;
    if (entry.is_regular_file(error) && !error) {
      std::ifstream input(entry.path(), std::ios::binary);
      std::ostringstream value;
      value << input.rdbuf();
      contents.emplace(
        entry.path().lexically_relative(root).generic_string(),
        value.str());
    }
  }
  return contents;
}

TEST(FloorAssetSnapshotLoader, LoadsExactAuthoritativeSourceBundleWithoutWriting)
{
  MapFixture fixture;
  const auto before = snapshot_file_contents(fixture.maps_root);

  const auto result = FloorAssetSnapshotLoader{}.load(fixture.request());

  ASSERT_TRUE(result.ok()) << result.message;
  ASSERT_TRUE(result.snapshot.has_value());
  EXPECT_EQ(result.error, FloorAssetSnapshotError::kNone);
  EXPECT_EQ(result.snapshot->asset_epoch, fixture.epoch);
  EXPECT_EQ(result.snapshot->asset_digest, fixture.digest);
  EXPECT_EQ(result.snapshot->paths.root, fs::absolute(fixture.root));
  EXPECT_EQ(
    result.snapshot->paths.nav_map_yaml,
    fs::absolute(fixture.root / "nav" / "delivery-map.yaml"));
  EXPECT_GT(result.snapshot->fingerprints.nav_map_yaml.size, 0U);
  EXPECT_EQ(snapshot_file_contents(fixture.maps_root), before);
}

TEST(FloorAssetSnapshotLoader, RejectsExpectedEpochMismatch)
{
  MapFixture fixture;
  auto request = fixture.request();
  ++request.expected_asset_epoch;

  const auto result = FloorAssetSnapshotLoader{}.load(request);

  EXPECT_EQ(result.error, FloorAssetSnapshotError::kIdentityMismatch);
  EXPECT_FALSE(result.snapshot.has_value());
}

TEST(FloorAssetSnapshotLoader, RejectsExpectedDigestMismatch)
{
  MapFixture fixture;
  auto request = fixture.request();
  request.expected_asset_digest =
    "sha256:aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa";

  const auto result = FloorAssetSnapshotLoader{}.load(request);

  EXPECT_EQ(result.error, FloorAssetSnapshotError::kIdentityMismatch);
}

TEST(FloorAssetSnapshotLoader, RejectsContentDriftAfterIdentityWasBound)
{
  MapFixture fixture;
  write_file(
    fixture.root / "filters" / "keepout_mask.pgm",
    "changed-after-bind");

  const auto result = FloorAssetSnapshotLoader{}.load(fixture.request());

  EXPECT_EQ(result.error, FloorAssetSnapshotError::kDigestMismatch);
}

TEST(FloorAssetSnapshotLoader, StreamsTheExactCanonicalDigestAcrossReadChunks)
{
  MapFixture fixture;
  std::string multi_chunk_content(3U * 64U * 1024U + 17U, '\0');
  for (std::size_t index = 0U; index < multi_chunk_content.size(); ++index) {
    multi_chunk_content[index] =
      static_cast<char>('a' + (index % 23U));
  }
  write_file(
    fixture.root / "nav" / (fixture.safe_map_name + ".pgm"),
    multi_chunk_content);
  fixture.rebind_current_role_contents();

  const auto result = FloorAssetSnapshotLoader{}.load(fixture.request());

  ASSERT_TRUE(result.ok()) << result.message;
  ASSERT_TRUE(result.snapshot.has_value());
  EXPECT_EQ(result.snapshot->asset_digest, fixture.digest);
}

TEST(FloorAssetSnapshotLoader, RejectsARequiredRoleAboveThePerFileLimit)
{
  MapFixture fixture;
  constexpr std::uintmax_t kOverRoleLimit =
    512ULL * 1024ULL * 1024ULL + 1ULL;
  fs::resize_file(
    fixture.root / "nav" / (fixture.safe_map_name + ".pgm"),
    kOverRoleLimit);

  const auto result = FloorAssetSnapshotLoader{}.load(fixture.request());

  EXPECT_EQ(result.error, FloorAssetSnapshotError::kUnsafeAsset);
  EXPECT_NE(result.message.find("bounded regular file"), std::string::npos);
}

TEST(FloorAssetSnapshotLoader, RejectsAnAggregateRoleSetAboveTheBundleLimit)
{
  MapFixture fixture;
  constexpr std::uintmax_t kSparseRoleBytes =
    384ULL * 1024ULL * 1024ULL;
  fs::resize_file(
    fixture.root / "nav" / (fixture.safe_map_name + ".yaml"),
    kSparseRoleBytes);
  fs::resize_file(
    fixture.root / "nav" / (fixture.safe_map_name + ".pgm"),
    kSparseRoleBytes);
  fs::resize_file(
    fixture.root / "localizer" / (fixture.safe_map_name + ".png"),
    kSparseRoleBytes);

  const auto result = FloorAssetSnapshotLoader{}.load(fixture.request());

  EXPECT_EQ(result.error, FloorAssetSnapshotError::kUnsafeAsset);
  EXPECT_NE(result.message.find("aggregate size limit"), std::string::npos);
}

TEST(FloorAssetSnapshotLoader, RejectsAmbiguousNavAssetSet)
{
  MapFixture fixture;
  write_file(fixture.root / "nav" / "other.yaml", "image: other.pgm\n");

  const auto result = FloorAssetSnapshotLoader{}.load(fixture.request());

  EXPECT_EQ(result.error, FloorAssetSnapshotError::kInvalidLayout);
}

TEST(FloorAssetSnapshotLoader, RejectsManifestIdentityTamperingDuringRegistryAudit)
{
  MapFixture fixture;
  fixture.write_manifest("  \"asset_epoch\": 999");

  const auto result = FloorAssetSnapshotLoader{}.load(fixture.request());

  EXPECT_EQ(result.error, FloorAssetSnapshotError::kRegistryUnavailable);
  EXPECT_NE(result.message.find("registry lookup failed"), std::string::npos);
}

TEST(FloorAssetSnapshotLoader, RejectsNonCanonicalManifestEpochDuringRegistryAudit)
{
  MapFixture fixture;
  auto manifest_path = fixture.root / "manifest.json";
  std::ifstream input(manifest_path, std::ios::binary);
  std::ostringstream text;
  text << input.rdbuf();
  auto content = text.str();
  const auto old_value =
    "\"asset_epoch\": " + std::to_string(fixture.epoch);
  const auto offset = content.find(old_value);
  ASSERT_NE(offset, std::string::npos);
  content.replace(
    offset, old_value.size(),
    "\"asset_epoch\": " + std::to_string(fixture.epoch) + ".0");
  write_file(manifest_path, content);

  const auto result = FloorAssetSnapshotLoader{}.load(fixture.request());

  EXPECT_EQ(result.error, FloorAssetSnapshotError::kRegistryUnavailable);
  EXPECT_NE(result.message.find("registry lookup failed"), std::string::npos);
}

TEST(FloorAssetSnapshotLoader, RejectsUnsafeIdentityComponent)
{
  MapFixture fixture;
  auto request = fixture.request();
  request.floor_id = "../floor-1";

  const auto result = FloorAssetSnapshotLoader{}.load(request);

  EXPECT_EQ(result.error, FloorAssetSnapshotError::kInvalidRequest);
}

TEST(FloorAssetSnapshotLoader, RejectsDotIdentityComponent)
{
  MapFixture fixture;
  auto request = fixture.request();
  request.map_id = ".";

  const auto result = FloorAssetSnapshotLoader{}.load(request);

  EXPECT_EQ(result.error, FloorAssetSnapshotError::kInvalidRequest);
}

#ifndef _WIN32
TEST(FloorAssetSnapshotLoader, RejectsSymlinkedRequiredAsset)
{
  MapFixture fixture;
  const auto target = fixture.temporary.path() / "outside.pgm";
  write_file(target, "outside");
  const auto path = fixture.root / "filters" / "binary_mask.pgm";
  fs::remove(path);
  fs::create_symlink(target, path);

  const auto result = FloorAssetSnapshotLoader{}.load(fixture.request());

  EXPECT_EQ(result.error, FloorAssetSnapshotError::kUnsafeAsset);
}

TEST(FloorAssetSnapshotLoader, RejectsHardLinkedRequiredAsset)
{
  MapFixture fixture;
  const auto original = fixture.root / "filters" / "speed_mask.pgm";
  const auto second_link = fixture.temporary.path() / "second-link.pgm";
  fs::create_hard_link(original, second_link);

  const auto result = FloorAssetSnapshotLoader{}.load(fixture.request());

  EXPECT_EQ(result.error, FloorAssetSnapshotError::kUnsafeAsset);
}

TEST(FloorAssetSnapshotLoader, RejectsSymlinkedBundleDirectory)
{
  MapFixture fixture;
  const auto original = fixture.root / "localizer";
  const auto moved = fixture.root / "localizer-real";
  fs::rename(original, moved);
  fs::create_directory_symlink(moved, original);

  const auto result = FloorAssetSnapshotLoader{}.load(fixture.request());

  EXPECT_EQ(result.error, FloorAssetSnapshotError::kUnsafePath);
}
#endif

}  // namespace
