#include <chrono>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <limits>
#include <string>
#include <utility>

#ifndef _WIN32
#include <sys/stat.h>
#endif

#include "gtest/gtest.h"

#include "robot_api_server/features/maps/catalog_activation/map_manifest_io.hpp"

namespace robot_api_server
{
namespace
{

namespace fs = std::filesystem;

constexpr char kDigest[] =
  "sha256:aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa"
  "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa";

void write_text(const fs::path & path, const std::string & text)
{
  std::ofstream output(path, std::ios::binary | std::ios::trunc);
  ASSERT_TRUE(output.is_open());
  output << text;
  ASSERT_TRUE(output.good());
}

std::string replace_once(
  std::string text,
  const std::string & expected,
  const std::string & replacement)
{
  const auto position = text.find(expected);
  EXPECT_NE(position, std::string::npos);
  if (position != std::string::npos) {
    text.replace(position, expected.size(), replacement);
  }
  return text;
}

class TemporaryDirectory
{
public:
  TemporaryDirectory()
  {
    root_ = fs::temp_directory_path() /
      ("njrh_map_manifest_test_" + std::to_string(
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

MapManifest make_v2_manifest(const fs::path & root)
{
  MapManifest manifest;
  manifest.schema = "njrh.map_manifest.v2";
  manifest.asset_epoch = 42U;
  manifest.asset_digest_algorithm = "sha256";
  manifest.asset_digest_contract = "njrh-map-asset-bundle-v1";
  manifest.asset_digest = kDigest;
  manifest.map_id = "map_a";
  manifest.display_name = "Map A";
  manifest.safe_map_name = "map_a";
  manifest.building_id = "building_a";
  manifest.floor_id = "floor_1";
  manifest.created_at = "2026-07-24T00:00:00Z";
  manifest.active = true;
  manifest.root = root;
  fill_manifest_paths(manifest);
  return manifest;
}

TEST(MapManifestIo, RoundTripsCanonicalV2Identity)
{
  TemporaryDirectory temporary;
  const auto expected = make_v2_manifest(temporary.path());

  write_map_manifest(expected);
  const auto loaded = read_map_manifest(expected.manifest_json);

  ASSERT_TRUE(loaded.has_value());
  EXPECT_EQ(loaded->schema, "njrh.map_manifest.v2");
  EXPECT_EQ(loaded->asset_epoch, 42U);
  EXPECT_EQ(loaded->asset_digest_algorithm, "sha256");
  EXPECT_EQ(
    loaded->asset_digest_contract,
    "njrh-map-asset-bundle-v1");
  EXPECT_EQ(loaded->asset_digest, kDigest);
  EXPECT_EQ(loaded->map_id, expected.map_id);
  EXPECT_EQ(loaded->building_id, expected.building_id);
  EXPECT_EQ(loaded->floor_id, expected.floor_id);
}

TEST(MapManifestIo, ReadsLegacyManifestWithoutInventingAssetIdentity)
{
  TemporaryDirectory temporary;
  const auto manifest_path = temporary.path() / "manifest.json";
  write_text(
    manifest_path,
    "{\n"
    "  \"map_id\": \"legacy_map\",\n"
    "  \"display_name\": \"Legacy Map\",\n"
    "  \"safe_map_name\": \"legacy_map\",\n"
    "  \"building_id\": \"building_a\",\n"
    "  \"floor_id\": \"floor_1\",\n"
    "  \"created_at\": \"2026-07-23T00:00:00Z\",\n"
    "  \"active\": false,\n"
    "  \"assets\": {}\n"
    "}\n");

  const auto loaded = read_map_manifest(manifest_path);

  ASSERT_TRUE(loaded.has_value());
  EXPECT_TRUE(loaded->schema.empty());
  EXPECT_EQ(loaded->asset_epoch, 0U);
  EXPECT_TRUE(loaded->asset_digest_algorithm.empty());
  EXPECT_TRUE(loaded->asset_digest_contract.empty());
  EXPECT_TRUE(loaded->asset_digest.empty());
  EXPECT_EQ(loaded->map_id, "legacy_map");
}

TEST(MapManifestIo, RejectsNonCanonicalV2EpochEncodings)
{
  TemporaryDirectory temporary;
  const auto manifest = make_v2_manifest(temporary.path());
  const auto canonical = map_manifest_json(manifest);
  const std::string invalid_epochs[] = {
    "0",
    "-1",
    "+1",
    "01",
    "1.0",
    "1e3",
    "\"1\"",
    "18446744073709551616",
  };

  for (const auto & invalid_epoch : invalid_epochs) {
    SCOPED_TRACE(invalid_epoch);
    write_text(
      manifest.manifest_json,
      replace_once(
        canonical,
        "\"asset_epoch\": 42",
        "\"asset_epoch\": " + invalid_epoch));
    EXPECT_FALSE(read_map_manifest(manifest.manifest_json).has_value());
  }
}

TEST(MapManifestIo, RejectsDuplicateV2IdentityKeys)
{
  TemporaryDirectory temporary;
  const auto manifest = make_v2_manifest(temporary.path());
  const auto canonical = map_manifest_json(manifest);
  const std::pair<std::string, std::string> duplicates[] = {
    {
      "\"schema\": \"njrh.map_manifest.v2\"",
      "\"schema\": \"njrh.map_manifest.v2\",\n"
      "  \"schema\": \"njrh.map_manifest.v2\"",
    },
    {
      "\"asset_epoch\": 42",
      "\"asset_epoch\": 42,\n  \"asset_epoch\": 43",
    },
    {
      "\"asset_epoch\": 42",
      "\"asset_epoch\": 42,\n  \"asset_\\u0065poch\": 43",
    },
    {
      "\"asset_digest_algorithm\": \"sha256\"",
      "\"asset_digest_algorithm\": \"sha256\",\n"
      "  \"asset_digest_algorithm\": \"sha256\"",
    },
    {
      "\"asset_digest_contract\": \"njrh-map-asset-bundle-v1\"",
      "\"asset_digest_contract\": \"njrh-map-asset-bundle-v1\",\n"
      "  \"asset_digest_contract\": \"njrh-map-asset-bundle-v1\"",
    },
    {
      std::string{"\"asset_digest\": \""} + kDigest + "\"",
      std::string{"\"asset_digest\": \""} + kDigest + "\",\n"
      "  \"asset_digest\": \"" + kDigest + "\"",
    },
  };

  for (const auto & duplicate : duplicates) {
    SCOPED_TRACE(duplicate.second);
    write_text(
      manifest.manifest_json,
      replace_once(canonical, duplicate.first, duplicate.second));
    EXPECT_FALSE(read_map_manifest(manifest.manifest_json).has_value());
  }
}

TEST(MapManifestIo, AcceptsMaximumUint64EpochWithoutPrecisionLoss)
{
  TemporaryDirectory temporary;
  auto manifest = make_v2_manifest(temporary.path());
  manifest.asset_epoch = std::numeric_limits<std::uint64_t>::max();

  write_map_manifest(manifest);
  const auto loaded = read_map_manifest(manifest.manifest_json);

  ASSERT_TRUE(loaded.has_value());
  EXPECT_EQ(
    loaded->asset_epoch,
    std::numeric_limits<std::uint64_t>::max());
}

TEST(MapManifestIo, RejectsNonCanonicalV2DigestMetadata)
{
  TemporaryDirectory temporary;
  const auto manifest = make_v2_manifest(temporary.path());
  const auto canonical = map_manifest_json(manifest);
  const std::pair<std::string, std::string> replacements[] = {
    {
      "\"asset_digest_algorithm\": \"sha256\"",
      "\"asset_digest_algorithm\": \"SHA256\"",
    },
    {
      "\"asset_digest_contract\": \"njrh-map-asset-bundle-v1\"",
      "\"asset_digest_contract\": \"njrh-map-asset-bundle-v2\"",
    },
    {
      std::string{"\"asset_digest\": \""} + kDigest + "\"",
      "\"asset_digest\": "
      "\"sha256:AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA"
      "AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA\"",
    },
    {
      std::string{"\"asset_digest\": \""} + kDigest + "\"",
      "\"asset_digest\": "
      "\"aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa"
      "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa\"",
    },
    {
      std::string{"\"asset_digest\": \""} + kDigest + "\"",
      "\"asset_digest\": \"sha256:aaaaaaaa\"",
    },
  };

  for (const auto & replacement : replacements) {
    SCOPED_TRACE(replacement.second);
    write_text(
      manifest.manifest_json,
      replace_once(canonical, replacement.first, replacement.second));
    EXPECT_FALSE(read_map_manifest(manifest.manifest_json).has_value());
  }
}

TEST(MapManifestIo, RejectsIncompleteV2Identity)
{
  TemporaryDirectory temporary;
  const auto manifest = make_v2_manifest(temporary.path());
  const auto canonical = map_manifest_json(manifest);
  const std::pair<std::string, std::string> replacements[] = {
    {
      "\"schema\": \"njrh.map_manifest.v2\"",
      "\"schema\": \"njrh.map_manifest.v3\"",
    },
    {
      "\"asset_epoch\": 42",
      "\"unrelated_epoch\": 42",
    },
    {
      "\"asset_digest_algorithm\": \"sha256\"",
      "\"asset_digest_algorithm\": 256",
    },
    {
      "\"asset_digest_contract\": \"njrh-map-asset-bundle-v1\"",
      "\"asset_digest_contract\": null",
    },
    {
      std::string{"\"asset_digest\": \""} + kDigest + "\"",
      "\"unrelated_digest\": true",
    },
  };

  for (const auto & replacement : replacements) {
    SCOPED_TRACE(replacement.second);
    write_text(
      manifest.manifest_json,
      replace_once(canonical, replacement.first, replacement.second));
    EXPECT_FALSE(read_map_manifest(manifest.manifest_json).has_value());
  }
}

TEST(MapManifestIo, RefusesToWriteIncompleteOrNonCanonicalV2Identity)
{
  TemporaryDirectory temporary;
  const auto canonical = make_v2_manifest(temporary.path());

  auto invalid = canonical;
  invalid.schema.clear();
  EXPECT_THROW(write_map_manifest(invalid), std::invalid_argument);

  invalid = canonical;
  invalid.asset_epoch = 0U;
  EXPECT_THROW(write_map_manifest(invalid), std::invalid_argument);

  invalid = canonical;
  invalid.asset_digest_algorithm = "SHA256";
  EXPECT_THROW(write_map_manifest(invalid), std::invalid_argument);

  invalid = canonical;
  invalid.asset_digest_contract.clear();
  EXPECT_THROW(write_map_manifest(invalid), std::invalid_argument);

  invalid = canonical;
  invalid.asset_digest =
    "sha256:AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA"
    "AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA";
  EXPECT_THROW(write_map_manifest(invalid), std::invalid_argument);

  EXPECT_FALSE(fs::exists(canonical.manifest_json));
}

TEST(MapManifestIo, RefusesManifestSymlinksWithoutChangingTheirTarget)
{
  TemporaryDirectory temporary;
  const auto symlink_target = temporary.path() / "target.json";
  write_text(symlink_target, "do not replace\n");

  const auto manifest = make_v2_manifest(temporary.path());
  std::error_code error;
  fs::create_symlink(symlink_target, manifest.manifest_json, error);
  if (error) {
    GTEST_SKIP() << "symlink creation is unavailable: " << error.message();
  }

  EXPECT_THROW(write_map_manifest(manifest), std::runtime_error);

  std::ifstream input(symlink_target, std::ios::binary);
  ASSERT_TRUE(input.is_open());
  const std::string content{
    std::istreambuf_iterator<char>(input),
    std::istreambuf_iterator<char>()};
  EXPECT_EQ(content, "do not replace\n");
  EXPECT_FALSE(read_map_manifest(manifest.manifest_json).has_value());
}

TEST(MapManifestIo, AtomicallyReplacesRegularManifestWithoutTempResidue)
{
  TemporaryDirectory temporary;
  auto manifest = make_v2_manifest(temporary.path());
  write_map_manifest(manifest);

#ifndef _WIN32
  struct stat before {};
  ASSERT_EQ(::stat(manifest.manifest_json.c_str(), &before), 0);
#endif

  manifest.active = false;
  write_map_manifest(manifest);

#ifndef _WIN32
  struct stat after {};
  ASSERT_EQ(::stat(manifest.manifest_json.c_str(), &after), 0);
  EXPECT_NE(before.st_ino, after.st_ino);
#endif

  const auto loaded = read_map_manifest(manifest.manifest_json);
  ASSERT_TRUE(loaded.has_value());
  EXPECT_FALSE(loaded->active);
  for (const auto & entry : fs::directory_iterator(temporary.path())) {
    EXPECT_EQ(
      entry.path().filename().string().find(".tmp."),
      std::string::npos);
  }
}

}  // namespace
}  // namespace robot_api_server
