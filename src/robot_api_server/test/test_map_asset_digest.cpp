#include <stdexcept>
#include <string>
#include <vector>

#include "gtest/gtest.h"

#include "robot_api_server/map_asset_digest.hpp"

namespace robot_api_server
{
namespace
{

TEST(MapAssetDigest, MatchesPublishedSha256Vectors)
{
  EXPECT_EQ(
    sha256_hex(""),
    "e3b0c44298fc1c149afbf4c8996fb924"
    "27ae41e4649b934ca495991b7852b855");
  EXPECT_EQ(
    sha256_hex("abc"),
    "ba7816bf8f01cfea414140de5dae2223"
    "b00361a396177a9cb410ff61f20015ad");

  const std::string binary{"a\0b", 3U};
  EXPECT_EQ(
    sha256_hex(binary),
    "59b271ae1bbcb1d31d41929817f4b16f"
    "b439eb4f31520b5ad1d5ce98920a7138");
}

TEST(MapAssetDigest, HandlesSha256PaddingBoundaries)
{
  EXPECT_EQ(
    sha256_hex(std::string(55U, 'a')),
    "9f4390f8d30c2dd92ec9f095b65e2b9a"
    "e9b0a925a5258e241c9f1e910f734318");
  EXPECT_EQ(
    sha256_hex(std::string(56U, 'a')),
    "b35439a4ac6f0948b6d6f9e3c6af0f5"
    "f590ce20f1bde7090ef7970686ec6738a");
  EXPECT_EQ(
    sha256_hex(std::string(64U, 'a')),
    "ffe054fe7ae0cb6dc65c3af9b61d5209"
    "f439851db43d0ba5997337df154668eb");
  EXPECT_EQ(
    sha256_hex(std::string(65U, 'a')),
    "635361c48bb9eab14198e76ea8ab7f1a"
    "41685d6ad62aa9146d301d4f17eb0ae0");
}

TEST(MapAssetDigest, RecognizesOnlyCanonicalLowercaseWireIdentity)
{
  const std::string valid =
    "sha256:0123456789abcdef0123456789abcdef"
    "0123456789abcdef0123456789abcdef";
  EXPECT_TRUE(is_canonical_sha256_digest(valid));
  EXPECT_FALSE(is_canonical_sha256_digest(valid.substr(0U, valid.size() - 1U)));
  EXPECT_FALSE(is_canonical_sha256_digest(
      "sha256:0123456789ABCDEF0123456789abcdef"
      "0123456789abcdef0123456789abcdef"));
  EXPECT_FALSE(is_canonical_sha256_digest(
      "fnv1a64-0123456789abcdef"));
}

TEST(MapAssetDigest, IsStableAcrossInputOrderAndBindsNamesAndBytes)
{
  const std::vector<MapAssetDigestEntry> first{
    {"nav_map_yaml", "image: map.pgm\n"},
    {"nav_map_pgm", std::string{"P5\n1 1\n255\n\0", 12U}},
  };
  const std::vector<MapAssetDigestEntry> reordered{
    first[1],
    first[0],
  };
  EXPECT_EQ(
    canonical_map_asset_digest(first),
    canonical_map_asset_digest(reordered));
  EXPECT_EQ(
    canonical_map_asset_digest(first),
    "sha256:68036bf64e35311d952e35743622cd6a"
    "03187e2c5306eceac66d6e23a3496ac4");
  EXPECT_TRUE(is_canonical_sha256_digest(
      canonical_map_asset_digest(first)));

  auto content_changed = first;
  content_changed[1].content.back() = '\x01';
  EXPECT_NE(
    canonical_map_asset_digest(first),
    canonical_map_asset_digest(content_changed));

  auto name_changed = first;
  name_changed[1].logical_name = "localizer_map_png";
  EXPECT_NE(
    canonical_map_asset_digest(first),
    canonical_map_asset_digest(name_changed));
}

TEST(MapAssetDigest, RejectsAmbiguousEntrySets)
{
  EXPECT_THROW(canonical_map_asset_digest({}), std::invalid_argument);
  EXPECT_THROW(
    canonical_map_asset_digest({
      {"nav_map_yaml", "a"},
      {"nav_map_yaml", "b"},
    }),
    std::invalid_argument);
  EXPECT_THROW(
    canonical_map_asset_digest({{"", "content"}}),
    std::invalid_argument);
  EXPECT_THROW(
    canonical_map_asset_digest({{"../nav_map_yaml", "content"}}),
    std::invalid_argument);
}

}  // namespace
}  // namespace robot_api_server
