#include <stdexcept>
#include <string>
#include <vector>

#include "gtest/gtest.h"

#include "robot_map_asset_identity/map_asset_identity.hpp"

namespace robot_map_asset_identity
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

TEST(MapAssetDigest, IsWireCompatibleWithTheExistingMapBundleContract)
{
  const std::vector<DigestEntry> first{
    {"nav_map_yaml", "image: map.pgm\n"},
    {"nav_map_pgm", std::string{"P5\n1 1\n255\n\0", 12U}},
  };
  const std::vector<DigestEntry> reordered{first[1], first[0]};

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
}

TEST(MapAssetDigest, RejectsNonCanonicalDigestsAndAmbiguousEntrySets)
{
  const std::string valid =
    "sha256:0123456789abcdef0123456789abcdef"
    "0123456789abcdef0123456789abcdef";
  EXPECT_TRUE(is_canonical_sha256_digest(valid));
  EXPECT_FALSE(is_canonical_sha256_digest(valid.substr(0U, valid.size() - 1U)));
  EXPECT_FALSE(is_canonical_sha256_digest(
      "sha256:0123456789ABCDEF0123456789abcdef"
      "0123456789abcdef0123456789abcdef"));
  EXPECT_FALSE(is_canonical_sha256_digest("fnv1a64-0123456789abcdef"));

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

TEST(MapAssetDigest, StreamingDigestMatchesBufferedContract)
{
  const std::vector<DigestEntry> entries{
    {"nav_map_pgm", std::string{"P5\n1 1\n255\n\0", 12U}},
    {"nav_map_yaml", "image: map.pgm\n"},
  };

  CanonicalMapAssetDigestStream stream;
  stream.begin_entry("nav_map_pgm", 12U);
  stream.update(std::string_view(entries[0].content.data(), 5U));
  stream.update(
    std::string_view(
      entries[0].content.data() + 5U,
      entries[0].content.size() - 5U));
  stream.end_entry();
  stream.begin_entry("nav_map_yaml", entries[1].content.size());
  stream.update(entries[1].content);
  stream.end_entry();

  EXPECT_EQ(stream.finish(), canonical_map_asset_digest(entries));
}

TEST(MapAssetDigest, StreamingDigestFailsClosedOnProtocolViolations)
{
  CanonicalMapAssetDigestStream missing_bytes;
  missing_bytes.begin_entry("nav_map_pgm", 2U);
  missing_bytes.update("a");
  EXPECT_THROW(missing_bytes.end_entry(), std::length_error);

  CanonicalMapAssetDigestStream too_many_bytes;
  too_many_bytes.begin_entry("nav_map_pgm", 1U);
  EXPECT_THROW(too_many_bytes.update("ab"), std::length_error);

  CanonicalMapAssetDigestStream unordered;
  unordered.begin_entry("nav_map_yaml", 0U);
  unordered.end_entry();
  EXPECT_THROW(
    unordered.begin_entry("nav_map_pgm", 0U),
    std::invalid_argument);

  CanonicalMapAssetDigestStream unfinished;
  unfinished.begin_entry("nav_map_pgm", 0U);
  EXPECT_THROW(unfinished.finish(), std::logic_error);

  CanonicalMapAssetDigestStream empty;
  EXPECT_THROW(empty.finish(), std::invalid_argument);
}

}  // namespace
}  // namespace robot_map_asset_identity
