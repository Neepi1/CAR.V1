#pragma once

#include <cstdint>
#include <filesystem>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace robot_map_asset_identity
{

struct DigestEntry
{
  std::string logical_name;
  std::string content;
};

std::string sha256_hex(std::string_view content);
bool is_canonical_sha256_digest(std::string_view digest) noexcept;
std::string canonical_map_asset_digest(
  const std::vector<DigestEntry> & entries);

// Streaming form of the canonical bundle digest. Entries must be appended in
// strictly increasing logical-name order and every declared byte must be fed
// before the next entry begins. This keeps large map images out of process
// memory while preserving the exact njrh-map-asset-bundle-v1 digest contract.
class CanonicalMapAssetDigestStream
{
public:
  CanonicalMapAssetDigestStream();
  ~CanonicalMapAssetDigestStream();

  CanonicalMapAssetDigestStream(CanonicalMapAssetDigestStream &&) noexcept;
  CanonicalMapAssetDigestStream & operator=(
    CanonicalMapAssetDigestStream &&) noexcept;

  CanonicalMapAssetDigestStream(const CanonicalMapAssetDigestStream &) = delete;
  CanonicalMapAssetDigestStream & operator=(
    const CanonicalMapAssetDigestStream &) = delete;

  void begin_entry(
    std::string_view logical_name,
    std::uint64_t content_size);
  void update(std::string_view content_chunk);
  void end_entry();
  std::string finish();

private:
  struct Implementation;
  std::unique_ptr<Implementation> implementation_;
};

struct AssetIdentityKey
{
  std::string building_id;
  std::string floor_id;
  std::string map_id;

  bool operator==(const AssetIdentityKey & other) const noexcept;
};

struct AssetIdentity
{
  std::uint64_t asset_epoch{0U};
  std::string asset_digest;

  bool operator==(const AssetIdentity & other) const noexcept;
};

class PersistentAssetEpochRegistry
{
public:
  explicit PersistentAssetEpochRegistry(std::filesystem::path maps_root);

  AssetIdentity bind(
    const AssetIdentityKey & key,
    std::string_view canonical_asset_digest);

  // Strictly read-only. Throws when the persistent registry or its
  // coordination lock is absent/corrupt because identity cannot be proved.
  std::optional<AssetIdentity> lookup(
    const AssetIdentityKey & key) const;

private:
  std::filesystem::path maps_root_;
};

}  // namespace robot_map_asset_identity
