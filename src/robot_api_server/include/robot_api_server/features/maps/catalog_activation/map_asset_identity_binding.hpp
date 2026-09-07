#pragma once

#include <filesystem>
#include <memory>
#include <optional>
#include <string>

#include "robot_api_server/features/maps/catalog_activation/storage_models.hpp"

namespace robot_api_server
{

struct VerifiedMapAssetSnapshot;

// Serializes a group of map payload, manifest, registry and current/ projection
// mutations against every cooperating process. The managed maps root and lock
// file are pinned for the complete lifetime of this object.
class MapAssetCommitTransaction
{
public:
  explicit MapAssetCommitTransaction(const std::filesystem::path & maps_root);
  ~MapAssetCommitTransaction();

  MapAssetCommitTransaction(const MapAssetCommitTransaction &) = delete;
  MapAssetCommitTransaction & operator=(const MapAssetCommitTransaction &) = delete;
  MapAssetCommitTransaction(MapAssetCommitTransaction &&) = delete;
  MapAssetCommitTransaction & operator=(MapAssetCommitTransaction &&) = delete;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;

  friend void stamp_map_asset_identity(
    MapManifest & manifest,
    const std::filesystem::path & maps_root,
    MapAssetCommitTransaction & transaction,
    std::optional<bool> active_override);
  friend VerifiedMapAssetSnapshot verify_map_asset_identity_snapshot(
    const MapManifest & manifest,
    const std::filesystem::path & maps_root,
    MapAssetCommitTransaction & transaction);
  friend VerifiedMapAssetSnapshot inspect_map_asset_identity_for_keepout_repair(
    const MapManifest & manifest,
    const std::filesystem::path & maps_root,
    MapAssetCommitTransaction & transaction);
};

struct VerifiedMapAssetSnapshot
{
  MapManifest manifest;
  std::string nav_map_yaml;
  std::string nav_map_pgm_header;
  std::string non_keepout_asset_digest;
};

// Pure read-only verification of an already committed manifest v2 identity.
// It never allocates an epoch, creates a lock file, or rewrites the bundle.
void verify_map_asset_identity(
  const MapManifest & manifest,
  const std::filesystem::path & maps_root);

// Returns the exact manifest and navigation-map bytes read from the same
// descriptor-pinned snapshot used for identity verification.
VerifiedMapAssetSnapshot verify_map_asset_identity_snapshot(
  const MapManifest & manifest,
  const std::filesystem::path & maps_root);
VerifiedMapAssetSnapshot verify_map_asset_identity_snapshot(
  const MapManifest & manifest,
  const std::filesystem::path & maps_root,
  MapAssetCommitTransaction & transaction);

// Descriptor-pinned inspection for a previously latched keepout transaction.
// Manifest/registry identity and every non-keepout role remain authenticated,
// while the two keepout payload roles may differ from the old aggregate digest.
VerifiedMapAssetSnapshot inspect_map_asset_identity_for_keepout_repair(
  const MapManifest & manifest,
  const std::filesystem::path & maps_root,
  MapAssetCommitTransaction & transaction);

// Recomputes the canonical 11-file bundle digest, binds it to a persistent
// monotonic epoch, and atomically writes the resulting manifest v2 identity.
// Payload assets and poses are never rewritten by this operation.
void stamp_map_asset_identity(
  MapManifest & manifest,
  const std::filesystem::path & maps_root);

// Transaction-aware form used when payload/current projection changes and the
// identity manifest must commit as one serialized operation. If active_override
// is absent, the currently committed active bit is preserved.
void stamp_map_asset_identity(
  MapManifest & manifest,
  const std::filesystem::path & maps_root,
  MapAssetCommitTransaction & transaction,
  std::optional<bool> active_override = std::nullopt);

}  // namespace robot_api_server
