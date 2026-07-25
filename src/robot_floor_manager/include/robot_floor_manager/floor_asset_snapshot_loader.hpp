#pragma once

#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>

namespace robot_floor_manager
{

enum class FloorAssetSnapshotError
{
  kNone = 0,
  kInvalidRequest,
  kRegistryUnavailable,
  kIdentityNotFound,
  kIdentityMismatch,
  kUnsafePath,
  kInvalidManifest,
  kInvalidLayout,
  kUnsafeAsset,
  kAssetChanged,
  kDigestMismatch,
  kIoError,
};

const char * floor_asset_snapshot_error_name(
  FloorAssetSnapshotError error) noexcept;

struct FloorAssetSnapshotRequest
{
  std::filesystem::path maps_root;
  std::string building_id;
  std::string floor_id;
  std::string map_id;
  std::uint64_t expected_asset_epoch{0U};
  std::string expected_asset_digest;
};

struct FloorAssetFileFingerprint
{
  std::uint64_t device{0U};
  std::uint64_t inode{0U};
  std::uint64_t size{0U};
  std::int64_t modified_seconds{0};
  std::int64_t modified_nanoseconds{0};

  bool operator==(const FloorAssetFileFingerprint & other) const noexcept;
};

struct FloorAssetSnapshotPaths
{
  std::filesystem::path root;
  std::filesystem::path manifest_json;
  std::filesystem::path nav_map_yaml;
  std::filesystem::path nav_map_pgm;
  std::filesystem::path localizer_map_png;
  std::filesystem::path localizer_params_yaml;
  std::filesystem::path keepout_mask_yaml;
  std::filesystem::path keepout_mask_pgm;
  std::filesystem::path speed_mask_yaml;
  std::filesystem::path speed_mask_pgm;
  std::filesystem::path binary_mask_yaml;
  std::filesystem::path binary_mask_pgm;
  std::filesystem::path asset_report_json;
  std::filesystem::path poses_yaml;
};

struct FloorAssetSnapshotFingerprints
{
  FloorAssetFileFingerprint manifest_json;
  FloorAssetFileFingerprint nav_map_yaml;
  FloorAssetFileFingerprint nav_map_pgm;
  FloorAssetFileFingerprint localizer_map_png;
  FloorAssetFileFingerprint localizer_params_yaml;
  FloorAssetFileFingerprint keepout_mask_yaml;
  FloorAssetFileFingerprint keepout_mask_pgm;
  FloorAssetFileFingerprint speed_mask_yaml;
  FloorAssetFileFingerprint speed_mask_pgm;
  FloorAssetFileFingerprint binary_mask_yaml;
  FloorAssetFileFingerprint binary_mask_pgm;
  FloorAssetFileFingerprint asset_report_json;
  FloorAssetFileFingerprint poses_yaml;
};

struct FloorAssetSnapshot
{
  std::string building_id;
  std::string floor_id;
  std::string map_id;
  std::uint64_t asset_epoch{0U};
  std::string asset_digest;
  FloorAssetSnapshotPaths paths;
  FloorAssetSnapshotFingerprints fingerprints;
};

struct FloorAssetSnapshotResult
{
  FloorAssetSnapshotError error{FloorAssetSnapshotError::kNone};
  std::string message;
  std::optional<FloorAssetSnapshot> snapshot;

  bool ok() const noexcept;
};

// Pure read-only verifier for a source map release. It never allocates an
// asset epoch, rewrites a manifest, or calls a runtime/localizer adapter.
class FloorAssetSnapshotLoader
{
public:
  FloorAssetSnapshotResult load(
    const FloorAssetSnapshotRequest & request) const noexcept;
};

}  // namespace robot_floor_manager
