#include "robot_api_server/features/maps/catalog_activation/map_asset_identity_binding.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <limits>
#include <mutex>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#ifndef _WIN32
#include <fcntl.h>
#include <sys/file.h>
#include <sys/stat.h>
#include <unistd.h>
#endif

#include "robot_api_server/features/maps/catalog_activation/map_manifest_io.hpp"
#include "robot_map_asset_identity/map_asset_identity.hpp"

namespace robot_api_server
{
namespace
{
namespace fs = std::filesystem;

constexpr char kManifestSchema[] = "njrh.map_manifest.v2";
constexpr char kDigestAlgorithm[] = "sha256";
constexpr char kDigestContract[] = "njrh-map-asset-bundle-v1";
constexpr char kCommitLockFilename[] = ".map_asset_identity_commit.lock";
constexpr std::uintmax_t kMaximumAssetBytes =
  512ULL * 1024ULL * 1024ULL;
constexpr std::uintmax_t kMaximumBundleBytes =
  1024ULL * 1024ULL * 1024ULL;
constexpr std::size_t kMaximumManifestBytes = 1024U * 1024U;
constexpr std::uintmax_t kMaximumPosesBytes = 64U * 1024U * 1024U;

std::mutex g_process_commit_mutex;

struct RolePath
{
  std::string logical_name;
  fs::path path;
};

bool same_normalized_path(const fs::path & left, const fs::path & right)
{
  std::error_code left_error;
  std::error_code right_error;
  const auto absolute_left = fs::absolute(left, left_error).lexically_normal();
  const auto absolute_right = fs::absolute(right, right_error).lexically_normal();
  return !left_error && !right_error && absolute_left == absolute_right;
}

std::vector<RolePath> required_roles(const MapManifest & manifest)
{
  std::vector<RolePath> roles{
    {"nav_map_yaml", manifest.nav_map_yaml},
    {"nav_map_pgm", manifest.nav_map_pgm},
    {"localizer_map_png", manifest.localizer_map_png},
    {"localizer_params_yaml", manifest.localizer_params_yaml},
    {"keepout_mask_yaml", manifest.keepout_mask_yaml},
    {"keepout_mask_pgm", manifest.keepout_mask_pgm},
    {"speed_mask_yaml", manifest.speed_mask_yaml},
    {"speed_mask_pgm", manifest.speed_mask_pgm},
    {"binary_mask_yaml", manifest.binary_mask_yaml},
    {"binary_mask_pgm", manifest.binary_mask_pgm},
    {"asset_report_json", manifest.asset_report_json},
  };
  std::sort(
    roles.begin(), roles.end(),
    [](const RolePath & left, const RolePath & right) {
      return left.logical_name < right.logical_name;
    });
  return roles;
}

void validate_manifest_paths(
  const MapManifest & manifest,
  const fs::path & maps_root)
{
  if (!safe_asset_id(manifest.building_id) ||
    !safe_asset_id(manifest.floor_id) ||
    !safe_asset_id(manifest.map_id) ||
    !safe_asset_id(manifest.safe_map_name))
  {
    throw std::runtime_error("map identity contains an unsafe identifier");
  }
  const auto expected_root =
    maps_root / manifest.building_id / manifest.floor_id / "maps" / manifest.map_id;
  if (!same_normalized_path(manifest.root, expected_root)) {
    throw std::runtime_error("map root does not match its building/floor/map identity");
  }

  MapManifest expected = manifest;
  fill_manifest_paths(expected);
  const auto actual_roles = required_roles(manifest);
  const auto expected_roles = required_roles(expected);
  for (std::size_t index = 0U; index < actual_roles.size(); ++index) {
    if (actual_roles[index].logical_name != expected_roles[index].logical_name ||
      !same_normalized_path(actual_roles[index].path, expected_roles[index].path))
    {
      throw std::runtime_error(
              "map asset path does not match role " +
              actual_roles[index].logical_name);
    }
  }
  if (!same_normalized_path(manifest.manifest_json, expected.manifest_json)) {
    throw std::runtime_error("map manifest path does not match its identity");
  }
  if (!same_normalized_path(manifest.poses_yaml, expected.poses_yaml)) {
    throw std::runtime_error("map poses path does not match its identity");
  }
}

void validate_committed_identity(const MapManifest & manifest)
{
  if (manifest.schema != kManifestSchema ||
    manifest.asset_epoch == 0U ||
    manifest.asset_digest_algorithm != kDigestAlgorithm ||
    manifest.asset_digest_contract != kDigestContract ||
    !robot_map_asset_identity::is_canonical_sha256_digest(
      manifest.asset_digest))
  {
    throw std::runtime_error(
            "map manifest has no complete authoritative v2 asset identity");
  }
}

robot_map_asset_identity::AssetIdentityKey identity_key(
  const MapManifest & manifest)
{
  return {
    manifest.building_id,
    manifest.floor_id,
    manifest.map_id,
  };
}

void require_registry_identity(
  const MapManifest & manifest,
  const fs::path & maps_root)
{
  robot_map_asset_identity::PersistentAssetEpochRegistry registry(maps_root);
  const auto identity = registry.lookup(identity_key(manifest));
  if (!identity ||
    identity->asset_epoch != manifest.asset_epoch ||
    identity->asset_digest != manifest.asset_digest)
  {
    throw std::runtime_error(
            "map manifest identity does not match the authoritative registry");
  }
}

#ifndef _WIN32

class UniqueFd
{
public:
  UniqueFd() noexcept = default;

  explicit UniqueFd(const int descriptor) noexcept
  : descriptor_(descriptor)
  {
  }

  ~UniqueFd()
  {
    if (descriptor_ >= 0) {
      (void)::close(descriptor_);
    }
  }

  UniqueFd(const UniqueFd &) = delete;
  UniqueFd & operator=(const UniqueFd &) = delete;

  UniqueFd(UniqueFd && other) noexcept
  : descriptor_(other.descriptor_)
  {
    other.descriptor_ = -1;
  }

  UniqueFd & operator=(UniqueFd && other) noexcept
  {
    if (this != &other) {
      if (descriptor_ >= 0) {
        (void)::close(descriptor_);
      }
      descriptor_ = other.descriptor_;
      other.descriptor_ = -1;
    }
    return *this;
  }

  int get() const noexcept
  {
    return descriptor_;
  }

private:
  int descriptor_{-1};
};

std::runtime_error io_error(
  const std::string & operation,
  const fs::path & path)
{
  return std::runtime_error(
    operation + " failed for " + path.string() + ": " +
    std::strerror(errno));
}

bool same_snapshot(const struct stat & left, const struct stat & right)
{
  return left.st_dev == right.st_dev &&
         left.st_ino == right.st_ino &&
         left.st_mode == right.st_mode &&
         left.st_nlink == right.st_nlink &&
         left.st_size == right.st_size &&
         left.st_mtim.tv_sec == right.st_mtim.tv_sec &&
         left.st_mtim.tv_nsec == right.st_mtim.tv_nsec &&
         left.st_ctim.tv_sec == right.st_ctim.tv_sec &&
         left.st_ctim.tv_nsec == right.st_ctim.tv_nsec;
}

bool same_directory_identity(
  const struct stat & left,
  const struct stat & right)
{
  return left.st_dev == right.st_dev &&
         left.st_ino == right.st_ino &&
         S_ISDIR(left.st_mode) &&
         S_ISDIR(right.st_mode);
}

struct PinnedDirectory
{
  UniqueFd descriptor;
  struct stat initial {};
  int parent_descriptor{-1};
  std::string name;
  fs::path path;
  bool root{false};
};

PinnedDirectory open_root_directory(const fs::path & path)
{
  UniqueFd descriptor(
    ::open(
      path.string().c_str(),
      O_RDONLY | O_DIRECTORY | O_CLOEXEC | O_NOFOLLOW));
  if (descriptor.get() < 0) {
    throw io_error("open managed maps root", path);
  }
  struct stat status {};
  if (::fstat(descriptor.get(), &status) != 0 || !S_ISDIR(status.st_mode)) {
    throw io_error("stat managed maps root", path);
  }
  return {
    std::move(descriptor),
    status,
    -1,
    {},
    path,
    true,
  };
}

PinnedDirectory open_child_directory(
  const PinnedDirectory & parent,
  const std::string & name,
  const fs::path & path)
{
  UniqueFd descriptor(
    ::openat(
      parent.descriptor.get(), name.c_str(),
      O_RDONLY | O_DIRECTORY | O_CLOEXEC | O_NOFOLLOW));
  if (descriptor.get() < 0) {
    throw io_error("open map bundle directory", path);
  }
  struct stat status {};
  struct stat path_status {};
  if (::fstat(descriptor.get(), &status) != 0 ||
    ::fstatat(
      parent.descriptor.get(), name.c_str(), &path_status,
      AT_SYMLINK_NOFOLLOW) != 0 ||
    !S_ISDIR(status.st_mode) || !S_ISDIR(path_status.st_mode) ||
    !same_directory_identity(status, path_status))
  {
    throw std::runtime_error(
            "map bundle path component is not a stable real directory: " +
            path.string());
  }
  return {
    std::move(descriptor),
    status,
    parent.descriptor.get(),
    name,
    path,
    false,
  };
}

void revalidate_directory(const PinnedDirectory & directory)
{
  struct stat opened {};
  struct stat path_status {};
  const auto path_result = directory.root ?
    ::lstat(directory.path.string().c_str(), &path_status) :
    ::fstatat(
    directory.parent_descriptor, directory.name.c_str(), &path_status,
    AT_SYMLINK_NOFOLLOW);
  if (::fstat(directory.descriptor.get(), &opened) != 0 ||
    path_result != 0 ||
    !S_ISDIR(path_status.st_mode) ||
    !same_directory_identity(directory.initial, opened) ||
    !same_directory_identity(opened, path_status))
  {
    throw std::runtime_error(
            "map bundle directory changed during identity transaction: " +
            directory.path.string());
  }
}

class CommitFileLock
{
public:
  CommitFileLock(
    const PinnedDirectory & maps_root,
    const bool create_if_missing,
    const int operation)
  {
    int flags =
      (operation == LOCK_SH ? O_RDONLY : O_RDWR) |
      O_CLOEXEC | O_NOFOLLOW;
    if (create_if_missing) {
      flags |= O_CREAT;
    }
    descriptor_ = UniqueFd(
      ::openat(
        maps_root.descriptor.get(), kCommitLockFilename, flags, 0600));
    if (descriptor_.get() < 0) {
      throw io_error(
              "open map asset identity commit lock",
              maps_root.path / kCommitLockFilename);
    }
    struct stat status {};
    if (::fstat(descriptor_.get(), &status) != 0 ||
      !S_ISREG(status.st_mode) || status.st_nlink != 1)
    {
      throw std::runtime_error(
              "map asset identity commit lock must be a single-link regular file");
    }
    if (::flock(descriptor_.get(), operation) != 0) {
      throw io_error(
              "lock map asset identity transaction",
              maps_root.path / kCommitLockFilename);
    }
  }

private:
  UniqueFd descriptor_;
};

struct PinnedAsset
{
  std::string logical_name;
  UniqueFd descriptor;
  struct stat initial {};
  int parent_descriptor{-1};
  std::string name;
  fs::path path;
};

PinnedAsset open_asset(
  const PinnedDirectory & parent,
  const RolePath & role,
  const std::uintmax_t maximum_bytes = kMaximumAssetBytes)
{
  const auto name = role.path.filename().string();
  struct stat path_status {};
  if (::fstatat(
      parent.descriptor.get(), name.c_str(), &path_status,
      AT_SYMLINK_NOFOLLOW) != 0)
  {
    throw io_error("inspect required map asset", role.path);
  }
  if (!S_ISREG(path_status.st_mode) || path_status.st_nlink != 1 ||
    path_status.st_size < 0 ||
    static_cast<std::uintmax_t>(path_status.st_size) > maximum_bytes)
  {
    throw std::runtime_error(
            "map asset must be a single-link bounded regular file: " +
            role.path.string());
  }
  UniqueFd descriptor(
    ::openat(
      parent.descriptor.get(), name.c_str(),
      O_RDONLY | O_CLOEXEC | O_NOFOLLOW));
  if (descriptor.get() < 0) {
    throw io_error("open required map asset", role.path);
  }
  struct stat opened {};
  if (::fstat(descriptor.get(), &opened) != 0 ||
    !same_snapshot(path_status, opened))
  {
    throw std::runtime_error(
            "map asset changed while it was opened: " + role.path.string());
  }
  return {
    role.logical_name,
    std::move(descriptor),
    opened,
    parent.descriptor.get(),
    name,
    role.path,
  };
}

void revalidate_asset(const PinnedAsset & asset)
{
  struct stat opened {};
  struct stat path_status {};
  if (::fstat(asset.descriptor.get(), &opened) != 0 ||
    ::fstatat(
      asset.parent_descriptor, asset.name.c_str(), &path_status,
      AT_SYMLINK_NOFOLLOW) != 0 ||
    !S_ISREG(path_status.st_mode) || path_status.st_nlink != 1 ||
    !same_snapshot(asset.initial, opened) ||
    !same_snapshot(opened, path_status))
  {
    throw std::runtime_error(
            "map asset changed during identity transaction: " +
            asset.path.string());
  }
}

struct PinnedBundle
{
  PinnedDirectory maps_root;
  PinnedDirectory building;
  PinnedDirectory floor;
  PinnedDirectory maps;
  PinnedDirectory root;
  PinnedDirectory nav;
  PinnedDirectory localizer;
  PinnedDirectory filters;
  PinnedDirectory reports;
  std::vector<PinnedAsset> assets;
  PinnedAsset poses;
};

const PinnedDirectory & role_parent(
  const PinnedBundle & bundle,
  const fs::path & relative_parent)
{
  if (relative_parent == "nav") {
    return bundle.nav;
  }
  if (relative_parent == "localizer") {
    return bundle.localizer;
  }
  if (relative_parent == "filters") {
    return bundle.filters;
  }
  if (relative_parent == "reports") {
    return bundle.reports;
  }
  throw std::runtime_error("map asset role has no trusted parent directory");
}

PinnedBundle open_bundle(
  const MapManifest & manifest,
  const fs::path & maps_root)
{
  auto root_directory = open_root_directory(maps_root);
  auto building = open_child_directory(
    root_directory, manifest.building_id,
    maps_root / manifest.building_id);
  auto floor = open_child_directory(
    building, manifest.floor_id,
    maps_root / manifest.building_id / manifest.floor_id);
  auto maps = open_child_directory(
    floor, "maps",
    maps_root / manifest.building_id / manifest.floor_id / "maps");
  auto root = open_child_directory(
    maps, manifest.map_id, manifest.root);
  auto nav = open_child_directory(root, "nav", manifest.root / "nav");
  auto localizer = open_child_directory(
    root, "localizer", manifest.root / "localizer");
  auto filters = open_child_directory(
    root, "filters", manifest.root / "filters");
  auto reports = open_child_directory(
    root, "reports", manifest.root / "reports");

  PinnedBundle bundle{
    std::move(root_directory),
    std::move(building),
    std::move(floor),
    std::move(maps),
    std::move(root),
    std::move(nav),
    std::move(localizer),
    std::move(filters),
    std::move(reports),
    {},
    {},
  };
  const auto roles = required_roles(manifest);
  bundle.assets.reserve(roles.size());
  std::uintmax_t aggregate_bytes = 0U;
  for (const auto & role : roles) {
    const auto relative = role.path.lexically_relative(manifest.root);
    if (relative.empty() || relative.is_absolute() ||
      relative.parent_path().empty() ||
      relative.parent_path().parent_path() != fs::path{})
    {
      throw std::runtime_error(
              "map asset role path is not a direct trusted child: " +
              role.path.string());
    }
    auto asset = open_asset(
      role_parent(bundle, relative.parent_path()), role);
    const auto bytes = static_cast<std::uintmax_t>(asset.initial.st_size);
    if (bytes > kMaximumBundleBytes - aggregate_bytes) {
      throw std::runtime_error(
              "map asset bundle exceeds the aggregate size limit");
    }
    aggregate_bytes += bytes;
    bundle.assets.push_back(std::move(asset));
  }
  bundle.poses = open_asset(
    bundle.root,
    RolePath{"poses_yaml", manifest.poses_yaml},
    kMaximumPosesBytes);
  return bundle;
}

void revalidate_bundle(const PinnedBundle & bundle)
{
  for (const auto & asset : bundle.assets) {
    revalidate_asset(asset);
  }
  revalidate_asset(bundle.poses);
  revalidate_directory(bundle.reports);
  revalidate_directory(bundle.filters);
  revalidate_directory(bundle.localizer);
  revalidate_directory(bundle.nav);
  revalidate_directory(bundle.root);
  revalidate_directory(bundle.maps);
  revalidate_directory(bundle.floor);
  revalidate_directory(bundle.building);
  revalidate_directory(bundle.maps_root);
}

bool keepout_payload_role(const std::string & logical_name)
{
  return logical_name == "keepout_mask_yaml" ||
         logical_name == "keepout_mask_pgm";
}

std::string digest_bundle(
  PinnedBundle & bundle,
  std::string * non_keepout_digest = nullptr)
{
  robot_map_asset_identity::CanonicalMapAssetDigestStream digest;
  robot_map_asset_identity::CanonicalMapAssetDigestStream non_keepout;
  std::array<char, 64U * 1024U> buffer{};
  for (auto & asset : bundle.assets) {
    if (::lseek(asset.descriptor.get(), 0, SEEK_SET) < 0) {
      throw io_error("rewind map asset", asset.path);
    }
    digest.begin_entry(
      asset.logical_name,
      static_cast<std::uint64_t>(asset.initial.st_size));
    const bool include_non_keepout =
      non_keepout_digest != nullptr &&
      !keepout_payload_role(asset.logical_name);
    if (include_non_keepout) {
      non_keepout.begin_entry(
        asset.logical_name,
        static_cast<std::uint64_t>(asset.initial.st_size));
    }
    std::uint64_t bytes_read = 0U;
    while (true) {
      const auto count =
        ::read(asset.descriptor.get(), buffer.data(), buffer.size());
      if (count == 0) {
        break;
      }
      if (count < 0) {
        if (errno == EINTR) {
          continue;
        }
        throw io_error("read map asset", asset.path);
      }
      digest.update(
        std::string_view(buffer.data(), static_cast<std::size_t>(count)));
      if (include_non_keepout) {
        non_keepout.update(
          std::string_view(
            buffer.data(), static_cast<std::size_t>(count)));
      }
      bytes_read += static_cast<std::uint64_t>(count);
    }
    if (bytes_read != static_cast<std::uint64_t>(asset.initial.st_size)) {
      throw std::runtime_error(
              "map asset size changed while it was hashed: " +
              asset.path.string());
    }
    digest.end_entry();
    if (include_non_keepout) {
      non_keepout.end_entry();
    }
  }
  if (non_keepout_digest != nullptr) {
    *non_keepout_digest = non_keepout.finish();
  }
  return digest.finish();
}

std::string read_asset_bytes(
  PinnedAsset & asset,
  const std::size_t maximum_bytes,
  const bool allow_prefix)
{
  if (asset.initial.st_size < 0) {
    throw std::runtime_error(
            "map asset has a negative size: " + asset.path.string());
  }
  const auto asset_size =
    static_cast<std::uint64_t>(asset.initial.st_size);
  if (!allow_prefix && asset_size > maximum_bytes) {
    throw std::runtime_error(
            "map asset exceeds snapshot read limit: " + asset.path.string());
  }
  const auto requested = static_cast<std::size_t>(
    std::min<std::uint64_t>(asset_size, maximum_bytes));
  if (::lseek(asset.descriptor.get(), 0, SEEK_SET) < 0) {
    throw io_error("rewind map snapshot asset", asset.path);
  }
  std::string content(requested, '\0');
  std::size_t offset = 0U;
  while (offset < requested) {
    const auto count = ::read(
      asset.descriptor.get(), content.data() + offset, requested - offset);
    if (count == 0) {
      break;
    }
    if (count < 0) {
      if (errno == EINTR) {
        continue;
      }
      throw io_error("read map snapshot asset", asset.path);
    }
    offset += static_cast<std::size_t>(count);
  }
  if (offset != requested) {
    throw std::runtime_error(
            "map snapshot asset ended before its pinned size: " +
            asset.path.string());
  }
  return content;
}

PinnedAsset & find_asset(
  PinnedBundle & bundle,
  const std::string & logical_name)
{
  const auto match = std::find_if(
    bundle.assets.begin(), bundle.assets.end(),
    [&logical_name](const PinnedAsset & asset) {
      return asset.logical_name == logical_name;
    });
  if (match == bundle.assets.end()) {
    throw std::runtime_error(
            "required map snapshot role is missing: " + logical_name);
  }
  return *match;
}

PinnedAsset open_manifest_asset(
  const PinnedBundle & bundle,
  const MapManifest & manifest)
{
  return open_asset(
    bundle.root,
    RolePath{"manifest_json", manifest.manifest_json},
    kMaximumManifestBytes);
}

bool same_manifest_snapshot(
  const MapManifest & left,
  const MapManifest & right)
{
  return
    left.schema == right.schema &&
    left.asset_epoch == right.asset_epoch &&
    left.asset_digest_algorithm == right.asset_digest_algorithm &&
    left.asset_digest_contract == right.asset_digest_contract &&
    left.asset_digest == right.asset_digest &&
    left.map_id == right.map_id &&
    left.display_name == right.display_name &&
    left.safe_map_name == right.safe_map_name &&
    left.building_id == right.building_id &&
    left.floor_id == right.floor_id &&
    left.created_at == right.created_at &&
    left.active == right.active &&
    same_normalized_path(left.root, right.root) &&
    same_normalized_path(left.manifest_json, right.manifest_json) &&
    same_normalized_path(left.nav_map_yaml, right.nav_map_yaml) &&
    same_normalized_path(left.nav_map_pgm, right.nav_map_pgm) &&
    same_normalized_path(left.localizer_map_png, right.localizer_map_png) &&
    same_normalized_path(
    left.localizer_params_yaml, right.localizer_params_yaml) &&
    same_normalized_path(left.keepout_mask_yaml, right.keepout_mask_yaml) &&
    same_normalized_path(left.keepout_mask_pgm, right.keepout_mask_pgm) &&
    same_normalized_path(left.speed_mask_yaml, right.speed_mask_yaml) &&
    same_normalized_path(left.speed_mask_pgm, right.speed_mask_pgm) &&
    same_normalized_path(left.binary_mask_yaml, right.binary_mask_yaml) &&
    same_normalized_path(left.binary_mask_pgm, right.binary_mask_pgm) &&
    same_normalized_path(left.asset_report_json, right.asset_report_json) &&
    same_normalized_path(left.poses_yaml, right.poses_yaml);
}

MapManifest read_pinned_manifest(
  PinnedAsset & manifest_asset,
  const fs::path & maps_root)
{
  const auto text =
    read_asset_bytes(manifest_asset, kMaximumManifestBytes, false);
  const auto parsed =
    parse_map_manifest_json(text, manifest_asset.path);
  if (!parsed) {
    throw std::runtime_error(
            "map manifest is not valid strict manifest JSON: " +
            manifest_asset.path.string());
  }
  validate_manifest_paths(*parsed, maps_root);
  revalidate_asset(manifest_asset);
  return *parsed;
}

void write_all(
  const int descriptor,
  const std::string & content,
  const fs::path & path)
{
  std::size_t offset = 0U;
  while (offset < content.size()) {
    const auto count = ::write(
      descriptor, content.data() + offset, content.size() - offset);
    if (count < 0) {
      if (errno == EINTR) {
        continue;
      }
      throw io_error("write map manifest", path);
    }
    if (count == 0) {
      throw std::runtime_error(
              "write map manifest made no progress for " + path.string());
    }
    offset += static_cast<std::size_t>(count);
  }
}

void write_manifest_at(
  const MapManifest & manifest,
  const PinnedDirectory & bundle_root)
{
  const auto content = map_manifest_json(manifest);
  if (content.size() > kMaximumManifestBytes) {
    throw std::runtime_error("map manifest exceeds the size limit");
  }

  struct stat existing {};
  if (::fstatat(
      bundle_root.descriptor.get(), "manifest.json", &existing,
      AT_SYMLINK_NOFOLLOW) == 0)
  {
    if (!S_ISREG(existing.st_mode) || existing.st_nlink != 1) {
      throw std::runtime_error(
              "map manifest destination must be a single-link regular file");
    }
  } else if (errno != ENOENT) {
    throw io_error(
            "inspect map manifest destination",
            manifest.manifest_json);
  }

  static std::atomic<std::uint64_t> sequence{0U};
  const auto temporary_name =
    ".manifest.json.tmp." + std::to_string(::getpid()) + "." +
    std::to_string(sequence.fetch_add(1U));
  UniqueFd temporary(
    ::openat(
      bundle_root.descriptor.get(), temporary_name.c_str(),
      O_WRONLY | O_CREAT | O_EXCL | O_CLOEXEC | O_NOFOLLOW,
      0644));
  if (temporary.get() < 0) {
    throw io_error(
            "create temporary map manifest",
            bundle_root.path / temporary_name);
  }
  bool renamed = false;
  try {
    write_all(temporary.get(), content, bundle_root.path / temporary_name);
    if (::fsync(temporary.get()) != 0) {
      throw io_error(
              "fsync temporary map manifest",
              bundle_root.path / temporary_name);
    }
    if (::renameat(
        bundle_root.descriptor.get(), temporary_name.c_str(),
        bundle_root.descriptor.get(), "manifest.json") != 0)
    {
      throw io_error("commit map manifest", manifest.manifest_json);
    }
    renamed = true;
    if (::fsync(bundle_root.descriptor.get()) != 0) {
      throw io_error("fsync map bundle directory", bundle_root.path);
    }
  } catch (...) {
    if (!renamed) {
      (void)::unlinkat(
        bundle_root.descriptor.get(), temporary_name.c_str(), 0);
    }
    throw;
  }
}

#else

void require_real_directory(const fs::path & path)
{
  std::error_code error;
  const auto status = fs::symlink_status(path, error);
  if (error || status.type() != fs::file_type::directory) {
    throw std::runtime_error(
            "map bundle path component must be a real directory: " +
            path.string());
  }
}

std::string digest_bundle_windows(
  const MapManifest & manifest,
  const fs::path & maps_root,
  std::string * non_keepout_digest = nullptr)
{
  require_real_directory(maps_root);
  require_real_directory(maps_root / manifest.building_id);
  require_real_directory(maps_root / manifest.building_id / manifest.floor_id);
  require_real_directory(
    maps_root / manifest.building_id / manifest.floor_id / "maps");
  require_real_directory(manifest.root);
  for (const auto & directory : {"nav", "localizer", "filters", "reports"}) {
    require_real_directory(manifest.root / directory);
  }
  {
    std::error_code error;
    const auto status = fs::symlink_status(manifest.poses_yaml, error);
    const auto size = fs::file_size(manifest.poses_yaml, error);
    if (error || status.type() != fs::file_type::regular ||
      fs::hard_link_count(manifest.poses_yaml, error) != 1U || error ||
      size > kMaximumPosesBytes)
    {
      throw std::runtime_error(
              "map poses must be a single-link bounded regular file: " +
              manifest.poses_yaml.string());
    }
  }

  robot_map_asset_identity::CanonicalMapAssetDigestStream digest;
  robot_map_asset_identity::CanonicalMapAssetDigestStream non_keepout;
  std::uintmax_t aggregate_bytes = 0U;
  std::array<char, 64U * 1024U> buffer{};
  for (const auto & role : required_roles(manifest)) {
    std::error_code error;
    const auto status = fs::symlink_status(role.path, error);
    const auto size = fs::file_size(role.path, error);
    if (error || status.type() != fs::file_type::regular ||
      fs::hard_link_count(role.path, error) != 1U || error ||
      size > kMaximumAssetBytes ||
      size > kMaximumBundleBytes - aggregate_bytes)
    {
      throw std::runtime_error(
              "map asset must be a single-link bounded regular file: " +
              role.path.string());
    }
    aggregate_bytes += size;
    std::ifstream input(role.path, std::ios::binary);
    if (!input) {
      throw std::runtime_error(
              "failed to open map asset: " + role.path.string());
    }
    digest.begin_entry(role.logical_name, static_cast<std::uint64_t>(size));
    const bool include_non_keepout =
      non_keepout_digest != nullptr &&
      !keepout_payload_role(role.logical_name);
    if (include_non_keepout) {
      non_keepout.begin_entry(
        role.logical_name, static_cast<std::uint64_t>(size));
    }
    std::uint64_t bytes_read = 0U;
    while (input) {
      input.read(buffer.data(), static_cast<std::streamsize>(buffer.size()));
      const auto count = input.gcount();
      if (count > 0) {
        digest.update(
          std::string_view(
            buffer.data(), static_cast<std::size_t>(count)));
        if (include_non_keepout) {
          non_keepout.update(
            std::string_view(
              buffer.data(), static_cast<std::size_t>(count)));
        }
        bytes_read += static_cast<std::uint64_t>(count);
      }
    }
    if (!input.eof() || bytes_read != size) {
      throw std::runtime_error(
              "map asset changed while it was hashed: " +
              role.path.string());
    }
    digest.end_entry();
    if (include_non_keepout) {
      non_keepout.end_entry();
    }
  }
  if (non_keepout_digest != nullptr) {
    *non_keepout_digest = non_keepout.finish();
  }
  return digest.finish();
}

#endif

}  // namespace

struct MapAssetCommitTransaction::Impl
{
  explicit Impl(const fs::path & requested_maps_root)
  : maps_root_path(fs::absolute(requested_maps_root).lexically_normal()),
    process_guard(g_process_commit_mutex)
#ifndef _WIN32
    , maps_root(open_root_directory(maps_root_path)),
    commit_lock(maps_root, true, LOCK_EX)
#endif
  {
#ifdef _WIN32
    require_real_directory(maps_root_path);
#endif
  }

  fs::path maps_root_path;
  std::unique_lock<std::mutex> process_guard;
#ifndef _WIN32
  PinnedDirectory maps_root;
  CommitFileLock commit_lock;
#endif
};

MapAssetCommitTransaction::MapAssetCommitTransaction(
  const fs::path & maps_root)
: impl_(std::make_unique<Impl>(maps_root))
{
}

MapAssetCommitTransaction::~MapAssetCommitTransaction() = default;

namespace
{

VerifiedMapAssetSnapshot verify_map_asset_identity_snapshot_locked(
  const MapManifest & manifest,
  const fs::path & maps_root,
  const bool require_aggregate_digest_match
#ifndef _WIN32
  , const PinnedDirectory * transaction_root
#endif
)
{
  validate_manifest_paths(manifest, maps_root);
  validate_committed_identity(manifest);

#ifndef _WIN32
  auto bundle = open_bundle(manifest, maps_root);
  std::unique_ptr<CommitFileLock> commit_lock;
  if (transaction_root == nullptr) {
    commit_lock =
      std::make_unique<CommitFileLock>(
      bundle.maps_root, false, LOCK_SH);
  } else {
    revalidate_directory(*transaction_root);
    if (!same_directory_identity(
        transaction_root->initial, bundle.maps_root.initial))
    {
      throw std::runtime_error(
              "managed maps root changed during read transaction");
    }
  }
  auto manifest_asset = open_manifest_asset(bundle, manifest);
  const auto disk_manifest =
    read_pinned_manifest(manifest_asset, maps_root);
  if (!same_manifest_snapshot(manifest, disk_manifest)) {
    throw std::runtime_error(
            "catalog manifest does not match the descriptor-pinned manifest");
  }
  validate_committed_identity(disk_manifest);
  require_registry_identity(disk_manifest, maps_root);
  revalidate_bundle(bundle);
  std::string non_keepout_digest;
  const auto digest = digest_bundle(bundle, &non_keepout_digest);
  constexpr std::size_t kMaximumNavYamlSnapshotBytes = 1024U * 1024U;
  constexpr std::size_t kMaximumPgmHeaderBytes = 1024U * 1024U;
  auto & nav_yaml_asset = find_asset(bundle, "nav_map_yaml");
  auto & nav_pgm_asset = find_asset(bundle, "nav_map_pgm");
  auto nav_yaml = read_asset_bytes(
    nav_yaml_asset, kMaximumNavYamlSnapshotBytes, false);
  auto nav_pgm_header = read_asset_bytes(
    nav_pgm_asset, kMaximumPgmHeaderBytes, true);
  revalidate_bundle(bundle);
  revalidate_asset(manifest_asset);
#else
  std::string non_keepout_digest;
  const auto digest = digest_bundle_windows(
    manifest, maps_root, &non_keepout_digest);
  const auto disk_manifest = read_map_manifest(manifest.manifest_json);
  if (!disk_manifest || !same_manifest_snapshot(manifest, *disk_manifest)) {
    throw std::runtime_error(
            "catalog manifest does not match the persisted manifest");
  }
  require_registry_identity(*disk_manifest, maps_root);
  constexpr std::uintmax_t kMaximumNavYamlSnapshotBytes = 1024U * 1024U;
  std::error_code nav_yaml_error;
  if (fs::file_size(manifest.nav_map_yaml, nav_yaml_error) >
      kMaximumNavYamlSnapshotBytes || nav_yaml_error)
  {
    throw std::runtime_error("navigation map YAML exceeds snapshot limit");
  }
  std::ifstream nav_yaml_input(manifest.nav_map_yaml, std::ios::binary);
  std::string nav_yaml{
    std::istreambuf_iterator<char>(nav_yaml_input),
    std::istreambuf_iterator<char>()};
  constexpr std::size_t kMaximumPgmHeaderBytes = 1024U * 1024U;
  std::ifstream nav_pgm_input(manifest.nav_map_pgm, std::ios::binary);
  std::string nav_pgm_header(kMaximumPgmHeaderBytes, '\0');
  nav_pgm_input.read(
    nav_pgm_header.data(),
    static_cast<std::streamsize>(nav_pgm_header.size()));
  nav_pgm_header.resize(
    static_cast<std::size_t>(nav_pgm_input.gcount()));
#endif
  if (require_aggregate_digest_match &&
    digest != manifest.asset_digest)
  {
    throw std::runtime_error(
            "map bundle content digest differs from its authoritative identity");
  }
  require_registry_identity(manifest, maps_root);
  return {
    manifest,
    std::move(nav_yaml),
    std::move(nav_pgm_header),
    std::move(non_keepout_digest),
  };
}

}  // namespace

VerifiedMapAssetSnapshot verify_map_asset_identity_snapshot(
  const MapManifest & manifest,
  const fs::path & maps_root)
{
  std::lock_guard<std::mutex> process_guard(g_process_commit_mutex);
  return verify_map_asset_identity_snapshot_locked(
    manifest, maps_root, true
#ifndef _WIN32
    , nullptr
#endif
  );
}

VerifiedMapAssetSnapshot verify_map_asset_identity_snapshot(
  const MapManifest & manifest,
  const fs::path & maps_root,
  MapAssetCommitTransaction & transaction)
{
  if (!transaction.impl_) {
    throw std::runtime_error("map asset commit transaction is not initialized");
  }
  const auto normalized_root =
    fs::absolute(maps_root).lexically_normal();
  if (normalized_root != transaction.impl_->maps_root_path) {
    throw std::runtime_error(
            "map asset commit transaction belongs to a different maps root");
  }
  return verify_map_asset_identity_snapshot_locked(
    manifest, maps_root, true
#ifndef _WIN32
    , &transaction.impl_->maps_root
#endif
  );
}

VerifiedMapAssetSnapshot inspect_map_asset_identity_for_keepout_repair(
  const MapManifest & manifest,
  const fs::path & maps_root,
  MapAssetCommitTransaction & transaction)
{
  if (!transaction.impl_) {
    throw std::runtime_error("map asset commit transaction is not initialized");
  }
  const auto normalized_root =
    fs::absolute(maps_root).lexically_normal();
  if (normalized_root != transaction.impl_->maps_root_path) {
    throw std::runtime_error(
            "map asset commit transaction belongs to a different maps root");
  }
  return verify_map_asset_identity_snapshot_locked(
    manifest, maps_root, false
#ifndef _WIN32
    , &transaction.impl_->maps_root
#endif
  );
}

void verify_map_asset_identity(
  const MapManifest & manifest,
  const fs::path & maps_root)
{
  (void)verify_map_asset_identity_snapshot(manifest, maps_root);
}

void stamp_map_asset_identity(
  MapManifest & manifest,
  const fs::path & maps_root)
{
  MapAssetCommitTransaction transaction(maps_root);
  stamp_map_asset_identity(
    manifest, maps_root, transaction, std::nullopt);
}

void stamp_map_asset_identity(
  MapManifest & manifest,
  const fs::path & maps_root,
  MapAssetCommitTransaction & transaction,
  const std::optional<bool> active_override)
{
  if (!transaction.impl_) {
    throw std::runtime_error("map asset commit transaction is not initialized");
  }
  const auto normalized_root =
    fs::absolute(maps_root).lexically_normal();
  if (normalized_root != transaction.impl_->maps_root_path) {
    throw std::runtime_error(
            "map asset commit transaction belongs to a different maps root");
  }
  validate_manifest_paths(manifest, maps_root);

#ifndef _WIN32
  revalidate_directory(transaction.impl_->maps_root);
  auto bundle = open_bundle(manifest, maps_root);
  if (!same_directory_identity(
      transaction.impl_->maps_root.initial,
      bundle.maps_root.initial))
  {
    throw std::runtime_error(
            "managed maps root changed during commit transaction");
  }
  std::optional<MapManifest> persisted_manifest;
  struct stat manifest_status {};
  if (::fstatat(
      bundle.root.descriptor.get(), "manifest.json", &manifest_status,
      AT_SYMLINK_NOFOLLOW) == 0)
  {
    auto manifest_asset = open_manifest_asset(bundle, manifest);
    persisted_manifest = read_pinned_manifest(manifest_asset, maps_root);
  } else if (errno != ENOENT) {
    throw io_error("inspect existing map manifest", manifest.manifest_json);
  }
  revalidate_bundle(bundle);
  const auto digest = digest_bundle(bundle);
  revalidate_bundle(bundle);
#else
  std::optional<MapManifest> persisted_manifest;
  if (fs::exists(manifest.manifest_json)) {
    persisted_manifest = read_map_manifest(manifest.manifest_json);
    if (!persisted_manifest) {
      throw std::runtime_error(
              "existing map manifest is invalid and will not be overwritten");
    }
  }
  const auto digest = digest_bundle_windows(manifest, maps_root);
#endif
  if (persisted_manifest) {
    if (persisted_manifest->building_id != manifest.building_id ||
      persisted_manifest->floor_id != manifest.floor_id ||
      persisted_manifest->map_id != manifest.map_id ||
      persisted_manifest->safe_map_name != manifest.safe_map_name)
    {
      throw std::runtime_error(
              "existing map manifest identity differs from commit request");
    }
    manifest.display_name = persisted_manifest->display_name;
    manifest.created_at = persisted_manifest->created_at;
    manifest.active = persisted_manifest->active;
  }
  if (active_override) {
    manifest.active = *active_override;
  }

  robot_map_asset_identity::PersistentAssetEpochRegistry registry(maps_root);
  const auto identity = registry.bind(identity_key(manifest), digest);

#ifndef _WIN32
  revalidate_bundle(bundle);
#endif
  manifest.schema = kManifestSchema;
  manifest.asset_epoch = identity.asset_epoch;
  manifest.asset_digest_algorithm = kDigestAlgorithm;
  manifest.asset_digest_contract = kDigestContract;
  manifest.asset_digest = identity.asset_digest;
#ifndef _WIN32
  write_manifest_at(manifest, bundle.root);
  revalidate_bundle(bundle);
  revalidate_directory(transaction.impl_->maps_root);
#else
  write_map_manifest(manifest);
#endif
  require_registry_identity(manifest, maps_root);
}

}  // namespace robot_api_server
