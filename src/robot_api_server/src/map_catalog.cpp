#include "robot_api_server/map_catalog.hpp"

#include <algorithm>
#include <exception>
#include <system_error>
#include <utility>

#include "robot_api_server/api_time_utils.hpp"
#include "robot_api_server/map_manifest_io.hpp"

namespace robot_api_server
{
namespace fs = std::filesystem;
namespace
{

bool path_lexically_within(
  const fs::path & child,
  const fs::path & parent)
{
  try {
    const auto normalized_child = fs::absolute(child).lexically_normal();
    const auto normalized_parent = fs::absolute(parent).lexically_normal();
    auto child_iterator = normalized_child.begin();
    for (auto parent_iterator = normalized_parent.begin();
      parent_iterator != normalized_parent.end();
      ++parent_iterator, ++child_iterator)
    {
      if (child_iterator == normalized_child.end() ||
        *child_iterator != *parent_iterator)
      {
        return false;
      }
    }
    return true;
  } catch (const std::exception &) {
    return false;
  }
}

bool safe_real_directory_chain(
  const fs::path & directory,
  const fs::path & managed_root)
{
  try {
    if (!path_lexically_within(directory, managed_root)) {
      return false;
    }
    const auto normalized_root =
      fs::absolute(managed_root).lexically_normal();
    const auto normalized_directory =
      fs::absolute(directory).lexically_normal();
    std::error_code error;
    fs::path cursor = normalized_root;
    auto status = fs::symlink_status(cursor, error);
    if (error || status.type() != fs::file_type::directory) {
      return false;
    }
    for (const auto & component :
      normalized_directory.lexically_relative(normalized_root))
    {
      if (component == "..") {
        return false;
      }
      cursor /= component;
      status = fs::symlink_status(cursor, error);
      if (error || status.type() != fs::file_type::directory) {
        return false;
      }
    }
    return true;
  } catch (const std::exception &) {
    return false;
  }
}

}  // namespace

MapCatalog::MapCatalog(fs::path maps_root, LegacyMigrationCallback legacy_migration)
: maps_root_(std::move(maps_root)), legacy_migration_(std::move(legacy_migration))
{
}

fs::path MapCatalog::floor_root_path(
  const std::string & building_id,
  const std::string & floor_id) const
{
  return maps_root_ / building_id / floor_id;
}

fs::path MapCatalog::floor_maps_root_path(
  const std::string & building_id,
  const std::string & floor_id) const
{
  return floor_root_path(building_id, floor_id) / "maps";
}

fs::path MapCatalog::floor_current_root_path(
  const std::string & building_id,
  const std::string & floor_id) const
{
  return floor_root_path(building_id, floor_id) / "current";
}

fs::path MapCatalog::map_root_path(
  const std::string & building_id,
  const std::string & floor_id,
  const std::string & map_id) const
{
  return floor_maps_root_path(building_id, floor_id) / map_id;
}

MapManifest MapCatalog::make_new_manifest(
  const std::string & building_id,
  const std::string & floor_id,
  const std::string & display_name) const
{
  MapManifest manifest;
  manifest.map_id = generate_map_id(building_id, floor_id, display_name);
  manifest.display_name = display_name;
  manifest.safe_map_name = safe_file_stem_from_display_name(display_name);
  manifest.building_id = building_id;
  manifest.floor_id = floor_id;
  manifest.created_at = utc_timestamp_iso8601();
  manifest.active = true;
  manifest.root = map_root_path(building_id, floor_id, manifest.map_id);
  fill_manifest_paths(manifest);
  return manifest;
}

std::vector<MapManifest> MapCatalog::read_floor_map_manifests(
  const std::string & building_id,
  const std::string & floor_id,
  const bool migrate_legacy) const
{
  if (!safe_asset_id(building_id) || !safe_asset_id(floor_id)) {
    return {};
  }
  const auto floor_root = floor_root_path(building_id, floor_id);
  if (!safe_real_directory_chain(floor_root, maps_root_)) {
    return {};
  }
  const auto maps_root = floor_maps_root_path(building_id, floor_id);
  std::error_code maps_status_error;
  const auto maps_status = fs::symlink_status(maps_root, maps_status_error);
  if (maps_status_error &&
    maps_status_error != std::errc::no_such_file_or_directory)
  {
    return {};
  }
  if (maps_status.type() != fs::file_type::not_found &&
    !safe_real_directory_chain(maps_root, maps_root_))
  {
    return {};
  }
  if (migrate_legacy && legacy_migration_) {
    legacy_migration_(building_id, floor_id);
  }

  std::vector<MapManifest> manifests;
  if (!safe_real_directory_chain(maps_root, maps_root_)) {
    return manifests;
  }
  for (const auto & entry : fs::directory_iterator(maps_root)) {
    if (entry.is_symlink() || !entry.is_directory()) {
      continue;
    }
    const auto manifest = read_map_manifest(entry.path() / "manifest.json");
    if (manifest && entry.path().filename() == manifest->map_id) {
      manifests.push_back(*manifest);
    }
  }
  std::sort(manifests.begin(), manifests.end(), [](const auto & lhs, const auto & rhs) {
    if (lhs.active != rhs.active) {
      return lhs.active > rhs.active;
    }
    return lhs.created_at > rhs.created_at;
  });
  return manifests;
}

std::vector<MapManifest> MapCatalog::read_all_map_manifests(const bool migrate_legacy) const
{
  std::vector<MapManifest> maps;
  if (!safe_real_directory_chain(maps_root_, maps_root_)) {
    return maps;
  }
  for (const auto & building : fs::directory_iterator(maps_root_)) {
    if (building.is_symlink() || !building.is_directory()) {
      continue;
    }
    const auto building_id = building.path().filename().string();
    if (!safe_asset_id(building_id)) {
      continue;
    }
    for (const auto & floor : fs::directory_iterator(building.path())) {
      if (floor.is_symlink() || !floor.is_directory()) {
        continue;
      }
      const auto floor_id = floor.path().filename().string();
      if (!safe_asset_id(floor_id)) {
        continue;
      }
      auto floor_maps = read_floor_map_manifests(building_id, floor_id, migrate_legacy);
      maps.insert(maps.end(), floor_maps.begin(), floor_maps.end());
    }
  }
  return maps;
}

std::optional<MapManifest> MapCatalog::find_map_by_id(const std::string & map_id) const
{
  if (!safe_asset_id(map_id)) {
    return std::nullopt;
  }
  for (const auto & manifest : read_all_map_manifests(false)) {
    if (manifest.map_id == map_id) {
      return manifest;
    }
  }
  return std::nullopt;
}

std::optional<MapManifest> MapCatalog::find_floor_map_by_name(
  const std::string & building_id,
  const std::string & floor_id,
  const std::string & display_name,
  std::string & error) const
{
  std::optional<MapManifest> match;
  for (const auto & manifest : read_floor_map_manifests(building_id, floor_id, false)) {
    if (manifest.display_name != display_name) {
      continue;
    }
    if (match) {
      error = "map_name is ambiguous on this floor; use map_id";
      return std::nullopt;
    }
    match = manifest;
  }
  return match;
}

std::optional<MapManifest> MapCatalog::active_floor_map(
  const std::string & building_id,
  const std::string & floor_id) const
{
  for (const auto & manifest : read_floor_map_manifests(building_id, floor_id, false)) {
    if (manifest.active) {
      return manifest;
    }
  }
  return std::nullopt;
}

std::optional<MapManifest> MapCatalog::unique_active_map_manifest() const
{
  std::optional<MapManifest> active;
  for (const auto & manifest : read_all_map_manifests(false)) {
    if (!manifest.active) {
      continue;
    }
    if (active) {
      return std::nullopt;
    }
    active = manifest;
  }
  return active;
}

}  // namespace robot_api_server
