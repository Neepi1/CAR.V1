#pragma once

#include <filesystem>
#include <functional>
#include <optional>
#include <string>
#include <vector>

#include "robot_api_server/storage_models.hpp"

namespace robot_api_server
{

class MapCatalog
{
public:
  using LegacyMigrationCallback = std::function<void(const std::string &, const std::string &)>;

  explicit MapCatalog(
    std::filesystem::path maps_root,
    LegacyMigrationCallback legacy_migration = {});

  std::filesystem::path floor_root_path(
    const std::string & building_id,
    const std::string & floor_id) const;
  std::filesystem::path floor_maps_root_path(
    const std::string & building_id,
    const std::string & floor_id) const;
  std::filesystem::path floor_current_root_path(
    const std::string & building_id,
    const std::string & floor_id) const;
  std::filesystem::path map_root_path(
    const std::string & building_id,
    const std::string & floor_id,
    const std::string & map_id) const;

  MapManifest make_new_manifest(
    const std::string & building_id,
    const std::string & floor_id,
    const std::string & display_name) const;

  std::vector<MapManifest> read_floor_map_manifests(
    const std::string & building_id,
    const std::string & floor_id,
    bool migrate_legacy = true) const;
  std::vector<MapManifest> read_all_map_manifests(bool migrate_legacy = true) const;

  std::optional<MapManifest> find_map_by_id(const std::string & map_id) const;
  std::optional<MapManifest> find_floor_map_by_name(
    const std::string & building_id,
    const std::string & floor_id,
    const std::string & display_name,
    std::string & error) const;
  std::optional<MapManifest> active_floor_map(
    const std::string & building_id,
    const std::string & floor_id) const;
  std::optional<MapManifest> unique_active_map_manifest() const;

private:
  std::filesystem::path maps_root_;
  LegacyMigrationCallback legacy_migration_;
};

}  // namespace robot_api_server
