#pragma once

#include <filesystem>
#include <optional>
#include <string>

#include "robot_api_server/map_catalog.hpp"
#include "robot_api_server/storage_models.hpp"

namespace robot_api_server
{

struct FloorAssetPaths
{
  std::filesystem::path root;
  std::filesystem::path nav_map_yaml;
  std::filesystem::path localizer_map_png;
  std::filesystem::path localizer_params_yaml;
};

bool resolve_floor_asset_paths(
  const MapCatalog & map_catalog,
  const std::string & building_id,
  const std::string & floor_id,
  FloorAssetPaths & assets,
  std::string & error);

std::filesystem::path poses_yaml_path(
  const MapCatalog & map_catalog,
  const std::string & building_id,
  const std::string & floor_id);

std::optional<StoredPose> find_floor_catalog_pose(
  const MapCatalog & map_catalog,
  const std::string & building_id,
  const std::string & floor_id,
  const std::string & pose_id);

}  // namespace robot_api_server
