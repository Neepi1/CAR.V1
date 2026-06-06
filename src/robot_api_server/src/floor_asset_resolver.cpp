#include "robot_api_server/floor_asset_resolver.hpp"

#include <vector>

#include "robot_api_server/poses_io.hpp"

namespace robot_api_server
{
namespace fs = std::filesystem;

bool resolve_floor_asset_paths(
  const MapCatalog & map_catalog,
  const std::string & building_id,
  const std::string & floor_id,
  FloorAssetPaths & assets,
  std::string & error)
{
  if (!safe_asset_id(building_id) || !safe_asset_id(floor_id)) {
    error = "building_id and floor_id must be safe asset ids";
    return false;
  }

  const auto floor_root = map_catalog.floor_root_path(building_id, floor_id);
  const auto current_root = map_catalog.floor_current_root_path(building_id, floor_id);
  assets.root = fs::exists(current_root / "nav" / "nav_map.yaml") ? current_root : floor_root;
  assets.nav_map_yaml = assets.root / "nav" / "nav_map.yaml";
  assets.localizer_map_png = assets.root / "localizer" / "localizer_map.png";
  assets.localizer_params_yaml = assets.root / "localizer" / "localizer_params.yaml";

  const std::vector<fs::path> required = {
    assets.nav_map_yaml,
    assets.root / "nav" / "nav_map.pgm",
    assets.localizer_map_png,
    assets.localizer_params_yaml,
    assets.root / "filters" / "keepout_mask.yaml",
    assets.root / "filters" / "keepout_mask.pgm",
    assets.root / "filters" / "speed_mask.yaml",
    assets.root / "filters" / "speed_mask.pgm",
    assets.root / "filters" / "binary_mask.yaml",
    assets.root / "filters" / "binary_mask.pgm",
    assets.root / "reports" / "asset_report.json",
    assets.root / "poses.yaml"
  };
  for (const auto & path : required) {
    if (!fs::exists(path)) {
      error = "floor asset is incomplete, missing: " + path.string();
      return false;
    }
  }
  return true;
}

fs::path poses_yaml_path(
  const MapCatalog & map_catalog,
  const std::string & building_id,
  const std::string & floor_id)
{
  const auto current_poses = map_catalog.floor_current_root_path(building_id, floor_id) / "poses.yaml";
  if (fs::exists(current_poses)) {
    return current_poses;
  }
  return map_catalog.floor_root_path(building_id, floor_id) / "poses.yaml";
}

std::optional<StoredPose> find_floor_catalog_pose(
  const MapCatalog & map_catalog,
  const std::string & building_id,
  const std::string & floor_id,
  const std::string & pose_id)
{
  const auto path = poses_yaml_path(map_catalog, building_id, floor_id);
  if (!fs::exists(path)) {
    return std::nullopt;
  }
  return find_floor_pose(path, pose_id);
}

}  // namespace robot_api_server
