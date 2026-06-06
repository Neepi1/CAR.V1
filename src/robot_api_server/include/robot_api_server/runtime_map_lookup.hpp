#pragma once

#include <filesystem>
#include <optional>
#include <string>
#include <vector>

#include "robot_api_server/http_common.hpp"

namespace robot_api_server
{

bool safe_map_name(const std::string & name);

std::vector<std::filesystem::path> runtime_map_asset_paths(
  const std::filesystem::path & runtime_maps_dir,
  const std::string & map_name);

std::optional<std::filesystem::path> newest_png_in_directory(
  const std::filesystem::path & directory);

std::optional<std::filesystem::path> newest_floor_localizer_png(
  const std::filesystem::path & maps_root);

std::optional<std::filesystem::path> resolve_mapping_2d_png(
  const HttpRequest & request,
  const std::filesystem::path & runtime_maps_dir,
  const std::filesystem::path & maps_root);

}  // namespace robot_api_server
