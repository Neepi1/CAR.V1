#pragma once

#include <filesystem>
#include <optional>
#include <string>

#include "robot_api_server/storage_models.hpp"

namespace robot_api_server
{

void fill_manifest_paths(MapManifest & manifest);
std::string map_manifest_json(const MapManifest & manifest);
std::optional<MapManifest> read_map_manifest(const std::filesystem::path & manifest_path);
void write_map_manifest(const MapManifest & manifest);

}  // namespace robot_api_server
