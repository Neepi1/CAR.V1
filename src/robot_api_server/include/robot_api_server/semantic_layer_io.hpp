#pragma once

#include <filesystem>
#include <string>

#include "robot_api_server/storage_models.hpp"

namespace robot_api_server
{

std::filesystem::path keepout_semantic_json_path(const MapManifest & manifest);
std::string json_raw_or_null(const std::string & text);
std::string keepout_semantic_payload_json(const std::string & semantic_json);
std::string keepout_filter_json(const MapManifest & manifest);

}  // namespace robot_api_server
