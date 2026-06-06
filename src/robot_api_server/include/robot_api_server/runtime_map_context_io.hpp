#pragma once

#include <filesystem>
#include <optional>
#include <string>

#include "robot_api_server/storage_models.hpp"

namespace robot_api_server
{

void write_runtime_map_context_file(
  const std::filesystem::path & path,
  const MapManifest & manifest,
  const std::string & state,
  bool confirmed,
  const std::string & message,
  double updated_at_sec);

std::optional<RuntimeMapContext> read_runtime_map_context_file(const std::filesystem::path & path);

}  // namespace robot_api_server
