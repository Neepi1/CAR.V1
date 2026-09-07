#pragma once

#include <filesystem>
#include <optional>
#include <string>

#include "robot_api_server/features/maps/catalog_activation/storage_models.hpp"

namespace robot_api_server::features::maps
{

// Owns the two small runtime files shared by map selection consumers. This is
// persistence only: it does not select/activate a map or call ROS/Nav2.
class MapRuntimeStateStore
{
public:
  MapRuntimeStateStore(
    std::filesystem::path runtime_map_context_file,
    std::filesystem::path last_navigation_map_file);

  void write_runtime_map_context(
    const MapManifest & manifest,
    const std::string & state,
    bool confirmed,
    const std::string & message,
    const std::string & startup_stage = "") const;
  std::optional<RuntimeMapContext> read_runtime_map_context() const;
  void clear_runtime_map_context() const;
  void write_last_navigation_map_selection(
    const MapManifest & manifest,
    const std::string & reason) const;

private:
  std::filesystem::path runtime_map_context_file_;
  std::filesystem::path last_navigation_map_file_;
};

}  // namespace robot_api_server::features::maps
