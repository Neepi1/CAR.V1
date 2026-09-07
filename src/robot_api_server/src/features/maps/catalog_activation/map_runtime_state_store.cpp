#include \
  "robot_api_server/features/maps/catalog_activation/map_runtime_state_store.hpp"

#include <iomanip>
#include <sstream>
#include <system_error>
#include <utility>

#include "robot_api_server/features/floor_switch/runtime_map_context_io.hpp"
#include "robot_api_server/features/maps/catalog_activation/api_time_utils.hpp"
#include "robot_api_server/features/maps/catalog_activation/file_utils.hpp"
#include "robot_api_server/infrastructure/http/http_common.hpp"

namespace robot_api_server::features::maps
{

MapRuntimeStateStore::MapRuntimeStateStore(
  std::filesystem::path runtime_map_context_file,
  std::filesystem::path last_navigation_map_file)
: runtime_map_context_file_(std::move(runtime_map_context_file)),
  last_navigation_map_file_(std::move(last_navigation_map_file))
{
}

void MapRuntimeStateStore::write_runtime_map_context(
  const MapManifest & manifest,
  const std::string & state,
  const bool confirmed,
  const std::string & message,
  const std::string & startup_stage) const
{
  if (runtime_map_context_file_.empty()) {
    return;
  }
  write_runtime_map_context_file(
    runtime_map_context_file_,
    manifest,
    state,
    confirmed,
    message,
    wall_time_seconds(),
    startup_stage);
}

std::optional<RuntimeMapContext> MapRuntimeStateStore::read_runtime_map_context() const
{
  if (runtime_map_context_file_.empty()) {
    return std::nullopt;
  }
  return read_runtime_map_context_file(runtime_map_context_file_);
}

void MapRuntimeStateStore::clear_runtime_map_context() const
{
  if (runtime_map_context_file_.empty()) {
    return;
  }
  std::error_code error;
  std::filesystem::remove(runtime_map_context_file_, error);
}

void MapRuntimeStateStore::write_last_navigation_map_selection(
  const MapManifest & manifest,
  const std::string & reason) const
{
  if (last_navigation_map_file_.empty()) {
    return;
  }
  std::ostringstream body;
  body << std::fixed << std::setprecision(6)
       << "{\n"
       << "  \"schema\": \"njrh.last_navigation_map.v1\",\n"
       << "  \"reason\": " << json_string(reason) << ",\n"
       << "  \"map_id\": " << json_string(manifest.map_id) << ",\n"
       << "  \"display_name\": " << json_string(manifest.display_name) << ",\n"
       << "  \"building_id\": " << json_string(manifest.building_id) << ",\n"
       << "  \"floor_id\": " << json_string(manifest.floor_id) << ",\n"
       << "  \"updated_at\": " << wall_time_seconds() << "\n"
       << "}\n";
  write_text_file(last_navigation_map_file_, body.str());
}

}  // namespace robot_api_server::features::maps
