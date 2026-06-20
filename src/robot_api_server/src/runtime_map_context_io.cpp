#include "robot_api_server/runtime_map_context_io.hpp"

#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>

#include "robot_api_server/http_common.hpp"

namespace robot_api_server
{
namespace fs = std::filesystem;
namespace
{

std::string read_text_file(const fs::path & path)
{
  std::ifstream file(path);
  if (!file) {
    throw std::runtime_error("failed to open file for reading: " + path.string());
  }
  std::ostringstream data;
  data << file.rdbuf();
  return data.str();
}

void write_text_file(const fs::path & path, const std::string & text)
{
  fs::create_directories(path.parent_path());
  std::ofstream file(path);
  if (!file) {
    throw std::runtime_error("failed to open file for writing: " + path.string());
  }
  file << text;
}

}  // namespace

void write_runtime_map_context_file(
  const fs::path & path,
  const MapManifest & manifest,
  const std::string & state,
  const bool confirmed,
  const std::string & message,
  const double updated_at_sec,
  const std::string & startup_stage)
{
  std::ostringstream body;
  body << std::fixed << std::setprecision(6)
       << "{"
       << "\"schema\":\"njrh.runtime_map_context.v1\","
       << "\"state\":" << json_string(state) << ","
       << "\"startup_stage\":" << json_string(startup_stage) << ","
       << "\"confirmed\":" << (confirmed ? "true" : "false") << ","
       << "\"message\":" << json_string(message) << ","
       << "\"map_id\":" << json_string(manifest.map_id) << ","
       << "\"display_name\":" << json_string(manifest.display_name) << ","
       << "\"building_id\":" << json_string(manifest.building_id) << ","
       << "\"floor_id\":" << json_string(manifest.floor_id) << ","
       << "\"updated_at\":" << updated_at_sec
       << "}\n";
  write_text_file(path, body.str());
}

std::optional<RuntimeMapContext> read_runtime_map_context_file(const fs::path & path)
{
  if (!fs::exists(path) || !fs::is_regular_file(path)) {
    return std::nullopt;
  }
  const auto text = read_text_file(path);
  const auto map_id = json_string_value(text, "map_id");
  const auto building_id = json_string_value(text, "building_id");
  const auto floor_id = json_string_value(text, "floor_id");
  if (!map_id || !building_id || !floor_id || !safe_asset_id(*map_id) ||
    !safe_asset_id(*building_id) || !safe_asset_id(*floor_id))
  {
    return std::nullopt;
  }
  RuntimeMapContext context;
  context.confirmed = json_bool_value(text, "confirmed", false);
  context.state = json_string_value(text, "state").value_or("");
  context.startup_stage = json_string_value(text, "startup_stage").value_or("");
  context.message = json_string_value(text, "message").value_or("");
  context.map_id = *map_id;
  context.display_name = json_string_value(text, "display_name").value_or(*map_id);
  context.building_id = *building_id;
  context.floor_id = *floor_id;
  context.updated_at_sec = json_number_value(text, "updated_at").value_or(0.0);
  return context;
}

}  // namespace robot_api_server
