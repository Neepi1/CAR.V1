#include "robot_api_server/map_manifest_io.hpp"

#include <fstream>
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

void fill_manifest_paths(MapManifest & manifest)
{
  manifest.manifest_json = manifest.root / "manifest.json";
  manifest.nav_map_yaml = manifest.root / "nav" / (manifest.safe_map_name + ".yaml");
  manifest.nav_map_pgm = manifest.root / "nav" / (manifest.safe_map_name + ".pgm");
  manifest.localizer_map_png = manifest.root / "localizer" / (manifest.safe_map_name + ".png");
  manifest.localizer_params_yaml = manifest.root / "localizer" / (manifest.safe_map_name + ".yaml");
  manifest.keepout_mask_yaml = manifest.root / "filters" / "keepout_mask.yaml";
  manifest.keepout_mask_pgm = manifest.root / "filters" / "keepout_mask.pgm";
  manifest.speed_mask_yaml = manifest.root / "filters" / "speed_mask.yaml";
  manifest.speed_mask_pgm = manifest.root / "filters" / "speed_mask.pgm";
  manifest.binary_mask_yaml = manifest.root / "filters" / "binary_mask.yaml";
  manifest.binary_mask_pgm = manifest.root / "filters" / "binary_mask.pgm";
  manifest.asset_report_json = manifest.root / "reports" / "asset_report.json";
  manifest.poses_yaml = manifest.root / "poses.yaml";
}

std::string map_manifest_json(const MapManifest & manifest)
{
  std::ostringstream out;
  out << "{\n"
      << "  \"map_id\": " << json_string(manifest.map_id) << ",\n"
      << "  \"display_name\": " << json_string(manifest.display_name) << ",\n"
      << "  \"map_name\": " << json_string(manifest.display_name) << ",\n"
      << "  \"safe_map_name\": " << json_string(manifest.safe_map_name) << ",\n"
      << "  \"building_id\": " << json_string(manifest.building_id) << ",\n"
      << "  \"floor_id\": " << json_string(manifest.floor_id) << ",\n"
      << "  \"created_at\": " << json_string(manifest.created_at) << ",\n"
      << "  \"active\": " << (manifest.active ? "true" : "false") << ",\n"
      << "  \"assets\": {\n"
      << "    \"nav_map_yaml\": " << json_string(manifest.nav_map_yaml.string()) << ",\n"
      << "    \"nav_map_pgm\": " << json_string(manifest.nav_map_pgm.string()) << ",\n"
      << "    \"localizer_map_png\": " << json_string(manifest.localizer_map_png.string()) << ",\n"
      << "    \"localizer_params_yaml\": " << json_string(manifest.localizer_params_yaml.string()) << ",\n"
      << "    \"keepout_mask_yaml\": " << json_string(manifest.keepout_mask_yaml.string()) << ",\n"
      << "    \"keepout_mask_pgm\": " << json_string(manifest.keepout_mask_pgm.string()) << ",\n"
      << "    \"speed_mask_yaml\": " << json_string(manifest.speed_mask_yaml.string()) << ",\n"
      << "    \"speed_mask_pgm\": " << json_string(manifest.speed_mask_pgm.string()) << ",\n"
      << "    \"binary_mask_yaml\": " << json_string(manifest.binary_mask_yaml.string()) << ",\n"
      << "    \"binary_mask_pgm\": " << json_string(manifest.binary_mask_pgm.string()) << ",\n"
      << "    \"asset_report_json\": " << json_string(manifest.asset_report_json.string()) << ",\n"
      << "    \"poses_yaml\": " << json_string(manifest.poses_yaml.string()) << "\n"
      << "  }\n"
      << "}\n";
  return out.str();
}

std::optional<MapManifest> read_map_manifest(const fs::path & manifest_path)
{
  if (!fs::exists(manifest_path) || !fs::is_regular_file(manifest_path)) {
    return std::nullopt;
  }
  const auto text = read_text_file(manifest_path);
  MapManifest manifest;
  const auto map_id = json_string_value(text, "map_id");
  const auto building_id = json_string_value(text, "building_id");
  const auto floor_id = json_string_value(text, "floor_id");
  if (!map_id || !building_id || !floor_id || !safe_asset_id(*map_id) ||
    !safe_asset_id(*building_id) || !safe_asset_id(*floor_id))
  {
    return std::nullopt;
  }
  manifest.map_id = *map_id;
  manifest.display_name = json_string_value(text, "display_name").value_or(
    json_string_value(text, "map_name").value_or(*map_id));
  manifest.safe_map_name = json_string_value(text, "safe_map_name").value_or(
    safe_file_stem_from_display_name(manifest.display_name));
  manifest.building_id = *building_id;
  manifest.floor_id = *floor_id;
  manifest.created_at = json_string_value(text, "created_at").value_or("");
  manifest.active = json_bool_value(text, "active", false);
  manifest.root = manifest_path.parent_path();
  fill_manifest_paths(manifest);
  return manifest;
}

void write_map_manifest(const MapManifest & manifest)
{
  write_text_file(manifest.manifest_json, map_manifest_json(manifest));
}

}  // namespace robot_api_server
