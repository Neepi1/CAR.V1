#pragma once

#include <array>
#include <cstdint>
#include <filesystem>
#include <string>

namespace robot_api_server
{

struct StoredPose
{
  std::string id;
  std::string name;
  std::string type{"delivery_point"};
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

struct MapManifest
{
  std::string map_id;
  std::string display_name;
  std::string safe_map_name;
  std::string building_id;
  std::string floor_id;
  std::string created_at;
  bool active{false};
  std::filesystem::path root;
  std::filesystem::path manifest_json;
  std::filesystem::path nav_map_yaml;
  std::filesystem::path nav_map_pgm;
  std::filesystem::path localizer_map_png;
  std::filesystem::path localizer_params_yaml;
  std::filesystem::path keepout_mask_yaml;
  std::filesystem::path keepout_mask_pgm;
  std::filesystem::path speed_mask_yaml;
  std::filesystem::path speed_mask_pgm;
  std::filesystem::path binary_mask_yaml;
  std::filesystem::path binary_mask_pgm;
  std::filesystem::path asset_report_json;
  std::filesystem::path poses_yaml;
};

struct RuntimeMapContext
{
  bool confirmed{false};
  std::string state;
  std::string startup_stage;
  std::string message;
  std::string map_id;
  std::string display_name;
  std::string building_id;
  std::string floor_id;
  double updated_at_sec{0.0};
};

struct MapYamlInfo
{
  std::uint32_t width{0};
  std::uint32_t height{0};
  double resolution{0.0};
  std::array<double, 3> origin{0.0, 0.0, 0.0};
};

bool safe_pose_id(const std::string & pose_id);
bool safe_asset_id(const std::string & id);
bool valid_display_map_name(const std::string & name);
std::string safe_file_stem_from_display_name(const std::string & display_name);
std::uint64_t fnv1a64(const std::string & value);
std::string fixed_hex(std::uint64_t value, int width);

}  // namespace robot_api_server
