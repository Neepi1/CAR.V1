#include "robot_api_server/map_asset_writer.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>

#include "robot_api_server/file_utils.hpp"
#include "robot_api_server/http_common.hpp"

namespace robot_api_server
{

namespace fs = std::filesystem;

namespace
{

double quaternion_yaw(const double x, const double y, const double z, const double w)
{
  const double siny_cosp = 2.0 * (w * z + x * y);
  const double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
  return std::atan2(siny_cosp, cosy_cosp);
}

double map_origin_yaw(const nav_msgs::msg::OccupancyGrid & map)
{
  const auto & q = map.info.origin.orientation;
  return quaternion_yaw(q.x, q.y, q.z, q.w);
}

}  // namespace

std::uint8_t occupancy_to_gray(const int value)
{
  if (value < 0) {
    return 205U;
  }
  if (value == 0) {
    return 254U;
  }
  if (value >= 100) {
    return 0U;
  }
  const double occupied_ratio = static_cast<double>(value) / 100.0;
  return static_cast<std::uint8_t>(std::clamp(254.0 - occupied_ratio * 254.0, 0.0, 254.0));
}

std::vector<std::uint8_t> occupancy_grid_to_image_pixels(const nav_msgs::msg::OccupancyGrid & map)
{
  const std::uint32_t width = map.info.width;
  const std::uint32_t height = map.info.height;
  std::vector<std::uint8_t> pixels(static_cast<std::size_t>(width) * height);
  for (std::uint32_t y = 0; y < height; ++y) {
    const std::uint32_t src_y = height - 1U - y;
    for (std::uint32_t x = 0; x < width; ++x) {
      const std::size_t src = static_cast<std::size_t>(src_y) * width + x;
      const std::size_t dst = static_cast<std::size_t>(y) * width + x;
      pixels[dst] = occupancy_to_gray(static_cast<int>(map.data[src]));
    }
  }
  return pixels;
}

std::string map_yaml_text(const std::string & image_name, const nav_msgs::msg::OccupancyGrid & map)
{
  std::ostringstream yaml;
  yaml << std::fixed << std::setprecision(6);
  yaml << "image: " << image_name << "\n";
  yaml << "resolution: " << map.info.resolution << "\n";
  yaml << "origin: [" << map.info.origin.position.x << ", " << map.info.origin.position.y << ", "
       << map_origin_yaw(map) << "]\n";
  yaml << "negate: 0\n";
  yaml << "occupied_thresh: 0.65\n";
  yaml << "free_thresh: 0.196\n";
  yaml << "mode: trinary\n";
  return yaml.str();
}

void write_neutral_filter_assets(const fs::path & filters_dir, const nav_msgs::msg::OccupancyGrid & map)
{
  const std::uint32_t width = map.info.width;
  const std::uint32_t height = map.info.height;
  const std::vector<std::uint8_t> neutral(static_cast<std::size_t>(width) * height, 254U);
  for (const auto & stem : {"keepout_mask", "speed_mask", "binary_mask"}) {
    write_pgm_file(filters_dir / (std::string(stem) + ".pgm"), width, height, neutral);
    write_text_file(filters_dir / (std::string(stem) + ".yaml"), map_yaml_text(std::string(stem) + ".pgm", map));
  }
}

void write_asset_report(const MapManifest & manifest, const nav_msgs::msg::OccupancyGrid & map)
{
  std::ostringstream report;
  report << "{\n";
  report << "  \"producer\": \"robot_api_server_slam_toolbox_save\",\n";
  report << "  \"map_id\": " << json_string(manifest.map_id) << ",\n";
  report << "  \"display_name\": " << json_string(manifest.display_name) << ",\n";
  report << "  \"map_name\": " << json_string(manifest.display_name) << ",\n";
  report << "  \"building_id\": " << json_string(manifest.building_id) << ",\n";
  report << "  \"floor_id\": " << json_string(manifest.floor_id) << ",\n";
  report << "  \"resolution\": " << map.info.resolution << ",\n";
  report << "  \"width\": " << map.info.width << ",\n";
  report << "  \"height\": " << map.info.height << ",\n";
  report << "  \"nav_map\": " << json_string(manifest.nav_map_yaml.string()) << ",\n";
  report << "  \"localizer_map\": " << json_string(manifest.localizer_params_yaml.string()) << "\n";
  report << "}\n";
  write_text_file(manifest.asset_report_json, report.str());
}

}  // namespace robot_api_server
