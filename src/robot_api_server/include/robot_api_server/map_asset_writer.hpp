#pragma once

#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "robot_api_server/storage_models.hpp"

namespace robot_api_server
{

std::uint8_t occupancy_to_gray(int value);
std::vector<std::uint8_t> occupancy_grid_to_image_pixels(const nav_msgs::msg::OccupancyGrid & map);
std::string map_yaml_text(const std::string & image_name, const nav_msgs::msg::OccupancyGrid & map);
void write_neutral_filter_assets(
  const std::filesystem::path & filters_dir,
  const nav_msgs::msg::OccupancyGrid & map);
void write_asset_report(const MapManifest & manifest, const nav_msgs::msg::OccupancyGrid & map);

}  // namespace robot_api_server
