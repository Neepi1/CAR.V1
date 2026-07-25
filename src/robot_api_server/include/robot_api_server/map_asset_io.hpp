#pragma once

#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "robot_api_server/storage_models.hpp"

namespace robot_api_server
{

std::string encode_grayscale_png(
  std::uint32_t width,
  std::uint32_t height,
  const std::vector<std::uint8_t> & pixels);

std::optional<std::pair<std::uint32_t, std::uint32_t>> read_pgm_dimensions(
  const std::filesystem::path & pgm_path);

std::optional<MapYamlInfo> read_nav_map_info(const std::filesystem::path & nav_map_yaml);
std::optional<MapYamlInfo> read_nav_map_info_exact(
  const std::filesystem::path & nav_map_yaml,
  const std::filesystem::path & exact_pgm_path);
std::optional<MapYamlInfo> read_nav_map_info_exact_content(
  const std::string & nav_map_yaml,
  const std::string & nav_map_pgm_header,
  const std::string & expected_pgm_filename);
std::string map_info_json(const std::optional<MapYamlInfo> & info);

}  // namespace robot_api_server
