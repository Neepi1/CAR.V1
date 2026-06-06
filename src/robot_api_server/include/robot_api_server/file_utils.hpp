#pragma once

#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

namespace robot_api_server
{

std::string read_text_file(const std::filesystem::path & path);
std::string read_optional_text_file(const std::filesystem::path & path);
std::string read_binary_file(const std::filesystem::path & path);

void write_text_file(const std::filesystem::path & path, const std::string & text);
void write_binary_file(const std::filesystem::path & path, const std::vector<std::uint8_t> & data);
void write_binary_file(const std::filesystem::path & path, const std::string & data);
void write_pgm_file(
  const std::filesystem::path & path,
  std::uint32_t width,
  std::uint32_t height,
  const std::vector<std::uint8_t> & pixels);

bool parse_yaml_key_value(const std::string & line, std::string & key, std::string & value);
std::string yaml_with_image_file(const std::string & yaml_text, const std::string & image_file);
void copy_file_if_exists(const std::filesystem::path & source, const std::filesystem::path & target);
void copy_yaml_with_image_if_exists(
  const std::filesystem::path & source,
  const std::filesystem::path & target,
  const std::string & image_file);

}  // namespace robot_api_server
