#include "robot_api_server/file_utils.hpp"

#include <fstream>
#include <sstream>
#include <stdexcept>

#include "robot_api_server/http_common.hpp"

namespace robot_api_server
{

namespace fs = std::filesystem;

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

std::string read_optional_text_file(const fs::path & path)
{
  if (!fs::exists(path) || !fs::is_regular_file(path)) {
    return "";
  }
  return read_text_file(path);
}

std::string read_binary_file(const fs::path & path)
{
  std::ifstream file(path, std::ios::binary);
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

void write_binary_file(const fs::path & path, const std::vector<std::uint8_t> & data)
{
  fs::create_directories(path.parent_path());
  std::ofstream file(path, std::ios::binary);
  if (!file) {
    throw std::runtime_error("failed to open file for writing: " + path.string());
  }
  file.write(reinterpret_cast<const char *>(data.data()), static_cast<std::streamsize>(data.size()));
}

void write_binary_file(const fs::path & path, const std::string & data)
{
  fs::create_directories(path.parent_path());
  std::ofstream file(path, std::ios::binary);
  if (!file) {
    throw std::runtime_error("failed to open file for writing: " + path.string());
  }
  file.write(data.data(), static_cast<std::streamsize>(data.size()));
}

void write_pgm_file(
  const fs::path & path,
  const std::uint32_t width,
  const std::uint32_t height,
  const std::vector<std::uint8_t> & pixels)
{
  std::vector<std::uint8_t> payload;
  const auto header = "P5\n" + std::to_string(width) + " " + std::to_string(height) + "\n255\n";
  payload.insert(payload.end(), header.begin(), header.end());
  payload.insert(payload.end(), pixels.begin(), pixels.end());
  write_binary_file(path, payload);
}

bool parse_yaml_key_value(const std::string & line, std::string & key, std::string & value)
{
  const auto colon = line.find(':');
  if (colon == std::string::npos) {
    return false;
  }
  key = trim(line.substr(0, colon));
  value = trim(line.substr(colon + 1));
  return !key.empty();
}

std::string yaml_with_image_file(const std::string & yaml_text, const std::string & image_file)
{
  std::ostringstream out;
  std::istringstream input(yaml_text);
  std::string line;
  bool replaced = false;
  while (std::getline(input, line)) {
    std::string key;
    std::string value;
    if (!replaced && parse_yaml_key_value(line, key, value) && key == "image") {
      out << "image: " << image_file << "\n";
      replaced = true;
    } else {
      out << line << "\n";
    }
  }
  if (!replaced) {
    out << "image: " << image_file << "\n";
  }
  return out.str();
}

void copy_file_if_exists(const fs::path & source, const fs::path & target)
{
  if (!fs::exists(source) || !fs::is_regular_file(source)) {
    return;
  }
  fs::create_directories(target.parent_path());
  fs::copy_file(source, target, fs::copy_options::overwrite_existing);
}

void copy_yaml_with_image_if_exists(
  const fs::path & source,
  const fs::path & target,
  const std::string & image_file)
{
  if (!fs::exists(source) || !fs::is_regular_file(source)) {
    return;
  }
  write_text_file(target, yaml_with_image_file(read_text_file(source), image_file));
}

}  // namespace robot_api_server
