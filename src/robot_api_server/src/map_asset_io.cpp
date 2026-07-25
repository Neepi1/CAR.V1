#include "robot_api_server/map_asset_io.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cctype>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>

#include <yaml-cpp/yaml.h>

#include "robot_api_server/http_common.hpp"

namespace robot_api_server
{
namespace fs = std::filesystem;
namespace
{

void append_u32_be(std::vector<std::uint8_t> & out, const std::uint32_t value)
{
  out.push_back(static_cast<std::uint8_t>((value >> 24U) & 0xFFU));
  out.push_back(static_cast<std::uint8_t>((value >> 16U) & 0xFFU));
  out.push_back(static_cast<std::uint8_t>((value >> 8U) & 0xFFU));
  out.push_back(static_cast<std::uint8_t>(value & 0xFFU));
}

std::uint32_t crc32_bytes(const std::uint8_t * data, const std::size_t size)
{
  std::uint32_t crc = 0xFFFFFFFFU;
  for (std::size_t i = 0; i < size; ++i) {
    crc ^= static_cast<std::uint32_t>(data[i]);
    for (int bit = 0; bit < 8; ++bit) {
      crc = (crc >> 1U) ^ (0xEDB88320U & static_cast<std::uint32_t>(-(static_cast<int>(crc & 1U))));
    }
  }
  return crc ^ 0xFFFFFFFFU;
}

std::uint32_t adler32_bytes(const std::vector<std::uint8_t> & data)
{
  std::uint32_t a = 1U;
  std::uint32_t b = 0U;
  for (const std::uint8_t byte : data) {
    a = (a + byte) % 65521U;
    b = (b + a) % 65521U;
  }
  return (b << 16U) | a;
}

void append_png_chunk(
  std::vector<std::uint8_t> & png,
  const char type[4],
  const std::vector<std::uint8_t> & data)
{
  append_u32_be(png, static_cast<std::uint32_t>(data.size()));
  const std::size_t crc_start = png.size();
  png.insert(png.end(), type, type + 4);
  png.insert(png.end(), data.begin(), data.end());
  append_u32_be(png, crc32_bytes(png.data() + crc_start, png.size() - crc_start));
}

std::vector<std::uint8_t> zlib_store_blocks(const std::vector<std::uint8_t> & raw)
{
  std::vector<std::uint8_t> out;
  out.reserve(raw.size() + raw.size() / 65535U * 5U + 16U);
  out.push_back(0x78U);
  out.push_back(0x01U);

  std::size_t offset = 0;
  while (offset < raw.size()) {
    const std::size_t block_size = std::min<std::size_t>(65535U, raw.size() - offset);
    const bool final_block = (offset + block_size) == raw.size();
    out.push_back(final_block ? 0x01U : 0x00U);
    const std::uint16_t len = static_cast<std::uint16_t>(block_size);
    const std::uint16_t nlen = static_cast<std::uint16_t>(~len);
    out.push_back(static_cast<std::uint8_t>(len & 0xFFU));
    out.push_back(static_cast<std::uint8_t>((len >> 8U) & 0xFFU));
    out.push_back(static_cast<std::uint8_t>(nlen & 0xFFU));
    out.push_back(static_cast<std::uint8_t>((nlen >> 8U) & 0xFFU));
    out.insert(out.end(), raw.begin() + static_cast<std::ptrdiff_t>(offset),
      raw.begin() + static_cast<std::ptrdiff_t>(offset + block_size));
    offset += block_size;
  }

  append_u32_be(out, adler32_bytes(raw));
  return out;
}

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

std::string yaml_scalar_unquote(std::string value)
{
  value = trim(value);
  const auto comment = value.find(" #");
  if (comment != std::string::npos) {
    value = trim(value.substr(0, comment));
  }
  if (value.size() >= 2U &&
    ((value.front() == '"' && value.back() == '"') || (value.front() == '\'' && value.back() == '\'')))
  {
    value = value.substr(1, value.size() - 2U);
  }
  return value;
}

std::optional<double> parse_yaml_double(std::string value)
{
  value = yaml_scalar_unquote(std::move(value));
  if (value.empty()) {
    return std::nullopt;
  }
  try {
    std::size_t idx = 0;
    const double parsed = std::stod(value, &idx);
    if (idx == 0U || !std::isfinite(parsed)) {
      return std::nullopt;
    }
    return parsed;
  } catch (...) {
    return std::nullopt;
  }
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

std::optional<std::array<double, 3>> parse_yaml_origin(std::string value)
{
  value = yaml_scalar_unquote(std::move(value));
  const auto open = value.find('[');
  const auto close = value.rfind(']');
  if (open == std::string::npos || close == std::string::npos || close <= open) {
    return std::nullopt;
  }

  std::array<double, 3> origin{0.0, 0.0, 0.0};
  std::string inner = value.substr(open + 1U, close - open - 1U);
  std::istringstream stream(inner);
  std::string token;
  std::size_t index = 0U;
  while (std::getline(stream, token, ',')) {
    if (index >= origin.size()) {
      return std::nullopt;
    }
    const auto parsed = parse_yaml_double(token);
    if (!parsed) {
      return std::nullopt;
    }
    origin[index++] = *parsed;
  }
  if (index != origin.size()) {
    return std::nullopt;
  }
  return origin;
}

fs::path resolve_yaml_image_path(const fs::path & yaml_path, std::string image_value)
{
  image_value = yaml_scalar_unquote(std::move(image_value));
  const fs::path image_path(image_value);
  if (image_path.is_absolute()) {
    return image_path;
  }
  return yaml_path.parent_path() / image_path;
}

}  // namespace

std::string encode_grayscale_png(
  const std::uint32_t width,
  const std::uint32_t height,
  const std::vector<std::uint8_t> & pixels)
{
  if (width == 0U || height == 0U || pixels.size() != static_cast<std::size_t>(width) * height) {
    return {};
  }

  std::vector<std::uint8_t> raw;
  raw.reserve(static_cast<std::size_t>(height) * (static_cast<std::size_t>(width) + 1U));
  for (std::uint32_t y = 0; y < height; ++y) {
    raw.push_back(0U);  // PNG filter type: None.
    raw.insert(raw.end(),
      pixels.begin() + static_cast<std::ptrdiff_t>(static_cast<std::size_t>(y) * width),
      pixels.begin() + static_cast<std::ptrdiff_t>((static_cast<std::size_t>(y) + 1U) * width));
  }

  std::vector<std::uint8_t> png = {0x89U, 'P', 'N', 'G', '\r', '\n', 0x1AU, '\n'};
  std::vector<std::uint8_t> ihdr;
  append_u32_be(ihdr, width);
  append_u32_be(ihdr, height);
  ihdr.push_back(8U);  // bit depth
  ihdr.push_back(0U);  // grayscale
  ihdr.push_back(0U);  // compression
  ihdr.push_back(0U);  // filter
  ihdr.push_back(0U);  // interlace
  append_png_chunk(png, "IHDR", ihdr);
  append_png_chunk(png, "IDAT", zlib_store_blocks(raw));
  append_png_chunk(png, "IEND", {});
  return std::string(reinterpret_cast<const char *>(png.data()), png.size());
}

std::optional<std::pair<std::uint32_t, std::uint32_t>> read_pgm_dimensions(const fs::path & pgm_path)
{
  std::ifstream file(pgm_path, std::ios::binary);
  if (!file) {
    return std::nullopt;
  }

  auto next_token = [&file]() -> std::optional<std::string> {
      std::string token;
      char c = '\0';
      while (file.get(c)) {
        if (std::isspace(static_cast<unsigned char>(c)) != 0) {
          continue;
        }
        if (c == '#') {
          std::string ignored;
          std::getline(file, ignored);
          continue;
        }
        token.push_back(c);
        break;
      }
      while (file.get(c)) {
        if (std::isspace(static_cast<unsigned char>(c)) != 0) {
          break;
        }
        if (c == '#') {
          std::string ignored;
          std::getline(file, ignored);
          break;
        }
        token.push_back(c);
      }
      if (token.empty()) {
        return std::nullopt;
      }
      return token;
    };

  const auto magic = next_token();
  const auto width = next_token();
  const auto height = next_token();
  if (!magic || (*magic != "P5" && *magic != "P2") || !width || !height) {
    return std::nullopt;
  }

  try {
    const auto parsed_width = static_cast<std::uint32_t>(std::stoul(*width));
    const auto parsed_height = static_cast<std::uint32_t>(std::stoul(*height));
    if (parsed_width == 0U || parsed_height == 0U) {
      return std::nullopt;
    }
    return std::make_pair(parsed_width, parsed_height);
  } catch (...) {
    return std::nullopt;
  }
}

std::optional<MapYamlInfo> read_nav_map_info(const fs::path & nav_map_yaml)
{
  if (!fs::exists(nav_map_yaml) || !fs::is_regular_file(nav_map_yaml)) {
    return std::nullopt;
  }

  const auto yaml_text = read_text_file(nav_map_yaml);
  std::optional<double> resolution;
  std::optional<std::array<double, 3>> origin;
  std::optional<fs::path> image_path;
  std::optional<std::uint32_t> width_from_yaml;
  std::optional<std::uint32_t> height_from_yaml;

  std::istringstream input(yaml_text);
  std::string line;
  while (std::getline(input, line)) {
    std::string key;
    std::string value;
    if (!parse_yaml_key_value(line, key, value)) {
      continue;
    }
    if (key == "resolution") {
      resolution = parse_yaml_double(value);
    } else if (key == "origin") {
      origin = parse_yaml_origin(value);
    } else if (key == "image") {
      image_path = resolve_yaml_image_path(nav_map_yaml, value);
    } else if (key == "width") {
      if (const auto parsed = parse_yaml_double(value)) {
        width_from_yaml = static_cast<std::uint32_t>(*parsed);
      }
    } else if (key == "height") {
      if (const auto parsed = parse_yaml_double(value)) {
        height_from_yaml = static_cast<std::uint32_t>(*parsed);
      }
    }
  }

  if (!resolution || !origin || *resolution <= 0.0) {
    return std::nullopt;
  }

  std::optional<std::pair<std::uint32_t, std::uint32_t>> dimensions;
  if (image_path) {
    dimensions = read_pgm_dimensions(*image_path);
  }

  MapYamlInfo info;
  info.resolution = *resolution;
  info.origin = *origin;
  if (dimensions) {
    info.width = dimensions->first;
    info.height = dimensions->second;
  } else if (width_from_yaml && height_from_yaml && *width_from_yaml > 0U && *height_from_yaml > 0U) {
    info.width = *width_from_yaml;
    info.height = *height_from_yaml;
  } else {
    return std::nullopt;
  }
  return info;
}

std::optional<MapYamlInfo> read_nav_map_info_exact(
  const fs::path & nav_map_yaml,
  const fs::path & exact_pgm_path)
{
  try {
    constexpr std::uintmax_t kMaximumNavMapYamlBytes = 1024U * 1024U;
    std::error_code size_error;
    const auto size = fs::file_size(nav_map_yaml, size_error);
    if (size_error || size > kMaximumNavMapYamlBytes ||
      !fs::is_regular_file(nav_map_yaml))
    {
      return std::nullopt;
    }

    constexpr std::uintmax_t kMaximumPgmHeaderBytes = 1024U * 1024U;
    std::ifstream pgm(exact_pgm_path, std::ios::binary);
    if (!pgm) {
      return std::nullopt;
    }
    std::string pgm_header(kMaximumPgmHeaderBytes, '\0');
    pgm.read(
      pgm_header.data(),
      static_cast<std::streamsize>(pgm_header.size()));
    pgm_header.resize(static_cast<std::size_t>(pgm.gcount()));
    return read_nav_map_info_exact_content(
      read_text_file(nav_map_yaml),
      pgm_header,
      exact_pgm_path.filename().string());
  } catch (const std::exception &) {
    return std::nullopt;
  }
}

std::optional<MapYamlInfo> read_nav_map_info_exact_content(
  const std::string & nav_map_yaml,
  const std::string & nav_map_pgm_header,
  const std::string & expected_pgm_filename)
{
  try {
    constexpr std::size_t kMaximumNavMapYamlBytes = 1024U * 1024U;
    constexpr std::size_t kMaximumPgmHeaderBytes = 1024U * 1024U;
    if (nav_map_yaml.size() > kMaximumNavMapYamlBytes ||
      nav_map_pgm_header.empty() ||
      nav_map_pgm_header.size() > kMaximumPgmHeaderBytes ||
      expected_pgm_filename.empty())
    {
      return std::nullopt;
    }

    const auto root = YAML::Load(nav_map_yaml);
    if (!root || !root.IsMap()) {
      return std::nullopt;
    }

    std::size_t image_count = 0U;
    std::size_t resolution_count = 0U;
    std::size_t origin_count = 0U;
    YAML::Node image;
    YAML::Node resolution;
    YAML::Node origin;
    for (const auto & entry : root) {
      if (!entry.first.IsScalar()) {
        return std::nullopt;
      }
      const auto key = entry.first.as<std::string>();
      if (key == "image") {
        ++image_count;
        image = entry.second;
      } else if (key == "resolution") {
        ++resolution_count;
        resolution = entry.second;
      } else if (key == "origin") {
        ++origin_count;
        origin = entry.second;
      }
    }
    if (image_count != 1U || resolution_count != 1U || origin_count != 1U ||
      !image.IsScalar() || !resolution.IsScalar() ||
      !origin.IsSequence() || origin.size() != 3U)
    {
      return std::nullopt;
    }

    const fs::path configured_image(image.as<std::string>());
    if (configured_image.empty() || configured_image.is_absolute() ||
      configured_image.has_parent_path() ||
      configured_image.filename() != configured_image ||
      configured_image == "." || configured_image == ".." ||
      configured_image.filename() != fs::path(expected_pgm_filename))
    {
      return std::nullopt;
    }

    MapYamlInfo info;
    info.resolution = resolution.as<double>();
    for (std::size_t index = 0U; index < info.origin.size(); ++index) {
      if (!origin[index].IsScalar()) {
        return std::nullopt;
      }
      info.origin[index] = origin[index].as<double>();
    }
    if (!std::isfinite(info.resolution) || info.resolution <= 0.0 ||
      !std::all_of(
        info.origin.begin(), info.origin.end(),
        [](const double value) {return std::isfinite(value);}))
    {
      return std::nullopt;
    }

    // Dimensions come from the descriptor-pinned PGM header returned by the
    // same identity snapshot. Nested YAML image keys cannot redirect the read.
    std::istringstream pgm(nav_map_pgm_header);
    auto next_pgm_token = [&pgm]() -> std::optional<std::string> {
        std::string token;
        char c = '\0';
        while (pgm.get(c)) {
          if (std::isspace(static_cast<unsigned char>(c)) != 0) {
            continue;
          }
          if (c == '#') {
            std::string ignored;
            std::getline(pgm, ignored);
            continue;
          }
          token.push_back(c);
          break;
        }
        while (pgm.get(c)) {
          if (std::isspace(static_cast<unsigned char>(c)) != 0) {
            break;
          }
          if (c == '#') {
            std::string ignored;
            std::getline(pgm, ignored);
            break;
          }
          token.push_back(c);
        }
        return token.empty() ? std::nullopt :
               std::optional<std::string>(std::move(token));
      };
    const auto magic = next_pgm_token();
    const auto width = next_pgm_token();
    const auto height = next_pgm_token();
    if (!magic || (*magic != "P5" && *magic != "P2") ||
      !width || !height)
    {
      return std::nullopt;
    }
    std::optional<std::pair<std::uint32_t, std::uint32_t>> dimensions;
    try {
      const auto parsed_width = std::stoull(*width);
      const auto parsed_height = std::stoull(*height);
      if (parsed_width == 0U || parsed_height == 0U ||
        parsed_width > std::numeric_limits<std::uint32_t>::max() ||
        parsed_height > std::numeric_limits<std::uint32_t>::max())
      {
        return std::nullopt;
      }
      dimensions = std::make_pair(
        static_cast<std::uint32_t>(parsed_width),
        static_cast<std::uint32_t>(parsed_height));
    } catch (const std::exception &) {
      return std::nullopt;
    }
    if (!dimensions) {
      return std::nullopt;
    }
    info.width = dimensions->first;
    info.height = dimensions->second;
    return info;
  } catch (const std::exception &) {
    return std::nullopt;
  }
}

std::string map_info_json(const std::optional<MapYamlInfo> & info)
{
  if (!info) {
    return "null";
  }
  std::ostringstream out;
  out << std::fixed << std::setprecision(6);
  out << "{\"width\":" << info->width
      << ",\"height\":" << info->height
      << ",\"resolution\":" << info->resolution
      << ",\"origin\":[" << info->origin[0] << "," << info->origin[1] << "," << info->origin[2]
      << "]}";
  return out.str();
}

}  // namespace robot_api_server
