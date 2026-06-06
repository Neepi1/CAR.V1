#include "robot_api_server/runtime_map_lookup.hpp"

#include <algorithm>
#include <cctype>

namespace robot_api_server
{
namespace fs = std::filesystem;

bool safe_map_name(const std::string & name)
{
  if (name.empty() || name == "." || name.size() > 128U) {
    return false;
  }
  if (name.find("..") != std::string::npos || name.find('/') != std::string::npos ||
    name.find('\\') != std::string::npos)
  {
    return false;
  }
  return std::all_of(name.begin(), name.end(), [](const unsigned char c) {
    return std::isalnum(c) != 0 || c == '-' || c == '_' || c == '.';
  });
}

std::vector<fs::path> runtime_map_asset_paths(
  const fs::path & runtime_maps_dir,
  const std::string & map_name)
{
  return {
    runtime_maps_dir / (map_name + ".yaml"),
    runtime_maps_dir / (map_name + ".pgm"),
    runtime_maps_dir / (map_name + ".png"),
    runtime_maps_dir / (map_name + ".localizer.yaml"),
    runtime_maps_dir / (map_name + ".localizer.png"),
    runtime_maps_dir / (map_name + ".localizer.pgm"),
    runtime_maps_dir / (map_name + ".meta.json"),
    runtime_maps_dir / (map_name + ".metadata.json")
  };
}

std::optional<fs::path> newest_png_in_directory(const fs::path & directory)
{
  if (!fs::exists(directory) || !fs::is_directory(directory)) {
    return std::nullopt;
  }

  std::optional<fs::path> newest;
  fs::file_time_type newest_time{};
  for (const auto & entry : fs::directory_iterator(directory)) {
    if (!entry.is_regular_file() || entry.path().extension() != ".png") {
      continue;
    }
    const auto stem = entry.path().stem().string();
    if (stem.size() >= 10U && stem.substr(stem.size() - 10U) == ".localizer") {
      continue;
    }
    const auto write_time = entry.last_write_time();
    if (!newest || write_time > newest_time) {
      newest = entry.path();
      newest_time = write_time;
    }
  }
  return newest;
}

std::optional<fs::path> newest_floor_localizer_png(const fs::path & maps_root)
{
  if (!fs::exists(maps_root) || !fs::is_directory(maps_root)) {
    return std::nullopt;
  }

  std::optional<fs::path> newest;
  fs::file_time_type newest_time{};
  for (const auto & entry : fs::recursive_directory_iterator(maps_root)) {
    if (!entry.is_regular_file() || entry.path().filename() != "localizer_map.png") {
      continue;
    }
    const auto write_time = entry.last_write_time();
    if (!newest || write_time > newest_time) {
      newest = entry.path();
      newest_time = write_time;
    }
  }
  return newest;
}

std::optional<fs::path> resolve_mapping_2d_png(
  const HttpRequest & request,
  const fs::path & runtime_maps_dir,
  const fs::path & maps_root)
{
  const auto name_it = request.query.find("name");
  if (name_it != request.query.end()) {
    if (!safe_map_name(name_it->second)) {
      return std::nullopt;
    }
    const auto candidate = runtime_maps_dir / (name_it->second + ".png");
    if (fs::exists(candidate) && fs::is_regular_file(candidate)) {
      return candidate;
    }
    return std::nullopt;
  }

  const auto latest_runtime = newest_png_in_directory(runtime_maps_dir);
  if (latest_runtime) {
    return latest_runtime;
  }
  return newest_floor_localizer_png(maps_root);
}

}  // namespace robot_api_server
