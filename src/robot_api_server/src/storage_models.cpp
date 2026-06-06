#include "robot_api_server/storage_models.hpp"

#include <algorithm>
#include <cctype>
#include <iomanip>
#include <sstream>

namespace robot_api_server
{

bool safe_pose_id(const std::string & pose_id)
{
  if (pose_id.empty() || pose_id.size() > 128U || pose_id.find("..") != std::string::npos ||
    pose_id.find('/') != std::string::npos || pose_id.find('\\') != std::string::npos)
  {
    return false;
  }
  return std::all_of(pose_id.begin(), pose_id.end(), [](const unsigned char c) {
    return std::isalnum(c) != 0 || c == '-' || c == '_' || c == '.' || c == ':';
  });
}

bool safe_asset_id(const std::string & id)
{
  if (id.empty() || id.size() > 128U || id.find("..") != std::string::npos ||
    id.find('/') != std::string::npos || id.find('\\') != std::string::npos)
  {
    return false;
  }
  return std::all_of(id.begin(), id.end(), [](const unsigned char c) {
    return std::isalnum(c) != 0 || c == '-' || c == '_' || c == '.';
  });
}

bool valid_display_map_name(const std::string & name)
{
  const auto first = std::find_if_not(name.begin(), name.end(), [](unsigned char c) {
    return std::isspace(c) != 0;
  });
  const auto last = std::find_if_not(name.rbegin(), name.rend(), [](unsigned char c) {
    return std::isspace(c) != 0;
  }).base();
  const bool empty_after_trim = first >= last;
  return !empty_after_trim && name.size() <= 256U &&
    name.find('/') == std::string::npos && name.find('\\') == std::string::npos &&
    name.find("..") == std::string::npos;
}

std::string safe_file_stem_from_display_name(const std::string & display_name)
{
  std::string safe;
  bool previous_underscore = false;
  for (const unsigned char c : display_name) {
    char out = '\0';
    if (std::isalnum(c) != 0 || c == '-' || c == '.') {
      out = static_cast<char>(c);
    } else if (c == '_' || std::isspace(c) != 0) {
      out = '_';
    } else {
      out = '_';
    }
    if (out == '_') {
      if (previous_underscore) {
        continue;
      }
      previous_underscore = true;
    } else {
      previous_underscore = false;
    }
    safe.push_back(out);
    if (safe.size() >= 80U) {
      break;
    }
  }
  while (!safe.empty() && (safe.front() == '_' || safe.front() == '.')) {
    safe.erase(safe.begin());
  }
  while (!safe.empty() && (safe.back() == '_' || safe.back() == '.')) {
    safe.pop_back();
  }
  if (safe.empty()) {
    safe = "map";
  }
  return safe;
}

std::uint64_t fnv1a64(const std::string & value)
{
  std::uint64_t hash = 1469598103934665603ULL;
  for (const unsigned char c : value) {
    hash ^= static_cast<std::uint64_t>(c);
    hash *= 1099511628211ULL;
  }
  return hash;
}

std::string fixed_hex(const std::uint64_t value, const int width)
{
  std::ostringstream out;
  out << std::hex << std::nouppercase << std::setw(width) << std::setfill('0') << value;
  auto text = out.str();
  if (static_cast<int>(text.size()) > width) {
    text = text.substr(text.size() - static_cast<std::size_t>(width));
  }
  return text;
}

}  // namespace robot_api_server
