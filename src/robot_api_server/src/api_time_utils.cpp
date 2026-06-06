#include "robot_api_server/api_time_utils.hpp"

#include <chrono>
#include <cctype>
#include <ctime>
#include <iomanip>
#include <sstream>

#include <unistd.h>

#include "robot_api_server/storage_models.hpp"

namespace robot_api_server
{

std::string utc_timestamp_compact()
{
  const auto now = std::chrono::system_clock::now();
  const std::time_t time = std::chrono::system_clock::to_time_t(now);
  std::tm tm{};
  gmtime_r(&time, &tm);
  std::ostringstream out;
  out << std::put_time(&tm, "%Y%m%dT%H%M%SZ");
  return out.str();
}

std::string utc_timestamp_iso8601()
{
  const auto now = std::chrono::system_clock::now();
  const std::time_t time = std::chrono::system_clock::to_time_t(now);
  std::tm tm{};
  gmtime_r(&time, &tm);
  std::ostringstream out;
  out << std::put_time(&tm, "%Y-%m-%dT%H:%M:%SZ");
  return out.str();
}

double wall_time_seconds()
{
  return std::chrono::duration<double>(
    std::chrono::system_clock::now().time_since_epoch()).count();
}

std::string generate_current_pose_id(const std::string & type, const std::string & name)
{
  std::string prefix;
  const std::string raw_prefix = type.empty() ? "pose" : type;
  for (const unsigned char c : raw_prefix) {
    if (std::isalnum(c) != 0 || c == '-' || c == '_' || c == '.' || c == ':') {
      prefix.push_back(static_cast<char>(c));
    } else {
      prefix.push_back('_');
    }
    if (prefix.size() >= 40U) {
      break;
    }
  }
  while (!prefix.empty() && (prefix.front() == '_' || prefix.front() == '.')) {
    prefix.erase(prefix.begin());
  }
  while (!prefix.empty() && (prefix.back() == '_' || prefix.back() == '.')) {
    prefix.pop_back();
  }
  if (prefix.empty() || !safe_pose_id(prefix)) {
    prefix = "pose";
  }
  const auto stamp = utc_timestamp_compact();
  const auto seed = prefix + "/" + name + "/" + stamp + "/" + std::to_string(::getpid());
  return prefix + "_" + stamp + "_" + fixed_hex(fnv1a64(seed), 8);
}

std::string generate_map_id(
  const std::string & building_id,
  const std::string & floor_id,
  const std::string & display_name)
{
  const auto stamp = utc_timestamp_compact();
  const auto seed = building_id + "/" + floor_id + "/" + display_name + "/" + stamp + "/" +
    std::to_string(::getpid());
  return "map_" + stamp + "_" + fixed_hex(fnv1a64(seed), 10);
}

}  // namespace robot_api_server
