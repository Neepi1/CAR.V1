#include "robot_api_server/features/localization/amcl_runtime_status.hpp"

#include <algorithm>
#include <cctype>
#include <exception>
#include <fstream>
#include <map>
#include <string>
#include <utility>

namespace robot_api_server::features::localization
{

namespace
{

std::string trim_copy(std::string value)
{
  const auto first = std::find_if_not(
    value.begin(), value.end(), [](const unsigned char character) {
      return std::isspace(character) != 0;
    });
  const auto last = std::find_if_not(
    value.rbegin(), value.rend(), [](const unsigned char character) {
      return std::isspace(character) != 0;
    }).base();
  if (first >= last) {
    return "";
  }
  return std::string(first, last);
}

std::string unquote_env_value(std::string value)
{
  value = trim_copy(std::move(value));
  if (value.size() >= 2U && value.front() == '"' && value.back() == '"') {
    value = value.substr(1U, value.size() - 2U);
  }
  std::string output;
  output.reserve(value.size());
  bool escaped = false;
  for (const char character : value) {
    if (escaped) {
      output.push_back(character);
      escaped = false;
    } else if (character == '\\') {
      escaped = true;
    } else {
      output.push_back(character);
    }
  }
  if (escaped) {
    output.push_back('\\');
  }
  return output;
}

}  // namespace

AmclRuntimeStatus read_amcl_runtime_status(
  const std::filesystem::path & status_file,
  const double now_sec,
  const double ttl_sec)
{
  AmclRuntimeStatus status;
  if (status_file.empty()) {
    return status;
  }
  std::ifstream input(status_file);
  if (!input.good()) {
    return status;
  }

  std::map<std::string, std::string> fields;
  std::string line;
  while (std::getline(input, line)) {
    line = trim_copy(std::move(line));
    if (line.empty() || line.front() == '#') {
      continue;
    }
    const auto equal_pos = line.find('=');
    if (equal_pos == std::string::npos) {
      continue;
    }
    const auto key = trim_copy(line.substr(0U, equal_pos));
    if (!key.empty()) {
      fields[key] = unquote_env_value(line.substr(equal_pos + 1U));
    }
  }

  const auto string_field = [&](const std::string & key) -> std::string {
      const auto iterator = fields.find(key);
      return iterator == fields.end() ? std::string() : iterator->second;
    };
  const auto bool_field = [&](const std::string & key) -> bool {
      const auto value = string_field(key);
      return value == "true" || value == "True" || value == "1";
    };
  const auto int_field = [&](const std::string & key) -> int {
      const auto value = string_field(key);
      if (value.empty()) {
        return 0;
      }
      try {
        return std::stoi(value);
      } catch (const std::exception &) {
        return 0;
      }
    };
  const auto double_field =
    [&](const std::string & key, const double fallback = -1.0) -> double {
      const auto value = string_field(key);
      if (value.empty()) {
        return fallback;
      }
      try {
        return std::stod(value);
      } catch (const std::exception &) {
        return fallback;
      }
    };

  status.available = true;
  status.mode = string_field("AMCL_MODE");
  status.state = string_field("AMCL_STATE");
  status.start_result = string_field("AMCL_START_RESULT");
  status.ready = bool_field("AMCL_READY");
  status.degraded = bool_field("AMCL_DEGRADED");
  status.degraded_reason = string_field("AMCL_FAILURE_REASON");
  status.process_alive = bool_field("AMCL_PID_ALIVE");
  status.scan_admission_alive = bool_field("SCAN_ADMISSION_ALIVE");
  status.pose_publisher_count = int_field("AMCL_POSE_PUBLISHER_COUNT");
  status.scan_admission_status_publisher_count =
    int_field("SCAN_ADMISSION_STATUS_PUBLISHER_COUNT");
  status.seed_succeeded = bool_field("AMCL_SEED_SUCCEEDED");
  status.seed_response_ok = bool_field("AMCL_SEED_RESPONSE_OK");
  status.nomotion_probe_used = bool_field("AMCL_NOMOTION_PROBE_USED");
  status.nomotion_pose_received = bool_field("AMCL_NOMOTION_POSE_RECEIVED");
  status.nomotion_pose_count = int_field("AMCL_NOMOTION_POSE_COUNT");
  status.nomotion_pose_header_age_ms =
    double_field("AMCL_NOMOTION_POSE_HEADER_AGE_MS");
  status.process_ready = bool_field("AMCL_PROCESS_READY");
  status.seeded = bool_field("AMCL_SEEDED") ||
    status.seed_succeeded || status.seed_response_ok;
  status.static_standby = bool_field("AMCL_STATIC_STANDBY");
  status.tracking_ready = bool_field("AMCL_TRACKING_READY");
  status.correction_ready = bool_field("AMCL_CORRECTION_READY");
  status.not_moving_no_update_ok = bool_field("AMCL_NOT_MOVING_NO_UPDATE_OK");
  status.stamp_sec = double_field("AMCL_STATUS_STAMP_SEC");
  if (status.stamp_sec > 0.0) {
    status.age_ms = std::max(0.0, (now_sec - status.stamp_sec) * 1000.0);
    status.stale = status.age_ms > ttl_sec * 1000.0;
  } else {
    status.age_ms = -1.0;
    status.stale = true;
  }
  status.stamp = string_field("TIMESTAMP");
  return status;
}

}  // namespace robot_api_server::features::localization
