#pragma once

#include <chrono>
#include <cstdint>
#include <string>

namespace robot_api_server
{

struct LocalizationResultSnapshot
{
  bool available{false};
  std::uint64_t seq{0U};
  std::string frame_id;
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
  double stamp_sec{0.0};
  double age_sec{-1.0};
  std::chrono::steady_clock::time_point received_at{};
};

std::string localization_result_success_detail(const LocalizationResultSnapshot & snapshot);

std::string localization_result_wait_failure_detail(
  const std::string & result_topic,
  double timeout_sec,
  const LocalizationResultSnapshot & snapshot);

std::string localization_result_recent_fallback_detail(
  const std::string & trigger_message,
  const std::string & result_topic,
  const LocalizationResultSnapshot & snapshot);

}  // namespace robot_api_server
