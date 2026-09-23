#pragma once

#include <chrono>
#include <string>

namespace robot_nav_config
{
// Package-local diagnostics only; no control decision reads this memory.
// Called by the existing serial command callback; no timer/thread/ROS endpoint.
class NavliteLogGate
{
public:
  bool observe(const std::string & key, bool abnormal,
    std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now())
  {
    if (seen_ && key == key_ && (!abnormal || now - last_ < std::chrono::seconds(1))) {
      return false;
    }
    seen_ = true;
    key_ = key;
    last_ = now;
    return true;
  }
private:
  bool seen_{false};
  std::string key_;
  std::chrono::steady_clock::time_point last_{};
};
}  // namespace robot_nav_config
