#pragma once

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <string>
#include <yaml-cpp/yaml.h>

namespace robot_api_server::features::navigation
{
// Status is advisory, goal-correlated and short-lived. Missing/old status never
// suspends the execution budget. It is not a motion permit or admission gate.
class NavigationRecoveryWait
{
public:
  void bind(std::int64_t goal_stamp_ns)
  {
    goal_stamp_ = goal_stamp_ns;
    stamp_ = 0;
    phase_.clear();
  }
  bool receive(const std::string & data, std::int64_t ros_now_ns, double steady_now)
  {
    if (goal_stamp_ <= 0 || data.size() > 512) {return false;}
    try {
      const auto msg = YAML::Load(data);
      const auto stamp = msg["stamp_ns"].as<std::int64_t>();
      const auto phase = msg["phase"].as<std::string>();
      if (msg["version"].as<int>() != 1 || msg["goal_stamp_ns"].as<std::int64_t>() != goal_stamp_ ||
        stamp <= stamp_ || stamp < goal_stamp_ || stamp > ros_now_ns ||
        ros_now_ns - stamp > 1500000000LL) {return false;}
      if (phase != "tracking" && phase != "waiting" && phase != "recovering" &&
        phase != "checking_failure" && phase != "idle" && phase != "succeeded" &&
        phase != "failed") {return false;}
      stamp_ = stamp;
      received_ = steady_now;
      phase_ = phase;
      return true;
    } catch (const YAML::Exception &) {return false;}
  }
  std::string phase(std::int64_t ros_now_ns, double steady_now) const
  {
    if (stamp_ <= 0 || ros_now_ns < stamp_ || ros_now_ns - stamp_ > 1500000000LL ||
      steady_now < received_ || steady_now - received_ > 1.5) {return {};}
    return phase_;
  }
  static bool pauses_execution(const std::string & phase)
  {
    return phase == "waiting" || phase == "recovering";
  }
private:
  std::int64_t goal_stamp_{0}, stamp_{0};
  double received_{0};
  std::string phase_;
};

class NavigationExecutionBudget
{
public:
  using Clock = std::chrono::steady_clock;
  NavigationExecutionBudget(double seconds, Clock::time_point now)
  : remaining_(seconds), last_(now) {}
  bool expired(Clock::time_point now, bool recovery_wait)
  {
    const double elapsed = std::max(0.0, std::chrono::duration<double>(now - last_).count());
    // Only continuously observed short polling intervals can be credited.
    // A suspended API thread cannot retroactively forgive a long unknown gap.
    const double credit = recovery_wait && was_waiting_ ? std::min(elapsed, 0.5) : 0.0;
    remaining_ -= elapsed - credit;
    was_waiting_ = recovery_wait;
    last_ = now;
    return remaining_ <= 0.0;
  }
private:
  double remaining_;
  Clock::time_point last_;
  bool was_waiting_{false};
};
}  // namespace robot_api_server::features::navigation
