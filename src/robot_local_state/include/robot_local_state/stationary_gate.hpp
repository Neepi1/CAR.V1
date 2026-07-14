#pragma once

#include <algorithm>

namespace robot_local_state
{

class StationaryGate
{
public:
  StationaryGate(
    const bool use_odom_stationary,
    const bool use_cmd_vel_stationary,
    const double odom_timeout_sec,
    const double cmd_timeout_sec,
    const double stationary_required_sec,
    const double command_motion_holdoff_sec,
    const bool require_fresh_cmd_vel)
  : use_odom_stationary_(use_odom_stationary),
    use_cmd_vel_stationary_(use_cmd_vel_stationary),
    odom_timeout_sec_(std::max(0.0, odom_timeout_sec)),
    cmd_timeout_sec_(std::max(0.0, cmd_timeout_sec)),
    stationary_required_sec_(std::max(0.0, stationary_required_sec)),
    command_motion_holdoff_sec_(std::max(0.0, command_motion_holdoff_sec)),
    require_fresh_cmd_vel_(require_fresh_cmd_vel)
  {
  }

  void observe_odom(const double now_sec, const bool stationary)
  {
    has_odom_ = true;
    last_odom_sec_ = now_sec;
    last_odom_stationary_ = stationary;
  }

  void observe_command(const double now_sec, const bool stationary)
  {
    has_cmd_vel_ = true;
    last_cmd_vel_sec_ = now_sec;
    last_cmd_vel_stationary_ = stationary;
    if (!stationary) {
      has_nonstationary_command_ = true;
      last_nonstationary_command_sec_ = now_sec;
    }
  }

  bool confirmed(const double now_sec)
  {
    if (!candidate(now_sec)) {
      stationary_since_sec_ = -1.0;
      return false;
    }
    if (stationary_since_sec_ < 0.0) {
      stationary_since_sec_ = now_sec;
      return stationary_required_sec_ <= 0.0;
    }
    return (now_sec - stationary_since_sec_) >= stationary_required_sec_;
  }

private:
  static bool fresh(
    const bool valid,
    const double sample_sec,
    const double timeout_sec,
    const double now_sec)
  {
    return valid && (now_sec - sample_sec) <= timeout_sec;
  }

  bool candidate(const double now_sec) const
  {
    if (use_odom_stationary_ &&
      (!fresh(has_odom_, last_odom_sec_, odom_timeout_sec_, now_sec) ||
      !last_odom_stationary_))
    {
      return false;
    }

    if (use_cmd_vel_stationary_) {
      const bool cmd_fresh = fresh(
        has_cmd_vel_, last_cmd_vel_sec_, cmd_timeout_sec_, now_sec);
      if (cmd_fresh && !last_cmd_vel_stationary_) {
        return false;
      }
      if (!cmd_fresh && require_fresh_cmd_vel_) {
        return false;
      }
      if (has_nonstationary_command_ &&
        (now_sec - last_nonstationary_command_sec_) < command_motion_holdoff_sec_)
      {
        return false;
      }
    }
    return true;
  }

  bool use_odom_stationary_;
  bool use_cmd_vel_stationary_;
  double odom_timeout_sec_;
  double cmd_timeout_sec_;
  double stationary_required_sec_;
  double command_motion_holdoff_sec_;
  bool require_fresh_cmd_vel_;
  bool has_odom_{false};
  bool has_cmd_vel_{false};
  bool has_nonstationary_command_{false};
  bool last_odom_stationary_{false};
  bool last_cmd_vel_stationary_{false};
  double last_odom_sec_{0.0};
  double last_cmd_vel_sec_{0.0};
  double last_nonstationary_command_sec_{0.0};
  double stationary_since_sec_{-1.0};
};

}  // namespace robot_local_state
