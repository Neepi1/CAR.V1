#pragma once

#include <chrono>
#include <mutex>
#include <string>

namespace robot_api_server::features::teleop
{

struct TeleopVelocity
{
  double linear_x{0.0};
  double angular_z{0.0};
};

struct TeleopCommandConfig
{
  double max_linear_x_mps{0.0};
  double max_angular_z_radps{0.0};
  bool allow_reverse{false};
  std::string cmd_topic;
};

enum class TeleopMessageKind
{
  Velocity,
  Stop,
  Unsupported,
};

struct TeleopPayloadResult
{
  TeleopMessageKind kind{TeleopMessageKind::Unsupported};
  bool valid{false};
  TeleopVelocity velocity;
  std::string response_json;
};

TeleopPayloadResult evaluate_teleop_payload(
  const std::string & payload,
  const TeleopCommandConfig & config);

enum class TeleopRepeatAction
{
  None,
  PublishVelocity,
  PublishZero,
};

struct TeleopRepeatDecision
{
  TeleopRepeatAction action{TeleopRepeatAction::None};
  TeleopVelocity velocity;
};

class TeleopSessionState
{
public:
  using Clock = std::chrono::steady_clock;

  void session_started();
  bool session_stopped();
  void store_velocity(const TeleopVelocity & velocity, Clock::time_point stored_at);
  void clear_velocity();
  bool session_active() const;
  bool idle() const;
  TeleopRepeatDecision repeat_decision(
    Clock::time_point now,
    std::chrono::duration<double> watchdog_timeout);

private:
  mutable std::mutex mutex_;
  int session_count_{0};
  TeleopVelocity latest_velocity_;
  bool command_active_{false};
  bool zero_sent_{true};
  Clock::time_point latest_command_at_{};
};

}  // namespace robot_api_server::features::teleop
