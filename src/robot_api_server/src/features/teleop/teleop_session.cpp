#include "robot_api_server/features/teleop/teleop_session.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>

#include "robot_api_server/infrastructure/http/http_common.hpp"

namespace robot_api_server::features::teleop
{

TeleopPayloadResult evaluate_teleop_payload(
  const std::string & payload,
  const TeleopCommandConfig & config)
{
  TeleopPayloadResult result;
  const auto message_type = json_string_value(payload, "type").value_or("cmd_vel");
  if (message_type == "stop") {
    result.kind = TeleopMessageKind::Stop;
    result.valid = true;
    result.response_json = "{\"ok\":true,\"type\":\"teleop_stopped\"}";
    return result;
  }
  if (message_type != "cmd_vel") {
    result.kind = TeleopMessageKind::Unsupported;
    result.response_json = error_json("unsupported teleop message type: " + message_type);
    return result;
  }
  result.kind = TeleopMessageKind::Velocity;
  const double raw_linear_x = json_number_value(payload, "linear_x")
                                .value_or(json_number_value(payload, "linearX")
                                            .value_or(json_number_value(payload, "vx")
                                                        .value_or(json_nested_number_value(
                                                            payload, "linear", "x").value_or(0.0))));
  const double raw_angular_z = json_number_value(payload, "angular_z")
                                .value_or(json_number_value(payload, "angularZ")
                                            .value_or(json_number_value(payload, "wz")
                                                        .value_or(json_nested_number_value(
                                                            payload, "angular", "z").value_or(0.0))));
  if (!std::isfinite(raw_linear_x) || !std::isfinite(raw_angular_z)) {
    result.response_json = error_json("linear_x/angular_z must be finite numbers");
    return result;
  }
  result.valid = true;
  const double min_linear_x = config.allow_reverse ? -config.max_linear_x_mps : 0.0;
  result.velocity.linear_x =
    std::clamp(raw_linear_x, min_linear_x, config.max_linear_x_mps);
  result.velocity.angular_z = std::clamp(
    raw_angular_z,
    -config.max_angular_z_radps,
    config.max_angular_z_radps);

  std::ostringstream out;
  out << "{\"ok\":true,\"type\":\"cmd_vel_ack\",";
  out << "\"linear_x\":" << result.velocity.linear_x << ",";
  out << "\"angular_z\":" << result.velocity.angular_z << ",";
  out << "\"allow_reverse\":" << (config.allow_reverse ? "true" : "false") << ",";
  out << "\"cmd_topic\":" << json_string(config.cmd_topic) << "}";
  result.response_json = out.str();
  return result;
}

void TeleopSessionState::session_started()
{
  std::lock_guard<std::mutex> lock(mutex_);
  ++session_count_;
}

bool TeleopSessionState::session_stopped()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (session_count_ > 0) {
    --session_count_;
  }
  if (session_count_ > 0) {
    return false;
  }
  latest_velocity_ = {};
  command_active_ = false;
  zero_sent_ = true;
  return true;
}

void TeleopSessionState::store_velocity(
  const TeleopVelocity & velocity,
  const Clock::time_point stored_at)
{
  std::lock_guard<std::mutex> lock(mutex_);
  latest_velocity_ = velocity;
  latest_command_at_ = stored_at;
  command_active_ = true;
  zero_sent_ = false;
}

void TeleopSessionState::clear_velocity()
{
  std::lock_guard<std::mutex> lock(mutex_);
  latest_velocity_ = {};
  command_active_ = false;
  zero_sent_ = true;
}

bool TeleopSessionState::session_active() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return session_count_ > 0;
}

bool TeleopSessionState::idle() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return session_count_ == 0 && !command_active_;
}

TeleopRepeatDecision TeleopSessionState::repeat_decision(
  const Clock::time_point now,
  const std::chrono::duration<double> watchdog_timeout)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (session_count_ == 0) {
    return {};
  }
  const bool command_fresh = command_active_ &&
    latest_command_at_.time_since_epoch().count() != 0 &&
    std::chrono::duration<double>(now - latest_command_at_) <= watchdog_timeout;
  if (command_fresh) {
    return {TeleopRepeatAction::PublishVelocity, latest_velocity_};
  }
  if (!zero_sent_) {
    latest_velocity_ = {};
    command_active_ = false;
    zero_sent_ = true;
    return {TeleopRepeatAction::PublishZero, {}};
  }
  return {};
}

}  // namespace robot_api_server::features::teleop
