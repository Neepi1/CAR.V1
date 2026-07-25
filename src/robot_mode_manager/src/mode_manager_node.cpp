#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "robot_interfaces/msg/operating_mode_state.hpp"
#include "robot_interfaces/srv/set_mode.hpp"
#include "robot_mode_manager/mode_lease_arbiter.hpp"

namespace robot_mode_manager
{

class ModeManagerNode : public rclcpp::Node
{
public:
  ModeManagerNode()
  : Node("robot_mode_manager")
  {
    const auto min_lease_duration_sec =
      declare_parameter<double>("min_lease_duration_sec", 0.20);
    const auto max_lease_duration_sec =
      declare_parameter<double>("max_lease_duration_sec", 30.0);
    const auto recovery_owner =
      declare_parameter<std::string>("recovery_owner", "robot_mission_manager");
    const auto heartbeat_period_sec =
      declare_parameter<double>("heartbeat_period_sec", 0.20);
    if (!std::isfinite(heartbeat_period_sec) || heartbeat_period_sec <= 0.0) {
      throw std::invalid_argument("heartbeat_period_sec must be finite and positive");
    }
    arbiter_ = std::make_unique<ModeLeaseArbiter>(
      min_lease_duration_sec, max_lease_duration_sec, recovery_owner);

    state_pub_ = create_publisher<robot_interfaces::msg::OperatingModeState>(
      "/robot_mode/state", rclcpp::QoS(1).reliable().transient_local());
    service_ = create_service<robot_interfaces::srv::SetMode>(
      "/robot_mode/set_mode",
      [this](
        const std::shared_ptr<robot_interfaces::srv::SetMode::Request> request,
        std::shared_ptr<robot_interfaces::srv::SetMode::Response> response)
      {
        handle_command(*request, *response);
      });
    const auto heartbeat_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(heartbeat_period_sec));
    timer_ = create_wall_timer(heartbeat_period, [this]() {publish_heartbeat();});
    publish_heartbeat();
  }

private:
  static double steady_now_sec()
  {
    return std::chrono::duration<double>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
  }

  static builtin_interfaces::msg::Duration make_duration(const double seconds)
  {
    builtin_interfaces::msg::Duration duration;
    if (!std::isfinite(seconds) || seconds <= 0.0) {
      return duration;
    }
    const auto whole_seconds = std::floor(seconds);
    const auto bounded_seconds = std::min(
      whole_seconds,
      static_cast<double>(std::numeric_limits<std::int32_t>::max()));
    duration.sec = static_cast<std::int32_t>(bounded_seconds);
    const auto fractional = seconds - bounded_seconds;
    duration.nanosec = static_cast<std::uint32_t>(
      std::min(999999999.0, std::floor(fractional * 1.0e9)));
    return duration;
  }

  robot_interfaces::msg::OperatingModeState make_state(
    const ModeSnapshot & snapshot) const
  {
    robot_interfaces::msg::OperatingModeState state;
    state.stamp = now();
    state.generation = snapshot.generation;
    state.mode = to_string(snapshot.mode);
    state.owner = snapshot.owner;
    state.mission_id = snapshot.mission_id;
    state.lease_id = snapshot.lease_id;
    state.lease_remaining = make_duration(snapshot.lease_remaining_sec);
    state.lease_active = snapshot.lease_active;
    state.transition_reason = snapshot.transition_reason;
    return state;
  }

  void handle_command(
    const robot_interfaces::srv::SetMode::Request & request,
    robot_interfaces::srv::SetMode::Response & response)
  {
    ModeCommand command;
    command.operation = static_cast<ModeOperation>(request.operation);
    command.mode = request.mode;
    command.owner = request.owner;
    command.mission_id = request.mission_id;
    command.lease_id = request.lease_id;
    command.lease_duration_sec =
      static_cast<double>(request.lease_duration.sec) +
      static_cast<double>(request.lease_duration.nanosec) * 1.0e-9;

    ModeDecision decision;
    {
      std::lock_guard<std::mutex> lock(arbiter_mutex_);
      decision = arbiter_->apply(command, steady_now_sec());
    }
    response.success = decision.accepted;
    response.result_code = static_cast<std::uint8_t>(decision.code);
    response.message = decision.message;
    response.state = make_state(decision.state);
    if (decision.accepted) {
      state_pub_->publish(response.state);
    }
  }

  void publish_heartbeat()
  {
    ModeSnapshot snapshot;
    {
      std::lock_guard<std::mutex> lock(arbiter_mutex_);
      const auto now_sec = steady_now_sec();
      (void)arbiter_->expire(now_sec);
      snapshot = arbiter_->snapshot(now_sec);
    }
    state_pub_->publish(make_state(snapshot));
  }

  std::unique_ptr<ModeLeaseArbiter> arbiter_;
  std::mutex arbiter_mutex_;
  rclcpp::Publisher<robot_interfaces::msg::OperatingModeState>::SharedPtr state_pub_;
  rclcpp::Service<robot_interfaces::srv::SetMode>::SharedPtr service_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace robot_mode_manager

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<robot_mode_manager::ModeManagerNode>());
  rclcpp::shutdown();
  return 0;
}
