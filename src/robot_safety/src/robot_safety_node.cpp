#include <algorithm>
#include <chrono>
#include <memory>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"

namespace
{

enum class SafetyState
{
  OK,
  ESTOP_ACTIVE,
  LOCALIZATION_INVALID,
  COMMAND_STALE,
};

const char * to_string(const SafetyState state)
{
  switch (state) {
    case SafetyState::OK:
      return "OK";
    case SafetyState::ESTOP_ACTIVE:
      return "ESTOP_ACTIVE";
    case SafetyState::LOCALIZATION_INVALID:
      return "LOCALIZATION_INVALID";
    case SafetyState::COMMAND_STALE:
      return "COMMAND_STALE";
  }
  return "COMMAND_STALE";
}

struct SafetySnapshot
{
  SafetyState state{SafetyState::COMMAND_STALE};
  bool motion_allowed{false};

  bool operator==(const SafetySnapshot & other) const
  {
    return state == other.state && motion_allowed == other.motion_allowed;
  }
};

}  // namespace

class RobotSafetyNode : public rclcpp::Node
{
public:
  RobotSafetyNode()
  : Node("robot_safety")
  {
    declare_parameter<bool>("mock_mode", false);
    watchdog_timeout_sec_ = declare_parameter<double>("watchdog_timeout_sec", 1.0);
    const double publish_rate_hz = std::max(1.0, declare_parameter<double>("publish_rate_hz", 10.0));
    cmd_vel_in_topic_ = declare_parameter<std::string>("cmd_vel_in_topic", "/cmd_vel_collision_checked");
    cmd_vel_out_topic_ = declare_parameter<std::string>("cmd_vel_out_topic", "/cmd_vel_safe");
    estop_topic_ = declare_parameter<std::string>("estop_topic", "/safety/estop");
    localization_ok_topic_ = declare_parameter<std::string>("localization_ok_topic", "/localization/health");
    require_localization_health_ = declare_parameter<bool>("require_localization_health", false);
    status_topic_ = declare_parameter<std::string>("status_topic", "/safety/status");
    motion_allowed_topic_ = declare_parameter<std::string>("motion_allowed_topic", "/safety/motion_allowed");
    const bool publish_zero_on_startup = declare_parameter<bool>("publish_zero_on_startup", true);

    localization_ok_ = !require_localization_health_;
    last_cmd_time_ = now();

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_out_topic_, rclcpp::QoS(10));
    status_pub_ = create_publisher<std_msgs::msg::String>(status_topic_, rclcpp::QoS(10));
    motion_allowed_pub_ = create_publisher<std_msgs::msg::Bool>(motion_allowed_topic_, rclcpp::QoS(10));

    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      cmd_vel_in_topic_,
      rclcpp::QoS(10),
      std::bind(&RobotSafetyNode::on_cmd, this, std::placeholders::_1));
    estop_sub_ = create_subscription<std_msgs::msg::Bool>(
      estop_topic_,
      rclcpp::QoS(10),
      std::bind(&RobotSafetyNode::on_estop, this, std::placeholders::_1));
    if (!localization_ok_topic_.empty()) {
      localization_sub_ = create_subscription<std_msgs::msg::Bool>(
        localization_ok_topic_,
        rclcpp::QoS(10),
        std::bind(&RobotSafetyNode::on_localization_ok, this, std::placeholders::_1));
    }

    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / publish_rate_hz)),
      std::bind(&RobotSafetyNode::on_timer, this));

    if (publish_zero_on_startup) {
      publish_command(geometry_msgs::msg::Twist{}, SafetySnapshot{SafetyState::COMMAND_STALE, false});
    }
  }

private:
  SafetySnapshot current_snapshot() const
  {
    if (estop_active_) {
      return {SafetyState::ESTOP_ACTIVE, false};
    }
    if (require_localization_health_ && !localization_ok_) {
      return {SafetyState::LOCALIZATION_INVALID, false};
    }
    const double age = (now() - last_cmd_time_).seconds();
    if (age > watchdog_timeout_sec_) {
      return {SafetyState::COMMAND_STALE, false};
    }
    return {SafetyState::OK, true};
  }

  void publish_snapshot(const SafetySnapshot & snapshot)
  {
    if (has_last_snapshot_ && last_snapshot_ == snapshot) {
      return;
    }
    std_msgs::msg::String status;
    status.data = to_string(snapshot.state);
    std_msgs::msg::Bool motion_allowed;
    motion_allowed.data = snapshot.motion_allowed;
    status_pub_->publish(status);
    motion_allowed_pub_->publish(motion_allowed);
    last_snapshot_ = snapshot;
    has_last_snapshot_ = true;
  }

  void publish_command(const geometry_msgs::msg::Twist & cmd, const SafetySnapshot & snapshot)
  {
    cmd_pub_->publish(cmd);
    publish_snapshot(snapshot);
  }

  void on_cmd(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    last_cmd_time_ = now();
    const auto snapshot = current_snapshot();
    publish_command(snapshot.motion_allowed ? *msg : geometry_msgs::msg::Twist{}, snapshot);
  }

  void on_estop(const std_msgs::msg::Bool::SharedPtr msg)
  {
    estop_active_ = msg->data;
    const auto snapshot = current_snapshot();
    if (snapshot.motion_allowed) {
      publish_snapshot(snapshot);
    } else {
      publish_command(geometry_msgs::msg::Twist{}, snapshot);
    }
  }

  void on_localization_ok(const std_msgs::msg::Bool::SharedPtr msg)
  {
    localization_ok_ = msg->data;
    const auto snapshot = current_snapshot();
    if (snapshot.motion_allowed) {
      publish_snapshot(snapshot);
    } else {
      publish_command(geometry_msgs::msg::Twist{}, snapshot);
    }
  }

  void on_timer()
  {
    const auto snapshot = current_snapshot();
    if (snapshot.motion_allowed) {
      publish_snapshot(snapshot);
    } else {
      publish_command(geometry_msgs::msg::Twist{}, snapshot);
    }
  }

  bool estop_active_{false};
  bool localization_ok_{true};
  bool require_localization_health_{false};
  bool has_last_snapshot_{false};
  double watchdog_timeout_sec_{1.0};
  std::string cmd_vel_in_topic_;
  std::string cmd_vel_out_topic_;
  std::string estop_topic_;
  std::string localization_ok_topic_;
  std::string status_topic_;
  std::string motion_allowed_topic_;
  rclcpp::Time last_cmd_time_;
  SafetySnapshot last_snapshot_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr motion_allowed_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr localization_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RobotSafetyNode>());
  rclcpp::shutdown();
  return 0;
}
