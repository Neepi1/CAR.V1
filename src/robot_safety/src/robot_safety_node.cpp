#include <algorithm>
#include <chrono>
#include <cctype>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"

namespace
{

enum class SafetyState
{
  OK,
  ESTOP_ACTIVE,
  LOCALIZATION_INVALID,
  DOCKED_CONTACT_BLOCK,
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
    case SafetyState::DOCKED_CONTACT_BLOCK:
      return "DOCKED_CONTACT_BLOCK";
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

std::string lower_copy(std::string text)
{
  std::transform(text.begin(), text.end(), text.begin(), [](const unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  return text;
}

bool starts_with(const std::string & value, const std::string & prefix)
{
  return value.rfind(prefix, 0) == 0;
}

bool voltage_in_contact_range(const float voltage, const double min_v, const double max_v)
{
  const double v = static_cast<double>(voltage);
  return std::isfinite(v) && v >= min_v && v <= max_v;
}

double normalized_soc_percent(const float percentage)
{
  if (!std::isfinite(percentage)) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  const double raw = static_cast<double>(percentage);
  return std::clamp(raw <= 1.0 ? raw * 100.0 : raw, 0.0, 100.0);
}

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
    docking_cmd_vel_in_topic_ = declare_parameter<std::string>("docking_cmd_vel_in_topic", "/cmd_vel_docking");
    cmd_vel_out_topic_ = declare_parameter<std::string>("cmd_vel_out_topic", "/cmd_vel_safe");
    estop_topic_ = declare_parameter<std::string>("estop_topic", "/safety/estop");
    localization_ok_topic_ = declare_parameter<std::string>("localization_ok_topic", "/localization/health");
    require_localization_health_ = declare_parameter<bool>("require_localization_health", false);
    block_normal_motion_when_docked_ = declare_parameter<bool>("block_normal_motion_when_docked", true);
    allow_docking_cmd_when_docked_ = declare_parameter<bool>("allow_docking_cmd_when_docked", true);
    enable_bms_contact_guard_ = declare_parameter<bool>("enable_bms_contact_guard", true);
    enable_docking_status_guard_ = declare_parameter<bool>("enable_docking_status_guard", true);
    enable_docked_latch_file_guard_ = declare_parameter<bool>("enable_docked_latch_file_guard", true);
    docked_status_prefixes_ =
      declare_parameter<std::vector<std::string>>("docked_status_prefixes", {"docked", "charging"});
    battery_state_topic_ = declare_parameter<std::string>("battery_state_topic", "/battery_state");
    docking_status_topic_ = declare_parameter<std::string>("docking_status_topic", "/docking/status");
    docking_contact_latch_file_ = declare_parameter<std::string>(
      "docking_contact_latch_file",
      "/workspaces/njrh-v3/workspace1/maps_release/docking_contact_latch.json");
    dock_contact_max_age_sec_ =
      std::max(0.1, declare_parameter<double>("dock_contact_max_age_sec", 3.0));
    charging_current_min_a_ =
      std::max(0.0, declare_parameter<double>("charging_current_min_a", 0.10));
    charging_contact_voltage_min_v_ =
      std::max(0.0, declare_parameter<double>("charging_contact_voltage_min_v", 40.0));
    charging_contact_voltage_max_v_ = std::max(
      charging_contact_voltage_min_v_,
      declare_parameter<double>("charging_contact_voltage_max_v", 1000.0));
    charging_full_soc_threshold_pct_ =
      std::clamp(declare_parameter<double>("charging_full_soc_threshold_pct", 99.0), 0.0, 100.0);
    charging_full_soc_voltage_contact_enable_ =
      declare_parameter<bool>("charging_full_soc_voltage_contact_enable", true);
    enable_docking_cmd_priority_ = declare_parameter<bool>("enable_docking_cmd_priority", true);
    docking_cmd_priority_timeout_sec_ =
      std::max(0.05, declare_parameter<double>("docking_cmd_priority_timeout_sec", 0.25));
    status_topic_ = declare_parameter<std::string>("status_topic", "/safety/status");
    motion_allowed_topic_ = declare_parameter<std::string>("motion_allowed_topic", "/safety/motion_allowed");
    const bool publish_zero_on_startup = declare_parameter<bool>("publish_zero_on_startup", true);

    localization_ok_ = !require_localization_health_;
    last_cmd_time_ = now();

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_out_topic_, rclcpp::QoS(10));
    const auto state_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    status_pub_ = create_publisher<std_msgs::msg::String>(status_topic_, state_qos);
    motion_allowed_pub_ = create_publisher<std_msgs::msg::Bool>(motion_allowed_topic_, state_qos);

    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      cmd_vel_in_topic_,
      rclcpp::QoS(10),
      std::bind(&RobotSafetyNode::on_normal_cmd, this, std::placeholders::_1));
    if (!docking_cmd_vel_in_topic_.empty()) {
      docking_cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        docking_cmd_vel_in_topic_,
        rclcpp::QoS(10),
        std::bind(&RobotSafetyNode::on_docking_cmd, this, std::placeholders::_1));
    }
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
    if (!battery_state_topic_.empty()) {
      battery_sub_ = create_subscription<sensor_msgs::msg::BatteryState>(
        battery_state_topic_,
        rclcpp::QoS(10),
        std::bind(&RobotSafetyNode::on_battery_state, this, std::placeholders::_1));
    }
    if (!docking_status_topic_.empty()) {
      docking_status_sub_ = create_subscription<std_msgs::msg::String>(
        docking_status_topic_,
        state_qos,
        std::bind(&RobotSafetyNode::on_docking_status, this, std::placeholders::_1));
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
  SafetySnapshot current_snapshot(const bool docking_command_allowed_context = false) const
  {
    if (estop_active_) {
      return {SafetyState::ESTOP_ACTIVE, false};
    }
    if (require_localization_health_ && !localization_ok_) {
      return {SafetyState::LOCALIZATION_INVALID, false};
    }
    if (dock_contact_active() &&
      !(docking_command_allowed_context && allow_docking_cmd_when_docked_))
    {
      return {SafetyState::DOCKED_CONTACT_BLOCK, false};
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

  bool docking_command_fresh() const
  {
    if (!enable_docking_cmd_priority_ || last_docking_cmd_time_.nanoseconds() <= 0) {
      return false;
    }
    return (now() - last_docking_cmd_time_).seconds() <= docking_cmd_priority_timeout_sec_;
  }

  bool fresh_docking_command_active() const
  {
    return have_last_docking_cmd_ && last_cmd_was_docking_ && docking_command_fresh();
  }

  void publish_checked_command(const geometry_msgs::msg::Twist & cmd)
  {
    publish_checked_command(cmd, false);
  }

  void publish_checked_command(const geometry_msgs::msg::Twist & cmd, const bool docking_command)
  {
    last_cmd_time_ = now();
    last_cmd_was_docking_ = docking_command;
    const auto snapshot = current_snapshot(docking_command);
    publish_command(snapshot.motion_allowed ? cmd : geometry_msgs::msg::Twist{}, snapshot);
  }

  void on_normal_cmd(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    last_normal_cmd_time_ = now();
    if (docking_command_fresh()) {
      return;
    }
    publish_checked_command(*msg);
  }

  void on_docking_cmd(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    last_docking_cmd_ = *msg;
    have_last_docking_cmd_ = true;
    last_docking_cmd_time_ = now();
    publish_checked_command(*msg, true);
  }

  void on_estop(const std_msgs::msg::Bool::SharedPtr msg)
  {
    estop_active_ = msg->data;
    const auto snapshot = current_snapshot(fresh_docking_command_active());
    if (snapshot.motion_allowed) {
      publish_snapshot(snapshot);
    } else {
      publish_command(geometry_msgs::msg::Twist{}, snapshot);
    }
  }

  void on_localization_ok(const std_msgs::msg::Bool::SharedPtr msg)
  {
    localization_ok_ = msg->data;
    const auto snapshot = current_snapshot(fresh_docking_command_active());
    if (snapshot.motion_allowed) {
      publish_snapshot(snapshot);
    } else {
      publish_command(geometry_msgs::msg::Twist{}, snapshot);
    }
  }

  void on_timer()
  {
    const bool docking_context = fresh_docking_command_active();
    const auto snapshot = current_snapshot(docking_context);
    if (snapshot.motion_allowed) {
      if (docking_context) {
        publish_command(last_docking_cmd_, snapshot);
      } else {
        publish_snapshot(snapshot);
      }
    } else {
      publish_command(geometry_msgs::msg::Twist{}, snapshot);
    }
  }

  bool battery_indicates_charging_contact(const sensor_msgs::msg::BatteryState & msg) const
  {
    if (msg.power_supply_status == sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_CHARGING ||
      msg.power_supply_status == sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_FULL)
    {
      return true;
    }
    if (std::isfinite(msg.current) && static_cast<double>(msg.current) > charging_current_min_a_) {
      return true;
    }
    if (msg.present &&
      voltage_in_contact_range(msg.voltage, charging_contact_voltage_min_v_, charging_contact_voltage_max_v_))
    {
      return true;
    }
    const double soc = normalized_soc_percent(msg.percentage);
    return charging_full_soc_voltage_contact_enable_ && msg.present && std::isfinite(soc) &&
      soc >= charging_full_soc_threshold_pct_ &&
      voltage_in_contact_range(msg.voltage, charging_contact_voltage_min_v_, charging_contact_voltage_max_v_);
  }

  void on_battery_state(const sensor_msgs::msg::BatteryState::SharedPtr msg)
  {
    battery_contact_active_ = battery_indicates_charging_contact(*msg);
    last_battery_time_ = now();
  }

  void on_docking_status(const std_msgs::msg::String::SharedPtr msg)
  {
    latest_docking_status_ = lower_copy(msg->data);
    last_docking_status_time_ = now();
  }

  bool dock_contact_latch_is_docked() const
  {
    if (docking_contact_latch_file_.empty()) {
      return false;
    }
    if (last_latch_read_time_.nanoseconds() > 0 &&
      (now() - last_latch_read_time_).seconds() < 1.0)
    {
      return cached_latch_docked_;
    }
    last_latch_read_time_ = now();
    std::ifstream file(docking_contact_latch_file_);
    if (!file) {
      cached_latch_docked_ = false;
      return false;
    }
    std::ostringstream data;
    data << file.rdbuf();
    const auto text = data.str();
    cached_latch_docked_ = text.find("\"latched_docked\": true") != std::string::npos ||
      text.find("\"latched_docked\":true") != std::string::npos ||
      text.find("\"docked\": true") != std::string::npos ||
      text.find("\"docked\":true") != std::string::npos;
    return cached_latch_docked_;
  }

  bool dock_contact_active() const
  {
    if (!block_normal_motion_when_docked_) {
      return false;
    }
    if (enable_bms_contact_guard_ &&
      last_battery_time_.nanoseconds() > 0 &&
      (now() - last_battery_time_).seconds() <= dock_contact_max_age_sec_ &&
      battery_contact_active_)
    {
      return true;
    }
    if (enable_docking_status_guard_) {
      for (const auto & prefix : docked_status_prefixes_) {
        if (!prefix.empty() && starts_with(latest_docking_status_, lower_copy(prefix))) {
          return true;
        }
      }
    }
    return enable_docked_latch_file_guard_ && dock_contact_latch_is_docked();
  }

  bool estop_active_{false};
  bool localization_ok_{true};
  bool require_localization_health_{false};
  bool block_normal_motion_when_docked_{true};
  bool allow_docking_cmd_when_docked_{true};
  bool enable_bms_contact_guard_{true};
  bool enable_docking_status_guard_{true};
  bool enable_docked_latch_file_guard_{true};
  bool has_last_snapshot_{false};
  double watchdog_timeout_sec_{1.0};
  double docking_cmd_priority_timeout_sec_{0.25};
  double dock_contact_max_age_sec_{3.0};
  double charging_current_min_a_{0.10};
  double charging_contact_voltage_min_v_{40.0};
  double charging_contact_voltage_max_v_{1000.0};
  double charging_full_soc_threshold_pct_{99.0};
  bool charging_full_soc_voltage_contact_enable_{true};
  bool enable_docking_cmd_priority_{true};
  std::string cmd_vel_in_topic_;
  std::string docking_cmd_vel_in_topic_;
  std::string cmd_vel_out_topic_;
  std::string estop_topic_;
  std::string localization_ok_topic_;
  std::string battery_state_topic_;
  std::string docking_status_topic_;
  std::string docking_contact_latch_file_;
  std::string status_topic_;
  std::string motion_allowed_topic_;
  std::vector<std::string> docked_status_prefixes_{"docked", "charging"};
  rclcpp::Time last_cmd_time_;
  rclcpp::Time last_normal_cmd_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_docking_cmd_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_battery_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_docking_status_time_{0, 0, RCL_ROS_TIME};
  bool last_cmd_was_docking_{false};
  bool have_last_docking_cmd_{false};
  bool battery_contact_active_{false};
  mutable bool cached_latch_docked_{false};
  std::string latest_docking_status_;
  mutable rclcpp::Time last_latch_read_time_{0, 0, RCL_ROS_TIME};
  SafetySnapshot last_snapshot_;
  geometry_msgs::msg::Twist last_docking_cmd_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr motion_allowed_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr docking_cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr localization_sub_;
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr docking_status_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RobotSafetyNode>());
  rclcpp::shutdown();
  return 0;
}
