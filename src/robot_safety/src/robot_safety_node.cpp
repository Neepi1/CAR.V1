#include <algorithm>
#include <chrono>
#include <cctype>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "sensor_msgs/msg/imu.hpp"
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

enum class CommandSource
{
  NORMAL,
  API,
  DOCKING,
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

double normalize_angle(const double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

double yaw_from_quaternion(const geometry_msgs::msg::Quaternion & q)
{
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
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

bool twist_near_zero(const geometry_msgs::msg::Twist & cmd, const double epsilon)
{
  return std::abs(cmd.linear.x) <= epsilon &&
         std::abs(cmd.linear.y) <= epsilon &&
         std::abs(cmd.linear.z) <= epsilon &&
         std::abs(cmd.angular.x) <= epsilon &&
         std::abs(cmd.angular.y) <= epsilon &&
         std::abs(cmd.angular.z) <= epsilon;
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
    api_cmd_vel_in_topic_ = declare_parameter<std::string>("api_cmd_vel_in_topic", "/cmd_vel_api");
    docking_cmd_vel_in_topic_ = declare_parameter<std::string>("docking_cmd_vel_in_topic", "/cmd_vel_docking");
    cmd_vel_out_topic_ = declare_parameter<std::string>("cmd_vel_out_topic", "/cmd_vel");
    cmd_vel_mirror_topic_ = declare_parameter<std::string>("cmd_vel_mirror_topic", "/cmd_vel_safe");
    cmd_vel_qos_depth_ =
      std::max(1, static_cast<int>(declare_parameter<int>("cmd_vel_qos_depth", 1)));
    zero_cmd_priority_enabled_ = declare_parameter<bool>("zero_cmd_priority_enabled", true);
    zero_cmd_priority_epsilon_ =
      std::max(0.0, declare_parameter<double>("zero_cmd_priority_epsilon", 0.0001));
    zero_cmd_priority_burst_sec_ =
      std::max(0.0, declare_parameter<double>("zero_cmd_priority_burst_sec", 0.25));
    spin_to_drive_settle_enabled_ = declare_parameter<bool>("spin_to_drive_settle_enabled", true);
    spin_to_drive_odom_topic_ = declare_parameter<std::string>("spin_to_drive_odom_topic", "/wheel/odom");
    spin_to_drive_wz_threshold_radps_ =
      std::max(0.0, declare_parameter<double>("spin_to_drive_wz_threshold_radps", 0.02));
    spin_to_drive_stable_samples_ =
      std::max(1, static_cast<int>(declare_parameter<int>("spin_to_drive_stable_samples", 5)));
    spin_to_drive_timeout_sec_ =
      std::max(0.0, declare_parameter<double>("spin_to_drive_timeout_sec", 2.0));
    spin_to_drive_linear_epsilon_mps_ =
      std::max(0.0, declare_parameter<double>("spin_to_drive_linear_epsilon_mps", 0.03));
    spin_to_drive_odom_max_age_sec_ =
      std::max(0.02, declare_parameter<double>("spin_to_drive_odom_max_age_sec", 0.20));
    spin_to_drive_require_local_odom_stable_ =
      declare_parameter<bool>("spin_to_drive_require_local_odom_stable", false);
    spin_to_drive_local_odom_topic_ =
      declare_parameter<std::string>("spin_to_drive_local_odom_topic", "/local_state/odometry");
    spin_to_drive_local_wz_threshold_radps_ =
      std::max(0.0, declare_parameter<double>("spin_to_drive_local_wz_threshold_radps", 0.03));
    spin_to_drive_local_stable_samples_ =
      std::max(1, static_cast<int>(declare_parameter<int>("spin_to_drive_local_stable_samples", 5)));
    spin_to_drive_local_stable_duration_sec_ =
      std::max(0.0, declare_parameter<double>("spin_to_drive_local_stable_duration_sec", 0.30));
    spin_to_drive_local_yaw_delta_threshold_rad_ =
      std::max(0.0, declare_parameter<double>("spin_to_drive_local_yaw_delta_threshold_rad", 0.005));
    spin_to_drive_local_odom_max_age_sec_ =
      std::max(0.02, declare_parameter<double>("spin_to_drive_local_odom_max_age_sec", 0.20));
    spin_to_drive_require_imu_stable_ =
      declare_parameter<bool>("spin_to_drive_require_imu_stable", true);
    spin_to_drive_imu_topic_ =
      declare_parameter<std::string>("spin_to_drive_imu_topic", "/lidar_imu_bias_corrected");
    spin_to_drive_imu_wz_threshold_radps_ =
      std::max(0.0, declare_parameter<double>("spin_to_drive_imu_wz_threshold_radps", 0.035));
    spin_to_drive_imu_stable_duration_sec_ =
      std::max(0.0, declare_parameter<double>("spin_to_drive_imu_stable_duration_sec", 0.30));
    spin_to_drive_imu_max_age_sec_ =
      std::max(0.02, declare_parameter<double>("spin_to_drive_imu_max_age_sec", 0.10));
    mode_exit_guard_enabled_ = declare_parameter<bool>("mode_exit_guard_enabled", true);
    mode_controller_status_topic_ =
      declare_parameter<std::string>("mode_controller_status_topic", "/ranger_mini3_mode_controller/status");
    mode_exit_guard_probe_speed_mps_ =
      std::max(0.0, declare_parameter<double>("mode_exit_guard_probe_speed_mps", 0.06));
    mode_exit_guard_timeout_sec_ =
      std::max(0.0, declare_parameter<double>("mode_exit_guard_timeout_sec", 1.0));
    mode_exit_guard_status_max_age_sec_ =
      std::max(0.05, declare_parameter<double>("mode_exit_guard_status_max_age_sec", 0.5));
    final_cmd_lateral_deadband_mps_ =
      std::max(0.0, declare_parameter<double>("final_cmd_lateral_deadband_mps", 0.001));
    allow_api_lateral_cmd_ = declare_parameter<bool>("allow_api_lateral_cmd", false);
    api_lateral_max_mps_ =
      std::max(0.0, declare_parameter<double>("api_lateral_max_mps", 0.10));
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
    enable_api_cmd_priority_ = declare_parameter<bool>("enable_api_cmd_priority", true);
    api_cmd_priority_timeout_sec_ =
      std::max(0.05, declare_parameter<double>("api_cmd_priority_timeout_sec", 0.25));
    status_topic_ = declare_parameter<std::string>("status_topic", "/safety/status");
    motion_allowed_topic_ = declare_parameter<std::string>("motion_allowed_topic", "/safety/motion_allowed");
    const bool publish_zero_on_startup = declare_parameter<bool>("publish_zero_on_startup", true);

    localization_ok_ = !require_localization_health_;
    last_cmd_time_ = now();

    const auto command_qos = rclcpp::QoS(rclcpp::KeepLast(cmd_vel_qos_depth_)).reliable();
    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_out_topic_, command_qos);
    if (!cmd_vel_mirror_topic_.empty() && cmd_vel_mirror_topic_ != cmd_vel_out_topic_) {
      cmd_mirror_pub_ = create_publisher<geometry_msgs::msg::Twist>(
        cmd_vel_mirror_topic_, command_qos);
    }
    const auto state_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    status_pub_ = create_publisher<std_msgs::msg::String>(status_topic_, state_qos);
    motion_allowed_pub_ = create_publisher<std_msgs::msg::Bool>(motion_allowed_topic_, state_qos);

    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      cmd_vel_in_topic_,
      command_qos,
      std::bind(&RobotSafetyNode::on_normal_cmd, this, std::placeholders::_1));
    if (!api_cmd_vel_in_topic_.empty()) {
      api_cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        api_cmd_vel_in_topic_,
        command_qos,
        std::bind(&RobotSafetyNode::on_api_cmd, this, std::placeholders::_1));
    }
    if (!docking_cmd_vel_in_topic_.empty()) {
      docking_cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        docking_cmd_vel_in_topic_,
        command_qos,
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
    if (spin_to_drive_settle_enabled_ && !spin_to_drive_odom_topic_.empty()) {
      spin_to_drive_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        spin_to_drive_odom_topic_,
        rclcpp::QoS(20),
        std::bind(&RobotSafetyNode::on_spin_to_drive_odom, this, std::placeholders::_1));
    }
    if (spin_to_drive_settle_enabled_ && !spin_to_drive_local_odom_topic_.empty()) {
      spin_to_drive_local_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        spin_to_drive_local_odom_topic_,
        rclcpp::QoS(20),
        std::bind(&RobotSafetyNode::on_spin_to_drive_local_odom, this, std::placeholders::_1));
    }
    if (spin_to_drive_settle_enabled_ && !spin_to_drive_imu_topic_.empty()) {
      spin_to_drive_imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        spin_to_drive_imu_topic_,
        rclcpp::SensorDataQoS(),
        std::bind(&RobotSafetyNode::on_spin_to_drive_imu, this, std::placeholders::_1));
    }
    if (mode_exit_guard_enabled_ && !mode_controller_status_topic_.empty()) {
      mode_controller_status_sub_ = create_subscription<std_msgs::msg::String>(
        mode_controller_status_topic_,
        rclcpp::QoS(10),
        std::bind(&RobotSafetyNode::on_mode_controller_status, this, std::placeholders::_1));
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
    if (cmd_mirror_pub_) {
      cmd_mirror_pub_->publish(cmd);
    }
    publish_snapshot(snapshot);
  }

  bool docking_command_fresh() const
  {
    if (!enable_docking_cmd_priority_ || last_docking_cmd_time_.nanoseconds() <= 0) {
      return false;
    }
    return (now() - last_docking_cmd_time_).seconds() <= docking_cmd_priority_timeout_sec_;
  }

  bool api_command_fresh() const
  {
    if (!enable_api_cmd_priority_ || last_api_cmd_time_.nanoseconds() <= 0) {
      return false;
    }
    return (now() - last_api_cmd_time_).seconds() <= api_cmd_priority_timeout_sec_;
  }

  bool fresh_docking_command_active() const
  {
    return have_last_docking_cmd_ && last_cmd_was_docking_ && docking_command_fresh();
  }

  bool fresh_api_command_active() const
  {
    return have_last_api_cmd_ && last_cmd_was_api_ && api_command_fresh();
  }

  bool zero_cmd_priority_active() const
  {
    return zero_cmd_priority_enabled_ &&
           zero_cmd_priority_until_time_.nanoseconds() > 0 &&
           (now() - zero_cmd_priority_until_time_).seconds() < 0.0;
  }

  SafetySnapshot stop_priority_snapshot_for_source(const CommandSource source) const
  {
    return current_snapshot(source == CommandSource::DOCKING && fresh_docking_command_active());
  }

  bool source_may_override_active_priority_with_stop(const CommandSource source) const
  {
    if (source == CommandSource::DOCKING) {
      return true;
    }
    if (source == CommandSource::API) {
      return !docking_command_fresh();
    }
    return !docking_command_fresh() && !api_command_fresh();
  }

  void remember_stop_command_for_source(
    const CommandSource source,
    const geometry_msgs::msg::Twist & stop_cmd,
    const rclcpp::Time & now_time)
  {
    last_cmd_was_docking_ = source == CommandSource::DOCKING;
    last_cmd_was_api_ = source == CommandSource::API;
    if (source == CommandSource::API) {
      last_api_cmd_ = stop_cmd;
      have_last_api_cmd_ = true;
      last_api_cmd_time_ = now_time;
    } else if (source == CommandSource::DOCKING) {
      last_docking_cmd_ = stop_cmd;
      have_last_docking_cmd_ = true;
      last_docking_cmd_time_ = now_time;
    }
  }

  void publish_stop_priority_command(const CommandSource source)
  {
    const auto now_time = now();
    geometry_msgs::msg::Twist stop_cmd;
    remember_stop_command_for_source(source, stop_cmd, now_time);
    last_cmd_time_ = now_time;
    if (zero_cmd_priority_burst_sec_ > 0.0) {
      zero_cmd_priority_until_time_ =
        now_time + rclcpp::Duration::from_seconds(zero_cmd_priority_burst_sec_);
    } else {
      zero_cmd_priority_until_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    }
    publish_command(stop_cmd, stop_priority_snapshot_for_source(source));
  }

  bool handle_zero_priority_command(
    const geometry_msgs::msg::Twist & cmd,
    const CommandSource source)
  {
    if (!zero_cmd_priority_enabled_) {
      return false;
    }
    const bool stop_cmd = twist_near_zero(cmd, zero_cmd_priority_epsilon_);
    if (stop_cmd) {
      if (!source_may_override_active_priority_with_stop(source)) {
        return false;
      }
      publish_stop_priority_command(source);
      return true;
    }
    if (zero_cmd_priority_active()) {
      last_cmd_time_ = now();
      publish_command(geometry_msgs::msg::Twist{}, stop_priority_snapshot_for_source(source));
      return true;
    }
    return false;
  }

  void publish_checked_command(const geometry_msgs::msg::Twist & cmd)
  {
    publish_checked_command(cmd, CommandSource::NORMAL);
  }

  void publish_checked_command(const geometry_msgs::msg::Twist & cmd, const CommandSource source)
  {
    last_cmd_time_ = now();
    const bool docking_command = source == CommandSource::DOCKING;
    last_cmd_was_docking_ = docking_command;
    last_cmd_was_api_ = source == CommandSource::API;
    const auto snapshot = current_snapshot(docking_command);
    const auto gated_cmd = snapshot.motion_allowed ?
      prepare_checked_command(cmd, source) : geometry_msgs::msg::Twist{};
    publish_command(
      gated_cmd,
      snapshot);
  }

  void on_normal_cmd(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    last_normal_cmd_time_ = now();
    if (handle_zero_priority_command(*msg, CommandSource::NORMAL)) {
      return;
    }
    if (docking_command_fresh() || api_command_fresh()) {
      return;
    }
    publish_checked_command(*msg);
  }

  void on_api_cmd(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    if (handle_zero_priority_command(*msg, CommandSource::API)) {
      return;
    }
    last_api_cmd_ = *msg;
    have_last_api_cmd_ = true;
    last_api_cmd_time_ = now();
    if (docking_command_fresh()) {
      return;
    }
    publish_checked_command(*msg, CommandSource::API);
  }

  void on_docking_cmd(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    if (handle_zero_priority_command(*msg, CommandSource::DOCKING)) {
      return;
    }
    last_docking_cmd_ = *msg;
    have_last_docking_cmd_ = true;
    last_docking_cmd_time_ = now();
    publish_checked_command(*msg, CommandSource::DOCKING);
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
    const bool api_context = !docking_context && fresh_api_command_active();
    const auto snapshot = current_snapshot(docking_context);
    if (zero_cmd_priority_active()) {
      publish_command(geometry_msgs::msg::Twist{}, snapshot);
      return;
    }
    if (snapshot.motion_allowed) {
      if (docking_context) {
        publish_command(
          prepare_checked_command(last_docking_cmd_, CommandSource::DOCKING),
          snapshot);
      } else if (api_context) {
        publish_command(
          prepare_checked_command(last_api_cmd_, CommandSource::API),
          snapshot);
      } else if (actual_motion_mode_is_lateral()) {
        RCLCPP_WARN_THROTTLE(
          get_logger(),
          *get_clock(),
          2000,
          "mode_exit_guard releasing stale lateral motion_mode=%d with exact zero dual Ackermann command",
          actual_motion_mode_code_);
        publish_command(geometry_msgs::msg::Twist{}, snapshot);
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

  void on_mode_controller_status(const std_msgs::msg::String::SharedPtr msg)
  {
    const auto mode_code = parse_actual_motion_mode_code(msg->data);
    if (!mode_code.has_value() || !mode_status_actual_fresh(msg->data)) {
      return;
    }
    actual_motion_mode_code_ = mode_code.value();
    latest_actual_motion_mode_time_ = now();
  }

  void on_spin_to_drive_odom(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    const double wz = msg->twist.twist.angular.z;
    latest_spin_to_drive_wz_radps_ = wz;
    latest_spin_to_drive_wz_time_ = now();
    if (std::isfinite(wz) && std::abs(wz) <= spin_to_drive_wz_threshold_radps_) {
      ++spin_to_drive_stable_sample_count_;
    } else {
      spin_to_drive_stable_sample_count_ = 0;
    }
  }

  void on_spin_to_drive_local_odom(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    const double wz = msg->twist.twist.angular.z;
    const double yaw = yaw_from_quaternion(msg->pose.pose.orientation);
    latest_spin_to_drive_local_wz_radps_ = wz;
    latest_spin_to_drive_local_yaw_rad_ = yaw;
    latest_spin_to_drive_local_odom_time_ = now();

    const bool wz_stable =
      std::isfinite(wz) && std::abs(wz) <= spin_to_drive_local_wz_threshold_radps_;
    const bool yaw_valid = std::isfinite(yaw);
    if (!wz_stable || !yaw_valid) {
      spin_to_drive_local_stable_sample_count_ = 0;
      spin_to_drive_local_stable_anchor_valid_ = false;
      spin_to_drive_local_stable_since_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
      return;
    }

    if (!spin_to_drive_local_stable_anchor_valid_) {
      spin_to_drive_local_stable_anchor_yaw_rad_ = yaw;
      spin_to_drive_local_stable_anchor_valid_ = true;
      spin_to_drive_local_stable_sample_count_ = 1;
      spin_to_drive_local_stable_since_time_ = latest_spin_to_drive_local_odom_time_;
      return;
    }

    const double yaw_delta =
      std::abs(normalize_angle(yaw - spin_to_drive_local_stable_anchor_yaw_rad_));
    if (yaw_delta <= spin_to_drive_local_yaw_delta_threshold_rad_) {
      ++spin_to_drive_local_stable_sample_count_;
      if (spin_to_drive_local_stable_since_time_.nanoseconds() <= 0) {
        spin_to_drive_local_stable_since_time_ = latest_spin_to_drive_local_odom_time_;
      }
    } else {
      spin_to_drive_local_stable_anchor_yaw_rad_ = yaw;
      spin_to_drive_local_stable_sample_count_ = 1;
      spin_to_drive_local_stable_since_time_ = latest_spin_to_drive_local_odom_time_;
    }
  }

  void on_spin_to_drive_imu(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    const double wz = msg->angular_velocity.z;
    latest_spin_to_drive_imu_wz_radps_ = wz;
    latest_spin_to_drive_imu_time_ = now();

    if (std::isfinite(wz) && std::abs(wz) <= spin_to_drive_imu_wz_threshold_radps_) {
      if (spin_to_drive_imu_stable_since_time_.nanoseconds() <= 0) {
        spin_to_drive_imu_stable_since_time_ = latest_spin_to_drive_imu_time_;
      }
      return;
    }
    spin_to_drive_imu_stable_since_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  }

  bool command_is_pure_spin(const geometry_msgs::msg::Twist & cmd) const
  {
    return std::abs(cmd.angular.z) >= spin_to_drive_wz_threshold_radps_ &&
           std::abs(cmd.linear.x) < spin_to_drive_linear_epsilon_mps_ &&
           std::abs(cmd.linear.y) < spin_to_drive_linear_epsilon_mps_;
  }

  bool command_has_linear_motion(const geometry_msgs::msg::Twist & cmd) const
  {
    return std::abs(cmd.linear.x) >= spin_to_drive_linear_epsilon_mps_ ||
           std::abs(cmd.linear.y) >= spin_to_drive_linear_epsilon_mps_;
  }

  std::optional<int> parse_actual_motion_mode_code(const std::string & status) const
  {
    const auto actual_pos = status.find("\"actual_motion_mode\"");
    if (actual_pos == std::string::npos) {
      return std::nullopt;
    }
    const auto code_pos = status.find("\"code\":", actual_pos);
    if (code_pos == std::string::npos) {
      return std::nullopt;
    }
    std::size_t pos = code_pos + std::string("\"code\":").size();
    while (pos < status.size() && std::isspace(static_cast<unsigned char>(status[pos]))) {
      ++pos;
    }
    if (pos >= status.size()) {
      return std::nullopt;
    }
    int sign = 1;
    if (status[pos] == '-') {
      sign = -1;
      ++pos;
    }
    int value = 0;
    bool have_digit = false;
    while (pos < status.size() && std::isdigit(static_cast<unsigned char>(status[pos]))) {
      have_digit = true;
      value = (value * 10) + (status[pos] - '0');
      ++pos;
    }
    if (!have_digit) {
      return std::nullopt;
    }
    return sign * value;
  }

  bool mode_status_actual_fresh(const std::string & status) const
  {
    const auto actual_pos = status.find("\"actual_motion_mode\"");
    if (actual_pos == std::string::npos) {
      return false;
    }
    const auto actual_end = status.find("},\"actual_motion_mode_source\"", actual_pos);
    const auto block = status.substr(
      actual_pos,
      actual_end == std::string::npos ? std::string::npos : actual_end - actual_pos);
    return block.find("\"available\":true") != std::string::npos &&
           block.find("\"fresh\":true") != std::string::npos;
  }

  bool actual_motion_mode_recent() const
  {
    return latest_actual_motion_mode_time_.nanoseconds() > 0 &&
           (now() - latest_actual_motion_mode_time_).seconds() <= mode_exit_guard_status_max_age_sec_;
  }

  bool actual_motion_mode_is_lateral() const
  {
    return actual_motion_mode_recent() &&
           (actual_motion_mode_code_ == 1 || actual_motion_mode_code_ == 3);
  }

  bool command_intends_dual_ackermann_drive(const geometry_msgs::msg::Twist & cmd) const
  {
    return std::abs(cmd.linear.x) >= spin_to_drive_linear_epsilon_mps_ &&
           std::abs(cmd.linear.y) < spin_to_drive_linear_epsilon_mps_;
  }

  geometry_msgs::msg::Twist sanitize_command_for_mode_contract(
    const geometry_msgs::msg::Twist & cmd,
    const CommandSource source) const
  {
    auto sanitized = cmd;
    const bool allow_lateral =
      source == CommandSource::DOCKING ||
      (source == CommandSource::API && allow_api_lateral_cmd_);
    if (!allow_lateral) {
      sanitized.linear.y = 0.0;
      return sanitized;
    }
    if (std::abs(sanitized.linear.y) <= final_cmd_lateral_deadband_mps_) {
      sanitized.linear.y = 0.0;
    }
    if (source == CommandSource::API && api_lateral_max_mps_ > 0.0) {
      sanitized.linear.y = std::clamp(
        sanitized.linear.y,
        -api_lateral_max_mps_,
        api_lateral_max_mps_);
    }
    return sanitized;
  }

  geometry_msgs::msg::Twist prepare_checked_command(
    const geometry_msgs::msg::Twist & cmd,
    const CommandSource source)
  {
    auto sanitized = sanitize_command_for_mode_contract(cmd, source);
    auto gated = apply_spin_to_drive_settle_gate(sanitized);
    gated = sanitize_command_for_mode_contract(gated, source);
    gated = apply_mode_exit_guard(gated, source);
    return sanitize_command_for_mode_contract(gated, source);
  }

  geometry_msgs::msg::Twist apply_mode_exit_guard(
    const geometry_msgs::msg::Twist & cmd,
    const CommandSource source)
  {
    if (!mode_exit_guard_enabled_ || source == CommandSource::DOCKING) {
      mode_exit_guard_started_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
      return cmd;
    }
    if (!command_intends_dual_ackermann_drive(cmd) || !actual_motion_mode_is_lateral()) {
      mode_exit_guard_started_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
      return cmd;
    }

    const auto now_time = now();
    if (mode_exit_guard_started_time_.nanoseconds() <= 0) {
      mode_exit_guard_started_time_ = now_time;
    }
    const double elapsed = (now_time - mode_exit_guard_started_time_).seconds();
    if (mode_exit_guard_timeout_sec_ > 0.0 && elapsed >= mode_exit_guard_timeout_sec_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "mode_exit_guard timed out; holding zero instead of passing dual Ackermann drive "
        "while actual motion_mode=%d",
        actual_motion_mode_code_);
      return geometry_msgs::msg::Twist{};
    }

    geometry_msgs::msg::Twist probe;
    const double speed = std::max(std::abs(cmd.linear.x), mode_exit_guard_probe_speed_mps_);
    probe.linear.x = std::copysign(speed, cmd.linear.x);
    probe.linear.y = 0.0;
    probe.angular.z = 0.0;
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      2000,
      "mode_exit_guard probing DUAL_ACKERMAN with %.3f m/s while actual motion_mode=%d",
      probe.linear.x,
      actual_motion_mode_code_);
    return probe;
  }

  bool spin_to_drive_actual_wz_stable() const
  {
    if (latest_spin_to_drive_wz_time_.nanoseconds() <= 0) {
      return false;
    }
    if ((now() - latest_spin_to_drive_wz_time_).seconds() > spin_to_drive_odom_max_age_sec_) {
      return false;
    }
    return spin_to_drive_stable_sample_count_ >= spin_to_drive_stable_samples_;
  }

  bool spin_to_drive_local_odom_stable() const
  {
    if (!spin_to_drive_require_local_odom_stable_) {
      return true;
    }
    if (spin_to_drive_local_odom_topic_.empty()) {
      return true;
    }
    if (latest_spin_to_drive_local_odom_time_.nanoseconds() <= 0) {
      return false;
    }
    if ((now() - latest_spin_to_drive_local_odom_time_).seconds() >
      spin_to_drive_local_odom_max_age_sec_)
    {
      return false;
    }
    if (spin_to_drive_local_stable_sample_count_ < spin_to_drive_local_stable_samples_) {
      return false;
    }
    if (spin_to_drive_local_stable_duration_sec_ <= 0.0) {
      return true;
    }
    if (spin_to_drive_local_stable_since_time_.nanoseconds() <= 0) {
      return false;
    }
    return (now() - spin_to_drive_local_stable_since_time_).seconds() >=
           spin_to_drive_local_stable_duration_sec_;
  }

  bool spin_to_drive_imu_stable() const
  {
    if (!spin_to_drive_require_imu_stable_) {
      return true;
    }
    if (spin_to_drive_imu_topic_.empty()) {
      return true;
    }
    if (latest_spin_to_drive_imu_time_.nanoseconds() <= 0) {
      return false;
    }
    if ((now() - latest_spin_to_drive_imu_time_).seconds() > spin_to_drive_imu_max_age_sec_) {
      return false;
    }
    if (!std::isfinite(latest_spin_to_drive_imu_wz_radps_) ||
      std::abs(latest_spin_to_drive_imu_wz_radps_) > spin_to_drive_imu_wz_threshold_radps_)
    {
      return false;
    }
    if (spin_to_drive_imu_stable_duration_sec_ <= 0.0) {
      return true;
    }
    if (spin_to_drive_imu_stable_since_time_.nanoseconds() <= 0) {
      return false;
    }
    return (now() - spin_to_drive_imu_stable_since_time_).seconds() >=
           spin_to_drive_imu_stable_duration_sec_;
  }

  geometry_msgs::msg::Twist apply_spin_to_drive_settle_gate(const geometry_msgs::msg::Twist & cmd)
  {
    if (!spin_to_drive_settle_enabled_) {
      return cmd;
    }

    const auto now_time = now();
    if (command_is_pure_spin(cmd)) {
      if (!spin_to_drive_settle_pending_) {
        spin_to_drive_stable_sample_count_ = 0;
        spin_to_drive_local_stable_sample_count_ = 0;
        spin_to_drive_local_stable_anchor_valid_ = false;
        spin_to_drive_local_stable_since_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
        spin_to_drive_imu_stable_since_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
      }
      spin_to_drive_settle_pending_ = true;
      spin_to_drive_settle_started_time_ = now_time;
      spin_to_drive_linear_hold_started_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
      return cmd;
    }

    if (!spin_to_drive_settle_pending_) {
      return cmd;
    }

    if (spin_to_drive_actual_wz_stable() && spin_to_drive_local_odom_stable() &&
      spin_to_drive_imu_stable())
    {
      spin_to_drive_settle_pending_ = false;
      spin_to_drive_linear_hold_started_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
      return cmd;
    }

    if (!command_has_linear_motion(cmd)) {
      return cmd;
    }

    if (spin_to_drive_linear_hold_started_time_.nanoseconds() <= 0) {
      spin_to_drive_linear_hold_started_time_ = now_time;
    }

    if (spin_to_drive_timeout_sec_ > 0.0 &&
      (now_time - spin_to_drive_linear_hold_started_time_).seconds() >= spin_to_drive_timeout_sec_)
    {
      spin_to_drive_settle_pending_ = false;
      spin_to_drive_linear_hold_started_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "spin_to_drive settle timed out while holding linear drive; passing command through. "
        "wheel_wz=%.4f wheel_stable_samples=%d require_local_odom=%s "
        "local_wz=%.4f local_stable_samples=%d "
        "local_stable_sec=%.3f require_imu=%s imu_wz=%.4f imu_stable_sec=%.3f",
        latest_spin_to_drive_wz_radps_,
        spin_to_drive_stable_sample_count_,
        spin_to_drive_require_local_odom_stable_ ? "true" : "false",
        latest_spin_to_drive_local_wz_radps_,
        spin_to_drive_local_stable_sample_count_,
        spin_to_drive_local_stable_since_time_.nanoseconds() > 0 ?
        (now_time - spin_to_drive_local_stable_since_time_).seconds() : 0.0,
        spin_to_drive_require_imu_stable_ ? "true" : "false",
        latest_spin_to_drive_imu_wz_radps_,
        spin_to_drive_imu_stable_since_time_.nanoseconds() > 0 ?
        (now_time - spin_to_drive_imu_stable_since_time_).seconds() : 0.0);
      return cmd;
    }

    return geometry_msgs::msg::Twist{};
  }

  bool fresh_battery_sample() const
  {
    return last_battery_time_.nanoseconds() > 0 &&
      (now() - last_battery_time_).seconds() <= dock_contact_max_age_sec_;
  }

  bool docking_status_indicates_docked() const
  {
    if (!enable_docking_status_guard_) {
      return false;
    }
    for (const auto & prefix : docked_status_prefixes_) {
      if (!prefix.empty() && starts_with(latest_docking_status_, lower_copy(prefix))) {
        return true;
      }
    }
    return false;
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
    if (enable_bms_contact_guard_ && fresh_battery_sample() && battery_contact_active_) {
      return true;
    }
    if (docking_status_indicates_docked()) {
      return true;
    }
    if (!enable_docked_latch_file_guard_ || !dock_contact_latch_is_docked()) {
      return false;
    }
    const bool bms_contradicts_latch =
      enable_bms_contact_guard_ && fresh_battery_sample() && !battery_contact_active_;
    const bool status_allows_latch_clear =
      !enable_docking_status_guard_ || !docking_status_indicates_docked();
    const bool live_no_contact = bms_contradicts_latch && status_allows_latch_clear;
    return !live_no_contact;
  }

  bool estop_active_{false};
  bool localization_ok_{true};
  bool require_localization_health_{false};
  bool block_normal_motion_when_docked_{true};
  bool allow_docking_cmd_when_docked_{true};
  bool enable_api_cmd_priority_{true};
  bool enable_bms_contact_guard_{true};
  bool enable_docking_status_guard_{true};
  bool enable_docked_latch_file_guard_{true};
  bool has_last_snapshot_{false};
  double watchdog_timeout_sec_{1.0};
  double api_cmd_priority_timeout_sec_{0.25};
  double docking_cmd_priority_timeout_sec_{0.25};
  double dock_contact_max_age_sec_{3.0};
  double charging_current_min_a_{0.10};
  double charging_contact_voltage_min_v_{40.0};
  double charging_contact_voltage_max_v_{1000.0};
  double charging_full_soc_threshold_pct_{99.0};
  bool charging_full_soc_voltage_contact_enable_{true};
  bool enable_docking_cmd_priority_{true};
  bool zero_cmd_priority_enabled_{true};
  std::string cmd_vel_in_topic_;
  std::string api_cmd_vel_in_topic_;
  std::string docking_cmd_vel_in_topic_;
  std::string cmd_vel_out_topic_;
  std::string cmd_vel_mirror_topic_;
  int cmd_vel_qos_depth_{1};
  double zero_cmd_priority_epsilon_{0.0001};
  double zero_cmd_priority_burst_sec_{0.25};
  bool spin_to_drive_settle_enabled_{true};
  std::string spin_to_drive_odom_topic_{"/wheel/odom"};
  std::string spin_to_drive_local_odom_topic_{"/local_state/odometry"};
  std::string spin_to_drive_imu_topic_{"/lidar_imu_bias_corrected"};
  bool spin_to_drive_require_local_odom_stable_{false};
  bool spin_to_drive_require_imu_stable_{true};
  double spin_to_drive_wz_threshold_radps_{0.02};
  double spin_to_drive_local_wz_threshold_radps_{0.03};
  double spin_to_drive_imu_wz_threshold_radps_{0.035};
  double spin_to_drive_local_yaw_delta_threshold_rad_{0.005};
  double spin_to_drive_local_stable_duration_sec_{0.30};
  double spin_to_drive_imu_stable_duration_sec_{0.30};
  int spin_to_drive_stable_samples_{5};
  int spin_to_drive_local_stable_samples_{5};
  double spin_to_drive_timeout_sec_{2.0};
  double spin_to_drive_linear_epsilon_mps_{0.03};
  double spin_to_drive_odom_max_age_sec_{0.20};
  double spin_to_drive_local_odom_max_age_sec_{0.20};
  double spin_to_drive_imu_max_age_sec_{0.10};
  bool mode_exit_guard_enabled_{true};
  std::string mode_controller_status_topic_{"/ranger_mini3_mode_controller/status"};
  double mode_exit_guard_probe_speed_mps_{0.06};
  double mode_exit_guard_timeout_sec_{1.0};
  double mode_exit_guard_status_max_age_sec_{0.5};
  double final_cmd_lateral_deadband_mps_{0.001};
  bool allow_api_lateral_cmd_{false};
  double api_lateral_max_mps_{0.10};
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
  rclcpp::Time last_api_cmd_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_docking_cmd_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_battery_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_docking_status_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time latest_spin_to_drive_wz_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time latest_spin_to_drive_local_odom_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time latest_spin_to_drive_imu_time_{0, 0, RCL_ROS_TIME};
  bool last_cmd_was_docking_{false};
  bool last_cmd_was_api_{false};
  bool have_last_api_cmd_{false};
  bool have_last_docking_cmd_{false};
  bool battery_contact_active_{false};
  int actual_motion_mode_code_{255};
  bool spin_to_drive_settle_pending_{false};
  rclcpp::Time spin_to_drive_settle_started_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time spin_to_drive_linear_hold_started_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time spin_to_drive_local_stable_since_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time spin_to_drive_imu_stable_since_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time mode_exit_guard_started_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time latest_actual_motion_mode_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time zero_cmd_priority_until_time_{0, 0, RCL_ROS_TIME};
  double latest_spin_to_drive_wz_radps_{0.0};
  double latest_spin_to_drive_local_wz_radps_{0.0};
  double latest_spin_to_drive_imu_wz_radps_{0.0};
  double latest_spin_to_drive_local_yaw_rad_{0.0};
  double spin_to_drive_local_stable_anchor_yaw_rad_{0.0};
  int spin_to_drive_stable_sample_count_{0};
  int spin_to_drive_local_stable_sample_count_{0};
  bool spin_to_drive_local_stable_anchor_valid_{false};
  mutable bool cached_latch_docked_{false};
  std::string latest_docking_status_;
  mutable rclcpp::Time last_latch_read_time_{0, 0, RCL_ROS_TIME};
  SafetySnapshot last_snapshot_;
  geometry_msgs::msg::Twist last_api_cmd_;
  geometry_msgs::msg::Twist last_docking_cmd_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_mirror_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr motion_allowed_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr api_cmd_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr docking_cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr localization_sub_;
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr docking_status_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr spin_to_drive_odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr spin_to_drive_local_odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr spin_to_drive_imu_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_controller_status_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RobotSafetyNode>());
  rclcpp::shutdown();
  return 0;
}
