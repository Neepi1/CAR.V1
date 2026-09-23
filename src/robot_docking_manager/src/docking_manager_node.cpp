#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <memory>
#include <numeric>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ranger_msgs/msg/motion_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_docking_manager/near_field_docking_controller.hpp"
#include "robot_interfaces/msg/dock_safety_interlock_state.hpp"
#include "robot_interfaces/msg/dock_target_observation.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace
{
constexpr double kPi = 3.14159265358979323846;

double clamp(double value, double low, double high)
{
  return std::min(std::max(value, low), high);
}

double deg_to_rad(double degrees)
{
  return degrees * kPi / 180.0;
}

double median(std::vector<double> values)
{
  if (values.empty()) {
    return 0.0;
  }
  const auto mid = values.begin() + static_cast<std::ptrdiff_t>(values.size() / 2);
  std::nth_element(values.begin(), mid, values.end());
  double result = *mid;
  if (values.size() % 2 == 0) {
    const auto lower = std::max_element(values.begin(), mid);
    result = 0.5 * (result + *lower);
  }
  return result;
}

struct BatteryContactEvaluation
{
  bool contact{false};
  std::string reason{"no_contact"};
};

double normalized_soc_percent(const float percentage)
{
  if (!std::isfinite(percentage)) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  const double raw = static_cast<double>(percentage);
  return std::clamp(raw <= 1.0 ? raw * 100.0 : raw, 0.0, 100.0);
}

bool voltage_in_contact_range(const float voltage, const double min_v, const double max_v)
{
  const double v = static_cast<double>(voltage);
  return std::isfinite(v) && v >= min_v && v <= max_v;
}
}  // namespace

class DockingManagerNode : public rclcpp::Node
{
public:
  DockingManagerNode()
  : Node("docking")
  {
    load_parameters();
    near_field_controller_ =
      std::make_unique<robot_docking_manager::NearFieldDockingController>(
      near_field_control_config());
    active_contact_timeout_s_ = contact_confirm_timeout_s_;
    active_contact_backoff_distance_m_ = contact_retry_backoff_min_distance_m_;

    if (observation_backend_ == "gs2_scan") {
      scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        gs2_scan_topic_, rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::LaserScan::SharedPtr msg) {
          latest_scan_ = std::move(msg);
          last_scan_time_ = now();
          ++observation_sequence_;
        });
    } else if (observation_backend_ == "target_observation") {
      target_observation_sub_ =
        create_subscription<robot_interfaces::msg::DockTargetObservation>(
        target_observation_topic_, rclcpp::QoS(5).reliable(),
        [this](robot_interfaces::msg::DockTargetObservation::SharedPtr msg) {
          if (!target_observation_source_.empty() && msg->source != target_observation_source_) {
            return;
          }
          latest_target_observation_ = std::move(msg);
          last_target_observation_time_ = now();
          ++observation_sequence_;
        });
    } else {
      throw std::invalid_argument("unsupported observation_backend: " + observation_backend_);
    }

    dock_interlock_sub_ = create_subscription<robot_interfaces::msg::DockSafetyInterlockState>(
      dock_interlock_state_topic_, rclcpp::QoS(1).reliable().transient_local(),
      [this](robot_interfaces::msg::DockSafetyInterlockState::SharedPtr msg) {
        latest_dock_interlock_ = std::move(msg);
        last_dock_interlock_time_ = now();
      });

    battery_sub_ = create_subscription<sensor_msgs::msg::BatteryState>(
      charging_state_topic_, rclcpp::QoS(10),
      [this](const sensor_msgs::msg::BatteryState::SharedPtr msg) {
        latest_battery_ = msg;
        const auto contact = battery_charging_contact(*msg, true);
        charging_detected_ = contact.contact;
        charging_contact_detected_ = contact.contact;
        // A standalone BMS sample must never create durable on-dock state.
        // The existing contact-stop path owns the strong latch and stop
        // confirmation. Current-only evidence is scoped to this fine-docking
        // callback, never a cached pre-start sample or an idle undock request.
        if (charging_detected_ && docking_is_active()) {
          begin_contact_stop("docked_charging_detected", contact.reason);
        }
      });

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      undock_odom_topic_, rclcpp::QoS(20),
      [this](nav_msgs::msg::Odometry::SharedPtr msg) {
        latest_odom_ = std::move(msg);
        last_odom_time_ = now();
      });

    wheel_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      contact_stop_wheel_odom_topic_, rclcpp::QoS(20),
      [this](nav_msgs::msg::Odometry::SharedPtr msg) {
        latest_wheel_odom_ = std::move(msg);
        last_wheel_odom_time_ = now();
        ++wheel_odom_sequence_;
      });

    motion_state_sub_ = create_subscription<ranger_msgs::msg::MotionState>(
      contact_stop_motion_state_topic_, rclcpp::QoS(20),
      [this](ranger_msgs::msg::MotionState::SharedPtr msg) {
        latest_motion_state_ = std::move(msg);
        last_motion_state_time_ = now();
        ++motion_state_sequence_;
      });

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(
      cmd_vel_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).reliable());
    status_pub_ = create_publisher<std_msgs::msg::String>(status_topic_, rclcpp::QoS(10).transient_local());
    forced_mode_pub_ = create_publisher<std_msgs::msg::String>(
      forced_mode_topic_, rclcpp::QoS(1).transient_local());
    park_pub_ = create_publisher<std_msgs::msg::Bool>(
      park_topic_, rclcpp::QoS(1).transient_local());
    reverse_enable_pub_ = create_publisher<std_msgs::msg::Bool>(
      reverse_enable_topic_, rclcpp::QoS(1).transient_local());

    start_srv_ = create_service<std_srvs::srv::Trigger>(
      start_service_,
      [this](
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        start_docking();
        response->success = true;
        response->message = "docking started";
      });

    stop_srv_ = create_service<std_srvs::srv::Trigger>(
      stop_service_,
      [this](
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        stop_docking("stopped by service");
        response->success = true;
        response->message = "docking stopped";
      });

    undock_srv_ = create_service<std_srvs::srv::Trigger>(
      undock_service_,
      [this](
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        std::string message;
        response->success = start_undocking(message);
        response->message = message;
      });

    timer_ = create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(1000.0 / control_rate_hz_)),
      [this]() { control_step(); });

    publish_status("idle");
    publish_zero();
    RCLCPP_INFO(
      get_logger(), "Docking observation backend=%s blind_approach=%s",
      observation_backend_.c_str(), allow_blind_approach_ ? "enabled" : "disabled");
  }

private:
  enum class State
  {
    Idle,
    BlindApproach,
    Acquire,
    Align,
    ContactVerify,
    ContactBackoff,
    ContactStopping,
    Undocking,
    Docked,
    Failed
  };

  enum class UndockPhase
  {
    UNDOCK_IDLE,
    UNDOCK_PREPARE,
    UNDOCK_WAIT_FIRST_MOTION,
    UNDOCK_ACTIVE,
    UNDOCK_SUCCEEDED,
    UNDOCK_FAILED_NO_COMMAND_PUBLISHED,
    UNDOCK_FAILED_MOTION_START_TIMEOUT,
    UNDOCK_FAILED_NO_PROGRESS,
    UNDOCK_FAILED_TIMEOUT
  };

  struct Detection
  {
    bool valid{false};
    std::uint64_t sequence{0};
    int points{0};
    double distance_x{0.0};
    double lateral_y{0.0};
    double yaw_error{0.0};
    double lateral_span{0.0};
    double confidence{0.0};
  };

  void load_parameters()
  {
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    observation_backend_ = declare_parameter<std::string>("observation_backend", "target_observation");
    gs2_scan_topic_ = declare_parameter<std::string>("gs2_scan_topic", "/dock/gs2_scan");
    target_observation_topic_ = declare_parameter<std::string>(
      "target_observation_topic", "/dock/target_observation");
    target_observation_source_ = declare_parameter<std::string>(
      "target_observation_source", "orbbec_336l_depth");
    cmd_vel_topic_ = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel_docking");
    status_topic_ = declare_parameter<std::string>("status_topic", "/docking/status");
    start_service_ = declare_parameter<std::string>("start_service", "/docking/start");
    stop_service_ = declare_parameter<std::string>("stop_service", "/docking/stop");
    undock_service_ = declare_parameter<std::string>("undock_service", "/docking/undock");
    charging_state_topic_ = declare_parameter<std::string>("charging_state_topic", "/battery_state");
    dock_interlock_state_topic_ = declare_parameter<std::string>(
      "dock_safety_interlock_state_topic", "/safety/dock_interlock_state");
    dock_interlock_max_age_sec_ = std::max(0.1, declare_parameter<double>(
      "dock_safety_interlock_state_max_age_sec", 1.0));
    docking_contact_latch_file_ = declare_parameter<std::string>(
      "docking_contact_latch_file",
      "/workspaces/njrh-v3/workspace1/maps_release/docking_contact_latch.json");
    forced_mode_topic_ = declare_parameter<std::string>("mode.forced_mode_topic", "/ranger_mini3/forced_mode");
    park_topic_ = declare_parameter<std::string>("mode.park_topic", "/ranger_mini3/park");
    reverse_enable_topic_ =
      declare_parameter<std::string>("mode.reverse_enable_topic", "/ranger_mini3/docking_allow_reverse");
    use_crab_mode_ = declare_parameter<bool>("mode.use_crab_mode", true);
    crab_forced_mode_ = declare_parameter<std::string>("mode.crab_forced_mode", "side_slip");
    yaw_forced_mode_ = declare_parameter<std::string>("mode.yaw_forced_mode", "spinning");
    release_forced_mode_ = declare_parameter<std::string>("mode.release_forced_mode", "auto");
    park_on_docked_ = declare_parameter<bool>("mode.park_on_docked", true);

    gs2_x_m_ = declare_parameter<double>("geometry.gs2_x_m", 0.360);
    charge_contact_x_m_ = declare_parameter<double>("geometry.charge_contact_x_m", 0.398);
    gs2_to_contact_x_m_ = declare_parameter<double>("geometry.gs2_to_contact_x_m", 0.038);

    blind_approach_max_distance_m_ = declare_parameter<double>("approach.blind_approach_max_distance_m", 0.50);
    blind_approach_speed_mps_ = declare_parameter<double>("approach.blind_approach_speed_mps", 0.06);
    allow_blind_approach_ = declare_parameter<bool>("approach.allow_blind_approach", true);
    gs2_acquire_distance_m_ = declare_parameter<double>("approach.gs2_acquire_distance_m", 0.28);
    final_target_distance_m_ = declare_parameter<double>("approach.final_target_distance_m", 0.05);
    undock_distance_m_ = declare_parameter<double>("undock.distance_m", 0.60);
    undock_speed_mps_ = declare_parameter<double>("undock.speed_mps", 0.50);
    undock_max_speed_mps_ = declare_parameter<double>("undock.max_speed_mps", 0.50);
    undock_min_clear_distance_m_ = declare_parameter<double>("undock.min_clear_distance_m", 0.45);
    undock_timeout_s_ = declare_parameter<double>("undock.timeout_s", 12.0);
    undock_odom_topic_ = declare_parameter<std::string>("undock.odom_topic", "/local_state/odometry");
    undock_odom_timeout_s_ = declare_parameter<double>("undock.odom_timeout_s", 0.50);
    undock_odom_start_timeout_s_ = declare_parameter<double>("undock.odom_start_timeout_s", 2.0);
    undock_command_settle_s_ = declare_parameter<double>("undock.command_settle_s", 0.5);
    undock_motion_start_timeout_s_ = declare_parameter<double>("undock.motion_start_timeout_s", 6.0);
    undock_no_progress_timeout_s_ = declare_parameter<double>("undock.no_progress_timeout_s", 2.0);
    undock_progress_epsilon_m_ = declare_parameter<double>("undock.progress_epsilon_m", 0.005);

    contact_stop_motion_state_topic_ = declare_parameter<std::string>(
      "contact_stop.motion_state_topic", "/motion_state");
    contact_stop_wheel_odom_topic_ = declare_parameter<std::string>(
      "contact_stop.wheel_odom_topic", "/wheel/odom");
    contact_stop_feedback_max_age_s_ = declare_parameter<double>(
      "contact_stop.feedback_max_age_s", 0.50);
    contact_stop_linear_speed_threshold_mps_ = declare_parameter<double>(
      "contact_stop.linear_speed_threshold_mps", 0.01);
    contact_stop_angular_speed_threshold_radps_ = declare_parameter<double>(
      "contact_stop.angular_speed_threshold_radps", 0.02);
    contact_stop_stable_duration_s_ = declare_parameter<double>(
      "contact_stop.stable_duration_s", 0.50);
    contact_stop_stable_samples_required_ = declare_parameter<int>(
      "contact_stop.stable_samples", 5);
    contact_stop_feedback_timeout_s_ = declare_parameter<double>(
      "contact_stop.feedback_timeout_s", 3.0);

    lateral_soft_limit_m_ = declare_parameter<double>("tolerances.lateral_soft_limit_m", 0.030);
    lateral_hard_limit_m_ = declare_parameter<double>("tolerances.lateral_hard_limit_m", 0.050);
    yaw_soft_limit_rad_ = deg_to_rad(declare_parameter<double>("tolerances.yaw_soft_limit_deg", 2.0));
    yaw_hard_limit_rad_ = deg_to_rad(declare_parameter<double>("tolerances.yaw_hard_limit_deg", 4.0));
    contact_confirm_timeout_s_ = declare_parameter<double>("tolerances.contact_confirm_timeout_s", 3.0);

    max_linear_speed_mps_ = declare_parameter<double>("safety.max_linear_speed_mps", 0.15);
    max_angular_speed_radps_ = declare_parameter<double>("safety.max_angular_speed_radps", 0.25);
    max_retries_ = declare_parameter<int>("safety.max_retries", 3);
    command_timeout_ms_ = declare_parameter<int>("safety.command_timeout_ms", 300);
    control_rate_hz_ = declare_parameter<double>("safety.control_rate_hz", 20.0);

    detector_min_points_ = declare_parameter<int>("detector.min_points", 8);
    detector_min_span_m_ = declare_parameter<double>("detector.min_lateral_span_m", 0.045);
    detector_lateral_gate_m_ = declare_parameter<double>("detector.lateral_gate_m", 0.20);
    detector_max_range_m_ = declare_parameter<double>("detector.max_range_m", 0.30);
    detector_min_range_m_ = declare_parameter<double>("detector.min_range_m", 0.025);
    detector_front_cluster_x_window_m_ =
      declare_parameter<double>("detector.front_cluster_x_window_m", 0.015);
    detector_min_confidence_ = declare_parameter<double>("detector.min_confidence", 0.10);
    detector_yaw_fit_min_lateral_span_m_ =
      declare_parameter<double>("detector.yaw_fit_min_lateral_span_m", 0.055);
    detector_stable_frames_required_ = declare_parameter<int>("detector.stable_frames_required", 3);
    detection_filter_alpha_ = declare_parameter<double>("detector.filter_alpha", 0.25);
    use_yaw_fit_ = declare_parameter<bool>("detector.use_yaw_fit", false);

    kx_ = declare_parameter<double>("controller.kx", 0.45);
    ky_lateral_ = declare_parameter<double>("controller.ky_lateral", 0.70);
    lateral_command_sign_ = declare_parameter<double>("controller.lateral_command_sign", -1.0);
    kyaw_ = declare_parameter<double>("controller.kyaw", 0.0);
    lateral_deadband_m_ = declare_parameter<double>("controller.lateral_deadband_m", 0.010);
    yaw_deadband_rad_ = deg_to_rad(declare_parameter<double>("controller.yaw_deadband_deg", 1.0));
    min_align_speed_mps_ = declare_parameter<double>("controller.min_align_speed_mps", 0.025);
    min_angular_speed_radps_ =
      declare_parameter<double>("controller.min_angular_speed_radps", 0.05);
    min_lateral_speed_mps_ = declare_parameter<double>("controller.min_lateral_speed_mps", 0.025);
    max_lateral_speed_mps_ = declare_parameter<double>("controller.max_lateral_speed_mps", 0.04);
    yaw_realign_enter_rad_ =
      deg_to_rad(declare_parameter<double>("controller.yaw_realign_enter_deg", 1.0));
    yaw_realign_stable_frames_required_ = std::max(
      1, static_cast<int>(declare_parameter<int>("controller.yaw_realign_stable_frames", 3)));
    yaw_realign_max_count_ = std::max(
      0, static_cast<int>(declare_parameter<int>("controller.yaw_realign_max_count", 1)));
    max_parallel_speed_mps_ = declare_parameter<double>(
      "controller.max_parallel_speed_mps", max_linear_speed_mps_);
    final_approach_window_m_ = declare_parameter<double>(
      "controller.final_approach_window_m", 0.10);
    final_lateral_lock_distance_m_ = declare_parameter<double>(
      "controller.final_lateral_lock_distance_m", 0.06);
    final_forward_speed_mps_ = declare_parameter<double>(
      "controller.final_forward_speed_mps", 0.05);
    lock_lateral_during_final_insert_ =
      declare_parameter<bool>("controller.lock_lateral_during_final_insert", true);
    contact_crawl_speed_mps_ = declare_parameter<double>("controller.contact_crawl_speed_mps", 0.05);
    contact_final_slow_zone_m_ = std::max(
      0.0, declare_parameter<double>("controller.contact_final_slow_zone_m", 0.06));
    contact_final_crawl_speed_mps_ = std::max(
      0.0, declare_parameter<double>("controller.contact_final_crawl_speed_mps", 0.02));
    contact_timeout_safety_factor_ = declare_parameter<double>(
      "controller.contact_timeout_safety_factor", 1.5);
    contact_timeout_margin_s_ = declare_parameter<double>(
      "controller.contact_timeout_margin_s", 2.0);
    contact_timeout_min_s_ = declare_parameter<double>(
      "controller.contact_timeout_min_s", 3.0);
    contact_verify_max_distance_m_ = std::max(
      0.01, declare_parameter<double>("controller.contact_verify_max_distance_m", 0.12));
    contact_verify_retry_enabled_ =
      declare_parameter<bool>("controller.contact_verify_retry_enabled", true);
    contact_retry_max_count_ = std::max(
      0, static_cast<int>(declare_parameter<int>("controller.contact_retry_max_count", 2)));
    contact_retry_backoff_distance_m_ = std::max(
      0.05, declare_parameter<double>("controller.contact_retry_backoff_distance_m", 0.60));
    contact_retry_backoff_min_distance_m_ = declare_parameter<double>(
      "controller.contact_retry_backoff_min_distance_m", 0.20);
    contact_retry_backoff_clearance_margin_m_ = declare_parameter<double>(
      "controller.contact_retry_backoff_clearance_margin_m", 0.08);
    contact_retry_backoff_speed_mps_ = std::max(
      0.0, declare_parameter<double>("controller.contact_retry_backoff_speed_mps", 0.06));
    contact_retry_backoff_timeout_s_ = std::max(
      1.0, declare_parameter<double>("controller.contact_retry_backoff_timeout_s", 20.0));
    contact_retry_backoff_command_settle_s_ = std::max(
      0.0, declare_parameter<double>("controller.contact_retry_backoff_command_settle_s", 0.5));
    contact_retry_backoff_motion_start_timeout_s_ = std::max(
      0.1, declare_parameter<double>("controller.contact_retry_backoff_motion_start_timeout_s", 6.0));
    contact_retry_backoff_no_progress_timeout_s_ = std::max(
      0.1, declare_parameter<double>("controller.contact_retry_backoff_no_progress_timeout_s", 2.0));
    contact_retry_backoff_progress_epsilon_m_ = std::max(
      0.001, declare_parameter<double>("controller.contact_retry_backoff_progress_epsilon_m", 0.005));
    contact_retry_backoff_max_lateral_drift_m_ = std::max(
      0.01, declare_parameter<double>("controller.contact_retry_backoff_max_lateral_drift_m", 0.05));
    min_charging_current_a_ = declare_parameter<double>("charging.min_current_a", 0.10);
    charging_contact_voltage_min_v_ =
      std::max(0.0, declare_parameter<double>("charging.contact_voltage_min_v", 40.0));
    charging_contact_voltage_max_v_ =
      std::max(charging_contact_voltage_min_v_, declare_parameter<double>("charging.contact_voltage_max_v", 1000.0));
    charging_full_soc_threshold_pct_ =
      std::clamp(declare_parameter<double>("charging.full_soc_threshold_pct", 99.0), 0.0, 100.0);
    charging_full_soc_voltage_contact_enable_ =
      declare_parameter<bool>("charging.full_soc_voltage_contact_enable", true);

    control_rate_hz_ = std::max(1.0, control_rate_hz_);
    undock_max_speed_mps_ = std::max(0.0, undock_max_speed_mps_);
    contact_stop_feedback_max_age_s_ = std::max(0.05, contact_stop_feedback_max_age_s_);
    contact_stop_linear_speed_threshold_mps_ =
      std::max(0.0, contact_stop_linear_speed_threshold_mps_);
    contact_stop_angular_speed_threshold_radps_ =
      std::max(0.0, contact_stop_angular_speed_threshold_radps_);
    contact_stop_stable_duration_s_ = std::max(0.0, contact_stop_stable_duration_s_);
    contact_stop_stable_samples_required_ = std::max(1, contact_stop_stable_samples_required_);
    contact_stop_feedback_timeout_s_ = std::max(
      contact_stop_stable_duration_s_, contact_stop_feedback_timeout_s_);
    yaw_soft_limit_rad_ = std::max(0.0, yaw_soft_limit_rad_);
    yaw_hard_limit_rad_ = std::max(yaw_soft_limit_rad_, yaw_hard_limit_rad_);
    yaw_realign_enter_rad_ = std::max(yaw_soft_limit_rad_, yaw_realign_enter_rad_);
    min_angular_speed_radps_ = clamp(
      min_angular_speed_radps_, 0.0, max_angular_speed_radps_);
    max_parallel_speed_mps_ = std::max(0.0, max_parallel_speed_mps_);
    final_approach_window_m_ = std::max(0.015, final_approach_window_m_);
    final_lateral_lock_distance_m_ = clamp(
      final_lateral_lock_distance_m_, 0.0, final_approach_window_m_);
    final_forward_speed_mps_ = clamp(
      final_forward_speed_mps_, 0.0, max_linear_speed_mps_);
    contact_timeout_safety_factor_ = std::max(1.0, contact_timeout_safety_factor_);
    contact_timeout_margin_s_ = std::max(0.0, contact_timeout_margin_s_);
    contact_timeout_min_s_ = std::max(0.0, contact_timeout_min_s_);
    contact_confirm_timeout_s_ = std::max(contact_timeout_min_s_, contact_confirm_timeout_s_);
    contact_retry_backoff_min_distance_m_ = clamp(
      contact_retry_backoff_min_distance_m_, 0.0, contact_retry_backoff_distance_m_);
    contact_retry_backoff_clearance_margin_m_ = std::max(
      0.0, contact_retry_backoff_clearance_margin_m_);
  }

  robot_docking_manager::NearFieldControlConfig near_field_control_config() const
  {
    robot_docking_manager::NearFieldControlConfig config;
    config.target_distance_m = final_target_distance_m_;
    config.distance_tolerance_m = 0.015;
    config.lateral_tolerance_m = lateral_soft_limit_m_;
    config.yaw_exit_tolerance_rad = yaw_soft_limit_rad_;
    config.yaw_reentry_threshold_rad = yaw_realign_enter_rad_;
    config.yaw_stable_samples = yaw_realign_stable_frames_required_;
    config.max_yaw_realignments = yaw_realign_max_count_;
    config.k_forward = kx_;
    config.k_lateral = ky_lateral_;
    config.k_yaw = kyaw_;
    config.lateral_command_sign = lateral_command_sign_;
    config.lateral_deadband_m = lateral_deadband_m_;
    config.yaw_deadband_rad = yaw_deadband_rad_;
    config.min_forward_speed_mps = min_align_speed_mps_;
    config.max_forward_speed_mps = max_linear_speed_mps_;
    config.min_lateral_speed_mps = min_lateral_speed_mps_;
    config.max_lateral_speed_mps = max_lateral_speed_mps_;
    config.min_yaw_speed_radps = min_angular_speed_radps_;
    config.max_yaw_speed_radps = max_angular_speed_radps_;
    config.max_parallel_speed_mps = max_parallel_speed_mps_;
    config.final_approach_window_m = final_approach_window_m_;
    config.final_lateral_lock_distance_m = lock_lateral_during_final_insert_ ?
      final_lateral_lock_distance_m_ : 0.0;
    config.final_forward_speed_mps = final_forward_speed_mps_;
    config.contact_cruise_speed_mps = contact_crawl_speed_mps_;
    config.contact_final_zone_m = contact_final_slow_zone_m_;
    config.contact_final_speed_mps = contact_final_crawl_speed_mps_;
    config.contact_timeout_safety_factor = contact_timeout_safety_factor_;
    config.contact_timeout_margin_sec = contact_timeout_margin_s_;
    config.contact_timeout_min_sec = contact_timeout_min_s_;
    config.retry_backoff_min_distance_m = contact_retry_backoff_min_distance_m_;
    config.retry_backoff_clearance_margin_m = contact_retry_backoff_clearance_margin_m_;
    config.retry_backoff_max_distance_m = contact_retry_backoff_distance_m_;
    return config;
  }

  void start_docking()
  {
    if (state_ == State::ContactStopping) {
      publish_contact_stop_zero(now());
      publish_status("contact_stopping start_ignored=true reason=brake_confirmation_in_progress");
      return;
    }
    reset_undock_tracking();
    reset_contact_tracking();
    reset_contact_backoff_tracking();
    reset_contact_stop_tracking();
    retries_ = 0;
    contact_retry_count_ = 0;
    near_field_controller_->reset();
    active_contact_timeout_s_ = contact_confirm_timeout_s_;
    active_contact_backoff_distance_m_ = contact_retry_backoff_min_distance_m_;
    charging_detected_ = latest_battery_ && battery_indicates_charging(*latest_battery_);
    charging_contact_detected_ = latest_battery_ && battery_indicates_charging_contact(*latest_battery_);
    valid_detection_streak_ = 0;
    has_filtered_detection_ = false;
    if (charging_detected_) {
      const auto contact = latest_battery_ ?
        battery_charging_contact(*latest_battery_) : BatteryContactEvaluation{};
      begin_contact_stop("docked_charging_already_detected", contact.reason);
      return;
    }
    enter_docking_motion_mode();
    state_ = allow_blind_approach_ ? State::BlindApproach : State::Acquire;
    state_entered_time_ = now();
    publish_status(allow_blind_approach_ ? "blind_approach" : "waiting_for_dock_observation");
  }

  void stop_docking(const std::string & reason)
  {
    if (state_ == State::ContactStopping) {
      publish_contact_stop_zero(now());
      publish_status("contact_stopping stop_requested=true reason=" + reason);
      return;
    }
    reset_undock_tracking();
    reset_contact_tracking();
    reset_contact_backoff_tracking();
    reset_contact_stop_tracking();
    near_field_controller_->reset();
    state_ = State::Idle;
    publish_zero();
    publish_reverse_enable(false);
    release_docking_motion_mode(false);
    publish_status(reason);
  }

  void fail(const std::string & reason)
  {
    if (state_ == State::ContactStopping) {
      publish_contact_stop_zero(now());
      publish_status("contact_stopping failure_deferred=true reason=" + reason);
      return;
    }
    reset_undock_tracking();
    reset_contact_tracking();
    reset_contact_backoff_tracking();
    reset_contact_stop_tracking();
    near_field_controller_->reset();
    state_ = State::Failed;
    publish_zero();
    publish_reverse_enable(false);
    release_docking_motion_mode(false);
    publish_status(reason);
  }

  bool safety_memory_allows_undock() const
  {
    if (!latest_dock_interlock_ || !latest_dock_interlock_->enabled ||
      !latest_dock_interlock_->active || !latest_dock_interlock_->memory_latched)
    {
      return false;
    }
    const auto stamp = now();
    const auto source_stamp = rclcpp::Time(latest_dock_interlock_->stamp);
    const double source_age = (stamp - source_stamp).seconds();
    const double receipt_age = (stamp - last_dock_interlock_time_).seconds();
    return source_stamp.nanoseconds() > 0 && source_age >= 0.0 &&
           source_age <= dock_interlock_max_age_sec_ && receipt_age >= 0.0 &&
           receipt_age <= dock_interlock_max_age_sec_;
  }

  bool start_undocking(std::string & message)
  {
    if (state_ == State::ContactStopping) {
      publish_contact_stop_zero(now());
      message = "undock rejected: charging contact brake confirmation is still active";
      publish_status("undock_rejected_contact_stop_unconfirmed");
      return false;
    }
    reset_contact_tracking();
    reset_contact_backoff_tracking();
    reset_contact_stop_tracking();
    charging_detected_ = latest_battery_ && battery_indicates_charging(*latest_battery_);
    charging_contact_detected_ = latest_battery_ && battery_indicates_charging_contact(*latest_battery_);
    const bool dock_latch_detected = dock_contact_latch_is_docked();
    const bool safety_memory_detected = safety_memory_allows_undock();
    if (state_ == State::Undocking) {
      message = "undocking already active";
      return true;
    }
    if (state_ != State::Docked && !charging_contact_detected_ && !dock_latch_detected &&
      !safety_memory_detected)
    {
      message = "undock rejected: robot is not docked and no charging contact is detected";
      publish_status("undock_rejected_not_docked");
      return false;
    }

    reset_undock_tracking();
    state_ = State::Undocking;
    undock_phase_ = UndockPhase::UNDOCK_PREPARE;
    state_entered_time_ = now();
    publish_park(false);
    publish_forced_mode(release_forced_mode_);
    publish_reverse_enable(true);
    // Memory authorizes this explicit recovery request, never a synthetic Docked state.
    message = safety_memory_detected ?
      "undocking started reason=safety_memory_latch" : "undocking started";
    publish_status(std::string(
      "undocking preparing phase=preparing cmd_count=0 reverse_enable=true reverse_enable_count=1") +
      (safety_memory_detected ? " admission=safety_memory_latch" : " admission=dock_contact"));
    return true;
  }

  void transition(State next, const std::string & status)
  {
    if (next == State::Align && state_ != State::Align) {
      near_field_controller_->reset();
    }
    state_ = next;
    state_entered_time_ = now();
    publish_status(status);
  }

  void control_step()
  {
    if (state_ == State::ContactStopping) {
      handle_contact_stopping();
      return;
    }

    if (state_ == State::Undocking) {
      handle_undocking();
      return;
    }

    if (state_ == State::Idle || state_ == State::Docked || state_ == State::Failed) {
      return;
    }

    if (charging_detected_) {
      const auto contact = latest_battery_ ?
        battery_charging_contact(*latest_battery_) : BatteryContactEvaluation{};
      begin_contact_stop("docked_charging_detected", contact.reason);
      return;
    }

    if (state_ == State::ContactBackoff) {
      handle_contact_backoff();
      return;
    }

    // ContactVerify is the calibrated visual handoff boundary. The dock feature
    // enters the camera's near blind zone here, so only BMS, odometry and the
    // bounded straight-crawl limits are authoritative after this transition.
    if (state_ == State::ContactVerify) {
      handle_contact_verify();
      return;
    }

    if (!observation_fresh()) {
      publish_zero();
      publish_status(
        observation_backend_ == "gs2_scan" ?
        "waiting_for_fresh_gs2_scan" : "waiting_for_fresh_dock_observation");
      return;
    }

    Detection detection = detect_dock();
    if (detection.valid) {
      ++valid_detection_streak_;
      detection = filter_detection(detection);
    } else {
      valid_detection_streak_ = 0;
      has_filtered_detection_ = false;
    }

    switch (state_) {
      case State::BlindApproach:
        handle_blind_approach(detection);
        break;
      case State::Acquire:
        handle_acquire(detection);
        break;
      case State::Align:
        handle_align(detection);
        break;
      case State::ContactVerify:
        break;
      case State::ContactBackoff:
        handle_contact_backoff();
        break;
      case State::ContactStopping:
        handle_contact_stopping();
        break;
      case State::Undocking:
        handle_undocking();
        break;
      case State::Idle:
      case State::Docked:
      case State::Failed:
        break;
    }
  }

  bool observation_fresh() const
  {
    if (observation_backend_ == "target_observation") {
      if (!latest_target_observation_) {
        return false;
      }
      const double age_ms = (now() - last_target_observation_time_).seconds() * 1000.0;
      return age_ms <= static_cast<double>(command_timeout_ms_);
    }
    if (!latest_scan_) {
      return false;
    }
    const double age_ms = (now() - last_scan_time_).seconds() * 1000.0;
    return age_ms <= static_cast<double>(command_timeout_ms_);
  }

  Detection detect_dock() const
  {
    if (observation_backend_ == "target_observation") {
      return detect_target_observation();
    }
    return detect_gs2_dock();
  }

  Detection detect_target_observation() const
  {
    Detection detection;
    if (!latest_target_observation_ || !latest_target_observation_->sensor_healthy ||
      !latest_target_observation_->valid)
    {
      return detection;
    }
    if (!latest_target_observation_->header.frame_id.empty() &&
      latest_target_observation_->header.frame_id != base_frame_)
    {
      return detection;
    }

    detection.points = static_cast<int>(std::min<std::uint32_t>(
      latest_target_observation_->inlier_count,
      static_cast<std::uint32_t>(std::numeric_limits<int>::max())));
    detection.sequence = observation_sequence_;
    detection.distance_x = latest_target_observation_->forward_gap_m;
    detection.lateral_y = latest_target_observation_->lateral_error_m;
    detection.yaw_error = latest_target_observation_->yaw_error_rad;
    detection.lateral_span = latest_target_observation_->lateral_span_m;
    detection.confidence = latest_target_observation_->confidence;
    detection.valid =
      std::isfinite(detection.distance_x) && detection.distance_x >= 0.0 &&
      std::isfinite(detection.lateral_y) && std::isfinite(detection.yaw_error) &&
      std::isfinite(detection.lateral_span) && std::isfinite(detection.confidence) &&
      detection.points >= detector_min_points_ &&
      detection.confidence > detector_min_confidence_;
    return detection;
  }

  Detection detect_gs2_dock() const
  {
    Detection detection;
    if (!latest_scan_) {
      return detection;
    }
    detection.sequence = observation_sequence_;

    std::vector<double> xs;
    std::vector<double> ys;
    xs.reserve(latest_scan_->ranges.size());
    ys.reserve(latest_scan_->ranges.size());

    for (size_t i = 0; i < latest_scan_->ranges.size(); ++i) {
      const double r = latest_scan_->ranges[i];
      if (!std::isfinite(r) || r < detector_min_range_m_ || r > detector_max_range_m_) {
        continue;
      }
      const double angle = static_cast<double>(latest_scan_->angle_min) +
        static_cast<double>(i) * static_cast<double>(latest_scan_->angle_increment);
      const double x = r * std::cos(angle);
      const double y = r * std::sin(angle);
      if (x <= 0.0 || x > gs2_acquire_distance_m_ || std::abs(y) > detector_lateral_gate_m_) {
        continue;
      }
      xs.push_back(x);
      ys.push_back(y);
    }

    detection.points = static_cast<int>(xs.size());
    if (detection.points < detector_min_points_) {
      return detection;
    }

    if (detector_front_cluster_x_window_m_ > 0.0) {
      const auto nearest_x_it = std::min_element(xs.begin(), xs.end());
      const double front_x_max = *nearest_x_it + detector_front_cluster_x_window_m_;
      std::vector<double> clustered_xs;
      std::vector<double> clustered_ys;
      clustered_xs.reserve(xs.size());
      clustered_ys.reserve(ys.size());
      for (size_t i = 0; i < xs.size(); ++i) {
        if (xs[i] <= front_x_max) {
          clustered_xs.push_back(xs[i]);
          clustered_ys.push_back(ys[i]);
        }
      }
      xs = std::move(clustered_xs);
      ys = std::move(clustered_ys);
      detection.points = static_cast<int>(xs.size());
      if (detection.points < detector_min_points_) {
        return detection;
      }
    }

    const auto [min_y_it, max_y_it] = std::minmax_element(ys.begin(), ys.end());
    detection.lateral_span = *max_y_it - *min_y_it;
    if (detection.lateral_span < detector_min_span_m_) {
      return detection;
    }

    detection.distance_x = median(xs);
    detection.lateral_y = median(ys);
    detection.yaw_error = (use_yaw_fit_ && detection.lateral_span >= detector_yaw_fit_min_lateral_span_m_) ?
      estimate_yaw_error(xs, ys) : 0.0;
    detection.confidence = std::min(1.0, static_cast<double>(detection.points) / 40.0) *
      std::min(1.0, detection.lateral_span / 0.12);
    detection.valid = detection.confidence > detector_min_confidence_;
    return detection;
  }

  double estimate_yaw_error(const std::vector<double> & xs, const std::vector<double> & ys) const
  {
    const double mean_x = std::accumulate(xs.begin(), xs.end(), 0.0) / static_cast<double>(xs.size());
    const double mean_y = std::accumulate(ys.begin(), ys.end(), 0.0) / static_cast<double>(ys.size());

    double numerator = 0.0;
    double denominator = 0.0;
    for (size_t i = 0; i < xs.size(); ++i) {
      const double dy = ys[i] - mean_y;
      numerator += dy * (xs[i] - mean_x);
      denominator += dy * dy;
    }
    if (std::abs(denominator) < 1e-6) {
      return 0.0;
    }

    // Fit x = slope * y + intercept. A slope means the dock face is yawed relative to the robot.
    return std::atan(numerator / denominator);
  }

  Detection filter_detection(const Detection & current)
  {
    if (!has_filtered_detection_) {
      filtered_detection_ = current;
      has_filtered_detection_ = true;
      return current;
    }

    const double alpha = clamp(detection_filter_alpha_, 0.0, 1.0);
    filtered_detection_.valid = current.valid;
    filtered_detection_.sequence = current.sequence;
    filtered_detection_.points = current.points;
    filtered_detection_.distance_x = alpha * current.distance_x + (1.0 - alpha) * filtered_detection_.distance_x;
    filtered_detection_.lateral_y = alpha * current.lateral_y + (1.0 - alpha) * filtered_detection_.lateral_y;
    filtered_detection_.yaw_error = alpha * current.yaw_error + (1.0 - alpha) * filtered_detection_.yaw_error;
    filtered_detection_.lateral_span = alpha * current.lateral_span + (1.0 - alpha) * filtered_detection_.lateral_span;
    filtered_detection_.confidence = alpha * current.confidence + (1.0 - alpha) * filtered_detection_.confidence;
    return filtered_detection_;
  }

  void handle_blind_approach(const Detection & detection)
  {
    if (detection.valid && valid_detection_streak_ >= detector_stable_frames_required_) {
      transition(State::Align, "dock_feature_acquired");
      return;
    }

    const double elapsed = (now() - state_entered_time_).seconds();
    const double max_time = blind_approach_max_distance_m_ / std::max(0.001, blind_approach_speed_mps_);
    if (elapsed > max_time) {
      transition(State::Acquire, "blind_approach_complete_waiting_for_dock_feature");
      return;
    }

    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = clamp(blind_approach_speed_mps_, 0.0, max_linear_speed_mps_);
    publish_cmd(cmd);
  }

  void handle_acquire(const Detection & detection)
  {
    if (detection.valid && valid_detection_streak_ >= detector_stable_frames_required_) {
      transition(State::Align, "dock_feature_acquired");
      return;
    }

    if ((now() - state_entered_time_).seconds() > 2.0) {
      if (retries_++ >= max_retries_) {
        fail("dock_feature_not_found");
        return;
      }
      transition(
        allow_blind_approach_ ? State::BlindApproach : State::Acquire,
        allow_blind_approach_ ? "retry_blind_approach" : "retry_waiting_for_dock_observation");
      return;
    }
    publish_zero();
  }

  static const char * near_field_phase_text(
    const robot_docking_manager::NearFieldPhase phase)
  {
    switch (phase) {
      case robot_docking_manager::NearFieldPhase::YawCapture:
        return "yaw_capture";
      case robot_docking_manager::NearFieldPhase::VectorApproach:
        return "vector_approach";
      case robot_docking_manager::NearFieldPhase::FinalApproach:
        return "final_approach";
      case robot_docking_manager::NearFieldPhase::AlignmentBlocked:
        return "alignment_blocked";
    }
    return "unknown";
  }

  void handle_align(const Detection & detection)
  {
    if (!detection.valid) {
      transition(State::Acquire, "lost_dock_feature");
      return;
    }
    if (!use_crab_mode_) {
      fail("near_field_docking_requires_parallel_mode");
      return;
    }

    if (std::abs(detection.lateral_y) > lateral_hard_limit_m_ ||
      std::abs(detection.yaw_error) > yaw_hard_limit_rad_)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "dock alignment outside hard limit: y=%.3f yaw=%.3fdeg",
        detection.lateral_y, detection.yaw_error * 180.0 / kPi);
    }

    const robot_docking_manager::NearFieldObservation observation{
      true,
      detection.sequence,
      detection.distance_x,
      detection.lateral_y,
      detection.yaw_error};
    const auto decision = near_field_controller_->step(observation);
    if (decision.alignment_blocked) {
      publish_zero();
      fail(
        "near_field_" +
        (decision.reason.empty() ? std::string("alignment_blocked") : decision.reason));
      return;
    }

    if (decision.enter_contact_verify) {
      if (!capture_contact_start_odom()) {
        publish_zero();
        publish_status("contact_verify_waiting_for_fresh_odom");
        return;
      }
      active_contact_timeout_s_ = std::min(
        contact_confirm_timeout_s_,
        near_field_controller_->contact_timeout_sec(contact_verify_max_distance_m_));
      std::ostringstream status;
      status << "contact_verify"
             << " timeout_s=" << std::fixed << std::setprecision(3)
             << active_contact_timeout_s_
             << " distance_budget_m=" << contact_verify_max_distance_m_;
      transition(State::ContactVerify, status.str());
      return;
    }

    const bool spinning =
      decision.mode == robot_docking_manager::NearFieldMotionMode::Spinning;
    const std::string & forced_mode = spinning ? yaw_forced_mode_ : crab_forced_mode_;
    publish_forced_mode(forced_mode);

    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = decision.linear_x_mps;
    cmd.linear.y = decision.linear_y_mps;
    cmd.angular.z = decision.angular_z_radps;
    publish_cmd(cmd);

    std::ostringstream status;
    status << "near_field_align"
           << " phase=" << near_field_phase_text(decision.phase)
           << " reason=" << decision.reason
           << " forced_mode=" << forced_mode
           << " cmd_vx=" << std::fixed << std::setprecision(3) << cmd.linear.x
           << " cmd_vy=" << cmd.linear.y
           << " cmd_wz=" << cmd.angular.z
           << " yaw_realignments=" << decision.yaw_realignments
           << "/" << yaw_realign_max_count_
           << " yaw_stable_frames=" << decision.yaw_stable_samples
           << "/" << yaw_realign_stable_frames_required_
           << " distance_x=" << detection.distance_x
           << " lateral_y=" << detection.lateral_y
           << " yaw_deg=" << detection.yaw_error * 180.0 / kPi;
    publish_status(status.str());
  }

  void begin_contact_stop(const std::string & success_status, const std::string & bms_reason)
  {
    const auto stamp = now();
    if (state_ == State::ContactStopping) {
      publish_contact_stop_zero(stamp);
      return;
    }

    const bool was_contact_verify = state_ == State::ContactVerify;
    const double contact_verify_traveled_at_bms =
      have_contact_start_odom_ ? contact_verify_traveled_m() : -1.0;
    const double contact_verify_elapsed_at_bms = was_contact_verify ?
      std::max(0.0, (stamp - state_entered_time_).seconds()) : -1.0;
    const bool have_wheel_pose_at_bms = static_cast<bool>(latest_wheel_odom_);
    const double wheel_x_at_bms = have_wheel_pose_at_bms ?
      latest_wheel_odom_->pose.pose.position.x : 0.0;
    const double wheel_y_at_bms = have_wheel_pose_at_bms ?
      latest_wheel_odom_->pose.pose.position.y : 0.0;

    reset_contact_tracking();
    reset_contact_backoff_tracking();
    near_field_controller_->reset();
    reset_contact_stop_tracking();
    state_ = State::ContactStopping;
    state_entered_time_ = stamp;
    contact_stop_success_status_ = success_status;
    contact_stop_bms_reason_ = bms_reason.empty() ? "unspecified" : bms_reason;
    contact_stop_bms_time_ = stamp;
    contact_stop_start_wheel_odom_sequence_ = wheel_odom_sequence_;
    contact_stop_start_motion_state_sequence_ = motion_state_sequence_;
    contact_stop_contact_verify_traveled_m_ = contact_verify_traveled_at_bms;
    contact_stop_contact_verify_elapsed_s_ = contact_verify_elapsed_at_bms;
    contact_stop_have_wheel_pose_at_bms_ = have_wheel_pose_at_bms;
    contact_stop_wheel_x_at_bms_ = wheel_x_at_bms;
    contact_stop_wheel_y_at_bms_ = wheel_y_at_bms;
    update_dock_contact_latch(true, "docking_manager", success_status, "");

    publish_contact_stop_zero(stamp);

    std::ostringstream event;
    event << "DOCK_BRAKE_START"
          << " bms_reason=" << contact_stop_bms_reason_
          << " bms_rx_ns=" << contact_stop_bms_time_.nanoseconds()
          << " first_zero_ns=" << contact_stop_first_zero_time_.nanoseconds()
          << " bms_to_first_zero_ms=" << std::fixed << std::setprecision(3)
          << contact_stop_elapsed_ms(contact_stop_bms_time_, contact_stop_first_zero_time_)
          << " contact_verify_traveled_at_bms_m=" << contact_stop_contact_verify_traveled_m_
          << " contact_verify_elapsed_at_bms_s=" << contact_stop_contact_verify_elapsed_s_
          << " wheel_sequence_at_bms=" << contact_stop_start_wheel_odom_sequence_
          << " motion_state_sequence_at_bms=" << contact_stop_start_motion_state_sequence_;
    RCLCPP_INFO(get_logger(), "%s", event.str().c_str());
    publish_status(contact_stop_status(stamp, false));
  }

  void publish_contact_stop_zero(const rclcpp::Time & stamp)
  {
    publish_zero();
    if (contact_stop_first_zero_time_.nanoseconds() == 0) {
      contact_stop_first_zero_time_ = stamp;
    }
    contact_stop_last_zero_time_ = stamp;
    ++contact_stop_zero_cmd_count_;
  }

  double contact_stop_elapsed_ms(
    const rclcpp::Time & start, const rclcpp::Time & end) const
  {
    if (start.nanoseconds() <= 0 || end.nanoseconds() <= 0) {
      return -1.0;
    }
    return std::max(0.0, (end - start).seconds() * 1000.0);
  }

  double contact_stop_post_bms_distance_m() const
  {
    if (!contact_stop_have_wheel_pose_at_bms_ || !latest_wheel_odom_) {
      return -1.0;
    }
    const auto & position = latest_wheel_odom_->pose.pose.position;
    return std::hypot(
      position.x - contact_stop_wheel_x_at_bms_,
      position.y - contact_stop_wheel_y_at_bms_);
  }

  std::string contact_stop_status(
    const rclcpp::Time & stamp, const bool feedback_timeout) const
  {
    const bool motion_post_bms =
      motion_state_sequence_ > contact_stop_start_motion_state_sequence_;
    const bool wheel_post_bms =
      wheel_odom_sequence_ > contact_stop_start_wheel_odom_sequence_;
    const double motion_age = last_motion_state_time_.nanoseconds() > 0 ?
      std::max(0.0, (stamp - last_motion_state_time_).seconds()) : -1.0;
    const double wheel_age = last_wheel_odom_time_.nanoseconds() > 0 ?
      std::max(0.0, (stamp - last_wheel_odom_time_).seconds()) : -1.0;
    const bool motion_fresh = motion_post_bms && motion_age >= 0.0 &&
      motion_age <= contact_stop_feedback_max_age_s_;
    const bool wheel_fresh = wheel_post_bms && wheel_age >= 0.0 &&
      wheel_age <= contact_stop_feedback_max_age_s_;
    const double linear_speed = latest_wheel_odom_ ? std::hypot(
      latest_wheel_odom_->twist.twist.linear.x,
      latest_wheel_odom_->twist.twist.linear.y) : std::numeric_limits<double>::quiet_NaN();
    const double angular_speed = latest_wheel_odom_ ?
      std::abs(latest_wheel_odom_->twist.twist.angular.z) :
      std::numeric_limits<double>::quiet_NaN();
    const int motion_mode = latest_motion_state_ ?
      static_cast<int>(latest_motion_state_->motion_mode) : -1;
    const double stable_for = contact_stop_first_stable_sample_time_.nanoseconds() > 0 ?
      std::max(0.0, (stamp - contact_stop_first_stable_sample_time_).seconds()) : 0.0;

    std::ostringstream status;
    status << "contact_stopping"
           << " brake_confirmed=false"
           << " bms_reason=" << contact_stop_bms_reason_
           << " elapsed_s=" << std::fixed << std::setprecision(3)
           << std::max(0.0, (stamp - contact_stop_bms_time_).seconds())
           << " bms_to_first_zero_ms="
           << contact_stop_elapsed_ms(contact_stop_bms_time_, contact_stop_first_zero_time_)
           << " first_zero_to_stop_ms="
           << contact_stop_elapsed_ms(
              contact_stop_first_zero_time_, contact_stop_first_stable_sample_time_)
           << " post_bms_distance_m=" << contact_stop_post_bms_distance_m()
           << " zero_cmd_count=" << contact_stop_zero_cmd_count_
           << " last_zero_ns=" << contact_stop_last_zero_time_.nanoseconds()
           << " wheel_linear_speed_mps=" << linear_speed
           << " wheel_angular_speed_radps=" << angular_speed
           << " wheel_odom_fresh=" << bool_text(wheel_fresh)
           << " wheel_odom_age_s=" << wheel_age
           << " wheel_odom_post_bms=" << bool_text(wheel_post_bms)
           << " motion_state_fresh=" << bool_text(motion_fresh)
           << " motion_state_age_s=" << motion_age
           << " motion_state_post_bms=" << bool_text(motion_post_bms)
           << " motion_mode=" << motion_mode
           << " stable_samples=" << contact_stop_stable_samples_
           << "/" << contact_stop_stable_samples_required_
           << " stable_for_s=" << stable_for
           << " contact_stop_feedback_timeout=" << bool_text(feedback_timeout);
    return status.str();
  }

  void handle_contact_stopping()
  {
    const auto stamp = now();
    publish_contact_stop_zero(stamp);

    const bool motion_post_bms =
      motion_state_sequence_ > contact_stop_start_motion_state_sequence_;
    const bool wheel_post_bms =
      wheel_odom_sequence_ > contact_stop_start_wheel_odom_sequence_;
    const bool motion_fresh = motion_post_bms &&
      last_motion_state_time_.nanoseconds() > 0 &&
      (stamp - last_motion_state_time_).seconds() >= 0.0 &&
      (stamp - last_motion_state_time_).seconds() <= contact_stop_feedback_max_age_s_;
    const bool wheel_fresh = wheel_post_bms &&
      last_wheel_odom_time_.nanoseconds() > 0 &&
      (stamp - last_wheel_odom_time_).seconds() >= 0.0 &&
      (stamp - last_wheel_odom_time_).seconds() <= contact_stop_feedback_max_age_s_;

    double linear_speed = std::numeric_limits<double>::infinity();
    double angular_speed = std::numeric_limits<double>::infinity();
    if (latest_wheel_odom_) {
      const auto & wheel_twist = latest_wheel_odom_->twist.twist;
      linear_speed = std::hypot(wheel_twist.linear.x, wheel_twist.linear.y);
      angular_speed = std::abs(wheel_twist.angular.z);
    }
    const bool speeds_finite = std::isfinite(linear_speed) && std::isfinite(angular_speed);
    const bool speeds_stopped = speeds_finite &&
      linear_speed <= contact_stop_linear_speed_threshold_mps_ &&
      angular_speed <= contact_stop_angular_speed_threshold_radps_;
    const bool feedback_ready = motion_fresh && wheel_fresh;

    if (!feedback_ready || !speeds_stopped) {
      contact_stop_stable_samples_ = 0;
      contact_stop_first_stable_sample_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
      contact_stop_last_evaluated_wheel_sequence_ = wheel_odom_sequence_;
    } else if (wheel_odom_sequence_ != contact_stop_last_evaluated_wheel_sequence_) {
      contact_stop_last_evaluated_wheel_sequence_ = wheel_odom_sequence_;
      if (contact_stop_stable_samples_ == 0) {
        contact_stop_first_stable_sample_time_ = last_wheel_odom_time_;
      }
      ++contact_stop_stable_samples_;
    }

    const double stable_for_s = contact_stop_first_stable_sample_time_.nanoseconds() > 0 ?
      std::max(0.0, (stamp - contact_stop_first_stable_sample_time_).seconds()) : 0.0;
    if (feedback_ready && speeds_stopped &&
      contact_stop_stable_samples_ >= contact_stop_stable_samples_required_ &&
      stable_for_s >= contact_stop_stable_duration_s_)
    {
      finalize_docked_stop(contact_stop_success_status_, stamp);
      return;
    }

    const bool feedback_timeout =
      (stamp - contact_stop_bms_time_).seconds() >= contact_stop_feedback_timeout_s_;
    const auto status = contact_stop_status(stamp, feedback_timeout);
    publish_status(status);
    if (feedback_timeout) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Charging contact stop feedback is not yet confirmed; continuing zero command: %s",
        status.c_str());
    } else {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500, "%s", status.c_str());
    }
  }

  void handle_contact_verify()
  {
    if (charging_detected_) {
      const auto contact = latest_battery_ ?
        battery_charging_contact(*latest_battery_) : BatteryContactEvaluation{};
      begin_contact_stop("docked_charging_detected", contact.reason);
      return;
    }

    if (!have_contact_start_odom_ || !odom_fresh()) {
      fail("contact_verify_failed_stale_odom");
      return;
    }

    const double traveled = contact_verify_traveled_m();
    if (traveled >= contact_verify_max_distance_m_) {
      begin_contact_retry("distance_limit", "contact_verify_failed_distance_limit");
      return;
    }

    if ((now() - state_entered_time_).seconds() > active_contact_timeout_s_) {
      begin_contact_retry("contact_wait_expired", "contact_verify_timeout");
      return;
    }

    const double remaining = std::max(0.0, contact_verify_max_distance_m_ - traveled);
    const bool final_slow_zone =
      contact_final_slow_zone_m_ > 0.0 && remaining <= contact_final_slow_zone_m_;
    double contact_speed = clamp(contact_crawl_speed_mps_, 0.0, max_linear_speed_mps_);
    if (final_slow_zone) {
      contact_speed = std::min(
        contact_speed,
        clamp(contact_final_crawl_speed_mps_, 0.0, max_linear_speed_mps_));
    }

    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = contact_speed;
    cmd.linear.y = 0.0;
    cmd.angular.z = 0.0;
    publish_cmd(cmd);

    std::ostringstream status;
    status << "contact_verify"
           << " distance=" << std::fixed << std::setprecision(3)
           << traveled << "/" << contact_verify_max_distance_m_
           << " remaining=" << remaining
           << " timeout_s=" << active_contact_timeout_s_
           << " final_slow_zone=" << bool_text(final_slow_zone)
           << " cmd_x=" << cmd.linear.x;
    publish_status(status.str());
  }

  void begin_contact_retry(
    const std::string & trigger, const std::string & terminal_failure_reason)
  {
    if (!contact_verify_retry_enabled_ || contact_retry_count_ >= contact_retry_max_count_) {
      fail(terminal_failure_reason);
      return;
    }

    const double attempted_contact_distance_m = have_contact_start_odom_ ?
      contact_verify_traveled_m() : 0.0;
    ++contact_retry_count_;
    publish_zero();
    reset_contact_tracking();
    reset_contact_backoff_tracking();
    active_contact_backoff_distance_m_ = near_field_controller_->retry_backoff_distance_m(
      attempted_contact_distance_m);
    publish_forced_mode(release_forced_mode_);
    publish_reverse_enable(true);

    std::ostringstream status;
    status << "contact_retry_backoff phase=prepare"
           << " attempt=" << contact_retry_count_ << "/" << contact_retry_max_count_
           << " trigger=" << trigger
           << " attempted_contact_distance=" << std::fixed << std::setprecision(3)
           << attempted_contact_distance_m
           << " target_distance=" << std::fixed << std::setprecision(3)
           << active_contact_backoff_distance_m_;
    transition(State::ContactBackoff, status.str());
  }

  void handle_contact_backoff()
  {
    if (charging_detected_) {
      const auto contact = latest_battery_ ?
        battery_charging_contact(*latest_battery_) : BatteryContactEvaluation{};
      begin_contact_stop("docked_charging_detected", contact.reason);
      return;
    }

    const auto stamp = now();
    const double elapsed = (stamp - state_entered_time_).seconds();
    const double distance = active_contact_backoff_distance_m_;
    const double speed = clamp(contact_retry_backoff_speed_mps_, 0.0, max_linear_speed_mps_);
    if (distance <= 0.0 || speed <= 1.0e-3) {
      fail("contact_retry_backoff_failed_invalid_config");
      return;
    }

    publish_forced_mode(release_forced_mode_);
    publish_reverse_enable(true);

    if (!have_contact_backoff_start_odom_) {
      if (capture_contact_backoff_start_odom()) {
        last_contact_backoff_progress_time_ = stamp;
        publish_zero();
        publish_status(contact_backoff_status("odom_reference_captured", 0.0, 0.0));
      } else {
        publish_zero();
        if (elapsed > undock_odom_start_timeout_s_) {
          fail("contact_retry_backoff_failed_no_fresh_odom");
          return;
        }
        publish_status(contact_backoff_status("waiting_for_fresh_odom", 0.0, 0.0));
      }
      return;
    }

    if (!odom_fresh()) {
      fail("contact_retry_backoff_failed_stale_odom");
      return;
    }

    const double traveled = contact_backoff_traveled_m();
    const double lateral = contact_backoff_lateral_m();
    if (std::abs(lateral) > contact_retry_backoff_max_lateral_drift_m_) {
      fail("contact_retry_backoff_failed_lateral_drift");
      return;
    }
    if (traveled >= distance) {
      finish_contact_backoff(traveled, lateral);
      return;
    }
    if (elapsed > contact_retry_backoff_timeout_s_) {
      fail("contact_retry_backoff_failed_timeout");
      return;
    }

    if (elapsed < contact_retry_backoff_command_settle_s_) {
      publish_zero();
      publish_status(contact_backoff_status("command_settle", traveled, lateral));
      return;
    }

    if (!have_contact_backoff_first_motion_) {
      if (traveled > contact_retry_backoff_progress_epsilon_m_) {
        have_contact_backoff_first_motion_ = true;
        contact_backoff_max_progress_m_ = traveled;
        last_contact_backoff_progress_time_ = stamp;
      } else if (contact_backoff_nonzero_cmd_start_time_.nanoseconds() > 0 &&
        (stamp - contact_backoff_nonzero_cmd_start_time_).seconds() >
        contact_retry_backoff_motion_start_timeout_s_)
      {
        fail("contact_retry_backoff_failed_motion_start_timeout");
        return;
      }
    } else if (traveled > contact_backoff_max_progress_m_ +
      contact_retry_backoff_progress_epsilon_m_)
    {
      contact_backoff_max_progress_m_ = traveled;
      last_contact_backoff_progress_time_ = stamp;
    } else if ((stamp - last_contact_backoff_progress_time_).seconds() >
      contact_retry_backoff_no_progress_timeout_s_)
    {
      fail("contact_retry_backoff_failed_no_progress");
      return;
    }

    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = -speed;
    cmd.linear.y = 0.0;
    cmd.angular.z = 0.0;
    publish_cmd(cmd);
    ++contact_backoff_cmd_count_;
    if (contact_backoff_nonzero_cmd_start_time_.nanoseconds() == 0) {
      contact_backoff_nonzero_cmd_start_time_ = stamp;
    }
    publish_status(contact_backoff_status(
      have_contact_backoff_first_motion_ ? "active" : "waiting_first_motion",
      traveled, lateral));
  }

  void finish_contact_backoff(const double traveled, const double lateral)
  {
    const int attempt = contact_retry_count_;
    const size_t cmd_count = contact_backoff_cmd_count_;
    publish_zero();
    publish_reverse_enable(false);
    reset_contact_backoff_tracking();
    retries_ = 0;
    valid_detection_streak_ = 0;
    has_filtered_detection_ = false;
    enter_docking_motion_mode();

    std::ostringstream status;
    status << "contact_retry_reacquire"
           << " attempt=" << attempt << "/" << contact_retry_max_count_
           << " backed_off_m=" << std::fixed << std::setprecision(3) << traveled
           << " lateral_m=" << lateral
           << " cmd_count=" << cmd_count;
    transition(State::Acquire, status.str());
  }

  std::string contact_backoff_status(
    const std::string & phase, const double traveled, const double lateral) const
  {
    std::ostringstream status;
    status << "contact_retry_backoff"
           << " phase=" << phase
           << " attempt=" << contact_retry_count_ << "/" << contact_retry_max_count_
           << " distance=" << std::fixed << std::setprecision(3)
           << traveled << "/" << active_contact_backoff_distance_m_
           << " lateral=" << lateral
           << " cmd_x=" << -clamp(contact_retry_backoff_speed_mps_, 0.0, max_linear_speed_mps_)
           << " cmd_count=" << contact_backoff_cmd_count_
           << " reverse_enable=true"
           << " first_motion_started=" << bool_text(have_contact_backoff_first_motion_);
    return status.str();
  }

  void handle_undocking()
  {
    const auto stamp = now();
    const double speed = clamp(undock_speed_mps_, 0.0, undock_max_speed_mps_);
    const double elapsed = (stamp - state_entered_time_).seconds();
    const double distance = std::max(0.0, undock_distance_m_);
    if (speed <= 1.0e-3) {
      fail_undock(
        UndockPhase::UNDOCK_FAILED_TIMEOUT,
        undock_failure_status("undock_failed_invalid_speed", 0.0, distance, stamp));
      return;
    }

    if (!have_undock_start_odom_) {
      if (capture_undock_start_odom()) {
        publish_status(undock_running_status("odom_reference_captured", stamp, 0.0, distance));
      } else {
        publish_zero();
        if (elapsed > undock_odom_start_timeout_s_) {
          fail_undock(
            UndockPhase::UNDOCK_FAILED_TIMEOUT,
            undock_failure_status("undock_failed_no_fresh_odom", 0.0, distance, stamp));
          return;
        }
        publish_status(undock_running_status("waiting_for_fresh_odom", stamp, 0.0, distance));
        return;
      }
    }

    if (!odom_fresh()) {
      fail_undock(
        UndockPhase::UNDOCK_FAILED_TIMEOUT,
        undock_failure_status("undock_failed_stale_odom", undock_traveled_m(), distance, stamp));
      return;
    }

    const double traveled = undock_traveled_m();
    if (traveled >= distance) {
      finish_undock_success(traveled);
      return;
    }
    if (elapsed > undock_timeout_s_) {
      if (undock_nonzero_cmd_publish_count_ == 0U) {
        fail_undock(
          UndockPhase::UNDOCK_FAILED_NO_COMMAND_PUBLISHED,
          undock_failure_status("undock_failed_no_command_published", traveled, distance, stamp));
        return;
      }
      fail_undock(
        UndockPhase::UNDOCK_FAILED_TIMEOUT,
        undock_failure_status("undock_failed_timeout", traveled, distance, stamp));
      return;
    }

    if (undock_phase_ == UndockPhase::UNDOCK_PREPARE) {
      publish_reverse_enable(true);
      publish_zero();
      last_undock_cmd_x_ = 0.0;
      const double settle_s = std::max(0.0, undock_command_settle_s_);
      if (elapsed < settle_s) {
        std::ostringstream status;
        status << "undocking command_settle phase=command_settle elapsed=" << std::fixed << std::setprecision(2)
               << elapsed << "/" << settle_s
               << " distance=" << std::setprecision(3) << traveled << "/" << distance
               << " cmd_x=0.000 cmd_count=" << undock_nonzero_cmd_publish_count_
               << " reverse_enable=true reverse_enable_count=" << undock_reverse_enable_publish_count_;
        publish_status(status.str());
        return;
      }
      undock_phase_ = UndockPhase::UNDOCK_WAIT_FIRST_MOTION;
    }

    bool command_published_this_tick = false;
    if (undock_phase_ == UndockPhase::UNDOCK_WAIT_FIRST_MOTION) {
      publish_undock_reverse_command(speed, stamp);
      command_published_this_tick = true;

      if (traveled > undock_progress_epsilon_m_) {
        undock_phase_ = UndockPhase::UNDOCK_ACTIVE;
        have_undock_first_motion_ = true;
        undock_max_progress_m_ = traveled;
        first_undock_motion_time_ = stamp;
        last_undock_progress_time_ = stamp;
      } else {
        const double motion_wait =
          undock_nonzero_cmd_start_time_.nanoseconds() > 0 ?
          (stamp - undock_nonzero_cmd_start_time_).seconds() : 0.0;
        if (undock_nonzero_cmd_publish_count_ == 0U &&
          elapsed > std::max(0.0, undock_command_settle_s_) + undock_motion_start_timeout_s_) {
          fail_undock(
            UndockPhase::UNDOCK_FAILED_NO_COMMAND_PUBLISHED,
            undock_failure_status("undock_failed_no_command_published", traveled, distance, stamp));
          return;
        }
        if (undock_nonzero_cmd_publish_count_ > 0U && motion_wait > undock_motion_start_timeout_s_) {
          fail_undock(
            UndockPhase::UNDOCK_FAILED_MOTION_START_TIMEOUT,
            undock_failure_status("undock_failed_motion_start_timeout", traveled, distance, stamp));
          return;
        }
        std::ostringstream status;
        status << "undocking waiting_first_motion phase=waiting_first_motion distance=" << std::fixed
               << std::setprecision(3)
               << traveled << "/" << distance
               << " cmd_x=" << last_undock_cmd_x_
               << " cmd_count=" << undock_nonzero_cmd_publish_count_
               << " reverse_enable=true reverse_enable_count=" << undock_reverse_enable_publish_count_
               << " command_start_elapsed_s=" << std::setprecision(2) << motion_wait
               << " motion_start_timeout_s=" << undock_motion_start_timeout_s_;
        publish_status(status.str());
        return;
      }
    }

    if (undock_phase_ == UndockPhase::UNDOCK_ACTIVE) {
      if (traveled > undock_max_progress_m_ + undock_progress_epsilon_m_) {
        undock_max_progress_m_ = traveled;
        last_undock_progress_time_ = stamp;
      }
      if ((stamp - last_undock_progress_time_).seconds() > undock_no_progress_timeout_s_) {
        fail_undock(
          UndockPhase::UNDOCK_FAILED_NO_PROGRESS,
          undock_failure_status("undock_failed_no_progress", traveled, distance, stamp));
        return;
      }
    }

    if (traveled >= distance) {
      finish_undock_success(traveled);
      return;
    }

    if (!command_published_this_tick) {
      publish_undock_reverse_command(speed, stamp);
    }

    std::ostringstream status;
    status << undock_status_prefix(traveled)
           << " phase=active"
           << " distance=" << std::fixed << std::setprecision(3) << traveled
           << "/" << distance
           << " cmd_x=" << last_undock_cmd_x_
           << " cmd_count=" << undock_nonzero_cmd_publish_count_
           << " reverse_enable=true reverse_enable_count=" << undock_reverse_enable_publish_count_
           << " no_progress_timeout_s=" << undock_no_progress_timeout_s_;
    if (undock_phase_ == UndockPhase::UNDOCK_ACTIVE && last_undock_progress_time_.nanoseconds() > 0) {
      status << " last_progress_age=" << std::setprecision(2)
             << (stamp - last_undock_progress_time_).seconds();
    }
    publish_status(status.str());
  }

  std::string undock_status_prefix(const double traveled) const
  {
    if (undock_phase_ == UndockPhase::UNDOCK_WAIT_FIRST_MOTION) {
      return "undocking waiting_first_motion";
    }
    if (traveled >= undock_min_clear_distance_m_) {
      return "undocking clear_distance_reached";
    }
    return "undocking active backing_out";
  }

  double elapsed_since_or_negative(const rclcpp::Time & reference, const rclcpp::Time & stamp) const
  {
    if (reference.nanoseconds() <= 0) {
      return -1.0;
    }
    return std::max(0.0, (stamp - reference).seconds());
  }

  std::string bool_text(const bool value) const
  {
    return value ? "true" : "false";
  }

  std::string undock_running_status(
    const std::string & phase, const rclcpp::Time & stamp, const double traveled, const double distance) const
  {
    std::ostringstream status;
    status << "undocking " << phase
           << " phase=" << phase
           << " distance=" << std::fixed << std::setprecision(3) << traveled << "/" << distance
           << " cmd_x=" << last_undock_cmd_x_
           << " cmd_count=" << undock_nonzero_cmd_publish_count_
           << " reverse_enable=" << bool_text(last_undock_reverse_enable_)
           << " reverse_enable_count=" << undock_reverse_enable_publish_count_
           << " last_cmd_stamp_age_s=" << std::setprecision(2)
           << elapsed_since_or_negative(last_undock_cmd_publish_time_, stamp)
           << " command_start_elapsed_s="
           << elapsed_since_or_negative(undock_nonzero_cmd_start_time_, stamp)
           << " first_motion_started=" << bool_text(have_undock_first_motion_);
    return status.str();
  }

  std::string undock_failure_status(
    const std::string & failure_reason,
    const double traveled,
    const double distance,
    const rclcpp::Time & stamp) const
  {
    std::ostringstream status;
    status << failure_reason
           << " phase=failed"
           << " failure_reason=" << failure_reason
           << " distance=" << std::fixed << std::setprecision(3) << traveled << "/" << distance
           << " cmd_count=" << undock_nonzero_cmd_publish_count_
           << " reverse_enable_count=" << undock_reverse_enable_publish_count_
           << " last_cmd_x=" << last_undock_cmd_x_
           << " cmd_x=" << last_undock_cmd_x_
           << " last_cmd_stamp_age_s=" << std::setprecision(2)
           << elapsed_since_or_negative(last_undock_cmd_publish_time_, stamp)
           << " last_cmd_age=" << elapsed_since_or_negative(last_undock_cmd_publish_time_, stamp)
           << " command_start_elapsed_s="
           << elapsed_since_or_negative(undock_nonzero_cmd_start_time_, stamp)
           << " motion_start_timeout_s=" << undock_motion_start_timeout_s_
           << " first_motion_started=" << bool_text(have_undock_first_motion_);
    return status.str();
  }

  void finish_undock_success(const double final_distance)
  {
    const auto stamp = now();
    const auto cmd_count = undock_nonzero_cmd_publish_count_;
    const auto reverse_enable_count = undock_reverse_enable_publish_count_;
    const double cmd_x = last_undock_cmd_x_;
    const double last_cmd_age = elapsed_since_or_negative(last_undock_cmd_publish_time_, stamp);
    const double command_elapsed = elapsed_since_or_negative(undock_nonzero_cmd_start_time_, stamp);
    const bool first_motion_started = have_undock_first_motion_;
    reset_undock_tracking();
    undock_phase_ = UndockPhase::UNDOCK_SUCCEEDED;
    state_ = State::Idle;
    publish_zero();
    publish_reverse_enable(false);
    release_docking_motion_mode(false);
    update_dock_contact_latch(false, "docking_manager", "undocked", "");
    std::ostringstream status;
    status << "undocked phase=succeeded failure_reason=none distance=" << std::fixed
           << std::setprecision(3) << final_distance
           << " cmd_count=" << cmd_count
           << " reverse_enable_count=" << reverse_enable_count
           << " last_cmd_x=" << cmd_x
           << " cmd_x=" << cmd_x
           << " last_cmd_stamp_age_s=" << std::setprecision(2) << last_cmd_age
           << " command_start_elapsed_s=" << command_elapsed
           << " motion_start_timeout_s=" << undock_motion_start_timeout_s_
           << " first_motion_started=" << bool_text(first_motion_started);
    publish_status(status.str());
  }

  void publish_undock_reverse_command(const double speed, const rclcpp::Time & stamp)
  {
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = -speed;
    publish_reverse_enable(true);
    publish_cmd(cmd);
    last_undock_cmd_x_ = cmd.linear.x;
    last_undock_cmd_publish_time_ = stamp;
    ++undock_nonzero_cmd_publish_count_;
    if (undock_nonzero_cmd_start_time_.nanoseconds() == 0) {
      undock_nonzero_cmd_start_time_ = stamp;
    }
  }

  void fail_undock(const UndockPhase phase, const std::string & reason)
  {
    reset_undock_tracking();
    undock_phase_ = phase;
    state_ = State::Failed;
    publish_zero();
    publish_reverse_enable(false);
    release_docking_motion_mode(false);
    publish_status(reason);
  }

  bool odom_fresh() const
  {
    if (!latest_odom_) {
      return false;
    }
    return (now() - last_odom_time_).seconds() <= undock_odom_timeout_s_;
  }

  bool capture_undock_start_odom()
  {
    if (!odom_fresh()) {
      return false;
    }
    const auto & position = latest_odom_->pose.pose.position;
    undock_start_x_ = position.x;
    undock_start_y_ = position.y;
    undock_max_progress_m_ = 0.0;
    have_undock_start_odom_ = true;
    return true;
  }

  double undock_traveled_m() const
  {
    if (!have_undock_start_odom_ || !latest_odom_) {
      return 0.0;
    }
    const auto & position = latest_odom_->pose.pose.position;
    return std::hypot(position.x - undock_start_x_, position.y - undock_start_y_);
  }

  bool capture_contact_start_odom()
  {
    if (!odom_fresh()) {
      return false;
    }
    const auto & position = latest_odom_->pose.pose.position;
    contact_start_x_ = position.x;
    contact_start_y_ = position.y;
    have_contact_start_odom_ = true;
    return true;
  }

  double contact_verify_traveled_m() const
  {
    if (!have_contact_start_odom_ || !latest_odom_) {
      return 0.0;
    }
    const auto & position = latest_odom_->pose.pose.position;
    return std::hypot(position.x - contact_start_x_, position.y - contact_start_y_);
  }

  void reset_contact_tracking()
  {
    have_contact_start_odom_ = false;
    contact_start_x_ = 0.0;
    contact_start_y_ = 0.0;
  }

  bool capture_contact_backoff_start_odom()
  {
    if (!odom_fresh()) {
      return false;
    }
    const auto & pose = latest_odom_->pose.pose;
    contact_backoff_start_x_ = pose.position.x;
    contact_backoff_start_y_ = pose.position.y;
    const auto & q = pose.orientation;
    const double sin_yaw = 2.0 * (q.w * q.z + q.x * q.y);
    const double cos_yaw = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    contact_backoff_start_yaw_ = std::atan2(sin_yaw, cos_yaw);
    have_contact_backoff_start_odom_ = true;
    return true;
  }

  double contact_backoff_traveled_m() const
  {
    if (!have_contact_backoff_start_odom_ || !latest_odom_) {
      return 0.0;
    }
    const auto & position = latest_odom_->pose.pose.position;
    const double dx = position.x - contact_backoff_start_x_;
    const double dy = position.y - contact_backoff_start_y_;
    const double forward =
      std::cos(contact_backoff_start_yaw_) * dx + std::sin(contact_backoff_start_yaw_) * dy;
    return std::max(0.0, -forward);
  }

  double contact_backoff_lateral_m() const
  {
    if (!have_contact_backoff_start_odom_ || !latest_odom_) {
      return 0.0;
    }
    const auto & position = latest_odom_->pose.pose.position;
    const double dx = position.x - contact_backoff_start_x_;
    const double dy = position.y - contact_backoff_start_y_;
    return -std::sin(contact_backoff_start_yaw_) * dx +
      std::cos(contact_backoff_start_yaw_) * dy;
  }

  void reset_contact_backoff_tracking()
  {
    have_contact_backoff_start_odom_ = false;
    have_contact_backoff_first_motion_ = false;
    contact_backoff_start_x_ = 0.0;
    contact_backoff_start_y_ = 0.0;
    contact_backoff_start_yaw_ = 0.0;
    contact_backoff_max_progress_m_ = 0.0;
    contact_backoff_cmd_count_ = 0U;
    contact_backoff_nonzero_cmd_start_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    last_contact_backoff_progress_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  }

  void reset_contact_stop_tracking()
  {
    contact_stop_success_status_.clear();
    contact_stop_bms_reason_.clear();
    contact_stop_start_wheel_odom_sequence_ = 0;
    contact_stop_start_motion_state_sequence_ = 0;
    contact_stop_last_evaluated_wheel_sequence_ = 0;
    contact_stop_stable_samples_ = 0;
    contact_stop_zero_cmd_count_ = 0U;
    contact_stop_contact_verify_traveled_m_ = -1.0;
    contact_stop_contact_verify_elapsed_s_ = -1.0;
    contact_stop_have_wheel_pose_at_bms_ = false;
    contact_stop_wheel_x_at_bms_ = 0.0;
    contact_stop_wheel_y_at_bms_ = 0.0;
    contact_stop_bms_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    contact_stop_first_zero_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    contact_stop_last_zero_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    contact_stop_first_stable_sample_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  }

  void reset_undock_tracking()
  {
    have_undock_start_odom_ = false;
    undock_start_x_ = 0.0;
    undock_start_y_ = 0.0;
    undock_max_progress_m_ = 0.0;
    have_undock_first_motion_ = false;
    undock_phase_ = UndockPhase::UNDOCK_IDLE;
    undock_nonzero_cmd_publish_count_ = 0U;
    undock_reverse_enable_publish_count_ = 0U;
    last_undock_cmd_x_ = 0.0;
    last_undock_reverse_enable_ = false;
    undock_nonzero_cmd_start_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    last_undock_cmd_publish_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    first_undock_motion_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    last_undock_progress_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  }

  bool battery_indicates_charging(const sensor_msgs::msg::BatteryState & msg) const
  {
    return battery_charging_contact(msg).contact;
  }

  bool battery_indicates_charging_contact(const sensor_msgs::msg::BatteryState & msg) const
  {
    return battery_charging_contact(msg).contact;
  }

  BatteryContactEvaluation battery_charging_contact(
    const sensor_msgs::msg::BatteryState & msg, const bool received_sample = false) const
  {
    if (msg.power_supply_status == sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_CHARGING) {
      return {true, "power_supply_status=CHARGING"};
    }
    if (msg.power_supply_status == sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_FULL) {
      const bool session_supports_full_contact =
        docking_is_active() || state_ == State::Docked || dock_contact_latch_is_docked();
      return session_supports_full_contact ?
        BatteryContactEvaluation{true, "power_supply_status=FULL_with_docking_session"} :
        BatteryContactEvaluation{false, "full_without_physical_contact_evidence"};
    }
    if (std::isfinite(msg.current) && static_cast<double>(msg.current) > min_charging_current_a_ &&
      ((received_sample && docking_is_active()) || state_ == State::Docked ||
      dock_contact_latch_is_docked() || safety_memory_allows_undock())) {
      return {true, "current_above_threshold"};
    }
    if (msg.present && voltage_in_contact_range(
      msg.voltage, charging_contact_voltage_min_v_, charging_contact_voltage_max_v_)) {
      return {true, "present_voltage_valid"};
    }
    const double soc = normalized_soc_percent(msg.percentage);
    if (charging_full_soc_voltage_contact_enable_ &&
      (docking_is_active() || state_ == State::Docked || dock_contact_latch_is_docked()) &&
      msg.present && std::isfinite(soc) &&
      soc >= charging_full_soc_threshold_pct_ &&
      voltage_in_contact_range(msg.voltage, charging_contact_voltage_min_v_, charging_contact_voltage_max_v_)) {
      return {true, "full_soc_present_voltage_valid"};
    }
    return {false, "no_contact"};
  }

  bool docking_is_active() const
  {
    return state_ == State::BlindApproach || state_ == State::Acquire ||
      state_ == State::Align || state_ == State::ContactVerify ||
      state_ == State::ContactBackoff || state_ == State::ContactStopping;
  }

  void finalize_docked_stop(const std::string & status, const rclcpp::Time & stamp)
  {
    publish_contact_stop_zero(stamp);
    const double bms_to_first_zero_ms = contact_stop_elapsed_ms(
      contact_stop_bms_time_, contact_stop_first_zero_time_);
    const double first_zero_to_stop_ms = contact_stop_elapsed_ms(
      contact_stop_first_zero_time_, contact_stop_first_stable_sample_time_);
    const double first_zero_to_stop_confirmed_ms = contact_stop_elapsed_ms(
      contact_stop_first_zero_time_, stamp);
    const double bms_to_stop_ms = contact_stop_elapsed_ms(
      contact_stop_bms_time_, contact_stop_first_stable_sample_time_);
    const double bms_to_stop_confirmed_ms = contact_stop_elapsed_ms(
      contact_stop_bms_time_, stamp);
    const double linear_speed = latest_wheel_odom_ ? std::hypot(
      latest_wheel_odom_->twist.twist.linear.x,
      latest_wheel_odom_->twist.twist.linear.y) : std::numeric_limits<double>::quiet_NaN();
    const double angular_speed = latest_wheel_odom_ ?
      std::abs(latest_wheel_odom_->twist.twist.angular.z) :
      std::numeric_limits<double>::quiet_NaN();
    const int motion_mode = latest_motion_state_ ?
      static_cast<int>(latest_motion_state_->motion_mode) : -1;

    reset_contact_tracking();
    reset_contact_backoff_tracking();
    state_ = State::Docked;
    publish_reverse_enable(false);
    release_docking_motion_mode(park_on_docked_);
    update_dock_contact_latch(true, "docking_manager", status, "");

    std::ostringstream result;
    result << status
           << " brake_confirmed=true"
           << " bms_reason=" << contact_stop_bms_reason_
           << " bms_rx_ns=" << contact_stop_bms_time_.nanoseconds()
           << " first_zero_ns=" << contact_stop_first_zero_time_.nanoseconds()
           << " stop_observed_ns=" << contact_stop_first_stable_sample_time_.nanoseconds()
           << " stop_confirmed_ns=" << stamp.nanoseconds()
           << " bms_to_first_zero_ms=" << std::fixed << std::setprecision(3)
           << bms_to_first_zero_ms
           << " first_zero_to_stop_ms=" << first_zero_to_stop_ms
           << " first_zero_to_stop_confirmed_ms=" << first_zero_to_stop_confirmed_ms
           << " bms_to_stop_ms=" << bms_to_stop_ms
           << " bms_to_stop_confirmed_ms=" << bms_to_stop_confirmed_ms
           << " contact_verify_traveled_at_bms_m=" << contact_stop_contact_verify_traveled_m_
           << " contact_verify_elapsed_at_bms_s=" << contact_stop_contact_verify_elapsed_s_
           << " post_bms_distance_m=" << contact_stop_post_bms_distance_m()
           << " zero_cmd_count=" << contact_stop_zero_cmd_count_
           << " wheel_linear_speed_mps=" << linear_speed
           << " wheel_angular_speed_radps=" << angular_speed
           << " motion_mode=" << motion_mode
           << " stable_samples=" << contact_stop_stable_samples_
           << "/" << contact_stop_stable_samples_required_
           << " park_after_stop=" << bool_text(park_on_docked_);
    RCLCPP_INFO(get_logger(), "DOCK_BRAKE_CONFIRMED %s", result.str().c_str());
    publish_status(result.str());
  }

  bool dock_contact_latch_is_docked() const
  {
    if (docking_contact_latch_file_.empty()) {
      return false;
    }
    std::ifstream file(docking_contact_latch_file_);
    if (!file) {
      return false;
    }
    std::ostringstream data;
    data << file.rdbuf();
    const auto text = data.str();
    return text.find("\"latched_docked\": true") != std::string::npos ||
      text.find("\"latched_docked\":true") != std::string::npos ||
      text.find("\"docked\": true") != std::string::npos ||
      text.find("\"docked\":true") != std::string::npos;
  }

  void update_dock_contact_latch(
    const bool docked,
    const std::string & source,
    const std::string & reason,
    const std::string & dock_id) const
  {
    if (docking_contact_latch_file_.empty()) {
      return;
    }
    if (have_last_dock_contact_latch_write_ &&
      docked == last_dock_contact_latch_docked_ &&
      source == last_dock_contact_latch_source_ &&
      reason == last_dock_contact_latch_reason_ &&
      dock_id == last_dock_contact_latch_dock_id_)
    {
      return;
    }
    try {
      const auto path = std::filesystem::path(docking_contact_latch_file_);
      std::filesystem::create_directories(path.parent_path());
      const auto tmp = path.string() + ".tmp";
      std::ofstream file(tmp);
      if (!file) {
        return;
      }
      const auto stamp = std::to_string(now().seconds());
      file << "{\n"
           << "  \"schema\": \"njrh.docking_contact_latch.v1\",\n"
           << "  \"latched_docked\": " << (docked ? "true" : "false") << ",\n"
           << "  \"docked\": " << (docked ? "true" : "false") << ",\n"
           << "  \"source\": \"" << source << "\",\n"
           << "  \"reason\": \"" << reason << "\",\n"
           << "  \"building_id\": \"\",\n"
           << "  \"floor_id\": \"\",\n"
           << "  \"map_id\": \"\",\n"
           << "  \"dock_id\": \"" << dock_id << "\",\n"
           << "  \"latched_at\": \"" << (docked ? stamp : "") << "\",\n"
           << "  \"last_confirmed_at\": \"" << (docked ? stamp : "") << "\",\n"
           << "  \"cleared_at\": \"" << (docked ? "" : stamp) << "\",\n"
           << "  \"clear_reason\": \"" << (docked ? "" : reason) << "\",\n"
           << "  \"note\": \"\",\n"
           << "  \"updated_at\": \"" << stamp << "\"\n"
           << "}\n";
      file.close();
      std::filesystem::rename(tmp, path);
      have_last_dock_contact_latch_write_ = true;
      last_dock_contact_latch_docked_ = docked;
      last_dock_contact_latch_source_ = source;
      last_dock_contact_latch_reason_ = reason;
      last_dock_contact_latch_dock_id_ = dock_id;
    } catch (...) {
      return;
    }
  }

  void enter_docking_motion_mode() const
  {
    publish_reverse_enable(false);
    publish_park(false);
    if (use_crab_mode_) {
      publish_forced_mode(crab_forced_mode_);
    }
  }

  void release_docking_motion_mode(bool park) const
  {
    publish_reverse_enable(false);
    if (park) {
      publish_forced_mode("park");
      publish_park(true);
    } else {
      publish_park(false);
      publish_forced_mode(release_forced_mode_);
    }
  }

  void publish_forced_mode(const std::string & mode) const
  {
    if (mode == last_forced_mode_request_) {
      return;
    }
    std_msgs::msg::String msg;
    msg.data = mode;
    forced_mode_pub_->publish(msg);
    last_forced_mode_request_ = mode;
  }

  void publish_park(bool park) const
  {
    std_msgs::msg::Bool msg;
    msg.data = park;
    park_pub_->publish(msg);
  }

  void publish_reverse_enable(bool enabled) const
  {
    std_msgs::msg::Bool msg;
    msg.data = enabled;
    reverse_enable_pub_->publish(msg);
    if (state_ == State::Undocking) {
      last_undock_reverse_enable_ = enabled;
      if (enabled) {
        ++undock_reverse_enable_publish_count_;
      }
    }
  }

  std::string detection_status(const std::string & state, const Detection & detection) const
  {
    std::ostringstream out;
    out << state
        << " points=" << detection.points
        << " x=" << std::fixed << std::setprecision(3) << detection.distance_x
        << " y=" << detection.lateral_y
        << " yaw_deg=" << detection.yaw_error * 180.0 / kPi
        << " confidence=" << detection.confidence;
    return out.str();
  }

  void publish_status(const std::string & text) const
  {
    std_msgs::msg::String msg;
    msg.data = text;
    status_pub_->publish(msg);
  }

  void publish_zero() const
  {
    geometry_msgs::msg::Twist cmd;
    publish_cmd(cmd);
  }

  void publish_cmd(const geometry_msgs::msg::Twist & cmd) const
  {
    cmd_pub_->publish(cmd);
  }

  std::string base_frame_{"base_link"};
  std::string observation_backend_{"target_observation"};
  std::string gs2_scan_topic_;
  std::string target_observation_topic_{"/dock/target_observation"};
  std::string target_observation_source_{"orbbec_336l_depth"};
  std::string cmd_vel_topic_;
  std::string status_topic_;
  std::string start_service_;
  std::string stop_service_;
  std::string undock_service_;
  std::string charging_state_topic_;
  std::string dock_interlock_state_topic_;
  double dock_interlock_max_age_sec_{1.0};
  std::string docking_contact_latch_file_;
  mutable bool have_last_dock_contact_latch_write_{false};
  mutable bool last_dock_contact_latch_docked_{false};
  mutable std::string last_dock_contact_latch_source_;
  mutable std::string last_dock_contact_latch_reason_;
  mutable std::string last_dock_contact_latch_dock_id_;
  mutable std::string last_forced_mode_request_;
  std::string undock_odom_topic_{"/local_state/odometry"};
  std::string contact_stop_motion_state_topic_{"/motion_state"};
  std::string contact_stop_wheel_odom_topic_{"/wheel/odom"};
  std::string contact_stop_success_status_;
  std::string contact_stop_bms_reason_;
  std::string forced_mode_topic_;
  std::string park_topic_;
  std::string reverse_enable_topic_;
  std::string crab_forced_mode_{"side_slip"};
  std::string yaw_forced_mode_{"spinning"};
  std::string release_forced_mode_{"auto"};
  bool use_crab_mode_{true};
  bool park_on_docked_{true};
  bool allow_blind_approach_{true};

  double gs2_x_m_{0.360};
  double charge_contact_x_m_{0.398};
  double gs2_to_contact_x_m_{0.038};
  double blind_approach_max_distance_m_{0.50};
  double blind_approach_speed_mps_{0.06};
  double gs2_acquire_distance_m_{0.28};
  double final_target_distance_m_{0.05};
  double undock_distance_m_{0.60};
  double undock_speed_mps_{0.50};
  double undock_max_speed_mps_{0.50};
  double undock_min_clear_distance_m_{0.45};
  double undock_timeout_s_{12.0};
  double undock_odom_timeout_s_{0.50};
  double undock_odom_start_timeout_s_{2.0};
  double undock_command_settle_s_{0.5};
  double undock_motion_start_timeout_s_{6.0};
  double undock_no_progress_timeout_s_{2.0};
  double undock_progress_epsilon_m_{0.005};
  double contact_stop_feedback_max_age_s_{0.50};
  double contact_stop_linear_speed_threshold_mps_{0.01};
  double contact_stop_angular_speed_threshold_radps_{0.02};
  double contact_stop_stable_duration_s_{0.50};
  int contact_stop_stable_samples_required_{5};
  double contact_stop_feedback_timeout_s_{3.0};
  double lateral_soft_limit_m_{0.030};
  double lateral_hard_limit_m_{0.050};
  double yaw_soft_limit_rad_{deg_to_rad(2.0)};
  double yaw_hard_limit_rad_{deg_to_rad(4.0)};
  double contact_confirm_timeout_s_{3.0};
  double max_linear_speed_mps_{0.15};
  double max_angular_speed_radps_{0.25};
  int max_retries_{3};
  int command_timeout_ms_{300};
  double control_rate_hz_{20.0};
  int detector_min_points_{8};
  double detector_min_span_m_{0.045};
  double detector_lateral_gate_m_{0.20};
  double detector_max_range_m_{0.30};
  double detector_min_range_m_{0.025};
  double detector_front_cluster_x_window_m_{0.015};
  double detector_min_confidence_{0.10};
  double detector_yaw_fit_min_lateral_span_m_{0.055};
  int detector_stable_frames_required_{3};
  double detection_filter_alpha_{0.25};
  bool use_yaw_fit_{false};
  double kx_{0.45};
  double ky_lateral_{0.70};
  double lateral_command_sign_{-1.0};
  double kyaw_{0.0};
  double lateral_deadband_m_{0.010};
  double yaw_deadband_rad_{deg_to_rad(1.0)};
  double min_align_speed_mps_{0.025};
  double min_angular_speed_radps_{0.05};
  double min_lateral_speed_mps_{0.025};
  double max_lateral_speed_mps_{0.04};
  double yaw_realign_enter_rad_{deg_to_rad(1.0)};
  int yaw_realign_stable_frames_required_{3};
  int yaw_realign_max_count_{1};
  double max_parallel_speed_mps_{0.15};
  double final_approach_window_m_{0.10};
  double final_lateral_lock_distance_m_{0.06};
  double final_forward_speed_mps_{0.05};
  bool lock_lateral_during_final_insert_{true};
  double contact_crawl_speed_mps_{0.05};
  double contact_final_slow_zone_m_{0.06};
  double contact_final_crawl_speed_mps_{0.02};
  double contact_timeout_safety_factor_{1.5};
  double contact_timeout_margin_s_{2.0};
  double contact_timeout_min_s_{3.0};
  double contact_verify_max_distance_m_{0.12};
  bool contact_verify_retry_enabled_{true};
  int contact_retry_max_count_{2};
  double contact_retry_backoff_distance_m_{0.60};
  double contact_retry_backoff_min_distance_m_{0.20};
  double contact_retry_backoff_clearance_margin_m_{0.08};
  double contact_retry_backoff_speed_mps_{0.06};
  double contact_retry_backoff_timeout_s_{20.0};
  double contact_retry_backoff_command_settle_s_{0.5};
  double contact_retry_backoff_motion_start_timeout_s_{6.0};
  double contact_retry_backoff_no_progress_timeout_s_{2.0};
  double contact_retry_backoff_progress_epsilon_m_{0.005};
  double contact_retry_backoff_max_lateral_drift_m_{0.05};
  double min_charging_current_a_{0.10};
  double charging_contact_voltage_min_v_{40.0};
  double charging_contact_voltage_max_v_{1000.0};
  double charging_full_soc_threshold_pct_{99.0};
  bool charging_full_soc_voltage_contact_enable_{true};

  State state_{State::Idle};
  int retries_{0};
  std::uint64_t observation_sequence_{0};
  int contact_retry_count_{0};
  double active_contact_timeout_s_{3.0};
  double active_contact_backoff_distance_m_{0.20};
  int valid_detection_streak_{0};
  int contact_stop_stable_samples_{0};
  bool charging_detected_{false};
  bool charging_contact_detected_{false};
  bool has_filtered_detection_{false};
  bool have_undock_start_odom_{false};
  bool have_contact_start_odom_{false};
  bool have_contact_backoff_start_odom_{false};
  bool have_contact_backoff_first_motion_{false};
  bool have_undock_first_motion_{false};
  UndockPhase undock_phase_{UndockPhase::UNDOCK_IDLE};
  double undock_start_x_{0.0};
  double undock_start_y_{0.0};
  double contact_start_x_{0.0};
  double contact_start_y_{0.0};
  double contact_backoff_start_x_{0.0};
  double contact_backoff_start_y_{0.0};
  double contact_backoff_start_yaw_{0.0};
  double contact_backoff_max_progress_m_{0.0};
  size_t contact_backoff_cmd_count_{0U};
  size_t contact_stop_zero_cmd_count_{0U};
  double contact_stop_contact_verify_traveled_m_{-1.0};
  double contact_stop_contact_verify_elapsed_s_{-1.0};
  bool contact_stop_have_wheel_pose_at_bms_{false};
  double contact_stop_wheel_x_at_bms_{0.0};
  double contact_stop_wheel_y_at_bms_{0.0};
  double undock_max_progress_m_{0.0};
  double last_undock_cmd_x_{0.0};
  size_t undock_nonzero_cmd_publish_count_{0U};
  std::uint64_t wheel_odom_sequence_{0};
  std::uint64_t motion_state_sequence_{0};
  std::uint64_t contact_stop_start_wheel_odom_sequence_{0};
  std::uint64_t contact_stop_start_motion_state_sequence_{0};
  std::uint64_t contact_stop_last_evaluated_wheel_sequence_{0};
  mutable size_t undock_reverse_enable_publish_count_{0U};
  mutable bool last_undock_reverse_enable_{false};
  Detection filtered_detection_;
  rclcpp::Time state_entered_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_scan_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_dock_interlock_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_target_observation_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_odom_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_wheel_odom_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_motion_state_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time contact_stop_bms_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time contact_stop_first_zero_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time contact_stop_last_zero_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time contact_stop_first_stable_sample_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time undock_nonzero_cmd_start_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_undock_cmd_publish_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time first_undock_motion_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_undock_progress_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time contact_backoff_nonzero_cmd_start_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_contact_backoff_progress_time_{0, 0, RCL_ROS_TIME};
  sensor_msgs::msg::LaserScan::SharedPtr latest_scan_;
  robot_interfaces::msg::DockTargetObservation::SharedPtr latest_target_observation_;
  sensor_msgs::msg::BatteryState::SharedPtr latest_battery_;
  robot_interfaces::msg::DockSafetyInterlockState::SharedPtr latest_dock_interlock_;
  nav_msgs::msg::Odometry::SharedPtr latest_odom_;
  nav_msgs::msg::Odometry::SharedPtr latest_wheel_odom_;
  ranger_msgs::msg::MotionState::SharedPtr latest_motion_state_;
  std::unique_ptr<robot_docking_manager::NearFieldDockingController> near_field_controller_;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<robot_interfaces::msg::DockTargetObservation>::SharedPtr
    target_observation_sub_;
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub_;
  rclcpp::Subscription<robot_interfaces::msg::DockSafetyInterlockState>::SharedPtr dock_interlock_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr wheel_odom_sub_;
  rclcpp::Subscription<ranger_msgs::msg::MotionState>::SharedPtr motion_state_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr forced_mode_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr park_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr reverse_enable_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr undock_srv_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DockingManagerNode>());
  rclcpp::shutdown();
  return 0;
}
