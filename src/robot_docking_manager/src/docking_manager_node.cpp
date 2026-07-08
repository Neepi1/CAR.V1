#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <memory>
#include <numeric>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
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

    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      gs2_scan_topic_, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::LaserScan::SharedPtr msg) {
        latest_scan_ = std::move(msg);
        last_scan_time_ = now();
      });

    battery_sub_ = create_subscription<sensor_msgs::msg::BatteryState>(
      charging_state_topic_, rclcpp::QoS(10),
      [this](const sensor_msgs::msg::BatteryState::SharedPtr msg) {
        latest_battery_ = msg;
        charging_detected_ = battery_indicates_charging(*msg);
        charging_contact_detected_ = battery_indicates_charging_contact(*msg);
        if (charging_contact_detected_) {
          update_dock_contact_latch(true, "charging_session", "bms_charging_observed", "");
        }
        if (charging_detected_ && docking_is_active()) {
          docked_stop("docked_charging_detected");
        }
      });

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      undock_odom_topic_, rclcpp::QoS(20),
      [this](nav_msgs::msg::Odometry::SharedPtr msg) {
        latest_odom_ = std::move(msg);
        last_odom_time_ = now();
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
  }

private:
  enum class State
  {
    Idle,
    BlindApproach,
    Acquire,
    Align,
    ContactVerify,
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
    int points{0};
    double distance_x{0.0};
    double lateral_y{0.0};
    double yaw_error{0.0};
    double lateral_span{0.0};
    double confidence{0.0};
  };

  void load_parameters()
  {
    gs2_scan_topic_ = declare_parameter<std::string>("gs2_scan_topic", "/dock/gs2_scan");
    cmd_vel_topic_ = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel_docking");
    status_topic_ = declare_parameter<std::string>("status_topic", "/docking/status");
    start_service_ = declare_parameter<std::string>("start_service", "/docking/start");
    stop_service_ = declare_parameter<std::string>("stop_service", "/docking/stop");
    undock_service_ = declare_parameter<std::string>("undock_service", "/docking/undock");
    charging_state_topic_ = declare_parameter<std::string>("charging_state_topic", "/battery_state");
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
    gs2_acquire_distance_m_ = declare_parameter<double>("approach.gs2_acquire_distance_m", 0.28);
    final_target_distance_m_ = declare_parameter<double>("approach.final_target_distance_m", 0.05);
    undock_distance_m_ = declare_parameter<double>("undock.distance_m", 0.60);
    undock_speed_mps_ = declare_parameter<double>("undock.speed_mps", 0.06);
    undock_min_clear_distance_m_ = declare_parameter<double>("undock.min_clear_distance_m", 0.45);
    undock_timeout_s_ = declare_parameter<double>("undock.timeout_s", 12.0);
    undock_odom_topic_ = declare_parameter<std::string>("undock.odom_topic", "/local_state/odometry");
    undock_odom_timeout_s_ = declare_parameter<double>("undock.odom_timeout_s", 0.50);
    undock_odom_start_timeout_s_ = declare_parameter<double>("undock.odom_start_timeout_s", 2.0);
    undock_command_settle_s_ = declare_parameter<double>("undock.command_settle_s", 0.5);
    undock_motion_start_timeout_s_ = declare_parameter<double>("undock.motion_start_timeout_s", 6.0);
    undock_no_progress_timeout_s_ = declare_parameter<double>("undock.no_progress_timeout_s", 2.0);
    undock_progress_epsilon_m_ = declare_parameter<double>("undock.progress_epsilon_m", 0.005);

    lateral_soft_limit_m_ = declare_parameter<double>("tolerances.lateral_soft_limit_m", 0.030);
    lateral_hard_limit_m_ = declare_parameter<double>("tolerances.lateral_hard_limit_m", 0.050);
    yaw_soft_limit_rad_ = deg_to_rad(declare_parameter<double>("tolerances.yaw_soft_limit_deg", 2.0));
    yaw_hard_limit_rad_ = deg_to_rad(declare_parameter<double>("tolerances.yaw_hard_limit_deg", 4.0));
    contact_confirm_timeout_s_ = declare_parameter<double>("tolerances.contact_confirm_timeout_s", 3.0);

    max_linear_speed_mps_ = declare_parameter<double>("safety.max_linear_speed_mps", 0.08);
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
    ky_ = declare_parameter<double>("controller.ky", 0.55);
    ky_lateral_ = declare_parameter<double>("controller.ky_lateral", 0.70);
    lateral_command_sign_ = declare_parameter<double>("controller.lateral_command_sign", -1.0);
    kyaw_ = declare_parameter<double>("controller.kyaw", 0.0);
    lateral_deadband_m_ = declare_parameter<double>("controller.lateral_deadband_m", 0.010);
    yaw_deadband_rad_ = deg_to_rad(declare_parameter<double>("controller.yaw_deadband_deg", 1.0));
    min_align_speed_mps_ = declare_parameter<double>("controller.min_align_speed_mps", 0.025);
    min_lateral_speed_mps_ = declare_parameter<double>("controller.min_lateral_speed_mps", 0.025);
    max_lateral_speed_mps_ = declare_parameter<double>("controller.max_lateral_speed_mps", 0.04);
    lateral_priority_threshold_m_ = declare_parameter<double>("controller.lateral_priority_threshold_m", 0.020);
    yaw_priority_threshold_rad_ = deg_to_rad(declare_parameter<double>("controller.yaw_priority_threshold_deg", 2.0));
    max_forward_while_lateral_mps_ = declare_parameter<double>("controller.max_forward_while_lateral_mps", 0.020);
    lock_lateral_during_final_insert_ =
      declare_parameter<bool>("controller.lock_lateral_during_final_insert", true);
    yaw_spin_priority_enabled_ =
      declare_parameter<bool>("controller.yaw_spin_priority_enabled", true);
    max_command_steering_rad_ = declare_parameter<double>("controller.max_command_steering_rad", 0.35);
    ackermann_wheelbase_m_ = declare_parameter<double>("controller.ackermann_wheelbase_m", 0.494);
    contact_crawl_speed_mps_ = declare_parameter<double>("controller.contact_crawl_speed_mps", 0.025);
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
  }

  void start_docking()
  {
    reset_undock_tracking();
    retries_ = 0;
    charging_detected_ = latest_battery_ && battery_indicates_charging(*latest_battery_);
    charging_contact_detected_ = latest_battery_ && battery_indicates_charging_contact(*latest_battery_);
    valid_detection_streak_ = 0;
    has_filtered_detection_ = false;
    if (charging_detected_) {
      state_ = State::Docked;
      state_entered_time_ = now();
      docked_stop("docked_charging_already_detected");
      return;
    }
    enter_docking_motion_mode();
    state_ = State::BlindApproach;
    state_entered_time_ = now();
    publish_status("blind_approach");
  }

  void stop_docking(const std::string & reason)
  {
    reset_undock_tracking();
    state_ = State::Idle;
    publish_zero();
    publish_reverse_enable(false);
    release_docking_motion_mode(false);
    publish_status(reason);
  }

  void fail(const std::string & reason)
  {
    reset_undock_tracking();
    state_ = State::Failed;
    publish_zero();
    publish_reverse_enable(false);
    release_docking_motion_mode(false);
    publish_status(reason);
  }

  bool start_undocking(std::string & message)
  {
    charging_detected_ = latest_battery_ && battery_indicates_charging(*latest_battery_);
    charging_contact_detected_ = latest_battery_ && battery_indicates_charging_contact(*latest_battery_);
    const bool dock_latch_detected = dock_contact_latch_is_docked();
    if (state_ == State::Undocking) {
      message = "undocking already active";
      return true;
    }
    if (state_ != State::Docked && !charging_contact_detected_ && !dock_latch_detected) {
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
    message = "undocking started";
    publish_status("undocking preparing phase=preparing cmd_count=0 reverse_enable=true reverse_enable_count=1");
    return true;
  }

  void transition(State next, const std::string & status)
  {
    state_ = next;
    state_entered_time_ = now();
    publish_status(status);
  }

  void control_step()
  {
    if (state_ == State::Undocking) {
      handle_undocking();
      return;
    }

    if (state_ == State::Idle || state_ == State::Docked || state_ == State::Failed) {
      return;
    }

    if (charging_detected_) {
      docked_stop("docked_charging_detected");
      return;
    }

    if (!scan_fresh()) {
      publish_zero();
      publish_status("waiting_for_fresh_gs2_scan");
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
        handle_contact_verify();
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

  bool scan_fresh() const
  {
    if (!latest_scan_) {
      return false;
    }
    const double age_ms = (now() - last_scan_time_).seconds() * 1000.0;
    return age_ms <= static_cast<double>(command_timeout_ms_);
  }

  Detection detect_dock() const
  {
    Detection detection;
    if (!latest_scan_) {
      return detection;
    }

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
      transition(State::Acquire, "blind_approach_complete_waiting_for_gs2_feature");
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
      transition(State::BlindApproach, "retry_blind_approach");
      return;
    }
    publish_zero();
  }

  void handle_align(const Detection & detection)
  {
    if (!detection.valid) {
      transition(State::Acquire, "lost_dock_feature");
      return;
    }

    const bool lateral_ok = std::abs(detection.lateral_y) <= lateral_soft_limit_m_;
    const bool yaw_ok = std::abs(detection.yaw_error) <= yaw_soft_limit_rad_;
    const bool distance_ok = detection.distance_x <= final_target_distance_m_ + 0.015;
    if (lateral_ok && yaw_ok && distance_ok) {
      transition(State::ContactVerify, "contact_verify");
      return;
    }

    if (std::abs(detection.lateral_y) > lateral_hard_limit_m_ ||
      std::abs(detection.yaw_error) > yaw_hard_limit_rad_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "dock alignment outside hard limit: y=%.3f yaw=%.3fdeg",
        detection.lateral_y, detection.yaw_error * 180.0 / kPi);
    }

    geometry_msgs::msg::Twist cmd;
    const double distance_error = std::max(0.0, detection.distance_x - final_target_distance_m_);
    cmd.linear.x = clamp(kx_ * distance_error, 0.0, max_linear_speed_mps_);
    if (!distance_ok && distance_error > 0.0) {
      cmd.linear.x = std::max(cmd.linear.x, std::min(min_align_speed_mps_, max_linear_speed_mps_));
    }

    const double lateral_error = apply_deadband(detection.lateral_y, lateral_deadband_m_);
    const double yaw_error = apply_deadband(detection.yaw_error, yaw_deadband_rad_);
    double desired_contact_vy = 0.0;
    double pivot_compensation_vy = 0.0;
    if (use_crab_mode_) {
      if (yaw_spin_priority_enabled_ && !yaw_ok) {
        publish_forced_mode(yaw_forced_mode_);
        cmd.linear.x = 0.0;
        cmd.linear.y = 0.0;
        cmd.angular.z = clamp(kyaw_ * yaw_error, -max_angular_speed_radps_, max_angular_speed_radps_);
        publish_cmd(cmd);
        publish_status(alignment_status(
          detection, cmd, desired_contact_vy, pivot_compensation_vy, "yaw_spin", yaw_forced_mode_));
        return;
      }

      publish_forced_mode(crab_forced_mode_);
      const bool final_insert_locked = lock_lateral_during_final_insert_ && lateral_ok && yaw_ok;
      if (final_insert_locked) {
        cmd.linear.y = 0.0;
        cmd.angular.z = 0.0;
      } else {
        desired_contact_vy = clamp(
          lateral_command_sign_ * ky_lateral_ * lateral_error,
          -max_lateral_speed_mps_,
          max_lateral_speed_mps_);
        if (std::abs(lateral_error) > 0.0 && std::abs(desired_contact_vy) < min_lateral_speed_mps_) {
          desired_contact_vy = std::copysign(
            std::min(min_lateral_speed_mps_, max_lateral_speed_mps_),
            desired_contact_vy);
        }
        // Once yaw is inside the fine-docking tolerance, keep yaw locked and correct
        // centerline only. Mixing angular.z with side-slip near the dock makes the
        // forward contact point sweep an arc and can drive a lateral/yaw limit cycle.
        cmd.angular.z = 0.0;
        pivot_compensation_vy = 0.0;
        cmd.linear.y = clamp(
          desired_contact_vy + pivot_compensation_vy,
          -max_lateral_speed_mps_,
          max_lateral_speed_mps_);
      }
      if (!final_insert_locked && (!lateral_ok || !yaw_ok ||
        std::abs(lateral_error) > lateral_priority_threshold_m_ ||
        std::abs(yaw_error) > yaw_priority_threshold_rad_)) {
        cmd.linear.x = std::min(cmd.linear.x, max_forward_while_lateral_mps_);
      }
    } else {
      const double requested_wz = ky_ * lateral_error + kyaw_ * yaw_error;
      cmd.angular.z = limit_yaw_rate_for_ackermann(cmd.linear.x, requested_wz);
    }
    publish_cmd(cmd);

    publish_status(alignment_status(
      detection, cmd, desired_contact_vy, pivot_compensation_vy,
      use_crab_mode_ ? "side_slip_align" : "ackermann_align",
      use_crab_mode_ ? crab_forced_mode_ : release_forced_mode_));
  }

  void handle_contact_verify()
  {
    if (charging_detected_) {
      docked_stop("docked_charging_detected");
      return;
    }

    if ((now() - state_entered_time_).seconds() > contact_confirm_timeout_s_) {
      if (retries_++ >= max_retries_) {
        fail("contact_verify_timeout");
        return;
      }
      transition(State::BlindApproach, "retry_after_contact_timeout");
      return;
    }

    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = clamp(contact_crawl_speed_mps_, 0.0, max_linear_speed_mps_);
    publish_cmd(cmd);
  }

  void handle_undocking()
  {
    const auto stamp = now();
    const double speed = clamp(undock_speed_mps_, 0.0, max_linear_speed_mps_);
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

  double apply_deadband(double value, double deadband) const
  {
    if (std::abs(value) <= deadband) {
      return 0.0;
    }
    return std::copysign(std::abs(value) - deadband, value);
  }

  bool battery_indicates_charging(const sensor_msgs::msg::BatteryState & msg) const
  {
    return msg.power_supply_status == sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_CHARGING ||
      msg.power_supply_status == sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_FULL ||
      (std::isfinite(msg.current) && static_cast<double>(msg.current) > min_charging_current_a_) ||
      (msg.present && voltage_in_contact_range(
        msg.voltage, charging_contact_voltage_min_v_, charging_contact_voltage_max_v_));
  }

  bool battery_indicates_charging_contact(const sensor_msgs::msg::BatteryState & msg) const
  {
    return battery_charging_contact(msg).contact;
  }

  BatteryContactEvaluation battery_charging_contact(const sensor_msgs::msg::BatteryState & msg) const
  {
    if (msg.power_supply_status == sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_CHARGING) {
      return {true, "power_supply_status=CHARGING"};
    }
    if (msg.power_supply_status == sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_FULL) {
      return {true, "power_supply_status=FULL"};
    }
    if (std::isfinite(msg.current) && static_cast<double>(msg.current) > min_charging_current_a_) {
      return {true, "current_above_threshold"};
    }
    if (msg.present && voltage_in_contact_range(
      msg.voltage, charging_contact_voltage_min_v_, charging_contact_voltage_max_v_)) {
      return {true, "present_voltage_valid"};
    }
    const double soc = normalized_soc_percent(msg.percentage);
    if (charging_full_soc_voltage_contact_enable_ && msg.present && std::isfinite(soc) &&
      soc >= charging_full_soc_threshold_pct_ &&
      voltage_in_contact_range(msg.voltage, charging_contact_voltage_min_v_, charging_contact_voltage_max_v_)) {
      return {true, "full_soc_present_voltage_valid"};
    }
    return {false, "no_contact"};
  }

  bool docking_is_active() const
  {
    return state_ == State::BlindApproach || state_ == State::Acquire ||
      state_ == State::Align || state_ == State::ContactVerify;
  }

  void docked_stop(const std::string & status)
  {
    state_ = State::Docked;
    publish_zero();
    publish_reverse_enable(false);
    release_docking_motion_mode(park_on_docked_);
    update_dock_contact_latch(true, "docking_manager", status, "");
    publish_status(status);
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

  double limit_yaw_rate_for_ackermann(double vx, double requested_wz) const
  {
    double limited = clamp(requested_wz, -max_angular_speed_radps_, max_angular_speed_radps_);
    if (vx <= 1.0e-3) {
      return 0.0;
    }
    const double max_by_curvature =
      std::abs(vx) * std::tan(std::max(0.01, max_command_steering_rad_)) * 2.0 /
      std::max(0.1, ackermann_wheelbase_m_);
    return clamp(limited, -max_by_curvature, max_by_curvature);
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

  std::string alignment_status(
    const Detection & detection,
    const geometry_msgs::msg::Twist & cmd,
    const double desired_contact_vy,
    const double pivot_compensation_vy,
    const std::string & phase,
    const std::string & forced_mode) const
  {
    std::ostringstream out;
    out << detection_status("aligning", detection)
        << " phase=" << phase
        << " forced_mode=" << forced_mode
        << " cmd_vx=" << cmd.linear.x
        << " cmd_vy=" << cmd.linear.y
        << " cmd_wz=" << cmd.angular.z
        << " desired_contact_vy=" << desired_contact_vy
        << " pivot_comp_vy=" << pivot_compensation_vy
        << " charge_contact_x=" << charge_contact_x_m_;
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

  std::string gs2_scan_topic_;
  std::string cmd_vel_topic_;
  std::string status_topic_;
  std::string start_service_;
  std::string stop_service_;
  std::string undock_service_;
  std::string charging_state_topic_;
  std::string docking_contact_latch_file_;
  mutable bool have_last_dock_contact_latch_write_{false};
  mutable bool last_dock_contact_latch_docked_{false};
  mutable std::string last_dock_contact_latch_source_;
  mutable std::string last_dock_contact_latch_reason_;
  mutable std::string last_dock_contact_latch_dock_id_;
  mutable std::string last_forced_mode_request_;
  std::string undock_odom_topic_{"/local_state/odometry"};
  std::string forced_mode_topic_;
  std::string park_topic_;
  std::string reverse_enable_topic_;
  std::string crab_forced_mode_{"side_slip"};
  std::string yaw_forced_mode_{"spinning"};
  std::string release_forced_mode_{"auto"};
  bool use_crab_mode_{true};
  bool park_on_docked_{true};

  double gs2_x_m_{0.360};
  double charge_contact_x_m_{0.398};
  double gs2_to_contact_x_m_{0.038};
  double blind_approach_max_distance_m_{0.50};
  double blind_approach_speed_mps_{0.06};
  double gs2_acquire_distance_m_{0.28};
  double final_target_distance_m_{0.05};
  double undock_distance_m_{0.60};
  double undock_speed_mps_{0.06};
  double undock_min_clear_distance_m_{0.45};
  double undock_timeout_s_{12.0};
  double undock_odom_timeout_s_{0.50};
  double undock_odom_start_timeout_s_{2.0};
  double undock_command_settle_s_{0.5};
  double undock_motion_start_timeout_s_{6.0};
  double undock_no_progress_timeout_s_{2.0};
  double undock_progress_epsilon_m_{0.005};
  double lateral_soft_limit_m_{0.030};
  double lateral_hard_limit_m_{0.050};
  double yaw_soft_limit_rad_{deg_to_rad(2.0)};
  double yaw_hard_limit_rad_{deg_to_rad(4.0)};
  double contact_confirm_timeout_s_{3.0};
  double max_linear_speed_mps_{0.08};
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
  double ky_{0.55};
  double ky_lateral_{0.70};
  double lateral_command_sign_{-1.0};
  double kyaw_{0.0};
  double lateral_deadband_m_{0.010};
  double yaw_deadband_rad_{deg_to_rad(1.0)};
  double min_align_speed_mps_{0.025};
  double min_lateral_speed_mps_{0.025};
  double max_lateral_speed_mps_{0.04};
  double lateral_priority_threshold_m_{0.020};
  double yaw_priority_threshold_rad_{deg_to_rad(2.0)};
  double max_forward_while_lateral_mps_{0.020};
  bool lock_lateral_during_final_insert_{true};
  bool yaw_spin_priority_enabled_{true};
  double max_command_steering_rad_{0.35};
  double ackermann_wheelbase_m_{0.494};
  double contact_crawl_speed_mps_{0.025};
  double min_charging_current_a_{0.10};
  double charging_contact_voltage_min_v_{40.0};
  double charging_contact_voltage_max_v_{1000.0};
  double charging_full_soc_threshold_pct_{99.0};
  bool charging_full_soc_voltage_contact_enable_{true};

  State state_{State::Idle};
  int retries_{0};
  int valid_detection_streak_{0};
  bool charging_detected_{false};
  bool charging_contact_detected_{false};
  bool has_filtered_detection_{false};
  bool have_undock_start_odom_{false};
  bool have_undock_first_motion_{false};
  UndockPhase undock_phase_{UndockPhase::UNDOCK_IDLE};
  double undock_start_x_{0.0};
  double undock_start_y_{0.0};
  double undock_max_progress_m_{0.0};
  double last_undock_cmd_x_{0.0};
  size_t undock_nonzero_cmd_publish_count_{0U};
  mutable size_t undock_reverse_enable_publish_count_{0U};
  mutable bool last_undock_reverse_enable_{false};
  Detection filtered_detection_;
  rclcpp::Time state_entered_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_scan_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_odom_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time undock_nonzero_cmd_start_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_undock_cmd_publish_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time first_undock_motion_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_undock_progress_time_{0, 0, RCL_ROS_TIME};
  sensor_msgs::msg::LaserScan::SharedPtr latest_scan_;
  sensor_msgs::msg::BatteryState::SharedPtr latest_battery_;
  nav_msgs::msg::Odometry::SharedPtr latest_odom_;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
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
