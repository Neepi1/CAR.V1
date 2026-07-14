#include <algorithm>
#include <chrono>
#include <cinttypes>
#include <cmath>
#include <cstdint>
#include <memory>
#include <optional>
#include <sstream>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ranger_msgs/msg/motion_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_ros/transform_broadcaster.h"

#include "robot_local_state/spin_yaw_corrector.hpp"

using namespace std::chrono_literals;

namespace
{
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

geometry_msgs::msg::Quaternion quaternion_from_yaw(const double yaw)
{
  geometry_msgs::msg::Quaternion q;
  q.z = std::sin(yaw * 0.5);
  q.w = std::cos(yaw * 0.5);
  return q;
}

void rotate_xy(double & x, double & y, const double yaw)
{
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  const double rx = c * x - s * y;
  const double ry = s * x + c * y;
  x = rx;
  y = ry;
}
}  // namespace

class LocalStateNode : public rclcpp::Node
{
public:
  LocalStateNode()
  : Node("robot_local_state")
  {
    mock_mode_ = declare_parameter<bool>("mock_mode", false);
    publish_tf_ = declare_parameter<bool>("publish_tf", true);
    output_topic_ = declare_parameter<std::string>("output_topic", "/local_state/odometry");
    input_odom_topic_ = declare_parameter<std::string>("input_odom_topic", "/wheel/odom");
    input_base_frame_ = declare_parameter<std::string>("input_base_frame", "base_link");
    input_imu_topic_ = declare_parameter<std::string>("input_imu_topic", "/lidar_imu");
    input_motion_state_topic_ =
      declare_parameter<std::string>("input_motion_state_topic", "/motion_state");
    input_cmd_vel_topic_ = declare_parameter<std::string>("input_cmd_vel_topic", "/cmd_vel");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    odom_yaw_offset_rad_ = declare_parameter<double>("odom_yaw_offset_rad", 0.0);
    rotate_odom_position_with_yaw_offset_ =
      declare_parameter<bool>("rotate_odom_position_with_yaw_offset", true);
    odom_position_scale_x_ = declare_parameter<double>("odom_position_scale_x", 1.0);
    odom_position_scale_y_ = declare_parameter<double>("odom_position_scale_y", 1.0);
    odom_position_y_to_x_shear_ = declare_parameter<double>("odom_position_y_to_x_shear", 0.0);
    odom_position_x_to_y_shear_ = declare_parameter<double>("odom_position_x_to_y_shear", 0.0);
    odom_yaw_scale_positive_ = declare_parameter<double>("odom_yaw_scale_positive", 1.0);
    odom_yaw_scale_negative_ = declare_parameter<double>("odom_yaw_scale_negative", 1.0);
    scale_odom_twist_with_yaw_scale_ =
      declare_parameter<bool>("scale_odom_twist_with_yaw_scale", true);
    anchor_pose_to_first_sample_ = declare_parameter<bool>("anchor_pose_to_first_sample", false);
    apply_twist_covariance_floor_ = declare_parameter<bool>("apply_twist_covariance_floor", false);
    apply_pose_covariance_floor_ = declare_parameter<bool>("apply_pose_covariance_floor", false);
    pose_covariance_floor_x_ = declare_parameter<double>("pose_covariance_floor_x", 0.0);
    pose_covariance_floor_y_ = declare_parameter<double>("pose_covariance_floor_y", 0.0);
    pose_covariance_floor_yaw_ = declare_parameter<double>("pose_covariance_floor_yaw", 0.0);
    twist_covariance_floor_vx_ = declare_parameter<double>("twist_covariance_floor_vx", 0.0);
    twist_covariance_floor_vy_ = declare_parameter<double>("twist_covariance_floor_vy", 0.0);
    twist_covariance_floor_vyaw_ = declare_parameter<double>("twist_covariance_floor_vyaw", 0.0);
    publish_rate_hz_ = std::max(1.0, declare_parameter<double>("publish_rate_hz", 20.0));
    publish_on_callback_ = declare_parameter<bool>("publish_on_callback", false);
    republish_latest_ = declare_parameter<bool>("republish_latest", true);
    republish_latest_max_age_sec_ =
      std::max(0.0, declare_parameter<double>("republish_latest_max_age_sec", 0.5));

    spin_yaw_correction_enabled_ =
      declare_parameter<bool>("spin_yaw_correction_enabled", false);
    spin_yaw_status_topic_ = declare_parameter<std::string>(
      "spin_yaw_status_topic", "/local_state/spin_yaw_correction_status");
    spin_yaw_status_rate_hz_ = std::max(
      0.2, declare_parameter<double>("spin_yaw_status_rate_hz", 5.0));

    if (spin_yaw_correction_enabled_) {
      robot_local_state::SpinYawCorrectorConfig config;
      const auto spinning_motion_mode =
        declare_parameter<std::int64_t>("spin_motion_mode", 2);
      config.spinning_motion_mode = static_cast<std::uint8_t>(std::clamp(
        spinning_motion_mode, static_cast<std::int64_t>(0), static_cast<std::int64_t>(255)));
      config.command_start_threshold_radps = std::max(
        0.0, declare_parameter<double>("spin_command_start_threshold_radps", 0.05));
      config.command_zero_linear_threshold_mps = std::max(
        0.0, declare_parameter<double>("spin_command_zero_linear_threshold_mps", 0.02));
      config.command_zero_angular_threshold_radps = std::max(
        0.0, declare_parameter<double>("spin_command_zero_angular_threshold_radps", 0.02));
      config.imu_motion_threshold_radps = std::max(
        0.0, declare_parameter<double>("spin_imu_motion_threshold_radps", 0.03));
      config.imu_stop_threshold_radps = std::max(
        0.0, declare_parameter<double>("spin_imu_stop_threshold_radps", 0.02));
      config.imu_stop_stable_sec = std::max(
        0.0, declare_parameter<double>("spin_imu_stop_stable_sec", 0.30));
      config.imu_timeout_sec = std::max(
        0.02, declare_parameter<double>("spin_imu_timeout_sec", 0.20));
      config.imu_max_integration_dt_sec = std::max(
        0.005, declare_parameter<double>("spin_imu_max_integration_dt_sec", 0.05));
      config.settle_timeout_sec = std::max(
        config.imu_stop_stable_sec,
        declare_parameter<double>("spin_settle_timeout_sec", 2.0));
      config.freeze_xy_max_command_linear_mps = std::max(
        0.0, declare_parameter<double>("spin_freeze_xy_max_command_linear_mps", 0.03));
      config.freeze_xy_while_spinning =
        declare_parameter<bool>("spin_freeze_xy_while_spinning", true);
      config.replace_spin_twist_with_imu =
        declare_parameter<bool>("spin_replace_twist_with_imu", false);
      spin_yaw_corrector_ =
        std::make_unique<robot_local_state::SpinYawCorrector>(config);
    }

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(output_topic_, rclcpp::QoS(20));
    if (publish_tf_) {
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }

    if (mock_mode_) {
      timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::duration<double>(1.0 / publish_rate_hz_)),
        std::bind(&LocalStateNode::on_mock_timer, this));
    } else {
      if (!publish_on_callback_ && !republish_latest_) {
        RCLCPP_WARN(
          get_logger(),
          "publish_on_callback=false and republish_latest=false would suppress odom output; "
          "enabling callback publication for compatibility");
        publish_on_callback_ = true;
      }
      odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        input_odom_topic_,
        rclcpp::QoS(20),
        std::bind(&LocalStateNode::on_wheel_odom, this, std::placeholders::_1));
      if (spin_yaw_corrector_) {
        imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
          input_imu_topic_,
          rclcpp::SensorDataQoS().keep_last(1),
          std::bind(&LocalStateNode::on_imu, this, std::placeholders::_1));
        motion_state_sub_ = create_subscription<ranger_msgs::msg::MotionState>(
          input_motion_state_topic_,
          rclcpp::QoS(20),
          std::bind(&LocalStateNode::on_motion_state, this, std::placeholders::_1));
        cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
          input_cmd_vel_topic_,
          rclcpp::QoS(20),
          std::bind(&LocalStateNode::on_cmd_vel, this, std::placeholders::_1));
        spin_yaw_status_pub_ = create_publisher<std_msgs::msg::String>(
          spin_yaw_status_topic_, rclcpp::QoS(10));
        RCLCPP_INFO(
          get_logger(),
          "Spin yaw correction enabled: wheel=%s imu=%s mode=%s cmd=%s status=%s",
          input_odom_topic_.c_str(), input_imu_topic_.c_str(),
          input_motion_state_topic_.c_str(), input_cmd_vel_topic_.c_str(),
          spin_yaw_status_topic_.c_str());
      }
      if (republish_latest_) {
        timer_ = create_wall_timer(
          std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(1.0 / publish_rate_hz_)),
          std::bind(&LocalStateNode::on_republish_timer, this));
      }
    }
  }

private:
  void apply_canonical_odom_transform(nav_msgs::msg::Odometry & odom) const
  {
    if (std::abs(odom_yaw_offset_rad_) < 1e-12) {
      return;
    }

    if (rotate_odom_position_with_yaw_offset_) {
      rotate_xy(odom.pose.pose.position.x, odom.pose.pose.position.y, odom_yaw_offset_rad_);
    }

    const double yaw = yaw_from_quaternion(odom.pose.pose.orientation);
    odom.pose.pose.orientation = quaternion_from_yaw(normalize_angle(yaw + odom_yaw_offset_rad_));
  }

  static void apply_covariance_floor(double & value, const double floor)
  {
    if (floor <= 0.0) {
      return;
    }
    if (!std::isfinite(value) || value < floor) {
      value = floor;
    }
  }

  void apply_planar_position_calibration(nav_msgs::msg::Odometry & odom) const
  {
    if (
      std::abs(odom_position_scale_x_ - 1.0) < 1e-12 &&
      std::abs(odom_position_scale_y_ - 1.0) < 1e-12 &&
      std::abs(odom_position_y_to_x_shear_) < 1e-12 &&
      std::abs(odom_position_x_to_y_shear_) < 1e-12)
    {
      return;
    }

    const double x = odom.pose.pose.position.x;
    const double y = odom.pose.pose.position.y;
    odom.pose.pose.position.x = odom_position_scale_x_ * x + odom_position_y_to_x_shear_ * y;
    odom.pose.pose.position.y = odom_position_x_to_y_shear_ * x + odom_position_scale_y_ * y;
  }

  double yaw_scale_for(const double yaw_or_rate) const
  {
    const double scale = yaw_or_rate >= 0.0 ? odom_yaw_scale_positive_ : odom_yaw_scale_negative_;
    if (!std::isfinite(scale) || scale <= 0.0) {
      return 1.0;
    }
    return scale;
  }

  void apply_yaw_scale_calibration(nav_msgs::msg::Odometry & odom) const
  {
    if (
      std::abs(odom_yaw_scale_positive_ - 1.0) < 1e-12 &&
      std::abs(odom_yaw_scale_negative_ - 1.0) < 1e-12)
    {
      return;
    }

    const double yaw = yaw_from_quaternion(odom.pose.pose.orientation);
    odom.pose.pose.orientation = quaternion_from_yaw(normalize_angle(yaw * yaw_scale_for(yaw)));

    if (!scale_odom_twist_with_yaw_scale_) {
      return;
    }

    auto & vyaw = odom.twist.twist.angular.z;
    if (std::isfinite(vyaw)) {
      vyaw *= yaw_scale_for(vyaw);
    }
  }

  void apply_pose_anchor(nav_msgs::msg::Odometry & odom)
  {
    if (!anchor_pose_to_first_sample_) {
      return;
    }

    const double yaw = yaw_from_quaternion(odom.pose.pose.orientation);
    if (!has_pose_anchor_) {
      anchor_x_ = odom.pose.pose.position.x;
      anchor_y_ = odom.pose.pose.position.y;
      anchor_yaw_ = yaw;
      has_pose_anchor_ = true;
      RCLCPP_INFO(
        get_logger(),
        "Anchored wheel odom pose at x=%.3f y=%.3f yaw=%.3f rad",
        anchor_x_,
        anchor_y_,
        anchor_yaw_);
    }

    double x = odom.pose.pose.position.x - anchor_x_;
    double y = odom.pose.pose.position.y - anchor_y_;
    rotate_xy(x, y, -anchor_yaw_);
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.orientation = quaternion_from_yaw(normalize_angle(yaw - anchor_yaw_));
  }

  void apply_twist_covariance_floor(nav_msgs::msg::Odometry & odom) const
  {
    if (!apply_twist_covariance_floor_) {
      return;
    }
    apply_covariance_floor(odom.twist.covariance[0], twist_covariance_floor_vx_);
    apply_covariance_floor(odom.twist.covariance[7], twist_covariance_floor_vy_);
    apply_covariance_floor(odom.twist.covariance[35], twist_covariance_floor_vyaw_);
  }

  void apply_pose_covariance_floor(nav_msgs::msg::Odometry & odom) const
  {
    if (!apply_pose_covariance_floor_) {
      return;
    }
    apply_covariance_floor(odom.pose.covariance[0], pose_covariance_floor_x_);
    apply_covariance_floor(odom.pose.covariance[7], pose_covariance_floor_y_);
    apply_covariance_floor(odom.pose.covariance[35], pose_covariance_floor_yaw_);
  }

  static double message_stamp_seconds(const builtin_interfaces::msg::Time & stamp)
  {
    return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1.0e-9;
  }

  void apply_spin_yaw_correction(nav_msgs::msg::Odometry & odom, const double now_sec)
  {
    if (!spin_yaw_corrector_) {
      return;
    }

    const auto corrected = spin_yaw_corrector_->correct_wheel(
      odom.pose.pose.position.x,
      odom.pose.pose.position.y,
      yaw_from_quaternion(odom.pose.pose.orientation),
      odom.twist.twist.angular.z,
      now_sec);
    odom.pose.pose.position.x = corrected.x;
    odom.pose.pose.position.y = corrected.y;
    odom.pose.pose.orientation = quaternion_from_yaw(normalize_angle(corrected.yaw));
    odom.twist.twist.angular.z = corrected.angular_velocity_z;
  }

  void maybe_log_spin_transition(const bool was_active, const char * source)
  {
    if (!spin_yaw_corrector_) {
      return;
    }
    const auto status = spin_yaw_corrector_->status(now().seconds());
    if (was_active == status.correction_active) {
      return;
    }
    if (status.correction_active) {
      RCLCPP_INFO(
        get_logger(), "Spin yaw correction started source=%s mode=%u",
        source, static_cast<unsigned int>(status.motion_mode));
    } else {
      RCLCPP_INFO(
        get_logger(),
        "Spin yaw correction ended source=%s imu_delta=%.4frad yaw_offset=%.4frad "
        "completed=%" PRIu64 " fallbacks=%" PRIu64,
        source, status.spin_imu_delta_rad, status.yaw_offset_rad,
        status.completed_spin_count, status.imu_fallback_count);
    }
  }

  void publish_spin_yaw_status(const double now_sec, const bool force = false)
  {
    if (!spin_yaw_corrector_ || !spin_yaw_status_pub_) {
      return;
    }
    const double min_period_sec = 1.0 / spin_yaw_status_rate_hz_;
    if (!force && last_spin_yaw_status_publish_sec_ > 0.0 &&
      (now_sec - last_spin_yaw_status_publish_sec_) < min_period_sec)
    {
      return;
    }

    const auto status = spin_yaw_corrector_->status(now_sec);
    std::ostringstream stream;
    stream.setf(std::ios::boolalpha);
    stream.precision(9);
    stream << "{\"schema\":\"njrh.spin_yaw_correction.v1\""
           << ",\"active\":" << status.correction_active
           << ",\"spin_command_seen\":" << status.spin_command_seen
           << ",\"zero_command_seen\":" << status.zero_command_seen
           << ",\"imu_fresh\":" << status.imu_fresh
           << ",\"settle_ready\":" << status.settle_ready
           << ",\"motion_mode\":" << static_cast<unsigned int>(status.motion_mode)
           << ",\"raw_yaw_unwrapped_rad\":" << status.raw_yaw_unwrapped
           << ",\"corrected_yaw_unwrapped_rad\":" << status.corrected_yaw_unwrapped
           << ",\"yaw_offset_rad\":" << status.yaw_offset_rad
           << ",\"spin_imu_delta_rad\":" << status.spin_imu_delta_rad
           << ",\"imu_yaw_rate_radps\":" << status.latest_imu_yaw_rate_radps
           << ",\"completed_spin_count\":" << status.completed_spin_count
           << ",\"imu_fallback_count\":" << status.imu_fallback_count
           << ",\"imu_gap_count\":" << status.imu_gap_count
           << ",\"forced_settle_count\":" << status.forced_settle_count
           << "}";
    std_msgs::msg::String msg;
    msg.data = stream.str();
    spin_yaw_status_pub_->publish(msg);
    last_spin_yaw_status_publish_sec_ = now_sec;
  }

  void on_imu(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    if (!spin_yaw_corrector_) {
      return;
    }
    const bool was_active = spin_yaw_corrector_->status(now().seconds()).correction_active;
    double stamp_sec = message_stamp_seconds(msg->header.stamp);
    if (stamp_sec <= 0.0) {
      stamp_sec = now().seconds();
    }
    spin_yaw_corrector_->observe_imu(msg->angular_velocity.z, stamp_sec);
    maybe_log_spin_transition(was_active, "imu");
  }

  void on_motion_state(const ranger_msgs::msg::MotionState::SharedPtr msg)
  {
    if (!spin_yaw_corrector_) {
      return;
    }
    const double now_sec = now().seconds();
    const bool was_active = spin_yaw_corrector_->status(now_sec).correction_active;
    spin_yaw_corrector_->observe_motion_mode(msg->motion_mode, now_sec);
    maybe_log_spin_transition(was_active, "motion_state");
  }

  void on_cmd_vel(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    if (!spin_yaw_corrector_) {
      return;
    }
    const double now_sec = now().seconds();
    const bool was_active = spin_yaw_corrector_->status(now_sec).correction_active;
    spin_yaw_corrector_->observe_command(
      msg->linear.x, msg->linear.y, msg->angular.z, now_sec);
    maybe_log_spin_transition(was_active, "cmd_vel");
  }

  void publish_local_state(const nav_msgs::msg::Odometry & odom)
  {
    odom_pub_->publish(odom);
    if (!publish_tf_ || !tf_broadcaster_) {
      return;
    }

    geometry_msgs::msg::TransformStamped tf;
    tf.header = odom.header;
    tf.child_frame_id = odom.child_frame_id;
    tf.transform.translation.x = odom.pose.pose.position.x;
    tf.transform.translation.y = odom.pose.pose.position.y;
    tf.transform.translation.z = odom.pose.pose.position.z;
    tf.transform.rotation = odom.pose.pose.orientation;
    tf_broadcaster_->sendTransform(tf);
  }

  void on_wheel_odom(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    if (!msg->child_frame_id.empty() && msg->child_frame_id != input_base_frame_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        5000,
        "Input odom child_frame_id is '%s', expected native chassis frame '%s'. "
        "Publishing canonical child frame '%s'.",
        msg->child_frame_id.c_str(),
        input_base_frame_.c_str(),
        base_frame_.c_str());
    }
    const double now_sec = now().seconds();
    const bool spin_was_active = spin_yaw_corrector_ &&
      spin_yaw_corrector_->status(now_sec).correction_active;
    nav_msgs::msg::Odometry local_odom = *msg;
    apply_pose_anchor(local_odom);
    apply_planar_position_calibration(local_odom);
    apply_yaw_scale_calibration(local_odom);
    apply_canonical_odom_transform(local_odom);
    apply_spin_yaw_correction(local_odom, now_sec);
    apply_pose_covariance_floor(local_odom);
    apply_twist_covariance_floor(local_odom);
    local_odom.header.frame_id = odom_frame_;
    local_odom.child_frame_id = base_frame_;
    latest_local_odom_ = local_odom;
    latest_local_odom_received_sec_ = now_sec;
    maybe_log_spin_transition(spin_was_active, "wheel_odom");
    publish_spin_yaw_status(now_sec);
    if (publish_on_callback_) {
      publish_local_state(local_odom);
    }
  }

  void on_republish_timer()
  {
    if (!latest_local_odom_) {
      return;
    }

    const auto stamp = now();
    const double age_sec = stamp.seconds() - latest_local_odom_received_sec_;
    if (age_sec > republish_latest_max_age_sec_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Skipping local odom republish because latest input odom is stale: %.3fs > %.3fs",
        age_sec, republish_latest_max_age_sec_);
      return;
    }

    auto odom = *latest_local_odom_;
    odom.header.stamp = stamp;
    publish_local_state(odom);
  }

  void on_mock_timer()
  {
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = now();
    odom.header.frame_id = odom_frame_;
    odom.child_frame_id = base_frame_;
    odom.pose.pose.orientation.w = 1.0;
    publish_local_state(odom);
  }

  bool mock_mode_{false};
  bool publish_tf_{true};
  std::string output_topic_;
  std::string input_odom_topic_;
  std::string input_base_frame_;
  std::string input_imu_topic_;
  std::string input_motion_state_topic_;
  std::string input_cmd_vel_topic_;
  std::string odom_frame_;
  std::string base_frame_;
  double odom_yaw_offset_rad_{0.0};
  bool rotate_odom_position_with_yaw_offset_{true};
  double odom_position_scale_x_{1.0};
  double odom_position_scale_y_{1.0};
  double odom_position_y_to_x_shear_{0.0};
  double odom_position_x_to_y_shear_{0.0};
  double odom_yaw_scale_positive_{1.0};
  double odom_yaw_scale_negative_{1.0};
  bool scale_odom_twist_with_yaw_scale_{true};
  bool anchor_pose_to_first_sample_{false};
  bool has_pose_anchor_{false};
  double anchor_x_{0.0};
  double anchor_y_{0.0};
  double anchor_yaw_{0.0};
  bool apply_twist_covariance_floor_{false};
  bool apply_pose_covariance_floor_{false};
  double pose_covariance_floor_x_{0.0};
  double pose_covariance_floor_y_{0.0};
  double pose_covariance_floor_yaw_{0.0};
  double twist_covariance_floor_vx_{0.0};
  double twist_covariance_floor_vy_{0.0};
  double twist_covariance_floor_vyaw_{0.0};
  double publish_rate_hz_{20.0};
  bool publish_on_callback_{false};
  bool republish_latest_{true};
  double republish_latest_max_age_sec_{0.5};
  bool spin_yaw_correction_enabled_{false};
  std::string spin_yaw_status_topic_;
  double spin_yaw_status_rate_hz_{5.0};
  double last_spin_yaw_status_publish_sec_{0.0};
  std::optional<nav_msgs::msg::Odometry> latest_local_odom_;
  double latest_local_odom_received_sec_{0.0};

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<ranger_msgs::msg::MotionState>::SharedPtr motion_state_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr spin_yaw_status_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::unique_ptr<robot_local_state::SpinYawCorrector> spin_yaw_corrector_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LocalStateNode>());
  rclcpp::shutdown();
  return 0;
}
