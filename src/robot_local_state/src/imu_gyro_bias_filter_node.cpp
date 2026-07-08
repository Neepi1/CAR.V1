#include <algorithm>
#include <array>
#include <chrono>
#include <cinttypes>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <string>

#include "builtin_interfaces/msg/time.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace
{
bool is_finite(const double value)
{
  return std::isfinite(value);
}

bool near_zero(const double value, const double threshold)
{
  return std::abs(value) <= threshold;
}

double ewma(const double alpha, const double previous, const double sample)
{
  return alpha * sample + (1.0 - alpha) * previous;
}

std::chrono::nanoseconds period_from_hz(const double rate_hz)
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(1.0 / std::max(0.1, rate_hz)));
}

double stamp_seconds(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1e-9;
}

geometry_msgs::msg::Vector3 rotate_vector(
  const geometry_msgs::msg::Vector3 & input,
  const tf2::Matrix3x3 & rotation_matrix)
{
  const tf2::Vector3 rotated = rotation_matrix * tf2::Vector3(input.x, input.y, input.z);
  geometry_msgs::msg::Vector3 output;
  output.x = rotated.x();
  output.y = rotated.y();
  output.z = rotated.z();
  return output;
}

std::array<double, 9> rotate_covariance(
  const std::array<double, 9> & covariance,
  const tf2::Matrix3x3 & rotation_matrix)
{
  std::array<double, 9> rotated{};
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      double value = 0.0;
      for (int left = 0; left < 3; ++left) {
        for (int right = 0; right < 3; ++right) {
          value += rotation_matrix[row][left] * covariance[left * 3 + right] *
            rotation_matrix[col][right];
        }
      }
      rotated[row * 3 + col] = value;
    }
  }
  return rotated;
}

tf2::Matrix3x3 rotation_matrix_from_quaternion(
  const geometry_msgs::msg::Quaternion & quaternion_msg)
{
  tf2::Quaternion quaternion(
    quaternion_msg.x,
    quaternion_msg.y,
    quaternion_msg.z,
    quaternion_msg.w);
  if (quaternion.length2() <= 0.0) {
    quaternion.setValue(0.0, 0.0, 0.0, 1.0);
  } else {
    quaternion.normalize();
  }
  return tf2::Matrix3x3(quaternion);
}
}  // namespace

class ImuGyroBiasFilterNode : public rclcpp::Node
{
public:
  ImuGyroBiasFilterNode()
  : Node("imu_gyro_bias_filter")
  {
    imu_topic_ = declare_parameter<std::string>("imu_topic", "/lidar_imu");
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/wheel/odom_ekf");
    cmd_vel_topic_ = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel_safe");
    output_imu_topic_ = declare_parameter<std::string>("output_imu_topic", "/lidar_imu_bias_corrected");
    bias_topic_ = declare_parameter<std::string>("bias_topic", "/local_state/imu_bias");
    transform_output_to_target_frame_ =
      declare_parameter<bool>("transform_output_to_target_frame", false);
    output_target_frame_ = declare_parameter<std::string>("output_target_frame", "base_link");
    transform_lookup_timeout_sec_ =
      std::max(0.0, declare_parameter<double>("transform_lookup_timeout_sec", 0.02));
    drop_output_on_transform_failure_ =
      declare_parameter<bool>("drop_output_on_transform_failure", true);
    use_odom_stationary_ = declare_parameter<bool>("use_odom_stationary", true);
    use_cmd_vel_stationary_ = declare_parameter<bool>("use_cmd_vel_stationary", true);
    require_fresh_cmd_vel_ = declare_parameter<bool>("require_fresh_cmd_vel", false);
    odom_timeout_sec_ = declare_parameter<double>("odom_timeout_sec", 0.5);
    cmd_vel_timeout_sec_ = declare_parameter<double>("cmd_vel_timeout_sec", 0.5);
    stationary_required_sec_ = declare_parameter<double>("stationary_required_sec", 1.0);
    odom_linear_threshold_mps_ = declare_parameter<double>("odom_linear_threshold_mps", 0.01);
    odom_angular_threshold_radps_ = declare_parameter<double>("odom_angular_threshold_radps", 0.01);
    cmd_linear_threshold_mps_ = declare_parameter<double>("cmd_linear_threshold_mps", 0.01);
    cmd_angular_threshold_radps_ = declare_parameter<double>("cmd_angular_threshold_radps", 0.01);
    max_bias_sample_abs_radps_ = declare_parameter<double>("max_bias_sample_abs_radps", 0.05);
    accumulator_alpha_ = std::clamp(declare_parameter<double>("accumulator_alpha", 0.02), 0.0, 1.0);
    zero_output_when_stationary_ = declare_parameter<bool>("zero_output_when_stationary", true);
    corrected_output_rate_hz_ =
      std::max(0.1, declare_parameter<double>("corrected_output_rate_hz", 100.0));
    bias_publish_rate_hz_ =
      std::max(0.1, declare_parameter<double>("bias_publish_rate_hz", 10.0));
    corrected_output_latest_on_timer_ =
      declare_parameter<bool>("corrected_output_latest_on_timer", true);
    corrected_output_preserve_source_stamp_ =
      declare_parameter<bool>("corrected_output_preserve_source_stamp", true);
    corrected_output_max_source_age_sec_ =
      std::max(0.0, declare_parameter<double>("corrected_output_max_source_age_sec", 0.20));
    bias_publish_preserve_source_stamp_ =
      declare_parameter<bool>("bias_publish_preserve_source_stamp", true);

    if (transform_output_to_target_frame_) {
      if (output_target_frame_.empty()) {
        throw std::runtime_error(
          "output_target_frame must be non-empty when transform_output_to_target_frame is true");
      }
      tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
      tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this, true);
    }

    corrected_imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(
      output_imu_topic_,
      rclcpp::SensorDataQoS().keep_last(100));
    bias_pub_ = create_publisher<geometry_msgs::msg::Vector3Stamped>(bias_topic_, rclcpp::QoS(10));

    if (use_odom_stationary_) {
      odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_,
        rclcpp::QoS(20),
        std::bind(&ImuGyroBiasFilterNode::on_odom, this, std::placeholders::_1));
    }
    if (use_cmd_vel_stationary_) {
      cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        cmd_vel_topic_,
        rclcpp::QoS(20),
        std::bind(&ImuGyroBiasFilterNode::on_cmd_vel, this, std::placeholders::_1));
    }
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&ImuGyroBiasFilterNode::on_imu, this, std::placeholders::_1));

    if (corrected_output_latest_on_timer_) {
      corrected_output_timer_ = create_wall_timer(
        period_from_hz(corrected_output_rate_hz_),
        std::bind(&ImuGyroBiasFilterNode::on_corrected_output_timer, this));
    }
    bias_publish_timer_ = create_wall_timer(
      period_from_hz(bias_publish_rate_hz_),
      std::bind(&ImuGyroBiasFilterNode::on_bias_publish_timer, this));
  }

private:
  bool twist_is_zero(
    const geometry_msgs::msg::Twist & twist,
    const double linear_threshold,
    const double angular_threshold) const
  {
    return near_zero(twist.linear.x, linear_threshold) &&
           near_zero(twist.linear.y, linear_threshold) &&
           near_zero(twist.linear.z, linear_threshold) &&
           near_zero(twist.angular.x, angular_threshold) &&
           near_zero(twist.angular.y, angular_threshold) &&
           near_zero(twist.angular.z, angular_threshold);
  }

  bool sample_is_safe_for_bias_update(const sensor_msgs::msg::Imu & msg) const
  {
    return is_finite(msg.angular_velocity.x) &&
           is_finite(msg.angular_velocity.y) &&
           is_finite(msg.angular_velocity.z) &&
           std::abs(msg.angular_velocity.x) <= max_bias_sample_abs_radps_ &&
           std::abs(msg.angular_velocity.y) <= max_bias_sample_abs_radps_ &&
           std::abs(msg.angular_velocity.z) <= max_bias_sample_abs_radps_;
  }

  bool is_fresh(const bool valid, const double stamp_sec, const double timeout_sec, const double now_sec) const
  {
    return valid && timeout_sec >= 0.0 && (now_sec - stamp_sec) <= timeout_sec;
  }

  bool stationary_candidate(const double now_sec) const
  {
    bool candidate = true;
    if (use_odom_stationary_) {
      const bool odom_fresh = is_fresh(has_odom_, last_odom_sec_, odom_timeout_sec_, now_sec);
      candidate = candidate && odom_fresh && last_odom_is_stationary_;
    }
    if (use_cmd_vel_stationary_) {
      const bool cmd_fresh = is_fresh(has_cmd_vel_, last_cmd_vel_sec_, cmd_vel_timeout_sec_, now_sec);
      if (cmd_fresh) {
        candidate = candidate && last_cmd_vel_is_stationary_;
      } else if (require_fresh_cmd_vel_) {
        candidate = false;
      }
    }
    return candidate;
  }

  bool stationary_confirmed(const double now_sec)
  {
    if (!stationary_candidate(now_sec)) {
      stationary_since_sec_ = 0.0;
      return false;
    }
    if (stationary_since_sec_ <= 0.0) {
      stationary_since_sec_ = now_sec;
      return stationary_required_sec_ <= 0.0;
    }
    return (now_sec - stationary_since_sec_) >= stationary_required_sec_;
  }

  void update_bias(const sensor_msgs::msg::Imu & msg)
  {
    if (!bias_initialized_) {
      bias_.x = msg.angular_velocity.x;
      bias_.y = msg.angular_velocity.y;
      bias_.z = msg.angular_velocity.z;
      bias_initialized_ = true;
      return;
    }
    bias_.x = ewma(accumulator_alpha_, bias_.x, msg.angular_velocity.x);
    bias_.y = ewma(accumulator_alpha_, bias_.y, msg.angular_velocity.y);
    bias_.z = ewma(accumulator_alpha_, bias_.z, msg.angular_velocity.z);
  }

  geometry_msgs::msg::Vector3Stamped make_bias_msg(const sensor_msgs::msg::Imu & msg)
  {
    geometry_msgs::msg::Vector3Stamped bias_msg;
    bias_msg.header = msg.header;
    if (!bias_publish_preserve_source_stamp_) {
      bias_msg.header.stamp = now();
    }
    bias_msg.vector = bias_;
    return bias_msg;
  }

  double corrected_source_age_sec(const double now_sec) const
  {
    if (!latest_corrected_imu_) {
      return std::numeric_limits<double>::infinity();
    }
    const double source_stamp_sec = stamp_seconds(latest_corrected_imu_->header.stamp);
    if (source_stamp_sec <= 0.0) {
      return now_sec - latest_imu_receive_sec_;
    }
    return now_sec - source_stamp_sec;
  }

  void publish_corrected_imu(const sensor_msgs::msg::Imu & msg)
  {
    corrected_imu_pub_->publish(msg);
    ++output_corrected_count_;
    ++output_corrected_window_count_;
  }

  void publish_bias_msg(const geometry_msgs::msg::Vector3Stamped & msg)
  {
    bias_pub_->publish(msg);
    ++output_bias_count_;
    ++output_bias_window_count_;
  }

  void maybe_publish_corrected_direct(const double now_sec)
  {
    if (!latest_corrected_imu_) {
      return;
    }
    const double min_period_sec = 1.0 / corrected_output_rate_hz_;
    if (last_corrected_publish_sec_ > 0.0 &&
      (now_sec - last_corrected_publish_sec_) < min_period_sec)
    {
      return;
    }
    if (corrected_source_age_sec(now_sec) > corrected_output_max_source_age_sec_) {
      ++stale_corrected_skip_count_;
      return;
    }
    publish_corrected_imu(*latest_corrected_imu_);
    last_corrected_publish_sec_ = now_sec;
  }

  void maybe_report_rates(const double now_sec)
  {
    if (rate_window_start_sec_ <= 0.0) {
      rate_window_start_sec_ = now_sec;
      return;
    }
    const double elapsed_sec = now_sec - rate_window_start_sec_;
    if (elapsed_sec < rate_report_period_sec_) {
      return;
    }

    const double input_hz = static_cast<double>(input_imu_window_count_) / elapsed_sec;
    const double corrected_hz = static_cast<double>(output_corrected_window_count_) / elapsed_sec;
    const double bias_hz = static_cast<double>(output_bias_window_count_) / elapsed_sec;
    RCLCPP_INFO(
      get_logger(),
      "IMU bias filter rates input=%.1fHz corrected_out=%.1fHz bias_out=%.1fHz "
      "totals input=%" PRIu64 " corrected=%" PRIu64 " bias=%" PRIu64 " stale_skips=%" PRIu64
      " transform_failures=%" PRIu64,
      input_hz,
      corrected_hz,
      bias_hz,
      input_imu_count_,
      output_corrected_count_,
      output_bias_count_,
      stale_corrected_skip_count_,
      output_transform_failure_count_);
    rate_window_start_sec_ = now_sec;
    input_imu_window_count_ = 0;
    output_corrected_window_count_ = 0;
    output_bias_window_count_ = 0;
  }

  void on_corrected_output_timer()
  {
    const double now_sec = now().seconds();
    if (!latest_corrected_imu_) {
      maybe_report_rates(now_sec);
      return;
    }
    if (corrected_source_age_sec(now_sec) > corrected_output_max_source_age_sec_) {
      ++stale_corrected_skip_count_;
      maybe_report_rates(now_sec);
      return;
    }
    publish_corrected_imu(*latest_corrected_imu_);
    last_corrected_publish_sec_ = now_sec;
    maybe_report_rates(now_sec);
  }

  void on_bias_publish_timer()
  {
    const double now_sec = now().seconds();
    if (latest_bias_msg_) {
      publish_bias_msg(*latest_bias_msg_);
    }
    maybe_report_rates(now_sec);
  }

  void on_odom(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    last_odom_sec_ = now().seconds();
    last_odom_is_stationary_ =
      twist_is_zero(msg->twist.twist, odom_linear_threshold_mps_, odom_angular_threshold_radps_);
    has_odom_ = true;
  }

  void on_cmd_vel(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    last_cmd_vel_sec_ = now().seconds();
    last_cmd_vel_is_stationary_ =
      twist_is_zero(*msg, cmd_linear_threshold_mps_, cmd_angular_threshold_radps_);
    has_cmd_vel_ = true;
  }

  void on_imu(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    const double now_sec = now().seconds();
    ++input_imu_count_;
    ++input_imu_window_count_;

    sensor_msgs::msg::Imu corrected = *msg;
    const bool stationary = stationary_confirmed(now_sec);
    if (stationary && sample_is_safe_for_bias_update(*msg)) {
      update_bias(*msg);
      if (zero_output_when_stationary_) {
        corrected.angular_velocity.x = 0.0;
        corrected.angular_velocity.y = 0.0;
        corrected.angular_velocity.z = 0.0;
      }
    } else if (bias_initialized_) {
      corrected.angular_velocity.x -= bias_.x;
      corrected.angular_velocity.y -= bias_.y;
      corrected.angular_velocity.z -= bias_.z;
    }
    if (!corrected_output_preserve_source_stamp_) {
      corrected.header.stamp = now();
    }
    if (!transform_corrected_output(corrected)) {
      ++output_transform_failure_count_;
      maybe_report_rates(now_sec);
      return;
    }
    latest_corrected_imu_ = corrected;
    latest_imu_receive_sec_ = now_sec;
    latest_bias_msg_ = make_bias_msg(*msg);

    if (!corrected_output_latest_on_timer_) {
      maybe_publish_corrected_direct(now_sec);
    }
    maybe_report_rates(now_sec);
  }

  bool transform_corrected_output(sensor_msgs::msg::Imu & corrected)
  {
    if (!transform_output_to_target_frame_) {
      return true;
    }
    if (corrected.header.frame_id == output_target_frame_) {
      return true;
    }
    if (corrected.header.frame_id.empty()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        5000,
        "Cannot transform corrected IMU output: input frame_id is empty");
      return !drop_output_on_transform_failure_;
    }

    try {
      const auto transform = tf_buffer_->lookupTransform(
        output_target_frame_,
        corrected.header.frame_id,
        rclcpp::Time(0),
        rclcpp::Duration::from_seconds(transform_lookup_timeout_sec_));
      const auto rotation_matrix = rotation_matrix_from_quaternion(transform.transform.rotation);
      corrected.angular_velocity = rotate_vector(corrected.angular_velocity, rotation_matrix);
      corrected.linear_acceleration = rotate_vector(corrected.linear_acceleration, rotation_matrix);
      corrected.angular_velocity_covariance = rotate_covariance(
        corrected.angular_velocity_covariance,
        rotation_matrix);
      corrected.linear_acceleration_covariance = rotate_covariance(
        corrected.linear_acceleration_covariance,
        rotation_matrix);
      corrected.header.frame_id = output_target_frame_;
      return true;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        5000,
        "Cannot transform corrected IMU output from '%s' to '%s': %s",
        corrected.header.frame_id.c_str(),
        output_target_frame_.c_str(),
        ex.what());
      return !drop_output_on_transform_failure_;
    }
  }

  std::string imu_topic_;
  std::string odom_topic_;
  std::string cmd_vel_topic_;
  std::string output_imu_topic_;
  std::string bias_topic_;
  bool transform_output_to_target_frame_{false};
  std::string output_target_frame_{"base_link"};
  double transform_lookup_timeout_sec_{0.02};
  bool drop_output_on_transform_failure_{true};
  bool use_odom_stationary_{true};
  bool use_cmd_vel_stationary_{true};
  bool require_fresh_cmd_vel_{false};
  double odom_timeout_sec_{0.5};
  double cmd_vel_timeout_sec_{0.5};
  double stationary_required_sec_{1.0};
  double odom_linear_threshold_mps_{0.01};
  double odom_angular_threshold_radps_{0.01};
  double cmd_linear_threshold_mps_{0.01};
  double cmd_angular_threshold_radps_{0.01};
  double max_bias_sample_abs_radps_{0.05};
  double accumulator_alpha_{0.02};
  bool zero_output_when_stationary_{true};
  double corrected_output_rate_hz_{100.0};
  double bias_publish_rate_hz_{10.0};
  bool corrected_output_latest_on_timer_{true};
  bool corrected_output_preserve_source_stamp_{true};
  double corrected_output_max_source_age_sec_{0.20};
  bool bias_publish_preserve_source_stamp_{true};

  bool has_odom_{false};
  bool has_cmd_vel_{false};
  bool last_odom_is_stationary_{false};
  bool last_cmd_vel_is_stationary_{false};
  double last_odom_sec_{0.0};
  double last_cmd_vel_sec_{0.0};
  double stationary_since_sec_{0.0};
  bool bias_initialized_{false};
  geometry_msgs::msg::Vector3 bias_;
  std::optional<sensor_msgs::msg::Imu> latest_corrected_imu_;
  std::optional<geometry_msgs::msg::Vector3Stamped> latest_bias_msg_;
  double latest_imu_receive_sec_{0.0};
  double last_corrected_publish_sec_{0.0};
  double rate_window_start_sec_{0.0};
  double rate_report_period_sec_{10.0};
  std::uint64_t input_imu_count_{0};
  std::uint64_t output_corrected_count_{0};
  std::uint64_t output_bias_count_{0};
  std::uint64_t input_imu_window_count_{0};
  std::uint64_t output_corrected_window_count_{0};
  std::uint64_t output_bias_window_count_{0};
  std::uint64_t stale_corrected_skip_count_{0};
  std::uint64_t output_transform_failure_count_{0};

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr corrected_imu_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr bias_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::TimerBase::SharedPtr corrected_output_timer_;
  rclcpp::TimerBase::SharedPtr bias_publish_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImuGyroBiasFilterNode>());
  rclcpp::shutdown();
  return 0;
}
