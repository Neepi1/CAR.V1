#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <sstream>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/exceptions.h"
#include "tf2/time.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace
{

double stamp_to_sec(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1.0e-9;
}

std::string json_escape(const std::string & input)
{
  std::string output;
  output.reserve(input.size());
  for (const char c : input) {
    switch (c) {
      case '\\':
        output += "\\\\";
        break;
      case '"':
        output += "\\\"";
        break;
      case '\n':
        output += "\\n";
        break;
      case '\r':
        output += "\\r";
        break;
      case '\t':
        output += "\\t";
        break;
      default:
        output += c;
        break;
    }
  }
  return output;
}

std::string bool_text(const bool value)
{
  return value ? "true" : "false";
}

}  // namespace

class AmclScanAdmissionNode : public rclcpp::Node
{
public:
  AmclScanAdmissionNode()
  : Node("amcl_scan_admission"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/scan");
    output_topic_ = declare_parameter<std::string>("output_topic", "/scan_amcl");
    status_topic_ = declare_parameter<std::string>("status_topic", "/amcl_scan_admission/status");
    target_frame_ = declare_parameter<std::string>("target_frame", "odom");
    frame_required_ = declare_parameter<std::string>("frame_required", "lidar_level_link");

    max_rate_hz_ = declare_parameter<double>("max_rate_hz", 5.0);
    const double legacy_rate_hz = declare_parameter<double>("rate_hz", max_rate_hz_);
    max_rate_hz_ = std::max(0.1, legacy_rate_hz);

    max_scan_age_ms_ = declare_parameter<double>("max_scan_age_ms", 250.0);
    const double legacy_max_age_ms = declare_parameter<double>("max_age_ms", max_scan_age_ms_);
    max_scan_age_ms_ = std::max(0.0, legacy_max_age_ms);

    tf_wait_timeout_ms_ = declare_parameter<double>("tf_wait_timeout_ms", 20.0);
    const double legacy_wait_ms =
      declare_parameter<double>("wait_for_tf_timeout_ms", tf_wait_timeout_ms_);
    tf_wait_timeout_ms_ = std::max(0.0, legacy_wait_ms);

    require_tf_available_ = declare_parameter<bool>("require_tf_available", true);
    const bool legacy_drop_tf =
      declare_parameter<bool>("drop_if_tf_unavailable", require_tf_available_);
    require_tf_available_ = legacy_drop_tf;

    preserve_stamp_ = declare_parameter<bool>("preserve_stamp", true);
    require_seeded_ = declare_parameter<bool>("require_seeded", false);
    require_tf_warmup_ = declare_parameter<bool>("require_tf_warmup", false);
    startup_warmup_sec_ = std::max(0.0, declare_parameter<double>("startup_warmup_sec", 0.0));
    status_log_period_sec_ =
      std::max(0.5, declare_parameter<double>("status_log_period_sec", 5.0));
    drop_if_future_stamp_ = declare_parameter<bool>("drop_if_future_stamp", true);
    max_future_stamp_ms_ =
      std::max(0.0, declare_parameter<double>("max_future_stamp_ms", 50.0));

    if (!preserve_stamp_) {
      RCLCPP_WARN(
        get_logger(),
        "preserve_stamp=false was requested, but amcl_scan_admission_node never restamps scans; "
        "the original LaserScan header.stamp will be preserved.");
    }
    if (require_seeded_) {
      RCLCPP_WARN(
        get_logger(),
        "require_seeded=true requested but no seed-state input is wired to this node; "
        "run_amcl_shadow_localization.sh still starts the relay only after seed.");
    }

    pub_ = create_publisher<sensor_msgs::msg::LaserScan>(
      output_topic_, rclcpp::SensorDataQoS());
    status_pub_ = create_publisher<std_msgs::msg::String>(status_topic_, rclcpp::QoS(10));
    sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      input_topic_, rclcpp::SensorDataQoS(),
      std::bind(&AmclScanAdmissionNode::onScan, this, std::placeholders::_1));
    status_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(status_log_period_sec_)),
      std::bind(&AmclScanAdmissionNode::publishStatus, this));

    startup_time_ = std::chrono::steady_clock::now();
    last_status_time_ = startup_time_;

    RCLCPP_INFO(
      get_logger(),
      "AMCL scan admission node implementation=cpp input=%s output=%s max_rate_hz=%.2f "
      "max_scan_age_ms=%.1f tf_wait_timeout_ms=%.1f target_frame=%s frame_required=%s "
      "preserve_stamp=true require_tf_available=%s",
      input_topic_.c_str(), output_topic_.c_str(), max_rate_hz_, max_scan_age_ms_,
      tf_wait_timeout_ms_, target_frame_.c_str(), frame_required_.c_str(),
      bool_text(require_tf_available_).c_str());
  }

private:
  double scanAgeMs(const sensor_msgs::msg::LaserScan & msg)
  {
    return (get_clock()->now().seconds() - stamp_to_sec(msg.header.stamp)) * 1000.0;
  }

  bool startupWarmupReady()
  {
    if (!require_tf_warmup_ && startup_warmup_sec_ <= 0.0) {
      return true;
    }
    const auto elapsed =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - startup_time_).count();
    if (elapsed >= startup_warmup_sec_) {
      return true;
    }
    ++dropped_warmup_count_;
    last_error_ = "AMCL_SCAN_WARMUP";
    return false;
  }

  bool tfReady(const sensor_msgs::msg::LaserScan & msg)
  {
    if (!require_tf_available_) {
      return true;
    }
    try {
      (void)tf_buffer_.lookupTransform(
        target_frame_, msg.header.frame_id, msg.header.stamp,
        tf2::durationFromSec(tf_wait_timeout_ms_ / 1000.0));
      return true;
    } catch (const tf2::TransformException & ex) {
      ++dropped_tf_count_;
      last_error_ = std::string("AMCL_SCAN_TF_UNAVAILABLE: ") + ex.what();
      return false;
    }
  }

  void onScan(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    ++input_count_;
    last_frame_id_ = msg->header.frame_id;
    last_age_ms_ = scanAgeMs(*msg);

    if (!startupWarmupReady()) {
      return;
    }

    if (
      !frame_required_.empty() && frame_required_ != "auto" &&
      msg->header.frame_id != frame_required_)
    {
      ++dropped_frame_count_;
      last_error_ = "AMCL_SCAN_FRAME_MISMATCH";
      return;
    }

    if (drop_if_future_stamp_ && last_age_ms_ < -max_future_stamp_ms_) {
      ++dropped_future_count_;
      last_error_ = "AMCL_SCAN_FUTURE_STAMP";
      return;
    }

    if (max_scan_age_ms_ > 0.0 && last_age_ms_ > max_scan_age_ms_) {
      ++dropped_age_count_;
      last_error_ = "AMCL_SCAN_STALE";
      return;
    }

    const auto now = std::chrono::steady_clock::now();
    const auto min_period = std::chrono::duration<double>(1.0 / max_rate_hz_);
    if (has_last_publish_ && now - last_publish_time_ < min_period) {
      ++dropped_rate_count_;
      return;
    }

    if (!tfReady(*msg)) {
      return;
    }

    pub_->publish(*msg);
    ++published_count_;
    last_publish_time_ = now;
    has_last_publish_ = true;
    last_error_ = "none";
  }

  void publishStatus()
  {
    const auto now = std::chrono::steady_clock::now();
    const double elapsed = std::max(
      1.0e-6, std::chrono::duration<double>(now - last_status_time_).count());
    const double hz =
      static_cast<double>(published_count_ - previous_published_count_) / elapsed;
    previous_published_count_ = published_count_;
    last_status_time_ = now;

    std::ostringstream json;
    json.setf(std::ios::fixed);
    json.precision(3);
    json << "{"
         << "\"dropped_age_count\":" << dropped_age_count_ << ","
         << "\"dropped_frame_count\":" << dropped_frame_count_ << ","
         << "\"dropped_future_count\":" << dropped_future_count_ << ","
         << "\"dropped_rate_count\":" << dropped_rate_count_ << ","
         << "\"dropped_tf_count\":" << dropped_tf_count_ << ","
         << "\"dropped_warmup_count\":" << dropped_warmup_count_ << ","
         << "\"enabled\":true,"
         << "\"frame_id\":\"" << json_escape(last_frame_id_) << "\","
         << "\"hz\":" << hz << ","
         << "\"implementation\":\"cpp\","
         << "\"input_count\":" << input_count_ << ","
         << "\"input_topic\":\"" << json_escape(input_topic_) << "\","
         << "\"last_age_ms\":" << last_age_ms_ << ","
         << "\"last_error\":\"" << json_escape(last_error_) << "\","
         << "\"max_future_stamp_ms\":" << max_future_stamp_ms_ << ","
         << "\"max_scan_age_ms\":" << max_scan_age_ms_ << ","
         << "\"message_filter_drop_detected\":false,"
         << "\"output_topic\":\"" << json_escape(output_topic_) << "\","
         << "\"preserve_stamp\":true,"
         << "\"published_count\":" << published_count_ << ","
         << "\"require_tf_available\":" << bool_text(require_tf_available_) << ","
         << "\"target_frame\":\"" << json_escape(target_frame_) << "\","
         << "\"tf_wait_timeout_ms\":" << tf_wait_timeout_ms_
         << "}";
    std_msgs::msg::String msg;
    msg.data = json.str();
    status_pub_->publish(msg);

    RCLCPP_INFO(
      get_logger(),
      "AMCL scan admission status implementation=cpp input=%lu published=%lu "
      "drop_rate=%lu drop_age=%lu drop_future=%lu drop_tf=%lu hz=%.2f last_age_ms=%.1f "
      "last_error=%s",
      input_count_, published_count_, dropped_rate_count_, dropped_age_count_,
      dropped_future_count_, dropped_tf_count_, hz, last_age_ms_, last_error_.c_str());
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string status_topic_;
  std::string target_frame_;
  std::string frame_required_;
  double max_rate_hz_{5.0};
  double max_scan_age_ms_{250.0};
  double tf_wait_timeout_ms_{20.0};
  bool require_tf_available_{true};
  bool preserve_stamp_{true};
  bool require_seeded_{false};
  bool require_tf_warmup_{false};
  double startup_warmup_sec_{0.0};
  double status_log_period_sec_{5.0};
  bool drop_if_future_stamp_{true};
  double max_future_stamp_ms_{50.0};

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr status_timer_;

  std::chrono::steady_clock::time_point startup_time_;
  std::chrono::steady_clock::time_point last_publish_time_;
  std::chrono::steady_clock::time_point last_status_time_;
  bool has_last_publish_{false};

  std::uint64_t input_count_{0};
  std::uint64_t published_count_{0};
  std::uint64_t previous_published_count_{0};
  std::uint64_t dropped_age_count_{0};
  std::uint64_t dropped_future_count_{0};
  std::uint64_t dropped_tf_count_{0};
  std::uint64_t dropped_rate_count_{0};
  std::uint64_t dropped_frame_count_{0};
  std::uint64_t dropped_warmup_count_{0};
  double last_age_ms_{-1.0};
  std::string last_frame_id_;
  std::string last_error_{"none"};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AmclScanAdmissionNode>());
  rclcpp::shutdown();
  return 0;
}
