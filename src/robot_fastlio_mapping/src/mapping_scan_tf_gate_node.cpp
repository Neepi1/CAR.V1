#include <algorithm>
#include <chrono>
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <sstream>
#include <string>

#include "builtin_interfaces/msg/time.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/buffer_core.h"
#include "tf2/exceptions.h"
#include "tf2_msgs/msg/tf_message.hpp"
#include "tf2_ros/buffer.h"

#include "robot_fastlio_mapping/mapping_scan_tf_gate_policy.hpp"

namespace
{

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

std::int64_t stamp_to_nanoseconds(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<std::int64_t>(stamp.sec) * 1000000000LL +
         static_cast<std::int64_t>(stamp.nanosec);
}

}  // namespace

class MappingScanTfGateNode : public rclcpp::Node
{
public:
  using Policy = robot_fastlio_mapping::MappingScanTfGatePolicy;
  using Decision = robot_fastlio_mapping::ScanTfGateDecision;
  using SteadyClock = Policy::Clock;

  MappingScanTfGateNode()
  : Node("mapping_scan_tf_gate"),
    tf_buffer_(get_clock())
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/mapping/scan_raw");
    output_topic_ = declare_parameter<std::string>("output_topic", "/mapping/scan");
    status_topic_ =
      declare_parameter<std::string>("status_topic", "/mapping/scan_tf_gate/status");
    target_frame_ = declare_parameter<std::string>("target_frame", "mapping_odom");
    required_frame_ = declare_parameter<std::string>("required_frame", "lidar_level_link");
    tf_topic_ = declare_parameter<std::string>("tf_topic", "/tf_slam2d");
    tf_static_topic_ = declare_parameter<std::string>("tf_static_topic", "/tf_static");
    max_wait_sec_ = std::max(0.0, declare_parameter<double>("max_wait_sec", 2.0));
    post_tf_settle_ms_ =
      std::max(0.0, declare_parameter<double>("post_tf_settle_ms", 20.0));
    poll_period_ms_ = std::max(1.0, declare_parameter<double>("poll_period_ms", 10.0));
    status_period_sec_ =
      std::max(0.2, declare_parameter<double>("status_period_sec", 1.0));
    max_queue_size_ = static_cast<std::size_t>(std::max<std::int64_t>(
      1, declare_parameter<std::int64_t>("max_queue_size", 50)));
    preserve_stamp_ = declare_parameter<bool>("preserve_stamp", true);

    if (!preserve_stamp_) {
      RCLCPP_WARN(
        get_logger(),
        "preserve_stamp=false was requested, but mapping_scan_tf_gate never restamps scans; "
        "the original LaserScan header.stamp remains authoritative.");
    }

    policy_ = std::make_unique<Policy>(
      std::chrono::duration<double>(max_wait_sec_),
      std::chrono::duration<double>(post_tf_settle_ms_ / 1000.0));

    auto scan_input_qos = rclcpp::SensorDataQoS();
    auto scan_output_qos = rclcpp::QoS(rclcpp::KeepLast(10));
    scan_output_qos.reliable().durability_volatile();
    auto dynamic_tf_qos = rclcpp::QoS(rclcpp::KeepLast(100));
    dynamic_tf_qos.best_effort().durability_volatile();
    auto static_tf_qos = rclcpp::QoS(rclcpp::KeepLast(1));
    static_tf_qos.reliable().transient_local();
    auto status_qos = rclcpp::QoS(rclcpp::KeepLast(1));
    status_qos.reliable().transient_local();

    scan_pub_ = create_publisher<sensor_msgs::msg::LaserScan>(output_topic_, scan_output_qos);
    status_pub_ = create_publisher<std_msgs::msg::String>(status_topic_, status_qos);
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      input_topic_, scan_input_qos,
      std::bind(&MappingScanTfGateNode::on_scan, this, std::placeholders::_1));
    tf_sub_ = create_subscription<tf2_msgs::msg::TFMessage>(
      tf_topic_, dynamic_tf_qos,
      [this](const tf2_msgs::msg::TFMessage::SharedPtr msg) {on_tf(msg, false);});
    tf_static_sub_ = create_subscription<tf2_msgs::msg::TFMessage>(
      tf_static_topic_, static_tf_qos,
      [this](const tf2_msgs::msg::TFMessage::SharedPtr msg) {on_tf(msg, true);});

    poll_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double, std::milli>(poll_period_ms_)),
      std::bind(&MappingScanTfGateNode::process_queue, this));
    status_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(status_period_sec_)),
      std::bind(&MappingScanTfGateNode::publish_status, this));

    RCLCPP_INFO(
      get_logger(),
      "mapping scan/TF gate input=%s output=%s target=%s tf=%s max_wait=%.3fs "
      "post_tf_settle=%.1fms max_queue=%zu preserve_stamp=true",
      input_topic_.c_str(), output_topic_.c_str(), target_frame_.c_str(), tf_topic_.c_str(),
      max_wait_sec_, post_tf_settle_ms_, max_queue_size_);
  }

private:
  struct PendingScan
  {
    sensor_msgs::msg::LaserScan::ConstSharedPtr scan;
    Policy::Timing timing;
  };

  bool transform_available(
    const sensor_msgs::msg::LaserScan & scan,
    std::string * error)
  {
    try {
      const tf2::TimePoint stamp{
        std::chrono::nanoseconds(stamp_to_nanoseconds(scan.header.stamp))};
      return static_cast<const tf2::BufferCore &>(tf_buffer_).canTransform(
        target_frame_, scan.header.frame_id, stamp, error);
    } catch (const tf2::TransformException & ex) {
      if (error != nullptr) {
        *error = ex.what();
      }
      return false;
    }
  }

  void on_scan(const sensor_msgs::msg::LaserScan::ConstSharedPtr msg)
  {
    ++input_count_;
    last_input_stamp_ns_ = stamp_to_nanoseconds(msg->header.stamp);
    last_frame_id_ = msg->header.frame_id;

    if (
      msg->header.frame_id.empty() || last_input_stamp_ns_ <= 0 ||
      (!required_frame_.empty() && msg->header.frame_id != required_frame_))
    {
      ++dropped_invalid_count_;
      last_error_ = "SCAN_HEADER_INVALID";
      return;
    }

    process_queue();
    if (pending_.size() >= max_queue_size_) {
      pending_.pop_front();
      ++dropped_queue_overflow_count_;
      last_error_ = "SCAN_TF_QUEUE_OVERFLOW";
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "mapping scan/TF gate queue overflow; dropping oldest scan without restamping");
    }

    const auto arrival = SteadyClock::now();
    PendingScan pending{msg, Policy::Timing{arrival, std::nullopt}};
    std::string tf_error;
    if (transform_available(*msg, &tf_error)) {
      policy_->mark_tf_ready_before_enqueue(pending.timing);
    } else {
      last_tf_error_ = tf_error;
    }
    pending_.push_back(std::move(pending));
    max_queue_depth_ = std::max(max_queue_depth_, pending_.size());
    process_queue();
  }

  void on_tf(const tf2_msgs::msg::TFMessage::SharedPtr msg, const bool is_static)
  {
    for (const auto & transform : msg->transforms) {
      try {
        if (tf_buffer_.setTransform(transform, "mapping_scan_tf_gate", is_static)) {
          ++tf_accepted_count_;
        } else {
          ++tf_rejected_count_;
        }
      } catch (const tf2::TransformException & ex) {
        ++tf_rejected_count_;
        last_tf_error_ = ex.what();
      }
    }
    process_queue();
  }

  void process_queue()
  {
    while (!pending_.empty()) {
      auto & pending = pending_.front();
      const auto now = SteadyClock::now();
      std::string tf_error;
      const bool ready = transform_available(*pending.scan, &tf_error);
      const auto decision = policy_->evaluate(pending.timing, now, ready);

      if (decision == Decision::wait) {
        if (!ready) {
          last_tf_error_ = tf_error;
          last_error_ = "WAITING_FOR_ORIGINAL_STAMP_TF";
        }
        return;
      }

      if (decision == Decision::drop_timeout) {
        ++dropped_tf_timeout_count_;
        last_error_ = "ORIGINAL_STAMP_TF_TIMEOUT";
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "dropping mapping scan only after %.3fs bounded wait for %s <- %s at original "
          "stamp; tf_error=%s",
          max_wait_sec_, target_frame_.c_str(), pending.scan->header.frame_id.c_str(),
          tf_error.c_str());
        pending_.pop_front();
        continue;
      }

      last_output_stamp_ns_ = stamp_to_nanoseconds(pending.scan->header.stamp);
      scan_pub_->publish(*pending.scan);
      ++published_count_;
      last_error_ = "none";
      last_tf_error_.clear();
      pending_.pop_front();
    }
  }

  void publish_status()
  {
    double oldest_wait_ms = 0.0;
    if (!pending_.empty()) {
      oldest_wait_ms = std::chrono::duration<double, std::milli>(
        SteadyClock::now() - pending_.front().timing.arrival).count();
    }

    std::ostringstream json;
    json.setf(std::ios::fixed);
    json.precision(3);
    json << "{"
         << "\"dropped_invalid_count\":" << dropped_invalid_count_ << ","
         << "\"dropped_queue_overflow_count\":" << dropped_queue_overflow_count_ << ","
         << "\"dropped_tf_timeout_count\":" << dropped_tf_timeout_count_ << ","
         << "\"frame_id\":\"" << json_escape(last_frame_id_) << "\","
         << "\"input_count\":" << input_count_ << ","
         << "\"last_error\":\"" << json_escape(last_error_) << "\","
         << "\"last_input_stamp_ns\":" << last_input_stamp_ns_ << ","
         << "\"last_output_stamp_ns\":" << last_output_stamp_ns_ << ","
         << "\"last_tf_error\":\"" << json_escape(last_tf_error_) << "\","
         << "\"max_queue_depth\":" << max_queue_depth_ << ","
         << "\"oldest_wait_ms\":" << oldest_wait_ms << ","
         << "\"pending_count\":" << pending_.size() << ","
         << "\"preserve_stamp\":true,"
         << "\"published_count\":" << published_count_ << ","
         << "\"target_frame\":\"" << json_escape(target_frame_) << "\","
         << "\"tf_accepted_count\":" << tf_accepted_count_ << ","
         << "\"tf_rejected_count\":" << tf_rejected_count_
         << "}";
    std_msgs::msg::String status;
    status.data = json.str();
    status_pub_->publish(status);
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string status_topic_;
  std::string target_frame_;
  std::string required_frame_;
  std::string tf_topic_;
  std::string tf_static_topic_;
  double max_wait_sec_{2.0};
  double post_tf_settle_ms_{20.0};
  double poll_period_ms_{10.0};
  double status_period_sec_{1.0};
  std::size_t max_queue_size_{50};
  bool preserve_stamp_{true};

  tf2_ros::Buffer tf_buffer_;
  std::unique_ptr<Policy> policy_;
  std::deque<PendingScan> pending_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_sub_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_static_sub_;
  rclcpp::TimerBase::SharedPtr poll_timer_;
  rclcpp::TimerBase::SharedPtr status_timer_;

  std::uint64_t input_count_{0};
  std::uint64_t published_count_{0};
  std::uint64_t dropped_invalid_count_{0};
  std::uint64_t dropped_queue_overflow_count_{0};
  std::uint64_t dropped_tf_timeout_count_{0};
  std::uint64_t tf_accepted_count_{0};
  std::uint64_t tf_rejected_count_{0};
  std::size_t max_queue_depth_{0};
  std::int64_t last_input_stamp_ns_{0};
  std::int64_t last_output_stamp_ns_{0};
  std::string last_frame_id_;
  std::string last_error_{"none"};
  std::string last_tf_error_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MappingScanTfGateNode>());
  rclcpp::shutdown();
  return 0;
}
