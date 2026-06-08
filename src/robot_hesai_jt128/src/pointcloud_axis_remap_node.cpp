#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <functional>
#include <iomanip>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "std_msgs/msg/string.hpp"

namespace
{

rclcpp::QoS make_qos(const std::size_t depth, const rmw_qos_reliability_policy_t reliability)
{
  rclcpp::QoS qos{rclcpp::KeepLast(depth)};
  qos.durability_volatile();
  if (reliability == RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT) {
    qos.best_effort();
  } else {
    qos.reliable();
  }
  return qos;
}

bool has_xyz_fields(const sensor_msgs::msg::PointCloud2 & msg)
{
  bool has_x = false;
  bool has_y = false;
  bool has_z = false;
  for (const auto & field : msg.fields) {
    if (field.name == "x") {
      has_x = true;
    } else if (field.name == "y") {
      has_y = true;
    } else if (field.name == "z") {
      has_z = true;
    }
  }
  return has_x && has_y && has_z;
}

bool find_float32_field_offset(
  const sensor_msgs::msg::PointCloud2 & msg,
  const std::string & name,
  std::size_t & offset)
{
  for (const auto & field : msg.fields) {
    if (
      field.name == name &&
      field.datatype == sensor_msgs::msg::PointField::FLOAT32 &&
      field.count >= 1)
    {
      offset = field.offset;
      return true;
    }
  }
  return false;
}

double stamp_to_sec(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1.0e-9;
}

}  // namespace

class PointCloudAxisRemapNode : public rclcpp::Node
{
public:
  explicit PointCloudAxisRemapNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("pointcloud_axis_remap", options), logged_ready_(false), warned_missing_xyz_(false)
  {
    declare_parameter<std::string>("input_topic", "/jt128/vendor/points_raw");
    declare_parameter<std::string>("output_topic", "/lidar_points");
    declare_parameter<std::string>("output_frame_id", "lidar_link");
    declare_parameter<std::string>("nav_output_topic", "");
    declare_parameter<int>("nav_output_stride", 2);
    declare_parameter<int>("nav_output_qos_depth", 1);
    declare_parameter<std::string>("local_output_topic", "");
    declare_parameter<int>("local_output_stride", 4);
    declare_parameter<int>("local_output_qos_depth", 1);
    declare_parameter<int>("input_qos_depth", 1);
    declare_parameter<bool>("input_reliable", false);
    declare_parameter<int>("output_qos_depth", 1);
    declare_parameter<bool>("output_reliable", false);
    declare_parameter<std::string>("status_topic", "/lidar/axis_remap_status");
    declare_parameter<double>("status_publish_period_sec", 1.0);
    declare_parameter<std::vector<double>>(
      "rotation_matrix",
      std::vector<double>{
        1.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
        0.0, 0.0, 1.0,
      });

    input_topic_ = get_parameter("input_topic").as_string();
    output_topic_ = get_parameter("output_topic").as_string();
    output_frame_id_ = get_parameter("output_frame_id").as_string();
    nav_output_topic_ = get_parameter("nav_output_topic").as_string();
    nav_output_stride_ = static_cast<std::size_t>(
      std::max<std::int64_t>(get_parameter("nav_output_stride").as_int(), 1));
    local_output_topic_ = get_parameter("local_output_topic").as_string();
    local_output_stride_ = static_cast<std::size_t>(
      std::max<std::int64_t>(get_parameter("local_output_stride").as_int(), 1));
    const auto nav_output_qos_depth_param = get_parameter("nav_output_qos_depth").as_int();
    const auto local_output_qos_depth_param = get_parameter("local_output_qos_depth").as_int();
    const auto input_qos_depth_param = get_parameter("input_qos_depth").as_int();
    const auto output_qos_depth_param = get_parameter("output_qos_depth").as_int();
    status_topic_ = get_parameter("status_topic").as_string();
    status_publish_period_sec_ = std::max(get_parameter("status_publish_period_sec").as_double(), 0.0);
    nav_output_qos_depth_ = static_cast<std::size_t>(
      nav_output_qos_depth_param > 0 ? nav_output_qos_depth_param : 1);
    local_output_qos_depth_ = static_cast<std::size_t>(
      local_output_qos_depth_param > 0 ? local_output_qos_depth_param : 1);
    input_qos_depth_ = static_cast<std::size_t>(
      input_qos_depth_param > 0 ? input_qos_depth_param : 1);
    output_qos_depth_ = static_cast<std::size_t>(
      output_qos_depth_param > 0 ? output_qos_depth_param : 1);
    rotation_ = load_rotation_matrix();
    const auto input_reliability = get_parameter("input_reliable").as_bool() ?
      RMW_QOS_POLICY_RELIABILITY_RELIABLE : RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
    const auto output_reliability = get_parameter("output_reliable").as_bool() ?
      RMW_QOS_POLICY_RELIABILITY_RELIABLE : RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;

    publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      output_topic_, make_qos(output_qos_depth_, output_reliability));
    if (!nav_output_topic_.empty()) {
      nav_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        nav_output_topic_,
        make_qos(nav_output_qos_depth_, RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT));
    }
    if (!local_output_topic_.empty()) {
      local_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        local_output_topic_,
        make_qos(local_output_qos_depth_, RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT));
    }
    if (!status_topic_.empty() && status_publish_period_sec_ > 0.0) {
      status_publisher_ = create_publisher<std_msgs::msg::String>(status_topic_, 10);
      status_timer_ = create_wall_timer(
        std::chrono::duration<double>(status_publish_period_sec_),
        std::bind(&PointCloudAxisRemapNode::publish_status, this));
      previous_status_time_ = std::chrono::steady_clock::now();
    }
    subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_,
      make_qos(input_qos_depth_, input_reliability),
      std::bind(&PointCloudAxisRemapNode::on_cloud, this, std::placeholders::_1));
  }

private:
  std::array<float, 9> load_rotation_matrix() const
  {
    const auto raw = get_parameter("rotation_matrix").as_double_array();
    if (raw.size() != 9) {
      throw std::runtime_error("rotation_matrix must contain 9 values");
    }

    std::array<float, 9> rotation{};
    for (std::size_t i = 0; i < rotation.size(); ++i) {
      rotation[i] = static_cast<float>(raw[i]);
    }
    return rotation;
  }

  void publish_downsample(
    const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & publisher,
    const std::size_t stride,
    const sensor_msgs::msg::PointCloud2 & cloud)
  {
    if (!publisher || publisher->get_subscription_count() == 0U) {
      return;
    }

    if (stride <= 1U) {
      auto full_cloud = std::make_unique<sensor_msgs::msg::PointCloud2>(cloud);
      publisher->publish(std::move(full_cloud));
      return;
    }

    const std::size_t point_count = static_cast<std::size_t>(cloud.width) * cloud.height;
    auto local_cloud = std::make_unique<sensor_msgs::msg::PointCloud2>();
    local_cloud->header = cloud.header;
    local_cloud->fields = cloud.fields;
    local_cloud->is_bigendian = cloud.is_bigendian;
    local_cloud->point_step = cloud.point_step;
    local_cloud->height = 1;
    local_cloud->is_dense = cloud.is_dense;

    if (point_count == 0U || cloud.point_step == 0U || cloud.data.empty()) {
      local_cloud->width = 0;
      local_cloud->row_step = 0;
      publisher->publish(std::move(local_cloud));
      return;
    }

    const std::size_t sampled_count = (point_count + stride - 1U) / stride;
    local_cloud->width = static_cast<std::uint32_t>(sampled_count);
    local_cloud->row_step = static_cast<std::uint32_t>(sampled_count * cloud.point_step);
    local_cloud->data.resize(static_cast<std::size_t>(local_cloud->row_step));

    std::size_t output_offset = 0U;
    for (std::size_t flat_index = 0U; flat_index < point_count; flat_index += stride) {
      const std::size_t row = flat_index / cloud.width;
      const std::size_t column = flat_index % cloud.width;
      const std::size_t input_offset = row * cloud.row_step + column * cloud.point_step;
      if (input_offset + cloud.point_step > cloud.data.size()) {
        break;
      }
      std::memcpy(
        local_cloud->data.data() + output_offset,
        cloud.data.data() + input_offset,
        cloud.point_step);
      output_offset += cloud.point_step;
    }

    if (output_offset != local_cloud->data.size()) {
      const std::size_t actual_count = output_offset / cloud.point_step;
      local_cloud->width = static_cast<std::uint32_t>(actual_count);
      local_cloud->row_step = static_cast<std::uint32_t>(output_offset);
      local_cloud->data.resize(output_offset);
    }
    publisher->publish(std::move(local_cloud));
  }

  void publish_outputs(std::unique_ptr<sensor_msgs::msg::PointCloud2> output)
  {
    const auto publish_start = std::chrono::steady_clock::now();
    publisher_->publish(*output);
    ++lidar_points_publish_count_;
    last_cloud_points_ = static_cast<std::size_t>(output->width) * output->height;
    last_cloud_bytes_ = output->data.size();
    last_output_stamp_sec_ = stamp_to_sec(output->header.stamp);
    last_output_subscription_count_ = publisher_->get_subscription_count();
    publish_downsample(local_publisher_, local_output_stride_, *output);
    publish_downsample(nav_publisher_, nav_output_stride_, *output);
    const auto publish_end = std::chrono::steady_clock::now();
    last_publish_time_ = publish_end;
    last_publish_duration_ms_ =
      std::chrono::duration<double, std::milli>(publish_end - publish_start).count();

    if (!logged_ready_) {
      RCLCPP_INFO(
        get_logger(),
        "compiled canonical pointcloud remap ready: %s -> %s frame=%s nav_output=%s stride=%zu local_output=%s stride=%zu",
        input_topic_.c_str(),
        output_topic_.c_str(),
        output_frame_id_.c_str(),
        nav_output_topic_.empty() ? "(disabled)" : nav_output_topic_.c_str(),
        nav_output_stride_,
        local_output_topic_.empty() ? "(disabled)" : local_output_topic_.c_str(),
        local_output_stride_);
      logged_ready_ = true;
    }
  }

  void on_cloud(sensor_msgs::msg::PointCloud2::UniquePtr msg)
  {
    ++raw_input_count_;
    last_raw_callback_time_ = std::chrono::steady_clock::now();
    last_raw_stamp_sec_ = stamp_to_sec(msg->header.stamp);
    auto output = std::move(msg);
    output->header.frame_id = output_frame_id_;

    if (output->data.empty() || output->width == 0U || output->height == 0U) {
      publish_outputs(std::move(output));
      return;
    }

    if (!has_xyz_fields(*output)) {
      if (!warned_missing_xyz_) {
        RCLCPP_ERROR(
          get_logger(), "cloud on %s does not expose x/y/z fields", input_topic_.c_str());
        warned_missing_xyz_ = true;
      }
      ++dropped_or_skipped_count_;
      return;
    }

    std::size_t x_offset = 0;
    std::size_t y_offset = 0;
    std::size_t z_offset = 0;
    const std::size_t point_count = static_cast<std::size_t>(output->width) * output->height;
    const bool fast_path_raw_y_neg_raw_x =
      rotation_[0] == 0.0F && rotation_[1] == 1.0F && rotation_[2] == 0.0F &&
      rotation_[3] == -1.0F && rotation_[4] == 0.0F && rotation_[5] == 0.0F &&
      rotation_[6] == 0.0F && rotation_[7] == 0.0F && rotation_[8] == 1.0F;
    const bool fast_path_neg_raw_y_neg_raw_x =
      rotation_[0] == 0.0F && rotation_[1] == -1.0F && rotation_[2] == 0.0F &&
      rotation_[3] == -1.0F && rotation_[4] == 0.0F && rotation_[5] == 0.0F &&
      rotation_[6] == 0.0F && rotation_[7] == 0.0F && rotation_[8] == 1.0F;
    const bool fast_path_identity =
      rotation_[0] == 1.0F && rotation_[1] == 0.0F && rotation_[2] == 0.0F &&
      rotation_[3] == 0.0F && rotation_[4] == 1.0F && rotation_[5] == 0.0F &&
      rotation_[6] == 0.0F && rotation_[7] == 0.0F && rotation_[8] == 1.0F;

    if (fast_path_identity) {
      publish_outputs(std::move(output));
      return;
    }

    if (
      !output->is_bigendian &&
      find_float32_field_offset(*output, "x", x_offset) &&
      find_float32_field_offset(*output, "y", y_offset) &&
      find_float32_field_offset(*output, "z", z_offset) &&
      x_offset == 0U && y_offset == sizeof(float) && z_offset == 2U * sizeof(float) &&
      output->point_step >= 3U * sizeof(float) &&
      (fast_path_raw_y_neg_raw_x || fast_path_neg_raw_y_neg_raw_x))
    {
      for (std::size_t index = 0; index < point_count; ++index) {
        auto * point = output->data.data() + index * output->point_step;
        auto * xyz = reinterpret_cast<float *>(point);
        const float raw_x = xyz[0];
        const float raw_y = xyz[1];
        xyz[0] = fast_path_raw_y_neg_raw_x ? raw_y : -raw_y;
        xyz[1] = -raw_x;
      }
      publish_outputs(std::move(output));
      return;
    }

    sensor_msgs::PointCloud2Iterator<float> iter_x(*output, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(*output, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(*output, "z");

    for (std::size_t index = 0; index < point_count; ++index, ++iter_x, ++iter_y, ++iter_z) {
      const float x = *iter_x;
      const float y = *iter_y;
      const float z = *iter_z;

      *iter_x = rotation_[0] * x + rotation_[1] * y + rotation_[2] * z;
      *iter_y = rotation_[3] * x + rotation_[4] * y + rotation_[5] * z;
      *iter_z = rotation_[6] * x + rotation_[7] * y + rotation_[8] * z;
    }

    publish_outputs(std::move(output));
  }

  void publish_status()
  {
    if (!status_publisher_) {
      return;
    }

    const auto now_steady = std::chrono::steady_clock::now();
    const double elapsed_sec = std::max(
      std::chrono::duration<double>(now_steady - previous_status_time_).count(), 1.0e-3);
    const auto raw_delta = raw_input_count_ - previous_raw_input_count_;
    const auto publish_delta = lidar_points_publish_count_ - previous_lidar_points_publish_count_;
    const double raw_age_ms = last_raw_callback_time_ == std::chrono::steady_clock::time_point{} ?
      -1.0 :
      std::chrono::duration<double, std::milli>(now_steady - last_raw_callback_time_).count();
    const double publish_age_ms = last_publish_time_ == std::chrono::steady_clock::time_point{} ?
      -1.0 :
      std::chrono::duration<double, std::milli>(now_steady - last_publish_time_).count();
    const double raw_stamp_age_ms = last_raw_stamp_sec_ <= 0.0 ?
      -1.0 :
      (now().seconds() - last_raw_stamp_sec_) * 1000.0;
    const double publish_stamp_age_ms = last_output_stamp_sec_ <= 0.0 ?
      -1.0 :
      (now().seconds() - last_output_stamp_sec_) * 1000.0;
    const double uptime_sec = std::chrono::duration<double>(now_steady - start_time_).count();

    std_msgs::msg::String msg;
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(3)
           << "input_topic=" << input_topic_
           << " output_topic=" << output_topic_
           << " raw_input_hz=" << static_cast<double>(raw_delta) / elapsed_sec
           << " lidar_points_publish_hz=" << static_cast<double>(publish_delta) / elapsed_sec
           << " last_raw_age_ms=" << raw_age_ms
           << " last_raw_stamp_age_ms=" << raw_stamp_age_ms
           << " last_publish_age_ms=" << publish_age_ms
           << " last_publish_stamp_age_ms=" << publish_stamp_age_ms
           << " last_publish_duration_ms=" << last_publish_duration_ms_
           << " last_cloud_points=" << last_cloud_points_
           << " last_cloud_bytes=" << last_cloud_bytes_
           << " output_subscription_count=" << last_output_subscription_count_
           << " nav_branch_enabled=" << (nav_publisher_ ? "true" : "false")
           << " local_branch_enabled=" << (local_publisher_ ? "true" : "false")
           << " dropped_or_skipped_count=" << dropped_or_skipped_count_
           << " node_uptime_sec=" << uptime_sec;
    msg.data = stream.str();
    status_publisher_->publish(msg);

    previous_status_time_ = now_steady;
    previous_raw_input_count_ = raw_input_count_;
    previous_lidar_points_publish_count_ = lidar_points_publish_count_;
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string output_frame_id_;
  std::string nav_output_topic_;
  std::size_t nav_output_stride_;
  std::size_t nav_output_qos_depth_;
  std::string local_output_topic_;
  std::size_t local_output_stride_;
  std::size_t local_output_qos_depth_;
  std::size_t input_qos_depth_;
  std::size_t output_qos_depth_;
  std::string status_topic_;
  double status_publish_period_sec_{1.0};
  std::array<float, 9> rotation_{};
  bool logged_ready_;
  bool warned_missing_xyz_;
  std::chrono::steady_clock::time_point start_time_{std::chrono::steady_clock::now()};
  std::chrono::steady_clock::time_point previous_status_time_{std::chrono::steady_clock::now()};
  std::chrono::steady_clock::time_point last_raw_callback_time_{};
  std::chrono::steady_clock::time_point last_publish_time_{};
  std::uint64_t raw_input_count_{0};
  std::uint64_t lidar_points_publish_count_{0};
  std::uint64_t previous_raw_input_count_{0};
  std::uint64_t previous_lidar_points_publish_count_{0};
  std::uint64_t dropped_or_skipped_count_{0};
  std::size_t last_cloud_points_{0};
  std::size_t last_cloud_bytes_{0};
  std::size_t last_output_subscription_count_{0};
  double last_raw_stamp_sec_{0.0};
  double last_output_stamp_sec_{0.0};
  double last_publish_duration_ms_{0.0};
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr nav_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr local_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr status_timer_;
};

#ifndef ROBOT_HESAI_JT128_COMPONENT_ONLY
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PointCloudAxisRemapNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
#endif

RCLCPP_COMPONENTS_REGISTER_NODE(PointCloudAxisRemapNode)
