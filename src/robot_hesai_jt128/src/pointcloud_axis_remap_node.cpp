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

struct BranchPublishResult
{
  bool published{false};
  bool skipped{false};
  std::size_t subscription_count{0};
  std::size_t points{0};
  std::size_t bytes{0};
  double duration_ms{0.0};
};

struct BranchDiagnostics
{
  std::uint64_t attempt_count{0};
  std::uint64_t publish_count{0};
  std::uint64_t skip_count{0};
  std::uint64_t previous_attempt_count{0};
  std::uint64_t previous_publish_count{0};
  std::uint64_t previous_skip_count{0};
  std::chrono::steady_clock::time_point last_publish_time{};
  double last_publish_duration_ms{0.0};
  std::size_t last_points{0};
  std::size_t last_bytes{0};
  std::size_t subscription_count{0};
};

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
    declare_parameter<int>("nav_output_publish_every_n", 1);
    declare_parameter<int>("nav_output_qos_depth", 1);
    declare_parameter<std::string>("local_output_topic", "");
    declare_parameter<int>("local_output_stride", 4);
    declare_parameter<int>("local_output_publish_every_n", 1);
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
    nav_output_publish_every_n_ = static_cast<std::size_t>(
      std::max<std::int64_t>(get_parameter("nav_output_publish_every_n").as_int(), 1));
    local_output_topic_ = get_parameter("local_output_topic").as_string();
    local_output_stride_ = static_cast<std::size_t>(
      std::max<std::int64_t>(get_parameter("local_output_stride").as_int(), 1));
    local_output_publish_every_n_ = static_cast<std::size_t>(
      std::max<std::int64_t>(get_parameter("local_output_publish_every_n").as_int(), 1));
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

  BranchPublishResult publish_downsample(
    const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & publisher,
    const std::size_t stride,
    const std::size_t publish_every_n,
    const sensor_msgs::msg::PointCloud2 & cloud)
  {
    BranchPublishResult result;
    if (!publisher) {
      return result;
    }

    result.subscription_count = publisher->get_subscription_count();
    if (result.subscription_count == 0U) {
      return result;
    }

    if (publish_every_n > 1U && lidar_points_publish_count_ % publish_every_n != 0U) {
      result.skipped = true;
      return result;
    }

    const auto branch_start = std::chrono::steady_clock::now();
    if (stride <= 1U) {
      auto full_cloud = std::make_unique<sensor_msgs::msg::PointCloud2>(cloud);
      result.points = static_cast<std::size_t>(full_cloud->width) * full_cloud->height;
      result.bytes = full_cloud->data.size();
      publisher->publish(std::move(full_cloud));
      result.published = true;
      result.duration_ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - branch_start).count();
      return result;
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
      result.published = true;
      result.duration_ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - branch_start).count();
      return result;
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
    result.points = static_cast<std::size_t>(local_cloud->width) * local_cloud->height;
    result.bytes = local_cloud->data.size();
    publisher->publish(std::move(local_cloud));
    result.published = true;
    result.duration_ms =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - branch_start).count();
    return result;
  }

  void update_branch_diagnostics(
    BranchDiagnostics & diagnostics,
    const BranchPublishResult & result,
    const std::chrono::steady_clock::time_point & now_steady)
  {
    ++diagnostics.attempt_count;
    diagnostics.subscription_count = result.subscription_count;
    if (result.skipped) {
      ++diagnostics.skip_count;
    }
    if (result.published) {
      ++diagnostics.publish_count;
      diagnostics.last_publish_time = now_steady;
      diagnostics.last_publish_duration_ms = result.duration_ms;
      diagnostics.last_points = result.points;
      diagnostics.last_bytes = result.bytes;
    }
  }

  void publish_outputs(std::unique_ptr<sensor_msgs::msg::PointCloud2> output)
  {
    const auto total_start = std::chrono::steady_clock::now();
    if (last_publish_time_ != std::chrono::steady_clock::time_point{}) {
      const double publish_interval_ms =
        std::chrono::duration<double, std::milli>(total_start - last_publish_time_).count();
      ++trunk_publish_interval_count_;
      trunk_publish_interval_sum_ms_ += publish_interval_ms;
      trunk_publish_interval_max_ms_ = std::max(trunk_publish_interval_max_ms_, publish_interval_ms);
      if (publish_interval_ms > 100.0) {
        ++trunk_publish_gap_over_100ms_count_;
      }
      if (publish_interval_ms > 150.0) {
        ++trunk_publish_gap_over_150ms_count_;
      }
      if (publish_interval_ms > 200.0) {
        ++trunk_publish_gap_over_200ms_count_;
      }
    }
    const auto trunk_start = total_start;
    publisher_->publish(*output);
    const auto trunk_end = std::chrono::steady_clock::now();
    ++lidar_points_publish_count_;
    last_cloud_points_ = static_cast<std::size_t>(output->width) * output->height;
    last_cloud_bytes_ = output->data.size();
    last_output_stamp_sec_ = stamp_to_sec(output->header.stamp);
    last_output_subscription_count_ = publisher_->get_subscription_count();
    last_trunk_publish_duration_ms_ =
      std::chrono::duration<double, std::milli>(trunk_end - trunk_start).count();

    const auto branch_start = std::chrono::steady_clock::now();
    if (local_publisher_) {
      const auto local_result =
        publish_downsample(local_publisher_, local_output_stride_, local_output_publish_every_n_, *output);
      update_branch_diagnostics(local_branch_, local_result, std::chrono::steady_clock::now());
    } else {
      local_branch_.subscription_count = 0U;
    }
    if (nav_publisher_) {
      const auto nav_result =
        publish_downsample(nav_publisher_, nav_output_stride_, nav_output_publish_every_n_, *output);
      update_branch_diagnostics(nav_branch_, nav_result, std::chrono::steady_clock::now());
    } else {
      nav_branch_.subscription_count = 0U;
    }
    const auto publish_end = std::chrono::steady_clock::now();
    last_branch_publish_duration_ms_ =
      std::chrono::duration<double, std::milli>(publish_end - branch_start).count();
    last_total_publish_outputs_duration_ms_ =
      std::chrono::duration<double, std::milli>(publish_end - total_start).count();
    last_publish_time_ = publish_end;
    last_publish_duration_ms_ = last_total_publish_outputs_duration_ms_;

    if (!logged_ready_) {
      RCLCPP_INFO(
        get_logger(),
        "compiled canonical pointcloud remap ready: %s -> %s frame=%s nav_output=%s stride=%zu every_n=%zu local_output=%s stride=%zu every_n=%zu",
        input_topic_.c_str(),
        output_topic_.c_str(),
        output_frame_id_.c_str(),
        nav_output_topic_.empty() ? "(disabled)" : nav_output_topic_.c_str(),
        nav_output_stride_,
        nav_output_publish_every_n_,
        local_output_topic_.empty() ? "(disabled)" : local_output_topic_.c_str(),
        local_output_stride_,
        local_output_publish_every_n_);
      logged_ready_ = true;
    }
  }

  void on_cloud(sensor_msgs::msg::PointCloud2::UniquePtr msg)
  {
    const auto callback_start = std::chrono::steady_clock::now();
    ++raw_input_count_;
    if (last_raw_callback_time_ != std::chrono::steady_clock::time_point{}) {
      const double raw_interarrival_ms =
        std::chrono::duration<double, std::milli>(callback_start - last_raw_callback_time_).count();
      ++raw_interarrival_count_;
      raw_interarrival_sum_ms_ += raw_interarrival_ms;
      raw_interarrival_max_ms_ = std::max(raw_interarrival_max_ms_, raw_interarrival_ms);
    }
    last_raw_callback_time_ = callback_start;
    last_raw_stamp_sec_ = stamp_to_sec(msg->header.stamp);
    auto output = std::move(msg);
    output->header.frame_id = output_frame_id_;

    if (output->data.empty() || output->width == 0U || output->height == 0U) {
      publish_outputs(std::move(output));
      last_raw_callback_duration_ms_ =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - callback_start).count();
      return;
    }

    if (!has_xyz_fields(*output)) {
      if (!warned_missing_xyz_) {
        RCLCPP_ERROR(
          get_logger(), "cloud on %s does not expose x/y/z fields", input_topic_.c_str());
        warned_missing_xyz_ = true;
      }
      ++dropped_or_skipped_count_;
      last_raw_callback_duration_ms_ =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - callback_start).count();
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
      last_raw_callback_duration_ms_ =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - callback_start).count();
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
      last_raw_callback_duration_ms_ =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - callback_start).count();
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
    last_raw_callback_duration_ms_ =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - callback_start).count();
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
    const auto nav_attempt_delta = nav_branch_.attempt_count - nav_branch_.previous_attempt_count;
    const auto nav_publish_delta = nav_branch_.publish_count - nav_branch_.previous_publish_count;
    const auto nav_skip_delta = nav_branch_.skip_count - nav_branch_.previous_skip_count;
    const auto local_attempt_delta = local_branch_.attempt_count - local_branch_.previous_attempt_count;
    const auto local_publish_delta = local_branch_.publish_count - local_branch_.previous_publish_count;
    const auto local_skip_delta = local_branch_.skip_count - local_branch_.previous_skip_count;
    const double raw_interarrival_ms_avg = raw_interarrival_count_ > 0U ?
      raw_interarrival_sum_ms_ / static_cast<double>(raw_interarrival_count_) : -1.0;
    const double trunk_publish_interval_ms_avg = trunk_publish_interval_count_ > 0U ?
      trunk_publish_interval_sum_ms_ / static_cast<double>(trunk_publish_interval_count_) : -1.0;
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
    const double nav_publish_age_ms = nav_branch_.last_publish_time == std::chrono::steady_clock::time_point{} ?
      -1.0 :
      std::chrono::duration<double, std::milli>(now_steady - nav_branch_.last_publish_time).count();
    const double local_publish_age_ms =
      local_branch_.last_publish_time == std::chrono::steady_clock::time_point{} ?
      -1.0 :
      std::chrono::duration<double, std::milli>(now_steady - local_branch_.last_publish_time).count();
    const double uptime_sec = std::chrono::duration<double>(now_steady - start_time_).count();

    std_msgs::msg::String msg;
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(3)
           << "input_topic=" << input_topic_
           << " output_topic=" << output_topic_
           << " raw_input_hz=" << static_cast<double>(raw_delta) / elapsed_sec
           << " lidar_points_publish_hz=" << static_cast<double>(publish_delta) / elapsed_sec
           << " fast_path_raw_input_hz=" << static_cast<double>(raw_delta) / elapsed_sec
           << " fast_path_lidar_points_publish_hz=" << static_cast<double>(publish_delta) / elapsed_sec
           << " fast_path_duration_ms=" << last_raw_callback_duration_ms_
           << " latest_buffer_update_hz=" << static_cast<double>(publish_delta) / elapsed_sec
           << " latest_buffer_points=" << last_cloud_points_
           << " latest_buffer_bytes=" << last_cloud_bytes_
           << " last_raw_age_ms=" << raw_age_ms
           << " last_raw_stamp_age_ms=" << raw_stamp_age_ms
           << " last_publish_age_ms=" << publish_age_ms
           << " last_publish_stamp_age_ms=" << publish_stamp_age_ms
           << " last_publish_duration_ms=" << last_publish_duration_ms_
           << " last_trunk_publish_duration_ms=" << last_trunk_publish_duration_ms_
           << " last_branch_publish_duration_ms=" << last_branch_publish_duration_ms_
           << " last_total_publish_outputs_duration_ms=" << last_total_publish_outputs_duration_ms_
           << " raw_interarrival_ms_avg=" << raw_interarrival_ms_avg
           << " raw_interarrival_ms_max=" << raw_interarrival_max_ms_
           << " lidar_points_publish_interval_ms_avg=" << trunk_publish_interval_ms_avg
           << " lidar_points_publish_interval_ms_max=" << trunk_publish_interval_max_ms_
           << " trunk_publish_gap_over_100ms_count=" << trunk_publish_gap_over_100ms_count_
           << " trunk_publish_gap_over_150ms_count=" << trunk_publish_gap_over_150ms_count_
           << " trunk_publish_gap_over_200ms_count=" << trunk_publish_gap_over_200ms_count_
           << " last_raw_callback_duration_ms=" << last_raw_callback_duration_ms_
           << " last_publish_outputs_start_to_end_ms=" << last_total_publish_outputs_duration_ms_
           << " raw_callback_count=" << raw_input_count_
           << " trunk_publish_count=" << lidar_points_publish_count_
           << " last_cloud_points=" << last_cloud_points_
           << " last_cloud_bytes=" << last_cloud_bytes_
           << " output_subscription_count=" << last_output_subscription_count_
           << " trunk_output_subscription_count=" << last_output_subscription_count_
           << " accel_profile=legacy"
           << " worker_local_enabled=false"
           << " worker_scan_enabled=false"
           << " nav_branch_enabled=" << (nav_publisher_ ? "true" : "false")
           << " local_branch_enabled=" << (local_publisher_ ? "true" : "false")
           << " nav_branch_attempt_count=" << nav_branch_.attempt_count
           << " nav_branch_publish_hz=" << static_cast<double>(nav_publish_delta) / elapsed_sec
           << " nav_branch_skip_hz=" << static_cast<double>(nav_skip_delta) / elapsed_sec
           << " nav_branch_attempt_hz=" << static_cast<double>(nav_attempt_delta) / elapsed_sec
           << " nav_branch_last_publish_age_ms=" << nav_publish_age_ms
           << " nav_branch_last_publish_duration_ms=" << nav_branch_.last_publish_duration_ms
           << " nav_branch_last_points=" << nav_branch_.last_points
           << " nav_branch_last_bytes=" << nav_branch_.last_bytes
           << " nav_branch_subscription_count=" << nav_branch_.subscription_count
           << " nav_output_stride=" << nav_output_stride_
           << " nav_output_publish_every_n=" << nav_output_publish_every_n_
           << " nav_branch_publish_count=" << nav_branch_.publish_count
           << " local_branch_attempt_count=" << local_branch_.attempt_count
           << " local_branch_publish_hz=" << static_cast<double>(local_publish_delta) / elapsed_sec
           << " local_branch_skip_hz=" << static_cast<double>(local_skip_delta) / elapsed_sec
           << " local_branch_attempt_hz=" << static_cast<double>(local_attempt_delta) / elapsed_sec
           << " local_branch_last_publish_age_ms=" << local_publish_age_ms
           << " local_branch_last_publish_duration_ms=" << local_branch_.last_publish_duration_ms
           << " local_branch_last_points=" << local_branch_.last_points
           << " local_branch_last_bytes=" << local_branch_.last_bytes
           << " local_branch_subscription_count=" << local_branch_.subscription_count
           << " local_output_stride=" << local_output_stride_
           << " local_output_publish_every_n=" << local_output_publish_every_n_
           << " local_branch_publish_count=" << local_branch_.publish_count
           << " local_compact_last_points=" << local_branch_.last_points
           << " local_compact_last_bytes=" << local_branch_.last_bytes
           << " local_compact_bytes_per_point=" <<
      (local_branch_.last_points > 0U ? local_branch_.last_bytes / local_branch_.last_points : 0U)
           << " local_compact_publish_hz=" << static_cast<double>(local_publish_delta) / elapsed_sec
           << " local_compact_skip_busy_count=0"
           << " nav_compact_last_points=" << nav_branch_.last_points
           << " nav_compact_last_bytes=" << nav_branch_.last_bytes
           << " nav_compact_bytes_per_point=" <<
      (nav_branch_.last_points > 0U ? nav_branch_.last_bytes / nav_branch_.last_points : 0U)
           << " nav_compact_publish_hz=" << static_cast<double>(nav_publish_delta) / elapsed_sec
           << " nav_compact_skip_busy_count=0"
           << " dropped_or_skipped_count=" << dropped_or_skipped_count_
           << " node_uptime_sec=" << uptime_sec;
    msg.data = stream.str();
    status_publisher_->publish(msg);

    previous_status_time_ = now_steady;
    previous_raw_input_count_ = raw_input_count_;
    previous_lidar_points_publish_count_ = lidar_points_publish_count_;
    nav_branch_.previous_attempt_count = nav_branch_.attempt_count;
    nav_branch_.previous_publish_count = nav_branch_.publish_count;
    nav_branch_.previous_skip_count = nav_branch_.skip_count;
    local_branch_.previous_attempt_count = local_branch_.attempt_count;
    local_branch_.previous_publish_count = local_branch_.publish_count;
    local_branch_.previous_skip_count = local_branch_.skip_count;
    raw_interarrival_count_ = 0U;
    raw_interarrival_sum_ms_ = 0.0;
    raw_interarrival_max_ms_ = 0.0;
    trunk_publish_interval_count_ = 0U;
    trunk_publish_interval_sum_ms_ = 0.0;
    trunk_publish_interval_max_ms_ = 0.0;
    trunk_publish_gap_over_100ms_count_ = 0U;
    trunk_publish_gap_over_150ms_count_ = 0U;
    trunk_publish_gap_over_200ms_count_ = 0U;
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string output_frame_id_;
  std::string nav_output_topic_;
  std::size_t nav_output_stride_;
  std::size_t nav_output_publish_every_n_;
  std::size_t nav_output_qos_depth_;
  std::string local_output_topic_;
  std::size_t local_output_stride_;
  std::size_t local_output_publish_every_n_;
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
  std::uint64_t raw_interarrival_count_{0};
  std::uint64_t trunk_publish_interval_count_{0};
  std::uint64_t trunk_publish_gap_over_100ms_count_{0};
  std::uint64_t trunk_publish_gap_over_150ms_count_{0};
  std::uint64_t trunk_publish_gap_over_200ms_count_{0};
  std::size_t last_cloud_points_{0};
  std::size_t last_cloud_bytes_{0};
  std::size_t last_output_subscription_count_{0};
  double last_raw_stamp_sec_{0.0};
  double last_output_stamp_sec_{0.0};
  double raw_interarrival_sum_ms_{0.0};
  double raw_interarrival_max_ms_{0.0};
  double trunk_publish_interval_sum_ms_{0.0};
  double trunk_publish_interval_max_ms_{0.0};
  double last_raw_callback_duration_ms_{0.0};
  double last_publish_duration_ms_{0.0};
  double last_trunk_publish_duration_ms_{0.0};
  double last_branch_publish_duration_ms_{0.0};
  double last_total_publish_outputs_duration_ms_{0.0};
  BranchDiagnostics nav_branch_;
  BranchDiagnostics local_branch_;
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
