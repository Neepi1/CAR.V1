#include <algorithm>
#include <atomic>
#include <array>
#include <chrono>
#include <condition_variable>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <pcl/common/transforms.h>
#include <pcl/filters/filter.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"
#include "std_msgs/msg/header.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/exceptions.h"
#include "tf2/time.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace
{
using PointT = pcl::PointXYZI;

double stamp_to_sec(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1.0e-9;
}

struct CropBox
{
  bool enabled{false};
  double min_x{0.0};
  double max_x{0.0};
  double min_y{0.0};
  double max_y{0.0};
  double min_z{0.0};
  double max_z{0.0};

  bool contains(const PointT & point) const
  {
    if (!enabled) {
      return false;
    }
    return point.x >= min_x && point.x <= max_x &&
           point.y >= min_y && point.y <= max_y &&
           point.z >= min_z && point.z <= max_z;
  }
};

struct AzimuthFilter
{
  bool enabled{false};
  double min_angle_rad{-M_PI};
  double max_angle_rad{M_PI};

  bool contains(const double angle_rad) const
  {
    if (!enabled) {
      return true;
    }
    if (min_angle_rad <= max_angle_rad) {
      return angle_rad >= min_angle_rad && angle_rad <= max_angle_rad;
    }
    return angle_rad >= min_angle_rad || angle_rad <= max_angle_rad;
  }
};

struct OutlierFilter
{
  bool enabled{false};
  double voxel_size{0.15};
  int min_points_per_voxel{2};
};

struct ModeProfile
{
  double range_min{0.5};
  double range_max{40.0};
  double min_z{-0.2};
  double max_z{1.6};
  AzimuthFilter azimuth_filter;
  CropBox self_mask;
  CropBox front_mask;
  OutlierFilter outlier_filter;
};

struct ClearingRayBin
{
  bool has_return{false};
  double range_xy{0.0};
  double angle_rad{0.0};
};

struct PendingClearingJob
{
  std_msgs::msg::Header header;
  ModeProfile profile;
  std::vector<ClearingRayBin> bins;
  std::vector<PointT> points;
  bool virtual_rays_enabled{false};
  double ray_origin_x{0.0};
  double ray_origin_y{0.0};
};

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

float read_float32(const std::uint8_t * data)
{
  float value = 0.0F;
  std::memcpy(&value, data, sizeof(float));
  return value;
}

void write_float32(std::uint8_t * data, const float value)
{
  std::memcpy(data, &value, sizeof(float));
}

sensor_msgs::msg::PointField makeFloat32Field(const std::string & name, const std::uint32_t offset)
{
  sensor_msgs::msg::PointField field;
  field.name = name;
  field.offset = offset;
  field.datatype = sensor_msgs::msg::PointField::FLOAT32;
  field.count = 1;
  return field;
}

sensor_msgs::msg::PointCloud2 makePointCloud2FromPoints(
  const std::vector<PointT> & points,
  const std_msgs::msg::Header & header)
{
  sensor_msgs::msg::PointCloud2 msg;
  msg.header = header;
  msg.height = 1;
  msg.width = static_cast<std::uint32_t>(points.size());
  msg.fields = {
    makeFloat32Field("x", 0U),
    makeFloat32Field("y", 4U),
    makeFloat32Field("z", 8U),
    makeFloat32Field("intensity", 16U)};
  msg.is_bigendian = false;
  msg.point_step = 32U;
  msg.row_step = msg.point_step * msg.width;
  msg.is_dense = false;
  msg.data.resize(static_cast<std::size_t>(msg.row_step), 0U);

  for (std::size_t index = 0; index < points.size(); ++index) {
    auto * point_data = msg.data.data() + index * msg.point_step;
    write_float32(point_data + 0U, points[index].x);
    write_float32(point_data + 4U, points[index].y);
    write_float32(point_data + 8U, points[index].z);
    write_float32(point_data + 16U, points[index].intensity);
  }
  return msg;
}

std::uint64_t packVoxelIndex(const int value)
{
  constexpr int kBits = 21;
  constexpr int kBias = 1 << (kBits - 1);
  constexpr std::uint64_t kMask = (1ULL << kBits) - 1ULL;
  const auto clamped = std::clamp(value, -kBias, kBias - 1);
  return static_cast<std::uint64_t>(clamped + kBias) & kMask;
}

std::uint64_t packedVoxelKey(const PointT & point, const double voxel_size)
{
  constexpr int kBits = 21;
  const auto ix = static_cast<int>(std::floor(static_cast<double>(point.x) / voxel_size));
  const auto iy = static_cast<int>(std::floor(static_cast<double>(point.y) / voxel_size));
  const auto iz = static_cast<int>(std::floor(static_cast<double>(point.z) / voxel_size));
  return packVoxelIndex(ix) |
         (packVoxelIndex(iy) << kBits) |
         (packVoxelIndex(iz) << (2 * kBits));
}

struct FusedTransform3x4
{
  float m00{1.0F};
  float m01{0.0F};
  float m02{0.0F};
  float m03{0.0F};
  float m10{0.0F};
  float m11{1.0F};
  float m12{0.0F};
  float m13{0.0F};
  float m20{0.0F};
  float m21{0.0F};
  float m22{1.0F};
  float m23{0.0F};
};
}  // namespace

class LocalPerceptionNode : public rclcpp::Node
{
public:
  explicit LocalPerceptionNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("robot_local_perception", options),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    declare_parameter<bool>("mock_mode", true);
    mode_topic_ = declare_parameter<std::string>("mode_topic", "/robot_mode");
    input_topic_ = declare_parameter<std::string>("input_topic", "/lidar_points");
    input_reliable_ = declare_parameter<bool>("input_reliable", false);
    input_qos_depth_ = static_cast<int>(std::max<std::int64_t>(
      declare_parameter<std::int64_t>("input_qos_depth", 1), 1));
    output_topic_ = declare_parameter<std::string>("output_topic", "/perception/obstacle_points");
    clearing_output_topic_ = declare_parameter<std::string>("clearing_output_topic", "/perception/clearing_points");
    output_frame_id_ = declare_parameter<std::string>("output_frame_id", "base_link");
    input_frame_id_override_ = declare_parameter<std::string>("input_frame_id_override", "");
    input_transform_use_latest_ = declare_parameter<bool>("input_transform_use_latest", true);
    loadInputRotation(
      declare_parameter<std::vector<double>>(
        "input_rotation_matrix",
        std::vector<double>{
          1.0, 0.0, 0.0,
          0.0, 1.0, 0.0,
          0.0, 0.0, 1.0}));
    output_stamp_tf_target_frame_ = declare_parameter<std::string>("output_stamp_tf_target_frame", "odom");
    output_stamp_odom_topic_ = declare_parameter<std::string>("output_stamp_odom_topic", "/local_state/odometry");
    current_mode_ = declare_parameter<std::string>("mode", "NORMAL");
    restamp_to_now_ = declare_parameter<bool>("restamp_to_now", true);
    restamp_to_latest_tf_ = declare_parameter<bool>("restamp_to_latest_tf", false);
    require_output_stamp_tf_ = declare_parameter<bool>("require_output_stamp_tf", false);
    lookup_timeout_sec_ = declare_parameter<double>("lookup_timeout_sec", 0.1);
    max_output_tf_stamp_age_sec_ = declare_parameter<double>("max_output_tf_stamp_age_sec", 0.45);
    output_stamp_tf_backoff_sec_ = declare_parameter<double>("output_stamp_tf_backoff_sec", 0.0);
    output_stamp_forward_sec_ = declare_parameter<double>("output_stamp_forward_sec", 0.0);
    require_startup_tf_ready_ = declare_parameter<bool>("require_startup_tf_ready", true);
    startup_tf_warmup_sec_ = std::max(declare_parameter<double>("startup_tf_warmup_sec", 1.0), 0.0);
    if (output_stamp_tf_backoff_sec_ < 0.0) {
      RCLCPP_WARN(
        get_logger(), "output_stamp_tf_backoff_sec %.3f is negative; clamping to 0.0",
        output_stamp_tf_backoff_sec_);
      output_stamp_tf_backoff_sec_ = 0.0;
    }
    if (output_stamp_forward_sec_ < 0.0) {
      RCLCPP_WARN(
        get_logger(), "output_stamp_forward_sec %.3f is negative; clamping to 0.0",
        output_stamp_forward_sec_);
      output_stamp_forward_sec_ = 0.0;
    } else if (output_stamp_forward_sec_ > 0.25) {
      RCLCPP_WARN(
        get_logger(),
        "output_stamp_forward_sec %.3f is too large for Nav2 costmap startup; clamping to 0.25",
        output_stamp_forward_sec_);
      output_stamp_forward_sec_ = 0.25;
    }
    if (max_output_tf_stamp_age_sec_ < 0.0) {
      RCLCPP_WARN(
        get_logger(), "max_output_tf_stamp_age_sec %.3f is negative; clamping to 0.0",
        max_output_tf_stamp_age_sec_);
      max_output_tf_stamp_age_sec_ = 0.0;
    }
    processing_rate_hz_ = std::max(declare_parameter<double>("processing_rate_hz", 8.0), 0.1);
    process_on_callback_ = declare_parameter<bool>("process_on_callback", true);
    point_sample_stride_ = static_cast<int>(std::max<std::int64_t>(
      declare_parameter<std::int64_t>("point_sample_stride", 4), 1));
    max_filtered_points_ = static_cast<int>(std::max<std::int64_t>(
      declare_parameter<std::int64_t>("max_filtered_points", 12000), 1));
    clearing_enabled_ = declare_parameter<bool>("clearing.enabled", true);
    clearing_publish_every_n_ = static_cast<int>(std::max<std::int64_t>(
      declare_parameter<std::int64_t>("clearing.publish_every_n", 1), 1));
    clearing_range_min_ = declare_parameter<double>("clearing.range_filter.min", 0.10);
    clearing_range_max_ = declare_parameter<double>("clearing.range_filter.max", 5.00);
    clearing_min_z_ = declare_parameter<double>("clearing.height_filter.min_z", -0.30);
    clearing_max_z_ = declare_parameter<double>("clearing.height_filter.max_z", 1.40);
    clearing_point_sample_stride_ = static_cast<int>(std::max<std::int64_t>(
      declare_parameter<std::int64_t>("clearing.point_sample_stride", 4), 1));
    clearing_max_points_ = static_cast<int>(std::max<std::int64_t>(
      declare_parameter<std::int64_t>("clearing.max_points", 72000), 1));
    clearing_virtual_rays_enabled_ = declare_parameter<bool>("clearing.virtual_rays.enabled", true);
    const auto virtual_angle_resolution_deg = std::clamp(
      declare_parameter<double>("clearing.virtual_rays.angular_resolution_deg", 0.5), 0.2, 10.0);
    clearing_virtual_ray_angle_resolution_rad_ = virtual_angle_resolution_deg * M_PI / 180.0;
    clearing_virtual_ray_range_ = std::max(
      clearing_range_min_, declare_parameter<double>("clearing.virtual_rays.range", 6.0));
    clearing_virtual_ray_ranges_ = declare_parameter<std::vector<double>>(
      "clearing.virtual_rays.range_steps",
      std::vector<double>{0.35, 0.50, 0.75, 1.25, 2.0, 3.5, 6.0});
    sanitizeVirtualRayRanges();
    clearing_virtual_ray_endpoint_z_values_ = declare_parameter<std::vector<double>>(
      "clearing.virtual_rays.endpoint_z_values",
      std::vector<double>{
        -0.15, -0.05, 0.00, 0.05, 0.10, 0.15, 0.20, 0.25, 0.30, 0.35, 0.45, 0.55,
        0.65, 0.75, 0.85, 0.95, 1.05, 1.15, 1.25, 1.35});
    clearing_virtual_ray_endpoint_z_values_.erase(
      std::remove_if(
        clearing_virtual_ray_endpoint_z_values_.begin(),
        clearing_virtual_ray_endpoint_z_values_.end(),
        [this](const double z) { return z < clearing_min_z_ || z > clearing_max_z_; }),
      clearing_virtual_ray_endpoint_z_values_.end());
    if (clearing_virtual_ray_endpoint_z_values_.empty()) {
      clearing_virtual_ray_endpoint_z_values_.push_back(0.5 * (clearing_min_z_ + clearing_max_z_));
    }
    publish_debug_log_ = declare_parameter<bool>("publish_debug_log", false);
    declare_parameter<std::string>("local_nav_preprocessor_reference", "");
    supported_modes_ = declare_parameter<std::vector<std::string>>(
      "supported_modes", std::vector<std::string>{"NORMAL", "RAMP", "ELEVATOR_WAIT", "DOORWAY"});
    if (std::find(supported_modes_.begin(), supported_modes_.end(), current_mode_) == supported_modes_.end()) {
      current_mode_ = supported_modes_.front();
    }
    profiles_ = loadProfiles(supported_modes_);

    auto output_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
    auto input_qos = rclcpp::QoS(rclcpp::KeepLast(input_qos_depth_)).durability_volatile();
    if (input_reliable_) {
      input_qos.reliable();
    } else {
      input_qos.best_effort();
    }
    publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, output_qos);
    clearing_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(clearing_output_topic_, output_qos);
    mode_publisher_ = create_publisher<std_msgs::msg::String>("/perception/mode", 10);
    subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_, input_qos, std::bind(&LocalPerceptionNode::cloudCallback, this, std::placeholders::_1));
    if (!mode_topic_.empty()) {
      mode_subscription_ = create_subscription<std_msgs::msg::String>(
        mode_topic_, 10, std::bind(&LocalPerceptionNode::modeCallback, this, std::placeholders::_1));
    }
    if (!output_stamp_odom_topic_.empty()) {
      odom_stamp_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
        output_stamp_odom_topic_,
        rclcpp::QoS(rclcpp::KeepLast(20)).reliable().durability_volatile(),
        std::bind(&LocalPerceptionNode::odomStampCallback, this, std::placeholders::_1));
    }
    clearing_worker_ = std::thread(&LocalPerceptionNode::clearingWorkerLoop, this);
    if (!process_on_callback_) {
      timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / processing_rate_hz_),
        std::bind(&LocalPerceptionNode::processLatestCloud, this));
    }
  }

  ~LocalPerceptionNode() override
  {
    {
      std::lock_guard<std::mutex> lock(clearing_mutex_);
      clearing_worker_stop_ = true;
    }
    clearing_cv_.notify_one();
    if (clearing_worker_.joinable()) {
      clearing_worker_.join();
    }
  }

private:
  static ModeProfile defaultProfile(const std::string & mode_name)
  {
    ModeProfile profile;
    if (mode_name == "RAMP") {
      profile.range_min = 0.5;
      profile.range_max = 35.0;
      profile.min_z = -0.35;
      profile.max_z = 1.80;
      profile.azimuth_filter = {true, -120.0 * M_PI / 180.0, 120.0 * M_PI / 180.0};
      profile.self_mask = {true, -0.50, 0.45, -0.40, 0.40, -0.20, 1.40};
      profile.front_mask = {false, 0.20, 0.55, -0.20, 0.20, -0.10, 1.60};
      profile.outlier_filter = {false, 0.15, 2};
      return profile;
    }
    if (mode_name == "ELEVATOR_WAIT") {
      profile.range_min = 0.3;
      profile.range_max = 8.0;
      profile.min_z = 0.02;
      profile.max_z = 1.40;
      profile.azimuth_filter = {true, -95.0 * M_PI / 180.0, 95.0 * M_PI / 180.0};
      profile.self_mask = {true, -0.50, 0.45, -0.40, 0.40, -0.20, 1.40};
      profile.front_mask = {false, 0.20, 0.55, -0.20, 0.20, -0.10, 1.40};
      profile.outlier_filter = {true, 0.12, 2};
      return profile;
    }
    if (mode_name == "DOORWAY") {
      profile.range_min = 0.3;
      profile.range_max = 12.0;
      profile.min_z = -0.05;
      profile.max_z = 1.45;
      profile.azimuth_filter = {true, -90.0 * M_PI / 180.0, 90.0 * M_PI / 180.0};
      profile.self_mask = {true, -0.50, 0.45, -0.40, 0.40, -0.20, 1.40};
      profile.front_mask = {false, 0.20, 0.55, -0.20, 0.20, -0.10, 1.60};
      profile.outlier_filter = {true, 0.10, 2};
      return profile;
    }

    profile.range_min = 0.5;
    profile.range_max = 40.0;
    profile.min_z = 0.40;
    profile.max_z = 1.20;
    profile.azimuth_filter = {true, -110.0 * M_PI / 180.0, 110.0 * M_PI / 180.0};
    profile.self_mask = {true, -0.50, 0.45, -0.40, 0.40, -0.20, 1.40};
    profile.front_mask = {false, 0.20, 0.55, -0.20, 0.20, -0.10, 1.60};
    profile.outlier_filter = {false, 0.15, 2};
    return profile;
  }

  std::map<std::string, ModeProfile> loadProfiles(const std::vector<std::string> & modes)
  {
    std::map<std::string, ModeProfile> profiles;
    for (const auto & mode_name : modes) {
      const auto defaults = defaultProfile(mode_name);
      const auto prefix = "profiles." + mode_name + ".";
      ModeProfile profile;
      profile.range_min = declare_parameter<double>(prefix + "range_filter.min", defaults.range_min);
      profile.range_max = declare_parameter<double>(prefix + "range_filter.max", defaults.range_max);
      profile.min_z = declare_parameter<double>(prefix + "height_filter.min_z", defaults.min_z);
      profile.max_z = declare_parameter<double>(prefix + "height_filter.max_z", defaults.max_z);
      const auto min_angle_deg = declare_parameter<double>(
        prefix + "azimuth_filter.min_angle_deg", defaults.azimuth_filter.min_angle_rad * 180.0 / M_PI);
      const auto max_angle_deg = declare_parameter<double>(
        prefix + "azimuth_filter.max_angle_deg", defaults.azimuth_filter.max_angle_rad * 180.0 / M_PI);
      profile.azimuth_filter.enabled = declare_parameter<bool>(
        prefix + "azimuth_filter.enabled", defaults.azimuth_filter.enabled);
      profile.azimuth_filter.min_angle_rad = min_angle_deg * M_PI / 180.0;
      profile.azimuth_filter.max_angle_rad = max_angle_deg * M_PI / 180.0;
      profile.self_mask.enabled = declare_parameter<bool>(
        prefix + "self_mask.enabled", defaults.self_mask.enabled);
      profile.self_mask.min_x = declare_parameter<double>(prefix + "self_mask.min_x", defaults.self_mask.min_x);
      profile.self_mask.max_x = declare_parameter<double>(prefix + "self_mask.max_x", defaults.self_mask.max_x);
      profile.self_mask.min_y = declare_parameter<double>(prefix + "self_mask.min_y", defaults.self_mask.min_y);
      profile.self_mask.max_y = declare_parameter<double>(prefix + "self_mask.max_y", defaults.self_mask.max_y);
      profile.self_mask.min_z = declare_parameter<double>(prefix + "self_mask.min_z", defaults.self_mask.min_z);
      profile.self_mask.max_z = declare_parameter<double>(prefix + "self_mask.max_z", defaults.self_mask.max_z);
      profile.front_mask.enabled = declare_parameter<bool>(
        prefix + "front_mask.enabled", defaults.front_mask.enabled);
      profile.front_mask.min_x = declare_parameter<double>(prefix + "front_mask.min_x", defaults.front_mask.min_x);
      profile.front_mask.max_x = declare_parameter<double>(prefix + "front_mask.max_x", defaults.front_mask.max_x);
      profile.front_mask.min_y = declare_parameter<double>(prefix + "front_mask.min_y", defaults.front_mask.min_y);
      profile.front_mask.max_y = declare_parameter<double>(prefix + "front_mask.max_y", defaults.front_mask.max_y);
      profile.front_mask.min_z = declare_parameter<double>(prefix + "front_mask.min_z", defaults.front_mask.min_z);
      profile.front_mask.max_z = declare_parameter<double>(prefix + "front_mask.max_z", defaults.front_mask.max_z);
      profile.outlier_filter.enabled = declare_parameter<bool>(
        prefix + "outlier_filter.enabled", defaults.outlier_filter.enabled);
      profile.outlier_filter.voxel_size = declare_parameter<double>(
        prefix + "outlier_filter.voxel_size", defaults.outlier_filter.voxel_size);
      profile.outlier_filter.min_points_per_voxel = declare_parameter<int>(
        prefix + "outlier_filter.min_points_per_voxel", defaults.outlier_filter.min_points_per_voxel);
      profiles.emplace(mode_name, profile);
    }
    return profiles;
  }

  const ModeProfile & activeProfile() const
  {
    return profiles_.at(current_mode_);
  }

  void modeCallback(const std_msgs::msg::String::SharedPtr msg)
  {
    const auto requested_mode = msg->data;
    if (profiles_.count(requested_mode) == 0U) {
      RCLCPP_WARN(get_logger(), "ignoring unsupported perception mode: %s", requested_mode.c_str());
      return;
    }
    current_mode_ = requested_mode;
  }

  void odomStampCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    latest_odom_stamp_ = msg->header.stamp;
  }

  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    ++received_cloud_count_;
    latest_cloud_ = msg;
    ++latest_cloud_seq_;
    if (process_on_callback_) {
      processLatestCloud();
    }
    logDebugStats();
  }

  bool startupTfGateReady(const std::string & source_frame)
  {
    if (!require_startup_tf_ready_ || startup_tf_ready_) {
      return true;
    }

    const auto elapsed = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - startup_time_).count();
    if (elapsed < startup_tf_warmup_sec_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Skipping local perception cloud while TF listener warms: %.3fs < %.3fs.",
        elapsed, startup_tf_warmup_sec_);
      return false;
    }

    const auto timeout = tf2::durationFromSec(std::min(lookup_timeout_sec_, 0.05));
    try {
      if (!output_frame_id_.empty() && !source_frame.empty() && output_frame_id_ != source_frame) {
        (void)tf_buffer_.lookupTransform(output_frame_id_, source_frame, tf2::TimePointZero, timeout);
      }
      if (
        !output_stamp_tf_target_frame_.empty() &&
        !output_frame_id_.empty() &&
        output_stamp_tf_target_frame_ != output_frame_id_)
      {
        (void)tf_buffer_.lookupTransform(
          output_stamp_tf_target_frame_, output_frame_id_, tf2::TimePointZero, timeout);
      }
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Skipping local perception cloud until startup TF is ready: %s.", ex.what());
      return false;
    }

    startup_tf_ready_ = true;
    RCLCPP_INFO(
      get_logger(), "local perception startup TF gate passed for %s -> %s and %s -> %s",
      output_frame_id_.c_str(), source_frame.c_str(),
      output_stamp_tf_target_frame_.c_str(), output_frame_id_.c_str());
    return true;
  }

  Eigen::Matrix4f lookupTransformMatrix(
    const std::string & target_frame,
    const std::string & source_frame,
    const builtin_interfaces::msg::Time & stamp) const
  {
    geometry_msgs::msg::TransformStamped transform_msg;
    const auto timeout = tf2::durationFromSec(lookup_timeout_sec_);
    try {
      if (stamp.sec == 0 && stamp.nanosec == 0) {
        transform_msg = tf_buffer_.lookupTransform(target_frame, source_frame, tf2::TimePointZero, timeout);
      } else {
        transform_msg = tf_buffer_.lookupTransform(target_frame, source_frame, stamp, timeout);
      }
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(
        get_logger(),
        "Failed to transform cloud from %s to %s at cloud stamp: %s. Falling back to latest TF.",
        source_frame.c_str(), target_frame.c_str(), ex.what());
      transform_msg = tf_buffer_.lookupTransform(target_frame, source_frame, tf2::TimePointZero, timeout);
    }

    const auto & translation = transform_msg.transform.translation;
    const auto & rotation = transform_msg.transform.rotation;
    Eigen::Quaternionf quaternion(
      static_cast<float>(rotation.w),
      static_cast<float>(rotation.x),
      static_cast<float>(rotation.y),
      static_cast<float>(rotation.z));

    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    transform.block<3, 3>(0, 0) = quaternion.normalized().toRotationMatrix();
    transform(0, 3) = static_cast<float>(translation.x);
    transform(1, 3) = static_cast<float>(translation.y);
    transform(2, 3) = static_cast<float>(translation.z);
    return transform;
  }

  void loadInputRotation(const std::vector<double> & values)
  {
    if (values.size() != input_rotation_.size()) {
      RCLCPP_WARN(
        get_logger(), "input_rotation_matrix must contain 9 values; using identity rotation");
      input_rotation_ = {
        1.0F, 0.0F, 0.0F,
        0.0F, 1.0F, 0.0F,
        0.0F, 0.0F, 1.0F};
      input_rotation_is_identity_ = true;
      return;
    }
    for (std::size_t i = 0; i < input_rotation_.size(); ++i) {
      input_rotation_[i] = static_cast<float>(values[i]);
    }
    input_rotation_is_identity_ =
      std::abs(input_rotation_[0] - 1.0F) < 1.0e-6F &&
      std::abs(input_rotation_[1]) < 1.0e-6F &&
      std::abs(input_rotation_[2]) < 1.0e-6F &&
      std::abs(input_rotation_[3]) < 1.0e-6F &&
      std::abs(input_rotation_[4] - 1.0F) < 1.0e-6F &&
      std::abs(input_rotation_[5]) < 1.0e-6F &&
      std::abs(input_rotation_[6]) < 1.0e-6F &&
      std::abs(input_rotation_[7]) < 1.0e-6F &&
      std::abs(input_rotation_[8] - 1.0F) < 1.0e-6F;
  }

  FusedTransform3x4 makeFusedTransform(const Eigen::Matrix4f & input_to_output) const
  {
    Eigen::Matrix4f fused = input_to_output;
    if (!input_rotation_is_identity_) {
      Eigen::Matrix4f input_rotation = Eigen::Matrix4f::Identity();
      input_rotation(0, 0) = input_rotation_[0];
      input_rotation(0, 1) = input_rotation_[1];
      input_rotation(0, 2) = input_rotation_[2];
      input_rotation(1, 0) = input_rotation_[3];
      input_rotation(1, 1) = input_rotation_[4];
      input_rotation(1, 2) = input_rotation_[5];
      input_rotation(2, 0) = input_rotation_[6];
      input_rotation(2, 1) = input_rotation_[7];
      input_rotation(2, 2) = input_rotation_[8];
      fused = input_to_output * input_rotation;
    }

    return FusedTransform3x4{
      fused(0, 0), fused(0, 1), fused(0, 2), fused(0, 3),
      fused(1, 0), fused(1, 1), fused(1, 2), fused(1, 3),
      fused(2, 0), fused(2, 1), fused(2, 2), fused(2, 3)};
  }

  PointT transformInputPoint(
    const FusedTransform3x4 & transform,
    const float x,
    const float y,
    const float z,
    const float intensity) const
  {
    PointT point;
    point.x = transform.m00 * x + transform.m01 * y + transform.m02 * z + transform.m03;
    point.y = transform.m10 * x + transform.m11 * y + transform.m12 * z + transform.m13;
    point.z = transform.m20 * x + transform.m21 * y + transform.m22 * z + transform.m23;
    point.intensity = intensity;
    return point;
  }

  bool passesFilters(const PointT & point, const ModeProfile & profile) const
  {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
      return false;
    }
    const auto range_xy = std::hypot(static_cast<double>(point.x), static_cast<double>(point.y));
    if (range_xy < profile.range_min || range_xy > profile.range_max) {
      return false;
    }
    if (point.z < profile.min_z || point.z > profile.max_z) {
      return false;
    }
    if (!profile.azimuth_filter.contains(std::atan2(static_cast<double>(point.y), static_cast<double>(point.x)))) {
      return false;
    }
    if (profile.self_mask.contains(point) || profile.front_mask.contains(point)) {
      return false;
    }
    return true;
  }

  bool passesClearingFilters(const PointT & point, const ModeProfile & profile) const
  {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
      return false;
    }
    const auto range_xy = std::hypot(static_cast<double>(point.x), static_cast<double>(point.y));
    if (range_xy < clearing_range_min_ || range_xy > clearing_range_max_) {
      return false;
    }
    if (point.z < clearing_min_z_ || point.z > clearing_max_z_) {
      return false;
    }
    if (!profile.azimuth_filter.contains(std::atan2(static_cast<double>(point.y), static_cast<double>(point.x)))) {
      return false;
    }
    if (profile.self_mask.contains(point) || profile.front_mask.contains(point)) {
      return false;
    }
    return true;
  }

  std::pair<double, double> clearingAzimuthRange(const ModeProfile & profile) const
  {
    if (!profile.azimuth_filter.enabled || profile.azimuth_filter.min_angle_rad > profile.azimuth_filter.max_angle_rad) {
      return {-M_PI, M_PI};
    }
    return {profile.azimuth_filter.min_angle_rad, profile.azimuth_filter.max_angle_rad};
  }

  std::vector<ClearingRayBin> makeClearingBins(const ModeProfile & profile) const
  {
    const auto [min_angle, max_angle] = clearingAzimuthRange(profile);
    const auto span = std::max(max_angle - min_angle, clearing_virtual_ray_angle_resolution_rad_);
    const auto bin_count = static_cast<std::size_t>(
      std::ceil(span / clearing_virtual_ray_angle_resolution_rad_)) + 1U;
    std::vector<ClearingRayBin> bins(bin_count);
    for (std::size_t index = 0; index < bins.size(); ++index) {
      bins[index].angle_rad = min_angle + (static_cast<double>(index) + 0.5) * clearing_virtual_ray_angle_resolution_rad_;
    }
    return bins;
  }

  void updateClearingBin(
    std::vector<ClearingRayBin> & bins,
    const ModeProfile & profile,
    const PointT & point,
    const double ray_origin_x,
    const double ray_origin_y) const
  {
    if (bins.empty()) {
      return;
    }
    const auto [min_angle, max_angle] = clearingAzimuthRange(profile);
    const auto dx = static_cast<double>(point.x) - ray_origin_x;
    const auto dy = static_cast<double>(point.y) - ray_origin_y;
    const auto angle = std::atan2(dy, dx);
    if (angle < min_angle || angle > max_angle) {
      return;
    }
    const auto range_xy = std::hypot(dx, dy);
    const auto raw_index = static_cast<long>(
      std::floor((angle - min_angle) / clearing_virtual_ray_angle_resolution_rad_));
    const auto clamped_index = std::clamp<long>(raw_index, 0, static_cast<long>(bins.size() - 1U));
    auto & bin = bins[static_cast<std::size_t>(clamped_index)];
    if (!bin.has_return || range_xy > bin.range_xy) {
      bin.has_return = true;
      bin.range_xy = range_xy;
      bin.angle_rad = angle;
    }
  }

  std::vector<PointT> buildVirtualClearingPoints(
    const std::vector<ClearingRayBin> & bins,
    const ModeProfile & profile,
    const double ray_origin_x,
    const double ray_origin_y) const
  {
    std::vector<PointT> clearing_points;
    clearing_points.reserve(
      bins.size() * clearing_virtual_ray_ranges_.size() * clearing_virtual_ray_endpoint_z_values_.size());
    for (const auto & bin : bins) {
      const auto angle = bin.angle_rad;
      if (!profile.azimuth_filter.contains(angle)) {
        continue;
      }
      const auto max_range_xy = std::clamp(
        bin.has_return ? bin.range_xy : clearing_virtual_ray_range_,
        clearing_range_min_,
        clearing_virtual_ray_range_);
      std::vector<double> ray_ranges;
      ray_ranges.reserve(clearing_virtual_ray_ranges_.size() + 1U);
      for (const auto configured_range : clearing_virtual_ray_ranges_) {
        if (configured_range < clearing_range_min_ || configured_range >= max_range_xy - 1e-3) {
          continue;
        }
        ray_ranges.push_back(configured_range);
      }
      ray_ranges.push_back(max_range_xy);
      for (const auto range_xy : ray_ranges) {
        const auto x = static_cast<float>(ray_origin_x + std::cos(angle) * range_xy);
        const auto y = static_cast<float>(ray_origin_y + std::sin(angle) * range_xy);
        for (const auto z_value : clearing_virtual_ray_endpoint_z_values_) {
          PointT endpoint;
          endpoint.x = x;
          endpoint.y = y;
          endpoint.z = static_cast<float>(z_value);
          endpoint.intensity = 0.0F;
          if (profile.self_mask.contains(endpoint) || profile.front_mask.contains(endpoint)) {
            continue;
          }
          clearing_points.push_back(endpoint);
        }
      }
    }
    return clearing_points;
  }

  void sanitizeVirtualRayRanges()
  {
    std::vector<double> sanitized;
    sanitized.reserve(clearing_virtual_ray_ranges_.size() + 1U);
    for (const auto range_xy : clearing_virtual_ray_ranges_) {
      if (!std::isfinite(range_xy)) {
        continue;
      }
      if (range_xy < clearing_range_min_ || range_xy > clearing_virtual_ray_range_) {
        continue;
      }
      sanitized.push_back(range_xy);
    }
    sanitized.push_back(clearing_virtual_ray_range_);
    std::sort(sanitized.begin(), sanitized.end());
    sanitized.erase(
      std::unique(
        sanitized.begin(),
        sanitized.end(),
        [](const double lhs, const double rhs) { return std::abs(lhs - rhs) < 1e-3; }),
      sanitized.end());
    clearing_virtual_ray_ranges_ = sanitized;
  }

  std::vector<PointT> applyVoxelOutlierFilter(
    std::vector<PointT> input_points,
    const ModeProfile & profile) const
  {
    if (!profile.outlier_filter.enabled || input_points.empty()) {
      return input_points;
    }

    const auto voxel_size = std::max(profile.outlier_filter.voxel_size, 1e-3);
    const auto min_points = std::max(profile.outlier_filter.min_points_per_voxel, 1);

    std::unordered_map<std::uint64_t, std::uint16_t> counts;
    counts.reserve(input_points.size() * 2U);
    std::vector<std::uint64_t> keys;
    keys.reserve(input_points.size());

    for (const auto & point : input_points) {
      const auto key = packedVoxelKey(point, voxel_size);
      auto & count = counts[key];
      if (count < std::numeric_limits<std::uint16_t>::max()) {
        ++count;
      }
      keys.push_back(key);
    }

    std::vector<PointT> filtered;
    filtered.reserve(input_points.size());
    for (std::size_t index = 0; index < input_points.size(); ++index) {
      if (counts[keys[index]] >= min_points) {
        filtered.push_back(input_points[index]);
      }
    }
    return filtered;
  }

  std::vector<PointT> limitPointCount(std::vector<PointT> points, const int max_points) const
  {
    if (max_points <= 0 || static_cast<int>(points.size()) <= max_points) {
      return points;
    }
    const auto reduction_stride = static_cast<std::size_t>(
      std::ceil(static_cast<double>(points.size()) / static_cast<double>(max_points)));
    std::vector<PointT> reduced;
    reduced.reserve(static_cast<std::size_t>(max_points));
    for (std::size_t index = 0; index < points.size(); index += reduction_stride) {
      reduced.push_back(points[index]);
    }
    return reduced;
  }

  void enqueueClearingJob(PendingClearingJob job)
  {
    {
      std::lock_guard<std::mutex> lock(clearing_mutex_);
      pending_clearing_job_ = std::move(job);
    }
    clearing_cv_.notify_one();
  }

  void clearingWorkerLoop()
  {
    while (true) {
      PendingClearingJob job;
      {
        std::unique_lock<std::mutex> lock(clearing_mutex_);
        clearing_cv_.wait(lock, [this]() {
          return clearing_worker_stop_ || pending_clearing_job_.has_value();
        });
        if (clearing_worker_stop_ && !pending_clearing_job_) {
          return;
        }
        job = std::move(*pending_clearing_job_);
        pending_clearing_job_.reset();
      }

      const auto t_start = std::chrono::steady_clock::now();
      auto clearing_points = job.virtual_rays_enabled ?
        buildVirtualClearingPoints(job.bins, job.profile, job.ray_origin_x, job.ray_origin_y) :
        std::move(job.points);
      clearing_points = limitPointCount(std::move(clearing_points), clearing_max_points_);
      auto clearing_output_msg = makePointCloud2FromPoints(clearing_points, job.header);
      clearing_publisher_->publish(clearing_output_msg);
      ++published_clearing_count_;
      if (publish_debug_log_) {
        const auto elapsed_ms = std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - t_start).count();
        RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "local perception clearing worker output=%zu timing_ms total=%.1f",
          clearing_points.size(), elapsed_ms);
      }
    }
  }

  builtin_interfaces::msg::Time nowMsg()
  {
    const auto now_ns = get_clock()->now().nanoseconds();
    builtin_interfaces::msg::Time stamp;
    stamp.sec = static_cast<std::int32_t>(now_ns / 1000000000LL);
    stamp.nanosec = static_cast<std::uint32_t>(now_ns % 1000000000LL);
    return stamp;
  }

  builtin_interfaces::msg::Time backedOffStamp(
    const builtin_interfaces::msg::Time & stamp,
    const double backoff_sec) const
  {
    if (backoff_sec <= 0.0) {
      return stamp;
    }
    const auto stamp_ns =
      static_cast<std::int64_t>(stamp.sec) * 1000000000LL + static_cast<std::int64_t>(stamp.nanosec);
    const auto backoff_ns = static_cast<std::int64_t>(std::llround(backoff_sec * 1000000000.0));
    const auto adjusted_ns = std::max<std::int64_t>(0LL, stamp_ns - backoff_ns);
    builtin_interfaces::msg::Time adjusted;
    adjusted.sec = static_cast<std::int32_t>(adjusted_ns / 1000000000LL);
    adjusted.nanosec = static_cast<std::uint32_t>(adjusted_ns % 1000000000LL);
    return adjusted;
  }

  builtin_interfaces::msg::Time shiftedStamp(
    const builtin_interfaces::msg::Time & stamp,
    const double shift_sec) const
  {
    if (std::abs(shift_sec) <= 1.0e-9) {
      return stamp;
    }
    const auto stamp_ns =
      static_cast<std::int64_t>(stamp.sec) * 1000000000LL + static_cast<std::int64_t>(stamp.nanosec);
    const auto shift_ns = static_cast<std::int64_t>(std::llround(shift_sec * 1000000000.0));
    const auto adjusted_ns = std::max<std::int64_t>(0LL, stamp_ns + shift_ns);
    builtin_interfaces::msg::Time adjusted;
    adjusted.sec = static_cast<std::int32_t>(adjusted_ns / 1000000000LL);
    adjusted.nanosec = static_cast<std::uint32_t>(adjusted_ns % 1000000000LL);
    return adjusted;
  }

  builtin_interfaces::msg::Time costmapOutputStamp()
  {
    return backedOffStamp(
      shiftedStamp(nowMsg(), output_stamp_forward_sec_),
      output_stamp_tf_backoff_sec_);
  }

  std::optional<builtin_interfaces::msg::Time> outputStampForCostmap()
  {
    if (!restamp_to_now_) {
      builtin_interfaces::msg::Time zero;
      return zero;
    }

    if (
      !restamp_to_latest_tf_ ||
      output_stamp_tf_target_frame_.empty() ||
      output_frame_id_.empty() ||
      output_stamp_tf_target_frame_ == output_frame_id_)
    {
      return costmapOutputStamp();
    }

    try {
      const auto timeout = tf2::durationFromSec(std::min(lookup_timeout_sec_, 0.05));
      const auto latest_tf = tf_buffer_.lookupTransform(
        output_stamp_tf_target_frame_, output_frame_id_, tf2::TimePointZero, timeout);
      if (latest_tf.header.stamp.sec != 0 || latest_tf.header.stamp.nanosec != 0) {
        const double tf_age_sec = now().seconds() - stamp_to_sec(latest_tf.header.stamp);
        if (tf_age_sec > max_output_tf_stamp_age_sec_) {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "Skipping local perception cloud because latest %s <- %s TF is stale: %.3fs > %.3fs.",
            output_stamp_tf_target_frame_.c_str(), output_frame_id_.c_str(), tf_age_sec,
            max_output_tf_stamp_age_sec_);
          if (require_output_stamp_tf_) {
            return std::nullopt;
          }
          return nowMsg();
        }
        // Gate output on a fresh odom<-base_link TF, then stamp at publication
        // time so Nav2 costmaps do not receive clouds in the TF future while
        // their independent buffers warm after lifecycle startup.
        return costmapOutputStamp();
      }
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Failed to restamp local perception cloud with latest %s <- %s TF: %s.",
        output_stamp_tf_target_frame_.c_str(), output_frame_id_.c_str(), ex.what());
      if (require_output_stamp_tf_) {
        return std::nullopt;
      }
    }

    if (latest_odom_stamp_) {
      const double odom_age_sec = now().seconds() - stamp_to_sec(*latest_odom_stamp_);
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Latest %s stamp is available but local costmap stamping requires latest %s <- %s TF; "
        "odom stamp age is %.3fs.",
        output_stamp_odom_topic_.c_str(),
        output_stamp_tf_target_frame_.c_str(),
        output_frame_id_.c_str(),
        odom_age_sec);
    }
    if (require_output_stamp_tf_) {
      return std::nullopt;
    }
    return costmapOutputStamp();
  }

  void publishEmptyCloud(const sensor_msgs::msg::PointCloud2 & input_msg)
  {
    sensor_msgs::msg::PointCloud2 output_msg = makePointCloud2FromPoints({}, input_msg.header);
    output_msg.header.frame_id = output_frame_id_;
    if (restamp_to_now_) {
      const auto output_stamp = outputStampForCostmap();
      if (!output_stamp) {
        return;
      }
      output_msg.header.stamp = *output_stamp;
    }
    publisher_->publish(output_msg);
  }

  bool shouldPublishClearingThisFrame() const
  {
    if (!clearing_enabled_) {
      return false;
    }
    if (clearing_publish_every_n_ <= 1) {
      return true;
    }
    return published_obstacle_count_ % static_cast<std::uint64_t>(clearing_publish_every_n_) == 0U;
  }

  void processLatestCloud()
  {
    if (!latest_cloud_ || latest_cloud_seq_ == last_processed_cloud_seq_) {
      return;
    }

    const auto t_start = std::chrono::steady_clock::now();
    auto msg = latest_cloud_;
    last_processed_cloud_seq_ = latest_cloud_seq_;
    const std::string source_frame =
      input_frame_id_override_.empty() ? msg->header.frame_id : input_frame_id_override_;
    if (!startupTfGateReady(source_frame)) {
      ++skipped_transform_count_;
      return;
    }

    const std::size_t point_count = static_cast<std::size_t>(msg->width) * msg->height;
    if (point_count == 0U || msg->point_step == 0U || msg->data.empty()) {
      publishEmptyCloud(*msg);
      return;
    }

    std::size_t x_offset = 0U;
    std::size_t y_offset = 0U;
    std::size_t z_offset = 0U;
    std::size_t intensity_offset = 0U;
    const bool has_xyz =
      !msg->is_bigendian &&
      find_float32_field_offset(*msg, "x", x_offset) &&
      find_float32_field_offset(*msg, "y", y_offset) &&
      find_float32_field_offset(*msg, "z", z_offset) &&
      x_offset + sizeof(float) <= msg->point_step &&
      y_offset + sizeof(float) <= msg->point_step &&
      z_offset + sizeof(float) <= msg->point_step;
    const bool has_intensity =
      !msg->is_bigendian &&
      find_float32_field_offset(*msg, "intensity", intensity_offset) &&
      intensity_offset + sizeof(float) <= msg->point_step;
    if (!has_xyz) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Skipping local perception cloud because %s does not expose little-endian FLOAT32 x/y/z fields.",
        input_topic_.c_str());
      ++skipped_field_count_;
      publishEmptyCloud(*msg);
      return;
    }
    const auto t_from_ros = std::chrono::steady_clock::now();

    Eigen::Matrix4f input_to_output = Eigen::Matrix4f::Identity();
    double ray_origin_x = 0.0;
    double ray_origin_y = 0.0;
    if (!output_frame_id_.empty() && source_frame != output_frame_id_) {
      try {
        builtin_interfaces::msg::Time transform_stamp;
        if (!input_transform_use_latest_) {
          transform_stamp = msg->header.stamp;
        }
        input_to_output = lookupTransformMatrix(output_frame_id_, source_frame, transform_stamp);
        ray_origin_x = static_cast<double>(input_to_output(0, 3));
        ray_origin_y = static_cast<double>(input_to_output(1, 3));
      } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN(
          get_logger(),
          "Skipping cloud because transform %s <- %s is unavailable: %s",
          output_frame_id_.c_str(), source_frame.c_str(), ex.what());
        ++skipped_transform_count_;
        return;
      }
    }
    const auto t_transform = std::chrono::steady_clock::now();

    const auto & profile = activeProfile();
    const bool publish_clearing_this_frame = shouldPublishClearingThisFrame();
    const auto fused_transform = makeFusedTransform(input_to_output);
    std::vector<PointT> filtered_points;
    filtered_points.reserve(point_count / static_cast<std::size_t>(point_sample_stride_) + 1U);
    std::vector<PointT> clearing_points;
    if (publish_clearing_this_frame) {
      clearing_points.reserve(point_count / static_cast<std::size_t>(clearing_point_sample_stride_) + 1U);
    }
    auto clearing_bins =
      publish_clearing_this_frame && clearing_virtual_rays_enabled_ ?
      makeClearingBins(profile) :
      std::vector<ClearingRayBin>{};

    for (std::size_t point_index = 0; point_index < point_count; ++point_index) {
      const std::size_t row = msg->width == 0U ? 0U : point_index / msg->width;
      const std::size_t column = msg->width == 0U ? 0U : point_index % msg->width;
      const std::size_t input_offset = row * msg->row_step + column * msg->point_step;
      if (input_offset + msg->point_step > msg->data.size()) {
        break;
      }
      const auto * point_data = msg->data.data() + input_offset;
      const float x = read_float32(point_data + x_offset);
      const float y = read_float32(point_data + y_offset);
      const float z = read_float32(point_data + z_offset);
      const float intensity = has_intensity ? read_float32(point_data + intensity_offset) : 0.0F;
      const PointT point = transformInputPoint(fused_transform, x, y, z, intensity);
      if (
        publish_clearing_this_frame &&
        point_index % static_cast<std::size_t>(clearing_point_sample_stride_) == 0U &&
        passesClearingFilters(point, profile))
      {
        if (clearing_virtual_rays_enabled_) {
          updateClearingBin(clearing_bins, profile, point, ray_origin_x, ray_origin_y);
        } else {
          clearing_points.push_back(point);
        }
      }
      if (point_index % static_cast<std::size_t>(point_sample_stride_) != 0U) {
        continue;
      }
      if (passesFilters(point, profile)) {
        filtered_points.push_back(point);
      }
    }
    filtered_points = applyVoxelOutlierFilter(std::move(filtered_points), profile);
    filtered_points = limitPointCount(std::move(filtered_points), max_filtered_points_);
    const auto t_filter = std::chrono::steady_clock::now();

    sensor_msgs::msg::PointCloud2 output_msg = makePointCloud2FromPoints(filtered_points, msg->header);
    output_msg.header.frame_id = output_frame_id_;
    const auto t_to_ros_obstacle = std::chrono::steady_clock::now();
    if (restamp_to_now_) {
      const auto output_stamp = outputStampForCostmap();
      if (!output_stamp) {
        ++skipped_stamp_count_;
        return;
      }
      output_msg.header.stamp = *output_stamp;
    }
    const auto t_stamp = std::chrono::steady_clock::now();
    auto t_to_ros_clearing = t_stamp;
    auto t_publish_clearing = t_stamp;
    publisher_->publish(output_msg);
    ++published_obstacle_count_;
    const auto t_publish_obstacle = std::chrono::steady_clock::now();
    t_publish_clearing = t_publish_obstacle;
    auto t_clearing = t_publish_obstacle;
    if (publish_clearing_this_frame) {
      PendingClearingJob job;
      job.header = output_msg.header;
      job.profile = profile;
      job.virtual_rays_enabled = clearing_virtual_rays_enabled_;
      job.ray_origin_x = ray_origin_x;
      job.ray_origin_y = ray_origin_y;
      if (clearing_virtual_rays_enabled_) {
        job.bins = std::move(clearing_bins);
      } else {
        job.points = std::move(clearing_points);
      }
      enqueueClearingJob(std::move(job));
      t_clearing = std::chrono::steady_clock::now();
      t_to_ros_clearing = t_clearing;
      t_publish_clearing = t_clearing;
    }
    std_msgs::msg::String mode_msg;
    mode_msg.data = current_mode_;
    mode_publisher_->publish(mode_msg);

    if (publish_debug_log_) {
      const auto elapsed_ms = [](const auto & begin, const auto & end) {
        return std::chrono::duration<double, std::milli>(end - begin).count();
      };
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "local perception mode=%s input=%zu sample_stride=%d output=%zu clearing_output=%zu "
        "timing_ms total=%.1f from_ros=%.1f transform=%.1f filter=%.1f clearing=%.1f "
        "to_ros_obstacle=%.1f stamp=%.1f to_ros_clearing=%.1f publish_clearing=%.1f publish_obstacle=%.1f",
        current_mode_.c_str(), point_count, point_sample_stride_, filtered_points.size(), clearing_points.size(),
        elapsed_ms(t_start, t_publish_clearing),
        elapsed_ms(t_start, t_from_ros),
        elapsed_ms(t_from_ros, t_transform),
        elapsed_ms(t_transform, t_filter),
        elapsed_ms(t_publish_obstacle, t_clearing),
        elapsed_ms(t_filter, t_to_ros_obstacle),
        elapsed_ms(t_to_ros_obstacle, t_stamp),
        elapsed_ms(t_stamp, t_to_ros_clearing),
        elapsed_ms(t_to_ros_clearing, t_publish_clearing),
        elapsed_ms(t_stamp, t_publish_obstacle));
    }
  }

  void logDebugStats()
  {
    if (!publish_debug_log_) {
      return;
    }
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "local perception stats received=%llu published_obstacle=%llu published_clearing=%llu "
      "skipped_field=%llu skipped_transform=%llu skipped_stamp=%llu",
      static_cast<unsigned long long>(received_cloud_count_),
      static_cast<unsigned long long>(published_obstacle_count_),
      static_cast<unsigned long long>(published_clearing_count_.load()),
      static_cast<unsigned long long>(skipped_field_count_),
      static_cast<unsigned long long>(skipped_transform_count_),
      static_cast<unsigned long long>(skipped_stamp_count_));
  }

  std::string mode_topic_;
  std::string input_topic_;
  bool input_reliable_{false};
  int input_qos_depth_{1};
  std::string output_topic_;
  std::string clearing_output_topic_;
  std::string output_frame_id_;
  std::string input_frame_id_override_;
  bool input_transform_use_latest_{true};
  std::array<float, 9> input_rotation_{
    1.0F, 0.0F, 0.0F,
    0.0F, 1.0F, 0.0F,
    0.0F, 0.0F, 1.0F};
  bool input_rotation_is_identity_{true};
  std::string output_stamp_tf_target_frame_;
  std::string output_stamp_odom_topic_;
  std::string current_mode_;
  std::vector<std::string> supported_modes_;
  bool restamp_to_now_{true};
  bool restamp_to_latest_tf_{false};
  bool require_output_stamp_tf_{false};
  double lookup_timeout_sec_{0.1};
  double max_output_tf_stamp_age_sec_{0.45};
  double output_stamp_tf_backoff_sec_{0.0};
  double output_stamp_forward_sec_{0.0};
  bool require_startup_tf_ready_{true};
  double startup_tf_warmup_sec_{1.0};
  bool startup_tf_ready_{false};
  std::chrono::steady_clock::time_point startup_time_{std::chrono::steady_clock::now()};
  double processing_rate_hz_{8.0};
  bool process_on_callback_{true};
  int point_sample_stride_{4};
  int max_filtered_points_{12000};
  bool clearing_enabled_{true};
  int clearing_publish_every_n_{1};
  double clearing_range_min_{0.10};
  double clearing_range_max_{5.00};
  double clearing_min_z_{-0.30};
  double clearing_max_z_{1.40};
  int clearing_point_sample_stride_{4};
  int clearing_max_points_{12000};
  bool clearing_virtual_rays_enabled_{true};
  double clearing_virtual_ray_angle_resolution_rad_{M_PI / 180.0};
  double clearing_virtual_ray_range_{5.0};
  std::vector<double> clearing_virtual_ray_ranges_;
  std::vector<double> clearing_virtual_ray_endpoint_z_values_;
  bool publish_debug_log_{false};
  std::map<std::string, ModeProfile> profiles_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr clearing_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mode_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_stamp_subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::mutex clearing_mutex_;
  std::condition_variable clearing_cv_;
  std::thread clearing_worker_;
  std::optional<PendingClearingJob> pending_clearing_job_;
  bool clearing_worker_stop_{false};
  sensor_msgs::msg::PointCloud2::SharedPtr latest_cloud_;
  std::optional<builtin_interfaces::msg::Time> latest_odom_stamp_;
  std::uint64_t latest_cloud_seq_{0};
  std::uint64_t last_processed_cloud_seq_{0};
  std::uint64_t received_cloud_count_{0};
  std::uint64_t published_obstacle_count_{0};
  std::atomic<std::uint64_t> published_clearing_count_{0};
  std::uint64_t skipped_field_count_{0};
  std::uint64_t skipped_transform_count_{0};
  std::uint64_t skipped_stamp_count_{0};
};

#ifndef ROBOT_LOCAL_PERCEPTION_COMPONENT_ONLY
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LocalPerceptionNode>());
  rclcpp::shutdown();
  return 0;
}
#endif

RCLCPP_COMPONENTS_REGISTER_NODE(LocalPerceptionNode)
