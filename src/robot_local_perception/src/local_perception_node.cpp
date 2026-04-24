#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <tuple>
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
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/exceptions.h"
#include "tf2/time.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace
{
using PointT = pcl::PointXYZI;

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
}  // namespace

class LocalPerceptionNode : public rclcpp::Node
{
public:
  LocalPerceptionNode()
  : Node("robot_local_perception"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    declare_parameter<bool>("mock_mode", true);
    mode_topic_ = declare_parameter<std::string>("mode_topic", "/robot_mode");
    input_topic_ = declare_parameter<std::string>("input_topic", "/lidar_points");
    output_topic_ = declare_parameter<std::string>("output_topic", "/perception/obstacle_points");
    clearing_output_topic_ = declare_parameter<std::string>("clearing_output_topic", "/perception/clearing_points");
    output_frame_id_ = declare_parameter<std::string>("output_frame_id", "base_link");
    output_stamp_tf_target_frame_ = declare_parameter<std::string>("output_stamp_tf_target_frame", "odom");
    current_mode_ = declare_parameter<std::string>("mode", "NORMAL");
    restamp_to_now_ = declare_parameter<bool>("restamp_to_now", true);
    restamp_to_latest_tf_ = declare_parameter<bool>("restamp_to_latest_tf", true);
    require_output_stamp_tf_ = declare_parameter<bool>("require_output_stamp_tf", true);
    lookup_timeout_sec_ = declare_parameter<double>("lookup_timeout_sec", 0.1);
    processing_rate_hz_ = std::max(declare_parameter<double>("processing_rate_hz", 8.0), 0.1);
    point_sample_stride_ = static_cast<int>(std::max<std::int64_t>(
      declare_parameter<std::int64_t>("point_sample_stride", 4), 1));
    max_filtered_points_ = static_cast<int>(std::max<std::int64_t>(
      declare_parameter<std::int64_t>("max_filtered_points", 12000), 1));
    clearing_enabled_ = declare_parameter<bool>("clearing.enabled", true);
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

    auto sensor_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
    publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, sensor_qos);
    clearing_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(clearing_output_topic_, sensor_qos);
    mode_publisher_ = create_publisher<std_msgs::msg::String>("/perception/mode", 10);
    subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_, sensor_qos, std::bind(&LocalPerceptionNode::cloudCallback, this, std::placeholders::_1));
    if (!mode_topic_.empty()) {
      mode_subscription_ = create_subscription<std_msgs::msg::String>(
        mode_topic_, 10, std::bind(&LocalPerceptionNode::modeCallback, this, std::placeholders::_1));
    }
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / processing_rate_hz_),
      std::bind(&LocalPerceptionNode::processLatestCloud, this));
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

  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    latest_cloud_ = msg;
    ++latest_cloud_seq_;
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

  pcl::PointCloud<PointT>::Ptr buildVirtualClearingCloud(
    const std::vector<ClearingRayBin> & bins,
    const ModeProfile & profile,
    const std_msgs::msg::Header & cloud_header,
    const double ray_origin_x,
    const double ray_origin_y) const
  {
    auto clearing_cloud = std::make_shared<pcl::PointCloud<PointT>>();
    clearing_cloud->header.frame_id = cloud_header.frame_id;
    clearing_cloud->reserve(
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
          clearing_cloud->push_back(endpoint);
        }
      }
    }
    return clearing_cloud;
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

  pcl::PointCloud<PointT>::Ptr applyVoxelOutlierFilter(
    const pcl::PointCloud<PointT>::Ptr & input_cloud,
    const ModeProfile & profile) const
  {
    if (!profile.outlier_filter.enabled || input_cloud->empty()) {
      return input_cloud;
    }

    const auto voxel_size = std::max(profile.outlier_filter.voxel_size, 1e-3);
    const auto min_points = std::max(profile.outlier_filter.min_points_per_voxel, 1);

    std::map<std::tuple<int, int, int>, int> counts;
    std::vector<std::tuple<int, int, int>> keys;
    keys.reserve(input_cloud->size());

    for (const auto & point : input_cloud->points) {
      const auto key = std::make_tuple(
        static_cast<int>(std::floor(point.x / voxel_size)),
        static_cast<int>(std::floor(point.y / voxel_size)),
        static_cast<int>(std::floor(point.z / voxel_size)));
      ++counts[key];
      keys.push_back(key);
    }

    auto filtered = std::make_shared<pcl::PointCloud<PointT>>();
    filtered->header = input_cloud->header;
    filtered->reserve(input_cloud->size());
    for (std::size_t index = 0; index < input_cloud->size(); ++index) {
      if (counts[keys[index]] >= min_points) {
        filtered->push_back(input_cloud->points[index]);
      }
    }
    return filtered;
  }

  builtin_interfaces::msg::Time nowMsg()
  {
    const auto now_ns = get_clock()->now().nanoseconds();
    builtin_interfaces::msg::Time stamp;
    stamp.sec = static_cast<std::int32_t>(now_ns / 1000000000LL);
    stamp.nanosec = static_cast<std::uint32_t>(now_ns % 1000000000LL);
    return stamp;
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
      return nowMsg();
    }

    try {
      const auto timeout = tf2::durationFromSec(std::min(lookup_timeout_sec_, 0.05));
      const auto latest_tf = tf_buffer_.lookupTransform(
        output_stamp_tf_target_frame_, output_frame_id_, tf2::TimePointZero, timeout);
      if (latest_tf.header.stamp.sec != 0 || latest_tf.header.stamp.nanosec != 0) {
        return latest_tf.header.stamp;
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
    return nowMsg();
  }

  void publishEmptyCloud(const sensor_msgs::msg::PointCloud2 & input_msg)
  {
    pcl::PointCloud<PointT> empty_cloud;
    empty_cloud.width = 0;
    empty_cloud.height = 1;
    empty_cloud.is_dense = false;
    sensor_msgs::msg::PointCloud2 output_msg;
    pcl::toROSMsg(empty_cloud, output_msg);
    output_msg.header = input_msg.header;
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

  void processLatestCloud()
  {
    if (!latest_cloud_ || latest_cloud_seq_ == last_processed_cloud_seq_) {
      return;
    }

    auto msg = latest_cloud_;
    last_processed_cloud_seq_ = latest_cloud_seq_;

    pcl::PointCloud<PointT>::Ptr raw_cloud(new pcl::PointCloud<PointT>());
    pcl::fromROSMsg(*msg, *raw_cloud);

    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*raw_cloud, *raw_cloud, indices);
    if (raw_cloud->empty()) {
      publishEmptyCloud(*msg);
      return;
    }

    pcl::PointCloud<PointT>::Ptr processing_cloud = raw_cloud;
    double ray_origin_x = 0.0;
    double ray_origin_y = 0.0;
    if (!output_frame_id_.empty() && msg->header.frame_id != output_frame_id_) {
      auto transformed_cloud = std::make_shared<pcl::PointCloud<PointT>>();
      try {
        const auto transform = lookupTransformMatrix(output_frame_id_, msg->header.frame_id, msg->header.stamp);
        ray_origin_x = static_cast<double>(transform(0, 3));
        ray_origin_y = static_cast<double>(transform(1, 3));
        pcl::transformPointCloud(*raw_cloud, *transformed_cloud, transform);
      } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN(
          get_logger(),
          "Skipping cloud because transform %s <- %s is unavailable: %s",
          output_frame_id_.c_str(), msg->header.frame_id.c_str(), ex.what());
        return;
      }
      processing_cloud = transformed_cloud;
    }

    const auto & profile = activeProfile();
    auto filtered_cloud = std::make_shared<pcl::PointCloud<PointT>>();
    filtered_cloud->header = processing_cloud->header;
    filtered_cloud->reserve(processing_cloud->size() / static_cast<std::size_t>(point_sample_stride_) + 1U);
    auto clearing_cloud = std::make_shared<pcl::PointCloud<PointT>>();
    clearing_cloud->header = processing_cloud->header;
    clearing_cloud->reserve(processing_cloud->size() / static_cast<std::size_t>(clearing_point_sample_stride_) + 1U);
    auto clearing_bins = clearing_virtual_rays_enabled_ ? makeClearingBins(profile) : std::vector<ClearingRayBin>{};

    for (std::size_t point_index = 0; point_index < processing_cloud->size(); ++point_index) {
      const auto & point = processing_cloud->points[point_index];
      if (
        clearing_enabled_ &&
        point_index % static_cast<std::size_t>(clearing_point_sample_stride_) == 0U &&
        passesClearingFilters(point, profile))
      {
        if (clearing_virtual_rays_enabled_) {
          updateClearingBin(clearing_bins, profile, point, ray_origin_x, ray_origin_y);
        } else {
          clearing_cloud->push_back(point);
        }
      }
      if (point_index % static_cast<std::size_t>(point_sample_stride_) != 0U) {
        continue;
      }
      if (passesFilters(point, profile)) {
        filtered_cloud->push_back(point);
      }
    }

    filtered_cloud = applyVoxelOutlierFilter(filtered_cloud, profile);
    if (clearing_enabled_ && clearing_virtual_rays_enabled_) {
      std_msgs::msg::Header cloud_header;
      cloud_header.frame_id = output_frame_id_;
      clearing_cloud = buildVirtualClearingCloud(clearing_bins, profile, cloud_header, ray_origin_x, ray_origin_y);
    }
    if (static_cast<int>(filtered_cloud->size()) > max_filtered_points_) {
      const auto reduction_stride = static_cast<std::size_t>(
        std::ceil(static_cast<double>(filtered_cloud->size()) / static_cast<double>(max_filtered_points_)));
      auto reduced_cloud = std::make_shared<pcl::PointCloud<PointT>>();
      reduced_cloud->header = filtered_cloud->header;
      reduced_cloud->reserve(static_cast<std::size_t>(max_filtered_points_));
      for (std::size_t index = 0; index < filtered_cloud->size(); index += reduction_stride) {
        reduced_cloud->push_back(filtered_cloud->points[index]);
      }
      filtered_cloud = reduced_cloud;
    }
    if (static_cast<int>(clearing_cloud->size()) > clearing_max_points_) {
      const auto reduction_stride = static_cast<std::size_t>(
        std::ceil(static_cast<double>(clearing_cloud->size()) / static_cast<double>(clearing_max_points_)));
      auto reduced_cloud = std::make_shared<pcl::PointCloud<PointT>>();
      reduced_cloud->header = clearing_cloud->header;
      reduced_cloud->reserve(static_cast<std::size_t>(clearing_max_points_));
      for (std::size_t index = 0; index < clearing_cloud->size(); index += reduction_stride) {
        reduced_cloud->push_back(clearing_cloud->points[index]);
      }
      clearing_cloud = reduced_cloud;
    }

    filtered_cloud->width = static_cast<std::uint32_t>(filtered_cloud->size());
    filtered_cloud->height = 1;
    filtered_cloud->is_dense = false;
    clearing_cloud->width = static_cast<std::uint32_t>(clearing_cloud->size());
    clearing_cloud->height = 1;
    clearing_cloud->is_dense = false;

    sensor_msgs::msg::PointCloud2 output_msg;
    pcl::toROSMsg(*filtered_cloud, output_msg);
    output_msg.header = msg->header;
    output_msg.header.frame_id = output_frame_id_;
    if (restamp_to_now_) {
      const auto output_stamp = outputStampForCostmap();
      if (!output_stamp) {
        return;
      }
      output_msg.header.stamp = *output_stamp;
    }
    if (clearing_enabled_) {
      sensor_msgs::msg::PointCloud2 clearing_output_msg;
      pcl::toROSMsg(*clearing_cloud, clearing_output_msg);
      clearing_output_msg.header = output_msg.header;
      clearing_publisher_->publish(clearing_output_msg);
    }
    publisher_->publish(output_msg);
    std_msgs::msg::String mode_msg;
    mode_msg.data = current_mode_;
    mode_publisher_->publish(mode_msg);

    if (publish_debug_log_) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "local perception mode=%s input=%zu sample_stride=%d output=%zu clearing_output=%zu",
        current_mode_.c_str(), raw_cloud->size(), point_sample_stride_, filtered_cloud->size(), clearing_cloud->size());
    }
  }

  std::string mode_topic_;
  std::string input_topic_;
  std::string output_topic_;
  std::string clearing_output_topic_;
  std::string output_frame_id_;
  std::string output_stamp_tf_target_frame_;
  std::string current_mode_;
  std::vector<std::string> supported_modes_;
  bool restamp_to_now_{true};
  bool restamp_to_latest_tf_{true};
  bool require_output_stamp_tf_{true};
  double lookup_timeout_sec_{0.1};
  double processing_rate_hz_{8.0};
  int point_sample_stride_{4};
  int max_filtered_points_{12000};
  bool clearing_enabled_{true};
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
  rclcpp::TimerBase::SharedPtr timer_;
  sensor_msgs::msg::PointCloud2::SharedPtr latest_cloud_;
  std::uint64_t latest_cloud_seq_{0};
  std::uint64_t last_processed_cloud_seq_{0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LocalPerceptionNode>());
  rclcpp::shutdown();
  return 0;
}
