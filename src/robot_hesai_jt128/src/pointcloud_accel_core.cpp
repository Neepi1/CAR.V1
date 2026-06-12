#include <algorithm>
#include <array>
#include <atomic>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iomanip>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#ifdef __linux__
#include <pthread.h>
#endif

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "std_msgs/msg/header.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/exceptions.h"
#include "tf2/time.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "robot_hesai_jt128/pointcloud_accel_core.hpp"

namespace
{

using Clock = std::chrono::steady_clock;
constexpr double kPi = 3.1415926535897932384626433832795;

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

double stamp_to_sec(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1.0e-9;
}

double age_ms_from_stamp_sec(const double now_sec, const double stamp_sec)
{
  if (stamp_sec <= 0.0) {
    return -1.0;
  }
  return (now_sec - stamp_sec) * 1000.0;
}

double age_ms_from_ros_stamp(const double now_sec, const rclcpp::Time & stamp)
{
  if (stamp.nanoseconds() <= 0) {
    return -1.0;
  }
  return age_ms_from_stamp_sec(now_sec, stamp.seconds());
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

bool has_xyz_fields(const sensor_msgs::msg::PointCloud2 & msg)
{
  std::size_t ignored = 0U;
  return find_float32_field_offset(msg, "x", ignored) &&
         find_float32_field_offset(msg, "y", ignored) &&
         find_float32_field_offset(msg, "z", ignored);
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

sensor_msgs::msg::PointField make_float32_field(const std::string & name, const std::uint32_t offset)
{
  sensor_msgs::msg::PointField field;
  field.name = name;
  field.offset = offset;
  field.datatype = sensor_msgs::msg::PointField::FLOAT32;
  field.count = 1U;
  return field;
}

std::string lower_copy(std::string value)
{
  std::transform(value.begin(), value.end(), value.begin(), [](const unsigned char ch) {
    return static_cast<char>(std::tolower(ch));
  });
  return value;
}

bool profile_uses_workers(const std::string & profile)
{
  const auto lower = lower_copy(profile);
  return lower == "ipc_worker" || lower == "nitros";
}

double safe_rate_period_sec(const double hz, const double fallback_hz)
{
  const double effective_hz = hz > 0.01 ? hz : fallback_hz;
  return 1.0 / std::max(effective_hz, 0.01);
}

void set_thread_name(const char * name)
{
#ifdef __linux__
  pthread_setname_np(pthread_self(), name);
#else
  (void)name;
#endif
}

struct Transform3x4
{
  double m00{1.0};
  double m01{0.0};
  double m02{0.0};
  double m03{0.0};
  double m10{0.0};
  double m11{1.0};
  double m12{0.0};
  double m13{0.0};
  double m20{0.0};
  double m21{0.0};
  double m22{1.0};
  double m23{0.0};
};

struct NormalizedPointView
{
  float x{0.0F};
  float y{0.0F};
  float z{0.0F};
  float intensity{0.0F};
};

using PointXYZI = NormalizedPointView;

struct LatestNormalizedBuffer
{
  std::vector<NormalizedPointView> points;
  rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
  std::string frame_id;
  std::uint64_t seq{0U};
  Clock::time_point update_time{};
  bool has_intensity{false};
  std::size_t source_bytes{0U};
};

PointXYZI transform_point(const Transform3x4 & transform, const PointXYZI & point)
{
  return PointXYZI{
    static_cast<float>(
      transform.m00 * point.x + transform.m01 * point.y + transform.m02 * point.z + transform.m03),
    static_cast<float>(
      transform.m10 * point.x + transform.m11 * point.y + transform.m12 * point.z + transform.m13),
    static_cast<float>(
      transform.m20 * point.x + transform.m21 * point.y + transform.m22 * point.z + transform.m23),
    point.intensity};
}

struct CloudFieldOffsets
{
  bool valid{false};
  bool has_intensity{false};
  std::size_t x{0U};
  std::size_t y{0U};
  std::size_t z{0U};
  std::size_t intensity{0U};
};

CloudFieldOffsets field_offsets(const sensor_msgs::msg::PointCloud2 & cloud)
{
  CloudFieldOffsets offsets;
  offsets.valid =
    !cloud.is_bigendian &&
    find_float32_field_offset(cloud, "x", offsets.x) &&
    find_float32_field_offset(cloud, "y", offsets.y) &&
    find_float32_field_offset(cloud, "z", offsets.z);
  offsets.has_intensity =
    offsets.valid &&
    find_float32_field_offset(cloud, "intensity", offsets.intensity) &&
    offsets.intensity + sizeof(float) <= cloud.point_step;
  return offsets;
}

std::optional<PointXYZI> read_point(
  const sensor_msgs::msg::PointCloud2 & cloud,
  const CloudFieldOffsets & offsets,
  const std::size_t flat_index)
{
  if (!offsets.valid || cloud.width == 0U || cloud.point_step == 0U) {
    return std::nullopt;
  }
  const std::size_t row = flat_index / cloud.width;
  const std::size_t column = flat_index % cloud.width;
  const std::size_t input_offset = row * cloud.row_step + column * cloud.point_step;
  if (input_offset + cloud.point_step > cloud.data.size()) {
    return std::nullopt;
  }
  const auto * data = cloud.data.data() + input_offset;
  return PointXYZI{
    read_float32(data + offsets.x),
    read_float32(data + offsets.y),
    read_float32(data + offsets.z),
    offsets.has_intensity ? read_float32(data + offsets.intensity) : 0.0F};
}

sensor_msgs::msg::PointCloud2 make_xyzi_cloud(
  const std::vector<PointXYZI> & points,
  const std_msgs::msg::Header & header,
  const bool include_intensity)
{
  sensor_msgs::msg::PointCloud2 msg;
  msg.header = header;
  msg.height = 1U;
  msg.width = static_cast<std::uint32_t>(points.size());
  msg.fields = {
    make_float32_field("x", 0U),
    make_float32_field("y", 4U),
    make_float32_field("z", 8U)};
  msg.point_step = 12U;
  if (include_intensity) {
    msg.fields.push_back(make_float32_field("intensity", 12U));
    msg.point_step = 16U;
  }
  msg.is_bigendian = false;
  msg.row_step = msg.point_step * msg.width;
  msg.is_dense = false;
  msg.data.resize(static_cast<std::size_t>(msg.row_step), 0U);
  for (std::size_t i = 0; i < points.size(); ++i) {
    auto * data = msg.data.data() + i * msg.point_step;
    write_float32(data + 0U, points[i].x);
    write_float32(data + 4U, points[i].y);
    write_float32(data + 8U, points[i].z);
    if (include_intensity) {
      write_float32(data + 12U, points[i].intensity);
    }
  }
  return msg;
}

struct CompactDiagnostics
{
  std::uint64_t publish_count{0U};
  std::uint64_t previous_publish_count{0U};
  std::uint64_t skip_busy_count{0U};
  std::uint64_t allocation_count{0U};
  std::size_t last_points{0U};
  std::size_t last_bytes{0U};
  std::size_t bytes_per_point{0U};
  bool intensity_missing{false};
};

struct WorkerDiagnostics
{
  std::uint64_t tick_count{0U};
  std::uint64_t previous_tick_count{0U};
  std::uint64_t processed_count{0U};
  std::uint64_t previous_processed_count{0U};
  std::uint64_t obstacle_publish_count{0U};
  std::uint64_t previous_obstacle_publish_count{0U};
  std::uint64_t clearing_publish_count{0U};
  std::uint64_t previous_clearing_publish_count{0U};
  std::uint64_t scan_publish_count{0U};
  std::uint64_t previous_scan_publish_count{0U};
  std::uint64_t skip_busy_count{0U};
  std::uint64_t full_cloud_copy_count{0U};
  std::uint64_t intermediate_pointcloud_build_count{0U};
  std::uint64_t allocation_count{0U};
  std::size_t last_obstacle_output_bytes{0U};
  std::size_t last_clearing_output_bytes{0U};
  std::size_t last_scan_output_ranges{0U};
  std::size_t last_scan_output_bytes{0U};
  bool reused_output_buffer{true};
  bool scan_bins_reused{true};
  double last_processing_ms{0.0};
  double processing_ms_max{0.0};
  double processing_ms_sum{0.0};
  std::uint64_t processing_ms_count{0U};
  double lock_wait_ms_max{0.0};
  double start_source_age_ms{-1.0};
  double end_source_age_ms{-1.0};
  double obstacle_output_header_age_ms{-1.0};
  double obstacle_output_source_age_ms{-1.0};
  double obstacle_output_publish_delay_ms{-1.0};
  double clearing_output_header_age_ms{-1.0};
  double clearing_output_source_age_ms{-1.0};
  double clearing_output_publish_delay_ms{-1.0};
  double scan_output_header_age_ms{-1.0};
  double scan_output_source_age_ms{-1.0};
};

struct ClearingRayBin
{
  bool has_return{false};
  double range_xy{0.0};
  double angle_rad{0.0};
};

}  // namespace

namespace robot_hesai_jt128
{

class PointCloudAccelCore::Impl
{
public:
  Impl(rclcpp::Node & node, PointCloudAccelCoreOptions options)
  : node_(node),
    options_(std::move(options)),
    tf_buffer_(node_.get_clock()),
    tf_listener_(tf_buffer_),
    logged_ready_(false),
    warned_missing_xyz_(false)
  {
    node_.declare_parameter<std::string>("input_topic", "/jt128/vendor/points_raw");
    node_.declare_parameter<std::string>("output_topic", "/lidar_points");
    node_.declare_parameter<std::string>("output_frame_id", "lidar_link");
    node_.declare_parameter<std::string>("accel_profile", "ipc_worker");
    node_.declare_parameter<int>("input_qos_depth", 1);
    node_.declare_parameter<bool>("input_reliable", false);
    node_.declare_parameter<int>("output_qos_depth", 1);
    node_.declare_parameter<bool>("output_reliable", false);
    node_.declare_parameter<std::string>("status_topic", "/lidar/axis_remap_status");
    node_.declare_parameter<std::string>("accel_status_topic", "/lidar/pointcloud_accel_status");
    node_.declare_parameter<double>("status_publish_period_sec", 1.0);
    node_.declare_parameter<std::vector<double>>(
      "rotation_matrix",
      std::vector<double>{
        1.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
        0.0, 0.0, 1.0,
      });

    node_.declare_parameter<std::string>("local_output_topic", "/_internal/lidar_points_local");
    node_.declare_parameter<int>("local_output_stride", 4);
    node_.declare_parameter<int>("local_output_publish_every_n", 1);
    node_.declare_parameter<std::string>("local_compact_fields", "xyzi");
    node_.declare_parameter<bool>("local_compact_enabled", true);
    node_.declare_parameter<int>("local_compact_stride", 4);
    node_.declare_parameter<double>("local_compact_max_rate_hz", 12.0);
    node_.declare_parameter<int>("local_output_qos_depth", 1);

    node_.declare_parameter<std::string>("nav_output_topic", "/lidar_points_nav");
    node_.declare_parameter<int>("nav_output_stride", 4);
    node_.declare_parameter<int>("nav_output_publish_every_n", 2);
    node_.declare_parameter<std::string>("nav_compact_fields", "xyzi");
    node_.declare_parameter<bool>("nav_compact_enabled", true);
    node_.declare_parameter<int>("nav_compact_stride", 4);
    node_.declare_parameter<double>("nav_compact_max_rate_hz", 10.0);
    node_.declare_parameter<int>("nav_output_qos_depth", 1);

    node_.declare_parameter<bool>("worker_local_enabled", true);
    node_.declare_parameter<bool>("local_worker_enabled", true);
    node_.declare_parameter<std::string>("obstacle_output_topic", "/perception/obstacle_points");
    node_.declare_parameter<std::string>("clearing_output_topic", "/perception/clearing_points");
    node_.declare_parameter<std::string>("local_worker_output_frame_id", "base_link");
    node_.declare_parameter<double>("local_worker_rate_hz", 12.0);
    node_.declare_parameter<double>("local_worker_range_min", 0.5);
    node_.declare_parameter<double>("local_worker_range_max", 5.5);
    node_.declare_parameter<double>("local_worker_min_z", 0.40);
    node_.declare_parameter<double>("local_worker_max_z", 1.30);
    node_.declare_parameter<double>("local_worker_min_angle_deg", -110.0);
    node_.declare_parameter<double>("local_worker_max_angle_deg", 110.0);
    node_.declare_parameter<int>("local_worker_point_stride", 1);
    node_.declare_parameter<int>("local_worker_max_points", 12000);
    node_.declare_parameter<double>("local_worker_self_mask_min_x", -0.50);
    node_.declare_parameter<double>("local_worker_self_mask_max_x", 0.45);
    node_.declare_parameter<double>("local_worker_self_mask_min_y", -0.40);
    node_.declare_parameter<double>("local_worker_self_mask_max_y", 0.40);
    node_.declare_parameter<double>("local_worker_self_mask_min_z", -0.20);
    node_.declare_parameter<double>("local_worker_self_mask_max_z", 1.40);
    node_.declare_parameter<double>("clearing_worker_rate_hz", 4.0);
    node_.declare_parameter<double>("clearing_worker_range_min", 0.10);
    node_.declare_parameter<double>("clearing_worker_range_max", 8.0);
    node_.declare_parameter<double>("clearing_worker_min_z", -0.30);
    node_.declare_parameter<double>("clearing_worker_max_z", 1.40);
    node_.declare_parameter<int>("clearing_worker_point_stride", 2);
    node_.declare_parameter<int>("clearing_worker_max_points", 15000);
    node_.declare_parameter<bool>("clearing_worker_virtual_rays_enabled", true);
    node_.declare_parameter<double>("clearing_worker_virtual_ray_angle_resolution_deg", 1.0);
    node_.declare_parameter<double>("clearing_worker_virtual_ray_range", 8.0);
    node_.declare_parameter<std::vector<double>>(
      "clearing_worker_virtual_ray_range_steps",
      std::vector<double>{0.50, 1.00, 2.00, 3.50, 5.50, 8.00});
    node_.declare_parameter<std::vector<double>>(
      "clearing_worker_virtual_ray_endpoint_z_values",
      std::vector<double>{-0.10, 0.05, 0.20, 0.40, 0.60, 0.85, 1.10, 1.30});

    node_.declare_parameter<bool>("worker_scan_enabled", true);
    node_.declare_parameter<bool>("scan_worker_enabled", true);
    node_.declare_parameter<std::string>("flatscan_output_topic", "/flatscan");
    node_.declare_parameter<std::string>("scan_output_topic", "/scan");
    node_.declare_parameter<std::string>("scan_worker_frame_id", "lidar_level_link");
    node_.declare_parameter<double>("scan_worker_rate_hz", 9.0);
    node_.declare_parameter<double>("scan_worker_min_height", -0.75);
    node_.declare_parameter<double>("scan_worker_max_height", 0.35);
    node_.declare_parameter<double>("scan_worker_angle_min", -3.141592653589793);
    node_.declare_parameter<double>("scan_worker_angle_max", 3.141592653589793);
    node_.declare_parameter<double>("scan_worker_angle_increment", 0.004363323129985824);
    node_.declare_parameter<double>("scan_worker_range_min", 0.25);
    node_.declare_parameter<double>("scan_worker_range_max", 40.0);
    node_.declare_parameter<bool>("scan_worker_use_inf", true);
    node_.declare_parameter<double>("scan_worker_inf_epsilon", 1.0);

    input_topic_ = node_.get_parameter("input_topic").as_string();
    output_topic_ = node_.get_parameter("output_topic").as_string();
    output_frame_id_ = node_.get_parameter("output_frame_id").as_string();
    accel_profile_ = lower_copy(node_.get_parameter("accel_profile").as_string());
    status_topic_ = node_.get_parameter("status_topic").as_string();
    accel_status_topic_ = node_.get_parameter("accel_status_topic").as_string();
    status_publish_period_sec_ = std::max(node_.get_parameter("status_publish_period_sec").as_double(), 0.0);
    input_qos_depth_ = positive_size_param("input_qos_depth", 1U);
    output_qos_depth_ = positive_size_param("output_qos_depth", 1U);
    nav_output_qos_depth_ = positive_size_param("nav_output_qos_depth", 1U);
    local_output_qos_depth_ = positive_size_param("local_output_qos_depth", 1U);
    rotation_ = load_rotation_matrix();

    local_output_topic_ = node_.get_parameter("local_output_topic").as_string();
    local_compact_enabled_ = node_.get_parameter("local_compact_enabled").as_bool();
    local_compact_fields_ = lower_copy(node_.get_parameter("local_compact_fields").as_string());
    local_compact_stride_ = positive_size_param("local_compact_stride", 4U);
    local_compact_max_rate_hz_ = std::max(node_.get_parameter("local_compact_max_rate_hz").as_double(), 0.1);
    nav_output_topic_ = node_.get_parameter("nav_output_topic").as_string();
    nav_compact_enabled_ = node_.get_parameter("nav_compact_enabled").as_bool();
    nav_compact_fields_ = lower_copy(node_.get_parameter("nav_compact_fields").as_string());
    nav_compact_stride_ = positive_size_param("nav_compact_stride", 4U);
    nav_compact_max_rate_hz_ = std::max(node_.get_parameter("nav_compact_max_rate_hz").as_double(), 0.1);
    sanitize_compact_fields(local_compact_fields_);
    sanitize_compact_fields(nav_compact_fields_);

    worker_local_enabled_ =
      node_.get_parameter("worker_local_enabled").as_bool() &&
      node_.get_parameter("local_worker_enabled").as_bool() &&
      profile_uses_workers(accel_profile_);
    worker_scan_enabled_ =
      node_.get_parameter("worker_scan_enabled").as_bool() &&
      node_.get_parameter("scan_worker_enabled").as_bool() &&
      profile_uses_workers(accel_profile_);
    obstacle_output_topic_ = node_.get_parameter("obstacle_output_topic").as_string();
    clearing_output_topic_ = node_.get_parameter("clearing_output_topic").as_string();
    local_worker_output_frame_id_ = node_.get_parameter("local_worker_output_frame_id").as_string();
    local_worker_rate_hz_ = std::max(node_.get_parameter("local_worker_rate_hz").as_double(), 0.1);
    local_worker_range_min_ = node_.get_parameter("local_worker_range_min").as_double();
    local_worker_range_max_ = node_.get_parameter("local_worker_range_max").as_double();
    local_worker_min_z_ = node_.get_parameter("local_worker_min_z").as_double();
    local_worker_max_z_ = node_.get_parameter("local_worker_max_z").as_double();
    local_worker_min_angle_rad_ = node_.get_parameter("local_worker_min_angle_deg").as_double() * kPi / 180.0;
    local_worker_max_angle_rad_ = node_.get_parameter("local_worker_max_angle_deg").as_double() * kPi / 180.0;
    local_worker_point_stride_ = positive_size_param("local_worker_point_stride", 1U);
    local_worker_max_points_ = positive_size_param("local_worker_max_points", 12000U);
    self_mask_min_x_ = node_.get_parameter("local_worker_self_mask_min_x").as_double();
    self_mask_max_x_ = node_.get_parameter("local_worker_self_mask_max_x").as_double();
    self_mask_min_y_ = node_.get_parameter("local_worker_self_mask_min_y").as_double();
    self_mask_max_y_ = node_.get_parameter("local_worker_self_mask_max_y").as_double();
    self_mask_min_z_ = node_.get_parameter("local_worker_self_mask_min_z").as_double();
    self_mask_max_z_ = node_.get_parameter("local_worker_self_mask_max_z").as_double();
    clearing_worker_rate_hz_ = std::max(node_.get_parameter("clearing_worker_rate_hz").as_double(), 0.1);
    clearing_worker_range_min_ = node_.get_parameter("clearing_worker_range_min").as_double();
    clearing_worker_range_max_ = node_.get_parameter("clearing_worker_range_max").as_double();
    clearing_worker_min_z_ = node_.get_parameter("clearing_worker_min_z").as_double();
    clearing_worker_max_z_ = node_.get_parameter("clearing_worker_max_z").as_double();
    clearing_worker_point_stride_ = positive_size_param("clearing_worker_point_stride", 2U);
    clearing_worker_max_points_ = positive_size_param("clearing_worker_max_points", 15000U);
    clearing_worker_virtual_rays_enabled_ =
      node_.get_parameter("clearing_worker_virtual_rays_enabled").as_bool();
    clearing_worker_virtual_ray_angle_resolution_rad_ =
      std::clamp(
      node_.get_parameter("clearing_worker_virtual_ray_angle_resolution_deg").as_double(),
      0.2,
      10.0) * kPi / 180.0;
    clearing_worker_virtual_ray_range_ =
      std::max(clearing_worker_range_min_, node_.get_parameter("clearing_worker_virtual_ray_range").as_double());
    clearing_worker_virtual_ray_ranges_ =
      node_.get_parameter("clearing_worker_virtual_ray_range_steps").as_double_array();
    clearing_worker_virtual_ray_endpoint_z_values_ =
      node_.get_parameter("clearing_worker_virtual_ray_endpoint_z_values").as_double_array();
    sanitize_clearing_virtual_rays();

    scan_output_topic_ = node_.get_parameter("scan_output_topic").as_string();
    scan_worker_frame_id_ = node_.get_parameter("scan_worker_frame_id").as_string();
    scan_worker_rate_hz_ = std::max(node_.get_parameter("scan_worker_rate_hz").as_double(), 0.1);
    scan_worker_min_height_ = node_.get_parameter("scan_worker_min_height").as_double();
    scan_worker_max_height_ = node_.get_parameter("scan_worker_max_height").as_double();
    scan_worker_angle_min_ = node_.get_parameter("scan_worker_angle_min").as_double();
    scan_worker_angle_max_ = node_.get_parameter("scan_worker_angle_max").as_double();
    scan_worker_angle_increment_ = std::max(node_.get_parameter("scan_worker_angle_increment").as_double(), 1.0e-5);
    scan_worker_range_min_ = node_.get_parameter("scan_worker_range_min").as_double();
    scan_worker_range_max_ = node_.get_parameter("scan_worker_range_max").as_double();
    scan_worker_use_inf_ = node_.get_parameter("scan_worker_use_inf").as_bool();
    scan_worker_inf_epsilon_ = node_.get_parameter("scan_worker_inf_epsilon").as_double();

    input_reliability_ = node_.get_parameter("input_reliable").as_bool() ?
      RMW_QOS_POLICY_RELIABILITY_RELIABLE : RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
    const auto output_reliability = node_.get_parameter("output_reliable").as_bool() ?
      RMW_QOS_POLICY_RELIABILITY_RELIABLE : RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;

    trunk_publisher_ = node_.create_publisher<sensor_msgs::msg::PointCloud2>(
      output_topic_, make_qos(output_qos_depth_, output_reliability));
    if (local_compact_enabled_ && !local_output_topic_.empty()) {
      local_compact_publisher_ = node_.create_publisher<sensor_msgs::msg::PointCloud2>(
        local_output_topic_, make_qos(local_output_qos_depth_, RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT));
    }
    if (nav_compact_enabled_ && !nav_output_topic_.empty()) {
      nav_compact_publisher_ = node_.create_publisher<sensor_msgs::msg::PointCloud2>(
        nav_output_topic_, make_qos(nav_output_qos_depth_, RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT));
    }
    if (worker_local_enabled_) {
      obstacle_publisher_ = node_.create_publisher<sensor_msgs::msg::PointCloud2>(
        obstacle_output_topic_, make_qos(1U, RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT));
      clearing_publisher_ = node_.create_publisher<sensor_msgs::msg::PointCloud2>(
        clearing_output_topic_, make_qos(1U, RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT));
    }
    if (worker_scan_enabled_) {
      scan_publisher_ = node_.create_publisher<sensor_msgs::msg::LaserScan>(
        scan_output_topic_, rclcpp::SensorDataQoS());
    }
    if (!status_topic_.empty() && status_publish_period_sec_ > 0.0) {
      status_publisher_ = node_.create_publisher<std_msgs::msg::String>(status_topic_, 10);
      accel_status_publisher_ = node_.create_publisher<std_msgs::msg::String>(accel_status_topic_, 10);
      status_timer_ = node_.create_wall_timer(
        std::chrono::duration<double>(status_publish_period_sec_),
        std::bind(&PointCloudAccelCore::Impl::publish_status, this));
      previous_status_time_ = Clock::now();
    }
    if (local_compact_publisher_ || worker_local_enabled_) {
      local_worker_thread_ = std::thread(&PointCloudAccelCore::Impl::local_worker_loop, this);
    }
    if (nav_compact_publisher_ || worker_scan_enabled_) {
      scan_worker_thread_ = std::thread(&PointCloudAccelCore::Impl::scan_worker_loop, this);
    }
  }

  const std::string & input_topic() const
  {
    return input_topic_;
  }

  rclcpp::QoS input_qos() const
  {
    return make_qos(input_qos_depth_, input_reliability_);
  }

  void process_pointcloud2(sensor_msgs::msg::PointCloud2::UniquePtr msg)
  {
    ++accel_core_process_pointcloud2_count_;
    process_pointcloud2_internal(std::move(msg));
  }

  void process_pointcloud2(const sensor_msgs::msg::PointCloud2 & msg)
  {
    auto copy = std::make_unique<sensor_msgs::msg::PointCloud2>(msg);
    process_pointcloud2(std::move(copy));
  }

  bool process_decoded_points(const DecodedCloudView & view)
  {
    (void)view;
    ++accel_core_process_decoded_view_count_;
    RCLCPP_WARN_ONCE(
      node_.get_logger(),
      "DecodedCloudView AccelCore path is not wired until a repo-owned Hesai driver overlay is available");
    return false;
  }

  ~Impl()
  {
    stop_workers_.store(true);
    if (local_worker_thread_.joinable()) {
      local_worker_thread_.join();
    }
    if (scan_worker_thread_.joinable()) {
      scan_worker_thread_.join();
    }
  }

private:
  std::size_t positive_size_param(const std::string & name, const std::size_t fallback) const
  {
    const auto raw = node_.get_parameter(name).as_int();
    return static_cast<std::size_t>(raw > 0 ? raw : static_cast<std::int64_t>(fallback));
  }

  std::array<float, 9> load_rotation_matrix() const
  {
    const auto raw = node_.get_parameter("rotation_matrix").as_double_array();
    if (raw.size() != 9U) {
      throw std::runtime_error("rotation_matrix must contain 9 values");
    }
    std::array<float, 9> rotation{};
    for (std::size_t i = 0U; i < rotation.size(); ++i) {
      rotation[i] = static_cast<float>(raw[i]);
    }
    return rotation;
  }

  static void sanitize_compact_fields(std::string & fields)
  {
    fields = lower_copy(fields);
    if (fields != "xyz" && fields != "xyzi") {
      fields = "xyzi";
    }
  }

  bool lookup_transform(
    const std::string & target_frame,
    const std::string & source_frame,
    Transform3x4 & output)
  {
    if (target_frame.empty() || source_frame.empty() || target_frame == source_frame) {
      output = Transform3x4{};
      return true;
    }
    try {
      const auto transform_msg = tf_buffer_.lookupTransform(
        target_frame, source_frame, tf2::TimePointZero, tf2::durationFromSec(0.02));
      const auto & t = transform_msg.transform.translation;
      const auto & q_msg = transform_msg.transform.rotation;
      tf2::Quaternion q(q_msg.x, q_msg.y, q_msg.z, q_msg.w);
      q.normalize();
      tf2::Matrix3x3 m(q);
      output.m00 = m[0][0];
      output.m01 = m[0][1];
      output.m02 = m[0][2];
      output.m03 = t.x;
      output.m10 = m[1][0];
      output.m11 = m[1][1];
      output.m12 = m[1][2];
      output.m13 = t.y;
      output.m20 = m[2][0];
      output.m21 = m[2][1];
      output.m22 = m[2][2];
      output.m23 = t.z;
      return true;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        node_.get_logger(), *node_.get_clock(), 2000,
        "pointcloud accel worker cannot lookup %s <- %s: %s",
        target_frame.c_str(), source_frame.c_str(), ex.what());
      return false;
    }
  }

  static builtin_interfaces::msg::Time stamp_to_msg(const rclcpp::Time & stamp)
  {
    const auto total_ns = stamp.nanoseconds();
    builtin_interfaces::msg::Time msg;
    msg.sec = static_cast<std::int32_t>(total_ns / 1000000000LL);
    msg.nanosec = static_cast<std::uint32_t>(total_ns % 1000000000LL);
    return msg;
  }

  static std_msgs::msg::Header header_from_buffer(const LatestNormalizedBuffer & buffer)
  {
    std_msgs::msg::Header header;
    header.stamp = stamp_to_msg(buffer.stamp);
    header.frame_id = buffer.frame_id;
    return header;
  }

  std::shared_ptr<LatestNormalizedBuffer> take_reusable_normalized_buffer()
  {
    std::lock_guard<std::mutex> lock(latest_cloud_mutex_);
    if (reusable_normalized_buffer_) {
      auto buffer = reusable_normalized_buffer_;
      reusable_normalized_buffer_.reset();
      return buffer;
    }
    ++normalized_buffer_allocation_count_;
    return std::make_shared<LatestNormalizedBuffer>();
  }

  std::shared_ptr<LatestNormalizedBuffer> build_latest_normalized_buffer(
    const sensor_msgs::msg::PointCloud2 & cloud)
  {
    auto buffer = take_reusable_normalized_buffer();
    buffer->stamp = rclcpp::Time(cloud.header.stamp);
    buffer->frame_id = cloud.header.frame_id;
    buffer->seq = 0U;
    buffer->update_time = Clock::time_point{};
    buffer->source_bytes = cloud.data.size();
    buffer->has_intensity = false;
    buffer->points.clear();

    const auto offsets = field_offsets(cloud);
    if (!offsets.valid || cloud.point_step == 0U || cloud.width == 0U || cloud.height == 0U) {
      return buffer;
    }

    buffer->has_intensity = offsets.has_intensity;
    const std::size_t point_count = static_cast<std::size_t>(cloud.width) * cloud.height;
    if (buffer->points.capacity() < point_count) {
      ++normalized_buffer_allocation_count_;
      buffer->points.reserve(point_count);
    }
    for (std::size_t i = 0U; i < point_count; ++i) {
      const auto point = read_point(cloud, offsets, i);
      if (!point) {
        break;
      }
      buffer->points.push_back(*point);
    }
    return buffer;
  }

  void publish_fast_path(
    std::unique_ptr<sensor_msgs::msg::PointCloud2> output,
    const Clock::time_point & callback_start,
    std::shared_ptr<LatestNormalizedBuffer> normalized_buffer)
  {
    const auto publish_start = Clock::now();
    if (last_publish_time_ != Clock::time_point{}) {
      const double interval_ms =
        std::chrono::duration<double, std::milli>(publish_start - last_publish_time_).count();
      ++trunk_publish_interval_count_;
      trunk_publish_interval_sum_ms_ += interval_ms;
      trunk_publish_interval_max_ms_ = std::max(trunk_publish_interval_max_ms_, interval_ms);
      if (interval_ms > 100.0) {
        ++trunk_publish_gap_over_100ms_count_;
      }
      if (interval_ms > 150.0) {
        ++trunk_publish_gap_over_150ms_count_;
      }
      if (interval_ms > 200.0) {
        ++trunk_publish_gap_over_200ms_count_;
      }
    }

    trunk_publisher_->publish(*output);
    const auto publish_end = Clock::now();
    ++lidar_points_publish_count_;
    ++latest_buffer_update_count_;
    last_publish_time_ = publish_end;
    last_output_stamp_sec_ = stamp_to_sec(output->header.stamp);
    last_output_subscription_count_ = trunk_publisher_->get_subscription_count();
    last_cloud_points_ = static_cast<std::size_t>(output->width) * output->height;
    last_cloud_bytes_ = output->data.size();
    last_trunk_publish_duration_ms_ =
      std::chrono::duration<double, std::milli>(publish_end - publish_start).count();
    last_fast_path_duration_ms_ =
      std::chrono::duration<double, std::milli>(publish_end - callback_start).count();

    if (!normalized_buffer) {
      normalized_buffer = std::make_shared<LatestNormalizedBuffer>();
      normalized_buffer->stamp = rclcpp::Time(output->header.stamp);
      normalized_buffer->frame_id = output->header.frame_id;
      normalized_buffer->source_bytes = output->data.size();
    }
    normalized_buffer->update_time = publish_end;
    std::shared_ptr<const LatestNormalizedBuffer> old_buffer;
    {
      const auto lock_start = Clock::now();
      std::lock_guard<std::mutex> lock(latest_cloud_mutex_);
      latest_buffer_lock_wait_ms_max_ = std::max(
        latest_buffer_lock_wait_ms_max_,
        std::chrono::duration<double, std::milli>(Clock::now() - lock_start).count());
      old_buffer = latest_normalized_buffer_;
      normalized_buffer->seq = latest_normalized_seq_ + 1U;
      latest_normalized_buffer_ = normalized_buffer;
      latest_normalized_time_ = publish_end;
      latest_normalized_seq_ = normalized_buffer->seq;
    }
    if (old_buffer && old_buffer.use_count() == 1U) {
      auto reusable = std::const_pointer_cast<LatestNormalizedBuffer>(old_buffer);
      std::lock_guard<std::mutex> lock(latest_cloud_mutex_);
      if (!reusable_normalized_buffer_) {
        reusable_normalized_buffer_ = reusable;
      }
    }

    if (!logged_ready_) {
      RCLCPP_INFO(
        node_.get_logger(),
        "pointcloud accel trunk ready profile=%s: %s -> %s frame=%s local_worker=%s scan_worker=%s local_compact=%s nav_compact=%s",
        accel_profile_.c_str(),
        input_topic_.c_str(),
        output_topic_.c_str(),
        output_frame_id_.c_str(),
        worker_local_enabled_ ? "true" : "false",
        worker_scan_enabled_ ? "true" : "false",
        local_compact_publisher_ ? local_output_topic_.c_str() : "(disabled)",
        nav_compact_publisher_ ? nav_output_topic_.c_str() : "(disabled)");
      logged_ready_ = true;
    }
  }

  void process_pointcloud2_internal(sensor_msgs::msg::PointCloud2::UniquePtr msg)
  {
    const auto callback_start = Clock::now();
    ++raw_input_count_;
    if (last_raw_callback_time_ != Clock::time_point{}) {
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
      auto normalized = build_latest_normalized_buffer(*output);
      publish_fast_path(std::move(output), callback_start, std::move(normalized));
      return;
    }
    if (!has_xyz_fields(*output)) {
      if (!warned_missing_xyz_) {
        RCLCPP_ERROR(node_.get_logger(), "cloud on %s does not expose x/y/z fields", input_topic_.c_str());
        warned_missing_xyz_ = true;
      }
      ++dropped_or_skipped_count_;
      last_fast_path_duration_ms_ =
        std::chrono::duration<double, std::milli>(Clock::now() - callback_start).count();
      return;
    }

    std::size_t x_offset = 0U;
    std::size_t y_offset = 0U;
    std::size_t z_offset = 0U;
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
      auto normalized = build_latest_normalized_buffer(*output);
      publish_fast_path(std::move(output), callback_start, std::move(normalized));
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
      for (std::size_t index = 0U; index < point_count; ++index) {
        auto * point = output->data.data() + index * output->point_step;
        auto * xyz = reinterpret_cast<float *>(point);
        const float raw_x = xyz[0];
        const float raw_y = xyz[1];
        xyz[0] = fast_path_raw_y_neg_raw_x ? raw_y : -raw_y;
        xyz[1] = -raw_x;
      }
      auto normalized = build_latest_normalized_buffer(*output);
      publish_fast_path(std::move(output), callback_start, std::move(normalized));
      return;
    }

    sensor_msgs::PointCloud2Iterator<float> iter_x(*output, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(*output, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(*output, "z");
    for (std::size_t index = 0U; index < point_count; ++index, ++iter_x, ++iter_y, ++iter_z) {
      const float x = *iter_x;
      const float y = *iter_y;
      const float z = *iter_z;
      *iter_x = rotation_[0] * x + rotation_[1] * y + rotation_[2] * z;
      *iter_y = rotation_[3] * x + rotation_[4] * y + rotation_[5] * z;
      *iter_z = rotation_[6] * x + rotation_[7] * y + rotation_[8] * z;
    }
    auto normalized = build_latest_normalized_buffer(*output);
    publish_fast_path(std::move(output), callback_start, std::move(normalized));
  }

  std::shared_ptr<const LatestNormalizedBuffer> latest_normalized_snapshot(
    Clock::time_point & stamp,
    std::uint64_t & seq,
    WorkerDiagnostics * diagnostics = nullptr) const
  {
    const auto lock_start = Clock::now();
    std::lock_guard<std::mutex> lock(latest_cloud_mutex_);
    if (diagnostics) {
      diagnostics->lock_wait_ms_max = std::max(
        diagnostics->lock_wait_ms_max,
        std::chrono::duration<double, std::milli>(Clock::now() - lock_start).count());
    }
    stamp = latest_normalized_time_;
    seq = latest_normalized_seq_;
    return latest_normalized_buffer_;
  }

  void local_worker_loop()
  {
    set_thread_name("pc_accel_local");
    auto next = Clock::now();
    const auto period = std::chrono::duration<double>(safe_rate_period_sec(local_worker_rate_hz_, 12.0));
    while (!stop_workers_.load() && rclcpp::ok()) {
      next += std::chrono::duration_cast<Clock::duration>(period);
      const auto begin = Clock::now();
      publish_local_compact_once();
      if (worker_local_enabled_) {
        process_local_obstacles_once();
      }
      const auto elapsed_ms = std::chrono::duration<double, std::milli>(Clock::now() - begin).count();
      local_worker_.last_processing_ms = elapsed_ms;
      local_worker_.processing_ms_max = std::max(local_worker_.processing_ms_max, elapsed_ms);
      local_worker_.processing_ms_sum += elapsed_ms;
      ++local_worker_.processing_ms_count;
      std::this_thread::sleep_until(next);
      if (Clock::now() > next + std::chrono::seconds(1)) {
        next = Clock::now();
      }
    }
  }

  void scan_worker_loop()
  {
    set_thread_name("pc_accel_scan");
    auto next = Clock::now();
    const auto period = std::chrono::duration<double>(safe_rate_period_sec(scan_worker_rate_hz_, 9.0));
    while (!stop_workers_.load() && rclcpp::ok()) {
      next += std::chrono::duration_cast<Clock::duration>(period);
      const auto begin = Clock::now();
      publish_nav_compact_once();
      if (worker_scan_enabled_) {
        process_scan_once();
      }
      const auto elapsed_ms = std::chrono::duration<double, std::milli>(Clock::now() - begin).count();
      scan_worker_.last_processing_ms = elapsed_ms;
      scan_worker_.processing_ms_max = std::max(scan_worker_.processing_ms_max, elapsed_ms);
      scan_worker_.processing_ms_sum += elapsed_ms;
      ++scan_worker_.processing_ms_count;
      std::this_thread::sleep_until(next);
      if (Clock::now() > next + std::chrono::seconds(1)) {
        next = Clock::now();
      }
    }
  }

  void prepare_xyzi_layout(sensor_msgs::msg::PointCloud2 & msg, const bool include_intensity)
  {
    const std::uint32_t expected_point_step = include_intensity ? 16U : 12U;
    const bool layout_matches =
      msg.point_step == expected_point_step &&
      msg.fields.size() == (include_intensity ? 4U : 3U) &&
      msg.fields.size() >= 3U &&
      msg.fields[0].name == "x" &&
      msg.fields[1].name == "y" &&
      msg.fields[2].name == "z" &&
      (!include_intensity || msg.fields[3].name == "intensity");
    if (layout_matches) {
      return;
    }
    msg.fields.clear();
    msg.fields.reserve(include_intensity ? 4U : 3U);
    msg.fields.push_back(make_float32_field("x", 0U));
    msg.fields.push_back(make_float32_field("y", 4U));
    msg.fields.push_back(make_float32_field("z", 8U));
    msg.point_step = 12U;
    if (include_intensity) {
      msg.fields.push_back(make_float32_field("intensity", 12U));
      msg.point_step = 16U;
    }
    msg.is_bigendian = false;
    msg.is_dense = false;
  }

  void fill_xyzi_cloud(
    const std::vector<PointXYZI> & points,
    const std_msgs::msg::Header & header,
    const bool include_intensity,
    sensor_msgs::msg::PointCloud2 & msg,
    WorkerDiagnostics & diagnostics)
  {
    prepare_xyzi_layout(msg, include_intensity);
    msg.header = header;
    msg.height = 1U;
    msg.width = static_cast<std::uint32_t>(points.size());
    msg.row_step = msg.point_step * msg.width;
    const auto required_bytes = static_cast<std::size_t>(msg.row_step);
    if (msg.data.capacity() < required_bytes) {
      ++diagnostics.allocation_count;
      diagnostics.reused_output_buffer = false;
    }
    msg.data.resize(required_bytes);
    for (std::size_t i = 0U; i < points.size(); ++i) {
      auto * data = msg.data.data() + i * msg.point_step;
      write_float32(data + 0U, points[i].x);
      write_float32(data + 4U, points[i].y);
      write_float32(data + 8U, points[i].z);
      if (include_intensity) {
        write_float32(data + 12U, points[i].intensity);
      }
    }
  }

  void fill_compact_cloud(
    const LatestNormalizedBuffer & buffer,
    const std::size_t stride,
    const std::string & fields,
    CompactDiagnostics & diagnostics,
    sensor_msgs::msg::PointCloud2 & msg)
  {
    const auto effective_stride = std::max<std::size_t>(stride, 1U);
    const bool include_intensity = fields == "xyzi";
    diagnostics.intensity_missing = include_intensity && !buffer.has_intensity;
    prepare_xyzi_layout(msg, include_intensity);
    msg.header = header_from_buffer(buffer);
    msg.height = 1U;
    msg.width = static_cast<std::uint32_t>(
      (buffer.points.size() + effective_stride - 1U) / effective_stride);
    msg.row_step = msg.point_step * msg.width;
    const auto required_bytes = static_cast<std::size_t>(msg.row_step);
    if (msg.data.capacity() < required_bytes) {
      ++diagnostics.allocation_count;
    }
    msg.data.resize(required_bytes);
    std::size_t out_index = 0U;
    for (std::size_t i = 0U; i < buffer.points.size(); i += effective_stride) {
      auto * data = msg.data.data() + out_index * msg.point_step;
      const auto & point = buffer.points[i];
      write_float32(data + 0U, point.x);
      write_float32(data + 4U, point.y);
      write_float32(data + 8U, point.z);
      if (include_intensity) {
        write_float32(data + 12U, point.intensity);
      }
      ++out_index;
    }
    diagnostics.last_points = static_cast<std::size_t>(msg.width) * msg.height;
    diagnostics.last_bytes = msg.data.size();
    diagnostics.bytes_per_point = msg.point_step;
  }

  void publish_local_compact_once()
  {
    if (!local_compact_publisher_) {
      return;
    }
    const auto sub_count = local_compact_publisher_->get_subscription_count();
    if (sub_count == 0U) {
      return;
    }
    const auto now = Clock::now();
    if (
      local_compact_last_publish_ != Clock::time_point{} &&
      std::chrono::duration<double>(now - local_compact_last_publish_).count() <
      safe_rate_period_sec(local_compact_max_rate_hz_, 12.0))
    {
      ++local_compact_.skip_busy_count;
      return;
    }
    Clock::time_point stamp;
    std::uint64_t seq = 0U;
    const auto buffer = latest_normalized_snapshot(stamp, seq);
    if (!buffer || seq == local_compact_last_seq_) {
      return;
    }
    fill_compact_cloud(*buffer, local_compact_stride_, local_compact_fields_, local_compact_, local_compact_msg_);
    local_compact_publisher_->publish(local_compact_msg_);
    ++local_compact_.publish_count;
    local_compact_last_publish_ = now;
    local_compact_last_seq_ = seq;
  }

  void publish_nav_compact_once()
  {
    if (!nav_compact_publisher_) {
      return;
    }
    const auto sub_count = nav_compact_publisher_->get_subscription_count();
    if (sub_count == 0U) {
      return;
    }
    const auto now = Clock::now();
    if (
      nav_compact_last_publish_ != Clock::time_point{} &&
      std::chrono::duration<double>(now - nav_compact_last_publish_).count() <
      safe_rate_period_sec(nav_compact_max_rate_hz_, 10.0))
    {
      ++nav_compact_.skip_busy_count;
      return;
    }
    Clock::time_point stamp;
    std::uint64_t seq = 0U;
    const auto buffer = latest_normalized_snapshot(stamp, seq);
    if (!buffer || seq == nav_compact_last_seq_) {
      return;
    }
    fill_compact_cloud(*buffer, nav_compact_stride_, nav_compact_fields_, nav_compact_, nav_compact_msg_);
    nav_compact_publisher_->publish(nav_compact_msg_);
    ++nav_compact_.publish_count;
    nav_compact_last_publish_ = now;
    nav_compact_last_seq_ = seq;
  }

  bool passes_angle_window(const double angle_rad) const
  {
    if (local_worker_min_angle_rad_ <= local_worker_max_angle_rad_) {
      return angle_rad >= local_worker_min_angle_rad_ && angle_rad <= local_worker_max_angle_rad_;
    }
    return angle_rad >= local_worker_min_angle_rad_ || angle_rad <= local_worker_max_angle_rad_;
  }

  bool in_self_mask(const PointXYZI & point) const
  {
    return point.x >= self_mask_min_x_ && point.x <= self_mask_max_x_ &&
           point.y >= self_mask_min_y_ && point.y <= self_mask_max_y_ &&
           point.z >= self_mask_min_z_ && point.z <= self_mask_max_z_;
  }

  bool passes_obstacle_filter(const PointXYZI & point) const
  {
    if (in_self_mask(point)) {
      return false;
    }
    const double range_xy = std::hypot(static_cast<double>(point.x), static_cast<double>(point.y));
    if (range_xy < local_worker_range_min_ || range_xy > local_worker_range_max_) {
      return false;
    }
    if (point.z < local_worker_min_z_ || point.z > local_worker_max_z_) {
      return false;
    }
    return passes_angle_window(std::atan2(static_cast<double>(point.y), static_cast<double>(point.x)));
  }

  bool passes_clearing_filter(const PointXYZI & point) const
  {
    if (in_self_mask(point)) {
      return false;
    }
    const double range_xy = std::hypot(static_cast<double>(point.x), static_cast<double>(point.y));
    return range_xy >= clearing_worker_range_min_ &&
           range_xy <= clearing_worker_range_max_ &&
           point.z >= clearing_worker_min_z_ &&
           point.z <= clearing_worker_max_z_;
  }

  std::pair<double, double> clearing_virtual_ray_angle_bounds() const
  {
    if (local_worker_min_angle_rad_ <= local_worker_max_angle_rad_) {
      return {local_worker_min_angle_rad_, local_worker_max_angle_rad_};
    }
    return {-kPi, kPi};
  }

  std::size_t clearing_virtual_ray_bin_count() const
  {
    const auto [min_angle, max_angle] = clearing_virtual_ray_angle_bounds();
    const auto span = std::max(max_angle - min_angle, clearing_worker_virtual_ray_angle_resolution_rad_);
    return static_cast<std::size_t>(std::ceil(span / clearing_worker_virtual_ray_angle_resolution_rad_)) + 1U;
  }

  void reset_clearing_virtual_ray_bins()
  {
    const auto [min_angle, max_angle] = clearing_virtual_ray_angle_bounds();
    const auto count = clearing_virtual_ray_bin_count();
    if (local_worker_clearing_bins_.size() != count) {
      local_worker_clearing_bins_.resize(count);
    }
    for (std::size_t index = 0U; index < local_worker_clearing_bins_.size(); ++index) {
      auto & bin = local_worker_clearing_bins_[index];
      bin.has_return = false;
      bin.range_xy = 0.0;
      bin.angle_rad =
        min_angle + (static_cast<double>(index) + 0.5) * clearing_worker_virtual_ray_angle_resolution_rad_;
      if (bin.angle_rad > max_angle) {
        bin.angle_rad = max_angle;
      }
    }
  }

  void update_clearing_virtual_ray_bin(const PointXYZI & point)
  {
    if (local_worker_clearing_bins_.empty()) {
      return;
    }
    const auto [min_angle, max_angle] = clearing_virtual_ray_angle_bounds();
    const auto angle = std::atan2(static_cast<double>(point.y), static_cast<double>(point.x));
    if (angle < min_angle || angle > max_angle) {
      return;
    }
    const auto range_xy = std::hypot(static_cast<double>(point.x), static_cast<double>(point.y));
    const auto raw_index = static_cast<long>(
      std::floor((angle - min_angle) / clearing_worker_virtual_ray_angle_resolution_rad_));
    const auto clamped_index = std::clamp<long>(
      raw_index, 0, static_cast<long>(local_worker_clearing_bins_.size() - 1U));
    auto & bin = local_worker_clearing_bins_[static_cast<std::size_t>(clamped_index)];
    if (!bin.has_return || range_xy > bin.range_xy) {
      bin.has_return = true;
      bin.range_xy = range_xy;
      bin.angle_rad = angle;
    }
  }

  std::size_t clearing_virtual_ray_reserve() const
  {
    if (!clearing_worker_virtual_rays_enabled_) {
      return 0U;
    }
    const auto raw_count =
      clearing_virtual_ray_bin_count() *
      clearing_worker_virtual_ray_ranges_.size() *
      clearing_worker_virtual_ray_endpoint_z_values_.size();
    return std::min(clearing_worker_max_points_, raw_count);
  }

  void build_virtual_clearing_points()
  {
    local_worker_clearing_points_.clear();
    for (const auto & bin : local_worker_clearing_bins_) {
      const auto max_range_xy = std::clamp(
        bin.has_return ? bin.range_xy : clearing_worker_virtual_ray_range_,
        clearing_worker_range_min_,
        clearing_worker_virtual_ray_range_);
      for (const auto range_xy : clearing_worker_virtual_ray_ranges_) {
        if (range_xy < clearing_worker_range_min_ || range_xy > max_range_xy + 1.0e-3) {
          continue;
        }
        const auto x = static_cast<float>(std::cos(bin.angle_rad) * range_xy);
        const auto y = static_cast<float>(std::sin(bin.angle_rad) * range_xy);
        for (const auto z_value : clearing_worker_virtual_ray_endpoint_z_values_) {
          PointXYZI endpoint;
          endpoint.x = x;
          endpoint.y = y;
          endpoint.z = static_cast<float>(z_value);
          endpoint.intensity = 0.0F;
          if (in_self_mask(endpoint)) {
            continue;
          }
          local_worker_clearing_points_.push_back(endpoint);
          if (local_worker_clearing_points_.size() >= clearing_worker_max_points_) {
            return;
          }
        }
      }
    }
  }

  void sanitize_clearing_virtual_rays()
  {
    clearing_worker_virtual_ray_range_ =
      std::max(clearing_worker_virtual_ray_range_, clearing_worker_range_min_);

    std::vector<double> sanitized_ranges;
    sanitized_ranges.reserve(clearing_worker_virtual_ray_ranges_.size() + 1U);
    for (const auto range_xy : clearing_worker_virtual_ray_ranges_) {
      if (!std::isfinite(range_xy)) {
        continue;
      }
      if (range_xy < clearing_worker_range_min_ || range_xy > clearing_worker_virtual_ray_range_) {
        continue;
      }
      sanitized_ranges.push_back(range_xy);
    }
    sanitized_ranges.push_back(clearing_worker_virtual_ray_range_);
    std::sort(sanitized_ranges.begin(), sanitized_ranges.end());
    sanitized_ranges.erase(
      std::unique(
        sanitized_ranges.begin(),
        sanitized_ranges.end(),
        [](const double lhs, const double rhs) { return std::abs(lhs - rhs) < 1.0e-3; }),
      sanitized_ranges.end());
    clearing_worker_virtual_ray_ranges_ = sanitized_ranges;

    std::vector<double> sanitized_z;
    sanitized_z.reserve(clearing_worker_virtual_ray_endpoint_z_values_.size());
    for (const auto z_value : clearing_worker_virtual_ray_endpoint_z_values_) {
      if (!std::isfinite(z_value)) {
        continue;
      }
      if (z_value < clearing_worker_min_z_ || z_value > clearing_worker_max_z_) {
        continue;
      }
      sanitized_z.push_back(z_value);
    }
    std::sort(sanitized_z.begin(), sanitized_z.end());
    sanitized_z.erase(
      std::unique(
        sanitized_z.begin(),
        sanitized_z.end(),
        [](const double lhs, const double rhs) { return std::abs(lhs - rhs) < 1.0e-3; }),
      sanitized_z.end());
    if (sanitized_z.empty()) {
      sanitized_z.push_back(0.5 * (clearing_worker_min_z_ + clearing_worker_max_z_));
    }
    clearing_worker_virtual_ray_endpoint_z_values_ = sanitized_z;
  }

  void process_local_obstacles_once()
  {
    ++local_worker_.tick_count;
    local_worker_.reused_output_buffer = true;
    if (obstacle_publisher_->get_subscription_count() == 0U) {
      return;
    }
    Clock::time_point stamp;
    std::uint64_t seq = 0U;
    const auto buffer = latest_normalized_snapshot(stamp, seq, &local_worker_);
    if (!buffer || seq == local_worker_last_seq_) {
      return;
    }
    local_worker_.start_source_age_ms = age_ms_from_ros_stamp(node_.now().seconds(), buffer->stamp);
    Transform3x4 transform;
    if (!lookup_transform(local_worker_output_frame_id_, buffer->frame_id, transform)) {
      return;
    }
    const std::size_t point_count = buffer->points.size();
    const auto obstacle_reserve =
      std::min(local_worker_max_points_, point_count / local_worker_point_stride_ + 1U);
    if (local_worker_obstacle_points_.capacity() < obstacle_reserve) {
      ++local_worker_.allocation_count;
      local_worker_obstacle_points_.reserve(obstacle_reserve);
      local_worker_.reused_output_buffer = false;
    }
    local_worker_obstacle_points_.clear();
    const bool publish_clearing =
      clearing_publisher_ &&
      clearing_publisher_->get_subscription_count() > 0U &&
      (local_worker_.tick_count % std::max<std::uint64_t>(
        1U, static_cast<std::uint64_t>(std::round(local_worker_rate_hz_ / clearing_worker_rate_hz_))) == 0U);
    if (publish_clearing) {
      const auto clearing_reserve = clearing_worker_virtual_rays_enabled_ ?
        clearing_virtual_ray_reserve() :
        std::min(clearing_worker_max_points_, point_count / clearing_worker_point_stride_ + 1U);
      if (local_worker_clearing_points_.capacity() < clearing_reserve) {
        ++local_worker_.allocation_count;
        local_worker_clearing_points_.reserve(clearing_reserve);
        local_worker_.reused_output_buffer = false;
      }
      local_worker_clearing_points_.clear();
      if (clearing_worker_virtual_rays_enabled_) {
        reset_clearing_virtual_ray_bins();
      }
    }
    for (std::size_t i = 0U; i < point_count; ++i) {
      const auto point = transform_point(transform, buffer->points[i]);
      if (publish_clearing && i % clearing_worker_point_stride_ == 0U && passes_clearing_filter(point)) {
        if (clearing_worker_virtual_rays_enabled_) {
          update_clearing_virtual_ray_bin(point);
        } else if (local_worker_clearing_points_.size() < clearing_worker_max_points_) {
          local_worker_clearing_points_.push_back(point);
        }
      }
      if (i % local_worker_point_stride_ != 0U) {
        continue;
      }
      if (passes_obstacle_filter(point)) {
        local_worker_obstacle_points_.push_back(point);
        if (local_worker_obstacle_points_.size() >= local_worker_max_points_) {
          break;
        }
      }
    }
    auto header = header_from_buffer(*buffer);
    header.frame_id = local_worker_output_frame_id_;
    fill_xyzi_cloud(local_worker_obstacle_points_, header, true, obstacle_output_msg_, local_worker_);
    local_worker_.last_obstacle_output_bytes = obstacle_output_msg_.data.size();
    const auto obstacle_ready = Clock::now();
    local_worker_.obstacle_output_source_age_ms = age_ms_from_ros_stamp(node_.now().seconds(), buffer->stamp);
    local_worker_.obstacle_output_publish_delay_ms =
      std::chrono::duration<double, std::milli>(Clock::now() - obstacle_ready).count();
    obstacle_publisher_->publish(obstacle_output_msg_);
    local_worker_.obstacle_output_header_age_ms =
      age_ms_from_stamp_sec(node_.now().seconds(), stamp_to_sec(obstacle_output_msg_.header.stamp));
    if (local_worker_.obstacle_output_header_age_ms > 100.0) {
      ++tf_drop_suspect_obstacle_header_age_over_100ms_count_;
    }
    if (local_worker_.obstacle_output_header_age_ms > 200.0) {
      ++tf_drop_suspect_obstacle_header_age_over_200ms_count_;
    }
    ++local_worker_.processed_count;
    ++local_worker_.obstacle_publish_count;
    if (publish_clearing) {
      if (clearing_worker_virtual_rays_enabled_) {
        build_virtual_clearing_points();
      }
      fill_xyzi_cloud(local_worker_clearing_points_, header, true, clearing_output_msg_, local_worker_);
      local_worker_.last_clearing_output_bytes = clearing_output_msg_.data.size();
      const auto clearing_ready = Clock::now();
      local_worker_.clearing_output_source_age_ms = age_ms_from_ros_stamp(node_.now().seconds(), buffer->stamp);
      local_worker_.clearing_output_publish_delay_ms =
        std::chrono::duration<double, std::milli>(Clock::now() - clearing_ready).count();
      clearing_publisher_->publish(clearing_output_msg_);
      local_worker_.clearing_output_header_age_ms =
        age_ms_from_stamp_sec(node_.now().seconds(), stamp_to_sec(clearing_output_msg_.header.stamp));
      ++local_worker_.clearing_publish_count;
    }
    local_worker_.end_source_age_ms = age_ms_from_ros_stamp(node_.now().seconds(), buffer->stamp);
    local_worker_last_seq_ = seq;
  }

  void process_scan_once()
  {
    ++scan_worker_.tick_count;
    scan_worker_.reused_output_buffer = true;
    if (!scan_publisher_ || scan_publisher_->get_subscription_count() == 0U) {
      return;
    }
    Clock::time_point stamp;
    std::uint64_t seq = 0U;
    const auto buffer = latest_normalized_snapshot(stamp, seq, &scan_worker_);
    if (!buffer || seq == scan_worker_last_seq_) {
      return;
    }
    scan_worker_.start_source_age_ms = age_ms_from_ros_stamp(node_.now().seconds(), buffer->stamp);
    Transform3x4 transform;
    if (!lookup_transform(scan_worker_frame_id_, buffer->frame_id, transform)) {
      return;
    }
    const auto bin_count = static_cast<std::size_t>(
      std::ceil((scan_worker_angle_max_ - scan_worker_angle_min_) / scan_worker_angle_increment_));
    if (bin_count == 0U || bin_count > 10000U) {
      return;
    }
    scan_worker_.scan_bins_reused = scan_msg_.ranges.capacity() >= bin_count;
    if (!scan_worker_.scan_bins_reused) {
      ++scan_worker_.allocation_count;
      scan_worker_.reused_output_buffer = false;
    }
    const float default_range = scan_worker_use_inf_ ?
      std::numeric_limits<float>::infinity() :
      static_cast<float>(scan_worker_range_max_ + scan_worker_inf_epsilon_);
    scan_msg_.ranges.assign(bin_count, default_range);
    const std::size_t point_count = buffer->points.size();
    for (std::size_t i = 0U; i < point_count; ++i) {
      const auto point = transform_point(transform, buffer->points[i]);
      if (point.z < scan_worker_min_height_ || point.z > scan_worker_max_height_) {
        continue;
      }
      const double range = std::hypot(static_cast<double>(point.x), static_cast<double>(point.y));
      if (range < scan_worker_range_min_ || range > scan_worker_range_max_) {
        continue;
      }
      const double angle = std::atan2(static_cast<double>(point.y), static_cast<double>(point.x));
      if (angle < scan_worker_angle_min_ || angle > scan_worker_angle_max_) {
        continue;
      }
      const auto index = static_cast<std::size_t>((angle - scan_worker_angle_min_) / scan_worker_angle_increment_);
      if (index < scan_msg_.ranges.size() && range < scan_msg_.ranges[index]) {
        scan_msg_.ranges[index] = static_cast<float>(range);
      }
    }
    scan_msg_.header = header_from_buffer(*buffer);
    scan_msg_.header.frame_id = scan_worker_frame_id_;
    scan_msg_.angle_min = static_cast<float>(scan_worker_angle_min_);
    scan_msg_.angle_max =
      static_cast<float>(scan_worker_angle_min_ + scan_worker_angle_increment_ * (bin_count - 1U));
    scan_msg_.angle_increment = static_cast<float>(scan_worker_angle_increment_);
    scan_msg_.time_increment = 0.0F;
    scan_msg_.scan_time = static_cast<float>(1.0 / scan_worker_rate_hz_);
    scan_msg_.range_min = static_cast<float>(scan_worker_range_min_);
    scan_msg_.range_max = static_cast<float>(scan_worker_range_max_);
    scan_worker_.last_scan_output_ranges = scan_msg_.ranges.size();
    scan_worker_.last_scan_output_bytes = scan_msg_.ranges.size() * sizeof(float);
    scan_publisher_->publish(scan_msg_);
    scan_worker_.scan_output_header_age_ms =
      age_ms_from_stamp_sec(node_.now().seconds(), stamp_to_sec(scan_msg_.header.stamp));
    scan_worker_.scan_output_source_age_ms = age_ms_from_ros_stamp(node_.now().seconds(), buffer->stamp);
    scan_worker_.end_source_age_ms = scan_worker_.scan_output_source_age_ms;
    ++scan_worker_.processed_count;
    ++scan_worker_.scan_publish_count;
    scan_worker_last_seq_ = seq;
  }

  void publish_status()
  {
    if (!status_publisher_) {
      return;
    }
    const auto now_steady = Clock::now();
    const double elapsed_sec = std::max(
      std::chrono::duration<double>(now_steady - previous_status_time_).count(), 1.0e-3);
    const auto raw_delta = raw_input_count_ - previous_raw_input_count_;
    const auto publish_delta = lidar_points_publish_count_ - previous_lidar_points_publish_count_;
    const auto latest_delta = latest_buffer_update_count_ - previous_latest_buffer_update_count_;
    const auto local_compact_delta = local_compact_.publish_count - local_compact_.previous_publish_count;
    const auto nav_compact_delta = nav_compact_.publish_count - nav_compact_.previous_publish_count;
    const auto local_tick_delta = local_worker_.tick_count - local_worker_.previous_tick_count;
    const auto local_processed_delta = local_worker_.processed_count - local_worker_.previous_processed_count;
    const auto obstacle_delta =
      local_worker_.obstacle_publish_count - local_worker_.previous_obstacle_publish_count;
    const auto clearing_delta =
      local_worker_.clearing_publish_count - local_worker_.previous_clearing_publish_count;
    const auto scan_tick_delta = scan_worker_.tick_count - scan_worker_.previous_tick_count;
    const auto scan_processed_delta = scan_worker_.processed_count - scan_worker_.previous_processed_count;
    const auto scan_delta = scan_worker_.scan_publish_count - scan_worker_.previous_scan_publish_count;
    const double raw_interarrival_ms_avg = raw_interarrival_count_ > 0U ?
      raw_interarrival_sum_ms_ / static_cast<double>(raw_interarrival_count_) : -1.0;
    const double trunk_publish_interval_ms_avg = trunk_publish_interval_count_ > 0U ?
      trunk_publish_interval_sum_ms_ / static_cast<double>(trunk_publish_interval_count_) : -1.0;
    const double raw_age_ms = last_raw_callback_time_ == Clock::time_point{} ?
      -1.0 : std::chrono::duration<double, std::milli>(now_steady - last_raw_callback_time_).count();
    const double publish_age_ms = last_publish_time_ == Clock::time_point{} ?
      -1.0 : std::chrono::duration<double, std::milli>(now_steady - last_publish_time_).count();
    Clock::time_point latest_stamp;
    std::uint64_t latest_seq = 0U;
    const auto latest = latest_normalized_snapshot(latest_stamp, latest_seq);
    const double latest_age_ms = latest_stamp == Clock::time_point{} ?
      -1.0 : std::chrono::duration<double, std::milli>(now_steady - latest_stamp).count();
    const double local_processing_ms_avg = local_worker_.processing_ms_count > 0U ?
      local_worker_.processing_ms_sum / static_cast<double>(local_worker_.processing_ms_count) : -1.0;
    const double scan_processing_ms_avg = scan_worker_.processing_ms_count > 0U ?
      scan_worker_.processing_ms_sum / static_cast<double>(scan_worker_.processing_ms_count) : -1.0;
    const double status_ros_now_sec = node_.now().seconds();
    const double raw_stamp_age_ms = last_raw_stamp_sec_ <= 0.0 ?
      -1.0 : (status_ros_now_sec - last_raw_stamp_sec_) * 1000.0;
    const double publish_stamp_age_ms = last_output_stamp_sec_ <= 0.0 ?
      -1.0 : (status_ros_now_sec - last_output_stamp_sec_) * 1000.0;
    const double latest_internal_buffer_stamp_age_ms = latest ?
      age_ms_from_ros_stamp(status_ros_now_sec, latest->stamp) : -1.0;
    const double latest_internal_buffer_update_age_ms =
      latest && latest->update_time != Clock::time_point{} ?
      std::chrono::duration<double, std::milli>(now_steady - latest->update_time).count() : -1.0;
    const double uptime_sec = std::chrono::duration<double>(now_steady - start_time_).count();

    std::ostringstream stream;
    stream << std::fixed << std::setprecision(3)
           << "input_topic=" << input_topic_
           << " output_topic=" << output_topic_
           << " raw_input_hz=" << static_cast<double>(raw_delta) / elapsed_sec
           << " lidar_points_publish_hz=" << static_cast<double>(publish_delta) / elapsed_sec
           << " fast_path_raw_input_hz=" << static_cast<double>(raw_delta) / elapsed_sec
           << " fast_path_lidar_points_publish_hz=" << static_cast<double>(publish_delta) / elapsed_sec
           << " fast_path_duration_ms=" << last_fast_path_duration_ms_
           << " latest_buffer_update_hz=" << static_cast<double>(latest_delta) / elapsed_sec
           << " latest_buffer_points=" << (latest ? latest->points.size() : 0U)
           << " latest_buffer_bytes=" << (latest ? latest->points.size() * sizeof(NormalizedPointView) : 0U)
           << " latest_buffer_age_ms=" << latest_age_ms
           << " internal_zero_copy_profile=true"
           << " latest_internal_buffer_points=" << (latest ? latest->points.size() : 0U)
           << " latest_internal_buffer_bytes=" << (latest ? latest->points.size() * sizeof(NormalizedPointView) : 0U)
           << " latest_internal_buffer_source_bytes=" << (latest ? latest->source_bytes : 0U)
           << " latest_internal_buffer_age_ms=" << latest_age_ms
           << " latest_internal_buffer_stamp_age_ms=" << latest_internal_buffer_stamp_age_ms
           << " latest_internal_buffer_update_age_ms=" << latest_internal_buffer_update_age_ms
           << " latest_internal_buffer_seq=" << latest_seq
           << " accel_ingress_profile=" << options_.accel_ingress_profile
           << " input_path=" << options_.input_path
           << " vendor_raw_ros_hop_required=" <<
             (options_.vendor_raw_ros_hop_required ? "true" : "false")
           << " vendor_raw_debug_publish_enabled=" <<
             (options_.vendor_raw_debug_publish_enabled ? "true" : "false")
           << " driver_integrated_process=" <<
             (options_.driver_integrated_process ? "true" : "false")
           << " accel_core_process_pointcloud2_count=" << accel_core_process_pointcloud2_count_
           << " accel_core_process_decoded_view_count=" << accel_core_process_decoded_view_count_
           << " driver_integrated_unavailable_reason=" <<
             options_.driver_integrated_unavailable_reason
           << " normalized_buffer_allocation_count=" << normalized_buffer_allocation_count_
           << " latest_buffer_lock_wait_ms_max=" << latest_buffer_lock_wait_ms_max_
           << " trunk_copy_count_per_frame=0"
           << " raw_header_age_ms=" << raw_stamp_age_ms
           << " last_raw_age_ms=" << raw_age_ms
           << " last_raw_stamp_age_ms=" << raw_stamp_age_ms
           << " last_publish_age_ms=" << publish_age_ms
           << " last_publish_stamp_age_ms=" << publish_stamp_age_ms
           << " last_publish_duration_ms=" << last_fast_path_duration_ms_
           << " last_trunk_publish_duration_ms=" << last_trunk_publish_duration_ms_
           << " last_branch_publish_duration_ms=0.000"
           << " last_total_publish_outputs_duration_ms=" << last_fast_path_duration_ms_
           << " raw_interarrival_ms_avg=" << raw_interarrival_ms_avg
           << " raw_interarrival_ms_max=" << raw_interarrival_max_ms_
           << " lidar_points_publish_interval_ms_avg=" << trunk_publish_interval_ms_avg
           << " lidar_points_publish_interval_ms_max=" << trunk_publish_interval_max_ms_
           << " trunk_publish_gap_over_100ms_count=" << trunk_publish_gap_over_100ms_count_
           << " trunk_publish_gap_over_150ms_count=" << trunk_publish_gap_over_150ms_count_
           << " trunk_publish_gap_over_200ms_count=" << trunk_publish_gap_over_200ms_count_
           << " trunk_output_subscription_count=" << last_output_subscription_count_
           << " output_subscription_count=" << last_output_subscription_count_
           << " raw_callback_count=" << raw_input_count_
           << " trunk_publish_count=" << lidar_points_publish_count_
           << " last_cloud_points=" << last_cloud_points_
           << " last_cloud_bytes=" << last_cloud_bytes_
           << " accel_profile=" << accel_profile_
           << " worker_local_enabled=" << (worker_local_enabled_ ? "true" : "false")
           << " worker_scan_enabled=" << (worker_scan_enabled_ ? "true" : "false")
           << " local_worker_enabled=" << (worker_local_enabled_ ? "true" : "false")
           << " scan_worker_enabled=" << (worker_scan_enabled_ ? "true" : "false")
           << " nav_branch_enabled=" << (nav_compact_publisher_ ? "true" : "false")
           << " local_branch_enabled=" << (local_compact_publisher_ ? "true" : "false")
           << " nav_branch_publish_hz=" << static_cast<double>(nav_compact_delta) / elapsed_sec
           << " nav_branch_last_points=" << nav_compact_.last_points
           << " nav_branch_last_bytes=" << nav_compact_.last_bytes
           << " nav_output_stride=" << nav_compact_stride_
           << " nav_output_publish_every_n=1"
           << " local_branch_publish_hz=" << static_cast<double>(local_compact_delta) / elapsed_sec
           << " local_branch_last_points=" << local_compact_.last_points
           << " local_branch_last_bytes=" << local_compact_.last_bytes
           << " local_output_stride=" << local_compact_stride_
           << " local_output_publish_every_n=1"
           << " local_compact_last_points=" << local_compact_.last_points
           << " local_compact_last_bytes=" << local_compact_.last_bytes
           << " local_compact_bytes_per_point=" << local_compact_.bytes_per_point
           << " local_compact_publish_hz=" << static_cast<double>(local_compact_delta) / elapsed_sec
           << " local_compact_skip_busy_count=" << local_compact_.skip_busy_count
           << " local_compact_intensity_missing=" << (local_compact_.intensity_missing ? "true" : "false")
           << " local_compact_allocation_count=" << local_compact_.allocation_count
           << " nav_compact_last_points=" << nav_compact_.last_points
           << " nav_compact_last_bytes=" << nav_compact_.last_bytes
           << " nav_compact_bytes_per_point=" << nav_compact_.bytes_per_point
           << " nav_compact_publish_hz=" << static_cast<double>(nav_compact_delta) / elapsed_sec
           << " nav_compact_skip_busy_count=" << nav_compact_.skip_busy_count
           << " nav_compact_intensity_missing=" << (nav_compact_.intensity_missing ? "true" : "false")
           << " nav_compact_allocation_count=" << nav_compact_.allocation_count
           << " local_worker_tick_hz=" << static_cast<double>(local_tick_delta) / elapsed_sec
           << " local_worker_processed_hz=" << static_cast<double>(local_processed_delta) / elapsed_sec
           << " local_worker_obstacle_publish_hz=" << static_cast<double>(obstacle_delta) / elapsed_sec
           << " local_worker_clearing_publish_hz=" << static_cast<double>(clearing_delta) / elapsed_sec
           << " local_worker_last_processing_ms=" << local_worker_.last_processing_ms
           << " local_worker_processing_ms_avg=" << local_processing_ms_avg
           << " local_worker_processing_ms_max=" << local_worker_.processing_ms_max
           << " local_worker_start_source_age_ms=" << local_worker_.start_source_age_ms
           << " local_worker_end_source_age_ms=" << local_worker_.end_source_age_ms
           << " local_worker_skip_busy_count=" << local_worker_.skip_busy_count
           << " local_worker_full_cloud_copy_count=" << local_worker_.full_cloud_copy_count
           << " local_worker_intermediate_pointcloud_build_count=" <<
             local_worker_.intermediate_pointcloud_build_count
           << " local_worker_allocation_count=" << local_worker_.allocation_count
           << " local_worker_reused_buffer=" << (local_worker_.reused_output_buffer ? "true" : "false")
           << " local_worker_lock_wait_ms_max=" << local_worker_.lock_wait_ms_max
           << " obstacle_output_last_bytes=" << local_worker_.last_obstacle_output_bytes
           << " obstacle_output_header_age_ms=" << local_worker_.obstacle_output_header_age_ms
           << " obstacle_output_source_age_ms=" << local_worker_.obstacle_output_source_age_ms
           << " obstacle_output_publish_delay_ms=" << local_worker_.obstacle_output_publish_delay_ms
           << " obstacle_output_frame_id=" << local_worker_output_frame_id_
           << " obstacle_output_header_stamp_source=source_stamp"
           << " clearing_output_last_bytes=" << local_worker_.last_clearing_output_bytes
           << " clearing_output_header_age_ms=" << local_worker_.clearing_output_header_age_ms
           << " clearing_output_source_age_ms=" << local_worker_.clearing_output_source_age_ms
           << " clearing_output_publish_delay_ms=" << local_worker_.clearing_output_publish_delay_ms
           << " clearing_output_frame_id=" << local_worker_output_frame_id_
           << " clearing_output_header_stamp_source=source_stamp"
           << " scan_worker_tick_hz=" << static_cast<double>(scan_tick_delta) / elapsed_sec
           << " scan_worker_processed_hz=" << static_cast<double>(scan_processed_delta) / elapsed_sec
           << " scan_worker_scan_publish_hz=" << static_cast<double>(scan_delta) / elapsed_sec
           << " scan_worker_flatscan_publish_hz=0.000"
           << " scan_worker_last_processing_ms=" << scan_worker_.last_processing_ms
           << " scan_worker_processing_ms_avg=" << scan_processing_ms_avg
           << " scan_worker_processing_ms_max=" << scan_worker_.processing_ms_max
           << " scan_worker_start_source_age_ms=" << scan_worker_.start_source_age_ms
           << " scan_worker_end_source_age_ms=" << scan_worker_.end_source_age_ms
           << " scan_worker_skip_busy_count=" << scan_worker_.skip_busy_count
           << " scan_worker_full_cloud_copy_count=" << scan_worker_.full_cloud_copy_count
           << " scan_worker_intermediate_pointcloud_build_count=" <<
             scan_worker_.intermediate_pointcloud_build_count
           << " scan_worker_allocation_count=" << scan_worker_.allocation_count
           << " scan_worker_reused_buffer=" << (scan_worker_.reused_output_buffer ? "true" : "false")
           << " scan_worker_scan_bins_reused=" << (scan_worker_.scan_bins_reused ? "true" : "false")
           << " scan_worker_lock_wait_ms_max=" << scan_worker_.lock_wait_ms_max
           << " scan_output_last_ranges=" << scan_worker_.last_scan_output_ranges
           << " scan_output_last_bytes=" << scan_worker_.last_scan_output_bytes
           << " scan_output_header_age_ms=" << scan_worker_.scan_output_header_age_ms
           << " scan_output_source_age_ms=" << scan_worker_.scan_output_source_age_ms
           << " scan_output_frame_id=" << scan_worker_frame_id_
           << " tf_drop_suspect_obstacle_header_age_over_100ms_count=" <<
             tf_drop_suspect_obstacle_header_age_over_100ms_count_
           << " tf_drop_suspect_obstacle_header_age_over_200ms_count=" <<
             tf_drop_suspect_obstacle_header_age_over_200ms_count_
           << " dropped_or_skipped_count=" << dropped_or_skipped_count_
           << " node_uptime_sec=" << uptime_sec;

    std_msgs::msg::String msg;
    msg.data = stream.str();
    status_publisher_->publish(msg);
    if (accel_status_publisher_) {
      accel_status_publisher_->publish(msg);
    }

    previous_status_time_ = now_steady;
    previous_raw_input_count_ = raw_input_count_;
    previous_lidar_points_publish_count_ = lidar_points_publish_count_;
    previous_latest_buffer_update_count_ = latest_buffer_update_count_;
    local_compact_.previous_publish_count = local_compact_.publish_count;
    nav_compact_.previous_publish_count = nav_compact_.publish_count;
    local_worker_.previous_tick_count = local_worker_.tick_count;
    local_worker_.previous_processed_count = local_worker_.processed_count;
    local_worker_.previous_obstacle_publish_count = local_worker_.obstacle_publish_count;
    local_worker_.previous_clearing_publish_count = local_worker_.clearing_publish_count;
    scan_worker_.previous_tick_count = scan_worker_.tick_count;
    scan_worker_.previous_processed_count = scan_worker_.processed_count;
    scan_worker_.previous_scan_publish_count = scan_worker_.scan_publish_count;
    raw_interarrival_count_ = 0U;
    raw_interarrival_sum_ms_ = 0.0;
    raw_interarrival_max_ms_ = 0.0;
    trunk_publish_interval_count_ = 0U;
    trunk_publish_interval_sum_ms_ = 0.0;
    trunk_publish_interval_max_ms_ = 0.0;
    trunk_publish_gap_over_100ms_count_ = 0U;
    trunk_publish_gap_over_150ms_count_ = 0U;
    trunk_publish_gap_over_200ms_count_ = 0U;
    latest_buffer_lock_wait_ms_max_ = 0.0;
    local_worker_.processing_ms_max = 0.0;
    local_worker_.processing_ms_sum = 0.0;
    local_worker_.processing_ms_count = 0U;
    local_worker_.lock_wait_ms_max = 0.0;
    scan_worker_.processing_ms_max = 0.0;
    scan_worker_.processing_ms_sum = 0.0;
    scan_worker_.processing_ms_count = 0U;
    scan_worker_.lock_wait_ms_max = 0.0;
  }

  rclcpp::Node & node_;
  PointCloudAccelCoreOptions options_;
  std::string input_topic_;
  std::string output_topic_;
  std::string output_frame_id_;
  std::string accel_profile_;
  std::string status_topic_;
  std::string accel_status_topic_;
  double status_publish_period_sec_{1.0};
  std::size_t input_qos_depth_{1U};
  rmw_qos_reliability_policy_t input_reliability_{RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT};
  std::size_t output_qos_depth_{1U};
  std::array<float, 9> rotation_{};

  std::string local_output_topic_;
  std::string local_compact_fields_{"xyzi"};
  bool local_compact_enabled_{true};
  std::size_t local_compact_stride_{4U};
  double local_compact_max_rate_hz_{12.0};
  std::size_t local_output_qos_depth_{1U};
  std::string nav_output_topic_;
  std::string nav_compact_fields_{"xyzi"};
  bool nav_compact_enabled_{true};
  std::size_t nav_compact_stride_{4U};
  double nav_compact_max_rate_hz_{10.0};
  std::size_t nav_output_qos_depth_{1U};

  bool worker_local_enabled_{true};
  bool worker_scan_enabled_{true};
  std::string obstacle_output_topic_;
  std::string clearing_output_topic_;
  std::string local_worker_output_frame_id_;
  double local_worker_rate_hz_{12.0};
  double local_worker_range_min_{0.5};
  double local_worker_range_max_{5.5};
  double local_worker_min_z_{0.40};
  double local_worker_max_z_{1.30};
  double local_worker_min_angle_rad_{-110.0 * kPi / 180.0};
  double local_worker_max_angle_rad_{110.0 * kPi / 180.0};
  std::size_t local_worker_point_stride_{1U};
  std::size_t local_worker_max_points_{12000U};
  double self_mask_min_x_{-0.5};
  double self_mask_max_x_{0.45};
  double self_mask_min_y_{-0.4};
  double self_mask_max_y_{0.4};
  double self_mask_min_z_{-0.2};
  double self_mask_max_z_{1.4};
  double clearing_worker_rate_hz_{4.0};
  double clearing_worker_range_min_{0.10};
  double clearing_worker_range_max_{8.0};
  double clearing_worker_min_z_{-0.30};
  double clearing_worker_max_z_{1.40};
  std::size_t clearing_worker_point_stride_{2U};
  std::size_t clearing_worker_max_points_{15000U};
  bool clearing_worker_virtual_rays_enabled_{true};
  double clearing_worker_virtual_ray_angle_resolution_rad_{kPi / 180.0};
  double clearing_worker_virtual_ray_range_{8.0};
  std::vector<double> clearing_worker_virtual_ray_ranges_{0.50, 1.00, 2.00, 3.50, 5.50, 8.00};
  std::vector<double> clearing_worker_virtual_ray_endpoint_z_values_{
    -0.10, 0.05, 0.20, 0.40, 0.60, 0.85, 1.10, 1.30};

  std::string scan_output_topic_;
  std::string scan_worker_frame_id_;
  double scan_worker_rate_hz_{9.0};
  double scan_worker_min_height_{-0.75};
  double scan_worker_max_height_{0.35};
  double scan_worker_angle_min_{-kPi};
  double scan_worker_angle_max_{kPi};
  double scan_worker_angle_increment_{0.004363323129985824};
  double scan_worker_range_min_{0.25};
  double scan_worker_range_max_{40.0};
  bool scan_worker_use_inf_{true};
  double scan_worker_inf_epsilon_{1.0};

  bool logged_ready_;
  bool warned_missing_xyz_;
  std::atomic<bool> stop_workers_{false};
  mutable std::mutex latest_cloud_mutex_;
  std::shared_ptr<const LatestNormalizedBuffer> latest_normalized_buffer_;
  std::shared_ptr<LatestNormalizedBuffer> reusable_normalized_buffer_;
  Clock::time_point latest_normalized_time_{};
  std::uint64_t latest_normalized_seq_{0U};
  std::uint64_t normalized_buffer_allocation_count_{0U};
  double latest_buffer_lock_wait_ms_max_{0.0};
  std::uint64_t local_worker_last_seq_{0U};
  std::uint64_t scan_worker_last_seq_{0U};
  std::uint64_t local_compact_last_seq_{0U};
  std::uint64_t nav_compact_last_seq_{0U};
  Clock::time_point local_compact_last_publish_{};
  Clock::time_point nav_compact_last_publish_{};

  Clock::time_point start_time_{Clock::now()};
  Clock::time_point previous_status_time_{Clock::now()};
  Clock::time_point last_raw_callback_time_{};
  Clock::time_point last_publish_time_{};
  std::uint64_t accel_core_process_pointcloud2_count_{0U};
  std::uint64_t accel_core_process_decoded_view_count_{0U};
  std::uint64_t raw_input_count_{0U};
  std::uint64_t lidar_points_publish_count_{0U};
  std::uint64_t latest_buffer_update_count_{0U};
  std::uint64_t previous_raw_input_count_{0U};
  std::uint64_t previous_lidar_points_publish_count_{0U};
  std::uint64_t previous_latest_buffer_update_count_{0U};
  std::uint64_t dropped_or_skipped_count_{0U};
  std::uint64_t raw_interarrival_count_{0U};
  std::uint64_t trunk_publish_interval_count_{0U};
  std::uint64_t trunk_publish_gap_over_100ms_count_{0U};
  std::uint64_t trunk_publish_gap_over_150ms_count_{0U};
  std::uint64_t trunk_publish_gap_over_200ms_count_{0U};
  std::uint64_t tf_drop_suspect_obstacle_header_age_over_100ms_count_{0U};
  std::uint64_t tf_drop_suspect_obstacle_header_age_over_200ms_count_{0U};
  std::size_t last_cloud_points_{0U};
  std::size_t last_cloud_bytes_{0U};
  std::size_t last_output_subscription_count_{0U};
  double last_raw_stamp_sec_{0.0};
  double last_output_stamp_sec_{0.0};
  double raw_interarrival_sum_ms_{0.0};
  double raw_interarrival_max_ms_{0.0};
  double trunk_publish_interval_sum_ms_{0.0};
  double trunk_publish_interval_max_ms_{0.0};
  double last_fast_path_duration_ms_{0.0};
  double last_trunk_publish_duration_ms_{0.0};

  CompactDiagnostics local_compact_;
  CompactDiagnostics nav_compact_;
  WorkerDiagnostics local_worker_;
  WorkerDiagnostics scan_worker_;
  sensor_msgs::msg::PointCloud2 local_compact_msg_;
  sensor_msgs::msg::PointCloud2 nav_compact_msg_;
  sensor_msgs::msg::PointCloud2 obstacle_output_msg_;
  sensor_msgs::msg::PointCloud2 clearing_output_msg_;
  sensor_msgs::msg::LaserScan scan_msg_;
  std::vector<PointXYZI> local_worker_obstacle_points_;
  std::vector<PointXYZI> local_worker_clearing_points_;
  std::vector<ClearingRayBin> local_worker_clearing_bins_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr trunk_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr local_compact_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr nav_compact_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr obstacle_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr clearing_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr accel_status_publisher_;
  rclcpp::TimerBase::SharedPtr status_timer_;
  std::thread local_worker_thread_;
  std::thread scan_worker_thread_;
};


PointCloudAccelCore::PointCloudAccelCore(rclcpp::Node & node, PointCloudAccelCoreOptions options)
: impl_(std::make_unique<Impl>(node, std::move(options)))
{
}

PointCloudAccelCore::~PointCloudAccelCore() = default;

const std::string & PointCloudAccelCore::input_topic() const
{
  return impl_->input_topic();
}

rclcpp::QoS PointCloudAccelCore::input_qos() const
{
  return impl_->input_qos();
}

void PointCloudAccelCore::process_pointcloud2(sensor_msgs::msg::PointCloud2::UniquePtr msg)
{
  impl_->process_pointcloud2(std::move(msg));
}

void PointCloudAccelCore::process_pointcloud2(sensor_msgs::msg::PointCloud2 && msg)
{
  impl_->process_pointcloud2(std::make_unique<sensor_msgs::msg::PointCloud2>(std::move(msg)));
}

void PointCloudAccelCore::process_pointcloud2(const sensor_msgs::msg::PointCloud2 & msg)
{
  impl_->process_pointcloud2(msg);
}

bool PointCloudAccelCore::process_decoded_points(const DecodedCloudView & view)
{
  return impl_->process_decoded_points(view);
}

}  // namespace robot_hesai_jt128
