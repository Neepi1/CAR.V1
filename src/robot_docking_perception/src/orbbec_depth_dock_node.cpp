#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "robot_docking_perception/depth_dock_geometry.hpp"
#include "robot_interfaces/msg/dock_target_observation.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace robot_docking_perception
{

namespace
{

bool host_is_big_endian()
{
  const std::uint16_t value = 0x0102U;
  return *reinterpret_cast<const std::uint8_t *>(&value) == 0x01U;
}

std::uint16_t byte_swap_16(const std::uint16_t value)
{
  return static_cast<std::uint16_t>((value >> 8U) | (value << 8U));
}

std::uint32_t byte_swap_32(const std::uint32_t value)
{
  return ((value & 0x000000ffU) << 24U) |
         ((value & 0x0000ff00U) << 8U) |
         ((value & 0x00ff0000U) >> 8U) |
         ((value & 0xff000000U) >> 24U);
}

tf2::Transform transform_from_message(const geometry_msgs::msg::Transform & message)
{
  tf2::Quaternion rotation;
  tf2::fromMsg(message.rotation, rotation);
  return tf2::Transform(
    rotation,
    tf2::Vector3(
      message.translation.x,
      message.translation.y,
      message.translation.z));
}

}  // namespace

class OrbbecDepthDockNode : public rclcpp::Node
{
public:
  OrbbecDepthDockNode()
  : Node("orbbec_depth_dock"),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
    depth_topic_ = declare_parameter<std::string>(
      "depth_topic", "/camera336l/depth/image_raw");
    camera_info_topic_ = declare_parameter<std::string>(
      "camera_info_topic", "/camera336l/depth/camera_info");
    observation_topic_ = declare_parameter<std::string>(
      "observation_topic", "/dock/target_observation");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    charge_contact_frame_ = declare_parameter<std::string>(
      "charge_contact_frame", "charge_contact_link");
    source_ = declare_parameter<std::string>("source", "orbbec_336l_depth");

    depth_unit_m_ = declare_parameter<double>("depth_unit_m", 0.001);
    min_depth_m_ = declare_parameter<double>("min_depth_m", 0.15);
    max_depth_m_ = declare_parameter<double>("max_depth_m", 2.0);
    min_base_z_m_ = declare_parameter<double>("min_base_z_m", 0.16);
    max_base_z_m_ = declare_parameter<double>("max_base_z_m", 0.36);
    pixel_stride_ = declare_parameter<int>("pixel_stride", 3);
    roi_u_min_fraction_ = declare_parameter<double>("roi_u_min_fraction", 0.0);
    roi_u_max_fraction_ = declare_parameter<double>("roi_u_max_fraction", 1.0);
    roi_v_min_fraction_ = declare_parameter<double>("roi_v_min_fraction", 0.0);
    roi_v_max_fraction_ = declare_parameter<double>("roi_v_max_fraction", 1.0);
    tf_timeout_sec_ = declare_parameter<double>("tf_timeout_sec", 0.05);
    max_processing_rate_hz_ = declare_parameter<double>("max_processing_rate_hz", 10.0);
    input_stale_timeout_sec_ = declare_parameter<double>("input_stale_timeout_sec", 1.0);
    health_publish_period_sec_ = declare_parameter<double>("health_publish_period_sec", 0.5);

    geometry_config_.min_forward_gap_m = declare_parameter<double>(
      "geometry.min_forward_gap_m", 0.02);
    geometry_config_.max_forward_gap_m = declare_parameter<double>(
      "geometry.max_forward_gap_m", 1.50);
    geometry_config_.lateral_gate_m = declare_parameter<double>(
      "geometry.lateral_gate_m", 0.45);
    geometry_config_.front_cluster_window_m = declare_parameter<double>(
      "geometry.front_cluster_window_m", 0.04);
    geometry_config_.line_inlier_threshold_m = declare_parameter<double>(
      "geometry.line_inlier_threshold_m", 0.015);
    geometry_config_.min_lateral_span_m = declare_parameter<double>(
      "geometry.min_lateral_span_m", 0.12);
    geometry_config_.max_lateral_span_m = declare_parameter<double>(
      "geometry.max_lateral_span_m", 0.36);
    geometry_config_.expected_lateral_span_m = declare_parameter<double>(
      "geometry.expected_lateral_span_m", 0.235);
    geometry_config_.expected_lateral_span_tolerance_m = declare_parameter<double>(
      "geometry.expected_lateral_span_tolerance_m", 0.06);
    geometry_config_.near_feature_expected_lateral_span_m = declare_parameter<double>(
      "geometry.near_feature_expected_lateral_span_m", 0.113);
    geometry_config_.near_feature_lateral_span_tolerance_m = declare_parameter<double>(
      "geometry.near_feature_lateral_span_tolerance_m", 0.025);
    geometry_config_.near_feature_max_forward_gap_m = declare_parameter<double>(
      "geometry.near_feature_max_forward_gap_m", 0.30);
    geometry_config_.near_feature_center_y_offset_m = declare_parameter<double>(
      "geometry.near_feature_center_y_offset_m", -0.0418);
    geometry_config_.max_rms_error_m = declare_parameter<double>(
      "geometry.max_rms_error_m", 0.015);
    geometry_config_.min_confidence = declare_parameter<double>(
      "geometry.min_confidence", 0.35);
    geometry_config_.min_points = static_cast<std::size_t>(declare_parameter<int>(
      "geometry.min_points", 60));
    geometry_config_.fit_iterations = static_cast<std::size_t>(declare_parameter<int>(
      "geometry.fit_iterations", 3));

    validate_parameters();

    observation_pub_ = create_publisher<robot_interfaces::msg::DockTargetObservation>(
      observation_topic_, rclcpp::QoS(5).reliable());
    camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      camera_info_topic_, rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::CameraInfo::SharedPtr message) {
        latest_camera_info_ = message;
      });
    depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
      depth_topic_, rclcpp::SensorDataQoS(),
      std::bind(&OrbbecDepthDockNode::handle_depth, this, std::placeholders::_1));
    auto health_period = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::duration<double>(health_publish_period_sec_));
    health_period = std::max(health_period, std::chrono::milliseconds(1));
    health_timer_ = create_wall_timer(
      health_period, [this]() {publish_stale_input_health();});

    RCLCPP_INFO(
      get_logger(),
      "Orbbec docking observer ready: depth=%s info=%s output=%s; motion output is intentionally absent",
      depth_topic_.c_str(), camera_info_topic_.c_str(), observation_topic_.c_str());
  }

private:
  void validate_parameters() const
  {
    if (depth_unit_m_ <= 0.0 || min_depth_m_ <= 0.0 || min_depth_m_ >= max_depth_m_) {
      throw std::invalid_argument("invalid depth range or depth_unit_m");
    }
    if (min_base_z_m_ >= max_base_z_m_) {
      throw std::invalid_argument("min_base_z_m must be less than max_base_z_m");
    }
    if (pixel_stride_ < 1) {
      throw std::invalid_argument("pixel_stride must be at least one");
    }
    if (roi_u_min_fraction_ < 0.0 || roi_u_max_fraction_ > 1.0 ||
      roi_u_min_fraction_ >= roi_u_max_fraction_ || roi_v_min_fraction_ < 0.0 ||
      roi_v_max_fraction_ > 1.0 || roi_v_min_fraction_ >= roi_v_max_fraction_)
    {
      throw std::invalid_argument("image ROI fractions must form a non-empty range inside [0, 1]");
    }
    if (geometry_config_.min_points < 2U || geometry_config_.fit_iterations < 1U) {
      throw std::invalid_argument("geometry point and iteration counts must be positive");
    }
    if (max_processing_rate_hz_ < 0.0) {
      throw std::invalid_argument("max_processing_rate_hz must be non-negative");
    }
    if (input_stale_timeout_sec_ <= 0.0 || health_publish_period_sec_ <= 0.0) {
      throw std::invalid_argument("input health periods must be positive");
    }
  }

  void publish_stale_input_health()
  {
    if (!have_received_depth_) {
      return;
    }
    const double age_sec = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - last_depth_received_time_).count();
    if (age_sec <= input_stale_timeout_sec_) {
      return;
    }

    robot_interfaces::msg::DockTargetObservation observation;
    observation.header.stamp = get_clock()->now();
    observation.header.frame_id = base_frame_;
    observation.source = source_;
    observation.sensor_healthy = false;
    observation.valid = false;
    observation.reason = "depth_stream_stale";
    observation_pub_->publish(observation);
  }

  void publish_invalid(
    const sensor_msgs::msg::Image & image,
    const bool sensor_healthy,
    const std::string & reason)
  {
    robot_interfaces::msg::DockTargetObservation observation;
    observation.header = image.header;
    observation.header.frame_id = base_frame_;
    observation.source = source_;
    observation.sensor_healthy = sensor_healthy;
    observation.valid = false;
    observation.reason = reason;
    observation_pub_->publish(observation);
  }

  bool read_depth_m(
    const sensor_msgs::msg::Image & image,
    const std::size_t row,
    const std::size_t column,
    double & depth_m) const
  {
    const bool swap_bytes = static_cast<bool>(image.is_bigendian) != host_is_big_endian();
    if (image.encoding == sensor_msgs::image_encodings::TYPE_16UC1 ||
      image.encoding == sensor_msgs::image_encodings::MONO16)
    {
      const std::size_t offset = row * image.step + column * sizeof(std::uint16_t);
      if (offset + sizeof(std::uint16_t) > image.data.size()) {
        return false;
      }
      std::uint16_t raw = 0U;
      std::memcpy(&raw, image.data.data() + offset, sizeof(raw));
      if (swap_bytes) {
        raw = byte_swap_16(raw);
      }
      if (raw == 0U) {
        return false;
      }
      depth_m = static_cast<double>(raw) * depth_unit_m_;
      return std::isfinite(depth_m);
    }

    if (image.encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
      const std::size_t offset = row * image.step + column * sizeof(float);
      if (offset + sizeof(float) > image.data.size()) {
        return false;
      }
      std::uint32_t raw = 0U;
      std::memcpy(&raw, image.data.data() + offset, sizeof(raw));
      if (swap_bytes) {
        raw = byte_swap_32(raw);
      }
      float value = 0.0F;
      std::memcpy(&value, &raw, sizeof(value));
      depth_m = static_cast<double>(value);
      return std::isfinite(depth_m) && depth_m > 0.0;
    }
    return false;
  }

  void handle_depth(const sensor_msgs::msg::Image::SharedPtr image)
  {
    const auto received_at = std::chrono::steady_clock::now();
    last_depth_received_time_ = received_at;
    have_received_depth_ = true;
    if (max_processing_rate_hz_ > 0.0 && have_last_processed_time_) {
      const double elapsed_sec = std::chrono::duration<double>(
        received_at - last_processed_time_).count();
      if (elapsed_sec < 1.0 / max_processing_rate_hz_) {
        return;
      }
    }
    last_processed_time_ = received_at;
    have_last_processed_time_ = true;

    const auto camera_info = latest_camera_info_;
    if (!camera_info) {
      publish_invalid(*image, false, "camera_info_unavailable");
      return;
    }
    if (image->header.frame_id.empty()) {
      publish_invalid(*image, false, "depth_frame_id_empty");
      return;
    }
    if (image->width == 0U || image->height == 0U || camera_info->k[0] <= 0.0 ||
      camera_info->k[4] <= 0.0)
    {
      publish_invalid(*image, false, "invalid_camera_calibration");
      return;
    }
    if ((camera_info->width != 0U && camera_info->width != image->width) ||
      (camera_info->height != 0U && camera_info->height != image->height))
    {
      publish_invalid(*image, false, "camera_info_resolution_mismatch");
      return;
    }
    if (!camera_info->header.frame_id.empty() &&
      camera_info->header.frame_id != image->header.frame_id)
    {
      publish_invalid(*image, false, "camera_info_frame_mismatch");
      return;
    }
    const bool supported_encoding =
      image->encoding == sensor_msgs::image_encodings::TYPE_16UC1 ||
      image->encoding == sensor_msgs::image_encodings::MONO16 ||
      image->encoding == sensor_msgs::image_encodings::TYPE_32FC1;
    if (!supported_encoding) {
      publish_invalid(*image, false, "unsupported_depth_encoding:" + image->encoding);
      return;
    }

    geometry_msgs::msg::TransformStamped sensor_transform_message;
    geometry_msgs::msg::TransformStamped contact_transform_message;
    try {
      const auto timeout = rclcpp::Duration::from_seconds(tf_timeout_sec_);
      sensor_transform_message = tf_buffer_.lookupTransform(
        base_frame_, image->header.frame_id, image->header.stamp, timeout);
      contact_transform_message = tf_buffer_.lookupTransform(
        base_frame_, charge_contact_frame_, image->header.stamp, timeout);
    } catch (const tf2::TransformException & error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Docking depth TF unavailable: %s", error.what());
      publish_invalid(*image, false, "required_tf_unavailable");
      return;
    }

    geometry_config_.charge_contact_x_m = contact_transform_message.transform.translation.x;
    geometry_config_.charge_contact_y_m = contact_transform_message.transform.translation.y;
    const tf2::Transform sensor_to_base = transform_from_message(
      sensor_transform_message.transform);

    const std::size_t u_begin = static_cast<std::size_t>(
      std::floor(roi_u_min_fraction_ * static_cast<double>(image->width)));
    const std::size_t u_end = std::min<std::size_t>(
      image->width,
      static_cast<std::size_t>(
        std::ceil(roi_u_max_fraction_ * static_cast<double>(image->width))));
    const std::size_t v_begin = static_cast<std::size_t>(
      std::floor(roi_v_min_fraction_ * static_cast<double>(image->height)));
    const std::size_t v_end = std::min<std::size_t>(
      image->height,
      static_cast<std::size_t>(
        std::ceil(roi_v_max_fraction_ * static_cast<double>(image->height))));

    const double fx = camera_info->k[0];
    const double fy = camera_info->k[4];
    const double cx = camera_info->k[2];
    const double cy = camera_info->k[5];
    std::vector<PlanarPoint> planar_points;
    planar_points.reserve(
      ((u_end - u_begin) / static_cast<std::size_t>(pixel_stride_) + 1U) *
      ((v_end - v_begin) / static_cast<std::size_t>(pixel_stride_) + 1U));

    for (std::size_t v = v_begin; v < v_end; v += static_cast<std::size_t>(pixel_stride_)) {
      for (std::size_t u = u_begin; u < u_end; u += static_cast<std::size_t>(pixel_stride_)) {
        double depth_m = 0.0;
        if (!read_depth_m(*image, v, u, depth_m) ||
          depth_m < min_depth_m_ || depth_m > max_depth_m_)
        {
          continue;
        }
        const tf2::Vector3 optical_point(
          (static_cast<double>(u) - cx) * depth_m / fx,
          (static_cast<double>(v) - cy) * depth_m / fy,
          depth_m);
        const tf2::Vector3 base_point = sensor_to_base * optical_point;
        if (base_point.z() < min_base_z_m_ || base_point.z() > max_base_z_m_) {
          continue;
        }
        planar_points.push_back({base_point.x(), base_point.y()});
      }
    }

    const auto estimate = estimate_dock_target(planar_points, geometry_config_);
    robot_interfaces::msg::DockTargetObservation observation;
    observation.header = image->header;
    observation.header.frame_id = base_frame_;
    observation.source = source_;
    observation.sensor_healthy = true;
    observation.valid = estimate.valid;
    observation.forward_gap_m = estimate.forward_gap_m;
    observation.lateral_error_m = estimate.lateral_error_m;
    observation.yaw_error_rad = estimate.yaw_error_rad;
    observation.lateral_span_m = estimate.lateral_span_m;
    observation.confidence = estimate.confidence;
    observation.inlier_count = static_cast<std::uint32_t>(std::min<std::size_t>(
      estimate.inlier_count, std::numeric_limits<std::uint32_t>::max()));
    observation.rms_error_m = estimate.rms_error_m;
    observation.reason = estimate.reason;
    observation_pub_->publish(observation);
  }

  std::string depth_topic_;
  std::string camera_info_topic_;
  std::string observation_topic_;
  std::string base_frame_;
  std::string charge_contact_frame_;
  std::string source_;
  double depth_unit_m_{0.001};
  double min_depth_m_{0.15};
  double max_depth_m_{2.0};
  double min_base_z_m_{0.16};
  double max_base_z_m_{0.36};
  int pixel_stride_{3};
  double roi_u_min_fraction_{0.0};
  double roi_u_max_fraction_{1.0};
  double roi_v_min_fraction_{0.0};
  double roi_v_max_fraction_{1.0};
  double tf_timeout_sec_{0.05};
  double max_processing_rate_hz_{10.0};
  double input_stale_timeout_sec_{1.0};
  double health_publish_period_sec_{0.5};
  bool have_received_depth_{false};
  std::chrono::steady_clock::time_point last_depth_received_time_{};
  bool have_last_processed_time_{false};
  std::chrono::steady_clock::time_point last_processed_time_{};
  DockGeometryConfig geometry_config_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  sensor_msgs::msg::CameraInfo::SharedPtr latest_camera_info_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Publisher<robot_interfaces::msg::DockTargetObservation>::SharedPtr observation_pub_;
  rclcpp::TimerBase::SharedPtr health_timer_;
};

}  // namespace robot_docking_perception

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<robot_docking_perception::OrbbecDepthDockNode>());
  rclcpp::shutdown();
  return 0;
}
