#pragma once

#include <cstddef>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/header.hpp"

namespace robot_hesai_jt128
{

struct DecodedCloudView
{
  std_msgs::msg::Header header;
  const void * points{nullptr};
  std::size_t point_count{0U};
  bool has_intensity{false};
  std::string point_type;
};

struct PointCloudAccelCoreOptions
{
  std::string accel_ingress_profile{"separate_process"};
  std::string input_path{"pointcloud2_topic"};
  bool vendor_raw_ros_hop_required{true};
  bool vendor_raw_debug_publish_enabled{false};
  bool driver_integrated_process{false};
  std::string driver_integrated_unavailable_reason{"none"};
};

class PointCloudAccelCore
{
public:
  PointCloudAccelCore(rclcpp::Node & node, PointCloudAccelCoreOptions options = {});
  ~PointCloudAccelCore();

  PointCloudAccelCore(const PointCloudAccelCore &) = delete;
  PointCloudAccelCore & operator=(const PointCloudAccelCore &) = delete;

  const std::string & input_topic() const;
  rclcpp::QoS input_qos() const;
  void process_pointcloud2(sensor_msgs::msg::PointCloud2::UniquePtr msg);
  void process_pointcloud2(sensor_msgs::msg::PointCloud2 && msg);
  void process_pointcloud2(const sensor_msgs::msg::PointCloud2 & msg);
  bool process_decoded_points(const DecodedCloudView & view);

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace robot_hesai_jt128
