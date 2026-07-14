/************************************************************************************************
  Copyright(C)2023 Hesai Technology Co., Ltd.
  All code in this repository is released under the terms of the following [Modified BSD License.]
  Modified BSD License:
  Redistribution and use in source and binary forms,with or without modification,are permitted
  provided that the following conditions are met:
  *Redistributions of source code must retain the above copyright notice,this list of conditions
   and the following disclaimer.
  *Redistributions in binary form must reproduce the above copyright notice,this list of conditions and
   the following disclaimer in the documentation and/or other materials provided with the distribution.
  *Neither the names of the University of Texas at Austin,nor Austin Robot Technology,nor the names of
   other contributors maybe used to endorse or promote products derived from this software without
   specific prior written permission.
  THIS SOFTWARE IS PROVIDED BY THE COPYRIGH THOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
  WARRANTIES,INCLUDING,BUT NOT LIMITED TO,THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
  PARTICULAR PURPOSE ARE DISCLAIMED.IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
  ANY DIRECT,INDIRECT,INCIDENTAL,SPECIAL,EXEMPLARY,OR CONSEQUENTIAL DAMAGES(INCLUDING,BUT NOT LIMITED TO,
  PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;LOSS OF USE,DATA,OR PROFITS;OR BUSINESS INTERRUPTION)HOWEVER
  CAUSED AND ON ANY THEORY OF LIABILITY,WHETHER IN CONTRACT,STRICT LIABILITY,OR TORT(INCLUDING NEGLIGENCE
  OR OTHERWISE)ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,EVEN IF ADVISED OF THE POSSIBILITY OF
  SUCHDAMAGE.
************************************************************************************************/

/*
 * File: source_driver_ros2.hpp
 * Author: Zhang Yu <zhangyu@hesaitech.com>
 * Description: Source Driver for ROS2
 * Created on June 12, 2023, 10:46 AM
 */

#pragma once
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sstream>
#include <hesai_ros_driver/msg/udp_frame.hpp>
#include <hesai_ros_driver/msg/udp_packet.hpp>
#include <hesai_ros_driver/msg/ptp.hpp>
#include <hesai_ros_driver/msg/firetime.hpp>
#include <hesai_ros_driver/msg/loss_packet.hpp>

#include <fstream>
#include <memory>
#include <chrono>
#include <string>
#include <functional>
#include <cmath>
#include <limits>
#include <boost/thread.hpp>
#include "source_drive_common.hpp"

#ifdef HESAI_ACCEL_DRIVER_INTEGRATED
#include "robot_hesai_jt128/pointcloud_accel_core.hpp"
#endif

#ifndef HESAI_ROS_DRIVER_NODE_NAME
#define HESAI_ROS_DRIVER_NODE_NAME "hesai_ros_driver_node"
#endif

class SourceDriver
{
public:
  typedef std::shared_ptr<SourceDriver> Ptr;
  // Initialize some necessary configuration parameters, create ROS nodes, and register callback functions
  virtual void Init(const YAML::Node& config);
  // Start working
  virtual void Start();
  // Stop working
  virtual void Stop();
  virtual ~SourceDriver();
  SourceDriver(SourceType src_type) {};
  void SpinRos2(){rclcpp::spin(this->node_ptr_);}
  std::shared_ptr<rclcpp::Node> node_ptr_;
  std::shared_ptr<HesaiLidarSdk<LidarPointXYZIRT>> driver_ptr_;
protected:
  // Save Correction file subscribed by "ros_recv_correction_topic"
  void ReceiveCorrection(const std_msgs::msg::UInt8MultiArray::SharedPtr msg);
  // Save packets subscribed by 'ros_recv_packet_topic'
  void ReceivePacket(const hesai_ros_driver::msg::UdpFrame::SharedPtr msg);
  // Used to publish point clouds through 'ros_send_point_cloud_topic'
  void SendPointCloud(const LidarDecodedFrame<LidarPointXYZIRT>& msg);
  // Used to publish the original pcake through 'ros_send_packet_topic'
  void SendPacket(const UdpFrame_t&  ros_msg, double timestamp);

  // Used to publish the Correction file through 'ros_send_correction_topic'
  void SendCorrection(const u8Array_t& msg);
  // Used to publish the Packet loss condition
  void SendPacketLoss(const uint32_t& total_packet_count, const uint32_t& total_packet_loss_count);
  // Used to publish the Packet loss condition
  void SendPTP(const uint8_t& ptp_lock_offset, const u8Array_t& ptp_status);
  // Used to publish the firetime correction
  void SendFiretime(const double *firetime_correction_);
  // Used to publish the imu packet
  void SendImuConfig(const LidarImuData& msg);

  // Convert ptp lock offset, status into ROS message
  hesai_ros_driver::msg::Ptp ToRosMsg(const uint8_t& ptp_lock_offset, const u8Array_t& ptp_status);
  // Convert packet loss condition into ROS message
  hesai_ros_driver::msg::LossPacket ToRosMsg(const uint32_t& total_packet_count, const uint32_t& total_packet_loss_count);
  // Convert correction string into ROS messages
  std_msgs::msg::UInt8MultiArray ToRosMsg(const u8Array_t& correction_string);
  // Convert double[512] to float64[512]
  hesai_ros_driver::msg::Firetime ToRosMsg(const double *firetime_correction_);
  // Convert point clouds into ROS messages
  sensor_msgs::msg::PointCloud2 ToRosMsg(const LidarDecodedFrame<LidarPointXYZIRT>& frame, const std::string& frame_id);
  // Convert packets into ROS messages
  hesai_ros_driver::msg::UdpFrame ToRosMsg(const UdpFrame_t& ros_msg, double timestamp);
  // Convert imu, imu into ROS message
  sensor_msgs::msg::Imu ToRosMsg(const LidarImuData& firetime_correction_);
  // Convert Linear Acceleration from g to m/s^2
  double From_g_To_ms2(double g);
  // Convert Angular Velocity from degree/s to radian/s
  double From_degs_To_rads(double degree);
  double NormalizeHeaderTimestamp(double raw_timestamp);
  builtin_interfaces::msg::Time ToBuiltinTime(double timestamp);
  std::string frame_id_;
  bool sensor_time_offset_initialized_ = false;
  double sensor_time_offset_sec_ = 0.0;
#ifdef HESAI_ACCEL_DRIVER_INTEGRATED
  bool publish_vendor_raw_debug_ = false;
  bool publish_vendor_imu_raw_debug_ = true;
  std::unique_ptr<robot_hesai_jt128::PointCloudAccelCore> accel_core_;
#endif

  rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr crt_sub_;
  rclcpp::Subscription<hesai_ros_driver::msg::UdpFrame>::SharedPtr pkt_sub_;
  rclcpp::Publisher<hesai_ros_driver::msg::UdpFrame>::SharedPtr pkt_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
  rclcpp::Publisher<hesai_ros_driver::msg::Firetime>::SharedPtr firetime_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr crt_pub_;
  rclcpp::Publisher<hesai_ros_driver::msg::LossPacket>::SharedPtr loss_pub_;
  rclcpp::Publisher<hesai_ros_driver::msg::Ptp>::SharedPtr ptp_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;

  //spin thread while Receive data from ROS topic
  boost::thread* subscription_spin_thread_{nullptr};
};
inline void SourceDriver::Init(const YAML::Node& config)
{
  DriverParam driver_param;
  DriveYamlParam yaml_param;
  yaml_param.GetDriveYamlParam(config, driver_param);
  frame_id_ = driver_param.input_param.frame_id;

  node_ptr_.reset(new rclcpp::Node(HESAI_ROS_DRIVER_NODE_NAME));
#ifdef HESAI_ACCEL_DRIVER_INTEGRATED
  publish_vendor_raw_debug_ =
    node_ptr_->declare_parameter<bool>("publish_vendor_raw_debug", false);
  publish_vendor_imu_raw_debug_ =
    node_ptr_->declare_parameter<bool>("publish_vendor_imu_raw_debug", true);
  robot_hesai_jt128::PointCloudAccelCoreOptions core_options;
  core_options.accel_ingress_profile = "driver_integrated";
  core_options.input_path = "driver_callback_pointcloud2";
  core_options.vendor_raw_ros_hop_required = false;
  core_options.vendor_raw_debug_publish_enabled = publish_vendor_raw_debug_;
  core_options.driver_integrated_process = true;
  core_options.driver_integrated_unavailable_reason = "available";
  accel_core_ = std::make_unique<robot_hesai_jt128::PointCloudAccelCore>(
    *node_ptr_, core_options);
#endif
  if (driver_param.input_param.send_point_cloud_ros) {
#ifdef HESAI_ACCEL_DRIVER_INTEGRATED
    if (publish_vendor_raw_debug_) {
      pub_ = node_ptr_->create_publisher<sensor_msgs::msg::PointCloud2>(
        driver_param.input_param.ros_send_point_topic, 10);
    }
#else
    pub_ = node_ptr_->create_publisher<sensor_msgs::msg::PointCloud2>(driver_param.input_param.ros_send_point_topic, 10);
#endif
  }
  if (driver_param.input_param.send_imu_ros) {
#ifdef HESAI_ACCEL_DRIVER_INTEGRATED
    if (publish_vendor_imu_raw_debug_) {
      imu_pub_ = node_ptr_->create_publisher<sensor_msgs::msg::Imu>(
        driver_param.input_param.ros_send_imu_topic, 10);
    }
#else
    imu_pub_ = node_ptr_->create_publisher<sensor_msgs::msg::Imu>(driver_param.input_param.ros_send_imu_topic, 10);
#endif
  }

  if (driver_param.input_param.ros_send_packet_loss_topic != NULL_TOPIC) {
    loss_pub_ = node_ptr_->create_publisher<hesai_ros_driver::msg::LossPacket>(driver_param.input_param.ros_send_packet_loss_topic, 10);
  }

  if (driver_param.input_param.source_type == DATA_FROM_LIDAR) {
    if (driver_param.input_param.ros_send_ptp_topic != NULL_TOPIC) {
      ptp_pub_ = node_ptr_->create_publisher<hesai_ros_driver::msg::Ptp>(driver_param.input_param.ros_send_ptp_topic, 10);
    }

    if (driver_param.input_param.ros_send_correction_topic != NULL_TOPIC) {
      crt_pub_ = node_ptr_->create_publisher<std_msgs::msg::UInt8MultiArray>(driver_param.input_param.ros_send_correction_topic, 10);
    }
  }
  if (! driver_param.input_param.firetimes_path.empty() ) {
    if (driver_param.input_param.ros_send_firetime_topic != NULL_TOPIC) {
      firetime_pub_ = node_ptr_->create_publisher<hesai_ros_driver::msg::Firetime>(driver_param.input_param.ros_send_firetime_topic, 10);
    }
  }

  if (driver_param.input_param.send_packet_ros) {
    pkt_pub_ = node_ptr_->create_publisher<hesai_ros_driver::msg::UdpFrame>(driver_param.input_param.ros_send_packet_topic, 10);
  }

  if (driver_param.input_param.source_type == DATA_FROM_ROS_PACKET) {
    pkt_sub_ = node_ptr_->create_subscription<hesai_ros_driver::msg::UdpFrame>(driver_param.input_param.ros_recv_packet_topic, 10,
                              std::bind(&SourceDriver::ReceivePacket, this, std::placeholders::_1));
    if (driver_param.input_param.ros_recv_correction_topic != NULL_TOPIC) {
      crt_sub_ = node_ptr_->create_subscription<std_msgs::msg::UInt8MultiArray>(driver_param.input_param.ros_recv_correction_topic, 10,
                              std::bind(&SourceDriver::ReceiveCorrection, this, std::placeholders::_1));
    }
    driver_param.decoder_param.enable_udp_thread = false;
    subscription_spin_thread_ = new boost::thread(boost::bind(&SourceDriver::SpinRos2,this));
  }
#ifdef HESAI_ACCEL_DRIVER_INTEGRATED
  if (subscription_spin_thread_ == nullptr) {
    subscription_spin_thread_ = new boost::thread(boost::bind(&SourceDriver::SpinRos2, this));
  }
#endif
  driver_ptr_.reset(new HesaiLidarSdk<LidarPointXYZIRT>());
  driver_param.decoder_param.enable_parser_thread = true;
  if (driver_param.input_param.send_point_cloud_ros) {
    driver_ptr_->RegRecvCallback([this](const hesai::lidar::LidarDecodedFrame<hesai::lidar::LidarPointXYZIRT>& frame) {
      this->SendPointCloud(frame);
    });
  }
  if (driver_param.input_param.send_imu_ros) {
#ifdef HESAI_ACCEL_DRIVER_INTEGRATED
    if (publish_vendor_imu_raw_debug_) {
      driver_ptr_->RegRecvCallback(std::bind(&SourceDriver::SendImuConfig, this, std::placeholders::_1));
    }
#else
    driver_ptr_->RegRecvCallback(std::bind(&SourceDriver::SendImuConfig, this, std::placeholders::_1));
#endif
  }
  if (driver_param.input_param.send_packet_ros) {
    driver_ptr_->RegRecvCallback(std::bind(&SourceDriver::SendPacket, this, std::placeholders::_1, std::placeholders::_2)) ;
  }
  if (driver_param.input_param.ros_send_packet_loss_topic != NULL_TOPIC) {
    driver_ptr_->RegRecvCallback(std::bind(&SourceDriver::SendPacketLoss, this, std::placeholders::_1, std::placeholders::_2));
  }
  if (driver_param.input_param.source_type == DATA_FROM_LIDAR) {
    if (driver_param.input_param.ros_send_correction_topic != NULL_TOPIC) {
      driver_ptr_->RegRecvCallback(std::bind(&SourceDriver::SendCorrection, this, std::placeholders::_1));
    }
    if (driver_param.input_param.ros_send_ptp_topic != NULL_TOPIC) {
      driver_ptr_->RegRecvCallback(std::bind(&SourceDriver::SendPTP, this, std::placeholders::_1, std::placeholders::_2));
    }
  }
  if (!driver_ptr_->Init(driver_param))
  {
    std::cout << "Driver Initialize Error...." << std::endl;
    exit(-1);
  }
}

inline void SourceDriver::Start()
{
  driver_ptr_->Start();
}

inline SourceDriver::~SourceDriver()
{
  Stop();
}

inline void SourceDriver::Stop()
{
  if (driver_ptr_) {
    driver_ptr_->Stop();
  }
#ifdef ROS2_FOUND
  if (subscription_spin_thread_ != nullptr) {
    rclcpp::shutdown();
    subscription_spin_thread_->join();
    delete subscription_spin_thread_;
    subscription_spin_thread_ = nullptr;
  }
#endif
}

inline void SourceDriver::SendPacket(const UdpFrame_t& msg, double timestamp)
{
  pkt_pub_->publish(ToRosMsg(msg, timestamp));
}

inline void SourceDriver::SendPointCloud(const LidarDecodedFrame<LidarPointXYZIRT>& msg)
{
  auto cloud = ToRosMsg(msg, frame_id_);
#ifdef HESAI_ACCEL_DRIVER_INTEGRATED
  if (publish_vendor_raw_debug_ && pub_) {
    pub_->publish(cloud);
  }
  if (accel_core_) {
    accel_core_->process_pointcloud2(std::move(cloud));
  }
#else
  pub_->publish(cloud);
#endif
}

inline double SourceDriver::NormalizeHeaderTimestamp(double raw_timestamp)
{
  if (!std::isfinite(raw_timestamp) || raw_timestamp <= 0.0)
  {
    return node_ptr_->get_clock()->now().seconds();
  }

  constexpr double kEpochThresholdSec = 946684800.0;  // 2000-01-01 00:00:00 UTC
  if (raw_timestamp >= kEpochThresholdSec)
  {
    return raw_timestamp;
  }

  if (!sensor_time_offset_initialized_)
  {
    sensor_time_offset_sec_ = node_ptr_->get_clock()->now().seconds() - raw_timestamp;
    sensor_time_offset_initialized_ = true;
  }
  return raw_timestamp + sensor_time_offset_sec_;
}

inline builtin_interfaces::msg::Time SourceDriver::ToBuiltinTime(double timestamp)
{
  builtin_interfaces::msg::Time stamp;
  if (!std::isfinite(timestamp) || timestamp <= 0.0)
  {
    const auto now_ns = node_ptr_->get_clock()->now().nanoseconds();
    stamp.sec = static_cast<int32_t>(now_ns / 1000000000ll);
    stamp.nanosec = static_cast<uint32_t>(now_ns % 1000000000ll);
    return stamp;
  }

  const double seconds_floor = std::floor(timestamp);
  const double fractional = timestamp - seconds_floor;
  const auto sec = static_cast<int64_t>(seconds_floor);
  if (sec > std::numeric_limits<int32_t>::max())
  {
    const auto now_ns = node_ptr_->get_clock()->now().nanoseconds();
    stamp.sec = static_cast<int32_t>(now_ns / 1000000000ll);
    stamp.nanosec = static_cast<uint32_t>(now_ns % 1000000000ll);
    return stamp;
  }

  stamp.sec = static_cast<int32_t>(sec);
  stamp.nanosec = static_cast<uint32_t>(std::llround(fractional * 1e9));
  if (stamp.nanosec >= 1000000000u)
  {
    stamp.nanosec -= 1000000000u;
    stamp.sec += 1;
  }
  return stamp;
}

inline void SourceDriver::SendCorrection(const u8Array_t& msg)
{
  crt_pub_->publish(ToRosMsg(msg));
}

inline void SourceDriver::SendPacketLoss(const uint32_t& total_packet_count, const uint32_t& total_packet_loss_count)
{
  loss_pub_->publish(ToRosMsg(total_packet_count, total_packet_loss_count));
}

inline void SourceDriver::SendPTP(const uint8_t& ptp_lock_offset, const u8Array_t& ptp_status)
{
  ptp_pub_->publish(ToRosMsg(ptp_lock_offset, ptp_status));
}

inline void SourceDriver::SendFiretime(const double *firetime_correction_)
{
  firetime_pub_->publish(ToRosMsg(firetime_correction_));
}

inline void SourceDriver::SendImuConfig(const LidarImuData& msg)
{
  imu_pub_->publish(ToRosMsg(msg));
}

inline sensor_msgs::msg::PointCloud2 SourceDriver::ToRosMsg(const LidarDecodedFrame<LidarPointXYZIRT>& frame, const std::string& frame_id)
{
  sensor_msgs::msg::PointCloud2 ros_msg;
  uint32_t points_number = (frame.fParam.IsMultiFrameFrequency() == 0) ? frame.points_num : frame.multi_points_num;
  LidarPointXYZIRT *pPoints = (frame.fParam.IsMultiFrameFrequency() == 0) ? frame.points : frame.multi_points;
  double frame_start_timestamp = (frame.fParam.IsMultiFrameFrequency() == 0) ? frame.frame_start_timestamp : frame.multi_frame_start_timestamp;
  const double normalized_frame_start_timestamp = NormalizeHeaderTimestamp(frame_start_timestamp);
  // Use 32-byte PointXYZIRT layout (aligned) for consumers that expect it (e.g. NVBlox).
  // x,y,z,pad (16 bytes) + intensity (4) + ring (2) + pad (2) + timestamp (8) = 32 bytes
  int fields = 8;
  ros_msg.fields.clear();
  ros_msg.fields.reserve(fields);
  uint32_t width = points_number;
  uint32_t height = 1;
  if (frame.fParam.remake_config.flag) {
    const int cfg_width = frame.fParam.remake_config.max_azi_scan;
    const int cfg_height = frame.fParam.remake_config.max_elev_scan;
    if (cfg_width > 0 && cfg_height > 0 &&
        static_cast<uint64_t>(cfg_width) * static_cast<uint64_t>(cfg_height) == points_number) {
      width = static_cast<uint32_t>(cfg_width);
      height = static_cast<uint32_t>(cfg_height);
    }
  }
  ros_msg.width = width;
  ros_msg.height = height;

  int offset = 0;
  offset = addPointField(ros_msg, "x", 1, sensor_msgs::msg::PointField::FLOAT32, offset);
  offset = addPointField(ros_msg, "y", 1, sensor_msgs::msg::PointField::FLOAT32, offset);
  offset = addPointField(ros_msg, "z", 1, sensor_msgs::msg::PointField::FLOAT32, offset);
  // Pad to 16-byte alignment after XYZ (PCL PointXYZIRT layout).
  offset = addPointField(ros_msg, "padding_xyz", 1, sensor_msgs::msg::PointField::UINT32, offset);
  offset = addPointField(ros_msg, "intensity", 1, sensor_msgs::msg::PointField::FLOAT32, offset);
  offset = addPointField(ros_msg, "ring", 1, sensor_msgs::msg::PointField::UINT16, offset);
  // Pad to 8-byte alignment before timestamp.
  offset = addPointField(ros_msg, "padding", 1, sensor_msgs::msg::PointField::UINT16, offset);
  offset = addPointField(ros_msg, "timestamp", 1, sensor_msgs::msg::PointField::FLOAT64, offset);

  ros_msg.point_step = offset;
  ros_msg.row_step = ros_msg.width * ros_msg.point_step;
  ros_msg.is_dense = false;
  ros_msg.data.resize(points_number * ros_msg.point_step);

  sensor_msgs::PointCloud2Iterator<float> iter_x_(ros_msg, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y_(ros_msg, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z_(ros_msg, "z");
  sensor_msgs::PointCloud2Iterator<float> iter_intensity_(ros_msg, "intensity");
  sensor_msgs::PointCloud2Iterator<uint16_t> iter_ring_(ros_msg, "ring");
  sensor_msgs::PointCloud2Iterator<double> iter_timestamp_(ros_msg, "timestamp");
  const bool force_ring_by_row = (frame.fParam.remake_config.flag && height > 1 && width > 1);
  for (size_t i = 0; i < points_number; i++)
  {
    LidarPointXYZIRT point = pPoints[i];
    *iter_x_ = point.x;
    *iter_y_ = point.y;
    *iter_z_ = point.z;
    *iter_intensity_ = point.intensity;
    if (force_ring_by_row) {
      *iter_ring_ = static_cast<uint16_t>(i / width);
    } else {
      *iter_ring_ = point.ring;
    }
    *iter_timestamp_ = point.timestamp;
    ++iter_x_;
    ++iter_y_;
    ++iter_z_;
    ++iter_intensity_;
    ++iter_ring_;
    ++iter_timestamp_;
  }
  // Per-frame stdout logging can generate gigabytes of runtime logs and is not
  // part of the pointcloud data path. Keep frame timing in explicit probes.
  ros_msg.header.stamp = ToBuiltinTime(normalized_frame_start_timestamp);
  ros_msg.header.frame_id = frame_id_;
  return ros_msg;
}

inline hesai_ros_driver::msg::UdpFrame SourceDriver::ToRosMsg(const UdpFrame_t& ros_msg, double timestamp) {
  hesai_ros_driver::msg::UdpFrame rs_msg;
  for (size_t i = 0 ; i < ros_msg.size(); i++) {
    hesai_ros_driver::msg::UdpPacket rawpacket;
    rawpacket.size = ros_msg[i].packet_len;
    rawpacket.data.resize(ros_msg[i].packet_len);
    memcpy(&rawpacket.data[0], &ros_msg[i].buffer[0], ros_msg[i].packet_len);
    rs_msg.packets.push_back(rawpacket);
  }
  rs_msg.header.stamp = ToBuiltinTime(NormalizeHeaderTimestamp(timestamp));
  rs_msg.header.frame_id = frame_id_;
  return rs_msg;
}

inline std_msgs::msg::UInt8MultiArray SourceDriver::ToRosMsg(const u8Array_t& correction_string) {
  auto msg = std::make_shared<std_msgs::msg::UInt8MultiArray>();
  msg->data.resize(correction_string.size());
  std::copy(correction_string.begin(), correction_string.end(), msg->data.begin());
  return *msg;
}

inline hesai_ros_driver::msg::LossPacket SourceDriver::ToRosMsg(const uint32_t& total_packet_count, const uint32_t& total_packet_loss_count)
{
  hesai_ros_driver::msg::LossPacket msg;
  msg.total_packet_count = total_packet_count;
  msg.total_packet_loss_count = total_packet_loss_count;
  return msg;
}

inline hesai_ros_driver::msg::Ptp SourceDriver::ToRosMsg(const uint8_t& ptp_lock_offset, const u8Array_t& ptp_status)
{
  hesai_ros_driver::msg::Ptp msg;
  msg.ptp_lock_offset = ptp_lock_offset;
  std::copy(ptp_status.begin(), ptp_status.begin() + std::min(16ul, ptp_status.size()), msg.ptp_status.begin());
  return msg;
}

inline hesai_ros_driver::msg::Firetime SourceDriver::ToRosMsg(const double *firetime_correction_)
{
  hesai_ros_driver::msg::Firetime msg;
  std::copy(firetime_correction_, firetime_correction_ + 512, msg.data.begin());
  return msg;
}

inline sensor_msgs::msg::Imu SourceDriver::ToRosMsg(const LidarImuData &imu_config_)
{
  sensor_msgs::msg::Imu ros_msg;
  ros_msg.header.stamp = ToBuiltinTime(NormalizeHeaderTimestamp(imu_config_.timestamp));
  ros_msg.header.frame_id = frame_id_;
  ros_msg.linear_acceleration.x = From_g_To_ms2(imu_config_.imu_accel_x);
  ros_msg.linear_acceleration.y = From_g_To_ms2(imu_config_.imu_accel_y);
  ros_msg.linear_acceleration.z = From_g_To_ms2(imu_config_.imu_accel_z);
  ros_msg.angular_velocity.x = From_degs_To_rads(imu_config_.imu_ang_vel_x);
  ros_msg.angular_velocity.y = From_degs_To_rads(imu_config_.imu_ang_vel_y);
  ros_msg.angular_velocity.z = From_degs_To_rads(imu_config_.imu_ang_vel_z);
  return ros_msg;
}

inline void SourceDriver::ReceivePacket(const hesai_ros_driver::msg::UdpFrame::SharedPtr msg)
{
  for (size_t i = 0; i < msg->packets.size(); i++) {
    if(driver_ptr_->lidar_ptr_->origin_packets_buffer_.full()) std::this_thread::sleep_for(std::chrono::microseconds(10000));
    driver_ptr_->lidar_ptr_->origin_packets_buffer_.emplace_back(&msg->packets[i].data[0], msg->packets[i].size);
  }
}

inline void SourceDriver::ReceiveCorrection(const std_msgs::msg::UInt8MultiArray::SharedPtr msg)
{
  driver_ptr_->lidar_ptr_->correction_string_.resize(msg->data.size());
  std::copy(msg->data.begin(), msg->data.end(), driver_ptr_->lidar_ptr_->correction_string_.begin());
  while (1) {
    if (! driver_ptr_->lidar_ptr_->LoadCorrectionFromROSbag()) {
      break;
    }
  }
}
inline double SourceDriver::From_g_To_ms2(double g)
{
  return g * 9.80665;
}

inline double SourceDriver::From_degs_To_rads(double degree)
{
  return degree * M_PI / 180.0;
}
