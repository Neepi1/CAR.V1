#include <algorithm>
#include <cmath>
#include <fstream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/static_transform_broadcaster.h"

namespace
{

std::string trim(const std::string & input)
{
  const auto begin = input.find_first_not_of(" \t\r\n");
  if (begin == std::string::npos) {
    return "";
  }
  const auto end = input.find_last_not_of(" \t\r\n");
  return input.substr(begin, end - begin + 1);
}

std::unordered_map<std::string, std::string> parse_flat_yaml(const std::string & path)
{
  std::ifstream input(path);
  if (!input) {
    throw std::runtime_error("failed to open config_file: " + path);
  }
  std::unordered_map<std::string, std::string> values;
  std::string line;
  while (std::getline(input, line)) {
    const auto comment = line.find('#');
    if (comment != std::string::npos) {
      line = line.substr(0, comment);
    }
    const auto sep = line.find(':');
    if (sep == std::string::npos) {
      continue;
    }
    const auto key = trim(line.substr(0, sep));
    const auto value = trim(line.substr(sep + 1));
    if (!key.empty() && !value.empty()) {
      values[key] = value;
    }
  }
  return values;
}

std::string require_string(const std::unordered_map<std::string, std::string> & values, const std::string & key)
{
  const auto it = values.find(key);
  if (it == values.end()) {
    throw std::runtime_error("missing required key in sensors config: " + key);
  }
  return it->second;
}

double require_double(const std::unordered_map<std::string, std::string> & values, const std::string & key)
{
  return std::stod(require_string(values, key));
}

tf2::Quaternion quaternion_from_rpy(const double roll, const double pitch, const double yaw)
{
  tf2::Quaternion q;
  q.setRPY(roll, pitch, yaw);
  q.normalize();
  return q;
}

double yaw_from_quaternion(const tf2::Quaternion & q)
{
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  return yaw;
}

geometry_msgs::msg::TransformStamped make_transform(
  const rclcpp::Time & stamp,
  const std::string & parent,
  const std::string & child,
  const double x,
  const double y,
  const double z,
  const double roll,
  const double pitch,
  const double yaw)
{
  geometry_msgs::msg::TransformStamped tf;
  tf.header.stamp = stamp;
  tf.header.frame_id = parent;
  tf.child_frame_id = child;
  tf.transform.translation.x = x;
  tf.transform.translation.y = y;
  tf.transform.translation.z = z;
  const auto q = quaternion_from_rpy(roll, pitch, yaw);
  tf.transform.rotation.x = q.x();
  tf.transform.rotation.y = q.y();
  tf.transform.rotation.z = q.z();
  tf.transform.rotation.w = q.w();
  return tf;
}

}  // namespace

class RobotDescriptionStaticTfNode : public rclcpp::Node
{
public:
  RobotDescriptionStaticTfNode()
  : Node("robot_description_static_tf"),
    broadcaster_(std::make_unique<tf2_ros::StaticTransformBroadcaster>(this))
  {
    const auto config_file = declare_parameter<std::string>("config_file", "");
    if (config_file.empty()) {
      throw std::runtime_error("config_file parameter is required");
    }
    broadcaster_->sendTransform(build_transforms(parse_flat_yaml(config_file)));
  }

private:
  std::vector<geometry_msgs::msg::TransformStamped> build_transforms(
    const std::unordered_map<std::string, std::string> & config)
  {
    const auto stamp = now();
    const auto base_frame = require_string(config, "base_frame");
    const auto ranger_base_frame =
      config.count("ranger_base_frame") ? require_string(config, "ranger_base_frame") : "";
    const auto base_footprint_frame = require_string(config, "base_footprint_frame");
    const auto lidar_mount_frame = require_string(config, "lidar_mount_frame");
    const auto lidar_frame = require_string(config, "lidar_frame");
    const auto lidar_level_frame =
      config.count("lidar_level_frame") ? require_string(config, "lidar_level_frame") : "lidar_level_link";
    const auto imu_frame = require_string(config, "imu_frame");
    const auto gs2_frame = config.count("gs2_frame") ? require_string(config, "gs2_frame") : "gs2_link";
    const auto charge_contact_frame =
      config.count("charge_contact_frame") ? require_string(config, "charge_contact_frame") : "charge_contact_link";

    const double lidar_x = require_double(config, "lidar_x");
    const double lidar_y = require_double(config, "lidar_y");
    const double lidar_z = require_double(config, "lidar_z");
    const double lidar_roll = require_double(config, "lidar_roll");
    const double lidar_pitch = require_double(config, "lidar_pitch");
    const double lidar_yaw = require_double(config, "lidar_yaw");
    const double axis_roll = require_double(config, "lidar_axis_roll");
    const double axis_pitch = require_double(config, "lidar_axis_pitch");
    const double axis_yaw = require_double(config, "lidar_axis_yaw");

    const auto install_q = quaternion_from_rpy(lidar_roll, lidar_pitch, lidar_yaw);
    const auto axis_q = quaternion_from_rpy(axis_roll, axis_pitch, axis_yaw);
    auto final_q = install_q * axis_q;
    final_q.normalize();
    const double lidar_level_yaw = yaw_from_quaternion(final_q);

    std::vector<geometry_msgs::msg::TransformStamped> transforms;
    if (!ranger_base_frame.empty() && ranger_base_frame != base_frame) {
      transforms.push_back(make_transform(stamp, base_frame, ranger_base_frame, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0));
    }
    transforms.push_back(make_transform(stamp, base_frame, base_footprint_frame, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0));
    transforms.push_back(
      make_transform(stamp, base_frame, lidar_mount_frame, lidar_x, lidar_y, lidar_z, lidar_roll, lidar_pitch, lidar_yaw));
    transforms.push_back(
      make_transform(stamp, lidar_mount_frame, lidar_frame, 0.0, 0.0, 0.0, axis_roll, axis_pitch, axis_yaw));
    transforms.push_back(
      make_transform(stamp, base_frame, lidar_level_frame, lidar_x, lidar_y, lidar_z, 0.0, 0.0, lidar_level_yaw));
    transforms.push_back(make_transform(
      stamp,
      base_frame,
      imu_frame,
      require_double(config, "imu_x"),
      require_double(config, "imu_y"),
      require_double(config, "imu_z"),
      require_double(config, "imu_roll"),
      require_double(config, "imu_pitch"),
      require_double(config, "imu_yaw")));
    if (config.count("gs2_x") || config.count("gs2_y") || config.count("gs2_z")) {
      transforms.push_back(make_transform(
        stamp,
        base_frame,
        gs2_frame,
        require_double(config, "gs2_x"),
        require_double(config, "gs2_y"),
        require_double(config, "gs2_z"),
        require_double(config, "gs2_roll"),
        require_double(config, "gs2_pitch"),
        require_double(config, "gs2_yaw")));
    }
    if (config.count("charge_contact_x") || config.count("charge_contact_y") || config.count("charge_contact_z")) {
      transforms.push_back(make_transform(
        stamp,
        base_frame,
        charge_contact_frame,
        require_double(config, "charge_contact_x"),
        require_double(config, "charge_contact_y"),
        require_double(config, "charge_contact_z"),
        require_double(config, "charge_contact_roll"),
        require_double(config, "charge_contact_pitch"),
        require_double(config, "charge_contact_yaw")));
    }
    return transforms;
  }

  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> broadcaster_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RobotDescriptionStaticTfNode>());
  rclcpp::shutdown();
  return 0;
}
