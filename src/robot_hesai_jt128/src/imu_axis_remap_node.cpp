#include <array>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Vector3.h"

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

geometry_msgs::msg::Vector3 rotate_vector(
  const geometry_msgs::msg::Vector3 & input,
  const tf2::Matrix3x3 & rotation_matrix)
{
  const tf2::Vector3 rotated = rotation_matrix * tf2::Vector3(input.x, input.y, input.z);
  geometry_msgs::msg::Vector3 output;
  output.x = rotated.x();
  output.y = rotated.y();
  output.z = rotated.z();
  return output;
}

std::array<double, 9> rotate_covariance(
  const std::array<double, 9> & covariance,
  const tf2::Matrix3x3 & rotation_matrix)
{
  std::array<double, 9> rotated{};
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      double value = 0.0;
      for (int left = 0; left < 3; ++left) {
        for (int right = 0; right < 3; ++right) {
          value += rotation_matrix[row][left] * covariance[left * 3 + right] * rotation_matrix[col][right];
        }
      }
      rotated[row * 3 + col] = value;
    }
  }
  return rotated;
}

tf2::Quaternion normalize_quaternion(const tf2::Quaternion & input)
{
  tf2::Quaternion output = input;
  if (output.length2() <= 0.0) {
    output.setValue(0.0, 0.0, 0.0, 1.0);
    return output;
  }
  output.normalize();
  return output;
}

}  // namespace

class ImuAxisRemapNode : public rclcpp::Node
{
public:
  ImuAxisRemapNode()
  : Node("imu_axis_remap"), logged_ready_(false)
  {
    declare_parameter<std::string>("input_topic", "/jt128/vendor/imu_raw");
    declare_parameter<std::string>("output_topic", "/lidar_imu");
    declare_parameter<std::string>("output_frame_id", "imu_link");
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
    load_rotation_matrix();

    publisher_ = create_publisher<sensor_msgs::msg::Imu>(
      output_topic_, make_qos(50, RMW_QOS_POLICY_RELIABILITY_RELIABLE));
    subscription_ = create_subscription<sensor_msgs::msg::Imu>(
      input_topic_,
      make_qos(50, RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT),
      std::bind(&ImuAxisRemapNode::on_imu, this, std::placeholders::_1));
  }

private:
  void load_rotation_matrix()
  {
    const auto raw = get_parameter("rotation_matrix").as_double_array();
    if (raw.size() != 9) {
      throw std::runtime_error("rotation_matrix must contain exactly 9 values");
    }

    rotation_matrix_.setValue(
      raw[0], raw[1], raw[2],
      raw[3], raw[4], raw[5],
      raw[6], raw[7], raw[8]);
    rotation_matrix_.getRotation(rotation_quaternion_);
    rotation_quaternion_ = normalize_quaternion(rotation_quaternion_);
  }

  void on_imu(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    sensor_msgs::msg::Imu output = *msg;
    output.header.frame_id = output_frame_id_;

    output.angular_velocity = rotate_vector(msg->angular_velocity, rotation_matrix_);
    output.linear_acceleration = rotate_vector(msg->linear_acceleration, rotation_matrix_);
    output.angular_velocity_covariance = rotate_covariance(msg->angular_velocity_covariance, rotation_matrix_);
    output.linear_acceleration_covariance = rotate_covariance(
      msg->linear_acceleration_covariance, rotation_matrix_);

    if (output.orientation_covariance[0] >= 0.0) {
      const tf2::Quaternion q_in = normalize_quaternion(
        tf2::Quaternion(
          msg->orientation.x,
          msg->orientation.y,
          msg->orientation.z,
          msg->orientation.w));
      const tf2::Quaternion q_out = normalize_quaternion(q_in * rotation_quaternion_.inverse());
      output.orientation.x = q_out.x();
      output.orientation.y = q_out.y();
      output.orientation.z = q_out.z();
      output.orientation.w = q_out.w();
      output.orientation_covariance = rotate_covariance(msg->orientation_covariance, rotation_matrix_);
    }

    publisher_->publish(output);

    if (!logged_ready_) {
      RCLCPP_INFO(
        get_logger(),
        "compiled canonical imu remap ready: %s -> %s frame=%s",
        input_topic_.c_str(),
        output_topic_.c_str(),
        output_frame_id_.c_str());
      logged_ready_ = true;
    }
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string output_frame_id_;
  bool logged_ready_;
  tf2::Matrix3x3 rotation_matrix_;
  tf2::Quaternion rotation_quaternion_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ImuAxisRemapNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
