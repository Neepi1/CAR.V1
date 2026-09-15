#pragma once

#include <array>
#include "geometry_msgs/msg/quaternion.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2/LinearMath/Matrix3x3.h"

// Frozen pre-optimization arithmetic, independent of the production cache.
namespace imu_rotation_reference
{
inline std::array<double, 9> covariance(
  const std::array<double, 9> & input, const tf2::Matrix3x3 & rotation)
{
  std::array<double, 9> output{};
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      double value = 0.0;
      for (int left = 0; left < 3; ++left) {
        for (int right = 0; right < 3; ++right) {
          value += rotation[row][left] * input[left * 3 + right] * rotation[col][right];
        }
      }
      output[row * 3 + col] = value;
    }
  }
  return output;
}

inline void apply(sensor_msgs::msg::Imu & imu, const geometry_msgs::msg::Quaternion & msg)
{
  tf2::Quaternion q(msg.x, msg.y, msg.z, msg.w);
  if (q.length2() <= 0.0) {q.setValue(0.0, 0.0, 0.0, 1.0);} else {q.normalize();}
  const tf2::Matrix3x3 rotation(q);
  for (auto * vector : {&imu.angular_velocity, &imu.linear_acceleration}) {
    const auto value = rotation * tf2::Vector3(vector->x, vector->y, vector->z);
    vector->x = value.x(); vector->y = value.y(); vector->z = value.z();
  }
  imu.angular_velocity_covariance = covariance(imu.angular_velocity_covariance, rotation);
  imu.linear_acceleration_covariance = covariance(imu.linear_acceleration_covariance, rotation);
}
}  // namespace imu_rotation_reference
