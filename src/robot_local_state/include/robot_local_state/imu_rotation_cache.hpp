#pragma once

#include <array>
#include <algorithm>
#include <cmath>
#include <cstring>
#include "geometry_msgs/msg/quaternion.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2/LinearMath/Matrix3x3.h"

namespace robot_local_state
{
// Arithmetic only: callers still query TF for every accepted input sample.
class ImuRotationCache
{
public:
  void apply(sensor_msgs::msg::Imu & imu, const geometry_msgs::msg::Quaternion & msg)
  {
    const std::array<double, 4> key{msg.x, msg.y, msg.z, msg.w};
    if (!have_rotation_ || !same_bits(key, rotation_key_)) {
      tf2::Quaternion q(msg.x, msg.y, msg.z, msg.w);
      if (q.length2() <= 0.0) {q.setValue(0.0, 0.0, 0.0, 1.0);} else {q.normalize();}
      rotation_ = tf2::Matrix3x3(q);
      rotation_key_ = key;
      have_rotation_ = all_finite(key);
      angular_.valid = false;
      linear_.valid = false;
    }
    for (auto * vector : {&imu.angular_velocity, &imu.linear_acceleration}) {
      const auto value = rotation_ * tf2::Vector3(vector->x, vector->y, vector->z);
      vector->x = value.x(); vector->y = value.y(); vector->z = value.z();
    }
    imu.angular_velocity_covariance = covariance(imu.angular_velocity_covariance, angular_);
    imu.linear_acceleration_covariance = covariance(imu.linear_acceleration_covariance, linear_);
  }

private:
  struct CovarianceCache
  {
    bool valid{false};
    std::array<double, 9> input{};
    std::array<double, 9> output{};
  };

  template<std::size_t Size>
  static bool same_bits(const std::array<double, Size> & a, const std::array<double, Size> & b)
  {
    // Preserve signed-zero distinctions; do not quantize small external changes.
    return std::memcmp(a.data(), b.data(), sizeof(double) * Size) == 0;
  }

  template<std::size_t Size>
  static bool all_finite(const std::array<double, Size> & values)
  {
    return std::all_of(values.begin(), values.end(), [](double value) {return std::isfinite(value);});
  }

  const std::array<double, 9> & covariance(
    const std::array<double, 9> & input, CovarianceCache & cache)
  {
    if (!cache.valid || !same_bits(input, cache.input)) {
      cache.output = rotate_covariance(input, rotation_);
      cache.input = input;
      cache.valid = have_rotation_ && all_finite(input);
    }
    return cache.output;
  }

  static std::array<double, 9> rotate_covariance(
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

  bool have_rotation_{false};
  std::array<double, 4> rotation_key_{};
  tf2::Matrix3x3 rotation_;
  CovarianceCache angular_;
  CovarianceCache linear_;
};
}  // namespace robot_local_state
