#include <cmath>
#include <cstring>
#include <limits>
#include <random>
#include <iostream>
#include <gtest/gtest.h>
#include "imu_rotation_reference.hpp"
#include "robot_local_state/imu_rotation_cache.hpp"

namespace
{
double max_abs_difference = 0.0;
void same_number(double actual, double expected)
{
  if (std::isnan(expected)) {EXPECT_TRUE(std::isnan(actual));}
  else if (std::isinf(expected)) {EXPECT_EQ(actual, expected);}
  else {
    ASSERT_TRUE(std::isfinite(actual));
    // Recompiling/extracting floating-point expressions need not be bitwise identical.
    const double scale = std::max({1.0, std::abs(actual), std::abs(expected)});
    max_abs_difference = std::max(max_abs_difference, std::abs(actual - expected));
    EXPECT_NEAR(actual, expected, 16.0 * std::numeric_limits<double>::epsilon() * scale);
    if (actual == 0.0 && expected == 0.0) {EXPECT_EQ(std::signbit(actual), std::signbit(expected));}
  }
}
void same_imu(const sensor_msgs::msg::Imu & actual, const sensor_msgs::msg::Imu & expected)
{
  EXPECT_EQ(actual.header, expected.header);
  EXPECT_EQ(actual.orientation, expected.orientation);
  EXPECT_EQ(actual.orientation_covariance, expected.orientation_covariance);
  for (auto pair : {std::make_pair(actual.angular_velocity, expected.angular_velocity),
      std::make_pair(actual.linear_acceleration, expected.linear_acceleration)})
  {
    same_number(pair.first.x, pair.second.x);
    same_number(pair.first.y, pair.second.y);
    same_number(pair.first.z, pair.second.z);
  }
  for (std::size_t i = 0; i < 9; ++i) {
    same_number(actual.angular_velocity_covariance[i], expected.angular_velocity_covariance[i]);
    same_number(actual.linear_acceleration_covariance[i], expected.linear_acceleration_covariance[i]);
  }
}

TEST(ImuRotationCache, MatchesLegacyAcrossRepeatedAndChangedInputs)
{
  robot_local_state::ImuRotationCache cache;
  std::mt19937 generator(42);
  std::uniform_real_distribution<double> number(-2.0, 2.0);
  sensor_msgs::msg::Imu input;
  geometry_msgs::msg::Quaternion q;
  for (int sample = 0; sample < 1000; ++sample) {
    if (sample % 17 == 0) {q.x = number(generator); q.y = number(generator);
      q.z = number(generator); q.w = number(generator);}
    if (sample % 11 == 0) {
      for (auto & value : input.angular_velocity_covariance) {value = number(generator);}
    }
    if (sample % 13 == 0) {
      for (auto & value : input.linear_acceleration_covariance) {value = number(generator);}
    }
    input.header.frame_id = sample % 2 ? "imu_link" : "other_imu";
    input.header.stamp.sec = sample;
    input.angular_velocity.x = number(generator); input.angular_velocity.y = number(generator);
    input.angular_velocity.z = number(generator); input.linear_acceleration.x = number(generator);
    input.linear_acceleration.y = number(generator); input.linear_acceleration.z = number(generator);
    auto actual = input; auto expected = input;
    cache.apply(actual, q); imu_rotation_reference::apply(expected, q);
    same_imu(actual, expected);
  }
  std::cout << "max_abs_difference=" << max_abs_difference << '\n';
}

TEST(ImuRotationCache, PreservesZeroQuaternionUnknownCovarianceAndNonfiniteInputs)
{
  robot_local_state::ImuRotationCache cache;
  geometry_msgs::msg::Quaternion q;
  q.x = q.y = q.z = q.w = 0.0;
  sensor_msgs::msg::Imu input;
  input.angular_velocity.x = 0.1; input.linear_acceleration.z = 9.81;
  for (const double special : {-1.0, -0.0, 0.0, 0.0025,
      std::numeric_limits<double>::infinity(), std::numeric_limits<double>::quiet_NaN()})
  {
    input.angular_velocity_covariance[0] = special;
    input.linear_acceleration_covariance[8] = special;
    for (int repeat = 0; repeat < 3; ++repeat) {
      auto actual = input; auto expected = input;
      cache.apply(actual, q); imu_rotation_reference::apply(expected, q);
      same_imu(actual, expected);
    }
  }
}

TEST(ImuRotationCache, DoesNotIgnoreSmallRotationChangesOrReuseInvalidRotations)
{
  robot_local_state::ImuRotationCache cache;
  sensor_msgs::msg::Imu input;
  input.angular_velocity.x = 0.5;
  input.linear_acceleration.z = 9.81;
  input.angular_velocity_covariance = {0.1,0,0,0,0.2,0,0,0,0.3};
  geometry_msgs::msg::Quaternion q;
  q.w = 1.0;
  for (const double z : {0.0, -0.0, 1.0e-10, 1.0e-14,
      std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity(),
      0.0, 0.5, -0.5})
  {
    q.z = z;
    auto actual = input; auto expected = input;
    cache.apply(actual, q); imu_rotation_reference::apply(expected, q);
    same_imu(actual, expected);
  }
}

TEST(ImuRotationCache, WarmAndFreshResultsMatchIncludingSingleUlpCovarianceChanges)
{
  robot_local_state::ImuRotationCache warm;
  sensor_msgs::msg::Imu input;
  input.angular_velocity.x = 0.5;
  input.linear_acceleration.z = 9.81;
  input.angular_velocity_covariance = {0.1,0,0,0,0.2,0,0,0,0.3};
  input.linear_acceleration_covariance = {0.4,0,0,0,0.5,0,0,0,0.6};
  geometry_msgs::msg::Quaternion q;
  q.w = 1.0;
  for (int i = 0; i < 10; ++i) {
    if (i == 3) {input.angular_velocity_covariance[0] = std::nextafter(0.1, 1.0);}
    if (i == 5) {input.linear_acceleration_covariance[8] = std::nextafter(0.6, 1.0);}
    if (i == 7) {q.z = 0.1;}
    if (i == 8) {q.z = std::nextafter(q.z, 1.0);}
    robot_local_state::ImuRotationCache fresh;
    auto cached = input; auto uncached = input;
    warm.apply(cached, q); fresh.apply(uncached, q);
    EXPECT_EQ(std::memcmp(cached.angular_velocity_covariance.data(),
      uncached.angular_velocity_covariance.data(), 9 * sizeof(double)), 0);
    EXPECT_EQ(std::memcmp(cached.linear_acceleration_covariance.data(),
      uncached.linear_acceleration_covariance.data(), 9 * sizeof(double)), 0);
    same_imu(cached, uncached);
  }
}
}  // namespace
