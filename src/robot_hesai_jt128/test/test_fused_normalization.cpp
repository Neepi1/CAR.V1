#include <gtest/gtest.h>
#include <cstdint>
#include <cstring>
#include <functional>
#include <limits>
#include "robot_hesai_jt128/detail/fused_normalization.hpp"

namespace
{
struct Point {float x, y, z, intensity;};
const std::array<float, 9> rotation{0, 1, 0, -1, 0, 0, 0, 0, 1};
void put(sensor_msgs::msg::PointCloud2 & c, std::size_t offset, float value)
{std::memcpy(c.data.data() + offset, &value, sizeof(value));}
sensor_msgs::msg::PointCloud2 fixture(std::size_t count = 2)
{
  sensor_msgs::msg::PointCloud2 c;
  c.header.frame_id = "lidar_link";
  c.header.stamp.sec = 123;
  c.header.stamp.nanosec = 456;
  c.width = count;
  c.height = 1;
  c.point_step = 32;
  c.row_step = c.width * c.point_step;
  for (auto entry : {std::pair{"x", 0U}, {"y", 4U}, {"z", 8U}, {"intensity", 16U}}) {
    sensor_msgs::msg::PointField field;
    field.name = entry.first; field.offset = entry.second;
    field.datatype = sensor_msgs::msg::PointField::FLOAT32; field.count = 1;
    c.fields.push_back(field);
  }
  c.data.assign(c.row_step, 0xa5);
  for (std::size_t i = 0; i < count; ++i) {
    put(c, i * 32, 1.25F + i); put(c, i * 32 + 4, -2.5F);
    put(c, i * 32 + 8, 3.0F); put(c, i * 32 + 16, 42.0F);
  }
  return c;
}
TEST(FusedNormalization, PreservesFullCloudAndBuildsCanonicalPoints)
{
  auto cloud = fixture();
  auto expected = cloud;
  put(expected, 0, -2.5F); put(expected, 4, -1.25F);
  put(expected, 32, -2.5F); put(expected, 36, -2.25F);
  std::vector<Point> points;
  const auto result = robot_hesai_jt128::detail::try_fused_axis_normalize(cloud, rotation, points);
  ASSERT_TRUE(result.has_value());
  EXPECT_TRUE(*result);
  EXPECT_EQ(cloud, expected);
  ASSERT_EQ(points.size(), 2U);
  EXPECT_FLOAT_EQ(points[0].x, -2.5F); EXPECT_FLOAT_EQ(points[0].y, -1.25F);
  EXPECT_FLOAT_EQ(points[0].z, 3.0F); EXPECT_FLOAT_EQ(points[0].intensity, 42.0F);
}
TEST(FusedNormalization, MissingOrUnsupportedIntensityBecomesZero)
{
  for (int variant = 0; variant < 3; ++variant) {
    auto cloud = fixture();
    if (variant == 0) {cloud.fields.pop_back();}
    if (variant == 1) {cloud.fields.back().datatype = sensor_msgs::msg::PointField::UINT8;}
    if (variant == 2) {cloud.fields.back().offset = cloud.point_step - 1;}
    std::vector<Point> points;
    const auto result = robot_hesai_jt128::detail::try_fused_axis_normalize(cloud, rotation, points);
    ASSERT_TRUE(result.has_value());
    EXPECT_FALSE(*result);
    ASSERT_EQ(points.size(), 2U);
    EXPECT_FLOAT_EQ(points[0].intensity, 0.0F);
  }
}
TEST(FusedNormalization, PreservesSpecialFloatBitsAndReusesCapacity)
{
  auto cloud = fixture(4);
  const std::uint32_t bits[] = {0x7fc12345U, 0x7f800000U, 0x00000000U, 0x80000000U};
  for (std::size_t i = 0; i < 4; ++i) {
    for (const auto offset : {0U, 4U, 8U, 16U}) {
      std::memcpy(cloud.data.data() + i * 32 + offset, &bits[i], 4);
    }
  }
  auto expected = cloud;
  for (std::size_t i = 0; i < 4; ++i) {
    const auto negated = bits[i] ^ 0x80000000U;
    std::memcpy(expected.data.data() + i * 32 + 4, &negated, 4);
  }
  std::vector<Point> points;
  points.reserve(8);
  const auto * storage = points.data();
  ASSERT_TRUE(robot_hesai_jt128::detail::try_fused_axis_normalize(cloud, rotation, points));
  EXPECT_EQ(cloud, expected);
  EXPECT_EQ(points.data(), storage);
  for (std::size_t i = 0; i < points.size(); ++i) {
    EXPECT_EQ(std::memcmp(&points[i].x, expected.data.data() + i * 32, 12), 0);
    EXPECT_EQ(std::memcmp(&points[i].intensity, expected.data.data() + i * 32 + 16, 4), 0);
  }
}
TEST(FusedNormalization, HandlesContiguousRowsAndSecondCanonicalRotation)
{
  auto cloud = fixture(4);
  cloud.height = 2; cloud.width = 2; cloud.row_step = 64;
  auto second_rotation = rotation; second_rotation[1] = -1;
  std::vector<Point> points;
  ASSERT_TRUE(robot_hesai_jt128::detail::try_fused_axis_normalize(cloud, second_rotation, points));
  ASSERT_EQ(points.size(), 4U);
  EXPECT_FLOAT_EQ(points[3].x, 2.5F);
  EXPECT_FLOAT_EQ(points[3].y, -4.25F);
}
TEST(FusedNormalization, InapplicableLayoutIsUntouched)
{
  using Cloud = sensor_msgs::msg::PointCloud2;
  const std::vector<std::function<void(Cloud &)>> mutations{
    [](Cloud & c) {c.row_step += 4; c.data.resize(c.row_step);},
    [](Cloud & c) {c.data.pop_back();},
    [](Cloud & c) {c.is_bigendian = true;},
    [](Cloud & c) {c.point_step = 8;},
    [](Cloud & c) {c.fields[0].offset = 12;},
    [](Cloud & c) {c.fields[0].count = 0;},
    [](Cloud & c) {c.width = 0;},
    [](Cloud & c) {c.row_step = 0;},
    [](Cloud & c) {c.width = c.height = c.point_step = 0xffffffffU;},
    [](Cloud & c) {c.fields.clear();}};
  for (const auto & mutate : mutations) {
    auto cloud = fixture(); mutate(cloud);
    const auto before = cloud;
    std::vector<Point> points{{1, 2, 3, 4}};
    EXPECT_FALSE(robot_hesai_jt128::detail::try_fused_axis_normalize(cloud, rotation, points));
    EXPECT_EQ(cloud, before);
    ASSERT_EQ(points.size(), 1U);
    EXPECT_FLOAT_EQ(points[0].x, 1);
    EXPECT_FLOAT_EQ(points[0].intensity, 4);
  }
  auto cloud = fixture(); const auto before = cloud;
  std::vector<Point> points;
  EXPECT_FALSE(robot_hesai_jt128::detail::try_fused_axis_normalize(
    cloud, std::array<float, 9>{1, 0, 0, 0, 1, 0, 0, 0, 1}, points));
  EXPECT_EQ(cloud, before);
}
TEST(FusedNormalization, IntensityAliasingMatchesPostTransformRead)
{
  auto cloud = fixture(); cloud.fields.back().offset = 4;
  std::vector<Point> points;
  ASSERT_TRUE(robot_hesai_jt128::detail::try_fused_axis_normalize(cloud, rotation, points));
  EXPECT_FLOAT_EQ(points[0].intensity, -1.25F);
}
}  // namespace
