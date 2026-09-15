#pragma once

#include <array>
#include <cstring>
#include <limits>
#include <optional>
#include <vector>
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace robot_hesai_jt128::detail
{
// nullopt means not applicable, with both cloud and points left untouched.
// A value reports whether the normalized points have a FLOAT32 intensity field.
template<class Point>
std::optional<bool> try_fused_axis_normalize(
  sensor_msgs::msg::PointCloud2 & cloud, const std::array<float, 9> & rotation,
  std::vector<Point> & points)
{
  const bool swap_y = rotation == std::array<float, 9>{0, 1, 0, -1, 0, 0, 0, 0, 1};
  const bool negate_y = rotation == std::array<float, 9>{0, -1, 0, -1, 0, 0, 0, 0, 1};
  if ((!swap_y && !negate_y) || cloud.is_bigendian || cloud.width == 0U ||
    cloud.height == 0U || cloud.point_step < 12U)
  {
    return std::nullopt;
  }
  const auto field_offset = [&cloud](const char * name) -> std::optional<std::size_t> {
      for (const auto & field : cloud.fields) {
        if (field.name == name && field.datatype == sensor_msgs::msg::PointField::FLOAT32 &&
          field.count >= 1U)
        {
          return field.offset;
        }
      }
      return std::nullopt;
    };
  if (field_offset("x") != 0U || field_offset("y") != 4U || field_offset("z") != 8U) {
    return std::nullopt;
  }
  const std::size_t width = cloud.width;
  const std::size_t height = cloud.height;
  const std::size_t step = cloud.point_step;
  const auto max_size = std::numeric_limits<std::size_t>::max();
  if (width > max_size / step || width * step != cloud.row_step ||
    height > max_size / cloud.row_step || width > max_size / height)
  {
    return std::nullopt;
  }
  const std::size_t count = width * height;
  if (cloud.data.size() < height * cloud.row_step || count > points.max_size()) {
    return std::nullopt;
  }
  const auto intensity_offset = field_offset("intensity");
  const bool has_intensity = intensity_offset && *intensity_offset <= step - sizeof(float);
  // All applicability checks and allocation precede mutation. Padded/foreign layouts
  // remain the caller's existing fallback, never a partially transformed cloud.
  points.reserve(count);
  points.clear();
  for (std::size_t index = 0U; index < count; ++index) {
    auto * data = cloud.data.data() + index * step;
    float raw_x, raw_y, z;
    std::memcpy(&raw_x, data, sizeof(float));
    std::memcpy(&raw_y, data + 4U, sizeof(float));
    std::memcpy(&z, data + 8U, sizeof(float));
    const float x = swap_y ? raw_y : -raw_y;
    const float y = -raw_x;
    std::memcpy(data, &x, sizeof(float));
    std::memcpy(data + 4U, &y, sizeof(float));
    float intensity = 0.0F;
    if (has_intensity) {
      // Match the old second pass even if an unusual intensity field aliases XY.
      std::memcpy(&intensity, data + *intensity_offset, sizeof(float));
    }
    points.push_back(Point{x, y, z, intensity});
  }
  return has_intensity;
}
}  // namespace robot_hesai_jt128::detail
