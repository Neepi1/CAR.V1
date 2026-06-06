#include "robot_api_server/tf_pose_utils.hpp"

#include <algorithm>
#include <cmath>

namespace robot_api_server
{

std::string normalized_frame_id(std::string frame)
{
  while (!frame.empty() && frame.front() == '/') {
    frame.erase(frame.begin());
  }
  return frame;
}

double normalize_angle(const double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

double stamp_to_seconds(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1e-9;
}

double older_nonzero_stamp(const double lhs, const double rhs)
{
  if (lhs > 0.0 && rhs > 0.0) {
    return std::min(lhs, rhs);
  }
  return std::max(lhs, rhs);
}

double quaternion_yaw(const double x, const double y, const double z, const double w)
{
  const double siny_cosp = 2.0 * (w * z + x * y);
  const double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
  return std::atan2(siny_cosp, cosy_cosp);
}

}  // namespace robot_api_server
