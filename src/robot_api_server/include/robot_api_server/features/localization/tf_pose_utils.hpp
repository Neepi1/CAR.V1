#pragma once

#include <string>

#include "builtin_interfaces/msg/time.hpp"

namespace robot_api_server
{

std::string normalized_frame_id(std::string frame);

double normalize_angle(double angle);

double stamp_to_seconds(const builtin_interfaces::msg::Time & stamp);

double older_nonzero_stamp(double lhs, double rhs);

double quaternion_yaw(double x, double y, double z, double w);

}  // namespace robot_api_server
