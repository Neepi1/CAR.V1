#pragma once

#include <cmath>

namespace robot_localization_bridge::se2
{

struct Pose2D
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

struct PoseDifference
{
  double dx_map_m{0.0};
  double dy_map_m{0.0};
  double dyaw_rad{0.0};
  double translation_m{0.0};
  double yaw_rad{0.0};
};

struct CorrectionSolution
{
  Pose2D target_map_odom;
  Pose2D predicted_map_base;
  double base_dx_map_m{0.0};
  double base_dy_map_m{0.0};
  double base_dyaw_rad{0.0};
  double base_translation_m{0.0};
  double base_yaw_rad{0.0};
  double map_odom_parameter_translation_m{0.0};
  double map_odom_parameter_yaw_rad{0.0};
};

inline double normalize_yaw(const double yaw)
{
  return std::atan2(std::sin(yaw), std::cos(yaw));
}

inline double degrees_to_radians(const double degrees)
{
  return degrees * 3.14159265358979323846 / 180.0;
}

inline Pose2D compose(const Pose2D & parent_child, const Pose2D & child_object)
{
  const double cos_yaw = std::cos(parent_child.yaw);
  const double sin_yaw = std::sin(parent_child.yaw);
  return Pose2D{
    parent_child.x + cos_yaw * child_object.x - sin_yaw * child_object.y,
    parent_child.y + sin_yaw * child_object.x + cos_yaw * child_object.y,
    normalize_yaw(parent_child.yaw + child_object.yaw)};
}

inline Pose2D solve_map_odom(
  const Pose2D & measured_map_base,
  const Pose2D & odom_base)
{
  const double map_odom_yaw = normalize_yaw(measured_map_base.yaw - odom_base.yaw);
  const double cos_yaw = std::cos(map_odom_yaw);
  const double sin_yaw = std::sin(map_odom_yaw);
  return Pose2D{
    measured_map_base.x - (cos_yaw * odom_base.x - sin_yaw * odom_base.y),
    measured_map_base.y - (sin_yaw * odom_base.x + cos_yaw * odom_base.y),
    map_odom_yaw};
}

inline PoseDifference difference(const Pose2D & from, const Pose2D & to)
{
  PoseDifference result;
  result.dx_map_m = to.x - from.x;
  result.dy_map_m = to.y - from.y;
  result.dyaw_rad = normalize_yaw(to.yaw - from.yaw);
  result.translation_m = std::hypot(result.dx_map_m, result.dy_map_m);
  result.yaw_rad = std::abs(result.dyaw_rad);
  return result;
}

inline CorrectionSolution solve_correction(
  const Pose2D & current_map_odom,
  const Pose2D & odom_base,
  const Pose2D & measured_map_base)
{
  CorrectionSolution result;
  result.target_map_odom = solve_map_odom(measured_map_base, odom_base);
  result.predicted_map_base = compose(current_map_odom, odom_base);

  const auto base_error = difference(result.predicted_map_base, measured_map_base);
  result.base_dx_map_m = base_error.dx_map_m;
  result.base_dy_map_m = base_error.dy_map_m;
  result.base_dyaw_rad = base_error.dyaw_rad;
  result.base_translation_m = base_error.translation_m;
  result.base_yaw_rad = base_error.yaw_rad;

  const auto parameter_delta = difference(current_map_odom, result.target_map_odom);
  result.map_odom_parameter_translation_m = parameter_delta.translation_m;
  result.map_odom_parameter_yaw_rad = parameter_delta.yaw_rad;
  return result;
}

inline PoseDifference compare_map_odom_hypotheses_at_reference(
  const Pose2D & first_map_odom,
  const Pose2D & second_map_odom,
  const Pose2D & reference_odom_base)
{
  return difference(
    compose(first_map_odom, reference_odom_base),
    compose(second_map_odom, reference_odom_base));
}

}  // namespace robot_localization_bridge::se2
