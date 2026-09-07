#pragma once

#include <cmath>
#include <stdexcept>

namespace robot_nav_config
{

struct DockStagingGoalLimits
{
  double forward_capture_min_m{-0.40};
  double forward_capture_max_m{0.55};
  double lateral_capture_max_m{0.25};
  double yaw_capture_max_rad{3.14159265358979323846};
};

struct DockStagingGoalSample
{
  double current_x{0.0};
  double current_y{0.0};
  double current_yaw{0.0};
  double goal_x{0.0};
  double goal_y{0.0};
  double goal_yaw{0.0};
};

struct DockStagingGoalAssessment
{
  bool valid{false};
  bool reached{false};
  double forward_error_m{0.0};
  double lateral_error_m{0.0};
  double yaw_error_rad{0.0};
};

// Pure policy for the Nav2-to-docking-manager capture boundary. Errors are
// projected into the commissioned predock heading, matching the docking
// manager's near-field coordinate convention.
class DockStagingGoalPolicy
{
public:
  explicit DockStagingGoalPolicy(DockStagingGoalLimits limits)
  : limits_(limits)
  {
    constexpr double kPi = 3.14159265358979323846;
    const bool finite =
      std::isfinite(limits_.forward_capture_min_m) &&
      std::isfinite(limits_.forward_capture_max_m) &&
      std::isfinite(limits_.lateral_capture_max_m) &&
      std::isfinite(limits_.yaw_capture_max_rad);
    if (!finite || limits_.forward_capture_min_m > 0.0 ||
      limits_.forward_capture_max_m < 0.0 ||
      limits_.forward_capture_min_m > limits_.forward_capture_max_m ||
      limits_.lateral_capture_max_m < 0.0 ||
      limits_.yaw_capture_max_rad < 0.0 || limits_.yaw_capture_max_rad > kPi)
    {
      throw std::invalid_argument(
              "dock staging goal limits must be finite, contain zero, and use "
              "non-negative lateral/yaw bounds no greater than pi");
    }
  }

  DockStagingGoalAssessment assess(const DockStagingGoalSample & sample) const
  {
    DockStagingGoalAssessment result;
    if (!sample_is_finite(sample)) {
      return result;
    }

    const double dx = sample.current_x - sample.goal_x;
    const double dy = sample.current_y - sample.goal_y;
    const double cosine = std::cos(sample.goal_yaw);
    const double sine = std::sin(sample.goal_yaw);
    result.valid = true;
    result.forward_error_m = cosine * dx + sine * dy;
    result.lateral_error_m = -sine * dx + cosine * dy;
    result.yaw_error_rad = std::fabs(normalize_angle(
        sample.current_yaw - sample.goal_yaw));
    result.reached =
      result.forward_error_m >= limits_.forward_capture_min_m &&
      result.forward_error_m <= limits_.forward_capture_max_m &&
      std::fabs(result.lateral_error_m) <= limits_.lateral_capture_max_m &&
      result.yaw_error_rad <= limits_.yaw_capture_max_rad;
    return result;
  }

  const DockStagingGoalLimits & limits() const
  {
    return limits_;
  }

private:
  static bool sample_is_finite(const DockStagingGoalSample & sample)
  {
    return std::isfinite(sample.current_x) &&
           std::isfinite(sample.current_y) &&
           std::isfinite(sample.current_yaw) &&
           std::isfinite(sample.goal_x) &&
           std::isfinite(sample.goal_y) &&
           std::isfinite(sample.goal_yaw);
  }

  static double normalize_angle(const double angle)
  {
    return std::atan2(std::sin(angle), std::cos(angle));
  }

  DockStagingGoalLimits limits_;
};

}  // namespace robot_nav_config
