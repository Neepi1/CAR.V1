#pragma once

#include <algorithm>
#include <cmath>
#include <optional>

namespace robot_nav_config
{

enum class StartupAlignmentDecision
{
  kHold,
  kRotate,
  kComplete,
};

class StartupAlignmentGuard
{
public:
  void reset()
  {
    rotation_started_ = false;
  }

  StartupAlignmentDecision evaluate(
    const std::optional<double> heading_error,
    const bool path_too_short,
    const double engagement_threshold,
    const double disengagement_threshold)
  {
    if (path_too_short) {
      return StartupAlignmentDecision::kComplete;
    }
    if (!heading_error.has_value() || !std::isfinite(*heading_error)) {
      return StartupAlignmentDecision::kHold;
    }

    const double threshold = std::max(
      0.0, rotation_started_ ? disengagement_threshold : engagement_threshold);
    if (std::abs(*heading_error) > threshold) {
      rotation_started_ = true;
      return StartupAlignmentDecision::kRotate;
    }
    return StartupAlignmentDecision::kComplete;
  }

  bool rotation_started() const
  {
    return rotation_started_;
  }

private:
  bool rotation_started_{false};
};

}  // namespace robot_nav_config
