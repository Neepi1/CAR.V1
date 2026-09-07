#pragma once

#include <cmath>

namespace robot_hesai_jt128
{

struct ScanSelfMask
{
  bool enabled{true};
  double min_x{-0.39};
  double max_x{0.39};
  double min_y{-0.28};
  double max_y{0.28};

  bool valid() const noexcept
  {
    return std::isfinite(min_x) && std::isfinite(max_x) &&
           std::isfinite(min_y) && std::isfinite(max_y) &&
           min_x <= max_x && min_y <= max_y;
  }

  bool contains(const double x, const double y) const noexcept
  {
    return enabled && valid() && std::isfinite(x) && std::isfinite(y) &&
           x >= min_x && x <= max_x && y >= min_y && y <= max_y;
  }
};

}  // namespace robot_hesai_jt128
