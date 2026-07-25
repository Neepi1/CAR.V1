#include "robot_elevator_manager/threshold_footprint.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace robot_elevator_manager
{
namespace
{

constexpr double kGeometryEpsilon = 1.0e-6;

bool finite(const Point2 & point) noexcept
{
  return std::isfinite(point.x) && std::isfinite(point.y);
}

double cross(
  const Point2 & origin,
  const Point2 & end,
  const Point2 & point) noexcept
{
  return (end.x - origin.x) * (point.y - origin.y) -
         (end.y - origin.y) * (point.x - origin.x);
}

FootprintClassification invalid(std::string message)
{
  FootprintClassification result;
  result.ok = false;
  result.occupancy = ThresholdOccupancy::kStraddling;
  result.message = std::move(message);
  return result;
}

}  // namespace

FootprintClassification classify_footprint(
  const DoorThreshold & threshold,
  const std::vector<Point2> & footprint)
{
  if (!finite(threshold.left) || !finite(threshold.right) ||
    !finite(threshold.cabin_reference) || !std::isfinite(threshold.clearance_m) ||
    threshold.clearance_m < 0.0 || !std::isfinite(threshold.jamb_clearance_m) ||
    threshold.jamb_clearance_m < 0.0)
  {
    return invalid("threshold contains non-finite geometry or an invalid clearance");
  }
  const double dx = threshold.right.x - threshold.left.x;
  const double dy = threshold.right.y - threshold.left.y;
  const double length = std::hypot(dx, dy);
  if (length <= kGeometryEpsilon) {
    return invalid("threshold endpoints are degenerate");
  }
  if (2.0 * threshold.jamb_clearance_m >= length) {
    return invalid("door jamb clearance leaves no usable opening");
  }
  const double raw_cabin_reference =
    cross(threshold.left, threshold.right, threshold.cabin_reference) / length;
  if (std::abs(raw_cabin_reference) <= kGeometryEpsilon) {
    return invalid("cabin reference lies on the threshold line");
  }
  if (footprint.size() < 3U) {
    return invalid("a footprint polygon requires at least three vertices");
  }

  double twice_area = 0.0;
  for (std::size_t index = 0; index < footprint.size(); ++index) {
    const auto & current = footprint[index];
    const auto & next = footprint[(index + 1U) % footprint.size()];
    if (!finite(current) || !finite(next)) {
      return invalid("footprint contains a non-finite vertex");
    }
    twice_area += current.x * next.y - next.x * current.y;
  }
  if (std::abs(twice_area) <= kGeometryEpsilon) {
    return invalid("footprint polygon is degenerate");
  }

  const double cabin_sign = raw_cabin_reference > 0.0 ? 1.0 : -1.0;
  double minimum = std::numeric_limits<double>::infinity();
  double maximum = -std::numeric_limits<double>::infinity();
  double minimum_projection = std::numeric_limits<double>::infinity();
  double maximum_projection = -std::numeric_limits<double>::infinity();
  for (const auto & vertex : footprint) {
    const double signed_distance =
      cabin_sign * cross(threshold.left, threshold.right, vertex) / length;
    const double doorway_projection =
      ((vertex.x - threshold.left.x) * dx + (vertex.y - threshold.left.y) * dy) /
      length;
    minimum = std::min(minimum, signed_distance);
    maximum = std::max(maximum, signed_distance);
    minimum_projection = std::min(minimum_projection, doorway_projection);
    maximum_projection = std::max(maximum_projection, doorway_projection);
  }

  FootprintClassification result;
  result.ok = true;
  result.minimum_cabin_distance_m = minimum;
  result.maximum_cabin_distance_m = maximum;
  result.minimum_doorway_projection_m = minimum_projection;
  result.maximum_doorway_projection_m = maximum_projection;
  result.within_jamb_clearance =
    minimum_projection > threshold.jamb_clearance_m &&
    maximum_projection < length - threshold.jamb_clearance_m;
  if (!result.within_jamb_clearance) {
    result.occupancy = ThresholdOccupancy::kStraddling;
    result.message = "full footprint is outside the usable door-jamb span";
  } else if (minimum > threshold.clearance_m) {
    result.occupancy = ThresholdOccupancy::kInside;
    result.message = "every footprint vertex is beyond the cabin clearance";
  } else if (maximum < -threshold.clearance_m) {
    result.occupancy = ThresholdOccupancy::kOutside;
    result.message = "every footprint vertex is beyond the hall clearance";
  } else {
    result.occupancy = ThresholdOccupancy::kStraddling;
    result.message = "footprint crosses or touches the threshold clearance band";
  }
  return result;
}

std::string to_string(const ThresholdOccupancy occupancy)
{
  switch (occupancy) {
    case ThresholdOccupancy::kInside:
      return "INSIDE";
    case ThresholdOccupancy::kOutside:
      return "OUTSIDE";
    case ThresholdOccupancy::kStraddling:
      return "STRADDLING";
  }
  return "STRADDLING";
}

}  // namespace robot_elevator_manager
