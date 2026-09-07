#include "robot_nav_config/elevator_scoped_clearance.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <mutex>
#include <vector>

#include "nav2_costmap_2d/cost_values.hpp"

namespace robot_nav_config {
namespace {

struct Point2d {
  double x;
  double y;
};

constexpr double kGeometryEpsilon = 1.0e-9;

double cross_product(const Point2d &start, const Point2d &end,
                     const Point2d &point) {
  return (end.x - start.x) * (point.y - start.y) -
         (end.y - start.y) * (point.x - start.x);
}

bool point_is_on_segment(const Point2d &point, const Point2d &start,
                         const Point2d &end) {
  if (std::abs(cross_product(start, end, point)) > kGeometryEpsilon) {
    return false;
  }
  return point.x >= std::min(start.x, end.x) - kGeometryEpsilon &&
         point.x <= std::max(start.x, end.x) + kGeometryEpsilon &&
         point.y >= std::min(start.y, end.y) - kGeometryEpsilon &&
         point.y <= std::max(start.y, end.y) + kGeometryEpsilon;
}

int orientation(const Point2d &start, const Point2d &end,
                const Point2d &point) {
  const double cross = cross_product(start, end, point);
  if (std::abs(cross) <= kGeometryEpsilon) {
    return 0;
  }
  return cross > 0.0 ? 1 : -1;
}

bool segments_intersect(const Point2d &first_start, const Point2d &first_end,
                        const Point2d &second_start,
                        const Point2d &second_end) {
  const int first_second_start =
      orientation(first_start, first_end, second_start);
  const int first_second_end = orientation(first_start, first_end, second_end);
  const int second_first_start =
      orientation(second_start, second_end, first_start);
  const int second_first_end = orientation(second_start, second_end, first_end);

  if (first_second_start * first_second_end < 0 &&
      second_first_start * second_first_end < 0) {
    return true;
  }
  return (first_second_start == 0 &&
          point_is_on_segment(second_start, first_start, first_end)) ||
         (first_second_end == 0 &&
          point_is_on_segment(second_end, first_start, first_end)) ||
         (second_first_start == 0 &&
          point_is_on_segment(first_start, second_start, second_end)) ||
         (second_first_end == 0 &&
          point_is_on_segment(first_end, second_start, second_end));
}

bool point_is_inside_or_on_polygon(const Point2d &point,
                                   const std::vector<Point2d> &polygon) {
  bool inside = false;
  for (std::size_t index = 0U; index < polygon.size(); ++index) {
    const auto &start = polygon[index];
    const auto &end = polygon[(index + 1U) % polygon.size()];
    if (point_is_on_segment(point, start, end)) {
      return true;
    }

    const bool crosses_y = (start.y > point.y) != (end.y > point.y);
    if (crosses_y) {
      const double crossing_x =
          start.x + (point.y - start.y) * (end.x - start.x) / (end.y - start.y);
      if (crossing_x >= point.x - kGeometryEpsilon) {
        inside = !inside;
      }
    }
  }
  return inside;
}

bool cell_intersects_polygon(const double center_x, const double center_y,
                             const double resolution,
                             const std::vector<Point2d> &polygon) {
  const double half_resolution = 0.5 * resolution;
  const double min_x = center_x - half_resolution;
  const double max_x = center_x + half_resolution;
  const double min_y = center_y - half_resolution;
  const double max_y = center_y + half_resolution;

  for (const auto &point : polygon) {
    if (point.x >= min_x - kGeometryEpsilon &&
        point.x <= max_x + kGeometryEpsilon &&
        point.y >= min_y - kGeometryEpsilon &&
        point.y <= max_y + kGeometryEpsilon) {
      return true;
    }
  }

  const std::array<Point2d, 4U> corners{{
      {min_x, min_y},
      {max_x, min_y},
      {max_x, max_y},
      {min_x, max_y},
  }};
  for (const auto &corner : corners) {
    if (point_is_inside_or_on_polygon(corner, polygon)) {
      return true;
    }
  }

  for (std::size_t polygon_index = 0U; polygon_index < polygon.size();
       ++polygon_index) {
    const auto &polygon_start = polygon[polygon_index];
    const auto &polygon_end = polygon[(polygon_index + 1U) % polygon.size()];
    for (std::size_t corner_index = 0U; corner_index < corners.size();
         ++corner_index) {
      if (segments_intersect(polygon_start, polygon_end, corners[corner_index],
                             corners[(corner_index + 1U) % corners.size()])) {
        return true;
      }
    }
  }
  return false;
}

} // namespace

ElevatorScopedClearanceResult evaluate_elevator_scoped_clearance(
    nav2_costmap_2d::Costmap2D &costmap,
    const std::vector<geometry_msgs::msg::Point> &footprint, const double x,
    const double y, const double yaw) {
  ElevatorScopedClearanceResult result;
  if (footprint.size() < 3U || !std::isfinite(x) || !std::isfinite(y) ||
      !std::isfinite(yaw) || !std::isfinite(costmap.getResolution()) ||
      costmap.getResolution() <= 0.0 || costmap.getSizeInCellsX() == 0U ||
      costmap.getSizeInCellsY() == 0U) {
    return result;
  }

  const double cos_yaw = std::cos(yaw);
  const double sin_yaw = std::sin(yaw);
  std::vector<Point2d> oriented_footprint;
  oriented_footprint.reserve(footprint.size());
  unsigned int minimum_cell_x = std::numeric_limits<unsigned int>::max();
  unsigned int minimum_cell_y = std::numeric_limits<unsigned int>::max();
  unsigned int maximum_cell_x = 0U;
  unsigned int maximum_cell_y = 0U;
  for (const auto &point : footprint) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y)) {
      return result;
    }

    const double world_x = x + point.x * cos_yaw - point.y * sin_yaw;
    const double world_y = y + point.x * sin_yaw + point.y * cos_yaw;
    if (!std::isfinite(world_x) || !std::isfinite(world_y)) {
      return result;
    }
    if (!costmap.worldToMap(world_x, world_y, result.cell_x, result.cell_y)) {
      result.status = ElevatorScopedClearanceStatus::kOutOfMap;
      return result;
    }
    oriented_footprint.push_back({world_x, world_y});
    minimum_cell_x = std::min(minimum_cell_x, result.cell_x);
    minimum_cell_y = std::min(minimum_cell_y, result.cell_y);
    maximum_cell_x = std::max(maximum_cell_x, result.cell_x);
    maximum_cell_y = std::max(maximum_cell_y, result.cell_y);
  }

  // Expand the cell search by one grid cell. The exact polygon/cell
  // intersection below removes false positives, while the expansion catches
  // lethal cells whose square only touches a footprint edge.
  minimum_cell_x = minimum_cell_x == 0U ? 0U : minimum_cell_x - 1U;
  minimum_cell_y = minimum_cell_y == 0U ? 0U : minimum_cell_y - 1U;
  maximum_cell_x =
      std::min(maximum_cell_x + 1U, costmap.getSizeInCellsX() - 1U);
  maximum_cell_y =
      std::min(maximum_cell_y + 1U, costmap.getSizeInCellsY() - 1U);

  for (unsigned int cell_y = minimum_cell_y; cell_y <= maximum_cell_y;
       ++cell_y) {
    for (unsigned int cell_x = minimum_cell_x; cell_x <= maximum_cell_x;
         ++cell_x) {
      const std::uint8_t cost = costmap.getCost(cell_x, cell_y);
      if (cost < nav2_costmap_2d::LETHAL_OBSTACLE) {
        continue;
      }

      double center_x = 0.0;
      double center_y = 0.0;
      costmap.mapToWorld(cell_x, cell_y, center_x, center_y);
      if (!cell_intersects_polygon(center_x, center_y, costmap.getResolution(),
                                   oriented_footprint)) {
        continue;
      }

      result.cell_x = cell_x;
      result.cell_y = cell_y;
      result.cost = cost;
      result.status = cost == nav2_costmap_2d::NO_INFORMATION
                          ? ElevatorScopedClearanceStatus::kUnknownSpace
                          : ElevatorScopedClearanceStatus::kLethalObstacle;
      return result;
    }
  }

  result.status = ElevatorScopedClearanceStatus::kClear;
  return result;
}

ElevatorScopedPathClearanceResult evaluate_elevator_scoped_path_clearance(
    nav2_costmap_2d::Costmap2D &costmap,
    const std::vector<geometry_msgs::msg::Point> &footprint,
    const std::vector<ElevatorScopedPathSample> &samples,
    const bool include_last_sample) {
  ElevatorScopedPathClearanceResult result;
  if (samples.empty()) {
    return result;
  }
  const std::size_t sample_count =
      include_last_sample ? samples.size() : samples.size() - 1U;
  if (sample_count == 0U) {
    result.clearance.status = ElevatorScopedClearanceStatus::kClear;
    return result;
  }
  for (std::size_t index = 0U; index < sample_count; ++index) {
    const auto &pose = samples[index].pose;
    result.clearance = evaluate_elevator_scoped_clearance(
        costmap, footprint, pose.x, pose.y, pose.yaw);
    result.sample_index = index;
    if (!result.clearance.is_clear()) {
      return result;
    }
  }
  result.clearance.status = ElevatorScopedClearanceStatus::kClear;
  return result;
}

ElevatorScopedPathClearanceResult
evaluate_elevator_scoped_path_clearance_locked(
    nav2_costmap_2d::Costmap2D &costmap,
    const std::vector<geometry_msgs::msg::Point> &footprint,
    const std::vector<ElevatorScopedPathSample> &samples,
    const bool include_last_sample) {
  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(
      *costmap.getMutex());
  return evaluate_elevator_scoped_path_clearance(
      costmap, footprint, samples, include_last_sample);
}

const char *elevator_scoped_clearance_status_name(
    const ElevatorScopedClearanceStatus status) noexcept {
  switch (status) {
  case ElevatorScopedClearanceStatus::kClear:
    return "clear";
  case ElevatorScopedClearanceStatus::kInvalidInput:
    return "invalid_input";
  case ElevatorScopedClearanceStatus::kOutOfMap:
    return "out_of_map";
  case ElevatorScopedClearanceStatus::kLethalObstacle:
    return "lethal_obstacle";
  case ElevatorScopedClearanceStatus::kUnknownSpace:
    return "unknown_space";
  case ElevatorScopedClearanceStatus::kTransformUnavailable:
    return "transform_unavailable";
  case ElevatorScopedClearanceStatus::kCostmapBusy:
    return "costmap_busy";
  }
  return "invalid_input";
}

} // namespace robot_nav_config
