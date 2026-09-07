#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "robot_nav_config/elevator_scoped_path.hpp"

namespace robot_nav_config {

enum class ElevatorScopedClearanceStatus {
  kClear,
  kInvalidInput,
  kOutOfMap,
  kLethalObstacle,
  kUnknownSpace,
  kTransformUnavailable,
  kCostmapBusy,
};

struct ElevatorScopedClearanceResult {
  ElevatorScopedClearanceStatus status{
      ElevatorScopedClearanceStatus::kInvalidInput};
  unsigned int cell_x{0U};
  unsigned int cell_y{0U};
  std::uint8_t cost{0U};

  bool is_clear() const noexcept {
    return status == ElevatorScopedClearanceStatus::kClear;
  }
};

struct ElevatorScopedPathClearanceResult {
  ElevatorScopedClearanceResult clearance;
  std::size_t sample_index{0U};

  bool is_clear() const noexcept { return clearance.is_clear(); }
};

ElevatorScopedClearanceResult evaluate_elevator_scoped_clearance(
    nav2_costmap_2d::Costmap2D &costmap,
    const std::vector<geometry_msgs::msg::Point> &footprint, double x, double y,
    double yaw);

ElevatorScopedPathClearanceResult evaluate_elevator_scoped_path_clearance(
    nav2_costmap_2d::Costmap2D &costmap,
    const std::vector<geometry_msgs::msg::Point> &footprint,
    const std::vector<ElevatorScopedPathSample> &samples,
    bool include_last_sample = true);

// The command-producing control loop needs an authoritative snapshot. It
// waits for the costmap's standard mutex instead of converting ordinary
// update contention into a false obstacle/zero command. Background replans
// remain free to use non-blocking snapshot acquisition.
ElevatorScopedPathClearanceResult
evaluate_elevator_scoped_path_clearance_locked(
    nav2_costmap_2d::Costmap2D &costmap,
    const std::vector<geometry_msgs::msg::Point> &footprint,
    const std::vector<ElevatorScopedPathSample> &samples,
    bool include_last_sample = true);

const char *elevator_scoped_clearance_status_name(
    ElevatorScopedClearanceStatus status) noexcept;

} // namespace robot_nav_config
