#pragma once

#include <string>
#include <vector>

#include "robot_elevator_manager/elevator_topology.hpp"

namespace robot_elevator_manager
{

enum class ThresholdOccupancy
{
  kInside,
  kOutside,
  kStraddling,
};

struct FootprintClassification
{
  bool ok{false};
  ThresholdOccupancy occupancy{ThresholdOccupancy::kStraddling};
  double minimum_cabin_distance_m{0.0};
  double maximum_cabin_distance_m{0.0};
  double minimum_doorway_projection_m{0.0};
  double maximum_doorway_projection_m{0.0};
  bool within_jamb_clearance{false};
  std::string message;
};

FootprintClassification classify_footprint(
  const DoorThreshold & threshold,
  const std::vector<Point2> & footprint);

std::string to_string(ThresholdOccupancy occupancy);

}  // namespace robot_elevator_manager
