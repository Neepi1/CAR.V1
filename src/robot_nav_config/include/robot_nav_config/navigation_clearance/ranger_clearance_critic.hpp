#pragma once

#include <vector>
#include "nav2_mppi_controller/critic_function.hpp"

namespace mppi::critics
{
// MPPI's loader prefixes critic IDs with mppi::critics::. This plugin adds a
// preference only; native ObstaclesCritic retains physical collision ownership.
class RangerClearanceCritic : public CriticFunction
{
public:
  void initialize() override;
  void score(CriticData & data) override;

private:
  struct Point {double x; double y;};
  void index_obstacles();
  double intrusion(double x, double y, double yaw) const;
  std::vector<std::vector<Point>> buckets_;
  int columns_{0};
  int rows_{0};
  double origin_x_{0.0};
  double origin_y_{0.0};
  double cell_half_{0.0};
  double stop_half_x_{0.47};
  double stop_half_y_{0.36};
  double preferred_margin_{0.05};
  double cost_weight_{30.0};
  double near_goal_distance_{0.75};
};
}  // namespace mppi::critics
