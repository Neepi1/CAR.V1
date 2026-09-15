#include "robot_nav_config/navigation_clearance/ranger_clearance_critic.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include "nav2_costmap_2d/cost_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace mppi::critics
{
void RangerClearanceCritic::initialize()
{
  auto get = parameters_handler_->getParamGetter(name_);
  // Geometry is commissioned at startup, not partially live-updated.
  get(stop_half_x_, "stop_half_x", 0.47, ParameterType::Static);
  get(stop_half_y_, "stop_half_y", 0.36, ParameterType::Static);
  get(preferred_margin_, "preferred_margin", 0.05, ParameterType::Static);
  get(cost_weight_, "cost_weight", 30.0, ParameterType::Static);
  get(near_goal_distance_, "near_goal_distance", 0.75, ParameterType::Static);
  if (!std::isfinite(stop_half_x_) || !std::isfinite(stop_half_y_) ||
    !std::isfinite(preferred_margin_) || !std::isfinite(cost_weight_) ||
    !std::isfinite(near_goal_distance_) || stop_half_x_ <= 0.0 ||
    stop_half_y_ <= 0.0 || preferred_margin_ <= 0.0 || cost_weight_ < 0.0 ||
    near_goal_distance_ < 0.0)
  {
    throw std::invalid_argument("RangerClearanceCritic: invalid clearance parameters");
  }
  RCLCPP_INFO(logger_, "Ranger planning preference: half-envelope %.2f / %.2f m; "
    "weight %.1f; native footprint unchanged", stop_half_x_ + preferred_margin_,
    stop_half_y_ + preferred_margin_, cost_weight_);
}

void RangerClearanceCritic::index_obstacles()
{
  // One read of the current costmap per scoring pass. No subscriptions, TF,
  // costmap writes or stale cache. The controller holds the costmap mutex.
  // One-metre bins bound work to nearby occupied cells, not the full grid for
  // each of the 1200 x 48 trajectory poses. Reuse bucket allocations.
  origin_x_ = costmap_->getOriginX();
  origin_y_ = costmap_->getOriginY();
  const double resolution = costmap_->getResolution();
  cell_half_ = resolution * 0.5;
  columns_ = std::max(1, static_cast<int>(std::ceil(costmap_->getSizeInCellsX() * resolution)));
  rows_ = std::max(1, static_cast<int>(std::ceil(costmap_->getSizeInCellsY() * resolution)));
  buckets_.resize(static_cast<std::size_t>(columns_ * rows_));
  for (auto & bucket : buckets_) {bucket.clear();}
  for (unsigned int my = 0; my < costmap_->getSizeInCellsY(); ++my) {
    for (unsigned int mx = 0; mx < costmap_->getSizeInCellsX(); ++mx) {
      // Inflation 1..253 is NOT a second physical obstacle. Unknown handling
      // and out-of-map collision decisions remain with the native critic.
      if (costmap_->getCost(mx, my) != nav2_costmap_2d::LETHAL_OBSTACLE) {continue;}
      double x, y;
      costmap_->mapToWorld(mx, my, x, y);
      const int bx = std::clamp(static_cast<int>(x - origin_x_), 0, columns_ - 1);
      const int by = std::clamp(static_cast<int>(y - origin_y_), 0, rows_ - 1);
      buckets_[by * columns_ + bx].push_back({x, y});
    }
  }
}

double RangerClearanceCritic::intrusion(double x, double y, double yaw) const
{
  if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(yaw)) {return 2.0;}
  const double c = std::cos(yaw), s = std::sin(yaw);
  // Conservatively include the occupied square, not just its cell centre.
  const double cell_projection = cell_half_ * (std::abs(c) + std::abs(s));
  const double half_x = stop_half_x_ + preferred_margin_ + cell_projection;
  const double half_y = stop_half_y_ + preferred_margin_ + cell_projection;
  const double reach_x = std::abs(c) * half_x + std::abs(s) * half_y;
  const double reach_y = std::abs(s) * half_x + std::abs(c) * half_y;
  const int min_x = std::max(0, static_cast<int>(std::floor(x - reach_x - origin_x_)));
  const int max_x = std::min(columns_ - 1,
    static_cast<int>(std::floor(x + reach_x - origin_x_)));
  const int min_y = std::max(0, static_cast<int>(std::floor(y - reach_y - origin_y_)));
  const int max_y = std::min(rows_ - 1,
    static_cast<int>(std::floor(y + reach_y - origin_y_)));
  double worst = 0.0;
  for (int by = min_y; by <= max_y; ++by) {
    for (int bx = min_x; bx <= max_x; ++bx) {
      for (const auto & point : buckets_[by * columns_ + bx]) {
        const double dx = point.x - x, dy = point.y - y;
        const double excess = std::max(
          (std::abs(c * dx + s * dy) - half_x) / preferred_margin_,
          (std::abs(-s * dx + c * dy) - half_y) / preferred_margin_);
        worst = std::max(worst, std::clamp(-excess, 0.0, 2.0));
        if (worst >= 2.0) {return worst;}
      }
    }
  }
  return worst;
}

void RangerClearanceCritic::score(CriticData & data)
{
  if (!enabled_ || data.fail_flag || data.trajectories.x.shape()[1] == 0U) {return;}
  // Preserve commissioned near-goal MPPI / native terminal handoff behavior.
  // The path is in the same local-costmap frame as state and trajectories.
  const auto n = data.path.x.size();
  if (n && std::hypot(data.state.pose.pose.position.x - data.path.x(n - 1),
    data.state.pose.pose.position.y - data.path.y(n - 1)) <= near_goal_distance_)
  {
    return;
  }
  index_obstacles();
  const auto steps = data.trajectories.x.shape()[1];
  for (std::size_t trajectory = 0; trajectory < data.costs.size(); ++trajectory) {
    double penalty = 0.0;
    for (std::size_t step = 0; step < steps; ++step) {
      const double depth = intrusion(data.trajectories.x(trajectory, step),
        data.trajectories.y(trajectory, step), data.trajectories.yaws(trajectory, step));
      penalty += depth * depth;
    }
    // Finite, horizon-normalized preference: faster escape scores below
    // remaining or moving further inward. Never set fail_flag or zero a Twist.
    data.costs(trajectory) += static_cast<float>(cost_weight_ * penalty / steps);
  }
}
}  // namespace mppi::critics

PLUGINLIB_EXPORT_CLASS(mppi::critics::RangerClearanceCritic, mppi::critics::CriticFunction)
