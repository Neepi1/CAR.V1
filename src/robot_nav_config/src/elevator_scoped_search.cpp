#include "robot_nav_config/elevator_scoped_search.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <iterator>
#include <limits>
#include <optional>
#include <queue>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

#include "nav2_costmap_2d/cost_values.hpp"

namespace robot_nav_config {
namespace {

enum class SearchMotion : std::uint8_t {
  kNone = 0U,
  kForward = 1U,
  kReverse = 2U,
  kLateral = 3U,
  kRotate = 4U,
};

constexpr int kHeadingCount = 16;
constexpr double kRotationObjectiveCostPerRadian = 0.10;
constexpr double kHeuristicWeight = 2.0;

// Exact grid-aligned motion directions for a 16-heading omnidirectional
// lattice. Each pose yaw is derived from the same integer vector used by its
// forward primitive, so the controller never receives a path whose geometric
// translation disagrees with its advertised forward/lateral axis.
constexpr std::array<std::pair<int, int>, kHeadingCount> kHeadingForwardDeltas{{
    {1, 0},
    {2, 1},
    {1, 1},
    {1, 2},
    {0, 1},
    {-1, 2},
    {-1, 1},
    {-2, 1},
    {-1, 0},
    {-2, -1},
    {-1, -1},
    {-1, -2},
    {0, -1},
    {1, -2},
    {1, -1},
    {2, -1},
}};

int normalized_heading_index(const int index) noexcept {
  const int remainder = index % kHeadingCount;
  return remainder < 0 ? remainder + kHeadingCount : remainder;
}

double relative_heading_yaw(const int heading_index) noexcept {
  const auto &delta =
      kHeadingForwardDeltas[normalized_heading_index(heading_index)];
  return std::atan2(static_cast<double>(delta.second),
                    static_cast<double>(delta.first));
}

struct SearchKey {
  int forward_index{0};
  int lateral_index{0};
  int heading_index{0};
  SearchMotion last_motion{SearchMotion::kNone};

  bool operator==(const SearchKey &other) const noexcept {
    return forward_index == other.forward_index &&
           lateral_index == other.lateral_index &&
           heading_index == other.heading_index &&
           last_motion == other.last_motion;
  }
};

struct SearchKeyHash {
  std::size_t operator()(const SearchKey &key) const noexcept {
    const auto first = static_cast<std::uint64_t>(
        static_cast<std::uint32_t>(key.forward_index));
    const auto second = static_cast<std::uint64_t>(
        static_cast<std::uint32_t>(key.lateral_index));
    const auto motion = static_cast<std::uint64_t>(key.last_motion);
    const auto heading = static_cast<std::uint64_t>(
        static_cast<std::uint32_t>(key.heading_index));
    std::uint64_t mixed = first * 0x9e3779b185ebca87ULL;
    mixed ^= second + 0x9e3779b97f4a7c15ULL + (mixed << 6U) + (mixed >> 2U);
    mixed ^= heading + 0x94d049bb133111ebULL + (mixed << 6U) + (mixed >> 2U);
    mixed ^= motion + 0x517cc1b727220a95ULL + (mixed << 6U) + (mixed >> 2U);
    return static_cast<std::size_t>(mixed);
  }
};

struct GeometricEdgeKey {
  int from_forward{0};
  int from_lateral{0};
  int from_heading{0};
  int to_forward{0};
  int to_lateral{0};
  int to_heading{0};

  bool operator==(const GeometricEdgeKey &other) const noexcept {
    return from_forward == other.from_forward &&
           from_lateral == other.from_lateral &&
           from_heading == other.from_heading &&
           to_forward == other.to_forward && to_lateral == other.to_lateral &&
           to_heading == other.to_heading;
  }
};

struct GeometricEdgeKeyHash {
  std::size_t operator()(const GeometricEdgeKey &key) const noexcept {
    std::size_t seed = 0U;
    const auto mix = [&seed](const int value) {
      seed ^=
          std::hash<int>{}(value) + 0x9e3779b9U + (seed << 6U) + (seed >> 2U);
    };
    mix(key.from_forward);
    mix(key.from_lateral);
    mix(key.from_heading);
    mix(key.to_forward);
    mix(key.to_lateral);
    mix(key.to_heading);
    return seed;
  }
};

GeometricEdgeKey
make_geometric_edge_key(const int from_forward, const int from_lateral,
                        const int from_heading, const int to_forward,
                        const int to_lateral, const int to_heading) {
  const auto from = std::make_tuple(from_forward, from_lateral,
                                    normalized_heading_index(from_heading));
  const auto to = std::make_tuple(to_forward, to_lateral,
                                  normalized_heading_index(to_heading));
  if (to < from) {
    return {
        to_forward,   to_lateral,   normalized_heading_index(to_heading),
        from_forward, from_lateral, normalized_heading_index(from_heading),
    };
  }
  return {
      from_forward, from_lateral, normalized_heading_index(from_heading),
      to_forward,   to_lateral,   normalized_heading_index(to_heading),
  };
}

struct GeometricPoseKey {
  int forward{0};
  int lateral{0};
  int heading{0};

  bool operator==(const GeometricPoseKey &other) const noexcept {
    return forward == other.forward && lateral == other.lateral &&
           heading == other.heading;
  }
};

struct GeometricPoseKeyHash {
  std::size_t operator()(const GeometricPoseKey &key) const noexcept {
    std::size_t seed = std::hash<int>{}(key.forward);
    seed ^= std::hash<int>{}(key.lateral) + 0x9e3779b9U + (seed << 6U) +
            (seed >> 2U);
    seed ^= std::hash<int>{}(key.heading) + 0x9e3779b9U + (seed << 6U) +
            (seed >> 2U);
    return seed;
  }
};

struct SearchNode {
  SearchKey key;
  double cost_from_start{0.0};
  std::size_t parent{std::numeric_limits<std::size_t>::max()};
  ElevatorScopedMotionPhase phase{ElevatorScopedMotionPhase::kForward};
};

struct QueueEntry {
  double estimated_total_cost{0.0};
  std::uint64_t insertion_order{0U};
  std::size_t node_index{0U};
};

struct QueueEntryGreater {
  bool operator()(const QueueEntry &left,
                  const QueueEntry &right) const noexcept {
    if (left.estimated_total_cost != right.estimated_total_cost) {
      return left.estimated_total_cost > right.estimated_total_cost;
    }
    return left.insertion_order > right.insertion_order;
  }
};

struct ValidationResult {
  bool clear{false};
  ElevatorScopedClearanceResult blocking;
};

struct DistanceFieldEntry {
  double distance_m{0.0};
  std::size_t cell_index{0U};
};

struct DistanceFieldEntryGreater {
  bool operator()(const DistanceFieldEntry &left,
                  const DistanceFieldEntry &right) const noexcept {
    return left.distance_m > right.distance_m;
  }
};

struct ObstacleDistanceField {
  unsigned int width{0U};
  unsigned int height{0U};
  std::vector<double> distance_m;
  std::vector<bool> traversable;

  double distance_at(nav2_costmap_2d::Costmap2D &costmap,
                     const ElevatorScopedPose &pose) const {
    unsigned int cell_x = 0U;
    unsigned int cell_y = 0U;
    if (!costmap.worldToMap(pose.x, pose.y, cell_x, cell_y) ||
        cell_x >= width || cell_y >= height) {
      return std::numeric_limits<double>::infinity();
    }
    return distance_m[static_cast<std::size_t>(cell_y) * width + cell_x];
  }
};

double footprint_inscribed_radius(
    const std::vector<geometry_msgs::msg::Point> &footprint) {
  double radius = std::numeric_limits<double>::infinity();
  for (std::size_t index = 0U; index < footprint.size(); ++index) {
    const auto &from = footprint[index];
    const auto &to = footprint[(index + 1U) % footprint.size()];
    const double edge_x = to.x - from.x;
    const double edge_y = to.y - from.y;
    const double edge_length = std::hypot(edge_x, edge_y);
    if (!std::isfinite(edge_length) || edge_length <= 1.0e-12) {
      continue;
    }
    const double distance =
        std::abs(from.x * to.y - from.y * to.x) / edge_length;
    if (std::isfinite(distance)) {
      radius = std::min(radius, distance);
    }
  }
  return std::isfinite(radius) ? radius : 0.0;
}

ObstacleDistanceField make_obstacle_distance_field(
    nav2_costmap_2d::Costmap2D &costmap,
    const std::vector<geometry_msgs::msg::Point> &footprint,
    const ElevatorScopedPose &goal,
    const std::chrono::steady_clock::time_point &deadline,
    bool &time_limit_reached) {
  ObstacleDistanceField field;
  field.width = costmap.getSizeInCellsX();
  field.height = costmap.getSizeInCellsY();
  field.distance_m.assign(static_cast<std::size_t>(field.width) * field.height,
                          std::numeric_limits<double>::infinity());
  field.traversable.assign(field.distance_m.size(), true);

  const auto index_of = [&field](const unsigned int x, const unsigned int y) {
    return static_cast<std::size_t>(y) * field.width + x;
  };
  const double resolution = costmap.getResolution();
  const double inscribed_radius = footprint_inscribed_radius(footprint);
  const int cell_radius =
      static_cast<int>(std::ceil(inscribed_radius / resolution));
  std::vector<std::pair<int, int>> blocked_offsets;
  for (int delta_y = -cell_radius; delta_y <= cell_radius; ++delta_y) {
    for (int delta_x = -cell_radius; delta_x <= cell_radius; ++delta_x) {
      if (std::hypot(static_cast<double>(delta_x),
                     static_cast<double>(delta_y)) *
              resolution <=
          inscribed_radius + 1.0e-9) {
        blocked_offsets.emplace_back(delta_x, delta_y);
      }
    }
  }
  for (unsigned int obstacle_y = 0U; obstacle_y < field.height; ++obstacle_y) {
    if (std::chrono::steady_clock::now() >= deadline) {
      time_limit_reached = true;
      return field;
    }
    for (unsigned int obstacle_x = 0U; obstacle_x < field.width; ++obstacle_x) {
      if (costmap.getCost(obstacle_x, obstacle_y) <
          nav2_costmap_2d::LETHAL_OBSTACLE) {
        continue;
      }
      for (const auto &offset : blocked_offsets) {
        const int blocked_x = static_cast<int>(obstacle_x) + offset.first;
        const int blocked_y = static_cast<int>(obstacle_y) + offset.second;
        if (blocked_x < 0 || blocked_y < 0 ||
            blocked_x >= static_cast<int>(field.width) ||
            blocked_y >= static_cast<int>(field.height)) {
          continue;
        }
        field.traversable[index_of(static_cast<unsigned int>(blocked_x),
                                   static_cast<unsigned int>(blocked_y))] =
            false;
      }
    }
  }

  unsigned int goal_x = 0U;
  unsigned int goal_y = 0U;
  if (!costmap.worldToMap(goal.x, goal.y, goal_x, goal_y) ||
      !field.traversable[index_of(goal_x, goal_y)]) {
    return field;
  }

  const std::size_t goal_index = index_of(goal_x, goal_y);
  field.distance_m[goal_index] = 0.0;
  std::priority_queue<DistanceFieldEntry, std::vector<DistanceFieldEntry>,
                      DistanceFieldEntryGreater>
      open;
  open.push({0.0, goal_index});
  const std::array<std::pair<int, int>, 8U> neighbors{{
      {-1, -1},
      {0, -1},
      {1, -1},
      {-1, 0},
      {1, 0},
      {-1, 1},
      {0, 1},
      {1, 1},
  }};

  std::size_t wavefront_iterations = 0U;
  while (!open.empty()) {
    if ((wavefront_iterations++ & 0xffU) == 0U &&
        std::chrono::steady_clock::now() >= deadline) {
      time_limit_reached = true;
      return field;
    }
    const auto current = open.top();
    open.pop();
    if (current.cell_index >= field.distance_m.size() ||
        current.distance_m > field.distance_m[current.cell_index] + 1.0e-12) {
      continue;
    }
    const unsigned int current_x =
        static_cast<unsigned int>(current.cell_index % field.width);
    const unsigned int current_y =
        static_cast<unsigned int>(current.cell_index / field.width);
    for (const auto &neighbor : neighbors) {
      const int next_x = static_cast<int>(current_x) + neighbor.first;
      const int next_y = static_cast<int>(current_y) + neighbor.second;
      if (next_x < 0 || next_y < 0 || next_x >= static_cast<int>(field.width) ||
          next_y >= static_cast<int>(field.height)) {
        continue;
      }
      const auto map_x = static_cast<unsigned int>(next_x);
      const auto map_y = static_cast<unsigned int>(next_y);
      if (!field.traversable[index_of(map_x, map_y)]) {
        continue;
      }
      const std::size_t next_index = index_of(map_x, map_y);
      const bool diagonal = neighbor.first != 0 && neighbor.second != 0;
      const double candidate =
          current.distance_m + resolution * (diagonal ? std::sqrt(2.0) : 1.0);
      if (candidate >= field.distance_m[next_index] - 1.0e-12) {
        continue;
      }
      field.distance_m[next_index] = candidate;
      open.push({candidate, next_index});
    }
  }
  return field;
}

bool search_parameters_are_valid(
    const ElevatorScopedSearchParameters &parameters) {
  return std::isfinite(parameters.sampling.max_distance_m) &&
         std::isfinite(parameters.sampling.translation_step_m) &&
         std::isfinite(parameters.sampling.rotation_step_rad) &&
         std::isfinite(parameters.search_grid_step_m) &&
         std::isfinite(parameters.search_radius_m) &&
         std::isfinite(parameters.goal_connect_distance_m) &&
         std::isfinite(parameters.forward_cost) &&
         std::isfinite(parameters.reverse_cost) &&
         std::isfinite(parameters.lateral_cost) &&
         std::isfinite(parameters.mode_switch_cost) &&
         std::isfinite(parameters.soft_cost_weight) &&
         std::isfinite(parameters.clearance_preference_cost_ratio) &&
         std::isfinite(parameters.maximum_search_time_sec) &&
         parameters.sampling.max_distance_m > 0.0 &&
         parameters.sampling.translation_step_m > 0.0 &&
         parameters.sampling.rotation_step_rad > 0.0 &&
         parameters.search_grid_step_m > 0.0 &&
         parameters.search_radius_m > 0.0 &&
         parameters.goal_connect_distance_m > 0.0 &&
         parameters.maximum_expansions > 0U && parameters.forward_cost > 0.0 &&
         parameters.reverse_cost > 0.0 && parameters.lateral_cost > 0.0 &&
         parameters.mode_switch_cost >= 0.0 &&
         parameters.soft_cost_weight >= 0.0 &&
         parameters.clearance_preference_cost_ratio >= 1.0 &&
         parameters.maximum_search_time_sec > 0.0;
}

ValidationResult
validate_samples(nav2_costmap_2d::Costmap2D &costmap,
                 const std::vector<geometry_msgs::msg::Point> &footprint,
                 const std::vector<ElevatorScopedPathSample> &samples) {
  ValidationResult result;
  const auto clearance =
      evaluate_elevator_scoped_path_clearance(costmap, footprint, samples);
  result.clear = clearance.is_clear();
  result.blocking = clearance.clearance;
  return result;
}

ValidationResult validate_interior_samples(
    nav2_costmap_2d::Costmap2D &costmap,
    const std::vector<geometry_msgs::msg::Point> &footprint,
    const std::vector<ElevatorScopedPathSample> &samples) {
  ValidationResult result;
  if (samples.size() <= 1U) {
    result.clear = true;
    result.blocking.status = ElevatorScopedClearanceStatus::kClear;
    return result;
  }
  const auto clearance = evaluate_elevator_scoped_path_clearance(
      costmap, footprint, samples, false);
  result.clear = clearance.is_clear();
  result.blocking = clearance.clearance;
  return result;
}

ElevatorScopedPose grid_pose(const ElevatorScopedPose &start,
                             const double grid_step, const int forward_index,
                             const int lateral_index, const int heading_index) {
  const double forward = static_cast<double>(forward_index) * grid_step;
  const double lateral = static_cast<double>(lateral_index) * grid_step;
  const double cos_yaw = std::cos(start.yaw);
  const double sin_yaw = std::sin(start.yaw);
  return {
      start.x + cos_yaw * forward - sin_yaw * lateral,
      start.y + sin_yaw * forward + cos_yaw * lateral,
      normalize_elevator_scoped_angle(start.yaw +
                                      relative_heading_yaw(heading_index)),
  };
}

std::vector<ElevatorScopedPathSample>
sample_translation(const ElevatorScopedPose &from, const ElevatorScopedPose &to,
                   const ElevatorScopedMotionPhase phase,
                   const double sample_step) {
  std::vector<ElevatorScopedPathSample> samples;
  const double distance = std::hypot(to.x - from.x, to.y - from.y);
  if (!std::isfinite(distance) || distance <= 1.0e-12) {
    return samples;
  }
  const auto count =
      static_cast<std::size_t>(std::ceil(distance / sample_step));
  samples.reserve(count);
  for (std::size_t index = 1U; index <= count; ++index) {
    const double ratio =
        static_cast<double>(index) / static_cast<double>(count);
    samples.push_back({
        phase,
        ElevatorScopedPose{
            from.x + ratio * (to.x - from.x),
            from.y + ratio * (to.y - from.y),
            to.yaw,
        },
    });
  }
  samples.back().pose = to;
  return samples;
}

std::optional<ElevatorScopedPath>
make_spin_translate_spin_path(const ElevatorScopedPose &start,
                              const ElevatorScopedPose &goal,
                              const ElevatorScopedPathParameters &sampling) {
  if (!elevator_scoped_pose_is_finite(start) ||
      !elevator_scoped_pose_is_finite(goal)) {
    return std::nullopt;
  }
  const double dx = goal.x - start.x;
  const double dy = goal.y - start.y;
  const double distance = std::hypot(dx, dy);
  if (!std::isfinite(distance) || distance > sampling.max_distance_m) {
    return std::nullopt;
  }
  if (distance <= 1.0e-12) {
    return make_elevator_scoped_path(start, goal, sampling);
  }

  const double travel_yaw = std::atan2(dy, dx);
  const ElevatorScopedPose aligned_start{start.x, start.y, travel_yaw};
  const ElevatorScopedPose aligned_goal{goal.x, goal.y, travel_yaw};
  const auto startup_rotation =
      make_elevator_scoped_path(start, aligned_start, sampling);
  const auto final_rotation =
      make_elevator_scoped_path(aligned_goal, goal, sampling);
  if (!startup_rotation || !final_rotation) {
    return std::nullopt;
  }

  ElevatorScopedPath path;
  path.forward_m = distance;
  path.samples = startup_rotation->samples;
  auto translation = sample_translation(aligned_start, aligned_goal,
                                        ElevatorScopedMotionPhase::kForward,
                                        sampling.translation_step_m);
  path.samples.insert(path.samples.end(), translation.begin(),
                      translation.end());
  if (final_rotation->samples.size() > 1U) {
    path.samples.insert(path.samples.end(),
                        std::next(final_rotation->samples.begin()),
                        final_rotation->samples.end());
  }
  if (path.samples.empty() || path.samples.size() > sampling.maximum_samples) {
    return std::nullopt;
  }
  path.samples.back().pose = goal;
  return path;
}

ElevatorScopedMotionPhase phase_for_motion(const SearchMotion motion) {
  switch (motion) {
  case SearchMotion::kForward:
    return ElevatorScopedMotionPhase::kForward;
  case SearchMotion::kReverse:
    return ElevatorScopedMotionPhase::kReverse;
  case SearchMotion::kLateral:
    return ElevatorScopedMotionPhase::kLateral;
  case SearchMotion::kRotate:
    return ElevatorScopedMotionPhase::kYaw;
  case SearchMotion::kNone:
    return ElevatorScopedMotionPhase::kYaw;
  }
  return ElevatorScopedMotionPhase::kYaw;
}

double motion_cost(const SearchMotion motion,
                   const ElevatorScopedSearchParameters &parameters) {
  switch (motion) {
  case SearchMotion::kForward:
    return parameters.forward_cost;
  case SearchMotion::kReverse:
    return parameters.reverse_cost;
  case SearchMotion::kLateral:
    return parameters.lateral_cost;
  case SearchMotion::kRotate:
    return 0.0;
  case SearchMotion::kNone:
    return 0.0;
  }
  return 0.0;
}

double phase_cost(const ElevatorScopedMotionPhase phase,
                  const ElevatorScopedSearchParameters &parameters) {
  switch (phase) {
  case ElevatorScopedMotionPhase::kForward:
    return parameters.forward_cost;
  case ElevatorScopedMotionPhase::kReverse:
    return parameters.reverse_cost;
  case ElevatorScopedMotionPhase::kLateral:
    return parameters.lateral_cost;
  case ElevatorScopedMotionPhase::kYaw:
    return 0.0;
  }
  return 0.0;
}

std::uint8_t center_cost(nav2_costmap_2d::Costmap2D &costmap,
                         const ElevatorScopedPose &pose) {
  unsigned int cell_x = 0U;
  unsigned int cell_y = 0U;
  if (!costmap.worldToMap(pose.x, pose.y, cell_x, cell_y)) {
    return nav2_costmap_2d::NO_INFORMATION;
  }
  return costmap.getCost(cell_x, cell_y);
}

double heuristic(nav2_costmap_2d::Costmap2D &costmap,
                 const ElevatorScopedPose &pose, const ElevatorScopedPose &goal,
                 const ElevatorScopedSearchParameters &parameters,
                 const ObstacleDistanceField &obstacle_distance) {
  const double direct_distance = std::hypot(goal.x - pose.x, goal.y - pose.y);
  const double routed_distance = obstacle_distance.distance_at(costmap, pose);
  return (std::isfinite(routed_distance) ? routed_distance : direct_distance) *
         std::min(parameters.forward_cost, parameters.lateral_cost);
}

std::vector<ElevatorScopedPathSample> make_axis_connection(
    const ElevatorScopedPose &from, const ElevatorScopedPose &goal,
    const double translation_yaw, const double sample_step,
    const bool lateral_first, const ElevatorScopedPathParameters &sampling) {
  const double cos_yaw = std::cos(translation_yaw);
  const double sin_yaw = std::sin(translation_yaw);
  const double dx = goal.x - from.x;
  const double dy = goal.y - from.y;
  const double forward = cos_yaw * dx + sin_yaw * dy;
  const double lateral = -sin_yaw * dx + cos_yaw * dy;
  const ElevatorScopedPose after_lateral{
      from.x - sin_yaw * lateral,
      from.y + cos_yaw * lateral,
      translation_yaw,
  };
  const ElevatorScopedPose after_forward{
      from.x + cos_yaw * forward,
      from.y + sin_yaw * forward,
      translation_yaw,
  };
  const ElevatorScopedPose translated_goal{goal.x, goal.y, translation_yaw};
  const auto forward_phase = forward < 0.0
                                 ? ElevatorScopedMotionPhase::kReverse
                                 : ElevatorScopedMotionPhase::kForward;

  std::vector<ElevatorScopedPathSample> result;
  const auto append = [&result](std::vector<ElevatorScopedPathSample> samples) {
    result.insert(result.end(), samples.begin(), samples.end());
  };
  if (lateral_first) {
    append(sample_translation(
        from, after_lateral, ElevatorScopedMotionPhase::kLateral, sample_step));
    append(sample_translation(after_lateral, translated_goal, forward_phase,
                              sample_step));
  } else {
    append(sample_translation(from, after_forward, forward_phase, sample_step));
    append(sample_translation(after_forward, translated_goal,
                              ElevatorScopedMotionPhase::kLateral,
                              sample_step));
  }

  const auto rotation =
      make_elevator_scoped_path(translated_goal, goal, sampling);
  if (rotation) {
    const auto first = rotation->samples.size() > 1U
                           ? std::next(rotation->samples.begin())
                           : rotation->samples.end();
    result.insert(result.end(), first, rotation->samples.end());
  }
  if (result.empty()) {
    result.push_back({ElevatorScopedMotionPhase::kYaw, goal});
  }
  result.back().pose = goal;
  return result;
}

std::optional<ElevatorScopedPath> make_commissioned_axis_sequence_path(
    const ElevatorScopedPose &start, const ElevatorScopedPose &goal,
    const ElevatorScopedPathParameters &sampling,
    const bool lateral_first) {
  if (!elevator_scoped_pose_is_finite(start) ||
      !elevator_scoped_pose_is_finite(goal)) {
    return std::nullopt;
  }
  const double distance = std::hypot(goal.x - start.x, goal.y - start.y);
  if (!std::isfinite(distance) || distance > sampling.max_distance_m) {
    return std::nullopt;
  }

  const ElevatorScopedPose aligned_start{start.x, start.y, goal.yaw};
  const auto rotation =
      make_elevator_scoped_path(start, aligned_start, sampling);
  if (!rotation) {
    return std::nullopt;
  }

  ElevatorScopedPath path;
  path.samples = rotation->samples;
  auto translation = make_axis_connection(
      aligned_start, goal, goal.yaw, sampling.translation_step_m,
      lateral_first, sampling);
  path.samples.insert(path.samples.end(), translation.begin(),
                      translation.end());
  const double dx = goal.x - start.x;
  const double dy = goal.y - start.y;
  path.forward_m = std::cos(goal.yaw) * dx + std::sin(goal.yaw) * dy;
  path.lateral_m = -std::sin(goal.yaw) * dx + std::cos(goal.yaw) * dy;
  if (path.samples.empty() || path.samples.size() > sampling.maximum_samples) {
    return std::nullopt;
  }
  path.samples.back().pose = goal;
  return path;
}

std::optional<std::vector<ElevatorScopedPathSample>>
clear_goal_connection(nav2_costmap_2d::Costmap2D &costmap,
                      const std::vector<geometry_msgs::msg::Point> &footprint,
                      const ElevatorScopedPose &from,
                      const ElevatorScopedPose &goal,
                      const ElevatorScopedSearchParameters &parameters) {
  for (const bool lateral_first : {false, true}) {
    auto samples = make_axis_connection(from, goal, from.yaw,
                                        parameters.sampling.translation_step_m,
                                        lateral_first, parameters.sampling);
    if (validate_samples(costmap, footprint, samples).clear) {
      return samples;
    }
  }
  return std::nullopt;
}

double path_length(const ElevatorScopedPath &path) {
  double length = 0.0;
  for (std::size_t index = 1U; index < path.samples.size(); ++index) {
    length += std::hypot(
        path.samples[index].pose.x - path.samples[index - 1U].pose.x,
        path.samples[index].pose.y - path.samples[index - 1U].pose.y);
  }
  return length;
}

bool path_has_soft_cost(nav2_costmap_2d::Costmap2D &costmap,
                        const ElevatorScopedPath &path) {
  return std::any_of(path.samples.begin(), path.samples.end(),
                     [&costmap](const ElevatorScopedPathSample &sample) {
                       const auto cost = center_cost(costmap, sample.pose);
                       return cost > nav2_costmap_2d::FREE_SPACE &&
                              cost < nav2_costmap_2d::LETHAL_OBSTACLE;
                     });
}

double path_objective(nav2_costmap_2d::Costmap2D &costmap,
                      const ElevatorScopedPose &start,
                      const ElevatorScopedPath &path,
                      const ElevatorScopedSearchParameters &parameters) {
  double objective = 0.0;
  ElevatorScopedPose previous_pose = start;
  std::optional<ElevatorScopedMotionPhase> previous_motion;
  for (const auto &sample : path.samples) {
    const double distance = std::hypot(sample.pose.x - previous_pose.x,
                                       sample.pose.y - previous_pose.y);
    if (sample.phase == ElevatorScopedMotionPhase::kYaw) {
      objective += std::abs(normalize_elevator_scoped_angle(
                       sample.pose.yaw - previous_pose.yaw)) *
                   kRotationObjectiveCostPerRadian * parameters.forward_cost;
    }
    if (sample.phase != ElevatorScopedMotionPhase::kYaw) {
      if (previous_motion && *previous_motion != sample.phase) {
        objective += parameters.mode_switch_cost;
      }
      previous_motion = sample.phase;
    }
    const auto cost = center_cost(costmap, sample.pose);
    const double normalized_soft_cost =
        cost < nav2_costmap_2d::LETHAL_OBSTACLE
            ? static_cast<double>(cost) /
                  static_cast<double>(
                      nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
            : 1.0;
    objective +=
        distance * (phase_cost(sample.phase, parameters) +
                    parameters.soft_cost_weight * normalized_soft_cost);
    previous_pose = sample.pose;
  }
  return objective;
}

double path_soft_cost_exposure(nav2_costmap_2d::Costmap2D &costmap,
                               const ElevatorScopedPose &start,
                               const ElevatorScopedPath &path) {
  double exposure = 0.0;
  ElevatorScopedPose previous_pose = start;
  for (const auto &sample : path.samples) {
    const double distance = std::hypot(sample.pose.x - previous_pose.x,
                                       sample.pose.y - previous_pose.y);
    const auto cost = center_cost(costmap, sample.pose);
    if (cost < nav2_costmap_2d::LETHAL_OBSTACLE) {
      exposure +=
          distance * static_cast<double>(cost) /
          static_cast<double>(nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
    }
    previous_pose = sample.pose;
  }
  return exposure;
}

std::size_t count_mode_switches(const ElevatorScopedPath &path) {
  std::optional<ElevatorScopedMotionPhase> previous;
  std::size_t switches = 0U;
  for (std::size_t index = 1U; index < path.samples.size(); ++index) {
    const auto &from = path.samples[index - 1U].pose;
    const auto &sample = path.samples[index];
    const bool moved =
        std::hypot(sample.pose.x - from.x, sample.pose.y - from.y) > 1.0e-9;
    const bool turned =
        std::abs(normalize_elevator_scoped_angle(sample.pose.yaw - from.yaw)) >
        1.0e-9;
    if (!moved && !turned) {
      continue;
    }
    if (previous && *previous != sample.phase) {
      ++switches;
    }
    previous = sample.phase;
  }
  return switches;
}

} // namespace

ElevatorScopedSearchResult search_elevator_scoped_path(
    nav2_costmap_2d::Costmap2D &costmap,
    const std::vector<geometry_msgs::msg::Point> &footprint,
    const ElevatorScopedPose &start, const ElevatorScopedPose &goal,
    const ElevatorScopedSearchParameters &parameters) {
  ElevatorScopedSearchResult result;
  const double request_distance =
      std::hypot(goal.x - start.x, goal.y - start.y);
  if (!elevator_scoped_pose_is_finite(start) ||
      !elevator_scoped_pose_is_finite(goal) ||
      !search_parameters_are_valid(parameters) || footprint.size() < 3U ||
      !std::isfinite(request_distance) ||
      request_distance > parameters.sampling.max_distance_m) {
    return result;
  }
  const auto search_deadline =
      std::chrono::steady_clock::now() +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
          std::chrono::duration<double>(parameters.maximum_search_time_sec));

  const auto start_clearance = evaluate_elevator_scoped_clearance(
      costmap, footprint, start.x, start.y, start.yaw);
  if (!start_clearance.is_clear()) {
    result.status = ElevatorScopedSearchStatus::kStartBlocked;
    result.direct_path_block = start_clearance;
    return result;
  }
  const auto goal_clearance = evaluate_elevator_scoped_clearance(
      costmap, footprint, goal.x, goal.y, goal.yaw);
  if (!goal_clearance.is_clear()) {
    result.status = ElevatorScopedSearchStatus::kGoalBlocked;
    result.direct_path_block = goal_clearance;
    return result;
  }

  const bool prefer_clear_startup_spin =
      parameters.route_policy ==
      ElevatorScopedRoutePolicy::kPreferClearStartupSpin;
  const bool reverse_docking_sequence =
      parameters.route_policy ==
      ElevatorScopedRoutePolicy::kReverseDockingSequence;
  const bool reverse_entry_staging_sequence =
      parameters.route_policy ==
      ElevatorScopedRoutePolicy::kReverseEntryStagingSequence;

  if (reverse_docking_sequence || reverse_entry_staging_sequence) {
    const auto candidate =
        make_commissioned_axis_sequence_path(
          start, goal, parameters.sampling,
          reverse_entry_staging_sequence);
    if (!candidate) {
      return result;
    }
    const auto validation =
        validate_samples(costmap, footprint, candidate->samples);
    if (!validation.clear) {
      result.status = ElevatorScopedSearchStatus::kNoPath;
      result.direct_path_block = validation.blocking;
      return result;
    }
    result.status = ElevatorScopedSearchStatus::kSuccess;
    result.path = *candidate;
    result.path_length_m = path_length(result.path);
    result.mode_switches = count_mode_switches(result.path);
    result.direct_path_objective =
        path_objective(costmap, start, result.path, parameters);
    result.selected_path_objective = result.direct_path_objective;
    result.direct_soft_cost_exposure =
        path_soft_cost_exposure(costmap, start, result.path);
    result.direct_path_block.status =
        ElevatorScopedClearanceStatus::kClear;
    return result;
  }

  std::optional<ElevatorScopedPath> direct_candidate;
  ValidationResult direct_validation;
  bool direct_validation_recorded = false;
  double direct_objective = std::numeric_limits<double>::infinity();
  if (prefer_clear_startup_spin) {
    const auto candidate =
        make_spin_translate_spin_path(start, goal, parameters.sampling);
    if (!candidate) {
      return result;
    }
    const auto validation =
        validate_samples(costmap, footprint, candidate->samples);
    direct_validation = validation;
    direct_validation_recorded = true;
    if (validation.clear) {
      direct_candidate = *candidate;
      direct_objective =
          path_objective(costmap, start, *direct_candidate, parameters);
      result.direct_path_objective = direct_objective;
      result.direct_soft_cost_exposure =
          path_soft_cost_exposure(costmap, start, *direct_candidate);
      result.direct_path_block.status = ElevatorScopedClearanceStatus::kClear;
    }
  } else {
    for (const bool lateral_first : {false, true}) {
      ElevatorScopedPath candidate;
      candidate.samples.push_back({ElevatorScopedMotionPhase::kYaw, start});
      auto connection = make_axis_connection(
          start, goal, start.yaw, parameters.sampling.translation_step_m,
          lateral_first, parameters.sampling);
      candidate.samples.insert(candidate.samples.end(), connection.begin(),
                               connection.end());
      const auto validation =
          validate_samples(costmap, footprint, candidate.samples);
      if (!direct_validation_recorded) {
        direct_validation = validation;
        direct_validation_recorded = true;
      }
      if (!validation.clear) {
        continue;
      }
      direct_candidate = std::move(candidate);
      direct_objective =
          path_objective(costmap, start, *direct_candidate, parameters);
      result.direct_path_objective = direct_objective;
      result.direct_soft_cost_exposure =
          path_soft_cost_exposure(costmap, start, *direct_candidate);
      result.direct_path_block.status = ElevatorScopedClearanceStatus::kClear;
      break;
    }
  }
  if (direct_candidate &&
      (prefer_clear_startup_spin || parameters.soft_cost_weight <= 0.0 ||
       !path_has_soft_cost(costmap, *direct_candidate))) {
    result.status = ElevatorScopedSearchStatus::kSuccess;
    result.path = *direct_candidate;
    result.path_length_m = path_length(result.path);
    result.mode_switches = count_mode_switches(result.path);
    result.selected_path_objective = direct_objective;
    return result;
  }
  if (!direct_candidate) {
    result.direct_path_block = direct_validation.blocking;
  }

  // The live start yaw is the proven ingress axis. Keep it until the search
  // explicitly selects a checked in-place transition to another grid-aligned
  // heading; the final commissioned goal yaw remains independent from the
  // landing-to-cabin chord.
  const double cos_yaw = std::cos(start.yaw);
  const double sin_yaw = std::sin(start.yaw);
  const double goal_dx = goal.x - start.x;
  const double goal_dy = goal.y - start.y;
  const double goal_forward = cos_yaw * goal_dx + sin_yaw * goal_dy;
  const double goal_lateral = -sin_yaw * goal_dx + cos_yaw * goal_dy;

  std::vector<SearchNode> nodes;
  nodes.reserve(std::min<std::size_t>(parameters.maximum_expansions, 4096U));
  nodes.push_back(SearchNode{});
  std::unordered_map<SearchKey, double, SearchKeyHash> best_cost;
  best_cost.emplace(nodes.front().key, 0.0);
  std::unordered_map<GeometricEdgeKey, bool, GeometricEdgeKeyHash>
      geometric_edge_is_clear;
  std::unordered_map<GeometricPoseKey, bool, GeometricPoseKeyHash>
      geometric_pose_is_clear;
  geometric_pose_is_clear.emplace(GeometricPoseKey{0, 0, 0}, true);
  const auto checked_pose_is_clear =
      [&costmap, &footprint, &geometric_pose_is_clear](
          const SearchKey &key, const ElevatorScopedPose &pose) {
        const GeometricPoseKey geometric_key{
            key.forward_index,
            key.lateral_index,
            normalized_heading_index(key.heading_index),
        };
        const auto cached = geometric_pose_is_clear.find(geometric_key);
        if (cached != geometric_pose_is_clear.end()) {
          return cached->second;
        }
        const bool clear = evaluate_elevator_scoped_clearance(
                               costmap, footprint, pose.x, pose.y, pose.yaw)
                               .is_clear();
        geometric_pose_is_clear.emplace(geometric_key, clear);
        return clear;
      };
  std::priority_queue<QueueEntry, std::vector<QueueEntry>, QueueEntryGreater>
      open;
  std::uint64_t insertion_order = 0U;
  const double heuristic_weight = direct_candidate ? 1.0 : kHeuristicWeight;
  bool distance_field_time_limit = false;
  const auto obstacle_distance = make_obstacle_distance_field(
      costmap, footprint, goal, search_deadline, distance_field_time_limit);
  if (distance_field_time_limit) {
    result.status = ElevatorScopedSearchStatus::kTimeLimit;
    return result;
  }
  if (!std::isfinite(obstacle_distance.distance_at(costmap, start))) {
    if (direct_candidate) {
      result.status = ElevatorScopedSearchStatus::kSuccess;
      result.path = *direct_candidate;
      result.path_length_m = path_length(result.path);
      result.mode_switches = count_mode_switches(result.path);
      result.selected_path_objective = direct_objective;
      return result;
    }
    result.status = ElevatorScopedSearchStatus::kNoPath;
    return result;
  }
  open.push({heuristic_weight *
                 heuristic(costmap, start, goal, parameters, obstacle_distance),
             insertion_order++, 0U});

  std::optional<std::size_t> goal_node;
  std::vector<ElevatorScopedPathSample> goal_connection;
  bool time_limit_reached = false;
  while (!open.empty() &&
         result.expanded_nodes < parameters.maximum_expansions) {
    if (std::chrono::steady_clock::now() >= search_deadline) {
      time_limit_reached = true;
      break;
    }
    const auto current_entry = open.top();
    open.pop();
    if (current_entry.node_index >= nodes.size()) {
      continue;
    }
    const auto current = nodes[current_entry.node_index];
    const auto best = best_cost.find(current.key);
    if (best == best_cost.end() ||
        current.cost_from_start > best->second + 1.0e-9) {
      continue;
    }
    ++result.expanded_nodes;
    const auto current_pose = grid_pose(
        start, parameters.search_grid_step_m, current.key.forward_index,
        current.key.lateral_index, current.key.heading_index);
    const bool should_try_goal_connection =
        std::hypot(goal.x - current_pose.x, goal.y - current_pose.y) <=
        std::max(parameters.goal_connect_distance_m,
                 1.5 * parameters.search_grid_step_m);
    if (should_try_goal_connection) {
      const auto connection = clear_goal_connection(
          costmap, footprint, current_pose, goal, parameters);
      if (connection) {
        goal_node = current_entry.node_index;
        goal_connection = *connection;
        break;
      }
    }

    const int heading = normalized_heading_index(current.key.heading_index);
    const int forward_x = kHeadingForwardDeltas[heading].first;
    const int forward_y = kHeadingForwardDeltas[heading].second;
    const std::array<std::tuple<int, int, SearchMotion>, 4U> transitions{{
        {forward_x, forward_y, SearchMotion::kForward},
        {-forward_x, -forward_y, SearchMotion::kReverse},
        {-forward_y, forward_x, SearchMotion::kLateral},
        {forward_y, -forward_x, SearchMotion::kLateral},
    }};
    for (const auto &transition : transitions) {
      const int forward_delta = std::get<0>(transition);
      const int lateral_delta = std::get<1>(transition);
      const SearchMotion motion = std::get<2>(transition);
      SearchKey next_key{
          current.key.forward_index + forward_delta,
          current.key.lateral_index + lateral_delta,
          current.key.heading_index,
          motion,
      };
      const double local_forward = static_cast<double>(next_key.forward_index) *
                                   parameters.search_grid_step_m;
      const double local_lateral = static_cast<double>(next_key.lateral_index) *
                                   parameters.search_grid_step_m;
      if (std::hypot(local_forward, local_lateral) >
          parameters.search_radius_m + 1.0e-9) {
        continue;
      }
      const auto next_pose = grid_pose(
          start, parameters.search_grid_step_m, next_key.forward_index,
          next_key.lateral_index, next_key.heading_index);
      if (!std::isfinite(obstacle_distance.distance_at(costmap, next_pose))) {
        continue;
      }
      if (!checked_pose_is_clear(next_key, next_pose)) {
        continue;
      }
      const auto edge_key = make_geometric_edge_key(
          current.key.forward_index, current.key.lateral_index,
          current.key.heading_index, next_key.forward_index,
          next_key.lateral_index, next_key.heading_index);
      auto cached_clearance = geometric_edge_is_clear.find(edge_key);
      bool edge_is_clear = false;
      if (cached_clearance != geometric_edge_is_clear.end()) {
        edge_is_clear = cached_clearance->second;
      } else {
        const auto edge_samples = sample_translation(
            current_pose, next_pose, phase_for_motion(motion),
            parameters.sampling.translation_step_m);
        edge_is_clear =
            validate_interior_samples(costmap, footprint, edge_samples).clear;
        geometric_edge_is_clear.emplace(edge_key, edge_is_clear);
      }
      if (!edge_is_clear) {
        continue;
      }
      const auto cost = center_cost(costmap, next_pose);
      if (cost >= nav2_costmap_2d::LETHAL_OBSTACLE) {
        continue;
      }
      const double normalized_soft_cost =
          static_cast<double>(cost) /
          static_cast<double>(nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
      const double edge_distance = std::hypot(next_pose.x - current_pose.x,
                                              next_pose.y - current_pose.y);
      double edge_cost =
          edge_distance * (motion_cost(motion, parameters) +
                           parameters.soft_cost_weight * normalized_soft_cost);
      if (current.key.last_motion != SearchMotion::kNone &&
          current.key.last_motion != motion) {
        edge_cost += parameters.mode_switch_cost;
      }
      const double tentative_cost = current.cost_from_start + edge_cost;
      const auto previous = best_cost.find(next_key);
      if (previous != best_cost.end() &&
          tentative_cost >= previous->second - 1.0e-9) {
        continue;
      }

      SearchNode next;
      next.key = next_key;
      next.cost_from_start = tentative_cost;
      next.parent = current_entry.node_index;
      next.phase = phase_for_motion(motion);
      const auto next_index = nodes.size();
      nodes.push_back(next);
      best_cost[next_key] = tentative_cost;
      open.push({
          tentative_cost + heuristic_weight * heuristic(costmap, next_pose,
                                                        goal, parameters,
                                                        obstacle_distance),
          insertion_order++,
          next_index,
      });
    }

    for (const int heading_delta : {-1, 1}) {
      SearchKey next_key{
          current.key.forward_index,
          current.key.lateral_index,
          normalized_heading_index(current.key.heading_index + heading_delta),
          SearchMotion::kRotate,
      };
      const auto next_pose = grid_pose(
          start, parameters.search_grid_step_m, next_key.forward_index,
          next_key.lateral_index, next_key.heading_index);
      if (!checked_pose_is_clear(next_key, next_pose)) {
        continue;
      }
      const auto edge_key = make_geometric_edge_key(
          current.key.forward_index, current.key.lateral_index,
          current.key.heading_index, next_key.forward_index,
          next_key.lateral_index, next_key.heading_index);
      auto cached_clearance = geometric_edge_is_clear.find(edge_key);
      bool edge_is_clear = false;
      if (cached_clearance != geometric_edge_is_clear.end()) {
        edge_is_clear = cached_clearance->second;
      } else {
        const auto rotation = make_elevator_scoped_path(current_pose, next_pose,
                                                        parameters.sampling);
        edge_is_clear = rotation && validate_interior_samples(
                                        costmap, footprint, rotation->samples)
                                        .clear;
        geometric_edge_is_clear.emplace(edge_key, edge_is_clear);
      }
      if (!edge_is_clear) {
        continue;
      }
      double edge_cost =
          std::abs(normalize_elevator_scoped_angle(next_pose.yaw -
                                                   current_pose.yaw)) *
          kRotationObjectiveCostPerRadian * parameters.forward_cost;
      if (current.key.last_motion != SearchMotion::kNone &&
          current.key.last_motion != SearchMotion::kRotate) {
        edge_cost += parameters.mode_switch_cost;
      }
      const double tentative_cost = current.cost_from_start + edge_cost;
      const auto previous = best_cost.find(next_key);
      if (previous != best_cost.end() &&
          tentative_cost >= previous->second - 1.0e-9) {
        continue;
      }

      SearchNode next;
      next.key = next_key;
      next.cost_from_start = tentative_cost;
      next.parent = current_entry.node_index;
      next.phase = ElevatorScopedMotionPhase::kYaw;
      const auto next_index = nodes.size();
      nodes.push_back(next);
      best_cost[next_key] = tentative_cost;
      open.push({
          tentative_cost + heuristic_weight * heuristic(costmap, next_pose,
                                                        goal, parameters,
                                                        obstacle_distance),
          insertion_order++,
          next_index,
      });
    }
  }

  if (!goal_node) {
    if (direct_candidate) {
      result.status = ElevatorScopedSearchStatus::kSuccess;
      result.path = *direct_candidate;
      result.path_length_m = path_length(result.path);
      result.mode_switches = count_mode_switches(result.path);
      result.selected_path_objective = direct_objective;
      return result;
    }
    result.status =
        time_limit_reached
            ? ElevatorScopedSearchStatus::kTimeLimit
            : (result.expanded_nodes >= parameters.maximum_expansions
                   ? ElevatorScopedSearchStatus::kExpansionLimit
                   : ElevatorScopedSearchStatus::kNoPath);
    return result;
  }

  std::vector<std::size_t> chain;
  for (std::size_t index = *goal_node;
       index != std::numeric_limits<std::size_t>::max();
       index = nodes[index].parent) {
    chain.push_back(index);
  }
  std::reverse(chain.begin(), chain.end());

  ElevatorScopedPath searched_path;
  searched_path.samples.push_back({ElevatorScopedMotionPhase::kYaw, start});
  for (std::size_t chain_index = 1U; chain_index < chain.size();
       ++chain_index) {
    const auto &previous = nodes[chain[chain_index - 1U]];
    const auto &current = nodes[chain[chain_index]];
    const auto previous_pose = grid_pose(
        start, parameters.search_grid_step_m, previous.key.forward_index,
        previous.key.lateral_index, previous.key.heading_index);
    const auto current_pose = grid_pose(
        start, parameters.search_grid_step_m, current.key.forward_index,
        current.key.lateral_index, current.key.heading_index);
    if (current.phase == ElevatorScopedMotionPhase::kYaw) {
      const auto rotation = make_elevator_scoped_path(
          previous_pose, current_pose, parameters.sampling);
      if (rotation && rotation->samples.size() > 1U) {
        searched_path.samples.insert(searched_path.samples.end(),
                                     std::next(rotation->samples.begin()),
                                     rotation->samples.end());
      }
    } else {
      auto edge = sample_translation(previous_pose, current_pose, current.phase,
                                     parameters.sampling.translation_step_m);
      searched_path.samples.insert(searched_path.samples.end(), edge.begin(),
                                   edge.end());
    }
  }
  searched_path.samples.insert(searched_path.samples.end(),
                               goal_connection.begin(), goal_connection.end());
  searched_path.forward_m = goal_forward;
  searched_path.lateral_m = goal_lateral;
  if (!searched_path.samples.empty()) {
    searched_path.samples.back().pose = goal;
  }

  const double searched_objective =
      path_objective(costmap, start, searched_path, parameters);
  result.searched_path_objective = searched_objective;
  result.searched_soft_cost_exposure =
      path_soft_cost_exposure(costmap, start, searched_path);
  const bool search_reduces_soft_exposure =
      result.searched_soft_cost_exposure + 1.0e-9 <
      result.direct_soft_cost_exposure;
  const bool search_is_within_clearance_budget =
      searched_objective <=
      direct_objective * parameters.clearance_preference_cost_ratio + 1.0e-9;
  if (direct_candidate && searched_objective >= direct_objective - 1.0e-9 &&
      !(search_reduces_soft_exposure && search_is_within_clearance_budget)) {
    result.path = *direct_candidate;
    result.used_detour = false;
    result.selected_path_objective = direct_objective;
  } else {
    result.path = std::move(searched_path);
    result.used_detour = true;
    result.used_startup_rotation_escape = prefer_clear_startup_spin;
    result.selected_path_objective = searched_objective;
  }
  result.status = ElevatorScopedSearchStatus::kSuccess;
  result.path_length_m = path_length(result.path);
  result.mode_switches = count_mode_switches(result.path);
  return result;
}

const char *elevator_scoped_search_status_name(
    const ElevatorScopedSearchStatus status) noexcept {
  switch (status) {
  case ElevatorScopedSearchStatus::kSuccess:
    return "success";
  case ElevatorScopedSearchStatus::kInvalidRequest:
    return "invalid_request";
  case ElevatorScopedSearchStatus::kStartBlocked:
    return "start_blocked";
  case ElevatorScopedSearchStatus::kGoalBlocked:
    return "goal_blocked";
  case ElevatorScopedSearchStatus::kNoPath:
    return "no_path";
  case ElevatorScopedSearchStatus::kExpansionLimit:
    return "expansion_limit";
  case ElevatorScopedSearchStatus::kTimeLimit:
    return "time_limit";
  }
  return "invalid_request";
}

} // namespace robot_nav_config
