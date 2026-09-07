#include "robot_nav_config/elevator_scoped_local_repair.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <optional>
#include <utility>
#include <vector>

namespace robot_nav_config
{
namespace
{

bool repair_parameters_are_valid(
  const ElevatorScopedLocalRepairParameters & parameters)
{
  return std::isfinite(parameters.minimum_rejoin_distance_m) &&
         std::isfinite(parameters.maximum_rejoin_distance_m) &&
         std::isfinite(parameters.rejoin_spacing_m) &&
         parameters.minimum_rejoin_distance_m > 0.0 &&
         parameters.maximum_rejoin_distance_m >=
         parameters.minimum_rejoin_distance_m &&
         parameters.rejoin_spacing_m > 0.0 &&
         parameters.maximum_rejoin_candidates > 0U;
}

std::vector<std::size_t> select_rejoin_candidates(
  const std::vector<ElevatorScopedIndexedPose> & remaining_path,
  const std::size_t nearest_position,
  const ElevatorScopedLocalRepairParameters & parameters)
{
  std::vector<std::size_t> candidates;
  candidates.reserve(parameters.maximum_rejoin_candidates);
  if (nearest_position + 1U >= remaining_path.size()) {
    return candidates;
  }

  double accumulated_distance = 0.0;
  double next_candidate_distance = parameters.minimum_rejoin_distance_m;
  std::optional<std::size_t> last_within_window;
  for (std::size_t position = nearest_position + 1U;
    position < remaining_path.size(); ++position)
  {
    const auto & previous = remaining_path[position - 1U].pose;
    const auto & current = remaining_path[position].pose;
    accumulated_distance += std::hypot(
      current.x - previous.x, current.y - previous.y);
    if (accumulated_distance + 1.0e-9 >=
      parameters.minimum_rejoin_distance_m &&
      accumulated_distance <= parameters.maximum_rejoin_distance_m + 1.0e-9)
    {
      last_within_window = position;
    }
    if (accumulated_distance + 1.0e-9 < next_candidate_distance) {
      continue;
    }
    if (accumulated_distance >
      parameters.maximum_rejoin_distance_m + 1.0e-9)
    {
      break;
    }

    candidates.push_back(position);
    if (candidates.size() >= parameters.maximum_rejoin_candidates) {
      break;
    }
    do {
      next_candidate_distance += parameters.rejoin_spacing_m;
    } while (next_candidate_distance <= accumulated_distance + 1.0e-9);
  }

  // Preserve a useful upper-window candidate when the sampled route spacing
  // skips the final threshold. This is often the first pose whose rear
  // footprint has completely cleared a door jamb.
  if (candidates.size() < parameters.maximum_rejoin_candidates &&
    last_within_window &&
    (candidates.empty() || candidates.back() != *last_within_window))
  {
    candidates.push_back(*last_within_window);
  }
  // Do not invent a sub-window final-pose repair. Near the terminal pose such
  // a micro rejoin cannot clear the obstacle that blocked the current command,
  // and repeatedly adopting it can restart the route at its yaw segment. With
  // no candidate at the configured minimum distance, WAIT_CLEAR is the safe
  // and monotonic behavior until the live costmap changes.
  return candidates;
}

}  // namespace

ElevatorScopedLocalRepairResult search_elevator_scoped_local_repair(
  nav2_costmap_2d::Costmap2D & costmap,
  const std::vector<geometry_msgs::msg::Point> & footprint,
  const ElevatorScopedPose & start,
  const std::vector<ElevatorScopedIndexedPose> & remaining_path,
  const ElevatorScopedSearchParameters & search_parameters,
  const ElevatorScopedLocalRepairParameters & repair_parameters)
{
  ElevatorScopedLocalRepairResult result;
  if (!elevator_scoped_pose_is_finite(start) || footprint.size() < 3U ||
    remaining_path.empty() ||
    !repair_parameters_are_valid(repair_parameters) ||
    !std::isfinite(search_parameters.maximum_search_time_sec) ||
    search_parameters.maximum_search_time_sec <= 0.0)
  {
    return result;
  }

  double nearest_distance = std::numeric_limits<double>::infinity();
  std::size_t nearest_position = 0U;
  std::size_t previous_path_index = 0U;
  bool have_previous_index = false;
  for (std::size_t position = 0U; position < remaining_path.size(); ++position) {
    const auto & indexed_pose = remaining_path[position];
    if (!elevator_scoped_pose_is_finite(indexed_pose.pose) ||
      (have_previous_index && indexed_pose.path_index <= previous_path_index))
    {
      return result;
    }
    have_previous_index = true;
    previous_path_index = indexed_pose.path_index;
    const double distance = std::hypot(
      indexed_pose.pose.x - start.x, indexed_pose.pose.y - start.y);
    if (distance < nearest_distance) {
      nearest_distance = distance;
      nearest_position = position;
    }
  }
  result.nearest_path_index = remaining_path[nearest_position].path_index;

  const auto candidate_positions = select_rejoin_candidates(
    remaining_path, nearest_position, repair_parameters);
  if (candidate_positions.empty()) {
    result.status = ElevatorScopedLocalRepairStatus::kNoRejoinCandidate;
    return result;
  }

  const auto deadline = std::chrono::steady_clock::now() +
    std::chrono::duration_cast<std::chrono::steady_clock::duration>(
    std::chrono::duration<double>(search_parameters.maximum_search_time_sec));
  ElevatorScopedSearchResult last_search;
  bool attempted = false;
  for (const auto candidate_position : candidate_positions) {
    const double remaining_sec = std::chrono::duration<double>(
      deadline - std::chrono::steady_clock::now()).count();
    if (remaining_sec <= 0.0) {
      result.status = ElevatorScopedLocalRepairStatus::kTimeLimit;
      result.search = std::move(last_search);
      if (!attempted || result.search.status ==
        ElevatorScopedSearchStatus::kInvalidRequest)
      {
        result.search.status = ElevatorScopedSearchStatus::kTimeLimit;
      }
      return result;
    }

    const auto & candidate = remaining_path[candidate_position];
    auto attempt_parameters = search_parameters;
    attempt_parameters.maximum_search_time_sec = remaining_sec;
    const double chord = std::hypot(
      candidate.pose.x - start.x, candidate.pose.y - start.y);
    // The repair search only needs enough radius for a bounded side-step and
    // rejoin. Keeping it local prevents a 100 ms recovery from restarting the
    // complete cabin search from scratch.
    attempt_parameters.search_radius_m = std::min(
      search_parameters.search_radius_m,
      std::max(0.75, chord + 0.60));
    last_search = search_elevator_scoped_path(
      costmap, footprint, start, candidate.pose, attempt_parameters);
    attempted = true;
    ++result.candidates_considered;
    if (last_search.succeeded()) {
      result.status = ElevatorScopedLocalRepairStatus::kSuccess;
      result.search = std::move(last_search);
      result.rejoin_path_index = candidate.path_index;
      return result;
    }
  }

  result.search = std::move(last_search);
  result.status =
    result.search.status == ElevatorScopedSearchStatus::kTimeLimit ?
    ElevatorScopedLocalRepairStatus::kTimeLimit :
    ElevatorScopedLocalRepairStatus::kSearchFailed;
  return result;
}

const char * elevator_scoped_local_repair_status_name(
  const ElevatorScopedLocalRepairStatus status) noexcept
{
  switch (status) {
    case ElevatorScopedLocalRepairStatus::kSuccess:
      return "success";
    case ElevatorScopedLocalRepairStatus::kInvalidRequest:
      return "invalid_request";
    case ElevatorScopedLocalRepairStatus::kNoRejoinCandidate:
      return "no_rejoin_candidate";
    case ElevatorScopedLocalRepairStatus::kSearchFailed:
      return "search_failed";
    case ElevatorScopedLocalRepairStatus::kTimeLimit:
      return "time_limit";
  }
  return "invalid_request";
}

}  // namespace robot_nav_config
