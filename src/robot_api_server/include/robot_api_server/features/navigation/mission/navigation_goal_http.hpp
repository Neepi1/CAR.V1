#pragma once

#include <cstdint>
#include <optional>
#include <string>

#include "robot_api_server/features/maps/catalog_activation/storage_models.hpp"
#include "robot_api_server/infrastructure/http/http_common.hpp"

namespace robot_api_server::features::navigation
{

struct NavigationGoalAcceptedPayload
{
  std::uint64_t navigation_goal_id{0U};
  std::string action;
  std::string source;
  std::string goal_completion_policy;
  bool native_nav2_goal_completion{false};
  bool api_final_yaw_align_enabled{false};
  bool nav2_rotation_shim_enabled{true};
  std::string frame_id{"map"};
  std::string pose_id;
  std::optional<std::string> building_id;
  std::optional<std::string> floor_id;
  bool pre_navigation_undock{false};
  std::string pre_navigation_undock_detail;
  std::string pre_navigation_dock_check_json{"{}"};
  bool pre_navigation_relocalization_requested{false};
  bool pre_navigation_relocalization_succeeded{false};
  std::string pre_navigation_relocalization_detail;
  StoredPose goal;
};

HttpResponse navigation_goal_error_response(
  int status,
  const std::string & error,
  bool pre_navigation_undock,
  const std::string & pre_navigation_undock_detail,
  const std::string & pre_navigation_dock_check_json,
  const std::string & code = "");

HttpResponse navigation_goal_accepted_response(const NavigationGoalAcceptedPayload & payload);

}  // namespace robot_api_server::features::navigation
