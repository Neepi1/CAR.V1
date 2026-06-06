#pragma once

#include <cstdint>
#include <string>

namespace robot_api_server
{

struct DockingJob
{
  std::uint64_t id{0U};
  std::string state{"idle"};
  std::string phase{"idle"};
  std::string building_id;
  std::string floor_id;
  std::string map_id;
  std::string dock_id;
  std::string dock_name;
  std::string dock_type;
  std::string predock_pose_id;
  std::string approach_source{"computed_from_dock_pose"};
  std::string detail;
  std::string last_status;
  std::string relocalization_detail;
  std::string post_predock_relocalization_detail;
  std::string post_fine_docking_relocalization_detail;
  std::string post_undock_relocalization_detail;
  std::string active_navigation_cancel_detail;
  std::string started_at;
  std::string finished_at;
  double dock_x{0.0};
  double dock_y{0.0};
  double dock_yaw{0.0};
  double approach_x{0.0};
  double approach_y{0.0};
  double approach_yaw{0.0};
  double approach_distance_m{0.0};
  bool ok{true};
  bool resume_navigation{true};
  bool nav_goal_sent{false};
  bool nav_goal_succeeded{false};
  bool relocalization_requested{false};
  bool relocalization_succeeded{false};
  bool post_predock_relocalization_requested{false};
  bool post_predock_relocalization_succeeded{false};
  bool post_predock_relocalization_required{false};
  bool post_fine_docking_relocalization_requested{false};
  bool post_fine_docking_relocalization_succeeded{false};
  bool post_fine_docking_relocalization_required{false};
  bool post_undock_relocalization_requested{false};
  bool post_undock_relocalization_succeeded{false};
  bool post_undock_relocalization_required{false};
  bool active_navigation_cancel_requested{false};
  bool docking_service_called{false};
  bool cancel_requested{false};
};

std::string docking_job_json(const DockingJob & job);

}  // namespace robot_api_server
