#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export NJRH_OVERLAY_ROOT="${NJRH_OVERLAY_ROOT:-$(cd "${SCRIPT_DIR}/.." && pwd)}"
source "${SCRIPT_DIR}/cpu_affinity.sh"

apply_pattern() {
  local service_name="$1"
  local pattern="$2"
  local pids
  pids="$(pgrep -f "${pattern}" || true)"
  if [[ -z "${pids}" ]]; then
    return 0
  fi
  # shellcheck disable=SC2086
  njrh_apply_affinity_to_pids "${service_name}" ${pids}
}

apply_pattern robot_api_server "robot_api_server_node"
apply_pattern ranger_base_node "ranger_base_node"
apply_pattern robot_safety "robot_safety_node"
apply_pattern velocity_smoother "velocity_smoother"
apply_pattern ranger_mini3_mode_controller "mode_controller_node"
apply_pattern docking_manager "docking_manager_node"

apply_pattern robot_local_state "ekf_node.*__node:=robot_local_state|local_state_node"
apply_pattern robot_localization_bridge "localization_bridge_node"

apply_pattern controller_server "controller_server"
apply_pattern collision_monitor "collision_monitor"

apply_pattern hesai_ros_driver "hesai_ros_driver_node"
apply_pattern pointcloud_perception_pipeline "pointcloud_perception_pipeline.launch.py|component_container_mt.*pointcloud_perception_pipeline|pointcloud_perception_pipeline"
apply_pattern pointcloud_axis_remap "pointcloud_axis_remap"
apply_pattern pointcloud_downsample "pointcloud_downsample"
apply_pattern pointcloud_fastlio_remap "pointcloud_fastlio_remap"
apply_pattern imu_axis_remap "imu_axis_remap"
apply_pattern nav_cloud_preprocessor "nav_cloud_preprocessor"
apply_pattern robot_local_perception "local_perception_node"

apply_pattern occupancy_grid_localizer "occupancy_grid_localizer"
apply_pattern robot_global_localization "global_localization_node"
apply_pattern laser_scan_to_flatscan "laser_scan_to_flatscan"
apply_pattern pointcloud_to_laserscan "pointcloud_to_laserscan"
apply_pattern scan_republisher "scan_republisher_node"

apply_pattern planner_server "planner_server"
apply_pattern bt_navigator "bt_navigator"
apply_pattern behavior_server "behavior_server"
apply_pattern smoother_server "smoother_server"
apply_pattern waypoint_follower "waypoint_follower"
apply_pattern nav2_map_server "map_server|costmap_filter_info_server"
apply_pattern nav2_lifecycle_manager "lifecycle_manager"

apply_pattern fastlio_mapping "fastlio_mapping|laser_mapping"
apply_pattern slam_toolbox_mapping "slam_toolbox"
apply_pattern pgo_mapping "pgo_node|fastlio_pgo|run_pgo.sh"
