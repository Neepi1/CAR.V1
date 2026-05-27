#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/common_env.sh"

publish_zero() {
  timeout 2s ros2 topic pub --once /cmd_vel_collision_checked geometry_msgs/msg/Twist '{}' >/dev/null 2>&1 || true
}

patterns=(
  "run_floor_navigation.sh"
  "run_nav2_navigation.sh"
  "run_occupancy_grid_localization.sh"
  "standard_navigation.launch.py"
  "occupancy_localization_stack.launch.py"
  "occupancy_localization.launch.py"
  "controller_server"
  "planner_server"
  "bt_navigator"
  "behavior_server"
  "smoother_server"
  "waypoint_follower"
  "velocity_smoother"
  "collision_monitor"
  "lifecycle_manager_navigation"
  "global_costmap"
  "local_costmap"
  "keepout_filter_mask_server"
  "speed_filter_mask_server"
  "keepout_costmap_filter_info_server"
  "speed_costmap_filter_info_server"
  "map_server"
  "lifecycle_manager_map"
  "occupancy_grid_localizer_container"
  "occupancy_grid_localizer"
  "laser_scan_to_flatscan"
  "pointcloud_to_laserscan_node"
  "pointcloud_to_laserscan"
  "scan_republisher_node"
  "nav_cloud_preprocessor"
  "run_global_localization.sh"
  "robot_global_localization/global_localization_node.py"
  "robot_localization_bridge/localization_bridge_node"
  "localization_bridge_node --ros-args"
  "map_to_odom_tf_bridge"
)

matching_navigation_processes() {
  local joined
  joined="$(printf '%s|' "${patterns[@]}")"
  joined="${joined%|}"
  ps -eo pid=,user=,args= \
    | grep -E "${joined}" \
    | grep -v grep \
    | grep -v "stop_floor_navigation.sh" \
    | grep -vE "/opt/ros/.*/bin/ros2 (action|daemon|doctor|lifecycle|node|param|service|topic) " \
    || true
}

wait_until_clear() {
  local timeout_sec="${1:-8}"
  local start
  start="$(date +%s)"
  while true; do
    if [[ -z "$(matching_navigation_processes)" ]]; then
      return 0
    fi
    if (( $(date +%s) - start >= timeout_sec )); then
      return 1
    fi
    sleep 0.25
  done
}

kill_navigation_patterns() {
  local signal="$1"
  local pattern
  for pattern in "${patterns[@]}"; do
    pkill "-${signal}" -f "${pattern}" 2>/dev/null || true
  done
}

echo "[runtime-overlay] stop floor navigation requested" >&2
publish_zero

kill_navigation_patterns INT
wait_until_clear 8 || true
publish_zero

kill_navigation_patterns TERM
wait_until_clear 5 || true
publish_zero

kill_navigation_patterns KILL
wait_until_clear 3 || true
publish_zero

lingering="$(matching_navigation_processes)"
if [[ -n "${lingering}" ]]; then
  echo "[runtime-overlay] navigation stack still has lingering processes:" >&2
  echo "${lingering}" >&2
  if [[ "$(id -u)" -ne 0 ]] && echo "${lingering}" | awk '{print $2}' | grep -qx "root"; then
    echo "[runtime-overlay] root-owned navigation processes remain; start/stop navigation with the same user as the API server or run this stop script as root once to clean stale processes" >&2
  fi
  exit 1
fi

echo "[runtime-overlay] floor navigation stack stopped; common driver/chassis/safety services kept alive" >&2
