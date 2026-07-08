#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/commercial_runtime_helpers.sh"

check_process() {
  local label="$1"
  local pattern="$2"
  if pgrep -f "${pattern}" >/dev/null 2>&1; then
    printf 'OK   process %-34s %s\n' "${label}" "${pattern}"
    return 0
  fi
  printf 'MISS process %-34s %s\n' "${label}" "${pattern}"
  return 1
}

check_process_absent() {
  local label="$1"
  local pattern="$2"
  if pgrep -f "${pattern}" >/dev/null 2>&1; then
    printf 'MISS process %-34s unexpected: %s\n' "${label}" "${pattern}"
    return 1
  fi
  printf 'OK   process %-34s absent as expected\n' "${label}"
  return 0
}

check_node() {
  local node_name="$1"
  if timeout 3 ros2 node info "${node_name}" >/dev/null 2>&1 ||
    timeout 3 ros2 node list 2>/dev/null | grep -Fxq -- "${node_name}"
  then
    printf 'OK   node    %s\n' "${node_name}"
    return 0
  fi
  printf 'MISS node    %s\n' "${node_name}"
  return 1
}

check_lifecycle_active() {
  local node_name="$1"
  local timeout_sec="${2:-8}"
  if nav2_lifecycle_node_active "${node_name}" "${timeout_sec}"; then
    printf 'OK   life    %-34s active [3]\n' "${node_name}"
    return 0
  fi
  printf 'MISS life    %-34s unavailable\n' "${node_name}"
  return 1
}

check_topic() {
  local topic="$1"
  local timeout_sec="${2:-2}"
  if wait_for_topic_message "${topic}" "${timeout_sec}" >/dev/null 2>&1; then
    printf 'OK   topic   %s\n' "${topic}"
    return 0
  fi
  printf 'MISS topic   %s\n' "${topic}"
  return 1
}

check_topic_once() {
  local topic="$1"
  local timeout_sec="${2:-2}"
  if timeout "${timeout_sec}" ros2 topic echo "${topic}" --once >/dev/null 2>&1; then
    printf 'OK   topic   %s\n' "${topic}"
    return 0
  fi
  printf 'MISS topic   %s\n' "${topic}"
  return 1
}

check_topic_publisher_node() {
  local topic="$1"
  local node_name="$2"
  local timeout_sec="${3:-2}"
  if wait_for_topic_publisher_from_node "${topic}" "${node_name}" "${timeout_sec}" >/dev/null 2>&1; then
    printf 'OK   topic   %s publisher=%s\n' "${topic}" "${node_name}"
    return 0
  fi
  printf 'MISS topic   %s publisher=%s\n' "${topic}" "${node_name}"
  return 1
}

check_tf() {
  local target="$1"
  local source="$2"
  local timeout_sec="${3:-2}"
  if wait_for_tf_transform "${target}" "${source}" "${timeout_sec}" >/dev/null 2>&1; then
    printf 'OK   tf      %s->%s\n' "${target}" "${source}"
    return 0
  fi
  printf 'MISS tf      %s->%s\n' "${target}" "${source}"
  return 1
}

failures=0
run_check() {
  "$@" || failures=$((failures + 1))
}

printf '[runtime-overlay] commercial runtime readiness check\n'

run_check check_process "jt128_driver" "hesai_ros_driver_node|hesai_accel_driver_node|jt128_accel_driver_node"
if [[ "${NJRH_NAV_LOCAL_STATE_MODE:-ekf}" == "fastlio" || "${NJRH_FASTLIO_AUTOSTART:-false}" == "true" ]]; then
  run_check check_process "fastlio_mapping" "fast_lio fastlio_mapping|laser_mapping"
else
  run_check check_process_absent "fastlio_mapping" "fast_lio fastlio_mapping|laser_mapping"
fi
run_check check_process "robot_local_state" "ekf_node --ros-args.*__node:=robot_local_state|robot_localization/ekf_node|robot_local_state/local_state_node|local_state_node --ros-args"
run_check check_process "localization_bridge" "robot_localization_bridge/localization_bridge_node|localization_bridge_node --ros-args"
run_check check_process "robot_safety" "robot_safety/robot_safety_node|robot_safety_node --ros-args"

run_check check_node "/robot_api_server"
run_check check_node "/robot_floor_manager"
run_check check_node "/robot_global_localization"
run_check check_node "/map_server"
run_check check_node "/controller_server"
run_check check_node "/planner_server"
run_check check_node "/bt_navigator"
run_check check_node "/velocity_smoother"
run_check check_node "/collision_monitor"

run_check check_lifecycle_active "/map_server" 12
run_check check_lifecycle_active "/controller_server" 12
run_check check_lifecycle_active "/planner_server" 12
run_check check_lifecycle_active "/bt_navigator" 12
run_check check_lifecycle_active "/velocity_smoother" 12
run_check check_lifecycle_active "/collision_monitor" 12

run_check check_topic "/local_state/odometry" 10
run_check check_topic "/scan" 10
run_check check_topic_publisher_node "/flatscan" "laser_scan_to_flatscan" 10
run_check check_topic_once "/safety/status" 10
run_check check_tf "odom" "base_link" 10
run_check check_tf "map" "odom" 10

if wait_for_occupancy_grid "/map" 10 >/dev/null 2>&1; then
  printf 'OK   map     /map\n'
else
  printf 'MISS map     /map\n'
  failures=$((failures + 1))
fi

if wait_for_global_costmap_static 10 >/dev/null 2>&1; then
  printf 'OK   costmap /global_costmap/costmap\n'
else
  printf 'MISS costmap /global_costmap/costmap\n'
  failures=$((failures + 1))
fi

if [[ "${failures}" -eq 0 ]]; then
  printf '[runtime-overlay] commercial runtime ready\n'
  exit 0
fi

printf '[runtime-overlay] commercial runtime not ready: %s failed checks\n' "${failures}" >&2
exit 1
