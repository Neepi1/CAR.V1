#!/usr/bin/env bash
set +e

cleanup_shell_pid="$$"
cleanup_parent_pid="${PPID}"

pids_by_pattern() {
  local pattern="$1"
  ps -eo pid=,args= \
    | awk \
      -v pattern="${pattern}" \
      -v cleanup_shell_pid="${cleanup_shell_pid}" \
      -v cleanup_parent_pid="${cleanup_parent_pid}" '
        $0 ~ pattern &&
        $1 != cleanup_shell_pid &&
        $1 != cleanup_parent_pid &&
        $0 !~ /awk -v pattern/ &&
        $0 !~ /pids_by_pattern/ &&
        $0 !~ /stop_exact_process_set/ &&
        $0 !~ /stop_runtime_processes[.]sh/ {print $1}
      '
}

wait_pids_gone() {
  local timeout_sec="$1"
  shift || true
  local pids=("$@")
  local deadline=$((SECONDS + timeout_sec))
  local pid
  while (( SECONDS < deadline )); do
    local alive=0
    for pid in "${pids[@]}"; do
      [[ -n "${pid}" && -d "/proc/${pid}" ]] && alive=1
    done
    [[ "${alive}" -eq 0 ]] && return 0
    sleep 0.2
  done
  return 1
}

stop_exact_process_set() {
  local label="$1"
  local pattern="$2"
  local pids=()
  mapfile -t pids < <(pids_by_pattern "${pattern}")
  [[ "${#pids[@]}" -gt 0 ]] || return 0
  echo "[njrh-systemd] stopping ${label} pids=${pids[*]}" >&2
  kill -INT "${pids[@]}" 2>/dev/null || true
  wait_pids_gone 2 "${pids[@]}" && return 0

  mapfile -t pids < <(pids_by_pattern "${pattern}")
  [[ "${#pids[@]}" -gt 0 ]] || return 0
  kill -TERM "${pids[@]}" 2>/dev/null || true
  wait_pids_gone 3 "${pids[@]}" && return 0

  mapfile -t pids < <(pids_by_pattern "${pattern}")
  [[ "${#pids[@]}" -gt 0 ]] || return 0
  echo "[njrh-systemd] killing exact stale ${label} pids=${pids[*]}" >&2
  kill -KILL "${pids[@]}" 2>/dev/null || true
}

common_pattern="run_common_services.sh"
node_pattern="run_driver.sh|run_pointcloud_accel_pipeline.sh|laser_scan_to_flatscan|hesai_ros_driver_node|pointcloud_accel_axis_node|pointcloud_axis_remap|imu_axis_remap|ranger_base_node|robot_description_static_tf_node|robot_eai_gs2/gs2_driver_node|gs2_driver_node --ros-args|ros2 launch robot_eai_gs2 gs2.launch.py|runtime_health_guard.py|run_runtime_health_guard.sh|ekf_node --ros-args.*__node:=robot_local_state|robot_localization/ekf_node|robot_local_perception/local_perception_node|robot_floor_manager/floor_manager_node|robot_safety/robot_safety_node|ranger_mini3_mode_controller/mode_controller_node|robot_docking_manager/docking_manager_node|docking_manager_node --ros-args|run_robot_api_server_supervised.sh|robot_api_server/robot_api_server_node|robot_api_server_node --ros-args|run_navigation_runtime_services.sh|nav2_lifecycle_sequence.py|call_global_localization_trigger.py|run_nav2_navigation.sh|run_occupancy_grid_localization.sh|standard_navigation.launch.py|occupancy_localization_stack.launch.py|occupancy_grid_localizer_container|occupancy_grid_localizer|robot_global_localization/global_localization_node|/install/robot_global_localization/lib/robot_global_localization/global_localization_node|robot_localization_bridge/localization_bridge_node|localization_bridge_node --ros-args|amcl --ros-args|nav2_amcl|amcl_scan_admission|__node:=map_server|__node:=controller_server|__node:=planner_server|__node:=bt_navigator|__node:=behavior_server|__node:=velocity_smoother|__node:=collision_monitor|__node:=lifecycle_manager_navigation|__node:=lifecycle_manager_costmap_filters"
ros2_cli_pattern="/opt/ros/humble/bin/ros2 (lifecycle get|topic echo|topic hz|topic info|node info|service call /amcl/(change_state|get_state))|ros2 (lifecycle get|topic echo|topic hz|topic info|node info|service call /amcl/(change_state|get_state))"

stop_exact_process_set "stale ros2 diagnostics cli" "${ros2_cli_pattern}"
stop_exact_process_set "common services" "${common_pattern}"
stop_exact_process_set "runtime nodes" "${node_pattern}"

rm -f \
  /tmp/njrh_runtime_map_context.json \
  /tmp/njrh_runtime_health.json \
  /tmp/njrh_amcl_runtime_status.env \
  /tmp/njrh_nav2_launch_hold_ready.env \
  /tmp/njrh_nav2_lifecycle_ready.env \
  2>/dev/null || true
