#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/common_env.sh"
source "${SCRIPT_DIR}/runtime_health_helpers.sh"

helper_pids=()

reuse_common_services_enabled() {
  [[ "${NJRH_REUSE_COMMON_SERVICES:-true}" == "true" ]]
}

force_restart_nav_helpers_enabled() {
  [[ "${NJRH_FORCE_RESTART_NAV_HELPERS:-false}" == "true" ]]
}

helper_process_pattern() {
  local helper_name="$1"
  case "${helper_name}" in
    local_perception*)
      printf '%s\n' "robot_local_perception/local_perception_node|/install/robot_local_perception/lib/robot_local_perception/local_perception_node"
      ;;
    robot_safety*)
      printf '%s\n' "robot_safety/robot_safety_node|/install/robot_safety/lib/robot_safety/robot_safety_node"
      ;;
    floor_manager*)
      printf '%s\n' "robot_floor_manager/floor_manager_node|/install/robot_floor_manager/lib/robot_floor_manager/floor_manager_node"
      ;;
    global_localization*)
      printf '%s\n' "robot_global_localization/global_localization_node|/install/robot_global_localization/lib/robot_global_localization/global_localization_node"
      ;;
    ranger_mini3_mode_controller*)
      printf '%s\n' "ranger_mini3_mode_controller/mode_controller_node|/install/ranger_mini3_mode_controller/lib/ranger_mini3_mode_controller/mode_controller_node"
      ;;
    *)
      return 1
      ;;
  esac
}

helper_process_running() {
  local pattern="$1"
  pgrep -f "${pattern}" >/dev/null 2>&1
}

local_perception_runtime_config_ready() {
  local restamp_to_now
  local restamp_to_latest_tf
  local require_output_stamp_tf
  local input_reliable
  local input_qos_depth
  local input_transform_use_latest
  local max_output_tf_stamp_age_sec
  local output_stamp_tf_backoff_sec
  local output_stamp_forward_sec
  restamp_to_now="$(timeout 3 ros2 param get /robot_local_perception restamp_to_now 2>/dev/null || true)"
  restamp_to_latest_tf="$(timeout 3 ros2 param get /robot_local_perception restamp_to_latest_tf 2>/dev/null || true)"
  require_output_stamp_tf="$(timeout 3 ros2 param get /robot_local_perception require_output_stamp_tf 2>/dev/null || true)"
  input_reliable="$(timeout 3 ros2 param get /robot_local_perception input_reliable 2>/dev/null || true)"
  input_qos_depth="$(timeout 3 ros2 param get /robot_local_perception input_qos_depth 2>/dev/null || true)"
  input_transform_use_latest="$(timeout 3 ros2 param get /robot_local_perception input_transform_use_latest 2>/dev/null || true)"
  max_output_tf_stamp_age_sec="$(timeout 3 ros2 param get /robot_local_perception max_output_tf_stamp_age_sec 2>/dev/null || true)"
  output_stamp_tf_backoff_sec="$(timeout 3 ros2 param get /robot_local_perception output_stamp_tf_backoff_sec 2>/dev/null || true)"
  output_stamp_forward_sec="$(timeout 3 ros2 param get /robot_local_perception output_stamp_forward_sec 2>/dev/null || true)"
  [[ "${restamp_to_now}" == *"False"* ]] || return 1
  [[ "${restamp_to_latest_tf}" == *"False"* ]] || return 1
  [[ "${require_output_stamp_tf}" == *"False"* ]] || return 1
  [[ "${input_reliable}" == *"False"* ]] || return 1
  [[ "${input_qos_depth}" == *"1"* ]] || return 1
  [[ "${input_transform_use_latest}" == *"True"* ]] || return 1
  [[ "${max_output_tf_stamp_age_sec}" == *"0.25"* ]] || return 1
  [[ "${output_stamp_tf_backoff_sec}" == *"0.0"* ]] || return 1
  [[ "${output_stamp_forward_sec}" == *"0.0"* ]] || return 1
}

helper_ready() {
  # Runtime startup must not depend on ROS graph probes. Reuse is process based;
  # detailed readiness belongs to diagnostics and API goal admission.
  return 0
}

cleanup_stale_overlay_helper() {
  local helper_name="$1"
  local helper_pattern="${2:-}"
  case "${helper_name}" in
    local_perception*)
      [[ -n "${helper_pattern}" ]] && kill_overlay_pattern "${helper_pattern}"
      kill_overlay_pattern "${NJRH_OVERLAY_ROOT}/scripts/run_local_perception.sh"
      ;;
    robot_safety*)
      [[ -n "${helper_pattern}" ]] && kill_overlay_pattern "${helper_pattern}"
      kill_overlay_pattern "${NJRH_OVERLAY_ROOT}/scripts/run_robot_safety.sh"
      ;;
    floor_manager*)
      [[ -n "${helper_pattern}" ]] && kill_overlay_pattern "${helper_pattern}"
      kill_overlay_pattern "${NJRH_OVERLAY_ROOT}/scripts/run_floor_manager.sh"
      ;;
    global_localization*)
      [[ -n "${helper_pattern}" ]] && kill_overlay_pattern "${helper_pattern}"
      kill_overlay_pattern "${NJRH_OVERLAY_ROOT}/scripts/run_global_localization.sh"
      ;;
    ranger_mini3_mode_controller*)
      [[ -n "${helper_pattern}" ]] && kill_overlay_pattern "${helper_pattern}"
      kill_overlay_pattern "${NJRH_OVERLAY_ROOT}/scripts/run_ranger_mini3_mode_controller.sh"
      ;;
    *)
      [[ -n "${helper_pattern}" ]] && kill_overlay_pattern "${helper_pattern}"
      ;;
  esac
}

kill_overlay_pattern() {
  local pattern="$1"
  pkill -INT -f "$pattern" 2>/dev/null || true
  sleep 1
  pkill -9 -f "$pattern" 2>/dev/null || true
}

stop_existing_standard_nav_stack() {
  local patterns=(
    "standard_navigation.launch.py"
    "__node:=keepout_filter_mask_server"
    "__node:=speed_filter_mask_server"
    "__node:=keepout_costmap_filter_info_server"
    "__node:=speed_costmap_filter_info_server"
    "__node:=controller_server"
    "__node:=smoother_server"
    "__node:=planner_server"
    "__node:=behavior_server"
    "__node:=bt_navigator"
    "__node:=waypoint_follower"
    "__node:=velocity_smoother"
    "__node:=collision_monitor"
    "__node:=lifecycle_manager_costmap_filters"
    "__node:=lifecycle_manager_navigation"
  )
  local pattern
  for pattern in "${patterns[@]}"; do
    pkill -INT -f "${pattern}" 2>/dev/null || true
  done
  sleep 1
  for pattern in "${patterns[@]}"; do
    pkill -9 -f "${pattern}" 2>/dev/null || true
  done
}

stop_existing_localization_stack() {
  local patterns=(
    "run_occupancy_grid_localization.sh"
    "occupancy_localization_stack.launch.py"
    "occupancy_localization.launch.py"
    "jt128_occupancy_localization_stack.launch.py"
    "jt128_occupancy_localization.launch.py"
    "occupancy_grid_localizer_container"
    "occupancy_grid_localizer"
    "continuous_flatscan_forwarder.py"
    "isaac_continuous_flatscan_forwarder"
    "map_server"
    "lifecycle_manager_map"
    "pointcloud_to_laserscan_node"
    "pointcloud_to_laserscan"
    "scan_republisher_node"
    "scan_republisher"
    "nav_cloud_preprocessor"
    "run_global_localization.sh"
    "robot_global_localization/global_localization_node"
    "global_localization_node --ros-args"
    "run_localization_bridge.sh"
    "robot_localization_bridge/localization_bridge_node"
    "localization_bridge_node --ros-args"
    "map_to_odom_tf_bridge"
  )
  local signal
  local pattern
  for signal in INT TERM KILL; do
    for pattern in "${patterns[@]}"; do
      pkill "-${signal}" -f "${pattern}" 2>/dev/null || true
    done
    sleep "${NJRH_LOCALIZATION_STACK_STOP_WAIT_SEC:-0.5}"
  done
}

stop_existing_overlay_nav_helpers() {
  if ! force_restart_nav_helpers_enabled; then
    echo "[runtime-overlay] reusing common nav helpers; set NJRH_FORCE_RESTART_NAV_HELPERS=true to restart them" >&2
    return 0
  fi
  kill_overlay_pattern "${NJRH_OVERLAY_ROOT}/scripts/run_local_perception.sh"
  kill_overlay_pattern "/install/robot_local_perception/lib/robot_local_perception/local_perception_node"
  kill_overlay_pattern "robot_local_perception/local_perception_node"
  kill_overlay_pattern "${NJRH_OVERLAY_ROOT}/scripts/run_robot_safety.sh"
  kill_overlay_pattern "/install/robot_safety/lib/robot_safety/robot_safety_node"
  kill_overlay_pattern "robot_safety/robot_safety_node"
  kill_overlay_pattern "${NJRH_OVERLAY_ROOT}/scripts/run_floor_manager.sh"
  kill_overlay_pattern "/install/robot_floor_manager/lib/robot_floor_manager/floor_manager_node"
  kill_overlay_pattern "robot_floor_manager/floor_manager_node"
  kill_overlay_pattern "${NJRH_OVERLAY_ROOT}/scripts/run_global_localization.sh"
  kill_overlay_pattern "/install/robot_global_localization/lib/robot_global_localization/global_localization_node"
  kill_overlay_pattern "robot_global_localization/global_localization_node"
  kill_overlay_pattern "${NJRH_OVERLAY_ROOT}/scripts/run_ranger_mini3_mode_controller.sh"
  kill_overlay_pattern "ranger_mini3_mode_controller/mode_controller_node.py"
  kill_overlay_pattern "python3 .*mode_controller_node.py"
  kill_overlay_pattern "/install/ranger_mini3_mode_controller/lib/ranger_mini3_mode_controller/mode_controller_node"
  kill_overlay_pattern "ranger_mini3_mode_controller/mode_controller_node"
}

start_overlay_helper() {
  local helper_name="$1"
  shift
  local helper_log="${NJRH_RUNTIME_LOG_DIR}/${helper_name}.log"
  local helper_pattern=""
  if helper_pattern="$(helper_process_pattern "${helper_name}")"; then
    if reuse_common_services_enabled && helper_process_running "${helper_pattern}"; then
      if helper_ready "${helper_name}"; then
        echo "[runtime-overlay] reusing existing ${helper_name}; pattern=${helper_pattern}" >&2
        return 0
      fi
      echo "[runtime-overlay] existing ${helper_name} process will be restarted" >&2
      cleanup_stale_overlay_helper "${helper_name}" "${helper_pattern}"
    fi
  fi
  echo "[runtime-overlay] starting ${helper_name}" >&2
  "$@" >>"${helper_log}" 2>&1 &
  local helper_pid=$!
  helper_pids+=("${helper_pid}")
  sleep 1
  if ! kill -0 "${helper_pid}" 2>/dev/null; then
    echo "[runtime-overlay] helper failed to stay alive: ${helper_name}. Check ${helper_log}" >&2
    return 1
  fi
  echo "[runtime-overlay] helper ready: ${helper_name} (pid=${helper_pid})" >&2
}

cleanup_overlay_helpers() {
  local helper_pid
  for helper_pid in "${helper_pids[@]:-}"; do
    kill -INT "${helper_pid}" 2>/dev/null || true
  done
  sleep 1
  for helper_pid in "${helper_pids[@]:-}"; do
    kill -9 "${helper_pid}" 2>/dev/null || true
  done
  helper_pids=()
}

wait_for_ros_service() {
  local service_name="$1"
  local timeout_sec="${2:-30}"
  runtime_readiness_probe service "${service_name}" "${timeout_sec}"
}

wait_for_topic_message() {
  local topic="$1"
  local timeout_sec="${2:-30}"
  if runtime_health_topic_message_ready "${topic}" >/dev/null 2>&1; then
    echo "[runtime-overlay] topic message ready from runtime health snapshot: ${topic}" >&2
    return 0
  fi
  runtime_readiness_probe topic "${topic}" "${timeout_sec}"
}

wait_for_tf_transform() {
  local target_frame="$1"
  local source_frame="$2"
  local timeout_sec="${3:-30}"
  if runtime_health_tf_seen "${target_frame}" "${source_frame}" >/dev/null 2>&1; then
    echo "[runtime-overlay] TF ready from runtime health snapshot: ${target_frame}->${source_frame}" >&2
    return 0
  fi
  runtime_readiness_probe tf "${target_frame}" "${source_frame}" "${timeout_sec}"
}

wait_for_fresh_tf_transform() {
  local target_frame="$1"
  local source_frame="$2"
  local timeout_sec="${3:-30}"
  local max_age_sec="${4:-0.25}"
  if runtime_health_fresh_tf_ready "${target_frame}" "${source_frame}" "${max_age_sec}" >/dev/null 2>&1; then
    echo "[runtime-overlay] fresh TF ready from runtime health snapshot: ${target_frame}->${source_frame}" >&2
    return 0
  fi
  runtime_readiness_probe fresh-tf "${target_frame}" "${source_frame}" "${timeout_sec}" "${max_age_sec}"
}

local_costmap_tf_drop_count() {
  local total=0
  local log_file
  local log_files=(
    "${NJRH_NAVIGATION_RESUME_LOG:-/tmp/njrh_navigation_resume.log}"
    "${NJRH_RUNTIME_LOG_DIR:-/tmp}/nav2_navigation.log"
    "${NJRH_RUNTIME_LOG_DIR:-/tmp}/local_perception.log"
  )
  for log_file in "${log_files[@]}"; do
    [[ -f "${log_file}" ]] || continue
    local count
    count="$(
      grep -c "local_costmap.local_costmap.*Message Filter dropping message" "${log_file}" 2>/dev/null || true
    )"
    count="${count:-0}"
    total=$((total + count))
  done
  printf '%s\n' "${total}"
}

wait_for_transformable_obstacle_points() {
  local timeout_sec="${1:-20}"
  local required_good="${NJRH_LOCAL_COSTMAP_REQUIRED_GOOD_OBSERVATIONS:-3}"
  runtime_readiness_probe transformable-obstacle-points "${timeout_sec}" "${required_good}"
}

wait_for_local_costmap_observation_ready() {
  local timeout_sec="${1:-20}"

  wait_for_topic_message "/perception/obstacle_points" "${timeout_sec}" || {
    echo "[runtime-overlay] obstacle_points_no_publisher_or_message" >&2
    return 1
  }
  wait_for_fresh_tf_transform "odom" "base_link" "${timeout_sec}" "${NJRH_NAV_TF_MAX_AGE_SEC:-0.25}" || {
    echo "[runtime-overlay] odom_base_tf_not_fresh" >&2
    return 1
  }
  wait_for_topic_message "/local_costmap/costmap" "${timeout_sec}" || {
    echo "[runtime-overlay] local_costmap_costmap_no_update" >&2
    return 1
  }
  wait_for_transformable_obstacle_points "${timeout_sec}" || {
    echo "[runtime-overlay] local_costmap_observation_tf_not_transformable" >&2
    return 1
  }

  echo "[runtime-overlay] local costmap observation ready: costmap updates observed and fresh obstacle clouds are TF-valid" >&2
}
