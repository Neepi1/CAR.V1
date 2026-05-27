#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/common_env.sh"

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
      printf '%s\n' "robot_global_localization/global_localization_node.py|/install/robot_global_localization/lib/robot_global_localization/global_localization_node.py"
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
  kill_overlay_pattern "/install/robot_global_localization/lib/robot_global_localization/global_localization_node.py"
  kill_overlay_pattern "robot_global_localization/global_localization_node.py"
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
      echo "[runtime-overlay] reusing existing ${helper_name}; pattern=${helper_pattern}" >&2
      return 0
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
  local start_ts now elapsed
  start_ts="$(date +%s)"
  while true; do
    if ros2 service list 2>/dev/null | grep -qx "${service_name}"; then
      echo "[runtime-overlay] service ready: ${service_name}" >&2
      return 0
    fi
    now="$(date +%s)"
    elapsed=$((now - start_ts))
    if [[ "${elapsed}" -ge "${timeout_sec}" ]]; then
      echo "[runtime-overlay] timed out waiting for service: ${service_name}" >&2
      return 1
    fi
    sleep 1
  done
}

wait_for_topic_message() {
  local topic="$1"
  local timeout_sec="${2:-30}"
  timeout "${timeout_sec}" ros2 topic echo "${topic}" --once >/dev/null 2>&1
}

wait_for_tf_transform() {
  local target_frame="$1"
  local source_frame="$2"
  local timeout_sec="${3:-30}"
  python3 - "${target_frame}" "${source_frame}" "${timeout_sec}" <<'PY'
import sys
import time

import rclpy
from rclpy.duration import Duration
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

target = sys.argv[1]
source = sys.argv[2]
timeout_sec = float(sys.argv[3])

rclpy.init()
node = rclpy.create_node("wait_for_tf_transform")
buffer = Buffer()
listener = TransformListener(buffer, node)
deadline = time.monotonic() + timeout_sec
ok = False
last_error = ""

try:
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        try:
            if buffer.can_transform(target, source, Time(), Duration(seconds=0.2)):
                ok = True
                break
        except Exception as exc:  # noqa: BLE001 - diagnostic path
            last_error = str(exc)
finally:
    node.destroy_node()
    rclpy.shutdown()

if not ok:
    if last_error:
        print(f"[runtime-overlay] timed out waiting for TF {target}->{source}: {last_error}", file=sys.stderr)
    else:
        print(f"[runtime-overlay] timed out waiting for TF {target}->{source}", file=sys.stderr)
    sys.exit(1)

print(f"[runtime-overlay] TF ready: {target}->{source}", file=sys.stderr)
PY
}
