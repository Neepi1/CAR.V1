#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/common_env.sh"

canonical_helper_pids=()

kill_canonical_pattern() {
  local pattern="$1"
  pkill -INT -f "$pattern" 2>/dev/null || true
  sleep 1
  pkill -9 -f "$pattern" 2>/dev/null || true
}

stop_existing_canonical_tf_publishers() {
  kill_canonical_pattern "hesai_lidar_state_publisher"
  kill_canonical_pattern "${NJRH_OVERLAY_ROOT}/scripts/run_robot_description.sh"
  kill_canonical_pattern "robot_description_static_tf_node.py"
  kill_canonical_pattern "${NJRH_OVERLAY_ROOT}/scripts/run_local_state.sh"
  kill_canonical_pattern "local_state_node.py"
  kill_canonical_pattern "${NJRH_OVERLAY_ROOT}/scripts/run_localization_bridge.sh"
  kill_canonical_pattern "localization_bridge_node.py"
  kill_canonical_pattern "map_to_odom_tf_bridge"
}

start_canonical_helper() {
  local helper_name="$1"
  shift
  local helper_log="${NJRH_RUNTIME_LOG_DIR}/${helper_name}.log"
  mkdir -p "${NJRH_RUNTIME_LOG_DIR}"
  if [[ -e "${helper_log}" && ! -w "${helper_log}" ]]; then
    rm -f "${helper_log}" 2>/dev/null || {
      echo "[runtime-overlay] helper log is not writable and could not be removed: ${helper_log}" >&2
      return 1
    }
  fi
  : >"${helper_log}" 2>/dev/null || {
    echo "[runtime-overlay] helper log is not writable: ${helper_log}" >&2
    return 1
  }
  echo "[runtime-overlay] starting ${helper_name}" >&2
  "$@" >>"${helper_log}" 2>&1 &
  local helper_pid=$!
  canonical_helper_pids+=("${helper_pid}")
  sleep 1
  if ! kill -0 "${helper_pid}" 2>/dev/null; then
    echo "[runtime-overlay] helper failed to stay alive: ${helper_name}. Check ${helper_log}" >&2
    return 1
  fi
  echo "[runtime-overlay] helper ready: ${helper_name} (pid=${helper_pid})" >&2
}

cleanup_canonical_helpers() {
  local helper_pid
  for helper_pid in "${canonical_helper_pids[@]:-}"; do
    kill -INT "${helper_pid}" 2>/dev/null || true
  done
  sleep 1
  for helper_pid in "${canonical_helper_pids[@]:-}"; do
    kill -9 "${helper_pid}" 2>/dev/null || true
  done
  canonical_helper_pids=()
}

require_can_interface_up() {
  local can_interface="${CAN_INTERFACE:-can0}"
  local operstate_file="/sys/class/net/${can_interface}/operstate"
  [[ -r "${operstate_file}" ]] || {
    echo "[runtime-overlay] CAN interface not found: ${can_interface}" >&2
    return 1
  }

  local operstate
  operstate="$(tr -d '[:space:]' < "${operstate_file}")"
  if [[ "${operstate}" != "up" ]]; then
    echo "[runtime-overlay] CAN interface ${can_interface} is ${operstate}." >&2
    echo "[runtime-overlay] Bring it up on the Jetson host first:" >&2
    echo "  sudo bash ${NJRH_UPSTREAM_HOST_ROOT}/scripts/bringup_ranger_can_host.sh" >&2
    return 1
  fi
}

wait_for_topic_message() {
  local topic_name="$1"
  local timeout_sec="${2:-10}"
  timeout "${timeout_sec}s" ros2 topic echo --once "${topic_name}" >/dev/null 2>&1
}

wait_for_tf_edge() {
  local parent_frame="$1"
  local child_frame="$2"
  local timeout_sec="${3:-10}"
  python3 - "${parent_frame}" "${child_frame}" "${timeout_sec}" <<'PY'
import sys
import time

import rclpy
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

parent_frame = sys.argv[1]
child_frame = sys.argv[2]
timeout_sec = float(sys.argv[3])

rclpy.init()
node = rclpy.create_node('wait_for_tf_edge')
buffer = Buffer()
listener = TransformListener(buffer, node, spin_thread=False)
deadline = time.time() + timeout_sec
success = False

try:
    while time.time() < deadline and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.2)
        try:
            if buffer.can_transform(parent_frame, child_frame, Time()):
                success = True
                break
        except Exception:
            pass
finally:
    del listener
    node.destroy_node()
    rclpy.shutdown()

sys.exit(0 if success else 1)
PY
}
