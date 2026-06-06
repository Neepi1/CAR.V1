#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/canonical_tf_helpers.sh"
source "${SCRIPT_DIR}/cpu_affinity.sh"

UPSTREAM_SLAM_SCRIPT="${NJRH_UPSTREAM_ROOT}/scripts/run_jt128_2d_mapping.sh"
USE_UPSTREAM_SLAM_SCRIPT="${NJRH_SLAM2D_USE_UPSTREAM_SCRIPT:-false}"
SLAM_LAUNCH_FILE="${NJRH_SLAM2D_LAUNCH_FILE:-${NJRH_OVERLAY_ROOT}/launch/jt128_slam_toolbox_mapping.launch.py}"
SLAM_PARAMS_FILE="${NJRH_SLAM2D_CONFIG:-${NJRH_OVERLAY_ROOT}/config/jt128_slam_toolbox_mapping.yaml}"
SCAN_PARAMS_FILE="${NJRH_SLAM2D_SCAN_CONFIG:-${NJRH_OVERLAY_ROOT}/config/jt128_scan_slam2d.yaml}"
FASTLIO_CONFIG_FILE="${NJRH_SLAM2D_FASTLIO_CONFIG:-${NJRH_OVERLAY_ROOT}/config/fastlio.yaml}"
POINTS_TOPIC="${NJRH_SLAM2D_POINTS_TOPIC:-/cloud_registered_body}"
SLAM2D_ODOM_SOURCE="${NJRH_SLAM2D_ODOM_SOURCE:-fastlio}"
FASTLIO_ODOM_TOPIC="${NJRH_SLAM2D_FASTLIO_ODOM_TOPIC:-/Odometry}"
FASTLIO_ODOM_READY_TIMEOUT="${NJRH_SLAM2D_FASTLIO_ODOM_READY_TIMEOUT:-60}"
SLAM2D_FASTLIO_ODOM_FRAME="${NJRH_SLAM2D_FASTLIO_ODOM_FRAME:-mapping_odom}"
SLAM2D_PRIVATE_TF_TOPIC="${NJRH_SLAM2D_PRIVATE_TF_TOPIC:-/tf_slam2d}"
SLAM2D_ALLOW_PRIVATE_FASTLIO="${NJRH_SLAM2D_ALLOW_PRIVATE_FASTLIO:-true}"
SLAM2D_REUSE_EXISTING_FASTLIO="${NJRH_SLAM2D_REUSE_EXISTING_FASTLIO:-false}"
SLAM2D_PRIVATE_FASTLIO_PID_FILE="${NJRH_SLAM2D_PRIVATE_FASTLIO_PID_FILE:-/tmp/njrh_slam2d_private_fastlio.pid}"
LOCAL_ODOM_READY_TIMEOUT="${NJRH_SLAM2D_ODOM_READY_TIMEOUT:-30}"
LOCAL_ODOM_MAX_AGE_SEC="${NJRH_SLAM2D_LOCAL_ODOM_MAX_AGE_SEC:-1.0}"
FASTLIO_POINTS_READY_TIMEOUT="${NJRH_SLAM2D_FASTLIO_POINTS_READY_TIMEOUT:-60}"
FASTLIO_POINTS_MAX_AGE_SEC="${NJRH_SLAM2D_FASTLIO_POINTS_MAX_AGE_SEC:-1.0}"
LOCAL_ODOM_MAX_WHEEL_DIFF_M="${NJRH_SLAM2D_LOCAL_ODOM_MAX_WHEEL_DIFF_M:-25.0}"

# Mode scripts can be launched by a long-lived API process that still carries
# older concrete CPU-set environment values. Re-derive this mapping path from
# the group defaults here, with explicit per-mode override knobs.
export NJRH_CPUSET_FASTLIO_DESKEW="${NJRH_SLAM2D_FASTLIO_CPUSET:-${NJRH_CPUSET_MAPPING_FRONTEND:-6,7}}"
export NJRH_CPUSET_SLAM_TOOLBOX_MAPPING="${NJRH_SLAM2D_SLAM_TOOLBOX_CPUSET:-${NJRH_CPUSET_MAPPING_BACKEND:-7}}"

for pattern in \
  "run_jt128_2d_mapping.sh" \
  "projected_occupancy_mapper.py" \
  "occupancy_builder_live_node.py" \
  "robot_occupancy_builder_live" \
  "frontend_pose_from_odometry.py" \
  "slam_toolbox" \
  "jt128_2d_mapping.launch.py" \
  "jt128_slam_toolbox_mapping.launch.py" \
  "nav_cloud_preprocessor" \
  "pointcloud_to_laserscan_node" \
  "robot_hesai_jt128/scan_republisher_node" \
  "scan_republisher_node" \
  "fastlio_mapping_odom_bridge.py" \
  "fastlio_odom_bridge_node.*mapping/fastlio_odometry"
do
  pkill -INT -f "$pattern" 2>/dev/null || true
done
sleep 1
for pattern in \
  "run_jt128_2d_mapping.sh" \
  "projected_occupancy_mapper.py" \
  "occupancy_builder_live_node.py" \
  "robot_occupancy_builder_live" \
  "frontend_pose_from_odometry.py" \
  "slam_toolbox" \
  "jt128_2d_mapping.launch.py" \
  "jt128_slam_toolbox_mapping.launch.py" \
  "nav_cloud_preprocessor" \
  "pointcloud_to_laserscan_node" \
  "robot_hesai_jt128/scan_republisher_node" \
  "scan_republisher_node" \
  "fastlio_mapping_odom_bridge.py" \
  "fastlio_odom_bridge_node.*mapping/fastlio_odometry"
do
  pkill -9 -f "$pattern" 2>/dev/null || true
done

require_can_interface_up

projected_map_pid=""
fastlio_deskew_pid=""
fastlio_odom_bridge_pid=""
fastlio_reused_for_slam2d="false"
projected_map_exit_code=0

private_fastlio_pid_is_owned() {
  local pid="$1"
  [[ -n "${pid}" && "${pid}" =~ ^[0-9]+$ ]] || return 1
  [[ -r "/proc/${pid}/environ" && -r "/proc/${pid}/cmdline" ]] || return 1
  tr '\0' '\n' <"/proc/${pid}/environ" | grep -qx "NJRH_SLAM2D_PRIVATE_FASTLIO=1" || return 1
  tr '\0' ' ' <"/proc/${pid}/cmdline" | grep -q "fast_lio" || return 1
  tr '\0' ' ' <"/proc/${pid}/cmdline" | grep -q "fastlio_mapping" || return 1
}

stop_fastlio_deskew_sources() {
  local pid=""
  if [[ ! -f "${SLAM2D_PRIVATE_FASTLIO_PID_FILE}" ]]; then
    return 0
  fi
  read -r pid <"${SLAM2D_PRIVATE_FASTLIO_PID_FILE}" || true
  if [[ -z "${pid}" || ! "${pid}" =~ ^[0-9]+$ || ! -e "/proc/${pid}" ]]; then
    rm -f "${SLAM2D_PRIVATE_FASTLIO_PID_FILE}" 2>/dev/null || true
    return 0
  fi
  if ! private_fastlio_pid_is_owned "${pid}"; then
    echo "[runtime-overlay] refusing to stop FAST-LIO2 pid=${pid}: missing slam2d private marker" >&2
    rm -f "${SLAM2D_PRIVATE_FASTLIO_PID_FILE}" 2>/dev/null || true
    return 0
  fi
  terminate_child "${pid}" "private FAST-LIO2 slam2d deskew source"
  rm -f "${SLAM2D_PRIVATE_FASTLIO_PID_FILE}" 2>/dev/null || true
}

stop_mapping_fastlio_processes() {
  local pids=()
  local proc pid
  for proc in /proc/[0-9]*; do
    [[ -r "${proc}/environ" && -r "${proc}/cmdline" ]] || continue
    pid="${proc##*/}"
    tr '\0' '\n' <"${proc}/environ" | grep -qx "NJRH_SLAM2D_PRIVATE_FASTLIO=1" || continue
    tr '\0' ' ' <"${proc}/cmdline" | grep -Eq "fastlio_mapping|laser_mapping" || continue
    pids+=("${pid}")
  done
  [[ ${#pids[@]} -gt 0 ]] || return 0
  for pid in "${pids[@]}"; do
    kill -INT "${pid}" 2>/dev/null || true
  done
  sleep "${NJRH_SLAM2D_FASTLIO_STOP_INT_WAIT_SEC:-1}"
  for pid in "${pids[@]}"; do
    kill -TERM "${pid}" 2>/dev/null || true
  done
  sleep "${NJRH_SLAM2D_FASTLIO_STOP_TERM_WAIT_SEC:-1}"
  for pid in "${pids[@]}"; do
    kill -9 "${pid}" 2>/dev/null || true
  done
}

terminate_child() {
  local pid="$1"
  local label="$2"
  local int_wait_steps="${3:-20}"
  if [[ -z "${pid}" ]] || ! kill -0 "${pid}" 2>/dev/null; then
    return 0
  fi

  kill -INT "${pid}" 2>/dev/null || true
  for _ in $(seq 1 "${int_wait_steps}"); do
    if ! kill -0 "${pid}" 2>/dev/null; then
      wait "${pid}" 2>/dev/null || true
      return 0
    fi
    sleep 0.1
  done

  echo "[runtime-overlay] ${label} did not exit after SIGINT; sending SIGTERM" >&2
  kill -TERM "${pid}" 2>/dev/null || true
  for _ in $(seq 1 10); do
    if ! kill -0 "${pid}" 2>/dev/null; then
      wait "${pid}" 2>/dev/null || true
      return 0
    fi
    sleep 0.1
  done

  echo "[runtime-overlay] ${label} did not exit after SIGTERM; sending SIGKILL" >&2
  kill -9 "${pid}" 2>/dev/null || true
  wait "${pid}" 2>/dev/null || true
}

check_local_state_odom_sane() {
  local local_topic="$1"
  local wheel_topic="$2"
  local timeout_sec="${3:-4}"
  local max_diff_m="${4:-25.0}"
  python3 - "${local_topic}" "${wheel_topic}" "${timeout_sec}" "${max_diff_m}" <<'PY'
import math
import sys
import time

import rclpy
from nav_msgs.msg import Odometry

local_topic = sys.argv[1]
wheel_topic = sys.argv[2]
timeout_sec = float(sys.argv[3])
max_diff_m = float(sys.argv[4])

rclpy.init()
node = rclpy.create_node("check_local_state_odom_sane")
latest = {}

def callback(name):
    def _cb(msg):
        latest[name] = msg
    return _cb

subs = [
    node.create_subscription(Odometry, local_topic, callback("local"), 10),
    node.create_subscription(Odometry, wheel_topic, callback("wheel"), 10),
]
deadline = time.monotonic() + timeout_sec

try:
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if "local" in latest and "wheel" in latest:
            break
    if "local" not in latest or "wheel" not in latest:
        print(
            f"[runtime-overlay] local odom sanity failed: missing samples "
            f"local={'local' in latest} wheel={'wheel' in latest}",
            file=sys.stderr,
        )
        sys.exit(1)

    local = latest["local"].pose.pose.position
    wheel = latest["wheel"].pose.pose.position
    values = (local.x, local.y, wheel.x, wheel.y)
    if not all(math.isfinite(v) for v in values):
        print("[runtime-overlay] local odom sanity failed: non-finite pose", file=sys.stderr)
        sys.exit(1)
    diff = math.hypot(local.x - wheel.x, local.y - wheel.y)
    if diff > max_diff_m:
        print(
            f"[runtime-overlay] local odom sanity failed: {local_topic} differs from "
            f"{wheel_topic} by {diff:.3f}m, max={max_diff_m:.3f}m",
            file=sys.stderr,
        )
        sys.exit(1)
    print(
        f"[runtime-overlay] local odom sanity ok: {local_topic} vs {wheel_topic} diff={diff:.3f}m",
        file=sys.stderr,
    )
finally:
    for sub in subs:
        node.destroy_subscription(sub)
    node.destroy_node()
    rclpy.shutdown()
PY
}

require_resident_common_mapping_prereqs() {
  local common_mode="${NJRH_NAV_LOCAL_STATE_MODE:-ekf}"
  local local_odom_reference_topic="/wheel/odom"
  case "${common_mode}" in
    fastlio)
      local_odom_reference_topic="/fastlio/base_odometry"
      ;;
    ekf)
      local_odom_reference_topic="/wheel/odom_ekf"
      ;;
    passthrough|legacy)
      local_odom_reference_topic="/wheel/odom"
      ;;
    *)
      echo "[runtime-overlay] invalid NJRH_NAV_LOCAL_STATE_MODE=${common_mode}; expected fastlio, ekf, passthrough, or legacy" >&2
      return 1
      ;;
  esac

  echo "[runtime-overlay] verifying resident common runtime before slam_toolbox mapping" >&2
  wait_for_tf_edge "base_link" "lidar_level_link" 10 || {
    echo "[runtime-overlay] resident static TF base_link -> lidar_level_link is not ready" >&2
    return 1
  }
  LOCAL_STATE_MODE="${common_mode}" local_state_endpoint_ready "${LOCAL_ODOM_READY_TIMEOUT}" || {
    echo "[runtime-overlay] resident robot_local_state endpoint is not ready; start common services before mapping" >&2
    return 1
  }
  wait_for_topic_publisher "/local_state/odometry" "${LOCAL_ODOM_READY_TIMEOUT}" || {
    echo "[runtime-overlay] resident /local_state/odometry publisher is not ready for slam_toolbox mapping" >&2
    return 1
  }
  wait_for_fresh_header_topic_message "/local_state/odometry" "${LOCAL_ODOM_READY_TIMEOUT}" "${LOCAL_ODOM_MAX_AGE_SEC}" 0.25 || {
    echo "[runtime-overlay] resident /local_state/odometry is stale; refusing to start mapping" >&2
    return 1
  }
  wait_for_topic_publisher "${local_odom_reference_topic}" "${LOCAL_ODOM_READY_TIMEOUT}" || {
    echo "[runtime-overlay] resident local-state reference topic ${local_odom_reference_topic} is not ready" >&2
    return 1
  }
  check_local_state_odom_sane "/local_state/odometry" "${local_odom_reference_topic}" 4 "${LOCAL_ODOM_MAX_WHEEL_DIFF_M}" || {
    echo "[runtime-overlay] refusing to start slam_toolbox mapping with unhealthy resident local odometry" >&2
    return 1
  }
}

cleanup() {
  trap - EXIT INT TERM
  if [[ -n "${projected_map_pid}" ]]; then
    terminate_child "${projected_map_pid}" "slam_toolbox mapping process"
  fi
  if [[ -n "${fastlio_odom_bridge_pid}" ]]; then
    terminate_child "${fastlio_odom_bridge_pid}" "FAST-LIO2 mapping odom bridge"
  fi
  if [[ -n "${fastlio_deskew_pid}" ]]; then
    terminate_child "${fastlio_deskew_pid}" "FAST-LIO2 deskew source"
    stop_fastlio_deskew_sources
  fi
  if [[ "${fastlio_reused_for_slam2d}" != "true" ]]; then
    stop_mapping_fastlio_processes
  fi
  cleanup_canonical_helpers
}

on_signal() {
  cleanup
  exit 130
}

trap cleanup EXIT
trap on_signal INT TERM

require_resident_common_mapping_prereqs || exit 1

if [[ "${USE_UPSTREAM_SLAM_SCRIPT}" == "true" && -f "${UPSTREAM_SLAM_SCRIPT}" ]]; then
  bash -lc "PUBLISH_LIDAR_TF=false bash '${UPSTREAM_SLAM_SCRIPT}'" &
  projected_map_pid=$!
  wait "${projected_map_pid}" || projected_map_exit_code=$?
  exit "${projected_map_exit_code}"
fi

for required_file in "${SLAM_LAUNCH_FILE}" "${SLAM_PARAMS_FILE}" "${SCAN_PARAMS_FILE}"; do
  [[ -f "${required_file}" ]] || {
    echo "[runtime-overlay] missing slam_toolbox runtime file: ${required_file}" >&2
    exit 1
  }
done
if [[ ! -f "${FASTLIO_CONFIG_FILE}" ]]; then
  echo "[runtime-overlay] missing mapping FAST-LIO2 config: ${FASTLIO_CONFIG_FILE}" >&2
  exit 1
fi

if [[ "${SLAM2D_ODOM_SOURCE}" != "fastlio" && "${SLAM2D_ODOM_SOURCE}" != "local_state" ]]; then
  echo "[runtime-overlay] invalid NJRH_SLAM2D_ODOM_SOURCE=${SLAM2D_ODOM_SOURCE}; expected fastlio or local_state" >&2
  exit 1
fi

fastlio_log="${NJRH_RUNTIME_LOG_DIR}/fastlio_slam2d_deskew.log"
if [[ "${SLAM2D_REUSE_EXISTING_FASTLIO}" == "true" ]] &&
  wait_for_fresh_header_topic_message "${POINTS_TOPIC}" 2 "${FASTLIO_POINTS_MAX_AGE_SEC}" 0.25; then
  fastlio_reused_for_slam2d="true"
  echo "[runtime-overlay] reusing existing FAST-LIO2 mapping source: ${POINTS_TOPIC}" >&2
else
  if [[ "${SLAM2D_ALLOW_PRIVATE_FASTLIO}" != "true" ]]; then
    echo "[runtime-overlay] FAST-LIO2 is required for mapping and mapping-owned startup is disabled" >&2
    exit 1
  fi
  stop_fastlio_deskew_sources
  if [[ "${SLAM2D_REUSE_EXISTING_FASTLIO}" != "true" ]]; then
    stop_mapping_fastlio_processes
  fi
  : >"${fastlio_log}" 2>/dev/null || {
    echo "[runtime-overlay] FAST-LIO log is not writable: ${fastlio_log}" >&2
    exit 1
  }
  echo "[runtime-overlay] starting mapping-owned FAST-LIO2 deskew source for slam_toolbox: ${POINTS_TOPIC}" >&2
  export NJRH_SLAM2D_PRIVATE_FASTLIO=1
  njrh_run_affined fastlio_deskew ros2 run fast_lio fastlio_mapping \
    --ros-args \
    --params-file "${FASTLIO_CONFIG_FILE}" \
    -p use_sim_time:=false \
    -r /tf:=/tf_fastlio_internal \
    -r /tf_static:=/tf_static_fastlio_internal >>"${fastlio_log}" 2>&1 &
  fastlio_deskew_pid=$!
  unset NJRH_SLAM2D_PRIVATE_FASTLIO
  printf '%s\n' "${fastlio_deskew_pid}" >"${SLAM2D_PRIVATE_FASTLIO_PID_FILE}"
  sleep 2
  if ! kill -0 "${fastlio_deskew_pid}" 2>/dev/null; then
    rm -f "${SLAM2D_PRIVATE_FASTLIO_PID_FILE}" 2>/dev/null || true
    echo "[runtime-overlay] FAST-LIO2 deskew source failed to stay alive. Check ${fastlio_log}" >&2
    exit 1
  fi
fi

wait_for_fresh_header_topic_message "${POINTS_TOPIC}" "${FASTLIO_POINTS_READY_TIMEOUT}" "${FASTLIO_POINTS_MAX_AGE_SEC}" 0.25 || {
  echo "[runtime-overlay] timed out waiting for FAST-LIO2 deskewed pointcloud: ${POINTS_TOPIC}" >&2
  echo "[runtime-overlay] check mapping-owned FAST-LIO2 and canonical /lidar_points + /lidar_imu input streams." >&2
  exit 1
}

slam_odom_frame="odom"
slam_tf_topic="/tf"
if [[ "${SLAM2D_ODOM_SOURCE}" == "fastlio" ]]; then
  bridge_bin="${NJRH_PROJECT_ROOT}/install/robot_fastlio_mapping/lib/robot_fastlio_mapping/fastlio_odom_bridge_node"
  [[ -x "${bridge_bin}" ]] || {
    echo "[runtime-overlay] missing compiled FAST-LIO mapping odom bridge: ${bridge_bin}" >&2
    exit 1
  }
  wait_for_topic_message "${FASTLIO_ODOM_TOPIC}" "${FASTLIO_ODOM_READY_TIMEOUT}" || {
    echo "[runtime-overlay] timed out waiting for FAST-LIO2 odometry: ${FASTLIO_ODOM_TOPIC}" >&2
    echo "[runtime-overlay] check ${fastlio_log}; slam_toolbox mapping is configured to use FAST-LIO odom." >&2
    exit 1
  }
  echo "[runtime-overlay] using FAST-LIO2 mapping odom from ${FASTLIO_ODOM_TOPIC} on private TF ${SLAM2D_PRIVATE_TF_TOPIC}" >&2
  njrh_run_affined fastlio_odom_bridge "${bridge_bin}" \
    --ros-args \
    -p input_topic:="${FASTLIO_ODOM_TOPIC}" \
    -p output_topic:=/mapping/fastlio_odometry \
    -p tf_topic:="${SLAM2D_PRIVATE_TF_TOPIC}" \
    -p output_odom_frame:="${SLAM2D_FASTLIO_ODOM_FRAME}" \
    -p output_base_frame:=base_link \
    -p sensor_frame:=lidar_link \
    -p anchor_on_first_sample:=true \
    -p flatten_to_2d:=true \
    -p publish_tf:=true \
    -p input_reliable:=false \
    -p input_qos_depth:=1 \
    -p output_reliable:=true \
    -p output_qos_depth:=20 &
  fastlio_odom_bridge_pid=$!
  wait_for_topic_message "/mapping/fastlio_odometry" 10 || {
    echo "[runtime-overlay] FAST-LIO mapping odom bridge did not publish /mapping/fastlio_odometry" >&2
    exit 1
  }
  slam_odom_frame="${SLAM2D_FASTLIO_ODOM_FRAME}"
  slam_tf_topic="${SLAM2D_PRIVATE_TF_TOPIC}"
fi

ros2 launch "${SLAM_LAUNCH_FILE}" \
  slam_params:="${SLAM_PARAMS_FILE}" \
  scan_params:="${SCAN_PARAMS_FILE}" \
  points_topic:="${POINTS_TOPIC}" \
  odom_frame:="${slam_odom_frame}" \
  tf_topic:="${slam_tf_topic}" &
projected_map_pid=$!
wait "${projected_map_pid}" || projected_map_exit_code=$?
exit "${projected_map_exit_code}"
