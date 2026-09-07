#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/canonical_tf_helpers.sh"
source "${SCRIPT_DIR}/cpu_affinity.sh"
source "${SCRIPT_DIR}/scan_ownership_helpers.sh"
source "${SCRIPT_DIR}/mapping_fastdds_transport.sh"

UPSTREAM_SLAM_SCRIPT="${NJRH_UPSTREAM_ROOT}/scripts/run_jt128_2d_mapping.sh"
USE_UPSTREAM_SLAM_SCRIPT="${NJRH_SLAM2D_USE_UPSTREAM_SCRIPT:-false}"
SLAM_LAUNCH_FILE="${NJRH_SLAM2D_LAUNCH_FILE:-${NJRH_OVERLAY_ROOT}/launch/jt128_slam_toolbox_mapping.launch.py}"
SLAM_PARAMS_FILE="${NJRH_SLAM2D_CONFIG:-${NJRH_OVERLAY_ROOT}/config/jt128_slam_toolbox_mapping.yaml}"
PREPROCESSOR_PARAMS_FILE="${NJRH_SLAM2D_PREPROCESSOR_CONFIG:-${NJRH_OVERLAY_ROOT}/config/jt128_nav_cloud_preprocessor.yaml}"
SCAN_PARAMS_FILE="${NJRH_SLAM2D_SCAN_CONFIG:-${NJRH_OVERLAY_ROOT}/config/jt128_scan_slam2d.yaml}"
SLAM2D_SCAN_TOPIC="${NJRH_SLAM2D_SCAN_TOPIC:-/scan}"
SLAM2D_SCAN_READY_TIMEOUT="${NJRH_SLAM2D_SCAN_READY_TIMEOUT:-15}"
SLAM2D_SCAN_OWNER_READY_TIMEOUT="${NJRH_SLAM2D_SCAN_OWNER_READY_TIMEOUT:-30}"
SLAM2D_SCAN_MAX_AGE_SEC="${NJRH_SLAM2D_SCAN_MAX_AGE_SEC:-0.5}"
SLAM2D_STAMPED_TF_READY_TIMEOUT="${NJRH_SLAM2D_STAMPED_TF_READY_TIMEOUT:-30}"
SLAM2D_STAMPED_TF_REQUIRED_GOOD="${NJRH_SLAM2D_STAMPED_TF_REQUIRED_GOOD:-3}"
FASTLIO_CONFIG_FILE="${NJRH_SLAM2D_FASTLIO_CONFIG:-${NJRH_OVERLAY_ROOT}/config/fastlio.yaml}"
POINTS_TOPIC="${NJRH_SLAM2D_POINTS_TOPIC:-/mapping/fastlio/cloud_registered_body}"
NAV_POINTS_TOPIC="${NJRH_SLAM2D_NAV_POINTS_TOPIC:-/mapping/points_nav}"
SLAM2D_ODOM_SOURCE="${NJRH_SLAM2D_ODOM_SOURCE:-fastlio}"
FASTLIO_ODOM_TOPIC="${NJRH_SLAM2D_FASTLIO_ODOM_TOPIC:-/mapping/fastlio/odometry}"
FASTLIO_ODOM_READY_TIMEOUT="${NJRH_SLAM2D_FASTLIO_ODOM_READY_TIMEOUT:-60}"
FASTLIO_PAIR_READY_TIMEOUT="${NJRH_SLAM2D_FASTLIO_PAIR_READY_TIMEOUT:-8}"
FASTLIO_PAIR_PROBE="${NJRH_SLAM2D_FASTLIO_PAIR_PROBE:-${NJRH_OVERLAY_ROOT}/scripts/verify_mapping_fastlio_pair.py}"
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
SLAM2D_LIDAR_RPS_XPS_ENABLED="${NJRH_SLAM2D_LIDAR_RPS_XPS_ENABLED:-true}"
SLAM2D_LIDAR_RPS_XPS_INTERFACE="${NJRH_SLAM2D_LIDAR_RPS_XPS_INTERFACE:-${NJRH_LIDAR_INTERFACE:-eth1}}"
SLAM2D_LIDAR_RPS_XPS_CPUSET="${NJRH_SLAM2D_LIDAR_RPS_XPS_CPUSET:-5}"
SLAM2D_LIDAR_RPS_XPS_STATE_DIR="${NJRH_SLAM2D_LIDAR_RPS_XPS_STATE_DIR:-/tmp/njrh_slam2d_lidar_rps_xps}"
SLAM2D_COMPOSITE_PREFLIGHT_ENABLED="${NJRH_SLAM2D_COMPOSITE_PREFLIGHT_ENABLED:-true}"
MAPPING_STARTUP_STARTED_SECONDS="${SECONDS}"

# Mode scripts can be launched by a long-lived API process that still carries
# older concrete CPU-set environment values. Re-derive this mapping path from
# the mapping-mode defaults here, with explicit per-mode override knobs.
export NJRH_CPUSET_FASTLIO_DESKEW="${NJRH_SLAM2D_FASTLIO_CPUSET:-${NJRH_CPUSET_MAPPING_BACKEND:-7}}"
export NJRH_CPUSET_SLAM_TOOLBOX_MAPPING="${NJRH_SLAM2D_SLAM_TOOLBOX_CPUSET:-3,7}"

log_mapping_startup_stage() {
  local stage="$1"
  echo "[runtime-overlay] MAPPING_STARTUP_STAGE stage=${stage} elapsed_sec=$((SECONDS - MAPPING_STARTUP_STARTED_SECONDS))" >&2
}

log_mapping_startup_stage "script_started"

require_mapping_ros_package() {
  python3 - <<'PY'
import sys

from ament_index_python.packages import PackageNotFoundError, get_package_prefix

try:
    get_package_prefix("robot_fastlio_mapping")
except PackageNotFoundError as exc:
    print(
        "[runtime-overlay] robot_fastlio_mapping is not visible in AMENT_PREFIX_PATH; "
        "rebuild the package and reload the workspace environment before starting mapping",
        file=sys.stderr,
    )
    print(f"[runtime-overlay] ament lookup detail: {exc}", file=sys.stderr)
    raise SystemExit(1)
PY
}

# Fail before stopping or starting any process. The mapping odometry bridge is
# provided by this package, so the long-lived API environment must resolve it.
require_mapping_ros_package || exit 1

stop_legacy_mapping_processes() {
  local candidate_pattern="run_jt128_2d_mapping[.]sh|projected_occupancy_mapper[.]py|occupancy_builder_live_node[.]py|robot_occupancy_builder_live|frontend_pose_from_odometry[.]py|slam_toolbox|jt128_2d_mapping[.]launch[.]py|jt128_slam_toolbox_mapping[.]launch[.]py|mapping_scan_tf_gate_node|fastlio_mapping_odom_bridge[.]py|fastlio_odom_bridge_node.*mapping/fastlio_odometry"
  local candidates=()
  local pids=()
  local pid
  local alive
  mapfile -t candidates < <(pgrep -f "${candidate_pattern}" 2>/dev/null || true)
  for pid in "${candidates[@]}"; do
    [[ "${pid}" != "$$" && "${pid}" != "${PPID}" ]] || continue
    pids+=("${pid}")
  done
  [[ ${#pids[@]} -gt 0 ]] || return 0

  echo "[runtime-overlay] stopping ${#pids[@]} legacy mapping process(es)" >&2
  for pid in "${pids[@]}"; do
    kill -INT "${pid}" 2>/dev/null || true
  done
  for _ in {1..10}; do
    alive=0
    for pid in "${pids[@]}"; do
      if kill -0 "${pid}" 2>/dev/null; then
        alive=1
        break
      fi
    done
    [[ "${alive}" -eq 1 ]] || return 0
    sleep 0.1
  done
  for pid in "${pids[@]}"; do
    kill -9 "${pid}" 2>/dev/null || true
  done
}

stop_legacy_mapping_processes

require_can_interface_up
log_mapping_startup_stage "legacy_processes_stopped"

projected_map_pid=""
fastlio_deskew_pid=""
fastlio_odom_bridge_pid=""
fastlio_reused_for_slam2d="false"
projected_map_exit_code=0
slam2d_lidar_rps_xps_applied="false"
resident_scan_released="false"

slam2d_truthy() {
  case "${1:-}" in
    1|true|TRUE|yes|YES|on|ON) return 0 ;;
    *) return 1 ;;
  esac
}

slam2d_cpulist_to_hex_mask() {
  local cpulist="$1"
  local mask=0
  local part start end cpu
  IFS=',' read -ra parts <<<"${cpulist}"
  for part in "${parts[@]}"; do
    [[ -n "${part}" ]] || continue
    if [[ "${part}" == *-* ]]; then
      start="${part%-*}"
      end="${part#*-}"
      [[ "${start}" =~ ^[0-9]+$ && "${end}" =~ ^[0-9]+$ && "${start}" -le "${end}" ]] || return 1
      for ((cpu=start; cpu<=end; cpu++)); do
        mask=$((mask | (1 << cpu)))
      done
    else
      cpu="${part}"
      [[ "${cpu}" =~ ^[0-9]+$ ]] || return 1
      mask=$((mask | (1 << cpu)))
    fi
  done
  printf '%x\n' "${mask}"
}

backup_slam2d_lidar_rps_xps() {
  local iface="$1"
  mkdir -p "${SLAM2D_LIDAR_RPS_XPS_STATE_DIR}"
  if [[ -f "${SLAM2D_LIDAR_RPS_XPS_STATE_DIR}/manifest.env" ]]; then
    echo "[runtime-overlay] keeping existing slam2d LiDAR RPS/XPS backup: ${SLAM2D_LIDAR_RPS_XPS_STATE_DIR}" >&2
    return 0
  fi
  {
    printf 'created_utc=%q\n' "$(date -u +%Y%m%dT%H%M%SZ)"
    printf 'interface=%q\n' "${iface}"
    printf 'cpuset=%q\n' "${SLAM2D_LIDAR_RPS_XPS_CPUSET}"
  } >"${SLAM2D_LIDAR_RPS_XPS_STATE_DIR}/manifest.env"

  : >"${SLAM2D_LIDAR_RPS_XPS_STATE_DIR}/rps_xps.tsv"
  local file value
  for file in /sys/class/net/"${iface}"/queues/rx-*/rps_cpus /sys/class/net/"${iface}"/queues/tx-*/xps_cpus; do
    [[ -e "${file}" && -r "${file}" ]] || continue
    value="$(cat "${file}" 2>/dev/null || true)"
    printf '%s\t%s\n' "${file}" "${value}" >>"${SLAM2D_LIDAR_RPS_XPS_STATE_DIR}/rps_xps.tsv"
  done
}

apply_slam2d_lidar_rps_xps() {
  if ! slam2d_truthy "${SLAM2D_LIDAR_RPS_XPS_ENABLED}"; then
    echo "[runtime-overlay] slam2d LiDAR RPS/XPS profile disabled" >&2
    return 0
  fi
  local iface="${SLAM2D_LIDAR_RPS_XPS_INTERFACE}"
  if [[ -z "${iface}" || ! -d "/sys/class/net/${iface}" ]]; then
    echo "[runtime-overlay] slam2d LiDAR RPS/XPS skipped; interface not found: ${iface:-unset}" >&2
    return 0
  fi
  local mask
  if ! mask="$(slam2d_cpulist_to_hex_mask "${SLAM2D_LIDAR_RPS_XPS_CPUSET}")"; then
    echo "[runtime-overlay] slam2d LiDAR RPS/XPS skipped; invalid cpuset=${SLAM2D_LIDAR_RPS_XPS_CPUSET}" >&2
    return 0
  fi

  backup_slam2d_lidar_rps_xps "${iface}"

  local file changed=0 failed=0
  for file in /sys/class/net/"${iface}"/queues/rx-*/rps_cpus /sys/class/net/"${iface}"/queues/tx-*/xps_cpus; do
    [[ -e "${file}" ]] || continue
    if printf '%s\n' "${mask}" >"${file}" 2>/dev/null; then
      changed=$((changed + 1))
      echo "[runtime-overlay] slam2d LiDAR RPS/XPS applied ${file}=${mask} cpuset=${SLAM2D_LIDAR_RPS_XPS_CPUSET}" >&2
    else
      failed=$((failed + 1))
      echo "[runtime-overlay] warning: cannot write slam2d LiDAR RPS/XPS file ${file}" >&2
    fi
  done
  if [[ "${changed}" -gt 0 ]]; then
    slam2d_lidar_rps_xps_applied="true"
  fi
  [[ "${failed}" -eq 0 ]] || return 0
}

restore_slam2d_lidar_rps_xps() {
  [[ -d "${SLAM2D_LIDAR_RPS_XPS_STATE_DIR}" ]] || return 0
  [[ -f "${SLAM2D_LIDAR_RPS_XPS_STATE_DIR}/rps_xps.tsv" ]] || return 0
  local file value failed=0
  while IFS=$'\t' read -r file value; do
    [[ -n "${file}" && -e "${file}" ]] || continue
    if printf '%s\n' "${value}" >"${file}" 2>/dev/null; then
      echo "[runtime-overlay] restored slam2d LiDAR RPS/XPS ${file}=${value}" >&2
    else
      failed=$((failed + 1))
      echo "[runtime-overlay] warning: failed to restore slam2d LiDAR RPS/XPS ${file}=${value}" >&2
    fi
  done <"${SLAM2D_LIDAR_RPS_XPS_STATE_DIR}/rps_xps.tsv"
  if [[ "${failed}" -eq 0 ]]; then
    rm -rf "${SLAM2D_LIDAR_RPS_XPS_STATE_DIR}" 2>/dev/null || true
  fi
}

private_fastlio_pid_is_owned() {
  local pid="$1"
  [[ -n "${pid}" && "${pid}" =~ ^[0-9]+$ ]] || return 1
  [[ -r "/proc/${pid}/environ" && -r "/proc/${pid}/cmdline" ]] || return 1
  tr '\0' '\n' <"/proc/${pid}/environ" | grep -qx "NJRH_SLAM2D_PRIVATE_FASTLIO=1" || return 1
  tr '\0' ' ' <"/proc/${pid}/cmdline" | grep -q "fast_lio" || return 1
  tr '\0' ' ' <"/proc/${pid}/cmdline" | grep -q "fastlio_mapping" || return 1
}

process_has_exact_env_marker() {
  local pid="$1"
  local marker="$2"
  local entry
  [[ "${pid}" =~ ^[0-9]+$ && -r "/proc/${pid}/environ" ]] || return 1
  while IFS= read -r -d '' entry; do
    [[ "${entry}" == "${marker}" ]] && return 0
  done <"/proc/${pid}/environ"
  return 1
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
  local candidates=()
  local pid
  local candidate_pattern="fastlio_mapping|laser_mapping"
  mapfile -t candidates < <(pgrep -f "${candidate_pattern}" 2>/dev/null || true)
  for pid in "${candidates[@]}"; do
    [[ "${pid}" != "$$" && "${pid}" != "${PPID}" ]] || continue
    process_has_exact_env_marker "${pid}" "NJRH_SLAM2D_PRIVATE_FASTLIO=1" || continue
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

stop_mapping_pipeline_processes() {
  local pids=()
  local candidates=()
  local pid
  local candidate_pattern="mapping_scan_tf_gate_node|pointcloud_to_laserscan|nav_cloud_preprocessor|slam_toolbox"
  mapfile -t candidates < <(pgrep -f "${candidate_pattern}" 2>/dev/null || true)
  for pid in "${candidates[@]}"; do
    [[ "${pid}" != "$$" && "${pid}" != "${PPID}" ]] || continue
    process_has_exact_env_marker "${pid}" "NJRH_SLAM2D_MAPPING_PIPELINE=1" || continue
    pids+=("${pid}")
  done
  [[ ${#pids[@]} -gt 0 ]] || return 0
  echo "[runtime-overlay] stopping ${#pids[@]} mapping scan-pipeline process(es)" >&2
  for pid in "${pids[@]}"; do
    kill -INT "${pid}" 2>/dev/null || true
  done
  sleep "${NJRH_SLAM2D_PIPELINE_STOP_INT_WAIT_SEC:-0.5}"
  for pid in "${pids[@]}"; do
    kill -TERM "${pid}" 2>/dev/null || true
  done
  sleep "${NJRH_SLAM2D_PIPELINE_STOP_TERM_WAIT_SEC:-0.5}"
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

require_resident_common_mapping_prereqs_legacy() {
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
  wait_for_topic_publisher_from_node "${SLAM2D_SCAN_TOPIC}" "pointcloud_accel_axis_node" "${SLAM2D_SCAN_READY_TIMEOUT}" || {
    echo "[runtime-overlay] canonical JT128 scan owner is not ready on ${SLAM2D_SCAN_TOPIC}" >&2
    return 1
  }
  wait_for_fresh_header_topic_message "${SLAM2D_SCAN_TOPIC}" "${SLAM2D_SCAN_READY_TIMEOUT}" "${SLAM2D_SCAN_MAX_AGE_SEC}" 0.25 || {
    echo "[runtime-overlay] canonical JT128 scan is stale on ${SLAM2D_SCAN_TOPIC}" >&2
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

  if [[ "${SLAM2D_COMPOSITE_PREFLIGHT_ENABLED}" != "true" ]]; then
    echo "[runtime-overlay] using explicit legacy mapping preflight" >&2
    require_resident_common_mapping_prereqs_legacy
    return $?
  fi

  echo "[runtime-overlay] verifying resident common runtime with one composite DDS participant" >&2
  runtime_readiness_probe mapping-preflight \
    "${SLAM2D_SCAN_TOPIC}" \
    "pointcloud_accel_axis_node" \
    "/local_state/odometry" \
    "${local_odom_reference_topic}" \
    "${common_mode}" \
    10 \
    "${SLAM2D_SCAN_READY_TIMEOUT}" \
    "${LOCAL_ODOM_READY_TIMEOUT}" \
    "${SLAM2D_SCAN_MAX_AGE_SEC}" \
    "${LOCAL_ODOM_MAX_AGE_SEC}" \
    0.25 \
    "${LOCAL_ODOM_MAX_WHEEL_DIFF_M}" || {
    echo "[runtime-overlay] refusing to start slam_toolbox mapping because composite preflight failed" >&2
    return 1
  }
}

cleanup() {
  trap - EXIT INT TERM
  stop_scan_owner_observer
  if [[ -n "${projected_map_pid}" ]]; then
    terminate_child "${projected_map_pid}" "slam_toolbox mapping process"
  fi
  stop_mapping_pipeline_processes
  if [[ "${resident_scan_released}" == "true" ]]; then
    if wait_for_scan_publisher_count "${SLAM2D_SCAN_TOPIC}" 0; then
      :
    else
      echo "[runtime-overlay] warning: mapping /scan publisher did not leave the graph before navigation restore" >&2
    fi
    if restore_navigation_scan_owner "${SLAM2D_SCAN_TOPIC}"; then
      resident_scan_released="false"
      echo "[runtime-overlay] canonical /scan ownership returned to ${RESIDENT_SCAN_OWNER_NODE}" >&2
    else
      echo "[runtime-overlay] CRITICAL canonical navigation /scan ownership could not be restored; runtime remains fail-closed" >&2
    fi
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
  restore_slam2d_lidar_rps_xps
  cleanup_mapping_fastdds_transport
  cleanup_canonical_helpers
}

on_signal() {
  cleanup
  exit 130
}

trap cleanup EXIT
trap on_signal INT TERM

configure_mapping_fastdds_transport || exit 1
stop_mapping_pipeline_processes
log_mapping_startup_stage "mapping_pipeline_stopped"
# Self-heal an interrupted previous handoff before validating the resident
# navigation source.  This is idempotent and still requires exact graph proof.
restore_navigation_scan_owner "${SLAM2D_SCAN_TOPIC}" || exit 1
require_resident_common_mapping_prereqs || exit 1
wait_for_scan_owner "${SLAM2D_SCAN_TOPIC}" "${RESIDENT_SCAN_OWNER_NODE}" 1 || {
  echo "[runtime-overlay] refusing mapping handoff: canonical resident /scan ownership is not unique" >&2
  exit 1
}
log_mapping_startup_stage "mapping_preflight_ready"
apply_slam2d_lidar_rps_xps

if [[ "${USE_UPSTREAM_SLAM_SCRIPT}" == "true" && -f "${UPSTREAM_SLAM_SCRIPT}" ]]; then
  bash -lc "PUBLISH_LIDAR_TF=false bash '${UPSTREAM_SLAM_SCRIPT}'" &
  projected_map_pid=$!
  wait "${projected_map_pid}" || projected_map_exit_code=$?
  exit "${projected_map_exit_code}"
fi

for required_file in \
  "${SLAM_LAUNCH_FILE}" \
  "${PREPROCESSOR_PARAMS_FILE}" \
  "${SCAN_PARAMS_FILE}" \
  "${SLAM_PARAMS_FILE}" \
  "${FASTLIO_PAIR_PROBE}"
do
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
  log_mapping_startup_stage "mapping_fastlio_residuals_stopped"
  : >"${fastlio_log}" 2>/dev/null || {
    echo "[runtime-overlay] FAST-LIO log is not writable: ${fastlio_log}" >&2
    exit 1
  }
  echo "[runtime-overlay] starting mapping-owned FAST-LIO2 deskew source for slam_toolbox: ${POINTS_TOPIC}" >&2
  export NJRH_SLAM2D_PRIVATE_FASTLIO=1
  njrh_start_affined_background fastlio_deskew_pid \
    fastlio_deskew env -u FASTDDS_BUILTIN_TRANSPORTS \
    FASTRTPS_DEFAULT_PROFILES_FILE="${NJRH_SLAM2D_FASTDDS_ACTIVE_PROFILE_FILE}" \
    FASTDDS_DEFAULT_PROFILES_FILE="${NJRH_SLAM2D_FASTDDS_ACTIVE_PROFILE_FILE}" \
    NJRH_SLAM2D_FASTDDS_ROLE=large_cloud \
    ros2 run fast_lio fastlio_mapping \
    --ros-args \
    --params-file "${FASTLIO_CONFIG_FILE}" \
    -p use_sim_time:=false \
    -r /cloud_registered_body:="${POINTS_TOPIC}" \
    -r /Odometry:="${FASTLIO_ODOM_TOPIC}" \
    -r /tf:=/tf_fastlio_internal \
    -r /tf_static:=/tf_static_fastlio_internal >>"${fastlio_log}" 2>&1
  unset NJRH_SLAM2D_PRIVATE_FASTLIO
  printf '%s\n' "${fastlio_deskew_pid}" >"${SLAM2D_PRIVATE_FASTLIO_PID_FILE}"
  log_mapping_startup_stage "fastlio_started"
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
  python3 "${FASTLIO_PAIR_PROBE}" \
    "${POINTS_TOPIC}" "${FASTLIO_ODOM_TOPIC}" \
    --timeout-sec "${FASTLIO_PAIR_READY_TIMEOUT}" || {
    echo "[runtime-overlay] refusing cross-instance FAST-LIO mapping cloud/odom wiring" >&2
    exit 1
  }
  log_mapping_startup_stage "fastlio_ready"
  echo "[runtime-overlay] using FAST-LIO2 mapping odom from ${FASTLIO_ODOM_TOPIC} on private TF ${SLAM2D_PRIVATE_TF_TOPIC}" >&2
  njrh_start_affined_background fastlio_odom_bridge_pid \
    fastlio_odom_bridge "${bridge_bin}" \
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
    -p output_qos_depth:=20
  wait_for_topic_message "/mapping/fastlio_odometry" 10 || {
    echo "[runtime-overlay] FAST-LIO mapping odom bridge did not publish /mapping/fastlio_odometry" >&2
    exit 1
  }
  log_mapping_startup_stage "odom_bridge_ready"
  slam_odom_frame="${SLAM2D_FASTLIO_ODOM_FRAME}"
  slam_tf_topic="${SLAM2D_PRIVATE_TF_TOPIC}"
fi

# Atomically surrender canonical /scan only after the corrected cloud and its
# odometry/TF chain are ready.  reset() in the resident service removes its
# publisher from the graph while /lidar_points and FAST-LIO2 continue running.
set_resident_scan_output false
resident_scan_released="true"
wait_for_scan_publisher_count "${SLAM2D_SCAN_TOPIC}" 0 || {
  echo "[runtime-overlay] refusing mapping scan startup: resident /scan publisher did not leave the graph" >&2
  exit 1
}
log_mapping_startup_stage "resident_scan_released"

# Join the graph before the mapping publisher exists.  Under Fast DDS load a
# participant created after the endpoint may temporarily receive a unique
# endpoint with UNKNOWN node metadata; the pre-armed observer sees the endpoint
# discovery and its ROS node identity in one continuous graph session.
start_scan_owner_observer \
  "${SLAM2D_SCAN_TOPIC}" "pointcloud_to_laserscan" 1 \
  "${SLAM2D_SCAN_OWNER_READY_TIMEOUT}"

ros2 launch "${SLAM_LAUNCH_FILE}" \
  preprocessor_params:="${PREPROCESSOR_PARAMS_FILE}" \
  scan_params:="${SCAN_PARAMS_FILE}" \
  slam_params:="${SLAM_PARAMS_FILE}" \
  points_topic:="${POINTS_TOPIC}" \
  nav_points_topic:="${NAV_POINTS_TOPIC}" \
  scan_topic:="${SLAM2D_SCAN_TOPIC}" \
  odom_frame:="${slam_odom_frame}" \
  tf_topic:="${slam_tf_topic}" &
projected_map_pid=$!
wait_for_scan_owner_observer || {
  echo "[runtime-overlay] refusing mapping runtime: corrected-cloud /scan owner is not unique" >&2
  exit 1
}
wait_for_fresh_header_topic_message "${SLAM2D_SCAN_TOPIC}" "${SLAM2D_SCAN_READY_TIMEOUT}" "${SLAM2D_SCAN_MAX_AGE_SEC}" 0.25 || {
  echo "[runtime-overlay] corrected-cloud mapping scan is stale on ${SLAM2D_SCAN_TOPIC}" >&2
  exit 1
}
log_mapping_startup_stage "mapping_scan_owner_ready"

# Prove the original FAST-LIO2 stamp is transformable; no relay or restamping
# node exists in this mapping path.
runtime_readiness_probe stamped-scan-tf \
  "${SLAM2D_SCAN_TOPIC}" \
  "${slam_tf_topic}" \
  "${slam_odom_frame}" \
  "${SLAM2D_STAMPED_TF_READY_TIMEOUT}" \
  "${SLAM2D_STAMPED_TF_REQUIRED_GOOD}" || {
  echo "[runtime-overlay] refusing mapping runtime: corrected /scan is not transformable through ${slam_tf_topic} to ${slam_odom_frame} at its original timestamp" >&2
  exit 1
}
log_mapping_startup_stage "stamped_scan_tf_ready"
log_mapping_startup_stage "slam_toolbox_started"
wait "${projected_map_pid}" || projected_map_exit_code=$?
exit "${projected_map_exit_code}"
