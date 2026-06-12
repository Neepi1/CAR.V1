#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/common_env.sh"

MODE="${NJRH_AMCL_LOCALIZATION_MODE:-disabled}"
DURATION_SEC="${NJRH_VERIFY_AMCL_DURATION_SEC:-30}"
SET_MODE="false"
RESTART="false"
FAILURES=()
WARNS=()
PASSES=()

usage() {
  cat <<'USAGE'
Usage: verify_amcl_shadow_localization.sh [--mode disabled|shadow|gated] [--duration-sec N] [--set-mode] [--restart]

Read-only by default. With --restart it restarts only the AMCL helper.
USAGE
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --mode)
      MODE="${2:-}"
      shift 2
      ;;
    --duration-sec)
      DURATION_SEC="${2:-30}"
      shift 2
      ;;
    --set-mode)
      SET_MODE="true"
      shift
      ;;
    --restart)
      RESTART="true"
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "[verify-amcl] unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

case "${MODE}" in
  disabled|shadow|gated)
    ;;
  *)
    echo "[verify-amcl] invalid mode=${MODE}; expected disabled, shadow, or gated" >&2
    exit 2
    ;;
esac

if [[ "${SET_MODE}" == "true" ]]; then
  export NJRH_AMCL_LOCALIZATION_MODE="${MODE}"
fi

if [[ "${RESTART}" == "true" ]]; then
  bash "${SCRIPT_DIR}/run_amcl_shadow_localization.sh" --mode "${MODE}" --restart
fi

topic_info() {
  local topic="$1"
  timeout 6 ros2 topic info -v "${topic}" 2>&1 || true
}

topic_type() {
  local topic="$1"
  timeout 6 ros2 topic type "${topic}" 2>/dev/null || true
}

topic_hz() {
  local topic="$1"
  local duration="${2:-5}"
  local output
  output="$(timeout "$((duration + 4))" ros2 topic hz "${topic}" --window 10 2>/dev/null || true)"
  awk '/average rate:/ {rate=$3} END {print rate}' <<<"${output}"
}

bridge_status_once() {
  timeout 8 python3 - <<'PY' 2>/dev/null || true
import rclpy
from std_msgs.msg import String

rclpy.init()
node = rclpy.create_node("verify_amcl_bridge_status_once")
result = {"data": ""}

def on_msg(msg):
    result["data"] = msg.data

sub = node.create_subscription(String, "/localization/bridge_status", on_msg, 10)
deadline = node.get_clock().now().nanoseconds + 7_000_000_000
while rclpy.ok() and not result["data"] and node.get_clock().now().nanoseconds < deadline:
    rclpy.spin_once(node, timeout_sec=0.2)
print(result["data"])
node.destroy_node()
rclpy.shutdown()
PY
}

bridge_status_field() {
  local status="$1"
  local field="$2"
  BRIDGE_STATUS_JSON="${status}" python3 - "${field}" <<'PY'
import json
import os
import sys

field = sys.argv[1]
try:
    data = json.loads(os.environ.get("BRIDGE_STATUS_JSON", ""))
except Exception:
    print("")
    raise SystemExit(0)
value = data.get(field, "")
if isinstance(value, bool):
    print("true" if value else "false")
else:
    print(value)
PY
}

param_value() {
  local node="$1"
  local param="$2"
  timeout 5 ros2 param get "${node}" "${param}" 2>/dev/null || true
}

node_exists() {
  local node="$1"
  timeout 5 ros2 node list 2>/dev/null | grep -Fxq "${node}"
}

process_exists() {
  local pattern="$1"
  pgrep -af "${pattern}" >/dev/null 2>&1
}

check_config_contract() {
  local params_file="${NJRH_AMCL_PARAMS_FILE:-${NJRH_OVERLAY_ROOT}/config/amcl_shadow.yaml}"
  if [[ -f "${params_file}" ]]; then
    grep -Eq 'tf_broadcast:[[:space:]]*false' "${params_file}" &&
      PASSES+=("AMCL config tf_broadcast=false") ||
      FAILURES+=("AMCL config does not force tf_broadcast=false")
    grep -Eq 'scan_topic:[[:space:]]*/scan' "${params_file}" &&
      PASSES+=("AMCL config scan_topic=/scan") ||
      FAILURES+=("AMCL config does not use /scan")
    if grep -Eq 'scan_topic:[[:space:]]*/flatscan' "${params_file}"; then
      FAILURES+=("AMCL config uses /flatscan")
    fi
  else
    FAILURES+=("AMCL params file missing: ${params_file}")
  fi
}

check_runtime_contract() {
  local scan_type
  scan_type="$(topic_type /scan)"
  if [[ "${scan_type}" == "sensor_msgs/msg/LaserScan" ]]; then
    PASSES+=("/scan type is sensor_msgs/msg/LaserScan")
  else
    WARNS+=("/scan type unavailable or unexpected: ${scan_type:-missing}")
  fi

  local tf_info
  tf_info="$(topic_info /tf)"
  if grep -q "Node name: robot_localization_bridge" <<<"${tf_info}"; then
    PASSES+=("/tf has robot_localization_bridge publisher")
  else
    WARNS+=("/tf publisher info missing robot_localization_bridge")
  fi
  if grep -q "Node name: ${AMCL_NODE_NAME:-amcl}" <<<"${tf_info}"; then
    WARNS+=("AMCL has a /tf endpoint; runtime tf_broadcast parameter is checked separately")
  else
    PASSES+=("AMCL is not a /tf publisher")
  fi

  local status
  status="$(bridge_status_once)"
  if [[ -n "${status}" ]]; then
    local owner
    owner="$(bridge_status_field "${status}" map_to_odom_publisher_owner)"
    [[ "${owner}" == "robot_localization_bridge" || -z "${owner}" ]] &&
      PASSES+=("bridge_status owner is robot_localization_bridge or pending") ||
      FAILURES+=("bridge_status map->odom owner is not robot_localization_bridge: ${owner}")
    for field in amcl_input_enabled amcl_gate_mode amcl_pose_count active_correction_source last_accepted_source last_rejected_source; do
      if [[ -n "$(bridge_status_field "${status}" "${field}")" ]]; then
        PASSES+=("bridge_status has ${field}")
      else
        FAILURES+=("bridge_status missing ${field}")
      fi
    done
  else
    FAILURES+=("/localization/bridge_status unavailable")
  fi
}

check_amcl_runtime() {
  if [[ "${MODE}" == "disabled" ]]; then
    if node_exists "/${AMCL_NODE_NAME:-amcl}" || process_exists "nav2_amcl.*amcl"; then
      FAILURES+=("AMCL is running while mode=disabled")
    else
      PASSES+=("AMCL not running while mode=disabled")
    fi
    return 0
  fi

  local lifecycle
  lifecycle="$(timeout 5 ros2 lifecycle get "/${AMCL_NODE_NAME:-amcl}" 2>/dev/null || true)"
  if [[ "${lifecycle}" == active* ]]; then
    PASSES+=("AMCL lifecycle active")
  else
    if node_exists "/${AMCL_NODE_NAME:-amcl}"; then
      WARNS+=("AMCL lifecycle not active or unavailable: ${lifecycle:-unknown}")
    else
      FAILURES+=("/${AMCL_NODE_NAME:-amcl} node missing for mode=${MODE}")
    fi
  fi

  local tf_broadcast
  tf_broadcast="$(param_value "/${AMCL_NODE_NAME:-amcl}" tf_broadcast)"
  if [[ "${tf_broadcast}" == *"False"* || "${tf_broadcast}" == *"false"* ]]; then
    PASSES+=("AMCL runtime tf_broadcast=false")
  else
    FAILURES+=("AMCL runtime tf_broadcast is not false: ${tf_broadcast:-missing}")
  fi

  local scan_topic
  scan_topic="$(param_value "/${AMCL_NODE_NAME:-amcl}" scan_topic)"
  if [[ "${scan_topic}" == *"/scan"* ]]; then
    PASSES+=("AMCL runtime scan_topic=/scan")
  else
    FAILURES+=("AMCL runtime scan_topic is not /scan: ${scan_topic:-missing}")
  fi

  local amcl_info
  amcl_info="$(topic_info /amcl_pose)"
  if grep -q "Publisher count: 0" <<<"${amcl_info}" || [[ -z "${amcl_info}" ]]; then
    WARNS+=("/amcl_pose has no publisher yet; AMCL may be waiting for initial pose")
  else
    PASSES+=("/amcl_pose has publisher")
  fi

  local sample_window=5
  if [[ "${DURATION_SEC}" =~ ^[0-9]+$ && "${DURATION_SEC}" -lt 12 ]]; then
    sample_window="${DURATION_SEC}"
  fi
  local scan_hz
  local amcl_hz
  scan_hz="$(topic_hz /scan "${sample_window}")"
  amcl_hz="$(topic_hz /amcl_pose "${sample_window}")"
  echo "[verify-amcl] /scan hz=${scan_hz:-unavailable}"
  echo "[verify-amcl] /amcl_pose hz=${amcl_hz:-unavailable}"
}

check_nav2_context() {
  local local_frame
  local_frame="$(timeout 5 ros2 param get /local_costmap/local_costmap global_frame 2>/dev/null || true)"
  if [[ "${local_frame}" == *"odom"* ]]; then
    PASSES+=("local_costmap.global_frame remains odom")
  else
    WARNS+=("local_costmap.global_frame unavailable or not odom: ${local_frame:-missing}")
  fi

  local fastlio
  fastlio="$(
    ps -eo pid=,args= |
      grep -E "fast_lio[[:space:]]+fastlio_mapping|fastlio_mapping --ros-args|laser_mapping" |
      grep -v grep |
      grep -v "verify_amcl_shadow_localization" || true
  )"
  if [[ -z "${fastlio}" ]]; then
    PASSES+=("FAST-LIO2 navigation residual not detected")
  else
    FAILURES+=("FAST-LIO2 navigation residual detected: ${fastlio}")
  fi
}

echo "[verify-amcl] mode=${MODE} duration_sec=${DURATION_SEC}"
check_config_contract
check_runtime_contract
check_amcl_runtime
check_nav2_context

if ((${#PASSES[@]} > 0)); then
  printf '[verify-amcl] PASS: %s\n' "${PASSES[@]}"
fi
if ((${#WARNS[@]} > 0)); then
  printf '[verify-amcl] WARN: %s\n' "${WARNS[@]}"
fi
if ((${#FAILURES[@]} > 0)); then
  printf '[verify-amcl] FAIL: %s\n' "${FAILURES[@]}"
  exit 1
fi

echo "[verify-amcl] PASS"
