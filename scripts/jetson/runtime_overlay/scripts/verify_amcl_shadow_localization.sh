#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/common_env.sh"

MODE="${NJRH_AMCL_LOCALIZATION_MODE:-disabled}"
DURATION_SEC="${NJRH_VERIFY_AMCL_DURATION_SEC:-30}"
SET_MODE="false"
RESTART="false"
SEED="false"
TF_WARMUP_SEC="${NJRH_AMCL_TF_WARMUP_SEC:-3.0}"
CHECK_LOGS="false"
SCAN_ADMISSION="${NJRH_AMCL_SCAN_ADMISSION_ENABLED:-true}"
FAILURES=()
WARNS=()
PASSES=()

usage() {
  cat <<'USAGE'
Usage: verify_amcl_shadow_localization.sh [--mode disabled|shadow|gated] [--duration-sec N] [--set-mode] [--restart] [--seed] [--tf-warmup-sec N] [--scan-admission] [--check-logs]

Read-only by default. With --restart it restarts only the AMCL helper. With
--seed it calls the bridge seed service and verifies AMCL readiness.
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
    --seed)
      SEED="true"
      shift
      ;;
    --tf-warmup-sec)
      TF_WARMUP_SEC="${2:-3.0}"
      shift 2
      ;;
    --scan-admission)
      SCAN_ADMISSION="true"
      shift
      ;;
    --check-logs)
      CHECK_LOGS="true"
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
  NJRH_AMCL_TF_WARMUP_SEC="${TF_WARMUP_SEC}" \
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

seed_amcl_for_verify() {
  local service="${NJRH_AMCL_SEED_SERVICE:-/robot_localization_bridge/seed_amcl_initial_pose}"
  local retry_count="${NJRH_AMCL_SEED_RETRY_COUNT:-5}"
  local retry_period_ms="${NJRH_AMCL_SEED_RETRY_PERIOD_MS:-300}"
  local attempt
  for ((attempt = 1; attempt <= retry_count; attempt += 1)); do
    if ! timeout 5 ros2 service type "${service}" >/dev/null 2>&1; then
      WARNS+=("AMCL seed service not visible yet attempt=${attempt}/${retry_count}")
    else
      local output
      output="$(timeout 8 ros2 service call "${service}" std_srvs/srv/Trigger "{}" 2>&1 || true)"
      echo "[verify-amcl] seed attempt=${attempt}/${retry_count}: ${output}"
      if grep -Eiq "success[:=][[:space:]]*true|success=True" <<<"${output}"; then
        PASSES+=("AMCL seed service succeeded")
        return 0
      fi
    fi
    sleep "$(awk -v ms="${retry_period_ms}" 'BEGIN {printf "%.3f", ms / 1000.0}')"
  done
  FAILURES+=("AMCL seed service unavailable or failed after retry")
}

fresh_pose_check() {
  local topic="${1:-/amcl_pose}"
  local timeout_sec="${NJRH_AMCL_POSE_FRESH_TIMEOUT_SEC:-5.0}"
  local max_age_sec="${NJRH_AMCL_POSE_MAX_AGE_SEC:-1.0}"
  timeout "$(( ${timeout_sec%.*} + 3 ))" python3 - "${topic}" "${timeout_sec}" "${max_age_sec}" <<'PY' 2>/dev/null
import sys
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped

topic = sys.argv[1]
timeout_sec = float(sys.argv[2])
max_age_sec = float(sys.argv[3])
rclpy.init()
node = rclpy.create_node("verify_amcl_pose_fresh")
state = {"fresh": False, "age": None}

def stamp_to_sec(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9

def on_msg(msg):
    now_sec = node.get_clock().now().nanoseconds * 1.0e-9
    age = now_sec - stamp_to_sec(msg.header.stamp)
    state["age"] = age
    if 0.0 <= age <= max_age_sec:
        state["fresh"] = True

sub = node.create_subscription(PoseWithCovarianceStamped, topic, on_msg, 10)
deadline = node.get_clock().now().nanoseconds + int(timeout_sec * 1.0e9)
while rclpy.ok() and not state["fresh"] and node.get_clock().now().nanoseconds < deadline:
    rclpy.spin_once(node, timeout_sec=0.2)
node.destroy_node()
rclpy.shutdown()
if not state["fresh"]:
    print(f"stale_or_missing age={state['age']}")
    raise SystemExit(1)
print(f"fresh age={state['age']}")
PY
}

check_config_contract() {
  local params_file="${NJRH_AMCL_PARAMS_FILE:-${NJRH_OVERLAY_ROOT}/config/amcl_shadow.yaml}"
  if [[ -f "${params_file}" ]]; then
    grep -Eq 'tf_broadcast:[[:space:]]*false' "${params_file}" &&
      PASSES+=("AMCL config tf_broadcast=false") ||
      FAILURES+=("AMCL config does not force tf_broadcast=false")
    if [[ "${SCAN_ADMISSION}" == "true" ]]; then
      grep -Eq 'scan_topic:[[:space:]]*/scan_amcl' "${params_file}" &&
        PASSES+=("AMCL config scan_topic=/scan_amcl") ||
        FAILURES+=("AMCL config does not use /scan_amcl while scan admission is enabled")
    else
      grep -Eq 'scan_topic:[[:space:]]*/scan' "${params_file}" &&
        PASSES+=("AMCL config scan_topic=/scan") ||
        FAILURES+=("AMCL config does not use /scan")
    fi
    if grep -Eq 'scan_topic:[[:space:]]*/flatscan' "${params_file}"; then
      FAILURES+=("AMCL config uses /flatscan")
    fi
    grep -Eq 'transform_tolerance:[[:space:]]*0.2' "${params_file}" &&
      PASSES+=("AMCL transform_tolerance kept at current default 0.2") ||
      WARNS+=("AMCL transform_tolerance differs from current default; record A/B evidence")
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

  if [[ "${SCAN_ADMISSION}" == "true" && "${MODE}" != "disabled" ]]; then
    local scan_amcl_type
    scan_amcl_type="$(topic_type "${NJRH_AMCL_SCAN_OUTPUT_TOPIC:-/scan_amcl}")"
    if [[ "${scan_amcl_type}" == "sensor_msgs/msg/LaserScan" ]]; then
      PASSES+=("/scan_amcl type is sensor_msgs/msg/LaserScan")
    else
      FAILURES+=("/scan_amcl type unavailable or unexpected: ${scan_amcl_type:-missing}")
    fi
  fi

  local tf_info
  tf_info="$(topic_info /tf)"
  if grep -q "Node name: robot_localization_bridge" <<<"${tf_info}"; then
    PASSES+=("/tf has robot_localization_bridge publisher")
  else
    WARNS+=("/tf publisher info missing robot_localization_bridge")
  fi
  if grep -q "Node name: ${AMCL_NODE_NAME:-amcl}" <<<"${tf_info}"; then
    FAILURES+=("AMCL is publishing /tf; tf_broadcast must remain false")
  else
    PASSES+=("AMCL is not a /tf publisher")
  fi
  if grep -q "Node name: robot_local_state" <<<"${tf_info}"; then
    PASSES+=("/tf has robot_local_state for odom->base_link")
  else
    WARNS+=("/tf publisher info missing robot_local_state")
  fi

  local status
  status="$(bridge_status_once)"
  if [[ -n "${status}" ]]; then
    local owner
    owner="$(bridge_status_field "${status}" map_to_odom_publisher_owner)"
    [[ "${owner}" == "robot_localization_bridge" || -z "${owner}" ]] &&
      PASSES+=("bridge_status owner is robot_localization_bridge or pending") ||
      FAILURES+=("bridge_status map->odom owner is not robot_localization_bridge: ${owner}")
    for field in \
      amcl_input_enabled amcl_gate_mode amcl_pose_count amcl_seed_requested \
      amcl_seed_succeeded amcl_seed_attempt_count amcl_seed_source \
      amcl_seed_last_error amcl_initial_pose_published_count \
      amcl_last_pose_age_ms amcl_pose_hz amcl_ready \
      amcl_scan_admission_enabled amcl_scan_admission_hz \
      amcl_scan_admission_dropped_age_count amcl_scan_admission_dropped_tf_count \
      amcl_scan_frame_id amcl_scan_last_age_ms amcl_message_filter_drop_detected \
      active_correction_source last_accepted_source last_rejected_source; do
      if [[ -n "$(bridge_status_field "${status}" "${field}")" ]]; then
        PASSES+=("bridge_status has ${field}")
      else
        FAILURES+=("bridge_status missing ${field}")
      fi
    done
    if [[ "${MODE}" != "disabled" && "$(bridge_status_field "${status}" amcl_ready)" != "true" ]]; then
      WARNS+=("bridge_status amcl_ready is not true yet")
    fi
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
  if [[ "${SCAN_ADMISSION}" == "true" ]]; then
    [[ "${scan_topic}" == *"/scan_amcl"* ]] &&
      PASSES+=("AMCL runtime scan_topic=/scan_amcl") ||
      FAILURES+=("AMCL runtime scan_topic is not /scan_amcl: ${scan_topic:-missing}")
  else
    [[ "${scan_topic}" == *"/scan"* ]] &&
      PASSES+=("AMCL runtime scan_topic=/scan") ||
      FAILURES+=("AMCL runtime scan_topic is not /scan: ${scan_topic:-missing}")
  fi

  local amcl_info
  amcl_info="$(topic_info /amcl_pose)"
  if grep -q "Publisher count: 0" <<<"${amcl_info}" || [[ -z "${amcl_info}" ]]; then
    WARNS+=("/amcl_pose has no publisher yet; AMCL may be waiting for initial pose")
  else
    PASSES+=("/amcl_pose has publisher")
  fi

  if [[ "${SEED}" == "true" ]]; then
    sleep "${TF_WARMUP_SEC}"
    seed_amcl_for_verify
  fi

  if fresh_pose_check /amcl_pose; then
    PASSES+=("/amcl_pose is fresh")
  else
    FAILURES+=("/amcl_pose is stale or missing")
  fi

  local sample_window=5
  if [[ "${DURATION_SEC}" =~ ^[0-9]+$ && "${DURATION_SEC}" -lt 12 ]]; then
    sample_window="${DURATION_SEC}"
  fi
  local scan_hz
  local scan_amcl_hz=""
  local amcl_hz
  scan_hz="$(topic_hz /scan "${sample_window}")"
  if [[ "${SCAN_ADMISSION}" == "true" ]]; then
    scan_amcl_hz="$(topic_hz "${NJRH_AMCL_SCAN_OUTPUT_TOPIC:-/scan_amcl}" "${sample_window}")"
  fi
  amcl_hz="$(topic_hz /amcl_pose "${sample_window}")"
  echo "[verify-amcl] /scan hz=${scan_hz:-unavailable}"
  [[ "${SCAN_ADMISSION}" == "true" ]] && echo "[verify-amcl] /scan_amcl hz=${scan_amcl_hz:-unavailable}"
  echo "[verify-amcl] /amcl_pose hz=${amcl_hz:-unavailable}"
}

check_logs() {
  [[ "${CHECK_LOGS}" == "true" ]] || return 0
  local log_file="${NJRH_AMCL_LOG_FILE:-${NJRH_RUNTIME_LOG_DIR}/amcl_shadow_localization.log}"
  [[ -f "${log_file}" ]] || {
    WARNS+=("AMCL log file missing: ${log_file}")
    return 0
  }
  local tail_text
  tail_text="$(tail -n "${NJRH_AMCL_VERIFY_LOG_TAIL_LINES:-300}" "${log_file}" 2>/dev/null || true)"
  if grep -Eq "Please set the initial pose|Message Filter dropping message|earlier than all (the )?data|transform timeout" <<<"${tail_text}"; then
    FAILURES+=("AMCL log tail still contains seed or MessageFilter drop errors")
  else
    PASSES+=("AMCL log tail has no persistent seed/MessageFilter errors")
  fi
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

echo "[verify-amcl] mode=${MODE} duration_sec=${DURATION_SEC} scan_admission=${SCAN_ADMISSION}"
check_config_contract
check_runtime_contract
check_amcl_runtime

if [[ "${MODE}" != "disabled" && "${DURATION_SEC}" =~ ^[0-9]+$ && "${DURATION_SEC}" -gt 0 ]]; then
  echo "[verify-amcl] observing AMCL for ${DURATION_SEC}s"
  sleep "${DURATION_SEC}"
  fresh_pose_check /amcl_pose >/dev/null 2>&1 &&
    PASSES+=("/amcl_pose remained fresh after observation") ||
    FAILURES+=("/amcl_pose became stale during observation")
fi

check_logs
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
