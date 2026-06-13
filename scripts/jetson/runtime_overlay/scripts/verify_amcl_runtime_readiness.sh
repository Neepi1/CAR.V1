#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/common_env.sh"

MODE="${NJRH_AMCL_LOCALIZATION_MODE:-gated}"
DURATION_SEC=30
SET_MODE=false
RESTART_RUNTIME=false
DO_SEED=false
CHECK_TRIGGERED=false
CHECK_AMCL=false
CHECK_OWNER=false
CHECK_MOTION_WINDOW=false

usage() {
  cat <<'USAGE'
Usage: verify_amcl_runtime_readiness.sh [options]

Options:
  --mode disabled|shadow|gated
  --seed
  --duration-sec N
  --check-triggered
  --check-amcl
  --check-owner
  --check-motion-window
  --set-mode
  --restart-runtime

This is a read-mostly AMCL readiness check. It does not send navigation goals.
--seed calls the bridge AMCL seed service and AMCL /request_nomotion_update so a
stationary robot can produce one /amcl_pose sample.
USAGE
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --mode)
      MODE="${2:-}"
      shift 2
      ;;
    --duration-sec)
      DURATION_SEC="${2:-}"
      shift 2
      ;;
    --seed)
      DO_SEED=true
      shift
      ;;
    --check-triggered)
      CHECK_TRIGGERED=true
      shift
      ;;
    --check-amcl)
      CHECK_AMCL=true
      shift
      ;;
    --check-owner)
      CHECK_OWNER=true
      shift
      ;;
    --check-motion-window)
      CHECK_MOTION_WINDOW=true
      shift
      ;;
    --set-mode)
      SET_MODE=true
      shift
      ;;
    --restart-runtime)
      RESTART_RUNTIME=true
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "[verify-amcl-runtime] unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

case "${MODE}" in
  disabled|shadow|gated)
    ;;
  *)
    echo "[verify-amcl-runtime] --mode must be disabled, shadow, or gated" >&2
    exit 2
    ;;
esac
case "${DURATION_SEC}" in
  ''|*[!0-9]*)
    echo "[verify-amcl-runtime] --duration-sec must be an integer" >&2
    exit 2
    ;;
esac

if [[ "${SET_MODE}" == "true" ]]; then
  export NJRH_AMCL_LOCALIZATION_MODE="${MODE}"
fi
if [[ "${RESTART_RUNTIME}" == "true" ]]; then
  bash "${SCRIPT_DIR}/run_amcl_shadow_localization.sh" --mode "${MODE}" --restart
fi

failures=0
warnings=0
pass() { echo "PASS $*"; }
warn() { echo "WARN $*"; warnings=$((warnings + 1)); }
fail() { echo "FAIL $*"; failures=$((failures + 1)); }

ros_param_get() {
  timeout 8 ros2 param get "$1" "$2" 2>/dev/null || true
}

topic_info() {
  timeout 8 ros2 topic info -v "$1" 2>/dev/null || true
}

bridge_status_once() {
  timeout 8 ros2 topic echo /localization/bridge_status --once --field data 2>/dev/null |
    awk '/^\{/ {print; exit}' || true
}

json_field() {
  local payload="$1"
  local field="$2"
  BRIDGE_STATUS_JSON="${payload}" python3 - "${field}" <<'PY'
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

measure_scan_amcl() {
  local duration="$1"
  timeout "$((duration + 5))" python3 - "${duration}" <<'PY'
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan

duration = float(sys.argv[1])
qos = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=100,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)
rclpy.init()
node = Node("verify_amcl_scan_amcl_hz")
count = 0
first = None
last = None

def cb(_msg):
    global count, first, last
    now = time.time()
    if first is None:
        first = now
    last = now
    count += 1

node.create_subscription(LaserScan, "/scan_amcl", cb, qos)
deadline = time.time() + duration
while rclpy.ok() and time.time() < deadline:
    rclpy.spin_once(node, timeout_sec=0.1)
node.destroy_node()
rclpy.shutdown()
hz = 0.0 if first is None or last is None or last <= first else max(0, count - 1) / (last - first)
print(f"{count} {hz:.3f}")
PY
}

if [[ "${MODE}" == "disabled" ]]; then
  if timeout 5 ros2 node list 2>/dev/null | grep -qx "/amcl"; then
    warn "/amcl is running while mode=disabled; verify this is intentional resident reuse"
  else
    pass "AMCL is not required in disabled mode"
  fi
else
  state="$(timeout 8 ros2 lifecycle get /amcl 2>/dev/null || true)"
  [[ "${state}" == active* ]] && pass "/amcl lifecycle active" || fail "/amcl lifecycle is not active: ${state:-missing}"

  tf_broadcast="$(ros_param_get /amcl tf_broadcast)"
  [[ "${tf_broadcast}" == *"False"* || "${tf_broadcast}" == *"false"* ]] && pass "/amcl tf_broadcast=false" || fail "/amcl tf_broadcast not false: ${tf_broadcast:-missing}"

  scan_topic="$(ros_param_get /amcl scan_topic)"
  [[ "${scan_topic}" == *"/scan_amcl"* ]] && pass "/amcl scan_topic=/scan_amcl" || fail "/amcl scan_topic is not /scan_amcl: ${scan_topic:-missing}"

  info="$(topic_info /scan_amcl)"
  grep -q "Node name: amcl_scan_admission" <<<"${info}" && pass "/scan_amcl publisher is amcl_scan_admission" || fail "/scan_amcl publisher is missing or wrong"
  grep -q "Node name: amcl" <<<"${info}" && pass "/scan_amcl subscribed by AMCL" || fail "/scan_amcl is not subscribed by AMCL"

  read -r scan_count scan_hz < <(measure_scan_amcl "${DURATION_SEC}")
  awk -v hz="${scan_hz:-0}" 'BEGIN {exit !(hz >= 3.0)}' && pass "/scan_amcl hz=${scan_hz}" || fail "/scan_amcl hz too low: count=${scan_count:-0} hz=${scan_hz:-0}"

  if [[ "${DO_SEED}" == "true" ]]; then
    seed_output="$(timeout 10 ros2 service call /robot_localization_bridge/seed_amcl_initial_pose std_srvs/srv/Trigger "{}" 2>&1 || true)"
    grep -Eiq "success[:=][[:space:]]*true|success=True" <<<"${seed_output}" && pass "AMCL seed service succeeded" || fail "AMCL seed service failed: ${seed_output}"
    timeout 6 ros2 service call /request_nomotion_update std_srvs/srv/Empty "{}" >/dev/null 2>&1 || warn "/request_nomotion_update failed or unavailable"
  fi
fi

status="$(bridge_status_once)"
if [[ -z "${status}" ]]; then
  fail "/localization/bridge_status missing"
else
  pass "/localization/bridge_status available"
  has_map="$(json_field "${status}" has_map_to_odom)"
  owner="$(json_field "${status}" map_to_odom_publisher_owner)"
  expected_owner="$(json_field "${status}" expected_map_to_odom_owner)"
  bridge_mode="$(json_field "${status}" amcl_gate_mode)"
  amcl_seeded="$(json_field "${status}" amcl_seed_succeeded)"
  amcl_shadow_ready="$(json_field "${status}" amcl_shadow_ready)"
  amcl_gated_ready="$(json_field "${status}" amcl_gated_ready)"
  amcl_degraded="$(json_field "${status}" localization_degraded)"

  if [[ "${CHECK_OWNER}" == "true" ]]; then
    [[ "${has_map}" == "true" ]] && pass "bridge has_map_to_odom=true" || fail "bridge has_map_to_odom is ${has_map:-missing}"
    [[ "${owner}" == "robot_localization_bridge" ]] && pass "map->odom owner is robot_localization_bridge" || fail "map->odom owner is ${owner:-missing}"
    [[ -z "${expected_owner}" || "${expected_owner}" == "robot_localization_bridge" ]] && pass "expected map->odom owner is bridge" || fail "expected owner is ${expected_owner}"
  fi

  if [[ "${MODE}" != "disabled" && "${CHECK_AMCL}" == "true" ]]; then
    [[ "${bridge_mode}" == "${MODE}" ]] && pass "bridge amcl_gate_mode=${MODE}" || fail "bridge amcl_gate_mode=${bridge_mode:-missing}, expected ${MODE}"
    [[ "${amcl_seeded}" == "true" ]] && pass "bridge reports amcl_seed_succeeded=true" || warn "bridge reports amcl_seed_succeeded=${amcl_seeded:-missing}"
    if [[ "${MODE}" == "shadow" ]]; then
      [[ "${amcl_shadow_ready}" == "true" ]] && pass "amcl_shadow_ready=true" || warn "amcl_shadow_ready=${amcl_shadow_ready:-missing}; stationary stale /amcl_pose is acceptable only after one seed/update sample"
    elif [[ "${MODE}" == "gated" ]]; then
      [[ "${amcl_gated_ready}" == "true" ]] && pass "amcl_gated_ready=true" || fail "amcl_gated_ready=${amcl_gated_ready:-missing}"
    fi
    [[ "${amcl_degraded}" != "true" ]] && pass "localization_degraded is not true" || warn "localization_degraded=true"
  fi
fi

if [[ "${CHECK_TRIGGERED}" == "true" ]]; then
  trigger_output="$(timeout 45 ros2 service call /global_localization/trigger robot_interfaces/srv/TriggerLocalization "{reason: 'verify_amcl_runtime_readiness'}" 2>&1 || true)"
  grep -Eiq 'accepted[=:][[:space:]]*(True|true)|accepted:[[:space:]]*true' <<<"${trigger_output}" && pass "Isaac triggered wrapper accepted" || fail "Isaac triggered wrapper failed: ${trigger_output}"
fi

tf_info="$(topic_info /tf)"
if grep -q "Node name: amcl" <<<"${tf_info}"; then
  warn "AMCL has a /tf endpoint even with tf_broadcast=false; treating bridge owner and tf_broadcast=false as the hard contract"
else
  pass "AMCL is not a /tf publisher"
fi

if [[ "${CHECK_MOTION_WINDOW}" == "true" ]]; then
  warn "--check-motion-window requires the operator to move or send a navigation goal during the duration window; this script reports readiness fields only"
fi

echo "[verify-amcl-runtime] mode=${MODE} failures=${failures} warnings=${warnings}"
[[ "${failures}" -eq 0 ]]
