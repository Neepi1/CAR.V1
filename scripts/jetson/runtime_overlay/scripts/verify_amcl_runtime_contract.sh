#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/common_env.sh"

MODE="${NJRH_AMCL_LOCALIZATION_MODE:-shadow}"
DURATION_SEC=10
EXPECT_READY=false
EXPECT_DEGRADED=false
EXPECT_FAILED=false
RESTART=false
DRY_RUN=false
KILL_AMCL_FOR_TEST=false
STATUS_FILE="${NJRH_AMCL_RUNTIME_STATUS_FILE:-/tmp/njrh_amcl_runtime_status.env}"

usage() {
  cat <<'USAGE'
Usage: verify_amcl_runtime_contract.sh [--mode disabled|shadow|gated] [--expect-ready|--expect-degraded|--expect-failed] [--duration-sec N] [--restart] [--dry-run] [--kill-amcl-for-test]

Verifies the AMCL runtime readiness contract without changing Nav2, TF, pointcloud,
Ranger odom, or EKF parameters.
USAGE
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --mode)
      MODE="${2:-}"
      shift 2
      ;;
    --expect-ready)
      EXPECT_READY=true
      shift
      ;;
    --expect-degraded)
      EXPECT_DEGRADED=true
      shift
      ;;
    --expect-failed)
      EXPECT_FAILED=true
      shift
      ;;
    --duration-sec)
      DURATION_SEC="${2:-}"
      shift 2
      ;;
    --restart)
      RESTART=true
      shift
      ;;
    --dry-run)
      DRY_RUN=true
      shift
      ;;
    --kill-amcl-for-test)
      KILL_AMCL_FOR_TEST=true
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "[amcl-contract] unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

case "${MODE}" in
  disabled|shadow|gated) ;;
  *)
    echo "[amcl-contract] invalid mode=${MODE}" >&2
    exit 2
    ;;
esac

failures=()
passes=()
warns=()

pass() { passes+=("$1"); }
warn() { warns+=("$1"); }
fail() { failures+=("$1"); }

topic_publishers() {
  local topic="$1"
  timeout 4 ros2 topic info "${topic}" 2>/dev/null |
    awk -F: '/Publisher count/ {gsub(/^[ \t]+|[ \t]+$/, "", $2); print $2; exit}'
}

node_exists() {
  local node="$1"
  timeout 4 ros2 node list 2>/dev/null | grep -Fxq "${node}"
}

json_field() {
  local json="$1"
  local field="$2"
  JSON_PAYLOAD="${json}" python3 - "${field}" <<'PY'
import json
import os
import sys

field = sys.argv[1]
try:
    payload = os.environ.get("JSON_PAYLOAD", "").strip()
    lines = [line.strip() for line in payload.splitlines() if line.strip().startswith("{")]
    data = json.loads(lines[-1] if lines else payload)
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

load_status_file() {
  AMCL_MODE=""
  AMCL_STATE=""
  AMCL_START_RESULT=""
  AMCL_READY="false"
  AMCL_DEGRADED="false"
  AMCL_FAILURE_REASON=""
  AMCL_PID_ALIVE="false"
  AMCL_PID_STALE_CLEARED="false"
  SCAN_ADMISSION_ALIVE="false"
  SCAN_ADMISSION_PID_STALE_CLEARED="false"
  SCAN_ADMISSION_STATUS_PUBLISHER_COUNT="0"
  AMCL_POSE_PUBLISHER_COUNT="0"
  AMCL_SEED_SUCCEEDED="false"
  if [[ -f "${STATUS_FILE}" ]]; then
    # shellcheck disable=SC1090
    source "${STATUS_FILE}"
    return 0
  fi
  return 1
}

if [[ "${DRY_RUN}" == "true" ]]; then
  echo "[amcl-contract] dry-run mode=${MODE} duration=${DURATION_SEC} status_file=${STATUS_FILE}"
  exit 0
fi

if [[ "${KILL_AMCL_FOR_TEST}" == "true" ]]; then
  echo "[amcl-contract] --kill-amcl-for-test requested; stopping AMCL through runner only" >&2
  NJRH_AMCL_LOCALIZATION_MODE="${MODE}" \
    NJRH_AMCL_RUNTIME_STATUS_FILE="${STATUS_FILE}" \
    bash "${SCRIPT_DIR}/run_amcl_shadow_localization.sh" --mode "${MODE}" --stop
fi

if [[ "${RESTART}" == "true" ]]; then
  set +e
  NJRH_AMCL_LOCALIZATION_MODE="${MODE}" \
    NJRH_AMCL_RUNTIME_STATUS_FILE="${STATUS_FILE}" \
    bash "${SCRIPT_DIR}/run_amcl_shadow_localization.sh" --mode "${MODE}" --restart
  runner_rc=$?
  set -e
  echo "[amcl-contract] runner_exit=${runner_rc}"
else
  runner_rc=0
fi

sleep "${DURATION_SEC}"

if load_status_file; then
  pass "status file exists: ${STATUS_FILE}"
else
  fail "status file missing: ${STATUS_FILE}"
fi

echo "[amcl-contract] status: mode=${AMCL_MODE:-} state=${AMCL_STATE:-} result=${AMCL_START_RESULT:-} ready=${AMCL_READY:-false} degraded=${AMCL_DEGRADED:-false} reason=${AMCL_FAILURE_REASON:-} amcl_pid_stale_cleared=${AMCL_PID_STALE_CLEARED:-false} scan_pid_stale_cleared=${SCAN_ADMISSION_PID_STALE_CLEARED:-false}"

if [[ "${MODE}" == "disabled" ]]; then
  [[ "${AMCL_START_RESULT:-}" == "disabled" ]] && pass "disabled status recorded" || fail "disabled mode did not record disabled result"
else
  node_exists /amcl && pass "/amcl node exists" || fail "/amcl node missing"
  lifecycle_state="$(timeout 8 ros2 lifecycle get /amcl 2>/dev/null || true)"
  [[ "${lifecycle_state}" == active* ]] && pass "/amcl lifecycle active" || fail "/amcl lifecycle not active: ${lifecycle_state:-missing}"
  tf_broadcast="$(timeout 5 ros2 param get /amcl tf_broadcast 2>/dev/null || true)"
  [[ "${tf_broadcast}" == *"False"* || "${tf_broadcast}" == *"false"* ]] && pass "AMCL tf_broadcast=false" || fail "AMCL tf_broadcast is not false: ${tf_broadcast:-missing}"
  scan_topic="$(timeout 5 ros2 param get /amcl scan_topic 2>/dev/null || true)"
  [[ "${scan_topic}" == *"/scan_amcl"* ]] && pass "AMCL scan_topic=/scan_amcl" || fail "AMCL scan_topic is not /scan_amcl: ${scan_topic:-missing}"
  amcl_pose_publishers="$(topic_publishers /amcl_pose)"
  scan_status_publishers="$(topic_publishers /amcl_scan_admission/status)"
  scan_amcl_publishers="$(topic_publishers /scan_amcl)"
  [[ "${amcl_pose_publishers:-0}" -gt 0 ]] && pass "/amcl_pose has publisher" || fail "/amcl_pose publisher_count=${amcl_pose_publishers:-0}"
  [[ "${scan_status_publishers:-0}" -gt 0 ]] && pass "/amcl_scan_admission/status has publisher" || fail "/amcl_scan_admission/status publisher_count=${scan_status_publishers:-0}"
  [[ "${scan_amcl_publishers:-0}" -gt 0 ]] && pass "/scan_amcl has publisher" || fail "/scan_amcl publisher_count=${scan_amcl_publishers:-0}"
fi

bridge_status="$(timeout 5 ros2 topic echo --once --field data /localization/bridge_status 2>/dev/null || true)"
if [[ -n "${bridge_status}" ]]; then
  bridge_ready="$(json_field "${bridge_status}" amcl_ready)"
  bridge_degraded="$(json_field "${bridge_status}" localization_degraded)"
  bridge_process_alive="$(json_field "${bridge_status}" amcl_process_alive)"
  bridge_pose_publishers="$(json_field "${bridge_status}" amcl_pose_publisher_count)"
  bridge_scan_status_publishers="$(json_field "${bridge_status}" amcl_scan_admission_status_publisher_count)"
  echo "[amcl-contract] bridge: amcl_ready=${bridge_ready} localization_degraded=${bridge_degraded} amcl_process_alive=${bridge_process_alive} pose_publishers=${bridge_pose_publishers} amcl_scan_admission_status_publisher_count=${bridge_scan_status_publishers}"
  if [[ "${bridge_pose_publishers:-0}" == "0" && "${bridge_ready}" == "true" ]]; then
    fail "bridge reports amcl_ready=true while /amcl_pose publisher_count=0"
  fi
  if [[ "${AMCL_READY:-false}" != "true" && "${bridge_ready}" == "true" ]]; then
    fail "bridge reports amcl_ready=true while runtime AMCL_READY=${AMCL_READY:-false}"
  fi
else
  warn "bridge_status unavailable"
fi

api_state="$(timeout 5 curl -fsS http://127.0.0.1:8080/api/v1/navigation/state 2>/dev/null || true)"
if [[ -n "${api_state}" ]]; then
  api_degraded="$(json_field "${api_state}" localization_degraded)"
  api_ready="$(json_field "${api_state}" amcl_ready)"
  api_scan_status_publishers="$(json_field "${api_state}" amcl_scan_admission_status_publisher_count)"
  echo "[amcl-contract] api: amcl_ready=${api_ready} localization_degraded=${api_degraded} amcl_scan_admission_status_publisher_count=${api_scan_status_publishers}"
else
  warn "API navigation state unavailable"
fi

if [[ "${EXPECT_READY}" == "true" ]]; then
  [[ "${AMCL_READY:-false}" == "true" ]] && pass "expected ready" || fail "expected ready but AMCL_READY=${AMCL_READY:-false}"
fi
if [[ "${EXPECT_DEGRADED}" == "true" ]]; then
  [[ "${AMCL_DEGRADED:-false}" == "true" ]] && pass "expected degraded" || fail "expected degraded but AMCL_DEGRADED=${AMCL_DEGRADED:-false}"
fi
if [[ "${EXPECT_FAILED}" == "true" ]]; then
  [[ "${AMCL_STATE:-}" == "AMCL_FAILED" ]] && pass "expected failed" || fail "expected failed but AMCL_STATE=${AMCL_STATE:-}"
fi

printf '[amcl-contract] PASS %s\n' "${passes[@]}"
for item in "${warns[@]}"; do
  printf '[amcl-contract] WARN %s\n' "${item}"
done
if [[ "${#failures[@]}" -gt 0 ]]; then
  for item in "${failures[@]}"; do
    printf '[amcl-contract] FAIL %s\n' "${item}" >&2
  done
  exit 1
fi

exit 0
