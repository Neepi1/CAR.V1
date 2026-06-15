#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/common_env.sh"

MODE="${NJRH_AMCL_LOCALIZATION_MODE:-gated}"
TIMEOUT_SEC=5.0
EXPECT_STATIC_STANDBY=false
WATCH_STATUS=false
STATUS_FILE="${NJRH_AMCL_RUNTIME_STATUS_FILE:-/tmp/njrh_amcl_runtime_status.env}"
NOMOTION_PROBE="${NJRH_AMCL_NOMOTION_PROBE:-${SCRIPT_DIR}/amcl_nomotion_update_probe.py}"

usage() {
  cat <<'USAGE'
Usage: verify_amcl_nomotion_readiness.sh [--mode shadow|gated] [--timeout-sec N] [--expect-static-standby] [--watch-status]

Verifies the AMCL /request_nomotion_update readiness race fix. It calls the
rclpy helper that subscribes to /amcl_pose before calling the service. It does
not send navigation goals, publish TF, publish /initialpose, or touch pointclouds.
USAGE
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --mode)
      MODE="${2:-}"
      shift 2
      ;;
    --timeout-sec)
      TIMEOUT_SEC="${2:-}"
      shift 2
      ;;
    --expect-static-standby)
      EXPECT_STATIC_STANDBY=true
      shift
      ;;
    --watch-status)
      WATCH_STATUS=true
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "[amcl-nomotion] unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

case "${MODE}" in
  shadow|gated) ;;
  *)
    echo "[amcl-nomotion] --mode must be shadow or gated" >&2
    exit 2
    ;;
esac

failures=0
warnings=0
pass() { echo "PASS $*"; }
warn() { echo "WARN $*"; warnings=$((warnings + 1)); }
fail() { echo "FAIL $*"; failures=$((failures + 1)); }

json_field() {
  local payload="$1"
  local field="$2"
  JSON_PAYLOAD="${payload}" python3 - "${field}" <<'PY'
import json
import os
import sys

field = sys.argv[1]
payload = os.environ.get("JSON_PAYLOAD", "").strip()
try:
    lines = [line.strip() for line in payload.splitlines() if line.strip().startswith("{")]
    data = json.loads(lines[-1] if lines else payload)
except Exception:
    print("")
    raise SystemExit(0)
for part in field.split("."):
    if isinstance(data, dict) and part in data:
        data = data[part]
    else:
        data = ""
        break
if isinstance(data, bool):
    print("true" if data else "false")
else:
    print(data)
PY
}

bridge_status_once() {
  timeout 8 ros2 topic echo /localization/bridge_status --once --field data 2>/dev/null |
    awk '/^\{/ {print; exit}' || true
}

api_status_once() {
  python3 - <<'PY'
import urllib.request
try:
    with urllib.request.urlopen("http://127.0.0.1:8080/api/v1/status", timeout=3.0) as resp:
        print(resp.read().decode("utf-8"))
except Exception:
    print("")
PY
}

status_file_state() {
  if [[ ! -f "${STATUS_FILE}" ]]; then
    echo "available=false stale=true state="
    return 0
  fi
  set +u
  # shellcheck disable=SC1090
  source "${STATUS_FILE}"
  set -u
  local now_sec stamp_sec age_ms stale
  now_sec="$(date +%s)"
  stamp_sec="${AMCL_STATUS_STAMP_SEC:-}"
  age_ms="-1"
  stale="true"
  if [[ "${stamp_sec}" =~ ^[0-9]+([.][0-9]+)?$ ]]; then
    read -r age_ms stale < <(python3 - "${now_sec}" "${stamp_sec}" "${NJRH_AMCL_RUNTIME_STATUS_TTL_SEC:-5.0}" <<'PY'
import sys
now_sec = float(sys.argv[1])
stamp_sec = float(sys.argv[2])
ttl_sec = float(sys.argv[3])
age_ms = max(0.0, (now_sec - stamp_sec) * 1000.0)
print(f"{age_ms:.0f} {'true' if age_ms > ttl_sec * 1000.0 else 'false'}")
PY
)
  fi
  echo "available=true stale=${stale} state=${AMCL_STATE:-} seeded=${AMCL_SEEDED:-false} static=${AMCL_STATIC_STANDBY:-false} nomotion=${AMCL_NOMOTION_POSE_RECEIVED:-false} age_ms=${age_ms}"
}

state="$(timeout 6 ros2 lifecycle get /amcl 2>/dev/null || true)"
[[ "${state}" == active* ]] && pass "/amcl lifecycle active" || fail "/amcl lifecycle not active: ${state:-missing}"

amcl_pose_info="$(timeout 6 ros2 topic info /amcl_pose 2>/dev/null || true)"
grep -q "Publisher count: 1" <<<"${amcl_pose_info}" && pass "/amcl_pose publisher > 0" || fail "/amcl_pose publisher missing"

scan_status_info="$(timeout 6 ros2 topic info /amcl_scan_admission/status 2>/dev/null || true)"
grep -q "Publisher count: 1" <<<"${scan_status_info}" && pass "/amcl_scan_admission/status publisher > 0" || fail "/amcl_scan_admission/status publisher missing"

if [[ ! -x "${NOMOTION_PROBE}" ]]; then
  fail "helper missing or not executable: ${NOMOTION_PROBE}"
else
  set +e
  probe_json="$(python3 "${NOMOTION_PROBE}" \
    --pose-topic /amcl_pose \
    --service /request_nomotion_update \
    --timeout-sec "${TIMEOUT_SEC}" \
    --pre-subscribe-warmup-sec 0.2 \
    --require-header-fresh false \
    --max-header-age-sec 1.0 2>&1)"
  probe_rc=$?
  set -e
  echo "[amcl-nomotion] helper_rc=${probe_rc} json=${probe_json}"
  [[ "${probe_rc}" -eq 0 ]] && pass "helper exit 0" || fail "helper exit ${probe_rc}"
  [[ "$(json_field "${probe_json}" service_available)" == "true" ]] && pass "service available" || fail "service unavailable"
  [[ "$(json_field "${probe_json}" service_call_ok)" == "true" ]] && pass "service call ok" || fail "service call failed"
  [[ "$(json_field "${probe_json}" pose_received)" == "true" ]] && pass "pose received in request window" || fail "pose not received in request window"
fi

status_summary="$(status_file_state)"
echo "[amcl-nomotion] status_file ${status_summary}"
if grep -q "stale=false" <<<"${status_summary}" && grep -q "state=AMCL_FAILED" <<<"${status_summary}"; then
  fail "fresh status file is AMCL_FAILED"
elif grep -q "stale=true" <<<"${status_summary}" && grep -q "state=AMCL_FAILED" <<<"${status_summary}"; then
  warn "status file is stale AMCL_FAILED; bridge/API must ignore it"
else
  pass "status file is not a fresh AMCL_FAILED"
fi

bridge="$(bridge_status_once)"
if [[ -n "${bridge}" ]]; then
  pass "bridge_status available"
  echo "[amcl-nomotion] bridge source=$(json_field "${bridge}" amcl_status_source) stale=$(json_field "${bridge}" amcl_status_file_stale) seeded=$(json_field "${bridge}" amcl_seeded) static=$(json_field "${bridge}" amcl_static_standby) tracking=$(json_field "${bridge}" amcl_tracking_ready) correction=$(json_field "${bridge}" amcl_correction_ready) nomotion=$(json_field "${bridge}" amcl_nomotion_pose_received)"
  if [[ "$(json_field "${bridge}" amcl_status_file_stale)" == "true" && "$(json_field "${bridge}" amcl_status_source)" != "stale_file_ignored" ]]; then
    fail "bridge did not ignore stale runtime status"
  fi
  if [[ "${EXPECT_STATIC_STANDBY}" == "true" && "$(json_field "${bridge}" amcl_static_standby)" != "true" ]]; then
    fail "expected bridge amcl_static_standby=true"
  fi
else
  fail "bridge_status unavailable"
fi

api="$(api_status_once)"
if [[ -n "${api}" ]]; then
  pass "API status available"
  echo "[amcl-nomotion] api source=$(json_field "${api}" localization.amcl_status_source) seeded=$(json_field "${api}" localization.amcl_seeded) static=$(json_field "${api}" localization.amcl_static_standby) correction=$(json_field "${api}" localization.amcl_correction_ready) nomotion=$(json_field "${api}" localization.amcl_nomotion_pose_received)"
else
  warn "API status unavailable"
fi

if [[ "${WATCH_STATUS}" == "true" ]]; then
  for _ in 1 2 3; do
    sleep 1
    bridge="$(bridge_status_once)"
    echo "[amcl-nomotion] watch bridge source=$(json_field "${bridge}" amcl_status_source) static=$(json_field "${bridge}" amcl_static_standby) correction=$(json_field "${bridge}" amcl_correction_ready)"
  done
fi

echo "[amcl-nomotion] warnings=${warnings} failures=${failures}"
exit "${failures}"
