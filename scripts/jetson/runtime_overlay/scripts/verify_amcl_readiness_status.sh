#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/common_env.sh"

MODE="${NJRH_AMCL_LOCALIZATION_MODE:-shadow}"
DURATION_SEC=10
WATCH=false
REQUEST_NOMOTION_UPDATE=false
EXPECT_STATIC_STANDBY=false
EXPECT_TRACKING_READY=false
STATUS_FILE="${NJRH_AMCL_RUNTIME_STATUS_FILE:-/tmp/njrh_amcl_runtime_status.env}"
NOMOTION_PROBE="${NJRH_AMCL_NOMOTION_PROBE:-${SCRIPT_DIR}/amcl_nomotion_update_probe.py}"

usage() {
  cat <<'USAGE'
Usage: verify_amcl_readiness_status.sh [options]

Options:
  --mode disabled|shadow|gated
  --duration-sec N
  --watch
  --request-nomotion-update
  --expect-static-standby
  --expect-tracking-ready

Read-only AMCL readiness/status verifier. With --request-nomotion-update it calls
/request_nomotion_update and confirms /amcl_pose is received by a waiter that is
subscribed before the service call. It does not publish navigation goals, TF, or
pointcloud data.
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
    --watch)
      WATCH=true
      shift
      ;;
    --request-nomotion-update)
      REQUEST_NOMOTION_UPDATE=true
      shift
      ;;
    --expect-static-standby)
      EXPECT_STATIC_STANDBY=true
      shift
      ;;
    --expect-tracking-ready)
      EXPECT_TRACKING_READY=true
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "[amcl-readiness-status] unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

case "${MODE}" in
  disabled|shadow|gated) ;;
  *)
    echo "[amcl-readiness-status] --mode must be disabled, shadow, or gated" >&2
    exit 2
    ;;
esac
case "${DURATION_SEC}" in
  ''|*[!0-9]*)
    echo "[amcl-readiness-status] --duration-sec must be an integer" >&2
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
value = data.get(field, "")
for part in field.split("."):
    if isinstance(data, dict) and part in data:
        data = data[part]
    else:
        data = ""
        break
value = data
if isinstance(value, bool):
    print("true" if value else "false")
else:
    print(value)
PY
}

bridge_status_once() {
  timeout 8 ros2 topic echo /localization/bridge_status --once --field data 2>/dev/null |
    awk '/^\{/ {print; exit}' || true
}

api_status_once() {
  python3 - <<'PY'
import json
import urllib.request

try:
    with urllib.request.urlopen("http://127.0.0.1:8080/api/v1/status", timeout=3.0) as resp:
        print(resp.read().decode("utf-8"))
except Exception:
    print("")
PY
}

status_file_summary() {
  if [[ ! -f "${STATUS_FILE}" ]]; then
    echo "missing"
    return 1
  fi
  local stamp_sec=""
  local state=""
  local ready=""
  local degraded=""
  set +u
  # shellcheck disable=SC1090
  source "${STATUS_FILE}"
  stamp_sec="${AMCL_STATUS_STAMP_SEC:-}"
  state="${AMCL_STATE:-}"
  ready="${AMCL_READY:-}"
  degraded="${AMCL_DEGRADED:-}"
  set -u
  local now_sec
  now_sec="$(date +%s)"
  local age_ms="-1"
  local stale="true"
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
  echo "state=${state} ready=${ready} degraded=${degraded} age_ms=${age_ms} stale=${stale}"
}

nomotion_update_probe() {
  local pose_topic="${NJRH_AMCL_POSE_TOPIC:-/amcl_pose}"
  local service="${NJRH_AMCL_NOMOTION_UPDATE_SERVICE:-/request_nomotion_update}"
  local timeout_sec="${AMCL_NOMOTION_UPDATE_RESPONSE_TIMEOUT_SEC:-${NJRH_AMCL_NOMOTION_UPDATE_RESPONSE_TIMEOUT_SEC:-5.0}}"
  [[ -x "${NOMOTION_PROBE}" ]] || {
    echo "helper_missing=${NOMOTION_PROBE}"
    return 1
  }
  timeout "$(( ${timeout_sec%.*} + 6 ))" python3 "${NOMOTION_PROBE}" \
    --pose-topic "${pose_topic}" \
    --service "${service}" \
    --timeout-sec "${timeout_sec}" \
    --pre-subscribe-warmup-sec "${NJRH_AMCL_NOMOTION_PRE_SUBSCRIBE_WARMUP_SEC:-0.2}" \
    --require-header-fresh false \
    --max-header-age-sec "${NJRH_AMCL_POSE_MAX_AGE_SEC:-1.0}"
}

run_once() {
  echo "[amcl-readiness-status] mode=${MODE} status_file=${STATUS_FILE}"
  summary="$(status_file_summary || true)"
  echo "[amcl-readiness-status] status_file ${summary}"
  grep -q "stale=true" <<<"${summary}" && pass "status file stale is detected or file has no fresh epoch" || pass "status file is fresh"

  if [[ "${MODE}" != "disabled" ]]; then
    state="$(timeout 6 ros2 lifecycle get /amcl 2>/dev/null || true)"
    [[ "${state}" == active* ]] && pass "/amcl lifecycle active" || fail "/amcl lifecycle not active: ${state:-missing}"

    amcl_pose_info="$(timeout 6 ros2 topic info /amcl_pose 2>/dev/null || true)"
    grep -q "Publisher count: 1" <<<"${amcl_pose_info}" && pass "/amcl_pose publisher exists" || fail "/amcl_pose publisher missing"

    scan_status_info="$(timeout 6 ros2 topic info /amcl_scan_admission/status 2>/dev/null || true)"
    grep -q "Publisher count: 1" <<<"${scan_status_info}" && pass "/amcl_scan_admission/status publisher exists" || fail "scan admission status publisher missing"
  fi

  if [[ "${REQUEST_NOMOTION_UPDATE}" == "true" ]]; then
    if output="$(nomotion_update_probe 2>&1)"; then
      pass "request_nomotion_update response observed: pose_received_after_request ${output}"
    else
      fail "request_nomotion_update response missing: ${output}"
    fi
  fi

  bridge="$(bridge_status_once)"
  if [[ -n "${bridge}" ]]; then
    pass "bridge_status available"
    echo "[amcl-readiness-status] bridge amcl_status_source=$(json_field "${bridge}" amcl_status_source) stale=$(json_field "${bridge}" amcl_status_file_stale) process_ready=$(json_field "${bridge}" amcl_process_ready) seeded=$(json_field "${bridge}" amcl_seeded) static_standby=$(json_field "${bridge}" amcl_static_standby) tracking_ready=$(json_field "${bridge}" amcl_tracking_ready) correction_ready=$(json_field "${bridge}" amcl_correction_ready) ready=$(json_field "${bridge}" amcl_ready) degraded=$(json_field "${bridge}" localization_degraded)"
    if [[ "$(json_field "${bridge}" amcl_status_file_stale)" == "true" && "$(json_field "${bridge}" amcl_status_source)" != "stale_file_ignored" ]]; then
      fail "bridge sees stale status file but does not mark source=stale_file_ignored"
    fi
    if [[ "$(json_field "${bridge}" amcl_pose_publisher_count)" == "0" && "$(json_field "${bridge}" amcl_gated_ready)" == "true" ]]; then
      fail "bridge reports amcl_gated_ready=true with /amcl_pose publisher_count=0"
    fi
    if [[ "${EXPECT_STATIC_STANDBY}" == "true" && "$(json_field "${bridge}" amcl_static_standby)" != "true" ]]; then
      fail "expected amcl_static_standby=true"
    fi
    if [[ "${EXPECT_TRACKING_READY}" == "true" && "$(json_field "${bridge}" amcl_tracking_ready)" != "true" ]]; then
      fail "expected amcl_tracking_ready=true"
    fi
  else
    fail "bridge_status unavailable"
  fi

  api="$(api_status_once)"
  if [[ -n "${api}" ]]; then
    pass "API status available"
    echo "[amcl-readiness-status] api amcl_status_source=$(json_field "${api}" localization.amcl_status_source) static_standby=$(json_field "${api}" localization.amcl_static_standby) tracking_ready=$(json_field "${api}" localization.amcl_tracking_ready) correction_ready=$(json_field "${api}" localization.amcl_correction_ready) using_triggered_baseline_only=$(json_field "${api}" using_triggered_baseline_only)"
  else
    warn "API status unavailable on 127.0.0.1:8080"
  fi
}

if [[ "${WATCH}" == "true" ]]; then
  end=$(( $(date +%s) + DURATION_SEC ))
  while [[ "$(date +%s)" -le "${end}" ]]; do
    run_once
    sleep 1
  done
else
  run_once
fi

echo "[amcl-readiness-status] passes complete warnings=${warnings} failures=${failures}"
exit "${failures}"
