#!/usr/bin/env bash
set -euo pipefail

API_URL="${API_URL:-http://127.0.0.1:8080}"
CURL_TIMEOUT_SEC="${CURL_TIMEOUT_SEC:-5}"
LATCH_FILE="${LATCH_FILE:-/workspaces/njrh-v3/workspace1/maps_release/docking_contact_latch.json}"
CLEAR_STALE=false

usage() {
  cat <<'USAGE'
Usage: verify_dock_contact_latch_gate.sh [options]

Read-only by default. Checks the persistent dock-contact latch, API dock gate,
/docking/state, /docking/status, and /battery_state without moving the robot.

Options:
  --api-url URL                 API base URL, default http://127.0.0.1:8080
  --latch-file PATH             Latch JSON path, default maps_release/docking_contact_latch.json
  --clear-stale-bms-latch       Explicitly clear a stale source=bms latch through the API
  -h, --help                    Show this help

Set ROBOT_API_TOKEN to send X-Robot-Token.
USAGE
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --api-url)
      API_URL="$2"
      shift 2
      ;;
    --latch-file)
      LATCH_FILE="$2"
      shift 2
      ;;
    --clear-stale-bms-latch)
      CLEAR_STALE=true
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "ERROR: unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

API_URL="${API_URL%/}"
TMP_DIR="$(mktemp -d)"
trap 'rm -rf "${TMP_DIR}"' EXIT

curl_json() {
  local method="$1"
  local url="$2"
  local body="${3:-}"
  local -a args=(-fsS --max-time "${CURL_TIMEOUT_SEC}" -H "Content-Type: application/json")
  if [[ -n "${ROBOT_API_TOKEN:-}" ]]; then
    args+=(-H "X-Robot-Token: ${ROBOT_API_TOKEN}")
  fi
  if [[ "${method}" == "GET" ]]; then
    curl "${args[@]}" "${url}"
  else
    curl "${args[@]}" -X "${method}" --data "${body}" "${url}"
  fi
}

topic_once() {
  local topic="$1"
  local output="$2"
  if command -v ros2 >/dev/null 2>&1; then
    timeout 4s ros2 topic echo "${topic}" --once > "${output}" 2>&1 || true
  else
    echo "ros2 not found" > "${output}"
  fi
}

STATUS_JSON="${TMP_DIR}/status.json"
NAV_STATE_JSON="${TMP_DIR}/navigation_state.json"
DOCKING_STATE_JSON="${TMP_DIR}/docking_state.json"
LATCH_JSON="${TMP_DIR}/latch.json"
DOCKING_STATUS_TXT="${TMP_DIR}/docking_status.txt"
BMS_TXT="${TMP_DIR}/battery_state.txt"
ANALYSIS_JSON="${TMP_DIR}/analysis.json"

if [[ -f "${LATCH_FILE}" ]]; then
  cp "${LATCH_FILE}" "${LATCH_JSON}"
else
  printf '{}\n' > "${LATCH_JSON}"
fi

curl_json GET "${API_URL}/api/v1/status" > "${STATUS_JSON}"
curl_json GET "${API_URL}/api/v1/navigation/state" > "${NAV_STATE_JSON}"
curl_json GET "${API_URL}/api/v1/docking/state" > "${DOCKING_STATE_JSON}"
topic_once /docking/status "${DOCKING_STATUS_TXT}"
topic_once /battery_state "${BMS_TXT}"

python3 - "$LATCH_JSON" "$NAV_STATE_JSON" "$DOCKING_STATE_JSON" "$ANALYSIS_JSON" <<'PY'
import json
import math
import sys
import time
from datetime import datetime, timezone

latch_path, nav_path, docking_path, analysis_path = sys.argv[1:]

def load(path):
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)

def parse_age(value):
    if not value:
        return None
    try:
        stamp = datetime.strptime(value, "%Y-%m-%dT%H:%M:%SZ").replace(tzinfo=timezone.utc)
    except ValueError:
        return None
    return max(0.0, time.time() - stamp.timestamp())

latch = load(latch_path)
nav = load(nav_path)
docking = load(docking_path)
check = nav.get("pre_navigation_dock_check") or docking.get("pre_navigation_dock_check") or {}

latch_active = bool(latch.get("latched_docked", latch.get("docked", False)))
latch_source = str(latch.get("source", "none"))
latch_age = check.get("dock_contact_latch_age_sec")
if latch_age is None:
    latch_age = parse_age(latch.get("latched_at") or latch.get("updated_at"))
if latch_age is None or (isinstance(latch_age, float) and not math.isfinite(latch_age)):
    latch_age = -1.0
latch_stale = bool(check.get("dock_contact_latch_stale", False))
if latch_source == "bms" and latch_active and latch_age >= 0 and latch_age > 300.0:
    latch_stale = True

docking_block = check.get("docking", {})
runtime_state = str(check.get("live_docking_state") or docking.get("state") or docking_block.get("state") or "")
status_docked = bool(check.get("live_docking_status_indicates_docked", docking_block.get("status_indicates_docked", False)))
status_charging = bool(check.get("live_docking_status_indicates_charging", docking_block.get("status_indicates_charging", False)))
bms_contact = bool(check.get("live_bms_charging_contact", check.get("api_bms_charging_contact", False)))
strong_live = bool(check.get("strong_live_docked", False))
latch_valid = bool(check.get("latch_valid_for_auto_undock", False))
auto_required = bool(check.get("final_auto_undock_required", False))
live_undocked = runtime_state.lower() == "undocked" or bool(docking_block.get("live_docking_state_undocked", False))

failures = []
warnings = []
passes = []

if latch_source == "bms" and latch_active and latch_stale and auto_required and not strong_live:
    failures.append("source=bms stale latch alone still triggers final_auto_undock_required")

if live_undocked and not status_docked and not status_charging and not bms_contact and auto_required:
    failures.append("live undocked/no_contact still reports final_auto_undock_required=true")

if latch_source == "bms" and latch_active and latch_stale and not live_undocked:
    warnings.append("stale source=bms latch exists but live undocked state is unknown")

if latch_source == "bms" and latch_active and latch_age > 600.0:
    warnings.append("source=bms latch age exceeds max-age warning threshold")

if latch_source == "bms" and latch_active and latch_stale and not auto_required:
    passes.append("stale source=bms latch is blocked from auto-undock")
elif not (latch_source == "bms" and latch_active and latch_stale):
    passes.append("no active stale source=bms latch")

if strong_live and auto_required:
    passes.append("strong live docked evidence still triggers auto-undock")
if latch_valid and auto_required:
    passes.append("valid non-stale latch evidence still triggers auto-undock")

analysis = {
    "latch_file": latch_path,
    "latch_active": latch_active,
    "latch_source": latch_source,
    "latch_age_sec": latch_age,
    "latch_stale": latch_stale,
    "runtime_docking_state": runtime_state,
    "status_indicates_docked": status_docked,
    "status_indicates_charging": status_charging,
    "live_bms_charging_contact": bms_contact,
    "strong_live_docked": strong_live,
    "latch_valid_for_auto_undock": latch_valid,
    "final_auto_undock_required": auto_required,
    "final_auto_undock_reason": check.get("final_auto_undock_reason", check.get("auto_undock_reason")),
    "dock_contact_latch_auto_cleared": bool(check.get("dock_contact_latch_auto_cleared", False)),
    "dock_contact_latch_clear_reason": check.get("dock_contact_latch_clear_reason", ""),
    "passes": passes,
    "warnings": warnings,
    "failures": failures,
    "clear_candidate": latch_source == "bms" and latch_active and latch_stale,
}
with open(analysis_path, "w", encoding="utf-8") as f:
    json.dump(analysis, f, indent=2, ensure_ascii=False)
PY

cat "${ANALYSIS_JSON}"
echo
echo "[dock-latch-gate] /docking/status sample:"
sed -n '1,40p' "${DOCKING_STATUS_TXT}"
echo "[dock-latch-gate] /battery_state sample:"
sed -n '1,40p' "${BMS_TXT}"

if python3 - "$ANALYSIS_JSON" <<'PY'
import json
import sys
with open(sys.argv[1], "r", encoding="utf-8") as f:
    data = json.load(f)
raise SystemExit(0 if data.get("clear_candidate") else 1)
PY
then
  if [[ "${CLEAR_STALE}" == "true" ]]; then
    echo "[dock-latch-gate] clearing stale source=bms latch through API"
    curl_json POST "${API_URL}/api/v1/docking/clear_docked_latch" \
      '{"reason":"verify_dock_contact_latch_gate_clear_stale_bms_latch","note":"explicit operator script flag"}'
  else
    echo "[dock-latch-gate] stale source=bms latch found; rerun with --clear-stale-bms-latch to clear it"
  fi
fi

python3 - "$ANALYSIS_JSON" <<'PY'
import json
import sys
with open(sys.argv[1], "r", encoding="utf-8") as f:
    data = json.load(f)
for item in data.get("passes", []):
    print(f"PASS: {item}")
for item in data.get("warnings", []):
    print(f"WARN: {item}")
for item in data.get("failures", []):
    print(f"FAIL: {item}")
raise SystemExit(1 if data.get("failures") else 0)
PY
