#!/usr/bin/env bash
set -euo pipefail

DRY_RUN=0
MOCK_CHARGING_OBSERVED=0
MOCK_FULL_CHARGE_IDLE=0
MOCK_BMS_NO_CONTACT=0
MOCK_DOCKING_JOB_LATCH=0
EXPECT_AUTO_UNDOCK=0
EXPECT_DOCKED_CHARGE_IDLE=0
EXPECT_NO_LATCH_CLEAR=0
API_BASE="${API_BASE:-http://127.0.0.1:8080}"

usage() {
  cat <<'EOF'
Usage: verify_full_charge_dock_session_gate.sh [options]

Options:
  --dry-run
  --mock-charging-observed
  --mock-full-charge-idle
  --mock-bms-no-contact
  --mock-docking-job-latch
  --expect-auto-undock
  --expect-docked-charge-idle
  --expect-no-latch-clear

This script is read-only. Mock mode does not write the runtime latch file,
does not send goals, and does not publish velocity.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --dry-run) DRY_RUN=1 ;;
    --mock-charging-observed) MOCK_CHARGING_OBSERVED=1 ;;
    --mock-full-charge-idle) MOCK_FULL_CHARGE_IDLE=1 ;;
    --mock-bms-no-contact) MOCK_BMS_NO_CONTACT=1 ;;
    --mock-docking-job-latch) MOCK_DOCKING_JOB_LATCH=1 ;;
    --expect-auto-undock) EXPECT_AUTO_UNDOCK=1 ;;
    --expect-docked-charge-idle) EXPECT_DOCKED_CHARGE_IDLE=1 ;;
    --expect-no-latch-clear) EXPECT_NO_LATCH_CLEAR=1 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "[full-charge-gate] unknown option: $1" >&2; usage >&2; exit 2 ;;
  esac
  shift
done

has_mock=0
if [[ "${MOCK_CHARGING_OBSERVED}" == "1" || "${MOCK_FULL_CHARGE_IDLE}" == "1" || \
      "${MOCK_BMS_NO_CONTACT}" == "1" || "${MOCK_DOCKING_JOB_LATCH}" == "1" ]]; then
  has_mock=1
fi

if [[ "${has_mock}" == "1" || "${DRY_RUN}" == "1" ]]; then
  python3 - "$MOCK_CHARGING_OBSERVED" "$MOCK_FULL_CHARGE_IDLE" "$MOCK_BMS_NO_CONTACT" \
    "$MOCK_DOCKING_JOB_LATCH" "$EXPECT_AUTO_UNDOCK" "$EXPECT_DOCKED_CHARGE_IDLE" \
    "$EXPECT_NO_LATCH_CLEAR" <<'PY'
import json
import sys

charging_observed = sys.argv[1] == "1"
full_charge_idle = sys.argv[2] == "1"
bms_no_contact = sys.argv[3] == "1" or full_charge_idle
docking_job_latch = sys.argv[4] == "1"
expect_auto = sys.argv[5] == "1"
expect_idle = sys.argv[6] == "1"
expect_no_clear = sys.argv[7] == "1"

source = "none"
if docking_job_latch:
    source = "docking_job"
elif charging_observed:
    source = "charging_session"

strong_latch = source in {"charging_session", "docking_job"}
latch_cleared = False
if strong_latch and bms_no_contact and full_charge_idle:
    state = "DOCKED_CHARGE_IDLE"
    reason = "charging_session_or_docking_latch_with_bms_idle"
elif strong_latch and bms_no_contact:
    state = "UNCERTAIN_ON_DOCK"
    reason = "recent_strong_dock_or_charging_session_latch"
elif charging_observed:
    state = "DOCKED_CHARGING"
    reason = "live_charging_evidence"
else:
    state = "UNKNOWN"
    reason = "no_strong_dock_evidence"

auto_undock = state in {
    "CONFIRMED_DOCKED",
    "DOCKED_CHARGING",
    "DOCKED_CHARGE_IDLE",
    "UNCERTAIN_ON_DOCK",
}
result = {
    "ok": True,
    "mode": "mock",
    "dock_occupancy_state": state,
    "dock_occupancy_reason": reason,
    "source": source,
    "charging_session_latched": source == "charging_session",
    "bms_live_contact": not bms_no_contact and charging_observed,
    "bms_percentage": 100.0 if full_charge_idle else None,
    "bms_current": 0.0 if bms_no_contact else 0.2,
    "bms_present": not bms_no_contact,
    "full_charge_idle_on_dock": full_charge_idle and strong_latch,
    "final_auto_undock_required": auto_undock,
    "latch_cleared": latch_cleared,
}
print(json.dumps(result, indent=2, sort_keys=True))

failures = []
if expect_auto and not auto_undock:
    failures.append("expected final_auto_undock_required=true")
if expect_idle and state != "DOCKED_CHARGE_IDLE":
    failures.append(f"expected DOCKED_CHARGE_IDLE, got {state}")
if expect_no_clear and latch_cleared:
    failures.append("expected latch not to be cleared")
if failures:
    for item in failures:
        print(f"[full-charge-gate] FAIL: {item}", file=sys.stderr)
    sys.exit(1)
print("[full-charge-gate] PASS")
PY
  exit $?
fi

if ! command -v curl >/dev/null 2>&1; then
  echo "[full-charge-gate] curl is required for live mode" >&2
  exit 1
fi

nav_json="$(curl -fsS --max-time 5 "${API_BASE}/api/v1/navigation/state")"
dock_json="$(curl -fsS --max-time 5 "${API_BASE}/api/v1/docking/state")"
python3 - "$nav_json" "$dock_json" "$EXPECT_AUTO_UNDOCK" "$EXPECT_DOCKED_CHARGE_IDLE" \
  "$EXPECT_NO_LATCH_CLEAR" <<'PY'
import json
import sys

nav = json.loads(sys.argv[1])
dock = json.loads(sys.argv[2])
expect_auto = sys.argv[3] == "1"
expect_idle = sys.argv[4] == "1"
expect_no_clear = sys.argv[5] == "1"
check = nav.get("pre_navigation_dock_check") or {}
state = nav.get("dock_occupancy_state") or check.get("dock_occupancy_state")
auto = bool(check.get("final_auto_undock_required", False))
cleared = bool(check.get("dock_contact_latch_auto_cleared", False))
summary = {
    "navigation_dock_occupancy_state": state,
    "navigation_dock_occupancy_reason": nav.get("dock_occupancy_reason") or check.get("dock_occupancy_reason"),
    "docking_dock_occupancy_state": dock.get("dock_occupancy_state"),
    "charging_session_latched": check.get("charging_session_latched"),
    "charging_session_age_sec": check.get("charging_session_age_sec"),
    "full_charge_idle_on_dock": check.get("full_charge_idle_on_dock"),
    "final_auto_undock_required": auto,
    "auto_undock_reason": check.get("auto_undock_reason"),
    "dock_contact_latch_auto_cleared": cleared,
}
print(json.dumps(summary, indent=2, sort_keys=True))
failures = []
if expect_auto and not auto:
    failures.append("expected final_auto_undock_required=true")
if expect_idle and state != "DOCKED_CHARGE_IDLE":
    failures.append(f"expected DOCKED_CHARGE_IDLE, got {state}")
if expect_no_clear and cleared:
    failures.append("expected latch not to be auto-cleared")
if failures:
    for item in failures:
        print(f"[full-charge-gate] FAIL: {item}", file=sys.stderr)
    sys.exit(1)
print("[full-charge-gate] PASS")
PY
