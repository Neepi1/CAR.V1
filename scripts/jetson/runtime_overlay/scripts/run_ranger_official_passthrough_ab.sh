#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/common_env.sh"

PROFILE="official_passthrough"
DURATION_SEC=180
APPLY=0
DO_RESTART=0

usage() {
  cat <<'EOF'
Usage:
  run_ranger_official_passthrough_ab.sh --profile official_passthrough|custom
      [--duration-sec 180] [--apply] [--restart]

This script does not send navigation goals or publish motion commands.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --profile)
      PROFILE="${2:-}"
      shift 2
      ;;
    --duration-sec)
      DURATION_SEC="${2:-180}"
      shift 2
      ;;
    --apply)
      APPLY=1
      shift
      ;;
    --restart)
      DO_RESTART=1
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "[ranger-pass-ab] unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

case "${PROFILE}" in
  official_passthrough|custom) ;;
  *) echo "[ranger-pass-ab] invalid profile: ${PROFILE}" >&2; exit 2 ;;
esac

if [[ "${APPLY}" -eq 1 ]]; then
  args=(--profile "${PROFILE}" --print)
  [[ "${DO_RESTART}" -eq 1 ]] && args+=(--restart)
  bash "${SCRIPT_DIR}/set_ranger_mode_controller_profile.sh" "${args[@]}"
fi

ts="$(date -u +%Y%m%dT%H%M%SZ)"
report_dir="${NJRH_PROJECT_ROOT}/reports/ranger_official_passthrough_ab_${ts}"
mkdir -p "${report_dir}"
report="${report_dir}.md"

echo "[ranger-pass-ab] observing profile=${PROFILE} duration=${DURATION_SEC}s"

timeout "${DURATION_SEC}" ros2 topic echo --field data /ranger_mini3_mode_controller/status \
  >"${report_dir}/mode_controller_status.log" 2>&1 &
pids=($!)
timeout "${DURATION_SEC}" ros2 topic echo /cmd_vel_safe \
  >"${report_dir}/cmd_vel_safe.log" 2>&1 &
pids+=($!)
timeout "${DURATION_SEC}" ros2 topic echo /cmd_vel \
  >"${report_dir}/cmd_vel.log" 2>&1 &
pids+=($!)
timeout "${DURATION_SEC}" ros2 topic echo /motion_state \
  >"${report_dir}/motion_state.log" 2>&1 &
pids+=($!)
timeout "${DURATION_SEC}" ros2 topic echo /system_state \
  >"${report_dir}/system_state.log" 2>&1 &
pids+=($!)
timeout "${DURATION_SEC}" ros2 topic echo /wheel/odom \
  >"${report_dir}/wheel_odom.log" 2>&1 &
pids+=($!)
timeout "${DURATION_SEC}" ros2 topic echo /wheel/odom_ekf \
  >"${report_dir}/wheel_odom_ekf.log" 2>&1 &
pids+=($!)
timeout "${DURATION_SEC}" ros2 topic echo /local_state/odometry \
  >"${report_dir}/local_state_odometry.log" 2>&1 &
pids+=($!)
timeout "${DURATION_SEC}" ros2 topic echo --field data /localization/bridge_status \
  >"${report_dir}/bridge_status.log" 2>&1 &
pids+=($!)

sleep "${DURATION_SEC}"
for pid in "${pids[@]}"; do
  wait "${pid}" 2>/dev/null || true
done

nav_state="$(curl -s --max-time 2 http://127.0.0.1:8080/api/v1/navigation/state || true)"
status_last="$(grep -E '^\{' "${report_dir}/mode_controller_status.log" | tail -n 1 || true)"

python3 - "${report}" "${PROFILE}" "${DURATION_SEC}" "${status_last}" "${nav_state}" <<'PY'
import json
import pathlib
import sys

report = pathlib.Path(sys.argv[1])
profile = sys.argv[2]
duration = sys.argv[3]
status_raw = sys.argv[4]
nav_raw = sys.argv[5]

def loads(raw):
    try:
        return json.loads(raw) if raw else {}
    except Exception:
        return {}

status = loads(status_raw)
nav = loads(nav_raw)
goal = nav.get("navigation_goal", {}) if isinstance(nav, dict) else {}

report.write_text(
    "\n".join([
        "# Ranger Official Passthrough A/B Observation",
        "",
        f"- profile: `{profile}`",
        f"- duration_sec: `{duration}`",
        f"- mode_controller_profile: `{status.get('mode_controller_profile', 'unknown')}`",
        f"- custom_ackermann_enabled: `{status.get('custom_ackermann_enabled', 'unknown')}`",
        f"- cmd_vel_passthrough: `{status.get('cmd_vel_passthrough', 'unknown')}`",
        f"- passthrough_preserves_twist: `{status.get('passthrough_preserves_twist', 'unknown')}`",
        f"- output_diff_from_input: `{status.get('output_diff_from_input', 'unknown')}`",
        f"- diff_reason: `{status.get('diff_reason', '')}`",
        f"- active nav goal id: `{goal.get('id', '')}`",
        f"- nav state: `{goal.get('state', '')}`",
        f"- final_distance_m: `{goal.get('final_distance_m', '')}`",
        f"- final_yaw_error_rad: `{goal.get('final_yaw_error_rad', '')}`",
        "",
        "Raw logs are stored beside this report directory.",
        "",
    ]),
    encoding="utf-8",
)
PY

echo "[ranger-pass-ab] report ${report}"
