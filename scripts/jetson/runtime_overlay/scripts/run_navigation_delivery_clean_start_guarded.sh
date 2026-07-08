#!/usr/bin/env bash
set -euo pipefail
umask 0002

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OVERLAY_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
PROJECT_ROOT="$(cd "${OVERLAY_ROOT}/../../.." && pwd)"

API_URL="${API_URL:-http://127.0.0.1:8080}"
EXPECTED_START=""
CYCLES="1"
TIMEOUT_SEC="180"
MAX_START_XY_M="0.50"
MAX_START_YAW_DEG="5.0"
MAX_MAP_ODOM_TRANSLATION_M="0.70"
MAX_MAP_BASE_TRANSLATION_M="0.50"
MAX_YAW_DEG="3.0"
OUTPUT_ROOT="${PROJECT_ROOT}/reports/navigation_pingpong_clean_start_guarded"
LABEL="clean_start"

usage() {
  cat <<'EOF'
Usage: run_navigation_delivery_clean_start_guarded.sh --expected-start 512355|675235 [options]

Checks that the current API map pose is close to the expected delivery endpoint,
then sends the opposite endpoint through the existing guarded ping-pong script.
This prevents EKF A/B runs from starting with a contaminated terminal pose.

Options:
  --expected-start 512355|675235  Required current endpoint.
  --cycles N                      Ping-pong cycles passed to child script. Default: 1.
  --timeout-sec SEC               Per-leg timeout. Default: 180.
  --max-start-xy-m M              Required start XY distance to expected endpoint. Default: 0.50.
  --max-start-yaw-deg DEG         Required start yaw error to expected endpoint. Default: 5.0.
  --max-map-odom-translation-m M  Child guard threshold. Default: 0.70.
  --max-map-base-translation-m M  Child guard threshold. Default: 0.50.
  --max-yaw-deg DEG               Child guard yaw threshold. Default: 3.0.
  --output-root DIR               Report root.
  --label NAME                    Report label suffix. Default: clean_start.
  --api-url URL                   robot_api_server URL. Default: http://127.0.0.1:8080.
EOF
}

while [[ "$#" -gt 0 ]]; do
  case "$1" in
    --expected-start)
      EXPECTED_START="${2:-}"
      shift 2
      ;;
    --cycles)
      CYCLES="${2:-}"
      shift 2
      ;;
    --timeout-sec)
      TIMEOUT_SEC="${2:-}"
      shift 2
      ;;
    --max-start-xy-m)
      MAX_START_XY_M="${2:-}"
      shift 2
      ;;
    --max-start-yaw-deg)
      MAX_START_YAW_DEG="${2:-}"
      shift 2
      ;;
    --max-map-odom-translation-m)
      MAX_MAP_ODOM_TRANSLATION_M="${2:-}"
      shift 2
      ;;
    --max-map-base-translation-m)
      MAX_MAP_BASE_TRANSLATION_M="${2:-}"
      shift 2
      ;;
    --max-yaw-deg)
      MAX_YAW_DEG="${2:-}"
      shift 2
      ;;
    --output-root)
      OUTPUT_ROOT="${2:-}"
      shift 2
      ;;
    --label)
      LABEL="${2:-}"
      shift 2
      ;;
    --api-url)
      API_URL="${2:-}"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "[nav-clean-start] unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

case "${EXPECTED_START}" in
  512355|delivery_512355) EXPECTED_START="512355" ;;
  675235|delivery_675235) EXPECTED_START="675235" ;;
  *)
    echo "[nav-clean-start] --expected-start must be 512355 or 675235" >&2
    usage >&2
    exit 2
    ;;
esac

if [[ -f "${SCRIPT_DIR}/common_env.sh" ]]; then
  # shellcheck source=/dev/null
  source "${SCRIPT_DIR}/common_env.sh"
fi
PROJECT_ROOT="${NJRH_PROJECT_ROOT:-${PROJECT_ROOT}}"
OUTPUT_ROOT="${OUTPUT_ROOT/#\~/${HOME}}"

safe_label="$(printf '%s' "${LABEL}" | tr -c 'A-Za-z0-9_.-' '_')"
timestamp="$(date -u +%Y%m%dT%H%M%SZ)"
OUT_DIR="${OUTPUT_ROOT}/${timestamp}_${safe_label}_expected_${EXPECTED_START}"
mkdir -p "${OUT_DIR}"

set +e
guard_output="$(
  python3 - \
    "${OUT_DIR}" \
    "${API_URL}" \
    "${EXPECTED_START}" \
    "${MAX_START_XY_M}" \
    "${MAX_START_YAW_DEG}" <<'PY'
import json
import math
import sys
import urllib.request
from datetime import datetime, timezone
from pathlib import Path

out_dir = Path(sys.argv[1])
api_url = sys.argv[2].rstrip("/")
expected = sys.argv[3]
max_xy = float(sys.argv[4])
max_yaw_deg = float(sys.argv[5])
max_yaw_rad = math.radians(max_yaw_deg)

poses = {
    "512355": {"x": -7.455135, "y": 8.007710, "yaw": 1.570796},
    "675235": {"x": -6.364297, "y": -3.032889, "yaw": -1.603842},
}

def now_iso() -> str:
    return datetime.now(timezone.utc).replace(microsecond=0).isoformat()

def angle_diff(a: float, b: float) -> float:
    return math.atan2(math.sin(a - b), math.cos(a - b))

def get_json(path: str):
    with urllib.request.urlopen(api_url + path, timeout=5.0) as response:
        return json.loads(response.read().decode("utf-8"))

pose_response = get_json("/api/v1/robot/pose")
nav_response = get_json("/api/v1/navigation/state")
if not pose_response.get("ok"):
    raise SystemExit("robot pose API returned ok=false")

x = float(pose_response["x"])
y = float(pose_response["y"])
yaw = float(pose_response["yaw"])
expected_pose = poses[expected]
expected_dx = x - expected_pose["x"]
expected_dy = y - expected_pose["y"]
expected_xy = math.hypot(expected_dx, expected_dy)
expected_dyaw = abs(angle_diff(yaw, expected_pose["yaw"]))

distances = {}
for name, target in poses.items():
    dx = x - target["x"]
    dy = y - target["y"]
    distances[name] = {
        "xy_m": math.hypot(dx, dy),
        "dyaw_rad": abs(angle_diff(yaw, target["yaw"])),
        "dyaw_deg": math.degrees(abs(angle_diff(yaw, target["yaw"]))),
    }
nearest = min(distances, key=lambda name: distances[name]["xy_m"])
first_target = "675235" if expected == "512355" else "512355"

ok = expected_xy <= max_xy and expected_dyaw <= max_yaw_rad
summary_path = out_dir / "summary.md"
payload = {
    "time_utc": now_iso(),
    "api_url": api_url,
    "expected_start": expected,
    "first_target": first_target,
    "max_start_xy_m": max_xy,
    "max_start_yaw_deg": max_yaw_deg,
    "robot_pose": pose_response,
    "navigation_state": nav_response,
    "distance_to_expected_xy_m": expected_xy,
    "yaw_error_to_expected_rad": expected_dyaw,
    "yaw_error_to_expected_deg": math.degrees(expected_dyaw),
    "nearest_delivery": nearest,
    "distances": distances,
    "decision": "continue" if ok else "stop_start_guard_failed",
}
(out_dir / "start_guard.json").write_text(json.dumps(payload, indent=2, sort_keys=True), encoding="utf-8")
with summary_path.open("w", encoding="utf-8") as f:
    f.write("# Navigation Clean Start Guard\n\n")
    f.write(f"- time_utc: `{payload['time_utc']}`\n")
    f.write(f"- expected_start: `delivery_{expected}`\n")
    f.write(f"- first_target_if_continue: `delivery_{first_target}`\n")
    f.write(f"- robot_pose_xy_yaw: `({x:.6f}, {y:.6f}, {yaw:.6f})`\n")
    f.write(f"- distance_to_expected_xy_m: `{expected_xy:.6f}`\n")
    f.write(f"- yaw_error_to_expected_deg: `{math.degrees(expected_dyaw):.6f}`\n")
    f.write(f"- nearest_delivery: `delivery_{nearest}`\n")
    f.write(f"- max_start_xy_m: `{max_xy}`\n")
    f.write(f"- max_start_yaw_deg: `{max_yaw_deg}`\n")
    f.write(f"- decision: `{payload['decision']}`\n")
    f.write("\n| delivery | xy_m | yaw_deg |\n")
    f.write("|---|---:|---:|\n")
    for name in ("512355", "675235"):
        f.write(
            f"| delivery_{name} | `{distances[name]['xy_m']:.6f}` | "
            f"`{distances[name]['dyaw_deg']:.6f}` |\n"
        )

print(f"summary_path={summary_path}")
print(f"first_target={first_target}")
raise SystemExit(0 if ok else 10)
PY
)"
guard_rc=$?
set -e

summary_path="$(printf '%s\n' "${guard_output}" | awk -F= '$1 == "summary_path" {print $2}' | tail -n 1)"
first_target="$(printf '%s\n' "${guard_output}" | awk -F= '$1 == "first_target" {print $2}' | tail -n 1)"

if [[ "${guard_rc}" != "0" ]]; then
  echo "${guard_output}"
  echo "[nav-clean-start] start guard failed; summary: ${summary_path:-${OUT_DIR}/summary.md}" >&2
  exit "${guard_rc}"
fi

echo "[nav-clean-start] start guard passed; summary: ${summary_path}"
echo "[nav-clean-start] first target: delivery_${first_target}"

child_root="${OUT_DIR}/child_pingpong"
set +e
bash "${SCRIPT_DIR}/run_navigation_delivery_pingpong_guarded.sh" \
  --cycles "${CYCLES}" \
  --start-target "${first_target}" \
  --timeout-sec "${TIMEOUT_SEC}" \
  --max-map-odom-translation-m "${MAX_MAP_ODOM_TRANSLATION_M}" \
  --max-map-base-translation-m "${MAX_MAP_BASE_TRANSLATION_M}" \
  --max-yaw-deg "${MAX_YAW_DEG}" \
  --output-root "${child_root}"
child_rc=$?
set -e

child_summary="$(find "${child_root}" -mindepth 2 -maxdepth 2 -type f -name summary.md | sort | tail -n 1 || true)"
{
  echo
  echo "## Child Guarded Navigation"
  echo
  echo "- child_exit_code: \`${child_rc}\`"
  if [[ -n "${child_summary}" ]]; then
    echo "- child_summary: \`${child_summary#${PROJECT_ROOT}/}\`"
  else
    echo "- child_summary: \`missing\`"
  fi
} >>"${summary_path}"

echo "[nav-clean-start] summary: ${summary_path}"
exit "${child_rc}"
