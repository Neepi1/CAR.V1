#!/usr/bin/env bash
set -euo pipefail
umask 0002

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OVERLAY_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
PROJECT_ROOT="$(cd "${OVERLAY_ROOT}/../../.." && pwd)"

API_URL="${API_URL:-http://127.0.0.1:8080}"
PROFILE=""
TARGET_A="675235"
TARGET_B="512355"
START_TARGET=""
EXPECTED_START=""
CYCLES="1"
TIMEOUT_SEC="180"
START_GUARD_POLICY="readiness_only"
MAX_START_XY_M="0.50"
MAX_START_YAW_DEG="5.0"
MAX_ONLINE_XY_M="0.20"
MAX_ONLINE_YAW_RAD="0.08"
MAX_MAP_ODOM_TRANSLATION_M="0.30"
MAX_MAP_BASE_TRANSLATION_M="0.50"
MAX_YAW_DEG="2.0"
GOAL_COMPLETION_POLICY="pose_required"
OUTPUT_ROOT="${PROJECT_ROOT}/reports/ekf_profile_delivery_ab_guarded"
APPLY="false"
PREFLIGHT_ONLY="false"
RESTORE_PROFILE="wheel_imu"
LOCAL_STATE_READY_TIMEOUT_SEC="20"
RELOCALIZE_TIMEOUT_SEC="60"
RELOCALIZE_SETTLE_TIMEOUT_SEC="20"
NAV_RESTART_READY_TIMEOUT_SEC="180"
FULL_RUNTIME_RESTART_CMD="${NJRH_FULL_RUNTIME_RESTART_CMD:-sudo systemctl restart njrh-runtime.service}"
RUNTIME_OVERRIDE_ENV="${NJRH_RUNTIME_OVERRIDE_ENV:-/tmp/njrh_runtime_override.env}"

usage() {
  cat <<'EOF'
Usage: run_ekf_profile_delivery_ab_guarded.sh --profile PROFILE [options] [--apply]

Default mode is dry-run. It prints the exact EKF A/B plan and does not restart
nodes, call relocalization, send navigation goals, or publish velocity.

With --apply, the script:
  1. restarts the full resident navigation runtime with the requested EKF profile;
  2. runs one explicit pre-A/B relocalization capture;
  3. runs target-a <-> target-b through the guarded API/Nav2 path;
  4. always restores the field-default wheel_imu profile through a full
     navigation runtime restart and relocalizes again.

Options:
  --profile PROFILE                 EKF profile to test.
  --target-a ID                     First saved delivery suffix/id. Default: 675235.
  --target-b ID                     Second saved delivery suffix/id. Default: 512355.
  --start-target ID                 First target for ping-pong. Default: target-a.
  --expected-start ID               Reference endpoint for start reporting.
                                     Default: opposite of --start-target.
  --cycles N                        Number of target-a->target-b pairs. Default: 1.
  --timeout-sec SEC                 Per-leg navigation timeout. Default: 180.
  --start-guard-policy POLICY       readiness_only or pose_required.
                                    readiness_only checks API/Nav2 readiness only.
                                    pose_required also requires current pose near
                                    --expected-start. Default: readiness_only.
  --max-start-xy-m M                Required start XY distance. Default: 0.50.
  --max-start-yaw-deg DEG           Required start yaw error. Default: 5.0.
  --max-online-xy-m M               API final XY gate. Default: 0.20.
  --max-online-yaw-rad RAD          API final yaw gate. Default: 0.08.
  --max-map-odom-translation-m M    map->odom correction gate. Default: 0.30.
  --max-map-base-translation-m M    map->base_link truth gate. Default: 0.50.
  --max-yaw-deg DEG                 Correction yaw gate. Default: 2.0.
  --goal-completion-policy POLICY   API goal policy for ping-pong legs:
                                    pose_required or position_only.
                                    Default: pose_required.
  --output-root DIR                 Report root.
  --preflight-only                  Read API pose/start readiness and exit.
                                     Does not restart nodes or move the robot.
  --apply                           Execute the plan. Required to move the robot.
  --local-state-ready-timeout-sec SEC
  --nav-restart-ready-timeout-sec SEC
  --runtime-restart-command CMD     Full product runtime restart command.
                                    Default: sudo systemctl restart njrh-runtime.service.
  --relocalize-timeout-sec SEC
  --relocalize-settle-timeout-sec SEC
  --api-url URL                     robot_api_server URL. Default: http://127.0.0.1:8080.

This script never publishes velocity and never bypasses robot_safety. Movement,
when --apply is used, is only through run_navigation_delivery_pingpong_guarded.sh.
EOF
}

while [[ "$#" -gt 0 ]]; do
  case "$1" in
    --profile)
      PROFILE="${2:-}"
      shift 2
      ;;
    --target-a)
      TARGET_A="${2:-}"
      shift 2
      ;;
    --target-b)
      TARGET_B="${2:-}"
      shift 2
      ;;
    --start-target)
      START_TARGET="${2:-}"
      shift 2
      ;;
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
    --start-guard-policy)
      START_GUARD_POLICY="${2:-}"
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
    --max-online-xy-m)
      MAX_ONLINE_XY_M="${2:-}"
      shift 2
      ;;
    --max-online-yaw-rad)
      MAX_ONLINE_YAW_RAD="${2:-}"
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
    --goal-completion-policy)
      GOAL_COMPLETION_POLICY="${2:-}"
      shift 2
      ;;
    --output-root)
      OUTPUT_ROOT="${2:-}"
      shift 2
      ;;
    --preflight-only)
      PREFLIGHT_ONLY="true"
      shift
      ;;
    --local-state-ready-timeout-sec)
      LOCAL_STATE_READY_TIMEOUT_SEC="${2:-}"
      shift 2
      ;;
    --nav-restart-ready-timeout-sec)
      NAV_RESTART_READY_TIMEOUT_SEC="${2:-}"
      shift 2
      ;;
    --runtime-restart-command)
      FULL_RUNTIME_RESTART_CMD="${2:-}"
      shift 2
      ;;
    --relocalize-timeout-sec)
      RELOCALIZE_TIMEOUT_SEC="${2:-}"
      shift 2
      ;;
    --relocalize-settle-timeout-sec)
      RELOCALIZE_SETTLE_TIMEOUT_SEC="${2:-}"
      shift 2
      ;;
    --api-url)
      API_URL="${2:-}"
      shift 2
      ;;
    --apply)
      APPLY="true"
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "[ekf-ab-guarded] unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

if [[ -f "${SCRIPT_DIR}/common_env.sh" ]]; then
  # shellcheck source=/dev/null
  source "${SCRIPT_DIR}/common_env.sh"
fi
PROJECT_ROOT="${NJRH_PROJECT_ROOT:-${PROJECT_ROOT}}"
NJRH_OVERLAY_ROOT="${NJRH_OVERLAY_ROOT:-${OVERLAY_ROOT}}"
OUTPUT_ROOT="${OUTPUT_ROOT/#\~/${HOME}}"

validate_profile() {
  case "$1" in
    wheel_imu_primary_vyaw|imu_primary_vyaw|wheel_imu|wheel_pose_imu_vyaw|pose_imu_vyaw|wheel_imu_pose_soft_yaw_015|pose_soft_yaw_015|\
    wheel_imu_twist_soft_yaw_012|twist_soft_yaw_012|wheel_imu_twist_soft_yaw_010|twist_soft_yaw_010|wheel_imu_twist_soft_yaw_015|twist_soft_yaw_015|wheel_imu_soft_yaw_018|soft_yaw_018|\
    wheel_imu_soft_yaw_016|soft_yaw_016|\
    wheel_imu_soft_yaw_014|soft_yaw_014|wheel_imu_soft_yaw_010|soft_yaw_010|wheel_imu_soft_yaw_015|soft_yaw_015|\
    wheel_imu_soft_yaw|soft_yaw|wheel_xy_imu_vyaw|xy_imu_vyaw|wheel_xy_imu_yaw|xy_imu_yaw|\
    twist_imu|wheel_twist_imu|twist_imu_vyaw_only|wheel_twist_imu_vyaw_only|twist_imu_gyro_only|twist_wheel_yaw_imu|wheel_yaw_twist_imu|wheel_only)
      return 0
      ;;
    *)
      return 1
      ;;
  esac
}

[[ -n "${PROFILE}" ]] || {
  echo "[ekf-ab-guarded] --profile is required" >&2
  usage >&2
  exit 2
}
validate_profile "${PROFILE}" || {
  echo "[ekf-ab-guarded] unsupported EKF profile: ${PROFILE}" >&2
  exit 2
}

normalize_delivery_id() {
  local value="$1"
  value="${value#delivery_}"
  [[ -n "${value}" ]] || return 1
  [[ "${value}" =~ ^[A-Za-z0-9_.-]+$ ]] || return 1
  printf '%s\n' "${value}"
}

TARGET_A="$(normalize_delivery_id "${TARGET_A}")" || {
  echo "[ekf-ab-guarded] --target-a must be a non-empty safe delivery id" >&2
  exit 2
}
TARGET_B="$(normalize_delivery_id "${TARGET_B}")" || {
  echo "[ekf-ab-guarded] --target-b must be a non-empty safe delivery id" >&2
  exit 2
}
[[ "${TARGET_A}" != "${TARGET_B}" ]] || {
  echo "[ekf-ab-guarded] --target-a and --target-b must differ" >&2
  exit 2
}
if [[ -z "${START_TARGET}" ]]; then
  START_TARGET="${TARGET_A}"
fi
START_TARGET="$(normalize_delivery_id "${START_TARGET}")" || {
  echo "[ekf-ab-guarded] --start-target must be a non-empty safe delivery id" >&2
  exit 2
}
if [[ "${START_TARGET}" != "${TARGET_A}" && "${START_TARGET}" != "${TARGET_B}" ]]; then
  echo "[ekf-ab-guarded] --start-target must match --target-a or --target-b" >&2
  exit 2
fi
if [[ -z "${EXPECTED_START}" ]]; then
  if [[ "${START_TARGET}" == "${TARGET_A}" ]]; then
    EXPECTED_START="${TARGET_B}"
  else
    EXPECTED_START="${TARGET_A}"
  fi
else
  EXPECTED_START="$(normalize_delivery_id "${EXPECTED_START}")" || {
    echo "[ekf-ab-guarded] --expected-start must be a non-empty safe delivery id" >&2
    exit 2
  }
  if [[ "${EXPECTED_START}" != "${TARGET_A}" && "${EXPECTED_START}" != "${TARGET_B}" ]]; then
    echo "[ekf-ab-guarded] --expected-start must match --target-a or --target-b" >&2
    exit 2
  fi
fi

case "${GOAL_COMPLETION_POLICY}" in
  pose_required|position_only) ;;
  *)
    echo "[ekf-ab-guarded] --goal-completion-policy must be pose_required or position_only" >&2
    exit 2
    ;;
esac
case "${START_GUARD_POLICY}" in
  readiness_only|pose_required) ;;
  *)
    echo "[ekf-ab-guarded] --start-guard-policy must be readiness_only or pose_required" >&2
    exit 2
    ;;
esac

python3 - \
  "${CYCLES}" \
  "${TIMEOUT_SEC}" \
  "${MAX_START_XY_M}" \
  "${MAX_START_YAW_DEG}" \
  "${MAX_ONLINE_XY_M}" \
  "${MAX_ONLINE_YAW_RAD}" \
  "${MAX_MAP_ODOM_TRANSLATION_M}" \
  "${MAX_MAP_BASE_TRANSLATION_M}" \
  "${MAX_YAW_DEG}" \
  "${LOCAL_STATE_READY_TIMEOUT_SEC}" \
  "${NAV_RESTART_READY_TIMEOUT_SEC}" \
  "${RELOCALIZE_TIMEOUT_SEC}" \
  "${RELOCALIZE_SETTLE_TIMEOUT_SEC}" <<'PY'
import math
import sys

cycles = int(sys.argv[1])
values = [float(arg) for arg in sys.argv[2:]]
if cycles < 1:
    raise SystemExit("cycles must be >= 1")
if not all(math.isfinite(value) and value > 0.0 for value in values):
    raise SystemExit("numeric options must be finite positive values")
PY

timestamp="$(date -u +%Y%m%dT%H%M%SZ)"
OUT_DIR="${OUTPUT_ROOT}/${timestamp}_${PROFILE}_delivery_ab_guarded"
mkdir -p "${OUT_DIR}"

summary="${OUT_DIR}/summary.md"
{
  echo "# EKF Profile Delivery A/B Guarded"
  echo "- timestamp_utc: ${timestamp}"
  echo "- apply: \`${APPLY}\`"
  echo "- preflight_only: \`${PREFLIGHT_ONLY}\`"
  echo "- api_url: \`${API_URL}\`"
  echo "- profile: \`${PROFILE}\`"
  echo "- restore_profile: \`${RESTORE_PROFILE}\`"
  echo "- target_a: \`delivery_${TARGET_A}\`"
  echo "- target_b: \`delivery_${TARGET_B}\`"
  echo "- expected_start: \`delivery_${EXPECTED_START}\`"
  echo "- start_target: \`delivery_${START_TARGET}\`"
  echo "- start_guard_policy: \`${START_GUARD_POLICY}\`"
  echo "- cycles: \`${CYCLES}\`"
  echo "- timeout_sec: \`${TIMEOUT_SEC}\`"
  echo "- max_start_xy_m: \`${MAX_START_XY_M}\`"
  echo "- max_start_yaw_deg: \`${MAX_START_YAW_DEG}\`"
  echo "- max_online_xy_m: \`${MAX_ONLINE_XY_M}\`"
  echo "- max_online_yaw_rad: \`${MAX_ONLINE_YAW_RAD}\`"
  echo "- max_map_odom_translation_m: \`${MAX_MAP_ODOM_TRANSLATION_M}\`"
  echo "- max_map_base_translation_m: \`${MAX_MAP_BASE_TRANSLATION_M}\`"
  echo "- max_yaw_deg: \`${MAX_YAW_DEG}\`"
  echo "- goal_completion_policy: \`${GOAL_COMPLETION_POLICY}\`"
  echo "- nav_restart_ready_timeout_sec: \`${NAV_RESTART_READY_TIMEOUT_SEC}\`"
  echo "- full_runtime_restart_cmd: \`${FULL_RUNTIME_RESTART_CMD}\`"
  echo "- runtime_override_env: \`${RUNTIME_OVERRIDE_ENV}\`"
  echo "- default_mode: \`dry-run\`"
  echo "- sends_velocity: \`false\`"
  echo "- direct_cmd_vel: \`false\`"
  echo "- movement_path: \`run_navigation_delivery_pingpong_guarded.sh -> robot_api_server/Nav2 -> robot_safety -> chassis\`"
  echo
  echo "## Plan"
  echo
  echo "1. Check API/Nav2 readiness; only enforce current pose near \`delivery_${EXPECTED_START}\` when \`start_guard_policy=pose_required\`."
  echo "2. Restart the full resident navigation runtime with \`${PROFILE}\`."
  echo "3. Run explicit restart relocalization/AMCL seed capture before the API goal-readiness gate."
  echo "4. Run explicit pre-A/B relocalization capture."
  echo "5. Run guarded delivery ping-pong."
  echo "6. Restore the full resident navigation runtime to \`${RESTORE_PROFILE}\` and relocalize."
} >"${summary}"

echo "[ekf-ab-guarded] report: ${OUT_DIR}"

run_start_guard() {
  local output_dir="$1"
  mkdir -p "${output_dir}"
  python3 - \
    "${output_dir}" \
    "${API_URL}" \
    "${EXPECTED_START}" \
    "${START_GUARD_POLICY}" \
    "${MAX_START_XY_M}" \
    "${MAX_START_YAW_DEG}" \
    "${TARGET_A}" \
    "${TARGET_B}" \
    "${PROJECT_ROOT}" <<'PY'
import json
import math
import sys
import urllib.request
from datetime import datetime, timezone
from pathlib import Path

out_dir = Path(sys.argv[1])
api_url = sys.argv[2].rstrip("/")
expected = sys.argv[3]
policy = sys.argv[4]
max_xy = float(sys.argv[5])
max_yaw_deg = float(sys.argv[6])
target_a = sys.argv[7]
target_b = sys.argv[8]
project_root = Path(sys.argv[9])
max_yaw_rad = math.radians(max_yaw_deg)
target_ids = (target_a, target_b)

def now_iso() -> str:
    return datetime.now(timezone.utc).replace(microsecond=0).isoformat()

def angle_diff(a: float, b: float) -> float:
    return math.atan2(math.sin(a - b), math.cos(a - b))

def finite(value):
    try:
        value = float(value)
    except Exception:
        return None
    return value if math.isfinite(value) else None

def get_json(path: str):
    with urllib.request.urlopen(api_url + path, timeout=5.0) as response:
        return json.loads(response.read().decode("utf-8"))

def find_runtime_context(data):
    if not isinstance(data, dict):
        return None
    candidates = [
        data.get("runtime_map_context"),
        data.get("map_context"),
    ]
    body = data.get("body")
    if isinstance(body, dict):
        candidates.extend([
            body.get("runtime_map_context"),
            body.get("map_context"),
        ])
    for candidate in candidates:
        if not isinstance(candidate, dict):
            continue
        building_id = str(candidate.get("building_id") or "")
        floor_id = str(candidate.get("floor_id") or "")
        if building_id and floor_id:
            return building_id, floor_id
    return None

def parse_poses_yaml(path: Path):
    poses = {}
    current = None

    def commit():
        if not current:
            return
        pose_id = str(current.get("id") or "")
        x = finite(current.get("x"))
        y = finite(current.get("y"))
        yaw = finite(current.get("yaw"))
        if pose_id.startswith("delivery_") and x is not None and y is not None and yaw is not None:
            poses[pose_id.removeprefix("delivery_")] = {"x": x, "y": y, "yaw": yaw}

    if not path.exists():
        return poses
    for raw in path.read_text(encoding="utf-8", errors="replace").splitlines():
        line = raw.strip()
        if line.startswith("- id:"):
            commit()
            value = line.split(":", 1)[1].strip().strip('"').strip("'")
            current = {"id": value}
        elif current is not None and ":" in line:
            key, value = line.split(":", 1)
            key = key.strip()
            value = value.strip().strip('"').strip("'")
            if key in {"x", "y", "yaw"}:
                current[key] = value
    commit()
    return poses

pose_response = get_json("/api/v1/robot/pose")
nav_response = get_json("/api/v1/navigation/state")
status_response = get_json("/api/v1/status")
if not pose_response.get("ok"):
    raise SystemExit("robot pose API returned ok=false")

runtime_context = find_runtime_context(nav_response) or find_runtime_context(status_response)
if runtime_context is None:
    raise SystemExit("could not resolve runtime map context for start guard")
building_id, floor_id = runtime_context
pose_files = [
    project_root / "maps_release" / building_id / floor_id / "current" / "poses.yaml",
    project_root / "maps_release" / building_id / floor_id / "poses.yaml",
]
poses = {}
pose_file_used = ""
for pose_file in pose_files:
    poses = parse_poses_yaml(pose_file)
    if all(target in poses for target in target_ids):
        pose_file_used = str(pose_file)
        break

x = float(pose_response["x"])
y = float(pose_response["y"])
yaw = float(pose_response["yaw"])
expected_pose = poses.get(expected)
if expected_pose is None and policy == "pose_required":
    raise SystemExit(f"expected start delivery_{expected} missing from current poses.yaml")
if expected_pose is not None:
    xy = math.hypot(x - expected_pose["x"], y - expected_pose["y"])
    dyaw = abs(angle_diff(yaw, expected_pose["yaw"]))
else:
    xy = None
    dyaw = None

distances = {}
for name in target_ids:
    target = poses.get(name)
    if target is None:
        continue
    target_dyaw = abs(angle_diff(yaw, target["yaw"]))
    distances[name] = {
        "xy_m": math.hypot(x - target["x"], y - target["y"]),
        "dyaw_rad": target_dyaw,
        "dyaw_deg": math.degrees(target_dyaw),
    }
nearest = min(distances, key=lambda name: distances[name]["xy_m"]) if distances else ""
nav_body = nav_response.get("body") if isinstance(nav_response, dict) else {}
goal = nav_body.get("navigation_goal") if isinstance(nav_body, dict) else {}
goal_state = str((goal or {}).get("state", "")).lower()
goal_phase = str((goal or {}).get("phase", "")).lower()
active_goal = goal_state not in {"", "succeeded", "failed", "canceled", "cancelled", "aborted"} and goal_phase not in {
    "final_pose_verified",
    "nav2_failed",
    "final_pose_verify_failed",
}

status_body = status_response.get("body") if isinstance(status_response, dict) else {}
localization_candidates = []
if isinstance(status_response, dict) and isinstance(status_response.get("localization"), dict):
    localization_candidates.append(status_response["localization"])
if isinstance(status_body, dict) and isinstance(status_body.get("localization"), dict):
    localization_candidates.append(status_body["localization"])
safe_for_goal_start = status_response.get("safe_for_goal_start") if isinstance(status_response, dict) else None
if safe_for_goal_start is None and isinstance(status_body, dict):
    safe_for_goal_start = status_body.get("safe_for_goal_start")
for localization in localization_candidates:
    if localization.get("safe_for_goal_start") is not None:
        safe_for_goal_start = localization.get("safe_for_goal_start")
        break
pose_required = policy == "pose_required"
pose_ok = xy is not None and dyaw is not None and xy <= max_xy and dyaw <= max_yaw_rad
ok = (not active_goal and safe_for_goal_start is True and (pose_ok or not pose_required))
payload = {
    "time_utc": now_iso(),
    "api_url": api_url,
    "expected_start": expected,
    "target_a": target_a,
    "target_b": target_b,
    "runtime_building_id": building_id,
    "runtime_floor_id": floor_id,
    "poses_file": pose_file_used,
    "start_guard_policy": policy,
    "max_start_xy_m": max_xy,
    "max_start_yaw_deg": max_yaw_deg,
    "robot_pose": pose_response,
    "navigation_state": nav_response,
    "status": status_response,
    "distance_to_expected_xy_m": xy,
    "yaw_error_to_expected_rad": dyaw,
    "yaw_error_to_expected_deg": None if dyaw is None else math.degrees(dyaw),
    "nearest_delivery": nearest,
    "safe_for_goal_start": safe_for_goal_start,
    "active_navigation_goal": active_goal,
    "pose_required": pose_required,
    "pose_within_start_gate": pose_ok,
    "distances": distances,
    "decision": "continue" if ok else "stop_start_guard_failed",
}
(out_dir / "start_guard.json").write_text(json.dumps(payload, indent=2, sort_keys=True), encoding="utf-8")
def fmt6(value):
    return "" if value is None else f"{value:.6f}"

with (out_dir / "summary.md").open("w", encoding="utf-8") as f:
    f.write("# EKF A/B Start Guard\n\n")
    f.write("- read_only: `true`\n")
    f.write("- sends_navigation_goals: `false`\n")
    f.write(f"- time_utc: `{payload['time_utc']}`\n")
    f.write(f"- start_guard_policy: `{policy}`\n")
    f.write(f"- target_a: `delivery_{target_a}`\n")
    f.write(f"- target_b: `delivery_{target_b}`\n")
    f.write(f"- expected_start: `delivery_{expected}`\n")
    f.write(f"- poses_file: `{pose_file_used}`\n")
    f.write(f"- robot_pose_xy_yaw: `({x:.6f}, {y:.6f}, {yaw:.6f})`\n")
    f.write(f"- distance_to_expected_xy_m: `{fmt6(xy)}`\n")
    f.write(f"- yaw_error_to_expected_deg: `{fmt6(None if dyaw is None else math.degrees(dyaw))}`\n")
    f.write(f"- nearest_delivery: `{'' if not nearest else f'delivery_{nearest}'}`\n")
    f.write(f"- max_start_xy_m: `{max_xy}`\n")
    f.write(f"- max_start_yaw_deg: `{max_yaw_deg}`\n")
    f.write(f"- pose_required: `{pose_required}`\n")
    f.write(f"- pose_within_start_gate: `{pose_ok}`\n")
    f.write(f"- safe_for_goal_start: `{safe_for_goal_start}`\n")
    f.write(f"- active_navigation_goal: `{active_goal}`\n")
    f.write(f"- decision: `{payload['decision']}`\n")
    f.write("\n| delivery | xy_m | yaw_deg |\n")
    f.write("|---|---:|---:|\n")
    for name in target_ids:
        if name not in distances:
            f.write(f"| delivery_{name} | `missing` | `missing` |\n")
        else:
            f.write(
                f"| delivery_{name} | `{distances[name]['xy_m']:.6f}` | "
                f"`{distances[name]['dyaw_deg']:.6f}` |\n"
            )

raise SystemExit(0 if ok else 10)
PY
}

if [[ "${PREFLIGHT_ONLY}" == "true" ]]; then
  set +e
  run_start_guard "${OUT_DIR}/start_guard"
  preflight_rc=$?
  set -e
  {
    echo
    echo "## Start Guard"
    echo
    echo "- start_guard_rc: \`${preflight_rc}\`"
    echo "- start_guard_summary: \`${OUT_DIR}/start_guard/summary.md\`"
    echo "- preflight_only: \`true\`"
  } >>"${summary}"
  echo "[ekf-ab-guarded] preflight-only complete rc=${preflight_rc}; no EKF profile switch or navigation was executed"
  echo "[ekf-ab-guarded] summary: ${summary}"
  exit "${preflight_rc}"
fi

if [[ "${APPLY}" != "true" ]]; then
  {
    echo
    echo "## Dry Run Commands"
    echo
    echo '```bash'
    echo "# internal start guard: API_URL=${API_URL} policy=${START_GUARD_POLICY} expected_start=delivery_${EXPECTED_START} max_start_xy_m=${MAX_START_XY_M} max_start_yaw_deg=${MAX_START_YAW_DEG}"
    echo "# runtime context is resolved from API/runtime state before full systemd restart"
    echo "cat >${RUNTIME_OVERRIDE_ENV} <<'EOF'"
    echo "NJRH_NAV_LOCAL_STATE_MODE=ekf"
    echo "LOCAL_STATE_EKF_PROFILE=${PROFILE}"
    echo "NJRH_LOCAL_STATE_EKF_PROFILE=${PROFILE}"
    echo "EOF"
    echo "${FULL_RUNTIME_RESTART_CMD}"
    echo "bash ${SCRIPT_DIR}/check_commercial_runtime_ready.sh"
    echo "bash ${SCRIPT_DIR}/capture_relocalize_correction_compare.sh --output-dir ${OUT_DIR}/api_ready_relocalize_candidate_${PROFILE} --reason ekf_ab_candidate_${PROFILE}_api_ready_seed --timeout-sec ${RELOCALIZE_TIMEOUT_SEC} --settle-timeout-sec ${RELOCALIZE_SETTLE_TIMEOUT_SEC}"
    echo "# internal API gate: wait for /api/v1/status and /api/v1/navigation/state to report running with safe_for_goal_start=true"
    echo "bash ${SCRIPT_DIR}/capture_relocalize_correction_compare.sh --output-dir ${OUT_DIR}/pre_ab_relocalize --reason ekf_ab_${PROFILE}_pre_pingpong --timeout-sec ${RELOCALIZE_TIMEOUT_SEC} --settle-timeout-sec ${RELOCALIZE_SETTLE_TIMEOUT_SEC}"
    echo "bash ${SCRIPT_DIR}/run_navigation_delivery_pingpong_guarded.sh --target-a ${TARGET_A} --target-b ${TARGET_B} --start-target ${START_TARGET} --cycles ${CYCLES} --timeout-sec ${TIMEOUT_SEC} --goal-completion-policy ${GOAL_COMPLETION_POLICY} --max-online-xy-m ${MAX_ONLINE_XY_M} --max-online-yaw-rad ${MAX_ONLINE_YAW_RAD} --max-map-odom-translation-m ${MAX_MAP_ODOM_TRANSLATION_M} --max-map-base-translation-m ${MAX_MAP_BASE_TRANSLATION_M} --max-yaw-deg ${MAX_YAW_DEG} --output-root ${OUT_DIR}/pingpong"
    echo "cat >${RUNTIME_OVERRIDE_ENV} <<'EOF'"
    echo "NJRH_NAV_LOCAL_STATE_MODE=ekf"
    echo "LOCAL_STATE_EKF_PROFILE=${RESTORE_PROFILE}"
    echo "NJRH_LOCAL_STATE_EKF_PROFILE=${RESTORE_PROFILE}"
    echo "EOF"
    echo "${FULL_RUNTIME_RESTART_CMD}"
    echo "bash ${SCRIPT_DIR}/check_commercial_runtime_ready.sh"
    echo "bash ${SCRIPT_DIR}/capture_relocalize_correction_compare.sh --output-dir ${OUT_DIR}/api_ready_relocalize_restore_${RESTORE_PROFILE} --reason ekf_ab_restore_${RESTORE_PROFILE}_api_ready_seed --timeout-sec ${RELOCALIZE_TIMEOUT_SEC} --settle-timeout-sec ${RELOCALIZE_SETTLE_TIMEOUT_SEC}"
    echo "# internal API gate: wait for /api/v1/status and /api/v1/navigation/state to report running with safe_for_goal_start=true"
    echo "bash ${SCRIPT_DIR}/capture_relocalize_correction_compare.sh --output-dir ${OUT_DIR}/restore_relocalize --reason ekf_ab_restore_${RESTORE_PROFILE} --timeout-sec ${RELOCALIZE_TIMEOUT_SEC} --settle-timeout-sec ${RELOCALIZE_SETTLE_TIMEOUT_SEC}"
    echo '```'
  } >>"${summary}"
  echo "[ekf-ab-guarded] dry-run only; rerun with --apply to execute"
  echo "[ekf-ab-guarded] summary: ${summary}"
  exit 0
fi

resolve_runtime_floor_context() {
  local output_dir="$1"
  mkdir -p "${output_dir}"
  python3 - "${output_dir}" "${API_URL}" <<'PY'
import json
import os
import re
import sys
import urllib.request
from pathlib import Path

out_dir = Path(sys.argv[1])
api_url = sys.argv[2].rstrip("/")
context_file = Path(os.environ.get("NJRH_RUNTIME_MAP_CONTEXT_FILE", "/tmp/njrh_runtime_map_context.json"))
safe_id = re.compile(r"^[A-Za-z0-9_.-]+$")


def valid_pair(building_id, floor_id):
    building_id = str(building_id or "")
    floor_id = str(floor_id or "")
    if safe_id.fullmatch(building_id) and safe_id.fullmatch(floor_id):
        return building_id, floor_id
    return None


def api_get(path):
    try:
        with urllib.request.urlopen(api_url + path, timeout=3.0) as response:
            return json.loads(response.read().decode("utf-8", errors="replace"))
    except Exception:
        return None


def find_runtime_context(data):
    if not isinstance(data, dict):
        return None
    candidates = [
        data.get("runtime_map_context"),
        data.get("map_context"),
    ]
    navigation = data.get("navigation")
    if isinstance(navigation, dict):
        candidates.extend([
            navigation.get("runtime_map_context"),
            navigation.get("map_context"),
        ])
    body = data.get("body")
    if isinstance(body, dict):
        candidates.extend([
            body.get("runtime_map_context"),
            body.get("map_context"),
        ])
        navigation = body.get("navigation")
        if isinstance(navigation, dict):
            candidates.extend([
                navigation.get("runtime_map_context"),
                navigation.get("map_context"),
            ])
    for candidate in candidates:
        if not isinstance(candidate, dict):
            continue
        pair = valid_pair(candidate.get("building_id"), candidate.get("floor_id"))
        if pair:
            return {"source": "api", "building_id": pair[0], "floor_id": pair[1], "context": candidate}
    return None


result = None
env_pair = valid_pair(os.environ.get("NJRH_BUILDING_ID"), os.environ.get("NJRH_FLOOR_ID"))
if env_pair:
    result = {"source": "environment", "building_id": env_pair[0], "floor_id": env_pair[1], "context": {}}

if result is None and context_file.exists():
    try:
        data = json.loads(context_file.read_text(encoding="utf-8"))
    except Exception:
        data = {}
    pair = valid_pair(data.get("building_id"), data.get("floor_id"))
    if pair:
        result = {"source": str(context_file), "building_id": pair[0], "floor_id": pair[1], "context": data}

if result is None:
    for path in ("/api/v1/navigation/state", "/api/v1/status"):
        found = find_runtime_context(api_get(path))
        if found:
            found["source"] = path
            result = found
            break

if result is None:
    (out_dir / "runtime_floor_context.json").write_text(
        json.dumps({"ok": False, "api_url": api_url, "context_file": str(context_file)}, indent=2),
        encoding="utf-8",
    )
    raise SystemExit("could not resolve runtime building_id/floor_id from environment, runtime context, or API")

payload = {"ok": True, "api_url": api_url, "context_file": str(context_file), **result}
(out_dir / "runtime_floor_context.json").write_text(json.dumps(payload, indent=2, sort_keys=True), encoding="utf-8")
print(f"{result['building_id']}\t{result['floor_id']}")
PY
}

wait_for_navigation_runtime_ready() {
  local profile="$1"
  local label="$2"
  local timeout_sec="$3"
  local ready_log="${OUT_DIR}/commercial_runtime_ready_${label}_${profile}.log"
  local deadline=$((SECONDS + timeout_sec))
  while (( SECONDS < deadline )); do
    if bash "${SCRIPT_DIR}/check_commercial_runtime_ready.sh" >"${ready_log}.tmp" 2>&1; then
      mv "${ready_log}.tmp" "${ready_log}"
      return 0
    fi
    sleep 2
  done
  bash "${SCRIPT_DIR}/check_commercial_runtime_ready.sh" >"${ready_log}" 2>&1 || true
  rm -f "${ready_log}.tmp" 2>/dev/null || true
  return 1
}

wait_for_api_navigation_ready() {
  local profile="$1"
  local label="$2"
  local timeout_sec="$3"
  local ready_json="${OUT_DIR}/api_navigation_ready_${label}_${profile}.json"
  local deadline=$((SECONDS + timeout_sec))
  while (( SECONDS < deadline )); do
    if python3 - "${API_URL}" >"${ready_json}.tmp" <<'PY'
import json
import sys
import urllib.request

api_url = sys.argv[1].rstrip("/")

def api_get(path):
    with urllib.request.urlopen(api_url + path, timeout=3.0) as response:
        return json.loads(response.read().decode("utf-8", errors="replace"))

status = api_get("/api/v1/status")
navigation_state = api_get("/api/v1/navigation/state")
localization = status.get("localization") if isinstance(status, dict) else {}
navigation = status.get("navigation") if isinstance(status, dict) else {}
payload = {
    "ok": bool(status.get("ok")) if isinstance(status, dict) else False,
    "healthy": bool(status.get("healthy")) if isinstance(status, dict) else False,
    "top_state": status.get("state") if isinstance(status, dict) else None,
    "navigation_active": navigation.get("active") if isinstance(navigation, dict) else None,
    "navigation_state": navigation.get("state") if isinstance(navigation, dict) else None,
    "navigation_endpoint_state": navigation_state.get("state") if isinstance(navigation_state, dict) else None,
    "safe_for_goal_start": localization.get("safe_for_goal_start") if isinstance(localization, dict) else None,
}
ready = (
    payload["ok"] is True
    and payload["healthy"] is True
    and payload["top_state"] == "running"
    and payload["navigation_active"] is True
    and payload["navigation_state"] == "running"
    and payload["navigation_endpoint_state"] == "running"
    and payload["safe_for_goal_start"] is True
)
payload["ready"] = ready
print(json.dumps(payload, indent=2, sort_keys=True))
raise SystemExit(0 if ready else 1)
PY
    then
      mv "${ready_json}.tmp" "${ready_json}"
      return 0
    fi
    sleep 2
  done
  mv "${ready_json}.tmp" "${ready_json}" 2>/dev/null || true
  return 1
}

run_api_ready_relocalize() {
  local profile="$1"
  local label="$2"
  local output_dir="${OUT_DIR}/api_ready_relocalize_${label}_${profile}"
  local rc=0

  echo "[ekf-ab-guarded] API readiness relocalize/seed profile=${profile} label=${label}"
  bash "${SCRIPT_DIR}/capture_relocalize_correction_compare.sh" \
    --output-dir "${output_dir}" \
    --reason "ekf_ab_${label}_${profile}_api_ready_seed" \
    --timeout-sec "${RELOCALIZE_TIMEOUT_SEC}" \
    --settle-timeout-sec "${RELOCALIZE_SETTLE_TIMEOUT_SEC}" || rc=$?
  {
    echo
    echo "## API Ready Relocalize ${label}"
    echo
    echo "- profile: \`${profile}\`"
    echo "- relocalize_rc: \`${rc}\`"
    echo "- relocalize_summary: \`${output_dir}/summary.md\`"
  } >>"${summary}"
  return "${rc}"
}

write_runtime_override_profile() {
  local profile="$1"
  local tmp_file="${RUNTIME_OVERRIDE_ENV}.$$"
  mkdir -p "$(dirname "${RUNTIME_OVERRIDE_ENV}")"
  cat >"${tmp_file}" <<EOF
NJRH_NAV_LOCAL_STATE_MODE=ekf
LOCAL_STATE_EKF_PROFILE=${profile}
NJRH_LOCAL_STATE_EKF_PROFILE=${profile}
EOF
  chmod 0644 "${tmp_file}" 2>/dev/null || true
  mv "${tmp_file}" "${RUNTIME_OVERRIDE_ENV}"
  echo "[ekf-ab-guarded] wrote runtime override ${RUNTIME_OVERRIDE_ENV} profile=${profile}"
}

restart_full_runtime_owner() {
  echo "[ekf-ab-guarded] restart command: ${FULL_RUNTIME_RESTART_CMD}"
  bash -lc "${FULL_RUNTIME_RESTART_CMD}"
}

restart_navigation_runtime_with_profile() {
  local profile="$1"
  local label="$2"
  local context_dir="${OUT_DIR}/runtime_context_${label}_${profile}"
  local context_line=""
  local building_id=""
  local floor_id=""

  context_line="$(resolve_runtime_floor_context "${context_dir}")" || return 1
  IFS=$'\t' read -r building_id floor_id <<<"${context_line}"
  [[ -n "${building_id}" && -n "${floor_id}" ]] || {
    echo "[ekf-ab-guarded] resolved runtime context is incomplete: ${context_line}" >&2
    return 1
  }

  echo "[ekf-ab-guarded] restarting product runtime owner profile=${profile} floor=${building_id}/${floor_id}"
  write_runtime_override_profile "${profile}"
  if ! restart_full_runtime_owner; then
    echo "[ekf-ab-guarded] product runtime restart failed; run from the host or set NJRH_FULL_RUNTIME_RESTART_CMD" >&2
    return 1
  fi
  if ! wait_for_navigation_runtime_ready "${profile}" "${label}" "${NAV_RESTART_READY_TIMEOUT_SEC}"; then
    echo "[ekf-ab-guarded] navigation runtime profile=${profile} did not become ready" >&2
    return 1
  fi
  if ! run_api_ready_relocalize "${profile}" "${label}"; then
    echo "[ekf-ab-guarded] API readiness relocalize/seed profile=${profile} label=${label} failed; continuing to API ready gate" >&2
  fi
  if ! wait_for_api_navigation_ready "${profile}" "${label}" "${NAV_RESTART_READY_TIMEOUT_SEC}"; then
    echo "[ekf-ab-guarded] API navigation context profile=${profile} did not become ready" >&2
    return 1
  fi
}

restore_rc=0
restore_stable_profile() {
  trap - EXIT
  echo "[ekf-ab-guarded] restoring stable EKF profile=${RESTORE_PROFILE} through full navigation runtime restart" >&2
  if ! restart_navigation_runtime_with_profile "${RESTORE_PROFILE}" "restore"; then
    restore_rc=1
  fi
  if ! bash "${SCRIPT_DIR}/capture_relocalize_correction_compare.sh" \
      --output-dir "${OUT_DIR}/restore_relocalize" \
      --reason "ekf_ab_restore_${RESTORE_PROFILE}" \
      --timeout-sec "${RELOCALIZE_TIMEOUT_SEC}" \
      --settle-timeout-sec "${RELOCALIZE_SETTLE_TIMEOUT_SEC}"; then
    restore_rc=1
  fi
  {
    echo
    echo "## Restore"
    echo
    echo "- restore_profile: \`${RESTORE_PROFILE}\`"
    echo "- restore_rc: \`${restore_rc}\`"
    echo "- restore_relocalize: \`${OUT_DIR}/restore_relocalize/summary.md\`"
  } >>"${summary}"
  return "${restore_rc}"
}

set +e
run_start_guard "${OUT_DIR}/start_guard"
start_guard_rc=$?
set -e
if [[ "${start_guard_rc}" != "0" ]]; then
  {
    echo
    echo "## Start Guard"
    echo
    echo "- start_guard_rc: \`${start_guard_rc}\`"
    echo "- start_guard_summary: \`${OUT_DIR}/start_guard/summary.md\`"
  } >>"${summary}"
  echo "[ekf-ab-guarded] start guard failed; no EKF profile switch or navigation was executed" >&2
  exit "${start_guard_rc}"
fi
{
  echo
  echo "## Start Guard"
  echo
  echo "- start_guard_rc: \`0\`"
  echo "- start_guard_summary: \`${OUT_DIR}/start_guard/summary.md\`"
} >>"${summary}"

trap restore_stable_profile EXIT
restart_navigation_runtime_with_profile "${PROFILE}" "candidate"

echo "[ekf-ab-guarded] pre-A/B relocalize"
bash "${SCRIPT_DIR}/capture_relocalize_correction_compare.sh" \
  --output-dir "${OUT_DIR}/pre_ab_relocalize" \
  --reason "ekf_ab_${PROFILE}_pre_pingpong" \
  --timeout-sec "${RELOCALIZE_TIMEOUT_SEC}" \
  --settle-timeout-sec "${RELOCALIZE_SETTLE_TIMEOUT_SEC}"

echo "[ekf-ab-guarded] running guarded ping-pong profile=${PROFILE}"
set +e
bash "${SCRIPT_DIR}/run_navigation_delivery_pingpong_guarded.sh" \
  --target-a "${TARGET_A}" \
  --target-b "${TARGET_B}" \
  --start-target "${START_TARGET}" \
  --cycles "${CYCLES}" \
  --timeout-sec "${TIMEOUT_SEC}" \
  --goal-completion-policy "${GOAL_COMPLETION_POLICY}" \
  --max-online-xy-m "${MAX_ONLINE_XY_M}" \
  --max-online-yaw-rad "${MAX_ONLINE_YAW_RAD}" \
  --max-map-odom-translation-m "${MAX_MAP_ODOM_TRANSLATION_M}" \
  --max-map-base-translation-m "${MAX_MAP_BASE_TRANSLATION_M}" \
  --max-yaw-deg "${MAX_YAW_DEG}" \
  --output-root "${OUT_DIR}/pingpong"
pingpong_rc=$?
set -e

{
  echo
  echo "## Result"
  echo
  echo "- candidate_profile: \`${PROFILE}\`"
  echo "- pre_ab_relocalize: \`${OUT_DIR}/pre_ab_relocalize/summary.md\`"
  echo "- pingpong_rc: \`${pingpong_rc}\`"
  echo "- pingpong_output_root: \`${OUT_DIR}/pingpong\`"
} >>"${summary}"

restore_stable_profile || true
trap - EXIT

echo "[ekf-ab-guarded] summary: ${summary}"
if [[ "${restore_rc}" != "0" ]]; then
  echo "[ekf-ab-guarded] restore failed; inspect ${OUT_DIR}" >&2
  exit 30
fi
exit "${pingpong_rc}"
