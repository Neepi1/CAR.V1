#!/usr/bin/env bash
set -euo pipefail
umask 0002

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OVERLAY_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
PROJECT_ROOT="$(cd "${OVERLAY_ROOT}/../../.." && pwd)"

API_URL="${API_URL:-http://127.0.0.1:8080}"
GOAL_JSON=""
POSE_ID=""
BUILDING_ID=""
FLOOR_ID=""
GOAL_X=""
GOAL_Y=""
GOAL_YAW=""
GOAL_COMPLETION_POLICY="pose_required"
TIMEOUT_SEC="180"
POLL_PERIOD_SEC="1.0"
SETTLE_SEC="3.0"
LABEL="nav_pose_error"
OUTPUT_ROOT="${PROJECT_ROOT}/reports/navigation_pose_error_test"
PRE_RELOCALIZE="false"
POST_RELOCALIZE="true"

usage() {
  cat <<'EOF'
Usage: run_navigation_pose_error_test.sh [goal options] [options]

Runs one normal point-navigation goal through robot_api_server and records the
API/Nav2 final pose audit plus an explicit post-goal relocalization correction.
It does not publish velocity commands and does not change parameters.

Goal options, choose one:
  --pose-id ID [--building-id ID --floor-id ID]
  --x M --y M --yaw RAD
  --goal-json JSON

Options:
  --goal-completion-policy pose_required|position_only
                            Default: pose_required.
  --timeout-sec SEC         Maximum wait for navigation terminal state. Default: 180.
  --poll-period-sec SEC     API poll period. Default: 1.0.
  --settle-sec SEC          Extra wait after terminal state before final snapshot. Default: 3.0.
  --label NAME              Report label. Default: nav_pose_error.
  --api-url URL             robot_api_server URL. Default: http://127.0.0.1:8080.
  --output-root DIR         Report root. Default: reports/navigation_pose_error_test.
  --pre-relocalize          Run an explicit relocalization before sending the goal.
  --no-pre-relocalize       Do not run an explicit relocalization before sending the goal. Default.
  --no-post-relocalize      Do not run the post-goal relocalization correction capture.

For pose_id navigation, building_id/floor_id are auto-filled from the current
runtime map context when omitted. The default intentionally avoids pre-goal
relocalization, matching repeated point-to-point field runs.
EOF
}

while [[ "$#" -gt 0 ]]; do
  case "$1" in
    --goal-json)
      GOAL_JSON="${2:-}"
      shift 2
      ;;
    --pose-id)
      POSE_ID="${2:-}"
      shift 2
      ;;
    --building-id)
      BUILDING_ID="${2:-}"
      shift 2
      ;;
    --floor-id)
      FLOOR_ID="${2:-}"
      shift 2
      ;;
    --x)
      GOAL_X="${2:-}"
      shift 2
      ;;
    --y)
      GOAL_Y="${2:-}"
      shift 2
      ;;
    --yaw|--theta)
      GOAL_YAW="${2:-}"
      shift 2
      ;;
    --goal-completion-policy)
      GOAL_COMPLETION_POLICY="${2:-}"
      shift 2
      ;;
    --timeout-sec)
      TIMEOUT_SEC="${2:-}"
      shift 2
      ;;
    --poll-period-sec)
      POLL_PERIOD_SEC="${2:-}"
      shift 2
      ;;
    --settle-sec)
      SETTLE_SEC="${2:-}"
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
    --output-root)
      OUTPUT_ROOT="${2:-}"
      shift 2
      ;;
    --pre-relocalize)
      PRE_RELOCALIZE="true"
      shift
      ;;
    --no-pre-relocalize)
      PRE_RELOCALIZE="false"
      shift
      ;;
    --no-post-relocalize)
      POST_RELOCALIZE="false"
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "[nav-pose-error] unknown argument: $1" >&2
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
OUTPUT_ROOT="${OUTPUT_ROOT/#\~/${HOME}}"

safe_label="$(printf '%s' "${LABEL}" | tr -c 'A-Za-z0-9_.-' '_')"
timestamp="$(date -u +%Y%m%dT%H%M%SZ)"
OUT_DIR="${OUTPUT_ROOT}/${timestamp}_${safe_label}"
mkdir -p "${OUT_DIR}"

python3 - \
  "${OUT_DIR}/goal_request.json" \
  "${GOAL_JSON}" \
  "${POSE_ID}" \
  "${BUILDING_ID}" \
  "${FLOOR_ID}" \
  "${GOAL_X}" \
  "${GOAL_Y}" \
  "${GOAL_YAW}" \
  "${GOAL_COMPLETION_POLICY}" \
  "${API_URL}" <<'PY'
import json
import math
import sys
import urllib.request
from pathlib import Path

path = Path(sys.argv[1])
goal_json = sys.argv[2]
pose_id = sys.argv[3]
building_id = sys.argv[4]
floor_id = sys.argv[5]
x_text = sys.argv[6]
y_text = sys.argv[7]
yaw_text = sys.argv[8]
policy = sys.argv[9]
api_url = sys.argv[10].rstrip("/")

if policy not in ("pose_required", "position_only"):
    raise SystemExit("goal completion policy must be pose_required or position_only")


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
        b = str(candidate.get("building_id") or "")
        f = str(candidate.get("floor_id") or "")
        if b and f:
            return b, f
    return None


def resolve_context():
    for path_name in ("/api/v1/navigation/state", "/api/v1/status"):
        found = find_runtime_context(api_get(path_name))
        if found:
            return found
    return None

if goal_json:
    body = json.loads(goal_json)
elif pose_id:
    if not building_id or not floor_id:
        resolved = resolve_context()
        if resolved:
            building_id, floor_id = resolved
        else:
            raise SystemExit("--pose-id requires --building-id and --floor-id when runtime context cannot be resolved")
    body = {
        "pose_id": pose_id,
        "building_id": building_id,
        "floor_id": floor_id,
    }
else:
    if not x_text or not y_text or not yaw_text:
        raise SystemExit("provide --goal-json, --pose-id, or --x/--y/--yaw")
    x = float(x_text)
    y = float(y_text)
    yaw = float(yaw_text)
    if not all(math.isfinite(v) for v in (x, y, yaw)):
        raise SystemExit("x/y/yaw must be finite")
    body = {
        "x": x,
        "y": y,
        "yaw": yaw,
        "frame_id": "map",
    }

body.setdefault("goal_completion_policy", policy)
path.write_text(json.dumps(body, ensure_ascii=False, indent=2, sort_keys=True) + "\n", encoding="utf-8")
PY

{
  echo "# Navigation Pose Error Test Environment"
  echo "- timestamp_utc: ${timestamp}"
  echo "- api_url: ${API_URL}"
  echo "- timeout_sec: ${TIMEOUT_SEC}"
  echo "- poll_period_sec: ${POLL_PERIOD_SEC}"
  echo "- settle_sec: ${SETTLE_SEC}"
  echo "- pre_relocalize: ${PRE_RELOCALIZE}"
  echo "- post_relocalize: ${POST_RELOCALIZE}"
  echo "- label: ${LABEL}"
  echo "- rmw: ${RMW_IMPLEMENTATION:-}"
  echo
  echo "## Goal Request"
  cat "${OUT_DIR}/goal_request.json"
  echo
  echo "## API Before"
  curl -fsS --max-time 3 "${API_URL}/api/v1/status" 2>&1 || true
  echo
  curl -fsS --max-time 3 "${API_URL}/api/v1/navigation/state" 2>&1 || true
  echo
  curl -fsS --max-time 3 "${API_URL}/api/v1/robot/pose" 2>&1 || true
  echo
  echo "## ROS Topic Info"
  for topic in \
    /cmd_vel_nav_raw \
    /cmd_vel_nav \
    /cmd_vel_collision_checked \
    /cmd_vel_safe \
    /cmd_vel \
    /wheel/odom \
    /local_state/odometry \
    /localization/bridge_status \
    /amcl_pose \
    /navigate_to_pose/_action/status; do
    echo "### ${topic}"
    ros2 topic info "${topic}" -v 2>&1 || true
  done
} >"${OUT_DIR}/environment.md"

echo "[nav-pose-error] report: ${OUT_DIR}"

if [[ "${PRE_RELOCALIZE}" == "true" ]]; then
  echo "[nav-pose-error] pre-relocalize capture..."
  bash "${SCRIPT_DIR}/capture_relocalize_correction_compare.sh" \
    --output-dir "${OUT_DIR}/pre_relocalize_compare" \
    --reason "nav_pose_error_pre_relocalize"
fi

set +e
python3 - \
  "${OUT_DIR}" \
  "${API_URL}" \
  "${TIMEOUT_SEC}" \
  "${POLL_PERIOD_SEC}" \
  "${SETTLE_SEC}" <<'PY'
import csv
import json
import math
import sys
import time
import urllib.error
import urllib.request
from pathlib import Path
from typing import Any, Dict, Optional, Tuple

out_dir = Path(sys.argv[1])
api_url = sys.argv[2].rstrip("/")
timeout_sec = float(sys.argv[3])
poll_period_sec = max(float(sys.argv[4]), 0.2)
settle_sec = max(float(sys.argv[5]), 0.0)

goal_request = json.loads((out_dir / "goal_request.json").read_text(encoding="utf-8"))


def now_iso() -> str:
    return time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())


def api_json(method: str, path: str, body: Optional[Dict[str, Any]] = None, timeout: float = 3.0) -> Dict[str, Any]:
    data = None
    headers = {}
    if body is not None:
        data = json.dumps(body).encode("utf-8")
        headers["Content-Type"] = "application/json"
    req = urllib.request.Request(api_url + path, data=data, method=method, headers=headers)
    try:
        with urllib.request.urlopen(req, timeout=timeout) as resp:
            text = resp.read().decode("utf-8", errors="replace")
            try:
                payload = json.loads(text) if text.strip() else None
            except Exception:
                payload = None
            return {
                "ok": 200 <= resp.status < 300,
                "status": resp.status,
                "body": payload,
                "text": text,
                "error": "",
            }
    except urllib.error.HTTPError as exc:
        text = exc.read().decode("utf-8", errors="replace")
        try:
            payload = json.loads(text) if text.strip() else None
        except Exception:
            payload = None
        return {
            "ok": False,
            "status": exc.code,
            "body": payload,
            "text": text,
            "error": str(exc),
        }
    except Exception as exc:
        return {
            "ok": False,
            "status": 0,
            "body": None,
            "text": "",
            "error": repr(exc),
        }


def write_json(name: str, payload: Any) -> None:
    (out_dir / name).write_text(
        json.dumps(payload, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )


def nested(data: Any, *keys: str) -> Any:
    cur = data
    for key in keys:
        if not isinstance(cur, dict):
            return None
        cur = cur.get(key)
    return cur


def num(value: Any) -> Optional[float]:
    try:
        value = float(value)
    except Exception:
        return None
    return value if math.isfinite(value) else None


def pose_xy_yaw(robot_pose: Any) -> Tuple[Optional[float], Optional[float], Optional[float]]:
    body = robot_pose.get("body") if isinstance(robot_pose, dict) else None
    candidates = []
    if isinstance(body, dict):
        candidates.extend([
            body,
            body.get("pose"),
            body.get("map_pose"),
            body.get("robot_pose"),
        ])
    for candidate in candidates:
        if not isinstance(candidate, dict):
            continue
        x = num(candidate.get("x"))
        y = num(candidate.get("y"))
        yaw = num(candidate.get("yaw", candidate.get("theta")))
        if x is not None and y is not None and yaw is not None:
            return x, y, yaw
    return None, None, None


def terminal_for_goal(nav_state: Dict[str, Any], expected_goal_id: Optional[int]) -> Tuple[bool, Dict[str, Any]]:
    body = nav_state.get("body")
    goal = {}
    if isinstance(body, dict):
        goal = body.get("navigation_goal") or body.get("goal") or {}
    if not isinstance(goal, dict):
        goal = {}
    if expected_goal_id is not None and goal.get("id") != expected_goal_id:
        return False, goal
    state = str(goal.get("state", "")).lower()
    phase = str(goal.get("phase", "")).lower()
    terminal_states = {"succeeded", "failed", "canceled", "cancelled", "aborted", "complete", "completed"}
    terminal_phases = {
        "final_pose_verified",
        "failed",
        "canceled",
        "cancelled",
        "aborted",
        "nav2_failed",
        "nav2_canceled",
        "final_pose_verify_failed",
    }
    if state in terminal_states or phase in terminal_phases:
        return True, goal
    if goal.get("task_complete") is True:
        return True, goal
    return False, goal


before = {
    "time_utc": now_iso(),
    "status": api_json("GET", "/api/v1/status"),
    "navigation_state": api_json("GET", "/api/v1/navigation/state"),
    "robot_pose": api_json("GET", "/api/v1/robot/pose"),
}
write_json("api_before.json", before)

post = api_json("POST", "/api/v1/navigation/goal", goal_request, timeout=8.0)
write_json("post_goal_response.json", post)

post_body = post.get("body") if isinstance(post.get("body"), dict) else {}
expected_goal_id = post_body.get("navigation_goal_id")
if not post.get("ok") or post_body.get("accepted") is False:
    with (out_dir / "summary.md").open("w", encoding="utf-8") as f:
        f.write("# Navigation Pose Error Test Summary\n\n")
        f.write("- result: `goal_post_failed`\n")
        f.write(f"- http_status: `{post.get('status')}`\n")
        f.write(f"- error: `{post.get('error')}`\n")
        if isinstance(post_body, dict):
            f.write(f"- response_error: `{post_body.get('error', '')}`\n")
    raise SystemExit(20)

samples_path = out_dir / "api_pose_poll.csv"
jsonl_path = out_dir / "api_pose_poll.jsonl"
fieldnames = [
    "elapsed_sec",
    "nav_http_status",
    "goal_id",
    "goal_state",
    "goal_phase",
    "task_complete",
    "target_x",
    "target_y",
    "target_yaw",
    "final_distance_m",
    "final_yaw_error_rad",
    "final_verify_xy_error_m",
    "final_verify_yaw_error_rad",
    "position_reached",
    "nav2_succeeded",
    "nav2_result_code",
    "robot_pose_x",
    "robot_pose_y",
    "robot_pose_yaw",
    "robot_pose_http_status",
    "bridge_safe_for_goal_start",
    "bridge_correction_active",
    "bridge_remaining_translation_error_m",
    "bridge_remaining_yaw_error_rad",
]

terminal = False
terminal_goal: Dict[str, Any] = {}
start = time.monotonic()
deadline = start + timeout_sec

with samples_path.open("w", newline="", encoding="utf-8") as csv_file, jsonl_path.open("w", encoding="utf-8") as jsonl:
    writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
    writer.writeheader()
    while time.monotonic() < deadline:
        elapsed = time.monotonic() - start
        nav = api_json("GET", "/api/v1/navigation/state", timeout=2.0)
        pose = api_json("GET", "/api/v1/robot/pose", timeout=2.0)
        body = nav.get("body") if isinstance(nav.get("body"), dict) else {}
        goal = body.get("navigation_goal") if isinstance(body, dict) else {}
        if not isinstance(goal, dict):
            goal = {}
        bridge = body.get("bridge") if isinstance(body, dict) else {}
        if not isinstance(bridge, dict):
            bridge = body.get("localization") if isinstance(body, dict) else {}
        if not isinstance(bridge, dict):
            bridge = {}
        rx, ry, ryaw = pose_xy_yaw(pose)
        target = goal.get("target") if isinstance(goal.get("target"), dict) else {}
        row = {
            "elapsed_sec": f"{elapsed:.3f}",
            "nav_http_status": nav.get("status"),
            "goal_id": goal.get("id", ""),
            "goal_state": goal.get("state", ""),
            "goal_phase": goal.get("phase", ""),
            "task_complete": goal.get("task_complete", ""),
            "target_x": target.get("x", ""),
            "target_y": target.get("y", ""),
            "target_yaw": target.get("yaw", ""),
            "final_distance_m": goal.get("final_distance_m", ""),
            "final_yaw_error_rad": goal.get("final_yaw_error_rad", ""),
            "final_verify_xy_error_m": goal.get("final_verify_xy_error_m", ""),
            "final_verify_yaw_error_rad": goal.get("final_verify_yaw_error_rad", ""),
            "position_reached": goal.get("position_reached", ""),
            "nav2_succeeded": goal.get("nav2_succeeded", ""),
            "nav2_result_code": goal.get("nav2_result_code", ""),
            "robot_pose_x": "" if rx is None else f"{rx:.6f}",
            "robot_pose_y": "" if ry is None else f"{ry:.6f}",
            "robot_pose_yaw": "" if ryaw is None else f"{ryaw:.6f}",
            "robot_pose_http_status": pose.get("status"),
            "bridge_safe_for_goal_start": bridge.get("safe_for_goal_start", ""),
            "bridge_correction_active": bridge.get("correction_active", ""),
            "bridge_remaining_translation_error_m": bridge.get("remaining_translation_error_m", ""),
            "bridge_remaining_yaw_error_rad": bridge.get("remaining_yaw_error_rad", ""),
        }
        writer.writerow(row)
        jsonl.write(json.dumps({
            "time_utc": now_iso(),
            "elapsed_sec": elapsed,
            "navigation_state": nav,
            "robot_pose": pose,
        }, ensure_ascii=False, sort_keys=True) + "\n")
        jsonl.flush()

        terminal, terminal_goal = terminal_for_goal(nav, expected_goal_id)
        if terminal:
            time.sleep(settle_sec)
            break
        time.sleep(poll_period_sec)

after = {
    "time_utc": now_iso(),
    "status": api_json("GET", "/api/v1/status"),
    "navigation_state": api_json("GET", "/api/v1/navigation/state"),
    "robot_pose": api_json("GET", "/api/v1/robot/pose"),
}
write_json("api_after.json", after)

final_nav = after.get("navigation_state", {})
_, final_goal = terminal_for_goal(final_nav, expected_goal_id)
if not final_goal and terminal_goal:
    final_goal = terminal_goal
target = final_goal.get("target") if isinstance(final_goal.get("target"), dict) else {}
before_pose = pose_xy_yaw(before.get("robot_pose", {}))
after_pose = pose_xy_yaw(after.get("robot_pose", {}))

with (out_dir / "summary.md").open("w", encoding="utf-8") as f:
    f.write("# Navigation Pose Error Test Summary\n\n")
    f.write(f"- result: `{'terminal' if terminal else 'timeout'}`\n")
    f.write(f"- api_url: `{api_url}`\n")
    f.write(f"- navigation_goal_id: `{expected_goal_id}`\n")
    f.write(f"- posted_goal: `{json.dumps(goal_request, ensure_ascii=False, sort_keys=True)}`\n")
    f.write(f"- accepted_goal_response_goal: `{json.dumps(post_body.get('goal', {}), ensure_ascii=False, sort_keys=True)}`\n")
    f.write("\n## API Final Goal\n\n")
    for key in (
        "id",
        "state",
        "phase",
        "detail",
        "pose_id",
        "building_id",
        "floor_id",
        "goal_completion_policy",
        "nav2_succeeded",
        "nav2_result_code",
        "position_reached",
        "final_distance_m",
        "final_yaw_error_rad",
        "final_verify_xy_error_m",
        "final_verify_yaw_error_rad",
        "final_pose_verified",
        "final_pose_verify_reason",
        "task_complete",
        "final_yaw_align_requested",
        "final_yaw_align_attempted",
        "final_yaw_align_succeeded",
        "final_yaw_align_blocked",
    ):
        if key in final_goal:
            f.write(f"- {key}: `{final_goal.get(key)}`\n")
    if target:
        f.write(f"- target: `{json.dumps(target, ensure_ascii=False, sort_keys=True)}`\n")
    f.write("\n## API Robot Pose\n\n")
    f.write(f"- before_map_pose_xy_yaw: `{before_pose}`\n")
    f.write(f"- after_map_pose_xy_yaw: `{after_pose}`\n")
    f.write("\n## Files\n\n")
    f.write("- `api_pose_poll.csv`: polled navigation state and robot pose\n")
    f.write("- `api_pose_poll.jsonl`: full API poll payloads\n")
    f.write("- `post_goal_response.json`: raw goal response\n")
    f.write("- `api_before.json`, `api_after.json`: API snapshots\n")

raise SystemExit(0 if terminal else 10)
PY
nav_rc=$?
set -e

if [[ "${POST_RELOCALIZE}" == "true" && "${nav_rc}" == "0" ]]; then
  echo "[nav-pose-error] post-goal relocalize capture..."
  set +e
  bash "${SCRIPT_DIR}/capture_relocalize_correction_compare.sh" \
    --output-dir "${OUT_DIR}/post_relocalize_compare" \
    --reason "nav_pose_error_after_goal"
  relocalize_rc=$?
  set -e
  python3 - "${OUT_DIR}" "${relocalize_rc}" <<'PY'
import json
import sys
from pathlib import Path

out = Path(sys.argv[1])
rc = sys.argv[2]
summary = out / "summary.md"
metrics_path = out / "post_relocalize_compare" / "correction_metrics.json"
with summary.open("a", encoding="utf-8") as f:
    f.write("\n## Post-Goal Relocalize Correction\n\n")
    f.write(f"- relocalize_exit_code: `{rc}`\n")
    if metrics_path.exists():
        metrics = json.loads(metrics_path.read_text(encoding="utf-8"))
        for key in ("map_base_link_delta", "map_odom_delta"):
            value = metrics.get(key)
            if isinstance(value, dict):
                f.write(
                    f"- {key}: translation_m=`{value.get('translation_m')}`, "
                    f"dyaw_deg=`{value.get('dyaw_deg')}`, "
                    f"forward_m=`{value.get('forward_m_in_before_frame')}`, "
                    f"left_m=`{value.get('left_m_in_before_frame')}`\n"
                )
        bridge = metrics.get("bridge") or {}
        f.write(
            f"- bridge_last_correction_delta_translation_m: "
            f"`{bridge.get('last_correction_delta_translation_m')}`\n"
        )
        f.write(
            f"- bridge_last_correction_delta_yaw_rad: "
            f"`{bridge.get('last_correction_delta_yaw_rad')}`\n"
        )
    else:
        f.write("- metrics: `missing`\n")
PY
else
  echo "[nav-pose-error] post-goal relocalize skipped nav_rc=${nav_rc} post_relocalize=${POST_RELOCALIZE}"
fi

echo "[nav-pose-error] summary: ${OUT_DIR}/summary.md"
exit "${nav_rc}"
