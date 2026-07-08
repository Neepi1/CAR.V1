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
GOAL_POST_TIMEOUT_SEC="30.0"
POLL_PERIOD_SEC="1.0"
SETTLE_SEC="3.0"
LABEL="nav_pose_error"
OUTPUT_ROOT="${PROJECT_ROOT}/reports/navigation_pose_error_test"
PRE_RELOCALIZE="false"
POST_RELOCALIZE="true"
MOTION_SETTLE_TIMEOUT_SEC="60.0"
MOTION_SETTLE_QUIET_SEC="2.0"
ODOM_SAMPLER="true"

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
  --goal-completion-policy pose_required|position_only|api_default
                            Default: pose_required. api_default omits the
                            request field so robot_api_server resolves the
                            policy from the target pose type.
  --timeout-sec SEC         Maximum wait for navigation terminal state. Default: 180.
  --goal-post-timeout-sec SEC
                            Maximum wait for API goal POST response. Default: 30.0.
  --poll-period-sec SEC     API poll period. Default: 1.0.
  --settle-sec SEC          Extra wait after terminal state before final snapshot. Default: 3.0.
  --label NAME              Report label. Default: nav_pose_error.
  --api-url URL             robot_api_server URL. Default: http://127.0.0.1:8080.
  --output-root DIR         Report root. Default: reports/navigation_pose_error_test.
  --pre-relocalize          Run an explicit relocalization before sending the goal.
  --no-pre-relocalize       Do not run an explicit relocalization before sending the goal. Default.
  --no-post-relocalize      Do not run the post-goal relocalization correction capture.
  --motion-settle-timeout-sec SEC
                            Maximum wait for cmd_vel to become quiet before post-goal
                            relocalization. Default: 60.0.
  --motion-settle-quiet-sec SEC
                            Required continuous quiet time before post-goal
                            relocalization. Default: 2.0.
  --no-odom-sampler          Do not record read-only odom/motion/cmd_vel topic
                            samples during the navigation leg.

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
    --goal-post-timeout-sec)
      GOAL_POST_TIMEOUT_SEC="${2:-}"
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
    --motion-settle-timeout-sec)
      MOTION_SETTLE_TIMEOUT_SEC="${2:-}"
      shift 2
      ;;
    --motion-settle-quiet-sec)
      MOTION_SETTLE_QUIET_SEC="${2:-}"
      shift 2
      ;;
    --no-odom-sampler)
      ODOM_SAMPLER="false"
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

odom_sampler_pid=""

start_odom_sampler() {
  [[ "${ODOM_SAMPLER}" == "true" ]] || return 0
  local samples_path="${OUT_DIR}/odom_motion_samples.jsonl"
  local max_runtime_sec
  max_runtime_sec="$(python3 - "${TIMEOUT_SEC}" "${SETTLE_SEC}" "${MOTION_SETTLE_TIMEOUT_SEC}" <<'PY'
import sys
print(max(float(sys.argv[1]) + float(sys.argv[2]) + float(sys.argv[3]) + 30.0, 60.0))
PY
)"

  python3 - "${samples_path}" "${max_runtime_sec}" <<'PY' &
import json
import math
import signal
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

try:
    from ranger_msgs.msg import MotionState, SystemState
except Exception:
    MotionState = None
    SystemState = None

samples_path = sys.argv[1]
max_runtime_sec = max(float(sys.argv[2]), 1.0)
running = True


def handle_signal(signum, frame):
    del signum, frame
    global running
    running = False


signal.signal(signal.SIGINT, handle_signal)
signal.signal(signal.SIGTERM, handle_signal)


def stamp_to_float(stamp):
    try:
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9
    except Exception:
        return None


def yaw_from_quat(q):
    x = float(q.x)
    y = float(q.y)
    z = float(q.z)
    w = float(q.w)
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def scalar(value):
    try:
        if isinstance(value, bool):
            return value
        if isinstance(value, int):
            return int(value)
        value = float(value)
        return value if math.isfinite(value) else None
    except Exception:
        return None


def header_payload(msg):
    header = getattr(msg, "header", None)
    if header is None:
        return {}
    return {
        "stamp": stamp_to_float(getattr(header, "stamp", None)),
        "frame_id": str(getattr(header, "frame_id", "")),
    }


def odom_payload(msg):
    payload = header_payload(msg)
    payload.update(
        {
            "child_frame_id": str(getattr(msg, "child_frame_id", "")),
            "x": scalar(msg.pose.pose.position.x),
            "y": scalar(msg.pose.pose.position.y),
            "yaw": scalar(yaw_from_quat(msg.pose.pose.orientation)),
            "vx": scalar(msg.twist.twist.linear.x),
            "vy": scalar(msg.twist.twist.linear.y),
            "wz": scalar(msg.twist.twist.angular.z),
        }
    )
    return payload


def twist_payload(msg):
    return {
        "linear_x": scalar(msg.linear.x),
        "linear_y": scalar(msg.linear.y),
        "angular_z": scalar(msg.angular.z),
    }


def fields_payload(msg, names):
    payload = header_payload(msg)
    for name in names:
        if hasattr(msg, name):
            payload[name] = scalar(getattr(msg, name))
    return payload


rclpy.init()
node = rclpy.create_node("navigation_pose_error_odom_motion_sampler")
started_wall = time.time()
started_mono = time.monotonic()
handle = open(samples_path, "w", encoding="utf-8")


def emit(topic, payload):
    row = {
        "wall_time": time.time(),
        "elapsed_sec": time.monotonic() - started_mono,
        "topic": topic,
    }
    row.update(payload)
    handle.write(json.dumps(row, ensure_ascii=False, sort_keys=True) + "\n")
    handle.flush()


subscriptions = []
for topic in ("/wheel/odom", "/wheel/odom_ekf", "/local_state/odometry"):
    subscriptions.append(
        node.create_subscription(
            Odometry,
            topic,
            lambda msg, topic=topic: emit(topic, odom_payload(msg)),
            50,
        )
    )
for topic in ("/cmd_vel_nav", "/cmd_vel_collision_checked", "/cmd_vel_api", "/cmd_vel_safe", "/cmd_vel"):
    subscriptions.append(
        node.create_subscription(
            Twist,
            topic,
            lambda msg, topic=topic: emit(topic, twist_payload(msg)),
            20,
        )
    )
if MotionState is not None:
    subscriptions.append(
        node.create_subscription(
            MotionState,
            "/motion_state",
            lambda msg: emit(
                "/motion_state",
                fields_payload(
                    msg,
                    (
                        "motion_mode",
                        "linear_velocity",
                        "angular_velocity",
                        "steering_angle",
                    ),
                ),
            ),
            50,
        )
    )
if SystemState is not None:
    subscriptions.append(
        node.create_subscription(
            SystemState,
            "/system_state",
            lambda msg: emit(
                "/system_state",
                fields_payload(
                    msg,
                    (
                        "motion_mode",
                        "vehicle_state",
                        "control_mode",
                        "error_code",
                        "battery_voltage",
                    ),
                ),
            ),
            50,
        )
    )

try:
    while rclpy.ok() and running and (time.time() - started_wall) < max_runtime_sec:
        try:
            rclpy.spin_once(node, timeout_sec=0.05)
        except Exception:
            if not running or not rclpy.ok():
                break
            raise
finally:
    handle.close()
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
PY
  odom_sampler_pid=$!
  echo "[nav-pose-error] odom/motion sampler started pid=${odom_sampler_pid} samples=${samples_path}"
}

summarize_odom_sampler() {
  [[ "${ODOM_SAMPLER}" == "true" ]] || return 0
  local samples_path="${OUT_DIR}/odom_motion_samples.jsonl"
  local summary_json="${OUT_DIR}/odom_motion_summary.json"
  [[ -f "${samples_path}" ]] || return 0

  python3 - "${samples_path}" "${summary_json}" "${OUT_DIR}/summary.md" <<'PY'
import json
import math
import sys
from collections import Counter, defaultdict
from pathlib import Path

samples_path = Path(sys.argv[1])
summary_json = Path(sys.argv[2])
summary_md = Path(sys.argv[3])


def finite(value):
    try:
        value = float(value)
    except Exception:
        return None
    return value if math.isfinite(value) else None


def angle_diff(a, b):
    return math.atan2(math.sin(a - b), math.cos(a - b))


rows_by_topic = defaultdict(list)
for line in samples_path.read_text(encoding="utf-8", errors="replace").splitlines():
    if not line.strip():
        continue
    try:
        row = json.loads(line)
    except Exception:
        continue
    topic = row.get("topic")
    if topic:
        rows_by_topic[str(topic)].append(row)

odom_topics = ("/wheel/odom", "/wheel/odom_ekf", "/local_state/odometry")
twist_topics = ("/cmd_vel_nav", "/cmd_vel_collision_checked", "/cmd_vel_api", "/cmd_vel_safe", "/cmd_vel")
summary = {"topics": {}}

for topic, rows in sorted(rows_by_topic.items()):
    entry = {"samples": len(rows)}
    if topic in odom_topics:
        usable = [
            r
            for r in rows
            if finite(r.get("x")) is not None
            and finite(r.get("y")) is not None
            and finite(r.get("yaw")) is not None
        ]
        if usable:
            first = usable[0]
            last = usable[-1]
            dx = finite(last.get("x")) - finite(first.get("x"))
            dy = finite(last.get("y")) - finite(first.get("y"))
            dyaw = angle_diff(finite(last.get("yaw")), finite(first.get("yaw")))
            entry.update(
                {
                    "dx_m": dx,
                    "dy_m": dy,
                    "translation_m": math.hypot(dx, dy),
                    "dyaw_deg": math.degrees(dyaw),
                    "start_x": finite(first.get("x")),
                    "start_y": finite(first.get("y")),
                    "start_yaw": finite(first.get("yaw")),
                    "end_x": finite(last.get("x")),
                    "end_y": finite(last.get("y")),
                    "end_yaw": finite(last.get("yaw")),
                    "max_abs_vx": max(abs(finite(r.get("vx")) or 0.0) for r in usable),
                    "max_abs_vy": max(abs(finite(r.get("vy")) or 0.0) for r in usable),
                    "max_abs_wz": max(abs(finite(r.get("wz")) or 0.0) for r in usable),
                }
            )
    elif topic in twist_topics:
        entry.update(
            {
                "max_abs_linear_x": max(abs(finite(r.get("linear_x")) or 0.0) for r in rows),
                "max_abs_linear_y": max(abs(finite(r.get("linear_y")) or 0.0) for r in rows),
                "max_abs_angular_z": max(abs(finite(r.get("angular_z")) or 0.0) for r in rows),
            }
        )
    elif topic == "/motion_state":
        modes = Counter(str(r.get("motion_mode")) for r in rows if r.get("motion_mode") is not None)
        entry["motion_modes"] = dict(sorted(modes.items()))
        for key in ("linear_velocity", "angular_velocity", "steering_angle"):
            values = [finite(r.get(key)) for r in rows]
            values = [v for v in values if v is not None]
            if values:
                entry[f"min_{key}"] = min(values)
                entry[f"max_{key}"] = max(values)
    elif topic == "/system_state":
        modes = Counter(str(r.get("motion_mode")) for r in rows if r.get("motion_mode") is not None)
        entry["motion_modes"] = dict(sorted(modes.items()))
    summary["topics"][topic] = entry

summary_json.write_text(json.dumps(summary, ensure_ascii=False, indent=2, sort_keys=True) + "\n", encoding="utf-8")

with summary_md.open("a", encoding="utf-8") as handle:
    handle.write("\n## Odom/Motion Samples\n\n")
    handle.write(f"- samples_file: `{samples_path.name}`\n")
    handle.write(f"- summary_file: `{summary_json.name}`\n\n")
    handle.write("| topic | samples | dx_m | dy_m | translation_m | dyaw_deg | max_abs_vx_or_linear_x | max_abs_wz_or_angular_z | modes |\n")
    handle.write("|---|---:|---:|---:|---:|---:|---:|---:|---|\n")
    for topic in sorted(summary["topics"]):
        entry = summary["topics"][topic]
        modes = entry.get("motion_modes", "")
        if isinstance(modes, dict):
            modes = ",".join(f"{k}:{v}" for k, v in modes.items())
        vx = entry.get("max_abs_vx", entry.get("max_abs_linear_x", ""))
        wz = entry.get("max_abs_wz", entry.get("max_abs_angular_z", ""))
        handle.write(
            f"| `{topic}` | `{entry.get('samples', '')}` | "
            f"`{entry.get('dx_m', '')}` | `{entry.get('dy_m', '')}` | "
            f"`{entry.get('translation_m', '')}` | `{entry.get('dyaw_deg', '')}` | "
            f"`{vx}` | `{wz}` | `{modes}` |\n"
        )
PY
}

stop_odom_sampler() {
  [[ -n "${odom_sampler_pid}" ]] || return 0
  kill -TERM "${odom_sampler_pid}" 2>/dev/null || true
  wait "${odom_sampler_pid}" 2>/dev/null || true
  odom_sampler_pid=""
  summarize_odom_sampler || true
}

wait_for_motion_quiet() {
  local output_path="$1"
  python3 - \
    "${MOTION_SETTLE_TIMEOUT_SEC}" \
    "${MOTION_SETTLE_QUIET_SEC}" \
    "${output_path}" <<'PY'
import json
import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist

timeout_sec = max(float(sys.argv[1]), 0.1)
quiet_sec = max(float(sys.argv[2]), 0.1)
output_path = sys.argv[3]
linear_eps = 0.01
angular_eps = 0.02

state = {
    "quiet": False,
    "timeout_sec": timeout_sec,
    "quiet_sec": quiet_sec,
    "linear_eps": linear_eps,
    "angular_eps": angular_eps,
    "started_at_wall": time.time(),
    "elapsed_sec": 0.0,
    "quiet_elapsed_sec": 0.0,
    "last_motion_wall": None,
    "message_count": 0,
    "nonzero_count": 0,
    "max_abs_linear_x": 0.0,
    "max_abs_angular_z": 0.0,
    "latest": {},
}


def row(msg):
    return {
        "linear_x": float(msg.linear.x),
        "linear_y": float(msg.linear.y),
        "angular_z": float(msg.angular.z),
        "stamp_wall": time.time(),
    }


def update(name, msg):
    now = time.time()
    value = row(msg)
    state["latest"][name] = value
    state["message_count"] += 1
    state["max_abs_linear_x"] = max(state["max_abs_linear_x"], abs(value["linear_x"]))
    state["max_abs_angular_z"] = max(state["max_abs_angular_z"], abs(value["angular_z"]))
    moving = abs(value["linear_x"]) > linear_eps or abs(value["angular_z"]) > angular_eps
    if moving:
        state["nonzero_count"] += 1
        state["last_motion_wall"] = now


rclpy.init()
node = rclpy.create_node("navigation_pose_error_motion_settle_wait")
node.create_subscription(Twist, "/cmd_vel", lambda msg: update("cmd_vel", msg), 20)
node.create_subscription(Twist, "/cmd_vel_safe", lambda msg: update("cmd_vel_safe", msg), 20)

start = time.time()
last_motion = start
try:
    while rclpy.ok():
        now = time.time()
        state["elapsed_sec"] = now - start
        if state["last_motion_wall"] is not None:
            last_motion = state["last_motion_wall"]
        state["quiet_elapsed_sec"] = now - last_motion
        if state["message_count"] > 0 and state["quiet_elapsed_sec"] >= quiet_sec:
            state["quiet"] = True
            break
        if state["elapsed_sec"] >= timeout_sec:
            break
        rclpy.spin_once(node, timeout_sec=0.05)
finally:
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()

with open(output_path, "w", encoding="utf-8") as handle:
    json.dump(state, handle, indent=2, sort_keys=True)

raise SystemExit(0 if state["quiet"] else 12)
PY
}

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

if policy not in ("pose_required", "position_only", "api_default", "default", "none", "omit"):
    raise SystemExit("goal completion policy must be pose_required, position_only, or api_default")


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

if policy in ("api_default", "default", "none", "omit"):
    body.pop("goal_completion_policy", None)
else:
    body.setdefault("goal_completion_policy", policy)
path.write_text(json.dumps(body, ensure_ascii=False, indent=2, sort_keys=True) + "\n", encoding="utf-8")
PY

{
  echo "# Navigation Pose Error Test Environment"
  echo "- timestamp_utc: ${timestamp}"
  echo "- api_url: ${API_URL}"
  echo "- timeout_sec: ${TIMEOUT_SEC}"
  echo "- goal_post_timeout_sec: ${GOAL_POST_TIMEOUT_SEC}"
  echo "- poll_period_sec: ${POLL_PERIOD_SEC}"
  echo "- settle_sec: ${SETTLE_SEC}"
  echo "- motion_settle_timeout_sec: ${MOTION_SETTLE_TIMEOUT_SEC}"
  echo "- motion_settle_quiet_sec: ${MOTION_SETTLE_QUIET_SEC}"
  echo "- odom_sampler: ${ODOM_SAMPLER}"
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
    /cmd_vel_api \
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

start_odom_sampler

set +e
python3 - \
  "${OUT_DIR}" \
  "${API_URL}" \
  "${TIMEOUT_SEC}" \
  "${POLL_PERIOD_SEC}" \
  "${SETTLE_SEC}" \
  "${GOAL_POST_TIMEOUT_SEC}" <<'PY'
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
goal_post_timeout_sec = max(float(sys.argv[6]), 1.0)

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

post = api_json("POST", "/api/v1/navigation/goal", goal_request, timeout=goal_post_timeout_sec)
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
    "goal_completion_policy",
    "task_complete",
    "target_x",
    "target_y",
    "target_yaw",
    "nav2_goal_yaw_rad",
    "nav2_goal_yaw_source",
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
            "goal_completion_policy": goal.get("goal_completion_policy", ""),
            "task_complete": goal.get("task_complete", ""),
            "target_x": target.get("x", ""),
            "target_y": target.get("y", ""),
            "target_yaw": target.get("yaw", ""),
            "nav2_goal_yaw_rad": goal.get("nav2_goal_yaw_rad", ""),
            "nav2_goal_yaw_source": goal.get("nav2_goal_yaw_source", ""),
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
        "nav2_goal_yaw_rad",
        "nav2_goal_yaw_source",
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
  echo "[nav-pose-error] waiting for motion quiet before post-goal relocalize..."
  set +e
  wait_for_motion_quiet "${OUT_DIR}/post_goal_motion_settle.json"
  motion_settle_rc=$?
  set -e
  python3 - "${OUT_DIR}" "${motion_settle_rc}" <<'PY'
import json
import sys
from pathlib import Path

out = Path(sys.argv[1])
rc = sys.argv[2]
summary = out / "summary.md"
settle_path = out / "post_goal_motion_settle.json"
with summary.open("a", encoding="utf-8") as f:
    f.write("\n## Post-Goal Motion Settle\n\n")
    f.write(f"- motion_settle_exit_code: `{rc}`\n")
    if settle_path.exists():
        settle = json.loads(settle_path.read_text(encoding="utf-8"))
        f.write(f"- quiet: `{settle.get('quiet')}`\n")
        f.write(f"- elapsed_sec: `{settle.get('elapsed_sec')}`\n")
        f.write(f"- quiet_elapsed_sec: `{settle.get('quiet_elapsed_sec')}`\n")
        f.write(f"- message_count: `{settle.get('message_count')}`\n")
        f.write(f"- nonzero_count: `{settle.get('nonzero_count')}`\n")
        f.write(f"- max_abs_linear_x: `{settle.get('max_abs_linear_x')}`\n")
        f.write(f"- max_abs_angular_z: `{settle.get('max_abs_angular_z')}`\n")
    else:
        f.write("- metrics: `missing`\n")
PY
  if [[ "${motion_settle_rc}" != "0" ]]; then
    echo "[nav-pose-error] post-goal relocalize skipped because motion did not settle rc=${motion_settle_rc}"
    nav_rc="${motion_settle_rc}"
  else
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
  fi
else
  echo "[nav-pose-error] post-goal relocalize skipped nav_rc=${nav_rc} post_relocalize=${POST_RELOCALIZE}"
fi

stop_odom_sampler

echo "[nav-pose-error] summary: ${OUT_DIR}/summary.md"
exit "${nav_rc}"
