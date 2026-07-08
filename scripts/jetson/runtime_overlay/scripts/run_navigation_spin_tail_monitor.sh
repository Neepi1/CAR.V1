#!/usr/bin/env bash
set -euo pipefail
umask 0002

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/../../../.." && pwd)"
if [[ -f "${SCRIPT_DIR}/common_env.sh" ]]; then
  # shellcheck source=/dev/null
  source "${SCRIPT_DIR}/common_env.sh"
fi
PROJECT_ROOT="${NJRH_PROJECT_ROOT:-${PROJECT_ROOT}}"

API_URL="${API_URL:-http://127.0.0.1:8080}"
POSE_ID=""
BUILDING_ID=""
FLOOR_ID=""
TIMEOUT_SEC=180
SAMPLE_PERIOD_SEC=0.05
POST_CAPTURE_SEC=5.0
LABEL="nav_spin_tail"
OUTPUT_ROOT="${PROJECT_ROOT}/reports/navigation_spin_tail_monitor"
PREFIX="[nav-spin-tail]"

usage() {
  cat <<'EOF'
Usage:
  run_navigation_spin_tail_monitor.sh --pose-id delivery_675235 [options]

Runs one normal robot_api_server navigation goal and records the spin-to-drive
handoff chain:
  /cmd_vel_* -> /wheel/odom -> /local_state/odometry -> /lidar_imu_bias_corrected

The summary reports whether the first following linear command was released
before or after wheel/IMU/local yaw-rate were stable.

Options:
  --pose-id ID              Saved pose id.
  --building-id ID          Optional building id; auto-filled from runtime context.
  --floor-id ID             Optional floor id; auto-filled from runtime context.
  --timeout-sec SEC         Navigation timeout. Default: 180.
  --sample-period-sec SEC   Sample period. Default: 0.05.
  --post-capture-sec SEC    Capture after terminal state. Default: 5.0.
  --label LABEL             Report label. Default: nav_spin_tail.
  --api-url URL             robot_api_server URL. Default: http://127.0.0.1:8080.
  --output-root DIR         Report root. Default: reports/navigation_spin_tail_monitor,
                            falling back to /tmp if not writable.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
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
    --timeout-sec)
      TIMEOUT_SEC="${2:-}"
      shift 2
      ;;
    --sample-period-sec)
      SAMPLE_PERIOD_SEC="${2:-}"
      shift 2
      ;;
    --post-capture-sec)
      POST_CAPTURE_SEC="${2:-}"
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
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "${PREFIX} unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

if [[ -z "${POSE_ID}" ]]; then
  echo "${PREFIX} --pose-id is required" >&2
  usage >&2
  exit 2
fi

python3 - "${TIMEOUT_SEC}" "${SAMPLE_PERIOD_SEC}" "${POST_CAPTURE_SEC}" <<'PY'
import math
import sys

for value in sys.argv[1:]:
    number = float(value)
    if not math.isfinite(number) or number <= 0.0:
        raise SystemExit("numeric options must be finite positive values")
PY

safe_label="$(printf '%s' "${LABEL}" | tr -c 'A-Za-z0-9_.-' '_')"
timestamp="$(date -u +%Y%m%dT%H%M%SZ)"
if ! mkdir -p "${OUTPUT_ROOT}" 2>/dev/null || [[ ! -w "${OUTPUT_ROOT}" ]]; then
  OUTPUT_ROOT="${TMPDIR:-/tmp}/navigation_spin_tail_monitor"
  mkdir -p "${OUTPUT_ROOT}"
fi
OUT_DIR="${OUTPUT_ROOT}/${timestamp}_${safe_label}"
mkdir -p "${OUT_DIR}"

echo "${PREFIX} report: ${OUT_DIR}"

python3 - \
  "${OUT_DIR}" \
  "${API_URL}" \
  "${POSE_ID}" \
  "${BUILDING_ID}" \
  "${FLOOR_ID}" \
  "${TIMEOUT_SEC}" \
  "${SAMPLE_PERIOD_SEC}" \
  "${POST_CAPTURE_SEC}" <<'PY'
import csv
import json
import math
import sys
import time
import urllib.error
import urllib.request
from pathlib import Path

import rclpy
from action_msgs.msg import GoalStatusArray
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Imu
from std_msgs.msg import String

out_dir = Path(sys.argv[1])
api_url = sys.argv[2].rstrip("/")
pose_id = sys.argv[3]
building_id = sys.argv[4]
floor_id = sys.argv[5]
timeout_sec = float(sys.argv[6])
sample_period_sec = float(sys.argv[7])
post_capture_sec = float(sys.argv[8])


def api_json(method, path, body=None, timeout=3.0):
    data = None
    headers = {}
    if body is not None:
        data = json.dumps(body).encode("utf-8")
        headers["Content-Type"] = "application/json"
    req = urllib.request.Request(api_url + path, data=data, method=method, headers=headers)
    try:
        with urllib.request.urlopen(req, timeout=timeout) as resp:
            text = resp.read().decode("utf-8", errors="replace")
            return {"ok": True, "status": resp.status, "body": json.loads(text) if text else {}}
    except urllib.error.HTTPError as exc:
        text = exc.read().decode("utf-8", errors="replace")
        try:
            parsed = json.loads(text)
        except Exception:
            parsed = {"raw": text}
        return {"ok": False, "status": exc.code, "body": parsed, "error": str(exc)}
    except Exception as exc:
        return {"ok": False, "status": None, "body": {}, "error": repr(exc)}


def find_context(payload):
    if not isinstance(payload, dict):
        return None
    body = payload.get("body") if isinstance(payload.get("body"), dict) else payload
    candidates = [body.get("runtime_map_context"), body.get("map_context")]
    nav = body.get("navigation")
    if isinstance(nav, dict):
        candidates.extend([nav.get("runtime_map_context"), nav.get("map_context")])
    for candidate in candidates:
        if not isinstance(candidate, dict):
            continue
        b = str(candidate.get("building_id") or "")
        f = str(candidate.get("floor_id") or "")
        if b and f:
            return b, f
    return None


def resolve_context():
    for path in ("/api/v1/navigation/state", "/api/v1/status"):
        found = find_context(api_json("GET", path, timeout=3.0))
        if found:
            return found
    return None


def yaw_from_quat(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def norm_angle(value):
    while value > math.pi:
        value -= 2.0 * math.pi
    while value < -math.pi:
        value += 2.0 * math.pi
    return value


def to_float(value, default=None):
    try:
        result = float(value)
    except Exception:
        return default
    return result if math.isfinite(result) else default


def terminal_goal(goal):
    if not isinstance(goal, dict):
        return False
    state = str(goal.get("state") or "").lower()
    phase = str(goal.get("phase") or "").lower()
    if state in {"succeeded", "failed", "canceled", "degraded", "rejected"}:
        return True
    if phase in {"succeeded", "failed", "canceled", "nav2_succeeded", "nav2_failed", "nav2_canceled"}:
        return True
    return goal.get("task_complete") is True


def twist_row(msg):
    return {"x": float(msg.linear.x), "y": float(msg.linear.y), "z": float(msg.angular.z)}


def odom_row(msg):
    return {
        "x": float(msg.pose.pose.position.x),
        "y": float(msg.pose.pose.position.y),
        "yaw": yaw_from_quat(msg.pose.pose.orientation),
        "vx": float(msg.twist.twist.linear.x),
        "vy": float(msg.twist.twist.linear.y),
        "wz": float(msg.twist.twist.angular.z),
    }


class Recorder(Node):
    def __init__(self):
        super().__init__("navigation_spin_tail_monitor")
        self.start = time.monotonic()
        self.data = {}
        self.rows = []
        self.last_nav_state = {}
        self.last_nav_goal = {}
        self.last_goal_status = ""

        reliable = QoSProfile(depth=80)
        best_effort = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=120, reliability=ReliabilityPolicy.BEST_EFFORT)
        for topic, key in (
            ("/cmd_vel_nav_raw", "cmd_nav_raw"),
            ("/cmd_vel_nav", "cmd_nav"),
            ("/cmd_vel_collision_checked", "cmd_collision"),
            ("/cmd_vel_safe", "cmd_safe"),
            ("/cmd_vel", "cmd_base"),
        ):
            self.create_subscription(Twist, topic, lambda msg, k=key: self.data.__setitem__(k, twist_row(msg)), reliable)
        for topic, key in (
            ("/wheel/odom", "wheel"),
            ("/wheel/odom_ekf", "wheel_ekf"),
            ("/local_state/odometry", "local"),
        ):
            self.create_subscription(Odometry, topic, lambda msg, k=key: self.data.__setitem__(k, odom_row(msg)), best_effort)
        self.create_subscription(Imu, "/lidar_imu_bias_corrected", self.on_imu, qos_profile_sensor_data)
        self.create_subscription(String, "/safety/status", lambda msg: self.data.__setitem__("safety_status", msg.data), reliable)
        self.create_subscription(String, "/localization/bridge_status", lambda msg: self.data.__setitem__("bridge_status", msg.data), reliable)
        self.create_subscription(GoalStatusArray, "/navigate_to_pose/_action/status", self.on_goal_status, reliable)
        self.create_timer(sample_period_sec, self.sample)

    def rel(self):
        return time.monotonic() - self.start

    def on_imu(self, msg):
        self.data["imu_wz"] = float(msg.angular_velocity.z)

    def on_goal_status(self, msg):
        if not msg.status_list:
            return
        self.last_goal_status = str(msg.status_list[-1].status)

    def refresh_api(self):
        state = api_json("GET", "/api/v1/navigation/state", timeout=0.35)
        if isinstance(state.get("body"), dict):
            self.last_nav_state = state["body"]
            goal = self.last_nav_state.get("navigation_goal")
            if isinstance(goal, dict):
                self.last_nav_goal = goal

    def sample(self):
        self.refresh_api()
        goal = self.last_nav_goal if isinstance(self.last_nav_goal, dict) else {}
        row = {
            "t": self.rel(),
            "goal_state": goal.get("state", ""),
            "goal_phase": goal.get("phase", ""),
            "pose_id": goal.get("pose_id", ""),
            "final_distance_m": goal.get("final_distance_m", ""),
            "final_yaw_error_rad": goal.get("final_yaw_error_rad", ""),
            "nav2_status": self.last_goal_status,
            "imu_wz": self.data.get("imu_wz", ""),
            "safety_status": self.data.get("safety_status", ""),
            "bridge_status": self.data.get("bridge_status", ""),
        }
        for key in ("cmd_nav_raw", "cmd_nav", "cmd_collision", "cmd_safe", "cmd_base"):
            value = self.data.get(key) or {}
            row[f"{key}_x"] = value.get("x", "")
            row[f"{key}_y"] = value.get("y", "")
            row[f"{key}_z"] = value.get("z", "")
        for key in ("wheel", "wheel_ekf", "local"):
            value = self.data.get(key) or {}
            row[f"{key}_x"] = value.get("x", "")
            row[f"{key}_y"] = value.get("y", "")
            row[f"{key}_yaw"] = value.get("yaw", "")
            row[f"{key}_vx"] = value.get("vx", "")
            row[f"{key}_wz"] = value.get("wz", "")
        self.rows.append(row)


def write_samples(rows):
    fields = [
        "t", "goal_state", "goal_phase", "pose_id", "final_distance_m", "final_yaw_error_rad", "nav2_status",
        "imu_wz", "safety_status", "bridge_status",
    ]
    for key in ("cmd_nav_raw", "cmd_nav", "cmd_collision", "cmd_safe", "cmd_base"):
        fields.extend([f"{key}_x", f"{key}_y", f"{key}_z"])
    for key in ("wheel", "wheel_ekf", "local"):
        fields.extend([f"{key}_x", f"{key}_y", f"{key}_yaw", f"{key}_vx", f"{key}_wz"])
    with (out_dir / "samples.csv").open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        for row in rows:
            writer.writerow({key: row.get(key, "") for key in fields})


def integrate_imu(rows):
    prev_t = None
    prev_w = None
    yaw = 0.0
    for row in rows:
        t = to_float(row.get("t"))
        w = to_float(row.get("imu_wz"))
        if prev_t is not None and prev_w is not None and t is not None and w is not None:
            dt = max(0.0, min(t - prev_t, 0.2))
            yaw += 0.5 * (prev_w + w) * dt
        row["imu_yaw_int"] = yaw
        if t is not None and w is not None:
            prev_t = t
            prev_w = w


def stable_index(rows, start_idx):
    for idx in range(start_idx, len(rows)):
        start_t = to_float(rows[idx].get("t"))
        if start_t is None:
            continue
        ok = True
        seen_future = False
        for j in range(idx, len(rows)):
            t = to_float(rows[j].get("t"))
            if t is None:
                continue
            if t - start_t > 0.35:
                seen_future = True
                break
            imu_wz = abs(to_float(rows[j].get("imu_wz"), 999.0))
            wheel_wz = abs(to_float(rows[j].get("wheel_wz"), 999.0))
            local_wz = abs(to_float(rows[j].get("local_wz"), 999.0))
            if imu_wz > 0.035 or wheel_wz > 0.02 or local_wz > 0.03:
                ok = False
                break
        if ok and seen_future:
            return idx
    return None


def angle_delta(rows, a, b, key):
    va = to_float(rows[a].get(key))
    vb = to_float(rows[b].get(key))
    if va is None or vb is None:
        return None
    if key.endswith("_yaw"):
        return norm_angle(vb - va)
    return vb - va


def analyze_tail(rows):
    integrate_imu(rows)
    segments = []
    i = 0
    while i < len(rows):
        cmd_z = abs(to_float(rows[i].get("cmd_base_z"), 0.0))
        cmd_x = abs(to_float(rows[i].get("cmd_base_x"), 0.0))
        if cmd_z <= 0.08 or cmd_x >= 0.05:
            i += 1
            continue
        start = i
        last_cmd_spin = i
        j = i + 1
        while j < len(rows):
            z = abs(to_float(rows[j].get("cmd_base_z"), 0.0))
            x = abs(to_float(rows[j].get("cmd_base_x"), 0.0))
            if z > 0.08 and x < 0.05:
                last_cmd_spin = j
            if j - last_cmd_spin > int(max(4, 0.8 / sample_period_sec)):
                break
            j += 1
        cmd_zero = min(last_cmd_spin + 1, len(rows) - 1)
        stable = stable_index(rows, cmd_zero)
        first_linear = None
        for k in range(cmd_zero, len(rows)):
            if abs(to_float(rows[k].get("cmd_base_x"), 0.0)) > 0.03:
                first_linear = k
                break
            if stable is not None and k > stable + int(1.0 / sample_period_sec):
                break
        if stable is None:
            end = min(len(rows) - 1, cmd_zero + int(3.0 / sample_period_sec))
        else:
            end = stable
        wheel_tail = angle_delta(rows, cmd_zero, end, "wheel_yaw")
        local_tail = angle_delta(rows, cmd_zero, end, "local_yaw")
        imu_tail = angle_delta(rows, cmd_zero, end, "imu_yaw_int")
        local_at_linear = None
        imu_at_linear = None
        if first_linear is not None:
            local_at_linear = angle_delta(rows, cmd_zero, first_linear, "local_yaw")
            imu_at_linear = angle_delta(rows, cmd_zero, first_linear, "imu_yaw_int")
        segments.append({
            "index": len(segments) + 1,
            "start_s": to_float(rows[start].get("t")),
            "cmd_zero_s": to_float(rows[cmd_zero].get("t")),
            "stable_s": None if stable is None else to_float(rows[stable].get("t")),
            "first_linear_s": None if first_linear is None else to_float(rows[first_linear].get("t")),
            "linear_released_after_stable": None if stable is None or first_linear is None else to_float(rows[first_linear].get("t")) >= to_float(rows[stable].get("t")),
            "wheel_tail_deg": None if wheel_tail is None else math.degrees(wheel_tail),
            "local_tail_deg": None if local_tail is None else math.degrees(local_tail),
            "imu_tail_deg": None if imu_tail is None else math.degrees(imu_tail),
            "local_minus_imu_tail_deg": None if local_tail is None or imu_tail is None else math.degrees(local_tail - imu_tail),
            "local_tail_before_linear_deg": None if local_at_linear is None else math.degrees(local_at_linear),
            "imu_tail_before_linear_deg": None if imu_at_linear is None else math.degrees(imu_at_linear),
        })
        i = max(j, cmd_zero + 1)
    return segments


if not building_id or not floor_id:
    context = resolve_context()
    if context:
        building_id, floor_id = context
if not building_id or not floor_id:
    raise SystemExit("cannot resolve building_id/floor_id from runtime context")

goal_request = {"pose_id": pose_id, "building_id": building_id, "floor_id": floor_id}
(out_dir / "goal_request.json").write_text(json.dumps(goal_request, indent=2, sort_keys=True) + "\n", encoding="utf-8")

rclpy.init()
node = Recorder()
try:
    warm_deadline = time.monotonic() + 1.0
    while time.monotonic() < warm_deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
    post = api_json("POST", "/api/v1/navigation/goal", goal_request, timeout=30.0)
    (out_dir / "post_goal_response.json").write_text(json.dumps(post, indent=2, sort_keys=True, ensure_ascii=False) + "\n", encoding="utf-8")
    if not post.get("ok") or (isinstance(post.get("body"), dict) and post["body"].get("accepted") is False):
        raise SystemExit(20)
    deadline = time.monotonic() + timeout_sec
    terminal_seen_at = None
    seen_current_goal = False
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        goal = node.last_nav_goal if isinstance(node.last_nav_goal, dict) else {}
        if str(goal.get("pose_id") or "") == pose_id:
            seen_current_goal = True
        if not seen_current_goal:
            continue
        if terminal_goal(goal):
            terminal_seen_at = time.monotonic()
            break
    if terminal_seen_at is None:
        nav_rc = 30
    else:
        while time.monotonic() - terminal_seen_at < post_capture_sec:
            rclpy.spin_once(node, timeout_sec=0.05)
        nav_rc = 0
finally:
    rows = node.rows
    write_samples(rows)
    segments = analyze_tail(rows)
    summary = {
        "pose_id": pose_id,
        "building_id": building_id,
        "floor_id": floor_id,
        "sample_count": len(rows),
        "last_goal_state": rows[-1].get("goal_state", "") if rows else "",
        "last_goal_phase": rows[-1].get("goal_phase", "") if rows else "",
        "last_final_distance_m": rows[-1].get("final_distance_m", "") if rows else "",
        "last_final_yaw_error_rad": rows[-1].get("final_yaw_error_rad", "") if rows else "",
        "spin_tail_segments": segments,
    }
    (out_dir / "summary.json").write_text(json.dumps(summary, indent=2, sort_keys=True, ensure_ascii=False) + "\n", encoding="utf-8")
    with (out_dir / "summary.md").open("w", encoding="utf-8") as handle:
        handle.write("# Navigation Spin Tail Monitor\n\n")
        handle.write(f"- pose_id: `{pose_id}`\n")
        handle.write(f"- sample_count: `{len(rows)}`\n")
        handle.write(f"- last_goal_state: `{summary['last_goal_state']}`\n")
        handle.write(f"- last_goal_phase: `{summary['last_goal_phase']}`\n")
        handle.write(f"- last_final_distance_m: `{summary['last_final_distance_m']}`\n")
        handle.write(f"- last_final_yaw_error_rad: `{summary['last_final_yaw_error_rad']}`\n")
        handle.write("\n## Spin Tail Segments\n\n")
        handle.write("| # | start_s | cmd_zero_s | stable_s | first_linear_s | released_after_stable | wheel_tail_deg | local_tail_deg | imu_tail_deg | local_minus_imu_deg | local_before_linear_deg | imu_before_linear_deg |\n")
        handle.write("|---:|---:|---:|---:|---:|---|---:|---:|---:|---:|---:|---:|\n")
        for s in segments:
            handle.write(
                f"| {s['index']} | {s['start_s']} | {s['cmd_zero_s']} | {s['stable_s']} | "
                f"{s['first_linear_s']} | {s['linear_released_after_stable']} | "
                f"{s['wheel_tail_deg']} | {s['local_tail_deg']} | {s['imu_tail_deg']} | "
                f"{s['local_minus_imu_tail_deg']} | {s['local_tail_before_linear_deg']} | {s['imu_tail_before_linear_deg']} |\n"
            )
        handle.write("\nFiles:\n\n- `samples.csv`\n- `summary.json`\n- `post_goal_response.json`\n")
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()

print(f"{out_dir}/summary.md")
raise SystemExit(nav_rc)
PY

echo "${PREFIX} summary: ${OUT_DIR}/summary.md"
