#!/usr/bin/env bash
set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="${WORKSPACE_ROOT:-$(cd "${SCRIPT_DIR}/../../../.." && pwd)}"
export NJRH_PROJECT_ROOT="${NJRH_PROJECT_ROOT:-${WORKSPACE_ROOT}}"
if [[ -f "${SCRIPT_DIR}/common_env.sh" ]]; then
  # shellcheck source=common_env.sh
  source "${SCRIPT_DIR}/common_env.sh"
fi
set +e

API_URL="${API_URL:-http://127.0.0.1:8080}"
DURATION_SEC=40
SAMPLE_PERIOD_SEC=0.5
LABEL="terminal_adjustment"
OUTPUT_DIR=""
NEAR_GOAL_M=1.5
VERY_NEAR_M=0.25
SLOW_LINEAR_MPS=0.05
SLOW_ANGULAR_RADPS=0.05
INCLUDE_ROSOUT=true
STOP_WHEN_TERMINAL=false
PREFIX="[nav-terminal-adjust]"

usage() {
  cat <<'EOF'
Usage:
  bash scripts/jetson/runtime_overlay/scripts/observe_navigation_terminal_adjustment.sh \
    --duration-sec 40 \
    --label delivery_675235_terminal

Start this script first, then send one normal navigation goal from the App.

Read-only observer:
  - does not send goals, publish velocity, call services, set params, or restart nodes
  - polls /api/v1/navigation/state and /api/v1/robot/pose
  - subscribes to Nav2 action status, /speed_limit, bridge/safety/mode status, and cmd_vel chain
  - separates slow Nav2 near-goal adjustment from API final_yaw_align handoff

Options:
  --duration-sec N              Capture duration in seconds. Default: 40.
  --sample-period-sec N         API/summary sample period. Default: 0.5.
  --label LABEL                 Report label. Default: terminal_adjustment.
  --api-url URL                 robot_api_server URL. Default: http://127.0.0.1:8080.
  --output-dir DIR              Report directory. Default: reports/navigation_terminal_adjustment/<timestamp>_<label>_<duration>s.
  --near-goal-m M               Near-goal window for Nav2 terminal analysis. Default: 1.5.
  --very-near-m M               Very-near window for terminal crawl analysis. Default: 0.25.
  --slow-linear-mps V           Linear command considered too slow. Default: 0.05.
  --slow-angular-radps W        Angular command considered too slow. Default: 0.05.
  --no-rosout                   Do not subscribe to filtered /rosout.
  --stop-when-terminal          Stop after this observer sees a running goal reach a terminal state.
  -h, --help                    Show this help.
EOF
}

sanitize_label() {
  printf '%s' "$1" | tr -c 'A-Za-z0-9_.-' '_'
}

is_number() {
  [[ "$1" =~ ^[0-9]+([.][0-9]+)?$ ]]
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --duration-sec)
      DURATION_SEC="${2:-}"
      shift 2
      ;;
    --sample-period-sec)
      SAMPLE_PERIOD_SEC="${2:-}"
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
    --output-dir)
      OUTPUT_DIR="${2:-}"
      shift 2
      ;;
    --near-goal-m)
      NEAR_GOAL_M="${2:-}"
      shift 2
      ;;
    --very-near-m)
      VERY_NEAR_M="${2:-}"
      shift 2
      ;;
    --slow-linear-mps)
      SLOW_LINEAR_MPS="${2:-}"
      shift 2
      ;;
    --slow-angular-radps)
      SLOW_ANGULAR_RADPS="${2:-}"
      shift 2
      ;;
    --no-rosout)
      INCLUDE_ROSOUT=false
      shift
      ;;
    --stop-when-terminal)
      STOP_WHEN_TERMINAL=true
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "${PREFIX} FAIL unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

if ! [[ "${DURATION_SEC}" =~ ^[0-9]+$ ]] || [[ "${DURATION_SEC}" -lt 20 ]]; then
  echo "${PREFIX} FAIL --duration-sec must be an integer >= 20" >&2
  exit 2
fi
for value in "${SAMPLE_PERIOD_SEC}" "${NEAR_GOAL_M}" "${VERY_NEAR_M}" "${SLOW_LINEAR_MPS}" "${SLOW_ANGULAR_RADPS}"; do
  if ! is_number "${value}"; then
    echo "${PREFIX} FAIL numeric argument is invalid: ${value}" >&2
    exit 2
  fi
done

TIMESTAMP="$(date -u +%Y%m%dT%H%M%SZ)"
LABEL="$(sanitize_label "${LABEL}")"
if [[ -z "${OUTPUT_DIR}" ]]; then
  OUTPUT_DIR="${NJRH_PROJECT_ROOT}/reports/navigation_terminal_adjustment/${TIMESTAMP}_${LABEL}_${DURATION_SEC}s"
fi
mkdir -p "${OUTPUT_DIR}"

{
  echo "timestamp_utc=${TIMESTAMP}"
  echo "duration_sec=${DURATION_SEC}"
  echo "sample_period_sec=${SAMPLE_PERIOD_SEC}"
  echo "label=${LABEL}"
  echo "api_url=${API_URL}"
  echo "near_goal_m=${NEAR_GOAL_M}"
  echo "very_near_m=${VERY_NEAR_M}"
  echo "slow_linear_mps=${SLOW_LINEAR_MPS}"
  echo "slow_angular_radps=${SLOW_ANGULAR_RADPS}"
  echo "stop_when_terminal=${STOP_WHEN_TERMINAL}"
  echo "workspace_root=${WORKSPACE_ROOT}"
} >"${OUTPUT_DIR}/metadata.env"

echo "${PREFIX} report_dir=${OUTPUT_DIR}"
echo "${PREFIX} duration_sec=${DURATION_SEC} sample_period_sec=${SAMPLE_PERIOD_SEC}"
echo "${PREFIX} near_goal_m=${NEAR_GOAL_M} very_near_m=${VERY_NEAR_M}"
echo "${PREFIX} read-only: start the App navigation goal now if it is not already running"

python3 - \
  "${DURATION_SEC}" \
  "${SAMPLE_PERIOD_SEC}" \
  "${API_URL}" \
  "${OUTPUT_DIR}" \
  "${NEAR_GOAL_M}" \
  "${VERY_NEAR_M}" \
  "${SLOW_LINEAR_MPS}" \
  "${SLOW_ANGULAR_RADPS}" \
  "${INCLUDE_ROSOUT}" \
  "${STOP_WHEN_TERMINAL}" <<'PY'
import json
import math
import re
import sys
import time
import urllib.error
import urllib.request
from collections import Counter, defaultdict
from pathlib import Path

import rclpy
from action_msgs.msg import GoalStatusArray
from geometry_msgs.msg import Twist
from nav2_msgs.msg import SpeedLimit
from rcl_interfaces.msg import Log
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


duration_sec = float(sys.argv[1])
sample_period_sec = float(sys.argv[2])
api_url = sys.argv[3].rstrip("/")
output_dir = Path(sys.argv[4])
near_goal_m = float(sys.argv[5])
very_near_m = float(sys.argv[6])
slow_linear_mps = float(sys.argv[7])
slow_angular_radps = float(sys.argv[8])
include_rosout = sys.argv[9].lower() == "true"
stop_when_terminal = sys.argv[10].lower() == "true"

output_dir.mkdir(parents=True, exist_ok=True)

STATUS_NAMES = {
    0: "UNKNOWN",
    1: "ACCEPTED",
    2: "EXECUTING",
    3: "CANCELING",
    4: "SUCCEEDED",
    5: "CANCELED",
    6: "ABORTED",
}

ROSOUT_FILTER = re.compile(
    r"controller_server|bt_navigator|follow_path|navigate|goal|progress|"
    r"mppi|critic|rotation.?shim|speed.?limit|cmd_vel|collision|abort|failed|"
    r"transform|local_costmap|bridge|localization|amcl",
    re.IGNORECASE,
)

CMD_TOPICS = (
    "/cmd_vel_nav_raw",
    "/cmd_vel_nav",
    "/cmd_vel_collision_checked",
    "/cmd_vel_safe",
    "/cmd_vel",
    "/cmd_vel_api",
)


def now_iso():
    return time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())


def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def finite_number(value):
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(result):
        return None
    return result


def short_uuid(uuid_msg):
    try:
        values = [int(x) for x in uuid_msg.uuid]
        return "".join(f"{x:02x}" for x in values)[:12]
    except Exception:
        return "unknown"


def twist_dict(msg):
    return {
        "linear_x": float(msg.linear.x),
        "linear_y": float(msg.linear.y),
        "angular_z": float(msg.angular.z),
    }


def twist_is_tiny(twist):
    if not twist:
        return True
    return (
        abs(float(twist.get("linear_x", 0.0))) < slow_linear_mps
        and abs(float(twist.get("linear_y", 0.0))) < slow_linear_mps
        and abs(float(twist.get("angular_z", 0.0))) < slow_angular_radps
    )


def twist_is_zero(twist):
    if not twist:
        return True
    return (
        abs(float(twist.get("linear_x", 0.0))) < 1.0e-4
        and abs(float(twist.get("linear_y", 0.0))) < 1.0e-4
        and abs(float(twist.get("angular_z", 0.0))) < 1.0e-4
    )


def api_get_json(path, timeout=0.45):
    url = f"{api_url}{path}"
    try:
        with urllib.request.urlopen(url, timeout=timeout) as resp:
            return json.loads(resp.read().decode("utf-8", errors="replace")), None
    except urllib.error.HTTPError as exc:
        try:
            body = exc.read().decode("utf-8", errors="replace")
        except Exception:
            body = ""
        return None, f"http_{exc.code}:{body[:160]}"
    except Exception as exc:
        return None, f"{type(exc).__name__}:{exc}"


def active_goal(nav_state):
    if not isinstance(nav_state, dict):
        return {}
    goal = nav_state.get("navigation_goal")
    return goal if isinstance(goal, dict) else {}


def target_from_goal(goal):
    target = goal.get("target")
    if not isinstance(target, dict):
        return None
    x = finite_number(target.get("x"))
    y = finite_number(target.get("y"))
    yaw = finite_number(target.get("yaw"))
    if x is None or y is None or yaw is None:
        return None
    return {"x": x, "y": y, "yaw": yaw}


def pose_from_payload(payload):
    if not isinstance(payload, dict) or payload.get("ok") is not True:
        return None
    x = finite_number(payload.get("x"))
    y = finite_number(payload.get("y"))
    yaw = finite_number(payload.get("yaw"))
    age = finite_number(payload.get("age_sec"))
    if x is None or y is None or yaw is None:
        return None
    return {"x": x, "y": y, "yaw": yaw, "age_sec": age}


def is_api_yaw_active(goal):
    phase = str(goal.get("phase", ""))
    return bool(
        goal.get("yaw_align_active")
        or goal.get("ordinary_final_yaw_align_active")
        or "final_yaw_align" in phase
        or phase == "position_reached_yaw_aligning"
    )


def is_nav2_terminal_phase(goal):
    phase = str(goal.get("phase", ""))
    if not goal:
        return False
    if goal.get("state") != "running":
        return False
    if is_api_yaw_active(goal):
        return False
    if phase in ("accepted", "sending_nav2_goal", "waiting_for_nav2_result"):
        return True
    return "near_goal_nav2" in phase or phase.endswith("_near_goal_watch")


class TerminalAdjustmentObserver(Node):
    def __init__(self):
        super().__init__("navigation_terminal_adjustment_observer")
        self.started_wall = time.time()
        self.samples = []
        self.phase_counts = Counter()
        self.state_counts = Counter()
        self.events = []
        self.api_errors = Counter()
        self.action_status = {}
        self.last_action_key = {}
        self.string_status = {}
        self.last_string_data = {}
        self.rosout_tail = []
        self.rosout_count = 0
        self.speed_limit = {
            "count": 0,
            "last": None,
            "min": None,
            "max": None,
            "changes": [],
        }
        self.cmd = {}
        self.cmd_near_samples = defaultdict(int)
        self.cmd_near_tiny_samples = defaultdict(int)
        self.cmd_near_zero_samples = defaultdict(int)
        self.cmd_near_max_abs_vx = defaultdict(float)
        self.cmd_near_max_abs_wz = defaultdict(float)

        self.first_active_elapsed = None
        self.first_near_goal_elapsed = None
        self.first_very_near_elapsed = None
        self.first_nav2_terminal_slow_elapsed = None
        self.first_api_yaw_elapsed = None
        self.first_task_complete_elapsed = None
        self.goal_terminal_elapsed = None
        self.has_observed_goal = False
        self.observed_goal_id = None
        self.observed_running_goal = False
        self.ignored_pre_active_samples = 0
        self.ignored_other_goal_samples = 0
        self.latest_observed_goal_sample = None
        self.nav2_near_goal_elapsed = 0.0
        self.nav2_near_goal_tiny_cmd_elapsed = 0.0
        self.nav2_very_near_elapsed = 0.0
        self.last_sample_elapsed = None

        self.samples_file = (output_dir / "samples.jsonl").open("a", encoding="utf-8")
        self.events_file = (output_dir / "events.jsonl").open("a", encoding="utf-8")
        self.cmd_file = (output_dir / "cmd_frames.jsonl").open("a", encoding="utf-8")
        self.rosout_file = (output_dir / "rosout_filtered.log").open("a", encoding="utf-8")

        qos = QoSProfile(depth=20)
        rosout_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=100,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )

        for topic in (
            "/navigate_to_pose/_action/status",
            "/follow_path/_action/status",
            "/compute_path_to_pose/_action/status",
        ):
            self.create_subscription(
                GoalStatusArray,
                topic,
                lambda msg, topic=topic: self.on_action_status(topic, msg),
                qos,
            )

        for topic in (
            "/localization/bridge_status",
            "/safety/status",
            "/ranger_mini3_mode_controller/status",
        ):
            self.create_subscription(String, topic, lambda msg, topic=topic: self.on_string(topic, msg), qos)

        for topic in CMD_TOPICS:
            self.cmd[topic] = {
                "count": 0,
                "last": None,
                "last_msg_elapsed": None,
                "nonzero_count": 0,
                "max_abs_vx": 0.0,
                "max_abs_vy": 0.0,
                "max_abs_wz": 0.0,
            }
            self.create_subscription(Twist, topic, lambda msg, topic=topic: self.on_twist(topic, msg), qos)

        self.create_subscription(SpeedLimit, "/speed_limit", self.on_speed_limit, qos)
        if include_rosout:
            self.create_subscription(Log, "/rosout", self.on_rosout, rosout_qos)

    def elapsed(self):
        return time.time() - self.started_wall

    def emit_event(self, kind, payload):
        row = {
            "elapsed_sec": round(self.elapsed(), 3),
            "observed_at": now_iso(),
            "kind": kind,
            "payload": payload,
        }
        self.events.append(row)
        self.events_file.write(json.dumps(row, ensure_ascii=True, sort_keys=True) + "\n")
        self.events_file.flush()

    def on_action_status(self, topic, msg):
        statuses = []
        for item in msg.status_list:
            status = int(item.status)
            statuses.append(
                {
                    "goal_id": short_uuid(item.goal_info.goal_id),
                    "status": status,
                    "status_name": STATUS_NAMES.get(status, str(status)),
                }
            )
        key = tuple((entry["goal_id"], entry["status"]) for entry in statuses)
        self.action_status[topic] = statuses
        if self.last_action_key.get(topic) != key:
            self.last_action_key[topic] = key
            self.emit_event("action_status_changed", {"topic": topic, "statuses": statuses})

    def on_string(self, topic, msg):
        data = msg.data
        self.string_status[topic] = data
        if self.last_string_data.get(topic) != data:
            self.last_string_data[topic] = data
            payload = {"topic": topic, "data": data[:600]}
            try:
                parsed = json.loads(data)
                if topic == "/localization/bridge_status":
                    payload["bridge"] = {
                        key: parsed.get(key)
                        for key in (
                            "safe_for_goal_start",
                            "correction_active",
                            "localization_degraded",
                            "amcl_correction_pending",
                            "remaining_translation_error_m",
                            "remaining_yaw_error_rad",
                            "last_accept_reason",
                            "last_reject_reason",
                        )
                    }
            except Exception:
                pass
            self.emit_event("string_status_changed", payload)

    def on_twist(self, topic, msg):
        twist = twist_dict(msg)
        stats = self.cmd[topic]
        stats["count"] += 1
        stats["last"] = twist
        stats["last_msg_elapsed"] = round(self.elapsed(), 3)
        stats["max_abs_vx"] = max(stats["max_abs_vx"], abs(twist["linear_x"]))
        stats["max_abs_vy"] = max(stats["max_abs_vy"], abs(twist["linear_y"]))
        stats["max_abs_wz"] = max(stats["max_abs_wz"], abs(twist["angular_z"]))
        if not twist_is_zero(twist):
            stats["nonzero_count"] += 1
        self.cmd_file.write(
            json.dumps(
                {
                    "elapsed_sec": round(self.elapsed(), 3),
                    "observed_at": now_iso(),
                    "topic": topic,
                    "twist": twist,
                },
                ensure_ascii=True,
                sort_keys=True,
            )
            + "\n"
        )
        if stats["count"] % 50 == 0:
            self.cmd_file.flush()

    def on_speed_limit(self, msg):
        value = float(msg.speed_limit)
        self.speed_limit["count"] += 1
        self.speed_limit["last"] = {
            "elapsed_sec": round(self.elapsed(), 3),
            "speed_limit": value,
            "percentage": bool(msg.percentage),
        }
        self.speed_limit["min"] = value if self.speed_limit["min"] is None else min(self.speed_limit["min"], value)
        self.speed_limit["max"] = value if self.speed_limit["max"] is None else max(self.speed_limit["max"], value)
        changes = self.speed_limit["changes"]
        if not changes or abs(float(changes[-1]["speed_limit"]) - value) > 1.0e-6:
            changes.append(self.speed_limit["last"])
            self.emit_event("speed_limit_changed", self.speed_limit["last"])

    def on_rosout(self, msg):
        text = msg.msg or ""
        if not ROSOUT_FILTER.search(text):
            return
        row = {
            "elapsed_sec": round(self.elapsed(), 3),
            "observed_at": now_iso(),
            "level": int(msg.level),
            "name": msg.name,
            "msg": text,
        }
        self.rosout_count += 1
        self.rosout_tail.append(row)
        self.rosout_tail = self.rosout_tail[-60:]
        self.rosout_file.write(json.dumps(row, ensure_ascii=True, sort_keys=True) + "\n")
        self.rosout_file.flush()

    def sample(self):
        elapsed = self.elapsed()
        dt = 0.0 if self.last_sample_elapsed is None else max(0.0, elapsed - self.last_sample_elapsed)
        self.last_sample_elapsed = elapsed

        nav_state, nav_err = api_get_json("/api/v1/navigation/state")
        pose_payload, pose_err = api_get_json("/api/v1/robot/pose")
        if nav_err:
            self.api_errors[f"navigation_state:{nav_err}"] += 1
        if pose_err:
            self.api_errors[f"robot_pose:{pose_err}"] += 1

        goal = active_goal(nav_state)
        target = target_from_goal(goal)
        pose = pose_from_payload(pose_payload)
        distance = None
        yaw_error = None
        if target and pose:
            distance = math.hypot(target["x"] - pose["x"], target["y"] - pose["y"])
            yaw_error = normalize_angle(target["yaw"] - pose["yaw"])

        phase = str(goal.get("phase", ""))
        state = str(goal.get("state", ""))
        goal_id = goal.get("id")
        if state == "running" and not self.has_observed_goal:
            self.has_observed_goal = True
            self.observed_goal_id = goal_id
            self.observed_running_goal = True
            self.first_active_elapsed = elapsed
            self.emit_event(
                "navigation_goal_active",
                {"goal_id": goal_id, "phase": phase, "pose_id": goal.get("pose_id")},
            )
        current_observed_goal = self.has_observed_goal and goal_id == self.observed_goal_id
        if current_observed_goal:
            self.phase_counts[phase] += 1
            self.state_counts[state] += 1
        elif not self.has_observed_goal:
            self.ignored_pre_active_samples += 1
        else:
            self.ignored_other_goal_samples += 1

        nav2_terminal = is_nav2_terminal_phase(goal)
        api_yaw_active = is_api_yaw_active(goal)
        near_goal = distance is not None and distance <= near_goal_m
        very_near = distance is not None and distance <= very_near_m
        nav_raw = self.cmd.get("/cmd_vel_nav_raw", {}).get("last")
        nav_out = self.cmd.get("/cmd_vel_nav", {}).get("last")
        safe_out = self.cmd.get("/cmd_vel_safe", {}).get("last")
        api_cmd = self.cmd.get("/cmd_vel_api", {}).get("last")
        nav_raw_tiny = twist_is_tiny(nav_raw)
        nav_out_tiny = twist_is_tiny(nav_out)
        safe_tiny = twist_is_tiny(safe_out)
        api_cmd_nonzero = api_cmd is not None and not twist_is_zero(api_cmd)

        if current_observed_goal and state == "running":
            self.observed_running_goal = True
        if current_observed_goal and near_goal and self.first_near_goal_elapsed is None:
            self.first_near_goal_elapsed = elapsed
            self.emit_event("entered_near_goal_window", {"distance_m": distance, "phase": phase})
        if current_observed_goal and very_near and self.first_very_near_elapsed is None:
            self.first_very_near_elapsed = elapsed
            self.emit_event("entered_very_near_goal_window", {"distance_m": distance, "phase": phase})
        if current_observed_goal and api_yaw_active and self.first_api_yaw_elapsed is None:
            self.first_api_yaw_elapsed = elapsed
            self.emit_event("api_final_yaw_active", {"phase": phase, "yaw_error_rad": yaw_error})
        if current_observed_goal and goal.get("task_complete") is True and self.first_task_complete_elapsed is None:
            self.first_task_complete_elapsed = elapsed
            self.emit_event("task_complete", {"phase": phase, "distance_m": distance, "yaw_error_rad": yaw_error})
        if (
            current_observed_goal
            and self.goal_terminal_elapsed is None
            and state in ("succeeded", "failed", "canceled")
        ):
            self.goal_terminal_elapsed = elapsed
            self.emit_event(
                "navigation_goal_terminal",
                {"state": state, "phase": phase, "distance_m": distance, "yaw_error_rad": yaw_error},
            )

        if current_observed_goal and near_goal and nav2_terminal:
            self.nav2_near_goal_elapsed += dt
            if nav_raw_tiny:
                self.nav2_near_goal_tiny_cmd_elapsed += dt
                if self.first_nav2_terminal_slow_elapsed is None:
                    self.first_nav2_terminal_slow_elapsed = elapsed
                    self.emit_event(
                        "nav2_terminal_tiny_cmd_started",
                        {
                            "distance_m": distance,
                            "phase": phase,
                            "cmd_vel_nav_raw": nav_raw,
                            "speed_limit": self.speed_limit.get("last"),
                        },
                    )
        if current_observed_goal and very_near and nav2_terminal:
            self.nav2_very_near_elapsed += dt

        if current_observed_goal and near_goal:
            for topic, stats in self.cmd.items():
                twist = stats.get("last")
                self.cmd_near_samples[topic] += 1
                if twist_is_tiny(twist):
                    self.cmd_near_tiny_samples[topic] += 1
                if twist_is_zero(twist):
                    self.cmd_near_zero_samples[topic] += 1
                if twist:
                    self.cmd_near_max_abs_vx[topic] = max(
                        self.cmd_near_max_abs_vx[topic],
                        abs(float(twist.get("linear_x", 0.0))),
                    )
                    self.cmd_near_max_abs_wz[topic] = max(
                        self.cmd_near_max_abs_wz[topic],
                        abs(float(twist.get("angular_z", 0.0))),
                    )

        row = {
            "elapsed_sec": round(elapsed, 3),
            "observed_at": now_iso(),
            "api_ok": nav_state is not None,
            "api_errors": {"navigation_state": nav_err, "robot_pose": pose_err},
            "navigation_active": (nav_state or {}).get("navigation_active"),
            "goal": {
                "id": goal.get("id"),
                "state": state,
                "phase": phase,
                "pose_id": goal.get("pose_id"),
                "detail": goal.get("detail"),
                "nav2_succeeded": goal.get("nav2_succeeded"),
                "nav2_result_code": goal.get("nav2_result_code"),
                "position_reached": goal.get("position_reached"),
                "yaw_align_active": goal.get("yaw_align_active"),
                "ordinary_final_yaw_align_active": goal.get("ordinary_final_yaw_align_active"),
                "final_pose_verified": goal.get("final_pose_verified"),
                "task_complete": goal.get("task_complete"),
                "final_distance_m": goal.get("final_distance_m"),
                "final_yaw_error_rad": goal.get("final_yaw_error_rad"),
                "final_verify_retry_count": goal.get("final_verify_retry_count"),
                "reposition_after_yaw_drift_retry_count": goal.get("reposition_after_yaw_drift_retry_count"),
                "target": target,
            },
            "pose": pose,
            "computed_distance_m": distance,
            "computed_yaw_error_rad": yaw_error,
            "computed_abs_yaw_error_deg": None if yaw_error is None else abs(yaw_error) * 180.0 / math.pi,
            "classification": {
                "near_goal": near_goal,
                "very_near": very_near,
                "nav2_terminal_phase": nav2_terminal,
                "api_yaw_active": api_yaw_active,
                "current_observed_goal": current_observed_goal,
                "nav_raw_tiny": nav_raw_tiny,
                "nav_out_tiny": nav_out_tiny,
                "safe_tiny": safe_tiny,
                "api_cmd_nonzero": api_cmd_nonzero,
            },
            "observer": {
                "has_observed_goal": self.has_observed_goal,
                "observed_goal_id": self.observed_goal_id,
                "current_observed_goal": current_observed_goal,
                "ignored_pre_active_samples": self.ignored_pre_active_samples,
                "ignored_other_goal_samples": self.ignored_other_goal_samples,
            },
            "speed_limit": self.speed_limit.get("last"),
            "cmd_last": {
                topic: self.cmd.get(topic, {}).get("last")
                for topic in CMD_TOPICS
            },
            "action_status": self.action_status,
        }
        self.samples.append(row)
        if current_observed_goal:
            self.latest_observed_goal_sample = row
        self.samples_file.write(json.dumps(row, ensure_ascii=True, sort_keys=True) + "\n")
        self.samples_file.flush()

    def summary(self):
        latest = self.latest_observed_goal_sample or (self.samples[-1] if self.samples else {})
        goal = latest.get("goal") or {}
        hints = []

        if not self.has_observed_goal:
            hints.append("no_running_goal_observed_after_observer_start")
        elif self.first_near_goal_elapsed is None:
            hints.append("never_entered_near_goal_window_or_target_pose_unavailable")
        if self.nav2_near_goal_tiny_cmd_elapsed >= 30.0:
            hints.append("nav2_spent_long_time_near_goal_with_tiny_cmd_vel_nav_raw")
        if self.nav2_near_goal_tiny_cmd_elapsed >= 60.0:
            hints.append("terminal_adjustment_delay_is_nav2_mppi_side_before_api_yaw")
        if self.first_api_yaw_elapsed is not None and self.first_near_goal_elapsed is not None:
            delay = self.first_api_yaw_elapsed - self.first_near_goal_elapsed
            if delay >= 30.0:
                hints.append(f"api_final_yaw_started_after_{delay:.1f}s_in_near_goal_window")
        if self.speed_limit.get("min") is not None and float(self.speed_limit["min"]) <= 0.11:
            hints.append("speed_limit_reached_0p10mps_terminal_crawl")
        nav_near = self.cmd_near_samples.get("/cmd_vel_nav_raw", 0)
        safe_near = self.cmd_near_samples.get("/cmd_vel_safe", 0)
        if nav_near and safe_near:
            nav_tiny = self.cmd_near_tiny_samples["/cmd_vel_nav_raw"] / max(1, nav_near)
            safe_tiny_ratio = self.cmd_near_tiny_samples["/cmd_vel_safe"] / max(1, safe_near)
            if nav_tiny < 0.4 and safe_tiny_ratio > 0.8:
                hints.append("nav2_commands_exist_but_downstream_safety_or_collision_output_is_tiny")
        if self.cmd["/cmd_vel_api"]["nonzero_count"] > 0:
            hints.append("api_cmd_vel_api_was_used_for_terminal_correction")

        cmd_summary = {}
        for topic, stats in self.cmd.items():
            near_count = self.cmd_near_samples.get(topic, 0)
            cmd_summary[topic] = {
                "total_count": stats["count"],
                "nonzero_count": stats["nonzero_count"],
                "max_abs_vx": round(stats["max_abs_vx"], 4),
                "max_abs_vy": round(stats["max_abs_vy"], 4),
                "max_abs_wz": round(stats["max_abs_wz"], 4),
                "near_goal_samples": near_count,
                "near_goal_tiny_ratio": (
                    None if near_count == 0 else round(self.cmd_near_tiny_samples[topic] / near_count, 3)
                ),
                "near_goal_zero_ratio": (
                    None if near_count == 0 else round(self.cmd_near_zero_samples[topic] / near_count, 3)
                ),
                "near_goal_max_abs_vx": round(self.cmd_near_max_abs_vx[topic], 4),
                "near_goal_max_abs_wz": round(self.cmd_near_max_abs_wz[topic], 4),
            }

        data = {
            "report_dir": str(output_dir),
            "samples": len(self.samples),
            "duration_observed_sec": round(self.elapsed(), 3),
            "observed_goal_id": self.observed_goal_id,
            "ignored_pre_active_samples": self.ignored_pre_active_samples,
            "ignored_other_goal_samples": self.ignored_other_goal_samples,
            "latest_goal": goal,
            "phase_counts": dict(self.phase_counts),
            "state_counts": dict(self.state_counts),
            "first_active_elapsed_sec": self.first_active_elapsed,
            "first_near_goal_elapsed_sec": self.first_near_goal_elapsed,
            "first_very_near_elapsed_sec": self.first_very_near_elapsed,
            "first_nav2_terminal_slow_elapsed_sec": self.first_nav2_terminal_slow_elapsed,
            "first_api_yaw_elapsed_sec": self.first_api_yaw_elapsed,
            "first_task_complete_elapsed_sec": self.first_task_complete_elapsed,
            "goal_terminal_elapsed_sec": self.goal_terminal_elapsed,
            "stop_when_terminal": stop_when_terminal,
            "near_goal_to_api_yaw_delay_sec": (
                None
                if self.first_near_goal_elapsed is None or self.first_api_yaw_elapsed is None
                else round(self.first_api_yaw_elapsed - self.first_near_goal_elapsed, 3)
            ),
            "nav2_near_goal_elapsed_sec": round(self.nav2_near_goal_elapsed, 3),
            "nav2_near_goal_tiny_cmd_elapsed_sec": round(self.nav2_near_goal_tiny_cmd_elapsed, 3),
            "nav2_very_near_elapsed_sec": round(self.nav2_very_near_elapsed, 3),
            "speed_limit": self.speed_limit,
            "cmd_summary": cmd_summary,
            "api_errors": dict(self.api_errors),
            "events_tail": self.events[-80:],
            "rosout_count": self.rosout_count,
            "rosout_tail": self.rosout_tail[-40:],
            "diagnosis_hints": hints,
        }
        return data

    def write_outputs(self):
        for handle in (self.samples_file, self.events_file, self.cmd_file, self.rosout_file):
            try:
                handle.flush()
            except Exception:
                pass
        data = self.summary()
        (output_dir / "summary.json").write_text(json.dumps(data, indent=2, sort_keys=True), encoding="utf-8")

        lines = [
            "# Navigation Terminal Adjustment Observation",
            "",
            f"- report_dir: `{output_dir}`",
            f"- samples: `{data['samples']}`",
            f"- observed_goal_id: `{data['observed_goal_id']}`",
            f"- ignored_pre_active_samples: `{data['ignored_pre_active_samples']}`",
            f"- ignored_other_goal_samples: `{data['ignored_other_goal_samples']}`",
            f"- latest_goal_state: `{data['latest_goal'].get('state', '')}`",
            f"- latest_goal_phase: `{data['latest_goal'].get('phase', '')}`",
            f"- latest_goal_detail: `{data['latest_goal'].get('detail', '')}`",
            f"- latest_pose_id: `{data['latest_goal'].get('pose_id', '')}`",
            f"- first_near_goal_elapsed_sec: `{data['first_near_goal_elapsed_sec']}`",
            f"- first_very_near_elapsed_sec: `{data['first_very_near_elapsed_sec']}`",
            f"- first_nav2_terminal_slow_elapsed_sec: `{data['first_nav2_terminal_slow_elapsed_sec']}`",
            f"- first_api_yaw_elapsed_sec: `{data['first_api_yaw_elapsed_sec']}`",
            f"- first_task_complete_elapsed_sec: `{data['first_task_complete_elapsed_sec']}`",
            f"- goal_terminal_elapsed_sec: `{data['goal_terminal_elapsed_sec']}`",
            f"- near_goal_to_api_yaw_delay_sec: `{data['near_goal_to_api_yaw_delay_sec']}`",
            f"- nav2_near_goal_elapsed_sec: `{data['nav2_near_goal_elapsed_sec']}`",
            f"- nav2_near_goal_tiny_cmd_elapsed_sec: `{data['nav2_near_goal_tiny_cmd_elapsed_sec']}`",
            f"- nav2_very_near_elapsed_sec: `{data['nav2_very_near_elapsed_sec']}`",
            f"- speed_limit_min: `{data['speed_limit'].get('min')}`",
            f"- speed_limit_max: `{data['speed_limit'].get('max')}`",
            f"- speed_limit_changes: `{data['speed_limit'].get('changes')}`",
            f"- phase_counts: `{data['phase_counts']}`",
            f"- state_counts: `{data['state_counts']}`",
            f"- diagnosis_hints: `{data['diagnosis_hints']}`",
            "",
            "## Command Summary",
        ]
        for topic, stats in data["cmd_summary"].items():
            lines.append(
                "- "
                f"{topic}: total=`{stats['total_count']}` nonzero=`{stats['nonzero_count']}` "
                f"max_vx=`{stats['max_abs_vx']}` max_wz=`{stats['max_abs_wz']}` "
                f"near_tiny_ratio=`{stats['near_goal_tiny_ratio']}` "
                f"near_zero_ratio=`{stats['near_goal_zero_ratio']}` "
                f"near_max_vx=`{stats['near_goal_max_abs_vx']}` "
                f"near_max_wz=`{stats['near_goal_max_abs_wz']}`"
            )
        lines.extend(
            [
                "",
                "## Raw Files",
                "- `samples.jsonl`: API state, robot pose, computed target distance/yaw error, command snapshots.",
                "- `cmd_frames.jsonl`: every observed Twist command on the command chain.",
                "- `events.jsonl`: phase/status/speed-limit changes.",
                "- `rosout_filtered.log`: filtered Nav2/controller/localization log lines.",
                "- `summary.json`: machine-readable summary.",
            ]
        )
        (output_dir / "summary.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
        for handle in (self.samples_file, self.events_file, self.cmd_file, self.rosout_file):
            try:
                handle.close()
            except Exception:
                pass


rclpy.init()
observer = TerminalAdjustmentObserver()
deadline = time.time() + duration_sec
next_sample = time.time()
interrupted = False
try:
    while rclpy.ok() and time.time() < deadline:
        rclpy.spin_once(observer, timeout_sec=0.05)
        now = time.time()
        if now >= next_sample:
            observer.sample()
            next_sample = now + sample_period_sec
            if (
                stop_when_terminal
                and observer.goal_terminal_elapsed is not None
                and observer.elapsed() - observer.goal_terminal_elapsed >= 1.0
            ):
                observer.emit_event(
                    "capture_stopped_after_terminal_goal",
                    {"terminal_elapsed_sec": observer.goal_terminal_elapsed},
                )
                break
except KeyboardInterrupt:
    interrupted = True
    try:
        observer.emit_event("capture_interrupted", {"reason": "keyboard_interrupt"})
    except Exception:
        pass
finally:
    try:
        observer.write_outputs()
    except Exception as exc:
        print(f"failed to write summary: {exc}", file=sys.stderr)
        raise
    try:
        observer.destroy_node()
    except Exception:
        pass
    try:
        if rclpy.ok():
            rclpy.shutdown()
    except Exception:
        pass

print(f"{output_dir}/summary.md")
if interrupted:
    print("capture interrupted by Ctrl+C after writing summary")
PY

rc=$?
if [[ "${rc}" -ne 0 ]]; then
  echo "${PREFIX} FAIL capture exited with rc=${rc}" >&2
  exit "${rc}"
fi

echo "${PREFIX} summary: ${OUTPUT_DIR}/summary.md"
echo "${PREFIX} complete: ${OUTPUT_DIR}"
