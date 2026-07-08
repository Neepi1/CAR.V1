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
DURATION_SEC=120
SAMPLE_PERIOD_SEC=0.25
LABEL="amcl_odom_correlation"
OUTPUT_DIR=""
INCLUDE_ROSOUT=true
STOP_WHEN_TERMINAL=false
PREFIX="[nav-amcl-odom]"

usage() {
  cat <<'EOF'
Usage:
  bash scripts/jetson/runtime_overlay/scripts/record_navigation_amcl_odom_correlation.sh \
    --duration-sec 120 \
    --label delivery_512355_amcl_odom

Start this script first, then send one normal navigation goal from the App.

Read-only observer:
  - does not send goals, publish velocity, call services, set params, or restart nodes
  - subscribes to /localization/bridge_status, /motion_state, wheel/local odom, cmd_vel chain, /speed_limit, Nav2 action status, and TF
  - polls /api/v1/navigation/state and /api/v1/robot/pose
  - correlates AMCL accepted/rejected deltas with navigation phase, command shape, chassis feedback, and map/odom pose

Options:
  --duration-sec N          Capture duration in seconds. Default: 120.
  --sample-period-sec N     CSV sample period in seconds. Default: 0.25.
  --label LABEL             Report label. Default: amcl_odom_correlation.
  --api-url URL             robot_api_server URL. Default: http://127.0.0.1:8080.
  --output-dir DIR          Report directory. Default: reports/navigation_amcl_odom_correlation/<timestamp>_<label>_<duration>s.
  --no-rosout               Do not subscribe to filtered /rosout.
  --stop-when-terminal      Stop after this observer sees a running goal reach a terminal state.
  -h, --help                Show this help.
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

if ! [[ "${DURATION_SEC}" =~ ^[0-9]+$ ]] || [[ "${DURATION_SEC}" -lt 10 ]]; then
  echo "${PREFIX} FAIL --duration-sec must be an integer >= 10" >&2
  exit 2
fi

if ! is_number "${SAMPLE_PERIOD_SEC}"; then
  echo "${PREFIX} FAIL --sample-period-sec must be numeric" >&2
  exit 2
fi

TIMESTAMP="$(date -u +%Y%m%dT%H%M%SZ)"
LABEL="$(sanitize_label "${LABEL}")"
if [[ -z "${OUTPUT_DIR}" ]]; then
  PREFERRED_OUTPUT_ROOT="${NJRH_PROJECT_ROOT}/reports/navigation_amcl_odom_correlation"
  FALLBACK_OUTPUT_ROOT="${TMPDIR:-/tmp}/njrh_reports/navigation_amcl_odom_correlation"
  if mkdir -p "${PREFERRED_OUTPUT_ROOT}" 2>/dev/null && [[ -w "${PREFERRED_OUTPUT_ROOT}" ]]; then
    OUTPUT_DIR="${PREFERRED_OUTPUT_ROOT}/${TIMESTAMP}_${LABEL}_${DURATION_SEC}s"
  else
    echo "${PREFIX} WARN preferred report root is not writable: ${PREFERRED_OUTPUT_ROOT}" >&2
    echo "${PREFIX} WARN falling back to ${FALLBACK_OUTPUT_ROOT}" >&2
    OUTPUT_DIR="${FALLBACK_OUTPUT_ROOT}/${TIMESTAMP}_${LABEL}_${DURATION_SEC}s"
  fi
fi
if ! mkdir -p "${OUTPUT_DIR}" 2>/dev/null; then
  echo "${PREFIX} FAIL cannot create output directory: ${OUTPUT_DIR}" >&2
  echo "${PREFIX} hint: pass --output-dir /tmp/<name> or fix reports directory ownership" >&2
  exit 1
fi
if [[ ! -w "${OUTPUT_DIR}" ]]; then
  echo "${PREFIX} FAIL output directory is not writable: ${OUTPUT_DIR}" >&2
  echo "${PREFIX} hint: pass --output-dir /tmp/<name> or fix reports directory ownership" >&2
  exit 1
fi

{
  echo "timestamp_utc=${TIMESTAMP}"
  echo "duration_sec=${DURATION_SEC}"
  echo "sample_period_sec=${SAMPLE_PERIOD_SEC}"
  echo "label=${LABEL}"
  echo "api_url=${API_URL}"
  echo "include_rosout=${INCLUDE_ROSOUT}"
  echo "stop_when_terminal=${STOP_WHEN_TERMINAL}"
  echo "workspace_root=${WORKSPACE_ROOT}"
  echo "read_only=true"
  echo "sends_navigation_goals=false"
  echo "publishes_velocity=false"
  echo "calls_services=false"
  echo "sets_params=false"
} >"${OUTPUT_DIR}/metadata.env"

echo "${PREFIX} report_dir=${OUTPUT_DIR}"
echo "${PREFIX} duration_sec=${DURATION_SEC} sample_period_sec=${SAMPLE_PERIOD_SEC}"
echo "${PREFIX} read-only: start the App navigation goal now"

python3 - \
  "${DURATION_SEC}" \
  "${SAMPLE_PERIOD_SEC}" \
  "${API_URL}" \
  "${OUTPUT_DIR}" \
  "${INCLUDE_ROSOUT}" \
  "${STOP_WHEN_TERMINAL}" <<'PY'
import csv
import json
import math
import re
import sys
import time
import urllib.error
import urllib.request
from collections import Counter
from pathlib import Path

import rclpy
from action_msgs.msg import GoalStatusArray
from geometry_msgs.msg import Twist
from nav2_msgs.msg import SpeedLimit
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import Log
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from std_msgs.msg import String
import tf2_ros

try:
    from ranger_msgs.msg import MotionState, SystemState
except Exception:
    MotionState = None
    SystemState = None


duration_sec = float(sys.argv[1])
sample_period_sec = float(sys.argv[2])
api_url = sys.argv[3].rstrip("/")
output_dir = Path(sys.argv[4])
include_rosout = sys.argv[5].lower() == "true"
stop_when_terminal = sys.argv[6].lower() == "true"
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

TERMINAL_GOAL_STATES = {"succeeded", "failed", "canceled", "degraded", "rejected"}

ROSOUT_FILTER = re.compile(
    r"robot_localization_bridge|amcl|localization|controller_server|bt_navigator|"
    r"follow_path|navigate|mppi|rotation.?shim|speed.?limit|cmd_vel|goal|abort|"
    r"failed|progress|collision|local_costmap|transform",
    re.IGNORECASE,
)


def safe_float(value, default=None):
    try:
        result = float(value)
    except Exception:
        return default
    if not math.isfinite(result):
        return default
    return result


def norm_angle(value):
    while value > math.pi:
        value -= 2.0 * math.pi
    while value < -math.pi:
        value += 2.0 * math.pi
    return value


def yaw_from_quat(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def pose_from_odom(msg):
    p = msg.pose.pose.position
    q = msg.pose.pose.orientation
    return {"x": p.x, "y": p.y, "yaw": yaw_from_quat(q)}


def twist_from_odom(msg):
    twist = msg.twist.twist
    return {
        "linear_x": twist.linear.x,
        "linear_y": twist.linear.y,
        "angular_z": twist.angular.z,
        "speed": math.hypot(twist.linear.x, twist.linear.y),
    }


def pose_from_tf(transform):
    p = transform.transform.translation
    q = transform.transform.rotation
    return {"x": p.x, "y": p.y, "yaw": yaw_from_quat(q)}


def pose_delta(current, origin):
    if current is None or origin is None:
        return {"dx": "", "dy": "", "dist": "", "dyaw": ""}
    dx = current["x"] - origin["x"]
    dy = current["y"] - origin["y"]
    return {
        "dx": dx,
        "dy": dy,
        "dist": math.hypot(dx, dy),
        "dyaw": norm_angle(current["yaw"] - origin["yaw"]),
    }


def twist_values(msg):
    if msg is None:
        return {"x": "", "y": "", "z": "", "speed": ""}
    return {
        "x": msg.linear.x,
        "y": msg.linear.y,
        "z": msg.angular.z,
        "speed": math.hypot(msg.linear.x, msg.linear.y),
    }


def motion_value(msg, name):
    if msg is None:
        return ""
    value = getattr(msg, name, "")
    if hasattr(value, "data"):
        value = value.data
    return value


def get_json(url, timeout=0.35):
    try:
        with urllib.request.urlopen(url, timeout=timeout) as resp:
            return json.loads(resp.read().decode("utf-8", errors="replace"))
    except Exception as exc:
        return {"_error": repr(exc)}


def fmt(value):
    if value is None:
        return ""
    if isinstance(value, bool):
        return "true" if value else "false"
    if isinstance(value, float):
        if not math.isfinite(value):
            return ""
        return f"{value:.6f}"
    return value


def stats(values):
    vals = sorted(v for v in values if v is not None and math.isfinite(v))
    if not vals:
        return {"count": 0, "min": None, "mean": None, "p50": None, "p95": None, "max": None}

    def pct(p):
        if len(vals) == 1:
            return vals[0]
        k = (len(vals) - 1) * p / 100.0
        f = math.floor(k)
        c = math.ceil(k)
        if f == c:
            return vals[int(k)]
        return vals[f] * (c - k) + vals[c] * (k - f)

    return {
        "count": len(vals),
        "min": vals[0],
        "mean": sum(vals) / len(vals),
        "p50": pct(50),
        "p95": pct(95),
        "max": vals[-1],
    }


class Probe(Node):
    def __init__(self):
        super().__init__("record_navigation_amcl_odom_correlation")
        self.start_wall = time.time()
        self.last_api_nav = {}
        self.last_api_pose = {}
        self.last_bridge = {}
        self.first_bridge = None
        self.last_goal_status = ""
        self.goal_was_running = False
        self.terminal_seen_wall = None
        self.latest_twist = {}
        self.latest_odom = {}
        self.latest_odom_twist = {}
        self.odom_origin = {}
        self.latest_speed_limit = None
        self.latest_motion_state = None
        self.latest_system_state = None
        self.bridge_events = []
        self.rosout_rows = 0
        self.accepted_event_translations = []
        self.rejected_event_translations = []
        self.candidate_translations = []
        self.last_counts = None

        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=15.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_origin = {}

        qos = QoSProfile(depth=80)
        be = QoSProfile(depth=120)
        be.reliability = ReliabilityPolicy.BEST_EFFORT

        for topic in (
            "/cmd_vel_nav_raw",
            "/cmd_vel_nav",
            "/cmd_vel_collision_checked",
            "/cmd_vel_api",
            "/cmd_vel",
        ):
            self.create_subscription(Twist, topic, lambda msg, t=topic: self.on_twist(t, msg), qos)

        self.create_subscription(SpeedLimit, "/speed_limit", self.on_speed_limit, qos)
        self.create_subscription(String, "/localization/bridge_status", self.on_bridge, qos)
        self.create_subscription(GoalStatusArray, "/navigate_to_pose/_action/status", self.on_nav_status, qos)
        self.create_subscription(Odometry, "/wheel/odom", lambda msg: self.on_odom("wheel", msg), be)
        self.create_subscription(Odometry, "/wheel/odom_ekf", lambda msg: self.on_odom("wheel_ekf", msg), be)
        self.create_subscription(Odometry, "/local_state/odometry", lambda msg: self.on_odom("local", msg), be)
        if MotionState is not None:
            self.create_subscription(MotionState, "/motion_state", self.on_motion_state, qos)
        if SystemState is not None:
            self.create_subscription(SystemState, "/system_state", self.on_system_state, qos)
        if include_rosout:
            self.create_subscription(Log, "/rosout", self.on_rosout, 100)

        self.samples_file = (output_dir / "samples.csv").open("w", newline="", encoding="utf-8")
        self.samples_writer = csv.DictWriter(self.samples_file, fieldnames=self.sample_fields())
        self.samples_writer.writeheader()

        self.events_file = (output_dir / "amcl_bridge_events.csv").open("w", newline="", encoding="utf-8")
        self.events_writer = csv.DictWriter(self.events_file, fieldnames=self.event_fields())
        self.events_writer.writeheader()

        self.api_file = (output_dir / "api_state_samples.jsonl").open("w", encoding="utf-8")
        self.rosout_file = (output_dir / "rosout_filtered.csv").open("w", newline="", encoding="utf-8")
        self.rosout_writer = csv.DictWriter(
            self.rosout_file,
            fieldnames=["rel_time_sec", "level", "node", "message"],
        )
        self.rosout_writer.writeheader()

    def rel(self):
        return time.time() - self.start_wall

    def sample_fields(self):
        fields = [
            "rel_time_sec",
            "goal_id",
            "goal_state",
            "goal_phase",
            "pose_id",
            "final_distance_m",
            "final_yaw_error_rad",
            "nav2_result_code",
            "nav2_status",
            "speed_limit_mps",
            "amcl_candidate_count",
            "amcl_accepted_count",
            "amcl_rejected_count",
            "amcl_last_state",
            "last_accept_reason",
            "last_reject_reason",
            "last_candidate_translation_m",
            "last_candidate_yaw_rad",
            "last_accepted_translation_m",
            "last_accepted_yaw_rad",
            "last_accepted_source",
            "active_correction_source",
            "correction_active",
            "amcl_robot_moving",
            "amcl_linear_speed_mps",
            "amcl_angular_speed_radps",
            "safe_for_goal_start",
            "localization_degraded",
            "motion_mode",
            "motion_linear_velocity",
            "motion_lateral_velocity",
            "motion_angular_velocity",
            "motion_steering_angle",
            "system_vehicle_state",
            "system_control_mode",
            "system_motion_mode",
            "system_error_code",
            "system_battery_voltage",
        ]
        for prefix in ("cmd_vel_nav_raw", "cmd_vel_nav", "cmd_vel_collision_checked", "cmd_vel_api", "cmd_vel"):
            fields.extend([f"{prefix}_x", f"{prefix}_y", f"{prefix}_z", f"{prefix}_speed"])
        for prefix in ("wheel", "wheel_ekf", "local", "tf_odom_base", "tf_map_base"):
            fields.extend([f"{prefix}_x", f"{prefix}_y", f"{prefix}_yaw", f"{prefix}_dx", f"{prefix}_dy", f"{prefix}_dist", f"{prefix}_dyaw"])
        for prefix in ("wheel", "wheel_ekf", "local"):
            fields.extend([f"{prefix}_twist_linear_x", f"{prefix}_twist_linear_y", f"{prefix}_twist_angular_z", f"{prefix}_twist_speed"])
        return fields

    def event_fields(self):
        return [
            "rel_time_sec",
            "event_type",
            "delta_candidate",
            "delta_accepted",
            "delta_rejected",
            "amcl_candidate_count",
            "amcl_accepted_count",
            "amcl_rejected_count",
            "goal_id",
            "goal_state",
            "goal_phase",
            "pose_id",
            "amcl_last_state",
            "last_accept_reason",
            "last_reject_reason",
            "last_candidate_translation_m",
            "last_candidate_yaw_rad",
            "last_accepted_translation_m",
            "last_accepted_yaw_rad",
            "last_accepted_source",
            "active_correction_source",
            "amcl_robot_moving",
            "motion_mode",
            "motion_linear_velocity",
            "motion_angular_velocity",
            "system_motion_mode",
            "system_control_mode",
            "wheel_twist_linear_x",
            "wheel_twist_angular_z",
            "local_twist_linear_x",
            "local_twist_angular_z",
            "cmd_vel_nav_raw_x",
            "cmd_vel_nav_raw_z",
            "cmd_vel_x",
            "cmd_vel_z",
            "tf_map_base_x",
            "tf_map_base_y",
            "tf_map_base_yaw",
            "local_dist",
            "wheel_dist",
        ]

    def on_twist(self, topic, msg):
        self.latest_twist[topic] = msg

    def on_speed_limit(self, msg):
        self.latest_speed_limit = msg

    def on_motion_state(self, msg):
        self.latest_motion_state = msg

    def on_system_state(self, msg):
        self.latest_system_state = msg

    def on_odom(self, name, msg):
        pose = pose_from_odom(msg)
        self.latest_odom[name] = pose
        self.latest_odom_twist[name] = twist_from_odom(msg)
        self.odom_origin.setdefault(name, pose)

    def on_nav_status(self, msg):
        if not msg.status_list:
            return
        status = msg.status_list[-1].status
        self.last_goal_status = STATUS_NAMES.get(int(status), str(status))

    def on_rosout(self, msg):
        text = msg.msg or ""
        if not ROSOUT_FILTER.search(text):
            return
        self.rosout_rows += 1
        self.rosout_writer.writerow({
            "rel_time_sec": f"{self.rel():.6f}",
            "level": msg.level,
            "node": msg.name,
            "message": text.replace("\n", "\\n"),
        })
        self.rosout_file.flush()

    def on_bridge(self, msg):
        try:
            data = json.loads(msg.data)
        except Exception:
            return
        if self.first_bridge is None:
            self.first_bridge = data
        self.last_bridge = data
        candidate = int(data.get("amcl_candidate_count") or 0)
        accepted = int(data.get("amcl_accepted_count") or 0)
        rejected = int(data.get("amcl_rejected_count") or 0)
        counts = (candidate, accepted, rejected)
        cand_translation = safe_float(data.get("last_candidate_correction_translation_m"))
        if cand_translation is not None:
            self.candidate_translations.append(cand_translation)
        if self.last_counts is None:
            self.last_counts = counts
            return
        dc = candidate - self.last_counts[0]
        da = accepted - self.last_counts[1]
        dr = rejected - self.last_counts[2]
        self.last_counts = counts
        if dc <= 0 and da <= 0 and dr <= 0:
            return
        event_type = "mixed"
        if da > 0 and dr <= 0:
            event_type = "accepted"
        elif dr > 0 and da <= 0:
            event_type = "rejected"
        elif dc > 0 and da <= 0 and dr <= 0:
            event_type = "candidate"
        if da > 0:
            value = safe_float(data.get("last_accepted_correction_translation_m"))
            if value is not None:
                self.accepted_event_translations.append(value)
        if dr > 0:
            value = safe_float(data.get("last_candidate_correction_translation_m"))
            if value is not None:
                self.rejected_event_translations.append(value)
        row = self.event_row(data, event_type, dc, da, dr)
        self.bridge_events.append(row)
        self.events_writer.writerow(row)
        self.events_file.flush()

    def current_tf_pose(self, target, source):
        try:
            tf = self.tf_buffer.lookup_transform(target, source, Time())
            return pose_from_tf(tf)
        except Exception:
            return None

    def update_api(self):
        nav = get_json(f"{api_url}/api/v1/navigation/state")
        pose = get_json(f"{api_url}/api/v1/robot/pose")
        self.last_api_nav = nav if isinstance(nav, dict) else {}
        self.last_api_pose = pose if isinstance(pose, dict) else {}
        self.api_file.write(json.dumps({
            "rel_time_sec": self.rel(),
            "navigation_state": self.last_api_nav,
            "robot_pose": self.last_api_pose,
        }, ensure_ascii=False, sort_keys=True) + "\n")
        self.api_file.flush()

        goal = self.current_goal()
        state = str(goal.get("state", ""))
        if state == "running":
            self.goal_was_running = True
        elif self.goal_was_running and state in TERMINAL_GOAL_STATES and self.terminal_seen_wall is None:
            self.terminal_seen_wall = time.time()

    def current_goal(self):
        goal = self.last_api_nav.get("navigation_goal") if isinstance(self.last_api_nav, dict) else {}
        return goal if isinstance(goal, dict) else {}

    def event_row(self, bridge, event_type, dc, da, dr):
        goal = self.current_goal()
        map_pose = self.current_tf_pose("map", "base_link")
        local_delta = pose_delta(self.latest_odom.get("local"), self.odom_origin.get("local"))
        wheel_delta = pose_delta(self.latest_odom.get("wheel"), self.odom_origin.get("wheel"))
        wheel_twist = self.latest_odom_twist.get("wheel", {})
        local_twist = self.latest_odom_twist.get("local", {})
        nav_raw = twist_values(self.latest_twist.get("/cmd_vel_nav_raw"))
        cmd = twist_values(self.latest_twist.get("/cmd_vel"))
        return {
            "rel_time_sec": f"{self.rel():.6f}",
            "event_type": event_type,
            "delta_candidate": dc,
            "delta_accepted": da,
            "delta_rejected": dr,
            "amcl_candidate_count": bridge.get("amcl_candidate_count"),
            "amcl_accepted_count": bridge.get("amcl_accepted_count"),
            "amcl_rejected_count": bridge.get("amcl_rejected_count"),
            "goal_id": goal.get("id"),
            "goal_state": goal.get("state"),
            "goal_phase": goal.get("phase"),
            "pose_id": goal.get("pose_id"),
            "amcl_last_state": bridge.get("amcl_last_state"),
            "last_accept_reason": bridge.get("last_accept_reason"),
            "last_reject_reason": bridge.get("last_reject_reason"),
            "last_candidate_translation_m": bridge.get("last_candidate_correction_translation_m"),
            "last_candidate_yaw_rad": bridge.get("last_candidate_correction_yaw_rad"),
            "last_accepted_translation_m": bridge.get("last_accepted_correction_translation_m"),
            "last_accepted_yaw_rad": bridge.get("last_accepted_correction_yaw_rad"),
            "last_accepted_source": bridge.get("last_accepted_source"),
            "active_correction_source": bridge.get("active_correction_source"),
            "amcl_robot_moving": bridge.get("amcl_robot_moving"),
            "motion_mode": motion_value(self.latest_motion_state, "motion_mode"),
            "motion_linear_velocity": motion_value(self.latest_motion_state, "linear_velocity"),
            "motion_angular_velocity": motion_value(self.latest_motion_state, "angular_velocity"),
            "system_motion_mode": motion_value(self.latest_system_state, "motion_mode"),
            "system_control_mode": motion_value(self.latest_system_state, "control_mode"),
            "wheel_twist_linear_x": wheel_twist.get("linear_x", ""),
            "wheel_twist_angular_z": wheel_twist.get("angular_z", ""),
            "local_twist_linear_x": local_twist.get("linear_x", ""),
            "local_twist_angular_z": local_twist.get("angular_z", ""),
            "cmd_vel_nav_raw_x": nav_raw["x"],
            "cmd_vel_nav_raw_z": nav_raw["z"],
            "cmd_vel_x": cmd["x"],
            "cmd_vel_z": cmd["z"],
            "tf_map_base_x": "" if map_pose is None else map_pose["x"],
            "tf_map_base_y": "" if map_pose is None else map_pose["y"],
            "tf_map_base_yaw": "" if map_pose is None else map_pose["yaw"],
            "local_dist": local_delta["dist"],
            "wheel_dist": wheel_delta["dist"],
        }

    def sample_row(self):
        goal = self.current_goal()
        bridge = self.last_bridge or {}
        row = {
            "rel_time_sec": f"{self.rel():.6f}",
            "goal_id": goal.get("id", ""),
            "goal_state": goal.get("state", ""),
            "goal_phase": goal.get("phase", ""),
            "pose_id": goal.get("pose_id", ""),
            "final_distance_m": goal.get("final_distance_m", ""),
            "final_yaw_error_rad": goal.get("final_yaw_error_rad", ""),
            "nav2_result_code": goal.get("nav2_result_code", ""),
            "nav2_status": self.last_goal_status,
            "speed_limit_mps": "" if self.latest_speed_limit is None else self.latest_speed_limit.speed_limit,
            "amcl_candidate_count": bridge.get("amcl_candidate_count", ""),
            "amcl_accepted_count": bridge.get("amcl_accepted_count", ""),
            "amcl_rejected_count": bridge.get("amcl_rejected_count", ""),
            "amcl_last_state": bridge.get("amcl_last_state", ""),
            "last_accept_reason": bridge.get("last_accept_reason", ""),
            "last_reject_reason": bridge.get("last_reject_reason", ""),
            "last_candidate_translation_m": bridge.get("last_candidate_correction_translation_m", ""),
            "last_candidate_yaw_rad": bridge.get("last_candidate_correction_yaw_rad", ""),
            "last_accepted_translation_m": bridge.get("last_accepted_correction_translation_m", ""),
            "last_accepted_yaw_rad": bridge.get("last_accepted_correction_yaw_rad", ""),
            "last_accepted_source": bridge.get("last_accepted_source", ""),
            "active_correction_source": bridge.get("active_correction_source", ""),
            "correction_active": bridge.get("correction_active", ""),
            "amcl_robot_moving": bridge.get("amcl_robot_moving", ""),
            "amcl_linear_speed_mps": bridge.get("amcl_linear_speed_mps", ""),
            "amcl_angular_speed_radps": bridge.get("amcl_angular_speed_radps", ""),
            "safe_for_goal_start": bridge.get("safe_for_goal_start", ""),
            "localization_degraded": bridge.get("localization_degraded", ""),
            "motion_mode": motion_value(self.latest_motion_state, "motion_mode"),
            "motion_linear_velocity": motion_value(self.latest_motion_state, "linear_velocity"),
            "motion_lateral_velocity": motion_value(self.latest_motion_state, "lateral_velocity"),
            "motion_angular_velocity": motion_value(self.latest_motion_state, "angular_velocity"),
            "motion_steering_angle": motion_value(self.latest_motion_state, "steering_angle"),
            "system_vehicle_state": motion_value(self.latest_system_state, "vehicle_state"),
            "system_control_mode": motion_value(self.latest_system_state, "control_mode"),
            "system_motion_mode": motion_value(self.latest_system_state, "motion_mode"),
            "system_error_code": motion_value(self.latest_system_state, "error_code"),
            "system_battery_voltage": motion_value(self.latest_system_state, "battery_voltage"),
        }
        for topic, prefix in (
            ("/cmd_vel_nav_raw", "cmd_vel_nav_raw"),
            ("/cmd_vel_nav", "cmd_vel_nav"),
            ("/cmd_vel_collision_checked", "cmd_vel_collision_checked"),
            ("/cmd_vel_api", "cmd_vel_api"),
            ("/cmd_vel", "cmd_vel"),
        ):
            values = twist_values(self.latest_twist.get(topic))
            row[f"{prefix}_x"] = values["x"]
            row[f"{prefix}_y"] = values["y"]
            row[f"{prefix}_z"] = values["z"]
            row[f"{prefix}_speed"] = values["speed"]
        for name, pose in (
            ("wheel", self.latest_odom.get("wheel")),
            ("wheel_ekf", self.latest_odom.get("wheel_ekf")),
            ("local", self.latest_odom.get("local")),
            ("tf_odom_base", self.current_tf_pose("odom", "base_link")),
            ("tf_map_base", self.current_tf_pose("map", "base_link")),
        ):
            if name.startswith("tf_") and pose is not None:
                self.tf_origin.setdefault(name, pose)
                origin = self.tf_origin.get(name)
            else:
                origin = self.odom_origin.get(name)
            delta = pose_delta(pose, origin)
            row[f"{name}_x"] = "" if pose is None else pose["x"]
            row[f"{name}_y"] = "" if pose is None else pose["y"]
            row[f"{name}_yaw"] = "" if pose is None else pose["yaw"]
            row[f"{name}_dx"] = delta["dx"]
            row[f"{name}_dy"] = delta["dy"]
            row[f"{name}_dist"] = delta["dist"]
            row[f"{name}_dyaw"] = delta["dyaw"]
        for name in ("wheel", "wheel_ekf", "local"):
            values = self.latest_odom_twist.get(name, {})
            row[f"{name}_twist_linear_x"] = values.get("linear_x", "")
            row[f"{name}_twist_linear_y"] = values.get("linear_y", "")
            row[f"{name}_twist_angular_z"] = values.get("angular_z", "")
            row[f"{name}_twist_speed"] = values.get("speed", "")
        return row

    def write_sample(self):
        self.update_api()
        row = self.sample_row()
        self.samples_writer.writerow({k: fmt(v) for k, v in row.items()})
        self.samples_file.flush()

    def close(self):
        self.samples_file.close()
        self.events_file.close()
        self.api_file.close()
        self.rosout_file.close()


rclpy.init()
node = Probe()
deadline = time.time() + duration_sec
next_sample = time.time()
try:
    while rclpy.ok() and time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.03)
        now = time.time()
        if now >= next_sample:
            node.write_sample()
            next_sample = now + sample_period_sec
        if stop_when_terminal and node.terminal_seen_wall is not None and now - node.terminal_seen_wall >= 1.0:
            break
finally:
    first = node.first_bridge or {}
    last = node.last_bridge or first

    def count_delta(key):
        return int(last.get(key) or 0) - int(first.get(key) or 0)

    event_reason_counts = Counter()
    event_phase_counts = Counter()
    accepted_events = []
    rejected_events = []
    for event in node.bridge_events:
        if safe_float(event.get("delta_accepted"), 0.0) > 0:
            accepted_events.append(event)
        if safe_float(event.get("delta_rejected"), 0.0) > 0:
            rejected_events.append(event)
        reason = event.get("last_reject_reason") if safe_float(event.get("delta_rejected"), 0.0) > 0 else event.get("last_accept_reason")
        if reason:
            event_reason_counts[str(reason)] += 1
        phase = str(event.get("goal_phase") or "unknown")
        event_phase_counts[phase] += 1

    goal = node.current_goal()
    summary = {
        "duration_sec": time.time() - node.start_wall,
        "sample_period_sec": sample_period_sec,
        "samples": sum(1 for _ in (output_dir / "samples.csv").open(encoding="utf-8")) - 1,
        "bridge_event_rows": len(node.bridge_events),
        "rosout_filtered_rows": node.rosout_rows,
        "baseline_amcl_candidate_count": int(first.get("amcl_candidate_count") or 0),
        "end_amcl_candidate_count": int(last.get("amcl_candidate_count") or 0),
        "delta_amcl_candidate_count": count_delta("amcl_candidate_count"),
        "baseline_amcl_accepted_count": int(first.get("amcl_accepted_count") or 0),
        "end_amcl_accepted_count": int(last.get("amcl_accepted_count") or 0),
        "delta_amcl_accepted_count": count_delta("amcl_accepted_count"),
        "baseline_amcl_rejected_count": int(first.get("amcl_rejected_count") or 0),
        "end_amcl_rejected_count": int(last.get("amcl_rejected_count") or 0),
        "delta_amcl_rejected_count": count_delta("amcl_rejected_count"),
        "last_amcl_state": last.get("amcl_last_state"),
        "last_accept_reason": last.get("last_accept_reason"),
        "last_reject_reason": last.get("last_reject_reason"),
        "last_candidate_correction_translation_m": last.get("last_candidate_correction_translation_m"),
        "last_candidate_correction_yaw_rad": last.get("last_candidate_correction_yaw_rad"),
        "last_accepted_correction_translation_m": last.get("last_accepted_correction_translation_m"),
        "last_accepted_correction_yaw_rad": last.get("last_accepted_correction_yaw_rad"),
        "max_observed_candidate_translation_m": max(node.candidate_translations) if node.candidate_translations else None,
        "accepted_event_translation_stats": stats(node.accepted_event_translations),
        "rejected_event_candidate_translation_stats": stats(node.rejected_event_translations),
        "event_reason_counts": dict(event_reason_counts),
        "event_phase_counts": dict(event_phase_counts),
        "final_goal": {
            "id": goal.get("id"),
            "state": goal.get("state"),
            "phase": goal.get("phase"),
            "pose_id": goal.get("pose_id"),
            "final_distance_m": goal.get("final_distance_m"),
            "final_yaw_error_rad": goal.get("final_yaw_error_rad"),
            "task_complete": goal.get("task_complete"),
            "final_pose_verify_reason": goal.get("final_pose_verify_reason"),
        },
    }
    (output_dir / "summary.json").write_text(
        json.dumps(summary, indent=2, sort_keys=True, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )

    top_rejected = sorted(
        rejected_events,
        key=lambda row: safe_float(row.get("last_candidate_translation_m"), -1.0) or -1.0,
        reverse=True,
    )[:8]
    top_accepted = sorted(
        accepted_events,
        key=lambda row: safe_float(row.get("last_accepted_translation_m"), -1.0) or -1.0,
        reverse=True,
    )[:8]

    lines = [
        "# Navigation AMCL/Odom Correlation",
        "",
        f"- report_dir: `{output_dir}`",
        f"- duration_sec: `{summary['duration_sec']:.1f}`",
        f"- samples: `{summary['samples']}`",
        f"- bridge_event_rows: `{summary['bridge_event_rows']}`",
        f"- delta_amcl_candidate_count: `{summary['delta_amcl_candidate_count']}`",
        f"- delta_amcl_accepted_count: `{summary['delta_amcl_accepted_count']}`",
        f"- delta_amcl_rejected_count: `{summary['delta_amcl_rejected_count']}`",
        f"- max_observed_candidate_translation_m: `{summary['max_observed_candidate_translation_m']}`",
        f"- accepted_event_translation_stats: `{summary['accepted_event_translation_stats']}`",
        f"- rejected_event_candidate_translation_stats: `{summary['rejected_event_candidate_translation_stats']}`",
        f"- last_amcl_state: `{summary['last_amcl_state']}`",
        f"- last_accept_reason: `{summary['last_accept_reason']}`",
        f"- last_reject_reason: `{summary['last_reject_reason']}`",
        f"- final_goal: `{summary['final_goal']}`",
        "",
        "## Event Reason Counts",
        "",
    ]
    if event_reason_counts:
        for reason, count in event_reason_counts.most_common():
            lines.append(f"- `{reason}`: `{count}`")
    else:
        lines.append("- none")
    lines.extend(["", "## Event Phase Counts", ""])
    if event_phase_counts:
        for phase, count in event_phase_counts.most_common():
            lines.append(f"- `{phase}`: `{count}`")
    else:
        lines.append("- none")

    def table(title, rows, translation_key):
        lines.extend(["", f"## {title}", "", "| rel_s | phase | event | d_acc | d_rej | translation_m | reject_reason | accept_reason | cmd_x | cmd_z | motion_v | motion_w | wheel_v | wheel_w |", "|---:|---|---|---:|---:|---:|---|---|---:|---:|---:|---:|---:|---:|"])
        if not rows:
            lines.append("| | | none | | | | | | | | | | | |")
            return
        for row in rows:
            lines.append(
                "| "
                + " | ".join(
                    str(row.get(key, ""))
                    for key in (
                        "rel_time_sec",
                        "goal_phase",
                        "event_type",
                        "delta_accepted",
                        "delta_rejected",
                        translation_key,
                        "last_reject_reason",
                        "last_accept_reason",
                        "cmd_vel_nav_raw_x",
                        "cmd_vel_nav_raw_z",
                        "motion_linear_velocity",
                        "motion_angular_velocity",
                        "wheel_twist_linear_x",
                        "wheel_twist_angular_z",
                    )
                )
                + " |"
            )

    table("Largest Rejected Candidate Events", top_rejected, "last_candidate_translation_m")
    table("Largest Accepted Correction Events", top_accepted, "last_accepted_translation_m")

    lines.extend([
        "",
        "Files:",
        "",
        "- `samples.csv`",
        "- `amcl_bridge_events.csv`",
        "- `api_state_samples.jsonl`",
        "- `rosout_filtered.csv`",
        "- `summary.json`",
    ])
    (output_dir / "summary.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    node.close()
    node.destroy_node()
    rclpy.shutdown()
    print(f"{output_dir}")
PY

rc=$?
if [[ "${rc}" -eq 0 ]]; then
  echo "${PREFIX} summary ${OUTPUT_DIR}/summary.md"
  echo "${PREFIX} complete: ${OUTPUT_DIR}"
else
  echo "${PREFIX} FAIL capture exited with rc=${rc}" >&2
fi
exit "${rc}"
