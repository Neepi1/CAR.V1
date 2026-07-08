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
LABEL="odom_goal_closure"
OUTPUT_DIR=""
POSE_ID=""
BUILDING_ID=""
FLOOR_ID=""
GOAL_COMPLETION_POLICY="pose_required"
SEND_GOAL=false
STOP_WHEN_TERMINAL=true
IMU_TOPIC="/lidar_imu"
IMU_BIAS_SEC=1.0
BASE_FRAME="base_link"
PROJECT_IMU_TO_BASE=true
TF_WAIT_SEC=2.0
PREFIX="[nav-odom-goal]"

usage() {
  cat <<'EOF'
Usage:
  bash scripts/jetson/runtime_overlay/scripts/record_navigation_odom_goal_closure.sh \
    --pose-id delivery_675235 \
    --send-goal \
    --duration-sec 120 \
    --label delivery_675235_odom_goal

Purpose:
  Runs or observes one normal API navigation goal and freezes the start
  map->odom transform. It converts the target map pose into the start odom
  frame, then compares the target relative motion with /wheel/odom and
  /local_state/odometry relative motion.

Safety:
  - does not publish velocity
  - does not call localization/relocalization services
  - does not set params or restart nodes
  - when --send-goal is used, only POSTs the normal robot_api_server
    /api/v1/navigation/goal endpoint

Options:
  --pose-id ID                  Saved pose id, e.g. delivery_675235.
  --building-id ID              Building id. If omitted, API runtime context is used.
  --floor-id ID                 Floor id. If omitted, API runtime context is used.
  --goal-completion-policy P    pose_required|position_only|api_default. Default: pose_required.
  --send-goal                   POST /api/v1/navigation/goal after baseline capture.
  --duration-sec N              Capture duration in seconds. Default: 120.
  --sample-period-sec N         CSV sample period in seconds. Default: 0.25.
  --label LABEL                 Report label. Default: odom_goal_closure.
  --api-url URL                 robot_api_server URL. Default: http://127.0.0.1:8080.
  --output-dir DIR              Report directory. Default: /tmp/njrh_reports/navigation_odom_goal_closure/<timestamp>_<label>.
  --imu-topic TOPIC             IMU topic for diagnostic yaw integration. Default: /lidar_imu.
  --imu-bias-sec N              Stationary IMU gyro bias capture before goal POST. Default: 1.0.
  --base-frame FRAME            Frame used to project IMU angular velocity. Default: base_link.
  --no-project-imu-to-base      Use raw IMU angular_velocity.z instead of projected base-frame z.
  --tf-wait-sec N               TF wait for IMU projection. Default: 2.0.
  --no-stop-when-terminal       Keep recording until duration even after terminal state.
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
    --goal-completion-policy)
      GOAL_COMPLETION_POLICY="${2:-}"
      shift 2
      ;;
    --send-goal)
      SEND_GOAL=true
      shift
      ;;
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
    --imu-topic)
      IMU_TOPIC="${2:-}"
      shift 2
      ;;
    --imu-bias-sec)
      IMU_BIAS_SEC="${2:-}"
      shift 2
      ;;
    --base-frame)
      BASE_FRAME="${2:-}"
      shift 2
      ;;
    --no-project-imu-to-base)
      PROJECT_IMU_TO_BASE=false
      shift
      ;;
    --tf-wait-sec)
      TF_WAIT_SEC="${2:-}"
      shift 2
      ;;
    --no-stop-when-terminal)
      STOP_WHEN_TERMINAL=false
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

if [[ -z "${POSE_ID}" ]]; then
  echo "${PREFIX} FAIL --pose-id is required" >&2
  exit 2
fi

if [[ "${GOAL_COMPLETION_POLICY}" != "pose_required" &&
      "${GOAL_COMPLETION_POLICY}" != "position_only" &&
      "${GOAL_COMPLETION_POLICY}" != "api_default" ]]; then
  echo "${PREFIX} FAIL --goal-completion-policy must be pose_required, position_only, or api_default" >&2
  exit 2
fi

if ! [[ "${DURATION_SEC}" =~ ^[0-9]+$ ]] || [[ "${DURATION_SEC}" -lt 10 ]]; then
  echo "${PREFIX} FAIL --duration-sec must be an integer >= 10" >&2
  exit 2
fi

if ! is_number "${SAMPLE_PERIOD_SEC}"; then
  echo "${PREFIX} FAIL --sample-period-sec must be numeric" >&2
  exit 2
fi

if ! is_number "${IMU_BIAS_SEC}"; then
  echo "${PREFIX} FAIL --imu-bias-sec must be numeric" >&2
  exit 2
fi

if ! is_number "${TF_WAIT_SEC}"; then
  echo "${PREFIX} FAIL --tf-wait-sec must be numeric" >&2
  exit 2
fi

TIMESTAMP="$(date -u +%Y%m%dT%H%M%SZ)"
LABEL="$(sanitize_label "${LABEL}")"
if [[ -z "${OUTPUT_DIR}" ]]; then
  OUTPUT_ROOT="${TMPDIR:-/tmp}/njrh_reports/navigation_odom_goal_closure"
  OUTPUT_DIR="${OUTPUT_ROOT}/${TIMESTAMP}_${LABEL}_${POSE_ID}_${DURATION_SEC}s"
fi
if ! mkdir -p "${OUTPUT_DIR}" 2>/dev/null; then
  echo "${PREFIX} FAIL cannot create output directory: ${OUTPUT_DIR}" >&2
  echo "${PREFIX} hint: pass --output-dir /tmp/<name>" >&2
  exit 1
fi
if [[ ! -w "${OUTPUT_DIR}" ]]; then
  echo "${PREFIX} FAIL output directory is not writable: ${OUTPUT_DIR}" >&2
  exit 1
fi

{
  echo "timestamp_utc=${TIMESTAMP}"
  echo "duration_sec=${DURATION_SEC}"
  echo "sample_period_sec=${SAMPLE_PERIOD_SEC}"
  echo "label=${LABEL}"
  echo "api_url=${API_URL}"
  echo "pose_id=${POSE_ID}"
  echo "building_id=${BUILDING_ID}"
  echo "floor_id=${FLOOR_ID}"
  echo "goal_completion_policy=${GOAL_COMPLETION_POLICY}"
  echo "send_goal=${SEND_GOAL}"
  echo "stop_when_terminal=${STOP_WHEN_TERMINAL}"
  echo "imu_topic=${IMU_TOPIC}"
  echo "imu_bias_sec=${IMU_BIAS_SEC}"
  echo "base_frame=${BASE_FRAME}"
  echo "project_imu_to_base=${PROJECT_IMU_TO_BASE}"
  echo "tf_wait_sec=${TF_WAIT_SEC}"
  echo "workspace_root=${WORKSPACE_ROOT}"
  echo "publishes_velocity=false"
  echo "calls_relocalization=false"
  echo "sets_params=false"
} >"${OUTPUT_DIR}/metadata.env"

echo "${PREFIX} report_dir=${OUTPUT_DIR}"
echo "${PREFIX} pose_id=${POSE_ID} send_goal=${SEND_GOAL} duration_sec=${DURATION_SEC}"

python3 - \
  "${DURATION_SEC}" \
  "${SAMPLE_PERIOD_SEC}" \
  "${API_URL}" \
  "${OUTPUT_DIR}" \
  "${POSE_ID}" \
  "${BUILDING_ID}" \
  "${FLOOR_ID}" \
  "${GOAL_COMPLETION_POLICY}" \
  "${SEND_GOAL}" \
  "${STOP_WHEN_TERMINAL}" \
  "${IMU_TOPIC}" \
  "${IMU_BIAS_SEC}" \
  "${BASE_FRAME}" \
  "${PROJECT_IMU_TO_BASE}" \
  "${TF_WAIT_SEC}" <<'PY'
import csv
import json
import math
import sys
import time
import urllib.error
import urllib.parse
import urllib.request
from pathlib import Path

import rclpy
from action_msgs.msg import GoalStatusArray
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import Imu
from std_msgs.msg import String
import tf2_ros

try:
    from nav2_msgs.msg import SpeedLimit
except Exception:
    SpeedLimit = None

try:
    from ranger_msgs.msg import MotionState, SystemState
except Exception:
    MotionState = None
    SystemState = None


duration_sec = float(sys.argv[1])
sample_period_sec = float(sys.argv[2])
api_url = sys.argv[3].rstrip("/")
output_dir = Path(sys.argv[4])
pose_id = sys.argv[5]
building_id_arg = sys.argv[6]
floor_id_arg = sys.argv[7]
goal_completion_policy = sys.argv[8]
send_goal = sys.argv[9].lower() == "true"
stop_when_terminal = sys.argv[10].lower() == "true"
imu_topic = sys.argv[11]
imu_bias_sec = float(sys.argv[12])
base_frame = sys.argv[13]
project_imu_to_base = sys.argv[14].lower() == "true"
tf_wait_sec = float(sys.argv[15])
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


def stamp_sec(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def rotate_vector_by_quaternion(vector, quaternion):
    x, y, z = vector
    qx, qy, qz, qw = quaternion
    # Expanded q * v * q^-1 for vector rotation.
    tx = 2.0 * (qy * z - qz * y)
    ty = 2.0 * (qz * x - qx * z)
    tz = 2.0 * (qx * y - qy * x)
    rx = x + qw * tx + (qy * tz - qz * ty)
    ry = y + qw * ty + (qz * tx - qx * tz)
    rz = z + qw * tz + (qx * ty - qy * tx)
    return (rx, ry, rz)


def pose_from_odom(msg):
    p = msg.pose.pose.position
    q = msg.pose.pose.orientation
    return {"x": float(p.x), "y": float(p.y), "yaw": yaw_from_quat(q)}


def twist_from_odom(msg):
    tw = msg.twist.twist
    return {
        "x": float(tw.linear.x),
        "y": float(tw.linear.y),
        "z": float(tw.angular.z),
        "speed": math.hypot(float(tw.linear.x), float(tw.linear.y)),
    }


def pose_from_tf(transform):
    p = transform.transform.translation
    q = transform.transform.rotation
    return {"x": float(p.x), "y": float(p.y), "yaw": yaw_from_quat(q)}


def compose(a, b):
    ca = math.cos(a["yaw"])
    sa = math.sin(a["yaw"])
    return {
        "x": a["x"] + ca * b["x"] - sa * b["y"],
        "y": a["y"] + sa * b["x"] + ca * b["y"],
        "yaw": norm_angle(a["yaw"] + b["yaw"]),
    }


def inverse(a):
    ca = math.cos(a["yaw"])
    sa = math.sin(a["yaw"])
    return {
        "x": -ca * a["x"] - sa * a["y"],
        "y": sa * a["x"] - ca * a["y"],
        "yaw": norm_angle(-a["yaw"]),
    }


def between(a, b):
    if a is None or b is None:
        return None
    return compose(inverse(a), b)


def pose_error(actual, target):
    err = between(actual, target)
    if err is None:
        return {"dx": None, "dy": None, "dist": None, "dyaw": None}
    return {
        "dx": err["x"],
        "dy": err["y"],
        "dist": math.hypot(err["x"], err["y"]),
        "dyaw": err["yaw"],
    }


def twist_values(msg):
    if msg is None:
        return {"x": None, "y": None, "z": None, "speed": None}
    return {
        "x": float(msg.linear.x),
        "y": float(msg.linear.y),
        "z": float(msg.angular.z),
        "speed": math.hypot(float(msg.linear.x), float(msg.linear.y)),
    }


def safe_float(value, default=None):
    try:
        result = float(value)
    except Exception:
        return default
    return result if math.isfinite(result) else default


def nested(data, path, default=None):
    cur = data
    for key in path:
        if not isinstance(cur, dict) or key not in cur:
            return default
        cur = cur[key]
    return cur


def api_request(method, path, body=None, timeout=3.0):
    data = None
    headers = {}
    if body is not None:
        data = json.dumps(body, ensure_ascii=False).encode("utf-8")
        headers["Content-Type"] = "application/json"
    req = urllib.request.Request(api_url + path, data=data, headers=headers, method=method)
    try:
        with urllib.request.urlopen(req, timeout=timeout) as resp:
            text = resp.read().decode("utf-8", errors="replace")
            parsed = json.loads(text) if text else {}
            if isinstance(parsed, dict):
                parsed["_http_status"] = resp.status
            return parsed
    except urllib.error.HTTPError as exc:
        text = exc.read().decode("utf-8", errors="replace")
        try:
            parsed = json.loads(text)
        except Exception:
            parsed = {"raw": text}
        if isinstance(parsed, dict):
            parsed["_http_status"] = exc.code
        return parsed
    except Exception as exc:
        return {"_error": repr(exc)}


def api_get(path, timeout=1.0):
    return api_request("GET", path, timeout=timeout)


def resolve_runtime_context():
    for path in ("/api/v1/navigation/state", "/api/v1/status"):
        data = api_get(path, timeout=2.0)
        if not isinstance(data, dict):
            continue
        candidates = [
            data.get("runtime_map_context"),
            data.get("map_context"),
            nested(data, ["navigation", "runtime_map_context"]),
            nested(data, ["navigation", "map_context"]),
            nested(data, ["body", "runtime_map_context"]),
            nested(data, ["body", "map_context"]),
            nested(data, ["body", "navigation", "runtime_map_context"]),
            nested(data, ["body", "navigation", "map_context"]),
        ]
        for candidate in candidates:
            if not isinstance(candidate, dict):
                continue
            building = str(candidate.get("building_id") or "")
            floor = str(candidate.get("floor_id") or "")
            if building and floor:
                return building, floor
    return None, None


def resolve_target_pose():
    building_id = building_id_arg
    floor_id = floor_id_arg
    if not building_id or not floor_id:
        resolved_building, resolved_floor = resolve_runtime_context()
        building_id = building_id or resolved_building
        floor_id = floor_id or resolved_floor
    if not building_id or not floor_id:
        raise RuntimeError("cannot resolve building_id/floor_id from API runtime context")
    query = urllib.parse.urlencode({
        "pose_id": pose_id,
        "building_id": building_id,
        "floor_id": floor_id,
    })
    precheck = api_get(f"/api/v1/navigation/pre_goal_check?{query}", timeout=3.0)
    goal = nested(precheck, ["pose_resolution", "goal"])
    if not isinstance(goal, dict):
        raise RuntimeError(f"pre_goal_check did not resolve target pose: {precheck}")
    target = {
        "x": float(goal["x"]),
        "y": float(goal["y"]),
        "yaw": float(goal["yaw"]),
    }
    return building_id, floor_id, target, precheck


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


def motion_value(msg, name):
    if msg is None:
        return None
    value = getattr(msg, name, None)
    if hasattr(value, "data"):
        value = value.data
    return value


def parse_mode_status(text):
    if not text:
        return {}
    try:
        data = json.loads(text)
    except Exception:
        return {"raw": text}
    if not isinstance(data, dict):
        return {"raw": text}
    return data


def classify_phase(row):
    vx = safe_float(row.get("cmd_vel_x"), 0.0) or 0.0
    vy = safe_float(row.get("cmd_vel_y"), 0.0) or 0.0
    wz = safe_float(row.get("cmd_vel_z"), 0.0) or 0.0
    wvx = safe_float(row.get("wheel_twist_x"), 0.0) or 0.0
    wvy = safe_float(row.get("wheel_twist_y"), 0.0) or 0.0
    wwz = safe_float(row.get("wheel_twist_z"), 0.0) or 0.0
    linear = max(abs(vx), abs(vy), abs(wvx), abs(wvy))
    angular = max(abs(wz), abs(wwz))
    mode = str(row.get("actual_mode_short") or row.get("actual_mode_name") or row.get("motion_mode") or "")
    if linear < 0.03 and angular < 0.035:
        return "settle_stop"
    if "SPIN" in mode.upper() or (linear < 0.04 and angular >= 0.035):
        return "spin"
    if abs(vy) > 0.03 or "SIDE" in mode.upper():
        return "lateral"
    if linear >= 0.03 and angular >= 0.05:
        return "arc_turn"
    if linear >= 0.03:
        return "straight"
    return "mixed"


class Probe(Node):
    def __init__(self):
        super().__init__("record_navigation_odom_goal_closure")
        self.start_wall = time.time()
        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=20.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.latest_odom = {}
        self.latest_odom_twist = {}
        self.latest_twist = {}
        self.latest_motion_state = None
        self.latest_system_state = None
        self.latest_mode_status = ""
        self.latest_speed_limit = None
        self.latest_bridge = {}
        self.last_goal_status = ""
        self.last_nav_state = {}
        self.last_robot_pose = {}
        self.goal_was_running = False
        self.terminal_seen_wall = None
        self.latest_imu = None
        self.imu_frame = ""
        self.imu_to_base_quat = None
        self.imu_yaw_rate_source = "raw_imu_z"
        self.collecting_imu_bias = False
        self.imu_bias_samples = []
        self.imu_bias = 0.0
        self.integrating_imu = False
        self.imu_prev_t = None
        self.imu_yaw_delta = 0.0
        self.imu_dt_values = []
        self.imu_sample_count = 0
        self.imu_wz_base = None
        self.imu_wz_bias_corrected = None
        self.imu_max_abs_wz_bias_corrected = 0.0

        qos = QoSProfile(depth=80)
        be = QoSProfile(depth=120, history=HistoryPolicy.KEEP_LAST)
        be.reliability = ReliabilityPolicy.BEST_EFFORT
        imu_qos = QoSProfile(depth=50, history=HistoryPolicy.KEEP_LAST)
        imu_qos.reliability = ReliabilityPolicy.BEST_EFFORT

        for topic in (
            "/cmd_vel_nav_raw",
            "/cmd_vel_nav",
            "/cmd_vel_collision_checked",
            "/cmd_vel_api",
            "/cmd_vel_safe",
            "/cmd_vel",
        ):
            self.create_subscription(Twist, topic, lambda msg, t=topic: self.on_twist(t, msg), qos)

        self.create_subscription(Odometry, "/wheel/odom", lambda msg: self.on_odom("wheel", msg), be)
        self.create_subscription(Odometry, "/wheel/odom_ekf", lambda msg: self.on_odom("wheel_ekf", msg), be)
        self.create_subscription(Odometry, "/local_state/odometry", lambda msg: self.on_odom("local", msg), be)
        if imu_topic:
            self.create_subscription(Imu, imu_topic, self.on_imu, imu_qos)
        self.create_subscription(String, "/ranger_mini3_mode_controller/status", self.on_mode_status, qos)
        self.create_subscription(String, "/localization/bridge_status", self.on_bridge, qos)
        self.create_subscription(GoalStatusArray, "/navigate_to_pose/_action/status", self.on_nav_status, qos)
        if SpeedLimit is not None:
            self.create_subscription(SpeedLimit, "/speed_limit", self.on_speed_limit, qos)
        if MotionState is not None:
            self.create_subscription(MotionState, "/motion_state", self.on_motion_state, qos)
        if SystemState is not None:
            self.create_subscription(SystemState, "/system_state", self.on_system_state, qos)

    def rel(self):
        return time.time() - self.start_wall

    def on_twist(self, topic, msg):
        self.latest_twist[topic] = msg

    def on_odom(self, name, msg):
        self.latest_odom[name] = pose_from_odom(msg)
        self.latest_odom_twist[name] = twist_from_odom(msg)

    def configure_imu_projection(self):
        if not self.latest_imu:
            return False
        if not project_imu_to_base:
            self.imu_to_base_quat = None
            self.imu_yaw_rate_source = "raw_imu_z"
            return True
        if not self.imu_frame:
            return False
        deadline = time.time() + max(0.1, tf_wait_sec)
        last_error = ""
        while rclpy.ok() and time.time() < deadline:
            try:
                transform = self.tf_buffer.lookup_transform(
                    base_frame,
                    self.imu_frame,
                    Time(),
                    timeout=Duration(seconds=0.15),
                )
                q = transform.transform.rotation
                self.imu_to_base_quat = (float(q.x), float(q.y), float(q.z), float(q.w))
                self.imu_yaw_rate_source = f"tf_projected_{self.imu_frame}_to_{base_frame}_z"
                return True
            except Exception as exc:
                last_error = str(exc)
                rclpy.spin_once(self, timeout_sec=0.03)
        self.imu_to_base_quat = None
        self.imu_yaw_rate_source = f"raw_imu_z_projection_failed:{last_error}"
        return False

    def imu_yaw_rate(self, msg):
        vector = (
            float(msg.angular_velocity.x),
            float(msg.angular_velocity.y),
            float(msg.angular_velocity.z),
        )
        if self.imu_to_base_quat is None:
            return vector[2]
        return rotate_vector_by_quaternion(vector, self.imu_to_base_quat)[2]

    def on_imu(self, msg):
        self.latest_imu = msg
        self.imu_frame = msg.header.frame_id
        t = stamp_sec(msg.header.stamp)
        wz = self.imu_yaw_rate(msg)
        self.imu_wz_base = wz

        if self.collecting_imu_bias:
            self.imu_bias_samples.append(wz)
            return

        corrected = wz - self.imu_bias
        self.imu_wz_bias_corrected = corrected
        if not self.integrating_imu:
            return
        if self.imu_prev_t is not None:
            dt = t - self.imu_prev_t
            if 0.0 < dt < 0.05:
                self.imu_yaw_delta = norm_angle(self.imu_yaw_delta + corrected * dt)
                self.imu_dt_values.append(dt)
        self.imu_prev_t = t
        self.imu_sample_count += 1
        self.imu_max_abs_wz_bias_corrected = max(self.imu_max_abs_wz_bias_corrected, abs(corrected))

    def on_mode_status(self, msg):
        self.latest_mode_status = msg.data

    def on_bridge(self, msg):
        try:
            data = json.loads(msg.data)
        except Exception:
            return
        if isinstance(data, dict):
            self.latest_bridge = data

    def on_nav_status(self, msg):
        if msg.status_list:
            self.last_goal_status = STATUS_NAMES.get(int(msg.status_list[-1].status), str(msg.status_list[-1].status))

    def on_speed_limit(self, msg):
        self.latest_speed_limit = msg

    def on_motion_state(self, msg):
        self.latest_motion_state = msg

    def on_system_state(self, msg):
        self.latest_system_state = msg

    def current_tf_pose(self, target, source):
        try:
            tf = self.tf_buffer.lookup_transform(target, source, Time())
            return pose_from_tf(tf)
        except Exception:
            return None

    def update_api(self):
        nav = api_get("/api/v1/navigation/state", timeout=0.35)
        pose = api_get("/api/v1/robot/pose", timeout=0.35)
        self.last_nav_state = nav if isinstance(nav, dict) else {}
        self.last_robot_pose = pose if isinstance(pose, dict) else {}
        goal = self.current_goal()
        state = str(goal.get("state", ""))
        if state == "running":
            self.goal_was_running = True
        elif self.goal_was_running and state in TERMINAL_GOAL_STATES and self.terminal_seen_wall is None:
            self.terminal_seen_wall = time.time()

    def current_goal(self):
        goal = self.last_nav_state.get("navigation_goal") if isinstance(self.last_nav_state, dict) else {}
        return goal if isinstance(goal, dict) else {}


def wait_for_baseline(node, timeout_sec=8.0):
    deadline = time.time() + timeout_sec
    while rclpy.ok() and time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        map_odom = node.current_tf_pose("map", "odom")
        odom_base = node.current_tf_pose("odom", "base_link")
        if map_odom and odom_base and node.latest_odom.get("wheel") and node.latest_odom.get("local"):
            return {
                "map_odom": map_odom,
                "odom_base": odom_base,
                "map_base": node.current_tf_pose("map", "base_link"),
                "wheel": dict(node.latest_odom["wheel"]),
                "wheel_ekf": None if node.latest_odom.get("wheel_ekf") is None else dict(node.latest_odom["wheel_ekf"]),
                "local": dict(node.latest_odom["local"]),
            }
    raise RuntimeError("timed out waiting for map->odom, odom->base_link, /wheel/odom, and /local_state/odometry")


def prepare_imu_recording(node):
    if not imu_topic:
        return
    deadline = time.time() + 3.0
    while rclpy.ok() and time.time() < deadline and node.latest_imu is None:
        rclpy.spin_once(node, timeout_sec=0.05)
    if node.latest_imu is None:
        return
    node.configure_imu_projection()
    node.collecting_imu_bias = True
    node.imu_bias_samples = []
    bias_deadline = time.time() + max(0.0, imu_bias_sec)
    while rclpy.ok() and time.time() < bias_deadline:
        rclpy.spin_once(node, timeout_sec=0.03)
    node.collecting_imu_bias = False
    if node.imu_bias_samples:
        node.imu_bias = sum(node.imu_bias_samples) / len(node.imu_bias_samples)
    node.imu_yaw_delta = 0.0
    node.imu_prev_t = None
    node.imu_dt_values = []
    node.imu_sample_count = 0
    node.imu_max_abs_wz_bias_corrected = 0.0
    node.integrating_imu = True


def build_goal_body(building_id, floor_id):
    body = {
        "pose_id": pose_id,
        "building_id": building_id,
        "floor_id": floor_id,
    }
    if goal_completion_policy != "api_default":
        body["goal_completion_policy"] = goal_completion_policy
    return body


def make_sample_fields():
    fields = [
        "rel_time_sec",
        "phase",
        "goal_state",
        "goal_phase",
        "goal_pose_id",
        "api_final_distance_m",
        "api_final_yaw_error_rad",
        "nav2_status",
        "speed_limit_mps",
        "target_odom_x",
        "target_odom_y",
        "target_odom_yaw",
        "local_goal_error_x",
        "local_goal_error_y",
        "local_goal_error_dist",
        "local_goal_error_yaw",
        "wheel_goal_error_x",
        "wheel_goal_error_y",
        "wheel_goal_error_dist",
        "wheel_goal_error_yaw",
        "local_vs_wheel_x",
        "local_vs_wheel_y",
        "local_vs_wheel_dist",
        "local_vs_wheel_yaw",
        "map_odom_x",
        "map_odom_y",
        "map_odom_yaw",
        "tf_odom_base_x",
        "tf_odom_base_y",
        "tf_odom_base_yaw",
        "tf_map_base_x",
        "tf_map_base_y",
        "tf_map_base_yaw",
        "wheel_x",
        "wheel_y",
        "wheel_yaw",
        "wheel_dx",
        "wheel_dy",
        "wheel_dist",
        "wheel_dyaw",
        "wheel_twist_x",
        "wheel_twist_y",
        "wheel_twist_z",
        "local_x",
        "local_y",
        "local_yaw",
        "local_dx",
        "local_dy",
        "local_dist",
        "local_dyaw",
        "local_twist_x",
        "local_twist_y",
        "local_twist_z",
        "imu_wz_base",
        "imu_wz_bias_corrected",
        "imu_yaw_delta",
        "imu_sample_count",
        "wheel_minus_imu_yaw",
        "local_minus_imu_yaw",
        "motion_mode",
        "motion_linear_velocity",
        "motion_lateral_velocity",
        "motion_angular_velocity",
        "motion_steering_angle",
        "system_motion_mode",
        "system_control_mode",
        "actual_mode_code",
        "actual_mode_name",
        "actual_mode_short",
        "mode_aligned",
        "mode_alignment_state",
        "amcl_candidate_count",
        "amcl_accepted_count",
        "amcl_rejected_count",
        "last_candidate_translation_m",
        "last_candidate_yaw_rad",
        "last_accepted_translation_m",
        "last_accepted_yaw_rad",
        "last_accepted_source",
    ]
    for prefix in ("cmd_vel_nav_raw", "cmd_vel_nav", "cmd_vel_collision_checked", "cmd_vel_api", "cmd_vel_safe", "cmd_vel"):
        fields.extend([f"{prefix}_x", f"{prefix}_y", f"{prefix}_z", f"{prefix}_speed"])
    return fields


def fill_pose(row, prefix, pose):
    row[f"{prefix}_x"] = None if pose is None else pose["x"]
    row[f"{prefix}_y"] = None if pose is None else pose["y"]
    row[f"{prefix}_yaw"] = None if pose is None else pose["yaw"]


def fill_error(row, prefix, err):
    row[f"{prefix}_x"] = err["dx"]
    row[f"{prefix}_y"] = err["dy"]
    row[f"{prefix}_dist"] = err["dist"]
    row[f"{prefix}_yaw"] = err["dyaw"]


def sample_row(node, baseline, target_odom, goal_delta):
    node.update_api()
    goal = node.current_goal()
    bridge = node.latest_bridge or {}
    mode_status = parse_mode_status(node.latest_mode_status)
    actual_mode = mode_status.get("actual_motion_mode") if isinstance(mode_status.get("actual_motion_mode"), dict) else {}

    map_odom = node.current_tf_pose("map", "odom")
    tf_odom_base = node.current_tf_pose("odom", "base_link")
    tf_map_base = node.current_tf_pose("map", "base_link")
    wheel_pose = node.latest_odom.get("wheel")
    local_pose = node.latest_odom.get("local")
    wheel_delta = between(baseline.get("wheel"), wheel_pose)
    local_delta = between(baseline.get("local"), local_pose)
    wheel_goal_err = pose_error(wheel_delta, goal_delta)
    local_goal_err = pose_error(local_delta, goal_delta)
    local_vs_wheel = pose_error(wheel_delta, local_delta)
    wheel_twist = node.latest_odom_twist.get("wheel", {})
    local_twist = node.latest_odom_twist.get("local", {})
    imu_yaw = node.imu_yaw_delta if node.imu_sample_count > 0 else None
    wheel_minus_imu = None if wheel_delta is None or imu_yaw is None else norm_angle(wheel_delta["yaw"] - imu_yaw)
    local_minus_imu = None if local_delta is None or imu_yaw is None else norm_angle(local_delta["yaw"] - imu_yaw)

    row = {
        "rel_time_sec": node.rel(),
        "goal_state": goal.get("state"),
        "goal_phase": goal.get("phase"),
        "goal_pose_id": goal.get("pose_id"),
        "api_final_distance_m": goal.get("final_distance_m"),
        "api_final_yaw_error_rad": goal.get("final_yaw_error_rad"),
        "nav2_status": node.last_goal_status,
        "speed_limit_mps": None if node.latest_speed_limit is None else getattr(node.latest_speed_limit, "speed_limit", None),
        "target_odom_x": target_odom["x"],
        "target_odom_y": target_odom["y"],
        "target_odom_yaw": target_odom["yaw"],
        "wheel_dx": None if wheel_delta is None else wheel_delta["x"],
        "wheel_dy": None if wheel_delta is None else wheel_delta["y"],
        "wheel_dist": None if wheel_delta is None else math.hypot(wheel_delta["x"], wheel_delta["y"]),
        "wheel_dyaw": None if wheel_delta is None else wheel_delta["yaw"],
        "wheel_twist_x": wheel_twist.get("x"),
        "wheel_twist_y": wheel_twist.get("y"),
        "wheel_twist_z": wheel_twist.get("z"),
        "local_dx": None if local_delta is None else local_delta["x"],
        "local_dy": None if local_delta is None else local_delta["y"],
        "local_dist": None if local_delta is None else math.hypot(local_delta["x"], local_delta["y"]),
        "local_dyaw": None if local_delta is None else local_delta["yaw"],
        "local_twist_x": local_twist.get("x"),
        "local_twist_y": local_twist.get("y"),
        "local_twist_z": local_twist.get("z"),
        "imu_wz_base": node.imu_wz_base,
        "imu_wz_bias_corrected": node.imu_wz_bias_corrected,
        "imu_yaw_delta": imu_yaw,
        "imu_sample_count": node.imu_sample_count,
        "wheel_minus_imu_yaw": wheel_minus_imu,
        "local_minus_imu_yaw": local_minus_imu,
        "motion_mode": motion_value(node.latest_motion_state, "motion_mode"),
        "motion_linear_velocity": motion_value(node.latest_motion_state, "linear_velocity"),
        "motion_lateral_velocity": motion_value(node.latest_motion_state, "lateral_velocity"),
        "motion_angular_velocity": motion_value(node.latest_motion_state, "angular_velocity"),
        "motion_steering_angle": motion_value(node.latest_motion_state, "steering_angle"),
        "system_motion_mode": motion_value(node.latest_system_state, "motion_mode"),
        "system_control_mode": motion_value(node.latest_system_state, "control_mode"),
        "actual_mode_code": actual_mode.get("code"),
        "actual_mode_name": actual_mode.get("name"),
        "actual_mode_short": actual_mode.get("short"),
        "mode_aligned": mode_status.get("mode_aligned", mode_status.get("motion_mode_matched")),
        "mode_alignment_state": mode_status.get("mode_alignment_state"),
        "amcl_candidate_count": bridge.get("amcl_candidate_count"),
        "amcl_accepted_count": bridge.get("amcl_accepted_count"),
        "amcl_rejected_count": bridge.get("amcl_rejected_count"),
        "last_candidate_translation_m": bridge.get("last_candidate_correction_translation_m"),
        "last_candidate_yaw_rad": bridge.get("last_candidate_correction_yaw_rad"),
        "last_accepted_translation_m": bridge.get("last_accepted_correction_translation_m"),
        "last_accepted_yaw_rad": bridge.get("last_accepted_correction_yaw_rad"),
        "last_accepted_source": bridge.get("last_accepted_source"),
    }
    fill_pose(row, "map_odom", map_odom)
    fill_pose(row, "tf_odom_base", tf_odom_base)
    fill_pose(row, "tf_map_base", tf_map_base)
    fill_pose(row, "wheel", wheel_pose)
    fill_pose(row, "local", local_pose)
    fill_error(row, "wheel_goal_error", wheel_goal_err)
    fill_error(row, "local_goal_error", local_goal_err)
    fill_error(row, "local_vs_wheel", local_vs_wheel)

    for topic, prefix in (
        ("/cmd_vel_nav_raw", "cmd_vel_nav_raw"),
        ("/cmd_vel_nav", "cmd_vel_nav"),
        ("/cmd_vel_collision_checked", "cmd_vel_collision_checked"),
        ("/cmd_vel_api", "cmd_vel_api"),
        ("/cmd_vel_safe", "cmd_vel_safe"),
        ("/cmd_vel", "cmd_vel"),
    ):
        values = twist_values(node.latest_twist.get(topic))
        row[f"{prefix}_x"] = values["x"]
        row[f"{prefix}_y"] = values["y"]
        row[f"{prefix}_z"] = values["z"]
        row[f"{prefix}_speed"] = values["speed"]

    row["phase"] = classify_phase(row)
    return row


def summarize_segments(rows, baseline):
    segments = []
    current = None
    last_row = None
    for row in rows:
        if current is None or row["phase"] != current["phase"]:
            if current is not None and last_row is not None:
                current["end_s"] = safe_float(last_row.get("rel_time_sec"), current["start_s"])
                current["end_row"] = last_row
                segments.append(current)
            current = {"phase": row["phase"], "start_s": safe_float(row.get("rel_time_sec"), 0.0), "start_row": row}
        last_row = row
    if current is not None and last_row is not None:
        current["end_s"] = safe_float(last_row.get("rel_time_sec"), current["start_s"])
        current["end_row"] = last_row
        segments.append(current)

    result = []
    for idx, segment in enumerate(segments, start=1):
        start = segment["start_row"]
        end = segment["end_row"]
        result.append({
            "segment": idx,
            "phase": segment["phase"],
            "start_s": segment["start_s"],
            "end_s": segment["end_s"],
            "duration_s": segment["end_s"] - segment["start_s"],
            "wheel_dist_start": safe_float(start.get("wheel_dist")),
            "wheel_dist_end": safe_float(end.get("wheel_dist")),
            "wheel_dist_delta": (safe_float(end.get("wheel_dist"), 0.0) or 0.0) - (safe_float(start.get("wheel_dist"), 0.0) or 0.0),
            "local_dist_start": safe_float(start.get("local_dist")),
            "local_dist_end": safe_float(end.get("local_dist")),
            "local_dist_delta": (safe_float(end.get("local_dist"), 0.0) or 0.0) - (safe_float(start.get("local_dist"), 0.0) or 0.0),
            "wheel_dyaw_delta": norm_angle((safe_float(end.get("wheel_dyaw"), 0.0) or 0.0) - (safe_float(start.get("wheel_dyaw"), 0.0) or 0.0)),
            "local_dyaw_delta": norm_angle((safe_float(end.get("local_dyaw"), 0.0) or 0.0) - (safe_float(start.get("local_dyaw"), 0.0) or 0.0)),
            "imu_yaw_delta": norm_angle((safe_float(end.get("imu_yaw_delta"), 0.0) or 0.0) - (safe_float(start.get("imu_yaw_delta"), 0.0) or 0.0)),
            "wheel_minus_imu_end_yaw": safe_float(end.get("wheel_minus_imu_yaw")),
            "local_minus_imu_end_yaw": safe_float(end.get("local_minus_imu_yaw")),
            "local_vs_wheel_end_m": safe_float(end.get("local_vs_wheel_dist")),
            "local_goal_error_end_m": safe_float(end.get("local_goal_error_dist")),
            "wheel_goal_error_end_m": safe_float(end.get("wheel_goal_error_dist")),
            "max_abs_cmd_wz": max(abs(safe_float(r.get("cmd_vel_z"), 0.0) or 0.0) for r in rows if segment["start_s"] <= (safe_float(r.get("rel_time_sec"), -1.0) or -1.0) <= segment["end_s"]),
            "max_abs_cmd_vx": max(abs(safe_float(r.get("cmd_vel_x"), 0.0) or 0.0) for r in rows if segment["start_s"] <= (safe_float(r.get("rel_time_sec"), -1.0) or -1.0) <= segment["end_s"]),
        })
    return result


def write_csv(path, rows, fields):
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        for row in rows:
            writer.writerow({key: fmt(row.get(key)) for key in fields})


def pose_dict_for_json(pose):
    if pose is None:
        return None
    return {"x": pose["x"], "y": pose["y"], "yaw": pose["yaw"]}


rclpy.init()
node = Probe()
exit_code = 0
try:
    resolved_building_id, resolved_floor_id, target_map, precheck = resolve_target_pose()
    (output_dir / "target_pre_goal_check.json").write_text(
        json.dumps(precheck, indent=2, sort_keys=True, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )

    # Warm the TF listener before IMU projection; otherwise a fresh listener can
    # miss static base_link->imu_link during the short IMU bias window.
    wait_for_baseline(node, timeout_sec=8.0)
    prepare_imu_recording(node)
    baseline = wait_for_baseline(node, timeout_sec=8.0)
    node.imu_yaw_delta = 0.0
    node.imu_prev_t = None
    node.imu_dt_values = []
    node.imu_sample_count = 0
    node.imu_max_abs_wz_bias_corrected = 0.0
    target_odom = between(baseline["map_odom"], target_map)
    goal_delta = between(baseline["local"], target_odom)
    if target_odom is None or goal_delta is None:
        raise RuntimeError("failed to compute target odom pose")

    baseline_payload = {
        "building_id": resolved_building_id,
        "floor_id": resolved_floor_id,
        "pose_id": pose_id,
        "target_map": pose_dict_for_json(target_map),
        "target_odom_start": pose_dict_for_json(target_odom),
        "goal_delta_from_local_start": pose_dict_for_json(goal_delta),
        "baseline": {key: pose_dict_for_json(value) for key, value in baseline.items()},
        "imu": {
            "topic": imu_topic,
            "frame": node.imu_frame,
            "bias_radps": node.imu_bias,
            "bias_sample_count": len(node.imu_bias_samples),
            "yaw_rate_source": node.imu_yaw_rate_source,
            "project_imu_to_base": project_imu_to_base,
            "base_frame": base_frame,
        },
    }
    (output_dir / "baseline.json").write_text(
        json.dumps(baseline_payload, indent=2, sort_keys=True, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )

    goal_response = None
    if send_goal:
        body = build_goal_body(resolved_building_id, resolved_floor_id)
        (output_dir / "goal_request.json").write_text(
            json.dumps(body, indent=2, sort_keys=True, ensure_ascii=False) + "\n",
            encoding="utf-8",
        )
        goal_response = api_request("POST", "/api/v1/navigation/goal", body=body, timeout=30.0)
        (output_dir / "goal_response.json").write_text(
            json.dumps(goal_response, indent=2, sort_keys=True, ensure_ascii=False) + "\n",
            encoding="utf-8",
        )
        status = int(goal_response.get("_http_status") or 0) if isinstance(goal_response, dict) else 0
        if status >= 400 or not (isinstance(goal_response, dict) and goal_response.get("accepted", goal_response.get("ok", False))):
            raise RuntimeError(f"navigation goal was not accepted: {goal_response}")

    fields = make_sample_fields()
    rows = []
    api_samples = []
    deadline = time.time() + duration_sec
    next_sample = time.time()
    while rclpy.ok() and time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.03)
        now = time.time()
        if now >= next_sample:
            row = sample_row(node, baseline, target_odom, goal_delta)
            rows.append(row)
            api_samples.append({
                "rel_time_sec": row["rel_time_sec"],
                "navigation_state": node.last_nav_state,
                "robot_pose": node.last_robot_pose,
            })
            next_sample = now + sample_period_sec
        if stop_when_terminal and node.terminal_seen_wall is not None and now - node.terminal_seen_wall >= 1.5:
            break

    write_csv(output_dir / "samples.csv", rows, fields)
    (output_dir / "api_state_samples.jsonl").write_text(
        "".join(json.dumps(sample, ensure_ascii=False, sort_keys=True) + "\n" for sample in api_samples),
        encoding="utf-8",
    )
    segments = summarize_segments(rows, baseline)
    segment_fields = [
        "segment",
        "phase",
        "start_s",
        "end_s",
        "duration_s",
        "wheel_dist_start",
        "wheel_dist_end",
        "wheel_dist_delta",
        "local_dist_start",
        "local_dist_end",
        "local_dist_delta",
        "wheel_dyaw_delta",
        "local_dyaw_delta",
        "imu_yaw_delta",
        "wheel_minus_imu_end_yaw",
        "local_minus_imu_end_yaw",
        "local_vs_wheel_end_m",
        "local_goal_error_end_m",
        "wheel_goal_error_end_m",
        "max_abs_cmd_wz",
        "max_abs_cmd_vx",
    ]
    write_csv(output_dir / "segments.csv", segments, segment_fields)

    final = rows[-1] if rows else {}
    first_bridge = {}
    last_bridge = node.latest_bridge or {}
    imu_dt_sum = sum(node.imu_dt_values)
    imu_integrated_rate_hz = len(node.imu_dt_values) / imu_dt_sum if imu_dt_sum > 0.0 else None
    summary = {
        "pose_id": pose_id,
        "building_id": resolved_building_id,
        "floor_id": resolved_floor_id,
        "duration_sec": rows[-1]["rel_time_sec"] if rows else 0.0,
        "sample_count": len(rows),
        "send_goal": send_goal,
        "goal_response": goal_response,
        "target_map": target_map,
        "target_odom_start": target_odom,
        "goal_delta_from_local_start": goal_delta,
        "baseline": baseline_payload["baseline"],
        "final": {
            "goal_state": final.get("goal_state"),
            "goal_phase": final.get("goal_phase"),
            "api_final_distance_m": safe_float(final.get("api_final_distance_m")),
            "api_final_yaw_error_rad": safe_float(final.get("api_final_yaw_error_rad")),
            "wheel_goal_error_dist_m": safe_float(final.get("wheel_goal_error_dist")),
            "wheel_goal_error_yaw_rad": safe_float(final.get("wheel_goal_error_yaw")),
            "local_goal_error_dist_m": safe_float(final.get("local_goal_error_dist")),
            "local_goal_error_yaw_rad": safe_float(final.get("local_goal_error_yaw")),
            "local_vs_wheel_dist_m": safe_float(final.get("local_vs_wheel_dist")),
            "local_vs_wheel_yaw_rad": safe_float(final.get("local_vs_wheel_yaw")),
            "wheel_dist_m": safe_float(final.get("wheel_dist")),
            "local_dist_m": safe_float(final.get("local_dist")),
            "wheel_dyaw_rad": safe_float(final.get("wheel_dyaw")),
            "local_dyaw_rad": safe_float(final.get("local_dyaw")),
            "imu_yaw_delta_rad": safe_float(final.get("imu_yaw_delta")),
            "wheel_minus_imu_yaw_rad": safe_float(final.get("wheel_minus_imu_yaw")),
            "local_minus_imu_yaw_rad": safe_float(final.get("local_minus_imu_yaw")),
            "map_odom_x": safe_float(final.get("map_odom_x")),
            "map_odom_y": safe_float(final.get("map_odom_y")),
            "map_odom_yaw": safe_float(final.get("map_odom_yaw")),
        },
        "imu": {
            "topic": imu_topic,
            "frame": node.imu_frame,
            "bias_radps": node.imu_bias,
            "bias_sample_count": len(node.imu_bias_samples),
            "yaw_rate_source": node.imu_yaw_rate_source,
            "integrated_rate_hz": imu_integrated_rate_hz,
            "sample_count_recorded": node.imu_sample_count,
            "max_abs_wz_bias_corrected_radps": node.imu_max_abs_wz_bias_corrected,
        },
        "bridge": {
            "amcl_candidate_count": last_bridge.get("amcl_candidate_count"),
            "amcl_accepted_count": last_bridge.get("amcl_accepted_count"),
            "amcl_rejected_count": last_bridge.get("amcl_rejected_count"),
            "last_candidate_correction_translation_m": last_bridge.get("last_candidate_correction_translation_m"),
            "last_candidate_correction_yaw_rad": last_bridge.get("last_candidate_correction_yaw_rad"),
            "last_accepted_correction_translation_m": last_bridge.get("last_accepted_correction_translation_m"),
            "last_accepted_correction_yaw_rad": last_bridge.get("last_accepted_correction_yaw_rad"),
            "last_accepted_source": last_bridge.get("last_accepted_source"),
            "amcl_gate_mode": last_bridge.get("amcl_gate_mode"),
            "amcl_shadow_mode": last_bridge.get("amcl_shadow_mode"),
        },
        "segments": segments,
    }
    (output_dir / "summary.json").write_text(
        json.dumps(summary, indent=2, sort_keys=True, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )

    def deg(rad):
        if rad is None:
            return None
        return rad * 180.0 / math.pi

    lines = [
        "# Navigation Odom Goal Closure",
        "",
        f"- report_dir: `{output_dir}`",
        f"- pose_id: `{pose_id}`",
        f"- building_id/floor_id: `{resolved_building_id}/{resolved_floor_id}`",
        f"- send_goal: `{send_goal}`",
        f"- samples: `{len(rows)}`",
        f"- duration_sec: `{summary['duration_sec']:.3f}`",
        "",
        "## Target",
        "",
        f"- target_map: `{target_map}`",
        f"- target_odom_start: `{target_odom}`",
        f"- goal_delta_from_local_start: `{goal_delta}`",
        "",
        "## Final Closure",
        "",
        "| metric | value |",
        "|---|---:|",
        f"| wheel_goal_error_dist_m | {fmt(summary['final']['wheel_goal_error_dist_m'])} |",
        f"| wheel_goal_error_yaw_deg | {fmt(deg(summary['final']['wheel_goal_error_yaw_rad']))} |",
        f"| local_goal_error_dist_m | {fmt(summary['final']['local_goal_error_dist_m'])} |",
        f"| local_goal_error_yaw_deg | {fmt(deg(summary['final']['local_goal_error_yaw_rad']))} |",
        f"| local_vs_wheel_dist_m | {fmt(summary['final']['local_vs_wheel_dist_m'])} |",
        f"| local_vs_wheel_yaw_deg | {fmt(deg(summary['final']['local_vs_wheel_yaw_rad']))} |",
        f"| wheel_dist_m | {fmt(summary['final']['wheel_dist_m'])} |",
        f"| local_dist_m | {fmt(summary['final']['local_dist_m'])} |",
        f"| wheel_dyaw_deg | {fmt(deg(summary['final']['wheel_dyaw_rad']))} |",
        f"| local_dyaw_deg | {fmt(deg(summary['final']['local_dyaw_rad']))} |",
        f"| imu_yaw_delta_deg | {fmt(deg(summary['final']['imu_yaw_delta_rad']))} |",
        f"| wheel_minus_imu_yaw_deg | {fmt(deg(summary['final']['wheel_minus_imu_yaw_rad']))} |",
        f"| local_minus_imu_yaw_deg | {fmt(deg(summary['final']['local_minus_imu_yaw_rad']))} |",
        f"| api_final_distance_m | {fmt(summary['final']['api_final_distance_m'])} |",
        f"| api_final_yaw_error_deg | {fmt(deg(summary['final']['api_final_yaw_error_rad']))} |",
        "",
        "## IMU Diagnostic",
        "",
        "| field | value |",
        "|---|---|",
    ]
    for key, value in summary["imu"].items():
        lines.append(f"| {key} | `{value}` |")

    lines.extend([
        "",
        "## Bridge Snapshot",
        "",
        "| field | value |",
        "|---|---|",
    ])
    for key, value in summary["bridge"].items():
        lines.append(f"| {key} | `{value}` |")

    lines.extend([
        "",
        "## Segments",
        "",
        "| # | phase | start_s | end_s | duration_s | wheel_delta_m | local_delta_m | wheel_dyaw_deg | local_dyaw_deg | imu_yaw_deg | wheel_minus_imu_end_deg | local_vs_wheel_end_m | local_goal_error_end_m | wheel_goal_error_end_m |",
        "|---:|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|",
    ])
    for seg in segments:
        lines.append(
            "| "
            + " | ".join(
                [
                    str(seg["segment"]),
                    str(seg["phase"]),
                    fmt(seg["start_s"]),
                    fmt(seg["end_s"]),
                    fmt(seg["duration_s"]),
                    fmt(seg["wheel_dist_delta"]),
                    fmt(seg["local_dist_delta"]),
                    fmt(deg(seg["wheel_dyaw_delta"])),
                    fmt(deg(seg["local_dyaw_delta"])),
                    fmt(deg(seg["imu_yaw_delta"])),
                    fmt(deg(seg["wheel_minus_imu_end_yaw"])),
                    fmt(seg["local_vs_wheel_end_m"]),
                    fmt(seg["local_goal_error_end_m"]),
                    fmt(seg["wheel_goal_error_end_m"]),
                ]
            )
            + " |"
        )

    lines.extend([
        "",
        "Files:",
        "",
        "- `baseline.json`",
        "- `goal_request.json`",
        "- `goal_response.json`",
        "- `samples.csv`",
        "- `segments.csv`",
        "- `api_state_samples.jsonl`",
        "- `summary.json`",
    ])
    (output_dir / "summary.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
finally:
    node.destroy_node()
    if rclpy.ok():
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
