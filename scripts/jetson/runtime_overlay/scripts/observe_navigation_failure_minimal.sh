#!/usr/bin/env bash
set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export NJRH_NAV_OBSERVER_MODULE_DIR="${SCRIPT_DIR}"
WORKSPACE_ROOT="${WORKSPACE_ROOT:-$(cd "${SCRIPT_DIR}/../../../.." && pwd)}"
export NJRH_PROJECT_ROOT="${NJRH_PROJECT_ROOT:-${WORKSPACE_ROOT}}"
source "${SCRIPT_DIR}/common_env.sh"
set +e

API_URL="${API_URL:-http://127.0.0.1:8080}"
DURATION_SEC=120
SAMPLE_PERIOD_SEC=1.0
LABEL="nav_failure_minimal"
OUTPUT_DIR=""
INCLUDE_ROSOUT=true
INCLUDE_CMD_VEL=true
INCLUDE_PERCEPTION_STATUS=false
INCLUDE_CONTROLLER_DETAIL=true
STORE_COSTMAP_SNAPSHOTS=false
COSTMAP_SNAPSHOT_PERIOD_SEC=0.5
ACKERMANN_MIN_TURNING_RADIUS_M="${ACKERMANN_MIN_TURNING_RADIUS_M:-0.81}"
PREFIX="[nav-failure-minimal]"

usage() {
  cat <<'EOF'
Usage:
  bash scripts/jetson/runtime_overlay/scripts/observe_navigation_failure_minimal.sh --duration-sec 180 --label nav_fail_1

Start this script first, then send one navigation goal from the App.

Read-only, low-impact observer:
  - creates one temporary rclpy participant
  - does not publish topics, send actions, call services, set params, or restart nodes
  - does not subscribe to /tf, PointCloud2, or LaserScan
  - records status/action/string/Twist messages, filtered /rosout, and controller detail summaries
  - controller detail mode records /speed_limit, Path summaries, and local costmap occupancy summaries
  - v3 records stamped Nav2 feedback, available collision state, log source stamps and low-rate process counters
  - missing MPPI internal rejection reasons are explicitly reported; no visualization is enabled

Options:
  --duration-sec N              Capture duration in seconds. Default: 120.
  --sample-period-sec N         Summary sample period. Default: 1.0.
  --label LABEL                 Report label. Default: nav_failure_minimal.
  --api-url URL                 robot_api_server URL. Default: http://127.0.0.1:8080.
  --output-dir DIR              New report directory (must not exist). Default: unique /tmp/njrh_reports/navigation_obstacle_<timestamp>_<label>_*.
  --no-rosout                   Do not subscribe to /rosout.
  --no-cmd-vel                  Do not subscribe to command-chain Twist topics.
  --include-perception-status   Also record lightweight status strings from perception/lidar status topics.
  --no-controller-detail        Do not subscribe to /speed_limit, Nav2 Path topics, or /local_costmap/costmap.
  --store-costmap-snapshots     Store an exact, sampled local-costmap timeline for later rendering.
  --costmap-snapshot-period-sec N
                                Full-grid snapshot period. Default: 0.5 s (2 Hz).
  --ackermann-min-radius M      Radius used only for cmd-shape diagnostics. Default: 0.81.
  -h, --help                    Show this help.
EOF
}

sanitize_label() {
  printf '%s' "$1" | tr -c 'A-Za-z0-9_.-' '_'
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
    --no-cmd-vel)
      INCLUDE_CMD_VEL=false
      shift
      ;;
    --include-perception-status)
      INCLUDE_PERCEPTION_STATUS=true
      shift
      ;;
    --no-controller-detail)
      INCLUDE_CONTROLLER_DETAIL=false
      shift
      ;;
    --store-costmap-snapshots)
      STORE_COSTMAP_SNAPSHOTS=true
      shift
      ;;
    --costmap-snapshot-period-sec)
      COSTMAP_SNAPSHOT_PERIOD_SEC="${2:-}"
      shift 2
      ;;
    --ackermann-min-radius)
      ACKERMANN_MIN_TURNING_RADIUS_M="${2:-}"
      shift 2
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

if ! [[ "${SAMPLE_PERIOD_SEC}" =~ ^[0-9]+([.][0-9]+)?$ ]]; then
  echo "${PREFIX} FAIL --sample-period-sec must be numeric" >&2
  exit 2
fi

if ! [[ "${ACKERMANN_MIN_TURNING_RADIUS_M}" =~ ^[0-9]+([.][0-9]+)?$ ]]; then
  echo "${PREFIX} FAIL --ackermann-min-radius must be numeric" >&2
  exit 2
fi

if ! [[ "${COSTMAP_SNAPSHOT_PERIOD_SEC}" =~ ^[0-9]+([.][0-9]+)?$ ]]; then
  echo "${PREFIX} FAIL --costmap-snapshot-period-sec must be numeric" >&2
  exit 2
fi

if [[ "${STORE_COSTMAP_SNAPSHOTS}" == "true" && "${INCLUDE_CONTROLLER_DETAIL}" != "true" ]]; then
  echo "${PREFIX} FAIL --store-costmap-snapshots cannot be combined with --no-controller-detail" >&2
  exit 2
fi

TIMESTAMP="$(date -u +%Y%m%dT%H%M%SZ)"
LABEL="$(sanitize_label "${LABEL}")"
if [[ -z "${OUTPUT_DIR}" ]]; then
  mkdir -p /tmp/njrh_reports || exit 1
  OUTPUT_DIR="$(mktemp -d "/tmp/njrh_reports/navigation_obstacle_${TIMESTAMP}_${LABEL}_XXXXXX")" || exit 1
else
  # Never mix goals from different runs or overwrite numbered grid snapshots.
  mkdir -p "$(dirname "${OUTPUT_DIR}")" || exit 1
  if ! mkdir "${OUTPUT_DIR}"; then
    echo "${PREFIX} FAIL report directory already exists or cannot be created: ${OUTPUT_DIR}" >&2
    exit 2
  fi
fi

# The runtime API normally keeps its token only in the server process environment.
# Reuse it for read-only status polling when this observer is run as container root.
if [[ -z "${ROBOT_API_TOKEN:-}" ]] && command -v pgrep >/dev/null 2>&1; then
  API_SERVER_PID="$(pgrep -fn '[r]obot_api_server_node' 2>/dev/null || true)"
  if [[ -n "${API_SERVER_PID}" && -r "/proc/${API_SERVER_PID}/environ" ]]; then
    ROBOT_API_TOKEN="$(
      tr '\0' '\n' < "/proc/${API_SERVER_PID}/environ" \
        | sed -n 's/^ROBOT_API_TOKEN=//p' \
        | head -n 1
    )"
    export ROBOT_API_TOKEN
  fi
fi

echo "${PREFIX} report_dir=${OUTPUT_DIR}"
echo "${PREFIX} duration_sec=${DURATION_SEC} sample_period_sec=${SAMPLE_PERIOD_SEC}"
echo "${PREFIX} controller_detail=${INCLUDE_CONTROLLER_DETAIL} ackermann_min_radius_m=${ACKERMANN_MIN_TURNING_RADIUS_M}"
echo "${PREFIX} store_costmap_snapshots=${STORE_COSTMAP_SNAPSHOTS} costmap_snapshot_period_sec=${COSTMAP_SNAPSHOT_PERIOD_SEC}"
echo "${PREFIX} read-only: no goals, no params, no services, no /tf, no PointCloud2, no LaserScan"
echo "${PREFIX} wait for READY before starting one App navigation goal"

python3 - \
  "${DURATION_SEC}" \
  "${SAMPLE_PERIOD_SEC}" \
  "${API_URL}" \
  "${OUTPUT_DIR}" \
  "${INCLUDE_ROSOUT}" \
  "${INCLUDE_CMD_VEL}" \
  "${INCLUDE_PERCEPTION_STATUS}" \
  "${INCLUDE_CONTROLLER_DETAIL}" \
  "${ACKERMANN_MIN_TURNING_RADIUS_M}" \
  "${STORE_COSTMAP_SNAPSHOTS}" \
  "${COSTMAP_SNAPSHOT_PERIOD_SEC}" <<'PY'
import gzip
import hashlib
import json
import math
import os
import re
import signal
import sys
import time
import urllib.error
import urllib.request
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

sys.path.insert(0, os.environ["NJRH_NAV_OBSERVER_MODULE_DIR"])
from navigation_observer_evidence import NavigationEvidence

import rclpy
from action_msgs.msg import GoalStatusArray
from geometry_msgs.msg import PolygonStamped, Twist
from rcl_interfaces.msg import Log
from nav2_msgs.msg import SpeedLimit
from nav2_msgs.action import FollowPath, NavigateToPose
try:
    from nav2_msgs.msg import CollisionMonitorState
except ImportError:
    CollisionMonitorState = None
from nav_msgs.msg import OccupancyGrid, Odometry, Path as NavPath
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Bool, String, UInt8


duration_sec = float(sys.argv[1])
sample_period_sec = float(sys.argv[2])
api_url = sys.argv[3].rstrip("/")
output_dir = Path(sys.argv[4])
include_rosout = sys.argv[5].lower() == "true"
include_cmd_vel = sys.argv[6].lower() == "true"
include_perception_status = sys.argv[7].lower() == "true"
include_controller_detail = sys.argv[8].lower() == "true"
ackermann_min_turning_radius_m = float(sys.argv[9])
store_costmap_snapshots = sys.argv[10].lower() == "true"
costmap_snapshot_period_sec = max(0.2, float(sys.argv[11]))
api_token = os.environ.get("ROBOT_API_TOKEN", "").strip()

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

ROSOUT_LEVELS = {
    10: "DEBUG",
    20: "INFO",
    30: "WARN",
    40: "ERROR",
    50: "FATAL",
}

ROSOUT_FILTER = re.compile(
    r"controller_server|bt_navigator|planner_server|local_costmap|global_costmap|"
    r"costmap|followpath|follow_path|compute_path|navigate|abort|aborted|failed|"
    r"failure|exception|extrapolat|transform|message filter|collision|progress|"
    r"oscillat|safety|cmd_vel|goal|cancel|bridge|localization|amcl|"
    r"speed.?filter|speed.?limit|mppi|rotation.?shim|critic|trajectory",
    re.IGNORECASE,
)

BRIDGE_KEYS = (
    "safe_for_goal_start",
    "correction_active",
    "localization_degraded",
    "amcl_ready",
    "amcl_correction_ready",
    "amcl_correction_pending",
    "has_map_to_odom",
    "latest_odom_tf_fresh",
    "map_odom_publish_loop_hz",
    "map_odom_publish_gap_ms",
    "map_odom_publish_gap_max_ms",
    "remaining_translation_error_m",
    "remaining_yaw_error_rad",
    "last_accept_reason",
)


def now_iso():
    wall = time.time()
    whole = int(wall)
    millis = int((wall - whole) * 1000.0)
    return f"{time.strftime('%Y-%m-%dT%H:%M:%S', time.gmtime(whole))}.{millis:03d}Z"


def quaternion_yaw(orientation):
    siny_cosp = 2.0 * (
        float(orientation.w) * float(orientation.z)
        + float(orientation.x) * float(orientation.y)
    )
    cosy_cosp = 1.0 - 2.0 * (
        float(orientation.y) * float(orientation.y)
        + float(orientation.z) * float(orientation.z)
    )
    return math.atan2(siny_cosp, cosy_cosp)


def short_uuid(uuid_msg):
    try:
        values = [int(x) for x in uuid_msg.uuid]
        return "".join(f"{x:02x}" for x in values)[:12]
    except Exception:
        return "unknown"


def twist_dict(msg):
    return {
        "linear": {"x": msg.linear.x, "y": msg.linear.y, "z": msg.linear.z},
        "angular": {"x": msg.angular.x, "y": msg.angular.y, "z": msg.angular.z},
    }


def twist_nonzero(msg):
    return (
        abs(msg.linear.x) > 1.0e-4
        or abs(msg.linear.y) > 1.0e-4
        or abs(msg.linear.z) > 1.0e-4
        or abs(msg.angular.x) > 1.0e-4
        or abs(msg.angular.y) > 1.0e-4
        or abs(msg.angular.z) > 1.0e-4
    )


def command_shape(msg):
    vx = float(msg.linear.x)
    vy = float(msg.linear.y)
    wz = float(msg.angular.z)
    abs_vx = abs(vx)
    abs_wz = abs(wz)
    if abs(vy) > 1.0e-4:
        return {
            "shape": "lateral" if abs_vx <= 1.0e-4 and abs_wz <= 1.0e-4 else "mixed_lateral",
            "turning_radius_m": None,
            "ackermann_radius_ok": None,
        }
    if abs_vx <= 1.0e-4 and abs_wz <= 1.0e-4:
        return {
            "shape": "zero",
            "turning_radius_m": None,
            "ackermann_radius_ok": True,
        }
    if abs_vx <= 1.0e-4 and abs_wz > 1.0e-4:
        return {
            "shape": "pure_yaw",
            "turning_radius_m": 0.0,
            "ackermann_radius_ok": False,
        }
    if abs_wz <= 1.0e-4:
        return {
            "shape": "straight",
            "turning_radius_m": None,
            "ackermann_radius_ok": True,
        }
    radius = abs_vx / abs_wz
    return {
        "shape": (
            "ackermann_too_tight"
            if radius < ackermann_min_turning_radius_m
            else "ackermann_feasible"
        ),
        "turning_radius_m": radius,
        "ackermann_radius_ok": radius >= ackermann_min_turning_radius_m,
    }


def pose_summary(pose_stamped):
    pose = pose_stamped.pose
    return {
        "frame_id": pose_stamped.header.frame_id,
        "stamp_sec": int(pose_stamped.header.stamp.sec),
        "stamp_nanosec": int(pose_stamped.header.stamp.nanosec),
        "x": float(pose.position.x),
        "y": float(pose.position.y),
        "z": float(pose.position.z),
        "qx": float(pose.orientation.x),
        "qy": float(pose.orientation.y),
        "qz": float(pose.orientation.z),
        "qw": float(pose.orientation.w),
    }


def path_length_m(path_msg):
    total = 0.0
    poses = path_msg.poses
    for prev, cur in zip(poses, poses[1:]):
        dx = float(cur.pose.position.x - prev.pose.position.x)
        dy = float(cur.pose.position.y - prev.pose.position.y)
        total += math.hypot(dx, dy)
    return total


def speed_limit_dict(msg):
    return {
        "stamp_sec": int(msg.header.stamp.sec),
        "stamp_nanosec": int(msg.header.stamp.nanosec),
        "frame_id": msg.header.frame_id,
        "percentage": bool(msg.percentage),
        "speed_limit": float(msg.speed_limit),
    }


def mode_snapshot(status):
    parsed = status.get("json") if isinstance(status, dict) else None
    if not isinstance(parsed, dict):
        parsed = parse_json_maybe(status.get("data")) if isinstance(status, dict) else None
    if not isinstance(parsed, dict):
        return None
    desired = parsed.get("desired_motion_mode") or {}
    actual = parsed.get("actual_motion_mode") or {}
    return {
        "desired_mode": parsed.get("desired_mode"),
        "desired_code": desired.get("code"),
        "desired_name": desired.get("name"),
        "actual_code": actual.get("code"),
        "actual_name": actual.get("name"),
        "actual_source": actual.get("source"),
        "mode_aligned": parsed.get("mode_aligned"),
        "motion_mode_matched": parsed.get("motion_mode_matched"),
        "mode_alignment_state": parsed.get("mode_alignment_state"),
    }


def parse_json_maybe(text):
    if not isinstance(text, str):
        return None
    stripped = text.strip()
    if not stripped.startswith("{"):
        return None
    try:
        return json.loads(stripped)
    except Exception:
        return None


def save_runtime_snapshot():
    """File/process evidence only: no ROS parameter clients or runtime writes."""
    names = ("controller_server", "planner_server", "bt_navigator", "velocity_smoother",
             "collision_monitor", "robot_safety_node", "robot_api_server_node", "ranger_base_node")
    result = {"captured_at": now_iso(), "processes": {name: [] for name in names}}
    for proc in Path("/proc").iterdir():
        if not proc.name.isdigit():
            continue
        try:
            executable = os.readlink(proc / "exe")
            name = Path(executable).name
            if name not in names:
                continue
            entry = {"pid": int(proc.name), "executable": executable}
            # Never persist process environments or API command lines/tokens.
            if name in ("controller_server", "velocity_smoother", "collision_monitor"):
                args = (proc / "cmdline").read_bytes().decode(errors="replace").split("\0")
                entry["parameter_files"] = []
                for index, arg in enumerate(args[:-1]):
                    if arg == "--params-file":
                        path = Path(args[index + 1])
                        content = path.read_bytes()
                        target = f"{name}_{proc.name}_params_{index}.yaml"
                        (output_dir / target).write_bytes(content)
                        entry["parameter_files"].append({"source": str(path), "file": target,
                                                        "sha256": hashlib.sha256(content).hexdigest()})
            result["processes"][name].append(entry)
        except (OSError, ValueError) as exc:
            result.setdefault("read_errors", []).append({"pid": proc.name, "error": str(exc)})
    (output_dir / "runtime_snapshot.json").write_text(json.dumps(result, indent=2) + "\n")


class MinimalNavigationObserver(Node):
    def __init__(self):
        super().__init__("minimal_navigation_failure_observer", enable_rosout=False,
                         start_parameter_services=False)
        self.started_wall = time.time()
        self.started_monotonic = time.monotonic()
        self.deadline_monotonic = self.started_monotonic + duration_sec
        self.stop_reason = "duration_complete"
        self.deadline_wall = self.started_wall + duration_sec
        self.samples_path = output_dir / "samples.jsonl"
        self.api_path = output_dir / "api_poll.jsonl"
        self.events_path = output_dir / "events.jsonl"
        self.cmd_frames_path = output_dir / "cmd_frames.jsonl"
        self.rosout_path = output_dir / "rosout_filtered.log"
        self.samples_file = self.samples_path.open("a", encoding="utf-8")
        self.api_file = self.api_path.open("a", encoding="utf-8")
        self.events_file = self.events_path.open("a", encoding="utf-8")
        self.cmd_frames_file = self.cmd_frames_path.open("a", encoding="utf-8")
        self.path_frames_file = gzip.open(output_dir / "path_frames.jsonl.gz", "at", encoding="utf-8")
        self.rosout_file = self.rosout_path.open("a", encoding="utf-8")
        self.costmap_snapshot_dir = output_dir / "costmap_snapshots"
        self.costmap_index_file = None
        if store_costmap_snapshots:
            self.costmap_snapshot_dir.mkdir(parents=True, exist_ok=True)
            self.costmap_index_file = (
                self.costmap_snapshot_dir / "index.jsonl"
            ).open("a", encoding="utf-8")

        self.action_status = {}
        self.action_status_keys = {}
        self.string_status = {}
        self.string_status_last = {}
        self.scalar_status = {}
        self.latest_footprint = None
        self.last_footprint_monotonic = -math.inf
        self.path_fingerprints = {}
        self.path_last_saved = {}
        self.twist_stats = {}
        self.latest_api_status = None
        self.latest_api_navigation = None
        self.api_errors = []
        self.api_worker = ThreadPoolExecutor(max_workers=1)
        self.api_future = None
        self.rosout_hits = 0
        self.rosout_tail = []
        self.event_counts = {}
        self.evidence = NavigationEvidence(output_dir)
        self.collision_state_subscriptions = {}
        self.speed_limit_stats = {
            "count": 0,
            "last_msg_at": None,
            "last": None,
            "min_speed_limit": None,
            "max_speed_limit": None,
            "zero_or_near_zero_count": 0,
        }
        self.path_stats = {}
        self.latest_path_geometry = {}
        self.latest_odometry = None
        self.last_costmap_snapshot_monotonic = None
        self.costmap_snapshot_count = 0
        self.local_costmap_stats = {
            "count": 0,
            "last_msg_at": None,
            "latest": None,
            "near_robot_occupied_count": 0,
            "near_robot_lethal_count": 0,
        }
        self.cmd_shape_stats = {}

        small_qos = QoSProfile(depth=10)
        retained_state_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                       durability=DurabilityPolicy.TRANSIENT_LOCAL)
        rosout_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=100,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )

        for topic in (
            "/navigate_to_pose/_action/status",
            "/compute_path_to_pose/_action/status",
            "/follow_path/_action/status",
        ):
            self.create_subscription(
                GoalStatusArray,
                topic,
                lambda msg, topic=topic: self.on_action_status(topic, msg),
                small_qos,
            )

        # Feedback subscription only; no ActionClient or goal request is created.
        for action, action_type in (("navigate_to_pose", NavigateToPose), ("follow_path", FollowPath)):
            self.create_subscription(
                action_type.Impl.FeedbackMessage, f"/{action}/_action/feedback",
                lambda msg, action=action: self.evidence.feedback(action, msg),
                QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT))

        string_topics = [
            "/localization/bridge_status",
            "/safety/status",
            "/ranger_base/status",
        ]
        if include_perception_status:
            string_topics.extend(
                [
                    "/lidar/axis_remap_status",
                    "/lidar/nav_cloud_preprocessor_status",
                    "/lidar/pointcloud_accel_status",
                ]
            )
        for topic in string_topics:
            self.create_subscription(
                String,
                topic,
                lambda msg, topic=topic: self.on_string(topic, msg),
                retained_state_qos if topic == "/safety/status" else small_qos,
            )

        for topic in (
            "/ranger_mini3/nav_terminal_reverse_enable",
            "/ranger_mini3/nav_terminal_lateral_enable",
            "/safety/motion_allowed",
            "/safety/estop",
        ):
            self.create_subscription(Bool, topic,
                                     lambda msg, topic=topic: self.on_scalar(topic, msg),
                                     retained_state_qos if topic == "/safety/motion_allowed" else small_qos)
        self.create_subscription(
            UInt8, "/ranger_mini3/nav_elevator_scoped_progress_state",
            lambda msg: self.on_scalar("/ranger_mini3/nav_elevator_scoped_progress_state", msg),
            small_qos)

        if include_cmd_vel:
            for topic in (
                "/cmd_vel_nav_raw",
                "/cmd_vel_nav",
                "/cmd_vel_collision_checked",
                "/cmd_vel_safe",
                "/cmd_vel",
                "/cmd_vel_api",
                "/cmd_vel_docking",
            ):
                self.twist_stats[topic] = {
                    "count": 0,
                    "nonzero_count": 0,
                    "first_nonzero_at": None,
                    "last_nonzero_at": None,
                    "last_msg_at": None,
                    "last": None,
                    "max_abs_linear_x": 0.0,
                    "max_abs_angular_z": 0.0,
                }
                self.cmd_shape_stats[topic] = {
                    "zero_count": 0,
                    "pure_yaw_count": 0,
                    "straight_count": 0,
                    "ackermann_feasible_count": 0,
                    "ackermann_too_tight_count": 0,
                    "min_turning_radius_m": None,
                    "last_shape": None,
                    "last_turning_radius_m": None,
                }
                self.create_subscription(
                    Twist,
                    topic,
                    lambda msg, topic=topic: self.on_twist(topic, msg),
                    small_qos,
                )

        if include_controller_detail:
            self.create_subscription(SpeedLimit, "/speed_limit", self.on_speed_limit, small_qos)
            for topic in (
                "/transformed_global_plan",
                "/received_global_plan",
                "/plan",
                "/plan_smoothed",
                "/ranger_mini3/ordinary_local_repair_path",
            ):
                self.path_stats[topic] = {
                    "count": 0,
                    "last_msg_at": None,
                    "latest": None,
                    "empty_count": 0,
                }
                self.create_subscription(
                    NavPath,
                    topic,
                    lambda msg, topic=topic: self.on_path(topic, msg),
                    small_qos,
                )
            self.create_subscription(
                OccupancyGrid,
                "/local_costmap/costmap",
                self.on_local_costmap,
                QoSProfile(depth=2),
            )
            if store_costmap_snapshots:
                self.create_subscription(
                    PolygonStamped, "/local_costmap/published_footprint", self.on_footprint,
                    QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
                self.create_subscription(
                    Odometry,
                    "/local_state/odometry",
                    self.on_odometry,
                    small_qos,
                )

        if include_rosout:
            self.create_subscription(Log, "/rosout", self.on_rosout, rosout_qos)

        self.create_timer(max(0.2, sample_period_sec), self.on_sample_timer)
        self.create_timer(1.0, self.on_api_timer)
        self.create_timer(10.0, self.refresh_evidence_graph)

    def refresh_evidence_graph(self):
        """One existing participant; sparse cached graph reads, never ROS RPC."""
        selected = {}
        errors = []
        try:
            graph = dict(self.get_topic_names_and_types())
            names = {"/navigate_to_pose/_action/feedback", "/follow_path/_action/feedback",
                     "/collision_monitor_state", "/cmd_vel_nav_raw", "/cmd_vel_nav",
                     "/cmd_vel_collision_checked", "/cmd_vel", "/local_state/odometry",
                     "/local_costmap/costmap", "/local_costmap/published_footprint",
                     "/ranger_mini3/ordinary_local_repair_path", "/transformed_global_plan"}
            candidates = sorted(name for name, types in graph.items()
                                if "nav2_msgs/msg/CollisionMonitorState" in types)
            names.update(candidates[:4])
            # Inventory only: NEVER subscribe to the full MPPI MarkerArray.
            names.update(sorted(name for name in graph if "critic" in name.lower() or name == "/trajectories")[:8])
            for name in sorted(names):
                selected[name] = {"types": graph.get(name, []),
                                  "publisher_count": self.count_publishers(name)}
            for name in candidates[:4]:
                if CollisionMonitorState is None:
                    errors.append("CollisionMonitorState Python type unavailable")
                    break
                if name not in self.collision_state_subscriptions and selected[name]["publisher_count"] > 0:
                    self.collision_state_subscriptions[name] = self.create_subscription(
                        CollisionMonitorState, name,
                        lambda msg, name=name: self.evidence.collision(name, msg),
                        QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        except Exception as exc:
            errors.append(f"{type(exc).__name__}: {exc}")
        self.evidence.graph(selected, errors)

    def close_files(self):
        self.api_worker.shutdown(wait=True)
        self.evidence.close()
        for handle in (
            self.samples_file,
            self.api_file,
            self.events_file,
            self.cmd_frames_file,
            self.path_frames_file,
            self.rosout_file,
            self.costmap_index_file,
        ):
            if handle is None:
                continue
            try:
                handle.flush()
                handle.close()
            except Exception:
                pass

    def emit_event(self, kind, payload):
        self.event_counts[kind] = self.event_counts.get(kind, 0) + 1
        row = {
            "captured_at": now_iso(),
            "elapsed_monotonic_sec": time.monotonic() - self.started_monotonic,
            "kind": kind,
            "payload": payload,
        }
        self.events_file.write(json.dumps(row, ensure_ascii=True, sort_keys=True) + "\n")
        self.events_file.flush()

    def on_scalar(self, topic, msg):
        previous = self.scalar_status.get(topic)
        self.scalar_status[topic] = {
            "captured_at": now_iso(), "received_monotonic": time.monotonic(), "data": msg.data,
        }
        if previous is None or previous["data"] != msg.data:
            self.emit_event("scalar_state_changed", {"topic": topic, "data": msg.data})

    def scalar_snapshot(self):
        now = time.monotonic()
        return {topic: {**value, "receive_age_sec": now - value["received_monotonic"]}
                for topic, value in self.scalar_status.items()}

    def on_footprint(self, msg):
        now = time.monotonic()
        if now - self.last_footprint_monotonic < 0.5:
            return
        self.last_footprint_monotonic = now
        self.latest_footprint = {
            "captured_at": now_iso(), "frame_id": msg.header.frame_id,
            "stamp_sec": msg.header.stamp.sec, "stamp_nanosec": msg.header.stamp.nanosec,
            "points_xyz": [[float(p.x), float(p.y), float(p.z)] for p in msg.polygon.points],
        }

    def on_action_status(self, topic, msg):
        statuses = []
        for item in msg.status_list:
            status_code = int(item.status)
            statuses.append(
                {
                    "goal_id": short_uuid(item.goal_info.goal_id),
                    "status": status_code,
                    "status_name": STATUS_NAMES.get(status_code, str(status_code)),
                }
            )
        key = tuple((entry["goal_id"], entry["status"]) for entry in statuses)
        self.action_status[topic] = statuses
        if self.action_status_keys.get(topic) != key:
            self.action_status_keys[topic] = key
            self.emit_event("action_status_changed", {"topic": topic, "statuses": statuses})

    def on_string(self, topic, msg):
        data = msg.data
        parsed = parse_json_maybe(data)
        now = now_iso()
        self.string_status[topic] = {
            "captured_at": now,
            "data": data,
            "json": parsed,
        }
        last_data = self.string_status_last.get(topic)
        if data != last_data:
            self.string_status_last[topic] = data
            payload = {"topic": topic, "data": data}
            if topic == "/localization/bridge_status" and isinstance(parsed, dict):
                payload["bridge_summary"] = {key: parsed.get(key) for key in BRIDGE_KEYS}
            self.emit_event("string_status_changed", payload)

    def on_twist(self, topic, msg):
        stats = self.twist_stats[topic]
        stats["count"] += 1
        stats["last_msg_at"] = now_iso()
        stats["last"] = twist_dict(msg)
        stats["max_abs_linear_x"] = max(stats["max_abs_linear_x"], abs(msg.linear.x))
        stats["max_abs_angular_z"] = max(stats["max_abs_angular_z"], abs(msg.angular.z))
        shape = command_shape(msg)
        shape_stats = self.cmd_shape_stats.get(topic)
        if shape_stats is not None:
            shape_name = shape["shape"]
            key = f"{shape_name}_count"
            shape_stats[key] = int(shape_stats.get(key, 0)) + 1
            radius = shape.get("turning_radius_m")
            if radius is not None:
                previous = shape_stats.get("min_turning_radius_m")
                shape_stats["min_turning_radius_m"] = (
                    radius if previous is None else min(float(previous), radius)
                )
            shape_stats["last_shape"] = shape_name
            shape_stats["last_turning_radius_m"] = radius
        cmd_row = {
            "captured_at": now_iso(),
            "elapsed_monotonic_sec": time.monotonic() - self.started_monotonic,
            "topic": topic,
            "twist": twist_dict(msg),
            "shape": shape,
            "mode": mode_snapshot(
                self.string_status.get("/ranger_base/status") or {}
            ),
            "permits_and_progress": self.scalar_snapshot(),
            "collision_state": self.evidence.collision_snapshot(),
        }
        self.cmd_frames_file.write(json.dumps(cmd_row, ensure_ascii=True, sort_keys=True) + "\n")
        if stats["count"] % 20 == 0:
            self.cmd_frames_file.flush()
        if twist_nonzero(msg):
            stats["nonzero_count"] += 1
            if stats["first_nonzero_at"] is None:
                stats["first_nonzero_at"] = now_iso()
                self.emit_event("first_nonzero_twist", {"topic": topic, "twist": twist_dict(msg)})
            stats["last_nonzero_at"] = now_iso()

    def on_speed_limit(self, msg):
        value = float(msg.speed_limit)
        stats = self.speed_limit_stats
        stats["count"] += 1
        stats["last_msg_at"] = now_iso()
        stats["last"] = speed_limit_dict(msg)
        stats["min_speed_limit"] = (
            value if stats["min_speed_limit"] is None else min(stats["min_speed_limit"], value)
        )
        stats["max_speed_limit"] = (
            value if stats["max_speed_limit"] is None else max(stats["max_speed_limit"], value)
        )
        if value <= 0.01:
            stats["zero_or_near_zero_count"] += 1
            self.emit_event("speed_limit_zero_or_near_zero", stats["last"])

    def on_path(self, topic, msg):
        stats = self.path_stats[topic]
        count = len(msg.poses)
        stats["count"] += 1
        stats["last_msg_at"] = now_iso()
        if count == 0:
            stats["empty_count"] += 1
        latest = {
            "frame_id": msg.header.frame_id,
            "stamp_sec": int(msg.header.stamp.sec),
            "stamp_nanosec": int(msg.header.stamp.nanosec),
            "pose_count": count,
            "path_length_m": path_length_m(msg),
            "first_pose": pose_summary(msg.poses[0]) if count else None,
            "second_pose": pose_summary(msg.poses[1]) if count > 1 else None,
            "last_pose": pose_summary(msg.poses[-1]) if count else None,
        }
        stats["latest"] = latest
        if store_costmap_snapshots:
            self.latest_path_geometry[topic] = {
                "captured_at": now_iso(),
                "frame_id": msg.header.frame_id,
                "stamp_sec": int(msg.header.stamp.sec),
                "stamp_nanosec": int(msg.header.stamp.nanosec),
                "poses_xy": [
                    [float(item.pose.position.x), float(item.pose.position.y)]
                    for item in msg.poses
                ],
                "poses_xyyaw": [
                    [float(item.pose.position.x), float(item.pose.position.y),
                     quaternion_yaw(item.pose.orientation)] for item in msg.poses
                ],
            }
            geometry = self.latest_path_geometry[topic]
            fingerprint = hashlib.sha256(json.dumps(
                [geometry["frame_id"], geometry["poses_xyyaw"]], separators=(",", ":")
            ).encode()).hexdigest()
            now = time.monotonic()
            changed = self.path_fingerprints.get(topic) != fingerprint
            # Reference/repaired paths are sparse; transformed MPPI paths can be 15 Hz.
            if changed and (topic in ("/plan", "/ranger_mini3/ordinary_local_repair_path")
                            or now - self.path_last_saved.get(topic, -math.inf) >= 0.5):
                self.path_frames_file.write(json.dumps({
                    "topic": topic, "sha256": fingerprint, **geometry,
                    "elapsed_monotonic_sec": now - self.started_monotonic,
                }) + "\n")
                self.path_frames_file.flush()
                self.path_fingerprints[topic] = fingerprint
                self.path_last_saved[topic] = now
        if count == 0:
            self.emit_event("path_empty", {"topic": topic})

    def on_odometry(self, msg):
        pose = msg.pose.pose
        twist = msg.twist.twist
        self.latest_odometry = {
            "captured_at": now_iso(),
            "stamp_sec": int(msg.header.stamp.sec),
            "stamp_nanosec": int(msg.header.stamp.nanosec),
            "frame_id": msg.header.frame_id,
            "child_frame_id": msg.child_frame_id,
            "x": float(pose.position.x),
            "y": float(pose.position.y),
            "yaw_rad": quaternion_yaw(pose.orientation),
            "linear_x": float(twist.linear.x),
            "linear_y": float(twist.linear.y),
            "angular_z": float(twist.angular.z),
        }
        self.evidence.remember_odom(self.latest_odometry)

    @staticmethod
    def costmap_preview_pixel(value):
        value = int(value)
        if value < 0:
            return 205
        if value >= 99:
            return 0
        return max(1, min(254, 254 - int(round(254.0 * value / 100.0))))

    def store_costmap_snapshot(self, msg, latest_stats):
        snapshot_index = self.costmap_snapshot_count
        self.costmap_snapshot_count += 1
        sec = int(msg.header.stamp.sec)
        nanosec = int(msg.header.stamp.nanosec)
        stem = f"costmap_{sec}_{nanosec:09d}_{snapshot_index:05d}"
        raw_name = f"{stem}.int8.bin.gz"
        pgm_name = f"{stem}.pgm"
        metadata_name = f"{stem}.json"
        raw_path = self.costmap_snapshot_dir / raw_name
        pgm_path = self.costmap_snapshot_dir / pgm_name
        metadata_path = self.costmap_snapshot_dir / metadata_name

        raw_bytes = bytes((int(value) & 0xFF) for value in msg.data)
        with gzip.open(raw_path, "wb", compresslevel=3) as handle:
            handle.write(raw_bytes)

        width = int(msg.info.width)
        height = int(msg.info.height)
        pixels = bytearray()
        for y in range(height - 1, -1, -1):
            row_start = y * width
            pixels.extend(
                self.costmap_preview_pixel(msg.data[row_start + x])
                for x in range(width)
            )
        with pgm_path.open("wb") as handle:
            handle.write(f"P5\n{width} {height}\n255\n".encode("ascii"))
            handle.write(pixels)

        metadata = {
            "captured_at": now_iso(),
            "captured_epoch_sec": time.time(),
            "snapshot_index": snapshot_index,
            "message": {
                "stamp_sec": sec,
                "stamp_nanosec": nanosec,
                "frame_id": msg.header.frame_id,
                "width": width,
                "height": height,
                "resolution": float(msg.info.resolution),
                "origin": {
                    "x": float(msg.info.origin.position.x),
                    "y": float(msg.info.origin.position.y),
                    "z": float(msg.info.origin.position.z),
                    "qx": float(msg.info.origin.orientation.x),
                    "qy": float(msg.info.origin.orientation.y),
                    "qz": float(msg.info.origin.orientation.z),
                    "qw": float(msg.info.origin.orientation.w),
                },
            },
            "raw_grid": {
                "file": raw_name,
                "encoding": "int8_twos_complement_row_major",
                "cell_count": len(msg.data),
            },
            "preview": {
                "file": pgm_name,
                "row_order": "top_row_is_positive_y",
                "unknown_gray": 205,
                "free_gray": 254,
                "lethal_gray": 0,
            },
            "statistics": latest_stats,
            "odometry": self.latest_odometry,
            "published_footprint": self.latest_footprint,
            "permits_and_progress": self.scalar_snapshot(),
            "paths": self.latest_path_geometry,
            "navigation_feedback": self.evidence.latest_feedback,
            "collision_state": self.evidence.collision_snapshot(),
            "action_status": self.action_status,
            "api_goal": (
                (self.latest_api_navigation or {}).get("navigation_goal")
                if isinstance(self.latest_api_navigation, dict)
                else None
            ),
            "latest_commands": {
                topic: {
                    "last_msg_at": stats.get("last_msg_at"),
                    "last": stats.get("last"),
                }
                for topic, stats in self.twist_stats.items()
            },
        }
        metadata_path.write_text(
            json.dumps(metadata, ensure_ascii=True, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        index_row = {
            "captured_at": metadata["captured_at"],
            "captured_epoch_sec": metadata["captured_epoch_sec"],
            "snapshot_index": snapshot_index,
            "stamp_sec": sec,
            "stamp_nanosec": nanosec,
            "frame_id": msg.header.frame_id,
            "raw_file": raw_name,
            "preview_file": pgm_name,
            "metadata_file": metadata_name,
        }
        self.costmap_index_file.write(
            json.dumps(index_row, ensure_ascii=True, sort_keys=True) + "\n"
        )
        self.costmap_index_file.flush()

    def summarize_costmap_window(self, data, width, height, cx, cy, radius_cells):
        counts = {
            "cells": 0,
            "unknown": 0,
            "free": 0,
            "low_cost": 0,
            "medium_cost": 0,
            "high_cost": 0,
            "lethal": 0,
            "max_cost": None,
        }
        x0 = max(0, cx - radius_cells)
        x1 = min(width - 1, cx + radius_cells)
        y0 = max(0, cy - radius_cells)
        y1 = min(height - 1, cy + radius_cells)
        for y in range(y0, y1 + 1):
            base = y * width
            for x in range(x0, x1 + 1):
                value = int(data[base + x])
                counts["cells"] += 1
                if value < 0:
                    counts["unknown"] += 1
                    continue
                counts["max_cost"] = value if counts["max_cost"] is None else max(counts["max_cost"], value)
                if value == 0:
                    counts["free"] += 1
                elif value >= 99:
                    counts["lethal"] += 1
                elif value >= 80:
                    counts["high_cost"] += 1
                elif value >= 50:
                    counts["medium_cost"] += 1
                else:
                    counts["low_cost"] += 1
        return counts

    def on_local_costmap(self, msg):
        width = int(msg.info.width)
        height = int(msg.info.height)
        resolution = float(msg.info.resolution)
        data = msg.data
        total = len(data)
        overall = {
            "cells": total,
            "unknown": 0,
            "free": 0,
            "low_cost": 0,
            "medium_cost": 0,
            "high_cost": 0,
            "lethal": 0,
            "max_cost": None,
        }
        for raw in data:
            value = int(raw)
            if value < 0:
                overall["unknown"] += 1
                continue
            overall["max_cost"] = value if overall["max_cost"] is None else max(overall["max_cost"], value)
            if value == 0:
                overall["free"] += 1
            elif value >= 99:
                overall["lethal"] += 1
            elif value >= 80:
                overall["high_cost"] += 1
            elif value >= 50:
                overall["medium_cost"] += 1
            else:
                overall["low_cost"] += 1
        cx = width // 2
        cy = height // 2
        near_0_5 = self.summarize_costmap_window(
            data, width, height, cx, cy, max(1, int(round(0.5 / max(resolution, 1.0e-6))))
        )
        near_1_0 = self.summarize_costmap_window(
            data, width, height, cx, cy, max(1, int(round(1.0 / max(resolution, 1.0e-6))))
        )
        latest = {
            "stamp_sec": int(msg.header.stamp.sec),
            "stamp_nanosec": int(msg.header.stamp.nanosec),
            "frame_id": msg.header.frame_id,
            "width": width,
            "height": height,
            "resolution": resolution,
            "origin_x": float(msg.info.origin.position.x),
            "origin_y": float(msg.info.origin.position.y),
            "overall": overall,
            "center_0_5m": near_0_5,
            "center_1_0m": near_1_0,
        }
        stats = self.local_costmap_stats
        stats["count"] += 1
        stats["last_msg_at"] = now_iso()
        stats["latest"] = latest
        if near_0_5["medium_cost"] + near_0_5["high_cost"] + near_0_5["lethal"] > 0:
            stats["near_robot_occupied_count"] += 1
        if near_0_5["lethal"] > 0:
            stats["near_robot_lethal_count"] += 1
            self.emit_event("local_costmap_center_lethal", latest)
        if store_costmap_snapshots:
            now_monotonic = time.monotonic()
            if (
                self.last_costmap_snapshot_monotonic is None
                or now_monotonic - self.last_costmap_snapshot_monotonic
                >= costmap_snapshot_period_sec
            ):
                self.last_costmap_snapshot_monotonic = now_monotonic
                try:
                    self.store_costmap_snapshot(msg, latest)
                except Exception as exc:
                    self.emit_event(
                        "costmap_snapshot_write_failed",
                        {"error": f"{type(exc).__name__}: {exc}"},
                    )

    def on_rosout(self, msg):
        text = msg.msg or ""
        name = msg.name or ""
        if not ROSOUT_FILTER.search(name) and not ROSOUT_FILTER.search(text):
            return
        self.evidence.log(msg, {
            "commands": {topic: {"last_msg_at": stats.get("last_msg_at"), "last": stats.get("last")}
                         for topic, stats in self.twist_stats.items()},
            "permits_and_progress": self.scalar_snapshot(),
            "odometry": self.latest_odometry,
            "collision_state": self.evidence.collision_snapshot(),
            "navigation_feedback": self.evidence.latest_feedback,
        })
        level = ROSOUT_LEVELS.get(int(msg.level), str(int(msg.level)))
        line = f"{now_iso()} [{level}] {name}: {text}"
        self.rosout_file.write(line + "\n")
        self.rosout_file.flush()
        self.rosout_hits += 1
        self.rosout_tail.append(line)
        self.rosout_tail = self.rosout_tail[-80:]

    def fetch_json(self, path):
        url = f"{api_url}{path}"
        try:
            request = urllib.request.Request(url)
            if api_token:
                request.add_header("X-Robot-Token", api_token)
            with urllib.request.urlopen(request, timeout=0.45) as response:
                body = response.read(1024 * 1024).decode("utf-8", errors="replace")
            return json.loads(body), None
        except (urllib.error.URLError, TimeoutError, json.JSONDecodeError, OSError) as exc:
            return None, f"{type(exc).__name__}: {exc}"

    def on_api_timer(self):
        # One bounded HTTP batch in flight; HTTP timeouts cannot block ROS callbacks.
        if self.api_future is None:
            self.api_future = self.api_worker.submit(self.fetch_api_batch)
            return
        if not self.api_future.done():
            return
        captured_at, (status, status_error), (navigation, navigation_error) = self.api_future.result()
        self.api_future = self.api_worker.submit(self.fetch_api_batch)
        if status is not None:
            self.latest_api_status = status
        if navigation is not None:
            self.latest_api_navigation = navigation
        errors = {}
        if status_error:
            errors["status"] = status_error
        if navigation_error:
            errors["navigation"] = navigation_error
        if errors:
            self.api_errors.append({"captured_at": now_iso(), "errors": errors})
            self.api_errors = self.api_errors[-20:]
        row = {
            "captured_at": captured_at,
            "status": status,
            "navigation_state": navigation,
            "errors": errors,
        }
        self.api_file.write(json.dumps(row, ensure_ascii=True, sort_keys=True) + "\n")
        self.api_file.flush()

    def fetch_api_batch(self):
        status = self.fetch_json("/api/v1/status")
        navigation = self.fetch_json("/api/v1/navigation/state")
        return now_iso(), status, navigation

    def on_sample_timer(self):
        self.evidence.process_sample()
        bridge = self.string_status.get("/localization/bridge_status", {})
        bridge_json = bridge.get("json") if isinstance(bridge, dict) else None
        sample = {
            "captured_at": now_iso(),
            "elapsed_sec": round(time.monotonic() - self.started_monotonic, 3),
            "permits_and_progress": self.scalar_snapshot(),
            "odometry": self.latest_odometry,
            "navigation_feedback": self.evidence.latest_feedback,
            "collision_state": self.evidence.collision_snapshot(),
            "published_footprint": self.latest_footprint,
            "action_status": self.action_status,
            "bridge_summary": {
                key: bridge_json.get(key)
                for key in BRIDGE_KEYS
            } if isinstance(bridge_json, dict) else None,
            "safety_status": (self.string_status.get("/safety/status") or {}).get("data"),
            "mode_controller_status": (
                self.string_status.get("/ranger_base/status") or {}
            ).get("data"),
            "twist_stats": self.twist_stats,
            "cmd_shape_stats": self.cmd_shape_stats,
            "speed_limit_stats": self.speed_limit_stats,
            "path_stats": self.path_stats,
            "local_costmap_stats": self.local_costmap_stats,
            "api_goal": (
                (self.latest_api_navigation or {}).get("navigation_goal")
                if isinstance(self.latest_api_navigation, dict)
                else None
            ),
            "rosout_hits": self.rosout_hits,
        }
        self.samples_file.write(json.dumps(sample, ensure_ascii=True, sort_keys=True) + "\n")
        self.samples_file.flush()

    def write_summary(self):
        bridge = self.string_status.get("/localization/bridge_status", {})
        bridge_json = bridge.get("json") if isinstance(bridge, dict) else None
        bridge_summary = (
            {key: bridge_json.get(key) for key in BRIDGE_KEYS}
            if isinstance(bridge_json, dict)
            else None
        )
        goal = {}
        if isinstance(self.latest_api_navigation, dict):
            goal = self.latest_api_navigation.get("navigation_goal") or {}

        classification = []
        follow_status = self.action_status.get("/follow_path/_action/status", [])
        compute_status = self.action_status.get("/compute_path_to_pose/_action/status", [])
        nav_status = self.action_status.get("/navigate_to_pose/_action/status", [])
        if any(item.get("status") == 6 for item in follow_status):
            classification.append("follow_path_aborted_controller_layer")
        if any(item.get("status") == 6 for item in compute_status):
            classification.append("compute_path_aborted_planner_layer")
        if any(item.get("status") == 6 for item in nav_status):
            classification.append("navigate_to_pose_aborted")
        if isinstance(goal, dict) and goal.get("nav2_result_code") == 6:
            classification.append("api_recorded_nav2_result_code_6")
        if isinstance(bridge_summary, dict):
            if bridge_summary.get("correction_active"):
                classification.append("bridge_correction_active_during_capture")
            if bridge_summary.get("localization_degraded"):
                classification.append("localization_degraded_during_capture")

        raw = self.twist_stats.get("/cmd_vel_nav_raw", {})
        checked = self.twist_stats.get("/cmd_vel_collision_checked", {})
        safe = self.twist_stats.get("/cmd_vel_safe", {})
        raw_shape = self.cmd_shape_stats.get("/cmd_vel_nav_raw", {})
        if raw.get("nonzero_count", 0) > 0 and checked.get("nonzero_count", 0) == 0:
            classification.append("controller_command_seen_but_collision_checked_zero")
        if checked.get("nonzero_count", 0) > 0 and safe.get("nonzero_count", 0) == 0:
            classification.append("collision_checked_command_seen_but_robot_safety_zero")
        if raw.get("count", 0) == 0 and any(item.get("status") == 6 for item in follow_status):
            classification.append("follow_path_aborted_without_observed_controller_cmd")
        if (
            any(item.get("status") == 6 for item in follow_status)
            and raw.get("count", 0) > 0
            and float(raw.get("max_abs_linear_x") or 0.0) < 0.05
        ):
            classification.append("controller_output_below_progress_velocity")
        if int(raw_shape.get("ackermann_too_tight_count") or 0) > 0:
            classification.append("controller_output_ackermann_infeasible_curvature")
        if int(raw_shape.get("pure_yaw_count") or 0) > 0:
            classification.append("controller_output_pure_yaw_seen")
        if int(self.speed_limit_stats.get("zero_or_near_zero_count") or 0) > 0:
            classification.append("speed_limit_zero_or_near_zero_seen")
        if int(self.local_costmap_stats.get("near_robot_lethal_count") or 0) > 0:
            classification.append("local_costmap_center_lethal_seen")
        if int(self.local_costmap_stats.get("near_robot_occupied_count") or 0) > 0:
            classification.append("local_costmap_center_occupied_seen")

        summary = {
            "recorder_version": 3,
            "evidence_availability": self.evidence.coverage(),
            "stop_reason": self.stop_reason,
            "elapsed_sec": time.monotonic() - self.started_monotonic,
            "permits_and_progress": self.scalar_snapshot(),
            "report_dir": str(output_dir),
            "duration_sec": duration_sec,
            "sample_period_sec": sample_period_sec,
            "impact_contract": {
                "ros_participants_created": 1,
                "publishes_topics": False,
                "sends_actions": False,
                "calls_services": False,
                "sets_params": False,
                "subscribes_tf": False,
                "subscribes_pointcloud": False,
                "subscribes_laserscan": False,
                "subscribes_local_costmap_summary_only": (
                    include_controller_detail and not store_costmap_snapshots
                ),
                "stores_full_costmap": store_costmap_snapshots,
                "costmap_snapshot_period_sec": (
                    costmap_snapshot_period_sec if store_costmap_snapshots else None
                ),
            },
            "api_auth_token_present": bool(api_token),
            "costmap_snapshot_count": self.costmap_snapshot_count,
            "classification": classification,
            "latest_api_goal": goal,
            "latest_api_status": self.latest_api_status,
            "action_status": self.action_status,
            "bridge_summary": bridge_summary,
            "safety_status": (self.string_status.get("/safety/status") or {}).get("data"),
            "mode_controller_status": (
                self.string_status.get("/ranger_base/status") or {}
            ).get("data"),
            "twist_stats": self.twist_stats,
            "cmd_shape_stats": self.cmd_shape_stats,
            "speed_limit_stats": self.speed_limit_stats,
            "path_stats": self.path_stats,
            "local_costmap_stats": self.local_costmap_stats,
            "event_counts": self.event_counts,
            "rosout_hits": self.rosout_hits,
            "rosout_tail": self.rosout_tail,
            "api_errors_tail": self.api_errors,
        }
        self.evidence.save_coverage()
        (output_dir / "summary.json").write_text(
            json.dumps(summary, indent=2, ensure_ascii=True, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        (output_dir / "summary.md").write_text(self.render_markdown(summary), encoding="utf-8")

    def render_markdown(self, summary):
        goal = summary.get("latest_api_goal") or {}
        bridge = summary.get("bridge_summary") or {}
        lines = [
            "# Minimal Navigation Failure Observation",
            "",
            f"- report_dir: `{summary['report_dir']}`",
            f"- duration_sec: `{summary['duration_sec']}`",
            f"- stop_reason: `{summary['stop_reason']}`",
            "- Classification is a capture-wide hint, not proof of this goal's root cause; inspect timestamps and goal IDs.",
            "- Missing permit/path messages mean unobserved, NOT false/disabled/no replanning.",
            "- center_0_5m/center_1_0m are map-axis square windows, NOT the robot outline or collision verdict.",
            "- Legacy lethal counters combine OccupancyGrid values 99 and 100; inspect exact grids to distinguish inscribed inflation from lethal obstacles.",
            "- impact: one temporary rclpy participant; no publish/action/service/param; no `/tf`, PointCloud2, or LaserScan subscriptions",
            f"- full_costmap_snapshots: `{summary['impact_contract']['stores_full_costmap']}` "
            f"count=`{summary.get('costmap_snapshot_count')}` "
            f"period_sec=`{summary['impact_contract'].get('costmap_snapshot_period_sec')}`",
            f"- classification: `{', '.join(summary['classification']) if summary['classification'] else 'no_failure_classification_yet'}`",
            "",
            "## Evidence Availability",
            f"- collision_state: `{summary['evidence_availability']['collision_state']['status']}`",
            f"- action_feedback_counts: `{summary['evidence_availability']['action_feedback_counts']}`",
            "- MPPI candidate rejection reasons and optimizer computation duration: **NOT COLLECTED**.",
            "- This capture cannot fully replay the optimizer. Missing diagnostics are NOT proof of clear obstacles or a healthy controller.",
            "- Map-pose/odometry pairing uses nearest original stamps (<=100 ms), not exact TF; retain the recorded mismatch and frames.",
            "- Process CPU/main-thread runqueue counters are scheduling clues, not MPPI cycle timing.",
            "",
            "## Final API Goal",
        ]
        if goal:
            for key in (
                "id",
                "state",
                "phase",
                "pose_id",
                "detail",
                "nav2_result_code",
                "nav2_succeeded",
                "position_reached",
                "final_pose_verified",
                "final_distance_m",
                "final_yaw_error_rad",
            ):
                if key in goal:
                    lines.append(f"- {key}: `{goal.get(key)}`")
        else:
            lines.append("- no navigation_goal observed from API")

        lines.extend(["", "## Nav2 Action Status"])
        for topic, statuses in summary.get("action_status", {}).items():
            status_text = ", ".join(
                f"{item.get('goal_id')}:{item.get('status_name')}"
                for item in statuses
            ) or "none"
            lines.append(f"- {topic}: `{status_text}`")

        lines.extend(["", "## Command Chain"])
        for topic, stats in summary.get("twist_stats", {}).items():
            lines.append(
                f"- {topic}: count=`{stats.get('count')}` nonzero=`{stats.get('nonzero_count')}` "
                f"first_nonzero=`{stats.get('first_nonzero_at')}` "
                f"max_vx=`{stats.get('max_abs_linear_x')}` max_wz=`{stats.get('max_abs_angular_z')}`"
            )

        lines.extend(["", "## Command Shape"])
        for topic, stats in summary.get("cmd_shape_stats", {}).items():
            lines.append(
                f"- {topic}: zero=`{stats.get('zero_count')}` pure_yaw=`{stats.get('pure_yaw_count')}` "
                f"ackermann_too_tight=`{stats.get('ackermann_too_tight_count')}` "
                f"ackermann_feasible=`{stats.get('ackermann_feasible_count')}` "
                f"min_radius=`{stats.get('min_turning_radius_m')}` last_shape=`{stats.get('last_shape')}`"
            )

        lines.extend(["", "## Controller Detail"])
        speed = summary.get("speed_limit_stats") or {}
        lines.append(
            f"- /speed_limit: count=`{speed.get('count')}` zero_or_near_zero=`{speed.get('zero_or_near_zero_count')}` "
            f"min=`{speed.get('min_speed_limit')}` max=`{speed.get('max_speed_limit')}` last=`{speed.get('last')}`"
        )
        for topic, stats in (summary.get("path_stats") or {}).items():
            latest = stats.get("latest") or {}
            lines.append(
                f"- {topic}: count=`{stats.get('count')}` empty=`{stats.get('empty_count')}` "
                f"poses=`{latest.get('pose_count')}` length_m=`{latest.get('path_length_m')}` "
                f"frame=`{latest.get('frame_id')}`"
            )
        costmap = summary.get("local_costmap_stats") or {}
        latest_costmap = costmap.get("latest") or {}
        center_0_5 = latest_costmap.get("center_0_5m") or {}
        center_1_0 = latest_costmap.get("center_1_0m") or {}
        lines.append(
            f"- /local_costmap/costmap: count=`{costmap.get('count')}` "
            f"frame=`{latest_costmap.get('frame_id')}` size=`{latest_costmap.get('width')}x{latest_costmap.get('height')}` "
            f"resolution=`{latest_costmap.get('resolution')}`"
        )
        lines.append(
            f"- local_costmap.center_0_5m: lethal=`{center_0_5.get('lethal')}` "
            f"high=`{center_0_5.get('high_cost')}` medium=`{center_0_5.get('medium_cost')}` "
            f"unknown=`{center_0_5.get('unknown')}` max=`{center_0_5.get('max_cost')}`"
        )
        lines.append(
            f"- local_costmap.center_1_0m: lethal=`{center_1_0.get('lethal')}` "
            f"high=`{center_1_0.get('high_cost')}` medium=`{center_1_0.get('medium_cost')}` "
            f"unknown=`{center_1_0.get('unknown')}` max=`{center_1_0.get('max_cost')}`"
        )

        lines.extend(["", "## Bridge And Safety"])
        if bridge:
            for key in BRIDGE_KEYS:
                if key in bridge:
                    lines.append(f"- bridge.{key}: `{bridge.get(key)}`")
        else:
            lines.append("- bridge: unavailable")
        lines.append(f"- safety_status: `{summary.get('safety_status')}`")
        lines.append(f"- mode_controller_status: `{summary.get('mode_controller_status')}`")

        lines.extend(["", "## Rosout Tail"])
        tail = summary.get("rosout_tail") or []
        if tail:
            lines.append("```text")
            lines.extend(tail[-40:])
            lines.append("```")
        else:
            lines.append("- no filtered rosout lines captured")

        lines.extend(
            [
                "",
                "## Files",
                "- `summary.json`: machine-readable summary",
                "- `runtime_snapshot.json` and `*_params_*.yaml`: process counts and startup parameter file evidence (not a live parameter RPC)",
                "- `evidence_availability.json`: explicit observed/missing evidence and installed package versions",
                "- `diagnostic_graph.jsonl`: sparse publisher inventory (no service requests)",
                "- `diagnostic_events.jsonl`: collision state changes when published; original-stamped controller warnings and command context",
                "- `action_feedback.jsonl`: goal UUID, stamped map pose, nearest buffered odometry, FollowPath distance/speed (max 5 Hz per action)",
                "- `process_samples.jsonl`: at most 1 Hz controller process CPU/main-thread scheduling counters; NOT MPPI computation duration",
                "- `samples.jsonl`: one aggregate sample per period",
                "- `cmd_frames.jsonl`: per-message Twist shape, turning radius, and motion-mode snapshot",
                "- `path_frames.jsonl.gz`: changed full path geometry with frames, stamps and yaw (snapshot mode)",
                "- `api_poll.jsonl`: `/api/v1/status` and `/api/v1/navigation/state` poll",
                "- `events.jsonl`: action/status changes and first nonzero command events",
                "- `rosout_filtered.log`: filtered Nav2/controller/localization/safety log lines",
                "- `costmap_snapshots/index.jsonl`: sampled full-grid timeline when `--store-costmap-snapshots` is enabled",
                "- `costmap_snapshots/*.int8.bin.gz`: exact signed int8 row-major OccupancyGrid payloads",
                "- `costmap_snapshots/*.pgm` and `*.json`: visual previews and frame/origin/odom/path metadata",
            ]
        )
        return "\n".join(lines) + "\n"


# Own signals: do not let SIGINT destroy the ROS context while take_message is
# converting a callback. This caused the historical Ctrl+C conversion traceback.
stop_requested = False


def request_stop(signum, _frame):
    global stop_requested
    stop_requested = True


signal.signal(signal.SIGINT, request_stop)
signal.signal(signal.SIGTERM, request_stop)
save_runtime_snapshot()
rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
node = MinimalNavigationObserver()
observer_error = None
print("[nav-failure-minimal] warming up DDS for 6 seconds; no robot action", flush=True)
ready_printed = False
try:
    while rclpy.ok() and not stop_requested and time.monotonic() < node.deadline_monotonic:
        rclpy.spin_once(node, timeout_sec=0.1)
        if not ready_printed and time.monotonic() - node.started_monotonic >= 6.0:
            ready_printed = True
            node.refresh_evidence_graph()
            availability = node.evidence.coverage()
            print("[nav-failure-minimal] EVIDENCE_LIMITS collision_state="
                  + availability["collision_state"]["status"]
                  + "; MPPI rejection reasons/compute duration NOT COLLECTED; see evidence_availability.json",
                  flush=True)
            print("[nav-failure-minimal] READY capture window open; start ONE App goal now", flush=True)
    if stop_requested:
        node.stop_reason = "operator_interrupt"
except KeyboardInterrupt:
    node.stop_reason = "operator_interrupt"
except Exception as exc:
    observer_error = exc
    node.stop_reason = f"observer_error: {type(exc).__name__}: {exc}"
    node.emit_event("observer_error", {"error": node.stop_reason})
finally:
    try:
        node.on_sample_timer()
        node.write_summary()
    finally:
        node.close_files()
        node.destroy_node()
        # SIGINT is handled by rclpy first and may already have shut down the
        # default context.  Keep Ctrl+C finalization idempotent so a completed
        # report is not followed by a misleading observer failure.
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception as exc:
                if "rcl_shutdown already called" not in str(exc):
                    raise
if observer_error is not None:
    print(f"[nav-failure-minimal] partial report saved; {node.stop_reason}", file=sys.stderr)
    sys.exit(1)
PY

status=$?
if [[ "${status}" -ne 0 ]]; then
  echo "${PREFIX} FAIL observer exited with status=${status}" >&2
  exit "${status}"
fi

echo "${PREFIX} wrote ${OUTPUT_DIR}"
echo "${PREFIX} summary ${OUTPUT_DIR}/summary.md"
