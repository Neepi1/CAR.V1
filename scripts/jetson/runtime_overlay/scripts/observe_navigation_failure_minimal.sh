#!/usr/bin/env bash
set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
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

Options:
  --duration-sec N              Capture duration in seconds. Default: 120.
  --sample-period-sec N         Summary sample period. Default: 1.0.
  --label LABEL                 Report label. Default: nav_failure_minimal.
  --api-url URL                 robot_api_server URL. Default: http://127.0.0.1:8080.
  --output-dir DIR              Report directory. Default: reports/navigation_failure_minimal/<timestamp>_<label>_<duration>s.
  --no-rosout                   Do not subscribe to /rosout.
  --no-cmd-vel                  Do not subscribe to command-chain Twist topics.
  --include-perception-status   Also record lightweight status strings from perception/lidar status topics.
  --no-controller-detail        Do not subscribe to /speed_limit, Nav2 Path topics, or /local_costmap/costmap.
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

TIMESTAMP="$(date -u +%Y%m%dT%H%M%SZ)"
LABEL="$(sanitize_label "${LABEL}")"
if [[ -z "${OUTPUT_DIR}" ]]; then
  OUTPUT_DIR="${NJRH_PROJECT_ROOT}/reports/navigation_failure_minimal/${TIMESTAMP}_${LABEL}_${DURATION_SEC}s"
fi
mkdir -p "${OUTPUT_DIR}"

echo "${PREFIX} report_dir=${OUTPUT_DIR}"
echo "${PREFIX} duration_sec=${DURATION_SEC} sample_period_sec=${SAMPLE_PERIOD_SEC}"
echo "${PREFIX} controller_detail=${INCLUDE_CONTROLLER_DETAIL} ackermann_min_radius_m=${ACKERMANN_MIN_TURNING_RADIUS_M}"
echo "${PREFIX} read-only: no goals, no params, no services, no /tf, no PointCloud2, no LaserScan"
echo "${PREFIX} start the App navigation goal now if you have not already"

python3 - \
  "${DURATION_SEC}" \
  "${SAMPLE_PERIOD_SEC}" \
  "${API_URL}" \
  "${OUTPUT_DIR}" \
  "${INCLUDE_ROSOUT}" \
  "${INCLUDE_CMD_VEL}" \
  "${INCLUDE_PERCEPTION_STATUS}" \
  "${INCLUDE_CONTROLLER_DETAIL}" \
  "${ACKERMANN_MIN_TURNING_RADIUS_M}" <<'PY'
import json
import math
import re
import sys
import time
import urllib.error
import urllib.request
from pathlib import Path

import rclpy
from action_msgs.msg import GoalStatusArray
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import Log
from nav2_msgs.msg import SpeedLimit
from nav_msgs.msg import OccupancyGrid, Path as NavPath
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


duration_sec = float(sys.argv[1])
sample_period_sec = float(sys.argv[2])
api_url = sys.argv[3].rstrip("/")
output_dir = Path(sys.argv[4])
include_rosout = sys.argv[5].lower() == "true"
include_cmd_vel = sys.argv[6].lower() == "true"
include_perception_status = sys.argv[7].lower() == "true"
include_controller_detail = sys.argv[8].lower() == "true"
ackermann_min_turning_radius_m = float(sys.argv[9])

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
    return time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())


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
    wz = float(msg.angular.z)
    abs_vx = abs(vx)
    abs_wz = abs(wz)
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


class MinimalNavigationObserver(Node):
    def __init__(self):
        super().__init__("minimal_navigation_failure_observer")
        self.started_wall = time.time()
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
        self.rosout_file = self.rosout_path.open("a", encoding="utf-8")

        self.action_status = {}
        self.action_status_keys = {}
        self.string_status = {}
        self.string_status_last = {}
        self.twist_stats = {}
        self.latest_api_status = None
        self.latest_api_navigation = None
        self.api_errors = []
        self.rosout_hits = 0
        self.rosout_tail = []
        self.event_counts = {}
        self.speed_limit_stats = {
            "count": 0,
            "last_msg_at": None,
            "last": None,
            "min_speed_limit": None,
            "max_speed_limit": None,
            "zero_or_near_zero_count": 0,
        }
        self.path_stats = {}
        self.local_costmap_stats = {
            "count": 0,
            "last_msg_at": None,
            "latest": None,
            "near_robot_occupied_count": 0,
            "near_robot_lethal_count": 0,
        }
        self.cmd_shape_stats = {}

        small_qos = QoSProfile(depth=10)
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

        string_topics = [
            "/localization/bridge_status",
            "/safety/status",
            "/ranger_mini3_mode_controller/status",
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
                small_qos,
            )

        if include_cmd_vel:
            for topic in (
                "/cmd_vel_nav_raw",
                "/cmd_vel_nav",
                "/cmd_vel_collision_checked",
                "/cmd_vel_safe",
                "/cmd_vel",
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

        if include_rosout:
            self.create_subscription(Log, "/rosout", self.on_rosout, rosout_qos)

        self.create_timer(max(0.2, sample_period_sec), self.on_sample_timer)
        self.create_timer(1.0, self.on_api_timer)

    def close_files(self):
        for handle in (
            self.samples_file,
            self.api_file,
            self.events_file,
            self.cmd_frames_file,
            self.rosout_file,
        ):
            try:
                handle.flush()
                handle.close()
            except Exception:
                pass

    def emit_event(self, kind, payload):
        self.event_counts[kind] = self.event_counts.get(kind, 0) + 1
        row = {
            "captured_at": now_iso(),
            "kind": kind,
            "payload": payload,
        }
        self.events_file.write(json.dumps(row, ensure_ascii=True, sort_keys=True) + "\n")
        self.events_file.flush()

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
            "topic": topic,
            "twist": twist_dict(msg),
            "shape": shape,
            "mode": mode_snapshot(
                self.string_status.get("/ranger_mini3_mode_controller/status") or {}
            ),
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
        if count == 0:
            self.emit_event("path_empty", {"topic": topic})

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

    def on_rosout(self, msg):
        text = msg.msg or ""
        name = msg.name or ""
        if not ROSOUT_FILTER.search(name) and not ROSOUT_FILTER.search(text):
            return
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
            with urllib.request.urlopen(url, timeout=0.45) as response:
                body = response.read(1024 * 1024).decode("utf-8", errors="replace")
            return json.loads(body), None
        except (urllib.error.URLError, TimeoutError, json.JSONDecodeError, OSError) as exc:
            return None, f"{type(exc).__name__}: {exc}"

    def on_api_timer(self):
        status, status_error = self.fetch_json("/api/v1/status")
        navigation, navigation_error = self.fetch_json("/api/v1/navigation/state")
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
            "captured_at": now_iso(),
            "status": status,
            "navigation_state": navigation,
            "errors": errors,
        }
        self.api_file.write(json.dumps(row, ensure_ascii=True, sort_keys=True) + "\n")
        self.api_file.flush()

    def on_sample_timer(self):
        bridge = self.string_status.get("/localization/bridge_status", {})
        bridge_json = bridge.get("json") if isinstance(bridge, dict) else None
        sample = {
            "captured_at": now_iso(),
            "elapsed_sec": round(time.time() - self.started_wall, 3),
            "action_status": self.action_status,
            "bridge_summary": {
                key: bridge_json.get(key)
                for key in BRIDGE_KEYS
            } if isinstance(bridge_json, dict) else None,
            "safety_status": (self.string_status.get("/safety/status") or {}).get("data"),
            "mode_controller_status": (
                self.string_status.get("/ranger_mini3_mode_controller/status") or {}
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
                "subscribes_local_costmap_summary_only": include_controller_detail,
                "stores_full_costmap": False,
            },
            "classification": classification,
            "latest_api_goal": goal,
            "latest_api_status": self.latest_api_status,
            "action_status": self.action_status,
            "bridge_summary": bridge_summary,
            "safety_status": (self.string_status.get("/safety/status") or {}).get("data"),
            "mode_controller_status": (
                self.string_status.get("/ranger_mini3_mode_controller/status") or {}
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
            "- impact: one temporary rclpy participant; no publish/action/service/param; no `/tf`, PointCloud2, or LaserScan subscriptions; local costmap is summarized only, not stored",
            f"- classification: `{', '.join(summary['classification']) if summary['classification'] else 'no_failure_classification_yet'}`",
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
                "- `samples.jsonl`: one aggregate sample per period",
                "- `cmd_frames.jsonl`: per-message Twist shape, turning radius, and motion-mode snapshot",
                "- `api_poll.jsonl`: `/api/v1/status` and `/api/v1/navigation/state` poll",
                "- `events.jsonl`: action/status changes and first nonzero command events",
                "- `rosout_filtered.log`: filtered Nav2/controller/localization/safety log lines",
            ]
        )
        return "\n".join(lines) + "\n"


rclpy.init()
node = MinimalNavigationObserver()
try:
    while rclpy.ok() and time.time() < node.deadline_wall:
        rclpy.spin_once(node, timeout_sec=0.1)
finally:
    try:
        node.on_sample_timer()
        node.write_summary()
    finally:
        node.close_files()
        node.destroy_node()
        rclpy.shutdown()
PY

status=$?
if [[ "${status}" -ne 0 ]]; then
  echo "${PREFIX} FAIL observer exited with status=${status}" >&2
  exit "${status}"
fi

echo "${PREFIX} wrote ${OUTPUT_DIR}"
echo "${PREFIX} summary ${OUTPUT_DIR}/summary.md"
