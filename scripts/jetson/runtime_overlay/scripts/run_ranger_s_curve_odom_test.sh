#!/usr/bin/env bash
set -euo pipefail
umask 0002

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OVERLAY_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
PROJECT_ROOT="$(cd "${OVERLAY_ROOT}/../../.." && pwd)"

DISTANCE_M="10.0"
LINEAR_SPEED_MPS="0.40"
HEADING_PEAK_DEG="20.0"
START_TURN="left"
REPEAT="1"
SAMPLE_HZ="20.0"
COUNTDOWN_SEC="3"
SETTLE_SEC="3.0"
LABEL="s_curve_10m"
CMD_TOPIC="/cmd_vel_collision_checked"
ODOM_TOPIC="/wheel/odom"
LOCAL_ODOM_TOPIC="/local_state/odometry"
OUTPUT_ROOT="${PROJECT_ROOT}/reports/ranger_s_curve_odom_test"
PAUSE_CORRECTION="true"
CORRECTION_PAUSE_SERVICE="/robot_localization_bridge/set_correction_paused"
DISTANCE_TOLERANCE_M="0.08"
LATERAL_TOLERANCE_M="0.20"
YAW_TOLERANCE_DEG="4.0"
MAX_EXTRA_SEC="15.0"

usage() {
  cat <<'EOF'
Usage: run_ranger_s_curve_odom_test.sh [options]

Runs an automated Ranger Mini 3 S-curve odometry test. The script publishes a
closed S-curve velocity profile through the safety chain and stops when
/wheel/odom reports the requested forward distance in the segment start frame.

The S-curve profile has zero expected yaw at the start and end, and near-zero
expected lateral offset at the end. This makes post-test relocalization
corrections easier to interpret.

Options:
  --distance-m M          Forward projection distance in the start frame. Default: 10.0
  --linear-speed MPS      Forward speed command. Default: 0.40
  --heading-peak-deg DEG  Peak expected heading in the S curve. Default: 20.0
  --start-turn left|right First half starts left or right. Default: left
  --repeat N              Repeat the segment N times. Default: 1
  --sample-hz HZ          Report sampling frequency. Default: 20.0
  --countdown-sec N       Countdown before motion. Default: 3
  --settle-sec SEC        Stop/record settle time after each segment. Default: 3.0
  --label NAME            Report label. Default: s_curve_10m
  --cmd-topic TOPIC       Safety-chain input topic. Default: /cmd_vel_collision_checked
  --odom-topic TOPIC      Distance feedback topic. Default: /wheel/odom
  --local-odom-topic TOPIC Local odom topic to record. Default: /local_state/odometry
  --output-root DIR       Report root. Default: reports/ranger_s_curve_odom_test
  --no-pause-correction   Do not call bridge correction pause service.
  --distance-tolerance-m M Final forward tolerance in summary. Default: 0.08
  --lateral-tolerance-m M  Final lateral tolerance in summary. Default: 0.20
  --yaw-tolerance-deg DEG Final yaw tolerance in summary. Default: 4.0
  --max-extra-sec SEC     Extra timeout beyond distance/speed. Default: 15.0

Use 10m as the default field test distance. A 0.10m S curve is too short to
measure Ranger odometry accuracy and should only be used as a bench smoke test.

The command path remains:
  test script -> /cmd_vel_collision_checked -> robot_safety -> /cmd_vel_safe
              -> ranger_mini3_mode_controller -> /cmd_vel -> ranger_base
EOF
}

while [[ "$#" -gt 0 ]]; do
  case "$1" in
    --distance-m)
      DISTANCE_M="${2:-}"
      shift 2
      ;;
    --linear-speed|--linear-speed-mps)
      LINEAR_SPEED_MPS="${2:-}"
      shift 2
      ;;
    --heading-peak-deg)
      HEADING_PEAK_DEG="${2:-}"
      shift 2
      ;;
    --start-turn)
      START_TURN="${2:-}"
      shift 2
      ;;
    --repeat)
      REPEAT="${2:-}"
      shift 2
      ;;
    --sample-hz)
      SAMPLE_HZ="${2:-}"
      shift 2
      ;;
    --countdown-sec)
      COUNTDOWN_SEC="${2:-}"
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
    --cmd-topic)
      CMD_TOPIC="${2:-}"
      shift 2
      ;;
    --odom-topic)
      ODOM_TOPIC="${2:-}"
      shift 2
      ;;
    --local-odom-topic)
      LOCAL_ODOM_TOPIC="${2:-}"
      shift 2
      ;;
    --output-root)
      OUTPUT_ROOT="${2:-}"
      shift 2
      ;;
    --no-pause-correction)
      PAUSE_CORRECTION="false"
      shift
      ;;
    --distance-tolerance-m)
      DISTANCE_TOLERANCE_M="${2:-}"
      shift 2
      ;;
    --lateral-tolerance-m)
      LATERAL_TOLERANCE_M="${2:-}"
      shift 2
      ;;
    --yaw-tolerance-deg)
      YAW_TOLERANCE_DEG="${2:-}"
      shift 2
      ;;
    --max-extra-sec)
      MAX_EXTRA_SEC="${2:-}"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "[ranger-s-curve-test] unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

case "${START_TURN}" in
  left|right) ;;
  *)
    echo "[ranger-s-curve-test] --start-turn must be left or right" >&2
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
OUT_DIR="${OUTPUT_ROOT}/${timestamp}_${safe_label}"
mkdir -p "${OUT_DIR}"

{
  echo "# Ranger S-Curve Odom Test Environment"
  echo "- timestamp_utc: ${timestamp}"
  echo "- distance_m: ${DISTANCE_M}"
  echo "- linear_speed_mps: ${LINEAR_SPEED_MPS}"
  echo "- heading_peak_deg: ${HEADING_PEAK_DEG}"
  echo "- start_turn: ${START_TURN}"
  echo "- repeat: ${REPEAT}"
  echo "- sample_hz: ${SAMPLE_HZ}"
  echo "- countdown_sec: ${COUNTDOWN_SEC}"
  echo "- settle_sec: ${SETTLE_SEC}"
  echo "- label: ${LABEL}"
  echo "- cmd_topic: ${CMD_TOPIC}"
  echo "- odom_topic: ${ODOM_TOPIC}"
  echo "- local_odom_topic: ${LOCAL_ODOM_TOPIC}"
  echo "- pause_correction: ${PAUSE_CORRECTION}"
  echo "- correction_pause_service: ${CORRECTION_PAUSE_SERVICE}"
  echo "- rmw: ${RMW_IMPLEMENTATION:-}"
  echo
  echo "## ROS Nodes"
  ros2 node list 2>&1 || true
  echo
  echo "## Topic Info"
  for topic in \
    "${CMD_TOPIC}" \
    /cmd_vel_safe \
    /cmd_vel \
    "${ODOM_TOPIC}" \
    "${LOCAL_ODOM_TOPIC}" \
    /motion_state \
    /system_state \
    /safety/status \
    /ranger_mini3_mode_controller/status \
    /localization/bridge_status; do
    echo "### ${topic}"
    ros2 topic info "${topic}" -v 2>&1 || true
  done
  echo
  echo "## Interfaces"
  ros2 interface show ranger_msgs/msg/MotionState 2>&1 || true
  echo "---"
  ros2 interface show ranger_msgs/msg/SystemState 2>&1 || true
  echo
  echo "## Services"
  ros2 service list -t 2>&1 | grep -E 'robot_localization_bridge|global_localization|trigger_grid' || true
} >"${OUT_DIR}/environment.md"

if [[ "${COUNTDOWN_SEC}" != "0" ]]; then
  echo "[ranger-s-curve-test] motion starts in ${COUNTDOWN_SEC}s. Ensure the S-curve envelope is clear and E-stop is available."
  remaining="${COUNTDOWN_SEC}"
  while [[ "${remaining}" -gt 0 ]]; do
    echo "[ranger-s-curve-test] ${remaining}..."
    sleep 1
    remaining=$((remaining - 1))
  done
fi

set +e
python3 - \
  "${OUT_DIR}" \
  "${DISTANCE_M}" \
  "${LINEAR_SPEED_MPS}" \
  "${HEADING_PEAK_DEG}" \
  "${START_TURN}" \
  "${REPEAT}" \
  "${SAMPLE_HZ}" \
  "${SETTLE_SEC}" \
  "${CMD_TOPIC}" \
  "${ODOM_TOPIC}" \
  "${LOCAL_ODOM_TOPIC}" \
  "${PAUSE_CORRECTION}" \
  "${CORRECTION_PAUSE_SERVICE}" \
  "${DISTANCE_TOLERANCE_M}" \
  "${LATERAL_TOLERANCE_M}" \
  "${YAW_TOLERANCE_DEG}" \
  "${MAX_EXTRA_SEC}" <<'PY'
import csv
import json
import math
import os
import sys
import time
from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Set, Tuple

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import SetBool

try:
    from ranger_msgs.msg import MotionState, SystemState
except Exception:
    MotionState = None
    SystemState = None


out_dir = sys.argv[1]
distance_m = float(sys.argv[2])
linear_speed = abs(float(sys.argv[3]))
heading_peak = math.radians(abs(float(sys.argv[4])))
start_turn = sys.argv[5]
repeat = int(sys.argv[6])
sample_hz = float(sys.argv[7])
settle_sec = float(sys.argv[8])
cmd_topic = sys.argv[9]
odom_topic = sys.argv[10]
local_odom_topic = sys.argv[11]
pause_correction = sys.argv[12].lower() == "true"
correction_pause_service = sys.argv[13]
distance_tolerance_m = abs(float(sys.argv[14]))
lateral_tolerance_m = abs(float(sys.argv[15]))
yaw_tolerance = math.radians(abs(float(sys.argv[16])))
max_extra_sec = float(sys.argv[17])

if distance_m <= 0.0:
    raise SystemExit("distance-m must be positive")
if linear_speed <= 0.0:
    raise SystemExit("linear speed must be positive")
if repeat < 1:
    raise SystemExit("repeat must be >= 1")
if sample_hz <= 0.0:
    raise SystemExit("sample_hz must be positive")
if start_turn not in ("left", "right"):
    raise SystemExit("start_turn must be left or right")

turn_sign = 1.0 if start_turn == "left" else -1.0
sample_period = 1.0 / sample_hz


def norm_angle(value: float) -> float:
    return math.atan2(math.sin(value), math.cos(value))


def yaw_from_quat(q: Any) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def odom_pose(msg: Odometry) -> Tuple[float, float, float]:
    pose = msg.pose.pose
    return (pose.position.x, pose.position.y, yaw_from_quat(pose.orientation))


def odom_twist(msg: Optional[Odometry]) -> Tuple[Optional[float], Optional[float], Optional[float]]:
    if msg is None:
        return (None, None, None)
    twist = msg.twist.twist
    return (twist.linear.x, twist.linear.y, twist.angular.z)


def twist_tuple(msg: Optional[Twist]) -> Tuple[Optional[float], Optional[float], Optional[float]]:
    if msg is None:
        return (None, None, None)
    return (msg.linear.x, msg.linear.y, msg.angular.z)


def relative_components(
    start: Tuple[float, float, float],
    current: Tuple[float, float, float],
) -> Tuple[float, float, float, float]:
    dx = current[0] - start[0]
    dy = current[1] - start[1]
    c = math.cos(start[2])
    s = math.sin(start[2])
    forward = dx * c + dy * s
    lateral = -dx * s + dy * c
    euclidean = math.hypot(dx, dy)
    yaw_delta = norm_angle(current[2] - start[2])
    return (forward, lateral, euclidean, yaw_delta)


def s_curve_profile(progress_m: float) -> Tuple[float, float, float]:
    u = min(1.0, max(0.0, progress_m / distance_m))
    sin_pi = math.sin(math.pi * u)
    sin_2pi = math.sin(2.0 * math.pi * u)
    cos_2pi = math.cos(2.0 * math.pi * u)
    expected_heading = turn_sign * heading_peak * sin_2pi * sin_pi * sin_pi
    dtheta_du = turn_sign * heading_peak * (
        2.0 * math.pi * cos_2pi * sin_pi * sin_pi
        + math.pi * sin_2pi * sin_2pi
    )
    # Progress is a start-frame forward projection, so use cos(expected_heading)
    # as the nominal dx/dt term. This keeps the profile smooth at both ends.
    expected_wz = linear_speed * max(0.0, math.cos(expected_heading)) * dtheta_du / distance_m
    return (u, expected_heading, expected_wz)


def motion_value(msg: Optional[Any], field: str) -> str:
    if msg is None:
        return ""
    return str(getattr(msg, field, ""))


def motion_float(msg: Optional[Any], field: str) -> Optional[float]:
    if msg is None:
        return None
    try:
        return float(getattr(msg, field))
    except Exception:
        return None


def fmt_optional(value: Optional[float], precision: int = 6) -> str:
    if value is None:
        return ""
    return f"{value:.{precision}f}"


def status_mode_code(mode_status: str) -> str:
    if not mode_status:
        return ""
    try:
        data = json.loads(mode_status)
        actual = data.get("actual_motion_mode") or {}
        return str(actual.get("code", ""))
    except Exception:
        return ""


def status_cmd_out(mode_status: str) -> Dict[str, Any]:
    if not mode_status:
        return {}
    try:
        data = json.loads(mode_status)
        return data.get("cmd_out") or {}
    except Exception:
        return {}


def pose_or_none(msg: Optional[Odometry]) -> Optional[Tuple[float, float, float]]:
    return odom_pose(msg) if msg is not None else None


@dataclass
class SegmentResult:
    index: int
    ok: bool
    reason: str
    duration_sec: float
    wheel_forward_m: float
    wheel_forward_error_m: float
    wheel_lateral_m: float
    wheel_distance_m: float
    wheel_yaw_delta_rad: float
    local_forward_m: Optional[float]
    local_forward_error_m: Optional[float]
    local_lateral_m: Optional[float]
    local_distance_m: Optional[float]
    local_yaw_delta_rad: Optional[float]
    max_abs_expected_wz: float
    max_abs_cmd_safe_vx: float
    max_abs_cmd_safe_wz: float
    max_abs_cmd_out_vx: float
    max_abs_cmd_out_wz: float
    max_abs_motion_linear: float
    max_abs_motion_angular: float
    max_abs_motion_steering: float
    motion_modes_seen: str
    system_modes_seen: str
    safety_status: str
    mode_status: str


class SCurveNode(Node):
    def __init__(self) -> None:
        super().__init__("ranger_s_curve_odom_test")
        qos = QoSProfile(depth=50)
        telemetry_qos = QoSProfile(depth=100)
        telemetry_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self.cmd_pub = self.create_publisher(Twist, cmd_topic, qos)
        self.pause_client = self.create_client(SetBool, correction_pause_service)
        self.wheel_odom: Optional[Odometry] = None
        self.local_odom: Optional[Odometry] = None
        self.cmd_safe: Optional[Twist] = None
        self.cmd_out: Optional[Twist] = None
        self.motion_state: Optional[Any] = None
        self.system_state: Optional[Any] = None
        self.safety_status = ""
        self.mode_status = ""
        self.bridge_status = ""
        self.create_subscription(Odometry, odom_topic, self._wheel_cb, telemetry_qos)
        self.create_subscription(Odometry, local_odom_topic, self._local_cb, telemetry_qos)
        self.create_subscription(Twist, "/cmd_vel_safe", self._cmd_safe_cb, telemetry_qos)
        self.create_subscription(Twist, "/cmd_vel", self._cmd_out_cb, telemetry_qos)
        self.create_subscription(String, "/safety/status", self._safety_cb, qos)
        self.create_subscription(String, "/ranger_mini3_mode_controller/status", self._mode_cb, qos)
        self.create_subscription(String, "/localization/bridge_status", self._bridge_cb, qos)
        if MotionState is not None:
            self.create_subscription(MotionState, "/motion_state", self._motion_cb, telemetry_qos)
        if SystemState is not None:
            self.create_subscription(SystemState, "/system_state", self._system_cb, telemetry_qos)

    def _wheel_cb(self, msg: Odometry) -> None:
        self.wheel_odom = msg

    def _local_cb(self, msg: Odometry) -> None:
        self.local_odom = msg

    def _cmd_safe_cb(self, msg: Twist) -> None:
        self.cmd_safe = msg

    def _cmd_out_cb(self, msg: Twist) -> None:
        self.cmd_out = msg

    def _motion_cb(self, msg: Any) -> None:
        self.motion_state = msg

    def _system_cb(self, msg: Any) -> None:
        self.system_state = msg

    def _safety_cb(self, msg: String) -> None:
        self.safety_status = msg.data

    def _mode_cb(self, msg: String) -> None:
        self.mode_status = msg.data

    def _bridge_cb(self, msg: String) -> None:
        self.bridge_status = msg.data

    def spin_some(self, duration: float) -> None:
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.02)

    def wait_for_odom(self, timeout_sec: float = 5.0) -> bool:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.wheel_odom is not None:
                return True
        return False

    def wait_for_local_odom(self, timeout_sec: float = 3.0) -> bool:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.local_odom is not None:
                return True
        return False

    def set_correction_pause(self, paused: bool) -> str:
        if not pause_correction:
            return "skipped"
        if not self.pause_client.wait_for_service(timeout_sec=2.0):
            return "service_unavailable"
        req = SetBool.Request()
        req.data = paused
        future = self.pause_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=4.0)
        if not future.done():
            return "timeout"
        result = future.result()
        return f"success={result.success} message={result.message}"

    def publish_cmd(self, vx: float, wz: float) -> None:
        msg = Twist()
        msg.linear.x = float(vx)
        msg.angular.z = float(wz)
        self.cmd_pub.publish(msg)

    def publish_zero_burst(self, duration: float = 1.0) -> None:
        end = time.monotonic() + duration
        while time.monotonic() < end and rclpy.ok():
            self.publish_cmd(0.0, 0.0)
            rclpy.spin_once(self, timeout_sec=0.02)
            time.sleep(0.03)


def write_sample(
    writer: csv.DictWriter,
    node: SCurveNode,
    segment_index: int,
    phase: str,
    elapsed: float,
    start_wheel: Tuple[float, float, float],
    start_local: Optional[Tuple[float, float, float]],
    cmd_vx: float,
    cmd_wz: float,
    expected_u: float,
    expected_heading: float,
    expected_wz: float,
) -> Tuple[float, float, float, float]:
    wheel_pose = odom_pose(node.wheel_odom) if node.wheel_odom is not None else start_wheel
    wheel_forward, wheel_lateral, wheel_distance, wheel_yaw_delta = relative_components(start_wheel, wheel_pose)
    local_pose = pose_or_none(node.local_odom)
    local_forward = local_lateral = local_distance = local_yaw_delta = None
    if start_local is not None and local_pose is not None:
        local_forward, local_lateral, local_distance, local_yaw_delta = relative_components(start_local, local_pose)
    cmd_safe = twist_tuple(node.cmd_safe)
    cmd_out = twist_tuple(node.cmd_out)
    wheel_twist = odom_twist(node.wheel_odom)
    local_twist = odom_twist(node.local_odom)
    mode_cmd = status_cmd_out(node.mode_status)
    writer.writerow({
        "segment": segment_index,
        "phase": phase,
        "elapsed_sec": f"{elapsed:.4f}",
        "target_forward_m": f"{distance_m:.6f}",
        "expected_u": f"{expected_u:.6f}",
        "expected_heading_rad": f"{expected_heading:.6f}",
        "expected_wz": f"{expected_wz:.6f}",
        "wheel_x": f"{wheel_pose[0]:.6f}",
        "wheel_y": f"{wheel_pose[1]:.6f}",
        "wheel_yaw": f"{wheel_pose[2]:.6f}",
        "wheel_forward_m": f"{wheel_forward:.6f}",
        "wheel_lateral_m": f"{wheel_lateral:.6f}",
        "wheel_distance_m": f"{wheel_distance:.6f}",
        "wheel_yaw_delta_rad": f"{wheel_yaw_delta:.6f}",
        "wheel_twist_vx": fmt_optional(wheel_twist[0]),
        "wheel_twist_wz": fmt_optional(wheel_twist[2]),
        "local_x": "" if local_pose is None else f"{local_pose[0]:.6f}",
        "local_y": "" if local_pose is None else f"{local_pose[1]:.6f}",
        "local_yaw": "" if local_pose is None else f"{local_pose[2]:.6f}",
        "local_forward_m": fmt_optional(local_forward),
        "local_lateral_m": fmt_optional(local_lateral),
        "local_distance_m": fmt_optional(local_distance),
        "local_yaw_delta_rad": fmt_optional(local_yaw_delta),
        "local_twist_vx": fmt_optional(local_twist[0]),
        "local_twist_wz": fmt_optional(local_twist[2]),
        "motion_linear_velocity": motion_value(node.motion_state, "linear_velocity"),
        "motion_lateral_velocity": motion_value(node.motion_state, "lateral_velocity"),
        "motion_angular_velocity": motion_value(node.motion_state, "angular_velocity"),
        "motion_steering_angle": motion_value(node.motion_state, "steering_angle"),
        "motion_mode": motion_value(node.motion_state, "motion_mode"),
        "system_motion_mode": motion_value(node.system_state, "motion_mode"),
        "cmd_requested_vx": f"{cmd_vx:.6f}",
        "cmd_requested_wz": f"{cmd_wz:.6f}",
        "cmd_safe_vx": fmt_optional(cmd_safe[0]),
        "cmd_safe_wz": fmt_optional(cmd_safe[2]),
        "cmd_out_vx": fmt_optional(cmd_out[0]),
        "cmd_out_wz": fmt_optional(cmd_out[2]),
        "mode_actual_code": status_mode_code(node.mode_status),
        "mode_cmd_out_vx": mode_cmd.get("linear_x", ""),
        "mode_cmd_out_wz": mode_cmd.get("angular_z", ""),
        "safety_status": node.safety_status,
    })
    return (wheel_forward, wheel_lateral, wheel_distance, wheel_yaw_delta)


def run_segment(node: SCurveNode, index: int, writer: csv.DictWriter) -> SegmentResult:
    timeout_sec = distance_m / linear_speed + max_extra_sec

    if node.wheel_odom is None:
        return SegmentResult(index, False, "missing_initial_wheel_odom", 0.0, 0.0, -distance_m, 0.0, 0.0, 0.0, None, None, None, None, None, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, "", "", node.safety_status, node.mode_status)

    start_wheel = odom_pose(node.wheel_odom)
    start_local = pose_or_none(node.local_odom)
    wheel_forward = 0.0
    wheel_lateral = 0.0
    wheel_distance = 0.0
    wheel_yaw_delta = 0.0
    local_forward: Optional[float] = None
    local_lateral: Optional[float] = None
    local_distance: Optional[float] = None
    local_yaw_delta: Optional[float] = None
    max_abs_expected_wz = 0.0
    max_abs_cmd_safe_vx = 0.0
    max_abs_cmd_safe_wz = 0.0
    max_abs_cmd_out_vx = 0.0
    max_abs_cmd_out_wz = 0.0
    max_abs_motion_linear = 0.0
    max_abs_motion_angular = 0.0
    max_abs_motion_steering = 0.0
    motion_modes: Set[str] = set()
    system_modes: Set[str] = set()
    observed_cmd_out = False
    reason = "target_reached"
    ok = True
    start_time = time.monotonic()
    next_sample = start_time

    while rclpy.ok():
        now = time.monotonic()
        elapsed = now - start_time
        if elapsed > timeout_sec:
            ok = False
            reason = "timeout"
            break

        if node.wheel_odom is not None:
            wheel_pose = odom_pose(node.wheel_odom)
            wheel_forward, wheel_lateral, wheel_distance, wheel_yaw_delta = relative_components(start_wheel, wheel_pose)

        progress = max(0.0, wheel_forward)
        expected_u, expected_heading, expected_wz = s_curve_profile(progress)
        cmd_vx = linear_speed
        cmd_wz = expected_wz
        node.publish_cmd(cmd_vx, cmd_wz)
        rclpy.spin_once(node, timeout_sec=0.01)

        local_pose = pose_or_none(node.local_odom)
        if local_pose is not None and start_local is not None:
            local_forward, local_lateral, local_distance, local_yaw_delta = relative_components(start_local, local_pose)

        if node.cmd_safe is not None:
            max_abs_cmd_safe_vx = max(max_abs_cmd_safe_vx, abs(node.cmd_safe.linear.x))
            max_abs_cmd_safe_wz = max(max_abs_cmd_safe_wz, abs(node.cmd_safe.angular.z))
        if node.cmd_out is not None:
            max_abs_cmd_out_vx = max(max_abs_cmd_out_vx, abs(node.cmd_out.linear.x))
            max_abs_cmd_out_wz = max(max_abs_cmd_out_wz, abs(node.cmd_out.angular.z))
            if abs(node.cmd_out.linear.x) >= linear_speed * 0.4:
                observed_cmd_out = True

        motion_linear = motion_float(node.motion_state, "linear_velocity")
        motion_angular = motion_float(node.motion_state, "angular_velocity")
        motion_steering = motion_float(node.motion_state, "steering_angle")
        if motion_linear is not None:
            max_abs_motion_linear = max(max_abs_motion_linear, abs(motion_linear))
        if motion_angular is not None:
            max_abs_motion_angular = max(max_abs_motion_angular, abs(motion_angular))
        if motion_steering is not None:
            max_abs_motion_steering = max(max_abs_motion_steering, abs(motion_steering))
        motion_mode = motion_value(node.motion_state, "motion_mode")
        system_mode = motion_value(node.system_state, "motion_mode")
        if motion_mode:
            motion_modes.add(motion_mode)
        if system_mode:
            system_modes.add(system_mode)
        max_abs_expected_wz = max(max_abs_expected_wz, abs(expected_wz))

        if now >= next_sample:
            write_sample(
                writer,
                node,
                index,
                "motion",
                elapsed,
                start_wheel,
                start_local,
                cmd_vx,
                cmd_wz,
                expected_u,
                expected_heading,
                expected_wz,
            )
            next_sample += sample_period

        if wheel_forward >= distance_m:
            break

        time.sleep(0.005)

    node.publish_zero_burst(1.0)
    settle_until = time.monotonic() + settle_sec
    while time.monotonic() < settle_until and rclpy.ok():
        elapsed = time.monotonic() - start_time
        node.publish_cmd(0.0, 0.0)
        rclpy.spin_once(node, timeout_sec=0.02)
        if time.monotonic() >= next_sample:
            progress = max(0.0, wheel_forward)
            expected_u, expected_heading, expected_wz = s_curve_profile(progress)
            write_sample(
                writer,
                node,
                index,
                "settle",
                elapsed,
                start_wheel,
                start_local,
                0.0,
                0.0,
                expected_u,
                expected_heading,
                0.0,
            )
            next_sample += sample_period
        time.sleep(0.03)

    end_wheel = odom_pose(node.wheel_odom) if node.wheel_odom is not None else start_wheel
    wheel_forward, wheel_lateral, wheel_distance, wheel_yaw_delta = relative_components(start_wheel, end_wheel)
    end_local = pose_or_none(node.local_odom)
    if start_local and end_local:
        local_forward, local_lateral, local_distance, local_yaw_delta = relative_components(start_local, end_local)
    duration = time.monotonic() - start_time

    wheel_forward_error = wheel_forward - distance_m
    local_forward_error = None if local_forward is None else local_forward - distance_m

    if ok and not observed_cmd_out:
        ok = False
        reason = "final_cmd_vel_not_observed"
    if ok and (
        abs(wheel_forward_error) > distance_tolerance_m
        or abs(wheel_lateral) > lateral_tolerance_m
        or abs(wheel_yaw_delta) > yaw_tolerance
    ):
        reason = "target_reached_final_error_gt_tolerance"

    return SegmentResult(
        index=index,
        ok=ok,
        reason=reason,
        duration_sec=duration,
        wheel_forward_m=wheel_forward,
        wheel_forward_error_m=wheel_forward_error,
        wheel_lateral_m=wheel_lateral,
        wheel_distance_m=wheel_distance,
        wheel_yaw_delta_rad=wheel_yaw_delta,
        local_forward_m=local_forward,
        local_forward_error_m=local_forward_error,
        local_lateral_m=local_lateral,
        local_distance_m=local_distance,
        local_yaw_delta_rad=local_yaw_delta,
        max_abs_expected_wz=max_abs_expected_wz,
        max_abs_cmd_safe_vx=max_abs_cmd_safe_vx,
        max_abs_cmd_safe_wz=max_abs_cmd_safe_wz,
        max_abs_cmd_out_vx=max_abs_cmd_out_vx,
        max_abs_cmd_out_wz=max_abs_cmd_out_wz,
        max_abs_motion_linear=max_abs_motion_linear,
        max_abs_motion_angular=max_abs_motion_angular,
        max_abs_motion_steering=max_abs_motion_steering,
        motion_modes_seen=",".join(sorted(motion_modes)),
        system_modes_seen=",".join(sorted(system_modes)),
        safety_status=node.safety_status,
        mode_status=node.mode_status,
    )


def main() -> int:
    rclpy.init(args=None)
    node = SCurveNode()
    results: List[SegmentResult] = []
    pause_enable_result = "not_called"
    pause_disable_result = "not_called"
    try:
        if not node.wait_for_odom():
            raise RuntimeError(f"no odometry received on {odom_topic}")
        node.wait_for_local_odom(timeout_sec=3.0)

        pause_enable_result = node.set_correction_pause(True)
        node.spin_some(0.5)

        samples_path = os.path.join(out_dir, "samples.csv")
        with open(samples_path, "w", newline="", encoding="utf-8") as f:
            fieldnames = [
                "segment",
                "phase",
                "elapsed_sec",
                "target_forward_m",
                "expected_u",
                "expected_heading_rad",
                "expected_wz",
                "wheel_x",
                "wheel_y",
                "wheel_yaw",
                "wheel_forward_m",
                "wheel_lateral_m",
                "wheel_distance_m",
                "wheel_yaw_delta_rad",
                "wheel_twist_vx",
                "wheel_twist_wz",
                "local_x",
                "local_y",
                "local_yaw",
                "local_forward_m",
                "local_lateral_m",
                "local_distance_m",
                "local_yaw_delta_rad",
                "local_twist_vx",
                "local_twist_wz",
                "motion_linear_velocity",
                "motion_lateral_velocity",
                "motion_angular_velocity",
                "motion_steering_angle",
                "motion_mode",
                "system_motion_mode",
                "cmd_requested_vx",
                "cmd_requested_wz",
                "cmd_safe_vx",
                "cmd_safe_wz",
                "cmd_out_vx",
                "cmd_out_wz",
                "mode_actual_code",
                "mode_cmd_out_vx",
                "mode_cmd_out_wz",
                "safety_status",
            ]
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            for i in range(1, repeat + 1):
                result = run_segment(node, i, writer)
                results.append(result)
                if not result.ok:
                    break

        return 0 if all(r.ok for r in results) else 10
    except KeyboardInterrupt:
        return 130
    except Exception as exc:
        with open(os.path.join(out_dir, "error.txt"), "w", encoding="utf-8") as f:
            f.write(str(exc) + "\n")
        return 1
    finally:
        try:
            node.publish_zero_burst(1.5)
        except Exception:
            pass
        try:
            pause_disable_result = node.set_correction_pause(False)
            node.spin_some(0.5)
        except Exception as exc:
            pause_disable_result = f"error={exc}"
        try:
            bridge_status = node.bridge_status
            with open(os.path.join(out_dir, "summary.md"), "w", encoding="utf-8") as f:
                f.write("# Ranger S-Curve Odom Test Summary\n\n")
                f.write(f"- distance_m: `{distance_m:.3f}`\n")
                f.write(f"- linear_speed_mps: `{linear_speed:.3f}`\n")
                f.write(f"- heading_peak_deg: `{math.degrees(heading_peak):.3f}`\n")
                f.write(f"- start_turn: `{start_turn}`\n")
                f.write(f"- repeat: `{repeat}`\n")
                f.write(f"- cmd_topic: `{cmd_topic}`\n")
                f.write(f"- odom_topic: `{odom_topic}`\n")
                f.write(f"- expected_final_forward_m: `{distance_m:.3f}`\n")
                f.write("- expected_final_lateral_m: `0.000`\n")
                f.write("- expected_final_yaw_deg: `0.000`\n")
                f.write(f"- distance_tolerance_m: `{distance_tolerance_m:.3f}`\n")
                f.write(f"- lateral_tolerance_m: `{lateral_tolerance_m:.3f}`\n")
                f.write(f"- yaw_tolerance_deg: `{math.degrees(yaw_tolerance):.3f}`\n")
                f.write(f"- pause_correction_enable: `{pause_enable_result}`\n")
                f.write(f"- pause_correction_disable: `{pause_disable_result}`\n")
                if bridge_status:
                    try:
                        bridge = json.loads(bridge_status)
                        f.write(f"- bridge_amcl_gate_mode: `{bridge.get('amcl_gate_mode', '')}`\n")
                        f.write(f"- bridge_correction_paused_final: `{bridge.get('correction_paused', bridge.get('map_odom_correction_paused', ''))}`\n")
                        f.write(f"- bridge_has_map_to_odom: `{bridge.get('has_map_to_odom', '')}`\n")
                    except Exception:
                        f.write("- bridge_status_parse: `failed`\n")
                f.write("\n| segment | ok | reason | duration_sec | wheel_forward_m | wheel_forward_error_m | wheel_lateral_m | wheel_distance_m | wheel_yaw_deg | local_forward_m | local_forward_error_m | local_lateral_m | local_distance_m | local_yaw_deg | max_expected_wz | max_cmd_safe_vx | max_cmd_safe_wz | max_cmd_out_vx | max_cmd_out_wz | max_motion_linear | max_motion_angular | max_motion_steering | motion_modes_seen | system_modes_seen |\n")
                f.write("|---:|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|---|\n")
                for r in results:
                    local_forward = "" if r.local_forward_m is None else f"{r.local_forward_m:.4f}"
                    local_error = "" if r.local_forward_error_m is None else f"{r.local_forward_error_m:.4f}"
                    local_lateral = "" if r.local_lateral_m is None else f"{r.local_lateral_m:.4f}"
                    local_distance = "" if r.local_distance_m is None else f"{r.local_distance_m:.4f}"
                    local_yaw_deg = "" if r.local_yaw_delta_rad is None else f"{math.degrees(r.local_yaw_delta_rad):.3f}"
                    f.write(
                        f"| {r.index} | {str(r.ok).lower()} | {r.reason} | {r.duration_sec:.3f} | "
                        f"{r.wheel_forward_m:.4f} | {r.wheel_forward_error_m:.4f} | "
                        f"{r.wheel_lateral_m:.4f} | {r.wheel_distance_m:.4f} | {math.degrees(r.wheel_yaw_delta_rad):.3f} | "
                        f"{local_forward} | {local_error} | {local_lateral} | {local_distance} | {local_yaw_deg} | "
                        f"{r.max_abs_expected_wz:.3f} | {r.max_abs_cmd_safe_vx:.3f} | {r.max_abs_cmd_safe_wz:.3f} | "
                        f"{r.max_abs_cmd_out_vx:.3f} | {r.max_abs_cmd_out_wz:.3f} | "
                        f"{r.max_abs_motion_linear:.3f} | {r.max_abs_motion_angular:.3f} | {r.max_abs_motion_steering:.3f} | "
                        f"{r.motion_modes_seen} | {r.system_modes_seen} |\n"
                    )
                if results:
                    f.write("\n## Last Safety Status\n\n")
                    f.write(f"`{results[-1].safety_status}`\n\n")
                    f.write("## Last Mode Controller Status\n\n")
                    f.write("```json\n")
                    f.write(results[-1].mode_status)
                    f.write("\n```\n")
        finally:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
PY

rc=$?
set -e
echo "[ranger-s-curve-test] report: ${OUT_DIR}"
exit "${rc}"
