#!/usr/bin/env bash
set -euo pipefail
umask 0002

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OVERLAY_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
PROJECT_ROOT="$(cd "${OVERLAY_ROOT}/../../.." && pwd)"

RADIUS_M="1.50"
LINEAR_SPEED_MPS="0.25"
ANGLE_DEG="360.0"
REPEAT="1"
SAMPLE_HZ="20.0"
COUNTDOWN_SEC="3"
SETTLE_SEC="3.0"
LABEL="right_arc_r1p5_360"
CMD_TOPIC="/cmd_vel_collision_checked"
ODOM_TOPIC="/wheel/odom"
LOCAL_ODOM_TOPIC="/local_state/odometry"
OUTPUT_ROOT="${PROJECT_ROOT}/reports/ranger_right_arc_odom_test"
PAUSE_CORRECTION="true"
CORRECTION_PAUSE_SERVICE="/robot_localization_bridge/set_correction_paused"
YAW_TOLERANCE_DEG="2.0"
CLOSURE_TOLERANCE_M="0.30"
STOP_LEAD_DEG="0.0"
MAX_EXTRA_SEC="15.0"

usage() {
  cat <<'EOF'
Usage: run_ranger_right_arc_odom_test.sh [options]

Runs an automated Ranger Mini 3 right Ackermann arc/circle odometry test. The
script publishes linear.x plus negative angular.z through the normal safety
chain and stops by accumulated /wheel/odom yaw.

Default profile:
  radius=1.5m, linear_speed=0.25m/s, angle=360deg right turn

Options:
  --radius-m M            Commanded turn radius. Default: 1.50
  --linear-speed MPS      Forward speed command. Default: 0.25
  --angle-deg DEG         Right-turn accumulated yaw target. Default: 360.0
  --repeat N              Repeat the segment N times. Default: 1
  --sample-hz HZ          Report sampling frequency. Default: 20.0
  --countdown-sec N       Countdown before motion. Default: 3
  --settle-sec SEC        Stop/record settle time after each segment. Default: 3.0
  --label NAME            Report label. Default: right_arc_r1p5_360
  --cmd-topic TOPIC       Safety-chain input topic. Default: /cmd_vel_collision_checked
  --odom-topic TOPIC      Yaw feedback topic. Default: /wheel/odom
  --local-odom-topic TOPIC Local odom topic to record. Default: /local_state/odometry
  --output-root DIR       Report root. Default: reports/ranger_right_arc_odom_test
  --no-pause-correction   Do not call bridge correction pause service.
  --yaw-tolerance-deg DEG Final accumulated-yaw tolerance. Default: 2.0
  --closure-tolerance-m M Final XY closure tolerance. Default: 0.30
  --stop-lead-deg DEG     Stop command lead to compensate braking/command-chain lag. Default: 0.0
  --max-extra-sec SEC     Extra timeout beyond angle/angular-speed. Default: 15.0

The command path remains:
  test script -> /cmd_vel_collision_checked -> robot_safety -> /cmd_vel -> ranger_base
  /cmd_vel_safe is a robot_safety diagnostic mirror.

After the report is written, run:
  capture_relocalize_correction_compare.sh --latest --kind right_arc
EOF
}

while [[ "$#" -gt 0 ]]; do
  case "$1" in
    --radius-m)
      RADIUS_M="${2:-}"
      shift 2
      ;;
    --linear-speed|--linear-speed-mps)
      LINEAR_SPEED_MPS="${2:-}"
      shift 2
      ;;
    --angle-deg)
      ANGLE_DEG="${2:-}"
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
    --yaw-tolerance-deg)
      YAW_TOLERANCE_DEG="${2:-}"
      shift 2
      ;;
    --closure-tolerance-m)
      CLOSURE_TOLERANCE_M="${2:-}"
      shift 2
      ;;
    --stop-lead-deg)
      STOP_LEAD_DEG="${2:-}"
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
      echo "[ranger-right-arc-test] unknown argument: $1" >&2
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

{
  echo "# Ranger Right Arc Odom Test Environment"
  echo "- timestamp_utc: ${timestamp}"
  echo "- radius_m: ${RADIUS_M}"
  echo "- linear_speed_mps: ${LINEAR_SPEED_MPS}"
  echo "- angle_deg: ${ANGLE_DEG}"
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
  echo "[ranger-right-arc-test] motion starts in ${COUNTDOWN_SEC}s. Ensure the right-turn circle envelope is clear and E-stop is available."
  remaining="${COUNTDOWN_SEC}"
  while [[ "${remaining}" -gt 0 ]]; do
    echo "[ranger-right-arc-test] ${remaining}..."
    sleep 1
    remaining=$((remaining - 1))
  done
fi

set +e
python3 - \
  "${OUT_DIR}" \
  "${RADIUS_M}" \
  "${LINEAR_SPEED_MPS}" \
  "${ANGLE_DEG}" \
  "${REPEAT}" \
  "${SAMPLE_HZ}" \
  "${SETTLE_SEC}" \
  "${CMD_TOPIC}" \
  "${ODOM_TOPIC}" \
  "${LOCAL_ODOM_TOPIC}" \
  "${PAUSE_CORRECTION}" \
  "${CORRECTION_PAUSE_SERVICE}" \
  "${YAW_TOLERANCE_DEG}" \
  "${CLOSURE_TOLERANCE_M}" \
  "${STOP_LEAD_DEG}" \
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
radius_m = float(sys.argv[2])
linear_speed = abs(float(sys.argv[3]))
angle_deg = abs(float(sys.argv[4]))
repeat = int(sys.argv[5])
sample_hz = float(sys.argv[6])
settle_sec = float(sys.argv[7])
cmd_topic = sys.argv[8]
odom_topic = sys.argv[9]
local_odom_topic = sys.argv[10]
pause_correction = sys.argv[11].lower() == "true"
correction_pause_service = sys.argv[12]
yaw_tolerance = math.radians(abs(float(sys.argv[13])))
closure_tolerance_m = abs(float(sys.argv[14]))
stop_lead = math.radians(abs(float(sys.argv[15])))
max_extra_sec = float(sys.argv[16])

if radius_m <= 0.0:
    raise SystemExit("radius must be positive")
if linear_speed <= 0.0:
    raise SystemExit("linear speed must be positive")
if angle_deg <= 0.0:
    raise SystemExit("angle must be positive")
if repeat < 1:
    raise SystemExit("repeat must be >= 1")
if sample_hz <= 0.0:
    raise SystemExit("sample_hz must be positive")

target_yaw = math.radians(angle_deg)
turn_sign = -1.0
commanded_wz = turn_sign * linear_speed / radius_m
timeout_sec = target_yaw / abs(commanded_wz) + max_extra_sec
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
    left = -dx * s + dy * c
    distance = math.hypot(dx, dy)
    yaw_delta = norm_angle(current[2] - start[2])
    return (forward, left, distance, yaw_delta)


def expected_arc(yaw_accum: float) -> Tuple[float, float, float]:
    theta = max(0.0, min(yaw_accum, target_yaw))
    return (
        radius_m * math.sin(theta),
        -radius_m * (1.0 - math.cos(theta)),
        norm_angle(turn_sign * theta),
    )


def fmt_optional(value: Optional[float], precision: int = 6) -> str:
    if value is None:
        return ""
    return f"{value:.{precision}f}"


def motion_value(msg: Optional[Any], field: str) -> str:
    if msg is None:
        return ""
    return str(getattr(msg, field, ""))


def motion_float(msg: Optional[Any], field: str) -> str:
    if msg is None or not hasattr(msg, field):
        return ""
    try:
        return f"{float(getattr(msg, field)):.6f}"
    except Exception:
        return str(getattr(msg, field, ""))


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
    wheel_yaw_accum_rad: float
    wheel_yaw_error_rad: float
    wheel_forward_m: float
    wheel_left_m: float
    wheel_closure_m: float
    wheel_peak_right_m: float
    local_yaw_accum_rad: Optional[float]
    local_yaw_error_rad: Optional[float]
    local_forward_m: Optional[float]
    local_left_m: Optional[float]
    local_closure_m: Optional[float]
    local_peak_right_m: Optional[float]
    max_abs_cmd_safe_vx: float
    max_abs_cmd_safe_wz: float
    max_abs_cmd_out_vx: float
    max_abs_cmd_out_wz: float
    motion_modes_seen: str
    system_modes_seen: str
    safety_status: str
    mode_status: str


class RightArcNode(Node):
    def __init__(self) -> None:
        super().__init__("ranger_right_arc_odom_test")
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

    def wait_for_odom(self, timeout: float = 5.0) -> bool:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.wheel_odom is not None:
                return True
        return False

    def wait_for_local_odom(self, timeout: float = 3.0) -> bool:
        deadline = time.monotonic() + timeout
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
    node: RightArcNode,
    segment: int,
    phase: str,
    elapsed: float,
    start_wheel: Tuple[float, float, float],
    start_local: Optional[Tuple[float, float, float]],
    wheel_yaw_accum: float,
    local_yaw_accum: Optional[float],
    cmd_vx: float,
    cmd_wz: float,
) -> Tuple[float, float, float]:
    wheel_pose = odom_pose(node.wheel_odom) if node.wheel_odom is not None else start_wheel
    wheel_forward, wheel_left, wheel_distance, wheel_yaw_norm = relative_components(start_wheel, wheel_pose)
    local_pose = pose_or_none(node.local_odom)
    local_forward = local_left = local_distance = local_yaw_norm = None
    if start_local is not None and local_pose is not None:
        local_forward, local_left, local_distance, local_yaw_norm = relative_components(start_local, local_pose)
    expected_forward, expected_left, expected_yaw_norm = expected_arc(wheel_yaw_accum)
    cmd_safe = twist_tuple(node.cmd_safe)
    cmd_out = twist_tuple(node.cmd_out)
    wheel_twist = odom_twist(node.wheel_odom)
    local_twist = odom_twist(node.local_odom)
    mode_cmd = status_cmd_out(node.mode_status)
    writer.writerow({
        "segment": segment,
        "phase": phase,
        "elapsed_sec": f"{elapsed:.4f}",
        "radius_m": f"{radius_m:.6f}",
        "target_yaw_rad": f"{target_yaw:.6f}",
        "expected_forward_m": f"{expected_forward:.6f}",
        "expected_left_m": f"{expected_left:.6f}",
        "expected_yaw_norm_rad": f"{expected_yaw_norm:.6f}",
        "wheel_x": f"{wheel_pose[0]:.6f}",
        "wheel_y": f"{wheel_pose[1]:.6f}",
        "wheel_yaw": f"{wheel_pose[2]:.6f}",
        "wheel_yaw_accum_rad": f"{wheel_yaw_accum:.6f}",
        "wheel_forward_m": f"{wheel_forward:.6f}",
        "wheel_left_m": f"{wheel_left:.6f}",
        "wheel_closure_m": f"{wheel_distance:.6f}",
        "wheel_yaw_norm_delta_rad": f"{wheel_yaw_norm:.6f}",
        "wheel_forward_error_m": f"{wheel_forward - expected_forward:.6f}",
        "wheel_left_error_m": f"{wheel_left - expected_left:.6f}",
        "wheel_twist_vx": fmt_optional(wheel_twist[0]),
        "wheel_twist_wz": fmt_optional(wheel_twist[2]),
        "local_x": "" if local_pose is None else f"{local_pose[0]:.6f}",
        "local_y": "" if local_pose is None else f"{local_pose[1]:.6f}",
        "local_yaw": "" if local_pose is None else f"{local_pose[2]:.6f}",
        "local_yaw_accum_rad": fmt_optional(local_yaw_accum),
        "local_forward_m": fmt_optional(local_forward),
        "local_left_m": fmt_optional(local_left),
        "local_closure_m": fmt_optional(local_distance),
        "local_yaw_norm_delta_rad": fmt_optional(local_yaw_norm),
        "local_twist_vx": fmt_optional(local_twist[0]),
        "local_twist_wz": fmt_optional(local_twist[2]),
        "motion_linear_velocity": motion_float(node.motion_state, "linear_velocity"),
        "motion_angular_velocity": motion_float(node.motion_state, "angular_velocity"),
        "motion_steering_angle": motion_float(node.motion_state, "steering_angle"),
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
    return wheel_forward, wheel_left, wheel_distance


def run_segment(node: RightArcNode, index: int, writer: csv.DictWriter) -> SegmentResult:
    if node.wheel_odom is None:
        return SegmentResult(index, False, "missing_initial_wheel_odom", 0.0, 0.0, -target_yaw, 0.0, 0.0, 0.0, 0.0, None, None, None, None, None, None, 0.0, 0.0, 0.0, 0.0, "", "", node.safety_status, node.mode_status)

    start_wheel = odom_pose(node.wheel_odom)
    start_local = pose_or_none(node.local_odom)
    last_wheel_yaw = start_wheel[2]
    last_local_yaw = start_local[2] if start_local else None
    wheel_yaw_accum = 0.0
    local_yaw_accum = 0.0 if start_local else None
    wheel_forward = 0.0
    wheel_left = 0.0
    wheel_closure = 0.0
    wheel_peak_right = 0.0
    local_forward: Optional[float] = None
    local_left: Optional[float] = None
    local_closure: Optional[float] = None
    local_peak_right: Optional[float] = 0.0 if start_local else None
    max_abs_cmd_safe_vx = 0.0
    max_abs_cmd_safe_wz = 0.0
    max_abs_cmd_out_vx = 0.0
    max_abs_cmd_out_wz = 0.0
    motion_modes: Set[str] = set()
    system_modes: Set[str] = set()
    observed_cmd_out = False
    ok = True
    reason = "target_reached"
    start_time = time.monotonic()
    next_sample = start_time

    while rclpy.ok():
        now = time.monotonic()
        elapsed = now - start_time
        if elapsed > timeout_sec:
            ok = False
            reason = "timeout"
            break

        node.publish_cmd(linear_speed, commanded_wz)
        rclpy.spin_once(node, timeout_sec=0.01)

        if node.wheel_odom is not None:
            wheel_pose = odom_pose(node.wheel_odom)
            delta = norm_angle(wheel_pose[2] - last_wheel_yaw)
            if abs(delta) < math.pi / 2.0:
                wheel_yaw_accum += turn_sign * delta
            last_wheel_yaw = wheel_pose[2]
            wheel_forward, wheel_left, wheel_closure, _ = relative_components(start_wheel, wheel_pose)
            wheel_peak_right = max(wheel_peak_right, -wheel_left)

        local_pose = pose_or_none(node.local_odom)
        if local_pose is not None and start_local is not None:
            if last_local_yaw is not None:
                delta = norm_angle(local_pose[2] - last_local_yaw)
                if abs(delta) < math.pi / 2.0 and local_yaw_accum is not None:
                    local_yaw_accum += turn_sign * delta
                last_local_yaw = local_pose[2]
            local_forward, local_left, local_closure, _ = relative_components(start_local, local_pose)
            if local_peak_right is not None:
                local_peak_right = max(local_peak_right, -local_left)

        if node.cmd_safe is not None:
            max_abs_cmd_safe_vx = max(max_abs_cmd_safe_vx, abs(node.cmd_safe.linear.x))
            max_abs_cmd_safe_wz = max(max_abs_cmd_safe_wz, abs(node.cmd_safe.angular.z))
        if node.cmd_out is not None:
            max_abs_cmd_out_vx = max(max_abs_cmd_out_vx, abs(node.cmd_out.linear.x))
            max_abs_cmd_out_wz = max(max_abs_cmd_out_wz, abs(node.cmd_out.angular.z))
            if abs(node.cmd_out.linear.x) >= linear_speed * 0.4 and abs(node.cmd_out.angular.z) >= abs(commanded_wz) * 0.4:
                observed_cmd_out = True

        motion_mode = motion_value(node.motion_state, "motion_mode")
        system_mode = motion_value(node.system_state, "motion_mode")
        if motion_mode:
            motion_modes.add(motion_mode)
        if system_mode:
            system_modes.add(system_mode)

        if now >= next_sample:
            write_sample(
                writer,
                node,
                index,
                "motion",
                elapsed,
                start_wheel,
                start_local,
                wheel_yaw_accum,
                local_yaw_accum,
                linear_speed,
                commanded_wz,
            )
            next_sample += sample_period

        if wheel_yaw_accum >= max(0.0, target_yaw - stop_lead):
            break

        time.sleep(0.005)

    node.publish_zero_burst(1.0)
    settle_until = time.monotonic() + settle_sec
    while time.monotonic() < settle_until and rclpy.ok():
        now = time.monotonic()
        elapsed = now - start_time
        node.publish_cmd(0.0, 0.0)
        rclpy.spin_once(node, timeout_sec=0.02)
        if node.wheel_odom is not None:
            wheel_pose = odom_pose(node.wheel_odom)
            delta = norm_angle(wheel_pose[2] - last_wheel_yaw)
            if abs(delta) < math.pi / 2.0:
                wheel_yaw_accum += turn_sign * delta
            last_wheel_yaw = wheel_pose[2]
            wheel_forward, wheel_left, wheel_closure, _ = relative_components(start_wheel, wheel_pose)
            wheel_peak_right = max(wheel_peak_right, -wheel_left)
        local_pose = pose_or_none(node.local_odom)
        if local_pose is not None and start_local is not None:
            if last_local_yaw is not None:
                delta = norm_angle(local_pose[2] - last_local_yaw)
                if abs(delta) < math.pi / 2.0 and local_yaw_accum is not None:
                    local_yaw_accum += turn_sign * delta
                last_local_yaw = local_pose[2]
            local_forward, local_left, local_closure, _ = relative_components(start_local, local_pose)
            if local_peak_right is not None:
                local_peak_right = max(local_peak_right, -local_left)
        if now >= next_sample:
            write_sample(
                writer,
                node,
                index,
                "settle",
                elapsed,
                start_wheel,
                start_local,
                wheel_yaw_accum,
                local_yaw_accum,
                0.0,
                0.0,
            )
            next_sample += sample_period
        time.sleep(0.03)

    duration = time.monotonic() - start_time
    wheel_yaw_error = wheel_yaw_accum - target_yaw
    local_yaw_error = None if local_yaw_accum is None else local_yaw_accum - target_yaw

    if ok and not observed_cmd_out:
        ok = False
        reason = "final_cmd_vel_not_observed"
    if ok and (abs(wheel_yaw_error) > yaw_tolerance or wheel_closure > closure_tolerance_m):
        ok = False
        reason = "target_reached_final_error_gt_tolerance"

    return SegmentResult(
        index=index,
        ok=ok,
        reason=reason,
        duration_sec=duration,
        wheel_yaw_accum_rad=wheel_yaw_accum,
        wheel_yaw_error_rad=wheel_yaw_error,
        wheel_forward_m=wheel_forward,
        wheel_left_m=wheel_left,
        wheel_closure_m=wheel_closure,
        wheel_peak_right_m=wheel_peak_right,
        local_yaw_accum_rad=local_yaw_accum,
        local_yaw_error_rad=local_yaw_error,
        local_forward_m=local_forward,
        local_left_m=local_left,
        local_closure_m=local_closure,
        local_peak_right_m=local_peak_right,
        max_abs_cmd_safe_vx=max_abs_cmd_safe_vx,
        max_abs_cmd_safe_wz=max_abs_cmd_safe_wz,
        max_abs_cmd_out_vx=max_abs_cmd_out_vx,
        max_abs_cmd_out_wz=max_abs_cmd_out_wz,
        motion_modes_seen=",".join(sorted(motion_modes)),
        system_modes_seen=",".join(sorted(system_modes)),
        safety_status=node.safety_status,
        mode_status=node.mode_status,
    )


def main() -> int:
    rclpy.init(args=None)
    node = RightArcNode()
    results: List[SegmentResult] = []
    pause_enable_result = "not_called"
    pause_disable_result = "not_called"
    try:
        if not node.wait_for_odom():
            raise RuntimeError(f"no odometry received on {odom_topic}")
        node.wait_for_local_odom()
        pause_enable_result = node.set_correction_pause(True)
        node.spin_some(0.5)

        samples_path = os.path.join(out_dir, "samples.csv")
        with open(samples_path, "w", newline="", encoding="utf-8") as f:
            fieldnames = [
                "segment",
                "phase",
                "elapsed_sec",
                "radius_m",
                "target_yaw_rad",
                "expected_forward_m",
                "expected_left_m",
                "expected_yaw_norm_rad",
                "wheel_x",
                "wheel_y",
                "wheel_yaw",
                "wheel_yaw_accum_rad",
                "wheel_forward_m",
                "wheel_left_m",
                "wheel_closure_m",
                "wheel_yaw_norm_delta_rad",
                "wheel_forward_error_m",
                "wheel_left_error_m",
                "wheel_twist_vx",
                "wheel_twist_wz",
                "local_x",
                "local_y",
                "local_yaw",
                "local_yaw_accum_rad",
                "local_forward_m",
                "local_left_m",
                "local_closure_m",
                "local_yaw_norm_delta_rad",
                "local_twist_vx",
                "local_twist_wz",
                "motion_linear_velocity",
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
            for segment in range(1, repeat + 1):
                result = run_segment(node, segment, writer)
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
            with open(os.path.join(out_dir, "summary.md"), "w", encoding="utf-8") as f:
                f.write("# Ranger Right Arc Odom Test Summary\n\n")
                f.write(f"- radius_m: `{radius_m:.3f}`\n")
                f.write(f"- linear_speed_mps: `{linear_speed:.3f}`\n")
                f.write(f"- commanded_angular_z_radps: `{commanded_wz:.6f}`\n")
                f.write(f"- angle_deg: `{angle_deg:.3f}`\n")
                f.write(f"- repeat: `{repeat}`\n")
                f.write(f"- cmd_topic: `{cmd_topic}`\n")
                f.write(f"- odom_topic: `{odom_topic}`\n")
                expected_final_forward, expected_final_left, _ = expected_arc(target_yaw)
                expected_peak_right = radius_m * (1.0 - math.cos(min(target_yaw, math.pi)))
                f.write(f"- expected_peak_right_m: `{expected_peak_right:.3f}`\n")
                f.write(f"- expected_final_forward_m: `{expected_final_forward:.3f}`\n")
                f.write(f"- expected_final_left_m: `{expected_final_left:.3f}`\n")
                f.write(f"- yaw_tolerance_deg: `{math.degrees(yaw_tolerance):.3f}`\n")
                f.write(f"- closure_tolerance_m: `{closure_tolerance_m:.3f}`\n")
                f.write(f"- stop_lead_deg: `{math.degrees(stop_lead):.3f}`\n")
                f.write(f"- pause_correction_enable: `{pause_enable_result}`\n")
                f.write(f"- pause_correction_disable: `{pause_disable_result}`\n")
                if node.bridge_status:
                    try:
                        bridge = json.loads(node.bridge_status)
                        f.write(f"- bridge_amcl_gate_mode: `{bridge.get('amcl_gate_mode', '')}`\n")
                        f.write(f"- bridge_correction_paused_final: `{bridge.get('correction_paused', bridge.get('map_odom_correction_paused', ''))}`\n")
                        f.write(f"- bridge_has_map_to_odom: `{bridge.get('has_map_to_odom', '')}`\n")
                    except Exception:
                        f.write("- bridge_status_parse: `failed`\n")
                f.write("\n| segment | ok | reason | duration_sec | wheel_yaw_deg | wheel_yaw_error_deg | wheel_forward_m | wheel_left_m | wheel_closure_m | wheel_peak_right_m | wheel_peak_right_error_m | wheel_effective_radius_m | local_yaw_deg | local_yaw_error_deg | local_forward_m | local_left_m | local_closure_m | local_peak_right_m | max_cmd_safe_vx | max_cmd_safe_wz | max_cmd_out_vx | max_cmd_out_wz | motion_modes_seen | system_modes_seen |\n")
                f.write("|---:|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|---|\n")
                for r in results:
                    local_yaw = "" if r.local_yaw_accum_rad is None else f"{math.degrees(r.local_yaw_accum_rad):.3f}"
                    local_yaw_error = "" if r.local_yaw_error_rad is None else f"{math.degrees(r.local_yaw_error_rad):.3f}"
                    local_forward = "" if r.local_forward_m is None else f"{r.local_forward_m:.4f}"
                    local_left = "" if r.local_left_m is None else f"{r.local_left_m:.4f}"
                    local_closure = "" if r.local_closure_m is None else f"{r.local_closure_m:.4f}"
                    local_peak_right = "" if r.local_peak_right_m is None else f"{r.local_peak_right_m:.4f}"
                    wheel_effective_radius = r.wheel_peak_right_m / 2.0
                    f.write(
                        f"| {r.index} | {str(r.ok).lower()} | {r.reason} | {r.duration_sec:.3f} | "
                        f"{math.degrees(r.wheel_yaw_accum_rad):.3f} | {math.degrees(r.wheel_yaw_error_rad):.3f} | "
                        f"{r.wheel_forward_m:.4f} | {r.wheel_left_m:.4f} | {r.wheel_closure_m:.4f} | {r.wheel_peak_right_m:.4f} | "
                        f"{r.wheel_peak_right_m - expected_peak_right:.4f} | {wheel_effective_radius:.4f} | "
                        f"{local_yaw} | {local_yaw_error} | {local_forward} | {local_left} | {local_closure} | {local_peak_right} | "
                        f"{r.max_abs_cmd_safe_vx:.3f} | {r.max_abs_cmd_safe_wz:.3f} | {r.max_abs_cmd_out_vx:.3f} | {r.max_abs_cmd_out_wz:.3f} | "
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
echo "[ranger-right-arc-test] report: ${OUT_DIR}"
exit "${rc}"
