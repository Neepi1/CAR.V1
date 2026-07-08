#!/usr/bin/env bash
set -euo pipefail
umask 0002

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OVERLAY_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
PROJECT_ROOT="$(cd "${OVERLAY_ROOT}/../../.." && pwd)"

PROFILE="standard"
SAMPLE_HZ="50.0"
COUNTDOWN_SEC="5"
CMD_TOPIC="/cmd_vel_collision_checked"
CAN_IFACE="${CAN_IFACE:-can0}"
OUTPUT_ROOT="${PROJECT_ROOT}/reports/ranger_chassis_dynamics_test"
PAUSE_CORRECTION="true"
CORRECTION_PAUSE_SERVICE="/robot_localization_bridge/set_correction_paused"
LABEL="chassis_dynamics"
INCLUDE_HIGH_SPEED="false"
DRY_RUN="false"
LINEAR_SPEEDS_OVERRIDE=""
ANGULAR_SPEEDS_OVERRIDE=""
LATERAL_SPEEDS_OVERRIDE=""

usage() {
  cat <<'EOF'
Usage: run_ranger_chassis_dynamics_test.sh [options]

Automated Ranger Mini 3 chassis dynamics calibration.

The command path remains:
  test script -> /cmd_vel_collision_checked -> robot_safety -> /cmd_vel -> ranger_base_node

This script measures:
  - command-chain latency
  - cmd_vel to CAN 0x221 / wheel odom response delay
  - acceleration and deceleration slope
  - braking stop time and stop distance
  - steering response delay
  - SDK / chassis internal speed smoothing/ramp
  - motion-mode switch latency from mode-controller status

Profiles:
  standard  low/medium linear + steering + spin mode tests, no 1.2m/s segment
  linear    linear step/brake tests only
  steering  Ackermann steering step tests only
  mode      spin/ackermann/lateral-mode switch tests only
  angular   angular step/brake tests only
  lateral   side-slip lateral step/brake tests only
  deadband  small command deadband tests only
  full      standard + deadband, still no 1.2m/s unless --include-high-speed

Options:
  --profile NAME             Default: standard
  --include-high-speed       Add a 1.2m/s straight/brake segment. Requires clear long space.
  --linear-speeds CSV        Override linear profile speeds, e.g. 0.40,0.35,0.30,0.25
  --angular-speeds CSV       Override angular profile speeds, e.g. 0.20,0.40,0.60,0.70
  --lateral-speeds CSV       Override lateral profile speeds, e.g. 0.04,0.08,0.12
  --sample-hz HZ             Default: 50.0
  --countdown-sec N          Default: 5
  --label NAME               Report label. Default: chassis_dynamics
  --cmd-topic TOPIC          Safety-chain input topic. Default: /cmd_vel_collision_checked
  --can-iface IFACE          CAN interface. Default: can0
  --output-root DIR          Default: reports/ranger_chassis_dynamics_test
  --no-pause-correction      Do not pause map->odom correction during the motion test
  --dry-run                  Print planned segments and exit without moving
  -h, --help                 Show this help

Run high-speed only in a straight clear lane with E-stop available.
EOF
}

while [[ "$#" -gt 0 ]]; do
  case "$1" in
    --profile)
      PROFILE="${2:-}"
      shift 2
      ;;
    --include-high-speed)
      INCLUDE_HIGH_SPEED="true"
      shift
      ;;
    --linear-speeds)
      LINEAR_SPEEDS_OVERRIDE="${2:-}"
      shift 2
      ;;
    --angular-speeds)
      ANGULAR_SPEEDS_OVERRIDE="${2:-}"
      shift 2
      ;;
    --lateral-speeds)
      LATERAL_SPEEDS_OVERRIDE="${2:-}"
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
    --label)
      LABEL="${2:-}"
      shift 2
      ;;
    --cmd-topic)
      CMD_TOPIC="${2:-}"
      shift 2
      ;;
    --can-iface)
      CAN_IFACE="${2:-}"
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
    --dry-run)
      DRY_RUN="true"
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "[ranger-dynamics] unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

case "${COUNTDOWN_SEC}" in
  ''|*[!0-9]*) echo "[ranger-dynamics] --countdown-sec must be an integer" >&2; exit 2 ;;
esac

python3 - "${SAMPLE_HZ}" <<'PY'
import sys
try:
    hz = float(sys.argv[1])
except Exception:
    raise SystemExit(2)
raise SystemExit(0 if hz > 0.0 else 2)
PY

if [[ -f "${SCRIPT_DIR}/common_env.sh" ]]; then
  # shellcheck source=/dev/null
  source "${SCRIPT_DIR}/common_env.sh"
fi
PROJECT_ROOT="${NJRH_PROJECT_ROOT:-${PROJECT_ROOT}}"
OUTPUT_ROOT="${OUTPUT_ROOT/#\~/${HOME}}"

safe_label="$(printf '%s' "${LABEL}" | tr -c 'A-Za-z0-9_.-' '_')"
timestamp="$(date -u +%Y%m%dT%H%M%SZ)"
OUT_DIR="${OUTPUT_ROOT}/${timestamp}_${safe_label}_${PROFILE}"
mkdir -p "${OUT_DIR}"

{
  echo "# Ranger Chassis Dynamics Test Environment"
  echo "- timestamp_utc: ${timestamp}"
  echo "- profile: ${PROFILE}"
  echo "- include_high_speed: ${INCLUDE_HIGH_SPEED}"
  echo "- linear_speeds_override: ${LINEAR_SPEEDS_OVERRIDE}"
  echo "- angular_speeds_override: ${ANGULAR_SPEEDS_OVERRIDE}"
  echo "- lateral_speeds_override: ${LATERAL_SPEEDS_OVERRIDE}"
  echo "- sample_hz: ${SAMPLE_HZ}"
  echo "- countdown_sec: ${COUNTDOWN_SEC}"
  echo "- cmd_topic: ${CMD_TOPIC}"
  echo "- can_iface: ${CAN_IFACE}"
  echo "- pause_correction: ${PAUSE_CORRECTION}"
  echo "- correction_pause_service: ${CORRECTION_PAUSE_SERVICE}"
  echo "- rmw: ${RMW_IMPLEMENTATION:-}"
  echo
  echo "## CAN"
  which candump 2>&1 || true
  ip -details link show "${CAN_IFACE}" 2>&1 || true
  echo
  echo "## ROS Nodes"
  ros2 node list 2>&1 || true
  echo
  echo "## Topic Info"
  for topic in \
    "${CMD_TOPIC}" \
    /cmd_vel_safe \
    /cmd_vel \
    /wheel/odom \
    /wheel/odom_ekf \
    /local_state/odometry \
    /motion_state \
    /system_state \
    /safety/status \
    /ranger_mini3_mode_controller/status \
    /localization/bridge_status; do
    echo "### ${topic}"
    ros2 topic info "${topic}" -v 2>&1 || true
  done
  echo
  echo "## Services"
  ros2 service list -t 2>&1 | grep -E 'robot_localization_bridge|global_localization|trigger_grid' || true
} >"${OUT_DIR}/environment.md"

if [[ "${COUNTDOWN_SEC}" -gt 0 && "${DRY_RUN}" != "true" ]]; then
  echo "[ranger-dynamics] profile=${PROFILE}, include_high_speed=${INCLUDE_HIGH_SPEED}"
  echo "[ranger-dynamics] motion starts in ${COUNTDOWN_SEC}s. Clear the lane and keep E-stop available."
  remaining="${COUNTDOWN_SEC}"
  while [[ "${remaining}" -gt 0 ]]; do
    echo "[ranger-dynamics] ${remaining}..."
    sleep 1
    remaining=$((remaining - 1))
  done
fi

python3 - \
  "${OUT_DIR}" \
  "${PROFILE}" \
  "${SAMPLE_HZ}" \
  "${CMD_TOPIC}" \
  "${CAN_IFACE}" \
  "${PAUSE_CORRECTION}" \
  "${CORRECTION_PAUSE_SERVICE}" \
  "${INCLUDE_HIGH_SPEED}" \
  "${DRY_RUN}" \
  "${LINEAR_SPEEDS_OVERRIDE}" \
  "${ANGULAR_SPEEDS_OVERRIDE}" \
  "${LATERAL_SPEEDS_OVERRIDE}" <<'PY'
import csv
import json
import math
import os
import re
import select
import signal
import subprocess
import sys
import threading
import time
from dataclasses import dataclass
from typing import Any, Dict, Iterable, List, Optional, Tuple

import rclpy
from rclpy.executors import SingleThreadedExecutor
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import SetBool

out_dir = sys.argv[1]
profile = sys.argv[2]
sample_hz = float(sys.argv[3])
cmd_topic = sys.argv[4]
can_iface = sys.argv[5]
pause_correction = sys.argv[6].lower() == "true"
correction_pause_service = sys.argv[7]
include_high_speed = sys.argv[8].lower() == "true"
dry_run = sys.argv[9].lower() == "true"
linear_speeds_override_raw = sys.argv[10].strip()
angular_speeds_override_raw = sys.argv[11].strip()
lateral_speeds_override_raw = sys.argv[12].strip()

WHEELBASE = 0.494
TRACK = 0.364
MODE_DUAL_ACKERMAN = 0
MODE_PARALLEL = 1
MODE_SPINNING = 2
MODE_SIDE_SLIP = 3


@dataclass
class Segment:
    name: str
    duration: float
    vx: float = 0.0
    vy: float = 0.0
    wz: float = 0.0
    kind: str = "hold"
    expected_mode: Optional[int] = None
    note: str = ""


def parse_positive_speeds(raw: str, option_name: str) -> List[float]:
    if not raw:
        return []
    speeds: List[float] = []
    for token in raw.split(","):
        value_text = token.strip()
        if not value_text:
            continue
        try:
            value = abs(float(value_text))
        except ValueError as exc:
            raise SystemExit(f"{option_name} contains non-numeric value: {value_text}") from exc
        if value <= 0.0:
            raise SystemExit(f"{option_name} values must be positive")
        speeds.append(value)
    if not speeds:
        raise SystemExit(f"{option_name} did not contain any usable positive speeds")
    return speeds


def speed_label(vx: float) -> str:
    return f"{vx:.2f}".replace(".", "p")


def linear_step_duration(vx: float) -> float:
    if vx >= 1.0:
        return 4.0
    return 3.0


def linear_brake_duration(vx: float) -> float:
    if vx >= 1.0:
        return 6.0
    if vx >= 0.55:
        return 4.0
    return 3.0


def build_segments(name: str, high_speed: bool, linear_speeds_override: List[float]) -> List[Segment]:
    segments: List[Segment] = [Segment("settle_zero_start", 1.0, kind="zero", note="initial zero")]

    def add_linear() -> None:
        if linear_speeds_override:
            for vx in linear_speeds_override:
                label = speed_label(vx)
                segments.extend([
                    Segment(
                        f"linear_step_{label}",
                        linear_step_duration(vx),
                        vx=vx,
                        kind="linear_accel",
                        expected_mode=MODE_DUAL_ACKERMAN,
                        note="custom linear speed override",
                    ),
                    Segment(
                        f"brake_from_{label}",
                        linear_brake_duration(vx),
                        kind="linear_decel",
                        expected_mode=MODE_DUAL_ACKERMAN,
                        note="custom linear speed override",
                    ),
                ])
            return
        segments.extend([
            Segment("linear_step_0p20", 3.0, vx=0.20, kind="linear_accel", expected_mode=MODE_DUAL_ACKERMAN),
            Segment("brake_from_0p20", 3.0, kind="linear_decel", expected_mode=MODE_DUAL_ACKERMAN),
            Segment("linear_step_0p60", 3.0, vx=0.60, kind="linear_accel", expected_mode=MODE_DUAL_ACKERMAN),
            Segment("brake_from_0p60", 4.0, kind="linear_decel", expected_mode=MODE_DUAL_ACKERMAN),
        ])
        if high_speed:
            segments.extend([
                Segment("linear_step_1p20", 4.0, vx=1.20, kind="linear_accel", expected_mode=MODE_DUAL_ACKERMAN,
                        note="requires clear long straight lane"),
                Segment("brake_from_1p20", 6.0, kind="linear_decel", expected_mode=MODE_DUAL_ACKERMAN),
            ])

    def add_steering() -> None:
        segments.extend([
            Segment("steer_left_v0p20_w0p15", 3.0, vx=0.20, wz=0.15, kind="steering_step",
                    expected_mode=MODE_DUAL_ACKERMAN),
            Segment("brake_after_steer_left", 3.0, kind="linear_decel", expected_mode=MODE_DUAL_ACKERMAN),
            Segment("steer_right_v0p20_w-0p15", 3.0, vx=0.20, wz=-0.15, kind="steering_step",
                    expected_mode=MODE_DUAL_ACKERMAN),
            Segment("brake_after_steer_right", 3.0, kind="linear_decel", expected_mode=MODE_DUAL_ACKERMAN),
        ])

    def add_mode() -> None:
        segments.extend([
            Segment("spin_pos_w0p25", 2.5, wz=0.25, kind="mode_switch", expected_mode=MODE_SPINNING),
            Segment("zero_after_spin_pos", 2.0, kind="zero", expected_mode=MODE_DUAL_ACKERMAN),
            Segment("spin_neg_w-0p25", 2.5, wz=-0.25, kind="mode_switch", expected_mode=MODE_SPINNING),
            Segment("zero_after_spin_neg", 2.0, kind="zero", expected_mode=MODE_DUAL_ACKERMAN),
            Segment("ackermann_v0p15_w0p12", 2.5, vx=0.15, wz=0.12, kind="mode_switch",
                    expected_mode=MODE_DUAL_ACKERMAN),
            Segment("zero_after_ackermann", 2.0, kind="zero", expected_mode=MODE_DUAL_ACKERMAN),
        ])

    def add_angular() -> None:
        speeds = angular_speeds_override or [0.20, 0.40, 0.60, 0.70]
        for wz in speeds:
            label = speed_label(wz)
            segments.extend([
                Segment(
                    f"angular_step_pos_{label}",
                    2.0,
                    wz=wz,
                    kind="angular_accel",
                    expected_mode=MODE_SPINNING,
                    note="positive spin dynamics",
                ),
                Segment(
                    f"angular_brake_pos_{label}",
                    2.5,
                    kind="angular_decel",
                    expected_mode=MODE_DUAL_ACKERMAN,
                    note="positive spin stop dynamics",
                ),
                Segment(
                    f"angular_step_neg_{label}",
                    2.0,
                    wz=-wz,
                    kind="angular_accel",
                    expected_mode=MODE_SPINNING,
                    note="negative spin dynamics",
                ),
                Segment(
                    f"angular_brake_neg_{label}",
                    2.5,
                    kind="angular_decel",
                    expected_mode=MODE_DUAL_ACKERMAN,
                    note="negative spin stop dynamics",
                ),
            ])

    def add_lateral() -> None:
        speeds = lateral_speeds_override or [0.04, 0.08, 0.12]
        for vy in speeds:
            label = speed_label(vy)
            segments.extend([
                Segment(
                    f"lateral_step_pos_{label}",
                    2.5,
                    vy=vy,
                    kind="lateral_accel",
                    expected_mode=MODE_PARALLEL,
                    note="forced parallel lateral positive dynamics",
                ),
                Segment(
                    f"lateral_brake_pos_{label}",
                    2.5,
                    kind="lateral_decel",
                    expected_mode=MODE_DUAL_ACKERMAN,
                    note="lateral stop dynamics",
                ),
                Segment(
                    f"lateral_step_neg_{label}",
                    2.5,
                    vy=-vy,
                    kind="lateral_accel",
                    expected_mode=MODE_PARALLEL,
                    note="forced parallel lateral negative dynamics",
                ),
                Segment(
                    f"lateral_brake_neg_{label}",
                    2.5,
                    kind="lateral_decel",
                    expected_mode=MODE_DUAL_ACKERMAN,
                    note="lateral stop dynamics",
                ),
            ])

    def add_deadband() -> None:
        for value in (0.01, 0.02, 0.03, 0.05, 0.08):
            segments.append(Segment(f"deadband_vx_{value:.2f}".replace(".", "p"), 1.2, vx=value,
                                    kind="deadband_linear", expected_mode=MODE_DUAL_ACKERMAN))
        segments.append(Segment("zero_after_deadband_vx", 2.0, kind="zero", expected_mode=MODE_DUAL_ACKERMAN))
        for value in (0.01, 0.02, 0.03, 0.05, 0.08):
            segments.append(Segment(f"deadband_wz_{value:.2f}".replace(".", "p"), 1.2, wz=value,
                                    kind="deadband_angular", expected_mode=MODE_SPINNING))
        segments.append(Segment("zero_after_deadband_wz", 2.0, kind="zero", expected_mode=MODE_DUAL_ACKERMAN))

    if name in ("standard", "linear", "full"):
        add_linear()
    if name in ("standard", "steering", "full"):
        add_steering()
    if name in ("standard", "mode", "full"):
        add_mode()
    if name in ("angular", "full"):
        add_angular()
    if name in ("lateral", "full"):
        add_lateral()
    if name in ("deadband", "full"):
        add_deadband()

    segments.append(Segment("final_zero", 2.0, kind="zero", expected_mode=MODE_DUAL_ACKERMAN))
    if len(segments) <= 2:
        raise SystemExit(f"unknown profile: {name}")
    return segments


def signed_i16(low: int, high: int) -> int:
    value = low | (high << 8)
    if value >= 0x8000:
        value -= 0x10000
    return value


def decode_can_221(data: List[int]) -> Dict[str, float]:
    return {
        "linear": signed_i16(data[1], data[0]) / 1000.0,
        "angular": signed_i16(data[3], data[2]) / 1000.0,
        "lateral": signed_i16(data[5], data[4]) / 1000.0,
        "steering": signed_i16(data[7], data[6]) / 1000.0,
    }


def yaw_from_quat(q: Any) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def twist_tuple(msg: Optional[Twist]) -> Tuple[float, float, float]:
    if msg is None:
        return (0.0, 0.0, 0.0)
    return (float(msg.linear.x), float(msg.linear.y), float(msg.angular.z))


def odom_pose_twist(msg: Optional[Odometry]) -> Dict[str, Any]:
    if msg is None:
        return {key: "" for key in ("x", "y", "yaw", "vx", "vy", "wz", "stamp_sec")}
    pose = msg.pose.pose
    twist = msg.twist.twist
    stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
    return {
        "x": pose.position.x,
        "y": pose.position.y,
        "yaw": yaw_from_quat(pose.orientation),
        "vx": twist.linear.x,
        "vy": twist.linear.y,
        "wz": twist.angular.z,
        "stamp_sec": stamp,
    }


def central_from_inner(angle: float) -> float:
    phi_i = abs(angle)
    phi = math.atan(
        WHEELBASE * math.sin(phi_i) /
        (WHEELBASE * math.cos(phi_i) + TRACK * math.sin(phi_i))
    )
    return phi if angle >= 0.0 else -phi


def expected_inner_steering(vx: float, wz: float) -> float:
    linear = abs(vx)
    angular = abs(wz)
    if linear < 1e-6 or angular < 1e-6:
        return 0.0
    central_arg = min((angular * WHEELBASE) / (2.0 * linear), 1.0)
    central_phi = math.asin(central_arg)
    phi = abs(central_phi)
    inner = math.atan(
        WHEELBASE * math.sin(phi) /
        max(1e-9, WHEELBASE * math.cos(phi) - TRACK * math.sin(phi))
    )
    sign = 1.0 if vx * wz >= 0.0 else -1.0
    return sign * min(inner, 40.0 * math.pi / 180.0)


class CandumpReader:
    def __init__(self, iface: str, out_path: str) -> None:
        self.latest_221: Optional[Dict[str, float]] = None
        self.latest_221_raw = ""
        self.count_221 = 0
        self.error = ""
        self.proc: Optional[subprocess.Popen[str]] = None
        self.raw = open(out_path, "w", encoding="utf-8")
        try:
            self.proc = subprocess.Popen(
                ["candump", iface],
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
            )
        except Exception as exc:
            self.error = repr(exc)
            self.proc = None

    def poll(self) -> None:
        if self.proc is None or self.proc.stdout is None:
            return
        while True:
            ready, _, _ = select.select([self.proc.stdout], [], [], 0.0)
            if not ready:
                break
            line = self.proc.stdout.readline()
            if not line:
                break
            line = line.rstrip("\n")
            self.raw.write(line + "\n")
            match = re.search(r"\b221\b\s+\[8\]\s+((?:[0-9A-Fa-f]{2}\s+){7}[0-9A-Fa-f]{2})", line)
            if not match:
                continue
            data = [int(x, 16) for x in match.group(1).split()]
            self.latest_221 = decode_can_221(data)
            self.latest_221_raw = " ".join(f"{x:02X}" for x in data)
            self.count_221 += 1

    def close(self) -> None:
        if self.proc is not None:
            try:
                self.proc.send_signal(signal.SIGINT)
                self.proc.wait(timeout=1.0)
            except Exception:
                try:
                    self.proc.terminate()
                except Exception:
                    pass
        self.raw.close()


class DynamicsNode(Node):
    def __init__(self) -> None:
        super().__init__("ranger_chassis_dynamics_test")
        qos = QoSProfile(depth=50)
        be = QoSProfile(depth=100)
        be.reliability = ReliabilityPolicy.BEST_EFFORT
        self.cmd_pub = self.create_publisher(Twist, cmd_topic, qos)
        self.forced_mode_pub = self.create_publisher(String, "/ranger_mini3/forced_mode", qos)
        self.pause_client = self.create_client(SetBool, correction_pause_service)
        self.latest: Dict[str, Any] = {
            "cmd_in": None,
            "cmd_safe": None,
            "cmd": None,
            "wheel": None,
            "wheel_ekf": None,
            "local": None,
            "mode_status": "",
            "safety_status": "",
            "bridge_status": "",
        }
        self.create_subscription(Twist, cmd_topic, lambda m: self._set("cmd_in", m), qos)
        self.create_subscription(Twist, "/cmd_vel_safe", lambda m: self._set("cmd_safe", m), qos)
        self.create_subscription(Twist, "/cmd_vel", lambda m: self._set("cmd", m), qos)
        self.create_subscription(Odometry, "/wheel/odom", lambda m: self._set("wheel", m), be)
        self.create_subscription(Odometry, "/wheel/odom_ekf", lambda m: self._set("wheel_ekf", m), be)
        self.create_subscription(Odometry, "/local_state/odometry", lambda m: self._set("local", m), be)
        self.create_subscription(String, "/ranger_mini3_mode_controller/status",
                                 lambda m: self._set("mode_status", m.data), qos)
        self.create_subscription(String, "/safety/status", lambda m: self._set("safety_status", m.data), qos)
        self.create_subscription(String, "/localization/bridge_status",
                                 lambda m: self._set("bridge_status", m.data), qos)

    def _set(self, name: str, value: Any) -> None:
        self.latest[name] = value

    def publish_cmd(self, vx: float, vy: float, wz: float) -> None:
        msg = Twist()
        msg.linear.x = float(vx)
        msg.linear.y = float(vy)
        msg.angular.z = float(wz)
        self.cmd_pub.publish(msg)

    def publish_forced_mode(self, mode: str) -> None:
        msg = String()
        msg.data = mode
        self.forced_mode_pub.publish(msg)

    def pause_corrections(self, paused: bool, timeout: float = 3.0) -> Tuple[bool, str]:
        if not pause_correction:
            return (True, "disabled_by_script_option")
        if not self.pause_client.wait_for_service(timeout_sec=timeout):
            return (False, "pause_service_unavailable")
        req = SetBool.Request()
        req.data = bool(paused)
        future = self.pause_client.call_async(req)
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            if future.done():
                try:
                    resp = future.result()
                    return (bool(resp.success), str(resp.message))
                except Exception as exc:
                    return (False, repr(exc))
            time.sleep(0.02)
        return (False, "pause_service_timeout")


def parse_mode_status(raw: str) -> Dict[str, Any]:
    out = {
        "desired_code": "",
        "actual_code": "",
        "actual_name": "",
        "mode_aligned": "",
    }
    if not raw:
        return out
    try:
        data = json.loads(raw)
    except Exception:
        return out
    desired = data.get("desired_motion_mode") or {}
    actual = data.get("actual_motion_mode") or {}
    out["desired_code"] = desired.get("code", "")
    out["actual_code"] = actual.get("code", "")
    out["actual_name"] = actual.get("name", "")
    out["mode_aligned"] = data.get("mode_aligned", "")
    return out


def row_from_state(
    node: DynamicsNode,
    can: CandumpReader,
    segment: Segment,
    segment_index: int,
    elapsed: float,
    segment_elapsed: float,
) -> Dict[str, Any]:
    cmd_in = twist_tuple(node.latest["cmd_in"])
    cmd_safe = twist_tuple(node.latest["cmd_safe"])
    cmd = twist_tuple(node.latest["cmd"])
    wheel = odom_pose_twist(node.latest["wheel"])
    wheel_ekf = odom_pose_twist(node.latest["wheel_ekf"])
    local = odom_pose_twist(node.latest["local"])
    mode = parse_mode_status(node.latest["mode_status"])
    can_221 = can.latest_221 or {}
    return {
        "elapsed_sec": elapsed,
        "segment_index": segment_index,
        "segment": segment.name,
        "segment_elapsed_sec": segment_elapsed,
        "segment_kind": segment.kind,
        "req_vx": segment.vx,
        "req_vy": segment.vy,
        "req_wz": segment.wz,
        "expected_mode": "" if segment.expected_mode is None else segment.expected_mode,
        "cmd_in_vx": cmd_in[0],
        "cmd_in_vy": cmd_in[1],
        "cmd_in_wz": cmd_in[2],
        "cmd_safe_vx": cmd_safe[0],
        "cmd_safe_vy": cmd_safe[1],
        "cmd_safe_wz": cmd_safe[2],
        "cmd_vx": cmd[0],
        "cmd_vy": cmd[1],
        "cmd_wz": cmd[2],
        "can221_count": can.count_221,
        "can221_raw": can.latest_221_raw,
        "can221_linear": can_221.get("linear", ""),
        "can221_angular": can_221.get("angular", ""),
        "can221_lateral": can_221.get("lateral", ""),
        "can221_steering": can_221.get("steering", ""),
        "wheel_x": wheel["x"],
        "wheel_y": wheel["y"],
        "wheel_yaw": wheel["yaw"],
        "wheel_vx": wheel["vx"],
        "wheel_vy": wheel["vy"],
        "wheel_wz": wheel["wz"],
        "wheel_stamp_sec": wheel["stamp_sec"],
        "wheel_ekf_x": wheel_ekf["x"],
        "wheel_ekf_y": wheel_ekf["y"],
        "wheel_ekf_yaw": wheel_ekf["yaw"],
        "wheel_ekf_vx": wheel_ekf["vx"],
        "wheel_ekf_vy": wheel_ekf["vy"],
        "wheel_ekf_wz": wheel_ekf["wz"],
        "wheel_ekf_stamp_sec": wheel_ekf["stamp_sec"],
        "local_x": local["x"],
        "local_y": local["y"],
        "local_yaw": local["yaw"],
        "local_vx": local["vx"],
        "local_vy": local["vy"],
        "local_wz": local["wz"],
        "local_stamp_sec": local["stamp_sec"],
        "mode_desired_code": mode["desired_code"],
        "mode_actual_code": mode["actual_code"],
        "mode_actual_name": mode["actual_name"],
        "mode_aligned": mode["mode_aligned"],
        "safety_status": node.latest["safety_status"],
    }


def finite_float(value: Any) -> Optional[float]:
    try:
        if value == "":
            return None
        f = float(value)
        if math.isfinite(f):
            return f
    except Exception:
        pass
    return None


def actual_linear(row: Dict[str, Any]) -> float:
    can = finite_float(row.get("can221_linear"))
    if can is not None:
        return can
    return float(row.get("wheel_vx") or 0.0)


def actual_lateral(row: Dict[str, Any]) -> float:
    wheel = float(row.get("wheel_vy") or 0.0)
    can = finite_float(row.get("can221_lateral"))
    if can is not None and abs(can) > 1e-6:
        return can
    return wheel


def actual_steering(row: Dict[str, Any]) -> Optional[float]:
    return finite_float(row.get("can221_steering"))


def actual_yaw_rate(row: Dict[str, Any]) -> float:
    can = finite_float(row.get("can221_angular"))
    if can is not None and abs(can) > 1e-6:
        return can
    return float(row.get("wheel_wz") or 0.0)


def first_cross(rows: Iterable[Dict[str, Any]], getter, threshold: float, sign: float = 1.0) -> Optional[float]:
    for row in rows:
        value = getter(row)
        if value is None:
            continue
        if sign >= 0.0 and value >= threshold:
            return float(row["segment_elapsed_sec"])
        if sign < 0.0 and value <= -threshold:
            return float(row["segment_elapsed_sec"])
    return None


def mean(values: List[float]) -> Optional[float]:
    if not values:
        return None
    return sum(values) / len(values)


def distance_between(a: Dict[str, Any], b: Dict[str, Any]) -> Optional[float]:
    ax = finite_float(a.get("wheel_x"))
    ay = finite_float(a.get("wheel_y"))
    bx = finite_float(b.get("wheel_x"))
    by = finite_float(b.get("wheel_y"))
    if None in (ax, ay, bx, by):
        return None
    return math.hypot(float(bx) - float(ax), float(by) - float(ay))


def normalize_angle(rad: float) -> float:
    return math.atan2(math.sin(rad), math.cos(rad))


def yaw_between(a: Dict[str, Any], b: Dict[str, Any]) -> Optional[float]:
    ayaw = finite_float(a.get("wheel_yaw"))
    byaw = finite_float(b.get("wheel_yaw"))
    if None in (ayaw, byaw):
        return None
    return normalize_angle(float(byaw) - float(ayaw))


def integrate_can_linear_distance(rows: List[Dict[str, Any]]) -> float:
    distance = 0.0
    last_t: Optional[float] = None
    last_v: Optional[float] = None
    for row in rows:
        t = finite_float(row.get("segment_elapsed_sec"))
        v = finite_float(row.get("can221_linear"))
        if t is None or v is None:
            continue
        if last_t is not None and last_v is not None:
            distance += max(0.0, 0.5 * (v + last_v)) * max(0.0, t - last_t)
        last_t = t
        last_v = v
    return distance


def integrate_can_axis_distance(rows: List[Dict[str, Any]], key: str) -> float:
    distance = 0.0
    last_t: Optional[float] = None
    last_v: Optional[float] = None
    for row in rows:
        t = finite_float(row.get("segment_elapsed_sec"))
        v = finite_float(row.get(key))
        if t is None or v is None:
            continue
        if last_t is not None and last_v is not None:
            distance += 0.5 * (v + last_v) * max(0.0, t - last_t)
        last_t = t
        last_v = v
    return distance


def compute_segment_metrics(segment: Segment, rows: List[Dict[str, Any]]) -> Dict[str, Any]:
    metric: Dict[str, Any] = {
        "segment": segment.name,
        "kind": segment.kind,
        "duration_sec": segment.duration,
        "req_vx": segment.vx,
        "req_vy": segment.vy,
        "req_wz": segment.wz,
        "expected_mode": segment.expected_mode,
        "note": segment.note,
    }
    if not rows:
        metric["sample_count"] = 0
        return metric
    metric["sample_count"] = len(rows)

    metric["cmd_chain_latency_sec"] = None
    command_axis = "vx"
    target = segment.vx
    if abs(segment.wz) > abs(segment.vx):
        command_axis = "wz"
        target = segment.wz
    elif abs(segment.vy) > abs(segment.vx):
        command_axis = "vy"
        target = segment.vy

    if abs(target) > 1e-6:
        sign = 1.0 if target >= 0.0 else -1.0
        cmd_key = {"vx": "cmd_vx", "vy": "cmd_vy", "wz": "cmd_wz"}[command_axis]
        metric["cmd_chain_latency_sec"] = first_cross(
            rows,
            lambda row, key=cmd_key: finite_float(row.get(key)),
            abs(target) * 0.5,
            sign,
        )

    if segment.kind in ("linear_accel", "deadband_linear"):
        target_v = segment.vx
        sign = 1.0 if target_v >= 0.0 else -1.0
        abs_target = max(abs(target_v), 1e-6)
        t10 = first_cross(rows, actual_linear, abs_target * 0.10, sign)
        t90 = first_cross(rows, actual_linear, abs_target * 0.90, sign)
        steady_rows = [r for r in rows if float(r["segment_elapsed_sec"]) >= max(0.0, segment.duration - 0.5)]
        steady_values = [actual_linear(r) for r in steady_rows]
        metric.update({
            "actual_delay_10pct_sec": t10,
            "rise_10_to_90_sec": None if t10 is None or t90 is None else max(0.0, t90 - t10),
            "steady_actual_linear_mps": mean(steady_values),
            "steady_gain": None if not steady_values or abs_target < 1e-6 else mean(steady_values) / target_v,
            "max_actual_linear_mps": max((actual_linear(r) for r in rows), key=abs),
        })

    if segment.kind == "lateral_accel":
        target_v = segment.vy
        sign = 1.0 if target_v >= 0.0 else -1.0
        abs_target = max(abs(target_v), 1e-6)
        t10 = first_cross(rows, actual_lateral, abs_target * 0.10, sign)
        t90 = first_cross(rows, actual_lateral, abs_target * 0.90, sign)
        steady_rows = [r for r in rows if float(r["segment_elapsed_sec"]) >= max(0.0, segment.duration - 0.5)]
        steady_values = [actual_lateral(r) for r in steady_rows]
        metric.update({
            "actual_delay_10pct_sec": t10,
            "rise_10_to_90_sec": None if t10 is None or t90 is None else max(0.0, t90 - t10),
            "steady_actual_lateral_mps": mean(steady_values),
            "steady_gain": None if not steady_values or abs_target < 1e-6 else mean(steady_values) / target_v,
            "max_actual_lateral_mps": max((actual_lateral(r) for r in rows), key=abs),
        })

    if segment.kind == "angular_accel":
        target_w = segment.wz
        sign = 1.0 if target_w >= 0.0 else -1.0
        abs_target = max(abs(target_w), 1e-6)
        t10 = first_cross(rows, actual_yaw_rate, abs_target * 0.10, sign)
        t90 = first_cross(rows, actual_yaw_rate, abs_target * 0.90, sign)
        steady_rows = [r for r in rows if float(r["segment_elapsed_sec"]) >= max(0.0, segment.duration - 0.5)]
        steady_values = [actual_yaw_rate(r) for r in steady_rows]
        metric.update({
            "actual_delay_10pct_sec": t10,
            "rise_10_to_90_sec": None if t10 is None or t90 is None else max(0.0, t90 - t10),
            "steady_actual_wz_radps": mean(steady_values),
            "steady_gain": None if not steady_values or abs_target < 1e-6 else mean(steady_values) / target_w,
            "max_abs_yaw_rate_radps": max((abs(actual_yaw_rate(r)) for r in rows), default=None),
        })

    if segment.kind == "linear_decel" or segment.kind == "zero":
        start_abs = max(abs(actual_linear(r)) for r in rows[:max(1, min(len(rows), int(sample_hz * 0.5)))])
        stop_index = None
        for idx, row in enumerate(rows):
            if abs(actual_linear(row)) < 0.02 and abs(actual_yaw_rate(row)) < 0.02:
                window = rows[idx:idx + max(1, int(sample_hz * 0.2))]
                if all(abs(actual_linear(w)) < 0.03 for w in window):
                    stop_index = idx
                    break
        stop_time = None if stop_index is None else float(rows[stop_index]["segment_elapsed_sec"])
        stop_distance = None if stop_index is None else distance_between(rows[0], rows[stop_index])
        metric.update({
            "initial_abs_linear_mps": start_abs,
            "stop_time_sec": stop_time,
            "wheel_odom_stop_distance_m": stop_distance,
            "can_integrated_stop_distance_m": integrate_can_linear_distance(rows),
            "residual_abs_linear_mps": abs(actual_linear(rows[-1])),
        })

    if segment.kind == "lateral_decel":
        start_abs = max(abs(actual_lateral(r)) for r in rows[:max(1, min(len(rows), int(sample_hz * 0.5)))])
        stop_index = None
        for idx, row in enumerate(rows):
            if abs(actual_lateral(row)) < 0.02 and abs(actual_linear(row)) < 0.02 and abs(actual_yaw_rate(row)) < 0.02:
                window = rows[idx:idx + max(1, int(sample_hz * 0.2))]
                if all(abs(actual_lateral(w)) < 0.03 for w in window):
                    stop_index = idx
                    break
        stop_time = None if stop_index is None else float(rows[stop_index]["segment_elapsed_sec"])
        stop_distance = None if stop_index is None else distance_between(rows[0], rows[stop_index])
        metric.update({
            "initial_abs_lateral_mps": start_abs,
            "stop_time_sec": stop_time,
            "wheel_odom_stop_distance_m": stop_distance,
            "can_integrated_stop_lateral_m": integrate_can_axis_distance(rows, "can221_lateral"),
            "residual_abs_lateral_mps": abs(actual_lateral(rows[-1])),
        })

    if segment.kind == "angular_decel":
        start_abs = max(abs(actual_yaw_rate(r)) for r in rows[:max(1, min(len(rows), int(sample_hz * 0.5)))])
        stop_index = None
        for idx, row in enumerate(rows):
            if abs(actual_yaw_rate(row)) < 0.02 and abs(actual_linear(row)) < 0.02 and abs(actual_lateral(row)) < 0.02:
                window = rows[idx:idx + max(1, int(sample_hz * 0.2))]
                if all(abs(actual_yaw_rate(w)) < 0.03 for w in window):
                    stop_index = idx
                    break
        stop_time = None if stop_index is None else float(rows[stop_index]["segment_elapsed_sec"])
        yaw_overrun = None if stop_index is None else yaw_between(rows[0], rows[stop_index])
        metric.update({
            "initial_abs_yaw_rate_radps": start_abs,
            "stop_time_sec": stop_time,
            "wheel_odom_stop_yaw_deg": None if yaw_overrun is None else math.degrees(yaw_overrun),
            "can_integrated_stop_yaw_rad": integrate_can_axis_distance(rows, "can221_angular"),
            "residual_abs_yaw_rate_radps": abs(actual_yaw_rate(rows[-1])),
        })

    if segment.kind == "steering_step":
        expected = expected_inner_steering(segment.vx, segment.wz)
        sign = 1.0 if expected >= 0.0 else -1.0
        abs_expected = max(abs(expected), 1e-6)
        t10 = first_cross(rows, actual_steering, abs_expected * 0.10, sign)
        t90 = first_cross(rows, actual_steering, abs_expected * 0.90, sign)
        steering_values = [v for v in (actual_steering(r) for r in rows) if v is not None]
        steady_rows = [r for r in rows if float(r["segment_elapsed_sec"]) >= max(0.0, segment.duration - 0.5)]
        steady_steering = [v for v in (actual_steering(r) for r in steady_rows) if v is not None]
        metric.update({
            "expected_inner_steering_rad": expected,
            "steering_delay_10pct_sec": t10,
            "steering_rise_10_to_90_sec": None if t10 is None or t90 is None else max(0.0, t90 - t10),
            "steady_can221_steering_rad": mean(steady_steering),
            "max_abs_can221_steering_rad": None if not steering_values else max(abs(v) for v in steering_values),
            "steady_yaw_rate_radps": mean([actual_yaw_rate(r) for r in steady_rows]),
        })

    if segment.kind == "mode_switch":
        desired = segment.expected_mode
        mode_time = None
        if desired is not None:
            for row in rows:
                try:
                    if int(row.get("mode_actual_code")) == desired:
                        mode_time = float(row["segment_elapsed_sec"])
                        break
                except Exception:
                    pass
        metric["mode_switch_latency_sec"] = mode_time
        metric["final_mode_actual_code"] = rows[-1].get("mode_actual_code")
        metric["final_mode_actual_name"] = rows[-1].get("mode_actual_name")

    if segment.kind == "deadband_angular":
        yaw_values = [abs(actual_yaw_rate(r)) for r in rows]
        metric["max_abs_yaw_rate_radps"] = max(yaw_values) if yaw_values else None
        metric["mean_abs_yaw_rate_radps"] = mean(yaw_values)

    return metric


linear_speeds_override = parse_positive_speeds(linear_speeds_override_raw, "--linear-speeds")
angular_speeds_override = parse_positive_speeds(angular_speeds_override_raw, "--angular-speeds")
lateral_speeds_override = parse_positive_speeds(lateral_speeds_override_raw, "--lateral-speeds")
segments = build_segments(profile, include_high_speed, linear_speeds_override)
plan_path = os.path.join(out_dir, "planned_segments.json")
with open(plan_path, "w", encoding="utf-8") as handle:
    json.dump([segment.__dict__ for segment in segments], handle, indent=2)

print(f"[ranger-dynamics] planned_segments: {plan_path}")
for i, segment in enumerate(segments, start=1):
    print(
        f"[ranger-dynamics] {i:02d} {segment.name} "
        f"duration={segment.duration:.1f}s vx={segment.vx:.3f} vy={segment.vy:.3f} wz={segment.wz:.3f} "
        f"kind={segment.kind}")

if dry_run:
    raise SystemExit(0)

rclpy.init()
node = DynamicsNode()
executor = SingleThreadedExecutor()
executor.add_node(node)
spin_thread = threading.Thread(target=executor.spin, daemon=True)
spin_thread.start()
can = CandumpReader(can_iface, os.path.join(out_dir, "candump.log"))
samples: List[Dict[str, Any]] = []
segment_metrics: List[Dict[str, Any]] = []
pause_enable: Tuple[bool, str] = (True, "not_requested")
pause_disable: Tuple[bool, str] = (True, "not_requested")
start_wall = time.monotonic()
sample_period = 1.0 / sample_hz

try:
    for _ in range(10):
        node.publish_cmd(0.0, 0.0, 0.0)
        time.sleep(0.02)

    pause_enable = node.pause_corrections(True)

    for segment_index, segment in enumerate(segments, start=1):
        print(f"[ranger-dynamics] segment {segment_index}/{len(segments)} {segment.name}")
        forced_mode = "parallel" if segment.kind == "lateral_accel" else "auto"
        for _ in range(5):
            node.publish_forced_mode(forced_mode)
            time.sleep(0.02)
        seg_start = time.monotonic()
        seg_rows: List[Dict[str, Any]] = []
        next_sample = seg_start
        while rclpy.ok() and (time.monotonic() - seg_start) <= segment.duration:
            now = time.monotonic()
            if segment.kind == "lateral_accel":
                node.publish_forced_mode("parallel")
            node.publish_cmd(segment.vx, segment.vy, segment.wz)
            can.poll()
            if now >= next_sample:
                row = row_from_state(
                    node,
                    can,
                    segment,
                    segment_index,
                    now - start_wall,
                    now - seg_start,
                )
                samples.append(row)
                seg_rows.append(row)
                next_sample += sample_period
            time.sleep(0.005)

        segment_metrics.append(compute_segment_metrics(segment, seg_rows))

    for _ in range(30):
        node.publish_forced_mode("auto")
        node.publish_cmd(0.0, 0.0, 0.0)
        time.sleep(0.02)
finally:
    for _ in range(20):
        try:
            node.publish_cmd(0.0, 0.0, 0.0)
            node.publish_forced_mode("auto")
            time.sleep(0.02)
        except Exception:
            pass
    pause_disable = node.pause_corrections(False)
    can.close()
    executor.shutdown()
    spin_thread.join(timeout=1.0)
    node.destroy_node()
    rclpy.shutdown()

csv_path = os.path.join(out_dir, "samples.csv")
fieldnames = sorted({key for row in samples for key in row.keys()})
with open(csv_path, "w", newline="", encoding="utf-8") as handle:
    writer = csv.DictWriter(handle, fieldnames=fieldnames)
    writer.writeheader()
    writer.writerows(samples)

metrics = {
    "profile": profile,
    "include_high_speed": include_high_speed,
    "sample_hz": sample_hz,
    "cmd_topic": cmd_topic,
    "can_iface": can_iface,
    "can_error": can.error,
    "can221_count": can.count_221,
    "pause_correction_enable": {"success": pause_enable[0], "message": pause_enable[1]},
    "pause_correction_disable": {"success": pause_disable[0], "message": pause_disable[1]},
    "segments": segment_metrics,
}
metrics_path = os.path.join(out_dir, "metrics.json")
with open(metrics_path, "w", encoding="utf-8") as handle:
    json.dump(metrics, handle, indent=2)

summary_path = os.path.join(out_dir, "summary.md")
with open(summary_path, "w", encoding="utf-8") as handle:
    handle.write("# Ranger Chassis Dynamics Test Summary\n\n")
    handle.write(f"- profile: `{profile}`\n")
    handle.write(f"- include_high_speed: `{include_high_speed}`\n")
    handle.write(f"- linear_speeds_override: `{linear_speeds_override}`\n")
    handle.write(f"- cmd_topic: `{cmd_topic}`\n")
    handle.write(f"- can_iface: `{can_iface}`\n")
    handle.write(f"- can221_count: `{can.count_221}`\n")
    handle.write(f"- pause_correction_enable: `success={pause_enable[0]} message={pause_enable[1]}`\n")
    handle.write(f"- pause_correction_disable: `success={pause_disable[0]} message={pause_disable[1]}`\n")
    handle.write("\n")
    handle.write("## Metrics\n\n")
    columns = [
        "segment",
        "kind",
        "req_vx",
        "req_wz",
        "cmd_chain_latency_sec",
        "actual_delay_10pct_sec",
        "rise_10_to_90_sec",
        "steady_actual_linear_mps",
        "steady_actual_lateral_mps",
        "steady_actual_wz_radps",
        "stop_time_sec",
        "can_integrated_stop_distance_m",
        "can_integrated_stop_lateral_m",
        "can_integrated_stop_yaw_rad",
        "wheel_odom_stop_distance_m",
        "wheel_odom_stop_yaw_deg",
        "expected_inner_steering_rad",
        "steering_delay_10pct_sec",
        "steering_rise_10_to_90_sec",
        "steady_can221_steering_rad",
        "mode_switch_latency_sec",
        "final_mode_actual_code",
    ]
    handle.write("| " + " | ".join(columns) + " |\n")
    handle.write("|" + "|".join("---" for _ in columns) + "|\n")
    for metric in segment_metrics:
        values = []
        for col in columns:
            value = metric.get(col, "")
            if isinstance(value, float):
                values.append(f"{value:.4f}")
            elif value is None:
                values.append("")
            else:
                values.append(str(value))
        handle.write("| " + " | ".join(values) + " |\n")
    handle.write("\n")
    handle.write("## Interpretation Guide\n\n")
    handle.write("- `cmd_chain_latency_sec`: test command to final `/cmd_vel` latency through `robot_safety`.\n")
    handle.write("- `actual_delay_10pct_sec`: final command to measurable chassis/wheel response delay.\n")
    handle.write("- If `/cmd_vel` is a step but CAN 0x221 rises slowly, smoothing/limit is inside SDK or chassis firmware.\n")
    handle.write("- `can_integrated_stop_distance_m` is integrated from CAN 0x221 linear feedback during a zero-command segment.\n")
    handle.write("- `wheel_odom_stop_distance_m` is measured from `/wheel/odom` pose; compare it with CAN distance to catch odom lag or replay artifacts.\n")
    handle.write("- Steering metrics use CAN 0x221 steering feedback; missing values usually mean `candump` is unavailable.\n")
    handle.write("- Mode switch latency uses `/ranger_mini3_mode_controller/status.actual_motion_mode`.\n")

print(f"[ranger-dynamics] samples: {csv_path}")
print(f"[ranger-dynamics] metrics: {metrics_path}")
print(f"[ranger-dynamics] summary: {summary_path}")
print(f"[ranger-dynamics] complete: {out_dir}")
PY

echo "[ranger-dynamics] report: ${OUT_DIR}"
