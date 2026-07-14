#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/common_env.sh"

CMD_TOPIC="/cmd_vel_collision_checked"
LINEAR_SPEED="0.00"
ANGULAR_SPEED="0.20"
COMMAND_SEC="1.50"
SETTLE_SEC="5.00"
SAMPLE_HZ="50.0"
COUNTDOWN_SEC="3"
ZERO_THRESH="0.005"
WHEEL_STOP_THRESH="0.025"
IMU_STOP_THRESH="0.035"
TARGET_YAW_DEG=""
ANGLE_TOLERANCE_DEG="1.0"
DISCOVERY_TIMEOUT_SEC="10.0"
LABEL="cmd_vel_stop_latency"
OUTPUT_ROOT="/tmp/cmd_vel_chain_latency"

usage() {
  cat <<'EOF'
Usage: record_cmd_vel_stop_latency.sh [options]

Runs a short Twist command and records stop propagation through:
  input topic -> /cmd_vel_safe -> /cmd_vel -> /wheel/odom

Options:
  --cmd-topic TOPIC          Command input topic. Default: /cmd_vel_collision_checked
  --linear-speed MPS         Commanded linear.x. Default: 0.00
  --angular-speed RADPS      Commanded angular.z. Default: 0.20
  --command-sec SEC          Nonzero command duration. Default: 1.50
  --target-yaw-deg DEG       Stop when /wheel/odom reaches this signed yaw delta.
                             When set, --command-sec is used as the timeout.
  --angle-tolerance-deg DEG  Stop early tolerance for target yaw. Default: 1.0
  --settle-sec SEC           Zero-command record duration. Default: 5.00
  --sample-hz HZ             CSV sample rate. Default: 50
  --countdown-sec SEC        Countdown before motion. Default: 3
  --zero-thresh VALUE        Topic considered zero below this linear.x and angular.z. Default: 0.005
  --wheel-stop-thresh VALUE  Wheel odom considered stopped below this linear.x and angular.z. Default: 0.025
  --imu-stop-thresh VALUE    Corrected IMU yaw rate considered stopped. Default: 0.035
  --discovery-timeout-sec SEC
                             Wait for wheel odom and the complete command chain. Default: 10.0
  --label NAME               Report label.
  --output-root DIR          Report root. Default: /tmp/cmd_vel_chain_latency
  -h, --help                 Show this help.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --cmd-topic) CMD_TOPIC="${2:-}"; shift 2 ;;
    --linear-speed) LINEAR_SPEED="${2:-}"; shift 2 ;;
    --angular-speed) ANGULAR_SPEED="${2:-}"; shift 2 ;;
    --command-sec) COMMAND_SEC="${2:-}"; shift 2 ;;
    --target-yaw-deg) TARGET_YAW_DEG="${2:-}"; shift 2 ;;
    --angle-tolerance-deg) ANGLE_TOLERANCE_DEG="${2:-}"; shift 2 ;;
    --settle-sec) SETTLE_SEC="${2:-}"; shift 2 ;;
    --sample-hz) SAMPLE_HZ="${2:-}"; shift 2 ;;
    --countdown-sec) COUNTDOWN_SEC="${2:-}"; shift 2 ;;
    --zero-thresh) ZERO_THRESH="${2:-}"; shift 2 ;;
    --wheel-stop-thresh) WHEEL_STOP_THRESH="${2:-}"; shift 2 ;;
    --imu-stop-thresh) IMU_STOP_THRESH="${2:-}"; shift 2 ;;
    --discovery-timeout-sec) DISCOVERY_TIMEOUT_SEC="${2:-}"; shift 2 ;;
    --label) LABEL="${2:-}"; shift 2 ;;
    --output-root) OUTPUT_ROOT="${2:-}"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "[cmd-vel-latency] unknown argument: $1" >&2; usage >&2; exit 2 ;;
  esac
done

STAMP="$(date -u +%Y%m%dT%H%M%SZ)"
REPORT_DIR="${OUTPUT_ROOT}/${STAMP}_${LABEL}"
mkdir -p "${REPORT_DIR}" || {
  echo "[cmd-vel-latency] FAIL cannot create output directory: ${REPORT_DIR}" >&2
  exit 1
}

cat >"${REPORT_DIR}/metadata.env" <<EOF
cmd_topic=${CMD_TOPIC}
linear_speed=${LINEAR_SPEED}
angular_speed=${ANGULAR_SPEED}
command_sec=${COMMAND_SEC}
settle_sec=${SETTLE_SEC}
sample_hz=${SAMPLE_HZ}
countdown_sec=${COUNTDOWN_SEC}
zero_thresh=${ZERO_THRESH}
wheel_stop_thresh=${WHEEL_STOP_THRESH}
imu_stop_thresh=${IMU_STOP_THRESH}
target_yaw_deg=${TARGET_YAW_DEG}
angle_tolerance_deg=${ANGLE_TOLERANCE_DEG}
discovery_timeout_sec=${DISCOVERY_TIMEOUT_SEC}
label=${LABEL}
EOF

if [[ "${COUNTDOWN_SEC}" != "0" ]]; then
  echo "[cmd-vel-latency] motion starts in ${COUNTDOWN_SEC}s. Ensure rotation clearance and E-stop access."
  remaining="${COUNTDOWN_SEC}"
  while [[ "${remaining}" -gt 0 ]]; do
    echo "[cmd-vel-latency] ${remaining}..."
    sleep 1
    remaining=$((remaining - 1))
  done
fi

python3 - "${REPORT_DIR}" "${CMD_TOPIC}" "${LINEAR_SPEED}" "${ANGULAR_SPEED}" "${COMMAND_SEC}" "${SETTLE_SEC}" \
  "${SAMPLE_HZ}" "${ZERO_THRESH}" "${WHEEL_STOP_THRESH}" "${TARGET_YAW_DEG}" "${ANGLE_TOLERANCE_DEG}" \
  "${DISCOVERY_TIMEOUT_SEC}" "${IMU_STOP_THRESH}" <<'PY'
import csv
import json
import math
import statistics
import sys
import threading
import time
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import String

try:
    from ranger_msgs.msg import MotionState
except Exception:
    MotionState = None

report_dir = Path(sys.argv[1])
cmd_topic = sys.argv[2]
linear_speed = float(sys.argv[3])
angular_speed = float(sys.argv[4])
command_sec = float(sys.argv[5])
settle_sec = float(sys.argv[6])
sample_hz = float(sys.argv[7])
zero_thresh = abs(float(sys.argv[8]))
wheel_stop_thresh = abs(float(sys.argv[9]))
target_yaw_arg = sys.argv[10].strip()
target_yaw_deg = float(target_yaw_arg) if target_yaw_arg else None
angle_tolerance_deg = abs(float(sys.argv[11]))
discovery_timeout_sec = max(0.5, float(sys.argv[12]))
imu_stop_thresh = abs(float(sys.argv[13]))

samples_path = report_dir / "samples.csv"
receive_events_path = report_dir / "receive_events.csv"
summary_path = report_dir / "summary.md"
metrics_path = report_dir / "metrics.json"

def twist_wz(msg: Optional[Twist]) -> Optional[float]:
    if msg is None:
        return None
    return float(msg.angular.z)

def twist_vx(msg: Optional[Twist]) -> Optional[float]:
    if msg is None:
        return None
    return float(msg.linear.x)

def odom_wz(msg: Optional[Odometry]) -> Optional[float]:
    if msg is None:
        return None
    return float(msg.twist.twist.angular.z)

def odom_vx(msg: Optional[Odometry]) -> Optional[float]:
    if msg is None:
        return None
    return float(msg.twist.twist.linear.x)

def odom_yaw(msg: Optional[Odometry]) -> Optional[float]:
    if msg is None:
        return None
    q = msg.pose.pose.orientation
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)

def angle_delta(a: float, b: float) -> float:
    return math.atan2(math.sin(a - b), math.cos(a - b))

class Probe(Node):
    def __init__(self) -> None:
        super().__init__("cmd_vel_stop_latency_probe")
        cmd_qos = QoSProfile(depth=1)
        odom_qos = QoSProfile(
            depth=1,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
        )
        self.pub = self.create_publisher(Twist, cmd_topic, cmd_qos)
        self.cmd_in: Optional[Twist] = None
        self.cmd_safe: Optional[Twist] = None
        self.cmd_out: Optional[Twist] = None
        self.wheel: Optional[Odometry] = None
        self.local: Optional[Odometry] = None
        self.motion_state = None
        self.imu: Optional[Imu] = None
        self.imu_count = 0
        self.imu_received_at = 0.0
        self.imu_integral_rad = 0.0
        self.imu_receive_integral_rad = 0.0
        self.imu_previous_wz: Optional[float] = None
        self.imu_previous_stamp_sec: Optional[float] = None
        self.imu_previous_received_at: Optional[float] = None
        self.imu_previous_receive_wz: Optional[float] = None
        self.imu_lock = threading.Lock()
        self.event_lock = threading.Lock()
        self.cmd_in_events: List[Dict[str, Any]] = []
        self.cmd_safe_events: List[Dict[str, Any]] = []
        self.cmd_out_events: List[Dict[str, Any]] = []
        self.wheel_events: List[Dict[str, Any]] = []
        self.local_events: List[Dict[str, Any]] = []
        self.motion_events: List[Dict[str, Any]] = []
        self.imu_events: List[Dict[str, Any]] = []
        self.raw_imu_events: List[Dict[str, Any]] = []
        self.raw_imu_count = 0
        self.safety_status = ""
        self.mode_status = ""
        self.wheel_count = 0
        self.wheel_received_at = 0.0
        self.cmd_safe_received_at = 0.0
        self.cmd_out_received_at = 0.0
        self.create_subscription(Twist, cmd_topic, self.on_cmd_in, cmd_qos)
        self.create_subscription(Twist, "/cmd_vel_safe", self.on_cmd_safe, cmd_qos)
        self.create_subscription(Twist, "/cmd_vel", self.on_cmd_out, cmd_qos)
        self.create_subscription(Odometry, "/wheel/odom", self.on_wheel, odom_qos)
        self.create_subscription(Odometry, "/local_state/odometry", self.on_local, odom_qos)
        self.create_subscription(Imu, "/lidar_imu", self.on_raw_imu, odom_qos)
        self.create_subscription(Imu, "/lidar_imu_bias_corrected", self.on_imu, odom_qos)
        self.create_subscription(String, "/safety/status", lambda m: setattr(self, "safety_status", m.data), 10)
        self.create_subscription(String, "/ranger_mini3_mode_controller/status", lambda m: setattr(self, "mode_status", m.data), 10)
        if MotionState is not None:
            self.create_subscription(MotionState, "/motion_state", self.on_motion_state, odom_qos)

    @staticmethod
    def command_event(msg: Twist, received_at: float) -> Dict[str, float]:
        return {
            "receive_time": received_at,
            "vx": float(msg.linear.x),
            "wz": float(msg.angular.z),
        }

    def on_cmd_in(self, msg: Twist) -> None:
        received_at = time.monotonic()
        self.cmd_in = msg
        with self.event_lock:
            self.cmd_in_events.append(self.command_event(msg, received_at))

    def on_cmd_safe(self, msg: Twist) -> None:
        received_at = time.monotonic()
        self.cmd_safe = msg
        self.cmd_safe_received_at = received_at
        with self.event_lock:
            self.cmd_safe_events.append(self.command_event(msg, received_at))

    def on_cmd_out(self, msg: Twist) -> None:
        received_at = time.monotonic()
        self.cmd_out = msg
        self.cmd_out_received_at = received_at
        with self.event_lock:
            self.cmd_out_events.append(self.command_event(msg, received_at))

    def on_wheel(self, msg: Odometry) -> None:
        received_at = time.monotonic()
        self.wheel = msg
        self.wheel_count += 1
        self.wheel_received_at = received_at
        stamp = msg.header.stamp
        with self.event_lock:
            self.wheel_events.append({
                "receive_time": received_at,
                "header_stamp_sec": float(stamp.sec) + float(stamp.nanosec) * 1.0e-9,
                "vx": odom_vx(msg),
                "wz": odom_wz(msg),
                "yaw": odom_yaw(msg),
            })

    def on_local(self, msg: Odometry) -> None:
        received_at = time.monotonic()
        self.local = msg
        stamp = msg.header.stamp
        with self.event_lock:
            self.local_events.append({
                "receive_time": received_at,
                "header_stamp_sec": float(stamp.sec) + float(stamp.nanosec) * 1.0e-9,
                "vx": odom_vx(msg),
                "wz": odom_wz(msg),
                "yaw": odom_yaw(msg),
            })

    def on_motion_state(self, msg) -> None:
        received_at = time.monotonic()
        self.motion_state = msg
        with self.event_lock:
            self.motion_events.append({
                "receive_time": received_at,
                "vx": float(getattr(msg, "linear_velocity", 0.0)),
                "wz": float(getattr(msg, "angular_velocity", 0.0)),
                "motion_mode": int(getattr(msg, "motion_mode", -1)),
            })

    def on_raw_imu(self, msg: Imu) -> None:
        received_at = time.monotonic()
        stamp = msg.header.stamp
        wx = float(msg.angular_velocity.x)
        wy = float(msg.angular_velocity.y)
        wz = float(msg.angular_velocity.z)
        self.raw_imu_count += 1
        with self.event_lock:
            self.raw_imu_events.append({
                "receive_time": received_at,
                "header_stamp_sec": float(stamp.sec) + float(stamp.nanosec) * 1.0e-9,
                "wx": wx,
                "wy": wy,
                "wz": wz,
                "angular_norm": math.sqrt(wx * wx + wy * wy + wz * wz),
            })

    def on_imu(self, msg: Imu) -> None:
        received_at = time.monotonic()
        stamp = msg.header.stamp
        stamp_sec = float(stamp.sec) + float(stamp.nanosec) * 1.0e-9
        if stamp_sec <= 0.0:
            stamp_sec = time.monotonic()
        wz = float(msg.angular_velocity.z)
        with self.imu_lock:
            if self.imu_previous_stamp_sec is not None and self.imu_previous_wz is not None:
                dt = stamp_sec - self.imu_previous_stamp_sec
                if (
                    0.0 < dt <= 0.10 and
                    math.isfinite(wz) and
                    math.isfinite(self.imu_previous_wz)
                ):
                    self.imu_integral_rad += 0.5 * (self.imu_previous_wz + wz) * dt
            if self.imu_previous_received_at is not None and self.imu_previous_receive_wz is not None:
                receive_dt = received_at - self.imu_previous_received_at
                if (
                    0.0 < receive_dt <= 0.10 and
                    math.isfinite(wz) and
                    math.isfinite(self.imu_previous_receive_wz)
                ):
                    self.imu_receive_integral_rad += (
                        0.5 * (self.imu_previous_receive_wz + wz) * receive_dt
                    )
            self.imu = msg
            self.imu_count += 1
            self.imu_received_at = received_at
            self.imu_previous_wz = wz
            self.imu_previous_stamp_sec = stamp_sec
            self.imu_previous_received_at = received_at
            self.imu_previous_receive_wz = wz
            self.imu_events.append({
                "receive_time": received_at,
                "header_stamp_sec": stamp_sec,
                "wz": wz,
                "header_integral_rad": self.imu_integral_rad,
                "receive_integral_rad": self.imu_receive_integral_rad,
            })

    def imu_snapshot(self):
        with self.imu_lock:
            wz = None if self.imu is None else float(self.imu.angular_velocity.z)
            return wz, self.imu_integral_rad

    def event_snapshots(self) -> Dict[str, List[Dict[str, Any]]]:
        with self.event_lock, self.imu_lock:
            return {
                "cmd_in": list(self.cmd_in_events),
                "cmd_safe": list(self.cmd_safe_events),
                "cmd_out": list(self.cmd_out_events),
                "wheel": list(self.wheel_events),
                "local": list(self.local_events),
                "motion": list(self.motion_events),
                "imu": list(self.imu_events),
                "raw_imu": list(self.raw_imu_events),
            }

    def publish_cmd(self, vx: float, wz: float) -> None:
        msg = Twist()
        msg.linear.x = float(vx)
        msg.angular.z = float(wz)
        self.pub.publish(msg)

def both_below_abs(row, first_key: str, second_key: str, threshold: float) -> bool:
    first = row.get(first_key)
    second = row.get(second_key)
    return (
        first is not None and
        second is not None and
        abs(first) <= threshold and
        abs(second) <= threshold
    )

def either_above_abs(row, first_key: str, second_key: str, threshold: float) -> bool:
    first = row.get(first_key)
    second = row.get(second_key)
    return (
        (first is not None and abs(first) > threshold) or
        (second is not None and abs(second) > threshold)
    )

def first_time_zero_twist(rows, vx_key: str, wz_key: str, stop_t: float, threshold: float) -> Optional[float]:
    for row in rows:
        if row["wall_time"] < stop_t:
            continue
        if both_below_abs(row, vx_key, wz_key, threshold):
            return row["wall_time"] - stop_t
    return None

def last_time_nonzero_twist(rows, vx_key: str, wz_key: str, stop_t: float, threshold: float) -> Optional[float]:
    last = None
    for row in rows:
        if row["wall_time"] < stop_t:
            continue
        if either_above_abs(row, vx_key, wz_key, threshold):
            last = row["wall_time"] - stop_t
    return last

def first_time_zero(rows, key: str, stop_t: float, threshold: float) -> Optional[float]:
    for row in rows:
        if row["wall_time"] < stop_t:
            continue
        value = row.get(key)
        if value is not None and abs(value) <= threshold:
            return row["wall_time"] - stop_t
    return None

def last_time_nonzero(rows, key: str, stop_t: float, threshold: float) -> Optional[float]:
    last = None
    for row in rows:
        if row["wall_time"] < stop_t:
            continue
        value = row.get(key)
        if value is not None and abs(value) > threshold:
            last = row["wall_time"] - stop_t
    return last

def first_time_below_abs(rows, key: str, stop_t: float, threshold: float) -> Optional[float]:
    return first_time_zero(rows, key, stop_t, threshold)

def command_receive_transition(
    events: List[Dict[str, Any]],
    after: float,
    threshold: float,
) -> Tuple[Optional[Dict[str, Any]], Optional[Dict[str, Any]]]:
    first_nonzero = None
    for event in events:
        if event["receive_time"] < after:
            continue
        nonzero = abs(float(event["vx"])) > threshold or abs(float(event["wz"])) > threshold
        if first_nonzero is None:
            if nonzero:
                first_nonzero = event
            continue
        if not nonzero:
            return first_nonzero, event
    return first_nonzero, None

def interpolate_event_value(
    events: List[Dict[str, Any]],
    key: str,
    target_time: float,
) -> Optional[float]:
    previous = None
    for event in events:
        value = event.get(key)
        if value is None or not math.isfinite(float(value)):
            continue
        if float(event["receive_time"]) >= target_time:
            if previous is None:
                return float(value)
            previous_time = float(previous["receive_time"])
            current_time = float(event["receive_time"])
            if current_time <= previous_time:
                return float(value)
            alpha = max(0.0, min(1.0, (target_time - previous_time) / (current_time - previous_time)))
            return float(previous[key]) + alpha * (float(value) - float(previous[key]))
        previous = event
    if previous is None:
        return None
    return float(previous[key])

def unwrap_yaw_events(events: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
    result: List[Dict[str, Any]] = []
    previous_yaw = None
    unwrapped_yaw = None
    for event in events:
        yaw = event.get("yaw")
        if yaw is None or not math.isfinite(float(yaw)):
            continue
        yaw = float(yaw)
        if previous_yaw is None:
            unwrapped_yaw = yaw
        else:
            unwrapped_yaw += angle_delta(yaw, previous_yaw)
        previous_yaw = yaw
        copied = dict(event)
        copied["unwrapped_yaw"] = unwrapped_yaw
        result.append(copied)
    return result

def prepare_raw_imu_motion_events(
    events: List[Dict[str, Any]],
    motion_start: Optional[float],
    direction: float,
) -> Tuple[List[Dict[str, Any]], Tuple[float, float, float]]:
    if motion_start is None:
        return [], (0.0, 0.0, 0.0)
    baseline = [
        event for event in events
        if motion_start - 0.75 <= float(event["receive_time"]) < motion_start
    ]
    if len(baseline) < 10:
        baseline = [event for event in events if float(event["receive_time"]) < motion_start][-200:]
    if baseline:
        bias = tuple(
            statistics.mean(float(event[key]) for event in baseline)
            for key in ("wx", "wy", "wz")
        )
    else:
        bias = (0.0, 0.0, 0.0)

    signed_direction = 1.0 if direction >= 0.0 else -1.0
    result: List[Dict[str, Any]] = []
    previous_time = None
    previous_rate = None
    integral = 0.0
    for event in events:
        wx = float(event["wx"]) - bias[0]
        wy = float(event["wy"]) - bias[1]
        wz = float(event["wz"]) - bias[2]
        motion_rate = signed_direction * math.sqrt(wx * wx + wy * wy + wz * wz)
        received_at = float(event["receive_time"])
        if previous_time is not None and previous_rate is not None:
            dt = received_at - previous_time
            if 0.0 < dt <= 0.10:
                integral += 0.5 * (previous_rate + motion_rate) * dt
        copied = dict(event)
        copied["motion_rate"] = motion_rate
        copied["receive_integral_rad"] = integral
        result.append(copied)
        previous_time = received_at
        previous_rate = motion_rate
    return result, bias

def first_above_threshold_time(
    events: List[Dict[str, Any]],
    key: str,
    after: Optional[float],
    threshold: float,
) -> Optional[float]:
    if after is None:
        return None
    for event in events:
        if float(event["receive_time"]) < after:
            continue
        value = event.get(key)
        if value is not None and math.isfinite(float(value)) and abs(float(value)) > threshold:
            return float(event["receive_time"])
    return None

def sustained_stop_window(
    events: List[Dict[str, Any]],
    key: str,
    after: float,
    threshold: float,
    hold_sec: float,
) -> Tuple[Optional[float], Optional[float]]:
    candidate = None
    for event in events:
        received_at = float(event["receive_time"])
        if received_at < after:
            continue
        value = event.get(key)
        if value is None or not math.isfinite(float(value)):
            candidate = None
            continue
        if abs(float(value)) <= threshold:
            if candidate is None:
                candidate = received_at
            if received_at - candidate >= hold_sec:
                return candidate, received_at
        else:
            candidate = None
    return None, None

def message_stamp_age_sec(node: Node, msg) -> Optional[float]:
    if msg is None or not hasattr(msg, "header"):
        return None
    stamp = msg.header.stamp
    stamp_ns = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
    if stamp_ns <= 0:
        return None
    return (node.get_clock().now().nanoseconds - stamp_ns) * 1.0e-9

rclpy.init()
node = Probe()
executor = SingleThreadedExecutor()
executor.add_node(node)
executor_thread = threading.Thread(target=executor.spin, daemon=True)
executor_thread.start()
sample_period = 1.0 / max(sample_hz, 1.0)
publish_period = 1.0 / 50.0
rows = []
start = 0.0
stop_t = None
first_zero_publish_t = None
start_wheel_yaw = None
target_start_wheel_yaw = None
stop_wheel_yaw = None
end_wheel_yaw = None
start_imu_yaw = None
stop_imu_yaw = None
end_imu_yaw = None
discovery_wait_sec = None
event_snapshots: Dict[str, List[Dict[str, Any]]] = {}

def append_sample(phase: str, requested_vx: float, requested_wz: float, wall: float) -> None:
    global start_wheel_yaw, end_wheel_yaw
    wheel_yaw = odom_yaw(node.wheel)
    if start_wheel_yaw is None and wheel_yaw is not None:
        start_wheel_yaw = wheel_yaw
    if wheel_yaw is not None:
        end_wheel_yaw = wheel_yaw
    imu_wz, imu_yaw = node.imu_snapshot()
    motion_wz = None
    motion_mode = None
    if node.motion_state is not None:
        raw_motion_wz = getattr(node.motion_state, "angular_velocity", None)
        motion_wz = None if raw_motion_wz is None else float(raw_motion_wz)
        motion_mode = int(getattr(node.motion_state, "motion_mode", -1))
    rows.append({
        "wall_time": wall,
        "phase": phase,
        "elapsed_sec": wall - start,
        "since_stop_sec": "" if stop_t is None else wall - stop_t,
        "requested_vx": requested_vx,
        "requested_wz": requested_wz,
        "cmd_in_vx": twist_vx(node.cmd_in),
        "cmd_in_wz": twist_wz(node.cmd_in),
        "cmd_safe_vx": twist_vx(node.cmd_safe),
        "cmd_safe_wz": twist_wz(node.cmd_safe),
        "cmd_out_vx": twist_vx(node.cmd_out),
        "cmd_out_wz": twist_wz(node.cmd_out),
        "wheel_vx": odom_vx(node.wheel),
        "wheel_wz": odom_wz(node.wheel),
        "wheel_yaw_accum_deg": None if start_wheel_yaw is None or wheel_yaw is None else math.degrees(angle_delta(wheel_yaw, start_wheel_yaw)),
        "wheel_stamp_age_sec": message_stamp_age_sec(node, node.wheel),
        "local_vx": odom_vx(node.local),
        "local_wz": odom_wz(node.local),
        "imu_wz": imu_wz,
        "imu_yaw_accum_deg": None if start_imu_yaw is None else math.degrees(imu_yaw - start_imu_yaw),
        "motion_wz": motion_wz,
        "motion_mode": motion_mode,
        "safety_status": node.safety_status,
        "mode_status": node.mode_status,
    })

try:
    # A fresh Fast DDS participant can need several seconds to discover the
    # command chain. Never start the motion clock until both state feedback,
    # the robot_safety input, and both downstream command publishers are
    # matched to this probe.
    def command_chain_discovered() -> bool:
        return (
            node.wheel_count >= 3 and
            node.imu_count >= 3 and
            node.raw_imu_count >= 3 and
            node.pub.get_subscription_count() >= 2 and
            node.count_publishers("/cmd_vel_safe") >= 1 and
            node.count_publishers("/cmd_vel") >= 1
        )

    discovery_started = time.monotonic()
    discovery_end = discovery_started + discovery_timeout_sec
    while time.monotonic() < discovery_end and rclpy.ok():
        time.sleep(0.02)
        if command_chain_discovered():
            break
    discovery_wait_sec = time.monotonic() - discovery_started
    if not command_chain_discovered():
        raise RuntimeError(
            "command chain discovery incomplete before timeout; refusing to move "
            f"wheel_samples={node.wheel_count} "
            f"imu_samples={node.imu_count} "
            f"raw_imu_samples={node.raw_imu_count} "
            f"command_subscriptions={node.pub.get_subscription_count()} "
            f"cmd_safe_publishers={node.count_publishers('/cmd_vel_safe')} "
            f"cmd_out_publishers={node.count_publishers('/cmd_vel')}"
        )

    # Prove that a zero command traverses robot_safety and reaches the final
    # command topic. Then leave a quiet interval longer than the production
    # zero-command priority burst before requesting motion.
    handshake_started = time.monotonic()
    handshake_end = handshake_started + 1.50
    next_publish = handshake_started
    while time.monotonic() < handshake_end and rclpy.ok():
        now = time.monotonic()
        if now >= next_publish:
            node.publish_cmd(0.0, 0.0)
            next_publish += publish_period
        time.sleep(0.002)
    if (
        node.cmd_safe_received_at < handshake_started or
        node.cmd_out_received_at < handshake_started or
        not both_below_abs(
            {"vx": twist_vx(node.cmd_safe), "wz": twist_wz(node.cmd_safe)},
            "vx", "wz", zero_thresh,
        ) or
        not both_below_abs(
            {"vx": twist_vx(node.cmd_out), "wz": twist_wz(node.cmd_out)},
            "vx", "wz", zero_thresh,
        )
    ):
        raise RuntimeError(
            "zero-command handshake did not reach /cmd_vel_safe and /cmd_vel; refusing to move "
            f"safe_rx_age={handshake_started - node.cmd_safe_received_at:.3f} "
            f"out_rx_age={handshake_started - node.cmd_out_received_at:.3f} "
            f"safe_vx={twist_vx(node.cmd_safe)} safe_wz={twist_wz(node.cmd_safe)} "
            f"out_vx={twist_vx(node.cmd_out)} out_wz={twist_wz(node.cmd_out)}"
        )
    quiet_end = time.monotonic() + 0.35
    while time.monotonic() < quiet_end and rclpy.ok():
        time.sleep(0.01)

    start = time.monotonic()
    start_wheel_yaw = odom_yaw(node.wheel)
    _, start_imu_yaw = node.imu_snapshot()

    next_sample = time.monotonic()
    next_publish = next_sample
    command_end = time.monotonic() + command_sec
    target_reached = False
    target_stop_reason = "duration_elapsed"
    if target_yaw_deg is not None:
        wait_end = time.monotonic() + 1.0
        while node.wheel is None and time.monotonic() < wait_end and rclpy.ok():
            time.sleep(0.02)
        target_start_wheel_yaw = odom_yaw(node.wheel)
        if target_start_wheel_yaw is None:
            target_stop_reason = "duration_elapsed_no_initial_target_reference"
    while time.monotonic() < command_end and rclpy.ok():
        now = time.monotonic()
        if now >= next_publish:
            node.publish_cmd(linear_speed, angular_speed)
            next_publish += publish_period
        if target_yaw_deg is not None and target_start_wheel_yaw is None and node.wheel is not None:
            target_start_wheel_yaw = odom_yaw(node.wheel)
        if now >= next_sample:
            append_sample("command", linear_speed, angular_speed, now)
            next_sample += sample_period
        if target_yaw_deg is not None and target_start_wheel_yaw is not None and node.wheel is not None:
            current_yaw = odom_yaw(node.wheel)
            if current_yaw is not None:
                accum_deg = math.degrees(angle_delta(current_yaw, target_start_wheel_yaw))
                target_sign = 1.0 if target_yaw_deg >= 0.0 else -1.0
                if target_sign * accum_deg >= abs(target_yaw_deg) - angle_tolerance_deg:
                    target_reached = True
                    target_stop_reason = "target_yaw_reached"
                    break
        time.sleep(0.002)

    stop_wheel_yaw = odom_yaw(node.wheel)
    _, stop_imu_yaw = node.imu_snapshot()
    stop_t = time.monotonic()
    settle_end = stop_t + settle_sec
    next_sample = stop_t
    next_publish = stop_t
    while time.monotonic() < settle_end and rclpy.ok():
        now = time.monotonic()
        if now >= next_publish:
            if first_zero_publish_t is None:
                first_zero_publish_t = now
            node.publish_cmd(0.0, 0.0)
            next_publish += publish_period
        if now >= next_sample:
            append_sample("settle", 0.0, 0.0, now)
            next_sample += sample_period
        time.sleep(0.002)
    _, end_imu_yaw = node.imu_snapshot()
finally:
    try:
        zero_end = time.monotonic() + 1.0
        next_publish = time.monotonic()
        while time.monotonic() < zero_end and rclpy.ok():
            now = time.monotonic()
            if now >= next_publish:
                node.publish_cmd(0.0, 0.0)
                next_publish += publish_period
            time.sleep(0.002)
    finally:
        event_snapshots = node.event_snapshots()
        executor.shutdown(timeout_sec=2.0)
        executor_thread.join(timeout=2.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

fieldnames = [
    "phase",
    "elapsed_sec",
    "since_stop_sec",
    "requested_vx",
    "requested_wz",
    "cmd_in_vx",
    "cmd_in_wz",
    "cmd_safe_vx",
    "cmd_safe_wz",
    "cmd_out_vx",
    "cmd_out_wz",
    "wheel_vx",
    "wheel_wz",
    "wheel_yaw_accum_deg",
    "wheel_stamp_age_sec",
    "local_vx",
    "local_wz",
    "imu_wz",
    "imu_yaw_accum_deg",
    "motion_wz",
    "motion_mode",
    "safety_status",
    "mode_status",
]
with samples_path.open("w", newline="", encoding="utf-8") as f:
    writer = csv.DictWriter(f, fieldnames=fieldnames)
    writer.writeheader()
    for row in rows:
        writer.writerow({
            key: "" if row.get(key) is None else row.get(key)
            for key in fieldnames
        })

cmd_in_start_event, cmd_in_zero_event = command_receive_transition(
    event_snapshots.get("cmd_in", []), start, zero_thresh
)
cmd_safe_start_event, cmd_safe_zero_event = command_receive_transition(
    event_snapshots.get("cmd_safe", []), start, zero_thresh
)
cmd_out_start_event, cmd_out_zero_event = command_receive_transition(
    event_snapshots.get("cmd_out", []), start, zero_thresh
)

cmd_out_start_receive_t = None if cmd_out_start_event is None else float(cmd_out_start_event["receive_time"])
cmd_out_zero_receive_t = None if cmd_out_zero_event is None else float(cmd_out_zero_event["receive_time"])
raw_imu_motion_events, raw_imu_bias = prepare_raw_imu_motion_events(
    event_snapshots.get("raw_imu", []), cmd_out_start_receive_t, angular_speed
)
event_snapshots["raw_imu"] = raw_imu_motion_events
stop_hold_sec = 0.30
wheel_stop_start_t = None
wheel_stop_confirm_t = None
imu_stop_start_t = None
imu_stop_confirm_t = None
motion_stop_start_t = None
motion_stop_confirm_t = None
raw_imu_stop_start_t = None
raw_imu_stop_confirm_t = None
physical_stop_t = None
physical_stop_source = "unavailable"

if cmd_out_zero_receive_t is not None:
    wheel_stop_start_t, wheel_stop_confirm_t = sustained_stop_window(
        event_snapshots.get("wheel", []),
        "wz",
        cmd_out_zero_receive_t,
        wheel_stop_thresh,
        stop_hold_sec,
    )
    imu_stop_start_t, imu_stop_confirm_t = sustained_stop_window(
        event_snapshots.get("imu", []),
        "wz",
        cmd_out_zero_receive_t,
        imu_stop_thresh,
        stop_hold_sec,
    )
    motion_stop_start_t, motion_stop_confirm_t = sustained_stop_window(
        event_snapshots.get("motion", []),
        "wz",
        cmd_out_zero_receive_t,
        wheel_stop_thresh,
        stop_hold_sec,
    )
    raw_imu_stop_start_t, raw_imu_stop_confirm_t = sustained_stop_window(
        raw_imu_motion_events,
        "motion_rate",
        cmd_out_zero_receive_t,
        imu_stop_thresh,
        stop_hold_sec,
    )
    if raw_imu_stop_start_t is not None:
        physical_stop_t = raw_imu_stop_start_t
        physical_stop_source = "raw_imu_norm_sustained_stop"
    elif imu_stop_start_t is not None:
        physical_stop_t = imu_stop_start_t
        physical_stop_source = "corrected_imu_sustained_stop_fallback"
    elif wheel_stop_start_t is not None:
        physical_stop_t = wheel_stop_start_t
        physical_stop_source = "wheel_sustained_stop_fallback"

wheel_unwrapped_events = unwrap_yaw_events(event_snapshots.get("wheel", []))
local_unwrapped_events = unwrap_yaw_events(event_snapshots.get("local", []))
imu_events = event_snapshots.get("imu", [])

def receive_time_delta(event, reference: Optional[float]) -> Optional[float]:
    if event is None or reference is None:
        return None
    return float(event["receive_time"]) - reference

def interval_delta_deg(
    events: List[Dict[str, Any]],
    key: str,
    interval_start: Optional[float],
    interval_end: Optional[float],
) -> Optional[float]:
    if interval_start is None or interval_end is None:
        return None
    first = interpolate_event_value(events, key, interval_start)
    last = interpolate_event_value(events, key, interval_end)
    if first is None or last is None:
        return None
    return math.degrees(last - first)

wheel_pre_zero_deg = interval_delta_deg(
    wheel_unwrapped_events, "unwrapped_yaw", cmd_out_start_receive_t, cmd_out_zero_receive_t
)
wheel_post_zero_tail_deg = interval_delta_deg(
    wheel_unwrapped_events, "unwrapped_yaw", cmd_out_zero_receive_t, physical_stop_t
)
wheel_receive_total_deg = interval_delta_deg(
    wheel_unwrapped_events, "unwrapped_yaw", cmd_out_start_receive_t, physical_stop_t
)
local_pre_zero_deg = interval_delta_deg(
    local_unwrapped_events, "unwrapped_yaw", cmd_out_start_receive_t, cmd_out_zero_receive_t
)
local_post_zero_tail_deg = interval_delta_deg(
    local_unwrapped_events, "unwrapped_yaw", cmd_out_zero_receive_t, physical_stop_t
)
local_receive_total_deg = interval_delta_deg(
    local_unwrapped_events, "unwrapped_yaw", cmd_out_start_receive_t, physical_stop_t
)
imu_pre_zero_deg = interval_delta_deg(
    imu_events, "receive_integral_rad", cmd_out_start_receive_t, cmd_out_zero_receive_t
)
imu_post_zero_tail_deg = interval_delta_deg(
    imu_events, "receive_integral_rad", cmd_out_zero_receive_t, physical_stop_t
)
imu_receive_total_deg = interval_delta_deg(
    imu_events, "receive_integral_rad", cmd_out_start_receive_t, physical_stop_t
)
raw_imu_pre_zero_deg = interval_delta_deg(
    raw_imu_motion_events, "receive_integral_rad", cmd_out_start_receive_t, cmd_out_zero_receive_t
)
raw_imu_post_zero_tail_deg = interval_delta_deg(
    raw_imu_motion_events, "receive_integral_rad", cmd_out_zero_receive_t, physical_stop_t
)
raw_imu_receive_total_deg = interval_delta_deg(
    raw_imu_motion_events, "receive_integral_rad", cmd_out_start_receive_t, physical_stop_t
)

wheel_motion_start_t = first_above_threshold_time(
    event_snapshots.get("wheel", []), "wz", cmd_out_start_receive_t, wheel_stop_thresh
)
imu_motion_start_t = first_above_threshold_time(
    imu_events, "wz", cmd_out_start_receive_t, imu_stop_thresh
)
raw_imu_motion_start_t = first_above_threshold_time(
    raw_imu_motion_events, "motion_rate", cmd_out_start_receive_t, imu_stop_thresh
)

def difference(first: Optional[float], second: Optional[float]) -> Optional[float]:
    if first is None or second is None:
        return None
    return first - second

receive_event_rows: List[Dict[str, Any]] = []
for source, events in event_snapshots.items():
    for event in events:
        received_at = float(event["receive_time"])
        receive_event_rows.append({
            "source": source,
            "elapsed_sec": received_at - start,
            "since_cmd_out_zero_sec": "" if cmd_out_zero_receive_t is None else received_at - cmd_out_zero_receive_t,
            "header_stamp_sec": event.get("header_stamp_sec"),
            "vx": event.get("vx"),
            "wz": event.get("wz"),
            "wx": event.get("wx"),
            "wy": event.get("wy"),
            "yaw": event.get("yaw"),
            "angular_norm": event.get("angular_norm"),
            "motion_rate": event.get("motion_rate"),
            "motion_mode": event.get("motion_mode"),
            "imu_header_integral_rad": event.get("header_integral_rad"),
            "imu_receive_integral_rad": event.get("receive_integral_rad"),
        })
receive_event_rows.sort(key=lambda row: float(row["elapsed_sec"]))
receive_event_fieldnames = [
    "source",
    "elapsed_sec",
    "since_cmd_out_zero_sec",
    "header_stamp_sec",
    "vx",
    "wz",
    "wx",
    "wy",
    "yaw",
    "angular_norm",
    "motion_rate",
    "motion_mode",
    "imu_header_integral_rad",
    "imu_receive_integral_rad",
]
with receive_events_path.open("w", newline="", encoding="utf-8") as f:
    writer = csv.DictWriter(f, fieldnames=receive_event_fieldnames)
    writer.writeheader()
    for row in receive_event_rows:
        writer.writerow({
            key: "" if row.get(key) is None else row.get(key)
            for key in receive_event_fieldnames
        })

metrics: Dict[str, Any] = {
    "cmd_topic": cmd_topic,
    "linear_speed_mps": linear_speed,
    "angular_speed_radps": angular_speed,
    "command_sec": command_sec,
    "settle_sec": settle_sec,
    "zero_thresh_radps": zero_thresh,
    "wheel_stop_thresh_radps": wheel_stop_thresh,
    "imu_stop_thresh_radps": imu_stop_thresh,
    "target_yaw_deg": target_yaw_deg,
    "angle_tolerance_deg": angle_tolerance_deg,
    "target_reached": target_reached,
    "target_stop_reason": target_stop_reason,
    "discovery_timeout_sec": discovery_timeout_sec,
    "discovery_wait_sec": discovery_wait_sec,
    "cmd_in_first_zero_sec": first_time_zero_twist(rows, "cmd_in_vx", "cmd_in_wz", stop_t, zero_thresh),
    "cmd_safe_first_zero_sec": first_time_zero_twist(rows, "cmd_safe_vx", "cmd_safe_wz", stop_t, zero_thresh),
    "cmd_out_first_zero_sec": first_time_zero_twist(rows, "cmd_out_vx", "cmd_out_wz", stop_t, zero_thresh),
    "wheel_first_stop_sec": first_time_zero_twist(rows, "wheel_vx", "wheel_wz", stop_t, wheel_stop_thresh),
    "motion_first_stop_sec": first_time_below_abs(rows, "motion_wz", stop_t, wheel_stop_thresh),
    "cmd_in_last_nonzero_sec": last_time_nonzero_twist(rows, "cmd_in_vx", "cmd_in_wz", stop_t, zero_thresh),
    "cmd_safe_last_nonzero_sec": last_time_nonzero_twist(rows, "cmd_safe_vx", "cmd_safe_wz", stop_t, zero_thresh),
    "cmd_out_last_nonzero_sec": last_time_nonzero_twist(rows, "cmd_out_vx", "cmd_out_wz", stop_t, zero_thresh),
    "wheel_last_moving_sec": last_time_nonzero_twist(rows, "wheel_vx", "wheel_wz", stop_t, wheel_stop_thresh),
    "imu_first_stop_sec": first_time_zero(rows, "imu_wz", stop_t, imu_stop_thresh),
    "imu_last_moving_sec": last_time_nonzero(rows, "imu_wz", stop_t, imu_stop_thresh),
    "receive_time_split_source": "/cmd_vel first zero callback after nonzero",
    "receive_time_stop_hold_sec": stop_hold_sec,
    "first_zero_publish_to_cmd_in_receive_sec": receive_time_delta(cmd_in_zero_event, first_zero_publish_t),
    "first_zero_publish_to_cmd_safe_receive_sec": receive_time_delta(cmd_safe_zero_event, first_zero_publish_t),
    "first_zero_publish_to_cmd_out_receive_sec": receive_time_delta(cmd_out_zero_event, first_zero_publish_t),
    "cmd_in_to_cmd_safe_zero_receive_sec": None if cmd_in_zero_event is None else receive_time_delta(cmd_safe_zero_event, float(cmd_in_zero_event["receive_time"])),
    "cmd_safe_to_cmd_out_zero_receive_sec": None if cmd_safe_zero_event is None else receive_time_delta(cmd_out_zero_event, float(cmd_safe_zero_event["receive_time"])),
    "cmd_out_zero_to_wheel_stop_start_sec": None if cmd_out_zero_receive_t is None or wheel_stop_start_t is None else wheel_stop_start_t - cmd_out_zero_receive_t,
    "cmd_out_zero_to_wheel_stop_confirm_sec": None if cmd_out_zero_receive_t is None or wheel_stop_confirm_t is None else wheel_stop_confirm_t - cmd_out_zero_receive_t,
    "cmd_out_zero_to_imu_stop_start_sec": None if cmd_out_zero_receive_t is None or imu_stop_start_t is None else imu_stop_start_t - cmd_out_zero_receive_t,
    "cmd_out_zero_to_imu_stop_confirm_sec": None if cmd_out_zero_receive_t is None or imu_stop_confirm_t is None else imu_stop_confirm_t - cmd_out_zero_receive_t,
    "cmd_out_zero_to_raw_imu_stop_start_sec": None if cmd_out_zero_receive_t is None or raw_imu_stop_start_t is None else raw_imu_stop_start_t - cmd_out_zero_receive_t,
    "cmd_out_zero_to_raw_imu_stop_confirm_sec": None if cmd_out_zero_receive_t is None or raw_imu_stop_confirm_t is None else raw_imu_stop_confirm_t - cmd_out_zero_receive_t,
    "cmd_out_zero_to_motion_stop_start_sec": None if cmd_out_zero_receive_t is None or motion_stop_start_t is None else motion_stop_start_t - cmd_out_zero_receive_t,
    "cmd_out_zero_to_physical_stop_sec": None if cmd_out_zero_receive_t is None or physical_stop_t is None else physical_stop_t - cmd_out_zero_receive_t,
    "physical_stop_source": physical_stop_source,
    "cmd_out_nonzero_to_wheel_motion_start_sec": None if cmd_out_start_receive_t is None or wheel_motion_start_t is None else wheel_motion_start_t - cmd_out_start_receive_t,
    "cmd_out_nonzero_to_corrected_imu_motion_start_sec": None if cmd_out_start_receive_t is None or imu_motion_start_t is None else imu_motion_start_t - cmd_out_start_receive_t,
    "cmd_out_nonzero_to_raw_imu_motion_start_sec": None if cmd_out_start_receive_t is None or raw_imu_motion_start_t is None else raw_imu_motion_start_t - cmd_out_start_receive_t,
    "raw_imu_bias_wx_radps": raw_imu_bias[0],
    "raw_imu_bias_wy_radps": raw_imu_bias[1],
    "raw_imu_bias_wz_radps": raw_imu_bias[2],
    "wheel_pre_zero_deg_receive_time": wheel_pre_zero_deg,
    "imu_pre_zero_deg_receive_time": imu_pre_zero_deg,
    "raw_imu_pre_zero_deg_receive_time": raw_imu_pre_zero_deg,
    "local_pre_zero_deg_receive_time": local_pre_zero_deg,
    "wheel_minus_imu_pre_zero_deg": difference(wheel_pre_zero_deg, imu_pre_zero_deg),
    "wheel_minus_raw_imu_pre_zero_deg": difference(wheel_pre_zero_deg, raw_imu_pre_zero_deg),
    "corrected_minus_raw_imu_pre_zero_deg": difference(imu_pre_zero_deg, raw_imu_pre_zero_deg),
    "local_minus_imu_pre_zero_deg": difference(local_pre_zero_deg, imu_pre_zero_deg),
    "wheel_post_zero_tail_deg_receive_time": wheel_post_zero_tail_deg,
    "imu_post_zero_tail_deg_receive_time": imu_post_zero_tail_deg,
    "raw_imu_post_zero_tail_deg_receive_time": raw_imu_post_zero_tail_deg,
    "local_post_zero_tail_deg_receive_time": local_post_zero_tail_deg,
    "wheel_minus_imu_post_zero_tail_deg": difference(wheel_post_zero_tail_deg, imu_post_zero_tail_deg),
    "wheel_minus_raw_imu_post_zero_tail_deg": difference(wheel_post_zero_tail_deg, raw_imu_post_zero_tail_deg),
    "corrected_minus_raw_imu_post_zero_tail_deg": difference(imu_post_zero_tail_deg, raw_imu_post_zero_tail_deg),
    "local_minus_imu_post_zero_tail_deg": difference(local_post_zero_tail_deg, imu_post_zero_tail_deg),
    "wheel_total_deg_receive_time": wheel_receive_total_deg,
    "imu_total_deg_receive_time": imu_receive_total_deg,
    "raw_imu_total_deg_receive_time": raw_imu_receive_total_deg,
    "local_total_deg_receive_time": local_receive_total_deg,
    "wheel_minus_imu_total_deg_receive_time": difference(wheel_receive_total_deg, imu_receive_total_deg),
    "wheel_minus_raw_imu_total_deg_receive_time": difference(wheel_receive_total_deg, raw_imu_receive_total_deg),
    "corrected_minus_raw_imu_total_deg_receive_time": difference(imu_receive_total_deg, raw_imu_receive_total_deg),
    "local_minus_imu_total_deg_receive_time": difference(local_receive_total_deg, imu_receive_total_deg),
}

wheel_stamp_ages = [
    row["wheel_stamp_age_sec"] for row in rows
    if row.get("wheel_stamp_age_sec") is not None
]
metrics["wheel_stamp_age_max_sec"] = max(wheel_stamp_ages) if wheel_stamp_ages else None
metrics["wheel_stamp_age_median_sec"] = statistics.median(wheel_stamp_ages) if wheel_stamp_ages else None

if start_wheel_yaw is not None and end_wheel_yaw is not None:
    metrics["wheel_yaw_delta_deg"] = math.degrees(angle_delta(end_wheel_yaw, start_wheel_yaw))
else:
    metrics["wheel_yaw_delta_deg"] = None
if start_wheel_yaw is not None and stop_wheel_yaw is not None:
    metrics["wheel_yaw_before_stop_deg"] = math.degrees(
        angle_delta(stop_wheel_yaw, start_wheel_yaw)
    )
else:
    metrics["wheel_yaw_before_stop_deg"] = None
if stop_wheel_yaw is not None and end_wheel_yaw is not None:
    metrics["wheel_yaw_tail_deg"] = math.degrees(
        angle_delta(end_wheel_yaw, stop_wheel_yaw)
    )
else:
    metrics["wheel_yaw_tail_deg"] = None
if start_imu_yaw is not None and stop_imu_yaw is not None:
    metrics["imu_yaw_before_stop_deg"] = math.degrees(stop_imu_yaw - start_imu_yaw)
else:
    metrics["imu_yaw_before_stop_deg"] = None
if stop_imu_yaw is not None and end_imu_yaw is not None:
    metrics["imu_yaw_tail_deg"] = math.degrees(end_imu_yaw - stop_imu_yaw)
else:
    metrics["imu_yaw_tail_deg"] = None
if start_imu_yaw is not None and end_imu_yaw is not None:
    metrics["imu_yaw_total_deg"] = math.degrees(end_imu_yaw - start_imu_yaw)
else:
    metrics["imu_yaw_total_deg"] = None
if metrics["wheel_yaw_tail_deg"] is not None and metrics["imu_yaw_tail_deg"] is not None:
    metrics["wheel_minus_imu_tail_deg"] = (
        metrics["wheel_yaw_tail_deg"] - metrics["imu_yaw_tail_deg"]
    )
else:
    metrics["wheel_minus_imu_tail_deg"] = None

for source_name, pre_zero_value, total_value in (
    ("wheel", wheel_pre_zero_deg, wheel_receive_total_deg),
    ("imu", imu_pre_zero_deg, imu_receive_total_deg),
    ("raw_imu", raw_imu_pre_zero_deg, raw_imu_receive_total_deg),
    ("local", local_pre_zero_deg, local_receive_total_deg),
):
    metrics[f"{source_name}_pre_zero_target_error_deg"] = (
        None if target_yaw_deg is None or pre_zero_value is None else pre_zero_value - target_yaw_deg
    )
    metrics[f"{source_name}_total_target_error_deg"] = (
        None if target_yaw_deg is None or total_value is None else total_value - target_yaw_deg
    )

metrics["receive_time_reference_source"] = "corrected_imu_base_link_yaw_rate"
metrics["raw_imu_role"] = "three_axis_norm_for_motion_boundaries_only"
pre_zero_mismatch = metrics["wheel_minus_imu_pre_zero_deg"]
post_zero_mismatch = metrics["wheel_minus_imu_post_zero_tail_deg"]
if pre_zero_mismatch is None or post_zero_mismatch is None:
    metrics["receive_time_dominant_mismatch_segment"] = "unavailable"
elif abs(pre_zero_mismatch) > 1.25 * abs(post_zero_mismatch):
    metrics["receive_time_dominant_mismatch_segment"] = "before_cmd_out_zero"
elif abs(post_zero_mismatch) > 1.25 * abs(pre_zero_mismatch):
    metrics["receive_time_dominant_mismatch_segment"] = "after_cmd_out_zero_tail"
else:
    metrics["receive_time_dominant_mismatch_segment"] = "mixed_or_similar"

with metrics_path.open("w", encoding="utf-8") as f:
    json.dump(metrics, f, indent=2, sort_keys=True)

def fmt(value) -> str:
    if value is None:
        return "n/a"
    if isinstance(value, float):
        return f"{value:.3f}"
    return str(value)

lines = [
    "# Cmd Vel Stop Latency Summary",
    "",
    f"- report_dir: `{report_dir}`",
    f"- cmd_topic: `{cmd_topic}`",
    f"- linear_speed_mps: `{linear_speed}`",
    f"- angular_speed_radps: `{angular_speed}`",
    f"- command_sec: `{command_sec}`",
    f"- target_yaw_deg: `{target_yaw_deg}`",
    f"- target_stop_reason: `{target_stop_reason}`",
    f"- discovery_wait_sec: `{fmt(metrics['discovery_wait_sec'])}`",
    f"- settle_sec: `{settle_sec}`",
    f"- zero_thresh_radps: `{zero_thresh}`",
    f"- wheel_stop_thresh_radps: `{wheel_stop_thresh}`",
    f"- imu_stop_thresh_radps: `{imu_stop_thresh}`",
    f"- wheel_stamp_age_max_sec: `{fmt(metrics['wheel_stamp_age_max_sec'])}`",
    "",
    "## Receive-Time Split (Primary)",
    "",
    f"- split: `{metrics['receive_time_split_source']}`",
    f"- physical_stop_source: `{metrics['physical_stop_source']}`",
    f"- yaw_reference_source: `{metrics['receive_time_reference_source']}`",
    f"- raw_imu_role: `{metrics['raw_imu_role']}`",
    f"- sustained_stop_hold_sec: `{fmt(metrics['receive_time_stop_hold_sec'])}`",
    f"- dominant_wheel_reference_mismatch_segment: `{metrics['receive_time_dominant_mismatch_segment']}`",
    "",
    "### Zero Command Propagation",
    "",
    "| boundary | latency_sec |",
    "|---|---:|",
    f"| first zero publish -> `{cmd_topic}` receive | {fmt(metrics['first_zero_publish_to_cmd_in_receive_sec'])} |",
    f"| first zero publish -> `/cmd_vel_safe` receive | {fmt(metrics['first_zero_publish_to_cmd_safe_receive_sec'])} |",
    f"| first zero publish -> `/cmd_vel` receive | {fmt(metrics['first_zero_publish_to_cmd_out_receive_sec'])} |",
    f"| `{cmd_topic}` zero receive -> `/cmd_vel_safe` zero receive | {fmt(metrics['cmd_in_to_cmd_safe_zero_receive_sec'])} |",
    f"| `/cmd_vel_safe` zero receive -> `/cmd_vel` zero receive | {fmt(metrics['cmd_safe_to_cmd_out_zero_receive_sec'])} |",
    f"| `/cmd_vel` zero receive -> wheel sustained-stop onset | {fmt(metrics['cmd_out_zero_to_wheel_stop_start_sec'])} |",
    f"| `/cmd_vel` zero receive -> IMU sustained-stop onset | {fmt(metrics['cmd_out_zero_to_imu_stop_start_sec'])} |",
    f"| `/cmd_vel` zero receive -> raw IMU sustained-stop onset | {fmt(metrics['cmd_out_zero_to_raw_imu_stop_start_sec'])} |",
    f"| `/cmd_vel` nonzero receive -> wheel motion onset | {fmt(metrics['cmd_out_nonzero_to_wheel_motion_start_sec'])} |",
    f"| `/cmd_vel` nonzero receive -> corrected IMU motion onset | {fmt(metrics['cmd_out_nonzero_to_corrected_imu_motion_start_sec'])} |",
    f"| `/cmd_vel` nonzero receive -> raw IMU motion onset | {fmt(metrics['cmd_out_nonzero_to_raw_imu_motion_start_sec'])} |",
    "",
    "### Yaw Error Split at Final `/cmd_vel` Zero Receive",
    "",
    "Raw IMU three-axis norm is used only to confirm motion onset/settle; its integral is not a yaw angle.",
    "",
    "| source | before_zero_deg | after_zero_tail_deg | total_to_physical_stop_deg | target_error_before_zero_deg | target_error_total_deg |",
    "|---|---:|---:|---:|---:|---:|",
    f"| wheel | {fmt(metrics['wheel_pre_zero_deg_receive_time'])} | {fmt(metrics['wheel_post_zero_tail_deg_receive_time'])} | {fmt(metrics['wheel_total_deg_receive_time'])} | {fmt(metrics['wheel_pre_zero_target_error_deg'])} | {fmt(metrics['wheel_total_target_error_deg'])} |",
    f"| corrected IMU (receipt-time integral) | {fmt(metrics['imu_pre_zero_deg_receive_time'])} | {fmt(metrics['imu_post_zero_tail_deg_receive_time'])} | {fmt(metrics['imu_total_deg_receive_time'])} | {fmt(metrics['imu_pre_zero_target_error_deg'])} | {fmt(metrics['imu_total_target_error_deg'])} |",
    f"| local odometry | {fmt(metrics['local_pre_zero_deg_receive_time'])} | {fmt(metrics['local_post_zero_tail_deg_receive_time'])} | {fmt(metrics['local_total_deg_receive_time'])} | {fmt(metrics['local_pre_zero_target_error_deg'])} | {fmt(metrics['local_total_target_error_deg'])} |",
    f"| wheel - IMU | {fmt(metrics['wheel_minus_imu_pre_zero_deg'])} | {fmt(metrics['wheel_minus_imu_post_zero_tail_deg'])} | {fmt(metrics['wheel_minus_imu_total_deg_receive_time'])} | n/a | n/a |",
    f"| local - IMU | {fmt(metrics['local_minus_imu_pre_zero_deg'])} | {fmt(metrics['local_minus_imu_post_zero_tail_deg'])} | {fmt(metrics['local_minus_imu_total_deg_receive_time'])} | n/a | n/a |",
    "",
    "## Periodic-Sample Stop Timing (Legacy Comparison)",
    "",
    "| signal | first_zero_or_stop_sec | last_nonzero_or_moving_sec |",
    "|---|---:|---:|",
    f"| `{cmd_topic}` | {fmt(metrics['cmd_in_first_zero_sec'])} | {fmt(metrics['cmd_in_last_nonzero_sec'])} |",
    f"| `/cmd_vel_safe` | {fmt(metrics['cmd_safe_first_zero_sec'])} | {fmt(metrics['cmd_safe_last_nonzero_sec'])} |",
    f"| `/cmd_vel` | {fmt(metrics['cmd_out_first_zero_sec'])} | {fmt(metrics['cmd_out_last_nonzero_sec'])} |",
    f"| `/wheel/odom` twist | {fmt(metrics['wheel_first_stop_sec'])} | {fmt(metrics['wheel_last_moving_sec'])} |",
    f"| `/lidar_imu_bias_corrected` yaw rate | {fmt(metrics['imu_first_stop_sec'])} | {fmt(metrics['imu_last_moving_sec'])} |",
    f"| `/motion_state` angular_velocity | {fmt(metrics['motion_first_stop_sec'])} | n/a |",
    "",
    "## Interpretation",
    "",
]

cmd_in_zero = metrics["cmd_in_first_zero_sec"]
safe_zero = metrics["cmd_safe_first_zero_sec"]
out_zero = metrics["cmd_out_first_zero_sec"]
wheel_stop = metrics["wheel_first_stop_sec"]
if cmd_in_zero is not None and safe_zero is not None and safe_zero - cmd_in_zero > 0.15:
    lines.append("- `robot_safety` output lags the input zero command by more than 0.15s.")
elif cmd_in_zero is not None and safe_zero is not None:
    lines.append("- `robot_safety` output follows the input zero command within the 0.15s threshold.")
else:
    lines.append("- Could not determine input-to-safety zero latency from the captured samples.")

if safe_zero is not None and out_zero is not None and abs(out_zero - safe_zero) <= 0.05:
    lines.append("- `/cmd_vel_safe` and `/cmd_vel` zero together, so the safety mirror and final command output are consistent.")
elif safe_zero is not None and out_zero is not None:
    lines.append("- `/cmd_vel_safe` and `/cmd_vel` zero at different times; inspect robot_safety publish path.")

if out_zero is not None and wheel_stop is not None and wheel_stop - out_zero > 0.5:
    lines.append("- Chassis motion continues more than 0.5s after `/cmd_vel` is zero; this points to chassis/SDK stop dynamics.")
elif out_zero is not None and wheel_stop is not None:
    lines.append("- Wheel odom stops within 0.5s after `/cmd_vel` reaches zero.")

lines.extend([
    "",
    f"- wheel_yaw_before_stop_deg: `{fmt(metrics['wheel_yaw_before_stop_deg'])}`",
    f"- wheel_yaw_tail_deg: `{fmt(metrics['wheel_yaw_tail_deg'])}`",
    f"- wheel_yaw_total_deg: `{fmt(metrics['wheel_yaw_delta_deg'])}`",
    f"- imu_yaw_before_stop_deg: `{fmt(metrics['imu_yaw_before_stop_deg'])}`",
    f"- imu_yaw_tail_deg: `{fmt(metrics['imu_yaw_tail_deg'])}`",
    f"- imu_yaw_total_deg: `{fmt(metrics['imu_yaw_total_deg'])}`",
    f"- wheel_minus_imu_tail_deg: `{fmt(metrics['wheel_minus_imu_tail_deg'])}`",
    "",
    "Files:",
    "",
    "- `samples.csv`",
    "- `receive_events.csv`",
    "- `metrics.json`",
])
summary_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
print(f"[cmd-vel-latency] summary: {summary_path}")
print(f"[cmd-vel-latency] complete: {report_dir}")
PY
