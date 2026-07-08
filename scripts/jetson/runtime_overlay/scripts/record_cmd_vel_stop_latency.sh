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
TARGET_YAW_DEG=""
ANGLE_TOLERANCE_DEG="1.0"
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
target_yaw_deg=${TARGET_YAW_DEG}
angle_tolerance_deg=${ANGLE_TOLERANCE_DEG}
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
  "${SAMPLE_HZ}" "${ZERO_THRESH}" "${WHEEL_STOP_THRESH}" "${TARGET_YAW_DEG}" "${ANGLE_TOLERANCE_DEG}" <<'PY'
import csv
import json
import math
import statistics
import sys
import time
from pathlib import Path
from typing import Dict, Optional

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
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

samples_path = report_dir / "samples.csv"
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
            depth=50,
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
        self.safety_status = ""
        self.mode_status = ""
        self.create_subscription(Twist, cmd_topic, lambda m: setattr(self, "cmd_in", m), cmd_qos)
        self.create_subscription(Twist, "/cmd_vel_safe", lambda m: setattr(self, "cmd_safe", m), cmd_qos)
        self.create_subscription(Twist, "/cmd_vel", lambda m: setattr(self, "cmd_out", m), cmd_qos)
        self.create_subscription(Odometry, "/wheel/odom", lambda m: setattr(self, "wheel", m), odom_qos)
        self.create_subscription(Odometry, "/local_state/odometry", lambda m: setattr(self, "local", m), odom_qos)
        self.create_subscription(String, "/safety/status", lambda m: setattr(self, "safety_status", m.data), 10)
        self.create_subscription(String, "/ranger_mini3_mode_controller/status", lambda m: setattr(self, "mode_status", m.data), 10)
        if MotionState is not None:
            self.create_subscription(MotionState, "/motion_state", lambda m: setattr(self, "motion_state", m), odom_qos)

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

rclpy.init()
node = Probe()
sample_period = 1.0 / max(sample_hz, 1.0)
rows = []
start = time.monotonic()
stop_t = None
start_wheel_yaw = None
target_start_wheel_yaw = None
end_wheel_yaw = None

def append_sample(phase: str, requested_vx: float, requested_wz: float, wall: float) -> None:
    global start_wheel_yaw, end_wheel_yaw
    wheel_yaw = odom_yaw(node.wheel)
    if start_wheel_yaw is None and wheel_yaw is not None:
        start_wheel_yaw = wheel_yaw
    if wheel_yaw is not None:
        end_wheel_yaw = wheel_yaw
    motion_wz = None
    motion_mode = None
    if node.motion_state is not None:
        motion_wz = float(getattr(node.motion_state, "angular_velocity", 0.0))
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
        "local_vx": odom_vx(node.local),
        "local_wz": odom_wz(node.local),
        "motion_wz": motion_wz,
        "motion_mode": motion_mode,
        "safety_status": node.safety_status,
        "mode_status": node.mode_status,
    })

try:
    # Let subscriptions connect and latch current runtime state.
    warm_end = time.monotonic() + 0.35
    while time.monotonic() < warm_end and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.02)
    odom_wait_end = time.monotonic() + 2.0
    while node.wheel is None and time.monotonic() < odom_wait_end and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.05)
    if node.wheel is None:
        raise RuntimeError("no /wheel/odom received before command; refusing to move")

    next_sample = time.monotonic()
    command_end = time.monotonic() + command_sec
    target_reached = False
    target_stop_reason = "duration_elapsed"
    if target_yaw_deg is not None:
        wait_end = time.monotonic() + 1.0
        while node.wheel is None and time.monotonic() < wait_end and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
        target_start_wheel_yaw = odom_yaw(node.wheel)
        if target_start_wheel_yaw is None:
            target_stop_reason = "duration_elapsed_no_initial_target_reference"
    while time.monotonic() < command_end and rclpy.ok():
        now = time.monotonic()
        node.publish_cmd(linear_speed, angular_speed)
        rclpy.spin_once(node, timeout_sec=0.005)
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

    stop_t = time.monotonic()
    settle_end = stop_t + settle_sec
    next_sample = stop_t
    while time.monotonic() < settle_end and rclpy.ok():
        now = time.monotonic()
        node.publish_cmd(0.0, 0.0)
        rclpy.spin_once(node, timeout_sec=0.005)
        if now >= next_sample:
            append_sample("settle", 0.0, 0.0, now)
            next_sample += sample_period
        time.sleep(0.002)
finally:
    try:
        zero_end = time.monotonic() + 1.0
        while time.monotonic() < zero_end and rclpy.ok():
            node.publish_cmd(0.0, 0.0)
            rclpy.spin_once(node, timeout_sec=0.01)
            time.sleep(0.02)
    finally:
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
    "local_vx",
    "local_wz",
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

metrics: Dict[str, Optional[float]] = {
    "cmd_topic": cmd_topic,
    "linear_speed_mps": linear_speed,
    "angular_speed_radps": angular_speed,
    "command_sec": command_sec,
    "settle_sec": settle_sec,
    "zero_thresh_radps": zero_thresh,
    "wheel_stop_thresh_radps": wheel_stop_thresh,
    "target_yaw_deg": target_yaw_deg,
    "angle_tolerance_deg": angle_tolerance_deg,
    "target_reached": target_reached,
    "target_stop_reason": target_stop_reason,
    "cmd_in_first_zero_sec": first_time_zero_twist(rows, "cmd_in_vx", "cmd_in_wz", stop_t, zero_thresh),
    "cmd_safe_first_zero_sec": first_time_zero_twist(rows, "cmd_safe_vx", "cmd_safe_wz", stop_t, zero_thresh),
    "cmd_out_first_zero_sec": first_time_zero_twist(rows, "cmd_out_vx", "cmd_out_wz", stop_t, zero_thresh),
    "wheel_first_stop_sec": first_time_zero_twist(rows, "wheel_vx", "wheel_wz", stop_t, wheel_stop_thresh),
    "motion_first_stop_sec": first_time_below_abs(rows, "motion_wz", stop_t, wheel_stop_thresh),
    "cmd_in_last_nonzero_sec": last_time_nonzero_twist(rows, "cmd_in_vx", "cmd_in_wz", stop_t, zero_thresh),
    "cmd_safe_last_nonzero_sec": last_time_nonzero_twist(rows, "cmd_safe_vx", "cmd_safe_wz", stop_t, zero_thresh),
    "cmd_out_last_nonzero_sec": last_time_nonzero_twist(rows, "cmd_out_vx", "cmd_out_wz", stop_t, zero_thresh),
    "wheel_last_moving_sec": last_time_nonzero_twist(rows, "wheel_vx", "wheel_wz", stop_t, wheel_stop_thresh),
}

if start_wheel_yaw is not None and end_wheel_yaw is not None:
    metrics["wheel_yaw_delta_deg"] = math.degrees(angle_delta(end_wheel_yaw, start_wheel_yaw))
else:
    metrics["wheel_yaw_delta_deg"] = None

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
    f"- settle_sec: `{settle_sec}`",
    f"- zero_thresh_radps: `{zero_thresh}`",
    f"- wheel_stop_thresh_radps: `{wheel_stop_thresh}`",
    "",
    "## Stop Timing",
    "",
    "| signal | first_zero_or_stop_sec | last_nonzero_or_moving_sec |",
    "|---|---:|---:|",
    f"| `{cmd_topic}` | {fmt(metrics['cmd_in_first_zero_sec'])} | {fmt(metrics['cmd_in_last_nonzero_sec'])} |",
    f"| `/cmd_vel_safe` | {fmt(metrics['cmd_safe_first_zero_sec'])} | {fmt(metrics['cmd_safe_last_nonzero_sec'])} |",
    f"| `/cmd_vel` | {fmt(metrics['cmd_out_first_zero_sec'])} | {fmt(metrics['cmd_out_last_nonzero_sec'])} |",
    f"| `/wheel/odom` twist | {fmt(metrics['wheel_first_stop_sec'])} | {fmt(metrics['wheel_last_moving_sec'])} |",
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
    f"- wheel_yaw_delta_deg: `{fmt(metrics['wheel_yaw_delta_deg'])}`",
    "",
    "Files:",
    "",
    "- `samples.csv`",
    "- `metrics.json`",
])
summary_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
print(f"[cmd-vel-latency] summary: {summary_path}")
print(f"[cmd-vel-latency] complete: {report_dir}")
PY
