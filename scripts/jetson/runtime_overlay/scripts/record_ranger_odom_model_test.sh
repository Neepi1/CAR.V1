#!/usr/bin/env bash
set -euo pipefail
umask 0002

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OVERLAY_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
PROJECT_ROOT="$(cd "${OVERLAY_ROOT}/../../.." && pwd)"

DURATION_SEC=60
SAMPLE_HZ=20.0
COUNTDOWN_SEC=3
LABEL="odom_model_test"
CAN_IFACE="${CAN_IFACE:-can0}"
OUTPUT_ROOT="${PROJECT_ROOT}/reports/ranger_odom_model_test"

usage() {
  cat <<'EOF'
Usage: record_ranger_odom_model_test.sh [--duration-sec N] [--sample-hz HZ] [--countdown-sec N] [--label NAME] [--can-iface can0]

Read-only Ranger Mini 3 odometry model capture.

Records:
  - raw CAN 0x221 motion state, decoded as SDK protocol v2:
    linear_velocity, angular_velocity, lateral_velocity, steering_angle
  - /wheel/odom, /wheel/odom_ekf, /local_state/odometry
  - TF map->odom, odom->base_link, map->base_link
  - /cmd_vel_safe, /cmd_vel, /cmd_vel_nav, /cmd_vel_collision_checked
  - /motion_state, /system_state, /ranger_mini3_mode_controller/status

The script does not publish velocity, does not call localization, and does not
change parameters. Drive the robot manually or run a navigation goal while it is
capturing.
EOF
}

while [[ "$#" -gt 0 ]]; do
  case "$1" in
    --duration-sec)
      [[ "$#" -ge 2 ]] || { echo "[ranger-odom-model] --duration-sec requires a value" >&2; exit 2; }
      DURATION_SEC="$2"
      shift 2
      ;;
    --sample-hz)
      [[ "$#" -ge 2 ]] || { echo "[ranger-odom-model] --sample-hz requires a value" >&2; exit 2; }
      SAMPLE_HZ="$2"
      shift 2
      ;;
    --countdown-sec)
      [[ "$#" -ge 2 ]] || { echo "[ranger-odom-model] --countdown-sec requires a value" >&2; exit 2; }
      COUNTDOWN_SEC="$2"
      shift 2
      ;;
    --label)
      [[ "$#" -ge 2 ]] || { echo "[ranger-odom-model] --label requires a value" >&2; exit 2; }
      LABEL="$2"
      shift 2
      ;;
    --can-iface)
      [[ "$#" -ge 2 ]] || { echo "[ranger-odom-model] --can-iface requires a value" >&2; exit 2; }
      CAN_IFACE="$2"
      shift 2
      ;;
    --output-root)
      [[ "$#" -ge 2 ]] || { echo "[ranger-odom-model] --output-root requires a value" >&2; exit 2; }
      OUTPUT_ROOT="$2"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "[ranger-odom-model] unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

case "${DURATION_SEC}" in
  ''|*[!0-9]*) echo "[ranger-odom-model] --duration-sec must be an integer" >&2; exit 2 ;;
esac
case "${COUNTDOWN_SEC}" in
  ''|*[!0-9]*) echo "[ranger-odom-model] --countdown-sec must be an integer" >&2; exit 2 ;;
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

safe_label="$(printf '%s' "${LABEL}" | tr -c 'A-Za-z0-9_.-' '_')"
timestamp="$(date -u +%Y%m%dT%H%M%SZ)"
OUT_DIR="${OUTPUT_ROOT}/${timestamp}_${safe_label}"
mkdir -p "${OUT_DIR}"

{
  echo "# Ranger Odom Model Test Environment"
  echo "- timestamp_utc: ${timestamp}"
  echo "- duration_sec: ${DURATION_SEC}"
  echo "- sample_hz: ${SAMPLE_HZ}"
  echo "- countdown_sec: ${COUNTDOWN_SEC}"
  echo "- label: ${LABEL}"
  echo "- can_iface: ${CAN_IFACE}"
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
    /motion_state \
    /system_state \
    /ranger_mini3_mode_controller/status \
    /wheel/odom \
    /wheel/odom_ekf \
    /local_state/odometry \
    /cmd_vel_safe \
    /cmd_vel \
    /cmd_vel_nav \
    /cmd_vel_collision_checked \
    /localization/bridge_status; do
    echo "### ${topic}"
    ros2 topic info "${topic}" 2>&1 || true
  done
  echo
  echo "## Interfaces"
  ros2 interface show ranger_msgs/msg/MotionState 2>&1 || true
  echo "---"
  ros2 interface show ranger_msgs/msg/SystemState 2>&1 || true
} >"${OUT_DIR}/environment.md"

if [[ "${COUNTDOWN_SEC}" -gt 0 ]]; then
  echo "[ranger-odom-model] capture starts in ${COUNTDOWN_SEC}s. Prepare the maneuver now."
  while [[ "${COUNTDOWN_SEC}" -gt 0 ]]; do
    echo "[ranger-odom-model] ${COUNTDOWN_SEC}..."
    sleep 1
    COUNTDOWN_SEC=$((COUNTDOWN_SEC - 1))
  done
fi

echo "[ranger-odom-model] capturing ${DURATION_SEC}s at ${SAMPLE_HZ} Hz, CAN=${CAN_IFACE}"
python3 - "${DURATION_SEC}" "${SAMPLE_HZ}" "${OUT_DIR}" "${CAN_IFACE}" <<'PY'
import csv
import json
import math
import os
import re
import select
import signal
import subprocess
import sys
import time
from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Tuple

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

try:
    from ranger_msgs.msg import MotionState, SystemState
except Exception:
    MotionState = None
    SystemState = None


duration = float(sys.argv[1])
sample_hz = float(sys.argv[2])
out_dir = sys.argv[3]
can_iface = sys.argv[4]
sample_period = 1.0 / sample_hz

WHEELBASE = 0.494
TRACK = 0.364
MODE_DUAL_ACKERMAN = 0
MODE_PARALLEL = 1
MODE_SPINNING = 2
MODE_SIDE_SLIP = 3


def norm_angle(value: float) -> float:
    return math.atan2(math.sin(value), math.cos(value))


def yaw_from_quat(q: Any) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def pose_from_odom(msg: Odometry) -> Tuple[float, float, float]:
    pose = msg.pose.pose
    return (pose.position.x, pose.position.y, yaw_from_quat(pose.orientation))


def twist_from_odom(msg: Odometry) -> Tuple[float, float, float]:
    twist = msg.twist.twist
    return (twist.linear.x, twist.linear.y, twist.angular.z)


def twist_tuple(msg: Twist) -> Tuple[float, float, float]:
    return (msg.linear.x, msg.linear.y, msg.angular.z)


def central_from_inner(angle: float) -> float:
    phi_i = abs(angle)
    phi = math.atan(
        WHEELBASE * math.sin(phi_i) /
        (WHEELBASE * math.cos(phi_i) + TRACK * math.sin(phi_i))
    )
    return phi if angle >= 0.0 else -phi


def signed_i16(low: int, high: int) -> int:
    value = low | (high << 8)
    if value >= 0x8000:
        value -= 0x10000
    return value


def decode_can_221(data: List[int]) -> Dict[str, float]:
    # ProtocolV2's default struct16_t layout is high_byte, low_byte on the CAN
    # wire. The SDK parser names the fields then combines low | high << 8.
    return {
        "linear": signed_i16(data[1], data[0]) / 1000.0,
        "angular": signed_i16(data[3], data[2]) / 1000.0,
        "lateral": signed_i16(data[5], data[4]) / 1000.0,
        "steering": signed_i16(data[7], data[6]) / 1000.0,
    }


def mode_name(code: Optional[int]) -> str:
    return {
        0: "DUAL_ACKERMAN",
        1: "PARALLEL",
        2: "SPINNING",
        3: "SIDE_SLIP",
    }.get(code, "UNKNOWN")


@dataclass
class Latest:
    msg: Any = None
    wall: float = 0.0


class Capture(Node):
    def __init__(self) -> None:
        super().__init__("record_ranger_odom_model_test")
        qos = QoSProfile(depth=50)
        be = QoSProfile(depth=100)
        be.reliability = ReliabilityPolicy.BEST_EFFORT
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.latest: Dict[str, Latest] = {name: Latest() for name in (
            "wheel", "wheel_ekf", "local", "cmd_safe", "cmd", "cmd_nav",
            "cmd_collision", "mode_status", "motion_state", "system_state",
            "bridge_status",
        )}
        self.create_subscription(Odometry, "/wheel/odom", self.cb("wheel"), be)
        self.create_subscription(Odometry, "/wheel/odom_ekf", self.cb("wheel_ekf"), be)
        self.create_subscription(Odometry, "/local_state/odometry", self.cb("local"), be)
        self.create_subscription(Twist, "/cmd_vel_safe", self.cb("cmd_safe"), qos)
        self.create_subscription(Twist, "/cmd_vel", self.cb("cmd"), qos)
        self.create_subscription(Twist, "/cmd_vel_nav", self.cb("cmd_nav"), qos)
        self.create_subscription(Twist, "/cmd_vel_collision_checked", self.cb("cmd_collision"), qos)
        self.create_subscription(String, "/ranger_mini3_mode_controller/status", self.cb("mode_status"), qos)
        self.create_subscription(String, "/localization/bridge_status", self.cb("bridge_status"), qos)
        if MotionState is not None:
            self.create_subscription(MotionState, "/motion_state", self.cb("motion_state"), be)
        if SystemState is not None:
            self.create_subscription(SystemState, "/system_state", self.cb("system_state"), be)

    def cb(self, name: str):
        def inner(msg: Any) -> None:
            self.latest[name] = Latest(msg=msg, wall=time.monotonic())
        return inner

    def now_sec(self) -> float:
        n = self.get_clock().now().nanoseconds
        return n * 1e-9

    def tf_pose(self, target: str, source: str) -> Optional[Tuple[float, float, float]]:
        try:
            tf = self.tf_buffer.lookup_transform(target, source, rclpy.time.Time())
        except TransformException:
            return None
        t = tf.transform.translation
        q = tf.transform.rotation
        return (t.x, t.y, yaw_from_quat(q))


class Can221Reader:
    def __init__(self, iface: str, raw_path: str) -> None:
        self.iface = iface
        self.raw_path = raw_path
        self.proc: Optional[subprocess.Popen[str]] = None
        self.latest: Optional[Dict[str, float]] = None
        self.latest_raw = ""
        self.count = 0
        self.error = ""
        self._raw_file = open(raw_path, "w", encoding="utf-8")
        try:
            self.proc = subprocess.Popen(
                ["candump", f"{iface},221:7FF"],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
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
            self._raw_file.write(line + "\n")
            self._raw_file.flush()
            match = re.search(r"\b221\b\s+\[8\]\s+((?:[0-9A-Fa-f]{2}\s+){7}[0-9A-Fa-f]{2})", line)
            if not match:
                continue
            data = [int(x, 16) for x in match.group(1).split()]
            self.latest = decode_can_221(data)
            self.latest_raw = " ".join(f"{x:02X}" for x in data)
            self.count += 1

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
        self._raw_file.close()


def odom_fields(name: str, latest: Latest) -> Dict[str, Any]:
    keys = ("x", "y", "yaw", "vx", "vy", "wz", "age_ms")
    if latest.msg is None:
        return {f"{name}_{k}": "" for k in keys}
    pose = pose_from_odom(latest.msg)
    twist = twist_from_odom(latest.msg)
    return {
        f"{name}_x": pose[0],
        f"{name}_y": pose[1],
        f"{name}_yaw": pose[2],
        f"{name}_vx": twist[0],
        f"{name}_vy": twist[1],
        f"{name}_wz": twist[2],
        f"{name}_age_ms": (time.monotonic() - latest.wall) * 1000.0,
    }


def twist_fields(name: str, latest: Latest) -> Dict[str, Any]:
    if latest.msg is None:
        return {f"{name}_{k}": "" for k in ("vx", "vy", "wz", "age_ms")}
    twist = twist_tuple(latest.msg)
    return {
        f"{name}_vx": twist[0],
        f"{name}_vy": twist[1],
        f"{name}_wz": twist[2],
        f"{name}_age_ms": (time.monotonic() - latest.wall) * 1000.0,
    }


def tf_fields(name: str, pose: Optional[Tuple[float, float, float]]) -> Dict[str, Any]:
    if pose is None:
        return {f"{name}_{k}": "" for k in ("x", "y", "yaw")}
    return {f"{name}_x": pose[0], f"{name}_y": pose[1], f"{name}_yaw": pose[2]}


def motion_mode_from_msg(latest: Latest) -> str:
    if latest.msg is None:
        return ""
    return str(getattr(latest.msg, "motion_mode", ""))


def parse_mode_status(latest: Latest) -> Dict[str, Any]:
    empty = {
        "status_desired_code": "",
        "status_actual_code": "",
        "status_actual_name": "",
        "status_mode_aligned": "",
        "status_raw": "",
    }
    if latest.msg is None:
        return empty
    raw = getattr(latest.msg, "data", "")
    try:
        data = json.loads(raw)
    except Exception:
        empty["status_raw"] = raw
        return empty
    desired = data.get("desired_motion_mode") or {}
    actual = data.get("actual_motion_mode") or {}
    return {
        "status_desired_code": desired.get("code", ""),
        "status_actual_code": actual.get("code", ""),
        "status_actual_name": actual.get("name", ""),
        "status_mode_aligned": data.get("mode_aligned", data.get("motion_mode_matched", "")),
        "status_raw": raw,
    }


def pose_from_row(row: Dict[str, Any], prefix: str) -> Optional[Tuple[float, float, float]]:
    try:
        x = row.get(f"{prefix}_x", "")
        y = row.get(f"{prefix}_y", "")
        yaw = row.get(f"{prefix}_yaw", "")
        if x == "" or y == "" or yaw == "":
            return None
        return (float(x), float(y), float(yaw))
    except Exception:
        return None


def start_frame_delta(a: Optional[Tuple[float, float, float]], b: Optional[Tuple[float, float, float]]) -> Dict[str, str]:
    if a is None or b is None:
        return {"forward_m": "nan", "left_m": "nan", "distance_m": "nan", "yaw_rad": "nan"}
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    c = math.cos(a[2])
    s = math.sin(a[2])
    forward = c * dx + s * dy
    left = -s * dx + c * dy
    return {
        "forward_m": f"{forward:.6f}",
        "left_m": f"{left:.6f}",
        "distance_m": f"{math.hypot(dx, dy):.6f}",
        "yaw_rad": f"{norm_angle(b[2] - a[2]):.6f}",
    }


def max_abs(values: List[float]) -> float:
    return max((abs(v) for v in values), default=float("nan"))


def float_values(rows: List[Dict[str, Any]], key: str) -> List[float]:
    out = []
    for row in rows:
        value = row.get(key, "")
        if value == "" or value is None:
            continue
        try:
            out.append(float(value))
        except Exception:
            pass
    return out


rclpy.init()
node = Capture()
can = Can221Reader(can_iface, os.path.join(out_dir, "candump_221.log"))

rows: List[Dict[str, Any]] = []
can_model = [0.0, 0.0, 0.0]
last_sample_wall: Optional[float] = None
deadline = time.monotonic() + duration
next_sample = time.monotonic()
try:
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.0)
        can.poll()
        now = time.monotonic()
        if now < next_sample:
            time.sleep(min(0.02, next_sample - now))
            continue

        dt = 0.0 if last_sample_wall is None else max(0.0, now - last_sample_wall)
        last_sample_wall = now
        mode_code = None
        try:
            mode_code = int(motion_mode_from_msg(node.latest["motion_state"]))
        except Exception:
            mode_code = None
        if can.latest is not None and dt > 0.0:
            linear = can.latest["linear"]
            angular = can.latest["angular"]
            steering = can.latest["steering"]
            if mode_code == MODE_DUAL_ACKERMAN:
                central = central_from_inner(steering)
                can_model[0] += linear * math.cos(central) * math.cos(can_model[2]) * dt
                can_model[1] += linear * math.cos(central) * math.sin(can_model[2]) * dt
                can_model[2] = norm_angle(can_model[2] + 2.0 * linear * math.sin(central) / WHEELBASE * dt)
            elif mode_code == MODE_SPINNING:
                can_model[2] = norm_angle(can_model[2] + angular * dt)
            elif mode_code in (MODE_PARALLEL, MODE_SIDE_SLIP):
                phi = math.pi / 2.0 if mode_code == MODE_SIDE_SLIP else steering
                can_model[0] += linear * math.cos(can_model[2] + phi) * dt
                can_model[1] += linear * math.sin(can_model[2] + phi) * dt

        row: Dict[str, Any] = {
            "wall_time_sec": time.time(),
            "elapsed_sec": duration - max(0.0, deadline - now),
            "ros_time_sec": node.now_sec(),
            "can221_count": can.count,
            "can221_raw": can.latest_raw,
            "can221_linear": "" if can.latest is None else can.latest["linear"],
            "can221_angular": "" if can.latest is None else can.latest["angular"],
            "can221_lateral": "" if can.latest is None else can.latest["lateral"],
            "can221_steering": "" if can.latest is None else can.latest["steering"],
            "can_model_x": can_model[0],
            "can_model_y": can_model[1],
            "can_model_yaw": can_model[2],
            "motion_mode": motion_mode_from_msg(node.latest["motion_state"]),
            "motion_mode_name": mode_name(mode_code),
            "system_motion_mode": motion_mode_from_msg(node.latest["system_state"]),
        }
        row.update(odom_fields("wheel", node.latest["wheel"]))
        row.update(odom_fields("wheel_ekf", node.latest["wheel_ekf"]))
        row.update(odom_fields("local", node.latest["local"]))
        row.update(twist_fields("cmd_safe", node.latest["cmd_safe"]))
        row.update(twist_fields("cmd", node.latest["cmd"]))
        row.update(twist_fields("cmd_nav", node.latest["cmd_nav"]))
        row.update(twist_fields("cmd_collision", node.latest["cmd_collision"]))
        row.update(parse_mode_status(node.latest["mode_status"]))
        row.update(tf_fields("tf_map_odom", node.tf_pose("map", "odom")))
        row.update(tf_fields("tf_odom_base", node.tf_pose("odom", "base_link")))
        row.update(tf_fields("tf_map_base", node.tf_pose("map", "base_link")))
        rows.append(row)
        next_sample += sample_period
        if len(rows) % max(1, int(sample_hz * 5.0)) == 0:
            print(f"[ranger-odom-model] sampled {len(rows)} rows elapsed={row['elapsed_sec']:.1f}s")
finally:
    can.close()
    node.destroy_node()
    rclpy.shutdown()

csv_path = os.path.join(out_dir, "samples.csv")
if rows:
    with open(csv_path, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)
else:
    open(csv_path, "w", encoding="utf-8").close()

def first_last(prefix: str) -> Tuple[Optional[Tuple[float, float, float]], Optional[Tuple[float, float, float]]]:
    first = next((pose_from_row(row, prefix) for row in rows if pose_from_row(row, prefix) is not None), None)
    last = next((pose_from_row(row, prefix) for row in reversed(rows) if pose_from_row(row, prefix) is not None), None)
    return first, last

summary: Dict[str, Any] = {
    "duration_sec": duration,
    "sample_hz": sample_hz,
    "sample_count": len(rows),
    "can221_count": can.count,
    "can221_error": can.error,
    "motion_modes_seen": ",".join(sorted(set(str(r.get("motion_mode", "")) for r in rows if str(r.get("motion_mode", "")) != ""))),
    "status_actual_modes_seen": ",".join(sorted(set(str(r.get("status_actual_code", "")) for r in rows if str(r.get("status_actual_code", "")) != ""))),
}

for prefix in ("wheel", "wheel_ekf", "local", "tf_odom_base", "tf_map_base", "can_model"):
    first, last = first_last(prefix)
    delta = start_frame_delta(first, last)
    for key, value in delta.items():
        summary[f"{prefix}_delta_{key}"] = value

summary["can221_max_abs_linear"] = f"{max_abs(float_values(rows, 'can221_linear')):.6f}"
summary["can221_max_abs_angular"] = f"{max_abs(float_values(rows, 'can221_angular')):.6f}"
summary["can221_max_abs_steering"] = f"{max_abs(float_values(rows, 'can221_steering')):.6f}"
summary["wheel_max_abs_vx"] = f"{max_abs(float_values(rows, 'wheel_vx')):.6f}"
summary["wheel_max_abs_wz"] = f"{max_abs(float_values(rows, 'wheel_wz')):.6f}"
summary["cmd_safe_max_abs_vx"] = f"{max_abs(float_values(rows, 'cmd_safe_vx')):.6f}"
summary["cmd_safe_max_abs_wz"] = f"{max_abs(float_values(rows, 'cmd_safe_wz')):.6f}"
summary["cmd_max_abs_vx"] = f"{max_abs(float_values(rows, 'cmd_vx')):.6f}"
summary["cmd_max_abs_wz"] = f"{max_abs(float_values(rows, 'cmd_wz')):.6f}"

if rows:
    first_wheel = next((pose_from_row(row, "wheel") for row in rows if pose_from_row(row, "wheel") is not None), None)
    last_wheel = pose_from_row(rows[-1], "wheel")
    first_can = next((pose_from_row(row, "can_model") for row in rows if pose_from_row(row, "can_model") is not None), None)
    last_can = pose_from_row(rows[-1], "can_model")
    if first_wheel is not None and last_wheel is not None and first_can is not None and last_can is not None:
        wheel_dx = last_wheel[0] - first_wheel[0]
        wheel_dy = last_wheel[1] - first_wheel[1]
        wheel_dyaw = norm_angle(last_wheel[2] - first_wheel[2])
        c = math.cos(first_wheel[2])
        s = math.sin(first_wheel[2])
        wheel_forward = c * wheel_dx + s * wheel_dy
        wheel_left = -s * wheel_dx + c * wheel_dy
        can_dx = last_can[0] - first_can[0]
        can_dy = last_can[1] - first_can[1]
        can_dyaw = norm_angle(last_can[2] - first_can[2])
        summary["wheel_vs_integrated_can_model_end_m"] = (
            f"{math.hypot(wheel_forward - can_dx, wheel_left - can_dy):.6f}"
        )
        summary["wheel_vs_integrated_can_model_end_yaw_rad"] = (
            f"{norm_angle(wheel_dyaw - can_dyaw):.6f}"
        )

summary_json = os.path.join(out_dir, "summary.json")
with open(summary_json, "w", encoding="utf-8") as f:
    json.dump(summary, f, indent=2, sort_keys=True)
    f.write("\n")

summary_md = os.path.join(out_dir, "summary.md")
with open(summary_md, "w", encoding="utf-8") as f:
    f.write("# Ranger Odom Model Test\n\n")
    f.write("Read-only capture. It did not publish velocity, call localization, or change parameters.\n\n")
    f.write("## Key Results\n\n")
    for key in (
        "duration_sec",
        "sample_count",
        "can221_count",
        "motion_modes_seen",
        "status_actual_modes_seen",
        "can221_max_abs_linear",
        "can221_max_abs_angular",
        "can221_max_abs_steering",
        "wheel_delta_forward_m",
        "wheel_delta_left_m",
        "wheel_delta_distance_m",
        "wheel_delta_yaw_rad",
        "local_delta_forward_m",
        "local_delta_left_m",
        "local_delta_distance_m",
        "local_delta_yaw_rad",
        "tf_odom_base_delta_forward_m",
        "tf_odom_base_delta_left_m",
        "tf_odom_base_delta_distance_m",
        "tf_odom_base_delta_yaw_rad",
        "tf_map_base_delta_distance_m",
        "tf_map_base_delta_yaw_rad",
        "can_model_delta_distance_m",
        "can_model_delta_yaw_rad",
        "wheel_vs_integrated_can_model_end_m",
        "wheel_vs_integrated_can_model_end_yaw_rad",
        "cmd_safe_max_abs_vx",
        "cmd_safe_max_abs_wz",
        "cmd_max_abs_vx",
        "cmd_max_abs_wz",
    ):
        f.write(f"- {key}: `{summary.get(key, '')}`\n")
    f.write("\n## Interpretation\n\n")
    f.write("- If CAN 0x221 steering is near zero during an arc turn, the chassis did not report the commanded Ackermann geometry.\n")
    f.write("- If `/wheel/odom` matches the integrated CAN model but the real robot does not, the driver model or SDK feedback convention is wrong.\n")
    f.write("- If `/wheel/odom` and `/local_state/odometry` diverge, the wheel odom EKF/local_state stage is adding error.\n")
    f.write("- For pure spin, `wheel_delta_distance_m` should stay small while yaw changes and motion mode should be `2`.\n")
    f.write("- For Ackermann arcs, compare left and right arc captures. Sign or scale asymmetry points to steering sign/scale or wheelbase/track mismatch.\n")
    f.write("\n## Files\n\n")
    f.write("- `samples.csv`\n")
    f.write("- `summary.json`\n")
    f.write("- `candump_221.log`\n")
    f.write("- `environment.md`\n")

print(f"[ranger-odom-model] wrote {csv_path}")
print(f"[ranger-odom-model] wrote {summary_md}")
PY

echo "[ranger-odom-model] complete: ${OUT_DIR}"
echo "[ranger-odom-model] summary: ${OUT_DIR}/summary.md"
