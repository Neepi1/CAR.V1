#!/usr/bin/env bash
set -euo pipefail
umask 0002

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OVERLAY_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
PROJECT_ROOT="$(cd "${OVERLAY_ROOT}/../../.." && pwd)"

ANGULAR_SPEED_RADPS="0.30"
ANGLE_DEG="90"
COUNTDOWN_SEC="5.0"
BIAS_SEC="3.0"
SETTLE_SEC="3.0"
LABEL="spin_imu_yaw"
CMD_TOPIC="/cmd_vel_collision_checked"
ODOM_TOPIC="/wheel/odom"
IMU_TOPIC="/lidar_imu"
BASE_FRAME="base_link"
PROJECT_IMU_TO_BASE="true"
SAMPLE_HZ="20.0"
ANGLE_TOLERANCE_DEG="1.0"
MAX_EXTRA_SEC="8.0"
TF_WAIT_SEC="10.0"
OUTPUT_ROOT="${NJRH_TEST_OUTPUT_ROOT:-/tmp/ranger_spin_imu_yaw_test}"

usage() {
  cat <<'EOF'
Usage: run_ranger_spin_imu_yaw_test.sh [options]

Runs one odom-controlled spin and compares /wheel/odom yaw against short-term
IMU yaw-rate integration. This does not trigger Isaac, AMCL, or relocalization.

Command path:
  test script -> /cmd_vel_collision_checked -> robot_safety -> /cmd_vel -> ranger_base

Options:
  --angular-speed RADPS       Absolute angular.z command. Default: 0.30
  --angle-deg DEG             Signed target angle. Default: 90
  --countdown-sec SEC         Zero-command warning window before bias. Default: 5.0
  --bias-sec SEC              Stationary gyro-bias collection before motion. Default: 3.0
  --settle-sec SEC            Zero-command settle recording after stop. Default: 3.0
  --sample-hz HZ              CSV sample rate. Default: 20
  --label NAME                Report label. Default: spin_imu_yaw
  --cmd-topic TOPIC           Safety-chain input topic. Default: /cmd_vel_collision_checked
  --odom-topic TOPIC          Wheel odom topic. Default: /wheel/odom
  --imu-topic TOPIC           IMU topic. Default: /lidar_imu
  --base-frame FRAME          Base frame for IMU angular velocity projection. Default: base_link
  --no-project-imu-to-base    Use raw IMU angular_velocity.z instead of projected base-frame z
  --output-root DIR           Report root. Default: /tmp/ranger_spin_imu_yaw_test
  --angle-tolerance-deg DEG   Stop tolerance against wheel odom yaw. Default: 1.0
  --max-extra-sec SEC         Extra timeout beyond target/speed. Default: 8.0
  --tf-wait-sec SEC           Max wait for base<-IMU TF before motion. Default: 10.0
  -h, --help                  Show this help
EOF
}

while [[ "$#" -gt 0 ]]; do
  case "$1" in
    --angular-speed|--speed)
      ANGULAR_SPEED_RADPS="${2:-}"
      shift 2
      ;;
    --angle-deg|--target-deg|--spin-deg)
      ANGLE_DEG="${2:-}"
      shift 2
      ;;
    --countdown-sec)
      COUNTDOWN_SEC="${2:-}"
      shift 2
      ;;
    --bias-sec)
      BIAS_SEC="${2:-}"
      shift 2
      ;;
    --settle-sec)
      SETTLE_SEC="${2:-}"
      shift 2
      ;;
    --sample-hz)
      SAMPLE_HZ="${2:-}"
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
    --imu-topic)
      IMU_TOPIC="${2:-}"
      shift 2
      ;;
    --base-frame)
      BASE_FRAME="${2:-}"
      shift 2
      ;;
    --no-project-imu-to-base)
      PROJECT_IMU_TO_BASE="false"
      shift
      ;;
    --output-root)
      OUTPUT_ROOT="${2:-}"
      shift 2
      ;;
    --angle-tolerance-deg)
      ANGLE_TOLERANCE_DEG="${2:-}"
      shift 2
      ;;
    --max-extra-sec)
      MAX_EXTRA_SEC="${2:-}"
      shift 2
      ;;
    --tf-wait-sec)
      TF_WAIT_SEC="${2:-}"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "[spin-imu-yaw] unknown argument: $1" >&2
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
  echo "# Ranger Spin IMU Yaw Test Environment"
  echo "- timestamp_utc: ${timestamp}"
  echo "- angular_speed_radps: ${ANGULAR_SPEED_RADPS}"
  echo "- angle_deg: ${ANGLE_DEG}"
  echo "- countdown_sec: ${COUNTDOWN_SEC}"
  echo "- bias_sec: ${BIAS_SEC}"
  echo "- settle_sec: ${SETTLE_SEC}"
  echo "- sample_hz: ${SAMPLE_HZ}"
  echo "- cmd_topic: ${CMD_TOPIC}"
  echo "- odom_topic: ${ODOM_TOPIC}"
  echo "- imu_topic: ${IMU_TOPIC}"
  echo "- base_frame: ${BASE_FRAME}"
  echo "- project_imu_to_base: ${PROJECT_IMU_TO_BASE}"
  echo "- tf_wait_sec: ${TF_WAIT_SEC}"
  echo "- rmw: ${RMW_IMPLEMENTATION:-}"
  echo
  echo "## Topic Info"
  for topic in "${CMD_TOPIC}" /cmd_vel_safe /cmd_vel "${ODOM_TOPIC}" "${IMU_TOPIC}" /motion_state; do
    echo "### ${topic}"
    ros2 topic info "${topic}" -v 2>&1 || true
  done
} >"${OUT_DIR}/environment.md"

set +e
python3 - \
  "${OUT_DIR}" \
  "${ANGULAR_SPEED_RADPS}" \
  "${ANGLE_DEG}" \
  "${COUNTDOWN_SEC}" \
  "${BIAS_SEC}" \
  "${SETTLE_SEC}" \
  "${SAMPLE_HZ}" \
  "${CMD_TOPIC}" \
  "${ODOM_TOPIC}" \
  "${IMU_TOPIC}" \
  "${BASE_FRAME}" \
  "${PROJECT_IMU_TO_BASE}" \
  "${ANGLE_TOLERANCE_DEG}" \
  "${MAX_EXTRA_SEC}" \
  "${TF_WAIT_SEC}" <<'PY'
import csv
import json
import math
import os
import statistics
import sys
import time
from pathlib import Path
from typing import Any, Optional, Tuple

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import Imu
from tf2_ros import Buffer, TransformException, TransformListener


out_dir = Path(sys.argv[1])
angular_speed = abs(float(sys.argv[2]))
target_deg = float(sys.argv[3])
countdown_sec = float(sys.argv[4])
bias_sec = float(sys.argv[5])
settle_sec = float(sys.argv[6])
sample_hz = float(sys.argv[7])
cmd_topic = sys.argv[8]
odom_topic = sys.argv[9]
imu_topic = sys.argv[10]
base_frame = sys.argv[11]
project_imu_to_base = sys.argv[12].lower() == "true"
angle_tolerance = math.radians(abs(float(sys.argv[13])))
max_extra_sec = float(sys.argv[14])
tf_wait_sec = float(sys.argv[15])

if angular_speed <= 0.0:
    raise SystemExit("angular speed must be positive")
if sample_hz <= 0.0:
    raise SystemExit("sample_hz must be positive")

target_rad = math.radians(target_deg)
direction = 1.0 if target_rad >= 0.0 else -1.0
command_wz = direction * angular_speed
timeout_sec = abs(target_rad) / max(angular_speed, 1e-6) + max_extra_sec
sample_period = 1.0 / sample_hz


def stamp_sec(stamp: Any) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def norm_angle(value: float) -> float:
    return math.atan2(math.sin(value), math.cos(value))


def yaw_from_quat(q: Any) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def odom_pose(msg: Odometry) -> Tuple[float, float, float]:
    pose = msg.pose.pose
    return (float(pose.position.x), float(pose.position.y), yaw_from_quat(pose.orientation))


def integrate_imu_series_to(series: list, end_stamp: float) -> float:
    total = 0.0
    prev_t: Optional[float] = None
    for t, wz in series:
        if prev_t is not None:
            raw_dt = t - prev_t
            seg_end = min(t, end_stamp)
            dt = seg_end - prev_t
            if 0.0 < dt < 0.05 and 0.0 < raw_dt < 0.05:
                total += wz * dt
        if t >= end_stamp:
            break
        prev_t = t
    return total


def integrate_wheel_series_to(series: list, start_yaw: float, end_stamp: float) -> float:
    total = 0.0
    last_yaw = start_yaw
    for t, yaw in series:
        if t > end_stamp:
            break
        total += norm_angle(yaw - last_yaw)
        last_yaw = yaw
    return total


def rotate_vector_by_quaternion(
    vector: Tuple[float, float, float],
    quaternion: Tuple[float, float, float, float],
) -> Tuple[float, float, float]:
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


class SpinImuNode(Node):
    def __init__(self) -> None:
        super().__init__("ranger_spin_imu_yaw_test")
        cmd_qos = QoSProfile(depth=1)
        # Use a small best-effort queue for the 400 Hz IMU so stale reliable
        # backlog cannot leak into the stationary bias or motion integration.
        imu_qos = QoSProfile(
            depth=50,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        odom_qos = QoSProfile(
            depth=80,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.cmd_pub = self.create_publisher(Twist, cmd_topic, cmd_qos)
        self.imu_msg: Optional[Imu] = None
        self.odom_msg: Optional[Odometry] = None
        self.imu_seq = 0
        self.odom_seq = 0
        self.imu_frame = ""
        self.imu_to_base_quat: Optional[Tuple[float, float, float, float]] = None
        self.imu_yaw_rate_source = "raw_imu_z"
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=False)

        self.collecting_bias = False
        self.bias_start_stamp: Optional[float] = None
        self.bias_samples = []
        self.bias = 0.0

        self.recording = False
        self.record_start_stamp: Optional[float] = None
        self.imu_prev_t: Optional[float] = None
        self.imu_integral = 0.0
        self.imu_integral_at_stop = 0.0
        self.imu_dt_values = []
        self.imu_sample_count = 0
        self.imu_max_abs_wz = 0.0
        self.imu_series = []
        self.odom_series = []

        self.create_subscription(Imu, imu_topic, self.on_imu, imu_qos)
        self.create_subscription(Odometry, odom_topic, self.on_odom, odom_qos)

    def configure_imu_projection(self) -> None:
        if not project_imu_to_base:
            self.imu_to_base_quat = None
            self.imu_yaw_rate_source = "raw_imu_z"
            return
        if not self.imu_frame:
            raise RuntimeError("cannot configure IMU projection before receiving imu frame")
        deadline = time.monotonic() + tf_wait_sec
        last_error = ""
        while time.monotonic() < deadline and rclpy.ok():
            try:
                transform = self.tf_buffer.lookup_transform(
                    base_frame,
                    self.imu_frame,
                    Time(),
                    timeout=Duration(seconds=0.2),
                )
                q = transform.transform.rotation
                self.imu_to_base_quat = (float(q.x), float(q.y), float(q.z), float(q.w))
                self.imu_yaw_rate_source = f"tf_projected_{self.imu_frame}_to_{base_frame}_z"
                return
            except TransformException as exc:
                last_error = str(exc)
                rclpy.spin_once(self, timeout_sec=0.05)
        raise RuntimeError(f"failed to lookup transform {base_frame} <- {self.imu_frame}: {last_error}")

    def imu_yaw_rate(self, msg: Imu) -> float:
        vector = (
            float(msg.angular_velocity.x),
            float(msg.angular_velocity.y),
            float(msg.angular_velocity.z),
        )
        if self.imu_to_base_quat is None:
            return vector[2]
        return rotate_vector_by_quaternion(vector, self.imu_to_base_quat)[2]

    def on_imu(self, msg: Imu) -> None:
        self.imu_msg = msg
        self.imu_seq += 1
        self.imu_frame = msg.header.frame_id
        t = stamp_sec(msg.header.stamp)
        wz = self.imu_yaw_rate(msg)

        if self.collecting_bias:
            if self.bias_start_stamp is not None and t > self.bias_start_stamp:
                self.bias_samples.append(wz)
            return

        if not self.recording:
            return
        if self.record_start_stamp is not None and t <= self.record_start_stamp:
            return
        self.imu_series.append((t, wz - self.bias))
        if self.imu_prev_t is not None:
            dt = t - self.imu_prev_t
            if 0.0 < dt < 0.05:
                self.imu_integral += (wz - self.bias) * dt
                self.imu_dt_values.append(dt)
        self.imu_prev_t = t
        self.imu_sample_count += 1
        self.imu_max_abs_wz = max(self.imu_max_abs_wz, abs(wz - self.bias))

    def on_odom(self, msg: Odometry) -> None:
        self.odom_msg = msg
        self.odom_seq += 1
        if self.recording:
            t = stamp_sec(msg.header.stamp)
            if self.record_start_stamp is None or t > self.record_start_stamp:
                self.odom_series.append((t, odom_pose(msg)[2]))

    def publish_cmd(self, wz: float) -> None:
        msg = Twist()
        msg.angular.z = float(wz)
        self.cmd_pub.publish(msg)

    def zero_burst(self, duration: float) -> None:
        end = time.monotonic() + duration
        while time.monotonic() < end and rclpy.ok():
            self.publish_cmd(0.0)
            rclpy.spin_once(self, timeout_sec=0.003)
            time.sleep(0.02)


def spin_until(node: SpinImuNode, deadline: float, timeout: float = 0.003) -> None:
    while time.monotonic() < deadline and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=timeout)


def main() -> int:
    rclpy.init(args=None)
    node = SpinImuNode()
    samples = []
    ok = False
    reason = "unknown"
    command_stop_time = None
    wheel_accum = 0.0
    wheel_accum_at_stop = 0.0
    max_abs_wheel_wz = 0.0
    try:
        wait_deadline = time.monotonic() + 6.0
        while time.monotonic() < wait_deadline and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.02)
            if node.imu_msg is not None and node.odom_msg is not None:
                break
        if node.imu_msg is None:
            raise RuntimeError(f"no imu received on {imu_topic}")
        if node.odom_msg is None:
            raise RuntimeError(f"no odom received on {odom_topic}")
        node.configure_imu_projection()

        print(
            f"[spin-imu-yaw] motion will start after countdown={countdown_sec:.1f}s "
            f"+ bias={bias_sec:.1f}s speed={angular_speed:.2f} target={target_deg:+.0f}",
            flush=True,
        )
        node.zero_burst(1.0)
        countdown_end = time.monotonic() + countdown_sec
        while time.monotonic() < countdown_end and rclpy.ok():
            node.publish_cmd(0.0)
            rclpy.spin_once(node, timeout_sec=0.01)
            time.sleep(0.02)

        # Mark the latest observed IMU stamp, then only accept newer samples for
        # stationary bias. This avoids stale high-rate backlog contaminating bias.
        node.collecting_bias = True
        node.bias_samples.clear()
        node.bias_start_stamp = stamp_sec(node.imu_msg.header.stamp)
        bias_end = time.monotonic() + bias_sec
        while time.monotonic() < bias_end and rclpy.ok():
            node.publish_cmd(0.0)
            rclpy.spin_once(node, timeout_sec=0.002)
        node.collecting_bias = False
        if len(node.bias_samples) < 100:
            raise RuntimeError(f"not enough imu bias samples: {len(node.bias_samples)}")
        node.bias = statistics.mean(node.bias_samples)
        bias_std = statistics.pstdev(node.bias_samples) if len(node.bias_samples) > 1 else 0.0

        start_pose = odom_pose(node.odom_msg)
        last_yaw = start_pose[2]
        last_odom_seq = node.odom_seq
        node.recording = True
        node.record_start_stamp = stamp_sec(node.imu_msg.header.stamp)
        node.imu_prev_t = None
        node.imu_integral = 0.0
        node.imu_integral_at_stop = 0.0
        node.imu_dt_values.clear()
        node.imu_sample_count = 0
        node.imu_max_abs_wz = 0.0

        start_time = time.monotonic()
        next_pub = start_time
        next_sample = start_time
        command_stop_ros_stamp = None
        while rclpy.ok():
            now = time.monotonic()
            elapsed = now - start_time
            if elapsed > timeout_sec:
                reason = "timeout_before_wheel_target"
                break
            if now >= next_pub:
                node.publish_cmd(command_wz)
                next_pub += 1.0 / 30.0
            rclpy.spin_once(node, timeout_sec=0.002)
            if node.odom_msg is not None and node.odom_seq != last_odom_seq:
                pose = odom_pose(node.odom_msg)
                wheel_accum += norm_angle(pose[2] - last_yaw)
                last_yaw = pose[2]
                last_odom_seq = node.odom_seq
                max_abs_wheel_wz = max(max_abs_wheel_wz, abs(float(node.odom_msg.twist.twist.angular.z)))
            if now >= next_sample:
                samples.append({
                    "phase": "command",
                    "elapsed_sec": elapsed,
                    "wheel_yaw_deg": math.degrees(wheel_accum),
                    "imu_yaw_deg": math.degrees(node.imu_integral),
                    "wheel_wz": float(node.odom_msg.twist.twist.angular.z) if node.odom_msg else float("nan"),
                    "imu_wz_bias_corrected": float(node.imu_yaw_rate(node.imu_msg) - node.bias) if node.imu_msg else float("nan"),
                    "cmd_wz": command_wz,
                })
                next_sample += sample_period
            if abs(wheel_accum) >= max(0.0, abs(target_rad) - angle_tolerance):
                reason = "wheel_target_reached"
                ok = True
                break
            time.sleep(0.001)

        command_stop_time = time.monotonic()
        command_stop_ros_stamp = stamp_sec(node.get_clock().now().to_msg())
        wheel_accum_at_stop_online = wheel_accum
        imu_integral_at_stop_online = node.imu_integral
        # Send zero immediately, then briefly drain queued IMU/odom callbacks so
        # the at-stop metrics are computed against the same ROS timestamp
        # boundary. Without this, the 400 Hz IMU callback stream can be a few
        # degrees behind the odom callback at the moment the command loop exits.
        drain_end = time.monotonic() + 0.20
        while time.monotonic() < drain_end and rclpy.ok():
            node.publish_cmd(0.0)
            rclpy.spin_once(node, timeout_sec=0.002)
        wheel_accum_at_stop = integrate_wheel_series_to(
            node.odom_series,
            start_pose[2],
            command_stop_ros_stamp,
        )
        node.imu_integral_at_stop = integrate_imu_series_to(
            node.imu_series,
            command_stop_ros_stamp,
        )

        settle_end = command_stop_time + settle_sec
        while time.monotonic() < settle_end and rclpy.ok():
            now = time.monotonic()
            node.publish_cmd(0.0)
            rclpy.spin_once(node, timeout_sec=0.002)
            if node.odom_msg is not None and node.odom_seq != last_odom_seq:
                pose = odom_pose(node.odom_msg)
                wheel_accum += norm_angle(pose[2] - last_yaw)
                last_yaw = pose[2]
                last_odom_seq = node.odom_seq
                max_abs_wheel_wz = max(max_abs_wheel_wz, abs(float(node.odom_msg.twist.twist.angular.z)))
            if now >= next_sample:
                samples.append({
                    "phase": "settle",
                    "elapsed_sec": now - start_time,
                    "wheel_yaw_deg": math.degrees(wheel_accum),
                    "imu_yaw_deg": math.degrees(node.imu_integral),
                    "wheel_wz": float(node.odom_msg.twist.twist.angular.z) if node.odom_msg else float("nan"),
                    "imu_wz_bias_corrected": float(node.imu_yaw_rate(node.imu_msg) - node.bias) if node.imu_msg else float("nan"),
                    "cmd_wz": 0.0,
                })
                next_sample += sample_period
            time.sleep(0.001)

        node.recording = False
        node.zero_burst(1.0)
        end_pose = odom_pose(node.odom_msg)
        wheel_xy_drift = math.hypot(end_pose[0] - start_pose[0], end_pose[1] - start_pose[1])
        imu_dt_sum = sum(node.imu_dt_values)
        imu_rate = len(node.imu_dt_values) / imu_dt_sum if imu_dt_sum > 0.0 else float("nan")

        metrics = {
            "ok": ok,
            "reason": reason,
            "speed_radps": angular_speed,
            "target_deg": target_deg,
            "cmd_topic": cmd_topic,
            "odom_topic": odom_topic,
            "imu_topic": imu_topic,
            "imu_frame": node.imu_frame,
            "base_frame": base_frame,
            "imu_yaw_rate_source": node.imu_yaw_rate_source,
            "bias_sec": bias_sec,
            "bias_sample_count": len(node.bias_samples),
            "imu_bias_radps": node.bias,
            "imu_bias_std_radps": bias_std,
            "imu_integrated_rate_hz": imu_rate,
            "imu_sample_count_recorded": node.imu_sample_count,
            "command_duration_sec": None if command_stop_time is None else command_stop_time - start_time,
            "command_stop_ros_stamp_sec": command_stop_ros_stamp,
            "settle_sec": settle_sec,
            "wheel_yaw_deg_at_stop": math.degrees(wheel_accum_at_stop),
            "imu_yaw_deg_at_stop": math.degrees(node.imu_integral_at_stop),
            "wheel_yaw_deg_at_stop_online_before_drain": math.degrees(wheel_accum_at_stop_online),
            "imu_yaw_deg_at_stop_online_before_drain": math.degrees(imu_integral_at_stop_online),
            "wheel_yaw_deg_final": math.degrees(wheel_accum),
            "imu_yaw_deg_final": math.degrees(node.imu_integral),
            "wheel_minus_imu_deg_final": math.degrees(wheel_accum - node.imu_integral),
            "imu_to_wheel_scale_final": (
                node.imu_integral / wheel_accum if abs(wheel_accum) > 1e-6 else None
            ),
            "wheel_target_error_deg_final": math.degrees(wheel_accum - target_rad),
            "imu_target_error_deg_final": math.degrees(node.imu_integral - target_rad),
            "wheel_post_stop_overrun_deg": math.degrees(wheel_accum - wheel_accum_at_stop),
            "imu_post_stop_overrun_deg": math.degrees(node.imu_integral - node.imu_integral_at_stop),
            "wheel_xy_drift_m": wheel_xy_drift,
            "max_abs_wheel_wz_radps": max_abs_wheel_wz,
            "max_abs_imu_wz_bias_corrected_radps": node.imu_max_abs_wz,
        }

        with (out_dir / "samples.csv").open("w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(
                handle,
                fieldnames=[
                    "phase",
                    "elapsed_sec",
                    "wheel_yaw_deg",
                    "imu_yaw_deg",
                    "wheel_wz",
                    "imu_wz_bias_corrected",
                    "cmd_wz",
                ],
            )
            writer.writeheader()
            for row in samples:
                writer.writerow(row)
        with (out_dir / "metrics.json").open("w", encoding="utf-8") as handle:
            json.dump(metrics, handle, indent=2, sort_keys=True)
            handle.write("\n")
        with (out_dir / "summary.md").open("w", encoding="utf-8") as handle:
            handle.write("# Ranger Spin IMU Yaw Test Summary\n\n")
            for key, value in metrics.items():
                handle.write(f"- {key}: `{value}`\n")
            handle.write("\nFiles:\n\n- `metrics.json`\n- `samples.csv`\n- `environment.md`\n")
        print(f"[spin-imu-yaw] summary: {out_dir}/summary.md", flush=True)
        print(f"[spin-imu-yaw] complete: {out_dir}", flush=True)
        return 0 if ok else 10
    except KeyboardInterrupt:
        return 130
    except Exception as exc:
        node.zero_burst(1.0)
        with (out_dir / "error.txt").open("w", encoding="utf-8") as handle:
            handle.write(str(exc) + "\n")
        print(f"[spin-imu-yaw] FAIL: {exc}", flush=True)
        print(f"[spin-imu-yaw] report: {out_dir}", flush=True)
        return 1
    finally:
        try:
            node.zero_burst(1.0)
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
PY

rc=$?
set -e
exit "${rc}"
