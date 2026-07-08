#!/usr/bin/env bash
set -euo pipefail
umask 0002

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OVERLAY_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
PROJECT_ROOT="$(cd "${OVERLAY_ROOT}/../../.." && pwd)"

SPEEDS_RADPS="0.10,0.20,0.30,0.40,0.50,0.60"
ANGLES_DEG="90,-90"
REPEAT="3"
SAMPLE_HZ="25.0"
COUNTDOWN_SEC="5"
SETTLE_MAX_SEC="8.0"
STABLE_HOLD_SEC="0.50"
STOP_WZ_THRESH="0.025"
ANGLE_TOLERANCE_DEG="1.0"
MAX_EXTRA_SEC="10.0"
LABEL="spin_dynamics_sweep"
CMD_TOPIC="/cmd_vel_collision_checked"
ODOM_TOPIC="/wheel/odom"
LOCAL_ODOM_TOPIC="/local_state/odometry"
OUTPUT_ROOT="${NJRH_TEST_OUTPUT_ROOT:-/tmp/ranger_spin_dynamics_sweep}"

usage() {
  cat <<'EOF'
Usage: run_ranger_spin_dynamics_sweep.sh [options]

Runs a readout-oriented Ranger Mini 3 spin dynamics sweep. It commands angular
velocity through the existing safety chain and stops each segment when wheel
odom says the target angle is reached. It then keeps recording zero-command
settling to measure post-stop overrun.

Command path:
  test script -> /cmd_vel_collision_checked -> robot_safety -> /cmd_vel -> ranger_base

Options:
  --speeds-radps LIST       Comma-separated angular speeds. Default: 0.10..0.60
  --angles-deg LIST         Comma-separated signed target angles. Default: 90,-90
  --repeat N                Repeat the angle list for each speed. Default: 3
  --sample-hz HZ            CSV sample rate. Default: 25
  --countdown-sec N         Countdown before motion. Default: 5
  --settle-max-sec SEC      Max zero-command settle wait. Default: 8
  --stable-hold-sec SEC     Required low-wz hold time. Default: 0.50
  --stop-wz-thresh RADPS    Consider stopped under this odom angular speed. Default: 0.025
  --angle-tolerance-deg DEG Stop when target - tolerance is reached. Default: 1.0
  --label NAME              Report label. Default: spin_dynamics_sweep
  --output-root DIR         Report root. Default: /tmp/ranger_spin_dynamics_sweep
  --cmd-topic TOPIC         Safety-chain input. Default: /cmd_vel_collision_checked
  --odom-topic TOPIC        Wheel odom topic. Default: /wheel/odom
  --local-odom-topic TOPIC  Local odom topic. Default: /local_state/odometry
  -h, --help                Show this help
EOF
}

while [[ "$#" -gt 0 ]]; do
  case "$1" in
    --speeds-radps|--angular-speeds)
      SPEEDS_RADPS="${2:-}"
      shift 2
      ;;
    --angles-deg)
      ANGLES_DEG="${2:-}"
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
    --settle-max-sec)
      SETTLE_MAX_SEC="${2:-}"
      shift 2
      ;;
    --stable-hold-sec)
      STABLE_HOLD_SEC="${2:-}"
      shift 2
      ;;
    --stop-wz-thresh)
      STOP_WZ_THRESH="${2:-}"
      shift 2
      ;;
    --angle-tolerance-deg)
      ANGLE_TOLERANCE_DEG="${2:-}"
      shift 2
      ;;
    --label)
      LABEL="${2:-}"
      shift 2
      ;;
    --output-root)
      OUTPUT_ROOT="${2:-}"
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
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "[ranger-spin-sweep] unknown argument: $1" >&2
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
  echo "# Ranger Spin Dynamics Sweep Environment"
  echo "- timestamp_utc: ${timestamp}"
  echo "- speeds_radps: ${SPEEDS_RADPS}"
  echo "- angles_deg: ${ANGLES_DEG}"
  echo "- repeat: ${REPEAT}"
  echo "- sample_hz: ${SAMPLE_HZ}"
  echo "- countdown_sec: ${COUNTDOWN_SEC}"
  echo "- settle_max_sec: ${SETTLE_MAX_SEC}"
  echo "- stable_hold_sec: ${STABLE_HOLD_SEC}"
  echo "- stop_wz_thresh: ${STOP_WZ_THRESH}"
  echo "- angle_tolerance_deg: ${ANGLE_TOLERANCE_DEG}"
  echo "- cmd_topic: ${CMD_TOPIC}"
  echo "- odom_topic: ${ODOM_TOPIC}"
  echo "- local_odom_topic: ${LOCAL_ODOM_TOPIC}"
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
    /safety/status \
    /ranger_mini3_mode_controller/status \
    /motion_state \
    /system_state \
    /localization/bridge_status; do
    echo "### ${topic}"
    ros2 topic info "${topic}" -v 2>&1 || true
  done
} >"${OUT_DIR}/environment.md"

if [[ "${COUNTDOWN_SEC}" != "0" ]]; then
  echo "[ranger-spin-sweep] motion starts in ${COUNTDOWN_SEC}s. Ensure the robot has rotation clearance and E-stop is available."
  remaining="${COUNTDOWN_SEC}"
  while [[ "${remaining}" -gt 0 ]]; do
    echo "[ranger-spin-sweep] ${remaining}..."
    sleep 1
    remaining=$((remaining - 1))
  done
fi

set +e
python3 - \
  "${OUT_DIR}" \
  "${SPEEDS_RADPS}" \
  "${ANGLES_DEG}" \
  "${REPEAT}" \
  "${SAMPLE_HZ}" \
  "${SETTLE_MAX_SEC}" \
  "${STABLE_HOLD_SEC}" \
  "${STOP_WZ_THRESH}" \
  "${ANGLE_TOLERANCE_DEG}" \
  "${MAX_EXTRA_SEC}" \
  "${CMD_TOPIC}" \
  "${ODOM_TOPIC}" \
  "${LOCAL_ODOM_TOPIC}" <<'PY'
import csv
import json
import math
import os
import statistics
import sys
import time
from dataclasses import asdict, dataclass
from typing import Any, Dict, List, Optional, Tuple

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


out_dir = sys.argv[1]
speeds = [abs(float(x)) for x in sys.argv[2].split(",") if x.strip()]
angles = [float(x) for x in sys.argv[3].split(",") if x.strip()]
repeat = int(sys.argv[4])
sample_hz = float(sys.argv[5])
settle_max_sec = float(sys.argv[6])
stable_hold_sec = float(sys.argv[7])
stop_wz_thresh = abs(float(sys.argv[8]))
angle_tolerance = math.radians(abs(float(sys.argv[9])))
max_extra_sec = float(sys.argv[10])
cmd_topic = sys.argv[11]
odom_topic = sys.argv[12]
local_odom_topic = sys.argv[13]

sample_period = 1.0 / sample_hz


def norm_angle(value: float) -> float:
    return math.atan2(math.sin(value), math.cos(value))


def yaw_from_quat(q: Any) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def odom_pose(msg: Odometry) -> Tuple[float, float, float]:
    p = msg.pose.pose
    return (p.position.x, p.position.y, yaw_from_quat(p.orientation))


def odom_wz(msg: Optional[Odometry]) -> float:
    if msg is None:
        return 0.0
    return float(msg.twist.twist.angular.z)


def twist_wz(msg: Optional[Twist]) -> float:
    return 0.0 if msg is None else float(msg.angular.z)


def signed_deg(rad: float) -> float:
    return math.degrees(rad)


def percentile(vals: List[float], p: float) -> Optional[float]:
    if not vals:
        return None
    vals = sorted(vals)
    if len(vals) == 1:
        return vals[0]
    k = (len(vals) - 1) * p / 100.0
    f = math.floor(k)
    c = math.ceil(k)
    if f == c:
        return vals[int(k)]
    return vals[f] * (c - k) + vals[c] * (k - f)


@dataclass
class SegmentMetric:
    segment: int
    speed_radps: float
    target_deg: float
    ok: bool
    reason: str
    command_duration_sec: float
    settle_duration_sec: float
    response_delay_sec: Optional[float]
    cmd_out_delay_sec: Optional[float]
    stop_command_yaw_deg: float
    final_yaw_deg: float
    target_error_deg: float
    post_stop_overrun_deg: float
    max_abs_wheel_wz: float
    max_abs_cmd_safe_wz: float
    max_abs_cmd_out_wz: float
    final_wheel_xy_drift_m: float
    final_local_yaw_deg: Optional[float]
    final_local_xy_drift_m: Optional[float]


class Probe(Node):
    def __init__(self) -> None:
        super().__init__("ranger_spin_dynamics_sweep")
        qos = QoSProfile(depth=80)
        telemetry_qos = QoSProfile(depth=120)
        telemetry_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self.cmd_pub = self.create_publisher(Twist, cmd_topic, qos)
        self.wheel_odom: Optional[Odometry] = None
        self.local_odom: Optional[Odometry] = None
        self.cmd_safe: Optional[Twist] = None
        self.cmd_out: Optional[Twist] = None
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

    def _wheel_cb(self, msg: Odometry) -> None:
        self.wheel_odom = msg

    def _local_cb(self, msg: Odometry) -> None:
        self.local_odom = msg

    def _cmd_safe_cb(self, msg: Twist) -> None:
        self.cmd_safe = msg

    def _cmd_out_cb(self, msg: Twist) -> None:
        self.cmd_out = msg

    def _safety_cb(self, msg: String) -> None:
        self.safety_status = msg.data

    def _mode_cb(self, msg: String) -> None:
        self.mode_status = msg.data

    def _bridge_cb(self, msg: String) -> None:
        self.bridge_status = msg.data

    def publish_cmd(self, wz: float) -> None:
        msg = Twist()
        msg.angular.z = float(wz)
        self.cmd_pub.publish(msg)

    def spin_some(self, duration: float) -> None:
        end = time.monotonic() + duration
        while time.monotonic() < end and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.02)

    def wait_for_odom(self, timeout_sec: float = 5.0) -> bool:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.wheel_odom is not None:
                return True
        return False

    def publish_zero_burst(self, duration: float = 1.0) -> None:
        end = time.monotonic() + duration
        while time.monotonic() < end and rclpy.ok():
            self.publish_cmd(0.0)
            rclpy.spin_once(self, timeout_sec=0.02)
            time.sleep(0.03)


def pose_or_none(msg: Optional[Odometry]) -> Optional[Tuple[float, float, float]]:
    return odom_pose(msg) if msg is not None else None


def xy_dist(a: Tuple[float, float, float], b: Tuple[float, float, float]) -> float:
    return math.hypot(b[0] - a[0], b[1] - a[1])


def run_segment(node: Probe, segment: int, speed: float, target_deg: float, writer: csv.DictWriter) -> SegmentMetric:
    sign = 1.0 if target_deg >= 0.0 else -1.0
    target = math.radians(target_deg)
    target_abs = abs(target)
    command_wz = sign * speed
    timeout_sec = target_abs / max(speed, 1e-6) + max_extra_sec

    if node.wheel_odom is None:
        return SegmentMetric(segment, speed, target_deg, False, "missing_initial_wheel_odom", 0, 0, None, None, 0, 0, 0, 0, 0, 0, 0, 0, None, None)

    start_wheel = odom_pose(node.wheel_odom)
    start_local = pose_or_none(node.local_odom)
    last_wheel_yaw = start_wheel[2]
    last_local_yaw = start_local[2] if start_local else None
    wheel_accum = 0.0
    local_accum = 0.0
    stop_command_accum = 0.0
    response_delay = None
    cmd_out_delay = None
    max_abs_wheel_wz = 0.0
    max_abs_cmd_safe_wz = 0.0
    max_abs_cmd_out_wz = 0.0
    ok = True
    reason = "target_reached"
    start_time = time.monotonic()
    next_sample = start_time

    while rclpy.ok():
        now = time.monotonic()
        elapsed = now - start_time
        if elapsed > timeout_sec:
            ok = False
            reason = "timeout_before_target"
            break

        node.publish_cmd(command_wz)
        rclpy.spin_once(node, timeout_sec=0.01)

        if node.wheel_odom is not None:
            wheel_pose = odom_pose(node.wheel_odom)
            delta = norm_angle(wheel_pose[2] - last_wheel_yaw)
            wheel_accum += delta
            last_wheel_yaw = wheel_pose[2]
            wz = odom_wz(node.wheel_odom)
            max_abs_wheel_wz = max(max_abs_wheel_wz, abs(wz))
            if response_delay is None and abs(wz) >= max(0.03, 0.20 * speed):
                response_delay = elapsed
        else:
            wheel_pose = start_wheel

        local_pose = pose_or_none(node.local_odom)
        if local_pose is not None and last_local_yaw is not None:
            local_delta = norm_angle(local_pose[2] - last_local_yaw)
            local_accum += local_delta
            last_local_yaw = local_pose[2]

        safe_wz = twist_wz(node.cmd_safe)
        out_wz = twist_wz(node.cmd_out)
        max_abs_cmd_safe_wz = max(max_abs_cmd_safe_wz, abs(safe_wz))
        max_abs_cmd_out_wz = max(max_abs_cmd_out_wz, abs(out_wz))
        if cmd_out_delay is None and abs(out_wz) >= max(0.03, 0.20 * speed):
            cmd_out_delay = elapsed

        if now >= next_sample:
            writer.writerow({
                "segment": segment,
                "phase": "command",
                "speed_radps": f"{speed:.6f}",
                "target_deg": f"{target_deg:.3f}",
                "elapsed_sec": f"{elapsed:.4f}",
                "wheel_yaw_accum_deg": f"{signed_deg(wheel_accum):.6f}",
                "local_yaw_accum_deg": "" if local_pose is None else f"{signed_deg(local_accum):.6f}",
                "wheel_wz": f"{odom_wz(node.wheel_odom):.6f}",
                "local_wz": "" if node.local_odom is None else f"{odom_wz(node.local_odom):.6f}",
                "cmd_requested_wz": f"{command_wz:.6f}",
                "cmd_safe_wz": f"{safe_wz:.6f}",
                "cmd_out_wz": f"{out_wz:.6f}",
                "safety_status": node.safety_status,
            })
            next_sample += sample_period

        if abs(wheel_accum) >= max(0.0, target_abs - angle_tolerance):
            break

        time.sleep(0.004)

    stop_command_time = time.monotonic()
    stop_command_accum = wheel_accum
    stable_since = None
    settle_reason = "settled"

    while rclpy.ok():
        now = time.monotonic()
        settle_elapsed = now - stop_command_time
        if settle_elapsed > settle_max_sec:
            settle_reason = "settle_timeout"
            break

        node.publish_cmd(0.0)
        rclpy.spin_once(node, timeout_sec=0.01)

        if node.wheel_odom is not None:
            wheel_pose = odom_pose(node.wheel_odom)
            delta = norm_angle(wheel_pose[2] - last_wheel_yaw)
            wheel_accum += delta
            last_wheel_yaw = wheel_pose[2]
            wz = odom_wz(node.wheel_odom)
            max_abs_wheel_wz = max(max_abs_wheel_wz, abs(wz))
        else:
            wheel_pose = start_wheel
            wz = 0.0

        local_pose = pose_or_none(node.local_odom)
        if local_pose is not None and last_local_yaw is not None:
            local_delta = norm_angle(local_pose[2] - last_local_yaw)
            local_accum += local_delta
            last_local_yaw = local_pose[2]

        safe_wz = twist_wz(node.cmd_safe)
        out_wz = twist_wz(node.cmd_out)
        max_abs_cmd_safe_wz = max(max_abs_cmd_safe_wz, abs(safe_wz))
        max_abs_cmd_out_wz = max(max_abs_cmd_out_wz, abs(out_wz))

        if now >= next_sample:
            writer.writerow({
                "segment": segment,
                "phase": "settle",
                "speed_radps": f"{speed:.6f}",
                "target_deg": f"{target_deg:.3f}",
                "elapsed_sec": f"{(now - start_time):.4f}",
                "wheel_yaw_accum_deg": f"{signed_deg(wheel_accum):.6f}",
                "local_yaw_accum_deg": "" if local_pose is None else f"{signed_deg(local_accum):.6f}",
                "wheel_wz": f"{wz:.6f}",
                "local_wz": "" if node.local_odom is None else f"{odom_wz(node.local_odom):.6f}",
                "cmd_requested_wz": "0.000000",
                "cmd_safe_wz": f"{safe_wz:.6f}",
                "cmd_out_wz": f"{out_wz:.6f}",
                "safety_status": node.safety_status,
            })
            next_sample += sample_period

        if abs(wz) <= stop_wz_thresh:
            if stable_since is None:
                stable_since = now
            elif now - stable_since >= stable_hold_sec:
                break
        else:
            stable_since = None

        time.sleep(0.004)

    end_wheel = odom_pose(node.wheel_odom) if node.wheel_odom is not None else start_wheel
    end_local = pose_or_none(node.local_odom)
    if settle_reason != "settled":
        ok = False
        reason = settle_reason if reason == "target_reached" else f"{reason}+{settle_reason}"

    signed_stop = stop_command_accum
    signed_final = wheel_accum
    signed_target = target
    final_local = local_accum if start_local and end_local else None
    metric = SegmentMetric(
        segment=segment,
        speed_radps=speed,
        target_deg=target_deg,
        ok=ok,
        reason=reason,
        command_duration_sec=stop_command_time - start_time,
        settle_duration_sec=time.monotonic() - stop_command_time,
        response_delay_sec=response_delay,
        cmd_out_delay_sec=cmd_out_delay,
        stop_command_yaw_deg=signed_deg(signed_stop),
        final_yaw_deg=signed_deg(signed_final),
        target_error_deg=signed_deg(signed_final - signed_target),
        post_stop_overrun_deg=signed_deg(signed_final - signed_stop),
        max_abs_wheel_wz=max_abs_wheel_wz,
        max_abs_cmd_safe_wz=max_abs_cmd_safe_wz,
        max_abs_cmd_out_wz=max_abs_cmd_out_wz,
        final_wheel_xy_drift_m=xy_dist(start_wheel, end_wheel),
        final_local_yaw_deg=None if final_local is None else signed_deg(final_local),
        final_local_xy_drift_m=None if not (start_local and end_local) else xy_dist(start_local, end_local),
    )
    node.publish_zero_burst(0.5)
    return metric


def summarize(metrics: List[SegmentMetric]) -> Dict[str, Any]:
    groups: Dict[Tuple[float, float], List[SegmentMetric]] = {}
    for m in metrics:
        groups.setdefault((m.speed_radps, m.target_deg), []).append(m)
    out: Dict[str, Any] = {}
    for (speed, target), vals in sorted(groups.items()):
        key = f"speed_{speed:.2f}_target_{target:+.0f}"
        final_err = [m.target_error_deg for m in vals]
        overrun = [m.post_stop_overrun_deg for m in vals]
        response = [m.response_delay_sec for m in vals if m.response_delay_sec is not None]
        settle = [m.settle_duration_sec for m in vals]
        out[key] = {
            "count": len(vals),
            "ok_count": sum(1 for m in vals if m.ok),
            "target_error_deg_mean": statistics.mean(final_err) if final_err else None,
            "target_error_deg_p95_abs": percentile([abs(v) for v in final_err], 95),
            "post_stop_overrun_deg_mean": statistics.mean(overrun) if overrun else None,
            "post_stop_overrun_deg_p95_abs": percentile([abs(v) for v in overrun], 95),
            "response_delay_sec_mean": statistics.mean(response) if response else None,
            "settle_duration_sec_mean": statistics.mean(settle) if settle else None,
            "max_abs_wheel_wz_mean": statistics.mean([m.max_abs_wheel_wz for m in vals]),
        }
    return out


def main() -> int:
    rclpy.init(args=None)
    node = Probe()
    metrics: List[SegmentMetric] = []
    try:
        if not node.wait_for_odom():
            raise RuntimeError(f"no odometry received on {odom_topic}")
        node.spin_some(0.5)

        with open(os.path.join(out_dir, "samples.csv"), "w", newline="", encoding="utf-8") as f:
            sample_fields = [
                "segment",
                "phase",
                "speed_radps",
                "target_deg",
                "elapsed_sec",
                "wheel_yaw_accum_deg",
                "local_yaw_accum_deg",
                "wheel_wz",
                "local_wz",
                "cmd_requested_wz",
                "cmd_safe_wz",
                "cmd_out_wz",
                "safety_status",
            ]
            writer = csv.DictWriter(f, fieldnames=sample_fields)
            writer.writeheader()
            segment = 0
            for speed in speeds:
                for _ in range(repeat):
                    for angle in angles:
                        segment += 1
                        print(f"[ranger-spin-sweep] segment={segment} speed={speed:.2f} target={angle:+.0f}deg", flush=True)
                        metric = run_segment(node, segment, speed, angle, writer)
                        metrics.append(metric)
                        print(
                            "[ranger-spin-sweep] result "
                            f"seg={segment} ok={metric.ok} final={metric.final_yaw_deg:.2f}deg "
                            f"err={metric.target_error_deg:.2f}deg overrun={metric.post_stop_overrun_deg:.2f}deg "
                            f"settle={metric.settle_duration_sec:.2f}s",
                            flush=True,
                        )
                        if not metric.ok:
                            break
                    if metrics and not metrics[-1].ok:
                        break
                if metrics and not metrics[-1].ok:
                    break

        metric_fields = list(asdict(metrics[0]).keys()) if metrics else list(SegmentMetric.__dataclass_fields__.keys())
        with open(os.path.join(out_dir, "metrics.csv"), "w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=metric_fields)
            writer.writeheader()
            for m in metrics:
                row = asdict(m)
                writer.writerow(row)

        summary = {
            "speeds_radps": speeds,
            "angles_deg": angles,
            "repeat": repeat,
            "metrics": [asdict(m) for m in metrics],
            "groups": summarize(metrics),
            "bridge_status_raw": node.bridge_status,
        }
        with open(os.path.join(out_dir, "metrics.json"), "w", encoding="utf-8") as f:
            json.dump(summary, f, indent=2, sort_keys=True)
            f.write("\n")

        lines = [
            "# Ranger Spin Dynamics Sweep Summary",
            "",
            f"- report_dir: `{out_dir}`",
            f"- speeds_radps: `{','.join(f'{s:.2f}' for s in speeds)}`",
            f"- angles_deg: `{','.join(f'{a:+.0f}' for a in angles)}`",
            f"- repeat: `{repeat}`",
            f"- sample_hz: `{sample_hz}`",
            f"- settle_max_sec: `{settle_max_sec}`",
            f"- stable_hold_sec: `{stable_hold_sec}`",
            f"- stop_wz_thresh: `{stop_wz_thresh}`",
            "",
            "## Per Segment",
            "",
            "| seg | speed | target_deg | ok | reason | cmd_sec | settle_sec | stop_yaw_deg | final_yaw_deg | target_error_deg | post_stop_overrun_deg | max_wheel_wz | response_delay_sec |",
            "|---:|---:|---:|---|---|---:|---:|---:|---:|---:|---:|---:|---:|",
        ]
        for m in metrics:
            lines.append(
                f"| {m.segment} | {m.speed_radps:.2f} | {m.target_deg:.0f} | {str(m.ok).lower()} | {m.reason} | "
                f"{m.command_duration_sec:.3f} | {m.settle_duration_sec:.3f} | {m.stop_command_yaw_deg:.2f} | "
                f"{m.final_yaw_deg:.2f} | {m.target_error_deg:.2f} | {m.post_stop_overrun_deg:.2f} | "
                f"{m.max_abs_wheel_wz:.3f} | {'' if m.response_delay_sec is None else f'{m.response_delay_sec:.3f}'} |"
            )
        lines.extend(["", "## Group Summary", ""])
        lines.append("| speed | target_deg | count | ok | mean_error_deg | p95_abs_error_deg | mean_overrun_deg | p95_abs_overrun_deg | mean_response_delay_sec | mean_settle_sec |")
        lines.append("|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|")
        group_data = summarize(metrics)
        for key, data in group_data.items():
            parts = key.split("_")
            speed = float(parts[1])
            target = float(parts[3])
            response_mean = data["response_delay_sec_mean"]
            response_text = "" if response_mean is None else f"{response_mean:.3f}"
            lines.append(
                f"| {speed:.2f} | {target:.0f} | {data['count']} | {data['ok_count']} | "
                f"{data['target_error_deg_mean']:.2f} | {data['target_error_deg_p95_abs']:.2f} | "
                f"{data['post_stop_overrun_deg_mean']:.2f} | {data['post_stop_overrun_deg_p95_abs']:.2f} | "
                f"{response_text} | "
                f"{data['settle_duration_sec_mean']:.3f} |"
            )
        lines.extend([
            "",
            "Files:",
            "",
            "- `samples.csv`",
            "- `metrics.csv`",
            "- `metrics.json`",
            "- `environment.md`",
        ])
        with open(os.path.join(out_dir, "summary.md"), "w", encoding="utf-8") as f:
            f.write("\n".join(lines) + "\n")
        return 0 if metrics and all(m.ok for m in metrics) else 10
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
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
PY

rc=$?
set -e
if [[ "${rc}" -eq 0 ]]; then
  echo "[ranger-spin-sweep] summary: ${OUT_DIR}/summary.md"
  echo "[ranger-spin-sweep] complete: ${OUT_DIR}"
else
  echo "[ranger-spin-sweep] FAIL rc=${rc} report: ${OUT_DIR}" >&2
fi
exit "${rc}"
