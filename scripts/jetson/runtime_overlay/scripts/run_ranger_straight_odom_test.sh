#!/usr/bin/env bash
set -euo pipefail
umask 0002

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OVERLAY_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
PROJECT_ROOT="$(cd "${OVERLAY_ROOT}/../../.." && pwd)"

DISTANCES_M="5.0"
LINEAR_SPEED_MPS="0.20"
REPEAT="1"
SAMPLE_HZ="20.0"
COUNTDOWN_SEC="3"
SETTLE_SEC="3.0"
LABEL="straight_5m"
CMD_TOPIC="/cmd_vel_collision_checked"
ODOM_TOPIC="/wheel/odom"
LOCAL_ODOM_TOPIC="/local_state/odometry"
OUTPUT_ROOT="${PROJECT_ROOT}/reports/ranger_straight_odom_test"
PAUSE_CORRECTION="true"
CORRECTION_PAUSE_SERVICE="/robot_localization_bridge/set_correction_paused"
DISTANCE_TOLERANCE_M="0.03"
MAX_EXTRA_SEC="12.0"

usage() {
  cat <<'EOF'
Usage: run_ranger_straight_odom_test.sh [options]

Runs an automated Ranger Mini 3 straight-line odometry test. The script
publishes pure linear.x commands itself and stops when /wheel/odom reports the
target relative forward distance from this segment's start pose.

Options:
  --distance-m M          Single signed target distance. Default: 5.0
  --distances-m LIST      Comma-separated signed target distances. Overrides --distance-m.
  --linear-speed MPS      Absolute linear speed command. Default: 0.20
  --repeat N              Repeat the distance list N times. Default: 1
  --sample-hz HZ          Report sampling frequency. Default: 20.0
  --countdown-sec N       Countdown before motion. Default: 3
  --settle-sec SEC        Stop/record settle time after each segment. Default: 3.0
  --label NAME            Report label. Default: straight_5m
  --cmd-topic TOPIC       Safety-chain input topic. Default: /cmd_vel_collision_checked
  --odom-topic TOPIC      Distance feedback topic. Default: /wheel/odom
  --local-odom-topic TOPIC Local odom topic to record. Default: /local_state/odometry
  --output-root DIR       Report root. Default: reports/ranger_straight_odom_test
  --no-pause-correction   Do not call bridge correction pause service.
  --distance-tolerance-m M Stop tolerance recorded in summary. Default: 0.03
  --max-extra-sec SEC     Extra timeout beyond abs(distance)/speed. Default: 12.0

Signed distances are supported for diagnostics. Reverse motion may be rejected
unless the current runtime profile explicitly permits reverse.

The command path remains:
  test script -> /cmd_vel_collision_checked -> robot_safety -> /cmd_vel_safe
              -> ranger_mini3_mode_controller -> /cmd_vel -> ranger_base
EOF
}

while [[ "$#" -gt 0 ]]; do
  case "$1" in
    --distance-m)
      DISTANCES_M="${2:-}"
      shift 2
      ;;
    --distances-m)
      DISTANCES_M="${2:-}"
      shift 2
      ;;
    --linear-speed|--linear-speed-mps)
      LINEAR_SPEED_MPS="${2:-}"
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
    --max-extra-sec)
      MAX_EXTRA_SEC="${2:-}"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "[ranger-straight-test] unknown argument: $1" >&2
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
  echo "# Ranger Straight Odom Test Environment"
  echo "- timestamp_utc: ${timestamp}"
  echo "- distances_m: ${DISTANCES_M}"
  echo "- linear_speed_mps: ${LINEAR_SPEED_MPS}"
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
  echo "## Services"
  ros2 service list -t 2>&1 | grep -E 'robot_localization_bridge|global_localization|trigger_grid' || true
} >"${OUT_DIR}/environment.md"

if [[ "${COUNTDOWN_SEC}" != "0" ]]; then
  echo "[ranger-straight-test] motion starts in ${COUNTDOWN_SEC}s. Ensure the path is clear and E-stop is available."
  remaining="${COUNTDOWN_SEC}"
  while [[ "${remaining}" -gt 0 ]]; do
    echo "[ranger-straight-test] ${remaining}..."
    sleep 1
    remaining=$((remaining - 1))
  done
fi

set +e
python3 - \
  "${OUT_DIR}" \
  "${DISTANCES_M}" \
  "${LINEAR_SPEED_MPS}" \
  "${REPEAT}" \
  "${SAMPLE_HZ}" \
  "${SETTLE_SEC}" \
  "${CMD_TOPIC}" \
  "${ODOM_TOPIC}" \
  "${LOCAL_ODOM_TOPIC}" \
  "${PAUSE_CORRECTION}" \
  "${CORRECTION_PAUSE_SERVICE}" \
  "${DISTANCE_TOLERANCE_M}" \
  "${MAX_EXTRA_SEC}" <<'PY'
import csv
import json
import math
import os
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
from std_srvs.srv import SetBool


out_dir = sys.argv[1]
distances_m_text = sys.argv[2]
linear_speed = abs(float(sys.argv[3]))
repeat = int(sys.argv[4])
sample_hz = float(sys.argv[5])
settle_sec = float(sys.argv[6])
cmd_topic = sys.argv[7]
odom_topic = sys.argv[8]
local_odom_topic = sys.argv[9]
pause_correction = sys.argv[10].lower() == "true"
correction_pause_service = sys.argv[11]
distance_tolerance_m = abs(float(sys.argv[12]))
max_extra_sec = float(sys.argv[13])

if linear_speed <= 0.0:
    raise SystemExit("linear speed must be positive")
if repeat < 1:
    raise SystemExit("repeat must be >= 1")
if sample_hz <= 0.0:
    raise SystemExit("sample_hz must be positive")

distances_m = [float(x.strip()) for x in distances_m_text.split(",") if x.strip()]
if not distances_m:
    raise SystemExit("at least one distance is required")
if any(abs(d) <= 0.0 for d in distances_m):
    raise SystemExit("distance values must be nonzero")

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


@dataclass
class SegmentResult:
    index: int
    target_m: float
    ok: bool
    reason: str
    duration_sec: float
    wheel_forward_m: float
    wheel_lateral_m: float
    wheel_distance_m: float
    wheel_yaw_delta_rad: float
    wheel_final_error_m: float
    local_forward_m: Optional[float]
    local_lateral_m: Optional[float]
    local_distance_m: Optional[float]
    local_yaw_delta_rad: Optional[float]
    local_final_error_m: Optional[float]
    max_abs_cmd_out_vx: float
    max_abs_cmd_safe_vx: float
    safety_status: str
    mode_status: str


class StraightNode(Node):
    def __init__(self) -> None:
        super().__init__("ranger_straight_odom_test")
        qos = QoSProfile(depth=50)
        telemetry_qos = QoSProfile(depth=50)
        telemetry_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self.cmd_pub = self.create_publisher(Twist, cmd_topic, qos)
        self.pause_client = self.create_client(SetBool, correction_pause_service)
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

    def publish_cmd(self, vx: float) -> None:
        msg = Twist()
        msg.linear.x = float(vx)
        self.cmd_pub.publish(msg)

    def publish_zero_burst(self, duration: float = 1.0) -> None:
        end = time.monotonic() + duration
        while time.monotonic() < end and rclpy.ok():
            self.publish_cmd(0.0)
            rclpy.spin_once(self, timeout_sec=0.02)
            time.sleep(0.03)


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


def fmt_optional(value: Optional[float], precision: int = 6) -> str:
    if value is None:
        return ""
    return f"{value:.{precision}f}"


def run_segment(node: StraightNode, index: int, target_m: float, writer: csv.DictWriter) -> SegmentResult:
    direction = 1.0 if target_m >= 0.0 else -1.0
    target_abs = abs(target_m)
    commanded_vx = direction * linear_speed
    timeout_sec = target_abs / linear_speed + max_extra_sec

    if node.wheel_odom is None:
        return SegmentResult(index, target_m, False, "missing_initial_wheel_odom", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, None, None, None, None, None, 0.0, 0.0, node.safety_status, node.mode_status)

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
    max_abs_cmd_out_vx = 0.0
    max_abs_cmd_safe_vx = 0.0
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

        node.publish_cmd(commanded_vx)
        rclpy.spin_once(node, timeout_sec=0.01)

        if node.wheel_odom is not None:
            wheel_pose = odom_pose(node.wheel_odom)
            wheel_forward, wheel_lateral, wheel_distance, wheel_yaw_delta = relative_components(start_wheel, wheel_pose)
        else:
            wheel_pose = start_wheel

        local_pose = pose_or_none(node.local_odom)
        if local_pose is not None and start_local is not None:
            local_forward, local_lateral, local_distance, local_yaw_delta = relative_components(start_local, local_pose)

        if node.cmd_safe is not None:
            max_abs_cmd_safe_vx = max(max_abs_cmd_safe_vx, abs(node.cmd_safe.linear.x))
        if node.cmd_out is not None:
            max_abs_cmd_out_vx = max(max_abs_cmd_out_vx, abs(node.cmd_out.linear.x))
            if abs(node.cmd_out.linear.x) >= linear_speed * 0.4:
                observed_cmd_out = True

        progress = direction * wheel_forward

        if now >= next_sample:
            cmd_safe = twist_tuple(node.cmd_safe)
            cmd_out = twist_tuple(node.cmd_out)
            wheel_twist = odom_twist(node.wheel_odom)
            local_twist = odom_twist(node.local_odom)
            mode_cmd = status_cmd_out(node.mode_status)
            writer.writerow({
                "segment": index,
                "target_m": target_m,
                "elapsed_sec": f"{elapsed:.4f}",
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
                "cmd_requested_vx": f"{commanded_vx:.6f}",
                "cmd_safe_vx": fmt_optional(cmd_safe[0]),
                "cmd_out_vx": fmt_optional(cmd_out[0]),
                "mode_actual_code": status_mode_code(node.mode_status),
                "mode_cmd_out_vx": mode_cmd.get("linear_x", ""),
                "safety_status": node.safety_status,
            })
            next_sample += sample_period

        if progress >= target_abs:
            break

        time.sleep(0.005)

    node.publish_zero_burst(1.0)
    settle_until = time.monotonic() + settle_sec
    while time.monotonic() < settle_until and rclpy.ok():
        node.publish_cmd(0.0)
        rclpy.spin_once(node, timeout_sec=0.02)
        time.sleep(0.03)

    end_wheel = odom_pose(node.wheel_odom) if node.wheel_odom is not None else start_wheel
    wheel_forward, wheel_lateral, wheel_distance, wheel_yaw_delta = relative_components(start_wheel, end_wheel)
    end_local = pose_or_none(node.local_odom)
    if start_local and end_local:
        local_forward, local_lateral, local_distance, local_yaw_delta = relative_components(start_local, end_local)
    duration = time.monotonic() - start_time

    wheel_final_error = direction * wheel_forward - target_abs
    local_final_error = None if local_forward is None else direction * local_forward - target_abs
    if ok and abs(wheel_final_error) > distance_tolerance_m:
        reason = "target_reached_stop_error_gt_tolerance"
    if ok and not observed_cmd_out:
        ok = False
        reason = "final_cmd_vel_not_observed"

    return SegmentResult(
        index=index,
        target_m=target_m,
        ok=ok,
        reason=reason,
        duration_sec=duration,
        wheel_forward_m=wheel_forward,
        wheel_lateral_m=wheel_lateral,
        wheel_distance_m=wheel_distance,
        wheel_yaw_delta_rad=wheel_yaw_delta,
        wheel_final_error_m=wheel_final_error,
        local_forward_m=local_forward,
        local_lateral_m=local_lateral,
        local_distance_m=local_distance,
        local_yaw_delta_rad=local_yaw_delta,
        local_final_error_m=local_final_error,
        max_abs_cmd_out_vx=max_abs_cmd_out_vx,
        max_abs_cmd_safe_vx=max_abs_cmd_safe_vx,
        safety_status=node.safety_status,
        mode_status=node.mode_status,
    )


def main() -> int:
    rclpy.init(args=None)
    node = StraightNode()
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
                "target_m",
                "elapsed_sec",
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
                "cmd_requested_vx",
                "cmd_safe_vx",
                "cmd_out_vx",
                "mode_actual_code",
                "mode_cmd_out_vx",
                "safety_status",
            ]
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            segment_index = 0
            for _ in range(repeat):
                for distance in distances_m:
                    segment_index += 1
                    result = run_segment(node, segment_index, distance, writer)
                    results.append(result)
                    if not result.ok:
                        break
                if results and not results[-1].ok:
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
                f.write("# Ranger Straight Odom Test Summary\n\n")
                f.write(f"- distances_m: `{distances_m_text}`\n")
                f.write(f"- linear_speed_mps: `{linear_speed:.3f}`\n")
                f.write(f"- repeat: `{repeat}`\n")
                f.write(f"- cmd_topic: `{cmd_topic}`\n")
                f.write(f"- odom_topic: `{odom_topic}`\n")
                f.write(f"- distance_tolerance_m: `{distance_tolerance_m:.3f}`\n")
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
                f.write("\n| segment | target_m | ok | reason | duration_sec | wheel_forward_m | wheel_error_m | wheel_lateral_m | wheel_distance_m | wheel_yaw_deg | local_forward_m | local_error_m | local_lateral_m | local_distance_m | local_yaw_deg | max_cmd_safe_vx | max_cmd_out_vx |\n")
                f.write("|---:|---:|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|\n")
                for r in results:
                    local_forward = "" if r.local_forward_m is None else f"{r.local_forward_m:.4f}"
                    local_error = "" if r.local_final_error_m is None else f"{r.local_final_error_m:.4f}"
                    local_lateral = "" if r.local_lateral_m is None else f"{r.local_lateral_m:.4f}"
                    local_distance = "" if r.local_distance_m is None else f"{r.local_distance_m:.4f}"
                    local_yaw_deg = "" if r.local_yaw_delta_rad is None else f"{math.degrees(r.local_yaw_delta_rad):.3f}"
                    f.write(
                        f"| {r.index} | {r.target_m:.3f} | {str(r.ok).lower()} | {r.reason} | "
                        f"{r.duration_sec:.3f} | {r.wheel_forward_m:.4f} | {r.wheel_final_error_m:.4f} | "
                        f"{r.wheel_lateral_m:.4f} | {r.wheel_distance_m:.4f} | {math.degrees(r.wheel_yaw_delta_rad):.3f} | "
                        f"{local_forward} | {local_error} | {local_lateral} | {local_distance} | {local_yaw_deg} | "
                        f"{r.max_abs_cmd_safe_vx:.3f} | {r.max_abs_cmd_out_vx:.3f} |\n"
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
echo "[ranger-straight-test] report: ${OUT_DIR}"
exit "${rc}"
