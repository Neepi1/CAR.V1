#!/usr/bin/env bash
set -euo pipefail
umask 0002

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OVERLAY_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
PROJECT_ROOT="$(cd "${OVERLAY_ROOT}/../../.." && pwd)"

ANGLES_DEG="30,-30"
ANGULAR_SPEED_RADPS="0.20"
REPEAT="1"
SAMPLE_HZ="20.0"
COUNTDOWN_SEC="3"
SETTLE_SEC="3.0"
LABEL="spin_smoke"
CMD_TOPIC="/cmd_vel_collision_checked"
ODOM_TOPIC="/wheel/odom"
LOCAL_ODOM_TOPIC="/local_state/odometry"
OUTPUT_ROOT="${PROJECT_ROOT}/reports/ranger_spin_odom_test"
PAUSE_CORRECTION="true"
CORRECTION_PAUSE_SERVICE="/robot_localization_bridge/set_correction_paused"
ANGLE_TOLERANCE_DEG="1.0"
MAX_EXTRA_SEC="8.0"
FEEDBACK_MAX_AGE_SEC="0.20"

usage() {
  cat <<'EOF'
Usage: run_ranger_spin_odom_test.sh [options]

Runs an automated Ranger Mini 3 spin odometry test. The script publishes pure
angular.z commands itself and stops by integrating yaw from /wheel/odom.

Options:
  --angles-deg LIST       Comma-separated signed target angles. Default: 30,-30
  --angular-speed RADPS   Absolute angular speed command. Default: 0.20
  --repeat N              Repeat the angle list N times. Default: 1
  --sample-hz HZ          Report sampling frequency. Default: 20.0
  --countdown-sec N       Countdown before motion. Default: 3
  --settle-sec SEC        Stop/record settle time after each spin. Default: 3.0
  --label NAME            Report label. Default: spin_smoke
  --cmd-topic TOPIC       Safety-chain input topic. Default: /cmd_vel_collision_checked
  --odom-topic TOPIC      Yaw feedback topic. Default: /wheel/odom
  --local-odom-topic TOPIC Local odom topic to record. Default: /local_state/odometry
  --output-root DIR       Report root. Default: reports/ranger_spin_odom_test
  --no-pause-correction   Do not call bridge correction pause service.
  --angle-tolerance-deg N Stop tolerance. Default: 1.0
  --max-extra-sec SEC     Extra timeout beyond target/speed. Default: 8.0
  --feedback-max-age-sec SEC Abort if wheel odom is older than this. Default: 0.20

The command path remains:
  test script -> /cmd_vel_collision_checked -> robot_safety -> /cmd_vel -> ranger_base
  /cmd_vel_safe is a robot_safety diagnostic mirror.
EOF
}

while [[ "$#" -gt 0 ]]; do
  case "$1" in
    --angles-deg)
      ANGLES_DEG="${2:-}"
      shift 2
      ;;
    --angular-speed)
      ANGULAR_SPEED_RADPS="${2:-}"
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
    --angle-tolerance-deg)
      ANGLE_TOLERANCE_DEG="${2:-}"
      shift 2
      ;;
    --max-extra-sec)
      MAX_EXTRA_SEC="${2:-}"
      shift 2
      ;;
    --feedback-max-age-sec)
      FEEDBACK_MAX_AGE_SEC="${2:-}"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "[ranger-spin-test] unknown argument: $1" >&2
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
  echo "# Ranger Spin Odom Test Environment"
  echo "- timestamp_utc: ${timestamp}"
  echo "- angles_deg: ${ANGLES_DEG}"
  echo "- angular_speed_radps: ${ANGULAR_SPEED_RADPS}"
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
  echo "- feedback_max_age_sec: ${FEEDBACK_MAX_AGE_SEC}"
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
    /localization/bridge_status; do
    echo "### ${topic}"
    ros2 topic info "${topic}" -v 2>&1 || true
  done
  echo
  echo "## Services"
  ros2 service list -t 2>&1 | grep -E 'robot_localization_bridge|global_localization|trigger_grid' || true
} >"${OUT_DIR}/environment.md"

if [[ "${COUNTDOWN_SEC}" != "0" ]]; then
  echo "[ranger-spin-test] motion starts in ${COUNTDOWN_SEC}s. Ensure the robot has clearance and E-stop is available."
  remaining="${COUNTDOWN_SEC}"
  while [[ "${remaining}" -gt 0 ]]; do
    echo "[ranger-spin-test] ${remaining}..."
    sleep 1
    remaining=$((remaining - 1))
  done
fi

set +e
python3 - \
  "${OUT_DIR}" \
  "${ANGLES_DEG}" \
  "${ANGULAR_SPEED_RADPS}" \
  "${REPEAT}" \
  "${SAMPLE_HZ}" \
  "${SETTLE_SEC}" \
  "${CMD_TOPIC}" \
  "${ODOM_TOPIC}" \
  "${LOCAL_ODOM_TOPIC}" \
  "${PAUSE_CORRECTION}" \
  "${CORRECTION_PAUSE_SERVICE}" \
  "${ANGLE_TOLERANCE_DEG}" \
  "${MAX_EXTRA_SEC}" \
  "${FEEDBACK_MAX_AGE_SEC}" <<'PY'
import csv
import json
import math
import os
import sys
import threading
import time
from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Tuple

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import SetBool


out_dir = sys.argv[1]
angles_deg_text = sys.argv[2]
angular_speed = abs(float(sys.argv[3]))
repeat = int(sys.argv[4])
sample_hz = float(sys.argv[5])
settle_sec = float(sys.argv[6])
cmd_topic = sys.argv[7]
odom_topic = sys.argv[8]
local_odom_topic = sys.argv[9]
pause_correction = sys.argv[10].lower() == "true"
correction_pause_service = sys.argv[11]
angle_tolerance = math.radians(abs(float(sys.argv[12])))
max_extra_sec = float(sys.argv[13])
feedback_max_age_sec = float(sys.argv[14])

if angular_speed <= 0.0:
    raise SystemExit("angular speed must be positive")
if repeat < 1:
    raise SystemExit("repeat must be >= 1")
if sample_hz <= 0.0:
    raise SystemExit("sample_hz must be positive")
if feedback_max_age_sec <= 0.0:
    raise SystemExit("feedback_max_age_sec must be positive")

angles_deg = [float(x.strip()) for x in angles_deg_text.split(",") if x.strip()]
if not angles_deg:
    raise SystemExit("at least one angle is required")

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


def twist_tuple(msg: Twist) -> Tuple[float, float, float]:
    return (msg.linear.x, msg.linear.y, msg.angular.z)


@dataclass
class SegmentResult:
    index: int
    target_deg: float
    ok: bool
    reason: str
    duration_sec: float
    wheel_yaw_delta_rad: float
    local_yaw_delta_rad: Optional[float]
    wheel_xy_drift_m: float
    local_xy_drift_m: Optional[float]
    max_abs_cmd_out_wz: float
    max_abs_cmd_safe_wz: float
    safety_status: str
    mode_status: str


class SpinNode(Node):
    def __init__(self) -> None:
        super().__init__("ranger_spin_odom_test")
        qos = QoSProfile(depth=1)
        telemetry_qos = QoSProfile(depth=1)
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
        self.wheel_odom_received_at: Optional[float] = None
        self.create_subscription(Odometry, odom_topic, self._wheel_cb, telemetry_qos)
        self.create_subscription(Odometry, local_odom_topic, self._local_cb, telemetry_qos)
        self.create_subscription(Twist, "/cmd_vel_safe", self._cmd_safe_cb, telemetry_qos)
        self.create_subscription(Twist, "/cmd_vel", self._cmd_out_cb, telemetry_qos)
        self.create_subscription(String, "/safety/status", self._safety_cb, qos)
        self.create_subscription(String, "/ranger_mini3_mode_controller/status", self._mode_cb, qos)
        self.create_subscription(String, "/localization/bridge_status", self._bridge_cb, qos)

    def _wheel_cb(self, msg: Odometry) -> None:
        self.wheel_odom = msg
        self.wheel_odom_received_at = time.monotonic()

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
            time.sleep(0.01)

    def wheel_odom_age_sec(self, now: Optional[float] = None) -> float:
        if self.wheel_odom_received_at is None:
            return math.inf
        current = time.monotonic() if now is None else now
        return max(0.0, current - self.wheel_odom_received_at)

    def wheel_odom_is_fresh(self, now: Optional[float] = None) -> bool:
        return self.wheel_odom is not None and self.wheel_odom_age_sec(now) <= feedback_max_age_sec

    def wait_for_odom(self, timeout_sec: float = 5.0) -> bool:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline and rclpy.ok():
            if self.wheel_odom_is_fresh():
                return True
            time.sleep(0.01)
        return False

    def set_correction_pause(self, paused: bool) -> str:
        if not pause_correction:
            return "skipped"
        if not self.pause_client.wait_for_service(timeout_sec=2.0):
            return "service_unavailable"
        req = SetBool.Request()
        req.data = paused
        future = self.pause_client.call_async(req)
        deadline = time.monotonic() + 4.0
        while not future.done() and time.monotonic() < deadline and rclpy.ok():
            time.sleep(0.01)
        if not future.done():
            return "timeout"
        result = future.result()
        return f"success={result.success} message={result.message}"

    def publish_cmd(self, wz: float) -> None:
        msg = Twist()
        msg.angular.z = float(wz)
        self.cmd_pub.publish(msg)

    def publish_zero_burst(self, duration: float = 1.0) -> None:
        end = time.monotonic() + duration
        while time.monotonic() < end and rclpy.ok():
            self.publish_cmd(0.0)
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


def xy_dist(a: Tuple[float, float, float], b: Tuple[float, float, float]) -> float:
    return math.hypot(b[0] - a[0], b[1] - a[1])


def run_segment(node: SpinNode, index: int, target_deg: float, writer: csv.DictWriter) -> SegmentResult:
    target = math.radians(target_deg)
    direction = 1.0 if target >= 0.0 else -1.0
    target_abs = abs(target)
    commanded_wz = direction * angular_speed
    timeout_sec = target_abs / angular_speed + max_extra_sec

    if not node.wheel_odom_is_fresh():
        reason = "missing_initial_wheel_odom" if node.wheel_odom is None else "stale_initial_wheel_odom"
        return SegmentResult(index, target_deg, False, reason, 0.0, 0.0, None, 0.0, None, 0.0, 0.0, node.safety_status, node.mode_status)

    start_wheel = odom_pose(node.wheel_odom)
    start_local = pose_or_none(node.local_odom)
    last_yaw = start_wheel[2]
    wheel_yaw_accum = 0.0
    local_yaw_accum = 0.0
    last_local_yaw = start_local[2] if start_local else None
    max_abs_cmd_out_wz = 0.0
    max_abs_cmd_safe_wz = 0.0
    observed_cmd_out = False
    observed_spinning = False
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

        wheel_odom_age = node.wheel_odom_age_sec(now)
        if wheel_odom_age > feedback_max_age_sec:
            ok = False
            reason = f"wheel_odom_stale_age_{wheel_odom_age:.3f}s"
            break

        node.publish_cmd(commanded_wz)

        if node.wheel_odom is not None:
            wheel_pose = odom_pose(node.wheel_odom)
            delta = norm_angle(wheel_pose[2] - last_yaw)
            wheel_yaw_accum += delta
            last_yaw = wheel_pose[2]
        else:
            wheel_pose = start_wheel

        local_pose = pose_or_none(node.local_odom)
        if local_pose is not None and last_local_yaw is not None:
            local_delta = norm_angle(local_pose[2] - last_local_yaw)
            local_yaw_accum += local_delta
            last_local_yaw = local_pose[2]

        if node.cmd_safe is not None:
            max_abs_cmd_safe_wz = max(max_abs_cmd_safe_wz, abs(node.cmd_safe.angular.z))
        if node.cmd_out is not None:
            max_abs_cmd_out_wz = max(max_abs_cmd_out_wz, abs(node.cmd_out.angular.z))
            if abs(node.cmd_out.angular.z) >= angular_speed * 0.5:
                observed_cmd_out = True

        if status_mode_code(node.mode_status) == "2":
            observed_spinning = True

        if now >= next_sample:
            cmd_safe = twist_tuple(node.cmd_safe) if node.cmd_safe is not None else ("", "", "")
            cmd_out = twist_tuple(node.cmd_out) if node.cmd_out is not None else ("", "", "")
            mode_cmd = status_cmd_out(node.mode_status)
            writer.writerow({
                "segment": index,
                "target_deg": target_deg,
                "elapsed_sec": f"{elapsed:.4f}",
                "wheel_x": f"{wheel_pose[0]:.6f}",
                "wheel_y": f"{wheel_pose[1]:.6f}",
                "wheel_yaw": f"{wheel_pose[2]:.6f}",
                "wheel_yaw_accum": f"{wheel_yaw_accum:.6f}",
                "wheel_odom_receive_age_sec": f"{wheel_odom_age:.6f}",
                "local_x": "" if local_pose is None else f"{local_pose[0]:.6f}",
                "local_y": "" if local_pose is None else f"{local_pose[1]:.6f}",
                "local_yaw": "" if local_pose is None else f"{local_pose[2]:.6f}",
                "local_yaw_accum": "" if local_pose is None else f"{local_yaw_accum:.6f}",
                "cmd_requested_wz": f"{commanded_wz:.6f}",
                "cmd_safe_wz": "" if cmd_safe[2] == "" else f"{cmd_safe[2]:.6f}",
                "cmd_out_wz": "" if cmd_out[2] == "" else f"{cmd_out[2]:.6f}",
                "mode_actual_code": status_mode_code(node.mode_status),
                "mode_cmd_out_wz": mode_cmd.get("angular_z", ""),
                "safety_status": node.safety_status,
            })
            next_sample += sample_period

        if abs(wheel_yaw_accum) >= max(0.0, target_abs - angle_tolerance):
            break

        time.sleep(0.005)

    node.publish_zero_burst(1.0)
    settle_until = time.monotonic() + settle_sec
    while time.monotonic() < settle_until and rclpy.ok():
        node.publish_cmd(0.0)
        time.sleep(0.03)

    end_wheel = odom_pose(node.wheel_odom) if node.wheel_odom is not None else start_wheel
    end_local = pose_or_none(node.local_odom)
    wheel_xy_drift = xy_dist(start_wheel, end_wheel)
    local_xy_drift = xy_dist(start_local, end_local) if start_local and end_local else None
    duration = time.monotonic() - start_time

    if ok and not observed_cmd_out:
        ok = False
        reason = "final_cmd_vel_not_observed"
    if ok and not observed_spinning:
        ok = False
        reason = "spinning_mode_not_observed"

    return SegmentResult(
        index=index,
        target_deg=target_deg,
        ok=ok,
        reason=reason,
        duration_sec=duration,
        wheel_yaw_delta_rad=wheel_yaw_accum,
        local_yaw_delta_rad=local_yaw_accum if start_local and end_local else None,
        wheel_xy_drift_m=wheel_xy_drift,
        local_xy_drift_m=local_xy_drift,
        max_abs_cmd_out_wz=max_abs_cmd_out_wz,
        max_abs_cmd_safe_wz=max_abs_cmd_safe_wz,
        safety_status=node.safety_status,
        mode_status=node.mode_status,
    )


def main() -> int:
    rclpy.init(args=None)
    node = SpinNode()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor_thread = threading.Thread(
        target=executor.spin,
        name="ranger_spin_test_callbacks",
        daemon=True,
    )
    executor_thread.start()
    results: List[SegmentResult] = []
    pause_enable_result = "not_called"
    pause_disable_result = "not_called"
    try:
        if not node.wait_for_odom():
            raise RuntimeError(f"no odometry received on {odom_topic}")

        pause_enable_result = node.set_correction_pause(True)
        node.spin_some(0.5)

        samples_path = os.path.join(out_dir, "samples.csv")
        with open(samples_path, "w", newline="", encoding="utf-8") as f:
            fieldnames = [
                "segment",
                "target_deg",
                "elapsed_sec",
                "wheel_x",
                "wheel_y",
                "wheel_yaw",
                "wheel_yaw_accum",
                "wheel_odom_receive_age_sec",
                "local_x",
                "local_y",
                "local_yaw",
                "local_yaw_accum",
                "cmd_requested_wz",
                "cmd_safe_wz",
                "cmd_out_wz",
                "mode_actual_code",
                "mode_cmd_out_wz",
                "safety_status",
            ]
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            segment_index = 0
            for _ in range(repeat):
                for angle in angles_deg:
                    segment_index += 1
                    result = run_segment(node, segment_index, angle, writer)
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
        except Exception as exc:
            pause_disable_result = f"error={exc}"
        try:
            bridge_status = node.bridge_status
            with open(os.path.join(out_dir, "summary.md"), "w", encoding="utf-8") as f:
                f.write("# Ranger Spin Odom Test Summary\n\n")
                f.write(f"- angles_deg: `{angles_deg_text}`\n")
                f.write(f"- angular_speed_radps: `{angular_speed:.3f}`\n")
                f.write(f"- repeat: `{repeat}`\n")
                f.write(f"- cmd_topic: `{cmd_topic}`\n")
                f.write(f"- odom_topic: `{odom_topic}`\n")
                f.write(f"- feedback_max_age_sec: `{feedback_max_age_sec:.3f}`\n")
                f.write(f"- pause_correction_enable: `{pause_enable_result}`\n")
                f.write(f"- pause_correction_disable: `{pause_disable_result}`\n")
                if bridge_status:
                    try:
                        bridge = json.loads(bridge_status)
                        f.write(f"- bridge_amcl_gate_mode: `{bridge.get('amcl_gate_mode', '')}`\n")
                        f.write(f"- bridge_correction_paused_final: `{bridge.get('map_odom_correction_paused', '')}`\n")
                        f.write(f"- bridge_has_map_to_odom: `{bridge.get('has_map_to_odom', '')}`\n")
                    except Exception:
                        f.write("- bridge_status_parse: `failed`\n")
                f.write("\n| segment | target_deg | ok | reason | duration_sec | wheel_yaw_deg | local_yaw_deg | wheel_xy_drift_m | local_xy_drift_m | max_cmd_safe_wz | max_cmd_out_wz |\n")
                f.write("|---:|---:|---|---|---:|---:|---:|---:|---:|---:|---:|\n")
                for r in results:
                    local_yaw_deg = "" if r.local_yaw_delta_rad is None else f"{math.degrees(r.local_yaw_delta_rad):.3f}"
                    local_xy = "" if r.local_xy_drift_m is None else f"{r.local_xy_drift_m:.4f}"
                    f.write(
                        f"| {r.index} | {r.target_deg:.3f} | {str(r.ok).lower()} | {r.reason} | "
                        f"{r.duration_sec:.3f} | {math.degrees(r.wheel_yaw_delta_rad):.3f} | "
                        f"{local_yaw_deg} | {r.wheel_xy_drift_m:.4f} | {local_xy} | "
                        f"{r.max_abs_cmd_safe_wz:.3f} | {r.max_abs_cmd_out_wz:.3f} |\n"
                    )
                if results:
                    f.write("\n## Last Safety Status\n\n")
                    f.write(f"`{results[-1].safety_status}`\n\n")
                    f.write("## Last Mode Controller Status\n\n")
                    f.write("```json\n")
                    f.write(results[-1].mode_status)
                    f.write("\n```\n")
        finally:
            executor.shutdown(timeout_sec=2.0)
            executor_thread.join(timeout=2.0)
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
PY

rc=$?
set -e
echo "[ranger-spin-test] report: ${OUT_DIR}"
exit "${rc}"
