#!/usr/bin/env bash
set -euo pipefail
umask 0002

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OVERLAY_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
PROJECT_ROOT="$(cd "${OVERLAY_ROOT}/../../.." && pwd)"

STRAIGHT_M="6.0"
RADIUS_M="1.50"
ANGLE_DEG="90.0"
LINEAR_SPEED_MPS="0.40"
SAMPLE_HZ="20.0"
COUNTDOWN_SEC="5"
SETTLE_SEC="3.0"
LABEL="straight6_left_r1p5_90"
CMD_TOPIC="/cmd_vel_collision_checked"
ODOM_TOPIC="/wheel/odom"
LOCAL_ODOM_TOPIC="/local_state/odometry"
OUTPUT_ROOT="${PROJECT_ROOT}/reports/ranger_composite_odom_test"
PAUSE_CORRECTION="true"
CORRECTION_PAUSE_SERVICE="/robot_localization_bridge/set_correction_paused"
DISTANCE_TOLERANCE_M="0.05"
YAW_TOLERANCE_DEG="3.0"
MAX_EXTRA_SEC="12.0"

usage() {
  cat <<'EOF'
Usage: run_ranger_straight_left_arc_odom_test.sh [options]

Runs one continuous Ranger odom isolation trajectory:
  straight forward 6m, then left Ackermann arc R=1.5m for 90deg.

Expected final pose in the starting base frame:
  x = straight + radius * sin(angle)
  y = radius * (1 - cos(angle))
  yaw = angle

For the default 6m + R1.5 90deg:
  expected x=7.5m, y=1.5m, yaw=90deg.

Options:
  --straight-m M          Forward straight distance. Default: 6.0
  --radius-m M            Left arc radius. Default: 1.50
  --angle-deg DEG         Left arc yaw target. Default: 90.0
  --linear-speed MPS      Commanded forward speed. Default: 0.40
  --sample-hz HZ          Report sampling frequency. Default: 20.0
  --countdown-sec N       Countdown before motion. Default: 5
  --settle-sec SEC        Stop/record settle time after trajectory. Default: 3.0
  --label NAME            Report label. Default: straight6_left_r1p5_90
  --cmd-topic TOPIC       Safety-chain input topic. Default: /cmd_vel_collision_checked
  --odom-topic TOPIC      Feedback odom topic. Default: /wheel/odom
  --local-odom-topic TOPIC Local odom topic to record. Default: /local_state/odometry
  --output-root DIR       Report root. Default: reports/ranger_composite_odom_test
  --no-pause-correction   Do not pause map->odom corrections during the test.
  --distance-tolerance-m M Summary pass tolerance. Default: 0.05
  --yaw-tolerance-deg DEG Summary pass tolerance. Default: 3.0
  --max-extra-sec SEC     Extra timeout beyond nominal trajectory time. Default: 12.0

The command path remains:
  test script -> /cmd_vel_collision_checked -> robot_safety -> /cmd_vel -> ranger_base
EOF
}

while [[ "$#" -gt 0 ]]; do
  case "$1" in
    --straight-m)
      STRAIGHT_M="${2:-}"
      shift 2
      ;;
    --radius-m)
      RADIUS_M="${2:-}"
      shift 2
      ;;
    --angle-deg)
      ANGLE_DEG="${2:-}"
      shift 2
      ;;
    --linear-speed|--linear-speed-mps)
      LINEAR_SPEED_MPS="${2:-}"
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
      echo "[ranger-composite-odom] unknown argument: $1" >&2
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
  echo "# Ranger Composite Odom Test Environment"
  echo "- timestamp_utc: ${timestamp}"
  echo "- straight_m: ${STRAIGHT_M}"
  echo "- radius_m: ${RADIUS_M}"
  echo "- angle_deg: ${ANGLE_DEG}"
  echo "- linear_speed_mps: ${LINEAR_SPEED_MPS}"
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
} >"${OUT_DIR}/environment.md"

if [[ "${COUNTDOWN_SEC}" != "0" ]]; then
  echo "[ranger-composite-odom] motion starts in ${COUNTDOWN_SEC}s."
  echo "[ranger-composite-odom] Required envelope: about ${STRAIGHT_M}m forward plus ${RADIUS_M}m left arc clearance."
  remaining="${COUNTDOWN_SEC}"
  while [[ "${remaining}" -gt 0 ]]; do
    echo "[ranger-composite-odom] ${remaining}..."
    sleep 1
    remaining=$((remaining - 1))
  done
fi

set +e
python3 - \
  "${OUT_DIR}" \
  "${STRAIGHT_M}" \
  "${RADIUS_M}" \
  "${ANGLE_DEG}" \
  "${LINEAR_SPEED_MPS}" \
  "${SAMPLE_HZ}" \
  "${SETTLE_SEC}" \
  "${CMD_TOPIC}" \
  "${ODOM_TOPIC}" \
  "${LOCAL_ODOM_TOPIC}" \
  "${PAUSE_CORRECTION}" \
  "${CORRECTION_PAUSE_SERVICE}" \
  "${DISTANCE_TOLERANCE_M}" \
  "${YAW_TOLERANCE_DEG}" \
  "${MAX_EXTRA_SEC}" <<'PY'
import csv
import json
import math
import os
import sys
import time
from dataclasses import dataclass
from typing import Dict, Optional, Tuple

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_srvs.srv import SetBool
import tf2_ros

out_dir = sys.argv[1]
straight_m = float(sys.argv[2])
radius_m = float(sys.argv[3])
angle_deg = float(sys.argv[4])
linear_speed = abs(float(sys.argv[5]))
sample_hz = float(sys.argv[6])
settle_sec = float(sys.argv[7])
cmd_topic = sys.argv[8]
odom_topic = sys.argv[9]
local_odom_topic = sys.argv[10]
pause_correction = sys.argv[11].lower() == "true"
correction_pause_service = sys.argv[12]
distance_tolerance_m = float(sys.argv[13])
yaw_tolerance_deg = float(sys.argv[14])
max_extra_sec = float(sys.argv[15])


def yaw_from_quat(q) -> float:
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def wrap_pi(v: float) -> float:
    while v > math.pi:
        v -= 2.0 * math.pi
    while v < -math.pi:
        v += 2.0 * math.pi
    return v


@dataclass
class Pose2:
    x: float
    y: float
    yaw: float


def odom_pose(msg: Optional[Odometry]) -> Optional[Pose2]:
    if msg is None:
        return None
    p = msg.pose.pose.position
    return Pose2(float(p.x), float(p.y), yaw_from_quat(msg.pose.pose.orientation))


def relative_pose(start: Pose2, current: Pose2) -> Pose2:
    dx = current.x - start.x
    dy = current.y - start.y
    c = math.cos(start.yaw)
    s = math.sin(start.yaw)
    return Pose2(c * dx + s * dy, -s * dx + c * dy, wrap_pi(current.yaw - start.yaw))


def pose_dict(p: Optional[Pose2]) -> Optional[Dict[str, float]]:
    if p is None:
        return None
    return {"x": p.x, "y": p.y, "yaw": p.yaw, "yaw_deg": math.degrees(p.yaw)}


def error_dict(expected: Pose2, actual: Optional[Pose2]) -> Optional[Dict[str, float]]:
    if actual is None:
        return None
    ex = actual.x - expected.x
    ey = actual.y - expected.y
    eyaw = wrap_pi(actual.yaw - expected.yaw)
    return {
        "x_error_m": ex,
        "y_error_m": ey,
        "position_error_m": math.hypot(ex, ey),
        "yaw_error_rad": eyaw,
        "yaw_error_deg": math.degrees(eyaw),
    }


class CompositeOdomTest(Node):
    def __init__(self) -> None:
        super().__init__("ranger_composite_odom_test")
        qos = QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE)
        self.cmd_pub = self.create_publisher(Twist, cmd_topic, 10)
        self.wheel: Optional[Odometry] = None
        self.local: Optional[Odometry] = None
        self.create_subscription(Odometry, odom_topic, self._wheel_cb, qos)
        self.create_subscription(Odometry, local_odom_topic, self._local_cb, qos)
        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=20.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self, spin_thread=False)
        self.pause_client = self.create_client(SetBool, correction_pause_service)
        self.rows = []

    def _wheel_cb(self, msg: Odometry) -> None:
        self.wheel = msg

    def _local_cb(self, msg: Odometry) -> None:
        self.local = msg

    def spin_for(self, sec: float) -> None:
        end = time.monotonic() + sec
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.02)

    def wait_ready(self, timeout_sec: float = 8.0) -> None:
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.wheel is not None and self.local is not None:
                return
        raise RuntimeError("timed out waiting for odom topics")

    def set_correction_pause(self, pause: bool) -> bool:
        if not pause_correction:
            return False
        if not self.pause_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warning("correction pause service unavailable")
            return False
        req = SetBool.Request()
        req.data = pause
        future = self.pause_client.call_async(req)
        deadline = time.monotonic() + 2.0
        while rclpy.ok() and time.monotonic() < deadline and not future.done():
            rclpy.spin_once(self, timeout_sec=0.05)
        return bool(future.done() and future.result() and future.result().success)

    def publish_cmd(self, x: float, z: float) -> None:
        msg = Twist()
        msg.linear.x = float(x)
        msg.angular.z = float(z)
        self.cmd_pub.publish(msg)

    def stop(self, repeats: int = 8) -> None:
        for _ in range(repeats):
            self.publish_cmd(0.0, 0.0)
            self.spin_for(0.03)

    def lookup_tf_pose(self, parent: str, child: str) -> Optional[Pose2]:
        try:
            tf = self.tf_buffer.lookup_transform(parent, child, Time())
        except Exception:
            return None
        t = tf.transform.translation
        q = tf.transform.rotation
        return Pose2(float(t.x), float(t.y), yaw_from_quat(q))

    def snapshot(self, phase: str, cmd_x: float, cmd_z: float, start: Pose2, t0: float) -> None:
        wheel_abs = odom_pose(self.wheel)
        local_abs = odom_pose(self.local)
        tf_odom_abs = self.lookup_tf_pose("odom", "base_link")
        tf_map_abs = self.lookup_tf_pose("map", "base_link")
        row = {
            "rel_time_sec": time.monotonic() - t0,
            "phase": phase,
            "cmd_x": cmd_x,
            "cmd_z": cmd_z,
        }
        for prefix, pose in (
            ("wheel", wheel_abs),
            ("local", local_abs),
            ("tf_odom_base", tf_odom_abs),
            ("tf_map_base", tf_map_abs),
        ):
            rel = relative_pose(start, pose) if pose is not None else None
            row[f"{prefix}_x"] = "" if pose is None else pose.x
            row[f"{prefix}_y"] = "" if pose is None else pose.y
            row[f"{prefix}_yaw"] = "" if pose is None else pose.yaw
            row[f"{prefix}_rel_x"] = "" if rel is None else rel.x
            row[f"{prefix}_rel_y"] = "" if rel is None else rel.y
            row[f"{prefix}_rel_yaw"] = "" if rel is None else rel.yaw
        self.rows.append(row)


def write_rows(rows) -> None:
    path = os.path.join(out_dir, "samples.csv")
    keys = []
    for row in rows:
        for key in row:
            if key not in keys:
                keys.append(key)
    with open(path, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=keys)
        writer.writeheader()
        writer.writerows(rows)


rclpy.init()
node = CompositeOdomTest()
result: Dict[str, object] = {}
try:
    node.wait_ready()
    node.spin_for(1.0)
    paused = node.set_correction_pause(True)

    start_wheel = odom_pose(node.wheel)
    start_local = odom_pose(node.local)
    if start_wheel is None or start_local is None:
        raise RuntimeError("missing odom start pose")
    start_tf_odom = node.lookup_tf_pose("odom", "base_link")
    start_tf_map = node.lookup_tf_pose("map", "base_link")

    angle_rad = math.radians(angle_deg)
    expected = Pose2(
        straight_m + radius_m * math.sin(angle_rad),
        radius_m * (1.0 - math.cos(angle_rad)),
        angle_rad,
    )
    arc_angular = linear_speed / max(radius_m, 1e-6)
    sample_period = 1.0 / max(sample_hz, 1.0)
    nominal_sec = straight_m / max(linear_speed, 1e-6) + abs(angle_rad) / max(arc_angular, 1e-6)
    deadline = time.monotonic() + nominal_sec + max_extra_sec
    t0 = time.monotonic()

    phase = "straight"
    while rclpy.ok() and time.monotonic() < deadline:
        node.publish_cmd(linear_speed, 0.0)
        node.spin_for(sample_period)
        node.snapshot(phase, linear_speed, 0.0, start_wheel, t0)
        wheel_rel = relative_pose(start_wheel, odom_pose(node.wheel))
        if wheel_rel.x >= straight_m:
            break

    node.stop()
    node.spin_for(0.25)
    arc_start = odom_pose(node.wheel)
    if arc_start is None:
        raise RuntimeError("missing arc start pose")

    phase = "left_arc"
    while rclpy.ok() and time.monotonic() < deadline:
        node.publish_cmd(linear_speed, arc_angular)
        node.spin_for(sample_period)
        node.snapshot(phase, linear_speed, arc_angular, start_wheel, t0)
        wheel_arc_rel = relative_pose(arc_start, odom_pose(node.wheel))
        if wheel_arc_rel.yaw >= angle_rad:
            break

    node.stop()
    settle_end = time.monotonic() + settle_sec
    while rclpy.ok() and time.monotonic() < settle_end:
        node.spin_for(sample_period)
        node.snapshot("settle", 0.0, 0.0, start_wheel, t0)

    final_wheel = odom_pose(node.wheel)
    final_local = odom_pose(node.local)
    final_tf_odom = node.lookup_tf_pose("odom", "base_link")
    final_tf_map = node.lookup_tf_pose("map", "base_link")
    rel_wheel = relative_pose(start_wheel, final_wheel) if final_wheel else None
    rel_local = relative_pose(start_local, final_local) if final_local else None
    rel_tf_odom = relative_pose(start_tf_odom, final_tf_odom) if start_tf_odom and final_tf_odom else None
    rel_tf_map = relative_pose(start_tf_map, final_tf_map) if start_tf_map and final_tf_map else None

    result = {
        "ok": True,
        "paused_correction": paused,
        "expected_relative_pose": pose_dict(expected),
        "start": {
            "wheel": pose_dict(start_wheel),
            "local": pose_dict(start_local),
            "tf_odom_base": pose_dict(start_tf_odom),
            "tf_map_base": pose_dict(start_tf_map),
        },
        "final_relative": {
            "wheel": pose_dict(rel_wheel),
            "local": pose_dict(rel_local),
            "tf_odom_base": pose_dict(rel_tf_odom),
            "tf_map_base": pose_dict(rel_tf_map),
        },
        "errors": {
            "wheel": error_dict(expected, rel_wheel),
            "local": error_dict(expected, rel_local),
            "tf_odom_base": error_dict(expected, rel_tf_odom),
            "tf_map_base": error_dict(expected, rel_tf_map),
        },
        "pass_thresholds": {
            "distance_tolerance_m": distance_tolerance_m,
            "yaw_tolerance_deg": yaw_tolerance_deg,
        },
    }
finally:
    try:
        node.stop()
        node.set_correction_pause(False)
    finally:
        write_rows(node.rows)
        with open(os.path.join(out_dir, "summary.json"), "w", encoding="utf-8") as f:
            json.dump(result, f, indent=2, sort_keys=True)
        with open(os.path.join(out_dir, "summary.md"), "w", encoding="utf-8") as f:
            f.write("# Ranger Composite Odom Test\n\n")
            f.write(f"- straight_m: `{straight_m}`\n")
            f.write(f"- radius_m: `{radius_m}`\n")
            f.write(f"- angle_deg: `{angle_deg}`\n")
            f.write(f"- linear_speed_mps: `{linear_speed}`\n")
            f.write(f"- cmd_topic: `{cmd_topic}`\n")
            f.write(f"- expected_relative_pose: `{pose_dict(Pose2(straight_m + radius_m * math.sin(math.radians(angle_deg)), radius_m * (1.0 - math.cos(math.radians(angle_deg))), math.radians(angle_deg)))}`\n")
            f.write("\n## Final Relative Pose\n\n")
            f.write("| source | x_m | y_m | yaw_deg | pos_error_m | yaw_error_deg |\n")
            f.write("|---|---:|---:|---:|---:|---:|\n")
            final_rel = result.get("final_relative", {}) if isinstance(result, dict) else {}
            errors = result.get("errors", {}) if isinstance(result, dict) else {}
            for key in ("wheel", "local", "tf_odom_base", "tf_map_base"):
                pose = final_rel.get(key) if isinstance(final_rel, dict) else None
                err = errors.get(key) if isinstance(errors, dict) else None
                if pose is None:
                    f.write(f"| {key} | missing | missing | missing | missing | missing |\n")
                else:
                    f.write(
                        f"| {key} | {pose['x']:.4f} | {pose['y']:.4f} | {pose['yaw_deg']:.3f} | "
                        f"{err['position_error_m']:.4f} | {err['yaw_error_deg']:.3f} |\n"
                    )
            f.write("\n## Notes\n\n")
            f.write("- Velocity is published to the safety-chain input topic, not directly to the chassis.\n")
            f.write("- Bridge corrections are paused during the run by default and restored at the end.\n")
        node.destroy_node()
        rclpy.shutdown()

if not result.get("ok", False):
    raise SystemExit(1)
PY
rc=$?
set -e

if [[ "${rc}" -ne 0 ]]; then
  echo "[ranger-composite-odom] FAIL rc=${rc} report=${OUT_DIR}" >&2
  exit "${rc}"
fi

tar -C "$(dirname "${OUT_DIR}")" -czf "${OUT_DIR}.tgz" "$(basename "${OUT_DIR}")"
echo "[ranger-composite-odom] summary: ${OUT_DIR}/summary.md"
echo "[ranger-composite-odom] archive: ${OUT_DIR}.tgz"
