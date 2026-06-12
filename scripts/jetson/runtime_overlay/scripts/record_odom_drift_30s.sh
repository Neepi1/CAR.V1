#!/usr/bin/env bash
set -euo pipefail
umask 0002

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OVERLAY_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
PROJECT_ROOT="$(cd "${OVERLAY_ROOT}/../../.." && pwd)"

DURATION_SEC=30
SAMPLE_HZ=10.0
COUNTDOWN_SEC=3
LABEL="manual_test"
OUTPUT_ROOT="${PROJECT_ROOT}/reports/odom_drift_30s"

usage() {
  cat <<'EOF'
Usage: record_odom_drift_30s.sh [--duration-sec N] [--sample-hz HZ] [--countdown-sec N] [--label NAME] [--output-root DIR]

Read-only 30 second odometry drift capture for Ranger Mini 3 navigation diagnosis.

Records:
  - /wheel/odom
  - /wheel/odom_ekf
  - /local_state/odometry
  - /localization_result
  - /motion_state and /system_state actual motion_mode
  - /ranger_mini3/desired_motion_mode
  - /ranger_mini3_mode_controller/status
  - /cmd_vel_nav, /cmd_vel_collision_checked, /cmd_vel_safe, /cmd_vel
  - TF lookups: map->odom, odom->base_link, map->base_link

Default capture duration is 30 seconds. The script does not publish velocity,
does not call localization trigger, and does not change parameters.
EOF
}

while [[ "$#" -gt 0 ]]; do
  case "$1" in
    --duration-sec)
      [[ "$#" -ge 2 ]] || { echo "[odom-drift] --duration-sec requires a value" >&2; exit 2; }
      DURATION_SEC="$2"
      shift 2
      ;;
    --sample-hz)
      [[ "$#" -ge 2 ]] || { echo "[odom-drift] --sample-hz requires a value" >&2; exit 2; }
      SAMPLE_HZ="$2"
      shift 2
      ;;
    --countdown-sec)
      [[ "$#" -ge 2 ]] || { echo "[odom-drift] --countdown-sec requires a value" >&2; exit 2; }
      COUNTDOWN_SEC="$2"
      shift 2
      ;;
    --label)
      [[ "$#" -ge 2 ]] || { echo "[odom-drift] --label requires a value" >&2; exit 2; }
      LABEL="$2"
      shift 2
      ;;
    --output-root)
      [[ "$#" -ge 2 ]] || { echo "[odom-drift] --output-root requires a value" >&2; exit 2; }
      OUTPUT_ROOT="$2"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "[odom-drift] unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

case "${DURATION_SEC}" in
  ''|*[!0-9]*) echo "[odom-drift] --duration-sec must be an integer" >&2; exit 2 ;;
esac
case "${COUNTDOWN_SEC}" in
  ''|*[!0-9]*) echo "[odom-drift] --countdown-sec must be an integer" >&2; exit 2 ;;
esac
python3 - "${SAMPLE_HZ}" <<'PY'
import sys
try:
    hz = float(sys.argv[1])
except Exception:
    sys.exit(2)
if hz <= 0.0:
    sys.exit(2)
PY
if [[ "${DURATION_SEC}" -lt 1 ]]; then
  echo "[odom-drift] --duration-sec must be >= 1" >&2
  exit 2
fi

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
  echo "# record_odom_drift_30s environment"
  echo "- timestamp_utc: ${timestamp}"
  echo "- duration_sec: ${DURATION_SEC}"
  echo "- sample_hz: ${SAMPLE_HZ}"
  echo "- countdown_sec: ${COUNTDOWN_SEC}"
  echo "- label: ${LABEL}"
  echo "- rmw: ${RMW_IMPLEMENTATION:-}"
  echo
  echo "## ROS Nodes"
  ros2 node list 2>&1 || true
  echo
  echo "## Topic Info"
  for topic in \
    /wheel/odom \
    /wheel/odom_ekf \
    /local_state/odometry \
    /localization_result \
    /motion_state \
    /system_state \
    /ranger_mini3/desired_motion_mode \
    /ranger_mini3_mode_controller/status \
    /cmd_vel_nav \
    /cmd_vel_collision_checked \
    /cmd_vel_safe \
    /cmd_vel; do
    echo "### ${topic}"
    ros2 topic info "${topic}" 2>&1 || true
  done
  echo
  echo "## robot_local_state params"
  ros2 param get /robot_local_state odom0 2>&1 || true
  ros2 param get /robot_local_state imu0 2>&1 || true
} >"${OUT_DIR}/environment.md"

if [[ "${COUNTDOWN_SEC}" -gt 0 ]]; then
  echo "[odom-drift] capture starts in ${COUNTDOWN_SEC}s. Prepare manual control now."
  while [[ "${COUNTDOWN_SEC}" -gt 0 ]]; do
    echo "[odom-drift] ${COUNTDOWN_SEC}..."
    sleep 1
    COUNTDOWN_SEC=$((COUNTDOWN_SEC - 1))
  done
fi

echo "[odom-drift] capturing ${DURATION_SEC}s at ${SAMPLE_HZ} Hz"
python3 - "${DURATION_SEC}" "${SAMPLE_HZ}" "${OUT_DIR}" <<'PY'
import csv
import json
import math
import os
import statistics
import sys
import time
from dataclasses import dataclass
from typing import Any, Dict, Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

try:
    from ranger_msgs.msg import MotionState, SystemState
except Exception:  # pragma: no cover - field runtime normally has ranger_msgs.
    MotionState = None
    SystemState = None


duration = float(sys.argv[1])
sample_hz = float(sys.argv[2])
out_dir = sys.argv[3]
sample_period = 1.0 / sample_hz


def yaw_from_quat(q: Any) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def norm_angle(value: float) -> float:
    return math.atan2(math.sin(value), math.cos(value))


def stamp_seconds(stamp: Any) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def pose_tuple_from_odom(msg: Odometry) -> Tuple[float, float, float]:
    pose = msg.pose.pose
    return (pose.position.x, pose.position.y, yaw_from_quat(pose.orientation))


def twist_tuple_from_odom(msg: Odometry) -> Tuple[float, float, float]:
    twist = msg.twist.twist
    return (twist.linear.x, twist.linear.y, twist.angular.z)


def pose_tuple_from_localization(msg: PoseWithCovarianceStamped) -> Tuple[float, float, float]:
    pose = msg.pose.pose
    return (pose.position.x, pose.position.y, yaw_from_quat(pose.orientation))


def hypot_delta(a: Optional[Tuple[float, float, float]], b: Optional[Tuple[float, float, float]]) -> str:
    if a is None or b is None:
        return "nan"
    return f"{math.hypot(b[0] - a[0], b[1] - a[1]):.6f}"


def yaw_delta(a: Optional[Tuple[float, float, float]], b: Optional[Tuple[float, float, float]]) -> str:
    if a is None or b is None:
        return "nan"
    return f"{norm_angle(b[2] - a[2]):.6f}"


@dataclass
class LatestMsg:
    msg: Any = None
    received_wall: float = 0.0

    def present(self) -> bool:
        return self.msg is not None


class CaptureNode(Node):
    def __init__(self) -> None:
        super().__init__("record_odom_drift_30s")
        qos = QoSProfile(depth=50)
        telemetry_qos = QoSProfile(depth=50)
        telemetry_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.latest: Dict[str, LatestMsg] = {
            "wheel_odom": LatestMsg(),
            "wheel_odom_ekf": LatestMsg(),
            "local_state": LatestMsg(),
            "localization": LatestMsg(),
            "desired_mode": LatestMsg(),
            "mode_status": LatestMsg(),
            "cmd_vel_nav": LatestMsg(),
            "cmd_vel_collision_checked": LatestMsg(),
            "cmd_vel_safe": LatestMsg(),
            "cmd_vel": LatestMsg(),
            "motion_state": LatestMsg(),
            "system_state": LatestMsg(),
        }

        self.create_subscription(Odometry, "/wheel/odom", self._cb("wheel_odom"), telemetry_qos)
        self.create_subscription(Odometry, "/wheel/odom_ekf", self._cb("wheel_odom_ekf"), telemetry_qos)
        self.create_subscription(Odometry, "/local_state/odometry", self._cb("local_state"), telemetry_qos)
        self.create_subscription(PoseWithCovarianceStamped, "/localization_result", self._cb("localization"), telemetry_qos)
        self.create_subscription(String, "/ranger_mini3/desired_motion_mode", self._cb("desired_mode"), qos)
        self.create_subscription(String, "/ranger_mini3_mode_controller/status", self._cb("mode_status"), qos)
        self.create_subscription(Twist, "/cmd_vel_nav", self._cb("cmd_vel_nav"), qos)
        self.create_subscription(Twist, "/cmd_vel_collision_checked", self._cb("cmd_vel_collision_checked"), qos)
        self.create_subscription(Twist, "/cmd_vel_safe", self._cb("cmd_vel_safe"), qos)
        self.create_subscription(Twist, "/cmd_vel", self._cb("cmd_vel"), qos)
        if MotionState is not None:
            self.create_subscription(MotionState, "/motion_state", self._cb("motion_state"), telemetry_qos)
        if SystemState is not None:
            self.create_subscription(SystemState, "/system_state", self._cb("system_state"), telemetry_qos)

    def _cb(self, name: str):
        def callback(msg: Any) -> None:
            self.latest[name] = LatestMsg(msg=msg, received_wall=time.time())
        return callback

    def now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def age_from_header(self, msg: Any) -> str:
        try:
            stamp = stamp_seconds(msg.header.stamp)
        except Exception:
            return "nan"
        if stamp <= 0.0:
            return "nan"
        return f"{self.now_sec() - stamp:.6f}"

    def lookup_tf(self, target: str, source: str) -> Optional[Tuple[float, float, float]]:
        try:
            tf = self.tf_buffer.lookup_transform(target, source, rclpy.time.Time())
        except TransformException:
            return None
        tr = tf.transform.translation
        rot = tf.transform.rotation
        return (tr.x, tr.y, yaw_from_quat(rot))


def odom_fields(node: CaptureNode, name: str) -> Dict[str, Any]:
    latest = node.latest[name]
    if not latest.present():
        return {f"{name}_{k}": "" for k in ("x", "y", "yaw", "vx", "vy", "vyaw", "age_sec")}
    x, y, yaw = pose_tuple_from_odom(latest.msg)
    vx, vy, vyaw = twist_tuple_from_odom(latest.msg)
    return {
        f"{name}_x": x,
        f"{name}_y": y,
        f"{name}_yaw": yaw,
        f"{name}_vx": vx,
        f"{name}_vy": vy,
        f"{name}_vyaw": vyaw,
        f"{name}_age_sec": node.age_from_header(latest.msg),
    }


def localization_fields(node: CaptureNode) -> Dict[str, Any]:
    latest = node.latest["localization"]
    if not latest.present():
        return {f"localization_{k}": "" for k in ("x", "y", "yaw", "age_sec")}
    x, y, yaw = pose_tuple_from_localization(latest.msg)
    return {
        "localization_x": x,
        "localization_y": y,
        "localization_yaw": yaw,
        "localization_age_sec": node.age_from_header(latest.msg),
    }


def tf_fields(name: str, value: Optional[Tuple[float, float, float]]) -> Dict[str, Any]:
    if value is None:
        return {f"{name}_{k}": "" for k in ("x", "y", "yaw")}
    return {f"{name}_x": value[0], f"{name}_y": value[1], f"{name}_yaw": value[2]}


def twist_fields(node: CaptureNode, name: str) -> Dict[str, Any]:
    latest = node.latest[name]
    if not latest.present():
        return {f"{name}_{k}": "" for k in ("vx", "vy", "wz")}
    msg = latest.msg
    return {f"{name}_vx": msg.linear.x, f"{name}_vy": msg.linear.y, f"{name}_wz": msg.angular.z}


def raw_string(node: CaptureNode, name: str) -> str:
    latest = node.latest[name]
    if not latest.present():
        return ""
    data = getattr(latest.msg, "data", "")
    return str(data).replace("\n", " ")


def motion_mode_value(node: CaptureNode, name: str) -> str:
    latest = node.latest[name]
    if not latest.present():
        return ""
    return str(getattr(latest.msg, "motion_mode", ""))


def pose_from_row(row: Dict[str, Any], prefix: str) -> Optional[Tuple[float, float, float]]:
    try:
        x = row[f"{prefix}_x"]
        y = row[f"{prefix}_y"]
        yaw = row[f"{prefix}_yaw"]
        if x == "" or y == "" or yaw == "":
            return None
        return (float(x), float(y), float(yaw))
    except Exception:
        return None


def max_abs(values):
    numeric = [abs(v) for v in values if isinstance(v, (int, float)) and math.isfinite(v)]
    return max(numeric) if numeric else float("nan")


rclpy.init()
node = CaptureNode()
rows = []
start_wall = time.time()
deadline = start_wall + duration
next_sample = start_wall
print_every = max(1, int(sample_hz * 5.0))
sample_index = 0

try:
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.02)
        now = time.time()
        if now < next_sample:
            continue

        map_odom = node.lookup_tf("map", "odom")
        odom_base = node.lookup_tf("odom", "base_link")
        map_base = node.lookup_tf("map", "base_link")

        row: Dict[str, Any] = {
            "sample_index": sample_index,
            "elapsed_sec": now - start_wall,
            "ros_time_sec": node.now_sec(),
        }
        row.update(odom_fields(node, "wheel_odom"))
        row.update(odom_fields(node, "wheel_odom_ekf"))
        row.update(odom_fields(node, "local_state"))
        row.update(localization_fields(node))
        row.update(tf_fields("tf_map_odom", map_odom))
        row.update(tf_fields("tf_odom_base", odom_base))
        row.update(tf_fields("tf_map_base", map_base))
        row.update(twist_fields(node, "cmd_vel_nav"))
        row.update(twist_fields(node, "cmd_vel_collision_checked"))
        row.update(twist_fields(node, "cmd_vel_safe"))
        row.update(twist_fields(node, "cmd_vel"))
        row["actual_motion_mode"] = motion_mode_value(node, "motion_state")
        row["system_motion_mode"] = motion_mode_value(node, "system_state")
        row["desired_motion_mode_raw"] = raw_string(node, "desired_mode")
        row["mode_controller_status_raw"] = raw_string(node, "mode_status")
        rows.append(row)

        sample_index += 1
        if sample_index % print_every == 0:
            print(f"[odom-drift] sampled {sample_index} rows, elapsed={now - start_wall:.1f}s", flush=True)
        next_sample += sample_period
finally:
    node.destroy_node()
    rclpy.shutdown()

csv_path = os.path.join(out_dir, "samples.csv")
if rows:
    fields = list(rows[0].keys())
    with open(csv_path, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)
else:
    with open(csv_path, "w", encoding="utf-8") as f:
        f.write("")

first = rows[0] if rows else {}
last = rows[-1] if rows else {}


def first_last_pose(prefix: str):
    valid = [(row, pose_from_row(row, prefix)) for row in rows if pose_from_row(row, prefix) is not None]
    if not valid:
        return None, None
    return valid[0][1], valid[-1][1]


def last_pose(prefix: str):
    for row in reversed(rows):
        value = pose_from_row(row, prefix)
        if value is not None:
            return value
    return None


def last_pair(prefix_a: str, prefix_b: str):
    for row in reversed(rows):
        a = pose_from_row(row, prefix_a)
        b = pose_from_row(row, prefix_b)
        if a is not None and b is not None:
            return a, b
    return None, None


wheel_odom_first, wheel_odom_last = first_last_pose("wheel_odom")
wheel_odom_ekf_first, wheel_odom_ekf_last = first_last_pose("wheel_odom_ekf")
local_state_first, local_state_last = first_last_pose("local_state")
tf_map_odom_first, tf_map_odom_last = first_last_pose("tf_map_odom")
tf_odom_base_first, tf_odom_base_last = first_last_pose("tf_odom_base")
tf_map_base_first, tf_map_base_last = first_last_pose("tf_map_base")
final_map_base, final_localization = last_pair("tf_map_base", "localization")

summary: Dict[str, Any] = {
    "duration_sec": duration,
    "sample_hz": sample_hz,
    "sample_count": len(rows),
    "wheel_odom_delta_m": hypot_delta(wheel_odom_first, wheel_odom_last),
    "wheel_odom_delta_yaw_rad": yaw_delta(wheel_odom_first, wheel_odom_last),
    "wheel_odom_ekf_delta_m": hypot_delta(wheel_odom_ekf_first, wheel_odom_ekf_last),
    "wheel_odom_ekf_delta_yaw_rad": yaw_delta(wheel_odom_ekf_first, wheel_odom_ekf_last),
    "local_state_delta_m": hypot_delta(local_state_first, local_state_last),
    "local_state_delta_yaw_rad": yaw_delta(local_state_first, local_state_last),
    "tf_map_odom_delta_m": hypot_delta(tf_map_odom_first, tf_map_odom_last),
    "tf_map_odom_delta_yaw_rad": yaw_delta(tf_map_odom_first, tf_map_odom_last),
    "tf_odom_base_delta_m": hypot_delta(tf_odom_base_first, tf_odom_base_last),
    "tf_odom_base_delta_yaw_rad": yaw_delta(tf_odom_base_first, tf_odom_base_last),
    "tf_map_base_delta_m": hypot_delta(tf_map_base_first, tf_map_base_last),
    "tf_map_base_delta_yaw_rad": yaw_delta(tf_map_base_first, tf_map_base_last),
}

summary["final_localization_vs_tf_map_base_m"] = hypot_delta(final_map_base, final_localization)
summary["final_localization_vs_tf_map_base_yaw_rad"] = yaw_delta(final_map_base, final_localization)
summary["localization_sample_count"] = sum(1 for row in rows if pose_from_row(row, "localization") is not None)
summary["tf_map_base_sample_count"] = sum(1 for row in rows if pose_from_row(row, "tf_map_base") is not None)

for cmd_name in ("cmd_vel_nav", "cmd_vel_collision_checked", "cmd_vel_safe", "cmd_vel"):
    summary[f"{cmd_name}_max_abs_vx"] = f"{max_abs([float(r[f'{cmd_name}_vx']) for r in rows if r.get(f'{cmd_name}_vx') not in ('', None)]):.6f}"
    summary[f"{cmd_name}_max_abs_wz"] = f"{max_abs([float(r[f'{cmd_name}_wz']) for r in rows if r.get(f'{cmd_name}_wz') not in ('', None)]):.6f}"

actual_modes = sorted(set(str(r.get("actual_motion_mode", "")) for r in rows if str(r.get("actual_motion_mode", "")) != ""))
system_modes = sorted(set(str(r.get("system_motion_mode", "")) for r in rows if str(r.get("system_motion_mode", "")) != ""))
summary["actual_motion_modes_seen"] = ",".join(actual_modes)
summary["system_motion_modes_seen"] = ",".join(system_modes)

summary_json_path = os.path.join(out_dir, "summary.json")
with open(summary_json_path, "w", encoding="utf-8") as f:
    json.dump(summary, f, indent=2, sort_keys=True)
    f.write("\n")

summary_md_path = os.path.join(out_dir, "summary.md")
with open(summary_md_path, "w", encoding="utf-8") as f:
    f.write("# 30s Odometry Drift Capture\n\n")
    f.write("This is a read-only capture. It does not publish velocity, trigger localization, or change parameters.\n\n")
    f.write("## Key Results\n\n")
    for key in (
        "duration_sec",
        "sample_count",
        "wheel_odom_delta_m",
        "wheel_odom_delta_yaw_rad",
        "wheel_odom_ekf_delta_m",
        "wheel_odom_ekf_delta_yaw_rad",
        "local_state_delta_m",
        "local_state_delta_yaw_rad",
        "tf_map_odom_delta_m",
        "tf_map_odom_delta_yaw_rad",
        "tf_map_base_delta_m",
        "tf_map_base_delta_yaw_rad",
        "final_localization_vs_tf_map_base_m",
        "final_localization_vs_tf_map_base_yaw_rad",
        "actual_motion_modes_seen",
        "system_motion_modes_seen",
    ):
        f.write(f"- {key}: `{summary.get(key, '')}`\n")
    f.write("\n## Command Maxima\n\n")
    for cmd_name in ("cmd_vel_nav", "cmd_vel_collision_checked", "cmd_vel_safe", "cmd_vel"):
        f.write(f"- {cmd_name}_max_abs_vx: `{summary.get(cmd_name + '_max_abs_vx', '')}`\n")
        f.write(f"- {cmd_name}_max_abs_wz: `{summary.get(cmd_name + '_max_abs_wz', '')}`\n")
    f.write("\n## Interpretation Hints\n\n")
    f.write("- For an in-place spin test, `wheel_odom_ekf_delta_m` and `local_state_delta_m` should stay small while yaw changes.\n")
    f.write("- If `final_localization_vs_tf_map_base_m` is large, global localization and the TF pose disagree at the end of the capture.\n")
    f.write("- If `tf_map_odom_delta_m` stays near zero while localization disagrees with `tf_map_base`, the bridge is likely holding or rejecting the correction.\n")
    f.write("- During pure yaw/final align, actual motion mode should include AgileX SPINNING=2. DUAL_ACKERMAN=0 during spin is suspect.\n")
    f.write("\n## Files\n\n")
    f.write("- `samples.csv`\n")
    f.write("- `summary.json`\n")
    f.write("- `environment.md`\n")

print(f"[odom-drift] wrote {csv_path}")
print(f"[odom-drift] wrote {summary_md_path}")
PY

echo "[odom-drift] complete: ${OUT_DIR}"
echo "[odom-drift] summary: ${OUT_DIR}/summary.md"
