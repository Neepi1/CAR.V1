#!/usr/bin/env bash
set -euo pipefail
umask 0002

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OVERLAY_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
PROJECT_ROOT="$(cd "${OVERLAY_ROOT}/../../.." && pwd)"

API_URL="${API_URL:-http://127.0.0.1:8080}"
POSE_ID=""
BUILDING_ID=""
FLOOR_ID=""
TIMEOUT_SEC="180"
POLL_PERIOD_SEC="1.0"
SETTLE_SEC="3.0"
SAMPLE_PERIOD_SEC="0.10"
POST_CAPTURE_SEC="3.0"
LABEL="terminal_yaw_trace"
OUTPUT_ROOT="${PROJECT_ROOT}/reports/navigation_terminal_yaw_trace"
POST_RELOCALIZE="true"

usage() {
  cat <<'EOF'
Usage: run_navigation_terminal_yaw_trace.sh --pose-id ID [options]

Runs one normal robot_api_server/Nav2 navigation goal and records the terminal
yaw command/state chain. This script never publishes velocity commands.

Options:
  --pose-id ID              Target saved pose ID, e.g. delivery_512355.
  --building-id ID          Optional building ID.
  --floor-id ID             Optional floor ID.
  --timeout-sec SEC         Navigation timeout. Default: 180.
  --poll-period-sec SEC     API poll period for child pose-error test. Default: 1.0.
  --settle-sec SEC          Child pose-error settle time. Default: 3.0.
  --sample-period-sec SEC   Trace sample period. Default: 0.10.
  --post-capture-sec SEC    Continue tracing after child exits. Default: 3.0.
  --label NAME              Report label. Default: terminal_yaw_trace.
  --api-url URL             robot_api_server URL. Default: http://127.0.0.1:8080.
  --output-root DIR         Report root. Default: reports/navigation_terminal_yaw_trace.
  --no-post-relocalize      Do not run child post-goal relocalization.
EOF
}

while [[ "$#" -gt 0 ]]; do
  case "$1" in
    --pose-id)
      POSE_ID="${2:-}"
      shift 2
      ;;
    --building-id)
      BUILDING_ID="${2:-}"
      shift 2
      ;;
    --floor-id)
      FLOOR_ID="${2:-}"
      shift 2
      ;;
    --timeout-sec)
      TIMEOUT_SEC="${2:-}"
      shift 2
      ;;
    --poll-period-sec)
      POLL_PERIOD_SEC="${2:-}"
      shift 2
      ;;
    --settle-sec)
      SETTLE_SEC="${2:-}"
      shift 2
      ;;
    --sample-period-sec)
      SAMPLE_PERIOD_SEC="${2:-}"
      shift 2
      ;;
    --post-capture-sec)
      POST_CAPTURE_SEC="${2:-}"
      shift 2
      ;;
    --label)
      LABEL="${2:-}"
      shift 2
      ;;
    --api-url)
      API_URL="${2:-}"
      shift 2
      ;;
    --output-root)
      OUTPUT_ROOT="${2:-}"
      shift 2
      ;;
    --no-post-relocalize)
      POST_RELOCALIZE="false"
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "[terminal-yaw-trace] unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

if [[ -z "${POSE_ID}" ]]; then
  echo "[terminal-yaw-trace] --pose-id is required" >&2
  usage >&2
  exit 2
fi

if [[ -f "${SCRIPT_DIR}/common_env.sh" ]]; then
  # shellcheck source=/dev/null
  source "${SCRIPT_DIR}/common_env.sh"
fi
PROJECT_ROOT="${NJRH_PROJECT_ROOT:-${PROJECT_ROOT}}"
OUTPUT_ROOT="${OUTPUT_ROOT/#\~/${HOME}}"

python3 - "${TIMEOUT_SEC}" "${POLL_PERIOD_SEC}" "${SETTLE_SEC}" "${SAMPLE_PERIOD_SEC}" "${POST_CAPTURE_SEC}" <<'PY'
import math
import sys

for value in sys.argv[1:]:
    number = float(value)
    if not math.isfinite(number) or number <= 0.0:
        raise SystemExit("numeric options must be finite positive values")
PY

safe_label="$(printf '%s' "${LABEL}" | tr -c 'A-Za-z0-9_.-' '_')"
timestamp="$(date -u +%Y%m%dT%H%M%SZ)"
OUT_DIR="${OUTPUT_ROOT}/${timestamp}_${safe_label}"
mkdir -p "${OUT_DIR}"

echo "[terminal-yaw-trace] report: ${OUT_DIR}"

RECORDER_PID=""
cleanup() {
  if [[ -n "${RECORDER_PID}" ]] && kill -0 "${RECORDER_PID}" 2>/dev/null; then
    kill "${RECORDER_PID}" 2>/dev/null || true
    wait "${RECORDER_PID}" 2>/dev/null || true
  fi
}
trap cleanup EXIT

python3 - \
  "${OUT_DIR}/samples.csv" \
  "${OUT_DIR}/samples.jsonl" \
  "${API_URL}" \
  "${SAMPLE_PERIOD_SEC}" <<'PY' &
import csv
import json
import math
import signal
import sys
import time
import urllib.request

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from ranger_msgs.msg import MotionState, SystemState

csv_path = sys.argv[1]
jsonl_path = sys.argv[2]
api_url = sys.argv[3].rstrip("/")
sample_period = float(sys.argv[4])

stop = False


def on_signal(_signum, _frame):
    global stop
    stop = True


signal.signal(signal.SIGTERM, on_signal)
signal.signal(signal.SIGINT, on_signal)


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def api_get(path):
    try:
        with urllib.request.urlopen(api_url + path, timeout=0.25) as response:
            return json.loads(response.read().decode("utf-8", errors="replace"))
    except Exception as exc:
        return {"ok": False, "error": repr(exc)}


def twist_row(msg):
    return {
        "lx": float(msg.linear.x),
        "ly": float(msg.linear.y),
        "az": float(msg.angular.z),
        "stamp": time.time(),
    }


def odom_row(msg):
    return {
        "x": float(msg.pose.pose.position.x),
        "y": float(msg.pose.pose.position.y),
        "yaw": yaw_from_quaternion(msg.pose.pose.orientation),
        "vx": float(msg.twist.twist.linear.x),
        "vy": float(msg.twist.twist.linear.y),
        "wz": float(msg.twist.twist.angular.z),
        "stamp": time.time(),
    }


def imu_row(msg):
    return {
        "wz": float(msg.angular_velocity.z),
        "stamp": time.time(),
    }


def motion_float(msg, name):
    try:
        return float(getattr(msg, name))
    except Exception:
        return float("nan")


class Recorder:
    def __init__(self):
        self.node = rclpy.create_node("navigation_terminal_yaw_trace_recorder")
        self.data = {}
        self.jsonl = open(jsonl_path, "w", encoding="utf-8")
        self.csv = open(csv_path, "w", newline="", encoding="utf-8")
        self.fields = [
            "t_wall",
            "t_rel",
            "api_nav_state",
            "api_nav_phase",
            "api_nav_pose_id",
            "api_nav2_result_code",
            "api_nav2_succeeded",
            "api_final_distance_m",
            "api_final_yaw_error_rad",
            "api_pose_x",
            "api_pose_y",
            "api_pose_yaw",
            "api_target_x",
            "api_target_y",
            "api_target_yaw",
            "cmd_nav_raw_lx",
            "cmd_nav_raw_az",
            "cmd_nav_lx",
            "cmd_nav_az",
            "cmd_collision_lx",
            "cmd_collision_az",
            "cmd_safe_lx",
            "cmd_safe_az",
            "cmd_base_lx",
            "cmd_base_az",
            "wheel_odom_x",
            "wheel_odom_y",
            "wheel_odom_yaw",
            "wheel_odom_vx",
            "wheel_odom_wz",
            "wheel_odom_ekf_x",
            "wheel_odom_ekf_y",
            "wheel_odom_ekf_yaw",
            "wheel_odom_ekf_vx",
            "wheel_odom_ekf_wz",
            "local_odom_x",
            "local_odom_y",
            "local_odom_yaw",
            "local_odom_vx",
            "local_odom_wz",
            "imu_wz",
            "motion_mode",
            "motion_linear_velocity",
            "motion_lateral_velocity",
            "motion_angular_velocity",
            "motion_steering_angle",
            "system_motion_mode",
            "mode_controller_status",
            "safety_status",
            "amcl_status",
        ]
        self.writer = csv.DictWriter(self.csv, fieldnames=self.fields)
        self.writer.writeheader()
        self.start = time.time()
        self.last_api_state = {}
        self.last_api_pose = {}

        self.subscribe_twist("/cmd_vel_nav_raw", "cmd_nav_raw")
        self.subscribe_twist("/cmd_vel_nav", "cmd_nav")
        self.subscribe_twist("/cmd_vel_collision_checked", "cmd_collision")
        self.subscribe_twist("/cmd_vel_safe", "cmd_safe")
        self.subscribe_twist("/cmd_vel", "cmd_base")
        self.subscribe_odom("/wheel/odom", "wheel_odom")
        self.subscribe_odom("/wheel/odom_ekf", "wheel_odom_ekf")
        self.subscribe_odom("/local_state/odometry", "local_odom")
        self.node.create_subscription(Imu, "/lidar_imu_bias_corrected", self.on_imu, qos_profile_sensor_data)
        self.node.create_subscription(MotionState, "/motion_state", self.on_motion_state, 20)
        self.node.create_subscription(SystemState, "/system_state", self.on_system_state, 20)
        self.node.create_subscription(String, "/ranger_mini3_mode_controller/status", self.on_string("mode_controller_status"), 10)
        self.node.create_subscription(String, "/safety/status", self.on_string("safety_status"), 10)
        self.node.create_subscription(String, "/amcl_scan_admission/status", self.on_string("amcl_status"), 10)
        self.node.create_timer(sample_period, self.sample)

    def subscribe_twist(self, topic, key):
        self.node.create_subscription(Twist, topic, lambda msg, k=key: self.data.__setitem__(k, twist_row(msg)), 20)

    def subscribe_odom(self, topic, key):
        self.node.create_subscription(Odometry, topic, lambda msg, k=key: self.data.__setitem__(k, odom_row(msg)), 20)

    def on_imu(self, msg):
        self.data["imu"] = imu_row(msg)

    def on_motion_state(self, msg):
        self.data["motion_state"] = {
            "motion_mode": int(msg.motion_mode),
            "linear_velocity": motion_float(msg, "linear_velocity"),
            "lateral_velocity": motion_float(msg, "lateral_velocity"),
            "angular_velocity": motion_float(msg, "angular_velocity"),
            "steering_angle": motion_float(msg, "steering_angle"),
            "stamp": time.time(),
        }

    def on_system_state(self, msg):
        self.data["system_motion_mode"] = int(msg.motion_mode)

    def on_string(self, key):
        def callback(msg):
            self.data[key] = str(msg.data)
        return callback

    def sample(self):
        state = api_get("/api/v1/navigation/state")
        pose = api_get("/api/v1/robot/pose")
        if isinstance(state, dict) and state.get("ok"):
            self.last_api_state = state
        if isinstance(pose, dict) and pose.get("ok"):
            self.last_api_pose = pose
        nav_goal = self.last_api_state.get("navigation_goal") if isinstance(self.last_api_state, dict) else {}
        if not isinstance(nav_goal, dict):
            nav_goal = {}
        target = nav_goal.get("target") if isinstance(nav_goal, dict) else {}
        if not isinstance(target, dict):
            target = {}

        row = {name: "" for name in self.fields}
        now = time.time()
        row.update({
            "t_wall": f"{now:.6f}",
            "t_rel": f"{now - self.start:.6f}",
            "api_nav_state": nav_goal.get("state", ""),
            "api_nav_phase": nav_goal.get("phase", ""),
            "api_nav_pose_id": nav_goal.get("pose_id", ""),
            "api_nav2_result_code": nav_goal.get("nav2_result_code", ""),
            "api_nav2_succeeded": nav_goal.get("nav2_succeeded", ""),
            "api_final_distance_m": nav_goal.get("final_distance_m", ""),
            "api_final_yaw_error_rad": nav_goal.get("final_yaw_error_rad", ""),
            "api_pose_x": self.last_api_pose.get("x", ""),
            "api_pose_y": self.last_api_pose.get("y", ""),
            "api_pose_yaw": self.last_api_pose.get("yaw", ""),
            "api_target_x": target.get("x", ""),
            "api_target_y": target.get("y", ""),
            "api_target_yaw": target.get("yaw", ""),
            "system_motion_mode": self.data.get("system_motion_mode", ""),
            "mode_controller_status": self.data.get("mode_controller_status", ""),
            "safety_status": self.data.get("safety_status", ""),
            "amcl_status": self.data.get("amcl_status", ""),
        })
        motion = self.data.get("motion_state") or {}
        row["motion_mode"] = motion.get("motion_mode", "")
        row["motion_linear_velocity"] = motion.get("linear_velocity", "")
        row["motion_lateral_velocity"] = motion.get("lateral_velocity", "")
        row["motion_angular_velocity"] = motion.get("angular_velocity", "")
        row["motion_steering_angle"] = motion.get("steering_angle", "")
        for key, prefix in (
            ("cmd_nav_raw", "cmd_nav_raw"),
            ("cmd_nav", "cmd_nav"),
            ("cmd_collision", "cmd_collision"),
            ("cmd_safe", "cmd_safe"),
            ("cmd_base", "cmd_base"),
        ):
            value = self.data.get(key) or {}
            row[f"{prefix}_lx"] = value.get("lx", "")
            row[f"{prefix}_az"] = value.get("az", "")
        for key, prefix in (
            ("wheel_odom", "wheel_odom"),
            ("wheel_odom_ekf", "wheel_odom_ekf"),
            ("local_odom", "local_odom"),
        ):
            value = self.data.get(key) or {}
            row[f"{prefix}_x"] = value.get("x", "")
            row[f"{prefix}_y"] = value.get("y", "")
            row[f"{prefix}_yaw"] = value.get("yaw", "")
            row[f"{prefix}_vx"] = value.get("vx", "")
            row[f"{prefix}_wz"] = value.get("wz", "")
        row["imu_wz"] = (self.data.get("imu") or {}).get("wz", "")

        self.writer.writerow(row)
        self.csv.flush()
        self.jsonl.write(json.dumps({"row": row, "state": state, "pose": pose}, ensure_ascii=False) + "\n")
        self.jsonl.flush()

    def close(self):
        self.csv.close()
        self.jsonl.close()
        self.node.destroy_node()


rclpy.init()
recorder = Recorder()
try:
    while rclpy.ok() and not stop:
        try:
            rclpy.spin_once(recorder.node, timeout_sec=0.1)
        except ExternalShutdownException:
            break
        except Exception:
            if stop or not rclpy.ok():
                break
            raise
finally:
    recorder.close()
    if rclpy.ok():
        rclpy.shutdown()
PY
RECORDER_PID=$!

child_label="${safe_label}_nav_pose_error"
child_args=(
  --pose-id "${POSE_ID}"
  --timeout-sec "${TIMEOUT_SEC}"
  --poll-period-sec "${POLL_PERIOD_SEC}"
  --settle-sec "${SETTLE_SEC}"
  --label "${child_label}"
  --api-url "${API_URL}"
)
if [[ -n "${BUILDING_ID}" ]]; then
  child_args+=(--building-id "${BUILDING_ID}")
fi
if [[ -n "${FLOOR_ID}" ]]; then
  child_args+=(--floor-id "${FLOOR_ID}")
fi
if [[ "${POST_RELOCALIZE}" == "false" ]]; then
  child_args+=(--no-post-relocalize)
fi

set +e
bash "${SCRIPT_DIR}/run_navigation_pose_error_test.sh" "${child_args[@]}" 2>&1 | tee "${OUT_DIR}/child_navigation.log"
child_rc="${PIPESTATUS[0]}"
set -e

sleep "${POST_CAPTURE_SEC}"
cleanup
trap - EXIT

child_report="$(find "${PROJECT_ROOT}/reports/navigation_pose_error_test" -mindepth 1 -maxdepth 1 -type d -name "*_${child_label}" | sort | tail -n 1 || true)"
if [[ -n "${child_report}" ]]; then
  printf '%s\n' "${child_report}" >"${OUT_DIR}/child_report_dir.txt"
fi

python3 - \
  "${OUT_DIR}" \
  "${POSE_ID}" \
  "${child_rc}" \
  "${child_report}" <<'PY'
import csv
import json
import math
import sys
from pathlib import Path

out = Path(sys.argv[1])
pose_id = sys.argv[2]
child_rc = int(sys.argv[3])
child_report = sys.argv[4]
samples_path = out / "samples.csv"
rows = []
if samples_path.exists():
    with samples_path.open(newline="", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))


def to_float(value):
    try:
        number = float(value)
    except Exception:
        return float("nan")
    return number if math.isfinite(number) else float("nan")


def finite_values(name):
    values = [to_float(row.get(name, "")) for row in rows]
    return [value for value in values if math.isfinite(value)]


def command_stats(prefix):
    lx_name = f"{prefix}_lx"
    az_name = f"{prefix}_az"
    nonzero = []
    max_lx = 0.0
    max_az = 0.0
    last_nonzero_t = float("nan")
    first_nonzero_t = float("nan")
    for row in rows:
        t = to_float(row.get("t_rel", ""))
        lx = to_float(row.get(lx_name, ""))
        az = to_float(row.get(az_name, ""))
        if math.isfinite(lx):
            max_lx = max(max_lx, abs(lx))
        if math.isfinite(az):
            max_az = max(max_az, abs(az))
        if math.isfinite(t) and (abs(lx) > 1e-3 or abs(az) > 1e-3):
            nonzero.append(t)
    if nonzero:
        first_nonzero_t = nonzero[0]
        last_nonzero_t = nonzero[-1]
    return {
        "nonzero_count": len(nonzero),
        "first_nonzero_t": first_nonzero_t,
        "last_nonzero_t": last_nonzero_t,
        "max_abs_lx": max_lx,
        "max_abs_az": max_az,
    }


def last_nonempty(name):
    for row in reversed(rows):
        value = row.get(name, "")
        if value != "":
            return value
    return ""


def yaw_delta(name):
    values = finite_values(name)
    if len(values) < 2:
        return float("nan")
    return math.atan2(math.sin(values[-1] - values[0]), math.cos(values[-1] - values[0]))


summary = {
    "pose_id": pose_id,
    "child_rc": child_rc,
    "child_report": child_report,
    "sample_count": len(rows),
    "duration_sec": (to_float(rows[-1]["t_rel"]) - to_float(rows[0]["t_rel"])) if len(rows) >= 2 else float("nan"),
    "last_api_nav_state": last_nonempty("api_nav_state"),
    "last_api_nav_phase": last_nonempty("api_nav_phase"),
    "last_api_nav2_result_code": last_nonempty("api_nav2_result_code"),
    "last_api_nav2_succeeded": last_nonempty("api_nav2_succeeded"),
    "last_api_final_distance_m": last_nonempty("api_final_distance_m"),
    "last_api_final_yaw_error_rad": last_nonempty("api_final_yaw_error_rad"),
    "last_api_pose_x": last_nonempty("api_pose_x"),
    "last_api_pose_y": last_nonempty("api_pose_y"),
    "last_api_pose_yaw": last_nonempty("api_pose_yaw"),
    "last_api_target_x": last_nonempty("api_target_x"),
    "last_api_target_y": last_nonempty("api_target_y"),
    "last_api_target_yaw": last_nonempty("api_target_yaw"),
    "cmd_stats": {
        "nav_raw": command_stats("cmd_nav_raw"),
        "nav": command_stats("cmd_nav"),
        "collision_checked": command_stats("cmd_collision"),
        "safe": command_stats("cmd_safe"),
        "base": command_stats("cmd_base"),
    },
    "yaw_delta_rad": {
        "api_map": yaw_delta("api_pose_yaw"),
        "wheel_odom": yaw_delta("wheel_odom_yaw"),
        "wheel_odom_ekf": yaw_delta("wheel_odom_ekf_yaw"),
        "local_odom": yaw_delta("local_odom_yaw"),
    },
    "max_abs_imu_wz": max([abs(v) for v in finite_values("imu_wz")] or [float("nan")]),
    "max_abs_motion_linear_velocity": max([abs(v) for v in finite_values("motion_linear_velocity")] or [float("nan")]),
    "max_abs_motion_angular_velocity": max([abs(v) for v in finite_values("motion_angular_velocity")] or [float("nan")]),
    "max_abs_motion_steering_angle": max([abs(v) for v in finite_values("motion_steering_angle")] or [float("nan")]),
    "motion_modes_seen": sorted(set(row.get("motion_mode", "") for row in rows if row.get("motion_mode", "") != "")),
    "system_motion_modes_seen": sorted(set(row.get("system_motion_mode", "") for row in rows if row.get("system_motion_mode", "") != "")),
}

(out / "summary.json").write_text(json.dumps(summary, indent=2, sort_keys=True), encoding="utf-8")

with (out / "summary.md").open("w", encoding="utf-8") as handle:
    handle.write("# Navigation Terminal Yaw Trace\n\n")
    handle.write(f"- pose_id: `{pose_id}`\n")
    handle.write(f"- child_rc: `{child_rc}`\n")
    handle.write(f"- child_report: `{child_report}`\n")
    handle.write(f"- sample_count: `{summary['sample_count']}`\n")
    handle.write(f"- duration_sec: `{summary['duration_sec']}`\n")
    handle.write(f"- last_api_nav_state: `{summary['last_api_nav_state']}`\n")
    handle.write(f"- last_api_nav_phase: `{summary['last_api_nav_phase']}`\n")
    handle.write(f"- last_api_nav2_result_code: `{summary['last_api_nav2_result_code']}`\n")
    handle.write(f"- last_api_final_distance_m: `{summary['last_api_final_distance_m']}`\n")
    handle.write(f"- last_api_final_yaw_error_rad: `{summary['last_api_final_yaw_error_rad']}`\n")
    handle.write(f"- last_api_pose_xy_yaw: `({summary['last_api_pose_x']}, {summary['last_api_pose_y']}, {summary['last_api_pose_yaw']})`\n")
    handle.write(f"- last_api_target_xy_yaw: `({summary['last_api_target_x']}, {summary['last_api_target_y']}, {summary['last_api_target_yaw']})`\n")
    handle.write(f"- yaw_delta_rad: `{summary['yaw_delta_rad']}`\n")
    handle.write(f"- max_abs_imu_wz: `{summary['max_abs_imu_wz']}`\n")
    handle.write(f"- max_abs_motion_linear_velocity: `{summary['max_abs_motion_linear_velocity']}`\n")
    handle.write(f"- max_abs_motion_angular_velocity: `{summary['max_abs_motion_angular_velocity']}`\n")
    handle.write(f"- max_abs_motion_steering_angle: `{summary['max_abs_motion_steering_angle']}`\n")
    handle.write(f"- motion_modes_seen: `{summary['motion_modes_seen']}`\n")
    handle.write(f"- system_motion_modes_seen: `{summary['system_motion_modes_seen']}`\n")
    handle.write("\n## Command Chain\n\n")
    handle.write("| topic stage | nonzero_count | first_nonzero_t | last_nonzero_t | max_abs_lx | max_abs_az |\n")
    handle.write("|---|---:|---:|---:|---:|---:|\n")
    for key, value in summary["cmd_stats"].items():
        handle.write(
            f"| {key} | {value['nonzero_count']} | {value['first_nonzero_t']} | "
            f"{value['last_nonzero_t']} | {value['max_abs_lx']} | {value['max_abs_az']} |\n"
        )
    handle.write("\n## Files\n\n")
    handle.write("- `samples.csv`: compact command/state samples\n")
    handle.write("- `samples.jsonl`: samples with raw API payloads\n")
    handle.write("- `child_navigation.log`: wrapped navigation test stdout/stderr\n")
PY

echo "[terminal-yaw-trace] summary: ${OUT_DIR}/summary.md"
exit "${child_rc}"
