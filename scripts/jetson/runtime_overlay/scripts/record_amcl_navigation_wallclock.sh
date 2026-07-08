#!/usr/bin/env bash
set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="${WORKSPACE_ROOT:-$(cd "${SCRIPT_DIR}/../../../.." && pwd)}"
export NJRH_PROJECT_ROOT="${NJRH_PROJECT_ROOT:-${WORKSPACE_ROOT}}"
source "${SCRIPT_DIR}/common_env.sh"
set -e

API_URL="${API_URL:-http://127.0.0.1:8080}"
DURATION_SEC=60
SAMPLE_PERIOD_SEC=1.0
START_DELAY_SEC=3
LABEL="amcl_navigation"
OUTPUT_DIR=""
SEND_GOAL=false
POSE_ID=""
BUILDING_ID="B10"
FLOOR_ID="F1"
PREFIX="[amcl-nav-wallclock]"

usage() {
  cat <<'EOF'
Usage:
  # Record only. Start this first, then send navigation from the App.
  bash scripts/jetson/runtime_overlay/scripts/record_amcl_navigation_wallclock.sh \
    --duration-sec 60 --label nav_amcl

  # Record first, then send a pose goal through robot_api_server after 3 seconds.
  bash scripts/jetson/runtime_overlay/scripts/record_amcl_navigation_wallclock.sh \
    --duration-sec 60 --label delivery_675235_amcl \
    --send-goal --pose-id delivery_675235

Read-only unless --send-goal is set. This script never publishes /cmd_vel,
never sets parameters, and never restarts nodes.

Options:
  --duration-sec N       Wall-clock capture duration. Default: 60.
  --sample-period-sec N  API poll period. Default: 1.0.
  --label LABEL          Report label. Default: amcl_navigation.
  --output-dir DIR       Report directory. Default: reports/amcl_navigation_wallclock/<timestamp>_<label>_<duration>s.
  --api-url URL          robot_api_server URL. Default: http://127.0.0.1:8080.
  --send-goal            Send navigation goal after recorder subscriptions are up.
  --pose-id ID           Pose id for --send-goal.
  --building-id ID       Building id for --send-goal. Default: B10.
  --floor-id ID          Floor id for --send-goal. Default: F1.
  --start-delay-sec N    Delay between recorder start and goal POST. Default: 3.
  -h, --help             Show this help.
EOF
}

sanitize_label() {
  printf '%s' "$1" | tr -c 'A-Za-z0-9_.-' '_' | sed 's/^_*//;s/_*$//'
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --duration-sec)
      DURATION_SEC="${2:-}"
      shift 2
      ;;
    --sample-period-sec)
      SAMPLE_PERIOD_SEC="${2:-}"
      shift 2
      ;;
    --label)
      LABEL="${2:-}"
      shift 2
      ;;
    --output-dir)
      OUTPUT_DIR="${2:-}"
      shift 2
      ;;
    --api-url)
      API_URL="${2:-}"
      shift 2
      ;;
    --send-goal)
      SEND_GOAL=true
      shift
      ;;
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
    --start-delay-sec)
      START_DELAY_SEC="${2:-}"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "${PREFIX} FAIL unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

if ! [[ "${DURATION_SEC}" =~ ^[0-9]+$ ]] || [[ "${DURATION_SEC}" -lt 10 ]]; then
  echo "${PREFIX} FAIL --duration-sec must be an integer >= 10" >&2
  exit 2
fi

if ! [[ "${SAMPLE_PERIOD_SEC}" =~ ^[0-9]+([.][0-9]+)?$ ]]; then
  echo "${PREFIX} FAIL --sample-period-sec must be numeric" >&2
  exit 2
fi

if ! [[ "${START_DELAY_SEC}" =~ ^[0-9]+([.][0-9]+)?$ ]]; then
  echo "${PREFIX} FAIL --start-delay-sec must be numeric" >&2
  exit 2
fi

if [[ "${SEND_GOAL}" == "true" && -z "${POSE_ID}" ]]; then
  echo "${PREFIX} FAIL --send-goal requires --pose-id" >&2
  exit 2
fi

SAFE_LABEL="$(sanitize_label "${LABEL}")"
[[ -n "${SAFE_LABEL}" ]] || SAFE_LABEL="amcl_navigation"

if [[ -z "${OUTPUT_DIR}" ]]; then
  RUN_ID="$(date -u +%Y%m%dT%H%M%SZ)_${SAFE_LABEL}_${DURATION_SEC}s"
  OUTPUT_DIR="${NJRH_PROJECT_ROOT}/reports/amcl_navigation_wallclock/${RUN_ID}"
fi
mkdir -p "${OUTPUT_DIR}"

echo "${PREFIX} report_dir=${OUTPUT_DIR}"
echo "${PREFIX} duration_sec=${DURATION_SEC} sample_period_sec=${SAMPLE_PERIOD_SEC}"
echo "${PREFIX} read-only=$([[ "${SEND_GOAL}" == "true" ]] && echo false || echo true)"

{
  echo "# command"
  printf '%q ' "$0" "$@"
  echo
  echo "# date_utc"
  date -u +%Y-%m-%dT%H:%M:%SZ
  echo "# api_url"
  echo "${API_URL}"
  echo "# topic_info"
  for topic in /scan /amcl_pose /localization/bridge_status /local_state/odometry /cmd_vel_nav /cmd_vel_collision_checked /cmd_vel_safe /cmd_vel /tf; do
    echo "## ${topic}"
    timeout 4 ros2 topic info -v "${topic}" 2>&1 || true
  done
  echo "# amcl_params"
  for param in scan_topic tf_broadcast update_min_d update_min_a resample_interval save_pose_rate laser_model_type max_beams min_particles max_particles transform_tolerance; do
    printf '/amcl %s: ' "${param}"
    timeout 4 ros2 param get /amcl "${param}" 2>&1 || true
  done
} >"${OUTPUT_DIR}/preflight.txt"

curl -sS "${API_URL}/api/v1/navigation/state" >"${OUTPUT_DIR}/navigation_before.json" || true

python3 - "${OUTPUT_DIR}" "${DURATION_SEC}" "${SAMPLE_PERIOD_SEC}" "${API_URL}" <<'PYREC' &
import csv
import json
import math
import sys
import time
import urllib.request
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage

out_dir = Path(sys.argv[1])
duration_sec = float(sys.argv[2])
sample_period_sec = float(sys.argv[3])
api_url = sys.argv[4].rstrip("/")

rclpy.init()
node = rclpy.create_node("amcl_navigation_wallclock_recorder")
wall_start = time.monotonic()

sensor_qos = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
)
tf_qos = QoSProfile(depth=100)

scan_count = 0
cmd_nonzero_count = 0
odom_distance_m = 0.0
last_odom_xy = None

status_rows = []
amcl_rows = []
odom_rows = []
cmd_rows = []
tf_rows = []
api_rows = []


def wall_t():
    return time.monotonic() - wall_start


def yaw_from_q(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def as_float(value):
    try:
        if value is None:
            return None
        return float(value)
    except Exception:
        return None


def fetch_navigation_state():
    row = {"t": wall_t()}
    try:
        with urllib.request.urlopen(f"{api_url}/api/v1/navigation/state", timeout=0.35) as response:
            data = json.loads(response.read().decode("utf-8", errors="replace"))
    except Exception as exc:
        row["error"] = repr(exc)
        api_rows.append(row)
        return
    goal = data.get("navigation_goal", {}) if isinstance(data, dict) else {}
    for key in [
        "state",
        "healthy",
        "safe_for_goal_start",
        "correction_active",
        "amcl_state",
        "amcl_ready",
        "amcl_correction_ready",
        "amcl_correction_pending",
        "localization_degraded",
        "localization_degraded_reason",
    ]:
        row[key] = data.get(key)
    for key in [
        "id",
        "state",
        "phase",
        "pose_id",
        "detail",
        "nav2_result_code",
        "nav2_succeeded",
        "final_distance_m",
        "final_yaw_error_rad",
        "position_reached",
        "yaw_align_required",
        "yaw_align_active",
        "yaw_align_succeeded",
        "final_pose_verified",
    ]:
        row[f"goal_{key}"] = goal.get(key)
    api_rows.append(row)


def on_scan(_msg):
    global scan_count
    scan_count += 1


def on_status(msg):
    try:
        data = json.loads(msg.data)
    except Exception as exc:
        status_rows.append({"t": wall_t(), "json_error": repr(exc)})
        return
    row = {"t": wall_t()}
    for key in [
        "localization_mode",
        "gate_mode",
        "active_correction_source",
        "last_candidate_source",
        "last_accepted_source",
        "last_rejected_source",
        "last_accept_reason",
        "last_reject_reason",
        "amcl_input_enabled",
        "amcl_gate_mode",
        "amcl_pose_count",
        "amcl_candidate_count",
        "amcl_shadow_candidate_count",
        "amcl_accepted_count",
        "amcl_rejected_count",
        "amcl_suppressed_after_isaac_count",
        "amcl_last_state",
        "last_amcl_pose_age_ms",
        "last_amcl_xy_covariance",
        "last_amcl_yaw_covariance",
        "last_candidate_correction_translation_m",
        "last_candidate_correction_yaw_rad",
        "last_accepted_correction_translation_m",
        "last_accepted_correction_yaw_rad",
        "has_map_to_odom",
        "map_to_odom_publisher_owner",
        "map_to_odom_age_ms",
        "latest_odom_tf_fresh",
        "latest_odom_tf_age_ms",
        "last_odom_tf_history_lookup_ok",
        "correction_active",
        "remaining_translation_error_m",
        "remaining_yaw_error_rad",
        "safe_for_goal_start",
        "isaac_background_correction_removed",
    ]:
        row[key] = data.get(key)
    status_rows.append(row)


def on_amcl(msg):
    amcl_rows.append(
        {
            "t": wall_t(),
            "stamp_sec": msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
            "x": msg.pose.pose.position.x,
            "y": msg.pose.pose.position.y,
            "yaw": yaw_from_q(msg.pose.pose.orientation),
            "cov_x": msg.pose.covariance[0],
            "cov_y": msg.pose.covariance[7],
            "cov_yaw": msg.pose.covariance[35],
        }
    )


def on_odom(msg):
    global odom_distance_m, last_odom_xy
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    if last_odom_xy is not None:
        odom_distance_m += math.hypot(x - last_odom_xy[0], y - last_odom_xy[1])
    last_odom_xy = (x, y)
    odom_rows.append(
        {
            "t": wall_t(),
            "stamp_sec": msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
            "x": x,
            "y": y,
            "yaw": yaw_from_q(msg.pose.pose.orientation),
            "distance_accum_m": odom_distance_m,
        }
    )


def make_cmd_cb(topic):
    def on_cmd(msg):
        global cmd_nonzero_count
        nonzero = (
            abs(msg.linear.x) > 1e-3
            or abs(msg.linear.y) > 1e-3
            or abs(msg.angular.z) > 1e-3
        )
        if nonzero:
            cmd_nonzero_count += 1
        cmd_rows.append(
            {
                "t": wall_t(),
                "topic": topic,
                "linear_x": msg.linear.x,
                "linear_y": msg.linear.y,
                "angular_z": msg.angular.z,
                "nonzero": nonzero,
            }
        )

    return on_cmd


def on_tf(msg):
    t = wall_t()
    for tr in msg.transforms:
        if (tr.header.frame_id, tr.child_frame_id) in {
            ("map", "odom"),
            ("odom", "base_link"),
        }:
            tf_rows.append(
                {
                    "t": t,
                    "stamp_sec": tr.header.stamp.sec + tr.header.stamp.nanosec * 1e-9,
                    "parent": tr.header.frame_id,
                    "child": tr.child_frame_id,
                    "x": tr.transform.translation.x,
                    "y": tr.transform.translation.y,
                    "yaw": yaw_from_q(tr.transform.rotation),
                }
            )


node.create_subscription(LaserScan, "/scan", on_scan, sensor_qos)
node.create_subscription(String, "/localization/bridge_status", on_status, 10)
node.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", on_amcl, 10)
node.create_subscription(Odometry, "/local_state/odometry", on_odom, sensor_qos)
for topic_name in ["/cmd_vel_nav", "/cmd_vel_collision_checked", "/cmd_vel_safe", "/cmd_vel"]:
    node.create_subscription(Twist, topic_name, make_cmd_cb(topic_name), sensor_qos)
node.create_subscription(TFMessage, "/tf", on_tf, tf_qos)

print(f"[amcl-nav-wallclock] recorder active duration={duration_sec:.1f}s", flush=True)
next_api_sample = 0.0
while rclpy.ok() and wall_t() < duration_sec:
    if wall_t() >= next_api_sample:
        fetch_navigation_state()
        next_api_sample = wall_t() + sample_period_sec
    rclpy.spin_once(node, timeout_sec=0.05)
fetch_navigation_state()
actual_duration = max(0.001, wall_t())
node.destroy_node()
rclpy.shutdown()


def write_csv(filename, rows):
    path = out_dir / filename
    if not rows:
        path.write_text("", encoding="utf-8")
        return
    fields = []
    for row in rows:
        for key in row:
            if key not in fields:
                fields.append(key)
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


write_csv("bridge_status.csv", status_rows)
write_csv("amcl_pose.csv", amcl_rows)
write_csv("local_state_odom.csv", odom_rows)
write_csv("cmd_vel.csv", cmd_rows)
write_csv("tf_main.csv", tf_rows)
write_csv("navigation_state_samples.csv", api_rows)

cmd_nonzero_by_topic = {}
for row in cmd_rows:
    if row.get("nonzero"):
        topic = row.get("topic", "")
        cmd_nonzero_by_topic[topic] = cmd_nonzero_by_topic.get(topic, 0) + 1

summary = {
    "duration_wall_sec": actual_duration,
    "scan_count": scan_count,
    "scan_hz": scan_count / actual_duration,
    "bridge_status_count": len(status_rows),
    "bridge_status_hz": len(status_rows) / actual_duration,
    "amcl_pose_messages_observed_by_recorder": len(amcl_rows),
    "odom_distance_accum_m": odom_distance_m,
    "cmd_nonzero_count": cmd_nonzero_count,
    "cmd_nonzero_by_topic": cmd_nonzero_by_topic,
    "api_sample_count": len(api_rows),
}

if status_rows:
    first = status_rows[0]
    last = status_rows[-1]
    for source_key, out_key in [
        ("amcl_pose_count", "amcl_pose_count_delta_bridge"),
        ("amcl_candidate_count", "amcl_candidate_count_delta_bridge"),
        ("amcl_shadow_candidate_count", "amcl_shadow_candidate_count_delta_bridge"),
        ("amcl_accepted_count", "amcl_accepted_count_delta_bridge"),
        ("amcl_rejected_count", "amcl_rejected_count_delta_bridge"),
        ("amcl_suppressed_after_isaac_count", "amcl_suppressed_after_isaac_count_delta_bridge"),
    ]:
        a = as_float(first.get(source_key))
        b = as_float(last.get(source_key))
        summary[out_key] = None if a is None or b is None else b - a

    translations = [
        as_float(row.get("last_candidate_correction_translation_m"))
        for row in status_rows
    ]
    translations = [value for value in translations if value is not None]
    yaws = [
        abs(as_float(row.get("last_candidate_correction_yaw_rad")))
        for row in status_rows
        if as_float(row.get("last_candidate_correction_yaw_rad")) is not None
    ]
    accepted_translations = [
        as_float(row.get("last_accepted_correction_translation_m"))
        for row in status_rows
    ]
    accepted_translations = [value for value in accepted_translations if value is not None]
    accepted_yaws = [
        abs(as_float(row.get("last_accepted_correction_yaw_rad")))
        for row in status_rows
        if as_float(row.get("last_accepted_correction_yaw_rad")) is not None
    ]
    summary.update(
        {
            "candidate_translation_m_max": max(translations) if translations else None,
            "candidate_translation_m_last": translations[-1] if translations else None,
            "candidate_yaw_rad_max": max(yaws) if yaws else None,
            "candidate_yaw_rad_last": yaws[-1] if yaws else None,
            "accepted_translation_m_max": max(accepted_translations) if accepted_translations else None,
            "accepted_translation_m_last": accepted_translations[-1] if accepted_translations else None,
            "accepted_yaw_rad_max": max(accepted_yaws) if accepted_yaws else None,
            "accepted_yaw_rad_last": accepted_yaws[-1] if accepted_yaws else None,
            "last_reject_reason_last": last.get("last_reject_reason"),
            "last_accept_reason_last": last.get("last_accept_reason"),
            "last_amcl_pose_age_ms_last": last.get("last_amcl_pose_age_ms"),
            "last_amcl_xy_covariance_last": last.get("last_amcl_xy_covariance"),
            "map_to_odom_age_ms_last": last.get("map_to_odom_age_ms"),
            "latest_odom_tf_age_ms_last": last.get("latest_odom_tf_age_ms"),
        }
    )

if api_rows:
    first_goal = next((row for row in api_rows if row.get("goal_state")), api_rows[0])
    last_goal = api_rows[-1]
    summary["goal_state_first"] = first_goal.get("goal_state")
    summary["goal_phase_first"] = first_goal.get("goal_phase")
    summary["goal_pose_id_first"] = first_goal.get("goal_pose_id")
    summary["goal_state_last"] = last_goal.get("goal_state")
    summary["goal_phase_last"] = last_goal.get("goal_phase")
    summary["goal_pose_id_last"] = last_goal.get("goal_pose_id")
    summary["goal_detail_last"] = last_goal.get("goal_detail")
    summary["goal_nav2_result_code_last"] = last_goal.get("goal_nav2_result_code")
    summary["goal_final_distance_m_last"] = last_goal.get("goal_final_distance_m")
    summary["goal_final_yaw_error_rad_last"] = last_goal.get("goal_final_yaw_error_rad")

(out_dir / "summary.json").write_text(
    json.dumps(summary, indent=2, ensure_ascii=False) + "\n",
    encoding="utf-8",
)
print(f"[amcl-nav-wallclock] wrote {out_dir / 'summary.json'}", flush=True)
PYREC
RECORDER_PID=$!

echo "${PREFIX} recorder_pid=${RECORDER_PID}"
if [[ "${SEND_GOAL}" == "true" ]]; then
  echo "${PREFIX} waiting ${START_DELAY_SEC}s before goal POST"
  sleep "${START_DELAY_SEC}"
  GOAL_JSON="$(python3 - "${BUILDING_ID}" "${FLOOR_ID}" "${POSE_ID}" <<'PY'
import json
import sys

print(json.dumps({"building_id": sys.argv[1], "floor_id": sys.argv[2], "pose_id": sys.argv[3]}))
PY
)"
  echo "${PREFIX} posting navigation goal pose_id=${POSE_ID}"
  {
    date -u +%Y-%m-%dT%H:%M:%SZ
    curl -sS -i -X POST "${API_URL}/api/v1/navigation/goal" \
      -H 'Content-Type: application/json' \
      --data "${GOAL_JSON}" || true
  } | tee "${OUTPUT_DIR}/goal_post_response.txt"
else
  echo "${PREFIX} recorder is active; start navigation from App now"
fi

set +e
wait "${RECORDER_PID}"
RECORDER_STATUS=$?
set -e

curl -sS "${API_URL}/api/v1/navigation/state" >"${OUTPUT_DIR}/navigation_after.json" || true

python3 - "${OUTPUT_DIR}" <<'PYSUMMARY'
import json
import pathlib
import sys

out_dir = pathlib.Path(sys.argv[1])


def load_json_file(path):
    if not path.exists():
        return {}
    text = path.read_text(encoding="utf-8", errors="replace")
    start = text.find("{")
    if start < 0:
        return {}
    try:
        return json.loads(text[start:])
    except Exception:
        return {}


def goal_from(nav):
    return nav.get("navigation_goal", {}) if isinstance(nav, dict) else {}


summary = load_json_file(out_dir / "summary.json")
nav_before = load_json_file(out_dir / "navigation_before.json")
nav_after = load_json_file(out_dir / "navigation_after.json")
goal_response = (out_dir / "goal_post_response.txt").read_text(
    encoding="utf-8", errors="replace"
) if (out_dir / "goal_post_response.txt").exists() else ""

status_line = ""
for line in goal_response.splitlines():
    if line.startswith("HTTP/"):
        status_line = line
        break

lines = [
    "# AMCL Navigation Wallclock Observation",
    "",
    f"- report_dir: {out_dir}",
    f"- goal_post_status: {status_line or 'not_sent'}",
]

for prefix, nav in [("before", nav_before), ("after", nav_after)]:
    goal = goal_from(nav)
    for key in [
        "id",
        "state",
        "phase",
        "pose_id",
        "detail",
        "nav2_result_code",
        "nav2_succeeded",
        "final_distance_m",
        "final_yaw_error_rad",
        "position_reached",
        "yaw_align_required",
        "yaw_align_succeeded",
        "final_pose_verified",
    ]:
        lines.append(f"- nav_{prefix}_{key}: {goal.get(key)}")

lines.append("")
for key in [
    "duration_wall_sec",
    "scan_count",
    "scan_hz",
    "bridge_status_count",
    "bridge_status_hz",
    "amcl_pose_messages_observed_by_recorder",
    "amcl_pose_count_delta_bridge",
    "amcl_candidate_count_delta_bridge",
    "amcl_shadow_candidate_count_delta_bridge",
    "amcl_accepted_count_delta_bridge",
    "amcl_rejected_count_delta_bridge",
    "amcl_suppressed_after_isaac_count_delta_bridge",
    "candidate_translation_m_max",
    "candidate_translation_m_last",
    "candidate_yaw_rad_max",
    "candidate_yaw_rad_last",
    "accepted_translation_m_max",
    "accepted_translation_m_last",
    "accepted_yaw_rad_max",
    "accepted_yaw_rad_last",
    "last_reject_reason_last",
    "last_accept_reason_last",
    "last_amcl_pose_age_ms_last",
    "last_amcl_xy_covariance_last",
    "map_to_odom_age_ms_last",
    "latest_odom_tf_age_ms_last",
    "odom_distance_accum_m",
    "cmd_nonzero_count",
    "cmd_nonzero_by_topic",
    "goal_state_last",
    "goal_phase_last",
    "goal_detail_last",
    "goal_nav2_result_code_last",
    "goal_final_distance_m_last",
    "goal_final_yaw_error_rad_last",
]:
    lines.append(f"- {key}: {summary.get(key)}")

(out_dir / "summary.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
print((out_dir / "summary.md").read_text(encoding="utf-8"))
PYSUMMARY

echo "${PREFIX} summary ${OUTPUT_DIR}/summary.md"
echo "${PREFIX} complete: ${OUTPUT_DIR}"
exit "${RECORDER_STATUS}"
