#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/common_env.sh"

DURATION_SEC="${NJRH_AMCL_NAV_OBSERVE_DURATION_SEC:-180}"
LABEL="nav"
SEED_AMCL="false"
ALLOW_GATED="false"
OUT_ROOT="${NJRH_PROJECT_ROOT}/reports/amcl_shadow_navigation_observation"

usage() {
  cat <<'USAGE'
Usage: record_amcl_shadow_navigation_observation.sh [--duration-sec N] [--label NAME] [--seed-amcl] [--allow-gated]

Read-only by default. Start this script, then send navigation from the App/RViz.
It records AMCL shadow candidates, bridge status, local odom movement, and
commanded velocity. It does not publish velocity and does not change Nav2.

Options:
  --duration-sec N  Recording window, default 180 seconds.
  --label NAME      Label used in the report directory name.
  --seed-amcl       Publish /initialpose once through the bridge seed service
                    before recording. This is optional and off by default.
  --allow-gated     Do not fail precheck if AMCL is already in gated mode.
USAGE
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --duration-sec)
      [[ $# -ge 2 ]] || { echo "[amcl-nav-observe] --duration-sec requires a value" >&2; exit 2; }
      DURATION_SEC="$2"
      shift 2
      ;;
    --label)
      [[ $# -ge 2 ]] || { echo "[amcl-nav-observe] --label requires a value" >&2; exit 2; }
      LABEL="$2"
      shift 2
      ;;
    --seed-amcl)
      SEED_AMCL="true"
      shift
      ;;
    --allow-gated)
      ALLOW_GATED="true"
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "[amcl-nav-observe] unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

case "${DURATION_SEC}" in
  ''|*[!0-9]*) echo "[amcl-nav-observe] --duration-sec must be an integer" >&2; exit 2 ;;
esac

SAFE_LABEL="$(printf '%s' "${LABEL}" | tr -c 'A-Za-z0-9_.-' '_' | sed 's/^_*//;s/_*$//')"
[[ -n "${SAFE_LABEL}" ]] || SAFE_LABEL="nav"
RUN_ID="$(date -u +%Y%m%dT%H%M%SZ)_${SAFE_LABEL}_${DURATION_SEC}s"
OUT_DIR="${OUT_ROOT}/${RUN_ID}"
mkdir -p "${OUT_DIR}"

echo "[amcl-nav-observe] output: ${OUT_DIR}"

{
  echo "# command"
  printf '%q ' "$0" "$@"
  echo
  echo "# date_utc"
  date -u +%Y-%m-%dT%H:%M:%SZ
  echo "# nodes"
  ros2 node list 2>&1 || true
  echo "# services"
  ros2 service list -t 2>&1 | grep -E 'global_localization|robot_localization_bridge|amcl|trigger_grid' || true
  echo "# topics"
  for topic in /scan /amcl_pose /localization/bridge_status /local_state/odometry /cmd_vel_nav /cmd_vel_safe /cmd_vel; do
    echo "## ${topic}"
    ros2 topic info -v "${topic}" 2>&1 || true
  done
  echo "# params"
  for param in scan_topic tf_broadcast update_min_d update_min_a resample_interval save_pose_rate laser_model_type max_beams min_particles max_particles transform_tolerance; do
    printf '/amcl %s: ' "${param}"
    ros2 param get /amcl "${param}" 2>&1 || true
  done
  printf '/local_costmap/local_costmap global_frame: '
  ros2 param get /local_costmap/local_costmap global_frame 2>&1 || true
} >"${OUT_DIR}/preflight.txt"

if [[ "${SEED_AMCL}" == "true" ]]; then
  echo "[amcl-nav-observe] seeding AMCL from current bridge-approved map->base_link" | tee -a "${OUT_DIR}/preflight.txt"
  timeout 10 ros2 service call \
    /robot_localization_bridge/seed_amcl_initial_pose \
    std_srvs/srv/Trigger "{}" 2>&1 | tee -a "${OUT_DIR}/preflight.txt" || true
fi

python3 - "${OUT_DIR}" "${DURATION_SEC}" "${ALLOW_GATED}" <<'PY'
import csv
import json
import math
import sys
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

out_dir = Path(sys.argv[1])
duration_sec = float(sys.argv[2])
allow_gated = sys.argv[3].lower() == "true"

rclpy.init()
node = rclpy.create_node("amcl_shadow_navigation_observation")
start = node.get_clock().now().nanoseconds / 1e9

status_rows = []
amcl_rows = []
odom_rows = []
cmd_rows = []
tf_rows = []
scan_count = 0
cmd_nonzero_count = 0
odom_distance_m = 0.0
last_odom_xy = None

sensor_qos = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
)
tf_qos = QoSProfile(depth=100)


def now_sec():
    return node.get_clock().now().nanoseconds / 1e9


def yaw_from_q(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def on_scan(_):
    global scan_count
    scan_count += 1


def on_status(msg):
    try:
      data = json.loads(msg.data)
    except Exception as exc:
      status_rows.append({"t": now_sec() - start, "json_error": str(exc)})
      return
    row = {"t": now_sec() - start}
    for key in [
        "localization_mode", "gate_mode", "active_correction_source", "last_candidate_source",
        "last_accepted_source", "last_rejected_source", "last_accept_reason", "last_reject_reason",
        "amcl_input_enabled", "amcl_gate_mode", "amcl_pose_count", "amcl_candidate_count",
        "amcl_shadow_candidate_count", "amcl_accepted_count", "amcl_rejected_count",
        "amcl_suppressed_after_isaac_count", "amcl_last_state", "last_amcl_pose_age_ms",
        "last_amcl_xy_covariance", "last_amcl_yaw_covariance",
        "last_candidate_correction_translation_m", "last_candidate_correction_yaw_rad",
        "last_accepted_correction_translation_m", "last_accepted_correction_yaw_rad",
        "has_map_to_odom", "map_to_odom_publisher_owner", "map_to_odom_age_ms",
        "latest_odom_tf_fresh", "latest_odom_tf_age_ms", "last_odom_tf_history_lookup_ok",
        "isaac_background_correction_removed",
    ]:
        row[key] = data.get(key)
    status_rows.append(row)


def on_amcl(msg):
    amcl_rows.append({
        "t": now_sec() - start,
        "stamp_sec": msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
        "x": msg.pose.pose.position.x,
        "y": msg.pose.pose.position.y,
        "yaw": yaw_from_q(msg.pose.pose.orientation),
        "cov_x": msg.pose.covariance[0],
        "cov_y": msg.pose.covariance[7],
        "cov_yaw": msg.pose.covariance[35],
    })


def on_odom(msg):
    global odom_distance_m, last_odom_xy
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    yaw = yaw_from_q(msg.pose.pose.orientation)
    if last_odom_xy is not None:
        odom_distance_m += math.hypot(x - last_odom_xy[0], y - last_odom_xy[1])
    last_odom_xy = (x, y)
    odom_rows.append({
        "t": now_sec() - start,
        "stamp_sec": msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
        "x": x,
        "y": y,
        "yaw": yaw,
        "distance_accum_m": odom_distance_m,
    })


def make_cmd_cb(topic):
    def on_cmd(msg):
        global cmd_nonzero_count
        nonzero = abs(msg.linear.x) > 1e-3 or abs(msg.linear.y) > 1e-3 or abs(msg.angular.z) > 1e-3
        if nonzero:
            cmd_nonzero_count += 1
        cmd_rows.append({
            "t": now_sec() - start,
            "topic": topic,
            "linear_x": msg.linear.x,
            "linear_y": msg.linear.y,
            "angular_z": msg.angular.z,
            "nonzero": nonzero,
        })
    return on_cmd


def on_tf(msg):
    t = now_sec() - start
    for tf in msg.transforms:
        if (tf.header.frame_id, tf.child_frame_id) in {("map", "odom"), ("odom", "base_link")}:
            tf_rows.append({
                "t": t,
                "stamp_sec": tf.header.stamp.sec + tf.header.stamp.nanosec * 1e-9,
                "parent": tf.header.frame_id,
                "child": tf.child_frame_id,
                "x": tf.transform.translation.x,
                "y": tf.transform.translation.y,
                "yaw": yaw_from_q(tf.transform.rotation),
            })


node.create_subscription(LaserScan, "/scan", on_scan, sensor_qos)
node.create_subscription(String, "/localization/bridge_status", on_status, 10)
node.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", on_amcl, 10)
node.create_subscription(Odometry, "/local_state/odometry", on_odom, sensor_qos)
for topic in ["/cmd_vel_nav", "/cmd_vel_safe", "/cmd_vel"]:
    node.create_subscription(Twist, topic, make_cmd_cb(topic), sensor_qos)
node.create_subscription(TFMessage, "/tf", on_tf, tf_qos)

print(f"[amcl-nav-observe] recording {duration_sec:.0f}s; start navigation now if not already active")
end = start + duration_sec
while rclpy.ok() and now_sec() < end:
    rclpy.spin_once(node, timeout_sec=0.1)
actual_duration = max(0.001, now_sec() - start)
node.destroy_node()
rclpy.shutdown()


def write_csv(path, rows, default_fields):
    fields = sorted(set().union(*(row.keys() for row in rows))) if rows else default_fields
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


write_csv(out_dir / "bridge_status_samples.csv", status_rows, ["t"])
write_csv(out_dir / "amcl_pose_samples.csv", amcl_rows, ["t", "x", "y", "yaw"])
write_csv(out_dir / "local_state_odom_samples.csv", odom_rows, ["t", "x", "y", "yaw", "distance_accum_m"])
write_csv(out_dir / "cmd_vel_samples.csv", cmd_rows, ["t", "topic", "linear_x", "linear_y", "angular_z", "nonzero"])
write_csv(out_dir / "tf_samples.csv", tf_rows, ["t", "parent", "child", "x", "y", "yaw"])


def numeric_values(rows, key):
    values = []
    for row in rows:
        try:
            value = row.get(key)
            if value is not None and value != "":
                values.append(float(value))
        except Exception:
            pass
    return values


def last_value(rows, key, default=None):
    for row in reversed(rows):
        value = row.get(key)
        if value is not None and value != "":
            return value
    return default


translations = numeric_values(status_rows, "last_candidate_correction_translation_m")
yaws = numeric_values(status_rows, "last_candidate_correction_yaw_rad")
xy_covs = numeric_values(status_rows, "last_amcl_xy_covariance")
yaw_covs = numeric_values(status_rows, "last_amcl_yaw_covariance")
ages = numeric_values(status_rows, "last_amcl_pose_age_ms")
amcl_pose_counts = numeric_values(status_rows, "amcl_pose_count")
amcl_candidate_counts = numeric_values(status_rows, "amcl_candidate_count")
amcl_accepted_counts = numeric_values(status_rows, "amcl_accepted_count")

first_pose_count = amcl_pose_counts[0] if amcl_pose_counts else None
last_pose_count = amcl_pose_counts[-1] if amcl_pose_counts else None
pose_count_delta = (
    last_pose_count - first_pose_count
    if first_pose_count is not None and last_pose_count is not None else None
)
first_candidate_count = amcl_candidate_counts[0] if amcl_candidate_counts else None
last_candidate_count = amcl_candidate_counts[-1] if amcl_candidate_counts else None
candidate_count_delta = (
    last_candidate_count - first_candidate_count
    if first_candidate_count is not None and last_candidate_count is not None else None
)

last_status = status_rows[-1] if status_rows else {}
recommendation = "hold_shadow"
reasons = []
if not last_status.get("amcl_input_enabled"):
    reasons.append("bridge amcl_input_enabled is false")
if last_status.get("amcl_gate_mode") != "shadow" and not allow_gated:
    reasons.append(f"amcl_gate_mode is {last_status.get('amcl_gate_mode')}, expected shadow")
if not last_status.get("has_map_to_odom"):
    reasons.append("bridge has_map_to_odom is false")
if last_status.get("map_to_odom_publisher_owner") != "robot_localization_bridge":
    reasons.append("map->odom owner is not robot_localization_bridge")
if pose_count_delta is None or pose_count_delta < 3:
    reasons.append("too few new AMCL pose samples during the run")
if odom_distance_m < 0.3:
    reasons.append("robot did not move enough to evaluate AMCL while navigating")
if translations and max(translations) > 0.20:
    reasons.append("AMCL candidate translation exceeds current small-correction gate 0.20 m")
if yaws and max(yaws) > 0.20:
    reasons.append("AMCL candidate yaw exceeds current small-correction gate 0.20 rad")
if amcl_accepted_counts and max(amcl_accepted_counts) > 0 and last_status.get("amcl_gate_mode") == "shadow":
    reasons.append("AMCL accepted count increased while in shadow mode")
if not reasons:
    recommendation = "eligible_for_short_gated_trial"
    reasons.append("AMCL samples updated during motion and stayed within current small-correction gate")

summary = {
    "duration_sec": actual_duration,
    "scan_count": scan_count,
    "scan_hz": scan_count / actual_duration,
    "amcl_pose_messages_observed_by_recorder": len(amcl_rows),
    "amcl_pose_hz_observed_by_recorder": len(amcl_rows) / actual_duration,
    "bridge_status_count": len(status_rows),
    "bridge_status_hz": len(status_rows) / actual_duration,
    "odom_distance_accum_m": odom_distance_m,
    "cmd_nonzero_count": cmd_nonzero_count,
    "amcl_pose_count_first": first_pose_count,
    "amcl_pose_count_last": last_pose_count,
    "amcl_pose_count_delta": pose_count_delta,
    "amcl_candidate_count_first": first_candidate_count,
    "amcl_candidate_count_last": last_candidate_count,
    "amcl_candidate_count_delta": candidate_count_delta,
    "candidate_translation_m_max": max(translations) if translations else None,
    "candidate_translation_m_last": translations[-1] if translations else None,
    "candidate_yaw_rad_max": max(yaws) if yaws else None,
    "candidate_yaw_rad_last": yaws[-1] if yaws else None,
    "amcl_xy_covariance_max": max(xy_covs) if xy_covs else None,
    "amcl_xy_covariance_last": xy_covs[-1] if xy_covs else None,
    "amcl_yaw_covariance_max": max(yaw_covs) if yaw_covs else None,
    "amcl_yaw_covariance_last": yaw_covs[-1] if yaw_covs else None,
    "last_amcl_pose_age_ms_last": ages[-1] if ages else None,
    "last_status": last_status,
    "recommendation": recommendation,
    "reasons": reasons,
}
(out_dir / "summary.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")

md = [
    "# AMCL Shadow Navigation Observation",
    "",
    f"- duration_sec: {actual_duration:.1f}",
    f"- scan_hz: {summary['scan_hz']:.3f}",
    f"- bridge_status_hz: {summary['bridge_status_hz']:.3f}",
    f"- odom_distance_accum_m: {odom_distance_m:.3f}",
    f"- cmd_nonzero_count: {cmd_nonzero_count}",
    f"- amcl_pose_messages_observed_by_recorder: {len(amcl_rows)}",
    f"- amcl_pose_count_delta_bridge: {pose_count_delta}",
    f"- amcl_candidate_count_delta_bridge: {candidate_count_delta}",
    f"- candidate_translation_m_max: {summary['candidate_translation_m_max']}",
    f"- candidate_translation_m_last: {summary['candidate_translation_m_last']}",
    f"- candidate_yaw_rad_max: {summary['candidate_yaw_rad_max']}",
    f"- candidate_yaw_rad_last: {summary['candidate_yaw_rad_last']}",
    f"- amcl_xy_covariance_last: {summary['amcl_xy_covariance_last']}",
    f"- last_amcl_pose_age_ms_last: {summary['last_amcl_pose_age_ms_last']}",
    f"- recommendation: {recommendation}",
    "",
    "## Reasons",
    "",
]
md.extend([f"- {reason}" for reason in reasons])
md.extend([
    "",
    "## Last Bridge Status",
    "",
    "```json",
    json.dumps(last_status, indent=2, ensure_ascii=False),
    "```",
])
(out_dir / "summary.md").write_text("\n".join(md) + "\n", encoding="utf-8")
print(f"[amcl-nav-observe] wrote {out_dir}")
print((out_dir / "summary.md").read_text(encoding="utf-8"))
PY

echo "[amcl-nav-observe] complete: ${OUT_DIR}"
