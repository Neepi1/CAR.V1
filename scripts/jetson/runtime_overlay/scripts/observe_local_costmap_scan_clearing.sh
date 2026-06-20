#!/usr/bin/env bash
set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="${WORKSPACE_ROOT:-$(cd "${SCRIPT_DIR}/../../../.." && pwd)}"
export NJRH_PROJECT_ROOT="${NJRH_PROJECT_ROOT:-${WORKSPACE_ROOT}}"
source "${SCRIPT_DIR}/common_env.sh"
set +e

PREFIX="[scan-clearing-observe]"
DURATION_SEC=60
SAMPLE_PERIOD_SEC=1.0
LABEL="scan_clearing"
OUTPUT_DIR=""
SCAN_TOPIC="${SCAN_TOPIC:-/scan}"
COSTMAP_TOPIC="${COSTMAP_TOPIC:-/local_costmap/costmap}"
COSTMAP_FRAME="${COSTMAP_FRAME:-odom}"
SENSOR_FRAME="${SENSOR_FRAME:-lidar_level_link}"
BASE_FRAME="${BASE_FRAME:-base_link}"
OBSTACLE_MIN_RANGE_M=0.25
OBSTACLE_MAX_RANGE_M=5.50
RAYTRACE_MIN_RANGE_M=0.25
RAYTRACE_MAX_RANGE_M=8.00
CLASSIFICATION_RADIUS_M=4.0
RECENT_SCAN_WINDOW=45
ENDPOINT_TOLERANCE_M=0.22
BEHIND_TOLERANCE_M=0.12

usage() {
  cat <<'EOF'
Usage:
  bash scripts/jetson/runtime_overlay/scripts/observe_local_costmap_scan_clearing.sh \
    --duration-sec 90 \
    --label person_walked_away

Read-only observer for the Nav2 /scan ObstacleLayer clearing question.

Start this script, reproduce the case where a person or obstacle walks around
the robot and then leaves, and let the script finish. It records /scan,
/local_costmap/costmap, and TF, then classifies occupied local-costmap cells as:
  - supported_by_current_scan_endpoint: current /scan still marks that cell
  - behind_current_scan_endpoint_blocked: a nearer current /scan point blocks clearing
  - inside_current_clear_ray_but_still_occupied: current /scan should clear it
  - too_near_to_sensor / outside_*: outside the configured clearing geometry

The script does not publish topics, send goals, call services, clear costmaps,
set params, or restart nodes.

Options:
  --duration-sec N              Capture duration in seconds. Default: 60.
  --sample-period-sec N         Summary sample period. Default: 1.0.
  --label LABEL                 Report label. Default: scan_clearing.
  --output-dir DIR              Report directory. Default: reports/local_costmap_scan_clearing/<timestamp>_<label>_<duration>s.
  --scan-topic TOPIC            LaserScan topic. Default: /scan.
  --costmap-topic TOPIC         OccupancyGrid topic. Default: /local_costmap/costmap.
  --costmap-frame FRAME         Costmap frame. Default: odom.
  --sensor-frame FRAME          Expected scan frame. Default: lidar_level_link.
  --base-frame FRAME            Robot base frame. Default: base_link.
  --obstacle-min-range M        Obstacle min range. Default: 0.25.
  --obstacle-max-range M        Obstacle max range. Default: 5.50.
  --raytrace-min-range M        Raytrace min range. Default: 0.25.
  --raytrace-max-range M        Raytrace max range. Default: 8.00.
  --classification-radius-m M   Analyze occupied cells within this base_link radius. Default: 4.0.
  --recent-scan-window N        Use last N scans for endpoint support. Default: 45.
  --endpoint-tolerance-m M      Endpoint matching tolerance. Default: 0.22.
  --behind-tolerance-m M        Distance margin for "blocked behind endpoint". Default: 0.12.
  -h, --help                    Show this help.
EOF
}

sanitize_label() {
  printf '%s' "$1" | tr -c 'A-Za-z0-9_.-' '_'
}

is_number() {
  [[ "${1:-}" =~ ^[0-9]+([.][0-9]+)?$ ]]
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
    --scan-topic)
      SCAN_TOPIC="${2:-}"
      shift 2
      ;;
    --costmap-topic)
      COSTMAP_TOPIC="${2:-}"
      shift 2
      ;;
    --costmap-frame)
      COSTMAP_FRAME="${2:-}"
      shift 2
      ;;
    --sensor-frame)
      SENSOR_FRAME="${2:-}"
      shift 2
      ;;
    --base-frame)
      BASE_FRAME="${2:-}"
      shift 2
      ;;
    --obstacle-min-range)
      OBSTACLE_MIN_RANGE_M="${2:-}"
      shift 2
      ;;
    --obstacle-max-range)
      OBSTACLE_MAX_RANGE_M="${2:-}"
      shift 2
      ;;
    --raytrace-min-range)
      RAYTRACE_MIN_RANGE_M="${2:-}"
      shift 2
      ;;
    --raytrace-max-range)
      RAYTRACE_MAX_RANGE_M="${2:-}"
      shift 2
      ;;
    --classification-radius-m)
      CLASSIFICATION_RADIUS_M="${2:-}"
      shift 2
      ;;
    --recent-scan-window)
      RECENT_SCAN_WINDOW="${2:-}"
      shift 2
      ;;
    --endpoint-tolerance-m)
      ENDPOINT_TOLERANCE_M="${2:-}"
      shift 2
      ;;
    --behind-tolerance-m)
      BEHIND_TOLERANCE_M="${2:-}"
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

if ! [[ "${DURATION_SEC}" =~ ^[0-9]+$ ]] || [[ "${DURATION_SEC}" -lt 5 ]]; then
  echo "${PREFIX} FAIL --duration-sec must be an integer >= 5" >&2
  exit 2
fi

if ! is_number "${SAMPLE_PERIOD_SEC}" ||
   ! is_number "${OBSTACLE_MIN_RANGE_M}" ||
   ! is_number "${OBSTACLE_MAX_RANGE_M}" ||
   ! is_number "${RAYTRACE_MIN_RANGE_M}" ||
   ! is_number "${RAYTRACE_MAX_RANGE_M}" ||
   ! is_number "${CLASSIFICATION_RADIUS_M}" ||
   ! is_number "${ENDPOINT_TOLERANCE_M}" ||
   ! is_number "${BEHIND_TOLERANCE_M}"; then
  echo "${PREFIX} FAIL numeric options must be positive decimal values" >&2
  exit 2
fi

if ! [[ "${RECENT_SCAN_WINDOW}" =~ ^[0-9]+$ ]] || [[ "${RECENT_SCAN_WINDOW}" -lt 1 ]]; then
  echo "${PREFIX} FAIL --recent-scan-window must be an integer >= 1" >&2
  exit 2
fi

TIMESTAMP="$(date -u +%Y%m%dT%H%M%SZ)"
LABEL="$(sanitize_label "${LABEL}")"
if [[ -z "${OUTPUT_DIR}" ]]; then
  OUTPUT_DIR="${NJRH_PROJECT_ROOT}/reports/local_costmap_scan_clearing/${TIMESTAMP}_${LABEL}_${DURATION_SEC}s"
fi
mkdir -p "${OUTPUT_DIR}"

echo "${PREFIX} report_dir=${OUTPUT_DIR}"
echo "${PREFIX} duration_sec=${DURATION_SEC} sample_period_sec=${SAMPLE_PERIOD_SEC}"
echo "${PREFIX} scan_topic=${SCAN_TOPIC} costmap_topic=${COSTMAP_TOPIC}"
echo "${PREFIX} read-only: no goals, no params, no services, no costmap clear, no restarts"
echo "${PREFIX} reproduce the obstacle-walk-away case now, then wait for the summary"

python3 - \
  "${DURATION_SEC}" \
  "${SAMPLE_PERIOD_SEC}" \
  "${OUTPUT_DIR}" \
  "${SCAN_TOPIC}" \
  "${COSTMAP_TOPIC}" \
  "${COSTMAP_FRAME}" \
  "${SENSOR_FRAME}" \
  "${BASE_FRAME}" \
  "${OBSTACLE_MIN_RANGE_M}" \
  "${OBSTACLE_MAX_RANGE_M}" \
  "${RAYTRACE_MIN_RANGE_M}" \
  "${RAYTRACE_MAX_RANGE_M}" \
  "${CLASSIFICATION_RADIUS_M}" \
  "${RECENT_SCAN_WINDOW}" \
  "${ENDPOINT_TOLERANCE_M}" \
  "${BEHIND_TOLERANCE_M}" <<'PY'
import csv
import json
import math
import sys
import time
from collections import Counter, deque
from pathlib import Path

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
import tf2_ros


duration_sec = float(sys.argv[1])
sample_period_sec = float(sys.argv[2])
output_dir = Path(sys.argv[3])
scan_topic = sys.argv[4]
costmap_topic = sys.argv[5]
costmap_frame_expected = sys.argv[6]
sensor_frame_expected = sys.argv[7]
base_frame = sys.argv[8]
obstacle_min_range = float(sys.argv[9])
obstacle_max_range = float(sys.argv[10])
raytrace_min_range = float(sys.argv[11])
raytrace_max_range = float(sys.argv[12])
classification_radius_m = float(sys.argv[13])
recent_scan_window = int(sys.argv[14])
endpoint_tolerance_m = float(sys.argv[15])
behind_tolerance_m = float(sys.argv[16])

output_dir.mkdir(parents=True, exist_ok=True)
samples_csv = output_dir / "samples.csv"
classifications_jsonl = output_dir / "classifications.jsonl"
summary_md = output_dir / "summary.md"
meta_json = output_dir / "meta.json"


class Probe(Node):
    def __init__(self):
        super().__init__("local_costmap_scan_clearing_observer", enable_rosout=False)
        self.scans = deque(maxlen=max(recent_scan_window, 1))
        self.costmap = None
        self.tf_errors = Counter()
        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.create_subscription(LaserScan, scan_topic, self.on_scan, qos_profile_sensor_data)
        self.create_subscription(OccupancyGrid, costmap_topic, self.on_costmap, 10)

    def on_scan(self, msg):
        self.scans.append(msg)

    def on_costmap(self, msg):
        self.costmap = msg


def stamp_age_ms(node, stamp):
    now_ns = node.get_clock().now().nanoseconds
    stamp_ns = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
    return (now_ns - stamp_ns) / 1.0e6


def q_to_mat(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z
    return [
        [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
        [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
        [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
    ]


def inverse_transform_point(tf_msg, x, y, z=0.0):
    t = tf_msg.transform.translation
    r = q_to_mat(tf_msg.transform.rotation)
    dx, dy, dz = x - t.x, y - t.y, z - t.z
    return (
        r[0][0] * dx + r[1][0] * dy + r[2][0] * dz,
        r[0][1] * dx + r[1][1] * dy + r[2][1] * dz,
        r[0][2] * dx + r[1][2] * dy + r[2][2] * dz,
    )


def finite_stats(values):
    finite = [value for value in values if math.isfinite(value)]
    if not finite:
        return {
            "finite": 0,
            "inf": sum(1 for value in values if math.isinf(value)),
            "nan": sum(1 for value in values if math.isnan(value)),
            "min": -1.0,
            "median": -1.0,
            "le_0_5": 0,
            "le_1_0": 0,
            "le_1_5": 0,
            "le_2_0": 0,
            "le_3_0": 0,
            "le_obstacle_max": 0,
            "le_raytrace_max": 0,
        }
    sorted_values = sorted(finite)
    return {
        "finite": len(finite),
        "inf": sum(1 for value in values if math.isinf(value)),
        "nan": sum(1 for value in values if math.isnan(value)),
        "min": sorted_values[0],
        "median": sorted_values[len(sorted_values) // 2],
        "le_0_5": sum(1 for value in finite if value <= 0.5),
        "le_1_0": sum(1 for value in finite if value <= 1.0),
        "le_1_5": sum(1 for value in finite if value <= 1.5),
        "le_2_0": sum(1 for value in finite if value <= 2.0),
        "le_3_0": sum(1 for value in finite if value <= 3.0),
        "le_obstacle_max": sum(1 for value in finite if value <= obstacle_max_range),
        "le_raytrace_max": sum(1 for value in finite if value <= raytrace_max_range),
    }


def build_recent_scan_min(scans, bin_count):
    min_ranges = [math.inf] * bin_count
    endpoint_counts = [0] * bin_count
    inf_counts = [0] * bin_count
    for scan in scans:
        for index, value in enumerate(scan.ranges[:bin_count]):
            if math.isfinite(value):
                if value < min_ranges[index]:
                    min_ranges[index] = value
                if obstacle_min_range <= value <= obstacle_max_range:
                    endpoint_counts[index] += 1
            elif math.isinf(value):
                inf_counts[index] += 1
    return min_ranges, endpoint_counts, inf_counts


def classify_cell(scan, min_ranges, tf_costmap_from_scan, wx, wy):
    sx, sy, _ = inverse_transform_point(tf_costmap_from_scan, wx, wy, 0.0)
    radius = math.hypot(sx, sy)
    angle = math.atan2(sy, sx)
    if radius < raytrace_min_range:
        return "too_near_to_sensor", radius, angle
    if angle < scan.angle_min or angle > scan.angle_max:
        return "outside_scan_angle", radius, angle
    if radius > raytrace_max_range + endpoint_tolerance_m:
        return "outside_raytrace_range", radius, angle
    index = int((angle - scan.angle_min) / scan.angle_increment)
    if index < 0 or index >= len(scan.ranges):
        return "outside_scan_bin", radius, angle
    min_range = min_ranges[index]
    endpoint = (
        math.isfinite(min_range)
        and obstacle_min_range <= min_range <= obstacle_max_range
        and abs(radius - min_range) <= endpoint_tolerance_m
    )
    blocked = (
        math.isfinite(min_range)
        and obstacle_min_range <= min_range <= obstacle_max_range
        and min_range < radius - behind_tolerance_m
    )
    clear_reaches = (
        (math.isinf(min_range) or min_range > radius + behind_tolerance_m or min_range > obstacle_max_range)
        and radius <= raytrace_max_range
    )
    if endpoint:
        return "supported_by_current_scan_endpoint", radius, angle
    if blocked:
        return "behind_current_scan_endpoint_blocked", radius, angle
    if clear_reaches:
        return "inside_current_clear_ray_but_still_occupied", radius, angle
    return "ambiguous_ray_boundary", radius, angle


def analyze(node):
    if not node.scans or node.costmap is None:
        return None
    scan = node.scans[-1]
    costmap = node.costmap
    try:
        tf_costmap_from_scan = node.tf_buffer.lookup_transform(
            costmap.header.frame_id,
            scan.header.frame_id,
            Time.from_msg(scan.header.stamp),
            timeout=Duration(seconds=0.25),
        )
        tf_mode = "scan_stamp"
    except Exception as exc:
        node.tf_errors[type(exc).__name__] += 1
        try:
            tf_costmap_from_scan = node.tf_buffer.lookup_transform(
                costmap.header.frame_id,
                scan.header.frame_id,
                Time(),
                timeout=Duration(seconds=0.50),
            )
            tf_mode = f"latest_fallback:{type(exc).__name__}"
        except Exception as fallback_exc:
            node.tf_errors[type(fallback_exc).__name__] += 1
            return {"error": f"tf_lookup_failed:{type(fallback_exc).__name__}"}
    try:
        tf_costmap_from_base = node.tf_buffer.lookup_transform(
            costmap.header.frame_id,
            base_frame,
            Time(),
            timeout=Duration(seconds=0.25),
        )
        base_x = tf_costmap_from_base.transform.translation.x
        base_y = tf_costmap_from_base.transform.translation.y
        base_source = "tf"
    except Exception as exc:
        node.tf_errors[type(exc).__name__] += 1
        base_x = costmap.info.origin.position.x + costmap.info.width * costmap.info.resolution / 2.0
        base_y = costmap.info.origin.position.y + costmap.info.height * costmap.info.resolution / 2.0
        base_source = "costmap_center_fallback"

    bin_count = len(scan.ranges)
    min_ranges, _, _ = build_recent_scan_min(list(node.scans), bin_count)
    scan_stats = finite_stats(list(scan.ranges))
    width = costmap.info.width
    height = costmap.info.height
    resolution = costmap.info.resolution
    origin_x = costmap.info.origin.position.x
    origin_y = costmap.info.origin.position.y

    classes_lethal = Counter()
    classes_occ50 = Counter()
    examples = {}
    near_radii = {
        "r0_5": [0, 0],
        "r1_0": [0, 0],
        "r1_5": [0, 0],
        "r2_0": [0, 0],
        "r3_0": [0, 0],
        "r4_0": [0, 0],
    }
    radius_values = {
        "r0_5": 0.5,
        "r1_0": 1.0,
        "r1_5": 1.5,
        "r2_0": 2.0,
        "r3_0": 3.0,
        "r4_0": 4.0,
    }
    occ50_near = 0
    lethal_near = 0
    for y in range(height):
        wy = origin_y + (float(y) + 0.5) * resolution
        dy = wy - base_y
        if abs(dy) > classification_radius_m:
            continue
        base_index = y * width
        for x in range(width):
            value = costmap.data[base_index + x]
            if value < 50:
                continue
            wx = origin_x + (float(x) + 0.5) * resolution
            dist_base = math.hypot(wx - base_x, dy)
            if dist_base > classification_radius_m:
                continue
            occ50_near += 1
            if value >= 100:
                lethal_near += 1
            for key, radius_limit in radius_values.items():
                if dist_base <= radius_limit:
                    near_radii[key][0] += 1
                    if value >= 100:
                        near_radii[key][1] += 1
            class_name, scan_radius, scan_angle = classify_cell(scan, min_ranges, tf_costmap_from_scan, wx, wy)
            if value >= 100:
                classes_lethal[class_name] += 1
                examples.setdefault(
                    class_name,
                    {
                        "costmap_x": round(wx, 3),
                        "costmap_y": round(wy, 3),
                        "dist_base_m": round(dist_base, 3),
                        "scan_radius_m": round(scan_radius, 3),
                        "scan_angle_rad": round(scan_angle, 3),
                        "cost": int(value),
                    },
                )
            else:
                classes_occ50[class_name] += 1

    return {
        "error": "",
        "scan_frame": scan.header.frame_id,
        "costmap_frame": costmap.header.frame_id,
        "tf_mode": tf_mode,
        "base_source": base_source,
        "scan_age_ms": round(stamp_age_ms(node, scan.header.stamp), 3),
        "costmap_age_ms": round(stamp_age_ms(node, costmap.header.stamp), 3),
        "scan_stats": scan_stats,
        "costmap_width": width,
        "costmap_height": height,
        "costmap_resolution_m": resolution,
        "base_x": round(base_x, 3),
        "base_y": round(base_y, 3),
        "occ50_near": occ50_near,
        "lethal_near": lethal_near,
        "near_radii": near_radii,
        "classes_lethal": dict(classes_lethal),
        "classes_occ50": dict(classes_occ50),
        "examples": examples,
    }


def value(mapping, key, default=0):
    return mapping.get(key, default)


metadata = {
    "duration_sec": duration_sec,
    "sample_period_sec": sample_period_sec,
    "scan_topic": scan_topic,
    "costmap_topic": costmap_topic,
    "costmap_frame_expected": costmap_frame_expected,
    "sensor_frame_expected": sensor_frame_expected,
    "base_frame": base_frame,
    "obstacle_min_range_m": obstacle_min_range,
    "obstacle_max_range_m": obstacle_max_range,
    "raytrace_min_range_m": raytrace_min_range,
    "raytrace_max_range_m": raytrace_max_range,
    "classification_radius_m": classification_radius_m,
    "recent_scan_window": recent_scan_window,
    "endpoint_tolerance_m": endpoint_tolerance_m,
    "behind_tolerance_m": behind_tolerance_m,
    "read_only": True,
    "calls_services": False,
    "sets_params": False,
    "publishes_control": False,
    "clears_costmap": False,
    "subscribes_tf": True,
    "subscribes_laserscan": True,
    "subscribes_costmap": True,
}
meta_json.write_text(json.dumps(metadata, indent=2, sort_keys=True), encoding="utf-8")

rclpy.init()
node = Probe()
start = time.monotonic()
deadline = start + duration_sec
next_sample = start
rows = []
last_result = None

fieldnames = [
    "elapsed_sec",
    "error",
    "scan_frame",
    "costmap_frame",
    "tf_mode",
    "base_source",
    "scan_age_ms",
    "costmap_age_ms",
    "scan_finite",
    "scan_inf",
    "scan_nan",
    "scan_min_m",
    "scan_median_m",
    "scan_le_1_5",
    "scan_le_3_0",
    "scan_le_obstacle_max",
    "scan_le_raytrace_max",
    "occ50_near",
    "lethal_near",
    "lethal_supported",
    "lethal_blocked",
    "lethal_clear_missed",
    "lethal_too_near",
    "lethal_outside",
    "lethal_ambiguous",
    "occ50_supported",
    "occ50_blocked",
    "occ50_clear_missed",
    "occ50_too_near",
    "occ50_outside",
    "occ50_ambiguous",
]

with samples_csv.open("w", newline="", encoding="utf-8") as csv_file, classifications_jsonl.open(
    "w", encoding="utf-8"
) as jsonl_file:
    writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
    writer.writeheader()
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        now = time.monotonic()
        if now < next_sample:
            continue
        result = analyze(node)
        elapsed = now - start
        next_sample += sample_period_sec
        if result is None:
            row = {"elapsed_sec": round(elapsed, 3), "error": "waiting_for_scan_or_costmap"}
            writer.writerow(row)
            rows.append(row)
            continue
        if result.get("error"):
            row = {"elapsed_sec": round(elapsed, 3), "error": result["error"]}
            writer.writerow(row)
            rows.append(row)
            continue
        last_result = result
        lethal = result["classes_lethal"]
        occ50 = result["classes_occ50"]
        row = {
            "elapsed_sec": round(elapsed, 3),
            "error": "",
            "scan_frame": result["scan_frame"],
            "costmap_frame": result["costmap_frame"],
            "tf_mode": result["tf_mode"],
            "base_source": result["base_source"],
            "scan_age_ms": result["scan_age_ms"],
            "costmap_age_ms": result["costmap_age_ms"],
            "scan_finite": result["scan_stats"]["finite"],
            "scan_inf": result["scan_stats"]["inf"],
            "scan_nan": result["scan_stats"]["nan"],
            "scan_min_m": round(result["scan_stats"]["min"], 3),
            "scan_median_m": round(result["scan_stats"]["median"], 3),
            "scan_le_1_5": result["scan_stats"]["le_1_5"],
            "scan_le_3_0": result["scan_stats"]["le_3_0"],
            "scan_le_obstacle_max": result["scan_stats"]["le_obstacle_max"],
            "scan_le_raytrace_max": result["scan_stats"]["le_raytrace_max"],
            "occ50_near": result["occ50_near"],
            "lethal_near": result["lethal_near"],
            "lethal_supported": value(lethal, "supported_by_current_scan_endpoint"),
            "lethal_blocked": value(lethal, "behind_current_scan_endpoint_blocked"),
            "lethal_clear_missed": value(lethal, "inside_current_clear_ray_but_still_occupied"),
            "lethal_too_near": value(lethal, "too_near_to_sensor"),
            "lethal_outside": (
                value(lethal, "outside_scan_angle")
                + value(lethal, "outside_scan_bin")
                + value(lethal, "outside_raytrace_range")
            ),
            "lethal_ambiguous": value(lethal, "ambiguous_ray_boundary"),
            "occ50_supported": value(occ50, "supported_by_current_scan_endpoint"),
            "occ50_blocked": value(occ50, "behind_current_scan_endpoint_blocked"),
            "occ50_clear_missed": value(occ50, "inside_current_clear_ray_but_still_occupied"),
            "occ50_too_near": value(occ50, "too_near_to_sensor"),
            "occ50_outside": (
                value(occ50, "outside_scan_angle")
                + value(occ50, "outside_scan_bin")
                + value(occ50, "outside_raytrace_range")
            ),
            "occ50_ambiguous": value(occ50, "ambiguous_ray_boundary"),
        }
        writer.writerow(row)
        rows.append(row)
        jsonl_file.write(json.dumps({"elapsed_sec": round(elapsed, 3), **result}, sort_keys=True) + "\n")
        jsonl_file.flush()
        print(
            "[scan-clearing-observe] t={:.1f}s scan_finite={} le3m={} occ50={} lethal={} "
            "lethal_supported={} lethal_blocked={} lethal_clear_missed={}".format(
                elapsed,
                row["scan_finite"],
                row["scan_le_3_0"],
                row["occ50_near"],
                row["lethal_near"],
                row["lethal_supported"],
                row["lethal_blocked"],
                row["lethal_clear_missed"],
            ),
            flush=True,
        )

valid_rows = [row for row in rows if not row.get("error")]
errors = [row.get("error") for row in rows if row.get("error")]
conclusion = "insufficient_data"
if valid_rows:
    last = valid_rows[-1]
    scan_marking = int(last["lethal_supported"]) + int(last["lethal_blocked"])
    clear_missed = int(last["lethal_clear_missed"])
    if scan_marking > clear_missed * 2 and scan_marking > 0:
        conclusion = "current_scan_is_still_marking_or_blocking_most_lethal_cells"
    elif clear_missed > scan_marking and clear_missed > 0:
        conclusion = "clear_rays_should_reach_many_lethal_cells_but_cells_remain"
    elif int(last["lethal_near"]) == 0 and int(last["occ50_near"]) == 0:
        conclusion = "local_costmap_near_robot_is_clear"
    else:
        conclusion = "mixed_or_ambiguous"

with summary_md.open("w", encoding="utf-8") as f:
    f.write("# Local Costmap Scan Clearing Observation\n\n")
    f.write("## Contract\n\n")
    f.write("- Read-only: no goals, params, services, costmap clear, restarts, or control publications.\n")
    f.write(f"- scan_topic: `{scan_topic}`\n")
    f.write(f"- costmap_topic: `{costmap_topic}`\n")
    f.write(f"- obstacle range: `{obstacle_min_range:.2f}..{obstacle_max_range:.2f} m`\n")
    f.write(f"- raytrace range: `{raytrace_min_range:.2f}..{raytrace_max_range:.2f} m`\n")
    f.write(f"- classification_radius_m: `{classification_radius_m:.2f}`\n")
    f.write(f"- recent_scan_window: `{recent_scan_window}`\n\n")
    f.write("## Result\n\n")
    f.write(f"- conclusion: `{conclusion}`\n")
    f.write(f"- samples: `{samples_csv}`\n")
    f.write(f"- classifications: `{classifications_jsonl}`\n")
    f.write(f"- tf_errors: `{dict(node.tf_errors)}`\n")
    if errors:
        f.write(f"- sample_errors: `{Counter(errors)}`\n")
    f.write("\n")
    if valid_rows:
        last = valid_rows[-1]
        f.write("## Last Sample\n\n")
        for key in fieldnames:
            if key in last:
                f.write(f"- {key}: `{last[key]}`\n")
        f.write("\n")
    if last_result:
        f.write("## Last Lethal Classification\n\n")
        for key, count in sorted(last_result["classes_lethal"].items(), key=lambda item: item[1], reverse=True):
            f.write(f"- {key}: `{count}`")
            example = last_result["examples"].get(key)
            if example:
                f.write(f" example=`{example}`")
            f.write("\n")
        f.write("\n")
        f.write("## Interpretation\n\n")
        f.write(
            "- If `supported_by_current_scan_endpoint` and `behind_current_scan_endpoint_blocked` dominate, "
            "the costmap is being refreshed by current `/scan` geometry; inspect scan generation height/filtering/self-mask.\n"
        )
        f.write(
            "- If `inside_current_clear_ray_but_still_occupied` dominates, current `/scan` should be clearing those cells; "
            "inspect ObstacleLayer raytracing, TF timing, and costmap update behavior.\n"
        )
        f.write(
            "- If `too_near_to_sensor` dominates, the cells are inside raytrace_min_range or the sensor origin/footprint region.\n"
        )

node.destroy_node()
rclpy.shutdown()

if not valid_rows:
    print(f"[scan-clearing-observe] FAIL no valid samples; summary {summary_md}", file=sys.stderr)
    sys.exit(1)

print(f"[scan-clearing-observe] summary {summary_md}")
PY

status=$?
if [[ "${status}" -ne 0 ]]; then
  echo "${PREFIX} FAIL observer exited with status=${status}" >&2
  exit "${status}"
fi

echo "${PREFIX} complete: ${OUTPUT_DIR}"
