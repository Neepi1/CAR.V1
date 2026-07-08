#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/common_env.sh"

DURATION_SEC=60
LABEL="yaw_rate_alignment"
OUTPUT_ROOT="${NJRH_PROJECT_ROOT}/reports/local_state_yaw_rate_alignment"

usage() {
  cat <<'EOF'
Usage: record_yaw_rate_alignment_window.sh [--duration-sec N] [--label LABEL] [--output-root DIR]

Records wheel odom yaw rate and corrected IMU yaw rate for sign/scale checks.
This script is read-only and does not publish velocity or navigation goals.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --duration-sec)
      DURATION_SEC="${2:-}"
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
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "[yaw-rate-align] unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

safe_label="$(printf '%s' "${LABEL}" | tr -c 'A-Za-z0-9_.-' '_')"
timestamp="$(date -u +%Y%m%dT%H%M%SZ)"
report_dir="${OUTPUT_ROOT}/${timestamp}_${safe_label}_${DURATION_SEC}s"
mkdir -p "${report_dir}"
echo "[yaw-rate-align] report_dir=${report_dir}" >&2

python3 - "${DURATION_SEC}" "${report_dir}" <<'PY'
import csv
import json
import math
import statistics
import sys
import time
from pathlib import Path

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu

duration_sec = float(sys.argv[1])
report_dir = Path(sys.argv[2])


def sign(value, eps=0.02):
    if value > eps:
        return 1
    if value < -eps:
        return -1
    return 0


def stats(values):
    vals = [float(v) for v in values if math.isfinite(float(v))]
    if not vals:
        return {"count": 0, "mean": None, "min": None, "max": None}
    return {
        "count": len(vals),
        "mean": statistics.mean(vals),
        "min": min(vals),
        "max": max(vals),
    }


class Probe(Node):
    def __init__(self):
        super().__init__("record_yaw_rate_alignment_window")
        self.start_wall = time.time()
        self.last_wheel = None
        self.last_imu = None
        self.rows = []
        self.create_subscription(Odometry, "/wheel/odom_ekf", self.on_wheel, 50)
        self.create_subscription(Imu, "/lidar_imu_bias_corrected", self.on_imu, 100)
        self.csv_file = (report_dir / "samples.csv").open("w", newline="", encoding="utf-8")
        self.writer = csv.writer(self.csv_file)
        self.writer.writerow([
            "rel_time_sec",
            "wheel_wz_radps",
            "imu_wz_radps",
            "abs_diff_radps",
            "ratio_imu_over_wheel",
            "sign_match",
        ])
        self.create_timer(0.05, self.on_timer)

    def rel_time(self):
        return time.time() - self.start_wall

    def on_wheel(self, msg):
        self.last_wheel = float(msg.twist.twist.angular.z)

    def on_imu(self, msg):
        self.last_imu = float(msg.angular_velocity.z)

    def on_timer(self):
        if self.last_wheel is None or self.last_imu is None:
            return
        wheel = self.last_wheel
        imu = self.last_imu
        diff = abs(imu - wheel)
        ratio = None
        if abs(wheel) > 0.02:
            ratio = imu / wheel
        wheel_sign = sign(wheel)
        imu_sign = sign(imu)
        sign_match = None
        if wheel_sign != 0 or imu_sign != 0:
            sign_match = wheel_sign == imu_sign
        row = {
            "rel_time_sec": self.rel_time(),
            "wheel_wz_radps": wheel,
            "imu_wz_radps": imu,
            "abs_diff_radps": diff,
            "ratio_imu_over_wheel": ratio,
            "sign_match": sign_match,
        }
        self.rows.append(row)
        self.writer.writerow([
            f"{row['rel_time_sec']:.6f}",
            f"{wheel:.9f}",
            f"{imu:.9f}",
            f"{diff:.9f}",
            "" if ratio is None else f"{ratio:.9f}",
            "" if sign_match is None else str(sign_match).lower(),
        ])
        self.csv_file.flush()

    def close(self):
        self.csv_file.close()


rclpy.init()
node = Probe()
deadline = time.time() + duration_sec
try:
    while rclpy.ok() and time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
finally:
    moving_rows = [r for r in node.rows if abs(r["wheel_wz_radps"]) > 0.02 or abs(r["imu_wz_radps"]) > 0.02]
    sign_rows = [r for r in moving_rows if r["sign_match"] is not None]
    ratios = [r["ratio_imu_over_wheel"] for r in moving_rows if r["ratio_imu_over_wheel"] is not None]
    diffs = [r["abs_diff_radps"] for r in moving_rows]
    sign_match_rate = None
    if sign_rows:
        sign_match_rate = sum(1 for r in sign_rows if r["sign_match"]) / len(sign_rows)
    summary = {
        "duration_sec": duration_sec,
        "sample_count": len(node.rows),
        "moving_sample_count": len(moving_rows),
        "sign_compared_count": len(sign_rows),
        "sign_match_rate": sign_match_rate,
        "ratio_imu_over_wheel_stats": stats(ratios),
        "abs_diff_radps_stats": stats(diffs),
        "wheel_wz_radps_stats": stats([r["wheel_wz_radps"] for r in moving_rows]),
        "imu_wz_radps_stats": stats([r["imu_wz_radps"] for r in moving_rows]),
    }
    (report_dir / "summary.json").write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n")
    lines = [
        "# Yaw Rate Alignment Window",
        "",
        f"- duration_sec: `{duration_sec:.0f}`",
        f"- sample_count: `{summary['sample_count']}`",
        f"- moving_sample_count: `{summary['moving_sample_count']}`",
        f"- sign_match_rate: `{summary['sign_match_rate']}`",
        f"- ratio_imu_over_wheel_stats: `{summary['ratio_imu_over_wheel_stats']}`",
        f"- abs_diff_radps_stats: `{summary['abs_diff_radps_stats']}`",
        f"- wheel_wz_radps_stats: `{summary['wheel_wz_radps_stats']}`",
        f"- imu_wz_radps_stats: `{summary['imu_wz_radps_stats']}`",
        "",
        "Files:",
        "",
        "- `samples.csv`",
        "- `summary.json`",
    ]
    (report_dir / "summary.md").write_text("\n".join(lines) + "\n")
    node.close()
    node.destroy_node()
    rclpy.shutdown()
    print(report_dir)
PY
