#!/usr/bin/env bash
set -euo pipefail
umask 0002

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

ANGLES_DEG="+30,+60,+90,+120,+150,+180,+210,+240,+270,+300,+330,+360,-30,-60,-90,-120,-150,-180,-210,-240,-270,-300,-330,-360"
REPEAT="5"
ANGULAR_SPEED_RADPS="0.60"
COUNTDOWN_SEC="0.0"
BIAS_SEC="2.0"
SETTLE_SEC="3.0"
SAMPLE_HZ="20.0"
LABEL="spin_yaw_scale_calibration"
CMD_TOPIC="/cmd_vel_collision_checked"
ODOM_TOPIC="/wheel/odom"
IMU_TOPIC="/lidar_imu"
OUTPUT_ROOT="${NJRH_TEST_OUTPUT_ROOT:-/tmp/ranger_spin_yaw_scale_calibration}"

usage() {
  cat <<'EOF'
Usage: run_ranger_spin_yaw_scale_calibration.sh [options]

Runs a batch Ranger Mini 3 spin yaw calibration against IMU yaw-rate
integration. It does not trigger Isaac, AMCL, or relocalization. Each segment
uses the existing safety chain:

  script -> /cmd_vel_collision_checked -> robot_safety -> /cmd_vel -> ranger_base

The fitted scale is:

  spinning_yaw_scale = imu_delta_yaw / wheel_delta_yaw

Positive and negative yaw are fitted separately with a weighted least-squares
ratio and median-ratio diagnostics.

Options:
  --angles-deg LIST       Comma-separated signed target angles.
                          Default: +/-30..+/-360 in 30 deg steps.
  --repeat N              Repeat count for the full angle list. Default: 5
  --angular-speed RADPS   Absolute spin command. Default: 0.60
  --countdown-sec SEC     Per-segment countdown passed to the single-spin script. Default: 0
  --bias-sec SEC          Per-segment stationary IMU bias window. Default: 2
  --settle-sec SEC        Per-segment stop settling window. Default: 3
  --sample-hz HZ          Per-segment sample rate. Default: 20
  --label NAME            Report label. Default: spin_yaw_scale_calibration
  --cmd-topic TOPIC       Safety-chain input topic. Default: /cmd_vel_collision_checked
  --odom-topic TOPIC      Wheel odom topic. Default: /wheel/odom
  --imu-topic TOPIC       IMU topic. Default: /lidar_imu
  --output-root DIR       Report root. Default: /tmp/ranger_spin_yaw_scale_calibration
  -h, --help              Show this help
EOF
}

while [[ "$#" -gt 0 ]]; do
  case "$1" in
    --angles-deg)
      ANGLES_DEG="${2:-}"
      shift 2
      ;;
    --repeat)
      REPEAT="${2:-}"
      shift 2
      ;;
    --angular-speed|--speed)
      ANGULAR_SPEED_RADPS="${2:-}"
      shift 2
      ;;
    --countdown-sec)
      COUNTDOWN_SEC="${2:-}"
      shift 2
      ;;
    --bias-sec)
      BIAS_SEC="${2:-}"
      shift 2
      ;;
    --settle-sec)
      SETTLE_SEC="${2:-}"
      shift 2
      ;;
    --sample-hz)
      SAMPLE_HZ="${2:-}"
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
    --imu-topic)
      IMU_TOPIC="${2:-}"
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
      echo "[spin-yaw-scale-calib] unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

if [[ -f "${SCRIPT_DIR}/common_env.sh" ]]; then
  # shellcheck source=/dev/null
  source "${SCRIPT_DIR}/common_env.sh"
fi

SPIN_IMU_SCRIPT="${SCRIPT_DIR}/run_ranger_spin_imu_yaw_test.sh"
if [[ ! -f "${SPIN_IMU_SCRIPT}" ]]; then
  echo "[spin-yaw-scale-calib] missing required script: ${SPIN_IMU_SCRIPT}" >&2
  exit 1
fi

safe_label="$(printf '%s' "${LABEL}" | tr -c 'A-Za-z0-9_.-' '_')"
timestamp="$(date -u +%Y%m%dT%H%M%SZ)"
OUTPUT_ROOT="${OUTPUT_ROOT/#\~/${HOME}}"
OUT_DIR="${OUTPUT_ROOT}/${timestamp}_${safe_label}"
SEGMENT_ROOT="${OUT_DIR}/segments"
mkdir -p "${SEGMENT_ROOT}"

{
  echo "# Ranger Spin Yaw Scale Calibration Environment"
  echo
  echo "- timestamp_utc: ${timestamp}"
  echo "- angles_deg: ${ANGLES_DEG}"
  echo "- repeat: ${REPEAT}"
  echo "- angular_speed_radps: ${ANGULAR_SPEED_RADPS}"
  echo "- countdown_sec: ${COUNTDOWN_SEC}"
  echo "- bias_sec: ${BIAS_SEC}"
  echo "- settle_sec: ${SETTLE_SEC}"
  echo "- sample_hz: ${SAMPLE_HZ}"
  echo "- cmd_topic: ${CMD_TOPIC}"
  echo "- odom_topic: ${ODOM_TOPIC}"
  echo "- imu_topic: ${IMU_TOPIC}"
  echo "- rmw: ${RMW_IMPLEMENTATION:-}"
  echo
  echo "## Runtime"
  ps -eo pid,args | grep -E "robot_local_state|ranger_base_node|imu_gyro_bias_filter" | grep -v grep || true
  echo
  echo "## Topic Info"
  for topic in "${CMD_TOPIC}" /cmd_vel_safe /cmd_vel "${ODOM_TOPIC}" /local_state/odometry "${IMU_TOPIC}" /motion_state; do
    echo "### ${topic}"
    ros2 topic info "${topic}" -v 2>&1 || true
  done
} >"${OUT_DIR}/environment.md"

echo "repeat,angle_deg,rc,report_dir,metrics_json" >"${OUT_DIR}/runs.csv"

echo "[spin-yaw-scale-calib] output: ${OUT_DIR}"
echo "[spin-yaw-scale-calib] motion starts in 5s. Ensure rotation clearance and E-stop are available."
for n in 5 4 3 2 1; do
  echo "[spin-yaw-scale-calib] ${n}..."
  sleep 1
done

IFS=',' read -r -a angle_array <<<"${ANGLES_DEG}"
for ((round = 1; round <= REPEAT; round += 1)); do
  for raw_angle in "${angle_array[@]}"; do
    angle="$(printf '%s' "${raw_angle}" | xargs)"
    [[ -n "${angle}" ]] || continue
    segment_label="${safe_label}_r${round}_a${angle//+/p}"
    segment_label="${segment_label//-/m}"
    segment_label="${segment_label//./p}"
    echo "[spin-yaw-scale-calib] round=${round}/${REPEAT} angle=${angle}deg"
    set +e
    bash "${SPIN_IMU_SCRIPT}" \
      --angular-speed "${ANGULAR_SPEED_RADPS}" \
      --angle-deg "${angle}" \
      --countdown-sec "${COUNTDOWN_SEC}" \
      --bias-sec "${BIAS_SEC}" \
      --settle-sec "${SETTLE_SEC}" \
      --sample-hz "${SAMPLE_HZ}" \
      --label "${segment_label}" \
      --cmd-topic "${CMD_TOPIC}" \
      --odom-topic "${ODOM_TOPIC}" \
      --imu-topic "${IMU_TOPIC}" \
      --output-root "${SEGMENT_ROOT}" | tee -a "${OUT_DIR}/segments.log"
    rc=${PIPESTATUS[0]}
    set -e
    report_dir="$(sed -n 's/^\[spin-imu-yaw\] complete: //p' "${OUT_DIR}/segments.log" | tail -1 || true)"
    metrics_json=""
    if [[ -n "${report_dir}" && -f "${report_dir}/metrics.json" ]]; then
      metrics_json="${report_dir}/metrics.json"
    fi
    echo "${round},${angle},${rc},${report_dir},${metrics_json}" >>"${OUT_DIR}/runs.csv"
    if [[ "${rc}" -ne 0 ]]; then
      echo "[spin-yaw-scale-calib] FAIL segment round=${round} angle=${angle} rc=${rc}" >&2
      break 2
    fi
  done
done

python3 - "${OUT_DIR}" <<'PY'
import csv
import json
import math
import statistics
import sys
from pathlib import Path

out_dir = Path(sys.argv[1])
runs_csv = out_dir / "runs.csv"
rows = []
with runs_csv.open(newline="", encoding="utf-8") as handle:
    for row in csv.DictReader(handle):
        if row.get("rc") != "0" or not row.get("metrics_json"):
            continue
        metrics_path = Path(row["metrics_json"])
        if not metrics_path.exists():
            continue
        data = json.loads(metrics_path.read_text(encoding="utf-8"))
        wheel = float(data["wheel_yaw_deg_final"])
        imu = float(data["imu_yaw_deg_final"])
        target = float(data["target_deg"])
        if abs(wheel) < 1e-6 or abs(imu) < 1e-6:
            continue
        rows.append({
            "round": int(row["repeat"]),
            "target_deg": target,
            "sign": "positive" if target >= 0.0 else "negative",
            "wheel_yaw_deg": wheel,
            "imu_yaw_deg": imu,
            "wheel_minus_imu_deg": wheel - imu,
            "ratio_imu_over_wheel": imu / wheel,
            "wheel_target_error_deg": float(data.get("wheel_target_error_deg_final", float("nan"))),
            "imu_target_error_deg": float(data.get("imu_target_error_deg_final", float("nan"))),
            "post_stop_wheel_overrun_deg": float(data.get("wheel_post_stop_overrun_deg", float("nan"))),
            "post_stop_imu_overrun_deg": float(data.get("imu_post_stop_overrun_deg", float("nan"))),
            "report_dir": row["report_dir"],
        })

def percentile(values, pct):
    if not values:
        return None
    ordered = sorted(values)
    if len(ordered) == 1:
        return ordered[0]
    pos = (len(ordered) - 1) * pct / 100.0
    lo = math.floor(pos)
    hi = math.ceil(pos)
    if lo == hi:
        return ordered[lo]
    return ordered[lo] * (hi - pos) + ordered[hi] * (pos - lo)

def fit(group):
    wheel = [r["wheel_yaw_deg"] for r in group]
    imu = [r["imu_yaw_deg"] for r in group]
    ratios = [r["ratio_imu_over_wheel"] for r in group]
    scale_lsq = sum(w * i for w, i in zip(wheel, imu)) / sum(w * w for w in wheel)
    residual_before = [r["wheel_yaw_deg"] - r["imu_yaw_deg"] for r in group]
    residual_after = [scale_lsq * r["wheel_yaw_deg"] - r["imu_yaw_deg"] for r in group]
    return {
        "count": len(group),
        "scale_lsq": scale_lsq,
        "scale_ratio_median": statistics.median(ratios),
        "scale_ratio_mean": statistics.mean(ratios),
        "scale_ratio_stdev": statistics.pstdev(ratios) if len(ratios) > 1 else 0.0,
        "wheel_minus_imu_mean_deg": statistics.mean(residual_before),
        "wheel_minus_imu_p95_abs_deg": percentile([abs(v) for v in residual_before], 95),
        "post_fit_residual_mean_deg": statistics.mean(residual_after),
        "post_fit_residual_p95_abs_deg": percentile([abs(v) for v in residual_after], 95),
    }

groups = {
    "positive": [r for r in rows if r["sign"] == "positive"],
    "negative": [r for r in rows if r["sign"] == "negative"],
}
summary = {
    "rows": rows,
    "positive": fit(groups["positive"]) if groups["positive"] else None,
    "negative": fit(groups["negative"]) if groups["negative"] else None,
}
(out_dir / "fit_summary.json").write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8")

with (out_dir / "calibration_samples.csv").open("w", newline="", encoding="utf-8") as handle:
    fields = [
        "round",
        "target_deg",
        "sign",
        "wheel_yaw_deg",
        "imu_yaw_deg",
        "wheel_minus_imu_deg",
        "ratio_imu_over_wheel",
        "wheel_target_error_deg",
        "imu_target_error_deg",
        "post_stop_wheel_overrun_deg",
        "post_stop_imu_overrun_deg",
        "report_dir",
    ]
    writer = csv.DictWriter(handle, fieldnames=fields)
    writer.writeheader()
    for row in rows:
        writer.writerow(row)

lines = [
    "# Ranger Spin Yaw Scale Calibration Summary",
    "",
    f"- report_dir: `{out_dir}`",
    f"- usable_samples: `{len(rows)}`",
    "",
    "## Suggested Scale",
    "",
    "| sign | samples | lsq_scale | median_ratio | mean_ratio | ratio_stdev | mean_wheel_minus_imu_deg | p95_abs_before_deg | p95_abs_after_deg |",
    "|---|---:|---:|---:|---:|---:|---:|---:|---:|",
]
for sign in ("positive", "negative"):
    data = summary[sign]
    if not data:
        lines.append(f"| {sign} | 0 |  |  |  |  |  |  |  |")
        continue
    lines.append(
        f"| {sign} | {data['count']} | {data['scale_lsq']:.6f} | "
        f"{data['scale_ratio_median']:.6f} | {data['scale_ratio_mean']:.6f} | "
        f"{data['scale_ratio_stdev']:.6f} | {data['wheel_minus_imu_mean_deg']:.3f} | "
        f"{data['wheel_minus_imu_p95_abs_deg']:.3f} | {data['post_fit_residual_p95_abs_deg']:.3f} |"
    )
lines.extend([
    "",
    "## Candidate Runtime Values",
    "",
])
if summary["positive"]:
    lines.append(f"- `RANGER_SPINNING_YAW_SCALE_POSITIVE={summary['positive']['scale_lsq']:.6f}`")
if summary["negative"]:
    lines.append(f"- `RANGER_SPINNING_YAW_SCALE_NEGATIVE={summary['negative']['scale_lsq']:.6f}`")
lines.extend([
    "",
    "Files:",
    "",
    "- `runs.csv`",
    "- `calibration_samples.csv`",
    "- `fit_summary.json`",
    "- `segments.log`",
    "- `environment.md`",
])
(out_dir / "summary.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
print(f"[spin-yaw-scale-calib] summary: {out_dir}/summary.md")
PY

echo "[spin-yaw-scale-calib] complete: ${OUT_DIR}"
