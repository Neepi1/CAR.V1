#!/usr/bin/env bash
set -euo pipefail
umask 0002

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/common_env.sh"

DURATION_SEC=60
LABEL="imu_yaw_projection"
SAMPLE_HZ=50.0
BASE_FRAME="base_link"
RAW_IMU_TOPIC="/lidar_imu"
CORRECTED_IMU_TOPIC="/lidar_imu_bias_corrected"
WHEEL_ODOM_TOPIC="/wheel/odom_ekf"
LOCAL_ODOM_TOPIC="/local_state/odometry"
MOVING_WZ_THRESHOLD=0.02
OUTPUT_ROOT="${NJRH_PROJECT_ROOT}/reports/local_state_imu_yaw_rate_projection"

usage() {
  cat <<'EOF'
Usage: record_imu_yaw_rate_projection_window.sh [options]

Records IMU angular velocity in its native frame and after TF projection into
base_link, then compares both against wheel/local yaw-rate.

This script is read-only. It does not publish velocity, navigation goals, or
service requests.

Options:
  --duration-sec N          Capture duration. Default: 60
  --label LABEL             Report label. Default: imu_yaw_projection
  --sample-hz HZ            CSV sampling rate. Default: 50.0
  --base-frame FRAME        Target frame for projection. Default: base_link
  --raw-imu-topic TOPIC     Raw IMU topic. Default: /lidar_imu
  --corrected-imu-topic TOPIC Bias-corrected IMU topic. Default: /lidar_imu_bias_corrected
  --wheel-odom-topic TOPIC  Wheel odom reference. Default: /wheel/odom_ekf
  --local-odom-topic TOPIC  Local EKF odom topic. Default: /local_state/odometry
  --moving-wz-threshold R   Moving sample threshold in rad/s. Default: 0.02
  --output-root DIR         Report root. Default: reports/local_state_imu_yaw_rate_projection
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
    --sample-hz)
      SAMPLE_HZ="${2:-}"
      shift 2
      ;;
    --base-frame)
      BASE_FRAME="${2:-}"
      shift 2
      ;;
    --raw-imu-topic)
      RAW_IMU_TOPIC="${2:-}"
      shift 2
      ;;
    --corrected-imu-topic)
      CORRECTED_IMU_TOPIC="${2:-}"
      shift 2
      ;;
    --wheel-odom-topic)
      WHEEL_ODOM_TOPIC="${2:-}"
      shift 2
      ;;
    --local-odom-topic)
      LOCAL_ODOM_TOPIC="${2:-}"
      shift 2
      ;;
    --moving-wz-threshold)
      MOVING_WZ_THRESHOLD="${2:-}"
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
      echo "[imu-yaw-projection] unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

safe_label="$(printf '%s' "${LABEL}" | tr -c 'A-Za-z0-9_.-' '_')"
timestamp="$(date -u +%Y%m%dT%H%M%SZ)"
report_dir="${OUTPUT_ROOT}/${timestamp}_${safe_label}_${DURATION_SEC}s"
mkdir -p "${report_dir}"
echo "[imu-yaw-projection] report_dir=${report_dir}" >&2

python3 - \
  "${DURATION_SEC}" \
  "${report_dir}" \
  "${SAMPLE_HZ}" \
  "${BASE_FRAME}" \
  "${RAW_IMU_TOPIC}" \
  "${CORRECTED_IMU_TOPIC}" \
  "${WHEEL_ODOM_TOPIC}" \
  "${LOCAL_ODOM_TOPIC}" \
  "${MOVING_WZ_THRESHOLD}" <<'PY'
import csv
import json
import math
import statistics
import sys
import time
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Tuple

import rclpy
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import Imu
from tf2_ros import Buffer, TransformException, TransformListener

duration_sec = float(sys.argv[1])
report_dir = Path(sys.argv[2])
sample_hz = float(sys.argv[3])
base_frame = sys.argv[4]
raw_imu_topic = sys.argv[5]
corrected_imu_topic = sys.argv[6]
wheel_odom_topic = sys.argv[7]
local_odom_topic = sys.argv[8]
moving_wz_threshold = abs(float(sys.argv[9]))

if duration_sec <= 0.0:
    raise SystemExit("duration-sec must be positive")
if sample_hz <= 0.0:
    raise SystemExit("sample-hz must be positive")


def stamp_sec(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def finite(value: Optional[float]) -> bool:
    return value is not None and math.isfinite(float(value))


def stats(values: Iterable[Optional[float]]) -> Dict[str, Optional[float]]:
    vals = [float(v) for v in values if finite(v)]
    if not vals:
        return {
            "count": 0,
            "mean": None,
            "median": None,
            "stdev": None,
            "min": None,
            "max": None,
        }
    return {
        "count": len(vals),
        "mean": statistics.mean(vals),
        "median": statistics.median(vals),
        "stdev": statistics.pstdev(vals) if len(vals) > 1 else 0.0,
        "min": min(vals),
        "max": max(vals),
    }


def sign(value: Optional[float], eps: float) -> int:
    if not finite(value):
        return 0
    if value > eps:
        return 1
    if value < -eps:
        return -1
    return 0


def normalize_quat(q: Tuple[float, float, float, float]) -> Tuple[float, float, float, float]:
    x, y, z, w = q
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm <= 0.0:
        return (0.0, 0.0, 0.0, 1.0)
    return (x / norm, y / norm, z / norm, w / norm)


def cross(a: Tuple[float, float, float], b: Tuple[float, float, float]) -> Tuple[float, float, float]:
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def rotate_vector(
    q: Tuple[float, float, float, float],
    v: Tuple[float, float, float],
) -> Tuple[float, float, float]:
    x, y, z, w = normalize_quat(q)
    qv = (x, y, z)
    uv = cross(qv, v)
    uuv = cross(qv, uv)
    return (
        v[0] + 2.0 * (w * uv[0] + uuv[0]),
        v[1] + 2.0 * (w * uv[1] + uuv[1]),
        v[2] + 2.0 * (w * uv[2] + uuv[2]),
    )


def quat_to_rpy_deg(q: Tuple[float, float, float, float]) -> Dict[str, float]:
    x, y, z, w = normalize_quat(q)
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return {
        "roll_deg": math.degrees(roll),
        "pitch_deg": math.degrees(pitch),
        "yaw_deg": math.degrees(yaw),
    }


def compare_series(
    rows: List[Dict[str, Optional[float]]],
    value_key: str,
    ref_key: str,
    eps: float,
) -> Dict[str, object]:
    compared = [
        r for r in rows
        if finite(r.get(value_key)) and finite(r.get(ref_key)) and abs(float(r[ref_key])) > eps
    ]
    diffs = [float(r[value_key]) - float(r[ref_key]) for r in compared]
    abs_diffs = [abs(v) for v in diffs]
    ratios = [float(r[value_key]) / float(r[ref_key]) for r in compared]
    sign_rows = [
        r for r in compared
        if sign(r.get(value_key), eps) != 0 or sign(r.get(ref_key), eps) != 0
    ]
    sign_match_rate = None
    if sign_rows:
        sign_match_rate = sum(
            1 for r in sign_rows
            if sign(r.get(value_key), eps) == sign(r.get(ref_key), eps)
        ) / len(sign_rows)
    return {
        "value_key": value_key,
        "reference_key": ref_key,
        "compared_count": len(compared),
        "sign_compared_count": len(sign_rows),
        "sign_match_rate": sign_match_rate,
        "value_stats": stats([r.get(value_key) for r in compared]),
        "reference_stats": stats([r.get(ref_key) for r in compared]),
        "diff_value_minus_ref_stats": stats(diffs),
        "abs_diff_stats": stats(abs_diffs),
        "ratio_value_over_ref_stats": stats(ratios),
    }


class ImuProjectionProbe(Node):
    def __init__(self) -> None:
        super().__init__("record_imu_yaw_rate_projection_window")
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=200,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.start_wall = time.time()
        self.last_raw: Optional[Imu] = None
        self.last_corrected: Optional[Imu] = None
        self.last_wheel: Optional[Odometry] = None
        self.last_local: Optional[Odometry] = None
        self.rows: List[Dict[str, Optional[float]]] = []
        self.tf_success_count = 0
        self.tf_failure_count = 0
        self.tf_failures: Dict[str, int] = {}
        self.transforms: Dict[str, Dict[str, object]] = {}
        self.frame_ids = {
            "raw": {},
            "corrected": {},
        }

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(Imu, raw_imu_topic, self._raw_cb, qos)
        self.create_subscription(Imu, corrected_imu_topic, self._corrected_cb, qos)
        self.create_subscription(Odometry, wheel_odom_topic, self._wheel_cb, qos)
        self.create_subscription(Odometry, local_odom_topic, self._local_cb, qos)

        self.csv_file = (report_dir / "samples.csv").open("w", newline="", encoding="utf-8")
        self.writer = csv.DictWriter(
            self.csv_file,
            fieldnames=[
                "rel_time_sec",
                "wheel_wz_radps",
                "local_wz_radps",
                "raw_stamp_sec",
                "raw_frame_id",
                "raw_wx_radps",
                "raw_wy_radps",
                "raw_wz_radps",
                "raw_base_wx_radps",
                "raw_base_wy_radps",
                "raw_base_wz_radps",
                "raw_tf_ok",
                "corrected_stamp_sec",
                "corrected_frame_id",
                "corrected_wx_radps",
                "corrected_wy_radps",
                "corrected_wz_radps",
                "corrected_base_wx_radps",
                "corrected_base_wy_radps",
                "corrected_base_wz_radps",
                "corrected_tf_ok",
                "raw_z_minus_wheel",
                "raw_base_z_minus_wheel",
                "corrected_z_minus_wheel",
                "corrected_base_z_minus_wheel",
                "raw_base_z_minus_raw_z",
                "corrected_base_z_minus_corrected_z",
            ],
        )
        self.writer.writeheader()
        self.create_timer(1.0 / sample_hz, self._timer_cb)

    def _raw_cb(self, msg: Imu) -> None:
        self.last_raw = msg
        self._note_frame("raw", msg.header.frame_id)

    def _corrected_cb(self, msg: Imu) -> None:
        self.last_corrected = msg
        self._note_frame("corrected", msg.header.frame_id)

    def _wheel_cb(self, msg: Odometry) -> None:
        self.last_wheel = msg

    def _local_cb(self, msg: Odometry) -> None:
        self.last_local = msg

    def _note_frame(self, source: str, frame_id: str) -> None:
        key = frame_id or "<empty>"
        frames = self.frame_ids[source]
        frames[key] = int(frames.get(key, 0)) + 1

    def _lookup_rotation(self, source_frame: str) -> Tuple[Optional[Tuple[float, float, float, float]], bool]:
        if not source_frame:
            self.tf_failure_count += 1
            self.tf_failures["empty_frame_id"] = self.tf_failures.get("empty_frame_id", 0) + 1
            return None, False
        try:
            transform = self.tf_buffer.lookup_transform(
                base_frame,
                source_frame,
                Time(),
                timeout=Duration(seconds=0.02),
            )
        except TransformException as exc:
            self.tf_failure_count += 1
            reason = f"{source_frame}->{base_frame}: {type(exc).__name__}"
            self.tf_failures[reason] = self.tf_failures.get(reason, 0) + 1
            return None, False

        q_msg = transform.transform.rotation
        t_msg = transform.transform.translation
        q = normalize_quat((q_msg.x, q_msg.y, q_msg.z, q_msg.w))
        self.tf_success_count += 1
        if source_frame not in self.transforms:
            self.transforms[source_frame] = {
                "target_frame": base_frame,
                "source_frame": source_frame,
                "translation": {
                    "x": t_msg.x,
                    "y": t_msg.y,
                    "z": t_msg.z,
                },
                "rotation_quat_xyzw": {
                    "x": q[0],
                    "y": q[1],
                    "z": q[2],
                    "w": q[3],
                },
                "rotation_rpy_deg": quat_to_rpy_deg(q),
            }
        return q, True

    def _project_imu(self, msg: Optional[Imu], prefix: str) -> Dict[str, Optional[float]]:
        values: Dict[str, Optional[float]] = {
            f"{prefix}_stamp_sec": None,
            f"{prefix}_frame_id": None,
            f"{prefix}_wx_radps": None,
            f"{prefix}_wy_radps": None,
            f"{prefix}_wz_radps": None,
            f"{prefix}_base_wx_radps": None,
            f"{prefix}_base_wy_radps": None,
            f"{prefix}_base_wz_radps": None,
            f"{prefix}_tf_ok": False,
        }
        if msg is None:
            return values

        frame_id = msg.header.frame_id
        vector = (
            float(msg.angular_velocity.x),
            float(msg.angular_velocity.y),
            float(msg.angular_velocity.z),
        )
        values[f"{prefix}_stamp_sec"] = stamp_sec(msg.header.stamp)
        values[f"{prefix}_frame_id"] = frame_id
        values[f"{prefix}_wx_radps"] = vector[0]
        values[f"{prefix}_wy_radps"] = vector[1]
        values[f"{prefix}_wz_radps"] = vector[2]

        rotation, ok = self._lookup_rotation(frame_id)
        values[f"{prefix}_tf_ok"] = ok
        if rotation is not None:
            projected = rotate_vector(rotation, vector)
            values[f"{prefix}_base_wx_radps"] = projected[0]
            values[f"{prefix}_base_wy_radps"] = projected[1]
            values[f"{prefix}_base_wz_radps"] = projected[2]
        return values

    def _timer_cb(self) -> None:
        row: Dict[str, Optional[float]] = {
            "rel_time_sec": time.time() - self.start_wall,
            "wheel_wz_radps": None,
            "local_wz_radps": None,
        }
        if self.last_wheel is not None:
            row["wheel_wz_radps"] = float(self.last_wheel.twist.twist.angular.z)
        if self.last_local is not None:
            row["local_wz_radps"] = float(self.last_local.twist.twist.angular.z)

        row.update(self._project_imu(self.last_raw, "raw"))
        row.update(self._project_imu(self.last_corrected, "corrected"))

        wheel = row["wheel_wz_radps"]
        for key in (
            "raw_wz_radps",
            "raw_base_wz_radps",
            "corrected_wz_radps",
            "corrected_base_wz_radps",
        ):
            out_key = {
                "raw_wz_radps": "raw_z_minus_wheel",
                "raw_base_wz_radps": "raw_base_z_minus_wheel",
                "corrected_wz_radps": "corrected_z_minus_wheel",
                "corrected_base_wz_radps": "corrected_base_z_minus_wheel",
            }[key]
            row[out_key] = float(row[key]) - float(wheel) if finite(row[key]) and finite(wheel) else None

        row["raw_base_z_minus_raw_z"] = (
            float(row["raw_base_wz_radps"]) - float(row["raw_wz_radps"])
            if finite(row["raw_base_wz_radps"]) and finite(row["raw_wz_radps"])
            else None
        )
        row["corrected_base_z_minus_corrected_z"] = (
            float(row["corrected_base_wz_radps"]) - float(row["corrected_wz_radps"])
            if finite(row["corrected_base_wz_radps"]) and finite(row["corrected_wz_radps"])
            else None
        )

        self.rows.append(row)
        self.writer.writerow({
            key: self._format_csv(row.get(key))
            for key in self.writer.fieldnames
        })
        self.csv_file.flush()

    @staticmethod
    def _format_csv(value) -> str:
        if value is None:
            return ""
        if isinstance(value, bool):
            return str(value).lower()
        if isinstance(value, float):
            return f"{value:.9f}" if math.isfinite(value) else ""
        return str(value)

    def close(self) -> None:
        self.csv_file.close()


rclpy.init()
node = ImuProjectionProbe()
deadline = time.time() + duration_sec
try:
    while rclpy.ok() and time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
finally:
    moving_rows = [
        r for r in node.rows
        if any(
            abs(float(r.get(k))) > moving_wz_threshold
            for k in (
                "wheel_wz_radps",
                "local_wz_radps",
                "raw_wz_radps",
                "raw_base_wz_radps",
                "corrected_wz_radps",
                "corrected_base_wz_radps",
            )
            if finite(r.get(k))
        )
    ]
    comparisons = {
        "raw_z_vs_wheel": compare_series(moving_rows, "raw_wz_radps", "wheel_wz_radps", moving_wz_threshold),
        "raw_base_z_vs_wheel": compare_series(moving_rows, "raw_base_wz_radps", "wheel_wz_radps", moving_wz_threshold),
        "corrected_z_vs_wheel": compare_series(moving_rows, "corrected_wz_radps", "wheel_wz_radps", moving_wz_threshold),
        "corrected_base_z_vs_wheel": compare_series(moving_rows, "corrected_base_wz_radps", "wheel_wz_radps", moving_wz_threshold),
        "local_wz_vs_wheel": compare_series(moving_rows, "local_wz_radps", "wheel_wz_radps", moving_wz_threshold),
    }
    summary = {
        "duration_sec": duration_sec,
        "sample_hz": sample_hz,
        "sample_count": len(node.rows),
        "moving_sample_count": len(moving_rows),
        "base_frame": base_frame,
        "raw_imu_topic": raw_imu_topic,
        "corrected_imu_topic": corrected_imu_topic,
        "wheel_odom_topic": wheel_odom_topic,
        "local_odom_topic": local_odom_topic,
        "moving_wz_threshold_radps": moving_wz_threshold,
        "frame_ids": node.frame_ids,
        "transforms_to_base": node.transforms,
        "tf_success_count": node.tf_success_count,
        "tf_failure_count": node.tf_failure_count,
        "tf_failures": node.tf_failures,
        "series_stats_moving": {
            "wheel_wz_radps": stats([r.get("wheel_wz_radps") for r in moving_rows]),
            "local_wz_radps": stats([r.get("local_wz_radps") for r in moving_rows]),
            "raw_wz_radps": stats([r.get("raw_wz_radps") for r in moving_rows]),
            "raw_base_wz_radps": stats([r.get("raw_base_wz_radps") for r in moving_rows]),
            "corrected_wz_radps": stats([r.get("corrected_wz_radps") for r in moving_rows]),
            "corrected_base_wz_radps": stats([r.get("corrected_base_wz_radps") for r in moving_rows]),
            "raw_base_z_minus_raw_z": stats([r.get("raw_base_z_minus_raw_z") for r in moving_rows]),
            "corrected_base_z_minus_corrected_z": stats([r.get("corrected_base_z_minus_corrected_z") for r in moving_rows]),
        },
        "comparisons": comparisons,
    }
    (report_dir / "summary.json").write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n")

    lines = [
        "# IMU Yaw Rate Projection Window",
        "",
        f"- duration_sec: `{duration_sec:.1f}`",
        f"- sample_count: `{len(node.rows)}`",
        f"- moving_sample_count: `{len(moving_rows)}`",
        f"- base_frame: `{base_frame}`",
        f"- raw_imu_topic: `{raw_imu_topic}`",
        f"- corrected_imu_topic: `{corrected_imu_topic}`",
        f"- wheel_odom_topic: `{wheel_odom_topic}`",
        f"- local_odom_topic: `{local_odom_topic}`",
        f"- tf_success_count: `{node.tf_success_count}`",
        f"- tf_failure_count: `{node.tf_failure_count}`",
        f"- raw_frame_ids: `{node.frame_ids['raw']}`",
        f"- corrected_frame_ids: `{node.frame_ids['corrected']}`",
        "",
        "## Transforms To Base",
        "",
    ]
    if node.transforms:
        for frame_id, transform in sorted(node.transforms.items()):
            lines.append(f"- `{frame_id}` -> `{base_frame}`: `{transform}`")
    else:
        lines.append("- none")
    lines.extend([
        "",
        "## Moving Series Stats",
        "",
    ])
    for key, value in summary["series_stats_moving"].items():
        lines.append(f"- {key}: `{value}`")
    lines.extend([
        "",
        "## Comparisons Against Wheel Yaw Rate",
        "",
        "| comparison | count | sign_match_rate | ratio_mean | abs_diff_mean_radps | diff_mean_radps |",
        "|---|---:|---:|---:|---:|---:|",
    ])
    for name, comp in comparisons.items():
        ratio_mean = comp["ratio_value_over_ref_stats"]["mean"]
        abs_diff_mean = comp["abs_diff_stats"]["mean"]
        diff_mean = comp["diff_value_minus_ref_stats"]["mean"]
        sign_match_rate = comp["sign_match_rate"]
        sign_text = "" if sign_match_rate is None else f"{sign_match_rate:.3f}"
        ratio_text = "" if ratio_mean is None else f"{ratio_mean:.6f}"
        abs_diff_text = "" if abs_diff_mean is None else f"{abs_diff_mean:.6f}"
        diff_text = "" if diff_mean is None else f"{diff_mean:.6f}"
        lines.append(
            f"| {name} | {comp['compared_count']} | "
            f"{sign_text} | "
            f"{ratio_text} | "
            f"{abs_diff_text} | "
            f"{diff_text} |"
        )
    lines.extend([
        "",
        "Files:",
        "",
        "- `samples.csv`",
        "- `summary.json`",
    ])
    (report_dir / "summary.md").write_text("\n".join(lines) + "\n")
    node.close()
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    print(report_dir)
PY
