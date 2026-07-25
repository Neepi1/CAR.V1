#!/usr/bin/env python3

import argparse
import csv
import json
import math
import os
import statistics
import sys
import time
from dataclasses import asdict, dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Iterable, Optional, Sequence, Tuple


Point2 = Tuple[float, float]


@dataclass(frozen=True)
class SurfaceDetectionConfig:
    charge_contact_x_m: float = 0.398
    charge_contact_y_m: float = 0.0
    min_forward_gap_m: float = 0.05
    max_forward_gap_m: float = 1.50
    lateral_gate_m: float = 0.45
    histogram_bin_m: float = 0.004
    histogram_smoothing_m: float = 0.012
    peak_half_window_m: float = 0.025
    peak_nms_m: float = 0.050
    min_peak_points: int = 30
    line_inlier_threshold_m: float = 0.012
    max_plane_rms_m: float = 0.015
    fit_iterations: int = 3
    min_head_points: int = 35
    min_head_span_m: float = 0.015
    max_head_span_m: float = 0.180
    min_face_points: int = 80
    min_face_span_m: float = 0.120
    max_face_span_m: float = 0.360
    min_protrusion_m: float = 0.060
    max_protrusion_m: float = 0.220
    max_center_delta_m: float = 0.080
    max_yaw_delta_rad: float = 0.250
    expected_head_gap_m: Optional[float] = None
    expected_head_gap_tolerance_m: float = 0.180


@dataclass(frozen=True)
class PlaneEstimate:
    forward_gap_m: float
    center_y_m: float
    yaw_error_rad: float
    lateral_span_m: float
    inlier_count: int
    rms_error_m: float
    peak_gap_m: float


@dataclass(frozen=True)
class DockSurfaceEstimate:
    valid: bool
    reason: str
    head: Optional[PlaneEstimate] = None
    face: Optional[PlaneEstimate] = None
    protrusion_m: float = math.nan
    pair_score: float = 0.0
    planes: Tuple[PlaneEstimate, ...] = field(default_factory=tuple)


def _percentile(sorted_values: Sequence[float], fraction: float) -> float:
    if not sorted_values:
        return math.nan
    index = fraction * (len(sorted_values) - 1)
    lower = int(math.floor(index))
    upper = int(math.ceil(index))
    weight = index - lower
    return sorted_values[lower] * (1.0 - weight) + sorted_values[upper] * weight


def _fit_x_from_y(points: Sequence[Point2]) -> Optional[Tuple[float, float]]:
    if len(points) < 2:
        return None
    mean_x = sum(point[0] for point in points) / len(points)
    mean_y = sum(point[1] for point in points) / len(points)
    covariance = sum((x - mean_x) * (y - mean_y) for x, y in points)
    variance_y = sum((y - mean_y) ** 2 for _, y in points)
    if variance_y <= 1e-12:
        return None
    slope = covariance / variance_y
    return slope, mean_x - slope * mean_y


def _residual(point: Point2, slope: float, intercept: float) -> float:
    x, y = point
    return (x - (slope * y + intercept)) / math.sqrt(1.0 + slope * slope)


def _fit_plane(
    points: Sequence[Point2],
    peak_gap_m: float,
    config: SurfaceDetectionConfig,
) -> Optional[PlaneEstimate]:
    inliers = list(points)
    for _ in range(config.fit_iterations):
        fit = _fit_x_from_y(inliers)
        if fit is None:
            return None
        slope, intercept = fit
        next_inliers = [
            point
            for point in points
            if abs(_residual(point, slope, intercept)) <= config.line_inlier_threshold_m
        ]
        if len(next_inliers) < config.min_peak_points:
            return None
        if len(next_inliers) == len(inliers):
            inliers = next_inliers
            break
        inliers = next_inliers

    fit = _fit_x_from_y(inliers)
    if fit is None:
        return None
    slope, intercept = fit
    rms = math.sqrt(sum(_residual(point, slope, intercept) ** 2 for point in inliers) / len(inliers))
    if rms > config.max_plane_rms_m:
        return None
    ys = sorted(point[1] for point in inliers)
    lower_y = _percentile(ys, 0.05)
    upper_y = _percentile(ys, 0.95)
    span = upper_y - lower_y
    return PlaneEstimate(
        forward_gap_m=slope * config.charge_contact_y_m + intercept - config.charge_contact_x_m,
        center_y_m=0.5 * (lower_y + upper_y),
        yaw_error_rad=-math.atan(slope),
        lateral_span_m=span,
        inlier_count=len(inliers),
        rms_error_m=rms,
        peak_gap_m=peak_gap_m,
    )


def _candidate_peaks(gaps: Sequence[float], config: SurfaceDetectionConfig) -> Sequence[float]:
    if not gaps:
        return []
    first_bin = int(math.floor(min(gaps) / config.histogram_bin_m))
    last_bin = int(math.ceil(max(gaps) / config.histogram_bin_m))
    counts = [0] * (last_bin - first_bin + 1)
    for gap in gaps:
        index = int(round(gap / config.histogram_bin_m)) - first_bin
        if 0 <= index < len(counts):
            counts[index] += 1

    smooth_radius = max(1, int(round(config.histogram_smoothing_m / config.histogram_bin_m)))
    smoothed = []
    for index in range(len(counts)):
        lower = max(0, index - smooth_radius)
        upper = min(len(counts), index + smooth_radius + 1)
        smoothed.append(sum(counts[lower:upper]))

    ranked = sorted(
        range(len(smoothed)),
        key=lambda index: (smoothed[index], counts[index]),
        reverse=True,
    )
    peaks = []
    for index in ranked:
        if smoothed[index] < config.min_peak_points:
            break
        gap = (first_bin + index) * config.histogram_bin_m
        if any(abs(gap - selected) < config.peak_nms_m for selected in peaks):
            continue
        peaks.append(gap)
    return peaks


def detect_dock_surfaces(
    points: Iterable[Point2],
    config: SurfaceDetectionConfig,
) -> DockSurfaceEstimate:
    filtered = []
    gaps = []
    for x, y in points:
        if not math.isfinite(x) or not math.isfinite(y):
            continue
        gap = x - config.charge_contact_x_m
        if gap < config.min_forward_gap_m or gap > config.max_forward_gap_m:
            continue
        if abs(y - config.charge_contact_y_m) > config.lateral_gate_m:
            continue
        filtered.append((x, y))
        gaps.append(gap)

    if len(filtered) < config.min_head_points + config.min_face_points:
        return DockSurfaceEstimate(False, "insufficient_points")

    planes = []
    for peak_gap in _candidate_peaks(gaps, config):
        candidate = [
            point
            for point in filtered
            if abs((point[0] - config.charge_contact_x_m) - peak_gap)
            <= config.peak_half_window_m
        ]
        if len(candidate) < config.min_peak_points:
            continue
        plane = _fit_plane(candidate, peak_gap, config)
        if plane is None:
            continue
        if any(abs(plane.forward_gap_m - existing.forward_gap_m) < config.peak_nms_m for existing in planes):
            continue
        planes.append(plane)
    planes.sort(key=lambda plane: plane.forward_gap_m)

    best_pair = None
    best_score = -math.inf
    for head_index, head in enumerate(planes):
        if head.inlier_count < config.min_head_points:
            continue
        if not config.min_head_span_m <= head.lateral_span_m <= config.max_head_span_m:
            continue
        if config.expected_head_gap_m is not None and (
            abs(head.forward_gap_m - config.expected_head_gap_m)
            > config.expected_head_gap_tolerance_m
        ):
            continue
        for face in planes[head_index + 1 :]:
            protrusion = face.forward_gap_m - head.forward_gap_m
            if not config.min_protrusion_m <= protrusion <= config.max_protrusion_m:
                continue
            if face.inlier_count < config.min_face_points:
                continue
            if not config.min_face_span_m <= face.lateral_span_m <= config.max_face_span_m:
                continue
            center_delta = abs(head.center_y_m - face.center_y_m)
            yaw_delta = abs(head.yaw_error_rad - face.yaw_error_rad)
            if center_delta > config.max_center_delta_m or yaw_delta > config.max_yaw_delta_rad:
                continue

            count_score = min(1.0, head.inlier_count / 120.0) + min(1.0, face.inlier_count / 400.0)
            rms_score = max(0.0, 1.0 - head.rms_error_m / config.max_plane_rms_m)
            rms_score += max(0.0, 1.0 - face.rms_error_m / config.max_plane_rms_m)
            alignment_score = 1.0 - center_delta / config.max_center_delta_m
            yaw_score = 1.0 - yaw_delta / config.max_yaw_delta_rad
            expected_score = 0.0
            if config.expected_head_gap_m is not None:
                expected_score = max(
                    0.0,
                    1.0
                    - abs(head.forward_gap_m - config.expected_head_gap_m)
                    / config.expected_head_gap_tolerance_m,
                )
            score = count_score + rms_score + alignment_score + yaw_score + expected_score
            if score > best_score:
                best_score = score
                best_pair = (head, face, protrusion)

    if best_pair is None:
        return DockSurfaceEstimate(
            False,
            "two_surface_pair_not_found",
            planes=tuple(planes),
        )
    head, face, protrusion = best_pair
    return DockSurfaceEstimate(
        True,
        "ok",
        head=head,
        face=face,
        protrusion_m=protrusion,
        pair_score=best_score,
        planes=tuple(planes),
    )


def _median(values: Sequence[float]) -> float:
    return statistics.median(values) if values else math.nan


def _mad(values: Sequence[float]) -> float:
    if not values:
        return math.nan
    center = statistics.median(values)
    return statistics.median(abs(value - center) for value in values)


def _safe_label(value: str) -> str:
    cleaned = "".join(character if character.isalnum() or character in "_-" else "_" for character in value)
    return cleaned.strip("_") or "dock_surface"


def _timestamp_tag() -> str:
    return datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")


def _plane_field(estimate: DockSurfaceEstimate, name: str, field_name: str) -> float:
    plane = getattr(estimate, name)
    return getattr(plane, field_name) if plane is not None else math.nan


def _write_report(
    output_dir: Path,
    args: argparse.Namespace,
    estimates: Sequence[Tuple[float, DockSurfaceEstimate]],
    last_points: Sequence[Point2],
) -> Tuple[Path, bool]:
    output_dir.mkdir(parents=True, exist_ok=False)
    samples_path = output_dir / "samples.csv"
    with samples_path.open("w", newline="", encoding="utf-8") as handle:
        fieldnames = [
            "elapsed_sec",
            "valid",
            "reason",
            "candidate_planes",
            "head_gap_m",
            "head_center_y_m",
            "head_yaw_rad",
            "head_span_m",
            "head_inliers",
            "head_rms_m",
            "face_gap_m",
            "face_center_y_m",
            "face_yaw_rad",
            "face_span_m",
            "face_inliers",
            "face_rms_m",
            "protrusion_m",
            "pair_score",
        ]
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for elapsed, estimate in estimates:
            writer.writerow(
                {
                    "elapsed_sec": f"{elapsed:.6f}",
                    "valid": int(estimate.valid),
                    "reason": estimate.reason,
                    "candidate_planes": len(estimate.planes),
                    "head_gap_m": _plane_field(estimate, "head", "forward_gap_m"),
                    "head_center_y_m": _plane_field(estimate, "head", "center_y_m"),
                    "head_yaw_rad": _plane_field(estimate, "head", "yaw_error_rad"),
                    "head_span_m": _plane_field(estimate, "head", "lateral_span_m"),
                    "head_inliers": _plane_field(estimate, "head", "inlier_count"),
                    "head_rms_m": _plane_field(estimate, "head", "rms_error_m"),
                    "face_gap_m": _plane_field(estimate, "face", "forward_gap_m"),
                    "face_center_y_m": _plane_field(estimate, "face", "center_y_m"),
                    "face_yaw_rad": _plane_field(estimate, "face", "yaw_error_rad"),
                    "face_span_m": _plane_field(estimate, "face", "lateral_span_m"),
                    "face_inliers": _plane_field(estimate, "face", "inlier_count"),
                    "face_rms_m": _plane_field(estimate, "face", "rms_error_m"),
                    "protrusion_m": estimate.protrusion_m,
                    "pair_score": estimate.pair_score,
                }
            )

    candidate_planes_path = output_dir / "candidate_planes.csv"
    with candidate_planes_path.open("w", newline="", encoding="utf-8") as handle:
        fieldnames = [
            "sample_index",
            "elapsed_sec",
            "plane_index",
            "forward_gap_m",
            "center_y_m",
            "yaw_error_rad",
            "lateral_span_m",
            "inlier_count",
            "rms_error_m",
            "peak_gap_m",
        ]
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for sample_index, (elapsed, estimate) in enumerate(estimates):
            for plane_index, plane in enumerate(estimate.planes):
                writer.writerow(
                    {
                        "sample_index": sample_index,
                        "elapsed_sec": f"{elapsed:.6f}",
                        "plane_index": plane_index,
                        **asdict(plane),
                    }
                )

    with (output_dir / "points_last_frame.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(["base_x_m", "base_y_m", "forward_gap_m"])
        for x, y in last_points:
            writer.writerow([x, y, x - args.charge_contact_x_m])

    valid = [estimate for _, estimate in estimates if estimate.valid]
    total_count = len(estimates)
    valid_ratio = len(valid) / total_count if total_count else 0.0
    head_gaps = [estimate.head.forward_gap_m for estimate in valid if estimate.head is not None]
    face_gaps = [estimate.face.forward_gap_m for estimate in valid if estimate.face is not None]
    protrusions = [estimate.protrusion_m for estimate in valid]
    center_deltas = [
        abs(estimate.head.center_y_m - estimate.face.center_y_m)
        for estimate in valid
        if estimate.head is not None and estimate.face is not None
    ]
    yaw_deltas = [
        abs(estimate.head.yaw_error_rad - estimate.face.yaw_error_rad)
        for estimate in valid
        if estimate.head is not None and estimate.face is not None
    ]
    nearest_candidates = [estimate.planes[0] for _, estimate in estimates if estimate.planes]
    second_candidates = [estimate.planes[1] for _, estimate in estimates if len(estimate.planes) > 1]

    def candidate_metrics(planes: Sequence[PlaneEstimate]) -> dict[str, float]:
        return {
            "samples": len(planes),
            "gap_median_m": _median([plane.forward_gap_m for plane in planes]),
            "gap_mad_m": _mad([plane.forward_gap_m for plane in planes]),
            "center_y_median_m": _median([plane.center_y_m for plane in planes]),
            "center_y_mad_m": _mad([plane.center_y_m for plane in planes]),
            "yaw_median_rad": _median([plane.yaw_error_rad for plane in planes]),
            "yaw_mad_rad": _mad([plane.yaw_error_rad for plane in planes]),
            "span_median_m": _median([plane.lateral_span_m for plane in planes]),
            "inliers_median": _median([float(plane.inlier_count) for plane in planes]),
            "rms_median_m": _median([plane.rms_error_m for plane in planes]),
        }

    metrics = {
        "schema": "njrh.orbbec_dock_surface_calibration.v1",
        "expected_head_gap_m": args.expected_head_gap_m,
        "total_samples": total_count,
        "valid_samples": len(valid),
        "valid_ratio": valid_ratio,
        "head_gap_median_m": _median(head_gaps),
        "head_gap_mad_m": _mad(head_gaps),
        "face_gap_median_m": _median(face_gaps),
        "face_gap_mad_m": _mad(face_gaps),
        "protrusion_median_m": _median(protrusions),
        "protrusion_mad_m": _mad(protrusions),
        "center_delta_median_m": _median(center_deltas),
        "yaw_delta_median_rad": _median(yaw_deltas),
        "nearest_candidate": candidate_metrics(nearest_candidates),
        "second_candidate": candidate_metrics(second_candidates),
    }
    accepted = (
        len(valid) >= args.min_valid_samples
        and valid_ratio >= args.min_valid_ratio
        and abs(metrics["head_gap_median_m"] - args.expected_head_gap_m)
        <= args.expected_head_gap_tolerance_m
        and args.min_protrusion_m
        <= metrics["protrusion_median_m"]
        <= args.max_protrusion_m
        and metrics["protrusion_mad_m"] <= args.max_protrusion_mad_m
        and metrics["center_delta_median_m"] <= args.max_pair_center_delta_m
        and metrics["yaw_delta_median_rad"] <= args.max_pair_yaw_delta_rad
    )
    metrics["accepted"] = accepted
    (output_dir / "metrics.json").write_text(
        json.dumps(metrics, indent=2, sort_keys=True, allow_nan=True) + "\n",
        encoding="utf-8",
    )

    summary_path = output_dir / "summary.md"
    summary_path.write_text(
        "\n".join(
            [
                "# Orbbec dock two-surface calibration",
                "",
                f"- accepted: `{str(accepted).lower()}`",
                f"- expected telescoping-head gap: `{args.expected_head_gap_m:.3f} m`",
                f"- valid samples: `{len(valid)}/{total_count}` (`{valid_ratio:.1%}`)",
                f"- head gap median / MAD: `{metrics['head_gap_median_m']:.4f} / {metrics['head_gap_mad_m']:.4f} m`",
                f"- fixed-face gap median / MAD: `{metrics['face_gap_median_m']:.4f} / {metrics['face_gap_mad_m']:.4f} m`",
                f"- protrusion median / MAD: `{metrics['protrusion_median_m']:.4f} / {metrics['protrusion_mad_m']:.4f} m`",
                f"- pair center delta median: `{metrics['center_delta_median_m']:.4f} m`",
                f"- pair yaw delta median: `{math.degrees(metrics['yaw_delta_median_rad']):.3f} deg`",
                f"- nearest candidate samples: `{metrics['nearest_candidate']['samples']}/{total_count}`",
                f"- nearest candidate gap median / MAD: `{metrics['nearest_candidate']['gap_median_m']:.4f} / {metrics['nearest_candidate']['gap_mad_m']:.4f} m`",
                f"- nearest candidate center Y median / MAD: `{metrics['nearest_candidate']['center_y_median_m']:.4f} / {metrics['nearest_candidate']['center_y_mad_m']:.4f} m`",
                f"- nearest candidate yaw median / MAD: `{math.degrees(metrics['nearest_candidate']['yaw_median_rad']):.3f} / {math.degrees(metrics['nearest_candidate']['yaw_mad_rad']):.3f} deg`",
                f"- nearest candidate span / RMS median: `{metrics['nearest_candidate']['span_median_m']:.4f} / {metrics['nearest_candidate']['rms_median_m']:.4f} m`",
                f"- second candidate samples: `{metrics['second_candidate']['samples']}/{total_count}`",
                f"- second candidate gap / span median: `{metrics['second_candidate']['gap_median_m']:.4f} / {metrics['second_candidate']['span_median_m']:.4f} m`",
                "",
                "Files: `samples.csv`, `candidate_planes.csv`, `points_last_frame.csv`, `metrics.json`.",
                "",
            ]
        ),
        encoding="utf-8",
    )
    return summary_path, accepted


def _quaternion_matrix(x: float, y: float, z: float, w: float):
    import numpy as np

    norm = x * x + y * y + z * z + w * w
    if norm <= 1e-12:
        return np.eye(3)
    scale = 2.0 / norm
    return np.array(
        [
            [1.0 - scale * (y * y + z * z), scale * (x * y - z * w), scale * (x * z + y * w)],
            [scale * (x * y + z * w), 1.0 - scale * (x * x + z * z), scale * (y * z - x * w)],
            [scale * (x * z - y * w), scale * (y * z + x * w), 1.0 - scale * (x * x + y * y)],
        ],
        dtype=float,
    )


def run_ros_capture(args: argparse.Namespace) -> Tuple[Sequence[Tuple[float, DockSurfaceEstimate]], Sequence[Point2]]:
    import numpy as np
    import rclpy
    from rclpy.duration import Duration
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.time import Time
    from sensor_msgs.msg import CameraInfo, Image
    from tf2_ros import Buffer, TransformException, TransformListener

    class CaptureNode(Node):
        def __init__(self):
            super().__init__("orbbec_dock_surface_calibration")
            self.camera_info = None
            self.estimates = []
            self.last_points = []
            self.started_at = time.monotonic()
            self.last_processed_at = 0.0
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
            self.create_subscription(CameraInfo, args.camera_info_topic, self._info, qos_profile_sensor_data)
            self.create_subscription(Image, args.depth_topic, self._depth, qos_profile_sensor_data)

        def _info(self, message):
            self.camera_info = message

        def _depth(self, image):
            now = time.monotonic()
            if now - self.last_processed_at < 1.0 / args.processing_rate_hz:
                return
            self.last_processed_at = now
            info = self.camera_info
            if info is None or info.k[0] <= 0.0 or info.k[4] <= 0.0:
                return
            if image.width != info.width or image.height != info.height:
                return
            try:
                sensor_tf = self.tf_buffer.lookup_transform(
                    args.base_frame,
                    image.header.frame_id,
                    Time(),
                    timeout=Duration(seconds=args.tf_timeout_sec),
                )
                contact_tf = self.tf_buffer.lookup_transform(
                    args.base_frame,
                    args.charge_contact_frame,
                    Time(),
                    timeout=Duration(seconds=args.tf_timeout_sec),
                )
            except TransformException:
                return

            if image.encoding in ("16UC1", "mono16"):
                dtype = ">u2" if image.is_bigendian else "<u2"
                depth_raw = np.ndarray(
                    shape=(image.height, image.width),
                    dtype=dtype,
                    buffer=bytes(image.data),
                    strides=(image.step, 2),
                )
                depth = depth_raw[:: args.pixel_stride, :: args.pixel_stride].astype(float)
                depth *= args.depth_unit_m
            elif image.encoding == "32FC1":
                dtype = ">f4" if image.is_bigendian else "<f4"
                depth_raw = np.ndarray(
                    shape=(image.height, image.width),
                    dtype=dtype,
                    buffer=bytes(image.data),
                    strides=(image.step, 4),
                )
                depth = depth_raw[:: args.pixel_stride, :: args.pixel_stride].astype(float)
            else:
                return

            rows = np.arange(0, image.height, args.pixel_stride, dtype=float)
            cols = np.arange(0, image.width, args.pixel_stride, dtype=float)
            uu, vv = np.meshgrid(cols, rows)
            valid = np.isfinite(depth) & (depth >= args.min_depth_m) & (depth <= args.max_depth_m)
            if not np.any(valid):
                return
            optical = np.column_stack(
                (
                    ((uu[valid] - info.k[2]) * depth[valid] / info.k[0]),
                    ((vv[valid] - info.k[5]) * depth[valid] / info.k[4]),
                    depth[valid],
                )
            )
            transform = sensor_tf.transform
            rotation = _quaternion_matrix(
                transform.rotation.x,
                transform.rotation.y,
                transform.rotation.z,
                transform.rotation.w,
            )
            translation = np.array(
                [transform.translation.x, transform.translation.y, transform.translation.z]
            )
            base_points = optical @ rotation.T + translation
            base_valid = (
                (base_points[:, 2] >= args.min_base_z_m)
                & (base_points[:, 2] <= args.max_base_z_m)
            )
            planar = base_points[base_valid, :2]
            points = [(float(x), float(y)) for x, y in planar]
            config = SurfaceDetectionConfig(
                charge_contact_x_m=contact_tf.transform.translation.x,
                charge_contact_y_m=contact_tf.transform.translation.y,
                min_forward_gap_m=args.min_forward_gap_m,
                max_forward_gap_m=args.max_forward_gap_m,
                lateral_gate_m=args.lateral_gate_m,
                min_protrusion_m=args.min_protrusion_m,
                max_protrusion_m=args.max_protrusion_m,
                max_center_delta_m=args.max_pair_center_delta_m,
                max_yaw_delta_rad=args.max_pair_yaw_delta_rad,
                expected_head_gap_m=args.expected_head_gap_m,
                expected_head_gap_tolerance_m=args.expected_head_gap_tolerance_m,
            )
            estimate = detect_dock_surfaces(points, config)
            self.estimates.append((now - self.started_at, estimate))
            self.last_points = points

    rclpy.init()
    node = CaptureNode()
    deadline = time.monotonic() + args.collect_sec + args.startup_timeout_sec
    first_sample_at = None
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.estimates and first_sample_at is None:
                first_sample_at = time.monotonic()
                deadline = first_sample_at + args.collect_sec
    finally:
        estimates = list(node.estimates)
        last_points = list(node.last_points)
        node.destroy_node()
        rclpy.shutdown()
    return estimates, last_points


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Read-only Orbbec telescoping-head/fixed-face calibration")
    parser.add_argument("--expected-head-gap-m", type=float, required=True)
    parser.add_argument("--label", default="dock_surface")
    parser.add_argument("--collect-sec", type=float, default=6.0)
    parser.add_argument("--startup-timeout-sec", type=float, default=10.0)
    parser.add_argument("--processing-rate-hz", type=float, default=8.0)
    parser.add_argument("--output-root", default="/tmp/njrh_reports/orbbec_dock_surface_calibration")
    parser.add_argument("--depth-topic", default="/camera336l/depth/image_raw")
    parser.add_argument("--camera-info-topic", default="/camera336l/depth/camera_info")
    parser.add_argument("--base-frame", default="base_link")
    parser.add_argument("--charge-contact-frame", default="charge_contact_link")
    parser.add_argument("--depth-unit-m", type=float, default=0.001)
    parser.add_argument("--min-depth-m", type=float, default=0.15)
    parser.add_argument("--max-depth-m", type=float, default=2.0)
    parser.add_argument("--min-base-z-m", type=float, default=0.16)
    parser.add_argument("--max-base-z-m", type=float, default=0.36)
    parser.add_argument("--pixel-stride", type=int, default=3)
    parser.add_argument("--tf-timeout-sec", type=float, default=0.10)
    parser.add_argument("--charge-contact-x-m", type=float, default=0.398)
    parser.add_argument("--min-forward-gap-m", type=float, default=0.02)
    parser.add_argument("--max-forward-gap-m", type=float, default=1.50)
    parser.add_argument("--lateral-gate-m", type=float, default=0.45)
    parser.add_argument("--min-protrusion-m", type=float, default=0.06)
    parser.add_argument("--max-protrusion-m", type=float, default=0.22)
    parser.add_argument("--expected-head-gap-tolerance-m", type=float, default=0.15)
    parser.add_argument("--max-protrusion-mad-m", type=float, default=0.010)
    parser.add_argument("--max-pair-center-delta-m", type=float, default=0.080)
    parser.add_argument("--max-pair-yaw-delta-rad", type=float, default=0.250)
    parser.add_argument("--min-valid-samples", type=int, default=15)
    parser.add_argument("--min-valid-ratio", type=float, default=0.70)
    args = parser.parse_args(argv)
    if args.collect_sec <= 0.0 or args.processing_rate_hz <= 0.0 or args.pixel_stride < 1:
        parser.error("collection duration, processing rate, and pixel stride must be positive")
    return args


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = parse_args(argv)
    output_dir = Path(args.output_root) / f"{_timestamp_tag()}_{_safe_label(args.label)}"
    try:
        estimates, last_points = run_ros_capture(args)
        summary_path, accepted = _write_report(output_dir, args, estimates, last_points)
    except Exception as error:
        print(f"[orbbec-dock-surface] FAIL: {error}", file=sys.stderr)
        return 1
    print(f"[orbbec-dock-surface] summary: {summary_path}")
    print(f"[orbbec-dock-surface] accepted: {str(accepted).lower()}")
    return 0 if accepted else 2


if __name__ == "__main__":
    raise SystemExit(main())
