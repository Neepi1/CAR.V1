#!/usr/bin/env python3
"""Estimate base_link -> lidar_level_link yaw error from a front wall scan.

The robot must be physically square to a straight wall segment.  The script
does not use map, odom, AMCL, or Isaac output; it only transforms /scan points
to base_link using the current static TF and fits the dominant front wall.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import random
import re
import statistics
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, List, Optional, Sequence, Tuple

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener


Point = Tuple[float, float]


def norm_angle(value: float) -> float:
    return math.atan2(math.sin(value), math.cos(value))


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def parse_float_list(text: str) -> List[float]:
    return [float(part.strip()) for part in text.split(",") if part.strip()]


def parse_current_lidar_yaw(path: str) -> float:
    text = Path(path).read_text(encoding="utf-8")
    flat = re.search(r"(?m)^\s*lidar_yaw\s*:\s*([-+0-9.eE]+)\s*$", text)
    if flat:
        return float(flat.group(1))
    rpy = re.search(r"(?m)^\s*lidar_rpy\s*:\s*\[([^\]]+)\]\s*$", text)
    if rpy:
        values = parse_float_list(rpy.group(1))
        if len(values) >= 3:
            return values[2]
    raise ValueError(f"could not read lidar_yaw/lidar_rpy yaw from {path}")


@dataclass
class Transform2D:
    x: float
    y: float
    yaw: float
    source_frame: str

    def apply(self, point: Point) -> Point:
        c = math.cos(self.yaw)
        s = math.sin(self.yaw)
        px, py = point
        return (self.x + c * px - s * py, self.y + s * px + c * py)


class ScanCollector(Node):
    def __init__(
        self,
        scan_topic: str,
        base_frame: str,
        collect_sec: float,
        range_min_m: float,
        range_max_m: float,
        front_half_angle_rad: float,
        max_scans: int,
    ) -> None:
        super().__init__("front_wall_lidar_yaw_fit")
        self.base_frame = base_frame
        self.collect_sec = collect_sec
        self.range_min_m = range_min_m
        self.range_max_m = range_max_m
        self.front_half_angle_rad = front_half_angle_rad
        self.max_scans = max_scans
        self.started = time.monotonic()
        self.points: List[Point] = []
        self.scan_count = 0
        self.last_error = ""
        self.scan_frame = ""
        self.transform: Optional[Transform2D] = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(LaserScan, scan_topic, self._scan_cb, qos_profile_sensor_data)

    def done(self) -> bool:
        return (time.monotonic() - self.started) >= self.collect_sec or self.scan_count >= self.max_scans

    def _lookup_transform(self, source_frame: str) -> Optional[Transform2D]:
        try:
            tf = self.tf_buffer.lookup_transform(
                self.base_frame,
                source_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.2),
            )
        except TransformException as exc:
            self.last_error = str(exc)
            return None
        tr = tf.transform.translation
        q = tf.transform.rotation
        return Transform2D(
            x=float(tr.x),
            y=float(tr.y),
            yaw=yaw_from_quaternion(q.x, q.y, q.z, q.w),
            source_frame=source_frame,
        )

    def _scan_cb(self, msg: LaserScan) -> None:
        if self.done():
            return
        source_frame = msg.header.frame_id or "lidar_level_link"
        transform = self._lookup_transform(source_frame)
        if transform is None:
            return
        self.transform = transform
        self.scan_frame = source_frame
        angle = msg.angle_min
        added = 0
        for raw_range in msg.ranges:
            r = float(raw_range)
            if math.isfinite(r) and self.range_min_m <= r <= self.range_max_m:
                p_scan = (r * math.cos(angle), r * math.sin(angle))
                p_base = transform.apply(p_scan)
                bx, by = p_base
                if bx > 0.05 and abs(math.atan2(by, bx)) <= self.front_half_angle_rad:
                    self.points.append(p_base)
                    added += 1
            angle += msg.angle_increment
        if added > 0:
            self.scan_count += 1


def line_from_points(a: Point, b: Point) -> Optional[Tuple[float, float, float]]:
    ax, ay = a
    bx, by = b
    dx = bx - ax
    dy = by - ay
    length = math.hypot(dx, dy)
    if length < 0.20:
        return None
    # ax + by + c = 0, normalized.
    la = dy / length
    lb = -dx / length
    lc = -(la * ax + lb * ay)
    return (la, lb, lc)


def line_distance(line: Tuple[float, float, float], point: Point) -> float:
    a, b, c = line
    x, y = point
    return abs(a * x + b * y + c)


def percentile(values: Sequence[float], q: float) -> float:
    ordered = sorted(values)
    if not ordered:
        return 0.0
    pos = (len(ordered) - 1) * q
    lo = int(math.floor(pos))
    hi = int(math.ceil(pos))
    if lo == hi:
        return ordered[lo]
    frac = pos - lo
    return ordered[lo] * (1.0 - frac) + ordered[hi] * frac


def principal_line(points: Sequence[Point]) -> Tuple[float, float, float, float, float]:
    mx = statistics.fmean(p[0] for p in points)
    my = statistics.fmean(p[1] for p in points)
    xx = statistics.fmean((p[0] - mx) * (p[0] - mx) for p in points)
    yy = statistics.fmean((p[1] - my) * (p[1] - my) for p in points)
    xy = statistics.fmean((p[0] - mx) * (p[1] - my) for p in points)
    line_angle = 0.5 * math.atan2(2.0 * xy, xx - yy)
    dx = math.cos(line_angle)
    dy = math.sin(line_angle)
    # Normal choices are +/- 90 deg from the line.  Pick the one pointing
    # forward in base_link, since this is a front-wall calibration.
    nx = -dy
    ny = dx
    if nx < 0.0:
        nx = -nx
        ny = -ny
    normal_angle = math.atan2(ny, nx)
    distances = [abs(nx * (p[0] - mx) + ny * (p[1] - my)) for p in points]
    rms = math.sqrt(statistics.fmean(value * value for value in distances))
    projections = [dx * (p[0] - mx) + dy * (p[1] - my) for p in points]
    wall_length = percentile(projections, 0.95) - percentile(projections, 0.05)
    wall_distance = statistics.fmean(nx * p[0] + ny * p[1] for p in points)
    return line_angle, normal_angle, rms, wall_length, wall_distance


def fit_wall(
    points: Sequence[Point],
    iterations: int,
    threshold_m: float,
    max_points: int,
) -> Tuple[List[Point], Tuple[float, float, float, float, float]]:
    if len(points) < 20:
        raise ValueError(f"not enough front-wall points: {len(points)}")

    rng = random.Random(42)
    sample_points = list(points)
    if len(sample_points) > max_points:
        sample_points = rng.sample(sample_points, max_points)

    best_inliers: List[Point] = []
    best_rms = float("inf")
    for _ in range(iterations):
        p1, p2 = rng.sample(sample_points, 2)
        line = line_from_points(p1, p2)
        if line is None:
            continue
        inliers = [p for p in sample_points if line_distance(line, p) <= threshold_m]
        if len(inliers) < 20:
            continue
        rms = math.sqrt(statistics.fmean(line_distance(line, p) ** 2 for p in inliers))
        if len(inliers) > len(best_inliers) or (len(inliers) == len(best_inliers) and rms < best_rms):
            best_inliers = inliers
            best_rms = rms

    if not best_inliers:
        raise ValueError("failed to fit a front wall line")

    # Re-select inliers once after PCA refinement for a cleaner estimate.
    line_angle, _, _, _, _ = principal_line(best_inliers)
    dx = math.cos(line_angle)
    dy = math.sin(line_angle)
    mx = statistics.fmean(p[0] for p in best_inliers)
    my = statistics.fmean(p[1] for p in best_inliers)
    normal = (-dy, dx)
    if normal[0] < 0:
        normal = (-normal[0], -normal[1])
    refined = [
        p for p in sample_points
        if abs(normal[0] * (p[0] - mx) + normal[1] * (p[1] - my)) <= threshold_m
    ]
    if len(refined) >= 20:
        best_inliers = refined
    return best_inliers, principal_line(best_inliers)


def quality_warnings(
    point_count: int,
    inlier_count: int,
    rms_m: float,
    wall_length_m: float,
    yaw_error_deg: float,
) -> List[str]:
    warnings: List[str] = []
    if point_count < 200:
        warnings.append("few_front_points")
    if inlier_count < 80:
        warnings.append("few_wall_inliers")
    if point_count > 0 and inlier_count / point_count < 0.25:
        warnings.append("low_inlier_ratio")
    if rms_m > 0.04:
        warnings.append("wall_fit_rms_high")
    if wall_length_m < 1.0:
        warnings.append("wall_segment_short")
    if abs(yaw_error_deg) > 5.0:
        warnings.append("large_yaw_error_check_robot_square_to_wall")
    return warnings


def write_summary(path: str, report: dict) -> None:
    with open(path, "w", encoding="utf-8") as f:
        f.write("# Front Wall Lidar Yaw Calibration\n\n")
        f.write("This report estimates the planar yaw error of `base_link -> lidar_level_link` from a static front-wall scan.\n\n")
        f.write("## Recommendation\n\n")
        for key in (
            "current_lidar_yaw_rad",
            "current_lidar_yaw_deg",
            "fitted_wall_normal_deg_in_base",
            "expected_wall_normal_deg",
            "observed_minus_expected_yaw_error_deg",
            "suggested_lidar_yaw_rad",
            "suggested_lidar_yaw_deg",
            "suggested_delta_deg",
        ):
            f.write(f"- {key}: `{report[key]:.6f}`\n")
        f.write("\n## Fit Quality\n\n")
        f.write(f"- scan_topic: `{report['scan_topic']}`\n")
        f.write(f"- scan_frame: `{report['scan_frame']}`\n")
        f.write(f"- base_frame: `{report['base_frame']}`\n")
        f.write(f"- scan_count: `{report['scan_count']}`\n")
        f.write(f"- front_point_count: `{report['front_point_count']}`\n")
        f.write(f"- wall_inlier_count: `{report['wall_inlier_count']}`\n")
        f.write(f"- wall_inlier_ratio: `{report['wall_inlier_ratio']:.6f}`\n")
        f.write(f"- wall_fit_rms_m: `{report['wall_fit_rms_m']:.6f}`\n")
        f.write(f"- wall_length_m: `{report['wall_length_m']:.6f}`\n")
        f.write(f"- wall_distance_m: `{report['wall_distance_m']:.6f}`\n")
        f.write(f"- warnings: `{', '.join(report['warnings']) if report['warnings'] else 'none'}`\n")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--scan-topic", default="/scan")
    parser.add_argument("--base-frame", default="base_link")
    parser.add_argument("--current-config", required=True)
    parser.add_argument("--collect-sec", type=float, default=3.0)
    parser.add_argument("--max-scans", type=int, default=60)
    parser.add_argument("--front-half-angle-deg", type=float, default=15.0)
    parser.add_argument("--range-min-m", type=float, default=0.5)
    parser.add_argument("--range-max-m", type=float, default=8.0)
    parser.add_argument("--expected-normal-deg", type=float, default=0.0)
    parser.add_argument("--ransac-iterations", type=int, default=1200)
    parser.add_argument("--line-inlier-threshold-m", type=float, default=0.03)
    parser.add_argument("--max-fit-points", type=int, default=5000)
    parser.add_argument("--output-json")
    parser.add_argument("--summary-md")
    args = parser.parse_args()

    current_yaw = parse_current_lidar_yaw(args.current_config)
    rclpy.init()
    node = ScanCollector(
        scan_topic=args.scan_topic,
        base_frame=args.base_frame,
        collect_sec=max(0.5, args.collect_sec),
        range_min_m=args.range_min_m,
        range_max_m=args.range_max_m,
        front_half_angle_rad=math.radians(args.front_half_angle_deg),
        max_scans=max(1, args.max_scans),
    )
    deadline = time.monotonic() + max(2.0, args.collect_sec + 2.0)
    while rclpy.ok() and not node.done() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)

    points = node.points
    scan_count = node.scan_count
    scan_frame = node.scan_frame
    last_error = node.last_error
    node.destroy_node()
    rclpy.shutdown()

    if not points:
        raise SystemExit(f"no usable front scan points collected; last_tf_error={last_error}")

    inliers, fit = fit_wall(
        points,
        iterations=max(100, args.ransac_iterations),
        threshold_m=args.line_inlier_threshold_m,
        max_points=max(100, args.max_fit_points),
    )
    line_angle, normal_angle, rms_m, wall_length_m, wall_distance_m = fit
    expected_normal = math.radians(args.expected_normal_deg)
    yaw_error = norm_angle(normal_angle - expected_normal)
    suggested_yaw = norm_angle(current_yaw - yaw_error)
    warnings = quality_warnings(
        point_count=len(points),
        inlier_count=len(inliers),
        rms_m=rms_m,
        wall_length_m=wall_length_m,
        yaw_error_deg=math.degrees(yaw_error),
    )
    report = {
        "scan_topic": args.scan_topic,
        "scan_frame": scan_frame,
        "base_frame": args.base_frame,
        "current_config": args.current_config,
        "current_lidar_yaw_rad": current_yaw,
        "current_lidar_yaw_deg": math.degrees(current_yaw),
        "line_angle_deg_in_base": math.degrees(line_angle),
        "fitted_wall_normal_deg_in_base": math.degrees(normal_angle),
        "expected_wall_normal_deg": args.expected_normal_deg,
        "observed_minus_expected_yaw_error_rad": yaw_error,
        "observed_minus_expected_yaw_error_deg": math.degrees(yaw_error),
        "suggested_lidar_yaw_rad": suggested_yaw,
        "suggested_lidar_yaw_deg": math.degrees(suggested_yaw),
        "suggested_delta_rad": norm_angle(suggested_yaw - current_yaw),
        "suggested_delta_deg": math.degrees(norm_angle(suggested_yaw - current_yaw)),
        "scan_count": scan_count,
        "front_point_count": len(points),
        "wall_inlier_count": len(inliers),
        "wall_inlier_ratio": len(inliers) / len(points) if points else 0.0,
        "wall_fit_rms_m": rms_m,
        "wall_length_m": wall_length_m,
        "wall_distance_m": wall_distance_m,
        "front_half_angle_deg": args.front_half_angle_deg,
        "range_min_m": args.range_min_m,
        "range_max_m": args.range_max_m,
        "line_inlier_threshold_m": args.line_inlier_threshold_m,
        "warnings": warnings,
    }
    if args.output_json:
        os.makedirs(os.path.dirname(os.path.abspath(args.output_json)), exist_ok=True)
        with open(args.output_json, "w", encoding="utf-8") as f:
            json.dump(report, f, indent=2, sort_keys=True)
            f.write("\n")
    if args.summary_md:
        os.makedirs(os.path.dirname(os.path.abspath(args.summary_md)), exist_ok=True)
        write_summary(args.summary_md, report)

    print(
        "[front-wall-yaw] "
        f"error_deg={report['observed_minus_expected_yaw_error_deg']:.3f} "
        f"delta_deg={report['suggested_delta_deg']:.3f} "
        f"suggested_yaw_rad={report['suggested_lidar_yaw_rad']:.9f} "
        f"inliers={len(inliers)}/{len(points)} "
        f"rms_m={rms_m:.4f} length_m={wall_length_m:.3f} "
        f"warnings={','.join(warnings) if warnings else 'none'}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
