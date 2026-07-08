#!/usr/bin/env python3
"""Estimate lidar yaw error from a 3D point cloud front-wall fit.

The robot must be physically square to a straight vertical wall segment.  This
script does not use map, odom, AMCL, or Isaac output; it transforms a 3D point
cloud into base_link with the current TF, crops a front ROI, and fits the
dominant wall line in XY.
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
from pathlib import Path
from typing import List, Optional, Tuple

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_ros import Buffer, TransformException, TransformListener

from fit_front_wall_lidar_yaw import (
    fit_wall,
    norm_angle,
    parse_float_list,
    parse_current_lidar_yaw,
    quality_warnings,
    yaw_from_quaternion,
)


Point2 = Tuple[float, float]
Point3 = Tuple[float, float, float]


class Transform3D:
    def __init__(self, translation: Point3, rotation: Tuple[float, float, float, float], source_frame: str) -> None:
        self.tx, self.ty, self.tz = translation
        x, y, z, w = rotation
        self.source_frame = source_frame
        self.yaw = yaw_from_quaternion(x, y, z, w)
        self.matrix = (
            (
                1.0 - 2.0 * (y * y + z * z),
                2.0 * (x * y - z * w),
                2.0 * (x * z + y * w),
            ),
            (
                2.0 * (x * y + z * w),
                1.0 - 2.0 * (x * x + z * z),
                2.0 * (y * z - x * w),
            ),
            (
                2.0 * (x * z - y * w),
                2.0 * (y * z + x * w),
                1.0 - 2.0 * (x * x + y * y),
            ),
        )

    def apply(self, point: Point3) -> Point3:
        x, y, z = point
        m = self.matrix
        return (
            self.tx + m[0][0] * x + m[0][1] * y + m[0][2] * z,
            self.ty + m[1][0] * x + m[1][1] * y + m[1][2] * z,
            self.tz + m[2][0] * x + m[2][1] * y + m[2][2] * z,
        )


def quaternion_from_rpy(roll: float, pitch: float, yaw: float) -> Tuple[float, float, float, float]:
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def parse_lidar_transform_config(path: str) -> Tuple[Point3, Tuple[float, float, float]]:
    text = Path(path).read_text(encoding="utf-8")

    def scalar(name: str) -> Optional[float]:
        match = re.search(rf"(?m)^\s*{re.escape(name)}\s*:\s*([-+0-9.eE]+)\s*$", text)
        return float(match.group(1)) if match else None

    flat = [scalar(name) for name in ("lidar_x", "lidar_y", "lidar_z", "lidar_roll", "lidar_pitch", "lidar_yaw")]
    if all(value is not None for value in flat):
        return (float(flat[0]), float(flat[1]), float(flat[2])), (
            float(flat[3]),
            float(flat[4]),
            float(flat[5]),
        )

    xyz_match = re.search(r"(?m)^\s*lidar_xyz\s*:\s*\[([^\]]+)\]\s*$", text)
    rpy_match = re.search(r"(?m)^\s*lidar_rpy\s*:\s*\[([^\]]+)\]\s*$", text)
    if xyz_match and rpy_match:
        xyz = parse_float_list(xyz_match.group(1))
        rpy = parse_float_list(rpy_match.group(1))
        if len(xyz) >= 3 and len(rpy) >= 3:
            return (xyz[0], xyz[1], xyz[2]), (rpy[0], rpy[1], rpy[2])

    raise ValueError(f"could not read lidar transform from {path}")


def normalize_frame_name(frame: str) -> str:
    return frame.strip().lstrip("/")


class CloudCollector(Node):
    def __init__(
        self,
        cloud_topic: str,
        base_frame: str,
        collect_sec: float,
        max_clouds: int,
        x_min_m: float,
        x_max_m: float,
        y_abs_max_m: float,
        z_min_m: float,
        z_max_m: float,
        point_stride: int,
        max_points: int,
        config_transform: Transform3D,
        level_config_transform: Transform3D,
    ) -> None:
        super().__init__("front_wall_lidar_yaw_pointcloud_fit")
        self.base_frame = base_frame
        self.collect_sec = collect_sec
        self.max_clouds = max_clouds
        self.x_min_m = x_min_m
        self.x_max_m = x_max_m
        self.y_abs_max_m = y_abs_max_m
        self.z_min_m = z_min_m
        self.z_max_m = z_max_m
        self.point_stride = max(1, point_stride)
        self.max_points = max_points
        self.config_transform = config_transform
        self.level_config_transform = level_config_transform
        self.started = time.monotonic()
        self.points_xy: List[Point2] = []
        self.cloud_count = 0
        self.raw_cloud_count = 0
        self.raw_point_count = 0
        self.cloud_frame = ""
        self.last_error = ""
        self.transform: Optional[Transform3D] = None
        self.min_x = float("inf")
        self.max_x = float("-inf")
        self.min_y = float("inf")
        self.max_y = float("-inf")
        self.min_z = float("inf")
        self.max_z = float("-inf")
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(PointCloud2, cloud_topic, self._cloud_cb, qos_profile_sensor_data)

    def done(self) -> bool:
        return (
            (time.monotonic() - self.started) >= self.collect_sec or
            self.cloud_count >= self.max_clouds or
            len(self.points_xy) >= self.max_points
        )

    def _lookup_transform(self, source_frame: str) -> Optional[Transform3D]:
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
        return Transform3D(
            translation=(float(tr.x), float(tr.y), float(tr.z)),
            rotation=(float(q.x), float(q.y), float(q.z), float(q.w)),
            source_frame=source_frame,
        )

    def _config_transform_for_frame(self, source_frame: str) -> Optional[Transform3D]:
        frame = normalize_frame_name(source_frame)
        if frame == normalize_frame_name(self.base_frame):
            return Transform3D((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0), source_frame)
        if frame == "lidar_link":
            return self.config_transform
        if frame == "lidar_level_link":
            return self.level_config_transform
        return None

    def _cloud_cb(self, msg: PointCloud2) -> None:
        if self.done():
            return
        source_frame = msg.header.frame_id or "lidar_link"
        transform = self._lookup_transform(source_frame)
        if transform is None:
            transform = self._config_transform_for_frame(source_frame)
            if transform is None:
                return
            self.last_error = (
                "tf lookup unavailable; using current sensors.yaml transform for "
                f"{source_frame}: {self.last_error}"
            )
        self.transform = transform
        self.cloud_frame = source_frame
        self.raw_cloud_count += 1
        added = 0
        for idx, raw in enumerate(point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)):
            if idx % self.point_stride != 0:
                continue
            bx, by, bz = transform.apply((float(raw[0]), float(raw[1]), float(raw[2])))
            self.raw_point_count += 1
            self.min_x = min(self.min_x, bx)
            self.max_x = max(self.max_x, bx)
            self.min_y = min(self.min_y, by)
            self.max_y = max(self.max_y, by)
            self.min_z = min(self.min_z, bz)
            self.max_z = max(self.max_z, bz)
            if (
                self.x_min_m <= bx <= self.x_max_m and
                abs(by) <= self.y_abs_max_m and
                self.z_min_m <= bz <= self.z_max_m
            ):
                self.points_xy.append((bx, by))
                added += 1
                if len(self.points_xy) >= self.max_points:
                    break
        if added > 0:
            self.cloud_count += 1


def write_summary(path: str, report: dict) -> None:
    with open(path, "w", encoding="utf-8") as f:
        f.write("# Front Wall 3D PointCloud Lidar Yaw Calibration\n\n")
        f.write("This report estimates `base_link -> lidar_level_link` yaw from a cropped `/lidar_points` front-wall fit.\n\n")
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
        for key in (
            "cloud_topic",
            "cloud_frame",
            "base_frame",
            "cloud_count",
            "front_point_count",
            "wall_inlier_count",
            "wall_inlier_ratio",
            "wall_fit_rms_m",
            "wall_length_m",
            "wall_distance_m",
            "x_min_m",
            "x_max_m",
            "y_abs_max_m",
            "z_min_m",
            "z_max_m",
            "point_stride",
        ):
            value = report[key]
            if isinstance(value, float):
                f.write(f"- {key}: `{value:.6f}`\n")
            else:
                f.write(f"- {key}: `{value}`\n")
        f.write(f"- warnings: `{', '.join(report['warnings']) if report['warnings'] else 'none'}`\n")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--cloud-topic", default="/lidar_points")
    parser.add_argument("--base-frame", default="base_link")
    parser.add_argument("--current-config", required=True)
    parser.add_argument("--collect-sec", type=float, default=3.0)
    parser.add_argument("--max-clouds", type=int, default=8)
    parser.add_argument("--x-min-m", type=float, default=1.0)
    parser.add_argument("--x-max-m", type=float, default=8.0)
    parser.add_argument("--y-abs-max-m", type=float, default=1.2)
    parser.add_argument("--z-min-m", type=float, default=0.5)
    parser.add_argument("--z-max-m", type=float, default=2.6)
    parser.add_argument("--point-stride", type=int, default=8)
    parser.add_argument("--max-points", type=int, default=8000)
    parser.add_argument("--expected-normal-deg", type=float, default=0.0)
    parser.add_argument("--ransac-iterations", type=int, default=1500)
    parser.add_argument("--line-inlier-threshold-m", type=float, default=0.04)
    parser.add_argument("--output-json")
    parser.add_argument("--summary-md")
    args = parser.parse_args()

    current_yaw = parse_current_lidar_yaw(args.current_config)
    lidar_xyz, lidar_rpy = parse_lidar_transform_config(args.current_config)
    config_transform = Transform3D(
        translation=lidar_xyz,
        rotation=quaternion_from_rpy(*lidar_rpy),
        source_frame="lidar_link",
    )
    level_config_transform = Transform3D(
        translation=lidar_xyz,
        rotation=quaternion_from_rpy(0.0, 0.0, lidar_rpy[2]),
        source_frame="lidar_level_link",
    )
    rclpy.init()
    node = CloudCollector(
        cloud_topic=args.cloud_topic,
        base_frame=args.base_frame,
        collect_sec=max(0.5, args.collect_sec),
        max_clouds=max(1, args.max_clouds),
        x_min_m=args.x_min_m,
        x_max_m=args.x_max_m,
        y_abs_max_m=args.y_abs_max_m,
        z_min_m=args.z_min_m,
        z_max_m=args.z_max_m,
        point_stride=args.point_stride,
        max_points=max(100, args.max_points),
        config_transform=config_transform,
        level_config_transform=level_config_transform,
    )
    deadline = time.monotonic() + max(2.0, args.collect_sec + 3.0)
    while rclpy.ok() and not node.done() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)

    points = list(node.points_xy)
    cloud_count = node.cloud_count
    raw_cloud_count = node.raw_cloud_count
    raw_point_count = node.raw_point_count
    cloud_frame = node.cloud_frame
    last_error = node.last_error
    raw_extent = {
        "min_x": node.min_x,
        "max_x": node.max_x,
        "min_y": node.min_y,
        "max_y": node.max_y,
        "min_z": node.min_z,
        "max_z": node.max_z,
    }
    node.destroy_node()
    rclpy.shutdown()

    if not points:
        extent_text = "no_raw_points"
        if raw_point_count > 0:
            extent_text = (
                f"x=[{raw_extent['min_x']:.3f},{raw_extent['max_x']:.3f}] "
                f"y=[{raw_extent['min_y']:.3f},{raw_extent['max_y']:.3f}] "
                f"z=[{raw_extent['min_z']:.3f},{raw_extent['max_z']:.3f}]"
            )
        raise SystemExit(
            "no usable front point-cloud points collected; "
            f"raw_clouds={raw_cloud_count} raw_points={raw_point_count} "
            f"cloud_frame={cloud_frame} transformed_extent_base={extent_text} "
            f"roi=x[{args.x_min_m},{args.x_max_m}] y_abs<={args.y_abs_max_m} "
            f"z[{args.z_min_m},{args.z_max_m}] last_tf_error={last_error}"
        )

    # Shuffle once so repeated scans do not over-weight cloud ordering.
    rng = random.Random(42)
    rng.shuffle(points)
    inliers, fit = fit_wall(
        points,
        iterations=max(100, args.ransac_iterations),
        threshold_m=args.line_inlier_threshold_m,
        max_points=max(100, args.max_points),
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
        "cloud_topic": args.cloud_topic,
        "cloud_frame": cloud_frame,
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
        "cloud_count": cloud_count,
        "raw_cloud_count": raw_cloud_count,
        "raw_point_count": raw_point_count,
        "raw_extent_base": raw_extent,
        "front_point_count": len(points),
        "wall_inlier_count": len(inliers),
        "wall_inlier_ratio": len(inliers) / len(points) if points else 0.0,
        "wall_fit_rms_m": rms_m,
        "wall_length_m": wall_length_m,
        "wall_distance_m": wall_distance_m,
        "x_min_m": args.x_min_m,
        "x_max_m": args.x_max_m,
        "y_abs_max_m": args.y_abs_max_m,
        "z_min_m": args.z_min_m,
        "z_max_m": args.z_max_m,
        "point_stride": args.point_stride,
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
        "[front-wall-yaw-3d] "
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
