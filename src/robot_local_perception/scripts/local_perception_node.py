#!/usr/bin/env python3
from __future__ import annotations

from collections import defaultdict
from dataclasses import dataclass
import math
from typing import Iterable

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header, String
from tf2_ros import Buffer, TransformException, TransformListener


@dataclass(frozen=True)
class CropBox:
    enabled: bool
    min_x: float
    max_x: float
    min_y: float
    max_y: float
    min_z: float
    max_z: float

    def contains(self, x: float, y: float, z: float) -> bool:
        if not self.enabled:
            return False
        return (
            self.min_x <= x <= self.max_x
            and self.min_y <= y <= self.max_y
            and self.min_z <= z <= self.max_z
        )


@dataclass(frozen=True)
class AzimuthFilter:
    enabled: bool
    min_angle_rad: float
    max_angle_rad: float

    def contains(self, angle_rad: float) -> bool:
        if not self.enabled:
            return True
        if self.min_angle_rad <= self.max_angle_rad:
            return self.min_angle_rad <= angle_rad <= self.max_angle_rad
        return angle_rad >= self.min_angle_rad or angle_rad <= self.max_angle_rad


@dataclass(frozen=True)
class OutlierFilter:
    enabled: bool
    voxel_size: float
    min_points_per_voxel: int


@dataclass(frozen=True)
class ModeProfile:
    range_min: float
    range_max: float
    min_z: float
    max_z: float
    azimuth_filter: AzimuthFilter
    self_mask: CropBox
    front_mask: CropBox
    outlier_filter: OutlierFilter


@dataclass
class ClearingRayBin:
    has_return: bool
    range_xy: float
    angle_rad: float


def obstacle_fields(include_intensity: bool) -> list[PointField]:
    fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    if include_intensity:
        fields.append(PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1))
    return fields


class LocalPerceptionNode(Node):
    DEFAULT_PROFILES = {
        "NORMAL": {
            "range_filter.min": 0.5,
            "range_filter.max": 4.50,
            "height_filter.min_z": 0.40,
            "height_filter.max_z": 1.20,
            "azimuth_filter.enabled": True,
            "azimuth_filter.min_angle_deg": -110.0,
            "azimuth_filter.max_angle_deg": 110.0,
            "self_mask.enabled": True,
            "self_mask.min_x": -0.50,
            "self_mask.max_x": 0.45,
            "self_mask.min_y": -0.40,
            "self_mask.max_y": 0.40,
            "self_mask.min_z": -0.20,
            "self_mask.max_z": 1.40,
            "front_mask.enabled": False,
            "front_mask.min_x": 0.20,
            "front_mask.max_x": 0.55,
            "front_mask.min_y": -0.20,
            "front_mask.max_y": 0.20,
            "front_mask.min_z": -0.10,
            "front_mask.max_z": 1.60,
            "outlier_filter.enabled": True,
            "outlier_filter.voxel_size": 0.15,
            "outlier_filter.min_points_per_voxel": 2,
        },
        "RAMP": {
            "range_filter.min": 0.5,
            "range_filter.max": 35.0,
            "height_filter.min_z": -0.35,
            "height_filter.max_z": 1.80,
            "azimuth_filter.enabled": True,
            "azimuth_filter.min_angle_deg": -120.0,
            "azimuth_filter.max_angle_deg": 120.0,
            "self_mask.enabled": True,
            "self_mask.min_x": -0.50,
            "self_mask.max_x": 0.45,
            "self_mask.min_y": -0.40,
            "self_mask.max_y": 0.40,
            "self_mask.min_z": -0.20,
            "self_mask.max_z": 1.40,
            "front_mask.enabled": False,
            "front_mask.min_x": 0.20,
            "front_mask.max_x": 0.55,
            "front_mask.min_y": -0.20,
            "front_mask.max_y": 0.20,
            "front_mask.min_z": -0.10,
            "front_mask.max_z": 1.60,
            "outlier_filter.enabled": False,
            "outlier_filter.voxel_size": 0.15,
            "outlier_filter.min_points_per_voxel": 2,
        },
        "ELEVATOR_WAIT": {
            "range_filter.min": 0.3,
            "range_filter.max": 8.0,
            "height_filter.min_z": 0.02,
            "height_filter.max_z": 1.40,
            "azimuth_filter.enabled": True,
            "azimuth_filter.min_angle_deg": -95.0,
            "azimuth_filter.max_angle_deg": 95.0,
            "self_mask.enabled": True,
            "self_mask.min_x": -0.50,
            "self_mask.max_x": 0.45,
            "self_mask.min_y": -0.40,
            "self_mask.max_y": 0.40,
            "self_mask.min_z": -0.20,
            "self_mask.max_z": 1.40,
            "front_mask.enabled": False,
            "front_mask.min_x": 0.20,
            "front_mask.max_x": 0.55,
            "front_mask.min_y": -0.20,
            "front_mask.max_y": 0.20,
            "front_mask.min_z": -0.10,
            "front_mask.max_z": 1.40,
            "outlier_filter.enabled": True,
            "outlier_filter.voxel_size": 0.12,
            "outlier_filter.min_points_per_voxel": 2,
        },
        "DOORWAY": {
            "range_filter.min": 0.3,
            "range_filter.max": 12.0,
            "height_filter.min_z": -0.05,
            "height_filter.max_z": 1.45,
            "azimuth_filter.enabled": True,
            "azimuth_filter.min_angle_deg": -90.0,
            "azimuth_filter.max_angle_deg": 90.0,
            "self_mask.enabled": True,
            "self_mask.min_x": -0.50,
            "self_mask.max_x": 0.45,
            "self_mask.min_y": -0.40,
            "self_mask.max_y": 0.40,
            "self_mask.min_z": -0.20,
            "self_mask.max_z": 1.40,
            "front_mask.enabled": False,
            "front_mask.min_x": 0.20,
            "front_mask.max_x": 0.55,
            "front_mask.min_y": -0.20,
            "front_mask.max_y": 0.20,
            "front_mask.min_z": -0.10,
            "front_mask.max_z": 1.60,
            "outlier_filter.enabled": True,
            "outlier_filter.voxel_size": 0.10,
            "outlier_filter.min_points_per_voxel": 2,
        },
    }

    def __init__(self) -> None:
        super().__init__("robot_local_perception")
        self.declare_parameter("mock_mode", True)
        self.declare_parameter("mode", "NORMAL")
        self.declare_parameter("mode_topic", "/robot_mode")
        self.declare_parameter("input_topic", "/lidar_points")
        self.declare_parameter("output_topic", "/perception/obstacle_points")
        self.declare_parameter("clearing_output_topic", "/perception/clearing_points")
        self.declare_parameter("output_frame_id", "base_link")
        self.declare_parameter("restamp_to_now", True)
        self.declare_parameter("lookup_timeout_sec", 0.1)
        self.declare_parameter("processing_rate_hz", 8.0)
        self.declare_parameter("point_sample_stride", 4)
        self.declare_parameter("max_filtered_points", 12000)
        self.declare_parameter("clearing.enabled", True)
        self.declare_parameter("clearing.range_filter.min", 0.10)
        self.declare_parameter("clearing.range_filter.max", 5.00)
        self.declare_parameter("clearing.height_filter.min_z", -0.30)
        self.declare_parameter("clearing.height_filter.max_z", 1.40)
        self.declare_parameter("clearing.point_sample_stride", 4)
        self.declare_parameter("clearing.max_points", 72000)
        self.declare_parameter("clearing.virtual_rays.enabled", True)
        self.declare_parameter("clearing.virtual_rays.angular_resolution_deg", 0.5)
        self.declare_parameter("clearing.virtual_rays.range", 6.00)
        self.declare_parameter("clearing.virtual_rays.range_steps", [0.35, 0.50, 0.75, 1.25, 2.00, 3.50, 6.00])
        self.declare_parameter(
            "clearing.virtual_rays.endpoint_z_values",
            [
                -0.15,
                -0.05,
                0.00,
                0.05,
                0.10,
                0.15,
                0.20,
                0.25,
                0.30,
                0.35,
                0.45,
                0.55,
                0.65,
                0.75,
                0.85,
                0.95,
                1.05,
                1.15,
                1.25,
                1.35,
            ],
        )
        self.declare_parameter("publish_debug_log", False)
        self.declare_parameter(
            "supported_modes",
            ["NORMAL", "RAMP", "ELEVATOR_WAIT", "DOORWAY"],
        )
        self.declare_parameter("local_nav_preprocessor_reference", "")
        supported_modes = [str(mode) for mode in self.get_parameter("supported_modes").value]
        self.supported_modes = set(supported_modes)
        default_mode = str(self.get_parameter("mode").value)
        self.current_mode = default_mode if default_mode in self.supported_modes else supported_modes[0]
        self.lookup_timeout_sec = float(self.get_parameter("lookup_timeout_sec").value)
        self.publish_debug_log = bool(self.get_parameter("publish_debug_log").value)
        processing_rate_hz = max(float(self.get_parameter("processing_rate_hz").value), 0.1)
        self.sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.profiles = self.load_profiles(supported_modes)
        self.pub = self.create_publisher(
            PointCloud2,
            self.get_parameter("output_topic").value,
            self.sensor_qos,
        )
        self.clearing_pub = self.create_publisher(
            PointCloud2,
            self.get_parameter("clearing_output_topic").value,
            self.sensor_qos,
        )
        self.mode_pub = self.create_publisher(String, "/perception/mode", 10)
        self.latest_cloud: PointCloud2 | None = None
        self.latest_cloud_seq = 0
        self.last_processed_cloud_seq = 0
        self.create_subscription(
            PointCloud2,
            self.get_parameter("input_topic").value,
            self.on_cloud,
            self.sensor_qos,
        )
        mode_topic = str(self.get_parameter("mode_topic").value)
        if mode_topic:
            self.create_subscription(String, mode_topic, self.on_mode, 10)
        self.create_timer(1.0 / processing_rate_hz, self.process_latest_cloud)

    def declare_mode_profile_parameters(self, mode_name: str) -> None:
        for suffix, value in self.DEFAULT_PROFILES[mode_name].items():
            self.declare_parameter(f"profiles.{mode_name}.{suffix}", value)

    def load_profiles(self, modes: Iterable[str]) -> dict[str, ModeProfile]:
        profiles: dict[str, ModeProfile] = {}
        for mode in modes:
            self.declare_mode_profile_parameters(mode)
            prefix = f"profiles.{mode}."
            profiles[mode] = ModeProfile(
                range_min=float(self.get_parameter(prefix + "range_filter.min").value),
                range_max=float(self.get_parameter(prefix + "range_filter.max").value),
                min_z=float(self.get_parameter(prefix + "height_filter.min_z").value),
                max_z=float(self.get_parameter(prefix + "height_filter.max_z").value),
                azimuth_filter=AzimuthFilter(
                    enabled=bool(self.get_parameter(prefix + "azimuth_filter.enabled").value),
                    min_angle_rad=math.radians(
                        float(self.get_parameter(prefix + "azimuth_filter.min_angle_deg").value)
                    ),
                    max_angle_rad=math.radians(
                        float(self.get_parameter(prefix + "azimuth_filter.max_angle_deg").value)
                    ),
                ),
                self_mask=CropBox(
                    enabled=bool(self.get_parameter(prefix + "self_mask.enabled").value),
                    min_x=float(self.get_parameter(prefix + "self_mask.min_x").value),
                    max_x=float(self.get_parameter(prefix + "self_mask.max_x").value),
                    min_y=float(self.get_parameter(prefix + "self_mask.min_y").value),
                    max_y=float(self.get_parameter(prefix + "self_mask.max_y").value),
                    min_z=float(self.get_parameter(prefix + "self_mask.min_z").value),
                    max_z=float(self.get_parameter(prefix + "self_mask.max_z").value),
                ),
                front_mask=CropBox(
                    enabled=bool(self.get_parameter(prefix + "front_mask.enabled").value),
                    min_x=float(self.get_parameter(prefix + "front_mask.min_x").value),
                    max_x=float(self.get_parameter(prefix + "front_mask.max_x").value),
                    min_y=float(self.get_parameter(prefix + "front_mask.min_y").value),
                    max_y=float(self.get_parameter(prefix + "front_mask.max_y").value),
                    min_z=float(self.get_parameter(prefix + "front_mask.min_z").value),
                    max_z=float(self.get_parameter(prefix + "front_mask.max_z").value),
                ),
                outlier_filter=OutlierFilter(
                    enabled=bool(self.get_parameter(prefix + "outlier_filter.enabled").value),
                    voxel_size=float(self.get_parameter(prefix + "outlier_filter.voxel_size").value),
                    min_points_per_voxel=int(
                        self.get_parameter(prefix + "outlier_filter.min_points_per_voxel").value
                    ),
                ),
            )
        return profiles

    def active_profile(self) -> ModeProfile:
        return self.profiles[self.current_mode]

    def on_mode(self, msg: String) -> None:
        requested_mode = str(msg.data).strip()
        if requested_mode in self.supported_modes:
            self.current_mode = requested_mode
            return
        self.get_logger().warning(f"ignoring unsupported perception mode: {requested_mode}")

    def lookup_transform_matrix(
        self, target_frame: str, source_frame: str, stamp
    ) -> tuple[tuple[float, ...], tuple[float, float, float]]:
        transform = self.tf_buffer.lookup_transform(
            target_frame,
            source_frame,
            Time.from_msg(stamp),
            timeout=Duration(seconds=self.lookup_timeout_sec),
        )
        tx = float(transform.transform.translation.x)
        ty = float(transform.transform.translation.y)
        tz = float(transform.transform.translation.z)
        qx = float(transform.transform.rotation.x)
        qy = float(transform.transform.rotation.y)
        qz = float(transform.transform.rotation.z)
        qw = float(transform.transform.rotation.w)
        norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
        if norm == 0.0:
            raise TransformException("received zero-length quaternion")
        qx /= norm
        qy /= norm
        qz /= norm
        qw /= norm
        return (
            (
                1.0 - 2.0 * (qy * qy + qz * qz),
                2.0 * (qx * qy - qz * qw),
                2.0 * (qx * qz + qy * qw),
                2.0 * (qx * qy + qz * qw),
                1.0 - 2.0 * (qx * qx + qz * qz),
                2.0 * (qy * qz - qx * qw),
                2.0 * (qx * qz - qy * qw),
                2.0 * (qy * qz + qx * qw),
                1.0 - 2.0 * (qx * qx + qy * qy),
            ),
            (tx, ty, tz),
        )

    def apply_transform(
        self,
        rotation_matrix: tuple[float, ...],
        translation: tuple[float, float, float],
        x: float,
        y: float,
        z: float,
    ) -> tuple[float, float, float]:
        r00, r01, r02, r10, r11, r12, r20, r21, r22 = rotation_matrix
        tx, ty, tz = translation
        return (
            r00 * x + r01 * y + r02 * z + tx,
            r10 * x + r11 * y + r12 * z + ty,
            r20 * x + r21 * y + r22 * z + tz,
        )

    def passes_filters(self, profile: ModeProfile, x: float, y: float, z: float) -> bool:
        range_xy = math.hypot(x, y)
        if range_xy < profile.range_min or range_xy > profile.range_max:
            return False
        if z < profile.min_z or z > profile.max_z:
            return False
        if not profile.azimuth_filter.contains(math.atan2(y, x)):
            return False
        if profile.self_mask.contains(x, y, z) or profile.front_mask.contains(x, y, z):
            return False
        return True

    def passes_clearing_filters(self, profile: ModeProfile, x: float, y: float, z: float) -> bool:
        range_xy = math.hypot(x, y)
        range_min = float(self.get_parameter("clearing.range_filter.min").value)
        range_max = float(self.get_parameter("clearing.range_filter.max").value)
        min_z = float(self.get_parameter("clearing.height_filter.min_z").value)
        max_z = float(self.get_parameter("clearing.height_filter.max_z").value)
        if range_xy < range_min or range_xy > range_max:
            return False
        if z < min_z or z > max_z:
            return False
        if not profile.azimuth_filter.contains(math.atan2(y, x)):
            return False
        if profile.self_mask.contains(x, y, z) or profile.front_mask.contains(x, y, z):
            return False
        return True

    def clearing_azimuth_range(self, profile: ModeProfile) -> tuple[float, float]:
        if not profile.azimuth_filter.enabled or profile.azimuth_filter.min_angle_rad > profile.azimuth_filter.max_angle_rad:
            return (-math.pi, math.pi)
        return (profile.azimuth_filter.min_angle_rad, profile.azimuth_filter.max_angle_rad)

    def make_clearing_bins(self, profile: ModeProfile, angle_resolution_rad: float) -> list[ClearingRayBin]:
        min_angle, max_angle = self.clearing_azimuth_range(profile)
        span = max(max_angle - min_angle, angle_resolution_rad)
        bin_count = int(math.ceil(span / angle_resolution_rad)) + 1
        return [
            ClearingRayBin(
                has_return=False,
                range_xy=0.0,
                angle_rad=min_angle + (index + 0.5) * angle_resolution_rad,
            )
            for index in range(bin_count)
        ]

    def update_clearing_bin(
        self,
        bins: list[ClearingRayBin],
        profile: ModeProfile,
        angle_resolution_rad: float,
        x: float,
        y: float,
        ray_origin_x: float,
        ray_origin_y: float,
    ) -> None:
        if not bins:
            return
        min_angle, max_angle = self.clearing_azimuth_range(profile)
        dx = x - ray_origin_x
        dy = y - ray_origin_y
        angle = math.atan2(dy, dx)
        if angle < min_angle or angle > max_angle:
            return
        range_xy = math.hypot(dx, dy)
        raw_index = math.floor((angle - min_angle) / angle_resolution_rad)
        index = min(max(int(raw_index), 0), len(bins) - 1)
        if not bins[index].has_return or range_xy > bins[index].range_xy:
            bins[index].has_return = True
            bins[index].range_xy = range_xy
            bins[index].angle_rad = angle

    def build_virtual_clearing_points(
        self,
        bins: list[ClearingRayBin],
        profile: ModeProfile,
        include_intensity: bool,
        ray_origin_x: float,
        ray_origin_y: float,
    ) -> list[tuple[float, ...]]:
        clearing_range_min = float(self.get_parameter("clearing.range_filter.min").value)
        virtual_range = max(
            clearing_range_min,
            float(self.get_parameter("clearing.virtual_rays.range").value),
        )
        range_steps = sorted(
            {
                round(float(range_xy), 3)
                for range_xy in self.get_parameter("clearing.virtual_rays.range_steps").value
                if math.isfinite(float(range_xy)) and clearing_range_min <= float(range_xy) <= virtual_range
            }
        )
        if not range_steps:
            range_steps = [virtual_range]
        elif abs(range_steps[-1] - virtual_range) > 1e-3:
            range_steps.append(virtual_range)
        clearing_min_z = float(self.get_parameter("clearing.height_filter.min_z").value)
        clearing_max_z = float(self.get_parameter("clearing.height_filter.max_z").value)
        endpoint_z_values = [
            float(z)
            for z in self.get_parameter("clearing.virtual_rays.endpoint_z_values").value
            if clearing_min_z <= float(z) <= clearing_max_z
        ]
        if not endpoint_z_values:
            endpoint_z_values = [0.5 * (clearing_min_z + clearing_max_z)]
        points: list[tuple[float, ...]] = []
        for ray_bin in bins:
            if not profile.azimuth_filter.contains(ray_bin.angle_rad):
                continue
            max_range_xy = min(
                max(ray_bin.range_xy if ray_bin.has_return else virtual_range, clearing_range_min),
                virtual_range,
            )
            ray_ranges = [range_xy for range_xy in range_steps if range_xy < max_range_xy - 1e-3]
            ray_ranges.append(max_range_xy)
            for range_xy in ray_ranges:
                x = ray_origin_x + math.cos(ray_bin.angle_rad) * range_xy
                y = ray_origin_y + math.sin(ray_bin.angle_rad) * range_xy
                for z in endpoint_z_values:
                    if profile.self_mask.contains(x, y, z) or profile.front_mask.contains(x, y, z):
                        continue
                    if include_intensity:
                        points.append((x, y, z, 0.0))
                    else:
                        points.append((x, y, z))
        return points

    def apply_voxel_outlier_filter(
        self, profile: ModeProfile, points: list[tuple[float, ...]]
    ) -> list[tuple[float, ...]]:
        if not profile.outlier_filter.enabled or not points:
            return points
        voxel_size = max(profile.outlier_filter.voxel_size, 1e-3)
        min_points = max(profile.outlier_filter.min_points_per_voxel, 1)
        counts: dict[tuple[int, int, int], int] = defaultdict(int)
        keys: list[tuple[int, int, int]] = []
        for point in points:
            key = (
                math.floor(point[0] / voxel_size),
                math.floor(point[1] / voxel_size),
                math.floor(point[2] / voxel_size),
            )
            counts[key] += 1
            keys.append(key)
        return [point for point, key in zip(points, keys) if counts[key] >= min_points]

    def on_cloud(self, msg: PointCloud2) -> None:
        self.latest_cloud = msg
        self.latest_cloud_seq += 1

    def process_latest_cloud(self) -> None:
        if self.latest_cloud is None or self.latest_cloud_seq == self.last_processed_cloud_seq:
            return

        msg = self.latest_cloud
        self.last_processed_cloud_seq = self.latest_cloud_seq
        output_frame = str(self.get_parameter("output_frame_id").value)
        restamp_to_now = bool(self.get_parameter("restamp_to_now").value)
        point_sample_stride = max(int(self.get_parameter("point_sample_stride").value), 1)
        max_filtered_points = max(int(self.get_parameter("max_filtered_points").value), 1)
        clearing_enabled = bool(self.get_parameter("clearing.enabled").value)
        clearing_point_sample_stride = max(int(self.get_parameter("clearing.point_sample_stride").value), 1)
        clearing_max_points = max(int(self.get_parameter("clearing.max_points").value), 1)
        clearing_virtual_rays_enabled = bool(self.get_parameter("clearing.virtual_rays.enabled").value)
        clearing_angle_resolution_rad = math.radians(
            min(max(float(self.get_parameter("clearing.virtual_rays.angular_resolution_deg").value), 0.2), 10.0)
        )
        field_names = [field.name for field in msg.fields]
        intensity_name = next(
            (name for name in ("intensity", "i", "reflectivity") if name in field_names),
            None,
        )
        read_fields = ["x", "y", "z"] + ([intensity_name] if intensity_name else [])
        try:
            if msg.header.frame_id != output_frame:
                rotation_matrix, translation = self.lookup_transform_matrix(
                    output_frame, msg.header.frame_id, msg.header.stamp
                )
            else:
                rotation_matrix = (1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0)
                translation = (0.0, 0.0, 0.0)
        except TransformException as ex:
            self.get_logger().warning(
                f"skipping obstacle cloud because transform {output_frame} <- {msg.header.frame_id} is unavailable: {ex}"
            )
            return

        profile = self.active_profile()
        filtered_points: list[tuple[float, ...]] = []
        clearing_points: list[tuple[float, ...]] = []
        ray_origin_x = float(translation[0])
        ray_origin_y = float(translation[1])
        clearing_bins = (
            self.make_clearing_bins(profile, clearing_angle_resolution_rad)
            if clearing_virtual_rays_enabled
            else []
        )
        for point_index, point in enumerate(
            point_cloud2.read_points(msg, field_names=read_fields, skip_nans=True)
        ):
            x, y, z = self.apply_transform(
                rotation_matrix,
                translation,
                float(point[0]),
                float(point[1]),
                float(point[2]),
            )
            if (
                clearing_enabled
                and point_index % clearing_point_sample_stride == 0
                and self.passes_clearing_filters(profile, x, y, z)
            ):
                if clearing_virtual_rays_enabled:
                    self.update_clearing_bin(
                        clearing_bins,
                        profile,
                        clearing_angle_resolution_rad,
                        x,
                        y,
                        ray_origin_x,
                        ray_origin_y,
                    )
                elif intensity_name:
                    clearing_points.append((x, y, z, float(point[3])))
                else:
                    clearing_points.append((x, y, z))
            if point_index % point_sample_stride != 0:
                continue
            if not self.passes_filters(profile, x, y, z):
                continue
            if intensity_name:
                filtered_points.append((x, y, z, float(point[3])))
            else:
                filtered_points.append((x, y, z))

        filtered_points = self.apply_voxel_outlier_filter(profile, filtered_points)
        if clearing_enabled and clearing_virtual_rays_enabled:
            clearing_points = self.build_virtual_clearing_points(
                clearing_bins,
                profile,
                include_intensity=bool(intensity_name),
                ray_origin_x=ray_origin_x,
                ray_origin_y=ray_origin_y,
            )
        if len(filtered_points) > max_filtered_points:
            reduction_stride = math.ceil(len(filtered_points) / max_filtered_points)
            filtered_points = filtered_points[::reduction_stride]
        if len(clearing_points) > clearing_max_points:
            clearing_reduction_stride = math.ceil(len(clearing_points) / clearing_max_points)
            clearing_points = clearing_points[::clearing_reduction_stride]
        output_header = Header()
        output_header.frame_id = output_frame
        output_header.stamp = self.get_clock().now().to_msg() if restamp_to_now else msg.header.stamp
        if clearing_enabled:
            clearing_output = point_cloud2.create_cloud(
                output_header,
                obstacle_fields(include_intensity=bool(intensity_name)),
                clearing_points,
            )
            clearing_output.is_dense = False
            self.clearing_pub.publish(clearing_output)
        output = point_cloud2.create_cloud(
            output_header,
            obstacle_fields(include_intensity=bool(intensity_name)),
            filtered_points,
        )
        output.is_dense = False
        self.pub.publish(output)
        self.mode_pub.publish(String(data=self.current_mode))
        if self.publish_debug_log:
            self.get_logger().info(
                "local perception "
                f"mode={self.current_mode} input={msg.width} "
                f"sample_stride={point_sample_stride} output={len(filtered_points)} "
                f"clearing_output={len(clearing_points)}"
            )


def main() -> None:
    rclpy.init()
    node = LocalPerceptionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
