#!/usr/bin/env python3
from __future__ import annotations

import binascii
import csv
import json
import math
import struct
import zlib
from bisect import bisect_left
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Optional

import numpy as np


UNKNOWN_PIXEL = 205
FREE_PIXEL = 254
OCCUPIED_PIXEL = 0


@dataclass
class Pose2D:
    x: float
    y: float
    z: float
    yaw: float


def _find_field_offset(msg, name: str) -> Optional[int]:
    for field in msg.fields:
        if field.name == name:
            return int(field.offset)
    return None


def parse_pointcloud_xyz(msg) -> np.ndarray:
    if not msg.data or msg.point_step <= 0:
        return np.empty((0, 3), dtype=np.float32)
    x_offset = _find_field_offset(msg, "x")
    y_offset = _find_field_offset(msg, "y")
    z_offset = _find_field_offset(msg, "z")
    if x_offset is None or y_offset is None or z_offset is None or msg.is_bigendian:
        return np.empty((0, 3), dtype=np.float32)
    point_count = int(msg.width) * int(msg.height)
    if point_count <= 0:
        return np.empty((0, 3), dtype=np.float32)
    dtype = np.dtype(
        {
            "names": ["x", "y", "z"],
            "formats": ["<f4", "<f4", "<f4"],
            "offsets": [x_offset, y_offset, z_offset],
            "itemsize": int(msg.point_step),
        }
    )
    cloud = np.frombuffer(msg.data, dtype=dtype, count=point_count)
    points = np.column_stack((cloud["x"], cloud["y"], cloud["z"])).astype(np.float32, copy=False)
    if points.size == 0:
        return np.empty((0, 3), dtype=np.float32)
    finite_mask = np.isfinite(points).all(axis=1)
    return points[finite_mask]


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def euler_to_quaternion(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
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


def quaternion_to_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    xx = x * x
    yy = y * y
    zz = z * z
    xy = x * y
    xz = x * z
    yz = y * z
    wx = w * x
    wy = w * y
    wz = w * z
    return np.array(
        [
            [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
            [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
            [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
        ],
        dtype=np.float32,
    )


def transform_points(points: np.ndarray, translation: Iterable[float], quaternion: Iterable[float]) -> np.ndarray:
    if points.size == 0:
        return points
    tx, ty, tz = [float(value) for value in translation]
    qx, qy, qz, qw = [float(value) for value in quaternion]
    rotation = quaternion_to_matrix(qx, qy, qz, qw)
    shifted = (rotation @ points.T).T
    shifted[:, 0] += tx
    shifted[:, 1] += ty
    shifted[:, 2] += tz
    return shifted


def pose_from_pose_stamped(msg) -> Pose2D:
    pose = msg.pose
    return Pose2D(
        x=float(pose.position.x),
        y=float(pose.position.y),
        z=float(pose.position.z),
        yaw=quaternion_to_yaw(
            float(pose.orientation.x),
            float(pose.orientation.y),
            float(pose.orientation.z),
            float(pose.orientation.w),
        ),
    )


def points_to_map(points_base: np.ndarray, pose: Pose2D) -> np.ndarray:
    if points_base.size == 0:
        return points_base
    cos_yaw = math.cos(pose.yaw)
    sin_yaw = math.sin(pose.yaw)
    world = points_base.copy()
    world_x = cos_yaw * points_base[:, 0] - sin_yaw * points_base[:, 1] + pose.x
    world_y = sin_yaw * points_base[:, 0] + cos_yaw * points_base[:, 1] + pose.y
    world[:, 0] = world_x
    world[:, 1] = world_y
    world[:, 2] = points_base[:, 2] + pose.z
    return world


def _mask_box(points: np.ndarray, enabled: bool, prefix: str, cfg: dict) -> np.ndarray:
    if not enabled or points.size == 0:
        return np.zeros(points.shape[0], dtype=bool)
    return (
        (points[:, 0] >= cfg[f"{prefix}_min_x"])
        & (points[:, 0] <= cfg[f"{prefix}_max_x"])
        & (points[:, 1] >= cfg[f"{prefix}_min_y"])
        & (points[:, 1] <= cfg[f"{prefix}_max_y"])
        & (points[:, 2] >= cfg[f"{prefix}_min_z"])
        & (points[:, 2] <= cfg[f"{prefix}_max_z"])
    )


def apply_prefilters(points: np.ndarray, cfg: dict) -> np.ndarray:
    if points.size == 0:
        return points
    range_xy = np.hypot(points[:, 0], points[:, 1])
    mask = (
        np.isfinite(points).all(axis=1)
        & (range_xy >= cfg["range_filter_min"])
        & (range_xy <= cfg["range_filter_max"])
        & (points[:, 2] >= cfg["height_filter_min_z"])
        & (points[:, 2] <= cfg["height_filter_max_z"])
    )
    if cfg["azimuth_filter_enabled"]:
        min_rad = math.radians(cfg["azimuth_filter_min_angle_deg"])
        max_rad = math.radians(cfg["azimuth_filter_max_angle_deg"])
        azimuth = np.arctan2(points[:, 1], points[:, 0])
        if min_rad <= max_rad:
            mask &= (azimuth >= min_rad) & (azimuth <= max_rad)
        else:
            mask &= (azimuth >= min_rad) | (azimuth <= max_rad)
    mask &= ~_mask_box(points, cfg["self_mask_enabled"], "self_mask", cfg)
    mask &= ~_mask_box(points, cfg["front_mask_enabled"], "front_mask", cfg)
    return points[mask]


def classify_points(points_base: np.ndarray, cfg: dict) -> dict[str, np.ndarray]:
    if points_base.size == 0:
        empty = np.empty((0, 3), dtype=np.float32)
        return {"ground": empty, "ramp": empty, "obstacle": empty}

    filtered = apply_prefilters(points_base, cfg)
    if filtered.size == 0:
        empty = np.empty((0, 3), dtype=np.float32)
        return {"ground": empty, "ramp": empty, "obstacle": empty}

    cell_size = cfg["terrain_cell_size"]
    x_min = cfg["terrain_x_min"]
    x_max = cfg["terrain_x_max"]
    y_min = cfg["terrain_y_min"]
    y_max = cfg["terrain_y_max"]
    cols = max(1, int(math.ceil((x_max - x_min) / cell_size)))
    rows = max(1, int(math.ceil((y_max - y_min) / cell_size)))

    cell_heights: list[list[float]] = [[] for _ in range(rows * cols)]
    point_cells: list[tuple[int, int, int]] = []
    for index, point in enumerate(filtered):
        if point[0] < x_min or point[0] >= x_max or point[1] < y_min or point[1] >= y_max:
            continue
        ix = int(math.floor((float(point[0]) - x_min) / cell_size))
        iy = int(math.floor((float(point[1]) - y_min) / cell_size))
        if ix < 0 or ix >= cols or iy < 0 or iy >= rows:
            continue
        cell_index = iy * cols + ix
        cell_heights[cell_index].append(float(point[2]))
        point_cells.append((index, ix, iy))

    if not point_cells:
        empty = np.empty((0, 3), dtype=np.float32)
        return {"ground": empty, "ramp": empty, "obstacle": empty}

    base_ground = np.full(rows * cols, np.nan, dtype=np.float32)
    min_points_per_cell = max(1, int(cfg["terrain_min_points_per_cell"]))
    for idx, heights in enumerate(cell_heights):
        if len(heights) < min_points_per_cell:
            continue
        ordered = np.array(sorted(heights), dtype=np.float32)
        quantile = float(np.clip(cfg["terrain_ground_quantile"], 0.0, 1.0))
        q_index = int(round((len(ordered) - 1) * quantile))
        base_ground[idx] = ordered[q_index]

    neighbor_radius = max(0, int(cfg["terrain_neighbor_radius"]))
    neighborhood_ground = np.full(rows * cols, np.nan, dtype=np.float32)
    for iy in range(rows):
        for ix in range(cols):
            values = []
            for dy in range(-neighbor_radius, neighbor_radius + 1):
                ny = iy + dy
                if ny < 0 or ny >= rows:
                    continue
                for dx in range(-neighbor_radius, neighbor_radius + 1):
                    nx = ix + dx
                    if nx < 0 or nx >= cols:
                        continue
                    value = base_ground[ny * cols + nx]
                    if np.isfinite(value):
                        values.append(value)
            if values:
                neighborhood_ground[iy * cols + ix] = min(values)

    ground_points: list[np.ndarray] = []
    ramp_points: list[np.ndarray] = []
    obstacle_points: list[np.ndarray] = []
    for index, ix, iy in point_cells:
        ground_z = neighborhood_ground[iy * cols + ix]
        if not np.isfinite(ground_z):
            continue
        point = filtered[index]
        rel_z = float(point[2] - ground_z)
        radial_range = max(1.0e-3, float(math.hypot(point[0], point[1])))
        slope_deg = math.degrees(math.atan2(max(rel_z, 0.0), radial_range))
        if cfg["class_ground_min_rel_z"] <= rel_z <= cfg["class_ground_max_rel_z"]:
            ground_points.append(point)
        elif (
            cfg["class_ramp_min_rel_z"] <= rel_z <= cfg["class_ramp_max_rel_z"]
            and slope_deg <= cfg["class_ramp_max_slope_deg"]
        ):
            ramp_points.append(point)
        elif cfg["class_obstacle_min_rel_z"] <= rel_z <= cfg["class_obstacle_max_rel_z"]:
            obstacle_points.append(point)

    def _stack(points_list: list[np.ndarray]) -> np.ndarray:
        if not points_list:
            return np.empty((0, 3), dtype=np.float32)
        return np.vstack(points_list).astype(np.float32, copy=False)

    return {
        "ground": _stack(ground_points),
        "ramp": _stack(ramp_points),
        "obstacle": _stack(obstacle_points),
    }


def _bresenham_line(r0: int, c0: int, r1: int, c1: int):
    dr = abs(r1 - r0)
    dc = abs(c1 - c0)
    sr = 1 if r0 < r1 else -1
    sc = 1 if c0 < c1 else -1
    err = dc - dr
    row = r0
    col = c0
    while True:
        yield row, col
        if row == r1 and col == c1:
            break
        e2 = 2 * err
        if e2 > -dr:
            err -= dr
            col += sc
        if e2 < dc:
            err += dc
            row += sr


def dilate_mask(mask: np.ndarray, iterations: int) -> np.ndarray:
    current = mask.astype(bool, copy=True)
    for _ in range(max(0, iterations)):
        padded = np.pad(current, 1, constant_values=False)
        next_mask = np.zeros_like(current)
        for dy in range(3):
            for dx in range(3):
                next_mask |= padded[dy : dy + current.shape[0], dx : dx + current.shape[1]]
        current = next_mask
    return current


def erode_mask(mask: np.ndarray, iterations: int) -> np.ndarray:
    current = mask.astype(bool, copy=True)
    for _ in range(max(0, iterations)):
        padded = np.pad(current, 1, constant_values=True)
        next_mask = np.ones_like(current)
        for dy in range(3):
            for dx in range(3):
                next_mask &= padded[dy : dy + current.shape[0], dx : dx + current.shape[1]]
        current = next_mask
    return current


def close_mask(mask: np.ndarray, iterations: int) -> np.ndarray:
    if iterations <= 0:
        return mask.astype(bool, copy=True)
    return erode_mask(dilate_mask(mask, iterations), iterations)


def remove_sparse_pixels(mask: np.ndarray, min_neighbors: int) -> np.ndarray:
    if min_neighbors <= 0:
        return mask.astype(bool, copy=True)
    current = mask.astype(bool, copy=True)
    padded = np.pad(current, 1, constant_values=False)
    neighbors = np.zeros_like(current, dtype=np.int32)
    for dy in range(3):
        for dx in range(3):
            if dy == 1 and dx == 1:
                continue
            neighbors += padded[dy : dy + current.shape[0], dx : dx + current.shape[1]].astype(np.int32)
    current &= neighbors >= min_neighbors
    return current


class OccupancyAccumulator:
    def __init__(self, cfg: dict):
        self.cfg = dict(cfg)
        self.resolution = float(cfg["resolution"])
        self.width = max(1, int(round(float(cfg["width_m"]) / self.resolution)))
        self.height = max(1, int(round(float(cfg["height_m"]) / self.resolution)))
        self.origin_x = float(cfg["origin_x"])
        self.origin_y = float(cfg["origin_y"])
        self.log_odds = np.zeros((self.height, self.width), dtype=np.float32)
        self.ground_hits = np.zeros((self.height, self.width), dtype=np.uint32)
        self.ramp_hits = np.zeros((self.height, self.width), dtype=np.uint32)
        self.obstacle_hits = np.zeros((self.height, self.width), dtype=np.uint32)
        self.free_hits = np.zeros((self.height, self.width), dtype=np.uint32)
        self.scans_processed = 0

    def reset(self) -> None:
        self.log_odds.fill(0.0)
        self.ground_hits.fill(0)
        self.ramp_hits.fill(0)
        self.obstacle_hits.fill(0)
        self.free_hits.fill(0)
        self.scans_processed = 0

    def _point_to_cell(self, x: float, y: float) -> tuple[int, int] | None:
        col = int(math.floor((x - self.origin_x) / self.resolution))
        row = int(math.floor((y - self.origin_y) / self.resolution))
        if row < 0 or row >= self.height or col < 0 or col >= self.width:
            return None
        return row, col

    def integrate_scan(self, points_base: np.ndarray, pose: Pose2D, sensor_xyz: Iterable[float]) -> dict[str, int]:
        semantic = classify_points(points_base, self.cfg)
        sensor_x, sensor_y, _ = [float(value) for value in sensor_xyz]
        cos_yaw = math.cos(pose.yaw)
        sin_yaw = math.sin(pose.yaw)
        origin_x = pose.x + cos_yaw * sensor_x - sin_yaw * sensor_y
        origin_y = pose.y + sin_yaw * sensor_x + cos_yaw * sensor_y
        origin_cell = self._point_to_cell(origin_x, origin_y)
        if origin_cell is None:
            return {"ground": 0, "ramp": 0, "obstacle": 0, "free": 0}

        free_ids: set[int] = set()
        ground_ids: set[int] = set()
        ramp_ids: set[int] = set()
        obstacle_ids: set[int] = set()

        for class_name, class_points in semantic.items():
            if class_points.size == 0:
                continue
            world_points = points_to_map(class_points, pose)
            for point in world_points:
                cell = self._point_to_cell(float(point[0]), float(point[1]))
                if cell is None:
                    continue
                end_row, end_col = cell
                for trace_row, trace_col in _bresenham_line(origin_cell[0], origin_cell[1], end_row, end_col):
                    flat_idx = trace_row * self.width + trace_col
                    if trace_row == end_row and trace_col == end_col:
                        break
                    free_ids.add(flat_idx)
                end_idx = end_row * self.width + end_col
                if class_name == "obstacle":
                    obstacle_ids.add(end_idx)
                elif class_name == "ramp":
                    ramp_ids.add(end_idx)
                    free_ids.add(end_idx)
                else:
                    ground_ids.add(end_idx)
                    free_ids.add(end_idx)

        flat = self.log_odds.ravel()
        if free_ids:
            free_arr = np.fromiter(free_ids, dtype=np.int64)
            flat[free_arr] = np.maximum(flat[free_arr] - float(self.cfg["miss_log"]), float(self.cfg["min_log"]))
            self.free_hits.ravel()[free_arr] += 1
        if obstacle_ids:
            obstacle_arr = np.fromiter(obstacle_ids, dtype=np.int64)
            flat[obstacle_arr] = np.minimum(
                flat[obstacle_arr] + float(self.cfg["hit_log"]),
                float(self.cfg["max_log"]),
            )
            self.obstacle_hits.ravel()[obstacle_arr] += 1
        if ground_ids:
            ground_arr = np.fromiter(ground_ids, dtype=np.int64)
            self.ground_hits.ravel()[ground_arr] += 1
        if ramp_ids:
            ramp_arr = np.fromiter(ramp_ids, dtype=np.int64)
            self.ramp_hits.ravel()[ramp_arr] += 1
        self.scans_processed += 1
        return {
            "ground": len(ground_ids),
            "ramp": len(ramp_ids),
            "obstacle": len(obstacle_ids),
            "free": len(free_ids),
        }

    def occupancy_data(self) -> np.ndarray:
        occupied = self.log_odds >= float(self.cfg["occupied_threshold"])
        occupied = dilate_mask(occupied, int(self.cfg["post_dilate"]))
        occupied = close_mask(occupied, int(self.cfg["post_close"]))
        occupied = remove_sparse_pixels(occupied, int(self.cfg["speckle_neighbors"]))
        free = self.log_odds <= float(self.cfg["free_threshold"])
        free_min_hits = max(0, int(self.cfg.get("free_min_hits", 0)))
        if free_min_hits > 0:
            free |= self.free_hits >= free_min_hits
            free |= self.ground_hits >= free_min_hits
            free |= self.ramp_hits >= free_min_hits
        free = close_mask(free, int(self.cfg.get("free_post_close", 0)))
        free = remove_sparse_pixels(free, int(self.cfg.get("free_speckle_neighbors", 0)))
        free &= ~occupied
        data = np.full(self.height * self.width, -1, dtype=np.int8)
        data[free.ravel()] = 0
        data[occupied.ravel()] = 100
        return data

    def occupancy_pixels(self, data: np.ndarray | None = None) -> list[int]:
        if data is None:
            data = self.occupancy_data()
        pixels = [UNKNOWN_PIXEL] * (self.width * self.height)
        for grid_y in range(self.height):
            src_row = grid_y * self.width
            dst_row = (self.height - 1 - grid_y) * self.width
            for grid_x in range(self.width):
                src_idx = src_row + grid_x
                dst_idx = dst_row + grid_x
                value = int(data[src_idx])
                if value >= 100:
                    pixels[dst_idx] = OCCUPIED_PIXEL
                elif value == 0:
                    pixels[dst_idx] = FREE_PIXEL
        return pixels

    def write_assets(
        self,
        output_root: Path,
        map_frame_id: str,
        source_summary: dict[str, object],
    ) -> dict[str, Path]:
        output_root.mkdir(parents=True, exist_ok=True)
        nav_dir = output_root / "nav"
        localizer_dir = output_root / "localizer"
        reports_dir = output_root / "reports"
        intermediate_dir = output_root / "intermediate"
        for directory in (nav_dir, localizer_dir, reports_dir, intermediate_dir):
            directory.mkdir(parents=True, exist_ok=True)

        occupancy_data = self.occupancy_data()
        pixels = self.occupancy_pixels(occupancy_data)

        nav_pgm = nav_dir / "nav_map.pgm"
        nav_yaml = nav_dir / "nav_map.yaml"
        localizer_png = localizer_dir / "localizer_map.png"
        localizer_yaml = localizer_dir / "localizer_params.yaml"
        asset_report = reports_dir / "asset_report.json"
        intermediate_npz = intermediate_dir / "occupancy_layers.npz"

        save_pgm(nav_pgm, self.width, self.height, pixels)
        save_png(localizer_png, self.width, self.height, pixels)
        save_map_yaml(nav_yaml, nav_pgm.name, self.resolution, self.origin_x, self.origin_y)
        save_map_yaml(localizer_yaml, localizer_png.name, self.resolution, self.origin_x, self.origin_y)

        np.savez_compressed(
            intermediate_npz,
            log_odds=self.log_odds,
            ground_hits=self.ground_hits,
            ramp_hits=self.ramp_hits,
            obstacle_hits=self.obstacle_hits,
            free_hits=self.free_hits,
        )

        report_payload = {
            "producer": "robot_occupancy_builder",
            "map_frame_id": map_frame_id,
            "resolution": self.resolution,
            "origin": [self.origin_x, self.origin_y, 0.0],
            "width": self.width,
            "height": self.height,
            "scans_processed": self.scans_processed,
            "occupied_cells": int(np.count_nonzero(occupancy_data == 100)),
            "free_cells": int(np.count_nonzero(occupancy_data == 0)),
            "unknown_cells": int(np.count_nonzero(occupancy_data == -1)),
            "source_summary": source_summary,
            "nav_map": str(nav_yaml),
            "localizer_map": str(localizer_yaml),
            "intermediate_layers": str(intermediate_npz),
        }
        asset_report.write_text(json.dumps(report_payload, indent=2), encoding="utf-8")
        return {
            "nav_yaml": nav_yaml,
            "nav_pgm": nav_pgm,
            "localizer_png": localizer_png,
            "localizer_yaml": localizer_yaml,
            "asset_report": asset_report,
            "intermediate_npz": intermediate_npz,
        }


def save_pgm(path: Path, width: int, height: int, pixels: Iterable[int]) -> None:
    with path.open("wb") as handle:
        handle.write(f"P5\n{width} {height}\n255\n".encode("ascii"))
        handle.write(bytes(int(pixel) for pixel in pixels))


def _png_chunk(tag: bytes, payload: bytes) -> bytes:
    return (
        struct.pack(">I", len(payload))
        + tag
        + payload
        + struct.pack(">I", binascii.crc32(tag + payload) & 0xFFFFFFFF)
    )


def save_png(path: Path, width: int, height: int, pixels: Iterable[int]) -> None:
    flat_pixels = bytes(int(pixel) for pixel in pixels)
    rows = []
    for row in range(height):
        start = row * width
        rows.append(b"\x00" + flat_pixels[start : start + width])
    raw = b"".join(rows)
    compressed = zlib.compress(raw, level=9)
    png = b"".join(
        [
            b"\x89PNG\r\n\x1a\n",
            _png_chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 8, 0, 0, 0, 0)),
            _png_chunk(b"IDAT", compressed),
            _png_chunk(b"IEND", b""),
        ]
    )
    path.write_bytes(png)


def save_map_yaml(path: Path, image_name: str, resolution: float, origin_x: float, origin_y: float) -> None:
    path.write_text(
        "\n".join(
            [
                f"image: {image_name}",
                f"resolution: {resolution:.6f}",
                f"origin: [{origin_x:.6f}, {origin_y:.6f}, 0.0]",
                "negate: 0",
                "occupied_thresh: 0.65",
                "free_thresh: 0.196",
                "mode: trinary",
                "",
            ]
        ),
        encoding="utf-8",
    )


def load_trajectory_csv(path: Path) -> tuple[list[int], list[Pose2D]]:
    stamps: list[int] = []
    poses: list[Pose2D] = []
    with path.open("r", encoding="utf-8", newline="") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            stamp_value = (
                row.get("timestamp_ns")
                or row.get("time_ns")
                or row.get("stamp_ns")
                or row.get("t")
                or row.get("timestamp")
                or ""
            ).strip()
            if not stamp_value:
                continue
            raw_stamp = float(stamp_value)
            stamp_ns = int(raw_stamp if raw_stamp > 1.0e12 else raw_stamp * 1.0e9)
            yaw_value = (row.get("yaw") or "").strip()
            if yaw_value:
                yaw = float(yaw_value)
            else:
                yaw = quaternion_to_yaw(
                    float((row.get("qx") or "0.0").strip()),
                    float((row.get("qy") or "0.0").strip()),
                    float((row.get("qz") or "0.0").strip()),
                    float((row.get("qw") or "1.0").strip()),
                )
            poses.append(
                Pose2D(
                    x=float((row.get("x") or "0.0").strip()),
                    y=float((row.get("y") or "0.0").strip()),
                    z=float((row.get("z") or "0.0").strip()),
                    yaw=yaw,
                )
            )
            stamps.append(stamp_ns)
    if not stamps:
        raise RuntimeError(f"No trajectory rows were loaded from {path}")
    ordered = sorted(zip(stamps, poses), key=lambda item: item[0])
    return [item[0] for item in ordered], [item[1] for item in ordered]


def nearest_pose(stamps_ns: list[int], poses: list[Pose2D], stamp_ns: int, tolerance_ns: int) -> Optional[Pose2D]:
    index = bisect_left(stamps_ns, stamp_ns)
    candidates = []
    if index < len(stamps_ns):
        candidates.append(index)
    if index > 0:
        candidates.append(index - 1)
    if not candidates:
        return None
    best_index = min(candidates, key=lambda item: abs(stamps_ns[item] - stamp_ns))
    if abs(stamps_ns[best_index] - stamp_ns) > tolerance_ns:
        return None
    return poses[best_index]


def iter_bag_pointclouds(bag_path: Path, pointcloud_topic: str, storage_id: str):
    try:
        import rosbag2_py  # type: ignore
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
    except ImportError as exc:  # pragma: no cover - runtime dependency
        raise RuntimeError("rosbag2_py and rosidl_runtime_py are required for release_rebuild") from exc

    storage_options = rosbag2_py.StorageOptions(uri=str(bag_path), storage_id=storage_id)
    converter_options = rosbag2_py.ConverterOptions("", "")
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)
    topic_types = {topic.name: topic.type for topic in reader.get_all_topics_and_types()}
    message_type = topic_types.get(pointcloud_topic)
    if message_type is None:
        raise RuntimeError(f"Pointcloud topic {pointcloud_topic} was not found in rosbag {bag_path}")
    message_class = get_message(message_type)
    while reader.has_next():
        topic_name, data, timestamp = reader.read_next()
        if topic_name != pointcloud_topic:
            continue
        yield int(timestamp), deserialize_message(data, message_class)


def run_release_rebuild(config: dict) -> dict[str, object]:
    bag_path = Path(str(config["raw_bag_path"])).resolve()
    trajectory_csv = Path(str(config["optimized_trajectory_csv"])).resolve()
    output_root = Path(str(config["output_root"])).resolve()
    if not bag_path.exists():
        raise RuntimeError(f"Raw bag path does not exist: {bag_path}")
    if not trajectory_csv.exists():
        raise RuntimeError(f"Optimized trajectory csv does not exist: {trajectory_csv}")

    trajectory_stamps, trajectory_poses = load_trajectory_csv(trajectory_csv)
    tolerance_ns = int(float(config["pose_match_tolerance_ms"]) * 1.0e6)
    sensor_xyz = [float(value) for value in config["sensor_xyz"]]
    sensor_rpy = [float(value) for value in config["sensor_rpy"]]
    sensor_quaternion = euler_to_quaternion(sensor_rpy[0], sensor_rpy[1], sensor_rpy[2])
    builder = OccupancyAccumulator(config)

    scans_seen = 0
    scans_used = 0
    for stamp_ns, cloud in iter_bag_pointclouds(
        bag_path,
        str(config["pointcloud_topic"]),
        str(config["bag_storage_id"]),
    ):
        scans_seen += 1
        pose = nearest_pose(trajectory_stamps, trajectory_poses, stamp_ns, tolerance_ns)
        if pose is None:
            continue
        points = parse_pointcloud_xyz(cloud)
        if points.size == 0:
            continue
        points_base = transform_points(points, sensor_xyz, sensor_quaternion)
        builder.integrate_scan(points_base, pose, sensor_xyz)
        scans_used += 1

    assets = builder.write_assets(
        output_root=output_root,
        map_frame_id=str(config["map_frame_id"]),
        source_summary={
            "mode": "release_rebuild",
            "raw_bag_path": str(bag_path),
            "optimized_trajectory_csv": str(trajectory_csv),
            "pointcloud_topic": str(config["pointcloud_topic"]),
            "bag_storage_id": str(config["bag_storage_id"]),
            "scans_seen": scans_seen,
            "scans_used": scans_used,
        },
    )
    return {
        "assets": {name: str(path) for name, path in assets.items()},
        "scans_seen": scans_seen,
        "scans_used": scans_used,
    }
