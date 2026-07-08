#!/usr/bin/env python3
"""Capture scan-to-map alignment diagnostics around an Isaac relocalization pose."""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
from pathlib import Path
import statistics
import time
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener

try:
    from isaac_ros_pointcloud_interfaces.msg import FlatScan
except Exception:  # pragma: no cover - interface is present only on the robot image.
    FlatScan = None  # type: ignore[assignment]

try:
    from PIL import Image, ImageDraw
except Exception as exc:  # pragma: no cover - exercised on robot image only.
    raise SystemExit(f"Pillow is required for scan-map alignment diagnostics: {exc}") from exc


Point = Tuple[float, float]


def norm_angle(value: float) -> float:
    return math.atan2(math.sin(value), math.cos(value))


def yaw_from_quat(q: Any) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def yaw_from_quat_dict(q: Dict[str, float]) -> float:
    return math.atan2(
        2.0 * (q.get("w", 1.0) * q.get("z", 0.0) + q.get("x", 0.0) * q.get("y", 0.0)),
        1.0 - 2.0 * (q.get("y", 0.0) ** 2 + q.get("z", 0.0) ** 2),
    )


def transform_to_pose_dict(transform_msg: Any) -> Dict[str, float]:
    t = transform_msg.transform.translation
    q = transform_msg.transform.rotation
    yaw = yaw_from_quat(q)
    return {
        "x": float(t.x),
        "y": float(t.y),
        "z": float(t.z),
        "qx": float(q.x),
        "qy": float(q.y),
        "qz": float(q.z),
        "qw": float(q.w),
        "yaw_rad": float(yaw),
        "yaw_deg": math.degrees(yaw),
    }


def parse_csv_list(value: str) -> List[str]:
    return [part.strip() for part in value.split(",") if part.strip()]


def parse_origin(value: Any) -> Tuple[float, float, float]:
    if isinstance(value, (list, tuple)) and len(value) >= 3:
        return float(value[0]), float(value[1]), float(value[2])
    text = str(value).strip().strip("[]")
    parts = [part.strip() for part in text.split(",") if part.strip()]
    if len(parts) < 3:
        raise ValueError(f"invalid map origin: {value!r}")
    return float(parts[0]), float(parts[1]), float(parts[2])


def read_simple_yaml(path: Path) -> Dict[str, Any]:
    values: Dict[str, Any] = {}
    for raw_line in path.read_text(encoding="utf-8").splitlines():
        line = raw_line.split("#", 1)[0].strip()
        if not line or ":" not in line:
            continue
        key, value = line.split(":", 1)
        values[key.strip()] = value.strip().strip('"').strip("'")
    return values


def resolve_image_path(yaml_path: Path, values: Dict[str, Any]) -> Path:
    image_value = values.get("image")
    if not image_value:
        raise ValueError(f"map yaml has no image field: {yaml_path}")
    image_path = Path(str(image_value))
    if not image_path.is_absolute():
        image_path = (yaml_path.parent / image_path).resolve()
    if not image_path.is_file():
        raise FileNotFoundError(f"map image not found: {image_path}")
    return image_path


def project_root_from_script() -> Path:
    return Path(__file__).resolve().parents[4]


def candidate_map_yamls(project_root: Path) -> Iterable[Path]:
    env_candidates = [
        os.environ.get("NAV2_LOCALIZER_MAP_YAML"),
        os.environ.get("NAV2_MAP_YAML"),
        os.environ.get("NJRH_LOCALIZER_MAP_YAML"),
    ]
    for item in env_candidates:
        if item:
            yield Path(item)

    context_file = Path(os.environ.get("NJRH_RUNTIME_MAP_CONTEXT_FILE", "/tmp/njrh_runtime_map_context.json"))
    if context_file.is_file():
        try:
            context = json.loads(context_file.read_text(encoding="utf-8"))
        except Exception:
            context = {}
        building = str(context.get("building_id") or "").strip()
        floor = str(context.get("floor_id") or "").strip()
        map_id = str(context.get("map_id") or "").strip()
        if building and floor:
            floor_root = project_root / "maps_release" / building / floor
            for leaf in (
                Path("current/localizer/localizer_params.yaml"),
                Path("current/nav/nav_map.yaml"),
                Path(map_id) / "localizer" / "localizer_params.yaml" if map_id else None,
                Path(map_id) / "nav" / "nav_map.yaml" if map_id else None,
                Path("localizer/localizer_params.yaml"),
                Path("nav/nav_map.yaml"),
            ):
                if leaf is not None:
                    yield floor_root / leaf


def resolve_map_yaml(explicit: str) -> Path:
    if explicit:
        path = Path(explicit)
        if not path.is_absolute():
            path = (Path.cwd() / path).resolve()
        if not path.is_file():
            raise FileNotFoundError(f"map yaml not found: {path}")
        return path

    project_root = project_root_from_script()
    tried: List[str] = []
    for candidate in candidate_map_yamls(project_root):
        tried.append(str(candidate))
        if candidate.is_file():
            return candidate.resolve()
    raise FileNotFoundError("could not resolve current map yaml; tried: " + ", ".join(tried))


def load_after_pose(path: Path) -> Dict[str, float]:
    data = json.loads(path.read_text(encoding="utf-8"))
    pose = ((data.get("tf") or {}).get("map_base_link") or {})
    if not all(key in pose for key in ("x", "y", "yaw_rad")):
        raise ValueError(f"{path} has no tf.map_base_link pose")
    return {
        "x": float(pose["x"]),
        "y": float(pose["y"]),
        "yaw_rad": float(pose["yaw_rad"]),
        "yaw_deg": math.degrees(float(pose["yaw_rad"])),
    }


def load_map(yaml_path: Path) -> Dict[str, Any]:
    values = read_simple_yaml(yaml_path)
    image_path = resolve_image_path(yaml_path, values)
    image = Image.open(image_path).convert("L")
    resolution = float(values.get("resolution", 0.05))
    origin = parse_origin(values.get("origin", "[0.0, 0.0, 0.0]"))
    occupied_thresh = float(values.get("occupied_thresh", 0.65))
    negate = int(float(values.get("negate", 0)))
    if negate:
        occupied_pixel_threshold = int(round(255.0 * occupied_thresh))
        occupied = [pixel >= occupied_pixel_threshold for pixel in image.getdata()]
    else:
        occupied_pixel_threshold = int(round(255.0 * (1.0 - occupied_thresh)))
        occupied = [pixel <= occupied_pixel_threshold for pixel in image.getdata()]
    return {
        "yaml_path": str(yaml_path),
        "image_path": str(image_path),
        "image": image,
        "width": image.width,
        "height": image.height,
        "resolution": resolution,
        "origin": origin,
        "occupied": occupied,
        "occupied_pixel_threshold": occupied_pixel_threshold,
    }


def build_distance_field(
    occupied: Sequence[bool],
    width: int,
    height: int,
    resolution: float,
    cap_m: float,
) -> List[float]:
    cap_cells = max(int(math.ceil(cap_m / resolution)), 1)
    inf = float(cap_cells + 2)
    diag = math.sqrt(2.0)
    dist = [0.0 if occ else inf for occ in occupied]

    for y in range(height):
        row = y * width
        prev_row = row - width
        for x in range(width):
            idx = row + x
            if dist[idx] == 0.0:
                continue
            best = dist[idx]
            if x > 0:
                best = min(best, dist[idx - 1] + 1.0)
            if y > 0:
                best = min(best, dist[prev_row + x] + 1.0)
                if x > 0:
                    best = min(best, dist[prev_row + x - 1] + diag)
                if x + 1 < width:
                    best = min(best, dist[prev_row + x + 1] + diag)
            dist[idx] = best

    for y in range(height - 1, -1, -1):
        row = y * width
        next_row = row + width
        for x in range(width - 1, -1, -1):
            idx = row + x
            if dist[idx] == 0.0:
                continue
            best = dist[idx]
            if x + 1 < width:
                best = min(best, dist[idx + 1] + 1.0)
            if y + 1 < height:
                best = min(best, dist[next_row + x] + 1.0)
                if x > 0:
                    best = min(best, dist[next_row + x - 1] + diag)
                if x + 1 < width:
                    best = min(best, dist[next_row + x + 1] + diag)
            dist[idx] = best

    return [min(value, float(cap_cells)) * resolution for value in dist]


def scan_to_points(scan: Any, max_range_m: float) -> List[Point]:
    points: List[Point] = []
    range_max = min(float(scan.range_max), max_range_m)
    range_min = max(float(scan.range_min), 0.01)
    if hasattr(scan, "angles"):
        for angle_value, range_value in zip(scan.angles, scan.ranges):
            r = float(range_value)
            if math.isfinite(r) and range_min <= r <= range_max:
                angle = float(angle_value)
                points.append((r * math.cos(angle), r * math.sin(angle)))
        return points

    angle = float(scan.angle_min)
    for value in scan.ranges:
        r = float(value)
        if math.isfinite(r) and range_min <= r <= range_max:
            points.append((r * math.cos(angle), r * math.sin(angle)))
        angle += float(scan.angle_increment)
    return points


def transform_points(points: Sequence[Point], pose: Dict[str, float]) -> List[Point]:
    c = math.cos(float(pose["yaw_rad"]))
    s = math.sin(float(pose["yaw_rad"]))
    tx = float(pose["x"])
    ty = float(pose["y"])
    return [(tx + c * x - s * y, ty + s * x + c * y) for x, y in points]


def transform_points_with_pose(points: Sequence[Point], x: float, y: float, yaw: float) -> List[Point]:
    c = math.cos(yaw)
    s = math.sin(yaw)
    return [(x + c * px - s * py, y + s * px + c * py) for px, py in points]


def world_to_pixel(x: float, y: float, map_info: Dict[str, Any]) -> Optional[Tuple[int, int]]:
    origin_x, origin_y, _ = map_info["origin"]
    resolution = float(map_info["resolution"])
    col = int(round((x - origin_x) / resolution))
    row = int(round((float(map_info["height"]) - 1.0) - ((y - origin_y) / resolution)))
    if 0 <= col < int(map_info["width"]) and 0 <= row < int(map_info["height"]):
        return col, row
    return None


def pixel_distance(x: float, y: float, dist: Sequence[float], map_info: Dict[str, Any]) -> Optional[float]:
    pixel = world_to_pixel(x, y, map_info)
    if pixel is None:
        return None
    col, row = pixel
    return float(dist[row * int(map_info["width"]) + col])


def downsample(points: Sequence[Point], max_points: int) -> List[Point]:
    if len(points) <= max_points:
        return list(points)
    step = max(int(math.ceil(len(points) / max_points)), 1)
    return list(points[::step])[:max_points]


def score_points(points_map: Sequence[Point], dist: Sequence[float], map_info: Dict[str, Any]) -> Dict[str, Any]:
    values = []
    out_of_bounds = 0
    for x, y in points_map:
        value = pixel_distance(x, y, dist, map_info)
        if value is None:
            out_of_bounds += 1
        else:
            values.append(value)
    if not values:
        return {
            "points_used": 0,
            "points_out_of_bounds": out_of_bounds,
            "mean_distance_m": None,
            "median_distance_m": None,
            "p90_distance_m": None,
            "p95_distance_m": None,
        }
    ordered = sorted(values)

    def percentile(frac: float) -> float:
        index = min(max(int(round(frac * (len(ordered) - 1))), 0), len(ordered) - 1)
        return ordered[index]

    return {
        "points_used": len(values),
        "points_out_of_bounds": out_of_bounds,
        "mean_distance_m": statistics.fmean(values),
        "median_distance_m": percentile(0.50),
        "p90_distance_m": percentile(0.90),
        "p95_distance_m": percentile(0.95),
    }


def brute_force_pose_score(
    points_base: Sequence[Point],
    after_pose: Dict[str, float],
    dist: Sequence[float],
    map_info: Dict[str, Any],
    xy_window_m: float,
    xy_step_m: float,
    yaw_window_deg: float,
    yaw_step_deg: float,
) -> Dict[str, Any]:
    xy_steps = range(-int(round(xy_window_m / xy_step_m)), int(round(xy_window_m / xy_step_m)) + 1)
    yaw_steps = range(-int(round(yaw_window_deg / yaw_step_deg)), int(round(yaw_window_deg / yaw_step_deg)) + 1)
    best: Optional[Dict[str, Any]] = None
    tested = 0
    for ix in xy_steps:
        dx = ix * xy_step_m
        for iy in xy_steps:
            dy = iy * xy_step_m
            for iyaw in yaw_steps:
                dyaw = math.radians(iyaw * yaw_step_deg)
                candidate_points = transform_points_with_pose(
                    points_base,
                    float(after_pose["x"]) + dx,
                    float(after_pose["y"]) + dy,
                    norm_angle(float(after_pose["yaw_rad"]) + dyaw),
                )
                score = score_points(candidate_points, dist, map_info)
                tested += 1
                mean = score.get("mean_distance_m")
                if mean is None:
                    continue
                if best is None or float(mean) < float(best["score"]["mean_distance_m"]):
                    best = {
                        "dx_map_m": dx,
                        "dy_map_m": dy,
                        "dyaw_rad": dyaw,
                        "dyaw_deg": math.degrees(dyaw),
                        "score": score,
                    }

    if best is None:
        return {"tested_candidates": tested, "best": None}

    c = math.cos(float(after_pose["yaw_rad"]))
    s = math.sin(float(after_pose["yaw_rad"]))
    best["forward_m_in_after_frame"] = best["dx_map_m"] * c + best["dy_map_m"] * s
    best["left_m_in_after_frame"] = -best["dx_map_m"] * s + best["dy_map_m"] * c
    return {"tested_candidates": tested, "best": best}


class AlignmentNode(Node):
    def __init__(
        self,
        scan_topics: Sequence[str],
        pose_topics: Sequence[str],
        flatscan_topics: Sequence[str],
    ) -> None:
        super().__init__("diagnose_isaac_scan_map_alignment")
        scan_qos = QoSProfile(depth=20)
        scan_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        pose_qos = QoSProfile(depth=20)
        self.scans: Dict[str, Any] = {}
        self.scan_types: Dict[str, str] = {}
        flatscan_topic_set = set(flatscan_topics)
        self.poses: Dict[str, PoseWithCovarianceStamped] = {}
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self._subs = []
        for topic in scan_topics:
            if topic in flatscan_topic_set:
                if FlatScan is None:
                    self.scan_types[topic] = "missing_isaac_ros_pointcloud_interfaces/msg/FlatScan"
                    continue
                msg_type = FlatScan
                type_label = "isaac_ros_pointcloud_interfaces/msg/FlatScan"
            else:
                msg_type = LaserScan
                type_label = "sensor_msgs/msg/LaserScan"
            self.scan_types[topic] = type_label
            self._subs.append(
                self.create_subscription(
                    msg_type,
                    topic,
                    lambda msg, topic_name=topic: self.scans.__setitem__(topic_name, msg),
                    scan_qos,
                )
            )
        for topic in pose_topics:
            self._subs.append(
                self.create_subscription(
                    PoseWithCovarianceStamped,
                    topic,
                    lambda msg, topic_name=topic: self.poses.__setitem__(topic_name, msg),
                    pose_qos,
                )
            )

    def collect(self, timeout_sec: float, min_scans: int, wait_full_window: bool) -> None:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if not wait_full_window and len(self.scans) >= min_scans:
                break

    def lookup_transform_pose(self, target: str, source: str, timeout_sec: float = 2.0) -> Optional[Dict[str, float]]:
        deadline = time.monotonic() + timeout_sec
        last_error = ""
        while time.monotonic() < deadline and rclpy.ok():
            try:
                tf_msg = self.tf_buffer.lookup_transform(
                    target,
                    source,
                    rclpy.time.Time(),
                    timeout=Duration(seconds=0.2),
                )
                return transform_to_pose_dict(tf_msg)
            except TransformException as exc:
                last_error = str(exc)
                rclpy.spin_once(self, timeout_sec=0.05)
        return {"error": last_error}

    def ros_stamp_age_sec(self, msg: Any) -> Optional[float]:
        stamp = getattr(getattr(msg, "header", None), "stamp", None)
        if stamp is None:
            return None
        stamp_ns = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
        now_ns = int(self.get_clock().now().nanoseconds)
        if stamp_ns <= 0:
            return None
        return (now_ns - stamp_ns) / 1_000_000_000.0


def write_points_csv(path: Path, points_scan: Sequence[Point], points_base: Sequence[Point], points_map: Sequence[Point]) -> None:
    with path.open("w", encoding="utf-8", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["scan_x_m", "scan_y_m", "base_x_m", "base_y_m", "map_x_m", "map_y_m"])
        for scan_pt, base_pt, map_pt in zip(points_scan, points_base, points_map):
            writer.writerow(
                [
                    f"{scan_pt[0]:.6f}",
                    f"{scan_pt[1]:.6f}",
                    f"{base_pt[0]:.6f}",
                    f"{base_pt[1]:.6f}",
                    f"{map_pt[0]:.6f}",
                    f"{map_pt[1]:.6f}",
                ]
            )


def draw_overlay(path: Path, map_info: Dict[str, Any], overlay_points: Dict[str, Sequence[Point]], after_pose: Dict[str, float]) -> None:
    image = map_info["image"].convert("RGB")
    draw = ImageDraw.Draw(image)
    colors = {
        "/flatscan": (255, 0, 0),
        "/scan_amcl": (0, 120, 255),
        "/scan": (0, 190, 80),
        "/flatscan_localization": (255, 150, 0),
    }
    for topic, points in overlay_points.items():
        color = colors.get(topic, (255, 0, 255))
        for x, y in points:
            pixel = world_to_pixel(x, y, map_info)
            if pixel is None:
                continue
            col, row = pixel
            draw.rectangle((col - 1, row - 1, col + 1, row + 1), fill=color)

    base_pixel = world_to_pixel(float(after_pose["x"]), float(after_pose["y"]), map_info)
    if base_pixel is not None:
        bx, by = base_pixel
        heading_len = 0.5
        hx = float(after_pose["x"]) + heading_len * math.cos(float(after_pose["yaw_rad"]))
        hy = float(after_pose["y"]) + heading_len * math.sin(float(after_pose["yaw_rad"]))
        head_pixel = world_to_pixel(hx, hy, map_info)
        draw.ellipse((bx - 5, by - 5, bx + 5, by + 5), outline=(255, 255, 0), width=2)
        if head_pixel is not None:
            draw.line((bx, by, head_pixel[0], head_pixel[1]), fill=(255, 255, 0), width=3)

    image.save(path)


def pose_msg_to_dict(msg: PoseWithCovarianceStamped, node: AlignmentNode) -> Dict[str, Any]:
    pose = msg.pose.pose
    q = {
        "x": float(pose.orientation.x),
        "y": float(pose.orientation.y),
        "z": float(pose.orientation.z),
        "w": float(pose.orientation.w),
    }
    yaw = yaw_from_quat_dict(q)
    return {
        "frame_id": msg.header.frame_id,
        "stamp_sec": int(msg.header.stamp.sec),
        "stamp_nanosec": int(msg.header.stamp.nanosec),
        "age_sec": node.ros_stamp_age_sec(msg),
        "x": float(pose.position.x),
        "y": float(pose.position.y),
        "z": float(pose.position.z),
        "yaw_rad": yaw,
        "yaw_deg": math.degrees(yaw),
    }


def write_summary(path: Path, metrics: Dict[str, Any]) -> None:
    with path.open("w", encoding="utf-8") as f:
        f.write("# Scan Map Alignment Diagnostics\n\n")
        f.write("This report is read-only. It scores current LaserScan/FlatScan endpoints against the active occupancy map near the relocalized `map->base_link` pose.\n\n")
        f.write("## Inputs\n\n")
        f.write(f"- map_yaml: `{metrics.get('map_yaml')}`\n")
        f.write(f"- map_image: `{metrics.get('map_image')}`\n")
        f.write(f"- after_pose_x_m: `{metrics['after_pose']['x']:.6f}`\n")
        f.write(f"- after_pose_y_m: `{metrics['after_pose']['y']:.6f}`\n")
        f.write(f"- after_pose_yaw_deg: `{metrics['after_pose']['yaw_deg']:.3f}`\n")
        f.write("\n## Topic Scores\n\n")
        f.write("| topic | type | frame | points | mean_m | median_m | p90_m | best_dx_m | best_dy_m | best_dyaw_deg | best_mean_m |\n")
        f.write("|---|---|---|---:|---:|---:|---:|---:|---:|---:|---:|\n")
        for topic, item in metrics.get("scan_topics", {}).items():
            if not item.get("available"):
                f.write(f"| `{topic}` | `{item.get('type', '')}` | missing | 0 |  |  |  |  |  |  |  |\n")
                continue
            score = item.get("score_at_after_pose") or {}
            best = ((item.get("bruteforce") or {}).get("best") or {})
            best_score = best.get("score") or {}
            f.write(
                f"| `{topic}` | `{item.get('type', '')}` | `{item.get('frame_id', '')}` | {score.get('points_used', 0)} | "
                f"{_fmt(score.get('mean_distance_m'))} | {_fmt(score.get('median_distance_m'))} | "
                f"{_fmt(score.get('p90_distance_m'))} | {_fmt(best.get('dx_map_m'))} | "
                f"{_fmt(best.get('dy_map_m'))} | {_fmt(best.get('dyaw_deg'))} | "
                f"{_fmt(best_score.get('mean_distance_m'))} |\n"
            )
        f.write("\n## Outputs\n\n")
        f.write("- overlay_png: `overlay.png`\n")
        f.write("- metrics_json: `metrics.json`\n")
        f.write("- points_csv: `points_<topic>.csv` for available scan topics\n")


def _fmt(value: Any) -> str:
    if value is None:
        return ""
    try:
        return f"{float(value):.4f}"
    except Exception:
        return str(value)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--after-snapshot", required=True)
    parser.add_argument("--map-yaml", default="")
    parser.add_argument("--scan-topics", default="/flatscan,/scan_amcl,/scan,/flatscan_localization")
    parser.add_argument("--flatscan-topics", default="/flatscan,/flatscan_localization")
    parser.add_argument("--pose-topics", default="/localization_result,/global_localization/pose,/amcl_pose")
    parser.add_argument("--timeout-sec", type=float, default=5.0)
    parser.add_argument("--tf-timeout-sec", type=float, default=4.0)
    parser.add_argument("--min-scans", type=int, default=1)
    parser.add_argument(
        "--early-exit-on-min-scans",
        action="store_true",
        help="Return as soon as --min-scans topics have arrived. Default waits the full capture window.",
    )
    parser.add_argument("--max-range-m", type=float, default=12.0)
    parser.add_argument("--max-score-points", type=int, default=900)
    parser.add_argument("--distance-cap-m", type=float, default=0.60)
    parser.add_argument("--search-xy-window-m", type=float, default=0.15)
    parser.add_argument("--search-xy-step-m", type=float, default=0.025)
    parser.add_argument("--search-yaw-window-deg", type=float, default=3.0)
    parser.add_argument("--search-yaw-step-deg", type=float, default=1.0)
    args = parser.parse_args()

    out_dir = Path(args.output_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    after_pose = load_after_pose(Path(args.after_snapshot))
    map_yaml = resolve_map_yaml(args.map_yaml)
    map_info = load_map(map_yaml)
    dist = build_distance_field(
        map_info["occupied"],
        int(map_info["width"]),
        int(map_info["height"]),
        float(map_info["resolution"]),
        float(args.distance_cap_m),
    )

    scan_topics = parse_csv_list(args.scan_topics)
    flatscan_topics = parse_csv_list(args.flatscan_topics)
    pose_topics = parse_csv_list(args.pose_topics)
    rclpy.init(args=None)
    node = AlignmentNode(scan_topics, pose_topics, flatscan_topics)
    try:
        node.collect(
            max(float(args.timeout_sec), 0.5),
            max(int(args.min_scans), 0),
            wait_full_window=not bool(args.early_exit_on_min_scans),
        )

        metrics: Dict[str, Any] = {
            "time_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
            "map_yaml": str(map_yaml),
            "map_image": str(map_info["image_path"]),
            "map": {
                "width": map_info["width"],
                "height": map_info["height"],
                "resolution": map_info["resolution"],
                "origin": map_info["origin"],
                "occupied_pixel_threshold": map_info["occupied_pixel_threshold"],
            },
            "after_pose": after_pose,
            "scan_topics_requested": scan_topics,
            "flatscan_topics_requested": flatscan_topics,
            "pose_topics_requested": pose_topics,
            "pose_topics": {
                topic: pose_msg_to_dict(msg, node) for topic, msg in sorted(node.poses.items())
            },
            "scan_topics": {},
        }

        overlay_points: Dict[str, Sequence[Point]] = {}
        for topic in scan_topics:
            scan = node.scans.get(topic)
            if scan is None:
                metrics["scan_topics"][topic] = {
                    "available": False,
                    "type": node.scan_types.get(topic, "unknown"),
                }
                continue

            scan_to_base = node.lookup_transform_pose(
                "base_link",
                scan.header.frame_id,
                timeout_sec=max(float(args.tf_timeout_sec), 0.2),
            )
            item: Dict[str, Any] = {
                "available": True,
                "type": node.scan_types.get(topic, type(scan).__name__),
                "frame_id": scan.header.frame_id,
                "stamp_sec": int(scan.header.stamp.sec),
                "stamp_nanosec": int(scan.header.stamp.nanosec),
                "age_sec": node.ros_stamp_age_sec(scan),
                "scan_to_base": scan_to_base,
            }
            if not scan_to_base or "error" in scan_to_base:
                item["error"] = "could not resolve scan frame to base_link"
                metrics["scan_topics"][topic] = item
                continue

            points_scan = scan_to_points(scan, max_range_m=float(args.max_range_m))
            points_scan_score = downsample(points_scan, max(int(args.max_score_points), 50))
            points_base_score = transform_points(points_scan_score, scan_to_base)
            points_map_score = transform_points(points_base_score, after_pose)
            score = score_points(points_map_score, dist, map_info)
            brute = brute_force_pose_score(
                points_base_score,
                after_pose,
                dist,
                map_info,
                float(args.search_xy_window_m),
                float(args.search_xy_step_m),
                float(args.search_yaw_window_deg),
                float(args.search_yaw_step_deg),
            )

            points_base_full = transform_points(points_scan, scan_to_base)
            points_map_full = transform_points(points_base_full, after_pose)
            safe_topic = topic.strip("/").replace("/", "_") or "scan"
            write_points_csv(out_dir / f"points_{safe_topic}.csv", points_scan, points_base_full, points_map_full)
            overlay_points[topic] = downsample(points_map_full, 4000)

            if brute.get("best") and score.get("mean_distance_m") is not None:
                best_score = ((brute.get("best") or {}).get("score") or {}).get("mean_distance_m")
                if best_score is not None:
                    brute["best"]["mean_improvement_m"] = float(score["mean_distance_m"]) - float(best_score)

            item.update(
                {
                    "raw_points": len(points_scan),
                    "score_points": len(points_scan_score),
                    "score_at_after_pose": score,
                    "bruteforce": brute,
                }
            )
            metrics["scan_topics"][topic] = item

        draw_overlay(out_dir / "overlay.png", map_info, overlay_points, after_pose)
        with (out_dir / "metrics.json").open("w", encoding="utf-8") as f:
            json.dump(metrics, f, indent=2, sort_keys=True)
            f.write("\n")
        write_summary(out_dir / "summary.md", metrics)
        print(f"[scan-map-align] summary: {out_dir / 'summary.md'}")
        print(f"[scan-map-align] overlay: {out_dir / 'overlay.png'}")
        return 0
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
