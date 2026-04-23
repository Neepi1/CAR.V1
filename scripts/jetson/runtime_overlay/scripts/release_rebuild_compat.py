#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import shutil
import sys
from pathlib import Path


WORKSPACE_ROOT = Path(__file__).resolve().parents[4]
OCCUPANCY_SCRIPTS = WORKSPACE_ROOT / "src" / "robot_occupancy_builder" / "scripts"
if str(OCCUPANCY_SCRIPTS) not in sys.path:
    sys.path.insert(0, str(OCCUPANCY_SCRIPTS))

from occupancy_postprocess import run_release_rebuild  # noqa: E402


def parse_triplet(value: str) -> list[float]:
    parts = [item.strip() for item in value.split(",") if item.strip()]
    if len(parts) != 3:
        raise argparse.ArgumentTypeError("expected three comma-separated floats")
    return [float(item) for item in parts]


def copy_asset(source: Path, target: Path) -> str:
    target.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(source, target)
    return str(target)


def main() -> int:
    parser = argparse.ArgumentParser(description="Generate release 2D assets and compatibility map files from raw bag + optimized trajectory.")
    parser.add_argument("--map-name", required=True)
    parser.add_argument("--raw-bag-path", required=True)
    parser.add_argument("--optimized-trajectory-csv", required=True)
    parser.add_argument("--output-root", required=True, help="Structured release asset root, e.g. maps_release/<name>")
    parser.add_argument("--maps-dir", required=True, help="Compatibility output directory used by the current dashboard/Nav2 picker")
    parser.add_argument("--pointcloud-topic", default="/lidar_points")
    parser.add_argument("--bag-storage-id", default="sqlite3")
    parser.add_argument("--map-frame-id", default="map")
    parser.add_argument("--resolution", type=float, default=0.05)
    parser.add_argument("--width-m", type=float, default=200.0)
    parser.add_argument("--height-m", type=float, default=200.0)
    parser.add_argument("--origin-x", type=float, default=-100.0)
    parser.add_argument("--origin-y", type=float, default=-100.0)
    parser.add_argument("--pose-match-tolerance-ms", type=float, default=100.0)
    parser.add_argument("--sensor-xyz", type=parse_triplet, default=[0.25, 0.0, 1.05])
    parser.add_argument("--sensor-rpy", type=parse_triplet, default=[0.0, 0.0, 0.0])
    args = parser.parse_args()

    output_root = Path(args.output_root).resolve()
    maps_dir = Path(args.maps_dir).resolve()
    result = run_release_rebuild(
        {
            "raw_bag_path": args.raw_bag_path,
            "bag_storage_id": args.bag_storage_id,
            "pointcloud_topic": args.pointcloud_topic,
            "optimized_trajectory_csv": args.optimized_trajectory_csv,
            "output_root": str(output_root),
            "map_frame_id": args.map_frame_id,
            "sensor_xyz": args.sensor_xyz,
            "sensor_rpy": args.sensor_rpy,
            "resolution": args.resolution,
            "width_m": args.width_m,
            "height_m": args.height_m,
            "origin_x": args.origin_x,
            "origin_y": args.origin_y,
            "hit_log": 1.2,
            "miss_log": 0.30,
            "min_log": -5.0,
            "max_log": 5.0,
            "occupied_threshold": 1.0,
            "free_threshold": -1.0,
            "post_dilate": 1,
            "post_close": 2,
            "speckle_neighbors": 2,
            "range_filter_min": 0.5,
            "range_filter_max": 40.0,
            "height_filter_min_z": -0.20,
            "height_filter_max_z": 1.60,
            "azimuth_filter_enabled": True,
            "azimuth_filter_min_angle_deg": -110.0,
            "azimuth_filter_max_angle_deg": 110.0,
            "self_mask_enabled": True,
            "self_mask_min_x": -0.55,
            "self_mask_max_x": 0.75,
            "self_mask_min_y": -0.40,
            "self_mask_max_y": 0.40,
            "self_mask_min_z": -0.10,
            "self_mask_max_z": 1.40,
            "front_mask_enabled": True,
            "front_mask_min_x": 0.20,
            "front_mask_max_x": 1.20,
            "front_mask_min_y": -0.45,
            "front_mask_max_y": 0.45,
            "front_mask_min_z": -0.10,
            "front_mask_max_z": 1.60,
            "terrain_cell_size": 0.20,
            "terrain_x_min": -4.5,
            "terrain_x_max": 40.0,
            "terrain_y_min": -10.0,
            "terrain_y_max": 10.0,
            "terrain_neighbor_radius": 1,
            "terrain_ground_quantile": 0.15,
            "terrain_min_points_per_cell": 2,
            "class_ground_min_rel_z": -0.08,
            "class_ground_max_rel_z": 0.08,
            "class_ramp_min_rel_z": 0.02,
            "class_ramp_max_rel_z": 0.28,
            "class_ramp_max_slope_deg": 12.0,
            "class_obstacle_min_rel_z": 0.12,
            "class_obstacle_max_rel_z": 1.60,
            "pose_match_tolerance_ms": args.pose_match_tolerance_ms,
        }
    )

    assets = {name: Path(path) for name, path in result["assets"].items()}
    safe_name = "".join(ch if ch.isalnum() or ch in ("-", "_") else "_" for ch in args.map_name).strip("_")
    if not safe_name:
        raise RuntimeError("Invalid map name")

    compat_yaml = maps_dir / f"{safe_name}.yaml"
    compat_pgm = maps_dir / f"{safe_name}.pgm"
    compat_localizer_png = maps_dir / f"{safe_name}.localizer.png"
    compat_localizer_yaml = maps_dir / f"{safe_name}.localizer.yaml"

    payload = {
        "ok": True,
        "map_name": safe_name,
        "release_root": str(output_root),
        "assets": {name: str(path) for name, path in assets.items()},
        "nav_map": {
            "yaml": str(assets["nav_yaml"]),
            "image": str(assets["nav_pgm"]),
        },
        "localizer_map": {
            "yaml": str(assets["localizer_yaml"]),
            "image": str(assets["localizer_png"]),
        },
        "compat_map": {
            "yaml": copy_asset(assets["nav_yaml"], compat_yaml),
            "image": copy_asset(assets["nav_pgm"], compat_pgm),
        },
        "compat_localizer_map": {
            "yaml": copy_asset(assets["localizer_yaml"], compat_localizer_yaml),
            "image": copy_asset(assets["localizer_png"], compat_localizer_png),
        },
        "scans_seen": int(result["scans_seen"]),
        "scans_used": int(result["scans_used"]),
    }
    print(json.dumps(payload, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
