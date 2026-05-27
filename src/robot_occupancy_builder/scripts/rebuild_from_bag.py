#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys


SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from occupancy_postprocess import run_release_rebuild  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser(description="Rebuild release 2D occupancy assets from raw bag + optimized trajectory.")
    parser.add_argument("--raw-bag-path", required=True)
    parser.add_argument("--optimized-trajectory-csv", required=True)
    parser.add_argument("--output-root", required=True)
    parser.add_argument("--pointcloud-topic", default="/sensors/lidar/points_raw")
    parser.add_argument("--bag-storage-id", default="sqlite3")
    parser.add_argument("--map-frame-id", default="map")
    parser.add_argument("--resolution", type=float, default=0.05)
    parser.add_argument("--width-m", type=float, default=200.0)
    parser.add_argument("--height-m", type=float, default=200.0)
    parser.add_argument("--origin-x", type=float, default=-100.0)
    parser.add_argument("--origin-y", type=float, default=-100.0)
    parser.add_argument("--pose-match-tolerance-ms", type=float, default=100.0)
    args = parser.parse_args()

    config = {
        "raw_bag_path": args.raw_bag_path,
        "bag_storage_id": args.bag_storage_id,
        "pointcloud_topic": args.pointcloud_topic,
        "optimized_trajectory_csv": args.optimized_trajectory_csv,
        "output_root": args.output_root,
        "map_frame_id": args.map_frame_id,
        "sensor_xyz": [0.25, 0.0, 0.85],
        "sensor_rpy": [0.0, 0.0, 0.0],
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
    result = run_release_rebuild(config)
    print(json.dumps(result, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
