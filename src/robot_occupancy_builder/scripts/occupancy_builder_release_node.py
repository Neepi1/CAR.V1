#!/usr/bin/env python3
from __future__ import annotations

from pathlib import Path
import sys

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from occupancy_postprocess import run_release_rebuild  # noqa: E402


class OccupancyBuilderReleaseNode(Node):
    def __init__(self) -> None:
        super().__init__("robot_occupancy_builder_release")
        defaults = {
            "raw_bag_path": "/tmp/robot_mapping/raw_bag",
            "bag_storage_id": "sqlite3",
            "pointcloud_topic": "/sensors/lidar/points_raw",
            "optimized_trajectory_csv": "mapping_result/optimized_trajectory.csv",
            "output_root": "maps/building_1/floor_1",
            "map_frame_id": "map",
            "sensor_xyz": [0.25, 0.0, 1.05],
            "sensor_rpy": [0.0, 0.0, 0.0],
            "resolution": 0.05,
            "width_m": 200.0,
            "height_m": 200.0,
            "origin_x": -100.0,
            "origin_y": -100.0,
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
            "pose_match_tolerance_ms": 100.0,
            "status_topic": "/mapping/release_builder/status",
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self._config = {name: self.get_parameter(name).value for name in defaults if name != "status_topic"}
        self._status_pub = self.create_publisher(String, str(self.get_parameter("status_topic").value), 10)
        self._done = False
        self.create_timer(0.2, self._run_once)

    def _run_once(self) -> None:
        if self._done:
            return
        self._done = True
        self._status_pub.publish(String(data="release_rebuild_running"))
        try:
            result = run_release_rebuild(self._config)
            self.get_logger().info(f"release_rebuild completed: {result}")
            self._status_pub.publish(String(data=f"release_rebuild_complete:{result['assets']['asset_report']}"))
        except Exception as exc:
            self.get_logger().error(f"release_rebuild failed: {exc}")
            self._status_pub.publish(String(data=f"release_rebuild_failed:{exc}"))
        finally:
            self.create_timer(0.5, lambda: rclpy.shutdown())


def main() -> None:
    rclpy.init()
    node = OccupancyBuilderReleaseNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()


if __name__ == "__main__":
    main()
