#!/usr/bin/env python3
from __future__ import annotations

from pathlib import Path
import sys

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener


SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from occupancy_postprocess import OccupancyAccumulator, euler_to_quaternion, parse_pointcloud_xyz, pose_from_pose_stamped, transform_points  # noqa: E402


class OccupancyBuilderLiveNode(Node):
    def __init__(self) -> None:
        super().__init__("robot_occupancy_builder_live")
        defaults = {
            "points_topic": "/sensors/lidar/points_raw",
            "pose_topic": "/mapping/frontend_pose",
            "map_topic": "/mapping/draft_map",
            "status_topic": "/mapping/draft_map/status",
            "map_frame_id": "map",
            "base_frame": "base_link",
            "use_tf_for_sensor_extrinsics": True,
            "sensor_xyz": [0.25, 0.0, 0.85],
            "sensor_rpy": [0.0, 0.0, 0.0],
            "publish_period_sec": 0.5,
            "max_points_per_scan": 9000,
            "resolution": 0.10,
            "width_m": 160.0,
            "height_m": 160.0,
            "origin_x": -80.0,
            "origin_y": -80.0,
            "hit_log": 1.0,
            "miss_log": 0.55,
            "min_log": -5.0,
            "max_log": 5.0,
            "occupied_threshold": 1.8,
            "free_threshold": -0.55,
            "free_min_hits": 2,
            "free_post_close": 0,
            "free_speckle_neighbors": 1,
            "post_dilate": 0,
            "post_close": 1,
            "speckle_neighbors": 3,
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
            "terrain_cell_size": 0.15,
            "terrain_x_min": -4.5,
            "terrain_x_max": 40.0,
            "terrain_y_min": -10.0,
            "terrain_y_max": 10.0,
            "terrain_neighbor_radius": 0,
            "terrain_ground_quantile": 0.10,
            "terrain_min_points_per_cell": 3,
            "class_ground_min_rel_z": -0.08,
            "class_ground_max_rel_z": 0.06,
            "class_ramp_min_rel_z": 0.03,
            "class_ramp_max_rel_z": 0.22,
            "class_ramp_max_slope_deg": 12.0,
            "class_obstacle_min_rel_z": 0.18,
            "class_obstacle_max_rel_z": 1.60,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self._config = {name: self.get_parameter(name).value for name in defaults}
        self._sensor_xyz = [float(value) for value in self._config["sensor_xyz"]]
        sensor_rpy = [float(value) for value in self._config["sensor_rpy"]]
        self._sensor_quaternion = euler_to_quaternion(sensor_rpy[0], sensor_rpy[1], sensor_rpy[2])
        self._accumulator = OccupancyAccumulator(self._config)
        self._last_pose = None
        self._last_stamp = None
        self._last_stats = "draft_idle"
        self._map_pub = self.create_publisher(
            OccupancyGrid,
            str(self._config["map_topic"]),
            QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )
        self._status_pub = self.create_publisher(String, str(self._config["status_topic"]), 10)
        self.create_subscription(PoseStamped, str(self._config["pose_topic"]), self._on_pose, 20)
        self.create_subscription(PointCloud2, str(self._config["points_topic"]), self._on_cloud, 5)
        self.create_timer(float(self._config["publish_period_sec"]), self._publish_map)
        self._tf_buffer = Buffer(cache_time=Duration(seconds=5.0))
        self._tf_listener = TransformListener(self._tf_buffer, self, spin_thread=False)

    def _on_pose(self, msg: PoseStamped) -> None:
        self._last_pose = pose_from_pose_stamped(msg)
        self._last_stamp = msg.header.stamp

    def _cloud_to_base(self, msg: PointCloud2):
        points = parse_pointcloud_xyz(msg)
        if points.size == 0:
            return points
        frame_id = msg.header.frame_id or ""
        base_frame = str(self._config["base_frame"])
        if frame_id == base_frame or not frame_id:
            return points
        if bool(self._config["use_tf_for_sensor_extrinsics"]):
            try:
                transform = self._tf_buffer.lookup_transform(base_frame, frame_id, Time())
                translation = (
                    transform.transform.translation.x,
                    transform.transform.translation.y,
                    transform.transform.translation.z,
                )
                quaternion = (
                    transform.transform.rotation.x,
                    transform.transform.rotation.y,
                    transform.transform.rotation.z,
                    transform.transform.rotation.w,
                )
                return transform_points(points, translation, quaternion)
            except TransformException as exc:
                self.get_logger().warn(
                    f"Failed to lookup base transform {base_frame} <- {frame_id}, using configured sensor extrinsics: {exc}"
                )
        return transform_points(points, self._sensor_xyz, self._sensor_quaternion)

    def _on_cloud(self, msg: PointCloud2) -> None:
        if self._last_pose is None:
            return
        points_base = self._cloud_to_base(msg)
        if points_base.size == 0:
            return
        max_points_per_scan = max(100, int(self._config["max_points_per_scan"]))
        if len(points_base) > max_points_per_scan:
            stride = max(1, len(points_base) // max_points_per_scan)
            points_base = points_base[::stride]
        stats = self._accumulator.integrate_scan(points_base, self._last_pose, self._sensor_xyz)
        self._last_stamp = msg.header.stamp
        self._last_stats = (
            f"scans={self._accumulator.scans_processed} "
            f"ground={stats['ground']} ramp={stats['ramp']} obstacle={stats['obstacle']} free={stats['free']}"
        )

    def _publish_map(self) -> None:
        msg = OccupancyGrid()
        msg.header.frame_id = str(self._config["map_frame_id"])
        msg.header.stamp = self._last_stamp if self._last_stamp is not None else self.get_clock().now().to_msg()
        msg.info.resolution = float(self._accumulator.resolution)
        msg.info.width = int(self._accumulator.width)
        msg.info.height = int(self._accumulator.height)
        msg.info.origin.position.x = float(self._accumulator.origin_x)
        msg.info.origin.position.y = float(self._accumulator.origin_y)
        msg.info.origin.orientation.w = 1.0
        msg.data = self._accumulator.occupancy_data().tolist()
        self._map_pub.publish(msg)
        self._status_pub.publish(String(data=self._last_stats))


def main() -> None:
    rclpy.init()
    node = OccupancyBuilderLiveNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
