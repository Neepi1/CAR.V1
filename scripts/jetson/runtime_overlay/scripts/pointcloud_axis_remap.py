#!/usr/bin/env python3
from __future__ import annotations

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


def make_qos(depth: int, reliability: ReliabilityPolicy) -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
        reliability=reliability,
        durability=DurabilityPolicy.VOLATILE,
    )


class PointCloudAxisRemap(Node):
    def __init__(self) -> None:
        super().__init__("pointcloud_axis_remap")
        self.declare_parameter("input_topic", "/jt128/vendor/points_raw")
        self.declare_parameter("output_topic", "/lidar_points")
        self.declare_parameter("output_frame_id", "lidar_link")
        self.declare_parameter(
            "rotation_matrix",
            [
                1.0,
                0.0,
                0.0,
                0.0,
                1.0,
                0.0,
                0.0,
                0.0,
                1.0,
            ],
        )

        self.input_topic = str(self.get_parameter("input_topic").value)
        self.output_topic = str(self.get_parameter("output_topic").value)
        self.output_frame_id = str(self.get_parameter("output_frame_id").value)
        self.rotation = self._load_rotation_matrix()

        self.publisher = self.create_publisher(
            PointCloud2,
            self.output_topic,
            make_qos(depth=10, reliability=ReliabilityPolicy.RELIABLE),
        )
        self.subscription = self.create_subscription(
            PointCloud2,
            self.input_topic,
            self._on_cloud,
            make_qos(depth=10, reliability=ReliabilityPolicy.RELIABLE),
        )
        self._warned_missing_xyz = False
        self._warned_dtype = False
        self._logged_ready = False

    def _load_rotation_matrix(self) -> np.ndarray:
        raw = [float(value) for value in self.get_parameter("rotation_matrix").value]
        if len(raw) != 9:
            raise RuntimeError(f"rotation_matrix must contain 9 values, got {len(raw)}")
        return np.asarray(raw, dtype=np.float32).reshape((3, 3))

    def _copy_cloud_metadata(self, source: PointCloud2) -> PointCloud2:
        output = PointCloud2()
        output.header.stamp = source.header.stamp
        output.header.frame_id = self.output_frame_id
        output.height = source.height
        output.width = source.width
        output.fields = list(source.fields)
        output.is_bigendian = source.is_bigendian
        output.point_step = source.point_step
        output.row_step = source.row_step
        output.is_dense = source.is_dense
        return output

    def _on_cloud(self, msg: PointCloud2) -> None:
        output = self._copy_cloud_metadata(msg)
        if not msg.data or msg.width == 0 or msg.height == 0:
            output.data = bytes(msg.data)
            self.publisher.publish(output)
            return

        try:
            dtype = point_cloud2.dtype_from_fields(msg.fields, msg.point_step)
        except Exception as exc:
            if not self._warned_dtype:
                self.get_logger().error(f"failed to derive PointCloud2 dtype: {exc}")
                self._warned_dtype = True
            return

        field_names = set(dtype.names or ())
        if not {"x", "y", "z"}.issubset(field_names):
            if not self._warned_missing_xyz:
                self.get_logger().error(
                    f"cloud on {self.input_topic} does not expose x/y/z fields: {sorted(field_names)}"
                )
                self._warned_missing_xyz = True
            return

        points = np.frombuffer(msg.data, dtype=dtype, count=msg.width * msg.height).copy()
        x = points["x"].astype(np.float32, copy=True)
        y = points["y"].astype(np.float32, copy=True)
        z = points["z"].astype(np.float32, copy=True)

        points["x"] = self.rotation[0, 0] * x + self.rotation[0, 1] * y + self.rotation[0, 2] * z
        points["y"] = self.rotation[1, 0] * x + self.rotation[1, 1] * y + self.rotation[1, 2] * z
        points["z"] = self.rotation[2, 0] * x + self.rotation[2, 1] * y + self.rotation[2, 2] * z

        output.data = points.tobytes()
        self.publisher.publish(output)

        if not self._logged_ready:
            self.get_logger().info(
                f"canonical pointcloud remap ready: {self.input_topic} -> {self.output_topic} frame={self.output_frame_id}"
            )
            self._logged_ready = True


def main() -> None:
    rclpy.init()
    node = PointCloudAxisRemap()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
