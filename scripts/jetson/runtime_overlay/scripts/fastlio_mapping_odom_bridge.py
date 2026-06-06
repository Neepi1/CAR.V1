#!/usr/bin/env python3
from __future__ import annotations

import math
from typing import Optional, Tuple

import rclpy
from geometry_msgs.msg import Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformException, TransformListener


Vector3 = Tuple[float, float, float]
Matrix3 = Tuple[Tuple[float, float, float], Tuple[float, float, float], Tuple[float, float, float]]
Transform = Tuple[Matrix3, Vector3]


def normalize_angle(value: float) -> float:
    return math.atan2(math.sin(value), math.cos(value))


def quaternion_to_matrix(q: Quaternion) -> Matrix3:
    x, y, z, w = q.x, q.y, q.z, q.w
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm <= 1e-12:
        return ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z
    return (
        (1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)),
        (2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)),
        (2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)),
    )


def yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    half = 0.5 * yaw
    q.z = math.sin(half)
    q.w = math.cos(half)
    return q


def transform_from_pose(position, orientation: Quaternion) -> Transform:
    return quaternion_to_matrix(orientation), (float(position.x), float(position.y), float(position.z))


def transform_from_msg(msg: TransformStamped) -> Transform:
    t = msg.transform.translation
    return quaternion_to_matrix(msg.transform.rotation), (float(t.x), float(t.y), float(t.z))


def rotate(matrix: Matrix3, vector: Vector3) -> Vector3:
    return (
        matrix[0][0] * vector[0] + matrix[0][1] * vector[1] + matrix[0][2] * vector[2],
        matrix[1][0] * vector[0] + matrix[1][1] * vector[1] + matrix[1][2] * vector[2],
        matrix[2][0] * vector[0] + matrix[2][1] * vector[1] + matrix[2][2] * vector[2],
    )


def matmul(a: Matrix3, b: Matrix3) -> Matrix3:
    return tuple(
        tuple(sum(a[row][k] * b[k][col] for k in range(3)) for col in range(3)) for row in range(3)
    )  # type: ignore[return-value]


def transpose(a: Matrix3) -> Matrix3:
    return tuple(tuple(a[col][row] for col in range(3)) for row in range(3))  # type: ignore[return-value]


def compose(a: Transform, b: Transform) -> Transform:
    ar, at = a
    br, bt = b
    r = matmul(ar, br)
    rotated_bt = rotate(ar, bt)
    t = (at[0] + rotated_bt[0], at[1] + rotated_bt[1], at[2] + rotated_bt[2])
    return r, t


def inverse(t: Transform) -> Transform:
    r, v = t
    rt = transpose(r)
    inv_v = rotate(rt, (-v[0], -v[1], -v[2]))
    return rt, inv_v


def yaw_from_matrix(r: Matrix3) -> float:
    return math.atan2(r[1][0], r[0][0])


class FastlioMappingOdomBridge(Node):
    def __init__(self) -> None:
        super().__init__("fastlio_mapping_odom_bridge")
        self.input_topic = self.declare_parameter("input_topic", "/Odometry").value
        self.output_topic = self.declare_parameter("output_topic", "/mapping/fastlio_odometry").value
        self.tf_topic = self.declare_parameter("tf_topic", "/tf_slam2d").value
        self.output_odom_frame = self.declare_parameter("output_odom_frame", "mapping_odom").value
        self.output_base_frame = self.declare_parameter("output_base_frame", "base_link").value
        self.sensor_frame = self.declare_parameter("sensor_frame", "lidar_link").value
        self.anchor_on_first_sample = bool(self.declare_parameter("anchor_on_first_sample", True).value)
        self.flatten_to_2d = bool(self.declare_parameter("flatten_to_2d", True).value)

        self.tf_buffer = Buffer(cache_time=Duration(seconds=30.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.odom_pub = self.create_publisher(Odometry, str(self.output_topic), 20)
        self.tf_pub = self.create_publisher(TFMessage, str(self.tf_topic), 100)
        self.create_subscription(Odometry, str(self.input_topic), self.on_odom, 20)

        self.base_from_child: Optional[Transform] = None
        self.anchor: Optional[Tuple[float, float, float]] = None
        self.get_logger().info(
            "mapping FAST-LIO odom bridge: "
            f"{self.input_topic} -> {self.output_topic}, private tf={self.tf_topic}"
        )

    def lookup_base_from_child(self, child_frame: str) -> Optional[Transform]:
        if child_frame == self.output_base_frame:
            identity: Matrix3 = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))
            return identity, (0.0, 0.0, 0.0)
        if self.base_from_child is not None:
            return self.base_from_child
        try:
            tf = self.tf_buffer.lookup_transform(
                str(self.output_base_frame), child_frame, Time(), timeout=Duration(seconds=0.05)
            )
        except TransformException as exc:
            self.get_logger().warn(
                f"waiting for static {self.output_base_frame}->{child_frame}: {exc}",
                throttle_duration_sec=2.0,
            )
            return None
        self.base_from_child = transform_from_msg(tf)
        self.get_logger().info(f"using static {self.output_base_frame}->{child_frame} to convert FAST-LIO odom")
        return self.base_from_child

    def on_odom(self, msg: Odometry) -> None:
        child_frame = msg.child_frame_id or str(self.sensor_frame)
        base_from_child = self.lookup_base_from_child(child_frame)
        if base_from_child is None:
            return

        odom_from_child = transform_from_pose(msg.pose.pose.position, msg.pose.pose.orientation)
        odom_from_base = compose(odom_from_child, inverse(base_from_child))
        rotation, translation = odom_from_base
        x, y, z = translation
        yaw = yaw_from_matrix(rotation)

        if self.anchor_on_first_sample:
            if self.anchor is None:
                self.anchor = (x, y, yaw)
            anchor_x, anchor_y, anchor_yaw = self.anchor
            dx, dy = x - anchor_x, y - anchor_y
            c, s = math.cos(anchor_yaw), math.sin(anchor_yaw)
            x = c * dx + s * dy
            y = -s * dx + c * dy
            yaw = normalize_angle(yaw - anchor_yaw)

        if self.flatten_to_2d:
            z = 0.0

        odom = Odometry()
        odom.header.stamp = msg.header.stamp
        odom.header.frame_id = str(self.output_odom_frame)
        odom.child_frame_id = str(self.output_base_frame)
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = z
        odom.pose.pose.orientation = yaw_to_quaternion(yaw)
        odom.pose.covariance = msg.pose.covariance
        odom.twist = msg.twist
        self.odom_pub.publish(odom)

        tf = TransformStamped()
        tf.header.stamp = msg.header.stamp
        tf.header.frame_id = str(self.output_odom_frame)
        tf.child_frame_id = str(self.output_base_frame)
        tf.transform.translation.x = x
        tf.transform.translation.y = y
        tf.transform.translation.z = z
        tf.transform.rotation = odom.pose.pose.orientation
        self.tf_pub.publish(TFMessage(transforms=[tf]))


def main() -> None:
    rclpy.init()
    node = FastlioMappingOdomBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
