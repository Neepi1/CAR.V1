#!/usr/bin/env python3
from __future__ import annotations

import math
from typing import Iterable

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu


def make_qos(depth: int, reliability: ReliabilityPolicy) -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
        reliability=reliability,
        durability=DurabilityPolicy.VOLATILE,
    )


def normalize_quaternion(qx: float, qy: float, qz: float, qw: float) -> tuple[float, float, float, float]:
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm <= 0.0:
        return (0.0, 0.0, 0.0, 1.0)
    return (qx / norm, qy / norm, qz / norm, qw / norm)


def quaternion_from_rotation_matrix(matrix: np.ndarray) -> tuple[float, float, float, float]:
    trace = float(matrix[0, 0] + matrix[1, 1] + matrix[2, 2])
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * s
        qx = (matrix[2, 1] - matrix[1, 2]) / s
        qy = (matrix[0, 2] - matrix[2, 0]) / s
        qz = (matrix[1, 0] - matrix[0, 1]) / s
    elif matrix[0, 0] > matrix[1, 1] and matrix[0, 0] > matrix[2, 2]:
        s = math.sqrt(1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2]) * 2.0
        qw = (matrix[2, 1] - matrix[1, 2]) / s
        qx = 0.25 * s
        qy = (matrix[0, 1] + matrix[1, 0]) / s
        qz = (matrix[0, 2] + matrix[2, 0]) / s
    elif matrix[1, 1] > matrix[2, 2]:
        s = math.sqrt(1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2]) * 2.0
        qw = (matrix[0, 2] - matrix[2, 0]) / s
        qx = (matrix[0, 1] + matrix[1, 0]) / s
        qy = 0.25 * s
        qz = (matrix[1, 2] + matrix[2, 1]) / s
    else:
        s = math.sqrt(1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1]) * 2.0
        qw = (matrix[1, 0] - matrix[0, 1]) / s
        qx = (matrix[0, 2] + matrix[2, 0]) / s
        qy = (matrix[1, 2] + matrix[2, 1]) / s
        qz = 0.25 * s
    return normalize_quaternion(qx, qy, qz, qw)


def quaternion_multiply(
    lhs: tuple[float, float, float, float],
    rhs: tuple[float, float, float, float],
) -> tuple[float, float, float, float]:
    lx, ly, lz, lw = lhs
    rx, ry, rz, rw = rhs
    return normalize_quaternion(
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
        lw * rw - lx * rx - ly * ry - lz * rz,
    )


def quaternion_conjugate(q: tuple[float, float, float, float]) -> tuple[float, float, float, float]:
    qx, qy, qz, qw = q
    return (-qx, -qy, -qz, qw)


def rotate_covariance(matrix: np.ndarray, covariance: Iterable[float]) -> list[float]:
    raw = [float(value) for value in covariance]
    if len(raw) != 9:
        return raw
    cov = np.asarray(raw, dtype=np.float64).reshape((3, 3))
    rotated = matrix @ cov @ matrix.T
    return rotated.reshape((9,)).tolist()


class ImuAxisRemap(Node):
    def __init__(self) -> None:
        super().__init__("imu_axis_remap")
        self.declare_parameter("input_topic", "/jt128/vendor/imu_raw")
        self.declare_parameter("output_topic", "/lidar_imu")
        self.declare_parameter("output_frame_id", "imu_link")
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
        self.rotation_quaternion = quaternion_from_rotation_matrix(self.rotation)

        self.publisher = self.create_publisher(
            Imu,
            self.output_topic,
            make_qos(depth=50, reliability=ReliabilityPolicy.RELIABLE),
        )
        self.subscription = self.create_subscription(
            Imu,
            self.input_topic,
            self._on_imu,
            make_qos(depth=50, reliability=ReliabilityPolicy.RELIABLE),
        )
        self._logged_ready = False

    def _load_rotation_matrix(self) -> np.ndarray:
        raw = [float(value) for value in self.get_parameter("rotation_matrix").value]
        if len(raw) != 9:
            raise RuntimeError(f"rotation_matrix must contain 9 values, got {len(raw)}")
        return np.asarray(raw, dtype=np.float64).reshape((3, 3))

    def _rotate_vector(self, x: float, y: float, z: float) -> tuple[float, float, float]:
        vector = self.rotation @ np.asarray([x, y, z], dtype=np.float64)
        return (float(vector[0]), float(vector[1]), float(vector[2]))

    def _on_imu(self, msg: Imu) -> None:
        output = Imu()
        output.header.stamp = msg.header.stamp
        output.header.frame_id = self.output_frame_id

        output.angular_velocity.x, output.angular_velocity.y, output.angular_velocity.z = self._rotate_vector(
            float(msg.angular_velocity.x),
            float(msg.angular_velocity.y),
            float(msg.angular_velocity.z),
        )
        output.linear_acceleration.x, output.linear_acceleration.y, output.linear_acceleration.z = self._rotate_vector(
            float(msg.linear_acceleration.x),
            float(msg.linear_acceleration.y),
            float(msg.linear_acceleration.z),
        )

        output.angular_velocity_covariance = rotate_covariance(self.rotation, msg.angular_velocity_covariance)
        output.linear_acceleration_covariance = rotate_covariance(self.rotation, msg.linear_acceleration_covariance)

        output.orientation = msg.orientation
        output.orientation_covariance = list(msg.orientation_covariance)
        if len(output.orientation_covariance) == 9 and float(output.orientation_covariance[0]) >= 0.0:
            q_in = normalize_quaternion(
                float(msg.orientation.x),
                float(msg.orientation.y),
                float(msg.orientation.z),
                float(msg.orientation.w),
            )
            q_out = quaternion_multiply(q_in, quaternion_conjugate(self.rotation_quaternion))
            output.orientation.x = float(q_out[0])
            output.orientation.y = float(q_out[1])
            output.orientation.z = float(q_out[2])
            output.orientation.w = float(q_out[3])
            output.orientation_covariance = rotate_covariance(self.rotation, output.orientation_covariance)

        self.publisher.publish(output)

        if not self._logged_ready:
            self.get_logger().info(
                f"canonical imu remap ready: {self.input_topic} -> {self.output_topic} frame={self.output_frame_id}"
            )
            self._logged_ready = True


def main() -> None:
    rclpy.init()
    node = ImuAxisRemap()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
