#!/usr/bin/env python3
from __future__ import annotations

import math
from pathlib import Path

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


def parse_flat_yaml(path: Path) -> dict[str, str]:
    values: dict[str, str] = {}
    for raw_line in path.read_text(encoding="utf-8").splitlines():
        line = raw_line.split("#", 1)[0].strip()
        if not line or ":" not in line:
            continue
        key, value = line.split(":", 1)
        values[key.strip()] = value.strip()
    return values


def quaternion_from_rpy(roll: float, pitch: float, yaw: float):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def quaternion_multiply(lhs: tuple[float, float, float, float], rhs: tuple[float, float, float, float]):
    lx, ly, lz, lw = lhs
    rx, ry, rz, rw = rhs
    return (
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
        lw * rw - lx * rx - ly * ry - lz * rz,
    )


def yaw_from_quaternion(qx: float, qy: float, qz: float, qw: float) -> float:
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


class RobotDescriptionStaticTfNode(Node):
    def __init__(self) -> None:
        super().__init__("robot_description_static_tf")
        self.declare_parameter("config_file", "")
        config_file = str(self.get_parameter("config_file").value).strip()
        if not config_file:
            raise RuntimeError("config_file parameter is required")
        config = parse_flat_yaml(Path(config_file))
        self.broadcaster = StaticTransformBroadcaster(self)
        self.broadcaster.sendTransform(self.build_transforms(config))

    def make_transform(
        self,
        parent: str,
        child: str,
        xyz: tuple[float, float, float],
        rpy: tuple[float, float, float],
    ) -> TransformStamped:
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = parent
        tf.child_frame_id = child
        tf.transform.translation.x = xyz[0]
        tf.transform.translation.y = xyz[1]
        tf.transform.translation.z = xyz[2]
        qx, qy, qz, qw = quaternion_from_rpy(*rpy)
        tf.transform.rotation.x = qx
        tf.transform.rotation.y = qy
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw
        return tf

    def build_transforms(self, config: dict[str, str]) -> list[TransformStamped]:
        base_frame = config["base_frame"]
        base_footprint_frame = config["base_footprint_frame"]
        lidar_mount_frame = config["lidar_mount_frame"]
        lidar_frame = config["lidar_frame"]
        lidar_level_frame = config.get("lidar_level_frame", "lidar_level_link")
        imu_frame = config["imu_frame"]
        lidar_xyz = (
            float(config["lidar_x"]),
            float(config["lidar_y"]),
            float(config["lidar_z"]),
        )
        lidar_install_rpy = (
            float(config["lidar_roll"]),
            float(config["lidar_pitch"]),
            float(config["lidar_yaw"]),
        )
        lidar_axis_rpy = (
            float(config["lidar_axis_roll"]),
            float(config["lidar_axis_pitch"]),
            float(config["lidar_axis_yaw"]),
        )
        lidar_install_quat = quaternion_from_rpy(*lidar_install_rpy)
        lidar_axis_quat = quaternion_from_rpy(*lidar_axis_rpy)
        lidar_final_quat = quaternion_multiply(lidar_install_quat, lidar_axis_quat)
        lidar_level_yaw = yaw_from_quaternion(*lidar_final_quat)
        return [
            self.make_transform(base_frame, base_footprint_frame, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)),
            self.make_transform(
                base_frame,
                lidar_mount_frame,
                lidar_xyz,
                lidar_install_rpy,
            ),
            self.make_transform(
                lidar_mount_frame,
                lidar_frame,
                (0.0, 0.0, 0.0),
                lidar_axis_rpy,
            ),
            self.make_transform(
                base_frame,
                lidar_level_frame,
                lidar_xyz,
                (0.0, 0.0, lidar_level_yaw),
            ),
            self.make_transform(
                base_frame,
                imu_frame,
                (float(config["imu_x"]), float(config["imu_y"]), float(config["imu_z"])),
                (float(config["imu_roll"]), float(config["imu_pitch"]), float(config["imu_yaw"])),
            ),
        ]


def main() -> None:
    rclpy.init()
    node = RobotDescriptionStaticTfNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
