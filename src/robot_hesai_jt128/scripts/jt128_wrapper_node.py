#!/usr/bin/env python3
from __future__ import annotations

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, PointCloud2


class Jt128WrapperNode(Node):
    def __init__(self) -> None:
        super().__init__("robot_hesai_jt128")
        defaults = {
            "mock_mode": True,
            "bag_mode": False,
            "device_ip": "192.168.1.201",
            "host_ip": "192.168.1.100",
            "lidar_port": 2368,
            "imu_port": 10110,
            "vendor_points_topic": "/jt128/vendor/points_raw",
            "vendor_imu_topic": "/jt128/vendor/imu_raw",
            "points_topic": "/lidar_points",
            "imu_topic": "/lidar_imu",
            "lidar_frame": "lidar_link",
            "imu_frame": "imu_link",
            "extrinsics_yaml": "",
            "publish_vendor_tf": False,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        self.mock_mode = self.get_parameter("mock_mode").value
        self.points_pub = self.create_publisher(PointCloud2, self.get_parameter("points_topic").value, 10)
        self.imu_pub = self.create_publisher(Imu, self.get_parameter("imu_topic").value, 10)
        self.create_timer(0.1, self.on_timer)

    def on_timer(self) -> None:
        if not self.mock_mode:
            return
        stamp = self.get_clock().now().to_msg()
        cloud = PointCloud2()
        cloud.header.stamp = stamp
        cloud.header.frame_id = self.get_parameter("lidar_frame").value
        imu = Imu()
        imu.header.stamp = stamp
        imu.header.frame_id = self.get_parameter("imu_frame").value
        self.points_pub.publish(cloud)
        self.imu_pub.publish(imu)


def main() -> None:
    rclpy.init()
    node = Jt128WrapperNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
