#!/usr/bin/env python3
from __future__ import annotations

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster


class LocalStateNode(Node):
    def __init__(self) -> None:
        super().__init__("robot_local_state")
        self.declare_parameter("mock_mode", True)
        self.declare_parameter("publish_tf", True)
        self.declare_parameter("output_topic", "/local_state/odometry")
        self.declare_parameter("input_odom_topic", "/wheel/odom")
        self.declare_parameter("input_imu_topic", "/sensors/lidar/imu")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("publish_rate_hz", 20.0)
        self.odom_pub = self.create_publisher(Odometry, self.get_parameter("output_topic").value, 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.mock_mode = bool(self.get_parameter("mock_mode").value)
        self.latest_imu = None
        if self.mock_mode:
            self.create_timer(1.0 / float(self.get_parameter("publish_rate_hz").value), self.on_mock_timer)
        else:
            self.create_subscription(
                Odometry,
                self.get_parameter("input_odom_topic").value,
                self.on_wheel_odom,
                20,
            )
            self.create_subscription(
                Imu,
                self.get_parameter("input_imu_topic").value,
                self.on_imu,
                20,
            )

    def publish_local_state(self, odom: Odometry) -> None:
        stamp = odom.header.stamp
        self.odom_pub.publish(odom)
        if not self.get_parameter("publish_tf").value:
            return
        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = odom.header.frame_id
        tf.child_frame_id = odom.child_frame_id
        tf.transform.translation.x = odom.pose.pose.position.x
        tf.transform.translation.y = odom.pose.pose.position.y
        tf.transform.translation.z = odom.pose.pose.position.z
        tf.transform.rotation = odom.pose.pose.orientation
        self.tf_broadcaster.sendTransform(tf)

    def on_wheel_odom(self, msg: Odometry) -> None:
        local_odom = Odometry()
        local_odom.header = msg.header
        local_odom.header.frame_id = self.get_parameter("odom_frame").value
        local_odom.child_frame_id = self.get_parameter("base_frame").value
        local_odom.pose = msg.pose
        local_odom.twist = msg.twist
        self.publish_local_state(local_odom)

    def on_imu(self, msg: Imu) -> None:
        self.latest_imu = msg

    def on_mock_timer(self) -> None:
        stamp = self.get_clock().now().to_msg()
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.get_parameter("odom_frame").value
        odom.child_frame_id = self.get_parameter("base_frame").value
        self.publish_local_state(odom)


def main() -> None:
    rclpy.init()
    node = LocalStateNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
