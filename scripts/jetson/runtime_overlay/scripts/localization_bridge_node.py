#!/usr/bin/env python3
from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool
from tf2_ros import TransformBroadcaster


def yaw_from_quaternion(quat: Quaternion) -> float:
    siny_cosp = 2.0 * (quat.w * quat.z + quat.x * quat.y)
    cosy_cosp = 1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z)
    return math.atan2(siny_cosp, cosy_cosp)


def quaternion_from_yaw(yaw: float) -> Quaternion:
    quat = Quaternion()
    quat.z = math.sin(yaw * 0.5)
    quat.w = math.cos(yaw * 0.5)
    return quat


class LocalizationBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__("robot_localization_bridge")
        self.declare_parameter("publish_tf", True)
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("jump_threshold_m", 1.0)
        self.declare_parameter("timeout_sec", 1.0)
        self.declare_parameter("localization_topic", "/localization_result")
        self.declare_parameter("local_odom_topic", "/local_state/odometry")
        self.declare_parameter("health_topic", "/localization/health")
        self.declare_parameter("two_d_mode", True)
        self.create_subscription(
            PoseWithCovarianceStamped,
            self.get_parameter("localization_topic").value,
            self.on_pose,
            20,
        )
        self.create_subscription(
            Odometry,
            self.get_parameter("local_odom_topic").value,
            self.on_odom,
            20,
        )
        self.tf_broadcaster = TransformBroadcaster(self)
        self.health_pub = self.create_publisher(Bool, self.get_parameter("health_topic").value, 10)
        self.latest_pose = None
        self.latest_pose_received_sec = None
        self.latest_odom = None
        self.latest_map_to_odom = None
        self.last_pose_stamp_used = None
        self.last_health_state = None
        self.last_health_reason = ""
        self.create_timer(0.1, self.on_timer)

    def on_pose(self, msg: PoseWithCovarianceStamped) -> None:
        self.latest_pose = msg
        self.latest_pose_received_sec = self.get_clock().now().nanoseconds / 1e9
        self._refresh_state("pose")

    def on_odom(self, msg: Odometry) -> None:
        self.latest_odom = msg
        if self.latest_map_to_odom is None and self.latest_pose is not None:
            self._refresh_state("odom")

    def on_timer(self) -> None:
        self._refresh_state("timer")

    def _publish_health(self, ok: bool, reason: str) -> None:
        self.health_pub.publish(Bool(data=ok))
        if self.last_health_state == ok and self.last_health_reason == reason:
            return
        self.last_health_state = ok
        self.last_health_reason = reason
        if ok:
            self.get_logger().info(reason)
        else:
            self.get_logger().warning(reason)

    def _refresh_state(self, source: str) -> None:
        timeout_sec = float(self.get_parameter("timeout_sec").value)
        if self.latest_odom is None:
            self._publish_health(False, f"bridge waiting for odom ({source})")
            return

        now_sec = self.get_clock().now().nanoseconds / 1e9
        odom_sec = self.latest_odom.header.stamp.sec + self.latest_odom.header.stamp.nanosec / 1e9
        if now_sec - odom_sec > timeout_sec:
            self._publish_health(False, f"bridge odom timeout ({source})")
            return

        update_from_pose = False
        pose_stamp = None
        if self.latest_pose is not None:
            pose_stamp = (
                int(self.latest_pose.header.stamp.sec),
                int(self.latest_pose.header.stamp.nanosec),
            )
            pose_received_sec = self.latest_pose_received_sec
            if pose_received_sec is None:
                pose_received_sec = self.latest_pose.header.stamp.sec + self.latest_pose.header.stamp.nanosec / 1e9
            if self.latest_map_to_odom is None:
                if now_sec - pose_received_sec > timeout_sec:
                    self._publish_health(False, f"bridge localization_result timeout before initial lock ({source})")
                    return
                update_from_pose = True
            elif pose_stamp != self.last_pose_stamp_used and now_sec - pose_received_sec <= timeout_sec:
                update_from_pose = True
        elif self.latest_map_to_odom is None:
            self._publish_health(False, f"bridge waiting for localization_result ({source})")
            return

        if update_from_pose:
            map_x = float(self.latest_pose.pose.pose.position.x)
            map_y = float(self.latest_pose.pose.pose.position.y)
            map_yaw = yaw_from_quaternion(self.latest_pose.pose.pose.orientation)
            odom_x = float(self.latest_odom.pose.pose.position.x)
            odom_y = float(self.latest_odom.pose.pose.position.y)
            odom_yaw = yaw_from_quaternion(self.latest_odom.pose.pose.orientation)

            map_to_odom_yaw = math.atan2(math.sin(map_yaw - odom_yaw), math.cos(map_yaw - odom_yaw))
            cos_delta = math.cos(map_to_odom_yaw)
            sin_delta = math.sin(map_to_odom_yaw)
            map_to_odom_x = map_x - (cos_delta * odom_x - sin_delta * odom_y)
            map_to_odom_y = map_y - (sin_delta * odom_x + cos_delta * odom_y)

            if self.latest_map_to_odom is not None:
                dx = map_to_odom_x - self.latest_map_to_odom["x"]
                dy = map_to_odom_y - self.latest_map_to_odom["y"]
                jump = math.hypot(dx, dy)
                if jump > float(self.get_parameter("jump_threshold_m").value):
                    self._publish_health(False, f"bridge map->odom jump rejected: {jump:.3f} m ({source})")
                    return

            self.latest_map_to_odom = {
                "x": map_to_odom_x,
                "y": map_to_odom_y,
                "yaw": map_to_odom_yaw,
            }
            self.last_pose_stamp_used = pose_stamp

        if self.latest_map_to_odom is None:
            self._publish_health(False, f"bridge has no map->odom solution ({source})")
            return
        self._publish_health(True, f"bridge map->odom active ({source})")
        if not self.get_parameter("publish_tf").value:
            return
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = self.get_parameter("map_frame").value
        tf.child_frame_id = self.get_parameter("odom_frame").value
        tf.transform.translation.x = self.latest_map_to_odom["x"]
        tf.transform.translation.y = self.latest_map_to_odom["y"]
        if not bool(self.get_parameter("two_d_mode").value):
            tf.transform.translation.z = (
                float(self.latest_pose.pose.pose.position.z) - float(self.latest_odom.pose.pose.position.z)
            )
        tf.transform.rotation = quaternion_from_yaw(self.latest_map_to_odom["yaw"])
        self.tf_broadcaster.sendTransform(tf)


def main() -> None:
    rclpy.init()
    node = LocalizationBridgeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
