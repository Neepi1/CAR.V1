#!/usr/bin/env python3
from __future__ import annotations

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from robot_interfaces.srv import ApplyFloorAssets, TriggerLocalization
from std_msgs.msg import String


class GlobalLocalizationNode(Node):
    def __init__(self) -> None:
        super().__init__("robot_global_localization")
        self.declare_parameter("mock_mode", True)
        self.declare_parameter("publish_tf", False)
        self.declare_parameter("pose_topic", "/global_localization/pose")
        self.declare_parameter("health_topic", "/global_localization/health")
        self.declare_parameter("default_floor_id", "floor_1")
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, self.get_parameter("pose_topic").value, 10)
        self.health_pub = self.create_publisher(String, self.get_parameter("health_topic").value, 10)
        self.create_service(TriggerLocalization, "/global_localization/trigger", self.on_trigger)
        self.create_service(ApplyFloorAssets, "/global_localization/apply_floor_assets", self.on_apply_floor)
        self.create_timer(1.0, self.on_timer)

    def on_trigger(self, request: TriggerLocalization.Request, response: TriggerLocalization.Response):
        response.accepted = True
        response.message = f"localization trigger accepted: {request.reason}"
        return response

    def on_apply_floor(self, request: ApplyFloorAssets.Request, response: ApplyFloorAssets.Response):
        response.success = True
        response.message = f"applied floor {request.floor_id}"
        return response

    def on_timer(self) -> None:
        pose = PoseWithCovarianceStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = "map"
        self.pose_pub.publish(pose)
        self.health_pub.publish(String(data="localizer_ready"))


def main() -> None:
    rclpy.init()
    node = GlobalLocalizationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
