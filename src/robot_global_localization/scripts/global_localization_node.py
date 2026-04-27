#!/usr/bin/env python3
from __future__ import annotations

from pathlib import Path

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
        self.active_floor_id = self.get_parameter("default_floor_id").value
        self.active_nav_map_yaml = ""
        self.active_localizer_map_png = ""
        self.active_localizer_params_yaml = ""
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
        required = {
            "nav_map_yaml": request.nav_map_yaml,
            "localizer_map_png": request.localizer_map_png,
            "localizer_params_yaml": request.localizer_params_yaml,
        }
        missing = [f"{name}={path}" for name, path in required.items() if not path or not Path(path).exists()]
        if missing:
            response.success = False
            response.message = "missing floor assets: " + "; ".join(missing)
            return response

        self.active_floor_id = request.floor_id
        self.active_nav_map_yaml = request.nav_map_yaml
        self.active_localizer_map_png = request.localizer_map_png
        self.active_localizer_params_yaml = request.localizer_params_yaml
        response.success = True
        response.message = f"applied floor {request.floor_id}: {request.nav_map_yaml}"
        return response

    def on_timer(self) -> None:
        pose = PoseWithCovarianceStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = "map"
        self.pose_pub.publish(pose)
        self.health_pub.publish(String(data=f"localizer_ready floor={self.active_floor_id}"))


def main() -> None:
    rclpy.init()
    node = GlobalLocalizationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
