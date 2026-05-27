#!/usr/bin/env python3
from __future__ import annotations

from pathlib import Path
from threading import Event

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from robot_interfaces.srv import ApplyFloorAssets, TriggerLocalization
from std_srvs.srv import Empty
from std_msgs.msg import String


class GlobalLocalizationNode(Node):
    def __init__(self) -> None:
        super().__init__("robot_global_localization")
        self.declare_parameter("mock_mode", True)
        self.declare_parameter("publish_tf", False)
        self.declare_parameter("pose_topic", "/global_localization/pose")
        self.declare_parameter("health_topic", "/global_localization/health")
        self.declare_parameter("default_floor_id", "floor_1")
        self.declare_parameter("grid_search_trigger_service", "/trigger_grid_search_localization")
        self.declare_parameter("service_timeout_sec", 10.0)
        self.declare_parameter("require_grid_search_trigger", True)
        self.active_floor_id = self.get_parameter("default_floor_id").value
        self.active_nav_map_yaml = ""
        self.active_localizer_map_png = ""
        self.active_localizer_params_yaml = ""
        self.callback_group = ReentrantCallbackGroup()
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, self.get_parameter("pose_topic").value, 10)
        self.health_pub = self.create_publisher(String, self.get_parameter("health_topic").value, 10)
        self.grid_search_trigger_service = self.get_parameter("grid_search_trigger_service").value
        self.service_timeout_sec = float(self.get_parameter("service_timeout_sec").value)
        self.require_grid_search_trigger = bool(self.get_parameter("require_grid_search_trigger").value)
        self.grid_search_trigger_client = self.create_client(
            Empty, self.grid_search_trigger_service, callback_group=self.callback_group
        )
        self.create_service(
            TriggerLocalization, "/global_localization/trigger", self.on_trigger, callback_group=self.callback_group
        )
        self.create_service(
            ApplyFloorAssets, "/global_localization/apply_floor_assets", self.on_apply_floor, callback_group=self.callback_group
        )
        self.create_timer(1.0, self.on_timer)

    def on_trigger(self, request: TriggerLocalization.Request, response: TriggerLocalization.Response):
        if not self.grid_search_trigger_client.wait_for_service(timeout_sec=self.service_timeout_sec):
            response.accepted = not self.require_grid_search_trigger
            response.message = f"service unavailable: {self.grid_search_trigger_service}"
            return response

        done = Event()
        future = self.grid_search_trigger_client.call_async(Empty.Request())
        future.add_done_callback(lambda _: done.set())
        if not done.wait(timeout=self.service_timeout_sec):
            response.accepted = False
            response.message = f"timed out calling {self.grid_search_trigger_service}"
            return response
        if future.exception() is not None:
            response.accepted = False
            response.message = f"failed to call {self.grid_search_trigger_service}: {future.exception()}"
            return response

        response.accepted = True
        response.message = f"grid search localization trigger accepted: {request.reason}"
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
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
