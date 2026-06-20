#!/usr/bin/env python3
"""Persistent ROS runtime health snapshot writer.

This process intentionally replaces many short-lived readiness probes with one
long-lived rclpy participant. Shell scripts can read the JSON snapshot without
creating extra DDS participants during navigation startup.

Keep this node lightweight. It must not become a second perception/costmap
consumer in normal navigation, because Python deserialization of PointCloud2 and
OccupancyGrid messages can steal scheduling time from FAST-LIO, perception, and
Nav2. Heavy topics are tracked through ROS graph endpoint counts by default;
startup scripts perform direct one-shot readiness checks when message freshness
is safety critical.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import tempfile
import time
from pathlib import Path
from typing import Any

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage


def _env_float(name: str, default: float) -> float:
    try:
        return float(os.environ.get(name, str(default)))
    except (TypeError, ValueError):
        return default


def _stamp_to_sec(stamp: Any) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9


def _strip_slash(frame_id: str) -> str:
    return str(frame_id or "").strip().lstrip("/")


def _full_node_name(name: str, namespace: str) -> str:
    namespace = namespace or "/"
    if namespace == "/":
        return "/" + name
    return namespace.rstrip("/") + "/" + name


def _bool_topic(value: dict[str, Any]) -> bool:
    return bool(value.get("publishers", 0) > 0 and value.get("last_age_sec") is not None)


class RuntimeHealthGuard(Node):
    def __init__(self, output_path: Path, once: bool) -> None:
        super().__init__("runtime_health_guard")
        self.output_path = output_path
        self.once = once
        self.started_monotonic = time.monotonic()
        self.graph_last_checked = 0.0
        self.graph_period_sec = _env_float("NJRH_RUNTIME_HEALTH_GRAPH_PERIOD_SEC", 2.0)
        self.write_period_sec = _env_float("NJRH_RUNTIME_HEALTH_WRITE_PERIOD_SEC", 1.0)
        self.topic_fresh_sec = _env_float("NJRH_RUNTIME_HEALTH_TOPIC_FRESH_SEC", 1.5)
        self.odom_fresh_sec = _env_float("NJRH_RUNTIME_HEALTH_ODOM_FRESH_SEC", 0.75)
        self.tf_fresh_sec = _env_float("NJRH_RUNTIME_HEALTH_TF_FRESH_SEC", 0.25)
        self.map_tf_fresh_sec = _env_float("NJRH_RUNTIME_HEALTH_MAP_TF_FRESH_SEC", 1.0)
        self.scan_fresh_sec = _env_float("NJRH_RUNTIME_HEALTH_SCAN_FRESH_SEC", 1.0)
        self.observe_heavy_topics = os.environ.get(
            "NJRH_RUNTIME_HEALTH_OBSERVE_HEAVY_TOPICS", "false"
        ).lower() in {"1", "true", "yes", "on"}

        self.topics: dict[str, dict[str, Any]] = {}
        self.tf_edges: dict[str, dict[str, Any]] = {}
        self.endpoints: dict[str, bool] = {}
        self.service_ready: dict[str, bool] = {}
        self.node_names: set[str] = set()
        self._health_subscriptions: list[Any] = []

        self._init_topic("/local_state/odometry", "nav_msgs/msg/Odometry")
        self._init_topic("/fastlio/base_odometry", "nav_msgs/msg/Odometry")
        self._init_topic("/Odometry", "nav_msgs/msg/Odometry")
        self._init_topic("/safety/status", "std_msgs/msg/String")
        self._init_topic("/scan", "sensor_msgs/msg/LaserScan")
        self._init_topic("/map", "nav_msgs/msg/OccupancyGrid")
        self._init_topic("/global_costmap/costmap", "nav_msgs/msg/OccupancyGrid")
        self._init_topic("/local_costmap/costmap", "nav_msgs/msg/OccupancyGrid")
        self._init_topic("/localization_result", "geometry_msgs/msg/PoseWithCovarianceStamped")

        self._make_subscription(Odometry, "/local_state/odometry", self._on_topic("/local_state/odometry"), self._reliable_qos())
        self._make_subscription(Odometry, "/fastlio/base_odometry", self._on_topic("/fastlio/base_odometry"), self._reliable_qos())
        self._make_subscription(Odometry, "/Odometry", self._on_topic("/Odometry"), self._best_effort_qos())
        self._make_subscription(String, "/safety/status", self._on_topic("/safety/status"), self._reliable_qos())
        self._make_subscription(LaserScan, "/scan", self._on_topic("/scan"), self._best_effort_qos())
        if self.observe_heavy_topics:
            self._make_subscription(OccupancyGrid, "/map", self._on_topic("/map"), self._transient_qos())
            self._make_subscription(
                OccupancyGrid,
                "/global_costmap/costmap",
                self._on_topic("/global_costmap/costmap"),
                self._reliable_qos(),
            )
            self._make_subscription(
                OccupancyGrid,
                "/local_costmap/costmap",
                self._on_topic("/local_costmap/costmap"),
                self._reliable_qos(),
            )
        self._make_subscription(PoseWithCovarianceStamped, "/localization_result", self._on_topic("/localization_result"), self._best_effort_qos())
        self._make_subscription(TFMessage, "/tf", self._on_tf, self._tf_qos(ReliabilityPolicy.RELIABLE))

        self.create_timer(self.write_period_sec, self.write_snapshot)

    def _make_subscription(self, msg_type: Any, topic: str, callback: Any, qos: QoSProfile) -> None:
        try:
            self._health_subscriptions.append(self.create_subscription(msg_type, topic, callback, qos))
        except Exception as exc:
            self.get_logger().warning(f"failed to subscribe {topic}: {exc}")

    def _init_topic(self, topic: str, type_name: str) -> None:
        self.topics[topic] = {
            "type": type_name,
            "publishers": 0,
            "subscriptions": 0,
            "last_received_at": None,
            "last_stamp_sec": None,
            "last_age_sec": None,
            "last_frame_id": "",
            "message_count": 0,
        }

    @staticmethod
    def _best_effort_qos() -> QoSProfile:
        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        qos.durability = DurabilityPolicy.VOLATILE
        return qos

    @staticmethod
    def _reliable_qos() -> QoSProfile:
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.VOLATILE
        return qos

    @staticmethod
    def _transient_qos() -> QoSProfile:
        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        return qos

    @staticmethod
    def _tf_qos(reliability: ReliabilityPolicy) -> QoSProfile:
        qos = QoSProfile(depth=100)
        qos.reliability = reliability
        qos.durability = DurabilityPolicy.VOLATILE
        return qos

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1.0e-9

    def _on_topic(self, topic: str) -> Any:
        def callback(msg: Any) -> None:
            now = self._now_sec()
            item = self.topics[topic]
            item["message_count"] += 1
            item["last_received_at"] = time.time()
            header = getattr(msg, "header", None)
            if header is not None:
                item["last_stamp_sec"] = _stamp_to_sec(header.stamp)
                item["last_age_sec"] = now - item["last_stamp_sec"]
                item["last_frame_id"] = str(getattr(header, "frame_id", "") or "")
            else:
                item["last_stamp_sec"] = None
                item["last_age_sec"] = 0.0
                item["last_frame_id"] = ""

        return callback

    def _on_tf(self, msg: TFMessage) -> None:
        now = self._now_sec()
        received_wall = time.time()
        for transform in msg.transforms:
            parent = _strip_slash(transform.header.frame_id)
            child = _strip_slash(transform.child_frame_id)
            if not parent or not child:
                continue
            stamp_sec = _stamp_to_sec(transform.header.stamp)
            self.tf_edges[f"{parent}->{child}"] = {
                "parent": parent,
                "child": child,
                "last_stamp_sec": stamp_sec,
                "last_age_sec": now - stamp_sec,
                "last_received_at": received_wall,
            }

    def _refresh_graph(self) -> None:
        now = time.monotonic()
        if now - self.graph_last_checked < self.graph_period_sec:
            return
        self.graph_last_checked = now

        self.node_names = {
            _full_node_name(name, namespace)
            for name, namespace in self.get_node_names_and_namespaces()
        }

        for topic, item in self.topics.items():
            try:
                item["publishers"] = len(self.get_publishers_info_by_topic(topic))
                item["subscriptions"] = len(self.get_subscriptions_info_by_topic(topic))
            except Exception:
                item["publishers"] = 0
                item["subscriptions"] = 0

        try:
            service_names = {name for name, types in self.get_service_names_and_types() if types}
        except Exception:
            service_names = set()

        self.service_ready = {
            "/global_localization/trigger": "/global_localization/trigger" in service_names,
            "/trigger_grid_search_localization": "/trigger_grid_search_localization" in service_names,
            "/floor_manager/switch_floor": "/floor_manager/switch_floor" in service_names,
            "/robot_localization_bridge/force_accept_next_localization": (
                "/robot_localization_bridge/force_accept_next_localization" in service_names
            ),
        }

        try:
            local_state_odom_publishers = self.get_publishers_info_by_topic("/local_state/odometry")
            fastlio_base_subscribers = self.get_subscriptions_info_by_topic("/fastlio/base_odometry")
            tf_publishers = self.get_publishers_info_by_topic("/tf")
        except Exception:
            local_state_odom_publishers = []
            fastlio_base_subscribers = []
            tf_publishers = []

        self.endpoints = {
            "robot_local_state_node": "/robot_local_state" in self.node_names,
            "robot_local_state_odom_pub": any(
                info.node_name == "robot_local_state" for info in local_state_odom_publishers
            ),
            "robot_local_state_fastlio_sub": any(
                info.node_name == "robot_local_state" for info in fastlio_base_subscribers
            ),
            "robot_localization_bridge_node": "/robot_localization_bridge" in self.node_names,
            "robot_localization_bridge_tf_pub": any(
                info.node_name == "robot_localization_bridge" for info in tf_publishers
            ),
        }

    def _refresh_ages(self) -> None:
        ros_now = self._now_sec()
        wall_now = time.time()
        for item in self.topics.values():
            stamp_sec = item.get("last_stamp_sec")
            received_at = item.get("last_received_at")
            if stamp_sec is not None:
                item["last_age_sec"] = ros_now - float(stamp_sec)
            elif received_at is not None:
                item["last_age_sec"] = wall_now - float(received_at)
        for item in self.tf_edges.values():
            stamp_sec = item.get("last_stamp_sec")
            if stamp_sec is not None:
                item["last_age_sec"] = ros_now - float(stamp_sec)

    def _topic_fresh(self, topic: str, max_age_sec: float) -> bool:
        item = self.topics.get(topic, {})
        if item.get("last_age_sec") is None:
            return False
        age = float(item.get("last_age_sec") or math.inf)
        return item.get("publishers", 0) > 0 and -0.25 <= age <= max_age_sec

    def _tf_fresh(self, edge: str, max_age_sec: float) -> bool:
        item = self.tf_edges.get(edge, {})
        if item.get("last_age_sec") is None:
            return False
        age = float(item.get("last_age_sec") or math.inf)
        return -0.25 <= age <= max_age_sec

    def _summary(self) -> dict[str, Any]:
        local_state_endpoint = (
            self.endpoints.get("robot_local_state_node", False)
            and self.endpoints.get("robot_local_state_odom_pub", False)
        )
        local_state_fastlio_endpoint = local_state_endpoint and self.endpoints.get(
            "robot_local_state_fastlio_sub", False
        )
        local_odom_fresh = self._topic_fresh("/local_state/odometry", self.odom_fresh_sec)
        odom_base_tf_fresh = self._tf_fresh("odom->base_link", self.tf_fresh_sec)
        map_odom_tf_ready = self._tf_fresh("map->odom", self.map_tf_fresh_sec)
        bridge_endpoint = (
            self.endpoints.get("robot_localization_bridge_node", False)
            and self.endpoints.get("robot_localization_bridge_tf_pub", False)
            and self.service_ready.get("/robot_localization_bridge/force_accept_next_localization", False)
        )
        return {
            "local_state_endpoint_ready": local_state_endpoint,
            "local_state_fastlio_endpoint_ready": local_state_fastlio_endpoint,
            "local_state_ready": local_state_endpoint and local_odom_fresh and odom_base_tf_fresh,
            "local_odom_fresh": local_odom_fresh,
            "odom_base_tf_fresh": odom_base_tf_fresh,
            "map_odom_tf_ready": map_odom_tf_ready,
            "localization_bridge_endpoint_ready": bridge_endpoint,
            "safety_status_fresh": self._topic_fresh("/safety/status", self.topic_fresh_sec),
            "local_scan_fresh": self._topic_fresh("/scan", self.scan_fresh_sec),
            "local_costmap_fresh": self._topic_fresh("/local_costmap/costmap", self.topic_fresh_sec),
            "global_costmap_fresh": self._topic_fresh("/global_costmap/costmap", self.topic_fresh_sec),
            "map_fresh": _bool_topic(self.topics.get("/map", {})),
            "global_localization_trigger_service": self.service_ready.get("/global_localization/trigger", False),
            "isaac_grid_search_trigger_service": self.service_ready.get(
                "/trigger_grid_search_localization", False
            ),
            "floor_switch_service": self.service_ready.get("/floor_manager/switch_floor", False),
        }

    def snapshot(self) -> dict[str, Any]:
        self._refresh_graph()
        self._refresh_ages()
        return {
            "schema": "njrh.runtime_health.v1",
            "updated_at": time.time(),
            "uptime_sec": max(0.0, time.monotonic() - self.started_monotonic),
            "topics": self.topics,
            "tf": self.tf_edges,
            "endpoints": self.endpoints,
            "services": self.service_ready,
            "nodes": sorted(self.node_names),
            "summary": self._summary(),
        }

    def write_snapshot(self) -> None:
        data = self.snapshot()
        self.output_path.parent.mkdir(parents=True, exist_ok=True)
        fd, tmp_name = tempfile.mkstemp(
            prefix=f".{self.output_path.name}.",
            suffix=".tmp",
            dir=str(self.output_path.parent),
            text=True,
        )
        try:
            with os.fdopen(fd, "w", encoding="utf-8") as file:
                json.dump(data, file, ensure_ascii=False, separators=(",", ":"))
                file.write("\n")
            os.replace(tmp_name, self.output_path)
        finally:
            try:
                os.unlink(tmp_name)
            except FileNotFoundError:
                pass


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--output",
        default=os.environ.get("NJRH_RUNTIME_HEALTH_FILE", "/tmp/njrh_runtime_health.json"),
    )
    parser.add_argument("--once", action="store_true")
    parser.add_argument("--once-spin-sec", type=float, default=1.0)
    args = parser.parse_args()

    rclpy.init()
    node = RuntimeHealthGuard(Path(args.output), once=args.once)
    try:
        if args.once:
            deadline = time.monotonic() + max(0.0, args.once_spin_sec)
            while rclpy.ok() and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.05)
            node.write_snapshot()
            return 0
        rclpy.spin(node)
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
