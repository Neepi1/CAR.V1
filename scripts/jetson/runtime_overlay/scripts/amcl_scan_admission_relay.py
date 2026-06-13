#!/usr/bin/env python3
"""AMCL production scan admission relay.

This node relays /scan to /scan_amcl without restamping or modifying ranges.
It only admits scans that are recent enough and transformable into odom at the
original scan stamp, reducing AMCL MessageFilter pressure during startup.
"""

import json
import os
import time

import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
import tf2_ros


def _env_bool(name: str, default: bool) -> bool:
    value = os.environ.get(name)
    if value is None:
        return default
    return value.strip().lower() in {"1", "true", "yes", "on"}


def _env_float(name: str, default: float) -> float:
    try:
        return float(os.environ.get(name, str(default)))
    except (TypeError, ValueError):
        return default


class AmclScanAdmissionRelay(Node):
    def __init__(self) -> None:
        super().__init__("amcl_scan_admission_relay")
        numeric_param = ParameterDescriptor(dynamic_typing=True)
        self.input_topic = self.declare_parameter(
            "input_topic", os.environ.get("NJRH_AMCL_SCAN_INPUT_TOPIC", "/scan")
        ).value
        self.output_topic = self.declare_parameter(
            "output_topic", os.environ.get("NJRH_AMCL_SCAN_OUTPUT_TOPIC", "/scan_amcl")
        ).value
        self.status_topic = self.declare_parameter(
            "status_topic",
            os.environ.get("NJRH_AMCL_SCAN_ADMISSION_STATUS_TOPIC", "/amcl_scan_admission/status"),
        ).value
        self.target_frame = self.declare_parameter(
            "target_frame", os.environ.get("NJRH_AMCL_SCAN_TARGET_FRAME", "odom")
        ).value
        self.frame_required = self.declare_parameter(
            "frame_required", os.environ.get("NJRH_AMCL_SCAN_FRAME_REQUIRED", "lidar_level_link")
        ).value
        self.rate_hz = max(
            0.1,
            float(
                self.declare_parameter(
                    "rate_hz", _env_float("NJRH_AMCL_SCAN_RATE_HZ", 5.0), numeric_param
                ).value
            ),
        )
        self.max_age_ms = max(
            0.0,
            float(
                self.declare_parameter(
                    "max_age_ms", _env_float("NJRH_AMCL_SCAN_MAX_AGE_MS", 250.0), numeric_param
                ).value
            ),
        )
        self.wait_for_tf_timeout_ms = max(
            0.0,
            float(
                self.declare_parameter(
                    "wait_for_tf_timeout_ms",
                    _env_float("NJRH_AMCL_SCAN_WAIT_FOR_TF_TIMEOUT_MS", 20.0),
                    numeric_param,
                ).value
            ),
        )
        self.drop_if_tf_unavailable = bool(
            self.declare_parameter(
                "drop_if_tf_unavailable",
                _env_bool("NJRH_AMCL_SCAN_DROP_IF_TF_UNAVAILABLE", True),
            ).value
        )

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.pub = self.create_publisher(LaserScan, self.output_topic, qos_profile_sensor_data)
        self.status_pub = self.create_publisher(String, self.status_topic, 10)
        self.sub = self.create_subscription(
            LaserScan, self.input_topic, self._on_scan, qos_profile_sensor_data
        )
        self.status_timer = self.create_timer(1.0, self._publish_status)

        self.input_count = 0
        self.published_count = 0
        self.dropped_age_count = 0
        self.dropped_tf_count = 0
        self.dropped_rate_count = 0
        self.dropped_frame_count = 0
        self.last_publish_wall = 0.0
        self.last_status_wall = time.monotonic()
        self.previous_published_count = 0
        self.last_age_ms = -1.0
        self.last_frame_id = ""
        self.last_error = "none"

        self.get_logger().info(
            "AMCL scan admission relay input=%s output=%s rate_hz=%.2f max_age_ms=%.1f "
            "target_frame=%s frame_required=%s preserve_stamp=true"
            % (
                self.input_topic,
                self.output_topic,
                self.rate_hz,
                self.max_age_ms,
                self.target_frame,
                self.frame_required,
            )
        )

    def _stamp_age_ms(self, msg: LaserScan) -> float:
        stamp_sec = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1.0e-9
        return (self.get_clock().now().nanoseconds * 1.0e-9 - stamp_sec) * 1000.0

    def _on_scan(self, msg: LaserScan) -> None:
        self.input_count += 1
        self.last_frame_id = msg.header.frame_id
        self.last_age_ms = self._stamp_age_ms(msg)

        if self.frame_required and self.frame_required != "auto" and msg.header.frame_id != self.frame_required:
            self.dropped_frame_count += 1
            self.last_error = "AMCL_SCAN_FRAME_MISMATCH"
            return

        if self.max_age_ms > 0.0 and self.last_age_ms > self.max_age_ms:
            self.dropped_age_count += 1
            self.last_error = "AMCL_SCAN_STALE"
            return

        now_wall = time.monotonic()
        min_period = 1.0 / self.rate_hz
        if self.last_publish_wall > 0.0 and now_wall - self.last_publish_wall < min_period:
            self.dropped_rate_count += 1
            return

        if self.drop_if_tf_unavailable:
            try:
                ok = self.tf_buffer.can_transform(
                    self.target_frame,
                    msg.header.frame_id,
                    msg.header.stamp,
                    timeout=Duration(nanoseconds=int(self.wait_for_tf_timeout_ms * 1_000_000.0)),
                )
            except Exception as exc:  # noqa: BLE001 - status reports exact ROS/TF failure text.
                ok = False
                self.last_error = f"AMCL_SCAN_TF_UNAVAILABLE: {exc}"
            if not ok:
                self.dropped_tf_count += 1
                if self.last_error == "none":
                    self.last_error = "AMCL_SCAN_TF_UNAVAILABLE"
                return

        # Preserve the original header.stamp, frame_id, angle metadata, and ranges.
        self.pub.publish(msg)
        self.published_count += 1
        self.last_publish_wall = now_wall
        self.last_error = "none"

    def _publish_status(self) -> None:
        now_wall = time.monotonic()
        elapsed = max(1.0e-6, now_wall - self.last_status_wall)
        hz = (self.published_count - self.previous_published_count) / elapsed
        self.previous_published_count = self.published_count
        self.last_status_wall = now_wall
        payload = {
            "enabled": True,
            "input_topic": self.input_topic,
            "output_topic": self.output_topic,
            "target_frame": self.target_frame,
            "frame_id": self.last_frame_id,
            "input_count": self.input_count,
            "published_count": self.published_count,
            "dropped_age_count": self.dropped_age_count,
            "dropped_tf_count": self.dropped_tf_count,
            "dropped_rate_count": self.dropped_rate_count,
            "dropped_frame_count": self.dropped_frame_count,
            "last_age_ms": self.last_age_ms,
            "hz": hz,
            "preserve_stamp": True,
            "message_filter_drop_detected": False,
            "last_error": self.last_error,
        }
        msg = String()
        msg.data = json.dumps(payload, sort_keys=True)
        self.status_pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = AmclScanAdmissionRelay()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
