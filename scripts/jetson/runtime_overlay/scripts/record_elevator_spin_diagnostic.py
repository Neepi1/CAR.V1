#!/usr/bin/env python3
"""Read-only, bounded recorder for segmented elevator spin diagnosis.

The recorder intentionally keeps one ROS participant, never publishes, and
reduces LaserScan messages to a few geometric counters instead of writing the
range array.  It is normally launched by record_elevator_spin_chain_ssh.sh.
"""

from __future__ import annotations

import argparse
import csv
import fcntl
import json
import math
import os
import signal
import sys
import time
from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List, Optional, TextIO, Tuple

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from ranger_msgs.msg import MotionState
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from rclpy.time import Time
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import String, UInt8
from tf2_ros import Buffer, TransformException, TransformListener


COMMAND_TOPICS: Tuple[Tuple[str, str], ...] = (
    ("raw", "/cmd_vel_nav_raw"),
    ("nav", "/cmd_vel_nav"),
    ("collision", "/cmd_vel_collision_checked"),
    ("safe", "/cmd_vel_safe"),
    ("final", "/cmd_vel"),
)
COMMAND_TOPIC_NAMES = tuple(topic for _label, topic in COMMAND_TOPICS)
READINESS_TOPICS = (
    "/wheel/odom",
    "/local_state/odometry",
    "/scan",
    "/ranger_base/status",
)
SPIN_THRESHOLD_RADPS = 0.05
RAW_STALE_SEC = 0.30
SCAN_PROCESS_PERIOD_SEC = 0.20

# These are the frozen field values in runtime_overlay/config/nav2.yaml.  They
# are emitted in metadata and used only for observation, never for admission.
PADDED_FOOTPRINT = (-0.39, 0.39, -0.28, 0.28)
STOP_ZONE = (-0.30, 0.42, -0.22, 0.22)
SLOW_ZONE = (-0.35, 0.85, -0.45, 0.45)
STOP_ZONE_MAX_POINTS = 8


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds")


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def angle_delta(current: float, previous: float) -> float:
    return math.atan2(math.sin(current - previous), math.cos(current - previous))


def finite_or_none(value: Any) -> Optional[float]:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def json_object(text: str) -> Dict[str, Any]:
    try:
        value = json.loads(text)
    except (TypeError, ValueError, json.JSONDecodeError):
        return {"text": text}
    return value if isinstance(value, dict) else {"value": value}


def inside_box(x: float, y: float, box: Tuple[float, float, float, float]) -> bool:
    xmin, xmax, ymin, ymax = box
    return xmin <= x <= xmax and ymin <= y <= ymax


def clearance_from_box(
    x: float, y: float, box: Tuple[float, float, float, float]
) -> float:
    xmin, xmax, ymin, ymax = box
    dx = max(xmin - x, 0.0, x - xmax)
    dy = max(ymin - y, 0.0, y - ymax)
    return math.hypot(dx, dy)


class UnwrappedYaw:
    def __init__(self) -> None:
        self.previous: Optional[float] = None
        self.value: Optional[float] = None

    def update(self, yaw: float) -> float:
        if self.previous is None:
            self.previous = yaw
            self.value = yaw
            return yaw
        assert self.value is not None
        self.value += angle_delta(yaw, self.previous)
        self.previous = yaw
        return self.value


@dataclass
class TimedTwist:
    elapsed: float
    x: float
    y: float
    z: float


@dataclass
class SpinSegment:
    start_s: float
    sign: int
    start_wheel_yaw: Optional[float]
    start_local_yaw: Optional[float]
    start_imu_yaw: float
    end_s: Optional[float] = None
    end_reason: str = ""
    end_wheel_yaw: Optional[float] = None
    end_local_yaw: Optional[float] = None
    end_imu_yaw: Optional[float] = None
    max_footprint_points: int = 0
    max_stop_zone_points: int = 0
    max_slow_zone_points: int = 0
    min_footprint_clearance_m: Optional[float] = None


class ElevatorSpinRecorder(Node):
    def __init__(self, output_dir: Path, sample_hz: float) -> None:
        super().__init__("elevator_spin_diagnostic_recorder")
        self.output_dir = output_dir
        self.start_monotonic = time.monotonic()
        self.stop_requested = False
        self.counts: Dict[str, int] = {}
        self.latest_commands: Dict[str, TimedTwist] = {}
        self.command_events: List[Tuple[float, str, float, float, float]] = []
        self.wheel_yaw = UnwrappedYaw()
        self.local_yaw = UnwrappedYaw()
        self.wheel_wz: Optional[float] = None
        self.local_wz: Optional[float] = None
        self.wheel_elapsed: Optional[float] = None
        self.local_elapsed: Optional[float] = None
        self.imu_wz: Optional[float] = None
        self.imu_elapsed: Optional[float] = None
        self.imu_yaw = 0.0
        self.imu_last_time: Optional[float] = None
        self.imu_last_wz: Optional[float] = None
        self.motion_mode: Optional[int] = None
        self.motion_state_elapsed: Optional[float] = None
        self.ranger_status: Dict[str, Any] = {}
        self.safety_status: Dict[str, Any] = {}
        self.bypass_status: Dict[str, Any] = {}
        self.elevator_progress: Optional[int] = None
        self.scan_metrics: Dict[str, Any] = {}
        self.last_scan_processed = -1e9
        self.scan_tf_errors = 0
        self.map_pose: Dict[str, Any] = {}
        self.raw_spin_open: Optional[SpinSegment] = None
        self.raw_spin_segments: List[SpinSegment] = []
        self.readiness_missing: List[str] = list(READINESS_TOPICS)

        self.events_file: TextIO = (output_dir / "events.jsonl").open(
            "w", encoding="utf-8", buffering=1
        )
        self.samples_file: TextIO = (output_dir / "samples.csv").open(
            "w", newline="", encoding="utf-8"
        )
        self.samples_writer = csv.DictWriter(
            self.samples_file, fieldnames=self._sample_fields()
        )
        self.samples_writer.writeheader()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=False)

        best_effort = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=20,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        for _label, topic in COMMAND_TOPICS:
            self.create_subscription(
                Twist,
                topic,
                lambda message, topic=topic: self._on_command(topic, message),
                best_effort,
            )
        self.create_subscription(Odometry, "/wheel/odom", self._on_wheel, best_effort)
        self.create_subscription(
            Odometry, "/local_state/odometry", self._on_local, best_effort
        )
        self.create_subscription(
            Imu, "/lidar_imu_bias_corrected", self._on_imu, best_effort
        )
        self.create_subscription(
            MotionState, "/motion_state", self._on_motion_state, best_effort
        )
        self.create_subscription(LaserScan, "/scan", self._on_scan, best_effort)
        self.create_subscription(
            String, "/ranger_base/status", self._on_ranger_status, best_effort
        )
        self.create_subscription(
            String, "/safety/status", self._on_safety_status, best_effort
        )
        self.create_subscription(
            String,
            "/ranger_mini3/elevator_entry_collision_bypass",
            self._on_bypass,
            best_effort,
        )
        self.create_subscription(
            UInt8,
            "/ranger_mini3/nav_elevator_scoped_progress_state",
            self._on_progress,
            best_effort,
        )
        self.create_timer(1.0 / sample_hz, self._write_sample)
        self.create_timer(0.10, self._housekeeping)
        self.create_timer(0.50, self._flush)

    def elapsed(self) -> float:
        return time.monotonic() - self.start_monotonic

    def _increment(self, topic: str) -> None:
        self.counts[topic] = self.counts.get(topic, 0) + 1

    def _event(self, kind: str, **values: Any) -> None:
        payload = {
            "elapsed_s": round(self.elapsed(), 6),
            "wall_time_utc": utc_now(),
            "kind": kind,
            **values,
        }
        self.events_file.write(json.dumps(payload, ensure_ascii=False) + "\n")

    def _on_command(self, topic: str, message: Twist) -> None:
        now = self.elapsed()
        command = TimedTwist(
            elapsed=now,
            x=float(message.linear.x),
            y=float(message.linear.y),
            z=float(message.angular.z),
        )
        self.latest_commands[topic] = command
        self.command_events.append((now, topic, command.x, command.y, command.z))
        self._increment(topic)
        self._event(
            "twist",
            topic=topic,
            linear_x=command.x,
            linear_y=command.y,
            angular_z=command.z,
        )
        if topic == "/cmd_vel_nav_raw":
            self._update_raw_spin(command)

    def _update_raw_spin(self, command: TimedTwist) -> None:
        active = abs(command.z) >= SPIN_THRESHOLD_RADPS
        sign = 1 if command.z > 0.0 else -1
        if active and self.raw_spin_open is None:
            self._start_raw_spin(command.elapsed, sign)
        elif active and self.raw_spin_open is not None and sign != self.raw_spin_open.sign:
            self._finish_raw_spin(command.elapsed, "sign_change")
            self._start_raw_spin(command.elapsed, sign)
        elif not active and self.raw_spin_open is not None:
            self._finish_raw_spin(command.elapsed, "raw_zero")

    def _start_raw_spin(self, now: float, sign: int) -> None:
        self.raw_spin_open = SpinSegment(
            start_s=now,
            sign=sign,
            start_wheel_yaw=self.wheel_yaw.value,
            start_local_yaw=self.local_yaw.value,
            start_imu_yaw=self.imu_yaw,
        )
        self._event("raw_spin_started", sign=sign)

    def _finish_raw_spin(self, now: float, reason: str) -> None:
        segment = self.raw_spin_open
        if segment is None:
            return
        segment.end_s = now
        segment.end_reason = reason
        segment.end_wheel_yaw = self.wheel_yaw.value
        segment.end_local_yaw = self.local_yaw.value
        segment.end_imu_yaw = self.imu_yaw
        self.raw_spin_segments.append(segment)
        self.raw_spin_open = None
        self._event("raw_spin_finished", reason=reason)

    def _on_wheel(self, message: Odometry) -> None:
        now = self.elapsed()
        orientation = message.pose.pose.orientation
        self.wheel_yaw.update(
            yaw_from_quaternion(
                orientation.x, orientation.y, orientation.z, orientation.w
            )
        )
        self.wheel_wz = float(message.twist.twist.angular.z)
        self.wheel_elapsed = now
        self._increment("/wheel/odom")

    def _on_local(self, message: Odometry) -> None:
        now = self.elapsed()
        orientation = message.pose.pose.orientation
        self.local_yaw.update(
            yaw_from_quaternion(
                orientation.x, orientation.y, orientation.z, orientation.w
            )
        )
        self.local_wz = float(message.twist.twist.angular.z)
        self.local_elapsed = now
        self._increment("/local_state/odometry")

    def _on_imu(self, message: Imu) -> None:
        now = self.elapsed()
        wz = float(message.angular_velocity.z)
        if self.imu_last_time is not None and self.imu_last_wz is not None:
            dt = now - self.imu_last_time
            if 0.0 < dt <= 0.10:
                self.imu_yaw += 0.5 * (self.imu_last_wz + wz) * dt
        self.imu_last_time = now
        self.imu_last_wz = wz
        self.imu_wz = wz
        self.imu_elapsed = now
        self._increment("/lidar_imu_bias_corrected")

    def _on_motion_state(self, message: MotionState) -> None:
        mode = int(message.motion_mode)
        if mode != self.motion_mode:
            self._event("motion_mode_changed", previous=self.motion_mode, current=mode)
        self.motion_mode = mode
        self.motion_state_elapsed = self.elapsed()
        self._increment("/motion_state")

    def _on_ranger_status(self, message: String) -> None:
        value = json_object(message.data)
        if value != self.ranger_status:
            self._event("ranger_status", value=value)
        self.ranger_status = value
        self._increment("/ranger_base/status")

    def _on_safety_status(self, message: String) -> None:
        value = json_object(message.data)
        if value != self.safety_status:
            self._event("safety_status", value=value)
        self.safety_status = value
        self._increment("/safety/status")

    def _on_bypass(self, message: String) -> None:
        value = json_object(message.data)
        if value != self.bypass_status:
            self._event("elevator_collision_bypass", value=value)
        self.bypass_status = value
        self._increment("/ranger_mini3/elevator_entry_collision_bypass")

    def _on_progress(self, message: UInt8) -> None:
        value = int(message.data)
        if value != self.elevator_progress:
            self._event(
                "elevator_scoped_progress",
                previous=self.elevator_progress,
                current=value,
            )
        self.elevator_progress = value
        self._increment("/ranger_mini3/nav_elevator_scoped_progress_state")

    def _on_scan(self, message: LaserScan) -> None:
        self._increment("/scan")
        now = self.elapsed()
        if now - self.last_scan_processed < SCAN_PROCESS_PERIOD_SEC:
            return
        self.last_scan_processed = now
        frame = message.header.frame_id or "lidar_level_link"
        try:
            transform = self.tf_buffer.lookup_transform("base_link", frame, Time())
        except TransformException as exc:
            self.scan_tf_errors += 1
            if self.scan_tf_errors <= 3:
                self._event("scan_tf_unavailable", frame=frame, error=str(exc))
            return
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        yaw = yaw_from_quaternion(rotation.x, rotation.y, rotation.z, rotation.w)
        cosine = math.cos(yaw)
        sine = math.sin(yaw)
        finite = 0
        footprint = 0
        stop = 0
        slow = 0
        minimum_clearance: Optional[float] = None
        angle = float(message.angle_min)
        range_min = float(message.range_min)
        range_max = float(message.range_max)
        for distance in message.ranges:
            value = float(distance)
            if math.isfinite(value) and range_min <= value <= range_max:
                scan_x = value * math.cos(angle)
                scan_y = value * math.sin(angle)
                base_x = float(translation.x) + cosine * scan_x - sine * scan_y
                base_y = float(translation.y) + sine * scan_x + cosine * scan_y
                finite += 1
                if inside_box(base_x, base_y, PADDED_FOOTPRINT):
                    footprint += 1
                if inside_box(base_x, base_y, STOP_ZONE):
                    stop += 1
                if inside_box(base_x, base_y, SLOW_ZONE):
                    slow += 1
                clearance = clearance_from_box(base_x, base_y, PADDED_FOOTPRINT)
                if minimum_clearance is None or clearance < minimum_clearance:
                    minimum_clearance = clearance
            angle += float(message.angle_increment)
        self.scan_metrics = {
            "elapsed": now,
            "frame": frame,
            "finite": finite,
            "footprint": footprint,
            "stop": stop,
            "slow": slow,
            "minimum_footprint_clearance_m": minimum_clearance,
        }
        if self.raw_spin_open is not None:
            segment = self.raw_spin_open
            segment.max_footprint_points = max(segment.max_footprint_points, footprint)
            segment.max_stop_zone_points = max(segment.max_stop_zone_points, stop)
            segment.max_slow_zone_points = max(segment.max_slow_zone_points, slow)
            if minimum_clearance is not None:
                if (
                    segment.min_footprint_clearance_m is None
                    or minimum_clearance < segment.min_footprint_clearance_m
                ):
                    segment.min_footprint_clearance_m = minimum_clearance
        self._event(
            "scan_geometry",
            frame=frame,
            finite_points=finite,
            footprint_points=footprint,
            stop_zone_points=stop,
            slow_zone_points=slow,
            minimum_footprint_clearance_m=minimum_clearance,
        )

    @staticmethod
    def _sample_fields() -> List[str]:
        fields = ["elapsed_s", "wall_time_utc"]
        for label, _topic in COMMAND_TOPICS:
            fields.extend(
                [
                    f"{label}_x",
                    f"{label}_y",
                    f"{label}_z",
                    f"{label}_age_s",
                ]
            )
        fields.extend(
            [
                "motion_mode",
                "motion_state_age_s",
                "ranger_desired_mode",
                "ranger_actual_mode",
                "ranger_mode_changing",
                "wheel_wz",
                "wheel_yaw_unwrapped_rad",
                "wheel_age_s",
                "local_wz",
                "local_yaw_unwrapped_rad",
                "local_age_s",
                "imu_wz",
                "imu_yaw_integral_rad",
                "imu_age_s",
                "map_x",
                "map_y",
                "map_yaw_rad",
                "map_tf_age_s",
                "scan_frame",
                "scan_finite_points",
                "scan_footprint_points",
                "scan_stop_zone_points",
                "scan_slow_zone_points",
                "scan_min_footprint_clearance_m",
                "scan_age_s",
                "elevator_progress",
                "safety_status",
                "elevator_collision_bypass",
            ]
        )
        return fields

    def _command_values(self, topic: str, now: float) -> Tuple[Any, Any, Any, Any]:
        command = self.latest_commands.get(topic)
        if command is None:
            return None, None, None, None
        return command.x, command.y, command.z, now - command.elapsed

    def _update_map_pose(self) -> None:
        try:
            transform = self.tf_buffer.lookup_transform("map", "base_link", Time())
        except TransformException:
            return
        stamp = transform.header.stamp
        stamp_sec = float(stamp.sec) + float(stamp.nanosec) * 1e-9
        now_ros = self.get_clock().now().nanoseconds * 1e-9
        rotation = transform.transform.rotation
        self.map_pose = {
            "x": float(transform.transform.translation.x),
            "y": float(transform.transform.translation.y),
            "yaw": yaw_from_quaternion(
                rotation.x, rotation.y, rotation.z, rotation.w
            ),
            "age": max(0.0, now_ros - stamp_sec) if stamp_sec > 0.0 else None,
        }

    @staticmethod
    def _status_mode(status: Dict[str, Any], key: str) -> Any:
        value = status.get(key, {})
        return value.get("code") if isinstance(value, dict) else value

    def _write_sample(self) -> None:
        now = self.elapsed()
        self._update_map_pose()
        row: Dict[str, Any] = {"elapsed_s": now, "wall_time_utc": utc_now()}
        for label, topic in COMMAND_TOPICS:
            x, y, z, age = self._command_values(topic, now)
            row.update(
                {
                    f"{label}_x": x,
                    f"{label}_y": y,
                    f"{label}_z": z,
                    f"{label}_age_s": age,
                }
            )
        actual = self.ranger_status.get("actual_motion_mode", {})
        row.update(
            {
                "motion_mode": self.motion_mode,
                "motion_state_age_s": None
                if self.motion_state_elapsed is None
                else now - self.motion_state_elapsed,
                "ranger_desired_mode": self._status_mode(
                    self.ranger_status, "desired_motion_mode"
                ),
                "ranger_actual_mode": self._status_mode(
                    self.ranger_status, "actual_motion_mode"
                ),
                "ranger_mode_changing": actual.get("mode_changing")
                if isinstance(actual, dict)
                else None,
                "wheel_wz": self.wheel_wz,
                "wheel_yaw_unwrapped_rad": self.wheel_yaw.value,
                "wheel_age_s": None
                if self.wheel_elapsed is None
                else now - self.wheel_elapsed,
                "local_wz": self.local_wz,
                "local_yaw_unwrapped_rad": self.local_yaw.value,
                "local_age_s": None
                if self.local_elapsed is None
                else now - self.local_elapsed,
                "imu_wz": self.imu_wz,
                "imu_yaw_integral_rad": self.imu_yaw,
                "imu_age_s": None
                if self.imu_elapsed is None
                else now - self.imu_elapsed,
                "map_x": self.map_pose.get("x"),
                "map_y": self.map_pose.get("y"),
                "map_yaw_rad": self.map_pose.get("yaw"),
                "map_tf_age_s": self.map_pose.get("age"),
                "scan_frame": self.scan_metrics.get("frame"),
                "scan_finite_points": self.scan_metrics.get("finite"),
                "scan_footprint_points": self.scan_metrics.get("footprint"),
                "scan_stop_zone_points": self.scan_metrics.get("stop"),
                "scan_slow_zone_points": self.scan_metrics.get("slow"),
                "scan_min_footprint_clearance_m": self.scan_metrics.get(
                    "minimum_footprint_clearance_m"
                ),
                "scan_age_s": None
                if "elapsed" not in self.scan_metrics
                else now - float(self.scan_metrics["elapsed"]),
                "elevator_progress": self.elevator_progress,
                "safety_status": json.dumps(
                    self.safety_status, ensure_ascii=False, separators=(",", ":")
                ),
                "elevator_collision_bypass": json.dumps(
                    self.bypass_status, ensure_ascii=False, separators=(",", ":")
                ),
            }
        )
        self.samples_writer.writerow(row)

    def _housekeeping(self) -> None:
        if self.raw_spin_open is None:
            return
        raw = self.latest_commands.get("/cmd_vel_nav_raw")
        if raw is None or self.elapsed() - raw.elapsed > RAW_STALE_SEC:
            self._finish_raw_spin(self.elapsed(), "raw_stale")

    def _flush(self) -> None:
        self.events_file.flush()
        self.samples_file.flush()

    def missing_readiness_topics(self) -> List[str]:
        return [topic for topic in READINESS_TOPICS if self.counts.get(topic, 0) == 0]

    def _topic_max_wz(self, topic: str, start_s: float, end_s: float) -> float:
        values = [
            abs(event[4])
            for event in self.command_events
            if event[1] == topic and start_s - 0.05 <= event[0] <= end_s + 0.15
        ]
        return max(values, default=0.0)

    @staticmethod
    def _delta(start: Optional[float], end: Optional[float]) -> Optional[float]:
        if start is None or end is None:
            return None
        return end - start

    @staticmethod
    def _degrees(value: Optional[float]) -> str:
        return "" if value is None else f"{math.degrees(value):.2f}"

    def _segment_diagnosis(self, segment: SpinSegment) -> str:
        assert segment.end_s is not None
        maxima = {
            label: self._topic_max_wz(topic, segment.start_s, segment.end_s)
            for label, topic in COMMAND_TOPICS
        }
        if maxima["nav"] < SPIN_THRESHOLD_RADPS:
            return "LOST_AT_VELOCITY_SMOOTHER"
        if maxima["collision"] < SPIN_THRESHOLD_RADPS:
            if (
                maxima["safe"] >= SPIN_THRESHOLD_RADPS
                and maxima["final"] >= SPIN_THRESHOLD_RADPS
            ):
                return "COLLISION_OUTPUT_BYPASSED_WHILE_FINAL_ROTATION_CONTINUED"
            return "SUPPRESSED_AT_COLLISION_MONITOR"
        if maxima["safe"] < SPIN_THRESHOLD_RADPS or maxima["final"] < SPIN_THRESHOLD_RADPS:
            return "SUPPRESSED_AT_ROBOT_SAFETY_OR_FINAL_OUTPUT"
        wheel_delta = self._delta(segment.start_wheel_yaw, segment.end_wheel_yaw)
        if wheel_delta is not None and abs(wheel_delta) < math.radians(3.0):
            return "COMMAND_REACHED_FINAL_BUT_CHASSIS_ROTATION_UNDER_3_DEG"
        if segment.max_stop_zone_points >= STOP_ZONE_MAX_POINTS:
            return "ROTATED_WITH_SCAN_STOP_ZONE_AT_OR_ABOVE_THRESHOLD"
        return "COMMAND_REACHED_FINAL_AND_CHASSIS_ROTATED"

    def _write_summary(self, duration: float) -> None:
        lines = [
            "# Elevator Spin Chain Diagnostic",
            "",
            f"- captured_at_utc: `{utc_now()}`",
            f"- duration_sec: `{duration:.3f}`",
            f"- raw_spin_segments: `{len(self.raw_spin_segments)}`",
            f"- readiness_missing: `{','.join(self.readiness_missing) if self.readiness_missing else 'none'}`",
            f"- scan_tf_errors: `{self.scan_tf_errors}`",
            "- recorder: `read-only; no command, goal, parameter, or service request`",
            "- scan_storage: `derived counters only; no LaserScan ranges or PointCloud2 saved`",
            f"- padded_footprint_xy: `{PADDED_FOOTPRINT}`",
            f"- collision_stop_zone_xy: `{STOP_ZONE}`; max_points=`{STOP_ZONE_MAX_POINTS}`",
            "",
            "## Topic counts",
            "",
            "| topic | messages |",
            "|---|---:|",
        ]
        expected = list(COMMAND_TOPIC_NAMES) + [
            "/wheel/odom",
            "/local_state/odometry",
            "/lidar_imu_bias_corrected",
            "/motion_state",
            "/ranger_base/status",
            "/safety/status",
            "/ranger_mini3/elevator_entry_collision_bypass",
            "/ranger_mini3/nav_elevator_scoped_progress_state",
            "/scan",
        ]
        for topic in expected:
            lines.append(f"| `{topic}` | {self.counts.get(topic, 0)} |")
        lines.extend(
            [
                "",
                "## Raw Nav2 spin segments",
                "",
                "A segment starts when `|/cmd_vel_nav_raw.angular.z| >= 0.05 rad/s` and ends on raw zero, sign change, or a 0.30 s raw-message gap.",
                "",
                "| # | start s | duration s | end | raw/nav/collision/safe/final max rad/s | wheel deg | local deg | IMU deg | footprint max | stop-zone max | min footprint clearance m | diagnosis |",
                "|---:|---:|---:|---|---|---:|---:|---:|---:|---:|---:|---|",
            ]
        )
        diagnoses: List[str] = []
        if not self.raw_spin_segments:
            lines.append(
                "| - | - | - | - | - | - | - | - | - | - | - | `NO_RAW_SPIN_SEGMENT_CAPTURED` |"
            )
        for index, segment in enumerate(self.raw_spin_segments, start=1):
            assert segment.end_s is not None
            maxima = [
                self._topic_max_wz(topic, segment.start_s, segment.end_s)
                for _label, topic in COMMAND_TOPICS
            ]
            diagnosis = self._segment_diagnosis(segment)
            diagnoses.append(diagnosis)
            clearance = (
                ""
                if segment.min_footprint_clearance_m is None
                else f"{segment.min_footprint_clearance_m:.3f}"
            )
            lines.append(
                "| {index} | {start:.3f} | {duration:.3f} | `{reason}` | `{chain}` | {wheel} | {local} | {imu} | {footprint} | {stop} | {clearance} | `{diagnosis}` |".format(
                    index=index,
                    start=segment.start_s,
                    duration=segment.end_s - segment.start_s,
                    reason=segment.end_reason,
                    chain="/".join(f"{value:.3f}" for value in maxima),
                    wheel=self._degrees(
                        self._delta(segment.start_wheel_yaw, segment.end_wheel_yaw)
                    ),
                    local=self._degrees(
                        self._delta(segment.start_local_yaw, segment.end_local_yaw)
                    ),
                    imu=self._degrees(
                        self._delta(segment.start_imu_yaw, segment.end_imu_yaw)
                    ),
                    footprint=segment.max_footprint_points,
                    stop=segment.max_stop_zone_points,
                    clearance=clearance,
                    diagnosis=diagnosis,
                )
            )
        lines.extend(
            [
                "",
                "## Quick reading",
                "",
                f"- segment_diagnoses: `{', '.join(diagnoses) if diagnoses else 'NO_RAW_SPIN_SEGMENT_CAPTURED'}`",
                "- If four rows each show about 45 degrees and all five command columns are nonzero, the segmentation originated at or above the controller, not in collision/safety/chassis delivery.",
                "- If raw/nav remain nonzero but collision becomes zero, inspect collision-monitor evidence and the same row's stop-zone count.",
                "- If collision is nonzero but safe/final becomes zero, inspect `safety_status` and Ranger mode events in `events.jsonl`.",
                f"- A stop-zone count of `{STOP_ZONE_MAX_POINTS}` or more is only evidence that the current `/scan` geometry met the configured collision-monitor point threshold; correlate it with the collision command before assigning cause.",
                "",
                "Files: `summary.md`, `samples.csv`, `events.jsonl`, `metadata.json`.",
            ]
        )
        (self.output_dir / "summary.md").write_text(
            "\n".join(lines) + "\n", encoding="utf-8"
        )

    def close_capture(self) -> None:
        self._finish_raw_spin(self.elapsed(), "capture_ended")
        self._flush()
        self.events_file.close()
        self.samples_file.close()
        self._write_summary(self.elapsed())


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Read-only elevator spin command/odom/scan diagnostic."
    )
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--duration-sec", type=float, default=120.0)
    parser.add_argument("--sample-hz", type=float, default=20.0)
    parser.add_argument("--warmup-sec", type=float, default=8.0)
    parser.add_argument("--label", default="elevator_spin")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if not 1.0 <= args.duration_sec <= 600.0:
        raise SystemExit("--duration-sec must be between 1 and 600")
    if not 2.0 <= args.sample_hz <= 50.0:
        raise SystemExit("--sample-hz must be between 2 and 50")
    if not 0.0 <= args.warmup_sec <= 15.0:
        raise SystemExit("--warmup-sec must be between 0 and 15")

    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    lock_file = Path("/tmp/njrh_elevator_spin_recorder.lock").open("w")
    try:
        fcntl.flock(lock_file.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        print("[elevator-spin] another recorder is already running", file=sys.stderr)
        return 3

    (output_dir / "recorder.pid").write_text(f"{os.getpid()}\n", encoding="utf-8")
    metadata = {
        "captured_at_utc": utc_now(),
        "label": args.label,
        "duration_sec": args.duration_sec,
        "sample_hz": args.sample_hz,
        "read_only": True,
        "scan_payload_saved": False,
        "pointcloud_subscribed": False,
        "padded_footprint": PADDED_FOOTPRINT,
        "stop_zone": STOP_ZONE,
        "stop_zone_max_points": STOP_ZONE_MAX_POINTS,
    }
    (output_dir / "metadata.json").write_text(
        json.dumps(metadata, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )

    rclpy.init()
    node = ElevatorSpinRecorder(output_dir, args.sample_hz)
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    def request_stop(_signum: int, _frame: Any) -> None:
        node.stop_requested = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    try:
        warmup_deadline = time.monotonic() + args.warmup_sec
        ready_since: Optional[float] = None
        while rclpy.ok() and not node.stop_requested and time.monotonic() < warmup_deadline:
            executor.spin_once(timeout_sec=0.05)
            missing = node.missing_readiness_topics()
            if missing:
                ready_since = None
            elif ready_since is None:
                ready_since = time.monotonic()
            elif time.monotonic() - ready_since >= 0.5:
                break
        node.readiness_missing = node.missing_readiness_topics()
        if node.readiness_missing:
            print(
                "[elevator-spin] WARNING missing readiness topics: "
                + ",".join(node.readiness_missing),
                flush=True,
            )
        print(
            "[elevator-spin] READY：现在从 App 触发乘梯；出现分段自旋后可按 Ctrl+C，或等待自动结束。",
            flush=True,
        )
        deadline = time.monotonic() + args.duration_sec
        while rclpy.ok() and not node.stop_requested and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.05)
    except KeyboardInterrupt:
        pass
    finally:
        node.close_capture()
        executor.remove_node(node)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        (output_dir / "completed.env").write_text(
            f"completed_at_utc={utc_now()}\n", encoding="utf-8"
        )
        fcntl.flock(lock_file.fileno(), fcntl.LOCK_UN)
        lock_file.close()
    print(f"[elevator-spin] container_report={output_dir}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
