#!/usr/bin/env python3
"""Low-overhead recorder for Nav2-to-Ranger spin command handoff."""

from __future__ import annotations

import argparse
import csv
import json
import math
import signal
import time
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from sensor_msgs.msg import Imu
from std_msgs.msg import String


COMMAND_TOPICS = (
    "/cmd_vel_nav_raw",
    "/cmd_vel_nav",
    "/cmd_vel_collision_checked",
    "/cmd_vel",
)
SPIN_MODE = 2
SPIN_COMMAND_THRESHOLD_RADPS = 0.05
PHYSICAL_ROTATION_THRESHOLD_RAD = 0.03
READINESS_TOPICS = (
    "/wheel/odom",
    "/local_state/odometry",
    "/lidar_imu_bias_corrected",
    "/ranger_base/status",
)


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds")


def finite_or_none(value: Any) -> Optional[float]:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def csv_value(value: Any) -> Any:
    if value is None:
        return ""
    if isinstance(value, float):
        return f"{value:.9f}"
    return value


def angle_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def angle_delta(current: float, previous: float) -> float:
    return math.atan2(math.sin(current - previous), math.cos(current - previous))


def message_stamp_sec(message: Any) -> Optional[float]:
    header = getattr(message, "header", None)
    stamp = getattr(header, "stamp", None)
    if stamp is None:
        return None
    value = float(stamp.sec) + float(stamp.nanosec) * 1e-9
    return value if value > 0.0 else None


def json_object(data: str) -> Dict[str, Any]:
    try:
        parsed = json.loads(data)
    except (TypeError, ValueError, json.JSONDecodeError):
        return {}
    return parsed if isinstance(parsed, dict) else {}


@dataclass
class TimedTwist:
    elapsed: float
    linear_x: float
    linear_y: float
    angular_z: float


class UnwrappedYaw:
    def __init__(self) -> None:
        self.previous: Optional[float] = None
        self.unwrapped: Optional[float] = None

    def update(self, yaw: float) -> float:
        if self.previous is None:
            self.previous = yaw
            self.unwrapped = yaw
            return yaw
        assert self.unwrapped is not None
        self.unwrapped += angle_delta(yaw, self.previous)
        self.previous = yaw
        return self.unwrapped


class SpinCommandChainRecorder(Node):
    def __init__(self, output_dir: Path, sample_hz: float) -> None:
        super().__init__("predock_spin_command_chain_recorder")
        self.output_dir = output_dir
        self.start_monotonic = time.monotonic()
        self.stop_requested = False
        self.counts: Dict[str, int] = {}
        self.latest_commands: Dict[str, TimedTwist] = {}
        self.command_events: List[Tuple[float, str, float, float, float]] = []
        self.motion_mode: Optional[int] = None
        self.motion_mode_elapsed: Optional[float] = None
        self.ranger_status: Dict[str, Any] = {}
        self.spin_status: Dict[str, Any] = {}
        self.spin_intervals: List[Dict[str, Any]] = []
        self.open_spin_interval: Optional[Dict[str, Any]] = None
        self.readiness_missing: List[str] = list(READINESS_TOPICS)

        self.wheel_yaw = UnwrappedYaw()
        self.local_yaw = UnwrappedYaw()
        self.wheel_elapsed: Optional[float] = None
        self.local_elapsed: Optional[float] = None
        self.wheel_wz: Optional[float] = None
        self.local_wz: Optional[float] = None

        self.imu_elapsed: Optional[float] = None
        self.imu_wz: Optional[float] = None
        self.imu_integral = 0.0
        self.imu_last_stamp: Optional[float] = None
        self.imu_last_receive: Optional[float] = None
        self.imu_last_wz: Optional[float] = None

        self.command_file = (output_dir / "command_events.csv").open(
            "w", newline="", encoding="utf-8"
        )
        self.command_writer = csv.writer(self.command_file)
        self.command_writer.writerow(
            [
                "elapsed_s",
                "wall_time_utc",
                "topic",
                "linear_x",
                "linear_y",
                "angular_z",
                "interarrival_s",
                "motion_mode",
            ]
        )
        self.mode_file = (output_dir / "mode_events.csv").open(
            "w", newline="", encoding="utf-8"
        )
        self.mode_writer = csv.writer(self.mode_file)
        self.mode_writer.writerow(
            ["elapsed_s", "wall_time_utc", "source", "event", "detail"]
        )
        self.samples_file = (output_dir / "samples.csv").open(
            "w", newline="", encoding="utf-8"
        )
        self.samples_writer = csv.writer(self.samples_file)
        self.sample_fields = self._sample_fields()
        self.samples_writer.writerow(self.sample_fields)

        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=100,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        for topic in COMMAND_TOPICS:
            self.create_subscription(
                Twist,
                topic,
                lambda message, topic=topic: self._on_command(topic, message),
                qos,
            )
        self.create_subscription(Odometry, "/wheel/odom", self._on_wheel, qos)
        self.create_subscription(
            Odometry, "/local_state/odometry", self._on_local, qos
        )
        self.create_subscription(
            Imu, "/lidar_imu_bias_corrected", self._on_imu, qos
        )
        self.create_subscription(
            String, "/ranger_base/status", self._on_ranger_status, qos
        )
        self.create_subscription(
            String,
            "/local_state/spin_yaw_correction_status",
            self._on_spin_status,
            qos,
        )
        self.create_timer(1.0 / sample_hz, self._write_sample)
        self.create_timer(0.5, self._flush_files)

    def elapsed(self) -> float:
        return time.monotonic() - self.start_monotonic

    def _increment(self, topic: str) -> None:
        self.counts[topic] = self.counts.get(topic, 0) + 1

    def _on_command(self, topic: str, message: Twist) -> None:
        now = self.elapsed()
        previous = self.latest_commands.get(topic)
        interarrival = None if previous is None else now - previous.elapsed
        command = TimedTwist(
            elapsed=now,
            linear_x=float(message.linear.x),
            linear_y=float(message.linear.y),
            angular_z=float(message.angular.z),
        )
        self.latest_commands[topic] = command
        self.command_events.append(
            (now, topic, command.linear_x, command.linear_y, command.angular_z)
        )
        self.command_writer.writerow(
            [
                csv_value(now),
                utc_now(),
                topic,
                csv_value(command.linear_x),
                csv_value(command.linear_y),
                csv_value(command.angular_z),
                csv_value(interarrival),
                csv_value(self.motion_mode),
            ]
        )
        self._increment(topic)

    def _state_snapshot(self) -> Dict[str, Any]:
        return {
            "imu_integral_rad": self.imu_integral,
            "wheel_yaw_rad": self.wheel_yaw.unwrapped,
            "local_yaw_rad": self.local_yaw.unwrapped,
        }

    def _start_spin_interval(self, now: float) -> None:
        if self.open_spin_interval is not None:
            return
        self.open_spin_interval = {
            "start_s": now,
            "start_state": self._state_snapshot(),
        }
        self._write_mode_event("ranger_base/status", "spin_started", "actual_mode=2")

    def _finish_spin_interval(self, now: float, reason: str) -> None:
        if self.open_spin_interval is None:
            return
        self.open_spin_interval["end_s"] = now
        self.open_spin_interval["end_state"] = self._state_snapshot()
        self.open_spin_interval["end_reason"] = reason
        self.spin_intervals.append(self.open_spin_interval)
        self.open_spin_interval = None
        self._write_mode_event("ranger_base/status", "spin_ended", reason)

    def _update_motion_mode(self, mode: int, now: float) -> None:
        if mode != self.motion_mode:
            old_mode = self.motion_mode
            if old_mode == SPIN_MODE and mode != SPIN_MODE:
                self._finish_spin_interval(now, f"motion_mode={mode}")
            self.motion_mode = mode
            self.motion_mode_elapsed = now
            self._write_mode_event(
                "ranger_base/status", "mode_changed", f"{old_mode}->{mode}"
            )
            if mode == SPIN_MODE:
                self._start_spin_interval(now)
        else:
            self.motion_mode_elapsed = now

    def _on_wheel(self, message: Odometry) -> None:
        now = self.elapsed()
        q = message.pose.pose.orientation
        self.wheel_yaw.update(angle_from_quaternion(q.x, q.y, q.z, q.w))
        self.wheel_wz = float(message.twist.twist.angular.z)
        self.wheel_elapsed = now
        self._increment("/wheel/odom")

    def _on_local(self, message: Odometry) -> None:
        now = self.elapsed()
        q = message.pose.pose.orientation
        self.local_yaw.update(angle_from_quaternion(q.x, q.y, q.z, q.w))
        self.local_wz = float(message.twist.twist.angular.z)
        self.local_elapsed = now
        self._increment("/local_state/odometry")

    def _on_imu(self, message: Imu) -> None:
        now = self.elapsed()
        stamp = message_stamp_sec(message)
        wz = float(message.angular_velocity.z)
        integration_time = stamp if stamp is not None else now
        previous_time = (
            self.imu_last_stamp if stamp is not None else self.imu_last_receive
        )
        if previous_time is not None and self.imu_last_wz is not None:
            dt = integration_time - previous_time
            if 0.0 < dt <= 0.1:
                self.imu_integral += 0.5 * (self.imu_last_wz + wz) * dt
        if stamp is not None:
            self.imu_last_stamp = stamp
        self.imu_last_receive = now
        self.imu_last_wz = wz
        self.imu_wz = wz
        self.imu_elapsed = now
        self._increment("/lidar_imu_bias_corrected")

    def _status_signature(self, status: Dict[str, Any]) -> Tuple[Any, ...]:
        desired = status.get("desired_motion_mode", {})
        actual = status.get("actual_motion_mode", {})
        if not isinstance(desired, dict):
            desired = {}
        if not isinstance(actual, dict):
            actual = {}
        return (
            status.get("state"),
            desired.get("code"),
            actual.get("code"),
            actual.get("mode_changing"),
            status.get("mode_aligned"),
        )

    def _on_ranger_status(self, message: String) -> None:
        now = self.elapsed()
        status = json_object(message.data)
        actual = status.get("actual_motion_mode", {})
        if isinstance(actual, dict):
            actual_code = actual.get("code")
            if isinstance(actual_code, (int, float)):
                self._update_motion_mode(int(actual_code), now)
        if self._status_signature(status) != self._status_signature(self.ranger_status):
            self._write_mode_event(
                "ranger_base/status",
                "status_changed",
                json.dumps(status, separators=(",", ":"), ensure_ascii=True),
            )
        self.ranger_status = status
        self._increment("/ranger_base/status")

    def _spin_signature(self, status: Dict[str, Any]) -> Tuple[Any, ...]:
        return (
            status.get("correction_active"),
            status.get("spin_command_seen"),
            status.get("zero_command_seen"),
            status.get("settle_ready"),
            status.get("completed_spin_count"),
            status.get("imu_fallback_count"),
        )

    def _on_spin_status(self, message: String) -> None:
        status = json_object(message.data)
        if self._spin_signature(status) != self._spin_signature(self.spin_status):
            self._write_mode_event(
                "spin_yaw_correction_status",
                "status_changed",
                json.dumps(status, separators=(",", ":"), ensure_ascii=True),
            )
        self.spin_status = status
        self._increment("/local_state/spin_yaw_correction_status")

    def _write_mode_event(self, source: str, event: str, detail: str) -> None:
        self.mode_writer.writerow(
            [csv_value(self.elapsed()), utc_now(), source, event, detail]
        )

    @staticmethod
    def _sample_fields() -> List[str]:
        fields = ["elapsed_s", "wall_time_utc", "motion_mode", "motion_age_s"]
        for label in ("raw", "smoothed", "collision", "final"):
            fields.extend(
                [
                    f"{label}_linear_x",
                    f"{label}_linear_y",
                    f"{label}_angular_z",
                    f"{label}_age_s",
                ]
            )
        fields.extend(
            [
                "ranger_state",
                "ranger_desired_mode",
                "ranger_actual_mode",
                "ranger_mode_changing",
                "ranger_mode_aligned",
                "wheel_wz",
                "wheel_yaw_unwrapped_rad",
                "wheel_age_s",
                "local_wz",
                "local_yaw_unwrapped_rad",
                "local_age_s",
                "imu_wz",
                "imu_integral_rad",
                "imu_age_s",
                "spin_correction_active",
                "spin_command_seen",
                "zero_command_seen",
                "settle_ready",
            ]
        )
        return fields

    def _command_sample(self, topic: str, now: float) -> List[Any]:
        command = self.latest_commands.get(topic)
        if command is None:
            return [None, None, None, None]
        return [
            command.linear_x,
            command.linear_y,
            command.angular_z,
            now - command.elapsed,
        ]

    def _write_sample(self) -> None:
        now = self.elapsed()
        desired = self.ranger_status.get("desired_motion_mode", {})
        actual = self.ranger_status.get("actual_motion_mode", {})
        if not isinstance(desired, dict):
            desired = {}
        if not isinstance(actual, dict):
            actual = {}
        row: List[Any] = [
            now,
            utc_now(),
            self.motion_mode,
            None if self.motion_mode_elapsed is None else now - self.motion_mode_elapsed,
        ]
        for topic in COMMAND_TOPICS:
            row.extend(self._command_sample(topic, now))
        row.extend(
            [
                self.ranger_status.get("state"),
                desired.get("code"),
                actual.get("code"),
                actual.get("mode_changing"),
                self.ranger_status.get("mode_aligned"),
                self.wheel_wz,
                self.wheel_yaw.unwrapped,
                None if self.wheel_elapsed is None else now - self.wheel_elapsed,
                self.local_wz,
                self.local_yaw.unwrapped,
                None if self.local_elapsed is None else now - self.local_elapsed,
                self.imu_wz,
                self.imu_integral,
                None if self.imu_elapsed is None else now - self.imu_elapsed,
                self.spin_status.get("correction_active"),
                self.spin_status.get("spin_command_seen"),
                self.spin_status.get("zero_command_seen"),
                self.spin_status.get("settle_ready"),
            ]
        )
        self.samples_writer.writerow([csv_value(value) for value in row])

    def _flush_files(self) -> None:
        self.command_file.flush()
        self.mode_file.flush()
        self.samples_file.flush()

    def missing_readiness_topics(self) -> List[str]:
        return [topic for topic in READINESS_TOPICS if self.counts.get(topic, 0) == 0]

    def close_capture(self) -> None:
        now = self.elapsed()
        self._finish_spin_interval(now, "capture_ended")
        self._flush_files()
        self.command_file.close()
        self.mode_file.close()
        self.samples_file.close()
        self._write_summary(now)

    def _topic_metrics(
        self, topic: str, start_s: float, end_s: float
    ) -> Dict[str, Any]:
        values = [
            event
            for event in self.command_events
            if event[1] == topic and start_s <= event[0] <= end_s
        ]
        if not values:
            return {"count": 0, "effective": 0, "max_wz": 0.0, "max_linear": 0.0}
        return {
            "count": len(values),
            "effective": sum(
                1 for event in values if abs(event[4]) >= SPIN_COMMAND_THRESHOLD_RADPS
            ),
            "max_wz": max(abs(event[4]) for event in values),
            "max_linear": max(math.hypot(event[2], event[3]) for event in values),
        }

    @staticmethod
    def _state_delta(interval: Dict[str, Any], key: str) -> Optional[float]:
        start = finite_or_none(interval["start_state"].get(key))
        end = finite_or_none(interval["end_state"].get(key))
        if start is None or end is None:
            return None
        return end - start

    def _classify_interval(
        self, during: Dict[str, Dict[str, Any]], pre: Dict[str, Dict[str, Any]], imu_delta: Optional[float]
    ) -> str:
        raw = during[COMMAND_TOPICS[0]]["max_wz"]
        smoothed = during[COMMAND_TOPICS[1]]["max_wz"]
        collision = during[COMMAND_TOPICS[2]]["max_wz"]
        final = during[COMMAND_TOPICS[3]]["max_wz"]
        if raw < SPIN_COMMAND_THRESHOLD_RADPS:
            if pre[COMMAND_TOPICS[0]]["max_wz"] >= SPIN_COMMAND_THRESHOLD_RADPS:
                return "RAW_SPIN_PULSE_NOT_SUSTAINED_AFTER_MODE_SWITCH"
            return "NO_EFFECTIVE_NAV_SPIN_COMMAND"
        if smoothed < SPIN_COMMAND_THRESHOLD_RADPS:
            return "LOST_AT_VELOCITY_SMOOTHER"
        if collision < SPIN_COMMAND_THRESHOLD_RADPS:
            return "SUPPRESSED_AT_COLLISION_MONITOR"
        if final < SPIN_COMMAND_THRESHOLD_RADPS:
            return "SUPPRESSED_AT_ROBOT_SAFETY"
        if imu_delta is not None and abs(imu_delta) < PHYSICAL_ROTATION_THRESHOLD_RAD:
            return "FINAL_SPIN_COMMAND_PRESENT_BUT_CHASSIS_DID_NOT_ROTATE"
        return "SPIN_COMMAND_AND_PHYSICAL_ROTATION_PRESENT"

    def _write_summary(self, duration: float) -> None:
        report = self.output_dir / "summary.md"
        lines = [
            "# Predock Spin Command Chain Capture",
            "",
            f"- captured_at_utc: `{utc_now()}`",
            f"- duration_sec: `{duration:.3f}`",
            f"- sample_topics: `{len(self.counts)}`",
            f"- spin_intervals: `{len(self.spin_intervals)}`",
            f"- readiness_missing_at_start: `{','.join(self.readiness_missing) if self.readiness_missing else 'none'}`",
            "- recorder_behavior: `read-only; no velocity or mode command is published`",
            "",
            "## Topic Counts",
            "",
            "| topic | messages |",
            "|---|---:|",
        ]
        expected_topics = list(COMMAND_TOPICS) + [
            "/wheel/odom",
            "/local_state/odometry",
            "/lidar_imu_bias_corrected",
            "/ranger_base/status",
            "/local_state/spin_yaw_correction_status",
        ]
        for topic in expected_topics:
            lines.append(f"| `{topic}` | {self.counts.get(topic, 0)} |")

        lines.extend(
            [
                "",
                "## SPINNING Intervals",
                "",
                "The command threshold is `0.05 rad/s`. The pre-window is the 0.5 s before Ranger reports SPINNING.",
                "",
                "| # | start_s | duration_s | raw pre/during | smooth during | collision during | final during | IMU delta | wheel delta | local delta | diagnosis |",
                "|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|",
            ]
        )
        if not self.spin_intervals:
            lines.append("| - | - | - | - | - | - | - | - | - | - | `NO_SPINNING_INTERVAL_CAPTURED` |")
        verdicts: List[str] = []
        for index, interval in enumerate(self.spin_intervals, start=1):
            start_s = float(interval["start_s"])
            end_s = float(interval["end_s"])
            pre = {
                topic: self._topic_metrics(topic, max(0.0, start_s - 0.5), start_s)
                for topic in COMMAND_TOPICS
            }
            during = {
                topic: self._topic_metrics(topic, start_s, end_s)
                for topic in COMMAND_TOPICS
            }
            imu_delta = self._state_delta(interval, "imu_integral_rad")
            wheel_delta = self._state_delta(interval, "wheel_yaw_rad")
            local_delta = self._state_delta(interval, "local_yaw_rad")
            diagnosis = self._classify_interval(during, pre, imu_delta)
            verdicts.append(diagnosis)
            lines.append(
                "| {index} | {start:.3f} | {duration:.3f} | {raw_pre:.3f}/{raw:.3f} | "
                "{smooth:.3f} | {collision:.3f} | {final:.3f} | {imu} | {wheel} | {local} | `{diagnosis}` |".format(
                    index=index,
                    start=start_s,
                    duration=end_s - start_s,
                    raw_pre=pre[COMMAND_TOPICS[0]]["max_wz"],
                    raw=during[COMMAND_TOPICS[0]]["max_wz"],
                    smooth=during[COMMAND_TOPICS[1]]["max_wz"],
                    collision=during[COMMAND_TOPICS[2]]["max_wz"],
                    final=during[COMMAND_TOPICS[3]]["max_wz"],
                    imu="" if imu_delta is None else f"{imu_delta:.4f}",
                    wheel="" if wheel_delta is None else f"{wheel_delta:.4f}",
                    local="" if local_delta is None else f"{local_delta:.4f}",
                    diagnosis=diagnosis,
                )
            )

        lines.extend(
            [
                "",
                "## Result",
                "",
                f"- diagnoses: `{', '.join(verdicts) if verdicts else 'NO_SPINNING_INTERVAL_CAPTURED'}`",
                "- `RAW_SPIN_PULSE_NOT_SUSTAINED_AFTER_MODE_SWITCH`: the trigger existed before mode feedback, but Nav2 did not keep sending it after the mode switch.",
                "- `LOST_AT_VELOCITY_SMOOTHER`: raw Nav2 yaw command existed, smoother output did not.",
                "- `SUPPRESSED_AT_COLLISION_MONITOR`: smoother command existed, collision output did not.",
                "- `SUPPRESSED_AT_ROBOT_SAFETY`: collision-checked command existed, final Ranger input did not.",
                "- `FINAL_SPIN_COMMAND_PRESENT_BUT_CHASSIS_DID_NOT_ROTATE`: the command reached Ranger, but IMU observed under 0.03 rad physical rotation.",
                "",
                "Files: `command_events.csv`, `mode_events.csv`, `samples.csv`.",
            ]
        )
        report.write_text("\n".join(lines) + "\n", encoding="utf-8")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Read-only capture of the Nav2-to-Ranger spin command chain."
    )
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--duration-sec", type=float, default=90.0)
    parser.add_argument("--sample-hz", type=float, default=20.0)
    parser.add_argument("--warmup-sec", type=float, default=8.0)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.duration_sec <= 0.0:
        raise SystemExit("--duration-sec must be > 0")
    if not 2.0 <= args.sample_hz <= 50.0:
        raise SystemExit("--sample-hz must be between 2 and 50")
    if not 0.0 <= args.warmup_sec <= 10.0:
        raise SystemExit("--warmup-sec must be between 0 and 10")

    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = SpinCommandChainRecorder(output_dir, args.sample_hz)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    warmup_deadline = time.monotonic() + args.warmup_sec

    def request_stop(_signum: int, _frame: Any) -> None:
        node.stop_requested = True

    signal.signal(signal.SIGTERM, request_stop)
    try:
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
                "[predock-spin-chain] WARNING missing feedback topics: "
                + ",".join(node.readiness_missing),
                flush=True,
            )
        print(
            "[predock-spin-chain] READY: trigger return-to-dock now; "
            "Ctrl+C finalizes early",
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

    print(f"[predock-spin-chain] summary: {output_dir / 'summary.md'}")
    print(f"[predock-spin-chain] report: {output_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
