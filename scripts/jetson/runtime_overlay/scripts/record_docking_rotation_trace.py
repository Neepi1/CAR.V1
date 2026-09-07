#!/usr/bin/env python3
"""Read-only recorder for the return-to-dock rotate-stop-rotate symptom.

The module deliberately keeps its analysis functions independent of ROS so a
synthetic trace can exercise the exact feedback-loop verdict in unit tests.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import signal
import sys
import time
import urllib.error
import urllib.request
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple

try:
    import fcntl
except ImportError:  # Windows runs the pure trace-analysis regression tests.
    fcntl = None  # type: ignore[assignment]


COMMAND_TOPICS: Dict[str, str] = {
    "nav_raw": "/cmd_vel_nav_raw",
    "nav_smoothed": "/cmd_vel_nav",
    "collision_checked": "/cmd_vel_collision_checked",
    "docking": "/cmd_vel_docking",
    "api": "/cmd_vel_api",
    "final": "/cmd_vel",
}

STRING_TOPICS: Dict[str, str] = {
    "ranger": "/ranger_base/status",
    "safety": "/safety/status",
    "docking_manager": "/docking/status",
    "spin_settle": "/local_state/spin_yaw_correction_status",
    "motion_state": "/motion_state",
    "localization_bridge": "/localization/bridge_status",
}

READINESS_TOPICS = (
    "/wheel/odom",
    "/local_state/odometry",
    "/ranger_base/status",
    "/safety/status",
)

ROTATION_THRESHOLD_RADPS = 0.025
COMMAND_THRESHOLD_RADPS = 0.020
EPISODE_SPLIT_GAP_SEC = 0.45
TWO_STAGE_MAX_GAP_SEC = 45.0

DOCKING_API_FIELDS = (
    "id",
    "state",
    "phase",
    "detail",
    "last_status",
    "failure_code",
    "last_error_code",
    "predock_nav_early_handoff",
    "predock_nav_handoff_detail",
    "dock_staging_handoff_ready",
    "predock_pose_verified",
    "predock_xy_ok",
    "predock_yaw_verified_by_nav2",
    "predock_yaw_align_active",
    "predock_yaw_align_attempted",
    "predock_yaw_aligned",
    "predock_yaw_align_succeeded",
    "predock_base_yaw_error_rad",
    "predock_contact_yaw_error_rad",
    "predock_yaw_align_initial_error_rad",
    "predock_yaw_align_final_error_rad",
    "predock_yaw_align_duration_sec",
    "predock_yaw_align_observed_yaw_motion_rad",
    "predock_yaw_align_detail",
    "predock_yaw_align_failure_code",
    "predock_lateral_align_active",
    "predock_lateral_aligned",
    "fine_bridge_settle_started",
    "fine_bridge_settle_complete",
    "fine_bridge_settle_remaining_translation_m",
    "fine_bridge_settle_remaining_yaw_rad",
    "fine_bridge_settle_failure_code",
    "fine_entry_checked",
    "fine_entry_ok",
    "fine_entry_base_yaw_error_rad",
    "fine_entry_contact_yaw_error_rad",
    "fine_entry_failure_code",
    "ordinary_final_yaw_align_active",
    "cmd_owner_conflict_detected",
    "final_yaw_align_blocked_by_docking",
    "docking_blocked_by_final_yaw_align",
)

NAVIGATION_API_FIELDS = (
    "id",
    "state",
    "phase",
    "detail",
    "goal_completion_policy",
    "nav2_goal_yaw_source",
    "nav2_succeeded",
    "position_reached",
    "yaw_align_required",
    "yaw_align_active",
    "final_yaw_align_requested",
    "final_yaw_align_attempted",
    "final_yaw_align_succeeded",
    "final_yaw_align_blocked",
    "final_yaw_align_blocked_reason",
    "final_yaw_align_initial_yaw_error_rad",
    "final_yaw_align_final_yaw_error_rad",
    "ordinary_final_yaw_align_active",
    "predock_yaw_align_active",
    "cmd_owner_conflict_detected",
)


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds")


def csv_value(value: Any) -> Any:
    if value is None:
        return ""
    if isinstance(value, float):
        return f"{value:.9f}"
    if isinstance(value, (dict, list, tuple)):
        return json.dumps(value, ensure_ascii=False, separators=(",", ":"), sort_keys=True)
    return value


def finite_float(value: Any) -> Optional[float]:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result if math.isfinite(result) else None


def angle_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def angle_delta(current: float, previous: float) -> float:
    return math.atan2(math.sin(current - previous), math.cos(current - previous))


def json_object(value: str) -> Dict[str, Any]:
    try:
        parsed = json.loads(value)
    except (TypeError, ValueError, json.JSONDecodeError):
        return {"raw": value}
    return parsed if isinstance(parsed, dict) else {"value": parsed}


def nested(data: Dict[str, Any], *path: str, default: Any = None) -> Any:
    current: Any = data
    for part in path:
        if not isinstance(current, dict):
            return default
        current = current.get(part)
    return default if current is None else current


def select_fields(data: Dict[str, Any], fields: Iterable[str]) -> Dict[str, Any]:
    return {field: data.get(field) for field in fields}


def resolve_api_token(proc_root: Path = Path("/proc")) -> Tuple[str, str]:
    environment_token = os.environ.get("ROBOT_API_TOKEN", "")
    if environment_token:
        return environment_token, "environment"

    discovered: List[str] = []
    try:
        process_dirs = list(proc_root.iterdir())
    except OSError:
        return "", "unavailable"
    for process_dir in process_dirs:
        if not process_dir.name.isdigit():
            continue
        try:
            command_line = (process_dir / "cmdline").read_bytes()
        except OSError:
            continue
        if b"robot_api_server_node" not in command_line:
            continue
        try:
            environment = (process_dir / "environ").read_bytes()
        except OSError:
            continue
        for entry in environment.split(b"\0"):
            if not entry.startswith(b"ROBOT_API_TOKEN="):
                continue
            token = entry.partition(b"=")[2].decode("utf-8", errors="strict")
            if token:
                discovered.append(token)
    unique_tokens = set(discovered)
    if len(unique_tokens) == 1:
        return unique_tokens.pop(), "robot_api_server_process"
    if len(unique_tokens) > 1:
        return "", "ambiguous_process_tokens"
    return "", "missing"


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


def _finish_episode(active: List[Dict[str, Any]], value_key: str) -> Optional[Dict[str, Any]]:
    if len(active) < 2:
        return None
    start = active[0]
    end = active[-1]
    episode = {
        "start_s": float(start["elapsed_s"]),
        "end_s": float(end["elapsed_s"]),
        "duration_s": max(0.0, float(end["elapsed_s"]) - float(start["elapsed_s"])),
        "sample_count": len(active),
        "max_abs_wz": max(abs(float(row[value_key])) for row in active),
    }
    start_yaw = finite_float(start.get("yaw_rad"))
    end_yaw = finite_float(end.get("yaw_rad"))
    episode["yaw_delta_rad"] = (
        None if start_yaw is None or end_yaw is None else end_yaw - start_yaw
    )
    return episode


def detect_active_episodes(
    records: Sequence[Dict[str, Any]],
    *,
    value_key: str,
    threshold: float,
    split_gap_sec: float = EPISODE_SPLIT_GAP_SEC,
) -> List[Dict[str, Any]]:
    active_records = [
        row
        for row in sorted(records, key=lambda item: float(item["elapsed_s"]))
        if abs(float(row.get(value_key, 0.0))) >= threshold
    ]
    episodes: List[Dict[str, Any]] = []
    current: List[Dict[str, Any]] = []
    for row in active_records:
        if current and float(row["elapsed_s"]) - float(current[-1]["elapsed_s"]) > split_gap_sec:
            finished = _finish_episode(current, value_key)
            if finished is not None:
                episodes.append(finished)
            current = []
        current.append(row)
    finished = _finish_episode(current, value_key)
    if finished is not None:
        episodes.append(finished)
    return episodes


def max_abs_command(
    records: Sequence[Dict[str, Any]], topic: str, start_s: float, end_s: float
) -> float:
    values = [
        abs(float(row.get("angular_z", 0.0)))
        for row in records
        if row.get("topic") == topic
        and start_s <= float(row["elapsed_s"]) <= end_s
    ]
    return max(values, default=0.0)


def api_window(
    api_records: Sequence[Dict[str, Any]], start_s: float, end_s: float
) -> Dict[str, Any]:
    rows = [
        row for row in api_records if start_s <= float(row["elapsed_s"]) <= end_s
    ]
    phases = sorted(
        {
            str(row.get("docking_phase", ""))
            for row in rows
            if str(row.get("docking_phase", ""))
        }
    )
    nav_phases = sorted(
        {
            str(row.get("navigation_phase", ""))
            for row in rows
            if str(row.get("navigation_phase", ""))
        }
    )
    return {
        "rows": rows,
        "phases": phases,
        "navigation_phases": nav_phases,
        "predock_yaw_active": any(bool(row.get("predock_yaw_align_active")) for row in rows),
        "ordinary_yaw_active": any(
            bool(row.get("ordinary_final_yaw_align_active"))
            or bool(row.get("navigation_yaw_align_active"))
            for row in rows
        ),
        "bridge_settle": any(
            bool(row.get("fine_bridge_settle_started"))
            and not bool(row.get("fine_bridge_settle_complete"))
            for row in rows
        )
        or any("BRIDGE_SETTLE" in phase.upper() for phase in phases),
    }


def status_window(
    status_records: Sequence[Dict[str, Any]], start_s: float, end_s: float
) -> Dict[str, Any]:
    rows = [
        row for row in status_records if start_s <= float(row["elapsed_s"]) <= end_s
    ]
    ranger_modes = sorted(
        {
            str(row.get("actual_mode"))
            for row in rows
            if row.get("source") == "ranger" and row.get("actual_mode") is not None
        }
    )
    safety_reasons = sorted(
        {
            str(row.get("blocked_reason"))
            for row in rows
            if row.get("source") == "safety" and str(row.get("blocked_reason", ""))
        }
    )
    return {
        "rows": rows,
        "ranger_modes": ranger_modes,
        "mode_switching": any(bool(row.get("mode_changing")) for row in rows),
        "safety_reasons": safety_reasons,
    }


def annotate_rotation_episode(
    episode: Dict[str, Any],
    command_records: Sequence[Dict[str, Any]],
    api_records: Sequence[Dict[str, Any]],
) -> Dict[str, Any]:
    result = dict(episode)
    start_s = float(episode["start_s"]) - 0.35
    end_s = float(episode["end_s"]) + 0.20
    maxima = {
        label: max_abs_command(command_records, topic, start_s, end_s)
        for label, topic in COMMAND_TOPICS.items()
    }
    api = api_window(api_records, start_s, end_s)
    nav_active = maxima["nav_raw"] >= COMMAND_THRESHOLD_RADPS
    docking_active = maxima["docking"] >= COMMAND_THRESHOLD_RADPS
    api_active = maxima["api"] >= COMMAND_THRESHOLD_RADPS
    phases_upper = " ".join(api["phases"]).upper()

    if nav_active and docking_active:
        source = "NAV2_AND_DOCKING_OVERLAP"
    elif nav_active:
        source = "NAV2_RAW"
    elif docking_active:
        if api["predock_yaw_active"] or "PREDOCK_YAW" in phases_upper:
            source = "API_PREDOCK_YAW"
        elif "FINE_DOCK" in phases_upper or "DOCKING" in phases_upper:
            source = "FINE_DOCKING"
        else:
            source = "DOCKING_TOPIC_UNATTRIBUTED"
    elif api_active:
        source = "API_VELOCITY"
    elif maxima["final"] >= COMMAND_THRESHOLD_RADPS:
        source = "OTHER_SAFETY_INPUT"
    else:
        source = "NO_MATCHING_COMMAND"

    result.update(
        {
            "source": source,
            "command_maxima_radps": maxima,
            "docking_phases": api["phases"],
            "navigation_phases": api["navigation_phases"],
            "predock_yaw_active": api["predock_yaw_active"],
            "ordinary_yaw_active": api["ordinary_yaw_active"],
        }
    )
    return result


def classify_pause(
    previous: Dict[str, Any],
    following: Dict[str, Any],
    command_records: Sequence[Dict[str, Any]],
    api_records: Sequence[Dict[str, Any]],
    status_records: Sequence[Dict[str, Any]],
) -> Dict[str, Any]:
    start_s = float(previous["end_s"])
    end_s = float(following["start_s"])
    maxima = {
        label: max_abs_command(command_records, topic, start_s, end_s)
        for label, topic in COMMAND_TOPICS.items()
    }
    api = api_window(api_records, start_s, end_s)
    status = status_window(status_records, start_s, end_s)
    reasons: List[str] = []

    if str(previous.get("source", "")).startswith("NAV2") and (
        "DOCKING" in str(following.get("source", ""))
        or "PREDOCK" in str(following.get("source", ""))
    ):
        reasons.append("NAV2_TO_DOCKING_HANDOFF")
    if api["bridge_settle"]:
        reasons.append("FINE_DOCKING_BRIDGE_SETTLE_GATE")
    if (
        maxima["nav_raw"] >= COMMAND_THRESHOLD_RADPS
        and maxima["nav_smoothed"] < COMMAND_THRESHOLD_RADPS
    ):
        reasons.append("VELOCITY_SMOOTHER_SUPPRESSION")
    if (
        maxima["nav_smoothed"] >= COMMAND_THRESHOLD_RADPS
        and maxima["collision_checked"] < COMMAND_THRESHOLD_RADPS
    ):
        reasons.append("COLLISION_MONITOR_SUPPRESSION")
    if (
        max(maxima["collision_checked"], maxima["docking"], maxima["api"])
        >= COMMAND_THRESHOLD_RADPS
        and maxima["final"] < COMMAND_THRESHOLD_RADPS
    ):
        reasons.append("ROBOT_SAFETY_SUPPRESSION")
    if status["mode_switching"] or len(status["ranger_modes"]) > 1:
        reasons.append("RANGER_MODE_SWITCH_OR_SETTLE")
    if status["safety_reasons"]:
        reasons.append("SAFETY_STATUS_CHANGED")
    if not reasons:
        reasons.append("COMMAND_OWNER_ZERO_WINDOW_OR_UNOBSERVED_GATE")

    return {
        "start_s": start_s,
        "end_s": end_s,
        "gap_s": max(0.0, end_s - start_s),
        "from_source": previous.get("source"),
        "to_source": following.get("source"),
        "command_maxima_radps": maxima,
        "docking_phases": api["phases"],
        "navigation_phases": api["navigation_phases"],
        "ranger_modes": status["ranger_modes"],
        "safety_reasons": status["safety_reasons"],
        "reasons": reasons,
    }


def analyze_capture(
    command_records: Sequence[Dict[str, Any]],
    wheel_records: Sequence[Dict[str, Any]],
    api_records: Sequence[Dict[str, Any]],
    status_records: Sequence[Dict[str, Any]],
) -> Dict[str, Any]:
    physical = detect_active_episodes(
        wheel_records,
        value_key="angular_z",
        threshold=ROTATION_THRESHOLD_RADPS,
    )
    episodes = [
        annotate_rotation_episode(row, command_records, api_records) for row in physical
    ]
    pauses: List[Dict[str, Any]] = []
    for previous, following in zip(episodes, episodes[1:]):
        gap_s = float(following["start_s"]) - float(previous["end_s"])
        if EPISODE_SPLIT_GAP_SEC <= gap_s <= TWO_STAGE_MAX_GAP_SEC:
            pauses.append(
                classify_pause(
                    previous,
                    following,
                    command_records,
                    api_records,
                    status_records,
                )
            )

    command_episodes: Dict[str, List[Dict[str, Any]]] = {}
    for label in ("nav_raw", "docking"):
        topic = COMMAND_TOPICS[label]
        rows = [
            {
                "elapsed_s": row["elapsed_s"],
                "angular_z": row.get("angular_z", 0.0),
                "yaw_rad": None,
            }
            for row in command_records
            if row.get("topic") == topic
        ]
        command_episodes[label] = detect_active_episodes(
            rows,
            value_key="angular_z",
            threshold=COMMAND_THRESHOLD_RADPS,
        )

    if len(wheel_records) < 5:
        verdict = "INCONCLUSIVE_NO_WHEEL_ODOM"
    elif pauses:
        verdict = "RED_ROTATE_STOP_ROTATE_CAPTURED"
    else:
        verdict = "NO_TWO_STAGE_ROTATION_CAPTURED"

    return {
        "verdict": verdict,
        "symptom_detected": bool(pauses),
        "physical_rotation_episodes": episodes,
        "pause_windows": pauses,
        "command_episodes": command_episodes,
    }


def api_get_json(
    api_url: str, path: str, token: str, timeout_sec: float
) -> Tuple[Dict[str, Any], Optional[int], str]:
    headers = {"Accept": "application/json"}
    if token:
        headers["X-Robot-Token"] = token
    request = urllib.request.Request(f"{api_url.rstrip('/')}{path}", headers=headers)
    try:
        with urllib.request.urlopen(request, timeout=timeout_sec) as response:
            payload = json.loads(response.read().decode("utf-8"))
            return (
                payload if isinstance(payload, dict) else {"value": payload},
                int(response.status),
                "",
            )
    except urllib.error.HTTPError as exc:
        return {}, int(exc.code), f"HTTP_{exc.code}"
    except Exception as exc:  # local read-only telemetry must not abort ROS capture
        return {}, None, type(exc).__name__


def _status_signature(source: str, payload: Dict[str, Any]) -> Dict[str, Any]:
    if source == "ranger":
        return {
            "source": source,
            "state": payload.get("state"),
            "desired_mode": nested(payload, "desired_motion_mode", "code"),
            "actual_mode": nested(payload, "actual_motion_mode", "code"),
            "mode_changing": nested(payload, "actual_motion_mode", "mode_changing"),
            "mode_aligned": payload.get("mode_aligned"),
        }
    if source == "safety":
        return {
            "source": source,
            "state": payload.get("state", payload.get("status")),
            "motion_allowed": payload.get("motion_allowed"),
            "active_source": payload.get("active_source"),
            "blocked_reason": payload.get(
                "blocked_reason",
                payload.get("reason", payload.get("normal_motion_blocked_reason")),
            ),
        }
    if source == "docking_manager":
        return {
            "source": source,
            "state": payload.get("state", payload.get("status")),
            "phase": payload.get("phase"),
            "reason": payload.get("reason", payload.get("detail")),
            "yaw_error": payload.get("yaw_error", payload.get("yaw_error_rad")),
        }
    if source == "localization_bridge":
        return {
            "source": source,
            "state": payload.get("state"),
            "safe_for_goal_start": payload.get("safe_for_goal_start"),
            "smoothing_active": payload.get("smoothing_active"),
            "remaining_yaw_rad": payload.get("remaining_yaw_rad"),
        }
    return {
        "source": source,
        "state": payload.get("state", payload.get("status", payload.get("raw"))),
        "correction_active": payload.get("correction_active"),
        "settle_ready": payload.get("settle_ready"),
    }


try:
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
    from std_msgs.msg import String
except ImportError as ros_import_error:  # unit tests exercise pure analysis without ROS
    rclpy = None
    ROS_IMPORT_ERROR = repr(ros_import_error)
    Node = object  # type: ignore[misc,assignment]
else:
    ROS_IMPORT_ERROR = ""


class DockingRotationTraceRecorder(Node):  # type: ignore[misc]
    def __init__(self, output_dir: Path, sample_hz: float) -> None:
        super().__init__(
            "docking_rotation_trace_recorder",
            enable_rosout=False,
            start_parameter_services=False,
        )
        parameter_publisher = getattr(self, "_parameter_event_publisher", None)
        if parameter_publisher is not None:
            self.destroy_publisher(parameter_publisher)
            self._parameter_event_publisher = None

        self.output_dir = output_dir
        self.start_monotonic = time.monotonic()
        self.stop_requested = False
        self.counts: Dict[str, int] = {}
        self.latest_commands: Dict[str, Dict[str, Any]] = {}
        self.command_records: List[Dict[str, Any]] = []
        self.wheel_records: List[Dict[str, Any]] = []
        self.local_records: List[Dict[str, Any]] = []
        self.api_records: List[Dict[str, Any]] = []
        self.status_records: List[Dict[str, Any]] = []
        self.latest_status: Dict[str, Dict[str, Any]] = {}
        self.status_signatures: Dict[str, str] = {}
        self.latest_api: Dict[str, Any] = {}
        self.api_signature = ""
        self.api_auth_ok = False
        self.api_errors: Dict[str, int] = {}
        self.readiness_missing: List[str] = list(READINESS_TOPICS)
        self.wheel_yaw = UnwrappedYaw()
        self.local_yaw = UnwrappedYaw()
        self.wheel_latest: Dict[str, Any] = {}
        self.local_latest: Dict[str, Any] = {}

        self.command_file = (output_dir / "command_events.csv").open(
            "w", newline="", encoding="utf-8"
        )
        self.command_fields = (
            "elapsed_s",
            "wall_time_device_utc_label",
            "label",
            "topic",
            "linear_x",
            "linear_y",
            "angular_z",
            "interarrival_s",
            "docking_phase",
            "predock_yaw_align_active",
            "navigation_phase",
        )
        self.command_writer = csv.DictWriter(
            self.command_file, fieldnames=self.command_fields
        )
        self.command_writer.writeheader()

        self.api_file = (output_dir / "api_samples.csv").open(
            "w", newline="", encoding="utf-8"
        )
        self.api_fields = (
            "elapsed_s",
            "wall_time_device_utc_label",
            "docking_http_status",
            "navigation_http_status",
            "docking_error",
            "navigation_error",
            *tuple(f"docking_{field}" for field in DOCKING_API_FIELDS),
            *tuple(f"navigation_{field}" for field in NAVIGATION_API_FIELDS),
        )
        self.api_writer = csv.DictWriter(self.api_file, fieldnames=self.api_fields)
        self.api_writer.writeheader()

        self.events_file = (output_dir / "state_events.jsonl").open(
            "w", encoding="utf-8"
        )
        self.samples_file = (output_dir / "timeline.csv").open(
            "w", newline="", encoding="utf-8"
        )
        self.sample_fields = self._sample_fields()
        self.samples_writer = csv.DictWriter(
            self.samples_file, fieldnames=self.sample_fields
        )
        self.samples_writer.writeheader()

        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=100,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        latched_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        for label, topic in COMMAND_TOPICS.items():
            self.create_subscription(
                Twist,
                topic,
                lambda message, label=label, topic=topic: self._on_command(
                    label, topic, message
                ),
                qos,
            )
        self.create_subscription(
            Odometry, "/wheel/odom", self._on_wheel_odometry, qos
        )
        self.create_subscription(
            Odometry, "/local_state/odometry", self._on_local_odometry, qos
        )
        for source, topic in STRING_TOPICS.items():
            self.create_subscription(
                String,
                topic,
                lambda message, source=source, topic=topic: self._on_string(
                    source, topic, message
                ),
                (
                    latched_qos
                    if source in {"safety", "docking_manager"}
                    else qos
                ),
            )
        self.create_timer(1.0 / sample_hz, self._write_sample)
        self.create_timer(0.5, self._flush)

    def elapsed(self) -> float:
        return time.monotonic() - self.start_monotonic

    def _increment(self, topic: str) -> None:
        self.counts[topic] = self.counts.get(topic, 0) + 1

    def _event(self, event: str, **fields: Any) -> None:
        payload = {
            "elapsed_s": round(self.elapsed(), 9),
            "wall_time_device_utc_label": utc_now(),
            "event": event,
            **fields,
        }
        self.events_file.write(
            json.dumps(payload, ensure_ascii=False, sort_keys=True) + "\n"
        )

    def _on_command(self, label: str, topic: str, message: Any) -> None:
        now = self.elapsed()
        previous = self.latest_commands.get(label)
        record = {
            "elapsed_s": now,
            "wall_time_device_utc_label": utc_now(),
            "label": label,
            "topic": topic,
            "linear_x": float(message.linear.x),
            "linear_y": float(message.linear.y),
            "angular_z": float(message.angular.z),
            "interarrival_s": (
                None if previous is None else now - float(previous["elapsed_s"])
            ),
            "docking_phase": self.latest_api.get("docking_phase"),
            "predock_yaw_align_active": self.latest_api.get(
                "predock_yaw_align_active"
            ),
            "navigation_phase": self.latest_api.get("navigation_phase"),
        }
        self.latest_commands[label] = record
        self.command_records.append(record)
        self.command_writer.writerow(
            {field: csv_value(record.get(field)) for field in self.command_fields}
        )
        self._increment(topic)

    def _on_odometry(
        self,
        topic: str,
        message: Any,
        unwrapper: UnwrappedYaw,
        records: List[Dict[str, Any]],
    ) -> Dict[str, Any]:
        now = self.elapsed()
        quaternion = message.pose.pose.orientation
        yaw = unwrapper.update(
            angle_from_quaternion(
                quaternion.x, quaternion.y, quaternion.z, quaternion.w
            )
        )
        record = {
            "elapsed_s": now,
            "angular_z": float(message.twist.twist.angular.z),
            "yaw_rad": yaw,
        }
        records.append(record)
        self._increment(topic)
        return record

    def _on_wheel_odometry(self, message: Any) -> None:
        self.wheel_latest = self._on_odometry(
            "/wheel/odom", message, self.wheel_yaw, self.wheel_records
        )

    def _on_local_odometry(self, message: Any) -> None:
        self.local_latest = self._on_odometry(
            "/local_state/odometry", message, self.local_yaw, self.local_records
        )

    def _on_string(self, source: str, topic: str, message: Any) -> None:
        payload = json_object(message.data)
        self.latest_status[source] = payload
        signature_data = _status_signature(source, payload)
        signature = json.dumps(
            signature_data, ensure_ascii=False, sort_keys=True, separators=(",", ":")
        )
        if signature != self.status_signatures.get(source):
            record = {
                "elapsed_s": self.elapsed(),
                **signature_data,
            }
            self.status_records.append(record)
            self._event("status_changed", **signature_data)
            self.status_signatures[source] = signature
        self._increment(topic)

    def update_api(
        self,
        docking_payload: Dict[str, Any],
        docking_status: Optional[int],
        docking_error: str,
        navigation_payload: Dict[str, Any],
        navigation_status: Optional[int],
        navigation_error: str,
    ) -> None:
        docking = docking_payload.get("docking", {})
        if not isinstance(docking, dict):
            docking = {}
        navigation = navigation_payload.get("navigation", {})
        if not isinstance(navigation, dict):
            navigation = navigation_payload.get("goal", {})
        if not isinstance(navigation, dict):
            navigation = {}

        now = self.elapsed()
        selected_docking = select_fields(docking, DOCKING_API_FIELDS)
        selected_navigation = select_fields(navigation, NAVIGATION_API_FIELDS)
        row: Dict[str, Any] = {
            "elapsed_s": now,
            "wall_time_device_utc_label": utc_now(),
            "docking_http_status": docking_status,
            "navigation_http_status": navigation_status,
            "docking_error": docking_error,
            "navigation_error": navigation_error,
        }
        row.update(
            {f"docking_{key}": value for key, value in selected_docking.items()}
        )
        row.update(
            {f"navigation_{key}": value for key, value in selected_navigation.items()}
        )
        self.api_writer.writerow(
            {field: csv_value(row.get(field)) for field in self.api_fields}
        )

        api_record = {
            "elapsed_s": now,
            "docking_state": selected_docking.get("state"),
            "docking_phase": selected_docking.get("phase"),
            "docking_detail": selected_docking.get("detail"),
            "predock_yaw_align_active": selected_docking.get(
                "predock_yaw_align_active"
            ),
            "predock_yaw_align_attempted": selected_docking.get(
                "predock_yaw_align_attempted"
            ),
            "predock_yaw_aligned": selected_docking.get("predock_yaw_aligned"),
            "predock_base_yaw_error_rad": selected_docking.get(
                "predock_base_yaw_error_rad"
            ),
            "predock_contact_yaw_error_rad": selected_docking.get(
                "predock_contact_yaw_error_rad"
            ),
            "predock_yaw_align_initial_error_rad": selected_docking.get(
                "predock_yaw_align_initial_error_rad"
            ),
            "predock_yaw_align_final_error_rad": selected_docking.get(
                "predock_yaw_align_final_error_rad"
            ),
            "fine_bridge_settle_started": selected_docking.get(
                "fine_bridge_settle_started"
            ),
            "fine_bridge_settle_complete": selected_docking.get(
                "fine_bridge_settle_complete"
            ),
            "fine_entry_ok": selected_docking.get("fine_entry_ok"),
            "ordinary_final_yaw_align_active": bool(
                selected_docking.get("ordinary_final_yaw_align_active")
            )
            or bool(selected_navigation.get("ordinary_final_yaw_align_active")),
            "navigation_state": selected_navigation.get("state"),
            "navigation_phase": selected_navigation.get("phase"),
            "navigation_yaw_align_active": selected_navigation.get(
                "yaw_align_active"
            ),
            "docking_http_status": docking_status,
            "navigation_http_status": navigation_status,
        }
        self.latest_api = api_record
        self.api_records.append(api_record)
        self.api_auth_ok = self.api_auth_ok or (
            docking_status == 200 and navigation_status == 200
        )
        for error in (docking_error, navigation_error):
            if error:
                self.api_errors[error] = self.api_errors.get(error, 0) + 1

        signature_data = {
            "docking_state": api_record["docking_state"],
            "docking_phase": api_record["docking_phase"],
            "docking_detail": api_record["docking_detail"],
            "predock_yaw_align_active": api_record["predock_yaw_align_active"],
            "predock_yaw_aligned": api_record["predock_yaw_aligned"],
            "fine_bridge_settle_started": api_record["fine_bridge_settle_started"],
            "fine_bridge_settle_complete": api_record["fine_bridge_settle_complete"],
            "fine_entry_ok": api_record["fine_entry_ok"],
            "navigation_state": api_record["navigation_state"],
            "navigation_phase": api_record["navigation_phase"],
            "navigation_yaw_align_active": api_record[
                "navigation_yaw_align_active"
            ],
            "docking_http_status": docking_status,
            "navigation_http_status": navigation_status,
            "docking_error": docking_error,
            "navigation_error": navigation_error,
        }
        signature = json.dumps(
            signature_data, ensure_ascii=False, sort_keys=True, separators=(",", ":")
        )
        if signature != self.api_signature:
            self._event("api_state_changed", **signature_data)
            self.api_signature = signature

    @staticmethod
    def _sample_fields() -> Tuple[str, ...]:
        fields: List[str] = ["elapsed_s", "wall_time_device_utc_label"]
        for label in COMMAND_TOPICS:
            fields.extend(
                (
                    f"{label}_linear_x",
                    f"{label}_linear_y",
                    f"{label}_angular_z",
                    f"{label}_age_s",
                )
            )
        fields.extend(
            (
                "wheel_angular_z",
                "wheel_yaw_unwrapped_rad",
                "wheel_age_s",
                "local_angular_z",
                "local_yaw_unwrapped_rad",
                "local_age_s",
                "ranger_actual_mode",
                "ranger_mode_changing",
                "safety_state",
                "safety_motion_allowed",
                "safety_active_source",
                "safety_blocked_reason",
                "docking_manager_state",
                "docking_manager_phase",
                "docking_state",
                "docking_phase",
                "predock_yaw_align_active",
                "predock_yaw_aligned",
                "predock_base_yaw_error_rad",
                "predock_contact_yaw_error_rad",
                "predock_yaw_align_initial_error_rad",
                "predock_yaw_align_final_error_rad",
                "fine_bridge_settle_started",
                "fine_bridge_settle_complete",
                "fine_entry_ok",
                "navigation_state",
                "navigation_phase",
                "navigation_yaw_align_active",
            )
        )
        return tuple(fields)

    def _write_sample(self) -> None:
        now = self.elapsed()
        row: Dict[str, Any] = {
            "elapsed_s": now,
            "wall_time_device_utc_label": utc_now(),
        }
        for label in COMMAND_TOPICS:
            command = self.latest_commands.get(label, {})
            row.update(
                {
                    f"{label}_linear_x": command.get("linear_x"),
                    f"{label}_linear_y": command.get("linear_y"),
                    f"{label}_angular_z": command.get("angular_z"),
                    f"{label}_age_s": (
                        None
                        if not command
                        else now - float(command.get("elapsed_s", now))
                    ),
                }
            )
        ranger = _status_signature(
            "ranger", self.latest_status.get("ranger", {})
        )
        safety = _status_signature(
            "safety", self.latest_status.get("safety", {})
        )
        docking_manager = _status_signature(
            "docking_manager", self.latest_status.get("docking_manager", {})
        )
        row.update(
            {
                "wheel_angular_z": self.wheel_latest.get("angular_z"),
                "wheel_yaw_unwrapped_rad": self.wheel_latest.get("yaw_rad"),
                "wheel_age_s": (
                    None
                    if not self.wheel_latest
                    else now - float(self.wheel_latest["elapsed_s"])
                ),
                "local_angular_z": self.local_latest.get("angular_z"),
                "local_yaw_unwrapped_rad": self.local_latest.get("yaw_rad"),
                "local_age_s": (
                    None
                    if not self.local_latest
                    else now - float(self.local_latest["elapsed_s"])
                ),
                "ranger_actual_mode": ranger.get("actual_mode"),
                "ranger_mode_changing": ranger.get("mode_changing"),
                "safety_state": safety.get("state"),
                "safety_motion_allowed": safety.get("motion_allowed"),
                "safety_active_source": safety.get("active_source"),
                "safety_blocked_reason": safety.get("blocked_reason"),
                "docking_manager_state": docking_manager.get("state"),
                "docking_manager_phase": docking_manager.get("phase"),
                **{
                    field: self.latest_api.get(field)
                    for field in (
                        "docking_state",
                        "docking_phase",
                        "predock_yaw_align_active",
                        "predock_yaw_aligned",
                        "predock_base_yaw_error_rad",
                        "predock_contact_yaw_error_rad",
                        "predock_yaw_align_initial_error_rad",
                        "predock_yaw_align_final_error_rad",
                        "fine_bridge_settle_started",
                        "fine_bridge_settle_complete",
                        "fine_entry_ok",
                        "navigation_state",
                        "navigation_phase",
                        "navigation_yaw_align_active",
                    )
                },
            }
        )
        self.samples_writer.writerow(
            {field: csv_value(row.get(field)) for field in self.sample_fields}
        )

    def _flush(self) -> None:
        self.command_file.flush()
        self.api_file.flush()
        self.events_file.flush()
        self.samples_file.flush()

    def missing_readiness_topics(self) -> List[str]:
        return [topic for topic in READINESS_TOPICS if self.counts.get(topic, 0) == 0]

    def graph_snapshot(self) -> Dict[str, Any]:
        publishers: Dict[str, List[Dict[str, Any]]] = {}
        for topic in COMMAND_TOPICS.values():
            endpoints: List[Dict[str, Any]] = []
            for info in self.get_publishers_info_by_topic(topic):
                endpoints.append(
                    {
                        "node_namespace": info.node_namespace,
                        "node_name": info.node_name,
                        "topic_type": info.topic_type,
                    }
                )
            publishers[topic] = endpoints
        return {
            "recorder_node": self.get_fully_qualified_name(),
            "owned_publishers": sum(1 for _publisher in self.publishers),
            "owned_subscriptions": sorted(
                subscription.topic_name for subscription in self.subscriptions
            ),
            "command_topic_publishers": publishers,
        }

    def close_capture(self) -> Dict[str, Any]:
        self._flush()
        analysis = analyze_capture(
            self.command_records,
            self.wheel_records,
            self.api_records,
            self.status_records,
        )
        self.command_file.close()
        self.api_file.close()
        self.events_file.close()
        self.samples_file.close()
        self._write_summary(analysis)
        (self.output_dir / "analysis.json").write_text(
            json.dumps(analysis, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        return analysis

    def _write_summary(self, analysis: Dict[str, Any]) -> None:
        lines = [
            "# Return-to-Dock Rotation Trace",
            "",
            f"- captured_at_device_utc_label: {utc_now()}",
            f"- verdict: {analysis['verdict']}",
            f"- rotate_stop_rotate_detected: {str(analysis['symptom_detected']).lower()}",
            f"- physical_rotation_episodes: {len(analysis['physical_rotation_episodes'])}",
            f"- pause_windows: {len(analysis['pause_windows'])}",
            f"- api_authenticated: {str(self.api_auth_ok).lower()}",
            f"- api_errors: {json.dumps(self.api_errors, sort_keys=True)}",
            "- recorder_behavior: read-only; no command, goal, parameter, service, scan, or pointcloud operation",
            "",
            "## Physical rotation episodes",
            "",
            "| # | start s | duration s | yaw delta deg | max wheel wz | source | phases |",
            "|---:|---:|---:|---:|---:|---|---|",
        ]
        episodes = analysis["physical_rotation_episodes"]
        if not episodes:
            lines.append("| - | - | - | - | - | NO_PHYSICAL_ROTATION | - |")
        for index, episode in enumerate(episodes, start=1):
            yaw_delta = finite_float(episode.get("yaw_delta_rad"))
            yaw_deg = "" if yaw_delta is None else f"{math.degrees(yaw_delta):.3f}"
            phases = ",".join(episode.get("docking_phases", []))
            lines.append(
                f"| {index} | {episode['start_s']:.3f} | {episode['duration_s']:.3f} "
                f"| {yaw_deg} | {episode['max_abs_wz']:.3f} | {episode['source']} "
                f"| {phases} |"
            )
        lines.extend(
            [
                "",
                "## Stop/handoff windows",
                "",
                "| # | start s | gap s | from | to | evidence | safety reasons |",
                "|---:|---:|---:|---|---|---|---|",
            ]
        )
        pauses = analysis["pause_windows"]
        if not pauses:
            lines.append("| - | - | - | - | - | NO_TWO_STAGE_WINDOW | - |")
        for index, pause in enumerate(pauses, start=1):
            lines.append(
                f"| {index} | {pause['start_s']:.3f} | {pause['gap_s']:.3f} "
                f"| {pause['from_source']} | {pause['to_source']} "
                f"| {','.join(pause['reasons'])} "
                f"| {','.join(pause['safety_reasons'])} |"
            )
        lines.extend(
            [
                "",
                "## Topic counts",
                "",
                "| topic | messages |",
                "|---|---:|",
            ]
        )
        for topic in (
            *COMMAND_TOPICS.values(),
            "/wheel/odom",
            "/local_state/odometry",
            *STRING_TOPICS.values(),
        ):
            lines.append(f"| {topic} | {self.counts.get(topic, 0)} |")
        lines.extend(
            [
                "",
                "Interpretation rule: tune nothing from a NO_TWO_STAGE result. "
                "For a RED result, compare the two source columns and the pause evidence "
                "with command_events.csv, api_samples.csv, timeline.csv, and state_events.jsonl.",
            ]
        )
        (self.output_dir / "summary.md").write_text(
            "\n".join(lines) + "\n", encoding="utf-8"
        )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Read-only return-to-dock rotation trace recorder."
    )
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--duration-sec", type=float, default=240.0)
    parser.add_argument("--sample-hz", type=float, default=20.0)
    parser.add_argument("--api-hz", type=float, default=4.0)
    parser.add_argument("--warmup-sec", type=float, default=8.0)
    parser.add_argument("--api-url", default="http://127.0.0.1:8080")
    parser.add_argument("--label", default="return_dock")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if rclpy is None:
        raise SystemExit(f"ROS_IMPORT_FAILED: {ROS_IMPORT_ERROR}")
    if fcntl is None:
        raise SystemExit("FCNTL_IMPORT_FAILED: recorder requires Linux")
    if not 5.0 <= args.duration_sec <= 900.0:
        raise SystemExit("--duration-sec must be between 5 and 900")
    if not 2.0 <= args.sample_hz <= 30.0:
        raise SystemExit("--sample-hz must be between 2 and 30")
    if not 1.0 <= args.api_hz <= 10.0:
        raise SystemExit("--api-hz must be between 1 and 10")
    if not 0.0 <= args.warmup_sec <= 15.0:
        raise SystemExit("--warmup-sec must be between 0 and 15")

    output_dir = Path(args.output_dir)
    if not str(output_dir).startswith("/tmp/njrh_reports/"):
        raise SystemExit("--output-dir must be below /tmp/njrh_reports")
    output_dir.mkdir(parents=True, exist_ok=True)

    lock_handle = Path("/tmp/njrh_docking_rotation_trace.lock").open("w")
    try:
        fcntl.flock(lock_handle.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        print("[docking-rotation-trace] another recorder is running", file=sys.stderr)
        return 3

    token, token_source = resolve_api_token()
    metadata = {
        "captured_at_device_utc_label": utc_now(),
        "label": args.label,
        "duration_sec": args.duration_sec,
        "sample_hz": args.sample_hz,
        "api_hz": args.api_hz,
        "api_url": args.api_url,
        "read_only": True,
        "publishes_commands": False,
        "sends_goals": False,
        "calls_services": False,
        "subscribes_scan": False,
        "subscribes_pointcloud": False,
        "api_token_present": bool(token),
        "api_token_source": token_source,
    }
    (output_dir / "metadata.json").write_text(
        json.dumps(metadata, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )

    rclpy.init()
    node = DockingRotationTraceRecorder(output_dir, args.sample_hz)
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    def request_stop(_signum: int, _frame: Any) -> None:
        node.stop_requested = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    analysis: Dict[str, Any] = {}
    try:
        warmup_started = time.monotonic()
        warmup_deadline = warmup_started + args.warmup_sec
        minimum_graph_warmup_sec = min(4.0, args.warmup_sec)
        ready_since: Optional[float] = None
        while (
            rclpy.ok()
            and not node.stop_requested
            and time.monotonic() < warmup_deadline
        ):
            executor.spin_once(timeout_sec=0.05)
            missing = node.missing_readiness_topics()
            if missing:
                ready_since = None
            elif ready_since is None:
                ready_since = time.monotonic()
            elif (
                time.monotonic() - ready_since >= 0.5
                and time.monotonic() - warmup_started >= minimum_graph_warmup_sec
            ):
                break
        node.readiness_missing = node.missing_readiness_topics()
        graph = node.graph_snapshot()
        (output_dir / "graph.json").write_text(
            json.dumps(graph, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        if graph["owned_publishers"] != 0:
            raise RuntimeError(
                f"recorder unexpectedly owns {graph['owned_publishers']} publishers"
            )
        if node.readiness_missing:
            print(
                "[docking-rotation-trace] WARNING missing readiness topics: "
                + ",".join(node.readiness_missing),
                flush=True,
            )
        print(
            "[docking-rotation-trace] READY: trigger normal return-to-dock from the App; "
            "press Ctrl+C after the symptom to finalize",
            flush=True,
        )

        deadline = time.monotonic() + args.duration_sec
        next_api_poll = 0.0
        api_period = 1.0 / args.api_hz
        while rclpy.ok() and not node.stop_requested and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.02)
            now = time.monotonic()
            if now >= next_api_poll:
                docking_payload, docking_status, docking_error = api_get_json(
                    args.api_url,
                    "/api/v1/docking/state",
                    token,
                    timeout_sec=0.20,
                )
                navigation_payload, navigation_status, navigation_error = api_get_json(
                    args.api_url,
                    "/api/v1/navigation/state",
                    token,
                    timeout_sec=0.20,
                )
                node.update_api(
                    docking_payload,
                    docking_status,
                    docking_error,
                    navigation_payload,
                    navigation_status,
                    navigation_error,
                )
                next_api_poll = now + api_period
    except KeyboardInterrupt:
        pass
    finally:
        analysis = node.close_capture()
        metadata.update(
            {
                "completed_at_device_utc_label": utc_now(),
                "readiness_missing": node.readiness_missing,
                "api_authenticated": node.api_auth_ok,
                "api_errors": node.api_errors,
                "topic_counts": node.counts,
                "verdict": analysis.get("verdict"),
            }
        )
        (output_dir / "metadata.json").write_text(
            json.dumps(metadata, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        executor.remove_node(node)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        fcntl.flock(lock_handle.fileno(), fcntl.LOCK_UN)
        lock_handle.close()

    print(f"[docking-rotation-trace] verdict={analysis.get('verdict')}", flush=True)
    print(f"[docking-rotation-trace] summary={output_dir / 'summary.md'}", flush=True)
    print(f"[docking-rotation-trace] report_dir={output_dir}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
