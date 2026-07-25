#!/usr/bin/env python3

import argparse
import csv
import json
import math
import statistics
import time
import urllib.error
import urllib.request
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Optional, Sequence


@dataclass(frozen=True)
class ObservationSnapshot:
    sensor_healthy: bool
    valid: bool
    forward_gap_m: float
    lateral_error_m: float
    yaw_error_rad: float
    confidence: float
    source: str
    reason: str = ""


@dataclass(frozen=True)
class SweepServoConfig:
    expected_source: str = "orbbec_336l_depth"
    tolerance_m: float = 0.008
    kp: float = 0.35
    min_speed_mps: float = 0.012
    max_speed_mps: float = 0.040
    max_reverse_speed_mps: float = 0.030
    max_lateral_error_m: float = 0.100
    max_yaw_error_rad: float = 0.120
    min_confidence: float = 0.50


@dataclass(frozen=True)
class SweepCommand:
    state: str
    speed_mps: float
    error_m: float


@dataclass(frozen=True)
class MinimumGapDecision:
    state: str
    low_streak: int


def evaluate_minimum_safe_gap(
    forward_gap_m: float,
    minimum_gap_m: float,
    direction: int,
    low_streak: int,
    reverse_confirm_samples: int,
    observation_usable: bool = True,
    is_new_observation: bool = True,
) -> MinimumGapDecision:
    if (
        not observation_usable
        or not math.isfinite(forward_gap_m)
        or forward_gap_m >= minimum_gap_m
    ):
        return MinimumGapDecision("clear", 0)
    if not is_new_observation:
        return MinimumGapDecision("hold", low_streak)

    next_streak = low_streak + 1
    if direction > 0 or next_streak >= reverse_confirm_samples:
        return MinimumGapDecision("abort", next_streak)
    return MinimumGapDecision("hold", next_streak)


def compute_leg_command(
    observation: ObservationSnapshot,
    target_gap_m: float,
    direction: int,
    config: SweepServoConfig,
) -> SweepCommand:
    error = observation.forward_gap_m - target_gap_m
    if direction not in (-1, 1):
        return SweepCommand("invalid_direction", 0.0, error)
    if observation.source != config.expected_source:
        return SweepCommand("source_mismatch", 0.0, error)
    if not observation.sensor_healthy or not observation.valid:
        return SweepCommand("invalid_observation", 0.0, error)
    values = (
        observation.forward_gap_m,
        observation.lateral_error_m,
        observation.yaw_error_rad,
        observation.confidence,
    )
    if not all(math.isfinite(value) for value in values):
        return SweepCommand("invalid_observation", 0.0, error)
    if observation.confidence < config.min_confidence:
        return SweepCommand("confidence_gate", 0.0, error)
    if abs(observation.lateral_error_m) > config.max_lateral_error_m:
        return SweepCommand("lateral_gate", 0.0, error)
    if abs(observation.yaw_error_rad) > config.max_yaw_error_rad:
        return SweepCommand("yaw_gate", 0.0, error)
    if abs(error) <= config.tolerance_m:
        return SweepCommand("at_target", 0.0, error)

    required_direction = 1 if error > 0.0 else -1
    if required_direction != direction:
        return SweepCommand("direction_mismatch", 0.0, error)

    limit = config.max_speed_mps if direction > 0 else config.max_reverse_speed_mps
    speed = min(limit, max(config.min_speed_mps, config.kp * abs(error)))
    return SweepCommand("drive", math.copysign(speed, direction), error)


def fit_range_motion(wheel_forward_m, observed_range_change_m) -> dict:
    pairs = [
        (float(x), float(y))
        for x, y in zip(wheel_forward_m, observed_range_change_m)
        if math.isfinite(float(x)) and math.isfinite(float(y))
    ]
    if len(pairs) < 2:
        return {
            "sample_count": len(pairs),
            "scale": math.nan,
            "offset_m": math.nan,
            "r_squared": math.nan,
        }
    xs = [pair[0] for pair in pairs]
    ys = [pair[1] for pair in pairs]
    x_mean = sum(xs) / len(xs)
    y_mean = sum(ys) / len(ys)
    x_variance = sum((value - x_mean) ** 2 for value in xs)
    if x_variance <= 1.0e-12:
        return {
            "sample_count": len(pairs),
            "scale": math.nan,
            "offset_m": math.nan,
            "r_squared": math.nan,
        }
    scale = sum((x - x_mean) * (y - y_mean) for x, y in pairs) / x_variance
    offset = y_mean - scale * x_mean
    residual_sum = sum((y - (scale * x + offset)) ** 2 for x, y in pairs)
    total_sum = sum((y - y_mean) ** 2 for y in ys)
    r_squared = 1.0 if total_sum <= 1.0e-12 else 1.0 - residual_sum / total_sum
    return {
        "sample_count": len(pairs),
        "scale": scale,
        "offset_m": offset,
        "r_squared": r_squared,
    }


def _row_flag_is_true(value) -> bool:
    if isinstance(value, str):
        return value.strip().lower() in {"1", "true", "yes"}
    return bool(value)


def select_valid_fit_rows(rows, expected_source: str):
    selected = []
    numeric_keys = (
        "wheel_x_m",
        "wheel_y_m",
        "forward_gap_m",
        "lateral_error_m",
        "yaw_error_rad",
        "confidence",
    )
    for row in rows:
        if not _row_flag_is_true(row.get("observation_sensor_healthy")):
            continue
        if not _row_flag_is_true(row.get("observation_valid")):
            continue
        if str(row.get("observation_source", "")) != expected_source:
            continue
        try:
            values = {key: float(row[key]) for key in numeric_keys}
        except (KeyError, TypeError, ValueError):
            continue
        if not all(math.isfinite(value) for value in values.values()):
            continue
        if values["forward_gap_m"] <= 0.0:
            continue
        selected.append(row)
    return selected


def build_sweep_targets(cycles: int, return_to_far_only: bool) -> list[tuple[int, str]]:
    if return_to_far_only:
        return [(-1, "reverse")]
    targets = []
    for _ in range(cycles):
        targets.extend([(1, "forward"), (-1, "reverse")])
    return targets


def _safe_label(value: str) -> str:
    cleaned = "".join(character if character.isalnum() or character in "_-" else "_" for character in value)
    return cleaned.strip("_") or "bidirectional_fit"


def _timestamp_tag() -> str:
    return datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")


def _yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _api_get(api_url: str, path: str, timeout_sec: float = 2.0) -> dict:
    with urllib.request.urlopen(api_url.rstrip("/") + path, timeout=timeout_sec) as response:
        return json.load(response)


def _runtime_gate(api_url: str) -> tuple:
    try:
        docking = _api_get(api_url, "/api/v1/docking/state")
        navigation = _api_get(api_url, "/api/v1/navigation/state")
    except (OSError, ValueError, urllib.error.URLError) as exc:
        return False, f"api_unavailable:{exc}", {}, {}

    docking_active = bool(docking.get("docking_active", docking.get("active", False)))
    docking_state = str(docking.get("state", "")).strip().lower()
    if docking_active or docking_state in {"docking", "undocking", "active"}:
        return False, f"docking_active:{docking_state or 'unknown'}", docking, navigation

    goal = navigation.get("navigation_goal") or {}
    goal_state = str(goal.get("state", "")).strip().lower()
    if goal_state in {"accepted", "starting", "running", "active", "canceling"}:
        return False, f"navigation_goal_active:{goal_state}", docking, navigation
    if bool(docking.get("charging_contact", False)):
        return False, "charging_contact", docking, navigation
    return True, "ready", docking, navigation


def run_ros_sweep(args: argparse.Namespace) -> tuple:
    import rclpy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
    from robot_interfaces.msg import DockTargetObservation
    from sensor_msgs.msg import BatteryState
    from std_msgs.msg import Bool, String

    class SweepNode(Node):
        def __init__(self):
            super().__init__("orbbec_dock_bidirectional_fit")
            self.observation = None
            self.observation_at = 0.0
            self.observation_seq = 0
            self.wheel_linear_x = math.nan
            self.wheel_angular_z = math.nan
            self.wheel_x = math.nan
            self.wheel_y = math.nan
            self.wheel_yaw = math.nan
            self.wheel_at = 0.0
            self.final_linear_x = math.nan
            self.final_linear_y = math.nan
            self.final_angular_z = math.nan
            self.final_cmd_at = 0.0
            self.docking_status = ""
            self.docking_status_seq = 0
            self.safety_status = ""
            self.estop_active = False
            self.battery_contact = False
            self.reverse_enable = False
            self.command_pub = self.create_publisher(Twist, args.cmd_topic, QoSProfile(depth=1))
            latched_qos = QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )
            self.reverse_enable_pub = self.create_publisher(
                Bool, args.reverse_enable_topic, latched_qos
            )
            self.create_subscription(
                DockTargetObservation,
                args.observation_topic,
                self._observation,
                QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE),
            )
            self.create_subscription(Odometry, args.wheel_odom_topic, self._wheel, qos_profile_sensor_data)
            self.create_subscription(Twist, args.final_cmd_topic, self._final_cmd, QoSProfile(depth=5))
            self.create_subscription(String, args.docking_status_topic, self._docking_status, latched_qos)
            self.create_subscription(String, args.safety_status_topic, self._safety_status, QoSProfile(depth=5))
            self.create_subscription(Bool, args.estop_topic, self._estop, QoSProfile(depth=5))
            self.create_subscription(BatteryState, args.battery_topic, self._battery, qos_profile_sensor_data)

        def _observation(self, message):
            self.observation = ObservationSnapshot(
                sensor_healthy=message.sensor_healthy,
                valid=message.valid,
                forward_gap_m=message.forward_gap_m,
                lateral_error_m=message.lateral_error_m,
                yaw_error_rad=message.yaw_error_rad,
                confidence=message.confidence,
                source=message.source,
                reason=message.reason,
            )
            self.observation_at = time.monotonic()
            self.observation_seq += 1

        def _wheel(self, message):
            self.wheel_linear_x = message.twist.twist.linear.x
            self.wheel_angular_z = message.twist.twist.angular.z
            self.wheel_x = message.pose.pose.position.x
            self.wheel_y = message.pose.pose.position.y
            orientation = message.pose.pose.orientation
            self.wheel_yaw = _yaw_from_quaternion(
                orientation.x, orientation.y, orientation.z, orientation.w
            )
            self.wheel_at = time.monotonic()

        def _final_cmd(self, message):
            self.final_linear_x = message.linear.x
            self.final_linear_y = message.linear.y
            self.final_angular_z = message.angular.z
            self.final_cmd_at = time.monotonic()

        def _docking_status(self, message):
            self.docking_status = message.data.strip().lower()
            self.docking_status_seq += 1

        def _safety_status(self, message):
            self.safety_status = message.data.strip().upper()

        def _estop(self, message):
            self.estop_active = bool(message.data)

        def _battery(self, message):
            charging_states = {
                BatteryState.POWER_SUPPLY_STATUS_CHARGING,
                BatteryState.POWER_SUPPLY_STATUS_FULL,
            }
            self.battery_contact = (
                message.power_supply_status in charging_states
                or (math.isfinite(message.current) and message.current > args.charging_current_min_a)
            )

        def publish_speed(self, speed_mps: float):
            message = Twist()
            message.linear.x = float(speed_mps)
            self.command_pub.publish(message)

        def publish_reverse_enable(self, enabled: bool):
            self.reverse_enable = bool(enabled)
            message = Bool()
            message.data = self.reverse_enable
            self.reverse_enable_pub.publish(message)

    def spin_for(
        node, duration_sec: float, speed_mps: float = 0.0, reverse_enabled: bool = False
    ):
        deadline = time.monotonic() + duration_sec
        period = 1.0 / args.command_rate_hz
        while rclpy.ok() and time.monotonic() < deadline:
            loop_started = time.monotonic()
            node.publish_reverse_enable(reverse_enabled)
            node.publish_speed(speed_mps)
            rclpy.spin_once(node, timeout_sec=min(0.03, period))
            remaining = period - (time.monotonic() - loop_started)
            if remaining > 0.0:
                time.sleep(remaining)

    def wait_stopped(node, reverse_enabled: bool = False) -> bool:
        deadline = time.monotonic() + args.stop_settle_timeout_sec
        stable = 0
        while rclpy.ok() and time.monotonic() < deadline:
            node.publish_reverse_enable(reverse_enabled)
            node.publish_speed(0.0)
            rclpy.spin_once(node, timeout_sec=0.05)
            stopped = (
                math.isfinite(node.wheel_linear_x)
                and math.isfinite(node.wheel_angular_z)
                and abs(node.wheel_linear_x) <= args.stop_linear_threshold_mps
                and abs(node.wheel_angular_z) <= args.stop_angular_threshold_radps
            )
            stable = stable + 1 if stopped else 0
            if stable >= args.stop_stable_samples:
                return True
        return False

    def sample_row(node, started_at, leg_index, phase, target_gap, direction, command_speed):
        observation = node.observation
        return {
            "elapsed_sec": time.monotonic() - started_at,
            "leg_index": leg_index,
            "phase": phase,
            "target_gap_m": target_gap,
            "direction": direction,
            "observation_seq": node.observation_seq,
            "observation_age_sec": time.monotonic() - node.observation_at,
            "forward_gap_m": observation.forward_gap_m if observation else math.nan,
            "lateral_error_m": observation.lateral_error_m if observation else math.nan,
            "yaw_error_rad": observation.yaw_error_rad if observation else math.nan,
            "confidence": observation.confidence if observation else math.nan,
            "observation_sensor_healthy": observation.sensor_healthy if observation else False,
            "observation_valid": observation.valid if observation else False,
            "observation_source": observation.source if observation else "",
            "observation_reason": observation.reason if observation else "",
            "command_linear_x_mps": command_speed,
            "wheel_linear_x_mps": node.wheel_linear_x,
            "wheel_angular_z_radps": node.wheel_angular_z,
            "wheel_x_m": node.wheel_x,
            "wheel_y_m": node.wheel_y,
            "wheel_yaw_rad": node.wheel_yaw,
            "final_linear_x_mps": node.final_linear_x,
            "final_linear_y_mps": node.final_linear_y,
            "final_angular_z_radps": node.final_angular_z,
            "reverse_enable": node.reverse_enable,
            "safety_status": node.safety_status,
            "docking_status_seq": node.docking_status_seq,
            "docking_status": node.docking_status,
        }

    config = SweepServoConfig(
        tolerance_m=args.tolerance_m,
        kp=args.kp,
        min_speed_mps=args.min_speed_mps,
        max_speed_mps=args.max_speed_mps,
        max_reverse_speed_mps=args.max_reverse_speed_mps,
        max_lateral_error_m=args.max_lateral_error_m,
        max_yaw_error_rad=args.max_yaw_error_rad,
        min_confidence=args.min_confidence,
    )
    result = {
        "schema": "njrh.orbbec_dock_bidirectional_fit.v1",
        "success": False,
        "fit_accepted": False,
        "reason": "not_started",
        "started_at_utc": datetime.now(timezone.utc).isoformat(),
        "near_fixed_face_gap_m": args.near_fixed_face_gap_m,
        "far_fixed_face_gap_m": args.far_fixed_face_gap_m,
        "cycles": args.cycles,
        "legs": [],
    }
    samples = []
    started_at = time.monotonic()
    node = None
    rclpy.init()
    try:
        node = SweepNode()
        preflight_deadline = time.monotonic() + args.preflight_timeout_sec
        while rclpy.ok() and time.monotonic() < preflight_deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            now = time.monotonic()
            if (
                node.observation is not None
                and now - node.observation_at <= args.observation_timeout_sec
                and now - node.wheel_at <= args.wheel_timeout_sec
                and node.docking_status_seq > 0
            ):
                break

        now = time.monotonic()
        if node.observation is None or now - node.observation_at > args.observation_timeout_sec:
            result["reason"] = "preflight_observation_unavailable"
            return result, samples
        if now - node.wheel_at > args.wheel_timeout_sec:
            result["reason"] = "preflight_wheel_odom_unavailable"
            return result, samples
        if node.docking_status_seq <= 0:
            result["reason"] = "preflight_docking_status_unavailable"
            return result, samples
        gate_ok, gate_reason, _, _ = _runtime_gate(args.api_url)
        if not gate_ok:
            result["reason"] = f"preflight_{gate_reason}"
            return result, samples
        if node.estop_active:
            result["reason"] = "preflight_estop_active"
            return result, samples
        if node.battery_contact:
            result["reason"] = "preflight_charging_contact"
            return result, samples
        if not args.enable_reverse and not args.dry_run:
            result["reason"] = "reverse_not_explicitly_enabled"
            return result, samples
        if args.near_fixed_face_gap_m < args.min_safe_fixed_face_gap_m:
            result["reason"] = "near_target_below_safe_gap"
            return result, samples
        if args.far_fixed_face_gap_m <= args.near_fixed_face_gap_m:
            result["reason"] = "invalid_target_order"
            return result, samples
        first_target_gap = (
            args.far_fixed_face_gap_m
            if args.return_to_far_only
            else args.near_fixed_face_gap_m
        )
        if abs(node.observation.forward_gap_m - first_target_gap) > args.max_leg_distance_m:
            result["reason"] = "preflight_first_leg_too_long"
            return result, samples

        result["initial_observation"] = asdict(node.observation)
        result["initial_map_pose"] = _api_get(args.api_url, "/api/v1/robot/pose")
        initial_docking_status_seq = node.docking_status_seq
        spin_for(node, args.zero_claim_sec, 0.0)
        if not wait_stopped(node):
            result["reason"] = "preflight_chassis_not_stopped"
            return result, samples
        if args.dry_run:
            result["success"] = True
            result["reason"] = "dry_run_ok"
            result["final_observation"] = asdict(node.observation)
            return result, samples

        targets = [
            (
                args.near_fixed_face_gap_m if direction > 0 else args.far_fixed_face_gap_m,
                direction,
                phase,
            )
            for direction, phase in build_sweep_targets(
                args.cycles, args.return_to_far_only
            )
        ]

        total_wheel_distance = 0.0
        previous_wheel_x = node.wheel_x
        previous_wheel_y = node.wheel_y
        for leg_index, (target_gap, direction, phase) in enumerate(targets, start=1):
            initial_gap = node.observation.forward_gap_m
            if abs(initial_gap - target_gap) > args.max_leg_distance_m:
                result["reason"] = f"leg_{leg_index}_distance_too_large"
                break
            leg_start_x = node.wheel_x
            leg_start_y = node.wheel_y
            leg_start_yaw = node.wheel_yaw
            leg_samples = []
            stable_samples = 0
            last_target_observation_seq = -1
            last_recorded_observation_seq = -1
            last_valid_at = time.monotonic()
            progress_reference_at = time.monotonic()
            progress_reference_gap = initial_gap
            minimum_gap_low_streak = 0
            last_minimum_gap_observation_seq = -1
            next_api_gate_at = time.monotonic()
            leg_reason = "timeout"
            leg_success = False
            deadline = time.monotonic() + args.leg_timeout_sec

            if direction < 0:
                spin_for(
                    node,
                    args.reverse_permit_settle_sec,
                    0.0,
                    reverse_enabled=True,
                )

            while rclpy.ok() and time.monotonic() < deadline:
                loop_started = time.monotonic()
                rclpy.spin_once(node, timeout_sec=0.02)
                now = time.monotonic()
                command_speed = 0.0
                node.publish_reverse_enable(direction < 0)

                if now >= next_api_gate_at:
                    gate_ok, gate_reason, _, _ = _runtime_gate(args.api_url)
                    next_api_gate_at = now + args.api_gate_period_sec
                    if not gate_ok:
                        leg_reason = gate_reason
                        break
                if node.docking_status_seq > initial_docking_status_seq:
                    status = node.docking_status
                    if "phase=active" in status or status.startswith(("docking ", "undocking ")):
                        leg_reason = f"docking_owner_became_active:{status}"
                        break
                if node.estop_active:
                    leg_reason = "estop_active"
                    break
                if node.battery_contact:
                    leg_reason = "charging_contact"
                    break
                if node.safety_status in {"ESTOP_ACTIVE", "LOCALIZATION_INVALID"}:
                    leg_reason = f"safety_block:{node.safety_status}"
                    break
                if node.observation is None or now - node.observation_at > args.observation_timeout_sec:
                    node.publish_speed(0.0)
                    if now - last_valid_at > args.invalid_abort_sec:
                        leg_reason = "observation_stale"
                        break
                else:
                    command = compute_leg_command(node.observation, target_gap, direction, config)
                    observation_usable_for_gap = command.state not in {
                        "source_mismatch",
                        "invalid_observation",
                        "confidence_gate",
                    }
                    is_new_gap_observation = (
                        node.observation_seq != last_minimum_gap_observation_seq
                    )
                    gap_decision = evaluate_minimum_safe_gap(
                        node.observation.forward_gap_m,
                        args.min_safe_fixed_face_gap_m,
                        direction,
                        minimum_gap_low_streak,
                        args.reverse_min_safe_confirm_samples,
                        observation_usable=observation_usable_for_gap,
                        is_new_observation=is_new_gap_observation,
                    )
                    if is_new_gap_observation:
                        last_minimum_gap_observation_seq = node.observation_seq
                    minimum_gap_low_streak = gap_decision.low_streak
                    minimum_gap_abort = False
                    invalid_observation_abort = False
                    if gap_decision.state != "clear":
                        last_valid_at = now
                        stable_samples = 0
                        last_target_observation_seq = -1
                        node.publish_speed(0.0)
                        if gap_decision.state == "abort":
                            leg_reason = "minimum_safe_gap_reached"
                            minimum_gap_abort = True
                    else:
                        if command.state in {"invalid_observation", "confidence_gate"}:
                            node.publish_speed(0.0)
                            if now - last_valid_at > args.invalid_abort_sec:
                                leg_reason = command.state
                                invalid_observation_abort = True
                        elif command.state == "at_target":
                            last_valid_at = now
                            if node.observation_seq != last_target_observation_seq:
                                stable_samples += 1
                                last_target_observation_seq = node.observation_seq
                            node.publish_speed(0.0)
                            if stable_samples >= args.stable_samples:
                                leg_success = True
                                leg_reason = "target_reached"
                        elif command.state == "drive":
                            last_valid_at = now
                            stable_samples = 0
                            last_target_observation_seq = -1
                            command_speed = command.speed_mps
                            node.publish_speed(command_speed)
                        else:
                            leg_reason = command.state
                            node.publish_speed(0.0)
                            break

                    if node.observation_seq != last_recorded_observation_seq:
                        row = sample_row(
                            node, started_at, leg_index, phase, target_gap, direction, command_speed
                        )
                        samples.append(row)
                        leg_samples.append(row)
                        last_recorded_observation_seq = node.observation_seq

                    if minimum_gap_abort or invalid_observation_abort:
                        break

                    if (
                        observation_usable_for_gap
                        and now - progress_reference_at >= args.no_progress_timeout_sec
                        and not leg_success
                    ):
                        signed_progress = direction * (
                            progress_reference_gap - node.observation.forward_gap_m
                        )
                        if signed_progress < args.min_progress_m:
                            leg_reason = "no_range_progress"
                            break
                        progress_reference_at = now
                        progress_reference_gap = node.observation.forward_gap_m

                if math.isfinite(node.wheel_x) and math.isfinite(previous_wheel_x):
                    total_wheel_distance += math.hypot(
                        node.wheel_x - previous_wheel_x, node.wheel_y - previous_wheel_y
                    )
                    previous_wheel_x = node.wheel_x
                    previous_wheel_y = node.wheel_y
                    if total_wheel_distance > args.max_total_wheel_distance_m:
                        leg_reason = "maximum_total_wheel_distance_reached"
                        break
                if leg_success:
                    break
                sleep_sec = 1.0 / args.command_rate_hz - (time.monotonic() - loop_started)
                if sleep_sec > 0.0:
                    time.sleep(sleep_sec)

            spin_for(
                node,
                args.between_leg_zero_sec,
                0.0,
                reverse_enabled=direction < 0,
            )
            wheel_stopped = wait_stopped(node, reverse_enabled=direction < 0)
            node.publish_reverse_enable(False)
            final_gap = node.observation.forward_gap_m if node.observation else math.nan
            wheel_forward = (
                (node.wheel_x - leg_start_x) * math.cos(leg_start_yaw)
                + (node.wheel_y - leg_start_y) * math.sin(leg_start_yaw)
            )
            fit_x = []
            fit_y = []
            lateral_values = []
            yaw_values = []
            confidence_values = []
            fit_rows = select_valid_fit_rows(leg_samples, config.expected_source)
            for row in fit_rows:
                projected = (
                    (float(row["wheel_x_m"]) - leg_start_x) * math.cos(leg_start_yaw)
                    + (float(row["wheel_y_m"]) - leg_start_y) * math.sin(leg_start_yaw)
                )
                fit_x.append(projected)
                fit_y.append(initial_gap - float(row["forward_gap_m"]))
                lateral_values.append(float(row["lateral_error_m"]))
                yaw_values.append(float(row["yaw_error_rad"]))
                confidence_values.append(float(row["confidence"]))
            fit = fit_range_motion(fit_x, fit_y)
            fit_ok = (
                math.isfinite(fit["scale"])
                and args.fit_scale_min <= fit["scale"] <= args.fit_scale_max
                and math.isfinite(fit["r_squared"])
                and fit["r_squared"] >= args.fit_min_r_squared
            )
            leg = {
                "index": leg_index,
                "phase": phase,
                "direction": direction,
                "target_gap_m": target_gap,
                "initial_gap_m": initial_gap,
                "final_gap_m": final_gap,
                "final_error_m": final_gap - target_gap,
                "wheel_forward_m": wheel_forward,
                "observed_range_change_m": initial_gap - final_gap,
                "wheel_stopped": wheel_stopped,
                "success": leg_success and wheel_stopped,
                "reason": leg_reason if wheel_stopped else "wheel_stop_not_confirmed",
                "fit": fit,
                "fit_accepted": fit_ok,
                "lateral_error_median": statistics.median(lateral_values) if lateral_values else math.nan,
                "lateral_error_max_abs": max(map(abs, lateral_values)) if lateral_values else math.nan,
                "yaw_error_rad_median": statistics.median(yaw_values) if yaw_values else math.nan,
                "yaw_error_rad_max_abs": max(map(abs, yaw_values)) if yaw_values else math.nan,
                "confidence_median": statistics.median(confidence_values) if confidence_values else math.nan,
                "sample_count": len(leg_samples),
                "fit_sample_count": len(fit_rows),
                "discarded_fit_sample_count": len(leg_samples) - len(fit_rows),
            }
            result["legs"].append(leg)
            if not leg["success"]:
                result["reason"] = f"leg_{leg_index}_{leg['reason']}"
                break
        else:
            result["success"] = True
            result["reason"] = "sweep_complete"

        result["fit_accepted"] = bool(result["legs"]) and all(
            leg["fit_accepted"] for leg in result["legs"]
        )
        result["total_wheel_distance_m"] = total_wheel_distance
        result["final_observation"] = asdict(node.observation) if node.observation else None
        result["final_map_pose"] = _api_get(args.api_url, "/api/v1/robot/pose")
        return result, samples
    finally:
        if node is not None:
            try:
                spin_for(node, args.zero_burst_sec, 0.0)
                result["final_wheel_stopped"] = wait_stopped(node)
            finally:
                node.publish_reverse_enable(False)
                result["final_reverse_enable"] = node.reverse_enable
                node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        result["elapsed_sec"] = time.monotonic() - started_at


def _write_report(output_dir: Path, result: dict, samples: list, args: argparse.Namespace):
    output_dir.mkdir(parents=True, exist_ok=False)
    fields = [
        "elapsed_sec",
        "leg_index",
        "phase",
        "target_gap_m",
        "direction",
        "observation_seq",
        "observation_age_sec",
        "forward_gap_m",
        "lateral_error_m",
        "yaw_error_rad",
        "confidence",
        "observation_sensor_healthy",
        "observation_valid",
        "observation_source",
        "observation_reason",
        "command_linear_x_mps",
        "wheel_linear_x_mps",
        "wheel_angular_z_radps",
        "wheel_x_m",
        "wheel_y_m",
        "wheel_yaw_rad",
        "final_linear_x_mps",
        "final_linear_y_mps",
        "final_angular_z_radps",
        "reverse_enable",
        "safety_status",
        "docking_status_seq",
        "docking_status",
    ]
    with (output_dir / "samples.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows(samples)
    payload = {**result, "arguments": vars(args)}
    (output_dir / "metrics.json").write_text(
        json.dumps(payload, indent=2, sort_keys=True, allow_nan=True) + "\n", encoding="utf-8"
    )
    rows = []
    for leg in result.get("legs", []):
        fit = leg.get("fit", {})
        rows.append(
            "| {index} | {phase} | {initial:.4f} | {final:.4f} | {wheel:.4f} | "
            "{change:.4f} | {scale:.4f} | {r2:.4f} | {fit_samples}/{discarded} | "
            "{lat:.4f} | {yaw:.2f} | {ok} |".format(
                index=leg["index"],
                phase=leg["phase"],
                initial=leg["initial_gap_m"],
                final=leg["final_gap_m"],
                wheel=leg["wheel_forward_m"],
                change=leg["observed_range_change_m"],
                scale=fit.get("scale", math.nan),
                r2=fit.get("r_squared", math.nan),
                fit_samples=leg.get("fit_sample_count", fit.get("sample_count", 0)),
                discarded=leg.get("discarded_fit_sample_count", 0),
                lat=leg.get("lateral_error_median", math.nan),
                yaw=math.degrees(leg.get("yaw_error_rad_median", math.nan)),
                ok=leg["success"],
            )
        )
    summary = [
        "# Orbbec dock bidirectional range fit",
        "",
        f"- success: `{result.get('success')}`",
        f"- reason: `{result.get('reason')}`",
        f"- fit accepted: `{result.get('fit_accepted')}`",
        f"- total wheel distance: `{result.get('total_wheel_distance_m', math.nan):.4f} m`",
        f"- elapsed: `{result.get('elapsed_sec', math.nan):.2f} s`",
        "",
        "| Leg | Phase | Initial gap | Final gap | Wheel forward | Range change | Scale | R2 | Fit/discarded | Median lateral | Median yaw deg | Motion OK |",
        "|---:|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|",
        *rows,
        "",
        "Motion path: `/cmd_vel_docking -> robot_safety -> /cmd_vel -> ranger_base_node`.",
        "Files: `samples.csv`, `metrics.json`.",
    ]
    (output_dir / "summary.md").write_text("\n".join(summary) + "\n", encoding="utf-8")


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Supervised Orbbec fixed-face forward/reverse range fit through robot_safety"
    )
    parser.add_argument("--near-fixed-face-gap-m", type=float, default=0.45)
    parser.add_argument("--far-fixed-face-gap-m", type=float, default=0.80)
    parser.add_argument("--cycles", type=int, default=1)
    parser.add_argument("--enable-reverse", action="store_true")
    parser.add_argument("--return-to-far-only", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--label", default="bidirectional_fit")
    parser.add_argument("--output-root", default="/tmp/njrh_reports/orbbec_dock_bidirectional_fit")
    parser.add_argument("--api-url", default="http://127.0.0.1:8080")
    parser.add_argument("--observation-topic", default="/dock/target_observation")
    parser.add_argument("--cmd-topic", default="/cmd_vel_docking")
    parser.add_argument(
        "--reverse-enable-topic", default="/ranger_mini3/docking_allow_reverse"
    )
    parser.add_argument("--final-cmd-topic", default="/cmd_vel")
    parser.add_argument("--wheel-odom-topic", default="/wheel/odom")
    parser.add_argument("--docking-status-topic", default="/docking/status")
    parser.add_argument("--safety-status-topic", default="/safety/status")
    parser.add_argument("--estop-topic", default="/safety/estop")
    parser.add_argument("--battery-topic", default="/battery_state")
    parser.add_argument("--preflight-timeout-sec", type=float, default=6.0)
    parser.add_argument("--leg-timeout-sec", type=float, default=35.0)
    parser.add_argument("--observation-timeout-sec", type=float, default=0.50)
    parser.add_argument("--wheel-timeout-sec", type=float, default=0.50)
    parser.add_argument("--invalid-abort-sec", type=float, default=1.0)
    parser.add_argument("--command-rate-hz", type=float, default=20.0)
    parser.add_argument("--api-gate-period-sec", type=float, default=0.50)
    parser.add_argument("--zero-claim-sec", type=float, default=0.60)
    parser.add_argument("--reverse-permit-settle-sec", type=float, default=0.20)
    parser.add_argument("--between-leg-zero-sec", type=float, default=0.80)
    parser.add_argument("--zero-burst-sec", type=float, default=0.80)
    parser.add_argument("--stop-settle-timeout-sec", type=float, default=3.0)
    parser.add_argument("--stop-stable-samples", type=int, default=6)
    parser.add_argument("--stop-linear-threshold-mps", type=float, default=0.008)
    parser.add_argument("--stop-angular-threshold-radps", type=float, default=0.020)
    parser.add_argument("--tolerance-m", type=float, default=0.008)
    parser.add_argument("--kp", type=float, default=0.35)
    parser.add_argument("--min-speed-mps", type=float, default=0.012)
    parser.add_argument("--max-speed-mps", type=float, default=0.040)
    parser.add_argument("--max-reverse-speed-mps", type=float, default=0.030)
    parser.add_argument("--max-leg-distance-m", type=float, default=0.45)
    parser.add_argument("--max-total-wheel-distance-m", type=float, default=1.0)
    parser.add_argument("--min-safe-fixed-face-gap-m", type=float, default=0.40)
    parser.add_argument("--reverse-min-safe-confirm-samples", type=int, default=3)
    parser.add_argument("--max-lateral-error-m", type=float, default=0.10)
    parser.add_argument("--max-yaw-error-rad", type=float, default=0.12)
    parser.add_argument("--min-confidence", type=float, default=0.50)
    parser.add_argument("--stable-samples", type=int, default=8)
    parser.add_argument("--no-progress-timeout-sec", type=float, default=5.0)
    parser.add_argument("--min-progress-m", type=float, default=0.010)
    parser.add_argument("--charging-current-min-a", type=float, default=0.10)
    parser.add_argument("--fit-scale-min", type=float, default=0.85)
    parser.add_argument("--fit-scale-max", type=float, default=1.15)
    parser.add_argument("--fit-min-r-squared", type=float, default=0.95)
    args = parser.parse_args(argv)
    if args.cmd_topic != "/cmd_vel_docking":
        parser.error("calibration motion must use /cmd_vel_docking through robot_safety")
    if args.reverse_enable_topic != "/ranger_mini3/docking_allow_reverse":
        parser.error("calibration reverse must use the robot_safety docking reverse permit")
    if args.cycles < 1:
        parser.error("cycles must be at least one")
    if args.min_speed_mps <= 0.0 or args.min_speed_mps > args.max_speed_mps:
        parser.error("invalid forward speed bounds")
    if args.max_reverse_speed_mps <= 0.0:
        parser.error("max reverse speed must be positive")
    if args.reverse_min_safe_confirm_samples < 2:
        parser.error("reverse minimum-gap confirmation requires at least two samples")
    return args


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = parse_args(argv)
    output_dir = Path(args.output_root) / f"{_timestamp_tag()}_{_safe_label(args.label)}"
    result, samples = run_ros_sweep(args)
    _write_report(output_dir, result, samples, args)
    print(f"[orbbec-dock-bidirectional-fit] summary: {output_dir / 'summary.md'}")
    print(
        "[orbbec-dock-bidirectional-fit] "
        f"success={str(result['success']).lower()} fit_accepted={str(result['fit_accepted']).lower()} "
        f"reason={result['reason']}"
    )
    return 0 if result["success"] else 2


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print("[orbbec-dock-bidirectional-fit] interrupted")
        raise SystemExit(130)
