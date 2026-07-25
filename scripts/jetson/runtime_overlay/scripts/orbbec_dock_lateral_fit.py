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


@dataclass(frozen=True)
class ObservationSnapshot:
    sensor_healthy: bool
    valid: bool
    forward_gap_m: float
    lateral_error_m: float
    yaw_error_rad: float
    lateral_span_m: float
    confidence: float
    source: str
    reason: str = ""


@dataclass(frozen=True)
class LateralServoConfig:
    expected_source: str = "orbbec_336l_depth"
    response_sign: int = -1
    tolerance_m: float = 0.006
    kp: float = 0.50
    min_speed_mps: float = 0.025
    max_speed_mps: float = 0.040
    min_forward_gap_m: float = 0.65
    max_forward_gap_m: float = 0.95
    max_abs_lateral_error_m: float = 0.22
    max_abs_yaw_error_rad: float = 0.08
    min_confidence: float = 0.50


@dataclass(frozen=True)
class LateralCommand:
    state: str
    speed_mps: float
    observation_error_m: float
    vehicle_error_m: float


def project_body_displacement(
    start_x_m: float,
    start_y_m: float,
    start_yaw_rad: float,
    x_m: float,
    y_m: float,
) -> tuple[float, float]:
    dx = x_m - start_x_m
    dy = y_m - start_y_m
    forward = dx * math.cos(start_yaw_rad) + dy * math.sin(start_yaw_rad)
    lateral = -dx * math.sin(start_yaw_rad) + dy * math.cos(start_yaw_rad)
    return forward, lateral


def normalize_angle(angle_rad: float) -> float:
    return math.atan2(math.sin(angle_rad), math.cos(angle_rad))


def compute_lateral_command(
    observation: ObservationSnapshot,
    target_lateral_m: float,
    direction: int,
    config: LateralServoConfig,
) -> LateralCommand:
    observation_error = target_lateral_m - observation.lateral_error_m
    vehicle_error = observation_error / float(config.response_sign)
    if direction not in (-1, 1):
        return LateralCommand("invalid_direction", 0.0, observation_error, vehicle_error)
    if config.response_sign not in (-1, 1):
        return LateralCommand("invalid_response_sign", 0.0, observation_error, vehicle_error)
    if observation.source != config.expected_source:
        return LateralCommand("source_mismatch", 0.0, observation_error, vehicle_error)
    if not observation.sensor_healthy or not observation.valid:
        return LateralCommand("invalid_observation", 0.0, observation_error, vehicle_error)
    values = (
        observation.forward_gap_m,
        observation.lateral_error_m,
        observation.yaw_error_rad,
        observation.confidence,
        target_lateral_m,
    )
    if not all(math.isfinite(value) for value in values):
        return LateralCommand("invalid_observation", 0.0, observation_error, vehicle_error)
    if observation.confidence < config.min_confidence:
        return LateralCommand("confidence_gate", 0.0, observation_error, vehicle_error)
    if not config.min_forward_gap_m <= observation.forward_gap_m <= config.max_forward_gap_m:
        return LateralCommand("forward_gap_gate", 0.0, observation_error, vehicle_error)
    if abs(observation.lateral_error_m) > config.max_abs_lateral_error_m:
        return LateralCommand("lateral_gate", 0.0, observation_error, vehicle_error)
    if abs(observation.yaw_error_rad) > config.max_abs_yaw_error_rad:
        return LateralCommand("yaw_gate", 0.0, observation_error, vehicle_error)
    if abs(observation_error) <= config.tolerance_m:
        return LateralCommand("at_target", 0.0, observation_error, vehicle_error)

    required_direction = 1 if vehicle_error > 0.0 else -1
    if required_direction != direction:
        return LateralCommand("direction_mismatch", 0.0, observation_error, vehicle_error)
    speed = min(config.max_speed_mps, max(config.min_speed_mps, config.kp * abs(vehicle_error)))
    return LateralCommand("drive", math.copysign(speed, direction), observation_error, vehicle_error)


def build_lateral_targets(
    baseline_lateral_m: float,
    amplitude_m: float,
    cycles: int,
    response_sign: int,
) -> list[tuple[float, int, str]]:
    targets = []
    for cycle in range(1, cycles + 1):
        targets.append(
            (
                baseline_lateral_m + response_sign * amplitude_m,
                1,
                f"left_{cycle}",
            )
        )
        targets.append(
            (
                baseline_lateral_m - response_sign * amplitude_m,
                -1,
                f"right_{cycle}",
            )
        )
    targets.append((baseline_lateral_m, 1, "return_baseline"))
    return targets


def fit_lateral_motion(
    wheel_lateral_m,
    observation_lateral_delta_m,
    response_sign: int,
) -> dict:
    pairs = [
        (float(wheel), float(delta) / float(response_sign))
        for wheel, delta in zip(wheel_lateral_m, observation_lateral_delta_m)
        if math.isfinite(float(wheel)) and math.isfinite(float(delta))
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


def probe_response_ratio(wheel_lateral_m: float, observation_delta_m: float) -> float:
    if not math.isfinite(wheel_lateral_m) or abs(wheel_lateral_m) <= 1.0e-9:
        return math.nan
    return observation_delta_m / wheel_lateral_m


def crossed_target_accepted(
    reached_target: bool,
    crossed_target: bool,
    wheel_stopped: bool,
    final_error_m: float,
    max_overshoot_m: float,
) -> bool:
    if not wheel_stopped:
        return False
    return reached_target or (
        crossed_target
        and math.isfinite(final_error_m)
        and abs(final_error_m) <= max_overshoot_m
    )


def is_lateral_motion_mode(mode_code: int) -> bool:
    return mode_code in {1, 3}


def is_active_lateral_fit_sample(
    mode_code: int,
    mode_matched: bool,
    wheel_linear_y_mps: float,
    minimum_speed_mps: float,
) -> bool:
    return (
        is_lateral_motion_mode(mode_code)
        and mode_matched
        and math.isfinite(wheel_linear_y_mps)
        and abs(wheel_linear_y_mps) >= minimum_speed_mps
    )


def _safe_label(value: str) -> str:
    cleaned = "".join(
        character if character.isalnum() or character in "_-" else "_"
        for character in value
    )
    return cleaned.strip("_") or "lateral_fit"


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


def run_ros_fit(args: argparse.Namespace) -> tuple[dict, list[dict]]:
    import rclpy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
    from robot_interfaces.msg import DockTargetObservation
    from sensor_msgs.msg import BatteryState
    from std_msgs.msg import Bool, String

    class FitNode(Node):
        def __init__(self):
            super().__init__("orbbec_dock_lateral_fit")
            self.observation = None
            self.observation_at = 0.0
            self.observation_seq = 0
            self.wheel_linear_x = math.nan
            self.wheel_linear_y = math.nan
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
            self.ranger_mode_code = -1
            self.ranger_mode_name = ""
            self.ranger_mode_matched = False
            self.estop_active = False
            self.battery_contact = False
            self.command_pub = self.create_publisher(Twist, args.cmd_topic, QoSProfile(depth=1))
            latched_qos = QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )
            self.create_subscription(
                DockTargetObservation,
                args.observation_topic,
                self._observation,
                QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE),
            )
            self.create_subscription(
                Odometry, args.wheel_odom_topic, self._wheel, qos_profile_sensor_data
            )
            self.create_subscription(Twist, args.final_cmd_topic, self._final_cmd, QoSProfile(depth=5))
            self.create_subscription(String, args.docking_status_topic, self._docking_status, latched_qos)
            self.create_subscription(String, args.safety_status_topic, self._safety_status, QoSProfile(depth=5))
            self.create_subscription(String, args.ranger_status_topic, self._ranger_status, QoSProfile(depth=5))
            self.create_subscription(Bool, args.estop_topic, self._estop, QoSProfile(depth=5))
            self.create_subscription(BatteryState, args.battery_topic, self._battery, qos_profile_sensor_data)

        def _observation(self, message):
            self.observation = ObservationSnapshot(
                sensor_healthy=message.sensor_healthy,
                valid=message.valid,
                forward_gap_m=message.forward_gap_m,
                lateral_error_m=message.lateral_error_m,
                yaw_error_rad=message.yaw_error_rad,
                lateral_span_m=message.lateral_span_m,
                confidence=message.confidence,
                source=message.source,
                reason=message.reason,
            )
            self.observation_at = time.monotonic()
            self.observation_seq += 1

        def _wheel(self, message):
            self.wheel_linear_x = message.twist.twist.linear.x
            self.wheel_linear_y = message.twist.twist.linear.y
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

        def _ranger_status(self, message):
            try:
                status = json.loads(message.data)
            except (TypeError, ValueError):
                return
            actual = status.get("actual_motion_mode") or {}
            self.ranger_mode_code = int(actual.get("code", -1))
            self.ranger_mode_name = str(actual.get("name", ""))
            self.ranger_mode_matched = bool(status.get("motion_mode_matched", False))

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

        def publish_lateral(self, speed_mps: float):
            message = Twist()
            message.linear.y = float(speed_mps)
            self.command_pub.publish(message)

    config = LateralServoConfig(
        response_sign=args.response_sign,
        tolerance_m=args.tolerance_m,
        kp=args.kp,
        min_speed_mps=args.min_speed_mps,
        max_speed_mps=args.max_speed_mps,
        min_forward_gap_m=args.min_forward_gap_m,
        max_forward_gap_m=args.max_forward_gap_m,
        max_abs_lateral_error_m=args.max_abs_lateral_error_m,
        max_abs_yaw_error_rad=args.max_abs_yaw_error_rad,
        min_confidence=args.min_confidence,
    )

    def spin_for(node, duration_sec: float, lateral_speed_mps: float = 0.0):
        deadline = time.monotonic() + duration_sec
        period = 1.0 / args.command_rate_hz
        while rclpy.ok() and time.monotonic() < deadline:
            loop_started = time.monotonic()
            node.publish_lateral(lateral_speed_mps)
            rclpy.spin_once(node, timeout_sec=min(0.03, period))
            remaining = period - (time.monotonic() - loop_started)
            if remaining > 0.0:
                time.sleep(remaining)

    def wait_stopped(node) -> bool:
        deadline = time.monotonic() + args.stop_settle_timeout_sec
        stable = 0
        while rclpy.ok() and time.monotonic() < deadline:
            node.publish_lateral(0.0)
            rclpy.spin_once(node, timeout_sec=0.05)
            stopped = (
                math.isfinite(node.wheel_linear_x)
                and math.isfinite(node.wheel_linear_y)
                and math.isfinite(node.wheel_angular_z)
                and math.hypot(node.wheel_linear_x, node.wheel_linear_y)
                <= args.stop_linear_threshold_mps
                and abs(node.wheel_angular_z) <= args.stop_angular_threshold_radps
            )
            stable = stable + 1 if stopped else 0
            if stable >= args.stop_stable_samples:
                return True
        return False

    def sample_row(node, started_at, leg_index, phase, target_lateral, direction, command_speed):
        observation = node.observation
        return {
            "elapsed_sec": time.monotonic() - started_at,
            "leg_index": leg_index,
            "phase": phase,
            "target_lateral_m": target_lateral,
            "direction": direction,
            "observation_seq": node.observation_seq,
            "observation_age_sec": time.monotonic() - node.observation_at,
            "forward_gap_m": observation.forward_gap_m if observation else math.nan,
            "lateral_error_m": observation.lateral_error_m if observation else math.nan,
            "yaw_error_rad": observation.yaw_error_rad if observation else math.nan,
            "lateral_span_m": observation.lateral_span_m if observation else math.nan,
            "confidence": observation.confidence if observation else math.nan,
            "observation_sensor_healthy": observation.sensor_healthy if observation else False,
            "observation_valid": observation.valid if observation else False,
            "observation_source": observation.source if observation else "",
            "observation_reason": observation.reason if observation else "",
            "command_linear_y_mps": command_speed,
            "wheel_linear_x_mps": node.wheel_linear_x,
            "wheel_linear_y_mps": node.wheel_linear_y,
            "wheel_angular_z_radps": node.wheel_angular_z,
            "wheel_x_m": node.wheel_x,
            "wheel_y_m": node.wheel_y,
            "wheel_yaw_rad": node.wheel_yaw,
            "final_linear_x_mps": node.final_linear_x,
            "final_linear_y_mps": node.final_linear_y,
            "final_angular_z_radps": node.final_angular_z,
            "ranger_mode_code": node.ranger_mode_code,
            "ranger_mode_name": node.ranger_mode_name,
            "ranger_mode_matched": node.ranger_mode_matched,
            "safety_status": node.safety_status,
            "docking_status_seq": node.docking_status_seq,
            "docking_status": node.docking_status,
        }

    def live_gate_reason(node) -> str:
        if node.estop_active:
            return "estop_active"
        if node.battery_contact:
            return "charging_contact"
        if node.safety_status in {"ESTOP_ACTIVE", "LOCALIZATION_INVALID"}:
            return f"safety_block:{node.safety_status}"
        return ""

    result = {
        "schema": "njrh.orbbec_dock_lateral_fit.v1",
        "success": False,
        "fit_accepted": False,
        "reason": "not_started",
        "started_at_utc": datetime.now(timezone.utc).isoformat(),
        "probe_only": args.probe_only,
        "response_sign": args.response_sign,
        "sweep_amplitude_m": args.sweep_amplitude_m,
        "single_target_lateral_m": args.single_target_lateral_m,
        "cycles": args.cycles,
        "legs": [],
    }
    samples = []
    started_at = time.monotonic()
    node = None
    rclpy.init()
    try:
        node = FitNode()
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
        if live_gate_reason(node):
            result["reason"] = f"preflight_{live_gate_reason(node)}"
            return result, samples
        preflight_command = compute_lateral_command(
            node.observation,
            node.observation.lateral_error_m,
            1,
            config,
        )
        if preflight_command.state != "at_target":
            result["reason"] = f"preflight_{preflight_command.state}"
            return result, samples

        result["initial_observation"] = asdict(node.observation)
        result["initial_map_pose"] = _api_get(args.api_url, "/api/v1/robot/pose")
        initial_docking_status_seq = node.docking_status_seq
        spin_for(node, args.zero_claim_sec)
        if not wait_stopped(node):
            result["reason"] = "preflight_chassis_not_stopped"
            return result, samples
        if args.dry_run:
            result["success"] = True
            result["reason"] = "dry_run_ok"
            result["final_observation"] = asdict(node.observation)
            return result, samples
        if not args.enable_motion:
            result["reason"] = "motion_not_explicitly_enabled"
            return result, samples

        if args.probe_only:
            probe_start_x = node.wheel_x
            probe_start_y = node.wheel_y
            probe_start_yaw = node.wheel_yaw
            probe_start_lateral = node.observation.lateral_error_m
            probe_target_lateral = (
                probe_start_lateral + args.response_sign * args.probe_distance_m
            )
            last_recorded_seq = -1
            deadline = time.monotonic() + args.probe_timeout_sec
            next_api_gate_at = time.monotonic()
            probe_reason = "probe_timeout"
            while rclpy.ok() and time.monotonic() < deadline:
                loop_started = time.monotonic()
                rclpy.spin_once(node, timeout_sec=0.02)
                now = time.monotonic()
                if now >= next_api_gate_at:
                    gate_ok, gate_reason, _, _ = _runtime_gate(args.api_url)
                    next_api_gate_at = now + args.api_gate_period_sec
                    if not gate_ok:
                        probe_reason = gate_reason
                        break
                if node.docking_status_seq > initial_docking_status_seq:
                    status = node.docking_status
                    if "phase=active" in status or status.startswith(("docking ", "undocking ")):
                        probe_reason = f"docking_owner_became_active:{status}"
                        break
                reason = live_gate_reason(node)
                if reason:
                    probe_reason = reason
                    break
                if node.observation is None or now - node.observation_at > args.observation_timeout_sec:
                    node.publish_lateral(0.0)
                    probe_reason = "observation_stale"
                    break
                command = compute_lateral_command(
                    node.observation,
                    probe_target_lateral,
                    1,
                    config,
                )
                if command.state not in {"drive", "at_target"}:
                    node.publish_lateral(0.0)
                    probe_reason = command.state
                    break
                forward, lateral = project_body_displacement(
                    probe_start_x,
                    probe_start_y,
                    probe_start_yaw,
                    node.wheel_x,
                    node.wheel_y,
                )
                if abs(forward) > args.max_forward_drift_m:
                    node.publish_lateral(0.0)
                    probe_reason = "probe_forward_drift"
                    break
                if abs(normalize_angle(node.wheel_yaw - probe_start_yaw)) > args.max_yaw_drift_rad:
                    node.publish_lateral(0.0)
                    probe_reason = "probe_yaw_drift"
                    break
                if lateral >= args.probe_distance_m:
                    node.publish_lateral(0.0)
                    probe_reason = "probe_distance_reached"
                    break
                node.publish_lateral(args.probe_speed_mps)
                if node.observation_seq != last_recorded_seq:
                    samples.append(
                        sample_row(
                            node,
                            started_at,
                            1,
                            "positive_y_probe",
                            probe_target_lateral,
                            1,
                            args.probe_speed_mps,
                        )
                    )
                    last_recorded_seq = node.observation_seq
                remaining = 1.0 / args.command_rate_hz - (time.monotonic() - loop_started)
                if remaining > 0.0:
                    time.sleep(remaining)

            spin_for(node, args.between_leg_zero_sec)
            wheel_stopped = wait_stopped(node)
            _, wheel_lateral = project_body_displacement(
                probe_start_x,
                probe_start_y,
                probe_start_yaw,
                node.wheel_x,
                node.wheel_y,
            )
            observation_delta = node.observation.lateral_error_m - probe_start_lateral
            ratio = probe_response_ratio(wheel_lateral, observation_delta)
            sign_ok = math.isfinite(ratio) and math.copysign(1.0, ratio) == args.response_sign
            magnitude_ok = math.isfinite(ratio) and args.probe_ratio_min <= abs(ratio) <= args.probe_ratio_max
            result["probe"] = {
                "reason": probe_reason,
                "wheel_lateral_m": wheel_lateral,
                "observation_lateral_delta_m": observation_delta,
                "response_ratio": ratio,
                "response_sign_confirmed": sign_ok,
                "response_magnitude_accepted": magnitude_ok,
                "wheel_stopped": wheel_stopped,
            }
            result["success"] = bool(
                probe_reason == "probe_distance_reached"
                and wheel_stopped
                and sign_ok
                and magnitude_ok
            )
            result["fit_accepted"] = result["success"]
            result["reason"] = "probe_complete" if result["success"] else f"probe_failed:{probe_reason}"
            result["final_observation"] = asdict(node.observation)
            result["final_map_pose"] = _api_get(args.api_url, "/api/v1/robot/pose")
            return result, samples

        baseline_lateral = node.observation.lateral_error_m
        if args.single_target_lateral_m is None:
            targets = build_lateral_targets(
                baseline_lateral,
                args.sweep_amplitude_m,
                args.cycles,
                args.response_sign,
            )
        else:
            vehicle_error = (
                args.single_target_lateral_m - baseline_lateral
            ) / float(args.response_sign)
            direction = 1 if vehicle_error >= 0.0 else -1
            targets = [(args.single_target_lateral_m, direction, "single_target")]
        if any(abs(target) > args.max_abs_lateral_error_m for target, _, _ in targets):
            result["reason"] = "target_exceeds_lateral_gate"
            return result, samples
        result["baseline_lateral_m"] = baseline_lateral

        total_wheel_distance = 0.0
        previous_wheel_x = node.wheel_x
        previous_wheel_y = node.wheel_y
        for leg_index, (target_lateral, direction, phase) in enumerate(targets, start=1):
            leg_start_observation = node.observation
            leg_start_x = node.wheel_x
            leg_start_y = node.wheel_y
            leg_start_yaw = node.wheel_yaw
            leg_samples = []
            stable_samples = 0
            last_target_seq = -1
            last_recorded_seq = -1
            last_valid_at = time.monotonic()
            progress_reference_at = time.monotonic()
            progress_reference_error = abs(target_lateral - leg_start_observation.lateral_error_m)
            next_api_gate_at = time.monotonic()
            leg_reason = "timeout"
            leg_success = False
            leg_had_drive = False
            leg_crossed_target = False
            deadline = time.monotonic() + args.leg_timeout_sec

            while rclpy.ok() and time.monotonic() < deadline:
                loop_started = time.monotonic()
                rclpy.spin_once(node, timeout_sec=0.02)
                now = time.monotonic()
                command_speed = 0.0
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
                gate_reason = live_gate_reason(node)
                if gate_reason:
                    leg_reason = gate_reason
                    break
                if node.observation is None or now - node.observation_at > args.observation_timeout_sec:
                    node.publish_lateral(0.0)
                    if now - last_valid_at > args.invalid_abort_sec:
                        leg_reason = "observation_stale"
                        break
                else:
                    command = compute_lateral_command(
                        node.observation,
                        target_lateral,
                        direction,
                        config,
                    )
                    if command.state in {"invalid_observation", "confidence_gate"}:
                        node.publish_lateral(0.0)
                        if now - last_valid_at > args.invalid_abort_sec:
                            leg_reason = command.state
                            break
                    elif command.state == "at_target":
                        last_valid_at = now
                        if node.observation_seq != last_target_seq:
                            stable_samples += 1
                            last_target_seq = node.observation_seq
                        node.publish_lateral(0.0)
                        if stable_samples >= args.stable_samples:
                            leg_success = True
                            leg_reason = "target_reached"
                    elif command.state == "drive":
                        last_valid_at = now
                        stable_samples = 0
                        last_target_seq = -1
                        leg_had_drive = True
                        command_speed = command.speed_mps
                        node.publish_lateral(command_speed)
                    elif command.state == "direction_mismatch" and leg_had_drive:
                        node.publish_lateral(0.0)
                        leg_crossed_target = True
                        leg_reason = "target_crossed"
                        break
                    else:
                        node.publish_lateral(0.0)
                        leg_reason = command.state
                        break

                    if node.observation_seq != last_recorded_seq:
                        row = sample_row(
                            node,
                            started_at,
                            leg_index,
                            phase,
                            target_lateral,
                            direction,
                            command_speed,
                        )
                        samples.append(row)
                        leg_samples.append(row)
                        last_recorded_seq = node.observation_seq

                    forward, lateral = project_body_displacement(
                        leg_start_x,
                        leg_start_y,
                        leg_start_yaw,
                        node.wheel_x,
                        node.wheel_y,
                    )
                    if abs(forward) > args.max_forward_drift_m:
                        leg_reason = "forward_drift_limit"
                        break
                    if abs(lateral) > args.max_leg_lateral_distance_m:
                        leg_reason = "leg_lateral_distance_limit"
                        break
                    yaw_drift = abs(normalize_angle(node.wheel_yaw - leg_start_yaw))
                    if yaw_drift > args.max_yaw_drift_rad:
                        leg_reason = "yaw_drift_limit"
                        break
                    if (
                        now - progress_reference_at >= args.no_progress_timeout_sec
                        and not leg_success
                    ):
                        current_error = abs(target_lateral - node.observation.lateral_error_m)
                        if progress_reference_error - current_error < args.min_progress_m:
                            leg_reason = "no_lateral_progress"
                            break
                        progress_reference_at = now
                        progress_reference_error = current_error

                if math.isfinite(node.wheel_x) and math.isfinite(previous_wheel_x):
                    total_wheel_distance += math.hypot(
                        node.wheel_x - previous_wheel_x,
                        node.wheel_y - previous_wheel_y,
                    )
                    previous_wheel_x = node.wheel_x
                    previous_wheel_y = node.wheel_y
                    if total_wheel_distance > args.max_total_wheel_distance_m:
                        leg_reason = "maximum_total_wheel_distance_reached"
                        break
                if leg_success:
                    break
                remaining = 1.0 / args.command_rate_hz - (time.monotonic() - loop_started)
                if remaining > 0.0:
                    time.sleep(remaining)

            spin_for(node, args.between_leg_zero_sec)
            wheel_stopped = wait_stopped(node)
            final_observation = node.observation
            forward, wheel_lateral = project_body_displacement(
                leg_start_x,
                leg_start_y,
                leg_start_yaw,
                node.wheel_x,
                node.wheel_y,
            )
            fit_wheel = []
            fit_observation = []
            active_fit_wheel = []
            active_fit_observation = []
            gap_values = []
            lateral_span_values = []
            yaw_values = []
            confidence_values = []
            mode_codes = []
            for row in leg_samples:
                required = ("wheel_x_m", "wheel_y_m", "lateral_error_m")
                if not all(math.isfinite(float(row[key])) for key in required):
                    continue
                _, projected_lateral = project_body_displacement(
                    leg_start_x,
                    leg_start_y,
                    leg_start_yaw,
                    float(row["wheel_x_m"]),
                    float(row["wheel_y_m"]),
                )
                fit_wheel.append(projected_lateral)
                observation_delta = (
                    float(row["lateral_error_m"]) - leg_start_observation.lateral_error_m
                )
                fit_observation.append(observation_delta)
                if is_active_lateral_fit_sample(
                    int(row["ranger_mode_code"]),
                    bool(row["ranger_mode_matched"]),
                    float(row["wheel_linear_y_mps"]),
                    args.fit_min_wheel_lateral_speed_mps,
                ):
                    active_fit_wheel.append(projected_lateral)
                    active_fit_observation.append(observation_delta)
                gap_values.append(float(row["forward_gap_m"]))
                lateral_span_values.append(float(row["lateral_span_m"]))
                yaw_values.append(float(row["yaw_error_rad"]))
                confidence_values.append(float(row["confidence"]))
                mode_codes.append(int(row["ranger_mode_code"]))
            all_sample_fit = fit_lateral_motion(
                fit_wheel, fit_observation, args.response_sign
            )
            fit = fit_lateral_motion(
                active_fit_wheel, active_fit_observation, args.response_sign
            )
            fit_ok = (
                fit["sample_count"] >= args.fit_min_active_samples
                and
                math.isfinite(fit["scale"])
                and args.fit_scale_min <= fit["scale"] <= args.fit_scale_max
                and math.isfinite(fit["r_squared"])
                and fit["r_squared"] >= args.fit_min_r_squared
            )
            final_error = final_observation.lateral_error_m - target_lateral
            motion_success = crossed_target_accepted(
                leg_success,
                leg_crossed_target,
                wheel_stopped,
                final_error,
                args.max_target_overshoot_m,
            )
            if leg_crossed_target and not motion_success:
                leg_reason = "target_overshoot"
            leg = {
                "index": leg_index,
                "phase": phase,
                "direction": direction,
                "target_lateral_m": target_lateral,
                "initial_lateral_m": leg_start_observation.lateral_error_m,
                "final_lateral_m": final_observation.lateral_error_m,
                "final_error_m": final_error,
                "wheel_lateral_m": wheel_lateral,
                "wheel_forward_drift_m": forward,
                "observed_vehicle_lateral_m": (
                    final_observation.lateral_error_m - leg_start_observation.lateral_error_m
                ) / float(args.response_sign),
                "wheel_stopped": wheel_stopped,
                "target_crossed": leg_crossed_target,
                "success": motion_success,
                "reason": leg_reason if wheel_stopped else "wheel_stop_not_confirmed",
                "fit": fit,
                "fit_all_samples": all_sample_fit,
                "fit_accepted": fit_ok,
                "forward_gap_median": statistics.median(gap_values) if gap_values else math.nan,
                "forward_gap_span_m": max(gap_values) - min(gap_values) if gap_values else math.nan,
                "lateral_span_median": statistics.median(lateral_span_values) if lateral_span_values else math.nan,
                "lateral_span_range_m": max(lateral_span_values) - min(lateral_span_values) if lateral_span_values else math.nan,
                "yaw_error_rad_median": statistics.median(yaw_values) if yaw_values else math.nan,
                "yaw_error_rad_max_abs": max(map(abs, yaw_values)) if yaw_values else math.nan,
                "confidence_median": statistics.median(confidence_values) if confidence_values else math.nan,
                "lateral_mode_sample_count": sum(is_lateral_motion_mode(code) for code in mode_codes),
                "parallel_mode_sample_count": sum(code == 1 for code in mode_codes),
                "side_slip_mode_sample_count": sum(code == 3 for code in mode_codes),
                "sample_count": len(leg_samples),
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
                spin_for(node, args.zero_burst_sec)
                result["final_wheel_stopped"] = wait_stopped(node)
            finally:
                node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        result["elapsed_sec"] = time.monotonic() - started_at


def _write_report(output_dir: Path, result: dict, samples: list[dict], args: argparse.Namespace):
    output_dir.mkdir(parents=True, exist_ok=False)
    fields = [
        "elapsed_sec",
        "leg_index",
        "phase",
        "target_lateral_m",
        "direction",
        "observation_seq",
        "observation_age_sec",
        "forward_gap_m",
        "lateral_error_m",
        "yaw_error_rad",
        "lateral_span_m",
        "confidence",
        "observation_sensor_healthy",
        "observation_valid",
        "observation_source",
        "observation_reason",
        "command_linear_y_mps",
        "wheel_linear_x_mps",
        "wheel_linear_y_mps",
        "wheel_angular_z_radps",
        "wheel_x_m",
        "wheel_y_m",
        "wheel_yaw_rad",
        "final_linear_x_mps",
        "final_linear_y_mps",
        "final_angular_z_radps",
        "ranger_mode_code",
        "ranger_mode_name",
        "ranger_mode_matched",
        "safety_status",
        "docking_status_seq",
        "docking_status",
    ]
    with (output_dir / "samples.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(samples)
    with (output_dir / "metrics.json").open("w", encoding="utf-8") as stream:
        json.dump(result, stream, indent=2, sort_keys=True)
        stream.write("\n")

    lines = [
        "# Orbbec dock lateral fit",
        "",
        f"- success: `{result.get('success', False)}`",
        f"- reason: `{result.get('reason', '')}`",
        f"- fit accepted: `{result.get('fit_accepted', False)}`",
        f"- response sign: `{result.get('response_sign', 0)}`",
        f"- elapsed: `{result.get('elapsed_sec', math.nan):.2f} s`",
        "",
    ]
    if result.get("probe"):
        probe = result["probe"]
        lines.extend(
            [
                "## Direction probe",
                "",
                f"- wheel lateral: `{probe.get('wheel_lateral_m', math.nan):.4f} m`",
                f"- observation delta: `{probe.get('observation_lateral_delta_m', math.nan):.4f} m`",
                f"- response ratio: `{probe.get('response_ratio', math.nan):.4f}`",
                f"- sign confirmed: `{probe.get('response_sign_confirmed', False)}`",
                "",
            ]
        )
    if result.get("legs"):
        lines.extend(
            [
                "| Leg | Phase | Initial lateral | Final lateral | Wheel lateral | Camera-derived lateral | Motion scale | Motion R2 | All scale | Forward drift | Motion OK |",
                "|---:|---|---:|---:|---:|---:|---:|---:|---:|---:|---|",
            ]
        )
        for leg in result["legs"]:
            fit = leg.get("fit", {})
            all_fit = leg.get("fit_all_samples", {})
            lines.append(
                "| {index} | {phase} | {initial:.4f} | {final:.4f} | {wheel:.4f} | "
                "{observed:.4f} | {scale:.4f} | {r2:.4f} | {all_scale:.4f} | "
                "{forward:.4f} | {ok} |".format(
                    index=leg["index"],
                    phase=leg["phase"],
                    initial=leg["initial_lateral_m"],
                    final=leg["final_lateral_m"],
                    wheel=leg["wheel_lateral_m"],
                    observed=leg["observed_vehicle_lateral_m"],
                    scale=fit.get("scale", math.nan),
                    r2=fit.get("r_squared", math.nan),
                    all_scale=all_fit.get("scale", math.nan),
                    forward=leg["wheel_forward_drift_m"],
                    ok=leg["success"],
                )
            )
        lines.append("")
    lines.extend(
        [
            "Motion path: `/cmd_vel_docking -> robot_safety -> /cmd_vel -> ranger_base_node`.",
            "Files: `samples.csv`, `metrics.json`.",
        ]
    )
    (output_dir / "summary.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    (output_dir / "metadata.env").write_text(
        "\n".join(
            [
                f"label={_safe_label(args.label)}",
                f"probe_only={str(args.probe_only).lower()}",
                f"response_sign={args.response_sign}",
                f"sweep_amplitude_m={args.sweep_amplitude_m}",
                f"cycles={args.cycles}",
            ]
        ) + "\n",
        encoding="utf-8",
    )


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Supervised Orbbec left/right fixed-face fit through robot_safety"
    )
    parser.add_argument("--probe-only", action="store_true")
    parser.add_argument("--enable-motion", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--probe-distance-m", type=float, default=0.020)
    parser.add_argument("--probe-speed-mps", type=float, default=0.025)
    parser.add_argument("--probe-timeout-sec", type=float, default=5.0)
    parser.add_argument("--probe-ratio-min", type=float, default=0.50)
    parser.add_argument("--probe-ratio-max", type=float, default=1.50)
    parser.add_argument("--sweep-amplitude-m", type=float, default=0.080)
    parser.add_argument("--single-target-lateral-m", type=float)
    parser.add_argument("--cycles", type=int, default=1)
    parser.add_argument("--response-sign", type=int, choices=(-1, 1), default=-1)
    parser.add_argument("--label", default="lateral_fit")
    parser.add_argument("--output-root", default="/tmp/njrh_reports/orbbec_dock_lateral_fit")
    parser.add_argument("--api-url", default="http://127.0.0.1:8080")
    parser.add_argument("--observation-topic", default="/dock/target_observation")
    parser.add_argument("--cmd-topic", default="/cmd_vel_docking")
    parser.add_argument("--final-cmd-topic", default="/cmd_vel")
    parser.add_argument("--wheel-odom-topic", default="/wheel/odom")
    parser.add_argument("--docking-status-topic", default="/docking/status")
    parser.add_argument("--safety-status-topic", default="/safety/status")
    parser.add_argument("--ranger-status-topic", default="/ranger_base/status")
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
    parser.add_argument("--between-leg-zero-sec", type=float, default=0.80)
    parser.add_argument("--zero-burst-sec", type=float, default=0.80)
    parser.add_argument("--stop-settle-timeout-sec", type=float, default=4.0)
    parser.add_argument("--stop-stable-samples", type=int, default=6)
    parser.add_argument("--stop-linear-threshold-mps", type=float, default=0.008)
    parser.add_argument("--stop-angular-threshold-radps", type=float, default=0.020)
    parser.add_argument("--tolerance-m", type=float, default=0.006)
    parser.add_argument("--kp", type=float, default=0.50)
    parser.add_argument("--min-speed-mps", type=float, default=0.025)
    parser.add_argument("--max-speed-mps", type=float, default=0.040)
    parser.add_argument("--min-forward-gap-m", type=float, default=0.65)
    parser.add_argument("--max-forward-gap-m", type=float, default=0.95)
    parser.add_argument("--max-abs-lateral-error-m", type=float, default=0.22)
    parser.add_argument("--max-abs-yaw-error-rad", type=float, default=0.08)
    parser.add_argument("--min-confidence", type=float, default=0.50)
    parser.add_argument("--max-forward-drift-m", type=float, default=0.035)
    parser.add_argument("--max-yaw-drift-rad", type=float, default=0.05)
    parser.add_argument("--max-leg-lateral-distance-m", type=float, default=0.22)
    parser.add_argument("--max-target-overshoot-m", type=float, default=0.020)
    parser.add_argument("--max-total-wheel-distance-m", type=float, default=0.50)
    parser.add_argument("--stable-samples", type=int, default=8)
    parser.add_argument("--no-progress-timeout-sec", type=float, default=7.0)
    parser.add_argument("--min-progress-m", type=float, default=0.010)
    parser.add_argument("--charging-current-min-a", type=float, default=0.10)
    parser.add_argument("--fit-scale-min", type=float, default=0.70)
    parser.add_argument("--fit-scale-max", type=float, default=1.30)
    parser.add_argument("--fit-min-r-squared", type=float, default=0.90)
    parser.add_argument("--fit-min-wheel-lateral-speed-mps", type=float, default=0.005)
    parser.add_argument("--fit-min-active-samples", type=int, default=6)
    args = parser.parse_args()

    if args.cmd_topic != "/cmd_vel_docking":
        parser.error("calibration motion must use /cmd_vel_docking through robot_safety")
    if args.probe_distance_m <= 0.0 or args.sweep_amplitude_m <= 0.0:
        parser.error("probe distance and sweep amplitude must be positive")
    if args.probe_speed_mps <= 0.0 or args.max_speed_mps <= 0.0:
        parser.error("lateral speeds must be positive")
    if args.probe_speed_mps > args.max_speed_mps:
        parser.error("probe speed cannot exceed max lateral speed")
    if args.min_speed_mps > args.max_speed_mps:
        parser.error("minimum lateral speed cannot exceed maximum")
    if args.cycles < 1:
        parser.error("cycles must be at least one")
    if args.command_rate_hz <= 0.0:
        parser.error("command rate must be positive")

    output_dir = Path(args.output_root) / f"{_timestamp_tag()}_{_safe_label(args.label)}"
    try:
        result, samples = run_ros_fit(args)
    except KeyboardInterrupt:
        result = {
            "schema": "njrh.orbbec_dock_lateral_fit.v1",
            "success": False,
            "fit_accepted": False,
            "reason": "interrupted",
            "elapsed_sec": math.nan,
            "legs": [],
        }
        samples = []
    _write_report(output_dir, result, samples, args)
    print(f"[orbbec-dock-lateral-fit] summary: {output_dir / 'summary.md'}")
    print(f"[orbbec-dock-lateral-fit] report: {output_dir}")
    return 0 if result.get("success") else 1


if __name__ == "__main__":
    raise SystemExit(main())
