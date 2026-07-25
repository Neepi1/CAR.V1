#!/usr/bin/env python3

import argparse
import json
import math
import sys
import time
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


@dataclass(frozen=True)
class RangeServoConfig:
    target_gap_m: float
    expected_source: str = "orbbec_336l_depth"
    tolerance_m: float = 0.008
    overshoot_limit_m: float = 0.015
    kp: float = 0.35
    min_speed_mps: float = 0.012
    max_speed_mps: float = 0.040
    max_lateral_error_m: float = 0.100
    max_yaw_error_rad: float = 0.120
    min_confidence: float = 0.50


@dataclass(frozen=True)
class RangeCommand:
    state: str
    speed_mps: float
    error_m: float


def compute_forward_command(
    observation: ObservationSnapshot,
    config: RangeServoConfig,
) -> RangeCommand:
    error = observation.forward_gap_m - config.target_gap_m
    if observation.source != config.expected_source:
        return RangeCommand("source_mismatch", 0.0, error)
    if not observation.sensor_healthy or not observation.valid:
        return RangeCommand("invalid_observation", 0.0, error)
    if not all(
        math.isfinite(value)
        for value in (
            observation.forward_gap_m,
            observation.lateral_error_m,
            observation.yaw_error_rad,
            observation.confidence,
        )
    ):
        return RangeCommand("invalid_observation", 0.0, error)
    if observation.confidence < config.min_confidence:
        return RangeCommand("confidence_gate", 0.0, error)
    if abs(observation.lateral_error_m) > config.max_lateral_error_m:
        return RangeCommand("lateral_gate", 0.0, error)
    if abs(observation.yaw_error_rad) > config.max_yaw_error_rad:
        return RangeCommand("yaw_gate", 0.0, error)
    if error < -config.overshoot_limit_m:
        return RangeCommand("overshoot", 0.0, error)
    if abs(error) <= config.tolerance_m:
        return RangeCommand("at_target", 0.0, error)
    if error < 0.0:
        return RangeCommand("at_target", 0.0, error)
    speed = min(config.max_speed_mps, max(config.min_speed_mps, config.kp * error))
    return RangeCommand("drive", speed, error)


def _safe_label(value: str) -> str:
    cleaned = "".join(character if character.isalnum() or character in "_-" else "_" for character in value)
    return cleaned.strip("_") or "range_step"


def _timestamp_tag() -> str:
    return datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")


def run_ros_servo(args: argparse.Namespace) -> dict:
    import rclpy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
    from robot_interfaces.msg import DockTargetObservation
    from sensor_msgs.msg import BatteryState
    from std_msgs.msg import Bool, String

    class RangeServoNode(Node):
        def __init__(self):
            super().__init__("orbbec_dock_range_step")
            self.observation = None
            self.observation_at = 0.0
            self.observation_seq = 0
            self.wheel_linear_x = math.nan
            self.wheel_angular_z = math.nan
            self.wheel_at = 0.0
            self.final_linear_x = math.nan
            self.final_linear_y = math.nan
            self.final_angular_z = math.nan
            self.final_cmd_at = 0.0
            self.docking_status = ""
            self.battery_contact = False
            self.estop_active = False
            self.safety_status = ""
            self.command_pub = self.create_publisher(Twist, args.cmd_topic, QoSProfile(depth=1))
            self.create_subscription(
                DockTargetObservation,
                args.observation_topic,
                self._observation,
                QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE),
            )
            self.create_subscription(Odometry, args.wheel_odom_topic, self._wheel, qos_profile_sensor_data)
            self.create_subscription(Twist, args.final_cmd_topic, self._final_cmd, QoSProfile(depth=5))
            latched_status_qos = QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )
            self.create_subscription(
                String,
                args.docking_status_topic,
                self._docking_status,
                latched_status_qos,
            )
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
            )
            self.observation_at = time.monotonic()
            self.observation_seq += 1

        def _wheel(self, message):
            self.wheel_linear_x = message.twist.twist.linear.x
            self.wheel_angular_z = message.twist.twist.angular.z
            self.wheel_at = time.monotonic()

        def _final_cmd(self, message):
            self.final_linear_x = message.linear.x
            self.final_linear_y = message.linear.y
            self.final_angular_z = message.angular.z
            self.final_cmd_at = time.monotonic()

        def _docking_status(self, message):
            self.docking_status = message.data.strip().lower()

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

        def publish_speed(self, speed_mps):
            message = Twist()
            message.linear.x = float(speed_mps)
            self.command_pub.publish(message)

    def spin_for(node, duration_sec, speed_mps=0.0):
        deadline = time.monotonic() + duration_sec
        period = 1.0 / args.command_rate_hz
        while rclpy.ok() and time.monotonic() < deadline:
            loop_started = time.monotonic()
            node.publish_speed(speed_mps)
            rclpy.spin_once(node, timeout_sec=min(0.03, period))
            remaining = period - (time.monotonic() - loop_started)
            if remaining > 0.0:
                time.sleep(remaining)

    rclpy.init()
    node = RangeServoNode()
    config = RangeServoConfig(
        target_gap_m=args.target_fixed_face_gap_m,
        tolerance_m=args.tolerance_m,
        overshoot_limit_m=args.overshoot_limit_m,
        kp=args.kp,
        min_speed_mps=args.min_speed_mps,
        max_speed_mps=args.max_speed_mps,
        max_lateral_error_m=args.max_lateral_error_m,
        max_yaw_error_rad=args.max_yaw_error_rad,
        min_confidence=args.min_confidence,
    )
    result = {
        "schema": "njrh.orbbec_dock_range_step.v1",
        "dry_run": args.dry_run,
        "target_fixed_face_gap_m": args.target_fixed_face_gap_m,
        "success": False,
        "reason": "not_started",
        "started_at_utc": datetime.now(timezone.utc).isoformat(),
        "initial_gap_m": math.nan,
        "final_gap_m": math.nan,
        "elapsed_sec": 0.0,
        "max_command_mps": 0.0,
    }
    started_at = time.monotonic()
    last_valid_at = 0.0
    stable_samples = 0
    progress_reference_at = 0.0
    progress_reference_gap = math.nan
    last_target_observation_seq = -1
    try:
        preflight_deadline = time.monotonic() + args.preflight_timeout_sec
        while rclpy.ok() and time.monotonic() < preflight_deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            now = time.monotonic()
            observation_fresh = node.observation is not None and now - node.observation_at <= args.observation_timeout_sec
            wheel_fresh = now - node.wheel_at <= args.wheel_timeout_sec
            if observation_fresh and wheel_fresh and node.docking_status:
                break
        now = time.monotonic()
        if node.observation is None or now - node.observation_at > args.observation_timeout_sec:
            result["reason"] = "preflight_observation_unavailable"
            return result
        if now - node.wheel_at > args.wheel_timeout_sec:
            result["reason"] = "preflight_wheel_odom_unavailable"
            return result
        if node.docking_status != "idle":
            result["reason"] = f"preflight_docking_not_idle:{node.docking_status or 'unknown'}"
            return result
        if node.estop_active:
            result["reason"] = "preflight_estop_active"
            return result
        if node.battery_contact:
            result["reason"] = "preflight_charging_contact"
            return result
        initial_command = compute_forward_command(node.observation, config)
        result["initial_gap_m"] = node.observation.forward_gap_m
        if initial_command.state not in ("drive", "at_target"):
            result["reason"] = f"preflight_{initial_command.state}"
            return result
        if initial_command.error_m > args.max_step_distance_m:
            result["reason"] = "preflight_step_distance_too_large"
            return result

        spin_for(node, args.zero_claim_sec, 0.0)
        if abs(node.wheel_linear_x) > args.stop_linear_threshold_mps or abs(node.wheel_angular_z) > args.stop_angular_threshold_radps:
            result["reason"] = "preflight_chassis_not_stopped"
            return result
        if args.dry_run:
            if time.monotonic() - node.final_cmd_at > args.wheel_timeout_sec:
                result["reason"] = "dry_run_final_command_unavailable"
                return result
            if any(
                abs(value) > args.final_zero_epsilon
                for value in (node.final_linear_x, node.final_linear_y, node.final_angular_z)
                if math.isfinite(value)
            ):
                result["reason"] = "dry_run_final_command_not_zero"
                return result
            result["success"] = True
            result["reason"] = "dry_run_ok"
            result["final_gap_m"] = node.observation.forward_gap_m
            return result

        deadline = time.monotonic() + args.timeout_sec
        last_valid_at = time.monotonic()
        progress_reference_at = time.monotonic()
        progress_reference_gap = node.observation.forward_gap_m
        while rclpy.ok() and time.monotonic() < deadline:
            loop_started = time.monotonic()
            rclpy.spin_once(node, timeout_sec=0.02)
            now = time.monotonic()
            if node.docking_status != "idle":
                result["reason"] = f"docking_owner_became_active:{node.docking_status}"
                break
            if node.estop_active:
                result["reason"] = "estop_active"
                break
            if node.battery_contact:
                result["reason"] = "charging_contact"
                break
            if node.safety_status in {"ESTOP_ACTIVE", "LOCALIZATION_INVALID"}:
                result["reason"] = f"safety_block:{node.safety_status}"
                break
            if node.observation is None or now - node.observation_at > args.observation_timeout_sec:
                node.publish_speed(0.0)
                if last_valid_at > 0.0 and now - last_valid_at > args.invalid_abort_sec:
                    result["reason"] = "observation_stale"
                    break
            else:
                command = compute_forward_command(node.observation, config)
                result["final_gap_m"] = node.observation.forward_gap_m
                if command.state in ("invalid_observation", "confidence_gate"):
                    node.publish_speed(0.0)
                    if last_valid_at > 0.0 and now - last_valid_at > args.invalid_abort_sec:
                        result["reason"] = command.state
                        break
                elif command.state == "at_target":
                    last_valid_at = now
                    if node.observation_seq != last_target_observation_seq:
                        stable_samples += 1
                        last_target_observation_seq = node.observation_seq
                    node.publish_speed(0.0)
                    if stable_samples >= args.stable_samples:
                        result["success"] = True
                        result["reason"] = "target_reached"
                        break
                elif command.state == "drive":
                    last_valid_at = now
                    stable_samples = 0
                    last_target_observation_seq = -1
                    node.publish_speed(command.speed_mps)
                    result["max_command_mps"] = max(result["max_command_mps"], command.speed_mps)
                    if now - progress_reference_at >= args.no_progress_timeout_sec:
                        progress = progress_reference_gap - node.observation.forward_gap_m
                        if progress < args.min_progress_m:
                            result["reason"] = "no_forward_progress"
                            break
                        progress_reference_at = now
                        progress_reference_gap = node.observation.forward_gap_m
                else:
                    result["reason"] = command.state
                    node.publish_speed(0.0)
                    break
            sleep_sec = 1.0 / args.command_rate_hz - (time.monotonic() - loop_started)
            if sleep_sec > 0.0:
                time.sleep(sleep_sec)
        else:
            result["reason"] = "timeout"
    finally:
        try:
            spin_for(node, args.zero_burst_sec, 0.0)
            stop_deadline = time.monotonic() + args.stop_settle_timeout_sec
            stopped_samples = 0
            while rclpy.ok() and time.monotonic() < stop_deadline:
                node.publish_speed(0.0)
                rclpy.spin_once(node, timeout_sec=0.05)
                if (
                    math.isfinite(node.wheel_linear_x)
                    and math.isfinite(node.wheel_angular_z)
                    and abs(node.wheel_linear_x) <= args.stop_linear_threshold_mps
                    and abs(node.wheel_angular_z) <= args.stop_angular_threshold_radps
                ):
                    stopped_samples += 1
                    if stopped_samples >= args.stop_stable_samples:
                        break
                else:
                    stopped_samples = 0
            result["wheel_stopped"] = stopped_samples >= args.stop_stable_samples
            result["final_wheel_linear_x_mps"] = node.wheel_linear_x
            result["final_wheel_angular_z_radps"] = node.wheel_angular_z
            if result["success"] and not result["wheel_stopped"]:
                result["success"] = False
                result["reason"] = "wheel_stop_not_confirmed"
        finally:
            result["elapsed_sec"] = time.monotonic() - started_at
            result["final_safety_status"] = node.safety_status
            result["final_docking_status"] = node.docking_status
            node.destroy_node()
            rclpy.shutdown()
    return result


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Forward-only Orbbec fixed-face range step through robot_safety")
    parser.add_argument("--target-fixed-face-gap-m", type=float, required=True)
    parser.add_argument("--label", default="range_step")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--output-root", default="/tmp/njrh_reports/orbbec_dock_range_step")
    parser.add_argument("--observation-topic", default="/dock/target_observation")
    parser.add_argument("--cmd-topic", default="/cmd_vel_docking")
    parser.add_argument("--final-cmd-topic", default="/cmd_vel")
    parser.add_argument("--wheel-odom-topic", default="/wheel/odom")
    parser.add_argument("--docking-status-topic", default="/docking/status")
    parser.add_argument("--safety-status-topic", default="/safety/status")
    parser.add_argument("--estop-topic", default="/safety/estop")
    parser.add_argument("--battery-topic", default="/battery_state")
    parser.add_argument("--preflight-timeout-sec", type=float, default=5.0)
    parser.add_argument("--timeout-sec", type=float, default=30.0)
    parser.add_argument("--observation-timeout-sec", type=float, default=0.50)
    parser.add_argument("--wheel-timeout-sec", type=float, default=0.50)
    parser.add_argument("--invalid-abort-sec", type=float, default=1.0)
    parser.add_argument("--command-rate-hz", type=float, default=20.0)
    parser.add_argument("--zero-claim-sec", type=float, default=0.60)
    parser.add_argument("--zero-burst-sec", type=float, default=0.80)
    parser.add_argument("--stop-settle-timeout-sec", type=float, default=2.5)
    parser.add_argument("--stop-stable-samples", type=int, default=6)
    parser.add_argument("--stop-linear-threshold-mps", type=float, default=0.008)
    parser.add_argument("--stop-angular-threshold-radps", type=float, default=0.020)
    parser.add_argument("--final-zero-epsilon", type=float, default=0.001)
    parser.add_argument("--tolerance-m", type=float, default=0.008)
    parser.add_argument("--overshoot-limit-m", type=float, default=0.015)
    parser.add_argument("--kp", type=float, default=0.35)
    parser.add_argument("--min-speed-mps", type=float, default=0.012)
    parser.add_argument("--max-speed-mps", type=float, default=0.040)
    parser.add_argument("--max-step-distance-m", type=float, default=0.30)
    parser.add_argument("--max-lateral-error-m", type=float, default=0.10)
    parser.add_argument("--max-yaw-error-rad", type=float, default=0.12)
    parser.add_argument("--min-confidence", type=float, default=0.50)
    parser.add_argument("--stable-samples", type=int, default=8)
    parser.add_argument("--no-progress-timeout-sec", type=float, default=5.0)
    parser.add_argument("--min-progress-m", type=float, default=0.010)
    parser.add_argument("--charging-current-min-a", type=float, default=0.10)
    args = parser.parse_args(argv)
    if args.target_fixed_face_gap_m <= 0.0 or args.max_speed_mps <= 0.0:
        parser.error("target gap and speed limits must be positive")
    if args.cmd_topic != "/cmd_vel_docking":
        parser.error("calibration motion must use /cmd_vel_docking through robot_safety")
    if args.min_speed_mps > args.max_speed_mps:
        parser.error("min speed must not exceed max speed")
    return args


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = parse_args(argv)
    output_dir = Path(args.output_root) / f"{_timestamp_tag()}_{_safe_label(args.label)}"
    output_dir.mkdir(parents=True, exist_ok=False)
    result = run_ros_servo(args)
    result["arguments"] = vars(args)
    summary_path = output_dir / "summary.json"
    summary_path.write_text(json.dumps(result, indent=2, sort_keys=True, allow_nan=True) + "\n", encoding="utf-8")
    print(f"[orbbec-dock-range-step] summary: {summary_path}")
    print(f"[orbbec-dock-range-step] success: {str(result['success']).lower()} reason={result['reason']}")
    return 0 if result["success"] else 2


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print("[orbbec-dock-range-step] interrupted", file=sys.stderr)
        raise SystemExit(130)
