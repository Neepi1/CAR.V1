#!/usr/bin/env python3
"""Request and validate a live Nav2 path without commanding robot motion."""

import argparse
import json
import math
import sys
from pathlib import Path

import rclpy
from action_msgs.msg import GoalStatus
from nav2_msgs.action import ComputePathToPose
from rclpy.action import ActionClient
from rclpy.node import Node


def normalize_angle(value: float) -> float:
    return math.atan2(math.sin(value), math.cos(value))


def yaw_from_quaternion(orientation) -> float:
    siny_cosp = 2.0 * (
        orientation.w * orientation.z + orientation.x * orientation.y
    )
    cosy_cosp = 1.0 - 2.0 * (
        orientation.y * orientation.y + orientation.z * orientation.z
    )
    return math.atan2(siny_cosp, cosy_cosp)


def quaternion_from_yaw(yaw: float):
    from geometry_msgs.msg import Quaternion

    orientation = Quaternion()
    orientation.z = math.sin(yaw * 0.5)
    orientation.w = math.cos(yaw * 0.5)
    return orientation


def parse_args():
    parser = argparse.ArgumentParser(
        description="Validate the path returned by the live SmacPlannerLattice instance."
    )
    parser.add_argument("--goal-x", required=True, type=float)
    parser.add_argument("--goal-y", required=True, type=float)
    parser.add_argument("--goal-yaw", required=True, type=float)
    parser.add_argument("--start-x", type=float)
    parser.add_argument("--start-y", type=float)
    parser.add_argument("--start-yaw", type=float)
    parser.add_argument("--planner-id", default="GridBased")
    parser.add_argument("--frame-id", default="map")
    parser.add_argument("--server-timeout-sec", type=float, default=5.0)
    parser.add_argument("--result-timeout-sec", type=float, default=5.0)
    parser.add_argument("--spin-translation-epsilon-m", type=float, default=0.002)
    parser.add_argument("--endpoint-spin-window-m", type=float, default=0.05)
    parser.add_argument("--curvature-min-segment-m", type=float, default=0.01)
    parser.add_argument("--max-curvature-inv-m", type=float, default=1.25)
    parser.add_argument("--reverse-admission-distance-m", type=float, default=0.30)
    parser.add_argument("--checkpoint-distance-m", type=float, default=1.0)
    parser.add_argument("--include-poses", action="store_true")
    parser.add_argument("--output-json", type=Path)
    return parser.parse_args()


class LivePathProbe(Node):
    def __init__(self):
        super().__init__("ranger_lattice_live_path_probe")
        self.client = ActionClient(self, ComputePathToPose, "/compute_path_to_pose")


def analyze_path(path, args, planning_time_sec: float):
    poses = path.poses
    if len(poses) < 2:
        return {
            "passed": False,
            "violations": ["path_has_fewer_than_two_poses"],
            "pose_count": len(poses),
        }

    segments = []
    pose_cumulative = [0.0]
    total_length = 0.0
    for index, (current, following) in enumerate(zip(poses, poses[1:])):
        dx = following.pose.position.x - current.pose.position.x
        dy = following.pose.position.y - current.pose.position.y
        distance = math.hypot(dx, dy)
        yaw = yaw_from_quaternion(current.pose.orientation)
        next_yaw = yaw_from_quaternion(following.pose.orientation)
        delta_yaw = normalize_angle(next_yaw - yaw)
        forward_projection = dx * math.cos(yaw) + dy * math.sin(yaw)
        segments.append(
            {
                "index": index,
                "distance_m": distance,
                "delta_yaw_rad": delta_yaw,
                "cumulative_m": total_length,
                "forward_projection_m": forward_projection,
                "direction": (
                    "forward"
                    if forward_projection > 1.0e-4
                    else "reverse"
                    if forward_projection < -1.0e-4
                    else "stationary"
                ),
            }
        )
        total_length += distance
        pose_cumulative.append(total_length)

    spin_segments = []
    reverse_segments = []
    reverse_segments_outside_admission = []
    curvature_segments = []
    for segment in segments:
        distance = segment["distance_m"]
        delta_yaw = segment["delta_yaw_rad"]
        cumulative = segment["cumulative_m"]
        if distance <= args.spin_translation_epsilon_m and abs(delta_yaw) > 1.0e-4:
            location = "midpath"
            if cumulative <= args.endpoint_spin_window_m:
                location = "start"
            elif total_length - cumulative <= args.endpoint_spin_window_m:
                location = "goal"
            spin_segments.append(
                {
                    "index": segment["index"],
                    "location": location,
                    "cumulative_m": round(cumulative, 6),
                    "delta_yaw_rad": round(delta_yaw, 6),
                }
            )
        if distance > args.spin_translation_epsilon_m and segment["forward_projection_m"] < -1.0e-4:
            index = segment["index"]
            current = poses[index].pose.position
            following = poses[index + 1].pose.position
            goal = poses[-1].pose.position
            goal_distance_m = max(
                math.hypot(current.x - goal.x, current.y - goal.y),
                math.hypot(following.x - goal.x, following.y - goal.y),
            )
            reverse_segments.append(
                {
                    "index": index,
                    "goal_distance_m": round(goal_distance_m, 6),
                }
            )
            if goal_distance_m > args.reverse_admission_distance_m + 1.0e-6:
                reverse_segments_outside_admission.append(index)
        if distance >= args.curvature_min_segment_m:
            curvature_segments.append(
                (abs(delta_yaw) / distance, segment["index"])
            )

    max_curvature, max_curvature_index = max(
        curvature_segments, default=(0.0, -1), key=lambda item: item[0]
    )
    route_dx = poses[-1].pose.position.x - poses[0].pose.position.x
    route_dy = poses[-1].pose.position.y - poses[0].pose.position.y
    route_length = math.hypot(route_dx, route_dy)
    max_cross_track_to_start_goal_m = 0.0
    if route_length > 1.0e-9:
        for pose in poses:
            point_dx = pose.pose.position.x - poses[0].pose.position.x
            point_dy = pose.pose.position.y - poses[0].pose.position.y
            cross_track = abs(
                route_dx * point_dy - route_dy * point_dx
            ) / route_length
            max_cross_track_to_start_goal_m = max(
                max_cross_track_to_start_goal_m, cross_track
            )
    moving_segments = [
        segment for segment in segments if segment["direction"] != "stationary"
    ]
    direction_change_segment_indices = []
    for previous, current in zip(moving_segments, moving_segments[1:]):
        if previous["direction"] != current["direction"]:
            direction_change_segment_indices.extend(
                [previous["index"], current["index"]]
            )
    direction_change_segment_indices = sorted(set(direction_change_segment_indices))
    max_curvature_segment = None
    if max_curvature_index >= 0:
        segment = segments[max_curvature_index]
        max_curvature_segment = {
            "index": max_curvature_index,
            "distance_m": round(segment["distance_m"], 6),
            "delta_yaw_rad": round(segment["delta_yaw_rad"], 6),
            "forward_projection_m": round(segment["forward_projection_m"], 6),
            "direction": segment["direction"],
            "adjacent_to_direction_change": (
                max_curvature_index in direction_change_segment_indices
            ),
        }
    terminal_profile = []
    for remaining_target in (1.0, 0.75, 0.5, 0.35, 0.25, 0.15, 0.10, 0.05, 0.0):
        index = min(
            range(len(poses)),
            key=lambda candidate: abs(
                (total_length - pose_cumulative[candidate]) - remaining_target
            ),
        )
        pose = poses[index].pose
        terminal_profile.append(
            {
                "remaining_m": round(total_length - pose_cumulative[index], 6),
                "pose_index": index,
                "x": round(float(pose.position.x), 6),
                "y": round(float(pose.position.y), 6),
                "yaw": round(yaw_from_quaternion(pose.orientation), 6),
            }
        )
    final_moving_segment = next(
        (
            segment
            for segment in reversed(segments)
            if segment["distance_m"] > args.spin_translation_epsilon_m
        ),
        None,
    )
    final_translation_heading = None
    if final_moving_segment is not None:
        index = final_moving_segment["index"]
        current = poses[index].pose.position
        following = poses[index + 1].pose.position
        final_translation_heading = math.atan2(
            following.y - current.y,
            following.x - current.x,
        )
    checkpoint = None
    cumulative = 0.0
    for index, segment in enumerate(segments):
        cumulative += segment["distance_m"]
        if cumulative + 1.0e-9 >= args.checkpoint_distance_m:
            pose = poses[index + 1].pose
            checkpoint = {
                "requested_distance_m": args.checkpoint_distance_m,
                "path_distance_m": round(cumulative, 6),
                "x": round(float(pose.position.x), 6),
                "y": round(float(pose.position.y), 6),
                "yaw": round(yaw_from_quaternion(pose.orientation), 6),
            }
            break
    midpath_spins = [item for item in spin_segments if item["location"] == "midpath"]
    violations = []
    if midpath_spins:
        violations.append(f"midpath_spin_segments={len(midpath_spins)}")
    if reverse_segments_outside_admission:
        violations.append(
            "reverse_segments_outside_admission="
            f"{len(reverse_segments_outside_admission)}"
        )
    if max_curvature > args.max_curvature_inv_m + 1.0e-6:
        violations.append(
            f"max_curvature={max_curvature:.6f}_exceeds_{args.max_curvature_inv_m:.6f}"
        )

    result = {
        "passed": not violations,
        "violations": violations,
        "frame_id": path.header.frame_id,
        "pose_count": len(poses),
        "path_length_m": round(total_length, 6),
        "max_cross_track_to_start_goal_m": round(
            max_cross_track_to_start_goal_m, 6
        ),
        "path_start": {
            "x": round(float(poses[0].pose.position.x), 6),
            "y": round(float(poses[0].pose.position.y), 6),
            "yaw": round(yaw_from_quaternion(poses[0].pose.orientation), 6),
        },
        "path_goal": {
            "x": round(float(poses[-1].pose.position.x), 6),
            "y": round(float(poses[-1].pose.position.y), 6),
            "yaw": round(yaw_from_quaternion(poses[-1].pose.orientation), 6),
        },
        "final_translation_heading_rad": (
            None if final_translation_heading is None else round(final_translation_heading, 6)
        ),
        "final_translation_to_pose_yaw_error_rad": (
            None
            if final_translation_heading is None
            else round(
                normalize_angle(
                    yaw_from_quaternion(poses[-1].pose.orientation) - final_translation_heading
                ),
                6,
            )
        ),
        "terminal_profile": terminal_profile,
        "checkpoint": checkpoint,
        "planning_time_sec": round(planning_time_sec, 6),
        "spin_segment_count": len(spin_segments),
        "spin_segments": spin_segments,
        "midpath_spin_segment_count": len(midpath_spins),
        "reverse_segment_count": len(reverse_segments),
        "reverse_segments": reverse_segments,
        "reverse_segments_outside_admission": reverse_segments_outside_admission,
        "max_moving_curvature_inv_m": round(max_curvature, 6),
        "max_moving_curvature_segment_index": max_curvature_index,
        "max_moving_curvature_segment": max_curvature_segment,
        "direction_change_segment_indices": direction_change_segment_indices,
        "limits": {
            "max_curvature_inv_m": args.max_curvature_inv_m,
            "endpoint_spin_window_m": args.endpoint_spin_window_m,
            "reverse_admission_distance_m": args.reverse_admission_distance_m,
        },
    }
    if args.include_poses:
        result["path_poses"] = [
            {
                "index": index,
                "x": round(float(pose.pose.position.x), 6),
                "y": round(float(pose.pose.position.y), 6),
                "yaw": round(yaw_from_quaternion(pose.pose.orientation), 6),
                "cumulative_m": round(pose_cumulative[index], 6),
            }
            for index, pose in enumerate(poses)
        ]
        result["segments"] = [
            {
                "index": segment["index"],
                "distance_m": round(segment["distance_m"], 6),
                "delta_yaw_rad": round(segment["delta_yaw_rad"], 6),
                "forward_projection_m": round(segment["forward_projection_m"], 6),
                "direction": segment["direction"],
                "curvature_inv_m": (
                    None
                    if segment["distance_m"] < args.curvature_min_segment_m
                    else round(abs(segment["delta_yaw_rad"]) / segment["distance_m"], 6)
                ),
            }
            for segment in segments
        ]
    return result


def write_result(result, output_path):
    rendered = json.dumps(result, indent=2, sort_keys=True)
    print(rendered)
    if output_path:
        output_path.parent.mkdir(parents=True, exist_ok=True)
        output_path.write_text(rendered + "\n", encoding="utf-8")


def main() -> int:
    args = parse_args()
    rclpy.init()
    node = LivePathProbe()
    result = None
    exit_code = 2
    try:
        if not node.client.wait_for_server(timeout_sec=args.server_timeout_sec):
            result = {
                "passed": False,
                "violations": ["compute_path_to_pose_server_unavailable"],
            }
            return exit_code

        goal = ComputePathToPose.Goal()
        goal.goal.header.frame_id = args.frame_id
        goal.goal.header.stamp = node.get_clock().now().to_msg()
        goal.goal.pose.position.x = args.goal_x
        goal.goal.pose.position.y = args.goal_y
        goal.goal.pose.orientation = quaternion_from_yaw(args.goal_yaw)
        goal.planner_id = args.planner_id
        start_values = (args.start_x, args.start_y, args.start_yaw)
        if any(value is not None for value in start_values) and not all(
            value is not None for value in start_values
        ):
            result = {
                "passed": False,
                "violations": ["start_x_start_y_start_yaw_must_be_provided_together"],
            }
            return 2
        goal.use_start = all(value is not None for value in start_values)
        if goal.use_start:
            goal.start.header.frame_id = args.frame_id
            goal.start.header.stamp = goal.goal.header.stamp
            goal.start.pose.position.x = args.start_x
            goal.start.pose.position.y = args.start_y
            goal.start.pose.orientation = quaternion_from_yaw(args.start_yaw)

        send_future = node.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(node, send_future, timeout_sec=args.server_timeout_sec)
        if not send_future.done() or send_future.result() is None:
            result = {"passed": False, "violations": ["goal_response_timeout"]}
            return exit_code
        goal_handle = send_future.result()
        if not goal_handle.accepted:
            result = {"passed": False, "violations": ["goal_rejected"]}
            return exit_code

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(node, result_future, timeout_sec=args.result_timeout_sec)
        if not result_future.done() or result_future.result() is None:
            goal_handle.cancel_goal_async()
            result = {"passed": False, "violations": ["path_result_timeout"]}
            return exit_code

        wrapped_result = result_future.result()
        if wrapped_result.status != GoalStatus.STATUS_SUCCEEDED:
            result = {
                "passed": False,
                "violations": [f"planner_action_status={wrapped_result.status}"],
            }
            return exit_code

        planning_time = wrapped_result.result.planning_time
        planning_time_sec = float(planning_time.sec) + float(planning_time.nanosec) * 1.0e-9
        result = analyze_path(wrapped_result.result.path, args, planning_time_sec)
        exit_code = 0 if result["passed"] else 3
        return exit_code
    except Exception as exc:  # Keep the field probe machine-readable on failure.
        result = {
            "passed": False,
            "violations": [f"probe_exception={type(exc).__name__}: {exc}"],
        }
        return 2
    finally:
        if result is not None:
            write_result(result, args.output_json)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
