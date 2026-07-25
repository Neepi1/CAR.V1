#!/usr/bin/env python3
"""Inspect global-costmap costs along a straight map-frame corridor."""

from __future__ import annotations

import argparse
import json
import math
import time
from pathlib import Path

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


class CostmapProbe(Node):
    def __init__(self, topic):
        super().__init__("inspect_global_costmap_corridor")
        self.message = None
        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(OccupancyGrid, topic, self.on_costmap, qos)

    def on_costmap(self, message):
        self.message = message


def percentile(values, quantile):
    ordered = sorted(values)
    if not ordered:
        return None
    index = min(len(ordered) - 1, max(0, round((len(ordered) - 1) * quantile)))
    return ordered[index]


def grid_value(message, x, y):
    info = message.info
    gx = int(math.floor((x - info.origin.position.x) / info.resolution))
    gy = int(math.floor((y - info.origin.position.y) / info.resolution))
    if gx < 0 or gy < 0 or gx >= info.width or gy >= info.height:
        return None
    return int(message.data[gy * info.width + gx])


def line_metrics(message, start, goal, offset_m):
    dx = goal[0] - start[0]
    dy = goal[1] - start[1]
    length = math.hypot(dx, dy)
    ux = dx / length
    uy = dy / length
    px = -uy
    py = ux
    sample_step = max(0.01, float(message.info.resolution) * 0.5)
    sample_count = max(2, int(math.ceil(length / sample_step)) + 1)
    values = []
    for index in range(sample_count):
        fraction = index / (sample_count - 1)
        x = start[0] + dx * fraction + px * offset_m
        y = start[1] + dy * fraction + py * offset_m
        values.append(grid_value(message, x, y))

    outside = sum(value is None for value in values)
    unknown = sum(value == -1 for value in values)
    known = [value for value in values if value is not None and value >= 0]
    lethal = sum(value >= 99 for value in known)
    high = sum(value >= 80 for value in known)
    medium = sum(value >= 50 for value in known)
    weighted = [100 if value is None or value < 0 else value for value in values]
    return {
        "offset_m": round(offset_m, 4),
        "samples": len(values),
        "outside": outside,
        "unknown": unknown,
        "lethal": lethal,
        "high": high,
        "medium": medium,
        "max_cost": max(known) if known else None,
        "mean_known_cost": sum(known) / len(known) if known else None,
        "p95_known_cost": percentile(known, 0.95),
        "mean_penalty": sum(weighted) / len(weighted),
    }


def render_markdown(result):
    lines = [
        "# Global Costmap Corridor",
        "",
        f"- topic: `{result['topic']}`",
        f"- frame: `{result['frame_id']}`",
        f"- resolution_m: `{result['resolution_m']}`",
        f"- start: `{result['start']}`",
        f"- goal: `{result['goal']}`",
        f"- best_parallel_offset_m: `{result['best_parallel_offset_m']}`",
        "",
        "| offset_m | unknown | lethal | high | medium | max | mean | p95 | penalty |",
        "|---:|---:|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for row in result["corridors"]:
        lines.append(
            "| {offset_m} | {unknown} | {lethal} | {high} | {medium} | "
            "{max_cost} | {mean_known_cost} | {p95_known_cost} | {mean_penalty} |".format(
                **row
            )
        )
    lines.append("")
    return "\n".join(lines)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--start-x", required=True, type=float)
    parser.add_argument("--start-y", required=True, type=float)
    parser.add_argument("--goal-x", required=True, type=float)
    parser.add_argument("--goal-y", required=True, type=float)
    parser.add_argument("--half-width-m", type=float, default=1.0)
    parser.add_argument("--offset-step-m", type=float, default=0.05)
    parser.add_argument("--topic", default="/global_costmap/costmap")
    parser.add_argument("--timeout-sec", type=float, default=12.0)
    parser.add_argument("--output-dir", required=True, type=Path)
    args = parser.parse_args()

    rclpy.init()
    node = CostmapProbe(args.topic)
    deadline = time.time() + args.timeout_sec
    try:
        while rclpy.ok() and node.message is None and time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if node.message is None:
            raise RuntimeError(f"timed out waiting for {args.topic}")

        offset_count = int(round(args.half_width_m / args.offset_step_m))
        offsets = [index * args.offset_step_m for index in range(-offset_count, offset_count + 1)]
        corridors = [
            line_metrics(
                node.message,
                (args.start_x, args.start_y),
                (args.goal_x, args.goal_y),
                offset,
            )
            for offset in offsets
        ]
        best = min(corridors, key=lambda row: (row["mean_penalty"], abs(row["offset_m"])))
        result = {
            "schema": "njrh.global_costmap_corridor.v1",
            "topic": args.topic,
            "frame_id": node.message.header.frame_id,
            "stamp": {
                "sec": node.message.header.stamp.sec,
                "nanosec": node.message.header.stamp.nanosec,
            },
            "resolution_m": node.message.info.resolution,
            "start": {"x": args.start_x, "y": args.start_y},
            "goal": {"x": args.goal_x, "y": args.goal_y},
            "best_parallel_offset_m": best["offset_m"],
            "center": next(row for row in corridors if abs(row["offset_m"]) < 1.0e-9),
            "corridors": corridors,
        }
        args.output_dir.mkdir(parents=True, exist_ok=True)
        (args.output_dir / "corridor.json").write_text(
            json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )
        (args.output_dir / "corridor.md").write_text(render_markdown(result), encoding="utf-8")
        print(json.dumps({
            "center": result["center"],
            "best_parallel_offset_m": result["best_parallel_offset_m"],
            "report": str(args.output_dir / "corridor.md"),
        }, sort_keys=True))
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
