#!/usr/bin/env python3
"""Score straight-line navigation from an odom-goal capture.

The analyzer intentionally works offline. It reads the sampled controller and
odometry data produced by record_navigation_odom_goal_closure.sh and never joins
the ROS graph.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import statistics
from pathlib import Path
from typing import Iterable


DEFAULT_LIMITS = {
    "cross_track_peak_to_peak_m": 0.12,
    "cross_track_rms_m": 0.04,
    "trajectory_length_ratio": 1.015,
    "cmd_wz_reversals": 3,
    "motion_wz_reversals": 3,
    "smoothed_plan_cross_track_m": 0.08,
    "smoothed_plan_length_ratio": 1.02,
}


def finite_float(value):
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result if math.isfinite(result) else None


def percentile(values: Iterable[float], quantile: float):
    ordered = sorted(float(value) for value in values)
    if not ordered:
        return None
    if len(ordered) == 1:
        return ordered[0]
    position = max(0.0, min(1.0, quantile)) * (len(ordered) - 1)
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    if lower == upper:
        return ordered[lower]
    fraction = position - lower
    return ordered[lower] * (1.0 - fraction) + ordered[upper] * fraction


def persistent_sign_reversals(values: Iterable[float], threshold: float, min_run: int = 2):
    runs = []
    active_sign = None
    active_count = 0
    for value in values:
        if abs(value) < threshold:
            continue
        sign = 1 if value > 0.0 else -1
        if sign == active_sign:
            active_count += 1
            continue
        if active_sign is not None and active_count >= min_run:
            runs.append(active_sign)
        active_sign = sign
        active_count = 1
    if active_sign is not None and active_count >= min_run:
        runs.append(active_sign)
    return max(0, len(runs) - 1), runs


def trajectory_metrics(rows):
    moving = []
    for row in rows:
        x = finite_float(row.get("local_x"))
        y = finite_float(row.get("local_y"))
        cmd_vx = finite_float(row.get("cmd_vel_nav_raw_x"))
        actual_vx = finite_float(row.get("motion_linear_velocity"))
        if None in (x, y, cmd_vx):
            continue
        if cmd_vx < 0.12:
            continue
        if actual_vx is not None and actual_vx < 0.08:
            continue
        moving.append(row)

    if len(moving) < 8:
        return {
            "sufficient": False,
            "reason": "fewer_than_8_forward_motion_samples",
            "sample_count": len(moving),
        }

    points = [
        (finite_float(row["local_x"]), finite_float(row["local_y"]))
        for row in moving
    ]
    start_x, start_y = points[0]
    end_x, end_y = points[-1]
    chord_dx = end_x - start_x
    chord_dy = end_y - start_y
    chord = math.hypot(chord_dx, chord_dy)
    if chord < 1.0:
        return {
            "sufficient": False,
            "reason": "forward_motion_chord_shorter_than_1m",
            "sample_count": len(moving),
            "chord_m": chord,
        }

    ux = chord_dx / chord
    uy = chord_dy / chord
    all_cross_track = []
    central_cross_track = []
    central_rows = []
    path_length = 0.0
    previous = points[0]
    for row, (x, y) in zip(moving, points):
        path_length += math.hypot(x - previous[0], y - previous[1])
        previous = (x, y)
        rel_x = x - start_x
        rel_y = y - start_y
        progress = rel_x * ux + rel_y * uy
        cross_track = rel_x * uy - rel_y * ux
        all_cross_track.append(cross_track)
        if 0.10 * chord <= progress <= 0.90 * chord:
            central_cross_track.append(cross_track)
            central_rows.append(row)

    if len(central_rows) < 5:
        central_rows = moving
        central_cross_track = all_cross_track

    cmd_wz = [finite_float(row.get("cmd_vel_nav_raw_z")) or 0.0 for row in central_rows]
    motion_wz = [finite_float(row.get("motion_angular_velocity")) or 0.0 for row in central_rows]
    steering = [finite_float(row.get("motion_steering_angle")) or 0.0 for row in central_rows]
    cmd_reversals, cmd_runs = persistent_sign_reversals(cmd_wz, 0.02)
    motion_reversals, motion_runs = persistent_sign_reversals(motion_wz, 0.02)
    steering_reversals, steering_runs = persistent_sign_reversals(steering, 0.02)

    mean_cross_track = statistics.fmean(central_cross_track)
    rms_cross_track = math.sqrt(
        statistics.fmean(value * value for value in central_cross_track)
    )
    return {
        "sufficient": True,
        "sample_count": len(moving),
        "central_sample_count": len(central_rows),
        "chord_m": chord,
        "path_length_m": path_length,
        "trajectory_length_ratio": path_length / chord,
        "cross_track_mean_m": mean_cross_track,
        "cross_track_rms_m": rms_cross_track,
        "cross_track_max_abs_m": max(abs(value) for value in central_cross_track),
        "cross_track_peak_to_peak_m": max(central_cross_track) - min(central_cross_track),
        "cross_track_p95_abs_m": percentile(
            [abs(value) for value in central_cross_track], 0.95
        ),
        "cmd_wz_reversals": cmd_reversals,
        "cmd_wz_sign_runs": cmd_runs,
        "motion_wz_reversals": motion_reversals,
        "motion_wz_sign_runs": motion_runs,
        "steering_reversals": steering_reversals,
        "steering_sign_runs": steering_runs,
        "max_abs_cmd_wz_radps": max(abs(value) for value in cmd_wz),
        "max_abs_motion_wz_radps": max(abs(value) for value in motion_wz),
    }


def select_smoothed_plan(path_metrics):
    for topic in ("/plan_smoothed", "/plan", "/received_global_plan"):
        stats = path_metrics.get(topic)
        if isinstance(stats, dict) and int(stats.get("count") or 0) > 0:
            return topic, stats
    return None, None


def classify(trajectory, path_metrics, limits):
    failures = []
    if not trajectory.get("sufficient"):
        return "INSUFFICIENT", [trajectory.get("reason", "insufficient_trajectory")]

    for key in (
        "cross_track_peak_to_peak_m",
        "cross_track_rms_m",
        "trajectory_length_ratio",
        "cmd_wz_reversals",
        "motion_wz_reversals",
    ):
        if float(trajectory.get(key) or 0.0) > float(limits[key]):
            failures.append(f"{key}>{limits[key]}")

    plan_topic, plan = select_smoothed_plan(path_metrics)
    if plan is not None:
        if float(plan.get("max_cross_track_m") or 0.0) > limits["smoothed_plan_cross_track_m"]:
            failures.append(
                f"{plan_topic}.max_cross_track_m>{limits['smoothed_plan_cross_track_m']}"
            )
        if float(plan.get("max_length_ratio") or 0.0) > limits["smoothed_plan_length_ratio"]:
            failures.append(
                f"{plan_topic}.max_length_ratio>{limits['smoothed_plan_length_ratio']}"
            )

    return ("RED" if failures else "PASS"), failures


def load_csv(path):
    with path.open(newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def render_markdown(result):
    trajectory = result["trajectory"]
    lines = [
        "# Navigation Straightness",
        "",
        f"- verdict: `{result['verdict']}`",
        f"- failures: `{result['failures']}`",
        f"- selected_plan_topic: `{result['selected_plan_topic']}`",
        "",
        "## Cruise Trajectory",
        "",
        "| metric | value | limit |",
        "|---|---:|---:|",
    ]
    for key in (
        "chord_m",
        "path_length_m",
        "trajectory_length_ratio",
        "cross_track_peak_to_peak_m",
        "cross_track_rms_m",
        "cross_track_p95_abs_m",
        "cmd_wz_reversals",
        "motion_wz_reversals",
        "steering_reversals",
    ):
        lines.append(f"| {key} | {trajectory.get(key)} | {result['limits'].get(key, '')} |")
    lines.extend(["", "## Path Topics", "", "```json"])
    lines.append(json.dumps(result["path_metrics"], indent=2, sort_keys=True))
    lines.extend(["```", ""])
    return "\n".join(lines)


def analyze(report_dir: Path, limits=None):
    limits = dict(DEFAULT_LIMITS if limits is None else limits)
    rows = load_csv(report_dir / "samples.csv")
    path_metrics_path = report_dir / "path_metrics.json"
    path_metrics = (
        json.loads(path_metrics_path.read_text(encoding="utf-8"))
        if path_metrics_path.exists()
        else {}
    )
    trajectory = trajectory_metrics(rows)
    verdict, failures = classify(trajectory, path_metrics, limits)
    selected_plan_topic, _ = select_smoothed_plan(path_metrics)
    result = {
        "schema": "njrh.navigation_straightness.v1",
        "verdict": verdict,
        "failures": failures,
        "limits": limits,
        "trajectory": trajectory,
        "selected_plan_topic": selected_plan_topic,
        "path_metrics": path_metrics,
    }
    (report_dir / "straightness.json").write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    (report_dir / "straightness.md").write_text(render_markdown(result), encoding="utf-8")
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--report-dir", required=True, type=Path)
    parser.add_argument("--assert-pass", action="store_true")
    args = parser.parse_args()
    result = analyze(args.report_dir)
    print(f"[nav-straightness] verdict={result['verdict']}")
    print(f"[nav-straightness] report={args.report_dir / 'straightness.md'}")
    if args.assert_pass and result["verdict"] != "PASS":
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
