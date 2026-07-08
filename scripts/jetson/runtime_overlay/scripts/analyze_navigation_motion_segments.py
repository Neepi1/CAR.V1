#!/usr/bin/env python3
"""Segment a navigation observer report by chassis motion shape.

Input is a report produced by record_navigation_amcl_odom_correlation.sh.
The script is read-only: it only reads CSV/JSON files and writes analysis files
inside the same report directory.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
from statistics import mean
from typing import Any


def as_float(value: Any, default: float | None = None) -> float | None:
    if value is None:
        return default
    if isinstance(value, str) and not value.strip():
        return default
    try:
        result = float(value)
    except (TypeError, ValueError):
        return default
    if not math.isfinite(result):
        return default
    return result


def absmax(*values: float | None) -> float:
    finite = [abs(v) for v in values if v is not None and math.isfinite(v)]
    return max(finite) if finite else 0.0


def signedmax(*values: float | None) -> float:
    finite = [v for v in values if v is not None and math.isfinite(v)]
    if not finite:
        return 0.0
    return max(finite, key=lambda v: abs(v))


def row_time(row: dict[str, str]) -> float:
    return as_float(row.get("rel_time_sec"), 0.0) or 0.0


def classify_row(row: dict[str, str]) -> str:
    cmd_x = signedmax(
        as_float(row.get("cmd_vel_nav_raw_x")),
        as_float(row.get("cmd_vel_x")),
    )
    cmd_y = signedmax(
        as_float(row.get("cmd_vel_nav_raw_y")),
        as_float(row.get("cmd_vel_y")),
    )
    cmd_w = signedmax(
        as_float(row.get("cmd_vel_nav_raw_z")),
        as_float(row.get("cmd_vel_z")),
    )
    wheel_v = as_float(row.get("wheel_twist_linear_x"))
    wheel_w = as_float(row.get("wheel_twist_angular_z"))
    local_v = as_float(row.get("local_twist_linear_x"))
    local_w = as_float(row.get("local_twist_angular_z"))
    motion_v = as_float(row.get("motion_linear_velocity"))
    motion_y = as_float(row.get("motion_lateral_velocity"))
    motion_w = as_float(row.get("motion_angular_velocity"))
    final_distance = as_float(row.get("final_distance_m"))

    v_abs = absmax(cmd_x, wheel_v, local_v, motion_v)
    y_abs = absmax(cmd_y, motion_y)
    w_abs = absmax(cmd_w, wheel_w, local_w, motion_w)
    moving = v_abs >= 0.025 or y_abs >= 0.025 or w_abs >= 0.04
    near_goal = final_distance is not None and 0.0 <= final_distance <= 0.8

    if not moving:
        return "settle_stop"
    if near_goal:
        if w_abs >= 0.08 and v_abs >= 0.04:
            return "terminal_slow_arc"
        if w_abs >= 0.08:
            return "terminal_slow_spin"
        return "terminal_slow"
    if y_abs >= 0.03 and v_abs < 0.08:
        return "lateral"
    if w_abs >= 0.10 and v_abs < 0.08:
        return "spin"
    if w_abs >= 0.08 and v_abs >= 0.06:
        return "arc_turn"
    if v_abs >= 0.06 and w_abs < 0.08:
        return "straight"
    return "mixed"


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open(encoding="utf-8", newline="") as handle:
        return list(csv.DictReader(handle))


def write_csv(path: Path, rows: list[dict[str, Any]], fieldnames: list[str]) -> None:
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow({key: row.get(key, "") for key in fieldnames})


def diff_value(rows: list[dict[str, str]], key: str) -> float | None:
    start = as_float(rows[0].get(key))
    end = as_float(rows[-1].get(key))
    if start is None or end is None:
        return None
    return end - start


def norm_angle(value: float) -> float:
    while value > math.pi:
        value -= 2.0 * math.pi
    while value < -math.pi:
        value += 2.0 * math.pi
    return value


def diff_angle_value(rows: list[dict[str, str]], key: str) -> float | None:
    start = as_float(rows[0].get(key))
    end = as_float(rows[-1].get(key))
    if start is None or end is None:
        return None
    return norm_angle(end - start)


def max_value(rows: list[dict[str, str]], key: str) -> float | None:
    values = [as_float(row.get(key)) for row in rows]
    finite = [value for value in values if value is not None]
    return max(finite) if finite else None


def mean_abs(rows: list[dict[str, str]], key: str) -> float | None:
    values = [as_float(row.get(key)) for row in rows]
    finite = [abs(value) for value in values if value is not None]
    return mean(finite) if finite else None


def segment_rows(samples: list[dict[str, str]]) -> list[dict[str, Any]]:
    if not samples:
        return []
    segments: list[dict[str, Any]] = []
    current_phase = classify_row(samples[0])
    current_rows = [samples[0]]

    def flush(rows: list[dict[str, str]], phase: str) -> None:
        start_t = row_time(rows[0])
        end_t = row_time(rows[-1])
        wheel_dist_delta = diff_value(rows, "wheel_dist")
        local_dist_delta = diff_value(rows, "local_dist")
        candidate_start = as_float(rows[0].get("last_candidate_translation_m"))
        candidate_end = as_float(rows[-1].get("last_candidate_translation_m"))
        accepted_start = as_float(rows[0].get("last_accepted_translation_m"))
        accepted_end = as_float(rows[-1].get("last_accepted_translation_m"))
        final_start = as_float(rows[0].get("final_distance_m"))
        final_end = as_float(rows[-1].get("final_distance_m"))
        segments.append(
            {
                "segment_index": len(segments) + 1,
                "phase": phase,
                "start_s": round(start_t, 3),
                "end_s": round(end_t, 3),
                "duration_s": round(max(0.0, end_t - start_t), 3),
                "samples": len(rows),
                "wheel_dist_delta_m": wheel_dist_delta,
                "local_dist_delta_m": local_dist_delta,
                "wheel_yaw_delta_rad": diff_angle_value(rows, "wheel_dyaw"),
                "local_yaw_delta_rad": diff_angle_value(rows, "local_dyaw"),
                "mean_abs_cmd_x_mps": mean_abs(rows, "cmd_vel_nav_raw_x"),
                "mean_abs_cmd_w_radps": mean_abs(rows, "cmd_vel_nav_raw_z"),
                "mean_abs_wheel_v_mps": mean_abs(rows, "wheel_twist_linear_x"),
                "mean_abs_wheel_w_radps": mean_abs(rows, "wheel_twist_angular_z"),
                "max_candidate_translation_m": max_value(rows, "last_candidate_translation_m"),
                "candidate_translation_delta_m": (
                    None if candidate_start is None or candidate_end is None else candidate_end - candidate_start
                ),
                "accepted_translation_delta_m": (
                    None if accepted_start is None or accepted_end is None else accepted_end - accepted_start
                ),
                "final_distance_start_m": final_start,
                "final_distance_end_m": final_end,
            }
        )

    for row in samples[1:]:
        phase = classify_row(row)
        if phase == current_phase:
            current_rows.append(row)
            continue
        flush(current_rows, current_phase)
        current_phase = phase
        current_rows = [row]
    flush(current_rows, current_phase)
    return segments


def aggregate_by_phase(segments: list[dict[str, Any]]) -> list[dict[str, Any]]:
    phases = sorted({str(segment["phase"]) for segment in segments})
    rows: list[dict[str, Any]] = []
    for phase in phases:
        group = [segment for segment in segments if segment["phase"] == phase]
        rows.append(
            {
                "phase": phase,
                "segment_count": len(group),
                "duration_s": sum(as_float(segment.get("duration_s"), 0.0) or 0.0 for segment in group),
                "wheel_dist_delta_m": sum(as_float(segment.get("wheel_dist_delta_m"), 0.0) or 0.0 for segment in group),
                "local_dist_delta_m": sum(as_float(segment.get("local_dist_delta_m"), 0.0) or 0.0 for segment in group),
                "max_candidate_translation_m": max(
                    [
                        as_float(segment.get("max_candidate_translation_m"), 0.0) or 0.0
                        for segment in group
                    ],
                    default=0.0,
                ),
                "candidate_translation_delta_m": sum(
                    as_float(segment.get("candidate_translation_delta_m"), 0.0) or 0.0
                    for segment in group
                ),
            }
        )
    return rows


def load_final_relocalize(report_dir: Path) -> dict[str, Any]:
    candidates = [
        report_dir / "post_relocalize_compare" / "correction_metrics.json",
        report_dir / "relocalize_compare" / "correction_metrics.json",
    ]
    candidates.extend(sorted(report_dir.glob("relocalize_compare_*/correction_metrics.json")))
    for path in candidates:
        if path.exists():
            try:
                data = json.loads(path.read_text(encoding="utf-8"))
            except json.JSONDecodeError:
                continue
            data["_path"] = str(path)
            return data
    return {}


def write_markdown(
    report_dir: Path,
    segments: list[dict[str, Any]],
    phase_rows: list[dict[str, Any]],
    final_relocalize: dict[str, Any],
) -> None:
    lines = [
        "# Navigation Motion Segment Analysis",
        "",
        f"- report_dir: `{report_dir}`",
        f"- segments: `{len(segments)}`",
    ]
    if final_relocalize:
        map_base = final_relocalize.get("map_base_link_delta", {})
        bridge = final_relocalize.get("bridge", {})
        lines.extend(
            [
                f"- final_relocalize_metrics: `{final_relocalize.get('_path')}`",
                f"- final_map_base_translation_m: `{map_base.get('translation_m')}`",
                f"- final_map_base_yaw_rad: `{map_base.get('dyaw_rad')}`",
                f"- bridge_last_accepted_translation_m: `{bridge.get('last_accepted_correction_translation_m')}`",
                f"- bridge_last_accepted_yaw_rad: `{bridge.get('last_accepted_correction_yaw_rad')}`",
            ]
        )
    else:
        lines.append("- final_relocalize_metrics: `missing`")

    lines.extend(
        [
            "",
            "## Phase Totals",
            "",
            "| phase | segments | duration_s | wheel_dist_m | local_dist_m | max_shadow_or_trigger_candidate_m | candidate_delta_sum_m |",
            "|---|---:|---:|---:|---:|---:|---:|",
        ]
    )
    for row in phase_rows:
        lines.append(
            "| "
            + " | ".join(
                str(row.get(key, ""))
                for key in (
                    "phase",
                    "segment_count",
                    "duration_s",
                    "wheel_dist_delta_m",
                    "local_dist_delta_m",
                    "max_candidate_translation_m",
                    "candidate_translation_delta_m",
                )
            )
            + " |"
        )

    lines.extend(
        [
            "",
            "## Segments",
            "",
            "| # | phase | start_s | end_s | duration_s | wheel_dist_m | wheel_yaw_rad | max_candidate_m | candidate_delta_m | final_dist_start | final_dist_end |",
            "|---:|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|",
        ]
    )
    for row in segments:
        lines.append(
            "| "
            + " | ".join(
                str(row.get(key, ""))
                for key in (
                    "segment_index",
                    "phase",
                    "start_s",
                    "end_s",
                    "duration_s",
                    "wheel_dist_delta_m",
                    "wheel_yaw_delta_rad",
                    "max_candidate_translation_m",
                    "candidate_translation_delta_m",
                    "final_distance_start_m",
                    "final_distance_end_m",
                )
            )
            + " |"
        )
    lines.extend(
        [
            "",
            "Files:",
            "",
            "- `motion_segments.csv`",
            "- `motion_phase_totals.csv`",
            "- `motion_segment_summary.json`",
        ]
    )
    (report_dir / "motion_segment_summary.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("report_dir", help="Report directory from record_navigation_amcl_odom_correlation.sh")
    args = parser.parse_args()

    report_dir = Path(args.report_dir).expanduser().resolve()
    samples_path = report_dir / "samples.csv"
    if not samples_path.exists():
        raise SystemExit(f"samples.csv not found: {samples_path}")

    samples = read_csv(samples_path)
    segments = segment_rows(samples)
    phase_rows = aggregate_by_phase(segments)
    final_relocalize = load_final_relocalize(report_dir)
    summary = {
        "report_dir": str(report_dir),
        "segments": segments,
        "phase_totals": phase_rows,
        "final_relocalize": final_relocalize,
    }
    segment_fields = [
        "segment_index",
        "phase",
        "start_s",
        "end_s",
        "duration_s",
        "samples",
        "wheel_dist_delta_m",
        "local_dist_delta_m",
        "wheel_yaw_delta_rad",
        "local_yaw_delta_rad",
        "mean_abs_cmd_x_mps",
        "mean_abs_cmd_w_radps",
        "mean_abs_wheel_v_mps",
        "mean_abs_wheel_w_radps",
        "max_candidate_translation_m",
        "candidate_translation_delta_m",
        "accepted_translation_delta_m",
        "final_distance_start_m",
        "final_distance_end_m",
    ]
    phase_fields = [
        "phase",
        "segment_count",
        "duration_s",
        "wheel_dist_delta_m",
        "local_dist_delta_m",
        "max_candidate_translation_m",
        "candidate_translation_delta_m",
    ]
    write_csv(report_dir / "motion_segments.csv", segments, segment_fields)
    write_csv(report_dir / "motion_phase_totals.csv", phase_rows, phase_fields)
    (report_dir / "motion_segment_summary.json").write_text(
        json.dumps(summary, indent=2, sort_keys=True, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    write_markdown(report_dir, segments, phase_rows, final_relocalize)
    print(report_dir / "motion_segment_summary.md")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
