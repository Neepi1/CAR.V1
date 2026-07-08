#!/usr/bin/env python3
"""Replay saved delivery ping-pong reports through the EKF A/B gate.

This is a read-only/offline helper. It never sends navigation goals. It never triggers relocalization. It never publishes velocity.
It exists so field runs between delivery_512355 and delivery_675235 can be re-scored with the same gate used by run_navigation_delivery_pingpong_guarded.sh before any EKF profile is promoted.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from datetime import datetime, timezone
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import validate_ekf_ab_report as validator  # noqa: E402


REJECT_EXIT = 10
INPUT_ERROR_EXIT = 2


def finite_float(value: object) -> float | None:
    try:
        parsed = float(str(value))
    except (TypeError, ValueError):
        return None
    return parsed if math.isfinite(parsed) else None


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--pose-report",
        action="append",
        required=True,
        help="Saved navigation_pose_error_test report directory or summary.md. Repeat per leg.",
    )
    parser.add_argument(
        "--trace-report",
        action="append",
        default=[],
        help=(
            "Optional saved navigation_terminal_yaw_trace report. Repeat with the "
            "same count as --pose-report; use NONE for a leg without a trace."
        ),
    )
    parser.add_argument(
        "--output-dir",
        default="reports/navigation_pingpong_ekf_gate_replay/latest",
        help="Directory for summary.md and replay_results.json.",
    )
    parser.add_argument("--max-online-xy-m", type=float, default=0.20)
    parser.add_argument("--max-online-yaw-rad", type=float, default=0.08)
    parser.add_argument("--max-map-base-translation-m", type=float, default=0.50)
    parser.add_argument("--max-map-odom-translation-m", type=float, default=0.30)
    parser.add_argument("--max-correction-yaw-deg", type=float, default=2.0)
    parser.add_argument(
        "--allow-missing-post-trigger-accepted",
        dest="require_post_trigger_accepted",
        action="store_false",
        help="Match validate_ekf_ab_report.py; normally not used for field acceptance.",
    )
    parser.set_defaults(require_post_trigger_accepted=True)
    return parser.parse_args(argv)


def validate_args(args: argparse.Namespace) -> None:
    thresholds = (
        args.max_online_xy_m,
        args.max_online_yaw_rad,
        args.max_map_base_translation_m,
        args.max_map_odom_translation_m,
        args.max_correction_yaw_deg,
    )
    if not all(math.isfinite(float(v)) and float(v) > 0.0 for v in thresholds):
        raise ValueError("all thresholds must be finite positive values")
    if args.trace_report and len(args.trace_report) != len(args.pose_report):
        raise ValueError("--trace-report count must match --pose-report count; use NONE for empty legs")


def trace_for_leg(args: argparse.Namespace, index: int) -> str | None:
    if not args.trace_report:
        return None
    value = args.trace_report[index]
    if value.strip().upper() in {"", "NONE", "NULL", "-"}:
        return None
    return value


def validate_leg(args: argparse.Namespace, index: int, pose_report_path: str) -> dict[str, object]:
    trace_report_path = trace_for_leg(args, index)
    pose_report = validator.load_pose_report(pose_report_path)
    trace_report = validator.load_trace_report(trace_report_path) if trace_report_path else None
    result = validator.build_result(pose_report, trace_report, args)
    result["leg"] = index + 1
    return result


def markdown_cell(value: object) -> str:
    if value is None:
        return ""
    text = str(value)
    return text.replace("|", "/").replace("\n", " ")[:500]


def result_row(result: dict[str, object]) -> str:
    metrics = result.get("metrics") if isinstance(result.get("metrics"), dict) else {}
    reasons = result.get("reasons") if isinstance(result.get("reasons"), list) else []
    return (
        f"| {result.get('leg')} | "
        f"{'ACCEPT' if result.get('accepted') else 'REJECT'} | "
        f"{markdown_cell(result.get('pose_report'))} | "
        f"{markdown_cell(result.get('trace_report'))} | "
        f"{markdown_cell(metrics.get('final_distance_m'))} | "
        f"{markdown_cell(metrics.get('final_yaw_error_rad'))} | "
        f"{markdown_cell(metrics.get('map_base_link_translation_m'))} | "
        f"{markdown_cell(metrics.get('map_base_link_dyaw_deg'))} | "
        f"{markdown_cell(metrics.get('map_odom_translation_m'))} | "
        f"{markdown_cell(metrics.get('map_odom_dyaw_deg'))} | "
        f"{markdown_cell(';'.join(str(reason) for reason in reasons))} |"
    )


def write_outputs(output_dir: Path, args: argparse.Namespace, results: list[dict[str, object]]) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    payload = {
        "timestamp_utc": datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"),
        "accepted": all(bool(result.get("accepted")) for result in results),
        "thresholds": {
            "max_online_xy_m": args.max_online_xy_m,
            "max_online_yaw_rad": args.max_online_yaw_rad,
            "max_map_base_translation_m": args.max_map_base_translation_m,
            "max_map_odom_translation_m": args.max_map_odom_translation_m,
            "max_correction_yaw_deg": args.max_correction_yaw_deg,
            "require_post_trigger_accepted": args.require_post_trigger_accepted,
        },
        "results": results,
    }
    (output_dir / "replay_results.json").write_text(
        json.dumps(payload, indent=2, sort_keys=True),
        encoding="utf-8",
    )

    lines = [
        "# Navigation Ping-Pong EKF Gate Replay",
        "",
        "- read_only: `true`",
        "- sends_navigation_goals: `false`",
        "- triggers_relocalization: `false`",
        f"- accepted: `{payload['accepted']}`",
        f"- max_online_xy_m: `{args.max_online_xy_m}`",
        f"- max_online_yaw_rad: `{args.max_online_yaw_rad}`",
        f"- max_map_base_translation_m: `{args.max_map_base_translation_m}`",
        f"- max_map_odom_translation_m: `{args.max_map_odom_translation_m}`",
        f"- max_correction_yaw_deg: `{args.max_correction_yaw_deg}`",
        "",
        "| leg | decision | pose_report | trace_report | online_xy_m | online_yaw_rad | map_base_m | map_base_yaw_deg | map_odom_m | map_odom_yaw_deg | reasons |",
        "|---:|---|---|---|---:|---:|---:|---:|---:|---:|---|",
    ]
    lines.extend(result_row(result) for result in results)
    lines.append("")
    (output_dir / "summary.md").write_text("\n".join(lines), encoding="utf-8")


def main(argv: list[str]) -> int:
    args = parse_args(argv)
    try:
        validate_args(args)
        results = [validate_leg(args, index, path) for index, path in enumerate(args.pose_report)]
    except (FileNotFoundError, ValueError) as exc:
        print(f"[ekf-gate-replay] input error: {exc}", file=sys.stderr)
        return INPUT_ERROR_EXIT

    output_dir = Path(args.output_dir)
    write_outputs(output_dir, args, results)
    print(f"[ekf-gate-replay] summary: {output_dir / 'summary.md'}")
    print(f"[ekf-gate-replay] results: {output_dir / 'replay_results.json'}")
    return 0 if all(bool(result.get("accepted")) for result in results) else REJECT_EXIT


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
