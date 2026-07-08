#!/usr/bin/env python3
"""Validate EKF A/B navigation reports before using them as field evidence.

The script is intentionally offline-only: it parses saved report files and never
talks to ROS, the API server, or the robot. A run is accepted only when the API
goal lifecycle, online final audit, post-goal relocalization, and optional
terminal trace all agree that the sample is clean enough for EKF comparison.
"""

from __future__ import annotations

import argparse
import json
import math
import re
import sys
from pathlib import Path
from typing import Any


SUMMARY_NAME = "summary.md"
REJECT_EXIT = 10
INPUT_ERROR_EXIT = 2


def finite_float(value: Any) -> float | None:
    if value is None:
        return None
    try:
        parsed = float(str(value).strip())
    except (TypeError, ValueError):
        return None
    return parsed if math.isfinite(parsed) else None


def bool_value(value: Any) -> bool | None:
    if isinstance(value, bool):
        return value
    if value is None:
        return None
    text = str(value).strip().lower()
    if text in {"true", "1", "yes", "y"}:
        return True
    if text in {"false", "0", "no", "n"}:
        return False
    return None


def resolve_summary(path_text: str) -> Path:
    path = Path(path_text)
    if path.is_dir():
        path = path / SUMMARY_NAME
    if not path.exists():
        raise FileNotFoundError(path)
    return path


def parse_report_summary(path: Path) -> dict[str, Any]:
    text = path.read_text(encoding="utf-8")
    data: dict[str, Any] = {"summary_path": str(path)}
    for line in text.splitlines():
        bullet = re.match(r"^- ([A-Za-z0-9_]+): `(.*)`$", line)
        if bullet:
            data[bullet.group(1)] = bullet.group(2)
            continue

        delta = re.match(
            r"^- (map_base_link_delta|map_odom_delta): "
            r"translation_m=`([^`]*)`, dyaw_deg=`([^`]*)`, "
            r"forward_m=`([^`]*)`, left_m=`([^`]*)`$",
            line,
        )
        if delta:
            data[delta.group(1)] = {
                "translation_m": finite_float(delta.group(2)),
                "dyaw_deg": finite_float(delta.group(3)),
                "forward_m": finite_float(delta.group(4)),
                "left_m": finite_float(delta.group(5)),
            }
    return data


def load_pose_report(path_text: str) -> dict[str, Any]:
    summary_path = resolve_summary(path_text)
    report_dir = summary_path.parent
    data = parse_report_summary(summary_path)
    post_summary = report_dir / "post_relocalize_compare" / SUMMARY_NAME
    if post_summary.exists():
        data["post_relocalize_summary_path"] = str(post_summary)
        post_data = parse_report_summary(post_summary)
        for key in ("trigger_accepted", "trigger_message"):
            if key in post_data:
                data[f"post_{key}"] = post_data[key]
    return data


def load_trace_report(path_text: str) -> dict[str, Any]:
    return parse_report_summary(resolve_summary(path_text))


def require_float(
    reasons: list[str],
    data: dict[str, Any],
    key: str,
    *,
    max_abs: float | None = None,
    max_value: float | None = None,
) -> float | None:
    value = finite_float(data.get(key))
    if value is None:
        reasons.append(f"missing_or_invalid_{key}")
        return None
    if max_abs is not None and abs(value) > max_abs:
        reasons.append(f"{key}_abs_{abs(value):.6f}_gt_{max_abs:.6f}")
    if max_value is not None and value > max_value:
        reasons.append(f"{key}_{value:.6f}_gt_{max_value:.6f}")
    return value


def delta_value(data: dict[str, Any], key: str, field: str) -> float | None:
    value = data.get(key)
    if isinstance(value, dict):
        return finite_float(value.get(field))
    return None


def validate_pose_report(data: dict[str, Any], args: argparse.Namespace) -> list[str]:
    reasons: list[str] = []

    state = str(data.get("state", "")).strip().lower()
    if state != "succeeded":
        reasons.append(f"state_not_succeeded:{state or 'missing'}")

    nav2_code = str(data.get("nav2_result_code", "")).strip()
    if nav2_code != "4":
        reasons.append(f"nav2_result_code_not_4:{nav2_code or 'missing'}")

    verified = bool_value(data.get("final_pose_verified"))
    if verified is not True:
        reasons.append(f"final_pose_verified_not_true:{data.get('final_pose_verified', 'missing')}")

    require_float(reasons, data, "final_distance_m", max_value=args.max_online_xy_m)
    require_float(reasons, data, "final_yaw_error_rad", max_value=args.max_online_yaw_rad)

    relocalize_rc = str(data.get("relocalize_exit_code", "")).strip()
    if relocalize_rc != "0":
        reasons.append(f"relocalize_exit_code_not_0:{relocalize_rc or 'missing'}")

    if "post_trigger_accepted" in data:
        trigger_accepted = bool_value(data.get("post_trigger_accepted"))
        if trigger_accepted is not True:
            reasons.append(
                "post_relocalize_trigger_not_accepted:"
                f"{data.get('post_trigger_accepted')}:{data.get('post_trigger_message', '')}"
            )
    elif args.require_post_trigger_accepted:
        reasons.append("post_relocalize_trigger_accepted_missing")

    for delta_key, max_translation in (
        ("map_base_link_delta", args.max_map_base_translation_m),
        ("map_odom_delta", args.max_map_odom_translation_m),
    ):
        translation = delta_value(data, delta_key, "translation_m")
        yaw_deg = delta_value(data, delta_key, "dyaw_deg")
        if translation is None:
            reasons.append(f"missing_{delta_key}_translation_m")
        elif translation > max_translation:
            reasons.append(f"{delta_key}_translation_{translation:.6f}_gt_{max_translation:.6f}")
        if yaw_deg is None:
            reasons.append(f"missing_{delta_key}_dyaw_deg")
        elif abs(yaw_deg) > args.max_correction_yaw_deg:
            reasons.append(f"{delta_key}_dyaw_abs_{abs(yaw_deg):.6f}_gt_{args.max_correction_yaw_deg:.6f}")

    return reasons


def validate_trace_report(data: dict[str, Any], args: argparse.Namespace) -> list[str]:
    reasons: list[str] = []

    child_rc = str(data.get("child_rc", "")).strip()
    if child_rc not in {"", "0"}:
        reasons.append(f"trace_child_rc_not_0:{child_rc}")

    trace_state = str(data.get("last_api_nav_state", "")).strip().lower()
    if trace_state and trace_state != "succeeded":
        reasons.append(f"trace_last_api_nav_state_not_succeeded:{trace_state}")

    trace_phase = str(data.get("last_api_nav_phase", "")).strip().lower()
    if trace_phase and trace_phase != "final_pose_verified":
        reasons.append(f"trace_last_api_nav_phase_not_final_pose_verified:{trace_phase}")

    trace_code = str(data.get("last_api_nav2_result_code", "")).strip()
    if trace_code and trace_code != "4":
        reasons.append(f"trace_last_api_nav2_result_code_not_4:{trace_code}")

    if "last_api_final_distance_m" in data:
        require_float(reasons, data, "last_api_final_distance_m", max_value=args.max_online_xy_m)
    if "last_api_final_yaw_error_rad" in data:
        require_float(reasons, data, "last_api_final_yaw_error_rad", max_value=args.max_online_yaw_rad)

    return reasons


def build_result(
    pose_report: dict[str, Any],
    trace_report: dict[str, Any] | None,
    args: argparse.Namespace,
) -> dict[str, Any]:
    reasons = validate_pose_report(pose_report, args)
    if trace_report is not None:
        reasons.extend(validate_trace_report(trace_report, args))

    metrics = {
        "state": pose_report.get("state"),
        "phase": pose_report.get("phase"),
        "nav2_result_code": pose_report.get("nav2_result_code"),
        "final_pose_verified": pose_report.get("final_pose_verified"),
        "final_distance_m": finite_float(pose_report.get("final_distance_m")),
        "final_yaw_error_rad": finite_float(pose_report.get("final_yaw_error_rad")),
        "relocalize_exit_code": pose_report.get("relocalize_exit_code"),
        "map_base_link_translation_m": delta_value(pose_report, "map_base_link_delta", "translation_m"),
        "map_base_link_dyaw_deg": delta_value(pose_report, "map_base_link_delta", "dyaw_deg"),
        "map_odom_translation_m": delta_value(pose_report, "map_odom_delta", "translation_m"),
        "map_odom_dyaw_deg": delta_value(pose_report, "map_odom_delta", "dyaw_deg"),
    }
    if trace_report is not None:
        metrics.update(
            {
                "trace_last_api_nav_state": trace_report.get("last_api_nav_state"),
                "trace_last_api_nav_phase": trace_report.get("last_api_nav_phase"),
                "trace_last_api_nav2_result_code": trace_report.get("last_api_nav2_result_code"),
                "trace_last_api_final_distance_m": finite_float(trace_report.get("last_api_final_distance_m")),
                "trace_last_api_final_yaw_error_rad": finite_float(trace_report.get("last_api_final_yaw_error_rad")),
            }
        )

    return {
        "accepted": not reasons,
        "reasons": reasons,
        "pose_report": pose_report["summary_path"],
        "trace_report": None if trace_report is None else trace_report["summary_path"],
        "metrics": metrics,
    }


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--pose-report", required=True, help="Pose error report directory or summary.md")
    parser.add_argument("--trace-report", help="Optional terminal yaw trace directory or summary.md")
    parser.add_argument("--json", action="store_true", help="Print machine-readable JSON")
    parser.add_argument("--max-online-xy-m", type=float, default=0.20)
    parser.add_argument("--max-online-yaw-rad", type=float, default=0.08)
    parser.add_argument("--max-map-base-translation-m", type=float, default=0.50)
    parser.add_argument("--max-map-odom-translation-m", type=float, default=0.30)
    parser.add_argument("--max-correction-yaw-deg", type=float, default=2.0)
    parser.add_argument(
        "--allow-missing-post-trigger-accepted",
        dest="require_post_trigger_accepted",
        action="store_false",
        help="Do not require post_relocalize_compare/summary.md trigger_accepted=true",
    )
    parser.set_defaults(require_post_trigger_accepted=True)
    return parser.parse_args(argv)


def print_text(result: dict[str, Any]) -> None:
    verdict = "ACCEPT" if result["accepted"] else "REJECT"
    print(f"[ekf-ab-validate] {verdict}: {result['pose_report']}")
    if result.get("trace_report"):
        print(f"[ekf-ab-validate] trace: {result['trace_report']}")
    metrics = result["metrics"]
    print(
        "[ekf-ab-validate] metrics: "
        f"nav2={metrics.get('nav2_result_code')} "
        f"online_xy={metrics.get('final_distance_m')} "
        f"online_yaw_rad={metrics.get('final_yaw_error_rad')} "
        f"map_base_m={metrics.get('map_base_link_translation_m')} "
        f"map_base_yaw_deg={metrics.get('map_base_link_dyaw_deg')} "
        f"map_odom_m={metrics.get('map_odom_translation_m')} "
        f"map_odom_yaw_deg={metrics.get('map_odom_dyaw_deg')}"
    )
    for reason in result["reasons"]:
        print(f"[ekf-ab-validate] reason: {reason}")


def main(argv: list[str]) -> int:
    args = parse_args(argv)
    try:
        pose_report = load_pose_report(args.pose_report)
        trace_report = load_trace_report(args.trace_report) if args.trace_report else None
    except FileNotFoundError as exc:
        print(f"[ekf-ab-validate] input missing: {exc}", file=sys.stderr)
        return INPUT_ERROR_EXIT

    result = build_result(pose_report, trace_report, args)
    if args.json:
        print(json.dumps(result, indent=2, sort_keys=True))
    else:
        print_text(result)
    return 0 if result["accepted"] else REJECT_EXIT


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
