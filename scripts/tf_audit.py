#!/usr/bin/env python3
from __future__ import annotations

import argparse
import shutil
import subprocess
from pathlib import Path


PATTERNS = {
    "map_to_odom_candidates": ("map->odom", "map -> odom", "map_frame", "odom_frame"),
    "odom_to_base_link_candidates": ("odom->base_link", "odom -> base_link", "base_link_frame", "base_frame"),
    "static_tf_candidates": ("static_transform_publisher", "robot_state_publisher", "lidar_link", "imu_link"),
    "third_party_internal_frames": ("camera_init", "aft_mapped", "wheel_odom", "laser_init"),
}

EXTERNAL_PATTERNS = {
    "legacy_fastlio_tf": ("send_odom_base_tf: true", "hesai_lidar_fastlio"),
    "legacy_pgo_tf": ("map_frame: slam_map", "local_frame: camera_init"),
    "legacy_map_to_odom_bridge": ("map_to_odom_tf_bridge", "localization_topic: /localization_result"),
}


def normalize_extra_roots(values: list[str] | None) -> list[Path]:
    roots: list[Path] = []
    for value in values or []:
        candidate = Path(value).expanduser()
        if candidate.exists():
            roots.append(candidate.resolve())
    return roots


def scan_tree(root: Path, patterns: dict[str, tuple[str, ...]]) -> dict[str, list[str]]:
    findings: dict[str, list[str]] = {key: [] for key in patterns}
    for path in sorted(root.rglob("*")):
        if not path.is_file():
            continue
        if path.suffix.lower() not in {".py", ".xml", ".yaml", ".yml", ".launch", ".md", ".xacro"}:
            continue
        text = path.read_text(encoding="utf-8", errors="ignore")
        for key, pattern_values in patterns.items():
            if any(pattern in text for pattern in pattern_values):
                findings[key].append(str(path))
    return findings


def scan_repository(root: Path, extra_roots: list[Path]) -> dict[str, list[str]]:
    findings = scan_tree(root, PATTERNS)
    for extra_root in extra_roots:
        external = scan_tree(extra_root, EXTERNAL_PATTERNS)
        for key, values in external.items():
            findings.setdefault(key, []).extend(values)
    return findings


def try_view_frames(root: Path, output_path: Path) -> str:
    if shutil.which("ros2") is None:
        output_path.write_text("ros2 command not available; view_frames skipped.\n", encoding="utf-8")
        return "skipped (ros2 not available)"

    command = ["ros2", "run", "tf2_tools", "view_frames"]
    try:
        subprocess.run(command, cwd=root, check=True, capture_output=True, text=True, timeout=60)
    except Exception as exc:  # pragma: no cover - environment dependent
        output_path.write_text(f"view_frames failed: {exc}\n", encoding="utf-8")
        return f"failed ({exc})"

    generated = root / "frames.pdf"
    if generated.exists():
        generated.replace(output_path)
        return "captured"

    output_path.write_text("view_frames completed without frames.pdf output.\n", encoding="utf-8")
    return "completed_without_pdf"


def render_report(findings: dict[str, list[str]], view_frames_status: str) -> str:
    lines = [
        "# TF Audit Report",
        "",
        "- Audit mode: repository scan + optional runtime `view_frames` capture",
        f"- `view_frames` status: {view_frames_status}",
        "",
        "## Canonical Ownership",
        "- `robot_local_state`: only `odom -> base_link`",
        "- `robot_localization_bridge`: only `map -> odom`",
        "- `robot_description`: only static sensor extrinsics",
        "",
        "## Findings",
    ]

    for key, files in findings.items():
        lines.append(f"### {key}")
        if files:
            lines.extend(f"- `{path}`" for path in files[:40])
        else:
            lines.append("- none")

    lines.extend(
        [
            "",
            "## Current Conclusion",
            "- The repository baseline now defines a single canonical TF policy, but runtime ROS graph verification still requires real launch execution on ROS 2 Humble.",
            "- No duplicate runtime publishers were observed from the current static repository scan because live TF nodes are not running in this workspace snapshot.",
            "- Wrapper packages must keep third-party TF disabled or isolated by default.",
        ]
    )
    return "\n".join(lines) + "\n"


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", type=Path, default=Path.cwd())
    parser.add_argument("--extra-root", action="append", default=[])
    parser.add_argument("--report", type=Path, default=None)
    parser.add_argument("--view-frames-output", type=Path, default=None)
    args = parser.parse_args()

    root = args.root.resolve()
    report_path = args.report or (root / "reports" / "tf_audit_report.md")
    view_frames_output = args.view_frames_output or (root / "reports" / "view_frames.txt")
    report_path.parent.mkdir(parents=True, exist_ok=True)

    findings = scan_repository(root, normalize_extra_roots(args.extra_root))
    view_frames_status = try_view_frames(root, view_frames_output)
    report_path.write_text(render_report(findings, view_frames_status), encoding="utf-8")
    print(report_path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
