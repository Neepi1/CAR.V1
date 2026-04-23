#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
from pathlib import Path


REQUIRED_COMPONENTS = {
    "hesai_driver": [
        "ros2_ws/src/hesai_lidar_ros2",
        "hesai_jt128/configs/jt128_network.yaml",
        "hesai_jt128/configs/ros2_driver_params.yaml",
    ],
    "fastlio2": [
        "ros2_ws/src/fast_lio",
        "ros2_ws/src/fast_lio/config/jt128.yaml",
    ],
    "pgo_backend": [
        "ros2_ws/src/fastlio_pgo",
        "ros2_ws/src/fastlio_pgo/config/pgo.yaml",
    ],
    "isaac_localizer": [
        "nav2_test/params/jt128_occupancy_grid_localizer.yaml",
        "nav2_test/params/jt128_flatscan.yaml",
        "nav2_test/jt128_occupancy_localization.launch.py",
    ],
    "nav2": [
        "nav2_test/params/nav2_jt128_rapid_avoidance.yaml",
        "nav2_test/behavior_tree/jt128_rapid_avoidance_replanning_and_recovery.xml",
    ],
    "robot_localization": ["robot_localization"],
    "ranger_driver": [
        "ros2_ws/src/ranger_ros2",
        "ros2_ws/src/ugv_sdk",
        "docs/ranger_mini_v3_integration.md",
    ],
    "nav_tools": [
        "ros2_ws/src/jt128_nav_tools",
        "nav2_test/params/jt128_map_to_odom_tf_bridge.yaml",
    ],
}


def normalize_extra_roots(values: list[str] | None) -> list[Path]:
    roots: list[Path] = []
    for value in values or []:
        candidate = Path(value).expanduser()
        if candidate.exists():
            roots.append(candidate.resolve())
    return roots


def find_local_matches(root: Path, extra_roots: list[Path]) -> dict[str, list[str]]:
    matches: dict[str, list[str]] = {key: [] for key in REQUIRED_COMPONENTS}
    search_roots = [root, root.parent, *extra_roots]
    for search_root in search_roots:
        for component, hints in REQUIRED_COMPONENTS.items():
            for hint in hints:
                candidate = search_root / hint
                if candidate.exists():
                    matches[component].append(str(candidate))
        for path in search_root.rglob("*"):
            lowered = str(path).lower().replace("\\", "/")
            for component, hints in REQUIRED_COMPONENTS.items():
                if any(hint.lower().replace("\\", "/") in lowered for hint in hints):
                    matches[component].append(str(path))
    return matches


def render_report(matches: dict[str, list[str]]) -> str:
    lines = [
        "# Third Party Resolution Report",
        "",
        "- Resolution order: local workspace -> parent workspace -> fallback `.repos` definitions",
        "",
        "## Component Resolution",
    ]

    for component, items in matches.items():
        lines.append(f"### {component}")
        unique_items = []
        for item in items:
            if item not in unique_items:
                unique_items.append(item)
        if unique_items:
            lines.extend(f"- local candidate: `{item}`" for item in unique_items[:20])
            lines.append("- decision: prefer local validation before any network fetch")
            if component == "isaac_localizer":
                lines.append("- note: local params and launch exist, but the upstream Isaac ROS source package may still need separate validation or fetch")
        else:
            lines.append("- local candidate: none found")
            lines.append("- decision: keep fallback repo entry; do not fetch automatically")

    lines.extend(
        [
            "",
            "## Fallback Policy",
            "- Missing components remain declared in `.repos/third_party.repos` only as explicit fallback metadata.",
            "- This script does not fetch from the network; it only documents whether a local-first path exists.",
        ]
    )
    return "\n".join(lines) + "\n"


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", type=Path, default=Path.cwd())
    parser.add_argument("--extra-root", action="append", default=[])
    parser.add_argument("--report", type=Path, default=None)
    parser.add_argument("--json", type=Path, default=None)
    args = parser.parse_args()

    root = args.root.resolve()
    report_path = args.report or (root / "reports" / "third_party_resolution_report.md")
    json_path = args.json or (root / "reports" / "third_party_resolution_index.json")
    report_path.parent.mkdir(parents=True, exist_ok=True)

    matches = find_local_matches(root, normalize_extra_roots(args.extra_root))
    report_path.write_text(render_report(matches), encoding="utf-8")
    json_path.write_text(json.dumps(matches, indent=2, ensure_ascii=False), encoding="utf-8")
    print(report_path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
