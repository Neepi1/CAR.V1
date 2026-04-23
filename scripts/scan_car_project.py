#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
from dataclasses import dataclass, asdict
from pathlib import Path
from typing import Iterable


ASSET_HINTS = {
    "jt128": ("jt128", "hesai", "lidar"),
    "fastlio": ("fastlio", "fast_lio", "lio"),
    "pgo": ("pgo", "loop", "closure"),
    "isaac_localizer": ("isaac", "localizer", "map_localization"),
    "docker": ("dockerfile", "devcontainer", "compose"),
    "network": ("ip", "udp", "port", "nic"),
    "extrinsics": ("extrinsic", "calib", "imu_link", "lidar_link"),
}

KNOWN_REUSE_PATHS = {
    "jt128": (
        "hesai_jt128/configs/jt128_network.yaml",
        "hesai_jt128/configs/ros2_driver_params.yaml",
        "ros2_ws/src/hesai_lidar_ros2/config/config.yaml",
    ),
    "extrinsics": (
        "ros2_ws/src/car_description/launch/hesai_lidar_tf.launch.py",
        "ros2_ws/src/car_description/urdf/hesai_lidar_mount.urdf.xacro",
        "ros2_ws/src/jt128_nav_tools/config/jt128_fastlio_pointcloud_remap.yaml",
        "ros2_ws/src/jt128_nav_tools/config/jt128_fastlio_imu_remap.yaml",
    ),
    "fastlio": (
        "ros2_ws/src/fast_lio/config/jt128.yaml",
        "docs/fast_lio2_mapping.md",
    ),
    "pgo": (
        "ros2_ws/src/fastlio_pgo/config/pgo.yaml",
        "ros2_ws/src/fastlio_pgo/launch",
    ),
    "isaac_localizer": (
        "nav2_test/params/jt128_occupancy_grid_localizer.yaml",
        "nav2_test/params/jt128_flatscan.yaml",
        "nav2_test/params/jt128_map_to_odom_tf_bridge.yaml",
        "nav2_test/jt128_occupancy_localization.launch.py",
    ),
    "nav2": (
        "nav2_test/params/nav2_jt128_rapid_avoidance.yaml",
        "nav2_test/jt128_nav_sensing.launch.py",
    ),
    "chassis": (
        "ros2_ws/src/ranger_ros2",
        "ros2_ws/src/ugv_sdk",
        "docs/ranger_mini_v3_integration.md",
    ),
    "docker": (
        "Dockerfile.car",
        ".tmp_isaac_ros_ws.bashrc",
        ".tmp_isaac_ros_ws_override.bashrc",
    ),
}


@dataclass
class Candidate:
    path: str
    category: str
    reason: str


def normalize_extra_roots(values: list[str] | None) -> list[Path]:
    roots: list[Path] = []
    for value in values or []:
        candidate = Path(value).expanduser()
        if candidate.exists():
            roots.append(candidate.resolve())
    return roots


def iter_candidate_roots(root: Path, extra_roots: list[Path]) -> Iterable[Path]:
    yielded: list[Path] = []

    def emit(candidate: Path) -> Iterable[Path]:
        resolved = candidate.resolve()
        if resolved.exists() and resolved not in yielded:
            yielded.append(resolved)
            yield resolved

    for rel in ("car", "car_project", "src"):
        candidate = (root / rel).resolve()
        yield from emit(candidate)

    parent = root.parent.resolve()
    for rel in ("car", "car_project"):
        candidate = (parent / rel).resolve()
        yield from emit(candidate)

    for child in root.glob("src/car*"):
        yield from emit(child)

    for child in parent.glob("car*"):
        yield from emit(child)

    for child in extra_roots:
        yield from emit(child)


def detect_assets(root: Path) -> list[Candidate]:
    candidates: list[Candidate] = []
    for category, rel_paths in KNOWN_REUSE_PATHS.items():
        for rel_path in rel_paths:
            known_path = root / rel_path
            if known_path.exists():
                candidates.append(Candidate(path=str(known_path), category=category, reason="known_reuse_path"))

    for path in sorted(root.rglob("*")):
        if not path.is_file():
            continue
        name = path.name.lower()
        relative = str(path)
        for category, hints in ASSET_HINTS.items():
            if any(hint in name or hint in relative.lower() for hint in hints):
                candidates.append(Candidate(path=relative, category=category, reason="filename_or_path_hint"))
                break
    return candidates


def render_markdown(root: Path, roots: list[Path], assets: list[Candidate]) -> str:
    grouped: dict[str, list[Candidate]] = {}
    for asset in assets:
        grouped.setdefault(asset.category, []).append(asset)

    lines = [
        "# Car Project Reuse Report",
        "",
        f"- Workspace root: `{root}`",
        "- Scan policy: local-first, network only as fallback",
        "",
        "## Candidate Roots",
    ]

    if roots:
        lines.extend(f"- `{candidate}`" for candidate in roots)
    else:
        lines.append("- No local `car` / `car_project` roots found in the required scan patterns.")

    lines.extend(["", "## Indexed Assets"])

    if grouped:
        for category in sorted(grouped):
            lines.append(f"### {category}")
            lines.extend(f"- `{item.path}` ({item.reason})" for item in grouped[category][:30])
    else:
        lines.append("- No reusable local assets were detected under the required scan roots.")

    local_paths = {asset.path for asset in assets}
    direct_reuse = [
        path
        for rel_paths in KNOWN_REUSE_PATHS.values()
        for path in [str(candidate) for candidate in roots]
    ]
    _ = direct_reuse

    lines.extend(["", "## Reuse Decision"])
    if assets:
        lines.extend(
            [
                "- Direct reuse candidates detected for JT128 networking, Hesai ROS 2 config, FAST-LIO2 config, PGO config, Isaac occupancy localizer params, lidar extrinsics, Ranger Mini V3 docs, and Docker assets.",
                "- Wrapped reuse candidates detected for `hesai_lidar_ros2`, `fast_lio`, `fastlio_pgo`, `jt128_nav_tools`, `ranger_ros2`, and `ugv_sdk` source trees.",
                "- Remaining likely network fallback items: upstream Isaac ROS source packages and `robot_localization` source are not bundled in the discovered car repository tree.",
            ]
        )
    else:
        lines.extend(
            [
                "- Direct reuse: none detected yet in the current workspace or parent scan roots.",
                "- Wrapped reuse: wrappers are prepared to inject local configs when matching assets appear later.",
                "- Missing locally and requires fallback planning: Hesai driver source, FAST-LIO2 source, PGO backend source, Isaac localizer source, verified launch/config bundles.",
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
    report_path = args.report or (root / "reports" / "car_project_reuse_report.md")
    json_path = args.json or (root / "reports" / "car_project_reuse_index.json")
    report_path.parent.mkdir(parents=True, exist_ok=True)

    extra_roots = normalize_extra_roots(args.extra_root)
    roots = []
    for candidate in iter_candidate_roots(root, extra_roots):
        if candidate not in roots:
            roots.append(candidate)

    assets: list[Candidate] = []
    for candidate in roots:
        assets.extend(detect_assets(candidate))

    report_path.write_text(render_markdown(root, roots, assets), encoding="utf-8")
    json_path.write_text(json.dumps([asdict(asset) for asset in assets], indent=2, ensure_ascii=False), encoding="utf-8")
    print(report_path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
