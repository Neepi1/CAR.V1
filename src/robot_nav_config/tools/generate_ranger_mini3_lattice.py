#!/usr/bin/env python3
"""Generate the pinned Ranger Mini3 Nav2 state-lattice artifact.

The operational runtime consumes the checked-in JSON artifact. This tool is
only for reproducible regeneration from the Nav2 Humble 1.1.20 generator.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import subprocess
import sys
import types
from typing import Any, Iterable


PINNED_NAV2_TAG = "1.1.20"
PINNED_NAV2_COMMIT = "a097086719c88f781aa59788eca29ac6ca5e56db"
ARTIFACT_DATE = "2026-07-21"
PINNED_SOURCE_HASHES = {
    "constants.py": "9f618c72d2428df571271eefc193a1525377e6df85e9579b4e9a8e5e8521edb8",
    "helper.py": "5638272e7ac6e1e8b0d76717d52f9d502b9561c0d05610652b52ce286263fed2",
    "trajectory.py": "6e449fb3c5193aba86bc7b24e8bf90451fe0e0974a7b673237d1e17c8e5f3d9c",
    "trajectory_generator.py": "a6a23118db8d4c38bf3161e8ee2d039af7d722f3b96dd6a941e34f3779033b59",
    "lattice_generator.py": "49b4d7fd0044b6e479192c78306e5cf2621cb3821170d7fc0666538144cf8154",
}

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CONFIG = PACKAGE_ROOT / "lattice" / "ranger_mini3_ackermann_0p05m_0p81m_16.config.json"
DEFAULT_OUTPUT = PACKAGE_ROOT / "lattice" / "ranger_mini3_ackermann_0p05m_0p81m_16.json"
DEFAULT_MANIFEST = PACKAGE_ROOT / "lattice" / "ranger_mini3_ackermann_0p05m_0p81m_16.manifest.json"


def normalized_sha256(path: Path) -> str:
    normalized = path.read_text(encoding="utf-8").replace("\r\n", "\n").replace("\r", "\n")
    return hashlib.sha256(normalized.encode("utf-8")).hexdigest()


def file_sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def locate_generator(path: Path) -> Path:
    candidates = (
        path,
        path / "lattice_primitives",
        path / "nav2_smac_planner" / "lattice_primitives",
    )
    for candidate in candidates:
        if all((candidate / filename).is_file() for filename in PINNED_SOURCE_HASHES):
            return candidate.resolve()
    raise RuntimeError(f"Nav2 lattice_primitives was not found below {path}")


def verify_generator_source(generator_root: Path) -> None:
    mismatches = []
    for filename, expected in PINNED_SOURCE_HASHES.items():
        actual = normalized_sha256(generator_root / filename)
        if actual != expected:
            mismatches.append(f"{filename}: expected {expected}, got {actual}")
    if mismatches:
        raise RuntimeError(
            "Nav2 lattice generator does not match the pinned Humble source:\n  "
            + "\n  ".join(mismatches)
        )


class _LinearSpatialIndex:
    """Small development fallback for the generator's optional Rtree dependency."""

    def __init__(self) -> None:
        self._entries: list[tuple[tuple[float, float, float, float], Any]] = []

    def insert(self, _entry_id: int, bounds: Iterable[float], obj: Any = None) -> None:
        self._entries.append((tuple(float(value) for value in bounds), obj))

    def intersection(self, bounds: Iterable[float], objects: str | bool = False):
        left, bottom, right, top = (float(value) for value in bounds)
        for (item_left, item_bottom, item_right, item_top), obj in self._entries:
            overlaps = not (
                item_right < left
                or item_left > right
                or item_top < bottom
                or item_bottom > top
            )
            if overlaps:
                yield obj if objects == "raw" else obj


def install_rtree_fallback_if_needed() -> None:
    try:
        __import__("rtree")
        return
    except ImportError:
        pass

    index_module = types.ModuleType("rtree.index")
    index_module.Index = _LinearSpatialIndex
    index_module.Rtree = _LinearSpatialIndex
    rtree_module = types.ModuleType("rtree")
    rtree_module.index = index_module
    sys.modules["rtree"] = rtree_module
    sys.modules["rtree.index"] = index_module


def angle_sort_key(angle: float) -> tuple[bool, float]:
    return (angle < 0.0, angle)


def serialize_lattice(config: dict[str, Any], minimal_set: dict[Any, list[Any]]) -> dict[str, Any]:
    source_headings = sorted(minimal_set.keys(), key=angle_sort_key)
    heading_lookup = {heading: index for index, heading in enumerate(source_headings)}
    heading_angles = [float(angle + 2.0 * math.pi if angle < 0.0 else angle) for angle in source_headings]

    output: dict[str, Any] = {
        "version": 1.0,
        "date_generated": ARTIFACT_DATE,
        "lattice_metadata": dict(config),
        "primitives": [],
    }
    output["lattice_metadata"]["heading_angles"] = heading_angles

    trajectory_id = 0
    for start_angle in source_headings:
        trajectories = sorted(
            minimal_set[start_angle],
            key=lambda trajectory: trajectory.parameters.end_angle,
        )
        for trajectory in trajectories:
            parameters = trajectory.parameters
            output["primitives"].append(
                {
                    "trajectory_id": trajectory_id,
                    "start_angle_index": heading_lookup[parameters.start_angle],
                    "end_angle_index": heading_lookup[parameters.end_angle],
                    "left_turn": bool(parameters.left_turn),
                    "trajectory_radius": float(parameters.turning_radius),
                    "trajectory_length": round(float(parameters.total_length), 5),
                    "arc_length": round(float(parameters.arc_length), 5),
                    "straight_length": round(
                        float(
                            parameters.start_straight_length
                            + parameters.end_straight_length
                        ),
                        5,
                    ),
                    "poses": trajectory.path.to_output_format(),
                }
            )
            trajectory_id += 1

    output["lattice_metadata"]["number_of_trajectories"] = trajectory_id
    return output


def add_ranger_spin_primitives(output: dict[str, Any]) -> None:
    """Add one-bin spin transitions while preserving Nav2's start-heading grouping."""
    headings = output["lattice_metadata"]["heading_angles"]
    groups: dict[int, list[dict[str, Any]]] = {index: [] for index in range(len(headings))}
    for primitive in output["primitives"]:
        groups[int(primitive["start_angle_index"])].append(primitive)

    for start_index, start_yaw in enumerate(headings):
        for direction in (-1, 1):
            end_index = (start_index + direction) % len(headings)
            end_yaw = headings[end_index]
            delta = math.atan2(math.sin(end_yaw - start_yaw), math.cos(end_yaw - start_yaw))
            poses = []
            for sample_index in range(4):
                if sample_index == 3:
                    sample_yaw = end_yaw
                else:
                    sample_yaw = (start_yaw + delta * sample_index / 3.0) % (2.0 * math.pi)
                poses.append([0.0, 0.0, sample_yaw])
            groups[start_index].append(
                {
                    "start_angle_index": start_index,
                    "end_angle_index": end_index,
                    "left_turn": delta > 0.0,
                    "trajectory_radius": 0.0,
                    "trajectory_length": 0.0,
                    "arc_length": 0.0,
                    "straight_length": 0.0,
                    "poses": poses,
                }
            )

    primitives = []
    for start_index in range(len(headings)):
        groups[start_index].sort(
            key=lambda primitive: (
                int(primitive["end_angle_index"]),
                float(primitive["trajectory_length"]),
                float(primitive["trajectory_radius"]),
            )
        )
        primitives.extend(groups[start_index])
    for trajectory_id, primitive in enumerate(primitives):
        primitive["trajectory_id"] = trajectory_id

    output["primitives"] = primitives
    output["lattice_metadata"]["number_of_trajectories"] = len(primitives)


def git_commit_for(generator_root: Path) -> str:
    repository = generator_root.parents[1]
    try:
        result = subprocess.run(
            ["git", "-C", str(repository), "rev-parse", "HEAD"],
            check=True,
            capture_output=True,
            text=True,
        )
        return result.stdout.strip()
    except (OSError, subprocess.CalledProcessError):
        return "unavailable"


def write_manifest(
    manifest_path: Path,
    artifact_path: Path,
    config_path: Path,
    generator_root: Path,
) -> None:
    artifact = json.loads(artifact_path.read_text(encoding="utf-8"))
    primitives = artifact["primitives"]
    spin_primitives = [
        primitive
        for primitive in primitives
        if abs(float(primitive["trajectory_length"])) < 1.0e-7
    ]
    translation_primitives = [
        primitive
        for primitive in primitives
        if abs(float(primitive["trajectory_length"])) >= 1.0e-7
    ]
    curved = [
        primitive
        for primitive in translation_primitives
        if abs(float(primitive["arc_length"])) >= 1.0e-7
    ]
    radii = [abs(float(primitive["trajectory_radius"])) for primitive in curved]
    lengths = [float(primitive["trajectory_length"]) for primitive in primitives]
    translation_lengths = [
        float(primitive["trajectory_length"]) for primitive in translation_primitives
    ]
    manifest = {
        "schema": "njrh.ranger_mini3_lattice_manifest.v2",
        "artifact": artifact_path.name,
        "artifact_sha256": file_sha256(artifact_path),
        "config": config_path.name,
        "config_sha256": file_sha256(config_path),
        "artifact_date": ARTIFACT_DATE,
        "generator": {
            "upstream_repository": "https://github.com/ros-navigation/navigation2",
            "upstream_tag": PINNED_NAV2_TAG,
            "upstream_commit": PINNED_NAV2_COMMIT,
            "observed_commit": git_commit_for(generator_root),
            "normalized_source_sha256": PINNED_SOURCE_HASHES,
            "rtree_fallback": "linear bounding-box index; generation only",
        },
        "motion_contract": {
            "platform": "AgileX Ranger Mini3",
            "costmap_resolution_m": 0.05,
            "minimum_turning_radius_m": 0.81,
            "heading_count": 16,
            "primitive_motion_mode": "dual_ackermann_bidirectional_plus_in_place_spin",
            "allow_reverse_expansion": True,
            "reverse_execution_scope": "terminal_goal_window_only_v1",
            "reverse_admission_distance_m": 0.3,
            "contains_spin_primitives": True,
            "contains_lateral_primitives": False,
            "spin_execution_owner": (
                "GoalScopedRotationShimController for path-entry and final heading via "
                "robot_safety and ranger_base"
            ),
            "mid_path_spin_execution": "not_admitted_v1",
            "lateral_owner": "docking/API via robot_safety and ranger_base",
        },
        "artifact_statistics": {
            "primitive_count": len(primitives),
            "spin_count": len(spin_primitives),
            "straight_count": len(translation_primitives) - len(curved),
            "left_turn_count": sum(bool(primitive["left_turn"]) for primitive in curved),
            "right_turn_count": sum(not bool(primitive["left_turn"]) for primitive in curved),
            "generated_minimum_radius_m": min(radii),
            "generated_maximum_radius_m": max(radii),
            "minimum_primitive_length_m": min(lengths),
            "maximum_primitive_length_m": max(lengths),
            "minimum_translation_primitive_length_m": min(translation_lengths),
        },
    }
    manifest_path.parent.mkdir(parents=True, exist_ok=True)
    manifest_path.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8", newline="\n")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--nav2-source", type=Path, required=True)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--manifest", type=Path, default=DEFAULT_MANIFEST)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    generator_root = locate_generator(args.nav2_source)
    verify_generator_source(generator_root)

    observed_commit = git_commit_for(generator_root)
    if observed_commit not in (PINNED_NAV2_COMMIT, "unavailable"):
        raise RuntimeError(
            f"Nav2 checkout is at {observed_commit}, expected {PINNED_NAV2_COMMIT}"
        )

    config = json.loads(args.config.read_text(encoding="utf-8"))
    expected_config = {
        "motion_model": "ackermann",
        "turning_radius": 0.81,
        "grid_resolution": 0.05,
        "stopping_threshold": 5,
        "num_of_headings": 16,
    }
    if config != expected_config:
        raise RuntimeError(f"Ranger Mini3 lattice config changed: {config!r}")

    install_rtree_fallback_if_needed()
    sys.path.insert(0, str(generator_root))
    from lattice_generator import LatticeGenerator  # pylint: disable=import-error,import-outside-toplevel

    minimal_set = LatticeGenerator(config).run()
    output = serialize_lattice(config, minimal_set)
    add_ranger_spin_primitives(output)

    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(output, indent="\t") + "\n", encoding="utf-8", newline="\n")
    write_manifest(args.manifest, args.output, args.config, generator_root)
    print(f"generated {len(output['primitives'])} primitives: {args.output}")
    print(f"manifest: {args.manifest}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
