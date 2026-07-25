#!/usr/bin/env python3
"""Validate the checked-in Ranger Mini3 state-lattice artifact."""

from __future__ import annotations

import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path
import sys
from typing import Any


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CONFIG = PACKAGE_ROOT / "lattice" / "ranger_mini3_ackermann_0p05m_0p81m_16.config.json"
DEFAULT_LATTICE = PACKAGE_ROOT / "lattice" / "ranger_mini3_ackermann_0p05m_0p81m_16.json"
DEFAULT_MANIFEST = PACKAGE_ROOT / "lattice" / "ranger_mini3_ackermann_0p05m_0p81m_16.manifest.json"


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def yaw_error(lhs: float, rhs: float) -> float:
    return math.atan2(math.sin(lhs - rhs), math.cos(lhs - rhs))


def require(condition: bool, message: str, errors: list[str]) -> None:
    if not condition:
        errors.append(message)


def validate(
    config_path: Path,
    lattice_path: Path,
    manifest_path: Path,
) -> list[str]:
    errors: list[str] = []
    for path in (config_path, lattice_path, manifest_path):
        require(path.is_file(), f"missing file: {path}", errors)
    if errors:
        return errors

    config = json.loads(config_path.read_text(encoding="utf-8"))
    lattice = json.loads(lattice_path.read_text(encoding="utf-8"))
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    metadata = lattice.get("lattice_metadata", {})
    primitives = lattice.get("primitives", [])

    expected_config = {
        "motion_model": "ackermann",
        "turning_radius": 0.81,
        "grid_resolution": 0.05,
        "stopping_threshold": 5,
        "num_of_headings": 16,
    }
    require(config == expected_config, f"unexpected generator config: {config!r}", errors)
    for key, value in expected_config.items():
        require(metadata.get(key) == value, f"metadata {key} != {value!r}", errors)

    headings = metadata.get("heading_angles", [])
    require(len(headings) == 16, f"expected 16 headings, got {len(headings)}", errors)
    require(
        metadata.get("number_of_trajectories") == len(primitives),
        "number_of_trajectories does not match primitives",
        errors,
    )
    require(len(primitives) > 0, "lattice has no primitives", errors)
    require(lattice.get("date_generated") == "2026-07-21", "artifact date is not pinned", errors)

    ids = []
    outgoing: Counter[int] = Counter()
    spin_outgoing: Counter[int] = Counter()
    left_count = 0
    right_count = 0
    straight_count = 0
    spin_count = 0
    grid_resolution = 0.05
    minimum_radius = 0.81

    for primitive_index, primitive in enumerate(primitives):
        prefix = f"primitive[{primitive_index}]"
        trajectory_id = primitive.get("trajectory_id")
        ids.append(trajectory_id)
        start_index = primitive.get("start_angle_index")
        end_index = primitive.get("end_angle_index")
        require(isinstance(start_index, int) and 0 <= start_index < len(headings), f"{prefix} bad start index", errors)
        require(isinstance(end_index, int) and 0 <= end_index < len(headings), f"{prefix} bad end index", errors)
        if not isinstance(start_index, int) or not (0 <= start_index < len(headings)):
            continue
        if not isinstance(end_index, int) or not (0 <= end_index < len(headings)):
            continue

        outgoing[start_index] += 1
        poses = primitive.get("poses", [])
        require(len(poses) >= 2, f"{prefix} has fewer than two poses", errors)
        if len(poses) < 2:
            continue
        for pose_index, pose in enumerate(poses):
            valid_pose = (
                isinstance(pose, list)
                and len(pose) == 3
                and all(isinstance(value, (int, float)) and math.isfinite(value) for value in pose)
            )
            require(valid_pose, f"{prefix}.poses[{pose_index}] is invalid", errors)
        if any(len(pose) != 3 for pose in poses):
            continue

        first_pose = poses[0]
        end_pose = poses[-1]
        trajectory_length = abs(float(primitive.get("trajectory_length", 0.0)))
        arc_length = abs(float(primitive.get("arc_length", 0.0)))
        is_spin = trajectory_length < 1.0e-7

        if is_spin:
            spin_count += 1
            spin_outgoing[start_index] += 1
            require(
                (end_index - start_index) % len(headings) in (1, len(headings) - 1),
                f"{prefix} spin does not move exactly one heading bin",
                errors,
            )
            require(
                all(math.hypot(pose[0], pose[1]) < 1.0e-9 for pose in poses),
                f"{prefix} spin translates in XY",
                errors,
            )
            require(arc_length < 1.0e-7, f"{prefix} spin has arc length", errors)
            require(
                abs(float(primitive.get("straight_length", 0.0))) < 1.0e-7,
                f"{prefix} spin has straight length",
                errors,
            )
            require(
                abs(float(primitive.get("trajectory_radius", 0.0))) < 1.0e-7,
                f"{prefix} spin has a turning radius",
                errors,
            )
            require(
                abs(yaw_error(first_pose[2], headings[start_index])) < 1.0e-6,
                f"{prefix} spin start yaw mismatch",
                errors,
            )
            require(
                abs(yaw_error(end_pose[2], headings[end_index])) < 1.0e-6,
                f"{prefix} spin end yaw mismatch",
                errors,
            )
            continue

        # Nav2's generator omits the origin and starts the sampled path about
        # one grid cell into the primitive. Curved primitives have already
        # changed yaw at that first stored sample.
        first_distance = math.hypot(first_pose[0], first_pose[1])
        require(
            0.0 < first_distance <= 1.5 * grid_resolution,
            f"{prefix} first sample is not one grid step from origin",
            errors,
        )
        first_forward_projection = (
            first_pose[0] * math.cos(headings[start_index])
            + first_pose[1] * math.sin(headings[start_index])
        )
        require(
            first_forward_projection > 0.0,
            f"{prefix} first sample moves behind its start heading",
            errors,
        )
        require(abs(yaw_error(end_pose[2], headings[end_index])) < 1.0e-6, f"{prefix} end yaw mismatch", errors)
        require(math.hypot(end_pose[0], end_pose[1]) >= grid_resolution, f"{prefix} translation is too short", errors)
        for coordinate_name, coordinate in (("x", end_pose[0]), ("y", end_pose[1])):
            grid_error = abs(coordinate / grid_resolution - round(coordinate / grid_resolution))
            require(grid_error < 1.0e-5, f"{prefix} endpoint {coordinate_name} is off-grid", errors)

        forward_projection = (
            end_pose[0] * math.cos(headings[start_index])
            + end_pose[1] * math.sin(headings[start_index])
        )
        require(forward_projection > 0.0, f"{prefix} is not a forward Ackermann primitive", errors)

        radius = abs(float(primitive.get("trajectory_radius", 0.0)))
        if arc_length < 1.0e-7:
            straight_count += 1
        else:
            require(radius + 1.0e-6 >= minimum_radius, f"{prefix} radius {radius} < {minimum_radius}", errors)
            if primitive.get("left_turn"):
                left_count += 1
            else:
                right_count += 1

    require(ids == list(range(len(primitives))), "trajectory IDs are not contiguous", errors)
    require(set(outgoing) == set(range(16)), "not every heading has outgoing primitives", errors)
    require(min(outgoing.values(), default=0) >= 5, "a heading has fewer than five primitives", errors)
    require(spin_count == 32, f"expected 32 spin primitives, got {spin_count}", errors)
    require(
        set(spin_outgoing) == set(range(16)) and set(spin_outgoing.values()) == {2},
        "each heading must have exactly two one-bin spin primitives",
        errors,
    )
    require(straight_count > 0, "lattice has no straight primitives", errors)
    require(left_count > 0 and right_count > 0, "lattice lacks left/right symmetry", errors)

    require(
        manifest.get("schema") == "njrh.ranger_mini3_lattice_manifest.v2",
        "manifest schema mismatch",
        errors,
    )
    require(manifest.get("artifact") == lattice_path.name, "manifest artifact name mismatch", errors)
    require(manifest.get("artifact_sha256") == sha256(lattice_path), "artifact SHA256 mismatch", errors)
    require(manifest.get("config_sha256") == sha256(config_path), "config SHA256 mismatch", errors)
    contract = manifest.get("motion_contract", {})
    require(contract.get("allow_reverse_expansion") is True, "manifest disables reverse expansion", errors)
    require(
        contract.get("reverse_execution_scope") == "terminal_goal_window_only_v1",
        "manifest reverse execution scope mismatch",
        errors,
    )
    require(
        contract.get("reverse_admission_distance_m") == 0.3,
        "manifest reverse admission distance mismatch",
        errors,
    )
    require(contract.get("contains_spin_primitives") is True, "manifest omits spin primitives", errors)
    require(contract.get("contains_lateral_primitives") is False, "manifest permits lateral primitives", errors)
    require(
        contract.get("mid_path_spin_execution") == "not_admitted_v1",
        "manifest does not prohibit unimplemented mid-path spin execution",
        errors,
    )
    statistics = manifest.get("artifact_statistics", {})
    curved_radii = [
        abs(float(primitive["trajectory_radius"]))
        for primitive in primitives
        if abs(float(primitive["arc_length"])) >= 1.0e-7
    ]
    primitive_lengths = [float(primitive["trajectory_length"]) for primitive in primitives]
    translation_lengths = [length for length in primitive_lengths if abs(length) >= 1.0e-7]
    require(statistics.get("primitive_count") == len(primitives), "manifest primitive count mismatch", errors)
    require(statistics.get("spin_count") == spin_count, "manifest spin count mismatch", errors)
    require(statistics.get("straight_count") == straight_count, "manifest straight count mismatch", errors)
    require(statistics.get("left_turn_count") == left_count, "manifest left count mismatch", errors)
    require(statistics.get("right_turn_count") == right_count, "manifest right count mismatch", errors)
    if curved_radii:
        require(statistics.get("generated_minimum_radius_m") == min(curved_radii), "manifest minimum radius mismatch", errors)
        require(statistics.get("generated_maximum_radius_m") == max(curved_radii), "manifest maximum radius mismatch", errors)
    if primitive_lengths:
        require(statistics.get("minimum_primitive_length_m") == min(primitive_lengths), "manifest minimum length mismatch", errors)
        require(statistics.get("maximum_primitive_length_m") == max(primitive_lengths), "manifest maximum length mismatch", errors)
    if translation_lengths:
        require(
            statistics.get("minimum_translation_primitive_length_m") == min(translation_lengths),
            "manifest minimum translation length mismatch",
            errors,
        )
    return errors


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--lattice", type=Path, default=DEFAULT_LATTICE)
    parser.add_argument("--manifest", type=Path, default=DEFAULT_MANIFEST)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    errors = validate(args.config, args.lattice, args.manifest)
    if errors:
        for error in errors:
            print(f"ERROR: {error}", file=sys.stderr)
        return 1
    lattice = json.loads(args.lattice.read_text(encoding="utf-8"))
    metadata = lattice["lattice_metadata"]
    print(
        "valid Ranger Mini3 lattice: "
        f"{metadata['number_of_trajectories']} primitives, "
        f"{sum(abs(float(item['trajectory_length'])) < 1.0e-7 for item in lattice['primitives'])} spins, "
        f"{metadata['num_of_headings']} headings, "
        f"radius={metadata['turning_radius']}m, "
        f"resolution={metadata['grid_resolution']}m"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
