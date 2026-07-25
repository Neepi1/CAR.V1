#!/usr/bin/env python3
"""Isolated HTTP smoke test for elevator configuration releases."""

from __future__ import annotations

import argparse
import json
import os
import signal
import socket
import subprocess
import tempfile
import time
import urllib.error
import urllib.request
from pathlib import Path
from typing import Any


TOKEN = "elevator-config-smoke-token"


def write_map(
    maps_root: Path,
    *,
    building_id: str,
    floor_id: str,
    map_id: str,
    with_legacy_poses: bool,
    active: bool,
    nav_image: str = "nav_map.pgm",
    nav_yaml_suffix: str = "",
) -> Path:
    map_root = maps_root / building_id / floor_id / "maps" / map_id
    nav = map_root / "nav"
    localizer = map_root / "localizer"
    filters = map_root / "filters"
    reports = map_root / "reports"
    for directory in (nav, localizer, filters, reports):
        directory.mkdir(parents=True, exist_ok=True)

    map_yaml = (
        f"image: {nav_image}\n"
        "resolution: 0.050000\n"
        "origin: [-5.000000, -5.000000, 0.000000]\n"
        "negate: 0\n"
        "occupied_thresh: 0.65\n"
        "free_thresh: 0.196\n"
        "mode: trinary\n"
        f"{nav_yaml_suffix}"
    )
    pgm = b"P5\n200 200\n255\n" + bytes([254]) * 40000
    (nav / "nav_map.yaml").write_text(map_yaml, encoding="utf-8")
    (nav / "nav_map.pgm").write_bytes(pgm)
    (localizer / "nav_map.png").write_bytes(b"synthetic-localizer-png")
    (localizer / "nav_map.yaml").write_text(
        "schema_version: 1\nmap_frame: map\n", encoding="utf-8"
    )
    (reports / "asset_report.json").write_text(
        json.dumps({"schema_version": 1, "map_id": map_id}) + "\n",
        encoding="utf-8",
    )
    (filters / "keepout_mask.yaml").write_text(
        map_yaml.replace(f"image: {nav_image}", "image: keepout_mask.pgm"),
        encoding="utf-8",
    )
    (filters / "keepout_mask.pgm").write_bytes(pgm)
    for mask_name in ("speed_mask", "binary_mask"):
        (filters / f"{mask_name}.yaml").write_text(
            map_yaml.replace(f"image: {nav_image}", f"image: {mask_name}.pgm"),
            encoding="utf-8",
        )
        (filters / f"{mask_name}.pgm").write_bytes(pgm)
    (map_root / "manifest.json").write_text(
        json.dumps(
            {
                "map_id": map_id,
                "display_name": map_id,
                "safe_map_name": "nav_map",
                "building_id": building_id,
                "floor_id": floor_id,
                "created_at": "2026-07-24T00:00:00Z",
                "active": active,
            },
            indent=2,
        )
        + "\n",
        encoding="utf-8",
    )
    if with_legacy_poses:
        poses_text = (
            "poses:\n"
            "  - id: delivery_1\n"
            "    type: delivery_point\n"
            "    name: delivery\n"
            "    x: -2.0\n"
            "    y: 0.0\n"
            "    yaw: 0.0\n"
            "  - id: legacy_elevator_cabin\n"
            "    type: elevator_internal\n"
            "    name: hidden cabin point\n"
            "    x: 0.8\n"
            "    y: 0.0\n"
            "    yaw: 3.1415926\n"
        )
        (map_root / "poses.yaml").write_text(poses_text, encoding="utf-8")
        (maps_root / building_id / floor_id / "poses.yaml").write_text(
            poses_text, encoding="utf-8"
        )
    else:
        (map_root / "poses.yaml").write_text("poses: []\n", encoding="utf-8")
    return map_root


def elevator_configuration(
    exit_x: float,
    *,
    f4_map_id: str = "map_f4",
) -> dict[str, Any]:
    def floor(floor_id: str, map_id: str) -> dict[str, Any]:
        return {
            "floor_id": floor_id,
            "map_id": map_id,
            "poses": {
                "hall_call": {"x": -1.0, "y": 0.0, "yaw": 0.0},
                "hall_wait": {"x": -0.8, "y": 0.0, "yaw": 0.0},
                "doorway": {"x": 0.0, "y": 0.0, "yaw": 0.0},
                "cabin": {"x": 0.8, "y": 0.0, "yaw": 3.1415926},
                "exit": {"x": exit_x, "y": 0.0, "yaw": 0.0},
            },
            "threshold": {
                "left": [0.0, -0.6],
                "right": [0.0, 0.6],
                "cabin_reference": [1.0, 0.0],
                "clearance_m": 0.05,
                "jamb_clearance_m": 0.05,
            },
        }

    return {
        "schema_version": 1,
        "building_id": "B3",
        "elevators": [
            {
                "elevator_id": "elevator_1",
                "display_name": "1号电梯",
                "floors": [
                    floor("F3", "map_f3"),
                    floor("F4", f4_map_id),
                ],
            }
        ],
    }


def wait_for_port(port: int, process: subprocess.Popen[str], timeout_sec: float) -> None:
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        if process.poll() is not None:
            raise RuntimeError(
                f"API node exited during startup with code {process.returncode}"
            )
        try:
            with socket.create_connection(("127.0.0.1", port), timeout=0.2):
                return
        except OSError:
            time.sleep(0.1)
    raise RuntimeError("timed out waiting for isolated API node")


def request_json(
    port: int,
    method: str,
    path: str,
    payload: dict[str, Any] | None = None,
    *,
    authenticated: bool = True,
) -> tuple[int, dict[str, Any]]:
    data = None if payload is None else json.dumps(payload).encode("utf-8")
    headers = {"Content-Type": "application/json"}
    if authenticated:
        headers["X-Robot-Token"] = TOKEN
    request = urllib.request.Request(
        f"http://127.0.0.1:{port}{path}",
        data=data,
        headers=headers,
        method=method,
    )
    try:
        with urllib.request.urlopen(request, timeout=10.0) as response:
            return response.status, json.loads(response.read().decode("utf-8"))
    except urllib.error.HTTPError as error:
        return error.code, json.loads(error.read().decode("utf-8"))

def identity_file_snapshot(maps_root: Path) -> dict[str, tuple[bytes, int]]:
    paths = list(maps_root.rglob("manifest.json"))
    registry_root = maps_root / ".map_asset_registry"
    if registry_root.is_dir():
        paths.extend(path for path in registry_root.iterdir() if path.is_file())
    return {
        str(path.relative_to(maps_root)): (path.read_bytes(), path.stat().st_mtime_ns)
        for path in sorted(set(paths))
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--node", required=True, type=Path)
    parser.add_argument("--port", type=int, default=18083)
    parser.add_argument(
        "--ros-domain-id",
        type=int,
        choices=range(200, 233),
        default=230,
        help="Dedicated non-production ROS domain for this isolated smoke test.",
    )
    args = parser.parse_args()
    if not args.node.is_file():
        raise FileNotFoundError(args.node)

    with tempfile.TemporaryDirectory(prefix="njrh_elevator_config_http_") as temporary:
        root = Path(temporary)
        maps_root = root / "maps"
        f3_root = write_map(
            maps_root,
            building_id="B3",
            floor_id="F3",
            map_id="map_f3",
            with_legacy_poses=True,
            active=True,
        )
        write_map(
            maps_root,
            building_id="B3",
            floor_id="F4",
            map_id="map_f4",
            with_legacy_poses=False,
            active=False,
        )
        write_map(
            maps_root,
            building_id="B3",
            floor_id="F4",
            map_id="map_bad_image",
            with_legacy_poses=False,
            active=False,
            nav_image="../filters/keepout_mask.pgm",
        )
        dotdot_root = write_map(
            maps_root,
            building_id="B3",
            floor_id="F4",
            map_id="map_symlink_dotdot",
            with_legacy_poses=False,
            active=False,
            nav_image="subdir/../nav_map.pgm",
        )
        write_map(
            maps_root,
            building_id="B3",
            floor_id="F4",
            map_id="map_nested_image",
            with_legacy_poses=False,
            active=False,
            nav_yaml_suffix="metadata:\n  image: ../../must_not_be_read.pgm\n",
        )
        write_map(
            maps_root,
            building_id="B3",
            floor_id="F4",
            map_id="map_duplicate_image",
            with_legacy_poses=False,
            active=False,
            nav_yaml_suffix="image: duplicate.pgm\n",
        )
        outside_nav = root / "outside_nav"
        (outside_nav / "child").mkdir(parents=True)
        (outside_nav / "nav_map.pgm").write_bytes(
            b"P5\n200 200\n255\n" + bytes([254]) * 40000
        )
        (dotdot_root / "nav" / "subdir").symlink_to(
            outside_nav / "child", target_is_directory=True
        )
        original_poses = (f3_root / "poses.yaml").read_bytes()
        runtime_context_path = root / "runtime_context.json"
        runtime_context_path.write_text(
            json.dumps(
                {
                    "schema": "njrh.runtime_map_context.v1",
                    "state": "ready",
                    "startup_stage": "ready",
                    "confirmed": True,
                    "message": "isolated elevator configuration test context",
                    "map_id": "map_f3",
                    "display_name": "map_f3",
                    "building_id": "B3",
                    "floor_id": "F3",
                    "updated_at": time.time(),
                }
            )
            + "\n",
            encoding="utf-8",
        )
        log_path = root / "api.log"
        environment = os.environ.copy()
        environment["ROS_DOMAIN_ID"] = str(args.ros_domain_id)
        environment["RMW_IMPLEMENTATION"] = "rmw_fastrtps_cpp"
        command = [
            str(args.node),
            "--ros-args",
            "-p",
            f"port:={args.port}",
            "-p",
            f"api_token:={TOKEN}",
            "-p",
            f"maps_root:={maps_root}",
            "-p",
            f"runtime_maps_dir:={root / 'runtime_maps'}",
            "-p",
            f"runtime_map_context_file:={runtime_context_path}",
            "-p",
            "floor_runtime_negative_interlock_enabled:=false",
        ]
        with log_path.open("w", encoding="utf-8") as log:
            process = subprocess.Popen(
                command,
                stdout=log,
                stderr=subprocess.STDOUT,
                text=True,
                env=environment,
                start_new_session=True,
            )
            try:
                wait_for_port(args.port, process, 15.0)

                status, unauthorized = request_json(
                    args.port,
                    "GET",
                    "/api/v1/elevator-config?building_id=B3",
                    authenticated=False,
                )
                assert status == 401, unauthorized

                status, invalid_exact_binding = request_json(
                    args.port,
                    "PUT",
                    "/api/v1/elevator-config/draft",
                    {
                        "configuration": elevator_configuration(
                            -0.6,
                            f4_map_id="map_bad_image",
                        ),
                        "actor_id": "commissioning_app",
                    },
                )
                assert status == 200, invalid_exact_binding
                assert (
                    invalid_exact_binding["valid_for_publish"] is False
                ), invalid_exact_binding
                invalid_draft_revision = invalid_exact_binding["draft_revision"]

                status, invalid_dotdot_binding = request_json(
                    args.port,
                    "PUT",
                    "/api/v1/elevator-config/draft",
                    {
                        "configuration": elevator_configuration(
                            -0.6,
                            f4_map_id="map_symlink_dotdot",
                        ),
                        "expected_draft_revision": invalid_draft_revision,
                        "actor_id": "commissioning_app",
                    },
                )
                assert status == 200, invalid_dotdot_binding
                assert (
                    invalid_dotdot_binding["valid_for_publish"] is False
                ), invalid_dotdot_binding
                invalid_draft_revision = invalid_dotdot_binding["draft_revision"]

                status, nested_image_binding = request_json(
                    args.port,
                    "PUT",
                    "/api/v1/elevator-config/draft",
                    {
                        "configuration": elevator_configuration(
                            -0.6,
                            f4_map_id="map_nested_image",
                        ),
                        "expected_draft_revision": invalid_draft_revision,
                        "actor_id": "commissioning_app",
                    },
                )
                assert status == 200, nested_image_binding
                assert nested_image_binding["valid_for_publish"] is True, nested_image_binding
                invalid_draft_revision = nested_image_binding["draft_revision"]

                status, duplicate_image_binding = request_json(
                    args.port,
                    "PUT",
                    "/api/v1/elevator-config/draft",
                    {
                        "configuration": elevator_configuration(
                            -0.6,
                            f4_map_id="map_duplicate_image",
                        ),
                        "expected_draft_revision": invalid_draft_revision,
                        "actor_id": "commissioning_app",
                    },
                )
                assert status == 200, duplicate_image_binding
                assert (
                    duplicate_image_binding["valid_for_publish"] is False
                ), duplicate_image_binding
                invalid_draft_revision = duplicate_image_binding["draft_revision"]

                status, saved_first = request_json(
                    args.port,
                    "PUT",
                    "/api/v1/elevator-config/draft",
                    {
                        "configuration": elevator_configuration(-0.6),
                        "expected_draft_revision": invalid_draft_revision,
                        "actor_id": "commissioning_app",
                    },
                )
                assert status == 200, saved_first
                assert saved_first["valid_for_publish"] is True, saved_first
                draft_first = saved_first["draft_revision"]

                status, published_first = request_json(
                    args.port,
                    "POST",
                    "/api/v1/elevator-config/publish",
                    {
                        "building_id": "B3",
                        "expected_draft_revision": draft_first,
                        "actor_id": "commissioning_app",
                    },
                )
                assert status == 201, published_first
                assert published_first["asset_published"] is True, published_first
                assert published_first["runtime_applied"] is False, published_first
                release_first = published_first["release_id"]

                identity_before_get = identity_file_snapshot(maps_root)
                status, current = request_json(
                    args.port,
                    "GET",
                    "/api/v1/elevator-config?building_id=B3",
                )
                assert status == 200, current
                assert current["current_release_id"] == release_first, current
                assert current["configuration_source"] == "draft", current
                assert isinstance(
                    current["configuration"]["elevators"], list
                ), current
                for elevator in current["configuration"]["elevators"]:
                    for floor in elevator["floors"]:
                        digest = floor["map_asset_digest"]
                        assert digest.startswith("sha256:"), floor
                        assert len(digest) == 71, floor
                        assert all(
                            character in "0123456789abcdef"
                            for character in digest[7:]
                        ), floor
                assert identity_file_snapshot(maps_root) == identity_before_get

                status, saved_second = request_json(
                    args.port,
                    "PUT",
                    "/api/v1/elevator-config/draft",
                    {
                        "configuration": elevator_configuration(-0.7),
                        "expected_draft_revision": draft_first,
                        "actor_id": "commissioning_app",
                    },
                )
                assert status == 200, saved_second
                draft_second = saved_second["draft_revision"]

                status, published_second = request_json(
                    args.port,
                    "POST",
                    "/api/v1/elevator-config/publish",
                    {
                        "building_id": "B3",
                        "expected_draft_revision": draft_second,
                        "expected_release_id": release_first,
                    },
                )
                assert status == 201, published_second
                release_second = published_second["release_id"]

                status, rolled_back = request_json(
                    args.port,
                    "POST",
                    "/api/v1/elevator-config/rollback",
                    {
                        "building_id": "B3",
                        "release_id": release_first,
                        "expected_release_id": release_second,
                    },
                )
                assert status == 201, rolled_back
                assert rolled_back["code"] == "CONFIGURATION_ROLLED_BACK"
                assert rolled_back["rollback_of"] == release_first
                assert rolled_back["release_id"] not in (
                    release_first,
                    release_second,
                )

                status, keepout_locked = request_json(
                    args.port,
                    "POST",
                    "/api/v1/maps/filters/keepout/save",
                    {
                        "building_id": "B3",
                        "floor_id": "F3",
                        "map_id": "map_f3",
                        "reload_filter": False,
                        "keepout_lines": [],
                        "keepout_polygons": [],
                    },
                )
                assert status == 409, keepout_locked
                assert (
                    keepout_locked["error"]["code"]
                    == "ELEVATOR_CONFIG_MAP_IN_USE"
                ), keepout_locked
                assert keepout_locked["release_id"] == rolled_back["release_id"]

                status, listed = request_json(
                    args.port,
                    "GET",
                    "/api/v1/maps/poses?building_id=B3&floor_id=F3&map_id=map_f3",
                )
                assert status == 200, listed
                assert [pose["pose_id"] for pose in listed["poses"]] == ["delivery_1"]

                status, semantic = request_json(
                    args.port,
                    "GET",
                    "/api/v1/maps/semantic_layer?building_id=B3&floor_id=F3&map_id=map_f3",
                )
                assert status == 200, semantic
                assert [pose["pose_id"] for pose in semantic["poses"]] == ["delivery_1"]

                status, reserved_write = request_json(
                    args.port,
                    "POST",
                    "/api/v1/maps/poses",
                    {
                        "building_id": "B3",
                        "floor_id": "F3",
                        "map_id": "map_f3",
                        "pose_id": "eip_manual_cabin",
                        "type": "elevator_internal",
                        "x": 0.8,
                        "y": 0.0,
                        "yaw": 0.0,
                    },
                )
                assert status == 400, reserved_write

                status, reserved_update = request_json(
                    args.port,
                    "PUT",
                    "/api/v1/maps/poses/legacy_elevator_cabin",
                    {
                        "building_id": "B3",
                        "floor_id": "F3",
                        "map_id": "map_f3",
                        "type": "delivery_point",
                        "name": "attempted downgrade",
                        "x": 0.8,
                        "y": 0.0,
                        "yaw": 0.0,
                    },
                )
                assert status == 400, reserved_update

                status, reserved_delete = request_json(
                    args.port,
                    "DELETE",
                    "/api/v1/maps/poses/legacy_elevator_cabin"
                    "?building_id=B3&floor_id=F3&map_id=map_f3",
                )
                assert status == 400, reserved_delete

                status, precheck = request_json(
                    args.port,
                    "GET",
                    "/api/v1/navigation/pre_goal_check"
                    "?building_id=B3&floor_id=F3&map_id=map_f3"
                    "&pose_id=legacy_elevator_cabin",
                )
                assert status == 200, precheck
                assert (
                    precheck["pose_resolution"]["status"]
                    == "elevator_internal_pose_requires_mission"
                ), precheck

                status, goal = request_json(
                    args.port,
                    "POST",
                    "/api/v1/navigation/goal",
                    {
                        "building_id": "B3",
                        "floor_id": "F3",
                        "pose_id": "legacy_elevator_cabin",
                    },
                )
                assert status == 409, goal
                assert (
                    goal["code"] == "ELEVATOR_INTERNAL_POSE_REQUIRES_MISSION"
                ), goal

                status, internal_dock = request_json(
                    args.port,
                    "POST",
                    "/api/v1/docking/start",
                    {
                        "building_id": "B3",
                        "floor_id": "F3",
                        "map_id": "map_f3",
                        "dock_id": "legacy_elevator_cabin",
                    },
                )
                assert status == 409, internal_dock
                assert (
                    internal_dock["code"]
                    == "ELEVATOR_INTERNAL_POSE_REQUIRES_MISSION"
                ), internal_dock

                status, internal_predock = request_json(
                    args.port,
                    "POST",
                    "/api/v1/docking/start",
                    {
                        "building_id": "B3",
                        "floor_id": "F3",
                        "map_id": "map_f3",
                        "dock_id": "delivery_1",
                        "predock_pose_id": "legacy_elevator_cabin",
                    },
                )
                assert status == 409, internal_predock
                assert (
                    internal_predock["code"]
                    == "ELEVATOR_INTERNAL_POSE_REQUIRES_MISSION"
                ), internal_predock

                assert (f3_root / "poses.yaml").read_bytes() == original_poses
                building_root = maps_root / "B3"
                assert (building_root / "elevators.yaml").is_file()
                assert (building_root / "elevator_internal_poses.yaml").is_file()
                assert (building_root / "elevators.yaml").is_symlink()
                assert (
                    building_root / "elevator_internal_poses.yaml"
                ).is_symlink()
                config_root = building_root / ".elevator_config"
                assert (config_root / "current").is_symlink()
                assert (config_root / "current.json").is_symlink()
                assert not (maps_root / "B3" / "F4" / "poses.yaml").exists()

                status, replaced = request_json(
                    args.port,
                    "PUT",
                    "/api/v1/maps/poses/batch",
                    {
                        "building_id": "B3",
                        "floor_id": "F3",
                        "map_id": "map_f3",
                        "poses": [
                            {
                                "pose_id": "delivery_2",
                                "type": "delivery_point",
                                "name": "replacement delivery",
                                "x": -2.5,
                                "y": 0.0,
                                "yaw": 0.0,
                            }
                        ],
                    },
                )
                assert status == 200, replaced
                assert replaced["count"] == 1, replaced
                persisted = (f3_root / "poses.yaml").read_text(encoding="utf-8")
                assert "delivery_2" in persisted
                assert "legacy_elevator_cabin" in persisted
                assert "elevator_internal" in persisted

                status, referenced_map_delete = request_json(
                    args.port,
                    "POST",
                    "/api/v1/maps/delete",
                    {"map_id": "map_f4"},
                )
                assert status == 409, referenced_map_delete
                assert (
                    referenced_map_delete["code"]
                    == "ELEVATOR_CONFIG_MAP_IN_USE"
                ), referenced_map_delete
                assert (
                    maps_root
                    / "B3"
                    / "F4"
                    / "maps"
                    / "map_f4"
                ).is_dir()

                print(
                    json.dumps(
                        {
                            "ok": True,
                            "release_count": 3,
                            "runtime_applied": False,
                            "ordinary_pose_isolation": True,
                            "navigation_internal_pose_rejected": True,
                            "docking_internal_pose_rejected": True,
                            "referenced_map_delete_rejected": True,
                            "bound_keepout_mutation_rejected": True,
                            "ros_domain_id": args.ros_domain_id,
                        },
                        separators=(",", ":"),
                    )
                )
            finally:
                if process.poll() is None:
                    os.killpg(process.pid, signal.SIGTERM)
                    try:
                        process.wait(timeout=5.0)
                    except subprocess.TimeoutExpired:
                        os.killpg(process.pid, signal.SIGKILL)
                        process.wait(timeout=5.0)
        if process.returncode not in (0, -signal.SIGTERM):
            raise RuntimeError(
                f"isolated API node exited with code {process.returncode}: "
                f"{log_path.read_text(encoding='utf-8')}"
            )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
