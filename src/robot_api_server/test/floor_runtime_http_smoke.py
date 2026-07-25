#!/usr/bin/env python3
"""Isolated typed floor-transition HTTP interlock smoke test."""

from __future__ import annotations

import argparse
import json
import os
import signal
import socket
import subprocess
import tempfile
import threading
import time
import urllib.error
import urllib.request
from pathlib import Path

import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from robot_interfaces.msg import FloorSwitchStatus, LocalizationHealth
from robot_interfaces.srv import SwitchFloor


def write_map_manifest(
    root: Path,
    *,
    building_id: str,
    floor_id: str,
    map_id: str,
    active: bool,
) -> Path:
    map_root = root / "maps" / building_id / floor_id / "maps" / map_id
    map_root.mkdir(parents=True)
    (map_root / "manifest.json").write_text(
        json.dumps(
            {
                "map_id": map_id,
                "display_name": map_id,
                "safe_map_name": "nav_map",
                "building_id": building_id,
                "floor_id": floor_id,
                "created_at": "2026-07-23T00:00:00Z",
                "active": active,
            },
            indent=2,
        )
        + "\n",
        encoding="utf-8",
    )
    return map_root

def write_complete_map_assets(map_root: Path) -> None:
    map_yaml = (
        "image: nav_map.pgm\n"
        "resolution: 0.05\n"
        "origin: [0.0, 0.0, 0.0]\n"
        "negate: 0\n"
        "occupied_thresh: 0.65\n"
        "free_thresh: 0.196\n"
        "mode: trinary\n"
    )
    pgm = b"P5\n2 2\n255\n" + bytes([254, 254, 254, 254])
    for directory in ("nav", "localizer", "filters", "reports"):
        (map_root / directory).mkdir(parents=True, exist_ok=True)
    (map_root / "nav" / "nav_map.yaml").write_text(map_yaml, encoding="utf-8")
    (map_root / "nav" / "nav_map.pgm").write_bytes(pgm)
    (map_root / "localizer" / "nav_map.png").write_bytes(b"png")
    (map_root / "localizer" / "nav_map.yaml").write_text(
        "schema_version: 1\nmap_frame: map\n", encoding="utf-8"
    )
    for mask_name in ("keepout_mask", "speed_mask", "binary_mask"):
        (map_root / "filters" / f"{mask_name}.yaml").write_text(
            map_yaml.replace("nav_map.pgm", f"{mask_name}.pgm"),
            encoding="utf-8",
        )
        (map_root / "filters" / f"{mask_name}.pgm").write_bytes(pgm)
    (map_root / "reports" / "asset_report.json").write_text(
        "{}\n", encoding="utf-8"
    )
    (map_root / "poses.yaml").write_text("poses: []\n", encoding="utf-8")


def wait_for_port(port: int, process: subprocess.Popen[str], timeout_sec: float) -> None:
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        if process.poll() is not None:
            raise RuntimeError(f"API node exited during startup with code {process.returncode}")
        try:
            with socket.create_connection(("127.0.0.1", port), timeout=0.2):
                return
        except OSError:
            time.sleep(0.1)
    raise RuntimeError("timed out waiting for isolated API node")


def request_json(
    port: int, method: str, path: str, payload: dict[str, object] | None = None
) -> tuple[int, dict[str, object]]:
    data = None if payload is None else json.dumps(payload).encode("utf-8")
    request = urllib.request.Request(
        f"http://127.0.0.1:{port}{path}",
        data=data,
        headers={"Content-Type": "application/json"},
        method=method,
    )
    try:
        with urllib.request.urlopen(request, timeout=5.0) as response:
            return response.status, json.loads(response.read().decode("utf-8"))
    except urllib.error.HTTPError as error:
        return error.code, json.loads(error.read().decode("utf-8"))


def publish_repeated(node: rclpy.node.Node, publisher: object, message: object) -> None:
    for _ in range(3):
        publisher.publish(message)
        rclpy.spin_once(node, timeout_sec=0.1)
        time.sleep(0.1)


def assert_floor_block(
    port: int,
    method: str,
    path: str,
    expected_reason: str,
) -> None:
    status, response = request_json(port, method, path, {})
    assert status in (409, 503), (path, status, response)
    assert response["code"] == "FLOOR_TRANSITION_BLOCKED", (path, response)
    assert response["reason_code"] == expected_reason, (path, response)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--node", required=True, type=Path)
    parser.add_argument("--port", type=int, default=18082)
    args = parser.parse_args()
    if not args.node.is_file():
        raise FileNotFoundError(args.node)

    environment = os.environ.copy()
    environment.setdefault("ROS_DOMAIN_ID", "229")
    environment.setdefault("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp")
    os.environ.update(
        {
            "ROS_DOMAIN_ID": environment["ROS_DOMAIN_ID"],
            "RMW_IMPLEMENTATION": environment["RMW_IMPLEMENTATION"],
        }
    )

    with tempfile.TemporaryDirectory(prefix="njrh_floor_runtime_http_") as temporary:
        root = Path(temporary)
        current_map_id = "map_floor_runtime_current"
        inactive_same_floor_map_id = "map_floor_runtime_inactive"
        target_map_id = "map_floor_runtime_other_floor"
        deletable_map_id = "map_floor_runtime_deletable"
        current_map_root = write_map_manifest(
            root,
            building_id="B1",
            floor_id="F1",
            map_id=current_map_id,
            active=True,
        )
        target_map_root = write_map_manifest(
            root,
            building_id="B1",
            floor_id="F2",
            map_id=target_map_id,
            active=True,
        )
        write_complete_map_assets(target_map_root)
        activation_journal = root / "maps" / ".map_activation_transaction.v1"
        activation_journal.write_text(
            "schema=njrh.map_activation_transaction.v1\n"
            "building_id=B1\n"
            "floor_id=F2\n"
            f"map_id={target_map_id}\n",
            encoding="utf-8",
        )
        inactive_same_floor_map_root = write_map_manifest(
            root,
            building_id="B1",
            floor_id="F1",
            map_id=inactive_same_floor_map_id,
            active=False,
        )
        deletable_map_root = write_map_manifest(
            root,
            building_id="B1",
            floor_id="F1",
            map_id=deletable_map_id,
            active=False,
        )
        symlink_escape_building_id = "B_symlink"
        symlink_escape_floor_id = "F9"
        symlink_escape_map_id = "map_symlink_escape"
        outside_building_root = root / "outside_map_assets"
        symlink_escape_map_root = (
            outside_building_root
            / symlink_escape_floor_id
            / "maps"
            / symlink_escape_map_id
        )
        symlink_escape_map_root.mkdir(parents=True)
        (symlink_escape_map_root / "manifest.json").write_text(
            json.dumps(
                {
                    "map_id": symlink_escape_map_id,
                    "display_name": symlink_escape_map_id,
                    "safe_map_name": "nav_map",
                    "building_id": symlink_escape_building_id,
                    "floor_id": symlink_escape_floor_id,
                    "created_at": "2026-07-23T00:00:00Z",
                    "active": False,
                },
                indent=2,
            )
            + "\n",
            encoding="utf-8",
        )
        outside_marker = symlink_escape_map_root / "must_survive.txt"
        outside_marker.write_text("do not delete through ancestor symlink\n", encoding="utf-8")
        symlink_escape_building_root = (
            root / "maps" / symlink_escape_building_id
        )
        runtime_context_path = root / "runtime_context.json"
        runtime_context_path.write_text(
            json.dumps(
                {
                    "schema": "njrh.runtime_map_context.v1",
                    "state": "ready",
                    "startup_stage": "ready",
                    "confirmed": True,
                    "message": "isolated current-floor context",
                    "map_id": current_map_id,
                    "display_name": current_map_id,
                    "building_id": "B1",
                    "floor_id": "F1",
                    "updated_at": time.time(),
                }
            )
            + "\n",
            encoding="utf-8",
        )
        log_path = root / "api.log"
        command = [
            str(args.node),
            "--ros-args",
            "-p",
            f"port:={args.port}",
            "-p",
            f"maps_root:={root / 'maps'}",
            "-p",
            f"runtime_maps_dir:={root / 'runtime_maps'}",
            "-p",
            f"runtime_map_context_file:={runtime_context_path}",
            "-p",
            "floor_runtime_negative_interlock_enabled:=true",
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
            rclpy.init()
            probe = rclpy.create_node("floor_runtime_http_smoke_probe")
            qos = QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )
            status_pub = probe.create_publisher(
                FloorSwitchStatus, "/floor_manager/transition_status", qos
            )
            health_pub = probe.create_publisher(
                LocalizationHealth, "/localization/floor_health", qos
            )
            try:
                wait_for_port(args.port, process, 15.0)
                assert not activation_journal.exists()
                recovered_current_manifest = (
                    root / "maps" / "B1" / "F2" / "current" / "manifest.json"
                )
                assert recovered_current_manifest.is_file()
                recovered_manifest = json.loads(
                    recovered_current_manifest.read_text(encoding="utf-8")
                )
                assert recovered_manifest["map_id"] == target_map_id
                assert recovered_manifest["active"] is True
                recovered_current_manifest_bytes = (
                    recovered_current_manifest.read_bytes()
                )

                invalid = LocalizationHealth()
                invalid.transition_active = True
                invalid.runtime_context_valid = False
                invalid.detail = "test transition active"
                publish_repeated(probe, health_pub, invalid)

                blocked_endpoints = (
                    ("POST", "/api/v1/navigation/goal"),
                    ("POST", "/api/v1/mapping/2d/start"),
                    ("POST", "/api/v1/mapping/2d/save"),
                    ("POST", "/api/v1/maps/delete"),
                    ("POST", "/api/v1/maps/filters/keepout/save"),
                    ("PUT", "/api/v1/elevator-config/draft"),
                    ("POST", "/api/v1/elevator-config/publish"),
                    ("POST", "/api/v1/elevator-config/rollback"),
                    ("POST", "/api/v1/floors/switch"),
                    ("POST", "/api/v1/localization/trigger"),
                    ("POST", "/api/v1/docking/start"),
                    ("POST", "/api/v1/docking/undock"),
                    ("POST", "/api/v1/safety/resume"),
                )
                for method, path in blocked_endpoints:
                    assert_floor_block(
                        args.port, method, path, "FLOOR_TRANSITION_ACTIVE"
                    )

                stop_status, stop_response = request_json(
                    args.port, "POST", "/api/v1/safety/stop", {}
                )
                assert stop_status == 202, stop_response

                healthy = LocalizationHealth()
                healthy.transition_active = False
                healthy.runtime_context_valid = True
                healthy.detail = "LEGACY_CONTEXT_UNSCOPED"
                publish_repeated(probe, health_pub, healthy)

                preflight = FloorSwitchStatus()
                preflight.transaction_id = "tx-preflight"
                preflight.state = "BLOCKED"
                preflight.stage = "LIVE_FLOOR_SWITCH_DISABLED"
                preflight.failure_code = 41
                preflight.detail = "preflight changed no runtime asset"
                publish_repeated(probe, status_pub, preflight)

                status, response = request_json(
                    args.port,
                    "POST",
                    "/api/v1/floors/switch",
                    {"resume_navigation": True},
                )
                assert status == 409, response
                assert response["code"] == "LIVE_FLOOR_SWITCH_DISABLED", response

                active = FloorSwitchStatus()
                active.transaction_id = "tx-live"
                active.state = "LOADING_NAV_MAP"
                active.stage = "LOAD_NAV_MAP"
                active.detail = "test mutation"
                publish_repeated(probe, status_pub, active)
                assert_floor_block(
                    args.port,
                    "POST",
                    "/api/v1/navigation/goal",
                    "FLOOR_TRANSITION_ACTIVE",
                )

                complete = FloorSwitchStatus()
                complete.transaction_id = "tx-live"
                complete.state = "COMPLETE"
                complete.stage = "COMPLETE"
                complete.detail = "target context committed"
                publish_repeated(probe, status_pub, complete)
                status, response = request_json(args.port, "GET", "/api/v1/status")
                assert status == 200, response
                assert response["floor_runtime_interlock"]["blocked"] is False, response

                status, response = request_json(
                    args.port,
                    "POST",
                    "/api/v1/docking/start",
                    {
                        "building_id": "B1",
                        "floor_id": "F1",
                        "map_id": inactive_same_floor_map_id,
                        "dock_id": "dock_inactive_map",
                    },
                )
                assert status == 409, response
                assert response["code"] == "FLOOR_SWITCH_REQUIRED", response
                assert inactive_same_floor_map_root.is_dir()

                status, response = request_json(
                    args.port,
                    "POST",
                    "/api/v1/docking/start",
                    {
                        "building_id": "B1",
                        "floor_id": "F2",
                        "map_id": target_map_id,
                        "dock_id": "dock_other_floor",
                    },
                )
                assert status == 409, response
                assert response["code"] == "FLOOR_SWITCH_REQUIRED", response
                assert target_map_root.is_dir()
                assert recovered_current_manifest.is_file()
                assert (
                    recovered_current_manifest.read_bytes()
                    == recovered_current_manifest_bytes
                )

                symlink_escape_building_root.symlink_to(
                    outside_building_root, target_is_directory=True
                )
                status, response = request_json(
                    args.port,
                    "POST",
                    "/api/v1/maps/delete",
                    {"map_id": current_map_id},
                )
                assert status == 409, response
                assert response["code"] == "ACTIVE_MAP_DELETE_DISABLED", response
                assert current_map_root.is_dir()

                runtime_context_path.write_text(
                    json.dumps(
                        {
                            "schema": "njrh.runtime_map_context.v1",
                            "state": "ready",
                            "startup_stage": "ready",
                            "confirmed": True,
                            "message": "isolated inactive runtime-bound context",
                            "map_id": inactive_same_floor_map_id,
                            "display_name": inactive_same_floor_map_id,
                            "building_id": "B1",
                            "floor_id": "F1",
                            "updated_at": time.time(),
                        }
                    )
                    + "\n",
                    encoding="utf-8",
                )
                status, response = request_json(
                    args.port,
                    "POST",
                    "/api/v1/maps/delete",
                    {"map_id": inactive_same_floor_map_id},
                )
                assert status == 409, response
                assert response["code"] == "ACTIVE_MAP_DELETE_DISABLED", response
                assert inactive_same_floor_map_root.is_dir()

                status, response = request_json(
                    args.port,
                    "POST",
                    "/api/v1/maps/delete",
                    {"map_id": symlink_escape_map_id},
                )
                assert status in (404, 409), response
                if status == 409:
                    assert response["code"] == "UNSAFE_MAP_ASSET_PATH", response
                assert symlink_escape_map_root.is_dir()
                assert outside_marker.is_file()
                # The adversarial catalog entry must keep later global epoch
                # audits fail-closed; remove only the test-created symlink
                # before exercising an otherwise valid floor selection.
                symlink_escape_building_root.unlink()
                assert not symlink_escape_building_root.exists()
                assert outside_marker.is_file()

                status, response = request_json(
                    args.port,
                    "POST",
                    "/api/v1/maps/delete",
                    {"map_id": deletable_map_id},
                )
                assert status == 200, response
                assert response["deleted"] is True, response
                assert not deletable_map_root.exists()

                def switch_floor_callback(
                    request: SwitchFloor.Request,
                    response: SwitchFloor.Response,
                ) -> SwitchFloor.Response:
                    assert request.building_id == "B1"
                    assert request.floor_id == "F2"
                    assert request.resume_navigation is False
                    response.success = True
                    response.message = "isolated offline floor selection complete"
                    response.nav_map_yaml = str(
                        target_map_root / "nav" / "nav_map.yaml"
                    )
                    response.localizer_map_png = str(
                        target_map_root / "localizer" / "nav_map.png"
                    )
                    response.localizer_params_yaml = str(
                        target_map_root / "localizer" / "nav_map.yaml"
                    )
                    return response

                switch_service = probe.create_service(
                    SwitchFloor,
                    "/floor_manager/switch_floor",
                    switch_floor_callback,
                )
                spin_stop = threading.Event()

                def spin_service() -> None:
                    while not spin_stop.is_set():
                        rclpy.spin_once(probe, timeout_sec=0.05)

                spin_thread = threading.Thread(target=spin_service, daemon=True)
                spin_thread.start()
                try:
                    status, response = request_json(
                        args.port,
                        "POST",
                        "/api/v1/floors/switch",
                        {
                            "building_id": "B1",
                            "floor_id": "F2",
                            "map_id": target_map_id,
                            "resume_navigation": False,
                        },
                    )
                finally:
                    spin_stop.set()
                    spin_thread.join(timeout=2.0)
                    probe.destroy_service(switch_service)
                assert status == 200, response
                assert response["ok"] is True, response
                assert response["map_id"] == target_map_id, response
                status, post_activation_status = request_json(
                    args.port, "GET", "/api/v1/status"
                )
                assert status == 200, post_activation_status
                assert (
                    post_activation_status["keepout_integrity_degraded"] is False
                )

                failed = FloorSwitchStatus()
                failed.transaction_id = "tx-failed"
                failed.state = "FAILED_LOCKED"
                failed.stage = "HOLD_AND_LOCK"
                failed.failure_code = 99
                failed.detail = "manual recovery required"
                publish_repeated(probe, status_pub, failed)
                publish_repeated(probe, health_pub, healthy)
                assert_floor_block(
                    args.port,
                    "POST",
                    "/api/v1/safety/resume",
                    "FLOOR_TRANSITION_FAILED_LOCKED",
                )

                print(
                    json.dumps(
                        {
                            "ok": True,
                            "blocked_endpoint_count": len(blocked_endpoints),
                            "safety_stop_allowed": True,
                            "preflight_blocked_not_latched": True,
                            "failed_lock_sticky": True,
                            "cross_floor_docking_rejected": True,
                            "inactive_map_docking_rejected": True,
                            "active_map_delete_rejected": True,
                            "runtime_bound_map_delete_rejected": True,
                            "symlink_escape_map_delete_rejected": True,
                            "inactive_unbound_map_delete_allowed": True,
                            "offline_floor_selection_completed_without_deadlock": True,
                            "activation_journal_recovered_on_restart": True,
                        },
                        separators=(",", ":"),
                    )
                )
            finally:
                probe.destroy_node()
                rclpy.shutdown()
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
