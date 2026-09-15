#!/usr/bin/env python3
"""Isolated typed floor-transition HTTP interlock smoke test."""

from __future__ import annotations

import argparse
import fcntl
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
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionServer
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


def request_bytes(port: int, path: str) -> tuple[int, str, bytes]:
    request = urllib.request.Request(
        f"http://127.0.0.1:{port}{path}",
        method="GET",
    )
    try:
        with urllib.request.urlopen(request, timeout=5.0) as response:
            return response.status, response.headers.get_content_type(), response.read()
    except urllib.error.HTTPError as error:
        return error.code, error.headers.get_content_type(), error.read()


def publish_repeated(node: rclpy.node.Node, publisher: object, message: object) -> None:
    for _ in range(3):
        publisher.publish(message)
        rclpy.spin_once(node, timeout_sec=0.1)
        time.sleep(0.1)


def snapshot_file_tree(root: Path) -> dict[str, tuple[bytes, int]]:
    return {
        str(path.relative_to(root)): (path.read_bytes(), path.stat().st_mtime_ns)
        for path in sorted(root.rglob("*"))
        if path.is_file()
    }


def snapshot_file_contents(root: Path) -> dict[str, bytes]:
    return {
        str(path.relative_to(root)): path.read_bytes()
        for path in sorted(root.rglob("*"))
        if path.is_file()
    }


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


def terminate_test_process_group(pid: int) -> None:
    try:
        os.killpg(pid, signal.SIGTERM)
    except ProcessLookupError:
        return
    deadline = time.monotonic() + 2.0
    while time.monotonic() < deadline:
        try:
            os.killpg(pid, 0)
        except ProcessLookupError:
            return
        time.sleep(0.05)
    try:
        os.killpg(pid, signal.SIGKILL)
    except ProcessLookupError:
        return


def assert_v2_pending_activation_tamper_fails_closed(
    node: Path,
    port: int,
    environment: dict[str, str],
    root: Path,
) -> None:
    fixture_root = root / "v2_pending_activation_recovery"
    map_id = "map_v2_pending_recovery"
    map_root = write_map_manifest(
        fixture_root,
        building_id="B_recovery",
        floor_id="F1",
        map_id=map_id,
        active=False,
    )
    write_complete_map_assets(map_root)
    maps_root = fixture_root / "maps"
    runtime_context_path = fixture_root / "runtime_context.json"
    last_navigation_map_path = fixture_root / "last_navigation_map.json"
    first_log_path = fixture_root / "stamp_api.log"
    second_log_path = fixture_root / "recovery_api.log"

    def command(api_port: int) -> list[str]:
        return [
            str(node),
            "--ros-args",
            "-p",
            f"port:={api_port}",
            "-p",
            f"maps_root:={maps_root}",
            "-p",
            f"runtime_maps_dir:={fixture_root / 'runtime_maps'}",
            "-p",
            f"runtime_map_context_file:={runtime_context_path}",
            "-p",
            f"last_navigation_map_file:={last_navigation_map_path}",
            "-p",
            "navigation_resume_command:=/bin/true",
            "-p",
            "floor_runtime_negative_interlock_enabled:=false",
        ]

    with first_log_path.open("w", encoding="utf-8") as first_log:
        stamp_process = subprocess.Popen(
            command(port),
            stdout=first_log,
            stderr=subprocess.STDOUT,
            text=True,
            env=environment,
            start_new_session=True,
        )
        try:
            wait_for_port(port, stamp_process, 15.0)
        finally:
            terminate_test_process_group(stamp_process.pid)
            stamp_process.wait(timeout=5.0)

    manifest_path = map_root / "manifest.json"
    frozen_manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    assert frozen_manifest["schema"] == "njrh.map_manifest.v2", frozen_manifest
    assert int(frozen_manifest["asset_epoch"]) > 0, frozen_manifest
    assert str(frozen_manifest["asset_digest"]).startswith("sha256:"), frozen_manifest
    frozen_manifest_bytes = manifest_path.read_bytes()

    activation_journal = maps_root / ".map_activation_transaction.v1"
    activation_journal.write_text(
        "schema=njrh.map_activation_transaction.v1\n"
        "building_id=B_recovery\n"
        "floor_id=F1\n"
        f"map_id={map_id}\n",
        encoding="utf-8",
    )
    nav_map_pgm = map_root / "nav" / "nav_map.pgm"
    original_pgm = nav_map_pgm.read_bytes()
    nav_map_pgm.write_bytes(original_pgm[:-1] + bytes([original_pgm[-1] ^ 0x01]))

    with second_log_path.open("w", encoding="utf-8") as second_log:
        recovery_process = subprocess.Popen(
            command(port + 1),
            stdout=second_log,
            stderr=subprocess.STDOUT,
            text=True,
            env=environment,
            start_new_session=True,
        )
        try:
            wait_for_port(port + 1, recovery_process, 15.0)
            status, recovery_status = request_json(
                port + 1, "GET", "/api/v1/status"
            )
            assert status == 200, recovery_status
            assert recovery_status["keepout_integrity_degraded"] is True, (
                recovery_status,
                second_log_path.read_text(encoding="utf-8"),
            )
            assert activation_journal.is_file()
            assert (
                maps_root / ".map_asset_integrity_degraded.v1"
            ).is_file()
            assert not (
                maps_root / "B_recovery" / "F1" / "current"
            ).exists()
            assert manifest_path.read_bytes() == frozen_manifest_bytes
        finally:
            terminate_test_process_group(recovery_process.pid)
            recovery_process.wait(timeout=5.0)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--node", required=True, type=Path)
    parser.add_argument("--port", type=int, default=18082)
    args = parser.parse_args()
    if not args.node.is_file():
        raise FileNotFoundError(args.node)

    environment = os.environ.copy()
    environment["ROS_DOMAIN_ID"] = "229"
    environment["RMW_IMPLEMENTATION"] = "rmw_fastrtps_cpp"
    environment["ROS_LOCALHOST_ONLY"] = "1"
    os.environ.update(
        {
            "ROS_DOMAIN_ID": environment["ROS_DOMAIN_ID"],
            "RMW_IMPLEMENTATION": environment["RMW_IMPLEMENTATION"],
            "ROS_LOCALHOST_ONLY": environment["ROS_LOCALHOST_ONLY"],
        }
    )

    with tempfile.TemporaryDirectory(prefix="njrh_floor_runtime_http_") as temporary:
        root = Path(temporary)
        assert_v2_pending_activation_tamper_fails_closed(
            args.node,
            args.port + 20,
            environment,
            root,
        )
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
        write_complete_map_assets(inactive_same_floor_map_root)
        exact_preview_png = b"\x89PNG\r\n\x1a\nexact-map-a"
        newer_preview_png = b"\x89PNG\r\n\x1a\nnewer-map-b"
        (inactive_same_floor_map_root / "localizer" / "nav_map.png").write_bytes(
            exact_preview_png
        )
        (target_map_root / "localizer" / "nav_map.png").write_bytes(
            newer_preview_png
        )
        runtime_maps_root = root / "runtime_maps"
        runtime_maps_root.mkdir()
        (runtime_maps_root / "newer-map-b.png").write_bytes(newer_preview_png)
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
        navigation_resume_trace_path = root / "navigation_resume_trace.txt"
        last_navigation_map_path = root / "last_navigation_map.json"
        navigation_resume_pid: int | None = None
        navigation_resume_command = root / "fake_navigation_resume.sh"
        navigation_resume_command.write_text(
            "#!/usr/bin/env bash\n"
            "set -eu\n"
            f"printf '%s\\n' \"$1\" \"$2\" \"$NJRH_MAP_ID\" "
            f"\"$NJRH_MAP_CONTEXT_BUILDING_ID\" \"$NJRH_MAP_CONTEXT_FLOOR_ID\" "
            f"\"$NJRH_NAVIGATION_START_SOURCE\" "
            f"> '{navigation_resume_trace_path.as_posix()}'\n"
            "while true; do sleep 1; done\n",
            encoding="utf-8",
        )
        navigation_resume_command.chmod(0o755)
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
            f"last_navigation_map_file:={last_navigation_map_path}",
            "-p",
            f"navigation_resume_command:={navigation_resume_command}",
            "-p",
            f"navigation_resume_log_file:={root / 'navigation_resume.log'}",
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
                assert not activation_journal.exists(), log_path.read_text(
                    encoding="utf-8"
                )
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

                status, content_type, preview = request_bytes(
                    args.port,
                    "/api/v1/mapping/2d/map"
                    "?source=saved"
                    f"&map_id={inactive_same_floor_map_id}"
                    "&building_id=B1"
                    "&floor_id=F1",
                )
                assert status == 200, preview
                assert content_type == "image/png", content_type
                assert preview == exact_preview_png, (
                    "exact saved-map preview returned a different, newer map"
                )

                status, response = request_json(
                    args.port,
                    "GET",
                    "/api/v1/mapping/2d/map"
                    "?source=saved"
                    "&map_id=map_missing_preview"
                    "&building_id=B1"
                    "&floor_id=F1",
                )
                assert status == 404, response
                assert response["error"] == "map_id not found: map_missing_preview"

                status, response = request_json(
                    args.port,
                    "GET",
                    "/api/v1/mapping/2d/map"
                    "?source=saved"
                    f"&map_id={inactive_same_floor_map_id}"
                    "&building_id=B2"
                    "&floor_id=F1",
                )
                assert status == 400, response
                assert (
                    response["error"]
                    == "map_id does not belong to requested building"
                )

                status, response = request_json(
                    args.port,
                    "GET",
                    "/api/v1/mapping/2d/map"
                    "?source=saved"
                    f"&map_id={inactive_same_floor_map_id}"
                    "&building_id=B1"
                    "&floor_id=F2",
                )
                assert status == 400, response
                assert (
                    response["error"] == "map_id does not belong to requested floor"
                )

                status, response = request_json(
                    args.port,
                    "GET",
                    "/api/v1/mapping/2d/map"
                    "?source=saved"
                    f"&map_id={deletable_map_id}"
                    "&building_id=B1"
                    "&floor_id=F1",
                )
                assert status == 404, response
                assert response["error"] == (
                    "saved 2D PNG map is unavailable for map_id: "
                    f"{deletable_map_id}"
                )

                invalid = LocalizationHealth()
                invalid.transition_active = True
                invalid.runtime_context_valid = False
                invalid.detail = "test transition active"
                publish_repeated(probe, health_pub, invalid)

                blocked_endpoints = (
                    ("POST", "/api/v1/navigation/start"),
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

                target_manifest = json.loads(
                    (target_map_root / "manifest.json").read_text(encoding="utf-8")
                )
                target_asset_epoch = int(target_manifest["asset_epoch"])
                target_asset_digest = str(target_manifest["asset_digest"])
                expected_switch_floor_call = (
                    "B1",
                    "F2",
                    target_map_id,
                    target_asset_epoch,
                    target_asset_digest,
                    False,
                )
                switch_floor_calls: list[tuple[str, str, str, int, str, bool]] = []
                switch_response_identity_valid = False
                switch_inject_source_drift = False
                switch_commit_lock_observations: list[bool] = []
                target_manifest_path = target_map_root / "manifest.json"
                target_manifest_original = target_manifest_path.read_bytes()

                def switch_floor_callback(
                    request: SwitchFloor.Request,
                    response: SwitchFloor.Response,
                ) -> SwitchFloor.Response:
                    switch_floor_calls.append(
                        (
                            request.building_id,
                            request.floor_id,
                            request.map_id,
                            request.expected_asset_epoch,
                            request.expected_asset_digest,
                            request.resume_navigation,
                        )
                    )
                    assert request.building_id == "B1"
                    assert request.floor_id == "F2"
                    assert request.map_id == target_map_id
                    assert request.expected_asset_epoch == target_asset_epoch
                    assert request.expected_asset_digest == target_asset_digest
                    assert request.resume_navigation is False
                    response.success = True
                    response.code = "OK"
                    response.message = "isolated offline floor selection complete"
                    response.selected_building_id = "B1"
                    response.selected_floor_id = "F2"
                    response.selected_map_id = (
                        target_map_id
                        if switch_response_identity_valid
                        else "map_wrong_response_identity"
                    )
                    response.asset_epoch = target_asset_epoch
                    response.asset_digest = target_asset_digest
                    response.nav_map_yaml = str(
                        target_map_root / "nav" / "nav_map.yaml"
                    )
                    response.localizer_map_png = str(
                        target_map_root / "localizer" / "nav_map.png"
                    )
                    response.localizer_params_yaml = str(
                        target_map_root / "localizer" / "nav_map.yaml"
                    )
                    with (
                        root / "maps" / ".map_asset_identity_commit.lock"
                    ).open("r+b") as commit_lock:
                        try:
                            fcntl.flock(
                                commit_lock.fileno(),
                                fcntl.LOCK_EX | fcntl.LOCK_NB,
                            )
                        except BlockingIOError:
                            switch_commit_lock_observations.append(True)
                        else:
                            switch_commit_lock_observations.append(False)
                            fcntl.flock(commit_lock.fileno(), fcntl.LOCK_UN)
                    if switch_inject_source_drift:
                        drifted_manifest = json.loads(
                            target_manifest_original.decode("utf-8")
                        )
                        drifted_manifest["display_name"] += "_source_drift"
                        target_manifest_path.write_text(
                            json.dumps(drifted_manifest, indent=2) + "\n",
                            encoding="utf-8",
                        )
                        assert (
                            target_manifest_path.read_bytes()
                            != target_manifest_original
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
                navigation_action_server: ActionServer | None = None
                try:
                    map_tree_before_identity_mismatch = snapshot_file_tree(
                        root / "maps"
                    )
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
                    assert status == 409, response
                    assert (
                        response["code"] == "FLOOR_SELECTION_IDENTITY_UNPROVEN"
                    ), response
                    assert (
                        snapshot_file_tree(root / "maps")
                        == map_tree_before_identity_mismatch
                    )
                    switch_response_identity_valid = True

                    switch_inject_source_drift = True
                    map_contents_before_source_drift = snapshot_file_contents(
                        root / "maps"
                    )
                    runtime_context_before_source_drift = (
                        runtime_context_path.read_bytes()
                    )
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
                    target_manifest_path.write_bytes(target_manifest_original)
                    switch_inject_source_drift = False
                    assert status == 409, response
                    assert (
                        response["code"] == "FLOOR_SELECTION_SOURCE_DRIFT"
                    ), response
                    assert (
                        snapshot_file_contents(root / "maps")
                        == map_contents_before_source_drift
                    )
                    assert (
                        runtime_context_path.read_bytes()
                        == runtime_context_before_source_drift
                    )

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
                    assert status == 200, response
                    assert response["ok"] is True, response
                    assert response["map_id"] == target_map_id, response
                    assert switch_floor_calls == [expected_switch_floor_call] * 3
                    assert switch_commit_lock_observations == [True, True, True]

                    current_projection_pgm = (
                        root
                        / "maps"
                        / "B1"
                        / "F2"
                        / "current"
                        / "nav"
                        / "nav_map.pgm"
                    )
                    original_projection_pgm = current_projection_pgm.read_bytes()
                    current_projection_pgm.write_bytes(
                        original_projection_pgm[:-1]
                        + bytes([original_projection_pgm[-1] ^ 0x01])
                    )
                    status, drifted_projection_start = request_json(
                        args.port,
                        "POST",
                        "/api/v1/navigation/start",
                        {
                            "building_id": "B1",
                            "floor_id": "F2",
                            "map_id": target_map_id,
                        },
                    )
                    assert status == 409, drifted_projection_start
                    assert (
                        drifted_projection_start["code"]
                        == "NAVIGATION_MAP_ASSETS_INVALID"
                    ), drifted_projection_start
                    assert not navigation_resume_trace_path.exists()
                    current_projection_pgm.write_bytes(original_projection_pgm)

                    status, wrong_map_start = request_json(
                        args.port,
                        "POST",
                        "/api/v1/navigation/start",
                        {
                            "building_id": "B1",
                            "floor_id": "F1",
                            "map_id": inactive_same_floor_map_id,
                        },
                    )
                    assert status == 409, wrong_map_start
                    assert (
                        wrong_map_start["code"] == "NAVIGATION_MAP_NOT_SELECTED"
                    ), wrong_map_start
                    assert not navigation_resume_trace_path.exists()

                    status, start_response = request_json(
                        args.port,
                        "POST",
                        "/api/v1/navigation/start",
                        {
                            "building_id": "B1",
                            "floor_id": "F2",
                            "map_id": target_map_id,
                        },
                    )
                    assert status == 202, start_response
                    assert start_response["ok"] is True, start_response
                    assert start_response["state"] == "navigation_resume_starting"
                    assert start_response["building_id"] == "B1"
                    assert start_response["floor_id"] == "F2"
                    assert start_response["map_id"] == target_map_id
                    navigation_resume_pid = int(start_response["pid"])
                    isolated_last_map = json.loads(
                        last_navigation_map_path.read_text(encoding="utf-8")
                    )
                    assert isolated_last_map["building_id"] == "B1"
                    assert isolated_last_map["floor_id"] == "F2"
                    assert isolated_last_map["map_id"] == target_map_id

                    resume_trace_deadline = time.monotonic() + 2.0
                    while (
                        time.monotonic() < resume_trace_deadline
                        and not navigation_resume_trace_path.exists()
                    ):
                        time.sleep(0.05)
                    assert navigation_resume_trace_path.is_file()
                    assert navigation_resume_trace_path.read_text(
                        encoding="utf-8"
                    ).splitlines() == [
                        "B1",
                        "F2",
                        target_map_id,
                        "B1",
                        "F2",
                        "api_resume",
                    ]

                    status, repeated_start = request_json(
                        args.port,
                        "POST",
                        "/api/v1/navigation/start",
                        {
                            "building_id": "B1",
                            "floor_id": "F2",
                            "map_id": target_map_id,
                        },
                    )
                    assert status == 202, repeated_start
                    assert repeated_start["ok"] is True, repeated_start
                    assert (
                        repeated_start["state"]
                        == "navigation_runtime_starting_reused"
                    ), repeated_start
                    assert repeated_start["reused"] is True, repeated_start
                    assert navigation_resume_trace_path.read_text(
                        encoding="utf-8"
                    ).splitlines() == [
                        "B1",
                        "F2",
                        target_map_id,
                        "B1",
                        "F2",
                        "api_resume",
                    ]

                    stale_navigation_resume_pid = navigation_resume_pid
                    terminate_test_process_group(stale_navigation_resume_pid)
                    stale_process_deadline = time.monotonic() + 2.0
                    while time.monotonic() < stale_process_deadline:
                        try:
                            os.kill(stale_navigation_resume_pid, 0)
                        except ProcessLookupError:
                            break
                        time.sleep(0.05)

                    status, restarted_after_stale_starting = request_json(
                        args.port,
                        "POST",
                        "/api/v1/navigation/start",
                        {
                            "building_id": "B1",
                            "floor_id": "F2",
                            "map_id": target_map_id,
                        },
                    )
                    assert status == 202, restarted_after_stale_starting
                    assert (
                        restarted_after_stale_starting["state"]
                        == "navigation_resume_starting"
                    ), restarted_after_stale_starting
                    navigation_resume_pid = int(
                        restarted_after_stale_starting["pid"]
                    )
                    assert navigation_resume_pid > 0
                    assert (
                        restarted_after_stale_starting.get("reused", False)
                        is False
                    )

                    runtime_context_path.write_text(
                        json.dumps(
                            {
                                "schema": "njrh.runtime_map_context.v1",
                                "state": "ready",
                                "startup_stage": "ready",
                                "confirmed": True,
                                "message": "isolated active navigation context",
                                "map_id": target_map_id,
                                "display_name": target_map_id,
                                "building_id": "B1",
                                "floor_id": "F2",
                                "updated_at": time.time(),
                            }
                        )
                        + "\n",
                        encoding="utf-8",
                    )

                    def execute_navigation_goal(goal_handle: object) -> NavigateToPose.Result:
                        goal_handle.abort()
                        return NavigateToPose.Result()

                    navigation_action_server = ActionServer(
                        probe,
                        NavigateToPose,
                        "/navigate_to_pose",
                        execute_navigation_goal,
                    )
                    navigation_active_observed = False
                    navigation_deadline = time.monotonic() + 5.0
                    while time.monotonic() < navigation_deadline:
                        status, runtime_status = request_json(
                            args.port, "GET", "/api/v1/status"
                        )
                        assert status == 200, runtime_status
                        if runtime_status["navigation"]["active"] is True:
                            navigation_active_observed = True
                            break
                        time.sleep(0.1)
                    assert navigation_active_observed, runtime_status

                    runtime_context_before_noop = runtime_context_path.read_bytes()
                    map_tree_before_noop = snapshot_file_tree(root / "maps")
                    status, same_map_response = request_json(
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
                    assert status == 200, same_map_response
                    assert same_map_response["ok"] is True, same_map_response
                    assert same_map_response["map_id"] == target_map_id, same_map_response
                    assert same_map_response["already_active"] is True, same_map_response
                    assert same_map_response["runtime_unchanged"] is True, same_map_response
                    assert same_map_response["selection_performed"] is False, same_map_response
                    assert runtime_context_path.read_bytes() == runtime_context_before_noop
                    assert snapshot_file_tree(root / "maps") == map_tree_before_noop
                    assert switch_floor_calls == [expected_switch_floor_call] * 3

                    status, other_map_response = request_json(
                        args.port,
                        "POST",
                        "/api/v1/floors/switch",
                        {
                            "building_id": "B1",
                            "floor_id": "F1",
                            "map_id": current_map_id,
                            "resume_navigation": False,
                        },
                    )
                    assert status == 409, other_map_response
                    assert (
                        other_map_response["code"] == "FLOOR_SELECTION_RUNTIME_BUSY"
                    ), other_map_response
                    assert "navigation" in other_map_response["detail"], other_map_response
                    assert runtime_context_path.read_bytes() == runtime_context_before_noop
                    assert snapshot_file_tree(root / "maps") == map_tree_before_noop
                    assert switch_floor_calls == [expected_switch_floor_call] * 3
                finally:
                    spin_stop.set()
                    spin_thread.join(timeout=2.0)
                    assert not spin_thread.is_alive()
                    if navigation_action_server is not None:
                        navigation_action_server.destroy()
                    probe.destroy_service(switch_service)
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
                invalid.transition_active = False
                invalid.runtime_context_valid = False
                invalid.detail = "ABORTED_CONTEXT_INVALID"
                publish_repeated(probe, health_pub, invalid)
                assert_floor_block(
                    args.port,
                    "POST",
                    "/api/v1/navigation/goal",
                    "FLOOR_RUNTIME_CONTEXT_INVALID",
                )
                publish_repeated(probe, health_pub, healthy)
                status, response = request_json(
                    args.port, "POST", "/api/v1/navigation/goal", {}
                )
                assert status == 400, response
                assert response.get("code") != "FLOOR_TRANSITION_BLOCKED", response

                print(
                    json.dumps(
                        {
                            "ok": True,
                            "blocked_endpoint_count": len(blocked_endpoints),
                            "safety_stop_allowed": True,
                            "preflight_blocked_not_latched": True,
                            "terminal_failure_not_a_latch": True,
                            "current_invalid_context_still_blocks_motion": True,
                            "cross_floor_docking_rejected": True,
                            "inactive_map_docking_rejected": True,
                            "active_map_delete_rejected": True,
                            "runtime_bound_map_delete_rejected": True,
                            "symlink_escape_map_delete_rejected": True,
                            "inactive_unbound_map_delete_allowed": True,
                            "offline_floor_selection_completed_without_deadlock": True,
                            "offline_floor_selection_holds_cross_process_asset_lock": True,
                            "offline_floor_selection_rejects_post_preflight_source_drift": True,
                            "navigation_start_rejects_drifted_current_projection": True,
                            "navigation_start_rejects_unselected_map": True,
                            "navigation_start_uses_selected_exact_map": True,
                            "navigation_start_is_idempotent_while_starting": True,
                            "same_runtime_map_selection_is_noop": True,
                            "different_runtime_map_selection_remains_blocked": True,
                            "activation_journal_recovered_on_restart": True,
                            "exact_saved_map_preview_uses_immutable_map_id": True,
                            "exact_saved_map_preview_errors_fail_closed": True,
                        },
                        separators=(",", ":"),
                    )
                )
            finally:
                probe.destroy_node()
                rclpy.shutdown()
                if navigation_resume_pid is not None:
                    terminate_test_process_group(navigation_resume_pid)
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
