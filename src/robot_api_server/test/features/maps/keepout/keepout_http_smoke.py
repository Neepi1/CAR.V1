#!/usr/bin/env python3
"""Isolated HTTP smoke test for the keepout replace transaction."""

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


def write_fixture(root: Path) -> tuple[Path, str]:
    map_id = "map_keepout_http_smoke"
    map_root = root / "maps" / "B1" / "F1" / "maps" / map_id
    nav_dir = map_root / "nav"
    localizer_dir = map_root / "localizer"
    filter_dir = map_root / "filters"
    reports_dir = map_root / "reports"
    nav_dir.mkdir(parents=True)
    localizer_dir.mkdir(parents=True)
    filter_dir.mkdir(parents=True)
    reports_dir.mkdir(parents=True)

    yaml_text = (
        "image: nav_map.pgm\n"
        "resolution: 0.500000\n"
        "origin: [1.000000, 2.000000, 0.300000]\n"
        "negate: 0\n"
        "occupied_thresh: 0.65\n"
        "free_thresh: 0.196\n"
        "mode: trinary\n"
    )
    (nav_dir / "nav_map.yaml").write_text(yaml_text, encoding="utf-8")
    (nav_dir / "nav_map.pgm").write_bytes(b"P5\n20 20\n255\n" + bytes([254]) * 400)
    (localizer_dir / "nav_map.png").write_bytes(b"synthetic-png")
    (localizer_dir / "nav_map.yaml").write_text(
        "schema_version: 1\nmap_frame: map\n", encoding="utf-8"
    )
    (filter_dir / "keepout_mask.yaml").write_text(
        yaml_text.replace("image: nav_map.pgm", "image: keepout_mask.pgm"),
        encoding="utf-8",
    )
    (filter_dir / "keepout_mask.pgm").write_bytes(
        b"P5\n20 20\n255\n" + bytes([254]) * 400
    )
    for mask_name in ("speed_mask", "binary_mask"):
        (filter_dir / f"{mask_name}.yaml").write_text(
            yaml_text.replace("image: nav_map.pgm", f"image: {mask_name}.pgm"),
            encoding="utf-8",
        )
        (filter_dir / f"{mask_name}.pgm").write_bytes(
            b"P5\n20 20\n255\n" + bytes([254]) * 400
        )
    (reports_dir / "asset_report.json").write_text("{}\n", encoding="utf-8")
    (map_root / "poses.yaml").write_text("poses: []\n", encoding="utf-8")
    manifest = {
        "map_id": map_id,
        "display_name": "keepout_http_smoke",
        "safe_map_name": "nav_map",
        "building_id": "B1",
        "floor_id": "F1",
        "created_at": "2026-07-23T00:00:00Z",
        "active": True,
    }
    (map_root / "manifest.json").write_text(
        json.dumps(manifest, indent=2) + "\n", encoding="utf-8"
    )
    return map_root, map_id


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

def stop_process(process: subprocess.Popen[str]) -> None:
    if process.poll() is not None:
        return
    os.killpg(process.pid, signal.SIGTERM)
    try:
        process.wait(timeout=5.0)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGKILL)
        process.wait(timeout=5.0)


def post_json(port: int, payload: dict[str, object]) -> tuple[int, dict[str, object]]:
    request = urllib.request.Request(
        f"http://127.0.0.1:{port}/api/v1/maps/filters/keepout/save",
        data=json.dumps(payload).encode("utf-8"),
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    try:
        with urllib.request.urlopen(request, timeout=10.0) as response:
            return response.status, json.loads(response.read().decode("utf-8"))
    except urllib.error.HTTPError as error:
        return error.code, json.loads(error.read().decode("utf-8"))

def get_json(port: int, path: str) -> tuple[int, dict[str, object]]:
    request = urllib.request.Request(
        f"http://127.0.0.1:{port}{path}",
        method="GET",
    )
    with urllib.request.urlopen(request, timeout=10.0) as response:
        return response.status, json.loads(response.read().decode("utf-8"))


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--node", required=True, type=Path)
    parser.add_argument("--port", type=int, default=18081)
    args = parser.parse_args()
    if not args.node.is_file():
        raise FileNotFoundError(args.node)

    with tempfile.TemporaryDirectory(prefix="njrh_keepout_http_") as temporary:
        root = Path(temporary)
        map_root, map_id = write_fixture(root)
        log_path = root / "api.log"
        environment = os.environ.copy()
        environment.setdefault("ROS_DOMAIN_ID", "228")
        environment.setdefault("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp")
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
            f"runtime_map_context_file:={root / 'runtime_context.json'}",
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
                base = {
                    "building_id": "B1",
                    "floor_id": "F1",
                    "map_id": map_id,
                    "reload_filter": True,
                }
                status, saved = post_json(
                    args.port,
                    {
                        **base,
                        "keepout_lines": [
                            {
                                "id": "line_1",
                                "name": "smoke line",
                                "width_m": 0.6,
                                "points": [
                                    {"x": 2.0, "y": 3.0},
                                    {"x": 5.0, "y": 4.0},
                                ],
                            }
                        ],
                        "keepout_polygons": [],
                    },
                )
                assert status == 200, saved
                assert saved["persisted"] is True, saved
                assert saved["runtime_selected"] is False, saved
                assert saved["effective_on_next_activation"] is True, saved
                assert saved["outcome"] == "SAVED_DEFERRED_INACTIVE", saved
                assert saved["mask"]["active_cells"] > 0, saved
                assert (map_root / "filters" / "keepout_semantic_layer.json").is_file()
                assert (
                    root
                    / "maps"
                    / "B1"
                    / "F1"
                    / "current"
                    / "filters"
                    / "keepout_mask.pgm"
                ).is_file()

                status, cleared = post_json(
                    args.port,
                    {**base, "keepout_lines": [], "keepout_polygons": []},
                )
                assert status == 200, cleared
                assert cleared["persisted"] is True, cleared
                assert cleared["mask"]["active_cells"] == 0, cleared

                repair_payload = {
                    **base,
                    "keepout_lines": [
                        {
                            "id": "repair_line",
                            "name": "identity repair regression",
                            "width_m": 0.5,
                            "points": [
                                {"x": 2.0, "y": 3.0},
                                {"x": 4.0, "y": 3.0},
                            ],
                        }
                    ],
                    "keepout_polygons": [],
                }
                status, prepared_repair = post_json(args.port, repair_payload)
                assert status == 200, prepared_repair
                assert prepared_repair["changed"] is True, prepared_repair
                non_keepout_digest = prepared_repair["non_keepout_asset_digest"]
                integrity_marker = (
                    root / "maps" / ".map_asset_integrity_degraded.v1"
                )
                integrity_marker.write_text(
                    "schema=njrh.map_asset_integrity_degraded.v1\n"
                    "map_id=map_other\n"
                    f"non_keepout_asset_digest={non_keepout_digest}\n"
                    "reason_digest=0000000000000000\n",
                    encoding="utf-8",
                )
                status, wrong_map_repair = post_json(args.port, repair_payload)
                assert status == 503, wrong_map_repair
                assert integrity_marker.is_file()
                integrity_marker.write_text(
                    "schema=njrh.map_asset_integrity_degraded.v1\n"
                    f"map_id={map_id}\n"
                    f"non_keepout_asset_digest={non_keepout_digest}\n"
                    "reason_digest=0000000000000000\n",
                    encoding="utf-8",
                )
                assert integrity_marker.is_file()
                # Reproduce the real crash window: the keepout payload changed
                # after the repairable marker became durable, but before the
                # aggregate manifest identity was re-signed.
                primary_mask = map_root / "filters" / "keepout_mask.pgm"
                corrupted_mask = bytearray(primary_mask.read_bytes())
                assert corrupted_mask[-1] >= 250
                corrupted_mask[-1] = 253
                primary_mask.write_bytes(corrupted_mask)

                # A process restart must preserve the exact non-keepout proof
                # instead of replacing it with an unrepairable "unknown".
                stop_process(process)
                process = subprocess.Popen(
                    command,
                    stdout=log,
                    stderr=subprocess.STDOUT,
                    text=True,
                    env=environment,
                    start_new_session=True,
                )
                wait_for_port(args.port, process, 15.0)
                status, restarted_degraded_status = get_json(
                    args.port, "/api/v1/status"
                )
                assert status == 200, restarted_degraded_status
                assert (
                    restarted_degraded_status["keepout_integrity_degraded"] is True
                )
                marker_after_restart = integrity_marker.read_text(encoding="utf-8")
                assert f"map_id={map_id}\n" in marker_after_restart
                assert (
                    f"non_keepout_asset_digest={non_keepout_digest}\n"
                    in marker_after_restart
                )

                status, repaired = post_json(args.port, repair_payload)
                assert status == 200, repaired
                assert repaired["changed"] is True, repaired
                assert repaired["integrity_degraded"] is False, repaired
                assert not integrity_marker.exists()
                status, repaired_status = get_json(args.port, "/api/v1/status")
                assert status == 200, repaired_status
                assert repaired_status["keepout_integrity_degraded"] is False

                # A dangling activation-journal symlink is an invalid pending
                # transaction, never equivalent to an absent journal.
                activation_journal = (
                    root / "maps" / ".map_activation_transaction.v1"
                )
                activation_journal.symlink_to(
                    root / "missing_activation_journal_target"
                )
                status, linked_journal_status = get_json(
                    args.port, "/api/v1/status"
                )
                assert status == 200, linked_journal_status
                assert (
                    linked_journal_status["keepout_integrity_degraded"] is True
                )
                status, linked_journal_repair = post_json(
                    args.port, repair_payload
                )
                assert status == 503, linked_journal_repair
                activation_journal.unlink()
                status, journal_removed_status = get_json(
                    args.port, "/api/v1/status"
                )
                assert status == 200, journal_removed_status
                assert (
                    journal_removed_status["keepout_integrity_degraded"] is False
                )

                status, rejected = post_json(
                    args.port,
                    {
                        **base,
                        "keepout_lines": [
                            {
                                "id": "all_blocked",
                                "width_m": 1000.0,
                                "points": [
                                    {"x": 1.0, "y": 2.0},
                                    {"x": 10.0, "y": 11.0},
                                ],
                            }
                        ],
                        "keepout_polygons": [],
                    },
                )
                assert status == 422, rejected
                assert rejected["error"]["code"] == "KEEP_OUT_MASK_ALL_BLOCKED", rejected
                print(
                    json.dumps(
                        {
                            "ok": True,
                            "save_outcome": saved["outcome"],
                            "saved_active_cells": saved["mask"]["active_cells"],
                            "clear_active_cells": cleared["mask"]["active_cells"],
                            "persistent_identity_latch_repaired": True,
                            "real_crash_window_repair_proof_preserved": True,
                            "different_map_cannot_clear_latch": True,
                            "dangling_activation_journal_blocked": True,
                            "all_blocked_rejected": True,
                        },
                        separators=(",", ":"),
                    )
                )
            finally:
                stop_process(process)
        if process.returncode not in (0, -signal.SIGTERM):
            raise RuntimeError(
                f"isolated API node exited with code {process.returncode}: "
                f"{log_path.read_text(encoding='utf-8')}"
            )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
