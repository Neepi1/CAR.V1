#!/usr/bin/env python3
"""Regression test for stopping an idle Nav2 runtime without latching recovery."""

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
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionServer


def wait_for_port(port: int, process: subprocess.Popen[str], timeout_sec: float) -> None:
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        if process.poll() is not None:
            raise RuntimeError(f"API node exited during startup with code {process.returncode}")
        try:
            with socket.create_connection(("127.0.0.1", port), timeout=0.2):
                return
        except OSError:
            time.sleep(0.05)
    raise RuntimeError("timed out waiting for isolated API node")


def request_json(
    port: int,
    method: str,
    path: str,
    payload: dict[str, object] | None = None,
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


def terminate_process_group(process: subprocess.Popen[str]) -> None:
    try:
        os.killpg(process.pid, signal.SIGTERM)
    except ProcessLookupError:
        return
    try:
        process.wait(timeout=5.0)
    except subprocess.TimeoutExpired:
        try:
            os.killpg(process.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        process.wait(timeout=5.0)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--node", type=Path, required=True)
    parser.add_argument("--port", type=int, required=True)
    args = parser.parse_args()

    isolated_domain_id = str(100 + (args.port % 100))
    os.environ["ROS_DOMAIN_ID"] = isolated_domain_id
    environment = os.environ.copy()
    environment["ROS_DOMAIN_ID"] = isolated_domain_id
    environment.setdefault("ROS_LOCALHOST_ONLY", "1")

    with tempfile.TemporaryDirectory(prefix="navigation_cancel_http_smoke_") as temp_dir:
        root = Path(temp_dir)
        maps_root = root / "maps"
        maps_root.mkdir()
        runtime_maps_dir = root / "runtime_maps"
        runtime_maps_dir.mkdir()
        stop_trace = root / "navigation_stop_trace.txt"
        stop_command = root / "fake_navigation_stop.sh"
        stop_command.write_text(
            "#!/usr/bin/env bash\n"
            "set -eu\n"
            f"printf '%s\\n' stopped > '{stop_trace.as_posix()}'\n",
            encoding="utf-8",
        )
        stop_command.chmod(0o755)

        rclpy.init()
        probe = rclpy.create_node("navigation_cancel_http_smoke_probe")

        def execute_navigation_goal(goal_handle: object) -> NavigateToPose.Result:
            goal_handle.abort()
            return NavigateToPose.Result()

        action_server = ActionServer(
            probe,
            NavigateToPose,
            "/navigate_to_pose",
            execute_navigation_goal,
        )
        spin_stop = threading.Event()

        def spin_probe() -> None:
            while not spin_stop.is_set():
                rclpy.spin_once(probe, timeout_sec=0.05)

        spin_thread = threading.Thread(target=spin_probe, daemon=True)
        spin_thread.start()

        log_path = root / "api.log"
        command = [
            str(args.node),
            "--ros-args",
            "-p",
            f"port:={args.port}",
            "-p",
            f"maps_root:={maps_root}",
            "-p",
            f"runtime_maps_dir:={runtime_maps_dir}",
            "-p",
            f"navigation_stop_command:={stop_command}",
            "-p",
            f"navigation_stop_log_file:={root / 'navigation_stop.log'}",
            "-p",
            "service_timeout_sec:=1.0",
            "-p",
            "navigation_cancel_action_wait_sec:=0.5",
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
                time.sleep(0.75)

                status, accepted = request_json(
                    args.port,
                    "POST",
                    "/api/v1/navigation/stop_runtime",
                    {"reason": "idle_nav2_stop_regression"},
                )
                assert status == 202, accepted
                assert accepted["accepted"] is True, accepted
                assert accepted["stop_stack"] is True, accepted

                cancel_state: dict[str, object] | None = None
                deadline = time.monotonic() + 8.0
                while time.monotonic() < deadline:
                    state_status, state = request_json(
                        args.port, "GET", "/api/v1/navigation/state"
                    )
                    assert state_status == 200, state
                    candidate = state["navigation_cancel"]
                    if candidate["state"] != "running":
                        cancel_state = candidate
                        break
                    time.sleep(0.05)
                assert cancel_state is not None, "navigation cancel job did not finish"
                assert cancel_state["state"] == "succeeded", cancel_state
                assert cancel_state["cancel_all_requested"] is True, cancel_state
                assert cancel_state["cancel_all_ok"] is True, cancel_state
                assert cancel_state["navigation_stack_stopped"] is True, cancel_state
                assert stop_trace.read_text(encoding="utf-8").strip() == "stopped"

                runtime_status_code, runtime_status = request_json(
                    args.port, "GET", "/api/v1/status"
                )
                assert runtime_status_code == 200, runtime_status
                assert runtime_status["delayed_side_effect_unknown_count"] == 0, runtime_status
                assert runtime_status["delayed_side_effect_recovery_blocked"] is False, runtime_status

                print(
                    json.dumps(
                        {
                            "ok": True,
                            "idle_cancel_all_proven": True,
                            "runtime_stop_gated_by_cancel_proof": True,
                            "delayed_side_effect_unknown_count": 0,
                        },
                        separators=(",", ":"),
                    )
                )
            finally:
                terminate_process_group(process)
                spin_stop.set()
                spin_thread.join(timeout=2.0)
                action_server.destroy()
                probe.destroy_node()
                rclpy.shutdown()
                assert not spin_thread.is_alive()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
