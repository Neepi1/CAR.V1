#!/usr/bin/env python3
"""Real API + isolated external FloorSwitch action; no production graph or motion."""

import argparse
import json
import os
from pathlib import Path
import subprocess
import tempfile
import threading
import time

# Set before importing ROS or constructing any participant.
os.environ.update(ROS_DOMAIN_ID="213", ROS_LOCALHOST_ONLY="1", RMW_IMPLEMENTATION="rmw_fastrtps_cpp")
os.environ.pop("ROBOT_API_TOKEN", None)

import rclpy
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from robot_interfaces.action import FloorSwitch
from robot_interfaces.msg import LocalizationHealth

from floor_runtime_http_smoke import (
    request_json, terminate_test_process_group, wait_for_port,
    write_complete_map_assets, write_map_manifest,
)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--node", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--port", type=int, default=18309)
    parser.add_argument("--negative-interlock", choices=("true", "false"), default="true")
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    observer = rclpy.create_node("isolated_unready_floor_action")
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(observer)
    requests = []

    def execute(goal):
        request = goal.request
        requests.append((request.building_id, request.floor_id, request.map_id))
        time.sleep(0.2)
        result = FloorSwitch.Result()
        result.success = True
        result.active_building_id = request.building_id
        result.active_floor_id = request.floor_id
        result.active_map_id = request.map_id
        result.asset_epoch = request.expected_asset_epoch
        result.asset_digest = request.expected_asset_digest
        result.explicit_relocalization_sequence = len(requests)
        result.runtime_context_valid = True
        goal.succeed()
        return result

    server = ActionServer(observer, FloorSwitch, "/floor_manager/floor_switch", execute)
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
    health_pub = observer.create_publisher(LocalizationHealth, "/localization/floor_health", qos)
    health = LocalizationHealth()
    health.runtime_context_valid = False
    health.detail = "isolated source localization failed"
    timer = observer.create_timer(0.2, lambda: health_pub.publish(health))
    worker = threading.Thread(target=executor.spin)
    worker.start()
    results = []
    try:
        with tempfile.TemporaryDirectory(prefix="fixtures_", dir=args.output_dir) as directory:
            root = Path(directory)
            for floor in ("F1", "F2"):
                bundle = write_map_manifest(root, building_id="B_test", floor_id=floor,
                                            map_id=f"map_{floor}", active=False)
                write_complete_map_assets(bundle)
            context = root / "runtime_context.json"
            base = dict(schema="njrh.runtime_map_context.v1", state="failed", confirmed=False,
                        building_id="B_test", floor_id="F1", map_id="map_F1",
                        asset_epoch=1, asset_digest="sha256:" + "a" * 64,
                        explicit_relocalization_sequence=0, updated_at=time.time())
            context.write_text(json.dumps(base), encoding="utf-8")
            command = [str(args.node), "--ros-args", "-p", f"port:={args.port}",
                       "-p", f"maps_root:={root / 'maps'}",
                       "-p", f"runtime_maps_dir:={root / 'runtime_maps'}",
                       "-p", f"runtime_map_context_file:={context}",
                       "-p", f"last_navigation_map_file:={root / 'last_map.json'}",
                       "-p", f"floor_runtime_negative_interlock_enabled:={args.negative_interlock}",
                       "-p", "navigation_resume_command:=/bin/false"]
            with (args.output_dir / "api.log").open("w") as log:
                process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                           start_new_session=True)
                try:
                    wait_for_port(args.port, process, 45.0)
                    # Allow retained source-invalid health to reach the real HTTP module.
                    time.sleep(1.0)
                    for index, state in enumerate(("failed", "starting", "missing", "ready_unlocalized")):
                        if state == "missing":
                            context.unlink(missing_ok=True)
                        else:
                            base["state"] = "ready" if state == "ready_unlocalized" else state
                            base["confirmed"] = state == "ready_unlocalized"
                            context.write_text(json.dumps(base), encoding="utf-8")
                        before = context.read_bytes() if context.exists() else None
                        floor = "F1" if index % 2 == 0 else "F2"
                        code, response = request_json(args.port, "POST", "/api/v1/floor-switch/start",
                            dict(building_id="B_test", floor_id=floor, map_id=f"map_{floor}"))
                        assert code == 202, (state, code, response)
                        tx = response["transaction"]["transaction_id"]
                        deadline = time.monotonic() + 10
                        while time.monotonic() < deadline:
                            code, response = request_json(args.port, "GET", f"/api/v1/floor-switch/state?transaction_id={tx}")
                            assert code == 200, response
                            if response["transaction"]["state"] == "COMPLETE":
                                break
                            time.sleep(0.05)
                        assert response["transaction"]["state"] == "COMPLETE", response
                        assert response["transaction"]["active_map_id"] == f"map_{floor}", response
                        assert (context.read_bytes() if context.exists() else None) == before, "API forged source readiness"
                        results.append(dict(source_state=state, target=floor, transaction=tx))
                    code, response = request_json(args.port, "POST", "/api/v1/floor-switch/start",
                        dict(building_id="B_test", floor_id="F3", map_id="missing_target"))
                    assert code == 404, response
                    assert len(requests) == 4, requests
                    print(json.dumps(dict(ok=True, cases=results, invalid_target_rejected=True,
                                          no_source_context_rewrite=True)))
                finally:
                    terminate_test_process_group(process.pid)
                    process.wait(timeout=5)
    finally:
        executor.shutdown(timeout_sec=3)
        worker.join(timeout=3)
        observer.destroy_timer(timer)
        server.destroy()
        observer.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
