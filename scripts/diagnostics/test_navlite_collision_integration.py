#!/usr/bin/env python3
"""Real CollisionMonitor I/O fixture; refuse the robot network/IPC namespaces.

Runs a supplied same-version executable with synthetic Scan/Twist, never a
robot task. Baseline and candidate runs should have equal observed commands.
Only fixture lifecycle services are called. No hardware node is launched.
"""
import argparse
import importlib.util
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time


def require_isolation():
    if os.environ.get("NAVLITE_ISOLATED") != "1" or os.environ.get("ROS_DOMAIN_ID") != "181":
        raise RuntimeError("requires the documented private namespace wrapper")
    for kind in ("net", "ipc", "mnt"):
        if os.readlink("/proc/self/ns/" + kind) == os.readlink("/proc/1/ns/" + kind):
            raise RuntimeError("refusing shared production " + kind + " namespace")
    if not Path("/dev/shm/.navlite_fixture_only").exists():
        raise RuntimeError("requires private tmpfs /dev/shm marker")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--binary", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--expect-navlite", action="store_true")
    parser.add_argument("--recorder", help="Optional file-only recorder under test")
    parser.add_argument("--expected-library", help="Assert the exact mapped candidate core library")
    args = parser.parse_args()
    require_isolation()
    import rclpy
    import yaml
    from geometry_msgs.msg import Twist
    from sensor_msgs.msg import LaserScan
    from lifecycle_msgs.srv import ChangeState
    from rclpy.qos import qos_profile_sensor_data

    root = Path(args.output).resolve()
    root.mkdir(exist_ok=False)
    params = {
        "use_sim_time": False, "base_frame_id": "base_link",
        "odom_frame_id": "odom", "base_shift_correction": False,
        "transform_tolerance": 0.01, "source_timeout": 0.25,
        "stop_pub_timeout": 0.20, "bond_heartbeat_period": 0.0,
        "cmd_vel_in_topic": "input", "cmd_vel_out_topic": "output",
        "polygons": ["StopZone", "SlowZone"],
        "StopZone.type": "polygon", "StopZone.action_type": "stop",
        "StopZone.points": [0.4, 0.3, 0.4, -0.3, -0.4, -0.3, -0.4, 0.3],
        "StopZone.max_points": 0, "StopZone.visualize": False,
        "SlowZone.type": "polygon", "SlowZone.action_type": "slowdown",
        "SlowZone.points": [0.8, 0.5, 0.8, -0.5, -0.8, -0.5, -0.8, 0.5],
        "SlowZone.max_points": 0, "SlowZone.slowdown_ratio": 0.5,
        "SlowZone.visualize": False, "observation_sources": ["scan"],
        "scan.type": "scan", "scan.topic": "scan",
    }
    config = root / "fixture.yaml"
    config.write_text(yaml.safe_dump({"/**": {"ros__parameters": params}}))
    log_path = root / "resident_navigation_runtime.log"
    result = {"isolated": True, "cases": [], "binary": str(Path(args.binary).resolve())}
    rclpy.init()
    node = rclpy.create_node("navlite_fixture_driver", namespace="navlite_fixture")
    received = []
    sub = node.create_subscription(Twist, "output", lambda msg: received.append(
        (time.monotonic(), msg.linear.x, msg.linear.y, msg.angular.z)), 20)
    cmd_pub = node.create_publisher(Twist, "input", 10)
    scan_pub = node.create_publisher(LaserScan, "scan", qos_profile_sensor_data)
    client = node.create_client(ChangeState, "collision_monitor/change_state")
    log_handle = log_path.open("w")
    recorder = None
    if args.recorder:
        recorder_output = root / "capture"
        recorder = subprocess.Popen(["python3", "-B", args.recorder, "record",
            "--log", str(log_path), "--output", str(recorder_output),
            "--duration", "35", "--interval", "0.1", "--no-stdin", "--min-free-mb", "0"],
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        deadline = time.monotonic() + 5
        while not (recorder_output / "events.jsonl").exists() or "opened" not in (
                recorder_output / "events.jsonl").read_text():
            if recorder.poll() is not None or time.monotonic() > deadline:
                if recorder.poll() is None:
                    recorder.send_signal(signal.SIGINT)
                raise RuntimeError("recorder did not open fixture log: " +
                                   recorder.communicate(timeout=5)[0].decode())
            time.sleep(0.02)
    child_env = dict(os.environ)
    if args.expected_library:
        # ROS setup prepends /opt/ros/lib, which otherwise shadows the candidate
        # RUNPATH. Removing this variable uses the executable's exact build RPATH;
        # it does not inject an alternative ROS dependency version.
        child_env.pop("LD_LIBRARY_PATH", None)
    proc = subprocess.Popen([args.binary, "--ros-args", "-r", "__ns:=/navlite_fixture",
        "--params-file", str(config), "--log-level", "warn"],
        stdout=log_handle, stderr=subprocess.STDOUT, env=child_env)

    def spin(seconds):
        stop = time.monotonic() + seconds
        while time.monotonic() < stop:
            if proc.poll() is not None:
                raise RuntimeError("fixture exited: " + log_path.read_text()[-4000:])
            rclpy.spin_once(node, timeout_sec=min(0.02, max(0.0, stop-time.monotonic())))

    def transition(code):
        request = ChangeState.Request()
        request.transition.id = code
        future = client.call_async(request)
        stop = time.monotonic() + 8
        while not future.done() and time.monotonic() < stop:
            spin(0.02)
        assert future.done() and future.result().success, (code, log_path.read_text())

    def scan(distance=2.0, frame="base_link", age=0.0):
        msg = LaserScan()
        stamp = node.get_clock().now().nanoseconds - int(age * 1e9)
        msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(stamp, 1000000000)
        msg.header.frame_id = frame
        msg.angle_min = 0.0
        msg.angle_max = 0.0
        msg.angle_increment = 0.01
        msg.range_min = 0.01
        msg.range_max = 10.0
        msg.ranges = [float(distance)]
        scan_pub.publish(msg)

    def command(vx=0.2, vy=-0.1, wz=0.05):
        msg = Twist()
        msg.linear.x, msg.linear.y, msg.angular.z = vx, vy, wz
        cmd_pub.publish(msg)

    def case(name, expected, distance=2.0, frame="base_link", age=0.0,
             command_values=(0.2, -0.1, 0.05), duration=0.13, scans=True):
        # Prime the independent Scan callback before driving input through the real node.
        if scans:
            scan(distance, frame, age)
            spin(0.06)
        received.clear()
        start = time.monotonic()
        while time.monotonic() - start < duration:
            if scans:
                scan(distance, frame, age)
            command(*command_values)
            spin(0.035)
        values = [list(row[1:]) for row in received]
        if expected is None:
            assert not values, (name, values)
        else:
            assert values, (name, log_path.read_text()[-3000:])
            assert all(all(abs(a-b) < 1e-7 for a, b in zip(row, expected))
                       for row in values), (name, values, expected)
        result["cases"].append({"name": name, "observed": values,
                                 "expected": expected, "passed": True})

    try:
        assert client.wait_for_service(timeout_sec=12), log_path.read_text()
        if args.expected_library:
            mapped = {line.split()[-1] for line in Path(f"/proc/{proc.pid}/maps").read_text().splitlines()
                      if "libcollision_monitor_core.so" in line}
            assert mapped == {str(Path(args.expected_library).resolve())}, mapped
            result["mapped_core"] = sorted(mapped)
        transition(1)  # configure only this isolated fixture
        transition(3)  # activate only this isolated fixture
        deadline = time.monotonic() + 8
        while (cmd_pub.get_subscription_count() == 0 or
               scan_pub.get_subscription_count() == 0 or node.count_publishers(sub.topic_name) == 0):
            assert time.monotonic() < deadline, "fixture discovery timeout"
            spin(0.05)
        case("no_scan_is_not_a_fabricated_stop", [0.2, -0.1, 0.05], scans=False)
        case("normal_pass_signed_components", [0.2, -0.1, 0.05])
        case("slowdown", [0.1, -0.05, 0.025], distance=0.6)
        case("stop", [0.0, 0.0, 0.0], distance=0.2)
        spin(0.30)
        case("stop_timeout_means_no_publication", None, distance=0.2, duration=1.15)
        case("obstacle_cleared_nonzero_release", [0.2, -0.1, 0.05])
        case("expired_scan_is_reported_not_relabelled_stop", [0.2, -0.1, 0.05], age=2.0)
        case("missing_tf_is_reported_not_relabelled_stop", [0.2, -0.1, 0.05], frame="no_such_frame")
        case("tf_scan_recovery", [0.2, -0.1, 0.05])
        case("active_zero_is_a_sent_command", [0.0, 0.0, 0.0], command_values=(0.0, 0.0, 0.0))
        spin(0.3)
        case("active_zero_timeout_is_no_message", None, command_values=(0.0, 0.0, 0.0))
        case("resume_after_active_zero", [0.2, -0.1, 0.05])
        case("invalid_twist_no_publication", None, command_values=(math.nan, 0.0, 0.0))
        transition(4)  # deactivate
        case("inactive_no_publication", None)
        text = log_path.read_text()
        if args.expect_navlite:
            for required in ("NAVLITE collision event=diagnostics_ready schema=1",
                             "action=STOP region=StopZone published=1",
                             "action=SLOWDOWN region=SlowZone published=1",
                             "reason=stop_pub_timeout_no_message", "publication=no_message",
                             "reason=inactive_no_output", "reason=invalid_input_no_output",
                             "scan=source_timeout", "scan=transform_unavailable_without_shift",
                             "scan=ok", "previous=STOP:StopZone"):
                assert required in text, "missing real producer log: " + required
            repeats = [x for x in text.splitlines() if "NAVLITE collision" in x and
                       "reason=stop_pub_timeout_no_message" in x and "region=StopZone" in x]
            assert 1 <= len(repeats) <= 3, ("failure log flood", repeats)
            result["navlite_lines"] = [x for x in text.splitlines() if "NAVLITE" in x]
        result["passed"] = True
    finally:
        if proc.poll() is None:
            proc.send_signal(signal.SIGINT)
        try:
            proc.wait(timeout=8)
        except subprocess.TimeoutExpired:
            proc.kill()  # only the subprocess created above
            proc.wait(timeout=3)
        log_handle.close()
        if recorder is not None:
            # Let the independent file-only poller drain the final bounded log batch.
            time.sleep(0.25)
            if recorder.poll() is None:
                recorder.send_signal(signal.SIGINT)
            recorder_text = recorder.communicate(timeout=5)[0].decode()
            (root / "recorder.log").write_text(recorder_text)
            result["recorder_exit"] = recorder.returncode
            events = [json.loads(line) for line in (root / "capture/events.jsonl").read_text().splitlines()]
            result["captured_navlite_lines"] = [x["text"] for x in events if "NAVLITE" in x.get("text", "")]
            if result.get("passed"):
                assert recorder.returncode == 0, recorder_text
                assert len(result["captured_navlite_lines"]) == len(result["navlite_lines"]), (
                    "real log capture lost events", len(result["captured_navlite_lines"]),
                    len(result["navlite_lines"]))
                spec = importlib.util.spec_from_file_location("navlite_fixture_parser", args.recorder)
                parser_module = importlib.util.module_from_spec(spec)
                spec.loader.exec_module(parser_module)
                parsed_rows = []
                for line_no, event in enumerate(events, 1):
                    if "NAVLITE" not in event.get("text", ""):
                        continue
                    row, _ = parser_module.navlite_row(event, line_no)
                    assert row and not row["parse_issues"], ("unparseable producer event", row)
                    assert all(value != "" for value in json.loads(row["fields_json"]).values()), row
                    parsed_rows.append(row)
                inactive = [row for row in parsed_rows if row["reason"] == "inactive_no_output"]
                assert len(inactive) == 1 and inactive[0]["output_status"] == "no_message", inactive
                result["parser_issue_count"] = 0
                result["parsed_navlite_count"] = len(parsed_rows)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        (root / "result.json").write_text(json.dumps(result, indent=2))
    print(json.dumps({"passed": result["passed"], "case_count": len(result["cases"]),
                      "report": str(root / "result.json")}))


if __name__ == "__main__":
    main()
