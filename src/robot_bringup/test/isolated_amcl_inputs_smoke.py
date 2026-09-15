#!/usr/bin/env python3
"""Actual AMCL input probe integration in a loopback-only nonproduction domain.

Run with ROS_DOMAIN_ID=224, ROS_LOCALHOST_ONLY=1 and a loopback-only Fast DDS
profile. Only synthetic map/scan/TF messages are published; no services, goals,
velocity commands, physical sensors or production ROS participants are used.
"""
import argparse
import json
import os
import queue
import subprocess
import threading
import time


PHASES = {"MAP", "SCAN", "MAP_TF", "ODOM_TF", "SENSOR_TF"}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--probe-bin", required=True)
    args = parser.parse_args()
    if (os.environ.get("ROS_DOMAIN_ID") != "224" or
            os.environ.get("ROS_LOCALHOST_ONLY") != "1" or
            os.environ.get("RMW_IMPLEMENTATION") != "rmw_fastrtps_cpp"):
        parser.error("Requires ROS_DOMAIN_ID=224 ROS_LOCALHOST_ONLY=1 "
                     "RMW_IMPLEMENTATION=rmw_fastrtps_cpp")

    import rclpy
    from geometry_msgs.msg import TransformStamped
    from nav_msgs.msg import OccupancyGrid
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import LaserScan
    from tf2_msgs.msg import TFMessage

    rclpy.init(args=[])
    node = Node("isolated_amcl_inputs_fixture", enable_rosout=False)
    probe = None
    try:
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            others = [name for name in node.get_node_names()
                      if name != "isolated_amcl_inputs_fixture"]
            if others:
                raise RuntimeError(f"Isolation domain is occupied: {others}")

        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        sensor = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        map_pub = node.create_publisher(OccupancyGrid, "/map", latched)
        scan_pub = node.create_publisher(LaserScan, "/amcl_inputs_test/scan", sensor)
        tf_pub = node.create_publisher(TFMessage, "/tf", 100)
        static_pub = node.create_publisher(TFMessage, "/tf_static", latched)
        lines = queue.Queue()

        def run_case(name, pending, *, known="-", scan_frame="actual_test_lidar",
                     map_delay=1.0, scan_delay=1.8, tf_delay=2.6,
                     map_timeout=15.0, scan_timeout=15.0, tf_timeout=15.0,
                     missing=None, expected_rc=0, expected=PHASES,
                     map_size=1, forbidden_subscriptions=()):
            nonlocal probe
            while not lines.empty():
                lines.get_nowait()
            command = [args.probe_bin, "amcl-inputs", "/amcl_inputs_test/scan",
                       "fallback_test_lidar", known, ",".join(sorted(pending)),
                       str(map_timeout), str(scan_timeout), str(tf_timeout)]
            # Replace the prior transient-local map sample before each fresh
            # probe; an empty-grid test must not see the earlier valid map.
            grid = OccupancyGrid()
            grid.header.frame_id = "map"
            grid.info.resolution = 0.05
            grid.info.origin.orientation.w = 1.0
            map_pub.publish(grid)
            grid.info.width = grid.info.height = map_size
            grid.data = [0] * (map_size * map_size)
            started = time.monotonic()
            probe = subprocess.Popen(command, stdout=subprocess.PIPE,
                                     stderr=subprocess.STDOUT, text=True)

            def read_output():
                for line in probe.stdout:
                    lines.put((time.monotonic() - started, line.rstrip("\n")))

            reader = threading.Thread(target=read_output, daemon=True)
            reader.start()
            output = []
            while probe.poll() is None:
                elapsed = time.monotonic() - started
                if elapsed > max(map_timeout, scan_timeout, tf_timeout) + 5:
                    raise AssertionError(f"{name}: exceeded bounded deadline")
                stamp = node.get_clock().now().to_msg()
                if map_delay is not None and elapsed >= map_delay:
                    grid.header.stamp = stamp
                    map_pub.publish(grid)
                if scan_delay is not None and elapsed >= scan_delay:
                    scan = LaserScan()
                    scan.header.frame_id = scan_frame
                    scan.header.stamp = stamp
                    scan_pub.publish(scan)
                if elapsed >= tf_delay:
                    dynamic = []
                    for target, source, phase in (
                            ("map", "odom", "MAP_TF"),
                            ("odom", "base_link", "ODOM_TF"),
                            ("base_link", scan_frame or "fallback_test_lidar", "SENSOR_TF")):
                        if missing == phase:
                            continue
                        transform = TransformStamped()
                        transform.header.frame_id = target
                        transform.child_frame_id = source
                        transform.header.stamp = stamp
                        transform.transform.rotation.w = 1.0
                        if phase == "SENSOR_TF":
                            static_pub.publish(TFMessage(transforms=[transform]))
                        else:
                            dynamic.append(transform)
                    tf_pub.publish(TFMessage(transforms=dynamic))
                rclpy.spin_once(node, timeout_sec=0.05)
                for topic in forbidden_subscriptions:
                    owners = [info.node_name for info in node.get_subscriptions_info_by_topic(topic)]
                    assert "runtime_readiness_probe" not in owners, (name, topic, owners)
                while not lines.empty():
                    at, line = lines.get_nowait()
                    output.append((at, line))
            probe.wait(timeout=2)
            reader.join(timeout=2)
            while not lines.empty():
                output.append(lines.get_nowait())
            stdout = "\n".join(line for _, line in output)
            received = {line.split("=", 1)[1] for _, line in output
                        if line.startswith("AMCL_INPUT_READY=")}
            elapsed = time.monotonic() - started
            assert probe.returncode == expected_rc, (name, probe.returncode, stdout)
            assert received == set(expected), (name, received, stdout)
            result = {"case": name, "rc": probe.returncode,
                      "elapsed_sec": round(elapsed, 3), "phases": sorted(received),
                      "progress": [{"at_sec": round(at, 3), "line": line}
                                   for at, line in output if line.startswith("AMCL_INPUT_")]}
            print(json.dumps(result), flush=True)
            probe = None
            return output

        output = run_case("delayed_inputs_converge_together", PHASES)
        assert any(line == "AMCL_INPUT_FRAME=actual_test_lidar" for _, line in output)
        phase_times = {line.split("=", 1)[1]: at for at, line in output
                       if line.startswith("AMCL_INPUT_READY=")}
        # Partial successes must be flushed, not withheld until the final TF.
        assert phase_times["MAP"] < phase_times["SENSOR_TF"]
        assert phase_times["SCAN"] < phase_times["SENSOR_TF"]

        output = run_case(
            "missing_sensor_tf_retains_partial_progress", PHASES,
            scan_frame="missing_test_lidar", map_delay=0.0, scan_delay=0.0,
            tf_delay=0.0, missing="SENSOR_TF", expected_rc=1,
            expected=PHASES - {"SENSOR_TF"})
        assert any("timed out" in line and "SENSOR_TF" in line for _, line in output)

        output = run_case(
            "sensor_only_resume_uses_known_frame_without_scan_subscription", {"SENSOR_TF"},
            known="resumed_test_lidar", scan_frame="resumed_test_lidar",
            map_delay=None, scan_delay=None, tf_delay=1.5, expected={"SENSOR_TF"},
            forbidden_subscriptions=("/map", "/amcl_inputs_test/scan"))
        assert not any(line.startswith("AMCL_INPUT_FRAME=") for _, line in output)

        output = run_case(
            "legacy_checkpoint_without_frame_observes_frame_before_sensor_tf", {"SENSOR_TF"},
            scan_frame="legacy_checkpoint_lidar", map_delay=None,
            scan_delay=1.5, tf_delay=0.0, expected={"SENSOR_TF"},
            forbidden_subscriptions=("/map",))
        assert any(line == "AMCL_INPUT_FRAME=legacy_checkpoint_lidar" for _, line in output)

        output = run_case(
            "empty_scan_frame_retains_configured_fallback", {"SCAN", "SENSOR_TF"},
            scan_frame="", map_delay=None, scan_delay=0.0, tf_delay=0.0,
            expected={"SCAN", "SENSOR_TF"}, forbidden_subscriptions=("/map",))
        assert any(line == "AMCL_INPUT_FRAME=fallback_test_lidar" for _, line in output)

        output = run_case(
            "missing_scan_does_not_accept_fallback_without_message", PHASES,
            scan_frame="unseen_test_lidar", map_delay=0.0, scan_delay=None,
            tf_delay=0.0, expected_rc=1,
            expected={"MAP", "MAP_TF", "ODOM_TF"})
        assert not any(line.startswith("AMCL_INPUT_FRAME=") for _, line in output)

        run_case("empty_grid_is_not_ready", {"MAP"}, map_size=0,
                 map_delay=0.0, scan_delay=None, tf_delay=99.0,
                 map_timeout=3.0, expected_rc=1, expected=set(),
                 forbidden_subscriptions=("/amcl_inputs_test/scan", "/tf", "/tf_static"))

        output = run_case(
            "scan_frame_cannot_inject_progress", {"SCAN"},
            scan_frame="bad\nAMCL_INPUT_READY=SENSOR_TF", map_delay=None,
            scan_delay=0.0, tf_delay=99.0, expected_rc=2, expected=set())
        assert not any(line.startswith("AMCL_INPUT_FRAME=") for _, line in output)

        for phases, map_timeout in (("MAP,BOGUS", "4"), ("MAP,MAP", "4"),
                                    ("", "4"), ("MAP,", "4"), ("MAP", "nan")):
            invalid = subprocess.run(
                [args.probe_bin, "amcl-inputs", "/amcl_inputs_test/scan",
                 "fallback_test_lidar", "-", phases, map_timeout, "4", "4"],
                capture_output=True, text=True, timeout=7)
            assert invalid.returncode == 2, (phases, map_timeout, invalid.stderr)
            assert "AMCL_INPUT_READY=" not in invalid.stdout
        print(json.dumps({"invalid_cli_inputs": "passed"}), flush=True)
    finally:
        if probe is not None and probe.poll() is None:
            probe.terminate()
            try:
                probe.wait(timeout=3)
            except subprocess.TimeoutExpired:
                probe.kill()
                probe.wait(timeout=3)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
