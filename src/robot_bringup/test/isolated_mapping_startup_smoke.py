#!/usr/bin/env python3
"""Synthetic inputs only. Require disposable --network none --ipc private.

Usage: ROS_DOMAIN_ID=226 ROS_LOCALHOST_ONLY=1 python3 ... /candidate/probe
No PointCloud2 subscription, production services, real TF or robot motion.
"""
import json
import os
from pathlib import Path
import subprocess
import sys
import time

assert os.environ.get("ROS_LOCALHOST_ONLY") == "1"
assert os.environ.get("ROS_DOMAIN_ID") == "226"
assert os.readlink("/proc/self/ns/net") != os.readlink("/proc/1/ns/net"), "requires private network"
assert os.readlink("/proc/self/ns/ipc") != os.readlink("/proc/1/ns/ipc"), "requires private IPC"
links = subprocess.check_output(["ip", "-o", "link"], text=True).splitlines()
assert len(links) == 1 and ": lo:" in links[0], "requires loopback-only network"

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import LaserScan, PointCloud2
from std_msgs.msg import String
from std_srvs.srv import SetBool
from tf2_msgs.msg import TFMessage

PROBE = sys.argv[1]


def run_case(case, index):
    rclpy.init()
    topic = f"/fixture_{index}/scan"
    status_topic = f"/fixture_{index}/status"
    service = f"/fixture_{index}/enable"
    odom_topic = f"/fixture_{index}/odom"
    points_topic = f"/fixture_{index}/points"
    bridge_topic = f"/fixture_{index}/bridge"
    tf_topic = f"/fixture_{index}/tf"
    node = rclpy.create_node(f"resident_{index}", start_parameter_services=False, enable_rosout=False)
    other = rclpy.create_node(f"other_{index}", start_parameter_services=False, enable_rosout=False)
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
    status_pub = node.create_publisher(String, status_topic, qos)
    enabled = case not in ("disabled_restore", "preflight_restore", "late_restore_reply")
    scan_pub = None
    calls = []
    release_at = None
    reject = case == "rejected_release"
    delayed = case == "late_discovery"
    need_scan = case not in ("disabled_restore", "preflight_restore", "late_restore_reply", "unknown_state") and not delayed
    if need_scan:
        scan_pub = node.create_publisher(LaserScan, topic, qos)
    duplicate = None
    if case in ("other_owner", "duplicate_scan", "preflight_duplicate"):
        duplicate = other.create_publisher(LaserScan, topic, qos)
    tf_pub = node.create_publisher(TFMessage, tf_topic, qos)
    odom_pub = node.create_publisher(Odometry, odom_topic, qos)
    bridge_pub = node.create_publisher(Odometry, bridge_topic, qos)
    cloud_owner = other if case == "mismatched_pair" else node
    # Graph endpoint only: no point data is ever published.
    cloud_pub = cloud_owner.create_publisher(PointCloud2, points_topic, qos)
    state_node = None
    static_pub = None
    if case.startswith("preflight_"):
        state_node = rclpy.create_node("robot_local_state", start_parameter_services=False, enable_rosout=False)
        node.destroy_publisher(odom_pub)
        odom_pub = state_node.create_publisher(Odometry, odom_topic, qos)
        static_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                durability=DurabilityPolicy.TRANSIENT_LOCAL)
        static_pub = node.create_publisher(TFMessage, "/tf_static", static_qos)
        tf = TransformStamped()
        tf.header.frame_id = "base_link"
        tf.child_frame_id = "lidar_level_link"
        tf.transform.rotation.w = 1.0
        static_pub.publish(TFMessage(transforms=[tf]))

    finished_before_reply = False

    def on_call(request, response):
        nonlocal enabled, scan_pub, release_at, finished_before_reply
        calls.append(request.data)
        response.success = not reject
        if reject:
            return response
        if request.data:
            enabled = True
            scan_pub = node.create_publisher(LaserScan, topic, qos)
            if case == "late_restore_reply":
                deadline = time.monotonic() + 1.5
                while child.poll() is None and time.monotonic() < deadline:
                    time.sleep(0.01)
                finished_before_reply = child.poll() == 0
        else:
            release_at = time.monotonic() + 0.6
        return response

    srv = node.create_service(SetBool, service, on_call)
    mode = "release" if "release" in case else "ensure"
    if case.startswith("preflight_"):
        command = [PROBE, "mapping-preflight", topic, node.get_name(), odom_topic,
                   bridge_topic, "ekf", "3", "3", "3", "0.5", "1", "0.25", "25",
                   service, status_topic]
    elif case.startswith("scan_") or case == "duplicate_scan":
        command = [PROBE, "mapping-scan-ready", topic, tf_topic, "mapping_odom",
                   "3", "3", node.get_name(), "0.5"]
    elif case in ("paired_odom", "mismatched_pair"):
        command = [PROBE, "mapping-fastlio-ready", points_topic, odom_topic, bridge_topic, "3", "1"]
    else:
        command = [PROBE, "scan-handoff", mode, topic, node.get_name(), service, status_topic, "3"]
    start = time.monotonic()
    child = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    try:
        while child.poll() is None and time.monotonic() - start < 6:
            now = time.monotonic()
            if delayed and now - start >= 1.3 and scan_pub is None:
                scan_pub = node.create_publisher(LaserScan, topic, qos)
            if release_at is not None and now >= release_at and scan_pub is not None:
                node.destroy_publisher(scan_pub)
                scan_pub = None
                enabled = False
            stamp = node.get_clock().now().to_msg()
            if case != "unknown_state":
                value = str(enabled).lower()
                status_pub.publish(String(data=f"worker_scan_enabled={value} scan_publisher_registered={value}"))
            if scan_pub is not None:
                scan = LaserScan()
                scan.header.frame_id = "lidar_level_link"
                scan.header.stamp = stamp
                if case == "scan_stale":
                    scan.header.stamp.sec -= 5
                scan_pub.publish(scan)
            transform = TransformStamped()
            transform.header.frame_id = "mapping_odom"
            transform.child_frame_id = "lidar_level_link"
            transform.header.stamp = stamp
            if case == "scan_wrong_stamp_tf":
                transform.header.stamp.sec -= 5
            transform.transform.rotation.w = 1.0
            tf_pub.publish(TFMessage(transforms=[transform]))
            odom = Odometry()
            odom.header.stamp = node.get_clock().now().to_msg()
            odom_pub.publish(odom)
            bridge_pub.publish(odom)
            rclpy.spin_once(node, timeout_sec=0.02)
        if child.poll() is None:
            child.kill()
        output = child.communicate(timeout=2)[0]
        expected = case not in ("unknown_state", "other_owner", "rejected_release",
                                "scan_stale", "scan_wrong_stamp_tf", "duplicate_scan", "mismatched_pair",
                                "preflight_duplicate")
        assert (child.returncode == 0) == expected, (case, child.returncode, output)
        if case in ("late_discovery", "other_owner", "unknown_state"):
            assert not calls, (case, calls)
        if case in ("disabled_restore", "preflight_restore", "late_restore_reply"):
            assert calls == [True], calls
        if case == "late_restore_reply":
            assert finished_before_reply, "actual owner was ready but client waited for late RPC reply"
        if case == "delayed_release":
            assert calls == [False] and time.monotonic() >= release_at, calls
        print(json.dumps({"case": case, "result": "PASS", "service_calls": calls,
                          "elapsed_sec": round(time.monotonic() - start, 3), "output": output}), flush=True)
    finally:
        if child.poll() is None:
            child.kill()
            child.wait()
        node.destroy_node()
        other.destroy_node()
        if state_node is not None:
            state_node.destroy_node()
        rclpy.shutdown()


for index, case in enumerate(("late_discovery", "disabled_restore", "delayed_release",
                             "rejected_release", "unknown_state", "other_owner", "scan_good",
                             "scan_stale", "scan_wrong_stamp_tf", "duplicate_scan",
                             "paired_odom", "mismatched_pair", "preflight_ready",
                             "preflight_restore", "preflight_duplicate", "late_restore_reply")):
    run_case(case, index)
