#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/common_env.sh"

test_id="${1:-$(date -u +%Y%m%dT%H%M%SZ)}"
prefix="/njrh_test/docking_contact_slow_zone/${test_id}"
node_name="docking_contact_slow_zone_${test_id//[^A-Za-z0-9_]/_}"
node_bin="${NJRH_PROJECT_ROOT}/install/robot_docking_manager/lib/robot_docking_manager/docking_manager_node"
log_file="/tmp/${node_name}.log"
node_pid=""

cleanup() {
  if [[ -n "${node_pid}" ]]; then
    kill -INT "${node_pid}" 2>/dev/null || true
    wait "${node_pid}" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

[[ -x "${node_bin}" ]] || {
  echo "missing docking manager binary: ${node_bin}" >&2
  exit 1
}

"${node_bin}" --ros-args \
  -r __node:="${node_name}" \
  -p observation_backend:=target_observation \
  -p target_observation_topic:="${prefix}/target" \
  -p target_observation_source:=slow_zone_test \
  -p cmd_vel_topic:="${prefix}/cmd_vel" \
  -p status_topic:="${prefix}/status" \
  -p start_service:="${prefix}/start" \
  -p stop_service:="${prefix}/stop" \
  -p undock_service:="${prefix}/undock" \
  -p charging_state_topic:="${prefix}/battery" \
  -p docking_contact_latch_file:="/tmp/${node_name}_latch.json" \
  -p mode.forced_mode_topic:="${prefix}/forced_mode" \
  -p mode.park_topic:="${prefix}/park" \
  -p mode.reverse_enable_topic:="${prefix}/reverse_enable" \
  -p undock.odom_topic:="${prefix}/odom" \
  -p contact_stop.motion_state_topic:="${prefix}/motion_state" \
  -p contact_stop.wheel_odom_topic:="${prefix}/wheel_odom" \
  -p approach.allow_blind_approach:=false \
  -p approach.final_target_distance_m:=0.34 \
  -p detector.stable_frames_required:=1 \
  -p controller.yaw_realign_stable_frames:=1 \
  -p controller.contact_crawl_speed_mps:=0.05 \
  -p controller.contact_final_slow_zone_m:=0.30 \
  -p controller.contact_final_crawl_speed_mps:=0.02 \
  -p controller.contact_verify_max_distance_m:=0.31 \
  -p safety.control_rate_hz:=20.0 \
  >"${log_file}" 2>&1 &
node_pid=$!

for _ in $(seq 1 50); do
  if ros2 service list 2>/dev/null | grep -Fx "${prefix}/start" >/dev/null; then
    break
  fi
  sleep 0.1
done
ros2 service list | grep -Fx "${prefix}/start" >/dev/null || {
  echo "isolated docking node did not become ready" >&2
  cat "${log_file}" >&2
  exit 1
}

export TEST_DOCKING_PREFIX="${prefix}"
python3 - <<'PY'
import os
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from robot_interfaces.msg import DockTargetObservation
from std_msgs.msg import String
from std_srvs.srv import Trigger


class SlowZoneHarness(Node):
    def __init__(self, prefix):
        super().__init__("docking_contact_slow_zone_harness")
        self.prefix = prefix
        normal = QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE)
        latest = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.target_pub = self.create_publisher(
            DockTargetObservation, prefix + "/target", normal
        )
        self.odom_pub = self.create_publisher(Odometry, prefix + "/odom", normal)
        self.start_client = self.create_client(Trigger, prefix + "/start")
        self.commands = []
        self.statuses = []
        self.create_subscription(Twist, prefix + "/cmd_vel", self._on_command, latest)
        self.create_subscription(String, prefix + "/status", self._on_status, latched)

    def _on_command(self, msg):
        self.commands.append((time.monotonic(), msg))

    def _on_status(self, msg):
        self.statuses.append((time.monotonic(), msg.data))

    def spin_for(self, duration):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.01)

    def wait_for_connections(self):
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if (
                self.target_pub.get_subscription_count() > 0
                and self.odom_pub.get_subscription_count() > 0
                and self.count_publishers(self.prefix + "/cmd_vel") > 0
            ):
                return
        raise RuntimeError("isolated docking subscriptions did not connect")

    def publish_scene(self, odom_x, duration):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            stamp = self.get_clock().now().to_msg()
            observation = DockTargetObservation()
            observation.header.stamp = stamp
            observation.header.frame_id = "base_link"
            observation.source = "slow_zone_test"
            observation.sensor_healthy = True
            observation.valid = True
            observation.forward_gap_m = 0.34
            observation.lateral_error_m = 0.0
            observation.yaw_error_rad = 0.0
            observation.lateral_span_m = 0.235
            observation.confidence = 1.0
            observation.inlier_count = 100

            odom = Odometry()
            odom.header.stamp = stamp
            odom.pose.pose.position.x = float(odom_x)
            odom.pose.pose.orientation.w = 1.0
            self.target_pub.publish(observation)
            self.odom_pub.publish(odom)
            rclpy.spin_once(self, timeout_sec=0.01)
            time.sleep(0.04)


rclpy.init()
node = SlowZoneHarness(os.environ["TEST_DOCKING_PREFIX"])
try:
    node.wait_for_connections()
    node.spin_for(0.25)
    if not node.start_client.wait_for_service(timeout_sec=10.0):
        raise RuntimeError("isolated start service unavailable")
    future = node.start_client.call_async(Trigger.Request())
    rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)
    if not future.done() or not future.result().success:
        raise RuntimeError("isolated docking start failed")

    normal_mark = time.monotonic()
    node.publish_scene(0.0, 1.2)
    normal_commands = [msg for stamp, msg in node.commands if stamp >= normal_mark]
    if not any(abs(msg.linear.x - 0.05) <= 1.0e-4 for msg in normal_commands):
        raise RuntimeError("ContactVerify normal zone did not command 0.05 m/s")

    slow_mark = time.monotonic()
    node.publish_scene(0.020, 0.8)
    slow_commands = [msg for stamp, msg in node.commands if stamp >= slow_mark]
    if not slow_commands or any(msg.linear.x > 0.0201 for msg in slow_commands[-5:]):
        print(
            "DEBUG slow command tail:",
            [round(msg.linear.x, 4) for msg in slow_commands[-10:]],
        )
        print("DEBUG status tail:", [status for _, status in node.statuses[-10:]])
        raise RuntimeError("ContactVerify final 30 cm exceeded 0.02 m/s")
    if not any(
        "final_slow_zone=true" in status and "cmd_x=0.020" in status
        for _, status in node.statuses
    ):
        raise RuntimeError("ContactVerify slow-zone status was not published")

    print("PASS: ContactVerify normal zone commands 0.05 m/s")
    print("PASS: ContactVerify final 30 cm is capped at 0.02 m/s")
finally:
    node.destroy_node()
    rclpy.shutdown()
PY
