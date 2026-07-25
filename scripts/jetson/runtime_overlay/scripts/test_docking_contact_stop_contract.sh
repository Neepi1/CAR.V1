#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/common_env.sh"

test_id="${1:-$(date -u +%Y%m%dT%H%M%SZ)}"
prefix="/njrh_test/docking_contact_stop/${test_id}"
node_name="docking_contact_stop_test_${test_id//[^A-Za-z0-9_]/_}"
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
  -p undock.odom_topic:="${prefix}/local_odom" \
  -p contact_stop.motion_state_topic:="${prefix}/motion_state" \
  -p contact_stop.wheel_odom_topic:="${prefix}/wheel_odom" \
  >"${log_file}" 2>&1 &
node_pid=$!

for _ in $(seq 1 50); do
  if ros2 service list 2>/dev/null | grep -Fxq "${prefix}/start"; then
    break
  fi
  sleep 0.1
done
ros2 service list | grep -Fxq "${prefix}/start" || {
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
from ranger_msgs.msg import MotionState
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger


class ContactStopHarness(Node):
    def __init__(self, prefix):
        super().__init__("docking_contact_stop_contract_harness")
        normal = QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE)
        latched = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.battery_pub = self.create_publisher(BatteryState, prefix + "/battery", normal)
        self.motion_pub = self.create_publisher(MotionState, prefix + "/motion_state", normal)
        self.wheel_pub = self.create_publisher(Odometry, prefix + "/wheel_odom", normal)
        self.start_client = self.create_client(Trigger, prefix + "/start")
        self.park_values = []
        self.status_values = []
        self.commands = []
        self.create_subscription(Bool, prefix + "/park", self.park_values.append, latched)
        self.create_subscription(String, prefix + "/status", self._status, latched)
        self.create_subscription(Twist, prefix + "/cmd_vel", self.commands.append, normal)

    def _status(self, msg):
        self.status_values.append(msg.data)

    def spin_for(self, duration):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)

    def wait_for(self, predicate, timeout, description):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if predicate():
                return
        raise RuntimeError("timeout waiting for " + description)

    def publish_feedback(self, linear_x, duration):
        motion = MotionState()
        motion.motion_mode = MotionState.MOTION_MODE_SIDE_SLIP
        wheel = Odometry()
        wheel.twist.twist.linear.x = float(linear_x)
        period = 0.05
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            stamp = self.get_clock().now().to_msg()
            motion.header.stamp = stamp
            wheel.header.stamp = stamp
            self.motion_pub.publish(motion)
            self.wheel_pub.publish(wheel)
            rclpy.spin_once(self, timeout_sec=0.01)
            time.sleep(max(0.0, period - 0.01))


rclpy.init()
node = ContactStopHarness(os.environ["TEST_DOCKING_PREFIX"])
try:
    if not node.start_client.wait_for_service(timeout_sec=5.0):
        raise RuntimeError("isolated start service unavailable")
    for publisher in (node.battery_pub, node.motion_pub, node.wheel_pub):
        node.wait_for(
            lambda publisher=publisher: publisher.get_subscription_count() > 0,
            5.0,
            "test publisher subscription",
        )

    future = node.start_client.call_async(Trigger.Request())
    rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
    if not future.done() or not future.result().success:
        raise RuntimeError("isolated docking start failed")

    battery = BatteryState()
    battery.current = 1.0
    battery.voltage = 48.0
    battery.present = True
    node.battery_pub.publish(battery)
    node.wait_for(
        lambda: any(value.startswith("contact_stopping") for value in node.status_values),
        3.0,
        "ContactStopping",
    )
    command_count_at_contact = len(node.commands)

    node.spin_for(3.2)
    if any(msg.data for msg in node.park_values):
        raise RuntimeError("Park was requested after feedback timeout without stop confirmation")
    if not any("contact_stop_feedback_timeout=true" in value for value in node.status_values):
        raise RuntimeError("feedback timeout was not reported")

    node.publish_feedback(0.05, 0.9)
    if any(msg.data for msg in node.park_values):
        raise RuntimeError("Park was requested while wheel feedback was moving")
    if any("brake_confirmed=true" in value for value in node.status_values):
        raise RuntimeError("brake was confirmed while wheel feedback was moving")

    node.publish_feedback(0.0, 1.1)
    node.wait_for(
        lambda: any("brake_confirmed=true" in value for value in node.status_values),
        3.0,
        "brake confirmation",
    )
    if not any(msg.data for msg in node.park_values):
        raise RuntimeError("Park was not requested after stable stopped feedback")

    braking_commands = node.commands[command_count_at_contact:]
    zero_commands = [
        msg for msg in braking_commands
        if msg.linear.x == 0.0 and msg.linear.y == 0.0 and msg.angular.z == 0.0
    ]
    if len(zero_commands) < 10 or len(zero_commands) != len(braking_commands):
        raise RuntimeError(
            f"expected continuous zero-only braking commands, got "
            f"zero={len(zero_commands)} total={len(braking_commands)}"
        )

    final_status = next(
        value for value in reversed(node.status_values) if "brake_confirmed=true" in value
    )
    print("PASS: feedback timeout held continuous zero without Park")
    print("PASS: moving feedback held continuous zero without Park")
    print("PASS: stable stopped feedback allowed Park")
    print("status:", final_status)
finally:
    node.destroy_node()
    rclpy.shutdown()
PY

grep 'DOCK_BRAKE_' "${log_file}" || true
