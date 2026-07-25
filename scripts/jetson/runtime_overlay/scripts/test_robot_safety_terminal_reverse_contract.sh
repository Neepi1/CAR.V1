#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/common_env.sh"

test_id="${1:-$(date -u +%Y%m%dT%H%M%SZ)}"
test_token="run_${test_id//[^A-Za-z0-9_]/_}"
prefix="/njrh_test/robot_safety_terminal_reverse/${test_token}"
node_name="robot_safety_terminal_reverse_${test_id//[^A-Za-z0-9_]/_}"
node_bin="${NJRH_PROJECT_ROOT}/install/robot_safety/lib/robot_safety/robot_safety_node"
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
  echo "missing robot_safety binary: ${node_bin}" >&2
  exit 1
}

"${node_bin}" --ros-args \
  -r __node:="${node_name}" \
  -p mock_mode:=true \
  -p publish_rate_hz:=50.0 \
  -p watchdog_timeout_sec:=1.0 \
  -p cmd_vel_in_topic:="${prefix}/normal_cmd" \
  -p api_cmd_vel_in_topic:="${prefix}/api_cmd" \
  -p docking_cmd_vel_in_topic:="${prefix}/docking_cmd" \
  -p cmd_vel_out_topic:="${prefix}/cmd_out" \
  -p cmd_vel_mirror_topic:="${prefix}/cmd_mirror" \
  -p estop_topic:="${prefix}/estop" \
  -p localization_ok_topic:="${prefix}/localization_ok" \
  -p require_localization_health:=false \
  -p enable_bms_contact_guard:=false \
  -p enable_docking_status_guard:=false \
  -p enable_docked_latch_file_guard:=false \
  -p block_normal_motion_when_docked:=false \
  -p spin_to_drive_settle_enabled:=false \
  -p mode_exit_guard_enabled:=false \
  -p allow_reverse:=false \
  -p reverse_enable_topic:="${prefix}/reverse_enable" \
  -p docking_reverse_enable_topic:="${prefix}/docking_reverse_enable" \
  -p teleop_reverse_enable_topic:="${prefix}/teleop_reverse_enable" \
  -p reverse_enable_timeout_sec:=0.40 \
  -p normal_navigation_reverse_max_mps:=0.08 \
  -p status_topic:="${prefix}/safety_status" \
  -p motion_allowed_topic:="${prefix}/motion_allowed" \
  -p publish_zero_on_startup:=false \
  >"${log_file}" 2>&1 &
node_pid=$!

for _ in $(seq 1 50); do
  if ros2 node list 2>/dev/null | grep -Fxq "/${node_name}"; then
    break
  fi
  sleep 0.1
done
ros2 node list | grep -Fxq "/${node_name}" || {
  echo "isolated robot_safety node did not become ready" >&2
  cat "${log_file}" >&2
  exit 1
}

export TEST_SAFETY_PREFIX="${prefix}"
python3 - <<'PY'
import math
import os
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool


EPSILON = 1.0e-5


def is_zero(msg):
    values = (
        msg.linear.x,
        msg.linear.y,
        msg.linear.z,
        msg.angular.x,
        msg.angular.y,
        msg.angular.z,
    )
    return all(math.isfinite(value) and abs(value) <= EPSILON for value in values)


class Harness(Node):
    def __init__(self, prefix):
        super().__init__("robot_safety_terminal_reverse_harness")
        qos = QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE)
        self.prefix = prefix
        self.command_pub = self.create_publisher(Twist, prefix + "/normal_cmd", qos)
        self.permit_pub = self.create_publisher(Bool, prefix + "/reverse_enable", qos)
        self.outputs = []
        self.create_subscription(Twist, prefix + "/cmd_out", self._on_output, qos)

    def _on_output(self, msg):
        self.outputs.append((time.monotonic(), msg))

    def spin_for(self, duration):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.01)

    def wait_for_connections(self):
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if (
                self.command_pub.get_subscription_count() > 0
                and self.permit_pub.get_subscription_count() > 0
                and self.count_publishers(self.prefix + "/cmd_out") > 0
            ):
                return
        raise RuntimeError("isolated safety subscriptions did not connect")

    def publish_command_for(self, command, duration=0.25):
        mark = time.monotonic()
        deadline = mark + duration
        while time.monotonic() < deadline:
            self.command_pub.publish(command)
            rclpy.spin_once(self, timeout_sec=0.01)
            time.sleep(0.01)
        self.spin_for(0.05)
        return [msg for stamp, msg in self.outputs if stamp >= mark]

    def publish_permit_for(self, enabled, duration=0.20):
        message = Bool()
        message.data = enabled
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            self.permit_pub.publish(message)
            rclpy.spin_once(self, timeout_sec=0.01)
            time.sleep(0.01)


rclpy.init()
node = Harness(os.environ["TEST_SAFETY_PREFIX"])
try:
    node.wait_for_connections()
    node.spin_for(0.20)

    reverse_arc = Twist()
    reverse_arc.linear.x = -0.04
    reverse_arc.angular.z = 0.10
    outputs = node.publish_command_for(reverse_arc)
    if not outputs or any(not is_zero(msg) for msg in outputs):
        raise RuntimeError("forbidden reverse arc was not rejected atomically")

    node.publish_permit_for(True)
    fast_reverse_arc = Twist()
    fast_reverse_arc.linear.x = -0.20
    fast_reverse_arc.angular.z = 0.10
    mark = time.monotonic()
    permit = Bool()
    permit.data = True
    deadline = mark + 0.30
    while time.monotonic() < deadline:
        node.permit_pub.publish(permit)
        node.command_pub.publish(fast_reverse_arc)
        rclpy.spin_once(node, timeout_sec=0.01)
        time.sleep(0.01)
    permitted_outputs = [msg for stamp, msg in node.outputs if stamp >= mark]
    if not any(
        abs(msg.linear.x + 0.08) <= 0.002 and abs(msg.angular.z - 0.10) <= 0.002
        for msg in permitted_outputs
    ):
        raise RuntimeError("permitted reverse exceeded -0.08 m/s safety cap")
    if any(msg.linear.x < -0.082 for msg in permitted_outputs):
        raise RuntimeError("permitted reverse exceeded -0.08 m/s safety cap")

    node.spin_for(0.55)
    outputs = node.publish_command_for(reverse_arc)
    if not outputs or any(not is_zero(msg) for msg in outputs):
        raise RuntimeError("expired reverse permit did not restore atomic rejection")

    forward_arc = Twist()
    forward_arc.linear.x = 0.05
    forward_arc.angular.z = 0.02
    outputs = node.publish_command_for(forward_arc)
    if not any(
        abs(msg.linear.x - 0.05) <= 0.002 and abs(msg.angular.z - 0.02) <= 0.002
        for msg in outputs
    ):
        raise RuntimeError("forward command was changed by terminal reverse contract")

    print("PASS: forbidden normal reverse arcs are rejected atomically")
    print("PASS: fresh terminal permit preserves curvature and caps reverse at -0.08 m/s")
    print("PASS: expired terminal permit restores reverse rejection")
    print("PASS: forward navigation remains unchanged")
finally:
    node.destroy_node()
    rclpy.shutdown()
PY
