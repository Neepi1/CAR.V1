#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/common_env.sh"

test_id="${1:-$(date -u +%Y%m%dT%H%M%SZ)}"
prefix="/njrh_test/robot_safety_bms_interlock/${test_id}"
node_name="robot_safety_bms_interlock_${test_id//[^A-Za-z0-9_]/_}"
node_bin="${NJRH_PROJECT_ROOT}/install/robot_safety/lib/robot_safety/robot_safety_node"
log_file="/tmp/${node_name}.log"
latch_file="/tmp/${node_name}_latch.json"
node_pid=""

cleanup() {
  if [[ -n "${node_pid}" ]]; then
    kill -INT "${node_pid}" 2>/dev/null || true
    wait "${node_pid}" 2>/dev/null || true
  fi
  rm -f "${latch_file}"
}
trap cleanup EXIT INT TERM

[[ -x "${node_bin}" ]] || {
  echo "missing robot_safety binary: ${node_bin}" >&2
  exit 1
}

rm -f "${latch_file}"
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
  -p battery_state_topic:="${prefix}/battery" \
  -p docking_status_topic:="${prefix}/docking_status" \
  -p docking_contact_latch_file:="${latch_file}" \
  -p enable_bms_contact_guard:=true \
  -p dock_contact_max_age_sec:=0.25 \
  -p enable_docking_status_guard:=false \
  -p enable_docked_latch_file_guard:=false \
  -p block_normal_motion_when_docked:=true \
  -p allow_docking_cmd_when_docked:=true \
  -p spin_to_drive_settle_enabled:=false \
  -p mode_exit_guard_enabled:=false \
  -p reverse_enable_topic:="${prefix}/reverse_enable" \
  -p docking_reverse_enable_topic:="${prefix}/docking_reverse_enable" \
  -p teleop_reverse_enable_topic:="${prefix}/teleop_reverse_enable" \
  -p reverse_enable_timeout_sec:=1.0 \
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
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool, String


def is_zero(msg, epsilon=1.0e-5):
    values = (
        msg.linear.x,
        msg.linear.y,
        msg.linear.z,
        msg.angular.x,
        msg.angular.y,
        msg.angular.z,
    )
    return all(math.isfinite(value) and abs(value) <= epsilon for value in values)


class InterlockHarness(Node):
    def __init__(self, prefix):
        super().__init__("robot_safety_bms_interlock_harness")
        self.prefix = prefix
        qos = QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE)
        # Separate publishers model the docking owner and a competing legacy owner.
        self.zero_owner = self.create_publisher(Twist, prefix + "/docking_cmd", qos)
        self.push_owner = self.create_publisher(Twist, prefix + "/docking_cmd", qos)
        self.battery_pub = self.create_publisher(BatteryState, prefix + "/battery", qos)
        self.reverse_pub = self.create_publisher(
            Bool, prefix + "/docking_reverse_enable", qos
        )
        self.outputs = []
        self.statuses = []
        self.create_subscription(Twist, prefix + "/cmd_out", self._on_output, qos)
        self.create_subscription(String, prefix + "/safety_status", self._on_status, qos)

    def _on_output(self, msg):
        self.outputs.append((time.monotonic(), msg))

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
                self.zero_owner.get_subscription_count() > 0
                and self.push_owner.get_subscription_count() > 0
                and self.battery_pub.get_subscription_count() > 0
                and self.reverse_pub.get_subscription_count() > 0
                and self.count_publishers(self.prefix + "/cmd_out") > 0
            ):
                return
        raise RuntimeError("isolated safety subscriptions did not connect")

    def publish_for(self, publisher, message, duration, period=0.02):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            publisher.publish(message)
            rclpy.spin_once(self, timeout_sec=0.005)
            time.sleep(max(0.0, period - 0.005))

    def publish_battery_contact(self, duration=0.20):
        battery = BatteryState()
        battery.current = 1.0
        battery.voltage = 48.0
        battery.present = True
        self.publish_for(self.battery_pub, battery, duration)


rclpy.init()
node = InterlockHarness(os.environ["TEST_SAFETY_PREFIX"])
try:
    node.wait_for_connections()
    node.spin_for(0.30)

    push = Twist()
    push.linear.x = 0.05
    node.publish_for(node.push_owner, push, 0.60)
    node.spin_for(0.20)
    if not any(msg.linear.x > 0.04 for _, msg in node.outputs):
        raise RuntimeError("pre-contact docking push did not reach isolated output")

    node.publish_battery_contact()
    contact_mark = time.monotonic()

    zero = Twist()
    collision_deadline = time.monotonic() + 0.60
    while time.monotonic() < collision_deadline:
        node.zero_owner.publish(zero)
        rclpy.spin_once(node, timeout_sec=0.005)
        node.push_owner.publish(push)
        rclpy.spin_once(node, timeout_sec=0.005)
        time.sleep(0.01)

    collision_outputs = [msg for stamp, msg in node.outputs if stamp >= contact_mark]
    if not collision_outputs or any(not is_zero(msg) for msg in collision_outputs):
        raise RuntimeError(
            "BMS contact allowed nonzero output while zero and push owners competed"
        )

    # The final arbiter must remain safe even if the docking owner stops publishing.
    # The synthetic BMS sample is deliberately stale by this point; the contact
    # event itself must remain latched until an explicit undock completes.
    fallback_mark = time.monotonic() + 0.30
    node.publish_for(node.push_owner, push, 0.70)
    fallback_outputs = [msg for stamp, msg in node.outputs if stamp >= fallback_mark]
    if not fallback_outputs or any(not is_zero(msg) for msg in fallback_outputs):
        raise RuntimeError(
            "BMS contact allowed a stale/competing owner to resume forward docking motion"
        )

    reverse_enable = Bool()
    reverse_enable.data = True
    # Production undock publishes the permit during its command-settle phase
    # before sending negative X. Preserve that sequencing in this contract.
    node.publish_for(node.reverse_pub, reverse_enable, 0.35)
    reverse = Twist()
    reverse.linear.x = -0.06
    reverse_mark = time.monotonic()
    reverse_deadline = reverse_mark + 0.35
    while time.monotonic() < reverse_deadline:
        node.reverse_pub.publish(reverse_enable)
        node.push_owner.publish(reverse)
        rclpy.spin_once(node, timeout_sec=0.01)
        time.sleep(0.01)
    reverse_outputs = [msg for stamp, msg in node.outputs if stamp >= reverse_mark]
    if not any(msg.linear.x < -0.05 and abs(msg.linear.y) < 1.0e-5 and abs(msg.angular.z) < 1.0e-5
               for msg in reverse_outputs):
        raise RuntimeError("explicitly permitted pure reverse undock was blocked")

    no_contact = BatteryState()
    no_contact.current = -1.0
    no_contact.voltage = 48.0
    no_contact.present = False
    node.publish_for(node.battery_pub, no_contact, 0.20)
    reverse_disable = Bool()
    reverse_disable.data = False
    node.publish_for(node.reverse_pub, reverse_disable, 0.20)

    released_mark = time.monotonic()
    node.publish_for(node.push_owner, push, 0.35)
    released_outputs = [msg for stamp, msg in node.outputs if stamp >= released_mark]
    if not any(msg.linear.x > 0.04 for msg in released_outputs):
        raise RuntimeError("BMS interlock did not release after a confirmed undock session")

    print("PASS: pre-contact docking push is available")
    print("PASS: BMS contact blocks competing forward docking commands")
    print("PASS: BMS interlock survives stale BMS and loss of the zero-command owner")
    print("PASS: explicit pure-reverse undock remains available")
    print("PASS: fresh no-contact plus reverse-permit release clears the interlock")
finally:
    node.destroy_node()
    rclpy.shutdown()
PY

grep -E 'BMS_DOCKING_INTERLOCK|charging contact' "${log_file}" || true
