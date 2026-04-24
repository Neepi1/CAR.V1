#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/common_env.sh"

wait_for_ros_node() {
  local node_name="$1"
  local timeout_sec="${2:-15}"
  local deadline=$((SECONDS + timeout_sec))

  while (( SECONDS < deadline )); do
    if ros2 node list 2>/dev/null | grep -qx "${node_name}"; then
      return 0
    fi
    sleep 0.5
  done
  return 1
}

ensure_map_server_active() {
  local map_yaml="${1:-}"
  local timeout_sec="${2:-30}"
  local deadline=$((SECONDS + timeout_sec))

  wait_for_ros_node "/map_server" "${timeout_sec}" || {
    echo "[runtime-overlay] /map_server did not appear within ${timeout_sec}s" >&2
    return 1
  }

  if [[ -n "${map_yaml}" ]]; then
    echo "[runtime-overlay] expecting map_server asset: ${map_yaml}" >&2
  else
    local current_yaml
    current_yaml="$(ros2 param get /map_server yaml_filename 2>/dev/null || true)"
    echo "[runtime-overlay] using current map_server asset: ${current_yaml}" >&2
  fi

  while (( SECONDS < deadline )); do
    local state
    state="$(ros2 lifecycle get /map_server 2>/dev/null || true)"
    case "${state}" in
      active*)
        return 0
        ;;
      unconfigured*)
        ros2 lifecycle set /map_server configure >/dev/null 2>&1 || true
        ;;
      inactive*)
        ros2 lifecycle set /map_server activate >/dev/null 2>&1 || true
        ;;
      *)
        sleep 0.5
        ;;
    esac
    sleep 0.5
  done

  echo "[runtime-overlay] /map_server did not reach active state within ${timeout_sec}s" >&2
  ros2 lifecycle get /map_server 2>/dev/null || true
  return 1
}

wait_for_occupancy_grid() {
  local topic_name="$1"
  local timeout_sec="${2:-20}"

  python3 - "${topic_name}" "${timeout_sec}" <<'PY'
import sys
import time

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

topic_name = sys.argv[1]
timeout_sec = float(sys.argv[2])

rclpy.init()
node = rclpy.create_node("wait_for_occupancy_grid")
qos = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)
state = {}

def on_msg(msg: OccupancyGrid) -> None:
    if msg.info.width > 0 and msg.info.height > 0:
        state["msg"] = msg

node.create_subscription(OccupancyGrid, topic_name, on_msg, qos)
deadline = time.time() + timeout_sec

try:
    while rclpy.ok() and time.time() < deadline and "msg" not in state:
        rclpy.spin_once(node, timeout_sec=0.2)
finally:
    node.destroy_node()
    rclpy.shutdown()

if "msg" not in state:
    print(f"[runtime-overlay] timed out waiting for {topic_name} OccupancyGrid", file=sys.stderr)
    sys.exit(1)

msg = state["msg"]
print(
    f"[runtime-overlay] {topic_name} ready: "
    f"{msg.info.width}x{msg.info.height} @ {msg.info.resolution:.3f}, "
    f"origin=({msg.info.origin.position.x:.3f}, {msg.info.origin.position.y:.3f})",
    file=sys.stderr,
)
PY
}

wait_for_global_costmap_static() {
  local timeout_sec="${1:-35}"
  local min_cells="${2:-101}"

  python3 - "${timeout_sec}" "${min_cells}" <<'PY'
import sys
import time

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

timeout_sec = float(sys.argv[1])
min_cells = int(sys.argv[2])

rclpy.init()
node = rclpy.create_node("wait_for_global_costmap_static")
qos = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)
state = {}

def on_msg(msg: OccupancyGrid) -> None:
    if msg.info.width >= min_cells and msg.info.height >= min_cells:
        state["msg"] = msg

node.create_subscription(OccupancyGrid, "/global_costmap/costmap", on_msg, qos)
deadline = time.time() + timeout_sec

try:
    while rclpy.ok() and time.time() < deadline and "msg" not in state:
        rclpy.spin_once(node, timeout_sec=0.2)
finally:
    node.destroy_node()
    rclpy.shutdown()

if "msg" not in state:
    print("[runtime-overlay] global costmap did not resize from static map in time", file=sys.stderr)
    sys.exit(1)

msg = state["msg"]
print(
    f"[runtime-overlay] global costmap static map ready: "
    f"{msg.info.width}x{msg.info.height} @ {msg.info.resolution:.3f}, "
    f"origin=({msg.info.origin.position.x:.3f}, {msg.info.origin.position.y:.3f})",
    file=sys.stderr,
)
PY
}
