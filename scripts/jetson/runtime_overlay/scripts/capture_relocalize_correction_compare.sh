#!/usr/bin/env bash
set -euo pipefail
umask 0002

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OVERLAY_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
PROJECT_ROOT="$(cd "${OVERLAY_ROOT}/../../.." && pwd)"

TEST_DIR=""
TEST_ROOT=""
TEST_KIND=""
LABEL_FILTER=""
OUTPUT_DIR=""
REASON="odom_test_after_relocalize"
SERVICE="/global_localization/trigger"
TIMEOUT_SEC="60.0"
SETTLE_TIMEOUT_SEC="15.0"

usage() {
  cat <<'EOF'
Usage: capture_relocalize_correction_compare.sh --test-dir DIR [options]

Captures before/after odom, TF, and bridge status around one explicit global
localization trigger. It writes the map-frame correction vector and robot-frame
forward/lateral correction into summary.md and correction_metrics.json.

Options:
  --test-dir DIR            Test report directory that should receive the capture.
  --latest                  Use the newest Ranger odom test report directory.
  --kind KIND               Limit --latest to spin, straight, or s_curve.
  --test-root DIR           Limit --latest search to this report root.
  --label-filter TEXT       Limit --latest to directory names containing TEXT.
  --output-dir DIR          Exact output directory. Default: DIR/relocalize_compare_<utc>.
  --reason TEXT             Trigger reason. Default: odom_test_after_relocalize
  --service NAME            Trigger service. Default: /global_localization/trigger
  --timeout-sec SEC         Trigger/data wait timeout. Default: 60.0
  --settle-timeout-sec SEC  Wait for bridge smoothing to finish. Default: 15.0

This script does not publish velocity commands. It only calls the explicit
global localization trigger service and records diagnostics.

Examples:
  capture_relocalize_correction_compare.sh --latest
  capture_relocalize_correction_compare.sh --latest --kind s_curve
  capture_relocalize_correction_compare.sh --test-dir reports/ranger_s_curve_odom_test/20260620T000000Z_s_curve_10m_left_01
EOF
}

while [[ "$#" -gt 0 ]]; do
  case "$1" in
    --test-dir)
      TEST_DIR="${2:-}"
      shift 2
      ;;
    --latest)
      TEST_DIR="latest"
      shift
      ;;
    --kind|--latest-kind)
      TEST_KIND="${2:-}"
      shift 2
      ;;
    --test-root|--latest-root)
      TEST_ROOT="${2:-}"
      shift 2
      ;;
    --label-filter)
      LABEL_FILTER="${2:-}"
      shift 2
      ;;
    --output-dir)
      OUTPUT_DIR="${2:-}"
      shift 2
      ;;
    --reason)
      REASON="${2:-}"
      shift 2
      ;;
    --service)
      SERVICE="${2:-}"
      shift 2
      ;;
    --timeout-sec)
      TIMEOUT_SEC="${2:-}"
      shift 2
      ;;
    --settle-timeout-sec)
      SETTLE_TIMEOUT_SEC="${2:-}"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "[relocalize-capture] unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

if [[ -f "${SCRIPT_DIR}/common_env.sh" ]]; then
  # shellcheck source=/dev/null
  source "${SCRIPT_DIR}/common_env.sh"
fi
PROJECT_ROOT="${NJRH_PROJECT_ROOT:-${PROJECT_ROOT}}"

abs_under_project() {
  local path="$1"
  if [[ "${path}" == /* ]]; then
    printf '%s\n' "${path}"
  else
    printf '%s/%s\n' "${PROJECT_ROOT}" "${path}"
  fi
}

default_roots_for_kind() {
  case "${TEST_KIND}" in
    ""|all)
      printf '%s\n' \
        "${PROJECT_ROOT}/reports/ranger_left_arc_odom_test" \
        "${PROJECT_ROOT}/reports/ranger_right_arc_odom_test" \
        "${PROJECT_ROOT}/reports/ranger_s_curve_odom_test" \
        "${PROJECT_ROOT}/reports/ranger_straight_odom_test" \
        "${PROJECT_ROOT}/reports/ranger_spin_odom_test"
      ;;
    s_curve|s-curve|scurve|s)
      printf '%s\n' "${PROJECT_ROOT}/reports/ranger_s_curve_odom_test"
      ;;
    left_arc|left-arc|left|arc_left|l)
      printf '%s\n' "${PROJECT_ROOT}/reports/ranger_left_arc_odom_test"
      ;;
    right_arc|right-arc|right|arc_right|r)
      printf '%s\n' "${PROJECT_ROOT}/reports/ranger_right_arc_odom_test"
      ;;
    arc|arcs)
      printf '%s\n' \
        "${PROJECT_ROOT}/reports/ranger_left_arc_odom_test" \
        "${PROJECT_ROOT}/reports/ranger_right_arc_odom_test"
      ;;
    straight|line)
      printf '%s\n' "${PROJECT_ROOT}/reports/ranger_straight_odom_test"
      ;;
    spin|rotate|rotation)
      printf '%s\n' "${PROJECT_ROOT}/reports/ranger_spin_odom_test"
      ;;
    *)
      echo "[relocalize-capture] unknown --kind: ${TEST_KIND}" >&2
      exit 2
      ;;
  esac
}

resolve_latest_test_dir() {
  local roots=()
  if [[ -n "${TEST_ROOT}" ]]; then
    roots+=("$(abs_under_project "${TEST_ROOT}")")
  else
    while IFS= read -r root; do
      roots+=("${root}")
    done < <(default_roots_for_kind)
  fi

  local best_name=""
  local best_path=""
  local root candidate name
  for root in "${roots[@]}"; do
    [[ -d "${root}" ]] || continue
    while IFS= read -r -d '' candidate; do
      name="$(basename "${candidate}")"
      [[ "${name}" == relocalize_compare_* ]] && continue
      if [[ -n "${LABEL_FILTER}" && "${name}" != *"${LABEL_FILTER}"* ]]; then
        continue
      fi
      if [[ -z "${best_name}" || "${name}" > "${best_name}" ]]; then
        best_name="${name}"
        best_path="${candidate}"
      fi
    done < <(find "${root}" -mindepth 1 -maxdepth 1 -type d -print0 2>/dev/null)
  done

  if [[ -z "${best_path}" ]]; then
    echo "[relocalize-capture] no matching latest test directory found" >&2
    echo "[relocalize-capture] kind=${TEST_KIND:-all} test_root=${TEST_ROOT:-default} label_filter=${LABEL_FILTER:-}" >&2
    exit 2
  fi

  TEST_DIR="${best_path}"
  echo "[relocalize-capture] latest test_dir: ${TEST_DIR#${PROJECT_ROOT}/}"
}

if [[ "${TEST_DIR}" == "latest" ]]; then
  resolve_latest_test_dir
fi

if [[ -z "${TEST_DIR}" && -z "${OUTPUT_DIR}" ]]; then
  echo "[relocalize-capture] --test-dir is required when --output-dir is not set" >&2
  usage >&2
  exit 2
fi

if [[ -n "${TEST_DIR}" && "${TEST_DIR}" != /* ]]; then
  TEST_DIR="$(abs_under_project "${TEST_DIR}")"
fi
if [[ -z "${OUTPUT_DIR}" ]]; then
  timestamp="$(date -u +%Y%m%dT%H%M%SZ)"
  OUTPUT_DIR="${TEST_DIR}/relocalize_compare_${timestamp}"
elif [[ "${OUTPUT_DIR}" != /* ]]; then
  OUTPUT_DIR="${PROJECT_ROOT}/${OUTPUT_DIR}"
fi

mkdir -p "${OUTPUT_DIR}"

{
  echo "# Relocalize Correction Capture Environment"
  echo "- timestamp_utc: $(date -u +%Y%m%dT%H%M%SZ)"
  echo "- test_dir: ${TEST_DIR}"
  echo "- output_dir: ${OUTPUT_DIR}"
  echo "- reason: ${REASON}"
  echo "- service: ${SERVICE}"
  echo "- timeout_sec: ${TIMEOUT_SEC}"
  echo "- settle_timeout_sec: ${SETTLE_TIMEOUT_SEC}"
  echo "- rmw: ${RMW_IMPLEMENTATION:-}"
  echo
  echo "## ROS Nodes"
  ros2 node list 2>&1 || true
  echo
  echo "## Topic Info"
  for topic in \
    /localization/bridge_status \
    /wheel/odom \
    /local_state/odometry \
    /tf; do
    echo "### ${topic}"
    ros2 topic info "${topic}" -v 2>&1 || true
  done
  echo
  echo "## Services"
  ros2 service list -t 2>&1 | grep -E 'global_localization|robot_localization_bridge|trigger_grid' || true
} >"${OUTPUT_DIR}/environment.md"

python3 - \
  "${OUTPUT_DIR}" \
  "${REASON}" \
  "${SERVICE}" \
  "${TIMEOUT_SEC}" \
  "${SETTLE_TIMEOUT_SEC}" <<'PY'
import json
import math
import os
import sys
import time
from typing import Any, Dict, Optional, Tuple

import rclpy
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

from robot_interfaces.srv import TriggerLocalization


out_dir = sys.argv[1]
reason = sys.argv[2]
service_name = sys.argv[3]
timeout_sec = max(float(sys.argv[4]), 1.0)
settle_timeout_sec = max(float(sys.argv[5]), 0.0)


def now_iso() -> str:
    return time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())


def yaw_from_quat(q: Any) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def norm_angle(value: float) -> float:
    return math.atan2(math.sin(value), math.cos(value))


def pose_delta(
    before: Optional[Dict[str, float]],
    after: Optional[Dict[str, float]],
) -> Optional[Dict[str, float]]:
    if before is None or after is None:
        return None
    dx = after["x"] - before["x"]
    dy = after["y"] - before["y"]
    dyaw = norm_angle(after["yaw_rad"] - before["yaw_rad"])
    c = math.cos(before["yaw_rad"])
    s = math.sin(before["yaw_rad"])
    return {
        "dx_map_m": dx,
        "dy_map_m": dy,
        "translation_m": math.hypot(dx, dy),
        "dyaw_rad": dyaw,
        "dyaw_deg": math.degrees(dyaw),
        "forward_m_in_before_frame": dx * c + dy * s,
        "left_m_in_before_frame": -dx * s + dy * c,
    }


def odom_to_dict(msg: Optional[Odometry]) -> Optional[Dict[str, Any]]:
    if msg is None:
        return None
    pose = msg.pose.pose
    twist = msg.twist.twist
    return {
        "stamp_sec": msg.header.stamp.sec,
        "stamp_nanosec": msg.header.stamp.nanosec,
        "frame_id": msg.header.frame_id,
        "child_frame_id": msg.child_frame_id,
        "pose": {
            "x": pose.position.x,
            "y": pose.position.y,
            "z": pose.position.z,
            "qx": pose.orientation.x,
            "qy": pose.orientation.y,
            "qz": pose.orientation.z,
            "qw": pose.orientation.w,
            "yaw_rad": yaw_from_quat(pose.orientation),
            "yaw_deg": math.degrees(yaw_from_quat(pose.orientation)),
        },
        "twist": {
            "linear_x": twist.linear.x,
            "linear_y": twist.linear.y,
            "angular_z": twist.angular.z,
        },
    }


def transform_to_pose_dict(transform_msg: Any) -> Dict[str, float]:
    t = transform_msg.transform.translation
    q = transform_msg.transform.rotation
    yaw = yaw_from_quat(q)
    return {
        "x": t.x,
        "y": t.y,
        "z": t.z,
        "qx": q.x,
        "qy": q.y,
        "qz": q.z,
        "qw": q.w,
        "yaw_rad": yaw,
        "yaw_deg": math.degrees(yaw),
    }


class CaptureNode(Node):
    def __init__(self) -> None:
        super().__init__("capture_relocalize_correction_compare")
        qos = QoSProfile(depth=50)
        telemetry_qos = QoSProfile(depth=50)
        telemetry_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self.bridge_status_raw = ""
        self.bridge_status: Dict[str, Any] = {}
        self.wheel_odom: Optional[Odometry] = None
        self.local_odom: Optional[Odometry] = None
        self.create_subscription(String, "/localization/bridge_status", self._bridge_cb, qos)
        self.create_subscription(Odometry, "/wheel/odom", self._wheel_cb, telemetry_qos)
        self.create_subscription(Odometry, "/local_state/odometry", self._local_cb, telemetry_qos)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.trigger_client = self.create_client(TriggerLocalization, service_name)

    def _bridge_cb(self, msg: String) -> None:
        self.bridge_status_raw = msg.data
        try:
            self.bridge_status = json.loads(msg.data)
        except Exception:
            self.bridge_status = {}

    def _wheel_cb(self, msg: Odometry) -> None:
        self.wheel_odom = msg

    def _local_cb(self, msg: Odometry) -> None:
        self.local_odom = msg

    def spin_until(self, predicate, timeout: float, period: float = 0.05) -> bool:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=period)
            if predicate():
                return True
        return bool(predicate())

    def lookup_pose(self, target: str, source: str, timeout: float = 2.0) -> Optional[Dict[str, float]]:
        deadline = time.monotonic() + timeout
        last_error = ""
        while time.monotonic() < deadline and rclpy.ok():
            try:
                tf_msg = self.tf_buffer.lookup_transform(
                    target,
                    source,
                    rclpy.time.Time(),
                    timeout=Duration(seconds=0.2),
                )
                return transform_to_pose_dict(tf_msg)
            except TransformException as exc:
                last_error = str(exc)
                rclpy.spin_once(self, timeout_sec=0.05)
        return {"error": last_error}

    def snapshot(self, label: str) -> Dict[str, Any]:
        self.spin_until(
            lambda: bool(self.bridge_status) and self.wheel_odom is not None,
            timeout=3.0,
        )
        snap = {
            "label": label,
            "time_utc": now_iso(),
            "bridge_status_raw": self.bridge_status_raw,
            "bridge_status": self.bridge_status,
            "wheel_odom": odom_to_dict(self.wheel_odom),
            "local_odom": odom_to_dict(self.local_odom),
            "tf": {
                "map_base_link": self.lookup_pose("map", "base_link"),
                "map_odom": self.lookup_pose("map", "odom"),
                "odom_base_link": self.lookup_pose("odom", "base_link"),
            },
        }
        return snap

    def call_trigger(self) -> Dict[str, Any]:
        if not self.trigger_client.wait_for_service(timeout_sec=timeout_sec):
            return {
                "accepted": False,
                "message": f"service unavailable: {service_name}",
                "return_code": 2,
            }
        req = TriggerLocalization.Request()
        req.reason = reason
        future = self.trigger_client.call_async(req)
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline and rclpy.ok() and not future.done():
            rclpy.spin_once(self, timeout_sec=0.1)
        if not future.done():
            return {
                "accepted": False,
                "message": f"request timeout after {timeout_sec:.1f}s",
                "return_code": 124,
            }
        res = future.result()
        return {
            "accepted": bool(res.accepted),
            "message": str(res.message),
            "return_code": 0 if res.accepted else 1,
        }


def write_json(name: str, data: Any) -> None:
    with open(os.path.join(out_dir, name), "w", encoding="utf-8") as f:
        json.dump(data, f, indent=2, sort_keys=True)
        f.write("\n")


def tf_pose(snapshot: Dict[str, Any], key: str) -> Optional[Dict[str, float]]:
    value = (snapshot.get("tf") or {}).get(key)
    if not isinstance(value, dict) or "error" in value:
        return None
    return value


def odom_pose(snapshot: Dict[str, Any], key: str) -> Optional[Dict[str, float]]:
    value = snapshot.get(key)
    if not isinstance(value, dict):
        return None
    pose = value.get("pose")
    return pose if isinstance(pose, dict) else None


def main() -> int:
    rclpy.init(args=None)
    node = CaptureNode()
    try:
        before = node.snapshot("before")
        before_seq = int((before.get("bridge_status") or {}).get("last_explicit_relocalization_sequence", -1))
        before_accept_time = float((before.get("bridge_status") or {}).get("last_correction_accept_time", 0.0) or 0.0)

        trigger = node.call_trigger()

        def bridge_updated() -> bool:
            status = node.bridge_status
            seq = int(status.get("last_explicit_relocalization_sequence", -1) or -1)
            accept_time = float(status.get("last_correction_accept_time", 0.0) or 0.0)
            return seq > before_seq or accept_time > before_accept_time

        node.spin_until(bridge_updated, timeout=min(timeout_sec, 20.0))

        def bridge_settled() -> bool:
            status = node.bridge_status
            if not status:
                return False
            remaining_xy = abs(float(status.get("remaining_translation_error_m", 0.0) or 0.0))
            remaining_yaw = abs(float(status.get("remaining_yaw_error_rad", 0.0) or 0.0))
            correction_active = bool(status.get("correction_active", False))
            current = status.get("current_sequence", status.get("map_odom_last_published_sequence"))
            target = status.get("target_sequence", status.get("map_odom_latest_accepted_sequence"))
            return (not correction_active) and remaining_xy <= 0.005 and remaining_yaw <= 0.005 and current == target

        if settle_timeout_sec > 0.0:
            node.spin_until(bridge_settled, timeout=settle_timeout_sec)

        after = node.snapshot("after")

        map_base_delta = pose_delta(tf_pose(before, "map_base_link"), tf_pose(after, "map_base_link"))
        map_odom_delta = pose_delta(tf_pose(before, "map_odom"), tf_pose(after, "map_odom"))
        odom_base_delta = pose_delta(tf_pose(before, "odom_base_link"), tf_pose(after, "odom_base_link"))
        wheel_delta = pose_delta(odom_pose(before, "wheel_odom"), odom_pose(after, "wheel_odom"))
        local_delta = pose_delta(odom_pose(before, "local_odom"), odom_pose(after, "local_odom"))

        after_status = after.get("bridge_status") or {}
        metrics = {
            "time_utc": now_iso(),
            "reason": reason,
            "trigger": trigger,
            "map_base_link_delta": map_base_delta,
            "map_odom_delta": map_odom_delta,
            "odom_base_link_delta": odom_base_delta,
            "wheel_odom_delta": wheel_delta,
            "local_odom_delta": local_delta,
            "bridge": {
                "amcl_gate_mode": after_status.get("amcl_gate_mode"),
                "last_accept_reason": after_status.get("last_accept_reason"),
                "last_correction_source": after_status.get("last_correction_source"),
                "last_correction_delta_translation_m": after_status.get("last_correction_delta_translation_m"),
                "last_correction_delta_yaw_rad": after_status.get("last_correction_delta_yaw_rad"),
                "last_accepted_correction_translation_m": after_status.get("last_accepted_correction_translation_m"),
                "last_accepted_correction_yaw_rad": after_status.get("last_accepted_correction_yaw_rad"),
                "last_candidate_correction_translation_m": after_status.get("last_candidate_correction_translation_m"),
                "last_candidate_correction_yaw_rad": after_status.get("last_candidate_correction_yaw_rad"),
                "current_source": after_status.get("current_source"),
                "has_map_to_odom": after_status.get("has_map_to_odom"),
                "correction_paused": after_status.get("correction_paused"),
                "correction_active": after_status.get("correction_active"),
                "safe_for_goal_start": after_status.get("safe_for_goal_start"),
                "remaining_translation_error_m": after_status.get("remaining_translation_error_m"),
                "remaining_yaw_error_rad": after_status.get("remaining_yaw_error_rad"),
                "last_explicit_relocalization_sequence": after_status.get("last_explicit_relocalization_sequence"),
            },
        }

        write_json("before_snapshot.json", before)
        write_json("after_snapshot.json", after)
        write_json("relocalize_call.json", trigger)
        write_json("correction_metrics.json", metrics)

        with open(os.path.join(out_dir, "relocalize_call.txt"), "w", encoding="utf-8") as f:
            f.write(f"accepted: {'true' if trigger.get('accepted') else 'false'}\n")
            f.write(f"message: {trigger.get('message', '')}\n")

        def md_delta(delta: Optional[Dict[str, float]]) -> str:
            if delta is None:
                return "| unavailable | | | | | | |\n"
            return (
                f"| `{delta['dx_map_m']:.4f}` | `{delta['dy_map_m']:.4f}` | "
                f"`{delta['translation_m']:.4f}` | `{delta['dyaw_deg']:.3f}` | "
                f"`{delta['forward_m_in_before_frame']:.4f}` | "
                f"`{delta['left_m_in_before_frame']:.4f}` |\n"
            )

        with open(os.path.join(out_dir, "summary.md"), "w", encoding="utf-8") as f:
            f.write("# Relocalize Correction Compare\n\n")
            f.write(f"- reason: `{reason}`\n")
            f.write(f"- trigger_accepted: `{str(trigger.get('accepted')).lower()}`\n")
            f.write(f"- trigger_message: `{trigger.get('message', '')}`\n")
            f.write(f"- bridge_last_correction_delta_translation_m: `{metrics['bridge'].get('last_correction_delta_translation_m')}`\n")
            f.write(f"- bridge_last_correction_delta_yaw_rad: `{metrics['bridge'].get('last_correction_delta_yaw_rad')}`\n")
            f.write(f"- bridge_safe_for_goal_start: `{metrics['bridge'].get('safe_for_goal_start')}`\n")
            f.write(f"- bridge_correction_paused: `{metrics['bridge'].get('correction_paused')}`\n")
            f.write("\n")
            f.write("Positive `forward_m_in_before_frame` means the accepted relocalization moved `map->base_link` forward along the robot heading captured before relocalization. Positive `left_m_in_before_frame` means it moved left in the same start frame.\n\n")
            f.write("| delta source | dx_map_m | dy_map_m | translation_m | dyaw_deg | forward_m_in_before_frame | left_m_in_before_frame |\n")
            f.write("|---|---:|---:|---:|---:|---:|---:|\n")
            f.write("| map->base_link | " + md_delta(map_base_delta).lstrip("| "))
            f.write("| map->odom | " + md_delta(map_odom_delta).lstrip("| "))
            f.write("| odom->base_link | " + md_delta(odom_base_delta).lstrip("| "))
            f.write("| /wheel/odom pose | " + md_delta(wheel_delta).lstrip("| "))
            f.write("| /local_state/odometry pose | " + md_delta(local_delta).lstrip("| "))

        print(f"[relocalize-capture] output: {out_dir}")
        return int(trigger.get("return_code", 1))
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
PY

rc=$?
tar -czf "${OUTPUT_DIR}.tgz" -C "$(dirname "${OUTPUT_DIR}")" "$(basename "${OUTPUT_DIR}")"
echo "capture_dir=${OUTPUT_DIR#${PROJECT_ROOT}/}"
echo "archive=${OUTPUT_DIR#${PROJECT_ROOT}/}.tgz"
exit "${rc}"
