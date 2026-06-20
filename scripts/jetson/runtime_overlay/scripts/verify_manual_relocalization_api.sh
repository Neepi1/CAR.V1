#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="${WORKSPACE_ROOT:-$(cd "${SCRIPT_DIR}/../../../.." && pwd)}"
if [[ -f "${SCRIPT_DIR}/common_env.sh" ]]; then
  # shellcheck source=common_env.sh
  source "${SCRIPT_DIR}/common_env.sh"
fi

API_URL="${API_URL:-http://127.0.0.1:8080}"
OUTPUT_DIR=""
TIMEOUT_SEC=20
WAIT_FOR_SETTLE=false
PREFIX="[manual-relocalization-verify]"

usage() {
  cat <<'EOF'
Usage:
  verify_manual_relocalization_api.sh [--wait-for-settle] [--timeout-sec N]

Calls POST /api/v1/localization/trigger and verifies the Phase V1 contract:
default success is bridge correction acceptance, not post-relocalization settle.
This script does not send motion commands and does not call Nav2 actions.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --wait-for-settle)
      WAIT_FOR_SETTLE=true
      shift
      ;;
    --timeout-sec)
      TIMEOUT_SEC="${2:-}"
      shift 2
      ;;
    --api-url)
      API_URL="${2:-}"
      shift 2
      ;;
    --output-dir)
      OUTPUT_DIR="${2:-}"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "${PREFIX} FAIL unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

if ! [[ "${TIMEOUT_SEC}" =~ ^[0-9]+$ ]] || [[ "${TIMEOUT_SEC}" -lt 5 ]]; then
  echo "${PREFIX} FAIL --timeout-sec must be an integer >= 5" >&2
  exit 2
fi

TIMESTAMP="$(date -u +%Y%m%dT%H%M%SZ)"
if [[ -z "${OUTPUT_DIR}" ]]; then
  OUTPUT_DIR="${WORKSPACE_ROOT}/reports/manual_relocalization_api_${TIMESTAMP}"
fi
mkdir -p "${OUTPUT_DIR}"

python3 - "${API_URL}" "${OUTPUT_DIR}" "${TIMEOUT_SEC}" "${WAIT_FOR_SETTLE}" <<'PY'
import json
import math
import sys
import time
import urllib.request
from datetime import datetime, timezone
from pathlib import Path

api_url = sys.argv[1].rstrip("/")
out_dir = Path(sys.argv[2])
timeout_sec = int(sys.argv[3])
wait_for_settle = sys.argv[4].lower() == "true"

try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
except Exception as exc:  # pragma: no cover
    rclpy = None
    ROS_IMPORT_ERROR = repr(exc)
else:
    ROS_IMPORT_ERROR = ""


def now_iso():
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


def http_json(method, path, body=None, timeout=10.0):
    data = None
    headers = {"Content-Type": "application/json"}
    if body is not None:
        data = json.dumps(body).encode("utf-8")
    req = urllib.request.Request(f"{api_url}{path}", data=data, headers=headers, method=method)
    try:
        with urllib.request.urlopen(req, timeout=timeout) as resp:
            raw = resp.read().decode("utf-8")
            return resp.status, json.loads(raw) if raw else {}
    except Exception as exc:
        return 0, {"ok": False, "error": str(exc)}


class BridgeProbe(Node):
    def __init__(self):
        super().__init__("verify_manual_relocalization_api_probe")
        self.samples = []
        self.create_subscription(String, "/localization/bridge_status", self.on_status, 10)

    def on_status(self, msg):
        try:
            data = json.loads(msg.data)
        except Exception:
            return
        data["_observed_at"] = now_iso()
        self.samples.append(data)


def seq(sample):
    for key in ("last_accepted_sequence", "current_sequence", "target_sequence"):
        try:
            return int(sample.get(key))
        except Exception:
            continue
    return -1


node = None
if rclpy is not None:
    rclpy.init()
    node = BridgeProbe()
    end = time.monotonic() + 2.0
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.1)

before = node.samples[-1] if node and node.samples else {}
trigger_start = now_iso()
start_mono = time.monotonic()
http_status, response = http_json(
    "POST",
    "/api/v1/localization/trigger",
    {"wait_for_settle": wait_for_settle},
    timeout=timeout_sec,
)
trigger_response = now_iso()

deadline = time.monotonic() + timeout_sec
while node is not None and time.monotonic() < deadline:
    rclpy.spin_once(node, timeout_sec=0.1)
    latest = node.samples[-1] if node.samples else {}
    if seq(latest) > seq(before) and latest.get("map_to_odom_publisher_owner") == "robot_localization_bridge":
        if not latest.get("correction_active", False):
            break

after = node.samples[-1] if node and node.samples else {}
if node is not None:
    node.destroy_node()
    rclpy.shutdown()

status_code, status = http_json("GET", "/api/v1/status", timeout=2.0)
localization = status.get("localization") if isinstance(status, dict) else {}
if not isinstance(localization, dict):
    localization = {}

raw = {
    "trigger_start_time": trigger_start,
    "trigger_response_time": trigger_response,
    "trigger_duration_sec": time.monotonic() - start_mono,
    "http_status": http_status,
    "response": response,
    "wait_for_settle_requested": wait_for_settle,
    "bridge_status_before": before,
    "bridge_status_after": after,
    "bridge_status_samples": node.samples if node else [],
    "api_status_after": status,
    "ros_import_error": ROS_IMPORT_ERROR,
}
(out_dir / "raw.json").write_text(json.dumps(raw, indent=2, ensure_ascii=False, sort_keys=True) + "\n", encoding="utf-8")

post_settle_requested = response.get("post_relocalization_settle_requested")
if post_settle_requested is None:
    post_settle_requested = response.get("post_relocalization_settle", {}).get("requested")

owner = after.get("map_to_odom_publisher_owner") or localization.get("map_to_odom_publisher_owner", "")
safe = after.get("safe_for_goal_start", localization.get("safe_for_goal_start", ""))
active = after.get("correction_active", localization.get("correction_active", ""))
accepted_before = seq(before)
accepted_after = seq(after)
accepted = accepted_after > accepted_before or "bridge accepted" in json.dumps(response).lower()

failures = []
passes = []
if http_status == 200 and response.get("ok") is True:
    passes.append("HTTP ok=true")
else:
    failures.append(f"HTTP status={http_status} ok={response.get('ok')}")
if not wait_for_settle and post_settle_requested is False:
    passes.append("default post_relocalization_settle_requested=false")
elif not wait_for_settle:
    failures.append(f"default post_relocalization_settle_requested={post_settle_requested}")
if accepted:
    passes.append("bridge accepted correction or API reported bridge acceptance")
else:
    failures.append("bridge acceptance was not observed")
if owner == "robot_localization_bridge":
    passes.append("map->odom owner is robot_localization_bridge")
else:
    failures.append(f"map->odom owner is {owner!r}")

summary = [
    "# Manual Relocalization API Verification",
    "",
    f"- report_dir: `{out_dir}`",
    f"- trigger_start_time: `{trigger_start}`",
    f"- trigger_response_time: `{trigger_response}`",
    f"- http_status: `{http_status}`",
    f"- ok: `{response.get('ok')}`",
    f"- wait_for_settle_requested: `{wait_for_settle}`",
    f"- post_relocalization_settle_requested: `{post_settle_requested}`",
    f"- bridge_accepted_sequence_before: `{accepted_before}`",
    f"- bridge_accepted_sequence_after: `{accepted_after}`",
    f"- correction_active: `{active}`",
    f"- safe_for_goal_start: `{safe}`",
    f"- map_odom_owner: `{owner}`",
    f"- map_odom_publish_loop_hz: `{after.get('map_odom_publish_loop_hz', '')}`",
    f"- map_odom_publish_gap_max_ms: `{after.get('map_odom_publish_gap_ms', '')}`",
    f"- localization_result_age: `{localization.get('last_result_age_sec', '')}`",
    f"- failure_reason: `{response.get('failure_reason', response.get('detail', ''))}`",
    "",
    "## Verdict",
    f"- result: `{'FAIL' if failures else 'PASS'}`",
    f"- passes: `{passes}`",
    f"- failures: `{failures}`",
]
(out_dir / "summary.md").write_text("\n".join(summary) + "\n", encoding="utf-8")
print(f"summary={out_dir / 'summary.md'}")
sys.exit(1 if failures else 0)
PY

echo "${PREFIX} wrote ${OUTPUT_DIR}"
