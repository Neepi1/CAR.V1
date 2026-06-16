#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="${WORKSPACE_ROOT:-$(cd "${SCRIPT_DIR}/../../../.." && pwd)}"
cd "${WORKSPACE_ROOT}"

DURATION_SEC=60
EXPECT_PASS=false

usage() {
  cat <<'USAGE'
Usage: verify_runtime_force_accept_reduction.sh [--duration-sec N] [--expect-pass]

Read-only/static contract check for Phase R1. It does not send navigation
goals, localization triggers, or velocity commands.
USAGE
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --duration-sec)
      DURATION_SEC="${2:-60}"
      shift 2
      ;;
    --expect-pass)
      EXPECT_PASS=true
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "[force-reduction] unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

case "${DURATION_SEC}" in
  ''|*[!0-9]*)
    echo "[force-reduction] --duration-sec must be an integer" >&2
    exit 2
    ;;
esac

python3 - <<'PY'
import json
import pathlib
import re
import sys

root = pathlib.Path.cwd()
api_cpp_path = root / "src/robot_api_server/src/robot_api_server_node.cpp"
api_cfg_paths = [
    root / "src/robot_api_server/config/robot_api_server.yaml",
    root / "scripts/jetson/runtime_overlay/config/robot_api_server.yaml",
]
bridge_cfg_paths = [
    root / "src/robot_localization_bridge/config/localization_bridge.yaml",
    root / "scripts/jetson/runtime_overlay/config/localization_bridge.yaml",
]
bridge_cpp_path = root / "src/robot_localization_bridge/src/localization_bridge_node.cpp"

api_cpp = api_cpp_path.read_text(encoding="utf-8")
bridge_cpp = bridge_cpp_path.read_text(encoding="utf-8")

failures = []
passes = []

def expect(cond, label):
    (passes if cond else failures).append(label)

def block(name):
    markers = (
        f"\n  HttpResponse {name}(",
        f"\n  bool {name}(",
        f"\n  void {name}(",
    )
    start = -1
    marker = ""
    for candidate in markers:
        start = api_cpp.find(candidate)
        if start >= 0:
            marker = candidate
            break
    if start < 0:
        return ""
    next_http = api_cpp.find("\n  HttpResponse ", start + len(marker))
    next_void = api_cpp.find("\n  void ", start + len(marker))
    candidates = [p for p in (next_http, next_void) if p > start]
    end = min(candidates) if candidates else len(api_cpp)
    return api_cpp[start:end]

nav_goal = block("handle_navigation_goal")
docking_job = block("run_docking_job")
trigger_endpoint = block("handle_trigger_localization")

expect("trigger_localization_and_wait_for_result(" not in nav_goal, "normal navigation goal does not call trigger_localization_and_wait_for_result")
expect("wait_for_post_relocalization_settle_barrier(" not in nav_goal, "normal navigation goal does not wait post-relocalization settle")
expect("force_relocalize is no longer executed inside normal navigation goals" in nav_goal, "force_relocalize is redirected to explicit recovery")
expect("bridge_safe_for_goal_start(\"navigation goal\"" in nav_goal, "normal navigation reads bridge safe_for_goal_start")
expect("handle_trigger_localization" in api_cpp and "trigger_localization_and_wait_for_result(" in trigger_endpoint, "manual recovery relocalization endpoint still exists")
expect("docking_relocalize_before_predock_" in docking_job, "docking recovery relocalization branches remain compiled")
expect("bridge_safe_for_goal_start(\"docking predock navigation\"" in docking_job, "docking predock checks bridge safe_for_goal_start")

for path in api_cfg_paths:
    text = path.read_text(encoding="utf-8")
    expect("navigation_relocalize_before_goal: false" in text, f"{path} normal navigation relocalization disabled")
    expect("navigation_relocalize_before_goal_required: false" in text, f"{path} normal navigation relocalization not required")
    expect("docking_relocalize_before_predock: false" in text, f"{path} before-predock relocalization disabled")
    expect("docking_relocalize_after_predock: false" in text, f"{path} after-predock relocalization disabled")
    expect("docking_relocalize_after_predock_required: false" in text, f"{path} after-predock relocalization not required")
    expect("docking_relocalize_after_fine_docking: false" in text, f"{path} after-fine relocalization disabled")

for path in bridge_cfg_paths:
    text = path.read_text(encoding="utf-8")
    expect("amcl_gate_mode: shadow" in text, f"{path} keeps AMCL mode configurable/default shadow")
    expect("amcl_input_enabled: false" in text, f"{path} keeps AMCL input available through runtime override")
    expect("map_odom_smoothing_enabled: true" in text, f"{path} bridge smoothing enabled")

expect("safe_for_goal_start" in bridge_cpp, "bridge publishes safe_for_goal_start")
expect("force_accept_service" in bridge_cpp, "force-accept recovery service remains available")
expect("tf_broadcast=true" not in api_cpp + bridge_cpp, "AMCL tf_broadcast is not enabled by API/bridge code")

print(json.dumps({
    "passes": passes,
    "failures": failures,
}, indent=2, ensure_ascii=False))
sys.exit(1 if failures else 0)
PY
