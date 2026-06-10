#!/usr/bin/env bash
set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="${WORKSPACE_ROOT:-$(cd "${SCRIPT_DIR}/../../../.." && pwd)}"
export NJRH_PROJECT_ROOT="${NJRH_PROJECT_ROOT:-${WORKSPACE_ROOT}}"
# shellcheck source=common_env.sh
source "${SCRIPT_DIR}/common_env.sh"
set +e

API_URL="${API_URL:-http://127.0.0.1:8080}"
DURATION_SEC=12
OUTPUT_DIR=""
EXECUTE_UNDOCK=false
PREFIX="[undock-logic]"

usage() {
  cat <<'EOF'
Usage:
  bash scripts/jetson/runtime_overlay/scripts/diagnose_undock_logic_and_no_motion.sh --dry-run
  bash scripts/jetson/runtime_overlay/scripts/diagnose_undock_logic_and_no_motion.sh --execute-undock --duration-sec 12

Default mode is --dry-run. It checks static config, source-code contracts,
API dock/latch state, and ROS topic topology without moving the robot.

--execute-undock calls the controlled API endpoint POST /api/v1/docking/undock
and records the normal docking command chain. It never publishes /cmd_vel,
/cmd_vel_safe, or any direct velocity command.

Options:
  --dry-run             Static/API/ROS graph checks only. Default.
  --execute-undock      Call POST /api/v1/docking/undock and record one attempt.
  --duration-sec N      Runtime capture duration for --execute-undock. Default: 12.
  --api-url URL         robot_api_server base URL. Default: http://127.0.0.1:8080.
  --output-dir DIR      Report directory. Default: reports/undock_logic/<utc timestamp>.
  -h, --help            Show this help.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --dry-run)
      EXECUTE_UNDOCK=false
      shift
      ;;
    --execute-undock)
      EXECUTE_UNDOCK=true
      shift
      ;;
    --duration-sec)
      DURATION_SEC="${2:-}"
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

if ! [[ "${DURATION_SEC}" =~ ^[0-9]+$ ]] || [[ "${DURATION_SEC}" -lt 5 ]]; then
  echo "${PREFIX} FAIL --duration-sec must be an integer >= 5" >&2
  exit 2
fi

TIMESTAMP="$(date -u +%Y%m%dT%H%M%SZ)"
if [[ -z "${OUTPUT_DIR}" ]]; then
  OUTPUT_DIR="${NJRH_PROJECT_ROOT}/reports/undock_logic/${TIMESTAMP}"
fi
mkdir -p "${OUTPUT_DIR}"

PIDS=()

cleanup() {
  local pid
  for pid in "${PIDS[@]:-}"; do
    if kill -0 "${pid}" 2>/dev/null; then
      kill "${pid}" 2>/dev/null || true
    fi
  done
}
trap cleanup EXIT INT TERM

run_logged() {
  local name="$1"
  shift
  {
    echo "\$ $*"
    "$@"
  } >"${OUTPUT_DIR}/${name}.log" 2>&1
}

start_logged() {
  local name="$1"
  shift
  {
    echo "\$ $*"
    "$@"
  } >"${OUTPUT_DIR}/${name}.log" 2>&1 &
  PIDS+=("$!")
}

start_timed_echo() {
  local name="$1"
  local topic="$2"
  shift 2
  start_logged "${name}" timeout --signal=INT "${DURATION_SEC}" bash -lc \
    "ros2 topic echo ${topic} $* | awk '{ print systime(), \$0; fflush(); }'"
}

curl_json() {
  local method="$1"
  local path="$2"
  local body="${3:-{}}"
  if [[ -n "${ROBOT_API_TOKEN:-}" ]]; then
    curl -fsS -X "${method}" "${API_URL}${path}" \
      -H "Content-Type: application/json" \
      -H "X-Robot-Token: ${ROBOT_API_TOKEN}" \
      --data "${body}"
  else
    curl -fsS -X "${method}" "${API_URL}${path}" \
      -H "Content-Type: application/json" \
      --data "${body}"
  fi
}

api_snapshot() {
  local name="$1"
  {
    echo "GET ${API_URL}/api/v1/status"
    curl -fsS "${API_URL}/api/v1/status" || true
    echo
    echo "GET ${API_URL}/api/v1/navigation/pre_goal_check"
    curl -fsS "${API_URL}/api/v1/navigation/pre_goal_check" || true
    echo
    echo "GET ${API_URL}/api/v1/docking/state"
    curl -fsS "${API_URL}/api/v1/docking/state" || true
    echo
  } >"${OUTPUT_DIR}/${name}.json" 2>"${OUTPUT_DIR}/${name}.err"
}

write_static_audit() {
  python3 - "${NJRH_PROJECT_ROOT}" <<'PY' >"${OUTPUT_DIR}/static_audit.md" 2>/dev/null || true
import pathlib
import re
import sys

root = pathlib.Path(sys.argv[1])
paths = {
    "docking_overlay": root / "scripts/jetson/runtime_overlay/config/docking.yaml",
    "docking_pkg": root / "src/robot_nav_config/config/docking.yaml",
    "api_overlay": root / "scripts/jetson/runtime_overlay/config/robot_api_server.yaml",
    "nav2_overlay": root / "scripts/jetson/runtime_overlay/config/nav2.yaml",
    "docking_cpp": root / "src/robot_docking_manager/src/docking_manager_node.cpp",
    "safety_cpp": root / "src/robot_safety/src/robot_safety_node.cpp",
    "mode_cpp": root / "src/ranger_mini3_mode_controller/src/mode_controller_node.cpp",
}

def text(name):
    try:
        return paths[name].read_text(encoding="utf-8")
    except Exception:
        return ""

def scalar(cfg, key):
    m = re.search(rf"^\s*{re.escape(key)}:\s*([-+]?\d+(?:\.\d+)?)\s*$", cfg, re.M)
    return float(m.group(1)) if m else None

docking = text("docking_overlay")
nav2 = text("nav2_overlay")
docking_cpp = text("docking_cpp")
safety_cpp = text("safety_cpp")
mode_cpp = text("mode_cpp")

distance = scalar(docking, "distance_m")
speed = scalar(docking, "speed_mps")
timeout = scalar(docking, "timeout_s")
settle = scalar(docking, "command_settle_s")
motion_start = scalar(docking, "motion_start_timeout_s")
no_progress = scalar(docking, "no_progress_timeout_s")
epsilon = scalar(docking, "progress_epsilon_m")
vx_min = scalar(nav2, "vx_min")

required_timeout = None
if None not in (distance, speed, settle, motion_start) and speed > 0:
    required_timeout = settle + motion_start + distance / speed + 2.0

print("# Undock Static Logic Audit")
print()
print("| item | value | result |")
print("| --- | --- | --- |")
for key, value in (
    ("undock.distance_m", distance),
    ("undock.speed_mps", speed),
    ("undock.timeout_s", timeout),
    ("undock.command_settle_s", settle),
    ("undock.motion_start_timeout_s", motion_start),
    ("undock.no_progress_timeout_s", no_progress),
    ("undock.progress_epsilon_m", epsilon),
    ("nav2.vx_min", vx_min),
):
    print(f"| {key} | {value} | recorded |")
if required_timeout is not None:
    ok = timeout is not None and timeout >= required_timeout
    print(f"| timeout_budget_min | {required_timeout:.3f} | {'PASS' if ok else 'FAIL'} |")

checks = [
    ("docking_manager reads persistent latch", "dock_contact_latch_is_docked()" in docking_cpp),
    ("docking_manager accepts latch in start_undocking", "dock_latch_detected" in docking_cpp and "!dock_latch_detected" in docking_cpp),
    ("first motion timeout exists", "undock_failed_motion_start_timeout" in docking_cpp),
    ("true no-command failure exists", "undock_failed_no_command_published" in docking_cpp),
    ("waiting-first-motion status exposes command evidence", "undocking waiting_first_motion" in docking_cpp and "cmd_count=" in docking_cpp and "cmd_x=" in docking_cpp),
    ("after-motion no-progress exists", "undock_failed_no_progress" in docking_cpp),
    ("no old no-motion failure string", "undock_failed_no_motion" not in docking_cpp),
    ("docking path remains /cmd_vel_docking", "/cmd_vel_docking" in docking and "/cmd_vel_docking" in safety_cpp),
    ("robot_safety allows docking cmd context", "current_snapshot(docking_command)" in safety_cpp and "allow_docking_cmd_when_docked_" in safety_cpp),
    ("robot_safety holds fresh docking command", "last_docking_cmd_" in safety_cpp and "fresh_docking_command_active()" in safety_cpp and "publish_command(last_docking_cmd_, snapshot)" in safety_cpp),
    ("mode controller fresh reverse permit", "effectiveAllowReverse" in mode_cpp and "reverse_enable_timeout_s_" in mode_cpp),
    ("Nav2 reverse remains disabled", vx_min is not None and vx_min >= 0.0),
]
print()
print("| static check | result |")
print("| --- | --- |")
for label, ok in checks:
    print(f"| {label} | {'PASS' if ok else 'FAIL'} |")
PY
}

write_runtime_summary() {
  python3 - "${OUTPUT_DIR}" "${EXECUTE_UNDOCK}" <<'PY' >"${OUTPUT_DIR}/summary.md" 2>/dev/null || true
import json
import math
import pathlib
import re
import sys

out = pathlib.Path(sys.argv[1])
execute_undock = sys.argv[2].lower() == "true"

def read(name):
    path = out / name
    return path.read_text(errors="ignore") if path.exists() else ""

def json_objects(text):
    dec = json.JSONDecoder()
    objs = []
    pos = 0
    while True:
        start = text.find("{", pos)
        if start < 0:
            break
        try:
            obj, end = dec.raw_decode(text[start:])
        except Exception:
            pos = start + 1
            continue
        objs.append(obj)
        pos = start + end
    return objs

def get(obj, dotted):
    cur = obj
    for part in dotted.split("."):
        if not isinstance(cur, dict):
            return None
        cur = cur.get(part)
    return cur

def first_nonzero_twist_time(text):
    for line in text.splitlines():
        m = re.match(r"^(\d+)\s+\s*(?:x|y|z):\s*(-?\d+(?:\.\d+)?)", line)
        if m and abs(float(m.group(2))) > 1.0e-6:
            return float(m.group(1))
    return None

def first_reverse_true_time(text):
    for line in text.splitlines():
        m = re.match(r"^(\d+)\s+(.*)$", line)
        if m and "true" in m.group(2).lower():
            return float(m.group(1))
    return None

def timestamped_status_lines(text):
    rows = []
    for line in text.splitlines():
        m = re.match(r"^(\d+)\s+(.*)$", line)
        if not m:
            continue
        body = m.group(2).strip()
        if body.startswith("$") or not body:
            continue
        if "undocking" in body or "undocked" in body or "undock_failed" in body:
            rows.append((float(m.group(1)), body))
    return rows

def status_cmd_count(lines):
    counts = []
    for _, body in lines:
        m = re.search(r"\bcmd_count=(\d+)", body)
        if m:
            counts.append(int(m.group(1)))
    return max(counts) if counts else None

def text_cmd_count(text):
    counts = [int(m.group(1)) for m in re.finditer(r"\bcmd_count=(\d+)", text or "")]
    return max(counts) if counts else None

def max_optional(*values):
    present = [v for v in values if v is not None]
    return max(present) if present else None

def first_status_phase(lines):
    return lines[0][1] if lines else None

def final_status(lines):
    return lines[-1][1] if lines else None

def status_has_waiting_first_motion_with_cmd(lines):
    for _, body in lines:
        count = re.search(r"\bcmd_count=(\d+)", body)
        cmd_x = re.search(r"\bcmd_x=(-?\d+(?:\.\d+)?)", body)
        if "waiting_first_motion" in body and count and int(count.group(1)) > 0:
            if cmd_x is None or abs(float(cmd_x.group(1))) > 1.0e-6:
                return True
    return False

def text_has_waiting_first_motion_with_cmd(text):
    count = re.search(r"\bcmd_count=(\d+)", text or "")
    cmd_x = re.search(r"\bcmd_x=(-?\d+(?:\.\d+)?)", text or "")
    if "waiting_first_motion" not in (text or "") or not count or int(count.group(1)) <= 0:
        return False
    return cmd_x is None or abs(float(cmd_x.group(1))) > 1.0e-6

def odom_motion(text, epsilon=0.005):
    samples = []
    current = {}
    current_time = None
    for line in text.splitlines():
        ts_match = re.match(r"^(\d+)\s+(.*)$", line)
        if not ts_match:
            continue
        t = float(ts_match.group(1))
        body = ts_match.group(2)
        if body.strip() == "---":
            if "x" in current and "y" in current:
                samples.append((current_time or t, current["x"], current["y"]))
            current = {}
            current_time = None
            continue
        m = re.match(r"\s*(x|y):\s*(-?\d+(?:\.\d+)?)", body)
        if m:
            current[m.group(1)] = float(m.group(2))
            current_time = current_time or t
    if "x" in current and "y" in current:
        samples.append((current_time or 0.0, current["x"], current["y"]))
    if not samples:
        return None, 0.0, 0
    x0, y0 = samples[0][1], samples[0][2]
    first_motion = None
    max_dist = 0.0
    for t, x, y in samples:
        dist = math.hypot(x - x0, y - y0)
        max_dist = max(max_dist, dist)
        if first_motion is None and dist > epsilon:
            first_motion = t
    return first_motion, max_dist, len(samples)

after = json_objects(read("api_after.json"))
post = json_objects(read("post_undock_response.json"))
status = after[0] if len(after) > 0 else {}
docking_state = after[2] if len(after) > 2 else {}
post_response = post[0] if len(post) > 0 else {}
cmd_time = first_nonzero_twist_time(read("echo_cmd_vel_docking.log"))
safe_time = first_nonzero_twist_time(read("echo_cmd_vel_safe.log"))
base_time = first_nonzero_twist_time(read("echo_cmd_vel.log"))
reverse_time = first_reverse_true_time(read("echo_docking_allow_reverse.log"))
first_motion, final_distance, odom_samples = odom_motion(read("echo_local_state_odometry.log"))
docking_status_text = read("echo_docking_status.log")
status_lines = timestamped_status_lines(docking_status_text)
status_first = first_status_phase(status_lines)
status_final = final_status(status_lines)
mode_status_text = read("echo_mode_controller_status.log")
reverse_text = read("echo_docking_allow_reverse.log")

failure_reason = None
for _, body in status_lines:
    if "undock_failed" in body or "undocked" in body:
        failure_reason = body
if failure_reason is None:
    failure_reason = get(docking_state, "docking.detail") or get(docking_state, "last_status")
api_status_text = "\n".join(str(v or "") for v in (
    get(docking_state, "last_status"),
    get(docking_state, "docking.detail"),
    post_response.get("message"),
    post_response.get("detail"),
))
topic_cmd_count = status_cmd_count(status_lines)
api_cmd_count = text_cmd_count(api_status_text)
max_cmd_count = max_optional(topic_cmd_count, api_cmd_count)
if topic_cmd_count is not None:
    cmd_source_evidence = "topic_status"
elif api_cmd_count is not None:
    cmd_source_evidence = "api_status"
else:
    cmd_source_evidence = "none"
waiting_with_cmd = status_has_waiting_first_motion_with_cmd(status_lines) or text_has_waiting_first_motion_with_cmd(api_status_text)

accepted = bool(post_response.get("accepted")) or bool(get(docking_state, "docking.docking_service_called"))
case = "CASE_DRY_RUN_NO_UNDOCK_EXECUTED" if not execute_undock else "CASE_UNKNOWN"
if not execute_undock:
    pass
elif "undock rejected" in (failure_reason or "") or "not docked" in (failure_reason or ""):
    case = "CASE_DOCKING_MANAGER_REJECTS_API_LATCH"
elif accepted and cmd_time is None and (max_cmd_count in (None, 0)):
    case = "CASE_DOCKING_MANAGER_NO_CMD"
elif waiting_with_cmd and cmd_time is not None and first_motion is None:
    case = "CASE_DOCKING_MANAGER_WAITING_FIRST_MOTION_WITH_CMD"
elif safe_time is None:
    case = "CASE_SAFETY_BLOCKED_DOCKING_CMD"
elif "cmd_out" in mode_status_text and "-0.06" not in mode_status_text and "linear_x" in mode_status_text:
    case = "CASE_MODE_CONTROLLER_BLOCKED_REVERSE"
elif odom_samples == 0:
    case = "CASE_ODOM_NOT_UPDATING"
elif first_motion is None and failure_reason and "motion_start_timeout" in failure_reason:
    case = "CASE_CHASSIS_NO_MOTION_AFTER_CMD"
elif first_motion is None and failure_reason and "no_motion" in failure_reason:
    case = "CASE_LOGIC_NO_PROGRESS_TIMER_TOO_EARLY"
elif first_motion is None:
    case = "CASE_CHASSIS_NO_MOTION_AFTER_CMD"
elif failure_reason and "no_progress" in failure_reason:
    case = "CASE_CHASSIS_NO_MOTION_AFTER_CMD"
else:
    case = "CASE_OK_OR_INCONCLUSIVE"

print("# Undock Logic And No-Motion Diagnostic")
print()
print(f"- report_dir: `{out}`")
print(f"- api_docking_state: `{get(docking_state, 'state')}`")
print(f"- api_docking_last_status: `{get(docking_state, 'last_status')}`")
print(f"- api_docking_job_detail: `{get(docking_state, 'docking.detail')}`")
print(f"- safety_status_after: `{get(status, 'safety.status')}`")
print()
print("## Timing")
print(f"- first_docking_status_phase: `{status_first}`")
print(f"- first_nonzero_cmd_time: `{cmd_time}`")
print(f"- first_reverse_enable_true_time: `{reverse_time}`")
print(f"- first_nonzero_safe_time: `{safe_time}`")
print(f"- first_nonzero_base_cmd_time: `{base_time}`")
print(f"- first_motion_time: `{first_motion}`")
if cmd_time is not None and first_motion is not None:
    print(f"- motion_start_elapsed_s: `{first_motion - cmd_time:.3f}`")
else:
    print("- motion_start_elapsed_s: `unknown`")
print(f"- no_progress_timer_started_at: `{first_motion if first_motion is not None else 'not_started'}`")
print(f"- final_distance: `{final_distance:.6f}`")
print(f"- odom_samples: `{odom_samples}`")
print(f"- final_docking_status: `{status_final}`")
print(f"- topic_status_cmd_count_max: `{topic_cmd_count}`")
print(f"- api_status_cmd_count_max: `{api_cmd_count}`")
print(f"- status_cmd_count_max: `{max_cmd_count}`")
print(f"- cmd_source_evidence: `{cmd_source_evidence}`")
print(f"- failure_reason: `{failure_reason}`")
print(f"- case: `{case}`")
print()
print("## Chain Evidence")
print(f"- docking_allow_reverse_true_observed: `{'data: true' in reverse_text}`")
print(f"- mode_controller_status_contains_cmd_out: `{'cmd_out' in mode_status_text}`")
PY
}

echo "${PREFIX} report_dir=${OUTPUT_DIR}"
echo "${PREFIX} execute_undock=${EXECUTE_UNDOCK}"
echo "${PREFIX} api_url=${API_URL}"

write_static_audit
api_snapshot "api_before"
run_logged "topic_info_before" bash -lc \
  'for topic in /cmd_vel_docking /cmd_vel_safe /cmd_vel /docking/status /safety/status /local_state/odometry /ranger_mini3/docking_allow_reverse /ranger_mini3_mode_controller/status; do echo "## ${topic}"; timeout 5 ros2 topic info -v "${topic}"; done'

if [[ "${EXECUTE_UNDOCK}" != "true" ]]; then
  api_snapshot "api_after"
  write_runtime_summary
  echo "${PREFIX} dry-run done"
  echo "${PREFIX} static_audit=${OUTPUT_DIR}/static_audit.md"
  echo "${PREFIX} summary=${OUTPUT_DIR}/summary.md"
  exit 0
fi

echo "${PREFIX} CAPTURE ACTIVE: executing controlled API undock; do not publish manual velocity commands."
start_timed_echo "echo_cmd_vel_docking" /cmd_vel_docking
start_timed_echo "echo_cmd_vel_safe" /cmd_vel_safe
start_timed_echo "echo_cmd_vel" /cmd_vel
start_timed_echo "echo_docking_allow_reverse" /ranger_mini3/docking_allow_reverse
start_timed_echo "echo_mode_controller_status" /ranger_mini3_mode_controller/status --field data
start_timed_echo "echo_local_state_odometry" /local_state/odometry --field pose.pose.position
start_timed_echo "echo_docking_status" /docking/status --field data

sleep 1
{
  echo "POST ${API_URL}/api/v1/docking/undock"
  curl_json POST /api/v1/docking/undock '{"reason":"diagnose_undock_logic_and_no_motion"}' || true
  echo
} >"${OUTPUT_DIR}/post_undock_response.json" 2>"${OUTPUT_DIR}/post_undock_response.err"

sleep "${DURATION_SEC}"
cleanup
trap - EXIT INT TERM

api_snapshot "api_after"
run_logged "topic_info_after" bash -lc \
  'for topic in /cmd_vel_docking /cmd_vel_safe /cmd_vel /docking/status /safety/status /local_state/odometry /ranger_mini3/docking_allow_reverse /ranger_mini3_mode_controller/status; do echo "## ${topic}"; timeout 5 ros2 topic info -v "${topic}"; done'
write_runtime_summary

echo "${PREFIX} done"
echo "${PREFIX} static_audit=${OUTPUT_DIR}/static_audit.md"
echo "${PREFIX} summary=${OUTPUT_DIR}/summary.md"
