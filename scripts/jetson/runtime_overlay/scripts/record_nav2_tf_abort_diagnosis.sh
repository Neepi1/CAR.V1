#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "${SCRIPT_DIR}/../../../.." && pwd)"

duration_sec="180"
label="nav_tf_abort"
report_root="${WORKSPACE_ROOT}/reports/nav2_tf_abort_diagnosis"
controller_pid=""
bridge_pid=""

usage() {
  cat <<'EOF'
Usage:
  record_nav2_tf_abort_diagnosis.sh [--duration-sec N] [--label LABEL]

Read-only Nav2 controller TF-abort recorder. It does not send navigation goals,
publish velocity, restart services, or subscribe to PointCloud2 topics.

Start this script, then send a navigation goal from the App. The script records:
  - map->odom and odom->base_link /tf receive gaps and stamp gaps
  - navigate_to_pose / follow_path / compute_path_to_pose action status
  - filtered /rosout TF/controller/costmap errors
  - controller_server and robot_localization_bridge thread/CPU snapshots
  - API /api/v1/navigation/state snapshots
  - controller_server log tail around the observation window
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --duration-sec)
      duration_sec="${2:-}"
      shift 2
      ;;
    --label)
      label="${2:-}"
      shift 2
      ;;
    --help|-h)
      usage
      exit 0
      ;;
    *)
      echo "[nav2-tf-abort] unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

if ! [[ "${duration_sec}" =~ ^[0-9]+([.][0-9]+)?$ ]]; then
  echo "[nav2-tf-abort] --duration-sec must be numeric" >&2
  exit 2
fi

sanitize_label() {
  printf '%s' "$1" | tr -c 'A-Za-z0-9_.-' '_'
}

if [[ -f /opt/ros/humble/setup.bash ]]; then
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
fi
if [[ -f "${WORKSPACE_ROOT}/install/setup.bash" ]]; then
  # shellcheck disable=SC1091
  source "${WORKSPACE_ROOT}/install/setup.bash"
fi

timestamp="$(date -u +%Y%m%dT%H%M%SZ)"
label="$(sanitize_label "${label}")"
report_dir="${report_root}/${timestamp}_${label}_${duration_sec}s"
mkdir -p "${report_dir}"

controller_pid="$(pgrep -f 'nav2_controller/controller_server' | head -n 1 || true)"
bridge_pid="$(pgrep -f 'robot_localization_bridge.*/localization_bridge_node|localization_bridge_node' | head -n 1 || true)"
controller_log="$(ls -t /root/.ros/log/controller_server_*.log 2>/dev/null | head -n 1 || true)"
bt_log="$(ls -t /root/.ros/log/bt_navigator_*.log 2>/dev/null | head -n 1 || true)"
planner_log="$(ls -t /root/.ros/log/planner_server_*.log 2>/dev/null | head -n 1 || true)"

echo "[nav2-tf-abort] report_dir=${report_dir}"
echo "[nav2-tf-abort] duration_sec=${duration_sec}"
echo "[nav2-tf-abort] controller_pid=${controller_pid:-missing} bridge_pid=${bridge_pid:-missing}"
echo "[nav2-tf-abort] start the App navigation goal now if you have not already"

sample_proc_loop() {
  local out="$1"
  local end_epoch
  end_epoch="$(python3 - <<PY
import time
print(time.time() + float("${duration_sec}"))
PY
)"
  while python3 - <<PY
import sys, time
sys.exit(0 if time.time() < float("${end_epoch}") else 1)
PY
  do
    {
      echo "=== $(date -u +%Y-%m-%dT%H:%M:%SZ) ==="
      if [[ -n "${controller_pid}" && -d "/proc/${controller_pid}" ]]; then
        echo "--- controller_server threads pid=${controller_pid} ---"
        ps -L -p "${controller_pid}" -o pid,tid,psr,pcpu,comm,wchan:28
        grep -E 'Name|Cpus_allowed_list|voluntary_ctxt_switches|nonvoluntary_ctxt_switches' \
          "/proc/${controller_pid}/status" 2>/dev/null || true
      else
        echo "--- controller_server missing ---"
      fi
      if [[ -n "${bridge_pid}" && -d "/proc/${bridge_pid}" ]]; then
        echo "--- robot_localization_bridge threads pid=${bridge_pid} ---"
        ps -L -p "${bridge_pid}" -o pid,tid,psr,pcpu,comm,wchan:28
        grep -E 'Name|Cpus_allowed_list|voluntary_ctxt_switches|nonvoluntary_ctxt_switches' \
          "/proc/${bridge_pid}/status" 2>/dev/null || true
      else
        echo "--- robot_localization_bridge missing ---"
      fi
      echo "--- top cpu ---"
      ps -eo pid,psr,pcpu,pmem,comm,args --sort=-pcpu | head -n 35
      echo
    } >>"${out}" 2>&1
    sleep 1
  done
}

record_api_state_loop() {
  local out="$1"
  local end_epoch
  end_epoch="$(python3 - <<PY
import time
print(time.time() + float("${duration_sec}"))
PY
)"
  while python3 - <<PY
import sys, time
sys.exit(0 if time.time() < float("${end_epoch}") else 1)
PY
  do
    {
      echo "=== $(date -u +%Y-%m-%dT%H:%M:%SZ) ==="
      timeout 3 curl -sS http://127.0.0.1:8080/api/v1/navigation/state || true
      echo
    } >>"${out}" 2>&1
    sleep 2
  done
}

record_tf_stats() {
  local out_json="$1"
  python3 - "${duration_sec}" "${out_json}" <<'PY'
import json
import sys
import time

import rclpy
from tf2_msgs.msg import TFMessage

duration = float(sys.argv[1])
out_json = sys.argv[2]

tracks = {
    ("map", "odom"): [],
    ("odom", "base_link"): [],
}
last_recv = {}
last_stamp = {}

def stamp_sec(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9

def add(frame, child, recv_time, stamp):
    key = (frame, child)
    if key not in tracks:
        return
    st = stamp_sec(stamp)
    tracks[key].append({
        "recv_time": recv_time,
        "stamp": st,
        "recv_minus_stamp_ms": (recv_time - st) * 1000.0,
        "recv_gap_ms": None if key not in last_recv else (recv_time - last_recv[key]) * 1000.0,
        "stamp_gap_ms": None if key not in last_stamp else (st - last_stamp[key]) * 1000.0,
    })
    last_recv[key] = recv_time
    last_stamp[key] = st

def summarize(rows):
    def stats(name):
      vals = [r[name] for r in rows if r[name] is not None]
      if not vals:
          return {"count": 0, "avg_ms": None, "p95_ms": None, "p99_ms": None, "max_ms": None}
      vals_sorted = sorted(vals)
      return {
          "count": len(vals),
          "avg_ms": sum(vals) / len(vals),
          "p95_ms": vals_sorted[int(0.95 * (len(vals_sorted) - 1))],
          "p99_ms": vals_sorted[int(0.99 * (len(vals_sorted) - 1))],
          "max_ms": max(vals),
      }
    return {
        "count": len(rows),
        "recv_gap": stats("recv_gap_ms"),
        "stamp_gap": stats("stamp_gap_ms"),
        "recv_minus_stamp": stats("recv_minus_stamp_ms"),
        "last_samples": rows[-20:],
    }

rclpy.init()
node = rclpy.create_node("record_nav2_tf_abort_tf_probe")
node.create_subscription(TFMessage, "/tf", lambda msg: [
    add(tf.header.frame_id, tf.child_frame_id, time.time(), tf.header.stamp)
    for tf in msg.transforms
], 100)

deadline = time.time() + duration
while rclpy.ok() and time.time() < deadline:
    rclpy.spin_once(node, timeout_sec=0.1)

node.destroy_node()
rclpy.shutdown()

summary = {f"{k[0]}->{k[1]}": summarize(v) for k, v in tracks.items()}
with open(out_json, "w", encoding="utf-8") as f:
    json.dump(summary, f, indent=2, sort_keys=True)
PY
}

record_topic() {
  local topic="$1"
  local out="$2"
  timeout "${duration_sec}" ros2 topic echo "${topic}" --full-length >"${out}" 2>&1 || true
}

record_rosout_filtered() {
  local out="$1"
  timeout "${duration_sec}" ros2 topic echo /rosout --qos-reliability best_effort 2>/dev/null |
    grep -Ei 'failed|abort|cancel|progress|collision|safety|tf|transform|extrapolat|message filter|controller|bt_navigator|goal|amcl|costmap|exception|follow_path' \
    >"${out}" || true
}

sample_proc_loop "${report_dir}/proc_cpu_threads.log" &
pids=($!)
record_api_state_loop "${report_dir}/api_navigation_state.jsonl" &
pids+=($!)
record_tf_stats "${report_dir}/tf_stats.json" &
pids+=($!)
record_rosout_filtered "${report_dir}/rosout_filtered.log" &
pids+=($!)
record_topic /navigate_to_pose/_action/status "${report_dir}/navigate_to_pose_status.log" &
pids+=($!)
record_topic /follow_path/_action/status "${report_dir}/follow_path_status.log" &
pids+=($!)
record_topic /compute_path_to_pose/_action/status "${report_dir}/compute_path_to_pose_status.log" &
pids+=($!)
record_topic /localization/bridge_status "${report_dir}/bridge_status.log" &
pids+=($!)
record_topic /amcl_scan_admission/status "${report_dir}/amcl_scan_admission_status.log" &
pids+=($!)

for pid in "${pids[@]}"; do
  wait "${pid}" || true
done

if [[ -n "${controller_log}" && -f "${controller_log}" ]]; then
  tail -n 800 "${controller_log}" >"${report_dir}/controller_server_tail.log" || true
fi
if [[ -n "${bt_log}" && -f "${bt_log}" ]]; then
  tail -n 300 "${bt_log}" >"${report_dir}/bt_navigator_tail.log" || true
fi
if [[ -n "${planner_log}" && -f "${planner_log}" ]]; then
  tail -n 300 "${planner_log}" >"${report_dir}/planner_server_tail.log" || true
fi

python3 - "${report_dir}" "${duration_sec}" "${controller_pid:-}" "${bridge_pid:-}" <<'PY'
import json
import re
import sys
from pathlib import Path

report = Path(sys.argv[1])
duration = sys.argv[2]
controller_pid = sys.argv[3] or "missing"
bridge_pid = sys.argv[4] or "missing"

tf_stats = {}
tf_path = report / "tf_stats.json"
if tf_path.exists():
    tf_stats = json.loads(tf_path.read_text(encoding="utf-8"))

controller_tail = (report / "controller_server_tail.log").read_text(errors="replace") if (report / "controller_server_tail.log").exists() else ""
abort_lines = [
    line for line in controller_tail.splitlines()
    if re.search(r"extrapolat|Unable to transform|Aborting handle|Failed to make progress|Exception", line, re.I)
]

summary = {
    "duration_sec": duration,
    "controller_pid": controller_pid,
    "bridge_pid": bridge_pid,
    "tf_stats": tf_stats,
    "controller_key_lines": abort_lines[-80:],
}
(report / "summary.json").write_text(json.dumps(summary, indent=2, sort_keys=True), encoding="utf-8")

md = ["# Nav2 TF Abort Diagnosis", ""]
md.append(f"- duration_sec: {duration}")
md.append(f"- controller_pid: {controller_pid}")
md.append(f"- bridge_pid: {bridge_pid}")
md.append("")
md.append("## TF Stats")
for key, value in tf_stats.items():
    md.append(f"### {key}")
    md.append("```json")
    md.append(json.dumps(value, indent=2, sort_keys=True))
    md.append("```")
md.append("")
md.append("## Controller Key Lines")
md.append("```text")
md.extend(abort_lines[-80:] if abort_lines else ["no key controller lines found in tail"])
md.append("```")
(report / "summary.md").write_text("\n".join(md) + "\n", encoding="utf-8")
PY

echo "[nav2-tf-abort] wrote ${report_dir}"
echo "[nav2-tf-abort] summary ${report_dir}/summary.md"
