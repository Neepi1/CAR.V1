#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/common_env.sh"

DURATION_SEC=180
LABEL="amcl_navigation_shadow"

usage() {
  cat <<'USAGE'
Usage: observe_amcl_navigation_shadow_180s.sh [--duration-sec N] [--label LABEL]

Records AMCL shadow/gated localization, bridge status, TF timing, cmd_vel chain,
and key CPU placement while an operator sends a short navigation goal. This
script does not send goals and does not subscribe to pointcloud topics.
USAGE
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --duration-sec)
      DURATION_SEC="${2:-}"
      shift 2
      ;;
    --label)
      LABEL="${2:-}"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "[observe-amcl-shadow] unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

case "${DURATION_SEC}" in
  ''|*[!0-9]*)
    echo "[observe-amcl-shadow] --duration-sec must be an integer" >&2
    exit 2
    ;;
esac

timestamp="$(date -u +%Y%m%dT%H%M%SZ)"
report_md="${NJRH_PROJECT_ROOT}/reports/amcl_navigation_shadow_${timestamp}.md"
mkdir -p "$(dirname "${report_md}")"

echo "[observe-amcl-shadow] duration_sec=${DURATION_SEC} label=${LABEL}" >&2
raw_output="$(
  bash "${SCRIPT_DIR}/observe_navigation_tf_jitter_180s.sh" \
    --duration-sec "${DURATION_SEC}" \
    --label "${LABEL}" 2>&1
)"
echo "${raw_output}" >&2

observation_dir="$(awk '/^\/.*controller_tf_backlog_180s|^\/.*navigation_tf_jitter_180s/ {path=$0} END {print path}' <<<"${raw_output}")"
if [[ -z "${observation_dir}" || ! -d "${observation_dir}" ]]; then
  echo "[observe-amcl-shadow] failed to locate underlying observation directory" >&2
  exit 1
fi

summary_json="${observation_dir}/summary.json"
bridge_csv="${observation_dir}/bridge_status_samples.csv"

python3 - "${summary_json}" "${bridge_csv}" "${report_md}" "${observation_dir}" <<'PY'
import csv
import json
import sys
from pathlib import Path

summary_path = Path(sys.argv[1])
bridge_csv = Path(sys.argv[2])
report_path = Path(sys.argv[3])
observation_dir = Path(sys.argv[4])

summary = json.loads(summary_path.read_text()) if summary_path.exists() else {}
last_bridge = summary.get("bridge_status_last", {})
bridge_rows = []
if bridge_csv.exists():
    with bridge_csv.open(newline="") as f:
        for row in csv.DictReader(f):
            try:
                bridge_rows.append(json.loads(row.get("data") or "{}"))
            except Exception:
                pass
if bridge_rows:
    last_bridge = bridge_rows[-1]

def delta(field):
    vals = [row.get(field) for row in bridge_rows if isinstance(row.get(field), (int, float))]
    if len(vals) < 2:
        return None
    return vals[-1] - vals[0]

amcl_pose = summary.get("/amcl_pose", {})
scan_amcl = summary.get("/scan_amcl", {})
map_odom = summary.get("tf:map->odom", {})
odom_base = summary.get("tf:odom->base_link", {})
rosout = summary.get("rosout_counts", {})

lines = [
    "# AMCL Navigation Shadow Observation",
    "",
    f"- observation_dir: {observation_dir}",
    f"- scan_amcl_hz: {scan_amcl.get('hz', 0.0):.3f}",
    f"- amcl_pose_hz: {amcl_pose.get('hz', 0.0):.3f}",
    f"- amcl_pose_count: {amcl_pose.get('count', 0)}",
    f"- amcl_candidate_count_delta: {delta('amcl_candidate_count')}",
    f"- amcl_accepted_count_delta: {delta('amcl_accepted_count')}",
    f"- amcl_rejected_count_delta: {delta('amcl_rejected_count')}",
    f"- amcl_shadow_ready_last: {last_bridge.get('amcl_shadow_ready')}",
    f"- amcl_gated_ready_last: {last_bridge.get('amcl_gated_ready')}",
    f"- localization_degraded_last: {last_bridge.get('localization_degraded')}",
    f"- active_correction_source_last: {last_bridge.get('active_correction_source')}",
    f"- last_reject_reason_last: {last_bridge.get('last_reject_reason')}",
    f"- map_to_odom_hz: {map_odom.get('hz', 0.0):.3f}",
    f"- odom_to_base_hz: {odom_base.get('hz', 0.0):.3f}",
    f"- tf_future_extrapolation_count: {rosout.get('tf_future_extrapolation')}",
    f"- message_filter_drop_count: {rosout.get('message_filter_drop')}",
    "",
    "## Last Bridge Status",
    "",
    "```json",
    json.dumps(last_bridge, indent=2, sort_keys=True),
    "```",
]
report_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
print(report_path)
PY
