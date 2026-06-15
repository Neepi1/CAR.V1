#!/usr/bin/env bash
set -Eeuo pipefail

# Read-only field observer for the timeline from explicit relocalization accept
# to the next Nav2/fine-docking stage. It does not send goals or velocity.

DURATION_SEC=120
LABEL="relocalization_to_next_stage"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --duration-sec)
      DURATION_SEC="${2:-120}"
      shift 2
      ;;
    --label)
      LABEL="${2:-relocalization_to_next_stage}"
      shift 2
      ;;
    -h|--help)
      cat <<'EOF'
Usage: observe_relocalization_to_next_stage.sh [--duration-sec N] [--label LABEL]
EOF
      exit 0
      ;;
    *)
      echo "[reloc-observe] unknown argument: $1" >&2
      exit 2
      ;;
  esac
done

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "${SCRIPT_DIR}/../../../.." && pwd)"
cd "${WORKSPACE_ROOT}"

set +u
if [[ -f /opt/ros/humble/setup.bash ]]; then
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
fi
if [[ -f install/setup.bash ]]; then
  # shellcheck disable=SC1091
  source install/setup.bash
fi
set -u

stamp="$(date -u +%Y%m%dT%H%M%SZ)"
safe_label="$(tr -cs 'A-Za-z0-9_.-' '_' <<<"${LABEL}" | sed 's/^_//;s/_$//')"
report_dir="${WORKSPACE_ROOT}/reports/relocalization_to_next_stage/${stamp}_${safe_label}_${DURATION_SEC}s"
mkdir -p "${report_dir}"

pids=()
cleanup() {
  for pid in "${pids[@]:-}"; do
    kill "${pid}" >/dev/null 2>&1 || true
  done
}
trap cleanup EXIT

record_topic() {
  local topic="$1"
  local file="$2"
  timeout "${DURATION_SEC}" ros2 topic echo "${topic}" >"${file}" 2>&1 &
  pids+=("$!")
}

record_topic_field() {
  local topic="$1"
  local field="$2"
  local file="$3"
  timeout "${DURATION_SEC}" ros2 topic echo --field "${field}" "${topic}" >"${file}" 2>&1 &
  pids+=("$!")
}

record_topic_field /localization/bridge_status data "${report_dir}/bridge_status.log"
record_topic_field /docking/status data "${report_dir}/docking_status.log"
record_topic /local_costmap/costmap "${report_dir}/local_costmap.log"
record_topic /cmd_vel_nav "${report_dir}/cmd_vel_nav.log"
record_topic /cmd_vel_collision_checked "${report_dir}/cmd_vel_collision_checked.log"
record_topic /cmd_vel_safe "${report_dir}/cmd_vel_safe.log"
record_topic /cmd_vel "${report_dir}/cmd_vel.log"
record_topic /rosout "${report_dir}/rosout.log"

(
  end=$((SECONDS + DURATION_SEC))
  while [[ ${SECONDS} -lt ${end} ]]; do
    {
      echo "### $(date -u +%Y-%m-%dT%H:%M:%SZ)"
      curl -fsS http://127.0.0.1:8080/api/v1/status 2>/dev/null || true
      echo
      curl -fsS http://127.0.0.1:8080/api/v1/navigation/state 2>/dev/null || true
      echo
      curl -fsS http://127.0.0.1:8080/api/v1/docking/state 2>/dev/null || true
      echo
    } >>"${report_dir}/api_poll.log"
    sleep 1
  done
) &
pids+=("$!")

sleep "${DURATION_SEC}"
cleanup
trap - EXIT

python3 - "${report_dir}" "${DURATION_SEC}" <<'PY'
import json
import re
import sys
from pathlib import Path

report_dir = Path(sys.argv[1])
duration = sys.argv[2]

def lines(path):
    p = report_dir / path
    if not p.exists():
        return []
    return p.read_text(errors="replace").splitlines()

bridge_json = []
for line in lines("bridge_status.log"):
    line = line.strip()
    if line.startswith("{") and line.endswith("}"):
        try:
            bridge_json.append(json.loads(line))
        except Exception:
            pass

rosout = "\n".join(lines("rosout.log"))
api = "\n".join(lines("api_poll.log"))

seqs = [x.get("last_explicit_relocalization_sequence") for x in bridge_json if isinstance(x, dict)]
seqs = [x for x in seqs if isinstance(x, (int, float))]
owners = [x.get("map_to_odom_publisher_owner") for x in bridge_json if isinstance(x, dict)]
map_age = [x.get("map_to_odom_age_ms") for x in bridge_json if isinstance(x, dict)]
map_age = [x for x in map_age if isinstance(x, (int, float))]

summary = {
    "duration_sec": duration,
    "bridge_status_samples": len(bridge_json),
    "first_explicit_sequence": seqs[0] if seqs else None,
    "last_explicit_sequence": seqs[-1] if seqs else None,
    "map_to_odom_owner_last": owners[-1] if owners else None,
    "map_to_odom_age_ms_max": max(map_age) if map_age else None,
    "message_filter_drop_count": len(re.findall(r"Message Filter dropping|earlier than all (the )?data", rosout, re.I)),
    "nav2_result_code_mentions": re.findall(r"nav2_result_code[^0-9-]*(-?\d+)", api),
    "settle_mentions": len(re.findall(r"post_relocalization_settle", api)),
}
(report_dir / "summary.json").write_text(json.dumps(summary, indent=2, sort_keys=True))
(report_dir / "summary.md").write_text(
    "\n".join([
        "# Relocalization To Next Stage Observation",
        "",
        f"- duration_sec: {duration}",
        f"- bridge_status_samples: {summary['bridge_status_samples']}",
        f"- first_explicit_sequence: {summary['first_explicit_sequence']}",
        f"- last_explicit_sequence: {summary['last_explicit_sequence']}",
        f"- map_to_odom_owner_last: {summary['map_to_odom_owner_last']}",
        f"- map_to_odom_age_ms_max: {summary['map_to_odom_age_ms_max']}",
        f"- message_filter_drop_count: {summary['message_filter_drop_count']}",
        f"- nav2_result_code_mentions: {summary['nav2_result_code_mentions']}",
        f"- settle_mentions: {summary['settle_mentions']}",
    ]) + "\n"
)
print(report_dir / "summary.md")
PY

echo "[reloc-observe] wrote ${report_dir}"
echo "[reloc-observe] summary ${report_dir}/summary.md"
