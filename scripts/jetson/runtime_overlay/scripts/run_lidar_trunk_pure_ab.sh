#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/common_env.sh"
source "${SCRIPT_DIR}/local_perception_profile.sh"
njrh_load_local_perception_input_profile

DURATION_SEC="${NJRH_LIDAR_PURE_TRUNK_DURATION_SEC:-20}"
MIN_AXIS_HZ="${NJRH_LIDAR_PURE_TRUNK_MIN_AXIS_HZ:-18.0}"
PURE_READY_TIMEOUT_SEC="${NJRH_LIDAR_PURE_TRUNK_READY_TIMEOUT_SEC:-45}"
INCLUDE_CLI_HZ=false
EXECUTE=false

usage() {
  cat <<'EOF'
Usage: run_lidar_trunk_pure_ab.sh [--duration-sec SECONDS] [--include-cli-hz] [--execute]

Runs a controlled diagnostic A/B for pointcloud_axis_remap with only the
full-density /lidar_points trunk enabled. By default it prints the test plan and
does not stop production runtime.

This is a stationary diagnostic. During --execute, the local and nav pointcloud
branches are temporarily disabled at the axis remap node, then the production
driver/remap profile is restored automatically.

Options:
  --duration-sec N   Seconds to collect pure-trunk axis status. Default: 20.
  --include-cli-hz   Also run ros2 topic hz /lidar_points during the pure run.
                    This creates a temporary full-density subscriber.
  --execute          Actually restart JT128 ingress for the temporary A/B.
  -h, --help         Show this help.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --duration-sec)
      [[ $# -ge 2 ]] || { echo "[lidar-pure-ab] --duration-sec requires a value" >&2; exit 2; }
      DURATION_SEC="$2"
      shift 2
      ;;
    --include-cli-hz)
      INCLUDE_CLI_HZ=true
      shift
      ;;
    --execute)
      EXECUTE=true
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "[lidar-pure-ab] unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

case "${DURATION_SEC}" in
  ''|*[!0-9]*)
    echo "[lidar-pure-ab] invalid --duration-sec: ${DURATION_SEC}" >&2
    exit 2
    ;;
esac

tmp_dir="$(mktemp -d /tmp/njrh_lidar_pure_ab_XXXXXX)"
trap 'rm -rf "${tmp_dir}"' EXIT

BASE_REMAP_CONFIG="${NJRH_JT128_CANONICAL_POINTCLOUD_REMAP_CONFIG:-${NJRH_OVERLAY_ROOT}/config/jt128_canonical_pointcloud_remap.yaml}"
PURE_REMAP_CONFIG="${tmp_dir}/jt128_canonical_pointcloud_remap_pure_trunk.yaml"
PURE_LOG="${NJRH_RUNTIME_LOG_DIR}/run_driver_lidar_pure_trunk_ab.log"
RESTORE_LOG="${NJRH_RUNTIME_LOG_DIR}/run_driver_lidar_pure_trunk_restore.log"
PURE_PID=""
RESTORE_REQUESTED=false

float_ge() {
  awk -v value="${1:-nan}" -v minimum="${2:-0}" 'BEGIN { exit(value + 0.0 >= minimum + 0.0 ? 0 : 1) }'
}

field_avg() {
  local file="$1"
  local key="$2"
  awk -v key="${key}" '
    {
      for (i = 1; i <= NF; i += 1) {
        split($i, kv, "=")
        if (kv[1] == key && kv[2] ~ /^-?[0-9]+([.][0-9]+)?$/) {
          sum += kv[2]
          count += 1
        }
      }
    }
    END {
      if (count > 0) {
        printf "%.3f", sum / count
      } else {
        exit 1
      }
    }
  ' "${file}"
}

field_latest() {
  local file="$1"
  local key="$2"
  awk -v key="${key}" '
    {
      for (i = 1; i <= NF; i += 1) {
        split($i, kv, "=")
        if (kv[1] == key) {
          value = substr($i, length(key) + 2)
        }
      }
    }
    END {
      if (value != "") {
        print value
      } else {
        exit 1
      }
    }
  ' "${file}"
}

make_pure_config() {
  [[ -f "${BASE_REMAP_CONFIG}" ]] || {
    echo "[lidar-pure-ab] base remap config missing: ${BASE_REMAP_CONFIG}" >&2
    exit 1
  }
  sed -E \
    -e 's|^([[:space:]]*nav_output_topic:[[:space:]]*).*|\1""|' \
    -e 's|^([[:space:]]*nav_output_stride:).*|\1 1|' \
    -e 's|^([[:space:]]*nav_output_publish_every_n:).*|\1 1|' \
    -e 's|^([[:space:]]*local_output_topic:[[:space:]]*).*|\1""|' \
    -e 's|^([[:space:]]*local_output_stride:).*|\1 1|' \
    -e 's|^([[:space:]]*local_output_publish_every_n:).*|\1 1|' \
    "${BASE_REMAP_CONFIG}" >"${PURE_REMAP_CONFIG}"
}

collect_axis_status() {
  local seconds="$1"
  local label="$2"
  local raw_file="${tmp_dir}/${label}.raw"
  local samples_file="${tmp_dir}/${label}.samples"
  timeout "${seconds}" ros2 topic echo /lidar/axis_remap_status --field data >"${raw_file}" 2>&1 || true
  awk 'NF && $0 != "---" {print}' "${raw_file}" >"${samples_file}"
  if [[ ! -s "${samples_file}" ]]; then
    echo "[lidar-pure-ab] WARN /lidar/axis_remap_status has no ${label} samples" >&2
    sed 's/^/[lidar-pure-ab]   /' "${raw_file}" >&2 || true
  fi
  printf '%s\n' "${samples_file}"
}

wait_for_pure_axis() {
  local samples
  local nav_enabled
  local local_enabled
  local raw_hz
  for _ in $(seq 1 "${PURE_READY_TIMEOUT_SEC}"); do
    samples="$(collect_axis_status 2 wait_pure)"
    nav_enabled="$(field_latest "${samples}" nav_branch_enabled 2>/dev/null || true)"
    local_enabled="$(field_latest "${samples}" local_branch_enabled 2>/dev/null || true)"
    raw_hz="$(field_latest "${samples}" raw_input_hz 2>/dev/null || true)"
    if [[ "${nav_enabled}" == "false" && "${local_enabled}" == "false" ]] &&
      [[ -n "${raw_hz}" ]] && float_ge "${raw_hz}" "1.0"
    then
      return 0
    fi
    sleep 1
  done
  echo "[lidar-pure-ab] pure trunk axis status with live raw input did not appear within ${PURE_READY_TIMEOUT_SEC}s" >&2
  return 1
}

restore_production_driver() {
  [[ "${RESTORE_REQUESTED}" == "true" ]] || return 0
  echo "[lidar-pure-ab] restoring production JT128 ingress/remap profile" >&2
  nohup env \
    NJRH_FORCE_RESTART_DRIVER=true \
    bash "${SCRIPT_DIR}/run_driver.sh" \
    >"${RESTORE_LOG}" 2>&1 &
  echo "[lidar-pure-ab] restore run_driver pid=$! log=${RESTORE_LOG}"
}

print_summary() {
  local samples="$1"
  local prefix="$2"
  local axis_raw_hz
  local axis_publish_hz
  local gap100
  local gap150
  local gap200
  local raw_interarrival_avg
  local publish_interval_avg
  axis_raw_hz="$(field_avg "${samples}" raw_input_hz 2>/dev/null || true)"
  axis_publish_hz="$(field_avg "${samples}" lidar_points_publish_hz 2>/dev/null || true)"
  gap100="$(field_latest "${samples}" trunk_publish_gap_over_100ms_count 2>/dev/null || true)"
  gap150="$(field_latest "${samples}" trunk_publish_gap_over_150ms_count 2>/dev/null || true)"
  gap200="$(field_latest "${samples}" trunk_publish_gap_over_200ms_count 2>/dev/null || true)"
  raw_interarrival_avg="$(field_avg "${samples}" raw_interarrival_ms_avg 2>/dev/null || true)"
  publish_interval_avg="$(field_avg "${samples}" lidar_points_publish_interval_ms_avg 2>/dev/null || true)"

  echo "[lidar-pure-ab] ${prefix} samples=$(wc -l <"${samples}")"
  echo "[lidar-pure-ab] ${prefix} raw_input_hz=${axis_raw_hz:-missing} lidar_points_publish_hz=${axis_publish_hz:-missing}"
  echo "[lidar-pure-ab] ${prefix} raw_interarrival_ms_avg=${raw_interarrival_avg:-missing} publish_interval_ms_avg=${publish_interval_avg:-missing}"
  echo "[lidar-pure-ab] ${prefix} trunk gaps: >100ms=${gap100:-missing} >150ms=${gap150:-missing} >200ms=${gap200:-missing}"

  if [[ -n "${axis_publish_hz}" ]] && float_ge "${axis_publish_hz}" "${MIN_AXIS_HZ}"; then
    echo "[lidar-pure-ab] ${prefix} PASS axis publish-side >= ${MIN_AXIS_HZ}Hz"
  else
    echo "[lidar-pure-ab] ${prefix} FAIL axis publish-side below ${MIN_AXIS_HZ}Hz or missing"
    return 1
  fi
}

make_pure_config

cat <<EOF
[lidar-pure-ab] profile=${NJRH_LOCAL_PERCEPTION_INPUT_PROFILE} source=${NJRH_LOCAL_PERCEPTION_INPUT_PROFILE_SOURCE}
[lidar-pure-ab] base_config=${BASE_REMAP_CONFIG}
[lidar-pure-ab] pure_config=${PURE_REMAP_CONFIG}
[lidar-pure-ab] duration_sec=${DURATION_SEC}
[lidar-pure-ab] ready_timeout_sec=${PURE_READY_TIMEOUT_SEC}
[lidar-pure-ab] include_cli_hz=${INCLUDE_CLI_HZ}
[lidar-pure-ab] execute=${EXECUTE}
[lidar-pure-ab] This A/B is diagnostic only. Run it while stationary or after stopping navigation.
[lidar-pure-ab] It does not change local_perception_input_profile.env and does not make local perception default to /lidar_points.
EOF

if [[ "${EXECUTE}" != "true" ]]; then
  cat <<EOF
[lidar-pure-ab] Dry run only. To execute:
[lidar-pure-ab]   bash scripts/jetson/runtime_overlay/scripts/run_lidar_trunk_pure_ab.sh --duration-sec ${DURATION_SEC} --execute
[lidar-pure-ab] The script will restart JT128 ingress with nav_output_topic="" and local_output_topic="", collect /lidar/axis_remap_status, then restore the production driver profile.
EOF
  exit 0
fi

mkdir -p "${NJRH_RUNTIME_LOG_DIR}"
RESTORE_REQUESTED=true
trap 'restore_production_driver; rm -rf "${tmp_dir}"' EXIT

before_samples="$(collect_axis_status 6 before)"
if [[ -s "${before_samples}" ]]; then
  print_summary "${before_samples}" "before" || true
fi

echo "[lidar-pure-ab] starting pure-trunk JT128 ingress/remap log=${PURE_LOG}" >&2
nohup env \
  NJRH_FORCE_RESTART_DRIVER=true \
  NJRH_JT128_CANONICAL_POINTCLOUD_REMAP_CONFIG="${PURE_REMAP_CONFIG}" \
  NJRH_POINTCLOUD_AXIS_LOCAL_OUTPUT_TOPIC="" \
  NJRH_POINTCLOUD_AXIS_LOCAL_OUTPUT_STRIDE=1 \
  NJRH_POINTCLOUD_AXIS_LOCAL_OUTPUT_PUBLISH_EVERY_N=1 \
  bash "${SCRIPT_DIR}/run_driver.sh" \
  >"${PURE_LOG}" 2>&1 &
PURE_PID="$!"
echo "[lidar-pure-ab] pure run_driver pid=${PURE_PID}"

wait_for_pure_axis

pure_samples="$(collect_axis_status "${DURATION_SEC}" pure)"
if [[ ! -s "${pure_samples}" ]]; then
  echo "[lidar-pure-ab] FAIL no pure-trunk axis status samples" >&2
  exit 1
fi

status=0
print_summary "${pure_samples}" "pure" || status=1

if [[ "${INCLUDE_CLI_HZ}" == "true" ]]; then
  echo "[lidar-pure-ab] WARNING: CLI hz creates a temporary full-density subscriber."
  cli_file="${tmp_dir}/pure_cli_hz.txt"
  timeout "${DURATION_SEC}" ros2 topic hz /lidar_points >"${cli_file}" 2>&1 || true
  awk '/average rate:/ {rate=$3; sum += rate; count += 1} END {if (count > 0) printf "[lidar-pure-ab] pure CLI /lidar_points average_of_averages=%.3f\n", sum / count; else print "[lidar-pure-ab] pure CLI /lidar_points missing"}' "${cli_file}"
fi

echo "[lidar-pure-ab] Pure-trunk conclusion:"
echo "[lidar-pure-ab]   - If pure publish-side is healthy but production is low, inspect branch fanout/CPU/DDS pressure."
echo "[lidar-pure-ab]   - If pure publish-side is also low, investigate JT128 raw input, axis remap CPU, binary freshness, or thermal throttling."

exit "${status}"
