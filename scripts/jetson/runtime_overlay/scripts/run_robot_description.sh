#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/common_env.sh"

CONFIG_FILE="${ROBOT_DESCRIPTION_CONFIG_FILE:-${NJRH_OVERLAY_ROOT}/config/sensors.yaml}"
[[ -f "${CONFIG_FILE}" ]] || {
  echo "[runtime-overlay] robot description config file missing: ${CONFIG_FILE}" >&2
  exit 1
}

# Keep static TF publication single-sourced even when this helper is started
# manually during debugging. The canonical stack also kills old publishers, but
# this makes the helper itself safe to re-run.
pkill -INT -f "robot_description_static_tf_node" 2>/dev/null || true
sleep 1
pkill -9 -f "robot_description_static_tf_node" 2>/dev/null || true

read_config_value() {
  local key="$1"
  awk -F':' -v target="${key}" '
    $1 ~ "^[[:space:]]*" target "[[:space:]]*$" {
      value=$2
      sub(/^[[:space:]]+/, "", value)
      sub(/[[:space:]]+$/, "", value)
      print value
      exit
    }
  ' "${CONFIG_FILE}"
}

# Publish the canonical static TF chain from one node so Euler angles are
# interpreted in explicit roll/pitch/yaw order instead of CLI positional order.
NODE_BIN="${NJRH_PROJECT_ROOT}/install/robot_description/lib/robot_description/robot_description_static_tf_node"
[[ -x "${NODE_BIN}" ]] || {
  echo "[runtime-overlay] compiled robot description static TF node missing or not executable: ${NODE_BIN}" >&2
  echo "[runtime-overlay] build robot_description; Python fallback has been removed." >&2
  exit 1
}

exec "${NODE_BIN}" \
  --ros-args \
  -p "config_file:=${CONFIG_FILE}"
