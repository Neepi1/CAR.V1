#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/common_env.sh"
source "${SCRIPT_DIR}/cpu_affinity.sh"
REPO_ROOT="$(cd "${NJRH_OVERLAY_ROOT}/../../.." && pwd)"

if [[ -f "${REPO_ROOT}/install/local_setup.bash" ]]; then
  # This package is repository-owned, not part of the upstream isaac_ros-dev workspace.
  set +u
  source "${REPO_ROOT}/install/local_setup.bash"
  set -u
fi

PARAMS_FILE="${RANGER_MINI3_MODE_CONTROLLER_PARAMS_FILE:-${NJRH_OVERLAY_ROOT}/config/ranger_mini3_mode_controller.yaml}"
[[ -f "${PARAMS_FILE}" ]] || {
  echo "[runtime-overlay] ranger mini3 mode controller params file missing: ${PARAMS_FILE}" >&2
  exit 1
}

if ros2 pkg prefix ranger_mini3_mode_controller >/dev/null 2>&1; then
  njrh_exec_affined ranger_mini3_mode_controller \
    ros2 run ranger_mini3_mode_controller mode_controller_node --ros-args --params-file "${PARAMS_FILE}"
fi

echo "[runtime-overlay] ranger_mini3_mode_controller is not built. Build the C++ package first:" >&2
echo "  colcon build --packages-select ranger_mini3_mode_controller" >&2
exit 1
