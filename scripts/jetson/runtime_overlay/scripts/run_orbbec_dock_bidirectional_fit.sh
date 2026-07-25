#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/common_env.sh"
source "${SCRIPT_DIR}/cpu_affinity.sh"

set +u
source /opt/ros/humble/setup.bash
source "${NJRH_PROJECT_ROOT}/install/setup.bash"
set -u

njrh_exec_affined docking_vision python3 \
  "${SCRIPT_DIR}/orbbec_dock_bidirectional_fit.py" "$@"
