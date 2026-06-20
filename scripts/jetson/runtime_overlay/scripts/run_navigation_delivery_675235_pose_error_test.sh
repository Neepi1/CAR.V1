#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

exec bash "${SCRIPT_DIR}/run_navigation_pose_error_test.sh" \
  --pose-id delivery_675235 \
  --label delivery_675235_nav_pose_error \
  "$@"
