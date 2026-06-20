#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

exec bash "${SCRIPT_DIR}/run_navigation_pose_error_test.sh" \
  --pose-id delivery_512355 \
  --label delivery_512355_nav_pose_error \
  "$@"
