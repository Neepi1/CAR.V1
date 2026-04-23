#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/common_env.sh"

export PUBLISH_LIDAR_TF="${PUBLISH_LIDAR_TF:-false}"

exec bash "$(require_upstream_script run_jt128_2d_mapping.sh)"
