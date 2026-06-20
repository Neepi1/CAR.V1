#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
echo "[runtime-overlay] robot_local_perception PointCloud2 obstacle pipeline has been removed from production." >&2
echo "[runtime-overlay] Nav2 local marking+clearing now uses /scan through nav2_costmap_2d::ObstacleLayer." >&2
echo "[runtime-overlay] This script is intentionally disabled to prevent retired local PointCloud2 obstacle topics from reappearing." >&2
exit 2
