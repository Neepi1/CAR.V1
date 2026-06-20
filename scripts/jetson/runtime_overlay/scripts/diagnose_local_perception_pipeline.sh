#!/usr/bin/env bash
set -euo pipefail

cat >&2 <<'EOF'
[local-perception-diag] The robot_local_perception PointCloud2 obstacle pipeline has been removed from production.
[local-perception-diag] Nav2 local marking and clearing now use /scan through nav2_costmap_2d::ObstacleLayer.
[local-perception-diag] This diagnostic is intentionally disabled so it cannot subscribe to retired /perception obstacle or clearing topics.
EOF
exit 2
