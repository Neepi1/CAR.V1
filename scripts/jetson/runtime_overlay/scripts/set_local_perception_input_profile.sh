#!/usr/bin/env bash
set -euo pipefail

cat >&2 <<'EOF'
[local-profile] robot_local_perception PointCloud2 obstacle profiles have been removed from production.
[local-profile] Nav2 local marking+clearing now uses /scan through nav2_costmap_2d::ObstacleLayer.
[local-profile] Do not restart robot_local_perception or publish local PointCloud2 obstacle/clearing topics.
EOF
exit 2
