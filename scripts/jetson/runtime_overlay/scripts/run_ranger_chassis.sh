#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/common_env.sh"

CAN_IFACE="${CAN_IFACE:-can0}"
export PUBLISH_ODOM_TF="${PUBLISH_ODOM_TF:-false}"
export ODOM_TOPIC="${ODOM_TOPIC:-/wheel/odom}"
export ODOM_FRAME="${ODOM_FRAME:-odom}"
export BASE_FRAME="${BASE_FRAME:-base_link}"
ROBOT_MODEL="${ROBOT_MODEL:-ranger_mini_v3}"

exec ros2 run ranger_base ranger_base_node \
  --ros-args \
  -p use_sim_time:=false \
  -p "port_name:=${CAN_IFACE}" \
  -p "odom_frame:=${ODOM_FRAME}" \
  -p "base_frame:=${BASE_FRAME}" \
  -p "odom_topic_name:=${ODOM_TOPIC}" \
  -p simulated_robot:=false \
  -p "publish_odom_tf:=${PUBLISH_ODOM_TF}" \
  -p "robot_model:=${ROBOT_MODEL}" \
  -r /tf:=/tf_ranger_internal \
  -r /tf_static:=/tf_static_ranger_internal
