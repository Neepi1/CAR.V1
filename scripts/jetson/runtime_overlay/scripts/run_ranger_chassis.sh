#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/common_env.sh"
source "${SCRIPT_DIR}/cpu_affinity.sh"

CAN_IFACE="${CAN_IFACE:-can0}"
export PUBLISH_ODOM_TF="${PUBLISH_ODOM_TF:-false}"
export ODOM_TOPIC="${ODOM_TOPIC:-/wheel/odom}"
export ODOM_FRAME="${ODOM_FRAME:-odom}"
export BASE_FRAME="${BASE_FRAME:-base_link}"
ROBOT_MODEL="${ROBOT_MODEL:-ranger_mini_v3}"
# Keep navigation base_link at the chassis motion center. The upstream Ranger
# SDK exposes a spin-center offset, but carrying that offset into /wheel/odom
# makes base_link translate during pure yaw and violates the canonical Nav2
# base-frame contract.
RANGER_SPINNING_BASE_TO_CENTER_X="${RANGER_SPINNING_BASE_TO_CENTER_X:-0.0}"
RANGER_SPINNING_BASE_TO_CENTER_Y="${RANGER_SPINNING_BASE_TO_CENTER_Y:-0.0}"
# Field-calibrated SPINNING wheel odom yaw scale from 2026-07-07
# wheel-vs-IMU spin sweep at 0.60 rad/s, +/-30..360 deg, 5 rounds. The scale
# corrects wheel yaw integration at the source before Nav2 consumes local odom.
RANGER_SPINNING_YAW_SCALE_POSITIVE="${RANGER_SPINNING_YAW_SCALE_POSITIVE:-0.976386}"
RANGER_SPINNING_YAW_SCALE_NEGATIVE="${RANGER_SPINNING_YAW_SCALE_NEGATIVE:-0.977672}"
# Keep zero-speed commands in SPINNING until the feedback yaw rate and mode
# transition have settled. This separates spin braking from the later
# SPINNING -> DUAL_ACKERMAN mode exit and prevents the first following drive
# segment from inheriting the chassis stop tail.
RANGER_SPINNING_ZERO_CMD_HOLD_ENABLED="${RANGER_SPINNING_ZERO_CMD_HOLD_ENABLED:-true}"
RANGER_SPINNING_ZERO_CMD_HOLD_WZ_THRESHOLD_RADPS="${RANGER_SPINNING_ZERO_CMD_HOLD_WZ_THRESHOLD_RADPS:-0.030}"
RANGER_DUAL_ACKERMANN_ODOM_USE_FEEDBACK_TWIST="${RANGER_DUAL_ACKERMANN_ODOM_USE_FEEDBACK_TWIST:-true}"
# Odom-only navigation redlines on 2026-07-05 showed /wheel/odom ahead of
# post-goal relocalized base_link mainly during near-straight and terminal
# slow DUAL_ACKERMAN motion. Apply this to near-straight odometry only so
# Ackermann arc curvature remains on the official feedback model.
RANGER_DUAL_ACKERMANN_LINEAR_ODOM_SCALE="${RANGER_DUAL_ACKERMANN_LINEAR_ODOM_SCALE:-0.960}"
RANGER_DUAL_ACKERMANN_LINEAR_ODOM_SCALE_MAX_ABS_YAW_RATE="${RANGER_DUAL_ACKERMANN_LINEAR_ODOM_SCALE_MAX_ABS_YAW_RATE:-0.060}"
# Field-calibrated DUAL_ACKERMAN yaw feedback scale for high-speed
# near-straight micro-corrections. 2026-07-06 two-leg audits showed the
# chassis feedback angular velocity under-integrates yaw when |wz| is small,
# while larger Ackermann arcs already match the IMU, so only near-straight
# odometry yaw is scaled.
RANGER_DUAL_ACKERMANN_YAW_SCALE_MAX_ABS_YAW_RATE="${RANGER_DUAL_ACKERMANN_YAW_SCALE_MAX_ABS_YAW_RATE:-0.060}"
RANGER_DUAL_ACKERMANN_NEAR_STRAIGHT_YAW_SCALE_POSITIVE="${RANGER_DUAL_ACKERMANN_NEAR_STRAIGHT_YAW_SCALE_POSITIVE:-1.120}"
RANGER_DUAL_ACKERMANN_NEAR_STRAIGHT_YAW_SCALE_NEGATIVE="${RANGER_DUAL_ACKERMANN_NEAR_STRAIGHT_YAW_SCALE_NEGATIVE:-1.120}"
# Field-calibrated straight-drive yaw drift term. This covers the remaining
# case where feedback yaw rate is nearly zero but the robot still accumulates a
# small clockwise heading drift at speed. Units are radian yaw per meter driven.
RANGER_DUAL_ACKERMANN_YAW_BIAS_MAX_ABS_YAW_RATE="${RANGER_DUAL_ACKERMANN_YAW_BIAS_MAX_ABS_YAW_RATE:-0.030}"
RANGER_DUAL_ACKERMANN_NEAR_STRAIGHT_YAW_BIAS_PER_METER="${RANGER_DUAL_ACKERMANN_NEAR_STRAIGHT_YAW_BIAS_PER_METER:--0.0041}"
LOCK_DIR="/tmp/njrh_ranger_chassis_${CAN_IFACE}.lock"
REAL_NODE_PATTERN="/ranger_base/lib/ranger_base/ranger_base_node.*port_name:=${CAN_IFACE}"
WRAPPER_PATTERN="ros2 run ranger_base ranger_base_node.*port_name:=${CAN_IFACE}"

monitor_existing_ranger() {
  echo "[runtime-overlay] ranger_base_node already owns ${CAN_IFACE}; monitoring existing instance instead of starting a duplicate" >&2
  while pgrep -f "${REAL_NODE_PATTERN}" >/dev/null || pgrep -f "${WRAPPER_PATTERN}" >/dev/null; do
    sleep 2
  done
  echo "[runtime-overlay] existing ranger_base_node for ${CAN_IFACE} exited" >&2
}

if ! mkdir "${LOCK_DIR}" 2>/dev/null; then
  if pgrep -f "${REAL_NODE_PATTERN}" >/dev/null || pgrep -f "${WRAPPER_PATTERN}" >/dev/null; then
    monitor_existing_ranger
    exit 0
  fi
  echo "[runtime-overlay] removing stale Ranger chassis lock: ${LOCK_DIR}" >&2
  rm -rf "${LOCK_DIR}"
  mkdir "${LOCK_DIR}" 2>/dev/null || {
    echo "[runtime-overlay] failed to acquire Ranger chassis lock: ${LOCK_DIR}" >&2
    exit 1
  }
fi

cleanup_lock() {
  rm -rf "${LOCK_DIR}" 2>/dev/null || true
}
chassis_pid=""

cleanup() {
  trap - EXIT INT TERM
  if [[ -n "${chassis_pid}" ]]; then
    kill -INT "${chassis_pid}" 2>/dev/null || true
    wait "${chassis_pid}" 2>/dev/null || true
  fi
  cleanup_lock
}

on_signal() {
  cleanup
  exit 130
}

trap cleanup EXIT
trap on_signal INT TERM

njrh_run_affined ranger_base_node ros2 run ranger_base ranger_base_node \
  --ros-args \
  -p use_sim_time:=false \
  -p "port_name:=${CAN_IFACE}" \
  -p "odom_frame:=${ODOM_FRAME}" \
  -p "base_frame:=${BASE_FRAME}" \
  -p "odom_topic_name:=${ODOM_TOPIC}" \
  -p simulated_robot:=false \
  -p "publish_odom_tf:=${PUBLISH_ODOM_TF}" \
  -p "robot_model:=${ROBOT_MODEL}" \
  -p "dual_ackermann_odom_use_feedback_twist:=${RANGER_DUAL_ACKERMANN_ODOM_USE_FEEDBACK_TWIST}" \
  -p "dual_ackermann_linear_odom_scale:=${RANGER_DUAL_ACKERMANN_LINEAR_ODOM_SCALE}" \
  -p "dual_ackermann_linear_odom_scale_max_abs_yaw_rate:=${RANGER_DUAL_ACKERMANN_LINEAR_ODOM_SCALE_MAX_ABS_YAW_RATE}" \
  -p "dual_ackermann_yaw_scale_max_abs_yaw_rate:=${RANGER_DUAL_ACKERMANN_YAW_SCALE_MAX_ABS_YAW_RATE}" \
  -p "dual_ackermann_near_straight_yaw_scale_positive:=${RANGER_DUAL_ACKERMANN_NEAR_STRAIGHT_YAW_SCALE_POSITIVE}" \
  -p "dual_ackermann_near_straight_yaw_scale_negative:=${RANGER_DUAL_ACKERMANN_NEAR_STRAIGHT_YAW_SCALE_NEGATIVE}" \
  -p "dual_ackermann_yaw_bias_max_abs_yaw_rate:=${RANGER_DUAL_ACKERMANN_YAW_BIAS_MAX_ABS_YAW_RATE}" \
  -p "dual_ackermann_near_straight_yaw_bias_per_meter:=${RANGER_DUAL_ACKERMANN_NEAR_STRAIGHT_YAW_BIAS_PER_METER}" \
  -p "spinning_base_to_center_x:=${RANGER_SPINNING_BASE_TO_CENTER_X}" \
  -p "spinning_base_to_center_y:=${RANGER_SPINNING_BASE_TO_CENTER_Y}" \
  -p "spinning_yaw_scale_positive:=${RANGER_SPINNING_YAW_SCALE_POSITIVE}" \
  -p "spinning_yaw_scale_negative:=${RANGER_SPINNING_YAW_SCALE_NEGATIVE}" \
  -p "spinning_zero_cmd_hold_enabled:=${RANGER_SPINNING_ZERO_CMD_HOLD_ENABLED}" \
  -p "spinning_zero_cmd_hold_wz_threshold_radps:=${RANGER_SPINNING_ZERO_CMD_HOLD_WZ_THRESHOLD_RADPS}" \
  -r /tf:=/tf_ranger_internal \
  -r /tf_static:=/tf_static_ranger_internal &
chassis_pid=$!
set +e
wait "${chassis_pid}"
chassis_rc=$?
set -e
echo "[runtime-overlay] ranger_base_node exited rc=${chassis_rc}" >&2
exit "${chassis_rc}"
