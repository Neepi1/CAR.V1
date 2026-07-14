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
# Positive SPINNING yaw scale came from the JT128 IMU sweep. The negative scale
# was finalized by a correction-frozen two-point A/B that bracketed 0.977672
# and 1.0, then repeated successfully at 0.986. Both scales correct wheel yaw
# integration at the source before Nav2 consumes local odom.
RANGER_SPINNING_YAW_SCALE_POSITIVE="${RANGER_SPINNING_YAW_SCALE_POSITIVE:-0.976386}"
RANGER_SPINNING_YAW_SCALE_NEGATIVE="${RANGER_SPINNING_YAW_SCALE_NEGATIVE:-0.986000}"
# Keep zero-speed commands in SPINNING until the feedback yaw rate and mode
# transition have settled. This separates spin braking from the later
# SPINNING -> DUAL_ACKERMAN mode exit and prevents the first following drive
# segment from inheriting the chassis stop tail.
RANGER_SPINNING_ZERO_CMD_HOLD_ENABLED="${RANGER_SPINNING_ZERO_CMD_HOLD_ENABLED:-true}"
RANGER_SPINNING_ZERO_CMD_HOLD_WZ_THRESHOLD_RADPS="${RANGER_SPINNING_ZERO_CMD_HOLD_WZ_THRESHOLD_RADPS:-0.030}"
# The chassis core owns all mode transitions. It holds zero, waits for physical
# stop, sends the mode request, and releases the latest command only after the
# firmware reports the requested mode with mode_changing=0.
RANGER_MODE_SWITCH_HANDSHAKE_ENABLED="${RANGER_MODE_SWITCH_HANDSHAKE_ENABLED:-true}"
RANGER_MODE_SWITCH_RETRY_PERIOD_SEC="${RANGER_MODE_SWITCH_RETRY_PERIOD_SEC:-0.10}"
RANGER_MODE_SWITCH_TIMEOUT_SEC="${RANGER_MODE_SWITCH_TIMEOUT_SEC:-2.0}"
RANGER_MODE_SWITCH_STABLE_DURATION_SEC="${RANGER_MODE_SWITCH_STABLE_DURATION_SEC:-0.15}"
RANGER_MODE_SWITCH_STOP_LINEAR_THRESHOLD_MPS="${RANGER_MODE_SWITCH_STOP_LINEAR_THRESHOLD_MPS:-0.02}"
RANGER_MODE_SWITCH_STOP_ANGULAR_THRESHOLD_RADPS="${RANGER_MODE_SWITCH_STOP_ANGULAR_THRESHOLD_RADPS:-0.03}"
RANGER_MODE_STATUS_TOPIC="${RANGER_MODE_STATUS_TOPIC:-/ranger_base/status}"
RANGER_LEGACY_MODE_STATUS_TOPIC="${RANGER_LEGACY_MODE_STATUS_TOPIC:-/ranger_mini3_mode_controller/status}"
RANGER_DUAL_ACKERMANN_ODOM_USE_FEEDBACK_TWIST="${RANGER_DUAL_ACKERMANN_ODOM_USE_FEEDBACK_TWIST:-true}"
# The 2026-07-13 calibration2/calibration3 shadow-mode round trip kept
# map->odom fixed and accepted no AMCL corrections. Physical endpoint offsets
# produced independent forward/reverse scale fits of 0.99095 and 0.99110, so
# use the rounded common value. Yaw and lateral residuals remain separate.
RANGER_DUAL_ACKERMANN_LINEAR_ODOM_SCALE="${RANGER_DUAL_ACKERMANN_LINEAR_ODOM_SCALE:-0.991}"
RANGER_DUAL_ACKERMANN_LINEAR_ODOM_SCALE_MAX_ABS_YAW_RATE="${RANGER_DUAL_ACKERMANN_LINEAR_ODOM_SCALE_MAX_ABS_YAW_RATE:-0.060}"
# Sign-specific near-straight DUAL_ACKERMAN scales fitted from the 2026-07-13
# correction-frozen six-leg replay. Positive feedback yaw accumulated 41.044
# deg while the projected LiDAR IMU accumulated 42.733 deg; negative feedback
# yaw accumulated -77.930 deg while the IMU accumulated -76.180 deg.
RANGER_DUAL_ACKERMANN_YAW_SCALE_MAX_ABS_YAW_RATE="${RANGER_DUAL_ACKERMANN_YAW_SCALE_MAX_ABS_YAW_RATE:-0.060}"
RANGER_DUAL_ACKERMANN_NEAR_STRAIGHT_YAW_SCALE_POSITIVE="${RANGER_DUAL_ACKERMANN_NEAR_STRAIGHT_YAW_SCALE_POSITIVE:-1.041151}"
RANGER_DUAL_ACKERMANN_NEAR_STRAIGHT_YAW_SCALE_NEGATIVE="${RANGER_DUAL_ACKERMANN_NEAR_STRAIGHT_YAW_SCALE_NEGATIVE:-0.977549}"
# Units are radian yaw per meter driven. Keep the bias neutral unless a
# sign-consistent residual is reproduced independently of localization.
RANGER_DUAL_ACKERMANN_YAW_BIAS_MAX_ABS_YAW_RATE="${RANGER_DUAL_ACKERMANN_YAW_BIAS_MAX_ABS_YAW_RATE:-0.030}"
RANGER_DUAL_ACKERMANN_NEAR_STRAIGHT_YAW_BIAS_PER_METER="${RANGER_DUAL_ACKERMANN_NEAR_STRAIGHT_YAW_BIAS_PER_METER:-0.0}"
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
  -p "mode_switch_handshake_enabled:=${RANGER_MODE_SWITCH_HANDSHAKE_ENABLED}" \
  -p "mode_switch_retry_period_sec:=${RANGER_MODE_SWITCH_RETRY_PERIOD_SEC}" \
  -p "mode_switch_timeout_sec:=${RANGER_MODE_SWITCH_TIMEOUT_SEC}" \
  -p "mode_switch_stable_duration_sec:=${RANGER_MODE_SWITCH_STABLE_DURATION_SEC}" \
  -p "mode_switch_stop_linear_threshold_mps:=${RANGER_MODE_SWITCH_STOP_LINEAR_THRESHOLD_MPS}" \
  -p "mode_switch_stop_angular_threshold_radps:=${RANGER_MODE_SWITCH_STOP_ANGULAR_THRESHOLD_RADPS}" \
  -p "mode_status_topic:=${RANGER_MODE_STATUS_TOPIC}" \
  -p "legacy_mode_status_topic:=${RANGER_LEGACY_MODE_STATUS_TOPIC}" \
  -r /tf:=/tf_ranger_internal \
  -r /tf_static:=/tf_static_ranger_internal &
chassis_pid=$!
set +e
wait "${chassis_pid}"
chassis_rc=$?
set -e
echo "[runtime-overlay] ranger_base_node exited rc=${chassis_rc}" >&2
exit "${chassis_rc}"
