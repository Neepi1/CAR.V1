#!/usr/bin/env bash
# Shared profile resolver for JT128 pointcloud local-perception input selection.
# Source this file from runtime scripts; it does not start or stop processes.

njrh_int_ge() {
  local value="$1"
  local minimum="$2"
  [[ "${value}" =~ ^[0-9]+$ ]] || return 1
  (( value >= minimum ))
}

njrh_load_local_perception_input_profile() {
  local profile_file="${NJRH_LOCAL_PERCEPTION_INPUT_PROFILE_FILE:-${NJRH_OVERLAY_ROOT}/config/local_perception_input_profile.env}"
  local profile_source="default"

  if [[ -z "${NJRH_LOCAL_PERCEPTION_INPUT_PROFILE:-}" && -f "${profile_file}" ]]; then
    # shellcheck source=/dev/null
    source "${profile_file}"
    profile_source="${profile_file}"
  elif [[ -n "${NJRH_LOCAL_PERCEPTION_INPUT_PROFILE:-}" ]]; then
    profile_source="environment"
  fi

  local profile="${NJRH_LOCAL_PERCEPTION_INPUT_PROFILE:-local_branch}"
  local branch_topic="${NJRH_LOCAL_PERCEPTION_LOCAL_BRANCH_TOPIC:-/_internal/lidar_points_local}"
  local branch_stride="${NJRH_LOCAL_PERCEPTION_LOCAL_BRANCH_STRIDE:-2}"
  local branch_publish_every_n="${NJRH_LOCAL_PERCEPTION_LOCAL_BRANCH_PUBLISH_EVERY_N:-1}"

  if ! njrh_int_ge "${branch_stride}" 1; then
    echo "[runtime-overlay] invalid NJRH_LOCAL_PERCEPTION_LOCAL_BRANCH_STRIDE=${branch_stride}" >&2
    return 2
  fi
  if ! njrh_int_ge "${branch_publish_every_n}" 1; then
    echo "[runtime-overlay] invalid NJRH_LOCAL_PERCEPTION_LOCAL_BRANCH_PUBLISH_EVERY_N=${branch_publish_every_n}" >&2
    return 2
  fi

  local profile_input_topic=""
  local profile_axis_local_output_topic=""
  local profile_axis_local_output_stride="1"
  local profile_axis_local_output_publish_every_n="1"

  case "${profile}" in
    local_branch)
      profile_input_topic="${branch_topic}"
      profile_axis_local_output_topic="${branch_topic}"
      profile_axis_local_output_stride="${branch_stride}"
      profile_axis_local_output_publish_every_n="${branch_publish_every_n}"
      ;;
    trunk)
      profile_input_topic="/lidar_points"
      profile_axis_local_output_topic=""
      profile_axis_local_output_stride="1"
      profile_axis_local_output_publish_every_n="1"
      ;;
    *)
      echo "[runtime-overlay] invalid NJRH_LOCAL_PERCEPTION_INPUT_PROFILE=${profile}; expected local_branch or trunk" >&2
      return 2
      ;;
  esac

  if [[ "${ROBOT_LOCAL_PERCEPTION_INPUT_TOPIC+x}" == "x" ]]; then
    RESOLVED_LOCAL_PERCEPTION_INPUT_TOPIC="${ROBOT_LOCAL_PERCEPTION_INPUT_TOPIC}"
    RESOLVED_LOCAL_PERCEPTION_INPUT_TOPIC_SOURCE="ROBOT_LOCAL_PERCEPTION_INPUT_TOPIC"
  else
    RESOLVED_LOCAL_PERCEPTION_INPUT_TOPIC="${profile_input_topic}"
    RESOLVED_LOCAL_PERCEPTION_INPUT_TOPIC_SOURCE="profile"
  fi

  if [[ "${NJRH_POINTCLOUD_AXIS_LOCAL_OUTPUT_TOPIC+x}" == "x" ]]; then
    RESOLVED_AXIS_LOCAL_OUTPUT_TOPIC="${NJRH_POINTCLOUD_AXIS_LOCAL_OUTPUT_TOPIC}"
    RESOLVED_AXIS_LOCAL_OUTPUT_TOPIC_SOURCE="NJRH_POINTCLOUD_AXIS_LOCAL_OUTPUT_TOPIC"
  else
    RESOLVED_AXIS_LOCAL_OUTPUT_TOPIC="${profile_axis_local_output_topic}"
    RESOLVED_AXIS_LOCAL_OUTPUT_TOPIC_SOURCE="profile"
  fi

  if [[ "${NJRH_POINTCLOUD_AXIS_LOCAL_OUTPUT_STRIDE+x}" == "x" ]]; then
    RESOLVED_AXIS_LOCAL_OUTPUT_STRIDE="${NJRH_POINTCLOUD_AXIS_LOCAL_OUTPUT_STRIDE}"
    RESOLVED_AXIS_LOCAL_OUTPUT_STRIDE_SOURCE="NJRH_POINTCLOUD_AXIS_LOCAL_OUTPUT_STRIDE"
  else
    RESOLVED_AXIS_LOCAL_OUTPUT_STRIDE="${profile_axis_local_output_stride}"
    RESOLVED_AXIS_LOCAL_OUTPUT_STRIDE_SOURCE="profile"
  fi

  if [[ "${NJRH_POINTCLOUD_AXIS_LOCAL_OUTPUT_PUBLISH_EVERY_N+x}" == "x" ]]; then
    RESOLVED_AXIS_LOCAL_OUTPUT_PUBLISH_EVERY_N="${NJRH_POINTCLOUD_AXIS_LOCAL_OUTPUT_PUBLISH_EVERY_N}"
    RESOLVED_AXIS_LOCAL_OUTPUT_PUBLISH_EVERY_N_SOURCE="NJRH_POINTCLOUD_AXIS_LOCAL_OUTPUT_PUBLISH_EVERY_N"
  else
    RESOLVED_AXIS_LOCAL_OUTPUT_PUBLISH_EVERY_N="${profile_axis_local_output_publish_every_n}"
    RESOLVED_AXIS_LOCAL_OUTPUT_PUBLISH_EVERY_N_SOURCE="profile"
  fi

  if ! njrh_int_ge "${RESOLVED_AXIS_LOCAL_OUTPUT_STRIDE}" 1; then
    echo "[runtime-overlay] invalid resolved local_output_stride=${RESOLVED_AXIS_LOCAL_OUTPUT_STRIDE}" >&2
    return 2
  fi
  if ! njrh_int_ge "${RESOLVED_AXIS_LOCAL_OUTPUT_PUBLISH_EVERY_N}" 1; then
    echo "[runtime-overlay] invalid resolved local_output_publish_every_n=${RESOLVED_AXIS_LOCAL_OUTPUT_PUBLISH_EVERY_N}" >&2
    return 2
  fi

  export NJRH_LOCAL_PERCEPTION_INPUT_PROFILE="${profile}"
  export NJRH_LOCAL_PERCEPTION_INPUT_PROFILE_SOURCE="${profile_source}"
  export NJRH_LOCAL_PERCEPTION_LOCAL_BRANCH_TOPIC="${branch_topic}"
  export NJRH_LOCAL_PERCEPTION_LOCAL_BRANCH_STRIDE="${branch_stride}"
  export NJRH_LOCAL_PERCEPTION_LOCAL_BRANCH_PUBLISH_EVERY_N="${branch_publish_every_n}"
  export RESOLVED_LOCAL_PERCEPTION_INPUT_TOPIC
  export RESOLVED_LOCAL_PERCEPTION_INPUT_TOPIC_SOURCE
  export RESOLVED_AXIS_LOCAL_OUTPUT_TOPIC
  export RESOLVED_AXIS_LOCAL_OUTPUT_TOPIC_SOURCE
  export RESOLVED_AXIS_LOCAL_OUTPUT_STRIDE
  export RESOLVED_AXIS_LOCAL_OUTPUT_STRIDE_SOURCE
  export RESOLVED_AXIS_LOCAL_OUTPUT_PUBLISH_EVERY_N
  export RESOLVED_AXIS_LOCAL_OUTPUT_PUBLISH_EVERY_N_SOURCE
}

njrh_print_local_perception_profile() {
  echo "[runtime-overlay] selected local perception profile=${NJRH_LOCAL_PERCEPTION_INPUT_PROFILE} source=${NJRH_LOCAL_PERCEPTION_INPUT_PROFILE_SOURCE}" >&2
  echo "[runtime-overlay] local perception input_topic=${RESOLVED_LOCAL_PERCEPTION_INPUT_TOPIC} source=${RESOLVED_LOCAL_PERCEPTION_INPUT_TOPIC_SOURCE}" >&2
  echo "[runtime-overlay] axis local_output_topic=${RESOLVED_AXIS_LOCAL_OUTPUT_TOPIC:-<disabled>} source=${RESOLVED_AXIS_LOCAL_OUTPUT_TOPIC_SOURCE}" >&2
  echo "[runtime-overlay] axis local_output_stride=${RESOLVED_AXIS_LOCAL_OUTPUT_STRIDE} source=${RESOLVED_AXIS_LOCAL_OUTPUT_STRIDE_SOURCE}" >&2
  echo "[runtime-overlay] axis local_output_publish_every_n=${RESOLVED_AXIS_LOCAL_OUTPUT_PUBLISH_EVERY_N} source=${RESOLVED_AXIS_LOCAL_OUTPUT_PUBLISH_EVERY_N_SOURCE}" >&2
}

