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

  local profile="${NJRH_LOCAL_PERCEPTION_INPUT_PROFILE:-disabled}"
  local branch_topic=""
  local branch_stride="1"
  local branch_publish_every_n="1"

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
    disabled)
      profile_input_topic=""
      profile_axis_local_output_topic=""
      profile_axis_local_output_stride="1"
      profile_axis_local_output_publish_every_n="1"
      ;;
    local_branch|trunk)
      echo "[runtime-overlay] retired NJRH_LOCAL_PERCEPTION_INPUT_PROFILE=${profile}; forcing disabled because Nav2 uses /scan" >&2
      profile="disabled"
      profile_input_topic=""
      profile_axis_local_output_topic=""
      profile_axis_local_output_stride="1"
      profile_axis_local_output_publish_every_n="1"
      ;;
    *)
      echo "[runtime-overlay] invalid NJRH_LOCAL_PERCEPTION_INPUT_PROFILE=${profile}; expected disabled" >&2
      return 2
      ;;
  esac

  RESOLVED_LOCAL_PERCEPTION_INPUT_TOPIC="${profile_input_topic}"
  RESOLVED_LOCAL_PERCEPTION_INPUT_TOPIC_SOURCE="disabled"

  RESOLVED_AXIS_LOCAL_OUTPUT_TOPIC="${profile_axis_local_output_topic}"
  RESOLVED_AXIS_LOCAL_OUTPUT_TOPIC_SOURCE="disabled"

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
  if [[ "${NJRH_LOCAL_PERCEPTION_INPUT_PROFILE}" == "disabled" ]]; then
    echo "[runtime-overlay] local perception PointCloud2 obstacle branch is disabled; /scan is the Nav2 local obstacle source" >&2
    return 0
  fi
  echo "[runtime-overlay] local perception input_topic=${RESOLVED_LOCAL_PERCEPTION_INPUT_TOPIC} source=${RESOLVED_LOCAL_PERCEPTION_INPUT_TOPIC_SOURCE}" >&2
  echo "[runtime-overlay] axis local_output_topic=${RESOLVED_AXIS_LOCAL_OUTPUT_TOPIC:-<disabled>} source=${RESOLVED_AXIS_LOCAL_OUTPUT_TOPIC_SOURCE}" >&2
  echo "[runtime-overlay] axis local_output_stride=${RESOLVED_AXIS_LOCAL_OUTPUT_STRIDE} source=${RESOLVED_AXIS_LOCAL_OUTPUT_STRIDE_SOURCE}" >&2
  echo "[runtime-overlay] axis local_output_publish_every_n=${RESOLVED_AXIS_LOCAL_OUTPUT_PUBLISH_EVERY_N} source=${RESOLVED_AXIS_LOCAL_OUTPUT_PUBLISH_EVERY_N_SOURCE}" >&2
}
