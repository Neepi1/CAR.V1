#!/usr/bin/env bash
set -euo pipefail
umask 0002

export USER="$(id -un)"
export HOME="$(getent passwd "$(id -u)" | cut -d: -f6)"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export FASTDDS_BUILTIN_TRANSPORTS="${FASTDDS_BUILTIN_TRANSPORTS:-UDPv4}"

configure_fastdds_interface_whitelist() {
  [[ "${NJRH_FASTDDS_PROFILE_ENABLED:-true}" == "true" ]] || return 0

  local explicit_profile="${FASTRTPS_DEFAULT_PROFILES_FILE:-${FASTDDS_DEFAULT_PROFILES_FILE:-}}"
  if [[ -n "${explicit_profile}" ]]; then
    export FASTRTPS_DEFAULT_PROFILES_FILE="${explicit_profile}"
    export FASTDDS_DEFAULT_PROFILES_FILE="${explicit_profile}"
    export SKIP_DEFAULT_XML="${SKIP_DEFAULT_XML:-1}"
    return 0
  fi

  local allow_ifaces="${NJRH_FASTDDS_ALLOWED_INTERFACES:-lo,wlan0}"
  local profile_file="${NJRH_FASTDDS_PROFILE_FILE:-/tmp/njrh_fastdds_profile.xml}"
  local addresses=()
  local iface
  local addr

  if [[ -e "${profile_file}" && ! -w "${profile_file}" ]]; then
    profile_file="/tmp/njrh_fastdds_profile_$(id -u).xml"
  fi

  for iface in ${allow_ifaces//,/ }; do
    iface="${iface//[[:space:]]/}"
    [[ -n "${iface}" ]] || continue
    if [[ "${iface}" == "lo" ]]; then
      addresses+=("127.0.0.1")
      continue
    fi
    addr="$(ip -o -4 addr show dev "${iface}" 2>/dev/null | awk '{split($4, a, "/"); print a[1]; exit}')"
    [[ -n "${addr}" ]] && addresses+=("${addr}")
  done

  if [[ "${#addresses[@]}" -le 1 ]]; then
    local default_iface
    default_iface="$(ip route show default 0.0.0.0/0 2>/dev/null | awk '{print $5; exit}')"
    if [[ -n "${default_iface}" && "${allow_ifaces}" != *"${default_iface}"* ]]; then
      addr="$(ip -o -4 addr show dev "${default_iface}" 2>/dev/null | awk '{split($4, a, "/"); print a[1]; exit}')"
      [[ -n "${addr}" ]] && addresses+=("${addr}")
    fi
  fi

  # Avoid writing a restrictive profile if no routable robot interface was found.
  [[ "${#addresses[@]}" -gt 1 ]] || return 0

  mkdir -p "$(dirname "${profile_file}")"
  {
    cat <<'XML'
<?xml version="1.0" encoding="UTF-8"?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
  <transport_descriptors>
    <transport_descriptor>
      <transport_id>njrh_udp_transport</transport_id>
      <type>UDPv4</type>
      <interfaceWhiteList>
XML
    local address
    for address in "${addresses[@]}"; do
      printf '        <address>%s</address>\n' "${address}"
    done
    cat <<'XML'
      </interfaceWhiteList>
    </transport_descriptor>
  </transport_descriptors>
  <participant profile_name="njrh_default_participant" is_default_profile="true">
    <rtps>
      <userTransports>
        <transport_id>njrh_udp_transport</transport_id>
      </userTransports>
      <useBuiltinTransports>false</useBuiltinTransports>
    </rtps>
  </participant>
</profiles>
XML
  } >"${profile_file}"

  export FASTRTPS_DEFAULT_PROFILES_FILE="${profile_file}"
  export FASTDDS_DEFAULT_PROFILES_FILE="${profile_file}"
  export SKIP_DEFAULT_XML="${SKIP_DEFAULT_XML:-1}"
  export NJRH_FASTDDS_ALLOWED_ADDRESSES="${addresses[*]}"
}

configure_fastdds_interface_whitelist

OVERLAY_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PROJECT_ROOT="${NJRH_PROJECT_ROOT:-/workspaces/njrh-v3/workspace1}"
UPSTREAM_ROOT="${NJRH_UPSTREAM_ROOT:-/workspaces/isaac_ros-dev}"
UPSTREAM_HOST_ROOT="${NJRH_UPSTREAM_HOST_ROOT:-/home/nvidia/workspaces/isaac_ros-dev}"
UPSTREAM_SCRIPTS="${UPSTREAM_ROOT}/scripts"
UPSTREAM_WS="${UPSTREAM_ROOT}/ros2_ws"

export NJRH_OVERLAY_ROOT="$OVERLAY_ROOT"
export NJRH_PROJECT_ROOT="$PROJECT_ROOT"
export NJRH_FASTLIO_PATCHED_OVERLAY="${NJRH_FASTLIO_PATCHED_OVERLAY:-${PROJECT_ROOT}/.runtime/fast_lio_overlay/install}"
export NJRH_JT128_NAV_TOOLS_PATCHED_OVERLAY="${NJRH_JT128_NAV_TOOLS_PATCHED_OVERLAY:-${PROJECT_ROOT}/.runtime/jt128_nav_tools_overlay/install}"
export NJRH_UPSTREAM_ROOT="$UPSTREAM_ROOT"
export NJRH_UPSTREAM_HOST_ROOT="$UPSTREAM_HOST_ROOT"
export NJRH_MAPS_DIR="${NJRH_MAPS_DIR:-${OVERLAY_ROOT}/maps}"
export NJRH_MAPS3D_DIR="${NJRH_MAPS3D_DIR:-${OVERLAY_ROOT}/maps3d}"
export NJRH_RELEASE_ASSETS_DIR="${NJRH_RELEASE_ASSETS_DIR:-${PROJECT_ROOT}/maps_release}"
export NJRH_WAYPOINTS_DIR="${NJRH_WAYPOINTS_DIR:-${OVERLAY_ROOT}/waypoints}"
export NJRH_RUNTIME_LOG_DIR="${NJRH_RUNTIME_LOG_DIR:-${OVERLAY_ROOT}/web_dashboard/runtime_logs}"

LOCAL_STATE_EKF_PROFILE_FILE="${NJRH_LOCAL_STATE_EKF_PROFILE_FILE:-${OVERLAY_ROOT}/config/local_state_ekf_profile.env}"
if [[ -f "${LOCAL_STATE_EKF_PROFILE_FILE}" ]]; then
  # shellcheck source=../config/local_state_ekf_profile.env
  source "${LOCAL_STATE_EKF_PROFILE_FILE}"
fi

ISAAC_LOCALIZATION_MODE_FILE="${NJRH_ISAAC_LOCALIZATION_MODE_FILE:-${OVERLAY_ROOT}/config/isaac_localization_mode.env}"
if [[ -f "${ISAAC_LOCALIZATION_MODE_FILE}" ]]; then
  # shellcheck source=../config/isaac_localization_mode.env
  source "${ISAAC_LOCALIZATION_MODE_FILE}"
fi

AMCL_LOCALIZATION_PROFILE_FILE="${NJRH_AMCL_LOCALIZATION_PROFILE_FILE:-${OVERLAY_ROOT}/config/amcl_localization_profile.env}"
if [[ -f "${AMCL_LOCALIZATION_PROFILE_FILE}" ]]; then
  # shellcheck source=../config/amcl_localization_profile.env
  source "${AMCL_LOCALIZATION_PROFILE_FILE}"
fi

project_overlay_missing() {
  case ":${AMENT_PREFIX_PATH:-}:" in
    *":${PROJECT_ROOT}/install:"*) return 1 ;;
    *) return 0 ;;
  esac
}

if [[ "${NJRH_COMMON_ENV_SETUP_DONE:-}" != "1" ]] || project_overlay_missing; then
  set +u
  source /opt/ros/humble/setup.bash
  if [[ -f "${UPSTREAM_WS}/install/local_setup.bash" ]]; then
    source "${UPSTREAM_WS}/install/local_setup.bash"
  fi
  if [[ -f "${UPSTREAM_ROOT}/install/local_setup.bash" ]]; then
    source "${UPSTREAM_ROOT}/install/local_setup.bash"
  fi
  if [[ -f "${NJRH_FASTLIO_PATCHED_OVERLAY}/local_setup.bash" ]]; then
    source "${NJRH_FASTLIO_PATCHED_OVERLAY}/local_setup.bash"
  fi
  if [[ -f "${NJRH_JT128_NAV_TOOLS_PATCHED_OVERLAY}/local_setup.bash" ]]; then
    source "${NJRH_JT128_NAV_TOOLS_PATCHED_OVERLAY}/local_setup.bash"
  fi
  if [[ -f "${PROJECT_ROOT}/install/local_setup.bash" ]]; then
    source "${PROJECT_ROOT}/install/local_setup.bash"
  fi
  set -u
  export NJRH_COMMON_ENV_SETUP_DONE=1
fi

mkdir -p \
  "${NJRH_RUNTIME_LOG_DIR}" \
  "${NJRH_MAPS_DIR}" \
  "${NJRH_MAPS3D_DIR}" \
  "${NJRH_RELEASE_ASSETS_DIR}" \
  "${NJRH_WAYPOINTS_DIR}"

require_upstream_script() {
  local script_name="$1"
  local script_path="${UPSTREAM_SCRIPTS}/${script_name}"
  [[ -f "$script_path" ]] || {
    echo "[runtime-overlay] missing upstream script: ${script_path}" >&2
    exit 1
  }
  printf '%s\n' "$script_path"
}

runtime_readiness_probe_bin() {
  local candidate="${NJRH_RUNTIME_READINESS_PROBE_BIN:-${NJRH_PROJECT_ROOT}/install/robot_bringup/lib/robot_bringup/runtime_readiness_probe}"
  if [[ ! -x "${candidate}" ]]; then
    echo "[runtime-overlay] missing C++ readiness probe: ${candidate}" >&2
    echo "[runtime-overlay] build it with: colcon build --packages-select robot_bringup" >&2
    return 127
  fi
  printf '%s\n' "${candidate}"
}

runtime_readiness_probe() {
  local probe
  probe="$(runtime_readiness_probe_bin)" || return $?
  "${probe}" "$@"
}
