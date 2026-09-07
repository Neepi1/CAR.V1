#!/usr/bin/env bash

# A mapping session moves two full-size PointCloud2 streams between local
# processes.  Keep the robot-wide Fast DDS profile UDP-only, but let the three
# participants on those two hops negotiate SHM with UDP as a fallback.

NJRH_SLAM2D_FASTDDS_TRANSPORT="${NJRH_SLAM2D_FASTDDS_TRANSPORT:-scoped_shm}"
NJRH_SLAM2D_FASTDDS_SEGMENT_SIZE_BYTES="${NJRH_SLAM2D_FASTDDS_SEGMENT_SIZE_BYTES:-134217728}"
NJRH_SLAM2D_FASTDDS_PORT_QUEUE_CAPACITY="${NJRH_SLAM2D_FASTDDS_PORT_QUEUE_CAPACITY:-512}"
NJRH_SLAM2D_FASTDDS_HEALTHY_CHECK_TIMEOUT_MS="${NJRH_SLAM2D_FASTDDS_HEALTHY_CHECK_TIMEOUT_MS:-1000}"

mapping_fastdds_profile_owned="false"
mapping_fastdds_active_participant_pids=()

mapping_fastdds_process_uses_profile() {
  local pid="$1"
  local profile="$2"
  local entry=""
  [[ "${pid}" =~ ^[0-9]+$ && -r "/proc/${pid}/environ" ]] || return 1
  while IFS= read -r -d '' entry; do
    [[ "${entry}" == "FASTRTPS_DEFAULT_PROFILES_FILE=${profile}" ||
       "${entry}" == "FASTDDS_DEFAULT_PROFILES_FILE=${profile}" ]] && return 0
  done <"/proc/${pid}/environ"
  return 1
}

mapping_fastdds_stop_profile_participants() {
  local profile="$1"
  local pid=""
  local signal=""
  local wait_steps=0
  local pids=()

  mapfile -t pids < <(pgrep -f 'fastlio_mapping|nav_cloud_preprocessor|pointcloud_to_laserscan' 2>/dev/null || true)
  mapping_fastdds_active_participant_pids=()
  for pid in "${pids[@]}"; do
    [[ "${pid}" != "$$" && "${pid}" != "${PPID}" ]] || continue
    mapping_fastdds_process_uses_profile "${pid}" "${profile}" || continue
    mapping_fastdds_active_participant_pids+=("${pid}")
  done
  [[ ${#mapping_fastdds_active_participant_pids[@]} -gt 0 ]] || return 0

  echo "[runtime-overlay] stopping ${#mapping_fastdds_active_participant_pids[@]} mapping SHM participant process(es) before profile cleanup" >&2
  for signal in INT TERM KILL; do
    for pid in "${mapping_fastdds_active_participant_pids[@]}"; do
      kill -s "${signal}" "${pid}" 2>/dev/null || true
    done
    wait_steps=0
    while (( wait_steps < 20 )); do
      local alive=0
      for pid in "${mapping_fastdds_active_participant_pids[@]}"; do
        kill -0 "${pid}" 2>/dev/null && alive=1 || true
      done
      [[ "${alive}" -eq 1 ]] || return 0
      sleep 0.1
      wait_steps=$((wait_steps + 1))
    done
  done
}

mapping_fastdds_collect_addresses() {
  local base_profile="$1"
  local candidates="${NJRH_FASTDDS_ALLOWED_ADDRESSES:-}"
  local address=""
  local seen=" "

  if [[ -z "${candidates}" && -r "${base_profile}" ]]; then
    candidates="$(sed -n 's:.*<address>\([^<]*\)</address>.*:\1:p' "${base_profile}" | paste -sd ' ' -)"
  fi
  if [[ -z "${candidates}" ]]; then
    candidates="127.0.0.1"
    address="$(ip -o -4 route get 1.1.1.1 2>/dev/null | sed -n 's/.* src \([^ ]*\).*/\1/p' | head -n 1)"
    [[ -z "${address}" ]] || candidates+=" ${address}"
  fi

  for address in ${candidates//,/ }; do
    [[ "${address}" =~ ^[0-9]+([.][0-9]+){3}$ ]] || continue
    case "${seen}" in
      *" ${address} "*) continue ;;
    esac
    printf '%s\n' "${address}"
    seen+="${address} "
  done
}

configure_mapping_fastdds_transport() {
  local base_profile="${FASTRTPS_DEFAULT_PROFILES_FILE:-${FASTDDS_DEFAULT_PROFILES_FILE:-}}"
  local active_profile=""
  local addresses=()
  local address=""

  export NJRH_SLAM2D_FASTDDS_BASE_PROFILE_FILE="${base_profile}"

  # Explicit rollback: NJRH_SLAM2D_FASTDDS_TRANSPORT=inherit
  if [[ "${NJRH_SLAM2D_FASTDDS_TRANSPORT}" == "inherit" ]]; then
    export NJRH_SLAM2D_FASTDDS_ACTIVE_PROFILE_FILE="${base_profile}"
    echo "[runtime-overlay] mapping Fast DDS transport inherits the normal UDP profile" >&2
    return 0
  fi
  if [[ "${NJRH_SLAM2D_FASTDDS_TRANSPORT}" != "scoped_shm" ]]; then
    echo "[runtime-overlay] invalid NJRH_SLAM2D_FASTDDS_TRANSPORT=${NJRH_SLAM2D_FASTDDS_TRANSPORT}; expected scoped_shm or inherit" >&2
    return 1
  fi
  [[ "${NJRH_SLAM2D_FASTDDS_SEGMENT_SIZE_BYTES}" =~ ^[0-9]+$ ]] || {
    echo "[runtime-overlay] invalid mapping SHM segment size: ${NJRH_SLAM2D_FASTDDS_SEGMENT_SIZE_BYTES}" >&2
    return 1
  }
  if (( NJRH_SLAM2D_FASTDDS_SEGMENT_SIZE_BYTES < 67108864 )); then
    echo "[runtime-overlay] mapping SHM segment must be at least 67108864 bytes" >&2
    return 1
  fi

  mapfile -t addresses < <(mapping_fastdds_collect_addresses "${base_profile}")
  [[ ${#addresses[@]} -gt 0 ]] || {
    echo "[runtime-overlay] cannot derive an IPv4 address for the mapping Fast DDS profile" >&2
    return 1
  }

  active_profile="$(mktemp --suffix=.xml "/tmp/njrh_slam2d_fastdds_profile_${BASHPID:-$$}_XXXXXX")"
  mapping_fastdds_profile_owned="true"
  export NJRH_SLAM2D_FASTDDS_ACTIVE_PROFILE_FILE="${active_profile}"
  {
    cat <<'XML'
<?xml version="1.0" encoding="UTF-8"?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
  <transport_descriptors>
    <transport_descriptor>
      <transport_id>njrh_mapping_udp_transport</transport_id>
      <type>UDPv4</type>
      <interfaceWhiteList>
XML
    for address in "${addresses[@]}"; do
      printf '        <address>%s</address>\n' "${address}"
    done
    cat <<XML
      </interfaceWhiteList>
    </transport_descriptor>
    <transport_descriptor>
      <transport_id>njrh_mapping_shm_transport</transport_id>
      <type>SHM</type>
      <segment_size>${NJRH_SLAM2D_FASTDDS_SEGMENT_SIZE_BYTES}</segment_size>
      <port_queue_capacity>${NJRH_SLAM2D_FASTDDS_PORT_QUEUE_CAPACITY}</port_queue_capacity>
      <healthy_check_timeout_ms>${NJRH_SLAM2D_FASTDDS_HEALTHY_CHECK_TIMEOUT_MS}</healthy_check_timeout_ms>
    </transport_descriptor>
  </transport_descriptors>
  <participant profile_name="njrh_mapping_participant" is_default_profile="true">
    <rtps>
      <userTransports>
        <transport_id>njrh_mapping_udp_transport</transport_id>
        <transport_id>njrh_mapping_shm_transport</transport_id>
      </userTransports>
      <useBuiltinTransports>false</useBuiltinTransports>
    </rtps>
  </participant>
</profiles>
XML
  } >"${active_profile}"
  chmod 0644 "${active_profile}"

  echo "[runtime-overlay] mapping large-cloud Fast DDS profile=${active_profile} transport=UDPv4+SHM segment_bytes=${NJRH_SLAM2D_FASTDDS_SEGMENT_SIZE_BYTES}" >&2
}

cleanup_mapping_fastdds_transport() {
  if [[ "${mapping_fastdds_profile_owned}" == "true" &&
        -n "${NJRH_SLAM2D_FASTDDS_ACTIVE_PROFILE_FILE:-}" &&
        "${NJRH_SLAM2D_FASTDDS_ACTIVE_PROFILE_FILE}" == /tmp/njrh_slam2d_fastdds_profile_*.xml ]]; then
    mapping_fastdds_stop_profile_participants "${NJRH_SLAM2D_FASTDDS_ACTIVE_PROFILE_FILE}"
    rm -f -- "${NJRH_SLAM2D_FASTDDS_ACTIVE_PROFILE_FILE}" 2>/dev/null || true
  fi
  mapping_fastdds_profile_owned="false"
}
