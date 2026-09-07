#!/usr/bin/env bash

# Stable transport contract for a resident publisher of full-size PointCloud2
# messages.  UDP remains available for remote/legacy participants while SHM is
# preferred automatically by Fast DDS for subscribers in the same container.

NJRH_LARGE_CLOUD_FASTDDS_SEGMENT_SIZE_BYTES="${NJRH_LARGE_CLOUD_FASTDDS_SEGMENT_SIZE_BYTES:-134217728}"
NJRH_LARGE_CLOUD_FASTDDS_PORT_QUEUE_CAPACITY="${NJRH_LARGE_CLOUD_FASTDDS_PORT_QUEUE_CAPACITY:-512}"
NJRH_LARGE_CLOUD_FASTDDS_HEALTHY_CHECK_TIMEOUT_MS="${NJRH_LARGE_CLOUD_FASTDDS_HEALTHY_CHECK_TIMEOUT_MS:-1000}"

large_cloud_fastdds_collect_addresses() {
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

configure_large_cloud_fastdds_transport() {
  local base_profile="${FASTRTPS_DEFAULT_PROFILES_FILE:-${FASTDDS_DEFAULT_PROFILES_FILE:-}}"
  local profile_file="${NJRH_LARGE_CLOUD_FASTDDS_PROFILE_FILE:-/tmp/njrh_large_cloud_fastdds_profile.xml}"
  local temporary_profile=""
  local addresses=()
  local address=""

  [[ "${NJRH_LARGE_CLOUD_FASTDDS_SEGMENT_SIZE_BYTES}" =~ ^[0-9]+$ ]] || {
    echo "[runtime-overlay] invalid large-cloud SHM segment size: ${NJRH_LARGE_CLOUD_FASTDDS_SEGMENT_SIZE_BYTES}" >&2
    return 1
  }
  if (( NJRH_LARGE_CLOUD_FASTDDS_SEGMENT_SIZE_BYTES < 67108864 )); then
    echo "[runtime-overlay] large-cloud SHM segment must be at least 67108864 bytes" >&2
    return 1
  fi

  if [[ -e "${profile_file}" && ! -w "${profile_file}" ]]; then
    profile_file="/tmp/njrh_large_cloud_fastdds_profile_$(id -u).xml"
  fi
  mkdir -p "$(dirname "${profile_file}")"
  mapfile -t addresses < <(large_cloud_fastdds_collect_addresses "${base_profile}")
  [[ ${#addresses[@]} -gt 0 ]] || {
    echo "[runtime-overlay] cannot derive an IPv4 address for the large-cloud Fast DDS profile" >&2
    return 1
  }

  temporary_profile="$(mktemp "${profile_file}.tmp.XXXXXX")"
  {
    cat <<'XML'
<?xml version="1.0" encoding="UTF-8"?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
  <transport_descriptors>
    <transport_descriptor>
      <transport_id>njrh_large_cloud_udp_transport</transport_id>
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
      <transport_id>njrh_large_cloud_shm_transport</transport_id>
      <type>SHM</type>
      <segment_size>${NJRH_LARGE_CLOUD_FASTDDS_SEGMENT_SIZE_BYTES}</segment_size>
      <port_queue_capacity>${NJRH_LARGE_CLOUD_FASTDDS_PORT_QUEUE_CAPACITY}</port_queue_capacity>
      <healthy_check_timeout_ms>${NJRH_LARGE_CLOUD_FASTDDS_HEALTHY_CHECK_TIMEOUT_MS}</healthy_check_timeout_ms>
    </transport_descriptor>
  </transport_descriptors>
  <participant profile_name="njrh_large_cloud_participant" is_default_profile="true">
    <rtps>
      <userTransports>
        <transport_id>njrh_large_cloud_udp_transport</transport_id>
        <transport_id>njrh_large_cloud_shm_transport</transport_id>
      </userTransports>
      <useBuiltinTransports>false</useBuiltinTransports>
    </rtps>
  </participant>
</profiles>
XML
  } >"${temporary_profile}"
  chmod 0644 "${temporary_profile}"
  mv -f -- "${temporary_profile}" "${profile_file}"

  export NJRH_LARGE_CLOUD_FASTDDS_PROFILE_FILE="${profile_file}"
  echo "[runtime-overlay] resident large-cloud Fast DDS profile=${profile_file} transport=UDPv4+SHM segment_bytes=${NJRH_LARGE_CLOUD_FASTDDS_SEGMENT_SIZE_BYTES}" >&2
}
