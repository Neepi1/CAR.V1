#!/usr/bin/env bash
set -euo pipefail

ACTION="${1:-install}"

SERVICE_NAME="${NJRH_AUTOSTART_SERVICE_NAME:-njrh-runtime.service}"
CAN_SERVICE_NAME="${NJRH_CAN_SERVICE_NAME:-njrh-can.service}"
WORKSPACE_HOST="${NJRH_WORKSPACE_HOST:-/home/nvidia/workspaces/njrh-v3/workspace1}"
WORKSPACE_CONTAINER="${NJRH_WORKSPACE_CONTAINER:-/workspaces/njrh-v3/workspace1}"
UPSTREAM_WORKSPACE_HOST="${NJRH_UPSTREAM_WORKSPACE_HOST:-/home/nvidia/workspaces/isaac_ros-dev}"
UPSTREAM_WORKSPACE_CONTAINER="${NJRH_UPSTREAM_WORKSPACE_CONTAINER:-/workspaces/isaac_ros-dev}"
CONTAINER_NAME="${NJRH_CONTAINER_NAME:-NJRH-car}"
CAN_IFACE="${CAN_IFACE:-can0}"
CAN_BITRATE="${CAN_BITRATE:-500000}"
ENV_FILE="${NJRH_AUTOSTART_ENV_FILE:-/etc/njrh/runtime.env}"
SECRETS_ENV_FILE="${NJRH_SECRETS_ENV_FILE:-/etc/njrh/secrets.env}"
PROVISION_MOTION_LOCK="${NJRH_PROVISION_MOTION_LOCK:-/var/lib/njrh/provision/motion.lock}"
SKIP_RUNTIME_ENV_WRITE="${NJRH_SKIP_RUNTIME_ENV_WRITE:-false}"
HOST_SERVICE_USER="${NJRH_HOST_SERVICE_USER:-nvidia}"
UNIT_PATH="/etc/systemd/system/${SERVICE_NAME}"
CAN_UNIT_PATH="/etc/systemd/system/${CAN_SERVICE_NAME}"
RUNNER="${WORKSPACE_HOST}/scripts/jetson/njrh_systemd_runtime.sh"
CAN_RUNNER="${WORKSPACE_HOST}/scripts/jetson/bringup_ranger_can_wait.sh"

require_sudo() {
  if [[ "${EUID}" -eq 0 ]]; then
    return 0
  fi
  sudo -v
}

migrate_runtime_secret_if_present() {
  local token=""
  [[ -f "${ENV_FILE}" ]] || return 0
  token="$(sudo awk -F= '/^ROBOT_API_TOKEN=/{sub(/^[^=]*=/, ""); print; exit}' "${ENV_FILE}")"
  if [[ -n "${token}" && "${token}" != "change-me" ]]; then
    sudo mkdir -p "$(dirname "${SECRETS_ENV_FILE}")"
    if [[ ! -f "${SECRETS_ENV_FILE}" ]]; then
      printf 'ROBOT_API_TOKEN=%s\n' "${token}" | sudo tee "${SECRETS_ENV_FILE}" >/dev/null
    fi
  fi
  sudo sed -i '/^ROBOT_API_TOKEN=/d' "${ENV_FILE}"
  if [[ -f "${SECRETS_ENV_FILE}" ]]; then
    sudo chown root:root "${SECRETS_ENV_FILE}"
    sudo chmod 0600 "${SECRETS_ENV_FILE}"
  fi
}

upsert_env_value() {
  local key="$1"
  local value="$2"
  if sudo grep -q "^${key}=" "${ENV_FILE}"; then
    sudo sed -i "s|^${key}=.*|${key}=${value}|" "${ENV_FILE}"
  else
    echo "${key}=${value}" | sudo tee -a "${ENV_FILE}" >/dev/null
  fi
}

write_env_file_if_missing() {
  require_sudo
  sudo mkdir -p "$(dirname "${ENV_FILE}")"
  if [[ -f "${ENV_FILE}" ]]; then
    migrate_runtime_secret_if_present
    upsert_env_value "NJRH_DOCKING_SENSOR_BACKEND" "orbbec_336l"
    upsert_env_value "NJRH_GS2_AUTOSTART" "false"
    if sudo grep -q '^NJRH_AMCL_RESIDENT_WARMUP_BEFORE_INITIAL_LOCALIZATION=' "${ENV_FILE}"; then
      sudo sed -i 's/^NJRH_AMCL_RESIDENT_WARMUP_BEFORE_INITIAL_LOCALIZATION=.*/NJRH_AMCL_RESIDENT_WARMUP_BEFORE_INITIAL_LOCALIZATION=false/' "${ENV_FILE}"
    else
      echo 'NJRH_AMCL_RESIDENT_WARMUP_BEFORE_INITIAL_LOCALIZATION=false' | sudo tee -a "${ENV_FILE}" >/dev/null
    fi
    if sudo grep -q '^NJRH_INITIAL_GLOBAL_LOCALIZATION_BACKGROUND_START=' "${ENV_FILE}"; then
      sudo sed -i 's/^NJRH_INITIAL_GLOBAL_LOCALIZATION_BACKGROUND_START=.*/NJRH_INITIAL_GLOBAL_LOCALIZATION_BACKGROUND_START=false/' "${ENV_FILE}"
    else
      echo 'NJRH_INITIAL_GLOBAL_LOCALIZATION_BACKGROUND_START=false' | sudo tee -a "${ENV_FILE}" >/dev/null
    fi
    if sudo grep -q '^NJRH_NAV2_PRESTART_BEFORE_INITIAL_LOCALIZATION=' "${ENV_FILE}"; then
      sudo sed -i 's/^NJRH_NAV2_PRESTART_BEFORE_INITIAL_LOCALIZATION=.*/NJRH_NAV2_PRESTART_BEFORE_INITIAL_LOCALIZATION=false/' "${ENV_FILE}"
    else
      echo 'NJRH_NAV2_PRESTART_BEFORE_INITIAL_LOCALIZATION=false' | sudo tee -a "${ENV_FILE}" >/dev/null
    fi
    if sudo grep -q '^NJRH_NAV2_LIFECYCLE_PARALLEL_CORE=' "${ENV_FILE}"; then
      sudo sed -i 's/^NJRH_NAV2_LIFECYCLE_PARALLEL_CORE=.*/NJRH_NAV2_LIFECYCLE_PARALLEL_CORE=false/' "${ENV_FILE}"
    else
      echo 'NJRH_NAV2_LIFECYCLE_PARALLEL_CORE=false' | sudo tee -a "${ENV_FILE}" >/dev/null
    fi
    if sudo grep -q '^NJRH_NAV2_LIFECYCLE_PARALLEL_BT=' "${ENV_FILE}"; then
      sudo sed -i 's/^NJRH_NAV2_LIFECYCLE_PARALLEL_BT=.*/NJRH_NAV2_LIFECYCLE_PARALLEL_BT=true/' "${ENV_FILE}"
    else
      echo 'NJRH_NAV2_LIFECYCLE_PARALLEL_BT=true' | sudo tee -a "${ENV_FILE}" >/dev/null
    fi
    if sudo grep -q '^NJRH_NAV2_LIFECYCLE_BACKGROUND_AFTER_LOCALIZATION_STACK=' "${ENV_FILE}"; then
      sudo sed -i 's/^NJRH_NAV2_LIFECYCLE_BACKGROUND_AFTER_LOCALIZATION_STACK=.*/NJRH_NAV2_LIFECYCLE_BACKGROUND_AFTER_LOCALIZATION_STACK=false/' "${ENV_FILE}"
    else
      echo 'NJRH_NAV2_LIFECYCLE_BACKGROUND_AFTER_LOCALIZATION_STACK=false' | sudo tee -a "${ENV_FILE}" >/dev/null
    fi
    if sudo grep -q '^NJRH_PREPARE_RUNTIME_PERMISSIONS_MODE=' "${ENV_FILE}"; then
      sudo sed -i 's/^NJRH_PREPARE_RUNTIME_PERMISSIONS_MODE=.*/NJRH_PREPARE_RUNTIME_PERMISSIONS_MODE=once/' "${ENV_FILE}"
    else
      echo 'NJRH_PREPARE_RUNTIME_PERMISSIONS_MODE=once' | sudo tee -a "${ENV_FILE}" >/dev/null
    fi
    if sudo grep -q '^NJRH_COMMON_LOCAL_STATE_START_READY_MODE=' "${ENV_FILE}"; then
      sudo sed -i 's/^NJRH_COMMON_LOCAL_STATE_START_READY_MODE=.*/NJRH_COMMON_LOCAL_STATE_START_READY_MODE=endpoint/' "${ENV_FILE}"
    else
      echo 'NJRH_COMMON_LOCAL_STATE_START_READY_MODE=endpoint' | sudo tee -a "${ENV_FILE}" >/dev/null
    fi
    if sudo grep -q '^NJRH_COMMON_LOCAL_STATE_BACKGROUND_START=' "${ENV_FILE}"; then
      sudo sed -i 's/^NJRH_COMMON_LOCAL_STATE_BACKGROUND_START=.*/NJRH_COMMON_LOCAL_STATE_BACKGROUND_START=true/' "${ENV_FILE}"
    else
      echo 'NJRH_COMMON_LOCAL_STATE_BACKGROUND_START=true' | sudo tee -a "${ENV_FILE}" >/dev/null
    fi
    if sudo grep -q '^NJRH_RESIDENT_NAVIGATION_PRESTART_BEFORE_LOCAL_STATE=' "${ENV_FILE}"; then
      sudo sed -i 's/^NJRH_RESIDENT_NAVIGATION_PRESTART_BEFORE_LOCAL_STATE=.*/NJRH_RESIDENT_NAVIGATION_PRESTART_BEFORE_LOCAL_STATE=false/' "${ENV_FILE}"
    else
      echo 'NJRH_RESIDENT_NAVIGATION_PRESTART_BEFORE_LOCAL_STATE=false' | sudo tee -a "${ENV_FILE}" >/dev/null
    fi
    if sudo grep -q '^NJRH_NAV2_HELD_PRESTART_WAIT_FOR_LOCALIZER_SERVICE=' "${ENV_FILE}"; then
      sudo sed -i 's/^NJRH_NAV2_HELD_PRESTART_WAIT_FOR_LOCALIZER_SERVICE=.*/NJRH_NAV2_HELD_PRESTART_WAIT_FOR_LOCALIZER_SERVICE=true/' "${ENV_FILE}"
    else
      echo 'NJRH_NAV2_HELD_PRESTART_WAIT_FOR_LOCALIZER_SERVICE=true' | sudo tee -a "${ENV_FILE}" >/dev/null
    fi
    echo "[njrh-autostart] env file already exists: ${ENV_FILE}"
    sudo chown root:root "${ENV_FILE}"
    sudo chmod 0640 "${ENV_FILE}"
    return 0
  fi
  sudo tee "${ENV_FILE}" >/dev/null <<EOF
NJRH_WORKSPACE_HOST=${WORKSPACE_HOST}
NJRH_WORKSPACE_CONTAINER=${WORKSPACE_CONTAINER}
NJRH_UPSTREAM_WORKSPACE_HOST=${UPSTREAM_WORKSPACE_HOST}
NJRH_UPSTREAM_WORKSPACE_CONTAINER=${UPSTREAM_WORKSPACE_CONTAINER}
NJRH_CONTAINER_NAME=${CONTAINER_NAME}
NJRH_REUSE_COMMON_SERVICES=true
NJRH_DOCKING_SENSOR_BACKEND=orbbec_336l
NJRH_GS2_AUTOSTART=false
NJRH_AMCL_RESIDENT_WARMUP_BEFORE_INITIAL_LOCALIZATION=false
NJRH_NAV2_PRESTART_BEFORE_INITIAL_LOCALIZATION=false
NJRH_INITIAL_GLOBAL_LOCALIZATION_BACKGROUND_START=false
NJRH_NAV2_LIFECYCLE_PARALLEL_CORE=false
NJRH_NAV2_LIFECYCLE_PARALLEL_BT=true
NJRH_NAV2_LIFECYCLE_BACKGROUND_AFTER_LOCALIZATION_STACK=false
NJRH_PREPARE_RUNTIME_PERMISSIONS_MODE=once
NJRH_COMMON_LOCAL_STATE_START_READY_MODE=endpoint
NJRH_COMMON_LOCAL_STATE_BACKGROUND_START=true
NJRH_RESIDENT_NAVIGATION_PRESTART_BEFORE_LOCAL_STATE=false
NJRH_NAV2_HELD_PRESTART_WAIT_FOR_LOCALIZER_SERVICE=true
RMW_IMPLEMENTATION=rmw_fastrtps_cpp
FASTDDS_BUILTIN_TRANSPORTS=UDPv4
EOF
  sudo chown root:root "${ENV_FILE}"
  sudo chmod 0640 "${ENV_FILE}"
  echo "[njrh-autostart] created env file: ${ENV_FILE}"
}

install_unit() {
  local enable_units="${1:-true}"
  require_sudo
  [[ -f "${RUNNER}" ]] || {
    echo "[njrh-autostart] missing runner: ${RUNNER}" >&2
    exit 1
  }
  [[ -f "${CAN_RUNNER}" ]] || {
    echo "[njrh-autostart] missing CAN runner: ${CAN_RUNNER}" >&2
    exit 1
  }
  id "${HOST_SERVICE_USER}" >/dev/null 2>&1 || {
    echo "[njrh-autostart] host service user does not exist: ${HOST_SERVICE_USER}" >&2
    exit 1
  }
  if [[ "${SKIP_RUNTIME_ENV_WRITE}" != "true" ]]; then
    write_env_file_if_missing
  fi
  sudo tee "${CAN_UNIT_PATH}" >/dev/null <<EOF
[Unit]
Description=NJRH Ranger CAN bringup
Wants=sys-subsystem-net-devices-can0.device
After=sys-subsystem-net-devices-can0.device
Before=${SERVICE_NAME}

[Service]
Type=oneshot
RemainAfterExit=yes
EnvironmentFile=-${ENV_FILE}
Environment=CAN_IFACE=${CAN_IFACE}
Environment=CAN_BITRATE=${CAN_BITRATE}
Environment=CAN_WAIT_TIMEOUT_SEC=120
ExecStartPre=/usr/bin/test ! -e ${PROVISION_MOTION_LOCK}
ExecStart=/usr/bin/env bash ${CAN_RUNNER}

[Install]
WantedBy=multi-user.target
EOF
  sudo tee "${UNIT_PATH}" >/dev/null <<EOF
[Unit]
Description=NJRH car container and common ROS services
Wants=network-online.target docker.service ${CAN_SERVICE_NAME}
After=network-online.target docker.service ${CAN_SERVICE_NAME}
Requires=docker.service ${CAN_SERVICE_NAME}

[Service]
Type=simple
User=${HOST_SERVICE_USER}
WorkingDirectory=${WORKSPACE_HOST}
EnvironmentFile=-${ENV_FILE}
EnvironmentFile=-${SECRETS_ENV_FILE}
ExecStartPre=/usr/bin/test ! -e ${PROVISION_MOTION_LOCK}
ExecStart=/usr/bin/env bash ${RUNNER} run
ExecStop=/usr/bin/env bash ${RUNNER} stop
Restart=always
RestartSec=10
TimeoutStartSec=240
TimeoutStopSec=30
KillSignal=SIGINT

[Install]
WantedBy=multi-user.target
EOF
  sudo systemctl daemon-reload
  if [[ "${enable_units}" == "true" ]]; then
    sudo systemctl enable "${CAN_SERVICE_NAME}"
    sudo systemctl enable "${SERVICE_NAME}"
    echo "[njrh-autostart] enabled ${CAN_SERVICE_NAME} and ${SERVICE_NAME}"
  else
    sudo systemctl disable --now "${SERVICE_NAME}" >/dev/null 2>&1 || true
    sudo systemctl disable --now "${CAN_SERVICE_NAME}" >/dev/null 2>&1 || true
    echo "[njrh-autostart] installed ${CAN_SERVICE_NAME} and ${SERVICE_NAME} disabled"
  fi
}

case "${ACTION}" in
  install)
    install_unit true
    systemctl is-enabled "${SERVICE_NAME}" || true
    ;;
  install-disabled)
    install_unit false
    systemctl is-enabled "${CAN_SERVICE_NAME}" || true
    systemctl is-enabled "${SERVICE_NAME}" || true
    ;;
  install-start)
    install_unit true
    sudo systemctl restart "${CAN_SERVICE_NAME}"
    sudo systemctl restart "${SERVICE_NAME}"
    sudo systemctl --no-pager --full status "${CAN_SERVICE_NAME}" || true
    sudo systemctl --no-pager --full status "${SERVICE_NAME}" || true
    ;;
  start)
    require_sudo
    sudo systemctl start "${SERVICE_NAME}"
    ;;
  stop)
    require_sudo
    sudo systemctl stop "${SERVICE_NAME}"
    ;;
  restart)
    require_sudo
    sudo systemctl restart "${SERVICE_NAME}"
    ;;
  status)
    systemctl --no-pager --full status "${SERVICE_NAME}" || true
    ;;
  disable)
    require_sudo
    sudo systemctl disable --now "${SERVICE_NAME}" || true
    sudo systemctl disable --now "${CAN_SERVICE_NAME}" || true
    sudo rm -f "${UNIT_PATH}"
    sudo rm -f "${CAN_UNIT_PATH}"
    sudo systemctl daemon-reload
    echo "[njrh-autostart] disabled ${CAN_SERVICE_NAME} and ${SERVICE_NAME}"
    ;;
  *)
    echo "usage: $0 {install|install-disabled|install-start|start|stop|restart|status|disable}" >&2
    exit 1
    ;;
esac
