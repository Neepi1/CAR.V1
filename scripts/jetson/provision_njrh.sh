#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROVISIONER="${SCRIPT_DIR}/provision/njrh_provision.py"
ACTION="${1:-}"
DEFAULT_MANIFEST="${NJRH_DEVICE_RELEASE_MANIFEST:-/opt/njrh/provisioning/device-release.json}"

usage() {
  echo "usage: $0 {enroll-trust|deploy|apply|status|verify|verify-bundle|accept-hardware|activate|preflight|validate} [release URL|device-release.json] [extra args]" >&2
}

[[ -f "${PROVISIONER}" ]] || {
  echo "[njrh-provision] missing provisioner: ${PROVISIONER}" >&2
  exit 1
}

case "${ACTION}" in
  enroll-trust)
    shift
    PUBLIC_KEY="${1:-}"
    EXPECTED_SHA256="${2:-}"
    [[ -f "${PUBLIC_KEY}" && ! -L "${PUBLIC_KEY}" ]] || {
      echo "[njrh-provision] public key must be a regular non-symlink file" >&2
      exit 2
    }
    [[ "${EXPECTED_SHA256}" =~ ^[0-9a-f]{64}$ ]] || {
      echo "[njrh-provision] expected public-key SHA-256 is required" >&2
      exit 2
    }
    if [[ "${EUID}" -ne 0 ]]; then
      exec sudo bash "$0" enroll-trust "${PUBLIC_KEY}" "${EXPECTED_SHA256}"
    fi
    command -v openssl >/dev/null 2>&1 || {
      echo "[njrh-provision] openssl is required" >&2
      exit 10
    }
    ACTUAL_SHA256="$(sha256sum "${PUBLIC_KEY}" | awk '{print $1}')"
    [[ "${ACTUAL_SHA256}" == "${EXPECTED_SHA256}" ]] || {
      echo "[njrh-provision] public-key SHA-256 mismatch" >&2
      exit 12
    }
    openssl pkey -pubin -in "${PUBLIC_KEY}" -text -noout 2>&1 \
      | grep -qi 'ED25519' || {
        echo "[njrh-provision] trust key is not an Ed25519 public key" >&2
        exit 12
      }
    install -d -o root -g root -m 0755 /etc/njrh/trust
    if [[ -e /etc/njrh/trust/release-ed25519-public.pem ]]; then
      EXISTING_SHA256="$(sha256sum /etc/njrh/trust/release-ed25519-public.pem | awk '{print $1}')"
      [[ "${EXISTING_SHA256}" == "${EXPECTED_SHA256}" ]] || {
        echo "[njrh-provision] a different trust key is already enrolled; refusing implicit rotation" >&2
        exit 12
      }
      echo "[njrh-provision] factory trust key is already enrolled"
      exit 0
    fi
    TRUST_TEMP="$(mktemp /etc/njrh/trust/.release-key.XXXXXXXX)"
    trap 'rm -f -- "${TRUST_TEMP}"' EXIT
    install -o root -g root -m 0644 "${PUBLIC_KEY}" "${TRUST_TEMP}"
    mv -f -- "${TRUST_TEMP}" /etc/njrh/trust/release-ed25519-public.pem
    echo "[njrh-provision] enrolled factory trust key sha256=${ACTUAL_SHA256}"
    ;;
  deploy)
    shift
    RELEASE_BASE_URL="${1:-}"
    [[ "${RELEASE_BASE_URL}" == https://* ]] || {
      echo "[njrh-provision] deploy requires an HTTPS GitHub Release base URL" >&2
      exit 2
    }
    if [[ "${EUID}" -ne 0 ]]; then
      exec sudo bash "$0" deploy "${RELEASE_BASE_URL}"
    fi
    command -v curl >/dev/null 2>&1 || {
      echo "[njrh-provision] curl is required" >&2
      exit 10
    }
    INCOMING_ROOT="/opt/njrh/provisioning"
    mkdir -p "${INCOMING_ROOT}/releases"
    DOWNLOAD_DIR="$(mktemp -d "${INCOMING_ROOT}/.download.XXXXXXXX")"
    trap 'rm -rf -- "${DOWNLOAD_DIR}"' EXIT
    for release_file in device-release.json release-lock.json device-release.json.sig; do
      curl --fail --show-error --silent --location \
        --proto '=https' --proto-redir '=https' --tlsv1.2 \
        --connect-timeout 15 --max-time 120 --max-filesize 1048576 \
        "${RELEASE_BASE_URL%/}/${release_file}" \
        --output "${DOWNLOAD_DIR}/${release_file}"
      [[ -s "${DOWNLOAD_DIR}/${release_file}" ]] || {
        echo "[njrh-provision] empty release file: ${release_file}" >&2
        exit 12
      }
    done
    python3 "${PROVISIONER}" verify-bundle \
      "${DOWNLOAD_DIR}/device-release.json" >/dev/null
    RELEASE_ID="$(python3 - "${DOWNLOAD_DIR}/device-release.json" <<'PY'
import json
import re
import sys
value = json.load(open(sys.argv[1], encoding="utf-8")).get("release_id", "")
if not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9._-]{0,95}", value):
    raise SystemExit("unsafe release_id")
print(value)
PY
)"
    RELEASE_DIR="${INCOMING_ROOT}/releases/${RELEASE_ID}"
    if [[ -e "${RELEASE_DIR}" ]]; then
      for release_file in device-release.json release-lock.json device-release.json.sig; do
        cmp -s "${DOWNLOAD_DIR}/${release_file}" "${RELEASE_DIR}/${release_file}" || {
          echo "[njrh-provision] immutable release metadata conflict: ${RELEASE_DIR}" >&2
          exit 12
        }
      done
    else
      mkdir "${RELEASE_DIR}"
      install -o root -g root -m 0644 \
        "${DOWNLOAD_DIR}/device-release.json" \
        "${DOWNLOAD_DIR}/release-lock.json" \
        "${DOWNLOAD_DIR}/device-release.json.sig" \
        "${RELEASE_DIR}/"
    fi
    if [[ ! -e /etc/njrh/secrets.env ]] \
      && ! grep -q '^ROBOT_API_TOKEN=.' /etc/njrh/runtime.env 2>/dev/null; then
      install -d -o root -g root -m 0755 /etc/njrh
      GENERATED_API_TOKEN="$(openssl rand -hex 32)"
      SECRET_TEMP="$(mktemp /etc/njrh/.secrets.XXXXXXXX)"
      trap 'rm -rf -- "${DOWNLOAD_DIR}"; rm -f -- "${SECRET_TEMP:-}"' EXIT
      printf 'ROBOT_API_TOKEN=%s\n' "${GENERATED_API_TOKEN}" >"${SECRET_TEMP}"
      chown root:root "${SECRET_TEMP}"
      chmod 0600 "${SECRET_TEMP}"
      mv -- "${SECRET_TEMP}" /etc/njrh/secrets.env
      echo "[njrh-provision] GENERATED ROBOT_API_TOKEN=${GENERATED_API_TOKEN}" >&2
      echo "[njrh-provision] Capture this token once in the factory credential vault/App pairing record." >&2
      unset GENERATED_API_TOKEN
    fi
    rm -rf -- "${DOWNLOAD_DIR}"
    trap - EXIT
    exec python3 "${PROVISIONER}" apply "${RELEASE_DIR}/device-release.json"
    ;;
  status)
    shift
    if [[ "${EUID}" -ne 0 ]]; then
      exec sudo bash "$0" status "$@"
    fi
    exec python3 "${PROVISIONER}" status "$@"
    ;;
  apply|activate|accept-hardware|verify|verify-bundle|preflight)
    shift
    MANIFEST="${1:-${DEFAULT_MANIFEST}}"
    if [[ $# -gt 0 ]]; then
      shift
    fi
    if [[ "${EUID}" -ne 0 ]]; then
      exec sudo --preserve-env=NJRH_DEVICE_RELEASE_MANIFEST \
        bash "$0" "${ACTION}" "${MANIFEST}" "$@"
    fi
    exec python3 "${PROVISIONER}" "${ACTION}" "${MANIFEST}" "$@"
    ;;
  validate)
    shift
    MANIFEST="${1:-${DEFAULT_MANIFEST}}"
    if [[ $# -gt 0 ]]; then
      shift
    fi
    exec python3 "${PROVISIONER}" "${ACTION}" "${MANIFEST}" "$@"
    ;;
  *)
    usage
    exit 2
    ;;
esac
