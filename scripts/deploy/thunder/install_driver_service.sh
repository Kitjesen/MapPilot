#!/usr/bin/env bash
# Install the native Thunder driver service.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONFIG_DIR="${LINGTU_CONFIG_DIR:-/opt/lingtu/config}"
BRAINSTEM_ENV="${CONFIG_DIR}/brainstem.env"

requested_host="${LINGTU_BRAINSTEM_HOST:-}"
requested_port="${LINGTU_BRAINSTEM_PORT:-}"
requested_ca="${LINGTU_BRAINSTEM_TLS_CA_FILE:-}"
requested_cert="${LINGTU_BRAINSTEM_TLS_CERT_FILE:-}"
requested_key="${LINGTU_BRAINSTEM_TLS_KEY_FILE:-}"
requested_server_name="${LINGTU_BRAINSTEM_TLS_SERVER_NAME:-}"
configured_host=""
configured_port=""
configured_ca=""
configured_cert=""
configured_key=""
configured_server_name=""

read_env_value() {
  local key="$1"
  sed -n "s/^${key}=//p" "${BRAINSTEM_ENV}" | tail -n 1 | sed -e 's/^"//' -e 's/"$//'
}

valid_ipv4() {
  local value="$1" part
  local -a octets
  [[ "${value}" =~ ^([0-9]{1,3}\.){3}[0-9]{1,3}$ ]] || return 1
  IFS=. read -r -a octets <<< "${value}"
  for part in "${octets[@]}"; do
    ((10#${part} <= 255)) || return 1
  done
}

if [ -f "${BRAINSTEM_ENV}" ]; then
  configured_host="$(read_env_value LINGTU_BRAINSTEM_HOST)"
  configured_port="$(read_env_value LINGTU_BRAINSTEM_PORT)"
  configured_ca="$(read_env_value LINGTU_BRAINSTEM_TLS_CA_FILE)"
  configured_cert="$(read_env_value LINGTU_BRAINSTEM_TLS_CERT_FILE)"
  configured_key="$(read_env_value LINGTU_BRAINSTEM_TLS_KEY_FILE)"
  configured_server_name="$(read_env_value LINGTU_BRAINSTEM_TLS_SERVER_NAME)"
fi
remote_host="${requested_host:-${configured_host}}"
remote_port="${requested_port:-${configured_port:-13145}}"
tls_ca="${requested_ca:-${configured_ca}}"
tls_cert="${requested_cert:-${configured_cert}}"
tls_key="${requested_key:-${configured_key}}"
tls_server_name="${requested_server_name:-${configured_server_name}}"

if ! valid_ipv4 "${remote_host}" || [[ "${remote_host}" == 127.* ]] ||
    [[ "${remote_host}" == "0.0.0.0" ]]; then
  echo "ERROR: invalid LINGTU_BRAINSTEM_HOST: ${remote_host}" >&2
  echo "Use the remote Brainstem computer's literal IPv4 address." >&2
  exit 64
fi
if [[ ! "${remote_port}" =~ ^[0-9]+$ ]] ||
    [ "${remote_port}" -lt 1 ] || [ "${remote_port}" -gt 65535 ]; then
  echo "ERROR: LINGTU_BRAINSTEM_PORT must be within [1, 65535]." >&2
  exit 64
fi
for variable in tls_ca tls_cert tls_key; do
  path="${!variable}"
  if [[ ! "${path}" =~ ^/[A-Za-z0-9._/-]+$ ]] ||
      [[ ! -r "${path}" || ! -s "${path}" ]]; then
    echo "ERROR: ${variable} must be an absolute readable non-empty PEM file: ${path:-<unset>}" >&2
    exit 64
  fi
done
if [[ -n "${tls_server_name}" && ! "${tls_server_name}" =~ ^[A-Za-z0-9.-]+$ ]]; then
  echo "ERROR: invalid LINGTU_BRAINSTEM_TLS_SERVER_NAME: ${tls_server_name}" >&2
  exit 64
fi

temporary_env="$(mktemp)"
trap 'rm -f "${temporary_env}"' EXIT
umask 077
printf '%s\n' \
  "LINGTU_BRAINSTEM_HOST=${remote_host}" \
  "LINGTU_BRAINSTEM_PORT=${remote_port}" \
  "LINGTU_BRAINSTEM_TLS_CA_FILE=${tls_ca}" \
  "LINGTU_BRAINSTEM_TLS_CERT_FILE=${tls_cert}" \
  "LINGTU_BRAINSTEM_TLS_KEY_FILE=${tls_key}" \
  "LINGTU_BRAINSTEM_TLS_SERVER_NAME=${tls_server_name}" > "${temporary_env}"
sudo mkdir -p "${CONFIG_DIR}"
sudo install -o root -g root -m 0644 "${temporary_env}" "${BRAINSTEM_ENV}"
rm -f "${temporary_env}"
trap - EXIT
echo "Installed canonical mTLS Brainstem endpoint ${remote_host}:${remote_port}: ${BRAINSTEM_ENV}"

bash "${SCRIPT_DIR}/install_catalog_service.sh" driver

LEGACY_MOTION_SINKS=(
  "lingtu-thunder-dds-endpoint.service"
  "thunder-dds-endpoint.service"
  "dds-endpoint.service"
  "lingtu-thunder-lite.service"
  "driver.service"
)

for unit in "${LEGACY_MOTION_SINKS[@]}"; do
  if systemctl list-unit-files "${unit}" --no-legend 2>/dev/null \
      | grep -Fq "${unit}"; then
    sudo systemctl disable --now "${unit}"
    echo "Disabled legacy hardware motion sink: ${unit}"
  fi
done
