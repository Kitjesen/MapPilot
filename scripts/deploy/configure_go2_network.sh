#!/usr/bin/env bash
set -euo pipefail

if [[ $# -ne 3 ]]; then
  echo "Usage: configure_go2_network.sh <interface> <address/prefix> <go2-probe-ip>" >&2
  exit 64
fi

INTERFACE="$1"
ADDRESS="$2"
PROBE_IP="$3"
PROFILE="lingtu-go2-${INTERFACE}"
ADDRESS_IP=""
ADDRESS_PREFIX=""

valid_ipv4() {
  local value="$1" part
  local -a octets
  [[ "${value}" =~ ^([0-9]{1,3}\.){3}[0-9]{1,3}$ ]] || return 1
  IFS=. read -r -a octets <<< "${value}"
  for part in "${octets[@]}"; do
    ((10#${part} <= 255)) || return 1
  done
}

if [[ ! "${INTERFACE}" =~ ^[A-Za-z0-9_.:-]+$ ]]; then
  echo "ERROR: invalid Go2 network interface: ${INTERFACE}" >&2
  exit 64
fi
if [[ "${ADDRESS}" =~ ^(.+)/([0-9]{1,2})$ ]]; then
  ADDRESS_IP="${BASH_REMATCH[1]}"
  ADDRESS_PREFIX="${BASH_REMATCH[2]}"
fi
if [[ -z "${ADDRESS_IP}" ]] || ! valid_ipv4 "${ADDRESS_IP}" ||
    ((10#${ADDRESS_PREFIX} > 32)); then
  echo "ERROR: invalid Go2 interface CIDR: ${ADDRESS}" >&2
  exit 64
fi
if ! valid_ipv4 "${PROBE_IP}"; then
  echo "ERROR: invalid Go2 probe IPv4 address: ${PROBE_IP}" >&2
  exit 64
fi
for command in nmcli ip ping; do
  if ! command -v "${command}" >/dev/null 2>&1; then
    echo "ERROR: Go2 network provisioning requires ${command}." >&2
    exit 69
  fi
done
if ! ip link show dev "${INTERFACE}" >/dev/null 2>&1; then
  echo "ERROR: Go2 network interface does not exist: ${INTERFACE}" >&2
  exit 69
fi

as_root() {
  if [[ ${EUID} -eq 0 ]]; then
    "$@"
  else
    sudo -n "$@"
  fi
}

if as_root nmcli -t -f NAME connection show | grep -Fxq "${PROFILE}"; then
  :
else
  as_root nmcli connection add type ethernet ifname "${INTERFACE}" con-name "${PROFILE}"
fi
as_root nmcli connection modify "${PROFILE}" \
  connection.interface-name "${INTERFACE}" \
  connection.autoconnect yes \
  connection.autoconnect-priority 100 \
  ipv4.method manual \
  ipv4.addresses "${ADDRESS}" \
  ipv4.gateway "" \
  ipv4.never-default yes \
  ipv6.method disabled
as_root nmcli connection up "${PROFILE}"

if ! ip -o -4 addr show dev "${INTERFACE}" | awk '{print $4}' | grep -Fxq "${ADDRESS}"; then
  echo "ERROR: ${INTERFACE} did not acquire ${ADDRESS}." >&2
  exit 69
fi
if ! ip -4 route get "${PROBE_IP}" oif "${INTERFACE}" >/dev/null 2>&1; then
  echo "ERROR: ${PROBE_IP} is not routed through ${INTERFACE}." >&2
  exit 69
fi
if ! ping -I "${INTERFACE}" -c 1 -W 1 "${PROBE_IP}" >/dev/null 2>&1; then
  echo "ERROR: Go2 motion host ${PROBE_IP} is not reachable through ${INTERFACE}." >&2
  exit 69
fi

echo "Go2 network ready: ${INTERFACE} ${ADDRESS} -> ${PROBE_IP}"
