#!/usr/bin/env bash
set -euo pipefail

: "${LINGTU_REPO:=/opt/lingtu/current}"
BIN="${LINGTU_DRIVER_BIN:-${LINGTU_REPO}/bin/lingtu_driver}"
BACKEND="${LINGTU_DRIVER_BACKEND:?LINGTU_DRIVER_BACKEND is required from the Product session}"

valid_ipv4() {
  local value="$1" part
  local -a octets
  [[ "${value}" =~ ^([0-9]{1,3}\.){3}[0-9]{1,3}$ ]] || return 1
  IFS=. read -r -a octets <<< "${value}"
  for part in "${octets[@]}"; do
    ((10#${part} <= 255)) || return 1
  done
}
valid_ipv4_cidr() {
  local value="$1" address prefix
  [[ "${value}" =~ ^(.+)/([0-9]{1,2})$ ]] || return 1
  address="${BASH_REMATCH[1]}"
  prefix="${BASH_REMATCH[2]}"
  valid_ipv4 "${address}" || return 1
  ((10#${prefix} <= 32))
}

verify_unitree_dds_runtime() {
  local magic ldd_output bad_dds
  magic="$(LC_ALL=C od -An -tx1 -N4 "${BIN}" 2>/dev/null | tr -d ' \n')"
  if [[ "${magic}" != "7f454c46" ]]; then
    return 0
  fi
  if ! command -v ldd >/dev/null 2>&1; then
    echo "ERROR: ldd is required to verify the Go2 DDS runtime." >&2
    exit 69
  fi
  if ! ldd_output="$(ldd "${BIN}" 2>&1)"; then
    echo "ERROR: cannot inspect native Go2 driver dependencies." >&2
    echo "${ldd_output}" >&2
    exit 69
  fi
  if grep -Fq "not found" <<< "${ldd_output}"; then
    echo "ERROR: native Go2 driver has unresolved shared libraries." >&2
    echo "${ldd_output}" >&2
    exit 69
  fi
  bad_dds="$(
    awk -v prefix="${UNITREE_PREFIX}/lib/" \
      '$1 ~ /^libddsc(xx)?\.so/ && index($3, prefix) != 1 {print $1 " => " $3}' \
      <<< "${ldd_output}"
  )"
  if [[ -n "${bad_dds}" ]]; then
    echo "ERROR: Go2 driver resolved CycloneDDS outside ${UNITREE_PREFIX}/lib:" >&2
    echo "${bad_dds}" >&2
    exit 69
  fi
  if ! grep -Eq '^[[:space:]]*libddsc\.so' <<< "${ldd_output}"; then
    echo "ERROR: Go2 driver did not resolve SDK2 libddsc." >&2
    exit 69
  fi
}
if [[ $# -ne 0 ]]; then
  echo "ERROR: run_driver does not accept arguments; configuration comes from the Product session." >&2
  exit 64
fi
if [[ ! -x "${BIN}" ]]; then
  echo "ERROR: native ${BACKEND} driver is missing or not executable: ${BIN}" >&2
  echo "Build it with: LINGTU_DRIVER_BACKEND=${BACKEND} bash scripts/build/build_driver.sh" >&2
  exit 1
fi

case "${BACKEND}" in
go2)
  GO2_INTERFACE="${LINGTU_DRIVER_NETWORK_INTERFACE:-}"
  GO2_ADDRESS="${LINGTU_DRIVER_NETWORK_ADDRESS:-}"
  GO2_PROBE_IP="${LINGTU_DRIVER_PROBE_IP:-}"
  UNITREE_PREFIX="${LINGTU_UNITREE_SDK2_PREFIX:-/opt/unitree_robotics}"
  if [[ -z "${GO2_INTERFACE}" ]] ||
      [[ ! "${GO2_INTERFACE}" =~ ^[A-Za-z0-9_.:-]+$ ]]; then
    echo "ERROR: Go2 requires LINGTU_DRIVER_NETWORK_INTERFACE." >&2
    exit 64
  fi
  if ! valid_ipv4_cidr "${GO2_ADDRESS}"; then
    echo "ERROR: Go2 requires LINGTU_DRIVER_NETWORK_ADDRESS as IPv4 CIDR." >&2
    exit 64
  fi
  if ! valid_ipv4 "${GO2_PROBE_IP}"; then
    echo "ERROR: Go2 requires LINGTU_DRIVER_PROBE_IP." >&2
    exit 64
  fi
  if [[ ! -d "/sys/class/net/${GO2_INTERFACE}" ]]; then
    echo "ERROR: Go2 network interface does not exist: ${GO2_INTERFACE}" >&2
    exit 69
  fi
  if [[ ! -r "/sys/class/net/${GO2_INTERFACE}/carrier" ]]; then
    echo "ERROR: Go2 network carrier state is unavailable: ${GO2_INTERFACE}" >&2
    exit 69
  fi
  if [[ "$(< "/sys/class/net/${GO2_INTERFACE}/carrier")" != "1" ]]; then
    echo "ERROR: Go2 network interface has no carrier: ${GO2_INTERFACE}" >&2
    exit 69
  fi
  if ! command -v ip >/dev/null 2>&1; then
    echo "ERROR: Go2 startup preflight requires iproute2." >&2
    exit 69
  fi
  if ! ip -o -4 addr show dev "${GO2_INTERFACE}" | awk '{print $4}' | grep -Fxq "${GO2_ADDRESS}"; then
    echo "ERROR: ${GO2_INTERFACE} does not own ${GO2_ADDRESS}." >&2
    exit 69
  fi
  if ! ip -4 route get "${GO2_PROBE_IP}" oif "${GO2_INTERFACE}" >/dev/null 2>&1; then
    echo "ERROR: ${GO2_PROBE_IP} is not routed through ${GO2_INTERFACE}." >&2
    exit 69
  fi
  if [[ "${LINGTU_DDS_NETWORK_INTERFACE:-}" != "${GO2_INTERFACE}" ]]; then
    echo "ERROR: LingTu DDS and Go2 SDK2 network interfaces disagree." >&2
    exit 64
  fi
  export LD_LIBRARY_PATH="${UNITREE_PREFIX}/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
  verify_unitree_dds_runtime
  ;;
doso)
  DRIVER_TARGET="${LINGTU_DRIVER_TARGET:-}"
  BRAINSTEM_HOST="${LINGTU_BRAINSTEM_HOST:-${DRIVER_TARGET%%:*}}"
  BRAINSTEM_PORT="${LINGTU_BRAINSTEM_PORT:-${DRIVER_TARGET##*:}}"
  if ! valid_ipv4 "${BRAINSTEM_HOST}" || [[ "${BRAINSTEM_HOST}" == 127.* ]] ||
      [[ "${BRAINSTEM_HOST}" == "0.0.0.0" ]]; then
    echo "ERROR: field driver requires the remote Brainstem's literal IPv4 address; got '${BRAINSTEM_HOST:-<unset>}'" >&2
    echo "Set driver.target in the selected robot's robot.yaml." >&2
    exit 64
  fi
  for variable in \
      LINGTU_DRIVER_TLS_CA_FILE \
      LINGTU_DRIVER_TLS_CERT_FILE \
      LINGTU_DRIVER_TLS_KEY_FILE; do
    path="${!variable:-}"
    if [[ -z "${path}" || ! -r "${path}" || ! -s "${path}" ]]; then
      echo "ERROR: remote Brainstem mTLS requires readable ${variable}; got '${path:-<unset>}'" >&2
      exit 64
    fi
  done
  export LINGTU_BRAINSTEM_HOST="${BRAINSTEM_HOST}"
  if [[ "${BRAINSTEM_PORT}" =~ ^[0-9]+$ ]]; then
    export LINGTU_BRAINSTEM_PORT="${BRAINSTEM_PORT}"
  fi
  ;;
*)
  echo "ERROR: unsupported LINGTU_DRIVER_BACKEND: ${BACKEND}" >&2
  exit 64
  ;;
esac

exec "${BIN}"
