#!/usr/bin/env bash
set -euo pipefail

BIN="${LINGTU_DRIVER_BIN:-/opt/lingtu/current/build/driver/lingtu_driver}"
REQUIRE_REMOTE="${LINGTU_BRAINSTEM_REQUIRE_REMOTE:-0}"

valid_ipv4() {
  local value="$1" part
  local -a octets
  [[ "${value}" =~ ^([0-9]{1,3}\.){3}[0-9]{1,3}$ ]] || return 1
  IFS=. read -r -a octets <<< "${value}"
  for part in "${octets[@]}"; do
    ((10#${part} <= 255)) || return 1
  done
}
while [[ $# -gt 0 ]]; do
  case "$1" in
    --require-remote)
      REQUIRE_REMOTE=1
      shift
      ;;
    --)
      shift
      break
      ;;
    *)
      echo "ERROR: unknown run_driver option: $1" >&2
      exit 64
      ;;
  esac
done

if [[ "${REQUIRE_REMOTE}" == "1" ]]; then
  BRAINSTEM_HOST="${LINGTU_BRAINSTEM_HOST:-}"
  if ! valid_ipv4 "${BRAINSTEM_HOST}" || [[ "${BRAINSTEM_HOST}" == 127.* ]] ||
      [[ "${BRAINSTEM_HOST}" == "0.0.0.0" ]]; then
    echo "ERROR: field driver requires the remote Brainstem's literal IPv4 address; got '${BRAINSTEM_HOST:-<unset>}'" >&2
    echo "Set LINGTU_BRAINSTEM_HOST in /opt/lingtu/config/brainstem.env." >&2
    exit 64
  fi
  for variable in \
      LINGTU_BRAINSTEM_TLS_CA_FILE \
      LINGTU_BRAINSTEM_TLS_CERT_FILE \
      LINGTU_BRAINSTEM_TLS_KEY_FILE; do
    path="${!variable:-}"
    if [[ -z "${path}" || ! -r "${path}" || ! -s "${path}" ]]; then
      echo "ERROR: remote Brainstem mTLS requires readable ${variable}; got '${path:-<unset>}'" >&2
      exit 64
    fi
  done
fi
if [[ ! -x "${BIN}" ]]; then
  echo "ERROR: native Thunder driver is missing or not executable: ${BIN}" >&2
  echo "Build it with: bash scripts/build/build_driver.sh" >&2
  exit 1
fi

exec "${BIN}"
