#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
TEST_ROOT="$(mktemp -d)"
trap 'rm -rf -- "${TEST_ROOT}"' EXIT

# Exercise the installer's real boot policy without installing system files.
BOOT_POLICY="$(awk '
  capture && /^echo "Installed Thunder runtime environment:/ { exit }
  /^REQUESTED_ENABLE=/ { capture = 1 }
  capture { print }
' "${ROOT}/scripts/deploy/thunder/install_catalog_service.sh")"
test -n "${BOOT_POLICY}"

sudo() {
  test "$1" = systemctl
  shift
  printf '%s\n' "$*" >> "${case_root}/calls"
  case "$*" in
    "enable lt-driver.service")
      ln -sf "${case_root}/lt-driver.service" "${case_root}/enabled"
      ;;
    "disable lt-driver.service")
      rm -f -- "${case_root}/enabled"
      ;;
    *)
      echo "Unexpected process or service operation: $*" >&2
      return 1
      ;;
  esac
}

run_case() {
  local case_root="${TEST_ROOT}/$1"
  local CATALOG_ENABLE_DEFAULT="$2"
  local ENABLE_DEFAULT="$3"
  local LINGTU_ENABLE_SERVICE="$4"
  local initially_enabled="$5"
  local expected_enabled="$6"
  local expected_call="$7"
  local SERVICE=driver
  local SERVICE_NAME=lt-driver.service
  mkdir -p "${case_root}"
  : > "${case_root}/lt-driver.service"
  : > "${case_root}/calls"
  if [[ "${initially_enabled}" == 1 ]]; then
    ln -s "${case_root}/lt-driver.service" "${case_root}/enabled"
  fi
  source /dev/stdin <<< "${BOOT_POLICY}"
  if [[ "${expected_enabled}" == 1 ]]; then
    test -L "${case_root}/enabled"
  else
    test ! -L "${case_root}/enabled"
  fi
  test "$(<"${case_root}/calls")" = "${expected_call}"
}

# An upgrade clears old enablement even if an environment requests enablement.
run_case driver-upgrade 0 0 '' 1 0 'disable lt-driver.service'
run_case driver-explicit-enable 0 0 1 1 0 'disable lt-driver.service'
run_case driver-default-override 0 1 '' 1 0 'disable lt-driver.service'
run_case driver-already-disabled 0 0 '' 0 0 'disable lt-driver.service'

# Catalog-authorized services retain the existing explicit-request semantics.
run_case allowed-default 1 1 '' 0 1 'enable lt-driver.service'
run_case allowed-explicit-enable 1 0 1 0 1 'enable lt-driver.service'
run_case allowed-explicit-disable 1 1 0 1 1 ''

echo "catalog boot enablement tests passed"
