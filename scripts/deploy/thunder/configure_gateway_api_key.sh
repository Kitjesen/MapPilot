#!/usr/bin/env bash
# Create protected, least-privilege credentials consumed by lt-host.service.

set -euo pipefail

readonly DEFAULT_GATEWAY_ENV_FILE="/etc/lingtu/gateway.env"
readonly DEFAULT_MAP_CLIENT_ENV_FILE="/etc/lingtu/map-client.env"
gateway_target="${LINGTU_GATEWAY_ENV_FILE:-${DEFAULT_GATEWAY_ENV_FILE}}"
map_target="${LINGTU_MAP_CLIENT_ENV_FILE:-${DEFAULT_MAP_CLIENT_ENV_FILE}}"
rotate_all=0
rotate_map=0
gateway_temp=""
map_temp=""

usage() {
  cat <<'EOF'
Usage: configure_gateway_api_key.sh [--rotate | --rotate-map]

Provision separate operator and customer-map credentials:
  /etc/lingtu/gateway.env    root:root    0600  LINGTU_API_KEY only
  /etc/lingtu/map-client.env root:lingtu  0640  LINGTU_MAP_API_KEY only

A legacy one-key Gateway file gains a new map-client credential. A legacy
Gateway file containing both keys is securely split without rotating either
key. Existing split credentials are left unchanged. Use --rotate to replace
both credentials or --rotate-map to replace only the map credential.

For isolated tests only, LINGTU_GATEWAY_ENV_FILE and
LINGTU_MAP_CLIENT_ENV_FILE may select alternate paths.
EOF
}

while (($# > 0)); do
  case "$1" in
    --rotate) rotate_all=1 ;;
    --rotate-map) rotate_map=1 ;;
    -h|--help) usage; exit 0 ;;
    *)
      printf 'Unknown argument: %s\n' "$1" >&2
      usage >&2
      exit 64
      ;;
  esac
  shift
done

if ((rotate_all == 1 && rotate_map == 1)); then
  printf '%s\n' '--rotate and --rotate-map are mutually exclusive.' >&2
  exit 64
fi
if [[ "${gateway_target}" == "${map_target}" ]]; then
  printf 'Gateway and map-client API key targets must differ.\n' >&2
  exit 78
fi
if [[ "${gateway_target}" == "${DEFAULT_GATEWAY_ENV_FILE}" && "${EUID}" -ne 0 ]]; then
  printf 'Run as root to write %s.\n' "${DEFAULT_GATEWAY_ENV_FILE}" >&2
  exit 77
fi
if [[ "${map_target}" == "${DEFAULT_MAP_CLIENT_ENV_FILE}" && "${EUID}" -ne 0 ]]; then
  printf 'Run as root to write %s.\n' "${DEFAULT_MAP_CLIENT_ENV_FILE}" >&2
  exit 77
fi
if ! command -v stat >/dev/null 2>&1; then
  printf 'stat is required to validate API key paths.\n' >&2
  exit 69
fi

current_uid="$(id -u)"
current_gid="$(id -g)"
gateway_expected_uid="${current_uid}"
gateway_expected_gid="${current_gid}"
map_expected_uid="${current_uid}"
map_expected_gid="${current_gid}"
gateway_parent_uid="${current_uid}"
gateway_parent_gid="${current_gid}"
map_parent_uid="${current_uid}"
map_parent_gid="${current_gid}"
if [[ "${gateway_target}" == "${DEFAULT_GATEWAY_ENV_FILE}" ]]; then
  gateway_expected_uid=0
  gateway_expected_gid=0
  gateway_parent_uid=0
  gateway_parent_gid=0
fi
if [[ "${map_target}" == "${DEFAULT_MAP_CLIENT_ENV_FILE}" ]]; then
  map_parent_uid=0
  map_parent_gid=0
  map_expected_uid=0
  runtime_group="${LINGTU_RUNTIME_GROUP:-lingtu}"
  if ! map_expected_gid="$(getent group "${runtime_group}" | cut -d: -f3)" ||
      [[ -z "${map_expected_gid}" ]]; then
    printf 'The %s group is required to own the map-client credential.\n' "${runtime_group}" >&2
    exit 78
  fi
fi

validate_parent() {
  local target="$1"
  local label="$2"
  local expected_uid="$3"
  local expected_gid="$4"
  local target_dir
  target_dir="$(dirname -- "${target}")"

  if [[ -L "${target_dir}" ]]; then
    printf 'Refusing symlink %s credential directory: %s\n' "${label}" "${target_dir}" >&2
    return 1
  fi
  if [[ ! -e "${target_dir}" ]]; then
    if [[ "${EUID}" -eq 0 ]]; then
      install -d -o "${expected_uid}" -g "${expected_gid}" -m 0755 "${target_dir}"
    else
      mkdir -p -- "${target_dir}"
    fi
  fi
  if [[ ! -d "${target_dir}" || -L "${target_dir}" ]]; then
    printf '%s credential parent is not a safe directory: %s\n' "${label}" "${target_dir}" >&2
    return 1
  fi

  local metadata directory_uid directory_gid directory_mode directory_mode_value
  metadata="$(stat -c '%u:%g:%a' -- "${target_dir}")" || return 1
  IFS=: read -r directory_uid directory_gid directory_mode <<<"${metadata}"
  if [[ ! "${directory_mode}" =~ ^[0-7]{3,4}$ ]]; then
    printf 'Cannot validate permissions for %s credential directory: %s\n' "${label}" "${target_dir}" >&2
    return 1
  fi
  directory_mode_value=$((8#${directory_mode}))
  if [[ "${directory_uid}:${directory_gid}" != "${expected_uid}:${expected_gid}" ]] \
      || ((directory_mode_value & 0022)); then
    printf '%s credential directory has unsafe owner or permissions: %s\n' "${label}" "${target_dir}" >&2
    return 1
  fi
}

if ! validate_parent \
    "${gateway_target}" "Gateway" "${gateway_parent_uid}" "${gateway_parent_gid}"; then
  exit 78
fi
if ! validate_parent \
    "${map_target}" "map-client" "${map_parent_uid}" "${map_parent_gid}"; then
  exit 78
fi

canonical_target() {
  local target="$1"
  local target_dir
  target_dir="$(cd -P -- "$(dirname -- "${target}")" && pwd)" || return 1
  printf '%s/%s\n' "${target_dir%/}" "$(basename -- "${target}")"
}
canonical_gateway_target="$(canonical_target "${gateway_target}")" || exit 78
canonical_map_target="$(canonical_target "${map_target}")" || exit 78
if [[ "${canonical_gateway_target}" == "${canonical_map_target}" ]]; then
  printf 'Gateway and map-client API key targets must differ.\n' >&2
  exit 78
fi

valid_key() {
  [[ "${#1}" -ge 64 && "$1" =~ ^[A-Za-z0-9_-]+$ ]]
}

validate_file_metadata() {
  local target="$1"
  local label="$2"
  local expected_uid="$3"
  local expected_gid="$4"
  local expected_mode="$5"

  if [[ -L "${target}" || ! -f "${target}" ]]; then
    printf '%s credential is not a regular non-symlink file: %s\n' "${label}" "${target}" >&2
    return 1
  fi
  local metadata
  metadata="$(stat -c '%u:%g:%a:%h' -- "${target}")" || return 1
  if [[ "${metadata}" != "${expected_uid}:${expected_gid}:${expected_mode}:1" ]]; then
    printf '%s credential has unsafe owner, mode, or link count: %s\n' "${label}" "${target}" >&2
    return 1
  fi
}

file_snapshot() {
  stat -c '%d:%i:%s:%y:%u:%g:%a:%h' -- "$1"
}

gateway_format=""
existing_api_key=""
embedded_map_key=""
parse_gateway_file() {
  local target="$1"
  local lines=()
  local line_count
  mapfile -t lines <"${target}" || return 1
  line_count="$(wc -l <"${target}")" || return 1

  gateway_format=""
  existing_api_key=""
  embedded_map_key=""
  if [[ "${line_count}" -eq 1 && "${#lines[@]}" -eq 1 \
      && "${lines[0]}" =~ ^LINGTU_API_KEY=[A-Za-z0-9_-]{64,}$ ]]; then
    gateway_format="operator_only"
    existing_api_key="${lines[0]#LINGTU_API_KEY=}"
    return 0
  fi
  if [[ "${line_count}" -eq 2 && "${#lines[@]}" -eq 2 \
      && "${lines[0]}" =~ ^LINGTU_API_KEY=[A-Za-z0-9_-]{64,}$ \
      && "${lines[1]}" =~ ^LINGTU_MAP_API_KEY=[A-Za-z0-9_-]{64,}$ ]]; then
    gateway_format="embedded_map"
    existing_api_key="${lines[0]#LINGTU_API_KEY=}"
    embedded_map_key="${lines[1]#LINGTU_MAP_API_KEY=}"
    return 0
  fi
  return 1
}

existing_map_key=""
parse_map_file() {
  local target="$1"
  local lines=()
  local line_count
  mapfile -t lines <"${target}" || return 1
  line_count="$(wc -l <"${target}")" || return 1

  existing_map_key=""
  if [[ "${line_count}" -eq 1 && "${#lines[@]}" -eq 1 \
      && "${lines[0]}" =~ ^LINGTU_MAP_API_KEY=[A-Za-z0-9_-]{64,}$ ]]; then
    existing_map_key="${lines[0]#LINGTU_MAP_API_KEY=}"
    return 0
  fi
  return 1
}

gateway_exists=0
gateway_snapshot=""
gateway_content_valid=0
if [[ -e "${gateway_target}" || -L "${gateway_target}" ]]; then
  if ! validate_file_metadata \
      "${gateway_target}" "Gateway" "${gateway_expected_uid}" \
      "${gateway_expected_gid}" 600; then
    exit 78
  fi
  gateway_exists=1
  gateway_snapshot="$(file_snapshot "${gateway_target}")" || exit 78
  if parse_gateway_file "${gateway_target}"; then
    gateway_content_valid=1
  fi
fi

map_exists=0
map_snapshot=""
map_content_valid=0
if [[ -e "${map_target}" || -L "${map_target}" ]]; then
  if ! validate_file_metadata \
      "${map_target}" "map-client" "${map_expected_uid}" \
      "${map_expected_gid}" 640; then
    exit 78
  fi
  map_exists=1
  map_snapshot="$(file_snapshot "${map_target}")" || exit 78
  if parse_map_file "${map_target}"; then
    map_content_valid=1
  fi
fi

original_api_key="${existing_api_key}"
original_embedded_map_key="${embedded_map_key}"
original_map_key="${existing_map_key}"
api_key=""
map_api_key=""
change_gateway=0
change_map=0
need_generate_api=0
need_generate_map=0
action=""

if ((rotate_all == 1)); then
  need_generate_api=1
  need_generate_map=1
  change_gateway=1
  change_map=1
  action="Rotated"
elif ((rotate_map == 1)); then
  if ((gateway_exists == 0)) || ((gateway_content_valid == 0)); then
    printf 'Map-key rotation requires a valid Gateway operator credential.\n' >&2
    exit 78
  fi
  api_key="${existing_api_key}"
  need_generate_map=1
  change_map=1
  if [[ "${gateway_format}" == "embedded_map" ]]; then
    change_gateway=1
  fi
  action="Rotated map"
else
  if ((gateway_exists == 0)); then
    if ((map_exists == 1)); then
      printf 'Refusing incomplete split credentials: Gateway credential is missing.\n' >&2
      exit 78
    fi
    need_generate_api=1
    need_generate_map=1
    change_gateway=1
    change_map=1
    action="Created"
  else
    if ((gateway_content_valid == 0)); then
      printf 'Existing Gateway credential has invalid content; use --rotate only after inspecting it.\n' >&2
      exit 78
    fi
    api_key="${existing_api_key}"
    if [[ "${gateway_format}" == "embedded_map" ]]; then
      if [[ "${embedded_map_key}" == "${api_key}" ]]; then
        printf 'Gateway operator and map API keys must differ.\n' >&2
        exit 78
      fi
      map_api_key="${embedded_map_key}"
      if ((map_exists == 1)); then
        if ((map_content_valid == 0)); then
          printf 'Existing map-client credential has invalid content.\n' >&2
          exit 78
        fi
        if [[ "${existing_map_key}" != "${embedded_map_key}" ]]; then
          printf 'Refusing conflicting embedded and map-client credentials; use explicit rotation.\n' >&2
          exit 78
        fi
      else
        change_map=1
      fi
      change_gateway=1
      action="Migrated"
    elif ((map_exists == 1)); then
      if ((map_content_valid == 0)); then
        printf 'Existing map-client credential has invalid content.\n' >&2
        exit 78
      fi
      map_api_key="${existing_map_key}"
      if [[ "${map_api_key}" == "${api_key}" ]]; then
        printf 'Gateway operator and map API keys must differ.\n' >&2
        exit 78
      fi
      action="Unchanged"
    else
      need_generate_map=1
      change_map=1
      action="Migrated"
    fi
  fi
fi

if ((need_generate_api == 1 || need_generate_map == 1)); then
  if ! command -v python3 >/dev/null 2>&1; then
    printf 'python3 is required to generate API keys.\n' >&2
    exit 69
  fi
fi

generate_key() {
  python3 - <<'PY'
import secrets

print(secrets.token_urlsafe(48))
PY
}

if ((need_generate_api == 1)); then
  api_key="$(generate_key)"
  while [[ "${api_key}" == "${original_api_key}" \
      || "${api_key}" == "${original_map_key}" \
      || "${api_key}" == "${original_embedded_map_key}" ]]; do
    api_key="$(generate_key)"
  done
fi
if ((need_generate_map == 1)); then
  map_api_key="$(generate_key)"
  while [[ "${map_api_key}" == "${api_key}" \
      || "${map_api_key}" == "${original_map_key}" \
      || "${map_api_key}" == "${original_embedded_map_key}" ]]; do
    map_api_key="$(generate_key)"
  done
fi
if ! valid_key "${api_key}" || ! valid_key "${map_api_key}" \
    || [[ "${api_key}" == "${map_api_key}" ]]; then
  printf 'Generated API keys failed validation.\n' >&2
  exit 70
fi

cleanup() {
  if [[ -n "${gateway_temp}" ]]; then
    rm -f -- "${gateway_temp}"
  fi
  if [[ -n "${map_temp}" ]]; then
    rm -f -- "${map_temp}"
  fi
}
trap cleanup EXIT
umask 077

validate_staged_content() {
  local target="$1"
  local key_name="$2"
  local expected_key="$3"
  local lines=()
  local line_count
  mapfile -t lines <"${target}" || return 1
  line_count="$(wc -l <"${target}")" || return 1
  [[ "${line_count}" -eq 1 && "${#lines[@]}" -eq 1 \
      && "${lines[0]}" == "${key_name}=${expected_key}" ]]
}

prepare_credential_temp() {
  local target="$1"
  local prefix="$2"
  local mode="$3"
  local expected_uid="$4"
  local expected_gid="$5"
  local key_name="$6"
  local key_value="$7"
  local output_variable="$8"
  local label="$9"
  local target_dir temporary_file
  target_dir="$(dirname -- "${target}")"
  if ! temporary_file="$(mktemp "${target_dir}/.${prefix}.XXXXXX")"; then
    printf 'Failed to stage %s credential.\n' "${label}" >&2
    return 1
  fi
  if ! printf '%s=%s\n' "${key_name}" "${key_value}" >"${temporary_file}" \
      || ! chmod "${mode}" "${temporary_file}"; then
    printf 'Failed to write staged %s credential.\n' "${label}" >&2
    rm -f -- "${temporary_file}"
    return 1
  fi
  if [[ "${EUID}" -eq 0 ]]; then
    if ! chown "${expected_uid}:${expected_gid}" "${temporary_file}"; then
      printf 'Failed to set staged %s credential ownership.\n' "${label}" >&2
      rm -f -- "${temporary_file}"
      return 1
    fi
  elif ! chgrp "${expected_gid}" "${temporary_file}"; then
    printf 'Failed to set staged %s credential group.\n' "${label}" >&2
    rm -f -- "${temporary_file}"
    return 1
  fi
  if ! validate_file_metadata \
      "${temporary_file}" "${label}" "${expected_uid}" "${expected_gid}" "${mode}" \
      || ! validate_staged_content "${temporary_file}" "${key_name}" "${key_value}"; then
    printf 'Staged %s credential failed validation.\n' "${label}" >&2
    rm -f -- "${temporary_file}"
    return 1
  fi
  printf -v "${output_variable}" '%s' "${temporary_file}"
}

assert_target_unchanged() {
  local target="$1"
  local existed="$2"
  local snapshot="$3"
  local label="$4"
  local expected_uid="$5"
  local expected_gid="$6"
  local expected_mode="$7"

  if ((existed == 1)); then
    if ! validate_file_metadata \
        "${target}" "${label}" "${expected_uid}" "${expected_gid}" "${expected_mode}"; then
      return 1
    fi
    if [[ "$(file_snapshot "${target}")" != "${snapshot}" ]]; then
      printf '%s credential changed during provisioning: %s\n' "${label}" "${target}" >&2
      return 1
    fi
  elif [[ -e "${target}" || -L "${target}" ]]; then
    printf '%s credential appeared during provisioning: %s\n' "${label}" "${target}" >&2
    return 1
  fi
}

install_prepared_file() {
  local temporary_file="$1"
  local target="$2"
  local existed="$3"
  local snapshot="$4"
  local label="$5"
  local expected_uid="$6"
  local expected_gid="$7"
  local expected_mode="$8"
  local temporary_variable="$9"

  if ! assert_target_unchanged \
      "${target}" "${existed}" "${snapshot}" "${label}" \
      "${expected_uid}" "${expected_gid}" "${expected_mode}"; then
    return 1
  fi
  if ((existed == 1)); then
    if ! mv -fT -- "${temporary_file}" "${target}"; then
      printf 'Failed to replace %s credential: %s\n' "${label}" "${target}" >&2
      return 1
    fi
  else
    if ! ln -T -- "${temporary_file}" "${target}" 2>/dev/null; then
      printf 'Failed to install %s credential without overwriting another file: %s\n' \
        "${label}" "${target}" >&2
      return 1
    fi
    if ! rm -f -- "${temporary_file}"; then
      printf 'Failed to finalize %s credential installation: %s\n' "${label}" "${target}" >&2
      return 1
    fi
  fi
  printf -v "${temporary_variable}" ''
}

validate_gateway_final() {
  local expected_key="$1"
  if ! validate_file_metadata \
      "${gateway_target}" "Gateway" "${gateway_expected_uid}" \
      "${gateway_expected_gid}" 600 \
      || ! parse_gateway_file "${gateway_target}" \
      || [[ "${gateway_format}" != "operator_only" \
          || "${existing_api_key}" != "${expected_key}" ]]; then
    printf 'Final Gateway credential validation failed: %s\n' "${gateway_target}" >&2
    return 1
  fi
}

validate_map_final() {
  local expected_key="$1"
  if ! validate_file_metadata \
      "${map_target}" "map-client" "${map_expected_uid}" \
      "${map_expected_gid}" 640 \
      || ! parse_map_file "${map_target}" \
      || [[ "${existing_map_key}" != "${expected_key}" ]]; then
    printf 'Final map-client credential validation failed: %s\n' "${map_target}" >&2
    return 1
  fi
}

if ((change_map == 1)); then
  if ! prepare_credential_temp \
      "${map_target}" map-client.env 640 "${map_expected_uid}" "${map_expected_gid}" \
      LINGTU_MAP_API_KEY "${map_api_key}" map_temp map-client; then
    exit 78
  fi
fi
if ((change_gateway == 1)); then
  if ! prepare_credential_temp \
      "${gateway_target}" gateway.env 600 "${gateway_expected_uid}" \
      "${gateway_expected_gid}" LINGTU_API_KEY "${api_key}" gateway_temp Gateway; then
    exit 78
  fi
fi

# Land and verify the map-client credential before removing an embedded map key
# from the Gateway file. This makes interrupted legacy migrations converge safely.
if ((change_map == 1)); then
  if ! install_prepared_file \
      "${map_temp}" "${map_target}" "${map_exists}" "${map_snapshot}" map-client \
      "${map_expected_uid}" "${map_expected_gid}" 640 map_temp; then
    exit 78
  fi
fi
if ! validate_map_final "${map_api_key}"; then
  exit 78
fi
if ((change_gateway == 1)); then
  if ! install_prepared_file \
      "${gateway_temp}" "${gateway_target}" "${gateway_exists}" "${gateway_snapshot}" Gateway \
      "${gateway_expected_uid}" "${gateway_expected_gid}" 600 gateway_temp; then
    exit 78
  fi
fi
if ! validate_gateway_final "${api_key}" || ! validate_map_final "${map_api_key}"; then
  exit 78
fi

trap - EXIT
case "${action}" in
  Created)
    printf 'Created scoped Gateway and map-client API key files at %s and %s.\n' \
      "${gateway_target}" "${map_target}"
    ;;
  Migrated)
    printf 'Migrated scoped Gateway and map-client API key files at %s and %s.\n' \
      "${gateway_target}" "${map_target}"
    ;;
  Rotated)
    printf 'Rotated scoped Gateway and map-client API key files at %s and %s.\n' \
      "${gateway_target}" "${map_target}"
    ;;
  "Rotated map")
    printf 'Rotated map-client API key file at %s; Gateway key was preserved.\n' \
      "${map_target}"
    ;;
  Unchanged)
    printf 'Scoped API key files already exist; left unchanged. Use --rotate or --rotate-map.\n' >&2
    ;;
esac
printf 'Secrets were not printed. Switch the current Product after provisioning clients:\n'
printf '  bash scripts/lingtu switch <product> --robot <vendor/model> --env real\n'
