#!/usr/bin/env bash
set -euo pipefail

build_dir="${1:-/opt/lingtu/current/build/nav_endpoint}"
source_dir="${2:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)}"
domain_id="${LINGTU_TEST_DDS_DOMAIN_ID:-191}"
status_file="${TMPDIR:-/tmp}/lingtu_nav_command_ack_status.json"
log_file="${TMPDIR:-/tmp}/lingtu_nav_command_ack.log"

endpoint="${build_dir}/lingtu_nav_native_endpoint"
control="${build_dir}/lingtu_nav_control"
path_library="${source_dir}/src/nav/local/paths"

for file in "${endpoint}" "${control}"; do
  if [[ ! -x "${file}" ]]; then
    echo "missing executable: ${file}" >&2
    exit 2
  fi
done

rm -f "${status_file}" "${log_file}"
"${endpoint}" \
  --path-library "${path_library}" \
  --domain-id "${domain_id}" \
  --control-mode teleop \
  --check-obstacle false \
  --publish-cmd-vel false \
  --status-s 0.25 \
  --status-file "${status_file}" \
  >"${log_file}" 2>&1 &
endpoint_pid=$!

cleanup() {
  rc=$?
  endpoint_alive=false
  if kill -0 "${endpoint_pid}" 2>/dev/null; then
    endpoint_alive=true
    kill "${endpoint_pid}" 2>/dev/null || true
  fi
  set +e
  wait "${endpoint_pid}"
  endpoint_rc=$?
  set -e
  if [[ ${rc} -ne 0 ]]; then
    echo "endpoint_alive_at_cleanup=${endpoint_alive} endpoint_rc=${endpoint_rc}" >&2
    echo "--- navigation endpoint log ---" >&2
    cat "${log_file}" >&2 || true
    echo "--- navigation endpoint status ---" >&2
    cat "${status_file}" >&2 || true
  fi
  return "${rc}"
}
trap cleanup EXIT

sleep 1
"${control}" teleop 0.2 -0.1 0.3 --domain-id "${domain_id}"
"${control}" cancel ack_smoke --domain-id "${domain_id}"
sleep 0.5

if [[ ! -s "${status_file}" ]]; then
  echo "navigation endpoint did not write a status snapshot" >&2
  cat "${log_file}" >&2 || true
  exit 1
fi
grep -Eq '"control_mode"[[:space:]]*:[[:space:]]*"teleop"' "${status_file}"
grep -Eq '"teleop_cmd"[[:space:]]*:[[:space:]]*[1-9][0-9]*' "${status_file}"
grep -Eq '"cancels"[[:space:]]*:[[:space:]]*[1-9][0-9]*' "${status_file}"
grep -Eq '"command_acks"[[:space:]]*:[[:space:]]*2' "${status_file}"

echo "typed navigation command request/ACK smoke: PASS"
grep -Eo '"control_mode"[[:space:]]*:[[:space:]]*"[^"]+"|"teleop_cmd"[[:space:]]*:[[:space:]]*[0-9]+|"cancels"[[:space:]]*:[[:space:]]*[0-9]+|"command_acks"[[:space:]]*:[[:space:]]*[0-9]+' \
  "${status_file}" | head -n 3
