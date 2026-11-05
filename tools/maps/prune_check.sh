#!/usr/bin/env bash
set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

DEFAULT_BUILD_DIR="${REPO_ROOT}/build/prune"
if grep -qiE "(microsoft|wsl)" /proc/version 2>/dev/null; then
  DEFAULT_BUILD_DIR="${REPO_ROOT}/build/prune_wsl"
fi
BUILD_DIR="${LINGTU_PRUNE_BUILD_DIR:-${DEFAULT_BUILD_DIR}}"
STATUS="pass"
ITEMS=()

usage() {
  cat <<'EOF'
Usage:
  bash tools/maps/prune_check.sh [options]

Options:
  --help          Show this message.

Build the production cleaner and run a native smoke test.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    -h|--help)
      usage
      exit 0
      ;;
    *)
      printf 'ERROR: unknown option: %s\n' "$1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

json_escape() {
  local s="${1:-}"
  if have python3; then
    printf '%s' "${s}" | python3 -c 'import json, sys; sys.stdout.write(json.dumps(sys.stdin.read())[1:-1])'
    return
  fi
  s="${s//\\/\\\\}"
  s="${s//\"/\\\"}"
  s="${s//$'\n'/\\n}"
  s="${s//$'\r'/}"
  s="$(printf '%s' "${s}" | tr -d '\000-\010\013\014\016-\037')"
  printf '%s' "${s}"
}

add_item() {
  local name="$1"
  local state="$2"
  local detail="${3:-}"
  ITEMS+=("{\"name\":\"$(json_escape "${name}")\",\"state\":\"$(json_escape "${state}")\",\"detail\":\"$(json_escape "${detail}")\"}")
  if [[ "${state}" == "fail" ]]; then
    STATUS="fail"
  elif [[ "${state}" == "blocked" && "${STATUS}" == "pass" ]]; then
    STATUS="blocked"
  fi
}

have() {
  command -v "$1" >/dev/null 2>&1
}

run_step() {
  local name="$1"
  shift
  local output rc
  output="$("$@" 2>&1)"
  rc=$?
  if [[ ${rc} -eq 0 ]]; then
    add_item "${name}" "pass" "${output}"
  else
    add_item "${name}" "fail" "rc=${rc}; ${output}"
  fi
  return "${rc}"
}

write_smoke_map() {
  local root="$1"
  rm -rf "${root}"
  mkdir -p "${root}/patches"
  cat >"${root}/map.pcd" <<'PCD'
# .PCD v0.7
VERSION 0.7
FIELDS x y z intensity
SIZE 4 4 4 4
TYPE F F F F
COUNT 1 1 1 1
WIDTH 13
HEIGHT 1
POINTS 13
DATA ascii
0 0 -1.5 1
1 0 0.0 1
2 0 0.0 1
0 2 0.0 1
2 2 0.0 1
4 0 -1.5 1
5 0 0.0 1
6 0 0.0 1
4 2 0.0 1
6 2 0.0 1
0 4 -1.5 1
1 4 0.0 1
2 4 0.0 1
PCD
  for frame in 000000 000001 000002; do
    cat >"${root}/patches/${frame}.pcd" <<'PCD'
# .PCD v0.7
VERSION 0.7
FIELDS x y z intensity
SIZE 4 4 4 4
TYPE F F F F
COUNT 1 1 1 1
WIDTH 5
HEIGHT 1
POINTS 5
DATA ascii
0 0 -1.5 1
1 0 0.0 1
2 0 0.0 1
0 2 0.0 1
2 2 0.0 1
PCD
  done
  cat >"${root}/poses.txt" <<'POSES'
000000.pcd 0 0 0 1 0 0 0
000001.pcd 4 0 0 1 0 0 0
000002.pcd 0 4 0 1 0 0 0
POSES
  echo 'lidar_origin_in_patch 0 0 0' >"${root}/scan_origin.txt"
}

cd "${REPO_ROOT}" || exit 1

for cmd in cmake c++; do
  if have "${cmd}"; then
    add_item "cmd:${cmd}" "pass" "$("${cmd}" --version 2>/dev/null | head -1)"
  else
    add_item "cmd:${cmd}" "fail" "missing command"
  fi
done

run_step "build_default_prune" env LINGTU_PRUNE_BUILD_DIR="${BUILD_DIR}" bash scripts/build/build_prune.sh

SMOKE_ROOT="${TMPDIR:-/tmp}/lingtu_prune_check_map"
write_smoke_map "${SMOKE_ROOT}"

if [[ -x "${BUILD_DIR}/prune" ]]; then
  run_step "prune_smoke" "${BUILD_DIR}/prune" --map-dir "${SMOKE_ROOT}" --overwrite
else
  add_item "prune_smoke" "fail" "missing ${BUILD_DIR}/prune"
fi

printf '{\n'
printf '  "schema": "lingtu.prune_check.v1",\n'
printf '  "status": "%s",\n' "$(json_escape "${STATUS}")"
printf '  "repo_root": "%s",\n' "$(json_escape "${REPO_ROOT}")"
printf '  "build_dir": "%s",\n' "$(json_escape "${BUILD_DIR}")"
printf '  "items": [\n'
for i in "${!ITEMS[@]}"; do
  if [[ "${i}" -gt 0 ]]; then
    printf ',\n'
  fi
  printf '    %s' "${ITEMS[${i}]}"
done
printf '\n  ]\n'
printf '}\n'

if [[ "${STATUS}" == "fail" ]]; then
  exit 1
fi
exit 0
