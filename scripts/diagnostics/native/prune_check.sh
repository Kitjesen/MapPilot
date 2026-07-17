#!/usr/bin/env bash
set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"

ERASOR2_DIR="${LINGTU_ERASOR2_DIR:-${REPO_ROOT}/third_party/research_nav/ERASOR2}"
BUILD_DIR="${LINGTU_PRUNE_BUILD_DIR:-${LINGTU_MAP_CLEANING_BUILD_DIR:-${REPO_ROOT}/build/prune}}"
RUN_FETCH=0
BUILD_ERASOR2=0
RUN_ERASOR2=0
STATUS="pass"
ITEMS=()

usage() {
  cat <<'EOF'
Usage:
  bash scripts/diagnostics/native/prune_check.sh [options]

Options:
  --fetch          Fetch pinned ERASOR2 source into third_party/research_nav/ERASOR2.
  --build         Build the optional GPLv3 ERASOR2 backend.
  --run           Run upstream ERASOR2 after staging a tiny smoke map. Implies --build.
  --help          Show this message.

Default behavior checks sunrise/robot readiness, builds the default LingTu map
cleaner, and runs native smoke tests. It reports missing optional ERASOR2
checkout/dependencies as blocked, not as a default failure.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --fetch)
      RUN_FETCH=1
      shift
      ;;
    --build)
      BUILD_ERASOR2=1
      shift
      ;;
    --run)
      BUILD_ERASOR2=1
      RUN_ERASOR2=1
      shift
      ;;
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

pkg_or_config() {
  local pkg="$1"
  local cmake_file="$2"
  if have pkg-config && pkg-config --exists "${pkg}" 2>/dev/null; then
    pkg-config --modversion "${pkg}" 2>/dev/null
    return 0
  fi
  find /usr/lib /usr/share /usr/local/lib /usr/local/share "${REPO_ROOT}/third_party/install" \
    -name "${cmake_file}" -print -quit 2>/dev/null | grep -q .
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
}

cd "${REPO_ROOT}" || exit 1

for cmd in git cmake c++; do
  if have "${cmd}"; then
    add_item "cmd:${cmd}" "pass" "$("${cmd}" --version 2>/dev/null | head -1)"
  else
    add_item "cmd:${cmd}" "fail" "missing command"
  fi
done

if [[ ${RUN_FETCH} -eq 1 ]]; then
  run_step "fetch_erasor2" bash scripts/build/fetch_erasor2.sh
fi

if [[ -f "${ERASOR2_DIR}/CMakeLists.txt" ]]; then
  add_item "erasor2_checkout" "pass" "${ERASOR2_DIR}"
else
  add_item "erasor2_checkout" "blocked" "missing ${ERASOR2_DIR}; run scripts/build/fetch_erasor2.sh"
fi

for spec in "eigen3:Eigen3Config.cmake" "pcl_common:PCLConfig.cmake" "opencv4:OpenCVConfig.cmake" "yaml-cpp:yaml-cpp-config.cmake"; do
  pkg="${spec%%:*}"
  cmake_file="${spec#*:}"
  if detail="$(pkg_or_config "${pkg}" "${cmake_file}" 2>/dev/null)"; then
    add_item "dep:${pkg}" "pass" "${detail:-found ${cmake_file}}"
  else
    add_item "dep:${pkg}" "blocked" "missing ${pkg} or ${cmake_file}"
  fi
done

run_step "build_default_prune" bash scripts/build/build_prune.sh

SMOKE_ROOT="${TMPDIR:-/tmp}/lingtu_prune_check_map"
SMOKE_STAGE="${TMPDIR:-/tmp}/lingtu_prune_check_stage"
write_smoke_map "${SMOKE_ROOT}"

if [[ -x "${BUILD_DIR}/prune" ]]; then
  run_step "prune_smoke" "${BUILD_DIR}/prune" --map-dir "${SMOKE_ROOT}" --overwrite
else
  add_item "prune_smoke" "fail" "missing ${BUILD_DIR}/prune"
fi

rm -rf "${SMOKE_STAGE}"
if [[ -x "${BUILD_DIR}/erasor2_stage" ]]; then
  run_step "erasor2_stage_smoke" "${BUILD_DIR}/erasor2_stage" --map-dir "${SMOKE_ROOT}" --out "${SMOKE_STAGE}" --overwrite
else
  add_item "erasor2_stage_smoke" "fail" "missing ${BUILD_DIR}/erasor2_stage"
fi

if [[ ${BUILD_ERASOR2} -eq 1 ]]; then
  output="$(LINGTU_PRUNE_ERASOR2=ON bash scripts/build/build_prune.sh 2>&1)"
  rc=$?
  if [[ ${rc} -eq 0 ]]; then
    add_item "build_erasor2_backend" "pass" "${output}"
  else
    add_item "build_erasor2_backend" "fail" "rc=${rc}; ${output}"
  fi
  if [[ -x "${BUILD_DIR}/erasor2_clean" ]]; then
    run_step "erasor2_clean_skip_run" "${BUILD_DIR}/erasor2_clean" --map-dir "${SMOKE_ROOT}" --stage-dir "${SMOKE_STAGE}" --overwrite --skip-run
  else
    add_item "erasor2_clean_skip_run" "fail" "missing ${BUILD_DIR}/erasor2_clean"
  fi
else
  add_item "build_erasor2_backend" "blocked" "not requested; pass --build after checkout/deps are present"
fi

if [[ ${RUN_ERASOR2} -eq 1 ]]; then
  if [[ -x "${BUILD_DIR}/erasor2_clean" ]]; then
    run_step "erasor2_full_smoke" "${BUILD_DIR}/erasor2_clean" --map-dir "${SMOKE_ROOT}" --stage-dir "${SMOKE_STAGE}" --overwrite
  else
    add_item "erasor2_full_smoke" "fail" "missing ${BUILD_DIR}/erasor2_clean"
  fi
fi

printf '{\n'
printf '  "schema": "lingtu.prune_check.v1",\n'
printf '  "status": "%s",\n' "$(json_escape "${STATUS}")"
printf '  "repo_root": "%s",\n' "$(json_escape "${REPO_ROOT}")"
printf '  "erasor2_dir": "%s",\n' "$(json_escape "${ERASOR2_DIR}")"
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
if [[ "${STATUS}" == "blocked" && ( ${BUILD_ERASOR2} -eq 1 || ${RUN_ERASOR2} -eq 1 ) ]]; then
  exit 3
fi
exit 0
