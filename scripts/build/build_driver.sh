#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
BUILD_DIR="${LINGTU_DRIVER_BUILD_DIR:-${ROOT}/build/driver}"
BUILD_TYPE="${CMAKE_BUILD_TYPE:-Release}"
JOBS="${LINGTU_BUILD_JOBS:-$(nproc 2>/dev/null || echo 4)}"
RUN_TESTS="${LINGTU_DRIVER_RUN_TESTS:-1}"

if [ -n "${LINGTU_CYCLONEDDS_PREFIX:-}" ]; then
  export CMAKE_PREFIX_PATH="${LINGTU_CYCLONEDDS_PREFIX}${CMAKE_PREFIX_PATH:+:${CMAKE_PREFIX_PATH}}"
  if [ -d "${LINGTU_CYCLONEDDS_PREFIX}/bin" ]; then
    export PATH="${LINGTU_CYCLONEDDS_PREFIX}/bin:${PATH}"
  fi
  multiarch="$(cc -dumpmachine 2>/dev/null || true)"
  for lib_dir in "${LINGTU_CYCLONEDDS_PREFIX}/lib" "${LINGTU_CYCLONEDDS_PREFIX}/lib/${multiarch}"; do
    if [ -d "${lib_dir}" ]; then
      export LD_LIBRARY_PATH="${lib_dir}${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
    fi
  done
fi

cmake -S "${ROOT}/src/drivers/real/thunder/native" -B "${BUILD_DIR}" \
  -DCMAKE_BUILD_TYPE="${BUILD_TYPE}" \
  -DLINGTU_DRIVER_BUILD_RUNTIME=ON \
  -DBUILD_TESTING=ON
cmake --build "${BUILD_DIR}" --parallel "${JOBS}"

case "${RUN_TESTS,,}" in
  1|on|true|yes)
    test_catalog="$(ctest --test-dir "${BUILD_DIR}" -N)"
    for required_test in test_driver_core test_driver_io; do
      if ! grep -Fq "${required_test}" <<<"${test_catalog}"; then
        echo "ERROR: required driver test is missing from CTest: ${required_test}" >&2
        exit 1
      fi
    done
    ctest --test-dir "${BUILD_DIR}" --output-on-failure
    ;;
esac

BIN="${BUILD_DIR}/lingtu_driver"
if [[ ! -x "${BIN}" ]]; then
  echo "ERROR: build finished but native Thunder driver is missing: ${BIN}" >&2
  exit 1
fi

echo "${BIN}"
