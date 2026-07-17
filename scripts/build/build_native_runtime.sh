#!/usr/bin/env bash
# Build the no-ROS native runtime command helpers.
#
# This produces lt_native, lt_pgo, lt_hba, and lt_loop_verify. On robots, pass
# --install-user-bin so field diagnostics can find the commands through
# ~/.local/bin, which is already in the service PATH.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
BUILD_DIR="${LINGTU_NATIVE_RUNTIME_BUILD_DIR:-${REPO_ROOT}/build/native-runtime}"
BUILD_TYPE="${CMAKE_BUILD_TYPE:-Release}"
PYTHON_BIN="${PYTHON:-python3}"
CMAKE_BIN="${CMAKE:-cmake}"
INSTALL_USER_BIN=0

if [[ -d "${HOME}/.cargo/bin" ]]; then
  export PATH="${HOME}/.cargo/bin:${PATH}"
fi

for arg in "$@"; do
  case "${arg}" in
    --install-user-bin)
      INSTALL_USER_BIN=1
      ;;
    -h|--help)
      cat <<'EOF'
Usage: scripts/build/build_native_runtime.sh [--install-user-bin]

Builds:
  build/native-runtime/lt_native
  build/native-runtime/lt_pgo
  build/native-runtime/lt_hba
  build/native-runtime/lt_loop_verify

Options:
  --install-user-bin  Symlink the four commands into ~/.local/bin.
EOF
      exit 0
      ;;
    *)
      echo "Unknown argument: ${arg}" >&2
      exit 2
      ;;
  esac
done

cd "${REPO_ROOT}"

"${PYTHON_BIN}" scripts/build/build_rust_kernels.py \
  --target pose_graph_opt \
  --release

POSE_GRAPH_DIR="${REPO_ROOT}/src/kernels/slam/pose_graph_opt/target/release"
if [[ "${OS:-}" == "Windows_NT" ]]; then
  POSE_GRAPH_LIB="${POSE_GRAPH_DIR}/lingtu_pose_graph_opt.lib"
else
  POSE_GRAPH_LIB="${POSE_GRAPH_DIR}/liblingtu_pose_graph_opt.a"
fi

if [[ ! -f "${POSE_GRAPH_LIB}" ]]; then
  echo "Missing pose_graph_opt static library: ${POSE_GRAPH_LIB}" >&2
  exit 1
fi

"${CMAKE_BIN}" \
  -S src/native/runtime \
  -B "${BUILD_DIR}" \
  -DCMAKE_BUILD_TYPE="${BUILD_TYPE}" \
  -DLINGTU_POSE_GRAPH_OPT_LIB="${POSE_GRAPH_LIB}"

"${CMAKE_BIN}" --build "${BUILD_DIR}" --config "${BUILD_TYPE}" -j "${LINGTU_BUILD_JOBS:-2}"

if [[ "${INSTALL_USER_BIN}" == "1" ]]; then
  mkdir -p "${HOME}/.local/bin"
  ln -sfn "${BUILD_DIR}/lt_native" "${HOME}/.local/bin/lt_native"
  ln -sfn "${BUILD_DIR}/lt_pgo" "${HOME}/.local/bin/lt_pgo"
  ln -sfn "${BUILD_DIR}/lt_hba" "${HOME}/.local/bin/lt_hba"
  ln -sfn "${BUILD_DIR}/lt_loop_verify" "${HOME}/.local/bin/lt_loop_verify"
  echo "Installed native runtime commands into ${HOME}/.local/bin"
fi

echo "Native runtime build complete: ${BUILD_DIR}"
