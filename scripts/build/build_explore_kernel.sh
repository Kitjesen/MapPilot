#!/usr/bin/env bash
# build_explore_kernel.sh - compile lingtu_explore_kernel.so (nanobind Python extension)
#
# This script builds the lingtu_explore_kernel extension from src/explore/cpp/.
# The resulting .so is placed in src/explore/cpp/build_nb/ and copied into src/
# so Python can import it directly from release packages.
#
# Usage:
#   bash scripts/build/build_explore_kernel.sh          # build + install
#   bash scripts/build/build_explore_kernel.sh --clean  # clean build dir first
#
# Requirements:
#   - cmake >= 3.16
#   - C++17 compiler (gcc/clang)
#   - Python dev headers (python3-dev)
#   - nanobind: pip install nanobind
#
# After building, add to ~/.bashrc (or lingtu does it automatically):
#   export PYTHONPATH=~/data/inovxio/lingtu/src/explore/cpp/build_nb:$PYTHONPATH

set -euo pipefail

find_repo_root() {
    local dir="$1"
    while [[ "$dir" != "/" ]]; do
        if [[ -f "$dir/pyproject.toml" && -f "$dir/AGENTS.md" ]]; then
            printf '%s\n' "$dir"
            return 0
        fi
        dir="$(dirname "$dir")"
    done
    return 1
}

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(find_repo_root "$SCRIPT_DIR")"
EXPLORE_CPP_DIR="$REPO_ROOT/src/explore/cpp"
BUILD_DIR="$EXPLORE_CPP_DIR/build_nb"
INSTALL_LINK="$REPO_ROOT/src"

# Colors
_G="\033[0;32m"; _Y="\033[1;33m"; _R="\033[0;31m"; _N="\033[0m"; _B="\033[1m"
ok()   { echo -e "  ${_G}[OK]${_N} $*"; }
info() { echo -e "  ${_B}[INFO]${_N} $*"; }
warn() { echo -e "  ${_Y}!${_N} $*"; }
fail() { echo -e "  ${_R}[FAIL]${_N} $*"; exit 1; }

echo ""
echo -e "  ${_B}LingTu - build native exploration kernel${_N}"
echo "  ----------------------------------------"

# Clean
if [[ "${1:-}" == "--clean" ]]; then
    info "Cleaning build directory..."
    rm -rf "$BUILD_DIR"
    ok "Cleaned"
fi

# Check dependencies
info "Checking dependencies..."

command -v cmake >/dev/null 2>&1 || fail "cmake not found. Install: sudo apt install cmake"
command -v python3 >/dev/null 2>&1 || fail "python3 not found"

PYTHON=$(command -v python3)
PY_VER=$($PYTHON --version 2>&1)
info "Python: $PY_VER"

PY_INCLUDE=$("$PYTHON" - <<'PY'
import sysconfig
print(sysconfig.get_path("include") or "")
PY
)
if [[ -z "$PY_INCLUDE" || ! -f "$PY_INCLUDE/Python.h" ]]; then
    fail "Python dev headers missing. Install: sudo apt install python3-dev"
fi

if "$PYTHON" -c "import nanobind" >/dev/null 2>&1; then
    NB_DIR=$("$PYTHON" -c "import nanobind; print(nanobind.cmake_dir())")
    ok "nanobind found at: $NB_DIR"
else
    info "nanobind Python package not found; CMake will fetch pinned nanobind v2.12.0"
fi

# --- IDL Code Generation ---
# Generate with the CycloneDDS toolchain whenever it is installed. Official
# idlc builds are available on Linux and Windows; the host OS is not a contract.
MESSAGE_IDL="${REPO_ROOT}/src/message/idl/messages.idl"
GEN_DIR="${BUILD_DIR}/dds_generated"

if command -v idlc &>/dev/null; then
    info "Generating C bindings from the LingTu message IDL..."
    mkdir -p "$GEN_DIR"
    info "  idlc -l c $(basename "$MESSAGE_IDL") -> $GEN_DIR"
    (cd "$GEN_DIR" && idlc -l c "$MESSAGE_IDL") || warn "idlc failed; DDS transport will be unavailable"
    ok "IDL generation complete"
else
    warn "idlc not found; skipping IDL code generation (DDS types must be pre-generated)"
fi

# Configure
info "Configuring CMake (standalone mode, no ROS2)..."
mkdir -p "$BUILD_DIR"

# Detect CycloneDDS (optional; enables native DDS transport in TARE).
DDS_CMAKE_ARGS=""
if pkg-config --exists CycloneDDS 2>/dev/null || \
   [ -n "${CycloneDDS_DIR:-}" ] || \
   dpkg -s cyclonedds-dev >/dev/null 2>&1; then
    info "CycloneDDS detected: enabling DDS transport"
    DDS_CMAKE_ARGS="-DLINGTU_EXPLORE_CPP_WITH_DDS=ON"
else
    warn "CycloneDDS not found: DDS transport disabled (in-process mode only)"
fi

# Create a minimal project that includes the real source.
cat > "$BUILD_DIR/CMakeLists.txt" << 'EOF'
cmake_minimum_required(VERSION 3.16)
project(explore_kernel_binding LANGUAGES C CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Python COMPONENTS Interpreter Development REQUIRED)

execute_process(
  COMMAND "${Python_EXECUTABLE}" -c "import nanobind; print(nanobind.cmake_dir())"
  OUTPUT_VARIABLE NB_DIR OUTPUT_STRIP_TRAILING_WHITESPACE
  RESULT_VARIABLE NB_RET)
if(NB_RET EQUAL 0)
  list(APPEND CMAKE_PREFIX_PATH "${NB_DIR}")
  find_package(nanobind CONFIG REQUIRED)
else()
  include(FetchContent)
  FetchContent_Declare(nanobind
    GIT_REPOSITORY https://github.com/wjakob/nanobind.git
    GIT_TAG        v2.12.0)
  FetchContent_MakeAvailable(nanobind)
endif()

# Source is in the parent directory (src/explore/cpp/).
set(EXPLORE_CPP_SRC "${CMAKE_CURRENT_SOURCE_DIR}/..")
# Repo root: injected by the outer build script as an absolute path.
set(REPO_ROOT "__REPO_ROOT_PLACEHOLDER__")

# ── idlc-generated C type bindings (optional) ──────────────────────────
# Compile the shared LingTu message types when generated.
set(LINGTU_DDS_GEN_DIR "${CMAKE_CURRENT_SOURCE_DIR}/dds_generated")
set(LINGTU_DDS_GEN_SOURCES "")
if(EXISTS "${LINGTU_DDS_GEN_DIR}/messages.c")
  set(LINGTU_DDS_GEN_SOURCES "${LINGTU_DDS_GEN_DIR}/messages.c")
  message(STATUS "idlc generated source: messages.c")
endif()

set(LINGTU_EXPLORE_BINDING_SOURCES
  "${EXPLORE_CPP_SRC}/tare_policy.cpp"
  "${EXPLORE_CPP_SRC}/bindings/bindings.cpp"
  "${EXPLORE_CPP_SRC}/bindings/bind_types.cpp"
  "${EXPLORE_CPP_SRC}/bindings/bind_tare.cpp"
  ${LINGTU_DDS_GEN_SOURCES})
nanobind_add_module(lingtu_explore_kernel ${LINGTU_EXPLORE_BINDING_SOURCES})
target_include_directories(lingtu_explore_kernel PRIVATE
  "${EXPLORE_CPP_SRC}"
  "${REPO_ROOT}/src"
  "${LINGTU_DDS_GEN_DIR}")

# Optional CycloneDDS transport
option(LINGTU_EXPLORE_CPP_WITH_DDS
  "Enable native CycloneDDS transport for TARE exploration" OFF)
if(LINGTU_EXPLORE_CPP_WITH_DDS)
  find_package(CycloneDDS QUIET)
  if(CycloneDDS_FOUND)
    message(STATUS "CycloneDDS found: enabling DDS transport")
    target_compile_definitions(lingtu_explore_kernel PRIVATE LINGTU_EXPLORE_HAS_DDS)
    target_link_libraries(lingtu_explore_kernel PRIVATE CycloneDDS::ddsc)
  else()
    message(WARNING "CycloneDDS not found; DDS transport disabled")
  endif()
endif()
EOF

# Inject the actual REPO_ROOT path from the shell script into the CMake file.
sed -i "s|__REPO_ROOT_PLACEHOLDER__|${REPO_ROOT}|g" "$BUILD_DIR/CMakeLists.txt"

cmake -B "$BUILD_DIR" -S "$BUILD_DIR" \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
    $DDS_CMAKE_ARGS \
    2>&1 | grep -E "^(--|\s*(CMake|Error|Warning))" || true

ok "CMake configured"

# Build
NPROC=$(nproc 2>/dev/null || echo 4)
info "Building with $NPROC cores..."
cmake --build "$BUILD_DIR" -j"$NPROC"

# Find the .so
SO_FILE=$(find "$BUILD_DIR" -name "lingtu_explore_kernel*.so" | head -1)
if [[ -z "$SO_FILE" ]]; then
    fail "lingtu_explore_kernel*.so not found after build. Check cmake output above."
fi
ok "Built: $SO_FILE"

# Install a real runtime artifact into src/.
SO_NAME=$(basename "$SO_FILE")
LINK_TARGET="$INSTALL_LINK/$SO_NAME"

find "$INSTALL_LINK" -maxdepth 1 \( -type f -o -type l \) -name "lingtu_explore_kernel*.so" -exec rm -f {} +
cp -f "$SO_FILE" "$LINK_TARGET"
ok "Installed: $LINK_TARGET"

# Verify import
info "Verifying import..."
if PYTHONPATH="$INSTALL_LINK:${PYTHONPATH:-}" $PYTHON -c "
import lingtu_explore_kernel
print('  lingtu_explore_kernel symbol check:')
print('  TarePolicy:      ', lingtu_explore_kernel.TarePolicy)
print('  TarePolicyConfig:', lingtu_explore_kernel.TarePolicyConfig)
print('  Grid2D:          ', lingtu_explore_kernel.Grid2D)
print('  HAS_DDS:         ', lingtu_explore_kernel.HAS_DDS)
if lingtu_explore_kernel.HAS_DDS:
    print('  TareDdsTransport:', lingtu_explore_kernel.TareDdsTransport)
"; then
    ok "lingtu_explore_kernel imported successfully"
else
    fail "Import failed - check build output"
fi

echo ""
echo -e "  ${_G}${_B}Done!${_N} lingtu_explore_kernel.so is ready."
echo ""
echo "  LingTu auto-detects the installed runtime in src/."
echo ""
