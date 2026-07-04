#!/usr/bin/env bash
# build_nav_kernel.sh - compile lingtu_nav_kernel.so (nanobind Python extension)
#
# This script builds the lingtu_nav_kernel extension from src/nav/kernel/.
# The resulting .so is placed in src/nav/kernel/build_nb/ and copied into src/
# so Python can import it directly from release packages.
#
# Usage:
#   bash scripts/build/build_nav_kernel.sh          # build + install
#   bash scripts/build/build_nav_kernel.sh --clean  # clean build dir first
#
# Requirements:
#   - cmake >= 3.14
#   - C++17 compiler (gcc/clang)
#   - Python dev headers (python3-dev)
#   - nanobind: pip install nanobind
#
# After building, add to ~/.bashrc (or lingtu does it automatically):
#   export PYTHONPATH=~/data/inovxio/lingtu/src/nav/kernel/build_nb:$PYTHONPATH

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
NAV_KERNEL_DIR="$REPO_ROOT/src/nav/kernel"
BUILD_DIR="$NAV_KERNEL_DIR/build_nb"
INSTALL_LINK="$REPO_ROOT/src"

# Colors
_G="\033[0;32m"; _Y="\033[1;33m"; _R="\033[0;31m"; _N="\033[0m"; _B="\033[1m"
ok()   { echo -e "  ${_G}[OK]${_N} $*"; }
info() { echo -e "  ${_B}[INFO]${_N} $*"; }
warn() { echo -e "  ${_Y}!${_N} $*"; }
fail() { echo -e "  ${_R}[FAIL]${_N} $*"; exit 1; }

echo ""
echo -e "  ${_B}LingTu - build native navigation kernel${_N}"
echo "  -------------------------------------"

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

# Configure
info "Configuring CMake (standalone mode, no ROS2)..."
mkdir -p "$BUILD_DIR"

# Create a minimal project that includes the real source.
cat > "$BUILD_DIR/CMakeLists.txt" << 'EOF'
cmake_minimum_required(VERSION 3.14)
project(nav_kernel_binding LANGUAGES CXX)
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

# Source is in the parent directory (src/nav/kernel/)
set(NAV_KERNEL_SRC "${CMAKE_CURRENT_SOURCE_DIR}/..")
set(LOCAL_PLANNER_CPP_SRC "${NAV_KERNEL_SRC}/../services/plan/local_planner/cpp")
add_subdirectory("${LOCAL_PLANNER_CPP_SRC}" "${CMAKE_CURRENT_BINARY_DIR}/local_planner_cpp")
set(NAV_KERNEL_BINDING_SOURCES
  "${NAV_KERNEL_SRC}/src/path_follower_core.cpp"
  "${NAV_KERNEL_SRC}/bindings/bindings.cpp"
  "${NAV_KERNEL_SRC}/bindings/bind_types.cpp"
  "${NAV_KERNEL_SRC}/bindings/bind_map_layers.cpp"
  "${NAV_KERNEL_SRC}/bindings/bind_path_follower.cpp"
  "${NAV_KERNEL_SRC}/bindings/bind_waypoint_helpers.cpp"
  "${NAV_KERNEL_SRC}/bindings/bind_local_planner.cpp"
  "${NAV_KERNEL_SRC}/bindings/bind_terrain.cpp")
nanobind_add_module(lingtu_nav_kernel ${NAV_KERNEL_BINDING_SOURCES})
target_include_directories(lingtu_nav_kernel PRIVATE
  "${NAV_KERNEL_SRC}/include"
  "${LOCAL_PLANNER_CPP_SRC}")
target_link_libraries(lingtu_nav_kernel PRIVATE local_planner_cpp)
EOF

cmake -B "$BUILD_DIR" -S "$BUILD_DIR" \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
    2>&1 | grep -E "^(--|\s*(CMake|Error|Warning))" || true

ok "CMake configured"

# Build
NPROC=$(nproc 2>/dev/null || echo 4)
info "Building with $NPROC cores..."
cmake --build "$BUILD_DIR" -j"$NPROC"

# Find the .so
SO_FILE=$(find "$BUILD_DIR" -name "lingtu_nav_kernel*.so" | head -1)
if [[ -z "$SO_FILE" ]]; then
    fail "lingtu_nav_kernel*.so not found after build. Check cmake output above."
fi
ok "Built: $SO_FILE"

# Install a real runtime artifact into src/.
SO_NAME=$(basename "$SO_FILE")
LINK_TARGET="$INSTALL_LINK/$SO_NAME"

find "$INSTALL_LINK" -maxdepth 1 \( -type f -o -type l \) -name "lingtu_nav_kernel*.so" -exec rm -f {} +
find "$INSTALL_LINK" -maxdepth 1 \( -type f -o -type l \) -name "_nav_kernel*.so" -exec rm -f {} +
cp -f "$SO_FILE" "$LINK_TARGET"
ok "Installed: $LINK_TARGET"

# Verify import
info "Verifying import..."
if PYTHONPATH="$INSTALL_LINK:${PYTHONPATH:-}" $PYTHON -c "
import lingtu_nav_kernel
print('  lingtu_nav_kernel version check:')
print('  TerrainAnalysisCore:', lingtu_nav_kernel.TerrainAnalysisCore)
print('  LocalPlanner:       ', lingtu_nav_kernel.LocalPlanner)
print('  compute_control:    ', lingtu_nav_kernel.compute_control)
"; then
    ok "lingtu_nav_kernel imported successfully"
else
    fail "Import failed - check build output"
fi

echo ""
echo -e "  ${_G}${_B}Done!${_N} lingtu_nav_kernel.so is ready."
echo ""
echo "  LingTu auto-detects the installed runtime in src/."
echo ""
