#!/usr/bin/env bash
# build_nav_core.sh - compile _nav_core.so (nanobind Python extension)
#
# This script builds the _nav_core nanobind extension from src/nav/core/.
# The resulting .so is placed in src/nav/core/build_nb/ and a symlink is
# created at src/ so Python can import it directly.
#
# Usage:
#   bash scripts/build/build_nav_core.sh          # build + install
#   bash scripts/build/build_nav_core.sh --clean  # clean build dir first
#
# Requirements:
#   - cmake >= 3.14
#   - C++17 compiler (gcc/clang)
#   - Python dev headers (python3-dev)
#   - nanobind: pip install nanobind
#
# After building, add to ~/.bashrc (or lingtu does it automatically):
#   export PYTHONPATH=~/data/inovxio/lingtu/src/nav/core/build_nb:$PYTHONPATH

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
NAV_CORE_DIR="$REPO_ROOT/src/nav/core"
BUILD_DIR="$NAV_CORE_DIR/build_nb"
INSTALL_LINK="$REPO_ROOT/src"

# Colors
_G="\033[0;32m"; _Y="\033[1;33m"; _R="\033[0;31m"; _N="\033[0m"; _B="\033[1m"
ok()   { echo -e "  ${_G}[OK]${_N} $*"; }
info() { echo -e "  ${_B}[INFO]${_N} $*"; }
warn() { echo -e "  ${_Y}!${_N} $*"; }
fail() { echo -e "  ${_R}[FAIL]${_N} $*"; exit 1; }

echo ""
echo -e "  ${_B}LingTu - build _nav_core.so${_N}"
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

nanobind_pip_user_flag() {
    "$PYTHON" - <<'PY'
import os
import site
import sys

in_virtualenv = (
    sys.prefix != getattr(sys, "base_prefix", sys.prefix)
    or hasattr(sys, "real_prefix")
)
in_conda = bool(os.environ.get("CONDA_PREFIX"))
user_site_enabled = bool(getattr(site, "ENABLE_USER_SITE", False))

if user_site_enabled and not in_virtualenv and not in_conda:
    print("--user")
PY
}

# Check Python dev headers
if ! $PYTHON -c "import sysconfig; sysconfig.get_path('include')" >/dev/null 2>&1; then
    fail "Python dev headers missing. Install: sudo apt install python3-dev"
fi

# Check nanobind
if ! "$PYTHON" -c "import nanobind" >/dev/null 2>&1; then
    warn "nanobind not found. Installing..."
    pip_install_args=(install)
    PIP_USER_FLAG="$(nanobind_pip_user_flag)"
    if [[ -n "$PIP_USER_FLAG" ]]; then
        pip_install_args+=("$PIP_USER_FLAG")
    fi
    pip_install_args+=(nanobind)
    "$PYTHON" -m pip "${pip_install_args[@]}" || fail "Failed to install nanobind"
fi
NB_DIR=$("$PYTHON" -c "import nanobind; print(nanobind.cmake_dir())")
ok "nanobind found at: $NB_DIR"

# Configure
info "Configuring CMake (standalone mode, no ROS2)..."
mkdir -p "$BUILD_DIR"

# Use the nanobind-only CMakeLists (avoids colcon/ament_cmake complications)
cp "$NAV_CORE_DIR/CMakeLists_nanobind_only.cmake" "$BUILD_DIR/CMakeLists.txt"

# Create a minimal project that includes the real source
cat > "$BUILD_DIR/CMakeLists.txt" << 'EOF'
cmake_minimum_required(VERSION 3.14)
project(nav_core_binding LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Python COMPONENTS Interpreter Development REQUIRED)

execute_process(
  COMMAND "${Python_EXECUTABLE}" -c "import nanobind; print(nanobind.cmake_dir())"
  OUTPUT_VARIABLE NB_DIR OUTPUT_STRIP_TRAILING_WHITESPACE
  RESULT_VARIABLE NB_RET)
if(NOT NB_RET EQUAL 0)
  message(FATAL_ERROR "nanobind not found. Install: pip install nanobind")
endif()
list(APPEND CMAKE_PREFIX_PATH "${NB_DIR}")
find_package(nanobind CONFIG REQUIRED)

# Source is in the parent directory (src/nav/core/)
set(NAV_CORE_SRC "${CMAKE_CURRENT_SOURCE_DIR}/..")
nanobind_add_module(_nav_core "${NAV_CORE_SRC}/bindings/py_nav_core.cpp")
target_include_directories(_nav_core PRIVATE "${NAV_CORE_SRC}/include")
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
SO_FILE=$(find "$BUILD_DIR" -name "_nav_core*.so" | head -1)
if [[ -z "$SO_FILE" ]]; then
    fail "_nav_core*.so not found after build. Check cmake output above."
fi
ok "Built: $SO_FILE"

# Install symlink into src/
SO_NAME=$(basename "$SO_FILE")
LINK_TARGET="$INSTALL_LINK/$SO_NAME"

# Remove old symlink/file if present
if [[ -L "$LINK_TARGET" || -f "$LINK_TARGET" ]]; then
    rm -f "$LINK_TARGET"
fi
ln -s "$SO_FILE" "$LINK_TARGET"
ok "Symlinked: $LINK_TARGET -> $SO_FILE"

# Verify import
info "Verifying import..."
if PYTHONPATH="$INSTALL_LINK:${PYTHONPATH:-}" $PYTHON -c "
import _nav_core
print('  _nav_core version check:')
print('  TerrainAnalysisCore:', _nav_core.TerrainAnalysisCore)
print('  LocalPlannerCore:   ', _nav_core.LocalPlannerCore)
print('  compute_control:    ', _nav_core.compute_control)
"; then
    ok "_nav_core imported successfully"
else
    fail "Import failed - check build output"
fi

echo ""
echo -e "  ${_G}${_B}Done!${_N} _nav_core.so is ready."
echo ""
echo "  To use permanently, add to ~/.bashrc:"
echo -e "    ${_B}export PYTHONPATH=$INSTALL_LINK:\$PYTHONPATH${_N}"
echo ""
echo "  Or just run lingtu - it auto-detects the .so location."
echo ""
