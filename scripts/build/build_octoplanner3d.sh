#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd "$script_dir/../.." && pwd)"

case "$(uname -s)" in
  MINGW*|MSYS*|CYGWIN*)
    cat >&2 <<'EOF'
OctoPlanner3D headless links the upstream Linux OctoMap/OctoMath .so libraries.
Build this wrapper on Linux/WSL/S100P. PCL is optional: without PCL the
executable supports .bt OctoMap input only; install PCL to enable .pcd conversion.
EOF
    exit 3
    ;;
esac

source_dir="${LINGTU_OCTOPLANNER3D_SOURCE_DIR:-$repo_root/src/nav/services/plan/global_planner/algorithm/OctoPlanner3D}"
build_dir="${LINGTU_OCTOPLANNER3D_BUILD_DIR:-$repo_root/build/octoplanner3d_headless}"
build_type="${CMAKE_BUILD_TYPE:-Release}"
build_python="${LINGTU_OCTOPLANNER3D_BUILD_PYTHON_BINDINGS:-0}"
require_pcl="${LINGTU_OCTOPLANNER3D_REQUIRE_PCL:-0}"
diagnose_only=0
jobs="${JOBS:-}"

usage() {
  cat <<'EOF'
Usage:
  bash scripts/build/build_octoplanner3d.sh [--python|--bindings] [--headless-only] [--converter-native|--require-pcl] [--diagnose]

Options:
  --python, --bindings    Also build the optional Python extension.
  --headless-only         Build only octoplanner3d_headless.
  --converter-native      Require the PCL-backed .pcd -> OctoMap converter path.
  --require-pcl           Alias for --converter-native.
  --diagnose              Print PCL/CMake/ldd diagnostics for the configured build dir and exit.

Environment:
  LINGTU_OCTOPLANNER3D_SOURCE_DIR   Default: src/nav/services/plan/global_planner/algorithm/OctoPlanner3D
  LINGTU_OCTOPLANNER3D_BUILD_DIR    Default: <repo>/build/octoplanner3d_headless
  LINGTU_OCTOPLANNER3D_REQUIRE_PCL  Set to 1 to require converter-native PCL linkage.
  PCL_DIR / CMAKE_PREFIX_PATH       Point CMake at the repo-local PCL install.
EOF
}

truthy() {
  case "${1,,}" in
    1|on|true|yes) return 0 ;;
    *) return 1 ;;
  esac
}

pcl_required() {
  truthy "$require_pcl"
}

candidate_exe() {
  local exe
  exe="$build_dir/octoplanner3d_headless"
  if [[ ! -x "$exe" && -x "$build_dir/Release/octoplanner3d_headless.exe" ]]; then
    exe="$build_dir/Release/octoplanner3d_headless.exe"
  elif [[ ! -x "$exe" && -x "$build_dir/Debug/octoplanner3d_headless.exe" ]]; then
    exe="$build_dir/Debug/octoplanner3d_headless.exe"
  fi
  printf '%s\n' "$exe"
}

print_dependency_diagnostics() {
  local cache exe
  cache="$build_dir/CMakeCache.txt"
  exe="$(candidate_exe)"

  cat <<EOF

OctoPlanner3D native dependency diagnostics:
  source_dir=$source_dir
  build_dir=$build_dir
  require_pcl=$require_pcl
  PCL_DIR=${PCL_DIR:-<unset>}
  CMAKE_PREFIX_PATH=${CMAKE_PREFIX_PATH:-<unset>}
  LD_LIBRARY_PATH=${LD_LIBRARY_PATH:-<unset>}
EOF

  if [[ -f "$cache" ]]; then
    echo "  CMake PCL cache entries:"
    grep -E '^(PCL_DIR|PCL_(COMMON|IO|OCTREE)_(INCLUDE_DIR|LIBRARY)):' "$cache" || true
  else
    echo "  CMake cache not found: $cache"
  fi

  if [[ -x "$exe" ]]; then
    echo "  ldd pcl/octomap entries for: $exe"
    if command -v ldd >/dev/null 2>&1; then
      ldd "$exe" | grep -E 'pcl|octomap' || echo "  no pcl/octomap entries matched"
    else
      echo "  ldd not available on this host"
    fi
  else
    echo "  executable not found yet: $exe"
  fi
}

require_pcl_cache() {
  local cache component missing=0
  pcl_required || return 0
  cache="$build_dir/CMakeCache.txt"

  if [[ ! -f "$source_dir/octomap/include/pcd2octomap_converter.h" || ! -f "$source_dir/octomap/src/pcd2octomap_converter.cpp" ]]; then
    echo "OctoPlanner3D converter-native requested, but pcd2octomap converter source/header is missing under: $source_dir" >&2
    exit 5
  fi

  if [[ ! -f "$cache" ]]; then
    echo "OctoPlanner3D converter-native requested, but CMake cache is missing: $cache" >&2
    exit 5
  fi

  for component in COMMON IO OCTREE; do
    component_line="$(grep -E "^PCL_${component}(_LIBRARY|_LIBRARY_RELEASE)?(:[^=]+)?=" "$cache" | grep -v NOTFOUND | head -1 || true)"
    if [[ -z "$component_line" ]]; then
      echo "Missing PCL ${component,,} library in CMake cache; check PCL_DIR/CMAKE_PREFIX_PATH." >&2
      missing=1
    fi
  done

  if [[ "$missing" != "0" ]]; then
    print_dependency_diagnostics >&2
    exit 5
  fi
}

for arg in "$@"; do
  case "$arg" in
    --python|--bindings)
      build_python=1
      ;;
    --headless-only)
      build_python=0
      ;;
    --converter-native|--require-pcl)
      require_pcl=1
      ;;
    --diagnose)
      diagnose_only=1
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown option: $arg" >&2
      usage >&2
      exit 2
      ;;
  esac
done

if [[ -z "$jobs" ]]; then
  if command -v nproc >/dev/null 2>&1; then
    jobs="$(nproc)"
  else
    jobs="2"
  fi
fi

if [[ "$diagnose_only" == "1" ]]; then
  print_dependency_diagnostics
  exit 0
fi

python_bindings_requested() {
  case "${build_python,,}" in
    1|on|true|yes) return 0 ;;
    *) return 1 ;;
  esac
}

if [[ ! -f "$source_dir/planner/include/global_planner.h" ]]; then
  cat >&2 <<EOF
Local OctoPlanner3D source not found at:
  $source_dir

Restore the source there, or set LINGTU_OCTOPLANNER3D_SOURCE_DIR=/path/to/OctoPlanner3D.
EOF
  exit 2
fi

if python_bindings_requested; then
  command -v python3 >/dev/null 2>&1 || {
    echo "python3 is required for OctoPlanner3D Python bindings" >&2
    exit 2
  }
  if ! python3 -c "import sysconfig; raise SystemExit(0 if sysconfig.get_path('include') else 1)" >/dev/null 2>&1; then
    echo "Python development headers are required for OctoPlanner3D Python bindings" >&2
    exit 2
  fi
fi

cmake_args=(
  -S "$repo_root/src/nav/services/plan/global_planner/algorithm/OctoPlanner3D/runtime"
  -B "$build_dir"
  -DOCTOPLANNER3D_SOURCE_DIR="$source_dir"
  -DOCTOPLANNER3D_BUILD_PYTHON_BINDINGS="$build_python"
  -DCMAKE_BUILD_TYPE="$build_type"
)
if [[ -n "${PCL_DIR:-}" ]]; then
  cmake_args+=("-DPCL_DIR=$PCL_DIR")
fi

cmake "${cmake_args[@]}"
require_pcl_cache

cmake --build "$build_dir" --target octoplanner3d_headless -j "$jobs"
if python_bindings_requested; then
  cmake --build "$build_dir" --target _native -j "$jobs"
fi

exe="$build_dir/octoplanner3d_headless"
if [[ ! -x "$exe" && -x "$build_dir/Release/octoplanner3d_headless.exe" ]]; then
  exe="$build_dir/Release/octoplanner3d_headless.exe"
elif [[ ! -x "$exe" && -x "$build_dir/Debug/octoplanner3d_headless.exe" ]]; then
  exe="$build_dir/Debug/octoplanner3d_headless.exe"
fi

native_module=""
if python_bindings_requested; then
  native_module="$(find "$build_dir" \( -name "_native*.so" -o -name "_native*.pyd" \) | head -1)"
  if [[ -z "$native_module" ]]; then
    cat >&2 <<EOF
OctoPlanner3D Python binding target was requested, but _native*.so was not found under:
  $build_dir
EOF
    exit 4
  fi
  package_dir="$repo_root/src/nav/services/plan/global_planner/algorithm"
  module_link="$package_dir/$(basename "$native_module")"
  rm -f "$module_link"
  ln -s "$native_module" "$module_link"
fi

cat <<EOF
Built OctoPlanner3D headless executable:
  $exe

Use it with LingTu:
  export LINGTU_OCTOPLANNER3D_EXECUTABLE="$exe"
  python lingtu.py nav --planner octoplanner3d
EOF

print_dependency_diagnostics

if [[ -n "$native_module" ]]; then
  cat <<EOF

Built OctoPlanner3D Python native module:
  $native_module

Use in-process planning with LingTu:
  export PYTHONPATH="$repo_root/src:\$PYTHONPATH"
  export LINGTU_OCTOPLANNER3D_NATIVE_MODULE=nav.services.plan.global_planner.algorithm._native
  python lingtu.py nav --planner octoplanner3d
EOF
fi
