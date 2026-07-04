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
require_pcl="${LINGTU_OCTOPLANNER3D_REQUIRE_PCL:-0}"
diagnose_only=0
jobs="${JOBS:-}"

usage() {
  cat <<'EOF'
Usage:
  bash scripts/build/build_octoplanner3d.sh [--require-pcl] [--diagnose]

Options:
  --require-pcl           Require the real PCL-backed octoplanner3d_pcd_to_octomap target.
  --diagnose              Print PCL/CMake/ldd diagnostics for the configured build dir and exit.

Environment:
  LINGTU_OCTOPLANNER3D_SOURCE_DIR   Default: src/nav/services/plan/global_planner/algorithm/OctoPlanner3D
  LINGTU_OCTOPLANNER3D_BUILD_DIR    Default: <repo>/build/octoplanner3d_headless
  LINGTU_OCTOPLANNER3D_REQUIRE_PCL  Set to 1 to require PCL converter linkage.
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

candidate_converter() {
  local exe
  exe="$build_dir/octoplanner3d_pcd_to_octomap"
  if [[ ! -x "$exe" && -x "$build_dir/Release/octoplanner3d_pcd_to_octomap.exe" ]]; then
    exe="$build_dir/Release/octoplanner3d_pcd_to_octomap.exe"
  elif [[ ! -x "$exe" && -x "$build_dir/Debug/octoplanner3d_pcd_to_octomap.exe" ]]; then
    exe="$build_dir/Debug/octoplanner3d_pcd_to_octomap.exe"
  fi
  printf '%s\n' "$exe"
}

print_dependency_diagnostics() {
  local cache exe converter
  cache="$build_dir/CMakeCache.txt"
  exe="$(candidate_exe)"
  converter="$(candidate_converter)"

  cat <<EOF

OctoPlanner3D C++ dependency diagnostics:
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
  if [[ -x "$converter" ]]; then
    echo "  converter: $converter"
  else
    echo "  converter not found yet: $converter"
  fi
}

require_pcl_cache() {
  local cache component missing=0
  pcl_required || return 0
  cache="$build_dir/CMakeCache.txt"

  if [[ ! -f "$source_dir/octomap/include/pcd2octomap_converter.h" || ! -f "$source_dir/octomap/src/pcd2octomap_converter.cpp" ]]; then
    echo "OctoPlanner3D PCL converter requested, but pcd2octomap converter source/header is missing under: $source_dir" >&2
    exit 5
  fi

  if [[ ! -f "$cache" ]]; then
    echo "OctoPlanner3D PCL converter requested, but CMake cache is missing: $cache" >&2
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
    --require-pcl)
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

if [[ ! -f "$source_dir/planner/include/global_planner.h" ]]; then
  cat >&2 <<EOF
Local OctoPlanner3D source not found at:
  $source_dir

Restore the source there, or set LINGTU_OCTOPLANNER3D_SOURCE_DIR=/path/to/OctoPlanner3D.
EOF
  exit 2
fi

cmake_args=(
  -S "$repo_root/src/nav/services/plan/global_planner/algorithm/OctoPlanner3D/runtime"
  -B "$build_dir"
  -DOCTOPLANNER3D_SOURCE_DIR="$source_dir"
  -DCMAKE_BUILD_TYPE="$build_type"
)
if pcl_required; then
  cmake_args+=("-DOCTOPLANNER3D_REQUIRE_PCL=ON")
else
  cmake_args+=("-DOCTOPLANNER3D_REQUIRE_PCL=OFF")
fi
if [[ -n "${PCL_DIR:-}" ]]; then
  cmake_args+=("-DPCL_DIR=$PCL_DIR")
fi

cmake "${cmake_args[@]}"
require_pcl_cache

cmake --build "$build_dir" --target octoplanner3d_headless -j "$jobs"
cmake --build "$build_dir" --target octoplanner3d_edit_octomap -j "$jobs"
if cmake --build "$build_dir" --target octoplanner3d_pcd_to_octomap -j "$jobs"; then
  :
elif pcl_required; then
  echo "OctoPlanner3D PCL converter requested, but CMake could not build octoplanner3d_pcd_to_octomap." >&2
  print_dependency_diagnostics >&2
  exit 5
else
  echo "OctoPlanner3D PCD converter target not generated; rerun with --require-pcl to make this fatal." >&2
fi
converter="$(candidate_converter)"
if pcl_required && [[ ! -x "$converter" ]]; then
  echo "OctoPlanner3D PCL converter requested, but executable is missing: $converter" >&2
  print_dependency_diagnostics >&2
  exit 5
fi
if [[ -x "$converter" ]]; then
  :
else
  if pcl_required; then
    exit 5
  fi
fi

exe="$build_dir/octoplanner3d_headless"
editor="$build_dir/octoplanner3d_edit_octomap"
if [[ ! -x "$exe" && -x "$build_dir/Release/octoplanner3d_headless.exe" ]]; then
  exe="$build_dir/Release/octoplanner3d_headless.exe"
elif [[ ! -x "$exe" && -x "$build_dir/Debug/octoplanner3d_headless.exe" ]]; then
  exe="$build_dir/Debug/octoplanner3d_headless.exe"
fi
if [[ ! -x "$editor" && -x "$build_dir/Release/octoplanner3d_edit_octomap.exe" ]]; then
  editor="$build_dir/Release/octoplanner3d_edit_octomap.exe"
elif [[ ! -x "$editor" && -x "$build_dir/Debug/octoplanner3d_edit_octomap.exe" ]]; then
  editor="$build_dir/Debug/octoplanner3d_edit_octomap.exe"
fi
converter="$(candidate_converter)"

cat <<EOF
Built OctoPlanner3D headless executable:
  $exe
Built OctoMap voxel editor:
  $editor
$(if [[ -x "$converter" ]]; then printf 'Built OctoMap PCD converter:\n  %s\n' "$converter"; else printf 'OctoMap PCD converter: not built (PCL path unavailable)\n'; fi)

Use it with LingTu:
  export LINGTU_OCTOPLANNER3D_EXECUTABLE="$exe"
  export LINGTU_OCTOMAP_EDITOR="$editor"
$(if [[ -x "$converter" ]]; then printf '  export LINGTU_MAP_ARTIFACT_CONVERTER="%s"\n' "$converter"; else printf '  # no LINGTU_MAP_ARTIFACT_CONVERTER export; build again with --require-pcl\n'; fi)
  python lingtu.py nav --planner octoplanner3d
EOF

print_dependency_diagnostics
