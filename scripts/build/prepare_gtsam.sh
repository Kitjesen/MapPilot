#!/usr/bin/env bash
# Build a relocatable CPU SDK for the current Linux distribution and architecture.
# Does not install system packages, overwrite /usr/local, or activate services.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
read -r GTSAM_VERSION GTSAM_REPOSITORY < <(python3 - "$ROOT/scripts/build/locks/gtsam.json" <<'PY'
import json
import sys
with open(sys.argv[1], encoding="utf-8") as source:
    lock = json.load(source)
print(lock["version"], lock["repository"])
PY
)
. /etc/os-release
ARCH="$(uname -m)"
case "$ARCH" in
  aarch64|x86_64) ;;
  *) echo "No LingTu GTSAM SDK build profile for architecture: $ARCH" >&2; exit 2 ;;
esac
PLATFORM="${ID}${VERSION_ID}-${ARCH}"
WORK="${LINGTU_GTSAM_BUILD_ROOT:-${ROOT}/build/deps/gtsam-${GTSAM_VERSION}-${PLATFORM}-build}"
PREFIX="${LINGTU_GTSAM_PREFIX:-${ROOT}/build/deps/gtsam-${GTSAM_VERSION}-${PLATFORM}}"
JOBS="${LINGTU_BUILD_JOBS:-2}"
mkdir -p "$WORK" "$PREFIX" "$ROOT/dist"
WORK="$(cd "$WORK" && pwd)"
PREFIX="$(cd "$PREFIX" && pwd)"
SOURCE="$WORK/src"
if [[ ! -f "$SOURCE/CMakeLists.txt" ]]; then
  curl -fL --retry 2 "${GTSAM_REPOSITORY}/archive/refs/tags/${GTSAM_VERSION}.tar.gz" -o "$WORK/source.tar.gz"
  mkdir -p "$SOURCE"
  tar -xzf "$WORK/source.tar.gz" --strip-components=1 -C "$SOURCE"
fi

cmake -S "$SOURCE" -B "$WORK/build" \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="$PREFIX" \
  -DCMAKE_INSTALL_LIBDIR=lib \
  '-DCMAKE_INSTALL_RPATH=$ORIGIN' \
  -DBUILD_SHARED_LIBS=ON \
  -DGTSAM_USE_SYSTEM_EIGEN=ON \
  -DGTSAM_WITH_TBB=OFF \
  -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF \
  -DGTSAM_BUILD_TESTS=OFF \
  -DGTSAM_INSTALL_CPPUNITLITE=OFF \
  -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
  -DCMAKE_DISABLE_FIND_PACKAGE_Ceres=ON \
  -DGTSAM_BUILD_UNSTABLE=OFF \
  -DGTSAM_BUILD_PYTHON=OFF \
  -DGTSAM_ENABLE_CUDA=OFF \
  -DGTSAM_USE_BOOST_FEATURES=OFF \
  -DGTSAM_ENABLE_BOOST_SERIALIZATION=OFF
python3 - "$WORK/build/gtsam/config.h" "$GTSAM_VERSION" <<'PY'
from pathlib import Path
import re
import sys
configured = re.search(r'#define GTSAM_VERSION_STRING "([^"]+)"', Path(sys.argv[1]).read_text())
if not configured or configured[1] != sys.argv[2]:
    raise SystemExit("GTSAM build directory contains another version; use a fresh build root")
PY
cmake --build "$WORK/build" --target gtsam --parallel "$JOBS"
cmake --install "$WORK/build"

mkdir -p "$PREFIX/share/lingtu"
mkdir -p "$PREFIX/share/lingtu/gtsam-licenses"
cp "$SOURCE/LICENSE" "$SOURCE/LICENSE.BSD" "$PREFIX/share/lingtu/gtsam-licenses/"
(
  cd "$SOURCE"
  find gtsam/3rdparty -type f \( -iname '*license*' -o -name 'COPYING*' \) \
    -exec cp --parents '{}' "$PREFIX/share/lingtu/gtsam-licenses/" \;
)
python3 - "$PREFIX/share/lingtu/gtsam-build.json" "$GTSAM_VERSION" "$ID" "$VERSION_ID" "$ARCH" "$WORK/build/CMakeCache.txt" <<'PY'
import json
from pathlib import Path
import sys

output, version, distro, release, arch, cache = sys.argv[1:]
keys = {"CMAKE_CXX_COMPILER", "CMAKE_CXX_COMPILER_VERSION", "Eigen3_DIR", "TBB_DIR",
        "GTSAM_USE_SYSTEM_EIGEN", "GTSAM_WITH_TBB", "GTSAM_POSE3_EXPMAP",
        "GTSAM_BUILD_WITH_MARCH_NATIVE", "GTSAM_USE_BOOST_FEATURES",
        "GTSAM_ENABLE_BOOST_SERIALIZATION"}
settings = {}
for line in Path(cache).read_text().splitlines():
    if ":" in line and "=" in line and not line.startswith(("#", "//")):
        key, value = line.split("=", 1)
        key = key.split(":", 1)[0]
        if key in keys:
            settings[key] = value
Path(output).write_text(json.dumps({"version": version, "os": distro,
    "os_version": release, "architecture": arch, "settings": settings}, indent=2) + "\n")
PY
ARCHIVE="$ROOT/dist/gtsam-${GTSAM_VERSION}-${PLATFORM}.tar.gz"
tar -czf "$ARCHIVE" -C "$PREFIX" .
printf 'SDK: %s\nArchive: %s\nBuild LingTu with CMAKE_PREFIX_PATH=%s\n' "$PREFIX" "$ARCHIVE" "$PREFIX"
