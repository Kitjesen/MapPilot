#!/usr/bin/env bash
set -euo pipefail
cmake -S src/drivers/real/camera/impl/realsense -B build/realsense_native \
  -DCMAKE_BUILD_TYPE=Release "$@"
cmake --build build/realsense_native --parallel 2
