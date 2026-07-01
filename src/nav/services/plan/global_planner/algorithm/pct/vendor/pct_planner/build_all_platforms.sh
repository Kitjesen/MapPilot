#!/usr/bin/env bash
# Build portable PCT Rust GPMP optimizer artifacts for x86_64 and aarch64.
#
# Usage:
#   bash build_all_platforms.sh          # build both
#   bash build_all_platforms.sh x86_64   # x86_64 only
#   bash build_all_platforms.sh aarch64  # aarch64 only
#
# Output:
#   src/nav/services/plan/global_planner/algorithm/pct/runtime/rust/x86_64/gpmp_optimize
#   src/nav/services/plan/global_planner/algorithm/pct/runtime/rust/x86_64/liblingtu_gpmp_trajectory_optimizer.so
#   src/nav/services/plan/global_planner/algorithm/pct/runtime/rust/aarch64/gpmp_optimize
#   src/nav/services/plan/global_planner/algorithm/pct/runtime/rust/aarch64/liblingtu_gpmp_trajectory_optimizer.so

set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../../../.." && pwd)"
cd "${REPO_ROOT}"

build_arch() {
    local arch=$1
    local platform=""
    local tag="pct-rust-gpmp-build-${arch}"
    local outdir="${REPO_ROOT}/src/nav/services/plan/global_planner/algorithm/pct/runtime/rust/${arch}"

    case $arch in
        x86_64)  platform="linux/amd64" ;;
        aarch64) platform="linux/arm64" ;;
        *) echo "Unknown arch: $arch"; exit 1 ;;
    esac

    echo "=== Building PCT Rust GPMP optimizer for ${arch} (${platform}) ==="
    mkdir -p "$outdir"

    docker buildx build \
        --platform "$platform" \
        -f src/nav/services/plan/global_planner/algorithm/pct/vendor/pct_planner/Dockerfile.build \
        -t "$tag" \
        --load \
        .

    docker run --rm -v "${outdir}:/output" "$tag"
    echo "=== ${arch} done ==="
    ls -la "${outdir}/"
    echo ""
}

TARGET=${1:-all}

case $TARGET in
    x86_64)  build_arch x86_64 ;;
    aarch64) build_arch aarch64 ;;
    all)
        build_arch x86_64
        build_arch aarch64
        ;;
    *) echo "Usage: $0 [x86_64|aarch64|all]"; exit 1 ;;
esac

echo "=== All builds complete ==="
