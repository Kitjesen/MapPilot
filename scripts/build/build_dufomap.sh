#!/usr/bin/env bash
# build_dufomap.sh — 在 aarch64 Ubuntu 22.04 上一键构建 DUFOMap binary.
#
# 为什么这么复杂?
#   1. DUFOMap PyPI 没 aarch64 wheel,必须源码编
#   2. UFOMap 三个 header hardcode #include <immintrin.h> 没 BMI2 守卫,
#      arm64 上编不过,要 patch
#   3. S100P apt 代理 (192.168.66.6:7897) 经常断,liblzf-dev 装不下来,
#      要 fallback 到 wget deb 直下
#
# 用法:
#   bash scripts/build/build_dufomap.sh          # 幂等, 已有 binary 会跳过
#   bash scripts/build/build_dufomap.sh --force  # 强制重建
#
# 输出:
#   ~/src/dufomap/build/dufomap_run        可执行
set -euo pipefail

FORCE=0
for arg in "$@"; do
    case "$arg" in
        -f|--force) FORCE=1 ;;
        -h|--help)
            sed -n '2,/^set /p' "$0" | sed 's/^# //;s/^#//'
            exit 0 ;;
    esac
done

SRC_DIR="${DUFOMAP_SRC:-$HOME/src/dufomap}"
BUILD_DIR="$SRC_DIR/build"
BIN="$BUILD_DIR/dufomap_run"

if [[ $FORCE -eq 0 && -x "$BIN" ]]; then
    echo "[build_dufomap] $BIN already exists, skipping (use --force to rebuild)"
    exit 0
fi

arch=$(uname -m)
if [[ "$arch" != "aarch64" && "$arch" != "x86_64" ]]; then
    echo "[build_dufomap] WARN: untested arch $arch, proceeding anyway" >&2
fi

echo "[build_dufomap] 1/6 checking toolchain"
for cmd in cmake git g++ pkg-config wget; do
    command -v "$cmd" >/dev/null || { echo "missing: $cmd (apt install build-essential cmake git wget pkg-config)"; exit 1; }
done

echo "[build_dufomap] 2/6 installing apt deps (libtbb, liblz4, python3-dev)"
# 本地 apt 代理不稳,尝试安装但失败不致命 (可能已装过)
sudo apt-get install -y libtbb-dev liblz4-dev python3-dev 2>&1 \
    | grep -vE '^(Get:|Ign:|Hit:|Reading|Building|Selecting|Preparing|Unpacking|Setting|Processing)' \
    || echo "[build_dufomap] apt install returned non-zero (probably already installed)"

echo "[build_dufomap] 3/6 ensuring liblzf-dev"
if ! dpkg -l liblzf-dev >/dev/null 2>&1; then
    # apt 代理被挡时走 wget 直下 .deb
    if ! sudo apt-get install -y liblzf-dev 2>/dev/null; then
        echo "[build_dufomap]   apt failed (proxy?), falling back to direct wget"
        cd /tmp
        liblzf1_url="https://mirrors.tuna.tsinghua.edu.cn/ubuntu-ports/pool/universe/libl/liblzf/liblzf1_3.6-3_arm64.deb"
        liblzf_dev_url="https://mirrors.tuna.tsinghua.edu.cn/ubuntu-ports/pool/universe/libl/liblzf/liblzf-dev_3.6-3_arm64.deb"
        liblzf1_deb="liblzf1_3.6-3_arm64.deb"
        liblzf_dev_deb="liblzf-dev_3.6-3_arm64.deb"

        if [[ -z "${LIBLZF1_SHA256:-}" || -z "${LIBLZF_DEV_SHA256:-}" ]]; then
            if [[ "${DUFOMAP_SKIP_DEB_SHA256:-0}" != "1" ]]; then
                echo "[build_dufomap] ERROR: direct .deb fallback requires LIBLZF1_SHA256 and LIBLZF_DEV_SHA256" >&2
                echo "[build_dufomap] Set DUFOMAP_SKIP_DEB_SHA256=1 only for lab/offline recovery." >&2
                exit 1
            fi
            echo "[build_dufomap] WARN: skipping .deb checksum verification by explicit request" >&2
        fi

        wget -q --tries=3 --timeout=15 "$liblzf1_url" "$liblzf_dev_url"
        if [[ -n "${LIBLZF1_SHA256:-}" && -n "${LIBLZF_DEV_SHA256:-}" ]]; then
            echo "${LIBLZF1_SHA256}  ${liblzf1_deb}" | sha256sum -c -
            echo "${LIBLZF_DEV_SHA256}  ${liblzf_dev_deb}" | sha256sum -c -
        fi
        sudo dpkg -i "$liblzf1_deb" "$liblzf_dev_deb"
    fi
else
    echo "[build_dufomap]   liblzf-dev already installed"
fi

echo "[build_dufomap] 4/6 cloning repo to $SRC_DIR"
mkdir -p "$(dirname "$SRC_DIR")"
if [[ ! -d "$SRC_DIR/.git" ]]; then
    git clone --recursive https://github.com/KTH-RPL/dufomap.git "$SRC_DIR"
else
    echo "[build_dufomap]   repo exists, skipping clone"
fi

echo "[build_dufomap] 5/6 patching ufomap headers for aarch64 (BMI2 guard)"
# UFOMap hardcode #include <immintrin.h> (x86 intrinsics) 没有条件编译守卫,
# aarch64 上直接编不过。Patch 三个 header 加 #if defined(__BMI2__) 守卫。
# 幂等: grep 发现已 patch 就跳过。
cd "$SRC_DIR"
for f in ufomap/include/ufo/map/key.hpp \
         ufomap/include/ufo/map/code.hpp \
         ufomap/include/ufo/map/ray_caster/ray_caster.hpp; do
    if [[ ! -f "$f" ]]; then
        echo "[build_dufomap]   WARN: $f not found, skip patch" >&2
        continue
    fi
    if grep -q '^#include <immintrin.h>' "$f"; then
        sed -i 's|^#include <immintrin.h>|#if defined(__BMI2__)\n#include <immintrin.h>\n#endif|' "$f"
        echo "[build_dufomap]   patched $f"
    else
        echo "[build_dufomap]   $f already patched"
    fi
done

echo "[build_dufomap] 6/6 cmake + build (may take 3-5 min on aarch64)"
rm -rf "$BUILD_DIR"
cmake -B "$BUILD_DIR" -D CMAKE_BUILD_TYPE=Release -S "$SRC_DIR"
cmake --build "$BUILD_DIR" -j"$(nproc)"

if [[ -x "$BIN" ]]; then
    echo ""
    echo "[build_dufomap] SUCCESS: $BIN"
    ls -la "$BIN"
    echo ""
    echo "Smoke test:"
    "$BIN" 2>&1 | head -2 || true
else
    echo "[build_dufomap] FAIL: $BIN not produced" >&2
    exit 2
fi
