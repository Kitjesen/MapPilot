#!/usr/bin/env bash
# Start the native Livox SDK2 -> CycloneDDS publisher.
#
# The important bit is that this script generates a field config before SDK2
# starts. Never point production at the vendor sample JSON: the host IP is
# robot-specific and must match the active LiDAR NIC.

set -euo pipefail

source /opt/lingtu/config/thunder-runtime-env.sh

: "${LINGTU_LIVOX_BIN:=/opt/lingtu/current/build/livox_sdk2_stream/livox_sdk2_stream}"
: "${LINGTU_LIVOX_CONFIG_DIR:=/opt/lingtu/config/livox}"
: "${LINGTU_LIVOX_NET_IFACE:=eth1}"
: "${LINGTU_LIVOX_LIDAR_FRAME:=lidar_link}"
: "${LINGTU_LIVOX_IMU_FRAME:=imu_link}"
: "${LINGTU_LIVOX_SCAN_HZ:=10}"
: "${LINGTU_LIVOX_IMU_HZ:=0}"
: "${LINGTU_DDS_DOMAIN_ID:=0}"
: "${LINGTU_CYCLONEDDS_PREFIX:=}"
: "${LINGTU_PYTHON:=python3}"

if [ -n "${LINGTU_CYCLONEDDS_PREFIX}" ] && [ -d "${LINGTU_CYCLONEDDS_PREFIX}/lib" ]; then
    export LD_LIBRARY_PATH="${LINGTU_CYCLONEDDS_PREFIX}/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
fi

if [ ! -x "${LINGTU_LIVOX_BIN}" ]; then
    echo "ERROR: native Livox DDS publisher is missing or not executable: ${LINGTU_LIVOX_BIN}" >&2
    echo "Build it with: LINGTU_LIVOX_SDK2_STREAM_BUILD_DDS=ON bash scripts/build/build_livox_sdk2_stream.sh" >&2
    exit 2
fi

if [ -z "${LINGTU_LIVOX_HOST_IP:-}" ] && [ -n "${LINGTU_LIVOX_NET_IFACE}" ]; then
    detected_ip="$(
        "${LINGTU_PYTHON}" - <<'PY'
import os
import subprocess

from runtime.config import load_config
from runtime.utils.livox_config import select_livox_host_ip

iface = os.environ.get("LINGTU_LIVOX_NET_IFACE", "").strip()
addresses = []
if iface:
    result = subprocess.run(
        ["ip", "-4", "-o", "addr", "show", "dev", iface],
        check=False,
        stdout=subprocess.PIPE,
        stderr=subprocess.DEVNULL,
        text=True,
    )
    if result.returncode == 0:
        for line in result.stdout.splitlines():
            parts = line.split()
            if len(parts) >= 4:
                addresses.append(parts[3])

host_ip = select_livox_host_ip(
    load_config(),
    addresses,
    lidar_ip=os.environ.get("LINGTU_LIVOX_LIDAR_IP") or None,
)
if host_ip:
    print(host_ip)
PY
    )"
    if [ -n "${detected_ip}" ]; then
        export LINGTU_LIVOX_HOST_IP="${detected_ip}"
    fi
fi

if [ -z "${LINGTU_LIVOX_CONFIG:-}" ]; then
    mkdir -p "${LINGTU_LIVOX_CONFIG_DIR}"
    LINGTU_LIVOX_CONFIG="$(
        "${LINGTU_PYTHON}" - <<'PY'
import os

from runtime.config import load_config
from runtime.utils.livox_config import ensure_mid360_config_file

print(ensure_mid360_config_file(load_config(), os.environ["LINGTU_LIVOX_CONFIG_DIR"]))
PY
    )"
    export LINGTU_LIVOX_CONFIG
fi

if [ ! -f "${LINGTU_LIVOX_CONFIG}" ]; then
    echo "ERROR: generated Livox config is missing: ${LINGTU_LIVOX_CONFIG}" >&2
    exit 2
fi

echo "LingTu Livox DDS config: ${LINGTU_LIVOX_CONFIG}"
echo "LingTu Livox host IP: ${LINGTU_LIVOX_HOST_IP:-from robot_config.yaml}"
echo "LingTu Livox scan Hz: ${LINGTU_LIVOX_SCAN_HZ}"
echo "LingTu Livox IMU Hz: ${LINGTU_LIVOX_IMU_HZ:-0} (0 means SDK rate)"

args=(
    --dds \
    --domain-id "${LINGTU_DDS_DOMAIN_ID}" \
    --publish-freq "${LINGTU_LIVOX_SCAN_HZ}" \
    --lidar-frame "${LINGTU_LIVOX_LIDAR_FRAME}" \
    --imu-frame "${LINGTU_LIVOX_IMU_FRAME}"
)

if [ -n "${LINGTU_LIVOX_IMU_HZ}" ] && [ "${LINGTU_LIVOX_IMU_HZ}" != "0" ]; then
    args+=(--imu-publish-freq "${LINGTU_LIVOX_IMU_HZ}")
fi

exec "${LINGTU_LIVOX_BIN}" "${args[@]}" "${LINGTU_LIVOX_CONFIG}"
