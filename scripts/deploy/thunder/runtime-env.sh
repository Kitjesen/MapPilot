#!/usr/bin/env bash
# Field runtime environment.
#
# This file is intentionally free of ROS setup. It is the shared deployment
# environment for native endpoints and the optional Python application graph.

: "${LINGTU_REPO:=/opt/lingtu/current}"
: "${LINGTU_CONFIG_DIR:=/opt/lingtu/config}"
python_env="${LINGTU_PYTHON_ENV:-${LINGTU_CONFIG_DIR}/python.env}"
if [ -r "${python_env}" ]; then
    source "${python_env}"
fi
: "${LINGTU_PYTHON:=python3}"
: "${LINGTU_ENV:=real}"
: "${LINGTU_DATA_SOURCE:=field}"
: "${LINGTU_RUNTIME_CONTRACT:=real}"
: "${LINGTU_MODULE_TRANSPORT:=local}"
: "${LINGTU_ENDPOINT_TRANSPORT:=local}"
: "${LINGTU_ENDPOINT_CONTRACT:=}"
: "${LINGTU_SIMULATION_ONLY:=0}"
: "${LINGTU_ENABLE_ROBOT_DRIVER:=1}"
: "${LINGTU_COMMAND_OUTPUT_MODE:=local_driver}"
: "${LINGTU_HARDWARE_CONTROL_BOUNDARY:=module_graph_driver}"
: "${LINGTU_GATEWAY_PORT:=5050}"
: "${LINGTU_MAP_ARTIFACT_CONVERTER:=${LINGTU_REPO}/bin/octoplanner3d_pcd_to_octomap}"
: "${NAV_MAP_DIR:=/var/lib/lingtu/maps}"
: "${LINGTU_LIVOX_CONFIG_DIR:=/opt/lingtu/config/livox}"
: "${LINGTU_LIVOX_NET_IFACE:=}"
: "${LINGTU_LIVOX_HOST_IP:=}"
: "${LINGTU_LIVOX_LIDAR_IP:=}"
: "${LINGTU_LIVOX_BIND_LIDAR_IP:=0}"

prepend_cyclonedds_libs() {
    local prefix="${LINGTU_CYCLONEDDS_PREFIX:-}"
    local multiarch="${LINGTU_CYCLONEDDS_MULTIARCH:-${DEB_HOST_MULTIARCH:-}}"
    local machine
    local paths=()
    local joined=""
    local path

    if [ -z "${prefix}" ] || [ ! -d "${prefix}/lib" ]; then
        return 0
    fi

    if [ -z "${multiarch}" ] && command -v dpkg-architecture >/dev/null 2>&1; then
        multiarch="$(dpkg-architecture -qDEB_HOST_MULTIARCH 2>/dev/null || true)"
    fi

    if [ -z "${multiarch}" ] && command -v gcc >/dev/null 2>&1; then
        multiarch="$(gcc -print-multiarch 2>/dev/null || true)"
    fi

    if [ -z "${multiarch}" ]; then
        machine="$(uname -m 2>/dev/null || true)"
        case "${machine}" in
            aarch64|arm64)
                multiarch="aarch64-linux-gnu"
                ;;
            x86_64|amd64)
                multiarch="x86_64-linux-gnu"
                ;;
            armv7l|armhf)
                multiarch="arm-linux-gnueabihf"
                ;;
        esac
    fi

    if [ -n "${multiarch}" ]; then
        if [ -d "${prefix}/lib/${multiarch}" ]; then
            paths+=("${prefix}/lib/${multiarch}")
        fi
    fi

    paths+=("${prefix}/lib")
    for path in "${paths[@]}"; do
        if [ -z "${joined}" ]; then
            joined="${path}"
        else
            joined="${joined}:${path}"
        fi
    done
    export LD_LIBRARY_PATH="${joined}${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
}

if [ -d "${LINGTU_REPO}/src" ]; then
    export PYTHONPATH="${LINGTU_REPO}/src${PYTHONPATH:+:${PYTHONPATH}}"
fi

export LINGTU_REPO
export LINGTU_CONFIG_DIR
export LINGTU_PYTHON
export LINGTU_ENV
export LINGTU_DATA_SOURCE
export LINGTU_RUNTIME_CONTRACT
export LINGTU_MODULE_TRANSPORT
export LINGTU_ENDPOINT_TRANSPORT
export LINGTU_ENDPOINT_CONTRACT
export LINGTU_SIMULATION_ONLY
export LINGTU_ENABLE_ROBOT_DRIVER
export LINGTU_COMMAND_OUTPUT_MODE
export LINGTU_HARDWARE_CONTROL_BOUNDARY
export LINGTU_GATEWAY_PORT
export LINGTU_MAP_ARTIFACT_CONVERTER
export NAV_MAP_DIR
export LINGTU_LIVOX_CONFIG_DIR
export LINGTU_LIVOX_NET_IFACE
export LINGTU_LIVOX_HOST_IP
export LINGTU_LIVOX_LIDAR_IP
export LINGTU_LIVOX_BIND_LIDAR_IP
