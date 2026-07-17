#!/usr/bin/env bash
# Thunder lightweight runtime environment.
#
# This file is intentionally free of ROS setup. Use it for local/lite product
# profiles where LingTu runs only the Python Module graph and local transports.

: "${LINGTU_REPO:=/opt/lingtu/current}"
: "${LINGTU_PROFILE:=thunder-lite}"
: "${LINGTU_MODULE_TRANSPORT:=local}"
: "${LINGTU_ENDPOINT:=thunder_lite}"
: "${LINGTU_ENDPOINT_TRANSPORT:=local}"
: "${LINGTU_ENDPOINT_CONTRACT:=}"
: "${LINGTU_SIMULATION_ONLY:=0}"
: "${LINGTU_ENABLE_ROBOT_DRIVER:=1}"
: "${LINGTU_COMMAND_OUTPUT_MODE:=local_driver}"
: "${LINGTU_HARDWARE_CONTROL_BOUNDARY:=module_graph_driver}"
: "${LINGTU_GATEWAY_PORT:=5050}"
: "${LINGTU_MAP_ARTIFACT_CONVERTER:=${LINGTU_REPO}/build/octoplanner3d_headless/octoplanner3d_pcd_to_octomap}"
: "${LINGTU_MAPS_LIB:=${LINGTU_REPO}/build/maps/liblingtu_maps.so}"
: "${LINGTU_LIVOX_CONFIG_DIR:=/opt/lingtu/config/livox}"
: "${LINGTU_LIVOX_NET_IFACE:=eth1}"
: "${LINGTU_LIVOX_HOST_IP:=}"
: "${LINGTU_LIVOX_LIDAR_IP:=}"
: "${LINGTU_LIVOX_BIND_LIDAR_IP:=0}"

if [ -d "${LINGTU_REPO}/src" ]; then
    export PYTHONPATH="${LINGTU_REPO}/src${PYTHONPATH:+:${PYTHONPATH}}"
fi

export LINGTU_REPO
export LINGTU_PROFILE
export LINGTU_MODULE_TRANSPORT
export LINGTU_ENDPOINT
export LINGTU_ENDPOINT_TRANSPORT
export LINGTU_ENDPOINT_CONTRACT
export LINGTU_SIMULATION_ONLY
export LINGTU_ENABLE_ROBOT_DRIVER
export LINGTU_COMMAND_OUTPUT_MODE
export LINGTU_HARDWARE_CONTROL_BOUNDARY
export LINGTU_GATEWAY_PORT
export LINGTU_MAP_ARTIFACT_CONVERTER
export LINGTU_MAPS_LIB
export LINGTU_LIVOX_CONFIG_DIR
export LINGTU_LIVOX_NET_IFACE
export LINGTU_LIVOX_HOST_IP
export LINGTU_LIVOX_LIDAR_IP
export LINGTU_LIVOX_BIND_LIDAR_IP
