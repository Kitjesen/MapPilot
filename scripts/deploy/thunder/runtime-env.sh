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
