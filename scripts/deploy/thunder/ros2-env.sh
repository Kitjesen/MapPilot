#!/usr/bin/env bash
# Thunder ROS 2 compatibility environment.
#
# Source this only from deployment compatibility services that still run ROS 2
# algorithm nodes. Product entrypoints should use LingTu profiles instead.

: "${LINGTU_ROS_DISTRO:=humble}"
: "${LINGTU_REPO:=${HOME}/data/inovxio/lingtu}"
: "${ROS_DOMAIN_ID:=0}"
: "${RMW_IMPLEMENTATION:=rmw_cyclonedds_cpp}"

ROS_SETUP="/opt/ros/${LINGTU_ROS_DISTRO}/setup.bash"
if [ -f "${ROS_SETUP}" ]; then
    # shellcheck disable=SC1090
    source "${ROS_SETUP}"
fi

LINGTU_OVERLAY_SETUP="${LINGTU_ROS_OVERLAY_SETUP:-${LINGTU_REPO}/install/setup.bash}"
if [ -f "${LINGTU_OVERLAY_SETUP}" ]; then
    # shellcheck disable=SC1090
    source "${LINGTU_OVERLAY_SETUP}"
fi

export ROS_DOMAIN_ID
export RMW_IMPLEMENTATION
