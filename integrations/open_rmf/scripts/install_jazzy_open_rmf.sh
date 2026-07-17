#!/usr/bin/env bash
set -euo pipefail

if [[ "${LINGTU_ENABLE_EXTERNAL_ROS2_SIDECAR:-0}" != "1" ]]; then
  echo "Refusing to install ROS 2 by default." >&2
  echo "LingTu product runtime is ROS-free; this script is only for an isolated external Open-RMF sidecar." >&2
  echo "Set LINGTU_ENABLE_EXTERNAL_ROS2_SIDECAR=1 only on a dedicated lab/coordination host." >&2
  exit 2
fi

source /etc/os-release
if [[ "${ID:-}" != "ubuntu" || "${VERSION_CODENAME:-}" != "noble" ]]; then
  echo "Expected Ubuntu 24.04 Noble; found ${PRETTY_NAME:-unknown}" >&2
  exit 1
fi

sudo apt-get update
sudo apt-get install -y curl locales software-properties-common
sudo add-apt-repository universe -y

ros_apt_source_version="${ROS_APT_SOURCE_VERSION:-1.2.0}"
ros_apt_source_sha256="${ROS_APT_SOURCE_SHA256:-0804d9b13db770eb87019be414cd78378835228ad5fa801fc88758596dd8f7e5}"

ros_apt_source_deb="/tmp/ros2-apt-source.deb"
curl -fsSL \
  -o "${ros_apt_source_deb}" \
  "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ros_apt_source_version}/ros2-apt-source_${ros_apt_source_version}.noble_all.deb"
printf '%s  %s\n' "${ros_apt_source_sha256}" "${ros_apt_source_deb}" | sha256sum -c -
sudo dpkg -i "${ros_apt_source_deb}"

sudo apt-get update
sudo apt-get install -y \
  ros-dev-tools \
  ros-jazzy-ros-base \
  ros-jazzy-rmf-dev

if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  sudo rosdep init
fi
rosdep update

echo "ROS 2 Jazzy and Open-RMF are installed."
echo "Run: source /opt/ros/jazzy/setup.bash"
