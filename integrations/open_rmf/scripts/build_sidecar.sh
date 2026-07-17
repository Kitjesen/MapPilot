#!/usr/bin/env bash
set -euo pipefail

lingtu_root="${LINGTU_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)}"
workspace="${LINGTU_RMF_WS:-${HOME}/lingtu_rmf_ws}"
package_source="${lingtu_root}/integrations/open_rmf/ros_ws/src/lingtu_rmf_adapter"

if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then
  echo "ROS 2 Jazzy is not installed. Run install_jazzy_open_rmf.sh first." >&2
  exit 1
fi
if [[ ! -d "${package_source}" ]]; then
  echo "LingTu RMF package not found: ${package_source}" >&2
  exit 1
fi

source /opt/ros/jazzy/setup.bash
mkdir -p "${workspace}/src"
ln -sfn "${package_source}" "${workspace}/src/lingtu_rmf_adapter"

cd "${workspace}"
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select lingtu_rmf_adapter

echo "Built LingTu RMF sidecar in ${workspace}"
