# Build helper scripts

This folder contains build-time helpers. Runtime profile launches live in
`lingtu.py`; robot deployment and service installation live under
`scripts/deploy/` and `scripts/ota/`.

Run commands from the repository root unless a script says otherwise.

| Script | Purpose |
|--------|---------|
| `build_ros_workspace.sh` | Build the ROS 2 workspace with `colcon`. |
| `build_tare.sh` | Build the external CMU TARE planner integration. |
| `fetch_ortools.sh` | Fetch OR-Tools binaries for the TARE planner path. |
| `build_ortools_from_source.sh` | Build OR-Tools locally when binaries are unavailable. |

Related root-level helpers:

| Script | Purpose |
|--------|---------|
| `scripts/build/build_nav_core.sh` | Build the Python `nav_core`/nanobind extension without a full ROS 2 workspace. |
| `scripts/build/build_dufomap.sh` | Build the DUFOMap cleanup binary used by map-save filtering. |
