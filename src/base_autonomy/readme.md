# base_autonomy — C++ Autonomy Stack (Python Module Layer)

Local obstacle avoidance, terrain analysis, and path following for quadruped robots.
Three independent Modules (Layer 2) wrapping compiled C++ cores via nanobind or NativeModule subprocess.

## Directory layout

```
base_autonomy/
├── modules/                   Python Module layer (runtime)
│   ├── terrain_module.py      TerrainModule        — ground estimation + obstacle detection
│   ├── local_planner_module.py LocalPlannerModule  — obstacle avoidance + path scoring
│   ├── path_follower_module.py PathFollowerModule  — pure-pursuit tracking → cmd_vel
│   ├── autonomy_module.py     add_autonomy_stack() convenience helper
│   └── _nav_core_loader.py    locates _nav_core.so (nanobind extension)
├── native_factories.py        NativeModule factories — reads params from robot_config.yaml
├── local_planner/             C++ ROS2 package (colcon)
│   ├── src/
│   │   ├── localPlanner.cpp   ROS2 thin shell (algorithm in nav_core/local_planner_core.hpp)
│   │   └── pathFollower.cpp   ROS2 thin shell (algorithm in nav_core/path_follower_core.hpp)
│   ├── paths/                 Pre-computed candidate paths (PLY + correspondences.txt)
│   │   └── path_generator.m   One-time MATLAB tool that generated the PLY files
│   ├── config/
│   │   ├── standard.yaml      Standard-wheel tuning (omniDirGoalThre=-1.0)
│   │   └── omniDir.yaml       Omni-directional tuning (omniDirGoalThre=0.5)
│   └── test/                  C++ unit tests (gtest — voxel grid, path scoring, pure pursuit)
├── terrain_analysis/          C++ ROS2 package — ground estimation + obstacle map
├── terrain_analysis_ext/      C++ ROS2 package — connectivity + 2.5D height map
└── sensor_scan_generation/    C++ ROS2 package — LiDAR scan preprocessing
```

## Module ports

### TerrainModule (layer=2)


| Direction | Port             | Type          | Description                       |
| --------- | ---------------- | ------------- | --------------------------------- |
| In        | `odometry`       | `Odometry`    | Robot pose for rolling map center |
| In        | `map_cloud`      | `PointCloud2` | Registered LiDAR point cloud      |
| Out       | `terrain_map`    | `PointCloud2` | Filtered obstacle + height map    |
| Out       | `traversability` | `dict`        | Traversability summary            |
| Out       | `elevation_map`  | `np.ndarray`  | 2D elevation grid                 |
| Out       | `alive`          | `bool`        | Backend health signal             |


**Backends** (set via `backend=` constructor arg):


| Backend                | Requires            | Description                                       |
| ---------------------- | ------------------- | ------------------------------------------------- |
| `nanobind` *(default)* | `_nav_core.so`      | C++ `TerrainAnalysisCore` in-process, zero ROS2   |
| `native`               | ROS2 + colcon build | C++ `terrainAnalysis` subprocess via NativeModule |
| `simple`               | nothing             | Passthrough for testing                           |


### LocalPlannerModule (layer=2)


| Direction | Port              | Type          | Description                        |
| --------- | ----------------- | ------------- | ---------------------------------- |
| In        | `odometry`        | `Odometry`    | Robot pose + yaw                   |
| In        | `terrain_map`     | `PointCloud2` | Obstacle cloud from TerrainModule  |
| In        | `waypoint`        | `PoseStamped` | Current navigation waypoint        |
| In        | `boundary`        | `PointCloud2` | Geofence boundary (hard obstacles) |
| In        | `added_obstacles` | `PointCloud2` | Externally injected obstacles      |
| Out       | `local_path`      | `Path`        | Best obstacle-free path segment    |
| Out       | `alive`           | `bool`        | Backend health signal              |


**Backends:**


| Backend                    | Requires            | Description                                          |
| -------------------------- | ------------------- | ---------------------------------------------------- |
| `nanobind` *(preferred)*   | `_nav_core.so`      | C++ `LocalPlannerCore`, full CMU scoring in-process  |
| `cmu_py` *(auto-fallback)* | numpy               | Pure-Python CMU scorer (same algorithm, ~10× slower) |
| `cmu`                      | ROS2 + colcon build | C++ `localPlanner` subprocess via NativeModule       |
| `simple`                   | nothing             | Straight-line path for testing                       |


The `nanobind` backend auto-falls back to `cmu_py` if `_nav_core.so` is missing.

### PathFollowerModule (layer=2)


| Direction | Port         | Type       | Description                  |
| --------- | ------------ | ---------- | ---------------------------- |
| In        | `odometry`   | `Odometry` | Robot pose                   |
| In        | `local_path` | `Path`     | Path from LocalPlannerModule |
| Out       | `cmd_vel`    | `Twist`    | Velocity command to driver   |
| Out       | `alive`      | `bool`     | Backend health signal        |


**Backends:**


| Backend                | Requires            | Description                                                              |
| ---------------------- | ------------------- | ------------------------------------------------------------------------ |
| `nav_core` *(default)* | `_nav_core.so`      | C++ `compute_control`: adaptive lookahead, two-way drive, accel limiting |
| `pure_pursuit`         | ROS2 + colcon build | C++ `pathFollower` subprocess via NativeModule                           |
| `pid`                  | nothing             | Python PID fallback for testing                                          |


## Usage

```python
from core.blueprint import Blueprint
from base_autonomy.modules import TerrainModule, LocalPlannerModule, PathFollowerModule

bp = Blueprint()
bp.add(TerrainModule,      backend="nanobind")
bp.add(LocalPlannerModule, backend="nanobind")
bp.add(PathFollowerModule, backend="nav_core")

# Or via convenience helper (same result):
from base_autonomy.modules import add_autonomy_stack
add_autonomy_stack(bp, backend="nanobind")
```

For production stacks use the `navigation()` factory in `src/core/blueprints/stacks/navigation.py`,
which handles enable/disable, backend selection, and explicit wiring automatically.

## Building the C++ extension (_nav_core.so)

The `nanobind` / `nav_core` backends require the compiled extension:

```bash
bash scripts/build/build_nav_core.sh   # cmake + nanobind, ~30s
# or
make nav_core
```

The `.so` is placed (or symlinked) under `src/` and auto-discovered by `_nav_core_loader.py`.

## Parameters

All runtime parameters come from `config/robot_config.yaml`.
The `native_factories.py` in this directory reads the `terrain`, `local_planner`,
`path_follower`, `control`, `autonomy`, `geometry`, `speed`, and `safety` sections
and passes them to the C++ NativeModule subprocess when `backend="native"` or `"cmu"`.

The Python `nanobind` / `cmu_py` / `pid` backends read the same config via `core.config.get_config()`.

## Gamepad reference (for manual override via TeleopModule)

When a joystick is connected and the `TeleopModule` is active:


| Axis / Button                | Effect                                      |
| ---------------------------- | ------------------------------------------- |
| LT pressed (axes[2] < −0.1)  | Switch to manual mode — autonomy paused     |
| LT released (axes[2] > −0.1) | Return to autonomy mode (dead-man switch)   |
| RT released                  | Obstacle avoidance enabled (default, safe)  |
| RT pressed (axes[5])         | Obstacle avoidance disabled (force-through) |
| Button 5                     | Clear nearby point cloud (reset local map)  |


Joystick cmd_vel does **not** bypass the path follower directly; it goes through
`TeleopModule → Driver.cmd_vel`. The local planner uses the joystick direction as a
reference goal direction. Priority: Teleop > VisualServo > PathFollower.

## noRotAtStop / noRotAtGoal

Controlled via `config/robot_config.yaml → path_follower.no_rot_at_stop / no_rot_at_goal`:


| Setting          | Value               | Behaviour                                                         |
| ---------------- | ------------------- | ----------------------------------------------------------------- |
| `no_rot_at_stop` | `false` *(default)* | Robot may rotate in-place to avoid obstacles after emergency stop |
| `no_rot_at_stop` | `true`              | Full lock on emergency stop — wait for human intervention         |
| `no_rot_at_goal` | `true` *(default)*  | Stop precisely at goal, no in-place spin                          |
