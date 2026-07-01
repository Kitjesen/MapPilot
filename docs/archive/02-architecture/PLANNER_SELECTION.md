# Global Planner Selection

This is the rationale for which global-planner backend a given profile
uses. The runtime contract is defined by `_PCTBackend` and `_AStarBackend`
(both registered under `runtime.registry`), and used by
`src/nav/services/plan/global_planner/service.py`.

## TL;DR

**On the S100P (`lingtu.py nav` / `s100p` / `explore` / `tare_explore`)
the planner is `pct`.** A* is only used in `map`, `sim`, `sim_nav`, `dev`,
and `stub` because `ele_planner.so` is built for aarch64.

Quick check:

```bash
ssh sunrise@192.168.66.190 "ps aux | grep 'lingtu.py' | grep -v grep"
# Output containing 'lingtu.py nav' 鈫?pct
# Output containing 'lingtu.py map' 鈫?astar (mapping doesn't usually plan)
```

## Runtime path: Web click 鈫?cmd_vel

```
[Web: scene tab]
  user clicks the 3-D ground
  鈫?pendingGoal = {x, y}
  鈫?confirm panel
  鈫?POST /api/v1/goto {x, y}

[GatewayModule]
  /api/v1/goto handler
  鈫?PlannerService.send_instruction(kind="goto", x, y)
  鈫?SemanticPlannerModule.goal_pose Out[PoseStamped]

[NavigationModule  src/nav/mission/navigation_module.py]
  goal_pose In[PoseStamped]
  鈫?mission FSM: IDLE 鈫?PLANNING
  鈫?GlobalPlannerService.plan(start=robot_pose, goal=user_goal)

[GlobalPlannerService  src/nav/services/plan/global_planner/service.py]
  picks the backend by `planner=` argument:
    "pct"   鈫?_PCTBackend (ele_planner.so, aarch64 C++)
    "astar" 鈫?_AStarBackend (pure Python, 8-connected)
  Input:  tomogram.pickle (built from map_cloud + elevation_map)
  Output: np.ndarray (N, 3) world-frame [x, y, z]

[WaypointTracker  src/nav/mission/tracking/waypoint_tracker.py]
  downsample path 鈫?waypoints
  鈫?NavigationModule.waypoint Out[PoseStamped] streamed point-by-point
  鈫?arrival / stuck detection, recovery, replan

[PathFollower  src/nav/core/path_follower_runtime.hpp]
  waypoint + current pose 鈫?Pure Pursuit 鈫?cmd_vel

[CmdVelMux  src/nav/services/safety/cmd_vel_mux_module.py]
  priority arbitration with 0.5 s freshness:
    teleop 100 > visual_servo 80 > recovery 60 > path_follower 40
  鈫?driver_cmd_vel Out

[ThunderDriver  src/drivers/thunder/thunder_driver.py]
  driver_cmd_vel 鈫?gRPC Walk 鈫?brainstem 鈫?motors
```

Important:

- The `nav` profile uses **PCT** (C++ `ele_planner.so`), with the
  tomogram built offline from the saved PCD via the elevation map.
- Goals do **not** travel through the ROS2 `/nav/goal_pose` topic; in the
  Module-First architecture the goal stays in-process Port 鈫?Port.
- The C++ autonomy ROS2 nodes (`terrain_analysis` + `local_planner` +
  `path_follower`) are not used when `enable_native=False`. The Python
  autonomy chain replaces them.

## Backend catalogue

### 1. PCT `ele_planner.so` 鈥?production, S100P only

| Property | Value |
|----------|-------|
| Registry name | `pct` |
| Implementation | C++ (HKU/HKUST, GPLv2) |
| Entry | `_PCTBackend` 鈫?`TomogramPlanner.plan()` |
| C++ source | `src/nav/services/plan/global_planner/algorithm/pct/vendor/pct_planner/planner/lib/src/ele_planner/` |
| Compiled `.so` | `planner/lib/ele_planner.so` (aarch64 only) |
| Map format | Tomogram `.pickle` (multi-layer traversability + elevation + gradient) |
| Capabilities | 3D terrain awareness (stairs, ramps), GPMP trajectory optimisation, slope penalties |
| Output | `np.ndarray (N, 3)` world coordinates |

Used by the `nav`, `explore`, `tare_explore`, and `s100p` profiles.

### 2. Pure-Python A* 鈥?fallback for non-aarch64

| Property | Value |
|----------|-------|
| Registry name | `astar` |
| Implementation | `_AStarBackend`, 8-connected A* |
| File | `src/nav/services/plan/global_planner/algorithm/pct/backend.py` |
| Map format | The same tomogram pickle, but only the ground-floor slice |
| Capabilities | 2D, no trajectory optimisation, no slope awareness |
| Reason for existing | `ele_planner.so` is aarch64-only; CI and dev machines need a backend |

Used by `map`, `sim`, `sim_nav`, `dev`, `stub`.

## Why we don't use Python A* in production

The `_AStarBackend` plus the legacy `pct_planner_astar.py` have several
issues that make them unsuitable for the real robot:

1. `_AStarBackend.plan()` reconstructs the path without inserting the
   start cell into `came_from`, so the start point is sometimes dropped.
2. `pct_planner_astar.py` falls back to a straight line from start to
   goal when A* fails *and* still publishes `FAILED` 鈥?execution
   continues despite the contradiction.
3. 2-D only 鈥?the ground-floor slice cannot represent stairs or ramps.
4. No trajectory smoothing, the resulting waypoint stream is jagged.
5. Pure Python with `dict`-based open / closed sets 鈥?a 200脳200 grid can
   take tens of seconds.

Treat A* purely as a portability shim, not as a quality option.

## Profile mapping

| Profile | Robot | Planner | Reason |
|---------|-------|---------|--------|
| `nav` | S100P | `pct` | Real robot, needs 3D terrain awareness |
| `explore` | S100P | `pct` | Same |
| `tare_explore` | S100P | `pct` | TARE provides waypoints; PCT plans the path |
| `s100p` | S100P | `pct` | Same as `nav` |
| `map` | S100P | `astar` | Mapping doesn't plan in earnest |
| `sim` | MuJoCo | `astar` | `ele_planner.so` won't load on x86 |
| `sim_nav` | stub | `astar` | Pure Python sim, no aarch64 binaries |
| `dev` | stub | `astar` | No hardware, no tomogram |
| `stub` | stub | `astar` | CI only |

## `_PCTBackend` API

```python
backend = _PCTBackend(
    tomogram_path="/path/to/building.pickle",
    obstacle_thr=49.9,
)

path = backend.plan(
    start=np.array([x, y, z]),
    goal=np.array([x, y, z]),
)
if not path:
    # genuine failure 鈥?triggers LERa recovery
    ...
```

`TomogramPlanner.plan()` internally:

1. world coordinates 鈫?grid index (`pos2idx`)
2. height 鈫?slice index (`pos2slice`)
3. C++ `ele_planner` runs 3D A* (`a_star.so`)
4. C++ `GPMP` smooths the trajectory (`traj_opt.so`)
5. grid index 鈫?world coordinates (`transTrajGrid2Map`)

## Files of record

| File | Role |
|------|------|
| `src/nav/services/plan/global_planner/algorithm/pct/backend.py` | `_PCTBackend` and `_AStarBackend` registration |
| `src/nav/services/plan/global_planner/algorithm/pct/vendor/pct_planner/planner/scripts/planner_wrapper.py` | `TomogramPlanner` Python wrapper around the `.so` |
| `src/nav/services/plan/global_planner/algorithm/pct/vendor/pct_planner/planner/lib/src/ele_planner/` | C++ source |
| `src/nav/services/plan/global_planner/service.py` | `GlobalPlannerService` consumed by `NavigationModule` |
| `cli/profiles_data.py` | Profile `planner` field |
| `src/runtime/blueprints/full_stack.py` | Blueprint default `planner_backend="astar"` |
