# LingTu Current Roadmap

Current target: a ROS-free, typed-DDS field runtime for Thunder that can pass
no-motion navigation preview before any real velocity output is enabled.

## How To Use This File Across Tasks

Every new Codex task should start here before touching code:

1. Pick one row from **Current Work Board**.
2. Read the linked source-of-truth document before editing.
3. Keep the implementation inside that row's boundary.
4. Record proof in the row's `Evidence` field or in
   `docs/07-testing/field-runs/`.
5. Move the row to `Done` only after a runnable check passes.

Do not create a new plan file unless this file becomes too large to review.

## Product Modes

| Profile | Product Session | Purpose |
| --- | --- | --- |
| `teleop` | `teleop` | Manual velocity commands through Gateway/Teleop/MCP, still gated by safety. |
| `teleop_avoid` | `teleop_avoid` | Manual driving with localization, map, traversability, and collision veto. |
| `map` | `mapping` | Livox + SLAM build map artifacts. |
| `tracking` | `tracking` | Follow supplied waypoints/path without semantic decision making. |
| `nav` | `navigation` | Goal -> OctoPlanner3D -> local planner -> path follower -> cmd_vel. |
| `inspection` | `inspection` | Patrol/task scheduler creates navigation goals and records results. |
| `tare_explore` | `exploration` | TARE/frontier target generation feeding normal navigation. |

`session_mode` remains the low-level Gateway resource state:
`mapping`, `navigating`, or `exploring`. UIs should display
`product_session`.

## Field Data Path

```text
Livox MID-360 / IMU
  -> lingtu-livox-dds
  -> lingtu-slam-dds
  -> odometry + map_cloud + registered_cloud + localization status
  -> map layers / traversability
  -> lingtu-nav-dds
  -> OctoPlanner3D global path
  -> C++ LocalPlanner / PathFollower
  -> /nav/cmd_vel
```

Map files are not DDS messages:

```text
map.pcd
metadata.json
octomap.ot
occupancy.npz
optional tomogram.pickle
```

## Current Priorities

1. Fix saved-map localization alignment on Sunrise before motion tests.
2. Keep `publish_cmd_vel=false` until no-motion route preview passes.
3. Finish typed C++ navigation endpoint coverage for goal/cancel/status/path.
4. Keep Python DDS adapters out of the field default path.
5. Keep ROS2 only as explicit compatibility adapter code.
6. Validate MapService save -> octomap artifact -> OctoPlanner3D preview.
7. Validate `tare_explore` as exploration target generation, not direct chassis control.

## Current Work Board

| Status | Work | Boundary | Evidence |
| --- | --- | --- | --- |
| Active | Align SLAM localization with the active saved map on Sunrise. | `lingtu-slam-dds`, relocalization, map frame, `map_odom_tf`. | Field note under `docs/07-testing/field-runs/`. |
| Active | Prove no-motion navigation preview. | Gateway goal API, MapService, OctoPlanner3D, native nav endpoint. | `validate_plan` returns OctoPlanner3D path and `publish_cmd_vel=false`. |
| Active | Finish C++ nav endpoint inputs. | Goal/cancel/status/path, odometry, traversability, registered cloud. | Native endpoint snapshot shows all required inputs true. |
| Active | Validate map package path. | `map.pcd`, `metadata.json`, `octomap.ot`, `occupancy.npz`. | Saved map is navigation-ready and active. |
| Next | Validate `tare_explore`. | TARE target generation only; navigation still owns planning/execution. | Exploration target becomes normal `goal_pose`. |
| Blocked | Real motion smoke. | CmdVelMux and robot command sink. | Blocked until no-motion preview and safety gates pass. |

## Source Of Truth

| Topic | Current document |
| --- | --- |
| Product/session naming | `docs/architecture/PRODUCT_MODE_RUNTIME_CONTRACT.md` |
| Navigation dataflow | `docs/architecture/NAVIGATION_RUNTIME_DATAFLOW.md` |
| Global planner contract | `docs/architecture/GLOBAL_PLANNING_CONTRACT.md` |
| Map lifecycle | `docs/architecture/MAP_SERVICE_CONTRACT.md` |
| Runtime bus | `docs/architecture/LINGTU_RUNTIME_BUS_DECISION.md` |
| Deployment | `docs/04-deployment/README.md` |
| Robot CLI | `docs/04-deployment/lingtu_cli.md` |

## Done When

- `lingtu mode switch map/nav/tracking/inspection/tare_explore` reports the
  expected `product_session`.
- Sunrise services are active: `lingtu-livox-dds`, `lingtu-slam-dds`,
  `lingtu-traversability-dds`, `lingtu-nav-dds`, and `lingtu`.
- SLAM publishes stable odometry inside the active map frame.
- `/api/v1/maps/{name}/validate_plan` returns an OctoPlanner3D path with no
  fallback to PCT/direct/A*.
- Local planner receives odometry, global path, registered cloud, and
  traversability input before speed output is enabled.
- Motion smoke proves cmd_vel only after safety and no-motion gates pass.
