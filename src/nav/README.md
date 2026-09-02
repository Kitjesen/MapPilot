# Navigation

`src/nav/` contains LingTu's navigation command surface and the native C++
navigation implementation. Current `real` and `sim` Products use the same
native endpoint shape.

```text
Gateway / Agent / CLI
  -> NavSkills
  -> GoalService
  -> native command adapter
  -> navd
       -> global planning
       -> local planning
       -> route tracking
       -> safety and motion authority
  -> /nav/cmd_vel
  -> driver
```

Python does not run a second navigation planner or motion-control chain.

## Ownership

Host-side Python owns:

- typed goal, cancel, stop, teleop, exploration, and inspection commands;
- Agent/MCP navigation skills;
- inspection service facade;
- native client adapters;
- status and telemetry presentation outside this package.

Native C++ `navd` owns:

- saved-map goal admission and global planning;
- local planning with CMU or SCAN;
- route execution and path or trajectory tracking;
- recovery, geofence, obstacle, traversability, and freshness gates;
- final velocity shaping, motion authority, E-stop, and `/nav/cmd_vel` output.

Map storage and live map state remain under `src/maps/`. SLAM,
traversability, and the robot driver remain separate native processes connected
to `navd` through typed DDS.

## Package layout

| Area | Responsibility |
| --- | --- |
| `commands/` | Host Module that forwards typed navigation commands to `navd`. |
| `services/` | Low-rate goal admission, task bookkeeping, and frame helpers. |
| `skills/` | Agent/MCP command and status surface. |
| `inspection/` | Python inspection facade; execution is native. |
| `adapters/native/` | Native command, operator-motion, exploration, and inspection clients. |
| `cpp/` | Global/local planning, tracking, safety, DDS endpoint, and native clients. |

The CMU local planner selects the robot-specific path bank under
`src/nav/cpp/planning/local/cmu/paths/` (`go2/` or `thunder/`) in development.
Native releases install the same banks under `share/lingtu/cmu_paths/`.
Each bank carries its own collision `search_radius.txt`; `navd` validates and
uses that radius when converting obstacle points into correspondence voxels.

The backend is an enhanced CMU-core port, not a byte-for-byte Go2 runtime.
LingTu retains the CMU candidate bank and selector, while its stateful obstacle
fusion, route guide, geometric follower, recovery, and final safety remain
LingTu-owned. The exact compatibility boundary is recorded in the local
planning contract below.

## Product rules

- A goal is intent, not a motor command. It must pass through `navd` planning,
  tracking, safety, and authority before reaching the driver.
- OctoPlanner3D is the default saved-map global planner. FAR is an explicit 2-D
  occupancy option; it is not a silent fallback.
- CMU and SCAN are explicit local-planner backends behind one native interface.
- `real` and `sim` differ in endpoints and devices, not in navigation
  ownership. Both use native `navd` rather than a Python planning substitute.
- `/nav/global_path` and `/nav/local_path` are telemetry. Internal planner-to-
  tracker handoff is a direct C++ call inside `navd`.
- Only the driver forwards the final checked command to robot hardware.

## References

- [Short file index](FILES.md)
- [Native navigation](cpp/README.md)
- [Native endpoint](cpp/endpoint/README.md)
- [Global planning contract](../../docs/architecture/GLOBAL_PLANNING_CONTRACT.md)
- [Local planning and tracking contract](../../docs/architecture/LOCAL_PLANNING_AND_TRACKING_CONTRACT.md)
- [Runtime data flow](../../docs/architecture/NAVIGATION_RUNTIME_DATAFLOW.md)

## Verification

Python tests cover the retained Host command and facade surfaces under
`src/nav/tests/`. Native planner, tracker, endpoint, and safety tests are built
through the CMake presets documented in [`cpp/README.md`](cpp/README.md).
