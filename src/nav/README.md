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
- local planning through `LocalPlanner`, using the selected SCAN or CMU algorithm;
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
- `LocalPlanner` is the local-planning module; SCAN and CMU are its selectable algorithms.
- `real` and `sim` differ in endpoints and devices, not in navigation
  ownership. Both use native `navd` rather than a Python planning substitute.
- `/nav/global_path` and `/nav/local_path` are telemetry. Internal planner-to-
  tracker handoff is a direct C++ call inside `navd`.
- Only the driver forwards the final checked command to robot hardware.

Recovery uses the same 3-D collision queries and selected body as LocalPlanner.
Explicit compound bodies query measured occupancy; the legacy cylinder path
uses its matching inflated map. Neither path authorizes motion through a 2-D
projection or adds a second inflation. Candidate retries use current geometry
without a separate map-generation barrier. The endpoint owns input freshness
and final checked output; Recovery owns only its bounded candidate search and
execution progress.

Finding a collision-free path does not prove that the selected locomotion
policy can execute it. Record the robot, policy, body part set and measured
motion for each MuJoCo trial; do not apply historical 4998-controller results
to the current Thunder v39 model61128. Current failures and validation levels
are in [the navigation error record](../../docs/operations/error_record.md).

For an active route, intermittent `Ready` / `Pending` / `Blocked` local plans
must not erase the observation that a commanded robot has made no progress.
The existing two-second no-progress trigger can therefore enter recovery even
when planning alternates between success and failure. Recovery still needs a
collision-checked action; a dynamic obstacle overlapping the body can rule out
every candidate.

Missing or stale motion inputs hold execution until current data is available.
Valid position, required transforms, local collision data and driver state are
admitted on the current tick without a second multi-frame recovery counter.
An auxiliary health message is not an independent readiness barrier; a current
explicit LOST or catastrophic localization report still holds motion. Scheduler
recovery releases its own temporary hold without taking operator authority or
discarding the route. Explicit pause, takeover and E-stop remain independent.

## References

- [Short file index](FILES.md)
- [Native navigation](cpp/README.md)
- [Native endpoint](cpp/endpoint/README.md)
- [Architecture and ownership](../../docs/architecture.md)
- [Product runtime and native dataflow](../../docs/runtime.md)

## Verification

Python tests cover the retained Host command and facade surfaces under
`tests/nav/`; exploration tests live under `tests/explore/`. Native planner,
tracker, endpoint, and safety tests are built through the CMake presets
documented in [`cpp/README.md`](cpp/README.md).
