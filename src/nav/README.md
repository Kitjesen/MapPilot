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

SCAN recovery uses the same inflated 3-D collision bitmap and cylinder chain
as SCAN planning, without projecting and inflating it again. The `nav` Product
allows up to three recovery attempts. A straight translation may leave an
initially occupied boundary cell only when both cylinder traces enter no other
occupied cell and end in free space. Final command checks restrict that exit
to a verified, slow translation with no commanded rotation, and reject measured
motion directed into another occupied cell. Simulation receive
times are rebased at the endpoint before comparing map freshness with execution
time. Recovery motion does not demonstrate stair-climbing capability.

The 4998 controller currently used in MuJoCo has a measured low-speed lateral
tracking limitation: a 0.15 m/s lateral command produced almost no steady lateral
motion in an isolated flat-ground probe, whereas 0.30 m/s tracked about 0.285 m/s.
Finding a collision-free recovery path therefore does not prove that this policy
can execute it. See the recorded physical runs in
`artifacts/mujoco-stair-side-exit-2026-09-09/report.md`; do not raise a checked
command after arbitration to compensate for this limitation.

A subsequent simulation at the recorded stair-side position completed recovery,
ground navigation, and stopping with a 0.30 m/s recovery limit and 2.0 m/s²
follower acceleration, while retaining 0.5 m/s² for SCAN and braking checks.
The boundary-exit gate now consumes the configured recovery limit and requires
the reaction-plus-braking distance to fit its checked exit. Defaults remain
unchanged; the single successful trial and its unsuccessful controls are recorded
in `artifacts/mujoco-faster-recovery-2026-09-09/report.md`.

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
