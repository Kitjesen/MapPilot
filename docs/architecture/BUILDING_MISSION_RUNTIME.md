# Building Mission Runtime

Status: extension contract; not a replacement for native navigation endpoint safety gates
Audience: building/facility integration maintainers
Replaced by: not replaced

LingTu owns building-scale execution without adding ROS to the robot runtime.
Open-RMF remains an optional facility-level sidecar; it may submit a
floor-aware objective, but it never publishes `cmd_vel`, changes the native
control mode, or bypasses LingTu safety.

## Functional ownership

| Function | LingTu owner | What it does |
| --- | --- | --- |
| Building objective | `BuildingMissionOrchestrator` | Admits one mission, preserves request correlation, runs a floor transition when required, then dispatches the final native goal. |
| Same-floor/cabin motion | `CorrelatedNativeNavigationPort` | Uses native navigation commands and accepts completion only from fresh, correlated native endpoint evidence in `autonomy` mode. |
| Lift transition | `LiftTransitionExecutor` | Runs approach, call/wait, enter, ride, target localization, and exit as a deterministic fail-closed state machine. |
| Floor map/localization | `NativeFloorLocalizationAdapter` | Validates the target source cloud, activates the native map, runs native saved-map relocalization, and requires fresh post-request `TRACKING` evidence. |
| Lift hardware/protocol | Site infrastructure adapter | Maps the neutral lift request/state contract to Open-RMF infrastructure or a building controller. No implementation is guessed in the robot core. |

All implementation files are under `src/nav/building/`.

## Runtime flow

```text
floor-aware objective
  -> verify request + exclusive native autonomy ownership
  -> same floor? final native goal
  -> resolve configured directed lift transition
  -> approach source lobby
  -> claim/call lift session and wait for fresh correlated open-door state
  -> enter cabin while continuously checking door + motion state
  -> command zero-motion hold and request destination floor
  -> wait for fresh correlated target-floor stopped/open state
  -> validate target map artifact
  -> activate target map
  -> native saved-map relocalization
  -> verify fresh target-map TRACKING state
  -> exit cabin while continuously checking door + motion state
  -> release lift session
  -> final native navigation goal
```

The transition uses four configured poses per directed route:
`source_lobby`, `source_cabin`, `target_cabin`, and `target_lobby`. Reverse
travel is a separate route; it is never inferred by swapping Z coordinates.

## Fail-closed gates

The mission or transition stops and does not dispatch the next motion when any
of these conditions occurs:

- native mode is not exactly `autonomy`, or estop/operator takeover is latched;
- native status, pose, or lift state is missing, stale, non-finite, or belongs
  to a different request/session;
- the lift moves or closes its door while the robot is entering or exiting;
- a transition route or target map binding is absent;
- the target source cloud/artifact gate, active-map verification, native
  relocalization, or fresh target-floor localization check fails;
- target floor identity after the transition does not exactly match
  `(building_id, floor_id, map_id)`;
- lift ownership cannot be released cleanly.

`autonomy`, `teleop`, and `teleop_avoid` remain mutually exclusive. Operator
takeover aborts the active building mission; stale motion is never resumed.

## Current evidence and limits

| Capability | Status |
| --- | --- |
| Same-floor orchestration through a correlated native goal | Unit-tested |
| Cross-floor state machine with fake navigation/lift/map ports | Unit-tested |
| Native map activation + native saved-map relocalization adapter | Unit-tested |
| Stale lift, door closure, mode loss, map failure, and localization hold gates | Unit-tested |
| MuJoCo named-joint lift/door adapter | Implemented and MuJoCo-backed unit-tested |
| Thunder model + MuJoCo lift kinematic source-to-target mission | One deterministic pass with phase timeline and video |
| Real building lift controller adapter | Not implemented; site protocol is unknown |
| Open-RMF live cross-floor task wiring | Disabled; current sidecar remains fail-closed |
| MuJoCo local/global planner, gait, SLAM, and saved-map relocalization through the lift | Not yet validated |
| MuJoCo reverse transition and injected-failure repetition matrix | Not yet validated |
| Real single-robot lift round trip | Not yet validated |

ROS 2 is not required for any LingTu class above. If Open-RMF is later used,
ROS 2 stays on a separate WSL2/container/coordination host.

The current simulator gate is intentionally narrow. It loads the configured
Thunder robot and moves real MuJoCo cabin/door joints, but robot waypoint
motion and rider coupling are kinematic. Its report sets
`physical_gait_verified=false`, `global_planner_verified=false`,
`local_planner_verified=false`, `real_localization_verified=false`, and
`native_dds_transport_exercised=false` so this evidence cannot be mistaken for
the navigation or hardware product gate.

Run it without ROS:

```bash
PYTHONPATH=src:. python sim/scripts/mujoco/lift_transition_acceptance.py \
  --output-dir artifacts/mujoco_lift_transition_acceptance \
  --record-video
```

Implementation surfaces:

- `src/drivers/sim/mujoco/lift.py`
- `sim/worlds/mujoco/lift_building_scene.xml`
- `sim/scripts/mujoco/lift_transition_acceptance.py`

## Acceptance ladder

1. Keep same-floor navigation at its existing product gate.
2. **Completed at kinematic scope:** deterministic MuJoCo lift/door adapter and
   one source-to-target Thunder-model mission with a video and phase timeline.
3. Add the simulator failure-injection matrix, then run repeated
   source-to-target and target-to-source transitions with map/localization
   evidence captured for each phase.
4. Implement the site infrastructure adapter and validate it without robot
   motion.
5. Run one real robot under supervised low-speed conditions, then interrupted
   door, stale state, operator takeover, and relocalization-failure cases.
6. Only then enable Open-RMF cross-floor dispatch; multi-robot bidding comes
   after the single-robot round trip passes repeatedly.
