# Native Control Modes: Functional Ownership

This document assigns each control issue to a product function. The native
endpoint owns exactly one process-level control mode at a time:
`autonomy`, `teleop`, or `teleop_avoid`. Python `CmdVelMux` is not part of any
field command path.

## Functional decomposition

| Function | Owned behaviour | `autonomy` | `teleop` | `teleop_avoid` |
| --- | --- | ---: | ---: | ---: |
| Control authority | Mutually exclusive command family and the single final `rt/nav/cmd_vel` writer | yes | yes | yes |
| Command lifecycle | Request/ACK, cancel, ordinary stop, restart-persistent software emergency stop, explicit clear | yes | yes | yes |
| Command freshness | Validate producer timestamps; expire stale velocity intent and reject replayed ClearEstop | yes | yes | yes |
| Motion context readiness | Gate motion on required localization/cloud/terrain inputs | yes | no | yes |
| Goal completion | Plan, follow, arrive, and align final yaw | yes | no | no |
| Compatibility isolation | Reject legacy goal/path/teleop inputs in product mode | yes | yes | yes |
| Localization/map quality | Long-run localization scale and saved-map match | product gate | not applicable | localization gate only |

## Problem ownership and environment

| Problem | Function owner | MuJoCo | Real robot | Treatment |
| --- | --- | ---: | ---: | --- |
| LiDAR rays hit `lidar2_link` | MuJoCo sensor model | yes | no, not in this form | Keep robot geometry in the reserved collision group and scan environment groups only. |
| Cancel leaves the previous Teleop request resumable | Command lifecycle | yes | yes | Clear path, Teleop request, freshness timestamp, follower/recovery state, then publish zero. |
| Generic stop enters Python or sends a Teleop zero that autonomy rejects | Control authority | yes | yes | Add mode-independent typed native Stop for WS release/CLI control. Preserve the public REST `/api/v1/stop` and MCP emergency-stop contracts as native latched Estop, with explicit `/api/v1/estop/reset`. |
| Emergency stop is not latched | Command lifecycle | yes | yes | Add a software latch that rejects new Goal/Teleop until explicit clear and persists across endpoint service restarts. This does not replace the hardware E-stop. |
| Delayed ClearEstop can release a newer stop | Command freshness | yes | yes; transport delay/replay is possible | Reject invalid, future, or stale ClearEstop source timestamps. Keep the latch if zero publication or persistent-marker removal fails. |
| Teleop age starts at endpoint receive time | Command freshness | yes | yes; DDS delay makes it more likely | Use `NavigationCommandRequest.header.stamp`; reject missing, future, and stale requests. |
| A DDS protocol ACK timeout aborts the client before the typed application ACK is checked | Command lifecycle | yes; WSL2 made it reproducible | yes; a busy endpoint or transport jitter can trigger it | Treat the matching `NavigationCommandAck(request_id, kind)` as the command authority. Reject clock samples above `100 ms` RTT, resample with a typed safe Stop, and retry one Teleop/ClearEstop after a real stale/future application rejection. Endpoint freshness limits remain unchanged. Gateway joystick traffic uses a background latest-value slot; release/stop quiesces it before synchronously confirming zero/Stop. |
| Wall clock steps while a command is in flight | Command freshness | yes; WSL2 stepped about `2.65 s` during acceptance | yes; NTP/PTP correction or a poorly synchronized remote operator can reproduce it | Record endpoint-clock offset and RTT for audit. A request crossing the step is rejected by the endpoint, then the client performs one safe re-sync and sends a traceable retry request id. Do not widen the endpoint's stale/future window. |
| Python `CmdVelMux` exists beside the endpoint | Control authority | visible in graph | yes | Remove it from `endpoint_only` product graphs; acceptance forbids it even when its output is disconnected. |
| Pure Teleop starts SLAM/traversability | Product process ownership | harness can hide it | yes, through deployment dependencies and reboot enablement | Pure Teleop acceptance forbids both processes. Mode switching stops them, disables legacy robot LiDAR/SLAM units, and persists the native per-mode systemd enable/disable set, so reboot cannot silently restore a second stack. |
| Traversability silently degrades when stale | Motion context readiness | yes | yes | `teleop_avoid` and configured autonomy fail closed on stale/missing traversability. |
| Recovery counts endpoint ticks instead of new sensor evidence | Motion context readiness | yes | yes | Track accepted DDS generations for odom, required TF, registered cloud, traversability, and localization health. Every required source must advance before one recovery frame is counted. |
| Localization health is declared but not consumed | Motion context readiness | yes | yes | Subscribe to native health, require healthy state and freshness for motion modes that depend on localization. |
| `teleop_avoid` terrain input is only a binary obstacle projection | Terrain safety | yes | yes | Derive graded `0..100` risk from the rolling registered cloud: local height, step, roughness, and slope; footprint-inflate it and merge it into every fast traversability publication. |
| A cloud is transformed with the latest planar pose instead of its acquisition pose | Spatial alignment | yes; accelerated simulation makes it easy to expose | yes; motion, DDS delay, roll, and pitch make it safety-critical | Buffer full translation/quaternion transforms, sample them at the cloud source timestamp, and reject the cloud when the pose/TF gap exceeds the configured bound. Apply the calibrated sensor offset through the full body rotation. |
| Relocalization changes `map->odom` while old live obstacles remain usable | Spatial alignment | yes; map-tracking correction exposes it | yes; relocalization and loop closure produce the same discontinuity | Treat a correction above `0.50 m / 0.25 rad` as a new map epoch. Publish zero, clear pose/TF/live-obstacle/terrain freshness, reject old-epoch samples, and require new generations through InputGate recovery. |
| Zero/NaN pose quaternion is normalized to identity | Transport validation | malformed-test input | yes; startup/version faults can produce partial samples | Validate finite translation and a finite non-zero quaternion before normalization. Invalid odom/TF does not advance freshness or generation. |
| A malformed `PointCloud2` field layout can be read out of bounds | Transport validation | malformed-test input | yes; mixed producers and version skew can trigger it | Require scalar FLOAT32 `x/y/z`, valid offsets/counts, consistent row/point/data sizes, and supported endianness before reading any point. Reject invalid samples rather than reusing old geometry as fresh. |
| Traversability dynamic clearing is configured but shell expansion disables it | Terrain safety | deployment-only | yes | Pass `LINGTU_TRAVERSABILITY_TERRAIN_CLEAR_DY_OBS` literally from the systemd environment; reject the former `$${...}` expansion in deployment tests. |
| Very small non-zero translations are labelled zero but leak to the driver | Command safety | yes | yes | Apply the deadband to the output, publish an explicit zero, and report `below_min_motion`. |
| Mixed translation/yaw checks only a straight centre corridor | Command safety | yes | yes; close side obstacles are more common in field teleop | Integrate a bounded constant-twist SE(2) trajectory and test the expanded rectangular footprint along it. Straight, lateral, pure-yaw, and curved motion share the same stop/slow and terrain sampling path. |
| Unknown/no-return terrain cells are encoded as free | Negative-obstacle safety | partly; a cliff scene can expose it | yes; drop-offs, water, vegetation occlusion, and limited FoV are more important in the field | The rolling grid starts hard/unknown and only source-time sensor rays mark observed-free before obstacle/terrain risk is overlaid. Observed cells persist for `0.60 s`, and every rolling layer snaps its origin to the same global resolution lattice so cached free cells cannot alias into a neighbouring cell. A 2D ray still cannot prove an unobserved depression between returns, so pit/cliff/no-return scenarios remain mandatory. |
| Point-budget reduction drops an arbitrary live obstacle | Obstacle safety | dense-cloud simulation can expose it | yes; MID-360 scans routinely exceed the endpoint budget | When a live layer exceeds its point budget, retain obstacles nearest to the current source-time sensor origin instead of sampling unordered-map iteration order. |
| Synchronous saved-map ICP starves sensor ingestion under host overload | Localization runtime | observed on WSL2 | possible when maps/scans or compute load exceed the target budget | Periodic map tracking now copies scan/pose input, runs only the standalone relocalizer on a worker, and commits on the estimator thread after job/map/alignment/observation sequence checks. Sensor feed/tick/publish remains non-blocking; manual relocalization keeps synchronous semantics. |
| Rejected LiDAR correction is still inserted into the local map | Localization runtime | produced a `137.5 m` runaway in the free scene | yes; IMU-only drift can poison the rolling map after any guard rejection | `IESKF::update()` reports whether the correction was accepted. Invalid residual, guard, or covariance failures do not call `incrCloudMap()`. The MuJoCo profile also permits bounded observable-subspace corrections for partially-degenerate non-converged iterations; the S100P tuning is unchanged. |
| SLAM reports TRACKING after periodic map tracking has stopped succeeding | Localization health | reproduced: five ICP rejections after three successes | yes | Before first alignment publish `LOCALIZING`; after three consecutive failures or no success for `max(10 s, 3 x period)` publish `DEGRADED` with zero confidence. Endpoint recovery still requires new healthy input generations. Per-scan estimator rejection/velocity/covariance health remains a separate hardening gate. |
| Simulated sensor time falls behind wall time under host load | Acceptance runtime | yes only | no; physical sensors own a hardware clock, although real producers can still drop frames | Keep a `50 ms` simulated-hardware lag budget. When behind, advance every dynamic physics/control tick but explicitly drop intermediate sensor observations; anchored static dropped ticks only advance the static clock. Report catch-up events and IMU/subscan/LiDAR drops instead of silently publishing stale timestamps. |
| A fallen MuJoCo policy is interpreted as an obstacle/terrain decision | Acceptance ownership | yes only | the field analogue belongs to attitude/fall safety, not map attribution | Product evidence requires base `z >= 0.30 m`, absolute roll/pitch at most `45 deg`, with three-frame confirmation. Invalid posture reports `simulation_posture_invalid` and suppresses obstacle/terrain attribution. |
| A Python geometry mirror is treated as MuJoCo product evidence | Acceptance ownership | yes | no | Keep the mirror supplemental. Product acceptance must run MuJoCo sensors, native SLAM, native traversability, the native endpoint, typed DDS command/tap, and the ThunderV4 policy. |
| Goal yaw is transported but discarded | Goal completion | yes | yes | Preserve yaw through planning context and perform a final in-place alignment before reporting arrival. |
| Legacy `rt/nav/global_path` bypasses typed request/ACK and OctoPlanner3D | Compatibility isolation | yes | yes | Disable legacy motion readers by default; keep an explicit diagnostic opt-in only. |
| Long-run trajectory scale is `6.22`; near-field map match is `0.708` | Localization/map quality | measured in simulation evidence | can become drift/map misalignment | Keep this separate from command-chain acceptance and fail the autonomy product-quality gate. |
| systemd mode and reported runtime spec disagree | Product process ownership | usually absent | yes | Treat endpoint status plus effective service environment as authority; validate both during field startup. |

## Mode data flows

### `teleop`

```text
operator -> rt/nav/command/request(Teleop, body, source stamp)
         -> native endpoint freshness + velocity limit
         -> rt/nav/command/ack
         -> rt/nav/cmd_vel
         -> C++ DDS tap -> ThunderV4 policy -> MuJoCo/Brainstem
```

SLAM, traversability, autonomous planning, and Python velocity arbitration are
not allowed in this function.

### `teleop_avoid`

```text
operator Teleop request
  + SLAM odometry / required TF
  + registered cloud
  + localization health
  + traversability
-> native input gate
-> native clamp / obstacle slow-stop / terrain slow-stop
-> rt/nav/cmd_vel
```

Goal and external global-path commands are rejected. Missing or stale required
context produces zero velocity, not a silent downgrade.

The maps in this flow have different owners and must not be conflated:

| Data product | Source | Direct `teleop_avoid` use |
| --- | --- | --- |
| Saved `map.pcd` | mapping session | SLAM localization reference only |
| Saved `octomap.ot` | map package | none; it belongs to autonomy global planning |
| Rolling obstacle layer | current `rt/slam/registered_cloud` | obstacle corridor slow/stop |
| Rolling traversability grid | current registered cloud plus odom/TF | height/step/roughness/slope slow/stop |
| `terrain_map` / `terrain_map_ext` point clouds | TerrainAnalysisCore | diagnostics and autonomy local-planner inputs; teleop safety consumes their graded grid result, not these point-cloud topics directly |

The field `teleop_avoid` terrain policy is intentionally more conservative
than the robot's absolute terrain capability: soft/hard height thresholds are
`0.08/0.20 m`, soft/hard slope thresholds are `12/28 deg`, and the footprint
radius is `0.45 m`. These are near-field operator safety limits, not claims
that the platform cannot traverse the larger limits used by global planning.

### `autonomy`

```text
typed Goal(map pose + yaw)
-> OctoPlanner3D(saved octomap)
-> C++ LocalPlanner(live cloud + traversability)
-> embedded PathFollower
-> final native command safety
-> rt/nav/cmd_vel
-> policy / robot motion
-> final yaw alignment
```

Teleop and external legacy paths are rejected. Stop remains mode-independent.

## Safety naming

The endpoint emergency-stop command is a **software-latched motion inhibit**.
It clears native motion intent and continuously commands zero. It is not a
safety-rated hardware E-stop and cannot claim motor power isolation. The real
robot still requires the independent Brainstem/driver watchdog and physical
emergency-stop path. In the deployed service the latch marker lives at
`/var/lib/lingtu/nav_estop_latched`, so restarting or switching the endpoint
does not release it. Only a fresh typed ClearEstop request may remove the
marker; product mode switching uses ordinary native cancellation instead.
