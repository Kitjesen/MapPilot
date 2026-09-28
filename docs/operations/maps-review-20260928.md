# Maps fixes before field validation — 2026-09-28

This change repairs query and save responsibilities without changing planner
collision, ground, unknown-space, or SCAN execution rules.

- List/active-map serialization uses the activation result obtained under its
  existing map lock. Nested health serialization no longer acquires that lock.
- Query summaries read metadata, file presence, and the OctoMap header, without
  loading every PCD/OctoMap/occupancy payload. Actual activation and explicit
  artifact validation retain full payload validation.
- Saved-ray construction reuses the loaded retained points and skips the
  sampled-preview occupancy/support preparation. Ray hit/miss semantics stay
  unchanged.
- Default SaveMap/navigation-package output is the 3D OctoMap. The 2D projection
  is built only on explicit request or as an ESDF/traversability dependency.
  A rebuild removes omitted stale derived artifacts transactionally.
- Snapshot handoff, failed-save rollback, and active-map mutation exclusion
  remain in place. Saved-map activation remains owned by ProductControl.

## Local verification

MSVC Release, native OctoMap enabled, assertions enabled:
`lingtu_maps_store_test`, `lingtu_maps_save_map_test`, `lingtu_maps_ray_test`,
`lingtu_maps_map_activation_test`, and `lingtu_maps_mapd_service_dispatch_test`
passed. Coverage includes valid list/health agreement, full validation rejecting
invalid payloads, optional 2D artifacts, rollback, and active-map exclusion.
`npm --prefix web run build` passed (TypeScript and production bundle).

Existing build warnings remain in mapd engine double-to-float conversion and
mapctl getenv usage; the frontend reports a large Three.js chunk. These are
not introduced by this patch.

At the initial field check, small PC 192.168.66.95 and NX 192.168.123.18 were
reachable; NX still had .82 installed and the queried services were inactive.
These local checks do not establish MuJoCo or field navigation success.
Deployment and field results must be recorded separately after completion.

## Go2/NX deployment and stationary validation

On 2026-09-28 the operator confirmed power, Ethernet, a stationary robot, and
no scans to retain. The full native ABI 11 release
`v2.3.0-go2.20260928.85`, commit
`a303f1548739a9003bc94e0cff15b68ecd445e49`, was installed through the release
installer. ProductControl committed the `real`, `unitree/go2`, `nav` session.
The previous `.82` and `.84` releases remain available.

ARM verification: navigation 490/490, driver 5/5, recording 14/14, Maps' seven
build-script tests plus store/ray tests, and map optimizer 3/3 passed. Four
focused OctoPlanner tests covered planning slices, neighboring support,
walls/gaps, stair fallback, cancellation, and no airborne climbing.
Full-build failures also exposed and repaired missing include directories in
the inspection-command and terrain benchmark tests, a semantic-query fixture
whose two-column gap was bridged by the supported one-cell neighbor fallback,
and a stale LF attribute path after recording shell tests moved directories.

The candidate `903room_v4_maps84` was rebuilt from the original map's
`map.pcd.preclean` and its complete 132-frame ray bundle. It was exported and
imported through Maps, then activated through ProductControl. The original
`903room_v4_5cm_rays` remains unchanged. Candidate builder version is `0.3.1`;
its imported content epoch is `1790596697069`. It contains 310,512 retained
points, 298,774 occupied voxels, and a full `.ot`; it has no optional 2D layer.
Replay statistics are:

| Statistic | Count |
| --- | ---: |
| valid_endpoints | 1,041,159 |
| retained_endpoints | 981,292 |
| dropped_endpoints | 59,867 |
| free_updates | 43,292,204 |
| hit_updates | 811,450 |
| guarded_miss_suppressions | 5,202,166 |

Maps' local list query took 2.84 ms; the live Gateway list took 52 ms and
reported `can_activate=true`, no blockers, and `ACTIVE`. Actual activation
performed full artifact validation. The live localization converged and
reported fresh poses; `/ready` returned ready. Native input gates were ready,
obstacle checks enabled, and SCAN planner/follower speed limits were 0.2 m/s.
The controller clamps planar velocity norm before the per-axis limits.

## Search performance found during field preview

The original fixed 0.3 m case needed 9.11 s of offline planning, exceeding
the 7 s preview deadline. Its snapped goal was one height layer away. At the
configured 30-degree slope, none of the basic 26-neighbor vertical offsets
were legal, yet the planner exhausted about 18,900 basic nodes before trying
the existing stair connections.

The fix filters offsets by existing step/slope limits before map queries and
skips the horizontal-only basic graph when start and goal heights differ.
It changes no collision, support, unknown, slope, or goal-tolerance rule and
adds no timeout/configuration switch. The fixed case fell to 0.71 s with the
same map, start, goal, and options. A new regression checks ascent, descent,
and preservation of the flat-route basic search.

Final `.85` online previews reused the four original goal coordinates:

| Goal relative to the initial observation | Result | Native planning time |
| --- | --- | ---: |
| Forward 0.3 m | Feasible | 767 ms |
| Forward 1.0 m | Feasible | 13 ms |
| Left 1.0 m | `goal_not_reached` | 16 ms |
| Right 1.0 m | Feasible | 308 ms |

The left target snaps approximately 0.262 m away, outside the existing
0.15 m arrival tolerance. The fixed offline right-target case at the earlier
exact start still exceeded 35 s; the later live preview has a newly estimated
start and does not prove that fixed case repaired. Preserve that input for
further search profiling; do not label every direction reachable.

Evidence is retained under `build/go2-release-20260928/` locally and
`/home/unitree/field-release-20260928/` on NX: `field85.json`,
`stationary-previews.json`, `stationary-repeat.json`,
`offline-preview-before.json`, `offline-preview-diagnosis.json`, and build /
installation logs. No movement goal was sent: final native counters show
`goals=0`, and output velocity is zero. This establishes Go2/NX deployment and
stationary checks, not MuJoCo, S100P, or supervised ordinary-channel motion
acceptance. A 0.3 m supervised trial was requested separately.

## Follow-up: directional preview diagnosis (21:07 onward)

The four targets are fixed map coordinates generated from one observed pose;
"left/right 1 m" describes their placement, not commanded sideways movement.
No movement target was issued. A later `.85` stationary check passed both
forward goals but timed out on both side goals after 7 seconds. The earlier
right-goal success therefore does not establish repeatable availability.

The side targets also have map limitations independent of search performance.
At the left target, the raw `map.pcd.preclean`, cleaned cloud and voxelized
cloud all contain **zero** points within 0.30 m horizontally in the expected
floor band `z=[-0.40,-0.25]` m. The nearest raw floor point is 0.3194 m away.
The adjacent OctoMap support columns are unknown. This local absence predates
pruning; it must not be reported as floor points removed by this cleanup.
At the right target, the 0.155 m radius / 0.20 m tall body region contains
21 raw points, 19 cleaned points and 8 voxelized points. Those are map returns,
not proof that the objects are still present in the current scene.

The candidate code now ranks goal snap candidates by distance to the actual
requested coordinate, rather than distance to its rounded grid cell. The
existing total/XY/Z arrival tolerances are passed to candidate selection, so
it does not run A* toward an endpoint that those same tolerances will reject.
Out-of-tolerance or unavailable candidates return the existing snap-exhausted
or outside-map reason with no route. Collision, ground support, unknown-space,
step, slope and arrival tolerance values are unchanged. The existing grid-key
hash was also improved; a hash-only experiment increased search throughput
but did not resolve these failures on its own. Fixing the horizontal lattice
did not resolve them and was not adopted.

ARM candidate tests: 9/9 OctoPlanner CTest cases passed, including actual-point
nearest snapping, no A* for an out-of-tolerance endpoint, an allowed endpoint,
independent terminal bounds, walls, unknown gaps and stair transitions. Replays
of the original three fixed cases and all four 21:07 cases preserved forward
success and returned side-goal failures without timeout, in 0.78–0.85 seconds
including process startup/map loading. This is prompt, accurate failure;
it is **not** successful navigation to either side target.

Evidence: `build/go2-release-20260928/goal-pcd-evidence.json`,
`snap-bounded-results.json`, `selfcheck-previews.json`. This follow-up candidate
was built and tested offline on NX; the running service remains `.85`.
Left-side floor capture and verification of right-side objects remain field
work. No replacement of the active map, service restart or motion was done.
