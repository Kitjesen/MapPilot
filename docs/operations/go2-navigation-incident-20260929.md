# Go2 navigation failure, 2026-09-29

## Scope and field clarification

The operator reported repeated planning failures and confirmed that the robot
was manually lifted back afterwards. The approximately 0.9 m difference between
the recorded failure pose and the subsequent pose is therefore not evidence of
localization drift or autonomous progress. Nonzero command counters likewise do
not establish actual robot displacement.

No motion goal or service restart was issued during this investigation.

## Running deployment

- Release: `v2.3.0-go2.20260929.88`.
- Source: `bcc0eea8d5979fda91a74244c30a1a41042d3b99`.
- Product session: `product-45ebc813a5734f99a127c4a696953436`.
- Map: `903room_v4_maps84`, epoch `1790596697069`.
- Map builder: `0.3.1`, 0.05 m, saved rays, full log odds.
- Global radius: 0.155 m; support height 0.35 m, tolerance 0.05 m;
  adjacent support radius one voxel; strict direct support disabled.

## Reproduced global failure

The recorded start `[0.702181, 0.111428, 0.018865]` and unchanged goal
`[4.928530, -3.332038, 0.036993]` were replayed through the deployed-source ARM
headless planner against the active saved OctoMap, without a live obstacle
overlay. Options were supplied through its `options` object and checked in the
returned constraints.

The replay returned `empty_path`, matching the incident. Basic search expanded
nine nodes; stair fallback brought the accumulated iteration count to 36 and
exhausted its open set. This is not an iteration timeout, control-ownership
failure, or a Gateway goal-delivery failure. The runtime reported healthy
localization and an input gate of `ready` at capture.

Direct voxel queries at the recorded body height showed the start passing but
its surrounding floor support disconnected. Some nearby columns have an
observed adjacent surface but are rejected by the existing explicit-free/drop
rule; others have no adjacent occupied support at all. Thus removing only the
explicit-free check would not establish a continuous supported route.

## Saved source observations

Transforms from `poses.txt` were applied to every saved patch, using the same
quaternion order as `VisitSavedScans`. Within 0.15 m horizontally of the failed
start and ground Z band `[-0.38, -0.27]`, there is exactly one saved endpoint,
in voxel `[14, 2, -7]` at 0.05 m resolution:

`[0.727714, 0.114238, -0.309756]`.

The next nearest ground observation is approximately 0.169 m from the start.
The same sparse pattern exists in `map.pcd.preclean` and `map.pcd`. The missing
measurements in this immediate area therefore precede the current clean/build
step. This does not establish whether sensor visibility or earlier scan
selection/downsampling caused the sparse saved observations.

Counts over a larger crop differ between preclean and final PCD; those counts
alone do not establish ground deletion, because voxel sampling and intermediate
processing also change point counts.

## Separate SCAN failure

The preserved SCAN snapshot (sequence 8) reports `collision_restart_limit` at
trajectory position `[0.899804, 0.359218, 0.021634]`, time 0.93 s. Its collision
volume contains occupied cells at body height near that position. This proves
the collision query had occupied input, not that each cell represents a current
physical obstacle. That requires comparison with the originating observations
and the scene. Global replay success alone would not close this local failure.

## Remaining work and acceptance

### Follow-up map-to-planner audit, 2026-09-29

A fresh read-only connection confirmed the same `.88` release, Product session,
active map, and content epoch. The deployed-source ARM headless planner again
returned `empty_path` for the exact incident request. As a diagnostic control,
the same map, goal, and options produced a five-point route from the current
pose about 0.71 m away; no goal was dispatched. The failure is local to the
incident start's support connectivity, not a globally unusable map.

All 132 saved patches contain 1,041,159 points. Their pose-transformed assembly
matches `map.pcd.preclean` within 2.38e-7 m. In the incident ground band and a
0.50 m radius around the failed start, cleanup removes zero ground points. The
single point within 0.15 m survives all save stages. This rules out patch-pose
assembly and persistent cleanup as the source of this particular hole.

Twenty-five matched current raw/registered scan pairs also contain no ground
returns within 0.50 m of the old failed start, including a broader low-height
band. This establishes current viewpoint coverage, not the original acquisition
history. Saved patches bypass the 0.25 m estimator voxel filter, but do pass
stride/range/tag filtering and time-plus-motion keyframe selection. The saved
bundle lacks original full-rate raw scans, so their historical contributions
remain unresolved.

The audit also confirmed that the live `map_cloud` snapshot is labeled `map`
while numerically using Fast-LIO's odom coordinates. It is not an input to the
saved-ray OctoMap build. Comparisons used registered body clouds transformed
by the native `map_scan_pose` instead. This separate frame-label defect was
recorded without changing production code.

Detailed report, map figures, captured data, and fresh replay output are under
the local ignored `artifacts/map-planning-20260929/` (`REPORT.md`,
`analysis-saved-map.json`, `replay.json`, and `live-frame-check.json`).

### Next controlled comparison

1. Obtain additional ground observations around the failed location from
   different sensor viewpoints, preserving the existing map as a baseline.
   Capture raw scans, filtered/undistorted scans, and selected keyframes together
   to distinguish visibility from observation retention. Inspect saved patch
   density before building a separate candidate map.
2. Repeat the exact recorded start/goal and configuration on that candidate;
   do not change test points or disable support checks to claim improvement.
3. Resolve the separate SCAN collision against its source observations before
   claiming motion readiness. Validate no-motion planning first, then supervised
   motion under the existing motion owner.

No algorithm/configuration change or navigation-success claim is made by this
investigation. Production services and the active map were left unchanged.

## Evidence locations

Local ignored evidence: `build/go2-release-20260929/`, including
`stopped-after-goal.json`, `scan-failure-after-goal.json`, `replan-request.json`,
`support-probe.csv`, and the standalone diagnostic sources.
Robot-side evidence: `/home/unitree/field-release-20260929/`, including the same
captures, `replan-result.json`, `nav-incident.log`, and floor probes. These are
diagnostic artifacts, not runtime dependencies.
