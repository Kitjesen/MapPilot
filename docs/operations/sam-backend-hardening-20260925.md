# Native SAM candidate changes, 2026-09-25

This implements the six work items in `slam-backend-review-20260925.md`.
Update: deployed as .72 on NX after the operator confirmed stationary/no pending
scans. ProductControl started map/camera, session
product-6e4a410ba59d42089387c07cd1f0c41e. Fresh static observations show TRACKING,
8–10 Hz scan processing, about 198 Hz IMU, ready driver with commanded_zero and
healthy teleop control loop. Navigation/moving-map acceptance remains separate.
Existing 903room_v3 map files were not rewritten. Field evidence:
build/sam-review-20260925/deploy72-evidence.tar.gz.

## Changes and references

1. Historical target members all obey the configured time exclusion. The source
   and recent neighbors cannot enter the target through the +/-25 frame window.
   The first 20 frames of the old v3 replay accepted 19 loops; with this window
   size each of those targets necessarily included its current source. Candidate
   evidence rejects these attempts for insufficient independent historical
   geometry. This follows inspection of the upstream/local window algorithm,
   not a claim that a paper prescribed a particular exclusion interval.
2. Save `sam_loops.json` alongside poses and real timestamps. Frame indices refer
   to the ordered pose files. Each appended frame has a reason, target membership,
   point counts, score, overlap, residual, accepted relative measurement and noise.
   Config is included. Without a complete backend snapshot, map_optimization.json
   explicitly reports evidence_complete=false and evidence_pose_count. A missing
   completed loop is not represented as completed optimization.
3. Candidate radius and ICP correspondence range are separate. An additional
   source overlap gate uses bounded nearest-neighbor distance. Angular uncertainty
   and metric uncertainty have separate units; translation variance is at least
   the configured floor and otherwise uses inlier mean-square residual. This is
   a documented approximation, not a calibrated full covariance or an estimate
   of geometric observability. Fixed odometry variances remain tunable.
4. Use GTSAM block Huber noise on loop BetweenFactors, retaining Gaussian
   continuous-odometry factors and full SE(3). The synthetic false-loop test
   compares the production noise function against ordinary Gaussian noise.
5. `slamd` now links `lingtu_online_mapping`, built from opt/online. It does not
   configure Cargo, the Rust kernel, or lt_pgo. Solver-independent pose and cloud
   readers moved into poses.hpp/cloud_io.cpp without changing their parsing
   behavior. The separate opt build retains supported legacy offline tools.
6. The adapter releases its duplicate full cloud after conversion; the backend
   retains a full cloud for preview and a cached filtered cloud for ICP. Submaps
   always transform cached local clouds using current optimized poses. Preview
   invalidation also includes a changed output anchor. Record worker/preview
   durations, cloud capacity bytes and queue high-water. Replay emits p95/max
   frame latency and Linux peak RSS. No limit was increased to conceal pressure.

Primary references:
- LIO-SAM pinned mapping implementation:
  https://github.com/TixiaoShan/LIO-SAM/blob/0be1fbe6275fb8366d5b800af4fc8c76a885c869/src/mapOptmization.cpp
- LIO-SAM paper: https://arxiv.org/abs/2007.00258
- iSAM2 paper: https://www.cs.cmu.edu/~kaess/pub/Kaess12ijrr.pdf
- PCL fitness definition (mean squared point distance, not pose covariance):
  https://pointclouds.org/documentation/classpcl_1_1_registration.html
- GTSAM robust models: https://gtsam.org/2019/09/20/robust-noise-model.html
- Switchable Constraints: https://nikosuenderhauf.github.io/assets/papers/IROS12-switchableConstraints.pdf
- Kimera-RPGO: https://github.com/MIT-SPARK/Kimera-RPGO

Huber is not switchable constraints or PCM. Those references explain alternatives;
no extra solver/dependency was added. Robust loss does not certify loop identity.

## Configuration

Optional `sam` section in the Fast-LIO YAML accepts these resolved defaults:

```yaml
sam:
  radius_m: 15.0
  min_time_s: 30.0
  submap_half_window: 25
  voxel_m: 0.4
  max_fitness: 0.3
  correspondence_m: 1.0
  inlier_distance_m: 0.4
  min_overlap: 0.5
  translation_sigma_m: 0.1
  rotation_sigma_rad: 0.05
  huber_k: 1.345
  odom_variance: [0.000001, 0.000001, 0.000001, 0.0001, 0.0001, 0.0001]
```

Variance order is rotation XYZ then translation XYZ. The angle values use radians.
The values are candidate tuning defaults checked on a room replay, not a completed
Go2 sensor calibration. Cached local downsampling changes voxel grouping versus
the original implementation and is explicitly recorded in upstream/UPSTREAM.md.

## Validation scope

ARM focused tests cover sparse history with a long pause, genuine revisit,
low overlap despite permissive fitness, a false-loop robust/Gaussian comparison,
odometry parity without loops including real tilt/elevation, async queue/reset,
continuous output anchoring, and evidence/performance fields.

The v3 replay uses actual raw poses, patches and recorded timestamps. It writes
new reports and corrected pose files outside the saved map. If sam_loops.json is
present, replay reads its resolved configuration through the same parser as the
field YAML. Older captures such as v3 use the candidate defaults, written into
the new evidence report. The solver itself does not depend on YAML; the field
and replay boundaries reuse the project's existing yaml-cpp dependency.

Large-map limits and live sensor/teleop concurrency still require field-scale
acceptance. A 138-frame room replay does not certify the full 3000-frame capacity.

## Measured ARM results

All nine distinct targeted checks passed on NX: native backend, online mapping,
Fast-LIO mock save flow, native map save, four affected legacy offline tests, and
the build-script argument test. Configuration round-trip is part of online mapping.
The new standalone online CMake cache has no Cargo/PGO entries; slamd's link line
contains no pose_graph_opt. Both slamd and mapd candidate binaries built.

The same 138-frame v3 recording, with no competing builds during timing:

| Metric | .71 baseline | Candidate, Release |
| --- | ---: | ---: |
| Completed frames | 138 | 138 |
| Accepted loops | 115 | 77 |
| Optimization failures | 0 | 0 |
| Total replay seconds | 24.1722 | 16.6761 |
| Maximum frame milliseconds | 384.964 | 331.953 |
| Peak process RSS, KiB | 368876 | 153376 |

Candidate p95 frame latency: 291.285 ms; maximum preview/snapshot time: 298.932 ms.
Cloud capacity accounting peaks at 40,220,672 bytes (not total process memory).
Baseline RSS was measured by Python resource.getrusage(RUSAGE_CHILDREN); candidate
RSS by getrusage(RUSAGE_SELF). These are standalone replay process measurements,
not complete live slamd memory/CPU measurements.

Candidate decisions: 77 accepted, 41 fitness rejected, 19 insufficient historical
geometry, 1 no candidate. No target violated temporal exclusion. Minimum overlap
among accepted candidates was 0.7549. Do not label all 38 fewer accepted loops as
proven false associations: several algorithm changes affect their acceptance.

Raw-to-candidate maximum keyframe translation change is 24.78 mm, maximum height
change 11.31 mm, and maximum rotation change 0.1095 degrees. Endpoint height delta
is 24.80 mm; this is not a ground-truth error, particularly with reported lifting
or stairs. These figures establish no recurrence of the previous large backend
height deformation on this recording, not improved absolute SLAM accuracy.

The synthetic wrong-loop comparison reduces endpoint displacement from 4.16667 m
(Gaussian) to 0.592772 m (Huber). A residual remains: robust loss limits damage,
it does not prove rejection of a wrong place match.

The earlier candidate.json run lacked Release optimization and is not used for
performance comparison. Final metrics are in release.json. Build and replay
evidence is archived under build/sam-review-20260925. No service was switched,
no robot movement was commanded, and the saved v3 map was not replaced.
