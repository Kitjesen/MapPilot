# Fast-LIO online mapping migration and acceptance

This implements the frontend/backend separation described by
[FAST_LIO_SLAM](https://github.com/gisbi-kim/FAST_LIO_SLAM): continuous LIO,
keyframe clouds, background place verification and pose-graph optimization,
then reconstruction from corrected keyframe poses. LingTu keeps its existing
ROS-free Fast-LIO2, geometric verifier and native Rust/C++ solver. No GTSAM,
ROS, CUDA or additional dependency is introduced.

## Migration sequence and ownership

| Step | Implementation | Acceptance |
| --- | --- | --- |
| Keyframe feed | `fastlio.cpp` sends accepted body-local scans with the corresponding raw body pose to `OnlineMapping`. | Native backend fixture reaches a nonempty global snapshot; odometry stays unchanged. |
| Online constraints | Memory and saved PCD inputs share `loop_constraints.cpp`. Adjacent registration and descriptor candidates undergo the same geometry, information and consensus gates. | Memory/file registration parity; cloud-driven revisit test. |
| Background correction | One job handles registration, loop verification, sparse batch PGO and reconstruction. New keyframes queue while it runs. | Queue bounds, epoch reset, measured loop correction, no fabricated loop. |
| Whole-map publication | `/slam/cumulative_map_cloud` is a native DDS whole-snapshot preview. Atomic cloud plus metadata snapshots support Host/Web. | Native DDS delivery; mismatched epochs/timestamps rejected by the preview reader. |
| Web | Mapping has Whole map / Local projection / Local map views. `/api/v1/map/global/points` serves native preview data without accumulating in Python. | HTTP-only cloud hook, reset handling, full bounds retained after sampling, production build. |
| Save | After online correction, reconstruct `map.pcd` from full-resolution body patches and the matching corrected poses. Apply the same corrections to `trajectory.txt` and `poses.txt`. | Patch identity/coverage gate; complete save/reload replay still required on field recordings. |
| Field acceptance | Build ARM, deploy slamd and Host/Web together, record a supervised closed walk and save/reload. | Not completed by local C++ tests. |

## Coordinate contract

Fast-LIO's continuous `T_odom_body` and navigation's `T_map_odom` are not
overwritten by an online mapping correction. Optimized historical poses are
expressed relative to the latest corrected anchor, converted back into current
odometry coordinates, and transformed by the runtime's existing map/odom
transform for display. Thus the newest robot pose and current scan remain
aligned while historical walls move into alignment. The snapshot records the
global/odom correction separately inside the backend.

No corrected preview is fed into collision checking or used as a motion
authorization. `mapd`, traversability and native navigation retain their
existing ownership. Navigation against the saved corrected map still enters
through ProductControl and saved-map localization.

## Data and quality behavior

- Full body-local patches remain the save source. The browser preview is
  sampled and must not be saved as the high-resolution map.
- A geometrically weak keyframe remains in the accumulated preview but does
  not add an invented pose-graph factor. Registration tries the next frame
  against the last geometrically accepted anchor. Corrections for intervening
  frames follow the preceding accepted anchor.
- A candidate must pass descriptor ambiguity, forward/reverse registration,
  residual, observability and multi-frame consensus checks. Nearby odometry
  poses alone do not establish a loop.
- Only completed submaps are cached for verification, inside one append-only
  epoch. Recent cache entries are pruned as the candidate window advances.
  Each new loop solve starts from the last accepted corrected prefix and has
  a 60-iteration budget. A failed solve rolls back its new loop constraints
  and preserves the previous correction; it does not contaminate later solves.
- Limits: 3000 input keyframes, 4 queued frames, 5000 retained preview/registration
  points per keyframe, 200000 whole-map preview points, 0.12 m final preview
  voxel size. Fast-LIO's existing keyframe gates are 1 s and 0.20 m or 5 degrees.
  These are computational bounds, not measured ARM performance guarantees.
- Default loop recall is spatially bounded: 6 m XY, 4 m Z, at least 20 accepted
  graph frames and 8 m path separation. It is not arbitrary-location global
  relocalization. A large odometry error beyond the candidate radius may miss
  the true loop.
- Queue overflow and capacity exhaustion increment `dropped_frames`. Corrected
  saving refuses an incomplete or mismatched patch prefix. An active correction
  returns `online_mapping_busy_retry_save` rather than mixing revisions.
- The solver is sparse **batch** optimization, not iSAM2. Large systems never
  fall back to allocating a dense matrix after sparse factorization failure.
- Whole-map metadata includes epoch, revision, keyframe counts, rejected/dropped
  frames, loop count, completed optimizations, busy state and point count.
  Web replaces snapshots. It does not blend old and new corrected geometry.

## Verification on 2026-09-16

Local synthetic point-cloud revisit: 13 frames, 5 committed loop constraints,
3 successful optimizations. Injected X drift was 0.24 m; the estimated final
global/odom X correction was -0.240897 m. Two intermediate solves exhausted
their budget and were rolled back; the final solve converged. A separate
one-iteration fixture verifies that failed optimization commits neither new
loops nor map correction. The current odometry anchor stayed
continuous. Fixture distances were shortened for the small synthetic scene;
production geometry/consensus thresholds were not relaxed.

Offline sequential replay of the real recording
`build/go2-live-validation-20260913/go2_expanded_20260914_0350`:
278 input patches, 235 geometrically registered graph frames, 43 rejected
registrations, 10 committed loop factors, 6 successful optimizations and zero
failed optimizations with the 60-iteration budget. Preview contains 162153
points. Elapsed time was 144.429 s; the slowest frame took 3965.69 ms on this
Windows desktop. This replay waits for each job; it does **not** prove paced
real-time throughput or ARM behavior. The source recording was not modified.

The last accepted anchor was index 234. Audited registrations from it to
indices 235, 236, 240 and 277 failed `insufficient_planar_correspondences`:
planar correspondence ratios were approximately 0.264, 0.223, 0.216 and 0.211.
The trailing 43 patches remain visible, with preceding-anchor correction,
but do not have independently verified graph registration. They must not be
described as fully corrected geometry. Web displays a partial-correction
notice. Resolving this coverage gap requires further registration/data work,
not accepting an unverified edge or deleting those patches.

Passed locally: online mapping and constraint assembly C++ tests, native
Fast-LIO odometry/global-map integration, cumulative-map DDS publication,
26 pose-graph Rust unit tests, 4 Gateway snapshot tests, 13 cloud decoder
tests, 6 cloud transport tests, Product/topic compilation contracts and the
Web TypeScript/production build. Existing Eigen deprecation/Windows encoding
warnings and the Web chunk-size warning remain. No browser visual acceptance
or robot installation is claimed.

Native Fast-LIO input/output integration is exercised using its supported
odometry-prior fixture, alongside existing non-bypass filter tests. This is not
a field recording or a simulation acceptance run. Windows DDS tests require
the configured CycloneDDS SDK `bin` directory on PATH.

Remaining field work: ARM latency/memory at long-map scale, same-place revisit
with real MID-360 data, saved cloud/trajectory/bundle alignment, corrected map
reload/localization and supervised navigation. Direct NX SSH at
192.168.123.18 timed out during this change; no robot-side installation or
movement is claimed.
