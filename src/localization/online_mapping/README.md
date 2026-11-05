# Online mapping

`online_mapping` owns the keyframe queue, background scheduling, loop verification,
incremental Pose3 graph and corrected preview. It links the real
GTSAM/iSAM2 and PCL libraries; neither library is reimplemented here.
The pinned LIO-SAM source and deliberate adaptations are documented in
[upstream/UPSTREAM.md](upstream/UPSTREAM.md).

## Integration

`slam/cpp/ltlio.cpp` selects keyframes from continuous, uncorrected body
odometry and deskewed body-local clouds. [`worker.cpp`](worker.cpp) schedules one
background job that calls `Backend::append`. `Worker::update()` collects its result
and starts the next queued batch; `enqueue()` only queues input. File reading is
shared through `lingtu_mapping_io`, independently of the worker and Backend.

- Adjacent factors use `raw_previous.between(raw_current)`. Optimized poses
  initialize new states but never replace those raw measurements.
- PCL ICP aligns the current cloud to a historical submap. Its correction
  left-multiplies the current global pose; the loop factor runs from current
  to previous with `corrected_current.between(previous)`.
- Historical radius/time candidates first share one coarse historical cloud
  and tree. At most 256 source points and three 3D rigid SVD steps per initial
  hypothesis group corrected poses into distinct registration modes. Up to 32
  modes are retained and the best two receive full ICP against their original
  local historical windows. A competing valid place with similar fine fitness
  remains `ambiguous_place`; an unresolved coarse tie outside the retained fine
  budget remains pending. Neither a coarse match nor the shared union supplies
  a graph factor. Coarse point counts do not replace fine geometry requirements.
- GTSAM Pose3 noise orders rotation before translation. Loop angular and
  translation variances have separate units. These are tuning values, not
  calibrated LTLIO covariances.
- A geometrically matched loop stays pending until another moved source and
  historical anchor support it. Both raw-odometry baselines must exceed
  `confirm_distance_m` (1.0 m). A full SE(3) cycle through the two ICP
  measurements and the two raw odometry deltas must stay within
  `consistency_m` (0.4 m) and `consistency_rad` (0.1 rad). These are initial
  engineering values, not universal paper thresholds or calibrated noise.
  The four anchor frames must be distinct; an earlier source cannot be recycled
  as the next historical anchor. Repeated stationary scans and adjacent
  historical views below that baseline
  do not count as independent confirmation. Confirmation uses raw odometry,
  so a newly optimized loop cannot serve as its own supporting motion.
  Up to 32 spatially distinct observations are retained. Pending means no
  factor has entered the graph; `confirmation_inconsistent` means a moved
  observation disagreed. Evidence records the supporting frame pair and both
  cycle errors. A stationary revisit must move before it can close a loop.
- This borrows the pairwise SE(3) cycle check from
  [PCM](https://robots.engin.umich.edu/publications/jmangelson-2018a.pdf) and
  [Kimera-RPGO](https://github.com/MIT-SPARK/Kimera-RPGO), without adding a
  maximum-clique solver or claiming full PCM. Independent motion alone cannot
  disambiguate coherent periodic aliases, so the competing-place stage above
  runs before confirmation. Finite sampling, initialization and coarse mode
  grouping can still miss a competing fine minimum; these checks do not certify
  global place uniqueness or zero false loops in an actual garage.
- `Worker` expresses corrected history relative to the latest continuous
  odometry anchor, so a loop does not jump the current body pose.
- Saving uses full recorded patches and corrected keyframe poses, not the
  reduced preview or the voxel-filtered ICP clouds. Raw poses, timestamps,
  trajectory and loop decisions are retained for replay.

The SLAM preview adapter resolves one `map<-odom` rotation and translation per
completed snapshot, then transforms its points in double precision before the
final float conversion. It retains the snapshot timestamp, frame and immutable
cloud ownership. This reduces repeated transform arithmetic; the preview copy
still runs on the SLAM owner thread. It is not a change to graph optimization,
save geometry or navigation. Conversion microbenchmarks must be reported
separately from live sensor-to-output latency.

Before choosing a save snapshot, mapping must finish its queued graph work,
including a possible first loop. The asynchronous entry returns the existing
`online_mapping_busy_retry_save` result while busy; the synchronous entry waits.
This does not change loop acceptance or navigation admission.

The native competing-place regression covers moving 4 m/near-periodic aliases,
0.6/0.8 m repeated aisles with overlapping default target windows, an observable
unique exit, unresolved budget ties, a sparse radius anchor with a usable wider
fine window, and 80 historical plus 20 true revisit frames. The dense revisit
retains fourteen confirmed loops and six pending observations. Existing full
SE(3) confirmation, height-drift correction and solver comparison assertions
remain in place.

In three round-robin development-machine experiments, sharing the coarse union
reduced median revisit-query time from 610.22 ms to 124.47 ms; the nearest-only
baseline took 20.12 ms. This is extra background keyframe work, not a faster LIO
update. Work still grows with eligible history; 32 retained modes is not a bound
on total query time. NX long-run throughput and a real garage remain unqualified.
The experiments and integration logs are in `build/scene-uncertainty-20261020/loops/`.

The incremental solver uses QR factorization. The upstream prior deliberately
leaves global translation weakly constrained; its information weights and the
odometry rotation weights span fourteen orders of magnitude. With the same
positive prior and odometry factors, GTSAM 4.3.0's default Cholesky decomposition
failed at the second keyframe of the native regression. QR solved the unchanged
eight-pose graph with finite estimates and retained the original pose tolerances.
This follows the upstream
[factorization guidance](https://github.com/borglab/gtsam/blob/4.3.0/gtsam/nonlinear/ISAM2Params.h#L248-L260)
for poorly conditioned systems. The prior, covariances, measurements, and loop
acceptance rules are unchanged; QR can require more computation than Cholesky.

## Dependencies and tests

GTSAM is pinned to 4.3.0 by
[`scripts/build/locks/gtsam.json`](../../../scripts/build/locks/gtsam.json).
PCL common/filters/registration remains required. On Linux,
[`scripts/build/prepare_gtsam.sh`](../../../scripts/build/prepare_gtsam.sh)
builds the locked CPU SDK without installing system packages; pass its printed
prefix through `CMAKE_PREFIX_PATH`. An existing 4.3.0 SDK can instead be selected
with `GTSAM_DIR`, pointing to the directory holding `GTSAMConfig.cmake`; PCL uses
`PCL_DIR` in the same way. The Windows dependency manifest is
`scripts/build/vcpkg/slam-windows/vcpkg.json`.

With those development libraries available, run from the repository root:

```sh
cmake -S src/localization/online_mapping -B build/online-mapping-check -DBUILD_TESTING=ON
cmake --build build/online-mapping-check --target online_mapping_backend_test online_mapping_worker_test mapping_io_test -j2
ctest --test-dir build/online-mapping-check --output-on-failure -R 'online_mapping_backend|online_mapping_worker|mapping_io'
```

The full LTLIO integration test `test_ltlio_mock_flow` additionally checks
save completion, original scan retention and graph evidence. Standalone backend
tests do not substitute for that integration test or recorded/field validation.

Audit on 2026-09-29: factor direction and native save wiring were reviewed against
the pinned upstream. The save-before-first-loop-completion gap and missing GTSAM
build preflight were corrected. 42 Python save/SLAM contract tests passed; these
include source-contract checks, not numerical solver execution. The new native
post-loop continuation and save-evidence assertions have NOT run: local WSL
configuration stopped at missing GTSAM. Windows also had no GTSAM CMake package.
No ARM deployment or field validation was performed in this audit.

Subsequent deployment on the same date completed those native tests on NX with
GTSAM 4.0.2, including the new assertions, and installed `.88`. See
[the deployment record](../../../docs/operations/go2-deploy-20260929.md) for the
separate ARM and field no-motion evidence. No supervised motion was performed.

## Offline riSAM comparison (GTSAM 4.3.0)

GTSAM 4.3.0 includes `gtsam::RISAM` in its main `gtsam/sam` library. The
[release notes](https://github.com/borglab/gtsam/releases/tag/4.3.0) and the
[fixed-version implementation](https://github.com/borglab/gtsam/blob/4.3.0/gtsam/sam/RISAM.h)
provide the integration reference. Inclusion in that release is not evidence
that riSAM is qualified on a LingTu robot. The production backend still uses
iSAM2 with its existing Huber loop noise.

riSAM incrementally applies graduated non-convexity to factors explicitly
wrapped with `RISAM::MakeGraduated`. Ordinary factors are treated as known
inliers. A future integration must therefore keep loop-candidate generation
and ICP verification, then identify which accepted measurements may still be
outliers. It must not robustify every odometry edge or bypass geometric loop
verification. riSAM does not repair missing scans, calibration, or incorrect
factor direction.

The [official example](https://github.com/borglab/gtsam/blob/4.3.0/gtsam/sam/tests/testRISAM.cpp)
uses a scale-invariant Geman-McClure loss, a graduation scheduler, and
`ISAM2DoglegLineSearchParams`. The first build-tree run with those exact
Dogleg settings failed on an already-satisfied prior: the GTSAM 4.3.0
[line-search implementation](https://github.com/borglab/gtsam/blob/4.3.0/gtsam/nonlinear/DoglegOptimizerImpl.h#L302-L336)
clamps the search step to the zero Gauss-Newton step and then throws
`Invalid DoglegLineSearch configuration. Would cause infinite search.`
This is a valid convergence case in the fixture, not an invalid configured
search bound. The comparison therefore uses the production Gauss-Newton and
QR settings in both arms. It does not perturb the measurements,
patch upstream GTSAM, or enable the Dogleg line-search strategy in production.

riSAM forces relinearization checking on each update and disables
linearized-factor caching. One external update can run several internal
GNC/iSAM2 iterations; equal external update counts do not imply equal solver
work. This CPU path does not require CUDA or the `gtsam_unstable` library.

The standalone [comparison test](../../../tests/localization/cpp/online_mapping/risam_test.cpp)
uses the production `Config` odometry variances and `loopNoise()` Huber model.
It freezes twelve synthetic Pose3 states with XYZ and attitude variation, raw
relative odometry, initial values, loop measurements, and external update
boundaries. A strong synthetic prior fixes the frame so absolute errors can
be measured; it is not the production prior tuning. The two comparison arms
are:

| Arm | Solver and loop treatment |
| --- | --- |
| `isam2_huber_gauss_newton` | Current Gauss-Newton/QR iSAM2 settings and production Huber loop noise |
| `risam_gm_gauss_newton` | Same Gauss-Newton/QR settings and ordinary factors; only loops use graduated Geman-McClure factors |

Each arm receives odometry only, a correct loop, a false loop, and both loops.
The test reports translation RMS, endpoint error, Gaussian-whitened loop residuals,
riSAM outlier count, external update count, and elapsed time. It checks that
ordinary inlier data is unchanged, a correct loop improves the trajectory,
and a false loop is identified without rejecting the correct one. False-loop
displacement is compared with the Huber arm using the same optimization step.
The comparison still includes riSAM's different loss and GNC schedule.
Unit-test timing is diagnostic, not a performance qualification.

With the GTSAM 4.3.0 development prefix configured:

```sh
cmake --build build/online-mapping-check --target online_mapping_risam_test -j2
ctest --test-dir build/online-mapping-check --output-on-failure -R '^online_mapping_risam$'
```

On 2026-10-15, the target built and its CTest passed against the relocated
CPU SDK in WSL Ubuntu 24.04 x86_64, using GNU C++ 13.3. The original build-tree
Dogleg failure and the successful relocated-SDK Gauss-Newton run are separate
results. After selecting QR for the production backend, both its existing
native regression and this comparison passed with QR against the same SDK.
The comparison metrics were unchanged at the precision shown below.
Translation RMS from the successful run was:

| Scenario | iSAM2 + Huber (m) | riSAM + Geman-McClure (m) |
| --- | --- | --- |
| Odometry only | 0.073753 | 0.073753 |
| Correct loop | 0.066494 | 0.070784 |
| False loop | 0.081447 | 0.073754 |
| Correct and false loops | 0.073448 | 0.071074 |

With only the false loop, endpoint displacement relative to the same arm's
odometry-only estimate was 0.014561 m for Huber and 0.00000219 m for riSAM.
riSAM identified the false loop in both affected cases and retained the correct
loop. Huber achieved lower error with only the correct loop. The fixture's
strong odometry already bounds the Huber false-loop effect; these results do
not establish a general accuracy or runtime advantage for riSAM.

This graph-only comparison does not exercise ICP, LiDAR, live SLAM, map saving,
or field motion. Recorded point-cloud and real-platform validation remain
necessary before changing the production backend. No runtime selection flag
or production backend replacement is added by this test.
