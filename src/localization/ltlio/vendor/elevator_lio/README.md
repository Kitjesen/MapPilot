# Elevator-LIO source provenance

This directory retains licenses and source correspondence. Production code is
maintained once in [`../../frontend.hpp`](../../frontend.hpp),
[`../../frontend.cpp`](../../frontend.cpp) and [`../../detail/`](../../detail/),
under namespace `lingtu::ltlio` and target `ltlio_core`. The former selected
24-state helpers and separate `native/` frontend are no longer parallel runtime
implementations.

The complete default mapping chain derives from Elevator-LIO revision
[`1d79af77f3d9747ea57ef52a9b01d326a8ec561a`](https://github.com/xiaofan4122/Elevator-LIO/tree/1d79af77f3d9747ea57ef52a9b01d326a8ec561a).
The retained flow includes IMU initialization, the 1000-entry IMU/state history,
interpolation and posterior reintegration, 21-state propagation, ESEKF matching,
adaptive sampling, five-frame initial map, deskew policy, door detection,
waiting observations and stop/exit updates. The numerical baseline uses adapted
original bodies rather than a second independent derivation of their equations.

## Exact source correspondence

All upstream paths refer to the pinned revision. Local paths are relative to
`src/localization/ltlio/`.

| Local file | Upstream path | License/provenance |
| --- | --- | --- |
| `detail/Types.h` | `include/support/type.h` | GPL-2.0-or-later |
| `detail/IMUProcess.{h,cpp}` | `include/estimator/IMUProcess.h`, `src/estimator/IMUProcess.cpp` | Header GPL-2.0-or-later; source repository GPL provenance, original yczhang attribution |
| `detail/ESEKF.{h,cpp}` | `include/estimator/ESEKF.h`, `src/estimator/ESEKF.cpp` | Header GPL-2.0-or-later; source repository GPL provenance, original xiaofan attribution |
| `detail/LidarPipeline.{h,cpp}` | `include/node/LidarPipeline.h`, `src/node/LidarPipeline.cpp` | GPL-2.0-or-later |
| `detail/ElevatorProcess.{h,cpp}` | `include/elevator/ElevatorProcess.h`, `src/elevator/ElevatorProcess.cpp` | MIT |
| `detail/ElevatorSelfExit.{h,cpp}` | `include/elevator/ElevatorSelfExit.h`, `src/elevator/ElevatorSelfExit.cpp` | Header GPL-2.0-or-later; implementation MIT |
| `detail/ikd_Tree.{h,cpp}` | `include/ikd_tree/ikd_Tree.{h,cpp}` | GPL-2.0-or-later; Yixi Cai / hku-mars and KennyWGH notices retained |
| `detail/IkdMap.hpp`, `detail/IkdNearestQuery.hpp` | Same basenames under `include/ikd_tree/` | GPL-2.0-or-later |
| `detail/AdaptiveVoxelPController.hpp` | `include/AdaptiveFilter/AdaptiveVoxelPController.hpp` | MIT |
| `detail/Context.h` | Numerical defaults/math from `support/common_lib` and mapping YAML | Native instance-ownership adapter, GPL provenance |
| `frontend.hpp`, `frontend.cpp` | Native adaptation of IMU timer, sensor intake and flag-callback sequencing | GPL-2.0-or-later native adapter |

Original copyright and file-specific SPDX declarations remain in the source.
[LICENSE-GPL-2.0](LICENSE-GPL-2.0) and [LICENSE-MIT](LICENSE-MIT) retain their
respective license texts. The entire upstream repository is **not MIT licensed**.
The [build legal notice](../../../../../scripts/build/provenance/slam/legal/notices/Elevator-LIO.md)
records the same provenance; unifying the implementation does not change its
license.

## Native ownership and retained local corrections

Each instance owns its Context, IMU history, estimator, map/tree, queues and
detector/controller history. Mutable globals became members. ROS messages,
publishers, visualization, logging and node lifecycle are outside this core.
Dependencies are the existing Eigen, PCL, Threads and optional OpenMP.

The caller serializes feed/process/snapshot/reset; different instances are
independent. The tree owns and joins its rebuild worker. Previously validated
LingTu lifecycle and concurrency fixes were ported into this sole tree:
queued rebuild cancellation, empty rebuild/replay, reader leases, mutation
serialization and lazy-deletion synchronization. Private diagnostic gates are
compiled only into the tree test. The numerical migration baseline retained the
upstream heap/tie rules. The current implementation instead uses one exact
distance/x/y/z ordering for the heap, candidate replacement and voxel
representative selection, and searches boxes at the cutoff distance. Formal
red/green, near-unequal-distance and voxel tests pass; the historical
`stable-selection` ordinary-mode solo and two concurrent full replays agreed
byte-for-byte, while that candidate's old-baseline agreement failed. These are
not repeatability measurements of later numerical corrections. Distance arithmetic and precision remain unchanged;
nearest-search `max_dist` retains the native double precision. Snapshot copying
holds a complete protected lease against background tree replacement and returns
independently owned points.

Additional native corrections include:

- The SO(3) logarithm uses the actual principal axis/angle. The pinned source's
  `3-1e6` trace condition always selected its small-angle branch; known-angle
  tests now exercise the error used by both IESKF update paths.
- IMU pose intervals use one scan-start time origin, including an end exactly
  equal to an IMU sample. A scan starting at the oldest retained sample is
  covered without requiring a nonexistent predecessor. Tests use known motion
  and fixed world landmarks across exact, interpolated and ring-wrap endpoints.
- Calibrated IMU noise densities reach the same propagation matrix. The four
  absent-key defaults retain the upstream coefficients exactly; supplied SI
  densities are squared once, without repeating accelerometer input scaling.
- A posterior committed between two IMU samples also interpolates the retained
  acceleration and gyro reading before reintegration. Moving only its timestamp
  would combine a newer posterior with an older sensor endpoint.
- Transport detection consumes the existing IMU history only through the current
  scan endpoint. A future covering IMU cannot authorize an earlier stop update.
  Missing unconsumed history clears stale continuous-stop evidence; ordinary
  history eviction and IMU propagation keep their existing ownership.
- Small or empty scans clear current sampled/effective-point evidence rather
  than reuse the previous scan's successful match.
- A query before a transport-entry IMU does not borrow that future mode or its
  covariance reset. Posterior repropagation preserves already observed later
  entry edges, without re-entering the old active tail after an exit. Native
  trip counters consume causal IMU observations and actual scan commits.
- Intake queue loss carries the first discarded timestamp to stop detection.
  Continuous stop evidence is reset when a scan reaches that missing interval,
  while scans before the gap retain their valid evidence. This handles LingTu's
  bounded intake queue, which the original unbounded ROS queue does not have.
  Loss preserves the already observed trip phase; a new full variance window
  and stop hold must be collected before stop updates resume. A full detector
  reset here would strand an already stopped trip in the pre-motion phase.
  A fresh waiting-like window can also recover an active trip whose complete
  deceleration was hidden by the gap, without requiring another movement.
- The upstream gyro-step threshold remains 10 rad/s, but its stale-reference
  lock is not retained. Initialization restarts its raw window on a step;
  propagation rejects isolated gyro spikes without replacing acceleration,
  and consecutive consistent raw readings can establish a fresh reference.
- Raw scan-end time survives intake filtering. Initialization keeps the original
  five-map-frame ordering; only subsequent state-estimation scans require retained
  IMU history coverage. Expired scans are counted and skipped, and the oldest valid
  ring-buffer slot is handled without an index of `-1`.
- Actual matching iterations and convergence are reported independently of update
  acceptance. Initialization and early returns do not inherit a previous success.
- The frontend accepts calibrated resource/sampling limits and stationary-window
  settings from LingTu. Ordinary stationary constraints need LiDAR motion evidence
  as well as IMU stability and are disabled in transport mode. LingTu's former
  ordinary-motion vertical zero-velocity prior is removed; it was not part of
  the pinned upstream default mapping chain.
- Native external-pose correction changes the same retained estimator state and
  reintegrates later IMUs. It does not introduce another filter or reuse the retired
  24-state layout.

## Numerical and interface contract

Transport covariance propagation includes the position sensitivity to attitude,
accelerometer bias and enabled gravity at the same second order as transport
acceleration. The pinned source omitted those terms, tilting the bias/transport
null direction and making it spuriously observable at varying IMU steps. The
ordinary branch is retained. The null-direction regression proves consistency;
the truth comparisons show this is not a general accuracy improvement.

Acceleration times explicit `acc_scale` is SI m/s²; gyro is rad/s. Initialization
averages raw acceleration before scaling the mean, preserving upstream arithmetic
order. Propagation scales samples once. The scheduler consumes real observations,
retains the covering IMU and does not invent tail samples. `initialState()` is the
exact initialization result, distinct from the first propagated LiDAR pose.

The error-state order is R/P/V/BW/BA/G/Z/VZ/AZ at offsets
0/3/6/9/12/15/18/19/20. During elevator motion, local position and velocity are
cabin-relative; transport height and velocity contribute to world Z. Exit
absorbs transport displacement/velocity and resets `Z/VZ/AZ`, restoring ordinary
building-map coordinates; no cumulative `Z` state remains between rides.
Fixed body/extrinsic transformations and
output anchoring stay at the backend boundary. Active-elevator deskew remains
the upstream policy. Phase diagnostics describe the pre-LiDAR/stop phase, while
state and counters describe the resulting update.

Point intake retains Livox line/tag and stride semantics; original end time is
independent of filtered returns. Queue capacities remain explicit, drop-oldest
overflow is counted, and reset clears the counts. The upstream ROS queues were
unbounded, so no-drop offline agreement does not prove overloaded scheduling
equivalence.

Fixed extrinsics are supported; online `esti_il: true` is explicitly retired.
The YAML keys `na/ng/nba/nbg` are supported as Kalibr continuous-time SI noise
densities: accelerometer noise, gyro noise, accelerometer bias random walk and
gyro bias random walk. The backend squares each supplied density once before
it reaches the IMU propagation matrix; `acc_scale` is not applied to these SI
densities again. Omitted keys retain the exact upstream `Q` coefficients
`1e-4/1e-5/1e-6/1e-6`. Negative or non-finite values are rejected. This repairs
the migration's previously disconnected calibration path; legacy variance
values must not be described as densities. Units and the equivalent defaults
are specified in the [calibration guide](../../../../../tools/calibration/imu/README.md).

The standalone `Config` defaults and public-dataset profile retain upstream
adaptive sampling, 200-sample initialization and enabled transport detection.
The production backend instead defaults to `elevator_mode: off`; the checked
Go2 and Thunder profiles do not override it. Those field profiles deliberately
use 400 initialization samples, fixed 0.25 m scan sampling, 0.20 m map voxels,
five matching iterations, and ordinary stationary constraints. Consequently,
the public-dataset comparison is not a claim that field configuration is
identical to upstream. These remain configurations of the same core, not a
second frozen algorithm or a selector restoring historical numerical behavior.

## Transport-boundary repair candidate

The retained exit policy, introduced by the historical `upstream-reset` candidate,
follows upstream's explicit position
covariance anchoring/reset approximation ([V-D2](https://arxiv.org/html/2605.24495v1#S5.SS4.SSS2))
and historical floor geometry. It does not reinterpret that approximation as an
upstream implementation bug or promise continuous world covariance across reset.
These are changes to one core. The test, replay and repeatability results below
belong to their named historical candidates. Current build and replay evidence
is reported in the linked audit record; full accuracy/field acceptance is not
established:

- `ElevatorModeEnter` consumes the current closed-like range episode for both
  manual and automatic entry; subsequent automatic entry needs a fresh open-like
  observation after exit. The Campus3 baseline exposed reuse of the manual
  entry's old episode immediately after self-exit.
- Entry resets `Z/VZ/AZ`; exit absorbs `Z` into position and `VZ` into velocity.
  The explicit velocity covariance transform preserves moving manual-exit
  behavior. Position covariance follows the upstream local-map reset policy,
  including reset transport cross-blocks and prior entries.
- The full zero-velocity posterior remains in the estimator. The same-time pose
  correction accumulates in `world_from_odom` (`C`); corrected map output retains
  it while navigation odom uses the continuous frame. This does not freeze map
  height or claim that localization error has disappeared; the following state
  reset still uses the documented upstream covariance approximation.
- Waiting updates retain their IMU request timestamp and apply only at an
  eligible accepted scan commit. A covering right-hand IMU cannot correct the
  frame relation of an earlier scan before that scan's state is updated.
- Exit retains the historical matching tree. Static building patches remain
  separate from active-cabin geometry.

Intermediate replay results rejected the initial Schmidt-style stop-height hold
(`fixed`), exposed a next-frame height correction despite tree reset (`framed`),
and traced a later correction mostly to retained inactive `Z` (`offset-dorm`).
`causal-offset` then failed same-floor terminal consistency despite smoother odom;
`history-dorm` with full position-covariance folding retained a 0.210184 m step.
Both are rejected. The historical `verified` automatic runs retain complete inputs
and timestamps; Office1/Dormitory1 terminal map Z is +0.001208/-0.001987 m, and
Campus3 completes one entry/exit. Map corrections are not suppressed.
The complete record, including exact outputs and retained failed gates, is in
[elevator-native.md](../../../../../docs/research/elevator-native.md) and
`build/ltlio-elevator-fix-20261017/`. These modifications add no dependencies or
robot deployment and do not alter source authorship or license obligations.

## Evidence limits

The numerical and integration corrections from the 2026-10-18 deep audit have
their own source snapshots and red/green evidence in
`build/elevator-deep-audit-20261018/`. Measurements of the named historical
binaries below are not measurements of the corrected source. Current outcomes
and unresolved capability limits are maintained in the
[current record](../../../../../docs/research/elevator-native.md).

This migration does not imply that all optional upstream features are exposed.
Internal `full21`, online-gravity, strong-prior, exit-height ICP and exit-map
rebuilding branches are not public `Config` options. High-frequency IMU odometry
publication, wheel `UpdateByWheel`, and Ouster/Velodyne/XT32-specific conversion
and missing point-time reconstruction are not connected to the native interface.
Upstream door range/hold/cooldown, waiting-ZUPT thresholds/period, adaptive
sampling target/range and optional matching-gate controls also remain internal
defaults rather than a generic passthrough of upstream YAML options.
The external pose prior does not reproduce the upstream wheel measurement model.
ROS lifecycle and saved-map integration retain LingTu's existing owners.

The earlier wording about omitted upstream image fusion overstated the source.
Pinned [ImgProcess](https://github.com/xiaofan4122/Elevator-LIO/blob/1d79af77f3d9747ea57ef52a9b01d326a8ec561a/src/node/LidarPipeline.cpp#L452-L454)
only dispatches `ImgEnd` and returns, and the corresponding
[FSM actions](https://github.com/xiaofan4122/Elevator-LIO/blob/1d79af77f3d9747ea57ef52a9b01d326a8ec561a/src/main.cpp#L43-L44)
are empty. There is no implemented image-measurement fusion there to list as a
missing native capability. See the [capability table](../../../../../docs/research/elevator-native.md#官方迁移范围)
for exact upstream references. Historical short-ride changes made only in the
retired partial integration are not automatically changes in the complete
upstream baseline.

The original offline numerical baseline remains under
`build/elevator-official-20261016/`. Historical dual-core evidence is under
`build/elevator-native-20261017/` and `build/elevator-integration-20261017/`.
The historical single-core migration has its own evidence directory,
`build/ltlio-unified-20261017/`. Its 25-suite pass and strict upstream comparisons
precede the transport-boundary repairs and do not verify the current candidate.
The rejected `causal-offset` matrix and each intermediate failure are retained;
their mechanical passes are not acceptance evidence for the current candidate.
The historical `verified` logs record 27 native suites and 43 Python tests passing.
That binary's ordinary Office1 strict gate failed: original vs repeat differs
by 0.240292 m / 1.371766 degrees, although the repeat matches both old runs exactly.
This establishes that binary's repeatability failure, not its cause or old-binary
non-determinism. Overall acceptance and field readiness remain unestablished.
The earlier two instrumented Office prefixes match the non-failing branch and have
identical traced queries/KNN results; they did not reproduce the anomaly.
Independent reconstructed corner geometry demonstrated equal-distance neighbor
subset/plane sensitivity; those earlier diagnostics alone did not connect it to
the Office failure. The new real-match evidence below supplies a specific link.
The historical immutable `ltlio_replay-stable-selection` also fixes voxel
representative ordering. Its evidence is in `build/ltlio-repeatability-20261017/`:
27/27 native suites pass in 32.86 s, and Office1 ordinary-mode solo plus two
concurrent odom/map outputs are byte-identical under zero timestamp tolerance
and `1e-6 m / 1e-4 degree` comparison. The old-baseline comparison still fails
by 0.240291756 m / 1.37176597 degrees; stable output does not demonstrate higher
accuracy.

The `stable-selection` [Office trace](../../../../../build/ltlio-repeatability-20261017/trace/analysis.json)
matches 2,551 uninstrumented stable-prefix poses exactly. At
`1774195436.2404275`, iteration 2, point index 1, recorded p/q/v and the float
query match the old trace, but the fifth of five neighbors changes at equal
distance squared `0.0016821692697703838`. The old candidate is also observed at
the stable heap boundary. Residual changes from -0.011066377405 to
-0.006347748399 m, with a first same-frame pose difference of 11.737 micrometres
and subsequent trajectory branching. This establishes a real tie/plane/filter
link for that comparison. Complete state/covariance equality was not recorded;
the older earlier approximately 1e-9 m discrepancy and all scheduling conditions
remain outside this proof. The old-baseline failure is retained.

Those historical `stable-selection` transport runs retain 16,378 timestamps, zero drops/errors and
entry/exit counts 2/2, 3/3 and 1/1. Terminal map Z is
+0.000780407/-0.001987045/-0.778067284 m; full odom/map and stop-window metrics
are in `assessment-stable.json`. The 43 Python passes belong only to the prior
`verified` round, not a rerun for `stable-selection` or a later C++ change.
Original failures are retained, and those assessments did not establish field readiness.
The `verified`-round synthetic +/-4.725 m rides end within 0.002816 m but reach 0.306986 m maximum
in-run error; terminal accuracy must not be generalized to the full ride.
The exact record belongs to `build/ltlio-elevator-fix-20261017/`, including
`final-decision.json`, not the migration baseline directory.
Current status belongs to the [migration record](../../../../../docs/research/elevator-native.md).
No such native build or public replay alone establishes robot-mounted precision,
mapping/navigation acceptance or autonomous elevator operation.
