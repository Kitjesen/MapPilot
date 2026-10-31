# LTLIO

LTLIO is LingTu's single native LiDAR-inertial frontend. Its public C++ interface
is [`frontend.hpp`](frontend.hpp), its implementation is
[`frontend.cpp`](frontend.cpp), and the estimation components live in
[`detail/`](detail/). The namespace is `lingtu::ltlio` and the only algorithm
library target is `ltlio_core`.

[`../slam/cpp/ltlio.cpp`](../slam/cpp/ltlio.cpp) connects this frontend to the
existing `ISlamBackend` and `slamd`. The public backend remains `ltlio`, enabled
by `LINGTU_SLAM_LTLIO_BACKEND`. There is no `estimator` selector or second
production filter. DDS, body/map transforms, SAM, map saving/loading and saved-map
relocalization retain their existing owners.

**Probability-plane integration (2026-10-21):** the single frontend now retains
local plane support and propagates joint normal/center uncertainty inside its
existing KD map. Query association also considers pose uncertainty; the update
uses sensor plus plane uncertainty without adding query-state covariance a
second time. Both pose-only and full-state updates use the same physical
measurement variance. The existing `lidar_cov_inv` supplies the isotropic sensor
variance; no extra noise configuration or alternate core was introduced.

The complete GEODE, Slippage, Corridor02 and Office1 comparisons pass the
original aggregate-error/coverage gates, not a per-axis no-regression gate.
Corridor02 total RMSE improves 1.605600 -> 1.551559 m, but height RMSE worsens
0.523353 -> 0.739513 m and maximum height error 1.192168 -> 1.933381 m. This remains
an open regression of the default comparison baseline. GEODE/Corridor use
qualified reference trajectories, Slippage uses optical truth, and Office1
has no independent height truth. The first-stage integration build passed 15/15
native CTest suites, including Go2/Thunder profiles, save/relocalization,
elevator persistence and KD-tree concurrency. Its 134-pose integration prefix
matches the clean candidate's map/odom and non-timing status/events exactly. Exact
scores, scope, failed candidates and commands have one entry in the
[experiment record](../../../build/prob-plane-20261021/README.md).

This is a first-order marginal uncertainty model: map points sharing a pose
and residuals sharing a plane are not represented by a full correlated-map
posterior. Support is bounded to 100 unique coordinates per node. Added CPU and
memory costs remain material; these Linux x86_64 offline results do not establish
NX real-time performance or field readiness. Point-to-point hybrid constraints
are not part of this accepted first-stage core.

**Optional GenZ-inspired mechanisms (2026-10-21):** the same frontend now also
supports scale-aware sampling, probabilistic point-to-point fallback and pruned
correspondence search. Top-level sensor YAML keys `scale_aware_sampling_enabled`,
`hybrid_enabled` and `correspondence_pruning_enabled` select them independently;
all default to `false`; enabled v1 and v2 comparisons have been rejected. They
do not select a different estimator or change IMU, elevator, calibration or health thresholds.
With scale-aware sampling enabled, `scan_resolution` is the initial leaf size;
otherwise its existing fixed/adaptive meaning is retained.

The initial v1 combination is rejected for default use: GEODE RMSE increases
from 0.292340 m to 0.852681 m and Corridor02 loses tracking after about 663 s of its
published trajectory, while Office1 also loses tracking after 76.50 s. Its full
coverage is not preserved. The isolated v2 point-candidate-budget repair still
regresses GEODE: hybrid-only 0.364164 m and all-three 0.457008 m against 0.292340 m.
v2 has no Slippage/Corridor/Office/garage result. Its three-file patch remains
isolated and is not approved for merging or enabling. Shared source still
contains v1's 16 integration files with all three switches off. The earlier
`prob-plane-20261021/hybrid` is a distinct rejected conservative KD adaptation;
its results must not be substituted for either fusion version. v2 changes both
target geometry/covariance and the candidate-count denominator in discretization
variance; its gain over v1 does not isolate correspondence from information weight.
Recorded point/plane counts are row counts, not precision or information shares.

Sampling uses a five-frame median-range window, bounded point targets and PD
voxel control. It retains actual returns and their fields; the estimation cloud
is a subset of the denser mapping cloud. This differs from the author's voxel
centroids. Hybrid matching prefers a valid probability plane and otherwise uses
eligible nonplanar/immature/terminal map points, with a scalar distance residual,
propagated target/sensor uncertainty and discretization variance. Rejected planar
cells do not supply fallback points in this adaptation. The point gate follows
the last accepted pose correction, with `hybrid_point_initial_threshold_m=0.5`
and `hybrid_point_weight=0.05`. AABB pruning preserves the exhaustive candidate
choice and pre-pruning uncertainty statistics. These are adaptations of pinned
GenZ-LIO revision `bf444f7a747ed3da8548b56b49c14f207e8d49bb`, not a claim of
identical author trajectories.

The frozen integration passes 16 native CTest suites, including 14 hybrid cases
and actual sampling insertion/rejection/reset checks. Its 134-pose all-off prefix
has byte-identical map/odom trajectories and equal pre-existing status/event
fields. Replay diagnostics expose plane/point matches, mapping/sampled counts,
leaf size and scene scale. Accuracy, coverage and runtime evidence are maintained
in the single [fusion experiment record](../../../build/genz-fusion-20261021/README.md);
passing component tests does not establish field readiness or improved accuracy.

The bounded Corridor diagnosis reproduces the original 8038-sample scores with
one whole-sequence SE(3) fit per trajectory. The approximately 31 cm initial
aligned Z difference is not startup motion: both original trajectories stay
within 1 cm for the first 60 s. Relative shape/attitude differences then grow;
130--240 s is the first targeted observation window, and the later 660--720 s
amplification is not established as the initiating cause. A retained-geometry
C++ fixture also confirms that one root can approximate two nearby horizontal
surfaces as a sloping plane, creating a roughly 2 cm accepted query residual
where the old 5NN fit has zero residual. This is a demonstrated model allowance,
not yet the cause of the real recording's height regression. Both original-start
241 s read-only traces now reproduce their saved 2390-pose map/odom prefixes
byte for byte. After the actual output-frame conversion, the 130--240 s net
relative height increment is -0.175532 m: prediction contributes -0.179477 m,
while instantaneous LiDAR position correction contributes +0.003945 m. Prediction
inherits earlier LiDAR attitude/velocity/bias feedback, so this is not proof of
an independent IMU propagation bug. Paired actual-row plane/5NN probes show
different normals but mixed correction signs; they do not justify weakening
LiDAR, restoring 5NN globally or promoting a new model correction. See the
[bounded diagnostic record](../../../build/corridor-height-20261021/README.md).
Total error, height, coverage, LOST, transport continuity and compute cost remain
separate decisions. No protection thresholds or production feature defaults are
relaxed for a candidate.

**Scene adaptation candidate (2026-10-19):** the real matched point-to-plane
Jacobian now supplies LiDAR-only geometry diagnostics, before adding the IMU
prior. `Result.geometry` reports availability, six principal directions,
eigenvalues, weighted support fractions and observable/partial/unobservable
categories. Both halves of each direction use IMU axes: the first three are
rotation increments multiplied by `rotation_scale_m`, the last three are
translation increments. Translation Jacobian columns therefore transform as
`J_world * R_world_from_imu`. The scale is the weighted RMS rotation/translation
Jacobian energy ratio; repeated samples do not change the mean information.
These normalized values are diagnostics, not a calibrated pose covariance.

The initial category boundaries use relative information `1e-4`/`0.02` and
directional support `0.01`/`0.1` (alignment within 80 degrees). They are local
engineering thresholds, not a complete X-ICP reproduction or field tuning.
The current implementation is diagnostic only. A first direction-aware
information-reduction candidate passed isolated matrix tests, but amplified
longitudinal error after a 0.06 m lateral pose injection in a synthetic inclined
corridor (about 2.683 m versus 0.128 m in the unchanged update). It was not
promoted. Its source and evidence are retained in
`build/slam-scene-20261019/d2-candidate-source` and `d2-decision.json`.
Full three-dimensional inertial prediction, geometric correction and state
cross-covariance remain active. No world-height or vertical-zero-velocity
observation is added. D2 remains unfinished.

Native status publishes this result in `ltlio_degeneracy`. Unavailable geometry
uses JSON null for the spectral values and direction arrays; a rank-deficient
condition number is also null. `update_applied` means that an **accepted** scan
used direction-aware information reduction; it is currently always false.
It does not mean merely that a normal LiDAR update ran. The Host and existing SLAM panel preserve that
distinction. Current validation and pending scene acceptance live in the
[implementation plan](../../../docs/design/slam-scene-adaptation.md).

**Current truth comparison (2026-10-18):** transport propagation now includes the
position sensitivity to bias/attitude/gravity at the same second order as its
transport-acceleration term. A varying-step null-direction regression fails
before and passes after this repair. The ordinary-mode branch is unchanged.
The complete native build and 28/28 regression suites pass. Eight analytical
sensor cases and a real IILABS3D optical-GT sequence have been compared against
the pinned official numerical pipeline under shared inputs/calibration. This
repair is not an overall accuracy improvement: the four transport cases retain
mixed submillimeter changes and higher height RMSE than the official baseline.
Four ordinary synthetic cases and real Slippage retain byte-identical map/odom
trajectories before and after the repair. Waiting-time and old tie-policy
ablations remain experimental; they were not promoted into production.
See [the current truth record](../../../docs/research/elevator-native.md).

**Preceding repairs (2026-10-18):** the single 21-state frontend includes corrections
to the SO(3) error logarithm, IMU deskew interval endpoints and calibrated process
noise consumption. Backend map replacement and runtime health publication are
also corrected. A follow-up investigation repairs IMU readings at interpolated
posterior times and makes transport decisions consume only scan-time evidence.
The latest correction preserves entry events across interpolated scan commits,
counts trips from causal observations, and clears stop evidence at the first
lost intake sample. The native build succeeds and all 28 regression suites
ultimately pass (27 initially; the new suite passes after correcting its test
input timing, with the first failure retained). Office1, Dormitory1, Campus3 and
ordinary-mode Office1 replay odom/map outputs are byte-identical to the previous
combined candidate; a second Dormitory1 run is also byte-identical. Maximum map
height steps remain 0.308938/0.107417/0.356184 m. These boundary repairs establish
correctness on the new regressions and preservation on those recordings, not
a new trajectory-accuracy improvement. The earlier numerical corrections had
mixed continuity results, retained in the audit record.
This source is not field-deployed. The authoritative current
results are in [the audit record](../../../docs/research/elevator-native.md).
The performance and trajectory-equivalence measurements below belong to the
retained historical binaries, not automatically to the current source.

**Historical performance implementation:** `ltlio_replay-query-storage`. The rejected
query-scratch candidate kept the entire heap in thread-local storage; the second
candidate retains only a `vector<PointType_CMP>` backing array there and creates
`MANUAL_HEAP(rawptr, capacity)` on the stack for each query. Downsample scratch
capacity reuse with `clear()` remains. Point order, distances, selection rules,
reader protection, input quality and estimator mathematics are unchanged.
`query-lease` is this round's comparison baseline. Query-storage's own tree suite
passed in **1.40 s**, followed by **27/27 native suites
in 39.98 s**; see [its regression log](../../../build/ltlio-performance-buffer-20261018/tests-storage.log).
The immutable binary is `ltlio_replay-query-storage`. All four assessments are
complete, with identical old/new odom/map bytes, non-tick states/events and
non-runtime summaries. They cover the three sequences' 16,378 poses plus a second
Campus3 run's 6,838 poses, with zero drops/errors and unchanged entries/exits
2/2, 3/3 and 1/1. Campus3 forward/reverse pose files also agree across runs in all
four [cross-order comparisons](../../../build/ltlio-performance-buffer-20261018/benchmark/storage-repeatability.json).

| Complete run / actual order | query-lease / query-storage elapsed (s) | Old / new total CPU (s) | Old / new native tick p95 (ms) | Same-batch official elapsed (s) |
| --- | ---: | ---: | ---: | ---: |
| Campus3 forward / old → new → official | 135.580702 / 113.656046 | 201.26 / 174.63 | 36.858 / **37.981** | 141.037534 |
| Office1 reverse / new → old → official | 103.192967 / 74.210044 | 138.45 / 108.15 | 40.930 / 31.353 | 76.361758 |
| Dormitory1 reverse / new → old → official | 60.987263 / 50.000522 | 86.57 / 75.17 | 20.605 / 18.015 | 62.560181 |
| Campus3 reverse / new → old → official | 104.607140 / 109.175538 | 165.95 / 170.99 | 32.308 / 30.683 | 95.182039 |

**A stable Campus3 speedup is not established.** Forward elapsed/CPU improve but
p95 regresses; reverse elapsed increases **4.37%** and CPU **3.04%**, while p95
decreases. Retain both directions without combining them into a stable benefit
or selecting the favorable metric. The simple capacity reuse implementation is
retained with verified output equivalence, not an across-the-board speedup claim.
All full reports are linked in the [current record](../../../docs/research/elevator-native.md).
The first reverse batch ended in an environment interruption without a completed
assessment; its cause is unestablished, and it is not an estimator failure or a
timing result. The completed retry uses a new tag. Every result is retained without
selecting the faster batch. The benchmark now defaults to query-storage; historical
command JSON files retain their actual query-scratch/query-storage binary paths.

**Rejected query-scratch:** its target tree suite passed in 1.08 s and **27/27
native suites passed in 29.47 s**, but both complete Campus3 orders were slower
than their adjacent old baseline despite identical odom/map bytes, non-tick
states/events and non-runtime summaries:

| Actual order | query-lease / query-scratch monotonic elapsed (s) | Old / new total CPU (s) |
| --- | ---: | ---: |
| old → new → official | 84.663720281 / 90.702988806 | 142.88 / 151.22 |
| new → old → official | 98.782548947 / 110.360592374 | 158.99 / 170.50 |

Elapsed increased 7.13% / 11.72%, CPU increased 5.84% / 7.24%. Both reports and
the rejected source remain preserved; no preferred run replaces the other.
The recovered WSL HCS startup timeout was an environment failure, not a compiler
failure. See the
[latest record](../../../docs/research/elevator-native.md).

Speed, repeatability and accuracy remain separate. The real sequences described
in the paper's [VI-A](https://arxiv.org/html/2605.24495v1) lack complete trajectory
ground truth; same-floor endpoint references do not establish trajectory ATE/RPE
or superiority over the official estimator.

The historical query-storage candidate's own synthetic native tests have known motion: +/-4.725 m,
endpoint absolute height errors 0.00281607/0.00281605 m, **maximum in-run height error
0.306986 m**, and stop odom residual 0.00168295 m. These are synthetic-test
measurements from [its detailed test log](../../../build/ltlio-performance-buffer-20261018/tests-storage-details.log),
not whole-ride millimetre accuracy, real-world trajectory truth or a field result.
The earlier candidate's log remains separate; neither storage layout changes estimation rules.

**Historical performance baseline (2026-10-17):** `ltlio_replay-query-lease`, compared with the preceding
`ltlio_replay-tree-reuse`. It avoids redundant query-node locking and locks for
pruned child bounds while retaining the reader lease, wakes waiting writers only
when the last reader leaves, and fills/returns neighbor-cache entries without
copying their vectors. Filter mathematics, point order, input quality, cache
rules and deterministic neighbor selection remain unchanged.

Aligned Dormitory1 profiling first confirmed the native instrumented output
against tree-reuse. Both programs process 4,889 frames and 9,779 rematches, with
3,829,753 / 3,830,349 matching queries (native / official). Matching-region time
is 27.170 / 20.124 s and map insertion is 28.840 / 19.369 s; the pose solve is
0.978 / 1.184 s. These are aligned nested-scope measurements, not additive totals
or pure KNN timing. See the [profiling evidence](../../../build/ltlio-performance-next-20261017/profile/assessment.json).

The pruned-child and last-reader/snapshot regressions pass: the old-code suite
failed in 5.39 s, the fixed suite passed in 0.59 s. All **27 native suites pass
in 28.50 s**. The serial old/new/official runs are complete for all three cases:
old/new odom and map files are byte-identical, every non-tick status/event field
and every non-runtime summary field agrees. All 16,378 timestamps are preserved,
with zero drops/errors and unchanged entry/exit counts 2/2, 3/3 and 1/1.

| Case | Same-batch tree-reuse / query-lease elapsed (s) | Elapsed reduction | CPU reduction | Old / new native tick p95 (ms) |
| --- | ---: | ---: | ---: | ---: |
| Office1 | 72.948 / 52.025 | 28.68% | 24.30% | 31.785 / 21.720 |
| Dormitory1 | 58.417 / 35.835 | 38.66% | 34.38% | 22.250 / 13.851 |
| Campus3 | 142.908 / 111.601 | 21.91% | 20.84% | 46.588 / 33.617 |

Same-batch official elapsed times are 54.052 / 54.698 / 78.062 s. The candidate
takes 0.962 / 0.655 / **1.430** times official elapsed; Campus3 still uses 42.96%
more monotonic time and 14.55% more CPU time. Office1's small advantage is one
observation, not a statistically significant result.
Official/native strict map agreement still fails, with position RMS/max of
4.691/48.237, 40.815/141.395 and 56.548/180.357 mm. These are implementation
differences, not ground-truth errors or accuracy improvements. Official poses
use fixed body calibration and initial anchoring, compared to corrected map.

The [current record](../../../docs/research/elevator-native.md) contains full
same-batch CPU/RSS, p95, terminal-height and lifecycle metrics and separates them
from retained tree-reuse, stable-selection and verified evidence. Monotonic and
GNU wall clocks are both retained; native backend tick and official pipeline
timing have different scopes. Absolute times from different batches must not be
combined into speedups. This is not a statistical or NX real-time guarantee.
At the end of that historical round, further query-workspace reuse and batched
reader leases had not been implemented or measured. Later candidate status is
above; the old profile is not a measurement of the new candidate's remaining cost.

Historical stable-selection repeats agree byte-for-byte, while its old-baseline
comparison retains the 0.240291756 m / 1.37176597 degree failure and the real
Office equal-distance-neighbor diagnostic. Those are historical numerical
findings, not new accuracy results. The 43 Python passes belong to the earlier
verified round and were not rerun for this C++ change. No field readiness is claimed.
Campus3 retains the dataset's manual entry event; it is not evidence of fully
automatic elevator recognition. The field backend defaults to `elevator_mode:
off`. Manual service wiring and transport-aware saved-map registration remain
open integration items; see the [field integration scope](../../../docs/research/elevator-native.md#普通定位运输状态与现场接线).

The numerical baseline is the complete default mapping chain from
[Elevator-LIO `1d79af77f3d9747ea57ef52a9b01d326a8ec561a`](https://github.com/xiaofan4122/Elevator-LIO/tree/1d79af77f3d9747ea57ef52a9b01d326a8ec561a),
itself in the Fast-LIO family. LTLIO identifies LingTu's maintained integration,
not a claim to have invented these algorithms. Upstream authorship, file-specific
licenses and exact source correspondence remain in
[`vendor/elevator_lio/README.md`](vendor/elevator_lio/README.md).

## Core and runtime boundary

| Component | Responsibility |
| --- | --- |
| `frontend.hpp`, `frontend.cpp` | Native IMU/cloud intake, per-instance ownership, scheduling and result snapshots |
| `detail/IMUProcess.{h,cpp}` | Initialization, IMU history, propagation, interpolation and posterior reintegration |
| `detail/ESEKF.{h,cpp}` | Point-to-plane iterated update and covariance |
| `detail/LidarPipeline.{h,cpp}` | Initial map, deskew, sampling, matching and map insertion |
| `detail/ElevatorProcess`, `detail/ElevatorSelfExit` | Transport states, detection and stop/exit updates |
| `detail/ikd_Tree`, `detail/IkdMap.hpp` | Incremental spatial map, bounds/budget and synchronized snapshots |
| `../slam/cpp/ltlio.cpp` | `ISlamBackend`, sensor/body calibration, health, external pose input, SAM/save/reload and relocalizer integration |

Each frontend owns its queues, IMU history, filter, map and detector history.
Feed, process, reset and snapshot calls on one instance are serialized by its
owner; separate instances share no mutable estimation state. The tree manages
its internal rebuild worker and parallel read-only matching queries. Reset joins
that worker and discards the prior instance's map and pending observations.

The state order is R/P/V/BW/BA/G/Z/VZ/AZ, with 21 error coordinates. Fixed
LiDAR/IMU calibration belongs to configuration, not estimated state. During
transport, `p/v` describe cabin-relative motion and `Z/VZ/AZ` describe transport;
world position is `p + [0, 0, Z]`. Exit absorbs `Z` into `p.z` and `VZ` into
`v.z`, then resets the transport states so ordinary LIO resumes in historical
building-map coordinates. `Z` does not accumulate across rides. The old 24-state
filter is no longer a supported alternative.

## Input and configuration

- IMU gyro is rad/s. Acceleration uses explicit `acc_scale`, where input times
  scale is m/s². Initialization averages raw measurements before scaling;
  propagation scales each observation once.
- Raw cloud points retain nanosecond offsets and Livox line/tag values. The
  frontend owns selection and filtering; adapters must not repeat the stride.
  Range filtering preserves the original frame end time.
- Fixed LiDAR/IMU and navigation-body transforms remain configurable for all
  RobotConfig platforms. `esti_il: true` is explicitly rejected: online extrinsic
  estimation belonged to the retired filter and is not silently emulated.
- Scan/map resolution, local-map range and point budget, nearest-neighbor count,
  iteration count, input queue bounds and configured range limits feed the sole
  implementation. A zero scan-resolution setting retains upstream adaptive
  sampling. These parameters do not create a second algorithm branch.
- Positive stationary IMU thresholds screen initialization windows. Ordinary
  stationary constraints additionally need consistent LiDAR motion evidence;
  a steady IMU alone cannot distinguish standing still from constant speed.
  The three-axis stationary update is suspended during transport. Ordinary
  motion has no world-vertical zero-velocity prior, so slopes and stairs remain
  fully three-dimensional. The former `vertical_velocity_constraint_enabled`
  and `vertical_velocity_sigma_v` configuration keys are retired.
- A gyro discontinuity during initialization restarts the raw calibration
  window. During propagation, isolated gyro spikes retain the preceding gyro
  only; current acceleration is preserved. Two consistent raw gyro readings
  can establish a new reference instead of being rejected indefinitely.
- IMU intake/history loss invalidates continuous stop evidence while preserving
  the observed elevator phase. A fresh 300-sample variance window and 1.5-second
  stop hold are required before new LiDAR stop updates can finish the trip.
  If a gap hid deceleration, an already active trip can resume stop verification
  when that fresh window satisfies the existing waiting velocity/acceleration
  bounds. A waiting trip or a still-moving window cannot use this recovery.
- Automatic entry still uses the upstream closed-range/time heuristic. Known
  moving-stair and corridor counterexamples remain unresolved; the frontend
  recovery regressions do not establish correct elevator recognition. Changes
  to entry need both real elevator positive cases and ordinary-motion negative
  cases. See the [entry evidence](../../../docs/research/elevator-native.md).
- External odometry-prior correction updates the same frontend's retained
  state and reintegrates subsequent IMU samples. The explicit simulation/truth
  bypass remains an input mode without another estimator. Missing time-aligned
  prior data does not silently fall back to estimated motion.

The obsolete `estimator` selector, old degeneracy projection/rejection flags,
`gravity_align` and `zupt_sigma_pos` are retired. The supported calibration keys
`na/ng/nba/nbg` are read as Kalibr SI noise densities and squared once for the
continuous-time process model; they are not per-sample variances. Omitting them
retains the upstream process-noise defaults. Existing fixed robot
calibration remains authoritative; public-dataset calibration is not a field
robot profile.

## Mapping and localization

`near_search_num` selects the queried neighbors that nominate local plane cells;
each cell fits its retained support, not only those nearest returns. The default
remains five. A Frontend regression checks that a truly collinear map cannot
manufacture a plane, while sparse scan lines can use the adjacent observed
support retained in the same floor cell. Plane support, covariance and cached
cells follow insertion, local-map removal, point-budget pruning and reset of
the existing KD map. Saved geometry and relocalization formats are unchanged.

Increasing the ordinary matching neighborhood is not a qualified default
accuracy improvement. The isolated twenty-neighbor experiment improves Slippage
and AIST and the full offline-core tunnel score, but worsens the default tunnel's
shared early reference samples and still ends LOST. It does not change field
configuration. Commands, failed candidates and comparisons are retained in
`build/scene-uncertainty-20261020/`; these are offline evidence, not robot tests.

The frontend initializes its matching map from five scans, then reports actual
matching attempts and convergence. Invalid/empty/sparse scans cannot republish
an earlier scan under a new timestamp. After initial map creation, scans older
than retained IMU history are dropped and counted; later covered scans can
continue without resetting localization.

The backend anchors the initial navigation body in odom without rotating the
estimator's internal voxel map. Map alignment changes `map -> odom`, leaving
local odometry continuous. Sensor time jumps invalidate live output; localization
retains the saved map but requires a new measured alignment before navigation
output resumes.

Active cabin scans remain available locally but do not become static building
patches. Map saving uses static patches after any transport, so a cabin-relative
matching tree cannot leak into the saved building map. SAM, asynchronous saves,
map loading and the resident relocalizer continue through the same backend.

## Elevator transport

`elevator_mode` accepts `off`, `auto` and `manual`; field enablement is explicit.
`ISlamBackend::setElevator(bool)` changes estimation mode, not robot locomotion,
door control or navigation task progress. Automatic range-based entry can confuse
an enclosed room with an elevator, and self-exit means detected transport stop,
not proof that the robot has left the cabin.

Automatic entry additionally bounds three-dimensional movement during the range
hold. The backend reads these optional settings from its sensor YAML:

```yaml
elevator_mode: auto
elevator_entry:
  range_m: 3.0
  hold_s: 2.0
  motion_m: 0.15
```

At every scan end, the detector compares the predicted IMU position with the
candidate window's origin. Exceeding `motion_m` restarts the origin and timer;
the check precedes entry, so an outward-and-return movement cannot borrow the
old hold. This is maximum excursion from an anchor, not accumulated odometry
distance or a requirement for exactly zero velocity. Small standing jitter does
not accumulate. Movement after an explicit request or after entering transport
does not cancel transport. The 0.15 m default is an initial engineering value,
not an upstream parameter or a field-calibrated threshold. A stopped narrow
corridor remains ambiguous; this is not semantic elevator recognition.

The pinned default chain provided the numerical migration baseline. The current
transport-boundary changes retain the upstream position-reset approximation and
make the native event/output boundaries explicit:

- Manual and automatic entry consume the same closed-like range episode. A new
  open-like observation outside transport re-arms entry; an old episode cannot
  immediately re-enter after self-exit. This does not identify a physical door.
- Entry initializes transport states. Exit absorbs position and velocity,
  retains the explicit velocity covariance transform for a moving manual exit,
  and applies upstream's position-covariance anchoring/reset approximation.
  Transport cross-blocks and priors reset as specified by that policy; world
  covariance is not claimed continuous across reset. This is the published
  design, not an upstream bug reclassified by this integration. The historical
  matching tree is retained rather than cleared at every exit.
- A waiting or stop zero-velocity observation keeps the complete 21-state
  posterior. Its same-time pose correction accumulates in `world_from_odom`
  (`C`), so the backend can publish continuous navigation odometry and corrected
  map poses. The subsequent transport-state reset follows the explicit upstream
  approximation above. Map height can still change; continuity does not remove
  estimator uncertainty or demonstrate accuracy.
- A waiting observation requested by an IMU to the right of a scan cannot change
  that earlier scan. The timestamped request is applied to an eligible accepted
  scan, and state, frame correction, IMU reintegration and map insertion share
  that scan's commit.

The initial Schmidt-style attempt, exit-map reset, cumulative-Z variants and
`history-dorm` full-position-covariance fold exposed regressions and were not
accepted. The paper separately identifies same-floor terminal consistency and
the role of historical-map revisits in [VI-B](https://arxiv.org/html/2605.24495v1#S6.SS2).
These provide endpoint evidence, not full-trajectory ATE. See the
[repair record](../../../docs/research/elevator-native.md) for retained failures.

## Migration scope

The default Livox/IMU, 21-state, mapping and transport chain is unified above.
Retained internal options such as `full21`, online gravity estimation, elevator
strong priors, exit-height ICP and exit-map rebuilding are not public `Config`
options or a claim that every combination has been validated.

The upstream high-frequency IMU odometry output, wheel `UpdateByWheel` measurement,
and Ouster/Velodyne/XT32-specific decoding and missing point-time reconstruction
are not connected to this native interface. The typed external pose prior is not
the wheel measurement model. Normalized point intake still follows Livox rules.
ROS lifecycle and saved-map integration retain LingTu's existing owners.

The previous reference to omitted upstream image fusion was incorrect: pinned
[ImgProcess](https://github.com/xiaofan4122/Elevator-LIO/blob/1d79af77f3d9747ea57ef52a9b01d326a8ec561a/src/node/LidarPipeline.cpp#L452-L454)
only dispatches `ImgEnd` and returns; its
[FSM actions](https://github.com/xiaofan4122/Elevator-LIO/blob/1d79af77f3d9747ea57ef52a9b01d326a8ec561a/src/main.cpp#L43-L44)
are empty. This is not an implemented image-measurement fusion capability missing
from LTLIO. The [capability table](../../../docs/research/elevator-native.md#官方迁移范围)
records the supported chain, internal options and unconnected adapters separately.

## Verification and public replay

The single-core migration's build, workflow and replay evidence belongs in
`build/ltlio-unified-20261017/`. Earlier two-core results are historical evidence
and must not be claimed as proof for the new binary. See the
[migration record](../../../docs/research/elevator-native.md) for current status
and the [historical replay report](../../../docs/research/ltlio-elevator-replay.md).

The regression surface includes body/mount frames, initialization and stationary
motion, scan-end corrections, DDS twist, asynchronous save, saved-map ICP,
reset/time-jump recovery, cabin exclusion, external pose timing, and tree rebuild
lifetimes. A passing native test is not a MuJoCo or field acceptance result.

The **historical single-core migration build**, before the transport-boundary
changes above, passed all 25 native CTest suites on 2026-10-17. Go2 and Thunder
tests use their actual calibrated profiles for matching, saving, loading and
measured ICP relocalization. All three complete public replays reproduced the
previous separate 21-state Elevator-LIO native position outputs exactly (16,378
poses; this is not a claim of numerical identity with the retired 24-state filter), with zero input drops
and identical transport/health states. Office1 and Campus3 also pass the strict
fixed-upstream comparison. Dormitory1 retains the existing 2.12 mm / 0.054 degree
fixed-reference discrepancy and matches the separately retained upstream repeat;
the comparison correctly remains nonzero. These passes and strict upstream
comparisons verify that historical binary, not the current changed algorithm.

Historical stable-selection evidence is in `build/ltlio-repeatability-20261017/`, using immutable
`ltlio_replay-stable-selection`. Its `tests-regression.log` records 27/27 native
suites passing in 32.86 s, including the targeted KNN and voxel-selection cases.
Office1 ordinary-mode solo and two concurrent replays have byte-identical
odom/map files and pass zero timestamp tolerance plus `1e-6 m / 1e-4 degree`
comparison. Old-baseline agreement still fails by 0.240291756 m / 1.37176597
degrees. See `assessment-stable-off.json`; no alignment or relaxed threshold
turns that failure into a pass.

The three transport replays retain all 16,378 published timestamps, zero input
drops/errors, and entry/exit counts Office1 2/2, Dormitory1 3/3, Campus3 1/1.
Terminal map heights are +0.000780407/-0.001987045/-0.778067284 m. Maximum adjacent
height changes in stop-check windows are 19.796/42.676/35.224 mm in odom and
326.936/52.644/368.099 mm in map. These windows include the adjacent poses around
each stop-check interval; they are not whole-sequence maxima. Full values are in
`assessment-stable.json`. Dormitory's same-floor reference constrains the endpoint;
complete trajectory truth is absent and posterior map corrections remain visible.

Earlier transport-repair evidence remains in `build/ltlio-elevator-fix-20261017/`.
That `verified` binary passed 27 native suites and 43 Python tests but exhibited
same-binary repeatability failure of 0.240292 m / 1.371766 degrees. Its matching
repeat did not erase the original failure. The Python tests were not rerun for
that historical C++ change. Its failed `final-decision.json` and original thresholds
remain preserved; overall accuracy and field readiness are not established.

Two earlier instrumented Office prefixes (2,551 poses each) reproduced the matching
branch, not the failed run. Their 66 traced rematches / 32,854 queries were
identical, without mixed tree generations, active rebuild or equal-distance
candidate events in the measured window. Instrumentation perturbs timing; this
does not erase the failure. A separate 3,321-point reconstruction diagnostic
demonstrates equal-distance KNN subset sensitivity (194/729 changed queries),
including a seven-point 90-degree plane example. Those earlier diagnostics alone
did not establish an Office1 link. The subsequent stable-selection implementation orders
heap/replacement and voxel representatives consistently by distance then x/y/z,
and searches boxes at the cutoff distance. Formal red/green, near-unequal-distance
and voxel tests pass; the complete repeats above now agree. Distance arithmetic,
precision and acceptance thresholds remain unchanged.

The new [real-match trace](../../../build/ltlio-repeatability-20261017/trace/analysis.json)
matches the uninstrumented stable prefix exactly across 2,551 poses. At
`1774195436.2404275`, iteration 2, point index 1, recorded prior p/q/v and the
float query agree with the old trace. The fifth neighbor changes at identical
squared distance `0.0016821692697703838`; the stable trace also records the old
candidate at the heap boundary, establishing coexistence. The plane residual
changes from -0.011066377405 to -0.006347748399 m, and that frame's first pose
difference is 11.737 micrometres before subsequent trajectory divergence. This
connects tie selection, plane fitting and filter output in this real comparison.
Full covariance/bias/gravity were not logged; equality of the full 21-state prior
is not claimed. The older approximately 1e-9 m discrepancy at an earlier timestamp
and all possible scheduling combinations are not explained by this trace.

The `verified`-round synthetic up/down tests have truth +/-4.725 m, endpoint error about
0.002816 m, but maximum in-run error 0.306986 m. Stop odom residual is 0.001683 m;
moving manual exit retains velocity from 1.06696 to 1.06731 m/s. These are native
synthetic tests, not MuJoCo physics or field motion, and endpoint millimetres are
not a whole-ride accuracy claim. No dependencies or field deployment are added.

The standard-library ROS1 bag reader requires neither ROS nor rosbags:

```bash
python -m tools.datasets.elevator_bag_to_ltu1 /data/Office1.bag build/elevator-input
cmake --build build/slam_core --target ltlio_replay
build/slam_core/ltlio_replay build/elevator-input/Office1.ltu \
  config/acceptance/elevator_native.yaml build/elevator-output/office-unified
```

Use a fresh output prefix. The acceptance YAML preserves public-dataset
calibration and an upstream-parameter control for the same sole core; it does
not select another estimator. Campus3 additionally needs its recorded event
file. Office1, Dormitory1 and Campus3 do not provide complete trajectory ground
truth for these selected recordings. Implementation agreement is therefore not
ATE/RPE, a field precision guarantee or autonomous-elevator acceptance.
