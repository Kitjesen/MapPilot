# Go2/NX stationary travel-distance repair — 2026-10-10

Installed `v2.3.0-go2.20261010.91`, source snapshot
`e3baba40b5fe7dcdf4482f4cd808b1edd0a519e3`, on NX at `192.168.66.65`.
The full ABI 11 package extends the [.89 deployment](go2-deploy-20261010.md).
Release snapshots are retained on the local `codex/go2-distance-20261010`
reference. This record does not claim the shared dirty working tree was
committed or pushed to main.

## Cause and native repair

`src/localization/slam/cpp/distance.hpp` previously added the planar distance
between every pair of successive continuous SLAM odometry poses. High-rate
millimetre-scale stationary pose fluctuations therefore accumulated as positive
travel. Gateway and the dashboard forwarded that native total; they did not
calculate the extra distance themselves. Global map/GNSS corrections were
already excluded by using continuous odom before the map transform.

The native accumulator now retains a fixed odometry anchor and commits a
segment only after its planar displacement reaches 5 cm. Subthreshold samples
do not move the anchor, so many small increments from slow travel eventually
count. Epoch changes, gaps over 2 s, backwards timestamps and missing/invalid
poses reanchor without inventing a connecting journey. Duplicate snapshots
and heading-only changes do not add travel. The counter still belongs to the
current SLAM process and starts again after restart; it is not lifetime mileage.

This is an approximate planar distance estimate. An unfinished straight
segment can remain below 5 cm, and small reversals or curved motion within that
distance are not fully represented. It suppresses measured stationary jitter;
it does not make larger odometry drift or a pose-estimation error into real
travel. No motion command or navigation gate is derived from this counter.
The old accumulated value, including the operator's reported 11.6 m while
stationary, is not a trustworthy historical mileage measurement.

## Startup calibration exposed during installation

The first `.90` deployment attempts failed Fast-LIO initialization. Live LiDAR
and IMU data were present; `waiting_for_lidar` was an unhelpful status reason
from an empty buffer after consumption, not evidence that the sensor was absent.
Initialization rejected the static-IMU windows and cleared its cache.

A read-only CycloneDDS collector consumed each sample once with `dds_take` and
evaluated two successive 400-sample windows using the initialization formula:

| Measurement | Window 1 | Window 2 | Previous limit |
| --- | --- | --- | --- |
| Normalized acceleration variation RMS | 0.256867 m/s² | 0.268835 m/s² | 0.25 m/s² |
| Centered gyro variation RMS | 0.011768 rad/s | 0.012805 rad/s | 0.02 rad/s |

The Go2 existing `imu_static_acc_thresh` was calibrated to `0.30` in
`config/robots/unitree/go2/sensors/mid360_fastlio2.yaml`; the gyro threshold
remains `0.02`. This threshold is shared by startup initialization and runtime
static/ZUPT qualification in `imu_processor.cpp`. It also widens the runtime
acceleration-variance and mean-gravity-error tests; it is not an
initialization-only exception. Gyro checks, the static-frame count and ZUPT
noise parameters remain unchanged. The collection proves the measured standing
variance and threshold comparison, not ZUPT correctness during real motion.
Raised-mount extrinsics were preserved.

## Verification and installed state

- Before the accumulator change, the new stationary-jitter ARM regression
  failed. After the change, `messages_travel_distance` and
  `messages_frame_export` passed, **2/2**. The tests include 2,000 stationary
  jitter samples, 1 mm incremental forward/reverse travel, heading-only changes
  and discontinuities. The forward and reverse test retains approximately
  1 m in each direction instead of discarding slow movement.
- Replaying **201 captured stationary frames** through the repaired C++
  accumulator returned **0 m**, while the old native counter increased
  **0.315324 m** over the corresponding approximately 20-second capture.
- `.91` reused the tested `.90` binaries after its configuration-only snapshot
  change and packaged the complete DDS, clients, Host, native services, tools
  and dashboard. ProductControl returned `active` for `nav/standard`, session
  `product-be58224091bd4fa2ad43096d0ba65897`. The selected map remains
  `903room_v5_raised`, content epoch **1790681532521**; it was not rebuilt.
- `/proc/<pid>/exe` confirmed LiDAR, slamd, mapd, navd and the driver child
  all run from `.91`; Host's working directory also resolves to `.91`.
  The identity snapshot is retained as `running-versions91.json`. The native process
  environment confirmed SCAN planner/follower session limits of **0.2 m/s**;
  the switch retained the **1.0 m/s²** acceleration setting.
- Detailed readiness reported `ready=true`, `data_ready=true` and
  `motion_ready=true`. No motion goal was submitted.

The field stationary check compares the native counter with the actual API
value used by the dashboard:

| Observation | Before repair | Installed .91 |
| --- | --- | --- |
| Native observation duration | 19.995 s | 30.111 s |
| Native counter increase | 0.315324 m | 0 m |
| Maximum XY displacement from first captured pose | 0.012640 m | 0.006536 m |
| API counter increase | 0.321301 m over a separate 20.124 s capture | 0 m |
| Native captured / distinct timestamps | 201 captured | 301 / 247 |
| API samples | 21 | 31 |
| Localization during post-install capture | — | TRACKING throughout |

The API read is `state.localization.odometry.session_distance_m`. An initial
validation-script attempt looked for a nonexistent top-level `state.odom` and
timed out. Only the corrected script's result is counted as field evidence;
both logs are retained. The post-install check requires advancing native
timestamps and matching API values, so a frozen stream cannot pass merely
because its counter stays unchanged. Task stayed IDLE and authority NONE.

This establishes ARM regression/replay, complete installation and field
stationary behavior. There was no supervised walking-distance calibration or
new field navigation acceptance in this task. Existing optional-camera and
model-loading presentation limitations are not resolved by this mileage fix.

Evidence is retained locally in `build/go2-distance-20261010/` (including
`stationary-result.json`, `stationary-raw-after.json`,
`stationary-api-after.json`, `imu-windows-before.jsonl` and `evidence91.tgz`)
and in `/home/unitree/field-distance-20261010/` on NX. Original captures are in
`build/go2-release-20261010/stationary-distance-before.json` and
`stationary-raw-before.json`. The control page is
[LingTu on NX](http://192.168.66.65:5050/).
