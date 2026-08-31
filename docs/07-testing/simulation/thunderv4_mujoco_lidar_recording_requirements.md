# ThunderV4 MuJoCo MID-360 Recording Requirements

Status: current recording acceptance requirements

This is the default acceptance format for ThunderV4 MID-360 LiDAR simulation
recordings in MuJoCo.

## Fixed Style

- The main view must be a MuJoCo RGB render, not a Matplotlib point-cloud image.
- The robot model must be `sim/robots/thunderv4/mjcf/thunderv4.xml`.
- Motion must use `sim/robots/thunderv4/policy/pose_flat_low_kpkd_microterrain_model29600_policy.pt`.
- The report must show `policy_class=TorchScriptPolicyRunner`.
- The scene should use a low-clutter gray MuJoCo background with shadows and simple obstacles.
- Robot brightness may be adjusted for readability, but physics, joints, policy, and collision bodies must not change.
- If trajectory is shown, it must be a continuous thin line, not scattered points.

## LiDAR Requirements

- The LiDAR backend must be MuJoCo-LiDAR: `backend=mujoco_lidar`.
- Product reports must use `product_lidar_backend_verified`; do not introduce
  `mature_*` fields for this path.
- The scan pattern must use `sim/assets/livox/mid360.npy`.
- Point clouds must come from `engine.get_lidar_points()` XYZI raycast hit points.
- Ground-projected patterns are not valid LiDAR point clouds.
- Do not draw LiDAR rays; render only hit points.
- Video rendering may sample points for readability, but the report must keep the real scan counts.
- XYZI intensity is the current simulation proxy, not true material reflectance.
- Current intensity model: `180/(1+(range_m/25)^2)+N(0,3)`, clipped to `[1,255]`.
- The report must include `product_lidar_backend_verified=true`, `ground_projected_pattern=false`, `rays_drawn=false`, `point_source=engine.get_lidar_points() XYZI raycast hits`, and `intensity.min/p50/mean/p95/max`.

## IMU And SLAM Requirements

- The MuJoCo engine must expose `imu_gyro`, `imu_projected_gravity`, and `imu_linear_acceleration`.
- The native MuJoCo sensor publisher must publish `/lidar/raw_frame` and `/imu/raw`; the Python `MujocoDriverModule` does not duplicate the IMU stream.
- The recording script is not the runtime topic publisher. It only records in the report whether IMU samples and driver IMU outputs are available.
- A video recording alone does not prove SLAM input is wired. Direct Fast-LIO validation still needs raw LiDAR frame, IMU, time sync, and DDS/portable adapter input checks.
- If the report has `ready_for_direct_slam=false`, do not claim direct SLAM is ready.
- If production SLAM services are already running on DDS domain `0`, use an
  isolated DDS domain for MuJoCo validation. Do not inject simulated frames into
  the live robot SLAM domain.

## Sunrise Native DDS SLAM Check

Run the sensor bridge against a temporary C++ SLAM runtime on an isolated domain:

```bash
export PYTHONPATH="$PWD/src:$PWD"

build/slam_core/slamd \
  --backend fastlio2 \
  --mode mapping \
  --config src/localization/fastlio2/config/sim_mid360_slam.yaml \
  --domain-id 83 \
  --status-json /tmp/lingtu_mujoco_slam_status_domain83.json \
  --cloud-snapshot-dir /dev/shm/lingtu_mujoco_slam_domain83 &

python3 sim/scripts/mujoco/native_dds_sensors.py \
  --duration 30.0 \
  --world industrial_park \
  --settle-s 3.0 \
  --warmup-s 2.0 \
  --drive-ramp-s 5.0 \
  --publish-hz 10 \
  --imu-hz 200 \
  --drive-mode policy \
  --imu-acc-mode sensor \
  --imu-acc-conditioning realistic \
  --imu-acc-axis-scale auto \
  --scan-time-profile physical_rolling \
  --timestamp-clock sim_hardware \
  --domain-id 83 \
  --publisher-bin build/livox_sdk2_stream/livox_sdk2_stream \
  --slam-status-json /tmp/lingtu_mujoco_slam_status_domain83.json \
  --min-slam-motion-ratio 0.5 \
  --max-slam-motion-ratio 1.6 \
  --min-sim-yaw-for-odom-check-rad 0.2 \
  --max-slam-yaw-error-rad 0.15 \
  --require-slam-output
```

Expected pass evidence:

- `/lidar/raw_frame` and `/imu/raw` are published by
  `livox_sdk2_stream --stdin-records --dds`.
- `slamd` reaches `state=TRACKING`.
- `observed_slam_outputs` includes `/slam/odometry`, `/slam/registered_cloud`,
  `/slam/map_cloud`, and `/slam/localization_health`.
- `motion.sim_xy_m` records nonzero MuJoCo motion and
  `motion.slam_to_sim_xy_ratio` is between the configured min and max ratio.
  If MuJoCo moves but SLAM odometry remains near zero, the report must fail
  with `native_slam_motion_mismatch`; if SLAM over-estimates too far, the
  report must fail with `native_slam_motion_overshoot`. Treat the map-frame
  transform as unverified when either gap appears.
- For turning runs, `motion.slam_to_sim_yaw_error_rad` must remain below the
  configured yaw error threshold. If it exceeds the threshold, the report must
  fail with `native_slam_yaw_mismatch`.
- `/dev/shm/lingtu_mujoco_slam_domain83/map_cloud.bin` is non-empty and can be
  decoded as `runtime.msgs.sensor.PointCloud2`.
- Do not stack `/slam/map_cloud` snapshots and call that a complete map.
  `/slam/map_cloud` is the current map-frame scan output. For a saved map image,
  call `POST /api/v1/map/save` while the `map` Product is active, then render the
  committed map artifact exposed by `mapd`. A saved PCD proves the native
  map-builder wrote a cumulative artifact, but it is usable
  navigation evidence only after both the motion-consistency gate and the saved
  map quality gate pass.

After `save-map`, run the saved-map quality gate for known MuJoCo worlds:

```bash
python3 sim/scripts/mujoco/saved_map_quality_gate.py \
  --pcd artifacts/<run>/native_saved_map.pcd \
  --world industrial_park \
  --json-out artifacts/<run>/native_saved_map_quality_gate.json \
  --plot-out artifacts/<run>/native_saved_map_quality_gate.png
```

Before running saved-map relocalization, use the native DDS preflight. It checks
the saved-map contract and required native inputs without starting MuJoCo,
`slamd`, or a relocalization request:

```bash
python3 sim/scripts/saved_map_relocalize_runtime_gate.py \
  --map-pcd artifacts/<run>/native_saved_map.pcd \
  --preflight-only \
  --strict \
  --json-out artifacts/<run>/saved_map_relocalize_preflight.json
```

The preflight must report `ok=true` before the live gate. This is host and
artifact readiness only; it does not validate localization behavior.

The gate filters obstacle-height cells (`0.30 <= z <= 1.60`), drops sparse and
small isolated components, and reports both raw scene-frame overlap and bounded
2D aligned overlap. Use the aligned result to judge local map shape. Use the
native DDS motion gate to judge map-frame yaw/translation consistency.

## Current Sunrise Native DDS Result

Current isolated-domain MuJoCo native DDS closure on sunrise:

```text
/home/sunrise/data/inovxio/lingtu/artifacts/sunrise_mujoco_fastlio_sensor_fixed_30s_20260706/
```

Motion result was `report.ok=true`, `save-map` succeeded, and saved-map
quality result was `quality.ok=true`. This is the current accepted MuJoCo
native DDS saved-map evidence because it uses policy motion, sensor-mode IMU
with realistic conditioning, no odom prior, unified simulated hardware clock,
and the native C++ Fast-LIO runtime.

Observed data:

- Native DDS sensor input worked with `imu_acc_mode=sensor`,
  `sim_hardware_sensor_model=mujoco_accelerometer_conditioned_imu`.
- Native SLAM output existed: `/slam/odometry`, `/slam/registered_cloud`,
  `/slam/map_cloud`, `/slam/localization_health`, and
  `/slam/localization_quality`.
- SLAM reached `state=TRACKING`.
- MuJoCo moved `motion.sim_xy_m=1.407 m`.
- SLAM odometry moved `motion.slam_odom_xy_m=1.381 m`.
- `motion.slam_to_sim_xy_ratio=0.982` with gate range `[0.5, 1.6]`.
- Yaw error was `-0.0006 rad` with gate max `0.150 rad`.
- `remaining_gaps=[]`.
- Native `save-map` produced `native_saved_map.pcd`.
- Saved-map quality gate passed after bounded 2D scene alignment:
  `99.65%` of candidate cells were within `0.4 m` of expected obstacle
  geometry, and `0.0%` were farther than `0.6 m`.
- Current native DDS saved-map evidence must use `--drive-mode policy`,
  `--imu-acc-mode sensor`, `--imu-acc-conditioning realistic`,
  `--timestamp-clock sim_hardware`, empty IMU/LiDAR timestamp overrides,
  physical rolling subscans, 200 Hz IMU target, and `--imu-acc-axis-scale auto`
  resolving to identity under policy sensor mode.
  `gravity_only` remains a diagnostic baseline only.

Long-run full-map status:

```text
/home/sunrise/data/inovxio/lingtu/artifacts/sunrise_mujoco_fastlio_sensor_fixed_240s_20260706/
```

The 240 s `box_explore` run is red and must not be used as full-map closure
evidence. It used the same sensor-conditioned Fast-LIO-only route and saved a
native map, but the motion gate failed: `sim_path_length_xy_m=15.257`,
`sim_xy_m=1.175`, `slam_odom_xy_m=2.635`, and yaw error `-0.263 rad`. The
saved-map quality gate also failed with aligned near ratio `45.73%`,
far-ghost ratio `40.78%`, median nearest obstacle distance `0.463 m`, and p90
distance `1.262 m`.

## Continuous Mapping Quality Gate (3-5 min)

Short endpoint motion gates are necessary but not sufficient. Use the continuous
gate when claiming "sim radar mapping is stable":

```bash
export PYTHONPATH="$PWD/src:$PWD"

python3 sim/scripts/mujoco/continuous_mapping_quality_gate.py \
  --world industrial_park \
  --duration 180 \
  --domain-id 231 \
  --drive-profile box_explore \
  --saved-map-dir /path/to/mapd/saved/map \
  --run-dir artifacts/mujoco_continuous_mapping_gate_<timestamp>
```

The gate does not call `slamctl save-map` or write a PCD directly. Save the map
first through the public SDK/Gateway -> native `mapd save_map` Product chain,
then pass that map directory with `--saved-map-dir`. It fails unless all of the
following pass:

| Group | What it checks |
| --- | --- |
| Bridge | MuJoCo policy drive publishes `/imu/raw` + `/lidar/raw_frame`; native SLAM outputs exist at end |
| Continuity | Periodic `status.json` samples stay `TRACKING`; zero dropped LiDAR/IMU; zero rollbacks; no scan stall; bounded odom/velocity |
| Convergence | `sim_motion.jsonl` vs saved `trajectory.txt`: windowed path-length ratios, cumulative path ratio, rigid-aligned ATE |
| Map quality | Externally supplied mapd PCD passes `saved_map_quality_gate.py` aligned obstacle overlap |

Artifacts written under `--run-dir`:

```text
summary.json
bridge_report.json
sim_motion.jsonl
slam_status_samples.jsonl
<saved-map-dir>/map.pcd
<saved-map-dir>/trajectory.txt
saved_map_quality.json
scale_convergence.png
trajectory_overlay.png
```

Recommended acceptance thresholds (defaults in the gate):

- Window path ratio `[0.5, 1.8]` every 30 s while driving
- Cumulative path ratio `[0.7, 1.4]` over the whole drive phase
- ATE RMSE `< 0.6 m`, ATE max `< 1.2 m` after 2D rigid alignment
- Map quality aligned near ratio `>= 80%`, far ghost ratio `<= 15%`

Do not use endpoint-only `motion.slam_to_sim_xy_ratio` from a 30 s run as
long-map closure evidence. The 240 s red run passed short-window checks in
isolation but failed endpoint displacement, yaw drift, and map ghosting together.

Sunrise remote runner (from dev machine on the same LAN):

```bash
export LINGTU_HOST=ROBOT_IP_OR_HOSTNAME
python sim/scripts/run_sunrise_continuous_mapping_gate.py \
  --host ${LINGTU_HOST} \
  --duration 180 \
  --domain-id 231
```

Field-run record with the first 180 s verdict matrix:
[2026-07-06-mujoco-continuous-mapping-gate.md](../field-runs/2026-07-06-mujoco-continuous-mapping-gate.md).

Keep `--domain-id` in **`200–232`**. Domain **234** fails CycloneDDS startup on
sunrise (multicast port out of range).

Interpretation of the long run: the raw LiDAR/IMU transport and short local map
path are connected, but complete long-trajectory map acceptance is not closed
yet. The current native DDS save path accumulates Fast-LIO odometry without
loop-closure/PGO correction, so long loop-like motion can create duplicate
walls and ghost obstacle cells.

Interpretation:

- The raw LiDAR/IMU -> native DDS -> Fast-LIO2 transport chain is closed. The
  old kinematic finite-difference result and the old gravity-only result are
  superseded. Saved-map acceptance now requires the policy sensor-conditioned
  bridge plus the saved-map quality gate.
- Use `sim_mid360_slam.yaml` for this simulation gate. Do not run
  MuJoCo synthetic IMU through the Thunder-specific `mid360_fastlio2.yaml` acceleration
  scale.
- Keep the current MuJoCo gate extrinsic colocated (`t_il=[0,0,0]`) because the
  simulation bridge now publishes a single simulated MID-360 package: raw LiDAR
  points and IMU samples both come from `lidar_site/lidar_link`.
- Do not use the old split-clock workaround (`LiDAR wall`, `IMU sim`) as the
  product default. It remains only a legacy diagnostic comparison.
- Real robot timestamp issues are possible but different from this MuJoCo
  problem. On hardware, validate device/driver timestamp source,
  `time_diff_lidar_to_imu`, extrinsics, and SLAM sync diagnostics; there is no
  MuJoCo `sim_time` competing with process `wall_time`.
- The native saved PCD is the cumulative map artifact for inspection. Do not use
  accumulated `/slam/map_cloud` snapshots as the complete map; that
  visualization can look like a TF error because it stacks current scan windows.
- The saved PCD top-down screenshot and any quick filtered occupancy preview are
  diagnostic only. The domain103 preview used a 0.2 m XY grid with
  count/z-span/z-max filtering; diagnostics showed 12,989 high points above
  1.60 m and 750 sparse kept cells under the previous preview rule. This is not
  a product navigation map.
- Product navigation acceptance requires a real map-layer output: saved PCD or
  live `/slam/map_cloud` -> height/ground filtering -> raycast free-space
  occupancy -> inflation/traversability cost -> planner input. Do not approve
  localplanner/exploration behavior from a raw PCD scatter plot.
- MuJoCo odom-prior map artifacts are diagnostic only. They prove raycast and
  native map-artifact plumbing with a ground-truth pose prior; they do not prove
  Fast-LIO mapping quality and are not product map acceptance evidence.
- Product simulation mapping acceptance requires Fast-LIO-only input by
  default: no `/slam/odom_prior`, unified simulated hardware timestamps,
  physical rolling subscan timing, 200 Hz IMU samples, Livox-compatible
  `tag/line` fields, and a saved obstacle layer that passes the map-quality
  gate. Real S100P/MID-360 maps must not use a MuJoCo pose prior.

Historical red gate:

```text
artifacts/sunrise_mujoco_native_dds_slam_motion_gate_domain88_20260705_112017/report.json
motion.sim_xy_m = 1.132
motion.slam_odom_xy_m = 0.000047
remaining_gaps = native_slam_motion_mismatch:sim_xy=1.132,slam_xy=0.000,min_slam_xy=0.226
```

That failure showed that DDS connectivity and SLAM output alone were not enough:
the initial bridge had no rest warmup and the ZUPT static detector could clamp
smooth kinematic motion. The current gate keeps this failure mode covered.

## Ground Contact Requirements

- Visible wheels must use the four wheel cylinders: `FR_wheel`, `FL_wheel`, `RR_wheel`, and `RL_wheel`.
- `*_foot_visual` meshes can intersect the ground and must not be treated as visible wheels in the recording.
- The report must output `visible_wheel_clearance_min_m`.
- `visible_wheel_clearance_min_m` must be greater than or equal to `0`.

## Generation Command

```powershell
$env:PYTHONPATH='D:\inovxio\brain\lingtu\src;D:\inovxio\brain\lingtu'
python sim\scripts\mujoco\record_thunderv4_mid360_policy.py
```

Default output:

```text
artifacts/mujoco_thunderv4_mid360_policy_<timestamp>/
  thunderv4_mid360_policy_browser.mp4
  thunderv4_mid360_policy_preview.gif
  first_frame.png
  mid_frame.png
  report.json
```

## Current Acceptance Sample

```text
artifacts/mujoco_thunderv4_mid360_policy_20260704_074304/
```

Key results:

- `.pt` policy used: `TorchScriptPolicyRunner`.
- Low-speed robot travel distance: about `1.08 m`.
- Minimum visible wheel clearance: about `1.1 mm`.
- LiDAR source: real MuJoCo-LiDAR hit points.
- No rays or ground-projected pattern used.
- IMU/raw LiDAR are available inside the engine and are published by the native sensor process. Direct SLAM still requires raw LiDAR/IMU input validation.

## No Python SLAM

Do not write or ship Python SLAM for MuJoCo validation. Python may render
MuJoCo, record videos, publish simulated raw MID-360/IMU frames, and collect
diagnostic reports. Pose estimation, map building, and relocalization must be
provided by native C++ SLAM/localization (`slamd` or an
explicit external native service). Reports must keep `no_python_slam=true` when
this path is used.
