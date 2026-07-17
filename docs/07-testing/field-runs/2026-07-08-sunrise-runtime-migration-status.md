# 2026-07-08 Sunrise Runtime Migration Status

Target: `sunrise@192.168.66.13`

Collection mode: read-only Gateway evidence from `http://192.168.66.13:5050`.

Artifacts:

- `artifacts/thunder_field_runtime/192_168_66_13_report.json`
- `artifacts/thunder_field_runtime/192_168_66_13_validation.json`

## What Is Proven

- Gateway is reachable and reports `status=ok`.
- Gateway health reports `modules_ok=31`, `modules_fail=0`.
- SLAM is tracking.
- Native LiDAR and Livox IMU data are visible to SLAM:
  - `lidar_input_hz` was about `6.96` to `9.99 Hz` across samples.
  - `imu_input_hz` was about `196` to `200 Hz` across samples.
- The runtime evidence collector observed fresh samples for:
  - `/lidar/raw_frame`
  - `/imu/raw`
  - `/slam/odometry`
  - `/slam/registered_cloud`
  - `/slam/map_cloud`
  - `/slam/localization_health`
  - `/slam/localization_quality`
  - `/nav/global_path`
  - `/nav/local_path`
- Required frame checks passed for:
  - `map_to_odom`
  - `odom_to_body`
  - `body_to_lidar`
  - `body_to_camera`

## What Is Not Proven

- Real robot motion is not proven:
  - `real_robot_motion=false`
  - `odom_delta_m=0.007504121800717503`, below the `0.05 m` gate.
- Hardware command sink is not proven:
  - `cmd_vel_sent_to_hardware=false`
  - `hardware_command_route_observed=false`
  - no `/nav/cmd_vel` sample in the read-only gate window.
- Camera is not closed:
  - Gateway health reports `camera.available=false`.
  - Gateway health reports `camera.status=idle`.
  - Gateway health reports `camera.backend=stub`.
- The deployed Gateway is older than the current service catalog work:
  - `/api/v1/services/status` is absent from OpenAPI.
  - field module names still include `CameraBridgeModule`.

## Gate Result

`scripts/gates/real_runtime_evidence_gate.py` returned `ok=false`.

Main blockers:

- `real_robot_motion is not true`
- `cmd_vel_sent_to_hardware is not true`
- `real motion odom_delta_m below min_motion_m`
- `nav command evidence missing`
- `topic sample window missing for /nav/cmd_vel`
- `real hardware command boundary missing`
- data-flow evidence missing or failed for:
  - `map_layers_and_exploration`
  - `global_planning`
  - `local_planning_and_following`
  - `command_boundary`

## Migration Read

This run proves the `lidar -> imu -> slam` field data path is alive, but it does
not prove the full product runtime. The remaining short-role closure is:

1. Deploy current code so Gateway exposes service catalog readiness.
2. Build and install `lingtu_camera_dds`.
3. Start `lingtu-camera-dds.service` and require DDS samples for camera color,
   depth, and camera info.
4. Re-run no-motion navigation preview with current code.
5. Only after safety and no-motion gates pass, run a short low-speed motion gate
   that proves `/nav/cmd_vel -> hw command sink`.

## Read-Only SSH Recheck

Collection time: `2026-07-08 07:00:48 CST`

Command mode: read-only SSH via Paramiko using `sunrise@192.168.66.13`.

System:

- Hostname: `ubuntu`
- Kernel: `6.1.158-rt58-DR-4.0.5-2603031328-g9f678e-g6caa4d`
- Architecture: `aarch64`

Systemd:

| Unit | ActiveState | SubState | NRestarts |
| --- | --- | --- | --- |
| `lingtu-livox-dds.service` | active | running | 0 |
| `lingtu-slam-dds.service` | active | running | 0 |
| `lingtu-traversability-dds.service` | active | running | 0 |
| `lingtu-nav-dds.service` | active | running | 0 |
| `lingtu.service` | active | running | 0 |
| `lingtu-camera-dds.service` | inactive | dead | 0 |

Status files:

- `/dev/shm/lingtu/camera_status.json`: missing.
- `/dev/shm/lingtu/nav_endpoint_status.json`: present; `publish_cmd_vel=false`,
  `has_odom=true`, `has_map_odom_tf=true`, `cmd_vel_published=0`.
- `/dev/shm/lingtu/traversability_status.json`: present; `has_odom=true`,
  `published=72928`, `errors=0`.
- `/tmp/lingtu_slam_status.json`: present; `state=TRACKING`,
  `alive=true`, `has_odom=true`, `map_loaded=true`,
  `lidar_input_hz=9.997736`, `imu_input_hz=204.585034`,
  `processed_scan_hz=10.027072`.

Gateway:

- `GET /health`: `{"status":"ok"}`.
- `GET /api/v1/services/status`: HTTP 404. The robot is still running a
  deployed Gateway build older than the local service catalog readiness route.

Observed native processes:

- `livox_sdk2_stream --dds`
- `lingtu_slam_cyclone_runtime`
- `lingtu_traversability_dds`
- `lingtu_nav_native_endpoint --publish-cmd-vel 0`

Updated migration read:

- Field LiDAR and Livox IMU into native SLAM are proven alive in this boot.
- Camera DDS is still not closed: service is installed/known but inactive, with
  no camera status file.
- Gateway service readiness cannot be proven on the robot until the current
  local Gateway/service catalog code is deployed.
- Motion remains gated off by `publish_cmd_vel=false`; this is expected for the
  no-motion validation stage.

Repeatable local collector added after this recheck:

```bash
python scripts/gates/thunder_service_readiness_collect.py \
  --gateway-url http://127.0.0.1:5050 \
  --json-out /tmp/lingtu_service_readiness.json \
  --pretty
```

Run it on S100P after deploying the current tree to capture the same systemd,
native binary, status-file, GNSS device, Gateway, and process evidence without
publishing commands or changing service state.

The current Gateway can also read that file from
`LINGTU_SERVICE_READINESS_JSON` (default `/tmp/lingtu_service_readiness.json`)
and expose it under `/api/v1/services/status.field_readiness`. Set
`LINGTU_REQUIRE_FIELD_SERVICE_READINESS=1` to make missing, stale, or failed
collector evidence block the Gateway service readiness summary.

## Read-Only SSH Recheck 2

Collection time: `2026-07-08 08:24:45 CST`

Command mode: read-only SSH via Paramiko using `sunrise@192.168.66.13`.

System:

- Hostname: `ubuntu`
- Kernel: `6.1.158-rt58-DR-4.0.5-2603031328-g9f678e-g6caa4d`
- Architecture: `aarch64`

Systemd:

- `lingtu-livox-dds.service`: active, enabled.
- `lingtu-slam-dds.service`: active, enabled.
- `lingtu-nav-dds.service`: active, enabled.
- `lingtu-traversability-dds.service`: active, enabled.
- `lingtu.service`: active.
- `lingtu-camera-dds.service`: inactive by `is-active`, but `is-enabled`
  reports `Failed to get unit file state ... No such file or directory`.

Camera deployment state:

- `/opt/lingtu/current/build/camera_dds/lingtu_camera_dds`: missing.
- `/opt/lingtu/current/build/orbbec_native/orbbec_capture`: missing.
- `/dev/shm/lingtu/camera_status.json`: missing.
- Camera SDK directories found on target:
  `/opt/lingtu/current/src/drivers/real/camera/OrbbecSDK_ROS2` only.
  No pure `deps/orbbec/OrbbecSDK` path was observed in this read-only check.

Native process evidence:

- Running: `livox_sdk2_stream --dds --domain-id 0`.
- Running: `lingtu_slam_cyclone_runtime --backend fastlio2 --mode localization`.
- Running: `lingtu_traversability_dds`.
- Running: `lingtu_nav_native_endpoint --publish-cmd-vel 0`.
- Not observed in process list: `livox_ros_driver2`, `lingtu_camera_dds`,
  `orbbec_capture`.

Status snapshots:

- `/tmp/lingtu_slam_status.json`: present; `state=TRACKING`,
  `alive=true`, `has_odom=true`, `lidar_input_hz=7.271335`,
  `imu_input_hz=197.596545`, `processed_scan_hz=6.989590`.
- `/dev/shm/lingtu/nav_endpoint_status.json`: present;
  `publish_cmd_vel=false`, `has_odom=true`, `has_map_odom_tf=true`,
  `has_traversability=true`, `has_terrain_map=true`,
  `has_terrain_map_ext=true`.
- `/dev/shm/lingtu/traversability_status.json`: present; `has_odom=true`,
  `published=74316`, `errors=0`.
- `/dev/shm/lingtu/camera_status.json`: missing.

Updated migration read:

- LiDAR real backend remains on the native Livox SDK2 DDS path in this sample.
- SLAM is receiving LiDAR and Livox IMU data and is tracking.
- `livox_ros_driver2` was not observed, supporting the intended fallback-only
  status for this boot.
- Camera real closure is still not deployed: no service unit, no DDS publisher
  binary, no Orbbec capture binary, no status file, and only the ROS2-wrapper
  SDK directory was observed.

## GNSS Read-Only SSH Recheck

Collection time: `2026-07-08 08:40:25 CST`

Command mode: read-only SSH via Paramiko using `sunrise@192.168.66.13`.

Device evidence:

- `/dev/wtrtk980`: missing.
- `/dev/ttyUSB*`: missing.
- `/dev/serial/by-id/*`: missing.

Runtime evidence:

- No process was observed for `gnss`, `ntrip`, `wtrtk`, `rtcm`, `gps`,
  `ublox`, or `nmea`.
- No active or installed systemd service matching `gnss`, `ntrip`, `gps`,
  `wtrtk`, or `rtk` was observed.

Config evidence:

- `/opt/lingtu/current/config/robot_config.yaml` declares:
  - `gnss.enabled: true`
  - `gnss.model: WTRTK-980`
  - `gnss.device: /dev/wtrtk980`
  - `gnss.rtcm.enabled: false`
  - `gnss.fusion.enabled: true`
- `/opt/lingtu/current/config/devices.yaml` lists `wtrtk980_main` as a GNSS
  inventory item with `serial.device: /dev/wtrtk980`.
- `/home/sunrise/data/SLAM/navigation/config/robot_config.yaml` and
  `/home/sunrise/data/SLAM/navigation/config/devices.yaml` were missing in this
  boot.

Updated GNSS migration read:

- GNSS is configured as enabled, but the expected serial device is absent.
- There is no evidence of a GNSS, RTK, or NTRIP runtime on the robot in this
  sample.
- `devices.yaml` currently describes the GNSS asset as inventory only; it does
  not prove a data path.
- Field GNSS remains unverified until `/dev/wtrtk980` exists and either direct
  serial samples, explicit `hw` bridge samples, or DDS compatibility samples are
  observed.

## Read-Only SSH Recheck 3

Collection time: `2026-07-08 09:42:57 CST`

Command mode: read-only SSH via Paramiko using `sunrise@192.168.66.13`.

Artifact:

- `artifacts/thunder_field_runtime/192_168_66_13_readonly_camera_service_sample.json`

Systemd:

- `lingtu-livox-dds.service`: loaded, active, running, enabled.
- `lingtu-slam-dds.service`: loaded, active, running, enabled.
- `lingtu-traversability-dds.service`: loaded, active, running, enabled.
- `lingtu-nav-dds.service`: loaded, active, running, enabled.
- `lingtu.service`: loaded, active, running, enabled.
- `lingtu-camera-dds.service`: `LoadState=not-found`,
  `ActiveState=inactive`, `SubState=dead`.

Native binary readiness:

- Present and executable:
  - `/opt/lingtu/current/build/livox_sdk2_stream/livox_sdk2_stream`
  - `/opt/lingtu/current/build/slam_core/lingtu_slam_cyclone_runtime`
  - `/opt/lingtu/current/build/nav_endpoint/lingtu_nav_native_endpoint`
  - `/opt/lingtu/current/build/nav_endpoint/lingtu_traversability_dds`
- Missing:
  - `/opt/lingtu/current/build/camera_dds/lingtu_camera_dds`
  - `/opt/lingtu/current/build/orbbec_native/orbbec_capture`

Camera evidence:

- `/dev/shm/lingtu/camera_status.json`: missing.
- `lingtu_camera_dds`: not observed in process list.
- `orbbec_capture`: not observed in process list.
- `orbbec_camera` ROS2 wrapper: not observed in process list.
- `lsusb` did not report an Orbbec/Gemini/Astra/`2bc5` device.
- Pure SDK path missing:
  `/opt/lingtu/current/src/drivers/real/camera/deps/orbbec/OrbbecSDK`.
- ROS2-wrapper SDK path observed only at:
  `/opt/lingtu/current/src/drivers/real/camera/OrbbecSDK_ROS2`.

LiDAR/IMU/SLAM evidence:

- Running process:
  `livox_sdk2_stream --dds --domain-id 0 --publish-freq 10`.
- `livox_ros_driver2`: not observed in process list.
- `/tmp/lingtu_slam_status.json`: present; `state=TRACKING`,
  `alive=true`, `has_odom=true`, `lidar_input_hz=9.130011`,
  `imu_input_hz=200.539856`, `processed_scan_hz=8.927502`.

Nav/traversability evidence:

- `/dev/shm/lingtu/nav_endpoint_status.json`: present;
  `publish_cmd_vel=false`, `has_odom=true`, `has_map_odom_tf=true`,
  `has_traversability=true`, `has_terrain_map=true`,
  `has_terrain_map_ext=true`, `cmd_vel_published=0`.
- `/dev/shm/lingtu/traversability_status.json`: present;
  `has_odom=true`, `published=75528`, `errors=0`.

Gateway evidence:

- `GET /health`: `{"status":"ok"}`.
- `GET /api/v1/services/status`: `{"detail":"Not Found"}`.

GNSS evidence:

- `/dev/wtrtk980`: missing.
- `/dev/ttyUSB0`: missing.
- `/dev/ttyUSB1`: missing.
- `/dev/serial/by-id`: missing.

Updated migration read:

- Core native LiDAR, SLAM, traversability, nav, and Gateway services remain up.
- LiDAR and Livox IMU remain on the native `livox_sdk2_stream` path; no
  `livox_ros_driver2` fallback process was observed in this sample.
- Camera remains the largest open runtime gap: no unit, no native binaries, no
  status file, no observed USB device, and no pure OrbbecSDK path on target.
- Gateway service readiness remains unproven on the deployed robot because the
  deployed Gateway still lacks `/api/v1/services/status`.
- GNSS remains configured but not physically present as `/dev/wtrtk980`.

## Camera DDS Closed Loop Recheck

Timestamp: `2026-07-08 10:52:56 CST`.

Command mode: bounded SSH deployment/build plus read-only verification on
`sunrise@192.168.66.13`.

Artifacts:

- `artifacts/thunder_field_runtime/192_168_66_13_camera_plugged_readonly.json`
- `artifacts/thunder_field_runtime/192_168_66_13_camera_service_started.json`
- `artifacts/thunder_field_runtime/192_168_66_13_camera_dds_closed_loop.json`

Deployment/build actions:

- Uploaded the minimal camera native/DDS file set to `/opt/lingtu/current`.
  Existing overwritten files were backed up with `.bak-20260708102208`,
  `.bak-20260708102934`, or `.bak-20260708105121` suffixes.
- Fetched pure OrbbecSDK v2 into
  `/opt/lingtu/current/src/drivers/real/camera/deps/orbbec/OrbbecSDK`.
- Built `/opt/lingtu/current/build/orbbec_native/orbbec_capture`.
- Built `/opt/lingtu/current/build/camera_dds/lingtu_camera_dds`.
- Installed `/etc/systemd/system/lingtu-camera-dds.service` and started it.
  The unit is intentionally `disabled` for boot until the full service catalog
  deployment is run.

Evidence:

- USB camera is present:
  `2bc5:0800 Orbbec 3D Technology International, Inc Orbbec Gemini 335`.
- `lingtu-camera-dds.service`: `LoadState=loaded`, `ActiveState=active`,
  `SubState=running`, `Result=success`, `UnitFileState=disabled`.
- Running processes:
  - `lingtu_camera_dds --domain-id 0 ... --color-topic rt/camera/color --depth-topic rt/camera/depth --info-topic rt/camera/info`
  - `orbbec_capture --color-width 640 --color-height 480 --color-fps 30 --depth-width 640 --depth-height 480 --depth-fps 30 --product-id 0x0800 --device-index 0`
- Status file:
  `/dev/shm/lingtu/camera_status.json` reports `status=running`,
  `color_frames=1659`, `depth_frames=1670`, `info_frames=56`,
  `last_error=""`.
- DDS late-subscriber probe over five seconds:
  - `rt/camera/color`: 132 samples.
  - `rt/camera/depth`: 132 samples.
  - `rt/camera/info`: 5 samples.
- Pure SDK metadata:
  `repo=orbbec/OrbbecSDK_v2`, `ref=latest`,
  `asset=OrbbecSDK_v2.8.7_202606161335_ab8672c_linux_arm64.tar.gz`.

Read:

- Camera DDS is now a real field closed loop on S100P: Orbbec USB device,
  pure SDK capture process, native DDS publisher, active systemd unit, status
  file frames, and DDS subscriber samples are all present.
- The first DDS probe exposed that `CameraInfo` was only published once and
  could be missed by late subscribers. `camera_dds.cpp` was patched to retain
  the latest intrinsics and republish `rt/camera/info` once per second; the
  second probe then received five `camera_info` samples over five seconds.
- Remaining camera work is product deployment polish: install through the
  catalog path, decide whether to enable the optional service at boot, and
  deploy the updated Gateway/service-readiness code so `/api/v1/services/status`
  can report the same camera evidence.

## Native DDS QoS Catalog Recheck

Timestamp: `2026-07-08 13:20 CST`.

Scope:

- Local regression around Python QoS loader, DDS dataflow templates, metrics,
  IDL codegen, service catalog, native camera module, native SLAM contract, and
  LiDAR source contracts.
- S100P compile/runtime verification for native camera, Livox, SLAM,
  traversability, and nav endpoint services.

Fixes made during testing:

- `nav_native_endpoint.cpp`, `traversability_dds.cpp`, and `nav_control.cpp`
  still had local hard-coded DDS QoS. They now include
  `message/cpp/dds_qos_profiles.hpp` and resolve QoS from the shared catalog by
  topic.
- `qos_profiles.yaml` and `dds_qos_profiles.hpp` now cover nav control,
  teleop, clear/cancel, event, grid, terrain-map-ext, and scan-cloud topic
  mappings.
- Remote `build_nav_endpoint.sh` initially failed because
  `src/explore/cpp/explore_contract.hpp` was missing on target. Uploading the
  missing contract restored the full build.
- Camera DDS initially had status frames but old/default probes read zero
  samples after the QoS switch. A QoS-aware probe confirmed the expected reader
  profile requirement. `camera_dds.cpp` was also tightened to set sequence
  `_maximum` and `_release=false` for `Image.data` and `CameraInfo.d`.

Local verification:

- `src/runtime/tests/test_qos_profiles.py ... test_idl_codegen.py`: 172 passed.
- Native camera/service catalog focused tests: 31 passed.
- Native SLAM/LiDAR/nav QoS focused tests: 43 passed.

S100P build verification:

- `build_camera_dds.sh`: passed.
- `build_livox_sdk2_stream.sh`: passed.
- `build_slam_core.sh`: passed.
- `build_nav_endpoint.sh`: passed after syncing `explore_contract.hpp`.

S100P service verification:

- `lingtu-camera-dds.service`: `ActiveState=active`, `SubState=running`,
  `NRestarts=0`, `ExecMainStatus=0`.
- `lingtu-livox-dds.service`: `ActiveState=active`, `SubState=running`,
  `NRestarts=0`, `ExecMainStatus=0`.
- `lingtu-slam-dds.service`: `ActiveState=active`, `SubState=running`,
  `NRestarts=0`, `ExecMainStatus=0`.
- `lingtu-traversability-dds.service`: `ActiveState=active`,
  `SubState=running`, `NRestarts=0`, `ExecMainStatus=0`.
- `lingtu-nav-dds.service`: `ActiveState=active`, `SubState=running`,
  `NRestarts=0`, `ExecMainStatus=0`.

Runtime status:

- Camera status: `color_frames=1524`, `depth_frames=1535`,
  `info_frames=51`, `last_error=""`.
- SLAM status: `state=TRACKING`, `confidence=1.0`,
  `imu_input_hz=200.399`, `lidar_input_hz=9.775`,
  `processed_scan_hz=10.145`, `map_loaded=true`.
- Traversability status: `has_odom=true`, `registered_clouds=1966`,
  `published=1717`, `errors=0`.
- Nav endpoint status: `publish_cmd_vel=false`, `has_odom=true`,
  `has_traversability=true`, `has_terrain_map=true`,
  `has_terrain_map_ext=true`, `cmd_vel_published=0`.

QoS-aware DDS probe over five seconds:

- `rt/camera/color`: 126 samples.
- `rt/camera/depth`: 127 samples.
- `rt/camera/info`: 6 samples.
- `rt/lidar/raw_frame`: 50 samples.
- `rt/imu/raw`: 997 samples.
- `rt/slam/odometry`: 240 samples.
- `rt/slam/registered_cloud`: 49 samples.
- `rt/nav/traversability`: 17 samples.
- `rt/nav/terrain_map`: 16 samples.

Read:

- Python and C++ native DDS paths now use the same named QoS policy surface for
  the tested services.
- Camera, LiDAR, IMU-from-Livox, SLAM, traversability, and nav endpoint data
  are all visible to a late-starting QoS-aware DDS subscriber.
- Gateway service-readiness still needs deployment verification separately;
  this recheck validates native services and DDS topics directly.
