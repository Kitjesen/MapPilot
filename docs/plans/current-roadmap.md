# LingTu Current Roadmap

Current target: a ROS-free, typed-DDS field runtime for Thunder that can pass
no-motion navigation preview before any real velocity output is enabled.

## How To Use This File Across Tasks

Every new Codex task should start here before touching code:

1. Pick one row from **Current Work Board**.
2. Read the linked source-of-truth document before editing.
3. Keep the implementation inside that row's boundary.
4. Record proof in the row's `Evidence` field or in
   `docs/07-testing/field-runs/`.
5. Move the row to `Done` only after a runnable check passes.

Do not create a new plan file unless this file becomes too large to review.

## Product Modes

| Profile | Product Session | Purpose |
| --- | --- | --- |
| `teleop` | `teleop` | Manual velocity commands through Gateway/Teleop/MCP, still gated by safety. |
| `teleop_avoid` | `teleop_avoid` | Manual driving with localization, map, traversability, and collision veto. |
| `map` | `mapping` | Livox + SLAM build map artifacts. |
| `tracking` | `tracking` | Follow supplied waypoints/path without semantic decision making. |
| `nav` | `navigation` | Goal -> OctoPlanner3D -> local planner -> path follower -> cmd_vel. |
| `inspection` | `inspection` | Patrol/task scheduler creates navigation goals and records results. |
| `tare_explore` | `exploration` | TARE/frontier target generation feeding normal navigation. |

`session_mode` remains the low-level Gateway resource state:
`mapping`, `navigating`, or `exploring`. UIs should display
`product_session`.

## Field Data Path

```text
Livox MID-360 / IMU
  -> lingtu-livox-dds
  -> lingtu-slam-dds
  -> odometry + map_cloud + registered_cloud + localization status
  -> map layers / traversability
  -> lingtu-nav-dds
  -> OctoPlanner3D global path
  -> C++ LocalPlanner / PathFollower
  -> /nav/cmd_vel
```

Command ingress is now a separate, single-owner path:

```text
Gateway coordinate goal / resolved semantic goal / teleop request
  -> process-wide NavigationCommandClient
  -> liblingtu_nav_client.so
  -> typed DDS goal_pose / cancel / teleop_cmd_vel
  -> lingtu-nav-dds
```

Within `lingtu-nav-dds`, OctoPlanner3D, LocalPlanner, and PathFollower exchange
C++ objects by direct function call. `/nav/local_path` is published telemetry;
PathFollower consumes the same in-memory local path before that publication.

Map files are not DDS messages:

```text
map.pcd
metadata.json
octomap.ot
occupancy.npz
optional tomogram.pickle
```

## Current Priorities

1. Fix saved-map localization alignment on Sunrise before motion tests.
2. Keep `publish_cmd_vel=false` until no-motion route preview passes.
3. Add an independent final C++ command-safety gate after autonomous
   PathFollower output; current autonomous safety is input/path-level.
4. Add typed command IDs and command acceptance/rejection acknowledgements;
   goal/cancel/teleop writers are already persistent C++ DDS clients.
5. Keep Python DDS adapters out of the field default path.
6. Keep ROS2 only as explicit compatibility adapter code.
7. Validate MapService save -> octomap artifact -> OctoPlanner3D preview.
8. Validate `tare_explore` as exploration target generation, not direct chassis control.
9. Replace Python camera DDS frame ingestion with a versioned C++ SHM image
   ring while retaining typed DDS metadata/health.
10. Make MuJoCo consume final native `/nav/cmd_vel` through the learned
   locomotion sink after using the same C++ SLAM, traversability, and nav
   services as the field endpoint.

## HW Runtime Migration Status

This section tracks the short-role migration. Do not mark a row done from code
shape alone; each row needs either a passing local test or a field evidence file.

| Status | Role | Remaining work | Evidence |
| --- | --- | --- | --- |
| Active | `camera` | Local C++ DDS publisher, color/depth/info IDL/topic/codec/reader contract, and pure OrbbecSDK preference are locked. `DdsCameraModule` now reports per-stream frames/readiness, rejects bad DDS samples without crashing the reader callback, and exposes the upstream catalog boundary (`source_service=camera`, `source_unit=lingtu-camera-dds.service`, ROS topics, DDS topics, and stream contract). The native Orbbec module/build script now labels `OrbbecSDK_ROS2/orbbec_camera/SDK` as `ros2_wrapper_fallback` instead of letting it look like the pure SDK path. `lingtu-camera-dds.service` and `run_camera_dds.sh` now pass explicit Orbbec capture selection/stream parameters (`product_id`, `device_index`, optional serial/uid/sdk_config/frame_sync, color/depth size/fps, and timeouts) instead of relying on capture-process defaults. The field readiness collector now emits a dedicated `camera` section that requires `lingtu-camera-dds.service` active, `camera_dds` and `orbbec_capture` executable, `/dev/shm/lingtu/camera_status.json` with nonzero color/depth/info frame counts, DDS samples on `rt/camera/color`, `rt/camera/depth`, and `rt/camera/info`, and no legacy `orbbec_camera` ROS2 wrapper process. S100P read-only recheck at `2026-07-08 09:42:57 CST` shows `lingtu-camera-dds.service` is not found, both `lingtu_camera_dds` and `orbbec_capture` are missing, `/dev/shm/lingtu/camera_status.json` is missing, no Orbbec USB device was observed, the pure `deps/orbbec/OrbbecSDK` path is missing, and only the legacy `/opt/lingtu/current/src/drivers/real/camera/OrbbecSDK_ROS2` path was observed. Remaining work is build, install, start, and sample `lingtu_camera_dds` on S100P using the pure OrbbecSDK path where available. | `src/runtime/tests/test_message_contract_location.py::test_camera_dds_topics_match_native_camera_publisher`; `src/runtime/tests/test_dds_typed_wire_adapter.py::test_camera_images_round_trip_through_registered_dds_payloads`; `src/runtime/tests/test_dds_typed_wire_adapter.py::test_camera_info_round_trips_through_registered_dds_payload`; `tests/drivers/test_orbbec_native_camera_module.py::test_orbbec_native_health_reports_sdk_boundary`; `tests/drivers/test_orbbec_native_camera_module.py::test_orbbec_native_build_includes_service_sdk`; `tests/drivers/test_orbbec_native_camera_module.py::test_camera_dds_cpp_publisher_has_typed_writers_and_build_target`; `tests/drivers/test_camera_dds_module.py`; `src/runtime/tests/test_thunder_deployment_entrypoints.py::test_thunder_camera_dds_service_is_optional_and_fails_without_native_publisher`; `src/runtime/tests/test_thunder_service_catalog.py::test_service_manager_camera_native_binary_missing_blocks_readiness`; `src/runtime/tests/test_thunder_service_readiness_collect.py::test_thunder_service_readiness_accepts_camera_closed_loop_evidence`; `src/runtime/tests/test_thunder_service_readiness_collect.py::test_thunder_service_readiness_blocks_incomplete_camera_closed_loop`; `src/runtime/tests/test_thunder_service_readiness_collect.py::test_thunder_service_readiness_collector_reports_missing_camera_binaries`; `docs/07-testing/field-runs/2026-07-08-sunrise-runtime-migration-status.md`; `artifacts/thunder_field_runtime/192_168_66_13_readonly_camera_service_sample.json`. |
| Active | `lidar` | Local policy now treats `lidar_start_driver`, `start_driver=True`, legacy driver sensor fallback, legacy sim LiDAR, and `legacy_lidar` service selection as explicit compatibility paths. Default `create_lidar_source()` returns the ROS-free SDK2 source without loading the ROS2 adapter, while the ROS2 `livox_ros_driver2` process factory is lazy and only reachable through the compatibility path. Product profiles/endpoints do not set legacy driver-start keys, `ServiceManager.start("lidar")` prefers native `lingtu-livox-dds.service`, managed SLAM wiring prefers canonical `lidar.raw_scan/imu`, and unknown or unimplemented LiDAR backends fail fast instead of falling back to MID-360/ROS2. S100P read-only recheck at `2026-07-08 09:42:57 CST` shows native `livox_sdk2_stream --dds`, no observed `livox_ros_driver2`, and SLAM `lidar_input_hz=9.130011`. The field readiness collector now emits a `lidar_imu` section that expects `livox_dds`/`livox_sdk2_stream` to own both `rt/lidar/raw_frame` and `rt/imu/raw`, and blocks `livox_ros_driver2` or suspected duplicate IMU writers. Remaining work is DDS sampling with the current collector after deployment to prove the same main path at topic level. | `src/runtime/tests/test_runtime_binding_policy.py`; `src/runtime/tests/test_runtime_catalogs.py::test_catalog_profiles_do_not_start_legacy_lidar_driver`; `src/runtime/tests/test_runtime_catalogs.py::test_runtime_endpoints_do_not_start_legacy_lidar_driver`; `src/runtime/tests/test_service_manager.py::test_start_lidar_prefers_native_dds_when_installed`; `src/runtime/tests/test_service_manager.py::test_start_legacy_lidar_uses_explicit_compat_alias`; `src/runtime/tests/test_stack_registry_resolution.py::test_lidar_stack_rejects_unknown_backend_without_mid360_fallback`; `src/runtime/tests/test_stack_registry_resolution.py::test_lidar_stack_rejects_declared_unimplemented_backends`; `src/runtime/tests/test_profile_graph_snapshots.py::test_thunder_field_profiles_use_endpoint_only_command_boundary_by_default`; `tests/drivers/test_lidar_module_source.py::test_create_lidar_source_builds_default_sdk2_source_without_connecting`; `tests/drivers/test_lidar_module_source.py::test_lidar_stack_default_does_not_enable_legacy_livox_driver`; `tests/drivers/test_lidar_module_source.py::test_lidar_ros2_process_factory_is_lazy_compatibility_path`; `src/runtime/tests/test_thunder_service_readiness_collect.py::test_thunder_service_readiness_accepts_livox_lidar_imu_owner`; `src/runtime/tests/test_thunder_service_readiness_collect.py::test_thunder_service_readiness_blocks_legacy_livox_and_duplicate_imu_owner`; `tests/drivers/test_lidar_module_source.py`; `docs/07-testing/field-runs/2026-07-08-sunrise-runtime-migration-status.md`; `artifacts/thunder_field_runtime/192_168_66_13_readonly_camera_service_sample.json`. |
| Active | `imu` | Local contract keeps Livox IMU as `lidar.imu` source for SLAM and treats independent `imu/livox` as a facade that publishes no duplicate samples. `imu/dds` now reports itself as an explicit diagnostic/profile reader, marks that it publishes samples only when deliberately enabled, and flags that its default source is the same `rt/imu/raw` Livox topic already owned by the LiDAR path. Unknown IMU backends and declared-but-unimplemented `replay` fail fast instead of falling back to Livox. S100P read-only recheck at `2026-07-08 09:42:57 CST` shows SLAM `imu_input_hz=200.539856` from the native Livox path, with no suspected duplicate IMU process observed. The service readiness collector now records duplicate IMU writer risk separately from generic process blockers. Remaining work is proving no duplicate IMU writer exists after current deployment with DDS sampling enabled. | `src/runtime/tests/test_stack_registry_resolution.py::test_imu_stack_rejects_unknown_backend_without_livox_fallback`; `src/runtime/tests/test_stack_registry_resolution.py::test_imu_stack_rejects_declared_unimplemented_replay_backend`; `src/localization/tests/test_native_slam_contract.py::test_slam_wiring_prefers_lidar_imu_over_independent_imu_role`; `tests/drivers/test_imu_dds_module.py::test_livox_imu_backend_is_facade_for_lidar_imu_not_second_publisher`; `tests/drivers/test_imu_dds_module.py::test_livox_imu_facade_and_dds_reader_expose_different_risk_boundaries`; `tests/drivers/test_imu_dds_module.py::test_imu_dds_backend_is_registered`; `src/runtime/tests/test_thunder_service_readiness_collect.py::test_thunder_service_readiness_accepts_livox_lidar_imu_owner`; `src/runtime/tests/test_thunder_service_readiness_collect.py::test_thunder_service_readiness_blocks_legacy_livox_and_duplicate_imu_owner`; `docs/07-testing/field-runs/2026-07-08-sunrise-runtime-migration-status.md`; `artifacts/thunder_field_runtime/192_168_66_13_readonly_camera_service_sample.json`. |
| Active | `driver` | Local defaults now keep MuJoCo driver sensor publishing off at both stack-config and bare-module constructor level. Legacy in-driver camera/lidar/imu ports remain visible for compatibility, health marks them as legacy with `sensor_ports_legacy=true`, and reports canonical owners as short roles (`camera/sim`, `lidar/mujoco`, `imu/mujoco`). `use_driver_*` plus `legacy_driver_sensor_fallback` are audited opt-ins, and `SimPointCloudProvider` is explicitly marked as legacy `sim_lidar` rather than canonical `lidar/mujoco`. Remaining work is cleanup of older docs/profiles after field validation. | `tests/drivers/test_mujoco_driver_contract.py`; `tests/drivers/test_sim_pointcloud.py`; `src/runtime/tests/test_stack_config.py`; `src/runtime/tests/test_runtime_binding_policy.py::test_legacy_sensor_binding_violations_report_driver_sensor_paths`; `src/localization/tests/test_native_slam_contract.py`; `src/runtime/tests/test_sim_semantic_pipeline_blueprint.py`. |
| Active | `hw` | Local contract now keeps top-level stack exports on short `hw`, leaves `system.device_manager()` as compat only, rejects old profile keys, documents/tests `devices.yaml` as physical inventory rather than sensor stream wiring, and reports `backend=inventory`, `sensor_streams=false`, and inventory-only scope through `Hw.health()`. Remaining work is field health proving `hw` replaces visible `DeviceManager` naming. | `src/runtime/tests/test_devices.py`; `src/runtime/tests/test_runtime_catalogs.py::test_catalog_profiles_do_not_use_device_manager_compat_keys`. |
| Active | `service` | Install entrypoint now resolves `field-cpp` and single-service modes through catalog service names (`install-services`) and calls the shared `install_catalog_service.sh` path, while per-service wrappers remain compatibility entrypoints. Service manager readiness details, Gateway `/api/v1/services/status`, catalog CLI, install enable defaults, and the read-only field service readiness collector resolve product services, units, native binary readiness, status files, GNSS device readiness, legacy ROS2/wrapper process blockers, and DDS topic contracts through catalog/config names. Camera now declares `native_binary` readiness for `lingtu_camera_dds` and `orbbec_capture`; core field DDS services now declare the same readiness boundary for `livox_dds`, `slam_dds`, `nav_dds`, and `traversability_dds` using the same env/path names as the service units. ServiceManager/Gateway live status can block on those binaries before DDS/status-file checks, and the collector reports the same blockers from a read-only robot-side sweep. The collector emits a top-level `summary.ok`, `summary.blockers`, and `summary.blocker_count` with source-prefixed blockers, and Gateway `/api/v1/services/status` emits `readiness.ok`, `readiness.blockers`, `ready_services`, and `not_ready_services` from the live `ServiceManager` detail payload. Gateway can also read the collector JSON from `LINGTU_SERVICE_READINESS_JSON` (default `/tmp/lingtu_service_readiness.json`) and expose it as `field_readiness`; `LINGTU_REQUIRE_FIELD_SERVICE_READINESS=1` makes missing, stale, or failed field evidence block the Gateway summary. Gateway service-status defaults now derive from `thunder_field_readiness_services()` plus the `gateway` HTTP alias instead of carrying a hand-written field service list. Service readiness now distinguishes an installed-but-inactive unit from a missing unit via `systemd_unit_missing:<unit>`, while the standalone field collector records `LoadState`, `UnitFileState`, and `missing` for each catalog unit. Traversability declares its DDS outputs (`traversability`, `terrain_map`, `terrain_map_ext`) instead of carrying an uncheckable `dds` readiness flag. S100P read-only recheck at `2026-07-08 09:42:57 CST` shows core native units active and executable, camera unit/binaries missing, and deployed Gateway still returns 404 for `/api/v1/services/status`; current local code still needs deployment and `--dds-seconds` sampling before readiness can be proven. | `src/runtime/tests/test_thunder_service_catalog.py`; `src/runtime/tests/test_thunder_deployment_entrypoints.py`; `src/runtime/tests/test_thunder_service_readiness_collect.py`; `python -m runtime.service_catalogs.thunder install-services field-cpp` with `PYTHONPATH=src`; `python -m runtime.service_catalogs.thunder install-plan field-cpp` with `PYTHONPATH=src`; `python -m runtime.service_catalogs.thunder readiness-units` with `PYTHONPATH=src`; `python -m runtime.service_catalogs.thunder status-files` with `PYTHONPATH=src`; `python -m runtime.service_catalogs.thunder readiness-dds-topics` with `PYTHONPATH=src`; `python -m runtime.service_catalogs.thunder install-enable-default traversability` with `PYTHONPATH=src`; `scripts/gates/thunder_service_readiness_collect.py --dds-seconds N`; `src/runtime/tests/test_thunder_service_catalog.py::test_service_manager_missing_unit_blocks_readiness_with_concrete_unit`; `src/runtime/tests/test_thunder_service_catalog.py::test_service_manager_camera_native_binary_missing_blocks_readiness`; `src/runtime/tests/test_thunder_service_catalog.py::test_service_manager_core_native_binary_missing_blocks_readiness`; `src/runtime/tests/test_thunder_service_readiness_collect.py::test_thunder_service_readiness_collector_marks_missing_systemd_units`; `src/runtime/tests/test_thunder_service_readiness_collect.py::test_thunder_service_readiness_collector_reports_missing_camera_binaries`; `src/runtime/tests/test_thunder_service_readiness_collect.py::test_thunder_service_readiness_collector_accepts_executable_camera_binaries`; `src/runtime/tests/test_thunder_service_readiness_collect.py::test_thunder_service_readiness_collector_marks_missing_gnss_device`; `src/runtime/tests/test_thunder_service_readiness_collect.py::test_thunder_service_readiness_collector_flags_legacy_ros2_processes`; `src/runtime/tests/test_thunder_service_readiness_summary_aggregates_blockers`; `src/gateway/tests/test_gateway_health_contract.py::test_service_status_marks_current_gateway_http_observed`; `src/gateway/tests/test_gateway_health_contract.py::test_service_status_summarizes_service_readiness_blockers`; `src/gateway/tests/test_gateway_health_contract.py::test_service_status_can_require_field_readiness_evidence`; `src/gateway/tests/test_gateway_health_contract.py::test_service_status_includes_fresh_field_readiness_summary`; `src/gateway/tests/test_gateway_health_contract.py::test_service_status_marks_stale_field_readiness_evidence`; `src/gateway/tests/test_gateway_session_map_contract.py::test_service_status_exposes_catalog_readiness_contract`; `src/gateway/tests/test_gateway_session_map_contract.py::test_service_status_default_names_follow_field_readiness_catalog`; `python -m pytest src/runtime/tests/test_thunder_service_catalog.py src/runtime/tests/test_thunder_service_readiness_collect.py src/gateway/tests/test_gateway_session_map_contract.py::test_service_status_exposes_catalog_readiness_contract src/gateway/tests/test_gateway_health_contract.py::test_service_status_summarizes_service_readiness_blockers -q`; `docs/07-testing/field-runs/2026-07-08-sunrise-runtime-migration-status.md`; `artifacts/thunder_field_runtime/192_168_66_13_validation.json`; `artifacts/thunder_field_runtime/192_168_66_13_readonly_camera_service_sample.json`. |
| Active | `profiles` | Every `thunder_field.supported_profiles` entry now has static graph and runtime run-spec coverage proving Python-owned `hw`, robot driver, LiDAR, IMU, nav/map output adapters, local planner/path follower stages, LiDAR/IMU backend keys, and audited legacy sensor bindings stay off by default. Semantic field profiles use short `camera` with Orbbec, not `CameraBridgeModule`. The managed local mapping branch is separately locked: bypassing `thunder_field` with `robot_preset=thunder` keeps `map` on canonical `lidar.raw_scan/imu -> SlamModule`. Remaining work is field evidence proving the deployed profile matches these local snapshots. | `src/runtime/tests/test_profile_graph_snapshots.py::test_thunder_field_profiles_use_endpoint_only_command_boundary_by_default`; `src/runtime/tests/test_profile_graph_snapshots.py::test_thunder_field_semantic_profiles_use_orbbec_camera`; `src/runtime/tests/test_profile_graph_snapshots.py::test_managed_map_profile_uses_canonical_lidar_when_field_endpoint_is_bypassed`; `src/runtime/tests/test_runtime_resolver.py::test_thunder_field_run_specs_never_manage_sensor_or_driver_services`; `src/runtime/tests/test_runtime_resolver.py::test_map_profile_default_field_endpoint_does_not_manage_lidar`; `src/runtime/tests/test_runtime_resolver.py::test_managed_map_profile_uses_local_lidar_when_endpoint_is_bypassed`; `src/runtime/tests/test_runtime_catalogs.py::test_real_runtime_endpoints_do_not_select_legacy_driver_sensor_paths`; `src/runtime/tests/test_runtime_catalogs.py::test_real_product_profiles_do_not_select_legacy_driver_sensor_paths`; `src/runtime/tests/test_runtime_binding_policy.py`. |
| Active | `nav` | Re-run field evidence with command output enabled only after no-motion and safety gates pass. Current read-only gate has no `/nav/cmd_vel` sample and no hw command boundary. | `artifacts/thunder_field_runtime/192_168_66_13_validation.json`. |
| Active | `gnss` | Direct serial, explicit `hw` bridge, explicit `dds`, and `replay` backend contracts are locked. Static profile graph now mirrors the runtime backend rule, `devices.yaml` inventory does not implicitly choose GNSS dataflow, unknown explicit GNSS backends fail fast instead of silently falling back, and `gnss(enabled=True, backend=...)` can create explicit non-serial roles without requiring `robot_config.yaml` to enable GNSS first. `GnssModule.health()` and `GNSS_CONTRACT.health_fields` now expose `dataflow_owner`, `serial_port`, `uses_hw_inventory`, `requires_hw_bridge`, `dds_compat_reader`, `replay_source`, and `direct_serial` so Gateway/status can distinguish direct, hw bridge, DDS compat, and replay paths. S100P read-only recheck at `2026-07-08 08:40:25 CST` shows `robot_config.yaml` declares `gnss.enabled=true` and `/dev/wtrtk980`, while `/dev/wtrtk980`, `/dev/ttyUSB*`, `/dev/serial/by-id/*`, GNSS/NTRIP/RTK processes, and matching systemd units are all absent. Remaining work is to restore or create the stable GNSS device path, prove direct serial or explicit bridge/DDS samples, and then verify RTK/NTRIP/fusion health. | `src/runtime/tests/test_full_stack_composition.py::test_gnss_stack_prefers_configured_serial_device`; `src/runtime/tests/test_full_stack_composition.py::test_gnss_stack_does_not_use_devices_yaml_inventory_as_dataflow`; `src/runtime/tests/test_full_stack_composition.py::test_gnss_stack_can_use_hw_bridge_explicitly`; `src/runtime/tests/test_full_stack_composition.py::test_gnss_stack_can_use_backend_key_for_dds_without_serial_or_hw`; `src/runtime/tests/test_full_stack_composition.py::test_gnss_stack_can_use_replay_backend_without_serial_or_hw`; `src/runtime/tests/test_full_stack_composition.py::test_gnss_stack_enabled_argument_can_create_non_serial_role_without_config`; `src/runtime/tests/test_full_stack_composition.py::test_gnss_stack_enabled_argument_can_create_hw_bridge_without_config`; `src/runtime/tests/test_full_stack_composition.py::test_gnss_stack_rejects_unknown_explicit_backend`; `src/runtime/tests/test_gnss_config_and_ntrip.py::TestNtripRtcmAutoWire::test_gnss_health_distinguishes_direct_dds_and_replay_backends`; `src/runtime/tests/test_gnss_config_and_ntrip.py`; `src/runtime/tests/test_contracts.py::test_gnss_contract_defines_generic_position_sensor_boundary`; `src/runtime/tests/test_profile_graph_snapshots.py::test_static_profile_graph_honors_gnss_backend_without_hw_bridge`; `docs/07-testing/field-runs/2026-07-08-sunrise-runtime-migration-status.md`. |
| Later | aliases | Remove `CameraBridgeModule`, `LidarModule`, and `DeviceManager` aliases only after field profiles no longer show them. | Current field health still reports `CameraBridgeModule`. |

Service evidence correction: the summary aggregation test path is
`src/runtime/tests/test_thunder_service_readiness_collect.py::test_thunder_service_readiness_summary_aggregates_blockers`.

Service evidence update: `thunder_field_native_binaries()` and the catalog CLI
`readiness-binaries` are now the source of truth for field binary readiness.
`scripts/gates/thunder_service_readiness_collect.py` derives
`NATIVE_BINARY_DEFAULTS` and `NATIVE_BINARY_ENV` from that catalog export.
`scripts/deploy/thunder/install_services.sh` also derives catalog mode help from
`install-modes` instead of keeping a separate hand-written product mode list.
Verified with:

```bash
python -m pytest src/runtime/tests/test_thunder_service_catalog.py src/runtime/tests/test_thunder_service_readiness_collect.py src/gateway/tests/test_gateway_session_map_contract.py::test_service_status_exposes_catalog_readiness_contract src/gateway/tests/test_gateway_health_contract.py::test_service_status_summarizes_service_readiness_blockers -q
PYTHONPATH=src python -m runtime.service_catalogs.thunder readiness-binaries
python -m pytest src/runtime/tests/test_thunder_deployment_entrypoints.py::test_thunder_service_installer_defaults_to_native_field_cpp_stack src/runtime/tests/test_thunder_service_catalog.py src/runtime/tests/test_thunder_service_readiness_collect.py -q
PYTHONPATH=src python -m runtime.service_catalogs.thunder install-modes
```

Camera DDS evidence update: native `CameraInfo` now carries `depth_scale` end to
end. The IDL, Python DDS type, generated mirror, Python codec, C++ camera DDS
publisher, and runtime message-format contract all include the field, so
`CameraIntrinsics.depth_scale` no longer silently falls back to `1.0` when
camera info crosses native DDS. This is still a local contract/code path update;
S100P build/install/sample of `lingtu_camera_dds` remains required. The camera
DDS build helper now mirrors the nav endpoint CycloneDDS prefix/multiarch setup,
uses `--parallel`, and fails if `lingtu_camera_dds` is not executable after the
build, so the S100P build step has the same package-prefix protection as other
native DDS services.
Verified with:

```bash
python -m pytest tests/drivers/test_camera_dds_module.py tests/drivers/test_orbbec_native_camera_module.py::test_camera_dds_cpp_publisher_has_typed_writers_and_build_target src/runtime/tests/test_dds_typed_wire_adapter.py src/runtime/tests/test_contracts.py -q
python -m pytest tests/drivers/test_camera_dds_module.py tests/drivers/test_orbbec_native_camera_module.py::test_camera_dds_cpp_publisher_has_typed_writers_and_build_target src/runtime/tests/test_thunder_deployment_entrypoints.py::test_thunder_camera_dds_service_is_optional_and_fails_without_native_publisher src/runtime/tests/test_dds_typed_wire_adapter.py::test_camera_info_round_trips_through_registered_dds_payload src/runtime/tests/test_dds_typed_wire_adapter.py::test_camera_images_round_trip_through_registered_dds_payloads -q
```

Camera SDK evidence update: `scripts/build/fetch_orbbec_sdk.sh` is now the
canonical pure OrbbecSDK_v2 fetch path for
`src/drivers/real/camera/deps/orbbec/OrbbecSDK`. `build_orbbec_native.sh`
points fallback users to that script instead of making the ROS2 wrapper look
like the preferred SDK source. The fetch script supports latest or tagged
GitHub release assets, ARM64/x64 SDK packages, `.deb`, `.tar.gz`, `.tgz`, and
`.zip` extraction, refuses to overwrite unmanaged SDK directories, and marks
managed installs with `.lingtu-orbbec-sdk`. This still needs an S100P
network/build run and an attached Orbbec device before it counts as field
closure.
Verified with:

```bash
bash -n scripts/build/fetch_orbbec_sdk.sh
python -m pytest tests/drivers/test_camera_layout_contract.py tests/drivers/test_orbbec_native_camera_module.py::test_orbbec_native_build_includes_service_sdk -q
python -m pytest src/runtime/tests/test_thunder_deployment_entrypoints.py::test_thunder_camera_dds_service_is_optional_and_fails_without_native_publisher tests/drivers/test_orbbec_native_camera_module.py::test_camera_dds_cpp_publisher_has_typed_writers_and_build_target tests/drivers/test_camera_dds_module.py -q
```

Camera field closure update: after the Orbbec Gemini 335 was plugged into
Sunrise, the native camera path was built and started on S100P using the pure
OrbbecSDK v2 ARM64 release asset. `lingtu-camera-dds.service` is loaded,
active, and running but intentionally disabled for boot until the catalog
deployment path is run. `/dev/shm/lingtu/camera_status.json` reports running
color/depth/info frame counts, and a late-subscriber DDS probe received
`rt/camera/color=132`, `rt/camera/depth=132`, and `rt/camera/info=5` samples
over five seconds. The first probe exposed that `CameraInfo` could be missed
when it was only written once; `camera_dds.cpp` now republishes the latest
camera info once per second. Remaining camera work is no longer capture/DDS
closure; it is catalog install polish, boot enable policy, and deployed
Gateway readiness reporting.
Evidence:

- `docs/07-testing/field-runs/2026-07-08-sunrise-runtime-migration-status.md`
- `artifacts/thunder_field_runtime/192_168_66_13_camera_dds_closed_loop.json`
- `python -m pytest tests/drivers/test_orbbec_native_camera_module.py::test_camera_dds_cpp_publisher_has_typed_writers_and_build_target -q`
- `python -m pytest tests/drivers/test_camera_dds_module.py -q`

## Current Work Board

| Status | Work | Boundary | Evidence |
| --- | --- | --- | --- |
| Active | Align SLAM localization with the active saved map on Sunrise. | `lingtu-slam-dds`, relocalization, map frame, `map_odom_tf`. | Field note under `docs/07-testing/field-runs/`. |
| Active | Prove no-motion navigation preview. | Gateway goal API, MapService, OctoPlanner3D, native nav endpoint. | `validate_plan` returns OctoPlanner3D path and `publish_cmd_vel=false`. |
| Done locally | Unify field navigation command ingress. | Persistent C++ goal/cancel/teleop client, semantic goal routing, no Gateway subprocess or Python DDS writer. | `test_nav_client`, Gateway command tests, route-contract tests. |
| Done locally | Separate endpoint control ownership. | `autonomy`, `teleop`, and `teleop_avoid` modes; reject cross-mode commands; pure teleop does not require SLAM. | `test_input_gate`, `test_teleop_safety`, `test_nav_endpoint_config`; full endpoint links. |
| Active | Add final autonomous command safety. | Independent last-chance stale/localization/obstacle/velocity veto after PathFollower and before the single `/nav/cmd_vel` writer. | Fault-injection tests prove every unsafe final command becomes zero or bounded. |
| Active | Finish typed C++ nav command feedback. | Request ID, accepted/rejected acknowledgement, final reason. | Client can distinguish DDS delivery from endpoint acceptance. |
| Active | Validate map package path. | `map.pcd`, `metadata.json`, `octomap.ot`, `occupancy.npz`. | Saved map is navigation-ready and active. |
| Next | Close native MuJoCo command sink. | Same typed DDS/C++ services as field; final cmd_vel drives learned locomotion policy. | Goal -> paths -> cmd_vel -> body motion evidence with no Python planner substitute. |
| Next | Remove Python camera DDS hot path. | C++ SHM image ring plus DDS metadata/health. | Field Module graph has no cyclonedds-python camera dependency. |
| Next | Validate `tare_explore`. | TARE target generation only; navigation still owns planning/execution. | Exploration target becomes normal `goal_pose`. |
| Blocked | Real motion smoke. | CmdVelMux and robot command sink. | Blocked until no-motion preview and safety gates pass. |

## Source Of Truth

| Topic | Current document |
| --- | --- |
| Product/session naming | `docs/architecture/PRODUCT_MODE_RUNTIME_CONTRACT.md` |
| Navigation dataflow | `docs/architecture/NAVIGATION_RUNTIME_DATAFLOW.md` |
| Global planner contract | `docs/architecture/GLOBAL_PLANNING_CONTRACT.md` |
| Map lifecycle | `docs/architecture/MAP_SERVICE_CONTRACT.md` |
| Runtime bus | `docs/architecture/LINGTU_RUNTIME_BUS_DECISION.md` |
| Deployment | `docs/04-deployment/README.md` |
| Robot CLI | `docs/04-deployment/lingtu_cli.md` |

## Done When

- `lingtu mode switch map/nav/tracking/inspection/tare_explore` reports the
  expected `product_session`.
- Sunrise services are active: `lingtu-livox-dds`, `lingtu-slam-dds`,
  `lingtu-traversability-dds`, `lingtu-nav-dds`, and `lingtu`.
- SLAM publishes stable odometry inside the active map frame.
- `/api/v1/maps/{name}/validate_plan` returns an OctoPlanner3D path with no
  fallback to PCT/direct/A*.
- Local planner receives odometry, global path, registered cloud, and
  traversability input before speed output is enabled.
- Motion smoke proves cmd_vel only after safety and no-motion gates pass.
