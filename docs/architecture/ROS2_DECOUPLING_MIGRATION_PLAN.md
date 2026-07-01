# ROS 2 Decoupling Migration Plan

Status: proposal (not yet approved, not yet started)
Scope: the remaining C++ SLAM, base-autonomy, and sensor-ingestion ROS 2 surface. The Python Module-First framework is already ROS2-optional by construction.
Builds on: [`LINGTU_RUNTIME_BUS_DECISION.md`](./LINGTU_RUNTIME_BUS_DECISION.md), [`ROS_ROLE_REPLACEMENT_MAP.md`](./ROS_ROLE_REPLACEMENT_MAP.md), [`ros_frame_contract.md`](./ros_frame_contract.md), [`PORTABLE_LEAN_PACKAGE_MATRIX.md`](./PORTABLE_LEAN_PACKAGE_MATRIX.md), `docs/plans/PRD-lingtu-native-slam-navigation-runtime.md`, `docs/plans/PRD-slam-transport-navigation-dataflow.md`, [`THUNDER_RUNTIME_REFACTOR_PLAN.md`](./THUNDER_RUNTIME_REFACTOR_PLAN.md) (style/structure reference).
Does not modify: any SLAM/perception/planning/build code. This document only.

## 0. Executive Summary

LingTu's Python layer is architecturally ROS2-independent by design: Modules talk through `In[T]`/`Out[T]` ports over a pluggable Transport (Local callback, DDS/CycloneDDS, SHM, Adapter), and CycloneDDS is already a production dependency on S100P (`~/cyclonedds/install/`, `cyclonedds==0.10.5`). **Current-state caveat**: the architecture itself is sound, but the endpoint-transport/evidence layer that implements it is mid-refactor today, not settled — a full run of `python -m pytest src/runtime/tests/ -q` on the current working tree reports 113 failed / 2636 passed / 19 skipped, with roughly 40 of those failures concentrated in exactly this layer (`test_runtime_evidence.py`, `test_transport.py`, `test_transport_wiring.py`, `test_lcm_endpoint_contracts.py`, `test_cli_no_repl.py`, `test_runtime_display.py`, `test_runtime_catalogs.py`, `test_deployment_service_contracts.py`, `test_run_state.py`, `test_module_boundaries.py`). See §2.6a for the concrete evidence and root-cause breakdown; it does not change the target architecture, the technical approach, or the phase order below, but it does add a stabilization work item to Phase 0 (§7) before Phase 2+ builds native C++ output paths on top of this layer. The remaining hard ROS 2 dependency — the thing that pins the robot to Ubuntu 22.04 / Humble — lives entirely in the C++ layer (`rclcpp` SLAM and base-autonomy nodes), two vendored third-party ROS2 driver packages (Livox, Orbbec), a handful of Python `rclpy`/`ros2`-CLI bridge and compatibility-adapter files, and the `colcon`/`ament_cmake` build graph that ties all of it together.

Critically, this is **not a green-field decision**. The repository already contains three converging bodies of prior art that this plan builds on rather than replaces:

1. A **transport policy decision** (`LINGTU_RUNTIME_BUS_DECISION.md`) that LCM is the default for cross-process/network endpoints, typed DDS is used at selected high-performance boundaries, and SHM is used for same-host bulk data — i.e. the target is **not** "replace ROS 2 with raw DDS everywhere," it is "finish routing the C++ boundary through the multi-transport policy that Python modules already use."
2. A **role-replacement map and native SLAM PRD** describing `ISlamBackend`, a pure-CycloneDDS SLAM runtime (`src/localization/slam/cpp/cyclone_runtime.cpp`), a native message/IDL layer (`src/message/idl/lingtu_slam.idl`, `src/message/dds.py`), and a ROS-free transform library (`src/runtime/tf/`) — all already written, but **not yet the production default** on S100P.
3. **Executed precedent**: `src/nav/local/legacy_ros/{terrain_analysis,terrain_analysis_ext,local_planner,sensor_scan_generation}` are already `COLCON_IGNORE`'d and superseded by the nanobind `nav_kernel` / `local_planner/cpp` standalone-CMake native path. This is proof the pattern works end-to-end for one full subsystem.

**Recommended approach**: standardize the C++ boundary on the already-adopted CycloneDDS/typed-IDL + LCM-endpoint policy (no new IPC technology), finish and field-validate the native paths that already exist in prototype form, and promote them to production defaults one bounded subsystem at a time — using the existing `slam_profile`/backend-registry pattern (already supports `fastlio2` / `pointlio` / `localizer` / `bridge` / `none`) as the built-in rollback mechanism, exactly as it works today.

**Recommended phase order** (detail in §7): (0) inventory/guardrails → (1) formalize base-autonomy retirement (already ~done) → (2) LiDAR ingestion (Livox-SDK2 native) → (3) camera ingestion (native Orbbec capture) → (4) GNSS confirmation (mostly already native) → (5) Fast-LIO2 native cutover → (6) PGO + Localizer native parity → (7) production promotion on S100P → (8) build/CI/deployment consolidation, ROS 2 becomes an explicit opt-in compatibility profile. SLAM is deliberately staged last among the runtime subsystems because it is the most complex and the only one directly in the safety-critical localization path.

**Biggest open question** (detail in §12): whether the founder's actual requirement is "no ROS 2 at all" or "no forced Ubuntu 22.04 / Humble pin" — the latter could be satisfied far more cheaply by tracking a newer ROS 2 distribution (e.g. Jazzy on 24.04) for the compatibility profile, while still doing the native work for performance/portability reasons. This plan assumes the founder's stated intent (full removal from the product's main path) is correct, but the cost/benefit math changes materially if only the OS pin matters.

## 1. Motivation and Problem Statement

Today, the production S100P navigation stack requires `source /opt/ros/humble/setup.bash` for its C++ localization and (historically) base-autonomy nodes, and its build is a `colcon`/`ament_cmake` graph (`make build`, per `CLAUDE.md`/`AGENTS.md`). ROS 2 Humble is only supported on Ubuntu 22.04. Concretely, this means:

- The robot's OS image, cross-compilation toolchain, and every future hardware target (successor boards, alternative compute modules, CI runners) inherit an Ubuntu-22.04-or-nothing constraint.
- Any Windows-side development or CI (e.g. the portable Fast-LIO2 gate workflow, see §2.11) must maintain a second, parallel non-ROS build path by hand, which the repository already does in prototype form — evidence that maintaining two build worlds is real, ongoing engineering cost, not a hypothetical one.
- `colcon`/`ament_cmake` per-package build orchestration is slower and more complex than the plain-CMake standalone builds LingTu already uses successfully for `nav_kernel` and the C++ local planner.
- The founder's stated business motivation is to decouple the robot software stack from ROS 2 Humble / Ubuntu 22.04 specifically, so LingTu is not locked into a single OS/middleware-distribution lifecycle as it scales to new hardware and (potentially) new deployment environments.

This plan enumerates exactly what remains ROS2-coupled today, confirms what already has a working native alternative, and sequences a field-safe cutover — because the only genuinely safety-critical subsystem in scope (localization) runs on a legged robot outdoors, where a bad migration is a physical-hardware and human-safety risk, not just a software regression.

## 2. Current-State ROS 2 Coupling Inventory

All paths below were verified against the current working tree on `codex/simulation-architecture-hardening` (uncommitted `src/core/` → `src/runtime/` refactor in progress; this inventory already reflects `src/runtime/...` post-refactor paths, not the older `src/core/...` paths in `CLAUDE.md`).

### 2.1 SLAM / Localization — `src/localization/`

| Package | Node / entry point | ROS 2 surface |
| --- | --- | --- |
| `src/localization/fastlio2/` | `src/lio_node.cpp` | `rclcpp::Node`, publishes `/cloud_registered`, `/cloud_map`, `/Odometry`; advertises service `save_map` (`interface::srv::SaveMaps`) |
| `src/localization/pointlio/` | `src/laserMapping.cpp` | `rclcpp` node, Fast-LIO2-compatible `save_map` service |
| `src/localization/pgo/` | `src/pgo_node.cpp` | `rclcpp::Node`, subscribes odometry via `message_filters`, publishes `/pgo/loop_markers`, broadcasts `map`→`odom` via `tf2_ros::TransformBroadcaster`, advertises `/pgo/save_maps` (`SaveMaps.srv`) |
| `src/localization/localizer/` | `src/localizer_node.cpp` | `rclcpp::Node`, ICP relocalization against a static PCD map, advertises `relocalize` (`Relocalize.srv`), `relocalize_check` (`IsValid.srv`), `global_relocalize`/`global_relocalize_status` (`std_srvs::Trigger`) |
| `src/localization/hba/` | `src/hba_node.cpp` | `rclcpp::Node`, advertises `refine_map` (`RefineMap.srv`), `save_poses` (`SavePoses.srv`) |
| `src/localization/interface/` | `srv`/`msg` package | `ament_cmake` interface-generation package (`SaveMaps.srv`, `SavePoses.srv`, `Relocalize.srv`, `IsValid.srv`, `RefineMap.srv`) — every service above depends on this package existing in a ROS 2 workspace |

All six are ROS 2 packages (`package.xml` + `ament_cmake` `CMakeLists.txt`) built by `colcon`. This is the highest-value and highest-risk cluster: it is the only cluster in the safety-critical localization path.

**Already-native counterpart** (not yet default): `src/localization/slam/cpp/` contains `cyclone_runtime.cpp` (pure CycloneDDS, no ROS 2 dependency) alongside `dds_runtime.cpp` (a ROS2-interoperable DDS runtime used for incremental rollout). `src/localization/slam/cpp/CMakeLists.txt` already gates these behind independent build flags: `LINGTU_SLAM_FASTLIO2_BACKEND`, `LINGTU_SLAM_BUILD_CPP_DDS_RUNTIME` (ROS2-adjacent), `LINGTU_SLAM_BUILD_CYCLONE_DDS_RUNTIME` (ROS-free). `scripts/build/build_slam_core.sh` already drives this. `ISlamBackend` (documented in `docs/plans/PRD-lingtu-native-slam-navigation-runtime.md`) is the intended native contract Fast-LIO2/Point-LIO would implement instead of `rclcpp::Node`.

### 2.2 Base Autonomy / Local Planning — `src/nav/local/legacy_ros/`

`src/base_autonomy/` (as named in `CLAUDE.md`) no longer exists as a path; it was relocated and is now **already substantially decoupled**:

| Package | Status |
| --- | --- |
| `src/nav/local/legacy_ros/terrain_analysis/` | `COLCON_IGNORE` present — excluded from default colcon build |
| `src/nav/local/legacy_ros/terrain_analysis_ext/` | `COLCON_IGNORE` present |
| `src/nav/local/legacy_ros/local_planner/` | `COLCON_IGNORE` present |
| `src/nav/local/legacy_ros/sensor_scan_generation/` | `COLCON_IGNORE` present |

All four still contain their original `rclcpp` source (`terrainAnalysis.cpp`, `terrainAnalysisExt.cpp`, `localPlanner.cpp`, `pathFollower.cpp`, `sensorScanGeneration.cpp`), `package.xml`, `CMakeLists.txt`, and `launch/*.launch` files, but are inert by default. Production local planning today runs through the nanobind-exposed `src/nav/kernel/` (header-only C++ algorithm library, already has a standalone non-ROS2 CMake path and a lightweight `ament_cmake` wrapper in `src/nav/kernel/package.xml` purely so it can *also* build inside a ROS 2 workspace if needed) and `src/nav/services/plan/local_planner/cpp/` (confirmed standalone CMake with its own test target, no ROS2 dependency). This cluster needs formalization/cleanup (§7 Phase 1), not new engineering.

### 2.3 Sensor Ingestion — LiDAR

- `livox_ros_driver2` — vendored third-party ROS 2 package (`rclcpp` driver node, publishes `/livox/lidar`, `/livox/imu`). This is the production LiDAR ingestion path today.
- Native alternative already in progress: `scripts/build/build_livox_sdk2_stream.sh` drives a Livox-SDK2 (vendor-neutral C/C++ callback API, no ROS dependency) integration that publishes into the typed DDS layer directly (`src/message/idl/lingtu_slam.idl` defines `ros_compatible=False` native types such as `LivoxFrame`, `Imu`, `PointCloud2` in `src/message/dds.py`). Not yet the default production path.

### 2.4 Sensor Ingestion — Camera

- `src/drivers/real/camera/OrbbecSDK_ROS2/` — a large vendored ROS 2 package (`orbbec_camera`, plus msg/srv support), including `ob_camera_node.cpp`, `ob_camera_node_driver.cpp`, `ros_service.cpp`. This is the production camera ingestion path today, launched via ROS 2.
- Native alternative already in progress: `src/drivers/real/camera/native/` (`camera.cpp`/`camera.hpp`, `capture_process.cpp`) plus a Python `native_camera_module.py`, built by `scripts/build/build_orbbec_native.sh` directly against the Orbbec SDK (no ROS2). Not yet the default production path — mirrors the Livox situation exactly.

### 2.5 Sensor Ingestion — GNSS

- `wtrtk980_ros2_reader.cpp` — small `rclcpp` node for the GNSS/RTK receiver.
- A native serial-port GNSS driver already exists and is the default in most profiles; the ROS2 reader is the optional/legacy path. Verification of "already native by default" status is the only remaining work item (§7 Phase 4) — this is the smallest cluster in scope.

### 2.6 Python Bridge / Compatibility-Adapter Modules

These are intentionally isolated per the architecture rule "no ROS2 in Modules; rclpy only in Bridge modules":

| File | Role |
| --- | --- |
| `src/localization/adapters/ros2/slam_bridge.py` | `rclpy` subscriber for `/Odometry`, `/cloud_registered`, `/cloud_map`-equivalent topics; re-publishes into the Module framework's canonical channels (see §2.12) |
| `src/localization/bridge.py` | Thin shim that dynamically imports `slam_bridge.py`; keeps the ROS2 import out of the module's default import path |
| `src/nav/adapters/ros2/*` (`nav_in.py`, `nav_out.py`, `map_out.py` and siblings, per `docs/plans/PRD-slam-transport-navigation-dataflow.md`) | `rclpy` publish/subscribe adapters for nav-plane compatibility (`goal_pose`, `global_path`, `local_path`, `way_point`, `cmd_vel`) |
| `src/perception/adapters/ros2/bag_reader.py` | `rclpy`-based rosbag ingestion for offline perception testing |
| `src/runtime/adapters/ros2/map_save.py` | **Not an `rclpy` import** — shells out to the `ros2` CLI (`subprocess.run(["ros2","service","call","/pgo/save_maps", ...])`) after sourcing `/opt/ros/humble/setup.bash` inline. This is a distinct and easy-to-miss coupling mode: it will not show up in an `import rclpy` grep, only in a `ros2 ` / `/opt/ros/` string search. |
| Camera bridge / `rerun_bridge.py` | Optional ROS2-sourced visualization/camera compatibility paths |

#### 2.6a Current Implementation State of the Endpoint-Transport / Evidence Layer

§0 and §3 describe the Python transport abstraction as settled prior art. That is true of the *architecture* (ports/wires/pluggable Transport); it is not yet true of *this specific layer's implementation*. A full run of `python -m pytest src/runtime/tests/ -q` (2768 tests) on the current working tree reports **113 failed, 2636 passed, 19 skipped**, and a targeted re-run of the ten most-implicated modules narrows to 56 failed / 444 passed / 6 skipped in isolation:

```text
pytest src/runtime/tests/test_runtime_evidence.py src/runtime/tests/test_transport.py \
  src/runtime/tests/test_transport_wiring.py src/runtime/tests/test_lcm_endpoint_contracts.py \
  src/runtime/tests/test_cli_no_repl.py src/runtime/tests/test_runtime_display.py \
  src/runtime/tests/test_runtime_catalogs.py src/runtime/tests/test_deployment_service_contracts.py \
  src/runtime/tests/test_run_state.py src/runtime/tests/test_module_boundaries.py -q
```

Root-cause characterization: this is predominantly **(b) genuine implementation/test drift from the in-flight `src/core/` → `src/runtime/` rewrite**, not environment noise, with a smaller amount of vendored-file noise and one unrelated enumeration gap mixed in. Specifically:

- **`dds` vs `lcm` for `thunder_field` is a real, current contradiction, not test ambiguity.** `RUNTIME_ENDPOINTS["thunder_field"]` in `src/runtime/profiles/catalog/endpoints.py` sets `endpoint_transport="dds"`, `localization_adapter="dds_endpoint"`, `nav_in_adapter="dds_nav_input"`, `nav_out_adapter="dds_nav_output"`, `hardware_control_boundary="dds_endpoint_source"` — every adapter slot for the real robot endpoint currently resolves into `src/runtime/adapters/dds/`. But `test_lcm_endpoint_contracts.py::test_thunder_field_endpoint_references_lcm_contract` still asserts `endpoint.endpoint_transport == "lcm"`, and `LINGTU_RUNTIME_BUS_DECISION.md` itself states, under "Transport Direction / Thunder Endpoint/Nav": "Uses LCM endpoint adapters for cross-process endpoint boundaries." So the accepted decision doc, the contract test, and the running code disagree with each other right now. This is not a missing-dependency problem: the `lcm` Python package is in fact not installed on this Windows dev box, but that only changes which *other* LCM-adapter tests get skipped elsewhere in the suite — this specific assertion fails on a plain dataclass-field comparison with no `lcm` import involved, so the mismatch is real, not environmental.
- **`src/runtime/adapters/dds/` and `src/runtime/adapters/lcm/` look like two parallel, overlapping implementations, not a deliberate split by boundary type.** Both packages independently define their own `endpoint_runner.py`, `endpoint_service.py`, `contracts.py`, and `localization_adapter.py` for the same `thunder_field` boundary — a deliberate DDS-for-X/LCM-for-Y split would give each package disjoint responsibilities, not duplicate ones. (`lcm/` additionally has `nav_input.py`, `nav_output.py`, `endpoint_codec.py`, `source.py`; `dds/` has `map_output.py`, `nav.py` in roughly the same slots.) Both directories — and `endpoints.py` itself — are wholly untracked in git (`git status -s` reports `??` for all three), so there is no commit history in this tree to establish which is "the old one being replaced" versus "the new one replacing it." Flagged as open question §12.8.
- **Two `test_module_boundaries.py` violations are real, not vendored false positives, and land squarely on this plan's own "no ROS2 in Modules" boundary rule (§2.6):** `src/drivers/real/lidar/native_factory.py` imports `runtime.native_install` and `runtime.native_module` at module scope, failing both `test_livox_driver_native_factory_is_lazy_compatibility_shim` and `test_native_module_helpers_are_limited_to_ros2_compat_adapters` (whose entire purpose is keeping this file a *lazy* compatibility shim); and `src/runtime/adapters/dds/endpoint_service.py` plus `src/runtime/adapters/dds/localization_adapter.py` both import `drivers.real.lidar` (the latter reaching into a private `drivers.real.lidar._dds` submodule), failing `test_package_does_not_import_forbidden_layers_directly[runtime-forbidden5]` — a `runtime/` package reaching up into `drivers/`, the exact layering inversion the boundary suite exists to catch. A related third finding: `test_architecture_layer_manifest_is_valid_and_drives_boundary_rules` fails because `config/architecture_layers.yaml` has no layer-ownership entry for the new `src/message/` package (`src/message/__init__.py`, `src/message/dds.py`) — the same typed-DDS/IDL message layer §2.1/§3 cite as existing prior art is currently invisible to the boundary-enforcement manifest. (The remaining `test_module_boundaries.py` failures are pure vendored noise from one unparseable `OrbbecSDK_ROS2/orbbec_camera/launch/gemini2.launch.py` file plus a few un-catalogued vendored `rclpy`-importing benchmark scripts in the same third-party tree — not LingTu-authored, not evidence of drift.)
- **A topic/channel-naming migration is the single biggest contributor to the `test_cli_no_repl.py`, `test_runtime_display.py`, `test_run_state.py`, part of `test_runtime_evidence.py`, and `test_deployment_service_contracts.py` failures.** For example, `test_runtime_contract_prints_canonical_manifest` expects `runtime_data_flow_topics["thunder_field"]` to start with `/nav/lidar_scan`, `/nav/imu`, `/nav/odometry`, ...; the current `scripts/lingtu` ops CLI instead already uses `/lidar/raw_frame`, `/imu/raw`, `/slam/odometry`, `/slam/localization_health`, `/slam/localization_quality`, and no longer contains the literal string `/nav/localization_quality` the deployment-contract tests look for. Multiple spellings of the same Tier A/Tier B naming distinction (§2.12) exist simultaneously across the runtime contract, the CLI display formatters, and the ops shell script, and their respective tests are not all pinned to the same one yet — this is that same rename caught mid-flight, not a new naming decision this plan needs to make.
- **One failure is unrelated enumeration drift, not a transport or naming issue**: `test_runtime_catalogs.py::test_product_runtime_configs_use_ros_free_autonomy_backends` fails not because any backend is actually ROS-coupled, but because its hardcoded expected profile set (`lite`, `map`, `nav`, `explore`, `tare_explore`, `super_lio`, `super_lio_relocation`) predates four profiles now present in `PRODUCT_PROFILE_ENDPOINTS` (`teleop`, `teleop_avoid`, `tracking`, `inspection`). Noted so this specific failure is not mistaken for a ROS-backend regression.

**Net characterization**: predominantly (b) genuine implementation/test drift from the in-flight rewrite, concentrated in the endpoint-transport, evidence-contract, and CLI-display layers — plus a smaller slice of (a) environment-specific noise (missing `lcm` pip package on this Windows box, affecting skip counts elsewhere but not the failures analyzed above) and vendored third-party parse noise. Nothing found here argues against the target architecture (§4), the technical approach (§5), or the phase order (§6/§7) — it argues that Phase 0 needs an explicit stabilization item first (§7).

### 2.7 `tf2` / Frame Transform Publishing

Confirmed usage: `src/localization/pgo/src/pgo_node.cpp` owns a `tf2_ros::TransformBroadcaster` publishing `map`→`odom`. Per `docs/architecture/ros_frame_contract.md`, the required TF shape is a fixed, shallow 4-frame chain: `map -> odom -> body -> lidar`. No other component in the current tree independently republishes or listens to this via `tf2_ros::TransformListener`/`Buffer` outside the ROS2 process graph — Python consumers get `map`↔`odom` correction indirectly today, through `SlamBridgeModule`'s odometry/health republishing, not through a tf2 buffer of their own. `src/runtime/tf/` already exists as a ROS-free transform library and explicitly documents itself as *not* a ROS2 drop-in (i.e., it is not attempting to be a general n-ary tf2 clone).

### 2.8 Build System

- Top-level `Makefile`: `make build` (colcon release build, requires sourcing Humble), `make test` — both ROS2/colcon-first today per `CLAUDE.md`/`AGENTS.md`. `make nav_kernel` is already a standalone, non-ROS2 target.
- `scripts/build/build_ros_workspace.sh` — builds the full ROS2 package set via colcon.
- Already-standalone precedents to extend: `scripts/build/build_slam_core.sh` (native SLAM contract, optional CycloneDDS runtime), `scripts/build/build_orbbec_native.sh`, `scripts/build/build_livox_sdk2_stream.sh`, `nav_kernel`'s own CMake, `src/nav/services/plan/local_planner/cpp`'s own CMake.
- Every ROS2 package (§2.1–2.5, §2.2) carries its own `package.xml` + `ament_cmake` `CMakeLists.txt` — roughly a dozen independent ROS2 build units in `src/` today.

### 2.9 Launch Files

Top-level `launch/` plus per-package `launch/` directories: `src/nav/local/legacy_ros/{terrain_analysis,terrain_analysis_ext,local_planner,sensor_scan_generation}/launch/*.launch` (inert, `COLCON_IGNORE`'d), plus `fastlio2`/`localizer`/`pgo` launch entry points invoked from `src/localization/launch/localizer_launch.py` and equivalents, plus Livox/Orbbec/GNSS vendor launch files. Roughly 10+ launch files total, none of which are consumed by any Python Module directly — they are only used by systemd units and manual `ros2 launch` invocations (§2.10).

### 2.10 S100P Deployment

Per `docs/04-deployment/README.md` and the (partially historical) `docs/04-deployment/S100P_STACK_INVENTORY.md`, production systemd services (`robot-lidar`, `robot-camera`, `robot-fastlio2`, `robot-localizer`) each source `/opt/ros/humble/setup.bash` and either `ros2 launch` or `ros2 run` the corresponding node. `scripts/lingtu` (the unified ops CLI) has `svc status|restart` sub-commands that key off these systemd unit names, and `log drift|dufomap|error|tail` filters that assume these unit names in `journalctl`. A native-path systemd unit already exists in prototype form: `scripts/deploy/thunder/lingtu-slam-dds.service` runs the pure-CycloneDDS SLAM runtime, not yet wired into `scripts/lingtu`'s default status/restart targets.

### 2.11 CI

- `.github/workflows/slam-aarch64-build.yml` — active CI, runs `colcon build` for the ROS2 SLAM packages on ARM64, sourcing ROS Humble.
- `.github/workflows/nav-core-tests.yml` — active CI for the standalone (non-ROS2) `nav_kernel` build; a working template for what native-SLAM CI should look like.
- `.github/workflows/portable-fastlio2-windows-gate.yml` — a Windows, non-ROS, ctypes-bound Fast-LIO2 CI gate. On inspection this workflow references file paths that no longer exist in the current tree; it appears stale relative to the `src/localization/slam/cpp/` native-runtime direction and should be repaired or retired as part of this plan rather than left silently broken.

### 2.12 Topic and Service Name Inventory (verified against current tree)

The task brief's assumed topic/service list (`/livox/lidar`, `/Odometry`, `/cloud_registered`, `/way_point`, `/cmd_vel`, `/relocalize`, `/pgo/save_maps`, etc.) is **still accurate, but only as the internal ROS2-node-to-ROS2-node wire names** — it is not what Python Module code sees today. Two distinct tiers exist:

**Tier A — internal ROS 2 process-graph names** (hardcoded in vendored/legacy C++ node source, only visible to other ROS2 nodes and to the explicit bridge adapters):

| Topic / Service | Type | Producer | Consumer |
| --- | --- | --- | --- |
| `/livox/lidar`, `/livox/imu` | `CustomMsg`, `Imu` | `livox_ros_driver2` | `fastlio2` |
| `/cloud_registered`, `/cloud_map`, `/Odometry` | `PointCloud2`, `Odometry` | `fastlio2`/`pointlio` | `pgo`, `localizer`, `SlamBridgeModule` |
| `/pgo/loop_markers` | `MarkerArray` | `pgo` | RViz/debug only |
| `/way_point`, `/planner_waypoint`, `/pct_path`, `/path`, `/cmd_vel`, `/slow_down`, `/stop` | mixed | legacy `local_planner`/`pathFollower` (now `COLCON_IGNORE`'d) | legacy chain only |
| `save_map` (srv) | `SaveMaps.srv` | `fastlio2`/`pointlio` | `map_save.py` adapter (via `ros2 service call`) |
| `/pgo/save_maps`, `/pgo/save_poses` (srv) | `SaveMaps.srv`, `SavePoses.srv` | `pgo` | `map_save.py` adapter |
| `relocalize`, `relocalize_check`, `global_relocalize`, `global_relocalize_status` (srv) | `Relocalize.srv`, `IsValid.srv`, `Trigger` | `localizer` | ops tooling / relocalization workflow |
| `refine_map`, `save_poses` (srv) | `RefineMap.srv`, `SavePoses.srv` | `hba` | offline map-refinement tooling |

**Tier B — LingTu canonical channel names** (transport-agnostic; what Python Modules actually consume, per `ros_frame_contract.md` / `LINGTU_RUNTIME_BUS_DECISION.md`): `/lidar/raw_frame`, `/imu/raw`, `/slam/odometry`, `/slam/registered_cloud`, `/slam/map_cloud`, `/slam/state_at_scan`, `/nav/terrain_map`, `/nav/global_path`, `/nav/local_path`, `/nav/way_point`, `/nav/goal_pose`, `/nav/cmd_vel`. `SlamBridgeModule` and the `nav/adapters/ros2/*` modules are exactly the translation boundary between Tier A and Tier B today. This means the migration's job is narrower than "rewire every Python consumer" — Python consumers already only know Tier B names; the work is replacing what feeds Tier B underneath the bridge.

### 2.13 Out of Scope

`src/legacy/` (retired pre-v1.7 code), research archives (far-planner/VLA-nav style directories), and one-off calibration ROS2 tooling are not part of the product's runtime path and are excluded from this plan's phases; they should be swept up only in the final cleanup pass (§7 Phase 8) if still present.

### 2.14 Inventory Summary

| Category | Approx. file/unit count | Notes |
| --- | --- | --- |
| LingTu-authored ROS2 C++ node packages (§2.1, §2.2) | ~10 build units, ~70 files | 4 of 10 already `COLCON_IGNORE`'d |
| Vendored third-party ROS2 driver packages (§2.3, §2.4) | 2 packages, ~100-170 files | Wholesale-replace candidates, not line-by-line ports |
| Python `rclpy` / `ros2`-CLI adapters (§2.6) | ~10 files | Already isolated behind `adapters/ros2/` boundaries |
| Build / deploy / CI (§2.8-§2.11) | ~10 files | Makefile, build scripts, systemd, workflows |
| **Total distinct ROS2 touchpoints** | **~90 LingTu-owned files across ~10 build targets, plus 2 vendored packages (~100-170 more files)** | See §2.14 for the reasoning behind not collapsing this into one false-precision number |

## 3. Prior Art This Plan Builds On (Do Not Re-decide)

To avoid re-litigating settled decisions, this plan explicitly defers to:

- **Transport policy** — `LINGTU_RUNTIME_BUS_DECISION.md` already decided: LCM for cross-process/network endpoints, typed DDS for selected high-performance boundaries, SHM for same-host high-bandwidth data, Local callback in-process. This plan does not propose a new IPC technology; it proposes finishing the C++ node cutover onto this existing policy.
- **Role mapping** — `ROS_ROLE_REPLACEMENT_MAP.md` already states which ROS roles are adapters vs. native LingTu, and explicitly removed `portable-lio`/`windows-fastlio2` as a product endpoint (i.e., Windows is a CI/dev-loop target, not a deployment target — see §12 open question on target OS scope).
- **Frame contract** — `ros_frame_contract.md` already fixes the frame tree shape (`map -> odom -> body -> lidar`) that any native transform mechanism must satisfy.
- **Native SLAM contract** — `docs/plans/PRD-lingtu-native-slam-navigation-runtime.md` already specifies `ISlamBackend` as the interface Fast-LIO2/Point-LIO should implement in place of `rclcpp::Node`, and mandates the product's main path be ROS-free while keeping ROS2 as an explicit compatibility path.
- **Transport dataflow contract** — `docs/plans/PRD-slam-transport-navigation-dataflow.md` already specifies `src/runtime/msgs` (Python) and `src/message/idl/lingtu_slam.idl` (typed DDS/IDL) as the schema layer, and `src/localization/slam/cpp/cyclone_runtime.cpp` plus `scripts/deploy/thunder/lingtu-slam-dds.service` as the native runtime + deployment unit.
- **Packaging tiers** — `PORTABLE_LEAN_PACKAGE_MATRIX.md` already places `rclpy`, `tf2_ros`, and ROS message packages in a "ROS compatibility" dependency tier, distinct from the core-portable tier — this is the existing mechanism this plan uses to make ROS2 an opt-in install (§8).
- **Executed precedent** — `src/nav/local/legacy_ros/*` with `COLCON_IGNORE` markers is proof the "quarantine legacy ROS2 node, promote nanobind-native replacement to default" pattern already works end-to-end for one full subsystem.

This document's contribution is: (a) the concrete file-level inventory above, (b) the sequencing of the *remaining* cutover work across subsystems, and (c) the validation gates and rollback mechanics needed to do that safely on a physical, safety-critical robot.

## 4. Target Architecture

```text
TODAY — per-sensor / per-node ROS 2 islands, bridged into the framework at the edge

Livox MID-360 --> livox_ros_driver2 (rclcpp) --> /livox/lidar, /livox/imu
                                                        |
                                                        v
                                              fastlio2 (rclcpp node)
                                                        |
                                     +------------------+------------------+
                                     v                                     v
                              pgo (rclcpp, tf2 map->odom)         [legacy_ros terrain_analysis/local_planner —
                                     |                              already COLCON_IGNORE'd, inert]
                                     v
                          localizer (rclcpp, ICP relocalize)
                                     |
                                     v  (rclpy)
                     SlamBridgeModule (src/localization/adapters/ros2/slam_bridge.py)
                                     |
                                     v  (In/Out ports, canonical Tier-B channel names)
                    NavigationModule, OccupancyGrid, ESDF, ElevationMap, Gateway, ...

Orbbec camera --> OrbbecSDK_ROS2 (rclcpp) --> rclpy adapter --> CameraBridge
GNSS receiver --> wtrtk980_ros2_reader (rclcpp, legacy path) [native serial driver already default in most profiles]

Build: colcon + ament_cmake, requires `source /opt/ros/humble/setup.bash` (Ubuntu 22.04 pinned)


TARGET — native sensor ingestion + native SLAM runtime, ROS 2 fully optional

Livox MID-360 --> Livox-SDK2 (native C callback API) --> LingTu lidar-ingest process
                                                                 |
                                                    typed DDS (LivoxFrame/PointCloud2,
                                                    ros_compatible=False, src/message/idl/lingtu_slam.idl)
                                                                 v
                                          fastlio2 (ISlamBackend, cyclone_runtime.cpp, no rclcpp)
                                                                 |
                                                native map->odom->body transform
                                                (src/runtime/tf/, fixed 4-frame chain, no tf2)
                                                                 |
                                   +----------------------------+----------------------------+
                                   v                                                          v
                          pgo / localizer (native, CycloneDDS)                nav_kernel / local_planner (nanobind,
                                   |                                            already native — see legacy_ros precedent)
                                   v  (typed DDS / LCM per LINGTU_RUNTIME_BUS_DECISION.md)
                    NavigationModule, OccupancyGrid, ESDF, ElevationMap, Gateway, ...

Orbbec camera --> native capture_process (src/drivers/real/camera/native/) --> typed DDS/local port
GNSS receiver --> native serial driver (default)

Build: plain CMake per component (nav_kernel / local_planner-cpp precedent), colcon/ament_cmake/Humble
       become an explicit opt-in "ros2-compat" extra, selected the same way `slam_profile="bridge"`
       is selected today — not deleted, not the default.
```

The target is not "no ROS 2 anywhere, ever" — it is "ROS 2 is an explicit, optional compatibility profile, never a hard dependency of the default build or the default S100P deployment."

## 5. Technical Options Assessed

### 5.a Pure CycloneDDS C++ nodes replacing `rclcpp` nodes directly

**Assessment**: Sound, and already prototyped (`cyclone_runtime.cpp`). ROS 2's default RMW under `rmw_cyclonedds` is itself CycloneDDS, so a raw DDS participant *can* speak wire-compatible RTPS to a still-ROS2 node during incremental rollout — but only if three caveats are handled explicitly, not assumed:

1. **Topic name mangling**: `rmw_cyclonedds` prefixes ROS2 topics with `rt/` (and services with `rq/`/`rr/` request/reply suffixes) at the DDS wire level. A raw DDS participant must publish/subscribe on the mangled name (e.g. `rt/Odometry`), not the ROS-level name (`/Odometry`), to interoperate.
2. **Type-support mangling**: ROS2 message types are wrapped in `rmw`-specific type descriptors (dds-generated struct layout with a magic prefix and hash-suffixed type name). A hand-written CycloneDDS IDL type will not automatically match an `Odometry` ROS message on the wire unless the IDL is generated from (or byte-compatible with) the same `.msg`-derived layout. This is a real interoperability trap, not just a naming inconvenience — it should not be relied on beyond a short, explicitly-tested incremental-rollout window.
3. Because of (1) and (2), **native LingTu topics should use LingTu's own IDL** (`src/message/idl/lingtu_slam.idl`, already `ros_compatible=False` for the sensor/SLAM hot path) rather than attempting byte-compatible ROS2 message mimicry. Wire interop with a still-ROS2 node, where genuinely needed during a transition window, should go through the explicit bridge adapter (§2.6), not through DDS-level type-mangling tricks.

**Recommendation**: proceed with pure CycloneDDS C++ nodes using LingTu's own IDL types, per the already-written `cyclone_runtime.cpp` direction. Do not build a general ROS2-wire-compatible DDS shim; it adds fragile complexity for a benefit (talking to a live ROS2 node without the bridge) that the existing Python bridge adapter already provides more robustly.

### 5.b Livox-SDK2 direct integration

**Assessment**: Strong candidate, already in progress (`build_livox_sdk2_stream.sh`, native DDS publishing). Livox-SDK2 is vendor-maintained, has no ROS dependency, and removes the single largest vendored-ROS2-package cluster from the sensor-ingestion boundary. This is the best "quick win with a real non-ROS SDK" in the whole inventory.

**Recommendation**: proceed; sequence early (§7 Phase 2) precisely because the SDK alternative already exists and is not on the safety-critical control path directly (it feeds SLAM, but is not itself doing localization math).

### 5.c Orbbec native SDK

**Assessment**: Same shape as Livox — a native capture path already exists (`src/drivers/real/camera/native/`), driven by the vendor SDK directly, no ROS2 dependency, not yet the production default. Camera data feeds perception/semantic modules, not the safety-critical control loop, making this a similarly low-blast-radius cutover.

**Recommendation**: proceed; sequence immediately after Livox (§7 Phase 3) using the same pattern.

### 5.d Alternative IPC comparison (for completeness / audit trail — not a new decision)

| Option | Verdict |
| --- | --- |
| CycloneDDS (typed, native IDL) | **Adopt** for the SLAM/sensor hot path and other selected high-performance boundaries — already a production dependency, already has a C++ prototype runtime. A Python transport backend for this exists and is wired up, but is currently in-flight rather than fully settled (§2.6a: ~40 failing tests in exactly this layer) — treat as "adopt, then stabilize," not "adopt, already done." |
| LCM | **Adopt** for cross-process/network endpoints per the existing `LINGTU_RUNTIME_BUS_DECISION.md` — do not replace this decision, extend the C++ boundary to honor it. **Current-state caveat**: `RUNTIME_ENDPOINTS["thunder_field"]` is wired to `endpoint_transport="dds"` today, not `"lcm"`, directly contradicting both this decision doc's own "Thunder Endpoint/Nav... Uses LCM endpoint adapters" rule and `test_lcm_endpoint_contracts.py`, which still asserts `"lcm"` and is currently failing (§2.6a). This needs an explicit resolution, not a silent default — see open question §12.8. |
| SHM | **Adopt** for same-host bulk data (e.g. map clouds) per the existing decision — no change needed. |
| Zenoh | Not recommended. No existing production dependency, no prototype code, and it would add a fourth transport technology on top of three (LCM/DDS/SHM) already decided and partially implemented. Revisit only if CycloneDDS proves insufficient for a concrete, measured bottleneck (none identified in this investigation). |
| ZeroMQ | Not recommended for the same reason as Zenoh — no existing foothold, and it solves a problem (lightweight messaging) that LCM already solves in this codebase. |
| Plain UNIX sockets / custom framing | Not recommended — reinvents QoS, discovery, and serialization that DDS/LCM already provide; would increase, not decrease, total system complexity. |

**Conclusion**: no new IPC technology is needed. The existing three-transport policy is sufficient; the gap is adoption at the C++ node boundary, not a gap in the policy itself.

### 5.e `map -> odom -> body` transform without `tf2`

**Assessment**: LingTu's frame tree is fixed, shallow, and fully known ahead of time (`map -> odom -> body -> lidar`, per `ros_frame_contract.md`) — it is not the arbitrary, dynamically-discovered n-ary tree that `tf2`'s general `Buffer`/`TransformListener` API is designed for. Building a general tf2 clone would be solving a harder problem than LingTu actually has.

**Recommendation**: extend `src/runtime/tf/` (already explicitly scoped as "not a ROS2 drop-in") with a minimal, fixed-frame native channel: the `map -> odom` correction that `pgo_node.cpp`'s `TransformBroadcaster` publishes today should become a field on the same typed SLAM state/odometry message already flowing through the native channel (Tier B, §2.12), rather than a separate generic "TF" concept. `odom -> body` already flows naturally as the SLAM odometry message itself. This avoids inventing new infrastructure for a 3-hop, statically-known chain. Before finalizing this API, every current *consumer* of the ROS2 TF tree (not just the one known publisher) must be enumerated — this is flagged as an explicit open question (§12.1) rather than assumed complete here.

## 6. Migration Order and Justification

Ranking by (a) existence of a mature non-ROS SDK/alternative, (b) blast radius / safety exposure if the cutover goes wrong, and (c) how much of the work is already done:

1. **Base autonomy** (§2.2) — already ~90% done (quarantined via `COLCON_IGNORE`, native nanobind path is already the production default). Lowest risk, mostly a formalization/cleanup task. Do first to "bank the win" and prove the phase-gate process on a subsystem that's already de-risked.
2. **LiDAR ingestion** (§2.3) — mature vendor-neutral SDK exists (Livox-SDK2), native prototype exists, feeds SLAM but is not itself safety-authority code. Good second step: contained blast radius, clears the way for SLAM's own input to already be native before SLAM itself cuts over.
3. **Camera ingestion** (§2.4) — same shape as LiDAR, one step behind it in native-path maturity. Not on the control-authority path (feeds perception/semantic, not safety/control).
4. **GNSS** (§2.5) — smallest cluster, likely already resolved; a verification task more than a build task.
5. **SLAM core, Stage A: Fast-LIO2** (§2.1) — highest complexity, directly in the localization path that safety and control depend on. Deliberately sequenced after ingestion is already native and stable, so that when Fast-LIO2 itself is being validated, its inputs are not simultaneously changing underneath it.
6. **SLAM core, Stage B: PGO + Localizer** (§2.1) — loop closure and relocalization build on Fast-LIO2's odometry; sequenced after Stage A is field-validated, not in parallel with it, because a regression here is much harder to attribute if both stages moved at once.
7. **Production promotion** (§2.10) — flip S100P systemd defaults only after 5 and 6 are individually field-validated.
8. **Build/CI/deployment consolidation** (§2.8, §2.9, §2.11) — last, because it is destructive/cleanup work (removing or fully demoting the colcon path) that should only happen once nothing in production still needs it as the default.

This order matches the task brief's own instinct: sensor-ingestion boundaries with strong non-ROS SDKs go first; SLAM, the most complex and safety-critical piece, goes last among the runtime subsystems.

## 7. Phased Rollout Plan

Each phase lists scope, entry criteria, work items, the validation gate that must pass before moving on, and its rollback.

### Phase 0 — Guardrails and Inventory Lock

- **Scope**: no runtime behavior change. Establish the scaffolding that makes every later phase auditable, and stabilize the existing Python endpoint-transport/evidence contract layer (§2.6a) before it carries any new native C++ output path.
- **Work items**: add a CI lint/grep check for ROS2 coupling that catches *both* `import rclpy`/`#include <rclcpp` *and* the `ros2 ` CLI-subprocess / `/opt/ros/` string patterns (§2.6 showed the latter is easy to miss); snapshot this document's §2.14 counts as a CI-checked baseline so future PRs can't silently add new ROS2 touchpoints without the count being visibly reviewed; inventory the toolchain requirements of every already-native binary (DUFOMap, nav_kernel, local_planner-cpp) so Phase 8's "drop Ubuntu 22.04 pin" claim doesn't quietly break on an unrelated toolchain assumption; **stabilize the endpoint-transport/evidence layer identified in §2.6a — currently ~40 failing tests — specifically: (1) resolve the `dds`-vs-`lcm` contradiction for `thunder_field` as an explicit, documented decision (§12.8), then bring `test_lcm_endpoint_contracts.py` and `LINGTU_RUNTIME_BUS_DECISION.md` back into agreement with whichever transport is chosen; (2) fix the two genuine `test_module_boundaries.py` layering violations (`src/drivers/real/lidar/native_factory.py` importing `runtime.native_module`/`runtime.native_install`; `src/runtime/adapters/dds/endpoint_service.py` and `.../localization_adapter.py` importing `drivers.real.lidar`) so the boundary-enforcement suite this plan's own guardrail work extends is actually green, not just present; (3) register `src/message/` in `config/architecture_layers.yaml`'s layer-ownership manifest so the boundary checker can see the typed-DDS/IDL layer §2.1/§3 already cite as prior art.**
- **Validation gate**: CI check merges and passes on current `main`/working branch without needing to touch any SLAM/perception/planning code, **and the ten test modules named in §2.6a pass cleanly** (`test_runtime_evidence.py`, `test_transport.py`, `test_transport_wiring.py`, `test_lcm_endpoint_contracts.py`, `test_cli_no_repl.py`, `test_runtime_display.py`, `test_runtime_catalogs.py`, `test_deployment_service_contracts.py`, `test_run_state.py`, `test_module_boundaries.py`) — this plan does not require fixing every one of these itself, but Phase 2+ must not start building native output paths on an endpoint-transport contract layer that is still internally self-contradictory.
- **Rollback**: trivial — CI-only change. The test stabilization work is an independent bug-fix effort with no migration-specific rollback risk of its own.

### Phase 1 — Base-Autonomy Retirement Formalization

- **Scope**: `src/nav/local/legacy_ros/*`.
- **Entry criteria**: Phase 0 guardrail merged.
- **Work items**: confirm zero production profile still depends on the `COLCON_IGNORE`'d packages (grep all profile configs in `cli/profiles_data.py`); either delete the legacy_ros ROS2 sources outright or move them to `docs/archive`-adjacent storage/a clearly marked `legacy/` tree, matching how other retired code already lives under `src/legacy/`; update `CLAUDE.md`/`AGENTS.md` base-autonomy references to point at `nav_kernel`/`local_planner/cpp` only.
- **Validation gate**: full framework test suite (`python -m pytest src/runtime/tests/ -q`) plus the C++ standalone `test_local_planner_core`/`test_benchmark`/`test_path_follower_core` targets pass with the legacy_ros tree removed from the build graph entirely (not just `COLCON_IGNORE`'d).
- **Rollback**: revert the deletion/move commit; `COLCON_IGNORE` markers mean no runtime behavior was ever depending on this, so rollback risk is effectively zero.

### Phase 2 — LiDAR Ingestion Cutover

- **Scope**: replace `livox_ros_driver2` with the native Livox-SDK2 path as the *default* LiDAR source.
- **Entry criteria**: Phase 1 complete; `build_livox_sdk2_stream.sh` builds cleanly on target hardware.
- **Work items**: promote the native lidar-ingest process to the default in the `lidar()` stack factory; wire its typed-DDS output (`LivoxFrame`/`PointCloud2`, `ros_compatible=False`) into the same consumers Fast-LIO2/SLAM currently receive from `livox_ros_driver2`; keep `livox_ros_driver2` selectable via an explicit backend flag (mirroring `slam_profile`) rather than deleted.
- **Validation gate**: side-by-side point-cloud comparison (native vs. ROS2 driver) on recorded and live S100P data — point rate, timestamp jitter, and point-cloud density/coverage within an agreed tolerance; Fast-LIO2 (still ROS2 at this stage) produces equivalent odometry drift whether fed by the native or ROS2 LiDAR path over a fixed test route.
- **Rollback**: flip the `lidar_backend` flag back to `ros2`; no SLAM/planning code changes required to roll back.

### Phase 3 — Camera Ingestion Cutover

- **Scope**: replace `OrbbecSDK_ROS2` with the native capture path as the default.
- **Entry criteria**: Phase 2 complete (proves the ingestion-cutover pattern once).
- **Work items**: promote `src/drivers/real/camera/native/` + `native_camera_module.py` to default in the `driver()`/camera-bridge stack factory; keep the ROS2 Orbbec path selectable via an explicit backend flag.
- **Validation gate**: perception pipeline (detector + encoder + scene graph) produces equivalent scene-graph output on a fixed recorded scenario whether fed by the native or ROS2 camera path; frame-rate and latency within tolerance.
- **Rollback**: flip the camera backend flag back to `ros2`.

### Phase 4 — GNSS Ingestion Confirmation

- **Scope**: `wtrtk980_ros2_reader.cpp` vs. the native serial driver.
- **Entry criteria**: none (independent of Phases 1-3).
- **Work items**: confirm which profiles, if any, still default to the ROS2 GNSS reader; if any do, cut them to the native driver; if none do, this phase is a documentation-only confirmation.
- **Validation gate**: no production profile's default configuration references the ROS2 GNSS node.
- **Rollback**: N/A (verification task).

### Phase 5 — SLAM Core Native Cutover, Stage A: Fast-LIO2

- **Scope**: `src/localization/fastlio2/` → `ISlamBackend` implementation over `cyclone_runtime.cpp`.
- **Entry criteria**: Phases 2-4 complete (native, stable sensor inputs feeding the new SLAM path); `ISlamBackend` contract and `cyclone_runtime.cpp` reach feature parity with the existing `lio_node.cpp` publish surface (odometry, registered cloud, map cloud, `save_map`-equivalent).
- **Work items**: implement/complete the native Fast-LIO2 backend against `ISlamBackend`; register it as a new `slam_profile` entry (e.g. `fastlio2_native`) alongside the existing `fastlio2`/`bridge`/`localizer`/`none` entries — additive, not a replacement in place; wire its typed-DDS odometry/cloud output directly into the Tier-B canonical channels Python Modules already consume, bypassing `SlamBridgeModule`'s `rclpy` path when this profile is active.
- **Validation gate**: **dual-run shadow validation** — run native and ROS2 Fast-LIO2 side-by-side consuming the same (now-native) LiDAR/IMU feed across multiple recorded and live S100P sessions; compare odometry drift, IMU-bias convergence, and map-cloud density/consistency against the ROS2 baseline within an agreed numeric tolerance (owner: whoever owns localization accuracy evaluation — flagged as open question §12.4, since no existing eval harness for this comparison was found in this investigation). Do not promote to any production default until this gate passes on live field data, not just recordings.
- **Rollback**: `slam_profile` flag back to `fastlio2` (ROS2); this is a config change only, matching how `--llm mock` already works today.

### Phase 6 — SLAM Core Native Cutover, Stage B: PGO + Localizer

- **Scope**: `src/localization/pgo/`, `src/localization/localizer/` → native equivalents; native `map -> odom` transform per §5.e.
- **Entry criteria**: Phase 5's `fastlio2_native` profile has passed its validation gate on live field data, not just recordings.
- **Work items**: implement native loop-closure (PGO) and ICP relocalization (Localizer) against `ISlamBackend`/CycloneDDS; implement the fixed-frame native `map -> odom` transform field per §5.e; register a new `slam_profile` entry (e.g. `localizer_native`) additive to the existing set; map save/load must read/write the *same* on-disk PCD/tomogram formats already used (`~/data/nova/maps/`) — no map-format migration, so maps built under either profile remain loadable under the other.
- **Validation gate**: relocalization success rate and loop-closure quality on the native path meet or exceed the ROS2 baseline across a defined set of field scenarios (e.g. return-to-start, kidnapped-robot recovery); map artifacts produced by the native path load correctly in the existing map-viewer/Gateway tooling.
- **Rollback**: `slam_profile` flag back to `localizer`/`bridge`.

### Phase 7 — Production Promotion on S100P

- **Scope**: flip the default systemd/profile selection on the robot itself.
- **Entry criteria**: Phases 5 and 6 have each independently passed their field-validation gates; a minimum dual-run field-hours threshold has been met (exact threshold: open question §12.3).
- **Work items**: add native systemd units (extending the existing `scripts/deploy/thunder/lingtu-slam-dds.service` prototype to cover LiDAR/camera ingestion too); update `scripts/lingtu`'s `svc status|restart` and `log` filters to know about both native and ROS2 unit names during the transition; flip the *default* profile on S100P from `bridge`/ROS2 to the native profile; keep the ROS2 systemd units installed but not auto-started, selectable via the same `svc` sub-commands as an explicit fallback.
- **Validation gate**: N consecutive full field days (patrol/mapping/nav sessions) on the native default with no fallback-to-ROS2 incident, and drift-watchdog / DUFOMap / map-save pipelines all confirmed working unchanged against the native SLAM output.
- **Rollback**: `scripts/lingtu svc restart` with the ROS2 profile flag — an ops-runbook action, not a code deployment.

### Phase 8 — Build/CI/Deployment Consolidation

- **Scope**: `Makefile`, `scripts/build/build_ros_workspace.sh`, CI workflows, packaging tiers.
- **Entry criteria**: Phase 7 has been stable in production for an agreed soak period.
- **Work items**: flip `make build`'s default meaning to the native/plain-CMake build; rename the colcon path to an explicit opt-in target (e.g. `make build-ros2-compat`); move `rclpy`/`tf2_ros`/ROS message packages fully into the existing "ROS compatibility" tier in `PORTABLE_LEAN_PACKAGE_MATRIX.md` as an optional install extra, not a default dependency; repair or retire the stale `portable-fastlio2-windows-gate.yml`; add a `slam-native-aarch64-build.yml` CI job mirroring `nav-core-tests.yml`'s standalone-CMake pattern for the native SLAM contract; keep `slam-aarch64-build.yml` (colcon) running, unmodified, as the compat-path CI — do not delete it in this phase, only stop treating it as the primary gate.
- **Validation gate**: a clean-machine build with no ROS2/Ubuntu-22.04-specific packages installed succeeds end-to-end for the default profile set (`stub`, `dev`, `sim`, `map`, `nav`); the ROS2 compat build still succeeds independently when explicitly requested.
- **Rollback**: N/A — this phase only changes defaults/labels on already-dual-tracked build paths; nothing is deleted here.

## 8. Required Build, Deploy, and CI Changes

Consolidated from the phase-by-phase items above:

- **Makefile**: introduce native-default targets now (`make slam-native`, extending existing `make nav_kernel`); do not flip `make build`'s default meaning until Phase 8.
- **`scripts/build/`**: `build_slam_core.sh`, `build_livox_sdk2_stream.sh`, `build_orbbec_native.sh` become the primary build entrypoints; consider a single `scripts/build/build_native_stack.sh` orchestrator once all three are individually stable, purely for operator convenience (not required for correctness).
- **systemd**: new native units alongside (not replacing) `robot-lidar`/`robot-camera`/`robot-fastlio2`/`robot-localizer`; extend the existing `lingtu-slam-dds.service` prototype's pattern to the other sensors.
- **`scripts/lingtu`**: `svc`/`log` sub-commands need to recognize both unit-name sets through the whole dual-run window (Phases 2-7), not just at the very end.
- **CI**: keep `slam-aarch64-build.yml` running unmodified throughout (it is the compat-path safety net); add native-path CI per phase as each native component stabilizes, rather than waiting for Phase 8; repair or retire `portable-fastlio2-windows-gate.yml` early (Phase 0/1 timeframe) since it is already broken and provides no signal today.
- **Packaging**: use the existing tiering in `PORTABLE_LEAN_PACKAGE_MATRIX.md` verbatim — ROS2 Python/build dependencies move to the already-defined "ROS compatibility" tier as an optional extra.

## 9. Rollback and Compatibility Strategy

The rollback mechanism for every phase in §7 is the same pattern already used in production today for `slam_profile` (`fastlio2` / `pointlio` / `localizer` / `bridge` / `none`) and for CLI overrides like `--llm mock`: **add new backends as additive registry entries, gate promotion to "default" behind an explicit field-validation gate, and make rollback a config flag flip, not a code change.**

Concretely:

- Every native component introduced by this plan (LiDAR, camera, Fast-LIO2, PGO, Localizer) is registered as a *new*, additively-named backend, never an in-place rewrite of the existing ROS2-backed one. The ROS2 path is not deleted until Phase 8, and even then only its "default" status changes, not its availability.
- **Map/artifact compatibility is a hard requirement, not an aspiration**: native SLAM must read and write the same PCD/tomogram formats already on disk at `~/data/nova/maps/`. If this constraint cannot be met for some map artifact type, that is a blocking finding for Phase 6, not a detail to patch around later.
- **Field rollback trigger**: define an explicit, ops-runbook-level threshold (e.g. localization health/covariance beyond X, or relocalization failure rate beyond Y over Z attempts — exact numbers are an open question, §12) that tells a field operator to flip the `slam_profile`/backend flag back to the ROS2 path immediately, the same way the existing SLAM drift watchdog already auto-restarts ROS2 SLAM services today on divergence.
- **Dual-run validation window**: for Phases 5-7 specifically, run native and ROS2 side-by-side (same sensor feed, via DDS fan-out) for a minimum number of field sessions before flipping any production default — this is the direct mechanism for the "validation gate" language used throughout §7, not just a testing nicety.

## 10. Risk Register

| # | Risk | Likelihood | Impact | Mitigation |
| --- | --- | --- | --- | --- |
| 1 | Native Fast-LIO2/PGO/Localizer introduces numerical or timing regressions vs. the battle-tested ROS2 build, in the one subsystem that is safety-critical | Medium | High | Dual-run shadow validation (§7 Phase 5-6) on recorded and live data before any default flip; `slam_profile` flag as instant rollback |
| 2 | Vendored third-party packages (`livox_ros_driver2`, `OrbbecSDK_ROS2`) receive upstream bugfixes/hardware-support updates that a hand-rolled native replacement won't automatically inherit | Medium | Medium | Pin an explicit SDK version per native integration; track upstream changelogs; budget periodic re-sync as ongoing maintenance, not a one-time cost |
| 3 | Two parallel build systems (colcon/ament_cmake + plain CMake) increase CI and maintenance burden during the multi-phase transition | High | Low-Medium | Time-box the dual-build period per phase; delete/archive legacy build files once a phase's gate passes, following the `COLCON_IGNORE` precedent already used for `legacy_ros` |
| 4 | Hidden ROS2 coupling via `ros2` CLI subprocess calls (e.g. `map_save.py`) is invisible to a naive `import rclpy` grep and could be missed by later cleanup passes | Medium | Medium | Phase 0 CI guardrail explicitly greps for `ros2 ` subprocess invocations and `/opt/ros/` string references, not just Python imports |
| 5 | Field validation requires real S100P hardware and outdoor test time, which is a scarce, slow resource relative to software iteration speed | High | Medium | Sequence phases so each gets its own short field slot; use recorded-data replay for the bulk of validation, reserve live field time for the final promotion gate only |
| 6 | Native transform mechanism (§5.e) has a latent behavioral gap if some consumer assumes general `tf2` semantics (arbitrary-time interpolation, multi-hop lookups) that the minimal fixed-frame channel doesn't implement | Medium | Medium | Enumerate every current TF consumer, not just the known publisher, before finalizing the native transform API (open question §12.1) |
| 7 | "Portable / no OS lock-in" claim gets ahead of reality if some surviving component still silently assumes Linux/systemd specifically | Medium | Low | Scope the portability claim explicitly to what has been proven (native build + CI), do not imply full cross-OS robot deployment beyond what Phase 8 actually validates |
| 8 | Native binaries (DUFOMap, nav_kernel, local-planner) built against a specific Ubuntu 22.04 toolchain could silently re-introduce the same OS pin this plan is trying to remove, even after ROS2 itself is gone | Medium | Medium | Phase 0 explicitly inventories the toolchain requirements of every already-native binary before declaring Phase 8 complete |
| 9 | Team re-litigates already-decided transport policy (LCM/DDS/SHM split) mid-migration, causing churn | Low-Medium | Medium | This plan explicitly defers to `LINGTU_RUNTIME_BUS_DECISION.md` (§3); any proposed change to that policy should be a separate, explicit ADR-style decision, not an incidental side effect of this migration |

## 11. Validation Gate Quick Reference

| Phase | Gate |
| --- | --- |
| 0 | CI guardrail merges without touching runtime code |
| 1 | Framework + C++ standalone tests pass with `legacy_ros` removed from the build graph |
| 2 | Native vs. ROS2 LiDAR point-cloud parity within tolerance; downstream Fast-LIO2 odometry drift equivalent on both |
| 3 | Native vs. ROS2 camera perception-pipeline output parity within tolerance |
| 4 | No production profile defaults to the ROS2 GNSS reader |
| 5 | Native Fast-LIO2 odometry/map-cloud output matches ROS2 baseline within tolerance across recorded + live dual-run sessions |
| 6 | Native PGO/Localizer relocalization success rate and map-artifact compatibility match or exceed ROS2 baseline |
| 7 | N consecutive field days on native default with zero fallback incidents |
| 8 | Clean-machine build (no ROS2/Ubuntu-22.04-specific packages) succeeds end-to-end for all default profiles; ROS2 compat build still succeeds independently |

## 12. Open Questions / Decisions Needed From the Team

1. **TF consumer audit**: beyond the one known publisher (`pgo_node.cpp`'s `TransformBroadcaster`), what is the complete set of current `map->odom` transform *consumers*? This must be enumerated before finalizing the native transform API in §5.e/Phase 6.
2. **Target OS/hardware matrix**: is the goal "any Linux, no Ubuntu-version pin" or genuinely cross-OS (including the Windows dev-loop implied by `portable-fastlio2-windows-gate.yml`)? `ROS_ROLE_REPLACEMENT_MAP.md` already states Windows is not a product deployment endpoint — confirming this scope explicitly changes how aggressively Linux/systemd-specific assumptions (independent of ROS2 itself) need to be removed.
3. **Dual-run duration threshold**: how many field-hours or field-days of native/ROS2 dual-run agreement are required before Phase 7 can flip the production default? This plan intentionally does not invent a number without a domain-expert/safety sign-off.
4. **Localization accuracy evaluation ownership**: is there an existing eval harness/dataset for comparing SLAM odometry drift and relocalization success rate numerically, or does one need to be built as part of Phase 5? Who owns defining the numeric tolerance referenced throughout §7/§11?
5. **Scope of sensor-ingestion cutover**: should Livox/Orbbec native promotion (Phases 2-3) be tracked inside this plan's timeline, or as an independently-scheduled workstream, given they already have native prototypes and the remaining work is closer to "product decision to promote" than "build the thing"?
6. **Is "no ROS 2" the actual requirement, or is "no forced Ubuntu 22.04" the actual requirement?** These are not the same ask. If the underlying need is only to escape the specific OS/distro pin, tracking a newer ROS 2 distribution (e.g. Jazzy on Ubuntu 24.04) for the compatibility profile could satisfy it at a fraction of this plan's engineering cost — while the native-runtime work in this plan would still be worth doing independently for performance and long-term portability reasons, but could be de-prioritized relative to other roadmap items. This plan proceeds under the assumption that full removal from the product's main path is the actual goal (per the founder's stated motivation), but it is flagged here because it materially changes the urgency and resourcing of Phases 5-8 specifically.
7. **Native transform performance/semantics budget**: does any current consumer need `tf2`-style time-interpolated lookups, or is "latest value" sufficient — matching the Module framework's general latest-value port semantics? This determines whether §5.e's minimal fixed-frame channel is actually sufficient or needs an interpolation layer.
8. **Are `src/runtime/adapters/dds/` and `src/runtime/adapters/lcm/` a deliberate boundary-type split, or two competing implementations that need to converge on one?** Per §2.6a: both packages currently implement an overlapping `endpoint_runner.py` / `endpoint_service.py` / `contracts.py` / `localization_adapter.py` surface for the same `thunder_field` boundary; `RUNTIME_ENDPOINTS["thunder_field"].endpoint_transport` currently resolves to `"dds"` while `LINGTU_RUNTIME_BUS_DECISION.md` and `test_lcm_endpoint_contracts.py` both say it should be `"lcm"`; and neither package has git history in the current tree to establish which is newer (both are untracked). Until this is resolved, §3's claim that "the multi-transport policy Python modules already use" is settled should be read narrowly as "the *policy* is decided; the *code* has not yet converged on it for `thunder_field` specifically." Needs an explicit team decision — keep `dds` and update the decision doc plus the LCM contract test; keep `lcm` and fix the endpoint spec; or confirm a genuine split and give each package disjoint, non-overlapping responsibilities — before Phase 2+ native work builds on top of either one (Phase 0 work item, §7).

## 13. References

- `docs/architecture/LINGTU_RUNTIME_BUS_DECISION.md` — transport policy (LCM/DDS/SHM split)
- `docs/architecture/ROS_ROLE_REPLACEMENT_MAP.md` — current ROS-role-to-adapter mapping
- `docs/architecture/ros_frame_contract.md` — frame naming and TF shape contract
- `docs/architecture/PORTABLE_LEAN_PACKAGE_MATRIX.md` — dependency tiering, incl. "ROS compatibility" tier
- `docs/architecture/THUNDER_RUNTIME_REFACTOR_PLAN.md` — structural/style reference for this document
- `docs/plans/PRD-lingtu-native-slam-navigation-runtime.md` — native SLAM/`ISlamBackend` product direction
- `docs/plans/PRD-slam-transport-navigation-dataflow.md` — typed DDS/IDL schema and native runtime deployment unit
- `src/localization/interface/README.md` — SLAM service inventory and runtime boundary
- `src/localization/slam/cpp/CMakeLists.txt` — native/ROS2-adjacent/pure-CycloneDDS build flags
- `src/runtime/tf/README.md` — ROS-free transform library scope statement
- `src/nav/local/legacy_ros/*/COLCON_IGNORE` — executed precedent for quarantining a retired ROS2 subsystem
