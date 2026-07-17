# Simulation Architecture Hardening Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Improve LingTu simulation fidelity and lock the runtime/profile architecture so server-side simulation claims stay reproducible and Module-First safe.

**Architecture:** Keep runtime behavior inside existing Module-First boundaries. Harden shared parsing/contract helpers first, then add behavior-locking tests before larger ownership moves such as profile centralization or Gateway service adapters.

**Tech Stack:** Python 3.10+ target runtime, pytest, LingTu Module/Blueprint framework, MuJoCo XML scene fixtures, existing server simulation closure gates.

---

## File Structure

- Modify: `src/drivers/sim/mujoco/scene.py`
  - Shared MuJoCo XML transform helpers for body/geom world poses.
- Modify: `src/drivers/sim/pointcloud.py`
  - Static point-cloud generation from transformed box geoms.
- Modify: `sim/validation/full_system.py`
  - Dependency-failure classification for simulation validation checks.
- Modify: `src/runtime/tests/test_profile_graph_snapshots.py`
  - Simulation profile/endpoint/data-source matrix locks.
- Modify: `src/runtime/tests/test_mujoco_scene_metadata.py`
  - Parent-body transform and rotation metadata regression.
- Create: `src/drivers/tests/test_sim_pointcloud_provider.py`
  - Static point-cloud transform and robot-placeholder exclusion regression.
- Modify: `sim/tests/test_sim_full_system_validation.py`
  - Environment dependency blocked/fail classification tests.
- Modify: `tests/README.md`
  - Correct canonical simulation closure test path.
- Modify: `src/runtime/blueprints/stacks/perception.py`
  - Resolve camera only when the active driver path needs it.
- Modify: `src/runtime/tests/test_stack_registry_resolution.py`
  - Lock camera resolution and driver-camera skip behavior.
- Modify: `src/drivers/sim/mujoco/driver.py`
  - Avoid duplicating the repo import root during MuJoCo setup.
- Modify: `src/drivers/tests/test_mujoco_driver_contract.py`
  - Lock MuJoCo setup import-root idempotence.

## Task 1: Environment-Aware Full-System Validation

**Files:**
- Modify: `sim/validation/full_system.py`
- Modify: `sim/tests/test_sim_full_system_validation.py`

- [x] **Step 1: Add missing dependency classification**

Use `ENVIRONMENT_DEPENDENCY_MODULES` and make `_timed()` return `BLOCKED` for missing external runtime dependencies such as `yaml`, `numpy`, `mujoco`, `onnxruntime`, `cv2`, and `scipy`.

- [x] **Step 2: Keep internal import typos as failures**

Add a regression test where `ModuleNotFoundError(name="runtime.internal_typo")` remains `FAIL`.

- [x] **Step 3: Verify**

Run:

```bash
python -m pytest sim/tests/test_sim_full_system_validation.py::test_timed_marks_missing_environment_dependency_as_blocked sim/tests/test_sim_full_system_validation.py::test_timed_keeps_internal_missing_module_as_failure -q --tb=short
```

Expected: `2 passed`.

## Task 2: Shared MuJoCo Geometry Transforms

**Files:**
- Modify: `src/drivers/sim/mujoco/scene.py`
- Modify: `src/drivers/sim/pointcloud.py`
- Modify: `src/runtime/tests/test_mujoco_scene_metadata.py`
- Create: `src/drivers/tests/test_sim_pointcloud_provider.py`

- [x] **Step 1: Add shared transform helpers**

Implement helpers for parent maps, ancestor name chains, local euler/quat rotations, world pose composition, and oriented-box AABB half-size calculation in `mujoco_scene_metadata.py`.

- [x] **Step 2: Use shared helpers in metadata extraction**

Use world pose and rotated AABB half sizes for box obstacles so nested bodies and rotated boxes are represented correctly.

- [x] **Step 3: Use shared helpers in static point clouds**

Use ancestor names for exclusion, world pose for center, and rotation-aware sampling for box perimeter points.

- [x] **Step 4: Add regressions**

Lock a nested body with `pos="1 2 0.5"` and `euler="0 0 1.57079632679"` plus a child box at `pos="1 0 0"`. Expected world center is `[1.0, 3.0, 0.5]`; expected rotated AABB half size is `[0.25, 1.0, 0.4]`.

- [x] **Step 5: Verify**

Run:

```bash
python -m pytest src/runtime/tests/test_mujoco_scene_metadata.py src/drivers/tests/test_sim_pointcloud_provider.py -q --tb=short
```

Expected on a healthy Python/NumPy runtime: all tests pass. In the current Windows Python 3.13 environment, NumPy import crashes before pytest can collect the provider test; verify with the bundled Python script noted in the session if needed.

## Task 3: Simulation Profile/Endpoint Matrix

**Files:**
- Modify: `src/runtime/tests/test_profile_graph_snapshots.py`

- [x] **Step 1: Add profile data-source matrix**

Assert every `SIMULATION_PROFILES` entry maps to the expected `profile_data_source`, optional profile-level simulation contract, and compatibility launcher fields from `runtime.runtime_profiles.PROFILES`.

- [x] **Step 2: Add endpoint run-spec matrix**

For `mujoco_live`, `replay`, `gazebo`, and `cmu_unity`, assert every supported profile produces a `RuntimeRunSpec` with matching endpoint, data source, runtime contract, launcher, default args, command sink, and `LINGTU_SIMULATION_ONLY=1`.

- [x] **Step 3: Verify**

Run:

```bash
python -m pytest src/runtime/tests/test_profile_graph_snapshots.py::test_simulation_profiles_match_runtime_data_source_matrix src/runtime/tests/test_profile_graph_snapshots.py::test_simulation_endpoints_generate_coherent_runtime_run_specs -q --tb=short
```

Expected on a healthy local runtime: `2 passed`.

## Task 4: Exploration Profile Behavior Locks

**Files:**
- Create: `src/runtime/tests/test_profile_exploration_wiring.py`

- [x] **Step 1: Add `explore` wavefront/traversable-frontier test**

Create a test that builds `graph_for_profile("explore")`, asserts `WavefrontFrontierExplorer` and `TraversableFrontierModule` are present, asserts `TAREExplorerModule` is absent, and asserts `WavefrontFrontierExplorer.exploration_goal->NavigationModule.goal_pose` is wired.

- [x] **Step 2: Add `tare_explore` TARE-only test**

Create a test that builds `graph_for_profile("tare_explore", run_startup_checks=False, manage_external_services=False)` if the local TARE constructor can compile without the external binary. If not, assert the profile config instead: `enable_frontier is False`, `exploration_backend == "tare"`, and no wavefront goal ownership is claimed.

- [x] **Step 3: Verify**

Run:

```bash
python -m pytest src/runtime/tests/test_profile_exploration_wiring.py -q --tb=short
```

Expected on a healthy runtime: tests pass without launching real robot services.

## Task 5: Immediate Low-Risk Architecture Fixes

**Files:**
- Modify: `src/runtime/blueprints/stacks/perception.py`
- Modify: `src/runtime/tests/test_stack_registry_resolution.py`
- Modify: `src/drivers/sim/mujoco/driver.py`
- Modify: `src/drivers/tests/test_mujoco_driver_contract.py`

- [x] **Step 1: Lazy-resolve camera**

Move camera module resolution inside the external-camera branch so sim drivers with native camera ports do not import the real camera module at blueprint construction time.

- [x] **Step 2: Lock camera resolution behavior**

Assert MuJoCo native camera and ROS2 sim driver-camera paths do not resolve camera, while external camera paths still do.

- [x] **Step 3: Fix MuJoCo setup import-root guard**

Check and insert the same repo-root path in `MujocoDriverModule.setup()` so repeated setup does not duplicate `sys.path`.

- [x] **Step 4: Verify**

Run syntax checks and targeted behavior scripts; run pytest on a healthy Python/NumPy runtime.

## Task 6: Larger Architecture Follow-Ups

**Files:**
- Create: `src/runtime/runtime_profiles.py`
- Modify: `cli/profiles_data.py`
- Modify: `src/runtime/blueprints/profile_graph.py`
- Modify: `src/runtime/tests/test_module_boundaries.py`
- Modify: `src/runtime/tests/test_profile_graph_snapshots.py`
- Modify: `src/runtime/blueprints/stacks/driver.py`
- Modify: `src/gateway/gateway_module.py`
- Modify: `src/gateway/mcp_server.py`
- Modify: `src/gateway/tests/test_gateway_runtime_status.py`
- Create: `src/localization/relocalization.py`
- Create: `src/localization/tests/test_relocalization.py`
- Modify: `src/localization/bridge.py`
- Modify: `src/gateway/routes/operations.py`
- Modify: `src/gateway/tests/test_gateway_session_map_contract.py`

- [x] **Step 1: Centralize profile ownership**

Move profile/preset source of truth out of CLI into a neutral core/runtime profile module, then let CLI import that module. Add an import-boundary regression forbidding production `runtime.blueprints.* -> cli.*` after migration.

- [x] **Step 2: Add driver-profile sync tests**

Assert each robot preset used by profiles resolves through the driver stack or is explicitly documented as non-runtime.

- [x] **Step 3: Introduce Gateway-facing service adapters**

Replace direct `_all_modules["NavigationModule"]` lookups for preview/reconfigure with small protocol adapters. Keep existing routes and output schemas stable while migrating one route at a time.

Implemented the first adapter boundary for backend reconfiguration: Gateway now caches explicit backend-switch module references and a public NavigationModule reference from `on_system_modules()`, rather than retaining the full module graph for switching. Gateway and MCP no longer fall back to private `NavigationModule._state`; motion backend switching requires public `health()` state.

Continuation hardening:

- Restored `_all_modules` as a read-only compatibility inventory for status, readiness, diagnostics, and runtime dataflow snapshots, while keeping backend mutation paths on `_navigation_module` and `_backend_reconfigure_modules`.
- Added `_cmd_vel_mux` and `_navigation_module` preference in runtime status so navigation status can still be built from narrow injected references when the full module inventory is absent or empty.
- Made health, devices, and diagnostic-pack routes tolerate missing `_all_modules` and added regressions for those startup-safe paths.

Verification:

```bash
python -m py_compile src\gateway\gateway_module.py src\gateway\services\runtime_status.py src\gateway\routes\status.py src\gateway\routes\diagnostics.py src\gateway\tests\test_gateway_runtime_status.py src\gateway\tests\test_gateway_health_contract.py src\gateway\tests\test_gateway_route_split.py
python -m pytest src\\localization\tests\test_relocalization.py -q
git diff --check
```

Additional Gateway compatibility was verified with a direct route script that stubs only NumPy/YAML imports on this Windows host, then confirms injected navigation/CmdVelMux runtime refs, health/devices without `_all_modules`, and diagnostic-pack export with an empty module inventory.

Code-review closure:

- Added a core-owned `RelocalizationService` protocol so Gateway depends on an injected capability instead of importing `localization.relocalization` directly. `GatewayModule.on_system_modules()` discovers the capability from SLAM modules, and operation routes return 503 when the adapter is unavailable after request/map validation.
- Extended Module-First boundary tests to forbid production `gateway -> slam` imports.
- Fixed `SlamBridgeModule._auto_relocalize()` so a ROS service response with process return code 0 but `success=False` leaves relocalization in `failed` instead of resetting drift and marking recovery `completed`.
- Consolidated `RelocalizationResult` into the core-owned contract after making `core` and `runtime.transport` light enough to import without eager optional backend imports.

Review-fix verification:

```bash
python -m py_compile <all changed and untracked Python files>
python -m pytest src\\localization\tests\test_relocalization.py -q
python -m pytest sim\tests\test_sim_full_system_validation.py::test_scene_catalog_identifies_multifloor_building_contract sim\tests\test_sim_full_system_validation.py::test_timed_marks_missing_environment_dependency_as_blocked sim\tests\test_sim_full_system_validation.py::test_timed_keeps_internal_missing_module_as_failure -q
git diff --check
```

Additional AST/direct checks verified Module-First import boundaries and Gateway relocalization adapter behavior without calling ROS subprocesses.

Continuation hardening:

- Made `runtime.transport` optional SHM/DDS/Dual backend exports lazy so importing base `core` or `runtime.relocalization` does not import NumPy-backed SHM code.
- Made `runtime.blueprints` package exports lazy and moved `profile_graph` full-stack import into graph construction, so profile/runtime matrix tests can inspect data without loading full runtime modules.
- Moved full-stack calibration self-check import into the startup preflight branch and made `runtime.utils` validation exports lazy.
- Fixed `SlamBridgeModule._auto_relocalize()` branch ordering so an unsupported backend uses the backend recovery action, while a missing active map still reaches the SLAM restart fallback.

Verification:

```bash
python -m pytest src\\localization\tests\test_relocalization.py -q
python -m pytest src\runtime\tests\test_module_boundaries.py::test_core_blueprints_do_not_import_cli_profile_surfaces src\runtime\tests\test_module_boundaries.py::test_package_does_not_import_forbidden_layers_directly -q --tb=short -p no:cacheprovider
python -m pytest src\runtime\tests\test_stack_registry_resolution.py::test_perception_stack_skips_camera_resolution_for_driver_camera src\runtime\tests\test_stack_registry_resolution.py::test_perception_stack_resolves_camera_for_external_camera -q --tb=short -p no:cacheprovider
python -m pytest src\runtime\tests\test_profile_graph_snapshots.py::test_top_level_blueprint_api_exposes_all_stack_factories src\runtime\tests\test_profile_graph_snapshots.py::test_simulation_profiles_match_runtime_data_source_matrix src\runtime\tests\test_profile_graph_snapshots.py::test_simulation_endpoints_generate_coherent_runtime_run_specs -q --tb=short -p no:cacheprovider
python -m pytest sim\tests\test_sim_full_system_validation.py::test_scene_catalog_identifies_multifloor_building_contract sim\tests\test_sim_full_system_validation.py::test_timed_marks_missing_environment_dependency_as_blocked sim\tests\test_sim_full_system_validation.py::test_timed_keeps_internal_missing_module_as_failure -q --tb=short -p no:cacheprovider
```

- [x] **Step 4: Move relocalization subprocess logic behind a service boundary**

Gateway should request relocalization; bridge/service-manager code should own ROS command assembly and subprocess execution.

Implemented a SLAM-side `relocalization_service` helper that owns ROS service command construction, environment injection, subprocess execution, timeout mapping, and quality parsing. `SlamBridgeModule` exposes a narrow relocalization capability backed by that helper. Gateway relocalize endpoints retain validation/envelope/persistence behavior while calling the injected core `RelocalizationService` protocol, so production Gateway code does not import `slam` or assemble ROS subprocess commands. `GatewayModule._spawn_auto_relocalize()` uses the same injected capability, while `SlamBridgeModule._auto_relocalize()` calls the SLAM-side helper directly.

Verification:

```bash
python -m py_compile src\\localization\relocalization.py src\gateway\routes\operations.py src\gateway\gateway_module.py src\\localization\bridge.py src\\localization\tests\test_relocalization.py src\gateway\tests\test_gateway_session_map_contract.py
python -m pytest src\\localization\tests\test_relocalization.py -q
git diff --check
```

Additional Gateway endpoint behavior was verified with a direct route script that stubs only NumPy/YAML imports on this Windows host, then confirms auto relocalize delegation, saved-map relocalize validated path forwarding, success-only pose persistence, and timeout-to-504 mapping.

Continuation hardening:

- Added a low-dependency regression that parses `core/runtime_interface.py` to keep `localization.relocalization` service tokens synchronized with `RuntimeTopics` without importing the full `core` package.
- Added a low-dependency AST regression that verifies the two Gateway relocalize route functions do not import or call `subprocess` directly. Bag recording subprocess behavior in the same route module remains out of scope.
- Strengthened Gateway contract tests so relocalize endpoint delegation tests patch global `subprocess.run` fail-fast while stubbing the unrelated unsupported-backend status helper.

Verification:

```bash
python -m py_compile src\\localization\relocalization.py src\\localization\tests\test_relocalization.py src\runtime\tests\test_module_boundaries.py src\gateway\tests\test_gateway_session_map_contract.py
python -m pytest src\\localization\tests\test_relocalization.py -q
```

Gateway behavior was re-verified with a direct route script using NumPy/YAML stubs and fail-fast `subprocess.run`; the script confirmed auto and saved-map relocalize route delegation, success-only pose persistence, and timeout-to-504 mapping. Running `src/runtime/tests/test_module_boundaries.py::test_gateway_relocalization_routes_delegate_subprocess_execution` under this host's pytest is not a reliable signal because `src/runtime/tests/conftest.py` imports `core` during session finish and the current Windows Python/NumPy stack exits with access violation `-1073741819`.

Continuation hardening:

- Added a static profile graph path that compiles primary profile module/wire graphs without constructing runtime blueprints or importing the message/NumPy stack. `graph_for_profile()` now defaults to `mode="static"` while retaining `mode="runtime"` for the previous behavior.
- Extracted pure full-stack wire specs and navigation config helpers so profile/stack tests can verify architecture data without resolving runtime module classes.
- Added `runtime.msgs.numpy_compat` and moved additional NumPy users behind lazy imports across core message types, Gateway map/status paths, base autonomy modules, SLAM bridge/visual odom, semantic reconstruction package exports, and simulation gate scripts.
- Kept saved-map JSON and PCD snapshot routes usable in control-plane tests without NumPy by parsing/writing binary XYZ PCD through the standard library.
- Reworked gateway dry-run and server-sim/video validation gates so local contract tests do not require importing the full GatewayModule, OpenCV, or NumPy in this broken Windows host.

Verification:

```bash
python -m pytest src\runtime\tests\test_profile_graph_snapshots.py src\runtime\tests\test_profile_exploration_wiring.py -q --tb=short -p no:cacheprovider
python -m pytest src\\localization\tests\test_relocalization.py src\gateway\tests\test_gateway_session_map_contract.py -q --tb=short -p no:cacheprovider
python -m pytest src\runtime\tests\test_module_boundaries.py::test_core_blueprints_do_not_import_cli_profile_surfaces src\runtime\tests\test_module_boundaries.py::test_package_does_not_import_forbidden_layers_directly src\runtime\tests\test_stack_registry_resolution.py -q --tb=short -p no:cacheprovider
python -m pytest sim\tests\test_server_sim_closure.py -q --tb=short -p no:cacheprovider
python -m pytest sim\tests\test_sim_full_system_validation.py::test_scene_catalog_identifies_multifloor_building_contract sim\tests\test_sim_full_system_validation.py::test_timed_marks_missing_environment_dependency_as_blocked sim\tests\test_sim_full_system_validation.py::test_timed_keeps_internal_missing_module_as_failure src\drivers\tests\test_sim_pointcloud_provider.py -q --tb=short -p no:cacheprovider
python -m py_compile <all changed and untracked Python files>
git diff --check
```

Results: profile graph/exploration `40 passed`; Gateway/SLAM contracts `47 passed`; Module boundary/registry `32 passed`; server-sim closure `131 passed`; sim validation/provider `3 passed, 1 skipped` because this host cannot safely import NumPy. `git diff --check` passed.

Continuation hardening:

- Extended runtime switch tests so in-process simulation profiles (`sim`, `sim_nav`) explicitly resolve without endpoint/launcher/runtime-contract environment keys, while first-class simulator profiles (`sim_mujoco_live`, `sim_gazebo`, `sim_industrial`, `sim_cmu_tare`) resolve coherent endpoint/data-source/runtime-contract specs without embedding external launcher metadata in normal profile config.
- Expanded external simulator action-contract coverage from MuJoCo live to CMU Unity/TARE, including default and record launcher arguments.
- Made `MujocoDriverModule` import-safe for control-plane tests by using lazy NumPy access and probing NumPy in an isolated subprocess before importing the MuJoCo engine. If NumPy is unavailable or crashes in the host interpreter, setup now soft-fails with no engine instead of crashing Python.
- Added driver contract regressions for NumPy-free import, NumPy-probe soft failure, and stable repo-root path insertion.

Verification:

```bash
python -m py_compile src\drivers\sim\mujoco_driver_module.py src\drivers\tests\test_mujoco_driver_contract.py src\runtime\tests\test_runtime_switch.py
python -m pytest src\drivers\tests\test_mujoco_driver_contract.py src\runtime\tests\test_runtime_switch.py -q --tb=short -p no:cacheprovider
git diff --check
```

Results: driver/runtime-switch contracts `48 passed`; `git diff --check` passed with Windows line-ending warnings only.

Code-review closure:

- Restored geometry scalar comparison tolerance after lazy-NumPy cleanup by routing `Vector3.__eq__`, `Vector3.is_zero()`, and `Quaternion.__eq__` through explicit `rel_tol=1e-5` / `abs_tol=1e-8` checks, with a low-dependency regression for near-zero values.
- Converted `SimPointCloudProvider` to lazy NumPy access and added an import guard so the module can be imported on hosts where NumPy crashes.
- Reworked server-side full-system navigation wiring validation to use the static profile graph instead of building a runtime `full_stack_blueprint()` in the default static validation path. `_numpy()` in the validation module now probes NumPy in a subprocess and reports a catchable missing dependency when the host runtime is unsafe.
- Made `policy_nav_smoke.py` import-safe by replacing its top-level NumPy import with lazy access after project path setup.
- Added a vectorized ndarray fast path for Gateway binary XYZ PCD snapshot writing while preserving the existing pure-Python iterable fallback.

Verification:

```bash
python -m py_compile src\runtime\msgs\geometry.py src\runtime\tests\test_geometry_tolerance.py src\drivers\sim\sim_pointcloud_provider.py src\drivers\tests\test_sim_pointcloud_provider.py sim\validation\full_system.py sim\tests\test_sim_full_system_validation.py sim\scripts\policy_nav_smoke.py src\gateway\routes\maps.py src\gateway\tests\test_gateway_session_map_contract.py src\drivers\sim\mujoco_driver_module.py src\drivers\tests\test_mujoco_driver_contract.py src\runtime\tests\test_runtime_switch.py
python -m pytest src\runtime\tests\test_geometry_tolerance.py src\drivers\tests\test_sim_pointcloud_provider.py sim\tests\test_sim_full_system_validation.py src\gateway\tests\test_gateway_session_map_contract.py::test_map_save_falls_back_to_super_lio_live_cloud_snapshot src\gateway\tests\test_gateway_session_map_contract.py::test_binary_xyz_pcd_writer_keeps_numpy_fast_path src\drivers\tests\test_mujoco_driver_contract.py src\runtime\tests\test_runtime_switch.py -q --tb=short -p no:cacheprovider
python lingtu.py --list
git diff --check
```

Results: targeted review-closure contracts `60 passed, 2 skipped`; `sim/tests/test_sim_full_system_validation.py` full file `8 passed`; `python lingtu.py --list` succeeded; `git diff --check` passed with Windows line-ending warnings only.

## Verification Notes

- Current Windows `python` is 3.13 and crashes while importing NumPy. Treat NumPy-based pytest failures in this host as environment blockers, not code evidence.
- Runtime profile graph construction can still import runtime modules and `runtime.msgs`, which require a healthy NumPy runtime. The default static profile graph, profile/runtime data-source matrix, and stack factory checks now run without those imports.
- Server-side full simulation closure still requires fresh artifacts under `artifacts/server_sim_closure/` plus `artifacts/server_sim_closure_summary_g4_current.json`.
- Do not claim G4 full simulation health until `sim/scripts/server_sim_closure.py --preset g4_server_full_sim --required-only --strict` reports `ok=true`, `simulation_only=true`, `real_robot_motion=false`, `cmd_vel_sent_to_hardware=false`, and `missing_or_failed=[]`.

## Final Continuation Status - 2026-06-06

Completed additional hardening after parallel architecture and simulation verifier review:

- Converted remaining control-plane NumPy crash sites in semantic planning and topology graph imports to lazy NumPy access, so the full core test suite no longer crashes on this Windows Python 3.13 host.
- Reused dependency classification for runtime profile parity in `sim/validation/full_system.py`: known host dependencies remain `BLOCKED`, while internal missing modules now become `FAIL`.
- Made default `run_validation(require_all=False)` treat blocked `sim_nav_planning_wiring` runtime parity as a required blocker, while leaving optional live MuJoCo evidence blocked when `--run-mujoco` is not requested.
- Restored PCT ROS executable scripts inside `src/nav/services/plan/global_planner/algorithm/pct/vendor/pct_planner/planner/scripts/` and updated CMake/tests to install package-local scripts instead of stale `src/legacy/pct_planner` paths.
- Kept server-sim video artifact gates strict: missing or undecodable required videos fail instead of passing without evidence.

Verification:

```bash
python -m pytest src\runtime\tests\ -q --tb=short -p no:cacheprovider
python scripts\gates\runtime_contract_audit.py --json
python -m pytest sim\tests\test_sim_full_system_validation.py sim\tests\test_server_sim_closure.py::test_server_sim_closure_rejects_missing_fastlio2_dynamic_inspection_video_file sim\tests\test_server_sim_closure.py::test_server_sim_closure_rejects_undecodable_fastlio2_dynamic_inspection_video_file sim\tests\test_server_sim_closure.py::test_server_sim_closure_accepts_fastlio2_dynamic_inspection_core_gate sim\tests\test_sim_runtime_compat.py -q -rs --tb=short -p no:cacheprovider
python -m pytest src\gateway\tests\ -q --tb=short -p no:cacheprovider
python -m py_compile src\nav\services\plan\global_planner\algorithm\pct\vendor\pct_planner\planner\scripts\global_planner.py src\nav\services\plan\global_planner\algorithm\pct\vendor\pct_planner\planner\scripts\pct_planner_astar.py src\nav\services\plan\global_planner\algorithm\pct\vendor\pct_planner\planner\scripts\fake_localization.py
```

Results:

- Core tests: `1216 passed, 7 skipped`.
- Runtime contract audit: `ok=true`, no blockers.
- Simulation focused closure/runtime tests: `13 passed, 2 skipped`; skips are unsafe NumPy and safe OpenCV/NumPy video generation on this host.
- Gateway tests: `373 passed, 5 skipped`.
- PCT package script syntax: passed.

Parallel review findings folded into follow-up risk:

- Static profile graph remains a parallel model of full-stack assembly. It is now guarded by tests and runtime parity, but long-term work should derive static and runtime graphs from one manifest/spec to avoid drift.
- Robot preset values still have duplicate sources between runtime profiles and driver stack registry; current tests cover resolution, but a value-level sync test or single source of truth is still preferred.
- Black-frame video rejection exists in the gate, but the positive decodable-video path is skipped on this host because safe NumPy/OpenCV imports are unavailable.
