# OctoPlanner3D Map Artifact Closed Loop Implementation Plan

> **For Hermes:** Use subagent-driven-development. Work in bounded slices, no commits, no hardware commands, and do not broaden the architecture beyond the files named here.

**Goal:** Turn the already-verified OctoPlanner3D official `.pcd -> OctoMap -> GlobalPlanner` chain into a LingTu system loop: saved map artifacts -> active map -> OctoPlanner3D global path -> local/path-follow validation.

**Architecture:** `.pcd` is source data; `octomap.bt` is the runtime planning artifact. Map conversion belongs to map-save/build, not every navigation plan. Windows/MuJoCo should consume `.bt` and must not require PCL/ROS2 by default; real robot mapping may use the native/PCL feature pack.

**Branch policy:** Stay on the current dirty branch `codex/simulation-architecture-hardening`. Preserve existing user/teammate changes. Do not commit unless explicitly asked.

**Execution order:**

1. W0 baseline isolation: snapshot current dirty branch and focused OctoPlanner3D tests.
2. A1 map artifact builder: `map.pcd` / `patches+poses` -> `octomap.bt`, `occupancy.npz`, `metadata.json`.
3. A4 path feasibility gate: report whether a global path is ground-executable, not merely `reached_goal`.
4. A5 native packaging: make repo-local PCL and OctoPlanner3D headless build reproducible/diagnosable.
5. A2 active map integration: `nav` automatically selects active `octomap.bt` with clear fallback diagnostics.
6. A3 Windows MuJoCo OctoPlanner3D gate: sensors + global/local path + cmd_vel + motion, no ROS2/PCL default.
7. A6 integration QA: focused tests, collect-only classification, artifact replay, diff split guidance.

---

## Agent W0: Baseline isolation

**Objective:** Freeze current state before parallel work.

**Outputs:**
- `artifacts/octoplanner3d_current_state/git_status_short.txt`
- `artifacts/octoplanner3d_current_state/git_diff_shortstat.txt`
- `artifacts/octoplanner3d_current_state/octoplanner_focused_tests.txt`
- this plan file

**Verification:**

```bash
uv run python -m pytest -q -p no:cacheprovider \
  src/nav/tests/planning_backends/test_octoplanner3d_backend.py \
  src/nav/tests/planning_backends/test_planner_registry.py
```

---

## Agent A1: MapArtifactBuilder

**Objective:** Build stable map artifacts once during map save/build, never per plan call.

**Files:**
- Create: `src/nav/services/map_layers/map_artifact_builder.py`
- Create/modify tests: `src/nav/tests/test_map_artifact_builder.py`
- Later modify: `src/nav/services/maps.py`

**Contract:**

Saved map directories should converge to:

```text
maps/<name>/
  map.pcd
  patches/*.pcd
  poses.txt
  octomap.bt
  occupancy.npz
  metadata.json
```

`metadata.json` must record source hashes, `build_mode`, resolution, frame, and builder version. Distinguish `raycast`, `occupied_only`, and `external_pcl_converter`.

**First implementation slice:** create builder/report classes and tests. If native converter is unavailable, tests may use a fake converter command but metadata behavior must be real.

**Gate:**

```bash
uv run python -m pytest -q -p no:cacheprovider src/nav/tests/test_map_artifact_builder.py
```

---

## Agent A4: Ground path feasibility

**Objective:** Decide whether a planner output is usable for a ground robot.

**Files:**
- Create: `src/nav/services/plan/global_planner/path_feasibility.py`
- Create: `src/nav/tests/test_path_feasibility.py`
- Later integrate diagnostics into `src/nav/services/plan/global_planner/service.py`

**Checks:**
- path point count
- 3D length and XY length
- `z_min`, `z_max`, `z_range`
- max per-segment `dz`
- max slope
- simple verdict/reasons, e.g. `z_range_too_large`, `segment_slope_too_large`

**Expected behavior:** official `building2_9` path should be `planner_ok=true` but `ground_executable=false` because `z_range鈮?3.6m`.

**Gate:**

```bash
uv run python -m pytest -q -p no:cacheprovider src/nav/tests/test_path_feasibility.py
```

---

## Agent A5: Native/PCL packaging

**Objective:** Turn the local PCL + OctoPlanner3D headless workaround into reproducible build/doctor surfaces.

**Files:**
- Modify: `scripts/build/build_vendored_pcl.sh`
- Modify: `scripts/build/build_octoplanner3d.sh`
- Modify: `scripts/build/README.md`
- Later add doctor: `tools/validate/validate_native_dependencies.py` or existing native diagnostics module

**Profiles:**
- `octoplanner3d-converter-native`: PCL common/io/octree + OctoMap, no ROS2.
- `slam-native`: broader Fast-LIO2/localizer/PGO dependencies; do not conflate this with the OctoPlanner3D converter subset.

**Gate:**

```bash
ldd build/octoplanner3d_headless_wsl_pcl/octoplanner3d_headless | grep -E 'pcl|octomap'
```

---

## Agent A2: Active map integration

**Depends on:** A1.

**Objective:** Make `planner=octoplanner3d` use active `octomap.bt` automatically.

**Files:**
- Modify: `src/nav/services/maps.py`
- Modify: `src/nav/services/plan/global_planner/service.py`
- Tests: `src/nav/tests/test_global_planner_diagnostics.py`, `src/nav/tests/test_nav_services.py`

**Rules:**
- Prefer active `octomap.bt`.
- Do not convert `.pcd` during every plan call.
- If artifact is missing, report `active_map_missing_octomap` and fallback to A* only with explicit diagnostics.

---

## Agent A3: Windows MuJoCo + OctoPlanner3D gate

**Depends on:** A1/A2.

**Objective:** Run desktop MuJoCo navigation with sensors and OctoPlanner3D without ROS2/PCL default.

**Files:**
- Modify: `sim/validation/full_system.py`
- Modify: relevant simulation profile files under `src/runtime/profiles/` and `src/runtime/blueprints/`
- Tests: `sim/tests/test_sim_full_system_validation.py`, `sim/tests/test_mujoco_mid360_pattern.py`

**Required evidence:** lidar, IMU, RGB/depth/intrinsics, `global_path>0`, `local_path>0`, `cmd_vel` counters, movement, `ros2_required=false`, `planner=octoplanner3d`.

---

## Agent A6: Integration QA

**Depends on:** all implementation slices.

**Objective:** Verify the merged behavior, classify failures, and prepare a clean diff split.

**Focused gate:**

```bash
uv run python -m pytest -q -p no:cacheprovider \
  src/nav/tests/planning_backends/test_octoplanner3d_backend.py \
  src/nav/tests/planning_backends/test_planner_registry.py \
  src/nav/tests/test_map_artifact_builder.py \
  src/nav/tests/test_path_feasibility.py \
  src/nav/tests/test_global_planner_diagnostics.py
```

**Collect gate:**

```bash
uv run python -m pytest --collect-only -q
```

If collection fails, classify failures as current-slice, historical dirty-branch, or environment/native dependency.
