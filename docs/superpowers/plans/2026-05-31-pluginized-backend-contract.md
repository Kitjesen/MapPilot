# Pluginized Backend Contract Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make every replaceable LingTu algorithm/provider report and resolve backends through a consistent plugin contract, so swapping local detection, planning, LLM, memory, SLAM, or exploration implementations does not require editing business modules.

**Architecture:** Keep Module-First boundaries: modules own ports and runtime state; registries own provider lookup; health owns configured/effective/degraded reporting. The first landing slice adds shared backend status and migrates autonomy hot-path modules without changing algorithm output. Later slices replace hardcoded factories with registry-first factories while preserving current defaults as compatibility fallbacks.

**Tech Stack:** Python modules, `runtime.registry`, `runtime.backend_status.BackendStatus`, pytest, existing blueprint stack factories.

---

## Audit Baseline

Two read-only agents and local inspection found these pluginization gaps:

- Navigation/autonomy: `TerrainModule`, `LocalPlannerModule`, `PathFollowerModule`, and `navigation()` still branch on backend strings internally. `GlobalPlannerService` already uses `runtime.registry` for `planner_backend`.
- Perception/semantic: `PerceptionModule` and `PerceptionFactory` hardcode detector/encoder/tracker creation. `llm_client.create_llm_client()` hardcodes LLM backend aliases. `SemanticPlannerModule` constructs `GoalResolver` with a default `LLMConfig()` instead of the planner stack's selected backend.
- Memory/reconstruction: vector-memory encoder fallback and reconstruction backend registry exist, but health/status fields are not aligned with configured/effective/degraded semantics.
- Gateway/MCP: status surfaces do not expose a unified plugin catalog or consistent degraded reason per backend.

## File Structure

- Create: `src/runtime/backend_status.py` - backend status value object shared by pluginized modules.
- Test: `src/runtime/tests/test_backend_status.py` - regression tests for status fields and autonomy fallback health.
- Modify: `src/nav/local/local_planner.py` - report nanobind to `cmu_py` fallback through shared status.
- Modify: `src/nav/local/terrain_module.py` - report simple/native/nanobind status through shared status.
- Modify: `src/nav/local/path_follower_module.py` - report `nav_kernel` to `pid` fallback through shared status.
- Later modify: `src/perception/semantic_perception/api/factory.py` - registry-first detector/encoder/tracker factories.
- Later modify: `src/decision/llm/client.py` - registry-first LLM client factory.
- Later modify: `src/memory/modules/vector_memory_module.py` - align encoder fallback health with `BackendStatus`.
- Later modify: `src/gateway/gateway_module.py` and `src/gateway/routes/diagnostics.py` - expose plugin catalog and degraded status.

---

### Task 1: Shared Backend Status Contract

**Files:**
- Create: `src/runtime/backend_status.py`
- Create: `src/runtime/tests/test_backend_status.py`
- Modify: `src/nav/local/local_planner.py`
- Modify: `src/nav/local/terrain_module.py`
- Modify: `src/nav/local/path_follower_module.py`

- [x] **Step 1: Write failing tests for shared status and autonomy fallback**

Run: `python -m pytest src/runtime/tests/test_backend_status.py -q`

Expected before implementation: FAIL with `ModuleNotFoundError: No module named 'runtime.backend_status'`.

- [x] **Step 2: Add `BackendStatus`**

Required behavior:

```python
status = BackendStatus.configured_as("nanobind")
status.as_health_fields()
# {
#   "configured_backend": "nanobind",
#   "backend": "nanobind",
#   "degraded": False,
#   "degraded_reason": "",
# }

status.use("cmu_py", reason="compatible _nav_kernel missing")
status.as_health_fields()["degraded"] is True
```

- [x] **Step 3: Migrate autonomy modules without changing output behavior**

Required health shape for each module:

```python
{
    "configured_backend": "<requested backend>",
    "backend": "<effective backend>",
    "degraded": bool,
    "degraded_reason": "<empty or concrete reason>",
}
```

- [x] **Step 4: Verify the slice**

Run:

```bash
python -m pytest src/runtime/tests/test_backend_status.py -q
python -m pytest src/runtime/tests/test_simplification_wave1.py::TestW1LocalPlannerNoFallback src/runtime/tests/test_simplification_wave1.py::TestW1TerrainNoFallback -q
python -m py_compile src/runtime/backend_status.py src/nav/local/local_planner.py src/nav/local/terrain_module.py src/nav/local/path_follower_module.py
```

Expected: all pass.

---

### Task 2: Autonomy Registry Coverage

**Files:**
- Modify: `src/nav/local/local_planner.py`
- Modify: `src/nav/local/path_follower_module.py`
- Modify: `src/nav/local/terrain_module.py`
- Test: `src/runtime/tests/test_backend_status.py` or `src/runtime/tests/test_registry.py`

- [x] **Step 1: Add registry coverage assertions**

Required assertions:

```python
from runtime.registry import list_plugins

def test_autonomy_backend_registry_names_are_visible():
    assert {"nanobind", "native", "simple"} <= set(list_plugins("terrain"))
    assert "nav_kernel" in set(list_plugins("path_follower"))
    assert "cmu" in set(list_plugins("local_planner"))
```

- [x] **Step 2: Register compatibility aliases without changing defaults**

Target aliases:

```text
local_planner: cmu, nanobind, cmu_py, simple
path_follower: nav_kernel, pure_pursuit, pid
terrain: nanobind, native, simple
```

- [x] **Step 3: Fail fast on unknown autonomy backend names**

The error should name the invalid backend and list available registered names.

- [x] **Step 4: Verify**

Run:

```bash
python -m pytest src/runtime/tests/test_backend_status.py src/runtime/tests/test_registry.py -q
python -m pytest src/runtime/tests/test_non_native_navigation_blueprint.py src/runtime/tests/test_terrain_local_planner_contract.py -q
```

---

### Task 3: Perception Provider Registry

**Files:**
- Modify: `src/perception/semantic_perception/api/factory.py`
- Modify: `src/perception/semantic_perception/perception_module.py`
- Test: add focused tests under `src/runtime/tests/` or the semantic test package.

- [x] **Step 1: Register detector providers**

Target detector plugin names:

```text
yoloe, yolo_world, bpu, sim_scene
```

- [x] **Step 2: Register encoder providers**

Target encoder plugin names:

```text
clip, mobileclip
```

- [x] **Step 3: Register tracker providers**

Target tracker plugin names:

```text
bpu, instance
```

- [x] **Step 4: Preserve current defaults**

`perception(detector="yoloe", encoder="mobileclip")` must still instantiate the same default components when dependencies are available.

- [x] **Step 5: Verify**

Run:

```bash
python -m pytest src/runtime/tests/test_perception_factory_registry.py src/runtime/tests/test_perception_module.py::TestDetectorConfiguration -q
```

---

### Task 4: LLM And Goal Resolver Plugin Alignment

**Files:**
- Modify: `src/decision/llm/client.py`
- Modify: `src/decision/modules/llm.py`
- Modify: `src/decision/modules/semantic_planner.py`
- Modify: `src/runtime/blueprints/stacks/planner.py`
- Test: semantic planner LLM/client tests.

- [x] **Step 1: Register LLM client providers**

Target LLM plugin names:

```text
mock, kimi, moonshot, openai, claude, anthropic, qwen, dashscope
```

- [x] **Step 2: Keep alias behavior stable**

Existing aliases in `_BACKEND_ALIASES` must continue resolving to the same client classes.

- [x] **Step 3: Pass selected planner backend into `GoalResolver`**

`planner(llm="qwen")` should make the semantic slow path use the same selected backend unless explicitly overridden.

- [x] **Step 4: Verify**

Run:

```bash
python -m pytest src/runtime/tests/test_llm_client_registry.py src/decision/tests/test_planner_node_init.py::TestSemanticPlannerInit -q
```

---

### Task 5: Memory, Reconstruction, SLAM, Exploration, Gateway

**Files:**
- Modify: `src/memory/modules/vector_memory_module.py`
- Modify: `src/perception/reconstruction/server/backends/registry.py` only if adapter metadata is missing.
- Modify: `src/runtime/blueprints/stacks/slam.py`
- Modify: `src/runtime/blueprints/stacks/exploration.py`
- Modify: `src/gateway/routes/diagnostics.py`

- [x] **Step 1a: Align VectorMemory encoder health fields**

`VectorMemoryModule.health()` and `get_memory_stats()` now expose nested
`encoder_backend` status without changing the existing storage `backend`
field.

- [x] **Step 1b: Align remaining SLAM/exploration/reconstruction health fields**

Every replaceable backend health payload must include:

```text
configured_backend, backend, degraded, degraded_reason
```

- [x] **Step 2: Expose plugin catalog read-only**

Diagnostics should expose categories from `runtime.registry.list_categories()` and plugin names from `list_plugins(category)` without allowing runtime mutation.

- [x] **Step 3: Verify remaining Task 5 surfaces**

Verified with:

```bash
python -m pytest src/memory/tests/test_memory_modules.py::TestVectorMemoryOperations -q
python -m pytest src/gateway/tests/test_gateway_runtime_status.py::test_diagnostics_plugin_catalog_exposes_registered_backends src/gateway/tests/test_gateway_runtime_status.py::test_diagnostics_plugin_catalog_route src/gateway/tests/test_gateway_runtime_status.py::test_diagnostics_plugin_catalog_route_exposes_active_backend_status -q
python -m pytest src/gateway/tests/test_gateway_route_split.py::test_gateway_module_builds_split_routes_once src/gateway/tests/test_gateway_route_split.py::test_gateway_module_keeps_client_route_inventory -q
python -m pytest src/localization/tests/test_slam_backend_status.py src/localization/tests/test_slam_bridge_tf.py src/localization/tests/test_slam_stack_services.py -q
python -m pytest src/runtime/tests/test_tare_exploration.py src/runtime/tests/test_reconstruction_backend_status.py -q
python -m pytest src/runtime/tests/test_backend_status.py src/runtime/tests/test_perception_factory_registry.py src/runtime/tests/test_llm_client_registry.py src/gateway/tests/test_gateway_runtime_status.py::test_diagnostics_plugin_catalog_exposes_registered_backends src/gateway/tests/test_gateway_runtime_status.py::test_diagnostics_plugin_catalog_route src/gateway/tests/test_gateway_runtime_status.py::test_diagnostics_plugin_catalog_route_exposes_active_backend_status src/localization/tests/test_slam_backend_status.py src/runtime/tests/test_reconstruction_backend_status.py -q
python -m py_compile src/runtime/backend_status.py src/gateway/routes/diagnostics.py src/localization/slam_module.py src/localization/bridge.py src/nav/exploration/tare/module.py src/nav/exploration/tare/supervisor.py src/runtime/blueprints/stacks/exploration.py src/perception/reconstruction/server/recon_server.py
```

---

## Stop Condition

Stop only when:

- All replaceable backends have a visible registry or documented compatibility shim.
- Every backend health payload reports configured/effective/degraded status.
- Unknown backend names fail fast before robot motion.
- Safety/cmd_vel wiring remains non-bypassable through `CmdVelMux`.
- Targeted tests pass for each migrated surface.
