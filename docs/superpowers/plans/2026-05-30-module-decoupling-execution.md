# Module Decoupling Execution Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Tighten LingTu's Module-First boundaries by moving shared contracts to `core`, adding import-boundary regression tests, and then iterating on the remaining module leaks with fresh verification.

**Architecture:** The current architecture is mostly healthy: Modules, Blueprints, typed ports, factories, and registries already provide the right decoupling surface. The immediate repair is to move cross-layer shared map artifact logic out of `nav` into `core`, then enforce that `gateway` cannot import `nav`, `semantic`, or `drivers` directly. Larger follow-up tasks extract optional ROS 2 behavior from normal Modules into bridge/native-module surfaces and harden server-side evidence gates.

**Tech Stack:** Python 3.10, pytest, AST-based static tests, existing LingTu Module-First framework.

---

## Algorithm Efficiency Gates

Decoupling is not a substitute for algorithm performance. Treat algorithm efficiency as a separate acceptance lane with these measurable gates:

- Global planning: record selected backend, fallback state, planning latency, path length, path safety score, and map artifact gate status for A* and PCT.
- Local planning and following: record local planner backend, path follower backend, command publication rate, control-loop latency, and whether direct-track or PID fallback was used.
- Mapping and artifact generation: record point count, map bounds, DUFOMap elapsed time, tomogram cell count, tomogram build latency, and memory-safe limits.
- Runtime constraints: keep server-side benchmark outputs tagged with platform assumptions, especially S100P/aarch64/no-CUDA constraints.
- Stop condition: do not claim an algorithm is faster, safer, or production-ready unless the benchmark artifact is fresh, structured, and identifies fallback paths explicitly.

## File Structure

- Create `src/core/tests/test_module_boundaries.py`: AST-based regression tests for import boundaries.
- Create `src/core/same_source_map_artifacts.py`: canonical shared saved-map artifact contract used by `core`, `nav`, and `gateway`.
- Create `src/core/dynamic_filter.py`: canonical DUFOMap save-time filtering helper used by map save paths.
- Create `src/core/yaml_helpers.py`: canonical YAML/JSON persistence helper used by diagnostics and nav services.
- Modify `src/nav/services/same_source_map_artifacts.py`: keep a backward-compatible shim for existing imports.
- Modify `src/nav/services/dynamic_filter.py`: keep a backward-compatible shim for existing imports and monkeypatch paths.
- Modify `src/nav/services/yaml_helpers.py`: keep a backward-compatible shim for existing imports.
- Modify `src/nav/global_planner_service.py`: import the shared artifact contract from `core`.
- Modify `src/nav/services/map_manager_module.py`: import the shared artifact contract from `core`.
- Modify `src/gateway/routes/maps.py`: remove direct `gateway -> nav` import.
- Modify `src/gateway/routes/session.py`: remove direct `gateway -> nav` import.
- Modify `src/gateway/routes/diagnostics.py`: remove lazy direct `gateway -> nav` import.
- Modify `src/gateway/services/map_safety.py`: consume the dynamic filter helper from `core`.
- Modify `src/gateway/gateway_module.py`: remove stale lazy `gateway -> nav` dynamic-filter import.
- Modify `src/core/product_field_check.py`: avoid `core -> nav` dependency for saved-map artifact validation.
- Modify `src/nav/services/map_manager_module.py`: avoid `nav -> gateway` dynamic-filter helper import.
- Modify `src/nav/services/task_scheduler_module.py`: import YAML helpers from `core`.
- Modify `src/nav/services/patrol_manager_module.py`: import YAML helpers from `core`.
- Modify `src/nav/services/geofence_manager_module.py`: import YAML helpers from `core`.

## Task 1: Record The Boundary Guard Before Fixing It

**Files:**
- Create: `src/core/tests/test_module_boundaries.py`

- [x] **Step 1: Write the failing import-boundary test**

Add this test file:

```python
from __future__ import annotations

import ast
from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]
SRC = ROOT / "src"


def _python_files(package: str) -> list[Path]:
    return sorted(
        path
        for path in (SRC / package).rglob("*.py")
        if "__pycache__" not in path.parts
    )


def _imported_modules(tree: ast.AST) -> list[str]:
    modules: list[str] = []
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            modules.extend(alias.name for alias in node.names)
        elif isinstance(node, ast.ImportFrom) and node.module:
            modules.append(node.module)
    return modules


def _top_level(module: str) -> str:
    return module.split(".", 1)[0]


def test_gateway_package_does_not_import_domain_layers_directly() -> None:
    forbidden = {"nav", "semantic", "drivers"}
    violations: list[str] = []

    for path in _python_files("gateway"):
        rel = path.relative_to(ROOT).as_posix()
        tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
        for module in _imported_modules(tree):
            if _top_level(module) in forbidden:
                violations.append(f"{rel}: imports {module}")

    assert violations == []
```

- [x] **Step 2: Verify the test fails for the existing leak**

Run:

```bash
python -m pytest src/core/tests/test_module_boundaries.py -q
```

Observed: FAIL showing six direct `gateway -> nav` imports across saved-map artifacts, dynamic filtering, and YAML helpers.

## Task 2: Move Saved-Map Artifact Contract To Core

**Files:**
- Create: `src/core/same_source_map_artifacts.py`
- Create: `src/core/dynamic_filter.py`
- Create: `src/core/yaml_helpers.py`
- Modify: `src/nav/services/same_source_map_artifacts.py`
- Modify: `src/nav/services/dynamic_filter.py`
- Modify: `src/nav/services/yaml_helpers.py`
- Modify: `src/nav/global_planner_service.py`
- Modify: `src/nav/services/map_manager_module.py`
- Modify: `src/nav/services/task_scheduler_module.py`
- Modify: `src/nav/services/patrol_manager_module.py`
- Modify: `src/nav/services/geofence_manager_module.py`
- Modify: `src/gateway/routes/maps.py`
- Modify: `src/gateway/routes/session.py`
- Modify: `src/gateway/routes/diagnostics.py`
- Modify: `src/gateway/services/map_safety.py`
- Modify: `src/gateway/gateway_module.py`
- Modify: `src/core/product_field_check.py`

- [x] **Step 1: Copy the canonical implementation to core**

Copy the current contents of these nav-local shared helpers to core without changing behavior:

```text
src/nav/services/same_source_map_artifacts.py -> src/core/same_source_map_artifacts.py
src/nav/services/dynamic_filter.py -> src/core/dynamic_filter.py
src/nav/services/yaml_helpers.py -> src/core/yaml_helpers.py
```

- [x] **Step 2: Replace the old nav modules with compatibility shims**

Use these implementations:

```python
"""Compatibility shim for saved-map artifact helpers.

The canonical implementation lives in core.same_source_map_artifacts so shared
contracts can be used by gateway, nav, and core without cross-layer imports.
"""

import sys as _sys

from core import same_source_map_artifacts as _impl

_sys.modules[__name__] = _impl
```

```python
"""Compatibility shim for saved-map dynamic filtering helpers."""

import sys as _sys

from core import dynamic_filter as _impl

_sys.modules[__name__] = _impl
```

```python
"""Compatibility shim for shared YAML/JSON persistence helpers."""

import sys as _sys

from core import yaml_helpers as _impl

_sys.modules[__name__] = _impl
```

- [x] **Step 3: Update production imports to the new core path**

Replace imports of:

```python
from nav.services.nav_services.same_source_map_artifacts import ...
```

with:

```python
from core.same_source_map_artifacts import ...
```

in production code. Also replace `gateway -> nav` dynamic-filter and YAML helper imports with `core.dynamic_filter` and `core.yaml_helpers`. The old import paths remain available for compatibility tests and downstream code.

- [x] **Step 4: Verify the boundary test passes**

Run:

```bash
python -m pytest src/core/tests/test_module_boundaries.py -q
```

Observed: PASS.

## Task 3: Verify Saved-Map Behavior Stayed Compatible

**Files:**
- Test: `src/core/tests/test_saved_map_artifact_gate.py`
- Test: `src/core/tests/test_mujoco_mid360_pattern.py`

- [x] **Step 1: Run focused artifact tests**

Run:

```bash
python -m pytest src/core/tests/test_saved_map_artifact_gate.py src/core/tests/test_mujoco_mid360_pattern.py::test_fastlio_live_gate_writes_same_source_map_artifacts -q
```

Observed: PASS as part of the focused 63-test command.

- [x] **Step 2: Run a focused nav-service compatibility check**

Run:

```bash
python -m pytest src/core/tests/test_nav_services.py -q
```

Observed: PASS.

## Task 4: Extract Navigation ROS 2 Publishing Into A Bridge Surface

**Files:**
- Modify: `src/nav/navigation_module.py`
- Create or modify: a bridge/native module under the existing ROS 2 bridge pattern
- Test: `src/core/tests/test_module_boundaries.py`

- [ ] **Step 1: Extend the boundary test with a normal-Module ROS 2 import rule**

Add an AST test that flags direct `rclpy` imports and ROS 2 publisher creation in normal Modules, with explicit allowlist entries for bridge/native/simulation adapter modules.

- [ ] **Step 2: Verify the test fails on `NavigationModule`**

Run:

```bash
python -m pytest src/core/tests/test_module_boundaries.py -q
```

Expected: FAIL for `src/nav/navigation_module.py`.

- [ ] **Step 3: Move optional waypoint publishing behind a bridge/native adapter**

Keep `NavigationModule` publishing typed Module outputs. Let the bridge/native adapter handle ROS 2 topics and lifecycle.

- [ ] **Step 4: Verify boundary and navigation tests**

Run:

```bash
python -m pytest src/core/tests/test_module_boundaries.py src/core/tests/test_nav_services.py -q
```

Expected: PASS.

## Task 5: Promote Server-Side Simulation Evidence Into A Freshness Gate

**Files:**
- Modify: `sim/scripts/server_sim_closure.py`
- Modify or create: scenario manifest under the existing sim test/artifact area
- Test: focused server simulation closure command

- [ ] **Step 1: Define required artifact claims**

Require scenario output to record `simulation_only=true`, `real_robot_motion=false`, and `cmd_vel_sent_to_hardware=false`.

- [ ] **Step 2: Add missing scenario entries**

Add or wire scenario entries for `moving_obstacle_sweep` and `large_loop_closure`.

- [ ] **Step 3: Run the closure gate**

Run the existing server simulation closure command and inspect generated JSON for scenario freshness and required claims.

Expected: command exits 0 and generated evidence is fresh.

## Task 6: Replace Fake Planner Benchmark With Real Structured Evidence

**Files:**
- Modify: `tests/benchmark/benchmark_planner.sh`
- Create or modify: structured benchmark output under existing test artifact conventions

- [ ] **Step 1: Replace sleep-only checks with real planner invocation**

Run A* and PCT on a fixed map fixture and emit structured JSON with:

```json
{
  "schema_version": 1,
  "planner": "pct",
  "selected_backend": "pct",
  "fallback_used": false,
  "direct_goal_fallback_used": false,
  "latency_ms": 0.0,
  "path_length_m": 0.0,
  "waypoint_count": 0,
  "path_safety_score": 0.0,
  "map_artifact_gate_ok": true,
  "platform_assumption": "server-side benchmark; S100P aarch64 budget tracked separately"
}
```

- [ ] **Step 2: Verify benchmark output contract**

Run:

```bash
tests/benchmark/benchmark_planner.sh
```

Expected: exits 0 and reports real planner metrics, not static sleeps.

## Task 7: Add Navigation Chain Efficiency Evidence

**Files:**
- Modify or create: existing navigation simulation artifact writer under `sim/scripts/` or `src/core/tests/`
- Test: focused navigation-chain simulation or artifact unit test

- [ ] **Step 1: Add chain-level algorithm telemetry**

Record these fields in navigation-chain evidence:

```json
{
  "global_planner": {
    "selected_planner": "pct",
    "fallback_used": false,
    "direct_goal_fallback": {"used": false},
    "latency_ms": 0.0,
    "path_length_m": 0.0
  },
  "local_planner": {
    "backend": "nav_core",
    "latency_ms": 0.0,
    "trajectory_count": 0
  },
  "path_follower": {
    "backend": "nav_core",
    "direct_track_fallback": false,
    "cmd_rate_hz": 0.0
  },
  "cmd_vel_mux": {
    "active_source": "path_follower"
  }
}
```

- [ ] **Step 2: Verify no fallback hides poor algorithm performance**

Run the focused simulation or artifact test and assert:

```text
selected_planner=pct
fallback_used=false
direct_goal_fallback.used=false
local_planner.backend=nav_core
path_follower.backend=nav_core
direct_track_fallback=false
cmd_vel_mux.active_source=path_follower
```

Expected: PASS with a fresh structured artifact.

## Self-Review

- Spec coverage: The plan covers the multi-agent findings: import-boundary leaks, navigation ROS 2 leakage, simulation evidence freshness, and benchmark credibility.
- Placeholder scan: No task uses `TBD`, `TODO`, or open-ended placeholder instructions.
- Type consistency: The new shared module name is consistently `core.same_source_map_artifacts`, with the old `nav.services.nav_services.same_source_map_artifacts` retained only as a compatibility shim.
