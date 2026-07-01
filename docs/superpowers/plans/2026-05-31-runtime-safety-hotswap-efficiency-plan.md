# LingTu Runtime Safety, Hot-Swap, and Efficiency Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Close the next LingTu readiness gap by making safety stops fail-closed, making backend selection observable and switchable, and making algorithm efficiency claims depend on fresh structured evidence.

**Architecture:** Keep Module-First boundaries. Safety and auth fixes land before runtime switching. Runtime backend changes use a two-stage model: restart-time configuration coverage for all backends first, then controlled in-process switching only for low-risk providers while motion backends switch only when the navigation state is idle or explicitly stopped.

**Tech Stack:** Python 3.10, LingTu Module/Blueprint framework, FastAPI/ASGI middleware, pytest, existing `runtime.registry`, existing `runtime.backend_status`, existing Dimos/server simulation gates, no new dependencies.

---

## Requirements Summary

- P0 safety: Geofence hard stop must reach the driver and NavigationModule in the full-stack graph.
- P0 security: MCP on port 8090 must not expose motion tools on `0.0.0.0` without a configured key in robot or non-dev operation.
- Safety watchdogs must be timer-driven, not only callback-driven.
- Teleop release must not silently restart a mission after manual intervention.
- Recovery motion post-actions must run after recovery finishes, not while recovery cmd_vel is still publishing.
- Backend selection must be modular across perception, planner, local planner, path follower, terrain, SLAM, LLM, reconstruction, exploration, and gateway diagnostics.
- "Change local detection quickly" must mean a supported backend selection path first, then a controlled runtime switch path with drain, instantiate, health check, and rollback.
- Algorithm performance claims must be tied to fresh, structured, non-synthetic evidence with explicit platform and claim scope.
- Module decoupling must be enforced by tests for `gateway`, `nav`, `semantic`, and `drivers`, not only by documentation.

## Current Evidence

- Multi-agent safety review found several earlier critical issues already repaired: SafetyRing now republishes current stop level, startup odometry defaults fail-closed, CmdVelMux sanitizes non-finite velocity and publishes zero on timeout, and drift watchdog detects non-finite odometry.
- Remaining P0 safety risk: `GeofenceManagerModule.stop_cmd` exists, but `src/runtime/blueprints/full_stack_wiring.py` only wires `SafetyRingModule.stop_cmd` and `GatewayModule.stop_cmd` to driver/navigation stop inputs.
- Remaining P0 security risk: `MCPServerModule` adds `APIKeyMiddleware`, but `APIKeyMiddleware` passes all requests when no key is configured.
- Hot-swap research found current backend selection is mostly constructor-time: CLI/profile strings flow into stacks and modules, while runtime reconfiguration is not a general contract.
- Performance research found Dimos gates exist, but current local artifacts are stale or missing for the strict `dimos_benchmark` preset; synthetic benchmarks must not be treated as S100P field performance.
- Boundary research found `src/runtime/tests/test_module_boundaries.py` currently covers `gateway` only; `nav`, `semantic`, and `drivers` need equivalent AST guards.
- External references reviewed:
  - ROS 2 Nav2 uses explicit plugin categories for planner, controller, behavior, BT, costmap, and navigator plugins.
  - Quad-SDK separates planning, control, estimation, communication, and simulation/hardware integration.
  - ANYbotics elevation mapping exposes explicit submap/reset/update-rate surfaces for rough-terrain map pipelines.
  - CMU exploration/TARE architecture treats local planner, terrain traversability, and waypoint following as separate navigation modules.

## File Responsibility Map

- `src/runtime/blueprints/full_stack_wiring.py`: Owns explicit safety-critical full-stack wires.
- `src/runtime/tests/test_profile_graph_snapshots.py`: Locks explicit profile graph edges.
- `src/nav/services/geofence.py`: Owns geofence intrusion detection and stop publication.
- `src/gateway/auth.py`: Owns API-key middleware behavior.
- `src/gateway/mcp_server.py`: Owns MCP server binding and middleware configuration.
- `src/nav/services/safety/safety_ring_module.py`: Owns L0 safety state and stop publication.
- `src/nav/services/safety/cmd_vel_mux_module.py`: Owns velocity source arbitration and finite velocity filtering.
- `src/nav/mission/navigation_module.py`: Owns mission FSM, teleop pause/resume, recovery motion, and planner service calls.
- `src/gateway/gateway_module.py`: Owns odometry publication and drift watchdog input path.
- `src/runtime/backend_status.py`: Owns backend status payload conventions.
- `src/perception/semantic_perception/perception_module.py`: Owns detector/encoder provider lifecycle.
- `src/nav/services/plan/global_planner/service.py`: Owns planner/fallback provider selection.
- `cli/main.py`: Owns profile and CLI backend override ingestion.
- `cli/repl.py`: Owns operator commands and map/tomogram reload path.
- `src/gateway/routes/diagnostics.py`: Owns algorithm benchmark and active backend diagnostics.
- `sim/scripts/server_sim_closure.py`: Owns strict Dimos gate sequence and freshness evaluation.
- `tests/benchmark/benchmark_planner.sh`: Owns shell benchmark status reporting.
- `tests/benchmark/benchmark_planner_structured.py`: Owns structured planner benchmark metadata.
- `src/runtime/tests/test_module_boundaries.py`: Owns static import-boundary regression tests.

## Acceptance Criteria

- Full-stack graph snapshots include geofence hard stop edges to driver stop and NavigationModule stop when GeofenceManagerModule is present.
- MCP without an API key is fail-closed for robot/non-dev exposure, or it binds only to localhost with motion tools unavailable.
- SafetyRing emits hard stop after odometry timeout even when no new input callbacks arrive.
- CmdVelMux shared state updates are lock-protected and existing priority/timeout behavior remains unchanged.
- Teleop release does not auto-plan unless an explicit resume path is invoked.
- Recovery `external_strategy` post-action is performed only after recovery motion completes.
- Gateway quarantines non-finite odometry before SSE/state publication.
- Perception and global planner diagnostics expose configured backend, effective backend, degraded flag, degraded reason, and fallback reason using one schema.
- CLI/profile overrides cover SLAM, exploration, local planner, path follower, terrain, detector, encoder, LLM, global planner, fallback planner, and native mode.
- Runtime switch MVP supports detector/encoder and LLM switching with drain, instantiate, health check, and rollback; motion backends reject runtime switch unless idle.
- Dimos required gates use one source of truth across server simulation and Gateway diagnostics.
- Benchmark artifacts expose platform, hardware, synthetic, freshness, and claim_scope fields.
- Skip-style benchmarks report SKIP separately from PASS and do not satisfy claim gates.
- Boundary tests cover forbidden production imports for `gateway`, `nav`, `semantic`, and `drivers`.

---

### Task 1: P0 Geofence Hard Stop Wiring

**Files:**
- Modify: `src/runtime/blueprints/full_stack_wiring.py`
- Modify: `src/runtime/tests/test_profile_graph_snapshots.py`
- Test: `src/nav/tests/test_nav_services.py`

- [ ] **Step 1: Write graph regression test**

Add assertions to `test_profile_graph_snapshot_locks_safety_gateway_and_mux_edges` after the existing `GatewayModule.stop_cmd` assertions:

```python
        if "GeofenceManagerModule" in modules:
            assert f"GeofenceManagerModule.stop_cmd->{driver}.stop_signal" in wires
            assert "GeofenceManagerModule.stop_cmd->NavigationModule.stop_signal" in wires
```

- [ ] **Step 2: Verify the test fails before implementation**

Run:

```powershell
python -m pytest src/runtime/tests/test_profile_graph_snapshots.py::test_profile_graph_snapshot_locks_safety_gateway_and_mux_edges -q
```

Expected result before code change: FAIL showing the missing `GeofenceManagerModule.stop_cmd` wire.

- [ ] **Step 3: Add explicit geofence stop wires**

In `_apply_required_safety_stop_wires()`, extend the required specs to include geofence when the module is present:

```python
    required = [
        WireSpec("SafetyRingModule", "stop_cmd", driver_module, "stop_signal"),
        WireSpec("SafetyRingModule", "stop_cmd", "NavigationModule", "stop_signal"),
    ]
    if "GeofenceManagerModule" in names:
        required.extend([
            WireSpec("GeofenceManagerModule", "stop_cmd", driver_module, "stop_signal"),
            WireSpec("GeofenceManagerModule", "stop_cmd", "NavigationModule", "stop_signal"),
        ])
    for spec in required:
        _require_wire(bp, names, spec)
```

- [ ] **Step 4: Run focused tests**

Run:

```powershell
python -m pytest src/runtime/tests/test_profile_graph_snapshots.py::test_profile_graph_snapshot_locks_safety_gateway_and_mux_edges src/nav/tests/test_nav_services.py::TestGeofenceManagerModule -q
```

Expected result: PASS.

- [ ] **Step 5: Commit with Lore protocol**

Use a commit message that records the safety invariant:

```text
Make geofence intrusion reach the hard-stop chain

Constraint: Physical robot motion must fail closed on geofence intrusion.
Confidence: high
Scope-risk: narrow
Tested: pytest profile graph safety wires and geofence service tests
Not-tested: real S100P geofence intrusion run
```

### Task 2: P0 MCP Fail-Closed Authentication

**Files:**
- Modify: `src/gateway/auth.py`
- Modify: `src/gateway/mcp_server.py`
- Create or modify: `src/gateway/tests/test_mcp_auth.py`
- Optionally modify: `config/robot_config.yaml`

- [ ] **Step 1: Add middleware tests for fail-closed mode**

Create `src/gateway/tests/test_mcp_auth.py` with ASGI-level tests that do not start a network listener:

```python
import pytest

from gateway.auth import APIKeyMiddleware


async def _ok_app(scope, receive, send):
    await send({"type": "http.response.start", "status": 200, "headers": []})
    await send({"type": "http.response.body", "body": b"ok"})


@pytest.mark.asyncio
async def test_api_key_middleware_rejects_missing_key_when_required(monkeypatch):
    monkeypatch.delenv("LINGTU_API_KEY", raising=False)
    sent = []
    app = APIKeyMiddleware(_ok_app, api_key=None, require_key=True)
    await app(
        {"type": "http", "path": "/mcp", "headers": [], "query_string": b""},
        lambda: None,
        sent.append,
    )
    assert sent[0]["status"] == 401


@pytest.mark.asyncio
async def test_api_key_middleware_accepts_valid_key_when_required(monkeypatch):
    monkeypatch.setenv("LINGTU_API_KEY", "secret")
    sent = []
    app = APIKeyMiddleware(_ok_app, require_key=True)
    await app(
        {
            "type": "http",
            "path": "/mcp",
            "headers": [(b"x-api-key", b"secret")],
            "query_string": b"",
        },
        lambda: None,
        sent.append,
    )
    assert sent[0]["status"] == 200
```

- [ ] **Step 2: Verify the fail-closed test fails**

Run:

```powershell
python -m pytest src/gateway/tests/test_mcp_auth.py -q
```

Expected result before implementation: FAIL because `require_key` is not supported.

- [ ] **Step 3: Implement explicit fail-closed parameter**

Extend `APIKeyMiddleware.__init__`:

```python
    def __init__(self, app, api_key: str | None = None, require_key: bool = False):
        self.app = app
        self._key = api_key or _get_configured_key()
        self._require_key = bool(require_key)
```

Then change the no-key branch:

```python
        if self._key_hash is None:
            if self._require_key:
                return await self._reject(scope, send, 401, "API key required")
            return await self.app(scope, receive, send)
```

- [ ] **Step 4: Make MCP choose fail-closed exposure**

In `MCPServerModule`, add constructor config:

```python
        require_api_key: bool | None = None,
        host: str = "0.0.0.0",
```

Use this rule in `_run_server()`:

```python
        require_key = (
            self._require_api_key
            if self._require_api_key is not None
            else self._host not in {"127.0.0.1", "localhost", "::1"}
        )
        app.add_middleware(APIKeyMiddleware, require_key=require_key)
```

This keeps localhost dev usable while preventing unauthenticated LAN motion access.

- [ ] **Step 5: Run focused gateway tests**

Run:

```powershell
python -m pytest src/gateway/tests/test_mcp_auth.py src/gateway/tests/test_gateway_route_split.py -q
```

Expected result: PASS.

### Task 3: Timer-Driven Safety Watchdog and CmdVelMux Locking

**Files:**
- Modify: `src/nav/services/safety/safety_ring_module.py`
- Modify: `src/nav/services/safety/cmd_vel_mux_module.py`
- Modify: `src/nav/tests/test_nav_modules.py`
- Modify: `src/runtime/tests/test_cmd_vel_mux.py`

- [ ] **Step 1: Add SafetyRing watchdog regression**

Add a test to `TestSafetyRingModule`:

```python
def test_safety_ring_times_out_without_new_callbacks(self):
    mod = SafetyRingModule(odom_timeout_ms=40, cmd_vel_timeout_ms=40)
    stops = []
    mod.stop_cmd._add_callback(stops.append)
    mod.setup()
    mod.start()
    try:
        mod._on_odom(Odometry(x=0.0, y=0.0, yaw=0.0))
        time.sleep(0.09)
        assert 2 in stops
    finally:
        mod.stop()
```

- [ ] **Step 2: Verify SafetyRing watchdog test fails**

Run:

```powershell
python -m pytest src/nav/tests/test_nav_modules.py::TestSafetyRingModule::test_safety_ring_times_out_without_new_callbacks -q
```

Expected result before implementation: FAIL because timeout publication depends on a callback.

- [ ] **Step 3: Implement SafetyRing monitor loop**

Add `threading` and maintain:

```python
        self._monitor_stop = threading.Event()
        self._monitor_thread: threading.Thread | None = None
```

Add start/stop:

```python
    def start(self) -> None:
        super().start()
        self._monitor_stop.clear()
        if self._monitor_thread is None or not self._monitor_thread.is_alive():
            self._monitor_thread = threading.Thread(
                target=self._monitor_loop,
                name="safety_ring_monitor",
                daemon=True,
            )
            self._monitor_thread.start()

    def stop(self) -> None:
        self._monitor_stop.set()
        if self._monitor_thread and self._monitor_thread.is_alive():
            self._monitor_thread.join(timeout=1.0)
        self._monitor_thread = None
        super().stop()

    def _monitor_loop(self) -> None:
        interval = max(0.02, min(self._odom_timeout, self._cmdvel_timeout) / 2.0)
        while not self._monitor_stop.wait(interval):
            self._publish_safety()
```

- [ ] **Step 4: Add CmdVelMux lock without changing behavior**

Wrap `_on_source()`, `_check_timeout()`, `_select_active()`, and health source iteration with one `threading.RLock`. Keep publish calls outside the lock when practical; if publish outside requires a selected action tuple, compute it under lock and publish after.

- [ ] **Step 5: Run safety/mux tests**

Run:

```powershell
python -m pytest src/nav/tests/test_nav_modules.py::TestSafetyRingModule src/runtime/tests/test_cmd_vel_mux.py -q
```

Expected result: PASS.

### Task 4: Teleop Resume Policy and Recovery Completion Ordering

**Files:**
- Modify: `src/nav/mission/navigation_module.py`
- Modify: `src/nav/tests/test_navigation_frame_contract.py`
- Optionally modify: `cli/repl.py`
- Optionally modify: `src/gateway/gateway_module.py`

- [ ] **Step 1: Add teleop release regression**

In a navigation test module, assert release does not auto-plan by default:

```python
def test_teleop_release_does_not_auto_resume_without_explicit_resume(monkeypatch):
    nav = NavigationModule(auto_resume_after_teleop=False)
    nav._state = MissionState.EXECUTING
    nav._goal = np.array([1.0, 2.0, 0.0])
    planned = []
    monkeypatch.setattr(nav, "_plan", lambda: planned.append(True))
    nav._on_teleop_active(True)
    nav._on_teleop_active(False)
    assert planned == []
    assert nav._state == MissionState.IDLE
```

- [ ] **Step 2: Add recovery ordering regression**

Patch `_run_recovery_motion` or use a short strategy to assert `external_strategy` post-action is not called until the recovery thread completes:

```python
def test_external_strategy_recovery_finishes_before_republish(monkeypatch):
    nav = NavigationModule()
    nav._state = MissionState.EXECUTING
    nav._using_external_strategy_path = True
    events = []
    monkeypatch.setattr(nav, "_republish_external_strategy_path", lambda: events.append("republish"))
    nav._execute_recovery_motion(post_action="external_strategy")
    assert events == []
    nav._request_recovery_stop(join_timeout=1.0)
```

Adjust the test to use deterministic zero-duration strategy if needed.

- [ ] **Step 3: Implement explicit teleop resume default**

Add config:

```python
        auto_resume_after_teleop: bool = False,
```

In `_on_teleop_active(False)`, only call `_plan()` when `self._auto_resume_after_teleop` is true. Otherwise clear pending resume state and publish an adapter status event:

```python
            if self._pre_teleop_goal is not None and not self._auto_resume_after_teleop:
                self.adapter_status.publish({
                    "event": "teleop_release_resume_required",
                    "ts": time.time(),
                })
```

- [ ] **Step 4: Fix external recovery post-action**

Replace the stuck branch:

```python
                    self._execute_recovery_motion(post_action="none")
                    self._finish_recovery_motion("external_strategy", ...)
```

with:

```python
                    self._execute_recovery_motion(post_action="external_strategy")
```

- [ ] **Step 5: Run navigation focused tests**

Run:

```powershell
python -m pytest src/nav/tests/test_navigation_frame_contract.py src/runtime/tests/test_cmd_vel_mux.py -q
```

Expected result: PASS.

### Task 5: Gateway Non-Finite Odometry Quarantine

**Files:**
- Modify: `src/gateway/gateway_module.py`
- Modify: `src/gateway/tests/test_gateway_runtime_status.py`
- Modify: `src/gateway/tests/test_gateway_readiness.py`

- [ ] **Step 1: Add quarantine regression**

Add a test that sends NaN odometry and asserts the Gateway does not update public odometry state:

```python
def test_gateway_quarantines_non_finite_odometry_before_publication():
    gw = GatewayModule()
    gw._odom = {"x": 1.0, "y": 2.0, "yaw": 0.0}
    gw._on_odometry(Odometry(x=float("nan"), y=0.0, yaw=0.0))
    assert gw._odom == {"x": 1.0, "y": 2.0, "yaw": 0.0}
    assert gw._drift_last_restart_reason in {"non_finite_odometry", "odom_diverged"}
```

- [ ] **Step 2: Implement finite guard at `_on_odometry()` entry**

Before assigning to `self._odom` or pushing events, validate all numeric pose/twist fields with `math.isfinite`. On failure:

```python
            self._drift_last_restart_reason = "non_finite_odometry"
            self._drift_evidence = {"reason": "non_finite_odometry", "ts": time.time()}
            return
```

Do not publish the malformed odometry event.

- [ ] **Step 3: Run gateway tests**

Run:

```powershell
python -m pytest src/gateway/tests/test_gateway_runtime_status.py src/gateway/tests/test_gateway_readiness.py -q
```

Expected result: PASS.

### Task 6: Backend Status and Restart-Time Configuration Coverage

**Files:**
- Modify: `src/perception/semantic_perception/perception_module.py`
- Modify: `src/nav/services/plan/global_planner/service.py`
- Modify: `src/nav/mission/navigation_module.py`
- Modify: `src/runtime/blueprints/stacks/navigation.py`
- Modify: `cli/main.py`
- Modify: `cli/profiles_data.py`
- Modify: `cli/repl.py`
- Modify: `src/gateway/routes/diagnostics.py`
- Test: `src/runtime/tests/test_backend_status.py`
- Test: `src/runtime/tests/test_perception_factory_registry.py`
- Test: `src/runtime/tests/test_nav_chain_efficiency.py`
- Test: `src/gateway/tests/test_gateway_runtime_status.py`

- [ ] **Step 1: Add status assertions for perception**

Extend perception tests to assert health exposes:

```python
health = module.health()
assert health["detector"]["configured_backend"] == "bpu"
assert health["detector"]["backend"] in {"bpu", "mock", "unavailable"}
assert "degraded" in health["detector"]
assert "degraded_reason" in health["detector"]
assert health["encoder"]["configured_backend"] == "mobileclip"
```

- [ ] **Step 2: Add status assertions for global planner**

In navigation efficiency tests, assert planner service status:

```python
status = nav.health()["planner_backend"]
assert status["configured_backend"] == "pct"
assert "backend" in status
assert "fallback_backend" in status
assert "degraded" in status
```

- [ ] **Step 3: Implement `BackendStatus` in PerceptionModule**

Use existing `runtime.backend_status.BackendStatus`. Maintain separate status objects for detector and encoder. On provider creation success, record effective backend. On fallback or import failure, record degraded reason.

- [ ] **Step 4: Implement planner backend status**

Expose a `GlobalPlannerService.backend_status()` method returning:

```python
{
    "configured_backend": self._planner_name,
    "backend": self._active_planner_name,
    "fallback_backend": self._fallback_planner_name,
    "degraded": bool(self._last_fallback_reason),
    "degraded_reason": self._last_fallback_reason,
}
```

Have `NavigationModule.health()` include it under `planner_backend`.

- [ ] **Step 5: Add CLI/profile backend overrides**

Add parser options:

```python
parser.add_argument("--slam-profile", dest="slam_profile", default=None)
parser.add_argument("--exploration-backend", default=None)
parser.add_argument("--local-planner-backend", default=None)
parser.add_argument("--path-follower-backend", default=None)
parser.add_argument("--terrain-backend", default=None)
```

Map them into `blueprint_cfg` using the existing override pattern. Unknown backend names must fail fast before `system.start()`.

- [ ] **Step 6: Fix REPL map/tomogram reload**

Replace the `nav._planner_backend` lookup with a public method:

```python
if nav and tomogram and os.path.exists(tomogram) and hasattr(nav, "reload_planner_tomogram"):
    nav.reload_planner_tomogram(tomogram)
```

Implement `NavigationModule.reload_planner_tomogram()` by delegating to `GlobalPlannerService.reload_tomogram()`.

- [ ] **Step 7: Run backend diagnostics tests**

Run:

```powershell
python -m pytest src/runtime/tests/test_backend_status.py src/runtime/tests/test_perception_factory_registry.py src/runtime/tests/test_nav_chain_efficiency.py src/gateway/tests/test_gateway_runtime_status.py -q
```

Expected result: PASS.

### Task 7: Runtime Backend Switch MVP

**Files:**
- Modify: `src/runtime/module.py`
- Modify: `src/perception/semantic_perception/perception_module.py`
- Modify: `src/decision/semantic_planner/llm_module.py`
- Modify: `src/gateway/gateway_module.py`
- Modify: `src/gateway/mcp_server.py`
- Create: `src/runtime/tests/test_runtime_backend_switch.py`
- Modify: `src/gateway/tests/test_gateway_runtime_status.py`

- [ ] **Step 1: Add default module reconfigure contract**

Add a conservative default to `Module`:

```python
    def reconfigure_backend(self, category: str, backend: str, **config: Any) -> dict:
        return {
            "ok": False,
            "category": category,
            "requested_backend": backend,
            "reason": "backend_reconfigure_unsupported",
        }
```

- [ ] **Step 2: Add perception runtime switch tests**

Create `src/runtime/tests/test_runtime_backend_switch.py` with:

```python
def test_perception_reconfigure_detector_rejects_unknown_backend():
    mod = PerceptionModule(detector_type="mock", encoder_type="mock")
    result = mod.reconfigure_backend("detector", "missing_backend")
    assert result["ok"] is False
    assert result["reason"] == "unknown_backend"


def test_motion_backend_reconfigure_is_unsupported_by_default():
    mod = Module()
    result = mod.reconfigure_backend("local_planner", "nav_kernel")
    assert result["ok"] is False
```

- [ ] **Step 3: Implement low-risk perception switch**

In PerceptionModule:

1. Pause image processing with a small internal lock.
2. Create the new detector or encoder through the existing registry/factory.
3. Run a cheap health probe.
4. Swap the provider and status only after the probe succeeds.
5. Restore the old provider and return `ok=False` on failure.

Return:

```python
{
    "ok": True,
    "category": "detector",
    "previous_backend": old_backend,
    "backend": new_backend,
    "degraded": False,
}
```

- [ ] **Step 4: Implement LLM runtime switch only when no agent turn is active**

In LLMModule, allow switch if no in-flight generation is running. Reuse existing LLM client factory. Reject unsupported backends with `unknown_backend`.

- [ ] **Step 5: Expose switch through Gateway/MCP with safety guard**

Expose a backend switch route/tool that rejects motion categories unless NavigationModule is idle:

```python
if category in {"planner", "local_planner", "path_follower", "terrain", "slam"}:
    if navigation and navigation.health().get("state") != "IDLE":
        return {"ok": False, "reason": "motion_backend_switch_requires_idle"}
```

- [ ] **Step 6: Run runtime switch tests**

Run:

```powershell
python -m pytest src/runtime/tests/test_runtime_backend_switch.py src/gateway/tests/test_gateway_runtime_status.py -q
```

Expected result: PASS.

### Task 8: Efficiency Schema and Dimos Benchmark Credibility

**Files:**
- Create: `src/runtime/efficiency_status.py`
- Create: `src/runtime/algorithm_gates.py`
- Modify: `src/nav/mission/navigation_module.py`
- Modify: `src/nav/local/local_planner.py`
- Modify: `src/nav/local/path_follower_module.py`
- Modify: `src/nav/services/safety/cmd_vel_mux_module.py`
- Modify: `src/localization/bridge.py`
- Modify: `src/gateway/routes/diagnostics.py`
- Modify: `sim/scripts/server_sim_closure.py`
- Modify: `tests/benchmark/benchmark_planner.sh`
- Modify: `tests/benchmark/benchmark_planner_structured.py`
- Test: `src/runtime/tests/test_nav_chain_efficiency.py`
- Test: `src/runtime/tests/test_server_sim_closure.py`
- Test: `src/gateway/tests/test_gateway_runtime_status.py`

- [ ] **Step 1: Add shared efficiency payload helper**

Create `src/runtime/efficiency_status.py`:

```python
from __future__ import annotations

from typing import Any


def efficiency_payload(
    *,
    latency_ms: float | None = None,
    hz: float | None = None,
    queue_age_ms: float | None = None,
    drop_count: int | None = None,
    fallback: dict[str, Any] | None = None,
    degraded: dict[str, Any] | None = None,
    error: dict[str, Any] | None = None,
    freshness: dict[str, Any] | None = None,
) -> dict[str, Any]:
    return {
        "latency_ms": latency_ms,
        "hz": hz,
        "queue_age_ms": queue_age_ms,
        "drop_count": drop_count,
        "fallback": fallback or {"used": False},
        "degraded": degraded or {"active": False, "reason": ""},
        "error": error or {"last": "", "count": 0},
        "freshness": freshness or {"last_sample_age_ms": None, "stale": None},
    }
```

- [ ] **Step 2: Add module health assertions**

Update `test_nav_chain_efficiency.py` so each relevant module health has:

```python
eff = health["efficiency"]
assert set(eff) >= {
    "latency_ms",
    "hz",
    "queue_age_ms",
    "drop_count",
    "fallback",
    "degraded",
    "error",
    "freshness",
}
```

- [ ] **Step 3: Implement health efficiency in first modules**

Add `health()["efficiency"]` to:

- NavigationModule: last planning latency, fallback status, last error.
- LocalPlannerModule: backend latency, direct-track fallback count/reason.
- PathFollowerModule: command rate, backend/degraded status.
- CmdVelMux: active source age, timeout count, finite-filter drops.
- SlamBridgeModule: odometry/map cloud freshness, worker drops.

- [ ] **Step 4: Unify Dimos gate source**

Create `src/runtime/algorithm_gates.py` with:

```python
DIMOS_BENCHMARK_REQUIRED_GATES = (
    "gateway_runtime_acceptance",
    "routecheck_preflight",
    "large_terrain",
    "native_pct_mujoco",
    "dynamic_obstacle_local_planner",
    "fastlio2_dynamic_inspection",
    "moving_obstacle_sweep",
    "large_loop_closure",
    "gazebo_runtime",
    "saved_map_relocalize",
    "pct_saved_map_navigation",
)
```

Import this constant from both diagnostics and `server_sim_closure.py`.

- [ ] **Step 5: Add anti-drift tests**

In `src/runtime/tests/test_server_sim_closure.py`, add:

```python
def test_dimos_required_gates_match_gateway_diagnostics():
    from runtime.algorithm_gates import DIMOS_BENCHMARK_REQUIRED_GATES
    from gateway.routes.diagnostics import DIMOS_BENCHMARK_REQUIRED_GATES as GATEWAY_GATES

    assert GATEWAY_GATES == DIMOS_BENCHMARK_REQUIRED_GATES
```

- [ ] **Step 6: Add benchmark claim metadata**

In `benchmark_planner_structured.py`, include:

```python
"platform": "server",
"hardware": "unknown",
"synthetic": True,
"claim_scope": "planner_regression_only",
"freshness": {"generated_at": time.time(), "max_age_s": 86400},
```

In shell benchmark summaries, report `PASS`, `FAIL`, and `SKIP` separately. A skipped dependency must not set `ok=True`.

- [ ] **Step 7: Run efficiency and benchmark tests**

Run:

```powershell
python -m pytest src/runtime/tests/test_nav_chain_efficiency.py src/runtime/tests/test_server_sim_closure.py src/gateway/tests/test_gateway_runtime_status.py -q
python tests/benchmark/benchmark_planner_structured.py --json-out artifacts/benchmark_planner/report_structured.json
```

Expected result: tests PASS and structured benchmark emits claim metadata.

### Task 9: Module Boundary Enforcement

**Files:**
- Modify: `src/runtime/tests/test_module_boundaries.py`
- Optionally modify: deprecated blueprint files only if tests expose production violations.

- [ ] **Step 1: Replace single-package boundary test with matrix**

Use an explicit production-only rule table:

```python
BOUNDARY_RULES = {
    "gateway": {"nav", "semantic", "drivers"},
    "nav": {"semantic", "drivers", "gateway"},
    "semantic": {"nav", "drivers", "gateway"},
    "drivers": {"nav", "semantic", "gateway"},
}

ALLOWED_BRIDGE_FILES = {
    "runtime/blueprints/full_stack.py",
    "runtime/blueprints/full_stack_wiring.py",
}
```

- [ ] **Step 2: Ignore tests and intentional bridge surfaces**

Implement helpers:

```python
def _is_test_file(path: Path) -> bool:
    return "tests" in path.parts or path.name.startswith("test_")


def _is_allowed_bridge(rel: str) -> bool:
    return rel in ALLOWED_BRIDGE_FILES or rel.startswith("runtime/blueprints/stacks/")
```

- [ ] **Step 3: Add parametrized AST test**

```python
@pytest.mark.parametrize("package,forbidden", BOUNDARY_RULES.items())
def test_package_does_not_import_forbidden_layers_directly(package, forbidden):
    violations = []
    for path in _python_files(package):
        rel = path.relative_to(ROOT).as_posix()
        if _is_test_file(path) or _is_allowed_bridge(rel):
            continue
        tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
        for module in _imported_modules(tree):
            if _top_level(module) in forbidden:
                violations.append(f"{rel}: imports {module}")
    assert violations == []
```

- [ ] **Step 4: Run boundary tests**

Run:

```powershell
python -m pytest src/runtime/tests/test_module_boundaries.py -q
```

Expected result: PASS, or FAIL with a concrete list of production imports to move behind `core` helpers, registry, or blueprint stack factories.

### Task 10: Whole-Plan Verification and Hardware Gate

**Files:**
- Modify: `docs/superpowers/plans/2026-05-31-runtime-safety-hotswap-efficiency-plan.md` only to check off completed steps.
- Runtime evidence output: `artifacts/benchmark_planner/report_structured.json`
- Runtime evidence output: `artifacts/server_sim_closure_summary.json`
- S100P evidence output: `artifacts/real_s100p_runtime/report.json`

- [ ] **Step 1: Run focused Python safety/security/backend tests**

```powershell
python -m pytest src/runtime/tests/test_profile_graph_snapshots.py::test_profile_graph_snapshot_locks_safety_gateway_and_mux_edges src/nav/tests/test_nav_modules.py::TestSafetyRingModule src/runtime/tests/test_cmd_vel_mux.py src/gateway/tests/test_mcp_auth.py -q
```

- [ ] **Step 2: Run modularity and backend diagnostics tests**

```powershell
python -m pytest src/runtime/tests/test_module_boundaries.py src/runtime/tests/test_backend_status.py src/runtime/tests/test_perception_factory_registry.py src/runtime/tests/test_nav_chain_efficiency.py src/gateway/tests/test_gateway_runtime_status.py -q
```

- [ ] **Step 3: Run benchmark credibility checks**

```powershell
python tests/benchmark/benchmark_planner_structured.py --json-out artifacts/benchmark_planner/report_structured.json
python sim/scripts/server_sim_closure.py --preset dimos_benchmark --max-report-age-s 86400 --strict
```

Expected result: structured benchmark completes. Strict Dimos may fail locally if simulation/runtime artifacts are missing; if it fails, the failure must list missing or stale gates instead of claiming readiness.

- [ ] **Step 4: Run no-ROS framework smoke tests**

```powershell
python -m pytest src/runtime/tests/ -q
```

Expected result: PASS, except known platform-specific tests that require ROS 2/S100P must be explicitly separated rather than silently skipped as pass.

- [ ] **Step 5: Run S100P hardware validation**

On the robot or operator-controlled field setup:

```bash
python lingtu.py nav --robot thunder --dog-host 192.168.66.190 --no-repl
scripts/lingtu health
scripts/lingtu product-field-check --json-out artifacts/real_s100p_runtime/report.json
```

Expected result: hard stop path, auth path, backend diagnostics, freshness, and navigation-chain efficiency are visible in the report. Do not claim physical readiness without this evidence.

## Parallel Agent Staffing

- Agent 1, safety executor: Tasks 1, 3, and 4. Write scope: `src/runtime/blueprints/full_stack_wiring.py`, nav safety/navigation modules, safety tests.
- Agent 2, security executor: Task 2. Write scope: gateway auth/MCP tests and middleware only.
- Agent 3, backend executor: Tasks 6 and 7. Write scope: backend status, CLI config, perception/LLM switch, diagnostics.
- Agent 4, performance executor: Task 8. Write scope: efficiency payloads, Dimos gate source, benchmark metadata.
- Agent 5, boundary verifier: Task 9 plus final cross-suite verification. Write scope: boundary tests and verification notes.

Each agent must report changed files and test commands. Shared files requiring coordination: `src/gateway/routes/diagnostics.py`, `src/nav/mission/navigation_module.py`, and `cli/main.py`.

## Risks and Mitigations

- Risk: Runtime switching on motion backends can destabilize a moving robot. Mitigation: runtime switch rejects motion categories unless NavigationModule is idle; restart-time configuration remains the default for motion.
- Risk: MCP fail-closed can disrupt local dev. Mitigation: localhost stays dev-friendly, LAN exposure requires a key.
- Risk: SafetyRing monitor thread can duplicate stop publications. Mitigation: repeated STOP is intentional for fail-closed safety; tests lock behavior.
- Risk: Dimos strict gate may fail in local dev. Mitigation: failed or stale gates are a valid result; they block claims rather than block code review.
- Risk: AST boundary tests can flag intentional bridges. Mitigation: bridge files are explicitly allowlisted and kept under `runtime/blueprints`.

## Stop Condition

Stop the implementation only when:

- P0 safety and MCP tests pass.
- Backend status and restart-time configuration coverage tests pass.
- Runtime switch MVP tests pass for perception/LLM and rejects unsafe motion switches.
- Efficiency schema appears in module health for the first navigation-chain modules.
- Dimos/Gateway required gate definitions cannot drift.
- Boundary tests cover all four package directions.
- Fresh Dimos and S100P artifacts are either passing or explicitly reported as missing/stale claim blockers.

