from __future__ import annotations

import asyncio
import hashlib
import json

import pytest

pytestmark = [pytest.mark.sim]


pytest.importorskip("fastapi")
from core.tests.numpy_guard import NUMPY_UNSAFE_REASON, numpy_import_is_safe

_NUMPY_IMPORT_SAFE = numpy_import_is_safe()


def _endpoint(gateway, path: str):
    gateway.setup()
    return next(route.endpoint for route in gateway._app.routes if route.path == path)


def _payload(response_or_payload):
    if hasattr(response_or_payload, "body"):
        return json.loads(response_or_payload.body)
    return response_or_payload


def test_algorithm_benchmark_dimos_required_gates_match_core_constant():
    from core.algorithm_gates import DIMOS_BENCHMARK_REQUIRED_GATES
    from gateway.routes.diagnostics import ALGORITHM_PRODUCT_PROFILES

    dimos = ALGORITHM_PRODUCT_PROFILES["dimos_benchmark"]

    assert tuple(dimos["required_gate_sequence"]) == tuple(DIMOS_BENCHMARK_REQUIRED_GATES)
    assert dimos["required_gate_sequence"][0] == "gateway_runtime_acceptance"


def _write_active_same_source_tomogram(map_root):
    active_dir = map_root / "active"
    active_dir.mkdir(parents=True)
    map_path = active_dir / "map.pcd"
    tomogram_path = active_dir / "tomogram.pickle"
    map_path.write_text(
        "\n".join(
            [
                "# .PCD v0.7 - Point Cloud Data file format",
                "VERSION 0.7",
                "FIELDS x y z",
                "SIZE 4 4 4",
                "TYPE F F F",
                "COUNT 1 1 1",
                "WIDTH 1",
                "HEIGHT 1",
                "VIEWPOINT 0 0 0 1 0 0 0",
                "POINTS 1",
                "DATA ascii",
                "0.0 0.0 0.0",
            ]
        )
        + "\n",
        encoding="ascii",
    )
    tomogram_path.write_bytes(b"lingtu-test-tomogram")
    map_sha = hashlib.sha256(map_path.read_bytes()).hexdigest()
    tomogram_sha = hashlib.sha256(tomogram_path.read_bytes()).hexdigest()
    (active_dir / "metadata.json").write_text(
        json.dumps(
            {
                "schema_version": "lingtu.saved_map_artifacts.v1",
                "source_profile": "thunder_field",
                "data_source": "thunder_field",
                "slam_source": "fastlio2",
                "localization_source": "fastlio2",
                "mapping_source": "fastlio2",
                "frame_id": "map",
                "created_at": "2026-05-25T00:00:00Z",
                "artifacts": {
                    "map_pcd": {
                        "path": "map.pcd",
                        "sha256": map_sha,
                        "source_profile": "thunder_field",
                        "data_source": "thunder_field",
                        "slam_source": "fastlio2",
                        "frame_id": "map",
                        "point_count": 1,
                    },
                    "tomogram": {
                        "path": "tomogram.pickle",
                        "sha256": tomogram_sha,
                        "source_map_sha256": map_sha,
                        "source_profile": "thunder_field",
                        "data_source": "thunder_field",
                        "frame_id": "map",
                        "shape": [1, 1, 1],
                    },
                },
            },
            sort_keys=True,
        ),
        encoding="utf-8",
    )
    return active_dir


def test_diagnostics_plugin_catalog_exposes_registered_backends():
    from base_autonomy.modules.local_planner_module import LocalPlannerModule  # noqa: F401
    from base_autonomy.modules.path_follower_module import PathFollowerModule  # noqa: F401
    from gateway.routes.diagnostics import build_plugin_catalog, clear_diagnostics_cache
    from semantic.planner.llm_client import MockLLMClient  # noqa: F401
    # ^ Cross-layer: gateway test imports from semantic/ for registry setup.
    #   Acceptable: test needs to register a semantic backend in the global
    #   registry so build_plugin_catalog() can detect it.  No production
    #   dependency â€?the import is for side-effect registration only.

    clear_diagnostics_cache()

    payload = build_plugin_catalog()

    assert payload["schema_version"] == 1
    categories = payload["categories"]
    assert "local_planner" in categories
    assert {entry["name"] for entry in categories["local_planner"]} >= {
        "nanobind",
        "cmu",
        "cmu_py",
        "simple",
    }
    assert "llm_client" in categories
    assert "mock" in {entry["name"] for entry in categories["llm_client"]}


def test_diagnostics_plugin_catalog_route():
    from gateway.gateway_module import GatewayModule
    from gateway.routes.diagnostics import clear_diagnostics_cache

    clear_diagnostics_cache()

    gateway = GatewayModule()
    payload = asyncio.run(_endpoint(gateway, "/api/v1/diagnostics/plugins")())

    assert payload["schema_version"] == 1
    assert "gateway" in payload["categories"]


def test_diagnostics_plugin_catalog_route_exposes_active_backend_status():
    from gateway.gateway_module import GatewayModule
    from gateway.routes.diagnostics import clear_diagnostics_cache

    clear_diagnostics_cache()

    class FakeLocalPlanner:
        def health(self):
            return {
                "local_planner": {
                    "configured_backend": "nanobind",
                    "backend": "cmu_py",
                    "degraded": True,
                    "degraded_reason": "compatible _nav_core missing",
                }
            }

    class FakeVectorMemory:
        def health(self):
            return {
                "backend": "numpy",
                "encoder_backend": {
                    "configured_backend": "auto",
                    "backend": "lexical_hash",
                    "degraded": True,
                    "degraded_reason": "no semantic text encoder available",
                },
            }

    class FakeNavigation:
        def health(self):
            return {
                "planner_backend": {
                    "configured_backend": "pct",
                    "backend": "astar",
                    "fallback_backend": "astar",
                    "reconfigurable": True,
                    "capabilities": ["global_planning", "saved_map_navigation"],
                    "readiness": {"pct_native": False, "fallback_ready": True},
                    "degraded": True,
                    "degraded_reason": "pct path_safety failed",
                }
            }

    class BrokenModule:
        def health(self):
            raise RuntimeError("health failed")

    gateway = GatewayModule()
    gateway.on_system_modules(
        {
            "LocalPlannerModule": FakeLocalPlanner(),
            "VectorMemoryModule": FakeVectorMemory(),
            "NavigationModule": FakeNavigation(),
            "BrokenModule": BrokenModule(),
        }
    )

    payload = asyncio.run(_endpoint(gateway, "/api/v1/diagnostics/plugins")())

    active = payload["active"]
    local = active["modules"]["LocalPlannerModule"]["backends"]["local_planner"]
    vector = active["modules"]["VectorMemoryModule"]["backends"]["encoder_backend"]
    planner = active["modules"]["NavigationModule"]["backends"]["planner_backend"]
    broken = active["modules"]["BrokenModule"]
    assert active["schema_version"] == 1
    assert local["configured_backend"] == "nanobind"
    assert local["backend"] == "cmu_py"
    assert local["degraded"] is True
    assert "compatible _nav_core missing" in local["degraded_reason"]
    assert vector["configured_backend"] == "auto"
    assert vector["backend"] == "lexical_hash"
    assert planner["configured_backend"] == "pct"
    assert planner["backend"] == "astar"
    assert planner["fallback_backend"] == "astar"
    assert planner["reconfigurable"] is True
    assert planner["capabilities"] == ["global_planning", "saved_map_navigation"]
    assert planner["readiness"] == {"pct_native": False, "fallback_ready": True}
    assert planner["degraded"] is True
    assert broken["error"] == "health failed"


def test_gateway_runtime_backend_switch_rejects_motion_backend_when_navigation_busy():
    from gateway.gateway_module import GatewayModule

    class BusyNavigation:
        def health(self):
            return {"state": "EXECUTING"}

    gateway = GatewayModule()
    gateway.on_system_modules({"NavigationModule": BusyNavigation()})

    result = gateway.reconfigure_backend("local_planner", "nav_core")

    assert result["ok"] is False
    assert result["reason"] == "motion_backend_switch_requires_idle"


def test_gateway_on_system_modules_preserves_read_only_status_inventory():
    from gateway.gateway_module import GatewayModule

    class Navigation:
        def health(self):
            return {"state": "IDLE"}

    class Mux:
        def health(self):
            return {"active_source": "path_follower", "sources": {"path_follower": {}}}

    class Relocalization:
        def trigger_global_relocalize(self, *, timeout_s: float = 10.0):
            return None

        def relocalize_saved_map(self, pcd_path, x, y, yaw, *, timeout_s: float = 30.0):
            return None

        def relocalize_saved_map_with_env(
            self,
            pcd_path,
            x,
            y,
            yaw,
            *,
            timeout_s: float = 20.0,
            base_env=None,
        ):
            return None

    modules = {
        "NavigationModule": Navigation(),
        "CmdVelMux": Mux(),
        "SlamBridgeModule": Relocalization(),
    }
    gateway = GatewayModule()
    gateway.on_system_modules(modules)

    assert gateway._all_modules == modules
    assert gateway._all_modules is not modules
    assert gateway._navigation_module is modules["NavigationModule"]
    assert gateway._backend_reconfigure_modules["NavigationModule"] is (
        modules["NavigationModule"]
    )
    assert gateway._relocalization_service is modules["SlamBridgeModule"]


def test_gateway_runtime_backend_switch_dispatches_when_navigation_idle():
    from gateway.gateway_module import GatewayModule

    class IdleNavigation:
        def health(self):
            return {"state": "IDLE"}

    class SwitchableModule:
        def reconfigure_backend(self, category, backend, **config):
            return {
                "ok": True,
                "category": category,
                "backend": backend,
                "config": config,
            }

    gateway = GatewayModule()
    gateway.on_system_modules(
        {
            "NavigationModule": IdleNavigation(),
            "PerceptionModule": SwitchableModule(),
        }
    )

    result = gateway.reconfigure_backend("detector", "mock_detector", threshold=0.4)

    assert result["ok"] is True
    assert result["category"] == "detector"
    assert result["backend"] == "mock_detector"
    assert result["config"] == {"threshold": 0.4}


def test_gateway_motion_backend_switch_reads_nested_navigation_state():
    from gateway.gateway_module import GatewayModule

    class IdleNavigation:
        def health(self):
            return {"navigation": {"state": "IDLE"}}

    gateway = GatewayModule()
    gateway.on_system_modules({"NavigationModule": IdleNavigation()})

    result = gateway.reconfigure_backend("local_planner", "nav_core")

    assert result["ok"] is False
    assert result["reason"] == "backend_reconfigure_unsupported"


def test_gateway_motion_backend_switch_requires_public_navigation_state():
    from gateway.gateway_module import GatewayModule

    class NavigationWithoutPublicState:
        _state = "IDLE"

        def health(self):
            return {}

    gateway = GatewayModule()
    gateway.on_system_modules({"NavigationModule": NavigationWithoutPublicState()})

    result = gateway.reconfigure_backend("local_planner", "nav_core")

    assert result["ok"] is False
    assert result["reason"] == "motion_backend_switch_requires_idle"
    assert result["navigation_state"] == "UNKNOWN"


def test_gateway_and_mcp_backend_route_tables_stay_in_parity():
    from gateway import gateway_module, mcp_server

    assert mcp_server._MOTION_BACKEND_CATEGORIES == gateway_module._MOTION_BACKEND_CATEGORIES
    assert mcp_server._BACKEND_RECONFIGURE_TARGETS == gateway_module._BACKEND_RECONFIGURE_TARGETS


def test_mcp_backend_switch_tool_uses_gateway_guard():
    from gateway.mcp_server import MCPServerModule

    class Gateway:
        def reconfigure_backend(self, category, backend, **config):
            return {
                "ok": False,
                "category": category,
                "requested_backend": backend,
                "reason": "motion_backend_switch_requires_idle",
                "config": config,
            }

    mcp = MCPServerModule()
    mcp.on_system_modules({"MCPServerModule": mcp, "GatewayModule": Gateway()})

    payload = json.loads(
        mcp.switch_backend("local_planner", "nav_core", '{"force": false}')
    )

    assert payload["ok"] is False
    assert payload["reason"] == "motion_backend_switch_requires_idle"
    assert payload["config"] == {"force": False}


def test_mcp_backend_switch_tool_guards_motion_without_gateway_module():
    from gateway.mcp_server import MCPServerModule

    class BusyNavigation:
        def health(self):
            return {"state": "EXECUTING"}

    mcp = MCPServerModule()
    mcp.on_system_modules({"MCPServerModule": mcp, "NavigationModule": BusyNavigation()})

    payload = json.loads(mcp.switch_backend("slam", "fastlio2"))

    assert payload["ok"] is False
    assert payload["reason"] == "motion_backend_switch_requires_idle"
    assert payload["navigation_state"] == "EXECUTING"


def test_mcp_backend_switch_reads_nested_navigation_state_without_gateway_module():
    from gateway.mcp_server import MCPServerModule

    class IdleNavigation:
        def health(self):
            return {"navigation": {"state": "IDLE"}}

    mcp = MCPServerModule()
    mcp.on_system_modules({"MCPServerModule": mcp, "NavigationModule": IdleNavigation()})

    payload = json.loads(mcp.switch_backend("slam", "fastlio2"))

    assert payload["ok"] is False
    assert payload["reason"] == "backend_reconfigure_unsupported"


def test_mcp_backend_switch_requires_public_navigation_state_without_gateway_module():
    from gateway.mcp_server import MCPServerModule

    class NavigationWithoutPublicState:
        _state = "IDLE"

        def health(self):
            return {}

    mcp = MCPServerModule()
    mcp.on_system_modules(
        {"MCPServerModule": mcp, "NavigationModule": NavigationWithoutPublicState()}
    )

    payload = json.loads(mcp.switch_backend("slam", "fastlio2"))

    assert payload["ok"] is False
    assert payload["reason"] == "motion_backend_switch_requires_idle"
    assert payload["navigation_state"] == "UNKNOWN"


def test_localization_status_covers_product_states():
    from gateway.gateway_module import GatewayModule
    from gateway.services.runtime_status import build_localization_status

    gateway = GatewayModule()

    payload = build_localization_status(gateway)
    assert payload["state"] == "no_odometry"
    assert payload["has_odometry"] is False
    assert payload["can_relocalize"] is False

    with gateway._state_lock:
        gateway._odom = {"x": 1.0}
        gateway._localization_status = {"state": "TRACKING", "confidence": 0.9}
    payload = build_localization_status(gateway)
    assert payload["state"] == "tracking"
    assert payload["reported_state"] == "TRACKING"

    gateway._icp_quality = 0.03
    with gateway._state_lock:
        gateway._localization_status = {
            "state": "TRACKING",
            "confidence": 0.9,
            "degeneracy": "NONE",
            "localizer_health": "RECOVERED",
        }
    payload = build_localization_status(gateway)
    assert payload["state"] == "ready"
    assert payload["ready"] is True
    assert payload["algorithm_healthy"] is True
    assert payload["pose_fresh"] is True

    gateway._icp_quality = 0.0
    with gateway._state_lock:
        gateway._localization_status = {
            "state": "TRACKING",
            "confidence": 0.89,
            "degeneracy": "NONE",
            "icp_fitness": 0.0,
            "localizer_health": "RECOVERED",
            "localizer_health_source": "localizer_health_topic",
            "localizer_health_fitness": 0.0246,
            "odom_age_ms": 222.4,
        }
    payload = build_localization_status(gateway)
    assert payload["state"] == "ready"
    assert payload["ready"] is True
    assert payload["algorithm_healthy"] is True

    gateway._icp_quality = 0.0
    with gateway._state_lock:
        gateway._localization_status = {
            "state": "TRACKING",
            "confidence": 0.7,
            "degeneracy": "MILD",
            "health_source": "odom_map_cloud",
            "pose_fresh": True,
            "map_cloud_fresh": True,
            "icp_fitness": 0.0,
            "odom_age_ms": 150.0,
            "cloud_age_ms": 120.0,
            "localizer_health": "LIO_TRACKING",
        }
    payload = build_localization_status(gateway)
    assert payload["state"] == "ready"
    assert payload["ready"] is True
    assert payload["algorithm_healthy"] is True
    assert payload["degeneracy"] == "MILD"
    assert payload["reasons"] == []

    gateway._session_mode = "navigating"
    gateway._icp_quality = 0.2
    with gateway._state_lock:
        gateway._localization_status = {"state": "TRACKING", "confidence": 0.9}
    payload = build_localization_status(gateway)
    assert payload["state"] == "ready"
    assert payload["ready"] is True

    with gateway._state_lock:
        gateway._localization_status = {
            "state": "DEGRADED",
            "confidence": 0.4,
            "degeneracy": "MILD",
        }
    payload = build_localization_status(gateway)
    assert payload["state"] == "degraded"
    assert payload["can_relocalize"] is True
    assert "low_confidence" in payload["reasons"]

    with gateway._state_lock:
        gateway._localization_status = {
            "state": "TRACKING",
            "confidence": 0.28,
            "degeneracy": "NONE",
            "icp_fitness": 0.028,
            "odom_age_ms": 1440.0,
            "cloud_age_ms": 120.0,
            "localizer_health": "RECOVERED",
        }
    payload = build_localization_status(gateway)
    assert payload["state"] == "ready"
    assert payload["algorithm_healthy"] is True
    assert payload["pose_fresh"] is True
    assert payload["pose_freshness"] == "fresh"
    assert payload["stale_odometry"] is False
    assert payload["odom_age_ms"] == 1440.0
    assert payload["reasons"] == []

    with gateway._state_lock:
        gateway._localization_status = {
            "state": "TRACKING",
            "confidence": 0.28,
            "degeneracy": "NONE",
            "icp_fitness": 0.028,
            "odom_age_ms": 2500.0,
            "cloud_age_ms": 120.0,
            "localizer_health": "RECOVERED",
        }
    payload = build_localization_status(gateway)
    assert payload["state"] == "degraded"
    assert payload["algorithm_healthy"] is True
    assert payload["pose_fresh"] is False
    assert payload["pose_freshness"] == "stale"
    assert payload["stale_odometry"] is True
    assert payload["odom_age_ms"] == 2500.0
    assert payload["reasons"] == ["reported_state:tracking", "stale_odometry"]

    with gateway._state_lock:
        gateway._localization_status = {"state": "LOST", "confidence": 0.0}
    payload = build_localization_status(gateway)
    assert payload["state"] == "lost"
    assert payload["can_relocalize"] is True

    with gateway._state_lock:
        gateway._localization_status = {
            "state": "TRACKING",
            "confidence": 0.9,
            "localizer_health": "LOST",
        }
    payload = build_localization_status(gateway)
    assert payload["state"] == "lost"
    assert "localizer_health:lost" in payload["reasons"]


def test_localization_status_route_returns_stable_schema():
    from gateway.gateway_module import GatewayModule

    gateway = GatewayModule()
    with gateway._state_lock:
        gateway._odom = {"x": 0.0}
        gateway._localization_status = {"state": "TRACKING", "confidence": 0.8}

    payload = asyncio.run(_endpoint(gateway, "/api/v1/localization/status")())

    assert payload["schema_version"] == 1
    assert payload["state"] == "tracking"
    assert payload["has_odometry"] is True
    assert payload["reported_state"] == "TRACKING"


def test_localization_status_reports_runtime_boundary_and_topic_frames(monkeypatch):
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import LocalizationStatusResponse
    from gateway.services.runtime_status import build_localization_status

    monkeypatch.setenv("LINGTU_PROFILE", "nav")
    monkeypatch.setenv("LINGTU_ENDPOINT", "thunder_field")
    monkeypatch.setenv("LINGTU_DATA_SOURCE", "thunder_field")
    monkeypatch.setenv("LINGTU_RUNTIME_CONTRACT", "thunder_field")
    monkeypatch.setenv("LINGTU_COMMAND_SINK", "hardware_driver_after_cmd_vel_mux")
    monkeypatch.setenv("LINGTU_SIMULATION_ONLY", "0")

    gateway = GatewayModule()
    gateway._session_mode = "navigating"
    with gateway._state_lock:
        gateway._odom = {"x": 0.0, "y": 0.0, "frame_id": "odom"}
        gateway._localization_status = {
            "state": "TRACKING",
            "confidence": 0.9,
            "registered_cloud_frame_id": "body",
            "map_cloud_frame_id": "map",
            "localizer_health": "RECOVERED",
            "odom_age_ms": 80.0,
            "cloud_age_ms": 60.0,
            "map_cloud_fresh": True,
        }

    payload = build_localization_status(gateway)
    model = LocalizationStatusResponse.model_validate(payload)

    runtime = payload["runtime"]
    assert runtime["ok"] is True
    assert runtime["data_source"] == "thunder_field"
    assert runtime["runtime_contract"] == "thunder_field"
    assert runtime["frames"]["map"] == "map"
    assert runtime["frames"]["odom"] == "odom"
    assert runtime["frames"]["body"] == "body"
    assert runtime["topic_default_frame_ids"]["/nav/odometry"] == "odom"
    assert runtime["topic_default_frame_ids"]["/nav/registered_cloud"] == "body"
    assert runtime["topic_default_frame_ids"]["/nav/map_cloud"] == "map"
    assert runtime["required_topic_frame_ids"][:5] == [
        "/nav/lidar_scan",
        "/nav/imu",
        "/nav/odometry",
        "/nav/registered_cloud",
        "/nav/map_cloud",
    ]
    assert runtime["runtime_data_flow_topics"][:3] == [
        "/nav/lidar_scan",
        "/nav/imu",
        "/nav/odometry",
    ]

    frames = payload["frames"]
    assert frames["runtime_contract"] == "thunder_field"
    assert frames["odometry_frame_id"] == "odom"
    assert frames["registered_cloud_frame_id"] == "body"
    assert frames["map_cloud_frame_id"] == "map"
    assert frames["odometry_expected_frame_ids"] == ["odom", "map"]
    assert frames["registered_cloud_expected_frame_ids"] == ["body"]
    assert frames["map_cloud_expected_frame_ids"] == ["map"]
    assert frames["missing_required_topic_frame_ids"] == []
    assert frames["mismatches"] == []
    assert frames["ok"] is True
    assert model.runtime.data_source == "thunder_field"
    assert model.frames.ok is True


def test_localization_status_exposes_gateway_diagnostic_age():
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import LocalizationStatusResponse
    from gateway.services.runtime_status import build_localization_status

    gateway = GatewayModule()
    with gateway._state_lock:
        gateway._odom = {"x": 0.0}
    gateway._on_localization_status({"state": "TRACKING", "confidence": 0.9})

    payload = build_localization_status(gateway)
    model = LocalizationStatusResponse.model_validate(payload)

    assert model.diag_received_ts is not None
    assert model.diag_age_ms is not None
    assert model.diag_age_ms >= 0.0
    assert "_gateway_received_mono" in payload["raw"]


def test_localization_status_exposes_slam_quality_diagnostics():
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import LocalizationStatusResponse
    from gateway.services.runtime_status import build_localization_status

    gateway = GatewayModule()
    with gateway._state_lock:
        gateway._odom = {"x": 0.0}
        gateway._localization_status = {
            "state": "DEGRADED",
            "confidence": 0.1,
            "degeneracy": "CRITICAL",
            "icp_fitness": 0.3049,
            "effective_ratio": 1.0,
            "condition_number": 50.4,
            "degenerate_dof_count": 0,
            "pos_cov_trace": 0.000017,
            "ieskf_iter_num": 10,
            "ieskf_converged": False,
            "localizer_health": "DEGRADED",
            "localizer_health_fitness": 0.3049,
            "localizer_health_iter": 11,
            "localizer_health_cov_trace": 0.000019,
        }

    payload = build_localization_status(gateway)
    model = LocalizationStatusResponse.model_validate(payload)

    assert model.state == "degraded"
    assert model.icp_fitness == 0.3049
    assert model.effective_ratio == 1.0
    assert model.condition_number == 50.4
    assert model.degenerate_dof_count == 0
    assert model.pos_cov_trace == 0.000017
    assert model.ieskf_iter_num == 10
    assert model.ieskf_converged is False
    assert model.localizer_health == "DEGRADED"
    assert model.localizer_health_fitness == 0.3049
    assert model.localizer_health_iter == 11
    assert model.localizer_health_cov_trace == 0.000019
    assert payload["raw"]["icp_fitness"] == 0.3049


def test_localization_status_accepts_super_lio_odom_map_health():
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import LocalizationStatusResponse
    from gateway.services.runtime_status import build_localization_status

    gateway = GatewayModule()
    gateway._session_mode = "navigating"
    gateway._icp_quality = 0.03
    with gateway._state_lock:
        gateway._odom = {"x": 0.0}
        gateway._localization_status = {
            "backend": "super_lio",
            "state": "TRACKING",
            "confidence": 0.92,
            "health_source": "odom_map_cloud",
            "pose_fresh": True,
            "map_cloud_fresh": True,
            "map_state": "live_map_cloud",
            "map_save_supported": True,
            "map_save_source": "live_map_cloud_snapshot",
            "relocalization_supported": False,
            "saved_map_relocalization_supported": False,
            "restart_recovery_supported": True,
            "recovery_method": "restart_super_lio",
            "relocalization_state": "unsupported",
            "recovery_signal": "NONE",
            "recovery_action": "none",
            "localizer_health": "LIO_TRACKING",
            "odom_age_ms": 120.0,
            "cloud_age_ms": 80.0,
        }

    payload = build_localization_status(gateway)
    model = LocalizationStatusResponse.model_validate(payload)

    assert model.state == "ready"
    assert model.ready is True
    assert model.algorithm_healthy is True
    assert model.backend == "super_lio"
    assert model.health_source == "odom_map_cloud"
    assert model.map_cloud_fresh is True
    assert model.map_save_supported is True
    assert model.map_save_source == "live_map_cloud_snapshot"
    assert model.relocalization_supported is False
    assert model.saved_map_relocalization_supported is False
    assert model.restart_recovery_supported is True
    assert model.recovery_method == "restart_super_lio"
    assert model.can_relocalize is False
    assert model.recovery_signal == "NONE"
    assert model.recovery_action == "none"


def test_localization_status_fills_super_lio_map_save_defaults():
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import LocalizationStatusResponse
    from gateway.services.runtime_status import build_localization_status

    gateway = GatewayModule()
    gateway._session_mode = "navigating"
    with gateway._state_lock:
        gateway._odom = {"x": 0.0}
        gateway._localization_status = {
            "backend": "super_lio",
            "state": "TRACKING",
            "confidence": 0.92,
            "health_source": "odom_map_cloud",
            "pose_fresh": True,
            "map_cloud_fresh": True,
            "localizer_health": "LIO_TRACKING",
            "odom_age_ms": 120.0,
            "cloud_age_ms": 80.0,
        }

    payload = build_localization_status(gateway)
    model = LocalizationStatusResponse.model_validate(payload)

    assert model.ready is True
    assert model.map_save_supported is True
    assert model.map_save_source == "live_map_cloud_snapshot"
    assert model.relocalization_supported is False
    assert model.saved_map_relocalization_supported is False
    assert model.restart_recovery_supported is True
    assert model.recovery_method == "restart_super_lio"


def test_localization_status_accepts_super_lio_relocation_contract():
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import LocalizationStatusResponse
    from gateway.services.runtime_status import build_localization_status

    gateway = GatewayModule()
    gateway._session_mode = "navigating"
    with gateway._state_lock:
        gateway._odom = {"x": 0.0}
        gateway._localization_status = {
            "backend": "super_lio_relocation",
            "state": "TRACKING",
            "confidence": 0.88,
            "health_source": "odom_map_cloud",
            "pose_fresh": True,
            "map_cloud_fresh": True,
            "map_state": "relocation_map_cloud",
            "map_save_supported": False,
            "map_save_source": "active_map",
            "relocalization_supported": False,
            "saved_map_relocalization_supported": False,
            "restart_recovery_supported": True,
            "recovery_method": "restart_super_lio_relocation",
            "relocalization_state": "unsupported",
            "recovery_signal": "NONE",
            "recovery_action": "none",
            "localizer_health": "LIO_TRACKING",
            "odom_age_ms": 120.0,
            "cloud_age_ms": 80.0,
        }

    payload = build_localization_status(gateway)
    model = LocalizationStatusResponse.model_validate(payload)

    assert model.state == "ready"
    assert model.ready is True
    assert model.backend == "super_lio_relocation"
    assert model.health_source == "odom_map_cloud"
    assert model.map_cloud_fresh is True
    assert model.map_save_supported is False
    assert model.map_save_source == "active_map"
    assert model.relocalization_supported is False
    assert model.saved_map_relocalization_supported is False
    assert model.restart_recovery_supported is True
    assert model.recovery_method == "restart_super_lio_relocation"
    assert model.can_relocalize is False


def test_super_lio_relocation_alias_keeps_conservative_capabilities():
    from gateway.services.runtime_status import backend_capability_defaults

    defaults = backend_capability_defaults("relocation")

    assert defaults["relocalization_supported"] is False
    assert defaults["saved_map_relocalization_supported"] is False
    assert defaults["restart_recovery_supported"] is True
    assert defaults["recovery_method"] == "restart_super_lio_relocation"


def test_super_lio_degraded_state_does_not_offer_relocalize():
    from gateway.gateway_module import GatewayModule
    from gateway.services.runtime_status import build_localization_status

    gateway = GatewayModule()
    with gateway._state_lock:
        gateway._odom = {"x": 0.0}
        gateway._localization_status = {
            "backend": "super_lio",
            "state": "DEGRADED",
            "confidence": 0.2,
            "health_source": "odom_map_cloud",
            "pose_fresh": True,
            "relocalization_supported": False,
            "localizer_health": "LIO_DEGRADED",
        }

    payload = build_localization_status(gateway)

    assert payload["state"] == "degraded"
    assert payload["relocalization_supported"] is False
    assert payload["saved_map_relocalization_supported"] is False
    assert payload["restart_recovery_supported"] is True
    assert payload["recovery_method"] == "restart_super_lio"
    assert payload["can_relocalize"] is False


def test_session_snapshot_exposes_super_lio_backend_capabilities():
    import time

    from gateway.gateway_module import GatewayModule
    from gateway.schemas import SessionResponse

    gateway = GatewayModule()
    gateway._cached_slam_profile = "super_lio"
    gateway._slam_profile_ts = time.time()
    gateway._session_mode = "mapping"
    with gateway._state_lock:
        gateway._localization_status = {
            "backend": "super_lio",
            "state": "TRACKING",
            "health_source": "odom_map_cloud",
            "pose_fresh": True,
            "map_state": "live_map_cloud",
            "map_save_supported": True,
            "map_save_source": "live_map_cloud_snapshot",
            "relocalization_supported": False,
            "saved_map_relocalization_supported": False,
            "restart_recovery_supported": True,
            "recovery_method": "restart_super_lio",
            "relocalization_state": "unsupported",
            "recovery_signal": "NONE",
            "recovery_action": "restart_super_lio",
            "localizer_health": "LIO_TRACKING",
        }

    session = gateway._session_snapshot()
    model = SessionResponse.model_validate(session)

    assert model.slam_profile == "super_lio"
    assert model.localization_backend == "super_lio"
    assert model.health_source == "odom_map_cloud"
    assert model.localizer_ready is True
    assert model.map_save_supported is True
    assert model.map_save_source == "live_map_cloud_snapshot"
    assert model.relocalization_supported is False
    assert model.saved_map_relocalization_supported is False
    assert model.restart_recovery_supported is True
    assert model.recovery_method == "restart_super_lio"
    assert model.relocalization_state == "unsupported"
    assert model.recovery_action == "restart_super_lio"


def test_session_snapshot_exposes_super_lio_relocation_capabilities():
    import time

    from gateway.gateway_module import GatewayModule
    from gateway.schemas import SessionResponse

    gateway = GatewayModule()
    gateway._cached_slam_profile = "super_lio_relocation"
    gateway._slam_profile_ts = time.time()
    gateway._session_mode = "navigating"
    gateway._session_map = "demo"
    with gateway._state_lock:
        gateway._localization_status = {
            "backend": "super_lio_relocation",
            "state": "TRACKING",
            "health_source": "odom_map_cloud",
            "pose_fresh": True,
            "map_state": "relocation_map_cloud",
            "map_save_supported": False,
            "map_save_source": "active_map",
            "relocalization_supported": False,
            "saved_map_relocalization_supported": False,
            "restart_recovery_supported": True,
            "recovery_method": "restart_super_lio_relocation",
            "relocalization_state": "unsupported",
            "recovery_signal": "NONE",
            "recovery_action": "restart_super_lio_relocation",
            "localizer_health": "LIO_TRACKING",
        }

    session = gateway._session_snapshot()
    model = SessionResponse.model_validate(session)

    assert model.slam_profile == "super_lio_relocation"
    assert model.localization_backend == "super_lio_relocation"
    assert model.health_source == "odom_map_cloud"
    assert model.localizer_ready is True
    assert model.map_save_supported is False
    assert model.map_save_source == "active_map"
    assert model.relocalization_supported is False
    assert model.saved_map_relocalization_supported is False
    assert model.restart_recovery_supported is True
    assert model.recovery_method == "restart_super_lio_relocation"
    assert model.relocalization_state == "unsupported"
    assert model.recovery_action == "restart_super_lio_relocation"


def test_navigation_status_reports_mission_path_and_control_source():
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import NavigationStatusResponse
    from gateway.services.runtime_status import build_navigation_status

    class FakeMux:
        def health(self):
            return {
                "active_source": "path_follower",
                "sources": {
                    "path_follower": {"active": True, "priority": 40},
                },
            }

    class FakeNavigation:
        def get_navigation_status(self):
            return json.dumps({
                "plan_safety_policy": "reject",
                "last_plan_report": {
                    "selected_planner": "pct",
                    "selected_path_safety": {
                        "ok": False,
                        "blocked_sample_count": 2,
                    },
                    "fallback_reason": "pct path_safety failed",
                    "rejected_plans": [{"planner": "pct", "reason": "unsafe"}],
                    "policy": "reject",
                },
            })

    gateway = GatewayModule()
    gateway._session_mode = "navigating"
    with gateway._state_lock:
        gateway._odom = {"x": 1.0, "y": 2.0, "vx": 0.3, "vy": 0.4}
        gateway._mode = "autonomous"
        gateway._mission = {
            "state": "EXECUTING",
            "planning_frame_id": "map",
            "odom_frame_id": "map",
            "costmap_frame_id": "map",
            "goal_frame_id": "map",
            "wp_index": 2,
            "wp_total": 5,
            "remaining_waypoints": 3,
            "goal": [4.0, 6.0, 0.0],
            "current_waypoint": [2.0, 3.0, 0.0],
            "distance_to_goal_m": 5.0,
            "active_waypoint_distance_m": 1.25,
            "replan_count": 1,
            "speed_scale": 0.5,
            "speed_policy": {
                "scale": 0.5,
                "mode": "cautious",
                "reason": "degeneracy=MILD",
                "source": "localization_degeneracy",
                "applied": True,
            },
            "failure_reason": "",
            "ts": 123.0,
        }
        gateway._last_path = [{"x": 0.0}, {"x": 1.0}, {"x": 2.0}]
        gateway._localization_status = {
            "state": "TRACKING",
            "confidence": 0.9,
            "icp_fitness": 0.03,
        }
    gateway._all_modules = {
        "CmdVelMux": FakeMux(),
        "NavigationModule": FakeNavigation(),
    }

    payload = build_navigation_status(gateway)
    NavigationStatusResponse.model_validate(payload)

    assert payload["state"] == "EXECUTING"
    assert payload["can_accept_goal"] is True
    assert payload["wp_index"] == 2
    assert payload["wp_total"] == 5
    assert payload["replan_count"] == 1
    assert payload["speed_scale"] == 0.5
    assert payload["path"]["points"] == 3
    assert payload["path"]["endpoint"] == "/api/v1/path"
    assert payload["frames"]["planning_frame_id"] == "map"
    assert payload["frames"]["odom_frame_id"] == "map"
    assert payload["frames"]["costmap_frame_id"] == "map"
    assert payload["frames"]["goal_frame_id"] == "map"
    assert payload["frames"]["ok"] is True
    assert payload["frames"]["mismatches"] == []
    assert payload["control"]["mode"] == "autonomous"
    assert payload["control"]["active_cmd_source"] == "path_follower"
    assert payload["control"]["command_owner"] == "navigation"
    assert payload["control"]["source_category"] == "autonomy"
    assert payload["control"]["manual_override"] is False
    assert payload["control"]["preempting_autonomy"] is False
    assert payload["progress"]["fraction"] == 0.4
    assert payload["progress"]["active"] is True
    assert payload["readiness"]["can_execute_autonomy"] is True
    assert payload["readiness"]["session_mode"] == "navigating"
    assert payload["target"]["goal"]["x"] == 4.0
    assert payload["target"]["current_waypoint"]["y"] == 3.0
    assert payload["target"]["distance_to_goal_m"] == 5.0
    assert payload["target"]["active_waypoint_distance_m"] == 1.414
    assert payload["target"]["remaining_waypoints"] == 3
    assert payload["motion"]["current_speed_mps"] == 0.5
    assert payload["motion"]["speed_policy"]["mode"] == "cautious"
    assert payload["motion"]["speed_policy"]["reason"] == "degeneracy=MILD"
    assert payload["feedback"]["next_action"] == "monitor_progress"
    assert payload["diagnostics"]["plan_safety_policy"] == "reject"
    assert payload["diagnostics"]["last_plan_report"]["selected_planner"] == "pct"
    assert (
        payload["diagnostics"]["last_plan_report"]["selected_path_safety"]["ok"]
        is False
    )
    assert payload["reason_codes"] == []
    assert payload["localization"]["degraded"] is False


def test_drift_watchdog_classifies_nan_odom_as_diverged():
    from gateway.gateway_module import GatewayModule

    gateway = GatewayModule()

    assert gateway._drift_odom_diverged({
        "x": float("nan"),
        "y": 0.0,
        "z": 0.0,
        "vx": 0.0,
    })[0] is True


def test_gateway_quarantines_non_finite_odometry_before_publication():
    from core.msgs.geometry import Pose, Quaternion, Twist, Vector3
    from core.msgs.nav import Odometry
    from gateway.gateway_module import GatewayModule

    gateway = GatewayModule()
    events = []
    gateway.push_event = events.append
    previous = {"x": 1.0, "y": 2.0, "z": 0.0, "yaw": 0.0}
    with gateway._state_lock:
        gateway._odom = dict(previous)

    gateway._on_odometry(Odometry(
        pose=Pose(
            position=Vector3(float("nan"), 0.0, 0.0),
            orientation=Quaternion(),
        ),
        twist=Twist(linear=Vector3(0.0, 0.0, 0.0)),
    ))

    assert gateway._odom == previous
    assert [event.get("type") for event in events] == []
    assert gateway._last_invalid_odometry["reason"] == "non_finite_odometry"


def test_drift_watchdog_uses_quarantined_non_finite_odometry():
    from core.msgs.geometry import Pose, Quaternion, Twist, Vector3
    from core.msgs.nav import Odometry
    from gateway.gateway_module import GatewayModule

    gateway = GatewayModule()
    with gateway._state_lock:
        gateway._odom = {"x": 1.0, "y": 2.0, "z": 0.0, "vx": 0.0}

    gateway._on_odometry(Odometry(
        pose=Pose(
            position=Vector3(float("nan"), 0.0, 0.0),
            orientation=Quaternion(),
        ),
        twist=Twist(linear=Vector3(0.0, 0.0, 0.0)),
    ))

    diverged, _x, _y, _z, _v, invalid = gateway._drift_current_odom_divergence()
    assert diverged is True
    assert invalid is True

    gateway._on_odometry(Odometry(
        pose=Pose(
            position=Vector3(0.0, 0.0, 0.0),
            orientation=Quaternion(),
        ),
        twist=Twist(linear=Vector3(0.0, 0.0, 0.0)),
    ))

    diverged, _x, _y, _z, _v, invalid = gateway._drift_current_odom_divergence()
    assert diverged is False
    assert invalid is False


def test_navigation_status_reports_current_runtime_boundary(monkeypatch):
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import NavigationStatusResponse
    from gateway.services.runtime_status import build_navigation_status

    monkeypatch.setenv("LINGTU_PROFILE", "nav")
    monkeypatch.setenv("LINGTU_ENDPOINT", "thunder_field")
    monkeypatch.setenv("LINGTU_DATA_SOURCE", "thunder_field")
    monkeypatch.setenv("LINGTU_RUNTIME_CONTRACT", "thunder_field")
    monkeypatch.setenv("LINGTU_COMMAND_SINK", "hardware_driver_after_cmd_vel_mux")
    monkeypatch.setenv("LINGTU_SIMULATION_ONLY", "0")

    gateway = GatewayModule()
    gateway._session_mode = "navigating"
    with gateway._state_lock:
        gateway._odom = {"x": 1.0, "y": 2.0, "frame_id": "map"}
        gateway._mode = "autonomous"
        gateway._mission = {
            "state": "EXECUTING",
            "planning_frame_id": "map",
            "costmap_frame_id": "map",
            "goal_frame_id": "map",
        }
        gateway._localization_status = {"state": "TRACKING", "confidence": 0.9}

    payload = build_navigation_status(gateway)
    NavigationStatusResponse.model_validate(payload)

    runtime = payload["runtime"]
    assert runtime["ok"] is True
    assert runtime["declared"] is True
    assert runtime["profile"] == "nav"
    assert runtime["endpoint"] == "thunder_field"
    assert runtime["data_source"] == "thunder_field"
    assert runtime["runtime_contract"] == "thunder_field"
    assert runtime["simulation_only"] is False
    assert runtime["command_sink"] == "hardware_driver_after_cmd_vel_mux"
    assert runtime["expected_command_sink"] == "hardware_driver_after_cmd_vel_mux"
    assert runtime["slam_source"] == "lingtu_fastlio_or_external_robot_slam"
    assert runtime["localization_source"] == "slam_localizer"
    assert runtime["mapping_source"] == "slam_map_cloud"
    assert runtime["frames"]["map"] == "map"
    assert runtime["frames"]["odom"] == "odom"
    assert runtime["frames"]["body"] == "body"
    assert runtime["frames"]["axis_convention"] == "x_forward_y_left_z_up"
    assert runtime["frame_links"]["body_to_lidar"] == {
        "parent": "body",
        "child": "lidar_link",
        "required": True,
    }
    assert runtime["topic_allowed_frame_ids"]["/nav/map_cloud"] == ["map"]
    assert runtime["topic_allowed_frame_ids"]["/nav/global_path"] == ["map"]
    assert runtime["topic_default_frame_ids"]["/nav/map_cloud"] == "map"
    assert runtime["topic_default_frame_ids"]["/nav/cmd_vel"] == "body"
    assert runtime["required_topic_frame_ids"] == [
        "/nav/lidar_scan",
        "/nav/imu",
        "/nav/odometry",
        "/nav/registered_cloud",
        "/nav/map_cloud",
        "/nav/global_path",
        "/nav/local_path",
        "/nav/cmd_vel",
    ]
    assert runtime["runtime_data_flow_topics"] == [
        "/nav/lidar_scan",
        "/nav/imu",
        "/nav/odometry",
        "/nav/registered_cloud",
        "/nav/map_cloud",
        "/nav/localization_health",
        "/nav/localization_quality",
        "/nav/exploration_grid",
        "/nav/terrain_map_ext",
        "/exploration/way_point",
        "/nav/goal_pose",
        "/nav/traversable_frontiers",
        "/nav/frontier_candidate",
        "/nav/global_path",
        "/nav/local_path",
        "/nav/cmd_vel",
        "/nav/added_obstacles",
        "/nav/check_obstacle",
        "/nav/planner_status",
    ]
    flow = {stage["name"]: stage for stage in runtime["resolved_runtime_data_flow"]}
    assert list(flow["endpoint_adapter"]["inputs"]) == [
        "/nav/lidar_scan",
        "/nav/imu",
    ]
    assert list(flow["command_boundary"]["outputs"]) == [
        "hardware_driver_after_cmd_vel_mux"
    ]
    assert runtime["runtime_data_flow_stage_algorithm_interfaces"]["global_planning"] == [
        "global_planning",
        "astar_global_planning",
        "pct_global_planning",
    ]
    assert runtime["runtime_data_flow_stage_algorithm_interfaces"][
        "local_planning_and_following"
    ] == ["local_planning_and_following"]


def test_navigation_frame_summary_defaults_planning_frame_from_runtime_contract(
    monkeypatch,
):
    import core.runtime_interface as runtime_interface
    from gateway.services import runtime_status

    calls: list[tuple[str | None, str]] = []

    def fake_runtime_topic_default_frame_id(
        runtime_contract: str | None,
        topic: str,
    ) -> str:
        calls.append((runtime_contract, topic))
        return "contract_map"

    monkeypatch.setenv("LINGTU_RUNTIME_CONTRACT", "thunder_field")
    monkeypatch.setattr(
        runtime_interface,
        "runtime_topic_default_frame_id",
        fake_runtime_topic_default_frame_id,
    )

    summary = runtime_status._navigation_frame_summary({}, None)

    assert summary["planning_frame_id"] == "contract_map"
    assert summary["ok"] is True
    assert calls == [("thunder_field", runtime_interface.TOPICS.global_path)]


def test_navigation_status_flags_runtime_boundary_mismatch(monkeypatch):
    from gateway.gateway_module import GatewayModule
    from gateway.services.runtime_status import build_navigation_status

    monkeypatch.setenv("LINGTU_PROFILE", "explore")
    monkeypatch.setenv("LINGTU_ENDPOINT", "mujoco_live")
    monkeypatch.setenv("LINGTU_DATA_SOURCE", "mujoco_fastlio2_live")
    monkeypatch.setenv("LINGTU_RUNTIME_CONTRACT", "thunder_field")
    monkeypatch.setenv("LINGTU_COMMAND_SINK", "hardware_driver_after_cmd_vel_mux")
    monkeypatch.setenv("LINGTU_SIMULATION_ONLY", "1")

    gateway = GatewayModule()
    with gateway._state_lock:
        gateway._odom = {"x": 1.0, "frame_id": "map"}
        gateway._mission = {"state": "EXECUTING", "planning_frame_id": "map"}
        gateway._localization_status = {"state": "TRACKING", "confidence": 0.9}

    runtime = build_navigation_status(gateway)["runtime"]

    assert runtime["ok"] is False
    assert "runtime_contract_data_source_mismatch" in runtime["blockers"]
    assert "command_sink_mismatch" in runtime["blockers"]
    assert runtime["expected_command_sink"] == "mujoco_velocity_adapter"
    assert runtime["topic_allowed_frame_ids"]["/nav/map_cloud"] == ["map"]
    assert runtime["required_topic_frame_ids"] == [
        "/nav/lidar_scan",
        "/nav/imu",
        "/nav/odometry",
        "/nav/registered_cloud",
        "/nav/map_cloud",
        "/nav/global_path",
        "/nav/local_path",
        "/nav/cmd_vel",
    ]
    assert runtime["runtime_data_flow_topics"][:2] == ["/points_raw", "/imu_raw"]


def test_navigation_status_reports_sim_runtime_topic_frames(monkeypatch):
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import NavigationStatusResponse
    from gateway.services.runtime_status import build_navigation_status

    monkeypatch.setenv("LINGTU_PROFILE", "explore")
    monkeypatch.setenv("LINGTU_ENDPOINT", "mujoco_live")
    monkeypatch.setenv("LINGTU_DATA_SOURCE", "mujoco_fastlio2_live")
    monkeypatch.setenv("LINGTU_RUNTIME_CONTRACT", "mujoco_fastlio2_live")
    monkeypatch.setenv("LINGTU_COMMAND_SINK", "mujoco_velocity_adapter")
    monkeypatch.setenv("LINGTU_SIMULATION_ONLY", "1")

    gateway = GatewayModule()
    gateway._session_mode = "navigating"
    with gateway._state_lock:
        gateway._odom = {"x": 1.0, "frame_id": "odom"}
        gateway._mode = "autonomous"
        gateway._mission = {
            "state": "EXECUTING",
            "planning_frame_id": "map",
            "costmap_frame_id": "map",
            "goal_frame_id": "map",
        }
        gateway._localization_status = {"state": "TRACKING", "confidence": 0.9}

    payload = build_navigation_status(gateway)
    NavigationStatusResponse.model_validate(payload)

    runtime = payload["runtime"]
    assert runtime["ok"] is True
    assert runtime["topic_allowed_frame_ids"]["/nav/map_cloud"] == ["map", "odom"]
    assert runtime["topic_allowed_frame_ids"]["/nav/global_path"] == ["map", "odom"]
    assert runtime["required_topic_frame_ids"] == []
    assert runtime["runtime_data_flow_topics"][:2] == ["/points_raw", "/imu_raw"]


def test_navigation_status_flags_unknown_topic_frame_contract(monkeypatch):
    from gateway.gateway_module import GatewayModule
    from gateway.services.runtime_status import build_navigation_status

    monkeypatch.setenv("LINGTU_PROFILE", "nav")
    monkeypatch.setenv("LINGTU_ENDPOINT", "thunder_field")
    monkeypatch.setenv("LINGTU_DATA_SOURCE", "thunder_field")
    monkeypatch.setenv("LINGTU_RUNTIME_CONTRACT", "typo_contract")
    monkeypatch.setenv("LINGTU_COMMAND_SINK", "hardware_driver_after_cmd_vel_mux")
    monkeypatch.setenv("LINGTU_SIMULATION_ONLY", "0")

    gateway = GatewayModule()
    with gateway._state_lock:
        gateway._odom = {"x": 1.0, "frame_id": "map"}
        gateway._mission = {"state": "EXECUTING", "planning_frame_id": "map"}
        gateway._localization_status = {"state": "TRACKING", "confidence": 0.9}

    runtime = build_navigation_status(gateway)["runtime"]

    assert runtime["ok"] is False
    assert "runtime_contract_data_source_mismatch" in runtime["blockers"]
    assert "topic_frame_contract_unavailable" in runtime["blockers"]
    assert runtime["topic_allowed_frame_ids"] == {}


def test_navigation_status_uses_mission_plan_report_when_module_status_unavailable():
    from gateway.gateway_module import GatewayModule
    from gateway.services.runtime_status import build_navigation_status

    class FakeMux:
        def health(self):
            return {
                "active_source": "path_follower",
                "sources": {
                    "path_follower": {"active": True, "priority": 40},
                },
            }

    gateway = GatewayModule()
    gateway._session_mode = "navigating"
    with gateway._state_lock:
        gateway._odom = {"x": 1.0, "y": 2.0}
        gateway._mode = "autonomous"
        gateway._mission = {
            "state": "PLANNING",
            "planning_frame_id": "map",
            "odom_frame_id": "map",
            "costmap_frame_id": "map",
            "goal_frame_id": "map",
            "plan_safety_policy": "fallback_astar",
            "last_plan_report": {
                "primary_planner": "pct",
                "selected_planner": "astar",
                "fallback_reason": "pct path_safety failed",
                "rejected_plans": [{"planner": "pct", "reason": "unsafe"}],
            },
        }
    gateway._all_modules = {"CmdVelMux": FakeMux()}

    payload = build_navigation_status(gateway)

    assert payload["diagnostics"]["plan_safety_policy"] == "fallback_astar"
    assert payload["diagnostics"]["last_plan_report"]["primary_planner"] == "pct"
    assert payload["diagnostics"]["last_plan_report"]["selected_planner"] == "astar"


def test_navigation_status_blocks_goal_on_odometry_frame_mismatch():
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import NavigationStatusResponse
    from gateway.services.runtime_status import build_navigation_status

    class FakeMux:
        def health(self):
            return {"active_source": "none", "sources": {}}

    gateway = GatewayModule()
    gateway._session_mode = "navigating"
    gateway._icp_quality = 0.03
    with gateway._state_lock:
        gateway._odom = {"x": 1.0, "y": 2.0, "frame_id": "camera_link"}
        gateway._mode = "autonomous"
        gateway._mission = {
            "state": "IDLE",
            "planning_frame_id": "map",
            "costmap_frame_id": "map",
        }
        gateway._localization_status = {"state": "TRACKING", "confidence": 0.9}
    gateway._all_modules = {"CmdVelMux": FakeMux()}

    payload = build_navigation_status(gateway)
    NavigationStatusResponse.model_validate(payload)

    assert payload["frames"]["ok"] is False
    assert payload["frames"]["mismatches"] == [
            {
                "source": "odometry",
                "expected_frame": "map",
                "received_frame": "camera_link",
            }
        ]
    assert "frame_mismatch_odometry" in payload["reason_codes"]
    assert "frame_mismatch_odometry" in payload["readiness"]["blockers"]
    assert payload["diagnostics"]["frame_mismatches"] == payload["frames"]["mismatches"]
    assert payload["can_accept_goal"] is False
    assert payload["readiness"]["can_execute_autonomy"] is False


def test_gateway_odometry_preserves_frame_for_navigation_status():
    from core.msgs.geometry import Pose, Quaternion, Vector3
    from core.msgs.nav import Odometry
    from gateway.gateway_module import GatewayModule
    from gateway.services.runtime_status import build_navigation_status

    gateway = GatewayModule()
    gateway._session_mode = "navigating"
    gateway._icp_quality = 0.03
    with gateway._state_lock:
        gateway._mode = "autonomous"
        gateway._mission = {"state": "IDLE", "planning_frame_id": "map"}
        gateway._localization_status = {"state": "TRACKING", "confidence": 0.9}

    gateway._on_odometry(Odometry(
        pose=Pose(position=Vector3(1.0, 2.0, 0.0), orientation=Quaternion()),
        frame_id="odom",
        child_frame_id="base_link",
    ))
    payload = build_navigation_status(gateway)

    assert gateway._odom["frame_id"] == "odom"
    assert gateway._odom["child_frame_id"] == "base_link"
    assert payload["frames"]["odom_frame_id"] == "odom"
    assert "frame_mismatch_odometry" in payload["reason_codes"]
    assert payload["frames"]["mismatches"] == [
        {
            "source": "odometry",
            "expected_frame": "map",
            "received_frame": "odom",
        }
    ]


def test_gateway_mission_event_pushes_navigation_status_update():
    from gateway.gateway_module import GatewayModule

    gateway = GatewayModule()
    gateway._session_mode = "navigating"
    gateway._icp_quality = 0.03
    with gateway._state_lock:
        gateway._odom = {"x": 0.0, "y": 0.0, "frame_id": "map"}
        gateway._mode = "autonomous"
        gateway._localization_status = {"state": "TRACKING", "confidence": 0.9}
    queue = gateway._sse_subscribe()

    try:
        gateway._on_mission({
            "state": "EXECUTING",
            "planning_frame_id": "map",
            "odom_frame_id": "map",
            "costmap_frame_id": "map",
        })
        events = []
        while not queue.empty():
            events.append(queue.get_nowait())
    finally:
        gateway._sse_unsubscribe(queue)

    assert [event["type"] for event in events] == ["mission", "navigation_status"]
    assert events[1]["data"]["state"] == "EXECUTING"
    assert events[1]["data"]["frames"]["ok"] is True


def test_navigation_status_reports_costmap_frame_mismatch():
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import NavigationStatusResponse
    from gateway.services.runtime_status import build_navigation_status

    class FakeMux:
        def health(self):
            return {"active_source": "none", "sources": {}}

    gateway = GatewayModule()
    gateway._session_mode = "navigating"
    gateway._icp_quality = 0.03
    with gateway._state_lock:
        gateway._odom = {"x": 1.0, "y": 2.0, "frame_id": "map"}
        gateway._mode = "autonomous"
        gateway._mission = {
            "state": "IDLE",
            "planning_frame_id": "map",
            "odom_frame_id": "map",
            "costmap_frame_id": "odom",
        }
        gateway._localization_status = {"state": "TRACKING", "confidence": 0.9}
    gateway._all_modules = {"CmdVelMux": FakeMux()}

    payload = build_navigation_status(gateway)
    NavigationStatusResponse.model_validate(payload)

    assert payload["frames"]["ok"] is False
    assert payload["frames"]["mismatches"] == [
        {
            "source": "costmap",
            "expected_frame": "map",
            "received_frame": "odom",
        }
    ]
    assert "frame_mismatch_costmap" in payload["reason_codes"]
    assert "frame_mismatch_costmap" in payload["readiness"]["blockers"]
    assert payload["can_accept_goal"] is False


def test_navigation_status_reads_idle_costmap_frame_from_navigation_module():
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import NavigationStatusResponse
    from gateway.services.runtime_status import build_navigation_status

    class FakeMux:
        def health(self):
            return {"active_source": "none", "sources": {}}

    class FakeNavigation:
        def get_navigation_status(self):
            return json.dumps({
                "state": "IDLE",
                "planning_frame_id": "map",
                "odom_frame_id": "map",
                "costmap_frame_id": "odom",
            })

    gateway = GatewayModule()
    gateway._session_mode = "navigating"
    gateway._icp_quality = 0.03
    with gateway._state_lock:
        gateway._odom = {"x": 1.0, "y": 2.0, "frame_id": "map"}
        gateway._mode = "autonomous"
        gateway._mission = {"state": "IDLE", "planning_frame_id": "map"}
        gateway._localization_status = {"state": "TRACKING", "confidence": 0.9}
    gateway._all_modules = {
        "CmdVelMux": FakeMux(),
        "NavigationModule": FakeNavigation(),
    }

    payload = build_navigation_status(gateway)
    NavigationStatusResponse.model_validate(payload)

    assert payload["frames"]["costmap_frame_id"] == "odom"
    assert payload["frames"]["ok"] is False
    assert payload["frames"]["mismatches"] == [
        {
            "source": "costmap",
            "expected_frame": "map",
            "received_frame": "odom",
        }
    ]
    assert "frame_mismatch_costmap" in payload["reason_codes"]
    assert "frame_mismatch_costmap" in payload["readiness"]["blockers"]
    assert payload["can_accept_goal"] is False


def test_navigation_status_prefers_injected_runtime_refs_without_module_inventory():
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import NavigationStatusResponse
    from gateway.services.runtime_status import build_navigation_status

    class FakeMux:
        def health(self):
            return {
                "active_source": "path_follower",
                "sources": {"path_follower": {"active": True, "priority": 40}},
            }

    class FakeNavigation:
        def get_navigation_status(self):
            return {
                "state": "IDLE",
                "planning_frame_id": "map",
                "odom_frame_id": "map",
                "costmap_frame_id": "odom",
            }

    gateway = GatewayModule()
    gateway._session_mode = "navigating"
    gateway._icp_quality = 0.03
    gateway._navigation_module = FakeNavigation()
    gateway._cmd_vel_mux = FakeMux()
    gateway._all_modules = {}
    with gateway._state_lock:
        gateway._odom = {"x": 1.0, "y": 2.0, "frame_id": "map"}
        gateway._mode = "autonomous"
        gateway._mission = {"state": "IDLE", "planning_frame_id": "map"}
        gateway._localization_status = {"state": "TRACKING", "confidence": 0.9}

    payload = build_navigation_status(gateway)
    NavigationStatusResponse.model_validate(payload)

    assert payload["frames"]["costmap_frame_id"] == "odom"
    assert payload["control"]["cmd_vel_mux"]["available"] is True
    assert payload["control"]["active_cmd_source"] == "path_follower"
    assert payload["control"]["source_category"] == "autonomy"


def test_navigation_status_blocks_goal_when_session_is_not_navigating():
    from gateway.gateway_module import GatewayModule
    from gateway.services.runtime_status import build_navigation_status

    class FakeMux:
        def health(self):
            return {"active_source": "none", "sources": {}}

    gateway = GatewayModule()
    gateway._session_mode = "idle"
    gateway._icp_quality = 0.03
    with gateway._state_lock:
        gateway._odom = {"x": 1.0, "y": 2.0}
        gateway._mode = "autonomous"
        gateway._mission = {"state": "IDLE"}
        gateway._localization_status = {"state": "TRACKING", "confidence": 0.9}
    gateway._all_modules = {"CmdVelMux": FakeMux()}

    payload = build_navigation_status(gateway)

    assert payload["can_accept_goal"] is False
    assert "navigation_session_inactive" in payload["reason_codes"]
    assert "navigation_session_inactive" in payload["readiness"]["blockers"]
    assert payload["readiness"]["session_mode"] == "idle"
    assert payload["feedback"]["next_action"] == "resolve_blockers"


def test_navigation_status_allows_exploring_session_for_external_tare():
    from gateway.gateway_module import GatewayModule
    from gateway.services.runtime_status import build_navigation_status

    class FakeMux:
        def health(self):
            return {"active_source": "path_follower", "sources": {}}

    gateway = GatewayModule()
    gateway._session_mode = "exploring"
    gateway._session_slam_profile = "none"
    gateway._icp_quality = 0.03
    with gateway._state_lock:
        gateway._odom = {"x": 1.0, "y": 2.0}
        gateway._mode = "autonomous"
        gateway._mission = {"state": "IDLE"}
        gateway._localization_status = {
            "state": "TRACKING",
            "confidence": 0.9,
            "pose_fresh": True,
            "odom_age_ms": 100.0,
        }
    gateway._all_modules = {"CmdVelMux": FakeMux()}

    payload = build_navigation_status(gateway)

    assert payload["readiness"]["session_mode"] == "exploring"
    assert "navigation_session_inactive" not in payload["reason_codes"]
    assert "navigation_session_inactive" not in payload["readiness"]["blockers"]


def test_navigation_status_finds_cmd_vel_mux_by_module_class_or_name():
    from gateway.gateway_module import GatewayModule
    from gateway.services.runtime_status import build_navigation_status

    class FakeCmdVelMuxModule:
        def health(self):
            return {
                "active_source": "path_follower",
                "sources": {
                    "path_follower": {"active": True, "priority": 40},
                },
            }

    gateway = GatewayModule()
    with gateway._state_lock:
        gateway._odom = {"x": 1.0, "y": 2.0}
        gateway._mission = {"state": "IDLE"}
        gateway._localization_status = {"state": "TRACKING", "confidence": 0.9}
    gateway._all_modules = {"navigation.CmdVelMuxModule": FakeCmdVelMuxModule()}

    payload = build_navigation_status(gateway)

    assert payload["control"]["cmd_vel_mux"]["available"] is True
    assert payload["control"]["active_cmd_source"] == "path_follower"
    assert payload["control"]["source_category"] == "autonomy"


def test_navigation_status_handles_failed_mission_and_missing_mux():
    from gateway.gateway_module import GatewayModule
    from gateway.services.runtime_status import build_navigation_status

    gateway = GatewayModule()
    with gateway._state_lock:
        gateway._odom = {"x": 0.0}
        gateway._mission = {
            "state": "STUCK",
            "failure_reason": "blocked",
            "speed_scale": 0.25,
        }
        gateway._localization_status = {
            "state": "DEGRADED",
            "confidence": 0.2,
        }

    payload = build_navigation_status(gateway)

    assert payload["state"] == "STUCK"
    assert payload["failure_reason"] == "blocked"
    assert payload["control"]["active_cmd_source"] == "unknown"
    assert payload["control"]["cmd_vel_mux"]["available"] is False
    assert payload["localization"]["degraded"] is True
    assert "mission_stuck" in payload["reason_codes"]
    assert "failure_blocked" in payload["reason_codes"]
    assert "localization_degraded" in payload["reason_codes"]
    assert "cmd_vel_mux_unavailable" in payload["reason_codes"]
    assert payload["diagnostics"]["failure_reason"] == "blocked"


def test_navigation_status_blocks_autonomy_when_pose_is_stale_but_algorithm_healthy():
    from gateway.gateway_module import GatewayModule
    from gateway.services.runtime_status import build_navigation_status

    class FakeMux:
        def health(self):
            return {"active_source": "none", "sources": {}}

    gateway = GatewayModule()
    gateway._session_mode = "navigating"
    gateway._icp_quality = 0.03
    with gateway._state_lock:
        gateway._odom = {"x": 0.0}
        gateway._mission = {"state": "IDLE"}
        gateway._localization_status = {
            "state": "TRACKING",
            "confidence": 0.28,
            "degeneracy": "NONE",
            "icp_fitness": 0.028,
            "odom_age_ms": 2500.0,
            "localizer_health": "RECOVERED",
        }
    gateway._all_modules = {"CmdVelMux": FakeMux()}

    payload = build_navigation_status(gateway)

    assert payload["localization"]["algorithm_healthy"] is True
    assert payload["localization"]["pose_fresh"] is False
    assert "pose_stale" in payload["reason_codes"]
    assert "pose_stale" in payload["readiness"]["blockers"]
    assert payload["can_accept_goal"] is False
    assert payload["readiness"]["can_accept_goal"] is False
    assert payload["readiness"]["can_execute_autonomy"] is False


def test_navigation_status_allows_fresh_pose_with_low_confidence_snapshot():
    from gateway.gateway_module import GatewayModule
    from gateway.services.runtime_status import build_navigation_status

    class FakeMux:
        def health(self):
            return {"active_source": "none", "sources": {}}

    gateway = GatewayModule()
    gateway._session_mode = "navigating"
    gateway._icp_quality = 0.03
    with gateway._state_lock:
        gateway._odom = {"x": 0.0}
        gateway._mission = {"state": "IDLE"}
        gateway._localization_status = {
            "state": "TRACKING",
            "confidence": 0.28,
            "degeneracy": "NONE",
            "icp_fitness": 0.028,
            "odom_age_ms": 1440.0,
            "localizer_health": "RECOVERED",
        }
    gateway._all_modules = {"CmdVelMux": FakeMux()}

    payload = build_navigation_status(gateway)

    assert payload["localization"]["algorithm_healthy"] is True
    assert payload["localization"]["pose_fresh"] is True
    assert payload["localization"]["pose_freshness"] == "fresh"
    assert "pose_stale" not in payload["reason_codes"]
    assert payload["can_accept_goal"] is True
    assert payload["readiness"]["blockers"] == []
    assert payload["readiness"]["can_accept_goal"] is True
    assert payload["readiness"]["can_execute_autonomy"] is True

    session = gateway._session_snapshot()
    assert session["localizer_ready"] is True
    assert session["pose_fresh"] is True
    assert session["pose_freshness"] == "fresh"


def test_navigation_status_blocks_goal_when_map_artifact_gate_fails():
    from gateway.gateway_module import GatewayModule
    from gateway.services.runtime_status import build_navigation_status

    class FakeMux:
        def health(self):
            return {"active_source": "none", "sources": {}}

    class FakeNavigation:
        def get_navigation_status(self):
            return json.dumps(
                {
                    "state": "IDLE",
                    "planning_frame_id": "map",
                    "odom_frame_id": "map",
                    "costmap_frame_id": "map",
                    "map_artifact_gate": {
                        "required": True,
                        "ok": False,
                        "blockers": ["metadata.json missing"],
                    },
                }
            )

    gateway = GatewayModule()
    gateway._session_mode = "navigating"
    gateway._icp_quality = 0.03
    with gateway._state_lock:
        gateway._odom = {"x": 0.0, "frame_id": "map"}
        gateway._mission = {"state": "IDLE"}
        gateway._localization_status = {
            "state": "TRACKING",
            "confidence": 0.9,
            "degeneracy": "NONE",
            "icp_fitness": 0.028,
            "localizer_health": "RECOVERED",
        }
    gateway._all_modules = {
        "NavigationModule": FakeNavigation(),
        "CmdVelMux": FakeMux(),
    }

    payload = build_navigation_status(gateway)

    assert payload["can_accept_goal"] is False
    assert payload["readiness"]["map_artifacts_ok"] is False
    assert "map_artifact_gate_failed" in payload["readiness"]["blockers"]
    assert payload["readiness"]["map_artifact_gate"]["blockers"] == [
        "metadata.json missing"
    ]


def test_navigation_status_treats_mild_degeneracy_as_advisory():
    from gateway.gateway_module import GatewayModule
    from gateway.services.runtime_status import build_navigation_status

    class FakeMux:
        def health(self):
            return {"active_source": "none", "sources": {}}

    gateway = GatewayModule()
    gateway._session_mode = "navigating"
    gateway._icp_quality = 0.0
    with gateway._state_lock:
        gateway._odom = {"x": 0.0}
        gateway._mission = {"state": "IDLE", "speed_scale": 0.7}
        gateway._localization_status = {
            "state": "TRACKING",
            "confidence": 0.7,
            "degeneracy": "MILD",
            "health_source": "odom_map_cloud",
            "pose_fresh": True,
            "map_cloud_fresh": True,
            "icp_fitness": 0.0,
            "odom_age_ms": 150.0,
            "cloud_age_ms": 120.0,
            "localizer_health": "LIO_TRACKING",
        }
    gateway._all_modules = {"CmdVelMux": FakeMux()}

    payload = build_navigation_status(gateway)

    assert payload["can_accept_goal"] is True
    assert payload["localization"]["ready"] is True
    assert payload["localization"]["degraded"] is False
    assert payload["localization"]["degeneracy"] == "MILD"
    assert payload["readiness"]["blockers"] == []
    assert payload["readiness"]["advisories"] == ["localization_mild_degeneracy"]
    assert payload["reason_codes"] == ["localization_mild_degeneracy"]


def test_navigation_status_uses_localizer_health_fitness_when_icp_quality_is_zero():
    from gateway.gateway_module import GatewayModule
    from gateway.services.runtime_status import build_localization_status, build_navigation_status

    class FakeMux:
        def health(self):
            return {"active_source": "none", "sources": {}}

    gateway = GatewayModule()
    gateway._session_mode = "navigating"
    gateway._icp_quality = 0.0
    with gateway._state_lock:
        gateway._odom = {"x": 0.0}
        gateway._mission = {"state": "IDLE"}
        gateway._localization_status = {
            "state": "TRACKING",
            "confidence": 0.92,
            "degeneracy": "NONE",
            "icp_fitness": 0.0,
            "odom_age_ms": 150.0,
            "cloud_age_ms": 140.0,
            "localizer_health": "RECOVERED",
            "localizer_health_source": "localizer_health_topic",
            "localizer_health_fitness": 0.0223,
        }
    gateway._all_modules = {"CmdVelMux": FakeMux()}

    localization = build_localization_status(gateway)
    navigation = build_navigation_status(gateway)
    session = gateway._session_snapshot()

    assert localization["state"] == "ready"
    assert localization["ready"] is True
    assert localization["algorithm_healthy"] is True
    assert localization["reasons"] == []
    assert navigation["can_accept_goal"] is True
    assert navigation["reason_codes"] == []
    assert session["localizer_ready"] is True


def test_localizer_health_topic_recovered_marks_gateway_ready_when_icp_quality_is_zero():
    from gateway.gateway_module import GatewayModule
    from gateway.services.runtime_status import build_localization_status

    gateway = GatewayModule()
    gateway._session_mode = "navigating"
    gateway._icp_quality = 0.0
    with gateway._state_lock:
        gateway._odom = {"x": 0.0}
        gateway._localization_status = {
            "backend": "localizer",
            "state": "TRACKING",
            "confidence": 0.91,
            "degeneracy": "NONE",
            "icp_fitness": 0.0,
            "odom_age_ms": 120.0,
            "cloud_age_ms": 90.0,
            "localizer_health": "RECOVERED",
            "localizer_health_source": "localizer_health_topic",
            "localizer_health_fitness": 0.0215,
            "relocalization_supported": True,
            "saved_map_relocalization_supported": True,
            "restart_recovery_supported": True,
            "recovery_method": "relocalize_service",
            "relocalization_state": "idle",
        }

    payload = build_localization_status(gateway)

    assert payload["state"] == "ready"
    assert payload["ready"] is True
    assert payload["algorithm_healthy"] is True
    assert payload["backend"] == "localizer"
    assert payload["localizer_health"] == "RECOVERED"
    assert payload["localizer_health_source"] == "localizer_health_topic"
    assert payload["localizer_health_fitness"] == 0.0215
    assert payload["relocalization_supported"] is True
    assert payload["saved_map_relocalization_supported"] is True
    assert payload["restart_recovery_supported"] is True
    assert payload["recovery_method"] == "relocalize_service"
    assert payload["relocalization_state"] == "idle"
    assert payload["reasons"] == []


def test_navigation_status_blocks_ready_when_map_cloud_is_stale():
    from gateway.gateway_module import GatewayModule
    from gateway.services.runtime_status import build_localization_status, build_navigation_status

    class FakeMux:
        def health(self):
            return {"active_source": "none", "sources": {}}

    gateway = GatewayModule()
    gateway._session_mode = "navigating"
    gateway._icp_quality = 0.0
    with gateway._state_lock:
        gateway._odom = {"x": 0.0}
        gateway._mission = {"state": "IDLE"}
        gateway._localization_status = {
            "state": "TRACKING",
            "confidence": 0.92,
            "degeneracy": "NONE",
            "icp_fitness": 0.0,
            "odom_age_ms": 150.0,
            "cloud_age_ms": 140.0,
            "map_cloud_fresh": False,
            "localizer_health": "RECOVERED",
            "localizer_health_source": "localizer_health_topic",
            "localizer_health_fitness": 0.0223,
        }
    gateway._all_modules = {"CmdVelMux": FakeMux()}

    localization = build_localization_status(gateway)
    navigation = build_navigation_status(gateway)
    session = gateway._session_snapshot()

    assert localization["state"] == "initializing"
    assert localization["ready"] is False
    assert localization["algorithm_healthy"] is False
    assert localization["reasons"] == ["localizer_not_ready"]
    assert navigation["can_accept_goal"] is False
    assert "localization_initializing" in navigation["reason_codes"]
    assert session["localizer_ready"] is False


def test_navigation_status_blocks_goal_when_super_lio_recovery_signal_is_active():
    from gateway.gateway_module import GatewayModule
    from gateway.services.runtime_status import build_localization_status, build_navigation_status

    class FakeMux:
        def health(self):
            return {"active_source": "none", "sources": {}}

    gateway = GatewayModule()
    gateway._session_mode = "navigating"
    gateway._icp_quality = 0.0
    with gateway._state_lock:
        gateway._odom = {"x": 0.0}
        gateway._mission = {"state": "IDLE"}
        gateway._localization_status = {
            "backend": "super_lio",
            "state": "TRACKING",
            "confidence": 0.92,
            "health_source": "odom_map_cloud",
            "pose_fresh": True,
            "map_cloud_fresh": True,
            "recovery_signal": "LOC_DIVERGED",
            "recovery_action": "restart_super_lio",
            "localizer_health": "LIO_TRACKING",
            "odom_age_ms": 120.0,
            "cloud_age_ms": 80.0,
        }
    gateway._all_modules = {"CmdVelMux": FakeMux()}

    localization = build_localization_status(gateway)
    navigation = build_navigation_status(gateway)

    assert localization["state"] == "degraded"
    assert localization["ready"] is False
    assert localization["algorithm_healthy"] is False
    assert localization["reasons"] == ["recovery_signal:loc_diverged"]
    assert localization["map_save_source"] == "live_map_cloud_snapshot"
    assert navigation["can_accept_goal"] is False
    assert "localization_recovery_active" in navigation["reason_codes"]
    assert "localization_recovery_active" in navigation["readiness"]["blockers"]


def test_goal_route_rejects_stale_localization_without_publishing():
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import GoalRequest
    from gateway.schemas import GatewayErrorResponse

    gateway = GatewayModule()
    gateway._session_mode = "navigating"
    gateway._icp_quality = 0.03
    with gateway._state_lock:
        gateway._odom = {"x": 0.0}
        gateway._mission = {"state": "IDLE"}
        gateway._localization_status = {
            "state": "TRACKING",
            "confidence": 0.28,
            "degeneracy": "NONE",
            "icp_fitness": 0.028,
            "odom_age_ms": 2500.0,
            "localizer_health": "RECOVERED",
        }
    sent_goals = []
    gateway.goal_pose._add_callback(sent_goals.append)

    response = asyncio.run(
        _endpoint(gateway, "/api/v1/goal")(
            GoalRequest(
                x=1.0,
                y=2.0,
                request_id="stale-goal",
                client_id="web",
            )
        )
    )
    payload = _payload(response)
    model = GatewayErrorResponse.model_validate(payload)

    assert response.status_code == 409
    assert model.error == "navigation_not_ready"
    assert model.command is not None
    assert model.command.name == "goal"
    assert model.command.accepted is False
    assert model.detail["blockers"] == ["pose_stale"]
    assert sent_goals == []


def test_goal_route_accepts_ready_navigation_goal():
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import GoalRequest
    from gateway.schemas import ControlCommandResponse

    class FakeNavigation:
        def __init__(self) -> None:
            self.calls = []

        def preview_plan(self, x, y, z):
            self.calls.append((x, y, z))
            return {"feasible": True}

    gateway = GatewayModule()
    gateway._session_mode = "navigating"
    gateway._icp_quality = 0.03
    with gateway._state_lock:
        gateway._odom = {"x": 0.0}
        gateway._mission = {"state": "IDLE"}
        gateway._localization_status = {
            "state": "TRACKING",
            "confidence": 0.28,
            "degeneracy": "NONE",
            "icp_fitness": 0.028,
            "odom_age_ms": 250.0,
            "localizer_health": "RECOVERED",
        }
    nav = FakeNavigation()
    gateway.on_system_modules({"NavigationModule": nav})
    sent_goals = []
    gateway.goal_pose._add_callback(sent_goals.append)

    payload = asyncio.run(
        _endpoint(gateway, "/api/v1/goal")(
            GoalRequest(
                x=1.0,
                y=2.0,
                request_id="ready-goal",
                client_id="web",
            )
        )
    )
    model = ControlCommandResponse.model_validate(payload)

    assert model.ok is True
    assert model.command.accepted is True
    assert model.goal == [1.0, 2.0, 0.0]
    assert nav.calls == [(1.0, 2.0, 0.0)]
    assert len(sent_goals) == 1


def test_navigation_status_reports_teleop_preemption_for_active_mission():
    from gateway.gateway_module import GatewayModule
    from gateway.services.runtime_status import build_navigation_status

    class FakeMux:
        def health(self):
            return {
                "active_source": "teleop",
                "sources": {
                    "teleop": {
                        "active": True,
                        "priority": 100,
                        "age_ms": 20,
                    },
                    "path_follower": {
                        "active": True,
                        "priority": 40,
                        "age_ms": 25,
                    },
                },
            }

    gateway = GatewayModule()
    with gateway._state_lock:
        gateway._odom = {"x": 1.0}
        gateway._mode = "autonomous"
        gateway._mission = {
            "state": "EXECUTING",
            "wp_index": 1,
            "wp_total": 4,
            "speed_scale": 1.0,
        }
        gateway._localization_status = {"state": "TRACKING", "confidence": 0.9}
    gateway._all_modules = {"CmdVelMux": FakeMux()}

    payload = build_navigation_status(gateway)

    assert payload["control"]["active_cmd_source"] == "teleop"
    assert payload["control"]["command_owner"] == "teleop"
    assert payload["control"]["source_category"] == "manual"
    assert payload["control"]["manual_override"] is True
    assert payload["control"]["preempting_autonomy"] is True
    assert payload["control"]["active_source"]["priority"] == 100
    assert "control_preempted_by_teleop" in payload["reason_codes"]
    assert "control_preempted_by_teleop" in payload["readiness"]["advisories"]


def test_navigation_status_route_returns_stable_schema():
    from gateway.gateway_module import GatewayModule

    gateway = GatewayModule()
    with gateway._state_lock:
        gateway._mission = {"state": "IDLE"}

    payload = asyncio.run(_endpoint(gateway, "/api/v1/navigation/status")())

    assert payload["schema_version"] == 1
    assert payload["state"] == "IDLE"
    assert payload["path"]["endpoint"] == "/api/v1/path"
    assert payload["control"]["active_cmd_source"] == "unknown"
    assert "odometry_missing" in payload["reason_codes"]
    assert payload["readiness"]["blockers"] == [
        "odometry_missing",
        "navigation_session_inactive",
    ]


def test_navigation_status_routes_pass_fastapi_response_validation():
    from fastapi.testclient import TestClient

    from gateway.gateway_module import GatewayModule

    gateway = GatewayModule()
    gateway.setup()
    with gateway._state_lock:
        gateway._mission = {"state": "IDLE"}

    client = TestClient(gateway._app)
    for path in ("/api/v1/navigation/status", "/api/v1/navigation"):
        response = client.get(path)

        assert response.status_code == 200
        payload = response.json()
        assert payload["schema_version"] == 1
        assert payload["state"] == "IDLE"
        assert payload["frames"]["planning_frame_id"] == "map"
        assert payload["target"] == {
            "goal": None,
            "current_waypoint": None,
            "distance_to_goal_m": None,
            "active_waypoint_distance_m": None,
            "remaining_waypoints": None,
        }
        assert payload["motion"]["active_cmd_source"] == "unknown"
        assert payload["feedback"]["next_action"] == "resolve_blockers"
        assert payload["feedback"]["blockers"] == [
            "odometry_missing",
            "navigation_session_inactive",
        ]


def test_drift_watchdog_restores_idle_running_localization_services(monkeypatch):
    import core.service_manager as service_manager
    import gateway.gateway_module as gateway_module
    from gateway.gateway_module import GatewayModule

    class FakeServiceManager:
        def __init__(self):
            self.running = {"slam", "localizer"}
            self.calls: list[tuple[str, tuple[str, ...]]] = []

        def is_running(self, service: str) -> bool:
            return service in self.running

        def stop(self, *services: str) -> None:
            self.calls.append(("stop", services))
            self.running.difference_update(services)

        def ensure(self, *services: str) -> None:
            self.calls.append(("ensure", services))
            self.running.update(services)

        def wait_ready(self, *services: str, timeout: float = 15.0) -> bool:
            self.calls.append(("wait_ready", services))
            return True

    fake = FakeServiceManager()
    monkeypatch.setattr(service_manager, "get_service_manager", lambda: fake)
    monkeypatch.setattr(gateway_module.time, "sleep", lambda _: None)

    gateway = GatewayModule()
    gateway._drift_restart_delay_s = 0.0
    gateway._session_mode = "idle"
    with gateway._state_lock:
        gateway._odom = {"x": 999.0}
        gateway._odom_timestamps.append(123.0)

    gateway._drift_restart_do_restart(xy=999.0, y_abs=0.0, v=0.0)

    assert (
        "stop",
        ("slam", "slam_pgo", "localizer", "super_lio", "super_lio_relocation"),
    ) in fake.calls
    assert ("ensure", ("slam", "localizer")) in fake.calls
    assert ("wait_ready", ("slam", "localizer")) in fake.calls
    assert gateway._odom == {}
    assert gateway._odom_timestamps == []


def test_drift_watchdog_restart_noops_after_shutdown(monkeypatch):
    import core.service_manager as service_manager
    from gateway.gateway_module import GatewayModule

    called = False

    def fail_if_called():
        nonlocal called
        called = True
        raise AssertionError("service manager should not be touched during shutdown")

    monkeypatch.setattr(service_manager, "get_service_manager", fail_if_called)

    gateway = GatewayModule()
    gateway._stop_event.set()

    gateway._drift_restart_do_restart(xy=999.0, y_abs=0.0, v=0.0)

    assert called is False


def test_drift_watchdog_restores_idle_running_super_lio_services(monkeypatch):
    import core.service_manager as service_manager
    import gateway.gateway_module as gateway_module
    from gateway.gateway_module import GatewayModule

    class FakeServiceManager:
        def __init__(self):
            self.running = {"super_lio"}
            self.calls: list[tuple[str, tuple[str, ...]]] = []

        def is_running(self, service: str) -> bool:
            return service in self.running

        def stop(self, *services: str) -> None:
            self.calls.append(("stop", services))
            self.running.difference_update(services)

        def ensure(self, *services: str) -> None:
            self.calls.append(("ensure", services))
            self.running.update(services)

        def wait_ready(self, *services: str, timeout: float = 15.0) -> bool:
            self.calls.append(("wait_ready", services))
            return True

    fake = FakeServiceManager()
    monkeypatch.setattr(service_manager, "get_service_manager", lambda: fake)
    monkeypatch.setattr(gateway_module.time, "sleep", lambda _: None)

    gateway = GatewayModule()
    gateway._drift_restart_delay_s = 0.0
    gateway._session_mode = "idle"

    gateway._drift_restart_do_restart(xy=999.0, y_abs=0.0, v=0.0)

    assert (
        "stop",
        ("slam", "slam_pgo", "localizer", "super_lio", "super_lio_relocation"),
    ) in fake.calls
    assert ("ensure", ("lidar", "super_lio")) in fake.calls
    assert ("wait_ready", ("lidar", "super_lio")) in fake.calls


def test_drift_watchdog_restores_idle_running_super_lio_relocation_services(
    monkeypatch,
):
    import core.service_manager as service_manager
    import gateway.gateway_module as gateway_module
    from gateway.gateway_module import GatewayModule

    class FakeServiceManager:
        def __init__(self):
            self.running = {"super_lio_relocation"}
            self.calls: list[tuple[str, tuple[str, ...]]] = []

        def is_running(self, service: str) -> bool:
            return service in self.running

        def stop(self, *services: str) -> None:
            self.calls.append(("stop", services))
            self.running.difference_update(services)

        def ensure(self, *services: str) -> None:
            self.calls.append(("ensure", services))
            self.running.update(services)

        def wait_ready(self, *services: str, timeout: float = 15.0) -> bool:
            self.calls.append(("wait_ready", services))
            return True

    fake = FakeServiceManager()
    monkeypatch.setattr(service_manager, "get_service_manager", lambda: fake)
    monkeypatch.setattr(gateway_module.time, "sleep", lambda _: None)

    gateway = GatewayModule()
    gateway._drift_restart_delay_s = 0.0
    gateway._session_mode = "idle"

    gateway._drift_restart_do_restart(xy=999.0, y_abs=0.0, v=0.0)

    assert (
        "stop",
        ("slam", "slam_pgo", "localizer", "super_lio", "super_lio_relocation"),
    ) in fake.calls
    assert ("ensure", ("lidar", "super_lio_relocation")) in fake.calls
    assert ("wait_ready", ("lidar", "super_lio_relocation")) in fake.calls


def test_runtime_dataflow_route_exposes_module_first_observability(monkeypatch):
    from core.runtime_interface import TOPICS
    from core.msgs.nav import Odometry
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import RuntimeDataflowResponse

    monkeypatch.setenv("LINGTU_PROFILE", "nav")
    monkeypatch.setenv("LINGTU_ENDPOINT", "thunder_field")
    monkeypatch.setenv("LINGTU_DATA_SOURCE", "thunder_field")
    monkeypatch.setenv("LINGTU_RUNTIME_CONTRACT", "thunder_field")
    monkeypatch.setenv("LINGTU_COMMAND_SINK", "hardware_driver_after_cmd_vel_mux")
    monkeypatch.setenv("LINGTU_SIMULATION_ONLY", "0")

    gateway = GatewayModule()
    endpoint = _endpoint(gateway, "/api/v1/runtime/dataflow")
    payload = asyncio.run(endpoint())
    RuntimeDataflowResponse.model_validate(payload)
    initial_payload = payload

    assert payload["schema_version"] == 1
    assert payload["runtime_contract"] == "thunder_field"
    assert payload["runtime_boundary"]["runtime_contract"] == "thunder_field"
    assert payload["ros2_topic_required"] is False
    assert payload["transport_layers"]["module_port_bus"]["primary"] is True
    assert payload["transport_layers"]["ros2_adapter"]["primary"] is False
    assert payload["control_boundary"]["arbitrary_publish_supported"] is False
    assert payload["control_boundary"]["policy"] == "whitelisted_gateway_commands_only"

    gateway_ports = payload["module_ports"]["GatewayModule"]
    assert "odometry" in gateway_ports["ports_in"]
    assert "cmd_vel" in gateway_ports["ports_out"]

    topics = {item["topic"]: item for item in payload["topics"]}
    assert TOPICS.odometry in topics
    assert TOPICS.lidar_scan in topics
    assert TOPICS.cmd_vel in topics

    odometry_observability = topics[TOPICS.odometry]["observability"]
    assert odometry_observability["ros2_topic_required"] is False
    assert "module_port_bus" in odometry_observability["observable_via"]
    assert "gateway_sse" in odometry_observability["observable_via"]
    assert "gateway_rest" in odometry_observability["observable_via"]
    assert odometry_observability["live_module_samples"] is False

    gateway.odometry._deliver(Odometry())
    payload = asyncio.run(endpoint())
    topics = {item["topic"]: item for item in payload["topics"]}
    odometry_observability = topics[TOPICS.odometry]["observability"]
    assert odometry_observability["live_module_samples"] is True
    assert odometry_observability["has_fresh_module_sample"] is True
    assert odometry_observability["module_port_candidates"][0]["msg_count"] > 0
    assert odometry_observability["module_port_candidates"][0]["stale_ms"] >= 0

    lidar_flow = {
        stage["name"]: stage
        for stage in topics[TOPICS.lidar_scan]["data_flow_stages"]
    }
    assert "endpoint_adapter" in lidar_flow
    assert "input" in lidar_flow["endpoint_adapter"]["roles"]

    stages = {stage["name"]: stage for stage in initial_payload["stage_evidence"]}
    assert "global_planning" in stages
    assert stages["global_planning"]["owner"] == "lingtu_navigation_or_pct"
    assert TOPICS.odometry in stages["global_planning"]["inputs"]
    assert TOPICS.global_path in stages["global_planning"]["outputs"]
    assert TOPICS.odometry in stages["global_planning"]["not_live_inputs"]
    odom_stage_input = next(
        item
        for item in stages["global_planning"]["input_evidence"]
        if item["token"] == TOPICS.odometry
    )
    assert odom_stage_input["observable"] is True
    assert odom_stage_input["live"] is False
    assert odom_stage_input["reason"] == "metadata_only"
    assert stages["command_boundary"]["output_evidence"][0]["kind"] == "runtime_boundary"
    assert stages["command_boundary"]["output_evidence"][0]["reason"] == "runtime_boundary_declared"

    cmd_vel_communication = topics[TOPICS.cmd_vel]["communication"]
    assert cmd_vel_communication["allowed"] is True
    assert cmd_vel_communication["arbitrary_publish_supported"] is False
    assert {
        interface["path"] for interface in cmd_vel_communication["interfaces"]
    } == {"/api/v1/cmd_vel", "/api/v1/stop"}


def test_runtime_dataflow_route_validates_active_saved_tomogram_artifact(
    monkeypatch,
    tmp_path,
):
    from core.runtime_interface import TOPICS
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import RuntimeDataflowResponse

    map_root = tmp_path / "maps"
    _write_active_same_source_tomogram(map_root)
    monkeypatch.setenv("NAV_MAP_DIR", str(map_root))
    monkeypatch.setenv("LINGTU_RUNTIME_CONTRACT", "thunder_field")

    gateway = GatewayModule()
    endpoint = _endpoint(gateway, "/api/v1/runtime/dataflow")
    payload = asyncio.run(endpoint())
    RuntimeDataflowResponse.model_validate(payload)

    stages = {stage["name"]: stage for stage in payload["stage_evidence"]}
    global_stage = stages["global_planning"]
    tomogram_input = next(
        item
        for item in global_stage["input_evidence"]
        if item["token"] == "artifact:tomogram"
    )

    assert "artifact:tomogram" in global_stage["inputs"]
    assert "artifact:tomogram" not in global_stage["missing_inputs"]
    assert tomogram_input["kind"] == "artifact"
    assert tomogram_input["observable"] is True
    assert tomogram_input["live"] is False
    assert tomogram_input["reason"] == "saved_map_artifact_ok"
    assert tomogram_input["artifact_gate"]["ok"] is True
    assert tomogram_input["artifact_gate"]["artifacts"]["tomogram"]["sha256_ok"] is True
    assert (
        tomogram_input["artifact_gate"]["artifacts"]["tomogram"][
            "source_map_sha256_matches_map"
        ]
        is True
    )
    assert tomogram_input["artifact_gate"]["ros2_topic_required"] is False
    assert TOPICS.global_path in global_stage["outputs"]


def test_runtime_dataflow_route_marks_missing_active_tomogram_artifact(
    monkeypatch,
    tmp_path,
):
    from gateway.gateway_module import GatewayModule

    map_root = tmp_path / "maps"
    map_root.mkdir()
    monkeypatch.setenv("NAV_MAP_DIR", str(map_root))
    monkeypatch.setenv("LINGTU_RUNTIME_CONTRACT", "thunder_field")

    gateway = GatewayModule()
    endpoint = _endpoint(gateway, "/api/v1/runtime/dataflow")
    payload = asyncio.run(endpoint())

    stages = {stage["name"]: stage for stage in payload["stage_evidence"]}
    global_stage = stages["global_planning"]
    tomogram_input = next(
        item
        for item in global_stage["input_evidence"]
        if item["token"] == "artifact:tomogram"
    )

    assert "artifact:tomogram" in global_stage["missing_inputs"]
    assert tomogram_input["kind"] == "artifact"
    assert tomogram_input["observable"] is False
    assert tomogram_input["live"] is False
    assert tomogram_input["reason"] == "saved_map_artifact_missing_or_invalid"
    assert tomogram_input["artifact_gate"]["ok"] is False
    assert tomogram_input["artifact_gate"]["ros2_topic_required"] is False
    assert "active map artifact directory missing" in tomogram_input["artifact_gate"][
        "blockers"
    ]


def test_runtime_dataflow_route_is_read_only_for_module_ports(monkeypatch):
    from gateway.gateway_module import GatewayModule

    monkeypatch.setenv("LINGTU_RUNTIME_CONTRACT", "thunder_field")

    gateway = GatewayModule()
    endpoint = _endpoint(gateway, "/api/v1/runtime/dataflow")
    before = {name: port.msg_count for name, port in gateway.ports_out.items()}

    payload = asyncio.run(endpoint())

    after = {name: port.msg_count for name, port in gateway.ports_out.items()}
    assert payload["ros2_topic_required"] is False
    assert after == before


def test_runtime_dataflow_route_does_not_mark_stale_port_as_live(
    monkeypatch,
):
    import gateway.services.runtime_dataflow as dataflow_mod
    from core.msgs.nav import Odometry
    from core.runtime_interface import TOPICS
    from gateway.gateway_module import GatewayModule

    monkeypatch.setenv("LINGTU_RUNTIME_CONTRACT", "thunder_field")
    monkeypatch.setattr(dataflow_mod, "LIVE_MODULE_SAMPLE_STALE_MS", -1.0)

    gateway = GatewayModule()
    endpoint = _endpoint(gateway, "/api/v1/runtime/dataflow")
    gateway.odometry._deliver(Odometry())

    payload = asyncio.run(endpoint())
    topics = {item["topic"]: item for item in payload["topics"]}
    odometry_observability = topics[TOPICS.odometry]["observability"]

    assert odometry_observability["module_port_candidates"][0]["msg_count"] > 0
    assert odometry_observability["has_fresh_module_sample"] is False
    assert odometry_observability["live_module_samples"] is False


def test_runtime_dataflow_topic_route_answers_one_stream_without_ros2(monkeypatch):
    from core.msgs.nav import Odometry
    from core.runtime_interface import TOPICS
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import RuntimeDataflowTopicDetailResponse

    monkeypatch.setenv("LINGTU_RUNTIME_CONTRACT", "thunder_field")

    gateway = GatewayModule()
    endpoint = _endpoint(gateway, "/api/v1/runtime/dataflow/topic")
    gateway.odometry._deliver(Odometry())

    payload = asyncio.run(endpoint(topic="odometry"))
    RuntimeDataflowTopicDetailResponse.model_validate(payload)

    assert payload["ok"] is True
    assert payload["selector"] == "odometry"
    assert payload["topic"]["topic"] == TOPICS.odometry
    assert payload["inspection"]["live"] is True
    assert payload["inspection"]["observation_level"] == "fresh_module_sample"
    assert payload["inspection"]["ros2_topic_required"] is False
    assert payload["inspection"]["arbitrary_publish_supported"] is False
    assert payload["inspection"]["payload_available"] is True
    assert {
        channel["transport"]
        for channel in payload["inspection"]["payload_interfaces"]
    } >= {"gateway_rest", "gateway_sse"}
    assert payload["inspection"]["stream_interfaces"] == [
        {
            "transport": "gateway_sse",
            "path": "/api/v1/events",
            "query": {"topic": TOPICS.odometry},
            "event_type": "odometry",
        }
    ]
    assert payload["inspection"]["communicate"] is False


def test_runtime_dataflow_subscribe_route_returns_read_only_sse_plan(monkeypatch):
    from core.msgs.nav import Odometry
    from core.runtime_interface import TOPICS
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import (
        RuntimeDataflowSubscribeRequest,
        RuntimeDataflowSubscribeResponse,
    )

    monkeypatch.setenv("LINGTU_RUNTIME_CONTRACT", "thunder_field")

    gateway = GatewayModule()
    endpoint = _endpoint(gateway, "/api/v1/runtime/dataflow/subscribe")
    gateway.odometry._deliver(Odometry())

    payload = asyncio.run(
        endpoint(RuntimeDataflowSubscribeRequest(selector="odometry"))
    )
    RuntimeDataflowSubscribeResponse.model_validate(payload)

    assert payload["ok"] is True
    assert payload["read_only"] is True
    assert payload["ros2_topic_required"] is False
    assert payload["arbitrary_publish_supported"] is False
    assert payload["publishes"] == []
    assert payload["selector"] == "odometry"
    assert payload["topic"] == TOPICS.odometry
    assert payload["event_types"] == ["odometry"]
    assert payload["stream_url"] == "/api/v1/events?topic=%2Fnav%2Fodometry"
    assert payload["stream_interfaces"] == [
        {
            "transport": "gateway_sse",
            "path": "/api/v1/events",
            "query": {"topic": TOPICS.odometry},
            "event_type": "odometry",
        }
    ]
    assert payload["blockers"] == []


@pytest.mark.skipif(not _NUMPY_IMPORT_SAFE, reason=NUMPY_UNSAFE_REASON)
def test_runtime_dataflow_exposes_traversable_frontier_candidates_read_only(
    monkeypatch,
):
    import numpy as np

    from core.msgs.geometry import Pose
    from core.msgs.nav import Odometry
    from core.runtime_interface import TOPICS
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import (
        RuntimeDataflowResponse,
        RuntimeDataflowSubscribeRequest,
        RuntimeDataflowSubscribeResponse,
        RuntimeDataflowTopicDetailResponse,
    )
    from nav.traversable_frontier_module import TraversableFrontierModule
    # ^ Cross-layer: gateway test imports from nav/ for runtime dataflow testing.
    #   Acceptable: test needs a concrete nav module to exercise the
    #   RuntimeDataflow endpoint with real module state.  Gateway tests
    #   serve as integration tests for the full stack through the HTTP layer.

    monkeypatch.setenv("LINGTU_RUNTIME_CONTRACT", "thunder_field")

    gateway = GatewayModule()
    gateway.setup()
    routes = {
        route.path: route.endpoint
        for route in gateway._app.routes
        if hasattr(route, "endpoint")
    }
    module = TraversableFrontierModule(
        min_frontier_size=1,
        safe_distance=1.0,
        max_slope_deg=30.0,
        max_frontier_cost=80.0,
    )
    module.setup()
    gateway._all_modules = {
        "GatewayModule": gateway,
        "TraversableFrontierModule": module,
    }
    module.traversable_frontiers._add_callback(gateway.traversable_frontiers._deliver)
    module.frontier_candidate._add_callback(gateway.frontier_candidate._deliver)

    grid = np.full((9, 9), -1, dtype=np.int16)
    grid[2:7, 2:7] = 0
    cost = np.zeros_like(grid, dtype=np.float32)
    slope = np.full_like(grid, 5.0, dtype=np.float32)
    clearance = np.full_like(grid, 2.0, dtype=np.float32)
    elevation = np.full_like(grid, 0.25, dtype=np.float32)
    payload = {
        "grid": grid,
        "resolution": 1.0,
        "origin": [-4.0, -4.0],
        "origin_x": -4.0,
        "origin_y": -4.0,
        "width": 9,
        "height": 9,
        "frame_id": "map",
    }

    module.odometry._deliver(Odometry(pose=Pose(0.0, 0.0, 0.0), frame_id="map"))
    module.exploration_grid._deliver(payload)
    module.costmap._deliver(payload)
    module.fused_cost._deliver({**payload, "grid": cost})
    module.slope_grid._deliver({**payload, "grid": slope})
    module.esdf_field._deliver({**payload, "distance_field": clearance})
    module.elevation_map._deliver({**payload, "max_z": elevation})
    result = json.loads(module.refresh_candidates())

    assert result["command_published"] is False
    assert module.exploration_goal.msg_count == 0
    assert gateway.goal_pose.msg_count == 0
    assert gateway.cmd_vel.msg_count == 0

    snapshot = asyncio.run(routes["/api/v1/runtime/dataflow"]())
    RuntimeDataflowResponse.model_validate(snapshot)
    topics = {item["topic"]: item for item in snapshot["topics"]}
    stages = {item["name"]: item for item in snapshot["stage_evidence"]}
    assert TOPICS.traversable_frontiers in topics
    assert TOPICS.frontier_candidate in topics
    assert "traversable_frontier_preview" in stages
    preview_stage = stages["traversable_frontier_preview"]
    assert preview_stage["outputs"] == [
        TOPICS.traversable_frontiers,
        TOPICS.frontier_candidate,
    ]
    assert "module:TraversableFrontierModule.fused_cost" in preview_stage["inputs"]
    assert "module:TraversableFrontierModule.slope_grid" in preview_stage["inputs"]
    assert "module:TraversableFrontierModule.esdf_field" in preview_stage["inputs"]
    assert "module:TraversableFrontierModule.elevation_map" in preview_stage["inputs"]
    evidence_by_token = {
        item["token"]: item
        for item in preview_stage["input_evidence"]
    }
    fused_evidence = evidence_by_token["module:TraversableFrontierModule.fused_cost"]
    assert fused_evidence["kind"] == "module_port"
    assert fused_evidence["live"] is True
    assert fused_evidence["module_ports"][0]["module"] == "TraversableFrontierModule"
    assert preview_stage["observable"] is True
    assert preview_stage["live"] is True
    assert preview_stage["missing_inputs"] == []
    assert topics[TOPICS.frontier_candidate]["communication"]["allowed"] is False
    assert (
        topics[TOPICS.frontier_candidate]["communication"][
            "arbitrary_publish_supported"
        ]
        is False
    )
    assert (
        snapshot["control_boundary"]["arbitrary_publish_supported"]
        is False
    )

    detail = asyncio.run(
        routes["/api/v1/runtime/dataflow/topic"](topic="frontier_candidate")
    )
    RuntimeDataflowTopicDetailResponse.model_validate(detail)
    assert detail["ok"] is True
    assert detail["topic"]["topic"] == TOPICS.frontier_candidate
    assert detail["inspection"]["live"] is True
    assert detail["inspection"]["communicate"] is False
    assert detail["inspection"]["write_interfaces"] == []
    assert detail["inspection"]["latest_payload"]["source"] == "traversable_frontier"
    assert detail["inspection"]["latest_payload"]["preview"] is True
    assert detail["inspection"]["latest_payload"]["command_published"] is False
    assert detail["inspection"]["latest_payload"]["reachable_score"] > 0.0
    assert detail["inspection"]["stream_interfaces"] == [
        {
            "transport": "gateway_sse",
            "path": "/api/v1/events",
            "query": {"topic": TOPICS.frontier_candidate},
            "event_type": "frontier_candidate",
        }
    ]

    list_detail = asyncio.run(
        routes["/api/v1/runtime/dataflow/topic"](topic="traversable_frontiers")
    )
    RuntimeDataflowTopicDetailResponse.model_validate(list_detail)
    assert list_detail["ok"] is True
    assert list_detail["topic"]["topic"] == TOPICS.traversable_frontiers
    assert list_detail["inspection"]["live"] is True
    assert list_detail["inspection"]["communicate"] is False
    assert list_detail["inspection"]["write_interfaces"] == []
    assert isinstance(list_detail["inspection"]["latest_payload"], list)
    assert list_detail["inspection"]["latest_payload"]
    assert (
        list_detail["inspection"]["latest_payload"][0]["source"]
        == "traversable_frontier"
    )
    assert list_detail["inspection"]["latest_payload"][0]["command_published"] is False
    assert list_detail["inspection"]["stream_interfaces"] == [
        {
            "transport": "gateway_sse",
            "path": "/api/v1/events",
            "query": {"topic": TOPICS.traversable_frontiers},
            "event_type": "traversable_frontiers",
        }
    ]

    subscription = asyncio.run(
        routes["/api/v1/runtime/dataflow/subscribe"](
            RuntimeDataflowSubscribeRequest(selector="frontier_candidate")
        )
    )
    RuntimeDataflowSubscribeResponse.model_validate(subscription)
    assert subscription["ok"] is True
    assert subscription["read_only"] is True
    assert subscription["publishes"] == []
    assert subscription["arbitrary_publish_supported"] is False
    assert subscription["event_types"] == ["frontier_candidate"]
    assert (
        subscription["stream_url"]
        == "/api/v1/events?topic=%2Fnav%2Ffrontier_candidate"
    )


def test_runtime_dataflow_subscribe_route_rejects_unknown_selector_without_publish(
    monkeypatch,
):
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import RuntimeDataflowSubscribeRequest

    monkeypatch.setenv("LINGTU_RUNTIME_CONTRACT", "thunder_field")

    gateway = GatewayModule()
    endpoint = _endpoint(gateway, "/api/v1/runtime/dataflow/subscribe")

    payload = asyncio.run(
        endpoint(RuntimeDataflowSubscribeRequest(selector="not_a_stream"))
    )

    assert payload["ok"] is False
    assert payload["read_only"] is True
    assert payload["ros2_topic_required"] is False
    assert payload["arbitrary_publish_supported"] is False
    assert payload["publishes"] == []
    assert payload["stream_url"] == ""
    assert "runtime_topic_not_found" in payload["blockers"]


def test_runtime_dataflow_topic_route_accepts_canonical_stream_token(
    monkeypatch,
):
    from core.msgs.nav import Odometry
    from core.runtime_interface import TOPICS
    from gateway.gateway_module import GatewayModule

    monkeypatch.setenv("LINGTU_RUNTIME_CONTRACT", "thunder_field")

    gateway = GatewayModule()
    endpoint = _endpoint(gateway, "/api/v1/runtime/dataflow/topic")
    gateway.odometry._deliver(Odometry())

    payload = asyncio.run(endpoint(topic=TOPICS.odometry))

    assert payload["ok"] is True
    assert payload["selector"] == TOPICS.odometry
    assert payload["topic"]["topic"] == TOPICS.odometry
    assert payload["inspection"]["live"] is True
    assert payload["inspection"]["ros2_topic_required"] is False


def test_runtime_dataflow_topic_route_exposes_whitelisted_command_interfaces(
    monkeypatch,
):
    from core.runtime_interface import TOPICS
    from gateway.gateway_module import GatewayModule

    monkeypatch.setenv("LINGTU_RUNTIME_CONTRACT", "thunder_field")

    gateway = GatewayModule()
    endpoint = _endpoint(gateway, "/api/v1/runtime/dataflow/topic")

    payload = asyncio.run(endpoint(topic="/cmd_vel"))

    assert payload["ok"] is True
    assert payload["topic"]["topic"] == TOPICS.cmd_vel
    assert payload["inspection"]["communicate"] is True
    assert payload["inspection"]["arbitrary_publish_supported"] is False
    assert {item["path"] for item in payload["inspection"]["write_interfaces"]} == {
        "/api/v1/cmd_vel",
        "/api/v1/stop",
    }


@pytest.mark.parametrize(
    (
        "contract",
        "data_source",
        "endpoint",
        "command_sink",
        "simulation_only",
    ),
    [
        (
            "thunder_field",
            "thunder_field",
            "thunder_field",
            "hardware_driver_after_cmd_vel_mux",
            "0",
        ),
        (
            "mujoco_fastlio2_live",
            "mujoco_fastlio2_live",
            "mujoco_live",
            "mujoco_velocity_adapter",
            "1",
        ),
    ],
)
def test_runtime_dataflow_exposes_all_contract_streams_for_real_and_sim(
    monkeypatch,
    contract: str,
    data_source: str,
    endpoint: str,
    command_sink: str,
    simulation_only: str,
):
    from core.runtime_interface import runtime_data_flow_topics
    from gateway.gateway_module import GatewayModule

    monkeypatch.setenv("LINGTU_RUNTIME_CONTRACT", contract)
    monkeypatch.setenv("LINGTU_DATA_SOURCE", data_source)
    monkeypatch.setenv("LINGTU_ENDPOINT", endpoint)
    monkeypatch.setenv("LINGTU_COMMAND_SINK", command_sink)
    monkeypatch.setenv("LINGTU_SIMULATION_ONLY", simulation_only)

    gateway = GatewayModule()
    endpoint_fn = _endpoint(gateway, "/api/v1/runtime/dataflow")

    payload = asyncio.run(endpoint_fn())
    topics = {item["topic"]: item for item in payload["topics"]}

    assert set(runtime_data_flow_topics(contract)) <= set(topics)
    assert payload["runtime_contract"] == contract
    assert payload["ros2_topic_required"] is False
    for topic in topics.values():
        assert topic["inspection"]["ros2_topic_required"] is False
        assert topic["inspection"]["arbitrary_publish_supported"] is False
        assert topic["communication"]["arbitrary_publish_supported"] is False


def test_runtime_dataflow_topic_route_exposes_all_whitelisted_write_interfaces(
    monkeypatch,
):
    from core.runtime_interface import TOPICS
    from gateway.gateway_module import GatewayModule

    monkeypatch.setenv("LINGTU_RUNTIME_CONTRACT", "thunder_field")

    gateway = GatewayModule()
    endpoint = _endpoint(gateway, "/api/v1/runtime/dataflow/topic")
    expected = {
        TOPICS.goal_pose: {"/api/v1/goal", "/api/v1/navigate/click"},
        TOPICS.cmd_vel: {"/api/v1/cmd_vel", "/api/v1/stop"},
        TOPICS.stop: {"/api/v1/stop"},
        TOPICS.cancel: {"/api/v1/navigation/cancel"},
        TOPICS.semantic_instruction: {"/api/v1/instruction"},
    }

    for topic, paths in expected.items():
        payload = asyncio.run(endpoint(topic=topic))
        assert payload["ok"] is True, topic
        assert payload["topic"]["topic"] == topic
        assert payload["inspection"]["communicate"] is True
        assert payload["inspection"]["arbitrary_publish_supported"] is False
        assert {item["path"] for item in payload["inspection"]["write_interfaces"]} == paths


def test_runtime_dataflow_reflects_sim_to_real_runtime_switch_without_cache(
    monkeypatch,
):
    from gateway.gateway_module import GatewayModule

    gateway = GatewayModule()
    endpoint = _endpoint(gateway, "/api/v1/runtime/dataflow")

    monkeypatch.setenv("LINGTU_RUNTIME_CONTRACT", "mujoco_fastlio2_live")
    monkeypatch.setenv("LINGTU_DATA_SOURCE", "mujoco_fastlio2_live")
    monkeypatch.setenv("LINGTU_ENDPOINT", "mujoco_live")
    monkeypatch.setenv("LINGTU_COMMAND_SINK", "mujoco_velocity_adapter")
    monkeypatch.setenv("LINGTU_SIMULATION_ONLY", "1")
    sim_payload = asyncio.run(endpoint())

    monkeypatch.setenv("LINGTU_RUNTIME_CONTRACT", "thunder_field")
    monkeypatch.setenv("LINGTU_DATA_SOURCE", "thunder_field")
    monkeypatch.setenv("LINGTU_ENDPOINT", "thunder_field")
    monkeypatch.setenv("LINGTU_COMMAND_SINK", "hardware_driver_after_cmd_vel_mux")
    monkeypatch.setenv("LINGTU_SIMULATION_ONLY", "0")
    real_payload = asyncio.run(endpoint())

    assert sim_payload["runtime_contract"] == "mujoco_fastlio2_live"
    assert sim_payload["runtime_boundary"]["simulation_only"] is True
    assert sim_payload["runtime_boundary"]["command_sink"] == "mujoco_velocity_adapter"
    assert real_payload["runtime_contract"] == "thunder_field"
    assert real_payload["runtime_boundary"]["simulation_only"] is False
    assert (
        real_payload["runtime_boundary"]["command_sink"]
        == "hardware_driver_after_cmd_vel_mux"
    )
    sim_topics = {item["topic"] for item in sim_payload["topics"]}
    real_topics = {item["topic"] for item in real_payload["topics"]}
    assert "/points_raw" in sim_topics
    assert "/nav/lidar_scan" in real_topics
    assert sim_topics != real_topics


def test_runtime_dataflow_topic_route_reports_unknown_selector(monkeypatch):
    from gateway.gateway_module import GatewayModule

    monkeypatch.setenv("LINGTU_RUNTIME_CONTRACT", "thunder_field")

    gateway = GatewayModule()
    endpoint = _endpoint(gateway, "/api/v1/runtime/dataflow/topic")

    payload = asyncio.run(endpoint(topic="not_a_lingtu_stream"))

    assert payload["ok"] is False
    assert payload["error"] == "runtime_topic_not_found"
    assert payload["inspection"]["observable"] is False
    assert payload["inspection"]["communicate"] is False
    assert payload["inspection"]["ros2_topic_required"] is False
    assert "/nav/odometry" in payload["available_topics"]


@pytest.mark.skipif(not _NUMPY_IMPORT_SAFE, reason=NUMPY_UNSAFE_REASON)
def test_runtime_dataflow_reports_live_samples_for_field_topics(monkeypatch):
    import numpy as np

    from core.gateway_runtime_acceptance import (
        FIELD_LIVE_TOPICS,
        FIELD_REQUIRED_LIVE_STAGE_NAMES,
    )
    from core.msgs.nav import Odometry, Path
    from core.msgs.geometry import Twist
    from core.msgs.sensor import PointCloud2
    from core.runtime_interface import TOPICS
    from gateway.gateway_module import GatewayModule

    monkeypatch.setenv("LINGTU_RUNTIME_CONTRACT", "thunder_field")

    gateway = GatewayModule()
    endpoint = _endpoint(gateway, "/api/v1/runtime/dataflow")

    gateway.odometry._deliver(Odometry())
    gateway.map_cloud._deliver(PointCloud2(points=np.zeros((1, 3), dtype=np.float32)))
    gateway.global_path._deliver([np.array([0.0, 0.0, 0.0])])
    gateway.local_path._deliver(Path())
    frontier = {
        "id": "traversable_frontier_0",
        "source": "traversable_frontier",
        "centroid_3d": [1.0, 0.0, 0.2],
        "reachable_score": 0.7,
        "semantic_value": 0.2,
        "nearby_labels": ["inspection_pump"],
        "preview": True,
        "command_published": False,
        "reasons": [],
    }
    gateway.traversable_frontiers._deliver([frontier])
    gateway.frontier_candidate._deliver(frontier)
    gateway.cmd_vel.publish(Twist())

    payload = asyncio.run(endpoint())
    topics = {item["topic"]: item for item in payload["topics"]}

    for topic in FIELD_LIVE_TOPICS:
        observability = topics[topic]["observability"]
        assert observability["has_fresh_module_sample"] is True, topic
        assert observability["live_module_samples"] is True, topic
        assert observability["ros2_topic_required"] is False, topic
        assert any(
            port["msg_count"] > 0
            for port in observability["module_port_candidates"]
        ), topic

    stages = {stage["name"]: stage for stage in payload["stage_evidence"]}
    for stage_name in FIELD_REQUIRED_LIVE_STAGE_NAMES:
        assert stages[stage_name]["live"] is True, stage_name
        assert stages[stage_name]["status"] == "live", stage_name

    assert topics[TOPICS.cmd_vel]["communication"]["arbitrary_publish_supported"] is False


def test_runtime_dataflow_topic_route_answers_every_product_observable_stream_without_ros2(
    monkeypatch,
):
    from core.gateway_runtime_acceptance import PRODUCT_OBSERVABLE_TOPICS
    from gateway.gateway_module import GatewayModule

    monkeypatch.setenv("LINGTU_RUNTIME_CONTRACT", "thunder_field")

    gateway = GatewayModule()
    endpoint = _endpoint(gateway, "/api/v1/runtime/dataflow/topic")

    for topic in PRODUCT_OBSERVABLE_TOPICS:
        payload = asyncio.run(endpoint(topic=topic))

        assert payload["ok"] is True, topic
        assert payload["topic"]["topic"] == topic
        assert payload["inspection"]["ros2_topic_required"] is False, topic
        assert payload["inspection"]["arbitrary_publish_supported"] is False, topic
        assert payload["inspection"]["payload_available"] is True, topic
        assert (
            payload["inspection"]["payload_interfaces"]
            or payload["inspection"]["stream_interfaces"]
        ), topic


def test_runtime_dataflow_subscribe_route_covers_every_gateway_realtime_stream(
    monkeypatch,
):
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import RuntimeDataflowSubscribeRequest

    monkeypatch.setenv("LINGTU_RUNTIME_CONTRACT", "thunder_field")

    gateway = GatewayModule()
    gateway.setup()
    routes = {
        route.path: route.endpoint
        for route in gateway._app.routes
        if hasattr(route, "endpoint")
    }
    dataflow_endpoint = routes["/api/v1/runtime/dataflow"]
    subscribe_endpoint = routes["/api/v1/runtime/dataflow/subscribe"]

    snapshot = asyncio.run(dataflow_endpoint())
    stream_topics = [
        topic["topic"]
        for topic in snapshot["topics"]
        if any(
            stream.get("transport") == "gateway_sse"
            for stream in topic["inspection"].get("stream_interfaces", [])
        )
    ]

    assert stream_topics
    for topic in stream_topics:
        payload = asyncio.run(
            subscribe_endpoint(RuntimeDataflowSubscribeRequest(selector=topic))
        )

        assert payload["ok"] is True, topic
        assert payload["read_only"] is True, topic
        assert payload["ros2_topic_required"] is False, topic
        assert payload["arbitrary_publish_supported"] is False, topic
        assert payload["publishes"] == [], topic
        assert payload["topic"] == topic
        assert payload["transport"] == "gateway_sse", topic
        assert payload["stream_url"] == f"/api/v1/events?topic={topic.replace('/', '%2F')}", topic
        assert payload["event_types"], topic
