from __future__ import annotations

import asyncio
import hashlib
import json
import time

import pytest

pytestmark = [pytest.mark.sim]


pytest.importorskip("fastapi")
from runtime.tests.numpy_guard import NUMPY_UNSAFE_REASON, numpy_import_is_safe

_NUMPY_IMPORT_SAFE = numpy_import_is_safe()


def _endpoint(gateway, path: str):
    gateway.setup()
    return next(route.endpoint for route in gateway._app.routes if route.path == path)


def _payload(response_or_payload):
    if hasattr(response_or_payload, "body"):
        return json.loads(response_or_payload.body)
    return response_or_payload


def _field_manifest(profile: str):
    from lingtu.assembly.products import resolve_product_host_runtime
    from lingtu.assembly.profile_builder import compile_run_plan

    resolved = resolve_product_host_runtime(profile, "real")
    return compile_run_plan(
        resolved.product,
        resolved.env,
        resolved.config,
    )


def _field_gateway(profile: str):
    from gateway.gateway_module import GatewayModule

    return GatewayModule(run_plan=_field_manifest(profile))


def test_algorithm_benchmark_dimos_required_gates_match_core_constant():
    from gateway.routes.diagnostics import ALGORITHM_BENCHMARK_VARIANTS
    from runtime.algorithm_gates import DIMOS_BENCHMARK_REQUIRED_GATES

    dimos = ALGORITHM_BENCHMARK_VARIANTS["dimos_benchmark"]

    assert tuple(dimos["required_gate_sequence"]) == tuple(DIMOS_BENCHMARK_REQUIRED_GATES)
    assert dimos["required_gate_sequence"][0] == "gateway_runtime_acceptance"


def _write_active_same_source_octomap(map_root):
    active_dir = map_root / "demo"
    active_dir.mkdir(parents=True)
    (map_root / "active_map.txt").write_text("demo\n", encoding="utf-8")
    map_path = active_dir / "map.pcd"
    octomap_path = active_dir / "octomap.ot"
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
    octomap_path.write_bytes(b"lingtu-test-octomap")
    map_sha = hashlib.sha256(map_path.read_bytes()).hexdigest()
    octomap_sha = hashlib.sha256(octomap_path.read_bytes()).hexdigest()
    (active_dir / "metadata.json").write_text(
        json.dumps(
            {
                "schema_version": "lingtu.saved_map_artifacts.v1",
                "source_profile": "thunder",
                "data_source": "thunder",
                "slam_source": "fastlio2",
                "localization_source": "fastlio2",
                "mapping_source": "fastlio2",
                "frame_id": "map",
                "created_at": "2026-05-25T00:00:00Z",
                "artifacts": {
                    "map_pcd": {
                        "path": "map.pcd",
                        "sha256": map_sha,
                        "source_profile": "thunder",
                        "data_source": "thunder",
                        "slam_source": "fastlio2",
                        "frame_id": "map",
                        "point_count": 1,
                    },
                    "octomap": {
                        "path": "octomap.ot",
                        "sha256": octomap_sha,
                        "source_map_sha256": map_sha,
                        "source_profile": "thunder",
                        "data_source": "thunder",
                        "frame_id": "map",
                        "resolution": 0.2,
                    },
                },
            },
            sort_keys=True,
        ),
        encoding="utf-8",
    )
    return active_dir


def test_diagnostics_plugin_catalog_exposes_registered_backends():
    from decision.llm.client import MockLLMClient
    from gateway.routes.diagnostics import build_plugin_catalog, clear_diagnostics_cache
    from nav.local.path_follower import PathFollower
    from nav.local.local_planner import LocalPlanner
    # ^ Cross-layer: gateway test imports from decision/perception for registry setup.
    #   Acceptable: test needs to register a semantic backend in the global
    #   registry so build_plugin_catalog() can detect it.  No production
    #   dependency �?the import is for side-effect registration only.

    clear_diagnostics_cache()

    payload = build_plugin_catalog()

    assert payload["schema_version"] == 1
    categories = payload["categories"]
    assert "local_planner" in categories
    local_planner_names = {entry["name"] for entry in categories["local_planner"]}
    assert local_planner_names >= {
        "nanobind",
        "cmu_py",
        "simple",
    }
    assert "cmu" not in local_planner_names
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
                    "degraded_reason": "compatible LingTu native navigation kernel missing",
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
            "nav.local_planner": FakeLocalPlanner(),
            "VectorMemoryModule": FakeVectorMemory(),
            "nav.mission": FakeNavigation(),
            "BrokenModule": BrokenModule(),
        }
    )

    payload = asyncio.run(_endpoint(gateway, "/api/v1/diagnostics/plugins")())

    active = payload["active"]
    local = active["modules"]["nav.local_planner"]["backends"]["local_planner"]
    vector = active["modules"]["VectorMemoryModule"]["backends"]["encoder_backend"]
    planner = active["modules"]["nav.mission"]["backends"]["planner_backend"]
    broken = active["modules"]["BrokenModule"]
    assert active["schema_version"] == 1
    assert local["configured_backend"] == "nanobind"
    assert local["backend"] == "cmu_py"
    assert local["degraded"] is True
    assert "compatible LingTu native navigation kernel missing" in local["degraded_reason"]
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
    gateway.on_system_modules({"nav.mission": BusyNavigation()})

    result = gateway.reconfigure_backend("local_planner", "nav_kernel")

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
        "nav.mission": Navigation(),
        "nav.velocity_mux": Mux(),
        "SlamAdapterModule": Relocalization(),
    }
    gateway = GatewayModule()
    gateway.on_system_modules(modules)

    assert gateway._all_modules == modules
    assert gateway._all_modules is not modules
    assert gateway._navigation is modules["nav.mission"]
    assert gateway._backend_reconfigure_modules["nav.mission"] is (modules["nav.mission"])
    assert gateway.localization.backend is modules["SlamAdapterModule"]


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
            "nav.mission": IdleNavigation(),
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
    gateway.on_system_modules({"nav.mission": IdleNavigation()})

    result = gateway.reconfigure_backend("local_planner", "nav_kernel")

    assert result["ok"] is False
    assert result["reason"] == "backend_reconfigure_unsupported"


def test_gateway_motion_backend_switch_requires_public_navigation_state():
    from gateway.gateway_module import GatewayModule

    class NavigationWithoutPublicState:
        _state = "IDLE"

        def health(self):
            return {}

    gateway = GatewayModule()
    gateway.on_system_modules({"nav.mission": NavigationWithoutPublicState()})

    result = gateway.reconfigure_backend("local_planner", "nav_kernel")

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

    payload = json.loads(mcp.switch_backend("local_planner", "nav_kernel", '{"force": false}'))

    assert payload["ok"] is False
    assert payload["reason"] == "motion_backend_switch_requires_idle"
    assert payload["config"] == {"force": False}


def test_mcp_backend_switch_tool_guards_motion_without_gateway_module():
    from gateway.mcp_server import MCPServerModule

    class BusyNavigation:
        def health(self):
            return {"state": "EXECUTING"}

    mcp = MCPServerModule()
    mcp.on_system_modules({"MCPServerModule": mcp, "nav.mission": BusyNavigation()})

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
    mcp.on_system_modules({"MCPServerModule": mcp, "nav.mission": IdleNavigation()})

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
    mcp.on_system_modules({"MCPServerModule": mcp, "nav.mission": NavigationWithoutPublicState()})

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

    gateway._icp_quality = 0.0
    with gateway._state_lock:
        gateway._localization_status = {
            "state": "TRACKING",
            "confidence": 0.66,
            "quality": 0.66,
            "backend": "fastlio2",
            "health_source": "slam_runtime",
            "pose_fresh": True,
            "has_odom": True,
            "map_odom_tf": {
                "valid": True,
                "frame_id": "map",
                "child_frame_id": "odom",
                "tx": 0.0,
                "ty": 0.0,
                "tz": 0.0,
                "qx": 0.0,
                "qy": 0.0,
                "qz": 0.0,
                "qw": 1.0,
            },
        }
    payload = build_localization_status(gateway)
    assert payload["state"] == "ready"
    assert payload["ready"] is True
    assert payload["algorithm_healthy"] is True
    assert payload["backend"] == "native_dds"
    assert payload["algorithm_profile"] == "fastlio2"
    assert payload["health_source"] == "slam_runtime"
    assert payload["has_map_odom_tf"] is True

    gateway._session_mode = "navigating"
    gateway._icp_quality = 0.2
    with gateway._state_lock:
        gateway._localization_status = {"state": "TRACKING", "confidence": 0.9}
    payload = build_localization_status(gateway)
    assert payload["state"] == "ready"
    assert payload["ready"] is True

    with gateway._state_lock:
        gateway._localization_status = {
            "backend": "localizer",
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
        gateway._localization_status = {
            "backend": "localizer",
            "state": "LOST",
            "confidence": 0.0,
        }
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


def test_localization_status_exposes_backend_reason():
    from gateway.gateway_module import GatewayModule
    from gateway.services.runtime_status import build_localization_status

    gateway = GatewayModule()
    with gateway._state_lock:
        gateway._odom = {"x": 0.0, "y": 0.0, "frame_id": "odom"}
        gateway._localization_status = {
            "state": "TRACKING",
            "confidence": 0.9,
            "reason": "tracking",
        }

    payload = build_localization_status(gateway)

    assert payload["reason"] == "tracking"
    assert payload["backend_reason"] == "tracking"


def test_localization_status_reports_runtime_boundary_and_topic_frames(monkeypatch):
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import LocalizationStatusResponse
    from gateway.services.runtime_status import build_localization_status

    monkeypatch.setenv("LINGTU_PROFILE", "nav")
    monkeypatch.setenv("LINGTU_DATA_SOURCE", "thunder")
    monkeypatch.setenv("LINGTU_RUNTIME_CONTRACT", "real")
    monkeypatch.setenv("LINGTU_COMMAND_SINK", "driver")
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
            "status_target_hz": 10.0,
            "imu_input_hz": 198.5,
            "lidar_input_hz": 10.0,
            "slam_tick_hz": 50.0,
            "processed_scan_hz": 9.8,
            "registered_points": 24000,
            "map_points": 512000,
            "imu_buffer": 4,
            "lidar_buffer": 1,
            "imu_batch": 20,
            "dropped_lidar_frames": 0,
            "dropped_imu_frames": 1,
            "scan_start_s": 122.9,
            "scan_end_s": 123.0,
            "last_imu_s": 123.01,
            "sync_wait_count": 2,
            "imu_rollback_count": 0,
            "lidar_rollback_count": 0,
            "map_loaded": True,
            "map_frame_jump": False,
            "scene_mode": "outdoor",
            "gnss_fusion_health": {"enabled": True, "alignment_locked": True},
            "map_odom_tf": {
                "valid": True,
                "frame_id": "map",
                "child_frame_id": "odom",
                "tx": 0.0,
                "ty": 0.0,
                "tz": 0.0,
                "qx": 0.0,
                "qy": 0.0,
                "qz": 0.0,
                "qw": 1.0,
            },
        }

    payload = build_localization_status(gateway)
    model = LocalizationStatusResponse.model_validate(payload)

    runtime = payload["runtime"]
    assert runtime["ok"] is True
    assert runtime["data_source"] == "thunder"
    assert runtime["runtime_contract"] == "real"
    assert runtime["frames"]["map"] == "map"
    assert runtime["frames"]["odom"] == "odom"
    assert runtime["frames"]["body"] == "body"
    assert runtime["topic_default_frame_ids"]["/slam/odometry"] == "odom"
    assert runtime["topic_default_frame_ids"]["/slam/registered_cloud"] == "body"
    assert runtime["topic_default_frame_ids"]["/slam/map_cloud"] == "map"
    assert runtime["required_topic_frame_ids"][:5] == [
        "/lidar/raw_frame",
        "/imu/raw",
        "/slam/odometry",
        "/slam/registered_cloud",
        "/slam/map_cloud",
    ]
    assert runtime["runtime_data_flow_topics"][:3] == [
        "/lidar/raw_frame",
        "/imu/raw",
        "/slam/odometry",
    ]

    frames = payload["frames"]
    assert frames["runtime_contract"] == "real"
    assert frames["odometry_frame_id"] == "odom"
    assert frames["registered_cloud_frame_id"] == "body"
    assert frames["map_cloud_frame_id"] == "map"
    assert payload["registered_cloud_frame_id"] == "body"
    assert payload["map_cloud_frame_id"] == "map"
    assert frames["odometry_expected_frame_ids"] == ["odom", "map"]
    assert frames["registered_cloud_expected_frame_ids"] == ["body"]
    assert frames["map_cloud_expected_frame_ids"] == ["map"]
    assert frames["missing_required_topic_frame_ids"] == []
    assert frames["mismatches"] == []
    assert frames["ok"] is True
    assert payload["status_target_hz"] == 10.0
    assert payload["imu_input_hz"] == 198.5
    assert payload["lidar_input_hz"] == 10.0
    assert payload["slam_tick_hz"] == 50.0
    assert payload["processed_scan_hz"] == 9.8
    assert payload["registered_points"] == 24000
    assert payload["map_points"] == 512000
    assert payload["imu_buffer"] == 4
    assert payload["lidar_buffer"] == 1
    assert payload["imu_batch"] == 20
    assert payload["dropped_imu_frames"] == 1
    assert payload["sync_wait_count"] == 2
    assert payload["map_loaded"] is True
    assert payload["map_frame_jump"] is False
    assert payload["scene_mode"] == "outdoor"
    assert payload["has_map_odom_tf"] is True
    assert payload["gnss_fusion_health"]["alignment_locked"] is True
    assert model.runtime.data_source == "thunder"
    assert model.frames.ok is True
    assert model.status_target_hz == 10.0
    assert model.processed_scan_hz == 9.8
    assert model.has_map_odom_tf is True
    assert model.map_odom_tf["child_frame_id"] == "odom"


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


def test_localization_runtime_generation_and_map_jump_reset_viewer_epoch():
    from gateway.gateway_module import GatewayModule

    gateway = GatewayModule()
    gateway._on_localization_status(
        {
            "state": "TRACKING",
            "runtime_instance_id": "slam-a",
            "observation_sequence": 10,
            "map_frame_jump": False,
            "map_frame_jump_sequence": 0,
        }
    )
    assert gateway._traffic_stats_snapshot()["scan"]["scene_epoch"] == 1

    gateway._on_localization_status(
        {
            "state": "STALE",
            "runtime_instance_id": "slam-a",
            "observation_sequence": 10,
            "map_frame_jump_sequence": 0,
        }
    )
    gateway._on_localization_status(
        {
            "state": "TRACKING",
            "runtime_instance_id": "slam-b",
            "observation_sequence": 1,
            "map_frame_jump": False,
            "map_frame_jump_sequence": 0,
        }
    )
    assert gateway._traffic_stats_snapshot()["scan"]["scene_epoch"] == 2
    assert gateway._localization_status["viewer_epoch_reset_reason"] == "slam_runtime_changed"

    gateway._on_localization_status(
        {
            "state": "TRACKING",
            "runtime_instance_id": "slam-b",
            "observation_sequence": 2,
            "map_frame_jump": False,
            "map_frame_jump_sequence": 1,
        }
    )
    assert gateway._traffic_stats_snapshot()["scan"]["scene_epoch"] == 3
    gateway._on_localization_status(
        {
            "state": "TRACKING",
            "runtime_instance_id": "slam-b",
            "observation_sequence": 3,
            "map_frame_jump": False,
            "map_frame_jump_sequence": 1,
        }
    )
    assert gateway._traffic_stats_snapshot()["scan"]["scene_epoch"] == 3


def test_localization_observation_sequence_rollback_resets_legacy_runtime_viewer():
    from gateway.gateway_module import GatewayModule

    gateway = GatewayModule()
    gateway._on_localization_status({"state": "TRACKING", "observation_sequence": 20})
    gateway._on_localization_status({"state": "TRACKING", "observation_sequence": 1})

    assert gateway._traffic_stats_snapshot()["scan"]["scene_epoch"] == 2
    assert gateway._localization_status["viewer_epoch_reset_reason"] == ("slam_observation_sequence_rollback")


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


def test_localization_status_accepts_genz_icp_restart_only_contract():
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import LocalizationStatusResponse
    from gateway.services.runtime_status import build_localization_status

    gateway = GatewayModule()
    gateway._session_mode = "navigating"
    with gateway._state_lock:
        gateway._odom = {"x": 0.0}
        gateway._localization_status = {
            "backend": "genz",
            "state": "TRACKING",
            "confidence": 0.86,
            "health_source": "odom_map_cloud",
            "localizer_health": "LIO_TRACKING",
            "pose_fresh": True,
            "map_cloud_fresh": True,
            "odom_age_ms": 110.0,
            "cloud_age_ms": 90.0,
        }

    payload = build_localization_status(gateway)
    model = LocalizationStatusResponse.model_validate(payload)

    assert model.state == "ready"
    assert model.ready is True
    assert model.backend == "genz"
    assert model.health_source == "odom_map_cloud"
    assert model.map_save_supported is False
    assert model.map_save_source is None
    assert model.relocalization_supported is False
    assert model.saved_map_relocalization_supported is False
    assert model.restart_recovery_supported is True
    assert model.recovery_method == "restart_genz_icp"
    assert model.can_relocalize is False


def test_mapping_session_snapshot_does_not_expose_saved_map_as_active(monkeypatch):
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import SessionResponse

    class _MapsService:
        @staticmethod
        def query(request):
            assert request.action == "get_active"
            return {"active": "old_nav_map"}

    gateway = GatewayModule()
    gateway._map_mgr = _MapsService()
    gateway._session_mode = "mapping"
    gateway._session_product_session = "mapping"
    gateway._session_product = "map"

    session = gateway._session_snapshot()
    model = SessionResponse.model_validate(session)

    assert model.active_map is None
    assert model.saved_active_map == "old_nav_map"
    assert session["active_map"] is None
    assert session["saved_active_map"] == "old_nav_map"


def test_slam_profile_prefers_live_native_dds_status_over_stopped_session():
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import SessionResponse

    gateway = GatewayModule()
    gateway._session_slam_profile = "stopped"
    gateway._cached_slam_profile = "stopped"
    gateway._localization_status = {
        "backend": "fastlio2",
        "state": "TRACKING",
        "confidence": 0.95,
        "pose_fresh": True,
        "map_cloud_fresh": True,
        "health_source": "slam_runtime",
    }

    assert gateway._get_slam_profile() == "native_dds"

    session = gateway._session_snapshot()
    model = SessionResponse.model_validate(session)

    assert model.slam_profile == "native_dds"
    assert model.localization_backend == "native_dds"
    assert model.map_save_supported is True


def test_slam_profile_keeps_native_dds_runtime_profile():
    from gateway.gateway_module import GatewayModule

    assert (
        GatewayModule._slam_profile_from_status(
            {
                "backend": "fastlio2",
                "health_source": "slam_runtime",
                "mode": "mapping",
                "state": "MAPPING",
            }
        )
        == "native_dds"
    )
    assert (
        GatewayModule._slam_profile_from_status(
            {
                "backend": "fastlio2",
                "health_source": "slam_runtime",
                "mode": "localization",
                "state": "TRACKING",
            }
        )
        == "native_dds"
    )
    assert (
        GatewayModule._slam_profile_from_status(
            {
                "backend": "fastlio2",
                "health_source": "slam_runtime",
                "mode": "localization",
                "state": "FAILED",
            }
        )
        == ""
    )


@pytest.mark.parametrize(
    "rejected_backend",
    (
        "genz-icp",
        "genz_icp",
        "point-lio",
        "point_lio",
        "cpp_dds_slam",
        "lingtu_slam_dds",
        "lingtu-slam-dds",
        "native_slam",
        "slam",
        "unexpected_backend",
    ),
)
def test_slam_profile_rejects_noncanonical_backend_names(rejected_backend):
    from gateway.gateway_module import GatewayModule

    assert (
        GatewayModule._slam_profile_from_status(
            {"backend": rejected_backend, "state": "TRACKING"}
        )
        == ""
    )


@pytest.mark.parametrize(
    "backend",
    ("fastlio2", "pointlio", "genz", "localizer", "native_dds"),
)
def test_slam_profile_accepts_canonical_live_backend(backend):
    from gateway.gateway_module import GatewayModule

    assert (
        GatewayModule._slam_profile_from_status(
            {"backend": backend, "state": "TRACKING"}
        )
        == backend
    )


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
            return json.dumps(
                {
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
                }
            )

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
        "nav.velocity_mux": FakeMux(),
        "nav.mission": FakeNavigation(),
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
    assert payload["diagnostics"]["last_plan_report"]["selected_path_safety"]["ok"] is False
    assert payload["reason_codes"] == []
    assert payload["localization"]["degraded"] is False


def test_navigation_status_prefers_native_navigation_state() -> None:
    from gateway.services.runtime_status import build_navigation_status
    from runtime.msgs.nav import NavigationState

    gateway = GatewayModule()
    gateway._session_mode = "navigating"
    with gateway._state_lock:
        gateway._odom = {"x": 1.0, "y": 2.0, "vx": 0.0, "vy": 0.0}
        gateway._mode = "autonomous"
        gateway._mission = {
            "state": "FAILED",
            "failure_reason": "stale_python_mission",
            "ts": 1.0,
        }
        gateway._localization_status = {
            "state": "TRACKING",
            "confidence": 0.9,
            "icp_fitness": 0.03,
        }
    gateway._on_navigation_state(
        NavigationState(
            ts=123.0,
            frame_id="map",
            boot_id="navd-boot",
            sequence=4,
            control_mode=1,
            lifecycle_state=2,
            active_task_id="navigation-task-3",
            active_request_id="goal-3",
            goal_epoch=3,
            map_id="site-a",
            map_version=2,
            map_hash="sha256",
            planning_state=2,
            execution_state=1,
            recovery_state=0,
            progress=0.6,
            authority="autonomy",
        )
    )

    payload = build_navigation_status(gateway)

    assert payload["state_source"] == "native_navigation_state"
    assert payload["state"] == "EXECUTING"
    assert payload["failure_reason"] == ""
    assert payload["progress"]["fraction"] == pytest.approx(0.6)
    assert payload["control"]["command_owner"] == "autonomy"
    assert payload["navigation_state"]["boot_id"] == "navd-boot"
    assert payload["mission"]["raw"]["active_task_id"] == "navigation-task-3"
    assert payload["mission"]["raw"]["active_request_id"] == "goal-3"


def test_drift_watchdog_classifies_nan_odom_as_diverged():
    from gateway.gateway_module import GatewayModule

    gateway = GatewayModule()

    assert (
        gateway._drift_odom_diverged(
            {
                "x": float("nan"),
                "y": 0.0,
                "z": 0.0,
                "vx": 0.0,
            }
        )[0]
        is True
    )


def test_gateway_quarantines_non_finite_odometry_before_publication():
    from gateway.gateway_module import GatewayModule
    from runtime.msgs.geometry import Pose, Quaternion, Twist, Vector3
    from runtime.msgs.nav import Odometry

    gateway = GatewayModule()
    events = []
    gateway.push_event = events.append
    previous = {"x": 1.0, "y": 2.0, "z": 0.0, "yaw": 0.0}
    with gateway._state_lock:
        gateway._odom = dict(previous)

    gateway._on_odometry(
        Odometry(
            pose=Pose(
                position=Vector3(float("nan"), 0.0, 0.0),
                orientation=Quaternion(),
            ),
            twist=Twist(linear=Vector3(0.0, 0.0, 0.0)),
        )
    )

    assert gateway._odom == previous
    assert [event.get("type") for event in events] == []
    assert gateway._last_invalid_odometry["reason"] == "non_finite_odometry"


def test_drift_watchdog_uses_quarantined_non_finite_odometry():
    from gateway.gateway_module import GatewayModule
    from runtime.msgs.geometry import Pose, Quaternion, Twist, Vector3
    from runtime.msgs.nav import Odometry

    gateway = GatewayModule()
    with gateway._state_lock:
        gateway._odom = {"x": 1.0, "y": 2.0, "z": 0.0, "vx": 0.0}

    gateway._on_odometry(
        Odometry(
            pose=Pose(
                position=Vector3(float("nan"), 0.0, 0.0),
                orientation=Quaternion(),
            ),
            twist=Twist(linear=Vector3(0.0, 0.0, 0.0)),
        )
    )

    diverged, _x, _y, _z, _v, invalid = gateway._drift_current_odom_divergence()
    assert diverged is True
    assert invalid is True

    gateway._on_odometry(
        Odometry(
            pose=Pose(
                position=Vector3(0.0, 0.0, 0.0),
                orientation=Quaternion(),
            ),
            twist=Twist(linear=Vector3(0.0, 0.0, 0.0)),
        )
    )

    diverged, _x, _y, _z, _v, invalid = gateway._drift_current_odom_divergence()
    assert diverged is False
    assert invalid is False


def test_navigation_status_reports_current_runtime_boundary(monkeypatch):
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import NavigationStatusResponse
    from gateway.services.runtime_status import build_navigation_status

    monkeypatch.setenv("LINGTU_PROFILE", "nav")
    monkeypatch.setenv("LINGTU_DATA_SOURCE", "thunder")
    monkeypatch.setenv("LINGTU_RUNTIME_CONTRACT", "real")
    monkeypatch.setenv("LINGTU_COMMAND_SINK", "driver")
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
    assert runtime["endpoint"] == "thunder_dds"
    assert runtime["data_source"] == "thunder"
    assert runtime["runtime_contract"] == "real"
    assert runtime["simulation_only"] is False
    assert runtime["command_sink"] == "driver"
    assert runtime["expected_command_sink"] == "driver"
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
    assert runtime["topic_allowed_frame_ids"]["/slam/map_cloud"] == ["map"]
    assert runtime["topic_allowed_frame_ids"]["/nav/global_path"] == ["map"]
    assert runtime["topic_default_frame_ids"]["/slam/map_cloud"] == "map"
    assert runtime["topic_default_frame_ids"]["/nav/cmd_vel"] == "body"
    assert runtime["required_topic_frame_ids"] == [
        "/lidar/raw_frame",
        "/imu/raw",
        "/slam/odometry",
        "/slam/registered_cloud",
        "/slam/map_cloud",
        "/nav/global_path",
        "/nav/local_path",
        "/nav/cmd_vel",
    ]
    assert runtime["runtime_data_flow_topics"] == [
        "/lidar/raw_frame",
        "/imu/raw",
        "/slam/odometry",
        "/slam/registered_cloud",
        "/slam/map_cloud",
        "/slam/localization_health",
        "/slam/localization_quality",
        "/nav/exploration_grid",
        "/nav/terrain_map_ext",
        "/exploration/way_point",
        "/nav/goal_pose",
        "/nav/traversable_frontiers",
        "/nav/frontier_candidate",
        "/nav/global_path",
        "/nav/way_point",
        "/nav/terrain_map",
        "/nav/traversability",
        "/nav/local_path",
        "/nav/local_planner/control_hint",
        "/nav/cmd_vel",
        "/nav/added_obstacles",
        "/nav/check_obstacle",
        "/nav/planner_status",
    ]
    flow = {stage["name"]: stage for stage in runtime["resolved_runtime_data_flow"]}
    assert list(flow["endpoint_adapter"]["inputs"]) == [
        "/lidar/raw_frame",
        "/imu/raw",
    ]
    assert list(flow["command_boundary"]["outputs"]) == ["driver"]
    assert runtime["runtime_data_flow_stage_algorithm_interfaces"]["global_planning"] == [
        "global_planning",
        "octoplanner3d_global_planning",
    ]
    assert runtime["runtime_data_flow_stage_algorithm_interfaces"]["local_planning_and_following"] == [
        "local_planning_and_following"
    ]


def test_navigation_frame_summary_defaults_planning_frame_from_runtime_contract(
    monkeypatch,
):
    import runtime.runtime_interface as runtime_interface
    from gateway.services import runtime_status

    calls: list[tuple[str | None, str]] = []

    def fake_runtime_topic_default_frame_id(
        runtime_contract: str | None,
        topic: str,
    ) -> str:
        calls.append((runtime_contract, topic))
        return "contract_map"

    monkeypatch.setenv("LINGTU_RUNTIME_CONTRACT", "real")
    monkeypatch.setattr(
        runtime_interface,
        "runtime_topic_default_frame_id",
        fake_runtime_topic_default_frame_id,
    )

    summary = runtime_status._navigation_frame_summary({}, None)

    assert summary["planning_frame_id"] == "contract_map"
    assert summary["ok"] is True
    assert calls == [("real", runtime_interface.TOPICS.global_path)]


def test_navigation_status_flags_runtime_boundary_mismatch(monkeypatch):
    from gateway.gateway_module import GatewayModule
    from gateway.services.runtime_status import build_navigation_status

    monkeypatch.setenv("LINGTU_PROFILE", "explore")
    monkeypatch.setenv("LINGTU_DATA_SOURCE", "mujoco_fastlio2_live")
    monkeypatch.setenv("LINGTU_RUNTIME_CONTRACT", "real")
    monkeypatch.setenv("LINGTU_COMMAND_SINK", "driver")
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
    assert runtime["topic_allowed_frame_ids"]["/slam/map_cloud"] == ["map"]
    assert runtime["required_topic_frame_ids"] == [
        "/lidar/raw_frame",
        "/imu/raw",
        "/slam/odometry",
        "/slam/registered_cloud",
        "/slam/map_cloud",
        "/nav/global_path",
        "/nav/local_path",
        "/nav/cmd_vel",
    ]
    assert runtime["runtime_data_flow_topics"][:2] == ["/lidar/raw_frame", "/imu/raw"]


def test_navigation_status_reports_sim_runtime_topic_frames(monkeypatch):
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import NavigationStatusResponse
    from gateway.services.runtime_status import build_navigation_status

    monkeypatch.setenv("LINGTU_PROFILE", "explore")
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
    assert runtime["topic_allowed_frame_ids"]["/slam/map_cloud"] == ["map", "odom"]
    assert runtime["topic_allowed_frame_ids"]["/nav/global_path"] == ["map", "odom"]
    assert runtime["required_topic_frame_ids"] == []
    assert runtime["runtime_data_flow_topics"][:2] == ["/lidar/raw_frame", "/imu/raw"]


def test_navigation_status_flags_unknown_topic_frame_contract(monkeypatch):
    from gateway.gateway_module import GatewayModule
    from gateway.services.runtime_status import build_navigation_status

    monkeypatch.setenv("LINGTU_PROFILE", "nav")
    monkeypatch.setenv("LINGTU_DATA_SOURCE", "thunder")
    monkeypatch.setenv("LINGTU_RUNTIME_CONTRACT", "typo_contract")
    monkeypatch.setenv("LINGTU_COMMAND_SINK", "driver")
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
            "plan_safety_policy": "reject",
            "last_plan_report": {
                "primary_planner": "octoplanner3d",
                "selected_planner": "octoplanner3d",
                "fallback_reason": "",
                "rejected_plans": [{"planner": "octoplanner3d", "reason": "unsafe"}],
            },
        }
    gateway._all_modules = {"nav.velocity_mux": FakeMux()}

    payload = build_navigation_status(gateway)

    assert payload["diagnostics"]["plan_safety_policy"] == "reject"
    assert payload["diagnostics"]["last_plan_report"]["primary_planner"] == "octoplanner3d"
    assert payload["diagnostics"]["last_plan_report"]["selected_planner"] == "octoplanner3d"


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
    gateway._all_modules = {"nav.velocity_mux": FakeMux()}

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
    from gateway.gateway_module import GatewayModule
    from gateway.services.runtime_status import build_navigation_status
    from runtime.msgs.geometry import Pose, Quaternion, Vector3
    from runtime.msgs.nav import Odometry

    gateway = GatewayModule()
    gateway._session_mode = "navigating"
    gateway._icp_quality = 0.03
    with gateway._state_lock:
        gateway._mode = "autonomous"
        gateway._mission = {"state": "IDLE", "planning_frame_id": "map"}
        gateway._localization_status = {"state": "TRACKING", "confidence": 0.9}

    gateway._on_odometry(
        Odometry(
            pose=Pose(position=Vector3(1.0, 2.0, 0.0), orientation=Quaternion()),
            frame_id="odom",
            child_frame_id="base_link",
        )
    )
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


def test_gateway_navigation_status_accepts_odom_when_map_odom_tf_is_valid():
    from gateway.gateway_module import GatewayModule
    from gateway.services.runtime_status import build_navigation_status
    from runtime.msgs.geometry import Pose, Quaternion, Vector3
    from runtime.msgs.nav import Odometry

    gateway = GatewayModule()
    gateway._session_mode = "navigating"
    gateway._icp_quality = 0.03
    with gateway._state_lock:
        gateway._mode = "autonomous"
        gateway._mission = {
            "state": "IDLE",
            "planning_frame_id": "map",
            "costmap_frame_id": "map",
        }
        gateway._localization_status = {
            "state": "TRACKING",
            "confidence": 0.9,
            "map_odom_tf": {
                "valid": True,
                "frame_id": "map",
                "child_frame_id": "odom",
                "tx": 0.0,
                "ty": 0.0,
                "tz": 0.0,
                "qx": 0.0,
                "qy": 0.0,
                "qz": 0.0,
                "qw": 1.0,
            },
        }

    gateway._on_odometry(
        Odometry(
            pose=Pose(position=Vector3(1.0, 2.0, 0.0), orientation=Quaternion()),
            frame_id="odom",
            child_frame_id="base_link",
        )
    )
    payload = build_navigation_status(gateway)

    assert payload["frames"]["ok"] is True
    assert payload["frames"]["mismatches"] == []
    assert payload["frames"]["odometry_expected_frame_ids"] == ["map", "odom"]
    assert payload["readiness"]["tf_ok"] is True
    assert "frame_mismatch_odometry" not in payload["reason_codes"]


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
        gateway._on_mission(
            {
                "state": "EXECUTING",
                "planning_frame_id": "map",
                "odom_frame_id": "map",
                "costmap_frame_id": "map",
            }
        )
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
    gateway._all_modules = {"nav.velocity_mux": FakeMux()}

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


def test_navigation_status_reads_idle_costmap_frame_from_nav_mission():
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import NavigationStatusResponse
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
                    "costmap_frame_id": "odom",
                }
            )

    gateway = GatewayModule()
    gateway._session_mode = "navigating"
    gateway._icp_quality = 0.03
    with gateway._state_lock:
        gateway._odom = {"x": 1.0, "y": 2.0, "frame_id": "map"}
        gateway._mode = "autonomous"
        gateway._mission = {"state": "IDLE", "planning_frame_id": "map"}
        gateway._localization_status = {"state": "TRACKING", "confidence": 0.9}
    gateway._all_modules = {
        "nav.velocity_mux": FakeMux(),
        "nav.mission": FakeNavigation(),
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
    gateway._navigation = FakeNavigation()
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
    gateway._all_modules = {"nav.velocity_mux": FakeMux()}

    payload = build_navigation_status(gateway)

    assert payload["can_accept_goal"] is False
    assert "navigation_session_inactive" in payload["reason_codes"]
    assert "navigation_session_inactive" in payload["readiness"]["blockers"]
    assert payload["readiness"]["session_mode"] == "idle"
    assert payload["feedback"]["next_action"] == "resolve_blockers"


def test_navigation_status_can_disable_real_runtime_evidence_gate_for_commissioning(monkeypatch):
    from gateway.gateway_module import GatewayModule
    from gateway.services.runtime_status import build_navigation_status

    class FakeMux:
        def health(self):
            return {"active_source": "none", "sources": {}}

    monkeypatch.setenv("LINGTU_RUNTIME_CONTRACT", "real")
    monkeypatch.setenv("LINGTU_REQUIRE_REAL_RUNTIME_EVIDENCE", "0")

    gateway = GatewayModule()
    gateway._session_mode = "navigating"
    gateway._icp_quality = 0.9
    with gateway._state_lock:
        gateway._odom = {"x": 1.0, "y": 2.0, "frame_id": "odom"}
        gateway._mode = "autonomous"
        gateway._mission = {
            "state": "IDLE",
            "planning_frame_id": "map",
            "odom_frame_id": "odom",
        }
        gateway._localization_status = {
            "state": "TRACKING",
            "confidence": 0.9,
            "algorithm_healthy": True,
            "map_odom_tf": {"valid": True, "frame_id": "map", "child_frame_id": "odom"},
        }
    gateway._all_modules = {"nav.velocity_mux": FakeMux()}

    payload = build_navigation_status(gateway)

    assert "real_runtime_evidence_missing_or_stale" not in payload["reason_codes"]
    assert "real_runtime_evidence_missing_or_stale" not in payload["readiness"]["blockers"]
    assert payload["readiness"]["real_runtime_evidence_ok"] is None
    assert payload["diagnostics"]["real_runtime_evidence"]["reason"] == "disabled_for_commissioning"


def test_navigation_status_uses_real_runtime_preflight_before_first_motion(monkeypatch):
    from gateway.gateway_module import GatewayModule
    from gateway.services.runtime_status import build_navigation_status

    class FakeMux:
        def health(self):
            return {"active_source": "none", "sources": {}}

    monkeypatch.setenv("LINGTU_RUNTIME_CONTRACT", "real")
    monkeypatch.setenv("LINGTU_REQUIRE_REAL_RUNTIME_EVIDENCE", "1")
    monkeypatch.setattr(
        "gateway.routes.diagnostics.build_real_runtime_evidence_latest_summary",
        lambda: {
            "ok": False,
            "preflight_ok": True,
            "blockers": ["real-runtime-evidence real_robot_motion is not true"],
            "preflight_blockers": [],
        },
    )

    gateway = GatewayModule()
    gateway._session_mode = "navigating"
    gateway._icp_quality = 0.9
    with gateway._state_lock:
        gateway._odom = {"x": 1.0, "y": 2.0, "frame_id": "odom"}
        gateway._mode = "autonomous"
        gateway._mission = {
            "state": "IDLE",
            "planning_frame_id": "map",
            "odom_frame_id": "odom",
        }
        gateway._localization_status = {
            "state": "TRACKING",
            "confidence": 0.9,
            "algorithm_healthy": True,
            "map_odom_tf": {"valid": True, "frame_id": "map", "child_frame_id": "odom"},
        }
    gateway._all_modules = {"nav.velocity_mux": FakeMux()}

    payload = build_navigation_status(gateway)

    assert payload["can_accept_goal"] is True
    assert "real_runtime_evidence_missing_or_stale" not in payload["reason_codes"]
    assert payload["readiness"]["real_runtime_evidence_ok"] is True
    assert payload["readiness"]["real_runtime_evidence_full_ok"] is False


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
    gateway._all_modules = {"nav.velocity_mux": FakeMux()}

    payload = build_navigation_status(gateway)

    assert payload["readiness"]["session_mode"] == "exploring"
    assert "navigation_session_inactive" not in payload["reason_codes"]
    assert "navigation_session_inactive" not in payload["readiness"]["blockers"]


def test_navigation_status_requires_canonical_velocity_mux_runtime_id():
    from gateway.gateway_module import GatewayModule
    from gateway.services.runtime_status import build_navigation_status

    class FakeVelocityMux:
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
    gateway._all_modules = {"unrelated_key": FakeVelocityMux()}

    payload = build_navigation_status(gateway)

    assert payload["control"]["cmd_vel_mux"]["available"] is False
    assert payload["control"]["active_cmd_source"] == "unknown"
    assert payload["control"]["source_category"] == "unknown"


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


def test_navigation_status_handles_recovering_mission_as_active():
    from gateway.gateway_module import GatewayModule
    from gateway.services.runtime_status import build_navigation_status

    gateway = GatewayModule()
    with gateway._state_lock:
        gateway._odom = {"x": 0.0}
        gateway._mission = {"state": "RECOVERING"}
        gateway._localization_status = {"state": "TRACKING", "confidence": 0.9}

    payload = build_navigation_status(gateway)

    assert payload["state"] == "RECOVERING"
    assert payload["progress"]["active"] is True
    assert "mission_recovering" in payload["reason_codes"]


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
    gateway._all_modules = {"nav.velocity_mux": FakeMux()}

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
    gateway._all_modules = {"nav.velocity_mux": FakeMux()}

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


def test_navigation_status_blocks_when_native_input_gate_is_not_ready(
    monkeypatch,
    tmp_path,
):
    from gateway.services.runtime_status import build_navigation_status

    class FakeMux:
        def health(self):
            return {"active_source": "none", "sources": {}}

    status_path = tmp_path / "nav_endpoint_status.json"
    monkeypatch.setenv("LINGTU_COMMAND_OUTPUT_MODE", "endpoint_only")
    monkeypatch.setenv("LINGTU_NAV_STATUS_FILE", str(status_path))
    monkeypatch.setenv("LINGTU_NAV_STATUS_MAX_AGE_S", "30")

    gateway = _field_gateway("nav")
    gateway._session_mode = "navigating"
    gateway._icp_quality = 0.03
    with gateway._state_lock:
        gateway._odom = {"x": 0.0}
        gateway._mission = {"state": "IDLE"}
        gateway._localization_status = {
            "state": "TRACKING",
            "confidence": 0.9,
            "degeneracy": "NONE",
            "icp_fitness": 0.028,
            "localizer_health": "RECOVERED",
            "odom_age_ms": 0.0,
        }
    gateway._all_modules = {"nav.velocity_mux": FakeMux()}

    status_path.write_text(
        json.dumps(
            {
                "stamp_s": time.time(),
                "input_gate": {
                    "ready": False,
                    "reason": "localization_unhealthy",
                },
            }
        ),
        encoding="utf-8",
    )

    payload = build_navigation_status(gateway)

    assert payload["can_accept_goal"] is False
    assert "native_input_gate_not_ready" in payload["readiness"]["blockers"]
    assert payload["readiness"]["native_endpoint"]["input_gate"]["reason"] == ("localization_unhealthy")


def test_native_endpoint_readiness_requires_product_control_mode_and_cmd_vel_publish(
    monkeypatch,
    tmp_path,
):
    from gateway.services.runtime_status import _native_endpoint_readiness

    status_path = tmp_path / "nav_endpoint_status.json"
    monkeypatch.setenv("LINGTU_COMMAND_OUTPUT_MODE", "endpoint_only")
    monkeypatch.setenv("LINGTU_NAV_STATUS_FILE", str(status_path))
    monkeypatch.setenv("LINGTU_NAV_STATUS_MAX_AGE_S", "30")

    def write_status(
        control_mode: str,
        publish_cmd_vel: bool,
        *,
        global_planner: str = "octoplanner3d",
        planner_map: str = "/maps/active/octomap.ot",
        product: str | None = None,
    ) -> None:
        native_status = {}
        if product:
            from runtime.profiles.native_nav_config import compile_native_nav_config

            manifest = _field_manifest(product)
            manifest_config = dict(manifest.host_config)
            manifest_config["native_nav"] = dict(manifest.native_nav)
            manifest_config["native_control_mode"] = manifest.native_nav["control_mode"]

            expected = compile_native_nav_config(
                product,
                manifest_config,
            ).as_dict()
            parameters = expected["parameters"]
            native_status = {
                "native_product": {
                    "product": product,
                    "config_fingerprint": expected["fingerprint"],
                },
                "path_follower": {
                    "max_speed_mps": parameters["path_follower_max_speed_mps"],
                    "min_speed_mps": parameters["path_follower_min_speed_mps"],
                    "max_accel_mps2": parameters["path_follower_max_accel_mps2"],
                    "lookahead_m": parameters["path_follower_lookahead_m"],
                    "goal_tolerance_m": parameters["path_follower_goal_tolerance_m"],
                },
                "nav_loop": {
                    "waypoint_reached_m": parameters["waypoint_reached_m"],
                    "goal_reached_m": parameters["goal_reached_m"],
                    "corridor_lookahead_m": parameters["corridor_lookahead_m"],
                },
            }
            if product == "teleop_avoid":
                native_status.update(
                    teleop_local_planner=True,
                    check_obstacle=True,
                    use_traversability_cost=True,
                    teleop_planner_horizon_m=2.0,
                    teleop_planner_max_deviation_deg=55.0,
                )
        status_path.write_text(
            json.dumps(
                {
                    "stamp_s": time.time(),
                    "control_loop_health": {
                        "ready": True,
                        "healthy": True,
                        "reason": "healthy",
                    },
                    "input_gate": {"ready": True, "reason": "ready"},
                    "control_mode": control_mode,
                    "operator_motion": {
                        "schema_version": 1,
                        "interface_enabled": True,
                        "authority_owner": "native_endpoint",
                        "control_mode": control_mode,
                        "allow_teleop_takeover": control_mode == "autonomy",
                        "control_ack_scope": "claim_hold_release",
                        "sample_evidence": "status_sequences",
                    },
                    "global_planner": global_planner,
                    "planner_map": planner_map,
                    "publish_cmd_vel": publish_cmd_vel,
                    "active_cmd_source": "none",
                    "control_authority": {
                        "owner": "native_endpoint",
                        "estop_latched": False,
                        "operator_takeover_latched": False,
                        "resume_required": False,
                    },
                    **native_status,
                }
            ),
            encoding="utf-8",
        )

    write_status("teleop", False)
    blocked = _native_endpoint_readiness({"mode": "navigating"})

    assert blocked["ok"] is False
    assert blocked["blockers"] == [
        "native_control_mode_mismatch",
        "native_cmd_vel_publish_disabled",
    ]

    write_status("autonomy", True)
    ready = _native_endpoint_readiness({"mode": "navigating"})

    assert ready["ok"] is True
    assert ready["blockers"] == []
    assert ready["global_planner"] == "octoplanner3d"
    assert ready["planner_map"] == "/maps/active/octomap.ot"

    write_status("teleop_avoid", True, product="teleop_avoid")
    assisted_gateway = _field_gateway("teleop_avoid")
    assisted = _native_endpoint_readiness(
        {"mode": "navigating", "product": "teleop_avoid"},
        assisted_gateway,
    )

    assert assisted["ok"] is True
    assert assisted["expected_control_mode"] == "teleop_avoid"
    assert assisted["blockers"] == []
    assert assisted["operator_motion"]["required"] is True
    assert assisted["operator_motion"]["status_available"] is True

    status_without_operator_motion = json.loads(status_path.read_text(encoding="utf-8"))
    status_without_operator_motion.pop("operator_motion")
    status_path.write_text(json.dumps(status_without_operator_motion), encoding="utf-8")
    missing_operator_motion = _native_endpoint_readiness(
        {"mode": "navigating", "product": "teleop_avoid"},
        assisted_gateway,
    )
    assert missing_operator_motion["ok"] is False
    assert missing_operator_motion["blockers"] == ["native_operator_motion_status_missing"]
    from gateway.services.runtime_status import _navigation_blockers

    assert _navigation_blockers(missing_operator_motion["blockers"]) == ["native_operator_motion_status_missing"]

    write_status(
        "autonomy",
        True,
        global_planner="far",
        planner_map="/maps/active/occupancy.npz",
    )
    far = _native_endpoint_readiness({"mode": "navigating", "global_planner": "far"})
    assert far["ok"] is True
    assert far["global_planner"] == "far"
    assert far["planner_map"] == "/maps/active/occupancy.npz"

    mismatch = _native_endpoint_readiness({"mode": "navigating", "global_planner": "octoplanner3d"})
    assert mismatch["ok"] is False
    assert "native_global_planner_mismatch" in mismatch["blockers"]


@pytest.mark.parametrize(
    ("product", "expected_control_mode"),
    [
        ("teleop", "teleop"),
        ("teleop_avoid", "teleop_avoid"),
        ("map", "teleop"),
        ("nav", "autonomy"),
        ("explore", "autonomy"),
    ],
)
def test_native_endpoint_readiness_is_required_by_product_contract_without_endpoint_env(
    monkeypatch,
    tmp_path,
    product,
    expected_control_mode,
):
    from gateway.services.runtime_status import _native_endpoint_readiness

    status_path = tmp_path / "missing_nav_endpoint_status.json"
    monkeypatch.delenv("LINGTU_COMMAND_OUTPUT_MODE", raising=False)
    monkeypatch.setenv("LINGTU_NAV_STATUS_FILE", str(status_path))
    monkeypatch.setenv("LINGTU_NAV_STATUS_MAX_AGE_S", "30")

    result = _native_endpoint_readiness(
        {"product": product},
        _field_gateway(product),
    )

    assert result["required"] is True
    assert result["ok"] is False
    assert result["status_available"] is False
    assert result["expected_control_mode"] == expected_control_mode
    assert result["blockers"] == ["native_endpoint_status_missing_or_stale"]


def test_navigation_status_treats_diverged_slam_as_lost():
    from gateway.gateway_module import GatewayModule
    from gateway.services.runtime_status import build_navigation_status

    class FakeMux:
        def health(self):
            return {"active_source": "none", "sources": {}}

    gateway = GatewayModule()
    gateway._session_mode = "navigating"
    with gateway._state_lock:
        gateway._odom = {"x": 0.0}
        gateway._mission = {"state": "IDLE"}
        gateway._localization_status = {
            "state": "DIVERGED",
            "confidence": 0.0,
            "reason": "fastlio_velocity_out_of_bounds",
            "fastlio_speed_mps": 2805.0,
            "odom_age_ms": 0.0,
        }
    gateway._all_modules = {"nav.velocity_mux": FakeMux()}

    payload = build_navigation_status(gateway)

    assert payload["localization"]["state"] == "lost"
    assert payload["localization"]["fastlio_speed_mps"] == 2805.0
    assert "localization_lost" in payload["readiness"]["blockers"]
    assert payload["can_accept_goal"] is False


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
        "nav.mission": FakeNavigation(),
        "nav.velocity_mux": FakeMux(),
    }

    payload = build_navigation_status(gateway)

    assert payload["can_accept_goal"] is False
    assert payload["readiness"]["map_artifacts_ok"] is False
    assert "map_artifact_gate_failed" in payload["readiness"]["blockers"]
    assert payload["readiness"]["map_artifact_gate"]["blockers"] == ["metadata.json missing"]


def test_navigation_status_blocks_native_saved_map_before_relocalize():
    from gateway.gateway_module import GatewayModule
    from gateway.services.runtime_status import build_navigation_status

    class FakeMux:
        def health(self):
            return {"active_source": "none", "sources": {}}

    gateway = GatewayModule()
    gateway._icp_quality = 0.03
    gateway._session_snapshot = lambda: {
        "mode": "navigating",
        "active_map": "accept_ready",
        "pending": False,
        "localizer_ready": True,
    }
    with gateway._state_lock:
        gateway._odom = {"x": 219143.1, "y": 421310.9, "frame_id": "odom"}
        gateway._mission = {"state": "IDLE"}
        gateway._localization_status = {
            "state": "TRACKING",
            "confidence": 0.9,
            "health_source": "slam_runtime",
            "backend": "fastlio2",
            "mode": "localization",
            "map_loaded": True,
            "saved_map_relocalization_supported": True,
            "relocalization_state": "idle",
            "relocalization_quality": -1.0,
            "map_odom_tf": {
                "valid": True,
                "frame_id": "map",
                "child_frame_id": "odom",
                "tx": 0.0,
                "ty": 0.0,
                "tz": 0.0,
                "qx": 0.0,
                "qy": 0.0,
                "qz": 0.0,
                "qw": 1.0,
            },
        }
    gateway._all_modules = {"nav.velocity_mux": FakeMux()}

    payload = build_navigation_status(gateway)

    assert payload["localization"]["ready"] is True
    assert payload["can_accept_goal"] is False
    assert "saved_map_relocalization_missing" in payload["readiness"]["blockers"]
    assert "saved_map_relocalization_missing" in payload["reason_codes"]


def test_saved_map_relocalization_gate_fails_closed_without_capability_or_commit():
    from gateway.services.runtime_status import _saved_map_relocalization_missing

    localization = {
        "active_map": "accept_ready",
        "backend": "native_dds",
        "native_mode": "localization",
        "map_loaded": True,
        "saved_map_relocalization_supported": False,
        "relocalization_state": "unsupported",
        "relocalization_quality": -1.0,
    }

    assert _saved_map_relocalization_missing(localization) is True

    localization.update(
        {
            "saved_map_relocalization_supported": True,
            "relocalization_state": "rejected",
            "relocalization_quality": 0.02,
        }
    )
    assert _saved_map_relocalization_missing(localization) is True

    localization["relocalization_state"] = "completed"
    assert _saved_map_relocalization_missing(localization) is False


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
    gateway._all_modules = {"nav.velocity_mux": FakeMux()}

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
    gateway._all_modules = {"nav.velocity_mux": FakeMux()}

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
    gateway._all_modules = {"nav.velocity_mux": FakeMux()}

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


def test_navigation_status_blocks_goal_when_genz_recovery_signal_is_active():
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
            "backend": "genz",
            "state": "TRACKING",
            "confidence": 0.92,
            "health_source": "odom_map_cloud",
            "pose_fresh": True,
            "map_cloud_fresh": True,
            "recovery_signal": "LOC_DIVERGED",
            "recovery_action": "restart_genz_icp",
            "localizer_health": "LIO_TRACKING",
            "odom_age_ms": 120.0,
            "cloud_age_ms": 80.0,
        }
    gateway._all_modules = {"nav.velocity_mux": FakeMux()}

    localization = build_localization_status(gateway)
    navigation = build_navigation_status(gateway)

    assert localization["state"] == "degraded"
    assert localization["ready"] is False
    assert localization["algorithm_healthy"] is False
    assert localization["reasons"] == ["recovery_signal:loc_diverged"]
    assert localization["map_save_source"] is None
    assert navigation["can_accept_goal"] is False
    assert "localization_recovery_active" in navigation["reason_codes"]
    assert "localization_recovery_active" in navigation["readiness"]["blockers"]


def test_goal_route_rejects_stale_localization_without_publishing():
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import GatewayErrorResponse, GoalRequest

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
    from gateway.schemas import ControlCommandResponse, GoalRequest

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
    gateway.on_system_modules({"nav.mission": nav})
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
    gateway._all_modules = {"nav.velocity_mux": FakeMux()}

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
    response = client.get("/api/v1/navigation/status")

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
    assert client.get("/api/v1/navigation").status_code == 404


def test_drift_watchdog_reports_missing_run_plan_without_restart() -> None:
    from gateway.gateway_module import GatewayModule

    gateway = GatewayModule()
    events = []
    gateway.push_event = events.append
    with gateway._state_lock:
        gateway._odom = {"x": 999.0}
        gateway._odom_timestamps.append(123.0)

    reported = gateway._drift_report_divergence(xy=999.0, y_abs=0.0, v=0.0)

    assert reported is True
    assert gateway._odom is None
    assert gateway._odom_timestamps == []
    assert events[-1]["action"] == "restart_unavailable"
    assert events[-1]["reason"] == "run_plan_missing"
    assert "operator_command" not in events[-1]


def test_native_product_drift_report_requires_operator_control() -> None:
    from gateway.gateway_module import GatewayModule

    plan = _field_manifest("nav")
    gateway = GatewayModule(run_plan=plan)
    events = []
    gateway.push_event = events.append
    with gateway._state_lock:
        gateway._odom = {"x": 999.0}
        gateway._odom_timestamps.append(123.0)

    reported = gateway._drift_report_divergence(xy=999.0, y_abs=0.0, v=0.0)

    assert reported is True
    assert gateway._odom is None
    assert gateway._odom_timestamps == []
    assert events[-1]["action"] == "operator_restart_required"
    assert events[-1]["reason"] == "operator_product_control_required"
    assert events[-1]["operator_command"] == ("python -m lingtu.control restart --process slam --env real")


def test_drift_watchdog_report_noops_after_shutdown() -> None:
    from gateway.gateway_module import GatewayModule

    gateway = GatewayModule(run_plan=_field_manifest("nav"))
    gateway._stop_event.set()

    reported = gateway._drift_report_divergence(xy=999.0, y_abs=0.0, v=0.0)

    assert reported is False


def test_runtime_dataflow_route_exposes_product_runtime_observability(monkeypatch):
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import RuntimeDataflowResponse
    from runtime.msgs.nav import Odometry
    from runtime.runtime_interface import TOPICS

    monkeypatch.setenv("LINGTU_PROFILE", "nav")
    monkeypatch.setenv("LINGTU_DATA_SOURCE", "thunder")
    monkeypatch.setenv("LINGTU_RUNTIME_CONTRACT", "real")
    monkeypatch.setenv("LINGTU_COMMAND_SINK", "driver")
    monkeypatch.setenv("LINGTU_SIMULATION_ONLY", "0")

    gateway = GatewayModule()
    endpoint = _endpoint(gateway, "/api/v1/runtime/dataflow")
    payload = asyncio.run(endpoint())
    RuntimeDataflowResponse.model_validate(payload)
    initial_payload = payload

    assert payload["schema_version"] == 1
    assert payload["runtime_contract"] == "real"
    assert payload["runtime_boundary"]["runtime_contract"] == "real"
    assert payload["ros2_topic_required"] is False
    assert payload["transport_layers"]["native_dds"]["primary"] is True
    assert payload["transport_layers"]["module_port_bus"]["primary"] is False
    assert payload["transport_layers"]["ros2_adapter"]["primary"] is False
    assert payload["motion_path"]["motion_owner"] == "nav"
    assert payload["motion_path"]["final_velocity_writer"] == "nav"
    assert payload["motion_path"]["actuator_owner"] == "driver"
    assert [stage["owner"] for stage in payload["motion_path"]["stages"]] == [
        "host",
        "slam",
        "traversability",
        "nav",
        "driver",
    ]
    assert payload["motion_path"]["control_paths"] == {
        "autonomy_goal": [
            "CommandIngress",
            "GlobalPlanner",
            "LocalPlanner",
            "PathFollower",
            "CommandSafety",
        ],
        "external_path": [
            "PathIngress",
            "LocalPlanner",
            "PathFollower",
            "CommandSafety",
        ],
        "teleop_avoid": [
            "OperatorIntent",
            "LocalPlanner",
            "PathFollower",
            "CommandSafety",
        ],
        "teleop": ["OperatorCommand", "CommandSafety"],
    }
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

    lidar_flow = {stage["name"]: stage for stage in topics[TOPICS.lidar_scan]["data_flow_stages"]}
    assert "endpoint_adapter" in lidar_flow
    assert "input" in lidar_flow["endpoint_adapter"]["roles"]

    stages = {stage["name"]: stage for stage in initial_payload["stage_evidence"]}
    assert "global_planning" in stages
    assert stages["global_planning"]["owner"] == "lingtu_navigation_or_planner_backend"
    assert TOPICS.odometry in stages["global_planning"]["inputs"]
    assert TOPICS.global_path in stages["global_planning"]["outputs"]
    assert TOPICS.odometry in stages["global_planning"]["not_live_inputs"]
    odom_stage_input = next(
        item for item in stages["global_planning"]["input_evidence"] if item["token"] == TOPICS.odometry
    )
    assert odom_stage_input["observable"] is True
    assert odom_stage_input["live"] is False
    assert odom_stage_input["reason"] == "metadata_only"
    assert stages["command_boundary"]["output_evidence"][0]["kind"] == "runtime_boundary"
    assert stages["command_boundary"]["output_evidence"][0]["reason"] == "runtime_boundary_declared"

    cmd_vel_communication = topics[TOPICS.cmd_vel]["communication"]
    assert cmd_vel_communication["allowed"] is True
    assert cmd_vel_communication["arbitrary_publish_supported"] is False
    assert {interface["path"] for interface in cmd_vel_communication["interfaces"]} == {
        "/api/v1/cmd_vel",
        "/api/v1/stop",
    }


def test_runtime_dataflow_route_validates_active_saved_octomap_artifact(
    monkeypatch,
    tmp_path,
):
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import RuntimeDataflowResponse
    from runtime.runtime_interface import TOPICS

    map_root = tmp_path / "maps"
    _write_active_same_source_octomap(map_root)
    monkeypatch.setenv("NAV_MAP_DIR", str(map_root))
    monkeypatch.setenv("LINGTU_RUNTIME_CONTRACT", "real")

    gateway = GatewayModule()
    from maps.modules.service import MapsModule

    gateway._map_mgr = MapsModule(
        map_dir=str(map_root),
        data_dir=str(tmp_path / "maps-data"),
    )
    gateway._map_mgr.setup()
    endpoint = _endpoint(gateway, "/api/v1/runtime/dataflow")
    payload = asyncio.run(endpoint())
    RuntimeDataflowResponse.model_validate(payload)

    stages = {stage["name"]: stage for stage in payload["stage_evidence"]}
    global_stage = stages["global_planning"]
    octomap_input = next(item for item in global_stage["input_evidence"] if item["token"] == "artifact:octomap")

    assert "artifact:octomap" in global_stage["inputs"]
    assert "artifact:octomap" not in global_stage["missing_inputs"]
    assert octomap_input["kind"] == "artifact"
    assert octomap_input["observable"] is True
    assert octomap_input["live"] is False
    assert octomap_input["reason"] == "saved_map_artifact_ok"
    assert octomap_input["artifact_gate"]["ok"] is True
    assert octomap_input["artifact_gate"]["artifacts"]["octomap"]["sha256_ok"] is True
    assert octomap_input["artifact_gate"]["artifacts"]["octomap"]["source_map_sha256_matches_map"] is True
    assert octomap_input["artifact_gate"]["ros2_topic_required"] is False
    assert TOPICS.global_path in global_stage["outputs"]


def test_runtime_dataflow_route_marks_missing_active_octomap_artifact(
    monkeypatch,
    tmp_path,
):
    from gateway.gateway_module import GatewayModule

    map_root = tmp_path / "maps"
    map_root.mkdir()
    monkeypatch.setenv("NAV_MAP_DIR", str(map_root))
    monkeypatch.setenv("LINGTU_RUNTIME_CONTRACT", "real")

    gateway = GatewayModule()
    from maps.modules.service import MapsModule

    gateway._map_mgr = MapsModule(
        map_dir=str(map_root),
        data_dir=str(tmp_path / "maps-data"),
    )
    gateway._map_mgr.setup()
    endpoint = _endpoint(gateway, "/api/v1/runtime/dataflow")
    payload = asyncio.run(endpoint())

    stages = {stage["name"]: stage for stage in payload["stage_evidence"]}
    global_stage = stages["global_planning"]
    octomap_input = next(item for item in global_stage["input_evidence"] if item["token"] == "artifact:octomap")

    assert "artifact:octomap" in global_stage["missing_inputs"]
    assert octomap_input["kind"] == "artifact"
    assert octomap_input["observable"] is False
    assert octomap_input["live"] is False
    assert octomap_input["reason"] == "saved_map_artifact_missing_or_invalid"
    assert octomap_input["artifact_gate"]["ok"] is False
    assert octomap_input["artifact_gate"]["ros2_topic_required"] is False
    assert "active map unavailable from maps service" in octomap_input["artifact_gate"]["blockers"]


def test_runtime_dataflow_route_is_read_only_for_module_ports(monkeypatch):
    from gateway.gateway_module import GatewayModule

    monkeypatch.setenv("LINGTU_RUNTIME_CONTRACT", "real")

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
    from gateway.gateway_module import GatewayModule
    from runtime.msgs.nav import Odometry
    from runtime.runtime_interface import TOPICS

    monkeypatch.setenv("LINGTU_RUNTIME_CONTRACT", "real")
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
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import RuntimeDataflowTopicDetailResponse
    from runtime.msgs.nav import Odometry
    from runtime.runtime_interface import TOPICS

    monkeypatch.setenv("LINGTU_RUNTIME_CONTRACT", "real")

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
    assert {channel["transport"] for channel in payload["inspection"]["payload_interfaces"]} >= {
        "gateway_rest",
        "gateway_sse",
    }
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
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import (
        RuntimeDataflowSubscribeRequest,
        RuntimeDataflowSubscribeResponse,
    )
    from runtime.msgs.nav import Odometry
    from runtime.runtime_interface import TOPICS

    monkeypatch.setenv("LINGTU_RUNTIME_CONTRACT", "real")

    gateway = GatewayModule()
    endpoint = _endpoint(gateway, "/api/v1/runtime/dataflow/subscribe")
    gateway.odometry._deliver(Odometry())

    payload = asyncio.run(endpoint(RuntimeDataflowSubscribeRequest(selector="odometry")))
    RuntimeDataflowSubscribeResponse.model_validate(payload)

    assert payload["ok"] is True
    assert payload["read_only"] is True
    assert payload["ros2_topic_required"] is False
    assert payload["arbitrary_publish_supported"] is False
    assert payload["publishes"] == []
    assert payload["selector"] == "odometry"
    assert payload["topic"] == TOPICS.odometry
    assert payload["event_types"] == ["odometry"]
    assert payload["stream_url"] == "/api/v1/events?topic=%2Fslam%2Fodometry"
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

    from explore.traversable_frontier import TraversableFrontierModule
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import (
        RuntimeDataflowResponse,
        RuntimeDataflowSubscribeRequest,
        RuntimeDataflowSubscribeResponse,
        RuntimeDataflowTopicDetailResponse,
    )
    from runtime.msgs.geometry import Pose
    from runtime.msgs.nav import Odometry
    from runtime.runtime_interface import TOPICS
    # ^ Cross-layer: gateway test imports from nav/ for runtime dataflow testing.
    #   Acceptable: test needs a concrete nav module to exercise the
    #   RuntimeDataflow endpoint with real module state.  Gateway tests
    #   serve as integration tests for the full stack through the HTTP layer.

    monkeypatch.setenv("LINGTU_RUNTIME_CONTRACT", "real")

    gateway = GatewayModule()
    gateway.setup()
    routes = {route.path: route.endpoint for route in gateway._app.routes if hasattr(route, "endpoint")}
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
    evidence_by_token = {item["token"]: item for item in preview_stage["input_evidence"]}
    fused_evidence = evidence_by_token["module:TraversableFrontierModule.fused_cost"]
    assert fused_evidence["kind"] == "module_port"
    assert fused_evidence["live"] is True
    assert fused_evidence["module_ports"][0]["module"] == "TraversableFrontierModule"
    assert preview_stage["observable"] is True
    assert preview_stage["live"] is True
    assert preview_stage["missing_inputs"] == []
    assert topics[TOPICS.frontier_candidate]["communication"]["allowed"] is False
    assert topics[TOPICS.frontier_candidate]["communication"]["arbitrary_publish_supported"] is False
    assert snapshot["control_boundary"]["arbitrary_publish_supported"] is False

    detail = asyncio.run(routes["/api/v1/runtime/dataflow/topic"](topic="frontier_candidate"))
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

    list_detail = asyncio.run(routes["/api/v1/runtime/dataflow/topic"](topic="traversable_frontiers"))
    RuntimeDataflowTopicDetailResponse.model_validate(list_detail)
    assert list_detail["ok"] is True
    assert list_detail["topic"]["topic"] == TOPICS.traversable_frontiers
    assert list_detail["inspection"]["live"] is True
    assert list_detail["inspection"]["communicate"] is False
    assert list_detail["inspection"]["write_interfaces"] == []
    assert isinstance(list_detail["inspection"]["latest_payload"], list)
    assert list_detail["inspection"]["latest_payload"]
    assert list_detail["inspection"]["latest_payload"][0]["source"] == "traversable_frontier"
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
        routes["/api/v1/runtime/dataflow/subscribe"](RuntimeDataflowSubscribeRequest(selector="frontier_candidate"))
    )
    RuntimeDataflowSubscribeResponse.model_validate(subscription)
    assert subscription["ok"] is True
    assert subscription["read_only"] is True
    assert subscription["publishes"] == []
    assert subscription["arbitrary_publish_supported"] is False
    assert subscription["event_types"] == ["frontier_candidate"]
    assert subscription["stream_url"] == "/api/v1/events?topic=%2Fnav%2Ffrontier_candidate"


def test_runtime_dataflow_subscribe_route_rejects_unknown_selector_without_publish(
    monkeypatch,
):
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import RuntimeDataflowSubscribeRequest

    monkeypatch.setenv("LINGTU_RUNTIME_CONTRACT", "real")

    gateway = GatewayModule()
    endpoint = _endpoint(gateway, "/api/v1/runtime/dataflow/subscribe")

    payload = asyncio.run(endpoint(RuntimeDataflowSubscribeRequest(selector="not_a_stream")))

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
    from gateway.gateway_module import GatewayModule
    from runtime.msgs.nav import Odometry
    from runtime.runtime_interface import TOPICS

    monkeypatch.setenv("LINGTU_RUNTIME_CONTRACT", "real")

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
    from gateway.gateway_module import GatewayModule
    from runtime.runtime_interface import TOPICS

    monkeypatch.setenv("LINGTU_RUNTIME_CONTRACT", "real")

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
    interfaces = payload["inspection"]["write_interfaces"]
    assert all(item["final_output_confirmed"] is False for item in interfaces)
    assert {item["response_evidence"] for item in interfaces} == {
        "source_request_accepted_only",
        "command_ack_not_driver_execution",
    }
    assert all(
        channel.get("event_type") != "command_ack" for channel in payload["topic"]["observability"]["gateway_channels"]
    )


@pytest.mark.parametrize(
    (
        "contract",
        "data_source",
        "command_sink",
        "simulation_only",
    ),
    [
        (
            "thunder",
            "thunder",
            "driver",
            "0",
        ),
        (
            "mujoco_fastlio2_live",
            "mujoco_fastlio2_live",
            "mujoco_velocity_adapter",
            "1",
        ),
    ],
)
def test_runtime_dataflow_exposes_all_contract_streams_for_real_and_sim(
    monkeypatch,
    contract: str,
    data_source: str,
    command_sink: str,
    simulation_only: str,
):
    from gateway.gateway_module import GatewayModule
    from runtime.runtime_interface import runtime_data_flow_topics

    monkeypatch.setenv("LINGTU_RUNTIME_CONTRACT", contract)
    monkeypatch.setenv("LINGTU_DATA_SOURCE", data_source)
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
    from gateway.gateway_module import GatewayModule
    from runtime.runtime_interface import TOPICS

    monkeypatch.setenv("LINGTU_RUNTIME_CONTRACT", "real")

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
    monkeypatch.setenv("LINGTU_COMMAND_SINK", "mujoco_velocity_adapter")
    monkeypatch.setenv("LINGTU_SIMULATION_ONLY", "1")
    sim_payload = asyncio.run(endpoint())

    monkeypatch.setenv("LINGTU_RUNTIME_CONTRACT", "real")
    monkeypatch.setenv("LINGTU_DATA_SOURCE", "thunder")
    monkeypatch.setenv("LINGTU_COMMAND_SINK", "driver")
    monkeypatch.setenv("LINGTU_SIMULATION_ONLY", "0")
    real_payload = asyncio.run(endpoint())

    assert sim_payload["runtime_contract"] == "mujoco_fastlio2_live"
    assert sim_payload["runtime_boundary"]["simulation_only"] is True
    assert sim_payload["runtime_boundary"]["command_sink"] == "mujoco_velocity_adapter"
    assert real_payload["runtime_contract"] == "real"
    assert real_payload["runtime_boundary"]["simulation_only"] is False
    assert real_payload["runtime_boundary"]["command_sink"] == "driver"
    sim_topics = {item["topic"] for item in sim_payload["topics"]}
    real_topics = {item["topic"] for item in real_payload["topics"]}
    assert "/lidar/raw_frame" in sim_topics
    assert "/lidar/raw_frame" in real_topics
    assert sim_topics != real_topics


def test_runtime_dataflow_topic_route_reports_unknown_selector(monkeypatch):
    from gateway.gateway_module import GatewayModule

    monkeypatch.setenv("LINGTU_RUNTIME_CONTRACT", "real")

    gateway = GatewayModule()
    endpoint = _endpoint(gateway, "/api/v1/runtime/dataflow/topic")

    payload = asyncio.run(endpoint(topic="not_a_lingtu_stream"))

    assert payload["ok"] is False
    assert payload["error"] == "runtime_topic_not_found"
    assert payload["inspection"]["observable"] is False
    assert payload["inspection"]["communicate"] is False
    assert payload["inspection"]["ros2_topic_required"] is False
    assert "/slam/odometry" in payload["available_topics"]


@pytest.mark.skipif(not _NUMPY_IMPORT_SAFE, reason=NUMPY_UNSAFE_REASON)
def test_runtime_dataflow_reports_live_samples_for_field_topics(monkeypatch):
    import numpy as np

    from diagnostics.field.gateway_acceptance import (
        FIELD_LIVE_TOPICS,
        FIELD_REQUIRED_LIVE_STAGE_NAMES,
    )
    from gateway.gateway_module import GatewayModule
    from runtime.msgs.geometry import Twist
    from runtime.msgs.nav import Odometry, Path
    from runtime.msgs.sensor import PointCloud2
    from runtime.runtime_interface import TOPICS

    monkeypatch.setenv("LINGTU_RUNTIME_CONTRACT", "real")

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
        assert any(port["msg_count"] > 0 for port in observability["module_port_candidates"]), topic

    stages = {stage["name"]: stage for stage in payload["stage_evidence"]}
    for stage_name in FIELD_REQUIRED_LIVE_STAGE_NAMES:
        assert stages[stage_name]["live"] is True, stage_name
        assert stages[stage_name]["status"] == "live", stage_name

    assert topics[TOPICS.cmd_vel]["communication"]["arbitrary_publish_supported"] is False


def test_runtime_dataflow_topic_route_answers_every_product_observable_stream_without_ros2(
    monkeypatch,
):
    from diagnostics.field.gateway_acceptance import PRODUCT_OBSERVABLE_TOPICS
    from gateway.gateway_module import GatewayModule

    monkeypatch.setenv("LINGTU_RUNTIME_CONTRACT", "real")

    gateway = GatewayModule()
    endpoint = _endpoint(gateway, "/api/v1/runtime/dataflow/topic")

    for topic in PRODUCT_OBSERVABLE_TOPICS:
        payload = asyncio.run(endpoint(topic=topic))

        assert payload["ok"] is True, topic
        assert payload["topic"]["topic"] == topic
        assert payload["inspection"]["ros2_topic_required"] is False, topic
        assert payload["inspection"]["arbitrary_publish_supported"] is False, topic
        assert payload["inspection"]["payload_available"] is True, topic
        assert payload["inspection"]["payload_interfaces"] or payload["inspection"]["stream_interfaces"], topic


def test_runtime_dataflow_subscribe_route_covers_every_gateway_realtime_stream(
    monkeypatch,
):
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import RuntimeDataflowSubscribeRequest

    monkeypatch.setenv("LINGTU_RUNTIME_CONTRACT", "real")

    gateway = GatewayModule()
    gateway.setup()
    routes = {route.path: route.endpoint for route in gateway._app.routes if hasattr(route, "endpoint")}
    dataflow_endpoint = routes["/api/v1/runtime/dataflow"]
    subscribe_endpoint = routes["/api/v1/runtime/dataflow/subscribe"]

    snapshot = asyncio.run(dataflow_endpoint())
    stream_topics = [
        topic["topic"]
        for topic in snapshot["topics"]
        if any(stream.get("transport") == "gateway_sse" for stream in topic["inspection"].get("stream_interfaces", []))
    ]

    assert stream_topics
    for topic in stream_topics:
        payload = asyncio.run(subscribe_endpoint(RuntimeDataflowSubscribeRequest(selector=topic)))

        assert payload["ok"] is True, topic
        assert payload["read_only"] is True, topic
        assert payload["ros2_topic_required"] is False, topic
        assert payload["arbitrary_publish_supported"] is False, topic
        assert payload["publishes"] == [], topic
        assert payload["topic"] == topic
        assert payload["transport"] == "gateway_sse", topic
        assert payload["stream_url"] == f"/api/v1/events?topic={topic.replace('/', '%2F')}", topic
        assert payload["event_types"], topic


def test_localization_status_preserves_relocalization_overlap_evidence():
    from gateway.services.runtime_status import build_localization_status_from_parts

    diagnostics = {
        "state": "TRACKING",
        "confidence": 0.9,
        "health_source": "slam_runtime",
        "relocalization_refine_backend": "fixed_transform_seed_check",
        "relocalization_refine_iterations": 0,
        "relocalization_refine_inliers": 292,
        "relocalization_refine_input_points": 485,
        "relocalization_refine_evaluated_points": 298,
        "relocalization_min_inliers": 30,
        "relocalization_min_evaluated_points": 100,
        "relocalization_refine_support_ratio": 298 / 485,
        "relocalization_refine_overlap_inlier_ratio": 292 / 298,
        "relocalization_refine_converged": True,
        "relocalization_refine_pos_cov_trace": 0.01,
    }

    payload = build_localization_status_from_parts(
        odometry={"x": 0.0, "y": 0.0, "frame_id": "odom"},
        session={"mode": "navigating", "localizer_ready": True},
        icp_quality=0.01,
        status=diagnostics,
    )

    assert payload["relocalization_refine_backend"] == "fixed_transform_seed_check"
    assert payload["relocalization_refine_iterations"] == 0
    assert payload["relocalization_refine_inliers"] == 292
    assert payload["relocalization_refine_input_points"] == 485
    assert payload["relocalization_refine_evaluated_points"] == 298
    assert payload["relocalization_min_inliers"] == 30
    assert payload["relocalization_min_evaluated_points"] == 100
    assert payload["relocalization_refine_support_ratio"] == pytest.approx(298 / 485)
    assert payload["relocalization_refine_overlap_inlier_ratio"] == pytest.approx(292 / 298)
    assert payload["relocalization_refine_converged"] is True
