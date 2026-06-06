from __future__ import annotations

import asyncio
import time

import pytest


pytest.importorskip("fastapi")


def _endpoint(gateway, path: str):
    return next(route.endpoint for route in gateway._app.routes if route.path == path)


def test_liveness_and_devices_routes_validate_response_contracts():
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import DevicesResponse, LivenessResponse

    gateway = GatewayModule()
    gateway.setup()

    liveness = asyncio.run(_endpoint(gateway, "/health")())
    devices = asyncio.run(_endpoint(gateway, "/api/v1/devices")())

    live_model = LivenessResponse.model_validate(liveness)
    devices_model = DevicesResponse.model_validate(devices)

    assert live_model.status == "ok"
    assert live_model.ts > 0
    assert devices_model.manager == "not_loaded"
    assert devices_model.devices == []


def test_health_and_devices_routes_tolerate_missing_module_inventory():
    from gateway.gateway_module import GatewayModule

    gateway = GatewayModule()
    gateway.setup()
    del gateway._all_modules

    devices = asyncio.run(_endpoint(gateway, "/api/v1/devices")())
    health = asyncio.run(_endpoint(gateway, "/api/v1/health")())

    assert devices["manager"] == "not_loaded"
    assert devices["devices"] == []
    assert health["modules"] == {}
    assert health["modules_ok"] == 0
    assert health["modules_fail"] == 0


def test_health_schema_keeps_top_level_app_contract_flexible():
    from gateway.schemas import HealthResponse

    model = HealthResponse.model_validate(
        {
            "status": "ok",
            "modules_ok": 3,
            "modules_fail": 0,
            "gateway": {
                "port": 5050,
                "mode": "manual",
                "traffic": {"sse": {"clients": 0}},
                "commands": {"idempotency_supported": True},
            },
            "teleop": {"active": False, "clients": 0},
            "sensors": {"camera": {"status": "idle"}},
            "slam_hz": 0.0,
            "map_points": 0,
            "has_odom": False,
            "modules": {"GatewayModule": "ok"},
            "brainstem": {"status": "unavailable"},
            "future_field": "preserved",
        }
    )

    assert model.status == "ok"
    assert model.gateway["port"] == 5050
    assert model.teleop.clients == 0
    assert model.model_extra["future_field"] == "preserved"


def test_health_uses_live_point_count_and_module_slam_rate(monkeypatch):
    from gateway.gateway_module import GatewayModule

    class _Slam:
        def health(self):
            return {
                "ports_out": {
                    "odometry": {
                        "msg_count": 10,
                        "rate_hz": 4.25,
                    }
                }
            }

    gateway = GatewayModule()
    gateway.setup()
    gateway._all_modules = {"SlamBridgeModule": _Slam()}
    gateway._map_points = [(0.0, 0.0, 0.0)] * 42
    gateway._map_cloud_count = 99

    def _fail_shell_hz():
        raise AssertionError("health should prefer module port rate")

    monkeypatch.setattr(gateway, "_get_slam_hz_cached", _fail_shell_hz)

    health = asyncio.run(_endpoint(gateway, "/api/v1/health")())

    assert health["map_points"] == 42
    assert health["slam_hz"] == 4.2
    assert health["sensors"]["slam"]["hz"] == 4.2


def test_health_falls_back_to_cached_slam_rate_when_module_rate_missing(monkeypatch):
    from gateway.gateway_module import GatewayModule

    gateway = GatewayModule()
    gateway.setup()
    gateway._all_modules = {}
    monkeypatch.setattr(gateway, "_get_slam_hz_cached", lambda: 3.7)

    health = asyncio.run(_endpoint(gateway, "/api/v1/health")())

    assert health["slam_hz"] == 3.7


def test_gateway_slam_rate_uses_existing_odometry_window(monkeypatch):
    import gateway.gateway_module as gateway_module

    gateway = gateway_module.GatewayModule()
    gateway._odom_timestamps = [100.0 + i * 0.1 for i in range(20)]

    monkeypatch.setattr(gateway_module.time, "time", lambda: 101.95)
    assert gateway._get_slam_hz_cached() == 10.0

    monkeypatch.setattr(gateway_module.time, "time", lambda: 104.0)
    assert gateway._get_slam_hz_cached() == 0.0


def test_health_reports_camera_idle_reason_from_bridge_health(monkeypatch):
    from gateway.gateway_module import GatewayModule

    class _CameraBridge:
        def health(self):
            return {
                "backend": "dds",
                "ports_out": {
                    "color_image": {
                        "msg_count": 0,
                        "rate_hz": 0.0,
                        "stale_ms": -1.0,
                    },
                    "depth_image": {
                        "msg_count": 0,
                        "rate_hz": 0.0,
                        "stale_ms": -1.0,
                    },
                    "camera_info": {"msg_count": 0},
                },
            }

    gateway = GatewayModule()
    gateway.setup()
    gateway._all_modules = {"CameraBridgeModule": _CameraBridge()}
    monkeypatch.setattr(gateway, "_get_slam_hz_cached", lambda: 0.0)

    health = asyncio.run(_endpoint(gateway, "/api/v1/health")())
    camera = health["sensors"]["camera"]

    assert camera["status"] == "idle"
    assert camera["available"] is False
    assert camera["reason"] == "no_color_frames"
    assert camera["frames"] == 0
    assert camera["jpeg"]["cached"] is False


def test_health_default_skips_unrelated_module_diagnostics(monkeypatch):
    from gateway.gateway_module import GatewayModule

    class _SlowUnrelated:
        def health(self):
            raise AssertionError("default health should not inspect every module")

    gateway = GatewayModule()
    gateway.setup()
    gateway._all_modules = {"VectorMemoryModule": _SlowUnrelated()}
    monkeypatch.setattr(gateway, "_get_slam_hz_cached", lambda: 0.0)

    health = asyncio.run(_endpoint(gateway, "/api/v1/health")())

    assert health["modules_ok"] == 1
    assert health["modules_fail"] == 0
    assert health["modules"]["VectorMemoryModule"] == "ok"
    assert health["gateway"]["diagnostic_details"] is False


def test_health_details_query_runs_full_module_diagnostics(monkeypatch):
    from gateway.gateway_module import GatewayModule

    class _BrokenUnrelated:
        def health(self):
            raise RuntimeError("slow diagnostic failed")

    gateway = GatewayModule()
    gateway.setup()
    gateway._all_modules = {"VectorMemoryModule": _BrokenUnrelated()}
    monkeypatch.setattr(gateway, "_get_slam_hz_cached", lambda: 0.0)

    health = asyncio.run(_endpoint(gateway, "/api/v1/health")(details=True))

    assert health["modules_ok"] == 0
    assert health["modules_fail"] == 1
    assert health["modules"]["VectorMemoryModule"] == "error"
    assert health["gateway"]["diagnostic_details"] is True


def test_health_uses_cached_brainstem_probe_for_short_app_polling_window(monkeypatch):
    from gateway.gateway_module import GatewayModule
    import gateway.routes.status as status_routes

    gateway = GatewayModule()
    gateway.setup()
    gateway._all_modules = {}
    gateway._brainstem_health_cache_ttl_s = 10.0
    gateway._brainstem_health_cache = {"status": "connected", "fsm": "GROUNDED"}
    gateway._brainstem_health_cache_ts = time.monotonic()
    monkeypatch.setattr(gateway, "_get_slam_hz_cached", lambda: 0.0)

    calls = {"count": 0}

    def _probe():
        calls["count"] += 1
        return {"status": "connected", "fsm": "GROUNDED"}

    monkeypatch.setattr(status_routes, "_probe_brainstem", _probe)

    endpoint = _endpoint(gateway, "/api/v1/health")
    first = asyncio.run(endpoint())
    second = asyncio.run(endpoint())

    assert calls["count"] == 0
    assert first["brainstem"]["status"] == "connected"
    assert first["brainstem"]["cached"] is True
    assert second["brainstem"]["status"] == "connected"
    assert second["brainstem"]["cached"] is True
    assert second["brainstem"]["cache_age_s"] >= 0.0


def test_health_returns_stale_brainstem_cache_and_refreshes_in_background(monkeypatch):
    from gateway.gateway_module import GatewayModule
    import gateway.routes.status as status_routes

    gateway = GatewayModule()
    gateway.setup()
    gateway._all_modules = {}
    gateway._brainstem_health_cache_ttl_s = 10.0
    gateway._brainstem_health_cache = {"status": "connected", "fsm": "OLD"}
    gateway._brainstem_health_cache_ts = time.monotonic() - 30.0
    monkeypatch.setattr(gateway, "_get_slam_hz_cached", lambda: 0.0)

    calls = {"count": 0}

    def _probe():
        calls["count"] += 1
        return {"status": "connected", "fsm": f"STATE_{calls['count']}"}

    monkeypatch.setattr(status_routes, "_probe_brainstem", _probe)

    endpoint = _endpoint(gateway, "/api/v1/health")
    first = asyncio.run(endpoint())

    deadline = time.time() + 1.0
    while time.time() < deadline and calls["count"] < 1:
        time.sleep(0.01)
    assert calls["count"] == 1

    second = asyncio.run(endpoint())

    assert first["brainstem"]["fsm"] == "OLD"
    assert first["brainstem"]["cached"] is True
    assert first["brainstem"]["stale"] is True
    assert first["brainstem"]["refreshing"] is True
    assert second["brainstem"]["fsm"] == "STATE_1"
    assert second["brainstem"]["cached"] is True
    assert "stale" not in second["brainstem"]


def test_health_default_brainstem_probe_does_not_block_without_cache(monkeypatch):
    from gateway.gateway_module import GatewayModule
    import gateway.routes.status as status_routes

    gateway = GatewayModule()
    gateway.setup()
    gateway._all_modules = {}
    gateway._brainstem_health_cache_ttl_s = 10.0
    monkeypatch.setattr(gateway, "_get_slam_hz_cached", lambda: 0.0)

    calls = {"count": 0}

    def _probe():
        calls["count"] += 1
        time.sleep(0.2)
        return {"status": "connected", "fsm": "GROUNDED"}

    monkeypatch.setattr(status_routes, "_probe_brainstem", _probe)

    endpoint = _endpoint(gateway, "/api/v1/health")
    started = time.perf_counter()
    health = asyncio.run(endpoint())
    elapsed = time.perf_counter() - started

    assert elapsed < 0.1
    assert health["brainstem"]["status"] == "unknown"
    assert health["brainstem"]["reason"] == "probe_pending"
    assert health["brainstem"]["refreshing"] is True
    if gateway._brainstem_health_refresh_thread is not None:
        gateway._brainstem_health_refresh_thread.join(timeout=1.0)


def test_health_details_query_waits_for_live_brainstem_probe(monkeypatch):
    from gateway.gateway_module import GatewayModule
    import gateway.routes.status as status_routes

    gateway = GatewayModule()
    gateway.setup()
    gateway._all_modules = {}
    gateway._brainstem_health_cache_ttl_s = 10.0
    monkeypatch.setattr(gateway, "_get_slam_hz_cached", lambda: 0.0)

    calls = {"count": 0}

    def _probe():
        calls["count"] += 1
        return {"status": "connected", "fsm": "GROUNDED"}

    monkeypatch.setattr(status_routes, "_probe_brainstem", _probe)

    endpoint = _endpoint(gateway, "/api/v1/health")
    health = asyncio.run(endpoint(details=True))

    assert calls["count"] == 1
    assert health["brainstem"]["status"] == "connected"
    assert health["brainstem"]["cached"] is False
