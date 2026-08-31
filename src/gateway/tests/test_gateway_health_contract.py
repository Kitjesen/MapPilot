from __future__ import annotations

import asyncio
import json
import os
import time

import pytest
from runtime.msgs.numpy_compat import np

pytest.importorskip("fastapi")


def _endpoint(gateway, path: str):
    return next(route.endpoint for route in gateway._app.routes if route.path == path)


def test_liveness_route_validates_response_contract():
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import LivenessResponse

    gateway = GatewayModule()
    gateway.setup()

    liveness = asyncio.run(_endpoint(gateway, "/health")())
    live_model = LivenessResponse.model_validate(liveness)

    assert live_model.status == "ok"
    assert live_model.ts > 0


def test_health_route_tolerates_missing_module_inventory():
    from gateway.gateway_module import GatewayModule

    gateway = GatewayModule()
    gateway.setup()
    del gateway._all_modules

    health = asyncio.run(_endpoint(gateway, "/api/v1/health")())
    assert health["modules"] == {}
    assert health["modules_ok"] == 0
    assert health["modules_fail"] == 0


def test_metrics_route_returns_operator_snapshot():
    from gateway.gateway_module import GatewayModule

    gateway = GatewayModule()
    gateway.setup()
    gateway._localization_status = {
        "state": "TRACKING",
        "backend": "native_dds",
        "mode": "localization",
        "processed_scan_hz": 9.5,
        "lidar_input_hz": 9.1,
        "imu_input_hz": 200.0,
        "slam_tick_hz": 50.0,
    }
    gateway._cloud_viewer.replace_map_points(
        np.asarray([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]], dtype=np.float32)
    )

    metrics = asyncio.run(_endpoint(gateway, "/api/v1/metrics")())

    assert metrics["ok"] is True
    assert metrics["gateway"]["port"] == gateway._port
    assert metrics["slam"]["hz"] == 9.5
    assert metrics["slam"]["backend"] == "native_dds"
    assert metrics["map"]["points"] == 2
    assert "traffic" in metrics
    assert "commands" in metrics


def test_service_status_marks_current_gateway_http_observed():
    from gateway.gateway_module import GatewayModule

    gateway = GatewayModule()
    gateway.setup()

    payload = asyncio.run(_endpoint(gateway, "/api/v1/services/status")("gateway,lingtu"))

    assert payload["services"] == {"gateway": "running", "lingtu": "running"}
    assert payload["readiness"] == {
        "ok": True,
        "ready": True,
        "selected": ["gateway", "lingtu"],
        "ready_services": ["gateway", "lingtu"],
        "not_ready_services": [],
        "blockers": [],
        "blocker_count": 0,
    }
    for name in ("gateway", "lingtu"):
        detail = payload["service_details"][name]
        assert detail["ready"] is True
        assert detail["blockers"] == []
        assert detail["observed"]["http"] == {
            "ok": True,
            "checked": True,
            "enabled": True,
            "source": "current_gateway_route",
            "path": "/api/v1/services/status",
            "blockers": [],
        }


def test_service_status_summarizes_field_readiness_blockers(monkeypatch, tmp_path):
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import ServiceStatusResponse

    evidence = tmp_path / "service_readiness.json"
    evidence.write_text(
        json.dumps(
            {
                "schema": "lingtu.thunder.service_readiness.v1",
                "summary": {"ok": True, "blockers": [], "blocker_count": 0},
                "systemd": {
                    "lt-lidar.service": {"active_state": "active", "sub_state": "running"},
                    "lt-camera.service": {"active_state": "inactive", "sub_state": "dead"},
                },
            }
        ),
        encoding="utf-8",
    )
    monkeypatch.setenv("LINGTU_SERVICE_READINESS_JSON", str(evidence))

    gateway = GatewayModule()
    gateway.setup()

    payload = asyncio.run(_endpoint(gateway, "/api/v1/services/status")("lidar,camera"))
    model = ServiceStatusResponse.model_validate(payload)

    assert model.readiness["ok"] is False
    assert model.readiness["ready_services"] == ["lidar"]
    assert model.readiness["not_ready_services"] == ["camera"]
    assert model.readiness["blockers"] == ["camera:status_stopped"]
    assert model.service_details["lidar"]["observed"]["field_readiness"]["source"] == "field_readiness"


def test_service_status_can_require_field_readiness_evidence(monkeypatch, tmp_path):
    from gateway.gateway_module import GatewayModule

    missing = tmp_path / "missing_service_readiness.json"
    monkeypatch.setenv("LINGTU_SERVICE_READINESS_JSON", str(missing))
    monkeypatch.setenv("LINGTU_REQUIRE_FIELD_SERVICE_READINESS", "1")

    gateway = GatewayModule()
    gateway.setup()

    payload = asyncio.run(_endpoint(gateway, "/api/v1/services/status")("lidar"))

    assert payload["field_readiness"]["required"] is True
    assert payload["field_readiness"]["available"] is False
    assert payload["readiness"]["ok"] is False
    assert payload["readiness"]["blockers"] == [
        "lidar:status_unknown",
        f"field_readiness:field_readiness_missing:{missing}",
    ]


def test_service_status_includes_fresh_field_readiness_summary(monkeypatch, tmp_path):
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import ServiceStatusResponse

    evidence = tmp_path / "service_readiness.json"
    evidence.write_text(
        json.dumps(
            {
                "schema": "lingtu.thunder.service_readiness.v1",
                "stamp_s": 123.0,
                    "summary": {
                    "ok": False,
                    "blockers": [
                        "native_binaries:native_binary_missing_or_not_executable:camera_dds:/opt/lingtu/current/build/camera_dds/lingtu_camera_dds"
                    ],
                        "blocker_count": 1,
                    },
                    "systemd": {
                        "lt-lidar.service": {
                            "active_state": "active",
                            "sub_state": "running",
                        }
                    },
                }
        ),
        encoding="utf-8",
    )
    now = time.time()
    os.utime(evidence, (now, now))
    monkeypatch.setenv("LINGTU_SERVICE_READINESS_JSON", str(evidence))
    monkeypatch.setenv("LINGTU_REQUIRE_FIELD_SERVICE_READINESS", "1")
    monkeypatch.setenv("LINGTU_SERVICE_READINESS_MAX_AGE_S", "300")

    gateway = GatewayModule()
    gateway.setup()

    payload = asyncio.run(_endpoint(gateway, "/api/v1/services/status")("lidar"))
    model = ServiceStatusResponse.model_validate(payload)

    assert model.field_readiness["available"] is True
    assert model.field_readiness["fresh"] is True
    assert model.field_readiness["summary"]["blocker_count"] == 1
    assert model.readiness["ok"] is False
    assert model.readiness["blockers"] == [
        "field_readiness:native_binaries:native_binary_missing_or_not_executable:camera_dds:/opt/lingtu/current/build/camera_dds/lingtu_camera_dds"
    ]


def test_service_status_marks_stale_field_readiness_evidence(monkeypatch, tmp_path):
    from gateway.gateway_module import GatewayModule

    evidence = tmp_path / "stale_service_readiness.json"
    evidence.write_text(
        json.dumps(
            {
                "schema": "lingtu.thunder.service_readiness.v1",
                "stamp_s": 1.0,
                "summary": {"ok": True, "blockers": [], "blocker_count": 0},
            }
        ),
        encoding="utf-8",
    )
    old = time.time() - 1000.0
    os.utime(evidence, (old, old))
    monkeypatch.setenv("LINGTU_SERVICE_READINESS_JSON", str(evidence))
    monkeypatch.setenv("LINGTU_REQUIRE_FIELD_SERVICE_READINESS", "1")
    monkeypatch.setenv("LINGTU_SERVICE_READINESS_MAX_AGE_S", "1")

    gateway = GatewayModule()
    gateway.setup()

    payload = asyncio.run(_endpoint(gateway, "/api/v1/services/status")("lidar"))

    assert payload["field_readiness"]["fresh"] is False
    assert payload["readiness"]["ok"] is False
    assert payload["readiness"]["blockers"] == [
        "lidar:status_unknown",
        f"field_readiness:field_readiness_stale:{evidence}",
    ]


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
    gateway._all_modules = {"SlamAdapterModule": _Slam()}
    gateway._cloud_viewer.replace_map_points(
        np.asarray([(0.0, 0.0, 0.0)] * 42, dtype=np.float32)
    )

    def _fail_shell_hz():
        raise AssertionError("health should prefer module port rate")

    monkeypatch.setattr(gateway, "_get_slam_hz_cached", _fail_shell_hz)

    health = asyncio.run(_endpoint(gateway, "/api/v1/health")())

    assert health["map_points"] == 42
    assert health["slam_hz"] == 4.2
    assert health["sensors"]["slam"]["hz"] == 4.2


def test_health_prefers_cpp_processed_scan_rate_over_gateway_odom_rate(monkeypatch):
    from gateway.gateway_module import GatewayModule

    gateway = GatewayModule()
    gateway.setup()
    gateway._localization_status = {
        "state": "TRACKING",
        "processed_scan_hz": 9.96,
    }
    gateway._all_modules = {}
    monkeypatch.setattr(gateway, "_get_slam_hz_cached", lambda: 4.9)

    health = asyncio.run(_endpoint(gateway, "/api/v1/health")())

    assert health["slam_hz"] == 10.0
    assert health["sensors"]["slam"]["source"] == "processed_scan_hz"
    assert health["sensors"]["slam"]["odom_hz"] == 4.9


def test_health_reports_gateway_cloud_debug():
    from gateway.gateway_module import GatewayModule

    gateway = GatewayModule()
    gateway.setup()
    gateway._cloud_viewer.publish_cloud_frame(
        b"PCLD",
        metadata={
            "point_count": 7,
            "source": "slam_map_cloud",
            "session_mode": "mapping",
            "z_min": -6.9,
            "z_max": -2.8,
        },
    )
    gateway._all_modules = {}

    health = asyncio.run(_endpoint(gateway, "/api/v1/health")())
    cloud = health["gateway"]["cloud"]

    assert cloud["has_latest_binary_frame"] is True
    assert cloud["latest_frame"]["point_count"] == 7
    assert cloud["latest_frame"]["source"] == "slam_map_cloud"
    assert cloud["latest_frame"]["z_min"] == pytest.approx(-6.9)
    assert cloud["latest_frame"]["z_max"] == pytest.approx(-2.8)


def test_liveness_reports_lightweight_health_summary():
    from gateway.gateway_module import GatewayModule

    gateway = GatewayModule()
    gateway.setup()
    gateway._cloud_viewer.publish_cloud_frame(
        b"PCLD",
        metadata={"point_count": 5, "source": "slam_map_cloud"},
    )

    health = asyncio.run(_endpoint(gateway, "/health")())

    assert health["status"] == "ok"
    assert health["details_url"] == "/api/v1/health?details=true"
    assert health["gateway"]["cloud"]["latest_frame"]["point_count"] == 5
    assert "slam" in health["sensors"]


def test_health_reports_stale_cpp_slam_status(monkeypatch):
    from gateway.gateway_module import GatewayModule

    gateway = GatewayModule()
    gateway.setup()
    gateway._localization_status = {
        "state": "STALE",
        "processed_scan_hz": 0.0,
        "reason": "slam_runtime_status_stale",
        "status_snapshot_stale": True,
        "status_snapshot_age_s": 1.8,
    }
    gateway._all_modules = {}
    monkeypatch.setattr(gateway, "_get_slam_hz_cached", lambda: 0.0)

    health = asyncio.run(_endpoint(gateway, "/api/v1/health")())

    assert health["slam_hz"] == 0.0
    assert health["sensors"]["slam"]["status"] == "stale"
    assert health["sensors"]["slam"]["source"] == "localization_status"
    assert health["sensors"]["slam"]["reason"] == "slam_runtime_status_stale"
    assert health["sensors"]["slam"]["status_snapshot_stale"] is True


def test_health_falls_back_to_cached_slam_rate_when_module_rate_missing(monkeypatch):
    from gateway.gateway_module import GatewayModule

    gateway = GatewayModule()
    gateway.setup()
    gateway._all_modules = {}
    monkeypatch.setattr(gateway, "_get_slam_hz_cached", lambda: 3.7)

    health = asyncio.run(_endpoint(gateway, "/api/v1/health")())

    assert health["slam_hz"] == 3.7


def test_gateway_slam_rate_uses_existing_odometry_window(monkeypatch):
    import gateway.services.slam_profile as slam_profile
    from gateway.gateway_module import GatewayModule

    gateway = GatewayModule()
    gateway._odom_timestamps = [100.0 + i * 0.1 for i in range(20)]

    monkeypatch.setattr(slam_profile.time, "time", lambda: 101.95)
    assert gateway._get_slam_hz_cached() == 10.0

    monkeypatch.setattr(slam_profile.time, "time", lambda: 104.0)
    assert gateway._get_slam_hz_cached() == 0.0


def test_health_reports_camera_idle_reason_from_camera_health(monkeypatch):
    from gateway.gateway_module import GatewayModule

    class _Camera:
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
    gateway._all_modules = {"camera": _Camera()}
    monkeypatch.setattr(gateway, "_get_slam_hz_cached", lambda: 0.0)

    health = asyncio.run(_endpoint(gateway, "/api/v1/health")())
    camera = health["sensors"]["camera"]

    assert camera["status"] == "idle"
    assert camera["available"] is False
    assert camera["reason"] == "no_color_frames"
    assert camera["frames"] == 0
    assert camera["jpeg"]["cached"] is False


def test_health_detail_selection_uses_only_canonical_sensor_roles():
    from gateway.routes.status import _health_module_needs_detail

    assert _health_module_needs_detail("camera") is True
    assert _health_module_needs_detail("lidar") is True
    assert _health_module_needs_detail("CameraBridgeModule") is False
    assert _health_module_needs_detail("CameraModule") is False
    assert _health_module_needs_detail("LidarModule") is False


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


def test_brainstem_probe_projects_native_driver_status(monkeypatch, tmp_path):
    import gateway.routes.status as status_routes

    status_path = tmp_path / "driver_status.json"
    status_path.write_text(
        json.dumps(
            {
                "schema_version": "lingtu.driver.status.v2",
                "ready": True,
                "connected": True,
                "stamp_s": time.time(),
                "adapter": {
                    "target": "192.168.66.20:13145",
                    "protocol": "brainstem_grpc",
                    "control_owner": "grpc",
                    "control_owner_id": "lingtu-driver@robot",
                },
                "control": {
                    "fsm": "STANDING",
                    "motors_enabled": True,
                    "lease_valid": True,
                    "initial_zero_acknowledged": True,
                    "decision": "ready",
                },
                "output_ack": {
                    "producer_boot_id": "navd:boot",
                    "output_sequence": 17,
                    "accepted": True,
                },
                "last_reason": "accepted",
                "last_error": "",
            }
        ),
        encoding="utf-8",
    )
    monkeypatch.setenv("LINGTU_DRIVER_STATUS_FILE", str(status_path))

    info = status_routes._probe_brainstem()

    assert info["source"] == "lingtu-driver-status"
    assert info["status"] == "connected"
    assert info["ready"] is True
    assert info["host"] == "192.168.66.20:13145"
    assert info["fsm"] == "STANDING"
    assert info["lease_valid"] is True
    assert info["output_ack"]["output_sequence"] == 17


def test_brainstem_probe_rejects_stale_driver_status(monkeypatch, tmp_path):
    import gateway.routes.status as status_routes

    status_path = tmp_path / "driver_status.json"
    status_path.write_text(
        json.dumps(
            {
                "ready": True,
                "connected": True,
                "stamp_s": time.time() - 5.0,
                "adapter": {"target": "192.168.66.20:13145"},
                "control": {"fsm": "STANDING"},
            }
        ),
        encoding="utf-8",
    )
    monkeypatch.setenv("LINGTU_DRIVER_STATUS_FILE", str(status_path))
    monkeypatch.setenv("LINGTU_DRIVER_STATUS_MAX_AGE_S", "0.5")

    info = status_routes._probe_brainstem()

    assert info["status"] == "stale"
    assert info["stale"] is True
    assert info["ready"] is True


def test_health_uses_cached_brainstem_probe_for_short_app_polling_window(monkeypatch):
    import gateway.routes.status as status_routes
    from gateway.gateway_module import GatewayModule

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
    import gateway.routes.status as status_routes
    from gateway.gateway_module import GatewayModule

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
    import gateway.routes.status as status_routes
    from gateway.gateway_module import GatewayModule

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
    assert health["brainstem"]["reason"] == "driver_status_probe_pending"
    assert health["brainstem"]["source"] == "lingtu-driver-status"
    assert "host" not in health["brainstem"]
    assert health["brainstem"]["refreshing"] is True
    if gateway._brainstem_health_refresh_thread is not None:
        gateway._brainstem_health_refresh_thread.join(timeout=1.0)


def test_health_details_query_waits_for_live_brainstem_probe(monkeypatch):
    import gateway.routes.status as status_routes
    from gateway.gateway_module import GatewayModule

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


def test_gateway_health_declares_inspection_task_projection_limits():
    """Local hosts declare that task durability is disabled unless configured."""

    from gateway.gateway_module import GatewayModule

    gateway = GatewayModule()
    gateway.setup()

    projection = gateway.health()["gateway"]["inspection_task_projection"]

    assert projection["retention"] == "process_local_gateway_projection"
    assert projection["tracked_tasks"] == 0
    assert projection["boot_cursors"] == {}
    assert projection["journal"] == {"status": "disabled", "path": "", "error": ""}
