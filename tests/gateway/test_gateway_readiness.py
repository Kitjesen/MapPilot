from __future__ import annotations

import asyncio
import json
import math
import sys
import types
from types import SimpleNamespace

import pytest

pytestmark = [pytest.mark.sim]


pytest.importorskip("fastapi")


class _HealthyModule:
    def health(self):
        return {"state": "ok"}


class _BrokenModule:
    def health(self):
        raise RuntimeError("boom")


class _NonFiniteModule:
    def health(self):
        return {
            "state": "diagnostic",
            "nan": math.nan,
            "pos_inf": math.inf,
            "nested": {"neg_inf": -math.inf},
            "values": [1.0, math.nan],
        }


class _SlowHealthModule:
    def health(self):
        raise AssertionError("summary readiness must not call module health")


class _ThreadState:
    def __init__(self, alive: bool):
        self._alive = alive

    def is_alive(self) -> bool:
        return self._alive


def _real_run_plan(product: str):
    from lingtu.assembly.compiler import compile_run_plan
    from lingtu.assembly.products import resolve_product_host_runtime

    resolved = resolve_product_host_runtime(product, "real", robot="unitree/go2")
    return compile_run_plan(
        resolved.product,
        resolved.env,
        robot="unitree/go2",
    )


def test_readiness_snapshot_reports_not_started_without_modules():
    from gateway.gateway_module import GatewayModule
    from gateway.services.readiness import build_readiness_snapshot

    gateway = GatewayModule()

    payload, status_code = build_readiness_snapshot(gateway, now=123.0)

    assert status_code == 503
    assert payload["schema_version"] == 1
    assert payload["status"] == "not_started"
    assert payload["ready"] is False
    assert payload["data_ready"] is False
    assert payload["motion_ready"] is False
    assert payload["non_motion_safe"] is True
    assert payload["modules"] == {}
    assert payload["module_count"] == 0
    assert payload["reasons"] == ["no_modules_loaded"]
    assert payload["startup_state"] is None
    assert payload["critical_modules"] == []
    assert payload["critical_failed_modules"] == []
    assert payload["ts"] == 123.0


def test_readiness_snapshot_reports_ready_when_all_modules_are_healthy():
    from gateway.gateway_module import GatewayModule
    from gateway.services.readiness import build_readiness_snapshot

    gateway = GatewayModule()
    gateway._all_modules = {"A": _HealthyModule(), "B": SimpleNamespace()}

    payload, status_code = build_readiness_snapshot(gateway, now=124.0)

    assert status_code == 200
    assert payload["status"] == "ready"
    assert payload["ready"] is True
    assert payload["data_ready"] is True
    assert payload["motion_ready"] is True
    assert payload["non_motion_safe"] is True
    assert payload["module_count"] == 2
    assert payload["failed_modules"] == []
    assert payload["modules"]["A"]["detail"] == {"state": "ok"}
    assert payload["modules"]["B"]["detail"] == {}


def test_readiness_snapshot_requires_native_endpoint_for_idle_teleop_product(
    monkeypatch,
    tmp_path,
):
    from gateway.gateway_module import GatewayModule
    from gateway.services.readiness import build_readiness_snapshot

    monkeypatch.setenv("LINGTU_ENV", "real")
    monkeypatch.setenv("LINGTU_PRODUCT", "teleop")
    monkeypatch.setenv("LINGTU_PRODUCT_SESSION_ID", "1" * 32)
    monkeypatch.setenv(
        "LINGTU_NAV_STATUS_FILE",
        str(tmp_path / "missing-nav-endpoint-status.json"),
    )
    plan = _real_run_plan("teleop")
    gateway = GatewayModule(
        run_plan=plan,
    )
    gateway._all_modules = {"GatewayModule": _HealthyModule()}
    gateway._session_product = None

    payload, status_code = build_readiness_snapshot(gateway, now=124.25)

    assert status_code == 503
    assert payload["ready"] is False
    assert "navigation_blocked:native_endpoint_status_missing_or_stale" in payload["reasons"]
    assert "native_endpoint_status_missing_or_stale" in payload["runtime"]["navigation"]["blockers"]
    assert payload["product_contract"]["product"] == "teleop"
    assert payload["product_contract"]["product_session_id"] == "1" * 32
    assert payload["product_contract"]["command_output_mode"] == "endpoint_only"
    assert payload["product_contract"]["hardware_control_boundary"] == "driver"
    assert payload["product_contract"]["native_readiness_required"] is True
    assert "/nav/operator_motion/status" in payload["product_contract"]["required_topics"]
    assert "native_operator_motion_authority" in payload["product_contract"]["required_capabilities"]

    from gateway.schemas import ReadinessResponse

    serialized = ReadinessResponse.model_validate(payload).model_dump()
    assert serialized["product_contract"]["product"] == "teleop"
    assert serialized["product_contract"]["native_readiness_required"] is True


def test_managed_gateway_without_run_plan_fails_readiness_closed():
    from gateway.gateway_module import GatewayModule
    from gateway.services.readiness import build_readiness_snapshot

    gateway = GatewayModule(
        command_output_mode="endpoint_only",
        hardware_control_boundary="driver",
        product="teleop",
    )
    gateway._all_modules = {"GatewayModule": _HealthyModule()}

    payload, status_code = build_readiness_snapshot(gateway, now=124.4)

    assert status_code == 503
    assert payload["ready"] is False
    assert "run_plan_missing" in payload["reasons"]


@pytest.mark.parametrize("local_profile", ["stub", "dev"])
def test_readiness_snapshot_keeps_local_profiles_module_only(monkeypatch, local_profile):
    from gateway.gateway_module import GatewayModule
    from gateway.services.readiness import build_readiness_snapshot

    monkeypatch.setenv("LINGTU_PROFILE", local_profile)
    monkeypatch.setenv("LINGTU_ENV", "sim")
    monkeypatch.delenv("LINGTU_PRODUCT", raising=False)
    gateway = GatewayModule()
    gateway._all_modules = {"A": _HealthyModule()}

    payload, status_code = build_readiness_snapshot(gateway, now=124.5)

    assert status_code == 200
    assert payload["ready"] is True
    assert payload["runtime"] == {}
    assert payload["product_contract"]["product"] is None
    assert payload["product_contract"]["native_readiness_required"] is False


def test_maps_blocker_fails_data_readiness_for_mapd_product(monkeypatch):
    from gateway.gateway_module import GatewayModule
    from gateway.services import readiness as readiness_service
    from gateway.services.readiness import build_readiness_snapshot

    class HostBus:
        def map_readiness(self):
            return "waiting_for_mapd_scene"

        def health(self):
            return {
                "map_scene": {
                    "required": True,
                    "received": False,
                    "generation": 0,
                }
            }

    monkeypatch.setattr(
        readiness_service,
        "_requires_runtime_readiness",
        lambda _gw, _modules: False,
    )
    gateway = GatewayModule(run_plan=_real_run_plan("nav"))
    gateway._all_modules = {"host.bus": HostBus()}

    payload, status_code = build_readiness_snapshot(gateway, now=124.75)

    assert status_code == 503
    assert payload["ready"] is False
    assert payload["data_ready"] is False
    assert payload["motion_ready"] is False
    assert payload["reasons"] == ["maps:waiting_for_mapd_scene"]
    assert payload["runtime"]["maps"]["ready"] is False
    assert payload["runtime"]["summary"]["data_blockers"] == ["maps:waiting_for_mapd_scene"]


def test_readiness_snapshot_can_omit_module_details_for_probe_payload():
    from gateway.gateway_module import GatewayModule
    from gateway.services.readiness import build_readiness_snapshot

    gateway = GatewayModule()
    gateway._all_modules = {"A": _SlowHealthModule()}

    payload, status_code = build_readiness_snapshot(
        gateway,
        now=124.5,
        include_details=False,
    )

    assert status_code == 200
    assert payload["ready"] is True
    assert payload["modules"]["A"] == {"ok": True}


def test_ready_route_defaults_to_summary_and_supports_details_query():
    from gateway.gateway_module import GatewayModule

    gateway = GatewayModule()
    gateway.setup()
    gateway._all_modules = {"A": _HealthyModule()}
    endpoint = next(route.endpoint for route in gateway._app.routes if route.path == "/ready")

    summary_response = asyncio.run(endpoint())
    summary_payload = json.loads(summary_response.body)
    detail_response = asyncio.run(endpoint(details=True))
    detail_payload = json.loads(detail_response.body)

    assert summary_payload["modules"]["A"] == {"ok": True}
    assert detail_payload["modules"]["A"]["detail"] == {"state": "ok"}


def test_readiness_snapshot_reports_failed_modules_and_keeps_legacy_fields():
    from gateway.gateway_module import GatewayModule
    from gateway.services.readiness import build_readiness_snapshot

    gateway = GatewayModule()
    gateway._all_modules = {"A": _HealthyModule(), "Bad": _BrokenModule()}

    payload, status_code = build_readiness_snapshot(gateway, now=125.0)

    assert status_code == 503
    assert payload["status"] == "degraded"
    assert payload["ready"] is False
    assert payload["data_ready"] is False
    assert payload["motion_ready"] is False
    assert payload["non_motion_safe"] is True
    assert payload["failed_modules"] == ["Bad"]
    assert payload["reasons"] == ["module_failed:Bad"]
    assert payload["modules"]["Bad"]["ok"] is False
    assert payload["modules"]["Bad"]["error"] == "boom"


def test_readiness_snapshot_sanitizes_non_finite_health_values():
    from gateway.gateway_module import GatewayModule
    from gateway.services.readiness import build_readiness_snapshot

    gateway = GatewayModule()
    gateway._all_modules = {"A": _NonFiniteModule()}

    payload, status_code = build_readiness_snapshot(gateway, now=126.0)

    assert status_code == 200
    detail = payload["modules"]["A"]["detail"]
    assert detail["nan"] is None
    assert detail["pos_inf"] is None
    assert detail["nested"]["neg_inf"] is None
    assert detail["values"] == [1.0, None]
    json.dumps(payload, allow_nan=False)


def test_readiness_snapshot_blocks_lost_robot_localization():
    from gateway.gateway_module import GatewayModule
    from gateway.services.readiness import build_readiness_snapshot

    gateway = GatewayModule()
    gateway._all_modules = {"SlamAdapterModule": _HealthyModule()}
    with gateway._state_lock:
        gateway._odom = {"x": 0.0}
        gateway._localization_status = {
            "state": "LOST",
            "confidence": 0.0,
        }

    payload, status_code = build_readiness_snapshot(gateway, now=127.0)

    assert status_code == 503
    assert payload["status"] == "degraded"
    assert payload["ready"] is False
    assert payload["data_ready"] is False
    assert payload["motion_ready"] is False
    assert payload["non_motion_safe"] is True
    assert payload["failed_modules"] == []
    assert "localization:lost" in payload["reasons"]
    assert payload["runtime"]["localization"]["state"] == "lost"


def test_readiness_snapshot_includes_navigation_blockers():
    from gateway.gateway_module import GatewayModule
    from gateway.services.readiness import build_readiness_snapshot

    gateway = GatewayModule()
    gateway._all_modules = {"host.bus": _HealthyModule()}
    gateway._session_mode = "navigating"
    gateway._icp_quality = 0.03
    with gateway._state_lock:
        gateway._odom = {"x": 0.0}
        gateway._localization_status = {
            "state": "TRACKING",
            "confidence": 0.9,
            "degeneracy": "NONE",
            "icp_fitness": 0.028,
            "odom_age_ms": 2500.0,
            "localizer_health": "RECOVERED",
        }

    payload, status_code = build_readiness_snapshot(gateway, now=128.0)

    assert status_code == 503
    assert "localization:degraded" in payload["reasons"]
    assert "navigation_blocked:pose_stale" in payload["reasons"]
    assert payload["data_ready"] is False
    assert payload["motion_ready"] is False
    assert payload["non_motion_safe"] is True
    assert payload["runtime"]["navigation"]["blockers"] == ["pose_stale"]
    assert payload["runtime"]["summary"]["data_blockers"] == [
        "localization:degraded",
        "localization:pose_stale",
        "navigation_blocked:pose_stale",
    ]


def test_readiness_snapshot_includes_runtime_boundary_blockers(monkeypatch):
    from gateway.gateway_module import GatewayModule
    from gateway.services.readiness import build_readiness_snapshot

    monkeypatch.setenv("LINGTU_ENV", "sim")
    monkeypatch.setenv("LINGTU_PRODUCT", "explore")
    monkeypatch.setenv("LINGTU_PRODUCT_SESSION_ID", "2" * 32)
    monkeypatch.setenv("LINGTU_DATA_SOURCE", "mujoco_fastlio2_live")
    monkeypatch.setenv("LINGTU_RUNTIME_CONTRACT", "real")
    monkeypatch.setenv("LINGTU_COMMAND_SINK", "driver")

    run_plan = SimpleNamespace(
        env="sim",
        product="explore",
        host_config={
            "command_output_mode": "",
            "hardware_control_boundary": "",
        },
        processes=(),
        required_topics=("/maps/state", "/maps/scene"),
        required_capabilities=(),
    )
    gateway = GatewayModule(run_plan=run_plan)
    gateway._all_modules = {"host.bus": _HealthyModule()}
    gateway._session_mode = "navigating"
    with gateway._state_lock:
        gateway._odom = {"x": 0.0, "frame_id": "map"}
        gateway._localization_status = {
            "state": "TRACKING",
            "confidence": 0.9,
            "degeneracy": "NONE",
            "icp_fitness": 0.028,
            "odom_age_ms": 100.0,
            "localizer_health": "RECOVERED",
        }

    payload, status_code = build_readiness_snapshot(gateway, now=128.25)

    assert status_code == 503
    assert payload["data_ready"] is False
    assert payload["motion_ready"] is False
    assert "runtime_blocked:runtime_contract_data_source_mismatch" in payload["reasons"]
    assert "runtime_blocked:command_sink_mismatch" in payload["reasons"]
    boundary = payload["runtime"]["boundary"]
    assert boundary["ok"] is False
    assert boundary["env"] == "sim"
    assert boundary["product"] == "explore"
    assert boundary["state"] == "active"
    assert boundary["product_session_id"] == "2" * 32
    assert "run_plan_path" not in boundary
    assert "identity_source" not in boundary
    assert "endpoint" not in boundary
    assert "profile" not in boundary
    assert boundary["expected_command_sink"] == ("mujoco_velocity_adapter")
    assert boundary["frames"]["axis_convention"] == ("x_forward_y_left_z_up")
    assert boundary["frame_links"]["body_to_lidar"] == {
        "parent": "body",
        "child": "lidar_link",
        "required": True,
    }
    assert boundary["topic_allowed_frame_ids"]["/slam/map_cloud"] == ["map"]
    assert boundary["topic_default_frame_ids"]["/slam/map_cloud"] == "map"
    assert boundary["required_topic_frame_ids"] == [
        "/lidar/raw_frame",
        "/imu/raw",
        "/slam/odometry",
        "/slam/registered_cloud",
        "/slam/map_cloud",
        "/nav/global_path",
        "/nav/local_path",
        "/nav/cmd_vel",
    ]
    assert boundary["runtime_data_flow_topics"][:2] == [
        "/lidar/raw_frame",
        "/imu/raw",
    ]
    flow = {stage["name"]: stage for stage in boundary["resolved_runtime_data_flow"]}
    assert flow["endpoint_adapter"]["inputs"] == ["/lidar/raw_frame", "/imu/raw"]
    assert flow["command_boundary"]["outputs"] == ["mujoco_velocity_adapter"]
    assert boundary["runtime_data_flow_stage_algorithm_interfaces"]["global_planning"] == [
        "global_planning",
        "octoplanner3d_global_planning",
    ]
    assert payload["runtime"]["summary"]["data_blockers"] == [
        "runtime_blocked:runtime_contract_data_source_mismatch",
        "runtime_blocked:command_sink_mismatch",
            "maps:map_readiness_contract_missing",
    ]
    assert payload["runtime"]["maps"] == {
        "required": True,
        "ready": False,
        "reason": "map_readiness_contract_missing",
        "host_bus": None,
    }


def test_readiness_snapshot_includes_localization_frame_contract(monkeypatch):
    from gateway.gateway_module import GatewayModule
    from gateway.services.readiness import build_readiness_snapshot

    monkeypatch.setenv("LINGTU_ENV", "real")
    monkeypatch.setenv("LINGTU_PRODUCT", "nav")
    monkeypatch.setenv("LINGTU_DATA_SOURCE", "field")
    monkeypatch.setenv("LINGTU_RUNTIME_CONTRACT", "real")
    monkeypatch.setenv("LINGTU_COMMAND_SINK", "driver")

    gateway = GatewayModule()
    gateway._all_modules = {"host.bus": _HealthyModule()}
    gateway._session_mode = "navigating"
    gateway._icp_quality = 0.03
    with gateway._state_lock:
        gateway._odom = {"x": 0.0, "frame_id": "odom"}
        gateway._mission = {
            "state": "IDLE",
            "planning_frame_id": "map",
            "costmap_frame_id": "map",
            "goal_frame_id": "map",
        }
        gateway._localization_status = {
            "state": "TRACKING",
            "confidence": 0.9,
            "degeneracy": "NONE",
            "icp_fitness": 0.028,
            "odom_age_ms": 100.0,
            "cloud_age_ms": 80.0,
            "registered_cloud_frame_id": "body",
            "map_cloud_frame_id": "map",
            "map_cloud_fresh": True,
            "localizer_health": "RECOVERED",
        }

    payload, status_code = build_readiness_snapshot(gateway, now=128.35)

    assert status_code == 503
    assert "navigation_blocked:frame_mismatch_odometry" in payload["reasons"]
    assert "navigation_blocked:real_runtime_evidence_missing_or_stale" in payload["reasons"]
    localization = payload["runtime"]["localization"]
    assert localization["runtime_contract"] == "real"
    assert localization["topic_default_frame_ids"]["/slam/map_cloud"] == "map"
    assert localization["required_topic_frame_ids"] == [
        "/lidar/raw_frame",
        "/imu/raw",
        "/slam/odometry",
        "/slam/registered_cloud",
        "/slam/map_cloud",
        "/nav/global_path",
        "/nav/local_path",
        "/nav/cmd_vel",
    ]
    assert "/slam/odometry" in localization["runtime_data_flow_topics"]
    assert localization["runtime_data_flow_stage_algorithm_interfaces"]["local_planning_and_following"] == [
        "local_planning_and_following",
    ]
    frames = localization["frames"]
    assert frames["runtime_contract"] == "real"
    assert frames["odometry_frame_id"] == "odom"
    assert frames["registered_cloud_frame_id"] == "body"
    assert frames["map_cloud_frame_id"] == "map"
    assert frames["observed_topic_frame_ids"] == {
        "/slam/odometry": "odom",
        "/slam/registered_cloud": "body",
        "/slam/map_cloud": "map",
    }
    assert frames["missing_required_topic_frame_ids"] == []
    assert frames["mismatches"] == []
    assert frames["ok"] is True
    navigation = payload["runtime"]["navigation"]
    assert navigation["tf_ok"] is False
    assert navigation["planning_frame_id"] == "map"
    assert navigation["odom_frame_id"] == "odom"
    assert navigation["real_runtime_evidence_ok"] is False
    boundary = payload["runtime"]["boundary"]
    assert boundary["env"] == "real"
    assert boundary["product"] == "nav"
    assert "runtime_contract_data_source_mismatch" not in boundary["blockers"]
    assert "endpoint" not in boundary
    assert "profile" not in boundary


def test_manual_hold_is_a_non_motion_safe_readiness_state():
    from gateway.services.readiness import _runtime_readiness_modes

    modes = _runtime_readiness_modes(
        failed_modules=[],
        reasons=["navigation_blocked:native_resume_required"],
        runtime={
            "navigation": {
                "state": "IDLE",
                "active_cmd_source": "manual_hold",
            }
        },
    )

    assert modes["data_ready"] is True
    assert modes["motion_ready"] is False
    assert modes["non_motion_safe"] is True


def test_readiness_snapshot_blocks_motion_but_not_data_when_safety_stop_active():
    from gateway.gateway_module import GatewayModule
    from gateway.services.readiness import build_readiness_snapshot

    gateway = GatewayModule()
    gateway._all_modules = {"host.bus": _HealthyModule()}
    gateway._session_mode = "navigating"
    gateway._icp_quality = 0.03
    with gateway._state_lock:
        gateway._odom = {"x": 0.0, "y": 0.0, "z": 0.0}
        gateway._mission = {"state": "IDLE"}
        gateway._navigation_state = {"authority": "estop", "hold_reason": "operator_estop"}
        gateway._localization_status = {
            "state": "TRACKING",
            "confidence": 0.9,
            "degeneracy": "NONE",
            "icp_fitness": 0.028,
            "odom_age_ms": 100.0,
            "localizer_health": "RECOVERED",
        }

    payload, status_code = build_readiness_snapshot(gateway, now=128.5)

    assert status_code == 503
    assert payload["ready"] is False
    assert payload["data_ready"] is True
    assert payload["motion_ready"] is False
    assert payload["non_motion_safe"] is True
    assert "navigation_blocked:safety_stop" in payload["reasons"]
    assert "safety:stop" in payload["reasons"]
    assert payload["runtime"]["safety"]["stop_active"] is True
    assert payload["runtime"]["summary"]["data_blockers"] == []




def test_readiness_snapshot_surfaces_calibration_warnings(monkeypatch):
    from gateway.gateway_module import GatewayModule
    from gateway.services import readiness as readiness_service
    from gateway.services.readiness import build_readiness_snapshot

    class Report:
        ok = True
        errors: list[str] = []
        warnings = ["Camera rotation is identity"]
        info = ["camera intrinsics ok"]

        def summary(self):
            return "Calibration: 1 warning(s)"

    monkeypatch.setattr(
        "runtime.utils.calibration_check.run_calibration_check",
        lambda **_kwargs: Report(),
    )
    readiness_service._CALIBRATION_CACHE = None

    gateway = GatewayModule()
    gateway._all_modules = {"host.bus": _HealthyModule()}
    gateway._session_mode = "navigating"
    with gateway._state_lock:
        gateway._odom = {"x": 0.0}
        gateway._mission = {"state": "IDLE"}
        gateway._localization_status = {
            "state": "TRACKING",
            "confidence": 0.9,
            "degeneracy": "NONE",
            "icp_fitness": 0.03,
            "localizer_health": "RECOVERED",
            "odom_age_ms": 100.0,
        }
        gateway._icp_quality = 0.03

    payload, status_code = build_readiness_snapshot(gateway, now=130.0)

    assert status_code == 200
    assert payload["ready"] is True
    assert payload["advisories"] == ["Camera rotation is identity"]
    assert payload["runtime"]["calibration"] == {
        "ok": True,
        "errors": [],
        "warnings": ["Camera rotation is identity"],
        "info_count": 1,
        "summary": "Calibration: 1 warning(s)",
    }
    assert "calibration:error" not in payload["reasons"]


@pytest.mark.parametrize(
    "startup_state",
    ["built", "starting", "failed", "stopping", "stopped"],
)
def test_readiness_snapshot_fails_data_closed_until_system_is_ready(startup_state):
    from gateway.gateway_module import GatewayModule
    from gateway.services.readiness import build_readiness_snapshot

    gateway = GatewayModule()
    gateway.set_system_handle(
        SimpleNamespace(
            startup_state=startup_state,
            critical_modules=("A",),
            failed_modules={},
            critical_failures={},
        )
    )
    gateway._all_modules = {"A": _HealthyModule()}

    payload, status_code = build_readiness_snapshot(gateway, now=131.0)

    assert status_code == 503
    assert payload["startup_state"] == startup_state
    assert payload["critical_modules"] == ["A"]
    assert payload["critical_failed_modules"] == []
    assert payload["data_ready"] is False
    assert f"startup_state:{startup_state}" in payload["reasons"]


def test_readiness_snapshot_reports_optional_startup_failure_as_advisory_only():
    from gateway.gateway_module import GatewayModule
    from gateway.services.readiness import build_readiness_snapshot

    gateway = GatewayModule()
    gateway.set_system_handle(
        SimpleNamespace(
            startup_state="ready",
            critical_modules=("A",),
            failed_modules={"OptionalModule": "setup: boom"},
            critical_failures={},
        )
    )
    gateway._all_modules = {
        "A": _HealthyModule(),
        "OptionalModule": _HealthyModule(),
    }

    payload, status_code = build_readiness_snapshot(gateway, now=132.0)

    assert status_code == 200
    assert payload["ready"] is True
    assert payload["data_ready"] is True
    assert payload["failed_modules"] == []
    assert payload["critical_failed_modules"] == []
    assert "optional_module_failed:OptionalModule:setup: boom" in payload["advisories"]


def test_readiness_snapshot_reports_critical_startup_failure():
    from gateway.gateway_module import GatewayModule
    from gateway.services.readiness import build_readiness_snapshot

    gateway = GatewayModule()
    gateway.set_system_handle(
        SimpleNamespace(
            startup_state="failed",
            critical_modules=("A",),
            failed_modules={"A": "start: boom"},
            critical_failures={"A": "start: boom"},
        )
    )
    gateway._all_modules = {"A": _HealthyModule()}

    payload, status_code = build_readiness_snapshot(gateway, now=133.0)

    assert status_code == 503
    assert payload["failed_modules"] == ["A"]
    assert payload["critical_failed_modules"] == ["A"]
    assert "critical_module_failed:A" in payload["reasons"]
    assert payload["data_ready"] is False


def test_readiness_snapshot_prefers_runtime_status_provider():
    from gateway.gateway_module import GatewayModule
    from gateway.services.readiness import build_readiness_snapshot

    gateway = GatewayModule()
    gateway.set_system_handle(
        SimpleNamespace(
            startup_state="ready",
            critical_modules=("Legacy",),
            failed_modules={},
            critical_failures={},
        )
    )
    gateway.set_runtime_status_provider(
        SimpleNamespace(
            startup_state="starting",
            critical_modules=("host.bus", "GatewayModule"),
            failed_modules={},
            critical_failures={},
            modules={"host.bus": _HealthyModule(), "GatewayModule": _HealthyModule()},
        )
    )

    payload, status_code = build_readiness_snapshot(gateway, now=133.5)

    assert status_code == 503
    assert payload["startup_state"] == "starting"
    assert payload["critical_modules"] == ["host.bus", "GatewayModule"]
    assert payload["module_count"] == 2
    assert "host.bus" in payload["modules"]
    assert "startup_state:starting" in payload["reasons"]
    assert "Legacy" not in payload["critical_modules"]


def test_gateway_startup_readiness_requires_live_started_uvicorn():
    from gateway.gateway_module import GatewayModule

    gateway = GatewayModule()
    assert gateway.startup_readiness() == "not_running"

    gateway._running = True
    gateway._defer_server = True
    assert gateway.startup_readiness() is None

    gateway._defer_server = False
    gateway._server_thread = _ThreadState(False)
    assert gateway.startup_readiness() == "server_thread_not_running"

    gateway._server_thread = _ThreadState(True)
    assert gateway.startup_readiness() == "server_not_created"

    gateway._server = SimpleNamespace(started=False)
    assert gateway.startup_readiness() == "server_not_started"

    gateway._server.started = True
    assert gateway.startup_readiness() is None

    gateway._server_error = "SystemExit: bind failed"
    assert gateway.startup_readiness() == "server_error:SystemExit: bind failed"


def test_gateway_run_server_records_bind_failure(monkeypatch):
    from gateway.gateway_module import GatewayModule

    class FakeConfig:
        def __init__(self, *_args, **_kwargs):
            pass

    class ExitingServer:
        started = False
        should_exit = False
        force_exit = False

        def __init__(self, _config):
            pass

        def run(self):
            raise SystemExit("bind failed")

    fake_uvicorn = types.ModuleType("uvicorn")
    fake_uvicorn.Config = FakeConfig
    fake_uvicorn.Server = ExitingServer
    monkeypatch.setitem(sys.modules, "uvicorn", fake_uvicorn)
    gateway = GatewayModule()
    gateway._app = object()

    assert gateway._run_server() is False
    assert gateway._server_error == "SystemExit: bind failed"
    assert gateway._server is None
