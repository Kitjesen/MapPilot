from __future__ import annotations

import asyncio
import json
import time

import pytest

pytest.importorskip("fastapi")


def test_client_links_schema_covers_declared_links():
    from gateway.schemas import ClientLinks
    from gateway.services.app_bootstrap import CLIENT_LINKS

    fields = getattr(ClientLinks, "model_fields", None) or ClientLinks.__fields__
    missing = sorted(set(CLIENT_LINKS) - set(fields))
    assert missing == []


def test_client_links_advertise_public_map_operation_flow():
    from gateway.services.app_bootstrap import CLIENT_ENDPOINTS, CLIENT_LINKS

    expected = {
        "map_operations": ("GET", "/api/v1/maps/operations"),
        "map_operation_status": ("GET", "/api/v1/maps/operations/{operation_id}"),
        "map_operation_cancel": ("POST", "/api/v1/maps/operations/{operation_id}/cancel"),
        "map_operation_retry": ("POST", "/api/v1/maps/operations/{operation_id}/retry"),
    }
    for name, (method, path) in expected.items():
        assert CLIENT_LINKS[name] == path
        assert CLIENT_ENDPOINTS["map"][name] == {
            "method": method,
            "path": path,
        }


def test_app_bootstrap_exposes_read_only_product_session_identity():
    from gateway.gateway_module import GatewayModule
    from gateway.services.app_bootstrap import build_app_bootstrap

    gateway = GatewayModule(product_session_id="product-session-1234")

    payload = build_app_bootstrap(gateway)

    assert payload["session"]["product_session_id"] == "product-session-1234"
    assert payload["links"]["session"] == "/api/v1/session"
    assert "session_start" not in payload["links"]
    assert "session_end" not in payload["links"]


def test_app_bootstrap_uses_run_plan_teleop_limits(monkeypatch):
    from gateway.gateway_module import GatewayModule
    from gateway.services.app_bootstrap import build_app_bootstrap
    from lingtu.assembly.compiler import compile_run_plan

    monkeypatch.setenv("LINGTU_TELEOP_MAX_SPEED_MPS", "0.2")
    monkeypatch.setenv("LINGTU_TELEOP_MAX_YAW_RATE", "0.3")
    monkeypatch.setenv("LINGTU_ROBOT", "doso/thunder_v4")
    plan = compile_run_plan("teleop_avoid", "real", robot="unitree/go2")

    gateway = GatewayModule(run_plan=plan)
    payload = build_app_bootstrap(gateway)

    assert gateway._teleop_max_speed == 0.5
    assert gateway._teleop_max_yaw == 1.0
    assert payload["control"]["teleop"]["limits"] == {
        "linear_mps": 0.5,
        "yaw_rad_s": 1.0,
    }
    assert payload["robot"]["model"] == "unitree/go2"


@pytest.mark.parametrize("configured, expected", [(None, None), ("unitree/go2", "unitree/go2")])
def test_app_bootstrap_robot_model_uses_environment_without_inventing_a_default(monkeypatch, configured, expected):
    from gateway.gateway_module import GatewayModule
    from gateway.services.app_bootstrap import build_app_bootstrap

    if configured is None:
        monkeypatch.delenv("LINGTU_ROBOT", raising=False)
    else:
        monkeypatch.setenv("LINGTU_ROBOT", configured)
    assert build_app_bootstrap(GatewayModule())["robot"]["model"] == expected


def test_client_links_advertise_task_status_and_estop_reset_controls():
    from gateway.services.app_bootstrap import CLIENT_ENDPOINTS, CLIENT_LINKS

    assert CLIENT_LINKS["navigation_task_status"] == ("/api/v1/navigation/tasks/{task_id}")
    assert CLIENT_LINKS["navigation_task_cancel"] == ("/api/v1/navigation/tasks/{task_id}/cancel")
    assert CLIENT_LINKS["navigation_task_pause"] == ("/api/v1/navigation/tasks/{task_id}/pause")
    assert CLIENT_LINKS["navigation_task_resume"] == ("/api/v1/navigation/tasks/{task_id}/resume")
    controls = CLIENT_ENDPOINTS["control"]
    for operation in (
        "navigation_task_cancel",
        "navigation_task_pause",
        "navigation_task_resume",
    ):
        assert controls[operation] == {
            "method": "POST",
            "path": CLIENT_LINKS[operation],
        }
    assert CLIENT_LINKS["estop_reset"] == "/api/v1/estop/reset"


def test_client_links_advertise_task_addressed_inspection_controls_only():
    """The product console must not discover a taskless inspection control path."""

    from gateway.services.app_bootstrap import CLIENT_ENDPOINTS, CLIENT_LINKS

    assert CLIENT_LINKS["inspection_tasks"] == "/api/v1/inspection/tasks"
    assert CLIENT_LINKS["inspection_task_status"] == ("/api/v1/inspection/tasks/{task_id}")
    assert CLIENT_LINKS["inspection_task_report"] == ("/api/v1/inspection/tasks/{task_id}/report")
    assert CLIENT_LINKS["inspection_task_pause"] == ("/api/v1/inspection/tasks/{task_id}/pause")
    assert CLIENT_LINKS["inspection_task_resume"] == ("/api/v1/inspection/tasks/{task_id}/resume")
    assert CLIENT_LINKS["inspection_task_cancel"] == ("/api/v1/inspection/tasks/{task_id}/cancel")
    for operation, method in (
        ("inspection_task_list", "GET"),
        ("inspection_task_start", "POST"),
        ("inspection_task_status", "GET"),
        ("inspection_task_report", "GET"),
        ("inspection_task_pause", "POST"),
        ("inspection_task_resume", "POST"),
        ("inspection_task_cancel", "POST"),
    ):
        assert CLIENT_ENDPOINTS["ops"][operation] == {
            "method": method,
            "path": CLIENT_LINKS[
                "inspection_tasks" if operation in {"inspection_task_list", "inspection_task_start"} else operation
            ],
        }
    assert not {
        "inspection_route_start",
        "inspection_pause",
        "inspection_resume",
        "inspection_cancel",
    } & set(CLIENT_LINKS)


def test_app_bootstrap_service_returns_client_contract():
    from gateway.gateway_module import GatewayModule
    from gateway.services.app_bootstrap import (
        build_app_bootstrap,
        build_app_capabilities,
        build_app_traffic,
    )

    gateway = GatewayModule()
    with gateway._state_lock:
        gateway._odom = {"x": 1.0, "y": 2.0, "yaw": 0.1}
        gateway._navigation_state = {
            "lifecycle_state_name": "EXECUTING",
            "active_task_id": "navigation-task-1",
            "active_request_id": "goal-1",
            "authority": "none",
            "ts": time.time(),
        }
        gateway._navigation_goal_status_by_task["navigation-task-1"] = {
            "task_id": "navigation-task-1",
            "request_id": "goal-1",
            "state_name": "EXECUTING",
        }
        gateway._mode = "autonomous"
        gateway._last_path = [{"x": 0.0, "y": 0.0}, {"x": 1.0, "y": 1.0}]
        gateway._localization_status = {
            "state": "DEGRADED",
            "confidence": 0.4,
            "degeneracy": "MILD",
            "ts": 123.0,
        }
    gateway._session_mode = "navigating"
    gateway._icp_quality = 0.2
    payload = build_app_bootstrap(gateway)

    assert payload["schema_version"] == 4
    assert isinstance(payload["ts"], float)
    assert payload["server"]["time"] == payload["ts"]
    assert payload["server"]["api_version"] == "v1"
    assert payload["robot"]["has_odometry"] is True
    assert "mission" not in payload
    assert payload["safety"]["ok"] is True
    assert payload["localization"]["state"] == "degraded"
    assert payload["localization"]["reported_state"] == "DEGRADED"
    assert payload["localization"]["confidence"] == 0.4
    assert payload["navigation"]["task"]["state"] == "EXECUTING"
    assert set(payload["navigation"]) == {
        "schema_version",
        "task",
        "goal_admission",
        "control",
        "motion",
        "ts",
    }
    assert set(payload["control"]) == {"teleop", "command_policy"}
    assert payload["path"]["points"] == 2
    assert payload["path"]["endpoint"] == "/api/v1/path"
    assert "path" not in payload["path"]
    assert payload["scene"]["endpoint"] == "/api/v1/scene_graph"
    assert "scene_graph" not in payload["scene"]
    scene_layers = payload["scene"]["layers"]
    assert scene_layers["schema_version"] == 1
    assert scene_layers["coordinate_mapping"] == "lingtu_xyz_to_three_x_z_neg_y"
    assert {layer["id"] for layer in scene_layers["layers"]} >= {
        "live_cloud",
        "elevation",
        "native_traversability",
        "path",
        "robot",
    }
    live_layer = next(layer for layer in scene_layers["layers"] if layer["id"] == "live_cloud")
    assert live_layer["source"] == "map_scene"
    elevation_layer = next(layer for layer in scene_layers["layers"] if layer["id"] == "elevation")
    assert elevation_layer["role"] == "lowest_observed_z_not_ground"
    assert elevation_layer["subscription"] == "include_elevation=1"
    native_layer = next(layer for layer in scene_layers["layers"] if layer["id"] == "native_traversability")
    assert native_layer["role"] == "control_risk_read_only_projection"
    assert native_layer["value_semantics"] == "control_risk_0_100"
    assert payload["traffic"]["client_policy"]["usage"] == "cold_start_only"
    assert payload["traffic"]["client_policy"]["events_endpoint"] == "/api/v1/events"
    assert payload["traffic"]["client_policy"]["traffic_endpoint"] == "/api/v1/app/traffic"
    assert payload["capabilities"]["exploration"] is False
    assert payload["media"]["camera_ws"] == "/ws/camera"
    assert payload["media"]["webrtc_whep"] == "/api/v1/webrtc/whep"
    assert payload["media"]["go2rtc_status"] == "/api/v1/webrtc/go2rtc/status"
    assert payload["media"]["whep"]["supported"] is True
    assert payload["media"]["whep"]["endpoint"] == "/api/v1/webrtc/whep"
    assert payload["capabilities_endpoint"] == "/api/v1/app/capabilities"
    assert payload["links"]["state"] == "/api/v1/state"
    assert payload["links"]["traffic"] == "/api/v1/app/traffic"
    assert payload["links"]["readiness"] == "/api/v1/readiness"
    assert payload["links"]["metrics"] == "/api/v1/metrics"
    assert payload["links"]["localization_status"] == "/api/v1/localization/status"
    assert payload["links"]["localization_relocalize"] == "/api/v1/localization/relocalizations"
    assert payload["links"]["localization_map_tracking"] == "/api/v1/localization/map-tracking"
    assert {
        "slam_auto_relocalize",
        "slam_relocalize",
        "slam_track_against_map",
    }.isdisjoint(payload["links"])
    assert payload["links"]["navigation_status"] == "/api/v1/navigation/status"
    assert payload["links"]["runtime_dataflow"] == "/api/v1/runtime/dataflow"
    assert payload["links"]["runtime_dataflow_topic"] == "/api/v1/runtime/dataflow/topic"
    assert payload["links"]["runtime_dataflow_subscribe"] == "/api/v1/runtime/dataflow/subscribe"
    assert "slam_restart" not in payload["links"]
    assert payload["links"]["navigation_goal_candidate"] == "/api/v1/navigation/goal_candidate"
    assert payload["links"]["navigation_plan"] == "/api/v1/navigation/plan"
    assert payload["links"]["field_check"] == "/api/v1/diagnostics/field-check"
    assert payload["links"]["inspection_acceptance"] == "/api/v1/inspection/acceptance"
    assert payload["links"]["navigation_cancel"] == "/api/v1/navigation/cancel"
    assert payload["links"]["routecheck_latest"] == "/api/v1/diagnostics/routecheck/latest"
    assert payload["links"]["real_runtime_evidence_latest"] == "/api/v1/diagnostics/real-runtime-evidence/latest"
    assert payload["links"]["teleop_ws"] == "/ws/teleop"
    assert payload["links"]["camera_ws"] == "/ws/camera"
    assert payload["links"]["session"] == "/api/v1/session"
    assert "session_start" not in payload["links"]
    assert "session_end" not in payload["links"]

    capabilities = build_app_capabilities(gateway)

    assert capabilities["schema_version"] == 2
    assert isinstance(capabilities["ts"], float)
    assert capabilities["server"]["time"] == capabilities["ts"]
    assert capabilities["features"]["exploration"] is False
    assert capabilities["features"]["localization"] is True
    assert capabilities["features"]["runtime_dataflow"] is True
    assert "slam_restart" not in capabilities["features"]
    assert capabilities["features"]["whep"] is True
    assert capabilities["realtime"]["scene_layers"]["schema_version"] == 1
    assert capabilities["realtime"]["scene_layers"]["layers"][1]["id"] == "live_cloud"
    assert capabilities["endpoints"]["app"]["bootstrap"]["path"] == "/api/v1/app/bootstrap"
    assert capabilities["endpoints"]["app"]["traffic"]["path"] == "/api/v1/app/traffic"
    assert capabilities["endpoints"]["state"]["localization_status"]["path"] == "/api/v1/localization/status"
    assert capabilities["endpoints"]["localization"]["relocalize"] == {
        "method": "POST",
        "path": "/api/v1/localization/relocalizations",
    }
    assert capabilities["endpoints"]["localization"]["map_tracking"] == {
        "method": "POST",
        "path": "/api/v1/localization/map-tracking",
    }
    assert capabilities["endpoints"]["state"]["navigation_status"]["path"] == "/api/v1/navigation/status"
    assert capabilities["endpoints"]["state"]["runtime_dataflow"]["path"] == "/api/v1/runtime/dataflow"
    assert capabilities["endpoints"]["state"]["runtime_dataflow_topic"]["path"] == "/api/v1/runtime/dataflow/topic"
    assert (
        capabilities["endpoints"]["state"]["runtime_dataflow_subscribe"]["path"] == "/api/v1/runtime/dataflow/subscribe"
    )
    assert capabilities["endpoints"]["state"]["runtime_dataflow_subscribe"]["method"] == "POST"
    assert capabilities["endpoints"]["state"]["readiness"]["path"] == "/api/v1/readiness"
    assert capabilities["endpoints"]["state"]["metrics"]["path"] == "/api/v1/metrics"
    assert capabilities["endpoints"]["control"]["goal"]["method"] == "POST"
    assert capabilities["endpoints"]["control"]["navigation_goal_candidate"]["method"] == "POST"
    assert capabilities["endpoints"]["control"]["navigation_plan"]["method"] == "POST"
    assert capabilities["endpoints"]["control"]["navigation_cancel"]["method"] == "POST"
    endpoint_paths: set[str] = set()

    def collect_endpoint_paths(value):
        if isinstance(value, dict):
            path = value.get("path")
            if isinstance(path, str):
                endpoint_paths.add(path)
            for child in value.values():
                collect_endpoint_paths(child)
        elif isinstance(value, list):
            for child in value:
                collect_endpoint_paths(child)

    collect_endpoint_paths(capabilities["endpoints"])
    assert "/api/v1/publish" not in endpoint_paths
    assert "/api/v1/runtime/dataflow/publish" not in endpoint_paths
    assert not any(path.endswith("/publish") for path in endpoint_paths)
    assert capabilities["endpoints"]["map"]["maps"] == {
        "method": "GET",
        "path": "/api/v1/slam/maps",
    }
    assert capabilities["endpoints"]["map"]["map_delete"] == {
        "method": "DELETE",
        "path": "/api/v1/maps/{name}",
    }
    assert capabilities["endpoints"]["map"]["map_build_occupancy"] == {
        "method": "POST",
        "path": "/api/v1/maps/{name}/build_occupancy",
    }
    assert "map_lifecycle" not in capabilities["endpoints"]["map"]
    assert "map_activate" not in capabilities["endpoints"]["map"]
    assert capabilities["endpoints"]["map"]["map_rename"]["path"] == "/api/v1/map/rename"
    assert capabilities["endpoints"]["map"]["map_save"]["path"] == "/api/v1/map/save"
    assert "map_restore_predufo" not in capabilities["endpoints"]["map"]
    assert capabilities["probes"]["readiness"]["path"] == "/ready"
    assert capabilities["endpoints"]["ops"]["routecheck_latest"]["path"] == "/api/v1/diagnostics/routecheck/latest"
    assert (
        capabilities["endpoints"]["ops"]["real_runtime_evidence_latest"]["path"]
        == "/api/v1/diagnostics/real-runtime-evidence/latest"
    )
    assert capabilities["endpoints"]["ops"]["inspection_acceptance"]["path"] == "/api/v1/inspection/acceptance"
    assert capabilities["endpoints"]["ops"]["field_check"]["path"] == "/api/v1/diagnostics/field-check"
    assert "slam_restart" not in capabilities["endpoints"]["ops"]
    assert capabilities["endpoints"]["ops"]["field_check"]["method"] == "POST"
    assert capabilities["endpoints"]["ops"]["inspection_acceptance"]["method"] == "POST"
    assert capabilities["realtime"]["events"]["transport"] == "sse"
    assert capabilities["realtime"]["events"]["event_schema"] == "SSEEventEnvelope"
    assert capabilities["realtime"]["events"]["event_id_field"] == "event_id"
    assert capabilities["realtime"]["events"]["retry_ms"] == 3000
    assert capabilities["realtime"]["events"]["replay_supported"] is False
    assert capabilities["realtime"]["events"]["last_event_id_header"] == "Last-Event-ID"
    assert capabilities["realtime"]["events"]["named_events"] is False
    assert capabilities["realtime"]["events"]["browser_handler"] == "onmessage"
    assert {
        "snapshot",
        "ping",
        "slam_diag",
        "gnss_fusion",
        "slam_drift",
        "navigation_status",
        "lease",
        "command_ack",
    } <= set(capabilities["realtime"]["events"]["event_types"])
    assert {"slam_diag", "gnss_fusion", "slam_drift"} <= set(
        capabilities["realtime"]["events"]["diagnostic_event_types"]
    )
    assert "legacy_event_types" not in capabilities["realtime"]["events"]
    assert capabilities["realtime"]["events"]["large_event_policy"]["point_cloud_payload"] == "binary_websocket"
    assert capabilities["realtime"]["teleop"]["binary_camera_frames"] is False
    assert "legacy_camera_query" not in capabilities["realtime"]["teleop"]
    assert capabilities["realtime"]["camera"]["path"] == "/ws/camera"
    assert capabilities["realtime"]["camera"]["binary_camera_frames"] is True
    assert capabilities["endpoints"]["realtime"]["camera"]["method"] == "WS"
    assert capabilities["client_policy"]["retry_safe_when_request_id_present"] is True
    assert capabilities["client_policy"]["commands"]["idempotency_supported"] is True

    traffic = build_app_traffic(gateway)

    assert traffic["schema_version"] == 1
    assert traffic["status"] == "ok"
    assert traffic["server"]["time"] == traffic["ts"]
    assert traffic["sse"]["queue_maxsize"] == gateway._sse_queue_maxsize
    assert traffic["cloud"]["queue_maxsize"] == gateway._cloud_viewer.cloud_queue_maxsize()
    assert traffic["client_policy"]["usage"] == "low_frequency_monitoring"
    assert traffic["client_policy"]["events_endpoint"] == "/api/v1/events"
    assert traffic["client_policy"]["cloud_endpoint"] == "/ws/cloud"
    assert traffic["links"]["traffic"] == "/api/v1/app/traffic"


def test_app_capabilities_expose_env_scoped_product_availability():
    from gateway.gateway_module import GatewayModule
    from gateway.services.app_bootstrap import build_app_capabilities

    class FakePlan:
        env = "sim"
        product = "teleop"
        host_config = {}
        simulation = {
            "session": {"runtime": {"backend": "mujoco"}},
        }

    gateway = GatewayModule(run_plan=FakePlan())

    capabilities = build_app_capabilities(gateway)
    runtime_products = capabilities["runtime_products"]

    assert runtime_products["env"] == "sim"
    assert runtime_products["product"] == "teleop"
    assert runtime_products["state"] == "active"
    assert "run_plan_path" not in runtime_products
    assert "identity_source" not in runtime_products
    assert runtime_products["backend"] == "mujoco"
    assert runtime_products["availability_source"] == "env_backend"
    assert all(item["available"] is True for item in runtime_products["products"].values())


def test_app_capabilities_do_not_advertise_disabled_mapd_management():
    from gateway.gateway_module import GatewayModule
    from gateway.services.app_bootstrap import build_app_capabilities

    class Command:
        argv = ("build/maps-windows/Release/mapd.exe", "--disable-query")

    class Process:
        command = Command()

    class FakePlan:
        env = "sim"
        product = "map"
        host_config = {}
        simulation = {"session": {"runtime": {"backend": "mujoco"}}}

        @staticmethod
        def process(name):
            if name != "maps":
                raise KeyError(name)
            return Process()

    features = build_app_capabilities(GatewayModule(run_plan=FakePlan()))["features"]

    assert features["mapping"] is True
    assert features["map_management"] is False


def test_app_capabilities_keep_all_real_operator_products_available():
    from gateway.gateway_module import GatewayModule
    from gateway.services.app_bootstrap import build_app_capabilities

    class FakePlan:
        env = "real"
        product = "teleop"
        host_config = {}
        simulation = {}

    gateway = GatewayModule(run_plan=FakePlan())
    products = build_app_capabilities(gateway)["runtime_products"]["products"]

    assert {
        "teleop",
        "teleop_avoid",
        "map",
        "explore",
        "nav",
        "tracking",
        "inspection",
    } == {name for name, item in products.items() if item["available"] is True}


def test_app_capabilities_fail_closed_when_sim_backend_is_unresolved(monkeypatch):
    from gateway.gateway_module import GatewayModule
    from gateway.services.app_bootstrap import build_app_capabilities

    class FakePlan:
        env = "sim"
        product = "teleop"
        host_config = {}
        simulation = {}

    monkeypatch.delenv("LINGTU_ENV_BACKEND", raising=False)
    gateway = GatewayModule(run_plan=FakePlan())

    products = build_app_capabilities(gateway)["runtime_products"]["products"]

    assert all(item["available"] is False for item in products.values())
    assert products["teleop"]["reason"] == "sim backend is not resolved by the active RunPlan"


def test_app_capabilities_do_not_infer_backend_from_process_environment(monkeypatch):
    from gateway.gateway_module import GatewayModule
    from gateway.services.app_bootstrap import build_app_capabilities

    class FakePlan:
        env = "sim"
        product = "teleop"
        host_config = {}
        simulation = {}

    monkeypatch.setenv("LINGTU_ENV_BACKEND", "mujoco")
    gateway = GatewayModule(run_plan=FakePlan())

    runtime_products = build_app_capabilities(gateway)["runtime_products"]

    assert runtime_products["backend"] is None
    assert all(item["available"] is False for item in runtime_products["products"].values())


def test_app_capabilities_fail_closed_for_unknown_run_plan_backend():
    from gateway.gateway_module import GatewayModule
    from gateway.services.app_bootstrap import build_app_capabilities

    class FakePlan:
        env = "sim"
        product = "teleop"
        host_config = {}
        simulation = {"session": {"runtime": {"backend": "bogus"}}}

    gateway = GatewayModule(run_plan=FakePlan())

    runtime_products = build_app_capabilities(gateway)["runtime_products"]

    assert runtime_products["backend"] == "bogus"
    assert all(item["available"] is False for item in runtime_products["products"].values())
    assert runtime_products["products"]["teleop"]["reason"] == (
        "sim backend 'bogus' is not declared by the active Runtime Graph"
    )


def test_app_capabilities_fail_closed_without_an_active_run_plan(monkeypatch):
    from gateway.gateway_module import GatewayModule
    from gateway.services.app_bootstrap import build_app_capabilities

    monkeypatch.delenv("LINGTU_ENV", raising=False)
    monkeypatch.delenv("LINGTU_PRODUCT", raising=False)
    gateway = GatewayModule()

    runtime_products = build_app_capabilities(gateway)["runtime_products"]

    assert runtime_products["state"] == "standby"
    assert "run_plan_path" not in runtime_products
    assert "identity_source" not in runtime_products
    assert all(item["available"] is False for item in runtime_products["products"].values())
    assert runtime_products["products"]["teleop"]["reason"] == ("active RunPlan is unavailable")


def test_app_bootstrap_disables_motion_controls_when_safety_stop_active():
    from gateway.gateway_module import GatewayModule
    from gateway.services.app_bootstrap import build_app_bootstrap

    gateway = GatewayModule()
    gateway._session_mode = "navigating"
    gateway._icp_quality = 0.03
    with gateway._state_lock:
        gateway._odom = {"x": 0.0, "y": 0.0, "z": 0.0}
        gateway._navigation_state = {
            "lifecycle_state_name": "IDLE",
            "authority": "estop",
            "hold_reason": "operator_estop",
            "ts": time.time(),
        }
        gateway._mode = "autonomous"
        gateway._localization_status = {
            "state": "TRACKING",
            "confidence": 0.9,
            "degeneracy": "NONE",
            "odom_age_ms": 100.0,
            "localizer_health": "RECOVERED",
        }

    payload = build_app_bootstrap(gateway)

    assert payload["safety"]["ok"] is False
    assert payload["safety"]["stop_active"] is True
    assert payload["navigation"]["goal_admission"]["state"] == "BLOCKED"
    assert payload["navigation"]["motion"]["permission"] == "ESTOPPED"
    assert payload["navigation"]["motion"]["reason"] == "estop_latched"


def test_app_bootstrap_blocks_goal_when_navigation_session_inactive():
    from gateway.gateway_module import GatewayModule
    from gateway.services.app_bootstrap import build_app_bootstrap

    gateway = GatewayModule()
    gateway._session_mode = "idle"
    gateway._icp_quality = 0.03
    with gateway._state_lock:
        gateway._odom = {"x": 0.0, "y": 0.0, "z": 0.0}
        gateway._navigation_state = {
            "lifecycle_state_name": "IDLE",
            "authority": "none",
            "ts": time.time(),
        }
        gateway._mode = "autonomous"
        gateway._localization_status = {
            "state": "TRACKING",
            "confidence": 0.9,
            "degeneracy": "NONE",
            "odom_age_ms": 100.0,
            "localizer_health": "RECOVERED",
        }

    payload = build_app_bootstrap(gateway)

    assert payload["navigation"]["goal_admission"] == {
        "state": "BLOCKED",
        "reason": "navigation_session_inactive",
    }
    assert payload["links"]["navigation_cancel"] == "/api/v1/navigation/cancel"


def test_app_bootstrap_includes_camera_media_runtime_status():
    from gateway.gateway_module import GatewayModule
    from gateway.services.app_bootstrap import build_app_bootstrap

    class Camera:
        def health(self):
            return {
                "backend": "dds",
                "ports_out": {
                    "color_image": {
                        "msg_count": 3,
                        "rate_hz": 12.25,
                        "stale_ms": 42.0,
                    },
                    "depth_image": {
                        "msg_count": 2,
                        "rate_hz": 10.0,
                        "stale_ms": 50.0,
                    },
                    "camera_info": {
                        "msg_count": 1,
                    },
                },
                "camera_info_active_topic": "/camera/color/camera_info",
                "camera_info_preferred_topic": "/camera/color/camera_info",
                "camera_info_topics": [
                    "/camera/color/camera_info",
                    "/camera/depth/camera_info",
                ],
                "reconnect_count": 1,
                "service_recovery_allowed": False,
                "service_recovery_suppressed": True,
            }

    gateway = GatewayModule()
    gateway._all_modules = {
        "camera": Camera(),
    }
    gateway.push_jpeg(b"jpeg-frame")

    payload = build_app_bootstrap(gateway)
    camera = payload["media"]["camera"]

    assert payload["media"]["whep"]["supported"] is True
    assert payload["media"]["whep"]["go2rtc_status"] == "/api/v1/webrtc/go2rtc/status"
    assert camera["available"] is True
    assert camera["status"] == "streaming"
    assert camera["reason"] is None
    assert camera["backend"] == "dds"
    assert camera["fps"] == 12.2
    assert camera["frames"] == 3
    assert camera["depth"]["frames"] == 2
    assert camera["camera_info"]["frames"] == 1
    assert camera["camera_info"]["active_topic"] == "/camera/color/camera_info"
    assert camera["jpeg"]["cached"] is True
    assert camera["jpeg"]["seq"] == 1
    assert camera["jpeg"]["bytes"] == len(b"jpeg-frame")
    assert camera["service_recovery_suppressed"] is True


def test_app_capabilities_report_native_exploration_backend(monkeypatch):
    from gateway.gateway_module import GatewayModule
    from gateway.services import exploration
    from gateway.services.app_bootstrap import build_app_capabilities

    gateway = GatewayModule()
    monkeypatch.setattr(exploration, "_native_status", lambda: {"active": False})
    monkeypatch.setattr(exploration, "_native_commands", lambda _gw: object())

    payload = build_app_capabilities(gateway)

    assert payload["features"]["exploration"] is True


def test_app_bootstrap_falls_back_when_session_snapshot_fails():
    from gateway.gateway_module import GatewayModule
    from gateway.services.app_bootstrap import build_app_bootstrap

    gateway = GatewayModule()

    def broken_session_snapshot():
        raise RuntimeError("boom")

    gateway._session_snapshot = broken_session_snapshot

    payload = build_app_bootstrap(gateway)

    assert payload["session"]["mode"] == "idle"
    assert "error" not in payload["session"]
    assert payload["localization"]["state"] == "no_odometry"


def test_app_bootstrap_route_endpoint_returns_payload():
    from fastapi import FastAPI

    from gateway.gateway_module import GatewayModule
    from gateway.routes.app import register_app_routes

    gateway = GatewayModule()
    app = FastAPI()
    register_app_routes(app, gateway)

    route = next(route for route in app.routes if route.path == "/api/v1/app/bootstrap")
    payload = asyncio.run(route.endpoint())

    assert payload["schema_version"] == 4
    assert payload["ts"] > 0
    assert payload["links"]["health"] == "/api/v1/health"

    assert all(route.path != "/api/v1/bootstrap" for route in app.routes)

    cap_route = next(route for route in app.routes if route.path == "/api/v1/app/capabilities")
    capabilities = asyncio.run(cap_route.endpoint())

    assert capabilities["schema_version"] == 2
    assert capabilities["ts"] > 0
    assert capabilities["endpoints"]["state"]["snapshot"]["path"] == "/api/v1/state"
    assert capabilities["endpoints"]["state"]["navigation_status"]["path"] == "/api/v1/navigation/status"

    traffic_route = next(route for route in app.routes if route.path == "/api/v1/app/traffic")
    traffic = asyncio.run(traffic_route.endpoint())

    assert traffic["schema_version"] == 1
    assert traffic["status"] == "ok"
    assert traffic["links"]["events"] == "/api/v1/events"


def test_client_readiness_endpoint_returns_details_without_probe_503():
    from fastapi.testclient import TestClient

    from gateway.gateway_module import GatewayModule

    gateway = GatewayModule()
    gateway.setup()
    client = TestClient(gateway._app)

    probe = client.get("/ready")
    client_status = client.get("/api/v1/readiness")

    assert probe.status_code == 503
    assert client_status.status_code == 200
    payload = client_status.json()
    assert payload["schema_version"] == 1
    assert payload["status"] == "not_started"
    assert payload["ready"] is False
    assert payload["reasons"] == ["no_modules_loaded"]


def test_app_capabilities_enriches_specs_from_openapi():
    from gateway.gateway_module import GatewayModule

    gateway = GatewayModule()
    gateway.setup()

    route = next(route for route in gateway._app.routes if route.path == "/api/v1/app/capabilities")
    capabilities = asyncio.run(route.endpoint())

    retired = {
        "bag_start",
        "bag_stop",
        "bag_status",
        "algorithm_benchmark_latest",
    }
    assert retired.isdisjoint(capabilities["links"])
    assert retired.isdisjoint(capabilities["endpoints"]["ops"])
    assert "algorithm_benchmark" not in capabilities["features"]

    app_traffic = capabilities["endpoints"]["app"]["traffic"]
    state = capabilities["endpoints"]["state"]["snapshot"]
    scene_graph = capabilities["endpoints"]["state"]["scene_graph"]
    locations = capabilities["endpoints"]["state"]["locations"]
    path = capabilities["endpoints"]["state"]["path"]
    localization = capabilities["endpoints"]["state"]["localization_status"]
    navigation = capabilities["endpoints"]["state"]["navigation_status"]
    readiness = capabilities["endpoints"]["state"]["readiness"]
    runtime_dataflow_topic = capabilities["endpoints"]["state"]["runtime_dataflow_topic"]
    runtime_dataflow_subscribe = capabilities["endpoints"]["state"]["runtime_dataflow_subscribe"]
    control = capabilities["endpoints"]["control"]
    navigation_goal_candidate = capabilities["endpoints"]["control"]["navigation_goal_candidate"]
    navigation_plan = capabilities["endpoints"]["control"]["navigation_plan"]
    navigation_cancel = capabilities["endpoints"]["control"]["navigation_cancel"]
    goal = capabilities["endpoints"]["control"]["goal"]
    map_list = capabilities["endpoints"]["map"]["maps"]
    map_delete = capabilities["endpoints"]["map"]["map_delete"]
    map_build_occupancy = capabilities["endpoints"]["map"]["map_build_occupancy"]
    session = capabilities["endpoints"]["map"]["session"]
    map_rename = capabilities["endpoints"]["map"]["map_rename"]
    map_save = capabilities["endpoints"]["map"]["map_save"]
    localization_relocalize = capabilities["endpoints"]["localization"]["relocalize"]
    localization_map_tracking = capabilities["endpoints"]["localization"]["map_tracking"]
    recording_start = capabilities["endpoints"]["ops"]["recording_start"]
    memory_semantic = capabilities["endpoints"]["ops"]["memory_temporal_semantic"]
    field_check = capabilities["endpoints"]["ops"]["field_check"]
    inspection_acceptance = capabilities["endpoints"]["ops"]["inspection_acceptance"]
    webrtc_whep = capabilities["endpoints"]["media"]["webrtc_whep"]
    events = capabilities["endpoints"]["realtime"]["events"]
    camera = capabilities["endpoints"]["media"]["camera_snapshot"]

    assert app_traffic["response_schema"] == "AppTrafficResponse"
    assert state["response_schema"] == "StateResponse"
    assert scene_graph["response_schema"] == "SceneGraphResponse"
    assert locations["response_schema"] == "LocationsResponse"
    assert path["response_schema"] == "PathResponse"
    assert localization["response_schema"] == "LocalizationStatusResponse"
    assert navigation["response_schema"] == "NavigationStatusResponse"
    assert readiness["response_schema"] == "ReadinessResponse"
    assert runtime_dataflow_topic["response_schema"] == "RuntimeDataflowTopicDetailResponse"
    assert runtime_dataflow_subscribe["request_schema"] == "RuntimeDataflowSubscribeRequest"
    assert runtime_dataflow_subscribe["response_schema"] == "RuntimeDataflowSubscribeResponse"
    assert navigation_goal_candidate["request_schema"] == "GoalCandidateRequest"
    assert navigation_goal_candidate["response_schema"] == "GoalCandidateResponse"
    assert navigation_plan["request_schema"] == "PlanPreviewRequest"
    assert navigation_plan["response_schema"] == "PlanPreviewResponse"
    assert navigation_cancel["request_schema"] == "CancelRequest"
    assert navigation_cancel["response_schema"] == "ControlCommandResponse"
    assert goal["request_schema"] == "GoalRequest"
    assert goal["response_schema"] == "ControlCommandResponse"
    for name in (
        "goal",
        "navigate_click",
        "navigation_cancel",
        "stop",
        "instruction",
        "mode",
        "lease",
    ):
        assert "409" in control[name]["status_codes"]
    assert "403" in control["lease"]["status_codes"]
    assert map_list["path"] == "/api/v1/slam/maps"
    assert map_list["response_schema"] == "MapListResponse"
    assert map_delete["response_schema"] == "MapLifecycleResponse"
    assert map_build_occupancy["response_schema"] == "MapLifecycleResponse"
    assert session["method"] == "GET"
    assert session["response_schema"] == "SessionResponse"
    assert "session_start" not in capabilities["endpoints"]["map"]
    assert "session_end" not in capabilities["endpoints"]["map"]
    assert map_rename["request_schema"] == "MapRenameRequest"
    assert map_rename["response_schema"] == "MapLifecycleResponse"
    assert map_save["request_schema"] == "MapSaveRequest"
    assert map_save["response_schema"] == "MapSaveOperationResponse"
    assert "slam_switch" not in capabilities["endpoints"]["ops"]
    assert localization_relocalize["request_schema"] == "LocalizationRelocalizationRequest"
    assert localization_relocalize["response_schema"] == "LocalizationOperationResponse"
    assert localization_map_tracking["request_schema"] == "LocalizationMapTrackingRequest"
    assert localization_map_tracking["response_schema"] == "LocalizationOperationResponse"
    assert {
        "slam_auto_relocalize",
        "slam_relocalize",
        "slam_track_against_map",
    }.isdisjoint(capabilities["endpoints"]["ops"])
    assert recording_start["path"] == "/api/v1/recordings/start"
    assert recording_start["request_schema"] == "RecordingStartRequest"
    assert memory_semantic["request_schema"] == "TemporalSemanticRequest"
    assert field_check["request_schema"] == "ProductFieldCheckRequest"
    assert field_check["response_schema"] == "ProductFieldCheckResponse"
    assert inspection_acceptance["request_schema"] == "InspectionAcceptanceRequest"
    assert inspection_acceptance["response_schema"] == "InspectionAcceptanceResponse"
    assert webrtc_whep["method"] == "POST"
    assert webrtc_whep["path"] == "/api/v1/webrtc/whep"
    assert events["response_schema"] == "SSEEventEnvelope"
    assert events["response_content_types"] == ["text/event-stream"]
    assert "image/jpeg" in camera["response_content_types"]


def test_runtime_products_reuse_host_declarations_without_freezing_identity(monkeypatch):
    from copy import deepcopy
    from types import SimpleNamespace

    from gateway.services.app_bootstrap import _runtime_product_capabilities
    from lingtu.assembly.graph import loader

    declarations = SimpleNamespace(
        products={"teleop": {"operator_switchable": True}},
        envs={"real": {"supported_products": ["teleop"]}},
    )
    loads = 0

    def load_declarations():
        nonlocal loads
        loads += 1
        return deepcopy(declarations)

    monkeypatch.setattr(loader, "load_runtime_graph", load_declarations)
    gateway = SimpleNamespace(
        _compiled_env="real", _compiled_product="teleop",
        _compiled_product_session_id="session-a", _compiled_run_plan=object(),
    )
    first = _runtime_product_capabilities(gateway)
    first["products"]["teleop"]["variants"].append("response-only")
    declarations.envs["real"]["supported_products"].clear()
    gateway._compiled_product_session_id = "session-b"
    second = _runtime_product_capabilities(gateway)
    assert loads == 1
    assert second["product_session_id"] == "session-b"
    assert second["products"]["teleop"]["available"] is True
    assert second["products"]["teleop"]["variants"] == []

    gateway._compiled_run_plan = None
    stopped = _runtime_product_capabilities(gateway)
    assert stopped["state"] == "standby"
    assert stopped["products"]["teleop"]["available"] is False
    assert loads == 1

    next_host = SimpleNamespace(_compiled_env="real", _compiled_run_plan=object())
    refreshed = _runtime_product_capabilities(next_host)
    assert loads == 2
    assert refreshed["products"]["teleop"]["available"] is False


def test_app_capabilities_reuses_openapi_contract_cache_without_freezing_runtime(
    monkeypatch,
):
    import gateway.services.app_bootstrap as app_bootstrap
    from gateway.gateway_module import GatewayModule

    gateway = GatewayModule()
    gateway.setup()
    if hasattr(gateway, app_bootstrap._OPERATION_CONTRACT_CACHE_ATTR):
        delattr(gateway, app_bootstrap._OPERATION_CONTRACT_CACHE_ATTR)

    calls = 0
    original_openapi = gateway._app.openapi

    def counted_openapi():
        nonlocal calls
        calls += 1
        return original_openapi()

    clock = {"value": 100.0}

    def next_time():
        clock["value"] += 1.0
        return clock["value"]

    monkeypatch.setattr(gateway._app, "openapi", counted_openapi)
    monkeypatch.setattr(app_bootstrap.time, "time", next_time)

    first = app_bootstrap.build_app_capabilities(gateway)
    second = app_bootstrap.build_app_capabilities(gateway)

    assert calls == 1
    assert second["ts"] > first["ts"]
    assert first["server"]["time"] == first["ts"]
    assert second["server"]["time"] == second["ts"]
    assert first["features"]["whep"] is True
    assert second["features"]["whep"] is True
    assert second["endpoints"]["app"]["bootstrap"]["response_schema"] == "AppBootstrapResponse"


def test_app_web_cold_start_routes_return_stable_client_shapes(monkeypatch):
    from fastapi.testclient import TestClient

    from gateway.gateway_module import GatewayModule
    from gateway.navigation import status as navigation_status
    from gateway.services.native_control import status_is_fresh

    snapshot_time = time.time()
    monkeypatch.setattr(
        navigation_status,
        "native_control_status_is_fresh",
        lambda snapshot: status_is_fresh(snapshot, now_s=snapshot_time),
    )

    gateway = GatewayModule()
    gateway.setup()
    snapshot_time = time.time()
    with gateway._state_lock:
        gateway._odom = {
            "x": 1.25,
            "y": -0.5,
            "z": 0.0,
            "yaw": 0.2,
            "frame_id": "map",
            "ts": 101.0,
        }
        gateway._navigation_state = {
            "lifecycle_state_name": "IDLE",
            "authority": "none",
            "ts": snapshot_time,
        }
        gateway._mode = "manual"
        gateway._last_path = [
            {"x": 1.25, "y": -0.5, "z": 0.0},
            {"x": 2.0, "y": 0.0, "z": 0.0},
        ]
        gateway._sg_json = json.dumps(
            {
                "frame_id": "map",
                "ts": 102.0,
                "objects": [
                    {
                        "id": "obj-1",
                        "label": "dock",
                        "x": 2.0,
                        "y": 0.0,
                        "confidence": 0.9,
                    }
                ],
            }
        )

    client = TestClient(gateway._app)

    bootstrap = client.get("/api/v1/app/bootstrap")
    traffic = client.get("/api/v1/app/traffic")
    state = client.get("/api/v1/state")
    readiness = client.get("/api/v1/readiness")
    scene_graph = client.get("/api/v1/scene_graph")
    locations = client.get("/api/v1/locations")
    path = client.get("/api/v1/path")
    capabilities = client.get("/api/v1/app/capabilities")

    for response in (
        bootstrap,
        traffic,
        state,
        readiness,
        scene_graph,
        locations,
        path,
        capabilities,
    ):
        assert response.status_code == 200

    bootstrap_payload = bootstrap.json()
    traffic_payload = traffic.json()
    state_payload = state.json()
    readiness_payload = readiness.json()
    scene_graph_payload = scene_graph.json()
    locations_payload = locations.json()
    path_payload = path.json()
    capabilities_payload = capabilities.json()

    assert bootstrap_payload["schema_version"] == 4
    assert bootstrap_payload["ts"] > 0
    assert bootstrap_payload["server"]["time"] == bootstrap_payload["ts"]
    assert bootstrap_payload["traffic"]["client_policy"]["usage"] == "cold_start_only"
    assert bootstrap_payload["links"]["state"] == "/api/v1/state"
    assert bootstrap_payload["links"]["events"] == "/api/v1/events"
    assert bootstrap_payload["links"]["traffic"] == "/api/v1/app/traffic"
    assert bootstrap_payload["links"]["navigation_cancel"] == "/api/v1/navigation/cancel"
    assert bootstrap_payload["links"]["explore_directed"] == "/api/v1/explore/directed"
    assert bootstrap_payload["links"]["explore_directed_clear"] == "/api/v1/explore/directed/clear"
    assert bootstrap_payload["media"]["webrtc_whep"] == "/api/v1/webrtc/whep"
    assert bootstrap_payload["media"]["go2rtc_status"] == "/api/v1/webrtc/go2rtc/status"
    assert "scene_graph" not in bootstrap_payload["scene"]
    assert "path" not in bootstrap_payload["path"]
    assert set(bootstrap_payload["control"]) == {"teleop", "command_policy"}

    assert traffic_payload["schema_version"] == 1
    assert traffic_payload["ts"] > 0
    assert traffic_payload["server"]["time"] == traffic_payload["ts"]
    assert traffic_payload["status"] == "ok"
    assert traffic_payload["sse"]["clients"] == 0
    assert traffic_payload["cloud"]["clients"] == 0
    assert traffic_payload["client_policy"]["usage"] == "low_frequency_monitoring"
    assert traffic_payload["client_policy"]["events_endpoint"] == "/api/v1/events"
    assert traffic_payload["client_policy"]["traffic_endpoint"] == "/api/v1/app/traffic"
    assert traffic_payload["warnings"] == []

    assert state_payload["schema_version"] == 4
    assert state_payload["ts"] > 0
    assert state_payload["server"]["time"] == state_payload["ts"]
    assert state_payload["links"]["state"] == "/api/v1/state"
    assert state_payload["links"]["events"] == "/api/v1/events"
    assert state_payload["links"]["scene_graph"] == "/api/v1/scene_graph"
    assert state_payload["links"]["locations"] == "/api/v1/locations"
    assert state_payload["links"]["path"] == "/api/v1/path"
    assert state_payload["links"]["camera_ws"] == "/ws/camera"
    assert state_payload["links"]["cloud_ws"] == "/ws/cloud"
    assert state_payload["links"]["health"] == "/api/v1/health"
    assert state_payload["links"]["readiness"] == "/api/v1/readiness"
    assert state_payload["links"]["goal"] == "/api/v1/goal"
    assert state_payload["links"]["stop"] == "/api/v1/stop"
    assert state_payload["links"]["navigation_cancel"] == "/api/v1/navigation/cancel"
    assert state_payload["path"]["points"] == 2
    assert state_payload["localization"]["odometry"]["x"] == 1.25
    assert state_payload["navigation"]["task"]["state"] == "IDLE"
    assert set(state_payload["navigation"]) == {
        "schema_version",
        "task",
        "goal_admission",
        "control",
        "motion",
        "ts",
    }

    assert readiness_payload["schema_version"] == 1
    assert readiness_payload["status"] in {"ready", "degraded", "not_started"}
    assert readiness_payload["ts"] > 0
    assert isinstance(readiness_payload["reasons"], list)
    assert isinstance(readiness_payload["modules"], dict)

    assert scene_graph_payload["schema_version"] == 1
    assert scene_graph_payload["frame_id"] == "map"
    assert scene_graph_payload["count"] == 1
    assert scene_graph_payload["objects"][0]["label"] == "dock"
    assert scene_graph_payload["scene_graph"] is not None

    assert locations_payload["schema_version"] == 1
    assert locations_payload["locations"] == []
    assert locations_payload["count"] == 0

    assert path_payload["schema_version"] == 1
    assert path_payload["count"] == 2
    assert path_payload["robot"]["x"] == 1.25
    assert path_payload["path"][1]["x"] == 2.0

    assert capabilities_payload["schema_version"] == 2
    assert capabilities_payload["ts"] > 0
    assert capabilities_payload["server"]["time"] == capabilities_payload["ts"]
    assert capabilities_payload["endpoints"]["app"]["bootstrap"]["response_schema"] == "AppBootstrapResponse"
    assert capabilities_payload["endpoints"]["app"]["traffic"]["response_schema"] == "AppTrafficResponse"
    assert capabilities_payload["endpoints"]["state"]["snapshot"]["response_schema"] == "StateResponse"
    assert capabilities_payload["endpoints"]["state"]["readiness"]["response_schema"] == "ReadinessResponse"
    assert (
        capabilities_payload["endpoints"]["state"]["runtime_dataflow_topic"]["path"] == "/api/v1/runtime/dataflow/topic"
    )
    assert capabilities_payload["links"]["events"] == "/api/v1/events"
    assert capabilities_payload["links"]["readiness"] == "/api/v1/readiness"
    assert "map_activate" not in capabilities_payload["links"]
    assert capabilities_payload["links"]["map_rename"] == "/api/v1/map/rename"
    assert capabilities_payload["links"]["map_save"] == "/api/v1/map/save"
    assert capabilities_payload["links"]["navigation_goal_candidate"] == "/api/v1/navigation/goal_candidate"
    assert capabilities_payload["links"]["navigation_cancel"] == "/api/v1/navigation/cancel"


def test_app_web_events_stream_starts_with_snapshot_contract():
    from gateway.gateway_module import GatewayModule

    async def read_first_event(gateway):
        route = next(route for route in gateway._app.routes if route.path == "/api/v1/events")
        response = await route.endpoint()
        iterator = response.body_iterator
        try:
            chunk = await iterator.__anext__()
        finally:
            close = getattr(iterator, "aclose", None)
            if close is not None:
                await close()
        return response, chunk

    gateway = GatewayModule()
    gateway.setup()
    with gateway._state_lock:
        gateway._odom = {"x": 1.0, "y": 2.0, "yaw": 0.25, "ts": 201.0}
        gateway._navigation_state = {
            "lifecycle_state_name": "IDLE",
            "authority": "none",
            "ts": time.time(),
        }
        gateway._mode = "manual"

    response, chunk = asyncio.run(read_first_event(gateway))
    line = chunk.decode("utf-8") if isinstance(chunk, bytes) else chunk
    data_line = next(item for item in line.splitlines() if item.startswith("data: "))
    payload = json.loads(data_line.removeprefix("data: "))

    assert response.media_type == "text/event-stream"
    assert payload["type"] == "snapshot"
    assert payload["data"]["localization"]["odometry"]["x"] == 1.0
    assert payload["schema_version"] == 1
    assert payload["event_id"] == 1
    assert payload["ts"] > 0
    assert payload["data"]["navigation"]["task"]["state"] == "IDLE"
    assert payload["data"]["session"]["mode"] in {"idle", "navigating", "mapping"}
    assert payload["data"]["lease"]["active"] is False
    assert set(payload["data"]["navigation"]) == {
        "schema_version",
        "task",
        "goal_admission",
        "control",
        "motion",
        "ts",
    }
    assert gateway._traffic_stats_snapshot()["sse"]["clients"] == 0


def test_app_web_events_stream_uses_sse_ids_without_named_event_type():
    from gateway.gateway_module import GatewayModule

    async def read_first_event(gateway):
        route = next(route for route in gateway._app.routes if route.path == "/api/v1/events")
        response = await route.endpoint()
        iterator = response.body_iterator
        try:
            chunk = await iterator.__anext__()
        finally:
            close = getattr(iterator, "aclose", None)
            if close is not None:
                await close()
        return chunk

    gateway = GatewayModule()
    gateway.setup()

    chunk = asyncio.run(read_first_event(gateway))
    text = chunk.decode("utf-8") if isinstance(chunk, bytes) else chunk

    assert text.startswith("retry: 3000\nid: 1\ndata: ")
    assert "\nevent:" not in text
    data_line = next(line for line in text.splitlines() if line.startswith("data: "))
    payload = json.loads(data_line.removeprefix("data: "))

    assert payload["type"] == "snapshot"
    assert payload["schema_version"] == 1
    assert payload["event_id"] == 1


def test_app_web_events_stream_flushes_live_event_before_heartbeat_delay():
    from gateway.gateway_module import GatewayModule

    def decode_payload(chunk):
        text = chunk.decode("utf-8") if isinstance(chunk, bytes) else chunk
        data_line = next(line for line in text.splitlines() if line.startswith("data: "))
        return json.loads(data_line.removeprefix("data: "))

    async def read_next_payload_after_delayed_push(gateway):
        route = next(route for route in gateway._app.routes if route.path == "/api/v1/events")
        response = await route.endpoint()
        iterator = response.body_iterator
        next_chunk_task = None
        try:
            first_payload = decode_payload(await iterator.__anext__())
            next_chunk_task = asyncio.create_task(iterator.__anext__())
            await asyncio.sleep(0.01)
            gateway.push_event(
                {
                    "type": "navigation_status",
                    "data": {"task": {"state": "IDLE"}},
                }
            )
            second_payload = decode_payload(await asyncio.wait_for(next_chunk_task, timeout=0.5))
            return first_payload, second_payload
        finally:
            if next_chunk_task is not None and not next_chunk_task.done():
                next_chunk_task.cancel()
            close = getattr(iterator, "aclose", None)
            if close is not None:
                await close()

    gateway = GatewayModule()
    gateway.setup()

    first, second = asyncio.run(read_next_payload_after_delayed_push(gateway))

    assert first["type"] == "snapshot"
    assert second["type"] == "navigation_status"
    assert second["data"]["task"]["state"] == "IDLE"
    assert second["event_id"] > first["event_id"]
    assert gateway._traffic_stats_snapshot()["sse"]["clients"] == 0


def test_app_web_events_stream_filters_runtime_dataflow_topic():
    from gateway.gateway_module import GatewayModule
    from lingtu.assembly.compiler import compile_run_plan
    from lingtu.assembly.products import resolve_product_host_runtime
    from runtime.msgs.nav import Odometry

    def decode_payload(chunk):
        text = chunk.decode("utf-8") if isinstance(chunk, bytes) else chunk
        data_line = next(line for line in text.splitlines() if line.startswith("data: "))
        return json.loads(data_line.removeprefix("data: "))

    async def read_filtered_payloads(gateway):
        route = next(route for route in gateway._app.routes if route.path == "/api/v1/events")
        response = await route.endpoint(topic="odometry")
        iterator = response.body_iterator
        next_chunk_task = None
        try:
            first_payload = decode_payload(await iterator.__anext__())
            next_chunk_task = asyncio.create_task(iterator.__anext__())
            await asyncio.sleep(0.01)
            gateway.push_event(
                {
                    "type": "navigation_status",
                    "data": {"task": {"state": "IDLE"}},
                }
            )
            await asyncio.sleep(0.01)
            gateway.odometry._deliver(Odometry())
            second_payload = decode_payload(await asyncio.wait_for(next_chunk_task, timeout=0.5))
            return first_payload, second_payload
        finally:
            if next_chunk_task is not None and not next_chunk_task.done():
                next_chunk_task.cancel()
            close = getattr(iterator, "aclose", None)
            if close is not None:
                await close()

    resolved = resolve_product_host_runtime("nav", "real", robot="unitree/go2")
    gateway = GatewayModule(
        run_plan=compile_run_plan(
            resolved.product,
            resolved.env,
            robot="unitree/go2",
        )
    )
    gateway.setup()

    first, second = asyncio.run(read_filtered_payloads(gateway))

    assert first["type"] == "runtime_dataflow_subscription"
    assert first["data"]["ok"] is True
    assert first["data"]["selector"] == "odometry"
    assert first["data"]["event_types"] == ["odometry"]
    assert second["type"] == "odometry"
    assert second["event_id"] > first["event_id"]
    assert gateway._traffic_stats_snapshot()["sse"]["clients"] == 0


def test_app_web_events_reconnect_snapshot_uses_latest_state_and_monotonic_id():
    from gateway.gateway_module import GatewayModule

    async def read_first_payload(gateway):
        route = next(route for route in gateway._app.routes if route.path == "/api/v1/events")
        response = await route.endpoint()
        iterator = response.body_iterator
        try:
            chunk = await iterator.__anext__()
        finally:
            close = getattr(iterator, "aclose", None)
            if close is not None:
                await close()
        text = chunk.decode("utf-8") if isinstance(chunk, bytes) else chunk
        data_line = next(line for line in text.splitlines() if line.startswith("data: "))
        return json.loads(data_line.removeprefix("data: "))

    gateway = GatewayModule()
    gateway.setup()
    with gateway._state_lock:
        gateway._odom = {"x": 1.0, "y": 2.0, "yaw": 0.25}
        gateway._navigation_state = {"lifecycle_state_name": "IDLE", "ts": time.time()}

    first = asyncio.run(read_first_payload(gateway))

    with gateway._state_lock:
        gateway._odom = {"x": 3.0, "y": 4.0, "yaw": 0.5}
        gateway._navigation_state = {
            "lifecycle_state_name": "EXECUTING",
            "active_task_id": "navigation-task-2",
            "active_request_id": "goal-2",
            "ts": time.time(),
        }
        gateway._navigation_goal_status_by_task["navigation-task-2"] = {
            "task_id": "navigation-task-2",
            "request_id": "goal-2",
            "state_name": "EXECUTING",
        }

    second = asyncio.run(read_first_payload(gateway))

    assert first["type"] == "snapshot"
    assert second["type"] == "snapshot"
    assert second["event_id"] > first["event_id"]
    assert second["data"]["localization"]["odometry"]["x"] == 3.0
    assert second["data"]["navigation"]["task"]["state"] == "EXECUTING"
    assert gateway._traffic_stats_snapshot()["sse"]["clients"] == 0
