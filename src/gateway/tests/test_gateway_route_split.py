from __future__ import annotations

import asyncio
import io
import json
import os
import shutil
import tarfile
import threading
import uuid
from collections import Counter
from pathlib import Path
from types import SimpleNamespace

import pytest

pytestmark = [pytest.mark.sim]


fastapi = pytest.importorskip("fastapi")


def test_auth_routes_register_expected_paths():
    from fastapi import FastAPI
    from fastapi.exceptions import RequestValidationError

    from gateway.routes.auth import register_auth_routes

    app = FastAPI()
    register_auth_routes(app)

    paths = {getattr(route, "path", "") for route in app.routes}
    assert "/api/v1/auth/login" in paths
    assert "/api/v1/auth/check" in paths
    assert RequestValidationError in app.exception_handlers


def test_realtime_routes_register_expected_websockets():
    from fastapi import FastAPI
    from starlette.websockets import WebSocket

    from gateway.routes.realtime import register_realtime_routes

    app = FastAPI()
    gw = SimpleNamespace()
    register_realtime_routes(app, gw)

    paths = {getattr(route, "path", "") for route in app.routes}
    assert "/ws/teleop" in paths
    assert "/ws/camera" in paths
    assert "/ws/cloud" in paths
    assert "/ws/scan" in paths
    for route in app.routes:
        if getattr(route, "path", "") in {"/ws/teleop", "/ws/camera", "/ws/cloud", "/ws/scan"}:
            assert route.endpoint.__annotations__.get("ws") is WebSocket


def test_map_routes_register_expected_paths():
    from fastapi import FastAPI

    from gateway.routes.maps import register_map_routes

    class MapsService:
        def service(self, action, **arguments):
            assert action == "get_voxel_edits"
            assert arguments == {"map_id": "demo"}
            return {
                "action": "get_voxel_edits",
                "success": True,
                "map_id": "demo",
                "edits": [],
            }

    app = FastAPI()
    register_map_routes(app, SimpleNamespace(_map_client=MapsService()))

    paths = {getattr(route, "path", "") for route in app.routes}
    assert "/api/v1/slam/maps" in paths
    assert "/api/v1/maps" not in paths
    assert "/api/v1/maps/{name}" in paths
    assert "/api/v1/maps/{name}/build_occupancy" in paths
    assert "/api/v1/maps/import_pcd" in paths
    assert "/api/v1/maps/{name}/crop" in paths
    assert "/api/v1/maps/{name}/mark_zone" in paths
    assert "/api/v1/maps/{name}/build_octomap" in paths
    assert "/api/v1/maps/{name}/validate_plan" in paths
    assert "/api/v1/maps/{name}/pcd" in paths
    assert "/api/v1/maps/{name}/points" in paths
    assert "/api/v1/maps/{name}/voxels/edit" in paths
    assert "/api/v1/maps/{name}/voxels/edits" in paths
    assert "/api/v1/map/points" in paths
    assert "/api/v1/map_cloud/reset" in paths
    assert "/map/viewer" in paths
    assert "/robot/meshes/{filename}" not in paths
    assert "/api/v1/map/restore_predufo" not in paths
    assert "/api/v1/map/activate" not in paths
    assert "/api/v1/map/rename" in paths
    assert "/api/v1/map/save" in paths
    assert "/api/v1/maps/operations" in paths
    assert "/api/v1/maps/operations/{operation_id}" in paths
    assert "/api/v1/maps/operations/{operation_id}/cancel" in paths
    assert "/api/v1/maps/operations/{operation_id}/retry" in paths
    assert "/api/v1/maps/save-jobs" not in paths
    assert "/api/v1/maps/save-jobs/{job_id}" not in paths
    assert "/api/v1/maps/{name}/versions" not in paths
    assert "/api/v1/maps/{name}/versions/{version}/rollback" not in paths


def test_slam_maps_does_not_turn_native_failure_into_empty_inventory(monkeypatch):
    from fastapi import FastAPI

    import gateway.routes.maps as map_routes

    monkeypatch.setattr(
        map_routes,
        "_mapd_command",
        lambda _gw, _payload: {
            "success": False,
            "reason_code": "native_list_failed",
            "message": "private native detail",
        },
    )
    app = FastAPI()
    map_routes.register_map_routes(app, SimpleNamespace())
    route = next(route for route in app.routes if route.path == "/api/v1/slam/maps")

    response = asyncio.run(route.endpoint())
    payload = json.loads(response.body)

    assert response.status_code == 503
    assert payload["ok"] is False
    assert payload["error"] == "map_service_unavailable"
    assert "private native detail" not in response.body.decode("utf-8")


def test_robot_mesh_defaults_to_bundled_thunder_v4_asset(monkeypatch):
    from fastapi import FastAPI

    from gateway.routes.assets import register_asset_routes, robot_mesh_path

    monkeypatch.delenv("DOG_MESH_DIR", raising=False)
    path = robot_mesh_path("base_link.STL")

    assert path is not None
    assert path.name == "base_link.STL"
    assert path.parent.name == "meshes"
    assert path.parent.parent.name == "thunder_v4"

    app = FastAPI()
    register_asset_routes(app)
    assert "/robot/meshes/{filename}" in {
        getattr(route, "path", "") for route in app.routes
    }


def test_map_content_epoch_routes_are_not_registered():
    from fastapi import FastAPI

    import gateway.routes.maps as map_routes

    app = FastAPI()
    map_routes.register_map_routes(app, SimpleNamespace())

    paths = {route.path for route in app.routes}
    assert "/api/v1/maps/{name}/versions" not in paths
    assert "/api/v1/maps/{name}/versions/{version}/rollback" not in paths


def test_save_map_returns_accepted_while_native_job_is_running(monkeypatch):
    from fastapi import FastAPI

    import gateway.routes.maps as map_routes

    monkeypatch.setattr(
        map_routes,
        "_mapd_command",
        lambda _gw, _payload: {
            "success": False,
            "accepted": True,
            "status": "running",
            "job_id": "save_job_1",
            "message": "SaveMap is still running; query by job_id",
        },
    )
    gateway = SimpleNamespace(_get_slam_profile=lambda: "native_dds")
    app = FastAPI()
    map_routes.register_map_routes(app, gateway)
    route = next(route for route in app.routes if route.path == "/api/v1/map/save")

    response = asyncio.run(route.endpoint({"name": "warehouse"}))
    payload = json.loads(response.body)
    assert response.status_code == 202
    assert payload["ok"] is True
    assert payload["success"] is None
    assert payload["accepted"] is True
    assert payload["operation_id"] == "save_job_1"
    assert "job_id" not in payload


def test_save_map_fails_closed_when_slam_profile_is_unavailable(monkeypatch):
    from fastapi import FastAPI

    import gateway.routes.maps as map_routes

    def submit_save(_gw, _payload):
        raise AssertionError("save must not be submitted without live SLAM profile")

    def read_slam_profile():
        raise RuntimeError("telemetry unavailable")

    monkeypatch.setattr(map_routes, "_mapd_command", submit_save)
    gateway = SimpleNamespace(
        _get_slam_profile=read_slam_profile,
        _session_slam_profile="native_dds",
    )
    app = FastAPI()
    map_routes.register_map_routes(app, gateway)
    route = next(route for route in app.routes if route.path == "/api/v1/map/save")

    response = asyncio.run(route.endpoint({"name": "warehouse"}))
    payload = json.loads(response.body)

    assert response.status_code == 503
    assert payload["ok"] is False
    assert payload["reason_code"] == "slam_profile_unavailable"


def test_map_viewer_serves_static_file_without_gateway_snapshot():
    from fastapi import FastAPI
    from fastapi.responses import FileResponse

    from gateway.routes.maps import MAP_VIEWER_TEMPLATE, register_map_routes

    class Gateway:
        def _generate_viewer_live(self):
            raise AssertionError("map viewer route must not snapshot live map")

        def _generate_viewer_from_pcd(self, map):
            raise AssertionError("map viewer route must not parse saved PCD")

    app = FastAPI()
    register_map_routes(app, Gateway())

    route = next(route for route in app.routes if route.path == "/map/viewer")
    response = asyncio.run(route.endpoint())

    assert isinstance(response, FileResponse)
    assert Path(response.path) == MAP_VIEWER_TEMPLATE
    assert response.headers["cache-control"] == "public, max-age=300"
    html = MAP_VIEWER_TEMPLATE.read_text(encoding="utf-8")
    assert "{coords}" not in html
    assert "{n*3}" not in html
    assert "{robot_visible}" not in html
    assert "_connectCloudWs()" in html


def test_map_voxel_overlay_route_reads_saved_edits(monkeypatch, tmp_path):
    from fastapi import FastAPI

    from gateway.routes.maps import register_map_routes

    monkeypatch.setenv("NAV_MAP_DIR", str(tmp_path))
    map_dir = tmp_path / "demo"
    map_dir.mkdir()
    expected_edit = {
        "state": "preblocked",
        "center": {"x": 1.0, "y": 2.0, "z": 0.4},
        "radius": 0.3,
    }

    class MapsService:
        def service(self, action, **arguments):
            assert action == "get_voxel_edits"
            assert arguments == {"map_id": "demo"}
            return {
                "action": "get_voxel_edits",
                "success": True,
                "map_id": "demo",
                "edits": [expected_edit],
            }

    app = FastAPI()
    register_map_routes(app, SimpleNamespace(_map_client=MapsService()))

    route = next(route for route in app.routes if route.path == "/api/v1/maps/{name}/voxels/edits")
    payload = asyncio.run(route.endpoint("demo"))

    assert payload["success"] is True
    assert payload["edits"][0]["state"] == "preblocked"


def test_diagnostics_maps_snapshot_uses_mapd_transport(monkeypatch):
    from gateway.routes.diagnostics import _maps_snapshot
    from runtime.endpoints import mapd as maps_client

    class Client:
        def service(self, action):
            assert action == "list_maps"
            return {"action": action, "success": True, "maps": [{"name": "demo"}]}

    monkeypatch.setattr(maps_client, "MapClient", Client)
    payload = _maps_snapshot(SimpleNamespace())

    assert payload["has_manager"] is False
    assert payload["source"] == "mapd"
    assert payload["manager"]["success"] is True
    assert payload["manager"]["maps"][0]["name"] == "demo"


def test_status_routes_register_expected_paths():
    from fastapi import FastAPI

    from gateway.routes.status import register_status_routes

    app = FastAPI()
    register_status_routes(app, SimpleNamespace())

    paths = {getattr(route, "path", "") for route in app.routes}
    assert "/api/v1/events" in paths
    assert "/api/v1/state" in paths
    assert "/api/v1/scene_graph" in paths
    assert "/api/v1/locations" in paths
    assert "/api/v1/locations/{name}" in paths
    assert "/api/v1/path" in paths
    assert "/api/v1/localization/status" in paths
    assert "/api/v1/navigation/status" in paths
    assert "/api/v1/runtime/dataflow" in paths
    assert "/api/v1/health" in paths
    assert "/health" in paths
    assert "/ready" in paths


def test_session_routes_register_expected_paths():
    from fastapi import FastAPI

    from gateway.routes.session import register_session_routes

    app = FastAPI()
    gw = SimpleNamespace()
    register_session_routes(app, gw)

    paths = {getattr(route, "path", "") for route in app.routes}
    assert "/api/v1/session" in paths
    assert "/api/v1/session/start" not in paths
    assert "/api/v1/session/end" not in paths


def test_camera_routes_register_expected_snapshot_path():
    from fastapi import FastAPI

    from gateway.routes.camera import register_camera_routes

    app = FastAPI()
    register_camera_routes(app)

    route = next(route for route in app.routes if route.path == "/api/v1/camera/snapshot")
    assert route.endpoint.__module__ == "gateway.routes.camera"


def test_camera_snapshot_returns_cached_gateway_jpeg(monkeypatch):
    from fastapi import FastAPI

    from gateway.routes.camera import register_camera_routes

    def fail_registered_snapshot():
        raise AssertionError("snapshot route should not probe adapters when JPEG is cached")

    monkeypatch.setattr(
        "gateway.routes.camera._registered_snapshot_adapter_jpeg",
        fail_registered_snapshot,
    )

    app = FastAPI()
    gw = SimpleNamespace(_latest_jpeg=b"\xff\xd8\xffcamera", _jpeg_lock=threading.Lock())
    register_camera_routes(app, gw)

    route = next(route for route in app.routes if route.path == "/api/v1/camera/snapshot")
    response = asyncio.run(route.endpoint())

    assert response.status_code == 200
    assert response.media_type == "image/jpeg"
    assert response.body == b"\xff\xd8\xffcamera"


def test_camera_snapshot_uses_camera_relay_one_shot_encoder(monkeypatch):
    from fastapi import FastAPI

    from gateway.routes.camera import register_camera_routes

    def fail_registered_snapshot():
        raise AssertionError("snapshot route should use the camera relay before adapter fallback")

    class Teleop:
        def __init__(self):
            self.calls = 0

        def snapshot_jpeg(self):
            self.calls += 1
            return b"\xff\xd8\xffteleop"

    monkeypatch.setattr(
        "gateway.routes.camera._registered_snapshot_adapter_jpeg",
        fail_registered_snapshot,
    )

    teleop = Teleop()
    app = FastAPI()
    gw = SimpleNamespace(
        _latest_jpeg=None,
        _jpeg_lock=threading.Lock(),
        _camera_module=teleop,
    )
    register_camera_routes(app, gw)

    route = next(route for route in app.routes if route.path == "/api/v1/camera/snapshot")
    response = asyncio.run(route.endpoint())

    assert response.status_code == 200
    assert response.media_type == "image/jpeg"
    assert response.body == b"\xff\xd8\xffteleop"
    assert teleop.calls == 1


def test_camera_snapshot_fast_fails_when_gateway_reports_no_camera(monkeypatch):
    from fastapi import FastAPI

    from gateway.routes.camera import register_camera_routes

    def fail_registered_snapshot():
        raise AssertionError("snapshot route should not probe adapters when camera is unavailable")

    monkeypatch.setattr(
        "gateway.routes.camera._registered_snapshot_adapter_jpeg",
        fail_registered_snapshot,
    )

    app = FastAPI()
    gw = SimpleNamespace(_all_modules={})
    register_camera_routes(app, gw)

    route = next(route for route in app.routes if route.path == "/api/v1/camera/snapshot")
    response = asyncio.run(route.endpoint())
    payload = json.loads(response.body)

    assert response.status_code == 503
    assert payload["error"] == "camera_unavailable"
    assert payload["ok"] is False
    assert payload["detail"]["camera"]["reason"] == "camera_not_loaded"


def test_camera_route_selects_snapshot_adapter_without_plugin_seeding() -> None:
    import gateway.routes.camera as camera_route

    source = Path(camera_route.__file__).read_text(encoding="utf-8-sig")

    assert "seed_registered_plugins" not in source
    assert "seed_builtin_plugins" not in source


def test_camera_snapshot_uses_registered_snapshot_adapter():
    from fastapi import FastAPI

    from gateway.routes.camera import register_camera_routes
    from runtime.registry import register, restore, snapshot

    saved = snapshot()
    try:

        @register("camera_snapshot_adapter", "test_adapter", priority=100)
        class TestSnapshotAdapter:
            calls = 0

            @staticmethod
            def capture_jpeg():
                TestSnapshotAdapter.calls += 1
                return b"\xff\xd8\xffadapter"

        app = FastAPI()
        register_camera_routes(app, None)

        route = next(route for route in app.routes if route.path == "/api/v1/camera/snapshot")
        response = asyncio.run(route.endpoint())

        assert response.status_code == 200
        assert response.media_type == "image/jpeg"
        assert response.body == b"\xff\xd8\xffadapter"
        assert TestSnapshotAdapter.calls == 1
    finally:
        restore(saved)


def test_command_routes_register_expected_paths():
    from fastapi import FastAPI

    from gateway.routes.commands import register_command_routes

    app = FastAPI()
    register_command_routes(app, SimpleNamespace())

    routes = {getattr(route, "path", ""): route for route in app.routes}
    assert "/api/v1/navigation/plan" in routes
    assert "/api/v1/navigation/goal_candidate" in routes
    assert "/api/v1/goal" in routes
    assert "/api/v1/navigate/click" in routes
    assert "/api/v1/cmd_vel" not in routes
    assert "/api/v1/stop" in routes
    assert "/api/v1/navigation/cancel" in routes
    assert "/api/v1/instruction" in routes
    assert "/api/v1/mode" in routes
    assert "/api/v1/lease" in routes
    assert routes["/api/v1/goal"].endpoint.__module__ == "gateway.routes.commands"


def test_diagnostic_routes_export_tarball(monkeypatch):
    from fastapi import FastAPI
    from fastapi.testclient import TestClient

    from gateway.gateway_module import GatewayModule
    from gateway.routes import diagnostics
    from gateway.routes.diagnostics import register_diagnostic_routes

    temp_root = Path.cwd() / ".tmp" / "test_gateway_route_split" / uuid.uuid4().hex
    temp_root.mkdir(parents=True, exist_ok=False)
    monkeypatch.setattr(diagnostics.tempfile, "gettempdir", lambda: str(temp_root))

    class HealthyModule:
        def health(self):
            return {"ok": True}

    try:
        gateway = GatewayModule()
        gateway._all_modules = {"healthy": HealthyModule()}
        app = FastAPI()
        register_diagnostic_routes(app, gateway)

        response = TestClient(app).get("/api/v1/diagnostic_pack")
        assert response.status_code == 200
        assert response.headers["content-type"].startswith("application/gzip")

        with tarfile.open(fileobj=io.BytesIO(response.content), mode="r:gz") as tar:
            names = set(tar.getnames())
            assert "diag/modules.json" in names
            assert "diag/health.json" in names
            assert "diag/app_web_snapshots.json" in names
            assert "diag/app_web/readiness.json" in names
            assert "diag/app_web/state.json" in names
            assert "diag/app_web/bootstrap.json" in names
            assert "diag/app_web/capabilities.json" in names
            assert "diag/app_web/traffic.json" in names
            assert "diag/app_web/media.json" in names
            assert "diag/app_web/session.json" in names
            assert "diag/app_web/maps.json" in names
            assert "diag/app_web/commands.json" in names
            assert "diag/app_web/routecheck.json" in names
            assert "diag/app_web/real_runtime_evidence.json" in names
            assert "diag/app_web/frame_contract.json" in names
            assert "diag/app_web/runtime_contract.json" in names
            assert "diag/runtime_contract.json" in names
            modules_file = tar.extractfile("diag/modules.json")
            assert modules_file is not None
            modules = json.loads(modules_file.read().decode("utf-8"))
            assert modules["healthy"] == {"ok": True}
            readiness_file = tar.extractfile("diag/app_web/readiness.json")
            assert readiness_file is not None
            readiness = json.loads(readiness_file.read().decode("utf-8"))
            assert readiness["ok"] is True
            assert readiness["data"]["schema_version"] == 1
            assert readiness["data"]["status"] in {"ready", "degraded", "not_started"}
            runtime_contract_file = tar.extractfile("diag/runtime_contract.json")
            assert runtime_contract_file is not None
            runtime_contract = json.loads(runtime_contract_file.read().decode("utf-8"))
            assert runtime_contract["manifest"]["schema_version"] == ("lingtu.runtime_interface.v1")
            assert runtime_contract["manifest"]["frame_links"]["map_to_odom"] == {
                "parent": "map",
                "child": "odom",
                "required": True,
            }
            frame_contract_file = tar.extractfile("diag/app_web/frame_contract.json")
            assert frame_contract_file is not None
            frame_contract = json.loads(frame_contract_file.read().decode("utf-8"))
            assert "runtime_boundary" in frame_contract["data"]
        assert list(temp_root.glob("diag_*.tar.gz")) == []
    finally:
        shutil.rmtree(temp_root, ignore_errors=True)


def test_diagnostic_pack_tolerates_missing_module_inventory(monkeypatch):
    from fastapi import FastAPI
    from fastapi.testclient import TestClient

    from gateway.gateway_module import GatewayModule
    from gateway.routes import diagnostics
    from gateway.routes.diagnostics import register_diagnostic_routes

    temp_root = Path.cwd() / ".tmp" / "test_gateway_route_split" / uuid.uuid4().hex
    temp_root.mkdir(parents=True, exist_ok=False)
    monkeypatch.setattr(diagnostics.tempfile, "gettempdir", lambda: str(temp_root))

    try:
        gateway = GatewayModule()
        gateway.setup()
        del gateway._all_modules
        app = FastAPI()
        register_diagnostic_routes(app, gateway)

        response = TestClient(app).get("/api/v1/diagnostic_pack")
        assert response.status_code == 200
        assert response.headers["content-type"].startswith("application/gzip")

        with tarfile.open(fileobj=io.BytesIO(response.content), mode="r:gz") as tar:
            modules_file = tar.extractfile("diag/modules.json")
            assert modules_file is not None
            modules = json.loads(modules_file.read().decode("utf-8"))
            assert modules == {}
        assert list(temp_root.glob("diag_*.tar.gz")) == []
    finally:
        shutil.rmtree(temp_root, ignore_errors=True)


def test_diagnostic_app_web_snapshots_cover_client_startup_surfaces():
    from gateway.gateway_module import GatewayModule
    from gateway.routes.diagnostics import _build_app_web_snapshots

    gateway = GatewayModule()
    gateway.setup()

    snapshots = _build_app_web_snapshots(gateway)
    expected = {
        "bootstrap",
        "capabilities",
        "traffic",
        "readiness",
        "state",
        "localization",
        "navigation",
        "path",
        "scene_graph",
        "locations",
        "media",
        "session",
        "maps",
        "commands",
        "routecheck",
        "frame_contract",
        "runtime_contract",
    }

    assert expected <= set(snapshots)
    for name in expected:
        assert "ok" in snapshots[name]

    assert snapshots["bootstrap"]["ok"] is True
    assert snapshots["bootstrap"]["data"]["links"]["diagnostic_pack"] == ("/api/v1/diagnostic_pack")
    assert snapshots["capabilities"]["ok"] is True
    assert (
        snapshots["capabilities"]["data"]["endpoints"]["app"]["bootstrap"]["response_schema"] == "AppBootstrapResponse"
    )
    assert (
        snapshots["capabilities"]["data"]["endpoints"]["ops"]["runtime_contract"]["path"]
        == "/api/v1/diagnostics/runtime-contract"
    )
    validation_gates = snapshots["capabilities"]["data"]["validation_gates"]
    assert validation_gates["saved_map_artifact_gate"]["command"] == (
        "python scripts/gates/saved_map_artifact_gate.py "
        "<map-id> "
        "[--require-octomap | --require-occupancy] "
        "--json-out artifacts/saved_map_artifacts/report.json"
    )
    assert validation_gates["saved_map_artifact_gate"]["artifact"] == ("artifacts/saved_map_artifacts/report.json")
    assert validation_gates["saved_map_artifact_gate"]["acceptance_step"] == 1
    assert validation_gates["real_runtime_evidence"]["expected_runtime_contract"] == ("real")
    assert validation_gates["real_runtime_evidence"]["command"] == (
        "python scripts/gates/real_runtime_evidence_collect.py "
        "--gateway-url http://<robot>:5050 "
        "--duration-sec 20 "
        "--expected-contract real "
        "--json-out artifacts/real_runtime/report.json"
    )
    assert validation_gates["real_runtime_evidence"]["artifact"] == ("artifacts/real_runtime/report.json")
    assert validation_gates["real_runtime_evidence"]["acceptance_step"] == 2
    assert snapshots["traffic"]["ok"] is True
    assert snapshots["traffic"]["data"]["client_policy"]["events_endpoint"] == ("/api/v1/events")
    assert snapshots["media"]["ok"] is True
    assert snapshots["session"]["ok"] is True
    assert snapshots["maps"]["ok"] is True
    assert snapshots["commands"]["ok"] is True
    assert snapshots["commands"]["data"]["idempotency_supported"] is True
    assert snapshots["routecheck"]["ok"] is True
    assert snapshots["routecheck"]["data"]["schema_version"] == 1
    assert snapshots["frame_contract"]["ok"] is True
    frame_contract = snapshots["frame_contract"]["data"]
    assert frame_contract["contract"]["schema_version"] == "lingtu.runtime_interface.v1"
    assert frame_contract["expected"]["map_frame"] == "map"
    assert frame_contract["expected"]["odom_frame"] == "odom"
    assert frame_contract["expected"]["body_frame"] == "body"
    assert frame_contract["navigation_frames"]["planning_frame_id"] == "map"
    assert "runtime_boundary" in frame_contract
    assert snapshots["runtime_contract"]["ok"] is True
    runtime_contract = snapshots["runtime_contract"]["data"]["manifest"]
    assert runtime_contract["data_sources"]["field"]["command_sink"] == ("driver")
    real_flow = {stage["name"]: stage for stage in runtime_contract["resolved_runtime_data_flow"]["field"]}
    assert list(real_flow["command_boundary"]["outputs"]) == ["driver"]


def test_diagnostic_runtime_contract_route_exposes_canonical_manifest():
    from fastapi import FastAPI
    from fastapi.testclient import TestClient

    from gateway.routes.diagnostics import (
        _runtime_contract_snapshot,
        register_diagnostic_routes,
    )

    snapshot = _runtime_contract_snapshot()
    assert snapshot["schema_version"] == 1
    assert snapshot["source"] == "runtime.runtime_interface.runtime_contract_manifest"
    manifest = snapshot["manifest"]
    assert manifest["schema_version"] == "lingtu.runtime_interface.v1"
    assert manifest["frame_links"]["body_to_lidar"] == {
        "parent": "body",
        "child": "lidar_link",
        "required": True,
    }
    assert manifest["data_sources"]["mujoco_fastlio2_live"]["command_sink"] == ("mujoco_velocity_adapter")
    mujoco_flow = {stage["name"]: stage for stage in manifest["resolved_runtime_data_flow"]["mujoco_fastlio2_live"]}
    assert list(mujoco_flow["command_boundary"]["outputs"]) == ["mujoco_velocity_adapter"]

    app = FastAPI()
    register_diagnostic_routes(app, SimpleNamespace(_all_modules={}))
    response = TestClient(app).get("/api/v1/diagnostics/runtime-contract")
    assert response.status_code == 200
    body = response.json()
    assert body["manifest"]["schema_version"] == "lingtu.runtime_interface.v1"
    assert body["manifest"]["frame_links"]["map_to_odom"]["parent"] == "map"
    assert body["manifest"]["data_sources"]["field"]["mapping_source"] == ("slam_map_cloud")


def test_runtime_contract_manifest_is_fully_typed_by_gateway_schema():
    from gateway.schemas import RuntimeContractManifest
    from runtime.runtime_interface import runtime_contract_manifest

    manifest = runtime_contract_manifest()
    schema_fields = set(RuntimeContractManifest.model_fields)

    assert set(manifest) == schema_fields
    model = RuntimeContractManifest.model_validate(manifest)
    dumped = model.model_dump()
    assert set(dumped) == schema_fields
    assert dumped["message_formats"]
    assert dumped["artifact_formats"]
    assert dumped["lidar_extrinsics"]


def test_diagnostic_frame_contract_reports_navigation_mismatches(monkeypatch):
    from gateway.gateway_module import GatewayModule
    from gateway.routes.diagnostics import _frame_contract_snapshot

    monkeypatch.setenv("LINGTU_PROFILE", "nav")
    monkeypatch.setenv("LINGTU_DATA_SOURCE", "field")
    monkeypatch.setenv("LINGTU_RUNTIME_CONTRACT", "real")
    monkeypatch.setenv("LINGTU_COMMAND_SINK", "driver")
    monkeypatch.setenv("LINGTU_SIMULATION_ONLY", "0")

    gateway = GatewayModule()
    gateway.setup()
    with gateway._state_lock:
        gateway._odom = {"x": 0.0, "y": 0.0, "frame_id": "camera_link"}
        gateway._mission = {
            "state": "EXECUTING",
            "planning_frame_id": "map",
            "goal_frame_id": "map",
        }

    snapshot = _frame_contract_snapshot(gateway)

    assert snapshot["ok"] is False
    assert snapshot["expected"]["map_frame"] == "map"
    assert {
        "source": "odometry",
        "expected_frame": "map",
        "received_frame": "camera_link",
    } in snapshot["mismatches"]
    runtime = snapshot["runtime_boundary"]
    assert runtime["data_source"] == "field"
    assert runtime["runtime_contract"] == "real"
    assert runtime["frame_links"]["body_to_lidar"] == {
        "parent": "body",
        "child": "lidar_link",
        "required": True,
    }
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
    assert runtime["runtime_data_flow_topics"][:2] == [
        "/lidar/raw_frame",
        "/imu/raw",
    ]
    flow = {stage["name"]: stage for stage in runtime["resolved_runtime_data_flow"]}
    assert flow["endpoint_adapter"]["inputs"] == ["/lidar/raw_frame", "/imu/raw"]
    assert flow["command_boundary"]["outputs"] == ["driver"]


def test_frame_contract_snapshot_defaults_to_runtime_contract(monkeypatch):
    import runtime.runtime_interface as runtime_interface
    from gateway.gateway_module import GatewayModule
    from gateway.routes import diagnostics

    fake_links = {
        "contract_map_to_odom": {
            "parent": "contract_map",
            "child": "contract_odom",
            "required": True,
        }
    }
    monkeypatch.setattr(
        runtime_interface,
        "runtime_contract_manifest",
        lambda: {
            "frames": {
                "map": "contract_map",
                "odom": "contract_odom",
                "body": "contract_body",
            },
            "frame_links": fake_links,
        },
    )

    snapshot = diagnostics._frame_contract_snapshot(GatewayModule())

    assert snapshot["expected"]["map_frame"] == "contract_map"
    assert snapshot["expected"]["odom_frame"] == "contract_odom"
    assert snapshot["expected"]["body_frame"] == "contract_body"
    assert snapshot["expected"]["links"] == fake_links


def test_gateway_module_builds_split_routes_once():
    from gateway.gateway_module import GatewayModule

    gateway = GatewayModule()
    gateway.setup()

    counts = Counter(getattr(route, "path", "") for route in gateway._app.routes)
    assert counts["/api/v1/app/bootstrap"] == 1
    assert counts["/api/v1/bootstrap"] == 0
    assert counts["/api/v1/app/capabilities"] == 1
    assert counts["/api/v1/auth/login"] == 1
    assert counts["/api/v1/auth/check"] == 1
    assert counts["/api/v1/diagnostic_pack"] == 1
    assert counts["/api/v1/diagnostics/routecheck/latest"] == 1
    assert counts["/api/v1/diagnostics/real-runtime-evidence/latest"] == 1
    assert counts["/api/v1/diagnostics/algorithm-benchmark/latest"] == 0
    assert counts["/api/v1/diagnostics/runtime-contract"] == 1
    assert counts["/api/v1/events"] == 1
    assert counts["/api/v1/state"] == 1
    assert counts["/api/v1/locations"] == 2
    assert counts["/api/v1/locations/{name}"] == 2
    assert counts["/api/v1/localization/status"] == 1
    assert counts["/api/v1/navigation/status"] == 1
    assert counts["/api/v1/runtime/dataflow"] == 1
    assert counts["/api/v1/slam/restart"] == 0
    assert counts["/api/v1/navigation"] == 0
    assert counts["/api/v1/status"] == 0
    assert counts["/api/v1/health"] == 1
    assert counts["/health"] == 1
    assert counts["/ready"] == 1
    assert counts["/api/v1/session"] == 1
    assert counts["/api/v1/session/start"] == 0
    assert counts["/api/v1/session/end"] == 0
    assert counts["/api/v1/slam/maps"] == 1
    assert counts["/api/v1/maps"] == 0
    assert counts["/api/v1/maps/{name}"] == 1
    assert counts["/api/v1/maps/{name}/build_occupancy"] == 1
    assert counts["/api/v1/map/points"] == 1
    assert counts["/api/v1/map_cloud/reset"] == 1
    assert counts["/api/v1/navigation/plan"] == 1
    assert counts["/api/v1/navigation/goal_candidate"] == 1
    assert counts["/api/v1/goal"] == 1
    assert counts["/api/v1/navigate/click"] == 1
    assert counts["/api/v1/cmd_vel"] == 0
    assert counts["/api/v1/stop"] == 1
    assert counts["/api/v1/navigation/cancel"] == 1
    assert counts["/api/v1/instruction"] == 1
    assert counts["/api/v1/mode"] == 1
    assert counts["/api/v1/lease"] == 1
    assert counts["/ws/teleop"] == 1
    assert counts["/ws/camera"] == 1
    assert counts["/ws/cloud"] == 1
    assert counts["/ws/scan"] == 1


def test_gateway_module_keeps_client_route_inventory():
    from gateway.gateway_module import GatewayModule

    gateway = GatewayModule()
    gateway.setup()

    paths = {getattr(route, "path", "") for route in gateway._app.routes}
    expected = {
        "/api/v1/app/bootstrap",
        "/api/v1/app/capabilities",
        "/api/v1/auth/login",
        "/api/v1/auth/check",
        "/api/v1/state",
        "/api/v1/scene_graph",
        "/api/v1/locations",
        "/api/v1/locations/{name}",
        "/api/v1/path",
        "/api/v1/localization/status",
        "/api/v1/navigation/status",
        "/api/v1/services/status",
        "/api/v1/runtime/dataflow",
        "/api/v1/health",
        "/health",
        "/ready",
        "/api/v1/session",
        "/api/v1/diagnostic_pack",
        "/api/v1/diagnostics/routecheck/latest",
        "/api/v1/diagnostics/real-runtime-evidence/latest",
        "/api/v1/diagnostics/runtime-contract",
        "/api/v1/events",
        "/api/v1/slam/maps",
        "/api/v1/map/points",
        "/api/v1/maps/{name}/points",
        "/api/v1/map_cloud/reset",
        "/api/v1/navigation/plan",
        "/api/v1/navigation/goal_candidate",
        "/api/v1/goal",
        "/api/v1/navigate/click",
        "/api/v1/stop",
        "/api/v1/navigation/cancel",
        "/api/v1/instruction",
        "/api/v1/mode",
        "/api/v1/lease",
        "/api/v1/camera/snapshot",
        "/ws/teleop",
        "/ws/camera",
        "/ws/cloud",
        "/ws/scan",
    }
    assert expected <= paths
    assert "/api/v1/status" not in paths


def test_capabilities_manifest_paths_exist_in_gateway_routes():
    from gateway.gateway_module import GatewayModule

    gateway = GatewayModule()
    gateway.setup()

    route_paths = {getattr(route, "path", "") for route in gateway._app.routes}
    route = next(route for route in gateway._app.routes if route.path == "/api/v1/app/capabilities")
    capabilities = asyncio.run(route.endpoint())

    manifest_paths = set()
    for group in capabilities["endpoints"].values():
        for spec in group.values():
            manifest_paths.add(spec["path"])
    for spec in capabilities["probes"].values():
        manifest_paths.add(spec["path"])

    assert manifest_paths <= route_paths
    assert capabilities["endpoints"]["realtime"]["camera"]["path"] == "/ws/camera"
    assert capabilities["endpoints"]["realtime"]["camera"]["method"] == "WS"


def test_routecheck_latest_diagnostic_reads_latest_summary(tmp_path, monkeypatch):
    from fastapi import FastAPI
    from fastapi.testclient import TestClient

    from gateway.routes.diagnostics import (
        build_routecheck_latest_summary,
        register_diagnostic_routes,
    )

    old_dir = tmp_path / "route_preflight_old"
    new_dir = tmp_path / "route_preflight_new"
    old_dir.mkdir()
    new_dir.mkdir()
    (old_dir / "summary.json").write_text(
        json.dumps(
            {
                "schema_version": 1,
                "mode": "routecheck_non_motion",
                "outcome": "fail",
                "non_motion": True,
            }
        ),
        encoding="utf-8",
    )
    new_summary = {
        "schema_version": 1,
        "mode": "routecheck_non_motion",
        "outcome": "pass",
        "non_motion": True,
        "simulation_only": True,
        "real_robot_motion": False,
        "cmd_vel_sent_to_hardware": False,
        "gateway_used": True,
        "driver_used": False,
        "published": {"goal_pose": 0, "cmd_vel": 0, "stop_cmd": 0},
        "phases": {"baseline": {"selected_planner": "octoplanner3d"}},
    }
    (new_dir / "summary.json").write_text(
        json.dumps(new_summary),
        encoding="utf-8",
    )
    os.utime(old_dir / "summary.json", (1, 1))
    os.utime(new_dir / "summary.json", (2, 2))

    payload = build_routecheck_latest_summary(tmp_path)
    assert payload["ok"] is True
    assert payload["count"] == 2
    assert payload["artifact_dir"] == str(new_dir)
    assert payload["report_mtime"] == 2
    assert payload["report_age_s"] >= 0.0
    assert payload["non_motion"] is True
    assert payload["simulation_only"] is True
    assert payload["real_robot_motion"] is False
    assert payload["cmd_vel_sent_to_hardware"] is False
    assert payload["gateway_used"] is True
    assert payload["driver_used"] is False
    assert payload["published"] == {"goal_pose": 0, "cmd_vel": 0, "stop_cmd": 0}
    assert payload["latest"]["outcome"] == "pass"
    assert payload["latest"]["phases"]["baseline"]["selected_planner"] == "octoplanner3d"

    monkeypatch.setenv("LINGTU_ROUTECHECK_ARTIFACT_ROOT", str(tmp_path))
    app = FastAPI()
    register_diagnostic_routes(app, SimpleNamespace(_all_modules={}))
    response = TestClient(app).get("/api/v1/diagnostics/routecheck/latest")
    assert response.status_code == 200
    body = response.json()
    assert body["schema_version"] == 1
    assert body["ok"] is True


def test_routecheck_latest_diagnostic_falls_back_to_project_artifacts(
    tmp_path,
    monkeypatch,
):
    from gateway.routes import diagnostics

    home = tmp_path / "home"
    project = tmp_path / "project"
    routecheck_dir = project / "artifacts" / "sim_diagnostics" / "routecheck"
    routecheck_dir.mkdir(parents=True)
    summary = {
        "schema_version": 1,
        "mode": "routecheck_non_motion",
        "outcome": "pass",
        "non_motion": True,
        "simulation_only": True,
        "real_robot_motion": False,
        "cmd_vel_sent_to_hardware": False,
        "gateway_used": True,
        "driver_used": False,
        "published": {"goal_pose": 0, "cmd_vel": 0, "stop_cmd": 0},
    }
    (routecheck_dir / "summary.json").write_text(
        json.dumps(summary),
        encoding="utf-8",
    )
    monkeypatch.delenv("LINGTU_ROUTECHECK_ARTIFACT_ROOT", raising=False)
    monkeypatch.setattr(diagnostics.pathlib.Path, "home", lambda: home)
    monkeypatch.setattr(diagnostics, "PROJECT_ROOT", project, raising=False)

    payload = diagnostics.build_routecheck_latest_summary()

    assert payload["ok"] is True
    assert payload["count"] == 1
    assert payload["artifacts_root"] == str(project / "artifacts")
    assert payload["searched_roots"] == [
        str(home / "data" / "SLAM" / "navigation" / "artifacts"),
        str(project / "artifacts"),
    ]
    assert payload["artifact_dir"] == str(routecheck_dir)
    assert payload["latest"]["outcome"] == "pass"


def _write_real_runtime_evidence_report(
    run_dir: Path,
    *,
    runtime_evidence: dict | None = None,
) -> Path:
    run_dir.mkdir(parents=True, exist_ok=True)
    validation = {
        "schema_version": "lingtu.real_runtime_evidence.validation.v1",
        "ok": True,
        "expected_contract": "real",
        "blockers": [],
    }
    if runtime_evidence is not None:
        validation = runtime_evidence
    report = {
        "schema_version": "lingtu.real_runtime_evidence.report.v1",
        "simulation_only": False,
        "real_robot_motion": True,
        "cmd_vel_sent_to_hardware": True,
        "runtime_contract": {"name": "real", "ok": True},
        "runtime_evidence": validation,
    }
    report_path = run_dir / "report.json"
    report_path.write_text(json.dumps(report), encoding="utf-8")
    return report_path


def test_real_runtime_evidence_latest_diagnostic_reads_gate_artifact(tmp_path, monkeypatch):
    from fastapi import FastAPI
    from fastapi.testclient import TestClient

    from gateway.routes.diagnostics import (
        build_real_runtime_evidence_latest_summary,
        register_diagnostic_routes,
    )

    run_dir = tmp_path / "real_runtime"
    report_path = _write_real_runtime_evidence_report(run_dir)
    os.utime(report_path, (100, 100))

    payload = build_real_runtime_evidence_latest_summary(
        tmp_path,
        max_age_s=1000.0,
        now=200.0,
    )
    assert payload["ok"] is True
    assert payload["report_path"] == str(run_dir / "report.json")
    assert payload["report_age_s"] == 100.0
    assert payload["max_age_s"] == 1000.0
    assert payload["runtime_contract"] == "real"
    assert payload["runtime_evidence_ok"] is True
    assert payload["simulation_only"] is False
    assert payload["real_robot_motion"] is True
    assert payload["cmd_vel_sent_to_hardware"] is True

    monkeypatch.setenv("LINGTU_REAL_RUNTIME_EVIDENCE_ROOT", str(tmp_path))
    monkeypatch.setenv("LINGTU_REAL_RUNTIME_EVIDENCE_MAX_AGE_SEC", "1000")
    monkeypatch.setattr("gateway.routes.diagnostics.time.time", lambda: 200.0)
    app = FastAPI()
    register_diagnostic_routes(app, SimpleNamespace(_all_modules={}))
    response = TestClient(app).get("/api/v1/diagnostics/real-runtime-evidence/latest")
    assert response.status_code == 200
    body = response.json()
    assert body["schema_version"] == 1
    assert body["ok"] is True
    assert body["runtime_evidence_ok"] is True


def test_real_runtime_evidence_latest_uses_newest_report_even_when_failing(tmp_path):
    from gateway.routes.diagnostics import build_real_runtime_evidence_latest_summary

    root = tmp_path / "real_runtime"
    old_report = _write_real_runtime_evidence_report(root / "old_pass")
    new_report = _write_real_runtime_evidence_report(
        root / "new_fail",
        runtime_evidence={
            "schema_version": "lingtu.real_runtime_evidence.validation.v1",
            "ok": False,
            "expected_contract": "real",
            "blockers": ["newer real motion evidence failed"],
        },
    )
    os.utime(old_report, (100, 100))
    os.utime(new_report, (150, 150))

    payload = build_real_runtime_evidence_latest_summary(
        root,
        max_age_s=1000.0,
        now=200.0,
    )

    assert payload["count"] == 2
    assert payload["report_path"] == str(new_report)
    assert payload["ok"] is False
    assert payload["runtime_evidence_ok"] is False
    assert payload["reason"] == "real-runtime-evidence gate did not pass"
    assert "newer real motion evidence failed" in payload["blockers"]


def test_real_runtime_evidence_latest_diagnostic_reports_missing_artifact(tmp_path):
    from gateway.routes.diagnostics import build_real_runtime_evidence_latest_summary

    payload = build_real_runtime_evidence_latest_summary(tmp_path, now=200.0)

    assert payload["ok"] is False
    assert payload["count"] == 0
    assert payload["reason"] == "real_runtime_evidence_report_not_found"
    assert payload["blockers"] == ["real_runtime_evidence_report_not_found"]


def test_real_runtime_evidence_latest_diagnostic_rejects_stale_artifact(tmp_path):
    from gateway.routes.diagnostics import build_real_runtime_evidence_latest_summary

    report_path = _write_real_runtime_evidence_report(tmp_path / "real_runtime")
    os.utime(report_path, (100, 100))

    payload = build_real_runtime_evidence_latest_summary(
        tmp_path,
        max_age_s=50.0,
        now=200.0,
    )

    assert payload["ok"] is False
    assert payload["reason"] == "real-runtime-evidence is stale"
    assert "real-runtime-evidence is stale" in payload["blockers"]


def test_real_runtime_evidence_latest_trusts_validator_verdict_without_mirror_sections(
    tmp_path,
):
    from gateway.routes.diagnostics import build_real_runtime_evidence_latest_summary

    validation = {
        "schema_version": "lingtu.real_runtime_evidence.validation.v1",
        "ok": True,
        "expected_contract": "real",
        "blockers": [],
    }
    report_path = _write_real_runtime_evidence_report(
        tmp_path / "real_runtime",
        runtime_evidence=validation,
    )
    os.utime(report_path, (100, 100))

    payload = build_real_runtime_evidence_latest_summary(
        tmp_path,
        max_age_s=1000.0,
        now=200.0,
    )

    assert payload["ok"] is True
    assert payload["blockers"] == []


def _schema_ref_for(
    openapi: dict,
    path: str,
    status: str = "200",
    method: str = "get",
) -> str:
    return openapi["paths"][path][method]["responses"][status]["content"]["application/json"]["schema"]["$ref"]


def _content_for(
    openapi: dict,
    path: str,
    method: str = "get",
    status: str = "200",
) -> dict:
    return openapi["paths"][path][method]["responses"][status]["content"]


def _request_schema_ref_for(
    openapi: dict,
    path: str,
    method: str = "post",
) -> str:
    schema = openapi["paths"][path][method]["requestBody"]["content"][
        "application/json"
    ]["schema"]
    if "$ref" in schema:
        return schema["$ref"]
    return next(item["$ref"] for item in schema.get("anyOf", []) if "$ref" in item)


def test_openapi_exposes_client_response_models():
    from gateway.gateway_module import GatewayModule

    gateway = GatewayModule()
    gateway.setup()

    openapi = gateway._app.openapi()
    schemas = openapi["components"]["schemas"]

    assert "StateResponse" in schemas
    assert "ReadinessResponse" in schemas
    assert "ReadinessRuntimeSummary" in schemas
    assert "ReadinessLocalizationRuntime" in schemas
    assert "ReadinessLocalizationFrameSummary" in schemas
    assert "ReadinessRuntimeBoundary" in schemas
    assert "RuntimeDataFlowStageSummary" in schemas
    assert "RuntimeFrameSummary" in schemas
    assert "RuntimeFrameLinkSummary" in schemas
    assert "AppBootstrapResponse" in schemas
    assert "AppCapabilitiesResponse" in schemas
    assert "ValidationGateCapability" in schemas
    validation_gate_properties = schemas["ValidationGateCapability"]["properties"]
    assert set(validation_gate_properties) == {
        "schema_version",
        "scope",
        "acceptance_step",
        "required_when",
        "command",
        "artifact",
        "expected_runtime_contract",
    }
    assert "AppTrafficResponse" in schemas
    assert "AppMediaLinks" in schemas
    assert "CameraMediaStatus" in schemas
    assert "CameraPortStatus" in schemas
    assert "CameraInfoStatus" in schemas
    assert "CameraJpegStatus" in schemas
    assert "WHEPMediaStatus" in schemas
    assert "TrafficSSEStats" in schemas
    assert "TrafficCloudStats" in schemas
    assert "HealthResponse" in schemas
    assert "LivenessResponse" in schemas
    assert "ControlCommandResponse" in schemas
    assert "GatewayErrorResponse" in schemas
    assert "RoutecheckLatestResponse" in schemas
    assert "RuntimeContractResponse" in schemas
    assert "RuntimeContractManifest" in schemas
    assert "RuntimeDataflowResponse" in schemas
    assert "RuntimeDataflowTopicSummary" in schemas
    assert "RuntimeDataflowObservability" in schemas
    assert "RuntimeDataflowCommunication" in schemas
    assert "RuntimeDataflowPortSummary" in schemas
    assert "RuntimeDataSourceSummary" in schemas
    assert "RuntimeAlgorithmInterfaceSummary" in schemas
    assert "RuntimeAdapterAliasSummary" in schemas
    assert "RuntimeProductDataSourceBinding" in schemas
    assert "RuntimeTransform3D" in schemas
    assert "RuntimeMessageFormatSummary" in schemas
    assert "RuntimeArtifactFormatSummary" in schemas
    assert "CommandReceipt" in schemas
    assert "CancelRequest" in schemas
    assert "GoalCandidateRequest" in schemas
    assert "GoalCandidateResponse" in schemas
    assert "ConstructedGoalTarget" in schemas
    assert "PlanPreviewRequest" in schemas
    assert "PlanPreviewResponse" in schemas
    assert "SceneGraphResponse" in schemas
    assert "SceneGraphObject" in schemas
    assert "SceneGraphRelation" in schemas
    assert "SceneGraphRegion" in schemas
    assert "LocationsResponse" in schemas
    assert "LocationEntry" in schemas
    assert "LocationUpsertRequest" in schemas
    assert "LocationOperationResponse" in schemas
    assert "PathResponse" in schemas
    assert "PathPoint" in schemas
    assert "RobotPoseSummary" in schemas
    assert "LocalizationStatusResponse" in schemas
    assert "NavigationStatusResponse" in schemas
    assert "NavigationControlSummary" in schemas
    assert "NavigationReadinessSummary" in schemas
    assert "NavigationProgressSummary" in schemas
    assert "NavigationTargetSummary" in schemas
    assert "NavigationSpeedPolicy" in schemas
    assert "NavigationMotionSummary" in schemas
    assert "NavigationFeedbackSummary" in schemas
    assert "NavigationDiagnosticsSummary" in schemas
    assert "AuthLoginRequest" in schemas
    assert "AuthLoginResponse" in schemas
    assert "AuthCheckResponse" in schemas
    assert "LeaseResponse" in schemas
    assert "SessionResponse" in schemas
    assert "product_session_id" in schemas["SessionResponse"]["properties"]
    assert "SessionStartRequest" not in schemas
    assert "SessionTransitionResponse" not in schemas
    assert "MapRenameRequest" in schemas
    assert "MapSaveRequest" in schemas
    assert "MapSaveOperationResponse" in schemas
    assert "operation_id" in schemas["MapSaveOperationResponse"]["properties"]
    assert "operation" in schemas["MapSaveOperationResponse"]["properties"]
    assert "job_id" not in schemas["MapSaveOperationResponse"]["properties"]
    assert "MapLifecycleResponse" in schemas
    assert "schema_version" in schemas["MapLifecycleResponse"]["properties"]
    assert "ok" in schemas["MapLifecycleResponse"]["properties"]
    assert "ts" in schemas["MapLifecycleResponse"]["properties"]
    assert "warnings" in schemas["MapLifecycleResponse"]["properties"]
    assert "MapListResponse" in schemas
    assert "schema_version" in schemas["MapListResponse"]["properties"]
    assert "count" in schemas["MapListResponse"]["properties"]
    assert "ts" in schemas["MapListResponse"]["properties"]
    assert "MapPointsResponse" in schemas
    assert "schema_version" in schemas["MapPointsResponse"]["properties"]
    assert "frame_id" in schemas["MapPointsResponse"]["properties"]
    assert "epoch" in schemas["MapPointsResponse"]["properties"]
    assert "sequence" in schemas["MapPointsResponse"]["properties"]
    assert "stream_kind" in schemas["MapPointsResponse"]["properties"]
    assert "source" in schemas["MapPointsResponse"]["properties"]
    assert "ts" in schemas["MapPointsResponse"]["properties"]
    assert "TemporalMemoryResponse" in schemas
    assert "ExplorationCommandResponse" in schemas
    assert "ExplorationStatusResponse" in schemas
    assert "SlamStatusResponse" in schemas
    assert "LocalizationInitialPose" in schemas
    assert "LocalizationRelocalizationRequest" in schemas
    assert "LocalizationMapTrackingRequest" in schemas
    assert "LocalizationOperationResponse" in schemas
    assert "mode" in schemas["LocalizationRelocalizationRequest"]["properties"]
    assert "initial_pose" in schemas["LocalizationRelocalizationRequest"]["properties"]
    assert "RecordingStartRequest" in schemas
    assert "RecordingOperationResponse" in schemas
    assert "RecordingStatusResponse" in schemas
    assert "Go2RTCStatusResponse" in schemas
    assert "TemporalSemanticRequest" in schemas
    assert schemas["AppMediaLinks"]["properties"]["camera"]["$ref"].endswith("/CameraMediaStatus")
    assert schemas["AppMediaLinks"]["properties"]["whep"]["$ref"].endswith("/WHEPMediaStatus")
    assert set(schemas["CameraMediaStatus"]["properties"]["status"]["enum"]) == {
        "streaming",
        "idle",
        "stale",
        "error",
        "not_loaded",
    }
    assert _schema_ref_for(openapi, "/api/v1/state").endswith("/StateResponse")
    assert _schema_ref_for(openapi, "/ready").endswith("/ReadinessResponse")
    assert _schema_ref_for(openapi, "/ready", "503").endswith("/ReadinessResponse")
    assert _schema_ref_for(openapi, "/api/v1/health").endswith("/HealthResponse")
    assert _schema_ref_for(openapi, "/health").endswith("/LivenessResponse")
    assert _schema_ref_for(openapi, "/api/v1/scene_graph").endswith("/SceneGraphResponse")
    assert _schema_ref_for(openapi, "/api/v1/locations").endswith("/LocationsResponse")
    assert _schema_ref_for(openapi, "/api/v1/locations", method="post").endswith("/LocationOperationResponse")
    assert _request_schema_ref_for(openapi, "/api/v1/locations").endswith("/LocationUpsertRequest")
    assert _schema_ref_for(openapi, "/api/v1/locations/{name}", method="put").endswith("/LocationOperationResponse")
    assert _request_schema_ref_for(openapi, "/api/v1/locations/{name}", method="put").endswith("/LocationUpsertRequest")
    assert _schema_ref_for(openapi, "/api/v1/locations/{name}", method="delete").endswith("/LocationOperationResponse")
    assert _schema_ref_for(openapi, "/api/v1/path").endswith("/PathResponse")
    assert _schema_ref_for(openapi, "/api/v1/diagnostics/routecheck/latest").endswith("/RoutecheckLatestResponse")
    assert _schema_ref_for(openapi, "/api/v1/diagnostics/runtime-contract").endswith("/RuntimeContractResponse")
    assert _schema_ref_for(openapi, "/api/v1/runtime/dataflow").endswith("/RuntimeDataflowResponse")
    assert _schema_ref_for(openapi, "/api/v1/runtime/dataflow/subscribe", method="post").endswith(
        "/RuntimeDataflowSubscribeResponse"
    )
    assert _request_schema_ref_for(openapi, "/api/v1/runtime/dataflow/subscribe").endswith(
        "/RuntimeDataflowSubscribeRequest"
    )
    assert schemas["RuntimeContractResponse"]["properties"]["manifest"]["$ref"].endswith("/RuntimeContractManifest")
    assert schemas["RuntimeDataflowResponse"]["properties"]["topics"]["items"]["$ref"].endswith(
        "/RuntimeDataflowTopicSummary"
    )
    assert "runtime_dataflow_subscribe" in schemas["ClientLinks"]["properties"]
    assert "AlgorithmBenchmarkLatestResponse" not in schemas
    assert schemas["RuntimeDataflowTopicSummary"]["properties"]["observability"]["$ref"].endswith(
        "/RuntimeDataflowObservability"
    )
    assert schemas["RuntimeDataflowTopicSummary"]["properties"]["communication"]["$ref"].endswith(
        "/RuntimeDataflowCommunication"
    )
    contract_manifest = schemas["RuntimeContractManifest"]["properties"]
    assert "topic_ros_types" not in contract_manifest
    assert "ros2_topic_required" not in schemas["RuntimeDataflowResponse"]["properties"]
    assert "ros2_topic_required" not in schemas["RuntimeDataflowObservability"]["properties"]
    assert "ros2_topic_required" not in schemas["RuntimeDataflowSubscribeResponse"]["properties"]
    assert contract_manifest["frames"]["$ref"].endswith("/RuntimeFrameSummary")
    assert contract_manifest["frame_links"]["additionalProperties"]["$ref"].endswith("/RuntimeFrameLinkSummary")
    assert contract_manifest["runtime_data_flow"]["items"]["$ref"].endswith("/RuntimeDataFlowStageSummary")
    assert contract_manifest["resolved_runtime_data_flow"]["additionalProperties"]["items"]["$ref"].endswith(
        "/RuntimeDataFlowStageSummary"
    )
    assert contract_manifest["data_sources"]["additionalProperties"]["$ref"].endswith("/RuntimeDataSourceSummary")
    assert contract_manifest["algorithm_interfaces"]["additionalProperties"]["$ref"].endswith(
        "/RuntimeAlgorithmInterfaceSummary"
    )
    assert contract_manifest["adapter_aliases"]["additionalProperties"]["items"]["$ref"].endswith(
        "/RuntimeAdapterAliasSummary"
    )
    assert contract_manifest["adapter_relays"]["additionalProperties"]["items"]["$ref"].endswith(
        "/RuntimeAdapterAliasSummary"
    )
    assert contract_manifest["product_data_sources"]["additionalProperties"]["$ref"].endswith(
        "/RuntimeProductDataSourceBinding"
    )
    assert contract_manifest["lidar_extrinsics"]["additionalProperties"]["$ref"].endswith("/RuntimeTransform3D")
    assert contract_manifest["message_formats"]["additionalProperties"]["$ref"].endswith("/RuntimeMessageFormatSummary")
    assert contract_manifest["artifact_formats"]["additionalProperties"]["$ref"].endswith(
        "/RuntimeArtifactFormatSummary"
    )
    assert contract_manifest["topic_formats"]["additionalProperties"]["items"]["type"] == "string"
    data_source_schema = schemas["RuntimeDataSourceSummary"]["properties"]
    assert {
        "name",
        "provider",
        "owns",
        "normalized_outputs",
        "command_sink",
        "algorithm_entry_outputs",
        "slam_source",
        "localization_source",
        "mapping_source",
    } <= set(data_source_schema)
    algorithm_schema = schemas["RuntimeAlgorithmInterfaceSummary"]["properties"]
    assert {"name", "inputs", "outputs", "owner", "map_dependency"} <= set(algorithm_schema)
    adapter_schema = schemas["RuntimeAdapterAliasSummary"]["properties"]
    assert {"source", "target", "msg_format", "scope", "note"} <= set(adapter_schema)
    product_binding_schema = schemas["RuntimeProductDataSourceBinding"]["properties"]
    assert {"product", "data_source", "mode", "note"} <= set(product_binding_schema)
    transform_schema = schemas["RuntimeTransform3D"]["properties"]
    assert {"parent", "child", "x", "y", "z", "roll", "pitch", "yaw"} <= set(transform_schema)
    message_format_schema = schemas["RuntimeMessageFormatSummary"]["properties"]
    assert {"name", "frame_role", "required_fields", "note"} <= set(message_format_schema)
    artifact_format_schema = schemas["RuntimeArtifactFormatSummary"]["properties"]
    assert {
        "name",
        "path",
        "artifact_type",
        "frame_role",
        "required_fields",
        "required_metadata",
        "note",
    } <= set(artifact_format_schema)
    assert _schema_ref_for(openapi, "/api/v1/localization/status").endswith("/LocalizationStatusResponse")
    assert _schema_ref_for(openapi, "/api/v1/navigation/status").endswith("/NavigationStatusResponse")
    assert _schema_ref_for(openapi, "/api/v1/goal", method="post").endswith("/ControlCommandResponse")
    assert _schema_ref_for(openapi, "/api/v1/navigation/goal_candidate", method="post").endswith(
        "/GoalCandidateResponse"
    )
    assert _request_schema_ref_for(openapi, "/api/v1/navigation/goal_candidate").endswith("/GoalCandidateRequest")
    assert _schema_ref_for(openapi, "/api/v1/navigation/plan", method="post").endswith("/PlanPreviewResponse")
    assert _request_schema_ref_for(openapi, "/api/v1/navigation/plan").endswith("/PlanPreviewRequest")
    assert "/api/v1/cmd_vel" not in openapi["paths"]
    assert _schema_ref_for(openapi, "/api/v1/stop", method="post").endswith("/ControlCommandResponse")
    assert _schema_ref_for(openapi, "/api/v1/navigation/cancel", method="post").endswith("/ControlCommandResponse")
    assert _request_schema_ref_for(openapi, "/api/v1/navigation/cancel").endswith("/CancelRequest")
    assert _schema_ref_for(openapi, "/api/v1/instruction", method="post").endswith("/ControlCommandResponse")
    assert _schema_ref_for(openapi, "/api/v1/mode", method="post").endswith("/ControlCommandResponse")
    for path in (
        "/api/v1/goal",
        "/api/v1/navigate/click",
        "/api/v1/stop",
        "/api/v1/navigation/cancel",
        "/api/v1/instruction",
        "/api/v1/mode",
        "/api/v1/lease",
    ):
        assert _schema_ref_for(openapi, path, status="409", method="post").endswith("/GatewayErrorResponse")
    assert _schema_ref_for(openapi, "/api/v1/lease", status="403", method="post").endswith("/GatewayErrorResponse")
    assert _schema_ref_for(openapi, "/api/v1/auth/login", method="post").endswith("/AuthLoginResponse")
    assert _request_schema_ref_for(openapi, "/api/v1/auth/login").endswith("/AuthLoginRequest")
    assert _schema_ref_for(openapi, "/api/v1/auth/check").endswith("/AuthCheckResponse")
    assert _schema_ref_for(openapi, "/api/v1/lease", method="post").endswith("/LeaseResponse")
    assert _schema_ref_for(openapi, "/api/v1/session").endswith("/SessionResponse")
    assert "/api/v1/session/start" not in openapi["paths"]
    assert "/api/v1/session/end" not in openapi["paths"]
    assert _schema_ref_for(openapi, "/api/v1/slam/maps").endswith("/MapListResponse")
    assert _schema_ref_for(openapi, "/api/v1/maps/{name}", method="delete").endswith("/MapLifecycleResponse")
    assert _schema_ref_for(openapi, "/api/v1/maps/{name}/build_occupancy", method="post").endswith(
        "/MapLifecycleResponse"
    )
    assert "/api/v1/maps" not in openapi["paths"]
    assert _schema_ref_for(openapi, "/api/v1/map/points").endswith("/MapPointsResponse")
    assert _schema_ref_for(openapi, "/api/v1/maps/{name}/points").endswith("/MapPointsResponse")
    assert _schema_ref_for(openapi, "/api/v1/memory/temporal").endswith("/TemporalMemoryResponse")
    assert _schema_ref_for(openapi, "/api/v1/memory/temporal/semantic", method="post").endswith(
        "/TemporalMemoryResponse"
    )
    assert _request_schema_ref_for(openapi, "/api/v1/memory/temporal/semantic").endswith("/TemporalSemanticRequest")
    assert _schema_ref_for(openapi, "/api/v1/explore/start", method="post").endswith("/ExplorationCommandResponse")
    assert _request_schema_ref_for(openapi, "/api/v1/explore/start").endswith(
        "/ExplorationStartRequest"
    )
    assert _schema_ref_for(
        openapi,
        "/api/v1/explore/runs/{exploration_run_id}",
    ).endswith("/ExplorationRunResponse")
    assert _schema_ref_for(openapi, "/api/v1/explore/runs").endswith(
        "/ExplorationRunListResponse"
    )
    for action in ("pause", "resume", "finish"):
        path = f"/api/v1/explore/runs/{{exploration_run_id}}/{action}"
        assert _schema_ref_for(openapi, path, method="post").endswith(
            "/ExplorationRunResponse"
        )
        assert _request_schema_ref_for(openapi, path).endswith(
            "/ExplorationRunCommandRequest"
        )
    assert _schema_ref_for(openapi, "/api/v1/explore/stop", method="post").endswith("/ExplorationCommandResponse")
    assert _schema_ref_for(openapi, "/api/v1/explore/status").endswith("/ExplorationStatusResponse")
    assert _schema_ref_for(openapi, "/api/v1/slam/status").endswith("/SlamStatusResponse")
    assert "/api/v1/slam/switch" not in openapi["paths"]
    assert _schema_ref_for(openapi, "/api/v1/localization/relocalizations", method="post").endswith(
        "/LocalizationOperationResponse"
    )
    assert _request_schema_ref_for(openapi, "/api/v1/localization/relocalizations").endswith(
        "/LocalizationRelocalizationRequest"
    )
    assert _schema_ref_for(openapi, "/api/v1/localization/map-tracking", method="post").endswith(
        "/LocalizationOperationResponse"
    )
    assert _request_schema_ref_for(openapi, "/api/v1/localization/map-tracking").endswith(
        "/LocalizationMapTrackingRequest"
    )
    assert "/api/v1/slam/auto_relocalize" not in openapi["paths"]
    assert "/api/v1/slam/relocalize" not in openapi["paths"]
    assert "/api/v1/slam/track_against_map" not in openapi["paths"]
    assert _schema_ref_for(openapi, "/api/v1/recordings/start", method="post").endswith(
        "/RecordingOperationResponse"
    )
    assert _request_schema_ref_for(openapi, "/api/v1/recordings/start").endswith(
        "/RecordingStartRequest"
    )
    assert _schema_ref_for(openapi, "/api/v1/recordings/stop", method="post").endswith(
        "/RecordingOperationResponse"
    )
    assert _schema_ref_for(openapi, "/api/v1/recordings/status").endswith(
        "/RecordingStatusResponse"
    )
    assert _schema_ref_for(openapi, "/api/v1/webrtc/go2rtc/status").endswith("/Go2RTCStatusResponse")
    assert _schema_ref_for(openapi, "/api/v1/map_cloud/reset", method="post").endswith("/MapLifecycleResponse")
    assert _schema_ref_for(openapi, "/api/v1/map/rename", method="post").endswith("/MapLifecycleResponse")
    assert _request_schema_ref_for(openapi, "/api/v1/map/rename").endswith("/MapRenameRequest")
    assert _schema_ref_for(openapi, "/api/v1/map/save", method="post").endswith("/MapSaveOperationResponse")
    assert _request_schema_ref_for(openapi, "/api/v1/map/save").endswith("/MapSaveRequest")
    assert _schema_ref_for(openapi, "/api/v1/maps/operations").endswith("/MapSaveOperationResponse")
    assert _schema_ref_for(openapi, "/api/v1/maps/operations/{operation_id}").endswith(
        "/MapSaveOperationResponse"
    )
    assert _schema_ref_for(openapi, "/api/v1/app/bootstrap").endswith("/AppBootstrapResponse")
    assert _schema_ref_for(openapi, "/api/v1/app/capabilities").endswith("/AppCapabilitiesResponse")
    assert _schema_ref_for(openapi, "/api/v1/app/traffic").endswith("/AppTrafficResponse")

    assert schemas["SceneGraphResponse"]["properties"]["objects"]["items"]["$ref"].endswith("/SceneGraphObject")
    assert schemas["SceneGraphResponse"]["properties"]["relations"]["items"]["$ref"].endswith("/SceneGraphRelation")
    assert schemas["SceneGraphResponse"]["properties"]["regions"]["items"]["$ref"].endswith("/SceneGraphRegion")
    assert schemas["LocationsResponse"]["properties"]["locations"]["items"]["$ref"].endswith("/LocationEntry")
    assert schemas["LocationOperationResponse"]["properties"]["location"]["anyOf"][0]["$ref"].endswith("/LocationEntry")
    assert schemas["LocationOperationResponse"]["properties"]["locations"]["$ref"].endswith("/LocationsResponse")
    assert schemas["PathResponse"]["properties"]["path"]["items"]["$ref"].endswith("/PathPoint")
    assert schemas["PlanPreviewResponse"]["properties"]["goal"]["$ref"].endswith("/PathPoint")
    assert schemas["PlanPreviewResponse"]["properties"]["path"]["items"]["$ref"].endswith("/PathPoint")
    assert "selected_planner" not in schemas["PlanPreviewResponse"]["properties"]
    assert "plan_safety_policy" not in schemas["PlanPreviewResponse"]["properties"]
    assert "path_safety" not in schemas["PlanPreviewResponse"]["properties"]
    assert "fallback_reason" not in schemas["PlanPreviewResponse"]["properties"]
    assert "rejected_plans" not in schemas["PlanPreviewResponse"]["properties"]
    assert schemas["GoalCandidateResponse"]["properties"]["target"]["anyOf"][0]["$ref"].endswith(
        "/ConstructedGoalTarget"
    )
    assert schemas["GoalCandidateResponse"]["properties"]["preview"]["anyOf"][0]["$ref"].endswith(
        "/PlanPreviewResponse"
    )
    assert schemas["NavigationStatusResponse"]["properties"]["control"]["$ref"].endswith("/NavigationControlSummary")
    assert schemas["NavigationStatusResponse"]["properties"]["readiness"]["$ref"].endswith(
        "/NavigationReadinessSummary"
    )
    assert schemas["NavigationStatusResponse"]["properties"]["progress"]["$ref"].endswith("/NavigationProgressSummary")
    assert schemas["NavigationStatusResponse"]["properties"]["frames"]["$ref"].endswith("/NavigationFrameSummary")
    assert schemas["ReadinessResponse"]["properties"]["runtime"]["$ref"].endswith("/ReadinessRuntimeSummary")
    assert schemas["ReadinessResponse"]["properties"]["product_contract"]["$ref"].endswith(
        "/ReadinessProductContract"
    )
    readiness_runtime = schemas["ReadinessRuntimeSummary"]["properties"]
    assert readiness_runtime["localization"]["anyOf"][0]["$ref"].endswith("/ReadinessLocalizationRuntime")
    assert readiness_runtime["boundary"]["anyOf"][0]["$ref"].endswith("/ReadinessRuntimeBoundary")
    readiness_localization = schemas["ReadinessLocalizationRuntime"]["properties"]
    assert readiness_localization["frames"]["$ref"].endswith("/ReadinessLocalizationFrameSummary")
    assert schemas["ReadinessLocalizationFrameSummary"]["properties"]["mismatches"]["items"]["$ref"].endswith(
        "/NavigationFrameMismatch"
    )
    assert "required_topic_frame_ids" in readiness_localization
    assert "runtime_data_flow_topics" in readiness_localization
    assert "runtime_data_flow_stage_algorithm_interfaces" in readiness_localization
    readiness_boundary = schemas["ReadinessRuntimeBoundary"]["properties"]
    assert readiness_boundary["resolved_runtime_data_flow"]["items"]["$ref"].endswith("/RuntimeDataFlowStageSummary")
    assert readiness_boundary["frames"]["$ref"].endswith("/RuntimeFrameSummary")
    assert readiness_boundary["frame_links"]["additionalProperties"]["$ref"].endswith("/RuntimeFrameLinkSummary")
    assert schemas["NavigationRuntimeBoundary"]["properties"]["resolved_runtime_data_flow"]["items"]["$ref"].endswith(
        "/RuntimeDataFlowStageSummary"
    )
    navigation_boundary = schemas["NavigationRuntimeBoundary"]["properties"]
    assert navigation_boundary["frames"]["$ref"].endswith("/RuntimeFrameSummary")
    assert navigation_boundary["frame_links"]["additionalProperties"]["$ref"].endswith("/RuntimeFrameLinkSummary")
    runtime_frame = schemas["RuntimeFrameSummary"]["properties"]
    assert {
        "map",
        "odom",
        "body",
        "lidar",
        "camera",
        "axis_convention",
        "body_aliases",
        "lidar_aliases",
    } <= set(runtime_frame)
    runtime_frame_link = schemas["RuntimeFrameLinkSummary"]["properties"]
    assert {"parent", "child", "required"} <= set(runtime_frame_link)
    data_flow_stage = schemas["RuntimeDataFlowStageSummary"]["properties"]
    assert {"name", "inputs", "outputs", "owner", "frame_role", "map_dependency"} <= (set(data_flow_stage))
    assert schemas["StateResponse"]["properties"]["localization"]["$ref"].endswith("/LocalizationStatusResponse")
    assert schemas["AppBootstrapResponse"]["properties"]["localization"]["$ref"].endswith("/LocalizationStatusResponse")
    assert schemas["LocalizationStatusResponse"]["properties"]["runtime"]["$ref"].endswith("/NavigationRuntimeBoundary")
    assert schemas["LocalizationStatusResponse"]["properties"]["frames"]["$ref"].endswith("/LocalizationFrameSummary")
    assert schemas["NavigationStatusResponse"]["properties"]["target"]["$ref"].endswith("/NavigationTargetSummary")
    assert schemas["NavigationStatusResponse"]["properties"]["motion"]["$ref"].endswith("/NavigationMotionSummary")
    assert schemas["NavigationStatusResponse"]["properties"]["feedback"]["$ref"].endswith("/NavigationFeedbackSummary")
    assert schemas["NavigationDiagnosticsSummary"]["properties"]["frame_mismatches"]["items"]["$ref"].endswith(
        "/NavigationFrameMismatch"
    )
    diagnostics = schemas["NavigationDiagnosticsSummary"]["properties"]
    assert "safety" in diagnostics
    assert "plan_safety_policy" not in diagnostics
    assert "last_plan_report" not in diagnostics
    assert schemas["AppTrafficResponse"]["properties"]["sse"]["$ref"].endswith("/TrafficSSEStats")
    assert schemas["AppTrafficResponse"]["properties"]["cloud"]["$ref"].endswith("/TrafficCloudStats")
    assert "schema_version" in schemas["ControlCommandResponse"]["properties"]
    assert "ok" in schemas["ControlCommandResponse"]["properties"]
    assert "yaw" in schemas["ControlCommandResponse"]["properties"]
    assert "reason" in schemas["ControlCommandResponse"]["properties"]
    assert schemas["ControlCommandResponse"]["properties"]["target"]["anyOf"][0]["$ref"].endswith(
        "/ConstructedGoalTarget"
    )
    assert schemas["ControlCommandResponse"]["properties"]["command"]["$ref"].endswith("/CommandReceipt")
    assert "schema_version" in schemas["GatewayErrorResponse"]["properties"]
    assert "ok" in schemas["GatewayErrorResponse"]["properties"]
    assert schemas["GatewayErrorResponse"]["properties"]["command"]["anyOf"][0]["$ref"].endswith("/CommandReceipt")
    assert "request_id" in schemas["LeaseRequest"]["properties"]
    assert schemas["LeaseResponse"]["properties"]["command"]["$ref"].endswith("/CommandReceipt")
    assert "command" in schemas["LeaseResponse"]["required"]

    assert "application/sdp" in _content_for(openapi, "/api/v1/webrtc/whep", method="post")
    assert "/api/v1/webrtc/offer" not in openapi["paths"]
    assert "/api/v1/webrtc/stats" not in openapi["paths"]
    assert "/api/v1/webrtc/bitrate" not in openapi["paths"]
    event_stream = _content_for(openapi, "/api/v1/events")["text/event-stream"]
    assert event_stream["schema"]["properties"]["type"]["type"] == "string"
    assert "event_id" in event_stream["schema"]["properties"]
    assert "application/gzip" in _content_for(openapi, "/api/v1/diagnostic_pack")
    assert "application/octet-stream" in _content_for(openapi, "/api/v1/maps/{name}/pcd")
    assert "image/jpeg" in _content_for(openapi, "/api/v1/camera/snapshot")


def test_capabilities_manifest_http_paths_exist_in_openapi():
    from gateway.gateway_module import GatewayModule
    from gateway.services.app_bootstrap import CLIENT_LINKS

    gateway = GatewayModule()
    gateway.setup()

    route = next(route for route in gateway._app.routes if route.path == "/api/v1/app/capabilities")
    capabilities = asyncio.run(route.endpoint())
    openapi_paths = set(gateway._app.openapi()["paths"])

    manifest_paths = set()
    for group in capabilities["endpoints"].values():
        for spec in group.values():
            if spec["method"] != "WS":
                manifest_paths.add(spec["path"])
    for spec in capabilities["probes"].values():
        manifest_paths.add(spec["path"])

    assert manifest_paths <= openapi_paths

    client_link_http_paths = {path.split("?", 1)[0] for path in CLIENT_LINKS.values() if path.startswith("/api/")}
    assert client_link_http_paths <= openapi_paths

    key_specs = [
        capabilities["endpoints"]["state"]["snapshot"],
        capabilities["endpoints"]["state"]["scene_graph"],
        capabilities["endpoints"]["state"]["locations"],
        capabilities["endpoints"]["state"]["location_create"],
        capabilities["endpoints"]["state"]["location_update"],
        capabilities["endpoints"]["state"]["location_delete"],
        capabilities["endpoints"]["state"]["path"],
        capabilities["endpoints"]["state"]["localization_status"],
        capabilities["endpoints"]["state"]["navigation_status"],
        capabilities["endpoints"]["state"]["health"],
        capabilities["endpoints"]["app"]["bootstrap"],
        capabilities["endpoints"]["app"]["capabilities"],
        capabilities["endpoints"]["app"]["traffic"],
        capabilities["endpoints"]["auth"]["login"],
        capabilities["endpoints"]["auth"]["check"],
        capabilities["endpoints"]["control"]["navigation_goal_candidate"],
        capabilities["endpoints"]["control"]["navigation_plan"],
        capabilities["endpoints"]["ops"]["routecheck_latest"],
        capabilities["endpoints"]["ops"]["runtime_contract"],
    ]
    for spec in key_specs:
        assert spec["response_schema"]
        assert "application/json" in spec["response_content_types"]
