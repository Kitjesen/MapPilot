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
        def execute(self, request):
            command = request.to_mapping()
            assert command == {"action": "get_voxel_edits", "name": "demo"}
            return {
                "action": "get_voxel_edits",
                "success": True,
                "map_id": "demo",
                "edits": [],
            }

    app = FastAPI()
    register_map_routes(app, SimpleNamespace(_map_mgr=MapsService()))

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
    assert "/robot/meshes/{filename}" in paths
    assert "/api/v1/map/restore_predufo" in paths
    assert "/api/v1/map/activate" in paths
    assert "/api/v1/map/rename" in paths
    assert "/api/v1/map/save" in paths
    assert "/api/v1/maps/operations" in paths
    assert "/api/v1/maps/operations/{operation_id}" in paths
    assert "/api/v1/maps/operations/{operation_id}/cancel" in paths
    assert "/api/v1/maps/operations/{operation_id}/retry" in paths
    assert "/api/v1/maps/save-jobs" not in paths
    assert "/api/v1/maps/save-jobs/{job_id}" not in paths
    assert "/api/v1/maps/{name}/versions" in paths
    assert "/api/v1/maps/{name}/versions/{version}/rollback" in paths


def test_slam_maps_does_not_turn_native_failure_into_empty_inventory(monkeypatch):
    from fastapi import FastAPI

    import gateway.routes.maps as map_routes

    monkeypatch.setattr(
        map_routes,
        "_map_service_command",
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
    from gateway.routes.maps import _robot_mesh_path

    monkeypatch.delenv("DOG_MESH_DIR", raising=False)
    path = _robot_mesh_path("base_link.STL")

    assert path is not None
    assert path.name == "base_link.STL"
    assert path.parent.name == "meshes"
    assert path.parent.parent.name == "thunderv4"


def test_map_version_routes_forward_typed_commands(monkeypatch):
    from fastapi import FastAPI

    import gateway.routes.maps as map_routes

    commands: list[dict] = []

    def command(_gw, payload):
        commands.append(payload)
        if payload["action"] == "list_map_versions":
            return {
                "success": True,
                "map_id": payload["name"],
                "versions": [{"version": 2, "current": True}],
                "count": 1,
            }
        if payload["action"] == "get_active":
            return {"success": True, "active": "another_map"}
        return {
            "success": True,
            "map_id": payload["name"],
            "version": payload["version"],
        }

    monkeypatch.setattr(map_routes, "_map_service_command", command)
    app = FastAPI()
    map_routes.register_map_routes(app, SimpleNamespace())
    list_route = next(route for route in app.routes if route.path == "/api/v1/maps/{name}/versions")
    rollback_route = next(
        route for route in app.routes if route.path == "/api/v1/maps/{name}/versions/{version}/rollback"
    )

    listed = asyncio.run(list_route.endpoint("warehouse"))
    rolled = asyncio.run(rollback_route.endpoint("warehouse", 1))
    assert listed.status_code == 200
    assert json.loads(listed.body)["versions"][0]["current"] is True
    assert rolled.status_code == 200
    assert json.loads(rolled.body)["version"] == 1
    assert commands == [
        {"action": "list_map_versions", "name": "warehouse"},
        {"action": "get_active"},
        {"action": "rollback_map_version", "name": "warehouse", "version": 1},
    ]


def test_save_map_returns_accepted_while_native_job_is_running(monkeypatch):
    from fastapi import FastAPI

    import gateway.routes.maps as map_routes

    monkeypatch.setattr(
        map_routes,
        "_map_service_command",
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

    monkeypatch.setattr(map_routes, "_map_service_command", submit_save)
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
        def execute(self, request):
            command = request.to_mapping()
            assert command == {"action": "get_voxel_edits", "name": "demo"}
            return {
                "action": "get_voxel_edits",
                "success": True,
                "map_id": "demo",
                "edits": [expected_edit],
            }

    app = FastAPI()
    register_map_routes(app, SimpleNamespace(_map_mgr=MapsService()))

    route = next(route for route in app.routes if route.path == "/api/v1/maps/{name}/voxels/edits")
    payload = asyncio.run(route.endpoint("demo"))

    assert payload["success"] is True
    assert payload["edits"][0]["state"] == "preblocked"


def test_diagnostics_maps_snapshot_uses_map_manager_private_list(
    monkeypatch,
    tmp_path,
):
    from gateway.routes.diagnostics import _maps_snapshot

    class Manager:
        def list_maps(self):
            return {"action": "list", "success": True, "maps": [{"name": "demo"}]}

    monkeypatch.setenv("NAV_MAP_DIR", str(tmp_path))
    payload = _maps_snapshot(SimpleNamespace(_map_mgr=Manager()))

    assert payload["has_manager"] is True
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
    assert "/api/v1/devices" in paths
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
    assert "/api/v1/session/start" in paths
    assert "/api/v1/session/end" in paths


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
    assert "/api/v1/cmd_vel" in routes
    assert "/api/v1/stop" in routes
    assert "/api/v1/navigation/cancel" in routes
    assert "/api/v1/instruction" in routes
    assert "/api/v1/mode" in routes
    assert "/api/v1/lease" in routes
    assert routes["/api/v1/goal"].endpoint.__module__ == "gateway.routes.commands"


def test_map_bundle_file_resolution_rejects_path_escape():
    from types import SimpleNamespace

    from gateway.services.map_service import artifact_path

    temp_root = Path.cwd() / ".tmp" / "test_gateway_route_split" / uuid.uuid4().hex
    map_root = temp_root / "maps"
    map_root.mkdir(parents=True, exist_ok=False)
    map_dir = map_root / "demo"
    map_dir.mkdir()
    safe_file = map_dir / "map.pcd"
    safe_file.write_bytes(b"pcd")
    escaped_file = temp_root / "outside.pcd"
    escaped_file.write_bytes(b"outside")

    class MapsService:
        uri = str(safe_file)

        def execute(self, request):
            command = request.to_mapping()
            return self.get_map_bundle(
                str(command.get("name") or ""),
                str(command.get("capability") or ""),
            )

        def get_map_bundle(self, name: str, capability: str):
            return {
                "success": True,
                "map_id": name,
                "map_dir": str(map_dir),
                "capability": capability,
                "artifact": {"uri": self.uri},
            }

    service = MapsService()
    gateway = SimpleNamespace(_map_mgr=service)

    try:
        assert artifact_path(gateway, "demo", "source_pointcloud") == safe_file
        service.uri = str(escaped_file)
        assert artifact_path(gateway, "demo", "source_pointcloud") is None
    finally:
        shutil.rmtree(temp_root, ignore_errors=True)


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
            assert "diag/app_web/algorithm_benchmark.json" in names
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
        "algorithm_benchmark",
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
    assert validation_gates["runtime_audit"]["schema_version"] == ("lingtu.runtime_contract_audit.v1")
    assert validation_gates["runtime_audit"]["command"] == (
        "python lingtu.py runtime-audit --json-out artifacts/runtime_contract_audit.json"
    )
    assert validation_gates["runtime_audit"]["requires_ros"] is False
    assert validation_gates["runtime_audit"]["acceptance_step"] == 1
    assert validation_gates["runtime_audit"]["requires_prior_gates"] == []
    assert "canonical_runtime_contract_matches_yaml" in (validation_gates["runtime_audit"]["proves"])
    assert validation_gates["runtime_audit"]["collector_publishes_control_topics"] is False
    assert validation_gates["runtime_audit"]["control_topics_published"] == []
    assert "runtime_contract_integrity" in validation_gates["runtime_audit"]["checks"]
    assert "runtime_stage_algorithm_interface_binding" in (validation_gates["runtime_audit"]["validates"])
    assert validation_gates["runtime_audit"]["coverage"]["data_source_resolved_flow_inputs_outputs"] == [
        "runtime_contract_integrity"
    ]
    assert validation_gates["saved_map_artifact_gate"]["command"] == (
        "python lingtu.py saved-map-artifact-gate "
        "<map-dir> "
        "--require-octomap "
        "--require-occupancy "
        "--json-out artifacts/saved_map_artifacts/report.json"
    )
    assert validation_gates["saved_map_artifact_gate"]["script"] == ("scripts/gates/saved_map_artifact_gate.py")
    assert validation_gates["saved_map_artifact_gate"]["artifact"] == ("artifacts/saved_map_artifacts/report.json")
    assert validation_gates["saved_map_artifact_gate"]["acceptance_step"] == 2
    assert validation_gates["saved_map_artifact_gate"]["requires_prior_gates"] == ["runtime_audit"]
    assert validation_gates["saved_map_artifact_gate"]["requires_ros"] is False
    assert validation_gates["saved_map_artifact_gate"]["requires_real_robot_runtime"] is False
    assert validation_gates["saved_map_artifact_gate"]["collector_publishes_control_topics"] is False
    assert validation_gates["saved_map_artifact_gate"]["control_topics_published"] == []
    assert "Required artifacts" in (validation_gates["saved_map_artifact_gate"]["operator_summary_sections"])
    assert "octomap_and_occupancy_derive_from_same_map_pcd" in (validation_gates["saved_map_artifact_gate"]["proves"])
    assert validation_gates["real_runtime_evidence"]["expected_runtime_contract"] == ("real")
    assert validation_gates["real_runtime_evidence"]["command"] == (
        "python lingtu.py real-runtime-evidence "
        "--collector gateway "
        "--gateway-url http://<robot>:5050 "
        "--duration-sec 20 "
        "--json-out artifacts/real_runtime/report.json"
    )
    assert validation_gates["real_runtime_evidence"]["collector_command"] == (
        "python scripts/gates/real_runtime_evidence_collect.py "
        "--collector gateway "
        "--gateway-url http://<robot>:5050 "
        "--duration-sec 20 "
        "--expected-contract real "
        "--json-out artifacts/real_runtime/report.json"
    )
    assert validation_gates["real_runtime_evidence"]["gate_command"] == (
        "python scripts/gates/real_runtime_evidence_gate.py "
        "artifacts/real_runtime/report.json "
        "--expected-contract real "
        "--json-out artifacts/real_runtime/profiles_evidence.json"
    )
    assert validation_gates["real_runtime_evidence"]["artifact"] == ("artifacts/real_runtime/report.json")
    assert validation_gates["real_runtime_evidence"]["acceptance_step"] == 3
    assert validation_gates["real_runtime_evidence"]["requires_prior_gates"] == ["runtime_audit"]
    assert "Topic frame evidence" in (validation_gates["real_runtime_evidence"]["operator_summary_sections"])
    assert "Stage evidence matrix" in (validation_gates["real_runtime_evidence"]["operator_summary_sections"])
    assert "observed_resolved_runtime_data_flow" in (validation_gates["real_runtime_evidence"]["proves"])
    assert validation_gates["real_runtime_evidence"]["requires_real_robot_runtime"] is True
    assert validation_gates["real_runtime_evidence"]["requires_ros"] is False
    assert validation_gates["real_runtime_evidence"]["requires_active_robot_run"] is True
    assert validation_gates["real_runtime_evidence"]["collector_publishes_control_topics"] is False
    assert validation_gates["real_runtime_evidence"]["control_topics_published"] == []
    assert "runtime_data_flow_stage_evidence_payload" in (validation_gates["real_runtime_evidence"]["validates"])
    assert snapshots["traffic"]["ok"] is True
    assert snapshots["traffic"]["data"]["client_policy"]["events_endpoint"] == ("/api/v1/events")
    assert snapshots["media"]["ok"] is True
    assert snapshots["session"]["ok"] is True
    assert snapshots["maps"]["ok"] is True
    assert snapshots["commands"]["ok"] is True
    assert snapshots["commands"]["data"]["idempotency_supported"] is True
    assert snapshots["routecheck"]["ok"] is True
    assert snapshots["routecheck"]["data"]["schema_version"] == 1
    assert snapshots["algorithm_benchmark"]["ok"] is True
    assert snapshots["algorithm_benchmark"]["data"]["read_only"] is True
    assert snapshots["algorithm_benchmark"]["data"]["ros2_topic_required"] is False
    assert snapshots["frame_contract"]["ok"] is True
    frame_contract = snapshots["frame_contract"]["data"]
    assert frame_contract["contract"]["exists"] is True
    assert frame_contract["expected"]["map_frame"] == "map"
    assert frame_contract["expected"]["odom_frame"] == "odom"
    assert frame_contract["expected"]["body_frame"] == "body"
    assert frame_contract["navigation_frames"]["planning_frame_id"] == "map"
    assert "runtime_boundary" in frame_contract
    assert snapshots["runtime_contract"]["ok"] is True
    runtime_contract = snapshots["runtime_contract"]["data"]["manifest"]
    assert runtime_contract["data_sources"]["thunder"]["command_sink"] == ("driver")
    real_flow = {stage["name"]: stage for stage in runtime_contract["resolved_runtime_data_flow"]["thunder"]}
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
    assert body["manifest"]["data_sources"]["thunder"]["mapping_source"] == ("slam_map_cloud")


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
    monkeypatch.setenv("LINGTU_DATA_SOURCE", "thunder")
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
    assert runtime["data_source"] == "thunder"
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
        diagnostics,
        "_load_topic_contract",
        lambda: {"path": "missing.yaml", "exists": False, "data": {}},
    )
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
    assert counts["/api/v1/diagnostics/algorithm-benchmark/latest"] == 1
    assert counts["/api/v1/diagnostics/runtime-contract"] == 1
    assert counts["/api/v1/events"] == 1
    assert counts["/api/v1/state"] == 1
    assert counts["/api/v1/locations"] == 2
    assert counts["/api/v1/locations/{name}"] == 2
    assert counts["/api/v1/localization/status"] == 1
    assert counts["/api/v1/navigation/status"] == 1
    assert counts["/api/v1/runtime/dataflow"] == 1
    assert counts["/api/v1/runtime/switch-plan"] == 1
    assert counts["/api/v1/runtime/switch"] == 0
    assert counts["/api/v1/navigation"] == 0
    assert counts["/api/v1/status"] == 0
    assert counts["/api/v1/health"] == 1
    assert counts["/health"] == 1
    assert counts["/ready"] == 1
    assert counts["/api/v1/session"] == 1
    assert counts["/api/v1/session/start"] == 1
    assert counts["/api/v1/session/end"] == 1
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
    assert counts["/api/v1/cmd_vel"] == 1
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
        "/api/v1/runtime/switch-plan",
        "/api/v1/devices",
        "/api/v1/health",
        "/health",
        "/ready",
        "/api/v1/session",
        "/api/v1/diagnostic_pack",
        "/api/v1/diagnostics/routecheck/latest",
        "/api/v1/diagnostics/real-runtime-evidence/latest",
        "/api/v1/diagnostics/algorithm-benchmark/latest",
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
        "/api/v1/cmd_vel",
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
        "phases": {"baseline": {"selected_planner": "pct"}},
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
    assert payload["latest"]["phases"]["baseline"]["selected_planner"] == "pct"

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
    routecheck_dir = project / "artifacts" / "server_sim_closure" / "routecheck"
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


def _write_algorithm_benchmark_summary(
    root: Path,
    name: str = "summary_dimos_benchmark_test.json",
    *,
    ok: bool = True,
    claim_allowed: bool = True,
    missing_or_failed: list[str] | None = None,
    claim_boundary: dict | None = None,
    gates: dict | None = None,
) -> Path:
    from gateway.routes.diagnostics import DIMOS_BENCHMARK_REQUIRED_GATES

    root.mkdir(parents=True, exist_ok=True)
    missing = missing_or_failed or []
    summary = {
        "schema_version": "lingtu.server_sim_closure.summary.v1",
        "ok": ok,
        "missing_or_failed": missing,
        "gates": gates or {},
        "algorithm_validation": {
            "claim_allowed": claim_allowed,
            "required_gate_sequence": list(DIMOS_BENCHMARK_REQUIRED_GATES),
            "validation_flow": [{"gate": "routecheck_preflight", "ok": True}],
            "claim_boundary": {
                "ros2_topic_required": False,
                "read_only": True,
                "global_planning_source": "static_saved_map_octomap",
                "live_costmap_role": "local_planning_and_safety_only",
                **(claim_boundary or {}),
            },
            "blocking_categories": {"hard": missing},
        },
    }
    path = root / name
    path.write_text(json.dumps(summary), encoding="utf-8")
    return path


def test_algorithm_benchmark_latest_diagnostic_reads_readonly_artifact(
    tmp_path,
    monkeypatch,
):
    from fastapi import FastAPI
    from fastapi.testclient import TestClient

    from gateway.routes.diagnostics import (
        build_algorithm_benchmark_latest_summary,
        register_diagnostic_routes,
    )

    summary_path = _write_algorithm_benchmark_summary(tmp_path)
    os.utime(summary_path, (100, 100))
    monkeypatch.setattr("gateway.routes.diagnostics.time.time", lambda: 200.0)

    payload = build_algorithm_benchmark_latest_summary(tmp_path, max_age_s=1000.0)

    assert payload["ok"] is True
    assert payload["schema_version"] == "lingtu.algorithm_benchmark_latest.v1"
    assert payload["read_only"] is True
    assert payload["ros2_topic_required"] is False
    assert payload["publishes"] == []
    assert payload["summary_path"] == str(summary_path)
    assert payload["report_age_s"] == 100.0
    assert payload["claim_allowed"] is True
    assert payload["missing_or_failed"] == []
    assert payload["blockers"] == []

    monkeypatch.setenv("LINGTU_ALGORITHM_BENCHMARK_ROOT", str(tmp_path))
    monkeypatch.setenv("LINGTU_ALGORITHM_BENCHMARK_MAX_AGE_SEC", "1000")
    app = FastAPI()
    register_diagnostic_routes(app, SimpleNamespace(_all_modules={}))
    response = TestClient(app).get("/api/v1/diagnostics/algorithm-benchmark/latest")
    assert response.status_code == 200
    body = response.json()
    assert body["schema_version"] == "lingtu.algorithm_benchmark_latest.v1"
    assert body["ok"] is True
    assert body["read_only"] is True
    assert body["ros2_topic_required"] is False
    assert body["publishes"] == []


def test_algorithm_benchmark_latest_diagnostic_reports_missing_artifact(tmp_path):
    from gateway.routes.diagnostics import build_algorithm_benchmark_latest_summary

    payload = build_algorithm_benchmark_latest_summary(tmp_path)

    assert payload["ok"] is False
    assert payload["count"] == 0
    assert payload["reason"] == "algorithm_benchmark_report_not_found"
    assert payload["read_only"] is True
    assert payload["ros2_topic_required"] is False
    assert payload["publishes"] == []
    assert payload["dimos_gap"]["schema_version"] == "lingtu.dimos_gap_report.v1"
    assert payload["dimos_gap"]["gap_counts"]["failed"] == len(payload["required_gate_sequence"])
    assert payload["dimos_gap"]["lingtu_readiness"]["highest_priority_blocker"] == ("moving_obstacle_sweep")
    assert payload["dimos_gap"]["next_steps"][0]["gate"] == ("gateway_runtime_acceptance")


def test_algorithm_benchmark_latest_diagnostic_blocks_failed_or_stale_artifact(
    tmp_path,
    monkeypatch,
):
    from gateway.routes.diagnostics import build_algorithm_benchmark_latest_summary

    summary_path = _write_algorithm_benchmark_summary(
        tmp_path,
        ok=False,
        claim_allowed=False,
        missing_or_failed=["large_loop_closure", "moving_obstacle_sweep"],
    )
    os.utime(summary_path, (100, 100))
    monkeypatch.setattr("gateway.routes.diagnostics.time.time", lambda: 200.0)

    payload = build_algorithm_benchmark_latest_summary(tmp_path, max_age_s=50.0)

    assert payload["ok"] is False
    assert payload["reason"] == "algorithm_benchmark_report_stale"
    assert "algorithm benchmark summary is stale" in payload["blockers"]
    assert "algorithm benchmark summary is not passing" in payload["blockers"]
    assert "algorithm validation claim_allowed is not true" in payload["blockers"]
    assert payload["missing_or_failed"] == [
        "large_loop_closure",
        "moving_obstacle_sweep",
    ]
    assert payload["dimos_gap"]["gap_counts"]["p0"] == 2
    assert payload["dimos_gap"]["lingtu_readiness"]["highest_priority_blocker"] == ("moving_obstacle_sweep")
    assert payload["dimos_gap"]["next_steps"][0]["gate"] == ("gateway_runtime_acceptance")


def test_algorithm_benchmark_latest_diagnostic_marks_stale_green_gap_not_ready(
    tmp_path,
    monkeypatch,
):
    from gateway.routes.diagnostics import (
        DIMOS_BENCHMARK_REQUIRED_GATES,
        build_algorithm_benchmark_latest_summary,
    )

    gates = {
        gate: {
            "name": gate,
            "ok": True,
            "status": "passed",
            "path": f"artifacts/server_sim_closure/{gate}/report.json",
            "blockers": [],
        }
        for gate in DIMOS_BENCHMARK_REQUIRED_GATES
    }
    summary_path = _write_algorithm_benchmark_summary(
        tmp_path,
        ok=True,
        claim_allowed=True,
        missing_or_failed=[],
        gates=gates,
    )
    os.utime(summary_path, (100, 100))
    monkeypatch.setattr("gateway.routes.diagnostics.time.time", lambda: 200.0)

    payload = build_algorithm_benchmark_latest_summary(tmp_path, max_age_s=50.0)

    dimos_gap = payload["dimos_gap"]
    assert payload["ok"] is False
    assert payload["reason"] == "algorithm_benchmark_report_stale"
    assert dimos_gap["lingtu_readiness"]["ok"] is False
    assert dimos_gap["lingtu_readiness"]["summary_fresh"] is False
    assert dimos_gap["summary_freshness"] == {
        "checked": True,
        "fresh": False,
        "report_age_s": 100.0,
        "max_age_s": 50.0,
        "blocker": "algorithm benchmark summary is stale",
    }
    assert dimos_gap["gap_counts"]["failed"] == len(DIMOS_BENCHMARK_REQUIRED_GATES)
    assert all(row["status"] == "stale" for row in dimos_gap["gap_matrix"])
    assert dimos_gap["next_steps"][0]["recommended_action"] == (
        "regenerate a fresh DimOS benchmark summary before claiming readiness"
    )


def test_algorithm_benchmark_latest_diagnostic_includes_runtime_dataflow(
    tmp_path,
):
    from gateway.routes.diagnostics import build_algorithm_benchmark_latest_summary

    native_report = tmp_path / "native_pct_mujoco" / "report.json"
    native_report.parent.mkdir(parents=True, exist_ok=True)
    native_report.write_text(
        json.dumps(
            {
                "schema_version": "lingtu.native_pct_mujoco_gate.v1",
                "ok": False,
                "pct_runtime_ok": True,
                "pct_path_count": 8,
                "selected_planner": "pct",
                "fallback_used": False,
                "source_planning_contract": {
                    "pct_optimizer_enabled": False,
                    "pct_planner_path_mode": "native_astar_raw_path",
                },
                "path_count": 2,
                "max_path_poses": 5,
                "local_path_samples": [{"points": [[0, 0, 0], [1, 0, 0]]}],
                "cmd_count_nonzero": 0,
                "cmd_samples": [],
                "moved_m": 0.0,
                "reached_goal": False,
            }
        ),
        encoding="utf-8",
    )
    _write_algorithm_benchmark_summary(
        tmp_path,
        ok=False,
        claim_allowed=False,
        missing_or_failed=["native_pct_mujoco"],
        gates={
            "native_pct_mujoco": {
                "ok": False,
                "status": "failed",
                "path": str(native_report),
                "blockers": ["cmd_vel evidence missing"],
            },
        },
    )

    payload = build_algorithm_benchmark_latest_summary(tmp_path, max_age_s=1000.0)

    rows = {row["gate"]: row for row in payload["dimos_gap"]["gap_matrix"]}
    flow = rows["native_pct_mujoco"]["runtime_dataflow"]
    assert flow["checked"] is True
    assert flow["edge_status"]["pct_backend"] is True
    assert flow["edge_status"]["global_path_to_local_planner"] is True
    assert flow["edge_status"]["path_follower_to_cmd_vel"] is False
    assert flow["primary_blocker"] == "path_follower_to_cmd_vel"
    assert (
        payload["dimos_gap"]["runtime_dataflow"]["failing_primary_blockers"]["native_pct_mujoco"]
        == "path_follower_to_cmd_vel"
    )


def test_algorithm_benchmark_latest_diagnostic_includes_aggregate_child_dataflow(
    tmp_path,
):
    from sim.scripts import dimos_gap_report

    from gateway.routes.diagnostics import build_algorithm_benchmark_latest_summary

    child_report = tmp_path / "moving_obstacle_sweep" / "children" / "report.json"
    child_report.parent.mkdir(parents=True, exist_ok=True)
    child_report.write_text(
        json.dumps(
            {
                "schema_version": "lingtu.mujoco_fastlio2_live_gate.v2",
                "ok": False,
                "outputs": {
                    "fastlio2_cloud_registered": 2,
                    "fastlio2_odometry": 2,
                    "nav_odometry": 2,
                    "nav_registered_cloud": 2,
                    "nav_cmd_vel": 4,
                    "nav_cmd_vel_nonzero": 0,
                },
                "lingtu_inspection": {
                    "global_path_count": 1,
                    "local_path_count": 1,
                    "successful_navigation_goal_count": 0,
                    "min_required_checkpoints": 1,
                },
                "fastlio2_motion_consistency": {"ok": True},
                "fastlio2_z_consistency": {"ok": True},
                "fastlio2_yaw_consistency": {"ok": True},
            }
        ),
        encoding="utf-8",
    )
    aggregate_report = tmp_path / "moving_obstacle_sweep" / "report.json"
    aggregate_report.write_text(
        json.dumps(
            {
                "schema_version": "lingtu.moving_obstacle_sweep_gate.v1",
                "ok": False,
                "minimal_red_defect": {"path": str(child_report)},
            }
        ),
        encoding="utf-8",
    )
    summary_path = _write_algorithm_benchmark_summary(
        tmp_path,
        ok=False,
        claim_allowed=False,
        missing_or_failed=["moving_obstacle_sweep"],
        gates={
            "moving_obstacle_sweep": {
                "ok": False,
                "status": "failed",
                "path": str(aggregate_report),
                "blockers": ["child cmd_vel evidence missing"],
            },
        },
    )

    payload = build_algorithm_benchmark_latest_summary(tmp_path, max_age_s=1000.0)
    cli_payload = dimos_gap_report.build_gap_report(
        summary_path=summary_path,
        include_dataflow=True,
    )

    rows = {row["gate"]: row for row in payload["dimos_gap"]["gap_matrix"]}
    cli_rows = {row["gate"]: row for row in cli_payload["gap_matrix"]}
    flow = rows["moving_obstacle_sweep"]["runtime_dataflow"]
    cli_flow = cli_rows["moving_obstacle_sweep"]["runtime_dataflow"]
    assert flow["checked"] is True
    assert flow["source_gate_report"] == str(aggregate_report)
    assert flow["source_report"] == str(child_report)
    assert flow["edge_status"]["global_planner_to_local_planner"] is True
    assert flow["edge_status"]["path_follower_to_cmd_vel"] is False
    assert flow["primary_blocker"] == "path_follower_to_cmd_vel"
    assert (
        payload["dimos_gap"]["runtime_dataflow"]["failing_primary_blockers"]["moving_obstacle_sweep"]
        == "path_follower_to_cmd_vel"
    )
    assert flow == cli_flow
    assert (
        payload["dimos_gap"]["runtime_dataflow"]["failing_primary_blockers"]
        == cli_payload["runtime_dataflow"]["failing_primary_blockers"]
    )


def test_algorithm_benchmark_latest_diagnostic_includes_embedded_case_dataflow(
    tmp_path,
):
    from gateway.routes.diagnostics import build_algorithm_benchmark_latest_summary

    aggregate_report = tmp_path / "large_loop_closure" / "report.json"
    aggregate_report.parent.mkdir(parents=True, exist_ok=True)
    aggregate_report.write_text(
        json.dumps(
            {
                "schema_version": "lingtu.large_loop_closure_gate.v1",
                "ok": False,
                "minimal_red_defect": {
                    "path": str(tmp_path / "large_loop_closure" / "missing_child.json"),
                    "fastlio2_path_length_m": 12.0,
                    "global_path_points_max": 8,
                    "local_path_count": 1,
                    "local_path_points_max": 4,
                    "nav_cmd_vel_nonzero": 0,
                    "successful_navigation_goal_count": 0,
                    "min_required_checkpoints": 2,
                },
            }
        ),
        encoding="utf-8",
    )
    _write_algorithm_benchmark_summary(
        tmp_path,
        ok=False,
        claim_allowed=False,
        missing_or_failed=["large_loop_closure"],
        gates={
            "large_loop_closure": {
                "ok": False,
                "status": "failed",
                "path": str(aggregate_report),
                "blockers": ["embedded case cmd_vel evidence missing"],
            },
        },
    )

    payload = build_algorithm_benchmark_latest_summary(tmp_path, max_age_s=1000.0)

    rows = {row["gate"]: row for row in payload["dimos_gap"]["gap_matrix"]}
    flow = rows["large_loop_closure"]["runtime_dataflow"]
    assert flow["checked"] is True
    assert flow["schema_detected"] == "aggregate_case"
    assert flow["edge_status"]["fastlio_feedback"] is True
    assert flow["edge_status"]["global_path"] is True
    assert flow["edge_status"]["local_path"] is True
    assert flow["edge_status"]["cmd_vel"] is False
    assert flow["primary_blocker"] == "cmd_vel"
    assert payload["dimos_gap"]["runtime_dataflow"]["failing_primary_blockers"]["large_loop_closure"] == "cmd_vel"


def test_algorithm_benchmark_latest_endpoint_preserves_runtime_dataflow(
    tmp_path,
    monkeypatch,
):
    from fastapi import FastAPI
    from fastapi.testclient import TestClient

    from gateway.routes.diagnostics import register_diagnostic_routes

    child_report = tmp_path / "moving_obstacle_sweep" / "children" / "report.json"
    child_report.parent.mkdir(parents=True, exist_ok=True)
    child_report.write_text(
        json.dumps(
            {
                "schema_version": "lingtu.mujoco_fastlio2_live_gate.v2",
                "ok": False,
                "outputs": {
                    "fastlio2_cloud_registered": 2,
                    "fastlio2_odometry": 2,
                    "nav_odometry": 2,
                    "nav_registered_cloud": 2,
                    "nav_cmd_vel": 3,
                    "nav_cmd_vel_nonzero": 0,
                },
                "lingtu_inspection": {
                    "global_path_count": 1,
                    "local_path_count": 1,
                    "successful_navigation_goal_count": 0,
                    "min_required_checkpoints": 1,
                },
                "fastlio2_motion_consistency": {"ok": True},
                "fastlio2_z_consistency": {"ok": True},
                "fastlio2_yaw_consistency": {"ok": True},
            }
        ),
        encoding="utf-8",
    )
    aggregate_report = tmp_path / "moving_obstacle_sweep" / "report.json"
    aggregate_report.write_text(
        json.dumps(
            {
                "schema_version": "lingtu.moving_obstacle_sweep_gate.v1",
                "ok": False,
                "minimal_red_defect": {"path": str(child_report)},
            }
        ),
        encoding="utf-8",
    )
    _write_algorithm_benchmark_summary(
        tmp_path,
        ok=False,
        claim_allowed=False,
        missing_or_failed=["moving_obstacle_sweep"],
        gates={
            "moving_obstacle_sweep": {
                "ok": False,
                "status": "failed",
                "path": str(aggregate_report),
                "blockers": ["child cmd_vel evidence missing"],
            },
        },
    )

    monkeypatch.setenv("LINGTU_ALGORITHM_BENCHMARK_ROOT", str(tmp_path))
    monkeypatch.setenv("LINGTU_ALGORITHM_BENCHMARK_MAX_AGE_SEC", "1000")
    app = FastAPI()
    register_diagnostic_routes(app, SimpleNamespace(_all_modules={}))

    response = TestClient(app).get("/api/v1/diagnostics/algorithm-benchmark/latest")

    assert response.status_code == 200
    body = response.json()
    rows = {row["gate"]: row for row in body["dimos_gap"]["gap_matrix"]}
    flow = rows["moving_obstacle_sweep"]["runtime_dataflow"]
    assert body["dimos_gap"]["runtime_dataflow"]["checked"] is True
    assert flow["primary_blocker"] == "path_follower_to_cmd_vel"
    assert flow["source_gate_report"] == str(aggregate_report)
    assert flow["source_report"] == str(child_report)
    assert (
        body["dimos_gap"]["runtime_dataflow"]["failing_primary_blockers"]["moving_obstacle_sweep"]
        == "path_follower_to_cmd_vel"
    )


def test_algorithm_benchmark_latest_splits_inspection_variant_from_strict_benchmark(
    tmp_path,
):
    from gateway.routes.diagnostics import build_algorithm_benchmark_latest_summary
    from runtime.algorithm_gates import INSPECTION_MVP_REQUIRED_GATES

    product_gates = {
        "gateway_runtime_acceptance",
        "routecheck_preflight",
        "large_terrain",
        "fastlio2_dynamic_inspection",
        "dynamic_obstacle_local_planner",
        "moving_obstacle_sweep",
    }
    strict_only_failures = [
        "native_pct_mujoco",
        "large_loop_closure",
        "gazebo_runtime",
        "saved_map_relocalize",
        "pct_saved_map_navigation",
    ]
    gates = {gate: {"ok": True, "status": "pass"} for gate in product_gates}
    gates.update({gate: {"ok": False, "status": "fail"} for gate in strict_only_failures})
    _write_algorithm_benchmark_summary(
        tmp_path,
        ok=False,
        claim_allowed=False,
        missing_or_failed=strict_only_failures,
        gates=gates,
    )

    payload = build_algorithm_benchmark_latest_summary(tmp_path, max_age_s=1000.0)

    assert payload["ok"] is False
    assert payload["active_benchmark_variant"] == "inspection_mvp"
    inspection = payload["benchmark_variants"]["inspection_mvp"]
    strict = payload["benchmark_variants"]["dimos_benchmark"]
    assert inspection["ok"] is True
    assert inspection["ros2_topic_required"] is False
    assert inspection["missing_or_failed"] == []
    assert tuple(inspection["required_gate_sequence"]) == INSPECTION_MVP_REQUIRED_GATES
    assert strict["ok"] is False
    assert strict["missing_or_failed"] == strict_only_failures


def test_algorithm_benchmark_latest_rejects_live_costmap_global_planning_claim(
    tmp_path,
):
    from gateway.routes.diagnostics import build_algorithm_benchmark_latest_summary

    _write_algorithm_benchmark_summary(
        tmp_path,
        claim_boundary={
            "global_planning_source": "live_costmap",
            "live_costmap_role": "global_planning",
        },
    )

    payload = build_algorithm_benchmark_latest_summary(tmp_path, max_age_s=1000.0)

    assert payload["ok"] is False
    assert payload["reason"] == "algorithm_benchmark_not_passing"
    assert "algorithm claim boundary must keep live_costmap local-only" in payload["blockers"]


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
        "checked_real_motion_evidence": {"ok": True},
        "checked_hardware_boundary_evidence": {"ok": True},
        "checked_live_topic_freshness": {"/slam/odometry": {"ok": True}},
        "checked_runtime_data_flow_evidence": {
            "command_boundary": {"ok": True},
        },
        "checked_frame_link_evidence": {
            "map_to_odom": {"ok": True},
            "odom_to_body": {"ok": True},
            "body_to_lidar": {"ok": True},
            "body_to_camera": {"ok": True},
        },
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
    assert payload["checked_real_motion_evidence"] == {"ok": True}
    assert payload["checked_hardware_boundary_evidence"] == {"ok": True}

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
            "checked_real_motion_evidence": {"ok": False},
            "checked_hardware_boundary_evidence": {"ok": True},
            "checked_live_topic_freshness": {"/slam/odometry": {"ok": True}},
            "checked_runtime_data_flow_evidence": {
                "command_boundary": {"ok": True},
            },
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


def test_real_runtime_evidence_preflight_allows_no_motion_without_hardware_boundary(
    tmp_path,
):
    from gateway.routes.diagnostics import build_real_runtime_evidence_latest_summary

    validation = {
        "schema_version": "lingtu.real_runtime_evidence.validation.v1",
        "ok": False,
        "expected_contract": "real",
        "hardware_boundary_required": False,
        "checked_real_motion_evidence": {"ok": False},
        "checked_hardware_boundary_evidence": {"ok": False},
        "checked_live_topic_freshness": {
            "/slam/odometry": {"ok": True},
            "/nav/cmd_vel": {"ok": False},
        },
        "checked_runtime_data_flow_evidence": {
            "endpoint_adapter": {"ok": True},
            "slam_or_relayed_localization_map": {"ok": True},
        },
        "checked_frame_link_evidence": {
            "map_to_odom": {"ok": True},
            "odom_to_body": {"ok": True},
            "body_to_lidar": {"ok": True},
            "body_to_camera": {"ok": True},
        },
        "blockers": ["no motion yet"],
    }
    report_path = _write_real_runtime_evidence_report(
        tmp_path / "real_runtime",
        runtime_evidence=validation,
    )
    report = json.loads(report_path.read_text(encoding="utf-8"))
    report["real_robot_motion"] = False
    report["cmd_vel_sent_to_hardware"] = False
    report_path.write_text(json.dumps(report), encoding="utf-8")

    payload = build_real_runtime_evidence_latest_summary(
        tmp_path,
        max_age_s=1000.0,
        now=200.0,
    )

    assert payload["ok"] is False
    assert payload["preflight_ok"] is True
    assert "real-runtime-evidence hardware command route missing" not in payload["preflight_blockers"]


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


def test_real_runtime_evidence_latest_diagnostic_rejects_missing_data_flow_section(
    tmp_path,
):
    from gateway.routes.diagnostics import build_real_runtime_evidence_latest_summary

    validation = {
        "schema_version": "lingtu.real_runtime_evidence.validation.v1",
        "ok": True,
        "expected_contract": "real",
        "checked_real_motion_evidence": {"ok": True},
        "checked_hardware_boundary_evidence": {"ok": True},
        "checked_live_topic_freshness": {"/slam/odometry": {"ok": True}},
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

    assert payload["ok"] is False
    assert payload["reason"] == "real-runtime-evidence data-flow section missing or failed"
    assert "real-runtime-evidence data-flow section missing or failed" in payload["blockers"]


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
    assert "script" in validation_gate_properties
    assert "collector_command" in validation_gate_properties
    assert "gate_command" in validation_gate_properties
    assert "requires_real_robot_runtime" in validation_gate_properties
    assert "requires_active_robot_run" in validation_gate_properties
    assert "collector_publishes_control_topics" in validation_gate_properties
    assert "control_topics_published" in validation_gate_properties
    assert "validates" in validation_gate_properties
    assert "checks" in validation_gate_properties
    assert "coverage" in validation_gate_properties
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
    assert "DevicesResponse" in schemas
    assert "LivenessResponse" in schemas
    assert "ControlCommandResponse" in schemas
    assert "GatewayErrorResponse" in schemas
    assert "RoutecheckLatestResponse" in schemas
    assert "RuntimeContractResponse" in schemas
    assert "RuntimeContractManifest" in schemas
    assert "RuntimeDataflowResponse" in schemas
    assert "RuntimeSwitchPlanRequest" in schemas
    assert "RuntimeSwitchPlanResponse" in schemas
    assert "RuntimeDataflowTopicSummary" in schemas
    assert "RuntimeDataflowObservability" in schemas
    assert "RuntimeDataflowCommunication" in schemas
    assert "RuntimeDataflowPortSummary" in schemas
    assert "RuntimeDataSourceSummary" in schemas
    assert "RuntimeAlgorithmInterfaceSummary" in schemas
    assert "RuntimeAdapterAliasSummary" in schemas
    assert "RuntimeProfileDataSourceBinding" in schemas
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
    assert "SessionStartRequest" in schemas
    assert "SessionTransitionResponse" in schemas
    assert "MapNameRequest" in schemas
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
    assert "SlamSwitchRequest" in schemas
    assert "SlamRelocalizeRequest" in schemas
    assert "SlamOperationResponse" in schemas
    assert "schema_version" in schemas["SlamOperationResponse"]["properties"]
    assert "ok" in schemas["SlamOperationResponse"]["properties"]
    assert "ts" in schemas["SlamOperationResponse"]["properties"]
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
    assert _schema_ref_for(openapi, "/api/v1/devices").endswith("/DevicesResponse")
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
    assert _schema_ref_for(openapi, "/api/v1/diagnostics/algorithm-benchmark/latest").endswith(
        "/AlgorithmBenchmarkLatestResponse"
    )
    assert _schema_ref_for(openapi, "/api/v1/diagnostics/runtime-contract").endswith("/RuntimeContractResponse")
    assert _schema_ref_for(openapi, "/api/v1/runtime/dataflow").endswith("/RuntimeDataflowResponse")
    assert _schema_ref_for(openapi, "/api/v1/runtime/dataflow/subscribe", method="post").endswith(
        "/RuntimeDataflowSubscribeResponse"
    )
    assert _request_schema_ref_for(openapi, "/api/v1/runtime/dataflow/subscribe").endswith(
        "/RuntimeDataflowSubscribeRequest"
    )
    assert _schema_ref_for(openapi, "/api/v1/runtime/switch-plan", method="post").endswith("/RuntimeSwitchPlanResponse")
    assert _request_schema_ref_for(openapi, "/api/v1/runtime/switch-plan").endswith("/RuntimeSwitchPlanRequest")
    field_products = {
        "teleop",
        "teleop_avoid",
        "map",
        "explore",
        "nav",
        "tracking",
        "inspection",
    }
    for schema_name in ("RuntimeSwitchPlanRequest",):
        properties = schemas[schema_name]["properties"]
        assert {"current_product", "target_product"} <= set(properties)
        assert set(properties["target_product"]["enum"]) == field_products
    assert schemas["RuntimeContractResponse"]["properties"]["manifest"]["$ref"].endswith("/RuntimeContractManifest")
    assert schemas["RuntimeDataflowResponse"]["properties"]["topics"]["items"]["$ref"].endswith(
        "/RuntimeDataflowTopicSummary"
    )
    assert "runtime_dataflow_subscribe" in schemas["ClientLinks"]["properties"]
    algorithm_schema = schemas["AlgorithmBenchmarkLatestResponse"]["properties"]
    assert {
        "read_only",
        "ros2_topic_required",
        "publishes",
        "claim_allowed",
        "missing_or_failed",
        "required_gate_sequence",
        "dimos_gap",
    } <= set(algorithm_schema)
    assert schemas["RuntimeDataflowTopicSummary"]["properties"]["observability"]["$ref"].endswith(
        "/RuntimeDataflowObservability"
    )
    assert schemas["RuntimeDataflowTopicSummary"]["properties"]["communication"]["$ref"].endswith(
        "/RuntimeDataflowCommunication"
    )
    contract_manifest = schemas["RuntimeContractManifest"]["properties"]
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
    assert contract_manifest["profile_data_sources"]["additionalProperties"]["$ref"].endswith(
        "/RuntimeProfileDataSourceBinding"
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
    profile_binding_schema = schemas["RuntimeProfileDataSourceBinding"]["properties"]
    assert {"profile", "data_source", "mode", "note"} <= set(profile_binding_schema)
    product_binding_schema = schemas["RuntimeProductDataSourceBinding"]["properties"]
    assert {"product", "data_source", "mode", "note"} <= set(product_binding_schema)
    transform_schema = schemas["RuntimeTransform3D"]["properties"]
    assert {"parent", "child", "x", "y", "z", "roll", "pitch", "yaw"} <= set(transform_schema)
    message_format_schema = schemas["RuntimeMessageFormatSummary"]["properties"]
    assert {"name", "ros_type", "frame_role", "required_fields", "note"} <= set(message_format_schema)
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
    assert _schema_ref_for(openapi, "/api/v1/cmd_vel", method="post").endswith("/ControlCommandResponse")
    assert _schema_ref_for(openapi, "/api/v1/stop", method="post").endswith("/ControlCommandResponse")
    assert _schema_ref_for(openapi, "/api/v1/navigation/cancel", method="post").endswith("/ControlCommandResponse")
    assert _request_schema_ref_for(openapi, "/api/v1/navigation/cancel").endswith("/CancelRequest")
    assert _schema_ref_for(openapi, "/api/v1/instruction", method="post").endswith("/ControlCommandResponse")
    assert _schema_ref_for(openapi, "/api/v1/mode", method="post").endswith("/ControlCommandResponse")
    for path in (
        "/api/v1/goal",
        "/api/v1/navigate/click",
        "/api/v1/cmd_vel",
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
    assert _schema_ref_for(openapi, "/api/v1/session/start", method="post").endswith("/SessionTransitionResponse")
    assert _request_schema_ref_for(openapi, "/api/v1/session/start").endswith("/SessionStartRequest")
    assert _schema_ref_for(openapi, "/api/v1/session/end", method="post").endswith("/SessionTransitionResponse")
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
    assert _schema_ref_for(openapi, "/api/v1/slam/switch", method="post").endswith("/SlamOperationResponse")
    assert _request_schema_ref_for(openapi, "/api/v1/slam/switch").endswith("/SlamSwitchRequest")
    assert _schema_ref_for(openapi, "/api/v1/slam/restart", method="post").endswith("/SlamOperationResponse")
    assert _schema_ref_for(openapi, "/api/v1/slam/auto_relocalize", method="post").endswith("/SlamOperationResponse")
    assert _schema_ref_for(openapi, "/api/v1/slam/relocalize", method="post").endswith("/SlamOperationResponse")
    assert _request_schema_ref_for(openapi, "/api/v1/slam/relocalize").endswith("/SlamRelocalizeRequest")
    assert _schema_ref_for(openapi, "/api/v1/slam/track_against_map", method="post").endswith("/SlamOperationResponse")
    assert _request_schema_ref_for(openapi, "/api/v1/slam/track_against_map").endswith("/SlamRelocalizeRequest")
    for prefix in ("/api/v1/recordings", "/api/v1/bag"):
        assert _schema_ref_for(openapi, f"{prefix}/start", method="post").endswith(
            "/RecordingOperationResponse"
        )
        assert _request_schema_ref_for(openapi, f"{prefix}/start").endswith(
            "/RecordingStartRequest"
        )
        assert _schema_ref_for(openapi, f"{prefix}/stop", method="post").endswith(
            "/RecordingOperationResponse"
        )
        assert _schema_ref_for(openapi, f"{prefix}/status").endswith(
            "/RecordingStatusResponse"
        )
    assert openapi["paths"]["/api/v1/bag/start"]["post"]["deprecated"] is True
    assert _schema_ref_for(openapi, "/api/v1/webrtc/go2rtc/status").endswith("/Go2RTCStatusResponse")
    assert _schema_ref_for(openapi, "/api/v1/map_cloud/reset", method="post").endswith("/MapLifecycleResponse")
    assert _schema_ref_for(openapi, "/api/v1/map/activate", method="post").endswith("/MapLifecycleResponse")
    assert _request_schema_ref_for(openapi, "/api/v1/map/activate").endswith("/MapNameRequest")
    assert _schema_ref_for(openapi, "/api/v1/map/rename", method="post").endswith("/MapLifecycleResponse")
    assert _request_schema_ref_for(openapi, "/api/v1/map/rename").endswith("/MapRenameRequest")
    assert _schema_ref_for(openapi, "/api/v1/map/save", method="post").endswith("/MapSaveOperationResponse")
    assert _request_schema_ref_for(openapi, "/api/v1/map/save").endswith("/MapSaveRequest")
    assert _schema_ref_for(openapi, "/api/v1/maps/operations").endswith("/MapSaveOperationResponse")
    assert _schema_ref_for(openapi, "/api/v1/maps/operations/{operation_id}").endswith(
        "/MapSaveOperationResponse"
    )
    assert _schema_ref_for(openapi, "/api/v1/map/restore_predufo", method="post").endswith("/MapLifecycleResponse")
    assert _request_schema_ref_for(openapi, "/api/v1/map/restore_predufo").endswith("/MapNameRequest")
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
    assert "selected_planner" in schemas["PlanPreviewResponse"]["properties"]
    assert "plan_safety_policy" in schemas["PlanPreviewResponse"]["properties"]
    assert "path_safety" in schemas["PlanPreviewResponse"]["properties"]
    assert "fallback_reason" in schemas["PlanPreviewResponse"]["properties"]
    assert "rejected_plans" in schemas["PlanPreviewResponse"]["properties"]
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
    assert "plan_safety_policy" in schemas["NavigationDiagnosticsSummary"]["properties"]
    assert "last_plan_report" in schemas["NavigationDiagnosticsSummary"]["properties"]
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
        capabilities["endpoints"]["state"]["devices"],
        capabilities["endpoints"]["state"]["health"],
        capabilities["endpoints"]["app"]["bootstrap"],
        capabilities["endpoints"]["app"]["capabilities"],
        capabilities["endpoints"]["app"]["traffic"],
        capabilities["endpoints"]["auth"]["login"],
        capabilities["endpoints"]["auth"]["check"],
        capabilities["endpoints"]["control"]["navigation_goal_candidate"],
        capabilities["endpoints"]["control"]["navigation_plan"],
        capabilities["endpoints"]["ops"]["routecheck_latest"],
        capabilities["endpoints"]["ops"]["algorithm_benchmark_latest"],
        capabilities["endpoints"]["ops"]["runtime_contract"],
    ]
    for spec in key_specs:
        assert spec["response_schema"]
        assert "application/json" in spec["response_content_types"]
