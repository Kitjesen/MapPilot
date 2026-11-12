"""SDK requests exercised through the Gateway's actual FastAPI routes."""

from __future__ import annotations

import io
import json
import urllib.error
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
from fastapi import FastAPI
from fastapi.responses import JSONResponse
from fastapi.testclient import TestClient

from lingtu.sdk import AsyncLingTuClient, LingTuClient


def _transport(client, request, **kwargs):
    response = client.request(
        request.get_method(),
        request.full_url,
        content=request.data,
        headers=dict(request.header_items()),
    )
    stream = io.BytesIO(response.content)
    stream.headers = response.headers
    if response.status_code >= 400:
        raise urllib.error.HTTPError(request.full_url, response.status_code, "Gateway error", response.headers, stream)
    return stream


@pytest.fixture
def maps_api(monkeypatch):
    from gateway.maps import routes

    calls = []
    native = {
        "success": True,
        "accepted": True,
        "map_id": "hall",
        "content_epoch": 12,
        "source_content_epoch": 11,
        "frame_id": "map",
        "regions": [],
        "edits": [],
        "jobs": [],
        "status": {"job_id": "save_1", "state": "QUEUED"},
    }
    monkeypatch.setattr(routes, "_mapd_http_request", lambda _gw, command: calls.append(command) or native)
    monkeypatch.setattr(routes, "resolve_exchange_path", lambda *args, **kwargs: Path("exchange/source.pcd"))
    app = FastAPI()
    routes.register_map_routes(app, SimpleNamespace(clear_map_cloud_cache=Mock()))
    with TestClient(app) as http:
        monkeypatch.setattr("urllib.request.urlopen", lambda request, **kwargs: _transport(http, request, **kwargs))
        yield LingTuClient().map, calls, native


@pytest.mark.parametrize(
    ("method", "args", "kwargs", "command"),
    [
        ("operations", (), {"limit": 4}, {"action": "list_save_map_jobs", "limit": 4}),
        (
            "reoptimize_operation",
            ("save_1",),
            {"request_id": "reopt_1"},
            {"action": "reoptimize_save_map", "job_id": "save_1", "request_id": "reopt_1"},
        ),
        ("process_operation", ("save_1",), {}, {"action": "process_save_map", "job_id": "save_1"}),
        ("no_go", ("hall",), {}, {"action": "get_no_go_regions", "map_id": "hall"}),
        (
            "set_no_go",
            ("hall", {"id": "door", "min": [1, 2, 0], "max": [2, 3, 2]}),
            {"expected_content_epoch": 12},
            {
                "action": "set_no_go_region",
                "map_id": "hall",
                "expected_content_epoch": 12,
                "region": {"id": "door", "min": (1, 2, 0), "max": (2, 3, 2)},
            },
        ),
        (
            "delete_no_go",
            ("hall", "door"),
            {"expected_content_epoch": 12},
            {"action": "delete_no_go_region", "map_id": "hall", "region_id": "door", "expected_content_epoch": 12},
        ),
        (
            "surface",
            ("hall",),
            {"headroom_m": 0.5, "radius_m": 0.3, "max_hole_area_m2": 0.1, "max_slope": 0.4},
            {
                "action": "get_surface_map",
                "map_id": "hall",
                "headroom_m": 0.5,
                "radius_m": 0.3,
                "max_hole_area_m2": 0.1,
                "max_slope": 0.4,
            },
        ),
        (
            "preview_edits",
            ("hall", []),
            {"expected_source_content_epoch": 11},
            {"action": "preview_source_edits", "map_id": "hall", "expected_source_content_epoch": 11, "operations": []},
        ),
        (
            "save_as",
            ("hall", "hall-copy", []),
            {"expected_source_content_epoch": 11},
            {
                "action": "save_edited_source",
                "map_id": "hall",
                "new_map_id": "hall-copy",
                "expected_source_content_epoch": 11,
                "operations": [],
            },
        ),
        (
            "crop",
            ("hall", {"min": [0, 0, 0], "max": [1, 1, 1]}),
            {"invert": True, "voxel_size": 0.1},
            {
                "action": "crop_pcd",
                "map_id": "hall",
                "bounds": {"min": [0, 0, 0], "max": [1, 1, 1]},
                "invert": True,
                "voxel_size": 0.1,
            },
        ),
        (
            "edit_voxels",
            ("hall", 1, 2, 3),
            {"state": "free", "radius_m": 0.4},
            {
                "action": "edit_octomap_voxels",
                "map_id": "hall",
                "center": [1, 2, 3],
                "state": "free",
                "radius_m": 0.4,
                "shape": "sphere",
            },
        ),
        ("voxel_edits", ("hall",), {}, {"action": "get_voxel_edits", "map_id": "hall"}),
        (
            "build_octomap",
            ("hall",),
            {
                "resolution": 0.1,
                "frame_id": "map",
                "source_profile": "lidar",
                "data_source": "lidar",
                "slam_source": "slam",
                "localization_source": "loc",
                "mapping_source": "mapping",
            },
            {
                "action": "build_octomap_artifact",
                "map_id": "hall",
                "resolution": 0.1,
                "frame_id": "map",
                "source_profile": "lidar",
                "data_source": "lidar",
                "slam_source": "slam",
                "localization_source": "loc",
                "mapping_source": "mapping",
            },
        ),
        (
            "import_pcd",
            ("hall", "source.pcd"),
            {"voxel_size": 0.05},
            {
                "action": "import_pcd",
                "map_id": "hall",
                "source_path": str(Path("exchange/source.pcd")),
                "voxel_size": 0.05,
                "bounds": None,
            },
        ),
        ("delete", ("hall",), {}, {"action": "delete_map", "map_id": "hall"}),
        ("build_occupancy", ("hall",), {}, {"action": "build_occupancy_snapshot", "map_id": "hall"}),
        ("mark_zone", ("hall", 1, 2, 3), {"state": "occupied"},
         {"action": "edit_octomap_voxels", "map_id": "hall", "center": [1, 2, 3],
          "state": "occupied", "radius_m": 0.2, "shape": "sphere"}),
    ],
)
def test_map_requests_reach_native_contract(maps_api, method, args, kwargs, command):
    maps, calls, _ = maps_api
    result = getattr(maps, method)(*args, **kwargs)
    assert calls == [command]
    assert result.get("success") is True if isinstance(result, dict) else result.ok


def test_no_go_conflict_preserves_rejection(maps_api):
    maps, _, native = maps_api
    native.update(success=False, accepted=False, reason_code="active_map_conflict")
    result = maps.set_no_go("hall", {"id": "door", "min": [0, 0, 0], "max": [1, 1, 1]}, expected_content_epoch=12)
    assert not result.ok
    assert result.raw["reason_code"] == "active_map_conflict"


@pytest.mark.parametrize("active", [True, False])
def test_plan_preview_keeps_active_map_guard(maps_api, monkeypatch, active):
    from gateway.maps import routes

    maps, calls, native = maps_api
    native["active"] = "hall" if active else "other"
    preview = Mock(return_value={"ok": True, "feasible": True})
    monkeypatch.setattr(routes.ControlCommandService, "preview_plan", preview)
    result = maps.validate_plan("hall", 1, 2, z=3, acceptance_radius_m=0.15)
    assert calls == [{"action": "get_active_map"}]
    assert result["motion_published"] is False
    if active:
        request = preview.call_args.args[0]
        assert request.model_dump() == {"x": 1, "y": 2, "z": 3, "frame_id": "map", "acceptance_radius_m": 0.15}
        assert result["feasible"] is True
    else:
        assert result["success"] is False
        preview.assert_not_called()


@pytest.mark.parametrize("artifact", ["source", "octomap"])
def test_artifact_download_uses_authenticated_atomic_transport(monkeypatch, tmp_path, artifact):
    response = io.BytesIO(b"map artifact")
    response.headers = {"Content-Length": "12"}
    transport = Mock(return_value=response)
    monkeypatch.setattr("urllib.request.urlopen", transport)
    maps = LingTuClient("robot.local", api_key="secret", timeout=3).map
    target = tmp_path / "map.bin"
    target.write_bytes(b"old artifact")
    assert getattr(maps, f"download_{artifact}")("save_1", target) == target
    assert target.read_bytes() == b"map artifact"
    request = transport.call_args.args[0]
    expected = "operations/save_1/source/pcd" if artifact == "source" else "save_1/octomap"
    assert request.full_url.endswith("/api/v1/maps/" + expected)
    assert request.get_header("X-api-key") == "secret"
    assert transport.call_args.kwargs["timeout"] == 3


def test_memory_queries_reach_actual_routes(monkeypatch):
    from gateway.routes import operations

    monkeypatch.setattr(operations.time, "time", lambda: 1000.0)

    store = Mock()
    store.query.return_value = []
    store.query_semantic.return_value = []
    gw = SimpleNamespace(_temporal_store=store, _go2rtc_upstream="http://media")
    app = FastAPI()
    operations.register_operation_routes(app, gw)
    with TestClient(app) as http:
        monkeypatch.setattr("urllib.request.urlopen", lambda request, **kwargs: _transport(http, request, **kwargs))
        observations = LingTuClient().observations
        assert (
            observations.temporal_memory(label="door", since="100", near_x=1, near_y=2, radius=3, limit=4)["count"] == 0
        )
        assert observations.search_memory_embedding([0.1, 0.2], top_k=3, label="door", since="100")["count"] == 0
    store.query.assert_called_once_with(label="door", since_ts=900, near_x=1, near_y=2, radius=3, limit=4)
    store.query_semantic.assert_called_once_with([0.1, 0.2], top_k=3, label="door", since_ts=900)


@pytest.mark.asyncio
async def test_async_save_flags_and_source_operations():
    robot = AsyncLingTuClient()
    sync_map = Mock()
    robot._client._map = sync_map
    await robot.map.save("hall", request_id="save_1", auto_process=False)
    await robot.map.save_and_wait("hall", auto_process=False, wait_for_source=True)
    await robot.map.process_operation("save_1")
    await robot.map.download_source("save_1", "source.pcd")
    sync_map.save.assert_called_once_with("hall", request_id="save_1", auto_process=False)
    sync_map.save_and_wait.assert_called_once_with(
        "hall", request_id=None, timeout=300, poll_interval=1, auto_process=False, wait_for_source=True
    )
    sync_map.process_operation.assert_called_once_with("save_1")
    sync_map.download_source.assert_called_once_with("save_1", "source.pcd")


def test_session_exposes_product_identity(monkeypatch):
    payload = {"mode": "idle", "env": "sim", "product": "nav", "product_session_id": "session-1"}
    monkeypatch.setattr("urllib.request.urlopen", lambda *args, **kwargs: io.BytesIO(json.dumps(payload).encode()))
    session = LingTuClient().system.session()
    assert (session.env, session.product, session.product_session_id) == ("sim", "nav", "session-1")


@pytest.mark.parametrize(
    ("method", "path"),
    [
        ("camera_status", "/api/v1/camera/status"),
        ("media_status", "/api/v1/webrtc/status"),
        ("camera_diagnostics", "/api/v1/camera/diagnostics"),
    ],
)
def test_camera_status_requests(monkeypatch, method, path):
    transport = Mock(return_value=io.BytesIO(b'{"available":false}'))
    monkeypatch.setattr("urllib.request.urlopen", transport)
    assert getattr(LingTuClient().observations, method)()["available"] is False
    assert transport.call_args.args[0].full_url.endswith(path)


@pytest.mark.parametrize(("method", "action", "args"), [
    ("stand_up", "stand", ()), ("sit_down", "sit", ()),
    ("enable_motors", "enable_motors", ()), ("disable_motors", "disable_motors", ()),
    ("switch_profile", "switch_profile", ("walk",)), ("stop_motion", "stop_motion", ()),
    ("clear_emergency_stop", "clear_emergency_stop", ()),
])
def test_system_commands_reach_actual_http_routes(monkeypatch, method, action, args):
    from fastapi.responses import JSONResponse

    from gateway.routes import body, commands

    calls = []
    def change(_gw, request, action):
        calls.append((action, request.model_dump()))
        return JSONResponse({"ok": True, "accepted": True, "execution_confirmed": False})

    monkeypatch.setattr(body, "change", change)
    monkeypatch.setattr(body, "stop_motion", lambda gw, request: change(gw, request, "stop_motion"))
    monkeypatch.setattr(commands, "clear_emergency_stop", lambda gw, request: change(gw, request, "clear_emergency_stop"))
    app = FastAPI()
    body.register_body_routes(app, SimpleNamespace())
    commands.register_command_routes(app, SimpleNamespace())
    with TestClient(app) as http:
        monkeypatch.setattr("urllib.request.urlopen", lambda request, **kwargs: _transport(http, request, **kwargs))
        result = getattr(LingTuClient().system, method)(*args, request_id="body_request", client_id="operator")
    expected = {"request_id": "body_request", "client_id": "operator"}
    if args:
        expected["name"] = "walk"
    assert calls == [(action, expected)]
    assert result.accepted and result.request_id == "body_request"
    assert result.raw["execution_confirmed"] is False


def test_product_switch_retains_durable_receipt(monkeypatch):
    from gateway.routes import product_control

    calls = []
    operation = {"request_id": "switch_1", "state": "running", "result": None}
    def forward(path, body=None):
        calls.append((path, body))
        if body and body.get("expected_product_session_id") == "old":
            return JSONResponse({"message": "session changed"}, status_code=409)
        return JSONResponse(operation if path != "/status" else {"available": True, "current": None})

    monkeypatch.setattr(product_control, "_forward", forward)
    app = FastAPI()
    product_control.register_product_control_routes(app)
    with TestClient(app) as http:
        monkeypatch.setattr("urllib.request.urlopen", lambda request, **kwargs: _transport(http, request, **kwargs))
        system = LingTuClient().system
        assert system.product_status()["available"]
        assert system.stop_product("session_1", request_id="switch_1") == operation
        assert calls[-1] == ("/stop", {"expected_product_session_id": "session_1", "request_id": "switch_1"})
        pose = {"x": 1, "y": 2, "z": 0, "yaw": 0.2}
        assert system.switch_product("nav", expected_product_session_id="session_1", map_name="hall", initial_pose=pose, request_id="switch_1") == operation
        assert system.product_operation("switch_1") == operation
        body = next(body for path, body in calls if path == "/switch")
        assert body == {"product": "nav", "expected_product_session_id": "session_1", "map_name": "hall", "initial_pose": pose, "request_id": "switch_1"}
        assert calls[-1] == ("/operations/switch_1", None)
        assert system.switch_product("map", expected_product_session_id="old") == {"message": "session changed"}


def test_recording_management_reaches_actual_routes(monkeypatch, tmp_path):
    from gateway.routes import recordings

    artifact = tmp_path / "scan.mcap"
    payload = b"recorded\x00bytes"
    artifact.write_bytes(payload)
    manifest = {"session_id": "scan_1", "state": "completed", "children": [
        {"name": "dds", "artifacts": ["nested/scan file.mcap"], "selected_topics": ["/nav/state"]},
    ]}
    service = Mock()
    service.list.return_value = {"sessions": [manifest], "truncated": False, "disk_free": 20, "disk_total": 100}
    service.manifest.return_value = manifest
    service.catalog.return_value = {"topics": ["/nav/state"]}
    service.streams.return_value = {"available": True}
    service.inspect_artifact.return_value = {"topics": [{"topic": "/nav/state", "message_count": 2}]}
    service.series_artifact.return_value = {"points": [[0, 1]]}
    service.scene_artifact.return_value = {"frames": []}
    service.remove.return_value = {"session_id": "scan_1"}
    resolve = Mock(return_value=(artifact, {"path": "nested/scan file.mcap"}))
    monkeypatch.setattr(recordings, "resolve_declared_artifact", resolve)
    monkeypatch.setattr(recordings, "recording_scene_info", lambda *args: {"ok": True, "duration": 4})
    monkeypatch.setattr(recordings, "dds_scene_artifact", lambda *args: artifact)
    frame = Mock(return_value=(b"\xff\xd8JPEG", {}))
    monkeypatch.setattr(recordings, "recording_camera_frame", frame)
    app = FastAPI()
    recordings.register_recording_routes(app, SimpleNamespace(_recording=service))
    with TestClient(app) as http:
        monkeypatch.setattr("urllib.request.urlopen", lambda request, **kwargs: _transport(http, request, **kwargs))
        observations = LingTuClient().observations
        assert observations.recordings(limit=3)["sessions"] == [{"session_id": "scan_1", "state": "completed"}]
        service.list.assert_called_once_with(limit=3)
        assert observations.recording("scan_1")["session"]["artifacts"][0]["path"] == "nested/scan file.mcap"
        assert observations.recording_catalog()["topics"] == ["/nav/state"]
        assert observations.recording_streams()["available"]
        target = tmp_path / "download.mcap"
        assert observations.download_recording_file("scan_1", "nested/scan file.mcap", target) == target
        assert target.read_bytes() == payload
        resolve.assert_called_with(service, session_id="scan_1", artifact_name="nested/scan file.mcap")
        archive = observations.download_recording_archive("scan_1", tmp_path / "scan.tar")
        import tarfile
        with tarfile.open(archive) as saved:
            assert saved.extractfile("scan_1/nested/scan file.mcap").read() == payload
        assert observations.recording_summary("scan_1")["summary"]["missing_topics"] == []
        assert observations.validate_recording("scan_1")["ok"]
        service.inspect_artifact.assert_called_with(artifact, validate=True)
        assert observations.recording_series("scan_1", "/nav/state", field="vx", end=2, max_points=5)["series"]["points"] == [[0, 1]]
        service.series_artifact.assert_called_with(artifact, topic="/nav/state", field="vx", start=0.0, end=2, max_points=5)
        assert observations.recording_scene_info("scan_1")["duration"] == 4
        assert observations.recording_scene("scan_1", path_topics=["/nav/path", "/local/path"])["ok"]
        assert service.scene_artifact.call_args.kwargs["path_topics"] == ("/nav/path", "/local/path")
        assert observations.recording_camera_frame("scan_1", 1.5, 2, camera_id="rear", width=640) == b"\xff\xd8JPEG"
        frame.assert_called_once_with(service, "scan_1", t=1.5, segment=2, camera_id="rear", width=640)
        frame.return_value = None
        assert observations.recording_camera_frame("scan_1", 1.5, 2) == b""
        assert observations.delete_recording("scan_1")["ok"]
        service.remove.assert_called_once_with(session_id="scan_1")


@pytest.mark.parametrize("legacy", [False, True])
def test_recording_stop_preserves_session_and_owner_rejection(monkeypatch, legacy):
    from gateway.routes.operations import register_operation_routes
    from gateway.services.recording import NativeRecordingError, RecordingSnapshot

    calls = []
    def stop(**options):
        calls.append(options)
        if options.get("expected_session_id") == "older-session":
            raise NativeRecordingError("recording_session_mismatch", "session changed", status_code=409)
        return RecordingSnapshot(available=True, healthy=True, state="completed", session_id="active-session")

    app = FastAPI()
    register_operation_routes(app, SimpleNamespace(_go2rtc_upstream="", _recording=SimpleNamespace(stop=stop)))
    with TestClient(app) as http:
        monkeypatch.setattr("urllib.request.urlopen", lambda request, **kwargs: _transport(http, request, **kwargs))
        robot = LingTuClient()
        method = robot.recording_stop if legacy else robot.observations.stop_recording
        assert method().raw["session_id"] == "active-session"
        assert method(expected_session_id="active-session").raw["state"] == "completed"
        rejected = method(expected_session_id="older-session")

    assert calls == [{}, {"expected_session_id": "active-session"}, {"expected_session_id": "older-session"}]
    assert not rejected.ok
    assert rejected.raw["error"] == "recording_session_mismatch"


def test_inspection_edit_and_evidence_reach_actual_schemas(monkeypatch, tmp_path):
    from gateway.routes import inspection

    stored = {"id": "route-1", "name": "Hall", "map_id": "hall", "revision": 2, "map_content_epoch": 7,
              "points": [{"id": "p1", "x": 1, "y": 2, "z": 0}]}
    store = Mock(side_effect=lambda _gw, action, *args: args[0] if action == "put" else stored)
    monkeypatch.setattr(inspection, "_run_store_operation", store)
    monkeypatch.setattr(inspection, "_map_content_epoch", lambda *args: 7)
    timeline = Mock()
    timeline.list_tasks.return_value = {"retention": "durable_gateway_projection", "count": 0, "tasks": [], "ts": 1}
    monkeypatch.setattr(inspection, "ensure_inspection_task_timeline", lambda *args: timeline)
    evidence = object()
    review = Mock(return_value=evidence)
    monkeypatch.setattr(inspection, "InspectionEvidenceStore", lambda *args: SimpleNamespace(review=review))
    monkeypatch.setattr(inspection, "_evidence_root", lambda: tmp_path)
    monkeypatch.setattr(inspection, "_evidence_result", lambda *args: evidence)
    monkeypatch.setattr(inspection, "_evidence_summary", lambda item: {"evidence_id": "evidence-1", "analysis": {"outcome": "original"}})
    monkeypatch.setattr(inspection, "_list_verified_evidence", lambda limit: {"ok": True, "count": limit})
    artifact = Mock(return_value=(b"verified\x00artifact", "application/octet-stream"))
    monkeypatch.setattr(inspection, "_read_evidence_artifact", artifact)
    app = FastAPI()
    inspection.register_inspection_routes(app, SimpleNamespace())
    with TestClient(app) as http:
        monkeypatch.setattr("urllib.request.urlopen", lambda request, **kwargs: _transport(http, request, **kwargs))
        tasks = LingTuClient().tasks
        assert tasks.save_inspection_route(stored)["route"]["revision"] == 2
        assert store.call_args.args[2]["map_content_epoch"] == 7
        assert tasks.inspection_route("route-1", map_id="hall")["route"]["id"] == "route-1"
        assert tasks.delete_inspection_route("route-1", map_id="hall")["ok"]
        assert store.call_args.args[1:] == ("delete", "hall", "route-1")
        assert tasks.inspection_tasks(map_id="hall", route_id="route-1", include_terminal=True, limit=4)["tasks"] == []
        timeline.list_tasks.assert_called_once_with(map_id="hall", route_id="route-1", include_terminal=True, limit=4)
        assert tasks.list_inspection_evidence(limit=3)["count"] == 3
        assert tasks.inspection_evidence("evidence-1")["evidence"]["evidence_id"] == "evidence-1"
        assert tasks.review_inspection_evidence("evidence-1", "anomaly", note="inspect valve")["evidence"]["analysis"]["outcome"] == "original"
        review.assert_called_once_with("evidence-1", verdict="anomaly", note="inspect valve")
        assert "detail" in tasks.review_inspection_evidence("evidence-1", "invalid")
        assert "detail" in tasks.save_inspection_route({**stored, "revision": 0})
        target = tasks.download_inspection_artifact("evidence-1", "image", tmp_path / "image.bin")
        assert target.read_bytes() == b"verified\x00artifact"
        measured = tasks.download_inspection_measurement("evidence-1", "thermal-1", tmp_path / "measure.bin")
        assert measured.read_bytes() == b"verified\x00artifact"
        artifact.assert_called_with(evidence, "measurement", sensor_id="thermal-1")


def test_goal_reconciliation_and_motion_resume_use_native_evidence(monkeypatch):
    from gateway.gateway_module import GatewayModule
    from gateway.navigation.routes import register_navigation_routes
    from runtime.msgs import NavigationCommandKind

    gateway = GatewayModule()
    gateway.setup()
    controls = SimpleNamespace(resume_autonomy_with_receipt=Mock())
    controls.resume_autonomy_with_receipt.side_effect = lambda reason, request_id: {
        "accepted": True, "kind": int(NavigationCommandKind.RESUME_AUTONOMY),
        "request_id": request_id, "task_id": "", "reason": "autonomy_already_ready",
        "endpoint_timestamp_s": 123.5,
    }
    gateway.on_system_modules({"nav.commands": controls})
    gateway._navigation_goal_status_by_request["goal_1"] = {
        "phase": "EXECUTING", "request_id": "goal_1", "task_id": "native_task",
    }
    app = FastAPI()
    register_navigation_routes(app, gateway)
    with TestClient(app) as http:
        monkeypatch.setattr("urllib.request.urlopen", lambda request, **kwargs: _transport(http, request, **kwargs))
        robot = LingTuClient()
        assert robot.navigation.goal_status("goal_1")["status"]["task_id"] == "native_task"
        assert robot.navigation.goal_status("unknown")["found"] is False
        result = robot.system.resume_motion(request_id="resume_1")
        assert result.accepted
        assert result.raw["previous_motion_restored"] is False
        assert result.raw["native_request_id"] == "resume_1"
        controls.resume_autonomy_with_receipt.assert_called_once_with(reason="operator_resume", request_id="resume_1")
        assert gateway._lease.acquire("other", 30)
        refused = robot.system.resume_motion(request_id="resume_2")
        assert not refused.accepted
        assert refused.raw["error"] == "control_lease"
        assert controls.resume_autonomy_with_receipt.call_count == 1


def test_recording_topics_reach_actual_start_schema(monkeypatch):
    from gateway.routes import operations

    service = Mock()
    service.start.return_value = SimpleNamespace(state="recording", session_id="scan_1")
    app = FastAPI()
    operations.register_operation_routes(app, SimpleNamespace(_recording=service, _go2rtc_upstream="http://media"))
    with TestClient(app) as http:
        monkeypatch.setattr("urllib.request.urlopen", lambda request, **kwargs: _transport(http, request, **kwargs))
        result = LingTuClient().recording_start("sdk", duration=60, topics=["/nav/state", "/cloud"])
        assert result.raw["topics"] == ["/nav/state", "/cloud"]
        service.start.assert_called_once_with(duration=60, prefix="sdk", capture_profile="sensors", topics=["/nav/state", "/cloud"], task_id=None, camera=False, minimum_free_gib=5)


@pytest.mark.asyncio
async def test_async_workflows_preserve_optional_fields_and_results():
    robot = AsyncLingTuClient()
    sync = robot._client
    sync.navigation.goal_status = Mock(return_value={"found": False})
    assert await robot.navigation.goal_status("missing") == {"found": False}
    sync.navigation.goal_status.assert_called_once_with("missing")
    sync.system.resume_motion = Mock(return_value={"accepted": True})
    assert await robot.system.resume_motion(request_id="resume_1", client_id="operator") == {"accepted": True}
    sync.system.resume_motion.assert_called_once_with(request_id="resume_1", client_id="operator")
    sync.system.stop_product = Mock(return_value={"state": "running"})
    assert await robot.system.stop_product("session_1", request_id="stop_1") == {"state": "running"}
    sync.system.stop_product.assert_called_once_with("session_1", request_id="stop_1")
    operation = {"state": "running", "request_id": "switch_1"}
    sync.system.switch_product = Mock(return_value=operation)
    pose = {"x": 1.0, "y": 2.0, "z": 0.0, "yaw": 0.5}
    assert await robot.system.switch_product("nav", expected_product_session_id="session_1", map_name="hall", initial_pose=pose, request_id="switch_1") is operation
    sync.system.switch_product.assert_called_once_with("nav", expected_product_session_id="session_1", map_name="hall", initial_pose=pose, request_id="switch_1")
    scene = {"scene": {"frames": []}}
    sync.observations.recording_scene = Mock(return_value=scene)
    assert await robot.observations.recording_scene("scan_1", path_topics=["/global", "/local"], end=3) is scene
    sync.observations.recording_scene.assert_called_once_with("scan_1", start=0.0, end=3, max_frames=32, max_points=2048, max_poses=2000, cloud_topic=None, pose_topic=None, path_topics=["/global", "/local"])
    payload = b"image\x00data"
    sync.observations.recording_camera_frame = Mock(return_value=payload)
    assert await robot.observations.recording_camera_frame("scan_1", 1.5, 2, camera_id="rear", width=640) == payload
    sync.observations.recording_camera_frame.assert_called_once_with("scan_1", 1.5, 2, camera_id="rear", width=640)
    sync.tasks.review_inspection_evidence = Mock(return_value={"ok": True})
    assert (await robot.tasks.review_inspection_evidence("evidence-1", "normal", note="checked"))["ok"]
    sync.tasks.review_inspection_evidence.assert_called_once_with("evidence-1", "normal", note="checked")
