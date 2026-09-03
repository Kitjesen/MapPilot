from __future__ import annotations

import asyncio
import json
import os
import shutil
import struct
import subprocess
import sys
import uuid
from pathlib import Path

import pytest

pytest.importorskip("fastapi")


def _endpoint(gateway, path: str):
    endpoint = next(route.endpoint for route in gateway._app.routes if route.path == path)
    request_model_name = {
        "/api/v1/memory/temporal/semantic": "TemporalSemanticRequest",
    }.get(path)
    if request_model_name is None:
        return endpoint

    from gateway import schemas

    request_model = getattr(schemas, request_model_name)

    async def validated_endpoint(body):
        return await endpoint(request_model.model_validate(body))

    return validated_endpoint


def _payload(response_or_payload):
    if hasattr(response_or_payload, "body"):
        return json.loads(response_or_payload.body)
    return response_or_payload


async def _stream_body(response) -> bytes:
    chunks = bytearray()
    async for chunk in response.body_iterator:
        chunks.extend(chunk.encode() if isinstance(chunk, str) else chunk)
    return bytes(chunks)


def _field_run_plan(profile: str):
    from lingtu.assembly.compiler import compile_run_plan
    from lingtu.assembly.products import resolve_product_host_runtime

    resolved = resolve_product_host_runtime(profile, "real", robot="unitree/go2")
    return compile_run_plan(
        resolved.product,
        resolved.env,
        robot="unitree/go2",
    )


def _field_guard_plan_stub(product: str):
    """Provide only the immutable Product identity needed by map-guard tests."""
    from types import SimpleNamespace

    return SimpleNamespace(
        product=product,
        process_control="systemd",
        env="real",
        robot="doso/thunder_v4",
        host_config={},
    )


def _activate_map(map_root: Path, name: str) -> None:
    map_root.mkdir(parents=True, exist_ok=True)
    (map_root / "active_map.txt").write_text(f"{name}\n", encoding="utf-8")


def _fake_octomap_converter_command(tmp_path: Path) -> str:
    script = tmp_path / "fake_octomap_converter.py"
    script.write_text(
        "from __future__ import annotations\n"
        "import argparse\n"
        "from pathlib import Path\n"
        "parser = argparse.ArgumentParser()\n"
        "parser.add_argument('--input', required=True)\n"
        "parser.add_argument('--output', required=True)\n"
        "parser.add_argument('--resolution', required=True)\n"
        "parser.add_argument('--free-layers-above', required=True)\n"
        "parser.add_argument('--free-dilation-cells', required=True)\n"
        "parser.add_argument('--frame', required=True)\n"
        "args = parser.parse_args()\n"
        "Path(args.output).write_bytes(Path(args.input).read_bytes())\n",
        encoding="utf-8",
    )
    return subprocess.list2cmdline(
        [
            sys.executable,
            str(script),
            "--input",
            "{input}",
            "--output",
            "{output}",
            "--resolution",
            "{resolution}",
            "--free-layers-above",
            "{free_layers_above}",
            "--free-dilation-cells",
            "{free_dilation_cells}",
            "--frame",
            "{frame}",
        ]
    )


class _FakeRelocalizationService:
    def __init__(
        self,
        *,
        global_result=None,
        saved_result=None,
        track_result=None,
    ):
        self.global_result = global_result
        self.saved_result = saved_result
        self.track_result = track_result
        self.global_calls = []
        self.saved_calls = []
        self.track_calls = []

    def trigger_global_relocalize(self, *, timeout_s: float = 10.0):
        self.global_calls.append(timeout_s)
        return self.global_result

    def relocalize_saved_map(self, map_id, x, y, yaw, *, timeout_s: float = 30.0):
        self.saved_calls.append((map_id, x, y, yaw, timeout_s))
        return self.saved_result

    def track_against_map(self, *, timeout_s: float = 10.0):
        self.track_calls.append(timeout_s)
        return self.track_result


def _seed_map_artifacts(map_dir: Path) -> None:
    """Create minimal valid map artifacts for OctoPlanner3D."""
    pcd_content = (
        "VERSION .7\n"
        + "FIELDS x y z\n"
        + "SIZE 4 4 4\n"
        + "TYPE F F F\n"
        + "COUNT 1 1 1\n"
        + "WIDTH 1\n"
        + "HEIGHT 1\n"
        + "VIEWPOINT 0 0 0 1 0 0 0\n"
        + "POINTS 1\n"
        + "DATA ascii\n"
        + "0.0 0.0 0.0\n"
    )
    map_path = map_dir / "map.pcd"
    map_path.write_text(pcd_content, encoding="ascii")
    octomap_path = map_dir / "octomap.ot"
    octomap_path.write_bytes(b"gateway-test-octomap")
    (map_dir / "metadata.json").write_text(
        json.dumps(
            {
                "schema_version": "lingtu.saved_map_artifacts.v1",
                "source_profile": "thunder",
                "data_source": "field",
                "slam_source": "fastlio2",
                "localization_source": "fastlio2",
                "mapping_source": "fastlio2",
                "frame_id": "map",
                "created_at": "2026-05-25T00:00:00Z",
                "artifacts": {
                    "map_pcd": {
                        "path": "map.pcd",
                        "source_profile": "thunder",
                        "data_source": "field",
                        "slam_source": "fastlio2",
                        "frame_id": "map",
                        "point_count": 1,
                    },
                    "octomap": {
                        "path": "octomap.ot",
                        "source_profile": "thunder",
                        "data_source": "field",
                        "frame_id": "map",
                    },
                },
            }
        )
    )


def _seed_octomap_only_artifacts(map_dir: Path) -> None:
    pcd_content = (
        "VERSION .7\n"
        + "FIELDS x y z\n"
        + "SIZE 4 4 4\n"
        + "TYPE F F F\n"
        + "COUNT 1 1 1\n"
        + "WIDTH 1\n"
        + "HEIGHT 1\n"
        + "VIEWPOINT 0 0 0 1 0 0 0\n"
        + "POINTS 1\n"
        + "DATA ascii\n"
        + "0.0 0.0 0.0\n"
    )
    map_path = map_dir / "map.pcd"
    map_path.write_text(pcd_content, encoding="ascii")
    octomap_path = map_dir / "octomap.ot"
    octomap_path.write_bytes(b"gateway-test-octomap")
    (map_dir / "metadata.json").write_text(
        json.dumps(
            {
                "schema_version": "lingtu.saved_map_artifacts.v1",
                "source_profile": "thunder",
                "data_source": "field",
                "slam_source": "fastlio2",
                "localization_source": "fastlio2",
                "mapping_source": "fastlio2",
                "frame_id": "map",
                "created_at": "2026-05-25T00:00:00Z",
                "artifacts": {
                    "map_pcd": {
                        "path": "map.pcd",
                        "source_profile": "thunder",
                        "data_source": "field",
                        "slam_source": "fastlio2",
                        "frame_id": "map",
                        "point_count": 1,
                    },
                    "octomap": {
                        "path": "octomap.ot",
                        "source_profile": "thunder",
                        "data_source": "field",
                        "frame_id": "map",
                    },
                },
            }
        )
    )


def _seed_pcd_only_metadata(map_dir: Path) -> None:
    pcd_content = (
        "VERSION .7\n"
        + "FIELDS x y z\n"
        + "SIZE 4 4 4\n"
        + "TYPE F F F\n"
        + "COUNT 1 1 1\n"
        + "WIDTH 1\n"
        + "HEIGHT 1\n"
        + "VIEWPOINT 0 0 0 1 0 0 0\n"
        + "POINTS 1\n"
        + "DATA ascii\n"
        + "0.0 0.0 0.0\n"
    )
    map_path = map_dir / "map.pcd"
    map_path.write_text(pcd_content, encoding="ascii")
    (map_dir / "metadata.json").write_text(
        json.dumps(
            {
                "schema_version": "lingtu.saved_map_artifacts.v1",
                "source_profile": "thunder",
                "data_source": "field",
                "slam_source": "fastlio2",
                "localization_source": "fastlio2",
                "mapping_source": "fastlio2",
                "frame_id": "map",
                "created_at": "2026-05-25T00:00:00Z",
                "artifacts": {
                    "map_pcd": {
                        "path": "map.pcd",
                        "source_profile": "thunder",
                        "data_source": "field",
                        "slam_source": "fastlio2",
                        "frame_id": "map",
                        "point_count": 1,
                    },
                },
            }
        ),
        encoding="utf-8",
    )


def _seed_ready_navigation(gateway):
    with gateway._state_lock:
        gateway._odom = {"x": 0.0, "y": 0.0, "z": 0.0}
        gateway._localization_status = {
            "backend": "native_dds",
            "state": "TRACKING",
            "confidence": 0.9,
            "health_source": "slam_runtime",
            "pose_fresh": True,
            "odom_age_ms": 100.0,
            "cloud_age_ms": 100.0,
            "map_cloud_fresh": True,
            "localizer_health": "RECOVERED",
            "recovery_signal": "NONE",
        }


class _JsonRequest:
    def __init__(self, payload: dict):
        self._payload = payload

    async def json(self):
        return self._payload


class _FakeMapClient:
    def __init__(self):
        self.calls: list[dict] = []

    def service(self, action: str, **arguments):
        command = {"action": action, **arguments}
        self.calls.append(command)
        return {
            "action": action,
            "success": True,
            "map_id": arguments.get("map_id"),
            "active": "demo" if action in {"get_active", "get_active_map"} else None,
        }


class _FilesystemMapdClient:
    """Stateless mapd test transport backed by an isolated fixture directory."""

    def __init__(self, root: Path):
        self.root = root
        self.commands: list[dict] = []

    def service(self, action: str, **arguments):
        command = {"action": action, **arguments}
        self.commands.append(command)
        name = str(arguments.get("map_id") or "")
        if action == "list_maps":
            active = self._active()
            return {
                "action": action,
                "success": True,
                "active": active,
                "maps": [self._map_summary(path, active) for path in self._map_dirs()],
            }
        if action in {"get_active", "get_active_map"}:
            return {"action": action, "success": True, "active": self._active()}
        if action == "get_record":
            map_dir = self._map_dir(name)
            if map_dir is None:
                return self._failure(action, "map_not_found", f"map not found: {name}")
            return {
                "action": action,
                "success": True,
                "record": {"map_id": name, "frame_id": self._map_frame_id(map_dir)},
            }
        if action == "get_bundle":
            legacy_action = "get_map_bundle"
        else:
            legacy_action = action
        request = type(
            "Request",
            (),
            {"to_mapping": lambda _self: {"action": legacy_action, "name": name, **arguments}},
        )()
        return self.execute(request)

    def execute(self, request):
        command = request.to_mapping()
        self.commands.append(command)
        action = str(command.get("action") or "")
        name = str(command.get("name") or command.get("map_id") or "")
        if action == "list":
            active = self._active()
            maps = [self._map_summary(path, active) for path in self._map_dirs()]
            return {
                "action": action,
                "success": True,
                "active": active,
                "map_dir": str(self.root),
                "maps": maps,
            }
        if action in {"get_active", "get_active_map"}:
            return {"action": action, "success": True, "active": self._active()}
        if action == "set_active":
            map_dir = self._map_dir(name)
            if map_dir is None:
                return self._failure(action, "map_not_found", f"map not found: {name}")
            self.root.mkdir(parents=True, exist_ok=True)
            (self.root / "active_map.txt").write_text(f"{name}\n", encoding="utf-8")
            return {
                "action": action,
                "success": True,
                "active": name,
                "pcd": str(map_dir / "map.pcd") if (map_dir / "map.pcd").is_file() else "",
                "octomap": self._artifact_path(map_dir, "navigation_safety_3d"),
                "occupancy": self._artifact_path(map_dir, "path_planning_2d"),
            }
        if action == "get_map_bundle":
            map_dir = self._map_dir(name)
            if map_dir is None:
                return self._failure(action, "map_not_found", f"map not found: {name}")
            capability = str(command.get("capability") or "")
            artifact = self._artifact_path(map_dir, capability)
            if not artifact:
                return self._failure(
                    action,
                    "missing_capability",
                    f"map {name} has no artifact for {capability}",
                )
            path = Path(artifact)
            artifact_type = {
                "source_pointcloud": "POINTCLOUD",
                "path_planning": "OCCUPANCY_2D",
                "path_planning_2d": "OCCUPANCY_2D",
                "navigation_safety": "OCTOMAP_3D",
                "navigation_safety_3d": "OCTOMAP_3D",
            }.get(capability, "UNKNOWN")
            return {
                "action": action,
                "success": True,
                "map_id": name,
                "map_dir": str(map_dir),
                "capability": capability,
                "artifact": {"type": artifact_type, "uri": path.name},
            }
        if action == "validate_artifacts":
            from diagnostics.field.field_check import validate_map

            map_dir = self._map_dir(name)
            if map_dir is None:
                return self._failure(action, "map_not_found", f"map not found: {name}")
            gate = validate_map(
                name,
                require_octomap=bool(command.get("require_octomap", False)),
                require_occupancy=bool(command.get("require_occupancy", False)),
                expected_data_source=command.get("expected_data_source"),
                expected_source_profile=command.get("expected_source_profile"),
                expected_frame_id=command.get("expected_frame_id"),
            )
            return {"action": action, "success": True, "gate": gate}
        if action == "get_map_points":
            map_dir = self._map_dir(name)
            if map_dir is None:
                return self._failure(action, "map_not_found", f"map not found: {name}")
            points = self._read_points(map_dir / "map.pcd")
            limit = max(0, int(command.get("max_points") or 0))
            if limit:
                points = points[:limit]
            return {
                "action": action,
                "success": True,
                "map_id": name,
                "content_epoch": 1,
                "frame_id": self._map_frame_id(map_dir),
                "returned": len(points),
                "points": points,
            }
        return self._failure(action, "unsupported_test_action", f"unsupported action: {action}")

    def _active(self) -> str:
        state = self.root / "active_map.txt"
        if not state.is_file():
            return ""
        name = state.read_text(encoding="utf-8").strip()
        return name if self._map_dir(name) is not None else ""

    def _map_dir(self, name: str) -> Path | None:
        if not name or Path(name).name != name or name.startswith("."):
            return None
        candidate = self.root / name
        return candidate if candidate.is_dir() else None

    def _map_dirs(self) -> list[Path]:
        if not self.root.is_dir():
            return []
        return sorted(
            path
            for path in self.root.iterdir()
            if path.is_dir() and not path.is_symlink() and self._map_dir(path.name) is not None
        )

    def _map_summary(self, map_dir: Path, active: str) -> dict:
        metadata = {}
        metadata_path = map_dir / "metadata.json"
        if metadata_path.is_file():
            try:
                metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
            except (OSError, ValueError):
                metadata = {}
        has_pcd = (map_dir / "map.pcd").is_file()
        has_octomap = bool(self._artifact_path(map_dir, "navigation_safety_3d"))
        return {
            "name": map_dir.name,
            "has_pcd": has_pcd,
            "has_occupancy": (map_dir / "occupancy.npz").is_file(),
            "has_octomap": has_octomap,
            "can_activate": has_pcd and has_octomap and metadata_path.is_file(),
            "state": metadata.get("state") or ("READY" if has_octomap else "STALE"),
            "is_active": map_dir.name == active,
            "patch_count": 0,
        }

    @staticmethod
    def _map_frame_id(map_dir: Path) -> str:
        metadata_path = map_dir / "metadata.json"
        if metadata_path.is_file():
            try:
                metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
                frame_id = str(metadata.get("frame_id") or "").strip().lstrip("/")
                if frame_id:
                    return frame_id
            except (OSError, ValueError):
                pass
        return "map"

    @staticmethod
    def _artifact_path(map_dir: Path, capability: str) -> str:
        candidates = {
            "source_pointcloud": ("map.pcd",),
            "path_planning": ("occupancy.npz",),
            "path_planning_2d": ("occupancy.npz",),
            "navigation_safety": ("octomap.ot", "octomap.bt"),
            "navigation_safety_3d": ("octomap.ot", "octomap.bt"),
        }.get(capability, ())
        for filename in candidates:
            path = map_dir / filename
            if path.is_file():
                return str(path)
        return ""

    @staticmethod
    def _read_points(path: Path) -> list[list[float]]:
        if not path.is_file():
            return []
        payload = path.read_bytes()
        marker = b"DATA binary\n"
        if marker in payload:
            body = payload.split(marker, 1)[1]
            return [list(struct.unpack_from("<fff", body, offset)) for offset in range(0, len(body) - 11, 12)]
        marker = b"DATA ascii\n"
        if marker in payload:
            rows = []
            for line in payload.split(marker, 1)[1].decode("ascii", errors="ignore").splitlines():
                values = line.split()
                if len(values) >= 3:
                    rows.append([float(values[0]), float(values[1]), float(values[2])])
            return rows
        return []

    @staticmethod
    def _failure(action: str, reason_code: str, message: str) -> dict:
        return {
            "action": action,
            "success": False,
            "reason_code": reason_code,
            "message": message,
        }


def _attach_test_map_client(gateway, map_root: Path) -> _FilesystemMapdClient:
    service = _FilesystemMapdClient(map_root)
    gateway._map_client = service
    return service


def _write_binary_xyz_pcd(path: Path) -> None:
    header = (
        b"# .PCD v0.7\n"
        b"VERSION 0.7\n"
        b"FIELDS x y z\n"
        b"SIZE 4 4 4\n"
        b"TYPE F F F\n"
        b"COUNT 1 1 1\n"
        b"WIDTH 2\n"
        b"HEIGHT 1\n"
        b"POINTS 2\n"
        b"DATA binary\n"
    )
    path.write_bytes(header + struct.pack("<ffffff", 1, 2, 3, 4, 5, 6))


def test_auth_routes_validate_response_contracts(monkeypatch):
    from gateway import auth
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import AuthCheckResponse, AuthLoginRequest, AuthLoginResponse

    monkeypatch.setattr(auth, "_get_configured_key", lambda: None)

    gateway = GatewayModule()
    gateway.setup()

    check_payload = asyncio.run(_endpoint(gateway, "/api/v1/auth/check")())
    login_response = asyncio.run(_endpoint(gateway, "/api/v1/auth/login")(AuthLoginRequest(key="")))

    check = AuthCheckResponse.model_validate(check_payload)
    login = AuthLoginResponse.model_validate(_payload(login_response))

    assert check.auth_required is False
    assert login.ok is True
    assert login.message == "\u8ba4\u8bc1\u672a\u542f\u7528"


def test_validation_error_handler_serializes_invalid_json_body(monkeypatch):
    from fastapi.exceptions import RequestValidationError

    from gateway import auth
    from gateway.gateway_module import GatewayModule

    monkeypatch.setattr(auth, "_get_configured_key", lambda: None)

    gateway = GatewayModule()
    gateway.setup()

    handler = gateway._app.exception_handlers[RequestValidationError]
    response = asyncio.run(
        handler(
            None,
            RequestValidationError(
                [
                    {
                        "type": "json_invalid",
                        "loc": ("body", 0),
                        "msg": "JSON decode error",
                        "input": b"{key:bad}",
                    }
                ]
            ),
        )
    )

    assert response.status_code == 422
    payload = json.loads(response.body)
    assert payload["error"] == "validation_error"
    assert isinstance(payload["detail"], list)


def test_auth_login_invalid_key_preserves_legacy_message(monkeypatch):
    from gateway import auth
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import AuthLoginRequest, AuthLoginResponse

    monkeypatch.setattr(auth, "_get_configured_key", lambda: "secret")

    gateway = GatewayModule()
    gateway.setup()

    login_response = asyncio.run(_endpoint(gateway, "/api/v1/auth/login")(AuthLoginRequest(key="bad")))
    login = AuthLoginResponse.model_validate(_payload(login_response))

    assert login_response.status_code == 403
    assert login.ok is False
    assert login.message == "Key \u65e0\u6548"


def test_lease_route_validates_success_and_conflict_payloads():
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import GatewayErrorResponse, LeaseRequest, LeaseResponse

    gateway = GatewayModule()
    gateway.setup()
    post_lease = _endpoint(gateway, "/api/v1/lease")

    acquired_payload = asyncio.run(post_lease(LeaseRequest(action="acquire", client_id="web", ttl=30.0)))
    conflict_response = asyncio.run(post_lease(LeaseRequest(action="acquire", client_id="phone", ttl=30.0)))
    released_payload = asyncio.run(post_lease(LeaseRequest(action="release", client_id="web", ttl=30.0)))

    acquired = LeaseResponse.model_validate(acquired_payload)
    conflict_payload = _payload(conflict_response)
    conflict = GatewayErrorResponse.model_validate(conflict_payload)
    released = LeaseResponse.model_validate(released_payload)

    assert acquired.schema_version == 1
    assert acquired.ok is True
    assert acquired.status == "acquired"
    assert acquired.holder == "web"
    assert acquired.active is True
    assert acquired.command.name == "lease"
    assert conflict.schema_version == 1
    assert conflict.ok is False
    assert conflict.error == "lease_conflict"
    assert conflict.command is not None
    assert conflict.command.name == "lease"
    assert conflict.command.accepted is False
    assert released.status == "released"
    assert released.active is False


def test_session_routes_validate_idle_contracts():
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import SessionResponse

    gateway = GatewayModule()
    gateway.setup()
    gateway._session_detect_current_mode = lambda: ("idle", None)

    session_payload = asyncio.run(_endpoint(gateway, "/api/v1/session")())

    session = SessionResponse.model_validate(session_payload)

    assert session.mode == "idle"
    assert session.explorer_available is False
    assert session.explorer_unavailable_reason == "explorer_backend_not_running"
    assert session.explorer_required_product == "explore"


def test_session_snapshot_uses_mapd_bundles_for_navigation_readiness():
    from gateway.gateway_module import GatewayModule

    class FakeMapClient:
        def __init__(self):
            self.bundle_calls: list[tuple[str, str]] = []

        def service(self, action, **arguments):
            if action in {"get_active", "get_active_map"}:
                return self.get_active_map()
            if action == "get_bundle":
                return self.get_map_bundle(
                    str(arguments.get("map_id") or ""),
                    str(arguments.get("capability") or ""),
                )
            if action == "list_maps":
                return {
                    "success": True,
                    "active": "native_active",
                    "maps": [
                        {
                            "name": "native_active",
                            "can_activate": True,
                            "has_pcd": True,
                            "has_octomap": True,
                        }
                    ],
                }
            return {"success": False, "reason_code": "unsupported_test_action"}

        def get_active_map(self):
            return {"success": True, "active": "native_active"}

        def get_map_bundle(self, name: str, capability: str):
            self.bundle_calls.append((name, capability))
            if name != "native_active":
                return {"success": False, "reason_code": "wrong_map"}
            if capability not in {"source_pointcloud", "navigation_safety_3d"}:
                return {"success": False, "reason_code": "missing_capability"}
            return {
                "success": True,
                "map_id": name,
                "capability": capability,
                "artifact": {
                    "type": "POINTCLOUD" if capability == "source_pointcloud" else "OCTOMAP_3D",
                    "uri": f"{capability}.artifact",
                },
            }

    gateway = GatewayModule()
    gateway.setup()
    maps = FakeMapClient()
    gateway._map_client = maps
    _seed_ready_navigation(gateway)

    snapshot = gateway._session_snapshot()

    assert snapshot["saved_active_map"] == "native_active"
    assert snapshot["active_map"] is None
    assert snapshot["map_has_pcd"] is True
    assert snapshot["map_has_octomap"] is True
    assert snapshot["can_activate"] is True
    assert "can_start_navigating" not in snapshot
    assert ("native_active", "source_pointcloud") in maps.bundle_calls
    assert ("native_active", "navigation_safety_3d") in maps.bundle_calls


def test_session_snapshot_does_not_guess_navigation_readiness_without_mapd(
    monkeypatch,
):
    import gateway.services.session_view as session_view
    from gateway.gateway_module import GatewayModule

    gateway = GatewayModule()
    gateway.setup()
    gateway._session_active_map_name = lambda: "orphaned_disk_map"
    _seed_ready_navigation(gateway)

    def fail_file_map_lookup(*_args, **_kwargs):
        raise AssertionError("Gateway must not infer map readiness from files")

    monkeypatch.setattr(
        session_view,
        "map_dir_for",
        fail_file_map_lookup,
        raising=False,
    )

    snapshot = gateway._session_snapshot()

    assert snapshot["saved_active_map"] == "orphaned_disk_map"
    assert snapshot["map_has_pcd"] is False
    assert snapshot["map_has_octomap"] is False
    assert "can_start_navigating" not in snapshot
    assert "error" not in snapshot


@pytest.mark.parametrize(
    "maps",
    [
        [],
        [{"name": "native_active"}],
        [{"name": "native_active", "can_activate": "yes"}],
    ],
)
def test_session_snapshot_does_not_claim_invalid_map_activation(maps):
    from gateway.gateway_module import GatewayModule

    class FakeMapClient:
        def service(self, action, **_arguments):
            if action in {"get_active", "get_active_map"}:
                return {"success": True, "active": "native_active"}
            if action == "list_maps":
                return {"success": True, "active": "native_active", "maps": maps}
            if action == "get_bundle":
                return {"success": False, "reason_code": "missing_capability"}
            return {"success": False, "reason_code": "unsupported_test_action"}

    gateway = GatewayModule()
    gateway.setup()
    gateway._map_client = FakeMapClient()
    _seed_ready_navigation(gateway)

    snapshot = gateway._session_snapshot()

    assert snapshot["can_activate"] is False
    assert "can_start_navigating" not in snapshot
    assert "error" not in snapshot


def test_map_routes_validate_json_contracts(monkeypatch):
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import (
        MapLifecycleResponse,
        MapListResponse,
        MapPointsResponse,
    )
    from runtime.endpoints.mapd import ArtifactHandle

    root = Path.cwd() / ".tmp_gateway_tests" / uuid.uuid4().hex
    try:
        map_dir = root / "maps"
        demo = map_dir / "demo"
        demo.mkdir(parents=True)
        _write_binary_xyz_pcd(demo / "map.pcd")
        (demo / "octomap.ot").write_bytes(b"octomap")
        (demo / "metadata.json").write_text(
            '{"state":"READY","frame_id":"odom"}',
            encoding="utf-8",
        )
        (map_dir / "active_map.txt").write_text("demo\n", encoding="utf-8")
        monkeypatch.setenv("NAV_MAP_DIR", str(map_dir))

        gateway = GatewayModule()
        gateway.setup()
        service = _attach_test_map_client(gateway, map_dir)

        class MapClient:
            def __init__(self) -> None:
                self.calls: list[tuple[str, str]] = []

            def service(self, action: str, **arguments):
                return service.service(action, **arguments)

            def open_artifact(self, map_id: str, capability: str) -> ArtifactHandle:
                self.calls.append((map_id, capability))
                descriptor = os.open(demo / "map.pcd", os.O_RDONLY)
                return ArtifactHandle(
                    metadata={"success": True, "map_id": map_id},
                    size_bytes=os.fstat(descriptor).st_size,
                    filename="map.pcd",
                    _descriptor=descriptor,
                )

        map_client = MapClient()
        gateway._map_client = map_client

        maps_payload = asyncio.run(_endpoint(gateway, "/api/v1/slam/maps")())
        live_points_payload = asyncio.run(_endpoint(gateway, "/api/v1/map/points")())
        saved_points_payload = asyncio.run(_endpoint(gateway, "/api/v1/maps/{name}/points")("demo"))
        pcd_response = asyncio.run(_endpoint(gateway, "/api/v1/maps/{name}/pcd")("demo"))
        pcd_body = asyncio.run(_stream_body(pcd_response))
        reset_payload = asyncio.run(_endpoint(gateway, "/api/v1/map_cloud/reset")())
        maps = MapListResponse.model_validate(maps_payload)
        live_points = MapPointsResponse.model_validate(live_points_payload)
        saved_points = MapPointsResponse.model_validate(saved_points_payload)
        reset = MapLifecycleResponse.model_validate(reset_payload)

        assert [item.name for item in maps.maps] == ["demo"]
        assert maps.schema_version == 1
        assert maps.count == 1
        assert maps.ts > 0
        assert maps.maps[0].has_pcd is True
        assert maps.maps[0].has_octomap is True
        assert maps.maps[0].can_activate is True
        assert maps.maps[0].state == "READY"
        assert live_points.schema_version == 1
        assert live_points.count == 0
        assert live_points.layout == "xyz_rows"
        assert live_points.frame_id == "map"
        assert live_points.source == "live_map_cloud"
        assert live_points.ts > 0
        assert live_points.points == []
        assert saved_points.schema_version == 1
        assert saved_points.count == 2
        assert saved_points.layout == "xyz_rows"
        assert saved_points.protocol_version == 2
        assert saved_points.frame_id == "odom"
        assert saved_points.epoch == 2
        assert saved_points.sequence >= 1
        assert saved_points.stream_kind == "map"
        assert saved_points.source == "mapd"
        assert saved_points.name == "demo"
        assert saved_points.content_epoch == 1
        assert saved_points.ts > 0
        assert saved_points.points == [(1.0, 2.0, 3.0), (4.0, 5.0, 6.0)]
        assert pcd_body == (demo / "map.pcd").read_bytes()
        assert pcd_response.headers["content-disposition"] == 'attachment; filename="map.pcd"'
        assert map_client.calls == [("demo", "source_pointcloud")]
        assert reset.schema_version == 1
        assert reset.ok is True
        assert reset.success is True
        assert reset.ts > 0
    finally:
        shutil.rmtree(root, ignore_errors=True)


def test_saved_map_points_require_stable_authoritative_active_scene(monkeypatch, tmp_path):
    from fastapi import HTTPException

    from gateway.gateway_module import GatewayModule

    map_dir = tmp_path / "maps"
    for name in ("map_a", "map_b"):
        target = map_dir / name
        target.mkdir(parents=True)
        _write_binary_xyz_pcd(target / "map.pcd")
        (target / "metadata.json").write_text(
            '{"state":"READY","frame_id":"map"}',
            encoding="utf-8",
        )
    (map_dir / "active_map.txt").write_text("map_b\n", encoding="utf-8")
    monkeypatch.setenv("NAV_MAP_DIR", str(map_dir))

    gateway = GatewayModule()
    gateway.setup()
    manager = _FilesystemMapdClient(map_dir)
    gateway._map_client = manager
    endpoint = _endpoint(gateway, "/api/v1/maps/{name}/points")

    with pytest.raises(HTTPException) as inactive:
        asyncio.run(endpoint("map_a"))
    assert inactive.value.status_code == 409
    assert "authoritative active map" in str(inactive.value.detail)

    original_service = manager.service

    def switch_scene_during_read(action, **arguments):
        if action == "get_map_points":
            gateway.clear_map_cloud_cache(reason="test_concurrent_scene_change")
        return original_service(action, **arguments)

    manager.service = switch_scene_during_read
    with pytest.raises(HTTPException) as changed:
        asyncio.run(endpoint("map_b"))
    assert changed.value.status_code == 409
    assert "scene changed" in str(changed.value.detail).lower()


def test_saved_map_crop_advances_viewer_epoch():
    from gateway.gateway_module import GatewayModule

    gateway = GatewayModule()
    gateway.setup()
    gateway._map_client = _FakeMapClient()
    initial_epoch = gateway._cloud_viewer.scene_identity()["epoch"]

    crop_response = asyncio.run(
        _endpoint(gateway, "/api/v1/maps/{name}/crop")(
            "demo",
            {"bounds": {"min": [0, 0, 0], "max": [1, 1, 1]}},
        )
    )
    crop_payload = _payload(crop_response)
    crop_epoch = gateway._cloud_viewer.scene_identity()["epoch"]

    assert crop_payload["live_cloud_reset"] is True
    assert crop_epoch == initial_epoch + 1


def test_slam_maps_uses_native_active_map_state(monkeypatch, tmp_path):
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import MapListResponse

    demo = tmp_path / "demo"
    other = tmp_path / "other"
    demo.mkdir()
    other.mkdir()
    (demo / "map.pcd").write_bytes(b"pcd")
    (other / "map.pcd").write_bytes(b"pcd")
    monkeypatch.setenv("NAV_MAP_DIR", str(tmp_path))

    gateway = GatewayModule()
    gateway.setup()
    _attach_test_map_client(gateway, tmp_path)
    (tmp_path / "active_map.txt").write_text("demo\n", encoding="utf-8")
    active = tmp_path / "active"
    try:
        active.symlink_to(demo, target_is_directory=True)
    except (NotImplementedError, OSError) as exc:
        pytest.skip(f"filesystem symlinks unavailable: {exc}")

    payload = asyncio.run(_endpoint(gateway, "/api/v1/slam/maps")())
    maps = MapListResponse.model_validate(payload)

    assert maps.active == "demo"
    assert {item.name: item.is_active for item in maps.maps} == {
        "demo": True,
        "other": False,
    }
    assert "active" not in {item.name for item in maps.maps}

    active.unlink()
    nested = tmp_path / "nested" / "child"
    nested.mkdir(parents=True)
    active.symlink_to(nested, target_is_directory=True)
    (tmp_path / "active_map.txt").write_text("nested/child\n", encoding="utf-8")

    payload = asyncio.run(_endpoint(gateway, "/api/v1/slam/maps")())
    maps = MapListResponse.model_validate(payload)

    assert maps.active == ""
    assert all(item.is_active is False for item in maps.maps)

    active.unlink()
    outside = tmp_path.parent / f"{tmp_path.name}-outside"
    try:
        outside.mkdir()
        active.symlink_to(outside, target_is_directory=True)
        (tmp_path / "active_map.txt").write_text("../outside\n", encoding="utf-8")

        payload = asyncio.run(_endpoint(gateway, "/api/v1/slam/maps")())
        maps = MapListResponse.model_validate(payload)

        assert maps.active == ""
        assert all(item.is_active is False for item in maps.maps)
    finally:
        shutil.rmtree(outside, ignore_errors=True)


@pytest.mark.parametrize("invalid_value", [None, "yes"])
def test_slam_maps_rejects_invalid_can_activate_contract(tmp_path, invalid_value):
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import GatewayErrorResponse

    for name in (
        "missing_field_with_octomap",
        "missing_field_incomplete",
        "missing_field_occupancy_only",
        "explicit_blocked",
    ):
        map_dir = tmp_path / name
        map_dir.mkdir()
        (map_dir / "map.pcd").write_bytes(b"pcd")
    (tmp_path / "missing_field_with_octomap" / "octomap.ot").write_bytes(b"octomap")
    (tmp_path / "missing_field_occupancy_only" / "occupancy.npz").write_bytes(b"grid")
    (tmp_path / "explicit_blocked" / "octomap.ot").write_bytes(b"octomap")

    gateway = GatewayModule()
    gateway.setup()
    service = _attach_test_map_client(gateway, tmp_path)
    original_service = service.service

    def list_contract_with_invalid_activation_field(action, **arguments):
        response = original_service(action, **arguments)
        if action != "list_maps":
            return response
        for item in response["maps"]:
            if invalid_value is None:
                item.pop("can_activate", None)
            else:
                item["can_activate"] = invalid_value
        return response

    service.service = list_contract_with_invalid_activation_field

    response = asyncio.run(_endpoint(gateway, "/api/v1/slam/maps")())
    payload = _payload(response)
    error = GatewayErrorResponse.model_validate(payload)

    assert response.status_code == 503
    assert error.error == "map_service_contract_invalid"


def test_map_save_reports_unavailable_mapd_transport():
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import MapSaveOperationResponse, MapSaveRequest

    gateway = GatewayModule()
    gateway.setup()
    gateway._get_slam_profile = lambda: "native_dds"

    class UnavailableMapdClient:
        @staticmethod
        def service(_action, **_arguments):
            raise RuntimeError("mapd unavailable")

    gateway._map_client = UnavailableMapdClient()

    response = asyncio.run(
        _endpoint(gateway, "/api/v1/map/save")(MapSaveRequest(name="native_dds_demo"))
    )
    payload = _payload(response)
    model = MapSaveOperationResponse.model_validate(payload)

    assert response.status_code == 503
    assert model.schema_version == 1
    assert model.ok is False
    assert model.success is False
    assert model.ts > 0
    assert model.name == "native_dds_demo"
    assert model.reason_code == "map_service_unavailable"
    assert model.message == "Native mapd is unavailable."


def test_map_save_operation_response_status_is_an_admission_string():
    from gateway.schemas import MapSaveOperationResponse

    assert MapSaveOperationResponse(ok=True, status="running").status == "running"
    with pytest.raises(ValueError):
        MapSaveOperationResponse(ok=True, status={"state": "RUNNING"})


def test_map_lifecycle_response_excludes_save_operation_fields():
    from gateway.schemas import MapLifecycleResponse

    properties = MapLifecycleResponse.model_json_schema()["properties"]

    assert not {
        "accepted",
        "request_id",
        "job_id",
        "job",
        "replayed",
    } & set(properties)
    assert {"path", "map_dir", "pcd", "octomap", "occupancy"}.isdisjoint(properties)


def test_map_workbench_routes_forward_canonical_mapd_actions(monkeypatch, tmp_path):
    from gateway.gateway_module import GatewayModule

    gateway = GatewayModule()
    gateway.setup()
    manager = _FakeMapClient()
    gateway._map_client = manager
    import_root = tmp_path / "imports"
    import_root.mkdir()
    source = import_root / "demo.pcd"
    source.write_bytes(b"pcd")
    monkeypatch.setenv("LINGTU_MAP_IMPORT_DIR", str(import_root))

    import_payload = _payload(
        asyncio.run(
            _endpoint(gateway, "/api/v1/maps/import_pcd")(
                {"name": "demo", "source_path": str(source), "voxel_size": 0.2}
            )
        )
    )
    crop_payload = _payload(
        asyncio.run(
            _endpoint(gateway, "/api/v1/maps/{name}/crop")(
                "demo",
                {"bounds": {"min": [0, 0, 0], "max": [1, 1, 1]}},
            )
        )
    )
    mark_payload = _payload(
        asyncio.run(
            _endpoint(gateway, "/api/v1/maps/{name}/mark_zone")(
                "demo",
                {"state": "preblocked", "center": [0, 0, 0], "radius": 0.5},
            )
        )
    )
    build_payload = _payload(asyncio.run(_endpoint(gateway, "/api/v1/maps/{name}/build_octomap")("demo")))

    assert import_payload["ok"] is True
    assert crop_payload["ok"] is True
    assert mark_payload["ok"] is True
    assert build_payload["ok"] is True
    assert manager.calls == [
        {
            "action": "import_pcd",
            "map_id": "demo",
            "has_bounds": False,
            "source_path": str(source.resolve()),
            "voxel_size": 0.2,
        },
        {
            "action": "crop_pcd",
            "map_id": "demo",
            "has_bounds": True,
            "min_x": 0.0,
            "min_y": 0.0,
            "min_z": 0.0,
            "max_x": 1.0,
            "max_y": 1.0,
            "max_z": 1.0,
            "invert": False,
            "voxel_size": 0.0,
        },
        {"action": "get_active_map"},
        {
            "action": "edit_octomap_voxels",
            "map_id": "demo",
            "editor_command": "",
            "state": "preblocked",
            "shape": "sphere",
            "x_m": 0.0,
            "y_m": 0.0,
            "z_m": 0.0,
            "radius_m": 0.5,
            "timeout_sec": 15.0,
        },
        {"action": "build_octomap_artifact", "map_id": "demo"},
    ]


def test_map_workbench_import_rejects_host_path_escape(monkeypatch, tmp_path):
    from gateway.gateway_module import GatewayModule

    import_root = tmp_path / "imports"
    import_root.mkdir()
    outside = tmp_path / "outside.pcd"
    outside.write_bytes(b"pcd")
    monkeypatch.setenv("LINGTU_MAP_IMPORT_DIR", str(import_root))

    gateway = GatewayModule()
    gateway.setup()
    gateway._map_client = _FakeMapClient()
    response = asyncio.run(
        _endpoint(gateway, "/api/v1/maps/import_pcd")({"name": "demo", "source_path": str(outside), "voxel_size": 0.0})
    )
    payload = _payload(response)

    assert response.status_code == 400
    assert payload["ok"] is False
    assert "escapes configured root" in payload["message"]


def test_operational_routes_validate_idle_json_contracts():
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import (
        ExplorationStatusResponse,
        GatewayErrorResponse,
        RecordingOperationResponse,
        RecordingStatusResponse,
        SlamStatusResponse,
        TemporalMemoryResponse,
    )

    gateway = GatewayModule()
    gateway.setup()

    class _TemporalStore:
        def query(self, **_kwargs):
            return [{"label": "door", "confidence": 0.9}]

        def query_semantic(self, *_args, **_kwargs):
            return [{"label": "door", "score": 0.98}]

    gateway._temporal_store = _TemporalStore()

    temporal_payload = asyncio.run(_endpoint(gateway, "/api/v1/memory/temporal")())
    semantic_payload = asyncio.run(
        _endpoint(gateway, "/api/v1/memory/temporal/semantic")({"embedding": [0.1, 0.2], "top_k": 1})
    )
    semantic_response = asyncio.run(_endpoint(gateway, "/api/v1/memory/temporal/semantic")({}))
    explore_status_payload = asyncio.run(_endpoint(gateway, "/api/v1/explore/status")())
    explore_start_response = asyncio.run(_endpoint(gateway, "/api/v1/explore/start")())
    slam_status_payload = asyncio.run(_endpoint(gateway, "/api/v1/slam/status")())
    recording_status_payload = asyncio.run(_endpoint(gateway, "/api/v1/recordings/status")())
    recording_stop_response = asyncio.run(_endpoint(gateway, "/api/v1/recordings/stop")())

    temporal = TemporalMemoryResponse.model_validate(temporal_payload)
    semantic_ok = TemporalMemoryResponse.model_validate(semantic_payload)
    semantic = GatewayErrorResponse.model_validate(_payload(semantic_response))
    explore_status = ExplorationStatusResponse.model_validate(explore_status_payload)
    explore_start = GatewayErrorResponse.model_validate(_payload(explore_start_response))
    slam_status = SlamStatusResponse.model_validate(slam_status_payload)
    recording_status = RecordingStatusResponse.model_validate(recording_status_payload)
    recording_stop = RecordingOperationResponse.model_validate(_payload(recording_stop_response))

    assert temporal.count == 1
    assert temporal.observations[0]["label"] == "door"
    assert semantic_ok.count == 1
    assert semantic.error == "embedding required"
    assert explore_status.available is False
    assert explore_status.can_start is False
    assert "explorer_backend_not_running" in explore_status.blockers
    assert explore_status.reason == "explorer_backend_not_running"
    assert explore_status.required_product == "explore"
    assert explore_status.supported_products == ["explore"]
    assert explore_start.error == "Exploration backend not running"
    assert explore_start.detail["reason"] == "explorer_backend_not_running"
    assert slam_status.mode in {
        "fastlio2",
        "localizer",
        "slam_only",
        "stopped",
    }
    assert recording_status.recording is False
    assert recording_stop.error == "native_recorder_unavailable"


def test_slam_status_uses_run_plan_and_native_telemetry():
    from gateway.gateway_module import GatewayModule

    gateway = GatewayModule(run_plan=_field_run_plan("nav"))
    gateway.setup()
    endpoint = _endpoint(gateway, "/api/v1/slam/status")

    gateway._localization_status = {
        "backend": "fastlio2",
        "health_source": "slam_runtime",
        "mode": "localization",
        "state": "TRACKING",
    }
    native_payload = asyncio.run(endpoint())

    assert native_payload["mode"] == "native_dds"
    assert native_payload["native_mode"] == "localization"
    assert native_payload["product_runtime"] == "native_dds"
    assert native_payload["manual_systemctl_required"] is False
    assert native_payload["service_groups"]["native_dds"] == [
        "lidar",
        "slam",
        "maps",
        "traversability",
        "nav",
        "driver",
        "explore",
    ]
    assert {
        "slam_pgo",
        "localizer",
        "legacy_slam",
        "legacy_localizer",
        "genz_icp",
        "hba",
    }.isdisjoint(native_payload["service_metadata"])
    assert native_payload["services"]["slam"] == "running"
    assert native_payload["service_details"]["slam"]["observed"]["process"]["declared"] is True


def test_service_status_exposes_catalog_readiness_contract(monkeypatch, tmp_path):
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import ServiceStatusResponse

    evidence = tmp_path / "service_readiness.json"
    evidence.write_text(
        json.dumps(
            {
                "schema": "lingtu.thunder.service_readiness.v1",
                "summary": {"ok": True, "blockers": [], "blocker_count": 0},
                "systemd": {
                    "lt-camera.service": {
                        "active_state": "active",
                        "sub_state": "running",
                    },
                    "lt-lidar.service": {
                        "active_state": "inactive",
                        "sub_state": "dead",
                    },
                },
            }
        ),
        encoding="utf-8",
    )
    monkeypatch.setenv("LINGTU_SERVICE_READINESS_JSON", str(evidence))
    monkeypatch.setenv("LINGTU_PRODUCT_SESSION_ID", "product-session-a")
    monkeypatch.setenv("LINGTU_RUN_PLAN", "/run/lingtu/plan-product-session-a.json")

    gateway = GatewayModule(run_plan=_field_run_plan("nav"))
    gateway.setup()
    endpoint = _endpoint(gateway, "/api/v1/services/status")

    payload = asyncio.run(endpoint(names="camera,lidar"))
    status = ServiceStatusResponse.model_validate(payload)

    assert status.services == {"camera": "running", "lidar": "stopped"}
    assert status.service_metadata["camera"]["checks"] == [
        "systemd",
        "native_binary",
        "dds",
        "status_file",
    ]
    assert status.service_metadata["camera"]["binaries"][0]["name"] == "camera_dds"
    assert status.service_metadata["camera"]["topics"] == ["/camera/color/camera_info"]
    assert status.service_metadata["camera"]["dds_topics"] == ["rt/camera/info"]
    assert status.service_details["camera"]["contract"]["dds_topics"] == ["rt/camera/info"]
    assert status.service_details["camera"]["contract"]["files"] == ["/dev/shm/lingtu/camera_status.json"]
    assert status.service_details["camera"]["contract"]["binaries"][1]["name"] == ("orbbec_capture")
    camera_observed = status.service_details["camera"]["observed"]
    assert camera_observed["process"]["declared"] is False
    assert camera_observed["runtime"] == {
        "env": "real",
        "product": "nav",
        "state": "active",
        "product_session_id": "product-session-a",
    }
    assert camera_observed["field_readiness"]["source"] == "field_readiness"
    assert camera_observed["field_readiness"]["systemd"]["lt-camera.service"]["active_state"] == "active"
    assert status.service_details["camera"]["ready"] is True
    assert status.service_details["camera"]["blockers"] == []
    assert status.service_details["lidar"]["observed"]["process"]["declared"] is True
    assert status.service_details["lidar"]["ready"] is False
    assert status.service_details["lidar"]["blockers"] == ["status_stopped"]


def test_service_status_default_names_follow_field_readiness_catalog():
    from gateway.gateway_module import GatewayModule
    from runtime.service_catalogs.thunder import thunder_runtime_services

    expected = tuple(dict.fromkeys((*thunder_runtime_services(), "gateway")))

    gateway = GatewayModule(run_plan=_field_run_plan("nav"))
    gateway.setup()
    endpoint = _endpoint(gateway, "/api/v1/services/status")

    payload = asyncio.run(endpoint())

    assert tuple(payload["services"]) == expected
    assert payload["services"]["explore"] == "not_declared"
    assert "camera" in payload["service_metadata"]
    assert "gateway" in payload["service_metadata"]
    assert payload["services"]["gateway"] == "running"


def test_removed_slam_relocalization_endpoints_are_not_registered():
    from gateway.gateway_module import GatewayModule

    gateway = GatewayModule()
    gateway.setup()
    paths = {route.path for route in gateway._app.routes}

    assert "/api/v1/slam/auto_relocalize" not in paths
    assert "/api/v1/slam/relocalize" not in paths
    assert "/api/v1/slam/track_against_map" not in paths


def test_localization_request_models_enforce_mode_specific_payloads():
    from pydantic import ValidationError

    from gateway.schemas import (
        LocalizationMapTrackingRequest,
        LocalizationRelocalizationRequest,
    )

    with pytest.raises(ValidationError, match="initial_pose is required"):
        LocalizationRelocalizationRequest(map_name="demo", mode="seeded")
    with pytest.raises(ValidationError, match="initial_pose is not allowed"):
        LocalizationRelocalizationRequest(
            map_name="demo",
            mode="global",
            initial_pose={"x": 1.0, "y": 2.0, "yaw": 0.3},
        )
    with pytest.raises(ValidationError):
        LocalizationMapTrackingRequest(
            map_name="demo",
            initial_pose={"x": 1.0, "y": 2.0, "yaw": 0.3},
        )


def test_seeded_relocalization_passes_map_id_to_service(monkeypatch, tmp_path):
    import subprocess

    from gateway.gateway_module import GatewayModule
    from localization.service import RelocalizationResult

    map_dir = tmp_path / "maps"
    (map_dir / "demo").mkdir(parents=True)
    (map_dir / "demo" / "map.pcd").write_text("pcd", encoding="utf-8")
    monkeypatch.setenv("NAV_MAP_DIR", str(map_dir))

    gateway = GatewayModule()
    gateway.setup()
    _attach_test_map_client(gateway, map_dir)
    gateway._localization_status = {
        "backend": "fastlio2",
        "health_source": "slam_runtime",
        "saved_map_relocalization_supported": True,
    }
    service = _FakeRelocalizationService(saved_result=RelocalizationResult(True, "success=True\n"))
    gateway.localization.bind(service)

    monkeypatch.setattr(
        subprocess,
        "run",
        lambda *_args, **_kwargs: (_ for _ in ()).throw(
            AssertionError("Gateway relocalize route must use injected service")
        ),
    )
    payload = asyncio.run(
        _endpoint(gateway, "/api/v1/localization/relocalizations")(
            {
                "map_name": "demo",
                "mode": "seeded",
                "initial_pose": {"x": 1.0, "y": 2.0, "yaw": 0.3},
            }
        )
    )

    assert payload["schema_version"] == 1
    assert payload["ok"] is True
    assert payload["success"] is True
    assert payload["ts"] > 0
    assert payload["message"] == "Relocalized to demo"
    assert service.saved_calls == [("demo", 1.0, 2.0, 0.3, 30.0)]


def test_field_seeded_relocalization_rejects_map_outside_active_product(tmp_path):
    from gateway.gateway_module import GatewayModule
    from localization.service import RelocalizationResult

    map_root = tmp_path / "maps"
    for name in ("active_map", "other_map"):
        map_dir = map_root / name
        map_dir.mkdir(parents=True)
        _seed_map_artifacts(map_dir)
    _activate_map(map_root, "active_map")

    gateway = GatewayModule()
    gateway.setup()
    gateway._compiled_run_plan = _field_guard_plan_stub("nav")
    _attach_test_map_client(gateway, map_root)
    gateway._localization_status = {
        "backend": "fastlio2",
        "health_source": "slam_runtime",
        "saved_map_relocalization_supported": True,
    }
    service = _FakeRelocalizationService(saved_result=RelocalizationResult(True, "unexpected"))
    gateway.localization.bind(service)

    response = asyncio.run(
        _endpoint(gateway, "/api/v1/localization/relocalizations")(
            {
                "map_name": "other_map",
                "mode": "seeded",
                "initial_pose": {"x": 1.0, "y": 2.0, "yaw": 0.3},
            }
        )
    )
    payload = _payload(response)

    assert response.status_code == 409
    assert payload["details"]["reason_code"] == "product_map_switch_required"
    assert payload["details"]["current_map"] == "active_map"
    assert payload["details"]["requested_map"] == "other_map"
    assert service.saved_calls == []


def test_field_tracking_rejects_map_outside_active_product(tmp_path):
    from gateway.gateway_module import GatewayModule
    from localization.service import RelocalizationResult

    map_root = tmp_path / "maps"
    for name in ("active_map", "other_map"):
        map_dir = map_root / name
        map_dir.mkdir(parents=True)
        _seed_map_artifacts(map_dir)
    _activate_map(map_root, "active_map")

    gateway = GatewayModule()
    gateway.setup()
    gateway._compiled_run_plan = _field_guard_plan_stub("nav")
    _attach_test_map_client(gateway, map_root)
    gateway._localization_status = {
        "backend": "fastlio2",
        "health_source": "slam_runtime",
        "saved_map_relocalization_supported": True,
    }
    service = _FakeRelocalizationService(track_result=RelocalizationResult(True, "unexpected"))
    gateway.localization.bind(service)

    response = asyncio.run(_endpoint(gateway, "/api/v1/localization/map-tracking")({"map_name": "other_map"}))
    payload = _payload(response)

    assert response.status_code == 409
    assert payload["details"]["reason_code"] == "product_map_switch_required"
    assert payload["details"]["current_map"] == "active_map"
    assert payload["details"]["requested_map"] == "other_map"
    assert service.track_calls == []


def test_global_relocalization_delegates_to_service_and_preserves_success_payload(
    monkeypatch,
    tmp_path,
):
    import subprocess

    from gateway.gateway_module import GatewayModule
    from localization.service import RelocalizationResult

    subprocess_calls = []

    def fail_run(*args, **kwargs):
        subprocess_calls.append((args, kwargs))
        raise AssertionError("Gateway relocalize route must delegate subprocess")

    gateway = GatewayModule()
    gateway.setup()
    gateway._localization_status = {
        "backend": "fastlio2",
        "saved_map_relocalization_supported": True,
    }
    gateway._get_slam_profile = lambda: "native_dds"
    service = _FakeRelocalizationService(
        global_result=RelocalizationResult(True, "native_global_relocalized", quality=0.04)
    )
    gateway.localization.bind(service)

    monkeypatch.setattr(subprocess, "run", fail_run)
    map_dir = tmp_path / "maps"
    (map_dir / "demo").mkdir(parents=True)
    (map_dir / "demo" / "map.pcd").write_text("pcd", encoding="utf-8")
    _attach_test_map_client(gateway, map_dir)
    payload = asyncio.run(
        _endpoint(gateway, "/api/v1/localization/relocalizations")({"map_name": "demo", "mode": "global"})
    )

    assert service.global_calls == [10.0]
    assert subprocess_calls == []
    assert payload["schema_version"] == 1
    assert payload["ok"] is True
    assert payload["success"] is True
    assert payload["ts"] > 0
    assert payload["message"] == "Relocalized to demo"
    assert payload["quality"] == 0.04


def test_seeded_relocalization_delegates_validated_request(
    monkeypatch,
    tmp_path,
):
    import subprocess

    from gateway.gateway_module import GatewayModule
    from localization.service import RelocalizationResult

    subprocess_calls = []
    map_dir = tmp_path / "maps"
    (map_dir / "demo").mkdir(parents=True)
    (map_dir / "demo" / "map.pcd").write_text("pcd", encoding="utf-8")
    monkeypatch.setenv("NAV_MAP_DIR", str(map_dir))

    def fail_run(*args, **kwargs):
        subprocess_calls.append((args, kwargs))
        raise AssertionError("Gateway relocalize route must delegate subprocess")

    gateway = GatewayModule()
    gateway.setup()
    _attach_test_map_client(gateway, map_dir)
    gateway._localization_status = {
        "backend": "fastlio2",
        "saved_map_relocalization_supported": True,
    }
    gateway._get_slam_profile = lambda: "native_dds"
    service = _FakeRelocalizationService(saved_result=RelocalizationResult(True, "service ok", quality=0.123))
    gateway.localization.bind(service)

    monkeypatch.setattr(subprocess, "run", fail_run)
    payload = asyncio.run(
        _endpoint(gateway, "/api/v1/localization/relocalizations")(
            {
                "map_name": "demo",
                "mode": "seeded",
                "initial_pose": {"x": 1.0, "y": 2.0, "yaw": 0.3},
            }
        )
    )

    assert service.saved_calls == [("demo", 1.0, 2.0, 0.3, 30.0)]
    assert subprocess_calls == []
    assert payload["schema_version"] == 1
    assert payload["ok"] is True
    assert payload["success"] is True
    assert payload["ts"] > 0
    assert payload["message"] == "Relocalized to demo"
    assert payload["quality"] == 0.123


def test_map_tracking_delegates_without_public_pose_seed(
    monkeypatch,
    tmp_path,
):
    import subprocess

    from gateway.gateway_module import GatewayModule
    from localization.service import RelocalizationResult

    subprocess_calls = []
    map_dir = tmp_path / "maps"
    (map_dir / "demo").mkdir(parents=True)
    (map_dir / "demo" / "map.pcd").write_text("pcd", encoding="utf-8")
    monkeypatch.setenv("NAV_MAP_DIR", str(map_dir))

    def fail_run(*args, **kwargs):
        subprocess_calls.append((args, kwargs))
        raise AssertionError("Gateway track route must use injected service")

    gateway = GatewayModule()
    gateway.setup()
    _attach_test_map_client(gateway, map_dir)
    gateway._localization_status = {
        "backend": "fastlio2",
        "saved_map_relocalization_supported": True,
    }
    gateway._get_slam_profile = lambda: "native_dds"
    service = _FakeRelocalizationService(
        track_result=RelocalizationResult(
            True,
            "track_against_map_started",
            quality=0.2,
            details={"track_against_map_enabled": True},
        )
    )
    gateway.localization.bind(service)

    monkeypatch.setattr(subprocess, "run", fail_run)
    payload = asyncio.run(_endpoint(gateway, "/api/v1/localization/map-tracking")({"map_name": "demo"}))

    assert service.track_calls == [10.0]
    assert subprocess_calls == []
    assert payload["schema_version"] == 1
    assert payload["ok"] is True
    assert payload["success"] is True
    assert payload["ts"] > 0
    assert payload["message"] == "track_against_map_started"
    assert payload["quality"] == 0.2
    assert payload["details"] == {"track_against_map_enabled": True}


def test_seeded_relocalization_reports_service_failure(
    monkeypatch,
    tmp_path,
):
    from gateway.gateway_module import GatewayModule
    from localization.service import RelocalizationResult

    map_dir = tmp_path / "maps"
    (map_dir / "demo").mkdir(parents=True)
    (map_dir / "demo" / "map.pcd").write_text("pcd", encoding="utf-8")
    monkeypatch.setenv("NAV_MAP_DIR", str(map_dir))

    gateway = GatewayModule()
    gateway.setup()
    _attach_test_map_client(gateway, map_dir)
    gateway._localization_status = {
        "backend": "fastlio2",
        "health_source": "slam_runtime",
        "saved_map_relocalization_supported": True,
    }
    gateway.localization.bind(_FakeRelocalizationService(saved_result=RelocalizationResult(False, "service failed")))

    payload = asyncio.run(
        _endpoint(gateway, "/api/v1/localization/relocalizations")(
            {
                "map_name": "demo",
                "mode": "seeded",
                "initial_pose": {"x": 1.0, "y": 2.0, "yaw": 0.3},
            }
        )
    )

    assert payload["schema_version"] == 1
    assert payload["ok"] is False
    assert payload["success"] is False
    assert payload["message"] == "service failed"


def test_seeded_relocalization_service_timeout_maps_to_504_payload(monkeypatch, tmp_path):
    from gateway.gateway_module import GatewayModule
    from localization.service import RelocalizationResult

    map_dir = tmp_path / "maps"
    (map_dir / "demo").mkdir(parents=True)
    (map_dir / "demo" / "map.pcd").write_text("pcd", encoding="utf-8")
    monkeypatch.setenv("NAV_MAP_DIR", str(map_dir))

    timeout_result = RelocalizationResult(
        False,
        "call timeout > 30s",
        timed_out=True,
    )

    gateway = GatewayModule()
    gateway.setup()
    _attach_test_map_client(gateway, map_dir)
    gateway._localization_status = {
        "backend": "fastlio2",
        "health_source": "slam_runtime",
        "saved_map_relocalization_supported": True,
    }
    gateway.localization.bind(_FakeRelocalizationService(saved_result=timeout_result))

    response = asyncio.run(
        _endpoint(gateway, "/api/v1/localization/relocalizations")(
            {
                "map_name": "demo",
                "mode": "seeded",
                "initial_pose": {"x": 1.0, "y": 2.0, "yaw": 0.3},
            }
        )
    )
    payload = _payload(response)

    assert response.status_code == 504
    assert payload["schema_version"] == 1
    assert payload["ok"] is False
    assert payload["success"] is False
    assert payload["message"] == "call timeout > 30s"


def test_seeded_relocalization_rejects_unsafe_map_name(monkeypatch, tmp_path):
    import subprocess

    from gateway.gateway_module import GatewayModule

    def fail_run(*_args, **_kwargs):
        raise AssertionError("unsafe map names must not call ROS")

    monkeypatch.setenv("NAV_MAP_DIR", str(tmp_path / "maps"))
    monkeypatch.setattr(subprocess, "run", fail_run)

    gateway = GatewayModule()
    gateway.setup()
    gateway._localization_status = {
        "backend": "fastlio2",
        "health_source": "slam_runtime",
        "saved_map_relocalization_supported": True,
    }

    response = asyncio.run(
        _endpoint(gateway, "/api/v1/localization/relocalizations")(
            {
                "map_name": "../outside",
                "mode": "seeded",
                "initial_pose": {"x": 1.0, "y": 2.0, "yaw": 0.3},
            }
        )
    )
    payload = _payload(response)

    assert response.status_code == 400
    assert payload["schema_version"] == 1
    assert payload["ok"] is False
    assert payload["success"] is False
    assert "unsafe characters" in payload["message"]


def test_temporal_memory_response_accepts_observation_rows():
    from gateway.schemas import TemporalMemoryResponse

    payload = {
        "observations": [{"label": "door", "score": 0.92}],
        "count": 1,
    }

    response = TemporalMemoryResponse.model_validate(payload)

    assert response.count == 1
    assert response.observations[0]["label"] == "door"
