from __future__ import annotations

import asyncio
import hashlib
import json
import shutil
import struct
import subprocess
import sys
import time
import uuid
from pathlib import Path

import pytest

pytest.importorskip("fastapi")


def _endpoint(gateway, path: str):
    return next(route.endpoint for route in gateway._app.routes if route.path == path)


def _payload(response_or_payload):
    if hasattr(response_or_payload, "body"):
        return json.loads(response_or_payload.body)
    return response_or_payload


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
        env_result=None,
        track_result=None,
    ):
        self.global_result = global_result
        self.saved_result = saved_result
        self.env_result = env_result
        self.track_result = track_result
        self.global_calls = []
        self.saved_calls = []
        self.env_calls = []
        self.track_calls = []

    def trigger_global_relocalize(self, *, timeout_s: float = 10.0):
        self.global_calls.append(timeout_s)
        return self.global_result

    def relocalize_saved_map(self, pcd_path, x, y, yaw, *, timeout_s: float = 30.0):
        self.saved_calls.append((pcd_path, x, y, yaw, timeout_s))
        return self.saved_result

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
        self.env_calls.append((pcd_path, x, y, yaw, timeout_s, base_env))
        return self.env_result

    def track_against_map(self, pcd_path, x, y, yaw, *, timeout_s: float = 10.0):
        self.track_calls.append((pcd_path, x, y, yaw, timeout_s))
        return self.track_result


def _seed_map_artifacts(map_dir: Path) -> None:
    """Create minimal valid map artifacts for OctoPlanner3D and legacy PCT."""
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
    tomogram_path = map_dir / "tomogram.pickle"
    tomogram_path.write_bytes(b"gateway-test-tomogram")
    octomap_path = map_dir / "octomap.ot"
    octomap_path.write_bytes(b"gateway-test-octomap")
    map_sha = hashlib.sha256(map_path.read_bytes()).hexdigest()
    tomogram_sha = hashlib.sha256(tomogram_path.read_bytes()).hexdigest()
    octomap_sha = hashlib.sha256(octomap_path.read_bytes()).hexdigest()
    (map_dir / "metadata.json").write_text(
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
                        "shape": [1],
                    },
                    "octomap": {
                        "path": "octomap.ot",
                        "sha256": octomap_sha,
                        "source_map_sha256": map_sha,
                        "source_profile": "thunder_field",
                        "data_source": "thunder_field",
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
    map_sha = hashlib.sha256(map_path.read_bytes()).hexdigest()
    octomap_sha = hashlib.sha256(octomap_path.read_bytes()).hexdigest()
    (map_dir / "metadata.json").write_text(
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
                    "octomap": {
                        "path": "octomap.ot",
                        "sha256": octomap_sha,
                        "source_map_sha256": map_sha,
                        "source_profile": "thunder_field",
                        "data_source": "thunder_field",
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
    map_sha = hashlib.sha256(map_path.read_bytes()).hexdigest()
    (map_dir / "metadata.json").write_text(
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
                },
            }
        ),
        encoding="utf-8",
    )


def _seed_ready_navigation(gateway):
    with gateway._state_lock:
        gateway._odom = {"x": 0.0, "y": 0.0, "z": 0.0}
        gateway._localization_status = {
            "backend": "super_lio",
            "state": "TRACKING",
            "confidence": 0.9,
            "health_source": "odom_map_cloud",
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


class _FakeMapResponse:
    def __init__(self):
        self._callbacks = []

    def _add_callback(self, callback):
        self._callbacks.append(callback)


class _FakeMapCommand:
    def __init__(self, response: _FakeMapResponse):
        self.response = response
        self.delivered = []

    def _deliver(self, raw: str):
        command = json.loads(raw)
        self.delivered.append(command)
        for callback in list(self.response._callbacks):
            callback(
                {
                    "action": command["action"],
                    "success": True,
                    "name": command.get("name"),
                }
            )


class _FakeMapManager:
    def __init__(self):
        self.map_response = _FakeMapResponse()
        self.map_command = _FakeMapCommand(self.map_response)

    def execute(self, request):
        command = request.to_mapping()
        self.map_command.delivered.append(command)
        return {
            "action": command["action"],
            "success": True,
            "name": command.get("name"),
        }


class _TypedFilesystemMapsService:
    """Test-only typed maps endpoint backed by an isolated fixture directory."""

    def __init__(self, root: Path):
        self.root = root
        self.commands: list[dict] = []

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
            from maps.artifacts import validate_saved_map_artifact_dir

            map_dir = self._map_dir(name)
            if map_dir is None:
                return self._failure(action, "map_not_found", f"map not found: {name}")
            gate = validate_saved_map_artifact_dir(
                map_dir,
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
            map_pcd_sha256 = hashlib.sha256((map_dir / "map.pcd").read_bytes()).hexdigest()
            return {
                "action": action,
                "success": True,
                "map_id": name,
                "version_id": f"{name}-lineage:v1",
                "frame_id": self._map_frame_id(map_dir),
                "map_pcd_sha256": map_pcd_sha256,
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
            "navigation_ready": has_pcd and has_octomap and metadata_path.is_file(),
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


def _attach_test_maps_service(gateway, map_root: Path) -> _TypedFilesystemMapsService:
    service = _TypedFilesystemMapsService(map_root)
    gateway._map_mgr = service
    modules = dict(getattr(gateway, "_all_modules", {}) or {})
    modules["maps.service"] = service
    gateway._all_modules = modules
    original_attach = gateway.on_system_modules

    def attach_with_maps(next_modules):
        merged = dict(next_modules)
        merged.setdefault("maps.service", service)
        return original_attach(merged)

    gateway.on_system_modules = attach_with_maps
    return service


class _FakeMapOnlyPreviewNav:
    def __init__(self):
        self.calls = []
        self.adjust_goal = False
        self.reached_goal_override: bool | None = None
        self.snap_diagnostics = None

    def preview_plan(
        self,
        x: float,
        y: float,
        z: float,
        *,
        map_only: bool = False,
        planner_constraints: dict | None = None,
    ):
        self.calls.append((x, y, z, map_only, dict(planner_constraints or {})))
        ts = 1.0
        adjusted_goal = {"x": 0.5, "y": 0.3, "z": z, "frame_id": "map", "ts": ts} if self.adjust_goal else None
        reached_goal = self.reached_goal_override if self.reached_goal_override is not None else not self.adjust_goal
        return {
            "schema_version": 1,
            "ok": True,
            "feasible": True,
            "reached_goal": reached_goal,
            "frame_id": "map",
            "start": {"x": 0.0, "y": 0.0, "z": 0.0, "frame_id": "map", "ts": ts},
            "goal": {"x": x, "y": y, "z": z, "frame_id": "map", "ts": ts},
            "adjusted_goal": adjusted_goal,
            "path": [
                {"x": 0.0, "y": 0.0, "z": 0.0, "frame_id": "map", "ts": ts},
                adjusted_goal or {"x": x, "y": y, "z": z, "frame_id": "map", "ts": ts},
            ],
            "count": 2,
            "distance_m": 1.0,
            "plan_ms": 0.5,
            "global_plan": {
                "reached_goal": reached_goal,
                "adjusted_goal": [0.5, 0.3, z] if self.adjust_goal else None,
            },
            "planner": "octoplanner3d",
            "selected_planner": "octoplanner3d",
            "plan_safety_policy": "map_only",
            "path_safety": None,
            "fallback_reason": "",
            "rejected_plans": [],
            "snap_diagnostics": self.snap_diagnostics,
            "source": "navigation_preview",
            "reasons": [],
            "error": None,
            "ts": ts,
        }


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
    from gateway.schemas import SessionResponse, SessionTransitionResponse

    gateway = GatewayModule()
    gateway.setup()
    gateway._session_detect_current_mode = lambda: ("idle", None)

    session_payload = asyncio.run(_endpoint(gateway, "/api/v1/session")())
    end_payload = asyncio.run(_endpoint(gateway, "/api/v1/session/end")())

    session = SessionResponse.model_validate(session_payload)
    ended = SessionTransitionResponse.model_validate(end_payload)

    assert session.mode == "idle"
    assert session.explorer_available is False
    assert session.explorer_unavailable_reason == "explorer_backend_not_running"
    assert session.explorer_required_profile == "explore_or_tare_explore"
    assert ended.schema_version == 1
    assert ended.ok is True
    assert ended.success is True
    assert ended.ts > 0
    assert ended.session is not None
    assert ended.session.mode == "idle"


def test_session_start_rejects_invalid_mode_with_stable_contract():
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import SessionTransitionResponse

    gateway = GatewayModule()
    gateway.setup()

    response = asyncio.run(_endpoint(gateway, "/api/v1/session/start")({"mode": "bad"}))

    rejected = SessionTransitionResponse.model_validate(_payload(response))

    assert response.status_code == 400
    assert rejected.schema_version == 1
    assert rejected.ok is False
    assert rejected.success is False
    assert rejected.session is None
    assert rejected.ts > 0
    assert "Unknown mode" in (rejected.message or "")


def test_external_session_start_rejects_cross_product_profile_bypass(monkeypatch):
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import SessionTransitionResponse

    monkeypatch.setenv("LINGTU_COMMAND_OUTPUT_MODE", "endpoint_only")
    monkeypatch.setenv("LINGTU_PROFILE", "nav")
    gateway = GatewayModule(manage_session_services=False)
    gateway.setup()
    gateway._runtime_product_profile = "map"  # stale pre-restart in-memory target

    response = asyncio.run(
        _endpoint(gateway, "/api/v1/session/start")(
            {
                "mode": "mapping",
                "profile": "map",
                "product_session": "mapping",
            }
        )
    )

    rejected = SessionTransitionResponse.model_validate(_payload(response))
    assert response.status_code == 409
    assert rejected.ok is False
    assert rejected.detail is not None
    assert rejected.detail["reason_code"] == "product_profile_mismatch"
    assert rejected.detail["current_profile"] == "nav"
    assert rejected.detail["requested_profile"] == "map"


def test_external_session_start_rejects_unknown_runtime_profile(monkeypatch):
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import SessionTransitionResponse

    monkeypatch.setenv("LINGTU_COMMAND_OUTPUT_MODE", "endpoint_only")
    monkeypatch.delenv("LINGTU_PROFILE", raising=False)
    gateway = GatewayModule(manage_session_services=False)
    gateway.setup()

    response = asyncio.run(_endpoint(gateway, "/api/v1/session/start")({"mode": "mapping"}))

    rejected = SessionTransitionResponse.model_validate(_payload(response))
    assert response.status_code == 409
    assert rejected.detail is not None
    assert rejected.detail["reason_code"] == "product_mode_switch_required"
    assert rejected.detail["current_profile"] is None


def test_external_mapping_session_rejects_wrong_native_control_mode(
    monkeypatch,
    tmp_path,
):
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import SessionTransitionResponse

    status_file = tmp_path / "nav_endpoint_status.json"
    status_file.write_text(
        json.dumps({"stamp_s": time.time(), "control_mode": "autonomy"}),
        encoding="utf-8",
    )
    monkeypatch.setenv("LINGTU_COMMAND_OUTPUT_MODE", "endpoint_only")
    monkeypatch.setenv("LINGTU_PROFILE", "map")
    monkeypatch.setenv("LINGTU_NAV_STATUS_FILE", str(status_file))
    monkeypatch.setenv("LINGTU_NAV_STATUS_MAX_AGE_S", "60")
    gateway = GatewayModule(manage_session_services=False)
    gateway.setup()

    response = asyncio.run(
        _endpoint(gateway, "/api/v1/session/start")(
            {
                "mode": "mapping",
                "profile": "map",
                "product_session": "mapping",
            }
        )
    )

    rejected = SessionTransitionResponse.model_validate(_payload(response))
    assert response.status_code == 409
    assert rejected.ok is False
    assert rejected.detail is not None
    assert rejected.detail["reason_code"] == "native_control_mode_not_ready"
    assert rejected.detail["expected_control_mode"] == "teleop"
    assert rejected.detail["actual_control_mode"] == "autonomy"


def test_external_session_rejects_requested_profile_that_differs_from_runtime(
    monkeypatch,
    tmp_path,
):
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import SessionTransitionResponse

    status_file = tmp_path / "nav_endpoint_status.json"
    status_file.write_text(
        json.dumps({"stamp_s": time.time(), "control_mode": "autonomy"}),
        encoding="utf-8",
    )
    monkeypatch.setenv("LINGTU_COMMAND_OUTPUT_MODE", "endpoint_only")
    monkeypatch.setenv("LINGTU_PROFILE", "nav")
    monkeypatch.setenv("LINGTU_NAV_STATUS_FILE", str(status_file))
    gateway = GatewayModule(manage_session_services=False)
    gateway.setup()

    response = asyncio.run(
        _endpoint(gateway, "/api/v1/session/start")({"mode": "navigating", "profile": "tracking", "map_name": "demo"})
    )

    rejected = SessionTransitionResponse.model_validate(_payload(response))
    assert response.status_code == 409
    assert rejected.detail is not None
    assert rejected.detail["reason_code"] == "product_profile_mismatch"
    assert rejected.detail["current_profile"] == "nav"
    assert rejected.detail["requested_profile"] == "tracking"


def test_external_mapping_session_accepts_matching_product_and_control_mode(
    monkeypatch,
    tmp_path,
):
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import SessionTransitionResponse

    status_file = tmp_path / "nav_endpoint_status.json"
    status_file.write_text(
        json.dumps({"stamp_s": time.time(), "control_mode": "teleop"}),
        encoding="utf-8",
    )
    monkeypatch.setenv("LINGTU_COMMAND_OUTPUT_MODE", "endpoint_only")
    monkeypatch.setenv("LINGTU_PROFILE", "map")
    monkeypatch.setenv("LINGTU_NAV_STATUS_FILE", str(status_file))
    gateway = GatewayModule(manage_session_services=False)
    gateway.setup()

    response = asyncio.run(
        _endpoint(gateway, "/api/v1/session/start")(
            {
                "mode": "mapping",
                "product_session": "mapping",
            }
        )
    )

    accepted = SessionTransitionResponse.model_validate(_payload(response))
    assert accepted.ok is True
    assert accepted.session is not None
    assert accepted.session.mode == "mapping"
    assert accepted.session.product_profile == "map"

    second_response = asyncio.run(
        _endpoint(gateway, "/api/v1/session/start")(
            {
                "mode": "mapping",
                "product_session": "mapping",
            }
        )
    )
    second = SessionTransitionResponse.model_validate(_payload(second_response))
    assert second.ok is True
    assert second.session is not None
    assert second.session.mode == "mapping"
    assert second.session.product_profile == "map"

    ended_payload = asyncio.run(_endpoint(gateway, "/api/v1/session/end")())
    ended = SessionTransitionResponse.model_validate(_payload(ended_payload))
    assert ended.ok is True
    assert ended.session is not None
    assert ended.session.mode == "idle"
    assert ended.session.product_profile == "map"
    assert ended.session.product_session == "idle"


def test_external_teleop_avoid_session_is_map_free(monkeypatch, tmp_path):
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import SessionTransitionResponse

    status_file = tmp_path / "nav_endpoint_status.json"
    status_file.write_text(
        json.dumps({"stamp_s": time.time(), "control_mode": "teleop_avoid"}),
        encoding="utf-8",
    )
    monkeypatch.setenv("LINGTU_COMMAND_OUTPUT_MODE", "endpoint_only")
    monkeypatch.setenv("LINGTU_PROFILE", "teleop_avoid")
    monkeypatch.setenv("LINGTU_NAV_STATUS_FILE", str(status_file))
    monkeypatch.setenv("LINGTU_NAV_STATUS_MAX_AGE_S", "60")
    gateway = GatewayModule(manage_session_services=False)
    gateway.setup()
    gateway._get_slam_profile = lambda: "fastlio2"

    response = asyncio.run(
        _endpoint(gateway, "/api/v1/session/start")(
            {
                "mode": "navigating",
                "profile": "teleop_avoid",
                "product_session": "teleop_avoid",
            }
        )
    )

    accepted = SessionTransitionResponse.model_validate(_payload(response))
    assert accepted.ok is True
    assert accepted.session is not None
    assert accepted.session.mode == "navigating"
    assert accepted.session.product_profile == "teleop_avoid"
    assert accepted.session.product_session == "teleop_avoid"
    assert accepted.session.active_map is None
    assert accepted.session.slam_profile == "fastlio2"


def test_session_start_accepts_legacy_map_field(monkeypatch):
    import gateway.gateway_module as gateway_module
    import gateway.routes.session as session_routes
    import runtime.service_manager as service_manager
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import SessionTransitionResponse

    class FakeServiceManager:
        def __init__(self):
            self.calls: list[tuple[str, tuple[str, ...]]] = []

        def stop(self, *services: str) -> None:
            self.calls.append(("stop", services))

        def ensure(self, *services: str) -> None:
            self.calls.append(("ensure", services))

        def wait_ready(self, *services: str, timeout: float = 15.0) -> bool:
            self.calls.append(("wait_ready", services))
            return True

    fake_service_manager = FakeServiceManager()
    monkeypatch.setattr(service_manager, "get_service_manager", lambda: fake_service_manager)
    monkeypatch.setattr(session_routes.os, "symlink", lambda src, dst: None)

    root = Path.cwd() / ".tmp_gateway_tests" / uuid.uuid4().hex
    try:
        monkeypatch.setenv("HOME", str(root))
        monkeypatch.setenv("USERPROFILE", str(root))
        map_root = root / "custom_maps"
        monkeypatch.setenv("NAV_MAP_DIR", str(map_root))
        map_dir = map_root / "demo"
        map_dir.mkdir(parents=True)
        _seed_map_artifacts(map_dir)

        gateway = GatewayModule()
        gateway.setup()
        _attach_test_maps_service(gateway, map_root)
        _activate_map(map_root, "demo")
        monkeypatch.setattr(gateway, "_spawn_auto_relocalize", lambda _: None)

        payload = asyncio.run(_endpoint(gateway, "/api/v1/session/start")({"mode": "navigating", "map": "demo"}))

        payload = _payload(payload)
        transition = SessionTransitionResponse.model_validate(payload)
        assert transition.schema_version == 1
        assert transition.ok is True
        assert transition.success is True
        assert transition.ts > 0
        assert transition.session is not None
        assert transition.session.mode == "navigating"
        assert transition.session.product_profile == "nav"
        assert transition.session.product_session == "navigation"
        assert transition.session.active_map == "demo"
        assert transition.session.map_has_pcd is True
        assert transition.session.map_has_octomap is True
        assert transition.session.map_has_octomap is True
        assert gateway._session_map == "demo"
        assert ("ensure", ("slam",)) in fake_service_manager.calls
    finally:
        shutil.rmtree(root, ignore_errors=True)


def test_session_start_accepts_octoplanner3d_octomap_without_legacy_tomogram(monkeypatch):
    import gateway.gateway_module as gateway_module
    import gateway.routes.session as session_routes
    import runtime.service_manager as service_manager
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import SessionTransitionResponse

    class FakeServiceManager:
        def __init__(self):
            self.calls: list[tuple[str, tuple[str, ...]]] = []

        def stop(self, *services: str) -> None:
            self.calls.append(("stop", services))

        def ensure(self, *services: str) -> None:
            self.calls.append(("ensure", services))

        def wait_ready(self, *services: str, timeout: float = 15.0) -> bool:
            self.calls.append(("wait_ready", services))
            return True

    fake_service_manager = FakeServiceManager()
    monkeypatch.setattr(service_manager, "get_service_manager", lambda: fake_service_manager)
    monkeypatch.setattr(session_routes.os, "symlink", lambda src, dst: None)

    root = Path.cwd() / ".tmp_gateway_tests" / uuid.uuid4().hex
    try:
        monkeypatch.setenv("HOME", str(root))
        monkeypatch.setenv("USERPROFILE", str(root))
        map_root = root / "custom_maps"
        monkeypatch.setenv("NAV_MAP_DIR", str(map_root))
        map_dir = map_root / "octomap_only"
        map_dir.mkdir(parents=True)
        _seed_octomap_only_artifacts(map_dir)

        gateway = GatewayModule()
        gateway.setup()
        _attach_test_maps_service(gateway, map_root)
        _activate_map(map_root, "octomap_only")
        monkeypatch.setattr(gateway, "_spawn_auto_relocalize", lambda _: None)

        payload = asyncio.run(
            _endpoint(gateway, "/api/v1/session/start")({"mode": "navigating", "map": "octomap_only"})
        )

        transition = SessionTransitionResponse.model_validate(_payload(payload))
        assert transition.ok is True
        assert transition.session is not None
        assert transition.session.mode == "navigating"
        assert transition.session.product_profile == "nav"
        assert transition.session.product_session == "navigation"
        assert transition.session.map_has_pcd is True
        assert transition.session.map_has_octomap is True
    finally:
        shutil.rmtree(root, ignore_errors=True)


def test_session_start_external_endpoint_does_not_manage_robot_services(monkeypatch):
    import gateway.gateway_module as gateway_module
    import gateway.routes.session as session_routes
    import runtime.service_manager as service_manager
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import SessionTransitionResponse

    def fail_service_manager():
        raise AssertionError("external DDS endpoint must not use service_manager")

    monkeypatch.setattr(service_manager, "get_service_manager", fail_service_manager)
    monkeypatch.setattr(session_routes.os, "symlink", lambda src, dst: None)

    root = Path.cwd() / ".tmp_gateway_tests" / uuid.uuid4().hex
    try:
        monkeypatch.setenv("HOME", str(root))
        monkeypatch.setenv("USERPROFILE", str(root))
        map_root = root / "custom_maps"
        monkeypatch.setenv("NAV_MAP_DIR", str(map_root))
        map_dir = map_root / "dds_external"
        map_dir.mkdir(parents=True)
        _seed_octomap_only_artifacts(map_dir)

        gateway = GatewayModule(manage_session_services=False)
        gateway.setup()
        _attach_test_maps_service(gateway, map_root)
        _activate_map(map_root, "dds_external")
        with gateway._state_lock:
            gateway._localization_status = {
                "backend": "fastlio2",
                "state": "TRACKING",
                "confidence": 1.0,
            }

        payload = asyncio.run(
            _endpoint(gateway, "/api/v1/session/start")({"mode": "navigating", "map": "dds_external"})
        )

        transition = SessionTransitionResponse.model_validate(_payload(payload))
        assert transition.ok is True
        assert transition.session is not None
        assert transition.session.mode == "navigating"
        assert transition.session.product_profile == "nav"
        assert transition.session.product_session == "navigation"
        assert transition.session.slam_profile == "fastlio2"
        assert gateway._session_map == "dds_external"

        ended = SessionTransitionResponse.model_validate(
            _payload(asyncio.run(_endpoint(gateway, "/api/v1/session/end")()))
        )
        assert ended.ok is True
        assert ended.session is not None
        assert ended.session.mode == "idle"
        assert ended.session.product_profile is None
        assert ended.session.product_session == "idle"
        assert gateway._session_map is None
    finally:
        shutil.rmtree(root, ignore_errors=True)


def test_session_start_rejects_legacy_active_map_symlink_alias(monkeypatch):
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import SessionTransitionResponse

    root = Path.cwd() / ".tmp_gateway_tests" / uuid.uuid4().hex
    try:
        monkeypatch.setenv("HOME", str(root))
        monkeypatch.setenv("USERPROFILE", str(root))
        map_root = root / "custom_maps"
        monkeypatch.setenv("NAV_MAP_DIR", str(map_root))
        map_dir = map_root / "demo"
        map_dir.mkdir(parents=True)
        _seed_octomap_only_artifacts(map_dir)
        active = map_root / "active"
        try:
            active.symlink_to(map_dir, target_is_directory=True)
        except (NotImplementedError, OSError) as exc:
            pytest.skip(f"filesystem symlinks unavailable: {exc}")

        gateway = GatewayModule(manage_session_services=False)
        gateway.setup()
        _attach_test_maps_service(gateway, map_root)
        with gateway._state_lock:
            gateway._localization_status = {
                "backend": "fastlio2",
                "state": "TRACKING",
                "confidence": 1.0,
            }

        payload = asyncio.run(_endpoint(gateway, "/api/v1/session/start")({"mode": "navigating", "map": "active"}))

        transition = SessionTransitionResponse.model_validate(_payload(payload))
        assert transition.ok is False
        assert transition.session is None
        assert "no active map" in (transition.message or "")
        assert active.is_symlink()
    finally:
        shutil.rmtree(root, ignore_errors=True)


def test_session_start_prefers_maps_service_bundle(monkeypatch):
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import SessionTransitionResponse

    class FakeMapsService:
        def __init__(self, map_root: Path):
            self.map_root = map_root
            self.bundle_calls = []
            self.active_calls = []

        def get_map_bundle(self, name: str, capability: str):
            self.bundle_calls.append((name, capability))
            return {
                "success": True,
                "map_id": name,
                "map_dir": str(self.map_root / name),
                "capability": capability,
                "artifact": {
                    "type": "OCTOMAP_3D",
                    "uri": "octomap.ot",
                },
            }

        def set_active_map(self, name: str):
            self.active_calls.append(name)
            return {
                "success": True,
                "active": name,
                "octomap": str(self.map_root / name / "octomap.ot"),
            }

    class FakeNav:
        def __init__(self):
            self.reloads = []

        def reload_planner_map(self, map_path: str):
            self.reloads.append(map_path)
            return {"ok": True}

    root = Path.cwd() / ".tmp_gateway_tests" / uuid.uuid4().hex
    try:
        monkeypatch.setenv("HOME", str(root))
        monkeypatch.setenv("USERPROFILE", str(root))
        map_root = root / "custom_maps"
        monkeypatch.setenv("NAV_MAP_DIR", str(map_root))
        map_dir = map_root / "demo"
        map_dir.mkdir(parents=True)
        _seed_octomap_only_artifacts(map_dir)

        gateway = GatewayModule(manage_session_services=False)
        gateway.setup()
        maps = _attach_test_maps_service(gateway, map_root)
        nav = FakeNav()
        gateway._all_modules["nav.mission"] = nav
        with gateway._state_lock:
            gateway._localization_status = {
                "backend": "fastlio2",
                "state": "TRACKING",
                "confidence": 1.0,
            }

        def fail_symlink(*_args, **_kwargs):
            raise AssertionError("session start must not mutate active symlink directly when maps service is present")

        import gateway.routes.session as session_routes

        monkeypatch.setattr(session_routes.os, "symlink", fail_symlink)

        payload = asyncio.run(_endpoint(gateway, "/api/v1/session/start")({"mode": "navigating", "map": "demo"}))

        transition = SessionTransitionResponse.model_validate(_payload(payload))
        assert transition.ok is True
        assert transition.session is not None
        assert transition.session.mode == "navigating"
        assert transition.session.active_map == "demo"
        bundle_calls = [
            (item.get("name"), item.get("capability"))
            for item in maps.commands
            if item.get("action") == "get_map_bundle"
        ]
        assert bundle_calls[0] == ("demo", "navigation_safety_3d")
        assert ("demo", "source_pointcloud") in bundle_calls
        assert ("demo", "navigation_safety_3d") in bundle_calls
        assert [item.get("name") for item in maps.commands if item.get("action") == "set_active"] == ["demo"]
        assert nav.reloads == [str(map_dir / "octomap.ot")]
    finally:
        shutil.rmtree(root, ignore_errors=True)


def test_session_snapshot_uses_maps_service_bundles_for_navigation_readiness():
    from gateway.gateway_module import GatewayModule

    class FakeMapsService:
        def __init__(self):
            self.bundle_calls: list[tuple[str, str]] = []

        def execute(self, request):
            command = request.to_mapping()
            if command["action"] == "get_active":
                return self.get_active_map()
            if command["action"] == "get_map_bundle":
                return self.get_map_bundle(
                    str(command.get("name") or ""),
                    str(command.get("capability") or ""),
                )
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

    gateway = GatewayModule(manage_session_services=False)
    gateway.setup()
    maps = FakeMapsService()
    gateway._all_modules = {"maps.service": maps}
    _seed_ready_navigation(gateway)

    snapshot = gateway._session_snapshot()

    assert snapshot["saved_active_map"] == "native_active"
    assert snapshot["active_map"] is None
    assert snapshot["map_has_pcd"] is True
    assert snapshot["map_has_octomap"] is True
    assert snapshot["can_start_navigating"] is True
    assert ("native_active", "source_pointcloud") in maps.bundle_calls
    assert ("native_active", "navigation_safety_3d") in maps.bundle_calls


def test_session_snapshot_exposes_runtime_switch_pending():
    from gateway.gateway_module import GatewayModule

    gateway = GatewayModule(manage_session_services=False)
    gateway.setup()
    gateway._get_slam_profile = lambda: "bridge"
    gateway._runtime_switch_pending = {
        "command_id": "switch-in-flight",
        "started_monotonic": time.monotonic(),
    }

    snapshot = gateway._session_snapshot()

    assert snapshot["pending"] is True
    assert snapshot["can_start_mapping"] is False
    assert snapshot["can_start_navigating"] is False

    gateway._runtime_switch_pending["started_monotonic"] = time.monotonic() - 301.0
    expired = gateway._session_snapshot()

    assert expired["pending"] is False
    assert getattr(gateway, "_runtime_switch_pending", None) is None


def test_session_snapshot_does_not_guess_navigation_readiness_without_maps_service(
    monkeypatch,
):
    import gateway.services.session_view as session_view
    from gateway.gateway_module import GatewayModule

    gateway = GatewayModule(manage_session_services=False)
    gateway.setup()
    gateway._all_modules = {}
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
    assert snapshot["can_start_navigating"] is False


def test_gateway_saved_map_points_prefer_maps_service_contract():
    from gateway.gateway_module import GatewayModule

    class FakeMapsService:
        def __init__(self):
            self.active_calls = 0
            self.points_calls = []

        def execute(self, request):
            command = request.to_mapping()
            if command["action"] == "get_active":
                return self.get_active_map()
            if command["action"] == "get_map_points":
                return self.get_map_points(
                    str(command.get("name") or ""),
                    max_points=int(command.get("max_points") or 0),
                )
            return {"success": False, "reason_code": "unsupported_test_action"}

        def get_active_map(self):
            self.active_calls += 1
            return {"success": True, "active": "demo"}

        def get_map_points(self, name: str, *, max_points: int = 0):
            self.points_calls.append((name, max_points))
            return {
                "success": True,
                "points": [[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]],
                "returned": 2,
            }

    gateway = GatewayModule()
    gateway.setup()
    maps = FakeMapsService()
    gateway._map_mgr = maps

    assert gateway._active_map_from_maps_service() == "demo"
    pts = gateway._saved_map_points_from_maps_service("demo", max_points=80000)

    assert pts is not None
    assert pts.shape == (2, 3)
    assert pts.dtype.name == "float32"
    assert maps.active_calls == 1
    assert maps.points_calls == [("demo", 80000)]


def test_saved_map_loader_never_reads_pcd_outside_maps_service():
    from gateway.services.saved_map_loader import load_saved_maps_loop

    class OneShotStop:
        def __init__(self):
            self.stopped = False

        def is_set(self):
            return self.stopped

        def wait(self, _seconds):
            self.stopped = True

    class FakeGateway:
        _stop_event = OneShotStop()
        _last_saved_map_event = None

        @staticmethod
        def _active_map_from_maps_service():
            return "demo"

        @staticmethod
        def _saved_map_points_from_maps_service(_name, *, max_points):
            assert max_points == 80_000
            return None

        @staticmethod
        def _load_pcd_xyz(_path):
            raise AssertionError("Gateway must not parse saved-map PCD files")

        @staticmethod
        def push_event(_event):
            raise AssertionError("no map event is expected without service data")

    gateway = FakeGateway()
    load_saved_maps_loop(gateway, stop_event=gateway._stop_event)

    assert gateway._last_saved_map_event is None


def test_session_start_preserves_product_session_for_tracking(monkeypatch):
    import gateway.gateway_module as gateway_module
    import gateway.routes.session as session_routes
    import runtime.service_manager as service_manager
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import SessionTransitionResponse

    def fail_service_manager():
        raise AssertionError("external DDS endpoint must not use service_manager")

    monkeypatch.setattr(service_manager, "get_service_manager", fail_service_manager)
    monkeypatch.setattr(session_routes.os, "symlink", lambda src, dst: None)

    root = Path.cwd() / ".tmp_gateway_tests" / uuid.uuid4().hex
    try:
        monkeypatch.setenv("HOME", str(root))
        monkeypatch.setenv("USERPROFILE", str(root))
        map_root = root / "custom_maps"
        monkeypatch.setenv("NAV_MAP_DIR", str(map_root))
        map_dir = map_root / "tracking_map"
        map_dir.mkdir(parents=True)
        _seed_octomap_only_artifacts(map_dir)

        gateway = GatewayModule(manage_session_services=False)
        gateway.setup()
        _attach_test_maps_service(gateway, map_root)
        _activate_map(map_root, "tracking_map")
        with gateway._state_lock:
            gateway._localization_status = {
                "backend": "fastlio2",
                "state": "TRACKING",
                "confidence": 1.0,
            }

        payload = asyncio.run(
            _endpoint(gateway, "/api/v1/session/start")(
                {
                    "mode": "navigating",
                    "map": "tracking_map",
                    "profile": "tracking",
                }
            )
        )

        transition = SessionTransitionResponse.model_validate(_payload(payload))
        assert transition.ok is True
        assert transition.session is not None
        assert transition.session.mode == "navigating"
        assert transition.session.product_profile == "tracking"
        assert transition.session.product_session == "tracking"
        assert gateway._session_product_profile == "tracking"
        assert gateway._session_product_session == "tracking"
    finally:
        shutil.rmtree(root, ignore_errors=True)


def test_session_start_fails_when_planner_map_reload_fails(monkeypatch):
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import SessionTransitionResponse

    class FakeNav:
        def reload_planner_map(self, map_path: str):
            return {
                "ok": False,
                "reason": "planner_reload_failed",
                "map_path": map_path,
            }

    root = Path.cwd() / ".tmp_gateway_tests" / uuid.uuid4().hex
    try:
        monkeypatch.setenv("HOME", str(root))
        monkeypatch.setenv("USERPROFILE", str(root))
        map_root = root / "custom_maps"
        monkeypatch.setenv("NAV_MAP_DIR", str(map_root))
        map_dir = map_root / "demo"
        map_dir.mkdir(parents=True)
        _seed_octomap_only_artifacts(map_dir)
        previous_dir = map_root / "previous"
        previous_dir.mkdir(parents=True)
        _seed_octomap_only_artifacts(previous_dir)
        _activate_map(map_root, "previous")

        gateway = GatewayModule(manage_session_services=False)
        gateway.setup()
        _attach_test_maps_service(gateway, map_root)
        gateway._all_modules["nav.mission"] = FakeNav()
        with gateway._state_lock:
            gateway._localization_status = {
                "backend": "fastlio2",
                "state": "TRACKING",
                "confidence": 1.0,
            }

        response = asyncio.run(_endpoint(gateway, "/api/v1/session/start")({"mode": "navigating", "map": "demo"}))
        transition = SessionTransitionResponse.model_validate(_payload(response))

        assert getattr(response, "status_code", 200) == 409
        assert transition.ok is False
        assert transition.session is None
        assert "planner" in (transition.message or "").lower()
        assert gateway._session_mode == "idle"
        assert (map_root / "active_map.txt").read_text(encoding="utf-8").strip() == "previous"
    finally:
        shutil.rmtree(root, ignore_errors=True)


def test_session_identity_tolerates_legacy_contract_without_product_session(monkeypatch):
    import gateway.routes.session as session_routes

    monkeypatch.setitem(session_routes.PRODUCT_MODE_CONTRACTS, "map", object())

    assert session_routes._normalize_product_identity(
        {
            "mode": "mapping",
            "profile": "map",
            "product_session": "mapping",
        },
        "mapping",
    ) == ("map", "mapping")
    assert session_routes._normalize_product_identity(
        {"mode": "mapping", "profile": "map"},
        "mapping",
    ) == ("map", "mapping")


def test_gateway_session_service_management_can_be_disabled_by_env(monkeypatch):
    from gateway.gateway_module import GatewayModule

    monkeypatch.setenv("LINGTU_MANAGE_SESSION_SERVICES", "0")
    gateway = GatewayModule()
    assert gateway._manage_session_services is False

    monkeypatch.setenv("LINGTU_MANAGE_SESSION_SERVICES", "1")
    gateway = GatewayModule(manage_session_services=False)
    assert gateway._manage_session_services is False


def test_map_validate_plan_rejects_missing_octomap_before_preview(monkeypatch):
    import gateway.routes.maps as map_routes
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import PlanPreviewRequest

    root = Path.cwd() / ".tmp_gateway_tests" / uuid.uuid4().hex
    try:
        monkeypatch.setenv("HOME", str(root))
        monkeypatch.setenv("USERPROFILE", str(root))
        map_root = root / "custom_maps"
        monkeypatch.setenv("NAV_MAP_DIR", str(map_root))
        map_dir = map_root / "pcd_only"
        map_dir.mkdir(parents=True)
        _seed_pcd_only_metadata(map_dir)

        gateway = GatewayModule()
        gateway.setup()
        _attach_test_maps_service(gateway, map_root)
        nav = _FakeMapOnlyPreviewNav()
        gateway.on_system_modules({"nav.mission": nav})
        _activate_map(map_root, "pcd_only")

        response = asyncio.run(
            _endpoint(gateway, "/api/v1/maps/{name}/validate_plan")(
                "pcd_only",
                PlanPreviewRequest(x=2.0, y=1.0, z=0.0),
            )
        )
        payload = _payload(response)

        assert response.status_code == 409
        assert payload["success"] is False
        assert payload["artifact_gate"]["required"] is True
        assert "octomap required but missing" in payload["artifact_gate"]["blockers"]
        assert payload["motion_published"] is False
        assert payload["no_motion_gate"]["blocked"] is True
        assert payload["no_motion_gate"]["required_artifacts"] == ["map_pcd", "octomap"]
        assert nav.calls == []
        assert gateway.goal_pose.msg_count == 0
        assert gateway.cmd_vel.msg_count == 0
    finally:
        shutil.rmtree(root, ignore_errors=True)


def test_map_validate_plan_is_explicit_no_motion_octoplanner_preview(monkeypatch):
    import gateway.routes.maps as map_routes
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import PlanPreviewRequest

    root = Path.cwd() / ".tmp_gateway_tests" / uuid.uuid4().hex
    try:
        monkeypatch.setenv("HOME", str(root))
        monkeypatch.setenv("USERPROFILE", str(root))
        map_root = root / "custom_maps"
        monkeypatch.setenv("NAV_MAP_DIR", str(map_root))
        map_dir = map_root / "octomap_ready"
        map_dir.mkdir(parents=True)
        _seed_octomap_only_artifacts(map_dir)

        gateway = GatewayModule()
        gateway.setup()
        _attach_test_maps_service(gateway, map_root)
        gateway._session_mode = "navigating"
        gateway._session_active_map_name = lambda: "octomap_ready"
        _seed_ready_navigation(gateway)
        nav = _FakeMapOnlyPreviewNav()
        gateway.on_system_modules({"nav.mission": nav})
        _activate_map(map_root, "octomap_ready")

        payload = asyncio.run(
            _endpoint(gateway, "/api/v1/maps/{name}/validate_plan")(
                "octomap_ready",
                PlanPreviewRequest(x=2.0, y=1.0, z=0.0),
            )
        )

        assert payload["success"] is True
        assert payload["motion_published"] is False
        assert payload["no_motion_gate"]["map_only"] is True
        assert payload["no_motion_gate"]["motion_published"] is False
        assert "real_runtime_evidence_missing_or_stale" in payload["no_motion_gate"]["ignored_readiness_blockers"]
        assert payload["no_motion_gate"]["selected_planner"] == "octoplanner3d"
        assert payload["preview"]["selected_planner"] == "octoplanner3d"
        assert payload["preview"]["fallback_reason"] == ""
        assert len(payload["preview"]["path"]) >= 2
        assert payload["executable_preview"]["source"] == "map_only_path_with_live_safety_overlay"
        assert nav.calls == [(2.0, 1.0, 0.0, True, {})]
        assert gateway.goal_pose.msg_count == 0
        assert gateway.cmd_vel.msg_count == 0
        assert gateway.stop_cmd.msg_count == 0
    finally:
        shutil.rmtree(root, ignore_errors=True)


def test_map_validate_plan_sanitizes_nonfinite_preview_payload(monkeypatch):
    import gateway.routes.maps as map_routes
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import PlanPreviewRequest

    root = Path.cwd() / ".tmp_gateway_tests" / uuid.uuid4().hex
    try:
        monkeypatch.setenv("HOME", str(root))
        monkeypatch.setenv("USERPROFILE", str(root))
        map_root = root / "custom_maps"
        monkeypatch.setenv("NAV_MAP_DIR", str(map_root))
        map_dir = map_root / "octomap_ready"
        map_dir.mkdir(parents=True)
        _seed_octomap_only_artifacts(map_dir)

        gateway = GatewayModule()
        gateway.setup()
        _attach_test_maps_service(gateway, map_root)
        gateway._session_mode = "navigating"
        gateway._session_active_map_name = lambda: "octomap_ready"
        _seed_ready_navigation(gateway)
        nav = _FakeMapOnlyPreviewNav()
        original_preview = nav.preview_plan

        def nonfinite_preview(*args, **kwargs):
            payload = original_preview(*args, **kwargs)
            payload["distance_m"] = float("inf")
            payload["planner_diagnostics"] = {"score": float("nan")}
            return payload

        nav.preview_plan = nonfinite_preview
        gateway.on_system_modules({"nav.mission": nav})
        _activate_map(map_root, "octomap_ready")

        payload = asyncio.run(
            _endpoint(gateway, "/api/v1/maps/{name}/validate_plan")(
                "octomap_ready",
                PlanPreviewRequest(x=2.0, y=1.0, z=0.0),
            )
        )

        json.dumps(payload, allow_nan=False)
        assert payload["preview"]["distance_m"] == 0.0
        assert payload["preview"]["planner_diagnostics"]["score"] == 0.0
        assert payload["motion_published"] is False
    finally:
        shutil.rmtree(root, ignore_errors=True)


def test_map_validate_plan_live_safety_blocks_no_motion_gate(monkeypatch):
    import gateway.routes.maps as map_routes
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import PlanPreviewRequest

    class LiveSafetyPlanner:
        def evaluate_current_path_safety(self, path):
            return {
                "ok": False,
                "reason": "live_obstacle",
                "blocked_sample_count": 2,
            }

    root = Path.cwd() / ".tmp_gateway_tests" / uuid.uuid4().hex
    try:
        monkeypatch.setenv("HOME", str(root))
        monkeypatch.setenv("USERPROFILE", str(root))
        map_root = root / "custom_maps"
        monkeypatch.setenv("NAV_MAP_DIR", str(map_root))
        map_dir = map_root / "octomap_ready"
        map_dir.mkdir(parents=True)
        _seed_octomap_only_artifacts(map_dir)

        gateway = GatewayModule()
        gateway.setup()
        _attach_test_maps_service(gateway, map_root)
        gateway._session_mode = "navigating"
        gateway._session_active_map_name = lambda: "octomap_ready"
        _seed_ready_navigation(gateway)
        nav = _FakeMapOnlyPreviewNav()
        nav._planner_svc = LiveSafetyPlanner()
        gateway.on_system_modules({"nav.mission": nav})
        _activate_map(map_root, "octomap_ready")

        payload = asyncio.run(
            _endpoint(gateway, "/api/v1/maps/{name}/validate_plan")(
                "octomap_ready",
                PlanPreviewRequest(x=2.0, y=1.0, z=0.0),
            )
        )

        gate = payload["no_motion_gate"]
        failure = gate["planner_failure"]
        assert payload["success"] is False
        assert payload["map_plan_ok"] is True
        assert payload["executable_feasible"] is False
        assert gate["blocked"] is True
        assert gate["live_safety_blocked"] is True
        assert "path_safety_failed" in gate["blockers"]
        assert "live_path_safety_failed" in gate["blockers"]
        assert failure["reason"] == "live_path_safety_failed"
        assert failure["path_safety"]["reason"] == "live_obstacle"
        assert gateway.goal_pose.msg_count == 0
        assert gateway.cmd_vel.msg_count == 0
    finally:
        shutil.rmtree(root, ignore_errors=True)


def test_map_validate_plan_blocks_large_start_snap(monkeypatch):
    import gateway.routes.maps as map_routes
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import PlanPreviewRequest

    root = Path.cwd() / ".tmp_gateway_tests" / uuid.uuid4().hex
    try:
        monkeypatch.setenv("HOME", str(root))
        monkeypatch.setenv("USERPROFILE", str(root))
        map_root = root / "custom_maps"
        monkeypatch.setenv("NAV_MAP_DIR", str(map_root))
        map_dir = map_root / "octomap_ready"
        map_dir.mkdir(parents=True)
        _seed_octomap_only_artifacts(map_dir)

        gateway = GatewayModule()
        gateway.setup()
        _attach_test_maps_service(gateway, map_root)
        gateway._session_mode = "navigating"
        gateway._session_active_map_name = lambda: "octomap_ready"
        _seed_ready_navigation(gateway)
        nav = _FakeMapOnlyPreviewNav()
        nav.snap_diagnostics = {
            "requested_start": [0.0, 0.0, 0.0],
            "effective_start": [0.8, 0.0, 0.0],
            "snapped_start": [0.8, 0.0, 0.0],
            "start_snapped": True,
            "start_snap_distance_m": 0.8,
            "requested_goal": [2.0, 1.0, 0.0],
            "effective_goal": [2.0, 1.0, 0.0],
            "snapped_goal": None,
            "goal_snapped": False,
            "goal_snap_distance_m": 0.0,
            "goal_snap_accepted": True,
        }
        gateway.on_system_modules({"nav.mission": nav})
        _activate_map(map_root, "octomap_ready")

        payload = asyncio.run(
            _endpoint(gateway, "/api/v1/maps/{name}/validate_plan")(
                "octomap_ready",
                PlanPreviewRequest(x=2.0, y=1.0, z=0.0),
            )
        )

        gate = payload["no_motion_gate"]
        assert payload["success"] is False
        assert payload["map_plan_ok"] is True
        assert payload["executable_feasible"] is False
        assert gate["blocked"] is True
        assert "start_snap_too_large" in gate["blockers"]
        assert gate["planner_failure"]["reason"] == "start_snap_too_large"
        assert gate["snap_diagnostics"]["start_snap_distance_m"] == 0.8
        assert gateway.goal_pose.msg_count == 0
        assert gateway.cmd_vel.msg_count == 0
    finally:
        shutil.rmtree(root, ignore_errors=True)


def test_map_validate_plan_allows_vertical_start_snap_when_xy_is_close(monkeypatch):
    import gateway.routes.maps as map_routes
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import PlanPreviewRequest

    root = Path.cwd() / ".tmp_gateway_tests" / uuid.uuid4().hex
    try:
        monkeypatch.setenv("HOME", str(root))
        monkeypatch.setenv("USERPROFILE", str(root))
        map_root = root / "custom_maps"
        monkeypatch.setenv("NAV_MAP_DIR", str(map_root))
        map_dir = map_root / "octomap_ready"
        map_dir.mkdir(parents=True)
        _seed_octomap_only_artifacts(map_dir)

        gateway = GatewayModule()
        gateway.setup()
        _attach_test_maps_service(gateway, map_root)
        gateway._session_mode = "navigating"
        gateway._session_active_map_name = lambda: "octomap_ready"
        _seed_ready_navigation(gateway)
        nav = _FakeMapOnlyPreviewNav()
        nav.snap_diagnostics = {
            "requested_start": [0.0, 0.0, 0.0],
            "effective_start": [0.1, 0.1, 0.7],
            "snapped_start": [0.1, 0.1, 0.7],
            "start_snapped": True,
            "start_snap_distance_m": 0.714,
            "start_snap_xy_distance_m": 0.141,
            "requested_goal": [2.0, 1.0, 0.0],
            "effective_goal": [2.0, 1.0, 0.0],
            "snapped_goal": None,
            "goal_snapped": False,
            "goal_snap_distance_m": 0.0,
            "goal_snap_xy_distance_m": 0.0,
            "goal_snap_accepted": True,
        }
        gateway.on_system_modules({"nav.mission": nav})
        _activate_map(map_root, "octomap_ready")

        payload = asyncio.run(
            _endpoint(gateway, "/api/v1/maps/{name}/validate_plan")(
                "octomap_ready",
                PlanPreviewRequest(x=2.0, y=1.0, z=0.0),
            )
        )

        gate = payload["no_motion_gate"]
        assert payload["success"] is True
        assert payload["map_plan_ok"] is True
        assert payload["executable_feasible"] is True
        assert gate["blocked"] is False
        assert "start_snap_too_large" not in gate["blockers"]
        assert gate["snap_diagnostics"]["start_snap_xy_distance_m"] == 0.141
        assert gateway.goal_pose.msg_count == 0
        assert gateway.cmd_vel.msg_count == 0
    finally:
        shutil.rmtree(root, ignore_errors=True)


def test_map_validate_plan_structures_octoplanner_start_out_of_map(monkeypatch):
    import gateway.routes.maps as map_routes
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import PlanPreviewRequest

    root = Path.cwd() / ".tmp_gateway_tests" / uuid.uuid4().hex
    try:
        monkeypatch.setenv("HOME", str(root))
        monkeypatch.setenv("USERPROFILE", str(root))
        map_root = root / "custom_maps"
        monkeypatch.setenv("NAV_MAP_DIR", str(map_root))
        map_dir = map_root / "octomap_ready"
        map_dir.mkdir(parents=True)
        _seed_octomap_only_artifacts(map_dir)

        gateway = GatewayModule()
        gateway.setup()
        _attach_test_maps_service(gateway, map_root)
        gateway._session_mode = "navigating"
        gateway._session_active_map_name = lambda: "octomap_ready"
        _seed_ready_navigation(gateway)
        nav = _FakeMapOnlyPreviewNav()

        def failed_preview(*args, **kwargs):
            return {
                "schema_version": 1,
                "ok": True,
                "feasible": False,
                "reached_goal": False,
                "frame_id": "map",
                "start": {
                    "x": 219143.115717,
                    "y": 421310.983082,
                    "z": 2678.665038,
                    "frame_id": "map",
                },
                "goal": {"x": 2.0, "y": 1.0, "z": 0.0, "frame_id": "map"},
                "path": [],
                "count": 0,
                "planner": "octoplanner3d",
                "selected_planner": "octoplanner3d",
                "fallback_reason": "empty path",
                "reasons": ["planning_failed"],
                "error": "empty path",
                "rejected_plans": [
                    {
                        "planner": "octoplanner3d",
                        "reason": "empty path",
                        "planner_diagnostics": {
                            "runtime_mode": "cxx_headless",
                            "process_boundary": "subprocess",
                            "executable_path": "/opt/lingtu/current/build/octoplanner3d_headless/octoplanner3d_headless",
                            "runtime_map_path": "/home/sunrise/data/nova/maps/active/octomap.ot",
                            "returncode": 2,
                            "stdout": (
                                "GlobalPlanner::startPlan() Start is occupied/out of map and no nearby free cell."
                            ),
                        },
                    }
                ],
            }

        nav.preview_plan = failed_preview
        gateway.on_system_modules({"nav.mission": nav})
        _activate_map(map_root, "octomap_ready")

        payload = asyncio.run(
            _endpoint(gateway, "/api/v1/maps/{name}/validate_plan")(
                "octomap_ready",
                PlanPreviewRequest(x=2.0, y=1.0, z=0.0),
            )
        )

        failure = payload["no_motion_gate"]["planner_failure"]
        assert payload["success"] is False
        assert payload["motion_published"] is False
        assert "planning_failed" in payload["no_motion_gate"]["blockers"]
        assert "start_occupied_or_out_of_map" in payload["no_motion_gate"]["blockers"]
        assert "pose_map_mismatch" in payload["no_motion_gate"]["blockers"]
        assert failure["reason"] == "start_occupied_or_out_of_map"
        assert failure["diagnostic_codes"] == ["pose_map_mismatch"]
        assert failure["start_goal_xy_delta_m"] > 1000.0
        assert failure["runtime_mode"] == "cxx_headless"
        assert failure["process_boundary"] == "subprocess"
        assert failure["returncode"] == 2
    finally:
        shutil.rmtree(root, ignore_errors=True)


def test_map_validate_plan_forwards_planner_constraints(monkeypatch):
    import gateway.routes.maps as map_routes
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import PlanPreviewRequest

    root = Path.cwd() / ".tmp_gateway_tests" / uuid.uuid4().hex
    try:
        monkeypatch.setenv("HOME", str(root))
        monkeypatch.setenv("USERPROFILE", str(root))
        map_root = root / "custom_maps"
        monkeypatch.setenv("NAV_MAP_DIR", str(map_root))
        map_dir = map_root / "octomap_ready"
        map_dir.mkdir(parents=True)
        _seed_octomap_only_artifacts(map_dir)

        gateway = GatewayModule()
        gateway.setup()
        _attach_test_maps_service(gateway, map_root)
        gateway._session_mode = "navigating"
        gateway._session_active_map_name = lambda: "octomap_ready"
        _seed_ready_navigation(gateway)
        nav = _FakeMapOnlyPreviewNav()
        gateway.on_system_modules({"nav.mission": nav})
        _activate_map(map_root, "octomap_ready")

        payload = asyncio.run(
            _endpoint(gateway, "/api/v1/maps/{name}/validate_plan")(
                "octomap_ready",
                PlanPreviewRequest(
                    x=2.0,
                    y=1.0,
                    z=0.0,
                    planner_constraints={
                        "robot_radius": 0.4,
                        "obstacle_clearance_weight": 2.0,
                    },
                ),
            )
        )

        assert payload["success"] is True
        assert nav.calls == [
            (
                2.0,
                1.0,
                0.0,
                True,
                {"robot_radius": 0.4, "obstacle_clearance_weight": 2.0},
            )
        ]
        assert gateway.goal_pose.msg_count == 0
        assert gateway.cmd_vel.msg_count == 0
    finally:
        shutil.rmtree(root, ignore_errors=True)


def test_map_validate_plan_rejects_adjusted_goal(monkeypatch):
    import gateway.routes.maps as map_routes
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import PlanPreviewRequest

    root = Path.cwd() / ".tmp_gateway_tests" / uuid.uuid4().hex
    try:
        monkeypatch.setenv("HOME", str(root))
        monkeypatch.setenv("USERPROFILE", str(root))
        map_root = root / "custom_maps"
        monkeypatch.setenv("NAV_MAP_DIR", str(map_root))
        map_dir = map_root / "octomap_ready"
        map_dir.mkdir(parents=True)
        _seed_octomap_only_artifacts(map_dir)

        gateway = GatewayModule()
        gateway.setup()
        _attach_test_maps_service(gateway, map_root)
        gateway._session_mode = "navigating"
        gateway._session_active_map_name = lambda: "octomap_ready"
        _seed_ready_navigation(gateway)
        nav = _FakeMapOnlyPreviewNav()
        nav.adjust_goal = True
        gateway.on_system_modules({"nav.mission": nav})
        _activate_map(map_root, "octomap_ready")

        payload = asyncio.run(
            _endpoint(gateway, "/api/v1/maps/{name}/validate_plan")(
                "octomap_ready",
                PlanPreviewRequest(x=2.0, y=1.0, z=0.0),
            )
        )

        assert payload["success"] is False
        assert payload["preview"]["feasible"] is True
        assert payload["preview"]["adjusted_goal"] is not None
        assert payload["executable_feasible"] is False
        assert payload["executable_preview"]["feasible"] is False
        assert payload["no_motion_gate"]["preview_feasible"] is True
        assert payload["no_motion_gate"]["target_reached"] is False
        assert "goal_adjusted" in payload["no_motion_gate"]["blockers"]
        assert gateway.goal_pose.msg_count == 0
        assert gateway.cmd_vel.msg_count == 0
    finally:
        shutil.rmtree(root, ignore_errors=True)


def test_map_validate_plan_accepts_adjusted_goal_within_planner_tolerance(monkeypatch):
    import gateway.routes.maps as map_routes
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import PlanPreviewRequest

    root = Path.cwd() / ".tmp_gateway_tests" / uuid.uuid4().hex
    try:
        monkeypatch.setenv("HOME", str(root))
        monkeypatch.setenv("USERPROFILE", str(root))
        map_root = root / "custom_maps"
        monkeypatch.setenv("NAV_MAP_DIR", str(map_root))
        map_dir = map_root / "octomap_ready"
        map_dir.mkdir(parents=True)
        _seed_octomap_only_artifacts(map_dir)

        gateway = GatewayModule()
        gateway.setup()
        _attach_test_maps_service(gateway, map_root)
        gateway._session_mode = "navigating"
        gateway._session_active_map_name = lambda: "octomap_ready"
        _seed_ready_navigation(gateway)
        nav = _FakeMapOnlyPreviewNav()
        nav.adjust_goal = True
        nav.reached_goal_override = True
        gateway.on_system_modules({"nav.mission": nav})
        _activate_map(map_root, "octomap_ready")

        payload = asyncio.run(
            _endpoint(gateway, "/api/v1/maps/{name}/validate_plan")(
                "octomap_ready",
                PlanPreviewRequest(x=2.0, y=1.0, z=0.0),
            )
        )

        assert payload["success"] is True
        assert payload["preview"]["adjusted_goal"] is not None
        assert payload["preview"]["reached_goal"] is True
        assert payload["no_motion_gate"]["target_reached"] is True
        assert payload["executable_feasible"] is True
        assert gateway.goal_pose.msg_count == 0
        assert gateway.cmd_vel.msg_count == 0
    finally:
        shutil.rmtree(root, ignore_errors=True)


def test_session_start_can_select_super_lio_backend(monkeypatch):
    import runtime.service_manager as service_manager
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import SessionTransitionResponse

    class FakeServiceManager:
        def __init__(self):
            self.calls: list[tuple[str, tuple[str, ...]]] = []
            self.services = {
                "lidar": "running",
                "slam": "stopped",
                "slam_pgo": "stopped",
                "localizer": "stopped",
                "super_lio": "running",
                "super_lio_relocation": "stopped",
            }

        def stop(self, *services: str) -> None:
            self.calls.append(("stop", services))

        def ensure(self, *services: str) -> None:
            self.calls.append(("ensure", services))

        def wait_ready(self, *services: str, timeout: float = 15.0) -> bool:
            self.calls.append(("wait_ready", services))
            return True

        def status(self, *names):
            return {name: self.services.get(name, "stopped") for name in names}

    fake_service_manager = FakeServiceManager()
    monkeypatch.setattr(service_manager, "get_service_manager", lambda: fake_service_manager)

    gateway = GatewayModule()
    gateway.setup()

    payload = asyncio.run(_endpoint(gateway, "/api/v1/session/start")({"mode": "mapping", "slam_profile": "super_lio"}))

    transition = SessionTransitionResponse.model_validate(payload)
    assert transition.schema_version == 1
    assert transition.ok is True
    assert transition.success is True
    assert transition.ts > 0
    assert transition.session is not None
    assert transition.session.mode == "mapping"
    assert transition.session.slam_profile == "super_lio"
    assert gateway._session_slam_profile == "super_lio"
    assert (
        "stop",
        ("slam", "slam_pgo", "localizer", "hba", "genz_icp", "super_lio_relocation"),
    ) in fake_service_manager.calls
    assert ("ensure", ("legacy_lidar", "super_lio")) in fake_service_manager.calls
    assert ("wait_ready", ("legacy_lidar", "super_lio")) in fake_service_manager.calls


def test_session_start_can_select_super_lio_relocation_backend(monkeypatch):
    import gateway.routes.session as session_routes
    import runtime.service_manager as service_manager
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import SessionTransitionResponse

    class FakeServiceManager:
        def __init__(self):
            self.calls: list[tuple[str, tuple[str, ...]]] = []
            self.services = {
                "lidar": "running",
                "slam": "stopped",
                "slam_pgo": "stopped",
                "localizer": "stopped",
                "super_lio": "stopped",
                "super_lio_relocation": "running",
            }

        def stop(self, *services: str) -> None:
            self.calls.append(("stop", services))

        def ensure(self, *services: str) -> None:
            self.calls.append(("ensure", services))

        def wait_ready(self, *services: str, timeout: float = 15.0) -> bool:
            self.calls.append(("wait_ready", services))
            return True

        def status(self, *names):
            return {name: self.services.get(name, "stopped") for name in names}

    fake_service_manager = FakeServiceManager()
    monkeypatch.setattr(service_manager, "get_service_manager", lambda: fake_service_manager)
    monkeypatch.setattr(session_routes.os, "symlink", lambda src, dst: None)

    root = Path.cwd() / ".tmp_gateway_tests" / uuid.uuid4().hex
    try:
        monkeypatch.setenv("HOME", str(root))
        monkeypatch.setenv("USERPROFILE", str(root))
        map_root = root / "custom_maps"
        monkeypatch.setenv("NAV_MAP_DIR", str(map_root))
        map_dir = map_root / "demo"
        map_dir.mkdir(parents=True)
        _seed_map_artifacts(map_dir)

        gateway = GatewayModule()
        gateway.setup()
        _attach_test_maps_service(gateway, map_root)
        auto_relocalize_calls: list[str] = []
        monkeypatch.setattr(gateway, "_spawn_auto_relocalize", auto_relocalize_calls.append)

        payload = asyncio.run(
            _endpoint(gateway, "/api/v1/session/start")(
                {
                    "mode": "navigating",
                    "map_name": "demo",
                    "slam_profile": "super_lio_relocation",
                }
            )
        )

        payload = _payload(payload)
        transition = SessionTransitionResponse.model_validate(payload)
        assert transition.schema_version == 1
        assert transition.ok is True
        assert transition.success is True
        assert transition.ts > 0
        assert transition.session is not None
        assert transition.session.mode == "navigating"
        assert transition.session.slam_profile == "super_lio_relocation"
        assert gateway._session_map == "demo"
        assert gateway._session_slam_profile == "super_lio_relocation"
        assert (
            "stop",
            ("slam", "slam_pgo", "localizer", "hba", "genz_icp", "super_lio"),
        ) in fake_service_manager.calls
        assert ("ensure", ("legacy_lidar", "super_lio_relocation")) in (fake_service_manager.calls)
        assert ("wait_ready", ("legacy_lidar", "super_lio_relocation")) in (fake_service_manager.calls)
        assert auto_relocalize_calls == []
    finally:
        shutil.rmtree(root, ignore_errors=True)


def test_map_routes_validate_json_contracts(monkeypatch):
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import (
        MapLifecycleResponse,
        MapListResponse,
        MapPointsResponse,
        MapRequest,
    )

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
        _attach_test_maps_service(gateway, map_dir)

        maps_payload = asyncio.run(_endpoint(gateway, "/api/v1/slam/maps")())
        live_points_payload = asyncio.run(_endpoint(gateway, "/api/v1/map/points")())
        saved_points_payload = asyncio.run(_endpoint(gateway, "/api/v1/maps/{name}/points")("demo"))
        pcd_response = asyncio.run(_endpoint(gateway, "/api/v1/maps/{name}/pcd")("demo"))
        reset_payload = asyncio.run(_endpoint(gateway, "/api/v1/map_cloud/reset")())
        map_command_response = asyncio.run(_endpoint(gateway, "/api/v1/maps")(MapRequest(action="list")))
        map_command_payload = _payload(map_command_response)

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
        assert maps.maps[0].navigation_ready is True
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
        assert saved_points.source == "maps_service"
        assert saved_points.name == "demo"
        assert saved_points.version_id == "demo-lineage:v1"
        assert saved_points.map_pcd_sha256 == hashlib.sha256((demo / "map.pcd").read_bytes()).hexdigest()
        assert saved_points.ts > 0
        assert saved_points.points == [(1.0, 2.0, 3.0), (4.0, 5.0, 6.0)]
        assert Path(pcd_response.path).name == "map.pcd"
        assert reset.schema_version == 1
        assert reset.ok is True
        assert reset.success is True
        assert reset.ts > 0
        assert map_command_payload["schema_version"] == 1
        assert map_command_payload["ok"] is True
        assert map_command_payload["maps"]
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
    manager = _TypedFilesystemMapsService(map_dir)
    gateway._map_mgr = manager
    endpoint = _endpoint(gateway, "/api/v1/maps/{name}/points")

    with pytest.raises(HTTPException) as inactive:
        asyncio.run(endpoint("map_a"))
    assert inactive.value.status_code == 409
    assert "authoritative active map" in str(inactive.value.detail)

    original_execute = manager.execute

    def switch_scene_during_read(request):
        if request.to_mapping().get("action") == "get_map_points":
            gateway.clear_map_cloud_cache(reason="test_concurrent_scene_change")
        return original_execute(request)

    manager.execute = switch_scene_during_read
    with pytest.raises(HTTPException) as changed:
        asyncio.run(endpoint("map_b"))
    assert changed.value.status_code == 409
    assert "scene changed" in str(changed.value.detail).lower()


def test_saved_map_point_mutations_advance_viewer_epoch():
    from gateway.gateway_module import GatewayModule

    gateway = GatewayModule()
    gateway.setup()
    gateway._map_mgr = _FakeMapManager()
    initial_epoch = gateway._cloud_viewer.scene_identity()["epoch"]

    crop_response = asyncio.run(
        _endpoint(gateway, "/api/v1/maps/{name}/crop")(
            "demo",
            {"bounds": {"min": [0, 0, 0], "max": [1, 1, 1]}},
        )
    )
    crop_payload = _payload(crop_response)
    crop_epoch = gateway._cloud_viewer.scene_identity()["epoch"]

    restore_response = asyncio.run(_endpoint(gateway, "/api/v1/map/restore_predufo")({"name": "demo"}))
    restore_payload = _payload(restore_response)
    restore_epoch = gateway._cloud_viewer.scene_identity()["epoch"]

    assert crop_payload["live_cloud_reset"] is True
    assert restore_payload["live_cloud_reset"] is True
    assert crop_epoch == initial_epoch + 1
    assert restore_epoch == crop_epoch + 1


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
    _attach_test_maps_service(gateway, tmp_path)
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


def test_slam_maps_derives_legacy_navigation_readiness_without_overriding_false(tmp_path):
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import MapListResponse

    for name in (
        "legacy_ready",
        "legacy_incomplete",
        "legacy_occupancy_only",
        "explicit_blocked",
    ):
        map_dir = tmp_path / name
        map_dir.mkdir()
        (map_dir / "map.pcd").write_bytes(b"pcd")
    (tmp_path / "legacy_ready" / "octomap.ot").write_bytes(b"octomap")
    (tmp_path / "legacy_occupancy_only" / "occupancy.npz").write_bytes(b"grid")
    (tmp_path / "explicit_blocked" / "octomap.ot").write_bytes(b"octomap")

    gateway = GatewayModule()
    gateway.setup()
    service = _attach_test_maps_service(gateway, tmp_path)
    original_execute = service.execute

    def legacy_list_contract(request):
        response = original_execute(request)
        if request.to_mapping().get("action") != "list":
            return response
        for item in response["maps"]:
            if item["name"] in {
                "legacy_ready",
                "legacy_incomplete",
                "legacy_occupancy_only",
            }:
                item.pop("navigation_ready", None)
            elif item["name"] == "explicit_blocked":
                item["navigation_ready"] = False
        return response

    service.execute = legacy_list_contract

    payload = asyncio.run(_endpoint(gateway, "/api/v1/slam/maps")())
    maps = MapListResponse.model_validate(payload)
    readiness = {item.name: item.navigation_ready for item in maps.maps}

    assert readiness == {
        "explicit_blocked": False,
        "legacy_incomplete": False,
        "legacy_occupancy_only": False,
        "legacy_ready": True,
    }


def test_map_lifecycle_error_responses_use_stable_envelope(monkeypatch, tmp_path):
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import MapLifecycleResponse, MapNameRequest

    monkeypatch.setenv("NAV_MAP_DIR", str(tmp_path))

    gateway = GatewayModule()
    gateway.setup()

    response = asyncio.run(_endpoint(gateway, "/api/v1/map/activate")(MapNameRequest(name="../bad")))
    model = MapLifecycleResponse.model_validate(_payload(response))

    assert response.status_code == 400
    assert model.schema_version == 1
    assert model.ok is False
    assert model.success is False
    assert model.ts > 0
    assert model.message


def test_map_activate_route_uses_map_service_gate():
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import MapNameRequest

    class FakeNav:
        def __init__(self):
            self.reloads = []

        def reload_planner_map(self, map_path: str = ""):
            self.reloads.append(map_path)
            return {"ok": True, "map_path": map_path}

    gateway = GatewayModule()
    gateway.setup()
    manager = _FakeMapManager()
    nav = FakeNav()
    gateway._map_mgr = manager
    gateway._all_modules = {"nav.mission": nav}

    payload = _payload(asyncio.run(_endpoint(gateway, "/api/v1/map/activate")(MapNameRequest(name="demo"))))

    assert payload["ok"] is True
    assert manager.map_command.delivered == [
        {"action": "get_active"},
        {"action": "get_active"},
        {"action": "set_active", "name": "demo"},
        {"action": "get_active"},
    ]
    assert nav.reloads == [""]
    assert payload["planner_reload"]["ok"] is True


def test_map_activate_rejects_cross_map_switch_during_navigation():
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import MapNameRequest

    gateway = GatewayModule()
    gateway.setup()
    manager = _FakeMapManager()
    gateway._map_mgr = manager
    gateway._session_mode = "navigating"
    gateway._session_map = "active_demo"

    response = asyncio.run(_endpoint(gateway, "/api/v1/map/activate")(MapNameRequest(name="other_demo")))
    payload = _payload(response)

    assert response.status_code == 409
    assert payload["ok"] is False
    assert "runtime switch" in payload["message"].lower()
    assert all(command.get("action") != "set_active" for command in manager.map_command.delivered)


def test_map_activate_rolls_back_when_planner_reload_fails(tmp_path):
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import MapNameRequest

    class FailingNav:
        def __init__(self):
            self.reloads = []

        def reload_planner_map(self, map_path: str = ""):
            self.reloads.append(map_path)
            if "previous" in map_path:
                return {"ok": True, "map_path": map_path}
            return {
                "ok": False,
                "reason": "planner_reload_failed",
                "map_path": map_path,
            }

    map_root = tmp_path / "maps"
    for name in ("previous", "target"):
        map_dir = map_root / name
        map_dir.mkdir(parents=True)
        _seed_octomap_only_artifacts(map_dir)
    _activate_map(map_root, "previous")

    gateway = GatewayModule()
    gateway.setup()
    manager = _TypedFilesystemMapsService(map_root)
    gateway._map_mgr = manager
    nav = FailingNav()
    gateway._all_modules = {"nav.mission": nav}

    response = asyncio.run(_endpoint(gateway, "/api/v1/map/activate")(MapNameRequest(name="target")))
    payload = _payload(response)

    assert response.status_code == 409
    assert payload["ok"] is False
    assert payload["planner_reload"]["ok"] is False
    assert payload["rollback"]["success"] is True
    assert payload["rollback"]["planner_reload"]["ok"] is True
    assert len(nav.reloads) == 2
    assert (map_root / "active_map.txt").read_text(encoding="utf-8").strip() == "previous"


def test_map_save_requires_injected_maps_service():
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import MapLifecycleResponse

    gateway = GatewayModule()
    gateway.setup()
    gateway._get_slam_profile = lambda: "fastlio2"

    response = asyncio.run(_endpoint(gateway, "/api/v1/map/save")({"name": "native_dds_demo"}))
    payload = _payload(response)
    model = MapLifecycleResponse.model_validate(payload)

    assert response.status_code == 503
    assert model.schema_version == 1
    assert model.ok is False
    assert model.success is False
    assert model.ts > 0
    assert model.name == "native_dds_demo"
    assert "ProductGraph/Blueprint must inject maps.service" in model.message
    assert gateway._map_mgr is None


def test_map_save_rejects_super_lio_relocation_profile(monkeypatch, tmp_path):
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import MapLifecycleResponse

    calls = []

    class FailingMapSaveAdapter:
        def save_nav_map(self, *args, **kwargs):
            calls.append(("nav", args, kwargs))
            raise AssertionError("relocation map save should fail before adapter calls")

        def save_slam_map(self, *args, **kwargs):
            calls.append(("slam", args, kwargs))
            raise AssertionError("relocation map save should fail before adapter calls")

    monkeypatch.setenv("NAV_MAP_DIR", str(tmp_path))

    gateway = GatewayModule(map_save_adapter=FailingMapSaveAdapter())
    gateway.setup()
    monkeypatch.setattr(gateway, "_get_slam_profile", lambda: "super_lio_relocation")

    response = asyncio.run(_endpoint(gateway, "/api/v1/map/save")({"name": "relocation_demo"}))
    payload = _payload(response)
    model = MapLifecycleResponse.model_validate(payload)

    assert response.status_code == 409
    assert model.ok is False
    assert model.success is False
    assert model.name == "relocation_demo"
    assert "not supported" in model.message
    assert calls == []
    assert not (tmp_path / "relocation_demo").exists()


def test_maps_route_normalizes_public_action_aliases():
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import MapLifecycleResponse, MapRequest

    gateway = GatewayModule()
    gateway.setup()
    manager = _FakeMapManager()
    gateway._map_mgr = manager
    post_maps = _endpoint(gateway, "/api/v1/maps")

    use_payload = asyncio.run(post_maps(MapRequest(action="use", name="demo")))
    build_payload = asyncio.run(post_maps(MapRequest(action="build", name="demo")))
    canonical_payload = asyncio.run(post_maps(MapRequest(action="build_octomap", name="demo")))
    occupancy_payload = asyncio.run(post_maps(MapRequest(action="build_occupancy", name="demo")))

    for payload in (use_payload, build_payload, canonical_payload, occupancy_payload):
        model = MapLifecycleResponse.model_validate(payload)
        assert model.schema_version == 1
        assert model.ok is True
        assert model.success is True
        assert model.ts > 0
    assert manager.map_command.delivered == [
        {"action": "set_active", "name": "demo"},
        {"action": "build_octomap", "name": "demo"},
        {"action": "build_octomap", "name": "demo"},
        {"action": "build_occupancy", "name": "demo"},
    ]


def test_map_workbench_routes_forward_to_map_service(monkeypatch, tmp_path):
    from gateway.gateway_module import GatewayModule

    gateway = GatewayModule()
    gateway.setup()
    manager = _FakeMapManager()
    gateway._map_mgr = manager
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
    assert manager.map_command.delivered == [
        {
            "action": "import_pcd",
            "name": "demo",
            "source_path": str(source.resolve()),
            "voxel_size": 0.2,
            "bounds": None,
        },
        {
            "action": "crop",
            "name": "demo",
            "bounds": {"min": [0, 0, 0], "max": [1, 1, 1]},
            "invert": False,
            "voxel_size": 0.0,
        },
        {"action": "get_active"},
        {
            "action": "edit_voxels",
            "state": "preblocked",
            "center": [0, 0, 0],
            "radius": 0.5,
            "name": "demo",
        },
        {"action": "build_octomap", "name": "demo"},
    ]


def test_map_workbench_import_route_writes_real_map_package(monkeypatch, tmp_path):
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import MapLifecycleResponse, MapListResponse
    from maps.modules.service import MapsModule

    import_root = tmp_path / "imports"
    import_root.mkdir()
    source = import_root / "source.pcd"
    _write_binary_xyz_pcd(source)
    map_dir = tmp_path / "maps"
    monkeypatch.setenv("NAV_MAP_DIR", str(map_dir))
    monkeypatch.setenv("LINGTU_MAP_IMPORT_DIR", str(import_root))

    gateway = GatewayModule()
    gateway.setup()
    manager = MapsModule(map_dir=str(map_dir), data_dir=str(tmp_path / "data"))
    manager.setup()
    gateway._map_mgr = manager

    response = asyncio.run(
        _endpoint(gateway, "/api/v1/maps/import_pcd")({"name": "demo", "source_path": str(source), "voxel_size": 0.0})
    )
    payload = _payload(response)
    model = MapLifecycleResponse.model_validate(payload)

    assert response.status_code == 200
    assert model.ok is True
    assert model.success is True
    assert model.navigation_ready is False
    assert (map_dir / "demo" / "map.pcd").is_file()
    assert (map_dir / "demo" / "metadata.json").is_file()
    assert not (map_dir / "demo" / "octomap.ot").exists()

    maps_payload = asyncio.run(_endpoint(gateway, "/api/v1/slam/maps")())
    maps = MapListResponse.model_validate(maps_payload)
    entry = next(item for item in maps.maps if item.name == "demo")
    assert entry.has_pcd is True
    assert entry.has_octomap is False
    assert entry.navigation_ready is False
    assert entry.state == "STALE"


def test_map_workbench_import_rejects_host_path_escape(monkeypatch, tmp_path):
    from gateway.gateway_module import GatewayModule

    import_root = tmp_path / "imports"
    import_root.mkdir()
    outside = tmp_path / "outside.pcd"
    outside.write_bytes(b"pcd")
    monkeypatch.setenv("LINGTU_MAP_IMPORT_DIR", str(import_root))

    gateway = GatewayModule()
    gateway.setup()
    gateway._map_mgr = _FakeMapManager()
    response = asyncio.run(
        _endpoint(gateway, "/api/v1/maps/import_pcd")({"name": "demo", "source_path": str(outside), "voxel_size": 0.0})
    )
    payload = _payload(response)

    assert response.status_code == 400
    assert payload["ok"] is False
    assert "escapes configured root" in payload["message"]


def test_maps_route_error_response_matches_openapi_contract():
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import GatewayErrorResponse, MapRequest

    gateway = GatewayModule()
    gateway.setup()
    manager = _FakeMapManager()
    gateway._map_mgr = manager

    def fail(request):
        command = request.to_mapping()
        manager.map_command.delivered.append(command)
        return {
            "action": command["action"],
            "success": False,
            "message": "map save failed",
        }

    manager.execute = fail

    response = asyncio.run(_endpoint(gateway, "/api/v1/maps")(MapRequest(action="save", name="demo")))
    payload = _payload(response)
    model = GatewayErrorResponse.model_validate(payload)

    assert response.status_code == 400
    assert payload["schema_version"] == 1
    assert payload["ok"] is False
    assert model.error == "map save failed"
    assert model.message == "map save failed"
    assert model.detail["action"] == "save"
    assert model.detail["success"] is False


def test_operational_routes_validate_idle_json_contracts():
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import (
        BagOperationResponse,
        BagStatusResponse,
        ExplorationStatusResponse,
        GatewayErrorResponse,
        SlamOperationResponse,
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
    slam_switch_response = asyncio.run(_endpoint(gateway, "/api/v1/slam/switch")({"profile": "bad"}))
    bag_status_payload = asyncio.run(_endpoint(gateway, "/api/v1/bag/status")())
    bag_stop_response = asyncio.run(_endpoint(gateway, "/api/v1/bag/stop")())

    temporal = TemporalMemoryResponse.model_validate(temporal_payload)
    semantic_ok = TemporalMemoryResponse.model_validate(semantic_payload)
    semantic = GatewayErrorResponse.model_validate(_payload(semantic_response))
    explore_status = ExplorationStatusResponse.model_validate(explore_status_payload)
    explore_start = GatewayErrorResponse.model_validate(_payload(explore_start_response))
    slam_status = SlamStatusResponse.model_validate(slam_status_payload)
    slam_switch = SlamOperationResponse.model_validate(_payload(slam_switch_response))
    bag_status = BagStatusResponse.model_validate(bag_status_payload)
    bag_stop = BagOperationResponse.model_validate(_payload(bag_stop_response))

    assert temporal.count == 1
    assert temporal.observations[0]["label"] == "door"
    assert semantic_ok.count == 1
    assert semantic.error == "embedding required"
    assert explore_status.available is False
    assert explore_status.can_start is False
    assert "explorer_backend_not_running" in explore_status.blockers
    assert explore_status.reason == "explorer_backend_not_running"
    assert explore_status.required_profile == "explore_or_tare_explore"
    assert explore_status.supported_profiles == ["explore", "tare_explore"]
    assert explore_start.error == "Exploration backend not running"
    assert explore_start.detail["reason"] == "explorer_backend_not_running"
    assert slam_status.mode in {
        "fastlio2",
        "genz",
        "localizer",
        "slam_only",
        "stopped",
        "super_lio",
        "super_lio_relocation",
    }
    assert slam_switch.schema_version == 1
    assert slam_switch.ok is False
    assert slam_switch.success is False
    assert slam_switch.ts > 0
    assert bag_status.recording is False
    assert bag_stop.error == "not_recording"


def test_tare_explorer_is_available_through_exploration_contracts():
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import (
        ExplorationCommandResponse,
        ExplorationStatusResponse,
    )

    class TAREExplorerModule:
        def __init__(self):
            self.started = False

        def start_tare_exploration(self):
            self.started = True
            return json.dumps({"status": "started"})

        def stop_tare_exploration(self):
            self.started = False
            return json.dumps({"status": "stopped"})

        def get_tare_status(self):
            return json.dumps(
                {
                    "alive": True,
                    "started": self.started,
                    "waypoint_count": 3,
                    "finished": False,
                }
            )

    gateway = GatewayModule()
    gateway.setup()
    tare = TAREExplorerModule()
    gateway.on_system_modules({"TAREExplorerModule": tare})
    _seed_ready_navigation(gateway)
    gateway._on_tare_stats({"runtime_ms": 12.5})
    gateway._on_exploration_supervisor({"mode": "idle", "reason": "ready"})

    status_payload = asyncio.run(_endpoint(gateway, "/api/v1/explore/status")())
    start_payload = asyncio.run(_endpoint(gateway, "/api/v1/explore/start")())
    running_payload = asyncio.run(_endpoint(gateway, "/api/v1/explore/status")())
    stop_payload = asyncio.run(_endpoint(gateway, "/api/v1/explore/stop")())
    stopped_payload = asyncio.run(_endpoint(gateway, "/api/v1/explore/status")())
    snapshot = gateway._session_snapshot()

    status = ExplorationStatusResponse.model_validate(status_payload)
    started = ExplorationCommandResponse.model_validate(start_payload)
    running = ExplorationStatusResponse.model_validate(running_payload)
    stopped = ExplorationCommandResponse.model_validate(stop_payload)
    final_status = ExplorationStatusResponse.model_validate(stopped_payload)

    assert status.available is True
    assert status.backend == "tare"
    assert status.can_start is True
    assert status.blockers == []
    assert status.exploring is False
    assert status.tare["status"]["alive"] is True
    assert status.tare["status"]["waypoint_count"] == 3
    assert status.tare["stats"]["runtime_ms"] == 12.5
    assert status.supervisor["mode"] == "idle"
    assert started.status["status"] == "started"
    assert running.exploring is True
    assert running.can_start is False
    assert "exploration_already_active" in running.blockers
    assert stopped.status["status"] == "stopped"
    assert final_status.exploring is False
    assert snapshot["explorer_backend"] == "tare"
    assert snapshot["explorer_available"] is True
    assert snapshot["explorer_unavailable_reason"] is None
    assert snapshot["explorer_required_profile"] is None
    assert snapshot["can_start_exploring"] is True
    assert snapshot["exploration_blockers"] == []


def test_wavefront_explorer_is_available_through_exploration_contracts():
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import (
        ExplorationCommandResponse,
        ExplorationStatusResponse,
    )

    class WavefrontFrontierExplorer:
        def __init__(self):
            self.started = False

        def begin_exploration(self):
            self.started = True
            return {"status": "started", "backend": "frontier"}

        def end_exploration(self):
            self.started = False
            return {"status": "stopped", "backend": "frontier"}

        def health(self):
            return {"frontier_count": 7}

    gateway = GatewayModule()
    gateway.setup()
    explorer = WavefrontFrontierExplorer()
    gateway.on_system_modules({"WavefrontFrontierExplorer": explorer})
    _seed_ready_navigation(gateway)

    status_payload = asyncio.run(_endpoint(gateway, "/api/v1/explore/status")())
    start_payload = asyncio.run(_endpoint(gateway, "/api/v1/explore/start")())
    running_payload = asyncio.run(_endpoint(gateway, "/api/v1/explore/status")())
    stop_payload = asyncio.run(_endpoint(gateway, "/api/v1/explore/stop")())
    stopped_payload = asyncio.run(_endpoint(gateway, "/api/v1/explore/status")())

    status = ExplorationStatusResponse.model_validate(status_payload)
    started = ExplorationCommandResponse.model_validate(start_payload)
    running = ExplorationStatusResponse.model_validate(running_payload)
    stopped = ExplorationCommandResponse.model_validate(stop_payload)
    final_status = ExplorationStatusResponse.model_validate(stopped_payload)

    assert status.available is True
    assert status.backend == "frontier"
    assert status.can_start is True
    assert status.blockers == []
    assert status.frontier_count == 7
    assert status.exploring is False
    assert started.status == {"status": "started", "backend": "frontier"}
    assert running.exploring is True
    assert running.can_start is False
    assert "exploration_already_active" in running.blockers
    assert stopped.status == {"status": "stopped", "backend": "frontier"}
    assert final_status.exploring is False
    assert gateway._session_snapshot()["explorer_backend"] == "frontier"


def test_exploration_readiness_ignores_inactive_navigation_session_only():
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import ExplorationStatusResponse

    class WavefrontFrontierExplorer:
        def __init__(self):
            self.started = False

        def begin_exploration(self):
            self.started = True
            return {"status": "started", "backend": "frontier"}

        def health(self):
            return {"frontier_count": 2}

    gateway = GatewayModule()
    gateway.setup()
    explorer = WavefrontFrontierExplorer()
    gateway.on_system_modules({"WavefrontFrontierExplorer": explorer})
    _seed_ready_navigation(gateway)

    ready_payload = asyncio.run(_endpoint(gateway, "/api/v1/explore/status")())
    ready = ExplorationStatusResponse.model_validate(ready_payload)

    assert gateway._session_mode == "idle"
    assert ready.can_start is True
    assert "navigation_session_inactive" not in ready.blockers
    assert ready.blockers == []

    with gateway._state_lock:
        gateway._localization_status["recovery_signal"] = "LOC_DIVERGED"

    blocked_payload = asyncio.run(_endpoint(gateway, "/api/v1/explore/status")())
    blocked = ExplorationStatusResponse.model_validate(blocked_payload)

    assert blocked.can_start is False
    assert "navigation_session_inactive" not in blocked.blockers
    assert blocked.blockers == ["localization_recovery_active"]
    assert explorer.started is False


def test_explore_start_rejects_localization_recovery_blocker():
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import ExplorationStatusResponse, GatewayErrorResponse

    class WavefrontFrontierExplorer:
        def __init__(self):
            self.started = False

        def begin_exploration(self):
            self.started = True
            return {"status": "started"}

        def health(self):
            return {"frontier_count": 2}

    gateway = GatewayModule()
    gateway.setup()
    explorer = WavefrontFrontierExplorer()
    gateway.on_system_modules({"WavefrontFrontierExplorer": explorer})
    _seed_ready_navigation(gateway)
    with gateway._state_lock:
        gateway._localization_status["recovery_signal"] = "LOC_DIVERGED"

    status_payload = asyncio.run(_endpoint(gateway, "/api/v1/explore/status")())
    start_response = asyncio.run(_endpoint(gateway, "/api/v1/explore/start")())

    status = ExplorationStatusResponse.model_validate(status_payload)
    error = GatewayErrorResponse.model_validate(_payload(start_response))

    assert status.available is True
    assert status.can_start is False
    assert "localization_recovery_active" in status.blockers
    assert start_response.status_code == 409
    assert error.error == "exploration_not_ready"
    assert "localization_recovery_active" in error.detail["blockers"]
    assert explorer.started is False
    assert gateway._exploring is False
    snapshot = gateway._session_snapshot()
    assert snapshot["can_start_exploring"] is False
    assert snapshot["exploration_blockers"] == ["localization_recovery_active"]


def test_explore_start_rejects_safety_stop_before_backend_start():
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import ExplorationStatusResponse, GatewayErrorResponse

    class WavefrontFrontierExplorer:
        def __init__(self):
            self.started = False

        def begin_exploration(self):
            self.started = True
            return {"status": "started"}

        def health(self):
            return {"frontier_count": 2}

    gateway = GatewayModule()
    gateway.setup()
    explorer = WavefrontFrontierExplorer()
    gateway.on_system_modules({"WavefrontFrontierExplorer": explorer})
    _seed_ready_navigation(gateway)
    with gateway._state_lock:
        gateway._safety = {"level": 2}

    status_payload = asyncio.run(_endpoint(gateway, "/api/v1/explore/status")())
    start_response = asyncio.run(_endpoint(gateway, "/api/v1/explore/start")())

    status = ExplorationStatusResponse.model_validate(status_payload)
    error = GatewayErrorResponse.model_validate(_payload(start_response))

    assert status.available is True
    assert status.can_start is False
    assert "safety_stop" in status.blockers
    assert start_response.status_code == 409
    assert error.error == "exploration_not_ready"
    assert "safety_stop" in error.detail["blockers"]
    assert explorer.started is False
    assert gateway._exploring is False
    snapshot = gateway._session_snapshot()
    assert snapshot["can_start_exploring"] is False
    assert "safety_stop" in snapshot["exploration_blockers"]
    assert snapshot["safety_clear"] is False
    assert snapshot["safety"]["stop_active"] is True


def test_exploring_session_start_rejects_localization_recovery_blocker(monkeypatch):
    import runtime.service_manager as service_manager
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import SessionTransitionResponse

    class FakeServiceManager:
        def __init__(self):
            self.calls: list[tuple[str, tuple[str, ...]]] = []

        def stop(self, *services: str) -> None:
            self.calls.append(("stop", services))

        def ensure(self, *services: str) -> None:
            self.calls.append(("ensure", services))

        def wait_ready(self, *services: str, timeout: float = 15.0) -> bool:
            self.calls.append(("wait_ready", services))
            return True

    class TAREExplorerModule:
        def __init__(self):
            self.started = False

        def start_tare_exploration(self):
            self.started = True
            return {"status": "started"}

        def get_tare_status(self):
            return {"started": self.started}

    fake_service_manager = FakeServiceManager()
    monkeypatch.setattr(service_manager, "get_service_manager", lambda: fake_service_manager)

    gateway = GatewayModule()
    gateway.setup()
    tare = TAREExplorerModule()
    gateway.on_system_modules({"TAREExplorerModule": tare})
    _seed_ready_navigation(gateway)
    with gateway._state_lock:
        gateway._localization_status["recovery_signal"] = "LOC_DIVERGED"

    response = asyncio.run(_endpoint(gateway, "/api/v1/session/start")({"mode": "exploring"}))
    started = SessionTransitionResponse.model_validate(_payload(response))

    assert response.status_code == 409
    assert started.ok is False
    assert started.success is False
    assert "localization_recovery_active" in (started.message or "")
    assert started.detail["blockers"] == ["localization_recovery_active"]
    assert fake_service_manager.calls == []
    assert tare.started is False
    assert gateway._session_mode == "idle"


def test_exploring_session_start_rejects_safety_stop_before_backend_start():
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import SessionTransitionResponse

    class TAREExplorerModule:
        def __init__(self):
            self.started = False

        def start_tare_exploration(self):
            self.started = True
            return {"status": "started"}

        def get_tare_status(self):
            return {"started": self.started}

    gateway = GatewayModule()
    gateway.setup()
    tare = TAREExplorerModule()
    gateway.on_system_modules({"TAREExplorerModule": tare})
    _seed_ready_navigation(gateway)
    with gateway._state_lock:
        gateway._safety = {"level": 2}

    response = asyncio.run(_endpoint(gateway, "/api/v1/session/start")({"mode": "exploring"}))
    started = SessionTransitionResponse.model_validate(_payload(response))

    assert response.status_code == 409
    assert started.ok is False
    assert started.success is False
    assert "safety_stop" in (started.message or "")
    assert started.detail["blockers"] == ["safety_stop"]
    assert tare.started is False
    assert gateway._session_mode == "idle"


def test_tare_explorer_session_start_end_uses_exploration_backend(monkeypatch):
    import runtime.service_manager as service_manager
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import SessionTransitionResponse

    class FakeServiceManager:
        def __init__(self):
            self.calls: list[tuple[str, tuple[str, ...]]] = []

        def stop(self, *services: str) -> None:
            self.calls.append(("stop", services))

        def ensure(self, *services: str) -> None:
            self.calls.append(("ensure", services))

        def wait_ready(self, *services: str, timeout: float = 15.0) -> bool:
            self.calls.append(("wait_ready", services))
            return True

    class TAREExplorerModule:
        def __init__(self):
            self.started = False
            self.start_count = 0
            self.stop_count = 0

        def start_tare_exploration(self):
            self.started = True
            self.start_count += 1
            return {"status": "started"}

        def stop_tare_exploration(self):
            self.started = False
            self.stop_count += 1
            return {"status": "stopped"}

        def get_tare_status(self):
            return {"started": self.started}

    fake_service_manager = FakeServiceManager()
    monkeypatch.setattr(service_manager, "get_service_manager", lambda: fake_service_manager)

    gateway = GatewayModule()
    gateway.setup()
    tare = TAREExplorerModule()
    gateway.on_system_modules({"TAREExplorerModule": tare})
    _seed_ready_navigation(gateway)

    start_payload = asyncio.run(_endpoint(gateway, "/api/v1/session/start")({"mode": "exploring"}))
    started = SessionTransitionResponse.model_validate(start_payload)

    assert started.ok is True
    assert started.success is True
    assert started.session is not None
    assert started.session.mode == "exploring"
    assert started.session.slam_profile == "native_dds"
    assert started.session.explorer_backend == "tare"
    assert tare.started is True
    assert tare.start_count == 1
    assert (
        "stop",
        ("slam_pgo", "localizer", "hba", "genz_icp", "super_lio", "super_lio_relocation"),
    ) in fake_service_manager.calls
    assert ("ensure", ("slam",)) in fake_service_manager.calls
    assert ("wait_ready", ("slam",)) in fake_service_manager.calls

    end_payload = asyncio.run(_endpoint(gateway, "/api/v1/session/end")())
    ended = SessionTransitionResponse.model_validate(end_payload)

    assert ended.ok is True
    assert ended.success is True
    assert ended.session is not None
    assert ended.session.mode == "idle"
    assert ended.session.explorer_backend == "tare"
    assert tare.started is False
    assert tare.stop_count == 1
    assert (
        "stop",
        (
            "super_lio_relocation",
            "super_lio",
            "hba",
            "genz_icp",
            "slam_pgo",
            "localizer",
            "slam",
        ),
    ) in fake_service_manager.calls


def test_tare_external_session_start_with_none_skips_robot_slam_services(monkeypatch):
    import runtime.service_manager as service_manager
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import SessionTransitionResponse

    class FakeServiceManager:
        def __init__(self):
            self.calls: list[tuple[str, tuple[str, ...]]] = []

        def stop(self, *services: str) -> None:
            self.calls.append(("stop", services))

        def ensure(self, *services: str) -> None:
            self.calls.append(("ensure", services))

        def wait_ready(self, *services: str, timeout: float = 15.0) -> bool:
            self.calls.append(("wait_ready", services))
            return True

    class TAREExplorerModule:
        def __init__(self):
            self.started = False

        def start_tare_exploration(self):
            self.started = True
            return {"status": "started"}

        def stop_tare_exploration(self):
            self.started = False
            return {"status": "stopped"}

        def get_tare_status(self):
            return {"started": self.started}

    fake_service_manager = FakeServiceManager()
    monkeypatch.setattr(service_manager, "get_service_manager", lambda: fake_service_manager)

    gateway = GatewayModule()
    gateway.setup()
    tare = TAREExplorerModule()
    gateway.on_system_modules({"TAREExplorerModule": tare})
    _seed_ready_navigation(gateway)

    payload = asyncio.run(_endpoint(gateway, "/api/v1/session/start")({"mode": "exploring", "slam_profile": "none"}))
    transition = SessionTransitionResponse.model_validate(payload)

    assert transition.ok is True
    assert transition.session is not None
    assert transition.session.mode == "exploring"
    assert transition.session.slam_profile == "none"
    assert tare.started is True
    assert fake_service_manager.calls == [("stop", ())]


def test_tare_external_session_get_preserves_exploring_with_none_slam(monkeypatch):
    import runtime.service_manager as service_manager
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import SessionResponse

    class FakeServiceManager:
        def status(self, *services: str) -> dict[str, str]:
            return {name: "stopped" for name in services}

    class TAREExplorerModule:
        def get_tare_status(self):
            return {"started": True}

    monkeypatch.setattr(service_manager, "get_service_manager", lambda: FakeServiceManager())

    gateway = GatewayModule()
    gateway.setup()
    gateway.on_system_modules({"TAREExplorerModule": TAREExplorerModule()})
    gateway._session_mode = "exploring"
    gateway._session_slam_profile = "none"
    gateway._cached_slam_profile = "none"
    gateway._slam_profile_ts = 0.0
    gateway._exploring = True
    with gateway._state_lock:
        gateway._localization_status = {"backend": "none"}

    payload = asyncio.run(_endpoint(gateway, "/api/v1/session")())
    session = SessionResponse.model_validate(payload)

    assert session.mode == "exploring"
    assert session.slam_profile == "none"
    assert gateway._session_mode == "exploring"
    assert gateway._cached_slam_profile == "none"


def test_slam_status_uses_logical_service_states(monkeypatch):
    import runtime.service_manager as service_manager
    from gateway.gateway_module import GatewayModule

    class _FakeServiceManager:
        def __init__(self, services):
            self._services = services

        def status(self, *names):
            assert names == (
                "lidar",
                "slam",
                "traversability",
                "nav",
                "explore",
                "slam_pgo",
                "localizer",
                "genz_icp",
                "hba",
                "super_lio",
                "super_lio_relocation",
            )
            return dict(self._services)

        def status_details(self, *names):
            canonical = {
                "lidar": "lingtu-livox-dds.service",
                "slam": "lingtu-slam-dds.service",
                "traversability": "lingtu-traversability-dds.service",
                "nav": "lingtu-nav-dds.service",
                "explore": "lingtu-explore-dds.service",
            }
            return {
                name: {
                    "status": self._services.get(name, "unknown"),
                    "canonical_unit": canonical.get(name, f"{name}.service"),
                    "selected_unit": canonical.get(name, f"{name}.service"),
                    "installed_units": [canonical.get(name, f"{name}.service")],
                    "active_units": (
                        [canonical.get(name, f"{name}.service")] if self._services.get(name) == "running" else []
                    ),
                    "candidate_units": [f"{name}.service"],
                }
                for name in names
            }

    gateway = GatewayModule()
    gateway.setup()
    endpoint = _endpoint(gateway, "/api/v1/slam/status")

    gateway._localization_status = {
        "backend": "cpp_dds_slam",
        "mode": "localization",
        "state": "TRACKING",
    }
    monkeypatch.setattr(
        service_manager,
        "get_service_manager",
        lambda: _FakeServiceManager(
            {
                "lidar": "running",
                "slam": "running",
                "traversability": "running",
                "nav": "running",
                "explore": "stopped",
                "slam_pgo": "stopped",
                "localizer": "stopped",
                "genz_icp": "stopped",
                "hba": "stopped",
                "super_lio": "stopped",
                "super_lio_relocation": "stopped",
            }
        ),
    )
    native_localizer_payload = asyncio.run(endpoint())
    assert native_localizer_payload["mode"] == "native_dds"
    assert native_localizer_payload["native_mode"] == "localization"
    assert native_localizer_payload["product_runtime"] == "native_dds"
    assert native_localizer_payload["ros2_required"] is False
    assert native_localizer_payload["manual_systemctl_required"] is False
    assert native_localizer_payload["service_groups"]["native_dds"] == [
        "lidar",
        "slam",
        "traversability",
        "nav",
        "driver",
        "explore",
    ]
    assert "legacy_ros2_compat" in native_localizer_payload["service_groups"]
    assert native_localizer_payload["service_metadata"]["slam_pgo"]["role"] == "map_optimization"
    assert native_localizer_payload["service_metadata"]["slam_pgo"]["ros2_compat"] is True
    assert native_localizer_payload["service_metadata"]["hba"]["role"] == "map_optimization"
    assert native_localizer_payload["service_metadata"]["hba"]["experimental"] is True
    assert native_localizer_payload["service_details"]["slam"]["status"] == "running"

    gateway._localization_status = {}
    monkeypatch.setattr(
        service_manager,
        "get_service_manager",
        lambda: _FakeServiceManager(
            {
                "lidar": "running",
                "slam": "stopped",
                "traversability": "running",
                "nav": "running",
                "explore": "stopped",
                "slam_pgo": "stopped",
                "localizer": "running",
                "genz_icp": "stopped",
                "hba": "stopped",
                "super_lio": "stopped",
                "super_lio_relocation": "stopped",
            }
        ),
    )
    localizer_payload = asyncio.run(endpoint())
    assert localizer_payload["mode"] == "localizer"
    assert localizer_payload["services"]["localizer"] == "running"

    monkeypatch.setattr(
        service_manager,
        "get_service_manager",
        lambda: _FakeServiceManager(
            {
                "lidar": "running",
                "slam": "running",
                "traversability": "running",
                "nav": "running",
                "explore": "stopped",
                "slam_pgo": "stopped",
                "localizer": "stopped",
                "genz_icp": "stopped",
                "hba": "stopped",
                "super_lio": "stopped",
                "super_lio_relocation": "stopped",
            }
        ),
    )
    fastlio_payload = asyncio.run(endpoint())
    assert fastlio_payload["mode"] == "native_dds"

    monkeypatch.setattr(
        service_manager,
        "get_service_manager",
        lambda: _FakeServiceManager(
            {
                "lidar": "running",
                "slam": "stopped",
                "traversability": "running",
                "nav": "running",
                "explore": "stopped",
                "slam_pgo": "stopped",
                "localizer": "stopped",
                "genz_icp": "stopped",
                "hba": "stopped",
                "super_lio": "running",
                "super_lio_relocation": "stopped",
            }
        ),
    )
    super_lio_payload = asyncio.run(endpoint())
    assert super_lio_payload["mode"] == "super_lio"
    assert super_lio_payload["services"]["super_lio"] == "running"

    monkeypatch.setattr(
        service_manager,
        "get_service_manager",
        lambda: _FakeServiceManager(
            {
                "lidar": "running",
                "slam": "stopped",
                "traversability": "running",
                "nav": "running",
                "explore": "stopped",
                "slam_pgo": "stopped",
                "localizer": "stopped",
                "genz_icp": "running",
                "hba": "stopped",
                "super_lio": "stopped",
                "super_lio_relocation": "stopped",
            }
        ),
    )
    genz_payload = asyncio.run(endpoint())
    assert genz_payload["mode"] == "genz"
    assert genz_payload["services"]["genz_icp"] == "running"

    monkeypatch.setattr(
        service_manager,
        "get_service_manager",
        lambda: _FakeServiceManager(
            {
                "lidar": "running",
                "slam": "stopped",
                "traversability": "running",
                "nav": "running",
                "explore": "stopped",
                "slam_pgo": "stopped",
                "localizer": "stopped",
                "genz_icp": "stopped",
                "hba": "stopped",
                "super_lio": "stopped",
                "super_lio_relocation": "running",
            }
        ),
    )
    relocation_payload = asyncio.run(endpoint())
    assert relocation_payload["mode"] == "super_lio_relocation"
    assert relocation_payload["services"]["super_lio_relocation"] == "running"


def test_service_status_exposes_catalog_readiness_contract(monkeypatch):
    import runtime.service_manager as service_manager
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import ServiceStatusResponse
    from runtime.service_catalogs.thunder import thunder_service_metadata

    metadata = thunder_service_metadata()

    class _FakeServiceManager:
        def status(self, *names):
            assert names == ("camera", "lidar")
            return {"camera": "running", "lidar": "stopped"}

        def status_details(self, *names):
            assert names == ("camera", "lidar")
            return {
                name: {
                    "status": "running" if name == "camera" else "stopped",
                    "ready": False,
                    "blockers": ["dds_unchecked"] if name == "camera" else ["systemd_inactive", "dds_unchecked"],
                    "observed": {
                        "systemd": name == "camera",
                        "native_binary": {
                            "ok": True,
                            "binaries": metadata[name].get("binaries", []),
                            "blockers": [],
                        },
                        "status_file": {
                            "ok": name == "camera",
                            "files": [
                                {
                                    "path": "/dev/shm/lingtu/camera_status.json",
                                    "exists": name == "camera",
                                }
                            ],
                            "blockers": []
                            if name == "camera"
                            else ["status_file_missing:/dev/shm/lingtu/camera_status.json"],
                        },
                        "dds": {
                            "ok": False,
                            "checked": False,
                            "enabled": False,
                            "reason": "set LINGTU_SERVICE_DDS_CHECK=1 to sample DDS topics",
                            "topics": metadata[name]["dds_topics"],
                            "samples": {},
                            "blockers": ["dds_unchecked"],
                        },
                    },
                    "contract": {
                        "checks": metadata[name]["checks"],
                        "topics": metadata[name]["topics"],
                        "dds_topics": metadata[name]["dds_topics"],
                        "files": metadata[name]["files"],
                        "binaries": metadata[name].get("binaries", []),
                    },
                }
                for name in names
            }

    monkeypatch.setattr(service_manager, "get_service_manager", lambda: _FakeServiceManager())

    gateway = GatewayModule()
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
    assert status.service_details["camera"]["observed"]["systemd"] is True
    assert status.service_details["camera"]["observed"]["native_binary"]["ok"] is True
    assert status.service_details["camera"]["observed"]["status_file"]["ok"] is True
    assert status.service_details["camera"]["observed"]["dds"] == {
        "ok": False,
        "checked": False,
        "enabled": False,
        "reason": "set LINGTU_SERVICE_DDS_CHECK=1 to sample DDS topics",
        "topics": ["rt/camera/info"],
        "samples": {},
        "blockers": ["dds_unchecked"],
    }
    assert status.service_details["camera"]["ready"] is False
    assert status.service_details["camera"]["blockers"] == ["dds_unchecked"]
    assert status.service_details["lidar"]["blockers"] == [
        "systemd_inactive",
        "dds_unchecked",
    ]


def test_service_status_default_names_follow_field_readiness_catalog(monkeypatch):
    import runtime.service_manager as service_manager
    from gateway.gateway_module import GatewayModule
    from runtime.service_catalogs.thunder import thunder_field_readiness_services

    expected = tuple(dict.fromkeys((*thunder_field_readiness_services(), "gateway")))

    class _FakeServiceManager:
        def status(self, *names):
            assert names == expected
            return {name: "running" for name in names}

        def status_details(self, *names):
            assert names == expected
            return {
                name: {
                    "status": "running",
                    "ready": True,
                    "blockers": [],
                    "observed": {"systemd": True},
                    "contract": {
                        "checks": ["systemd"],
                        "topics": [],
                        "dds_topics": [],
                        "files": [],
                    },
                }
                for name in names
            }

    monkeypatch.setattr(service_manager, "get_service_manager", lambda: _FakeServiceManager())

    gateway = GatewayModule()
    gateway.setup()
    endpoint = _endpoint(gateway, "/api/v1/services/status")

    payload = asyncio.run(endpoint())

    assert tuple(payload["services"]) == expected
    assert "explore" not in payload["services"]
    assert "camera" in payload["service_metadata"]
    assert "gateway" in payload["service_metadata"]


def test_slam_switch_can_select_super_lio(monkeypatch):
    import runtime.service_manager as service_manager
    from gateway.gateway_module import GatewayModule

    class _FakeServiceManager:
        def __init__(self):
            self.calls: list[tuple[str, tuple[str, ...]]] = []

        def stop(self, *names):
            self.calls.append(("stop", names))

        def ensure(self, *names):
            self.calls.append(("ensure", names))

        def wait_ready(self, *names, timeout: float = 15.0):
            self.calls.append(("wait_ready", names))
            return True

    fake = _FakeServiceManager()
    monkeypatch.setattr(service_manager, "get_service_manager", lambda: fake)

    gateway = GatewayModule()
    gateway.setup()
    endpoint = _endpoint(gateway, "/api/v1/slam/switch")

    payload = asyncio.run(endpoint({"profile": "super_lio"}))

    assert payload["schema_version"] == 1
    assert payload["ok"] is True
    assert payload["success"] is True
    assert payload["ts"] > 0
    assert payload["profile"] == "super_lio"
    assert (
        "stop",
        ("slam", "slam_pgo", "localizer", "hba", "genz_icp", "super_lio_relocation"),
    ) in fake.calls
    assert ("ensure", ("legacy_lidar", "super_lio")) in fake.calls
    assert ("wait_ready", ("legacy_lidar", "super_lio")) in fake.calls


def test_slam_switch_can_select_genz_icp(monkeypatch):
    import runtime.service_manager as service_manager
    from gateway.gateway_module import GatewayModule

    class _FakeServiceManager:
        def __init__(self):
            self.calls: list[tuple[str, tuple[str, ...]]] = []

        def stop(self, *names):
            self.calls.append(("stop", names))

        def ensure(self, *names):
            self.calls.append(("ensure", names))

        def wait_ready(self, *names, timeout: float = 15.0):
            self.calls.append(("wait_ready", names))
            return True

    fake = _FakeServiceManager()
    monkeypatch.setattr(service_manager, "get_service_manager", lambda: fake)

    gateway = GatewayModule()
    gateway.setup()
    endpoint = _endpoint(gateway, "/api/v1/slam/switch")

    payload = asyncio.run(endpoint({"profile": "genz-icp"}))

    assert payload["schema_version"] == 1
    assert payload["ok"] is True
    assert payload["success"] is True
    assert payload["profile"] == "genz"
    assert (
        "stop",
        ("slam", "slam_pgo", "localizer", "hba", "super_lio", "super_lio_relocation"),
    ) in fake.calls
    assert ("ensure", ("legacy_lidar", "genz_icp")) in fake.calls
    assert ("wait_ready", ("legacy_lidar", "genz_icp")) in fake.calls


def test_slam_switch_can_select_super_lio_relocation(monkeypatch):
    import runtime.service_manager as service_manager
    from gateway.gateway_module import GatewayModule

    class _FakeServiceManager:
        def __init__(self):
            self.calls: list[tuple[str, tuple[str, ...]]] = []

        def stop(self, *names):
            self.calls.append(("stop", names))

        def ensure(self, *names):
            self.calls.append(("ensure", names))

        def wait_ready(self, *names, timeout: float = 15.0):
            self.calls.append(("wait_ready", names))
            return True

    fake = _FakeServiceManager()
    monkeypatch.setattr(service_manager, "get_service_manager", lambda: fake)

    gateway = GatewayModule()
    gateway.setup()
    endpoint = _endpoint(gateway, "/api/v1/slam/switch")

    payload = asyncio.run(endpoint({"profile": "super_lio_reloc"}))

    assert payload["schema_version"] == 1
    assert payload["ok"] is True
    assert payload["success"] is True
    assert payload["ts"] > 0
    assert payload["profile"] == "super_lio_relocation"
    assert (
        "stop",
        ("slam", "slam_pgo", "localizer", "hba", "genz_icp", "super_lio"),
    ) in fake.calls
    assert ("ensure", ("legacy_lidar", "super_lio_relocation")) in fake.calls
    assert ("wait_ready", ("legacy_lidar", "super_lio_relocation")) in fake.calls


def test_super_lio_relocalize_endpoints_fail_fast_without_ros_call(monkeypatch):
    import subprocess

    from gateway.gateway_module import GatewayModule

    def fail_run(*_args, **_kwargs):
        raise AssertionError("ROS relocalization service should not be called")

    monkeypatch.setattr(subprocess, "run", fail_run)

    gateway = GatewayModule()
    gateway.setup()
    gateway._localization_status = {
        "localization_backend": "super_lio",
        "saved_map_relocalization_supported": False,
        "recovery_method": "restart_super_lio",
    }

    auto_response = asyncio.run(_endpoint(gateway, "/api/v1/slam/auto_relocalize")())
    relocalize_response = asyncio.run(
        _endpoint(gateway, "/api/v1/slam/relocalize")({"map_name": "demo", "x": 1.0, "y": 2.0, "yaw": 0.3})
    )

    auto_payload = _payload(auto_response)
    relocalize_payload = _payload(relocalize_response)
    assert auto_response.status_code == 409
    assert relocalize_response.status_code == 409
    assert auto_payload["schema_version"] == 1
    assert relocalize_payload["schema_version"] == 1
    assert auto_payload["ok"] is False
    assert relocalize_payload["ok"] is False
    assert auto_payload["success"] is False
    assert relocalize_payload["success"] is False
    assert auto_payload["ts"] > 0
    assert relocalize_payload["ts"] > 0
    assert "unsupported" in auto_payload["message"]
    assert "unsupported" in relocalize_payload["message"]


def test_super_lio_relocation_relocalize_endpoints_fail_fast_without_ros_call(
    monkeypatch,
):
    import subprocess

    from gateway.gateway_module import GatewayModule

    def fail_run(*_args, **_kwargs):
        raise AssertionError("ROS relocalization service should not be called")

    monkeypatch.setattr(subprocess, "run", fail_run)

    gateway = GatewayModule()
    gateway.setup()
    gateway._localization_status = {
        "localization_backend": "super_lio_relocation",
        "saved_map_relocalization_supported": False,
        "recovery_method": "restart_super_lio_relocation",
    }

    auto_response = asyncio.run(_endpoint(gateway, "/api/v1/slam/auto_relocalize")())
    relocalize_response = asyncio.run(
        _endpoint(gateway, "/api/v1/slam/relocalize")({"map_name": "demo", "x": 1.0, "y": 2.0, "yaw": 0.3})
    )

    auto_payload = _payload(auto_response)
    relocalize_payload = _payload(relocalize_response)
    assert auto_response.status_code == 409
    assert relocalize_response.status_code == 409
    assert auto_payload["schema_version"] == 1
    assert relocalize_payload["schema_version"] == 1
    assert auto_payload["ok"] is False
    assert relocalize_payload["ok"] is False
    assert auto_payload["success"] is False
    assert relocalize_payload["success"] is False
    assert auto_payload["ts"] > 0
    assert relocalize_payload["ts"] > 0
    assert "unsupported" in auto_payload["message"]
    assert "unsupported" in relocalize_payload["message"]


def test_localizer_relocalize_passes_saved_map_path_to_service(monkeypatch, tmp_path):
    import subprocess

    from gateway.gateway_module import GatewayModule
    from localization.service import RelocalizationResult

    map_dir = tmp_path / "maps"
    (map_dir / "demo").mkdir(parents=True)
    (map_dir / "demo" / "map.pcd").write_text("pcd", encoding="utf-8")
    monkeypatch.setenv("NAV_MAP_DIR", str(map_dir))

    gateway = GatewayModule()
    gateway.setup()
    _attach_test_maps_service(gateway, map_dir)
    gateway._localization_status = {
        "backend": "localizer",
        "saved_map_relocalization_supported": True,
    }
    gateway._persist_last_nav_pose = lambda *_args, **_kwargs: None
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
        _endpoint(gateway, "/api/v1/slam/relocalize")({"map_name": "demo", "x": 1.0, "y": 2.0, "yaw": 0.3})
    )

    assert payload["schema_version"] == 1
    assert payload["ok"] is True
    assert payload["success"] is True
    assert payload["ts"] > 0
    assert payload["message"] == "Relocalized to demo"
    assert service.saved_calls == [(map_dir / "demo" / "map.pcd", 1.0, 2.0, 0.3, 30.0)]


def test_auto_relocalize_delegates_to_service_and_preserves_success_payload(
    monkeypatch,
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
        "backend": "localizer",
        "saved_map_relocalization_supported": True,
    }
    gateway._get_slam_profile = lambda: "localizer"
    service = _FakeRelocalizationService(
        global_result=RelocalizationResult(True, "native_global_relocalized", quality=0.04)
    )
    gateway.localization.bind(service)

    monkeypatch.setattr(subprocess, "run", fail_run)
    payload = asyncio.run(_endpoint(gateway, "/api/v1/slam/auto_relocalize")())

    assert service.global_calls == [10.0]
    assert subprocess_calls == []
    assert payload["schema_version"] == 1
    assert payload["ok"] is True
    assert payload["success"] is True
    assert payload["ts"] > 0
    assert payload["message"] == "native_global_relocalized"
    assert payload["quality"] == 0.04


def test_relocalize_delegates_validated_request_and_persists_on_success(
    monkeypatch,
    tmp_path,
):
    import subprocess

    from gateway.gateway_module import GatewayModule
    from localization.service import RelocalizationResult

    persisted = []
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
    _attach_test_maps_service(gateway, map_dir)
    gateway._localization_status = {
        "backend": "localizer",
        "saved_map_relocalization_supported": True,
    }
    gateway._get_slam_profile = lambda: "localizer"
    gateway._persist_last_nav_pose = lambda *args: persisted.append(args)
    service = _FakeRelocalizationService(saved_result=RelocalizationResult(True, "service ok", quality=0.123))
    gateway.localization.bind(service)

    monkeypatch.setattr(subprocess, "run", fail_run)
    payload = asyncio.run(
        _endpoint(gateway, "/api/v1/slam/relocalize")({"map_name": "demo", "x": 1.0, "y": 2.0, "yaw": 0.3})
    )

    assert service.saved_calls == [(map_dir / "demo" / "map.pcd", 1.0, 2.0, 0.3, 30.0)]
    assert persisted == [("demo", 1.0, 2.0, 0.3, 0.123)]
    assert subprocess_calls == []
    assert payload["schema_version"] == 1
    assert payload["ok"] is True
    assert payload["success"] is True
    assert payload["ts"] > 0
    assert payload["message"] == "Relocalized to demo"
    assert payload["quality"] == 0.123


def test_track_against_map_delegates_validated_request_to_service(
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
    _attach_test_maps_service(gateway, map_dir)
    gateway._localization_status = {
        "backend": "localizer",
        "saved_map_relocalization_supported": True,
    }
    gateway._get_slam_profile = lambda: "localizer"
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
    payload = asyncio.run(
        _endpoint(gateway, "/api/v1/slam/track_against_map")({"map_name": "demo", "x": 1.0, "y": 2.0, "yaw": 0.3})
    )

    assert service.track_calls == [(map_dir / "demo" / "map.pcd", 1.0, 2.0, 0.3, 10.0)]
    assert subprocess_calls == []
    assert payload["schema_version"] == 1
    assert payload["ok"] is True
    assert payload["success"] is True
    assert payload["ts"] > 0
    assert payload["message"] == "track_against_map_started"
    assert payload["quality"] == 0.2
    assert payload["details"] == {"track_against_map_enabled": True}


def test_relocalize_does_not_persist_last_pose_when_service_reports_failure(
    monkeypatch,
    tmp_path,
):
    from gateway.gateway_module import GatewayModule
    from localization.service import RelocalizationResult

    persisted = []
    map_dir = tmp_path / "maps"
    (map_dir / "demo").mkdir(parents=True)
    (map_dir / "demo" / "map.pcd").write_text("pcd", encoding="utf-8")
    monkeypatch.setenv("NAV_MAP_DIR", str(map_dir))

    gateway = GatewayModule()
    gateway.setup()
    _attach_test_maps_service(gateway, map_dir)
    gateway._localization_status = {
        "backend": "localizer",
        "saved_map_relocalization_supported": True,
    }
    gateway._persist_last_nav_pose = lambda *args: persisted.append(args)
    gateway.localization.bind(_FakeRelocalizationService(
        saved_result=RelocalizationResult(False, "service failed")
    ))

    payload = asyncio.run(
        _endpoint(gateway, "/api/v1/slam/relocalize")({"map_name": "demo", "x": 1.0, "y": 2.0, "yaw": 0.3})
    )

    assert persisted == []
    assert payload["schema_version"] == 1
    assert payload["ok"] is False
    assert payload["success"] is False
    assert payload["message"] == "service failed"


def test_relocalize_service_timeout_maps_to_504_payload(monkeypatch, tmp_path):
    from gateway.gateway_module import GatewayModule
    from localization.service import RelocalizationResult

    persisted = []
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
    _attach_test_maps_service(gateway, map_dir)
    gateway._localization_status = {
        "backend": "localizer",
        "saved_map_relocalization_supported": True,
    }
    gateway._persist_last_nav_pose = lambda *args: persisted.append(args)
    gateway.localization.bind(_FakeRelocalizationService(saved_result=timeout_result))

    response = asyncio.run(
        _endpoint(gateway, "/api/v1/slam/relocalize")({"map_name": "demo", "x": 1.0, "y": 2.0, "yaw": 0.3})
    )
    payload = _payload(response)

    assert response.status_code == 504
    assert persisted == []
    assert payload["schema_version"] == 1
    assert payload["ok"] is False
    assert payload["success"] is False
    assert payload["message"] == "call timeout > 30s"


def test_localizer_relocalize_rejects_unsafe_map_name(monkeypatch, tmp_path):
    import subprocess

    from gateway.gateway_module import GatewayModule

    def fail_run(*_args, **_kwargs):
        raise AssertionError("unsafe map names must not call ROS")

    monkeypatch.setenv("NAV_MAP_DIR", str(tmp_path / "maps"))
    monkeypatch.setattr(subprocess, "run", fail_run)

    gateway = GatewayModule()
    gateway.setup()
    gateway._localization_status = {
        "backend": "localizer",
        "saved_map_relocalization_supported": True,
    }

    response = asyncio.run(
        _endpoint(gateway, "/api/v1/slam/relocalize")({"map_name": "../outside", "x": 1.0, "y": 2.0, "yaw": 0.3})
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
