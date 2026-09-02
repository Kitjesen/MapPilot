# ruff: noqa: D101,D102,D103,S101
"""HTTP behavior slice for the local SimStudio API."""

from __future__ import annotations

import copy
from pathlib import Path
from types import SimpleNamespace
from typing import Any

import pytest
from tools.simstudio.http import API_PREFIX, create_app
from tools.simstudio.service.application import SimulationStudioService
from tools.simstudio.service.artifact_service import ArtifactService
from tools.simstudio.service.store import StudioStore

REPO_ROOT = Path(__file__).resolve().parents[3]


class FakeInteractiveSession:
    def __init__(self, *, artifact_root: Path, **_: Any) -> None:
        self.artifact_root = Path(artifact_root)
        self.calls: list[str] = []

    def prepare(self) -> dict[str, Any]:
        self.calls.append("prepare")
        self.artifact_root.joinpath("events.log").write_text("prepared\n", encoding="utf-8")
        return {
            "readiness": {"physics": "ready", "controllers": "ready"},
            "sensor_summary": {"imu": "ready", "truth_odom": "ready"},
        }

    def start(self) -> dict[str, Any]:
        self.calls.append("start")
        return {"readiness": {"physics": "running"}}

    def pause(self) -> dict[str, Any]:
        self.calls.append("pause")
        return {"readiness": {"physics": "paused"}}

    def reset(self) -> dict[str, Any]:
        self.calls.append("reset")
        return {"reset_generation": 1, "sensor_summary": {"imu": "ready"}}

    def stop(self) -> dict[str, Any]:
        self.calls.append("stop")
        return {"stopped": True}


def _intent() -> dict[str, Any]:
    return {
        "schema": "lingtu.sim.session-intent.v1",
        "session": {
            "session_id": "http_contract",
            "mujoco_version": "3.10.0",
            "seed": 7,
            "world": "open_field@1.0.0",
            "robots": [
                {
                    "instance_id": "robot_01",
                    "package": "thunderv4@1.0.3",
                    "sensor_rig": "thunderv4_truth_telemetry@1.0.0",
                    "controller": "thunderv4_locomotion@1.0.0",
                    "spawn": {
                        "position_m": [0.0, 0.0, 0.0],
                        "quaternion_wxyz": [1.0, 0.0, 0.0, 0.0],
                    },
                }
            ],
            "runtime": {
                "backend": "mujoco",
                "mode": "headless",
                "required_bindings": ["physics"],
            },
        },
    }


@pytest.fixture
def client(tmp_path: Path) -> Any:
    pytest.importorskip("fastapi")
    pytest.importorskip("httpx")
    from fastapi.testclient import TestClient

    sessions: list[FakeInteractiveSession] = []

    def session_factory(**kwargs: Any) -> FakeInteractiveSession:
        session = FakeInteractiveSession(**kwargs)
        sessions.append(session)
        return session

    service = SimulationStudioService.from_repository(
        REPO_ROOT,
        artifact_root=tmp_path / "studio",
        session_factory=session_factory,
    )
    app = create_app(service)
    with TestClient(app) as test_client:
        test_client.fake_sessions = sessions
        yield test_client


def _result(response: Any, status_code: int = 200) -> Any:
    assert response.status_code == status_code, response.text
    payload = response.json()
    assert payload["ok"] is True
    return payload["result"]


def _error(response: Any, status_code: int) -> dict[str, Any]:
    assert response.status_code == status_code, response.text
    payload = response.json()
    assert payload["ok"] is False
    assert payload["error"] == payload["diagnostics"][0]
    return payload["error"]


def _synthetic_ui_service(repo_root: Path) -> Any:
    dist = repo_root / "tools" / "simstudio" / "ui" / "dist"
    assets = dist / "assets"
    assets.mkdir(parents=True)
    (dist / "index.html").write_text(
        "<!doctype html><html><head><title>Synthetic SimStudio</title></head></html>",
        encoding="utf-8",
    )
    (assets / "app.js").write_text("console.log('synthetic');\n", encoding="utf-8")
    return SimpleNamespace(package_service=SimpleNamespace(repo_root=repo_root))


def test_http_lists_import_jobs() -> None:
    from fastapi.testclient import TestClient

    expected = [{"id": "import-01", "status": "READY"}]

    class PackageReadModel:
        def list_import_jobs(self) -> list[dict[str, str]]:
            return expected

    service = type("StudioReadModel", (), {"package_service": PackageReadModel()})()
    with TestClient(create_app(service)) as client:
        imports = _result(client.get(f"{API_PREFIX}/imports"))

    assert imports == expected


def test_http_serves_the_server_trusted_built_ui(tmp_path: Path) -> None:
    from fastapi.testclient import TestClient

    repo_root = tmp_path / "repo"
    service = _synthetic_ui_service(repo_root)
    asset_path = repo_root / "tools" / "simstudio" / "ui" / "dist" / "assets" / "app.js"
    with TestClient(create_app(service)) as client:
        index = client.get("/")
        asset = client.get(f"/assets/{asset_path.name}")

    assert index.status_code == 200
    assert index.text == (asset_path.parents[1] / "index.html").read_text(encoding="utf-8")
    assert asset.status_code == 200
    assert asset.content == asset_path.read_bytes()


def test_http_reports_a_clear_error_when_the_ui_is_not_built(tmp_path: Path) -> None:
    from fastapi.testclient import TestClient

    service = SimpleNamespace(package_service=SimpleNamespace(repo_root=tmp_path / "repo"))
    with TestClient(create_app(service)) as client:
        response = client.get("/")

    assert response.status_code == 503
    payload = response.json()
    assert payload["ok"] is False
    assert payload["error"]["code"] == "SIMSTUDIO_UI_NOT_BUILT"
    assert "dist/index.html" in payload["error"]["message"]


def test_http_rejects_asset_traversal_and_non_flat_paths(tmp_path: Path) -> None:
    from fastapi.testclient import TestClient

    repo_root = tmp_path / "repo"
    service = _synthetic_ui_service(repo_root)
    (repo_root / "tools" / "simstudio" / "ui" / "dist" / "secret.txt").write_text(
        "outside asset root\n", encoding="utf-8"
    )
    with TestClient(create_app(service)) as client:
        traversal = client.get("/assets/%2E%2E/secret.txt")
        nested = client.get("/assets/nested/app.js")
        dist_file = client.get("/index.html")
        missing_api = client.get(f"{API_PREFIX}/does-not-exist")

    assert traversal.status_code == 404
    assert nested.status_code == 404
    assert dist_file.status_code == 404
    assert missing_api.status_code == 404
    assert missing_api.json()["detail"] == "Not Found"


def test_http_rejects_reparse_asset_escape(tmp_path: Path) -> None:
    from fastapi.testclient import TestClient

    repo_root = tmp_path / "repo"
    service = _synthetic_ui_service(repo_root)
    external = repo_root / "external.js"
    external.write_text("external\n", encoding="utf-8")
    link = repo_root / "tools" / "simstudio" / "ui" / "dist" / "assets" / "escape.js"
    try:
        link.symlink_to(external)
    except (OSError, NotImplementedError) as exc:
        pytest.skip(f"symlink creation is unavailable: {exc}")

    with TestClient(create_app(service)) as client:
        response = client.get("/assets/escape.js")

    assert response.status_code == 404


def test_http_rejects_reparse_assets_directory_escape(tmp_path: Path) -> None:
    from fastapi.testclient import TestClient

    repo_root = tmp_path / "repo"
    service = _synthetic_ui_service(repo_root)
    assets = repo_root / "tools" / "simstudio" / "ui" / "dist" / "assets"
    external_assets = repo_root / "external-assets"
    external_assets.mkdir()
    (external_assets / "escape.js").write_text("external\n", encoding="utf-8")
    (assets / "app.js").unlink()
    assets.rmdir()
    try:
        assets.symlink_to(external_assets, target_is_directory=True)
    except (OSError, NotImplementedError) as exc:
        pytest.skip(f"directory symlink creation is unavailable: {exc}")

    with TestClient(create_app(service)) as client:
        response = client.get("/assets/escape.js")

    assert response.status_code == 404


def test_http_lists_session_drafts(client: Any) -> None:
    draft = _result(client.post(f"{API_PREFIX}/session-drafts", json={"intent": _intent()}))

    drafts = _result(client.get(f"{API_PREFIX}/session-drafts"))

    assert [item["id"] for item in drafts] == [draft["id"]]


def test_http_lists_compiled_bundles(client: Any) -> None:
    draft = _result(client.post(f"{API_PREFIX}/session-drafts", json={"intent": _intent()}))
    bundle = _result(client.post(f"{API_PREFIX}/session-drafts/{draft['id']}/compose"))

    bundles = _result(client.get(f"{API_PREFIX}/bundles"))

    assert [item["id"] for item in bundles] == [bundle["id"]]


def test_http_previews_a_run_artifact_by_opaque_id(tmp_path: Path) -> None:
    from fastapi.testclient import TestClient

    run_id = "a" * 32
    store = StudioStore(tmp_path / "studio", id_factory=iter([run_id]).__next__)
    run = store.create_run(
        {
            "schema": "lingtu.sim.studio.run-payload.v1",
            "bundle_id": "b" * 32,
            "launch_profile": "headless",
            "artifact_path": f"artifacts/runs/{run_id}",
        },
        status="STOPPED",
    )
    artifact_root = store.root / "artifacts" / "runs" / run.id
    artifact_root.mkdir(parents=True)
    artifact_root.joinpath("events.log").write_bytes(b"prepared\n")
    service = SimulationStudioService(store=store, artifact_service=ArtifactService(store))

    with TestClient(create_app(service)) as client:
        artifacts = _result(client.get(f"{API_PREFIX}/runs/{run.id}/artifacts"))
        preview = _result(
            client.get(
                f"{API_PREFIX}/runs/{run.id}/artifacts/"
                f"{artifacts[0]['artifact_id']}/preview"
            )
        )

    assert preview["artifact_id"] == artifacts[0]["artifact_id"]
    assert preview["run_id"] == run.id
    assert preview["path"] == "events.log"
    assert preview["content"] == "prepared\n"
    assert preview["previewable"] is True


def test_http_artifact_preview_rejects_caller_controlled_paths() -> None:
    from fastapi.testclient import TestClient

    class NoArtifactLookup:
        def list_artifacts(self, run_id: str) -> list[dict[str, Any]]:
            raise AssertionError(f"unsafe artifact lookup reached for {run_id}")

    with TestClient(create_app(NoArtifactLookup())) as client:
        error = _error(
            client.get(
                f"{API_PREFIX}/runs/{'a' * 32}/artifacts/"
                "C:%5CWindows%5Cwin.ini/preview"
            ),
            422,
        )

    assert error["code"] == "SIMSTUDIO_INVALID_REQUEST"
    assert "opaque" in error["message"]


def test_http_artifact_preview_rejects_reparse_components(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    from fastapi.testclient import TestClient

    run_id = "a" * 32
    store = StudioStore(tmp_path / "studio", id_factory=iter([run_id]).__next__)
    run = store.create_run(
        {
            "schema": "lingtu.sim.studio.run-payload.v1",
            "bundle_id": "b" * 32,
            "launch_profile": "headless",
            "artifact_path": f"artifacts/runs/{run_id}",
        },
        status="STOPPED",
    )
    nested = store.root / "artifacts" / "runs" / run.id / "nested"
    nested.mkdir(parents=True)
    nested.joinpath("report.json").write_bytes(b'{"ready":true}')
    service = SimulationStudioService(store=store, artifact_service=ArtifactService(store))

    with TestClient(create_app(service)) as client:
        artifacts = _result(client.get(f"{API_PREFIX}/runs/{run.id}/artifacts"))
        artifact_id = next(item["artifact_id"] for item in artifacts if item["path"] == "nested/report.json")
        original = StudioStore._is_reparse_point

        def fake_reparse(path: Path) -> bool:
            return Path(path) == nested or original(path)

        monkeypatch.setattr(StudioStore, "_is_reparse_point", staticmethod(fake_reparse))
        inventory_error = _error(
            client.get(f"{API_PREFIX}/runs/{run.id}/artifacts"),
            422,
        )
        preview_error = _error(
            client.get(
                f"{API_PREFIX}/runs/{run.id}/artifacts/{artifact_id}/preview"
            ),
            422,
        )

    assert inventory_error["code"] == "SIMSTUDIO_INVALID_REQUEST"
    assert preview_error["code"] == "SIMSTUDIO_INVALID_REQUEST"


def test_http_describes_read_only_management_capabilities(tmp_path: Path) -> None:
    from fastapi.testclient import TestClient

    store = StudioStore(tmp_path / "studio")
    service = SimulationStudioService(store=store, artifact_service=ArtifactService(store))

    with TestClient(create_app(service)) as client:
        capabilities = _result(client.get(f"{API_PREFIX}/capabilities"))

    assert capabilities == {
        "schema": "lingtu.sim.studio.capabilities.v1",
        "api_version": "v1",
        "api_prefix": API_PREFIX,
        "field_isolated": True,
        "read_models": {
            "imports": {"list": f"{API_PREFIX}/imports"},
            "session_drafts": {"list": f"{API_PREFIX}/session-drafts"},
            "bundles": {"list": f"{API_PREFIX}/bundles"},
            "run_artifacts": {
                "list": f"{API_PREFIX}/runs/{{run_id}}/artifacts",
                "preview": (
                    f"{API_PREFIX}/runs/{{run_id}}/artifacts/"
                    "{artifact_id}/preview"
                ),
            },
        },
        "artifact_preview": {
            "addressing": "opaque_artifact_id",
            "raw_path_input": False,
            "max_bytes": 1_048_576,
        },
        "schema_document": {
            "href": f"{API_PREFIX}/schema",
            "format": "openapi-3.1",
        },
    }


def test_http_exposes_the_live_openapi_schema_under_the_sim_prefix() -> None:
    from fastapi.testclient import TestClient

    with TestClient(create_app(object())) as client:
        schema = _result(client.get(f"{API_PREFIX}/schema"))

    assert schema["openapi"].startswith("3.1")
    assert f"{API_PREFIX}/imports" in schema["paths"]
    assert f"{API_PREFIX}/session-drafts" in schema["paths"]
    assert f"{API_PREFIX}/bundles" in schema["paths"]
    assert (
        f"{API_PREFIX}/runs/{{run_id}}/artifacts/"
        "{artifact_id}/preview"
    ) in schema["paths"]
    assert "RunCreateRequest" in schema["components"]["schemas"]
    assert all(path.startswith(f"{API_PREFIX}/") for path in schema["paths"])


def test_http_session_run_flow_with_readiness_and_artifacts(client: Any) -> None:
    health = _result(client.get(f"{API_PREFIX}/health"))
    assert health["status"] == "ok"
    assert health["field_isolated"] is True
    assert health["runtime_bound"] is True

    draft = _result(
        client.post(
            f"{API_PREFIX}/session-drafts",
            json={"intent": _intent()},
            headers={"Idempotency-Key": "draft-flow"},
        ),
        200,
    )
    bundle = _result(
        client.post(
            f"{API_PREFIX}/session-drafts/{draft['id']}/compose",
            json={"revision": draft["revision"]},
            headers={"Idempotency-Key": "compose-flow"},
        )
    )
    run = _result(
        client.post(
            f"{API_PREFIX}/runs",
            json={"bundle_id": bundle["id"], "launch_profile": "headless"},
            headers={"Idempotency-Key": "run-flow"},
        )
    )

    prepared = _result(
        client.post(
            f"{API_PREFIX}/runs/{run['id']}/prepare",
            json={"revision": run["revision"]},
            headers={"Idempotency-Key": "prepare-flow"},
        )
    )
    running = _result(client.post(f"{API_PREFIX}/runs/{run['id']}/start", json={"revision": prepared["revision"]}))
    paused = _result(client.post(f"{API_PREFIX}/runs/{run['id']}/pause", json={"revision": running["revision"]}))
    reset = _result(client.post(f"{API_PREFIX}/runs/{run['id']}/reset", json={"revision": paused["revision"]}))
    stopped = _result(client.post(f"{API_PREFIX}/runs/{run['id']}/stop", json={"revision": reset["revision"]}))

    assert stopped["status"] == "STOPPED"
    assert client.fake_sessions[0].calls == ["prepare", "start", "pause", "reset", "stop"]

    readiness = _result(client.get(f"{API_PREFIX}/runs/{run['id']}/readiness"))
    assert readiness["run_id"] == run["id"]
    assert readiness["status"] == "STOPPED"
    assert readiness["ready"] is False
    assert readiness["sensors"]["imu"] == "ready"

    artifacts = _result(client.get(f"{API_PREFIX}/runs/{run['id']}/artifacts"))
    assert [(item["path"], item["kind"]) for item in artifacts] == [("events.log", "file")]
    artifact = _result(client.get(f"{API_PREFIX}/artifacts/{artifacts[0]['artifact_id']}"))
    assert artifact["path"] == "events.log"


def test_http_errors_use_stable_envelopes_for_not_found_and_conflicts(client: Any) -> None:
    missing = _error(client.get(f"{API_PREFIX}/session-drafts/{'a' * 32}"), 404)
    assert missing["code"] == "SIMSTUDIO_NOT_FOUND"

    draft = _result(client.post(f"{API_PREFIX}/session-drafts", json={"intent": _intent()}))
    changed = copy.deepcopy(_intent())
    changed["session"]["seed"] = 9
    updated = _result(
        client.put(
            f"{API_PREFIX}/session-drafts/{draft['id']}",
            json={"revision": draft["revision"], "intent": changed},
        )
    )
    conflict = _error(
        client.post(
            f"{API_PREFIX}/session-drafts/{draft['id']}/compose",
            json={"revision": draft["revision"]},
        ),
        409,
    )
    assert conflict["code"] == "SIMSTUDIO_CONFLICT"
    assert conflict["details"] == {"expected": draft["revision"], "actual": updated["revision"]}

    first = _result(
        client.post(
            f"{API_PREFIX}/session-drafts",
            json={"intent": _intent()},
            headers={"Idempotency-Key": "same-key"},
        )
    )
    other = copy.deepcopy(_intent())
    other["session"]["session_id"] = "http_contract_other"
    idempotency = _error(
        client.post(
            f"{API_PREFIX}/session-drafts",
            json={"intent": other},
            headers={"Idempotency-Key": "same-key"},
        ),
        409,
    )
    assert first["id"] != ""
    assert idempotency["code"] == "SIMSTUDIO_CONFLICT"


def test_invalid_launch_profile_and_catalog_errors_are_422(client: Any) -> None:
    profile_error = _error(
        client.post(
            f"{API_PREFIX}/runs",
            json={"bundle_id": "bundle-does-not-matter", "launch_profile": "debug-shell"},
        ),
        422,
    )
    assert profile_error["code"] == "SIMSTUDIO_INVALID_REQUEST"

    invalid = copy.deepcopy(_intent())
    invalid["session"]["robots"][0]["package"] = "does_not_exist@1.0.0"
    catalog_error = _error(
        client.post(f"{API_PREFIX}/session-drafts", json={"intent": invalid}),
        422,
    )
    assert catalog_error["code"] != "SIMSTUDIO_INTERNAL_ERROR"
    assert "does_not_exist@1.0.0" in catalog_error["message"]


@pytest.mark.parametrize(
    "nested_control",
    [
        {"session": {"robots": [{"executable": "sim.exe"}]}},
        {"session": {"runtime": {"cwd": "outside"}}},
        {"session": {"runtime": {"ports": [5050]}}},
        {"session": {"runtime": {"dds_domain": 42}}},
        {"session": {"runtime": {"env": {"ROBOT_HOST": "field"}}}},
        {"session": {"world": {"source_path": "../outside"}}},
    ],
)
def test_http_rejects_nested_caller_controlled_process_path_ports_dds_and_env(
    client: Any,
    nested_control: dict[str, Any],
) -> None:
    invalid = copy.deepcopy(_intent())

    def merge(target: dict[str, Any], patch: dict[str, Any]) -> None:
        for key, value in patch.items():
            if isinstance(value, dict) and isinstance(target.get(key), dict):
                merge(target[key], value)
            else:
                target[key] = value

    merge(invalid, nested_control)
    error = _error(client.post(f"{API_PREFIX}/session-drafts", json={"intent": invalid}), 422)
    assert error["code"] == "SIMSTUDIO_INVALID_REQUEST"
    assert "caller-controlled Studio field" in error["message"]
