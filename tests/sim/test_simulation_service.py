# ruff: noqa: S101
"""S3 contract tests for the simulation-local Studio application service."""

from __future__ import annotations

import json
import subprocess
from io import StringIO
from pathlib import Path
from typing import Any

import pytest


def _service(root: Path, *, artifact_root: Path | None = None) -> Any:
    from tools.simstudio.service.application import SimulationStudioService

    if hasattr(SimulationStudioService, "from_repository"):
        return SimulationStudioService.from_repository(root, artifact_root=artifact_root)
    return SimulationStudioService(root)


def _write_world(root: Path) -> None:
    package = root / "sim" / "packages" / "worlds" / "field"
    package.mkdir(parents=True)
    (package / "world.xml").write_text(
        '<mujoco><option timestep="0.002" integrator="RK4" solver="Newton" iterations="100" '
        'gravity="0 0 -9.81"/><worldbody/></mujoco>\n',
        encoding="utf-8",
    )
    (package / "world.package.yaml").write_text(
        """schema: lingtu.sim.world-package.v1
id: field
version: 1.0.0
kind: world
physics:
  mjcf: world.xml
  global_policy:
    timestep_s: 0.002
    integrator: rk4
    solver: newton
    iterations: 100
    gravity_mps2: [0.0, 0.0, -9.81]
visual:
  binding: WorldVisual:Field
  level: /Game/RobotSim/Maps/Field
entities: []
""",
        encoding="utf-8",
    )


def _write_scenario(root: Path) -> None:
    package = root / "sim" / "packages" / "scenarios" / "patrol"
    package.mkdir(parents=True)
    (package / "scenario.package.yaml").write_text(
        """schema: lingtu.sim.scenario-package.v1
id: patrol
version: 1.0.0
kind: scenario
world: field@1.0.0
entities:
  - entity_id: pedestrian_01
    entity_type: pedestrian
    authority: scenario
    initial_transform:
      position_m: [0.0, 0.0, 0.0]
      quaternion_wxyz: [1.0, 0.0, 0.0, 0.0]
    physics_proxy: kinematic
    semantic_class: person
""",
        encoding="utf-8",
    )


def test_service_list_and_inspect_are_core_queries_that_do_not_start_processes(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    def fail_popen(*args: Any, **kwargs: Any) -> None:
        raise AssertionError(f"service query unexpectedly started a process: {args!r} {kwargs!r}")

    monkeypatch.setattr(subprocess, "Popen", fail_popen)
    _write_world(tmp_path)
    service = _service(tmp_path)

    listed = service.list_packages(kind="world")
    inspected = service.inspect_package("field@1.0.0", kind="world")

    assert listed["packages"][0]["package"]["id"] == inspected["package"]["id"]
    assert set(listed["packages"][0]["manifest"]) == {"path"}
    assert "fingerprint" not in inspected


def test_service_and_catalog_cli_return_the_same_core_catalog_result(tmp_path: Path) -> None:
    _write_world(tmp_path)
    service = _service(tmp_path)

    from sim.catalog import __main__ as cli

    stdout = StringIO()
    assert cli.main(["list", "--kind", "world", "--repo-root", str(tmp_path)], stdout=stdout) == 0
    cli_result = json.loads(stdout.getvalue())["result"]

    service_result = service.list_packages(kind="world")

    assert cli_result == service_result


def test_catalog_cli_requires_explicit_resolve_command() -> None:
    from sim.catalog import __main__ as cli

    with pytest.raises(SystemExit) as exc_info:
        cli.main(["sim/sessions/example.yaml"])

    assert exc_info.value.code == 2


def test_catalog_cli_explicit_resolve_keeps_json_contract() -> None:
    from sim.catalog import __main__ as cli

    stdout = StringIO()
    assert (
        cli.main(
            [
                "resolve",
                "sim/sessions/examples/thunder_omni_contract/session.yaml",
                "--repo-root",
                ".",
            ],
            stdout=stdout,
        )
        == 0
    )
    assert json.loads(stdout.getvalue()) == {
        "ok": True,
        "result": {
            "bundle_dir": None,
            "schema": "lingtu.sim.resolved-session-result.v1",
            "session_id": "thunder_omni_contract",
        },
    }


def test_service_can_inspect_scenario_packages_even_though_import_is_robot_world_only(
    tmp_path: Path,
) -> None:
    _write_world(tmp_path)
    _write_scenario(tmp_path)
    service = _service(tmp_path)

    inspected = service.inspect_package("patrol@1.0.0", kind="scenario")

    assert inspected["package"] == {
        "kind": "scenario",
        "id": "patrol",
        "version": "1.0.0",
        "ref": "patrol@1.0.0",
    }
    assert inspected["dependencies"]["edges"][0]["target"]["ref"] == "field@1.0.0"


@pytest.mark.parametrize(
    ("command", "reference", "kind", "expected_schema"),
    [
        ("dependencies", "patrol@1.0.0", "scenario", "lingtu.sim.package-dependencies.v1"),
        ("qualification", "field@1.0.0", "world", "lingtu.sim.qualification-view.v1"),
    ],
)
def test_catalog_cli_supports_read_only_management_queries(
    tmp_path: Path,
    command: str,
    reference: str,
    kind: str,
    expected_schema: str,
) -> None:
    _write_world(tmp_path)
    _write_scenario(tmp_path)
    from sim.catalog import __main__ as cli

    stdout = StringIO()
    assert (
        cli.main(
            [command, reference, "--kind", kind, "--repo-root", str(tmp_path)],
            stdout=stdout,
        )
        == 0
    )
    cli_envelope = json.loads(stdout.getvalue())
    assert cli_envelope["ok"] is True
    assert cli_envelope["result"]["schema"] == expected_schema


def test_http_adapter_exposes_v1_session_drafts_and_returns_structured_malformed_request(tmp_path: Path) -> None:
    pytest.importorskip("fastapi")
    pytest.importorskip("httpx")
    _write_world(tmp_path)
    service = _service(tmp_path, artifact_root=tmp_path / "owned")
    from fastapi.testclient import TestClient
    from tools.simstudio.service.http import API_PREFIX, create_app

    app = create_app(service)
    routes = {route.path: route for route in app.routes if hasattr(route, "endpoint")}
    assert f"{API_PREFIX}/session-drafts" in routes
    assert "/api/sim/sessions/resolve" not in routes

    with TestClient(app) as client:
        response = client.post(f"{API_PREFIX}/session-drafts", json={})
        payload = response.json()
        assert response.status_code == 422
        assert payload["ok"] is False
        assert payload["error"] == payload["diagnostics"][0]
        assert payload["diagnostics"][0]["code"] == "SIMSTUDIO_INVALID_REQUEST"

        typed_response = client.post(f"{API_PREFIX}/session-drafts", json={"intent": []})
        typed_payload = typed_response.json()
        assert typed_response.status_code == 422
        assert typed_payload["ok"] is False
        assert typed_payload["error"] == typed_payload["diagnostics"][0]
        assert typed_payload["diagnostics"][0]["code"] == "SIMSTUDIO_INVALID_REQUEST"

        guarded_response = client.post(
            f"{API_PREFIX}/session-drafts",
            json={"intent": {"session": {"output_dir": "bundle"}}},
        )
        guarded_payload = guarded_response.json()
        assert guarded_response.status_code == 422
        assert guarded_payload["ok"] is False
        assert guarded_payload["error"] == guarded_payload["diagnostics"][0]
        assert guarded_payload["diagnostics"][0]["code"] == "SIMSTUDIO_INVALID_REQUEST"
