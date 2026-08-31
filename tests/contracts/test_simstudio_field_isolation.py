# ruff: noqa: D103,S101
"""Field-boundary and import-graph contracts for SimStudio."""

from __future__ import annotations

import json
import os
import subprocess
import sys
from pathlib import Path
from typing import Any

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
FORBIDDEN_MODULE_PREFIXES = (
    "lingtu.control",
    "lingtu.real.systemd",
    "src.gateway",
    "gateway",
)
FORBIDDEN_ENVIRONMENT = (
    "ROBOT_HOST",
    "LINGTU_PRODUCT",
    "LINGTU_ENV",
    "DDS_DOMAIN",
    "CYCLONEDDS_URI",
)


def _subprocess_import_report() -> dict[str, Any]:
    script = r'''
import json
import os
import sys

from tools.simstudio.service import application, http

print(json.dumps({
    "modules": sorted(sys.modules),
    "application": application.__name__,
    "http": http.__name__,
    "environment": {key: os.environ.get(key) for key in (
        "ROBOT_HOST", "LINGTU_PRODUCT", "LINGTU_ENV", "DDS_DOMAIN", "CYCLONEDDS_URI"
    )},
}))
'''
    environment = os.environ.copy()
    environment.update(
        {
            "ROBOT_HOST": "field.example.invalid",
            "LINGTU_PRODUCT": "nav",
            "LINGTU_ENV": "real",
            "DDS_DOMAIN": "999",
            "CYCLONEDDS_URI": "file:///field/cyclonedds.xml",
            "PYTHONPATH": os.pathsep.join((str(REPO_ROOT), str(REPO_ROOT / "src"))),
        }
    )
    completed = subprocess.run(  # noqa: S603
        [sys.executable, "-c", script],
        cwd=REPO_ROOT,
        env=environment,
        capture_output=True,
        text=True,
        check=True,
    )
    return json.loads(completed.stdout)


def _app() -> Any:
    pytest.importorskip("fastapi")
    from tools.simstudio.service.http import create_app

    return create_app(object())


def test_importing_simstudio_does_not_import_field_owners() -> None:
    report = _subprocess_import_report()
    imported = tuple(report["modules"])
    leaked = sorted(
        module
        for module in imported
        if module == "lingtu.control"
        or module == "lingtu.real.systemd"
        or module == "src.gateway"
        or any(module.startswith(prefix + ".") for prefix in FORBIDDEN_MODULE_PREFIXES)
    )
    assert not leaked, f"SimStudio import graph leaked field ownership modules: {leaked}"
    assert report["environment"]["ROBOT_HOST"] == "field.example.invalid"


def test_simstudio_does_not_expose_field_routes_or_field_transport_ports() -> None:
    app = _app()
    routes = {
        (method.upper(), route.path)
        for route in app.routes
        if hasattr(route, "methods")
        for method in route.methods
    }
    leaked = sorted(
        (method, path)
        for method, path in routes
        if path.startswith("/api/v1")
        or path.startswith("/api/sim/catalog")
        or path.startswith("/api/sim/sessions")
        or ":5050" in path
        or ":8090" in path
        or "/5050" in path
        or "/8090" in path
    )
    assert not leaked, f"field Gateway routes/ports leaked into SimStudio: {leaked}"


def test_simstudio_app_creation_has_no_field_process_or_network_side_effect(monkeypatch: pytest.MonkeyPatch) -> None:
    app = _app()
    del app

    # Importing/constructing the local app is a pure adapter operation.  Any
    # field ownership or process launch belongs outside this factory.
    import socket
    import subprocess

    monkeypatch.setattr(subprocess, "Popen", lambda *args, **kwargs: (_ for _ in ()).throw(AssertionError(args)))
    monkeypatch.setattr(socket, "create_connection", lambda *args, **kwargs: (_ for _ in ()).throw(AssertionError(args)))
    _app()


def test_simstudio_configuration_cannot_be_selected_by_field_environment() -> None:
    report = _subprocess_import_report()
    assert set(report["environment"]) == set(FORBIDDEN_ENVIRONMENT)

    # The environment is deliberately visible to the child, but the imported
    # modules must not turn those field controls into Studio API configuration.
    pytest.importorskip("fastapi")
    from tools.simstudio.service.http import create_app

    app = create_app(object())
    assert not any(
        field in {"robot_host", "host", "port", "domain", "dds_domain", "environment", "env"}
        for route in app.routes
        if hasattr(route, "endpoint")
        for field in getattr(route.endpoint, "__annotations__", {})
    )
