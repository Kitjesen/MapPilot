# ruff: noqa: D103,S101
"""Adversarial contract tests for the isolated SimStudio HTTP API.

These tests deliberately describe the G006 surface before all of its
implementation exists.  They are kept independent of the field Gateway and
only inspect the FastAPI contract; they must not require a running UE, MuJoCo,
DDS, or systemd process.
"""

from __future__ import annotations

import inspect
from collections.abc import Mapping
from typing import Any

import pytest

API_PREFIX = "/api/sim/v1"

# Package references are intentionally the only non-opaque identifiers in a
# URL.  Imports, drafts, bundles, runs, and artifacts are Studio-owned opaque
# IDs and are never filesystem paths or caller-selected process identifiers.
REQUIRED_ROUTES: frozenset[tuple[str, str]] = frozenset(
    {
        ("GET", f"{API_PREFIX}/health"),
        ("GET", f"{API_PREFIX}/packages"),
        ("GET", f"{API_PREFIX}/packages/{{kind}}/{{reference}}"),
        ("GET", f"{API_PREFIX}/inbox/sources"),
        ("GET", f"{API_PREFIX}/inbox/sources/{{source_id}}/inspection"),
        ("POST", f"{API_PREFIX}/imports"),
        ("GET", f"{API_PREFIX}/imports/{{import_id}}"),
        ("POST", f"{API_PREFIX}/imports/{{import_id}}/promote"),
        ("POST", f"{API_PREFIX}/session-drafts"),
        ("GET", f"{API_PREFIX}/session-drafts/{{draft_id}}"),
        ("PUT", f"{API_PREFIX}/session-drafts/{{draft_id}}"),
        ("POST", f"{API_PREFIX}/session-drafts/{{draft_id}}/compose"),
        ("GET", f"{API_PREFIX}/bundles/{{bundle_id}}"),
        ("GET", f"{API_PREFIX}/runs"),
        ("POST", f"{API_PREFIX}/runs"),
        ("GET", f"{API_PREFIX}/runs/{{run_id}}"),
        ("POST", f"{API_PREFIX}/runs/{{run_id}}/prepare"),
        ("POST", f"{API_PREFIX}/runs/{{run_id}}/start"),
        ("POST", f"{API_PREFIX}/runs/{{run_id}}/pause"),
        ("POST", f"{API_PREFIX}/runs/{{run_id}}/reset"),
        ("POST", f"{API_PREFIX}/runs/{{run_id}}/stop"),
        ("GET", f"{API_PREFIX}/runs/{{run_id}}/readiness"),
        ("GET", f"{API_PREFIX}/runs/{{run_id}}/artifacts"),
        ("GET", f"{API_PREFIX}/artifacts/{{artifact_id}}"),
    }
)

STATE_CHANGING_ROUTES = {
    route for route in REQUIRED_ROUTES if route[0] in {"POST", "PUT", "PATCH", "DELETE"}
}

# These names are forbidden in request parameters and request-body schemas.
# They are deployment/process concerns, or raw filesystem controls, and must
# be owned by the local Studio launcher and service implementation.
FORBIDDEN_INPUT_NAMES = frozenset(
    {
        "absolute_path",
        "bind_host",
        "command",
        "command_line",
        "cwd",
        "dds_domain",
        "domain",
        "env",
        "environment",
        "executable",
        "executable_path",
        "filesystem_path",
        "host",
        "output_dir",
        "port",
        "ports",
        "robot_host",
        "shm_name",
        "source_path",
        "working_dir",
    }
)


def _app() -> Any:
    fastapi = pytest.importorskip("fastapi")
    del fastapi
    from tools.simstudio.service.http import create_app

    # Route/OpenAPI contract tests must not construct a catalog or start a
    # process.  The adapter only needs a protocol-compatible placeholder until
    # the application service is assembled by the G006 implementation.
    return create_app(object())


def _route_table(app: Any) -> set[tuple[str, str]]:
    return {
        (method.upper(), route.path)
        for route in app.routes
        if hasattr(route, "methods")
        for method in route.methods
    }


def _openapi(app: Any) -> Mapping[str, Any]:
    """Return OpenAPI or fail with an actionable contract diagnostic."""

    try:
        document = app.openapi()
    except Exception as exc:  # pragma: no cover - exercised by incomplete adapters
        pytest.fail(f"SimStudio must expose a valid OpenAPI contract: {type(exc).__name__}: {exc}")
    assert isinstance(document, Mapping)
    return document


def _resolve_schema(app: Any, schema: Any) -> Any:
    if not isinstance(schema, Mapping):
        return schema
    reference = schema.get("$ref")
    if not isinstance(reference, str) or not reference.startswith("#/components/schemas/"):
        return schema
    name = reference.rsplit("/", 1)[-1]
    return app.openapi().get("components", {}).get("schemas", {}).get(name, schema)


def _input_names(app: Any) -> set[str]:
    """Collect names from route parameters and request schemas only."""

    names: set[str] = set()
    document = _openapi(app)
    for path, methods in document.get("paths", {}).items():
        del path
        for operation in methods.values():
            if not isinstance(operation, Mapping):
                continue
            for parameter in operation.get("parameters", []):
                if isinstance(parameter, Mapping) and isinstance(parameter.get("name"), str):
                    names.add(parameter["name"].lower().replace("-", "_"))
            body = operation.get("requestBody")
            if not isinstance(body, Mapping):
                continue
            content = body.get("content", {})
            json_schema = content.get("application/json", {}).get("schema")
            stack = [_resolve_schema(app, json_schema)]
            while stack:
                current = stack.pop()
                if not isinstance(current, Mapping):
                    continue
                properties = current.get("properties", {})
                if isinstance(properties, Mapping):
                    names.update(str(name).lower().replace("-", "_") for name in properties)
                    stack.extend(properties.values())
                items = current.get("items")
                if items is not None:
                    stack.append(items)
                for branch in current.get("oneOf", []) + current.get("anyOf", []):
                    stack.append(_resolve_schema(app, branch))
    return names


def _operation(app: Any, method: str, path: str) -> Mapping[str, Any]:
    operation = _openapi(app)["paths"].get(path, {}).get(method.lower())
    assert operation is not None, f"missing OpenAPI operation {method} {path}"
    return operation


def test_api_is_versioned_and_exposes_the_complete_studio_surface() -> None:
    app = _app()
    routes = _route_table(app)

    missing = sorted(REQUIRED_ROUTES - routes)
    assert not missing, f"G006 SimStudio endpoint(s) missing: {missing}"

    # A simulation-local application must not silently become the field HMI.
    forbidden = sorted(
        (method, path)
        for method, path in routes
        if path.startswith("/api/v1")
        or "/5050" in path
        or "/8090" in path
        or path.startswith("/api/sim/catalog")
        or path.startswith("/api/sim/sessions")
    )
    assert not forbidden, f"field or unversioned routes leaked into SimStudio: {forbidden}"

    unversioned_sim_routes = sorted(
        (method, path)
        for method, path in routes
        if path.startswith("/api/sim/") and not path.startswith(f"{API_PREFIX}/")
    )
    assert not unversioned_sim_routes, f"unversioned SimStudio routes remain: {unversioned_sim_routes}"


def test_state_changes_declare_idempotency_and_revision_contracts() -> None:
    app = _app()

    for method, path in sorted(STATE_CHANGING_ROUTES):
        operation = _operation(app, method, path)
        headers = {
            str(parameter.get("name", "")).lower()
            for parameter in operation.get("parameters", [])
            if isinstance(parameter, Mapping) and parameter.get("in") == "header"
        }
        assert "idempotency-key" in headers, f"{method} {path} must accept Idempotency-Key"

    draft_update = _operation(app, "PUT", f"{API_PREFIX}/session-drafts/{{draft_id}}")
    body = draft_update.get("requestBody", {})
    schema = body.get("content", {}).get("application/json", {}).get("schema", {})
    schema = _resolve_schema(app, schema)
    required = set(schema.get("required", [])) if isinstance(schema, Mapping) else set()
    assert "revision" in required, "session-draft updates must use an explicit optimistic revision"


def test_dynamic_ids_are_opaque_and_one_active_run_is_the_only_run_policy() -> None:
    app = _app()
    document = _openapi(app)

    for path in document.get("paths", {}):
        for segment in path.split("/"):
            if not (segment.startswith("{") and segment.endswith("}")):
                continue
            parameter = segment[1:-1]
            if (
                "/packages/" in path or "/import-contracts/" in path
            ) and parameter in {"kind", "reference"}:
                continue
            if parameter.endswith("_index"):
                continue
            assert parameter.endswith("_id"), f"non-package route parameter is not opaque: {path}"

    run_create = _operation(app, "POST", f"{API_PREFIX}/runs")
    body = run_create.get("requestBody", {})
    schema = _resolve_schema(
        app,
        body.get("content", {}).get("application/json", {}).get("schema", {}),
    )
    properties = set(schema.get("properties", {})) if isinstance(schema, Mapping) else set()
    assert not properties & {"parallel", "concurrent", "allow_multiple", "max_active_runs"}, (
        "Studio must enforce one active prepared/running session; callers cannot turn on concurrency"
    )

    assert f"{API_PREFIX}/runs" in document["paths"]
    assert "get" in document["paths"][f"{API_PREFIX}/runs"]


def test_http_contract_rejects_raw_process_and_filesystem_controls() -> None:
    app = _app()
    leaked = sorted(FORBIDDEN_INPUT_NAMES & _input_names(app))
    assert not leaked, f"forbidden caller-controlled HTTP inputs leaked into SimStudio: {leaked}"

    # The import/session contracts may identify inbox-relative source records,
    # but must never accept a caller-selected absolute path or output root.
    import_operation = _operation(app, "POST", f"{API_PREFIX}/imports")
    import_schema = _resolve_schema(
        app,
        import_operation.get("requestBody", {})
        .get("content", {})
        .get("application/json", {})
        .get("schema", {}),
    )
    import_properties = set(import_schema.get("properties", {})) if isinstance(import_schema, Mapping) else set()
    assert "source" in import_properties, "imports must use a typed source descriptor"
    assert "source_path" not in import_properties

    for method, path in sorted(STATE_CHANGING_ROUTES):
        operation = _operation(app, method, path)
        parameters = operation.get("parameters", [])
        assert not any(
            isinstance(parameter, Mapping)
            and parameter.get("in") == "query"
            and str(parameter.get("name", "")).lower() in FORBIDDEN_INPUT_NAMES
            for parameter in parameters
        ), f"{method} {path} exposes a forbidden process/filesystem query control"


def test_service_and_launcher_are_simulation_local_and_loopback_only() -> None:
    from tools.simstudio.service.application import SimulationStudioService
    from tools.simstudio.service.http import create_app

    service_parameters = set(inspect.signature(SimulationStudioService).parameters)
    app_parameters = set(inspect.signature(create_app).parameters)
    forbidden = service_parameters | app_parameters
    assert not forbidden & {"host", "port", "domain", "executable", "env", "environment"}

    # Binding belongs to the local launcher, and its public default must be
    # loopback.  This assertion intentionally requires the launcher contract;
    # merely keeping a FastAPI app factory is not sufficient for a local tool.
    from tools.simstudio import __main__ as launcher

    bind_host = getattr(launcher, "DEFAULT_HOST", None)
    assert bind_host == "127.0.0.1", "SimStudio launcher must default to loopback"
    assert create_app is not None
