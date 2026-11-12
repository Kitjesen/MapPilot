import subprocess
import sys
from pathlib import Path

import pytest
from tools.validate.validate_docs import validate_repository

ROOT = Path(__file__).resolve().parents[2]


def test_first_party_documentation_contract() -> None:
    """Keep maintained Markdown within the documented structure."""

    violations, scanned = validate_repository(ROOT)

    assert scanned > 0
    assert violations == [], "\n".join(violations)


def test_documentation_ownership_keeps_first_party_checks(monkeypatch, tmp_path) -> None:
    from tools.validate import validate_docs

    excluded = tuple(tmp_path / prefix.relative_to(validate_docs.ROOT) for prefix in validate_docs.EXCLUDED_MARKDOWN_PREFIXES)
    monkeypatch.setattr(validate_docs, "ROOT", tmp_path)
    monkeypatch.setattr(validate_docs, "SCAN_ROOTS", (tmp_path, tmp_path / "docs", tmp_path / "sim"))
    monkeypatch.setattr(validate_docs, "EXCLUDED_MARKDOWN_PREFIXES", excluded)
    monkeypatch.setattr(validate_docs, "DOCS_MARKDOWN", {"README.md"})
    pages = {
        "docs/README.md": "# Docs\n**Status:** Current\n**Audience:** Developers\n**Runs on:** Local\n",
        "docs/assets/planning/README.md": "# Figures\n",
        "docs/assets/demo/README.md": "# Screenshots\n",
        "sim/packages/README.md": "# Owned models\n[Missing](missing.md)\n",
        "sim/.imports/external_robots/upstream/README.md": "Upstream title\n[Missing](missing.md)\n",
    }
    for relative, text in pages.items():
        path = tmp_path / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(text, encoding="utf-8")
    for relative in ("docs/assets/planning/figure.pdf", "docs/snapshot.pdf", "docs/assets/planning/snapshot.docx"):
        (tmp_path / relative).touch()

    violations, scanned = validate_docs.validate_repository(tmp_path)

    assert scanned == 4
    assert violations == [
        "binary snapshot belongs outside docs or in generated artifacts: docs/assets/planning/snapshot.docx",
        "binary snapshot belongs outside docs or in generated artifacts: docs/snapshot.pdf",
        "broken local link: sim/packages/README.md -> missing.md",
    ]


def test_generated_api_documentation_is_current() -> None:
    """Generated route and MCP inventories must match current source."""

    result = subprocess.run(
        [sys.executable, "tools/docs/extract_api_docs.py", "--check"],
        cwd=ROOT,
        capture_output=True,
        text=True,
        check=False,
    )

    assert result.returncode == 0, result.stdout + result.stderr


def test_gateway_inventory_preserves_canonical_recording_routes() -> None:
    """The generated inventory must expose the canonical recording API."""

    inventory = (ROOT / "docs" / "api.md").read_text(encoding="utf-8")
    for action in ("start", "status", "stop"):
        assert f"/api/v1/recordings/{action}" in inventory


@pytest.mark.parametrize(
    ("module", "register_name"),
    [
        ("gateway.navigation.routes", "register_navigation_routes"),
        ("gateway.navigation.diagnostics", "register_navigation_diagnostic_routes"),
        ("gateway.maps.routes", "register_map_routes"),
        ("gateway.maps.locations", "register_location_routes"),
        ("gateway.maps.places", "register_place_routes"),
        ("gateway.routes.status", "register_status_routes"),
        ("gateway.routes.health", "register_health_routes"),
        ("gateway.routes.diagnostics", "register_diagnostic_routes"),
        ("gateway.routes.realtime", "register_realtime_routes"),
    ],
)
def test_gateway_inventory_matches_route_owners(module, register_name) -> None:
    from importlib import import_module
    from types import SimpleNamespace

    from fastapi import FastAPI
    from tools.docs.extract_api_docs import extract_gateway_routes

    app = FastAPI()
    getattr(import_module(module), register_name)(app, SimpleNamespace())
    expected = {
        (method, route.path)
        for route in app.routes
        if route.endpoint.__module__ == module
        for method in getattr(route, "methods", ())
    }
    documented = [
        (route["method"], route["path"])
        for route in extract_gateway_routes()
        if route["file"] == "src/" + module.replace(".", "/") + ".py"
    ]
    assert expected
    assert set(documented) == expected
    assert len(documented) == len(expected)
