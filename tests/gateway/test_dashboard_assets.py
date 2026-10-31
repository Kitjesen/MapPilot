"""Static Web releases remain independent of native runtime installation."""

from pathlib import Path

import pytest
from fastapi import FastAPI
from fastapi.testclient import TestClient

from gateway.routes import assets


def _app() -> FastAPI:
    app = FastAPI()

    @app.get("/api/v1/status")
    async def status():
        return {"source": "api"}

    assets.register_asset_routes(app)
    return app


def _dist(path: Path, label: str) -> Path:
    path.mkdir(parents=True)
    (path / "index.html").write_text(f"<html>{label}</html>", encoding="utf-8")
    return path


def test_explicit_web_release_serves_html_and_preserves_api_mesh_and_asset_behavior(tmp_path, monkeypatch):
    standalone = _dist(tmp_path / "web-release", "independent Web")
    (standalone / "assets").mkdir()
    (standalone / "assets" / "app.js").write_text("console.log('app')", encoding="utf-8")
    (standalone / "api" / "v1").mkdir(parents=True)
    (standalone / "api" / "v1" / "status").write_text("static shadow", encoding="utf-8")
    mesh_dir = tmp_path / "meshes"
    mesh_dir.mkdir()
    (mesh_dir / "body.STL").write_bytes(b"solid body")
    monkeypatch.setenv("LINGTU_WEB_DIST", str(standalone))
    monkeypatch.setenv("DOG_MESH_DIR", str(mesh_dir))
    app = _app()
    assets.mount_dashboard_assets(app)

    with TestClient(app) as client:
        html = client.get("/")
        assert html.status_code == 200
        assert html.text == "<html>independent Web</html>"
        assert html.headers["cache-control"] == "no-cache, no-store, must-revalidate"
        javascript = client.get("/assets/app.js")
        assert javascript.status_code == 200
        assert "cache-control" not in javascript.headers
        assert "etag" in javascript.headers
        assert client.get("/api/v1/status").json() == {"source": "api"}
        mesh = client.get("/robot/meshes/body.STL")
        assert mesh.status_code == 200
        assert mesh.content == b"solid body"
        assert mesh.headers["cache-control"] == "public, max-age=3600"


def test_unconfigured_web_release_uses_the_checkout_build(tmp_path, monkeypatch):
    monkeypatch.delenv("LINGTU_WEB_DIST", raising=False)
    monkeypatch.setattr(assets, "__file__", str(tmp_path / "src/gateway/routes/assets.py"))
    _dist(tmp_path / "web" / "dist", "checkout Web")
    app = _app()
    assets.mount_dashboard_assets(app)

    with TestClient(app) as client:
        assert client.get("/").text == "<html>checkout Web</html>"
        assert client.get("/api/v1/status").json() == {"source": "api"}


def test_unconfigured_missing_web_build_keeps_api_only_mode(tmp_path, monkeypatch, caplog):
    monkeypatch.delenv("LINGTU_WEB_DIST", raising=False)
    monkeypatch.setattr(assets, "__file__", str(tmp_path / "src/gateway/routes/assets.py"))
    app = _app()
    assets.mount_dashboard_assets(app)

    with TestClient(app) as client:
        assert client.get("/").status_code == 404
        assert client.get("/api/v1/status").json() == {"source": "api"}
    assert not any(record.name == assets.__name__ for record in caplog.records)


@pytest.mark.parametrize("kind", ["blank", "missing", "file", "no_index"])
def test_invalid_explicit_web_release_logs_error_and_keeps_api_without_fallback(tmp_path, monkeypatch, caplog, kind):
    checkout = tmp_path / "checkout"
    monkeypatch.setattr(assets, "__file__", str(checkout / "src/gateway/routes/assets.py"))
    _dist(checkout / "web" / "dist", "old checkout Web")
    target = tmp_path / "invalid-dist"
    if kind == "file":
        target.write_text("not a directory", encoding="utf-8")
    elif kind == "no_index":
        target.mkdir()
    monkeypatch.setenv("LINGTU_WEB_DIST", "" if kind == "blank" else str(target))

    app = _app()
    assets.mount_dashboard_assets(app)
    with TestClient(app) as client:
        api = client.get("/api/v1/status")
        assert api.status_code == 200
        assert api.json() == {"source": "api"}
        assert client.get("/").status_code == 404
    assert any(record.name == assets.__name__ and record.levelname == "ERROR"
               and "LINGTU_WEB_DIST" in record.getMessage() for record in caplog.records)
