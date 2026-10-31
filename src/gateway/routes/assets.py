"""Static asset routes for the Gateway dashboard."""

from __future__ import annotations

import logging
import os
from pathlib import Path

from fastapi.responses import FileResponse, JSONResponse

logger = logging.getLogger(__name__)


def robot_mesh_path(filename: str) -> Path | None:
    """Resolve one robot mesh from the configured or bundled Thunder V4 assets."""

    safe_name = os.path.basename(filename)
    configured = os.environ.get("DOG_MESH_DIR")
    repo_root = Path(__file__).resolve().parents[3]
    candidates = [
        Path(configured).expanduser() if configured else None,
        repo_root / "sim" / "packages" / "robots" / "doso" / "thunder_v4" / "meshes",
    ]
    for mesh_dir in candidates:
        if mesh_dir is not None and (mesh_dir / safe_name).is_file():
            return mesh_dir / safe_name
    return None


def register_asset_routes(app) -> None:
    """Register robot visual asset routes before mounting the dashboard."""

    @app.get("/robot/meshes/{filename}", summary="Serve robot STL mesh files")
    async def serve_robot_mesh(filename: str):
        safe_name = os.path.basename(filename)
        path = robot_mesh_path(safe_name)
        if path is None:
            return JSONResponse(
                status_code=404,
                content={"error": "mesh not found", "name": safe_name},
            )
        return FileResponse(
            str(path),
            media_type="application/octet-stream",
            headers={
                "Access-Control-Allow-Origin": "*",
                "Cache-Control": "public, max-age=3600",
            },
        )


def mount_dashboard_assets(app) -> None:
    """Serve an explicit Web release, or the local build when available."""
    configured = os.environ.get("LINGTU_WEB_DIST")
    if configured is not None and not configured.strip():
        logger.error("LINGTU_WEB_DIST must name a Web distribution directory; dashboard not mounted")
        return
    web_dist = (
        Path(configured).expanduser()
        if configured is not None
        else Path(__file__).resolve().parents[3] / "web" / "dist"
    )
    if not web_dist.is_dir():
        if configured is not None:
            logger.error("LINGTU_WEB_DIST is not a directory: %s; dashboard not mounted", web_dist)
        return
    if configured is not None and not (web_dist / "index.html").is_file():
        logger.error("LINGTU_WEB_DIST is missing index.html: %s; dashboard not mounted", web_dist)
        return

    from starlette.staticfiles import StaticFiles
    from starlette.types import Receive, Scope, Send

    inner_app = StaticFiles(directory=web_dist, html=True)

    async def no_cache_html(scope: Scope, receive: Receive, send: Send) -> None:
        async def send_with_headers(message: dict) -> None:
            if message.get("type") == "http.response.start":
                path = scope.get("path", "")
                if not path.startswith("/assets/"):
                    raw = list(message.get("headers", []))
                    raw.append((b"cache-control", b"no-cache, no-store, must-revalidate"))
                    message = {**message, "headers": raw}
            await send(message)

        await inner_app(scope, receive, send_with_headers)

    app.mount("/", no_cache_html, name="dashboard")
    logger.info("Dashboard served from %s", web_dist)
