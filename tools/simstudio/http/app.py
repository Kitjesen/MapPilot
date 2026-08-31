"""Read-model extensions for the isolated SimStudio HTTP application."""

from __future__ import annotations

import mimetypes
import os
import re
from pathlib import Path
from typing import Any

from tools.simstudio.service.artifact_service import (
    ArtifactNotFound,
    ArtifactSecurityError,
)
from tools.simstudio.service.http import API_PREFIX
from tools.simstudio.service.http import create_app as create_service_app

_FILE_ATTRIBUTE_REPARSE_POINT = 0x400
_SAFE_ASSET_NAME = re.compile(r"[A-Za-z0-9._-]+")


def _is_reparse_point(path: Path) -> bool:
    """Return whether a path is a symlink or Windows reparse point."""

    try:
        stat_result = os.lstat(path)
    except OSError:
        return False
    return path.is_symlink() or bool(
        getattr(stat_result, "st_file_attributes", 0) & _FILE_ATTRIBUTE_REPARSE_POINT
    )


def _contains_reparse_component(path: Path, *, root: Path) -> bool:
    """Reject reparse points anywhere between ``root`` and ``path``."""

    try:
        path.relative_to(root)
    except ValueError:
        return True

    current = path
    while True:
        if _is_reparse_point(current):
            return True
        if current == root:
            return False
        parent = current.parent
        if parent == current:
            return True
        current = parent


def create_app(service: Any) -> Any:
    """Create SimStudio's versioned app with its read-only management views."""

    from fastapi.responses import FileResponse, JSONResponse, Response

    app = create_service_app(service)

    def failure(code: str, message: str, *, status_code: int):
        error = {"code": code, "message": message}
        return JSONResponse(
            {"ok": False, "error": error, "diagnostics": [error]},
            status_code=status_code,
        )

    package_service = getattr(service, "package_service", None)
    repo_root = getattr(service, "repo_root", None)
    if repo_root is None:
        repo_root = getattr(package_service, "repo_root", None)
    trusted_root = Path(repo_root).resolve() if repo_root is not None else None
    ui_dist = (
        trusted_root / "tools" / "simstudio" / "ui" / "dist"
        if trusted_root is not None
        else None
    )
    index_path = ui_dist / "index.html" if ui_dist is not None else None

    @app.get("/", include_in_schema=False)
    def serve_ui_index():
        if (
            trusted_root is not None
            and index_path is not None
            and index_path.is_file()
            and not _contains_reparse_component(index_path, root=trusted_root)
        ):
            return FileResponse(index_path)
        return failure(
            "SIMSTUDIO_UI_NOT_BUILT",
            "SimStudio UI is not built; build tools/simstudio/ui so dist/index.html is available",
            status_code=503,
        )

    assets_path = ui_dist / "assets" if ui_dist is not None else None

    @app.get("/assets/{asset_name}", include_in_schema=False)
    def serve_ui_asset(asset_name: str):
        if (
            trusted_root is None
            or assets_path is None
            or not assets_path.is_dir()
            or "/" in asset_name
            or "\\" in asset_name
            or asset_name in {".", ".."}
            or _SAFE_ASSET_NAME.fullmatch(asset_name) is None
            or Path(asset_name).name != asset_name
        ):
            return Response(status_code=404)

        asset_path = assets_path / asset_name
        if (
            not _contains_reparse_component(assets_path, root=trusted_root)
            and asset_path.is_file()
            and not _contains_reparse_component(asset_path, root=trusted_root)
        ):
            media_type = mimetypes.guess_type(asset_path.name)[0]
            return FileResponse(asset_path, media_type=media_type)
        return Response(status_code=404)

    @app.get(f"{API_PREFIX}/imports")
    def list_imports():
        package_service = getattr(service, "package_service", None)
        if package_service is None:
            raise RuntimeError("package service is not configured")
        return JSONResponse({"ok": True, "result": package_service.list_import_jobs()})

    @app.get(f"{API_PREFIX}/session-drafts")
    def list_session_drafts():
        session_service = getattr(service, "session_service", None)
        if session_service is None:
            raise RuntimeError("session service is not configured")
        return JSONResponse({"ok": True, "result": session_service.list_session_drafts()})

    @app.get(f"{API_PREFIX}/bundles")
    def list_bundles():
        session_service = getattr(service, "session_service", None)
        if session_service is None:
            raise RuntimeError("session service is not configured")
        return JSONResponse({"ok": True, "result": session_service.list_bundles()})

    @app.get(f"{API_PREFIX}/capabilities")
    def capabilities():
        artifact_service = getattr(service, "artifact_service", None)
        max_preview_bytes = getattr(artifact_service, "max_preview_bytes", 1_048_576)
        read_models = {
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
        }
        if getattr(service, "source_inbox_service", None) is not None:
            read_models["source_inbox"] = {
                "list": f"{API_PREFIX}/inbox/sources",
                "upload": f"{API_PREFIX}/inbox/uploads",
                "inspect": f"{API_PREFIX}/inbox/sources/{{source_id}}/inspection",
            }
        if getattr(service, "package_service", None) is not None:
            read_models["import_contracts"] = {
                "get": f"{API_PREFIX}/import-contracts/{{kind}}",
            }
        if getattr(service, "recording_service", None) is not None:
            read_models["recordings"] = {
                "list": f"{API_PREFIX}/recordings",
                "inspect": f"{API_PREFIX}/runs/{{run_id}}/recording",
                "timeline": f"{API_PREFIX}/runs/{{run_id}}/recording/timeline",
                "frame": (
                    f"{API_PREFIX}/runs/{{run_id}}/recording/frames/"
                    "{frame_index}"
                ),
                "start": f"{API_PREFIX}/runs/{{run_id}}/recording/start",
                "stop": f"{API_PREFIX}/runs/{{run_id}}/recording/stop",
            }
        if getattr(service, "scene_draft_service", None) is not None:
            read_models["scene_drafts"] = {
                "list": f"{API_PREFIX}/scene-drafts",
                "get": f"{API_PREFIX}/scene-drafts/{{scene_draft_id}}",
                "create": f"{API_PREFIX}/scene-drafts",
                "update": f"{API_PREFIX}/scene-drafts/{{scene_draft_id}}",
            }
            if getattr(service, "scene_publication_service", None) is not None:
                read_models["scene_drafts"]["publish"] = (
                    f"{API_PREFIX}/scene-drafts/{{scene_draft_id}}/publish"
                )
        return JSONResponse(
            {
                "ok": True,
                "result": {
                    "schema": "lingtu.sim.studio.capabilities.v1",
                    "api_version": "v1",
                    "api_prefix": API_PREFIX,
                    "field_isolated": True,
                    "read_models": read_models,
                    "artifact_preview": {
                        "addressing": "opaque_artifact_id",
                        "raw_path_input": False,
                        "max_bytes": max_preview_bytes,
                    },
                    "schema_document": {
                        "href": f"{API_PREFIX}/schema",
                        "format": "openapi-3.1",
                    },
                },
            }
        )

    @app.get(f"{API_PREFIX}/runs/{{run_id}}/artifacts/{{artifact_id}}/preview")
    def preview_artifact(run_id: str, artifact_id: str, max_bytes: int | None = None):
        if re.fullmatch(r"[0-9a-f]{32}", artifact_id) is None:
            return failure(
                "SIMSTUDIO_INVALID_REQUEST",
                "artifact id must be an opaque lowercase hexadecimal identifier",
                status_code=422,
            )
        try:
            artifact = next(
                (item for item in service.list_artifacts(run_id) if item.get("artifact_id") == artifact_id),
                None,
            )
            if artifact is None:
                return failure(
                    "SIMSTUDIO_NOT_FOUND",
                    f"artifact {artifact_id} was not found",
                    status_code=404,
                )
            artifact_service = getattr(service, "artifact_service", None)
            if artifact_service is None:
                raise RuntimeError("artifact service is not configured")
            preview = artifact_service.preview(run_id, artifact["path"], max_bytes=max_bytes)
        except ArtifactNotFound as exc:
            return failure("SIMSTUDIO_NOT_FOUND", str(exc), status_code=404)
        except (ArtifactSecurityError, ValueError) as exc:
            return failure("SIMSTUDIO_INVALID_REQUEST", str(exc), status_code=422)
        return JSONResponse(
            {
                "ok": True,
                "result": {
                    **preview,
                    "artifact_id": artifact_id,
                    "run_id": run_id,
                },
            }
        )

    @app.get(f"{API_PREFIX}/schema")
    def schema_document():
        return JSONResponse({"ok": True, "result": app.openapi()})

    return app


__all__ = ["API_PREFIX", "create_app"]
