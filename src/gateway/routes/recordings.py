"""HTTP file-management adapter for native recording sessions."""

from __future__ import annotations

import asyncio
from pathlib import PurePosixPath
from typing import Any

from gateway.services.recording import NativeRecordingError
from gateway.services.recording_files import resolve_declared_artifact


def _error(exc: NativeRecordingError):
    from fastapi.responses import JSONResponse

    return JSONResponse(
        status_code=exc.status_code,
        content={"error": exc.code, "detail": None},
    )


def _public_manifest(session: dict[str, Any]) -> dict[str, Any]:
    """Hide host paths and process argv while keeping reviewable metadata."""
    public: dict[str, Any] = {
        key: session.get(key)
        for key in (
            "version",
            "session_id",
            "state",
            "created_at_unix_ns",
            "started_at_unix_ns",
            "ended_at_unix_ns",
            "context",
            "error",
        )
        if key in session
    }
    children: list[dict[str, Any]] = []
    for raw_child in session.get("children", []):
        if not isinstance(raw_child, dict):
            continue
        artifacts = [
            str(item)
            for item in raw_child.get("artifacts", [])
            if isinstance(item, str) and item
        ]
        children.append(
            {
                "name": raw_child.get("name"),
                "required": raw_child.get("required"),
                "artifacts": artifacts,
                "selected_topics": list(raw_child.get("selected_topics", []))
                if isinstance(raw_child.get("selected_topics"), list)
                else [],
            }
        )
    public["children"] = children
    public["artifacts"] = [
        {"path": artifact, "download": f"files/{artifact}"}
        for child in children
        for artifact in child["artifacts"]
    ]
    return public


def register_recording_routes(app, gw) -> None:
    """Register list/detail/download/delete without duplicating native state."""

    @app.get("/api/v1/recordings")
    async def recording_list(limit: int = 100):
        try:
            result = await asyncio.to_thread(gw._recording.list, limit=limit)
        except NativeRecordingError as exc:
            return _error(exc)
        return {
            "ok": True,
            "sessions": [
                {
                    key: item.get(key)
                    for key in ("session_id", "state")
                    if key in item
                }
                for item in result["sessions"]
            ],
            "truncated": result["truncated"],
            "disk_free": result["disk_free"],
            "disk_total": result["disk_total"],
        }

    @app.get("/api/v1/recordings/{session_id}")
    async def recording_detail(session_id: str):
        try:
            session = await asyncio.to_thread(gw._recording.manifest, session_id=session_id)
        except NativeRecordingError as exc:
            return _error(exc)
        return {"ok": True, "session": _public_manifest(session)}

    @app.get("/api/v1/recordings/{session_id}/files/{artifact_path:path}")
    async def recording_artifact(session_id: str, artifact_path: str):
        from fastapi.responses import FileResponse

        try:
            path, metadata = await asyncio.to_thread(
                resolve_declared_artifact,
                gw._recording,
                session_id=session_id,
                artifact_name=artifact_path,
            )
        except NativeRecordingError as exc:
            return _error(exc)
        suffix = PurePosixPath(metadata["path"]).suffix.lower()
        media_type = {
            ".mcap": "application/octet-stream",
            ".mkv": "video/x-matroska",
        }.get(suffix, "application/octet-stream")
        return FileResponse(path, media_type=media_type, filename=path.name)

    @app.delete("/api/v1/recordings/{session_id}")
    async def recording_remove(session_id: str):
        try:
            result = await asyncio.to_thread(gw._recording.remove, session_id=session_id)
        except NativeRecordingError as exc:
            return _error(exc)
        return {"ok": True, "session_id": result.get("session_id", session_id)}
