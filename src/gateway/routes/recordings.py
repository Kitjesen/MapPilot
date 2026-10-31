"""HTTP file-management adapter for native recording sessions."""

from __future__ import annotations

import asyncio
import json
import tarfile
from collections.abc import Iterator
from pathlib import Path, PurePosixPath
from typing import Any

from fastapi import Query

from gateway.services.recording import NativeRecordingError
from gateway.services.recording_files import resolve_declared_artifact
from gateway.services.recording_media import (
    clear_recording_media_cache,
    dds_scene_artifact,
    recording_camera_frame,
    recording_scene_info,
)


def _error(exc: NativeRecordingError):
    from fastapi.responses import JSONResponse

    return JSONResponse(
        status_code=exc.status_code,
        content={
            "error": exc.code,
            "detail": str(exc) if exc.code.startswith(("recording_series_", "recording_scene_", "recording_camera_")) else None,
        },
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
                "required_topics": list(raw_child.get("required_topics", []))
                if isinstance(raw_child.get("required_topics"), list)
                else [],
                "exit_code": raw_child.get("exit_code"),
            }
        )
    public["children"] = children
    public["artifacts"] = [
        {"path": artifact, "download": f"files/{artifact}"}
        for child in children
        for artifact in child["artifacts"]
    ]
    return public


def _dds_artifact(service, session_id: str) -> tuple[Path, dict[str, Any]]:
    session = service.manifest(session_id=session_id)
    for child in session.get("children", []):
        if isinstance(child, dict) and child.get("name") == "dds":
            for name in child.get("artifacts", []):
                if isinstance(name, str) and name.endswith((".mcap", ".mcap.tmp")):
                    path, _metadata = resolve_declared_artifact(
                        service, session_id=session_id, artifact_name=name
                    )
                    return path, session
    raise NativeRecordingError(
        "recording_dds_artifact_missing", "recording has no declared DDS MCAP artifact", status_code=404
    )


def _archive_entries(service, session_id: str) -> tuple[dict[str, Any], list[tuple[str, Path]]]:
    session = service.manifest(session_id=session_id)
    if session.get("state") not in {"completed", "failed"}:
        raise NativeRecordingError(
            "recording_not_terminal", "stop the recording before exporting it", status_code=409
        )
    entries: list[tuple[str, Path]] = []
    names: set[str] = set()
    for child in session.get("children", []):
        if not isinstance(child, dict):
            continue
        for name in child.get("artifacts", []):
            if not isinstance(name, str) or name in names:
                continue
            path, metadata = resolve_declared_artifact(service, session_id=session_id, artifact_name=name)
            entries.append((metadata["path"], path))
            names.add(name)
    return _public_manifest(session), entries


def _tar_stream(session: dict[str, Any], entries: list[tuple[str, Path]]) -> Iterator[bytes]:
    """Stream already-authorized files without staging a second large archive."""
    prefix = str(session["session_id"])
    manifest = (json.dumps(session, ensure_ascii=False, indent=2) + "\n").encode("utf-8")
    info = tarfile.TarInfo(f"{prefix}/session.json")
    info.size = len(manifest)
    yield info.tobuf(format=tarfile.PAX_FORMAT)
    yield manifest
    yield b"\0" * (-len(manifest) % tarfile.BLOCKSIZE)
    for name, path in entries:
        info = tarfile.TarInfo(f"{prefix}/{name}")
        info.size = path.stat().st_size
        yield info.tobuf(format=tarfile.PAX_FORMAT)
        with path.open("rb") as source:
            while block := source.read(1024 * 1024):
                yield block
        yield b"\0" * (-info.size % tarfile.BLOCKSIZE)
    yield b"\0" * (2 * tarfile.BLOCKSIZE)


def register_recording_routes(app, gw) -> None:
    """Register list/detail/download/delete without duplicating native state."""

    @app.get("/api/v1/recordings/streams")
    async def recording_streams():
        return await asyncio.to_thread(gw._recording.streams)

    @app.get("/api/v1/recordings/catalog")
    async def recording_topic_catalog():
        try:
            catalog = await asyncio.to_thread(gw._recording.catalog)
        except NativeRecordingError as exc:
            return _error(exc)
        return {"ok": True, **catalog}

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

    @app.get("/api/v1/recordings/{session_id}/archive")
    async def recording_archive(session_id: str):
        from fastapi.responses import StreamingResponse

        try:
            session, entries = await asyncio.to_thread(_archive_entries, gw._recording, session_id)
        except NativeRecordingError as exc:
            return _error(exc)
        return StreamingResponse(
            _tar_stream(session, entries),
            media_type="application/x-tar",
            headers={"Content-Disposition": f'attachment; filename="{session["session_id"]}.tar"'},
        )

    @app.get("/api/v1/recordings/{session_id}/summary")
    async def recording_summary(session_id: str):
        try:
            artifact, session = await asyncio.to_thread(_dds_artifact, gw._recording, session_id)
            summary = await asyncio.to_thread(gw._recording.inspect_artifact, artifact)
        except NativeRecordingError as exc:
            return _error(exc)
        counts = {item["topic"]: item["message_count"] for item in summary.get("topics", [])}
        selected = [
            topic
            for child in session.get("children", [])
            if isinstance(child, dict) and child.get("name") == "dds"
            for topic in child.get("selected_topics", [])
        ]
        summary["missing_topics"] = [topic for topic in selected if counts.get(topic, 0) == 0]
        return {"ok": True, "summary": summary}

    @app.get("/api/v1/recordings/{session_id}/series")
    async def recording_series(
        session_id: str,
        topic: str,
        field: str | None = None,
        start: float = 0.0,
        end: float | None = None,
        max_points: int = 2000,
    ):
        try:
            artifact, session = await asyncio.to_thread(_dds_artifact, gw._recording, session_id)
            series = await asyncio.to_thread(
                gw._recording.series_artifact,
                artifact,
                topic=topic,
                field=field,
                start=start,
                end=end,
                max_points=max_points,
            )
        except NativeRecordingError as exc:
            return _error(exc)
        return {"ok": True, "session_id": session_id, "state": session["state"], "series": series}

    @app.get("/api/v1/recordings/{session_id}/scene-info")
    async def scene_info(session_id: str):
        try:
            return await asyncio.to_thread(recording_scene_info, gw._recording, session_id)
        except NativeRecordingError as exc:
            return _error(exc)

    @app.get("/api/v1/recordings/{session_id}/scene")
    async def scene_window(
        session_id: str, start: float = 0.0, end: float = 5.0,
        max_frames: int = 32, max_points: int = 2048, max_poses: int = 2000,
        cloud_topic: str | None = None, pose_topic: str | None = None,
        path_topic: list[str] = Query(default=[]),
    ):
        from fastapi.responses import JSONResponse

        try:
            artifact = await asyncio.to_thread(dds_scene_artifact, gw._recording, session_id)
            scene = None if artifact is None else await asyncio.to_thread(
                gw._recording.scene_artifact, artifact, start=start, end=end,
                max_frames=max_frames, max_points=max_points, max_poses=max_poses,
                cloud_topic=cloud_topic, pose_topic=pose_topic,
                path_topics=tuple(path_topic),
            )
            return await asyncio.to_thread(
                JSONResponse, content={"ok": True, "session_id": session_id, "scene": scene},
            )
        except NativeRecordingError as exc:
            return _error(exc)

    @app.get("/api/v1/recordings/{session_id}/camera-frame")
    async def camera_frame(session_id: str, t: float, segment: int, camera_id: str = "color", width: int = 960):
        from fastapi.responses import Response

        try:
            frame = await asyncio.to_thread(
                recording_camera_frame, gw._recording, session_id,
                t=t, segment=segment, camera_id=camera_id, width=width,
            )
        except NativeRecordingError as exc:
            return _error(exc)
        if frame is None:
            return Response(status_code=204)
        payload, headers = frame
        return Response(payload, media_type="image/jpeg", headers=headers)

    @app.post("/api/v1/recordings/{session_id}/validate")
    async def recording_validate(session_id: str):
        try:
            artifact, _session = await asyncio.to_thread(_dds_artifact, gw._recording, session_id)
            validation = await asyncio.to_thread(gw._recording.inspect_artifact, artifact, validate=True)
        except NativeRecordingError as exc:
            return _error(exc)
        return {"ok": True, "validation": validation}

    @app.delete("/api/v1/recordings/{session_id}")
    async def recording_remove(session_id: str):
        try:
            result = await asyncio.to_thread(gw._recording.remove, session_id=session_id)
        except NativeRecordingError as exc:
            return _error(exc)
        clear_recording_media_cache()
        return {"ok": True, "session_id": result.get("session_id", session_id)}
