"""Current observations, retained history and native data recording."""

from __future__ import annotations

import urllib.request
from pathlib import Path
from typing import TYPE_CHECKING, Any, cast
from urllib.parse import quote, urlencode

from ._common import _download_file
from .models import CommandResult

if TYPE_CHECKING:
    from .client import LingTuClient


class ObservationClient:
    """Current observations, retained history and native data recording."""

    def __init__(self, client: LingTuClient) -> None:
        self._client = client

    def scene(self) -> dict[str, Any]:
        """Get the current scene graph (detected objects, relations, regions)."""
        return self._client._get("/api/v1/scene_graph")

    def camera_snapshot(self, *, viewer_token: str | None = None) -> bytes:
        """Fetch a JPEG camera snapshot from the robot."""
        url = self._client.base_url + "/api/v1/camera/snapshot"
        req = urllib.request.Request(url)
        if self._client._api_key:
            req.add_header("X-API-Key", self._client._api_key)
        if viewer_token is not None:
            req.add_header("Authorization", f"Bearer {viewer_token}")
        with urllib.request.urlopen(req, timeout=self._client._timeout) as r:
            return cast(bytes, r.read())

    def camera_status(self, *, viewer_token: str | None = None) -> dict[str, Any]:
        """Read camera frame availability and rates."""
        return self._client._get("/api/v1/camera/status", headers={"Authorization": f"Bearer {viewer_token}"} if viewer_token is not None else None)

    def media_status(self, *, viewer_token: str | None = None) -> dict[str, Any]:
        """Read the selected video endpoint and camera display configuration."""
        return self._client._get("/api/v1/webrtc/status", headers={"Authorization": f"Bearer {viewer_token}"} if viewer_token is not None else None)

    def camera_diagnostics(self, *, viewer_token: str | None = None) -> dict[str, Any]:
        """Read camera, video pipeline and WHEP diagnostic observations."""
        return self._client._get("/api/v1/camera/diagnostics", headers={"Authorization": f"Bearer {viewer_token}"} if viewer_token is not None else None)

    def temporal_memory(
        self, *, label: str | None = None, since: str | None = None,
        near_x: float | None = None, near_y: float | None = None,
        radius: float | None = None, limit: int = 100,
    ) -> dict[str, Any]:
        """Query temporal entity observations."""
        parameters = {key: value for key, value in {
            "label": label, "since": since, "near_x": near_x,
            "near_y": near_y, "radius": radius, "limit": limit if limit != 100 else None,
        }.items() if value is not None}
        query = "?" + urlencode(parameters) if parameters else ""
        return self._client._get("/api/v1/memory/temporal" + query)

    def search_memory(self, query: str) -> dict[str, Any]:
        """Text search requires an encoder matching the stored observation embeddings."""
        raise NotImplementedError(
            "The Gateway requires an embedding and provides no text encoder. "
            "Encode the query with the model used by the observation store, then call search_memory_embedding()."
        )

    def search_memory_embedding(
        self, embedding: list[float], *, since: str | None = None,
        top_k: int = 10, label: str | None = None,
    ) -> dict[str, Any]:
        """Search temporal memory with a vector from the store's embedding model."""
        body: dict[str, Any] = {"embedding": embedding, "top_k": top_k}
        if since is not None:
            body["since"] = since
        if label is not None:
            body["label"] = label
        return self._client._request_json("/api/v1/memory/temporal/semantic", body)

    def start_recording(
        self,
        prefix: str | None = None,
        *,
        duration: int = 600,
        capture_profile: str | None = None,
        task_id: str | None = None,
        camera: bool | None = None,
        minimum_free_gib: int | None = None,
        topics: list[str] | None = None,
    ) -> CommandResult:
        """Start a native MCAP recording session with safe capture choices."""
        body: dict[str, Any] = {"duration": duration}
        if prefix:
            body["prefix"] = prefix
        if capture_profile is not None:
            body["capture_profile"] = capture_profile
        if task_id is not None:
            body["task_id"] = task_id
        if camera is not None:
            body["camera"] = camera
        if minimum_free_gib is not None:
            body["minimum_free_gib"] = minimum_free_gib
        if topics is not None:
            body["topics"] = topics
        return self._client._command("/api/v1/recordings/start", body)

    def stop_recording(self, *, expected_session_id: str | None = None) -> CommandResult:
        """Stop the current recording, optionally requiring its session identity."""
        body = {} if expected_session_id is None else {"expected_session_id": expected_session_id}
        return self._client._command("/api/v1/recordings/stop", body)

    def recording_status(self) -> dict[str, Any]:
        """Get native MCAP recording status."""
        return self._client._get("/api/v1/recordings/status")

    def recordings(self, *, limit: int = 100) -> dict[str, Any]:
        """List retained recording sessions and available disk space."""
        return self._client._get(f"/api/v1/recordings?{urlencode({'limit': limit})}")

    def recording(self, session_id: str) -> dict[str, Any]:
        """Read one recording's public manifest and declared artifact list."""
        return self._client._get(f"/api/v1/recordings/{quote(session_id, safe='')}")

    def recording_catalog(self) -> dict[str, Any]:
        """Read selectable recording topics from the native catalog."""
        return self._client._get("/api/v1/recordings/catalog")

    def recording_streams(self) -> dict[str, Any]:
        """Read available recording streams and their capabilities."""
        return self._client._get("/api/v1/recordings/streams")

    def download_recording_file(self, session_id: str, artifact_path: str, target: str | Path) -> Path:
        """Download a declared artifact, preserving its nested relative path."""
        path = f"/api/v1/recordings/{quote(session_id, safe='')}/files/{quote(artifact_path, safe='/')}"
        return _download_file(self._client, path, target)

    def download_recording_archive(self, session_id: str, target: str | Path) -> Path:
        """Download a terminal recording as a tar archive."""
        return _download_file(self._client, f"/api/v1/recordings/{quote(session_id, safe='')}/archive", target)

    def recording_summary(self, session_id: str) -> dict[str, Any]:
        """Read recorded DDS topics, counts and missing selected topics."""
        return self._client._get(f"/api/v1/recordings/{quote(session_id, safe='')}/summary")

    def validate_recording(self, session_id: str) -> dict[str, Any]:
        """Ask the native decoder to validate the retained DDS artifact."""
        return self._client._request_json(f"/api/v1/recordings/{quote(session_id, safe='')}/validate")

    def delete_recording(self, session_id: str) -> dict[str, Any]:
        """Delete one recording through its owning service."""
        return self._client._request_json(f"/api/v1/recordings/{quote(session_id, safe='')}", method="DELETE")

    def recording_series(
        self, session_id: str, topic: str, *, field: str | None = None,
        start: float = 0.0, end: float | None = None, max_points: int = 2000,
    ) -> dict[str, Any]:
        """Read a bounded numeric series without replaying DDS messages."""
        parameters = {key: value for key, value in {
            "topic": topic, "field": field, "start": start, "end": end, "max_points": max_points,
        }.items() if value is not None}
        return self._client._get(f"/api/v1/recordings/{quote(session_id, safe='')}/series?{urlencode(parameters)}")

    def recording_scene_info(self, session_id: str) -> dict[str, Any]:
        """Read recorded scene timing and available preview streams."""
        return self._client._get(f"/api/v1/recordings/{quote(session_id, safe='')}/scene-info")

    def recording_scene(
        self, session_id: str, *, start: float = 0.0, end: float = 5.0,
        max_frames: int = 32, max_points: int = 2048, max_poses: int = 2000,
        cloud_topic: str | None = None, pose_topic: str | None = None,
        path_topics: list[str] | None = None,
    ) -> dict[str, Any]:
        """Read a bounded recorded scene with its original topic selection."""
        parameters = {key: value for key, value in {
            "start": start, "end": end, "max_frames": max_frames,
            "max_points": max_points, "max_poses": max_poses,
            "cloud_topic": cloud_topic, "pose_topic": pose_topic, "path_topic": path_topics,
        }.items() if value is not None}
        return self._client._get(f"/api/v1/recordings/{quote(session_id, safe='')}/scene?{urlencode(parameters, doseq=True)}")

    def recording_camera_frame(
        self, session_id: str, t: float, segment: int, *, camera_id: str = "color", width: int = 960,
    ) -> bytes:
        """Read one recorded JPEG, returning empty bytes when no frame is available."""
        query = urlencode({"t": t, "segment": segment, "camera_id": camera_id, "width": width})
        path = f"/api/v1/recordings/{quote(session_id, safe='')}/camera-frame?{query}"
        request = urllib.request.Request(self._client.base_url + path)
        if self._client._api_key:
            request.add_header("X-API-Key", self._client._api_key)
        with urllib.request.urlopen(request, timeout=self._client._timeout) as response:
            return cast(bytes, response.read())
