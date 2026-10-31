"""Current observations, retained history and native data recording."""

from __future__ import annotations

import urllib.request
from typing import TYPE_CHECKING, Any, cast

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

    def camera_snapshot(self) -> bytes:
        """Fetch a JPEG camera snapshot from the robot."""
        url = self._client.base_url + "/api/v1/camera/snapshot"
        req = urllib.request.Request(url)
        if self._client._api_key:
            req.add_header("X-API-Key", self._client._api_key)
        with urllib.request.urlopen(req, timeout=self._client._timeout) as r:
            return cast(bytes, r.read())

    def temporal_memory(self) -> dict[str, Any]:
        """Query temporal entity observations."""
        return self._client._get("/api/v1/memory/temporal")

    def search_memory(self, query: str) -> dict[str, Any]:
        """Semantic similarity search over temporal observations."""
        return self._client._request_json("/api/v1/memory/temporal/semantic", {"query": query})

    def start_recording(
        self,
        prefix: str | None = None,
        *,
        duration: int = 600,
        capture_profile: str | None = None,
        task_id: str | None = None,
        camera: bool | None = None,
        minimum_free_gib: int | None = None,
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
        return self._client._command("/api/v1/recordings/start", body)

    def stop_recording(self) -> CommandResult:
        """Stop the active native MCAP recording session."""
        return self._client._command("/api/v1/recordings/stop")

    def recording_status(self) -> dict[str, Any]:
        """Get native MCAP recording status."""
        return self._client._get("/api/v1/recordings/status")
