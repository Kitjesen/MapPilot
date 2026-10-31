"""Legacy localization entry point; use NavigationClient for new integrations."""

from __future__ import annotations

from typing import TYPE_CHECKING

from .models import CommandResult, Pose2D

if TYPE_CHECKING:
    from .client import LingTuClient


class LocalizationClient:
    """Preserve the original localization commands through NavigationClient."""

    def __init__(self, client: LingTuClient) -> None:
        self._client = client

    def relocalize(
        self,
        map_name: str,
        *,
        initial_pose: Pose2D,
        request_id: str | None = None,
    ) -> CommandResult:
        """Relocalize on the active map from a caller-provided pose seed."""
        return self._client.navigation.relocalize(map_name, initial_pose=initial_pose, request_id=request_id)

    def global_relocalize(
        self,
        map_name: str,
        *,
        request_id: str | None = None,
    ) -> CommandResult:
        """Relocalize on the active map without a pose seed."""
        return self._client.navigation.global_relocalize(map_name, request_id=request_id)

    def start_map_tracking(
        self,
        map_name: str,
        *,
        request_id: str | None = None,
    ) -> CommandResult:
        """Start continuous alignment against the active map."""
        return self._client.navigation.start_map_tracking(map_name, request_id=request_id)
