"""Source plugin contract for Thunder LCM endpoint processes."""

from __future__ import annotations

from collections.abc import Mapping
from typing import Any, Protocol, runtime_checkable

from .endpoint_service import LCMEndpointEvent, LCMEndpointService


@runtime_checkable
class LCMEndpointSource(Protocol):
    """Protocol implemented by endpoint-side hardware or smoke sources."""

    name: str

    def start(self, service: LCMEndpointService) -> None:
        """Attach to the endpoint service and publish normalized inputs."""
        ...

    def stop(self) -> None:
        """Release source resources."""
        ...

    def health(self) -> Mapping[str, Any]:
        """Return source-specific status."""
        ...

    def on_lingtu_message(self, event: LCMEndpointEvent) -> None:
        """Consume one decoded LingTu-to-endpoint command or path event."""
        ...
