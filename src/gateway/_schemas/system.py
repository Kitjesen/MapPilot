"""Internal Gateway system request and response models."""

from __future__ import annotations

from typing import Any

from pydantic import BaseModel, Field

from gateway._schemas.common import (
    GatewayResponseModel,
)


class DriverSwapRequest(BaseModel):
    """POST /api/v1/driver/swap request body."""

    driver: str = Field(min_length=1, max_length=64)
    config: dict[str, Any] = Field(default_factory=dict)


class DriverSwapResponse(GatewayResponseModel):
    """POST /api/v1/driver/swap response."""

    schema_version: int = 1
    success: bool
    message: str = ""
    swap_time_ms: float = 0.0
    driver: str | None = None
    detail: dict[str, Any] | None = None
    error: str | None = None


__all__ = (
    "DriverSwapRequest",
    "DriverSwapResponse",
)
