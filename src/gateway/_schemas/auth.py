"""Internal Gateway auth request and response models."""

from __future__ import annotations

from pydantic import BaseModel, Field

from gateway._schemas.common import (
    GatewayResponseModel,
)


class AuthLoginRequest(BaseModel):
    key: str = Field(default="", max_length=4096)


class AuthLoginResponse(GatewayResponseModel):
    ok: bool
    message: str


class AuthCheckResponse(GatewayResponseModel):
    auth_required: bool


__all__ = (
    "AuthCheckResponse",
    "AuthLoginRequest",
    "AuthLoginResponse",
)
