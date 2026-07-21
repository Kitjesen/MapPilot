"""Internal Gateway common request and response models."""

from __future__ import annotations

from typing import Any, Literal

from pydantic import BaseModel, ConfigDict, Field

from runtime.runtime_interface import body_frame_id, map_frame_id

GATEWAY_MAP_FRAME_ID = map_frame_id()


GATEWAY_BODY_FRAME_ID = body_frame_id()


MapFrameId = Literal["map"]


class GatewayResponseModel(BaseModel):
    """Base model that documents known fields without stripping future additions."""

    model_config = ConfigDict(extra="allow")


class CommandReceipt(GatewayResponseModel):
    name: str
    request_id: str | None = None
    client_id: str
    accepted: bool
    replay: bool
    ts: float


class GatewayCommandErrorDetail(GatewayResponseModel):
    reason_code: str
    reason: str | None = None
    source: str | None = None
    path: str | None = None
    blockers: list[str] = Field(default_factory=list)
    advisories: list[str] = Field(default_factory=list)
    safety: dict[str, Any] | None = None
    preview: dict[str, Any] | None = None
    lease: dict[str, Any] | None = None
    state: str | None = None
    has_odometry: bool | None = None
    session_mode: str | None = None
    localization: dict[str, Any] | None = None
    error: str | None = None


class GatewayErrorResponse(GatewayResponseModel):
    schema_version: int = 1
    ok: Literal[False] = False
    error: str
    message: str | None = None
    detail: dict[str, Any] | GatewayCommandErrorDetail | None = None
    command: CommandReceipt | None = None


class ServerInfo(GatewayResponseModel):
    api_version: str
    time: float


class EndpointSpec(GatewayResponseModel):
    method: str
    path: str
    operation_id: str | None = None
    request_schema: str | None = None
    response_schema: str | None = None
    response_content_types: list[str] = Field(default_factory=list)
    status_codes: list[str] = Field(default_factory=list)


class SSEEventEnvelope(GatewayResponseModel):
    schema_version: int = 1
    event_id: int | None = None
    type: str
    ts: float
    data: Any = None


class TeleopSummary(GatewayResponseModel):
    active: bool
    clients: int


class NavigationFrameMismatch(GatewayResponseModel):
    source: str
    expected_frame: str
    received_frame: str


__all__ = (
    "GATEWAY_BODY_FRAME_ID",
    "GATEWAY_MAP_FRAME_ID",
    "CommandReceipt",
    "EndpointSpec",
    "GatewayCommandErrorDetail",
    "GatewayErrorResponse",
    "GatewayResponseModel",
    "MapFrameId",
    "NavigationFrameMismatch",
    "SSEEventEnvelope",
    "ServerInfo",
    "TeleopSummary",
)
