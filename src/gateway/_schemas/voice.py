"""Internal Gateway voice request and response models."""

from __future__ import annotations

from typing import Any, Literal

from pydantic import BaseModel, ConfigDict, Field, field_validator

from gateway._schemas.common import (
    CommandReceipt,
    GatewayResponseModel,
)


class VoiceTurnRequest(BaseModel):
    """AskMe voice-runtime bridge request."""

    model_config = ConfigDict(extra="forbid")

    text: str = Field(min_length=1, max_length=1024)
    operator_id: str = Field(default="", max_length=128)
    session_id: str = Field(default="", max_length=128)
    channel: str = Field(default="voice", max_length=32)
    robot_id: str | None = Field(default=None, max_length=128)
    site_id: str | None = Field(default=None, max_length=128)
    submit: bool = True
    metadata: dict[str, Any] = Field(default_factory=dict)
    request_id: str | None = Field(default=None, max_length=128)
    client_id: str | None = Field(default=None, max_length=128)

    @field_validator("text")
    @classmethod
    def normalized_text(cls, value: str) -> str:
        text = value.strip()
        if not text:
            raise ValueError("text must not be blank")
        return text

    @field_validator(
        "operator_id",
        "session_id",
        "channel",
        "robot_id",
        "site_id",
        "request_id",
        "client_id",
    )
    @classmethod
    def normalized_identifier(cls, value: str | None) -> str | None:
        if value is None:
            return None
        return value.strip()


class VoiceTurnResult(GatewayResponseModel):
    action_type: Literal["runtime"] = "runtime"
    spoken_reply: str = Field(min_length=1)
    text: str
    status: str
    submitted: bool
    accepted: bool
    replay: bool = False
    request_id: str | None = None
    client_id: str
    operator_id: str = ""
    session_id: str = ""
    channel: str = "voice"
    robot_id: str | None = None
    site_id: str | None = None
    metadata: dict[str, Any] = Field(default_factory=dict)
    command: CommandReceipt | None = None
    reason: str | None = None


class VoiceTurnResponse(GatewayResponseModel):
    handled: Literal[True] = True
    turn: VoiceTurnResult


__all__ = (
    "VoiceTurnRequest",
    "VoiceTurnResponse",
    "VoiceTurnResult",
)
