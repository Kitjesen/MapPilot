"""Internal Gateway control request and response models."""

from __future__ import annotations

from pydantic import BaseModel, ConfigDict, Field, field_validator, model_validator

from gateway._schemas.common import (
    CommandReceipt,
    GatewayResponseModel,
)
from gateway._schemas.navigation import (
    ConstructedGoalTarget,
)


class CmdVelRequest(BaseModel):
    vx: float
    vy: float = 0.0
    wz: float
    request_id: str | None = Field(default=None, max_length=128)
    client_id: str = Field(default="unknown", max_length=128)

    @field_validator("vx", "vy", "wz")
    @classmethod
    def finite(cls, v: float) -> float:
        import math

        if not math.isfinite(v):
            raise ValueError("must be finite")
        return v


class InstructionRequest(BaseModel):
    text: str = Field(min_length=1, max_length=1024)
    request_id: str | None = Field(default=None, max_length=128)
    client_id: str = Field(default="unknown", max_length=128)


class SafetyEstopRequest(BaseModel):
    """AskMe-compatible emergency-stop activation request."""

    model_config = ConfigDict(extra="forbid")

    enabled: bool
    request_id: str | None = Field(default=None, max_length=128)
    client_id: str | None = Field(default=None, max_length=128)


class VisualServoRequest(BaseModel):
    mode: str = Field(default="find", max_length=16)
    target: str | None = Field(default=None, max_length=256)
    request_id: str | None = Field(default=None, max_length=128)
    client_id: str = Field(default="unknown", max_length=128)

    @model_validator(mode="after")
    def validate_command(self) -> VisualServoRequest:
        self.mode = self.mode.strip().lower()
        if self.mode not in {"find", "follow", "stop"}:
            raise ValueError("mode must be find|follow|stop")
        if self.target is not None:
            self.target = self.target.strip()
        if self.mode != "stop" and not self.target:
            raise ValueError("target is required for find/follow")
        return self


class StopRequest(BaseModel):
    request_id: str | None = Field(default=None, max_length=128)
    client_id: str = Field(default="unknown", max_length=128)


class CancelRequest(BaseModel):
    reason: str = Field(default="client_cancel", max_length=256)
    request_id: str | None = Field(default=None, max_length=128)
    client_id: str = Field(default="unknown", max_length=128)


class ModeRequest(BaseModel):
    mode: str
    request_id: str | None = Field(default=None, max_length=128)
    client_id: str = Field(default="unknown", max_length=128)

    @field_validator("mode")
    @classmethod
    def valid_mode(cls, v: str) -> str:
        if v not in ("manual", "autonomous", "estop"):
            raise ValueError(f"mode must be manual|autonomous|estop, got {v!r}")
        return v


class LeaseRequest(BaseModel):
    action: str
    client_id: str = Field(default="unknown", max_length=128)
    request_id: str | None = Field(default=None, max_length=128)
    ttl: float = Field(default=30.0, gt=0, le=3600)

    @field_validator("action")
    @classmethod
    def valid_action(cls, v: str) -> str:
        if v not in ("acquire", "release", "renew"):
            raise ValueError(f"action must be acquire|release|renew, got {v!r}")
        return v


class ControlCommandResponse(GatewayResponseModel):
    schema_version: int = 1
    ok: bool = True
    status: str
    command: CommandReceipt
    goal: list[float] | None = None
    yaw: float | None = None
    frame_id: str | None = None
    instruction: str | None = None
    mode: str | None = None
    reason: str | None = None
    target: ConstructedGoalTarget | None = None


class SafetyEstopResponse(GatewayResponseModel):
    active: bool
    enabled: bool
    timestamp: float
    accepted: bool = True
    replay: bool = False
    request_id: str | None = None
    client_id: str = "unknown"
    control_boundary: str | None = None
    message: str = ""
    command: CommandReceipt | None = None


class LeaseResponse(GatewayResponseModel):
    schema_version: int = 1
    ok: bool = True
    status: str
    command: CommandReceipt
    holder: str | None = None
    active: bool | None = None
    expires_in: float | None = None


__all__ = (
    "CancelRequest",
    "CmdVelRequest",
    "ControlCommandResponse",
    "InstructionRequest",
    "LeaseRequest",
    "LeaseResponse",
    "ModeRequest",
    "SafetyEstopRequest",
    "SafetyEstopResponse",
    "StopRequest",
    "VisualServoRequest",
)
