"""Operator sensor commands forwarded to the bound Host source-owner module."""

from __future__ import annotations

import asyncio
import time
from dataclasses import asdict
from typing import Annotated, Any, Literal

from fastapi import FastAPI
from fastapi.responses import JSONResponse
from pydantic import BaseModel, ConfigDict, Field, StrictBool, StrictInt, StrictStr, ValidationError

from message.generated.enums import SensorControlKind
from runtime.contracts.sensor_control import SensorControlConflict, SensorControlRequest

CONTROL_TIMEOUT_S = 2.0
FiniteNumber = Annotated[float, Field(strict=True, allow_inf_nan=False)]
IdentityText = Annotated[StrictStr, Field(min_length=1, max_length=255)]


class _Params(BaseModel):
    model_config = ConfigDict(extra="forbid")


class _EnabledParams(_Params):
    enabled: StrictBool


class _RateParams(_Params):
    rate_hz: Annotated[FiniteNumber, Field(gt=0)]


class _NoiseParams(_EnabledParams):
    unit: Annotated[StrictStr, Field(min_length=1, max_length=15)]
    stddev: Annotated[FiniteNumber, Field(ge=0)]
    seed: Annotated[StrictInt, Field(ge=0, lt=2**64)]


class _GimbalParams(_Params):
    pan_deg: FiniteNumber
    tilt_deg: FiniteNumber


class SensorControlBody(BaseModel):
    model_config = ConfigDict(extra="forbid")

    request_id: IdentityText
    expected_product_session_id: IdentityText
    operation: Literal["query", "enabled", "rate", "noise", "noise_enabled", "recording", "gimbal"]
    params: dict[str, Any] = Field(default_factory=dict)


_OPERATIONS: dict[str, tuple[SensorControlKind, type[_Params]]] = {
    "query": (SensorControlKind.QUERY, _Params),
    "enabled": (SensorControlKind.SET_ENABLED, _EnabledParams),
    "rate": (SensorControlKind.SET_RATE, _RateParams),
    "noise": (SensorControlKind.CONFIGURE_NOISE, _NoiseParams),
    "noise_enabled": (SensorControlKind.SET_NOISE_ENABLED, _EnabledParams),
    "recording": (SensorControlKind.SET_RECORDING, _EnabledParams),
    "gimbal": (SensorControlKind.SET_GIMBAL, _GimbalParams),
}


def register_sensor_control_routes(app: FastAPI, gw: Any = None) -> None:
    @app.post("/api/v1/sensors/{sensor_id}/control", tags=["sensors"])
    async def control_sensor(sensor_id: str, body: SensorControlBody):
        operation, schema = _OPERATIONS[body.operation]
        try:
            params = schema.model_validate(body.params).model_dump()
        except ValidationError as exc:
            return JSONResponse(
                {"error": "invalid_sensor_control", "request_id": body.request_id, "message": str(exc)},
                status_code=422,
            )

        module = getattr(gw, "_sensor_control_module", None)
        if module is None:
            return JSONResponse(
                {"error": "sensor_control_unavailable", "request_id": body.request_id},
                status_code=503,
            )
        try:
            binding = module.binding(sensor_id)
        except (KeyError, ValueError):
            return JSONResponse(
                {"error": "unknown_sensor", "request_id": body.request_id, "sensor_id": sensor_id},
                status_code=404,
            )
        if body.expected_product_session_id != binding["product_session_id"]:
            return JSONResponse(
                {"error": "product_session_mismatch", "request_id": body.request_id, "sensor_id": sensor_id},
                status_code=409,
            )

        if body.operation == "noise":
            params = {
                "enabled": params["enabled"],
                "noise_unit": params["unit"],
                "noise_stddev": params["stddev"],
                "noise_seed": params["seed"],
            }
        try:
            command = SensorControlRequest(
                request_id=body.request_id,
                product_session_id=binding["product_session_id"],
                session_id=binding["session_id"],
                sensor_id=binding["sensor_id"],
                model_generation=binding["model_generation"],
                reset_generation=binding["reset_generation"],
                deadline_s=time.time() + CONTROL_TIMEOUT_S,
                operation=operation,
                **params,
            )
        except ValueError as exc:
            return JSONResponse(
                {"error": "invalid_sensor_control", "request_id": body.request_id, "message": str(exc)},
                status_code=422,
            )
        try:
            result = await asyncio.wait_for(asyncio.to_thread(module.execute, command), timeout=CONTROL_TIMEOUT_S)
        except SensorControlConflict:
            return JSONResponse({"error": "request_id_conflict", "request_id": body.request_id}, status_code=409)
        except (TimeoutError, asyncio.TimeoutError):
            return JSONResponse(
                {"error": "sensor_control_timeout", "request_id": body.request_id, "sensor_id": sensor_id},
                status_code=504,
            )

        # JSONResponse rejects nonfinite numbers; do not rewrite the owner's ACK.
        try:
            return JSONResponse(asdict(result), status_code=200 if result.applied else 409)
        except (TypeError, ValueError):
            return JSONResponse(
                {"error": "invalid_sensor_control_ack", "request_id": body.request_id},
                status_code=502,
            )
