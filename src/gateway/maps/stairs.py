"""Map-bound stair annotations persisted atomically by native mapd."""

from __future__ import annotations

import json
from typing import Any

from fastapi import HTTPException, Path, Query
from fastapi.responses import JSONResponse
from pydantic import BaseModel, ConfigDict, Field, model_validator

from gateway.maps.transport import map_client, safe_map_name


class StairPoint(BaseModel):
    model_config = ConfigDict(extra="forbid", strict=True, allow_inf_nan=False)

    x: float
    y: float
    z: float


class StairBounds(BaseModel):
    model_config = ConfigDict(extra="forbid")

    min: StairPoint
    max: StairPoint

    @model_validator(mode="after")
    def ordered_bounds(self) -> StairBounds:
        if any(getattr(self.min, axis) >= getattr(self.max, axis) for axis in ("x", "y", "z")):
            raise ValueError("bounds min must be below max on every axis")
        return self


class StairPutRequest(BaseModel):
    model_config = ConfigDict(extra="forbid", str_strip_whitespace=True)

    name: str = Field(min_length=1, max_length=256)
    content_epoch: int = Field(gt=0, strict=True)
    frame_id: str = Field(min_length=1, max_length=128)
    bounds: StairBounds
    lower: StairPoint
    upper: StairPoint

    @model_validator(mode="after")
    def ordered_entries(self) -> StairPutRequest:
        if self.lower.z >= self.upper.z:
            raise ValueError("upper entry must be higher than lower entry")
        return self


class StairSurface(BaseModel):
    s_min: float
    s_max: float
    lateral_min: float
    lateral_max: float
    surface_z: float
    observed_points: int


class StairGeometry(BaseModel):
    recognized: bool
    reason: str
    origin: StairPoint | None = None
    lower: StairPoint | None = None
    upper: StairPoint | None = None
    axis_x: float | None = None
    axis_y: float | None = None
    width_m: float | None = None
    mean_rise_m: float | None = None
    mean_run_m: float | None = None
    surfaces: list[StairSurface] = Field(default_factory=list)


class StairAnnotation(StairPutRequest):
    id: str
    geometry: StairGeometry | None = None


class StairListResponse(BaseModel):
    success: bool
    map_id: str
    content_epoch: int
    stairs: list[StairAnnotation]


def _stair_request(gw: Any, action: str, map_id: str, **arguments: Any) -> dict[str, Any] | JSONResponse:
    error = safe_map_name(map_id)
    if error:
        raise HTTPException(status_code=422, detail=error)
    try:
        response = map_client(gw).service(action, map_id=map_id, **arguments)
    except (RuntimeError, OSError) as exc:
        raise HTTPException(status_code=503, detail="Native map service is unavailable.") from exc
    if response.get("success") is True:
        return response
    reason = response.get("reason_code")
    if reason in {"map_not_found", "stair_not_found"}:
        status = 404
    elif reason in {"content_epoch_conflict", "frame_id_conflict", "poi_name_conflict", "map_write_in_progress"}:
        status = 409
    elif reason in {"invalid_stair", "invalid_request", "invalid_map_name"}:
        status = 422
    else:
        status = 503
    return JSONResponse(response, status_code=status)


def register_stair_routes(app: Any, gw: Any) -> None:
    @app.get("/api/v1/maps/{name}/stairs", response_model=StairListResponse, response_model_exclude_unset=True)
    def list_stairs(name: str):
        return _stair_request(gw, "list_stairs", name)

    @app.put("/api/v1/maps/{name}/stairs/{stair_id}", response_model=StairListResponse, response_model_exclude_unset=True)
    def put_stair(
        name: str,
        body: StairPutRequest,
        stair_id: str = Path(min_length=1, max_length=128, pattern=r"^[A-Za-z0-9_][A-Za-z0-9_.-]*$"),
    ):
        return _stair_request(
            gw,
            "set_stair",
            name,
            stair_id=stair_id,
            annotation_json=json.dumps(body.model_dump(), ensure_ascii=False, allow_nan=False),
        )

    @app.delete("/api/v1/maps/{name}/stairs/{stair_id}", response_model=StairListResponse, response_model_exclude_unset=True)
    def delete_stair(
        name: str,
        stair_id: str = Path(min_length=1, max_length=128, pattern=r"^[A-Za-z0-9_][A-Za-z0-9_.-]*$"),
        content_epoch: int = Query(gt=0),
    ):
        return _stair_request(gw, "delete_stair", name, stair_id=stair_id, content_epoch=content_epoch)
