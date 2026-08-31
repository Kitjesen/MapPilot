"""Canonical semantic place provisioning routes.

These endpoints expose the native POI-backed :mod:`memory.spatial.places` catalog to
operators, BIM importers, and admin tools.  They intentionally do not read POI
files directly or own a database; native mapd remains the only authority
for saved-map binding metadata.
"""

from __future__ import annotations

import json
import time
from collections.abc import Mapping, Sequence
from typing import Any, Literal

from fastapi import HTTPException, Query
from pydantic import BaseModel, ConfigDict, Field, field_validator

from gateway.services.mapd_transport import map_client
from memory.spatial.places import (
    PLACE_SCHEMA_VERSION,
    PlaceCatalog,
    PlaceCatalogError,
    PlaceRef,
    PlaceRegistration,
    PlaceResolution,
)

MAX_TEXT_LENGTH = 128
MAX_ALIASES = 16


class _MapdPlacePort:
    """Expose only the native mapd operations required by ``PlaceCatalog``."""

    def __init__(self, gw: Any) -> None:
        self._client = map_client(gw)

    def list_maps(self) -> Mapping[str, Any]:
        return self._client.service("list_maps")

    def get_record(self, map_id: str) -> Mapping[str, Any]:
        return self._client.service("get_record", map_id=map_id)

    def poi_list(self, map_id: str = "") -> Mapping[str, Any]:
        return self._client.service("list_poi", map_id=map_id)

    def poi_set(self, command: Mapping[str, Any]) -> Mapping[str, Any]:
        yaw = command.get("yaw")
        return self._client.service(
            "set_poi",
            map_id=str(command.get("map_id") or ""),
            name=str(command.get("name") or ""),
            x_m=float(command.get("x", 0.0)),
            y_m=float(command.get("y", 0.0)),
            z_m=float(command.get("z", 0.0)),
            yaw_rad=0.0 if yaw is None else float(yaw),
            has_yaw=yaw is not None,
            frame_id=str(command.get("frame_id") or "map"),
            tags_json=json.dumps(command.get("tags") or {}, sort_keys=True),
        )


class PlaceCreateRequest(BaseModel):
    """Caller-supplied semantic place data.

    Map version, version id, and point-cloud hash are deliberately forbidden:
    the service derives those fields from native maps at upsert time.
    """

    model_config = ConfigDict(extra="forbid")

    place_id: str = Field(min_length=1, max_length=MAX_TEXT_LENGTH)
    name: str = Field(min_length=1, max_length=MAX_TEXT_LENGTH)
    map_id: str = Field(min_length=1, max_length=MAX_TEXT_LENGTH)
    x: float
    y: float
    z: float
    source: str = Field(min_length=1, max_length=MAX_TEXT_LENGTH)
    aliases: list[str] = Field(default_factory=list, max_length=MAX_ALIASES)
    kind: str = Field(default="", max_length=MAX_TEXT_LENGTH)
    building_id: str = Field(default="", max_length=MAX_TEXT_LENGTH)
    floor_id: str = Field(default="", max_length=MAX_TEXT_LENGTH)
    yaw: float | None = None
    connector_id: str = Field(default="", max_length=MAX_TEXT_LENGTH)
    confidence: float = Field(default=1.0, ge=0.0, le=1.0)
    frame_id: str | None = Field(default=None, min_length=1, max_length=MAX_TEXT_LENGTH)

    @field_validator(
        "place_id",
        "name",
        "map_id",
        "source",
        "kind",
        "building_id",
        "floor_id",
        "connector_id",
        "frame_id",
        mode="before",
    )
    @classmethod
    def strip_text(cls, value: Any) -> Any:
        if value is None:
            return value
        if not isinstance(value, str):
            return value
        return value.strip()

    @field_validator("aliases", mode="before")
    @classmethod
    def validate_aliases(cls, value: Any) -> list[str]:
        if value is None:
            return []
        if not isinstance(value, Sequence) or isinstance(value, (str, bytes)):
            raise ValueError("aliases must be a list of strings")
        aliases: list[str] = []
        seen: set[str] = set()
        for raw_alias in value:
            if not isinstance(raw_alias, str):
                raise ValueError("aliases must be a list of strings")
            alias = raw_alias.strip()
            if not alias:
                raise ValueError("aliases cannot contain blank values")
            if len(alias) > MAX_TEXT_LENGTH:
                raise ValueError(f"aliases cannot exceed {MAX_TEXT_LENGTH} characters")
            if alias in seen:
                continue
            seen.add(alias)
            aliases.append(alias)
        if len(aliases) > MAX_ALIASES:
            raise ValueError(f"aliases cannot exceed {MAX_ALIASES} entries")
        return aliases

    @field_validator("x", "y", "z", "yaw", "confidence", mode="before")
    @classmethod
    def finite_number(cls, value: Any) -> Any:
        if value is None:
            return value
        if isinstance(value, bool):
            raise ValueError("must be a finite number")
        import math

        try:
            number = float(value)
        except (TypeError, ValueError) as exc:
            raise ValueError("must be a finite number") from exc
        if not math.isfinite(number):
            raise ValueError("must be finite")
        return value

    def to_registration(self) -> PlaceRegistration:
        return PlaceRegistration(
            place_id=self.place_id,
            name=self.name,
            map_id=self.map_id,
            x=self.x,
            y=self.y,
            z=self.z,
            source=self.source,
            aliases=tuple(self.aliases),
            kind=self.kind,
            building_id=self.building_id,
            floor_id=self.floor_id,
            yaw=self.yaw,
            connector_id=self.connector_id,
            confidence=self.confidence,
            frame_id=self.frame_id,
        )


class PlaceUpsertResponse(BaseModel):
    schema_version: Literal["lingtu.place_admin.v1"] = "lingtu.place_admin.v1"
    ok: bool
    place: dict[str, Any]
    ts: float


class PlaceListResponse(BaseModel):
    schema_version: Literal["lingtu.place_admin.v1"] = "lingtu.place_admin.v1"
    ok: bool
    places: list[dict[str, Any]]
    count: int
    ts: float


class PlaceResolveResponse(BaseModel):
    schema_version: Literal["lingtu.place_admin.v1"] = "lingtu.place_admin.v1"
    ok: bool
    status: str
    query: str
    place: dict[str, Any] | None
    candidates: list[dict[str, Any]]
    ts: float


def register_place_routes(app, gw) -> None:
    """Register canonical semantic place administration routes."""

    def catalog() -> PlaceCatalog:
        try:
            return PlaceCatalog(_MapdPlacePort(gw))
        except RuntimeError as exc:
            raise HTTPException(status_code=503, detail=str(exc)) from exc

    @app.post(
        "/api/v1/places",
        summary="Create or update a canonical semantic place on a native map",
        response_model=PlaceUpsertResponse,
    )
    def post_place(body: PlaceCreateRequest):
        try:
            place = catalog().upsert(body.to_registration())
        except ValueError as exc:
            raise HTTPException(status_code=422, detail=str(exc)) from exc
        except PlaceCatalogError as exc:
            message = str(exc)
            status_code = 409 if _looks_like_conflict(message) else 400
            raise HTTPException(status_code=status_code, detail=message) from exc
        except (TypeError, RuntimeError) as exc:
            raise HTTPException(status_code=503, detail=str(exc)) from exc
        return {
            "ok": True,
            "place": _serialize_place(place),
            "ts": time.time(),
        }

    @app.get(
        "/api/v1/places",
        summary="List canonical semantic places",
        response_model=PlaceListResponse,
    )
    def get_places(map_id: str | None = Query(default=None, min_length=1, max_length=MAX_TEXT_LENGTH)):
        try:
            places = [_serialize_place(place) for place in catalog().load(map_id)]
        except PlaceCatalogError as exc:
            raise HTTPException(status_code=400, detail=str(exc)) from exc
        except (TypeError, RuntimeError) as exc:
            raise HTTPException(status_code=503, detail=str(exc)) from exc
        return {
            "ok": True,
            "places": places,
            "count": len(places),
            "ts": time.time(),
        }

    @app.get(
        "/api/v1/places/resolve",
        summary="Resolve a canonical semantic place by name or alias",
        response_model=PlaceResolveResponse,
    )
    def resolve_place(
        q: str = Query(min_length=1, max_length=MAX_TEXT_LENGTH),
        map_id: str | None = Query(default=None, alias="map", min_length=1, max_length=MAX_TEXT_LENGTH),
        building: str | None = Query(default=None, min_length=1, max_length=MAX_TEXT_LENGTH),
        floor: str | None = Query(default=None, min_length=1, max_length=MAX_TEXT_LENGTH),
        kind: str | None = Query(default=None, min_length=1, max_length=MAX_TEXT_LENGTH),
    ):
        try:
            resolution = catalog().resolve(
                q,
                map_id=map_id,
                building_id=building,
                floor_id=floor,
                kind=kind,
            )
        except PlaceCatalogError as exc:
            raise HTTPException(status_code=400, detail=str(exc)) from exc
        except (TypeError, RuntimeError) as exc:
            raise HTTPException(status_code=503, detail=str(exc)) from exc
        return _serialize_resolution(resolution)


def _serialize_resolution(resolution: PlaceResolution) -> dict[str, Any]:
    return {
        "ok": resolution.status == "resolved",
        "status": resolution.status,
        "query": resolution.query,
        "place": _serialize_place(resolution.place) if resolution.place is not None else None,
        "candidates": [_serialize_place(place) for place in resolution.candidates],
        "ts": time.time(),
    }


def _serialize_place(place: PlaceRef) -> dict[str, Any]:
    provenance = _json_mapping(place.provenance)
    binding = {
        "schema_version": PLACE_SCHEMA_VERSION,
        "status": provenance.get("binding_status") or ("bound" if place.executable else place.non_executable_reason),
        "stored": _json_mapping(provenance.get("stored_binding")),
        "current": _json_mapping(provenance.get("current_binding")),
        "errors": list(provenance.get("binding_errors") or []),
    }
    return {
        "place_id": place.place_id,
        "name": place.name,
        "aliases": list(place.aliases),
        "kind": place.kind,
        "building_id": place.building_id,
        "floor_id": place.floor_id,
        "map_id": place.map_id,
        "content_epoch": place.content_epoch,
        "frame_id": place.frame_id,
        "x": place.x,
        "y": place.y,
        "z": place.z,
        "yaw": place.yaw,
        "connector_id": place.connector_id,
        "source": place.source,
        "confidence": place.confidence,
        "executable": place.executable,
        "reason": place.non_executable_reason,
        "binding": binding,
        "provenance": provenance,
    }


def _json_mapping(value: Any) -> dict[str, Any]:
    if not isinstance(value, Mapping):
        return {}
    return {str(key): _json_value(item) for key, item in value.items()}


def _json_value(value: Any) -> Any:
    if isinstance(value, Mapping):
        return _json_mapping(value)
    if isinstance(value, tuple):
        return [_json_value(item) for item in value]
    if isinstance(value, list):
        return [_json_value(item) for item in value]
    return value


def _looks_like_conflict(message: str) -> bool:
    lowered = message.casefold()
    return "already" in lowered or "conflict" in lowered or "belongs to another" in lowered
