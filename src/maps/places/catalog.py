"""Typed place projection over the native maps and POI interfaces."""

from __future__ import annotations

import math
import re
import unicodedata
from collections.abc import Mapping, Sequence
from dataclasses import dataclass, field
from typing import Any, Literal, Protocol

PLACE_SCHEMA_VERSION = "lingtu.place.v1"


class PlaceCatalogError(RuntimeError):
    """Raised when the native maps interface cannot satisfy a catalog operation."""


class MapsServiceProtocol(Protocol):
    """Native maps behavior consumed by :class:`PlaceCatalog`."""

    def list_maps(self) -> Mapping[str, Any]:
        """Return native map records."""
        ...

    def get_record(self, map_id: str) -> Mapping[str, Any]:
        """Return the current native record for one map."""
        ...

    def get_map_points(self, map_id: str, *, max_points: int = 0) -> Mapping[str, Any]:
        """Return native point-cloud binding metadata and sampled points."""
        ...

    def poi_list(self, map_id: str = "") -> Mapping[str, Any]:
        """Return native POIs for one map."""
        ...

    def poi_set(self, command: Mapping[str, Any]) -> Mapping[str, Any]:
        """Upsert one native POI."""
        ...


class NativeMapsAdapter:
    """Adapt current MapsModule and native service method names to one seam."""

    def __init__(self, service: object) -> None:
        self._service = service

    def list_maps(self) -> Mapping[str, Any]:
        """Return native map records."""
        return self._call("list_maps")

    def get_record(self, map_id: str) -> Mapping[str, Any]:
        """Return the current native record for one map."""
        return self._call("get_record", map_id)

    def get_map_points(self, map_id: str, *, max_points: int = 0) -> Mapping[str, Any]:
        """Return native point-cloud binding metadata and sampled points."""
        return self._call("get_map_points", map_id, max_points=max_points)

    def poi_list(self, map_id: str = "") -> Mapping[str, Any]:
        """Return native POIs for one map."""
        method = self._find_method("poi_list", "list_poi")
        return _mapping(method(map_id), operation="poi_list")

    def poi_set(self, command: Mapping[str, Any]) -> Mapping[str, Any]:
        """Upsert one native POI using either façade or native naming."""
        direct = self._find_method("poi_set", required=False)
        if direct is not None:
            return _mapping(direct(dict(command)), operation="poi_set")
        native = self._find_method("set_poi")
        payload = dict(command)
        return _mapping(
            native(
                str(payload["map_id"]),
                str(payload["name"]),
                x=float(payload["x"]),
                y=float(payload["y"]),
                z=float(payload["z"]),
                yaw=payload.get("yaw"),
                frame_id=str(payload["frame_id"]),
                tags=payload["tags"],
            ),
            operation="poi_set",
        )

    def _call(self, name: str, *args: Any, **kwargs: Any) -> Mapping[str, Any]:
        method = self._find_method(name)
        return _mapping(method(*args, **kwargs), operation=name)

    def _find_method(self, *names: str, required: bool = True):
        for owner in (self._service, getattr(self._service, "api", None)):
            if owner is None:
                continue
            for name in names:
                method = getattr(owner, name, None)
                if callable(method):
                    return method
        if required:
            joined = " or ".join(names)
            raise TypeError(f"maps service does not expose {joined}")
        return None


@dataclass(frozen=True, slots=True)
class PlaceRegistration:
    """Caller-supplied place data; XYZ coordinates are intentionally required."""

    place_id: str
    name: str
    map_id: str
    x: float
    y: float
    z: float
    source: str
    aliases: Sequence[str] = ()
    kind: str = ""
    building_id: str = ""
    floor_id: str = ""
    yaw: float | None = None
    connector_id: str = ""
    confidence: float = 1.0
    frame_id: str | None = None

    def __post_init__(self) -> None:
        for field_name in ("place_id", "name", "map_id", "source"):
            if not str(getattr(self, field_name) or "").strip():
                raise ValueError(f"{field_name} is required")
        for field_name in ("x", "y", "z"):
            _finite_float(getattr(self, field_name), name=field_name)
        if self.yaw is not None:
            _finite_float(self.yaw, name="yaw")
        confidence = _finite_float(self.confidence, name="confidence")
        if not 0.0 <= confidence <= 1.0:
            raise ValueError("confidence must be between 0 and 1")


@dataclass(frozen=True, slots=True)
class PlaceRef:
    """A place plus its saved-map execution binding."""

    place_id: str
    name: str
    aliases: tuple[str, ...]
    kind: str
    building_id: str
    floor_id: str
    map_id: str
    map_version: int | None
    version_id: str
    map_pcd_sha256: str
    frame_id: str
    x: float | None
    y: float | None
    z: float | None
    yaw: float | None
    connector_id: str
    source: str
    confidence: float | None
    executable: bool
    non_executable_reason: str | None = None
    provenance: Mapping[str, Any] = field(default_factory=dict)


ResolutionStatus = Literal["resolved", "ambiguous", "not_found", "stale_map"]


@dataclass(frozen=True, slots=True)
class PlaceResolution:
    """Deterministic result of resolving one exact canonical name or alias."""

    status: ResolutionStatus
    query: str
    place: PlaceRef | None
    candidates: tuple[PlaceRef, ...]


@dataclass(frozen=True, slots=True)
class _MapBinding:
    map_id: str
    map_version: int | None
    version_id: str
    map_pcd_sha256: str
    frame_id: str
    state: str
    complete: bool
    errors: tuple[str, ...]


class PlaceCatalog:
    """Project stable places from POI tags without owning another database."""

    def __init__(self, maps_service: MapsServiceProtocol | object) -> None:
        self._maps = maps_service if isinstance(maps_service, NativeMapsAdapter) else NativeMapsAdapter(maps_service)

    def upsert(self, registration: PlaceRegistration) -> PlaceRef:
        """Store one place against the map version currently reported by native maps."""

        if not isinstance(registration, PlaceRegistration):
            raise TypeError("registration must be a PlaceRegistration")
        binding = self._current_binding(registration.map_id)
        if not binding.complete:
            details = "; ".join(binding.errors) or "current map binding is incomplete"
            raise PlaceCatalogError(f"cannot bind place to {registration.map_id}: {details}")
        if registration.frame_id and registration.frame_id != binding.frame_id:
            raise ValueError(
                f"frame_id {registration.frame_id!r} does not match current map frame {binding.frame_id!r}"
            )
        self._reject_identity_conflicts(registration)

        aliases = _unique_text(registration.aliases)
        tags: dict[str, Any] = {
            "schema_version": PLACE_SCHEMA_VERSION,
            "place_id": registration.place_id.strip(),
            "aliases": list(aliases),
            "kind": registration.kind.strip(),
            "building_id": registration.building_id.strip(),
            "floor_id": normalize_floor_id(registration.floor_id),
            "connector_id": registration.connector_id.strip(),
            "map_version": binding.map_version,
            "version_id": binding.version_id,
            "map_pcd_sha256": binding.map_pcd_sha256,
            "source": registration.source.strip(),
            "confidence": float(registration.confidence),
        }
        command = {
            "map_id": binding.map_id,
            "name": registration.name.strip(),
            "x": float(registration.x),
            "y": float(registration.y),
            "z": float(registration.z),
            "yaw": None if registration.yaw is None else float(registration.yaw),
            "frame_id": binding.frame_id,
            "tags": tags,
        }
        response = self._maps.poi_set(command)
        if response.get("success") is not True:
            message = str(response.get("message") or "native poi_set failed")
            raise PlaceCatalogError(message)
        return PlaceRef(
            place_id=tags["place_id"],
            name=command["name"],
            aliases=aliases,
            kind=tags["kind"],
            building_id=tags["building_id"],
            floor_id=tags["floor_id"],
            map_id=binding.map_id,
            map_version=binding.map_version,
            version_id=binding.version_id,
            map_pcd_sha256=binding.map_pcd_sha256,
            frame_id=binding.frame_id,
            x=command["x"],
            y=command["y"],
            z=command["z"],
            yaw=command["yaw"],
            connector_id=tags["connector_id"],
            source=tags["source"],
            confidence=tags["confidence"],
            executable=True,
            provenance={
                "schema_version": PLACE_SCHEMA_VERSION,
                "binding_status": "bound",
                "current_binding": _binding_dict(binding),
            },
        )

    register = upsert

    def _reject_identity_conflicts(self, registration: PlaceRegistration) -> None:
        wanted_name = _normalized_identifier(registration.name)
        wanted_storage_name = registration.name.strip()
        wanted_id = registration.place_id.strip()
        map_ids = set(self._map_ids())
        map_ids.add(registration.map_id)
        for map_id in sorted(map_ids):
            response = self._maps.poi_list(map_id)
            if response.get("success") is not True:
                message = str(response.get("message") or f"native poi_list failed for {map_id}")
                raise PlaceCatalogError(message)
            pois = response.get("pois")
            if not isinstance(pois, Mapping):
                raise PlaceCatalogError(f"native poi_list returned invalid POIs for {map_id}")

            for existing_name, raw_poi in pois.items():
                tags = raw_poi.get("tags") if isinstance(raw_poi, Mapping) else None
                if not isinstance(tags, Mapping) or tags.get("schema_version") != PLACE_SCHEMA_VERSION:
                    continue
                existing_id = str(tags.get("place_id") or "").strip()
                existing_name_text = str(existing_name).strip()
                same_storage_location = map_id == registration.map_id and existing_name_text == wanted_storage_name
                if existing_id == wanted_id and not same_storage_location:
                    raise PlaceCatalogError(
                        f"place_id {wanted_id!r} is already bound to {existing_name!r} on map {map_id!r}"
                    )
                same_normalized_name = _normalized_identifier(existing_name_text) == wanted_name
                if map_id == registration.map_id and same_normalized_name and existing_id != wanted_id:
                    raise PlaceCatalogError(f"place name {registration.name!r} already belongs to another identity")

    def load(self, map_id: str | None = None) -> tuple[PlaceRef, ...]:
        """Load v1 place-tagged POIs through native map queries only."""

        map_ids = (map_id,) if map_id else self._map_ids()
        places: list[PlaceRef] = []
        for current_map_id in map_ids:
            binding = self._current_binding(current_map_id)
            response = self._maps.poi_list(current_map_id)
            if response.get("success") is not True:
                message = str(response.get("message") or f"native poi_list failed for {current_map_id}")
                raise PlaceCatalogError(message)
            pois = response.get("pois")
            if not isinstance(pois, Mapping):
                raise PlaceCatalogError(f"native poi_list returned invalid POIs for {current_map_id}")
            for name, raw_poi in pois.items():
                if not isinstance(raw_poi, Mapping):
                    continue
                place = self._place_from_poi(
                    name=str(name),
                    map_id=current_map_id,
                    raw_poi=raw_poi,
                    binding=binding,
                )
                if place is not None:
                    places.append(place)
        return tuple(sorted(places, key=_place_sort_key))

    list_places = load

    def resolve(
        self,
        query: str,
        *,
        map_id: str | None = None,
        building_id: str | None = None,
        floor_id: str | None = None,
        kind: str | None = None,
    ) -> PlaceResolution:
        """Resolve an exact normalized canonical name or alias."""

        normalized_query = _normalized_identifier(query)
        if not normalized_query:
            return PlaceResolution("not_found", query, None, ())
        building_filter = _normalized_identifier(building_id or "")
        floor_filter = normalize_floor_id(floor_id) if floor_id else ""
        kind_filter = _normalized_identifier(kind or "")
        places = self.load(map_id) if map_id else self.load()
        matches: tuple[PlaceRef, ...] = tuple(
            place
            for place in places
            if normalized_query
            in {_normalized_identifier(place.name), *(_normalized_identifier(alias) for alias in place.aliases)}
            and (not building_filter or _normalized_identifier(place.building_id) == building_filter)
            and (not floor_filter or normalize_floor_id(place.floor_id) == floor_filter)
            and (not kind_filter or _normalized_identifier(place.kind) == kind_filter)
        )
        if not matches:
            return PlaceResolution("not_found", query, None, ())
        if len(matches) > 1:
            return PlaceResolution("ambiguous", query, None, matches)
        place = matches[0]
        if not place.executable:
            return PlaceResolution("stale_map", query, None, matches)
        return PlaceResolution("resolved", query, place, matches)

    def _map_ids(self) -> tuple[str, ...]:
        response = self._maps.list_maps()
        if response.get("success") is not True:
            raise PlaceCatalogError(str(response.get("message") or "native list_maps failed"))
        raw_maps = response.get("maps")
        if not isinstance(raw_maps, Sequence) or isinstance(raw_maps, (str, bytes)):
            raise PlaceCatalogError("native list_maps returned an invalid map list")
        map_ids: set[str] = set()
        for entry in raw_maps:
            if isinstance(entry, str):
                map_id = entry
            elif isinstance(entry, Mapping):
                record = entry.get("record") if isinstance(entry.get("record"), Mapping) else {}
                map_id = str(record.get("map_id") or entry.get("map_id") or entry.get("name") or "")
            else:
                continue
            if map_id:
                map_ids.add(map_id)
        return tuple(sorted(map_ids))

    def _place_from_poi(
        self,
        *,
        name: str,
        map_id: str,
        raw_poi: Mapping[str, Any],
        binding: _MapBinding,
    ) -> PlaceRef | None:
        tags = raw_poi.get("tags")
        if not isinstance(tags, Mapping) or tags.get("schema_version") != PLACE_SCHEMA_VERSION:
            return None
        place_id = str(tags.get("place_id") or "").strip()
        if not place_id:
            return None

        aliases = _aliases_from_tag(tags.get("aliases"))
        stored_map_version = _optional_int(tags.get("map_version"))
        stored_version_id = str(tags.get("version_id") or "")
        stored_hash = str(tags.get("map_pcd_sha256") or "")
        frame_id = str(raw_poi.get("frame_id") or "")
        x = _optional_finite_float(raw_poi.get("x"))
        y = _optional_finite_float(raw_poi.get("y"))
        z = _optional_finite_float(raw_poi.get("z"))
        yaw = _optional_finite_float(raw_poi.get("yaw"))
        confidence = _optional_finite_float(tags.get("confidence"))

        missing: list[str] = []
        for label, value in (
            ("stored map_version", stored_map_version),
            ("stored version_id", stored_version_id),
            ("stored map_pcd_sha256", stored_hash),
            ("stored frame_id", frame_id),
            ("x", x),
            ("y", y),
            ("z", z),
            ("confidence", confidence),
        ):
            if value is None or value == "":
                missing.append(f"{label} is missing or invalid")
        if not binding.complete and binding.state.upper() not in {"FAILED", "RETIRED", "STALE"}:
            missing.extend(binding.errors)

        mismatches: list[str] = []
        if stored_map_version is not None and binding.map_version is not None:
            if stored_map_version != binding.map_version:
                mismatches.append("map_version does not match current map")
        if stored_version_id and binding.version_id and stored_version_id != binding.version_id:
            mismatches.append("version_id does not match current map")
        if stored_hash and binding.map_pcd_sha256:
            if stored_hash.casefold() != binding.map_pcd_sha256.casefold():
                mismatches.append("map_pcd_sha256 does not match current map")
        if frame_id and binding.frame_id and frame_id != binding.frame_id:
            mismatches.append("frame_id does not match current map")

        if missing:
            reason = "unbound"
        elif mismatches or binding.state.upper() in {"FAILED", "RETIRED", "STALE"}:
            reason = "stale_map"
        else:
            reason = None
        stored_binding = {
            "map_id": map_id,
            "map_version": stored_map_version,
            "version_id": stored_version_id,
            "map_pcd_sha256": stored_hash,
            "frame_id": frame_id,
        }
        return PlaceRef(
            place_id=place_id,
            name=name,
            aliases=aliases,
            kind=str(tags.get("kind") or ""),
            building_id=str(tags.get("building_id") or ""),
            floor_id=normalize_floor_id(str(tags.get("floor_id") or "")),
            map_id=map_id,
            map_version=stored_map_version,
            version_id=stored_version_id,
            map_pcd_sha256=stored_hash,
            frame_id=frame_id,
            x=x,
            y=y,
            z=z,
            yaw=yaw,
            connector_id=str(tags.get("connector_id") or ""),
            source=str(tags.get("source") or ""),
            confidence=confidence,
            executable=reason is None,
            non_executable_reason=reason,
            provenance={
                "schema_version": PLACE_SCHEMA_VERSION,
                "binding_status": "bound" if reason is None else reason,
                "stored_binding": stored_binding,
                "current_binding": _binding_dict(binding),
                "binding_errors": tuple(dict.fromkeys((*missing, *mismatches, *binding.errors))),
            },
        )

    def _current_binding(self, map_id: str) -> _MapBinding:
        errors: list[str] = []
        try:
            record_response = self._maps.get_record(map_id)
        except Exception as exc:
            record_response = {}
            errors.append(f"get_record failed: {exc}")
        try:
            points_response = self._maps.get_map_points(map_id, max_points=1)
        except Exception as exc:
            points_response = {}
            errors.append(f"get_map_points failed: {exc}")

        record = _record_from_response(record_response)
        record_ok = record_response.get("success") is True and bool(record)
        points_ok = points_response.get("success") is True
        if not record_ok:
            errors.append("native map record is unavailable")
        if not points_ok:
            errors.append("native map binding is unavailable")

        record_map_id = str(record.get("map_id") or "")
        points_map_id = str(points_response.get("map_id") or "")
        if record_map_id and record_map_id != map_id:
            errors.append("map record id does not match requested map")
        if points_map_id and points_map_id != map_id:
            errors.append("map point binding id does not match requested map")

        map_version = _optional_int(record.get("version"))
        version_id = str(points_response.get("version_id") or record.get("version_id") or "")
        map_hash = str(
            points_response.get("map_pcd_sha256") or record.get("map_pcd_sha256") or _pointcloud_hash(record) or ""
        )
        scope = record.get("scope") if isinstance(record.get("scope"), Mapping) else {}
        frame_id = str(points_response.get("frame_id") or record.get("frame_id") or scope.get("frame_id") or "")
        state = str(record.get("state") or "")
        for label, value in (
            ("map_version", map_version),
            ("version_id", version_id),
            ("map_pcd_sha256", map_hash),
            ("frame_id", frame_id),
        ):
            if value is None or value == "":
                errors.append(f"current {label} is missing")
        if state.upper() in {"FAILED", "RETIRED", "STALE"}:
            errors.append(f"current map state is {state.upper()}")
        return _MapBinding(
            map_id=map_id,
            map_version=map_version,
            version_id=version_id,
            map_pcd_sha256=map_hash,
            frame_id=frame_id,
            state=state,
            complete=not errors,
            errors=tuple(dict.fromkeys(errors)),
        )


_ARABIC_FLOOR = re.compile(r"^(?:floor[\s_-]*|第)?(?P<number>\d+)(?:[\s_-]*(?:楼|层|f))?$", re.IGNORECASE)
_BASEMENT_FLOOR = re.compile(r"^floor[\s_-]*b(?P<number>\d+)$", re.IGNORECASE)
_CHINESE_FLOOR = re.compile(r"^第?(?P<negative>负)?(?P<number>[零〇一二两三四五六七八九十百]+)(?:楼|层)$")


def normalize_floor_id(value: str | None) -> str:
    """Normalize explicit floor labels to the existing building-floor IDs."""

    text = unicodedata.normalize("NFKC", str(value or "")).strip()
    if not text:
        return ""
    basement = _BASEMENT_FLOOR.fullmatch(text)
    if basement:
        return _floor_id(-int(basement.group("number")))
    arabic = _ARABIC_FLOOR.fullmatch(text)
    if arabic:
        return _floor_id(int(arabic.group("number")))
    chinese = _CHINESE_FLOOR.fullmatch(text)
    if chinese:
        number = _chinese_integer(chinese.group("number"))
        if number is not None:
            if chinese.group("negative"):
                number = -number
            return _floor_id(number)
    return _normalized_identifier(text)


def _floor_id(number: int) -> str:
    return f"floor-{number}" if number >= 0 else f"floor-b{abs(number)}"


def _chinese_integer(value: str) -> int | None:
    digits = {
        "零": 0,
        "〇": 0,
        "一": 1,
        "二": 2,
        "两": 2,
        "三": 3,
        "四": 4,
        "五": 5,
        "六": 6,
        "七": 7,
        "八": 8,
        "九": 9,
    }
    if value == "十":
        return 10
    if "百" in value:
        left, _, right = value.partition("百")
        hundreds = digits.get(left, 1) if left else 1
        tail = _chinese_integer(right) if right else 0
        return None if hundreds is None or tail is None else hundreds * 100 + tail
    if "十" in value:
        left, _, right = value.partition("十")
        tens = digits.get(left, 1) if left else 1
        ones = digits.get(right, 0) if right else 0
        return None if tens is None or ones is None else tens * 10 + ones
    if len(value) == 1:
        return digits.get(value)
    return None


def _normalized_identifier(value: str) -> str:
    folded = unicodedata.normalize("NFKC", value).casefold()
    return "".join(character for character in folded if character.isalnum())


def _unique_text(values: Sequence[str]) -> tuple[str, ...]:
    seen: set[str] = set()
    result: list[str] = []
    for value in values:
        text = str(value or "").strip()
        if not text or text in seen:
            continue
        seen.add(text)
        result.append(text)
    return tuple(result)


def _aliases_from_tag(value: Any) -> tuple[str, ...]:
    if not isinstance(value, Sequence) or isinstance(value, (str, bytes)):
        return ()
    return _unique_text([str(item) for item in value])


def _record_from_response(response: Mapping[str, Any]) -> Mapping[str, Any]:
    record = response.get("record")
    if isinstance(record, Mapping):
        return record
    if response.get("map_id") and response.get("version_id"):
        return response
    return {}


def _pointcloud_hash(record: Mapping[str, Any]) -> str:
    artifacts = record.get("artifacts")
    if not isinstance(artifacts, Sequence) or isinstance(artifacts, (str, bytes)):
        return ""
    for artifact in artifacts:
        if not isinstance(artifact, Mapping):
            continue
        artifact_type = str(artifact.get("type") or "").upper()
        name = str(artifact.get("name") or "").lower()
        if artifact_type == "POINTCLOUD" or name in {"map_pcd", "source_pointcloud"}:
            return str(artifact.get("hash") or artifact.get("sha256") or "")
    return ""


def _optional_int(value: Any) -> int | None:
    if isinstance(value, bool) or value is None or value == "":
        return None
    try:
        return int(value)
    except (TypeError, ValueError):
        return None


def _finite_float(value: Any, *, name: str) -> float:
    if isinstance(value, bool):
        raise ValueError(f"{name} must be a finite number")
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name} must be a finite number") from exc
    if not math.isfinite(result):
        raise ValueError(f"{name} must be a finite number")
    return result


def _optional_finite_float(value: Any) -> float | None:
    if value is None or value == "" or isinstance(value, bool):
        return None
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result if math.isfinite(result) else None


def _mapping(value: Any, *, operation: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise TypeError(f"{operation} must return a mapping")
    return value


def _binding_dict(binding: _MapBinding) -> dict[str, Any]:
    return {
        "map_id": binding.map_id,
        "map_version": binding.map_version,
        "version_id": binding.version_id,
        "map_pcd_sha256": binding.map_pcd_sha256,
        "frame_id": binding.frame_id,
        "state": binding.state,
    }


def _place_sort_key(place: PlaceRef) -> tuple[str, str, str]:
    return (place.map_id, _normalized_identifier(place.name), place.place_id)
