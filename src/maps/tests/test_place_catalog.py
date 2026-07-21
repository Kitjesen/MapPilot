"""Behavior tests for the native-backed place catalog projection."""

# ruff: noqa: S101

from __future__ import annotations

import builtins
from copy import deepcopy
from typing import Any

import pytest

from maps.places import (
    PlaceCatalog,
    PlaceCatalogError,
    PlaceRegistration,
    normalize_floor_id,
)


class InMemoryMapsService:
    """Pure in-memory implementation of the PlaceCatalog maps protocol."""

    def __init__(self) -> None:
        self.records: dict[str, dict[str, Any]] = {
            "campus_6f": {
                "map_id": "campus_6f",
                "version": 3,
                "version_id": "campus_6f:v3",
                "state": "READY",
                "scope": {"frame_id": "map"},
                "artifacts": [
                    {
                        "type": "POINTCLOUD",
                        "name": "map_pcd",
                        "hash": "sha-current",
                    }
                ],
            }
        }
        self.bindings: dict[str, dict[str, Any]] = {
            "campus_6f": {
                "map_id": "campus_6f",
                "version_id": "campus_6f:v3",
                "map_pcd_sha256": "sha-current",
                "frame_id": "map",
            }
        }
        self.pois: dict[str, dict[str, dict[str, Any]]] = {"campus_6f": {}}
        self.calls: list[tuple[str, Any]] = []

    def list_maps(self) -> dict[str, Any]:
        self.calls.append(("list_maps", None))
        return {
            "success": True,
            "maps": [{"name": map_id, "record": deepcopy(record)} for map_id, record in self.records.items()],
        }

    def get_record(self, map_id: str) -> dict[str, Any]:
        self.calls.append(("get_record", map_id))
        return {"success": True, "record": deepcopy(self.records[map_id])}

    def get_map_points(self, map_id: str, *, max_points: int = 0) -> dict[str, Any]:
        self.calls.append(("get_map_points", (map_id, max_points)))
        return {"success": True, **deepcopy(self.bindings[map_id]), "points": []}

    def poi_list(self, map_id: str = "") -> dict[str, Any]:
        self.calls.append(("poi_list", map_id))
        return {
            "success": True,
            "map_id": map_id,
            "pois": deepcopy(self.pois.get(map_id, {})),
        }

    def poi_set(self, command: dict[str, Any]) -> dict[str, Any]:
        stored = deepcopy(command)
        self.calls.append(("poi_set", stored))
        poi = {
            "frame_id": stored["frame_id"],
            "x": stored["x"],
            "y": stored["y"],
            "z": stored["z"],
            "yaw": stored["yaw"],
            "tags": deepcopy(stored["tags"]),
        }
        self.pois.setdefault(stored["map_id"], {})[stored["name"]] = poi
        return {
            "success": True,
            "map_id": stored["map_id"],
            "name": stored["name"],
            "poi": deepcopy(poi),
        }


def test_upsert_persists_stable_identity_and_current_map_binding_tags() -> None:
    native = InMemoryMapsService()
    catalog = PlaceCatalog(native)

    place = catalog.upsert(
        PlaceRegistration(
            place_id="place-dock-a",
            name="Charging Dock A",
            aliases=("Dock A", "A号充电桩"),
            kind="charger",
            building_id="hq",
            floor_id="6楼",
            map_id="campus_6f",
            x=1.25,
            y=-2.5,
            z=0.15,
            yaw=1.57,
            connector_id="lift-east",
            source="operator_survey",
            confidence=0.95,
        )
    )

    command = next(payload for name, payload in native.calls if name == "poi_set")
    assert command["name"] == "Charging Dock A"
    assert (command["x"], command["y"], command["z"], command["yaw"]) == (
        1.25,
        -2.5,
        0.15,
        1.57,
    )
    assert command["tags"] == {
        "schema_version": "lingtu.place.v1",
        "place_id": "place-dock-a",
        "aliases": ["Dock A", "A号充电桩"],
        "kind": "charger",
        "building_id": "hq",
        "floor_id": "floor-6",
        "connector_id": "lift-east",
        "map_version": 3,
        "version_id": "campus_6f:v3",
        "map_pcd_sha256": "sha-current",
        "source": "operator_survey",
        "confidence": 0.95,
    }
    assert place.place_id == "place-dock-a"
    assert place.executable is True
    assert place.non_executable_reason is None


def test_resolve_matches_an_exact_alias_after_case_spacing_and_punctuation_normalization() -> None:
    native = InMemoryMapsService()
    catalog = PlaceCatalog(native)
    catalog.upsert(
        PlaceRegistration(
            place_id="place-dock-a",
            name="Charging Dock A",
            aliases=("East Dock",),
            kind="charger",
            building_id="hq",
            floor_id="floor-6",
            map_id="campus_6f",
            x=1.0,
            y=2.0,
            z=0.0,
            source="operator_survey",
        )
    )

    result = catalog.resolve("  EAST-dock!!! ")

    assert result.status == "resolved"
    assert result.place is not None
    assert result.place.place_id == "place-dock-a"
    assert result.candidates == (result.place,)


@pytest.mark.parametrize("label", ["6楼", "六楼", "六层", "第六层"])
def test_chinese_floor_labels_normalize_to_a_stable_floor_id(label: str) -> None:
    assert normalize_floor_id(label) == "floor-6"


def test_duplicate_exact_aliases_are_ambiguous_until_explicit_filters_select_one() -> None:
    native = InMemoryMapsService()
    catalog = PlaceCatalog(native)
    catalog.upsert(
        PlaceRegistration(
            place_id="place-charger",
            name="Charger Bay",
            aliases=("Service Point",),
            kind="charger",
            building_id="HQ",
            floor_id="6楼",
            map_id="campus_6f",
            x=1.0,
            y=2.0,
            z=0.0,
            source="operator_survey",
        )
    )
    catalog.upsert(
        PlaceRegistration(
            place_id="place-lift",
            name="Lift Lobby",
            aliases=("Service Point",),
            kind="elevator",
            building_id="Annex",
            floor_id="2楼",
            map_id="campus_6f",
            x=5.0,
            y=6.0,
            z=0.0,
            source="operator_survey",
        )
    )

    ambiguous = catalog.resolve("service point")

    assert ambiguous.status == "ambiguous"
    assert ambiguous.place is None
    assert [candidate.place_id for candidate in ambiguous.candidates] == [
        "place-charger",
        "place-lift",
    ]

    filtered = catalog.resolve(
        "service point",
        building_id=" h-q ",
        floor_id="第六层",
        kind="CHARGER",
    )

    assert filtered.status == "resolved"
    assert filtered.place is not None
    assert filtered.place.place_id == "place-charger"


@pytest.mark.parametrize(
    "second",
    [
        PlaceRegistration(
            place_id="place-other",
            name="Dock A",
            map_id="campus_6f",
            x=3.0,
            y=4.0,
            z=0.0,
            source="operator_survey",
        ),
        PlaceRegistration(
            place_id="place-dock-a",
            name="Renamed Dock",
            map_id="campus_6f",
            x=3.0,
            y=4.0,
            z=0.0,
            source="operator_survey",
        ),
    ],
)
def test_upsert_rejects_name_or_stable_id_rebinding(second: PlaceRegistration) -> None:
    native = InMemoryMapsService()
    catalog = PlaceCatalog(native)
    catalog.upsert(
        PlaceRegistration(
            place_id="place-dock-a",
            name="Dock A",
            map_id="campus_6f",
            x=1.0,
            y=2.0,
            z=0.0,
            source="operator_survey",
        )
    )

    with pytest.raises(PlaceCatalogError):
        catalog.upsert(second)

    writes = [payload for name, payload in native.calls if name == "poi_set"]
    assert len(writes) == 1


def test_upsert_rejects_rebinding_a_stable_id_to_another_map() -> None:
    native = InMemoryMapsService()
    native.records["annex_2f"] = {
        "map_id": "annex_2f",
        "version": 1,
        "version_id": "annex_2f:v1",
        "state": "READY",
        "scope": {"frame_id": "map"},
        "artifacts": [
            {
                "type": "POINTCLOUD",
                "name": "map_pcd",
                "hash": "sha-annex",
            }
        ],
    }
    native.bindings["annex_2f"] = {
        "map_id": "annex_2f",
        "version_id": "annex_2f:v1",
        "map_pcd_sha256": "sha-annex",
        "frame_id": "map",
    }
    native.pois["annex_2f"] = {}
    catalog = PlaceCatalog(native)
    catalog.upsert(
        PlaceRegistration(
            place_id="place-dock-a",
            name="Dock A",
            map_id="campus_6f",
            x=1.0,
            y=2.0,
            z=0.0,
            source="operator_survey",
        )
    )

    with pytest.raises(PlaceCatalogError, match="already bound"):
        catalog.upsert(
            PlaceRegistration(
                place_id="place-dock-a",
                name="Dock A",
                map_id="annex_2f",
                x=3.0,
                y=4.0,
                z=0.0,
                source="operator_survey",
            )
        )

    writes = [payload for name, payload in native.calls if name == "poi_set"]
    assert len(writes) == 1


@pytest.mark.parametrize("changed_binding", ["version", "hash"])
def test_stale_map_version_or_hash_is_never_resolved(changed_binding: str) -> None:
    native = InMemoryMapsService()
    catalog = PlaceCatalog(native)
    catalog.upsert(
        PlaceRegistration(
            place_id="place-dock-a",
            name="Dock A",
            map_id="campus_6f",
            x=1.0,
            y=2.0,
            z=0.0,
            source="operator_survey",
        )
    )
    if changed_binding == "version":
        native.records["campus_6f"]["version"] = 4
        native.records["campus_6f"]["version_id"] = "campus_6f:v4"
        native.bindings["campus_6f"]["version_id"] = "campus_6f:v4"
    else:
        native.records["campus_6f"]["artifacts"][0]["hash"] = "sha-next"
        native.bindings["campus_6f"]["map_pcd_sha256"] = "sha-next"

    result = catalog.resolve("Dock A")

    assert result.status == "stale_map"
    assert result.place is None
    assert len(result.candidates) == 1
    assert result.candidates[0].executable is False
    assert result.candidates[0].non_executable_reason == "stale_map"


def test_missing_saved_map_binding_is_reported_as_unbound() -> None:
    native = InMemoryMapsService()
    catalog = PlaceCatalog(native)
    catalog.upsert(
        PlaceRegistration(
            place_id="place-dock-a",
            name="Dock A",
            map_id="campus_6f",
            x=1.0,
            y=2.0,
            z=0.0,
            source="operator_survey",
        )
    )
    del native.pois["campus_6f"]["Dock A"]["tags"]["version_id"]

    place = catalog.load("campus_6f")[0]

    assert place.executable is False
    assert place.non_executable_reason == "unbound"
    assert place.provenance["binding_status"] == "unbound"
    assert catalog.resolve("Dock A").status == "stale_map"


def test_registration_requires_caller_coordinates_before_any_native_write() -> None:
    native = InMemoryMapsService()

    with pytest.raises(TypeError):
        PlaceRegistration(  # type: ignore[call-arg]
            place_id="place-no-z",
            name="No Coordinates",
            map_id="campus_6f",
            x=1.0,
            y=2.0,
            source="operator_survey",
        )

    assert all(name != "poi_set" for name, _payload in native.calls)


def test_catalog_queries_only_the_native_seam_without_opening_files(monkeypatch: pytest.MonkeyPatch) -> None:
    native = InMemoryMapsService()
    catalog = PlaceCatalog(native)
    catalog.upsert(
        PlaceRegistration(
            place_id="place-dock-a",
            name="Dock A",
            map_id="campus_6f",
            x=1.0,
            y=2.0,
            z=0.0,
            source="operator_survey",
        )
    )
    native.calls.clear()

    def reject_file_access(*_args: Any, **_kwargs: Any) -> None:
        raise AssertionError("PlaceCatalog must not access the filesystem")

    monkeypatch.setattr(builtins, "open", reject_file_access)
    result = catalog.resolve("Dock A")

    assert result.status == "resolved"
    assert [name for name, _payload in native.calls] == [
        "list_maps",
        "get_record",
        "get_map_points",
        "poi_list",
    ]
