from __future__ import annotations

import asyncio
from types import SimpleNamespace

import pytest

pytest.importorskip("fastapi")


def _endpoint(gateway, path: str, method: str):
    for route in gateway._app.routes:
        if route.path == path and method.upper() in getattr(route, "methods", set()):
            return route.endpoint
    raise AssertionError(f"route not found: {method} {path}")


def _gateway():
    from gateway.gateway_module import GatewayModule
    from memory.spatial.tagged_locations import TaggedLocationStore

    gateway = GatewayModule()
    gateway.setup()
    gateway._tagged_loc_module = SimpleNamespace(
        store=TaggedLocationStore(),
        tag_status=SimpleNamespace(publish=lambda _message: None),
    )
    return gateway


def test_location_normalizer_exposes_map_binding_fields():
    from gateway.schemas import LocationEntry
    from gateway.services.telemetry_normalizers import build_locations_response

    payload = build_locations_response(
        [
            {
                "name": "inspection-dock",
                "position": [1.25, -2.5, 0.1],
                "metadata": {
                    "map_id": "yard",
                    "map_content_epoch": 7,
                    "frame_id": "map",
                    "binding_status": "bound",
                    "checkpoint_kind": "thermal",
                },
            }
        ]
    )

    entry = LocationEntry.model_validate(payload["locations"][0])
    assert entry.map_id == "yard"
    assert entry.map_content_epoch == 7
    assert entry.frame_id == "map"
    assert entry.metadata == {
        "map_id": "yard",
        "map_content_epoch": 7,
        "frame_id": "map",
        "binding_status": "bound",
        "checkpoint_kind": "thermal",
    }


def test_location_create_binds_active_map_and_rejects_caller_spoof(monkeypatch):
    import gateway.routes.status as status
    from gateway.schemas import LocationOperationResponse, LocationUpsertRequest

    gateway = _gateway()
    monkeypatch.setattr(status, "_active_map_from_service", lambda _gw: "yard")
    monkeypatch.setattr(
        status,
        "mapd_query",
        lambda _gw, request: {
            "success": True,
            "record": {"name": request["map_id"], "content_epoch": 7},
        },
    )
    body = LocationUpsertRequest(
        name="inspection-dock",
        x=1.0,
        y=2.0,
        metadata={
            "checkpoint_kind": "thermal",
            "map_id": "spoofed-map",
            "map_content_epoch": 999,
            "frame_id": "odom",
            "binding_status": "bound",
        },
    )

    payload = asyncio.run(_endpoint(gateway, "/api/v1/locations", "POST")(body))
    saved = LocationOperationResponse.model_validate(payload)
    stored = gateway._tagged_loc_module.store.query("inspection-dock")

    assert saved.ok is True
    assert saved.location is not None
    assert saved.location.map_id == "yard"
    assert saved.location.map_content_epoch == 7
    assert saved.location.frame_id == "map"
    assert stored["metadata"] == {
        "checkpoint_kind": "thermal",
        "map_id": "yard",
        "map_content_epoch": 7,
        "frame_id": "map",
        "binding_status": "bound",
    }


def test_location_update_preserves_non_binding_metadata(monkeypatch):
    import gateway.routes.status as status
    from gateway.schemas import LocationUpsertRequest

    gateway = _gateway()
    gateway._tagged_loc_module.store.tag(
        "inspection-dock",
        x=0.0,
        y=0.0,
        metadata={
            "floor": "1f",
            "note": "old",
            "map_id": "old-map",
            "map_content_epoch": 1,
            "frame_id": "map",
            "binding_status": "bound",
        },
    )
    monkeypatch.setattr(status, "_active_map_from_service", lambda _gw: "yard")
    monkeypatch.setattr(
        status,
        "mapd_query",
        lambda _gw, _request: {
            "success": True,
            "record": {"name": "yard", "content_epoch": 8},
        },
    )
    body = LocationUpsertRequest(
        name="inspection-dock",
        x=3.0,
        y=4.0,
        metadata={"note": "updated"},
    )

    asyncio.run(_endpoint(gateway, "/api/v1/locations/{name}", "PUT")("inspection-dock", body))
    stored = gateway._tagged_loc_module.store.query("inspection-dock")

    assert stored["metadata"] == {
        "floor": "1f",
        "note": "updated",
        "map_id": "yard",
        "map_content_epoch": 8,
        "frame_id": "map",
        "binding_status": "bound",
    }


def test_location_without_active_map_is_saved_unbound(monkeypatch):
    import gateway.routes.status as status
    from gateway.schemas import LocationOperationResponse, LocationUpsertRequest

    gateway = _gateway()
    monkeypatch.setattr(status, "_active_map_from_service", lambda _gw: "")
    monkeypatch.setattr(
        status,
        "mapd_query",
        lambda *_args, **_kwargs: pytest.fail("map record must not be queried without an active map"),
    )
    body = LocationUpsertRequest(
        name="unbound-checkpoint",
        x=1.0,
        y=2.0,
        metadata={
            "inspection_kind": "visual",
            "map_id": "spoofed-map",
            "map_content_epoch": 999,
            "binding_status": "bound",
        },
    )

    payload = asyncio.run(_endpoint(gateway, "/api/v1/locations", "POST")(body))
    saved = LocationOperationResponse.model_validate(payload)
    metadata = gateway._tagged_loc_module.store.query("unbound-checkpoint")["metadata"]

    assert saved.ok is True
    assert saved.location is not None
    assert saved.location.map_id is None
    assert saved.location.map_content_epoch is None
    assert saved.location.frame_id == "map"
    assert metadata == {
        "inspection_kind": "visual",
        "frame_id": "map",
        "binding_status": "unbound",
    }


def test_location_map_query_failure_does_not_invent_content_epoch(monkeypatch):
    import gateway.routes.status as status
    from gateway.schemas import LocationUpsertRequest

    gateway = _gateway()
    monkeypatch.setattr(status, "_active_map_from_service", lambda _gw: "yard")

    def fail_query(_gw, _request):
        raise RuntimeError("maps service offline")

    monkeypatch.setattr(status, "mapd_query", fail_query)
    body = LocationUpsertRequest(
        name="versionless-checkpoint",
        x=1.0,
        y=2.0,
        metadata={"map_content_epoch": 999},
    )

    asyncio.run(_endpoint(gateway, "/api/v1/locations", "POST")(body))
    metadata = gateway._tagged_loc_module.store.query("versionless-checkpoint")["metadata"]

    assert metadata["map_id"] == "yard"
    assert metadata["frame_id"] == "map"
    assert metadata["binding_status"] == "content_epoch_unavailable"
    assert "map_content_epoch" not in metadata


def test_location_binding_retries_when_active_map_changes_mid_request(monkeypatch):
    import gateway.routes.status as status
    from gateway.schemas import LocationUpsertRequest

    gateway = _gateway()
    active_names = iter(("old-map", "new-map", "new-map", "new-map"))
    monkeypatch.setattr(status, "_active_map_from_service", lambda _gw: next(active_names))
    monkeypatch.setattr(
        status,
        "mapd_query",
        lambda _gw, request: {
            "success": True,
            "record": {
                "name": request["map_id"],
                "content_epoch": 1 if request["map_id"] == "old-map" else 2,
            },
        },
    )
    body = LocationUpsertRequest(name="switch-safe", x=1.0, y=2.0)

    asyncio.run(_endpoint(gateway, "/api/v1/locations", "POST")(body))
    metadata = gateway._tagged_loc_module.store.query("switch-safe")["metadata"]

    assert metadata["map_id"] == "new-map"
    assert metadata["map_content_epoch"] == 2
    assert metadata["binding_status"] == "bound"
