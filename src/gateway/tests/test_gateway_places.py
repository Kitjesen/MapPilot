from __future__ import annotations

import pytest

pytestmark = [pytest.mark.sim]
pytest.importorskip("fastapi")


class FakePlaceMapsService:
    def __init__(self) -> None:
        self.records = {
            "floor-6-map": {
                "map_id": "floor-6-map",
                "version": 7,
                "version_id": "version-7",
                "state": "READY",
                "frame_id": "map",
                "artifacts": [{"type": "POINTCLOUD", "hash": "hash-7"}],
            },
            "floor-5-map": {
                "map_id": "floor-5-map",
                "version": 5,
                "version_id": "version-5",
                "state": "READY",
                "frame_id": "map",
                "artifacts": [{"type": "POINTCLOUD", "hash": "hash-5"}],
            },
        }
        self.points = {
            "floor-6-map": {
                "success": True,
                "map_id": "floor-6-map",
                "version_id": "version-7",
                "map_pcd_sha256": "hash-7",
                "frame_id": "map",
                "points": [[0.0, 0.0, 0.0]],
            },
            "floor-5-map": {
                "success": True,
                "map_id": "floor-5-map",
                "version_id": "version-5",
                "map_pcd_sha256": "hash-5",
                "frame_id": "map",
                "points": [[0.0, 0.0, 0.0]],
            },
        }
        self.pois: dict[str, dict[str, dict]] = {map_id: {} for map_id in self.records}
        self.last_command: dict | None = None

    def list_maps(self):
        return {
            "success": True,
            "maps": [{"map_id": map_id} for map_id in sorted(self.records)],
        }

    def get_record(self, map_id: str):
        record = self.records.get(map_id)
        if record is None:
            return {"success": False, "message": "missing map"}
        return {"success": True, "record": dict(record)}

    def get_map_points(self, map_id: str, *, max_points: int = 0):
        return dict(self.points.get(map_id, {"success": False, "message": "missing points"}))

    def poi_list(self, map_id: str = ""):
        if map_id:
            return {"success": True, "pois": self.pois.setdefault(map_id, {})}
        merged = {}
        for pois in self.pois.values():
            merged.update(pois)
        return {"success": True, "pois": merged}

    def poi_set(self, command):
        self.last_command = dict(command)
        self.pois.setdefault(command["map_id"], {})[command["name"]] = {
            "x": command["x"],
            "y": command["y"],
            "z": command["z"],
            "yaw": command.get("yaw"),
            "frame_id": command["frame_id"],
            "tags": dict(command["tags"]),
        }
        return {"success": True}


def _gateway_with_maps(monkeypatch):
    from gateway.gateway_module import GatewayModule

    monkeypatch.delenv("LINGTU_GATEWAY_REQUIRE_API_KEY", raising=False)
    monkeypatch.delenv("LINGTU_ENDPOINT", raising=False)
    gateway = GatewayModule()
    gateway.setup()
    maps = FakePlaceMapsService()
    gateway.on_system_modules({"maps.service": maps})
    return gateway, maps


def _place_payload(**updates):
    payload = {
        "place_id": "poi-company-6f",
        "name": "某公司",
        "map_id": "floor-6-map",
        "x": 1.25,
        "y": 2.5,
        "z": 0.0,
        "source": "operator",
        "aliases": ["6楼某公司", "六层某公司"],
        "kind": "company",
        "building_id": "main",
        "floor_id": "6楼",
        "yaw": 1.57,
        "confidence": 0.95,
    }
    payload.update(updates)
    return payload


def test_places_post_binds_version_from_native_maps(monkeypatch):
    from fastapi.testclient import TestClient

    gateway, maps = _gateway_with_maps(monkeypatch)
    response = TestClient(gateway._app).post("/api/v1/places", json=_place_payload())

    assert response.status_code == 200
    payload = response.json()
    place = payload["place"]
    assert place["executable"] is True
    assert place["map_version"] == 7
    assert place["version_id"] == "version-7"
    assert place["map_pcd_sha256"] == "hash-7"
    assert place["frame_id"] == "map"
    assert place["floor_id"] == "floor-6"
    assert place["binding"]["status"] == "bound"
    assert maps.last_command is not None
    assert maps.last_command["tags"]["version_id"] == "version-7"


def test_places_post_rejects_caller_supplied_map_binding_fields(monkeypatch):
    from fastapi.testclient import TestClient

    gateway, _maps = _gateway_with_maps(monkeypatch)
    response = TestClient(gateway._app).post(
        "/api/v1/places",
        json=_place_payload(map_version=999, version_id="fake", map_pcd_sha256="fake"),
    )

    assert response.status_code == 422


@pytest.mark.parametrize("field", ["x", "y", "z", "yaw", "confidence"])
def test_places_post_rejects_boolean_numeric_fields(monkeypatch, field: str):
    from fastapi.testclient import TestClient

    gateway, _maps = _gateway_with_maps(monkeypatch)
    response = TestClient(gateway._app).post(
        "/api/v1/places",
        json=_place_payload(**{field: True}),
    )

    assert response.status_code == 422


@pytest.mark.parametrize("invalid", [[], {}])
def test_places_post_rejects_structured_numeric_fields(monkeypatch, invalid):
    from fastapi.testclient import TestClient

    gateway, _maps = _gateway_with_maps(monkeypatch)
    response = TestClient(gateway._app, raise_server_exceptions=False).post(
        "/api/v1/places",
        json=_place_payload(x=invalid),
    )

    assert response.status_code == 422


def test_places_post_reports_identity_conflict(monkeypatch):
    from fastapi.testclient import TestClient

    gateway, _maps = _gateway_with_maps(monkeypatch)
    client = TestClient(gateway._app)

    assert client.post("/api/v1/places", json=_place_payload()).status_code == 200
    response = client.post(
        "/api/v1/places",
        json=_place_payload(place_id="different-id", aliases=[]),
    )

    assert response.status_code == 409
    assert "another identity" in response.json()["detail"]


def test_places_routes_return_503_without_maps_service(monkeypatch):
    from fastapi.testclient import TestClient

    from gateway.gateway_module import GatewayModule

    monkeypatch.delenv("LINGTU_GATEWAY_REQUIRE_API_KEY", raising=False)
    monkeypatch.delenv("LINGTU_ENDPOINT", raising=False)
    gateway = GatewayModule()
    gateway.setup()

    response = TestClient(gateway._app).get("/api/v1/places")

    assert response.status_code == 503
    assert "maps.service is unavailable" in response.json()["detail"]


def test_places_list_and_resolve_ambiguity(monkeypatch):
    from fastapi.testclient import TestClient

    gateway, _maps = _gateway_with_maps(monkeypatch)
    client = TestClient(gateway._app)

    assert client.post("/api/v1/places", json=_place_payload()).status_code == 200
    assert (
        client.post(
            "/api/v1/places",
            json=_place_payload(
                place_id="poi-company-5f",
                map_id="floor-5-map",
                aliases=["五层某公司"],
                floor_id="5楼",
                x=3.0,
                y=4.0,
            ),
        ).status_code
        == 200
    )

    listed = client.get("/api/v1/places", params={"map_id": "floor-6-map"}).json()
    assert listed["count"] == 1
    assert listed["places"][0]["map_id"] == "floor-6-map"

    ambiguous = client.get("/api/v1/places/resolve", params={"q": "某公司"}).json()
    assert ambiguous["ok"] is False
    assert ambiguous["status"] == "ambiguous"
    assert len(ambiguous["candidates"]) == 2

    resolved = client.get(
        "/api/v1/places/resolve",
        params={"q": "某公司", "map": "floor-6-map", "floor": "六层"},
    ).json()
    assert resolved["ok"] is True
    assert resolved["status"] == "resolved"
    assert resolved["place"]["place_id"] == "poi-company-6f"


def test_places_resolve_reports_stale_as_non_executable(monkeypatch):
    from fastapi.testclient import TestClient

    gateway, maps = _gateway_with_maps(monkeypatch)
    client = TestClient(gateway._app)

    assert client.post("/api/v1/places", json=_place_payload()).status_code == 200
    maps.records["floor-6-map"]["version"] = 8
    maps.records["floor-6-map"]["version_id"] = "version-8"
    maps.records["floor-6-map"]["artifacts"] = [{"type": "POINTCLOUD", "hash": "hash-8"}]
    maps.points["floor-6-map"]["version_id"] = "version-8"
    maps.points["floor-6-map"]["map_pcd_sha256"] = "hash-8"

    response = client.get("/api/v1/places/resolve", params={"q": "六层某公司"})

    assert response.status_code == 200
    payload = response.json()
    assert payload["ok"] is False
    assert payload["status"] == "stale_map"
    assert payload["place"] is None
    assert payload["candidates"][0]["executable"] is False
    assert payload["candidates"][0]["reason"] == "stale_map"
