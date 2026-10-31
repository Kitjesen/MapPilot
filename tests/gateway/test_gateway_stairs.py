from __future__ import annotations

import copy
import json

import pytest

pytestmark = [pytest.mark.sim]
pytest.importorskip("fastapi")


def _annotation():
    return {
        "name": "East stairs",
        "content_epoch": 7,
        "frame_id": "map",
        "bounds": {"min": {"x": 0, "y": 0, "z": 0}, "max": {"x": 4, "y": 2, "z": 3}},
        "lower": {"x": 0.5, "y": 1, "z": 0},
        "upper": {"x": 3.5, "y": 1, "z": 3},
    }


class NativeStairClient:
    def __init__(self):
        self.calls = []
        self.response = {
            "success": True,
            "map_id": "building",
            "content_epoch": 7,
            "stairs": [{"id": "east", **_annotation()}],
        }

    def service(self, action, **arguments):
        assert all(isinstance(value, (str, int, float, bool)) for value in arguments.values())
        self.calls.append((action, arguments))
        return copy.deepcopy(self.response)


@pytest.fixture
def client(monkeypatch):
    from fastapi.testclient import TestClient

    from gateway.gateway_module import GatewayModule

    monkeypatch.delenv("LINGTU_GATEWAY_REQUIRE_API_KEY", raising=False)
    gateway = GatewayModule()
    gateway.setup()
    native = NativeStairClient()
    gateway._map_client = native
    return TestClient(gateway._app), native


def test_put_stair_forwards_complete_geometry_in_one_native_request(client):
    http, native = client
    response = http.put("/api/v1/maps/building/stairs/east", json=_annotation())
    assert response.status_code == 200
    assert response.json() == native.response
    action, arguments = native.calls[-1]
    assert action == "set_stair"
    assert arguments["map_id"] == "building"
    assert arguments["stair_id"] == "east"
    assert json.loads(arguments["annotation_json"]) == _annotation()


def test_stair_list_preserves_old_binding_and_delete_requires_displayed_epoch(client):
    http, native = client
    native.response["content_epoch"] = 8
    listed = http.get("/api/v1/maps/building/stairs")
    assert listed.status_code == 200
    assert listed.json()["content_epoch"] == 8
    assert listed.json()["stairs"][0]["content_epoch"] == 7
    assert native.calls[-1] == ("list_stairs", {"map_id": "building"})
    assert http.delete("/api/v1/maps/building/stairs/east").status_code == 422
    deleted = http.delete("/api/v1/maps/building/stairs/east?content_epoch=8")
    assert deleted.status_code == 200
    assert native.calls[-1] == (
        "delete_stair", {"map_id": "building", "stair_id": "east", "content_epoch": 8}
    )


@pytest.mark.parametrize("reason,status", [
    ("content_epoch_conflict", 409), ("frame_id_conflict", 409),
    ("map_write_in_progress", 409), ("stair_not_found", 404), ("map_not_found", 404),
])
def test_stair_native_failure_keeps_reason(client, reason, status):
    http, native = client
    native.response = {"success": False, "reason_code": reason, "message": "native rejection"}
    response = http.put("/api/v1/maps/building/stairs/east", json=_annotation())
    assert response.status_code == status
    assert response.json()["reason_code"] == reason


@pytest.mark.parametrize("case", ["flat_bounds", "flat_entries", "missing_z", "boolean", "fractional_epoch"])
def test_stair_invalid_geometry_never_reaches_native(client, case):
    http, native = client
    body = _annotation()
    if case == "flat_bounds":
        body["bounds"]["max"]["z"] = 0
    elif case == "flat_entries":
        body["upper"]["z"] = 0
    elif case == "missing_z":
        del body["lower"]["z"]
    elif case == "boolean":
        body["lower"]["x"] = True
    else:
        body["content_epoch"] = 7.5
    response = http.put("/api/v1/maps/building/stairs/east", json=body)
    assert response.status_code == 422
    assert native.calls == []


def test_native_recognition_is_exposed_but_not_accepted_from_browser(client):
    http, native = client
    geometry = {
        "recognized": True, "reason": "recognized",
        "origin": {"x": 0, "y": 0, "z": 0},
        "lower": {"x": 0.5, "y": 1, "z": 0},
        "upper": {"x": 3.5, "y": 1, "z": 3},
        "axis_x": 1, "axis_y": 0, "width_m": 1.0,
        "mean_rise_m": 0.15, "mean_run_m": 0.3,
        "surfaces": [{"s_min": 0, "s_max": 1, "lateral_min": -0.5,
                      "lateral_max": 0.5, "surface_z": 0, "observed_points": 45}],
    }
    native.response["stairs"][0]["geometry"] = geometry
    response = http.get("/api/v1/maps/building/stairs")
    assert response.status_code == 200
    assert response.json()["stairs"][0]["geometry"] == geometry
    calls = len(native.calls)
    response = http.put("/api/v1/maps/building/stairs/east", json={**_annotation(), "geometry": geometry})
    assert response.status_code == 422
    assert len(native.calls) == calls
