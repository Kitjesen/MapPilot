import json

import pytest
from fastapi import FastAPI
from fastapi.testclient import TestClient

from gateway.routes.app import register_app_routes
from gateway.services.geography import geographic_location


@pytest.fixture
def snapshot(tmp_path, monkeypatch):
    path = tmp_path / "gnss.json"
    monkeypatch.setenv("LINGTU_GNSS_STATUS_FILE", str(path))
    monkeypatch.setattr("gateway.services.geography.time.time", lambda: 100)
    return path


def test_missing_no_fix_and_stale_never_return_coordinates(snapshot):
    assert geographic_location()["state"] == "unavailable"
    snapshot.write_text(json.dumps({"location": None}))
    assert geographic_location()["state"] == "no_fix"
    snapshot.write_text(json.dumps({"location": {"lat": 30, "lon": 120, "fix_type": 4, "ts": 96}}))
    assert geographic_location()["state"] == "stale"
    assert geographic_location()["location"] is None


@pytest.mark.parametrize("change", [{"lat": 91}, {"lon": float("nan")}, {"ts": 102}, {"ts": True}, {"fix_type": 0}])
def test_invalid_fix_not_exposed(snapshot, change):
    snapshot.write_text(json.dumps({"location": {"lat": 30, "lon": 120, "fix_type": 4, "ts": 99, **change}}))
    assert geographic_location()["location"] is None


def test_fresh_snapshot_route_has_no_cache_and_no_invented_accuracy(snapshot):
    snapshot.write_text(json.dumps({"location": {"lat": 30, "lon": 120, "fix_type": 4, "ts": 99, "accuracy_m": None}}))
    app = FastAPI()
    register_app_routes(app, object())
    with TestClient(app) as client:
        response = client.get("/api/v1/geography/location")
    assert response.headers["cache-control"] == "no-store"
    assert response.json() == {"state": "ready", "datum": "WGS84", "age_s": 1,
        "location": {"latitude": 30, "longitude": 120, "fix_type": 4, "timestamp": 99, "accuracy_m": None}}


@pytest.mark.parametrize("source", ['{', '[]', '{"location": "invalid"}'])
def test_broken_snapshot(snapshot, source):
    snapshot.write_text(source)
    assert geographic_location()["state"] == "invalid"
