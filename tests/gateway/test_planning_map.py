"""A static planning layer must remain bound to live native map identity."""

from copy import deepcopy
from types import SimpleNamespace

import pytest

from gateway.navigation.planning_map import planning_map_view
from gateway.services.native_status import read_planning_map


@pytest.fixture
def evidence():
    return [
        {"schema_version": 1, "available": True, "reason": "ready", "frame_id": "map",
         "product_session_id": "run-a", "map_id": "hall", "map_content_epoch": 2,
         "stamp_s": 1, "resolution": .2, "rows": 1, "cols": 3,
         "origin": [-1, -2, .1], "reference_z": .12, "cells": [0, 1, 2]},
        {"stamp_s": 100, "native_product": {"product_session_id": "run-a"}},
        {"ts": 100, "map_id": "hall", "map_content_epoch": 2},
        {"product_session_id": "run-a", "active_map": "hall"},
    ]


def test_static_layer_does_not_expire_with_fresh_matching_native_evidence(evidence):
    assert planning_map_view(*evidence, now_s=100) == evidence[0]


@pytest.mark.parametrize(("index", "key", "value", "reason"), [
    (1, "stamp_s", 97, "navigation_status_stale"),
    (2, "ts", 97, "navigation_status_stale"),
    (0, "product_session_id", "old-run", "planning_map_session_mismatch"),
    (2, "map_content_epoch", 3, "planning_map_identity_mismatch"),
    (3, "active_map", "another-map", "planning_map_identity_mismatch"),
    (0, "cells", [0, 1], "planning_map_invalid"),
    (0, "cells", [0, 1, 9], "planning_map_invalid"),
    (0, "origin", [0, 0, float("nan")], "planning_map_invalid"),
])
def test_stale_changed_or_incomplete_map_is_not_rendered(evidence, index, key, value, reason):
    evidence = deepcopy(evidence)
    evidence[index][key] = value
    view = planning_map_view(*evidence, now_s=100)
    assert view["available"] is False
    assert view["reason"] == reason
    assert view["cells"] == []


def test_missing_projection_is_pending(evidence):
    evidence[0] = None
    assert planning_map_view(*evidence, now_s=100)["reason"] == "planning_map_pending"


def test_projection_sidecar_follows_navigation_status_path(monkeypatch, tmp_path):
    path = tmp_path / "nav.json"
    monkeypatch.setenv("LINGTU_NAV_STATUS_FILE", str(path))
    path.with_name("nav.json.planning-map.json").write_text('{"available": false}')
    assert read_planning_map() == {"available": False}


def test_read_only_route_returns_native_cells_without_navigation_commands(evidence, monkeypatch):
    from fastapi import FastAPI
    from fastapi.testclient import TestClient

    from gateway.navigation import planning_map
    from gateway.navigation.routes import register_navigation_routes

    monkeypatch.setattr(planning_map, "read_planning_map", lambda: evidence[0])
    monkeypatch.setattr(planning_map, "read_navigation_status", lambda: evidence[1])
    monkeypatch.setattr(planning_map, "status_is_fresh", lambda value, **_: value.get("stamp_s") == 100)
    gw = SimpleNamespace(_navigation_state=evidence[2], _session_snapshot=lambda: evidence[3])
    app = FastAPI()
    register_navigation_routes(app, gw)
    with TestClient(app) as client:
        response = client.get("/api/v1/navigation/planning_map")
    assert response.status_code == 200
    assert response.json() == evidence[0]
