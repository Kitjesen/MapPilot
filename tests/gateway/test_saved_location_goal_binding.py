"""Saved-location identity must survive preview and final goal submission."""

from __future__ import annotations

from types import SimpleNamespace

import pytest

pytest.importorskip("fastapi")
from fastapi import FastAPI
from fastapi.testclient import TestClient

from gateway.gateway_module import GatewayModule
from gateway.navigation.routes import register_navigation_routes
from gateway.services.control_commands import ControlCommandService
from memory.spatial.tagged_locations import TaggedLocationStore


@pytest.fixture
def navigation(monkeypatch):
    # Readiness is covered separately; isolate the saved-location command boundary.
    monkeypatch.setattr(ControlCommandService, "motion_safety_rejection", lambda *args: None)
    monkeypatch.setattr(ControlCommandService, "_goal_readiness_rejection", lambda *args: None)
    store = TaggedLocationStore()
    store.tag("dock", x=1.0, y=2.0, z=0.8, yaw=1.25, metadata={
        "map_id": "yard", "map_content_epoch": 7, "frame_id": "map", "binding_status": "bound",
    })
    submitted = []

    def submit(goal, **kwargs):
        submitted.append(goal)
        return {"accepted": True, "task_id": kwargs["task_id"]}

    gw = GatewayModule()
    gw._navigation_state = {"map_id": "yard", "map_content_epoch": 7}
    gw._tagged_loc_module = SimpleNamespace(store=store)
    gw._goals = SimpleNamespace(submit_goal=submit)
    app = FastAPI()
    register_navigation_routes(app, gw)
    with TestClient(app) as client:
        yield client, gw, submitted


def _goal_request(**updates):
    return {"x": 99.0, "y": 99.0, "z": 0.0, "source": "saved_location",
            "target_type": "saved_location", "metadata": {"location_name": "dock"}, **updates}


@pytest.mark.parametrize("state,reason", [
    ({"map_id": "another-yard", "map_content_epoch": 7}, "location_map_mismatch"),
    ({"map_id": "yard", "map_content_epoch": 8}, "location_map_version_mismatch"),
    ({}, "location_active_map_unavailable"),
])
def test_preview_and_submit_reject_a_different_map(navigation, state, reason):
    client, gw, submitted = navigation
    gw._navigation_state = state
    preview = client.post("/api/v1/navigation/goal_candidate", json={"location_name": "dock", "preview": False})
    assert preview.json()["ok"] is False
    assert preview.json()["error"] == reason

    result = client.post("/api/v1/goal", json=_goal_request())
    assert result.status_code == 409
    assert result.json()["detail"]["reason"] == reason
    assert submitted == []


def test_submit_uses_saved_xyz_instead_of_browser_copy(navigation):
    client, _gw, submitted = navigation
    result = client.post("/api/v1/goal", json=_goal_request())
    assert result.status_code == 200, result.text
    assert len(submitted) == 1
    assert (submitted[0].x, submitted[0].y, submitted[0].z) == (1.0, 2.0, 0.8)
    assert submitted[0].yaw == pytest.approx(1.25)


def test_operator_can_explicitly_override_saved_heading(navigation):
    client, _gw, submitted = navigation
    result = client.post("/api/v1/goal", json=_goal_request(yaw=0.0))
    assert result.status_code == 200
    assert submitted[0].yaw == pytest.approx(0.0)


@pytest.mark.parametrize("change", ["map_version", "delete"])
def test_saved_target_is_checked_again_after_preview(navigation, change):
    client, gw, submitted = navigation
    preview = client.post("/api/v1/navigation/goal_candidate", json={"location_name": "dock", "preview": False})
    assert preview.json()["ok"] is True
    if change == "map_version":
        gw._navigation_state["map_content_epoch"] = 8
    else:
        gw._tagged_loc_module.store.remove("dock")
    result = client.post("/api/v1/goal", json=_goal_request())
    assert result.status_code == 409
    assert result.json()["command"]["accepted"] is False
    assert submitted == []


def test_unbound_saved_location_cannot_become_a_coordinate_goal(navigation):
    client, gw, submitted = navigation
    gw._tagged_loc_module.store.tag("dock", x=1, y=2)
    result = client.post("/api/v1/goal", json=_goal_request())
    assert result.status_code == 409
    assert result.json()["detail"]["reason"] == "location_map_unbound"
    assert submitted == []


def test_coordinate_goals_do_not_require_a_saved_location(navigation):
    client, _gw, submitted = navigation
    result = client.post("/api/v1/goal", json={"x": 3.0, "y": 4.0, "z": 1.2})
    assert result.status_code == 200, result.text
    assert (submitted[0].x, submitted[0].y, submitted[0].z) == (3.0, 4.0, 1.2)


def test_deleted_location_does_not_resolve_to_a_similar_name(navigation):
    client, gw, submitted = navigation
    store = gw._tagged_loc_module.store
    store.tag("dock-east", x=8, y=9, metadata=store.query("dock")["metadata"])
    preview = client.post("/api/v1/navigation/goal_candidate", json={"location_name": "dock", "preview": False})
    assert preview.json()["target"]["x"] == 1.0
    store.remove("dock")

    result = client.post("/api/v1/goal", json=_goal_request())
    assert result.status_code == 409
    assert result.json()["detail"]["reason"] == "location_not_found:dock"
    assert submitted == []

    preview = client.post("/api/v1/navigation/goal_candidate", json={"location_name": "dock", "preview": False})
    assert preview.json()["error"] == "location_not_found:dock"
