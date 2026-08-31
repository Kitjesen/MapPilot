from __future__ import annotations

import asyncio
import json
import time

import pytest

pytest.importorskip("fastapi")


def _endpoint(gateway, path: str):
    for route in gateway._app.routes:
        if route.path == path:
            return route.endpoint
    raise AssertionError(f"route not found: {path}")


def _current_scene(*, stamp_s: float = 100.0) -> dict:
    identity = {
        "producer_boot_id": "mapd-boot-a",
        "reset_epoch": 7,
        "observation_sequence": 41,
        "generation": 43,
        "live": True,
    }
    return {
        "ts": stamp_s,
        "frame_id": "map",
        "map_id": "yard-a",
        "source": "mapd",
        "metadata": identity,
        "layers": [
            {"id": "maps.live_cloud", "type": "pointcloud"},
            {"id": "maps.elevation", "type": "grid", "metadata": identity},
        ],
    }


def test_environment_map_feedback_unifies_layers_without_exposing_internal_epochs() -> None:
    from gateway.services.environment_map_feedback import EnvironmentMapFeedback

    feedback = EnvironmentMapFeedback()
    feedback.observe_scene(_current_scene())

    payload = feedback.snapshot(
        traversability_status={
            "stamp_s": 100.5,
            "counters": {"published": 3},
        },
        nav_endpoint_status={"has_traversability": True},
        now_s=101.0,
    )

    assert payload["schema_version"] == "lingtu.environment_map.status.v1"
    assert payload["state"] == "ready"
    assert payload["map_id"] == "yard-a"
    assert payload["frame_id"] == "map"
    assert payload["layers"]["scene"]["state"] == "ready"
    assert payload["layers"]["elevation"]["state"] == "ready"
    assert payload["layers"]["traversability"]["state"] == "diagnostic_only"
    assert payload["layers"]["traversability"]["message_code"] == "native_grid_available_via_sse"
    assert payload["limitations"] == ["native_risk_grid_available_via_sse_only"]
    assert "reset_epoch" not in json.dumps(payload)
    assert "producer_boot_id" not in json.dumps(payload)


def test_environment_map_feedback_marks_old_scene_stale_and_does_not_claim_elevation_ready() -> None:
    from gateway.services.environment_map_feedback import EnvironmentMapFeedback

    feedback = EnvironmentMapFeedback(freshness_limit_s=5.0)
    feedback.observe_scene(_current_scene(stamp_s=10.0))

    payload = feedback.snapshot(
        traversability_status={"stamp_s": 10.0, "counters": {"published": 3}},
        nav_endpoint_status={"has_traversability": True},
        now_s=16.0,
    )

    assert payload["state"] == "stale"
    assert payload["message_code"] == "wait_for_current_environment_map"
    assert payload["layers"]["scene"]["state"] == "stale"
    assert payload["layers"]["elevation"]["state"] == "stale"
    assert payload["layers"]["traversability"]["state"] == "stale"


def test_environment_map_feedback_does_not_call_an_undated_risk_engine_current() -> None:
    from gateway.services.environment_map_feedback import EnvironmentMapFeedback

    feedback = EnvironmentMapFeedback()
    feedback.observe_scene(_current_scene())

    payload = feedback.snapshot(
        traversability_status={"counters": {"published": 3}},
        nav_endpoint_status={"has_traversability": True},
        now_s=101.0,
    )

    assert payload["state"] == "unavailable"
    assert payload["layers"]["traversability"]["state"] == "unavailable"
    assert payload["layers"]["traversability"]["message_code"] == "traversability_status_timestamp_missing"


def test_environment_map_feedback_ignores_a_noncanonical_elevation_preview() -> None:
    from gateway.services.environment_map_feedback import EnvironmentMapFeedback

    scene = _current_scene()
    scene["layers"][1]["id"] = "maps.elevation_preview"
    feedback = EnvironmentMapFeedback()
    feedback.observe_scene(scene)

    payload = feedback.snapshot(
        traversability_status={"stamp_s": 100.5, "counters": {"published": 3}},
        nav_endpoint_status={"has_traversability": True},
        now_s=101.0,
    )

    assert payload["layers"]["elevation"]["state"] == "unavailable"
    assert payload["layers"]["elevation"]["message_code"] == "elevation_layer_not_available"


def test_environment_map_status_route_returns_operator_feedback(monkeypatch, tmp_path) -> None:
    from gateway.gateway_module import GatewayModule

    traversability_status = tmp_path / "traversability_status.json"
    now_s = time.time()
    traversability_status.write_text(
        json.dumps({"stamp_s": now_s, "counters": {"published": 3}}),
        encoding="utf-8",
    )
    nav_status = tmp_path / "nav_endpoint_status.json"
    nav_status.write_text(
        json.dumps({"has_traversability": True}),
        encoding="utf-8",
    )
    monkeypatch.setenv("LINGTU_TRAVERSABILITY_STATUS_FILE", str(traversability_status))
    monkeypatch.setenv("LINGTU_NAV_STATUS_FILE", str(nav_status))

    gateway = GatewayModule()
    gateway.setup()
    gateway._on_map_scene(_current_scene(stamp_s=now_s))

    payload = asyncio.run(_endpoint(gateway, "/api/v1/maps/environment/layers")())

    assert payload["state"] == "ready"
    assert payload["layers"]["traversability"]["message_code"] == "native_grid_available_via_sse"
    assert "reset_epoch" not in json.dumps(payload)


def test_environment_map_cache_reset_clears_public_scene_feedback() -> None:
    from gateway.gateway_module import GatewayModule

    gateway = GatewayModule()
    gateway.setup()
    gateway._on_map_scene(_current_scene(stamp_s=time.time()))

    gateway.clear_map_cloud_cache(reason="localization_restart")
    payload = asyncio.run(_endpoint(gateway, "/api/v1/maps/environment/layers")())

    assert payload["state"] == "updating"
    assert payload["map_id"] is None
    assert payload["frame_id"] is None
    assert payload["layers"]["scene"]["state"] == "updating"
    assert payload["layers"]["scene"]["message_code"] == "map_scene_reset_pending"


def test_localization_reset_clears_public_environment_map_feedback() -> None:
    from gateway.gateway_module import GatewayModule
    from gateway.services.pose_recovery import clear_localization_runtime_cache

    gateway = GatewayModule()
    gateway.setup()
    gateway._on_map_scene(_current_scene(stamp_s=time.time()))

    clear_localization_runtime_cache(gateway, reason="localization_restart")
    payload = asyncio.run(_endpoint(gateway, "/api/v1/maps/environment/layers")())

    assert payload["state"] == "updating"
    assert payload["layers"]["scene"]["message_code"] == "map_scene_reset_pending"


def test_environment_map_status_does_not_hide_a_missing_feedback_service() -> None:
    from fastapi import HTTPException

    from gateway.gateway_module import GatewayModule

    gateway = GatewayModule()
    gateway.setup()
    gateway._environment_map_feedback = None

    with pytest.raises(HTTPException) as raised:
        asyncio.run(_endpoint(gateway, "/api/v1/maps/environment/layers")())

    assert raised.value.status_code == 503
    assert raised.value.detail == "environment_map_feedback_unavailable"
