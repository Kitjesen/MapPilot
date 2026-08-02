"""Gateway contract tests for explicit native directed exploration intent."""

from __future__ import annotations

import asyncio
import json
import math
from types import SimpleNamespace

import pytest
from fastapi import FastAPI
from pydantic import ValidationError

from gateway.routes.operations import register_operation_routes
from gateway.schemas import (
    DirectedExplorationClearRequest,
    DirectedExplorationResponse,
    DirectedExplorationTargetRequest,
)
from gateway.services import exploration


def _route_endpoint(app: FastAPI, path: str):
    for route in app.routes:
        if getattr(route, "path", None) == path:
            return route.endpoint
    raise AssertionError(f"route not found: {path}")


def _native_tare_status(
    *,
    active: bool = True,
    session_id: str = "tare-session-42",
    frame_id: str = "map",
) -> dict[str, object]:
    return {
        "schema_version": "lingtu.explore.status.v2",
        "endpoint": "lingtu_explore_dds",
        "stamp_s": 100.0,
        "active": active,
        "session_id": session_id,
        "map": {
            "frame_id": frame_id,
            "session_id": "rolling-map-session-7",
        },
    }


class _DirectedCommands:
    def __init__(self, *, accepted: bool = True) -> None:
        self.accepted = accepted
        self.calls: list[tuple[object, ...]] = []

    def set_directed_exploration_target(
        self,
        x: float,
        y: float,
        ttl_s: float,
        session_id: str,
        reason: str,
        request_id: str | None,
    ) -> bool:
        self.calls.append(("set", x, y, ttl_s, session_id, reason, request_id))
        return self.accepted

    def clear_directed_exploration_target(
        self,
        session_id: str,
        reason: str,
        request_id: str | None,
    ) -> bool:
        self.calls.append(("clear", session_id, reason, request_id))
        return self.accepted


class _KeywordOnlyDirectedCommands:
    """Mimics the runtime RPC proxy, which exposes keyword-only callers."""

    def __init__(self) -> None:
        self.calls: list[dict[str, object]] = []

    def set_directed_exploration_target(self, **kwargs: object) -> bool:
        self.calls.append(kwargs)
        return True


def _app_and_gateway(commands: object | None) -> tuple[FastAPI, SimpleNamespace]:
    gateway = SimpleNamespace(
        _go2rtc_upstream="",
        _nav_commands=commands,
        _all_modules={},
        # Deliberately present a local explorer: directed intent must never
        # fall back to it when the native command boundary is unavailable.
        _tare_explorer=SimpleNamespace(start_tare_exploration=lambda: (_ for _ in ()).throw(AssertionError())),
        _frontier_explorer=None,
    )
    app = FastAPI()
    register_operation_routes(app, gateway)
    return app, gateway


def _response_payload(response_or_payload):
    if hasattr(response_or_payload, "body"):
        return json.loads(response_or_payload.body)
    return response_or_payload


def test_directed_schema_requires_finite_coordinates_and_conservative_ttl() -> None:
    request = DirectedExplorationTargetRequest(x=2.0, y=-3.0, ttl_s=30.0)

    assert request.ttl_s == 30.0
    assert request.reason == "operator_directed_explore"
    assert DirectedExplorationClearRequest().reason == "operator_clear_directed_explore"
    for invalid in (math.nan, math.inf, -math.inf):
        with pytest.raises(ValidationError):
            DirectedExplorationTargetRequest(x=invalid, y=0.0)
    with pytest.raises(ValidationError):
        DirectedExplorationTargetRequest(x=0.0, y=0.0, ttl_s=0.0)
    with pytest.raises(ValidationError):
        DirectedExplorationTargetRequest(x=0.0, y=0.0, ttl_s=121.0)
    with pytest.raises(ValidationError):
        DirectedExplorationTargetRequest(x=0.0, y=0.0, session_id="client-must-not-select-session")
    with pytest.raises(ValidationError):
        DirectedExplorationTargetRequest(x=0.0, y=0.0, bbox=[-1.0, -1.0, 1.0, 1.0])
    with pytest.raises(ValidationError):
        DirectedExplorationTargetRequest(x=0.0, y=0.0, frame_id="odom")


def test_directed_route_forwards_server_derived_session_and_map_context(monkeypatch) -> None:
    commands = _DirectedCommands()
    app, _gateway = _app_and_gateway(commands)
    monkeypatch.setattr(exploration, "_native_status", lambda: _native_tare_status())

    response = asyncio.run(
        _route_endpoint(app, "/api/v1/explore/directed")(
            DirectedExplorationTargetRequest(
                x=12.5,
                y=-4.25,
                ttl_s=45.0,
                reason="operator_check_east",
                request_id="directed-123",
            )
        )
    )
    payload = _response_payload(response)
    model = DirectedExplorationResponse.model_validate(payload)

    assert payload["ok"] is True
    assert payload["accepted"] is True
    assert payload["status"] == "accepted"
    assert payload["intent"] == {
        "active": True,
        "x": 12.5,
        "y": -4.25,
        "ttl_s": 45.0,
        "session_id": "tare-session-42",
        "frame_id": "map",
        "reason": "operator_check_east",
        "request_id": "directed-123",
    }
    assert model.intent.session_id == "tare-session-42"
    assert model.intent.frame_id == "map"
    assert payload["native"] == {"active": True, "state": None, "stamp_s": 100.0}
    assert commands.calls == [
        ("set", 12.5, -4.25, 45.0, "tare-session-42", "operator_check_east", "directed-123")
    ]


@pytest.mark.parametrize(
    ("status", "error"),
    [
        (None, "native_tare_status_unavailable"),
        (_native_tare_status(active=False), "native_tare_not_active"),
        (_native_tare_status(session_id=""), "native_tare_session_unavailable"),
        (_native_tare_status(frame_id="odom"), "native_tare_map_frame_mismatch"),
        ({**_native_tare_status(), "map": None}, "native_tare_map_context_unavailable"),
    ],
)
def test_directed_route_rejects_missing_or_incompatible_native_context(monkeypatch, status, error) -> None:
    commands = _DirectedCommands()
    app, _gateway = _app_and_gateway(commands)
    monkeypatch.setattr(exploration, "_native_status", lambda: status)

    response = asyncio.run(
        _route_endpoint(app, "/api/v1/explore/directed")(
            DirectedExplorationTargetRequest(x=1.0, y=2.0)
        )
    )
    payload = _response_payload(response)

    assert response.status_code in {409, 503}
    assert payload["ok"] is False
    assert payload["error"] == error
    assert commands.calls == []


def test_directed_route_never_falls_back_when_native_command_is_missing(monkeypatch) -> None:
    app, gateway = _app_and_gateway(commands=None)
    monkeypatch.setattr(exploration, "_native_status", lambda: _native_tare_status())

    response = asyncio.run(
        _route_endpoint(app, "/api/v1/explore/directed")(
            DirectedExplorationTargetRequest(x=1.0, y=2.0)
        )
    )
    payload = _response_payload(response)

    assert response.status_code == 503
    assert payload["ok"] is False
    assert payload["error"] == "directed_exploration_command_unavailable"
    assert gateway._tare_explorer is not None


def test_directed_route_reports_native_rejection_and_clear_forwards_context(monkeypatch) -> None:
    rejected = _DirectedCommands(accepted=False)
    app, _gateway = _app_and_gateway(rejected)
    monkeypatch.setattr(exploration, "_native_status", lambda: _native_tare_status())

    rejected_response = asyncio.run(
        _route_endpoint(app, "/api/v1/explore/directed")(
            DirectedExplorationTargetRequest(x=1.0, y=2.0, request_id="set-rejected")
        )
    )
    rejected_payload = _response_payload(rejected_response)

    assert rejected_response.status_code == 409
    assert rejected_payload["error"] == "directed_exploration_command_rejected"
    assert rejected.calls == [
        ("set", 1.0, 2.0, 30.0, "tare-session-42", "operator_directed_explore", "set-rejected")
    ]

    accepted = _DirectedCommands()
    app, _gateway = _app_and_gateway(accepted)
    clear_response = asyncio.run(
        _route_endpoint(app, "/api/v1/explore/directed/clear")(
            DirectedExplorationClearRequest(
                reason="operator_clear_directed_explore",
                request_id="clear-1",
            )
        )
    )
    clear_payload = _response_payload(clear_response)
    clear_model = DirectedExplorationResponse.model_validate(clear_payload)

    assert clear_payload["ok"] is True
    assert clear_payload["accepted"] is True
    assert clear_payload["status"] == "cleared"
    assert clear_payload["intent"] == {
        "active": False,
        "x": None,
        "y": None,
        "ttl_s": None,
        "session_id": "tare-session-42",
        "frame_id": "map",
        "reason": "operator_clear_directed_explore",
        "request_id": "clear-1",
    }
    assert clear_model.intent.active is False
    assert accepted.calls == [
        ("clear", "tare-session-42", "operator_clear_directed_explore", "clear-1")
    ]
