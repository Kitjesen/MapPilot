from __future__ import annotations

import asyncio
import json
from types import SimpleNamespace

from fastapi import FastAPI
from starlette.requests import Request

from gateway.routes.session import _session_end_control_blocker, register_session_routes


def _request(token: str | None = None) -> Request:
    headers = []
    if token is not None:
        headers.append((b"x-lingtu-product-session", token.encode("ascii")))
    return Request(
        {
            "type": "http",
            "method": "POST",
            "path": "/api/v1/session/end",
            "headers": headers,
        }
    )


def _gateway(*, process_control: str = "systemd"):
    return SimpleNamespace(
        _compiled_run_plan=SimpleNamespace(
            product="map",
            env="real",
            process_control=process_control,
        ),
        _session_product="map",
    )

def _route_endpoint(app: FastAPI, path: str):
    return next(route.endpoint for route in app.routes if getattr(route, "path", None) == path)



def test_field_session_end_requires_current_product_control_credential(monkeypatch):
    monkeypatch.setenv("LINGTU_PRODUCT_SESSION_ID", "session-token-1234")
    gateway = _gateway()

    missing = _session_end_control_blocker(gateway, _request())
    wrong = _session_end_control_blocker(gateway, _request("session-token-wrong"))

    assert missing == {
        "reason_code": "operator_product_control_required",
        "current_product": "map",
        "operator_command": "python -m lingtu.control stop-session --env real",
        "control_session": "unauthorized",
    }
    assert wrong is not None
    assert wrong["control_session"] == "unauthorized"
    assert (
        _session_end_control_blocker(
            gateway,
            _request("session-token-1234"),
        )
        is None
    )


def test_field_session_end_fails_closed_when_host_has_no_session_credential(monkeypatch):
    monkeypatch.delenv("LINGTU_PRODUCT_SESSION_ID", raising=False)

    blocker = _session_end_control_blocker(_gateway(), _request())

    assert blocker is not None
    assert blocker["control_session"] == "unavailable"


def test_local_session_end_does_not_require_product_control_credential(monkeypatch):
    monkeypatch.delenv("LINGTU_PRODUCT_SESSION_ID", raising=False)

    assert (
        _session_end_control_blocker(
            _gateway(process_control="local"),
            _request(),
        )
        is None
    )


def test_field_session_end_cannot_report_success_from_stale_idle_snapshot(monkeypatch):
    monkeypatch.setenv("LINGTU_PRODUCT_SESSION_ID", "session-token-1234")
    gateway = _gateway()
    gateway._session_mode = "idle"
    gateway._session_snapshot = lambda: {"mode": "idle"}
    app = FastAPI()
    register_session_routes(app, gateway)

    response = asyncio.run(_route_endpoint(app, "/api/v1/session/end")(_request()))

    assert response.status_code == 409
    payload = json.loads(response.body)
    assert payload["ok"] is False
    assert payload["detail"]["reason_code"] == "operator_product_control_required"
