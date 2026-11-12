from __future__ import annotations

import asyncio
import io
import json
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
from fastapi import FastAPI
from pydantic import ValidationError

from gateway.navigation.routes import register_navigation_routes
from gateway.schemas import SelectPlannerRequest
from lingtu.sdk import LingTuClient
from nav.commands import module as command_module
from runtime.msgs import NavigationCommandKind


@pytest.mark.parametrize("accepted, reason", [(True, "planner_selected"), (False, "task_active")])
def test_http_selection_preserves_native_result(monkeypatch, accepted, reason):
    native = Mock()
    native.select_planner.return_value = {
        "accepted": accepted, "kind": int(NavigationCommandKind.SELECT_PLANNER),
        "task_id": "", "request_id": "planner-1", "endpoint_timestamp_s": 123.0, "reason": reason,
    }
    monkeypatch.setattr(command_module, "get_native_navigation_client", lambda **_: native)
    gateway = SimpleNamespace(
        _nav_commands=command_module.Commands(),
        _run_control_command=lambda command, body, action, **_: action(),
    )
    app = FastAPI()
    register_navigation_routes(app, gateway)
    endpoint = next(route.endpoint for route in app.routes if route.path == "/api/v1/navigation/planner")
    response = asyncio.run(endpoint(SelectPlannerRequest(name="cmu", request_id="planner-1")))
    if accepted:
        assert response["accepted"] is True
        assert response["reason"] == reason
    else:
        assert response.status_code == 409
        assert reason in response.body.decode()
    native.select_planner.assert_called_once_with("cmu", request_id="planner-1")


def test_selection_rejects_unknown_name_before_native_call():
    with pytest.raises(ValidationError):
        SelectPlannerRequest(name="anything")


def test_selection_rejects_mismatched_native_receipt(monkeypatch):
    native = Mock()
    native.select_planner.return_value = {
        "accepted": True, "kind": int(NavigationCommandKind.SELECT_PLANNER),
        "task_id": "", "request_id": "another-request", "endpoint_timestamp_s": 123.0,
        "reason": "planner_selected",
    }
    monkeypatch.setattr(command_module, "get_native_navigation_client", lambda **_: native)
    with pytest.raises(RuntimeError, match="wrong request_id"):
        command_module.Commands().select_planner("scan", request_id="planner-1")


def test_sdk_selection_uses_existing_http_transport(monkeypatch):
    transport = Mock(return_value=io.BytesIO(b'{"accepted":true}'))
    monkeypatch.setattr("urllib.request.urlopen", transport)
    result = LingTuClient().navigation.select_planner("scan", request_id="planner-1")
    assert result.accepted
    request = transport.call_args.args[0]
    assert request.full_url.endswith("/api/v1/navigation/planner")
    assert json.loads(request.data) == {"name": "scan", "request_id": "planner-1", "client_id": "sdk"}
