"""Inspection SDK contracts against the Gateway HTTP surface."""

from __future__ import annotations

import json
import urllib.error
from io import BytesIO
from types import SimpleNamespace
from unittest.mock import MagicMock, Mock, patch

import pytest
from fastapi import FastAPI
from fastapi.testclient import TestClient

from lingtu.sdk import AsyncLingTuClient, CommandResult, LingTuClient


def _http_response(payload: dict[str, object]) -> MagicMock:
    response = MagicMock()
    response.__enter__.return_value.read.return_value = json.dumps(payload).encode()
    return response


@patch("urllib.request.urlopen")
def test_inspection_routes_queries_the_selected_map(mock_urlopen: Mock) -> None:
    mock_urlopen.return_value = _http_response(
        {"ok": True, "map_id": "factory/a", "routes": [], "count": 0}
    )

    result = LingTuClient().inspection_routes("factory/a")

    assert result["map_id"] == "factory/a"
    request = mock_urlopen.call_args.args[0]
    assert request.full_url.endswith("/api/v1/inspection/routes?map_id=factory%2Fa")


@patch("urllib.request.urlopen")
@pytest.mark.parametrize("binding", [{}, {"expected_product_session_id": "session-1", "expected_env": "sim"}])
def test_start_inspection_preserves_task_and_reusable_request_id(
    mock_urlopen: Mock,
    binding: dict[str, str],
) -> None:
    mock_urlopen.return_value = _http_response(
        {
            "ok": True,
            "accepted": True,
            "action": "start",
            "task_id": "inspection-task-1",
            "request_id": "request-1",
            "lifecycle": "submission_accepted",
        }
    )

    result = LingTuClient().start_inspection(
        "route-1",
        map_id="factory",
        revision=4,
        request_id="request-1",
        **binding,
    )

    assert isinstance(result, CommandResult)
    assert result.ok is True
    assert result.task_id == "inspection-task-1"
    assert result.request_id == "request-1"
    request = mock_urlopen.call_args.args[0]
    assert request.method == "POST"
    assert json.loads(request.data) == {
        "route_id": "route-1",
        "map_id": "factory",
        "revision": 4,
        "request_id": "request-1",
        **binding,
    }


@pytest.mark.parametrize("action", ["pause", "resume", "cancel"])
@pytest.mark.parametrize("binding", [{}, {"expected_product_session_id": "session-1", "expected_env": "sim"}])
@patch("urllib.request.urlopen")
def test_control_inspection_uses_task_identity_and_request_id(
    mock_urlopen: Mock,
    action: str,
    binding: dict[str, str],
) -> None:
    mock_urlopen.return_value = _http_response(
        {
            "ok": True,
            "accepted": True,
            "action": action,
            "task_id": "task/1",
            "request_id": f"request-{action}",
        }
    )
    client = LingTuClient()

    result = getattr(client, f"{action}_inspection")(
        "task/1",
        reason=f"operator_{action}",
        request_id=f"request-{action}",
        **binding,
    )

    assert result.task_id == "task/1"
    assert result.request_id == f"request-{action}"
    request = mock_urlopen.call_args.args[0]
    assert request.method == "POST"
    assert request.full_url.endswith(
        f"/api/v1/inspection/tasks/task%2F1/{action}"
    )
    assert json.loads(request.data) == {
        "reason": f"operator_{action}",
        "request_id": f"request-{action}",
        **binding,
    }


@pytest.mark.parametrize(
    ("method_name", "suffix"),
    [("inspection_task", ""), ("inspection_report", "/report")],
)
@patch("urllib.request.urlopen")
def test_inspection_task_queries_quote_the_task_id(
    mock_urlopen: Mock,
    method_name: str,
    suffix: str,
) -> None:
    mock_urlopen.return_value = _http_response(
        {"found": True, "task_id": "task/1", "current_state": "EXECUTING"}
    )

    result = getattr(LingTuClient(), method_name)("task/1")

    assert result["task_id"] == "task/1"
    request = mock_urlopen.call_args.args[0]
    assert request.full_url.endswith(f"/api/v1/inspection/tasks/task%2F1{suffix}")


@pytest.mark.asyncio
async def test_async_inspection_methods_delegate_to_the_sync_client() -> None:
    client = Mock(spec=LingTuClient)
    client.start_inspection.return_value = CommandResult(
        ok=True,
        task_id="inspection-task-1",
        request_id="request-1",
    )
    robot = AsyncLingTuClient()
    robot._client = client

    result = await robot.start_inspection(
        "route-1",
        map_id="factory",
        revision=4,
        request_id="request-1",
        expected_product_session_id="session-1",
        expected_env="sim",
    )

    assert result.task_id == "inspection-task-1"
    client.start_inspection.assert_called_once_with(
        "route-1",
        map_id="factory",
        revision=4,
        request_id="request-1",
        expected_product_session_id="session-1",
        expected_env="sim",
    )


@pytest.mark.parametrize("action", ["start", "pause", "resume", "cancel"])
@pytest.mark.parametrize(("binding", "reason"), [
    ({"expected_product_session_id": "older-session"}, "inspection_product_session_mismatch"),
    ({"expected_env": "real"}, "inspection_expected_env_mismatch"),
])
def test_binding_rejection_comes_from_gateway(monkeypatch, action, binding, reason):
    from gateway.routes import inspection

    monkeypatch.setattr(inspection, "runtime_identity", lambda _gw: {
        "state": "active", "env": "sim", "product_session_id": "session-1",
    })
    timeline = Mock()
    timeline.query.return_value = {"product_session_id": "session-1"}
    monkeypatch.setattr(inspection, "ensure_inspection_task_timeline", lambda _gw: timeline)
    native = Mock()
    monkeypatch.setattr(inspection, "_run_native_task_command", native)
    app = FastAPI()
    inspection.register_inspection_routes(app, SimpleNamespace())

    with TestClient(app) as http:
        def transport(request, **_options):
            response = http.request(
                request.get_method(), request.full_url, content=request.data,
                headers=dict(request.header_items()),
            )
            assert response.status_code == 409, response.text
            raise urllib.error.HTTPError(request.full_url, 409, "Gateway conflict", response.headers, BytesIO(response.content))

        monkeypatch.setattr("urllib.request.urlopen", transport)
        result = getattr(LingTuClient().tasks, f"{action}_inspection")("task-1", **binding)

    assert not result.ok
    assert result.raw["error"] == reason
    native.assert_not_called()


@pytest.mark.asyncio
@pytest.mark.parametrize("action", ["start", "pause", "resume", "cancel"])
async def test_async_grouped_binding_reaches_http(monkeypatch, action):
    transport = Mock(return_value=_http_response({"ok": True, "accepted": True}))
    monkeypatch.setattr("urllib.request.urlopen", transport)
    await getattr(AsyncLingTuClient().tasks, f"{action}_inspection")(
        "task-1", expected_product_session_id="session-1", expected_env="sim",
    )
    body = json.loads(transport.call_args.args[0].data)
    assert body["expected_product_session_id"] == "session-1"
    assert body["expected_env"] == "sim"
