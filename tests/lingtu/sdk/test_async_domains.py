"""Behavioral contracts for the grouped asynchronous SDK clients."""

from __future__ import annotations

import asyncio
import io
import json
import threading
import urllib.error
from unittest.mock import Mock

import pytest

from lingtu.sdk import AsyncLingTuClient, Pose2D
from lingtu.sdk.client import LingTuClient


@pytest.mark.asyncio
async def test_agent_domain_preserves_owner_receipt_and_shared_http_settings(monkeypatch):
    payload = {
        "ok": True,
        "accepted": True,
        "operation": "query",
        "run_id": "agent-current",
        "task_id": "navigation-child",
        "state": "running",
        "execution_confirmed": False,
        "motor_confirmed": False,
    }
    transport = Mock(return_value=io.BytesIO(json.dumps(payload).encode()))
    monkeypatch.setattr("urllib.request.urlopen", transport)
    robot = AsyncLingTuClient("robot.local", api_key="test-key", timeout=2.5)

    result = await robot.tasks.send_instruction(
        "current task?",
        run_id="agent-current",
        control_only=True,
        request_id="query-1",
        client_id="operator-console",
    )

    request = transport.call_args.args[0]
    assert request.full_url == "http://robot.local:5050/api/v1/instruction"
    assert request.get_method() == "POST"
    assert request.get_header("X-api-key") == "test-key"
    assert transport.call_args.kwargs["timeout"] == 2.5
    assert json.loads(request.data) == {
        "text": "current task?",
        "run_id": "agent-current",
        "control_only": True,
        "request_id": "query-1",
        "client_id": "operator-console",
    }
    assert result.accepted and result.run_id == "agent-current"
    assert result.task_id == "navigation-child"
    assert result.raw == payload


@pytest.mark.asyncio
async def test_follow_domain_cancel_preserves_both_owner_identities(monkeypatch):
    payload = {"ok": True, "accepted": True, "stop_confirmed": False}
    transport = Mock(return_value=io.BytesIO(json.dumps(payload).encode()))
    monkeypatch.setattr("urllib.request.urlopen", transport)

    result = await AsyncLingTuClient().tasks.cancel_follow_task(
        "follow/one",
        "product-session-1",
        request_id="cancel-1",
        client_id="operator-console",
    )

    request = transport.call_args.args[0]
    assert request.full_url.endswith("/api/v1/follow/follow%2Fone/cancel")
    assert json.loads(request.data) == {
        "product_session_id": "product-session-1",
        "reason": "operator_cancel",
        "request_id": "cancel-1",
        "client_id": "operator-console",
    }
    assert result.accepted and result.request_id == "cancel-1"
    assert result.raw["stop_confirmed"] is False


@pytest.mark.asyncio
async def test_navigation_relocalization_preserves_pose_seed(monkeypatch):
    transport = Mock(return_value=io.BytesIO(b'{"ok": true, "accepted": true}'))
    monkeypatch.setattr("urllib.request.urlopen", transport)
    robot = AsyncLingTuClient()

    result = await robot.navigation.relocalize(
        "factory",
        initial_pose=Pose2D(x=1.0, y=2.0, yaw=0.5),
        request_id="seed-1",
    )

    request = transport.call_args.args[0]
    assert request.full_url.endswith("/api/v1/localization/relocalizations")
    assert json.loads(request.data) == {
        "mode": "seeded",
        "map_name": "factory",
        "initial_pose": {"x": 1.0, "y": 2.0, "yaw": 0.5},
        "request_id": "seed-1",
    }
    assert result.accepted


@pytest.mark.asyncio
async def test_recording_capture_and_emergency_stop_keep_distinct_routes(monkeypatch):
    transport = Mock(side_effect=lambda *args, **kwargs: io.BytesIO(b'{"ok": true}'))
    monkeypatch.setattr("urllib.request.urlopen", transport)
    robot = AsyncLingTuClient()

    await robot.observations.start_recording(
        "inspection",
        duration=120,
        capture_profile="evidence",
        task_id="inspection-1",
        camera=True,
        minimum_free_gib=12,
    )
    await robot.observations.stop_recording(expected_session_id="recording-1")
    await robot.system.emergency_stop()

    requests = [call.args[0] for call in transport.call_args_list]
    assert [request.full_url.rsplit("/api/v1/", 1)[1] for request in requests] == [
        "recordings/start",
        "recordings/stop",
        "stop",
    ]
    assert json.loads(requests[0].data) == {
        "prefix": "inspection",
        "duration": 120,
        "capture_profile": "evidence",
        "task_id": "inspection-1",
        "camera": True,
        "minimum_free_gib": 12,
    }
    assert json.loads(requests[1].data) == {"expected_session_id": "recording-1"}


@pytest.mark.asyncio
async def test_grouped_queries_and_commands_keep_distinct_offline_results(monkeypatch):
    transport = Mock(side_effect=urllib.error.URLError("connection refused"))
    monkeypatch.setattr("urllib.request.urlopen", transport)
    robot = AsyncLingTuClient()

    with pytest.raises(ConnectionError, match="GET /api/v1/slam/maps failed"):
        await robot.map.list()

    result = await robot.tasks.send_instruction("find a bottle")

    assert not result.ok and not result.accepted
    assert result.run_id is None
    assert result.raw["error"] == "robot not reachable"


@pytest.mark.asyncio
async def test_domain_operation_runs_off_the_event_loop(monkeypatch):
    robot = AsyncLingTuClient()
    loop_thread = threading.get_ident()
    entered = threading.Event()
    release = threading.Event()
    expected = {"operation": {"state": "RUNNING"}}

    def blocking_operation(operation_id):
        assert operation_id == "operation-1"
        assert threading.get_ident() != loop_thread
        entered.set()
        assert release.wait(timeout=2), "event loop did not release the worker"
        return expected

    monkeypatch.setattr(robot._client.map, "operation", blocking_operation)
    task = asyncio.create_task(robot.map.operation("operation-1"))
    try:
        assert await asyncio.to_thread(entered.wait, 1)
        assert not task.done()
    finally:
        release.set()
    assert await asyncio.wait_for(task, timeout=2) is expected


@pytest.mark.asyncio
async def test_new_domain_view_uses_the_current_synchronous_client(monkeypatch):
    transport = Mock(return_value=io.BytesIO(b'{"ready": false}'))
    monkeypatch.setattr("urllib.request.urlopen", transport)
    robot = AsyncLingTuClient("old-host")
    robot._client = LingTuClient("new-host", 6060, timeout=1.5)

    assert await robot.system.readiness() == {"ready": False}
    assert transport.call_args.args[0].full_url == "http://new-host:6060/api/v1/readiness"
    assert transport.call_args.kwargs["timeout"] == 1.5
