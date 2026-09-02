# ruff: noqa: S101

import json

import pytest

from gateway.services.rate_limit import RateLimitMiddleware


async def _ok_app(scope, receive, send):
    await send({"type": "http.response.start", "status": 200, "headers": []})
    await send({"type": "http.response.body", "body": b"ok"})


async def _request(app, path="/api/v1/estop/reset"):
    sent = []

    async def receive():
        return {"type": "http.request", "body": b"", "more_body": False}

    async def send(message):
        sent.append(message)

    await app(
        {
            "type": "http",
            "path": path,
            "headers": [],
            "query_string": b"",
            "client": ("192.0.2.10", 5050),
        },
        receive,
        send,
    )
    return sent


@pytest.mark.asyncio
async def test_estop_reset_uses_the_control_rate_limit():
    app = RateLimitMiddleware(
        _ok_app,
        enabled=True,
        control_rps=1,
        query_rps=100,
        window_s=60,
        burst=1,
    )

    first = await _request(app)
    second = await _request(app)

    assert first[0]["status"] == 200
    assert second[0]["status"] == 429
    assert json.loads(second[1]["body"])["message"] == ("Too many control requests. Slow down.")


@pytest.mark.asyncio
async def test_task_cancel_uses_control_limit_without_reclassifying_task_status_reads():
    app = RateLimitMiddleware(
        _ok_app,
        enabled=True,
        control_rps=1,
        query_rps=2,
        window_s=60,
        burst=1,
    )
    cancel_path = "/api/v1/navigation/tasks/task-1/cancel"
    status_path = "/api/v1/navigation/tasks/task-1"

    first_cancel = await _request(app, cancel_path)
    second_cancel = await _request(app, cancel_path)

    assert first_cancel[0]["status"] == 200
    assert second_cancel[0]["status"] == 429

    query_app = RateLimitMiddleware(
        _ok_app,
        enabled=True,
        control_rps=1,
        query_rps=2,
        window_s=60,
        burst=1,
    )
    first_status = await _request(query_app, status_path)
    second_status = await _request(query_app, status_path)
    assert first_status[0]["status"] == 200
    assert second_status[0]["status"] == 200


@pytest.mark.parametrize("action", ["pause", "resume"])
@pytest.mark.asyncio
async def test_task_pause_and_resume_use_the_control_rate_limit(action):
    app = RateLimitMiddleware(
        _ok_app,
        enabled=True,
        control_rps=1,
        query_rps=100,
        window_s=60,
        burst=1,
    )
    path = f"/api/v1/navigation/tasks/task-1/{action}"

    first = await _request(app, path)
    second = await _request(app, path)

    assert first[0]["status"] == 200
    assert second[0]["status"] == 429
    assert json.loads(second[1]["body"])["message"] == (
        "Too many control requests. Slow down."
    )
