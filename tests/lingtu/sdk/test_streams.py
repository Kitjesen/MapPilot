"""SSE parsing and subscription lifetime through the actual SDK transport."""

import asyncio
import io
from contextlib import closing
from unittest.mock import patch

import pytest

from lingtu.sdk import LingTuClient
from lingtu.sdk.streams import Event, _parse_events, async_events, events


def test_multiline_ids_comments_and_partial_eof():
    lines = (
        b"\xef\xbb\xbf: ping\r\n",
        b"id: 7\r\n", b"event: scene\r\n", b"data: first\r\n", b"data: second\r\n", b"\r\n",
        b"data: {\"ok\":true}\n", b"\n", b"data: unfinished\n",
    )
    result = list(_parse_events(lines))
    assert result == [Event("first\nsecond", "scene", "7"), Event('{"ok":true}', id="7")]
    assert result[1].json() == {"ok": True}


def test_stream_uses_configuration_and_closes_early():
    response = io.BytesIO('data: {"name":"地图"}\n\ndata: unused\n\n'.encode())
    client = LingTuClient("robot", api_key="test-key", timeout=2)
    with patch("urllib.request.urlopen", return_value=response) as open_url:
        with closing(events(client, topic="/nav/state", include_elevation=True, last_event_id="6")) as stream:
            assert next(stream).json() == {"name": "地图"}
        assert response.closed
    request = open_url.call_args.args[0]
    assert request.full_url == "http://robot:5050/api/v1/events?topic=%2Fnav%2Fstate&include_elevation=1"
    assert request.get_header("X-api-key") == "test-key"
    assert request.get_header("Last-event-id") == "6"
    assert open_url.call_args.kwargs == {"timeout": 2}


@pytest.mark.asyncio
async def test_async_stream_keeps_loop_running_and_closes():
    response = io.BytesIO(b"data: first\n\ndata: second\n\n")
    with patch("urllib.request.urlopen", return_value=response):
        stream = async_events(LingTuClient())
        assert (await anext(stream)).data == "first"
        await stream.aclose()
    assert response.closed


@pytest.mark.asyncio
async def test_async_cancellation_reaps_read_before_closing(monkeypatch):
    import threading

    entered = threading.Event()
    release = threading.Event()
    response = io.BytesIO(b"data: event\n\n")

    def open_url(*args, **kwargs):
        entered.set()
        release.wait(2)
        return response

    monkeypatch.setattr("urllib.request.urlopen", open_url)
    stream = async_events(LingTuClient())
    task = asyncio.create_task(anext(stream))
    await asyncio.to_thread(entered.wait, 2)
    task.cancel()
    release.set()
    with pytest.raises(asyncio.CancelledError):
        await task
    assert response.closed


@pytest.mark.asyncio
async def test_async_cancellation_survives_pending_read_failure(monkeypatch):
    import threading

    entered = threading.Event()
    release = threading.Event()

    def open_url(*args, **kwargs):
        entered.set()
        release.wait(2)
        raise TimeoutError("stream timed out while cancelling")

    monkeypatch.setattr("urllib.request.urlopen", open_url)
    stream = async_events(LingTuClient())
    task = asyncio.create_task(anext(stream))
    await asyncio.to_thread(entered.wait, 2)
    task.cancel()
    release.set()
    with pytest.raises(asyncio.CancelledError):
        await task
