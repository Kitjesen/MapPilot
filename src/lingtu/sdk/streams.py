"""Resource-scoped subscriptions to the Gateway's Server-Sent Events."""

from __future__ import annotations

import asyncio
import json
import urllib.request
from collections.abc import AsyncIterator, Generator, Iterable, Iterator
from contextlib import suppress
from dataclasses import dataclass
from typing import TYPE_CHECKING, Any
from urllib.parse import urlencode

if TYPE_CHECKING:
    from .client import LingTuClient


@dataclass(frozen=True)
class Event:
    """One SSE message. Gateway event types are inside the JSON data."""

    data: str
    event: str = "message"
    id: str = ""

    def json(self) -> Any:
        return json.loads(self.data)


def _parse_events(lines: Iterable[bytes]) -> Iterator[Event]:
    data: list[str] = []
    event = "message"
    event_id = ""
    first = True
    for raw in lines:
        line = raw.decode("utf-8-sig" if first else "utf-8").rstrip("\r\n")
        first = False
        if not line:
            if data:
                yield Event("\n".join(data), event or "message", event_id)
            data = []
            event = "message"
            continue
        if line.startswith(":"):
            continue
        field, _, value = line.partition(":")
        if value.startswith(" "):
            value = value[1:]
        if field == "data":
            data.append(value)
        elif field == "event":
            event = value
        elif field == "id" and "\x00" not in value:
            event_id = value


def events(
    client: LingTuClient,
    *,
    topic: str | None = None,
    include_elevation: bool = False,
    last_event_id: str | None = None,
) -> Generator[Event, None, None]:
    """Subscribe until closed. Reconnection is explicit and never replays commands.

    Close the iterator or use ``contextlib.closing`` when stopping early.
    The client's timeout bounds a blocked socket read, including during close.
    """
    query: dict[str, str] = {}
    if topic is not None:
        query["topic"] = topic
    if include_elevation:
        query["include_elevation"] = "1"
    url = client.base_url + "/api/v1/events"
    if query:
        url += "?" + urlencode(query)
    headers = {"Accept": "text/event-stream"}
    if client._api_key:
        headers["X-API-Key"] = client._api_key
    if last_event_id is not None:
        headers["Last-Event-ID"] = last_event_id
    request = urllib.request.Request(url, headers=headers)
    with urllib.request.urlopen(request, timeout=client._timeout) as response:
        yield from _parse_events(response)


def _next_event(stream: Iterator[Event]) -> Event | None:
    return next(stream, None)


async def async_events(
    client: LingTuClient,
    *,
    topic: str | None = None,
    include_elevation: bool = False,
    last_event_id: str | None = None,
) -> AsyncIterator[Event]:
    """Subscribe without blocking the event loop and close the pending read on exit.

    Cancellation waits for the active read, bounded by the client's socket
    timeout. A Gateway subscription normally receives a ping every second.
    """
    stream = events(client, topic=topic, include_elevation=include_elevation, last_event_id=last_event_id)
    pending: asyncio.Task[Event | None] | None = None
    try:
        while True:
            pending = asyncio.create_task(asyncio.to_thread(_next_event, stream))
            event = await asyncio.shield(pending)
            if event is None:
                return
            yield event
    except asyncio.CancelledError:
        if pending is not None:
            # A read failure during cleanup must not replace the cancellation.
            with suppress(Exception):
                await pending
        raise
    finally:
        stream.close()
