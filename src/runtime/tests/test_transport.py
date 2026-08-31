from __future__ import annotations

from runtime.transport.local import LocalTransport


def test_local_bus_delivers_by_topic() -> None:
    transport = LocalTransport()
    seen: list[object] = []
    transport.subscribe("/test", seen.append)
    transport.publish("/test", {"value": 1})
    transport.publish("/other", {"value": 2})
    assert seen == [{"value": 1}]
