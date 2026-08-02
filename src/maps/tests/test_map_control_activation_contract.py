from __future__ import annotations

from types import SimpleNamespace

from maps.services.command_router import dispatch_map_command


def test_clear_active_command_uses_native_map_facade() -> None:
    calls: list[str] = []
    service = SimpleNamespace(
        _clear_active_map=lambda: calls.append("clear") or {"success": True, "active": ""}
    )

    result = dispatch_map_command(service, {"action": "clear_active"})

    assert result == {"success": True, "active": ""}
    assert calls == ["clear"]
