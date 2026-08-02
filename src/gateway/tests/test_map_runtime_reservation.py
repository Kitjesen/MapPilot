from __future__ import annotations

import pytest

pytest.importorskip("fastapi")


class FakeMaps:
    def __init__(self) -> None:
        self.commands: list[dict[str, object]] = []

    def execute(self, request):
        self.commands.append(request.to_mapping())
        raise AssertionError("maps service must not run during a reserved transition")


def test_normal_activation_observes_session_reservation_inside_shared_lock() -> None:
    from gateway.services.map_service import activate_runtime_map

    class Gateway:
        _session_mode = "idle"
        _session_map = None
        _session_pending = True
        _map_mgr = FakeMaps()

    result = activate_runtime_map(Gateway(), "office", lambda _path: {"ok": True})

    assert result["success"] is False
    assert result["reason_code"] == "session_transition_in_progress"
    assert Gateway._map_mgr.commands == []
