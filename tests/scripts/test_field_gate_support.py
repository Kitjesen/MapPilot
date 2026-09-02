from __future__ import annotations

import pytest
from diagnostics.field.gate_support import GateError, require_no_active_command_source


def test_no_active_command_source_accepts_native_idle() -> None:
    require_no_active_command_source({"control": {"active_cmd_source": "none"}}, "test")


@pytest.mark.parametrize("source", ["autonomy", "teleop", "manual_hold", "estop"])
def test_no_active_command_source_rejects_native_authority(source: str) -> None:
    with pytest.raises(GateError, match=f"active_cmd_source={source}"):
        require_no_active_command_source({"control": {"active_cmd_source": source}}, "test")
