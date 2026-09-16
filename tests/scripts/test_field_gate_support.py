from __future__ import annotations

import pytest

from diagnostics.field.gate_support import GateError, require_no_control_authority


def test_no_active_command_source_accepts_no_authority() -> None:
    require_no_control_authority({"control": {"authority": "NONE"}}, "test")


@pytest.mark.parametrize("authority", ["AUTONOMY", "OPERATOR", "UNKNOWN"])
def test_no_active_command_source_rejects_authority(authority: str) -> None:
    with pytest.raises(GateError, match=f"control_authority={authority}"):
        require_no_control_authority({"control": {"authority": authority}}, "test")


def test_no_active_command_source_fails_closed_when_control_is_missing() -> None:
    with pytest.raises(GateError, match="control_authority=UNKNOWN"):
        require_no_control_authority({}, "test")
