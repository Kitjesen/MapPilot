from __future__ import annotations

from dataclasses import dataclass

import pytest

from lingtu.sim.viewer_input import (
    ViewerInput,
    ViewerInputConfig,
    ViewerKeys,
)


@dataclass
class _Receipt:
    accepted: bool = True
    reason: str = "accepted"


class _Client:
    def __init__(self) -> None:
        self.calls: list[tuple[object, ...]] = []

    def claim(self, source_id, source_epoch, sequence, **kwargs):
        self.calls.append(("claim", source_id, source_epoch, sequence, kwargs))
        return _Receipt()

    def sample(self, source_id, source_epoch, sequence, vx, vy, wz, **kwargs):
        self.calls.append(
            ("sample", source_id, source_epoch, sequence, vx, vy, wz, kwargs)
        )
        return True

    def hold(self, source_id, source_epoch, sequence, **kwargs):
        self.calls.append(("hold", source_id, source_epoch, sequence, kwargs))
        return _Receipt()

    def release(self, source_id, source_epoch, sequence, **kwargs):
        self.calls.append(("release", source_id, source_epoch, sequence, kwargs))
        return _Receipt()

    def close(self) -> None:
        self.calls.append(("close",))


def test_viewer_focus_is_the_deadman_and_plain_w_drives_forward() -> None:
    held = {ord("W")}
    keys = ViewerKeys(
        key_state=lambda code: 0x8000 if code in held else 0,
        foreground_title=lambda: "MuJoCo : thunderv4",
    )

    command = keys.command(linear_speed_mps=0.5, yaw_speed_rad_s=0.6)

    assert command.vx == pytest.approx(0.5)
    assert command.vy == pytest.approx(0.0)
    assert command.wz == pytest.approx(0.0)
    assert command.manual_mode is False


def test_viewer_keys_release_motion_when_mujoco_loses_focus() -> None:
    keys = ViewerKeys(
        key_state=lambda _code: 0x8000,
        foreground_title=lambda: "Codex",
    )

    assert keys.command(linear_speed_mps=0.5, yaw_speed_rad_s=0.6).is_zero()


def test_viewer_input_claims_samples_then_holds_and_releases() -> None:
    client = _Client()
    pressed = {ord("W")}
    keys = ViewerKeys(
        key_state=lambda code: 0x8000 if code in pressed else 0,
        foreground_title=lambda: "MuJoCo : thunderv4",
    )
    control = ViewerInput(
        ViewerInputConfig(linear_speed_mps=0.5, yaw_speed_rad_s=0.6),
        keys=keys,
        client_factory=lambda: client,
        source_epoch=123,
    )

    pressed.clear()
    control.poll_once()
    pressed.add(ord("W"))
    control.poll_once()
    pressed.clear()
    control.poll_once()
    control.close()

    assert [call[0] for call in client.calls] == [
        "claim",
        "sample",
        "hold",
        "release",
        "close",
    ]
    sample = client.calls[1]
    assert sample[4:7] == pytest.approx((0.5, 0.0, 0.0))
    assert sample[7]["deadman"] is True


def test_viewer_input_uses_a_new_epoch_after_releasing_authority() -> None:
    client = _Client()
    pressed = {ord("W")}
    keys = ViewerKeys(
        key_state=lambda code: 0x8000 if code in pressed else 0,
        foreground_title=lambda: "MuJoCo : thunderv4",
    )
    control = ViewerInput(
        ViewerInputConfig(),
        keys=keys,
        client_factory=lambda: client,
        source_epoch=123,
    )

    pressed.clear()
    control.poll_once()
    pressed.add(ord("W"))
    control.poll_once()
    pressed.clear()
    control.poll_once()
    pressed.add(ord("W"))
    control.poll_once()

    claims = [call for call in client.calls if call[0] == "claim"]
    assert [call[2] for call in claims] == [123, 124]


def test_viewer_input_requires_neutral_keys_before_first_motion() -> None:
    client = _Client()
    pressed = {ord("E")}
    keys = ViewerKeys(
        key_state=lambda code: 0x8000 if code in pressed else 0,
        foreground_title=lambda: "MuJoCo : thunderv4",
    )
    control = ViewerInput(
        ViewerInputConfig(),
        keys=keys,
        client_factory=lambda: client,
        source_epoch=123,
    )

    control.poll_once()
    assert client.calls == []
    pressed.clear()
    control.poll_once()
    pressed.add(ord("E"))
    control.poll_once()

    assert [call[0] for call in client.calls] == ["claim", "sample"]
