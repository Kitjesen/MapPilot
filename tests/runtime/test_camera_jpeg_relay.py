from __future__ import annotations

import importlib
from types import SimpleNamespace

import pytest

from drivers.real.camera_jpeg_relay import CameraJpegRelayModule


def test_camera_jpeg_relay_has_no_motion_control_surface() -> None:
    relay = CameraJpegRelayModule()

    assert set(relay.ports_in) == {"color_image", "scene_graph"}
    assert relay.ports_out == {}
    assert not hasattr(relay, "velocity_input")
    assert not hasattr(relay, "cmd_vel")
    assert not hasattr(relay, "teleop_active")
    assert not hasattr(relay, "force_release")


def test_camera_jpeg_relay_registers_only_as_gateway_media_provider() -> None:
    relay = CameraJpegRelayModule()
    gateway = SimpleNamespace(_camera_module=None)

    relay.on_system_modules({"GatewayModule": gateway})

    assert gateway._camera_module is relay
    assert not hasattr(gateway, "_teleop_module")


def test_camera_preview_reports_missing_encoder_at_setup(monkeypatch) -> None:
    original = importlib.import_module

    def unavailable(name: str):
        if name == "cv2":
            raise ModuleNotFoundError("No module named 'cv2'")
        return original(name)

    monkeypatch.setattr("drivers.real.camera_jpeg_relay.importlib.import_module", unavailable)
    with pytest.raises(RuntimeError, match="declared vision dependency"):
        CameraJpegRelayModule().setup()
