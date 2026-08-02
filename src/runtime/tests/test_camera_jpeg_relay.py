from __future__ import annotations

from types import SimpleNamespace

from drivers.real.camera_jpeg_relay import CameraJpegRelayModule


def test_camera_jpeg_relay_has_no_motion_control_surface() -> None:
    relay = CameraJpegRelayModule()

    assert set(relay.ports_in) == {"color_image", "scene_graph"}
    assert relay.ports_out == {}
    assert not hasattr(relay, "joy_input")
    assert not hasattr(relay, "cmd_vel")
    assert not hasattr(relay, "teleop_active")
    assert not hasattr(relay, "force_release")


def test_camera_jpeg_relay_registers_only_as_gateway_media_provider() -> None:
    relay = CameraJpegRelayModule()
    gateway = SimpleNamespace(_camera_module=None)

    relay.on_system_modules({"GatewayModule": gateway})

    assert gateway._camera_module is relay
    assert not hasattr(gateway, "_teleop_module")
