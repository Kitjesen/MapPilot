from __future__ import annotations

import importlib
import threading
from concurrent.futures import ThreadPoolExecutor
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


def test_concurrent_viewers_preserve_counts_and_encoder_wake() -> None:
    relay = CameraJpegRelayModule()
    relay._latest_raw = object()
    connected = threading.Barrier(17)
    release = threading.Event()

    def viewer(index: int) -> None:
        connect = relay.on_client_connect if index % 2 else relay.on_camera_client_connect
        disconnect = relay.on_client_disconnect if index % 2 else relay.on_camera_client_disconnect
        connect()
        connected.wait(timeout=3)
        assert release.wait(timeout=3)
        disconnect()

    with ThreadPoolExecutor(max_workers=16) as pool:
        futures = [pool.submit(viewer, index) for index in range(16)]
        try:
            connected.wait(timeout=3)
            status = relay.health()
            assert (status["teleop_clients"], status["camera_clients"], status["stream_clients"]) == (8, 8, 16)
            assert relay._new_frame.is_set()
        finally:
            release.set()
        for future in futures:
            future.result(timeout=3)

    status = relay.health()
    assert (status["teleop_clients"], status["camera_clients"], status["stream_clients"]) == (0, 0, 0)
    relay.on_client_disconnect()
    relay.on_camera_client_disconnect()
    assert relay.health()["stream_clients"] == 0


def test_camera_images_wake_encoder_only_with_a_viewer() -> None:
    relay = CameraJpegRelayModule()
    relay._gateway = object()
    image = SimpleNamespace(to_bgr=lambda: SimpleNamespace(data=b"frame"))
    relay._on_image(image)
    assert not relay._new_frame.is_set()
    relay.on_camera_client_connect()
    relay._new_frame.clear()
    relay._on_image(image)
    assert relay._new_frame.is_set()
