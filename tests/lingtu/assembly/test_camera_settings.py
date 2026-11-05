from pathlib import Path

import pytest
import yaml

from lingtu.assembly.config.camera import camera_environment
from lingtu.real.systemd import _ServiceInspector
from runtime.config import CameraConfig
from runtime.service_catalogs.real import service_metadata


@pytest.fixture
def settings():
    path = Path(__file__).resolve().parents[3] / "config/system/settings/camera.yaml"
    return yaml.safe_load(path.read_text(encoding="utf-8"))["camera"]


def test_acquisition_and_video_settings_reach_process_environment(settings):
    settings["capture"]["color_fps"] = 24
    settings["capture"]["timeout_ms"] = 1234
    settings["orbbec"]["frame_sync"] = True
    settings["shm"]["color"] = "/fixture_color"
    settings["video"]["start_bitrate"] = 900000
    settings["video"]["listen"] = "http://127.0.0.1:9099"

    environment = camera_environment(settings, CameraConfig(), include_video=True)

    assert environment["LINGTU_CAMERA_COLOR_FPS"] == "24"
    assert environment["LINGTU_CAMERA_CAPTURE_TIMEOUT_MS"] == "1234"
    assert environment["LINGTU_ORBBEC_ENABLE_FRAME_SYNC"] == "1"
    assert environment["LINGTU_VIDEO_COLOR_SHM"] == "/fixture_color"
    assert environment["LINGTU_VIDEO_START_BITRATE"] == "900000"
    assert environment["LINGTU_WEBRTC_URL"] == "http://127.0.0.1:9099/whep/endpoint"


def test_robot_acquisition_overrides_shared_settings(settings):
    settings["capture"]["color_fps"] = 24
    settings["video"]["fps"] = 17
    camera = CameraConfig(capture_color_fps=15, video_fps=10, video_encoder="x264enc")

    environment = camera_environment(settings, camera, include_video=True)

    assert environment["LINGTU_CAMERA_COLOR_FPS"] == "15"
    assert environment["LINGTU_VIDEO_FPS"] == "10"
    assert environment["LINGTU_VIDEO_ENCODER"] == "x264enc"


def test_shm_diagnostics_use_session_channels(settings, monkeypatch):
    settings["shm"].update(color="/test_color", depth="/test_depth", info="/test_info")
    environment = camera_environment(settings, CameraConfig(), include_video=True)
    inspector = _ServiceInspector(environment=environment)
    monkeypatch.setattr(inspector._systemd, "available", lambda _target: False)
    metadata = service_metadata(environment=environment)

    for service, expected in (
        ("camera", ["/test_color", "/test_depth", "/test_info"]),
        ("video", ["/test_color"]),
    ):
        detail = inspector.status_details(service, dds_check=False, http_check=False)[service]
        assert detail["contract"]["shm_channels"] == expected
        assert metadata[service]["shm_channels"] == expected


@pytest.mark.parametrize("group, field, value", [
    ("capture", "color_width", 0),
    ("capture", "color_fps", True),
    ("orbbec", "frame_sync", "true"),
    ("shm", "slots", 0),
    ("video", "fps", 121),
    ("video", "start_bitrate", 5000000),
])
def test_invalid_settings_fail_at_launch_compilation(settings, group, field, value):
    settings[group][field] = value
    with pytest.raises(ValueError, match="camera"):
        camera_environment(settings, CameraConfig(), include_video=True)
