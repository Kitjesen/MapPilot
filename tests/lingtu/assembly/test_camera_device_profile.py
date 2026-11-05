from dataclasses import replace
from pathlib import Path

import pytest
import yaml

import lingtu.assembly.compile_plan as compiler
from lingtu.assembly.config.camera import camera_environment
from lingtu.assembly.config.product import resolve_product_host_runtime
from lingtu.run_plan import RunPlan
from runtime.config import CameraConfig, load_config


def _settings():
    path = Path(__file__).resolve().parents[3] / "config/system/settings/camera.yaml"
    return yaml.safe_load(path.read_text(encoding="utf-8"))["camera"]


def _compile_with_camera(monkeypatch, camera):
    resolved = resolve_product_host_runtime("nav", "real", robot="unitree/go2", product_variant="camera")
    env = replace(resolved.env_spec, robot_config=replace(resolved.env_spec.robot_config, camera=camera))
    monkeypatch.setattr(compiler, "resolve_product_host_runtime", lambda *_args, **_kwargs: replace(resolved, env_spec=env))
    return compiler.compile_run_plan("nav", "real", robot="unitree/go2", product_variant="camera")


def test_common_settings_reach_run_plan_without_target_encoder(monkeypatch):
    plan = _compile_with_camera(monkeypatch, CameraConfig())
    environment = plan.process_environment
    assert environment["LINGTU_CAMERA_DRIVER"] == "orbbec_native"
    assert environment["LINGTU_ORBBEC_SERIAL_NUMBER"] == ""
    assert environment["LINGTU_CAMERA_COLOR_WIDTH"] == "640"
    assert environment["LINGTU_CAMERA_DEPTH_FPS"] == "30"
    assert environment["LINGTU_CAMERA_SHM_SLOT_CAPACITY_BYTES"] == "1048576"
    assert environment["LINGTU_VIDEO_FPS"] == "20"
    assert "LINGTU_VIDEO_ENCODER" not in environment


def test_explicit_yaml_profile_reaches_run_plan_without_rewriting_calibration(monkeypatch, tmp_path):
    path = tmp_path / "robot.yaml"
    path.write_text("""camera:
  capture_driver: realsense_native
  serial_number: fixture-camera
  capture_color_width: 1280
  capture_color_height: 720
  capture_color_fps: 30
  capture_depth_width: 640
  capture_depth_height: 480
  capture_depth_fps: 15
  video_encoder: x264enc
  video_fps: 20
  width: 320
  height: 240
  fx: 310.0
""", encoding="utf-8")
    config = load_config(str(path))
    assert (config.camera.width, config.camera.height, config.camera.fx) == (320, 240, 310.0)
    plan = _compile_with_camera(monkeypatch, config.camera)
    environment = plan.process_environment
    expected = {
        "LINGTU_CAMERA_DRIVER": "realsense_native", "LINGTU_REALSENSE_SERIAL_NUMBER": "fixture-camera",
        "LINGTU_CAMERA_COLOR_WIDTH": "1280", "LINGTU_CAMERA_COLOR_HEIGHT": "720", "LINGTU_CAMERA_COLOR_FPS": "30",
        "LINGTU_CAMERA_DEPTH_WIDTH": "640", "LINGTU_CAMERA_DEPTH_HEIGHT": "480", "LINGTU_CAMERA_DEPTH_FPS": "15",
        "LINGTU_CAMERA_SHM_SLOT_CAPACITY_BYTES": str(1280 * 720 * 3),
        "LINGTU_VIDEO_ENCODER": "x264enc", "LINGTU_VIDEO_FPS": "20",
    }
    assert expected.items() <= environment.items()
    assert RunPlan.from_dict(plan.as_dict()).process_environment == environment


@pytest.mark.parametrize("settings", [
    {"capture_color_width": 1280},
    {"capture_color_height": 720},
    {"capture_depth_width": 640},
    {"capture_depth_height": 480},
    {"capture_color_width": 1280, "capture_color_height": 720},
    {"capture_depth_width": 640, "capture_depth_height": 480},
    {"capture_color_width": 1280, "capture_color_height": 720, "capture_depth_width": 640},
])
def test_partial_sizes_are_rejected_instead_of_guessing_other_service_overrides(monkeypatch, settings):
    with pytest.raises(ValueError, match="dimensions must set color and depth width/height together"):
        _compile_with_camera(monkeypatch, CameraConfig(**settings))


def test_complete_sizes_keep_minimum_capacity_and_fps_can_be_overridden_independently():
    small = camera_environment(_settings(), CameraConfig(
        capture_color_width=320, capture_color_height=240,
        capture_depth_width=320, capture_depth_height=240,
    ), include_video=False)
    assert small["LINGTU_CAMERA_SHM_SLOT_CAPACITY_BYTES"] == "1048576"
    fps_only = camera_environment(_settings(), CameraConfig(capture_color_fps=15, capture_depth_fps=30), include_video=False)
    assert fps_only["LINGTU_CAMERA_COLOR_FPS"] == "15"
    assert fps_only["LINGTU_CAMERA_DEPTH_FPS"] == "30"
    assert fps_only["LINGTU_CAMERA_SHM_SLOT_CAPACITY_BYTES"] == "1048576"


@pytest.mark.parametrize("settings, message", [
    ({"capture_color_width": 0}, "capture_color_width"),
    ({"capture_color_height": 16385}, "capture_color_height"),
    ({"capture_color_fps": True}, "capture_color_fps"),
    ({"capture_depth_fps": "30"}, "capture_depth_fps"),
    ({"capture_depth_width": -1}, "capture_depth_width"),
    ({"capture_color_width": 16384, "capture_color_height": 16384,
      "capture_depth_width": 640, "capture_depth_height": 480}, "128 MiB"),
    ({"video_encoder": ""}, "video_encoder"),
    ({"video_encoder": "x264enc tune=zerolatency"}, "video_encoder"),
    ({"video_fps": 121}, "video_fps"),
    ({"video_fps": 20.0}, "video_fps"),
])
def test_invalid_profile_is_rejected_before_creating_a_run_plan(monkeypatch, settings, message):
    with pytest.raises(ValueError, match=message):
        _compile_with_camera(monkeypatch, CameraConfig(**settings))


def test_video_overrides_are_only_emitted_when_video_is_selected():
    environment = camera_environment(_settings(), CameraConfig(video_encoder="x264enc", video_fps=20), include_video=False)
    assert "LINGTU_VIDEO_ENCODER" not in environment
    assert "LINGTU_VIDEO_FPS" not in environment
