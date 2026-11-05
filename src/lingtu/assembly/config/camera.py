"""Resolve camera acquisition and preview settings for native process launch."""

from __future__ import annotations

import re
from collections.abc import Mapping
from typing import Any

from runtime.config import CameraConfig
from runtime.service_catalogs.real import runtime_status_files
from runtime.tf.frames import FRAMES

_CAPTURE_NUMBERS = {
    "color_width": ("COLOR_WIDTH", 16384),
    "color_height": ("COLOR_HEIGHT", 16384),
    "color_fps": ("COLOR_FPS", 2147483647),
    "depth_width": ("DEPTH_WIDTH", 16384),
    "depth_height": ("DEPTH_HEIGHT", 16384),
    "depth_fps": ("DEPTH_FPS", 2147483647),
    "timeout_ms": ("CAPTURE_TIMEOUT_MS", 2147483647),
    "startup_timeout_ms": ("STARTUP_FRAME_TIMEOUT_MS", 2147483647),
    "stale_timeout_ms": ("CAPTURE_STALE_TIMEOUT_MS", 2147483647),
}
_VIDEO_NUMBERS = {
    "fps": 120,
    "min_bitrate": 2147483647,
    "start_bitrate": 2147483647,
    "max_bitrate": 2147483647,
    "max_age_ms": 2147483647,
    "idle_timeout_ms": 2147483647,
    "metrics_interval_ms": 2147483647,
}


def _group(settings: Mapping[str, Any], name: str, fields: set[str]) -> Mapping[str, Any]:
    values = settings.get(name)
    if not isinstance(values, Mapping) or set(values) != fields:
        raise ValueError(f"camera.{name} must define: {', '.join(sorted(fields))}")
    return values


def _integer(value: Any, owner: str, maximum: int = 2147483647, *, minimum: int = 1) -> int:
    if type(value) is not int or not minimum <= value <= maximum:
        raise ValueError(f"{owner} must be an integer in [{minimum}, {maximum}]")
    return value


def _text(value: Any, owner: str, *, empty: bool = False) -> str:
    if not isinstance(value, str) or (not empty and not value):
        raise ValueError(f"{owner} must be a {'possibly empty ' if empty else 'nonempty '}string")
    return value


def camera_environment(
    settings: Mapping[str, Any], camera: CameraConfig, *, include_video: bool
) -> dict[str, str]:
    """Apply common settings, then explicit RobotConfig acquisition overrides."""
    if set(settings) != {"capture", "orbbec", "shm", "video"}:
        raise ValueError("camera settings must define capture, orbbec, shm and video")
    if camera.capture_driver not in {"orbbec_native", "realsense_native"}:
        raise ValueError(f"unsupported camera capture driver: {camera.capture_driver}")
    capture = _group(settings, "capture", set(_CAPTURE_NUMBERS))
    orbbec = _group(settings, "orbbec", {"product_id", "device_index", "connect_timeout_ms", "sdk_config", "frame_sync"})
    shm = _group(settings, "shm", {"color", "depth", "info", "slots", "min_slot_bytes", "publish_image_dds"})
    serial_key = "LINGTU_REALSENSE_SERIAL_NUMBER" if camera.capture_driver == "realsense_native" else "LINGTU_ORBBEC_SERIAL_NUMBER"
    environment = {"LINGTU_CAMERA_DRIVER": camera.capture_driver, serial_key: camera.serial_number}

    # Validate the explicit size group before common defaults can fill omissions.
    dimensions = (
        camera.capture_color_width, camera.capture_color_height,
        camera.capture_depth_width, camera.capture_depth_height,
    )
    for field, (suffix, maximum) in _CAPTURE_NUMBERS.items():
        raw = capture[field]
        owner = f"camera.capture.{field}"
        if field.startswith(("color_", "depth_")):
            override = getattr(camera, f"capture_{field}")
            if override is not None:
                raw, owner = override, f"camera.capture_{field}"
        environment[f"LINGTU_CAMERA_{suffix}"] = str(_integer(raw, owner, maximum))
    if any(value is not None for value in dimensions) and any(value is None for value in dimensions):
        raise ValueError("camera capture dimensions must set color and depth width/height together")
    environment["LINGTU_CAMERA_FRAME_ID"] = FRAMES.camera
    status_files = runtime_status_files()
    environment["LINGTU_CAMERA_STATUS_FILE"] = status_files["camera"]
    color_bytes = int(environment["LINGTU_CAMERA_COLOR_WIDTH"]) * int(environment["LINGTU_CAMERA_COLOR_HEIGHT"]) * 3
    depth_bytes = int(environment["LINGTU_CAMERA_DEPTH_WIDTH"]) * int(environment["LINGTU_CAMERA_DEPTH_HEIGHT"]) * 2
    capacity = max(_integer(shm["min_slot_bytes"], "camera.shm.min_slot_bytes"), color_bytes, depth_bytes)
    if capacity > 128 * 1024 * 1024:
        raise ValueError("camera capture dimensions exceed the 128 MiB frame protocol limit")
    environment["LINGTU_CAMERA_SHM_SLOT_CAPACITY_BYTES"] = str(capacity)

    environment["LINGTU_ORBBEC_CONNECT_TIMEOUT_MS"] = str(_integer(orbbec["connect_timeout_ms"], "camera.orbbec.connect_timeout_ms"))
    environment["LINGTU_ORBBEC_PRODUCT_ID"] = _text(orbbec["product_id"], "camera.orbbec.product_id")
    environment["LINGTU_ORBBEC_DEVICE_INDEX"] = str(_integer(orbbec["device_index"], "camera.orbbec.device_index", minimum=0))
    environment["LINGTU_ORBBEC_SDK_CONFIG"] = _text(orbbec["sdk_config"], "camera.orbbec.sdk_config", empty=True)
    for values, field, target in (
        (orbbec, "frame_sync", "LINGTU_ORBBEC_ENABLE_FRAME_SYNC"),
        (shm, "publish_image_dds", "LINGTU_CAMERA_PUBLISH_IMAGE_DDS"),
    ):
        if not isinstance(values[field], bool):
            raise ValueError(f"camera.{field} must be a boolean")
        environment[target] = "1" if values[field] else "0"
    for field in ("color", "depth", "info"):
        environment[f"LINGTU_CAMERA_{field.upper()}_SHM"] = _text(shm[field], f"camera.shm.{field}")
    environment["LINGTU_CAMERA_SHM_SLOT_COUNT"] = str(_integer(shm["slots"], "camera.shm.slots"))

    if include_video:
        video = _group(settings, "video", {*_VIDEO_NUMBERS, "listen", "metrics_file", "stun_server"})
        for field, maximum in _VIDEO_NUMBERS.items():
            overridden = field == "fps" and camera.video_fps is not None
            value = camera.video_fps if overridden else video[field]
            owner = "camera.video_fps" if overridden else f"camera.video.{field}"
            minimum = 0 if field == "idle_timeout_ms" else 1
            environment[f"LINGTU_VIDEO_{field.upper()}"] = str(_integer(value, owner, maximum, minimum=minimum))
        if not video["min_bitrate"] <= video["start_bitrate"] <= video["max_bitrate"]:
            raise ValueError("camera.video bitrates must satisfy min <= start <= max")
        for field in ("listen", "metrics_file", "stun_server"):
            environment[f"LINGTU_VIDEO_{field.upper()}"] = _text(video[field], f"camera.video.{field}", empty=field == "stun_server")
        environment["LINGTU_VIDEO_COLOR_SHM"] = environment["LINGTU_CAMERA_COLOR_SHM"]
        environment["LINGTU_VIDEO_STATUS_FILE"] = status_files["video"]
        environment["LINGTU_WEBRTC_URL"] = environment["LINGTU_VIDEO_LISTEN"].rstrip("/") + "/whep/endpoint"
        if camera.video_encoder is not None:
            if not isinstance(camera.video_encoder, str) or re.fullmatch(r"[A-Za-z][A-Za-z0-9_-]*", camera.video_encoder) is None:
                raise ValueError("camera.video_encoder must be a GStreamer factory name")
            environment["LINGTU_VIDEO_ENCODER"] = camera.video_encoder
    return environment
