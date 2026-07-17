"""MuJoCo camera source."""

from __future__ import annotations

import time
from typing import Any

from runtime.msgs.sensor import CameraIntrinsics, Image, ImageFormat
from runtime.runtime_interface import FRAMES

from ...native import Config, Info, Sample


class Camera:
    """Read RGB-D frames from a MuJoCo engine."""

    def __init__(self, engine: Any | None = None) -> None:
        self._engine = engine
        self._config = Config()
        self._connected = engine is not None

    def bind(self, engine: Any) -> None:
        self._engine = engine
        self._connected = True

    def connect(self, config: Config) -> None:
        self._config = config
        self._connected = self._engine is not None

    def disconnect(self) -> None:
        self._connected = False

    def is_connected(self) -> bool:
        return self._connected and self._engine is not None

    def info(self) -> Info:
        return Info(name=self._config.name, backend="mujoco")

    def read(self, timeout_ms: int = 0) -> Sample:
        del timeout_ms
        if not self.is_connected():
            return Sample(status={"connected": False})
        camera = self._engine.get_camera_data(self._config.name)
        if camera is None:
            return Sample(status={"connected": True, "empty": True})
        return sample_from_camera(camera)


def sample_from_camera(camera: Any, *, frame_id: str = FRAMES.camera) -> Sample:
    rgb = getattr(camera, "rgb", None)
    depth = getattr(camera, "depth", None)
    if rgb is not None:
        height, width = rgb.shape[:2]
    elif depth is not None:
        height, width = depth.shape[:2]
    else:
        return Sample(status={"empty": True})

    ts = float(getattr(camera, "timestamp", 0.0) or time.time())
    color = None
    if rgb is not None:
        color = Image(data=rgb, format=ImageFormat.RGB, ts=ts, frame_id=frame_id)

    depth_image = None
    if depth is not None:
        depth_image = Image(data=depth, format=ImageFormat.DEPTH_F32, ts=ts, frame_id=frame_id)

    fx, fy, cx, cy = camera.intrinsics
    intrinsics = CameraIntrinsics(
        fx=float(fx),
        fy=float(fy),
        cx=float(cx),
        cy=float(cy),
        width=int(width),
        height=int(height),
        depth_scale=1.0,
    )
    return Sample(
        color=color,
        depth=depth_image,
        intrinsics=intrinsics,
        timestamp=ts,
        status={"connected": True},
    )
