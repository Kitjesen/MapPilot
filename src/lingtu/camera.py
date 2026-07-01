"""Camera — Orbbec Gemini 335, import and use."""

from __future__ import annotations

import logging

import numpy as np

logger = logging.getLogger(__name__)


class Camera:
    """RGB-D camera — color + depth + intrinsics.

    Usage::

        camera = Camera()
        camera.start()
        rgb = camera.get_color()      # numpy HxWx3
        depth = camera.get_depth()    # numpy HxW
        camera.stop()
    """

    def __init__(
        self,
        color_topic: str = "/camera/color/image_raw",
        depth_topic: str = "/camera/depth/image_raw",
    ):
        self._color_topic = color_topic
        self._depth_topic = depth_topic
        self._bridge = None
        self._latest_color: np.ndarray | None = None
        self._latest_depth: np.ndarray | None = None
        self._intrinsics = None
        self._started = False

    def start(self) -> Camera:
        if self._started:
            return self
        try:
            from compat.ros2.camera_bridge import CameraBridgeModule
            self._bridge = CameraBridgeModule(
                color_topic=self._color_topic,
                depth_topic=self._depth_topic,
            )
            self._bridge.setup()
            self._bridge.color_image._add_callback(self._on_color)
            self._bridge.depth_image._add_callback(self._on_depth)
            self._bridge.camera_info._add_callback(self._on_info)
            self._bridge.start()
            self._started = True
            logger.info("Camera started")
        except Exception as e:
            logger.error("Camera start failed: %s", e)
        return self

    def stop(self) -> None:
        if self._bridge:
            self._bridge.stop()
            self._bridge = None
        self._started = False

    def get_color(self) -> np.ndarray | None:
        return self._latest_color

    def get_depth(self) -> np.ndarray | None:
        return self._latest_depth

    def get_intrinsics(self):
        return self._intrinsics

    def _on_color(self, img):
        self._latest_color = img.data

    def _on_depth(self, img):
        self._latest_depth = img.data

    def _on_info(self, info):
        self._intrinsics = (info.fx, info.fy, info.cx, info.cy)

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, *args):
        self.stop()

    def __repr__(self):
        return "Camera(running=%s)" % self._started
