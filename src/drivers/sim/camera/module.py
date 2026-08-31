"""Simulation camera runtime module."""

from __future__ import annotations

import logging
import threading
import time
from collections import deque
from typing import Any

from runtime.contracts import CAMERA_BACKEND_SIM, CAMERA_ROLE
from runtime.module import Module
from runtime.msgs.sensor import CameraIntrinsics, Image, PointCloud2
from runtime.registry import register
from runtime.stream import Out

from .impl.mujoco import Camera
from .native import Config

logger = logging.getLogger(__name__)


@register(CAMERA_ROLE, CAMERA_BACKEND_SIM, description="MuJoCo RGB-D camera stream")
class MujocoCameraModule(Module, layer=1):
    """Publish MuJoCo RGB-D camera frames through the camera ports."""

    color_image: Out[Image]
    depth_image: Out[Image]
    camera_info: Out[CameraIntrinsics]
    points: Out[PointCloud2]
    alive: Out[bool]
    runtime_id = CAMERA_ROLE

    def __init__(
        self,
        engine: Any | None = None,
        camera_name: str = "front_camera",
        fps: float | None = None,
        driver_module: str = "MujocoDriverModule",
        **kw,
    ) -> None:
        super().__init__(**kw)
        self._source = Camera(engine)
        default_config = Config(name=camera_name)
        self._fps = float(default_config.fps if fps is None else fps)
        if self._fps <= 0.0:
            raise ValueError("fps must be greater than zero")
        self._config = Config(name=camera_name, fps=int(round(self._fps)))
        self._driver_module = driver_module
        self._modules: dict[str, Module] = {}
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None
        self._publish_times: deque[float] = deque(maxlen=16)
        self._rate_lock = threading.Lock()
        self._camera_info_period_s = 1.0
        self._next_camera_info_publish_s: float | None = None
        self._error: str | None = None

    def bind(self, engine: Any) -> None:
        self._source.bind(engine)
        self._source.connect(self._config)
        self._next_camera_info_publish_s = None
        self._error = None

    def on_system_modules(self, modules: dict[str, Module]) -> None:
        self._modules = dict(modules)
        self._try_bind_driver_engine()

    def setup(self) -> None:
        self._try_bind_driver_engine()
        self._source.connect(self._config)
        if not self._source.is_connected():
            self._error = "MuJoCo camera engine is not bound"
            self.alive.publish(False)
        self._stop_event.clear()
        if self._thread is None or not self._thread.is_alive():
            self._next_camera_info_publish_s = None
            self._thread = threading.Thread(target=self._loop, name="sim-camera", daemon=True)
            self._thread.start()
        self.alive.publish(self._source.is_connected())

    def stop(self) -> None:
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
        self._source.disconnect()
        self.alive.publish(False)
        super().stop()

    def health(self) -> dict[str, object]:
        info = super().port_summary()
        info["role"] = CAMERA_ROLE
        info["camera_backend"] = CAMERA_BACKEND_SIM
        info["backend"] = "mujoco"
        info["device"] = self._config.name
        info["error"] = self._error
        info["nominal_rate_hz"] = self._fps
        info["camera_info_nominal_rate_hz"] = 1.0
        info["fps"] = self._observed_publish_rate()
        return info

    def poll(self) -> None:
        self._publish_sample()

    def _try_bind_driver_engine(self) -> bool:
        if self._source.is_connected():
            return True
        driver = self._modules.get(self._driver_module)
        if driver is None:
            for name, module in self._modules.items():
                if name == self._driver_module or type(module).__name__ == self._driver_module:
                    driver = module
                    break
        engine = getattr(driver, "engine", None) if driver is not None else None
        if engine is None:
            return False
        self.bind(engine)
        return True

    def _loop(self) -> None:
        period = 1.0 / self._fps
        next_deadline = time.monotonic()
        while not self._stop_event.is_set():
            self._publish_sample()
            next_deadline += period
            now = time.monotonic()
            if now >= next_deadline:
                missed = int((now - next_deadline) // period) + 1
                next_deadline += missed * period
            if self._stop_event.wait(max(0.0, next_deadline - now)):
                break

    def _publish_sample(self) -> None:
        if not self._source.is_connected() and not self._try_bind_driver_engine():
            self._error = "MuJoCo camera engine is not bound"
            self.alive.publish(False)
            return
        try:
            sample = self._source.read()
        except Exception as exc:
            self._error = str(exc)
            logger.debug("MujocoCameraModule: read failed: %s", exc)
            return

        published = False
        if sample.color is not None:
            self.color_image.publish(sample.color)
            published = True
        if sample.depth is not None:
            self.depth_image.publish(sample.depth)
            published = True
        if sample.points is not None:
            self.points.publish(sample.points)
            published = True
        observed_at = time.monotonic()
        if sample.intrinsics is not None and self._camera_info_due(observed_at):
            self.camera_info.publish(sample.intrinsics)
        if published:
            self._record_publication(observed_at)
        self._error = None
        self.alive.publish(True)

    def _record_publication(self, now: float | None = None) -> None:
        observed_at = time.monotonic() if now is None else float(now)
        with self._rate_lock:
            self._publish_times.append(observed_at)

    def _camera_info_due(self, now: float) -> bool:
        deadline = self._next_camera_info_publish_s
        if deadline is not None and now < deadline:
            return False
        self._next_camera_info_publish_s = now + self._camera_info_period_s
        return True

    def _observed_publish_rate(self) -> float:
        with self._rate_lock:
            if len(self._publish_times) < 2:
                return 0.0
            elapsed = self._publish_times[-1] - self._publish_times[0]
            if elapsed <= 0.0:
                return 0.0
            return (len(self._publish_times) - 1) / elapsed


CameraModule = MujocoCameraModule
