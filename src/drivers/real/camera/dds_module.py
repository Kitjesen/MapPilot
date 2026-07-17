"""Camera Module backed by the native POSIX shared-memory data plane.

``DdsCameraModule`` is kept as the registered compatibility name.  Full color
and depth frames are read from SHM; Python does not create a CycloneDDS reader.
The native camera service may still publish low-bandwidth CameraInfo over DDS.
"""

from __future__ import annotations

import logging
import os
import threading
import time
from pathlib import Path
from typing import cast

from message.dds import dds_topic_name
from runtime.contracts import CAMERA_BACKEND_DDS, CAMERA_ROLE
from runtime.module import Module
from runtime.msgs.numpy_compat import np
from runtime.msgs.sensor import CameraIntrinsics, Image, ImageFormat
from runtime.registry import register
from runtime.runtime_interface import TOPICS
from runtime.stream import Out

from .shm import (
    POSIX_SHM_DIRECTORY,
    FrameChanged,
    FrameCorrupt,
    FrameNotReady,
    FrameSnapshot,
    FrameStale,
    ShmFrameReader,
    ShmUnavailable,
    StreamKind,
    posix_shm_path,
)

logger = logging.getLogger(__name__)

_DEFAULT_COLOR_SHM = "/lingtu_camera_color"
_DEFAULT_DEPTH_SHM = "/lingtu_camera_depth"
_DEFAULT_INFO_SHM = "/lingtu_camera_info"


def _configured_shm_path(argument: str | None, env_name: str, fallback: str) -> Path:
    value = (argument or os.environ.get(env_name, "") or fallback).strip()
    if value.startswith(f"{POSIX_SHM_DIRECTORY}/"):
        return Path(value)
    return posix_shm_path(value)


@register(CAMERA_ROLE, CAMERA_BACKEND_DDS, description="Native SHM RGB-D camera stream")
class DdsCameraModule(Module, layer=1):  # type: ignore[call-arg]
    """Publish canonical camera ports from C++-owned SHM frame rings."""

    color_image: Out[Image]
    depth_image: Out[Image]
    camera_info: Out[CameraIntrinsics]
    alive: Out[bool]
    runtime_id = CAMERA_ROLE

    def __init__(
        self,
        domain_id: int | None = None,
        color_topic: str | None = None,
        depth_topic: str | None = None,
        info_topic: str | None = None,
        color_shm_path: str | None = None,
        depth_shm_path: str | None = None,
        info_shm_path: str | None = None,
        stale_timeout_s: float = 1.0,
        poll_interval_s: float = 0.005,
        **kw,
    ) -> None:
        super().__init__(**kw)
        self._domain_id = domain_id
        self._color_topic = color_topic or dds_topic_name(TOPICS.camera_color, typed=True)
        self._depth_topic = depth_topic or dds_topic_name(TOPICS.camera_depth, typed=True)
        self._info_topic = info_topic or dds_topic_name(TOPICS.camera_info, typed=True)
        self._shm_paths = {
            "color": _configured_shm_path(color_shm_path, "LINGTU_CAMERA_COLOR_SHM", _DEFAULT_COLOR_SHM),
            "depth": _configured_shm_path(depth_shm_path, "LINGTU_CAMERA_DEPTH_SHM", _DEFAULT_DEPTH_SHM),
            "info": _configured_shm_path(info_shm_path, "LINGTU_CAMERA_INFO_SHM", _DEFAULT_INFO_SHM),
        }
        self._stale_timeout_s = max(0.001, float(stale_timeout_s))
        self._poll_interval_s = max(0.001, float(poll_interval_s))
        self._readers: dict[str, ShmFrameReader] = {}
        self._thread: threading.Thread | None = None
        self._stop_event = threading.Event()
        self._last_color_ts = 0.0
        self._last_depth_ts = 0.0
        self._last_info_ts = 0.0
        self._frames = {"color": 0, "depth": 0, "info": 0}
        self._rejected = {"color": 0, "depth": 0, "info": 0}
        self._errors: dict[str, str] = {}
        self._running = False

    def setup(self) -> None:
        """Start the non-blocking mmap polling thread."""

        self._stop_event.clear()
        self._readers = {
            name: ShmFrameReader(path, max_age_s=self._stale_timeout_s) for name, path in self._shm_paths.items()
        }
        self._running = True
        self._thread = threading.Thread(
            target=self._read_loop,
            name="camera-shm-reader",
            daemon=True,
        )
        self._thread.start()
        self.alive.publish(True)

    def stop(self) -> None:
        """Stop polling and close all mapped SHM objects."""

        self._stop_event.set()
        thread = self._thread
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=1.0)
        self._thread = None
        self._running = False
        for reader in self._readers.values():
            reader.close()
        self._readers.clear()
        self.alive.publish(False)
        super().stop()

    def poll_once(self) -> int:
        """Read and publish every stream that has a new committed sequence."""

        published = 0
        for name, reader in self._readers.items():
            try:
                frame = reader.read_latest()
            except (ShmUnavailable, FrameNotReady) as exc:
                self._errors[name] = str(exc)
                continue
            except FrameChanged:
                self._rejected[name] += 1
                continue
            except (FrameStale, FrameCorrupt, UnicodeError, ValueError) as exc:
                self._rejected[name] += 1
                self._errors[name] = f"{type(exc).__name__}: {exc}"
                continue
            if frame is None:
                continue
            try:
                self._publish_frame(name, frame)
            except (ValueError, TypeError) as exc:
                self._rejected[name] += 1
                self._errors[name] = f"{type(exc).__name__}: {exc}"
                continue
            self._errors.pop(name, None)
            published += 1
        return published

    def health(self) -> dict[str, object]:
        """Return data-plane readiness and per-stream diagnostics."""

        info = cast(dict[str, object], super().port_summary())
        now = time.time()
        stale_ms = {
            "color": self._age_ms(now, self._last_color_ts),
            "depth": self._age_ms(now, self._last_depth_ts),
            "info": self._age_ms(now, self._last_info_ts),
        }
        fresh = all(value is not None and value <= int(self._stale_timeout_s * 1000) for value in stale_ms.values())
        ready = self._running and fresh and all(value > 0 for value in self._frames.values())
        info["role"] = CAMERA_ROLE
        info["backend"] = CAMERA_BACKEND_DDS
        info["status"] = "ready" if ready else "running" if self._running else "stopped"
        info["ready"] = ready
        info["frames"] = dict(self._frames)
        info["rejected_frames"] = dict(self._rejected)
        info["fps"] = None
        info["error"] = "; ".join(f"{name}: {value}" for name, value in sorted(self._errors.items())) or None
        info["transport"] = {
            "frame_data": "posix_shm",
            "metadata": "posix_shm_and_optional_typed_dds",
        }
        info["shm_schema"] = "lingtu.camera.shm_frame.v1"
        info["shm_paths"] = {name: str(path) for name, path in self._shm_paths.items()}
        info["topics"] = {
            "color": self._color_topic,
            "depth": self._depth_topic,
            "info": self._info_topic,
        }
        info["source_service"] = "camera"
        info["source_unit"] = "lingtu-camera-dds.service"
        info["stream_contract"] = {
            "color": "color_image",
            "depth": "depth_image",
            "info": "camera_info",
        }
        info["stale_ms"] = stale_ms
        return info

    def _read_loop(self) -> None:
        try:
            while not self._stop_event.is_set():
                self.poll_once()
                self._stop_event.wait(self._poll_interval_s)
        except Exception as exc:
            self._errors["reader"] = f"{type(exc).__name__}: {exc}"
            logger.exception("DdsCameraModule: SHM reader stopped unexpectedly")
        finally:
            self._running = False
            self.alive.publish(False)

    def _publish_frame(self, name: str, frame: FrameSnapshot) -> None:
        expected_kind = {
            "color": StreamKind.COLOR,
            "depth": StreamKind.DEPTH,
            "info": StreamKind.INFO,
        }[name]
        if frame.stream_kind is not expected_kind:
            raise ValueError(f"{name} SHM carries {frame.stream_kind.name.lower()} frames")
        received_s = time.time()
        if name == "color":
            self.color_image.publish(self._decode_image(frame))
            self._last_color_ts = received_s
        elif name == "depth":
            self.depth_image.publish(self._decode_image(frame))
            self._last_depth_ts = received_s
        else:
            self.camera_info.publish(
                CameraIntrinsics(
                    fx=frame.fx,
                    fy=frame.fy,
                    cx=frame.cx,
                    cy=frame.cy,
                    width=frame.width,
                    height=frame.height,
                    depth_scale=frame.depth_scale or 0.001,
                    dist_k1=frame.dist_k1,
                    dist_k2=frame.dist_k2,
                    dist_p1=frame.dist_p1,
                    dist_p2=frame.dist_p2,
                    dist_k3=frame.dist_k3,
                    ts=frame.timestamp_s,
                    frame_id=frame.frame_id,
                )
            )
            self._last_info_ts = received_s
        self._frames[name] += 1

    @staticmethod
    def _decode_image(frame: FrameSnapshot) -> Image:
        formats = {
            "rgb8": (np.uint8, 3, ImageFormat.RGB),
            "bgr8": (np.uint8, 3, ImageFormat.BGR),
            "rgba8": (np.uint8, 4, ImageFormat.RGBA),
            "mono8": (np.uint8, 1, ImageFormat.GRAY),
            "8UC1": (np.uint8, 1, ImageFormat.GRAY),
            "16UC1": (np.uint16, 1, ImageFormat.DEPTH_U16),
            "32FC1": (np.float32, 1, ImageFormat.DEPTH_F32),
        }
        try:
            dtype, channels, image_format = formats[frame.encoding]
        except KeyError as exc:
            raise ValueError(f"unsupported SHM image encoding: {frame.encoding}") from exc
        item_size = np.dtype(dtype).itemsize
        if frame.stride % item_size:
            raise ValueError("SHM image stride is not aligned to its element size")
        row_elements = frame.stride // item_size
        visible_elements = frame.width * channels
        rows = np.frombuffer(frame.payload, dtype=dtype).reshape(frame.height, row_elements)
        visible = rows[:, :visible_elements]
        data = (
            visible.reshape(frame.height, frame.width, channels)
            if channels > 1
            else visible.reshape(frame.height, frame.width)
        )
        return Image(
            data=data.copy(),
            format=image_format,
            ts=frame.timestamp_s,
            frame_id=frame.frame_id,
        )

    @staticmethod
    def _age_ms(now: float, timestamp: float) -> int | None:
        if timestamp <= 0.0:
            return None
        return int(max(0.0, now - timestamp) * 1000)


ShmCameraModule = DdsCameraModule
CameraModule = DdsCameraModule
