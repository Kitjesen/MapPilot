"""Livox SDK2 source that feeds LingTu's LiDAR frame stream directly."""

from __future__ import annotations

import logging
import os
import struct
import subprocess
import sys
import threading
import time
from collections.abc import Callable
from copy import deepcopy
from pathlib import Path
from typing import BinaryIO

from runtime.msgs.geometry import Quaternion, Vector3
from runtime.msgs.numpy_compat import np
from runtime.msgs.sensor import Imu
from runtime.runtime_interface import real_lidar_frame_id
from runtime.utils.livox_config import ensure_mid360_config_file

from .frames import POINT_DTYPE, LivoxPointFrame
from .frame_stream import LidarFrameStream
from .lidar import LidarHealth, LidarState

logger = logging.getLogger(__name__)

_REPO_ROOT = Path(__file__).resolve().parents[4]
_STREAM_BIN_ENV = "LINGTU_LIVOX_SDK2_STREAM_BIN"
_STREAM_BUILD_HINT = "scripts/build/build_livox_sdk2_stream.sh"
_MAGIC = b"LTU1"
_RECORD_CLOUD = 1
_RECORD_IMU = 2
_HEADER = struct.Struct("<4sB3xQIII")
_IMU_PAYLOAD = struct.Struct("<ffffff")


class Sdk2Source:
    """Native Livox SDK2 process source.

    The official SDK owns device discovery, UDP/TCP, commands, work mode and
    callbacks. This class only starts the thin SDK2 stream executable and
    converts its records into LingTu runtime messages.
    """

    def __init__(
        self,
        *,
        ip: str | None = None,
        config_path: str | None = None,
        executable: str | None = None,
    ) -> None:
        self._ip = ip
        self._config_path = config_path
        self._executable = executable
        self._state = LidarState.DISCONNECTED
        self._state_lock = threading.Lock()
        self._frames = LidarFrameStream()
        self._health = LidarHealth()
        self._connect_time = 0.0
        self._process: subprocess.Popen[bytes] | None = None
        self._reader_thread: threading.Thread | None = None
        self._stderr_thread: threading.Thread | None = None
        self._stop = threading.Event()

    def on_cloud(self, callback: Callable[[np.ndarray], None]) -> Sdk2Source:
        self._frames.on_cloud(callback)
        return self

    def on_raw_cloud(self, callback: Callable[[LivoxPointFrame], None]) -> Sdk2Source:
        self._frames.on_raw_cloud(callback)
        return self

    def on_imu(self, callback: Callable[[Imu], None]) -> Sdk2Source:
        self._frames.on_imu(callback)
        return self

    def connect(self, ip: str | None = None) -> Sdk2Source:
        if self._state == LidarState.CONNECTED:
            return self
        if ip:
            self._ip = ip
        self._set_state(LidarState.CONNECTING)
        self._stop.clear()
        try:
            config_path = self._resolve_config_path()
            executable = self._resolve_executable()
            command = [str(executable), str(config_path)]
            if executable.suffix.lower() == ".py":
                command = [sys.executable, str(executable), str(config_path)]
            self._process = subprocess.Popen(
                command,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
            )
            self._reader_thread = threading.Thread(
                target=self._read_loop,
                name="livox-sdk2-stream-reader",
                daemon=True,
            )
            self._reader_thread.start()
            self._stderr_thread = threading.Thread(
                target=self._stderr_loop,
                name="livox-sdk2-stream-stderr",
                daemon=True,
            )
            self._stderr_thread.start()
            self._connect_time = time.monotonic()
            self._health.ip = self._ip or ""
            self._set_state(LidarState.CONNECTED)
        except Exception as exc:
            self._health.last_error = str(exc)
            self._set_state(LidarState.ERROR)
            self.disconnect()
            raise
        return self

    def disconnect(self) -> None:
        self._stop.set()
        proc = self._process
        self._process = None
        if proc is not None and proc.poll() is None:
            proc.terminate()
            try:
                proc.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait(timeout=2.0)
        self._set_state(LidarState.DISCONNECTED)
        self._health.driver_pid = None

    @property
    def health(self) -> LidarHealth:
        h = self._health
        h.state = self._state
        h.fps = self._frames.fps
        metrics = self._frames.metrics
        h.total_frames = metrics.total_frames
        h.total_points = metrics.total_points
        h.last_frame_points = metrics.last_frame_points
        h.last_frame_time = metrics.last_frame_time
        if self._connect_time > 0 and self._state == LidarState.CONNECTED:
            h.uptime_s = time.monotonic() - self._connect_time
        proc = self._process
        if proc is not None:
            h.driver_pid = proc.pid if proc.poll() is None else None
        return h

    @property
    def ip(self) -> str | None:
        return self._ip

    @property
    def state(self) -> LidarState:
        return self._state

    def _resolve_config_path(self) -> Path:
        if self._config_path:
            return Path(self._config_path).expanduser().resolve()

        from runtime.config import get_config

        cfg = get_config()
        if self._ip and self._ip != cfg.lidar.lidar_ip:
            cfg = deepcopy(cfg)
            cfg.lidar.lidar_ip = self._ip
        path = ensure_mid360_config_file(cfg)
        return Path(path).expanduser().resolve()

    def _resolve_executable(self) -> Path:
        candidates: list[Path] = []
        if self._executable:
            candidates.append(Path(self._executable).expanduser())
        env_path = os.environ.get(_STREAM_BIN_ENV)
        if env_path:
            candidates.append(Path(env_path).expanduser())
        candidates.extend(
            [
                _REPO_ROOT / "build" / "livox_sdk2_stream" / "livox_sdk2_stream",
                _REPO_ROOT / "build" / "livox_sdk2_stream" / "Debug" / "livox_sdk2_stream.exe",
                _REPO_ROOT / "build" / "livox_sdk2_stream" / "Release" / "livox_sdk2_stream.exe",
            ]
        )
        for candidate in candidates:
            if candidate.exists():
                return candidate.resolve()
        raise FileNotFoundError(
            "livox_sdk2_stream binary not found. "
            f"Build it with {_STREAM_BUILD_HINT} or set {_STREAM_BIN_ENV}."
        )

    def _read_loop(self) -> None:
        stream = self._process.stdout if self._process else None
        if stream is None:
            return
        try:
            while not self._stop.is_set():
                header = _read_exact(stream, _HEADER.size)
                if not header:
                    break
                if len(header) != _HEADER.size:
                    raise EOFError("truncated livox sdk2 stream header")
                record = _parse_record(stream, header)
                if isinstance(record, LivoxPointFrame):
                    self._frames.ingest_point_frame(record)
                elif isinstance(record, Imu):
                    self._frames.ingest_imu(record)
        except Exception as exc:
            if not self._stop.is_set():
                self._health.last_error = str(exc)
                self._set_state(LidarState.ERROR)
                logger.error("Livox SDK2 read loop failed: %s", exc)

    def _stderr_loop(self) -> None:
        stream = self._process.stderr if self._process else None
        if stream is None:
            return
        while not self._stop.is_set():
            line = stream.readline()
            if not line:
                break
            logger.info("livox_sdk2_stream: %s", line.decode(errors="replace").strip())

    def _set_state(self, new: LidarState) -> None:
        with self._state_lock:
            self._state = new


def _read_exact(stream: BinaryIO, n: int) -> bytes:
    chunks = bytearray()
    while len(chunks) < n:
        chunk = stream.read(n - len(chunks))
        if not chunk:
            return bytes(chunks)
        chunks.extend(chunk)
    return bytes(chunks)


def _parse_record(stream: BinaryIO, header: bytes):
    magic, record_type, timestamp_ns, sequence, count, payload_bytes = _HEADER.unpack(header)
    if magic != _MAGIC:
        raise ValueError("bad livox sdk2 stream magic")
    payload = _read_exact(stream, payload_bytes)
    if len(payload) != payload_bytes:
        raise EOFError("truncated livox sdk2 stream payload")
    if record_type == _RECORD_CLOUD:
        if payload_bytes != count * POINT_DTYPE.itemsize:
            raise ValueError("bad livox point payload size")
        points = np.frombuffer(payload, dtype=POINT_DTYPE, count=count).copy()
        return LivoxPointFrame(
            points=points,
            timestamp_ns=int(timestamp_ns),
            sequence=int(sequence),
        )
    if record_type == _RECORD_IMU:
        if payload_bytes != _IMU_PAYLOAD.size:
            raise ValueError("bad livox imu payload size")
        gx, gy, gz, ax, ay, az = _IMU_PAYLOAD.unpack(payload)
        return Imu(
            orientation=Quaternion.identity(),
            angular_velocity=Vector3(gx, gy, gz),
            linear_acceleration=Vector3(ax, ay, az),
            ts=int(timestamp_ns) * 1e-9,
            frame_id=real_lidar_frame_id(),
        )
    return None


__all__ = ["Sdk2Source"]
