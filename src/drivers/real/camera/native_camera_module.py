"""Orbbec native camera bridge.

The C++ process talks to Orbbec SDK over the device driver and writes local
binary records over stdout. This Module is the LingTu dataflow boundary: it
publishes those records as color_image, depth_image, and camera_info ports.
"""

from __future__ import annotations

import logging
import os
import platform
import struct
import subprocess
import threading
import time
from pathlib import Path
from typing import Any, BinaryIO

from runtime.module import Module
from runtime.msgs.numpy_compat import np
from runtime.msgs.sensor import CameraIntrinsics, Image, ImageFormat
from runtime.registry import register
from runtime.stream import Out

logger = logging.getLogger(__name__)

_HEADER = struct.Struct("<4sHHIIIIddddddI")
_MAGIC = b"LTOB"
_VERSION = 1
_KIND_INTRINSICS = 1
_KIND_COLOR = 2
_KIND_DEPTH = 3
_FMT_RGB8 = 1
_FMT_BGR8 = 2
_FMT_DEPTH_U16 = 3


def _repo_root() -> Path:
    return Path(__file__).resolve().parents[4]


def _path_from_env(env_name: str, fallback: Path) -> Path:
    value = os.environ.get(env_name, "").strip()
    path = Path(value) if value else fallback
    if not path.is_absolute():
        path = _repo_root() / path
    return path


def _sdk_arch() -> str:
    machine = platform.machine().lower()
    if machine in {"aarch64", "arm64"}:
        return "arm64"
    return "x64"


def orbbec_sdk_root() -> Path:
    ros2_root = _path_from_env(
        "LINGTU_ORBBEC_ROS2_DIR",
        _repo_root() / "src" / "drivers" / "real" / "camera" / "OrbbecSDK_ROS2",
    )
    return ros2_root / "orbbec_camera" / "SDK"


def orbbec_native_build_dir() -> Path:
    return _path_from_env(
        "LINGTU_ORBBEC_NATIVE_BUILD_DIR",
        _repo_root() / "build" / "orbbec_native",
    )


def _default_executable() -> Path:
    name = "orbbec_capture.exe" if os.name == "nt" else "orbbec_capture"
    return orbbec_native_build_dir() / name


def _runtime_library_paths() -> tuple[Path, ...]:
    sdk_lib = orbbec_sdk_root() / "lib" / _sdk_arch()
    return (
        orbbec_native_build_dir() / "lib",
        sdk_lib,
        sdk_lib / "extensions",
    )


def _with_runtime_library_env() -> dict[str, str]:
    env = dict(os.environ)
    if os.name == "nt":
        key = "PATH"
    else:
        key = "LD_LIBRARY_PATH"
    existing = env.get(key, "")
    paths = [str(path) for path in _runtime_library_paths() if path.exists()]
    if paths:
        env[key] = os.pathsep.join([*paths, existing] if existing else paths)
    env.setdefault("ORBBEC_SDK_ROOT", str(orbbec_sdk_root()))
    return env


def _default_camera_config() -> dict[str, Any]:
    path = _repo_root() / "config" / "devices.yaml"
    try:
        import yaml

        data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    except Exception:
        return {}
    for item in data.get("devices", []):
        if item.get("type") != "camera" or item.get("enabled") is False:
            continue
        if item.get("driver") in {"orbbec_native", "orbbec_sdk"}:
            return dict(item.get("config") or {})
    return {}


def _read_exact(stream: BinaryIO, size: int) -> bytes | None:
    chunks = bytearray()
    while len(chunks) < size:
        chunk = stream.read(size - len(chunks))
        if not chunk:
            return None
        chunks.extend(chunk)
    return bytes(chunks)


@register("camera_bridge", "default", description="Native Orbbec SDK camera stream")
class OrbbecNativeCameraModule(Module, layer=1):
    """Publish Orbbec RGB-D frames without ROS2."""

    color_image: Out[Image]
    depth_image: Out[Image]
    camera_info: Out[CameraIntrinsics]
    alive: Out[bool]

    def __init__(
        self,
        executable: str | None = None,
        width: int | None = None,
        height: int | None = None,
        fps: int | None = None,
        color_width: int | None = None,
        color_height: int | None = None,
        color_fps: int | None = None,
        depth_width: int | None = None,
        depth_height: int | None = None,
        depth_fps: int | None = None,
        serial_number: str | None = None,
        uid: str | None = None,
        usb_port: str | None = None,
        product_id: int | str | None = None,
        device_index: int | None = None,
        connect_timeout_ms: int | None = None,
        sdk_config_path: str | None = None,
        enable_frame_sync: bool | None = None,
        rotate: int | None = None,
        frame_id: str | None = None,
        **kw,
    ) -> None:
        super().__init__(**kw)
        config = _default_camera_config()
        color = config.get("color") if isinstance(config.get("color"), dict) else {}
        depth = config.get("depth") if isinstance(config.get("depth"), dict) else {}
        base_width = int(width if width is not None else config.get("width", 640))
        base_height = int(height if height is not None else config.get("height", 480))
        base_fps = int(fps if fps is not None else config.get("fps", 30))
        env_exe = os.environ.get("LINGTU_ORBBEC_NATIVE_EXECUTABLE", "").strip()
        self._executable = Path(executable or env_exe or _default_executable())
        self._color_width = int(color_width if color_width is not None else color.get("width", base_width))
        self._color_height = int(color_height if color_height is not None else color.get("height", base_height))
        self._color_fps = int(color_fps if color_fps is not None else color.get("fps", base_fps))
        self._depth_width = int(depth_width if depth_width is not None else depth.get("width", base_width))
        self._depth_height = int(depth_height if depth_height is not None else depth.get("height", base_height))
        self._depth_fps = int(depth_fps if depth_fps is not None else depth.get("fps", base_fps))
        self._serial_number = str(
            serial_number if serial_number is not None else config.get("serial_number", "")
        ).strip()
        self._uid = str(
            uid if uid is not None else usb_port if usb_port is not None else config.get("uid", "")
        ).strip()
        raw_product_id = product_id if product_id is not None else config.get("product_id", 0)
        self._product_id = int(str(raw_product_id), 0) if str(raw_product_id).strip() else 0
        raw_device_index = device_index if device_index is not None else config.get("device_index")
        self._device_index = None if raw_device_index is None else int(raw_device_index)
        self._connect_timeout_ms = int(
            connect_timeout_ms if connect_timeout_ms is not None else config.get("connect_timeout_ms", 10000)
        )
        self._sdk_config_path = str(
            sdk_config_path if sdk_config_path is not None else config.get("sdk_config_path", "")
        ).strip()
        self._enable_frame_sync = bool(
            enable_frame_sync if enable_frame_sync is not None else config.get("enable_frame_sync", False)
        )
        self._rotate = int(rotate if rotate is not None else config.get("rotate", 0))
        self._frame_id = str(frame_id or config.get("frame_id", "camera_link"))
        self._proc: subprocess.Popen[bytes] | None = None
        self._reader: threading.Thread | None = None
        self._stop_event = threading.Event()
        self._last_color_ts = 0.0
        self._last_depth_ts = 0.0
        self._last_info: CameraIntrinsics | None = None
        self._backend = "native_orbbec"
        self._error: str | None = None
        self._stderr_reader: threading.Thread | None = None
        self._stderr_tail = ""
        self._stderr_lock = threading.Lock()

    def setup(self) -> None:
        if not self._executable.exists():
            self._backend = "stub"
            self._error = f"native Orbbec executable not found: {self._executable}"
            logger.info("OrbbecNativeCameraModule: %s", self._error)
            return

        cmd = [
            str(self._executable),
            "--color-width",
            str(self._color_width),
            "--color-height",
            str(self._color_height),
            "--color-fps",
            str(self._color_fps),
            "--depth-width",
            str(self._depth_width),
            "--depth-height",
            str(self._depth_height),
            "--depth-fps",
            str(self._depth_fps),
            "--connect-timeout-ms",
            str(self._connect_timeout_ms),
        ]
        if self._serial_number:
            cmd.extend(["--serial-number", self._serial_number])
        if self._uid:
            cmd.extend(["--uid", self._uid])
        if self._product_id:
            cmd.extend(["--product-id", str(self._product_id)])
        if self._device_index is not None:
            cmd.extend(["--device-index", str(self._device_index)])
        if self._sdk_config_path:
            cmd.extend(["--sdk-config", self._sdk_config_path])
        if self._enable_frame_sync:
            cmd.append("--enable-frame-sync")
        try:
            self._proc = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                env=_with_runtime_library_env(),
            )
        except OSError as exc:
            self._backend = "stub"
            self._error = str(exc)
            logger.warning("OrbbecNativeCameraModule: failed to start: %s", exc)
            return

        self._reader = threading.Thread(
            target=self._read_loop,
            name="orbbec-native-camera",
            daemon=True,
        )
        self._reader.start()
        if self._proc.stderr is not None:
            self._stderr_reader = threading.Thread(
                target=self._read_stderr_loop,
                name="orbbec-native-camera-stderr",
                daemon=True,
            )
            self._stderr_reader.start()
        self.alive.publish(True)

    def stop(self) -> None:
        self._stop_event.set()
        proc = self._proc
        if proc is not None and proc.poll() is None:
            proc.terminate()
            try:
                proc.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                proc.kill()
        self._proc = None
        super().stop()

    def health(self) -> dict[str, object]:
        info = super().port_summary()
        info["backend"] = self._backend
        info["error"] = self._error
        info["native_executable"] = str(self._executable)
        info["sdk_root"] = str(orbbec_sdk_root())
        info["sdk_root_exists"] = orbbec_sdk_root().exists()
        info["runtime_library_paths"] = [str(path) for path in _runtime_library_paths()]
        info["camera_info_topics"] = ["native_orbbec"]
        info["camera_info_preferred_topic"] = "native_orbbec"
        info["camera_info_active_topic"] = "native_orbbec" if self._last_info else None
        info["reconnect_count"] = 0
        info["service_recovery_allowed"] = False
        info["service_recovery_suppressed"] = True
        with self._stderr_lock:
            if self._stderr_tail:
                info["native_stderr_tail"] = self._stderr_tail
        now = time.time()
        if self._last_color_ts:
            dt = now - self._last_color_ts
            info["fps"] = round(1.0 / dt, 1) if dt > 0 else 0.0
        else:
            info["fps"] = 0.0
        return info

    def _read_loop(self) -> None:
        assert self._proc is not None and self._proc.stdout is not None
        stream = self._proc.stdout
        while not self._stop_event.is_set():
            header = _read_exact(stream, _HEADER.size)
            if header is None:
                break
            try:
                record = self._decode_record(header, stream)
            except Exception as exc:
                self._error = str(exc)
                logger.debug("OrbbecNativeCameraModule: bad record: %s", exc)
                break
            self._publish_record(record)
        proc = self._proc
        if proc is not None and proc.poll() not in (None, 0):
            with self._stderr_lock:
                detail = self._stderr_tail
            self._error = detail or f"native Orbbec process exited with {proc.returncode}"
        self.alive.publish(False)

    def _read_stderr_loop(self) -> None:
        assert self._proc is not None and self._proc.stderr is not None
        for raw_line in self._proc.stderr:
            text = raw_line.decode("utf-8", errors="replace").strip()
            if not text:
                continue
            with self._stderr_lock:
                self._stderr_tail = text[-500:]

    def _decode_record(self, header: bytes, stream: BinaryIO) -> dict[str, object]:
        (
            magic,
            version,
            kind,
            width,
            height,
            channels,
            fmt,
            ts,
            fx,
            fy,
            cx,
            cy,
            depth_scale,
            payload_size,
        ) = _HEADER.unpack(header)
        if magic != _MAGIC or version != _VERSION:
            raise ValueError("invalid Orbbec native stream header")
        payload = _read_exact(stream, payload_size)
        if payload is None:
            raise EOFError("truncated Orbbec native payload")
        return {
            "kind": kind,
            "width": width,
            "height": height,
            "channels": channels,
            "format": fmt,
            "ts": ts,
            "fx": fx,
            "fy": fy,
            "cx": cx,
            "cy": cy,
            "depth_scale": depth_scale,
            "payload": payload,
        }

    def _publish_record(self, record: dict[str, object]) -> None:
        kind = int(record["kind"])
        if kind == _KIND_INTRINSICS:
            self._publish_intrinsics(record)
        elif kind == _KIND_COLOR:
            self._publish_color(record)
        elif kind == _KIND_DEPTH:
            self._publish_depth(record)

    def _publish_intrinsics(self, record: dict[str, object]) -> None:
        intrinsics = CameraIntrinsics(
            fx=float(record["fx"]),
            fy=float(record["fy"]),
            cx=float(record["cx"]),
            cy=float(record["cy"]),
            width=int(record["width"]),
            height=int(record["height"]),
            depth_scale=float(record["depth_scale"]) or 0.001,
        )
        if self._rotate:
            intrinsics = self._rotate_intrinsics(intrinsics)
        self._last_info = intrinsics
        self.camera_info.publish(intrinsics)

    def _publish_color(self, record: dict[str, object]) -> None:
        width = int(record["width"])
        height = int(record["height"])
        channels = int(record["channels"]) or 3
        arr = np.frombuffer(record["payload"], dtype=np.uint8).reshape(
            height,
            width,
            channels,
        )
        fmt = int(record["format"])
        if fmt == _FMT_RGB8:
            arr = arr[..., ::-1]
        elif fmt != _FMT_BGR8:
            return
        arr = self._rotate_array(arr)
        self._last_color_ts = time.time()
        self.color_image.publish(
            Image(
                data=arr.copy(),
                format=ImageFormat.BGR,
                ts=float(record["ts"]) or time.time(),
                frame_id=self._frame_id,
            )
        )

    def _publish_depth(self, record: dict[str, object]) -> None:
        if int(record["format"]) != _FMT_DEPTH_U16:
            return
        width = int(record["width"])
        height = int(record["height"])
        arr = np.frombuffer(record["payload"], dtype=np.uint16).reshape(height, width)
        arr = self._rotate_array(arr)
        self._last_depth_ts = time.time()
        self.depth_image.publish(
            Image(
                data=arr.copy(),
                format=ImageFormat.DEPTH_U16,
                ts=float(record["ts"]) or time.time(),
                frame_id=self._frame_id,
            )
        )

    def _rotate_array(self, arr):
        if self._rotate == 90:
            return np.rot90(arr, k=3)
        if self._rotate == 180:
            return np.rot90(arr, k=2)
        if self._rotate == 270:
            return np.rot90(arr, k=1)
        return arr

    def _rotate_intrinsics(self, intrinsics: CameraIntrinsics) -> CameraIntrinsics:
        w, h = intrinsics.width, intrinsics.height
        fx, fy = intrinsics.fx, intrinsics.fy
        cx, cy = intrinsics.cx, intrinsics.cy
        if self._rotate == 90:
            return CameraIntrinsics(
                fx=fy,
                fy=fx,
                cx=float(h - 1 - cy),
                cy=float(cx),
                width=h,
                height=w,
                depth_scale=intrinsics.depth_scale,
            )
        if self._rotate == 180:
            return CameraIntrinsics(
                fx=fx,
                fy=fy,
                cx=float(w - 1 - cx),
                cy=float(h - 1 - cy),
                width=w,
                height=h,
                depth_scale=intrinsics.depth_scale,
            )
        if self._rotate == 270:
            return CameraIntrinsics(
                fx=fy,
                fy=fx,
                cx=float(cy),
                cy=float(w - cx),
                width=h,
                height=w,
                depth_scale=intrinsics.depth_scale,
            )
        return intrinsics
