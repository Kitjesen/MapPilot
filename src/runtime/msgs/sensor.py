"""lingtu.runtime.msgs.sensor — sensor data types for perception and SLAM modules.

Classes
-------
ImageFormat   — image pixel format enum
Image         — NumPy image container (BGR/RGB/RGBA/GRAY/Depth)
CameraIntrinsics — camera intrinsics (fx, fy, cx, cy)
PointField    — PointCloud2 field descriptor
PointCloud2   — N×3 / N×4 point cloud (pure NumPy, no Open3D, ROS2-compatible metadata)
Imu           — inertial measurement unit data
LivoxPointFrame — decoded Livox frame shared by real and simulated sources
"""

from __future__ import annotations

import os
import struct
import time
from dataclasses import dataclass, field
from enum import Enum
from pathlib import Path
from typing import Any, ClassVar

from runtime.runtime_interface import camera_frame_id, map_frame_id

from .geometry import Quaternion, Vector3
from .numpy_compat import is_numpy_array, np, numpy_import_is_safe

SENSOR_CAMERA_FRAME_ID = camera_frame_id()
SENSOR_MAP_FRAME_ID = map_frame_id()

POINT_DTYPE = np.dtype(
    [
        ("x", "<f4"),
        ("y", "<f4"),
        ("z", "<f4"),
        ("intensity", "<f4"),
        ("offset_time_ns", "<u4"),
        ("tag", "u1"),
        ("line", "u1"),
        ("flags", "<u2"),
    ]
)


@dataclass
class LivoxPointFrame:
    """Lossless frame decoded from official Livox driver output."""

    points: Any
    timestamp_ns: int = 0
    sequence: int = 0

    def __post_init__(self) -> None:
        self.points = np.asarray(self.points, dtype=POINT_DTYPE)

    @property
    def point_count(self) -> int:
        return int(self.points.shape[0])

    def to_xyzi(self) -> Any:
        out = np.empty((self.point_count, 4), dtype=np.float32)
        out[:, 0] = self.points["x"]
        out[:, 1] = self.points["y"]
        out[:, 2] = self.points["z"]
        out[:, 3] = self.points["intensity"]
        return out


@dataclass
class RosTime:
    sec: int = 0
    nanosec: int = 0

    @classmethod
    def from_seconds(cls, value: float) -> RosTime:
        sec = int(value)
        nanosec = int(round((float(value) - sec) * 1_000_000_000))
        if nanosec >= 1_000_000_000:
            sec += 1
            nanosec -= 1_000_000_000
        return cls(sec=sec, nanosec=nanosec)

    def to_seconds(self) -> float:
        return float(self.sec) + float(self.nanosec) * 1e-9

    def to_dict(self) -> dict[str, int]:
        return {"sec": int(self.sec), "nanosec": int(self.nanosec)}


@dataclass
class Header:
    stamp: RosTime = field(default_factory=RosTime)
    frame_id: str = ""

    @classmethod
    def from_stamp(cls, ts: float, frame_id: str) -> Header:
        return cls(stamp=RosTime.from_seconds(ts), frame_id=frame_id)

    def to_dict(self) -> dict[str, Any]:
        return {"stamp": self.stamp.to_dict(), "frame_id": self.frame_id}

# ---------------------------------------------------------------------------
# ImageFormat
# ---------------------------------------------------------------------------


class ImageFormat(Enum):
    """Pixel encoding formats."""

    BGR = "BGR"
    RGB = "RGB"
    RGBA = "RGBA"
    GRAY = "GRAY"
    DEPTH_F32 = "DEPTH_F32"  # float32 metres
    DEPTH_U16 = "DEPTH_U16"  # uint16 millimetres


# ---------------------------------------------------------------------------
# Image
# ---------------------------------------------------------------------------

# Header layout for encode(): format_len(I) + format_str + ndim(I) + shape(I*ndim) + dtype_len(I) + dtype_str
_IMG_HEADER = struct.Struct("<I")  # reused for individual uint32 fields
_ROS_IMAGE_ENCODING = {
    ImageFormat.BGR: "bgr8",
    ImageFormat.RGB: "rgb8",
    ImageFormat.RGBA: "rgba8",
    ImageFormat.GRAY: "mono8",
    ImageFormat.DEPTH_F32: "32FC1",
    ImageFormat.DEPTH_U16: "16UC1",
}


@dataclass
class Image:
    """NumPy-backed image with format metadata.

    Parameters
    ----------
    data : np.ndarray
        Pixel data — shape ``(H, W)`` for grayscale/depth, ``(H, W, C)`` for colour.
    format : ImageFormat
        Pixel encoding.
    ts : float
        Timestamp (seconds since epoch).
    frame_id : str
        TF frame this image was captured in.
    """

    msg_name: ClassVar[str] = "sensor_msgs.Image"
    data: Any = field(default_factory=lambda: [[[0, 0, 0]]])
    format: ImageFormat = field(default=ImageFormat.BGR)
    ts: float = field(default_factory=time.time)
    frame_id: str = field(default=SENSOR_CAMERA_FRAME_ID)
    is_bigendian: bool = False

    def __post_init__(self) -> None:
        if not is_numpy_array(self.data):
            if not numpy_import_is_safe():
                return
            self.data = np.asarray(self.data)

    # -- factory methods -----------------------------------------------------

    @classmethod
    def from_numpy(cls, arr: np.ndarray, fmt: ImageFormat = ImageFormat.BGR,
                   frame_id: str = SENSOR_CAMERA_FRAME_ID) -> Image:
        """Wrap an existing array."""
        return cls(data=np.asarray(arr), format=fmt, frame_id=frame_id)

    @classmethod
    def from_file(cls, path: str | Path, fmt: ImageFormat | None = None,
                  frame_id: str = SENSOR_CAMERA_FRAME_ID) -> Image:
        """Load from an image file.  Requires *cv2* (optional dependency)."""
        try:
            import cv2  # type: ignore[import-untyped]
        except ImportError as exc:
            raise ImportError("cv2 is required for Image.from_file()") from exc

        img = cv2.imread(str(path), cv2.IMREAD_UNCHANGED)
        if img is None:
            raise FileNotFoundError(f"Cannot read image: {path}")

        if fmt is None:
            if img.ndim == 2:
                fmt = ImageFormat.GRAY
            elif img.shape[2] == 4:
                fmt = ImageFormat.RGBA
            else:
                fmt = ImageFormat.BGR  # OpenCV default
        return cls(data=img, format=fmt, frame_id=frame_id)

    # -- properties ----------------------------------------------------------

    @property
    def height(self) -> int:
        if is_numpy_array(self.data):
            return int(self.data.shape[0])
        return len(self.data) if isinstance(self.data, list) else 0

    @property
    def width(self) -> int:
        if is_numpy_array(self.data):
            return int(self.data.shape[1])
        if isinstance(self.data, list) and self.data:
            row = self.data[0]
            return len(row) if isinstance(row, list) else 0
        return 0

    @property
    def channels(self) -> int:
        if is_numpy_array(self.data):
            return 1 if self.data.ndim == 2 else int(self.data.shape[2])
        if isinstance(self.data, list) and self.data and isinstance(self.data[0], list):
            row = self.data[0]
            if row and isinstance(row[0], list):
                return len(row[0])
            return 1
        return 0

    @property
    def encoding(self) -> str:
        """ROS ``sensor_msgs/Image`` encoding name."""

        return _ROS_IMAGE_ENCODING[self.format]

    @property
    def step(self) -> int:
        """Full row length in bytes, matching ROS ``sensor_msgs/Image.step``."""

        if not is_numpy_array(self.data) or self.data.ndim < 2:
            return 0
        return int(self.data.strides[0])

    @property
    def header(self) -> Header:
        return Header.from_stamp(self.ts, self.frame_id)

    # -- format conversion ---------------------------------------------------

    def to_rgb(self) -> Image:
        """Convert to RGB format."""
        if self.format is ImageFormat.RGB:
            return self
        if self.format is ImageFormat.BGR:
            return Image(
                data=self.data[..., ::-1].copy(),
                format=ImageFormat.RGB,
                ts=self.ts,
                frame_id=self.frame_id,
                is_bigendian=self.is_bigendian,
            )
        if self.format is ImageFormat.RGBA:
            return Image(
                data=self.data[..., :3][..., ::-1].copy(),
                format=ImageFormat.RGB,
                ts=self.ts,
                frame_id=self.frame_id,
                is_bigendian=self.is_bigendian,
            )
        if self.format is ImageFormat.GRAY:
            return Image(
                data=np.stack([self.data] * 3, axis=-1),
                format=ImageFormat.RGB,
                ts=self.ts,
                frame_id=self.frame_id,
                is_bigendian=self.is_bigendian,
            )
        raise ValueError(f"Cannot convert {self.format} to RGB")

    def to_bgr(self) -> Image:
        """Convert to BGR format."""
        if self.format is ImageFormat.BGR:
            return self
        if self.format is ImageFormat.RGB:
            return Image(
                data=self.data[..., ::-1].copy(),
                format=ImageFormat.BGR,
                ts=self.ts,
                frame_id=self.frame_id,
                is_bigendian=self.is_bigendian,
            )
        if self.format is ImageFormat.RGBA:
            return Image(
                data=self.data[..., :3].copy(),
                format=ImageFormat.BGR,
                ts=self.ts,
                frame_id=self.frame_id,
                is_bigendian=self.is_bigendian,
            )
        if self.format is ImageFormat.GRAY:
            return Image(
                data=np.stack([self.data] * 3, axis=-1),
                format=ImageFormat.BGR,
                ts=self.ts,
                frame_id=self.frame_id,
                is_bigendian=self.is_bigendian,
            )
        raise ValueError(f"Cannot convert {self.format} to BGR")

    def to_grayscale(self) -> Image:
        """Convert to single-channel grayscale."""
        if self.format is ImageFormat.GRAY:
            return self
        if self.format in (ImageFormat.BGR, ImageFormat.RGB):
            # ITU-R BT.601 luma weights
            if self.format is ImageFormat.RGB:
                r, g, b = self.data[..., 0], self.data[..., 1], self.data[..., 2]
            else:
                b, g, r = self.data[..., 0], self.data[..., 1], self.data[..., 2]
            gray = (0.299 * r + 0.587 * g + 0.114 * b).astype(self.data.dtype)
            return Image(
                data=gray,
                format=ImageFormat.GRAY,
                ts=self.ts,
                frame_id=self.frame_id,
                is_bigendian=self.is_bigendian,
            )
        if self.format is ImageFormat.RGBA:
            r, g, b = self.data[..., 0], self.data[..., 1], self.data[..., 2]
            gray = (0.299 * r + 0.587 * g + 0.114 * b).astype(self.data.dtype)
            return Image(
                data=gray,
                format=ImageFormat.GRAY,
                ts=self.ts,
                frame_id=self.frame_id,
                is_bigendian=self.is_bigendian,
            )
        raise ValueError(f"Cannot convert {self.format} to grayscale")

    # -- spatial ops ---------------------------------------------------------

    def resize(self, w: int, h: int) -> Image:
        """Resize using nearest-neighbour (no cv2 dependency)."""
        try:
            import cv2  # type: ignore[import-untyped]
            resized = cv2.resize(self.data, (w, h))
        except ImportError:
            # Pure-numpy nearest-neighbour fallback
            row_idx = (np.arange(h) * self.height / h).astype(int)
            col_idx = (np.arange(w) * self.width / w).astype(int)
            resized = self.data[np.ix_(row_idx, col_idx)] if self.data.ndim == 2 \
                else self.data[np.ix_(row_idx, col_idx, np.arange(self.channels))]
        return Image(
            data=resized,
            format=self.format,
            ts=self.ts,
            frame_id=self.frame_id,
            is_bigendian=self.is_bigendian,
        )

    def crop(self, x: int, y: int, w: int, h: int) -> Image:
        """Crop region ``(x, y, w, h)`` — top-left origin."""
        cropped = self.data[y: y + h, x: x + w].copy()
        return Image(
            data=cropped,
            format=self.format,
            ts=self.ts,
            frame_id=self.frame_id,
            is_bigendian=self.is_bigendian,
        )

    # -- serialisation -------------------------------------------------------

    def encode(self) -> bytes:
        """Serialise to raw bytes:  header + pixel data.

        Header: format_len(u32) + format_str + ndim(u32)
                + shape(u32×ndim) + dtype_len(u32) + dtype_str + ts(f64) + frame_len(u32) + frame_str
        """
        fmt_bytes = self.format.value.encode()
        dtype_bytes = self.data.dtype.str.encode()
        frame_bytes = self.frame_id.encode()
        buf = bytearray()
        buf += struct.pack("<I", len(fmt_bytes)) + fmt_bytes
        buf += struct.pack("<I", self.data.ndim)
        for s in self.data.shape:
            buf += struct.pack("<I", s)
        buf += struct.pack("<I", len(dtype_bytes)) + dtype_bytes
        buf += struct.pack("<d", self.ts)
        buf += struct.pack("<I", len(frame_bytes)) + frame_bytes
        buf += self.data.tobytes()
        return bytes(buf)

    @classmethod
    def decode(cls, raw: bytes) -> Image:
        """Reconstruct from bytes produced by :meth:`encode`."""
        off = 0

        def _read_u32() -> int:
            nonlocal off
            val = struct.unpack_from("<I", raw, off)[0]
            off += 4
            return val

        fmt_len = _read_u32()
        fmt_str = raw[off: off + fmt_len].decode()
        off += fmt_len
        ndim = _read_u32()
        shape = tuple(_read_u32() for _ in range(ndim))
        dtype_len = _read_u32()
        dtype_str = raw[off: off + dtype_len].decode()
        off += dtype_len
        ts = struct.unpack_from("<d", raw, off)[0]
        off += 8
        frame_len = _read_u32()
        frame_id = raw[off: off + frame_len].decode()
        off += frame_len
        data = np.frombuffer(raw, dtype=np.dtype(dtype_str), offset=off).reshape(shape)
        return cls(data=data.copy(), format=ImageFormat(fmt_str), ts=ts, frame_id=frame_id)

    # -- introspection -------------------------------------------------------

    def to_dict(self) -> dict[str, Any]:
        """Metadata dict (no pixel data)."""
        return {
            "header": self.header.to_dict(),
            "format": self.format.value,
            "encoding": self.encoding,
            "height": self.height,
            "width": self.width,
            "channels": self.channels,
            "is_bigendian": int(self.is_bigendian),
            "step": self.step,
            "dtype": self.data.dtype.str,
            "ts": self.ts,
            "frame_id": self.frame_id,
        }

    def to_ros_dict(self) -> dict[str, Any]:
        return {
            "header": self.header.to_dict(),
            "height": self.height,
            "width": self.width,
            "encoding": self.encoding,
            "is_bigendian": int(self.is_bigendian),
            "step": self.step,
            "data": self.data.tobytes() if is_numpy_array(self.data) else bytes(self.data),
        }

    def __repr__(self) -> str:
        return (f"Image({self.width}x{self.height}, {self.format.value}, "
                f"dtype={self.data.dtype}, frame='{self.frame_id}')")


# ---------------------------------------------------------------------------
# CameraIntrinsics
# ---------------------------------------------------------------------------

_INTRINSICS_FMT = struct.Struct("<6d2I")  # fx,fy,cx,cy,depth_scale, _pad, W, H  → 56 bytes
# Actually: fx(d) fy(d) cx(d) cy(d) depth_scale(d) width(I) height(I) → 48 bytes
_INTRINSICS_FMT = struct.Struct("<5dII")  # 5×8 + 2×4 = 48 bytes


@dataclass
class CameraIntrinsics:
    """Pinhole camera intrinsic parameters.

    Parameters
    ----------
    fx, fy : float
        Focal lengths in pixels.
    cx, cy : float
        Principal point in pixels.
    width, height : int
        Image dimensions.
    depth_scale : float
        Multiplier to convert raw depth values to metres (e.g. 0.001 for mm).
    """

    msg_name: ClassVar[str] = "sensor_msgs.CameraInfo"
    fx: float = 0.0
    fy: float = 0.0
    cx: float = 0.0
    cy: float = 0.0
    width: int = 0
    height: int = 0
    depth_scale: float = 1.0
    # Distortion coefficients (Brown-Conrady / plumb_bob model)
    dist_k1: float = 0.0
    dist_k2: float = 0.0
    dist_p1: float = 0.0
    dist_p2: float = 0.0
    dist_k3: float = 0.0
    ts: float = field(default_factory=time.time)
    frame_id: str = field(default=SENSOR_CAMERA_FRAME_ID)

    # -- properties ----------------------------------------------------------

    @property
    def D_vector(self) -> np.ndarray:
        """5-element distortion vector ``[k1, k2, p1, p2, k3]`` (OpenCV order)."""
        return np.array([self.dist_k1, self.dist_k2, self.dist_p1,
                         self.dist_p2, self.dist_k3], dtype=np.float64)

    @property
    def has_distortion(self) -> bool:
        """True if any distortion coefficient is non-zero."""
        return any(abs(v) > 1e-12 for v in
                   [self.dist_k1, self.dist_k2, self.dist_p1,
                    self.dist_p2, self.dist_k3])

    @property
    def K_matrix(self) -> np.ndarray:
        """3×3 intrinsic matrix ``K``."""
        return np.array([
            [self.fx, 0.0, self.cx],
            [0.0, self.fy, self.cy],
            [0.0, 0.0, 1.0],
        ], dtype=np.float64)

    @property
    def header(self) -> Header:
        return Header.from_stamp(self.ts, self.frame_id)

    @property
    def distortion_model(self) -> str:
        return "plumb_bob"

    @property
    def D(self) -> list[float]:
        return self.D_vector.tolist()

    @property
    def K(self) -> list[float]:
        return self.K_matrix.reshape(-1).tolist()

    @property
    def R(self) -> list[float]:
        return [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]

    @property
    def P(self) -> list[float]:
        return [
            float(self.fx), 0.0, float(self.cx), 0.0,
            0.0, float(self.fy), float(self.cy), 0.0,
            0.0, 0.0, 1.0, 0.0,
        ]

    def project(self, u: float, v: float, depth: float) -> Vector3:
        """Back-project pixel ``(u, v)`` at *depth* (metres) to 3-D point.

        Uses the standard pinhole model::

            X = (u - cx) * depth / fx
            Y = (v - cy) * depth / fy
            Z = depth
        """
        x = (u - self.cx) * depth / self.fx
        y = (v - self.cy) * depth / self.fy
        return Vector3(x, y, depth)

    # -- factory / conversion ------------------------------------------------

    @classmethod
    def from_yaml(cls, path: str | Path) -> CameraIntrinsics:
        """Load from OpenCV-style YAML calibration file."""
        import yaml  # type: ignore[import-untyped]

        with open(path, encoding="utf-8") as f:
            data = yaml.safe_load(f)

        width = data.get("image_width", 0)
        height = data.get("image_height", 0)
        K = data.get("camera_matrix", {}).get("data", [0.0] * 9)
        D = data.get("distortion_coefficients", {}).get("data", [0.0] * 5)
        depth_scale = data.get("depth_scale", 1.0)
        return cls(
            fx=K[0], fy=K[4], cx=K[2], cy=K[5],
            width=width, height=height, depth_scale=depth_scale,
            dist_k1=D[0] if len(D) > 0 else 0.0,
            dist_k2=D[1] if len(D) > 1 else 0.0,
            dist_p1=D[2] if len(D) > 2 else 0.0,
            dist_p2=D[3] if len(D) > 3 else 0.0,
            dist_k3=D[4] if len(D) > 4 else 0.0,
        )

    @classmethod
    def from_dict(cls, d: dict[str, Any]) -> CameraIntrinsics:
        return cls(
            fx=d["fx"], fy=d["fy"], cx=d["cx"], cy=d["cy"],
            width=d["width"], height=d["height"],
            depth_scale=d.get("depth_scale", 1.0),
            dist_k1=d.get("dist_k1", 0.0),
            dist_k2=d.get("dist_k2", 0.0),
            dist_p1=d.get("dist_p1", 0.0),
            dist_p2=d.get("dist_p2", 0.0),
            dist_k3=d.get("dist_k3", 0.0),
            ts=float(d.get("ts", time.time())),
            frame_id=str(d.get("frame_id", SENSOR_CAMERA_FRAME_ID)),
        )

    def to_dict(self) -> dict[str, Any]:
        return {
            "fx": self.fx, "fy": self.fy,
            "cx": self.cx, "cy": self.cy,
            "width": self.width, "height": self.height,
            "depth_scale": self.depth_scale,
            "dist_k1": self.dist_k1, "dist_k2": self.dist_k2,
            "dist_p1": self.dist_p1, "dist_p2": self.dist_p2,
            "dist_k3": self.dist_k3,
            "ts": self.ts,
            "frame_id": self.frame_id,
        }

    def to_ros_dict(self) -> dict[str, Any]:
        return {
            "header": self.header.to_dict(),
            "height": int(self.height),
            "width": int(self.width),
            "distortion_model": self.distortion_model,
            "d": self.D,
            "k": self.K,
            "r": self.R,
            "p": self.P,
            "binning_x": 0,
            "binning_y": 0,
            "roi": {
                "x_offset": 0,
                "y_offset": 0,
                "height": 0,
                "width": 0,
                "do_rectify": False,
            },
        }

    # -- binary encode / decode ----------------------------------------------

    def encode(self) -> bytes:
        """48 bytes: 5×float64 + 2×uint32."""
        return _INTRINSICS_FMT.pack(
            self.fx, self.fy, self.cx, self.cy, self.depth_scale,
            self.width, self.height,
        )

    @classmethod
    def decode(cls, data: bytes) -> CameraIntrinsics:
        fx, fy, cx, cy, ds, w, h = _INTRINSICS_FMT.unpack(data[: _INTRINSICS_FMT.size])
        return cls(fx=fx, fy=fy, cx=cx, cy=cy, width=w, height=h, depth_scale=ds)

    def __repr__(self) -> str:
        return (f"CameraIntrinsics(fx={self.fx:.1f}, fy={self.fy:.1f}, "
                f"cx={self.cx:.1f}, cy={self.cy:.1f}, "
                f"{self.width}x{self.height}, scale={self.depth_scale})")


# ---------------------------------------------------------------------------
# PointCloud2
# ---------------------------------------------------------------------------

# Header: num_points(u32) + cols(u32) + ts(f64) + frame_len(u32) + frame_str
_PC_HDR = struct.Struct("<IId")  # 16 bytes fixed part


@dataclass
class PointField:
    """Minimal ROS2-compatible PointField descriptor."""

    INT8: ClassVar[int] = 1
    UINT8: ClassVar[int] = 2
    INT16: ClassVar[int] = 3
    UINT16: ClassVar[int] = 4
    INT32: ClassVar[int] = 5
    UINT32: ClassVar[int] = 6
    FLOAT32: ClassVar[int] = 7
    FLOAT64: ClassVar[int] = 8

    name: str
    offset: int
    datatype: int
    count: int = 1


def _default_point_fields(cols: int) -> list[PointField]:
    fields = [
        PointField("x", 0, PointField.FLOAT32),
        PointField("y", 4, PointField.FLOAT32),
        PointField("z", 8, PointField.FLOAT32),
    ]
    if cols == 4:
        fields.append(PointField("intensity", 12, PointField.FLOAT32))
    return fields


@dataclass
class PointCloud2:
    """Lightweight point cloud — ``(N, 3)`` XYZ or ``(N, 4)`` XYZ+intensity.

    The payload stays NumPy-native, while the metadata mirrors the common
    ROS2 ``sensor_msgs/PointCloud2`` fields used by bridge code.
    """

    msg_name: ClassVar[str] = "sensor_msgs.PointCloud2"
    points: Any = field(default_factory=list)
    ts: float = field(default_factory=time.time)
    frame_id: str = field(default=SENSOR_MAP_FRAME_ID)
    height: int = 1
    width: int = 0
    fields: list[PointField] = field(default_factory=list)
    is_dense: bool = True
    is_bigendian: bool = False
    point_step: int = 0
    row_step: int = 0

    def __post_init__(self) -> None:
        if not is_numpy_array(self.points):
            if self.points or numpy_import_is_safe():
                self.points = np.asarray(self.points, dtype=np.float32)
        if is_numpy_array(self.points):
            if self.points.ndim == 1 and self.points.size == 0:
                self.points = self.points.reshape(0, 3)
            if self.points.ndim != 2 or self.points.shape[1] not in (3, 4):
                raise ValueError(f"points must be (N,3) or (N,4), got {self.points.shape}")
            num_points = int(self.points.shape[0])
            cols = int(self.points.shape[1])
        else:
            if self.points:
                raise ImportError("NumPy import is unsafe in this host interpreter")
            num_points = 0
            cols = 3
        if self.height <= 0:
            raise ValueError(f"height must be positive, got {self.height}")

        if self.width <= 0:
            if self.height == 1:
                self.width = num_points
            elif num_points == 0:
                self.width = 0
            elif num_points % self.height == 0:
                self.width = num_points // self.height
            else:
                raise ValueError(
                    f"cannot infer width from {num_points} points and height={self.height}"
                )
        elif self.height * self.width != num_points:
            raise ValueError(
                f"height*width must match num_points, got {self.height}*{self.width} != {num_points}"
            )

        if self.fields:
            norm_fields: list[PointField] = []
            for spec in self.fields:
                if isinstance(spec, PointField):
                    norm_fields.append(spec)
                elif isinstance(spec, dict):
                    norm_fields.append(PointField(**spec))
                else:
                    norm_fields.append(
                        PointField(
                            name=str(spec.name),
                            offset=int(spec.offset),
                            datatype=int(spec.datatype),
                            count=int(getattr(spec, "count", 1)),
                        )
                    )
            self.fields = norm_fields
        else:
            self.fields = _default_point_fields(cols)

        if self.point_step <= 0:
            self.point_step = int(cols * 4)
        if self.row_step <= 0:
            self.row_step = int(self.point_step * self.width)
        if (
            is_numpy_array(self.points)
            and self.is_dense
            and not os.environ.get("LINGTU_SKIP_DENSE_CHECK")
            and not np.isfinite(self.points).all()
        ):
            self.is_dense = False

    # -- factory methods -----------------------------------------------------

    @classmethod
    def from_numpy(
        cls,
        points: np.ndarray,
        frame_id: str = SENSOR_MAP_FRAME_ID,
        **kw,
    ) -> PointCloud2:
        return cls(points=points, frame_id=frame_id, **kw)

    @classmethod
    def from_depth(cls, depth: np.ndarray, intrinsics: CameraIntrinsics,
                   frame_id: str = SENSOR_CAMERA_FRAME_ID) -> PointCloud2:
        """Back-project a depth image to a 3-D point cloud.

        Parameters
        ----------
        depth : np.ndarray
            ``(H, W)`` depth image.  Values are in raw sensor units;
            ``intrinsics.depth_scale`` is applied automatically.
        intrinsics : CameraIntrinsics
            Camera calibration.
        """
        h, w = depth.shape[:2]
        u = np.arange(w, dtype=np.float32)
        v = np.arange(h, dtype=np.float32)
        u, v = np.meshgrid(u, v)

        z = depth.astype(np.float32) * intrinsics.depth_scale
        mask = z > 0
        z = z[mask]
        x = ((u[mask] - intrinsics.cx) * z / intrinsics.fx)
        y = ((v[mask] - intrinsics.cy) * z / intrinsics.fy)
        pts = np.stack([x, y, z], axis=-1)
        return cls(points=pts, frame_id=frame_id)

    # -- properties ----------------------------------------------------------

    @property
    def num_points(self) -> int:
        if is_numpy_array(self.points):
            return int(self.points.shape[0])
        return len(self.points) if isinstance(self.points, list) else 0

    @property
    def is_empty(self) -> bool:
        return self.num_points == 0

    @property
    def header(self) -> Header:
        return Header.from_stamp(self.ts, self.frame_id)

    @property
    def data(self) -> bytes:
        """Raw point bytes laid out like a dense PointCloud2 payload."""
        return self.points.tobytes() if is_numpy_array(self.points) else b""

    # -- transforms ----------------------------------------------------------

    def transform(self, matrix: np.ndarray) -> PointCloud2:
        """Apply a 4×4 homogeneous transform, returning a **new** cloud."""
        matrix = np.asarray(matrix, dtype=np.float64)
        if matrix.shape != (4, 4):
            raise ValueError(f"Transform must be 4x4, got {matrix.shape}")

        xyz = self.points[:, :3].astype(np.float64)
        ones = np.ones((xyz.shape[0], 1), dtype=np.float64)
        homo = np.hstack([xyz, ones])  # (N, 4)
        transformed = (matrix @ homo.T).T[:, :3].astype(np.float32)

        if self.points.shape[1] == 4:
            # preserve intensity column
            transformed = np.hstack([transformed, self.points[:, 3:4]])

        return PointCloud2(
            points=transformed,
            ts=self.ts,
            frame_id=self.frame_id,
            height=self.height,
            width=self.width,
            fields=list(self.fields),
            is_dense=self.is_dense,
            is_bigendian=self.is_bigendian,
            point_step=self.point_step,
            row_step=self.row_step,
        )

    def voxel_downsample(self, voxel_size: float) -> PointCloud2:
        """Grid-based voxel down-sampling (pure NumPy)."""
        if self.is_empty or voxel_size <= 0:
            return PointCloud2(
                points=self.points.copy() if hasattr(self.points, "copy") else list(self.points),
                ts=self.ts,
                frame_id=self.frame_id,
                height=self.height,
                width=self.width,
                fields=list(self.fields),
                is_dense=self.is_dense,
                is_bigendian=self.is_bigendian,
                point_step=self.point_step,
                row_step=self.row_step,
            )

        xyz = self.points[:, :3]
        keys = np.floor(xyz / voxel_size).astype(np.int64)
        # unique voxel keys → keep first point per voxel
        _, idx = np.unique(keys, axis=0, return_index=True)
        idx.sort()  # preserve original ordering
        return PointCloud2(
            points=self.points[idx].copy(),
            ts=self.ts,
            frame_id=self.frame_id,
            fields=list(self.fields),
            is_dense=self.is_dense,
            is_bigendian=self.is_bigendian,
        )

    # -- serialisation -------------------------------------------------------

    def encode(self) -> bytes:
        """Serialise: fixed header + frame string + raw float32 blob."""
        frame_bytes = self.frame_id.encode()
        if is_numpy_array(self.points):
            n, cols = self.points.shape
            points_bytes = self.points.tobytes()
        elif not self.points:
            n, cols = 0, 3
            points_bytes = b""
        else:
            points = np.asarray(self.points, dtype=np.float32)
            n, cols = points.shape
            points_bytes = points.tobytes()
        buf = bytearray()
        buf += _PC_HDR.pack(n, cols, self.ts)
        buf += struct.pack("<I", len(frame_bytes)) + frame_bytes
        buf += points_bytes
        return bytes(buf)

    @classmethod
    def decode(cls, raw: bytes) -> PointCloud2:
        off = 0
        n, cols, ts = _PC_HDR.unpack_from(raw, off)
        off += _PC_HDR.size
        frame_len = struct.unpack_from("<I", raw, off)[0]
        off += 4
        frame_id = raw[off: off + frame_len].decode()
        off += frame_len
        pts = np.frombuffer(raw, dtype=np.float32, offset=off, count=n * cols).reshape(n, cols)
        return cls(points=pts.copy(), ts=ts, frame_id=frame_id)

    # -- introspection -------------------------------------------------------

    def to_dict(self) -> dict[str, Any]:
        """Metadata dict (no point data)."""
        if self.is_empty:
            bounds = {"min": [0, 0, 0], "max": [0, 0, 0]}
        else:
            xyz = self.points[:, :3]
            bounds = {
                "min": xyz.min(axis=0).tolist(),
                "max": xyz.max(axis=0).tolist(),
            }
        return {
            "header": self.header.to_dict(),
            "num_points": self.num_points,
            "height": self.height,
            "width": self.width,
            "cols": int(self.points.shape[1]) if is_numpy_array(self.points) else 3,
            "frame_id": self.frame_id,
            "ts": self.ts,
            "fields": [
                {
                    "name": f.name,
                    "offset": f.offset,
                    "datatype": f.datatype,
                    "count": f.count,
                }
                for f in self.fields
            ],
            "is_dense": self.is_dense,
            "is_bigendian": self.is_bigendian,
            "point_step": self.point_step,
            "row_step": self.row_step,
            "bounds": bounds,
        }

    def to_ros_dict(self) -> dict[str, Any]:
        return {
            "header": self.header.to_dict(),
            "height": self.height,
            "width": self.width,
            "fields": [
                {
                    "name": f.name,
                    "offset": f.offset,
                    "datatype": f.datatype,
                    "count": f.count,
                }
                for f in self.fields
            ],
            "is_bigendian": self.is_bigendian,
            "point_step": self.point_step,
            "row_step": self.row_step,
            "data": self.data,
            "is_dense": self.is_dense,
        }

    def __repr__(self) -> str:
        return (
            f"PointCloud2({self.num_points} pts, {self.height}x{self.width}, "
            f"frame='{self.frame_id}')"
        )



# ---------------------------------------------------------------------------
# Imu
# ---------------------------------------------------------------------------

_IMU_LEGACY_FMT = struct.Struct("<10d")  # quat(4) + gyro(3) + accel(3)
_IMU_FMT = struct.Struct("<37d")  # quat(4) + cov(9) + gyro(3) + cov(9) + accel(3) + cov(9)


def _covariance9_unknown() -> list[float]:
    return [0.0] * 9


def _covariance9_unavailable() -> list[float]:
    values = [0.0] * 9
    values[0] = -1.0
    return values


def _coerce_covariance9(name: str, values: Any) -> list[float]:
    out = [float(v) for v in values]
    if len(out) != 9:
        raise ValueError(f"{name} must contain 9 values, got {len(out)}")
    return out


def _read_frame_id(raw: bytes, off: int) -> tuple[str, int]:
    frame_len = struct.unpack_from("<I", raw, off)[0]
    off += 4
    end = off + frame_len
    if end > len(raw):
        raise ValueError("IMU frame_id extends past payload")
    return raw[off:end].decode(), end


@dataclass
class Imu:
    """Inertial measurement unit reading.

    Fields mirror ROS ``sensor_msgs/Imu``.
    """

    msg_name: ClassVar[str] = "sensor_msgs.Imu"
    orientation: Quaternion = field(default_factory=Quaternion.identity)
    orientation_covariance: list[float] = field(default_factory=_covariance9_unavailable)
    angular_velocity: Vector3 = field(default_factory=Vector3)
    angular_velocity_covariance: list[float] = field(default_factory=_covariance9_unknown)
    linear_acceleration: Vector3 = field(default_factory=Vector3)
    linear_acceleration_covariance: list[float] = field(default_factory=_covariance9_unknown)
    ts: float = field(default_factory=time.time)
    frame_id: str = field(default="imu_link")

    def __post_init__(self) -> None:
        self.orientation_covariance = _coerce_covariance9(
            "orientation_covariance", self.orientation_covariance
        )
        self.angular_velocity_covariance = _coerce_covariance9(
            "angular_velocity_covariance", self.angular_velocity_covariance
        )
        self.linear_acceleration_covariance = _coerce_covariance9(
            "linear_acceleration_covariance", self.linear_acceleration_covariance
        )

    @property
    def header(self) -> Header:
        return Header.from_stamp(self.ts, self.frame_id)

    def to_dict(self) -> dict[str, Any]:
        return {
            "header": self.header.to_dict(),
            "orientation": self.orientation.to_dict(),
            "orientation_covariance": list(self.orientation_covariance),
            "angular_velocity": self.angular_velocity.to_dict(),
            "angular_velocity_covariance": list(self.angular_velocity_covariance),
            "linear_acceleration": self.linear_acceleration.to_dict(),
            "linear_acceleration_covariance": list(self.linear_acceleration_covariance),
            "ts": float(self.ts),
            "frame_id": self.frame_id,
        }

    def to_ros_dict(self) -> dict[str, Any]:
        return {
            "header": self.header.to_dict(),
            "orientation": self.orientation.to_dict(),
            "orientation_covariance": list(self.orientation_covariance),
            "angular_velocity": self.angular_velocity.to_dict(),
            "angular_velocity_covariance": list(self.angular_velocity_covariance),
            "linear_acceleration": self.linear_acceleration.to_dict(),
            "linear_acceleration_covariance": list(self.linear_acceleration_covariance),
        }

    @classmethod
    def from_dict(cls, d: dict[str, Any]) -> Imu:
        return cls(
            orientation=Quaternion.from_dict(d.get("orientation") or {}),
            orientation_covariance=d.get("orientation_covariance", _covariance9_unavailable()),
            angular_velocity=Vector3.from_dict(d.get("angular_velocity") or {}),
            angular_velocity_covariance=d.get(
                "angular_velocity_covariance", _covariance9_unknown()
            ),
            linear_acceleration=Vector3.from_dict(d.get("linear_acceleration") or {}),
            linear_acceleration_covariance=d.get(
                "linear_acceleration_covariance", _covariance9_unknown()
            ),
            ts=float(d.get("ts", time.time())),
            frame_id=str(d.get("frame_id", "imu_link")),
        )

    # -- encode / decode -----------------------------------------------------

    def encode(self) -> bytes:
        """Binary: ROS Imu numeric fields + ts(f64) + frame."""
        frame_bytes = self.frame_id.encode()
        buf = bytearray()
        buf += _IMU_FMT.pack(
            self.orientation.x, self.orientation.y,
            self.orientation.z, self.orientation.w,
            *self.orientation_covariance,
            self.angular_velocity.x, self.angular_velocity.y, self.angular_velocity.z,
            *self.angular_velocity_covariance,
            self.linear_acceleration.x, self.linear_acceleration.y, self.linear_acceleration.z,
            *self.linear_acceleration_covariance,
        )
        buf += struct.pack("<d", self.ts)
        buf += struct.pack("<I", len(frame_bytes)) + frame_bytes
        return bytes(buf)

    @classmethod
    def decode(cls, raw: bytes) -> Imu:
        if len(raw) >= _IMU_FMT.size + 12:
            try:
                return cls._decode_full(raw)
            except (struct.error, UnicodeDecodeError, ValueError):
                pass
        return cls._decode_legacy(raw)

    @classmethod
    def _decode_full(cls, raw: bytes) -> Imu:
        vals = _IMU_FMT.unpack_from(raw, 0)
        off = _IMU_FMT.size
        ts = struct.unpack_from("<d", raw, off)[0]
        off += 8
        frame_id, _ = _read_frame_id(raw, off)
        return cls(
            orientation=Quaternion(vals[0], vals[1], vals[2], vals[3]),
            orientation_covariance=list(vals[4:13]),
            angular_velocity=Vector3(vals[13], vals[14], vals[15]),
            angular_velocity_covariance=list(vals[16:25]),
            linear_acceleration=Vector3(vals[25], vals[26], vals[27]),
            linear_acceleration_covariance=list(vals[28:37]),
            ts=ts,
            frame_id=frame_id,
        )

    @classmethod
    def _decode_legacy(cls, raw: bytes) -> Imu:
        vals = _IMU_LEGACY_FMT.unpack_from(raw, 0)
        off = _IMU_LEGACY_FMT.size
        ts = struct.unpack_from("<d", raw, off)[0]
        off += 8
        frame_id, _ = _read_frame_id(raw, off)
        return cls(
            orientation=Quaternion(vals[0], vals[1], vals[2], vals[3]),
            angular_velocity=Vector3(vals[4], vals[5], vals[6]),
            linear_acceleration=Vector3(vals[7], vals[8], vals[9]),
            ts=ts,
            frame_id=frame_id,
        )

    def __repr__(self) -> str:
        return (f"Imu(gyro=({self.angular_velocity.x:.3f}, "
                f"{self.angular_velocity.y:.3f}, {self.angular_velocity.z:.3f}), "
                f"accel=({self.linear_acceleration.x:.3f}, "
                f"{self.linear_acceleration.y:.3f}, {self.linear_acceleration.z:.3f}), "
                f"frame='{self.frame_id}')")
