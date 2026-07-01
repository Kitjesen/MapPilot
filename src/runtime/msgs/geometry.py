"""lingtu.runtime.msgs.geometry — core geometry message types.

Vector3 / Quaternion / Pose / PoseStamped / Twist / TwistStamped / Transform
shared by all navigation modules. NumPy is loaded lazily for array conversions.

Encoding: little-endian double ('<d').
"""
from __future__ import annotations

import math
import struct
import time
from typing import Any, ClassVar

from runtime.runtime_interface import body_frame_id, map_frame_id

from .numpy_compat import is_numpy_array, np

GEOMETRY_MAP_FRAME_ID = map_frame_id()
GEOMETRY_BODY_FRAME_ID = body_frame_id()
_FLOAT_RTOL = 1e-5
_FLOAT_ATOL = 1e-8


def _float_close(left: float, right: float) -> bool:
    return math.isclose(left, right, rel_tol=_FLOAT_RTOL, abs_tol=_FLOAT_ATOL)


def _header_from_stamp(ts: float, frame_id: str) -> Any:
    from .sensor import Header

    return Header.from_stamp(ts, frame_id)

# ---------------------------------------------------------------------------
# Vector3
# ---------------------------------------------------------------------------


class Vector3:
    """3-D vector with full math ops and binary serialisation.

    Construction::

        Vector3()                      # (0, 0, 0)
        Vector3(1, 2, 3)               # three scalars
        Vector3([1, 2, 3])             # list / tuple
        Vector3(np.array([1, 2, 3]))   # numpy
        Vector3(other_vec3)            # copy
        Vector3(x=1, y=2, z=3)         # keywords
    """

    msg_name: ClassVar[str] = "geometry_msgs.Vector3"
    __slots__ = ("x", "y", "z")
    _FMT = struct.Struct("<3d")  # 24 bytes, little-endian

    def __init__(self, *args: Any, **kwargs: Any) -> None:
        if kwargs and not args:
            self.x = float(kwargs.get("x", 0.0))
            self.y = float(kwargs.get("y", 0.0))
            self.z = float(kwargs.get("z", 0.0))
        elif not args:
            self.x = 0.0
            self.y = 0.0
            self.z = 0.0
        elif len(args) == 1:
            a = args[0]
            if isinstance(a, Vector3):
                self.x, self.y, self.z = a.x, a.y, a.z
            elif is_numpy_array(a):
                d = a.ravel().astype(float)
                self.x = float(d[0]) if len(d) > 0 else 0.0
                self.y = float(d[1]) if len(d) > 1 else 0.0
                self.z = float(d[2]) if len(d) > 2 else 0.0
            elif isinstance(a, (list, tuple)):
                self.x = float(a[0]) if len(a) > 0 else 0.0
                self.y = float(a[1]) if len(a) > 1 else 0.0
                self.z = float(a[2]) if len(a) > 2 else 0.0
            elif isinstance(a, (int, float)):
                self.x = float(a)
                self.y = 0.0
                self.z = 0.0
            else:
                raise TypeError(f"Cannot init Vector3 from {type(a)}")
        elif len(args) == 2:
            self.x = float(args[0])
            self.y = float(args[1])
            self.z = 0.0
        elif len(args) == 3:
            self.x = float(args[0])
            self.y = float(args[1])
            self.z = float(args[2])
        else:
            raise TypeError(f"Vector3 takes 0-3 positional args ({len(args)} given)")

    # -- operators -----------------------------------------------------------

    def __add__(self, other: Any) -> Vector3:
        o = _to_vec3(other)
        return Vector3(self.x + o.x, self.y + o.y, self.z + o.z)

    def __sub__(self, other: Any) -> Vector3:
        o = _to_vec3(other)
        return Vector3(self.x - o.x, self.y - o.y, self.z - o.z)

    def __mul__(self, s: float) -> Vector3:
        return Vector3(self.x * s, self.y * s, self.z * s)

    def __rmul__(self, s: float) -> Vector3:
        return self.__mul__(s)

    def __truediv__(self, s: float) -> Vector3:
        return Vector3(self.x / s, self.y / s, self.z / s)

    def __neg__(self) -> Vector3:
        return Vector3(-self.x, -self.y, -self.z)

    def __eq__(self, other: object) -> bool:
        if not isinstance(other, Vector3):
            return NotImplemented
        return (
            _float_close(self.x, other.x)
            and _float_close(self.y, other.y)
            and _float_close(self.z, other.z)
        )

    def __repr__(self) -> str:
        return f"Vector3({self.x:.4f}, {self.y:.4f}, {self.z:.4f})"

    def __str__(self) -> str:
        return self.__repr__()

    # -- vector math ---------------------------------------------------------

    def dot(self, other: Any) -> float:
        """Dot product with another vector."""
        o = _to_vec3(other)
        return self.x * o.x + self.y * o.y + self.z * o.z

    def cross(self, other: Any) -> Vector3:
        """Cross product with another vector."""
        o = _to_vec3(other)
        return Vector3(
            self.y * o.z - self.z * o.y,
            self.z * o.x - self.x * o.z,
            self.x * o.y - self.y * o.x,
        )

    def length(self) -> float:
        """Euclidean norm (L2 norm)."""
        return math.sqrt(self.x * self.x + self.y * self.y + self.z * self.z)

    def length_squared(self) -> float:
        """Squared Euclidean norm (avoids sqrt)."""
        return self.x * self.x + self.y * self.y + self.z * self.z

    def normalize(self) -> Vector3:
        """Return unit vector. Returns zero vector if length is near zero."""
        n = self.length()
        if n < 1e-12:
            return Vector3()
        return Vector3(self.x / n, self.y / n, self.z / n)

    def distance(self, other: Any) -> float:
        """Euclidean distance to another vector."""
        return (self - _to_vec3(other)).length()

    def angle(self, other: Any) -> float:
        """Angle between two vectors (radians)."""
        o = _to_vec3(other)
        n1, n2 = self.length(), o.length()
        if n1 < 1e-12 or n2 < 1e-12:
            return 0.0
        cos_a = max(-1.0, min(1.0, self.dot(o) / (n1 * n2)))
        return math.acos(cos_a)

    def is_zero(self) -> bool:
        """True if all components are approximately zero."""
        return (
            _float_close(self.x, 0.0)
            and _float_close(self.y, 0.0)
            and _float_close(self.z, 0.0)
        )

    # -- conversions ---------------------------------------------------------

    def to_list(self) -> list[float]:
        """Return components as a Python list."""
        return [self.x, self.y, self.z]

    def to_tuple(self) -> tuple:
        """Return components as a tuple."""
        return (self.x, self.y, self.z)

    def to_numpy(self) -> np.ndarray:
        """Return as a float64 numpy array."""
        return np.array([self.x, self.y, self.z], dtype=np.float64)

    @classmethod
    def from_numpy(cls, arr: np.ndarray) -> Vector3:
        """Construct from a numpy array (padded with zeros if < 3 elements)."""
        arr = np.asarray(arr, dtype=float).ravel()
        if arr.size < 3:
            padded = np.zeros(3)
            padded[: arr.size] = arr
            arr = padded
        return cls(float(arr[0]), float(arr[1]), float(arr[2]))

    def to_dict(self) -> dict[str, float]:
        """Serialize to a dict with keys x, y, z."""
        return {"x": self.x, "y": self.y, "z": self.z}

    def to_ros_dict(self) -> dict[str, float]:
        return self.to_dict()

    @classmethod
    def from_dict(cls, d: dict[str, Any]) -> Vector3:
        """Construct from a dict with keys x, y, z (defaults to 0)."""
        return cls(d.get("x", 0.0), d.get("y", 0.0), d.get("z", 0.0))

    # -- binary --------------------------------------------------------------

    def encode(self) -> bytes:
        """Binary encode (24 bytes, little-endian doubles)."""
        return self._FMT.pack(self.x, self.y, self.z)

    @classmethod
    def decode(cls, data: bytes) -> Vector3:
        """Binary decode from 24-byte little-endian double buffer."""
        x, y, z = cls._FMT.unpack(data[: cls._FMT.size])
        return cls(x, y, z)


def _to_vec3(v: Any) -> Vector3:
    """Convenience: list / tuple / ndarray → Vector3."""
    return v if isinstance(v, Vector3) else Vector3(v)


# ---------------------------------------------------------------------------
# Quaternion
# ---------------------------------------------------------------------------


class Quaternion:
    """Quaternion (x, y, z, w); w=1 is identity rotation. Hamilton product, rotate vectors, Euler interop.

    Construction::

        Quaternion()                          # identity (0,0,0,1)
        Quaternion(x, y, z, w)                # four scalars
        Quaternion([x, y, z, w])              # list / tuple / ndarray
        Quaternion(other_quat)                # copy
        Quaternion(x=0, y=0, z=0, w=1)       # keywords
    """

    msg_name: ClassVar[str] = "geometry_msgs.Quaternion"
    __slots__ = ("w", "x", "y", "z")
    _FMT = struct.Struct("<4d")  # 32 bytes

    def __init__(self, *args: Any, **kwargs: Any) -> None:
        if kwargs and not args:
            self.x = float(kwargs.get("x", 0.0))
            self.y = float(kwargs.get("y", 0.0))
            self.z = float(kwargs.get("z", 0.0))
            self.w = float(kwargs.get("w", 1.0))
        elif not args:
            self.x = 0.0
            self.y = 0.0
            self.z = 0.0
            self.w = 1.0
        elif len(args) == 1:
            a = args[0]
            if isinstance(a, Quaternion):
                self.x, self.y, self.z, self.w = a.x, a.y, a.z, a.w
            elif isinstance(a, (list, tuple)) or is_numpy_array(a):
                seq = list(a)
                if len(seq) != 4:
                    raise ValueError(
                        "Quaternion requires exactly 4 components [x,y,z,w]"
                    )
                self.x, self.y, self.z, self.w = (float(v) for v in seq)
            else:
                raise TypeError(f"Cannot init Quaternion from {type(a)}")
        elif len(args) == 4:
            self.x = float(args[0])
            self.y = float(args[1])
            self.z = float(args[2])
            self.w = float(args[3])
        else:
            raise TypeError(
                f"Quaternion takes 0, 1, or 4 positional args ({len(args)} given)"
            )

    # -- Hamilton product ----------------------------------------------------

    def __mul__(self, other: Quaternion) -> Quaternion:
        if not isinstance(other, Quaternion):
            raise TypeError(f"Cannot multiply Quaternion with {type(other)}")
        w = self.w * other.w - self.x * other.x - self.y * other.y - self.z * other.z
        x = self.w * other.x + self.x * other.w + self.y * other.z - self.z * other.y
        y = self.w * other.y - self.x * other.z + self.y * other.w + self.z * other.x
        z = self.w * other.z + self.x * other.y - self.y * other.x + self.z * other.w
        return Quaternion(x, y, z, w)

    def conjugate(self) -> Quaternion:
        """Return conjugate (negate x, y, z)."""
        return Quaternion(-self.x, -self.y, -self.z, self.w)

    def inverse(self) -> Quaternion:
        """Return multiplicative inverse. Raises ZeroDivisionError if zero."""
        ns = self.x ** 2 + self.y ** 2 + self.z ** 2 + self.w ** 2
        if ns < 1e-15:
            raise ZeroDivisionError("Cannot invert zero quaternion")
        if abs(ns - 1.0) < 1e-8:
            return self.conjugate()
        c = self.conjugate()
        return Quaternion(c.x / ns, c.y / ns, c.z / ns, c.w / ns)

    def normalize(self) -> Quaternion:
        """Return unit quaternion. Raises ZeroDivisionError if zero."""
        n = math.sqrt(self.x ** 2 + self.y ** 2 + self.z ** 2 + self.w ** 2)
        if n < 1e-15:
            raise ZeroDivisionError("Cannot normalize zero quaternion")
        return Quaternion(self.x / n, self.y / n, self.z / n, self.w / n)

    def rotate_vector(self, v: Vector3) -> Vector3:
        """q * v * q^* (rotate vector)."""
        vq = Quaternion(v.x, v.y, v.z, 0.0)
        r = self * vq * self.conjugate()
        return Vector3(r.x, r.y, r.z)

    # -- Euler (ZYX intrinsic) -----------------------------------------------

    def to_euler(self) -> Vector3:
        """Return Vector3(roll, pitch, yaw) in radians, ZYX intrinsic."""
        sinr = 2.0 * (self.w * self.x + self.y * self.z)
        cosr = 1.0 - 2.0 * (self.x * self.x + self.y * self.y)
        roll = math.atan2(sinr, cosr)

        sinp = 2.0 * (self.w * self.y - self.z * self.x)
        sinp = max(-1.0, min(1.0, sinp))
        pitch = math.asin(sinp)

        siny = 2.0 * (self.w * self.z + self.x * self.y)
        cosy = 1.0 - 2.0 * (self.y * self.y + self.z * self.z)
        yaw = math.atan2(siny, cosy)
        return Vector3(roll, pitch, yaw)

    @classmethod
    def from_euler(cls, roll: float, pitch: float, yaw: float) -> Quaternion:
        """Build quaternion from (roll, pitch, yaw) radians (ZYX intrinsic)."""
        cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
        cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
        cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
        return cls(
            x=sr * cp * cy - cr * sp * sy,
            y=cr * sp * cy + sr * cp * sy,
            z=cr * cp * sy - sr * sp * cy,
            w=cr * cp * cy + sr * sp * sy,
        )

    @classmethod
    def from_yaw(cls, yaw: float) -> Quaternion:
        """Build quaternion from yaw (rad)."""
        return cls(0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5))

    def to_rotation_matrix(self) -> np.ndarray:
        """Return 3×3 rotation matrix."""
        x, y, z, w = self.x, self.y, self.z, self.w
        return np.array([
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ])

    @property
    def yaw(self) -> float:
        """Extract yaw (Z rotation) in radians."""
        return self.to_euler().z

    # -- comparison / repr ---------------------------------------------------

    def __eq__(self, other: object) -> bool:
        if not isinstance(other, Quaternion):
            return NotImplemented
        return (
            _float_close(self.x, other.x)
            and _float_close(self.y, other.y)
            and _float_close(self.z, other.z)
            and _float_close(self.w, other.w)
        )

    def __repr__(self) -> str:
        return f"Quaternion({self.x:.6f}, {self.y:.6f}, {self.z:.6f}, {self.w:.6f})"

    def __str__(self) -> str:
        return self.__repr__()

    # -- conversions ---------------------------------------------------------

    def to_list(self) -> list[float]:
        """Return components as a Python list [x, y, z, w]."""
        return [self.x, self.y, self.z, self.w]

    def to_tuple(self) -> tuple:
        """Return components as a tuple (x, y, z, w)."""
        return (self.x, self.y, self.z, self.w)

    def to_numpy(self) -> np.ndarray:
        """Return as a float64 numpy array."""
        return np.array([self.x, self.y, self.z, self.w], dtype=np.float64)

    @classmethod
    def from_numpy(cls, arr: np.ndarray) -> Quaternion:
        """Construct from a 4-element numpy array."""
        arr = np.asarray(arr, dtype=float).ravel()
        if arr.size != 4:
            raise ValueError("Quaternion requires exactly 4 components [x,y,z,w]")
        return cls(float(arr[0]), float(arr[1]), float(arr[2]), float(arr[3]))

    @classmethod
    def identity(cls) -> Quaternion:
        """Return the identity quaternion (0, 0, 0, 1)."""
        return cls(0.0, 0.0, 0.0, 1.0)

    def to_dict(self) -> dict[str, float]:
        """Serialize to a dict with keys x, y, z, w."""
        return {"x": self.x, "y": self.y, "z": self.z, "w": self.w}

    def to_ros_dict(self) -> dict[str, float]:
        return self.to_dict()

    @classmethod
    def from_dict(cls, d: dict[str, Any]) -> Quaternion:
        """Construct from a dict (defaults to identity)."""
        return cls(d.get("x", 0.0), d.get("y", 0.0),
                   d.get("z", 0.0), d.get("w", 1.0))

    # -- binary --------------------------------------------------------------

    def encode(self) -> bytes:
        """Binary encode (32 bytes, little-endian doubles)."""
        return self._FMT.pack(self.x, self.y, self.z, self.w)

    @classmethod
    def decode(cls, data: bytes) -> Quaternion:
        """Binary decode from 32-byte little-endian double buffer."""
        x, y, z, w = cls._FMT.unpack(data[: cls._FMT.size])
        return cls(x, y, z, w)


# ---------------------------------------------------------------------------
# Pose
# ---------------------------------------------------------------------------


class Pose:
    """Pose = position (Vector3) + orientation (Quaternion).

    Construction::

        Pose()                                     # origin + identity
        Pose(x, y, z)                              # position, identity orientation
        Pose(x, y, z, qx, qy, qz, qw)            # full seven parameters
        Pose(Vector3, Quaternion)                  # pass objects directly
        Pose(other_pose)                           # copy
        Pose(position=vec, orientation=quat)       # keywords
    """

    msg_name: ClassVar[str] = "geometry_msgs.Pose"
    __slots__ = ("orientation", "position")

    def __init__(self, *args: Any, **kwargs: Any) -> None:
        if kwargs and not args:
            pos = kwargs.get("position", None)
            ori = kwargs.get("orientation", None)
            self.position = (
                pos if isinstance(pos, Vector3)
                else Vector3(pos) if pos is not None
                else Vector3()
            )
            self.orientation = (
                ori if isinstance(ori, Quaternion)
                else Quaternion(ori) if ori is not None
                else Quaternion()
            )
        elif not args:
            self.position = Vector3()
            self.orientation = Quaternion()
        elif len(args) == 1:
            a = args[0]
            if isinstance(a, Pose):
                self.position = Vector3(a.position)
                self.orientation = Quaternion(a.orientation)
            elif isinstance(a, Vector3):
                self.position = Vector3(a)
                self.orientation = Quaternion()
            else:
                raise TypeError(f"Cannot init Pose from {type(a)}")
        elif len(args) == 2:
            p, o = args
            self.position = p if isinstance(p, Vector3) else Vector3(p)
            self.orientation = o if isinstance(o, Quaternion) else Quaternion(o)
        elif len(args) == 3:
            self.position = Vector3(float(args[0]), float(args[1]), float(args[2]))
            self.orientation = Quaternion()
        elif len(args) == 7:
            self.position = Vector3(float(args[0]), float(args[1]), float(args[2]))
            self.orientation = Quaternion(
                float(args[3]), float(args[4]), float(args[5]), float(args[6])
            )
        else:
            raise TypeError(
                f"Pose takes 0, 1, 2, 3, or 7 positional args ({len(args)} given)"
            )

    # -- convenience properties (backward compat with dataclass version) -----

    @property
    def x(self) -> float:
        """Position X coordinate."""
        return self.position.x

    @property
    def y(self) -> float:
        """Position Y coordinate."""
        return self.position.y

    @property
    def z(self) -> float:
        """Position Z coordinate."""
        return self.position.z

    @property
    def yaw(self) -> float:
        """Extract yaw (Z rotation) from orientation in radians."""
        return self.orientation.yaw

    # -- composition ---------------------------------------------------------

    def __add__(self, other: Any) -> Pose:
        """Pose compose: apply *other* transform in *self* frame."""
        if isinstance(other, Transform):
            op, oq = other.translation, other.rotation
        elif isinstance(other, Pose):
            op, oq = other.position, other.orientation
        else:
            raise TypeError(f"Cannot compose Pose with {type(other)}")
        new_q = self.orientation * oq
        new_p = self.position + self.orientation.rotate_vector(op)
        return Pose(new_p, new_q)

    def __sub__(self, other: Pose) -> Pose:
        """Pose delta: delta = self - other."""
        dp = self.position - other.position
        dq = self.orientation * other.orientation.inverse()
        return Pose(dp, dq)

    def __eq__(self, other: object) -> bool:
        if not isinstance(other, Pose):
            return NotImplemented
        return self.position == other.position and self.orientation == other.orientation

    def __repr__(self) -> str:
        return f"Pose(pos=[{self.x:.3f}, {self.y:.3f}, {self.z:.3f}], yaw={self.yaw:.3f})"

    def __str__(self) -> str:
        e = self.orientation.to_euler()
        return (
            f"Pose(pos=[{self.x:.3f}, {self.y:.3f}, {self.z:.3f}], "
            f"rpy=[{math.degrees(e.x):.1f}, {math.degrees(e.y):.1f}, "
            f"{math.degrees(e.z):.1f}])"
        )

    # -- serialisation -------------------------------------------------------

    def to_dict(self) -> dict[str, Any]:
        """Serialize to a dict with position and orientation."""
        return {
            "position": self.position.to_dict(),
            "orientation": self.orientation.to_dict(),
        }

    def to_ros_dict(self) -> dict[str, Any]:
        return {
            "position": self.position.to_ros_dict(),
            "orientation": self.orientation.to_ros_dict(),
        }

    @classmethod
    def from_dict(cls, d: dict[str, Any]) -> Pose:
        """Construct from a dict with position and orientation keys."""
        p = d.get("position", {})
        o = d.get("orientation", {})
        return cls(
            position=Vector3.from_dict(p),
            orientation=Quaternion.from_dict(o),
        )

    # Binary: 7 doubles = 56 bytes (position xyz + orientation xyzw)
    _FMT = struct.Struct("<7d")

    def encode(self) -> bytes:
        """Binary encode (56 bytes, little-endian doubles)."""
        return self._FMT.pack(
            self.position.x, self.position.y, self.position.z,
            self.orientation.x, self.orientation.y,
            self.orientation.z, self.orientation.w,
        )

    @classmethod
    def decode(cls, data: bytes) -> Pose:
        """Binary decode from 56-byte little-endian double buffer."""
        px, py, pz, qx, qy, qz, qw = cls._FMT.unpack(data[: cls._FMT.size])
        return cls(Vector3(px, py, pz), Quaternion(qx, qy, qz, qw))


# ---------------------------------------------------------------------------
# PoseStamped
# ---------------------------------------------------------------------------


class PoseStamped:
    """Pose with timestamp and frame id (composition; includes .pose)."""

    msg_name: ClassVar[str] = "geometry_msgs.PoseStamped"
    __slots__ = ("frame_id", "pose", "ts")

    def __init__(
        self,
        pose: Pose | None = None,
        ts: float = 0.0,
        frame_id: str = GEOMETRY_MAP_FRAME_ID,
    ) -> None:
        self.pose: Pose = pose if pose is not None else Pose()
        self.ts: float = ts if ts != 0.0 else time.time()
        self.frame_id: str = frame_id

    # -- pass-through properties ---------------------------------------------

    @property
    def x(self) -> float:
        """Position X coordinate."""
        return self.pose.x

    @property
    def y(self) -> float:
        """Position Y coordinate."""
        return self.pose.y

    @property
    def z(self) -> float:
        """Position Z coordinate."""
        return self.pose.z

    @property
    def yaw(self) -> float:
        """Extract yaw (Z rotation) in radians."""
        return self.pose.yaw

    @property
    def position(self) -> Vector3:
        """Position as a Vector3."""
        return self.pose.position

    @property
    def orientation(self) -> Quaternion:
        """Orientation as a Quaternion."""
        return self.pose.orientation

    def __len__(self) -> int:
        return 3

    def __iter__(self):
        return iter((self.x, self.y, self.z))

    def __getitem__(self, index: int | slice):
        return (self.x, self.y, self.z)[index]

    def __array__(self, dtype: Any = None):
        return np.asarray((self.x, self.y, self.z), dtype=dtype)

    def __eq__(self, other: object) -> bool:
        if not isinstance(other, PoseStamped):
            return NotImplemented
        return (
            self.pose == other.pose
            and self.frame_id == other.frame_id
            and abs(self.ts - other.ts) < 1e-6
        )

    def __repr__(self) -> str:
        return (
            f"PoseStamped(pos=[{self.x:.3f}, {self.y:.3f}, {self.z:.3f}], "
            f"yaw={self.yaw:.3f}, frame='{self.frame_id}')"
        )

    def __str__(self) -> str:
        e = self.orientation.to_euler()
        return (
            f"PoseStamped(pos=[{self.x:.3f}, {self.y:.3f}, {self.z:.3f}], "
            f"rpy=[{math.degrees(e.x):.1f}, {math.degrees(e.y):.1f}, "
            f"{math.degrees(e.z):.1f}], frame='{self.frame_id}')"
        )

    # -- serialisation -------------------------------------------------------

    @property
    def header(self) -> Any:
        return _header_from_stamp(self.ts, self.frame_id)

    def to_dict(self) -> dict[str, Any]:
        """Serialize to a dict with pose, ts, and frame_id."""
        return {"pose": self.pose.to_dict(), "ts": self.ts, "frame_id": self.frame_id}

    def to_ros_dict(self) -> dict[str, Any]:
        return {
            "header": self.header.to_dict(),
            "pose": self.pose.to_ros_dict(),
        }

    @classmethod
    def from_dict(cls, d: dict[str, Any]) -> PoseStamped:
        """Construct from a dict with pose, ts, and frame_id keys."""
        return cls(
            pose=Pose.from_dict(d.get("pose", {})),
            ts=float(d.get("ts", 0)),
            frame_id=str(d.get("frame_id", GEOMETRY_MAP_FRAME_ID)),
        )

    # Binary header: ts (double 8B) + frame_id length (uint32 4B) = 12 bytes
    _META = struct.Struct("<dI")

    def encode(self) -> bytes:
        """Binary encode with header: ts + frame_id length + frame_id + pose."""
        frame_bytes = self.frame_id.encode("utf-8")
        return self._META.pack(self.ts, len(frame_bytes)) + frame_bytes + self.pose.encode()

    @classmethod
    def decode(cls, data: bytes) -> PoseStamped:
        """Binary decode from encoded bytes."""
        ts, flen = cls._META.unpack(data[: cls._META.size])
        off = cls._META.size
        frame_id = data[off : off + flen].decode("utf-8")
        pose = Pose.decode(data[off + flen :])
        return cls(pose=pose, ts=ts, frame_id=frame_id)


# ---------------------------------------------------------------------------
# Twist
# ---------------------------------------------------------------------------


class Twist:
    """Linear + angular velocity."""

    msg_name: ClassVar[str] = "geometry_msgs.Twist"
    __slots__ = ("angular", "linear")

    def __init__(self, linear: Any = None, angular: Any = None) -> None:
        self.linear: Vector3 = (
            linear if isinstance(linear, Vector3)
            else Vector3(linear) if linear is not None
            else Vector3()
        )
        self.angular: Vector3 = (
            angular if isinstance(angular, Vector3)
            else Vector3(angular) if angular is not None
            else Vector3()
        )

    @classmethod
    def zero(cls) -> Twist:
        """Return a zero-velocity twist."""
        return cls()

    def is_zero(self) -> bool:
        """True if both linear and angular velocities are zero."""
        return self.linear.is_zero() and self.angular.is_zero()

    def __add__(self, other: Any) -> Twist:
        if not isinstance(other, Twist):
            return NotImplemented
        return Twist(self.linear + other.linear, self.angular + other.angular)

    def __sub__(self, other: Any) -> Twist:
        if not isinstance(other, Twist):
            return NotImplemented
        return Twist(self.linear - other.linear, self.angular - other.angular)

    def __eq__(self, other: object) -> bool:
        if not isinstance(other, Twist):
            return NotImplemented
        return self.linear == other.linear and self.angular == other.angular

    def __repr__(self) -> str:
        return f"Twist(linear={self.linear!r}, angular={self.angular!r})"

    def __str__(self) -> str:
        return self.__repr__()

    # -- serialisation -------------------------------------------------------

    def to_dict(self) -> dict[str, Any]:
        """Serialize to a dict with linear and angular keys."""
        return {
            "linear": self.linear.to_dict(),
            "angular": self.angular.to_dict(),
        }

    def to_ros_dict(self) -> dict[str, Any]:
        return {
            "linear": self.linear.to_ros_dict(),
            "angular": self.angular.to_ros_dict(),
        }

    @classmethod
    def from_dict(cls, d: dict[str, Any]) -> Twist:
        """Construct from a dict with linear and angular keys."""
        li = d.get("linear", {})
        an = d.get("angular", {})
        return cls(Vector3.from_dict(li), Vector3.from_dict(an))

    _FMT = struct.Struct("<6d")  # 48 bytes

    def encode(self) -> bytes:
        """Binary encode (48 bytes, little-endian doubles)."""
        return self._FMT.pack(
            self.linear.x, self.linear.y, self.linear.z,
            self.angular.x, self.angular.y, self.angular.z,
        )

    @classmethod
    def decode(cls, data: bytes) -> Twist:
        """Binary decode from 48-byte little-endian double buffer."""
        lx, ly, lz, ax, ay, az = cls._FMT.unpack(data[: cls._FMT.size])
        return cls(Vector3(lx, ly, lz), Vector3(ax, ay, az))


# ---------------------------------------------------------------------------
# TwistStamped
# ---------------------------------------------------------------------------


class TwistStamped(Twist):
    """Twist with timestamp and frame id."""

    msg_name: ClassVar[str] = "geometry_msgs.TwistStamped"
    __slots__ = ("frame_id", "ts")

    def __init__(
        self,
        linear: Any = None,
        angular: Any = None,
        ts: float = 0.0,
        frame_id: str = GEOMETRY_BODY_FRAME_ID,
    ) -> None:
        super().__init__(linear, angular)
        self.ts: float = ts if ts != 0.0 else time.time()
        self.frame_id: str = frame_id

    def __eq__(self, other: object) -> bool:
        if not isinstance(other, TwistStamped):
            return NotImplemented
        return (
            super().__eq__(other)
            and self.frame_id == other.frame_id
            and abs(self.ts - other.ts) < 1e-6
        )

    def __repr__(self) -> str:
        return (
            f"TwistStamped(linear={self.linear!r}, angular={self.angular!r}, "
            f"frame_id='{self.frame_id}', ts={self.ts:.3f})"
        )

    def __str__(self) -> str:
        return self.__repr__()

    def to_dict(self) -> dict[str, Any]:
        """Serialize to a dict with linear, angular, ts, and frame_id."""
        d = super().to_dict()
        d["ts"] = self.ts
        d["frame_id"] = self.frame_id
        return d

    @property
    def header(self) -> Any:
        return _header_from_stamp(self.ts, self.frame_id)

    def to_ros_dict(self) -> dict[str, Any]:
        return {
            "header": self.header.to_dict(),
            "twist": super().to_ros_dict(),
        }

    @classmethod
    def from_dict(cls, d: dict[str, Any]) -> TwistStamped:
        """Construct from a dict with linear, angular, ts, and frame_id keys."""
        return cls(
            Vector3.from_dict(d.get("linear", {})),
            Vector3.from_dict(d.get("angular", {})),
            ts=d.get("ts", 0.0),
            frame_id=d.get("frame_id", GEOMETRY_BODY_FRAME_ID),
        )

    _HEADER = struct.Struct("<dH")  # ts (8) + frame_id len (2) = 10 bytes

    def encode(self) -> bytes:
        """Binary encode with header: ts + frame_id length + frame_id + velocities."""
        fid = self.frame_id.encode("utf-8")
        header = self._HEADER.pack(self.ts, len(fid)) + fid
        return header + self.linear.encode() + self.angular.encode()

    @classmethod
    def decode(cls, data: bytes) -> TwistStamped:
        """Binary decode from encoded bytes."""
        ts, fid_len = cls._HEADER.unpack(data[: cls._HEADER.size])
        off = cls._HEADER.size
        frame_id = data[off : off + fid_len].decode("utf-8")
        off += fid_len
        lin = Vector3.decode(data[off : off + 24])
        off += 24
        ang = Vector3.decode(data[off : off + 24])
        return cls(lin, ang, ts=ts, frame_id=frame_id)


# ---------------------------------------------------------------------------
# Transform
# ---------------------------------------------------------------------------


class Transform:
    """Rigid transform: translation (Vector3) + rotation (Quaternion) + frame metadata."""

    msg_name: ClassVar[str] = "geometry_msgs.Transform"
    __slots__ = ("child_frame_id", "frame_id", "rotation", "translation", "ts")

    def __init__(
        self,
        translation: Any = None,
        rotation: Any = None,
        frame_id: str = GEOMETRY_MAP_FRAME_ID,
        child_frame_id: str = GEOMETRY_BODY_FRAME_ID,
        ts: float = 0.0,
    ) -> None:
        self.translation: Vector3 = (
            translation if isinstance(translation, Vector3)
            else Vector3(translation) if translation is not None
            else Vector3()
        )
        self.rotation: Quaternion = (
            rotation if isinstance(rotation, Quaternion)
            else Quaternion(rotation) if rotation is not None
            else Quaternion()
        )
        self.frame_id = frame_id
        self.child_frame_id = child_frame_id
        self.ts: float = ts if ts != 0.0 else time.time()

    @classmethod
    def identity(cls) -> Transform:
        """Return the identity transform (zero translation, identity rotation)."""
        return cls()

    # -- composition ---------------------------------------------------------

    def __add__(self, other: Transform) -> Transform:
        """Compose transforms: self * other (apply self then other)."""
        if not isinstance(other, Transform):
            raise TypeError(f"Cannot compose Transform with {type(other)}")
        new_r = self.rotation * other.rotation
        new_t = self.translation + self.rotation.rotate_vector(other.translation)
        return Transform(
            new_t, new_r,
            frame_id=self.frame_id,
            child_frame_id=other.child_frame_id,
            ts=self.ts,
        )

    def inverse(self) -> Transform:
        """Inverse: if self is A→B, returns B→A."""
        inv_r = self.rotation.inverse()
        inv_t = -(inv_r.rotate_vector(self.translation))
        return Transform(
            inv_t, inv_r,
            frame_id=self.child_frame_id,
            child_frame_id=self.frame_id,
            ts=self.ts,
        )

    def __neg__(self) -> Transform:
        return self.inverse()

    def to_matrix(self) -> np.ndarray:
        """Return 4×4 homogeneous transform matrix."""
        m = np.eye(4)
        m[:3, :3] = self.rotation.to_rotation_matrix()
        m[:3, 3] = [self.translation.x, self.translation.y, self.translation.z]
        return m

    # -- comparison / repr ---------------------------------------------------

    def __eq__(self, other: object) -> bool:
        if not isinstance(other, Transform):
            return NotImplemented
        return self.translation == other.translation and self.rotation == other.rotation

    def __repr__(self) -> str:
        return (
            f"Transform(translation={self.translation!r}, "
            f"rotation={self.rotation!r})"
        )

    def __str__(self) -> str:
        return (
            f"{self.frame_id} -> {self.child_frame_id}: "
            f"t={self.translation!r}, r={self.rotation!r}"
        )

    # -- serialisation -------------------------------------------------------

    def to_dict(self) -> dict[str, Any]:
        """Serialize to a dict with translation, rotation, and frame metadata."""
        return {
            "translation": self.translation.to_dict(),
            "rotation": self.rotation.to_dict(),
            "frame_id": self.frame_id,
            "child_frame_id": self.child_frame_id,
            "ts": self.ts,
        }

    @property
    def header(self) -> Any:
        return _header_from_stamp(self.ts, self.frame_id)

    def to_ros_dict(self) -> dict[str, Any]:
        return {
            "header": self.header.to_dict(),
            "child_frame_id": self.child_frame_id,
            "transform": {
                "translation": self.translation.to_ros_dict(),
                "rotation": self.rotation.to_ros_dict(),
            },
        }

    @classmethod
    def from_dict(cls, d: dict[str, Any]) -> Transform:
        """Construct from a dict with translation, rotation, and frame metadata."""
        return cls(
            translation=Vector3.from_dict(d.get("translation", {})),
            rotation=Quaternion.from_dict(d.get("rotation", {})),
            frame_id=d.get("frame_id", GEOMETRY_MAP_FRAME_ID),
            child_frame_id=d.get("child_frame_id", GEOMETRY_BODY_FRAME_ID),
            ts=d.get("ts", 0.0),
        )

    _HEADER = struct.Struct("<dHH")  # ts (8) + frame_id len (2) + child len (2) = 12

    def encode(self) -> bytes:
        """Binary encode with header: ts + frame_id + child_frame_id + translation + rotation."""
        fid = self.frame_id.encode("utf-8")
        cid = self.child_frame_id.encode("utf-8")
        header = self._HEADER.pack(self.ts, len(fid), len(cid)) + fid + cid
        return header + self.translation.encode() + self.rotation.encode()

    @classmethod
    def decode(cls, data: bytes) -> Transform:
        """Binary decode from encoded bytes."""
        ts, fid_len, cid_len = cls._HEADER.unpack(data[: cls._HEADER.size])
        off = cls._HEADER.size
        frame_id = data[off : off + fid_len].decode("utf-8")
        off += fid_len
        child_frame_id = data[off : off + cid_len].decode("utf-8")
        off += cid_len
        t = Vector3.decode(data[off : off + 24])
        off += 24
        r = Quaternion.decode(data[off : off + 32])
        return cls(t, r, frame_id=frame_id, child_frame_id=child_frame_id, ts=ts)
