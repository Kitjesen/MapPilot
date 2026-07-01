"""Pure Module endpoint driver for externally owned simulation streams.

The external process owns physics, raw sensor transport, and any compatibility
middleware.  This module only exposes driver-shaped LingTu ports and accepts
already-normalized odometry/cloud callbacks from that process.
"""

from __future__ import annotations

import logging
import struct
import time
from collections.abc import Callable
from typing import Any

from runtime.module import Module
from runtime.msgs.geometry import Pose, PoseStamped, Quaternion, Twist, Vector3
from runtime.msgs.nav import Odometry
from runtime.msgs.numpy_compat import np
from runtime.msgs.sensor import CameraIntrinsics, Image, ImageFormat, PointCloud2
from runtime.registry import register
from runtime.runtime_interface import (
    TOPICS,
    runtime_fixed_path_frame_ids,
    topic_default_frame_id,
)
from runtime.stream import In, Out

logger = logging.getLogger(__name__)

_XYZI_POINT_STEP = 16

SIM_ENDPOINT_ODOM_FRAME_ID = topic_default_frame_id(TOPICS.odometry)
SIM_ENDPOINT_REGISTERED_CLOUD_FRAME_ID = topic_default_frame_id(TOPICS.registered_cloud)
SIM_ENDPOINT_LIVE_MAP_CLOUD_FRAME_ID = SIM_ENDPOINT_ODOM_FRAME_ID
SIM_ENDPOINT_GOAL_FRAME_ID = runtime_fixed_path_frame_ids()[0]
SIM_ENDPOINT_BODY_FRAME_ID = topic_default_frame_id(TOPICS.cmd_vel)


@register("driver", "sim_endpoint", description="Pure endpoint simulation driver")
@register("driver_protocol", "sim_endpoint", description="Pure endpoint simulation driver")
class SimEndpointDriverModule(Module, layer=1):
    """Driver-shaped endpoint for externally injected sim odom/cloud streams."""

    cmd_vel: In[Twist]
    stop_signal: In[int]

    odometry: Out[Odometry]
    lidar_cloud: Out[PointCloud2]
    map_cloud: Out[PointCloud2]
    camera_image: Out[Image]
    depth_image: Out[Image]
    camera_info: Out[CameraIntrinsics]
    goal_pose: Out[PoseStamped]
    alive: Out[bool]
    robot_state: Out[dict]

    def __init__(
        self,
        *,
        latch_stop_signal: bool = True,
        command_callback: Callable[[Twist], None] | None = None,
        **kw: Any,
    ) -> None:
        super().__init__(**kw)
        self._latch_stop_signal = bool(latch_stop_signal)
        self._command_callback = command_callback
        self._running = False
        self._stopped = False
        self._cmd_vx = 0.0
        self._cmd_vy = 0.0
        self._cmd_wz = 0.0
        self._odom_count = 0
        self._registered_cloud_count = 0
        self._map_cloud_count = 0
        self._camera_image_count = 0
        self._depth_image_count = 0
        self._camera_info_count = 0
        self._cloud_conversion_errors = 0
        self._image_conversion_errors = 0

    def setup(self) -> None:
        self.cmd_vel.subscribe(self._on_cmd_vel)
        self.stop_signal.subscribe(self._on_stop)

    def start(self) -> None:
        super().start()
        self._running = True
        self.alive.publish(True)
        self._publish_robot_state()

    def stop(self) -> None:
        self._running = False
        self.alive.publish(False)
        self._publish_robot_state()
        super().stop()

    def set_command_callback(self, callback: Callable[[Twist], None] | None) -> None:
        self._command_callback = callback

    def latest_command(self) -> dict[str, float]:
        return {
            "vx": self._cmd_vx,
            "vy": self._cmd_vy,
            "wz": self._cmd_wz,
        }

    def publish_odometry_from_ros(self, msg: Any) -> None:
        """Publish a core Odometry message from a ROS-like odometry object."""

        odom = _odometry_from_ros_like(msg)
        self._odom_count += 1
        self.odometry.publish(odom)

    def publish_registered_cloud_from_ros(self, msg: Any) -> None:
        """Publish a core lidar cloud from a ROS-like PointCloud2 object."""

        cloud = self._pointcloud_from_ros_like(
            msg,
            default_frame=SIM_ENDPOINT_REGISTERED_CLOUD_FRAME_ID,
        )
        if cloud is None:
            return
        self._registered_cloud_count += 1
        self.lidar_cloud.publish(cloud)

    def publish_map_cloud_from_ros(self, msg: Any) -> None:
        """Publish a core map cloud from a ROS-like PointCloud2 object."""

        cloud = self._pointcloud_from_ros_like(
            msg,
            default_frame=SIM_ENDPOINT_LIVE_MAP_CLOUD_FRAME_ID,
        )
        if cloud is None:
            return
        self._map_cloud_count += 1
        self.map_cloud.publish(cloud)

    def publish_goal_pose_from_ros(self, msg: Any) -> None:
        pose = getattr(msg, "pose", None)
        if pose is None:
            return
        position = getattr(pose, "position", None)
        orientation = getattr(pose, "orientation", None)
        self.goal_pose.publish(
            PoseStamped(
                pose=Pose(
                    position=Vector3(
                        float(getattr(position, "x", 0.0)),
                        float(getattr(position, "y", 0.0)),
                        float(getattr(position, "z", 0.0)),
                    ),
                    orientation=Quaternion(
                        float(getattr(orientation, "x", 0.0)),
                        float(getattr(orientation, "y", 0.0)),
                        float(getattr(orientation, "z", 0.0)),
                        float(getattr(orientation, "w", 1.0)),
                    ),
                ),
                frame_id=_header_frame_id(msg) or SIM_ENDPOINT_GOAL_FRAME_ID,
                ts=time.time(),
            )
        )

    def publish_camera_image_from_ros(self, msg: Any) -> None:
        image = self._image_from_ros_like(msg, default_format=ImageFormat.RGB)
        if image is None:
            return
        self._camera_image_count += 1
        self.camera_image.publish(image)

    def publish_depth_image_from_ros(self, msg: Any) -> None:
        image = self._image_from_ros_like(msg, default_format=ImageFormat.DEPTH_F32)
        if image is None:
            return
        self._depth_image_count += 1
        self.depth_image.publish(image)

    def publish_camera_info_from_ros(self, msg: Any) -> None:
        matrix = list(getattr(msg, "k", getattr(msg, "K", [])) or [])
        dist = list(getattr(msg, "d", getattr(msg, "D", [])) or [])
        fx = _matrix_value(matrix, 0)
        fy = _matrix_value(matrix, 4)
        cx = _matrix_value(matrix, 2)
        cy = _matrix_value(matrix, 5)
        self._camera_info_count += 1
        self.camera_info.publish(
            CameraIntrinsics(
                fx=fx,
                fy=fy,
                cx=cx,
                cy=cy,
                width=int(getattr(msg, "width", 0) or 0),
                height=int(getattr(msg, "height", 0) or 0),
                dist_k1=_matrix_value(dist, 0),
                dist_k2=_matrix_value(dist, 1),
                dist_p1=_matrix_value(dist, 2),
                dist_p2=_matrix_value(dist, 3),
                dist_k3=_matrix_value(dist, 4),
            )
        )

    def _pointcloud_from_ros_like(
        self,
        msg: Any,
        *,
        default_frame: str,
    ) -> PointCloud2 | None:
        try:
            n_pts = int(getattr(msg, "width", 0)) * int(getattr(msg, "height", 1))
            if n_pts <= 0:
                return None

            raw = bytes(getattr(msg, "data", b""))
            step = int(getattr(msg, "point_step", 0))
            if step <= 0:
                return None

            if step == _XYZI_POINT_STEP:
                arr = np.frombuffer(raw, dtype=np.float32).reshape(n_pts, 4)
                pts = np.ascontiguousarray(arr[:, :3])
            else:
                offsets = _parse_xyz_offsets(getattr(msg, "fields", []))
                if offsets is None:
                    logger.warning("SimEndpointDriverModule: cannot parse PointCloud2 fields")
                    return None
                ox, oy, oz = offsets
                pts = np.zeros((n_pts, 3), dtype=np.float32)
                for i in range(n_pts):
                    base = i * step
                    pts[i, 0] = struct.unpack_from("<f", raw, base + ox)[0]
                    pts[i, 1] = struct.unpack_from("<f", raw, base + oy)[0]
                    pts[i, 2] = struct.unpack_from("<f", raw, base + oz)[0]

            pts = pts[np.isfinite(pts).all(axis=1)]
            if pts.shape[0] == 0:
                return None

            return PointCloud2(
                points=pts.copy(),
                frame_id=_header_frame_id(msg) or default_frame,
                ts=time.time(),
            )
        except Exception as exc:
            self._cloud_conversion_errors += 1
            logger.error("SimEndpointDriverModule: cloud conversion error: %s", exc)
            return None

    def _image_from_ros_like(
        self,
        msg: Any,
        *,
        default_format: ImageFormat,
    ) -> Image | None:
        try:
            height = int(getattr(msg, "height", 0) or 0)
            width = int(getattr(msg, "width", 0) or 0)
            if height <= 0 or width <= 0:
                return None

            encoding = str(getattr(msg, "encoding", "") or "").lower()
            raw = bytes(getattr(msg, "data", b""))
            dtype, channels, fmt = _image_encoding_shape(encoding, default_format)
            expected = height * width * channels * int(np.dtype(dtype).itemsize)
            if len(raw) < expected:
                return None

            arr = np.frombuffer(raw[:expected], dtype=dtype)
            if channels == 1:
                arr = arr.reshape((height, width))
            else:
                arr = arr.reshape((height, width, channels))
                if encoding == "bgra8":
                    arr = arr[..., :3]
                    fmt = ImageFormat.BGR
                elif encoding == "rgba8":
                    arr = arr[..., :3]
                    fmt = ImageFormat.RGB

            return Image(
                data=np.asarray(arr).copy(),
                format=fmt,
                frame_id=_header_frame_id(msg) or "",
                ts=time.time(),
            )
        except Exception as exc:
            self._image_conversion_errors += 1
            logger.error("SimEndpointDriverModule: image conversion error: %s", exc)
            return None

    def _on_cmd_vel(self, twist: Twist) -> None:
        if self._stopped:
            return
        self._cmd_vx = _component(getattr(twist, "linear", None), "x")
        self._cmd_vy = _component(getattr(twist, "linear", None), "y")
        self._cmd_wz = _component(getattr(twist, "angular", None), "z")
        if self._command_callback is not None:
            self._command_callback(twist)

    def _on_stop(self, level: int) -> None:
        if level >= 1:
            self._cmd_vx = 0.0
            self._cmd_vy = 0.0
            self._cmd_wz = 0.0
            self._stopped = self._latch_stop_signal
            if self._command_callback is not None:
                self._command_callback(Twist.zero())
            return
        self._stopped = False

    def _publish_robot_state(self) -> None:
        from drivers.sim import build_sim_robot_state

        self.robot_state.publish(build_sim_robot_state())

    def health(self) -> dict[str, Any]:
        info = super().port_summary()
        info["sim_endpoint"] = {
            "running": self._running,
            "stopped": self._stopped,
            "odom_count": self._odom_count,
            "registered_cloud_count": self._registered_cloud_count,
            "map_cloud_count": self._map_cloud_count,
            "camera_image_count": self._camera_image_count,
            "depth_image_count": self._depth_image_count,
            "camera_info_count": self._camera_info_count,
            "cloud_conversion_errors": self._cloud_conversion_errors,
            "image_conversion_errors": self._image_conversion_errors,
            "latest_command": self.latest_command(),
        }
        return info


def _odometry_from_ros_like(msg: Any) -> Odometry:
    pose_msg = getattr(getattr(msg, "pose", None), "pose", None)
    twist_msg = getattr(getattr(msg, "twist", None), "twist", None)
    position = getattr(pose_msg, "position", None)
    orientation = getattr(pose_msg, "orientation", None)
    linear = getattr(twist_msg, "linear", None)
    angular = getattr(twist_msg, "angular", None)
    return Odometry(
        pose=Pose(
            position=Vector3(
                _component(position, "x"),
                _component(position, "y"),
                _component(position, "z"),
            ),
            orientation=Quaternion(
                _component(orientation, "x"),
                _component(orientation, "y"),
                _component(orientation, "z"),
                _component(orientation, "w", 1.0),
            ),
        ),
        twist=Twist(
            linear=Vector3(
                _component(linear, "x"),
                _component(linear, "y"),
                _component(linear, "z"),
            ),
            angular=Vector3(
                _component(angular, "x"),
                _component(angular, "y"),
                _component(angular, "z"),
            ),
        ),
        ts=time.time(),
        frame_id=_header_frame_id(msg) or SIM_ENDPOINT_ODOM_FRAME_ID,
        child_frame_id=str(getattr(msg, "child_frame_id", "") or SIM_ENDPOINT_BODY_FRAME_ID),
    )


def _component(obj: Any, name: str, default: float = 0.0) -> float:
    return float(getattr(obj, name, default) if obj is not None else default)


def _matrix_value(values: list[Any], index: int, default: float = 0.0) -> float:
    try:
        return float(values[index])
    except (IndexError, TypeError, ValueError):
        return float(default)


def _image_encoding_shape(
    encoding: str,
    default_format: ImageFormat,
) -> tuple[Any, int, ImageFormat]:
    normalized = encoding.replace("-", "").replace("_", "")
    if normalized in {"rgb8"}:
        return np.uint8, 3, ImageFormat.RGB
    if normalized in {"bgr8"}:
        return np.uint8, 3, ImageFormat.BGR
    if normalized in {"rgba8"}:
        return np.uint8, 4, ImageFormat.RGB
    if normalized in {"bgra8"}:
        return np.uint8, 4, ImageFormat.BGR
    if normalized in {"mono8", "8uc1"}:
        return np.uint8, 1, ImageFormat.GRAY
    if normalized in {"16uc1", "mono16"}:
        return np.uint16, 1, ImageFormat.DEPTH_U16
    if normalized in {"32fc1"}:
        return np.float32, 1, ImageFormat.DEPTH_F32
    if default_format is ImageFormat.DEPTH_F32:
        return np.float32, 1, default_format
    if default_format is ImageFormat.DEPTH_U16:
        return np.uint16, 1, default_format
    return np.uint8, 3, default_format


def _header_frame_id(msg: Any) -> str:
    return str(getattr(getattr(msg, "header", None), "frame_id", "") or "")


def _parse_xyz_offsets(fields: Any) -> tuple[int, int, int] | None:
    offsets: dict[str, int] = {}
    for field in fields or []:
        name = str(getattr(field, "name", "")).lower()
        if name in {"x", "y", "z"}:
            offsets[name] = int(getattr(field, "offset", 0))
    if len(offsets) < 3:
        return None
    return offsets["x"], offsets["y"], offsets["z"]
