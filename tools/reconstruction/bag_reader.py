"""bag_reader.py — 从 ROS2 bag 或点云 bag 提取 RGB-D 关键帧。

支持两类 bag：

1. **RGB-D bag** — 含彩色图 + 深度图 + 里程计
       典型话题:
           /camera/color/image_raw        (sensor_msgs/Image)
           /camera/depth/image_raw        (sensor_msgs/Image)
           /camera/color/camera_info      (sensor_msgs/CameraInfo)
           /slam/odometry / /Odometry      (nav_msgs/Odometry)

2. **点云 bag** — 含 LiDAR 点云 + 里程计（可选）
       典型话题:
           /cloud_registered / /lidar/raw_frame  (sensor_msgs/PointCloud2)
           /slam/odometry / /Odometry        (nav_msgs/Odometry)
       → 点云会被直接拼合为 PLY，不经过体素重建

使用方式:
    from tools.reconstruction.bag_reader import read_rgb_d_bag, read_lidar_bag

    # RGB-D bag → Keyframe 列表（再送入任意重建后端）
    keyframes = read_rgb_d_bag("~/data/run1.db3", output_dir="~/data/run1_frames/")

    # 点云 bag → 直接写 PLY
    read_lidar_bag("~/data/run1.db3", output_ply="~/data/run1_map.ply")

Requirements（仅在使用时才导入，不作为硬依赖）:
    pip install rosbag2-py               # ROS2 bag 读取
    pip install rclpy                    # ROS2 消息反序列化
    # 或: pip install ros2bag-reader     # 非 ROS 环境下的轻量读取器

注意: 如果不在 ROS2 环境中，可用 rosbags（纯 Python）替代:
    pip install rosbags                  # https://github.com/rpng/rosbags
"""

from __future__ import annotations

import json
import logging
import math
import struct
import time
from pathlib import Path
from typing import Any

import numpy as np

from perception.reconstruction.server.backends.base import Keyframe

logger = logging.getLogger(__name__)

# 默认话题名
_DEFAULT_COLOR_TOPICS  = ["/camera/color/image_raw", "/camera/rgb/image_raw",
                           "/camera/image_color"]
_DEFAULT_DEPTH_TOPICS  = ["/camera/depth/image_raw", "/camera/depth/image",
                           "/camera/aligned_depth_to_color/image_raw"]
_DEFAULT_INFO_TOPICS   = ["/camera/color/camera_info", "/camera/rgb/camera_info"]
_DEFAULT_ODOM_TOPICS   = ["/slam/odometry", "/Odometry",
                           "/nav/dog_odometry", "/odom"]
_DEFAULT_CLOUD_TOPICS  = ["/cloud_registered", "/cloud_map",
                           "/lidar/raw_frame", "/livox/lidar"]


# ── 主入口：RGB-D bag ─────────────────────────────────────────────────────────

def read_rgb_d_bag(
    bag_path: str | Path,
    output_dir: str | Path,
    *,
    color_topic:  str | None = None,
    depth_topic:  str | None = None,
    info_topic:   str | None = None,
    odom_topic:   str | None = None,
    keyframe_dist_m:  float = 0.15,
    keyframe_rot_rad: float = 0.17,
    keyframe_time_s:  float = 1.0,
    max_depth_m: float = 6.0,
    jpeg_quality: int  = 90,
    max_frames:  int   = 0,
) -> list[Keyframe]:
    """从 ROS2 RGB-D bag 提取关键帧并写入磁盘。

    参数
    ----
    bag_path     : ROS2 bag 目录（含 metadata.yaml 和 *.db3）或 .db3 文件路径
    output_dir   : 输出目录（同 DatasetRecorderModule 格式）
    *_topic      : 手动指定话题名，None = 自动检测
    keyframe_*   : 关键帧筛选阈值（与 DatasetRecorderModule 相同）
    max_frames   : 最多提取帧数，0 = 全部

    返回
    ----
    list[Keyframe]
    """
    bag_path   = Path(bag_path).expanduser()
    output_dir = Path(output_dir).expanduser()
    (output_dir / "images").mkdir(parents=True, exist_ok=True)
    (output_dir / "depths").mkdir(exist_ok=True)

    reader = _open_bag(bag_path)
    if reader is None:
        raise RuntimeError(
            f"Cannot open bag {bag_path}. "
            f"Make sure rosbags or rosbag2_py is installed:\n"
            f"  pip install rosbags     # pure Python, recommended\n"
            f"  # or source /opt/ros/humble/setup.bash"
        )

    # 探测话题
    topics = reader.topics
    color_t = color_topic or _first_match(topics, _DEFAULT_COLOR_TOPICS)
    depth_t = depth_topic or _first_match(topics, _DEFAULT_DEPTH_TOPICS)
    info_t  = info_topic  or _first_match(topics, _DEFAULT_INFO_TOPICS)
    odom_t  = odom_topic  or _first_match(topics, _DEFAULT_ODOM_TOPICS)

    logger.info("bag_reader: color=%s depth=%s odom=%s", color_t, depth_t, odom_t)
    if not color_t:
        raise ValueError(
            f"No color image topic found in bag. Topics: {list(topics)}\n"
            f"Use color_topic= to specify manually."
        )

    # 缓冲最近消息
    latest_color:  Any | None = None
    latest_depth:  Any | None = None
    latest_odom:   Any | None = None
    intrinsics:    dict | None = None

    # 关键帧选择状态
    last_kf_pos  = None
    last_kf_yaw  = 0.0
    last_kf_time = 0.0
    frame_count  = 0
    transforms_frames: list[dict] = []

    for topic, msg, ts_ns in reader.messages(
        topics=[t for t in [color_t, depth_t, info_t, odom_t] if t]
    ):
        ts = ts_ns * 1e-9

        if topic == color_t:
            latest_color = (msg, ts)
        elif topic == depth_t:
            latest_depth = (msg, ts)
        elif topic == info_t and intrinsics is None:
            intrinsics = _parse_camera_info(msg)
        elif topic == odom_t:
            latest_odom  = (msg, ts)

        # 每次收到里程计触发关键帧判定
        if topic != odom_t or latest_color is None:
            continue

        odom_msg, odom_ts = latest_odom
        pos, yaw = _parse_odom_pos_yaw(odom_msg)

        # 关键帧判定
        if last_kf_pos is not None:
            dist = float(np.linalg.norm(pos - last_kf_pos))
            rot  = abs(_angle_diff(yaw, last_kf_yaw))
            dt   = odom_ts - last_kf_time
            if dist < keyframe_dist_m and rot < keyframe_rot_rad and dt < keyframe_time_s:
                continue

        last_kf_pos  = pos.copy()
        last_kf_yaw  = yaw
        last_kf_time = odom_ts

        # 提取帧
        if max_frames > 0 and frame_count >= max_frames:
            break

        if intrinsics is None:
            intrinsics = {"fx": 615, "fy": 615, "cx": 320, "cy": 240, "w": 640, "h": 480}

        # 保存彩色图
        color_msg, color_ts = latest_color
        color_path = output_dir / "images" / f"{frame_count:06d}.jpg"
        try:
            import cv2  # type: ignore
            bgr = _decode_image_msg(color_msg, bgr=True)
            cv2.imwrite(str(color_path), bgr, [cv2.IMWRITE_JPEG_QUALITY, jpeg_quality])
        except Exception:
            logger.warning("Frame %d: cannot save color", frame_count, exc_info=True)
            continue

        # 保存深度图
        depth_path = output_dir / "depths" / f"{frame_count:06d}.png"
        if latest_depth is not None:
            depth_msg, _ = latest_depth
            try:
                import cv2  # type: ignore
                depth_arr = _decode_depth_msg(depth_msg, max_depth_m=max_depth_m)
                cv2.imwrite(str(depth_path), depth_arr)
            except Exception:
                pass

        # 计算相机到世界变换
        R_body, t_body = _parse_odom_Rt(odom_msg)
        T_body = np.eye(4, dtype=np.float64)
        T_body[:3, :3] = R_body
        T_body[:3, 3]  = t_body
        cam_to_world = T_body   # TODO: compose with body→cam extrinsic if available
        T_gl = cam_to_world @ np.diag([1., -1., -1., 1.])

        transforms_frames.append({
            "file_path":        f"images/{frame_count:06d}.jpg",
            "depth_file_path":  f"depths/{frame_count:06d}.png",
            "transform_matrix": T_gl.tolist(),
            "timestamp": odom_ts,
        })
        frame_count += 1

        if frame_count % 50 == 0:
            logger.info("bag_reader: extracted %d keyframes", frame_count)
            _flush_transforms_json(output_dir, intrinsics, transforms_frames)

    reader.close()

    _flush_transforms_json(output_dir, intrinsics, transforms_frames)
    _write_metadata(output_dir, intrinsics, keyframe_dist_m, keyframe_rot_rad,
                    keyframe_time_s, max_depth_m)

    logger.info("bag_reader: done — %d keyframes saved to %s",
                frame_count, output_dir)

    # 返回 Keyframe 对象列表
    from perception.reconstruction.dataset_io import load_dataset
    return load_dataset(output_dir)


# ── 主入口：点云 bag ──────────────────────────────────────────────────────────

def read_lidar_bag(
    bag_path: str | Path,
    output_ply: str | Path,
    *,
    cloud_topic: str | None = None,
    odom_topic:  str | None = None,
    voxel_size:  float = 0.05,
    max_points:  int   = 5_000_000,
) -> Path:
    """从 ROS2 点云 bag 提取所有点云帧，合并写入 PLY。

    参数
    ----
    bag_path    : ROS2 bag 目录或 .db3 文件
    output_ply  : 输出 PLY 文件路径
    cloud_topic : 手动指定点云话题（None = 自动）
    voxel_size  : 合并前体素降采样（0 = 不降采样）
    max_points  : 超过此数量时强制降采样

    返回
    ----
    Path  输出 PLY 路径
    """
    bag_path   = Path(bag_path).expanduser()
    output_ply = Path(output_ply).expanduser()
    output_ply.parent.mkdir(parents=True, exist_ok=True)

    reader = _open_bag(bag_path)
    if reader is None:
        raise RuntimeError("Cannot open bag — install rosbags: pip install rosbags")

    topics = reader.topics
    cloud_t = cloud_topic or _first_match(topics, _DEFAULT_CLOUD_TOPICS)
    odom_t  = odom_topic  or _first_match(topics, _DEFAULT_ODOM_TOPICS)

    if not cloud_t:
        raise ValueError(f"No point cloud topic found. Topics: {list(topics)}")
    logger.info("bag_reader(lidar): cloud=%s odom=%s", cloud_t, odom_t)

    all_points: list[np.ndarray] = []
    latest_odom: Any | None = None
    odom_map: dict[float, np.ndarray] = {}  # ts → T_body

    sub_topics = [t for t in [cloud_t, odom_t] if t]
    for topic, msg, ts_ns in reader.messages(topics=sub_topics):
        ts = ts_ns * 1e-9
        if topic == odom_t:
            R, t = _parse_odom_Rt(msg)
            T = np.eye(4)
            T[:3, :3] = R
            T[:3, 3]  = t
            odom_map[ts] = T
            latest_odom = T
        elif topic == cloud_t:
            try:
                pts = _decode_pointcloud2(msg)  # Nx3 float32
                if pts is None or len(pts) == 0:
                    continue
                # 变换到世界坐标
                T = latest_odom if latest_odom is not None else np.eye(4)
                ones = np.ones((len(pts), 1), dtype=np.float32)
                pts_h = np.hstack([pts[:, :3], ones])  # Nx4
                pts_w = (T @ pts_h.T).T[:, :3].astype(np.float32)
                all_points.append(pts_w)
            except Exception:
                pass

    reader.close()

    if not all_points:
        raise RuntimeError("No point cloud data extracted from bag")

    merged = np.vstack(all_points)
    logger.info("bag_reader(lidar): %d total points", len(merged))

    # 体素降采样
    if voxel_size > 0 or len(merged) > max_points:
        vs = voxel_size if voxel_size > 0 else 0.05
        keys = np.floor(merged / vs).astype(np.int32)
        _, idx = np.unique(keys, axis=0, return_index=True)
        merged = merged[idx]
        logger.info("bag_reader(lidar): %d points after voxel downsample", len(merged))

    # 写 PLY
    from perception.reconstruction.ply_writer import save_ply
    n = save_ply(merged, filepath=str(output_ply))
    logger.info("bag_reader(lidar): saved %d points to %s", n, output_ply)
    return output_ply


# ── Bag 读取后端（自动选择）────────────────────────────────────────────────────

class _BagReader:
    """通用 bag 读取接口，支持 rosbags（纯 Python）和 rosbag2_py（ROS2 原生）。"""
    def __init__(self): pass
    @property
    def topics(self) -> list[str]: return []
    def messages(self, topics=None): return iter([])
    def close(self): pass


def _open_bag(bag_path: Path) -> _BagReader | None:
    """尝试用 rosbags（首选）或 rosbag2_py 打开 bag，返回读取器。"""
    # 1. 优先用 rosbags（无需 ROS2 环境）
    try:
        return _RosbagsPyReader(bag_path)
    except ImportError:
        pass
    except Exception as exc:
        logger.debug("rosbags open failed: %s", exc)

    # 2. 回退到 rosbag2_py（需要 ROS2 源 setup.bash）
    try:
        return _Rosbag2PyReader(bag_path)
    except ImportError:
        pass
    except Exception as exc:
        logger.debug("rosbag2_py open failed: %s", exc)

    return None


class _RosbagsPyReader(_BagReader):
    """基于 rosbags 库（pip install rosbags）的读取器。"""

    def __init__(self, bag_path: Path):
        from rosbags.rosbag2 import Reader  # type: ignore
        self._reader = Reader(bag_path)
        self._reader.open()
        self._topics = list(self._reader.topics.keys())

    @property
    def topics(self) -> list[str]:
        return self._topics

    def messages(self, topics=None):
        from rosbags.serde import deserialize_cdr  # type: ignore
        conn_filter = (
            [c for c in self._reader.connections
             if c.topic in (topics or self._topics)]
            if topics else None
        )
        for connection, ts_ns, rawdata in self._reader.messages(
            connections=conn_filter
        ):
            try:
                msg = deserialize_cdr(rawdata, connection.msgtype)
                yield connection.topic, msg, ts_ns
            except Exception:
                continue

    def close(self):
        self._reader.close()


class _Rosbag2PyReader(_BagReader):
    """基于 rosbag2_py（ROS2 原生）的读取器。"""

    def __init__(self, bag_path: Path):
        import rosbag2_py  # type: ignore
        from rclpy.serialization import deserialize_message  # type: ignore

        self._deserialize = deserialize_message
        storage_opts = rosbag2_py.StorageOptions(
            uri=str(bag_path), storage_id="sqlite3"
        )
        conv_opts = rosbag2_py.ConverterOptions("", "")
        self._reader = rosbag2_py.SequentialReader()
        self._reader.open(storage_opts, conv_opts)
        meta = self._reader.get_metadata()
        self._topic_types = {
            t.topic_metadata.name: t.topic_metadata.type
            for t in meta.topics_with_message_count
        }
        self._topics = list(self._topic_types.keys())

    @property
    def topics(self) -> list[str]:
        return self._topics

    def messages(self, topics=None):
        import importlib
        filter_set = set(topics) if topics else None
        while self._reader.has_next():
            topic, data, ts_ns = self._reader.read_next()
            if filter_set and topic not in filter_set:
                continue
            msg_type = self._topic_types.get(topic, "")
            try:
                pkg, msg_name = msg_type.rsplit("/", 2)[::2]
                mod = importlib.import_module(f"{pkg}.msg")
                cls = getattr(mod, msg_name)
                msg = self._deserialize(data, cls)
                yield topic, msg, ts_ns
            except Exception:
                continue

    def close(self):
        pass


# ── 消息解析辅助 ──────────────────────────────────────────────────────────────

def _parse_camera_info(msg) -> dict:
    try:
        K = list(msg.k)
        return {
            "fx": K[0], "fy": K[4], "cx": K[2], "cy": K[5],
            "w": int(msg.width), "h": int(msg.height),
        }
    except Exception:
        return {"fx": 615, "fy": 615, "cx": 320, "cy": 240, "w": 640, "h": 480}


def _parse_odom_pos_yaw(msg) -> tuple[np.ndarray, float]:
    try:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = _quat_to_yaw(q.x, q.y, q.z, q.w)
        return np.array([p.x, p.y, p.z]), yaw
    except Exception:
        return np.zeros(3), 0.0


def _parse_odom_Rt(msg) -> tuple[np.ndarray, np.ndarray]:
    try:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        R = _quat_to_R_np(q.x, q.y, q.z, q.w)
        t = np.array([p.x, p.y, p.z])
        return R, t
    except Exception:
        return np.eye(3), np.zeros(3)


def _decode_image_msg(msg, bgr: bool = True) -> np.ndarray:
    """将 sensor_msgs/Image 解码为 numpy 数组。"""
    enc = getattr(msg, "encoding", "bgr8").lower()
    h, w = int(msg.height), int(msg.width)
    data = bytes(msg.data)
    if "bgr8" in enc or "rgb8" in enc:
        arr = np.frombuffer(data, dtype=np.uint8).reshape(h, w, 3)
        if bgr and "rgb8" in enc:
            arr = arr[..., ::-1].copy()
    elif "mono8" in enc or "gray" in enc:
        arr = np.frombuffer(data, dtype=np.uint8).reshape(h, w)
        if bgr:
            import cv2  # type: ignore
            arr = cv2.cvtColor(arr, cv2.COLOR_GRAY2BGR)
    else:
        arr = np.frombuffer(data, dtype=np.uint8).reshape(h, w, -1)
    return arr


def _decode_depth_msg(msg, max_depth_m: float = 6.0) -> np.ndarray:
    """将 sensor_msgs/Image（深度）解码为 uint16 mm 数组。"""
    enc = getattr(msg, "encoding", "16UC1").lower()
    h, w = int(msg.height), int(msg.width)
    data = bytes(msg.data)
    if "16uc1" in enc or "16u" in enc:
        arr = np.frombuffer(data, dtype=np.uint16).reshape(h, w)
    elif "32fc1" in enc or "32f" in enc:
        f = np.frombuffer(data, dtype=np.float32).reshape(h, w)
        arr = (f * 1000.0).clip(0, max_depth_m * 1000).astype(np.uint16)
    else:
        arr = np.frombuffer(data, dtype=np.uint16).reshape(h, w)
    arr[arr > int(max_depth_m * 1000)] = 0
    return arr


def _decode_pointcloud2(msg) -> np.ndarray | None:
    """将 sensor_msgs/PointCloud2 解码为 (N, 3) float32 XYZ 数组。"""
    try:
        data = bytes(msg.data)
        n     = int(msg.width) * int(msg.height)
        step  = int(msg.point_step)
        # 找 x, y, z 的 offset
        offsets = {}
        for f in msg.fields:
            if f.name in ("x", "y", "z"):
                offsets[f.name] = int(f.offset)
        if len(offsets) < 3:
            return None
        ox, oy, oz = offsets["x"], offsets["y"], offsets["z"]
        pts = np.zeros((n, 3), dtype=np.float32)
        for i in range(n):
            base = i * step
            pts[i, 0] = struct.unpack_from("<f", data, base + ox)[0]
            pts[i, 1] = struct.unpack_from("<f", data, base + oy)[0]
            pts[i, 2] = struct.unpack_from("<f", data, base + oz)[0]
        # 去掉 NaN / Inf
        valid = np.isfinite(pts).all(axis=1)
        return pts[valid]
    except Exception:
        return None


def _quat_to_yaw(qx, qy, qz, qw) -> float:
    siny = 2.0 * (qw * qz + qx * qy)
    cosy = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny, cosy)


def _quat_to_R_np(qx, qy, qz, qw) -> np.ndarray:
    return np.array([
        [1-2*(qy*qy+qz*qz), 2*(qx*qy-qz*qw), 2*(qx*qz+qy*qw)],
        [2*(qx*qy+qz*qw),   1-2*(qx*qx+qz*qz), 2*(qy*qz-qx*qw)],
        [2*(qx*qz-qy*qw),   2*(qy*qz+qx*qw),   1-2*(qx*qx+qy*qy)],
    ], dtype=np.float64)


def _angle_diff(a: float, b: float) -> float:
    d = a - b
    while d > math.pi:
        d -= 2 * math.pi
    while d < -math.pi:
        d += 2 * math.pi
    return d


def _first_match(topics: list[str], candidates: list[str]) -> str | None:
    for c in candidates:
        if c in topics:
            return c
    return None


def _flush_transforms_json(
    output_dir: Path, intrinsics: dict | None, frames: list[dict]
) -> None:
    intr = intrinsics or {}
    payload = {
        "camera_model": "OPENCV",
        "fl_x": float(intr.get("fx", 615.0)),
        "fl_y": float(intr.get("fy", 615.0)),
        "cx":   float(intr.get("cx", 320.0)),
        "cy":   float(intr.get("cy", 240.0)),
        "w":    int(intr.get("w",   640)),
        "h":    int(intr.get("h",   480)),
        "k1": 0.0, "k2": 0.0, "p1": 0.0, "p2": 0.0,
        "depth_unit_scale_factor": 0.001,
        "frames": frames,
    }
    tmp = output_dir / "transforms.json.tmp"
    dst = output_dir / "transforms.json"
    tmp.write_text(json.dumps(payload, indent=2))
    tmp.rename(dst)


def _write_metadata(
    output_dir: Path, intrinsics: dict | None,
    kf_dist: float, kf_rot: float, kf_time: float, max_depth: float,
) -> None:
    meta = {
        "recorder":         "bag_reader",
        "created_at":       time.strftime("%Y-%m-%dT%H:%M:%S"),
        "intrinsics":       intrinsics or {},
        "keyframe_dist_m":  kf_dist,
        "keyframe_rot_rad": kf_rot,
        "keyframe_time_s":  kf_time,
        "max_depth_m":      max_depth,
    }
    (output_dir / "metadata.json").write_text(json.dumps(meta, indent=2))
