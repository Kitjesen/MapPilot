"""DatasetRecorderModule — 机器狗端 RGB-D 关键帧录制器（无需服务器）。

在机器狗漫游时，按运动阈值（位移 / 旋转 / 时间）筛选关键帧，
将每帧的彩色图、深度图、位姿和内参写入磁盘，格式与本地重建工具兼容。

输出目录结构:
    <save_dir>/
    ├── images/
    │   ├── 000000.jpg   — JPEG 彩色帧
    │   ├── 000001.jpg
    │   └── ...
    ├── depths/
    │   ├── 000000.png   — 16-bit PNG 深度图（单位：mm）
    │   └── ...
    ├── transforms.json  — nerfstudio / COLMAP 兼容格式（实时更新）
    └── metadata.json    — 录制会话信息（内参、机器人型号、时间戳等）

该格式可直接送入:
    python tools/reconstruction/reconstruct_local.py --dataset <save_dir> --backend tsdf
    python tools/reconstruction/reconstruct_local.py --dataset <save_dir> --backend nerfstudio
    ns-train instant-ngp --data <save_dir>    (nerfstudio 原生)

Ports:
    In: color_image (Image)
        depth_image (Image)
        camera_info (CameraIntrinsics)
        odometry (Odometry)
    Out: recorder_stats (dict)   — {frames, path, ...}
"""

from __future__ import annotations

import json
import logging
import math
import threading
import time
from pathlib import Path
from typing import Any

from runtime import In, Module, Out
from runtime.msgs.nav import Odometry
from runtime.msgs.numpy_compat import np
from runtime.msgs.sensor import CameraIntrinsics, Image, ImageFormat
from runtime.registry import register

from .reconstruction_module import _load_cam_body_extrinsic, _pose_to_matrix

logger = logging.getLogger(__name__)

_DEFAULT_SAVE_DIR   = "datasets/recording"
_DEFAULT_KF_DIST_M  = 0.15     # 最小关键帧间距（m）
_DEFAULT_KF_ROT_RAD = 0.17     # 最小关键帧旋转（rad，约 10°）
_DEFAULT_KF_TIME_S  = 1.0      # 最大间隔（即使不动也每 N 秒记一帧）
_DEFAULT_MAX_DEPTH  = 6.0      # 深度超过此值截断为 0（m）
_DEFAULT_JPEG_Q     = 90       # JPEG 质量（1-100）


def _angle_diff(a: float, b: float) -> float:
    d = a - b
    while d > math.pi:
        d -= 2 * math.pi
    while d < -math.pi:
        d += 2 * math.pi
    return d


@register(
    "reconstruction",
    "dataset_recorder",
    description="RGB-D keyframe dataset recorder",
)
class DatasetRecorderModule(Module, layer=3):
    """将机器狗漫游过程录制为本地 RGB-D 关键帧数据集。

    配置键（均有默认值）:
        save_dir         str    "datasets/recording"
        keyframe_dist_m  float  0.15   — 最小关键帧位移（m）
        keyframe_rot_rad float  0.17   — 最小关键帧旋转（rad，≈10°）
        keyframe_time_s  float  1.0    — 最大关键帧间隔（s）
        max_depth_m      float  6.0    — 深度截断（m）
        jpeg_quality     int    90     — JPEG 质量
        max_frames       int    0      — 最多录制帧数，0=无限制
        recording        bool   True   — 设 False 暂停录制（可运行时动态切换）
        session_name     str    ""     — 子目录名（空则用时间戳）
    """

    color_image: In[Image]
    depth_image: In[Image]
    camera_info: In[CameraIntrinsics]
    odometry: In[Odometry]
    recorder_stats: Out[dict]

    def __init__(self, **config: Any) -> None:
        super().__init__(**config)

        base_dir    = Path(config.get("save_dir", _DEFAULT_SAVE_DIR))
        session     = str(config.get("session_name", "")) or \
                      time.strftime("%Y%m%d_%H%M%S")
        self._save_dir = base_dir / session

        self._kf_dist    = float(config.get("keyframe_dist_m",  _DEFAULT_KF_DIST_M))
        self._kf_rot     = float(config.get("keyframe_rot_rad", _DEFAULT_KF_ROT_RAD))
        self._kf_time    = float(config.get("keyframe_time_s",  _DEFAULT_KF_TIME_S))
        self._max_depth  = float(config.get("max_depth_m",      _DEFAULT_MAX_DEPTH))
        self._jpeg_q     = int(config.get("jpeg_quality",       _DEFAULT_JPEG_Q))
        self._max_frames = int(config.get("max_frames", 0))
        self.recording   = bool(config.get("recording", True))

        self._camera_pose_in_body = _load_cam_body_extrinsic()

        # 缓冲（最新帧优先）
        self._latest_color:  Image | None            = None
        self._latest_depth:  Image | None            = None
        self._latest_odom:   Odometry | None         = None
        self._intrinsics:    CameraIntrinsics | None = None
        self._buf_lock = threading.Lock()

        # 关键帧选择状态
        self._last_kf_pos:  np.ndarray | None = None
        self._last_kf_yaw:  float = 0.0
        self._last_kf_time: float = 0.0
        self._frame_count:  int   = 0

        # 用于写入 transforms.json 的帧列表（追加模式）
        self._transforms_frames: list[dict] = []
        self._meta_written = False

    # ── 生命周期 ────────────────────────────────────────────────────────────

    def setup(self) -> None:
        (self._save_dir / "images").mkdir(parents=True, exist_ok=True)
        (self._save_dir / "depths").mkdir(parents=True, exist_ok=True)
        logger.info("DatasetRecorderModule: saving to %s", self._save_dir)

        self.color_image.subscribe(self._on_color)
        self.depth_image.subscribe(self._on_depth)
        self.camera_info.subscribe(self._on_camera_info)
        self.odometry.subscribe(self._on_odom)

    # ── Port 回调 ───────────────────────────────────────────────────────────

    def _on_color(self, img: Image) -> None:
        with self._buf_lock:
            self._latest_color = img

    def _on_depth(self, img: Image) -> None:
        with self._buf_lock:
            self._latest_depth = img

    def _on_camera_info(self, info: CameraIntrinsics) -> None:
        if self._intrinsics is None:
            self._intrinsics = info
            self._write_metadata(info)

    def _on_odom(self, odom: Odometry) -> None:
        with self._buf_lock:
            self._latest_odom = odom
        self._try_capture(odom)

    # ── 关键帧选择与保存 ────────────────────────────────────────────────────

    def _try_capture(self, odom: Odometry) -> None:
        if not self.recording:
            return
        if self._max_frames > 0 and self._frame_count >= self._max_frames:
            return

        with self._buf_lock:
            color = self._latest_color
            depth = self._latest_depth

        if color is None or depth is None:
            return

        now = time.time()
        pos = np.array([odom.pose.position.x,
                        odom.pose.position.y,
                        odom.pose.position.z])
        yaw = odom.pose.yaw

        # 关键帧判定
        if self._last_kf_pos is not None:
            dist = float(np.linalg.norm(pos - self._last_kf_pos))
            rot  = abs(_angle_diff(yaw, self._last_kf_yaw))
            dt   = now - self._last_kf_time
            if dist < self._kf_dist and rot < self._kf_rot and dt < self._kf_time:
                return

        self._last_kf_pos  = pos.copy()
        self._last_kf_yaw  = yaw
        self._last_kf_time = now

        self._save_frame(color, depth, odom, now)

    def _save_frame(
        self,
        color: Image,
        depth: Image,
        odom:  Odometry,
        ts:    float,
    ) -> None:
        idx     = self._frame_count
        img_dir = self._save_dir / "images"
        dep_dir = self._save_dir / "depths"

        # ── 保存彩色图 ──────────────────────────────────────────────────────
        color_path = img_dir / f"{idx:06d}.jpg"
        try:
            import cv2  # type: ignore
            bgr = color.data
            if color.format.value == "RGB":
                bgr = bgr[..., ::-1].copy()
            elif color.format.value == "GRAY":
                bgr = np.stack([bgr] * 3, axis=-1)
            cv2.imwrite(str(color_path), bgr,
                        [cv2.IMWRITE_JPEG_QUALITY, self._jpeg_q])
        except ImportError:
            # 无 cv2：直接写原始字节（格式可能不正确，但不丢帧）
            color_path.write_bytes(color.data.tobytes())
        except Exception:
            logger.warning("Failed to save color frame %d", idx, exc_info=True)
            return

        # ── 保存深度图（16-bit PNG，单位 mm）───────────────────────────────
        depth_path = dep_dir / f"{idx:06d}.png"
        try:
            import cv2  # type: ignore
            data = depth.data
            if depth.format == ImageFormat.DEPTH_F32:
                depth_mm = (data * 1000.0).clip(0, self._max_depth * 1000).astype(np.uint16)
            else:
                depth_mm = data.astype(np.uint16)
                depth_mm[depth_mm > int(self._max_depth * 1000)] = 0
            cv2.imwrite(str(depth_path), depth_mm)
        except ImportError:
            depth_path.write_bytes(depth.data.tobytes())
        except Exception:
            logger.warning("Failed to save depth frame %d", idx, exc_info=True)
            return

        # ── 计算相机到世界变换 ──────────────────────────────────────────────
        body_to_world   = _pose_to_matrix(odom)
        cam_to_world_cv = body_to_world @ self._camera_pose_in_body   # OpenCV 约定
        # nerfstudio / transforms.json 使用 OpenGL 约定（Y 朝上，Z 朝后）
        cam_to_world_gl = cam_to_world_cv @ np.diag([1., -1., -1., 1.])

        frame_entry = {
            "file_path":       f"images/{idx:06d}.jpg",
            "depth_file_path": f"depths/{idx:06d}.png",
            "transform_matrix": cam_to_world_gl.tolist(),
            "timestamp": ts,
        }
        self._transforms_frames.append(frame_entry)
        self._frame_count = idx + 1

        # ── 更新 transforms.json（每帧写一次，避免录制中途崩溃丢数据）──────
        self._flush_transforms()

        # ── 发布统计 ────────────────────────────────────────────────────────
        self.recorder_stats.publish({
            "frames":   self._frame_count,
            "save_dir": str(self._save_dir),
            "last_ts":  ts,
        })

        if self._frame_count % 50 == 0:
            logger.info("Recorder: %d keyframes saved to %s",
                        self._frame_count, self._save_dir)

    def _flush_transforms(self) -> None:
        """将当前 transforms 列表原子写入 transforms.json。"""
        intr = self._intrinsics
        payload = {
            "camera_model": "OPENCV",
            "fl_x": float(intr.fx)    if intr else 615.0,
            "fl_y": float(intr.fy)    if intr else 615.0,
            "cx":   float(intr.cx)    if intr else 320.0,
            "cy":   float(intr.cy)    if intr else 240.0,
            "w":    int(intr.width)   if intr else 640,
            "h":    int(intr.height)  if intr else 480,
            "k1": 0.0, "k2": 0.0, "p1": 0.0, "p2": 0.0,
            "depth_unit_scale_factor": 0.001,  # mm → m（深度图单位 mm）
            "frames": self._transforms_frames,
        }
        tmp = self._save_dir / "transforms.json.tmp"
        dst = self._save_dir / "transforms.json"
        tmp.write_text(json.dumps(payload, indent=2))
        tmp.rename(dst)

    def _write_metadata(self, intr: CameraIntrinsics) -> None:
        if self._meta_written:
            return
        meta = {
            "recorder":   "DatasetRecorderModule",
            "created_at": time.strftime("%Y-%m-%dT%H:%M:%S"),
            "intrinsics": {
                "fx": intr.fx, "fy": intr.fy,
                "cx": intr.cx, "cy": intr.cy,
                "w":  intr.width, "h": intr.height,
                "depth_scale": intr.depth_scale,
            },
            "keyframe_dist_m":  self._kf_dist,
            "keyframe_rot_rad": self._kf_rot,
            "keyframe_time_s":  self._kf_time,
            "max_depth_m":      self._max_depth,
        }
        (self._save_dir / "metadata.json").write_text(
            json.dumps(meta, indent=2)
        )
        self._meta_written = True

    # ── 对外接口 ────────────────────────────────────────────────────────────

    def start_recording(self) -> None:
        """开始/恢复录制。"""
        self.recording = True
        logger.info("Recorder: started")

    def stop_recording(self) -> None:
        """暂停录制（不清除缓冲，随时可恢复）。"""
        self.recording = False
        logger.info("Recorder: stopped at %d frames", self._frame_count)

    def reset(self, new_session: str = "") -> None:
        """重置为新会话（创建新子目录）。"""
        session = new_session or time.strftime("%Y%m%d_%H%M%S")
        self._save_dir = self._save_dir.parent / session
        (self._save_dir / "images").mkdir(parents=True, exist_ok=True)
        (self._save_dir / "depths").mkdir(parents=True, exist_ok=True)
        self._frame_count = 0
        self._transforms_frames.clear()
        self._last_kf_pos  = None
        self._meta_written = False
        if self._intrinsics:
            self._write_metadata(self._intrinsics)
        logger.info("Recorder: reset to new session %s", self._save_dir)

    def health(self) -> dict[str, Any]:
        info = super().port_summary()
        info.update({
            "save_dir":   str(self._save_dir),
            "frames":     self._frame_count,
            "recording":  self.recording,
        })
        return info

    @property
    def dataset_path(self) -> Path:
        return self._save_dir

    @property
    def frame_count(self) -> int:
        return self._frame_count
