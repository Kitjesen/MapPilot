"""MuJoCo camera rendering -- RGB + depth.

Implements offscreen RGB and depth rendering via mujoco.Renderer.
"""
import threading
from typing import Optional

import numpy as np

from sim.compat.engine.core.engine import CameraData
from sim.compat.engine.core.sensor import CameraConfig


class MuJoCoCamera:
    """MuJoCo camera renderer."""

    def __init__(self, model, config: CameraConfig) -> None:
        import mujoco

        self._model = model
        self._config = config
        self._camera_name = config.name

        cam_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, config.name)
        if cam_id < 0:
            raise ValueError(
                f"[MuJoCoCamera] Camera '{config.name}' not found in MuJoCo model. "
                f"Check robot.xml camera definitions."
            )
        self._cam_id = cam_id
        if config.fovy > 0:
            model.cam_fovy[cam_id] = float(config.fovy)
        self._mujoco = mujoco
        self._renderer_rgb: Optional[object] = None
        self._renderer_depth: Optional[object] = None
        self._renderer_thread_id: Optional[int] = None
        self._render_lock = threading.Lock()

        print(
            f"[MuJoCoCamera] Ready: {config.name} ({config.width}x{config.height}, "
            f"fovy={config.fovy}deg, depth={config.render_depth})"
        )

    def render(self, data) -> CameraData:
        """Render current frame and return CameraData."""
        with self._render_lock:
            self._ensure_renderers()
            self._renderer_rgb.update_scene(data, camera=self._camera_name)
            rgb = self._renderer_rgb.render().copy()

            if self._renderer_depth is not None:
                self._renderer_depth.update_scene(data, camera=self._camera_name)
                self._renderer_depth.enable_depth_rendering()
                depth_raw = self._renderer_depth.render().copy()
                self._renderer_depth.disable_depth_rendering()
                depth_meters = self._coerce_depth_meters(
                    depth_raw,
                    self._config.depth_near,
                    self._config.depth_far,
                )
            else:
                depth_meters = np.zeros((self._config.height, self._config.width), dtype=np.float32)

        return CameraData(
            rgb=rgb,
            depth=depth_meters,
            intrinsics=self._config.intrinsics,
            timestamp=float(data.time),
        )

    @staticmethod
    def _coerce_depth_meters(depth_raw: np.ndarray, near: float, far: float) -> np.ndarray:
        """Normalize MuJoCo depth output to metric depth.

        Newer MuJoCo Python renderers can already return scene-unit depth, while
        older paths may still expose a normalized 0..1 depth buffer. Accept both.
        """
        depth = np.asarray(depth_raw, dtype=np.float32)
        finite = np.isfinite(depth)
        if not finite.any():
            return np.full(depth.shape, far, dtype=np.float32)

        finite_vals = depth[finite]
        if float(finite_vals.max()) <= 1.5 and float(finite_vals.min()) >= 0.0:
            depth_meters = MuJoCoCamera._linearize_depth(depth, near, far)
        else:
            depth_meters = depth

        depth_meters = np.nan_to_num(
            depth_meters,
            nan=far,
            posinf=far,
            neginf=0.0,
        )
        return np.clip(depth_meters, 0.0, far).astype(np.float32)

    @staticmethod
    def _linearize_depth(depth_raw: np.ndarray, near: float, far: float) -> np.ndarray:
        """Convert normalized depth buffer to metric depth."""
        depth = near * far / (far - depth_raw * (far - near))
        return np.clip(depth, near, far).astype(np.float32)

    def get_rgb(self, data) -> np.ndarray:
        """Render RGB only, skip depth."""
        with self._render_lock:
            self._ensure_renderers()
            self._renderer_rgb.update_scene(data, camera=self._camera_name)
            return self._renderer_rgb.render().copy()

    def close(self) -> None:
        """Release renderer resources."""
        with self._render_lock:
            self._close_renderers()

    def _ensure_renderers(self) -> None:
        """Create renderers in the calling thread.

        MuJoCo EGL contexts are thread-affine on Linux, so offscreen renderers
        must be created in the same thread that later calls ``render()``.
        """
        thread_id = threading.get_ident()
        if self._renderer_rgb is not None and self._renderer_thread_id == thread_id:
            return

        self._close_renderers()
        self._renderer_rgb = self._mujoco.Renderer(
            self._model, self._config.height, self._config.width
        )
        self._renderer_depth = None
        if self._config.render_depth:
            self._renderer_depth = self._mujoco.Renderer(
                self._model, self._config.height, self._config.width
            )
        self._renderer_thread_id = thread_id

    def _close_renderers(self) -> None:
        if self._renderer_rgb is not None:
            self._renderer_rgb.close()
            self._renderer_rgb = None
        if self._renderer_depth is not None:
            self._renderer_depth.close()
            self._renderer_depth = None
        self._renderer_thread_id = None

    @property
    def config(self) -> CameraConfig:
        return self._config

    @property
    def cam_id(self) -> int:
        return self._cam_id
