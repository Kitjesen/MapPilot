"""MuJoCo camera rendering -- RGB + depth.

Implements offscreen RGB and depth rendering via mujoco.Renderer.
"""
import threading

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
        self._renderer: object | None = None
        self._renderer_thread_id: int | None = None
        self._snapshot_data: object | None = None
        self._render_lock = threading.Lock()

        print(
            f"[MuJoCoCamera] Ready: {config.name} ({config.width}x{config.height}, "
            f"fovy={config.fovy}deg, depth={config.render_depth})"
        )

    def render(self, data) -> CameraData:
        """Render current frame and return CameraData."""
        with self._render_lock:
            return self._render_locked(data)

    def render_snapshot(self, snapshot) -> CameraData:
        """Render an immutable integration-state snapshot on the calling thread."""
        if snapshot is None:
            raise ValueError("camera snapshot is required")
        with self._render_lock:
            if self._snapshot_data is None:
                self._snapshot_data = self._mujoco.MjData(self._model)
            self._mujoco.mj_setState(
                self._model,
                self._snapshot_data,
                np.asarray(snapshot, dtype=np.float64),
                self._mujoco.mjtState.mjSTATE_INTEGRATION,
            )
            self._mujoco.mj_forward(self._model, self._snapshot_data)
            return self._render_locked(self._snapshot_data)

    def _render_locked(self, data) -> CameraData:
        self._ensure_renderer()
        self._renderer.update_scene(data, camera=self._camera_name)
        rgb = self._renderer.render().copy()

        if self._config.render_depth:
            self._renderer.enable_depth_rendering()
            try:
                depth_raw = self._renderer.render().copy()
            finally:
                self._renderer.disable_depth_rendering()
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
        """Clamp metric depth returned by mujoco.Renderer.

        Renderer already reverses the OpenGL projection. A near-only frame is
        still metric; its value range cannot identify a normalized Z buffer.
        """
        depth = np.asarray(depth_raw, dtype=np.float32)
        depth_meters = np.nan_to_num(
            depth,
            nan=far,
            posinf=far,
            neginf=0.0,
        )
        return np.clip(depth_meters, 0.0, far).astype(np.float32)

    def get_rgb(self, data) -> np.ndarray:
        """Render RGB only, skip depth."""
        with self._render_lock:
            self._ensure_renderer()
            self._renderer.update_scene(data, camera=self._camera_name)
            return self._renderer.render().copy()

    def close(self) -> None:
        """Release renderer resources."""
        with self._render_lock:
            self._close_renderer()
            self._snapshot_data = None

    def _ensure_renderer(self) -> None:
        """Create the renderer in the calling thread.

        MuJoCo EGL contexts are thread-affine on Linux, so the offscreen renderer
        must be created in the same thread that later calls ``render()``.
        """
        thread_id = threading.get_ident()
        if self._renderer is not None and self._renderer_thread_id == thread_id:
            return

        self._close_renderer()
        self._renderer = self._mujoco.Renderer(
            self._model, self._config.height, self._config.width
        )
        self._renderer_thread_id = thread_id

    def _close_renderer(self) -> None:
        if self._renderer is not None:
            self._renderer.close()
            self._renderer = None
        self._renderer_thread_id = None

    @property
    def config(self) -> CameraConfig:
        return self._config

    @property
    def cam_id(self) -> int:
        return self._cam_id
