"""MuJoCo LiDAR wrapper.

Production validation requires product LiDAR backends:

* ``mujoco_lidar``: MuJoCo-LiDAR package wrapper, including
  Livox MID-360 scan-pattern support and optional CPU/Taichi/JAX/Warp backends.
* ``ray_caster_lidar``: native ``mujoco.sensor.ray_caster_lidar`` plugin.

The legacy local ``mj_multiRay`` implementation remains available only as an
explicit development fallback. Product gates should set
``LidarConfig.require_product_backend`` so fallback cannot pass silently.
"""

from __future__ import annotations

import sys
from pathlib import Path
from typing import Any, Optional

import numpy as np

from sim.engine.core.sensor import LidarConfig

# Add sim/sensors/ to path so livox_mid360 can be imported.
_SIM_SENSORS = Path(__file__).resolve().parents[2] / "sensors"
if str(_SIM_SENSORS) not in sys.path:
    sys.path.insert(0, str(_SIM_SENSORS))

_MUJOCO_LIDAR_PARENT = Path(__file__).resolve().parents[3] / "src" / "drivers" / "sim" / "lidar"
if _MUJOCO_LIDAR_PARENT.is_dir() and str(_MUJOCO_LIDAR_PARENT) not in sys.path:
    sys.path.insert(0, str(_MUJOCO_LIDAR_PARENT))

_PRODUCT_BACKENDS = {"mujoco_lidar", "ray_caster_lidar"}
_BACKEND_ALIASES = {
    "": "auto",
    "auto": "auto",
    "mujoco-lidar": "mujoco_lidar",
    "mujoco_lidar": "mujoco_lidar",
    "mujoco_lidar_package": "mujoco_lidar",
    "mujoco-lidar-package": "mujoco_lidar",
    "ray_caster": "ray_caster_lidar",
    "ray-caster": "ray_caster_lidar",
    "ray_caster_lidar": "ray_caster_lidar",
    "ray-caster-lidar": "ray_caster_lidar",
    "plugin": "ray_caster_lidar",
    "mj_multiray": "mj_multiray",
    "mj-multiray": "mj_multiray",
    "mj_multiRay": "mj_multiray",
    "multiray": "mj_multiray",
    "fallback": "mj_multiray",
}


class MuJoCoLidar:
    """MuJoCo LiDAR sensor wrapper returning world-frame XYZI point clouds."""

    def __init__(self, model, data, config: LidarConfig) -> None:
        self._model = model
        self._data = data
        self._config = config
        self._rng = np.random.default_rng(0)
        self._ray_angles: np.ndarray | None = None
        self._ray_cursor = 0
        self._ray_dirs_local: np.ndarray | None = None
        self._frame_idx = 0
        self._body_id = 0
        self._mujoco_lidar: Any | None = None
        self._mujoco_lidar_impl = ""
        self._plugin_reader: Any | None = None
        self._backend = ""
        self._backend_error = ""
        self._fallback_used = False
        self._product_backend = False

        self._geomgroup = self._geomgroup_from_config(config)
        requested = self._normalize_backend(config.backend)
        init_errors: list[str] = []

        if requested == "auto":
            for candidate in ("mujoco_lidar", "ray_caster_lidar", "mj_multiray"):
                if candidate == "mj_multiray" and not bool(config.allow_legacy_fallback):
                    continue
                if self._try_init(candidate, init_errors):
                    break
        else:
            if not self._try_init(requested, init_errors):
                if bool(config.allow_legacy_fallback) and requested != "mj_multiray":
                    self._try_init("mj_multiray", init_errors)

        if not self._backend:
            detail = "; ".join(init_errors) or f"unsupported backend: {requested}"
            raise RuntimeError(f"MuJoCo LiDAR backend initialization failed: {detail}")

        if bool(config.require_product_backend) and self._backend not in _PRODUCT_BACKENDS:
            detail = "; ".join(init_errors) or self._backend_error or "legacy fallback selected"
            raise RuntimeError(
                "Product MuJoCo LiDAR backend required; "
                f"selected {self._backend!r}. Details: {detail}"
            )

        print(
            "[MuJoCoLidar] Initialized: "
            f"backend={self._backend}, body={config.body_name}, "
            f"site={config.site_name}, sensor={config.sensor_name}"
        )

    @staticmethod
    def _normalize_backend(name: str | None) -> str:
        key = str(name or "auto").strip()
        return _BACKEND_ALIASES.get(key, _BACKEND_ALIASES.get(key.lower(), key.lower()))

    @staticmethod
    def _geomgroup_from_config(config: LidarConfig) -> np.ndarray:
        geomgroup = np.zeros(6, dtype=np.uint8)
        # Existing LingTu gates expect default scene geoms plus explicit env geoms.
        geomgroup[0] = 1
        geomgroup[1] = 1
        try:
            idx = int(config.geom_group)
        except (TypeError, ValueError):
            idx = 1
        if 0 <= idx < len(geomgroup):
            geomgroup[idx] = 1
        return geomgroup

    def _try_init(self, backend: str, init_errors: list[str]) -> bool:
        try:
            if backend == "mujoco_lidar":
                self._init_mujoco_lidar()
            elif backend == "ray_caster_lidar":
                self._init_ray_caster_plugin()
            elif backend == "mj_multiray":
                self._init_fallback(self._model, self._data, self._config)
            else:
                raise ValueError(f"unsupported backend: {backend}")
        except Exception as exc:
            msg = f"{backend}: {type(exc).__name__}: {exc}"
            init_errors.append(msg)
            self._backend_error = msg
            return False
        self._backend = backend
        self._product_backend = backend in _PRODUCT_BACKENDS
        self._fallback_used = backend == "mj_multiray"
        self._backend_error = ""
        return True

    def _resolve_body_id(self) -> int:
        import mujoco

        body_id = mujoco.mj_name2id(
            self._model, mujoco.mjtObj.mjOBJ_BODY, self._config.body_name
        )
        if body_id < 0:
            print(
                f'[MuJoCoLidar] body "{self._config.body_name}" not found, '
                "using world origin"
            )
            return 0
        return int(body_id)

    def _resolve_site_id(self) -> int:
        import mujoco

        site_id = mujoco.mj_name2id(
            self._model, mujoco.mjtObj.mjOBJ_SITE, self._config.site_name
        )
        if site_id < 0:
            raise ValueError(f"MuJoCo LiDAR site not found: {self._config.site_name}")
        return int(site_id)

    def _init_mujoco_lidar(self) -> None:
        """Initialize discoverse-dev/MuJoCo-LiDAR wrapper."""

        self._resolve_site_id()
        self._body_id = self._resolve_body_id()
        from mujoco_lidar import MjLidarWrapper

        self._ray_angles = self._load_scan_mode_angles(self._config.mid360_npy_path)
        if self._ray_angles is None:
            self._ray_angles = self._load_package_mid360_angles()
        self._ray_cursor = 0
        impl = str(self._config.mujoco_lidar_backend or "cpu").strip().lower()
        self._mujoco_lidar_impl = impl
        self._mujoco_lidar = MjLidarWrapper(
            self._model,
            site_name=self._config.site_name,
            backend=impl,
            cutoff_dist=float(self._config.range_max),
            args={
                "geomgroup": self._geomgroup,
                "bodyexclude": self._body_id,
            },
        )

    def _init_ray_caster_plugin(self) -> None:
        """Initialize native mujoco.sensor.ray_caster_lidar plugin reader."""

        import mujoco
        from livox_mid360 import read_plugin_lidar

        sensor_id = mujoco.mj_name2id(
            self._model, mujoco.mjtObj.mjOBJ_SENSOR, self._config.sensor_name
        )
        if sensor_id < 0:
            raise ValueError(f"ray_caster_lidar sensor not found: {self._config.sensor_name}")
        if int(self._model.sensor_plugin[sensor_id]) < 0:
            raise RuntimeError(
                f"MuJoCo sensor {self._config.sensor_name!r} is not backed by a loaded plugin"
            )
        self._plugin_reader = read_plugin_lidar

    def _init_fallback(self, model, data, config: LidarConfig) -> None:
        """Initialize legacy local ``mj_multiRay`` backend."""

        self._body_id = self._resolve_body_id()
        self._rng = np.random.default_rng(0)
        self._ray_angles = self._load_scan_mode_angles(config.mid360_npy_path)
        if self._ray_angles is None:
            self._ray_dirs_local = self._build_golden_spiral(config.n_rays)
        else:
            self._ray_cursor = 0
            self._ray_dirs_local = None
        self._frame_idx = 0

    @staticmethod
    def _build_golden_spiral(
        n: int,
        vfov_min: float = np.deg2rad(-7.0),
        vfov_max: float = np.deg2rad(52.0),
    ) -> np.ndarray:
        """Build golden-angle spiral ray directions for legacy fallback only."""

        golden_ang = np.pi * (3 - np.sqrt(5))
        i = np.arange(n, dtype=np.float64)
        ha = (i * golden_ang) % (2 * np.pi)
        va = vfov_min + i / n * (vfov_max - vfov_min)
        cv = np.cos(va)
        return np.column_stack([cv * np.cos(ha), cv * np.sin(ha), np.sin(va)])

    @staticmethod
    def _load_scan_mode_angles(path: Optional[str]) -> Optional[np.ndarray]:
        """Load Livox scan-mode angles as ``[theta, phi]`` radians.

        ``.npy`` files are expected to already contain the same two-column
        representation used by MuJoCo-LiDAR. Official Livox CSV files use
        ``Time/s,Azimuth/deg,Zenith/deg``; convert zenith to elevation phi.
        """

        if not path:
            return None
        scan_path = Path(path).expanduser()
        if not scan_path.exists():
            raise FileNotFoundError(f"MID-360 scan-mode file not found: {scan_path}")
        if scan_path.suffix.lower() == ".npy":
            angles = np.load(scan_path)
        else:
            csv_angles = np.loadtxt(
                scan_path,
                delimiter=",",
                skiprows=1,
                usecols=(1, 2),
                dtype=np.float32,
            )
            theta = np.deg2rad(csv_angles[:, 0])
            phi = np.deg2rad(90.0 - csv_angles[:, 1])
            angles = np.column_stack([theta, phi])
        angles = np.asarray(angles, dtype=np.float32)
        if angles.ndim != 2 or angles.shape[1] != 2:
            raise ValueError(
                f"MID-360 scan-mode file must contain Nx2 theta/phi angles: {scan_path}"
            )
        if len(angles) == 0:
            raise ValueError(f"MID-360 scan-mode file is empty: {scan_path}")
        return angles

    @staticmethod
    def _load_package_mid360_angles() -> np.ndarray:
        """Load MuJoCo-LiDAR's packaged MID-360 scan pattern."""

        try:
            from mujoco_lidar.scan_gen import LivoxGenerator

            generator = LivoxGenerator("mid360")
            return np.asarray(generator.ray_angles, dtype=np.float32)
        except Exception as exc:
            raise RuntimeError(
                "MuJoCo-LiDAR backend requires a MID-360 scan pattern. "
                "Set LidarConfig.mid360_npy_path or install a package that ships "
                "mujoco_lidar.scan_gen.LivoxGenerator('mid360')."
            ) from exc

    def _next_pattern_angles(self, sample_count: int | None = None) -> tuple[np.ndarray, np.ndarray]:
        assert self._ray_angles is not None
        samples = max(
            1,
            int(self._config.samples_per_frame if sample_count is None else sample_count),
        )
        n_angles = len(self._ray_angles)
        start = int(getattr(self, "_ray_cursor", 0))
        indices = (np.arange(samples, dtype=np.int64) + start) % n_angles
        angles = self._ray_angles[indices]
        self._ray_cursor = (start + samples) % n_angles
        angle_noise = float(getattr(self._config, "angle_noise_std_rad", 0.0) or 0.0)
        if bool(getattr(self._config, "add_noise", False)) and angle_noise > 0.0:
            angles = angles.copy()
            angles += self._rng.normal(0.0, angle_noise, angles.shape).astype(np.float32)
            angles[:, 0] = np.mod(angles[:, 0], np.float32(2.0 * np.pi))
        return (
            angles[:, 0].astype(np.float32, copy=False),
            angles[:, 1].astype(np.float32, copy=False),
        )

    def _next_pattern_dirs_local(self, sample_count: int | None = None) -> np.ndarray:
        theta, phi = self._next_pattern_angles(sample_count)
        theta64 = theta.astype(np.float64, copy=False)
        phi64 = phi.astype(np.float64, copy=False)
        cp = np.cos(phi64)
        return np.column_stack([cp * np.cos(theta64), cp * np.sin(theta64), np.sin(phi64)])

    def scan(self, sample_count: int | None = None) -> np.ndarray:
        """Perform one LiDAR scan.

        Returns:
            (N, 4) float32 XYZI point cloud in world frame.
        """

        pts_xyz = self.scan_xyz(sample_count)
        if len(pts_xyz) == 0:
            return np.zeros((0, 4), dtype=np.float32)
        return self._points_with_return_model(pts_xyz)

    def _sensor_origin(self) -> np.ndarray:
        if self._mujoco_lidar is not None:
            return np.asarray(self._mujoco_lidar.sensor_position, dtype=np.float32)
        try:
            import mujoco

            site_id = mujoco.mj_name2id(
                self._model, mujoco.mjtObj.mjOBJ_SITE, self._config.site_name
            )
            if site_id >= 0:
                return np.asarray(self._data.site_xpos[site_id], dtype=np.float32)
        except Exception:
            pass
        try:
            return np.asarray(self._data.xpos[getattr(self, "_body_id", 0)], dtype=np.float32)
        except Exception:
            return np.zeros(3, dtype=np.float32)

    def _points_with_return_model(self, pts_xyz: np.ndarray) -> np.ndarray:
        pts = np.asarray(pts_xyz, dtype=np.float32)
        origin = self._sensor_origin()
        ranges = np.linalg.norm(pts - origin, axis=1).astype(np.float32)
        valid = (
            np.isfinite(pts).all(axis=1)
            & np.isfinite(ranges)
            & (ranges >= float(self._config.range_min))
            & (ranges <= float(self._config.range_max))
        )
        dropout = float(getattr(self._config, "pixel_dropout_prob", 0.0) or 0.0)
        if dropout > 0.0:
            valid &= self._rng.random(len(pts)) >= min(max(dropout, 0.0), 1.0)
        max_dropout = float(getattr(self._config, "distance_dropout_prob_at_max", 0.0) or 0.0)
        if max_dropout > 0.0:
            span = max(float(self._config.range_max) - float(self._config.range_min), 1e-3)
            extra = np.clip((ranges - float(self._config.range_min)) / span, 0.0, 1.0)
            valid &= self._rng.random(len(pts)) >= extra * min(max(max_dropout, 0.0), 1.0)
        if not np.any(valid):
            return np.zeros((0, 4), dtype=np.float32)

        pts = pts[valid]
        ranges = ranges[valid]
        scale = max(float(getattr(self._config, "intensity_range_scale_m", 25.0) or 25.0), 1e-3)
        intensity = float(getattr(self._config, "intensity_base", 180.0) or 180.0) / (
            1.0 + (ranges / scale) ** 2
        )
        noise = float(getattr(self._config, "intensity_noise_std", 0.0) or 0.0)
        if bool(getattr(self._config, "add_noise", False)) and noise > 0.0:
            intensity += self._rng.normal(0.0, noise, len(intensity)).astype(np.float32)
        intensity = np.clip(
            intensity,
            float(getattr(self._config, "intensity_min", 1.0) or 1.0),
            float(getattr(self._config, "intensity_max", 255.0) or 255.0),
        ).astype(np.float32)
        return np.column_stack([pts, intensity]).astype(np.float32, copy=False)

    def scan_xyz(self, sample_count: int | None = None) -> np.ndarray:
        """Return XYZ-only point cloud in world frame."""

        if self._backend == "mujoco_lidar":
            return self._scan_mujoco_lidar(sample_count)
        if self._backend == "ray_caster_lidar":
            return self._scan_plugin()
        return self._scan_fallback(sample_count)

    def _scan_mujoco_lidar(self, sample_count: int | None = None) -> np.ndarray:
        if self._mujoco_lidar is None:
            return np.zeros((0, 3), dtype=np.float32)
        theta, phi = self._next_pattern_angles(sample_count)
        distances = np.asarray(
            self._mujoco_lidar.trace_rays(
                self._data,
                theta,
                phi,
                site_name=self._config.site_name,
            ),
            dtype=np.float32,
        )
        hit_local = self._mujoco_lidar.get_hit_points()
        if hit_local is None:
            return np.zeros((0, 3), dtype=np.float32)
        hit_local = np.asarray(hit_local, dtype=np.float32)
        if hit_local.ndim != 2 or hit_local.shape[1] != 3 or len(hit_local) == 0:
            return np.zeros((0, 3), dtype=np.float32)
        valid = (
            np.isfinite(hit_local).all(axis=1)
            & np.isfinite(distances)
            & (distances >= float(self._config.range_min))
            & (distances <= float(self._config.range_max))
        )
        if not np.any(valid):
            return np.zeros((0, 3), dtype=np.float32)
        rot = np.asarray(self._mujoco_lidar.sensor_rotation, dtype=np.float32)
        pos = np.asarray(self._mujoco_lidar.sensor_position, dtype=np.float32)
        pts = pos + hit_local[valid] @ rot.T
        if self._config.add_noise:
            pts += self._rng.normal(0, self._config.noise_std, pts.shape).astype(np.float32)
        return pts.astype(np.float32, copy=False)

    def _scan_plugin(self) -> np.ndarray:
        if self._plugin_reader is None:
            return np.zeros((0, 3), dtype=np.float32)
        pts = self._plugin_reader(self._model, self._data, self._config.sensor_name)
        if pts is None or len(pts) == 0:
            return np.zeros((0, 3), dtype=np.float32)
        pts = np.asarray(pts, dtype=np.float32)
        valid = np.isfinite(pts).all(axis=1)
        return pts[valid]

    def _scan_fallback(self, sample_count: int | None = None) -> np.ndarray:
        """Legacy ``mj_multiRay`` scan used only when fallback is explicit/allowed."""

        import mujoco

        body_id = getattr(self, "_body_id", 0)
        pos = self._data.xpos[body_id].copy()
        rmat = self._data.xmat[body_id].reshape(3, 3).copy()

        if self._ray_angles is not None:
            dirs_local = self._next_pattern_dirs_local(sample_count)
        else:
            ang = self._frame_idx * 0.628
            self._frame_idx += 1
            c, s = np.cos(ang), np.sin(ang)
            Rz = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]], dtype=np.float64)
            dirs_local = self._ray_dirs_local @ Rz.T
        dirs_world = dirs_local @ rmat.T

        n_rays = len(dirs_world)
        dist_out = np.full(n_rays, -1.0, dtype=np.float64)
        geomid_out = np.full(n_rays, -1, dtype=np.int32)

        mujoco.mj_multiRay(
            self._model,
            self._data,
            pos,
            dirs_world.flatten(),
            self._geomgroup,
            1,
            body_id,
            geomid_out,
            dist_out,
            None,
            n_rays,
            self._config.range_max,
        )

        mask = dist_out > self._config.range_min
        if not mask.any():
            return np.zeros((0, 3), dtype=np.float32)

        pts = (pos + dirs_world[mask] * dist_out[mask, None]).astype(np.float32)
        if self._config.add_noise:
            pts += self._rng.normal(0, self._config.noise_std, pts.shape).astype(np.float32)
        return pts

    def update_data(self, data) -> None:
        """Update MjData reference after simulation reset."""

        self._data = data
        if hasattr(self, "_frame_idx"):
            self._frame_idx = 0

    @property
    def config(self) -> LidarConfig:
        return self._config

    @property
    def backend(self) -> str:
        return self._backend

    def backend_report(self) -> dict[str, Any]:
        """Return a JSON-ready LiDAR backend contract report."""

        return {
            "backend": self._backend,
            "product_backend": bool(self._product_backend),
            "product_lidar_backend_verified": bool(
                self._product_backend and not self._fallback_used
            ),
            "fallback_used": bool(self._fallback_used),
            "requested_backend": str(self._config.backend),
            "mujoco_lidar_backend": self._mujoco_lidar_impl,
            "require_product_backend": bool(self._config.require_product_backend),
            "allow_legacy_fallback": bool(self._config.allow_legacy_fallback),
            "site_name": str(self._config.site_name),
            "sensor_name": str(self._config.sensor_name),
            "body_name": str(self._config.body_name),
            "pattern_path": str(self._config.mid360_npy_path or ""),
            "samples_per_frame": int(self._config.samples_per_frame),
            "fallback_n_rays": int(self._config.n_rays),
            "range_min_m": float(self._config.range_min),
            "range_max_m": float(self._config.range_max),
            "noise_std_m": float(self._config.noise_std if self._config.add_noise else 0.0),
            "angle_noise_std_rad": float(
                getattr(self._config, "angle_noise_std_rad", 0.0)
                if self._config.add_noise
                else 0.0
            ),
            "pixel_dropout_prob": float(getattr(self._config, "pixel_dropout_prob", 0.0)),
            "distance_dropout_prob_at_max": float(
                getattr(self._config, "distance_dropout_prob_at_max", 0.0)
            ),
            "self_occlusion_mode": "scene_geoms_with_sensor_body_excluded",
            "fields": ["x", "y", "z", "intensity"],
            "error": self._backend_error,
        }
