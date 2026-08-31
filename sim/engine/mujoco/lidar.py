"""MuJoCo LiDAR wrapper.

Production validation requires product LiDAR backends:

* ``mujoco_lidar``: MuJoCo-LiDAR package wrapper, including
  Livox MID-360 scan-pattern support and optional CPU/Taichi/JAX/Warp backends.
* ``ray_caster_lidar``: native ``mujoco.sensor.ray_caster_lidar`` plugin.

Initialization fails when neither supported backend is available.
"""

from __future__ import annotations

import importlib.metadata
from pathlib import Path
from typing import Any

import numpy as np

from sim.engine.core.sensor import LidarConfig

_PRODUCT_BACKENDS = {"mujoco_lidar", "ray_caster_lidar"}
ROBOT_COLLISION_GEOM_GROUP = 3
ROBOT_VISUAL_GEOM_GROUP = 5
_REPO_ROOT = Path(__file__).resolve().parents[3]
_REPO_SOURCE_ROOT = _REPO_ROOT / "src"
_MUJOCO_LIDAR_VERSION = "0.3.3"
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
}


def _load_official_mujoco_lidar_wrapper() -> type:
    """Load the pinned distribution without accepting a source-tree shadow."""

    try:
        installed_version = importlib.metadata.version("mujoco-lidar")
    except importlib.metadata.PackageNotFoundError as exc:
        raise RuntimeError("mujoco-lidar 0.3.3 is required; install the sim-mujoco extra") from exc
    if installed_version != _MUJOCO_LIDAR_VERSION:
        raise RuntimeError(f"unsupported mujoco-lidar version: {installed_version}; expected {_MUJOCO_LIDAR_VERSION}")

    import mujoco_lidar

    origin = Path(str(getattr(mujoco_lidar, "__file__", ""))).resolve()
    if origin == _REPO_SOURCE_ROOT or _REPO_SOURCE_ROOT in origin.parents:
        raise RuntimeError(
            "mujoco_lidar resolved inside LingTu src; install the pinned distribution instead of a source shadow"
        )
    return mujoco_lidar.MjLidarWrapper


class MuJoCoLidar:
    """MuJoCo LiDAR sensor wrapper returning world-frame XYZI point clouds."""

    def __init__(self, model, data, config: LidarConfig) -> None:
        self._model = model
        self._data = data
        self._config = config
        self._rng = np.random.default_rng(0)
        self._ray_angles: np.ndarray | None = None
        self._ray_cursor = 0
        self._body_id = 0
        self._exclude_body_id = 0
        self._mujoco_lidar: Any | None = None
        self._mujoco_lidar_impl = ""
        self._plugin_reader: Any | None = None
        self._backend = ""
        self._backend_error = ""
        self._self_geom_count = 0

        self._geomgroup = self._geomgroup_from_config(config)
        requested = self._normalize_backend(config.backend)
        init_errors: list[str] = []

        if requested == "auto":
            for candidate in ("mujoco_lidar", "ray_caster_lidar"):
                if self._try_init(candidate, init_errors):
                    break
        else:
            self._try_init(requested, init_errors)

        if not self._backend:
            detail = "; ".join(init_errors) or f"unsupported backend: {requested}"
            raise RuntimeError(f"MuJoCo LiDAR backend initialization failed: {detail}")

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
            else:
                raise ValueError(f"unsupported backend: {backend}")
        except Exception as exc:
            msg = f"{backend}: {type(exc).__name__}: {exc}"
            init_errors.append(msg)
            self._backend_error = msg
            return False
        self._backend = backend
        self._backend_error = ""
        return True

    def _resolve_body_id(self) -> int:
        import mujoco

        body_id = mujoco.mj_name2id(self._model, mujoco.mjtObj.mjOBJ_BODY, self._config.body_name)
        if body_id < 0:
            print(f'[MuJoCoLidar] body "{self._config.body_name}" not found, using world origin')
            return 0
        return int(body_id)

    def _resolve_exclude_body_id(
        self,
        config: LidarConfig | None = None,
    ) -> int:
        import mujoco

        resolved_config = config or self._config
        configured = str(getattr(resolved_config, "exclude_body_name", None) or "").strip()
        if not configured:
            return int(self._body_id)
        body_id = mujoco.mj_name2id(
            self._model,
            mujoco.mjtObj.mjOBJ_BODY,
            configured,
        )
        if body_id < 0:
            raise ValueError(f"MuJoCo LiDAR exclusion body not found: {configured}")
        return int(body_id)

    def _resolve_site_id(self) -> int:
        import mujoco

        site_id = mujoco.mj_name2id(self._model, mujoco.mjtObj.mjOBJ_SITE, self._config.site_name)
        if site_id < 0:
            raise ValueError(f"MuJoCo LiDAR site not found: {self._config.site_name}")
        return int(site_id)

    def _exclude_robot_geoms(self) -> tuple[tuple[int, int], ...]:
        """Keep robot collision and visual geometry in separate LiDAR-excluded groups."""

        root_body = int(self._exclude_body_id)
        if root_body <= 0:
            return ()

        excluded = 0
        changed: list[tuple[int, int]] = []
        for geom_id in range(int(self._model.ngeom)):
            body_id = int(self._model.geom_bodyid[geom_id])
            while body_id > 0 and body_id != root_body:
                body_id = int(self._model.body_parentid[body_id])
            if body_id != root_body:
                continue
            previous_group = int(self._model.geom_group[geom_id])
            collidable = bool(
                int(self._model.geom_contype[geom_id])
                or int(self._model.geom_conaffinity[geom_id])
            )
            target_group = (
                ROBOT_COLLISION_GEOM_GROUP if collidable else ROBOT_VISUAL_GEOM_GROUP
            )
            if previous_group != target_group:
                changed.append((geom_id, previous_group))
            self._model.geom_group[geom_id] = target_group
            excluded += 1

        # MuJoCo ray queries filter by geom group, while bodyexclude only skips
        # one body. Both robot groups must stay outside the LiDAR mask.
        self._geomgroup[ROBOT_COLLISION_GEOM_GROUP] = 0
        self._geomgroup[ROBOT_VISUAL_GEOM_GROUP] = 0
        self._self_geom_count = excluded
        return tuple(changed)

    def _restore_robot_geom_groups(
        self,
        changed: tuple[tuple[int, int], ...],
    ) -> None:
        for geom_id, group in changed:
            self._model.geom_group[geom_id] = group
        self._self_geom_count = 0

    def _init_mujoco_lidar(self) -> None:
        """Initialize discoverse-dev/MuJoCo-LiDAR wrapper."""

        MjLidarWrapper = _load_official_mujoco_lidar_wrapper()
        self._resolve_site_id()
        self._body_id = self._resolve_body_id()
        self._exclude_body_id = self._resolve_exclude_body_id()
        self._ray_angles = self._load_configured_mid360_angles()
        self._ray_cursor = 0
        impl = str(self._config.mujoco_lidar_backend or "cpu").strip().lower()
        self._mujoco_lidar_impl = impl
        changed = self._exclude_robot_geoms()
        try:
            wrapper = MjLidarWrapper(
                self._model,
                site_name=self._config.site_name,
                backend=impl,
                cutoff_dist=float(self._config.range_max),
                args={
                    "geomgroup": self._geomgroup,
                    "bodyexclude": self._exclude_body_id,
                },
            )
        except BaseException:
            self._restore_robot_geom_groups(changed or ())
            raise
        self._mujoco_lidar = wrapper

    def _init_ray_caster_plugin(self) -> None:
        """Initialize native mujoco.sensor.ray_caster_lidar plugin reader."""

        import mujoco

        from sim.sensors.livox_mid360 import read_plugin_lidar

        sensor_id = mujoco.mj_name2id(self._model, mujoco.mjtObj.mjOBJ_SENSOR, self._config.sensor_name)
        if sensor_id < 0:
            raise ValueError(f"ray_caster_lidar sensor not found: {self._config.sensor_name}")
        if int(self._model.sensor_plugin[sensor_id]) < 0:
            raise RuntimeError(f"MuJoCo sensor {self._config.sensor_name!r} is not backed by a loaded plugin")
        self._plugin_reader = read_plugin_lidar

    @staticmethod
    def _load_scan_mode_angles(path: str | None) -> np.ndarray | None:
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
            raise ValueError(f"MID-360 scan-mode file must contain Nx2 theta/phi angles: {scan_path}")
        if len(angles) == 0:
            raise ValueError(f"MID-360 scan-mode file is empty: {scan_path}")
        return angles

    def _load_configured_mid360_angles(self) -> np.ndarray:
        """Load a configured pattern and enforce the product pattern contract."""

        configured = str(self._config.mid360_npy_path or "").strip()
        require_product = bool(self._config.require_product_backend)
        if not configured:
            if require_product:
                raise RuntimeError("product MuJoCo LiDAR requires an explicit canonical MID-360 pattern")
            return self._load_package_mid360_angles()

        pattern_path = Path(configured).expanduser().resolve()
        angles = self._load_scan_mode_angles(str(pattern_path))
        assert angles is not None
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

            site_id = mujoco.mj_name2id(self._model, mujoco.mjtObj.mjOBJ_SITE, self._config.site_name)
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
        angle_noise = float(getattr(self._config, "angle_noise_std_rad", 0.0) or 0.0)
        if (
            getattr(self, "_backend", "") == "ray_caster_lidar"
            and bool(getattr(self._config, "add_noise", False))
            and angle_noise > 0.0
            and len(pts) > 0
        ):
            # The plugin owns ray generation and cannot accept our noisy angle
            # table. Apply the same angular measurement error to returned ray
            # vectors. This preserves range but cannot reproduce edge/occlusion
            # changes that a pre-raycast perturbation would cause.
            plugin_vectors = pts - origin
            plugin_ranges = np.linalg.norm(plugin_vectors, axis=1).astype(np.float32)
            nonzero = plugin_ranges > 1e-9
            if np.any(nonzero):
                angular_error = self._rng.normal(
                    0.0,
                    angle_noise,
                    (int(np.count_nonzero(nonzero)), 2),
                ).astype(np.float32)
                vectors = plugin_vectors[nonzero]
                radii = plugin_ranges[nonzero]
                theta = np.arctan2(vectors[:, 1], vectors[:, 0]) + angular_error[:, 0]
                phi = np.arcsin(np.clip(vectors[:, 2] / radii, -1.0, 1.0)) + angular_error[:, 1]
                phi = np.clip(phi, -0.5 * np.pi, 0.5 * np.pi)
                cos_phi = np.cos(phi)
                plugin_vectors[nonzero] = np.column_stack(
                    [
                        radii * cos_phi * np.cos(theta),
                        radii * cos_phi * np.sin(theta),
                        radii * np.sin(phi),
                    ]
                )
                pts = origin + plugin_vectors
        ray_vectors = pts - origin
        ranges = np.linalg.norm(ray_vectors, axis=1).astype(np.float32)
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
        ray_vectors = ray_vectors[valid]
        ranges = ranges[valid]
        range_noise = float(getattr(self._config, "noise_std", 0.0) or 0.0)
        if bool(getattr(self._config, "add_noise", False)) and range_noise > 0.0:
            near_std = getattr(self._config, "range_noise_near_std_m", None)
            far_std = getattr(self._config, "range_noise_far_std_m", None)
            if near_std is not None and far_std is not None:
                near_range = float(getattr(self._config, "range_noise_near_m", 0.2))
                far_range = max(
                    near_range + 1e-6,
                    float(getattr(self._config, "range_noise_far_m", 10.0)),
                )
                blend = np.clip(
                    (ranges - near_range) / (far_range - near_range),
                    0.0,
                    1.0,
                )
                range_sigmas = (float(near_std) + blend * (float(far_std) - float(near_std))).astype(np.float32)
            else:
                range_sigmas = np.full(len(ranges), range_noise, dtype=np.float32)
            measured_ranges = ranges + self._rng.normal(
                0.0,
                range_sigmas,
                len(ranges),
            ).astype(np.float32)
            measured_valid = (
                np.isfinite(measured_ranges)
                & (measured_ranges >= float(self._config.range_min))
                & (measured_ranges <= float(self._config.range_max))
            )
            if not np.any(measured_valid):
                return np.zeros((0, 4), dtype=np.float32)
            unit_rays = ray_vectors[measured_valid] / ranges[measured_valid, None]
            ranges = measured_ranges[measured_valid]
            pts = origin + unit_rays * ranges[:, None]

        scale = max(float(getattr(self._config, "intensity_range_scale_m", 25.0) or 25.0), 1e-3)
        intensity = float(getattr(self._config, "intensity_base", 180.0) or 180.0) / (1.0 + (ranges / scale) ** 2)
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
        raise RuntimeError(f"unsupported active LiDAR backend: {self._backend}")

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

    def update_data(self, data) -> None:
        """Update MjData reference after simulation reset."""

        self._data = data

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
            "product_backend": self._backend in _PRODUCT_BACKENDS,
            "product_lidar_backend_verified": self._backend in _PRODUCT_BACKENDS,
            "requested_backend": str(self._config.backend),
            "mujoco_lidar_backend": self._mujoco_lidar_impl,
            "require_product_backend": bool(self._config.require_product_backend),
            "site_name": str(self._config.site_name),
            "sensor_name": str(self._config.sensor_name),
            "body_name": str(self._config.body_name),
            "exclude_body_name": str(getattr(self._config, "exclude_body_name", None) or self._config.body_name),
            "pattern_path": str(self._config.mid360_npy_path or ""),
            "samples_per_frame": int(self._config.samples_per_frame),
            "range_min_m": float(self._config.range_min),
            "range_max_m": float(self._config.range_max),
            "noise_std_m": float(self._config.noise_std if self._config.add_noise else 0.0),
            "range_noise_model": (
                "radial_piecewise_gaussian"
                if self._config.add_noise
                and getattr(self._config, "range_noise_near_std_m", None) is not None
                and getattr(self._config, "range_noise_far_std_m", None) is not None
                else "radial_gaussian"
                if self._config.add_noise
                else "disabled"
            ),
            "range_noise_near_std_m": float(
                getattr(self._config, "range_noise_near_std_m", self._config.noise_std)
                if self._config.add_noise
                else 0.0
            ),
            "range_noise_far_std_m": float(
                getattr(self._config, "range_noise_far_std_m", self._config.noise_std)
                if self._config.add_noise
                else 0.0
            ),
            "angle_noise_std_rad": float(
                getattr(self._config, "angle_noise_std_rad", 0.0) if self._config.add_noise else 0.0
            ),
            "angle_noise_model": (
                "post_return_gaussian_approximation"
                if self._config.add_noise and self._backend == "ray_caster_lidar"
                else "pre_raycast_gaussian"
                if self._config.add_noise
                else "disabled"
            ),
            "pixel_dropout_prob": float(getattr(self._config, "pixel_dropout_prob", 0.0)),
            "distance_dropout_prob_at_max": float(getattr(self._config, "distance_dropout_prob_at_max", 0.0)),
            "intensity_model": "distance_proxy_not_material_calibrated",
            "multi_return_model": "single_return",
            "self_occlusion_mode": "robot_group" if self._self_geom_count else "backend_default",
            "self_geom_group": ROBOT_VISUAL_GEOM_GROUP,
            "self_collision_geom_group": ROBOT_COLLISION_GEOM_GROUP,
            "self_geoms": int(self._self_geom_count),
            "fields": ["x", "y", "z", "intensity"],
            "error": self._backend_error,
        }
